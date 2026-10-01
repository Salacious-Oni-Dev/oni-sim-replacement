// Standalone tests of the sim's kernels, starting with sim/gas_mixture.h, plus a perf pass
// measuring dirty/sleep mixing against a naive baseline on a 40k-tile grid. Mirrors bench.cpp's
// pattern: links the real kernels directly (no DLL load, no sim_initialize). A synthetic
// ElementTable stands in for the real game's element blob so this runs with zero dependency
// on a live install or a captured corpus.
//
// `--perf` runs the benchmark instead of the correctness scenarios (kept
// separate: the correctness scenarios are a handful of ticks on 2-3 cells and finish
// instantly, the perf run is thousands of ticks on 40,000 cells and is meant to be timed,
// not mixed into the same pass).
//
// Exit code 0 on all PASS (correctness mode) or always 0 (perf mode, prints numbers only —
// there is no pass/fail threshold for a timing: measure first, decide targets after).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <type_traits>
#include <vector>

#include "goldens.h"

#include "../../abi/sim_abi_ext.h"
#include "../../sim/effervescence.h"
#include "../../sim/gas_mixture.h"
#include "../../sim/gas_rooms.h"
#include "../../sim/physics.h"
#include "../../sim/projection.h"
#include "../../sim/fields.h"
#include "../../sim/radiation.h"
#include "../../sim/raycast.h"
#include "../../sim/textures.h"
#include "../../sim/world.h"

using namespace oni_sim;
using namespace oni_sim::gas;

namespace {

int g_fail_count = 0;
// Arms that RAN -- see goldens.h. Asserted against `gastest.checks`.
long g_check_count = 0;

void Check(bool cond, const char* what) {
  ++g_check_count;
  if (cond) {
    printf("  PASS: %s\n", what);
  } else {
    printf("  FAIL: %s\n", what);
    ++g_fail_count;
  }
}

// Builds a 3-element synthetic table (vacuum, O2, CO2) in the wire format
// ElementTable::Load expects: int32 count, then that many Element records.
ElementTable MakeTestTable() {
  std::vector<Element> els(3);
  memset(els.data(), 0, els.size() * sizeof(Element));
  els[0].id = 0;                    // vacuum-ish; molarMass 0 so it never contributes moles
  els[1].id = 1;
  els[1].molarMass = 0.032f;        // O2, kg/mol
  els[2].id = 2;
  els[2].molarMass = 0.044f;        // CO2, kg/mol

  std::vector<uint8_t> blob(4 + els.size() * sizeof(Element));
  const int32_t n = static_cast<int32_t>(els.size());
  memcpy(blob.data(), &n, 4);
  memcpy(blob.data() + 4, els.data(), els.size() * sizeof(Element));

  ElementTable table;
  if (!table.Load(blob.data(), blob.size())) {
    printf("  FAIL: synthetic ElementTable failed to load\n");
    ++g_fail_count;
  }
  return table;
}

constexpr uint16_t kVacuum = 0;
constexpr uint16_t kO2 = 1;
constexpr uint16_t kCO2 = 2;

// ---------------------------------------------------------------- projection rules, count > 1
//
// `sim/projection.h`'s three per-cell rules -- `DominantElement`, `SolidFraction`,
// `MeanTemperature` -- all take `(phases, count)` and all loop correctly for count > 1, but
// `project_cell` still hardcodes `const int count = 1`, so NOTHING in this repository has ever
// executed their general path. They have been written-and-never-run since the volume-fractions
// work, which is the state a rule is in right before it is trusted for the first time.
//
// Pressure-aware phase change is what makes them reachable: a cell holding gaseous and liquid
// CO2 at once. These arms exercise the general path directly, ahead of anything feeding it, so the
// rules are proven before the feature that depends on them exists rather than after.
//
// A four-element table, because these rules need what `MakeTestTable` deliberately does not
// carry: a real `state` (so `IsSolid` can answer) and a real `specificHeatCapacity` (so the
// heat-capacity weighting has something to weigh). Vacuum, gaseous CO2, liquid CO2 and a
// solid, with the two CO2 phases given specific heats that differ by more than 2x -- which is
// what real CO2 does, and the reason mass weighting stopped being good enough.
ElementTable MakeProjectionTable() {
  std::vector<Element> els(4);
  memset(els.data(), 0, els.size() * sizeof(Element));
  els[0].id = 0;  els[0].state = kStateVacuum;
  els[1].id = 1;  els[1].state = kStateGas;     // gaseous CO2
  els[1].molarMass = 0.044f;  els[1].specificHeatCapacity = 0.846f;
  els[2].id = 2;  els[2].state = kStateLiquid;  // liquid CO2
  els[2].molarMass = 0.044f;  els[2].specificHeatCapacity = 2.000f;
  els[3].id = 3;  els[3].state = kStateSolid;   // any solid
  els[3].molarMass = 0.100f;  els[3].specificHeatCapacity = 1.000f;

  std::vector<uint8_t> blob(4 + els.size() * sizeof(Element));
  const int32_t n = static_cast<int32_t>(els.size());
  memcpy(blob.data(), &n, 4);
  memcpy(blob.data() + 4, els.data(), els.size() * sizeof(Element));

  ElementTable table;
  if (!table.Load(blob.data(), blob.size())) {
    printf("  FAIL: synthetic projection ElementTable failed to load\n");
    ++g_fail_count;
  }
  return table;
}

// ===================================================================================
// The cell condensation rule (`CondensationAllowed`, sim/physics.h).
//
// Stationeers' gas condensation test as a gate on Klei's low-temperature branch. The
// arms below pin four things the implementation could get wrong quietly:
//
//   1. An element nobody described keeps Klei's threshold EXACTLY. This is what makes the
//      823-scenario byte-identity gate safe by construction, so it is asserted, not assumed.
//   2. Pressure under the floor refuses -- the Mars defect, and the whole point of the change.
//   3. A curve-carrying LIQUID still freezes. OniFramework pushes one MaterialProperties to
//      both halves of a family, so the liquid carries its gas's curve while `CanCondense`
//      reads false for every liquid in Stationeers' table. Keying the rule on "has a curve"
//      would have refused freezing outright, and nothing else in the suite would have noticed.
//   4. The clamp stays. Bypassing it would only be
//      safe if the clamp is provably unreachable below critical and protective above it.
//
// A six-element table: the two CO2 phases with real Stationeers curve constants, a nitrogen
// liquid/solid pair for the freezing arm, and helium -- whose `MinLiquidPressure` really is
// 0.0 Pa, so only the `CanCondense` table stops it, which is why that table cannot be inferred.
// `liquid_co2_freezes` gives the liquid CO2 member a low-temperature branch it does not
// otherwise have. Defaulted off so every arm written before the boil rule sees the same table
// it always did; the one arm that needs it is asserting that a REFUSED BOIL falls through to
// the freeze branch rather than stranding the cell.
ElementTable MakeCondensationTable(bool liquid_co2_freezes = false) {
  std::vector<Element> els(6);
  memset(els.data(), 0, els.size() * sizeof(Element));
  els[0].id = 0;  els[0].state = kStateVacuum;
  // Gaseous CO2. `lowTemp` is ONI's own one-atmosphere condensation point, and the
  // `lowTempTransitionIdx` makes this a real gas -> liquid branch.
  els[1].id = 1;  els[1].state = kStateGas;     els[1].molarMass = 44.0f;
  els[1].specificHeatCapacity = 0.846f;  els[1].lowTemp = 194.65f;
  els[1].lowTempTransitionIdx = 2;       els[1].highTempTransitionIdx = kNoElement;
  els[2].id = 2;  els[2].state = kStateLiquid;  els[2].molarMass = 44.0f;
  els[2].specificHeatCapacity = 2.000f;  els[2].highTemp = 194.65f;
  els[2].highTempTransitionIdx = 1;      els[2].lowTempTransitionIdx = kNoElement;
  if (liquid_co2_freezes) {
    // ONI's own arrangement for this family: liquid CO2 freezes to the solid a little under
    // where Klei has it boiling, because Klei's 194.65 K is a one-atmosphere boiling point for
    // a liquid that cannot exist at one atmosphere at all.
    els[2].lowTemp = 216.15f;
    els[2].lowTempTransitionIdx = 4;
  }
  // Liquid nitrogen freezing to solid nitrogen: a LIQUID whose low-temperature branch must
  // survive the rule untouched.
  els[3].id = 3;  els[3].state = kStateLiquid;  els[3].molarMass = 28.0f;
  els[3].specificHeatCapacity = 2.042f;  els[3].lowTemp = 63.15f;
  els[3].lowTempTransitionIdx = 4;       els[3].highTempTransitionIdx = kNoElement;
  els[4].id = 4;  els[4].state = kStateSolid;   els[4].molarMass = 28.0f;
  els[4].specificHeatCapacity = 1.040f;  els[4].lowTempTransitionIdx = kNoElement;
  els[4].highTempTransitionIdx = kNoElement;
  // Helium, which condenses in no table anywhere.
  els[5].id = 5;  els[5].state = kStateGas;     els[5].molarMass = 4.0f;
  els[5].specificHeatCapacity = 5.193f;  els[5].lowTemp = 4.22f;
  els[5].lowTempTransitionIdx = 2;       els[5].highTempTransitionIdx = kNoElement;

  std::vector<uint8_t> blob(4 + els.size() * sizeof(Element));
  const int32_t n = static_cast<int32_t>(els.size());
  memcpy(blob.data(), &n, 4);
  memcpy(blob.data() + 4, els.data(), els.size() * sizeof(Element));

  ElementTable table;
  if (!table.Load(blob.data(), blob.size())) {
    printf("  FAIL: synthetic condensation ElementTable failed to load\n");
    ++g_fail_count;
  }
  return table;
}

// Stationeers' own constants, `MoleHelper.EvaporationCoefficientA/B`.
constexpr float kCO2CurveA = 1.579573e-26f;
constexpr float kCO2CurveB = 12.195837931f;
constexpr float kCO2MinLiquidPressurePa = 517000.0f;  // 517 kPa, as in Stationeers
constexpr float kN2CurveA = 5.5757107833e-07f;
constexpr float kN2CurveB = 4.40221368946f;

void WriteCurve(ElementTable& table, int32_t id, float a, float b, float freezing, float critical,
                float latent) {
  const float v[5] = {a, b, freezing, critical, latent};
  for (int32_t k = 0; k < ext::kPhaseCurveArity; ++k) {
    uint32_t bits = 0;
    memcpy(&bits, &v[k], sizeof(float));
    table.MutableAttributes().Write(table.PhaseCurveAttribute(), id, k, bits);
  }
}

void TestCellCondensationRule() {
  printf("\n=== cell condensation rule ===\n");
  ElementTable table = MakeCondensationTable();
  constexpr uint16_t kGasCO2 = 1, kLiqCO2 = 2, kLiqN2 = 3, kSolidN2 = 4, kHelium = 5;

  // Stationeers derives each freezing/critical constant from its OWN curve at its own bounds:
  // TRIPLE_POINT_TEMPERATURE_CARBON_DIOXIDE_K = EvaporationTemperature(CO2, 517 kPa) and
  // CRITICAL_TEMPERATURE_CARBON_DIOXIDE_K = EvaporationTemperature(CO2, 6000 kPa). Computing
  // them the same way here keeps the test honest about the identity the clamp argument rests on.
  ElementTable::PhaseCurve raw_co2;
  raw_co2.a = kCO2CurveA;  raw_co2.b = kCO2CurveB;
  raw_co2.freezing_k = 0.0f;  raw_co2.critical_k = 1.0e9f;  raw_co2.latent_j_per_kg = 571000.0f;
  const float co2_freezing = EvaporationTemperatureClampedK(raw_co2, 517000.0f);
  const float co2_critical = EvaporationTemperatureClampedK(raw_co2, 6000000.0f);
  printf("  CO2 curve: freezing(=raw at 517 kPa) = %.4f K, critical(=raw at 6 MPa) = %.4f K\n",
         co2_freezing, co2_critical);
  Check(co2_freezing > 217.0f && co2_freezing < 218.0f,
        "CO2 freezing point derives from its own curve at 517 kPa (~217.8 K)");
  Check(co2_critical > 266.0f && co2_critical < 267.0f,
        "CO2 critical point derives from its own curve at 6 MPa (~266.3 K)");

  ElementTable::PhaseCurve co2 = raw_co2;
  co2.freezing_k = co2_freezing;
  co2.critical_k = co2_critical;

  // -------- ARM 1: an undescribed element keeps Klei's threshold, untouched.
  PhaseEntry cell{};
  cell.element = kGasCO2;
  cell.mass = 1.0f;
  cell.temperature = 150.0f;  // far below lowTemp - 3, so Klei's branch has already fired
  Check(CondensationAllowed(table, cell, kLiqCO2, table.AnyCondensationRules()),
        "an element with no registered attributes keeps Klei's threshold (gate safe by construction)");

  // Half a ruleset is still no ruleset: a curve with no `sim.can_condense` must not engage.
  WriteCurve(table, /*id=*/1, co2.a, co2.b, co2.freezing_k, co2.critical_k, co2.latent_j_per_kg);
  Check(CondensationAllowed(table, cell, kLiqCO2, table.AnyCondensationRules()),
        "a curve without sim.can_condense is not a rule -- still vanilla");

  // -------- ARM 2: the Mars case. Described, but four orders of magnitude under the floor.
  table.MutableAttributes().WriteF32(table.CanCondenseAttribute(), 1, 1.0f);
  table.MutableAttributes().WriteF32(table.MinLiquidPressureAttribute(), 1,
                                     kCO2MinLiquidPressurePa);
  // 1 kg of CO2 at 210 K in a 1 m^3 cell: n = 1000/44 mol, P = nRT/V ~ 39.7 kPa. Martian
  // surface pressure is lower still; either way it is far under the 517 kPa floor.
  cell.mass = 1.0f;
  cell.temperature = 210.0f;
  const float mars_pressure =
      gas::PressureFromMoles(cell.mass * gas::kGramsPerKilogram / 44.0f, cell.temperature,
                             g_tunables.cell_volume_m3);
  printf("  Mars-like cell: 1.0 kg CO2 at 210 K -> %.1f Pa (floor is %.0f Pa)\n", mars_pressure,
         kCO2MinLiquidPressurePa);
  Check(mars_pressure < kCO2MinLiquidPressurePa, "the Mars-like cell really is under the floor");
  Check(!CondensationAllowed(table, cell, kLiqCO2, table.AnyCondensationRules()),
        "CO2 under its 517 kPa floor REFUSES to condense however cold it gets (the Mars defect)");

  // And it refuses at every temperature, not just this one -- a floor is not a threshold.
  bool refused_everywhere = true;
  for (float t = 60.0f; t <= 190.0f; t += 5.0f) {
    cell.temperature = t;
    if (CondensationAllowed(table, cell, kLiqCO2, table.AnyCondensationRules())) refused_everywhere = false;
  }
  Check(refused_everywhere, "the pressure floor refuses across 60..190 K, not at one temperature");

  // -------- ARM 3: above the floor, the saturation temperature decides.
  //
  // The two tests are NOT independent, and this arm is built around that. A cell's pressure is
  // `nRT/V`, so it falls with its own temperature: cooling a fixed mass toward its saturation
  // point pushes it back DOWN through its pressure floor. 12 kg of CO2 clears 517 kPa at 240 K
  // and no longer does at 214 K, which is the first shape this test was written in and why it
  // failed. The honest arm holds the MASS fixed and moves only the temperature, with the floor
  // cleared at both ends.
  //
  // 14 kg: 556 kPa at 210 K (t_sat 219.1, so it condenses) and 635 kPa at 240 K (t_sat 221.5,
  // so it does not).
  cell.mass = 14.0f;
  cell.temperature = 210.0f;
  const float cold_pressure =
      gas::PressureFromMoles(cell.mass * gas::kGramsPerKilogram / 44.0f, cell.temperature,
                             g_tunables.cell_volume_m3);
  const float cold_t_sat = EvaporationTemperatureClampedK(co2, cold_pressure);
  printf("  14 kg CO2 at 210 K -> %.0f Pa, t_sat = %.2f K\n", cold_pressure, cold_t_sat);
  Check(cold_pressure > kCO2MinLiquidPressurePa, "the pressurised cell clears the floor at 210 K");
  Check(210.0f < cold_t_sat && CondensationAllowed(table, cell, kLiqCO2, table.AnyCondensationRules()),
        "above the floor and below t_sat, the rule permits the transition");

  cell.temperature = 240.0f;
  const float warm_pressure =
      gas::PressureFromMoles(cell.mass * gas::kGramsPerKilogram / 44.0f, cell.temperature,
                             g_tunables.cell_volume_m3);
  const float warm_t_sat = EvaporationTemperatureClampedK(co2, warm_pressure);
  printf("  14 kg CO2 at 240 K -> %.0f Pa, t_sat = %.2f K\n", warm_pressure, warm_t_sat);
  Check(warm_pressure > kCO2MinLiquidPressurePa, "the same cell still clears the floor at 240 K");
  Check(240.0f > warm_t_sat && !CondensationAllowed(table, cell, kLiqCO2, table.AnyCondensationRules()),
        "above the floor and above t_sat, the rule refuses (Temperature >= t_sat -> Invalid)");

  // -------- ARM 3b: RESTRICTIVE-ONLY, asserted end to end through TransitionCell.
  //
  // The property the whole design rests on. `CondensationAllowed` is an `&&` on Klei's branch,
  // so a cell the rule PERMITS still only transitions if Klei's fixed threshold also fired.
  // At 210 K the 14 kg cell above is below its saturation temperature and over its floor --
  // Stationeers would condense it -- and Klei's `lowTemp - 3` is 191.65 K, so nothing happens.
  //
  // That gap is deliberate and is documented at the rule: a transition the rule ADDED would
  // oscillate, because the product liquid's boil-back threshold is still a fixed
  // one-atmosphere constant with no pressure term. Asserting it here means the day someone
  // lifts the restriction, this arm fails and points at the reason.
  {
    World w;
    w.Allocate(3, 3);
    const size_t mid = w.Padded(1 + 1 * 3);
    std::vector<StateChangeOre> ores;

    w.Phase(mid).element = kGasCO2;
    w.Phase(mid).mass = 14.0f;
    w.Phase(mid).temperature = 210.0f;
    const bool fired_permitted = TransitionCell(&w, table, 1 + 1, 1 + 1, &ores);
    Check(!fired_permitted && w.Phase(mid).element == kGasCO2,
          "a cell the RULE permits does not transition unless Klei's threshold also fired "
          "(restrictive-only: the rule can subtract a transition, never add one)");

    // And the subtraction itself, end to end: the same element cold enough for Klei's branch,
    // under its pressure floor. Vanilla would have made this liquid CO2; the rule refuses.
    w.Phase(mid).element = kGasCO2;
    w.Phase(mid).mass = 1.0f;       // 1 kg -> ~37 kPa at 185 K, far under the 517 kPa floor
    w.Phase(mid).temperature = 185.0f;  // under lowTemp - TransitionMargin = 191.65 K
    const bool fired_refused = TransitionCell(&w, table, 1 + 1, 1 + 1, &ores);
    Check(!fired_refused && w.Phase(mid).element == kGasCO2,
          "under the floor, a cell past Klei's threshold stays gas -- the defect fix, end to end");

    // The control that proves the arm above is testing the RULE and not the geometry: the same
    // cell, same temperature, with the rule withdrawn, must transition exactly as vanilla did.
    ElementTable vanilla = MakeCondensationTable();
    w.Phase(mid).element = kGasCO2;
    w.Phase(mid).mass = 1.0f;
    w.Phase(mid).temperature = 185.0f;
    const bool fired_vanilla = TransitionCell(&w, vanilla, 1 + 1, 1 + 1, &ores);
    Check(fired_vanilla && w.Phase(mid).element == kLiqCO2,
          "...and with no rule registered the SAME cell condenses, as vanilla always did");
  }

  // -------- ARM 4: Helium. Floor 0.0 Pa, so ONLY the table stops it.
  WriteCurve(table, /*id=*/5, kN2CurveA, kN2CurveB, 1.0f, 5.2f, 20000.0f);
  table.MutableAttributes().WriteF32(table.CanCondenseAttribute(), 5, 0.0f);
  table.MutableAttributes().WriteF32(table.MinLiquidPressureAttribute(), 5, 0.0f);
  PhaseEntry helium{};
  helium.element = kHelium;
  helium.mass = 1.0f;
  helium.temperature = 2.0f;
  Check(!CondensationAllowed(table, helium, kLiqCO2, table.AnyCondensationRules()),
        "Helium refuses on the CanCondense table alone -- its pressure floor is 0.0 Pa");

  // -------- ARM 5: a curve-carrying LIQUID still freezes.
  // This is the arm that would have caught keying the rule on "has a curve". Liquid nitrogen
  // carries its gas's curve because RegisterFamily pushes one MaterialProperties to both.
  WriteCurve(table, /*id=*/3, kN2CurveA, kN2CurveB, 40.0f, 190.0f, 199000.0f);
  table.MutableAttributes().WriteF32(table.CanCondenseAttribute(), 3, 0.0f);
  table.MutableAttributes().WriteF32(table.MinLiquidPressureAttribute(), 3, 6300.0f);
  PhaseEntry n2{};
  n2.element = kLiqN2;
  n2.mass = 100.0f;
  n2.temperature = 50.0f;  // well under its 63.15 K freezing point
  Check(CondensationAllowed(table, n2, kSolidN2, table.AnyCondensationRules()),
        "a LIQUID carrying a curve and can_condense=0 still freezes (rule is gas->liquid only)");

  // -------- ARM 6: the clamp. Retracting the doc's bypass is only safe if these hold.
  // Below the floor the lower clamp moves the answer a long way -- and every one of those
  // pressures is refused before the curve is ever evaluated, which is why it is unreachable.
  const float raw_at_2470 = EvaporationTemperatureClampedK(raw_co2, 2470.0f);
  const float clamped_at_2470 = EvaporationTemperatureClampedK(co2, 2470.0f);
  printf("  at 2.47 kPa: raw = %.2f K, clamped = %.2f K (gate refuses this pressure first)\n",
         raw_at_2470, clamped_at_2470);
  Check(clamped_at_2470 > raw_at_2470 + 70.0f,
        "below the floor the lower clamp moves t_sat by ~77 K -- the doc's worry, unreachable");
  Check(2470.0f < kCO2MinLiquidPressurePa,
        "...unreachable because 2.47 kPa is refused by the floor before the curve is evaluated");

  // At and above the floor the lower clamp is a no-op, because freezing_k IS raw(floor).
  bool lower_clamp_inert = true;
  for (float p = kCO2MinLiquidPressurePa; p <= 6000000.0f; p *= 1.5f) {
    if (EvaporationTemperatureClampedK(co2, p) < EvaporationTemperatureClampedK(raw_co2, p)) {
      lower_clamp_inert = false;
    }
  }
  Check(lower_clamp_inert,
        "at every pressure the floor admits, the lower clamp is inert (freezing_k == raw(floor))");

  // The upper clamp is the half that earns its keep: above the critical pressure the raw curve
  // would hand back a saturation temperature above the critical temperature, and a cell there
  // would condense into a liquid that cannot exist.
  const float raw_8mpa = EvaporationTemperatureClampedK(raw_co2, 8000000.0f);
  const float clamped_8mpa = EvaporationTemperatureClampedK(co2, 8000000.0f);
  printf("  at 8 MPa: raw = %.2f K, clamped = %.2f K (critical is %.2f K)\n", raw_8mpa,
         clamped_8mpa, co2_critical);
  Check(raw_8mpa > co2_critical && clamped_8mpa <= co2_critical + 1e-3f,
        "above critical pressure the upper clamp holds t_sat at the critical temperature");
  cell.mass = 12.0f;
  cell.temperature = 270.0f;
  Check(270.0f < raw_8mpa && 270.0f > clamped_8mpa,
        "a 270 K cell at 8 MPa would condense UNCLAMPED and correctly refuses CLAMPED");
}

// The cell boil rule (`BoilingAllowed` / `AmbientPressureAtLiquidCellPa`,
// sim/physics.h) -- the other half of the pressure term.
//
// The condensation rule could only ever SUBTRACT a transition because the boil-back test was
// Klei's fixed one-atmosphere `highTemp`. These arms assert the replacement in both
// directions: the ambient pressure a liquid cell sits under, and the gate that reads it.
void TestCellBoilRule() {
  printf("\n=== cell boil rule ===\n");
  ElementTable table = MakeCondensationTable();
  constexpr uint16_t kGasCO2 = 1, kLiqCO2 = 2, kLiqN2 = 3, kSolidN2 = 4;
  (void)kLiqN2;
  (void)kSolidN2;

  // -------- ARM 1: the ambient pressure itself, one column shape at a time.
  //
  // Every number below is arithmetic that can be checked by hand, which is the point: the
  // hydrostatic term is `mass * g` exactly (a 1 m^2 cell cross-section), so a 1000 kg tile is
  // 9806.65 Pa and nothing about it is a fitted constant.
  {
    World w;
    w.Allocate(3, 6);
    const int32_t px = 1 + 1, py0 = 1 + 1;  // padded coords of game column x=1
    ElementTable& t = table;

    // (a) OPEN TO VACUUM. Nothing above, so the only term is the cell's own half-mass.
    w.Phase(w.Padded(1 + 1 * 3)).element = kLiqCO2;
    w.Phase(w.Padded(1 + 1 * 3)).mass = 500.0f;
    w.Phase(w.Padded(1 + 1 * 3)).temperature = 200.0f;
    const float open = AmbientPressureAtLiquidCellPa(w, t, px, py0);
    const float expect_open = 0.5f * 500.0f * 9.80665f;
    printf("  open to vacuum: %.1f Pa (expected %.1f)\n", open, expect_open);
    Check(std::fabs(open - expect_open) < 0.5f,
          "a liquid cell open to vacuum is pressed only by half of its own mass");

    // (b) GAS ABOVE. The gas cell's own nRT/V, plus the same half-mass.
    const size_t above = w.Padded(1 + 2 * 3);
    w.Phase(above).element = kGasCO2;
    w.Phase(above).mass = 1.2f;
    w.Phase(above).temperature = 300.0f;
    const float gas_pa = gas::PressureFromMoles(1.2f * gas::kGramsPerKilogram / 44.0f, 300.0f,
                                                g_tunables.cell_volume_m3);
    const float with_gas = AmbientPressureAtLiquidCellPa(w, t, px, py0);
    printf("  gas above: %.1f Pa = %.1f gas + %.1f head\n", with_gas, gas_pa, expect_open);
    Check(std::fabs(with_gas - (gas_pa + expect_open)) < 1.0f,
          "gas above the column contributes its own pressure");

    // (c) SOLID ABOVE. A sealed cell has no ambient to read and keeps the head alone --
    // which is what leaves Klei's threshold governing a sealed tile.
    w.Phase(above).element = kSolidN2;
    w.Phase(above).mass = 900.0f;
    w.Phase(above).temperature = 50.0f;
    const float sealed = AmbientPressureAtLiquidCellPa(w, t, px, py0);
    printf("  solid above: %.1f Pa\n", sealed);
    Check(std::fabs(sealed - expect_open) < 0.5f,
          "a solid ceiling contributes nothing -- a sealed liquid tile keeps its head only");

    // (d) A COLUMN. Two full liquid tiles on top of the cell, then gas.
    w.Phase(above).element = kLiqCO2;
    w.Phase(above).mass = 1000.0f;
    w.Phase(above).temperature = 200.0f;
    const size_t above2 = w.Padded(1 + 3 * 3);
    w.Phase(above2).element = kLiqCO2;
    w.Phase(above2).mass = 1000.0f;
    w.Phase(above2).temperature = 200.0f;
    const size_t sky = w.Padded(1 + 4 * 3);
    w.Phase(sky).element = kGasCO2;
    w.Phase(sky).mass = 1.2f;
    w.Phase(sky).temperature = 300.0f;
    const float deep = AmbientPressureAtLiquidCellPa(w, t, px, py0);
    const float expect_deep = gas_pa + (2000.0f + 0.5f * 500.0f) * 9.80665f;
    printf("  under 2 full tiles: %.1f Pa (expected %.1f)\n", deep, expect_deep);
    Check(std::fabs(deep - expect_deep) < 1.0f,
          "the column above presses with its own mass -- 1000 kg of liquid is 9.81 kPa, "
          "which is a metre of water and is not a tuned number");
    Check(deep > with_gas,
          "and depth therefore matters: the same cell under two tiles is at a higher "
          "pressure than the same cell open to the same sky");
  }

  // -------- ARM 2: the gate, and ROW 12's EXACT CASE.
  //
  // This is the arm the whole workstream exists for. Without pressure-dependent phase change,
  // condensing CO2 at 2 MPa yields liquid at 241.5 K -- 240 K plus `TransitionOvershoot` --
  // which Klei's fixed 194.65 K boil-back threshold then boils straight back, every substep,
  // re-flooding the room graph on each flip. At 2 MPa the curve puts t_sat at 243.37 K, so the
  // pressure-aware rule refuses that boil and the oscillation cannot start.
  {
    WriteCurve(table, /*id=*/1, kCO2CurveA, kCO2CurveB, 217.82f, 266.31f, 13636.0f);
    ElementTable::PhaseCurve co2;
    Check(table.PhaseCurveOf(kGasCO2, &co2), "the curve is readable off the GAS member");
    const float t_sat_2mpa = EvaporationTemperatureClampedK(co2, 2000000.0f);
    printf("  t_sat at 2 MPa = %.2f K; the liquid row 12 describes is at 241.50 K\n", t_sat_2mpa);
    Check(t_sat_2mpa > 241.5f && t_sat_2mpa < 244.0f,
          "CO2's saturation temperature at 2 MPa is above the 241.5 K a 240 K condensation "
          "leaves behind -- which is what makes the flip-flop impossible");

    World w;
    w.Allocate(3, 4);
    const int32_t px = 1 + 1, py = 1 + 1;
    const size_t mid = w.Padded(1 + 1 * 3);
    const size_t above = w.Padded(1 + 2 * 3);
    std::vector<StateChangeOre> ores;

    // The liquid CO2 cell, under 2 MPa of its own gas. 2 MPa at 300 K is 35.3 kg in a cell.
    auto set_scene = [&](float liquid_t) {
      w.Phase(mid).element = kLiqCO2;
      w.Phase(mid).mass = 0.0f;  // no head: this arm is about the gas term alone
      w.Phase(mid).temperature = liquid_t;
      w.Phase(above).element = kGasCO2;
      w.Phase(above).temperature = 300.0f;
      w.Phase(above).mass = 2000000.0f * 44.0f /
                            (gas::kGasConstantR * 300.0f) / gas::kGramsPerKilogram;
    };
    set_scene(241.5f);
    const float ambient = AmbientPressureAtLiquidCellPa(w, table, px, py);
    printf("  the gas above is %.0f Pa\n", ambient);
    Check(ambient > 1.9e6f && ambient < 2.1e6f, "the scene really is at about 2 MPa");
    Check(!BoilingAllowed(w, table, w.Phase(mid), px, py, kGasCO2, table.AnyPhaseCurves()),
          "ROW 12 UNBLOCKED: liquid CO2 at 241.5 K under 2 MPa is BELOW its saturation "
          "temperature and the rule refuses the boil Klei's fixed threshold wanted");

    // Mass back, so `TransitionCell` has something to transition, and end to end.
    w.Phase(mid).mass = 1.0f;
    const bool fired = TransitionCell(&w, table, px, py, &ores,
                                      PhaseRules{table.AnyCondensationRules(),
                                                 table.AnyPhaseCurves()});
    Check(!fired && w.Phase(mid).element == kLiqCO2,
          "...and end to end through TransitionCell the cell stays liquid CO2");

    // The control. Same cell, same temperature, rule withdrawn: vanilla boils it, which is
    // what proves the arm above is testing the RULE and not the geometry.
    ElementTable vanilla = MakeCondensationTable();
    w.Phase(mid).element = kLiqCO2;
    w.Phase(mid).mass = 1.0f;
    w.Phase(mid).temperature = 241.5f;
    const bool fired_vanilla = TransitionCell(&w, vanilla, px, py, &ores);
    Check(fired_vanilla && w.Phase(mid).element == kGasCO2,
          "...and with no curve registered the SAME cell boils, as vanilla always did -- "
          "which is the oscillation, measured");
  }

  // -------- ARM 3: RESTRICTIVE-ONLY, the same property the condensation rule has.
  //
  // The rule is an `&&` on Klei's branch, so a cell the rule would boil does not boil unless
  // Klei's fixed threshold fired too. Asserting it means the day someone lifts the
  // restriction, this arm fails and points at the reason.
  {
    World w;
    w.Allocate(3, 4);
    const int32_t px = 1 + 1, py = 1 + 1;
    const size_t mid = w.Padded(1 + 1 * 3);
    std::vector<StateChangeOre> ores;

    // FINDING, and it cost this arm one rewrite: with CO2's own numbers there is no case
    // where the rule permits a boil Klei refuses, because the LOWER CLAMP holds t_sat at the
    // freezing point (217.82 K) and that is already above this table's Klei threshold of
    // 194.65 + 3 K. Liquid CO2 below its own triple point cannot exist, so the clamp is
    // saying something true. To assert the restrictive-only PROPERTY, the curve needs a
    // freezing point below Klei's boiling threshold -- which is the ordinary arrangement for
    // every real substance, water included (273.15 K against 372.15 K), and is only unusual
    // for CO2 because its liquid does not exist at one atmosphere at all.
    ElementTable low = MakeCondensationTable();
    WriteCurve(low, /*id=*/1, kCO2CurveA, kCO2CurveB, 100.0f, 266.31f, 13636.0f);
    w.Phase(mid).element = kLiqCO2;
    w.Phase(mid).mass = 1.0f;
    w.Phase(mid).temperature = 150.0f;
    Check(BoilingAllowed(w, low, w.Phase(mid), px, py, kGasCO2, low.AnyPhaseCurves()),
          "in vacuum the rule PERMITS a boil well under Klei's threshold");
    const bool fired = TransitionCell(&w, low, px, py, &ores,
                                      PhaseRules{low.AnyCondensationRules(),
                                                 low.AnyPhaseCurves()});
    Check(!fired && w.Phase(mid).element == kLiqCO2,
          "...and nothing happens anyway: the rule can subtract a boil, never add one");
  }

  // -------- ARM 4: the rule is keyed on the CURVE and the PHASE PAIR, nothing else.
  //
  // Stationeers gates its Liquid case on `MoleHelper.CanEvaporate` (`:223`), which reads true
  // for all fifteen liquids and false for every gas -- so it says exactly what the phase of
  // the element says, and there is no second table to carry. These arms pin that reading.
  {
    World w;
    w.Allocate(3, 3);
    const int32_t px = 1 + 1, py = 1 + 1;
    const size_t mid = w.Padded(1 + 1 * 3);

    // A GAS cell never reaches the rule, whatever its curve says.
    w.Phase(mid).element = kGasCO2;
    w.Phase(mid).mass = 1.0f;
    w.Phase(mid).temperature = 100.0f;
    Check(BoilingAllowed(w, table, w.Phase(mid), px, py, kGasCO2, table.AnyPhaseCurves()),
          "a gas cell is not a liquid and the boil rule declines to have an opinion");

    // A liquid whose high-temperature target is not a gas is not a boil either.
    w.Phase(mid).element = kLiqCO2;
    Check(BoilingAllowed(w, table, w.Phase(mid), px, py, kSolidN2, table.AnyPhaseCurves()),
          "a liquid -> solid target is freezing, not boiling, and keeps Klei's threshold");

    // And an undescribed element keeps Klei's threshold -- unset is not zero, which is what
    // makes the byte-identity gate safe by construction.
    ElementTable bare = MakeCondensationTable();
    Check(!bare.AnyPhaseCurves(),
          "a table nobody has written to reports no curves at all (the offline state)");
    Check(BoilingAllowed(w, bare, w.Phase(mid), px, py, kGasCO2, bare.AnyPhaseCurves()),
          "and with no curve registered the rule permits everything Klei permits");
  }

  // -------- ARM 5: A REFUSED BOIL DOES NOT STRAND THE CELL.
  //
  // `TransitionCell` tests the branches as an if / else-if, so a high branch the rule refuses
  // falls through to the low one. That is not a detail: liquid CO2 at 200 K is below its own
  // triple point, where Klei's fixed 194.65 K boiling point has it becoming GAS and the real
  // substance becomes SOLID. Refusing the boil is what lets the freeze happen, so the rule
  // does not merely subtract a wrong transition -- here it uncovers the right one.
  {
    ElementTable freezes = MakeCondensationTable(/*liquid_co2_freezes=*/true);
    WriteCurve(freezes, /*id=*/1, kCO2CurveA, kCO2CurveB, 217.82f, 266.31f, 13636.0f);
    World w;
    w.Allocate(3, 3);
    const int32_t px = 1 + 1, py = 1 + 1;
    const size_t mid = w.Padded(1 + 1 * 3);
    std::vector<StateChangeOre> ores;
    w.Phase(mid).element = kLiqCO2;
    w.Phase(mid).mass = 1.0f;
    w.Phase(mid).temperature = 200.0f;  // past Klei's 197.65 K boil threshold, under 213.15 K
    const bool fired = TransitionCell(&w, freezes, px, py, &ores,
                                      PhaseRules{freezes.AnyCondensationRules(),
                                                 freezes.AnyPhaseCurves()});
    Check(fired && w.Phase(mid).element == kSolidN2,
          "a boil the rule refuses falls through to the freeze branch: liquid CO2 at 200 K is "
          "below its triple point and becomes SOLID, where vanilla made it gas");

    // The control, and it is the same cell in the same table with the curve withdrawn.
    ElementTable no_curve = MakeCondensationTable(/*liquid_co2_freezes=*/true);
    w.Phase(mid).element = kLiqCO2;
    w.Phase(mid).mass = 1.0f;
    w.Phase(mid).temperature = 200.0f;
    const bool fired_vanilla = TransitionCell(&w, no_curve, px, py, &ores);
    Check(fired_vanilla && w.Phase(mid).element == kGasCO2,
          "...and with no curve the SAME cell boils to gas, which is the transition the rule "
          "replaced rather than merely removed");
  }
}

void TestProjectionMultiPhaseRules() {
  printf("\n=== projection rules on a multi-phase cell (count > 1) ===\n");
  ElementTable table = MakeProjectionTable();
  constexpr uint16_t kGasCO2 = 1, kLiqCO2 = 2, kSolid = 3;

  // -------- MeanTemperature: the single-phase path stays BIT-exact.
  // Not "close to": `m*T/m` is not `T` in float, and projection.h records that every gas
  // scenario went one ulp adrift the one time this division was allowed to happen. The
  // heat-capacity change must not have reintroduced it.
  {
    PhaseEntry one{};
    one.element = kGasCO2; one.mass = 3.25f; one.temperature = 293.15f;
    Check(MeanTemperature(&one, 1, table) == 293.15f,
          "MeanTemperature: one phase returns its own temperature bit-exactly, no division");
    PhaseEntry massless{};
    massless.element = kGasCO2; massless.mass = 0.0f; massless.temperature = 293.15f;
    Check(MeanTemperature(&massless, 1, table) == 293.15f,
          "MeanTemperature: a massless single phase keeps its temperature, not 0 K");
  }

  // -------- MeanTemperature: two phases weight by HEAT CAPACITY, not by mass.
  // Equal masses, specific heats 0.846 and 2.000, temperatures 200 K and 300 K. Mass
  // weighting would say 250.0; the heat-capacity mean is pulled toward the liquid.
  {
    PhaseEntry two[2]{};
    two[0].element = kGasCO2; two[0].mass = 1.0f; two[0].temperature = 200.0f;
    two[1].element = kLiqCO2; two[1].mass = 1.0f; two[1].temperature = 300.0f;
    const float got = MeanTemperature(two, 2, table);
    const float mc0 = 1.0f * 0.846f, mc1 = 1.0f * 2.000f;
    const float want = (mc0 * 200.0f + mc1 * 300.0f) / (mc0 + mc1);
    printf("  two-phase mean: got %.6f K, heat-capacity answer %.6f K, mass answer 250.0 K\n",
           got, want);
    Check(std::fabs(got - want) < 1e-4f,
          "MeanTemperature: two phases are weighted by mass * specific heat");
    Check(std::fabs(got - 250.0f) > 1.0f,
          "MeanTemperature: and that is NOT the mass-weighted answer -- the fix is load-bearing");
  }

  // -------- MeanTemperature: one contributor among several is exact, and the all-zero
  // specific-heat case falls back to mass weighting instead of dividing by zero.
  {
    PhaseEntry two[2]{};
    two[0].element = kGasCO2; two[0].mass = 0.0f; two[0].temperature = 111.0f;
    two[1].element = kLiqCO2; two[1].mass = 4.0f; two[1].temperature = 277.75f;
    Check(MeanTemperature(two, 2, table) == 277.75f,
          "MeanTemperature: a single contributor among several reports its temperature exactly");

    ElementTable zero_table = MakeTestTable();  // every specificHeatCapacity is 0 there
    PhaseEntry z[2]{};
    z[0].element = kO2;  z[0].mass = 1.0f; z[0].temperature = 200.0f;
    z[1].element = kCO2; z[1].mass = 3.0f; z[1].temperature = 300.0f;
    const float zf = MeanTemperature(z, 2, zero_table);
    Check(std::fabs(zf - 275.0f) < 1e-4f,
          "MeanTemperature: all-zero specific heats fall back to the mass-weighted mean");
  }

  // -------- HasSolidPhase / SolidFraction: the premise the relaxed solidity cache rests on.
  // A gas+liquid cell is never PART solid, which is why `!HasSolidPhase` is a sound second
  // arm for `solid_unchanged` and why `count == 1` alone was stricter than its justification.
  {
    PhaseEntry gl[2]{};
    gl[0].element = kGasCO2; gl[0].mass = 2.0f; gl[0].temperature = 250.0f;
    gl[1].element = kLiqCO2; gl[1].mass = 5.0f; gl[1].temperature = 250.0f;
    Check(!HasSolidPhase(gl, 2, table),
          "HasSolidPhase: a gas+liquid cell has no solid phase -- the cache stays valid");
    Check(SolidFraction(gl, 2, table) == 0.0f,
          "SolidFraction: and its solid fraction is exactly 0, so ProjectSolid cannot move");

    PhaseEntry with_solid[2]{};
    with_solid[0].element = kGasCO2; with_solid[0].mass = 2.0f;
    with_solid[1].element = kSolid;  with_solid[1].mass = 5.0f;
    Check(HasSolidPhase(with_solid, 2, table),
          "HasSolidPhase: a cell with real solid mass is part solid -- the cache is surrendered");
    Check(SolidFraction(with_solid, 2, table) > 0.0f && SolidFraction(with_solid, 2, table) < 1.0f,
          "SolidFraction: a part-solid cell reports a fraction strictly between 0 and 1");

    PhaseEntry massless_solid[2]{};
    massless_solid[0].element = kGasCO2; massless_solid[0].mass = 2.0f;
    massless_solid[1].element = kSolid;  massless_solid[1].mass = 0.0f;
    Check(!HasSolidPhase(massless_solid, 2, table),
          "HasSolidPhase: a solid phase carrying no mass does not make the cell part solid");

    // The single-phase SOLID cell is the case a straight substitution of the guard would have
    // broken: it keeps the cache today via the `count == 1` arm, and `HasSolidPhase` is true
    // for it -- so the two arms are genuinely different questions, not one spelled two ways.
    PhaseEntry solid_only{};
    solid_only.element = kSolid; solid_only.mass = 800.0f;
    Check(HasSolidPhase(&solid_only, 1, table),
          "HasSolidPhase: true for a single-phase solid -- which is why the guard needed a "
          "disjunction, not a substitution");
  }

  // -------- DominantElement: by VOLUME across phases, not by mass.
  // Equal masses of two phases with the same molar mass tie and resolve by lower element id;
  // the solid, denser per the table, needs less mass to dominate.
  {
    PhaseEntry two[2]{};
    two[0].element = kLiqCO2; two[0].mass = 1.0f;
    two[1].element = kGasCO2; two[1].mass = 4.0f;
    Check(DominantElement(two, 2, table) == kGasCO2,
          "DominantElement: the larger volume wins across a two-phase list");
    Check(TotalMass(two, 2) == 5.0f, "TotalMass: sums every phase in the list");

    PhaseEntry zero_mass[2]{};
    zero_mass[0].element = kGasCO2; zero_mass[0].mass = 0.0f;
    zero_mass[1].element = kLiqCO2; zero_mass[1].mass = 0.0f;
    Check(DominantElement(zero_mass, 2, table) == kGasCO2,
          "DominantElement: an all-massless list falls back to the first phase rather than "
          "reading uninitialised state");
  }
}

// Single species, two cells, one high-pressure one empty: mass should flow one direction,
// conserve exactly, and pressures should converge.
void TestSinglePairEqualizes() {
  printf("\n=== single-species pair ===\n");
  ElementTable table = MakeTestTable();
  World w;
  w.Allocate(3, 1);
  const size_t a = w.Padded(0);
  const size_t b = w.Padded(1);
  w.Phase(a).temperature = 300.0f;
  w.Phase(b).temperature = 300.0f;
  w.Phase(a).element = kVacuum;
  w.Phase(b).element = kVacuum;

  InjectSpecies(w, a, kO2, 10.0f);  // b starts empty

  const float mass_before = w.GasMass(a, FindSlot(w, a, kO2));
  for (int i = 0; i < 200; ++i) MixPair(w, table, a, b, 0.2f);

  const int slot_a = FindSlot(w, a, kO2);
  const int slot_b = FindSlot(w, b, kO2);
  const float mass_a_after = slot_a >= 0 ? w.GasMass(a, slot_a) : 0.0f;
  const float mass_b_after = slot_b >= 0 ? w.GasMass(b, slot_b) : 0.0f;
  const float total_after = mass_a_after + mass_b_after;

  printf("  before: a=%.4f kg, b=0 kg\n", mass_before);
  printf("  after 200 steps: a=%.4f kg, b=%.4f kg (total %.4f, started %.4f)\n",
         mass_a_after, mass_b_after, total_after, mass_before);
  Check(std::fabs(total_after - mass_before) < 1e-4f, "mass conserved across the pair");
  Check(std::fabs(mass_a_after - mass_b_after) < 0.05f * mass_before,
        "equal-volume equal-temperature pair converges toward equal mass");

  const float pa = CellPressure(w, table, a);
  const float pb = CellPressure(w, table, b);
  printf("  final pressure: a=%.4f Pa, b=%.4f Pa\n", pa, pb);
  Check(std::fabs(pa - pb) < 0.05f * pa, "partial pressures converge");
}

// MixPair carries heat with the mass it moves (moving mass alone would let a promoted room
// gain energy from nothing). Warm O2 on one side, cold CO2 on the other, real specific heats,
// and a vanilla gas under b's mixture so the vanilla-mass share of the ledger booking is
// exercised too. It is not a liquid pool, because the mixture is never given to a cell holding
// liquid (see TestMixPairSkipsLiquid); a vanilla gas sharing a cell with the mixture is a real
// state. MakeTestTable's elements carry no specific heat, which is why this test has its own
// table.
void TestMixPairCarriesHeat() {
  printf("\n=== MixPair carries heat with the mass ===\n");
  std::vector<Element> els(4);
  memset(els.data(), 0, els.size() * sizeof(Element));
  els[0].id = 0;  els[0].state = kStateVacuum;
  els[1].id = 1;  els[1].state = kStateGas;     els[1].molarMass = 0.032f;
  els[1].specificHeatCapacity = 1.005f;         // O2
  els[2].id = 2;  els[2].state = kStateGas;     els[2].molarMass = 0.044f;
  els[2].specificHeatCapacity = 0.846f;         // CO2
  els[3].id = 3;  els[3].state = kStateGas;     els[3].molarMass = 0.028f;
  els[3].specificHeatCapacity = 1.040f;         // N2, held as vanilla mass
  std::vector<uint8_t> blob(4 + els.size() * sizeof(Element));
  const int32_t n = static_cast<int32_t>(els.size());
  memcpy(blob.data(), &n, 4);
  memcpy(blob.data() + 4, els.data(), els.size() * sizeof(Element));
  ElementTable table;
  if (!table.Load(blob.data(), blob.size())) {
    printf("  FAIL: synthetic ElementTable failed to load\n");
    ++g_fail_count;
    return;
  }

  World w;
  w.Allocate(3, 1);
  const size_t a = w.Padded(0);
  const size_t b = w.Padded(1);
  w.Phase(a).element = 0;
  w.Phase(a).temperature = 400.0f;
  w.Phase(b).element = 3;  // 0.5 kg of vanilla N2 shares b's temperature with the mixture
  w.Phase(b).mass = 0.5f;
  w.Phase(b).temperature = 250.0f;
  InjectSpecies(w, a, 1, 2.0f);
  InjectSpecies(w, b, 2, 2.0f);

  auto cell_energy = [&](size_t c) {
    double hc = w.Phase(c).mass > 0.0f
                    ? static_cast<double>(w.Phase(c).mass) *
                          table.At(w.Phase(c).element).specificHeatCapacity
                    : 0.0;
    for (int s = 0; s < kMaxSpeciesPerCell; ++s) {
      if (!(w.GasOccupiedMask(c) & (1u << s))) continue;
      hc += static_cast<double>(w.GasMass(c, s)) * table.At(w.GasSpecies(c, s)).specificHeatCapacity;
    }
    return hc * w.Phase(c).temperature;
  };
  const double before = cell_energy(a) + cell_energy(b);
  const double grid_before = GridCellEnergy(w, table, b);
  const double emitted_before = w.EnergyBooks().emitted;
  for (int i = 0; i < 200; ++i) MixPair(w, table, a, b, 0.2f);
  const double after = cell_energy(a) + cell_energy(b);
  const double booked = w.EnergyBooks().emitted - emitted_before;
  const double grid_delta = GridCellEnergy(w, table, b) - grid_before;

  printf("  a %.4f K, b %.4f K; m*c*T %.6f -> %.6f kJ (drift %.3e)\n", w.Phase(a).temperature,
         w.Phase(b).temperature, before, after, after - before);
  printf("  vanilla N2 in b: grid energy moved %.6f kJ, ledger booked %.6f kJ\n", grid_delta, booked);
  Check(std::fabs(after - before) < 1e-4 * before,
        "energy conserved: the pair's total m*c*T is unchanged after 200 steps of mixing");
  Check(w.Phase(a).temperature < 400.0f && w.Phase(b).temperature > 250.0f,
        "the heat moved: a cooled from 400 K as cold CO2 arrived, b warmed from 250 K as warm O2 "
        "arrived");
  Check(grid_delta > 0.0 && std::fabs(booked - grid_delta) < 1e-9 + 1e-6 * std::fabs(grid_delta),
        "the vanilla gas in b warmed with the mixture, and the ledger booked exactly that as "
        "emitted energy -- the mixture is outside the grid's books");
}

// Liquid and the mixture never share a cell. `MixPair` does not give to a cell holding
// liquid (and drains one that already has slots); `EvictMixture` moves a cell's slots into one
// gas neighbour of its room with their heat, or refuses and moves nothing; `SwapMixture`
// carries the slots with the PhaseEntry they share a temperature with. Without this the mixture
// would spread into a promoted room's pond cells and every liquid kernel would refuse them, so
// the pond, and any liquid resting on mixture gas, would stop moving.
ElementTable MakeLiquidMixtureTable() {
  std::vector<Element> els(5);
  memset(els.data(), 0, els.size() * sizeof(Element));
  els[0].id = 0;  els[0].state = kStateVacuum;
  els[1].id = 1;  els[1].state = kStateGas;     els[1].molarMass = 0.032f;
  els[1].specificHeatCapacity = 1.005f;         // O2
  els[2].id = 2;  els[2].state = kStateGas;     els[2].molarMass = 0.044f;
  els[2].specificHeatCapacity = 0.846f;         // CO2
  els[3].id = 3;  els[3].state = kStateLiquid;  els[3].specificHeatCapacity = 4.179f;  // water
  els[4].id = 4;  els[4].state = kStateSolid;   els[4].specificHeatCapacity = 0.8f;    // rock
  std::vector<uint8_t> blob(4 + els.size() * sizeof(Element));
  const int32_t n = static_cast<int32_t>(els.size());
  memcpy(blob.data(), &n, 4);
  memcpy(blob.data() + 4, els.data(), els.size() * sizeof(Element));
  ElementTable table;
  if (!table.Load(blob.data(), blob.size())) {
    printf("  FAIL: synthetic ElementTable failed to load\n");
    ++g_fail_count;
  }
  return table;
}

void TestLiquidMeetsMixture() {
  printf("\n=== liquid and the mixture never share a cell ===\n");
  ElementTable table = MakeLiquidMixtureTable();
  constexpr uint16_t kWater = 3, kRock = 4;
  auto slots_mass = [&](const World& w, size_t c) {
    double m = 0.0;
    for (int s = 0; s < kMaxSpeciesPerCell; ++s) {
      if (w.GasOccupiedMask(c) & (1u << s)) m += w.GasMass(c, s);
    }
    return m;
  };
  auto cell_energy = [&](const World& w, size_t c) {
    double hc = w.Phase(c).mass > 0.0f
                    ? static_cast<double>(w.Phase(c).mass) *
                          table.At(w.Phase(c).element).specificHeatCapacity
                    : 0.0;
    for (int s = 0; s < kMaxSpeciesPerCell; ++s) {
      if (!(w.GasOccupiedMask(c) & (1u << s))) continue;
      hc += static_cast<double>(w.GasMass(c, s)) *
            table.At(w.GasSpecies(c, s)).specificHeatCapacity;
    }
    return hc * w.Phase(c).temperature;
  };

  // MixPair: a water cell beside mixture gas is never given any, and gives back what it has.
  {
    World w;
    w.Allocate(2, 1);
    const size_t a = w.Padded(0), b = w.Padded(1);
    w.Phase(a).element = 0;
    w.Phase(a).temperature = 300.0f;
    w.Phase(b).element = kWater;
    w.Phase(b).mass = 0.5f;
    w.Phase(b).temperature = 280.0f;
    InjectSpecies(w, a, kO2, 2.0f);
    for (int i = 0; i < 50; ++i) MixPair(w, table, a, b, 0.2f);
    printf("  gas beside water: water cell slots mask 0x%02x after 50 steps\n",
           w.GasOccupiedMask(b));
    Check(w.GasOccupiedMask(b) == 0 && w.Phase(b).temperature == 280.0f,
          "MixPair never gives mixture gas to a cell holding liquid, and leaves its temperature");
    InjectSpecies(w, b, kCO2, 0.1f);
    const double total_before = slots_mass(w, a) + slots_mass(w, b);
    for (int i = 0; i < 50; ++i) MixPair(w, table, a, b, 0.2f);
    const int sb = FindSlot(w, b, kCO2);
    const float left = sb >= 0 ? w.GasMass(b, sb) : 0.0f;
    printf("  stray CO2 in the water cell: 0.1000 kg -> %.4f kg\n", left);
    Check(left < 0.1f && std::fabs(slots_mass(w, a) + slots_mass(w, b) - total_before) < 1e-6,
          "and a water cell that already carries slots gives them back, conserving mass");
  }

  // EvictMixture, into the first qualifying neighbour, heat and all.
  {
    World w;
    w.Allocate(3, 3);
    for (size_t c = 0; c < 9; ++c) w.Phase(w.Padded(c)).temperature = 290.0f;
    const size_t mid = w.Padded(4), up = w.Padded(7), left = w.Padded(3);
    w.Phase(mid).element = kWater;  // a liquid cell carrying stray slots
    w.Phase(mid).mass = 10.0f;
    w.Phase(mid).temperature = 300.0f;
    InjectSpecies(w, mid, kO2, 1.0f);
    InjectSpecies(w, mid, kCO2, 0.5f);
    w.Phase(up).temperature = 250.0f;
    InjectSpecies(w, up, kO2, 0.2f);
    RoomGraph g = BuildRoomGraph(w, table, 3, 3);
    w.MutableGasSleeping(up) = 1;
    RoomOnCellSlept(g, up);
    const int32_t awake_before = g.awake_count[static_cast<size_t>(g.room_of[up])];
    const double mass_before = slots_mass(w, mid) + slots_mass(w, up);
    const double energy_before = cell_energy(w, mid) + cell_energy(w, up);
    const double emitted_before = w.EnergyBooks().emitted;
    const bool ok = EvictMixture(w, table, &g, mid, 0);
    const double energy_after = cell_energy(w, mid) + cell_energy(w, up);
    printf("  evict up: ok %d, mid mask 0x%02x, up O2 %.4f CO2 %.4f kg at %.4f K; "
           "m*c*T %.6f -> %.6f kJ\n",
           ok, w.GasOccupiedMask(mid), w.GasMass(up, FindSlot(w, up, kO2)),
           w.GasMass(up, FindSlot(w, up, kCO2)), w.Phase(up).temperature, energy_before,
           energy_after);
    Check(ok && w.GasOccupiedMask(mid) == 0 && w.Phase(mid).temperature == 300.0f &&
              std::fabs(slots_mass(w, up) - mass_before) < 1e-6,
          "EvictMixture moves every slot to the neighbour above (rotation 0), all the mass, "
          "and leaves the liquid's temperature alone");
    Check(std::fabs(energy_after - energy_before) < 1e-9 * energy_before &&
              w.Phase(up).temperature > 250.0f && w.Phase(up).temperature < 300.0f,
          "the heat goes with the gas: the pair's m*c*T is unchanged and the target settles "
          "between the two temperatures");
    Check(w.EnergyBooks().emitted == emitted_before,
          "a target holding no vanilla mass books nothing on the grid's ledger");
    Check(w.GasSleeping(up) == 0 &&
              g.awake_count[static_cast<size_t>(g.room_of[up])] == awake_before + 1,
          "the target is woken and the room's awake count follows");

    // Second eviction, rotation 0 again, but the cell above now holds water: the next
    // orthogonal in `DisplaceGas`'s order (left) takes it.
    w.Phase(up).element = kWater;
    w.Phase(up).mass = 5.0f;
    InjectSpecies(w, mid, kCO2, 0.3f);
    const bool ok2 = EvictMixture(w, table, &g, mid, 0);
    Check(ok2 && FindSlot(w, left, kCO2) >= 0 && w.GasOccupiedMask(mid) == 0,
          "a neighbour holding liquid is passed over and the next one in DisplaceGas's order "
          "takes the gas");
  }

  // Refusal: a one-cell room has no neighbour to take the gas, so nothing moves.
  {
    World w;
    w.Allocate(3, 3);
    for (size_t c = 0; c < 9; ++c) {
      w.Phase(w.Padded(c)).element = kRock;
      w.Phase(w.Padded(c)).mass = 100.0f;
      w.Phase(w.Padded(c)).temperature = 290.0f;
    }
    const size_t mid = w.Padded(4);
    w.Phase(mid).element = kWater;
    w.Phase(mid).mass = 10.0f;
    InjectSpecies(w, mid, kO2, 1.0f);
    RoomGraph g = BuildRoomGraph(w, table, 3, 3);
    const bool ok = EvictMixture(w, table, &g, mid, 0);
    Check(!ok && std::fabs(slots_mass(w, mid) - 1.0) < 1e-9,
          "with nowhere to go EvictMixture refuses and the slots stay exactly as they were");
  }

  // SwapMixture: the slots travel with the PhaseEntry.
  {
    World w;
    w.Allocate(1, 2);
    const size_t lo = w.Padded(0), hi = w.Padded(1);
    InjectSpecies(w, lo, kO2, 0.4f);
    w.MutableGasSleeping(lo) = 1;
    SwapMixture(w, lo, hi);
    Check(w.GasOccupiedMask(lo) == 0 && FindSlot(w, hi, kO2) >= 0 &&
              w.GasMass(hi, FindSlot(w, hi, kO2)) == 0.4f && w.GasSleeping(hi) == 1 &&
              w.GasSleeping(lo) == 0,
          "SwapMixture exchanges the slots and the sleep flags of the two cells");
  }
}

// Two DIFFERENT species, each starting on opposite sides, should each equalize
// independently (Dalton's law) rather than the mixture behaving as one lump.
void TestTwoSpeciesIndependent() {
  printf("\n=== two independent species, opposite starting sides ===\n");
  ElementTable table = MakeTestTable();
  World w;
  w.Allocate(3, 1);
  const size_t a = w.Padded(0);
  const size_t b = w.Padded(1);
  w.Phase(a).temperature = 300.0f;
  w.Phase(b).temperature = 300.0f;

  InjectSpecies(w, a, kO2, 10.0f);   // only in a
  InjectSpecies(w, b, kCO2, 10.0f);  // only in b

  for (int i = 0; i < 200; ++i) MixPair(w, table, a, b, 0.2f);

  const int o2_a = FindSlot(w, a, kO2), o2_b = FindSlot(w, b, kO2);
  const int co2_a = FindSlot(w, a, kCO2), co2_b = FindSlot(w, b, kCO2);
  const float o2_total = (o2_a >= 0 ? w.GasMass(a, o2_a) : 0.0f) +
                          (o2_b >= 0 ? w.GasMass(b, o2_b) : 0.0f);
  const float co2_total = (co2_a >= 0 ? w.GasMass(a, co2_a) : 0.0f) +
                           (co2_b >= 0 ? w.GasMass(b, co2_b) : 0.0f);

  printf("  O2:  a=%.4f b=%.4f (total %.4f)\n",
         o2_a >= 0 ? w.GasMass(a, o2_a) : 0.0f, o2_b >= 0 ? w.GasMass(b, o2_b) : 0.0f,
         o2_total);
  printf("  CO2: a=%.4f b=%.4f (total %.4f)\n",
         co2_a >= 0 ? w.GasMass(a, co2_a) : 0.0f, co2_b >= 0 ? w.GasMass(b, co2_b) : 0.0f,
         co2_total);

  Check(std::fabs(o2_total - 10.0f) < 1e-4f, "O2 mass conserved");
  Check(std::fabs(co2_total - 10.0f) < 1e-4f, "CO2 mass conserved");
  Check(o2_a >= 0 && o2_b >= 0 && std::fabs(w.GasMass(a, o2_a) - w.GasMass(b, o2_b)) < 0.5f,
        "O2 crossed over and equalized independently of CO2");
  Check(co2_a >= 0 && co2_b >= 0 &&
        std::fabs(w.GasMass(a, co2_a) - w.GasMass(b, co2_b)) < 0.5f,
        "CO2 crossed over and equalized independently of O2");
  // Both species ending up on both sides is the actual point: real gas mixing, not
  // vanilla's one-element-per-tile swap.
  Check(w.GasOccupiedMask(a) != 0 && (w.GasOccupiedMask(a) & (w.GasOccupiedMask(a) - 1)) != 0,
        "cell a ends up holding two distinct species at once (occupied_mask has 2 bits set)");
}

// A cold cell and a hot cell, same species mass, should NOT show equal pressure — this is
// the sanity check that CellPressure is actually reading temperature, not just mass.
void TestTemperatureAffectsPressure() {
  printf("\n=== same mass, different temperature ===\n");
  ElementTable table = MakeTestTable();
  World w;
  w.Allocate(2, 1);
  const size_t a = w.Padded(0);
  const size_t b = w.Padded(1);
  w.Phase(a).temperature = 400.0f;
  w.Phase(b).temperature = 200.0f;
  InjectSpecies(w, a, kO2, 5.0f);
  InjectSpecies(w, b, kO2, 5.0f);

  const float pa = CellPressure(w, table, a);
  const float pb = CellPressure(w, table, b);
  printf("  hot cell (400K): %.4f Pa, cold cell (200K): %.4f Pa\n", pa, pb);
  Check(pa > pb, "hot cell reads higher pressure than cold cell at equal mass (PV=nRT)");
}

// Total mass across every occupied slot in the grid — the invariant both the naive and the
// dirty/sleep sweep must preserve exactly (mixing only ever moves mass between cells, never
// creates or destroys it).
double TotalGasMass(const World& w, int32_t width, int32_t height) {
  double total = 0.0;
  for (int32_t y = 0; y < height; ++y) {
    for (int32_t x = 0; x < width; ++x) {
      const size_t p = w.Padded(static_cast<size_t>(y) * width + x);
      const uint8_t mask = w.GasOccupiedMask(p);
      for (int s = 0; s < kMaxSpeciesPerCell; ++s) {
        if (mask & (1u << s)) total += w.GasMass(p, s);
      }
    }
  }
  return total;
}

// Seeds a `width` x `height` grid at vacuum/300K, then injects a burst of O2 into a small
// square in the middle: an empty map with one large injected mass.
void SeedInjectionMap(World& w, int32_t width, int32_t height) {
  w.Allocate(width, height);
  for (int32_t y = 0; y < height; ++y) {
    for (int32_t x = 0; x < width; ++x) {
      w.Phase(w.Padded(static_cast<size_t>(y) * width + x)).temperature = 300.0f;
    }
  }
  const int32_t cx = width / 2, cy = height / 2;
  for (int32_t y = cy - 5; y < cy + 5; ++y) {
    for (int32_t x = cx - 5; x < cx + 5; ++x) {
      if (x < 0 || y < 0 || x >= width || y >= height) continue;
      InjectSpecies(w, w.Padded(static_cast<size_t>(y) * width + x), kO2, 100.0f);
    }
  }
}

// Step-4 perf pass. Runs the same injection-map scenario through the naive full sweep and
// through the dirty/sleep-aware sweep, times both, and reports the speedup, measured rather than
// assumed.
// Sleep policy lives here, not in gas_mixture.h: a cell that produced no dirty flag for
// `kSleepThreshold` consecutive ticks is put to sleep; MixPair wakes one immediately the
// moment a real transfer touches it. This threshold is a first guess to tune once there's a
// reason to, not a frozen constant.
void RunPerfBenchmark(int32_t width, int32_t height, int ticks) {
  constexpr int kSleepThreshold = 5;
  constexpr float kRate = 0.2f;
  const size_t cells = static_cast<size_t>(width) * height;
  printf("\n=== step-4 perf: %dx%d grid (%zu cells), %d ticks, rate=%.2f ===\n", width,
         height, cells, ticks, kRate);

  ElementTable table = MakeTestTable();

  // Naive: unconditional full sweep every tick.
  World naive;
  SeedInjectionMap(naive, width, height);
  const double mass_before = TotalGasMass(naive, width, height);
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < ticks; ++i) MixGridNaive(naive, table, width, height, kRate);
  const auto t1 = std::chrono::steady_clock::now();
  const double naive_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  const double mass_after_naive = TotalGasMass(naive, width, height);

  // Dirty/sleep: same scenario, same rate, but cells that go `kSleepThreshold` ticks
  // without a dirty flag are parked and skipped until a neighbor wakes them.
  World lazy;
  SeedInjectionMap(lazy, width, height);
  std::vector<uint8_t> stable(cells, 0);
  const auto t2 = std::chrono::steady_clock::now();
  for (int i = 0; i < ticks; ++i) {
    for (size_t c = 0; c < cells; ++c) lazy.MutableGasDirty(lazy.Padded(c)) = 0;
    MixGridDirtySleep(lazy, table, width, height, kRate);
    for (size_t c = 0; c < cells; ++c) {
      const size_t p = lazy.Padded(c);
      if (lazy.GasDirty(p)) {
        stable[c] = 0;
      } else if (stable[c] < 255) {
        if (++stable[c] >= kSleepThreshold) lazy.MutableGasSleeping(p) = 1;
      }
    }
  }
  const auto t3 = std::chrono::steady_clock::now();
  const double lazy_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();
  const double mass_after_lazy = TotalGasMass(lazy, width, height);

  size_t sleeping_count = 0;
  for (size_t c = 0; c < cells; ++c) {
    if (lazy.GasSleeping(lazy.Padded(c))) ++sleeping_count;
  }

  printf("  naive:       %.2f ms total, %.4f ms/tick\n", naive_ms, naive_ms / ticks);
  printf("  dirty/sleep: %.2f ms total, %.4f ms/tick (%.1f%% of cells asleep by the end)\n",
         lazy_ms, lazy_ms / ticks, 100.0 * static_cast<double>(sleeping_count) / cells);
  printf("  speedup: %.2fx\n", lazy_ms > 0.0 ? naive_ms / lazy_ms : 0.0);

  Check(std::fabs(mass_after_naive - mass_before) < 1e-2, "naive sweep conserves mass");
  Check(std::fabs(mass_after_lazy - mass_before) < 1e-2, "dirty/sleep sweep conserves mass");
  Check(std::fabs(mass_after_lazy - mass_after_naive) < 1.0,
        "dirty/sleep and naive reach the same final mass distribution (within tolerance)");
}

// Partitions a grid into a checkerboard of rooms via 1-cell-thick walls
// (kGasImpermeable), each wall carrying exactly one doorway cell so rooms are connected but
// only through a narrow gap — a burst injected into one room's interior takes many ticks to
// reach a distant room inside a short test window, if it reaches one at all. `room_size` is
// each room's interior side length; a wall column/row is inserted after every `room_size`
// cells.
void BuildWalledTestMap(World& w, int32_t width, int32_t height, int32_t room_size) {
  w.Allocate(width, height);
  for (int32_t y = 0; y < height; ++y) {
    for (int32_t x = 0; x < width; ++x) {
      w.Phase(w.Padded(static_cast<size_t>(y) * width + x)).temperature = 300.0f;
    }
  }
  for (int32_t x = room_size; x < width; x += room_size + 1) {
    const int32_t doorway_y = (x / (room_size + 1)) % height;
    for (int32_t y = 0; y < height; ++y) {
      if (y == doorway_y) continue;
      w.MutableProperties(w.Padded(static_cast<size_t>(y) * width + x)) |= kGasImpermeable;
    }
  }
  for (int32_t y = room_size; y < height; y += room_size + 1) {
    const int32_t doorway_x = (y / (room_size + 1)) % width;
    for (int32_t x = 0; x < width; ++x) {
      if (x == doorway_x) continue;
      w.MutableProperties(w.Padded(static_cast<size_t>(y) * width + x)) |= kGasImpermeable;
    }
  }
}

// Room perf pass: same injection-burst idea as RunPerfBenchmark, but on a
// walled, multi-room grid, comparing three sweeps that all walk the identical wall-respecting
// pair set (RoomGraph's interior_pairs + boundary_pairs) so the only variable is *how much of
// that set each sweep bothers to visit*:
//   flat        - every pair, every tick, unconditionally (the walled equivalent of the naive
//                 baseline)
//   dirty/sleep - the per-pair "skip if both ends already asleep" technique, no room
//                 grouping
//   room-pooled - gas_rooms.h's MixRoomPooled: skips a fully-asleep room's whole interior
//                 block via one awake_count read, per-pair skip still applies within an
//                 active room
void RunRoomPerfBenchmark(int32_t width, int32_t height, int32_t room_size, int ticks) {
  constexpr int kSleepThreshold = 5;
  constexpr float kRate = 0.2f;
  printf("\n=== step-4b perf: %dx%d grid, room_size=%d, %d ticks ===\n", width, height,
         room_size, ticks);

  ElementTable table = MakeTestTable();

  auto seed = [&](World& w) {
    BuildWalledTestMap(w, width, height, room_size);
    const int32_t cx = room_size / 2, cy = room_size / 2;
    for (int32_t y = cy - 5; y < cy + 5; ++y) {
      for (int32_t x = cx - 5; x < cx + 5; ++x) {
        if (x < 0 || y < 0 || x >= width || y >= height) continue;
        InjectSpecies(w, w.Padded(static_cast<size_t>(y) * width + x), kO2, 100.0f);
      }
    }
  };
  const size_t cells = static_cast<size_t>(width) * height;

  // flat: unconditional walk of the room graph's own pair list, every tick.
  World flat;
  seed(flat);
  RoomGraph flat_graph = BuildRoomGraph(flat, table, width, height);
  const double mass_before_flat = TotalGasMass(flat, width, height);
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < ticks; ++i) {
    for (const auto& pr : flat_graph.interior_pairs) MixPair(flat, table, pr.a, pr.b, kRate);
    for (const auto& pr : flat_graph.boundary_pairs) MixPair(flat, table, pr.a, pr.b, kRate);
  }
  const auto t1 = std::chrono::steady_clock::now();
  const double flat_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  const double mass_after_flat = TotalGasMass(flat, width, height);

  // dirty/sleep: same pair list, per-pair skip when both ends are already asleep.
  World lazy;
  seed(lazy);
  RoomGraph lazy_graph = BuildRoomGraph(lazy, table, width, height);
  std::vector<uint8_t> lazy_stable(cells, 0);
  const double mass_before_lazy = TotalGasMass(lazy, width, height);
  const auto t2 = std::chrono::steady_clock::now();
  for (int i = 0; i < ticks; ++i) {
    for (size_t c = 0; c < cells; ++c) lazy.MutableGasDirty(lazy.Padded(c)) = 0;
    auto run_pair = [&](size_t a, size_t b) {
      if (lazy.GasSleeping(a) && lazy.GasSleeping(b)) return;
      MixPair(lazy, table, a, b, kRate);
    };
    for (const auto& pr : lazy_graph.interior_pairs) run_pair(pr.a, pr.b);
    for (const auto& pr : lazy_graph.boundary_pairs) run_pair(pr.a, pr.b);
    for (size_t c = 0; c < cells; ++c) {
      const size_t p = lazy.Padded(c);
      if (lazy.GasDirty(p)) {
        lazy_stable[c] = 0;
      } else if (lazy_stable[c] < 255) {
        if (++lazy_stable[c] >= kSleepThreshold) lazy.MutableGasSleeping(p) = 1;
      }
    }
  }
  const auto t3 = std::chrono::steady_clock::now();
  const double lazy_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();
  const double mass_after_lazy = TotalGasMass(lazy, width, height);

  // room-pooled: MixRoomPooled, skipping whole sleeping rooms via awake_count.
  World pooled;
  seed(pooled);
  RoomGraph pooled_graph = BuildRoomGraph(pooled, table, width, height);
  std::vector<uint8_t> pooled_stable(cells, 0);
  const double mass_before_pooled = TotalGasMass(pooled, width, height);
  const auto t4 = std::chrono::steady_clock::now();
  for (int i = 0; i < ticks; ++i) {
    for (size_t c = 0; c < cells; ++c) pooled.MutableGasDirty(pooled.Padded(c)) = 0;
    MixRoomPooled(pooled, table, pooled_graph, kRate);
    for (size_t c = 0; c < cells; ++c) {
      const size_t p = pooled.Padded(c);
      if (pooled.GasDirty(p)) {
        pooled_stable[c] = 0;
      } else if (pooled_stable[c] < 255) {
        if (++pooled_stable[c] >= kSleepThreshold) {
          if (!pooled.GasSleeping(p)) RoomOnCellSlept(pooled_graph, p);
          pooled.MutableGasSleeping(p) = 1;
        }
      }
    }
  }
  const auto t5 = std::chrono::steady_clock::now();
  const double pooled_ms = std::chrono::duration<double, std::milli>(t5 - t4).count();
  const double mass_after_pooled = TotalGasMass(pooled, width, height);

  size_t rooms_asleep = 0;
  for (int32_t r = 0; r < pooled_graph.RoomCount(); ++r) {
    if (pooled_graph.awake_count[static_cast<size_t>(r)] == 0) ++rooms_asleep;
  }

  printf("  rooms: %d\n", pooled_graph.RoomCount());
  printf("  flat (walled, unconditional):           %.2f ms total, %.4f ms/tick\n", flat_ms,
         flat_ms / ticks);
  printf("  dirty/sleep (walled, no room grouping):  %.2f ms total, %.4f ms/tick\n", lazy_ms,
         lazy_ms / ticks);
  printf("  room-pooled:                             %.2f ms total, %.4f ms/tick"
         " (%zu/%d rooms fully asleep by the end)\n",
         pooled_ms, pooled_ms / ticks, rooms_asleep, pooled_graph.RoomCount());
  printf("  speedup vs flat: dirty/sleep %.2fx, room-pooled %.2fx\n",
         lazy_ms > 0.0 ? flat_ms / lazy_ms : 0.0, pooled_ms > 0.0 ? flat_ms / pooled_ms : 0.0);

  Check(std::fabs(mass_after_flat - mass_before_flat) < 1e-2,
        "flat walled sweep conserves mass");
  Check(std::fabs(mass_after_lazy - mass_before_lazy) < 1e-2,
        "dirty/sleep walled sweep conserves mass");
  Check(std::fabs(mass_after_pooled - mass_before_pooled) < 1e-2,
        "room-pooled sweep conserves mass");
  Check(std::fabs(mass_after_pooled - mass_after_flat) < 1.0,
        "room-pooled and flat reach the same final mass distribution (within tolerance)");
  Check(rooms_asleep > 0, "at least one room goes fully asleep and gets skipped whole");
}

// Count of cells holding any gas at all — a spread proxy for how far the injected burst has
// propagated by the end of a run. Not a precision metric, just enough to see whether a slower
// mixing cadence visibly lags equalization or not.
size_t OccupiedCellCount(const World& w, int32_t width, int32_t height) {
  size_t n = 0;
  for (int32_t y = 0; y < height; ++y) {
    for (int32_t x = 0; x < width; ++x) {
      if (w.GasOccupiedMask(w.Padded(static_cast<size_t>(y) * width + x)) != 0) ++n;
    }
  }
  return n;
}

// Multi-rate ticking. Same walled/room-pooled scenario as RunRoomPerfBenchmark, but the whole
// MixRoomPooled pass is gated by TickRates::mixing_every_n_ticks via ShouldTick
// (abi/gas_mixture_abi.h) instead of running every tick. A tick the gate skips does zero
// work — no array reset, no bookkeeping — matching the literal meaning of "this subsystem
// runs at a slower cadence," not a rescan that happens to find nothing changed.
//
// Only mixing_every_n_ticks is exercised: convection_every_n_ticks/radiant_every_n_ticks have
// no kernel in this codebase yet (see ShouldTick's doc comment) — building stub kernels just
// to demonstrate the gate would be inventing work ahead of the plan.
//
// The finding that matters isn't "fewer passes is faster" — that's arithmetic, not a
// measurement. It's what a slower cadence actually costs in spread (how far the injected
// burst has propagated by the end), alongside the speed number, on the same scenario
// RunRoomPerfBenchmark already validated for correctness.
void RunMultiRateBenchmark(int32_t width, int32_t height, int32_t room_size, int ticks) {
  constexpr int kSleepThreshold = 5;
  constexpr float kRate = 0.2f;
  printf(
      "\n=== step-4 perf: multi-rate mixing cadence, %dx%d grid, %d ticks ===\n",
      width, height, ticks);

  ElementTable table = MakeTestTable();
  auto seed = [&](World& w) {
    BuildWalledTestMap(w, width, height, room_size);
    const int32_t cx = room_size / 2, cy = room_size / 2;
    for (int32_t y = cy - 5; y < cy + 5; ++y) {
      for (int32_t x = cx - 5; x < cx + 5; ++x) {
        if (x < 0 || y < 0 || x >= width || y >= height) continue;
        InjectSpecies(w, w.Padded(static_cast<size_t>(y) * width + x), kO2, 100.0f);
      }
    }
  };
  const size_t cells = static_cast<size_t>(width) * height;
  const uint8_t rates[] = {1, 2, 4, 8};
  double ms_at_rate1 = 0.0, ms_at_rate8 = 0.0;

  printf("  %-12s %12s %16s %8s\n", "mixing rate", "ms/tick", "spread (cells)", "mass");
  for (uint8_t every_n : rates) {
    World w;
    seed(w);
    RoomGraph g = BuildRoomGraph(w, table, width, height);
    std::vector<uint8_t> stable(cells, 0);
    const double mass_before = TotalGasMass(w, width, height);
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < ticks; ++i) {
      if (!ShouldTick(static_cast<uint64_t>(i), every_n)) continue;
      for (size_t c = 0; c < cells; ++c) w.MutableGasDirty(w.Padded(c)) = 0;
      MixRoomPooled(w, table, g, kRate);
      for (size_t c = 0; c < cells; ++c) {
        const size_t p = w.Padded(c);
        if (w.GasDirty(p)) {
          stable[c] = 0;
        } else if (stable[c] < 255) {
          if (++stable[c] >= kSleepThreshold) {
            if (!w.GasSleeping(p)) RoomOnCellSlept(g, p);
            w.MutableGasSleeping(p) = 1;
          }
        }
      }
    }
    const auto t1 = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    const double mass_after = TotalGasMass(w, width, height);
    const size_t spread = OccupiedCellCount(w, width, height);
    const bool mass_ok = std::fabs(mass_after - mass_before) < 1e-2;

    printf("  every_n=%-4u %12.4f %16zu %8s\n", every_n, ms / ticks, spread,
           mass_ok ? "ok" : "BAD");
    Check(mass_ok, "multi-rate sweep conserves mass at this cadence");

    if (every_n == 1) ms_at_rate1 = ms;
    if (every_n == 8) ms_at_rate8 = ms;
  }

  printf("  every_n=8 vs every_n=1: %.2fx wall time\n",
         ms_at_rate1 > 0.0 ? ms_at_rate8 / ms_at_rate1 : 0.0);
  Check(ms_at_rate8 < ms_at_rate1,
        "an 8x slower mixing cadence costs less wall time than every-tick");
}

// =========================================================================================
// THE PUBLISHED MESSAGE SURFACE, AND ITS DRIFT GUARD.
//
// This arm is in gastest rather than vftest ON PURPOSE, and the reason is a constraint
// rather than a preference. `vftest` and `bench` both take `--corpus` as a mandatory
// argument and a corpus is recorded from a running game, so neither can run on a machine without
// the game. `gastest` compiles the sim's kernels into itself and reads nothing, which makes it
// the ONE suite any machine can run -- and the message table is compile-
// time, so checking it needs no game content either. Putting the guard anywhere else would
// have made it a check only a machine that owns the game ever runs.
//
// `vftest` holds the other half, which genuinely does need a DLL: that the table is actually
// EXPORTED, that the descriptors round-trip through the ABI boundary, and that the refusals
// leave the caller's buffer alone. This half is the table itself.
//
// The literal list below is TYPED OUT BY HAND. Expanding ONI_EXT_MESSAGE_LIST here would
// make the arm pass by construction and prove nothing -- the same standard the phase
// guard is held to. Written out independently it is a second opinion, and a message whose
// delivery is changed in the header without anyone deciding to change the published surface
// fails here with the offending row named.
//
// If this fails after a genuine, intended change, the fix is to update BOTH this list and
// ONI_EXT_MESSAGE_LIST -- and to treat a `delivery` change as the ABI break it is, because a
// caller's code shape depends on it: a mod that reads back in the same tick works against
// IMMEDIATE and silently reads a stale value against QUEUED.
void TestExtMessageTable() {
  printf("--- published message surface ---\n");
  namespace ext = oni_sim::ext;
  struct Expect {
    const char* name;
    int32_t id;
    int32_t messageClass;
    int32_t delivery;
    int32_t phase;
  };
  static const Expect kExpected[] = {
      {"kSetCellThermalMassBonus", 0x4F4E4931, ext::kMessageClassParameter,
       ext::kDeliveryQueued, ext::kPhaseConduction},
      {"kInjectGasSpecies", 0x4F4E4932, ext::kMessageClassOperation, ext::kDeliveryQueued,
       ext::kPhaseDrain},
      {"kRemoveVanillaMass", 0x4F4E4933, ext::kMessageClassOperation, ext::kDeliveryQueued,
       ext::kPhaseDrain},
      {"kConvertToVanillaMass", 0x4F4E4934, ext::kMessageClassOperation, ext::kDeliveryQueued,
       ext::kPhaseDrain},
      {"kPromoteRoom", 0x4F4E4935, ext::kMessageClassParameter, ext::kDeliveryQueued,
       ext::kPhaseUnpublished},
      {"kSetInvertedGravityElement", 0x4F4E4936, ext::kMessageClassParameter,
       ext::kDeliveryQueued, ext::kPhaseFlow},
      {"kSetMolecularMass", 0x4F4E4937, ext::kMessageClassParameter, ext::kDeliveryImmediate,
       ext::kPhaseUnpublished},
      {"kSetBuildingWasteHeatKilowatts", 0x4F4E4938, ext::kMessageClassParameter,
       ext::kDeliveryQueued, ext::kPhaseBuildingHeat},
      {"kSetBuildingExhaust", 0x4F4E4939, ext::kMessageClassParameter, ext::kDeliveryQueued,
       ext::kPhaseBuildingHeat},
      {"kSetBuildingRadiation", 0x4F4E493A, ext::kMessageClassParameter, ext::kDeliveryQueued,
       ext::kPhaseBuildingHeat},
      {"kSetEnvironmentTemperature", 0x4F4E493B, ext::kMessageClassParameter,
       ext::kDeliveryQueued, ext::kPhaseBuildingHeat},
      {"kSetRandomState", 0x4F4E493C, ext::kMessageClassCheckpoint, ext::kDeliveryQueued,
       ext::kPhaseUnpublished},
      {"kSetSchedulingState", 0x4F4E493D, ext::kMessageClassCheckpoint, ext::kDeliveryQueued,
       ext::kPhaseUnpublished},
      {"kSetStableTicks", 0x4F4E493E, ext::kMessageClassCheckpoint, ext::kDeliveryQueued,
       ext::kPhasePostProcess},
      {"kSetDiseaseGrowth", 0x4F4E493F, ext::kMessageClassCheckpoint, ext::kDeliveryQueued,
       ext::kPhaseDiseasePostProcess},
      {"kSetRegistryState", 0x4F4E4940, ext::kMessageClassCheckpoint, ext::kDeliveryQueued,
       ext::kPhaseUnpublished},
      {"kRegisterCellProperty", 0x4F4E4941, ext::kMessageClassStore, ext::kDeliveryImmediate,
       ext::kPhaseUnpublished},
      {"kSetCellProperty", 0x4F4E4942, ext::kMessageClassStore, ext::kDeliveryQueued,
       ext::kPhaseUnpublished},
      {"kSetExtCellState", 0x4F4E4943, ext::kMessageClassCheckpoint, ext::kDeliveryQueued,
       ext::kPhaseUnpublished},
      {"kPublishCellProperty", 0x4F4E4944, ext::kMessageClassStore, ext::kDeliveryQueued,
       ext::kPhaseProject},
      {"kSubscribeEventStream", 0x4F4E4945, ext::kMessageClassStore, ext::kDeliveryQueued,
       ext::kPhaseUnpublished},
      {"kRegisterElementAttribute", 0x4F4E4946, ext::kMessageClassStore,
       ext::kDeliveryImmediate, ext::kPhaseUnpublished},
      {"kSetElementAttribute", 0x4F4E4947, ext::kMessageClassStore, ext::kDeliveryImmediate,
       ext::kPhaseUnpublished},
      {"kSetBuildingConvection", 0x4F4E4948, ext::kMessageClassParameter, ext::kDeliveryQueued,
       ext::kPhaseBuildingHeat},
      {"kSetWorldEnvironment", 0x4F4E4949, ext::kMessageClassParameter, ext::kDeliveryQueued,
       ext::kPhaseUnpublished},
      {"kSetWorldSun", 0x4F4E494A, ext::kMessageClassParameter, ext::kDeliveryQueued,
       ext::kPhaseUnpublished},
      {"kSetCellRadiation", 0x4F4E494B, ext::kMessageClassCheckpoint, ext::kDeliveryImmediate,
       ext::kPhaseRadiationField},
      {"kSetBlockedGasAddPolicy", 0x4F4E494C, ext::kMessageClassParameter, ext::kDeliveryQueued,
       ext::kPhaseDrain},
      {"kSetCellPropertyTransport", 0x4F4E494D, ext::kMessageClassStore, ext::kDeliveryQueued,
       ext::kPhaseUnpublished},
      {"kAddCellPropertyAmount", 0x4F4E494E, ext::kMessageClassStore, ext::kDeliveryQueued,
       ext::kPhaseUnpublished},
      // The generic field solver. The registration is immediate and returns the
      // field index, like the other two registrations; the source is a parameter of a phase,
      // so it is queued and names the phase that reads it.
      {"kRegisterField", 0x4F4E494F, ext::kMessageClassStore, ext::kDeliveryImmediate,
       ext::kPhaseUnpublished},
      {"kSetFieldSource", 0x4F4E4950, ext::kMessageClassParameter, ext::kDeliveryQueued,
       ext::kPhaseFields},
      // Payload mixing. A parameter of one phase, so it is
      // queued and names the phase that reads it, exactly as kSetFieldSource does.
      {"kSetPayloadMixing", 0x4F4E4951, ext::kMessageClassParameter, ext::kDeliveryQueued,
       ext::kPhasePayloadMixing},
      // 0x4F4E4952 "ONIR" is NOT here: it is reserved for a message not in this release.
      // The dissolved tint. A handler without a row is refused by the dispatcher; read by the
      // texture fill, which runs in Project.
      {"kSetDissolvedTint", 0x4F4E4953, ext::kMessageClassParameter, ext::kDeliveryQueued,
       ext::kPhaseProject},
      // 0x4F4E4954 "ONIT" is NOT here: it is reserved for a message not in this release.
      // Layer C3 effervescence, a parameter of its own phase.
      {"kSetEffervescence", 0x4F4E4955, ext::kMessageClassParameter, ext::kDeliveryQueued,
       ext::kPhaseEffervescence},
      // Tunables: immediate, so `SubstepSeconds` can be set while no world exists (there is
      // no drain without a frame), and so it can return its result. Read by every phase.
      {"kSetTunable", 0x4F4E4956, ext::kMessageClassParameter, ext::kDeliveryImmediate,
       ext::kPhaseUnpublished},
      // The ninth checkpoint component, the visibility mask. Immediate for the reason
      // kSetCellRadiation is: the next PrepareGameData writes a buffer this restores, before
      // a queued message would drain.
      {"kSetVisibilityState", 0x4F4E4957, ext::kMessageClassCheckpoint, ext::kDeliveryImmediate,
       ext::kPhaseUnpublished},
      // The next Load is a checkpoint restore. Immediate because it is sent BEFORE the
      // Load, which is handled on the calling thread; a queued one would drain after it.
      {"kSetLoadIsRestore", 0x4F4E4958, ext::kMessageClassCheckpoint, ext::kDeliveryImmediate,
       ext::kPhaseUnpublished},
  };
  const int32_t expected_n = static_cast<int32_t>(sizeof(kExpected) / sizeof(kExpected[0]));

  // The macro's own row order and contents, expanded here so the two can be compared. This
  // expansion is the SUBJECT of the check, not the check -- kExpected above is the opinion.
  struct Row {
    const char* name;
    int32_t id;
    int32_t messageClass;
    int32_t delivery;
    int32_t phase;
  };
  static const Row kRows[] = {
#define ONI_EXT_MESSAGE_TESTROW(e, n, i, c, d, p) {n, ext::i, ext::c, ext::d, ext::p},
      ONI_EXT_MESSAGE_LIST(ONI_EXT_MESSAGE_TESTROW)
#undef ONI_EXT_MESSAGE_TESTROW
  };
  const int32_t actual_n = static_cast<int32_t>(sizeof(kRows) / sizeof(kRows[0]));

  Check(actual_n == expected_n,
        "ONI_EXT_MESSAGE_LIST has the number of rows this test independently expects -- an "
        "extension message added without publishing its delivery fails here");
  Check(actual_n == ext::kExtMessageCount,
        "and kExtMessageCount agrees with the row count it is generated from");
  printf("    (header declares %d messages, this test expects %d)\n", actual_n, expected_n);

  bool names_ok = true, ids_ok = true, class_ok = true, delivery_ok = true, phase_ok = true;
  for (int32_t i = 0; i < expected_n && i < actual_n; ++i) {
    if (strcmp(kRows[i].name, kExpected[i].name) != 0) {
      printf("    message %d: header says \"%s\", this test expects \"%s\"\n", i,
             kRows[i].name, kExpected[i].name);
      names_ok = false;
    }
    if (kRows[i].id != kExpected[i].id) {
      printf("    message %d (%s): id 0x%08X, expected 0x%08X\n", i, kRows[i].name,
             kRows[i].id, kExpected[i].id);
      ids_ok = false;
    }
    if (kRows[i].messageClass != kExpected[i].messageClass) {
      printf("    message %d (%s): class %d, expected %d\n", i, kRows[i].name,
             kRows[i].messageClass, kExpected[i].messageClass);
      class_ok = false;
    }
    if (kRows[i].delivery != kExpected[i].delivery) {
      printf("    message %d (%s): delivery %d, expected %d\n", i, kRows[i].name,
             kRows[i].delivery, kExpected[i].delivery);
      delivery_ok = false;
    }
    if (kRows[i].phase != kExpected[i].phase) {
      printf("    message %d (%s): phase %d, expected %d\n", i, kRows[i].name, kRows[i].phase,
             kExpected[i].phase);
      phase_ok = false;
    }
  }
  Check(names_ok, "every message name matches, in order");
  Check(ids_ok,
        "every wire id matches -- the id is what SIM_HandleMessage takes, so this is the "
        "field a mod compiled against an older header would disagree with");
  Check(class_ok,
        "every class matches: eight of these are the checkpoint ABI and extend nothing, and "
        "that is published as a field rather than as prose (EXTENSION-POINTS.md 6.1)");
  Check(delivery_ok,
        "every delivery matches -- THE point of this item: queued means a same-tick read-back "
        "returns the old value, and four ids do not");
  Check(phase_ok, "every declared reading phase matches");

  // The delivery census, pinned as a number. A row flipped from queued to immediate and a
  // second flipped the other way would leave every per-row check above passing on a list that
  // had been reordered to match -- this one still fails. 19 and 4 are the counts
  // EXTENSION-POINTS.md 3.2 publishes.
  int32_t queued = 0, immediate = 0;
  for (int32_t i = 0; i < actual_n; ++i) {
    if (kRows[i].delivery == ext::kDeliveryQueued) ++queued;
    if (kRows[i].delivery == ext::kDeliveryImmediate) ++immediate;
  }
  Check(queued == 29 && immediate == 9,
        "29 queued, 9 immediate -- the census the docs publish, checked as a total so that "
        "two compensating edits cannot pass");

  // The immediate ones by name, because WHICH ones is the load-bearing part. The three
  // registrations have to be immediate (registration closes at the first allocate, and each
  // returns the index everything later needs); the two element-attribute writes were made
  // immediate after the tick of latency turned out to be a race. Anything else appearing here
  // is a design change, not a maintenance edit.
  bool immediate_set_ok = true;
  for (int32_t i = 0; i < actual_n; ++i) {
    if (kRows[i].delivery != ext::kDeliveryImmediate) continue;
    const bool known = strcmp(kRows[i].name, "kSetMolecularMass") == 0 ||
                       strcmp(kRows[i].name, "kRegisterCellProperty") == 0 ||
                       strcmp(kRows[i].name, "kRegisterElementAttribute") == 0 ||
                       strcmp(kRows[i].name, "kSetElementAttribute") == 0 ||
                       // Checkpoint radiation: the only published component, and a queued
                       // restore landed one frame after the first publication (abi header).
                       strcmp(kRows[i].name, "kSetCellRadiation") == 0 ||
                       // The third registration. Immediate for the same reason the first two
                       // are: it returns the index every later message needs, and a queued
                       // registration would hand its caller a sequencing problem with no
                       // answer.
                       strcmp(kRows[i].name, "kRegisterField") == 0 ||
                       // A tunable: queued could never set SubstepSeconds (refused while a
                       // world exists, and no frame drains without one), and it returns its
                       // result to the caller.
                       strcmp(kRows[i].name, "kSetTunable") == 0 ||
                       // Checkpoint visibility: the next PrepareGameData writes one of the
                       // buffers it restores, before a queued restore would drain.
                       strcmp(kRows[i].name, "kSetVisibilityState") == 0 ||
                       // Restore mode: sent before the Load, which is handled on the calling
                       // thread, so a queued one would drain after the Load it was for.
                       strcmp(kRows[i].name, "kSetLoadIsRestore") == 0;
    if (!known) {
      printf("    %s is declared immediate and this test does not know why\n", kRows[i].name);
      immediate_set_ok = false;
    }
  }
  Check(immediate_set_ok,
        "the immediate messages are the nine with a written reason to be immediate");

  // THE DISPATCHER'S TWO PREDICATES, against this test's own hand-typed opinion. Both are
  // generated from ONI_EXT_MESSAGE_LIST -- which means the generator needs a check that is NOT
  // generated from the same list, or it passes by construction. kExpected above is that check:
  // a person typed it, including every delivery, and it is the only thing in this file entitled
  // to disagree with the header.
  //
  // IsQueuedExtMessage is the one that decides behaviour. An id it returns false for is applied
  // inside SIM_HandleMessage, on the caller's thread, before the call returns; an id it returns
  // true for lands on the deferred queue and takes effect at the next drain. That was previously
  // encoded as which terms somebody had remembered to write into a disjunction.
  bool queued_predicate_ok = true, ours_predicate_ok = true;
  for (int32_t i = 0; i < expected_n; ++i) {
    const bool want_queued = kExpected[i].delivery == ext::kDeliveryQueued;
    if (ext::IsQueuedExtMessage(kExpected[i].id) != want_queued) {
      printf("    %s: IsQueuedExtMessage says %d, this test expects %d\n", kExpected[i].name,
             ext::IsQueuedExtMessage(kExpected[i].id) ? 1 : 0, want_queued ? 1 : 0);
      queued_predicate_ok = false;
    }
    if (!ext::IsExtMessage(kExpected[i].id)) {
      printf("    %s: IsExtMessage does not recognise its own id\n", kExpected[i].name);
      ours_predicate_ok = false;
    }
  }
  Check(ours_predicate_ok,
        "IsExtMessage recognises every id this test knows about -- the generated replacement "
        "for KnownMessage's 23-line chain");
  Check(queued_predicate_ok,
        "IsQueuedExtMessage agrees with the delivery column for every id, as typed out by hand "
        "here -- the generated replacement for QueueDeferredMessage's 19-term disjunction, "
        "whose ABSENCES used to be that column");

  // Both predicates must say no to an id that is not ours, since a caller asking about
  // somebody else's message is asking whether WE defer it. 0 is Klei's own "no message" and
  // the two neighbours of our range are the ids a fencepost error would reach.
  Check(!ext::IsExtMessage(0) && !ext::IsQueuedExtMessage(0),
        "id 0 is not ours and is not queued by us");
  Check(!ext::IsExtMessage(ext::kExtMessageIdFirst - 1) &&
            !ext::IsExtMessage(ext::kExtMessageIdLast + 1),
        "neither neighbour of the id range is claimed -- a fencepost in the generator would "
        "claim one of them");
  Check(!ext::IsQueuedExtMessage(ext::kExtMessageIdLast + 1),
        "and an unclaimed id is not queued either, so an unknown message can never be filed "
        "onto the deferred queue by a predicate that was merely being permissive");

  // Ids are contiguous and ascending. The header's static_assert leans on this to notice a
  // constant added with no row; if the property stops holding, that assert stops meaning
  // anything, so it is checked here rather than assumed.
  //
  // WITH TWO NAMED GAPS: "ONIR" and "ONIT" are reserved for messages not in this release, and `kExtMessageReservedIds` is how the header's assert
  // accounts for them. So "contiguous" means: ascending, every step +1 except a step over
  // exactly those ids, and the number of skipped ids equals the reserved count.
  bool contiguous = true;
  int32_t skipped = 0;
  int32_t want = ext::kExtMessageIdFirst;
  for (int32_t i = 0; i < actual_n; ++i) {
    while (kRows[i].id != want && (want == 0x4F4E4952 || want == 0x4F4E4954)) {
      ++want;
      ++skipped;
    }
    if (kRows[i].id != want) contiguous = false;
    ++want;
  }
  if (skipped != ext::kExtMessageReservedIds) contiguous = false;
  Check(contiguous,
        "ids are contiguous and ascending from kExtMessageIdFirst -- the property the "
        "header's row-count assert depends on");

  // An Operation is applied at the drain by definition (EXTENSION-POINTS.md 3.1), so a row
  // that calls itself an Operation and names some other phase is describing something the
  // class does not permit.
  bool ops_ok = true;
  for (int32_t i = 0; i < actual_n; ++i) {
    if (kRows[i].messageClass == ext::kMessageClassOperation &&
        kRows[i].phase != ext::kPhaseDrain) {
      printf("    %s is an Operation but names phase %d, not the drain\n", kRows[i].name,
             kRows[i].phase);
      ops_ok = false;
    }
  }
  Check(ops_ok, "every Operation names the drain as the phase that applies it");
}


// =========================================================================================
// THE PUBLISHED C HEADER, AND THE HALF OF ITS GUARD THAT IS A COMPILE
// ERROR RATHER THAN AN ARM.
//
// `abi/sim_ext_api.h` is the C view of the extension ABI: what a third party binds against.
// `abi/sim_abi_ext.h` is the C++ view, and carries the reasoning. Two files describing one
// wire format is a drift hazard, so the two are held together mechanically.
//
// FOUR MECHANISMS, AND THIS FILE OWNS THE THIRD:
//
//   1. `sim/simdll.cpp` includes the C header, so a declared signature that drifts from its
//      definition is a compile error at the definition -- C++ will not overload an
//      `extern "C"` function.
//   2. The eight types that appear in an export's signature are DEFINED in the C header and
//      aliased by the C++ one. One definition; nothing to drift.
//   3. Everything else -- 23 ids, 8 enums, 4 caps, 7 well-known names, 21 payload layouts --
//      is RESTATED in the C header, and checked here.
//   4. `tools/check-ext-api-header.sh` diffs the declared exports against the built DLL's
//      export table, in both directions, because a MISSING declaration is not an error to
//      any compiler anywhere.
//
// WHY RESTATED AND NOT SHARED. The same standard the phase guard and the message guard are
// held to: a check generated from its own subject passes by construction. A
// published header that is a `#define` away from the internal one proves the two agree
// because they are the same text, which is not the question. Written out independently it is
// a second opinion, and the assertions below are what make holding one cheap.
//
// WHY MOST OF IT IS static_assert AND NOT Check(). A layout comparison has no runtime: both
// sides are compile-time constants, so the honest place to fail is the build, where a wrong
// answer cannot be committed at all rather than merely reported by a suite somebody has to
// run. The `Check()` arms below cover the residue that genuinely needs one -- the string
// constants, which cannot be compared at compile time in C++17 -- plus the counts, which are
// worth PRINTING so a reader of the log can see how much was checked.
//
// This lives in gastest for the same reason the message guard does: `vftest` and `bench`
// require a corpus recorded from a running game. gastest reads
// nothing, and a header comparison needs no game content either.

#define ONI_ABI_SAME_SIZE(c_type, cxx_type)                                                 \
  static_assert(sizeof(c_type) == sizeof(cxx_type),                                         \
                #c_type " and " #cxx_type " disagree about their size -- abi/sim_ext_api.h " \
                "and abi/sim_abi_ext.h describe different wire formats")

// Offset AND exact type, not offset alone: two fields at the same offset with the same width
// but different types (a float where an int32 belongs) is a real way to get this wrong, and
// it is invisible to a size or an offset comparison.
#define ONI_ABI_SAME_FIELD(c_type, cxx_type, field)                                         \
  static_assert(offsetof(c_type, field) == offsetof(cxx_type, field),                       \
                #c_type "::" #field " is at a different offset than " #cxx_type "::" #field); \
  static_assert(std::is_same<decltype(c_type::field), decltype(cxx_type::field)>::value,    \
                #c_type "::" #field " has a different type than " #cxx_type "::" #field)

#define ONI_ABI_SAME_VALUE(c_const, cxx_const)                                              \
  static_assert(static_cast<int64_t>(c_const) == static_cast<int64_t>(cxx_const),           \
                #c_const " and " #cxx_const " are different numbers")

namespace abi_guard {
namespace ext = oni_sim::ext;

// ---- the 23 message ids -----------------------------------------------------------------
ONI_ABI_SAME_VALUE(ONI_MSG_SET_CELL_THERMAL_MASS_BONUS, ext::kSetCellThermalMassBonus);
ONI_ABI_SAME_VALUE(ONI_MSG_INJECT_GAS_SPECIES, ext::kInjectGasSpecies);
ONI_ABI_SAME_VALUE(ONI_MSG_REMOVE_VANILLA_MASS, ext::kRemoveVanillaMass);
ONI_ABI_SAME_VALUE(ONI_MSG_CONVERT_TO_VANILLA_MASS, ext::kConvertToVanillaMass);
ONI_ABI_SAME_VALUE(ONI_MSG_PROMOTE_ROOM, ext::kPromoteRoom);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_INVERTED_GRAVITY_ELEMENT, ext::kSetInvertedGravityElement);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_MOLECULAR_MASS, ext::kSetMolecularMass);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_BUILDING_WASTE_HEAT_KW, ext::kSetBuildingWasteHeatKilowatts);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_BUILDING_EXHAUST, ext::kSetBuildingExhaust);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_BUILDING_RADIATION, ext::kSetBuildingRadiation);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_ENVIRONMENT_TEMPERATURE, ext::kSetEnvironmentTemperature);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_RANDOM_STATE, ext::kSetRandomState);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_SCHEDULING_STATE, ext::kSetSchedulingState);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_STABLE_TICKS, ext::kSetStableTicks);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_DISEASE_GROWTH, ext::kSetDiseaseGrowth);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_REGISTRY_STATE, ext::kSetRegistryState);
ONI_ABI_SAME_VALUE(ONI_MSG_REGISTER_CELL_PROPERTY, ext::kRegisterCellProperty);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_CELL_PROPERTY, ext::kSetCellProperty);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_EXT_CELL_STATE, ext::kSetExtCellState);
ONI_ABI_SAME_VALUE(ONI_MSG_PUBLISH_CELL_PROPERTY, ext::kPublishCellProperty);
ONI_ABI_SAME_VALUE(ONI_MSG_SUBSCRIBE_EVENT_STREAM, ext::kSubscribeEventStream);
ONI_ABI_SAME_VALUE(ONI_MSG_REGISTER_ELEMENT_ATTRIBUTE, ext::kRegisterElementAttribute);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_ELEMENT_ATTRIBUTE, ext::kSetElementAttribute);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_BUILDING_CONVECTION, ext::kSetBuildingConvection);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_WORLD_ENVIRONMENT, ext::kSetWorldEnvironment);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_WORLD_SUN, ext::kSetWorldSun);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_CELL_RADIATION, ext::kSetCellRadiation);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_BLOCKED_GAS_ADD_POLICY, ext::kSetBlockedGasAddPolicy);
ONI_ABI_SAME_VALUE(ONI_BLOCKED_GAS_ADD_VANILLA, ext::kBlockedGasAddVanilla);
ONI_ABI_SAME_VALUE(ONI_BLOCKED_GAS_ADD_PROMOTE_AND_MIX, ext::kBlockedGasAddPromoteAndMix);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_CELL_PROPERTY_TRANSPORT, ext::kSetCellPropertyTransport);
ONI_ABI_SAME_VALUE(ONI_MSG_ADD_CELL_PROPERTY_AMOUNT, ext::kAddCellPropertyAmount);
ONI_ABI_SAME_VALUE(ONI_TRANSPORT_STATIC, ext::kTransportStatic);
ONI_ABI_SAME_VALUE(ONI_TRANSPORT_FOLLOWS_LIQUID_MASS, ext::kTransportFollowsLiquidMass);
ONI_ABI_SAME_VALUE(ONI_DISSOLVED_GAS_LANES, ext::kDissolvedGasLanes);
ONI_ABI_SAME_VALUE(ONI_PAYLOAD_RELEASED_MASSLESS, ext::kPayloadReleasedMassless);
ONI_ABI_SAME_VALUE(ONI_PAYLOAD_RELEASED_EFFERVESCENCE, ext::kPayloadReleasedEffervescence);
ONI_ABI_SAME_VALUE(ONI_PAYLOAD_CONSUMER_MASS_CONSUMPTION, ext::kPayloadConsumerMassConsumption);
ONI_ABI_SAME_VALUE(ONI_MSG_FIRST, ext::kExtMessageIdFirst);
ONI_ABI_SAME_VALUE(ONI_MSG_LAST, ext::kExtMessageIdLast);

// ---- the enums a caller switches on ------------------------------------------------------
ONI_ABI_SAME_VALUE(ONI_EXT_F32, ext::kExtF32);
ONI_ABI_SAME_VALUE(ONI_EXT_U8, ext::kExtU8);
ONI_ABI_SAME_VALUE(ONI_EXT_U16, ext::kExtU16);
ONI_ABI_SAME_VALUE(ONI_EXT_I32, ext::kExtI32);
ONI_ABI_SAME_VALUE(ONI_EXT_ELEMENT_IDX, ext::kExtElementIdx);
ONI_ABI_SAME_VALUE(ONI_EXT_NO_ELEMENT, ext::kExtNoElement);
ONI_ABI_SAME_VALUE(ONI_PERSIST_SAVED, ext::kSaved);
ONI_ABI_SAME_VALUE(ONI_PERSIST_REHYDRATED, ext::kRehydrated);
ONI_ABI_SAME_VALUE(ONI_PERSIST_CHECKPOINT_ONLY, ext::kCheckpointOnly);
ONI_ABI_SAME_VALUE(ONI_EXT_REGISTER_BAD_NAME, ext::kExtRegisterBadName);
ONI_ABI_SAME_VALUE(ONI_EXT_REGISTER_DUPLICATE, ext::kExtRegisterDuplicate);
ONI_ABI_SAME_VALUE(ONI_EXT_REGISTER_RESERVED, ext::kExtRegisterReserved);
ONI_ABI_SAME_VALUE(ONI_EXT_REGISTER_BAD_TYPE, ext::kExtRegisterBadType);
ONI_ABI_SAME_VALUE(ONI_EXT_REGISTER_CLOSED, ext::kExtRegisterClosed);
ONI_ABI_SAME_VALUE(ONI_EXT_REGISTER_FULL, ext::kExtRegisterFull);
ONI_ABI_SAME_VALUE(ONI_EXT_REGISTER_UNSUPPORTED, ext::kExtRegisterUnsupported);
ONI_ABI_SAME_VALUE(ONI_EXT_REGISTER_BAD_ARITY, ext::kExtRegisterBadArity);
ONI_ABI_SAME_VALUE(ONI_EXT_REFUSAL_SHORT_PAYLOAD, ext::kExtRefusalShortPayload);
ONI_ABI_SAME_VALUE(ONI_EXT_REFUSAL_UNKNOWN_MESSAGE, ext::kExtRefusalUnknownMessage);
ONI_ABI_SAME_VALUE(ONI_EXT_REFUSAL_BAD_TARGET, ext::kExtRefusalBadTarget);
ONI_ABI_SAME_VALUE(ONI_EXT_REFUSAL_NOT_FINITE, ext::kExtRefusalNotFinite);
ONI_ABI_SAME_VALUE(ONI_EXT_REFUSAL_OUT_OF_RANGE, ext::kExtRefusalOutOfRange);
ONI_ABI_SAME_VALUE(ONI_EXT_REFUSAL_WORLD_LOADED, ext::kExtRefusalWorldLoaded);
ONI_ABI_SAME_VALUE(ONI_EXT_REFUSAL_RESERVED_BITS, ext::kExtRefusalReservedBits);
ONI_ABI_SAME_VALUE(ONI_EXT_REFUSAL_CROSS_FIELD, ext::kExtRefusalCrossField);
ONI_ABI_SAME_VALUE(ONI_PHASE_SCOPE_FRAME, ext::kPhaseScopeFrame);
ONI_ABI_SAME_VALUE(ONI_PHASE_SCOPE_SUBSTEP, ext::kPhaseScopeSubstep);
ONI_ABI_SAME_VALUE(ONI_PHASE_SCOPE_REGION, ext::kPhaseScopeRegion);
ONI_ABI_SAME_VALUE(ONI_PHASE_SCOPE_GRID_IN_REGION, ext::kPhaseScopeGridInRegion);
ONI_ABI_SAME_VALUE(ONI_PHASE_GATE_NONE, ext::kPhaseGateNone);
ONI_ABI_SAME_VALUE(ONI_PHASE_GATE_SUBSTEP, ext::kPhaseGateSubstep);
ONI_ABI_SAME_VALUE(ONI_PHASE_GATE_WORLD, ext::kPhaseGateWorld);
ONI_ABI_SAME_VALUE(ONI_DELIVERY_QUEUED, ext::kDeliveryQueued);
ONI_ABI_SAME_VALUE(ONI_DELIVERY_IMMEDIATE, ext::kDeliveryImmediate);
ONI_ABI_SAME_VALUE(ONI_MESSAGE_CLASS_PARAMETER, ext::kMessageClassParameter);
ONI_ABI_SAME_VALUE(ONI_MESSAGE_CLASS_OPERATION, ext::kMessageClassOperation);
ONI_ABI_SAME_VALUE(ONI_MESSAGE_CLASS_STORE, ext::kMessageClassStore);
ONI_ABI_SAME_VALUE(ONI_MESSAGE_CLASS_CHECKPOINT, ext::kMessageClassCheckpoint);
ONI_ABI_SAME_VALUE(ONI_PHASE_UNPUBLISHED, ext::kPhaseUnpublished);
ONI_ABI_SAME_VALUE(ONI_EXT_MAX_ARITY, ext::kExtMaxArity);
ONI_ABI_SAME_VALUE(ONI_EXT_MAX_EVENT_STREAMS, ext::kExtMaxEventStreams);
ONI_ABI_SAME_VALUE(ONI_EXT_STREAM_BYTES_PER_FRAME, ext::kExtStreamBytesPerFrame);
ONI_ABI_SAME_VALUE(ONI_EXT_MAX_ELEMENT_ATTRIBUTES, ext::kExtMaxElementAttributes);
// The generic field solver . Both message ids, the three refusal codes the
// registration can return, all four enums and both caps -- the whole published surface, because
// the C header is what a third-party mod compiles against and nothing else compares the two.
ONI_ABI_SAME_VALUE(ONI_MSG_REGISTER_FIELD, ext::kRegisterField);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_FIELD_SOURCE, ext::kSetFieldSource);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_PAYLOAD_MIXING, ext::kSetPayloadMixing);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_DISSOLVED_TINT, ext::kSetDissolvedTint);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_EFFERVESCENCE, ext::kSetEffervescence);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_TUNABLE, ext::kSetTunable);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_VISIBILITY_STATE, ext::kSetVisibilityState);
ONI_ABI_SAME_VALUE(ONI_MSG_SET_LOAD_IS_RESTORE, ext::kSetLoadIsRestore);
ONI_ABI_SAME_VALUE(ONI_TUNABLE_RESET_ALL, ext::kTunableResetAll);
ONI_ABI_SAME_VALUE(ONI_TUNABLE_F32, oni_sim::kTunableF32);
ONI_ABI_SAME_VALUE(ONI_TUNABLE_I32, oni_sim::kTunableI32);
ONI_ABI_SAME_VALUE(ONI_TUNABLE_F64, oni_sim::kTunableF64);
ONI_ABI_SAME_VALUE(ONI_EFFERVESCENCE_MAX_SOLVENTS, ext::kEffervescenceMaxSolvents);
ONI_ABI_SAME_VALUE(ONI_EXT_REGISTER_BAD_PROPERTY, ext::kExtRegisterBadProperty);
ONI_ABI_SAME_VALUE(ONI_EXT_REGISTER_BAD_ATTRIBUTE, ext::kExtRegisterBadAttribute);
ONI_ABI_SAME_VALUE(ONI_EXT_REGISTER_BAD_RULE, ext::kExtRegisterBadRule);
ONI_ABI_SAME_VALUE(ONI_ATTEN_FLAT, ext::kAttenFlat);
ONI_ABI_SAME_VALUE(ONI_ATTEN_RADIATION_MASS, ext::kAttenRadiationMass);
ONI_ABI_SAME_VALUE(ONI_ATTEN_LIGHT_MASS, ext::kAttenLightMass);
ONI_ABI_SAME_VALUE(ONI_COMBINE_TRANSMISSION, ext::kCombineTransmission);
ONI_ABI_SAME_VALUE(ONI_COMBINE_EXPOSURE, ext::kCombineExposure);
ONI_ABI_SAME_VALUE(ONI_DECAY_NONE, ext::kDecayNone);
ONI_ABI_SAME_VALUE(ONI_DECAY_FACTOR, ext::kDecayFactor);
ONI_ABI_SAME_VALUE(ONI_SOURCE_POINT, ext::kSourcePoint);
ONI_ABI_SAME_VALUE(ONI_SOURCE_DIRECTIONAL, ext::kSourceDirectional);
ONI_ABI_SAME_VALUE(ONI_SOURCE_ELEMENT, ext::kSourceElement);
ONI_ABI_SAME_VALUE(ONI_EXT_MAX_FIELDS, ext::kExtMaxFields);
ONI_ABI_SAME_VALUE(ONI_EXT_MAX_FIELD_SOURCES, ext::kExtMaxFieldSources);
// The raycast's two masks . The phase bits are `1 << phase` over world.h's own phase
// enum, and the property bits are Sim.Cell.Properties -- which the sim restates in world.h and
// the C header restates again, so both copies are held to the one the sim actually tests.
ONI_ABI_SAME_VALUE(ONI_RAYCAST_PHASE_VACUUM, 1u << kStateVacuum);
ONI_ABI_SAME_VALUE(ONI_RAYCAST_PHASE_GAS, 1u << kStateGas);
ONI_ABI_SAME_VALUE(ONI_RAYCAST_PHASE_LIQUID, 1u << kStateLiquid);
ONI_ABI_SAME_VALUE(ONI_RAYCAST_PHASE_SOLID, 1u << kStateSolid);
ONI_ABI_SAME_VALUE(ONI_CELL_PROPERTY_GAS_IMPERMEABLE, kGasImpermeable);
ONI_ABI_SAME_VALUE(ONI_CELL_PROPERTY_LIQUID_IMPERMEABLE, kLiquidImpermeable);
ONI_ABI_SAME_VALUE(ONI_CELL_PROPERTY_SOLID_IMPERMEABLE, kSolidImpermeable);
ONI_ABI_SAME_VALUE(ONI_CELL_PROPERTY_UNBREAKABLE, kUnbreakable);
ONI_ABI_SAME_VALUE(ONI_CELL_PROPERTY_TRANSPARENT, kTransparent);
ONI_ABI_SAME_VALUE(ONI_CELL_PROPERTY_OPAQUE, kOpaque);
ONI_ABI_SAME_VALUE(ONI_CELL_PROPERTY_NOTIFY_ON_MELT, kNotifyOnMelt);
ONI_ABI_SAME_VALUE(ONI_CELL_PROPERTY_CONSTRUCTED_TILE, kConstructedTile);

// ---- the 21 restated payload layouts ------------------------------------------------------
// ONI1 reuses Klei's own SetCellFloatValueMessage layout rather than declaring one of its
// own, so the C header's restatement is checked against Klei's struct, not against an ext one.
ONI_ABI_SAME_SIZE(OniSetCellThermalMassBonusMessage, oni_sim::SetCellFloatValueMessage);
ONI_ABI_SAME_FIELD(OniSetCellThermalMassBonusMessage, oni_sim::SetCellFloatValueMessage, cellIdx);
ONI_ABI_SAME_FIELD(OniSetCellThermalMassBonusMessage, oni_sim::SetCellFloatValueMessage, value);

ONI_ABI_SAME_SIZE(OniInjectGasSpeciesMessage, ext::InjectGasSpeciesMessage);
ONI_ABI_SAME_FIELD(OniInjectGasSpeciesMessage, ext::InjectGasSpeciesMessage, cellIdx);
ONI_ABI_SAME_FIELD(OniInjectGasSpeciesMessage, ext::InjectGasSpeciesMessage, speciesIdx);
ONI_ABI_SAME_FIELD(OniInjectGasSpeciesMessage, ext::InjectGasSpeciesMessage, massKg);
ONI_ABI_SAME_FIELD(OniInjectGasSpeciesMessage, ext::InjectGasSpeciesMessage, temperatureK);

ONI_ABI_SAME_SIZE(OniRemoveVanillaMassMessage, ext::RemoveVanillaMassMessage);
ONI_ABI_SAME_FIELD(OniRemoveVanillaMassMessage, ext::RemoveVanillaMassMessage, cellIdx);
ONI_ABI_SAME_FIELD(OniRemoveVanillaMassMessage, ext::RemoveVanillaMassMessage, massKg);

ONI_ABI_SAME_SIZE(OniConvertToVanillaMassMessage, ext::ConvertToVanillaMassMessage);
ONI_ABI_SAME_FIELD(OniConvertToVanillaMassMessage, ext::ConvertToVanillaMassMessage, cellIdx);
ONI_ABI_SAME_FIELD(OniConvertToVanillaMassMessage, ext::ConvertToVanillaMassMessage, speciesIdx);
ONI_ABI_SAME_FIELD(OniConvertToVanillaMassMessage, ext::ConvertToVanillaMassMessage, massKg);
ONI_ABI_SAME_FIELD(OniConvertToVanillaMassMessage, ext::ConvertToVanillaMassMessage, temperatureK);

ONI_ABI_SAME_SIZE(OniPromoteRoomMessage, ext::PromoteRoomMessage);
ONI_ABI_SAME_FIELD(OniPromoteRoomMessage, ext::PromoteRoomMessage, cellIdx);

ONI_ABI_SAME_SIZE(OniSetInvertedGravityElementMessage, ext::SetInvertedGravityElementMessage);
ONI_ABI_SAME_FIELD(OniSetInvertedGravityElementMessage, ext::SetInvertedGravityElementMessage,
                   elementIdx);

ONI_ABI_SAME_SIZE(OniSetMolecularMassMessage, ext::SetMolecularMassMessage);
ONI_ABI_SAME_FIELD(OniSetMolecularMassMessage, ext::SetMolecularMassMessage, idHash);
ONI_ABI_SAME_FIELD(OniSetMolecularMassMessage, ext::SetMolecularMassMessage, gPerMol);

ONI_ABI_SAME_SIZE(OniSetBuildingWasteHeatKilowattsMessage,
                  ext::SetBuildingWasteHeatKilowattsMessage);
ONI_ABI_SAME_FIELD(OniSetBuildingWasteHeatKilowattsMessage,
                   ext::SetBuildingWasteHeatKilowattsMessage, handle);
ONI_ABI_SAME_FIELD(OniSetBuildingWasteHeatKilowattsMessage,
                   ext::SetBuildingWasteHeatKilowattsMessage, kilowatts);

ONI_ABI_SAME_SIZE(OniSetBuildingExhaustMessage, ext::SetBuildingExhaustMessage);
ONI_ABI_SAME_FIELD(OniSetBuildingExhaustMessage, ext::SetBuildingExhaustMessage, handle);
ONI_ABI_SAME_FIELD(OniSetBuildingExhaustMessage, ext::SetBuildingExhaustMessage, kilowatts);
ONI_ABI_SAME_FIELD(OniSetBuildingExhaustMessage, ext::SetBuildingExhaustMessage, maxTemperature);

ONI_ABI_SAME_SIZE(OniSetBuildingRadiationMessage, ext::SetBuildingRadiationMessage);
ONI_ABI_SAME_FIELD(OniSetBuildingRadiationMessage, ext::SetBuildingRadiationMessage, handle);
ONI_ABI_SAME_FIELD(OniSetBuildingRadiationMessage, ext::SetBuildingRadiationMessage,
                   radiationFactor);
ONI_ABI_SAME_FIELD(OniSetBuildingRadiationMessage, ext::SetBuildingRadiationMessage,
                   surfaceAreaM2);

ONI_ABI_SAME_SIZE(OniSetBuildingConvectionMessage, ext::SetBuildingConvectionMessage);
ONI_ABI_SAME_FIELD(OniSetBuildingConvectionMessage, ext::SetBuildingConvectionMessage, handle);
ONI_ABI_SAME_FIELD(OniSetBuildingConvectionMessage, ext::SetBuildingConvectionMessage,
                   convectionFactor);
ONI_ABI_SAME_FIELD(OniSetBuildingConvectionMessage, ext::SetBuildingConvectionMessage,
                   surfaceAreaM2);
ONI_ABI_SAME_FIELD(OniSetBuildingConvectionMessage, ext::SetBuildingConvectionMessage,
                   reachCells);

ONI_ABI_SAME_SIZE(OniSetWorldEnvironmentMessage, ext::SetWorldEnvironmentMessage);
ONI_ABI_SAME_FIELD(OniSetWorldEnvironmentMessage, ext::SetWorldEnvironmentMessage, worldIndex);
ONI_ABI_SAME_FIELD(OniSetWorldEnvironmentMessage, ext::SetWorldEnvironmentMessage, sinkKelvin);
ONI_ABI_SAME_FIELD(OniSetWorldEnvironmentMessage, ext::SetWorldEnvironmentMessage,
                   peakIrradianceWPerM2);
ONI_ABI_SAME_FIELD(OniSetWorldEnvironmentMessage, ext::SetWorldEnvironmentMessage, fullSunLux);
ONI_ABI_SAME_FIELD(OniSetWorldEnvironmentMessage, ext::SetWorldEnvironmentMessage,
                   solarAbsorptivity);
ONI_ABI_SAME_FIELD(OniSetWorldEnvironmentMessage, ext::SetWorldEnvironmentMessage,
                   surfacePressureKPa);
ONI_ABI_SAME_FIELD(OniSetWorldEnvironmentMessage, ext::SetWorldEnvironmentMessage,
                   boundaryTemperatureK);
ONI_ABI_SAME_FIELD(OniSetWorldEnvironmentMessage, ext::SetWorldEnvironmentMessage, boundaryRate);
ONI_ABI_SAME_FIELD(OniSetWorldEnvironmentMessage, ext::SetWorldEnvironmentMessage,
                   boundaryElement);

ONI_ABI_SAME_SIZE(OniSetEnvironmentTemperatureMessage, ext::SetEnvironmentTemperatureMessage);
ONI_ABI_SAME_FIELD(OniSetEnvironmentTemperatureMessage, ext::SetEnvironmentTemperatureMessage,
                   kelvin);

ONI_ABI_SAME_SIZE(OniSetRandomStateMessage, ext::SetRandomStateMessage);
ONI_ABI_SAME_FIELD(OniSetRandomStateMessage, ext::SetRandomStateMessage, state);

ONI_ABI_SAME_SIZE(OniSetStableTicksMessage, ext::SetStableTicksMessage);
ONI_ABI_SAME_FIELD(OniSetStableTicksMessage, ext::SetStableTicksMessage, count);

ONI_ABI_SAME_SIZE(OniSetDiseaseGrowthMessage, ext::SetDiseaseGrowthMessage);
ONI_ABI_SAME_FIELD(OniSetDiseaseGrowthMessage, ext::SetDiseaseGrowthMessage, count);
ONI_ABI_SAME_SIZE(OniSetCellRadiationMessage, ext::SetCellRadiationMessage);
ONI_ABI_SAME_FIELD(OniSetCellRadiationMessage, ext::SetCellRadiationMessage, count);
ONI_ABI_SAME_SIZE(OniSetBlockedGasAddPolicyMessage, ext::SetBlockedGasAddPolicyMessage);
ONI_ABI_SAME_FIELD(OniSetBlockedGasAddPolicyMessage, ext::SetBlockedGasAddPolicyMessage, policy);

ONI_ABI_SAME_SIZE(OniRegisterCellPropertyMessage, ext::RegisterCellPropertyMessage);
ONI_ABI_SAME_FIELD(OniRegisterCellPropertyMessage, ext::RegisterCellPropertyMessage, name);
ONI_ABI_SAME_FIELD(OniRegisterCellPropertyMessage, ext::RegisterCellPropertyMessage, type);
ONI_ABI_SAME_FIELD(OniRegisterCellPropertyMessage, ext::RegisterCellPropertyMessage, persist);
ONI_ABI_SAME_FIELD(OniRegisterCellPropertyMessage, ext::RegisterCellPropertyMessage, arity);
ONI_ABI_SAME_FIELD(OniRegisterCellPropertyMessage, ext::RegisterCellPropertyMessage,
                   defaultBits);

ONI_ABI_SAME_SIZE(OniSetCellPropertyMessage, ext::SetCellPropertyMessage);
ONI_ABI_SAME_FIELD(OniSetCellPropertyMessage, ext::SetCellPropertyMessage, cellIdx);
ONI_ABI_SAME_FIELD(OniSetCellPropertyMessage, ext::SetCellPropertyMessage, propertyIdx);
ONI_ABI_SAME_FIELD(OniSetCellPropertyMessage, ext::SetCellPropertyMessage, component);
ONI_ABI_SAME_FIELD(OniSetCellPropertyMessage, ext::SetCellPropertyMessage, valueBits);

ONI_ABI_SAME_SIZE(OniPublishCellPropertyMessage, ext::PublishCellPropertyMessage);
ONI_ABI_SAME_FIELD(OniPublishCellPropertyMessage, ext::PublishCellPropertyMessage, propertyIdx);
ONI_ABI_SAME_FIELD(OniPublishCellPropertyMessage, ext::PublishCellPropertyMessage, enable);

ONI_ABI_SAME_SIZE(OniSubscribeEventStreamMessage, ext::SubscribeEventStreamMessage);
ONI_ABI_SAME_FIELD(OniSubscribeEventStreamMessage, ext::SubscribeEventStreamMessage, streamIdx);
ONI_ABI_SAME_FIELD(OniSubscribeEventStreamMessage, ext::SubscribeEventStreamMessage, enable);

ONI_ABI_SAME_SIZE(OniRegisterElementAttributeMessage, ext::RegisterElementAttributeMessage);
ONI_ABI_SAME_FIELD(OniRegisterElementAttributeMessage, ext::RegisterElementAttributeMessage,
                   name);
ONI_ABI_SAME_FIELD(OniRegisterElementAttributeMessage, ext::RegisterElementAttributeMessage,
                   type);
ONI_ABI_SAME_FIELD(OniRegisterElementAttributeMessage, ext::RegisterElementAttributeMessage,
                   arity);

ONI_ABI_SAME_SIZE(OniSetElementAttributeMessage, ext::SetElementAttributeMessage);
ONI_ABI_SAME_FIELD(OniSetElementAttributeMessage, ext::SetElementAttributeMessage, attrIdx);
ONI_ABI_SAME_FIELD(OniSetElementAttributeMessage, ext::SetElementAttributeMessage, idHash);
ONI_ABI_SAME_FIELD(OniSetElementAttributeMessage, ext::SetElementAttributeMessage, component);
ONI_ABI_SAME_FIELD(OniSetElementAttributeMessage, ext::SetElementAttributeMessage, valueBits);
ONI_ABI_SAME_FIELD(OniSetElementAttributeMessage, ext::SetElementAttributeMessage, clear);

ONI_ABI_SAME_SIZE(OniExtRefusedMessage, ext::ExtRefusedMessage);
ONI_ABI_SAME_SIZE(OniSetCellPropertyTransportMessage, ext::SetCellPropertyTransportMessage);
ONI_ABI_SAME_SIZE(OniAddCellPropertyAmountMessage, ext::AddCellPropertyAmountMessage);
ONI_ABI_SAME_FIELD(OniAddCellPropertyAmountMessage, ext::AddCellPropertyAmountMessage, amount);
ONI_ABI_SAME_SIZE(OniLiquidPayloadReleased, ext::LiquidPayloadReleased);
ONI_ABI_SAME_FIELD(OniLiquidPayloadReleased, ext::LiquidPayloadReleased, reason);
ONI_ABI_SAME_SIZE(OniLiquidPayloadConsumed, ext::LiquidPayloadConsumed);
ONI_ABI_SAME_FIELD(OniLiquidPayloadConsumed, ext::LiquidPayloadConsumed, temperatureK);
ONI_ABI_SAME_FIELD(OniExtRefusedMessage, ext::ExtRefusedMessage, messageId);
ONI_ABI_SAME_FIELD(OniExtRefusedMessage, ext::ExtRefusedMessage, reason);
ONI_ABI_SAME_FIELD(OniExtRefusedMessage, ext::ExtRefusedMessage, payloadBytes);
ONI_ABI_SAME_FIELD(OniExtRefusedMessage, ext::ExtRefusedMessage, expectedBytes);

// The field solver's two payloads. Every field of both, not only the size: the two messages
// carry ten and eleven scalars respectively and a pair swapped between them would still be 40
// and 44 bytes.
ONI_ABI_SAME_SIZE(OniRegisterFieldMessage, ext::RegisterFieldMessage);
ONI_ABI_SAME_FIELD(OniRegisterFieldMessage, ext::RegisterFieldMessage, propertyIdx);
ONI_ABI_SAME_FIELD(OniRegisterFieldMessage, ext::RegisterFieldMessage, attributeIdx);
ONI_ABI_SAME_FIELD(OniRegisterFieldMessage, ext::RegisterFieldMessage, attributeFallback);
ONI_ABI_SAME_FIELD(OniRegisterFieldMessage, ext::RegisterFieldMessage, law);
ONI_ABI_SAME_FIELD(OniRegisterFieldMessage, ext::RegisterFieldMessage, combine);
ONI_ABI_SAME_FIELD(OniRegisterFieldMessage, ext::RegisterFieldMessage, decayMode);
ONI_ABI_SAME_FIELD(OniRegisterFieldMessage, ext::RegisterFieldMessage, decayKeep);
ONI_ABI_SAME_FIELD(OniRegisterFieldMessage, ext::RegisterFieldMessage, floorValue);
ONI_ABI_SAME_FIELD(OniRegisterFieldMessage, ext::RegisterFieldMessage, clampLo);
ONI_ABI_SAME_FIELD(OniRegisterFieldMessage, ext::RegisterFieldMessage, clampHi);
ONI_ABI_SAME_SIZE(OniSetPayloadMixingMessage, ext::SetPayloadMixingMessage);
ONI_ABI_SAME_FIELD(OniSetPayloadMixingMessage, ext::SetPayloadMixingMessage, propertyIdx);
ONI_ABI_SAME_FIELD(OniSetPayloadMixingMessage, ext::SetPayloadMixingMessage, share);
ONI_ABI_SAME_SIZE(OniSetDissolvedTintMessage, ext::SetDissolvedTintMessage);
ONI_ABI_SAME_FIELD(OniSetDissolvedTintMessage, ext::SetDissolvedTintMessage, enabled);
ONI_ABI_SAME_FIELD(OniSetDissolvedTintMessage, ext::SetDissolvedTintMessage, fullScaleGramsPerKg);
ONI_ABI_SAME_FIELD(OniSetDissolvedTintMessage, ext::SetDissolvedTintMessage, maxBlend);
ONI_ABI_SAME_FIELD(OniSetDissolvedTintMessage, ext::SetDissolvedTintMessage, laneWeight);
ONI_ABI_SAME_FIELD(OniSetDissolvedTintMessage, ext::SetDissolvedTintMessage, laneColour);
ONI_ABI_SAME_SIZE(OniSetEffervescenceMessage, ext::SetEffervescenceMessage);
ONI_ABI_SAME_FIELD(OniSetEffervescenceMessage, ext::SetEffervescenceMessage, enabled);
ONI_ABI_SAME_FIELD(OniSetEffervescenceMessage, ext::SetEffervescenceMessage, margin);
ONI_ABI_SAME_FIELD(OniSetEffervescenceMessage, ext::SetEffervescenceMessage, ratePerSecond);
ONI_ABI_SAME_FIELD(OniSetEffervescenceMessage, ext::SetEffervescenceMessage, periodSeconds);
ONI_ABI_SAME_FIELD(OniSetEffervescenceMessage, ext::SetEffervescenceMessage, minReleaseKg);
ONI_ABI_SAME_FIELD(OniSetEffervescenceMessage, ext::SetEffervescenceMessage, laneHenryMolPerM3Pa);
ONI_ABI_SAME_FIELD(OniSetEffervescenceMessage, ext::SetEffervescenceMessage, laneVantHoffK);
ONI_ABI_SAME_FIELD(OniSetEffervescenceMessage, ext::SetEffervescenceMessage, laneMolarMassKgPerMol);
ONI_ABI_SAME_FIELD(OniSetEffervescenceMessage, ext::SetEffervescenceMessage,
                   lanePartialMolarVolumeM3PerMol);
ONI_ABI_SAME_FIELD(OniSetEffervescenceMessage, ext::SetEffervescenceMessage, solventCount);
ONI_ABI_SAME_FIELD(OniSetEffervescenceMessage, ext::SetEffervescenceMessage, solventElementIdx);
ONI_ABI_SAME_FIELD(OniSetEffervescenceMessage, ext::SetEffervescenceMessage, solventFactor);
ONI_ABI_SAME_FIELD(OniSetEffervescenceMessage, ext::SetEffervescenceMessage,
                   solventDensityKgPerM3);
ONI_ABI_SAME_SIZE(OniSetEffervescenceMessageV2, ext::SetEffervescenceMessageV2);
// `base` is each header's own 248-byte struct, checked above; its offset is 0 in both by
// construction, so only the appended fields are compared here.
ONI_ABI_SAME_FIELD(OniSetEffervescenceMessageV2, ext::SetEffervescenceMessageV2,
                   surfaceRatePerSecond);
ONI_ABI_SAME_FIELD(OniSetEffervescenceMessageV2, ext::SetEffervescenceMessageV2,
                   surfaceMinReleaseKg);
ONI_ABI_SAME_FIELD(OniSetEffervescenceMessageV2, ext::SetEffervescenceMessageV2, laneElementIdx);
ONI_ABI_SAME_SIZE(OniSetTunableMessage, ext::SetTunableMessage);
ONI_ABI_SAME_FIELD(OniSetTunableMessage, ext::SetTunableMessage, tunableId);
ONI_ABI_SAME_FIELD(OniSetTunableMessage, ext::SetTunableMessage, reserved);
ONI_ABI_SAME_FIELD(OniSetTunableMessage, ext::SetTunableMessage, valueBits);
ONI_ABI_SAME_SIZE(OniSetVisibilityStateMessage, ext::SetVisibilityStateMessage);
ONI_ABI_SAME_FIELD(OniSetVisibilityStateMessage, ext::SetVisibilityStateMessage, count);
ONI_ABI_SAME_FIELD(OniSetVisibilityStateMessage, ext::SetVisibilityStateMessage, simSlot);
ONI_ABI_SAME_SIZE(OniSetLoadIsRestoreMessage, ext::SetLoadIsRestoreMessage);
ONI_ABI_SAME_FIELD(OniSetLoadIsRestoreMessage, ext::SetLoadIsRestoreMessage, restore);
ONI_ABI_SAME_FIELD(OniSetLoadIsRestoreMessage, ext::SetLoadIsRestoreMessage, reserved);
ONI_ABI_SAME_VALUE(ONI_PAYLOAD_RELEASED_SURFACE, ext::kPayloadReleasedSurface);
ONI_ABI_SAME_VALUE(ONI_EFFERVESCENCE_CENSUS_SURFACES, ext::kEffervescenceCensusSurfaces);
ONI_ABI_SAME_VALUE(ONI_EFFERVESCENCE_CENSUS_SURFACE_RELEASES,
                   ext::kEffervescenceCensusSurfaceReleases);
ONI_ABI_SAME_VALUE(ONI_EFFERVESCENCE_CENSUS_SURFACE_KG, ext::kEffervescenceCensusSurfaceKg);
ONI_ABI_SAME_VALUE(ONI_PAYLOAD_ABSORBED_SURFACE, ext::kPayloadAbsorbedSurface);
ONI_ABI_SAME_VALUE(ONI_EFFERVESCENCE_CENSUS_SURFACE_UPTAKES,
                   ext::kEffervescenceCensusSurfaceUptakes);
ONI_ABI_SAME_VALUE(ONI_EFFERVESCENCE_CENSUS_SURFACE_UPTAKE_KG,
                   ext::kEffervescenceCensusSurfaceUptakeKg);
ONI_ABI_SAME_VALUE(ONI_EFFERVESCENCE_CENSUS_SURFACE_UPTAKE_HEAT_KJ,
                   ext::kEffervescenceCensusSurfaceUptakeHeatKJ);
ONI_ABI_SAME_VALUE(ONI_EFFERVESCENCE_CENSUS_FIELDS, ext::kEffervescenceCensusFields);
static_assert(ONI_EFFERVESCENCE_DEFAULT_SURFACE_RATE_PER_SECOND ==
                  ext::kEffervescenceDefaultSurfaceRatePerSecond &&
                  ONI_EFFERVESCENCE_DEFAULT_SURFACE_MIN_RELEASE_KG ==
                  ext::kEffervescenceDefaultSurfaceMinReleaseKg,
              "abi/sim_ext_api.h and abi/sim_abi_ext.h disagree about the surface exchange's "
              "defaults");
ONI_ABI_SAME_VALUE(ONI_PAYLOAD_MIXING_ALL_PROPERTIES, ext::kPayloadMixingAllProperties);
// A FLOAT, so `ONI_ABI_SAME_VALUE` is the wrong tool: its int64 cast would turn both sides
// into zero and pass for any pair of values below one. Compared directly.
static_assert(ONI_PAYLOAD_MIXING_DEFAULT_SHARE == ext::kPayloadMixingDefaultShare,
              "abi/sim_ext_api.h and abi/sim_abi_ext.h disagree about the default payload "
              "mixing share");

ONI_ABI_SAME_SIZE(OniSetFieldSourceMessage, ext::SetFieldSourceMessage);
ONI_ABI_SAME_FIELD(OniSetFieldSourceMessage, ext::SetFieldSourceMessage, fieldIdx);
ONI_ABI_SAME_FIELD(OniSetFieldSourceMessage, ext::SetFieldSourceMessage, sourceId);
ONI_ABI_SAME_FIELD(OniSetFieldSourceMessage, ext::SetFieldSourceMessage, kind);
ONI_ABI_SAME_FIELD(OniSetFieldSourceMessage, ext::SetFieldSourceMessage, strength);
ONI_ABI_SAME_FIELD(OniSetFieldSourceMessage, ext::SetFieldSourceMessage, target);
ONI_ABI_SAME_FIELD(OniSetFieldSourceMessage, ext::SetFieldSourceMessage, radiusX);
ONI_ABI_SAME_FIELD(OniSetFieldSourceMessage, ext::SetFieldSourceMessage, radiusY);
ONI_ABI_SAME_FIELD(OniSetFieldSourceMessage, ext::SetFieldSourceMessage, coneDirection);
ONI_ABI_SAME_FIELD(OniSetFieldSourceMessage, ext::SetFieldSourceMessage, coneAngle);
ONI_ABI_SAME_FIELD(OniSetFieldSourceMessage, ext::SetFieldSourceMessage, dirX);
ONI_ABI_SAME_FIELD(OniSetFieldSourceMessage, ext::SetFieldSourceMessage, dirY);

// ---- and the eight the C header OWNS, asserted the other way round ------------------------
// These are aliases, so `sizeof` agreeing is a tautology. What is NOT a tautology is that
// the alias still points at a type of the size the ABI promises: a field added to one of
// these in sim_ext_api.h changes what every consumer's buffer has to be, and the numbers
// below are the ones every marshalling layer is written against.
static_assert(sizeof(OniExtPhaseDesc) == 44, "ExtPhaseDesc is 44 bytes on the wire");
static_assert(sizeof(OniExtMessageDesc) == 60, "ExtMessageDesc is 60 bytes on the wire");
static_assert(sizeof(OniExtCellPropertyDesc) == 76, "ExtCellPropertyDesc is 76 bytes");
static_assert(sizeof(OniExtEventStreamDesc) == 60, "ExtEventStreamDesc is 60 bytes");
static_assert(sizeof(OniExtElementAttributeDesc) == 80, "ExtElementAttributeDesc is 80 bytes");
static_assert(sizeof(OniExtFieldDesc) == 96, "ExtFieldDesc is 96 bytes");
static_assert(sizeof(OniExtPublishedProperty) == 80, "ExtPublishedProperty is 80 bytes");
static_assert(sizeof(OniExtPublishedStream) == 72, "ExtPublishedStream is 72 bytes");
static_assert(sizeof(OniSetSchedulingStateMessage) == 20, "SetSchedulingState is 20 bytes");
static_assert(sizeof(OniExtTunableDesc) == 312, "ExtTunableDesc is 312 bytes");
static_assert(std::is_same<oni_sim::ext::ExtTunableDesc, ::OniExtTunableDesc>::value,
              "the C++ ExtTunableDesc is supposed to BE the published one");
static_assert(std::is_same<oni_sim::ext::ExtPhaseDesc, ::OniExtPhaseDesc>::value,
              "the C++ ExtPhaseDesc is supposed to BE the published one, not a copy of it");
static_assert(std::is_same<oni_sim::ext::ExtMessageDesc, ::OniExtMessageDesc>::value,
              "the C++ ExtMessageDesc is supposed to BE the published one");
static_assert(std::is_same<oni_sim::ext::ExtFieldDesc, ::OniExtFieldDesc>::value,
              "the C++ ExtFieldDesc is supposed to BE the published one");

}  // namespace abi_guard

// The residue that cannot be a static_assert. A string literal comparison is not a constant
// expression in C++17, and these eight names are ABI: a mod resolves a property or a stream
// by spelling one, so a name that differs between the two headers is a lookup that returns
// -1 for a reason nobody can see.
// ===================================================================================
// The grid raycast (sim/raycast.h, published as SIM_QueryRaycast).
//
// Its whole claim is that it is Klei's radiation ray walk with a stop condition added, so both
// halves of that claim are checked against something that is NOT sim/raycast.h:
//
//   * WHICH CELLS. The set comes from `RadiationAbsorptionAlongLine` itself, probed: a vacuum
//     world with one absorbing cell dims the ray iff the walk steps on that cell. That is the
//     radiation field's own answer to "is this cell between", with no second walk involved.
//     Ordered from the caller's start, the reference then finds the first stopping cell, the one
//     before it, and a product over the prefix -- a plain second implementation of the rest.
//   * THE TRANSMISSION. With both masks zero it must be `RadiationAbsorptionAlongLine`'s float,
//     BIT FOR BIT; with a stop, within 1e-5 of the reference's double product (the order of a
//     float product is the only thing allowed to differ).
//
// Every line is cast BOTH WAYS. Klei's walk always runs low-to-high on the major axis, so a ray
// cast from the high end is the case a tidier implementation gets wrong without noticing: it
// would walk its own cells in its own order and pass every test written from the low end.
ElementTable MakeRaycastTable() {
  std::vector<Element> els(4);
  memset(els.data(), 0, els.size() * sizeof(Element));
  els[0].id = 0;  els[0].state = kStateVacuum;  els[0].radiationAbsorptionFactor = 0.0f;
  els[1].id = 1;  els[1].state = kStateGas;     els[1].radiationAbsorptionFactor = 0.05f;
  els[2].id = 2;  els[2].state = kStateLiquid;  els[2].radiationAbsorptionFactor = 0.3f;
  els[3].id = 3;  els[3].state = kStateSolid;   els[3].radiationAbsorptionFactor = 0.8f;
  std::vector<uint8_t> blob(4 + els.size() * sizeof(Element));
  const int32_t n = static_cast<int32_t>(els.size());
  memcpy(blob.data(), &n, 4);
  memcpy(blob.data() + 4, els.data(), els.size() * sizeof(Element));
  ElementTable table;
  if (!table.Load(blob.data(), blob.size())) {
    printf("  FAIL: synthetic raycast ElementTable failed to load\n");
    ++g_fail_count;
  }
  return table;
}

void TestRaycast() {
  printf("\n=== grid raycast: Klei's ray walk with a stop condition ===\n");
  const ElementTable table = MakeRaycastTable();
  constexpr int32_t kW = 13, kH = 9;

  // The world the rays are cast through: every phase, random masses, and properties set on
  // some cells, so both masks and the constructed-tile branch of the absorption all run.
  World world;
  world.Allocate(kW, kH);
  uint32_t rng = 0x2545F491u;
  auto next = [&rng]() { rng = rng * 1664525u + 1013904223u; return rng >> 8; };
  for (int32_t c = 0; c < kW * kH; ++c) {
    const size_t p = world.Padded(static_cast<size_t>(c));
    const uint16_t e = static_cast<uint16_t>(next() % 4);
    world.Phase(p).element = e;
    world.Phase(p).mass = e == 0 ? 0.0f : static_cast<float>(next() % 3000) * 0.5f;
    uint8_t props = 0;
    if (e == 3 && next() % 3 == 0) props |= kConstructedTile;
    if (next() % 7 == 0) props |= kSolidImpermeable;
    if (e == 3 && next() % 4 == 0) props |= kTransparent;  // a window: solid, but see-through
    world.MutableProperties(p) = props;
  }

  // The probe world: vacuum everywhere, one absorbing cell moved around.
  World probe;
  probe.Allocate(kW, kH);
  for (int32_t c = 0; c < kW * kH; ++c) {
    const size_t p = probe.Padded(static_cast<size_t>(c));
    probe.Phase(p).element = 0;
    probe.Phase(p).mass = 0.0f;
  }
  const int32_t pw = world.PaddedWidth();

  struct Line { int32_t x0, y0, x1, y1; };
  std::vector<Line> lines;
  // The shapes a walk gets wrong first: a point, both axes, both diagonals, a knight's move.
  lines.push_back({4, 4, 4, 4});
  lines.push_back({1, 3, kW, 3});
  lines.push_back({6, 1, 6, kH});
  lines.push_back({1, 1, 9, 9});
  lines.push_back({1, 9, 9, 1});
  lines.push_back({3, 3, 4, 5});
  while (lines.size() < 200) {
    lines.push_back({static_cast<int32_t>(1 + next() % kW), static_cast<int32_t>(1 + next() % kH),
                     static_cast<int32_t>(1 + next() % kW), static_cast<int32_t>(1 + next() % kH)});
  }

  struct Masks { uint32_t phase, property, ignore; const char* name; };
  const Masks masks[] = {
      {0u, 0u, 0u, "nothing"},
      {1u << kStateSolid, 0u, 0u, "solid"},
      {(1u << kStateLiquid) | (1u << kStateSolid), 0u, 0u, "liquid|solid"},
      {1u << kStateGas, 0u, 0u, "gas"},
      {0u, kConstructedTile, 0u, "constructed tile"},
      {1u << kStateVacuum, kSolidImpermeable, 0u, "vacuum or solid-impermeable"},
      {1u << kStateSolid, 0u, kTransparent, "solid, except transparent"},
      {1u << kStateSolid, kSolidImpermeable, kSolidImpermeable, "ignore beats both masks"},
  };

  int rays = 0, reversed_rays = 0, stopped = 0, stopped_reversed = 0;
  int set_mismatch = 0, endpoint_mismatch = 0, result_mismatch = 0, unmasked_bit_mismatch = 0;
  int unmasked_shape_mismatch = 0;
  int ignore_mattered = 0;  // rays whose answer the ignore mask changed
  std::vector<size_t> ordered;
  for (const Line& base : lines) {
    for (int dir = 0; dir < 2; ++dir) {
      const Line l = dir == 0 ? base : Line{base.x1, base.y1, base.x0, base.y0};
      const size_t start = static_cast<size_t>(l.y0) * pw + l.x0;
      const size_t end = static_cast<size_t>(l.y1) * pw + l.x1;

      // The cell set, from Klei's own function.
      ordered.clear();
      for (int32_t c = 0; c < kW * kH; ++c) {
        const size_t p = probe.Padded(static_cast<size_t>(c));
        probe.Phase(p).element = 3;
        probe.Phase(p).mass = 100.0f;
        if (RadiationAbsorptionAlongLine(probe, table, l.x0, l.y0, l.x1, l.y1) < 1.0f) {
          ordered.push_back(p);
        }
        probe.Phase(p).element = 0;
        probe.Phase(p).mass = 0.0f;
      }
      const bool along_x = std::abs(l.y1 - l.y0) <= std::abs(l.x1 - l.x0);
      auto dist = [&](size_t p) {
        const int32_t x = static_cast<int32_t>(p % pw), y = static_cast<int32_t>(p / pw);
        return along_x ? std::abs(x - l.x0) : std::abs(y - l.y0);
      };
      std::sort(ordered.begin(), ordered.end(),
                [&](size_t a, size_t b) { return dist(a) < dist(b); });
      const int32_t span = along_x ? std::abs(l.x1 - l.x0) : std::abs(l.y1 - l.y0);
      if (static_cast<int32_t>(ordered.size()) != span + 1) ++set_mismatch;
      if (ordered.empty() || ordered.front() != start || ordered.back() != end) {
        ++endpoint_mismatch;
      }

      for (const Masks& m : masks) {
        const RaycastHit got =
            Raycast(world, table, l.x0, l.y0, l.x1, l.y1, m.phase, m.property, m.ignore);
        ++rays;
        if (m.ignore != 0 &&
            Raycast(world, table, l.x0, l.y0, l.x1, l.y1, m.phase, m.property, 0u).hit != got.hit) {
          ++ignore_mattered;
        }
        const bool is_reversed = along_x ? l.x1 < l.x0 : l.y1 < l.y0;
        if (is_reversed) ++reversed_rays;

        // The reference: first stopping cell from the start, the cell before it, the prefix.
        size_t hit = RaycastHit::kNoCell, last_clear = RaycastHit::kNoCell;
        int32_t visited = 0;
        double product = 1.0;
        for (size_t k = 0; k < ordered.size(); ++k) {
          const size_t p = ordered[k];
          product *= 1.0 - static_cast<double>(RadiationAbsorption(world, table, p));
          ++visited;
          // Written out rather than calling RaycastStops, so the reference does not share
          // the rule it is checking.
          const uint8_t props = world.Properties()[p];
          const bool phase_stop = ((m.phase >> table.Phase(world.Phase(p).element)) & 1u) != 0;
          if ((props & m.ignore) == 0 && (phase_stop || (props & m.property) != 0)) {
            hit = p;
            break;
          }
          last_clear = p;
        }
        if (product > 1.0) product = 1.0;
        if (product < 0.0) product = 0.0;
        if (hit != RaycastHit::kNoCell) {
          ++stopped;
          if (is_reversed) ++stopped_reversed;
        }
        if (got.hit != hit || got.last_clear != last_clear || got.visited != visited ||
            std::fabs(static_cast<double>(got.transmission) - product) > 1e-5) {
          if (result_mismatch < 3) {
            printf("  mismatch (%d,%d)->(%d,%d) stop on %s: got hit %zd clear %zd visited %d "
                   "T %.7f; reference hit %zd clear %zd visited %d T %.7f\n",
                   l.x0, l.y0, l.x1, l.y1, m.name, static_cast<ptrdiff_t>(got.hit),
                   static_cast<ptrdiff_t>(got.last_clear), got.visited, got.transmission,
                   static_cast<ptrdiff_t>(hit), static_cast<ptrdiff_t>(last_clear), visited,
                   product);
          }
          ++result_mismatch;
        }
        if (m.phase == 0 && m.property == 0 && m.ignore == 0) {
          const float klei = RadiationAbsorptionAlongLine(world, table, l.x0, l.y0, l.x1, l.y1);
          uint32_t a_bits, b_bits;
          memcpy(&a_bits, &got.transmission, 4);
          memcpy(&b_bits, &klei, 4);
          if (a_bits != b_bits) ++unmasked_bit_mismatch;
          if (got.hit != RaycastHit::kNoCell || got.last_clear != end ||
              got.visited != span + 1) {
            ++unmasked_shape_mismatch;
          }
        }
      }
    }
  }
  printf("  %d rays over %zu lines cast both ways, %d of them from the high end; %d stopped "
         "(%d of those from the high end)\n",
         rays, lines.size(), reversed_rays, stopped, stopped_reversed);
  Check(set_mismatch == 0 && endpoint_mismatch == 0,
        "Klei's walk, probed one cell at a time, visits one cell per step of the major axis, "
        "starting at the caller's start and ending at its end -- in both directions");
  printf("  the ignore mask changed the answer of %d rays\n", ignore_mattered);
  Check(reversed_rays > 0 && stopped > 0 && stopped_reversed > 0 && stopped < rays &&
            ignore_mattered > 0,
        "the sample actually exercises the high-end start, stopped rays, stopped high-end rays, "
        "rays that run clear and rays the ignore mask let through -- a reference that never "
        "disagrees about nothing proves nothing");
  Check(result_mismatch == 0,
        "every ray's hit, last clear cell, visited count and transmission match the reference "
        "built from Klei's own cell set, under eight stop conditions, cast both ways");
  Check(unmasked_bit_mismatch == 0,
        "with nothing to stop it the transmission IS RadiationAbsorptionAlongLine's float, bit for "
        "bit -- cast from either end");
  Check(unmasked_shape_mismatch == 0,
        "and with nothing to stop it the ray reports no hit, the caller's end as its last clear "
        "cell, and the whole line visited");

  // The start cell counts. A ray cast from inside a solid is stopped where it starts, with no
  // clear cell before it -- the case a caller reads `lastClear == -1` for.
  {
    size_t solid = RaycastHit::kNoCell;
    for (int32_t c = 0; c < kW * kH && solid == RaycastHit::kNoCell; ++c) {
      const size_t p = world.Padded(static_cast<size_t>(c));
      // Not in the first or last column, so a ray can leave it in both directions.
      const int32_t x = static_cast<int32_t>(p % pw);
      if (world.Phase(p).element == 3 && x > 1 && x < kW) solid = p;
    }
    const int32_t sx = static_cast<int32_t>(solid % pw), sy = static_cast<int32_t>(solid / pw);
    const RaycastHit r = Raycast(world, table, sx, sy, kW, sy, 1u << kStateSolid, 0u, 0u);
    Check(r.hit == solid && r.last_clear == RaycastHit::kNoCell && r.visited == 1,
          "a ray cast from inside a solid stops on its own start cell, with no clear cell before "
          "it and one cell visited");
    const RaycastHit back = Raycast(world, table, sx, sy, 1, sy, 1u << kStateSolid, 0u, 0u);
    Check(back.hit == solid && back.last_clear == RaycastHit::kNoCell && back.visited == 1,
          "and the same cast the other way, from the high end, where the answer comes out "
          "of the END of Klei's order rather than the start");
  }
}

void TestExtApiHeader() {
  printf("--- published C header (abi/sim_ext_api.h) ---\n");
  namespace ext = oni_sim::ext;
  struct NamePair { const char* c; const char* cxx; const char* what; };
  static const NamePair kNames[] = {
      {ONI_PROP_THERMAL_MASS_BONUS, ext::kThermalMassBonusProperty, "sim.thermal_mass_bonus"},
      {ONI_PROP_GAS_OCCUPIED_MASK, ext::kGasOccupiedMaskProperty, "sim.gas_occupied_mask"},
      {ONI_PROP_GAS_SPECIES, ext::kGasSpeciesProperty, "sim.gas_species"},
      {ONI_PROP_GAS_MASS, ext::kGasMassProperty, "sim.gas_mass"},
      {ONI_PROP_ROOM_PROMOTED, ext::kRoomPromotedProperty, "sim.room_promoted"},
      {ONI_ATTR_MOLECULAR_MASS, ext::kAttrMolecularMass, "sim.molecular_mass"},
      {ONI_ATTR_LATENT_FUSION, ext::kAttrLatentFusion, "sim.latent_fusion"},
      {ONI_STREAM_MESSAGE_REFUSED, ext::kStreamMessageRefused, "sim.message_refused"},
  };
  bool names_ok = true;
  for (size_t i = 0; i < sizeof(kNames) / sizeof(kNames[0]); ++i) {
    if (strcmp(kNames[i].c, kNames[i].cxx) != 0) {
      printf("    name %zu: C header says \"%s\", C++ header says \"%s\"\n", i, kNames[i].c,
             kNames[i].cxx);
      names_ok = false;
    }
  }
  Check(names_ok,
        "the eight well-known property/attribute/stream names are spelled identically in "
        "abi/sim_ext_api.h and abi/sim_abi_ext.h");

  // The id range, checked as arithmetic rather than as a restatement: a published header that
  // has 23 ids in it while the DLL's table has 24 is a header describing a DLL that no longer
  // exists, and the contiguity the DLL already asserts is what makes this subtraction legal.
  Check(ONI_MSG_LAST - ONI_MSG_FIRST + 1 - ONI_MSG_RESERVED_IDS == ext::kExtMessageCount &&
            ONI_MSG_RESERVED_IDS == ext::kExtMessageReservedIds,
        "the C header's id range covers exactly the number of messages the DLL publishes");
  printf("    (ids 0x%08X..0x%08X, %d messages; %d payload layouts across %d fields, and\n"
         "     %d constants, are checked at COMPILE time and cannot reach this line)\n",
         ONI_MSG_FIRST, ONI_MSG_LAST, ext::kExtMessageCount, 22, 55, 64);
}

// ===================================================================================
// The generic field solver (sim/fields.h, published as kRegisterField /
// kSetFieldSource / SIM_ExtField*).
//
// Its claim is narrow and checkable: a field is a registered arity-1 F32 cell property plus
// one of the walks the sim ALREADY performs, generalised. So the two walks are asserted
// against the functions they generalise -- `RadiationAbsorptionAlongLine` in sim/radiation.h
// and `ComputeSunBeam` in sim/textures.h -- rather than against numbers typed into this file.
// A copy of a loop header that is checked against its own original is a copy that cannot
// quietly drift; a copy checked against a table of expected floats is a copy that drifts the
// moment somebody re-records the table.
//
// And the two INVARIANTS are asserted with their faults
// planted, because an invariant nobody has seen fail is an invariant nobody has tested:
//
//   * a rule writes only inside the region rect -- the thing that keeps region-parallel
//     stepping possible;
//   * a rule never advances the shared LCG -- the thing that keeps `diffsim` byte-identity
//     true for a world that has registered a field.
// ===================================================================================

// Four elements, as MakeRaycastTable's, but carrying BOTH absorption factors and a maxMass,
// because the light law reads all three and the radiation law reads one of them.
ElementTable MakeFieldTable() {
  std::vector<Element> els(4);
  memset(els.data(), 0, els.size() * sizeof(Element));
  els[0].id = 0; els[0].state = kStateVacuum; els[0].radiationAbsorptionFactor = 0.0f;
  els[0].lightAbsorptionFactor = 0.0f;  els[0].maxMass = 0.0f;
  els[1].id = 1; els[1].state = kStateGas;    els[1].radiationAbsorptionFactor = 0.05f;
  els[1].lightAbsorptionFactor = 0.10f; els[1].maxMass = 2.0f;
  els[2].id = 2; els[2].state = kStateLiquid; els[2].radiationAbsorptionFactor = 0.3f;
  els[2].lightAbsorptionFactor = 0.45f; els[2].maxMass = 1000.0f;
  els[3].id = 3; els[3].state = kStateSolid;  els[3].radiationAbsorptionFactor = 0.8f;
  els[3].lightAbsorptionFactor = 0.9f;  els[3].maxMass = 0.0f;
  std::vector<uint8_t> blob(4 + els.size() * sizeof(Element));
  const int32_t n = static_cast<int32_t>(els.size());
  memcpy(blob.data(), &n, 4);
  memcpy(blob.data() + 4, els.data(), els.size() * sizeof(Element));
  ElementTable table;
  if (!table.Load(blob.data(), blob.size())) {
    printf("  FAIL: synthetic field ElementTable failed to load\n");
    ++g_fail_count;
  }
  return table;
}

// The random world both oracle arms walk: every phase, masses that straddle `maxMass`, and
// the property bits that make the constructed-tile branch of the radiation law run.
void FillFieldWorld(World& w, int32_t kW, int32_t kH) {
  uint32_t rng = 0x9E3779B9u;
  auto next = [&rng]() { rng = rng * 1664525u + 1013904223u; return rng >> 8; };
  for (int32_t c = 0; c < kW * kH; ++c) {
    const size_t p = w.Padded(static_cast<size_t>(c));
    const uint16_t e = static_cast<uint16_t>(next() % 4);
    w.Phase(p).element = e;
    w.Phase(p).mass = e == 0 ? 0.0f : static_cast<float>(next() % 3000) * 0.5f;
    uint8_t props = 0;
    if (e == 3 && next() % 3 == 0) props |= kConstructedTile;
    if (next() % 7 == 0) props |= kSolidImpermeable;
    w.MutableProperties(p) = props;
  }
}

// A registration message that is VALID, so each refusal arm below breaks exactly one field of
// it and the refusal it gets back names the thing it broke. A zeroed message is NOT valid --
// its clamp range is [0, 0], which would silently pin every value at zero -- and starting the
// arms from one would have made "refused" and "accepted but useless" the same shape.
ext::RegisterFieldMessage GoodFieldMessage(int32_t property_idx, int32_t attribute_idx) {
  ext::RegisterFieldMessage m{};
  m.propertyIdx = property_idx;
  m.attributeIdx = attribute_idx;
  m.attributeFallback = 0.0f;
  m.law = ext::kAttenFlat;
  m.combine = ext::kCombineTransmission;
  m.decayMode = ext::kDecayNone;
  m.decayKeep = 1.0f;
  m.floorValue = 0.0f;
  m.clampLo = 0.0f;
  m.clampHi = 1.0f;
  return m;
}

void TestFieldSolver() {
  printf("\n=== generic field solver: a property that has been given a rule ===\n");

  // -------------------------------------------------------------------- registration
  {
    World reg_world;
    ext::CellPropertyRegistry& props = reg_world.ExtCells();
    ElementTable table = MakeFieldTable();
    ext::ElementAttributeRegistry& attrs = table.MutableAttributes();
    std::string err;

    const int32_t p_ok = props.Register("test.field_ok", ext::kExtF32, ext::kRehydrated, 1, 0,
                                        false, &err);
    const int32_t p_two = props.Register("test.field_two", ext::kExtF32, ext::kRehydrated, 1, 0,
                                         false, &err);
    const int32_t p_i32 = props.Register("test.field_i32", ext::kExtI32, ext::kRehydrated, 1, 0,
                                         false, &err);
    const int32_t p_vec = props.Register("test.field_vec", ext::kExtF32, ext::kRehydrated, 4, 0,
                                         false, &err);
    const int32_t a_ok = attrs.Register("test.field_atten", ext::kExtF32, 1, false, &err);
    Check(p_ok >= 0 && p_two >= 0 && p_i32 >= 0 && p_vec >= 0 && a_ok >= 0,
          "the four properties and one attribute these arms are built on all registered");

    ext::FieldRegistry reg;
    err.clear();
    Check(reg.Register(props, attrs, GoodFieldMessage(p_ok, a_ok), &err) == 0 && err.empty(),
          "a field over an F32 arity-1 property with a registered attribute is field 0");

    // Each of these breaks exactly ONE field of the message above.
    struct Refusal {
      const char* what;
      ext::ExtRegisterResult code;
      ext::RegisterFieldMessage m;
    };
    ext::RegisterFieldMessage dup = GoodFieldMessage(p_ok, a_ok);
    ext::RegisterFieldMessage no_prop = GoodFieldMessage(9999, a_ok);
    ext::RegisterFieldMessage wrong_type = GoodFieldMessage(p_i32, a_ok);
    ext::RegisterFieldMessage wrong_arity = GoodFieldMessage(p_vec, a_ok);
    ext::RegisterFieldMessage no_attr = GoodFieldMessage(p_two, 9999);
    ext::RegisterFieldMessage bad_law = GoodFieldMessage(p_two, a_ok);
    bad_law.law = 99;
    ext::RegisterFieldMessage bad_combine = GoodFieldMessage(p_two, a_ok);
    bad_combine.combine = 99;
    ext::RegisterFieldMessage bad_decay = GoodFieldMessage(p_two, a_ok);
    bad_decay.decayMode = 99;
    ext::RegisterFieldMessage big_keep = GoodFieldMessage(p_two, a_ok);
    big_keep.decayKeep = 1.5f;
    ext::RegisterFieldMessage nan_keep = GoodFieldMessage(p_two, a_ok);
    nan_keep.decayKeep = std::numeric_limits<float>::quiet_NaN();
    ext::RegisterFieldMessage flipped = GoodFieldMessage(p_two, a_ok);
    flipped.clampLo = 1.0f;
    flipped.clampHi = 0.0f;

    const Refusal refusals[] = {
        {"a second field on the same property", ext::kExtRegisterDuplicate, dup},
        {"a property index nothing registered", ext::kExtRegisterBadProperty, no_prop},
        {"an I32 property -- a field is a float field", ext::kExtRegisterBadProperty,
         wrong_type},
        {"an arity-4 property -- a field is one value per cell", ext::kExtRegisterBadProperty,
         wrong_arity},
        {"an attribute index nothing registered", ext::kExtRegisterBadAttribute, no_attr},
        {"a law outside the enum", ext::kExtRegisterBadRule, bad_law},
        {"a combine mode outside the enum", ext::kExtRegisterBadRule, bad_combine},
        {"a decay mode outside the enum", ext::kExtRegisterBadRule, bad_decay},
        {"a decay factor above 1", ext::kExtRegisterBadRule, big_keep},
        {"a decay factor that is NaN", ext::kExtRegisterBadRule, nan_keep},
        {"a clamp range with its ends swapped", ext::kExtRegisterBadRule, flipped},
    };
    bool refusals_ok = true, reasons_ok = true;
    for (const Refusal& r : refusals) {
      err.clear();
      const int32_t got = reg.Register(props, attrs, r.m, &err);
      if (got != -static_cast<int32_t>(r.code)) {
        printf("    %s: got %d, expected %d\n", r.what, got, -static_cast<int32_t>(r.code));
        refusals_ok = false;
      }
      // A refusal that does not say why is a refusal a mod author cannot act on -- the reason
      // three codes exist here rather than one.
      if (err.empty()) {
        printf("    %s: refused with an empty reason\n", r.what);
        reasons_ok = false;
      }
    }
    Check(refusals_ok, "every bad registration is refused with the code that names what was "
                       "wrong, not one general failure");
    Check(reasons_ok, "and every refusal carries a written reason");

    err.clear();
    ext::RegisterFieldMessage no_atten = GoodFieldMessage(p_two, -1);
    Check(reg.Register(props, attrs, no_atten, &err) == 1,
          "attribute -1 is ACCEPTED: a field that asked for no attenuation is not a field "
          "with a bad attribute");
    Check(reg.Count() == 2, "and the two accepted registrations are the only two rows");
    Check(reg.FindByProperty(p_ok) == 0 && reg.FindByProperty(p_i32) == -1,
          "FindByProperty answers for a property that has a field and -1 for one that does "
          "not -- the export's whole lookup path");

    // The cap. Registered on fresh properties, because the duplicate rule would refuse them
    // first and this arm is about the cap rather than about the duplicate.
    ext::FieldRegistry full;
    bool cap_ok = true;
    for (int32_t i = 0; i < ext::kExtMaxFields; ++i) {
      char name[64];
      snprintf(name, sizeof(name), "test.field_cap%d", i);
      const int32_t pi = props.Register(name, ext::kExtF32, ext::kRehydrated, 1, 0, false, &err);
      if (pi < 0 || full.Register(props, attrs, GoodFieldMessage(pi, -1), &err) != i) {
        cap_ok = false;
        break;
      }
    }
    err.clear();
    const int32_t over = props.Register("test.field_over", ext::kExtF32, ext::kRehydrated, 1, 0,
                                        false, &err);
    err.clear();
    Check(cap_ok && full.Register(props, attrs, GoodFieldMessage(over, -1), &err) ==
                        -static_cast<int32_t>(ext::kExtRegisterFull),
          "the 33rd field is refused as full rather than allocated -- a mod registering in a "
          "loop hits a wall with a name in the log");
  }

  // ------------------------------------------------------------------------ sources
  {
    World reg_world;
    ext::CellPropertyRegistry& props = reg_world.ExtCells();
    ElementTable table = MakeFieldTable();
    std::string err;
    const int32_t pi = props.Register("test.field_src", ext::kExtF32, ext::kRehydrated, 1, 0,
                                      false, &err);
    ext::FieldRegistry reg;
    const int32_t fi = reg.Register(props, table.MutableAttributes(),
                                    GoodFieldMessage(pi, -1), &err);

    ext::FieldSource s;
    s.id = 7;
    s.kind = ext::kSourcePoint;
    s.strength = 2.0f;
    Check(!reg.SetSource(fi + 99, s),
          "a source for a field index nothing registered is refused, not dropped -- a field "
          "index comes back from a registration, so a bad one is a caller bug");
    Check(reg.SetSource(fi, s) && reg.At(fi)->sources.size() == 1,
          "the first source lands");
    s.strength = 5.0f;
    Check(reg.SetSource(fi, s) && reg.At(fi)->sources.size() == 1 &&
              reg.At(fi)->sources[0].strength == 5.0f,
          "re-sending the same owner id REPLACES rather than appends -- which is what makes "
          "the after-load re-push idempotent");
    s.strength = 0.0f;
    Check(reg.SetSource(fi, s) && reg.At(fi)->sources.empty(),
          "strength 0 removes the source");
    s.id = 404;
    Check(reg.SetSource(fi, s) && reg.At(fi)->sources.empty(),
          "and removing one that was never there is not an error: a source contributing "
          "nothing and a source that is not there are the same thing");

    bool source_cap_ok = true;
    for (int32_t i = 0; i < ext::kExtMaxFieldSources; ++i) {
      ext::FieldSource f;
      f.id = 1000 + i;
      f.strength = 1.0f;
      if (!reg.SetSource(fi, f)) { source_cap_ok = false; break; }
    }
    ext::FieldSource over;
    over.id = 9999;
    over.strength = 1.0f;
    Check(source_cap_ok && !reg.SetSource(fi, over),
          "the 65th source on one field is refused -- the cap that stops one field from "
          "silently costing a hundred ellipse rasters a substep");
  }

  // ------------------------------------------------- oracle 1: the field IS the radiation walk
  {
    constexpr int32_t kW = 13, kH = 9;
    ElementTable table = MakeFieldTable();
    World world;
    world.Allocate(kW, kH);
    FillFieldWorld(world, kW, kH);
    // Klei's own tunables, set to values that are not the defaults so the arm would notice a
    // law that ignored them. `RadiationMaxMass` is deliberately NOT zero here; the zero case
    // is its own arm below.
    world.SetRadiationParam(2, 0.2f);   // base weight
    world.SetRadiationParam(3, 0.7f);   // density weight
    world.SetRadiationParam(4, 0.35f);  // constructed factor
    world.SetRadiationParam(5, 800.0f); // max mass

    // The attribute the FIELD reads is a mirror of the element table's own
    // `radiationAbsorptionFactor`. That is the whole point of the arm: same numbers, one
    // reached through registry 2 and one out of Klei's struct, and the two walks must agree.
    std::string err;
    ext::ElementAttributeRegistry& attrs = table.MutableAttributes();
    const int32_t ai = attrs.Register("test.rad_mirror", ext::kExtF32, 1, false, &err);
    for (int32_t i = 0; i < table.Count(); ++i) {
      const Element& el = table.At(static_cast<uint16_t>(i));
      uint32_t bits = 0;
      const float v = el.radiationAbsorptionFactor;
      memcpy(&bits, &v, 4);
      attrs.Write(ai, el.id, 0, bits);
    }

    ext::Field f;
    f.property = 0;
    f.attribute = ai;
    f.fallback = 0.0f;
    f.law = ext::kAttenRadiationMass;
    f.combine = ext::kCombineTransmission;
    ext::FieldAttenuation atten;
    atten.Build(table, attrs, f);

    uint32_t rng = 0xB5297A4Du;
    auto next = [&rng]() { rng = rng * 1664525u + 1013904223u; return rng >> 8; };
    struct Line { int32_t x0, y0, x1, y1; };
    std::vector<Line> lines;
    lines.push_back({4, 4, 4, 4});
    lines.push_back({1, 3, kW, 3});
    lines.push_back({6, 1, 6, kH});
    lines.push_back({1, 1, 9, 9});
    lines.push_back({1, 9, 9, 1});
    lines.push_back({3, 3, 4, 5});
    while (lines.size() < 300) {
      lines.push_back({static_cast<int32_t>(1 + next() % kW),
                       static_cast<int32_t>(1 + next() % kH),
                       static_cast<int32_t>(1 + next() % kW),
                       static_cast<int32_t>(1 + next() % kH)});
    }

    int mismatches = 0, dimmed = 0;
    for (const Line& l : lines) {
      const float klei = RadiationAbsorptionAlongLine(world, table, l.x0, l.y0, l.x1, l.y1);
      const float mine = ext::FieldAttenuationAlongLine(world, table, atten, f,
                                                        l.x0, l.y0, l.x1, l.y1);
      if (memcmp(&klei, &mine, sizeof(float)) != 0) {
        if (mismatches < 4) {
          printf("    line (%d,%d)->(%d,%d): Klei %.9g, field %.9g\n", l.x0, l.y0, l.x1, l.y1,
                 static_cast<double>(klei), static_cast<double>(mine));
        }
        ++mismatches;
      }
      if (klei < 1.0f) ++dimmed;
    }
    Check(mismatches == 0,
          "a field with the radiation law and the transmission combine IS "
          "RadiationAbsorptionAlongLine, bit for bit, over 300 lines -- which is what keeps "
          "the third copy of Klei's Bresenham header in this tree honest");
    // A green arm over 300 lines that never attenuated anything would prove only that two
    // functions both return 1.0.
    Check(dimmed > 200, "and the lines actually crossed something: most of them came back "
                        "attenuated, so the agreement is not two functions both returning 1");

    // THE NaN CORNER, and the reason `FieldAbsorbed` does not take its transparent-element
    // shortcut for this law. With max mass zero, a transparent cell's mass term is 0 * inf,
    // and the game's SSE min/max clamp returns 1.0 for the NaN where a comparison clamp -- or an
    // early return of 0 -- would call the cell transparent. Opaque and transparent are not a
    // rounding difference.
    World zero_mass;
    zero_mass.Allocate(kW, kH);
    FillFieldWorld(zero_mass, kW, kH);
    zero_mass.SetRadiationParam(5, 0.0f);
    int nan_mismatches = 0;
    for (const Line& l : lines) {
      const float klei =
          RadiationAbsorptionAlongLine(zero_mass, table, l.x0, l.y0, l.x1, l.y1);
      const float mine = ext::FieldAttenuationAlongLine(zero_mass, table, atten, f,
                                                        l.x0, l.y0, l.x1, l.y1);
      if (memcmp(&klei, &mine, sizeof(float)) != 0) ++nan_mismatches;
    }
    Check(nan_mismatches == 0,
          "and it still agrees with a radiation max mass of ZERO, where the absorption is a "
          "NaN that Klei's clamp turns into a fully opaque cell");
  }

  // ------------------------------------------------- oracle 2: the field IS the sun beam
  {
    constexpr int32_t kW = 21, kH = 15;
    ElementTable table = MakeFieldTable();
    World world;
    world.Allocate(kW, kH);
    FillFieldWorld(world, kW, kH);
    World::WorldOffset off;
    off.x = 0; off.y = 0; off.w = kW; off.h = kH;
    world.AddWorldOffset(off);

    std::string err;
    ext::ElementAttributeRegistry& attrs = table.MutableAttributes();
    const int32_t ai = attrs.Register("test.light_mirror", ext::kExtF32, 1, false, &err);
    for (int32_t i = 0; i < table.Count(); ++i) {
      const Element& el = table.At(static_cast<uint16_t>(i));
      uint32_t bits = 0;
      const float v = el.lightAbsorptionFactor;
      memcpy(&bits, &v, 4);
      attrs.Write(ai, el.id, 0, bits);
    }

    ext::Field f;
    f.property = 0;
    f.attribute = ai;
    f.law = ext::kAttenLightMass;
    f.combine = ext::kCombineExposure;
    f.decay_mode = ext::kDecayNone;
    f.floor_value = 0.0f;
    f.clamp_lo = 0.0f;
    f.clamp_hi = 1.0f;
    ext::FieldAttenuation atten;
    atten.Build(table, attrs, f);

    // The projection the sun beam reads, filled from the same cells the field walks. Without
    // this the two would be reading two different grids and the comparison would mean nothing.
    ProjectionBuffers projected;
    projected.Allocate(static_cast<size_t>(kW) * static_cast<size_t>(kH));
    for (int32_t c = 0; c < kW * kH; ++c) {
      const size_t p = world.Padded(static_cast<size_t>(c));
      projected.element[static_cast<size_t>(c)] = world.Phases()[p].element;
      projected.mass[static_cast<size_t>(c)] = world.Phases()[p].mass;
    }

    // Overhead, both diagonals within 45 degrees, and both low suns -- the row branch, the
    // column branch and the two sides each one can arrive from.
    const World::SunDirection suns[] = {
        {0.0f, 1.0f}, {0.4f, 1.0f}, {-0.4f, 1.0f}, {1.0f, 0.35f}, {-1.0f, 0.35f},
    };
    int beam_mismatches = 0, lit_cells = 0, dark_cells = 0;
    std::vector<float> klei_lanes, field_lanes;
    std::vector<uint8_t> beam(static_cast<size_t>(kW) * static_cast<size_t>(kH), 0);
    std::vector<float> values(world.PaddedCount(), 0.0f);
    const World::PaddedRect whole{1, 1, kW + 1, kH + 1};
    for (const World::SunDirection& sun : suns) {
      world.SetWorldSun(0, sun);
      ComputeSunBeam(world, table, projected, &klei_lanes, &beam);

      std::fill(values.begin(), values.end(), 0.0f);
      ext::FieldSource s;
      s.id = 1;
      s.kind = ext::kSourceDirectional;
      s.strength = 1.0f;
      s.target = 0;
      s.dir_x = sun.dir_x;
      s.dir_y = sun.dir_y;
      ext::StepFieldDirectional(world, table, atten, f, s, values.data(), whole, &field_lanes);

      for (int32_t c = 0; c < kW * kH; ++c) {
        const size_t p = world.Padded(static_cast<size_t>(c));
        // Klei's texture is a byte; the field's value is the float behind it. The comparison
        // is therefore the byte Klei writes against the byte the field's value would produce,
        // truncated exactly as `ComputeSunBeam` truncates it.
        const uint8_t mine = values[p] > 0.0f ? static_cast<uint8_t>(values[p] * 255.0f) : 0;
        if (mine != beam[static_cast<size_t>(c)]) {
          if (beam_mismatches < 4) {
            printf("    sun (%.2f,%.2f) cell %d: beam %u, field %u\n",
                   static_cast<double>(sun.dir_x), static_cast<double>(sun.dir_y), c,
                   beam[static_cast<size_t>(c)], mine);
          }
          ++beam_mismatches;
        }
        if (beam[static_cast<size_t>(c)] != 0) ++lit_cells; else ++dark_cells;
      }
    }
    Check(beam_mismatches == 0,
          "a field with the light law and the exposure combine IS ComputeSunBeam over five "
          "sun directions -- the row branch, the column branch, and both sides each arrives "
          "from");
    Check(lit_cells > 0 && dark_cells > 0,
          "and the world actually cast shadows: both lit and dark cells are present, so the "
          "agreement is not two functions both writing zero");
  }

  // ----------------------------------------------- invariant 1: writes stay in the region
  {
    constexpr int32_t kW = 21, kH = 15;
    ElementTable table = MakeFieldTable();
    World world;
    world.Allocate(kW, kH);
    FillFieldWorld(world, kW, kH);

    ext::Field f;
    f.property = 0;
    f.attribute = -1;
    f.law = ext::kAttenFlat;
    f.combine = ext::kCombineTransmission;
    f.clamp_lo = 0.0f;
    f.clamp_hi = 1e9f;
    ext::FieldAttenuation atten;
    atten.Build(table, table.Attributes(), f);

    // A region covering a quarter of the world, and a source at its centre whose radius
    // reaches well past every edge of it.
    const World::PaddedRect rect{3, 3, 11, 9};
    ext::FieldSource s;
    s.id = 1;
    s.kind = ext::kSourcePoint;
    s.strength = 100.0f;
    s.target = static_cast<int32_t>(6 * world.PaddedWidth() + 7);
    s.radius_x = 9;
    s.radius_y = 9;
    s.cone_angle = 360.0f;

    std::vector<float> values(world.PaddedCount(), 0.0f);
    ext::StepFieldPoint(world, table, atten, f, s, values.data(), rect);

    int outside = 0, inside = 0;
    for (int32_t y = 0; y < world.PaddedHeight(); ++y) {
      for (int32_t x = 0; x < world.PaddedWidth(); ++x) {
        const size_t p = static_cast<size_t>(y) * static_cast<size_t>(world.PaddedWidth()) +
                         static_cast<size_t>(x);
        if (values[p] == 0.0f) continue;
        const bool in = x >= rect.x0 && y >= rect.y0 && x < rect.x1 && y < rect.y1;
        if (in) ++inside; else ++outside;
      }
    }
    Check(outside == 0,
          "a point source whose radius reaches past every edge of its region writes NOTHING "
          "outside the region rect -- the invariant region-parallel stepping needs");
    Check(inside > 0, "and it did write inside it, so the arm is not passing on an empty run");

    // THE FAULT, PLANTED. The same loop with the clip removed, so that the check above is
    // shown to fail when the invariant is broken rather than merely to pass when it holds.
    std::vector<float> unclipped(world.PaddedCount(), 0.0f);
    const World::PaddedRect no_clip{0, 0, world.PaddedWidth(), world.PaddedHeight()};
    ext::StepFieldPoint(world, table, atten, f, s, unclipped.data(), no_clip);
    int fault_outside = 0;
    for (int32_t y = 0; y < world.PaddedHeight(); ++y) {
      for (int32_t x = 0; x < world.PaddedWidth(); ++x) {
        const size_t p = static_cast<size_t>(y) * static_cast<size_t>(world.PaddedWidth()) +
                         static_cast<size_t>(x);
        if (unclipped[p] == 0.0f) continue;
        if (!(x >= rect.x0 && y >= rect.y0 && x < rect.x1 && y < rect.y1)) ++fault_outside;
      }
    }
    Check(fault_outside > 0,
          "and the same source given the WHOLE world as its rect writes outside that region "
          "-- the check above has teeth");

    // `FieldAdd` is the one write path, and it refuses on its own rather than trusting the
    // loop that called it. A rule that forgot the clip entirely still cannot write out.
    std::vector<float> direct(world.PaddedCount(), 0.0f);
    ext::FieldAdd(direct.data(), rect, world.PaddedWidth(), f, rect.x0 - 1, rect.y0, 1.0f);
    ext::FieldAdd(direct.data(), rect, world.PaddedWidth(), f, rect.x1, rect.y0, 1.0f);
    ext::FieldAdd(direct.data(), rect, world.PaddedWidth(), f, rect.x0, rect.y1, 1.0f);
    bool untouched = true;
    for (float v : direct) { if (v != 0.0f) untouched = false; }
    ext::FieldAdd(direct.data(), rect, world.PaddedWidth(), f, rect.x0, rect.y0, 1.0f);
    Check(untouched &&
              direct[static_cast<size_t>(rect.y0) *
                     static_cast<size_t>(world.PaddedWidth()) +
                     static_cast<size_t>(rect.x0)] == 1.0f,
          "FieldAdd itself refuses three out-of-rect coordinates and accepts the corner -- "
          "the clip lives in the write path, not in each rule's loop");
  }

  // --------------------------------------- invariant 2: the shared LCG is never advanced
  {
    constexpr int32_t kW = 21, kH = 15;
    ElementTable table = MakeFieldTable();
    World world;
    world.Allocate(kW, kH);
    FillFieldWorld(world, kW, kH);
    World::WorldOffset off;
    off.x = 0; off.y = 0; off.w = kW; off.h = kH;
    world.AddWorldOffset(off);
    world.SetWorldSun(0, {0.3f, 1.0f});

    std::string err;
    ext::CellPropertyRegistry& props = world.ExtCells();
    // Registration closes at Allocate, so this world gets its property before it is sized --
    // which is the rule every extension property lives under, not a quirk of this test.
    World lcg;
    ext::CellPropertyRegistry& lcg_props = lcg.ExtCells();
    const int32_t pi = lcg_props.Register("test.field_lcg", ext::kExtF32, ext::kRehydrated, 1,
                                          0, false, &err);
    lcg.Allocate(kW, kH);
    FillFieldWorld(lcg, kW, kH);
    lcg.AddWorldOffset(off);
    lcg.SetWorldSun(0, {0.3f, 1.0f});
    (void)props;

    ext::FieldState state;
    ext::RegisterFieldMessage m = GoodFieldMessage(pi, -1);
    m.decayMode = ext::kDecayFactor;
    m.decayKeep = 0.5f;
    m.clampHi = 1e9f;
    const int32_t fi = state.registry.Register(lcg_props, table.Attributes(), m, &err);

    // One source of every kind, so that every walk in the file runs before the state is read.
    ext::FieldSource point;
    point.id = 1;
    point.kind = ext::kSourcePoint;
    point.strength = 10.0f;
    point.target = static_cast<int32_t>(7 * lcg.PaddedWidth() + 9);
    point.radius_x = 6;
    point.radius_y = 4;
    point.cone_angle = 360.0f;
    ext::FieldSource beam;
    beam.id = 2;
    beam.kind = ext::kSourceDirectional;
    beam.strength = 1.0f;
    beam.target = 0;
    beam.dir_x = 0.3f;
    beam.dir_y = 1.0f;
    ext::FieldSource elem;
    elem.id = 3;
    elem.kind = ext::kSourceElement;
    elem.strength = 0.01f;
    elem.target = 3;
    state.registry.SetSource(fi, point);
    state.registry.SetSource(fi, beam);
    state.registry.SetSource(fi, elem);

    const uint32_t seed = 0x1234ABCDu;
    lcg.SetRandomState(seed);
    for (size_t ri = 0; ri < lcg.RegionCount(); ++ri) {
      ext::StepFields(&lcg, table, &lcg_props, table.Attributes(), &state, ri);
    }
    Check(lcg.RandomState() == seed,
          "a field that stepped a point source, a directional sweep, an element source and a "
          "decay pass left the shared LCG exactly where it found it -- which is why a world "
          "with a field registered is still byte-identical to vanilla in diffsim");

    const float* stepped = lcg_props.MutableF32(pi);
    int written = 0;
    for (size_t i = 0; i < lcg.PaddedCount(); ++i) { if (stepped[i] != 0.0f) ++written; }
    Check(written > 0, "and it did step: the field is not empty, so the LCG arm is not "
                       "passing because nothing ran");

    // THE FAULT, PLANTED: one draw from the stream this phase is forbidden to touch, so the
    // check above is shown to be capable of failing.
    const uint32_t before = lcg.RandomState();
    (void)lcg.NextRandomState();
    Check(lcg.RandomState() != before,
          "and a single NextRandomState() call DOES move it -- the check above has teeth");
  }

  // ------------------------------------- the two parameters are load-bearing, not decoration
  {
    constexpr int32_t kW = 13, kH = 9;
    ElementTable table = MakeFieldTable();
    World world;
    world.Allocate(kW, kH);
    FillFieldWorld(world, kW, kH);

    std::string err;
    ext::ElementAttributeRegistry& attrs = table.MutableAttributes();
    const int32_t ai = attrs.Register("test.both_laws", ext::kExtF32, 1, false, &err);
    for (int32_t i = 0; i < table.Count(); ++i) {
      const Element& el = table.At(static_cast<uint16_t>(i));
      uint32_t bits = 0;
      const float v = el.radiationAbsorptionFactor;
      memcpy(&bits, &v, 4);
      attrs.Write(ai, el.id, 0, bits);
    }

    ext::Field t_field;
    t_field.property = 0;
    t_field.attribute = ai;
    t_field.law = ext::kAttenFlat;
    t_field.combine = ext::kCombineTransmission;
    ext::Field e_field = t_field;
    e_field.combine = ext::kCombineExposure;
    ext::FieldAttenuation atten;
    atten.Build(table, attrs, t_field);

    int differing = 0;
    for (int32_t y = 1; y <= kH; ++y) {
      const float a = ext::FieldAttenuationAlongLine(world, table, atten, t_field, 1, y, kW, y);
      const float b = ext::FieldAttenuationAlongLine(world, table, atten, e_field, 1, y, kW, y);
      if (memcmp(&a, &b, sizeof(float)) != 0) ++differing;
    }
    Check(differing > 0,
          "transmission and exposure are NOT the same walk with a different name: the same "
          "line through the same world gives different carriers");

    // The same for the laws: a flat law and the radiation law over one attribute disagree,
    // because the second mixes the cell's mass in.
    ext::Field r_field = t_field;
    r_field.law = ext::kAttenRadiationMass;
    world.SetRadiationParam(5, 800.0f);
    int law_differing = 0;
    for (int32_t y = 1; y <= kH; ++y) {
      const float a = ext::FieldAttenuationAlongLine(world, table, atten, t_field, 1, y, kW, y);
      const float b = ext::FieldAttenuationAlongLine(world, table, atten, r_field, 1, y, kW, y);
      if (memcmp(&a, &b, sizeof(float)) != 0) ++law_differing;
    }
    Check(law_differing > 0,
          "and the flat law and the radiation law disagree over the same attribute -- the "
          "law is the mass mixing, not a label");

    // Decay applied twice is not decay applied once. The arm exists because `StepFields`
    // runs the decay pass once per field per region, and a field whose regions overlapped
    // would decay a cell twice in one substep -- which is the failure a region rework would
    // introduce and nothing else in this file would see.
    ext::Field d_field = t_field;
    d_field.decay_mode = ext::kDecayFactor;
    d_field.decay_keep = 0.5f;
    d_field.clamp_hi = 1e9f;
    const World::PaddedRect rect{1, 1, kW + 1, kH + 1};
    std::vector<float> once(world.PaddedCount(), 4.0f);
    std::vector<float> twice(world.PaddedCount(), 4.0f);
    ext::StepFieldDecay(world, d_field, once.data(), rect);
    ext::StepFieldDecay(world, d_field, twice.data(), rect);
    ext::StepFieldDecay(world, d_field, twice.data(), rect);
    const size_t probe = static_cast<size_t>(world.PaddedWidth()) + 1;
    Check(once[probe] == 2.0f && twice[probe] == 1.0f,
          "decay applied twice is not decay applied once -- a region rework that stepped a "
          "cell in two regions would halve it twice, and this is what would say so");
  }

  // -------------------------------------------------------------- an idle field does nothing
  {
    constexpr int32_t kW = 13, kH = 9;
    ElementTable table = MakeFieldTable();
    std::string err;
    World world;
    ext::CellPropertyRegistry& props = world.ExtCells();
    const int32_t pi = props.Register("test.field_idle", ext::kExtF32, ext::kRehydrated, 1, 0,
                                      false, &err);
    world.Allocate(kW, kH);
    FillFieldWorld(world, kW, kH);

    ext::FieldState state;
    const int32_t fi = state.registry.Register(props, table.Attributes(),
                                               GoodFieldMessage(pi, -1), &err);
    Check(fi == 0 && state.registry.At(fi)->Idle(),
          "a field with no decay and no source reports itself idle");

    float* values = props.MutableF32(pi);
    for (size_t i = 0; i < world.PaddedCount(); ++i) {
      values[i] = static_cast<float>(i % 7) * 0.25f;
    }
    std::vector<float> before(values, values + world.PaddedCount());
    for (size_t ri = 0; ri < world.RegionCount(); ++ri) {
      ext::StepFields(&world, table, &props, table.Attributes(), &state, ri);
    }
    Check(memcmp(before.data(), values, before.size() * sizeof(float)) == 0,
          "and stepping it changes not one byte: an idle field still HOLDS its values and a "
          "mod can still read and write them -- it simply is not swept");
  }
}

}  // namespace

// Physical latent heats: the planetary
// accumulator's per-transition energy, one arm per branch, against hand arithmetic.
//
// `LatentEnergyOfTransition` is handed the element the cell WAS and reads the one it is NOW, so
// each arm writes the "now" element into the cell and passes the "was" -- no `TransitionCell`,
// because what is under test is the billing, not the threshold. The values are STATED, not read
// back out of the table: what matters is that the sim multiplies by the number it was given and
// adds the right two, which an oracle re-reading the attribute could not tell.
void TestLatentEnergyOfTransition() {
  printf("\n=== planetary latent accumulator: vaporization, fusion, and both ===\n");
  constexpr uint16_t kGasCO2 = 1, kLiqCO2 = 2, kLiqN2 = 3, kSolid = 4;
  // CO2's physical values at the registry's 44.01 g/mol, rounded: 15,420 J/mol and 9,019 J/mol.
  constexpr double kVap = 350000.0;
  constexpr double kFus = 205000.0;
  constexpr float kMass = 2.0f;

  World w;
  w.Allocate(3, 3);
  const int32_t px = 1 + 1, py = 1 + 1;
  const size_t mid = w.Padded(1 + 1 * 3);
  std::vector<uint8_t> sky(9, 255);

  auto energy = [&](const ElementTable& table, uint16_t was, uint16_t now) {
    w.Phase(mid).element = now;
    w.Phase(mid).mass = kMass;
    w.Phase(mid).temperature = 200.0f;
    return LatentEnergyOfTransition(w, table, mid, was, px, py, 3, 3, sky);
  };
  auto near = [](double got, double want) {
    return want == 0.0 ? got == 0.0 : std::fabs(got - want) <= std::fabs(want) * 1.0e-6;
  };

  // Liquid CO2 freezes in this table (element 4), so all six branches are reachable.
  ElementTable table = MakeCondensationTable(/*liquid_co2_freezes=*/true);
  WriteCurve(table, kGasCO2, kCO2CurveA, kCO2CurveB, 217.82f, 266.31f, static_cast<float>(kVap));
  WriteCurve(table, kLiqCO2, kCO2CurveA, kCO2CurveB, 217.82f, 266.31f, static_cast<float>(kVap));

  // -------- CONTROL FIRST: the curve alone, which is what every framework before this one pushes.
  Check(!table.AnyLatentFusion(), "a table with curves and no fusion entry reports no fusion");
  Check(near(energy(table, kGasCO2, kLiqCO2), kMass * kVap),
        "condensation is billed its vaporization exactly as before");
  Check(energy(table, kLiqCO2, kSolid) == 0.0,
        "with no sim.latent_fusion a freeze contributes NOTHING, not the vaporization number");
  Check(energy(table, kGasCO2, kSolid) == 0.0,
        "and a deposition is not billed its vaporization half alone: a latent heat that is not "
        "fully described is not guessed at");

  // -------- Fusion described on every member, as `SendPhaseData` pushes it.
  for (uint16_t id : {kGasCO2, kLiqCO2, kSolid}) {
    table.MutableAttributes().WriteF32(table.LatentFusionAttribute(), id,
                                       static_cast<float>(kFus));
  }
  Check(table.AnyLatentFusion(), "and once written, the table reports it");
  float read_back = 0.0f;
  Check(table.LatentFusionOf(kLiqCO2, &read_back) && read_back == static_cast<float>(kFus),
        "LatentFusionOf reads the value back off the element");
  Check(!table.LatentFusionOf(kLiqN2, &read_back),
        "and reports ABSENT, not zero, for an element nobody described");

  struct Arm { const char* name; uint16_t was; uint16_t now; double want; };
  const Arm arms[] = {
      {"gas -> liquid   +m*L_vap", kGasCO2, kLiqCO2, kMass * kVap},
      {"liquid -> gas   -m*L_vap", kLiqCO2, kGasCO2, -kMass * kVap},
      {"liquid -> solid +m*L_fus", kLiqCO2, kSolid, kMass * kFus},
      {"solid -> liquid -m*L_fus", kSolid, kLiqCO2, -kMass * kFus},
      {"gas -> solid    +m*(L_vap+L_fus)", kGasCO2, kSolid, kMass * (kVap + kFus)},
      {"solid -> gas    -m*(L_vap+L_fus)", kSolid, kGasCO2, -kMass * (kVap + kFus)},
  };
  for (const Arm& a : arms) {
    const double got = energy(table, a.was, a.now);
    printf("  %-34s %+.6g J (hand %+.6g J)\n", a.name, got, a.want);
    Check(near(got, a.want), a.name);
  }

  // -------- READ OFF THE NON-SOLID MEMBER. A different fusion value on the solid must not move
  // a freeze or a melt; a different one on the gas must move only the gas <-> solid pair.
  table.MutableAttributes().WriteF32(table.LatentFusionAttribute(), kSolid, 1.0f);
  Check(near(energy(table, kLiqCO2, kSolid), kMass * kFus) &&
            near(energy(table, kSolid, kLiqCO2), -kMass * kFus),
        "a freeze and a melt read fusion off the LIQUID, never the solid");
  table.MutableAttributes().WriteF32(table.LatentFusionAttribute(), kGasCO2, 1000.0f);
  Check(near(energy(table, kGasCO2, kSolid), kMass * (kVap + 1000.0)) &&
            near(energy(table, kLiqCO2, kSolid), kMass * kFus),
        "a deposition reads fusion off the GAS, and that leaves the liquid's freeze alone");

  // -------- THE SKY GATE IS UNCHANGED for the new branches: weighted by the lit fraction, zero
  // when shaded.
  sky[1 * 3 + 1] = 51;  // 51/255 = 0.2
  Check(near(energy(table, kLiqCO2, kSolid), kMass * kFus * (51.0 / 255.0)),
        "a freeze under a partial sky is weighted by the lit fraction, as a condensation is");
  sky[1 * 3 + 1] = 0;
  Check(energy(table, kLiqCO2, kSolid) == 0.0, "and a shaded freeze is not the planet's at all");
}


// ---------------------------------------------------------------- what caps a column
//
// Layer C3 (`sim/effervescence.h`) reads the pressure on a column from the cell above it. The
// liquid kernel moves water a whole cell per substep and leaves the cell it vacated empty until
// gas flows back, so a pond under a full atmosphere shows a VACUUM cap for a few substeps at a
// time, and read at 0 Pa that gap would fizz water nowhere near saturated. (On the DISSOLVE rig
// the live census found this case rare -- 3 of 2210 false fizzes -- and a thin-gas cap the rest,
// which `TestEffervescenceThinPocket` covers.) Three two-cell columns of the same liquid, same
// temperature, same load, differing only in their cap:
//
//   x 1  GAS       gas directly above
//   x 4  BORROWED  an empty cell above, gas beside it and above it
//   x 7  VACUUM    an empty cell above, nothing beside or above it
//
// The kernel is driven directly, one pass, so no liquid kernel can refill the gap first -- the
// only way to hold a gap still long enough to test the rule on it.
void TestEffervescenceCap() {
  printf("\n=== C3 effervescence: a cap holding no gas is weighed by its neighbours ===\n");
  ElementTable table = MakeCondensationTable();
  constexpr uint16_t kGasCO2 = 1, kLiq = 2, kSolid = 4;
  constexpr int32_t W = 9, H = 6;
  constexpr float kT = 300.0f, kGasKg = 1.8f, kLiqKg = 1000.0f;
  constexpr float kH = 3.3e-4f, kVantHoff = 2400.0f, kMolar = 0.044f;

  // The surface pressure every non-vacuum column should see: 1.8 kg of CO2 over one cubic
  // metre (the liquid under it is a full 1000 kg at 1000 kg/m3, so it adds no free volume).
  const float gas_pa = kGasKg / kMolar * g_tunables.gas_constant_r * kT / 1.0f;
  const float henry = kH * std::exp(kVantHoff * (1.0f / kT - 1.0f / g_tunables.fizz_reference_k));

  struct Result {
    float by_col[3] = {0.0f, 0.0f, 0.0f};
    double census[ext::kEffervescenceCensusFields] = {};
    double released = 0.0;
  };
  auto run = [&](float load_x_top) -> Result {
    Result out;
    World w;
    w.Allocate(W, H);
    auto set = [&](int x, int y, uint16_t el, float kg) {
      PhaseEntry& e = w.Phase(w.Padded(static_cast<size_t>(x + y * W)));
      e.element = el;
      e.mass = kg;
      e.temperature = kT;
    };
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        // Solid under the cap row everywhere but the three columns; gas over the left two
        // thirds of the cap row and above; vacuum over the right third.
        if (y < 2) set(x, y, kSolid, 1000.0f);
        else if (x <= 5) set(x, y, kGasCO2, kGasKg);
        else set(x, y, kVacuum, 0.0f);
      }
    }
    const int cols[3] = {1, 4, 7};
    // The top cell's static capacity, which is what the load is a multiple of.
    const float top_pa = gas_pa + 0.5f * kLiqKg * g_tunables.standard_gravity;
    const float load = load_x_top * henry * top_pa * kMolar * 1.0f;
    float* d = w.DissolvedMassData();
    for (int c : cols) {
      for (int y = 0; y < 2; ++y) {
        set(c, y, kLiq, kLiqKg);
        d[w.Padded(static_cast<size_t>(c + y * W)) * ext::kDissolvedGasLanes] = load;
      }
    }
    set(4, 2, kVacuum, 0.0f);  // the gap: vacuum, with gas at (3,2), (5,2) and (4,3)
    w.ExtCells().NoteNonzero(w.DissolvedMassProperty());

    World::Effervescence& cfg = w.MutableEffervescenceConfig();
    cfg.enabled = true;
    cfg.margin = 0.10f;
    cfg.rate_per_second = 0.10f;
    cfg.period_seconds = 1.0f;
    cfg.min_release_kg = 0.0f;
    cfg.lane_henry[0] = kH;
    cfg.lane_vant_hoff_k[0] = kVantHoff;
    cfg.lane_molar_kg[0] = kMolar;
    cfg.solvent_count = 1;
    cfg.solvent_element[0] = kLiq;
    cfg.solvent_factor[0] = 1.0f;
    cfg.solvent_density[0] = 1000.0f;

    StepEffervescence(&w, table, 1.0f);
    for (const ext::LiquidPayloadReleased& r : w.PayloadReleased()) {
      const int x = r.cell % W;
      const int k = x == 1 ? 0 : x == 4 ? 1 : x == 7 ? 2 : -1;
      if (k >= 0) out.by_col[k] += r.amount;
      out.released += r.amount;
    }
    for (int i = 0; i < ext::kEffervescenceCensusFields; ++i) {
      out.census[i] = w.EffervescenceConfig().census[i];
    }
    return out;
  };

  // (a) Half of saturation: under gas it holds, in the gap it must hold too, under vacuum it
  // must go.
  const Result half = run(0.5f);
  printf("  load 0.5x: released gas %.4f g, borrowed %.4f g, vacuum %.4f g\n",
         half.by_col[0] * 1000.0f, half.by_col[1] * 1000.0f, half.by_col[2] * 1000.0f);
  printf("  census: passes %.0f, columns gas/borrowed/vacuum/confined %.0f/%.0f/%.0f/%.0f, "
         "fizzing cells %.0f/%.0f/%.0f\n",
         half.census[ext::kEffervescenceCensusPasses],
         half.census[ext::kEffervescenceCensusColumnsGas],
         half.census[ext::kEffervescenceCensusColumnsBorrowed],
         half.census[ext::kEffervescenceCensusColumnsVacuum],
         half.census[ext::kEffervescenceCensusColumnsConfined],
         half.census[ext::kEffervescenceCensusFizzGas],
         half.census[ext::kEffervescenceCensusFizzBorrowed],
         half.census[ext::kEffervescenceCensusFizzVacuum]);
  Check(half.by_col[0] == 0.0f, "half-saturated liquid under gas does not fizz");
  Check(half.by_col[1] == 0.0f,
        "and neither does the same liquid under an EMPTY cell with gas beside it -- the gap a "
        "sloshing pond leaves is weighed at its neighbours' pressure, not at 0 Pa");
  Check(half.by_col[2] > 0.0f,
        "but under genuine vacuum, with no gas beside or above the cap, it degasses");
  Check(half.census[ext::kEffervescenceCensusPasses] == 1.0 &&
            half.census[ext::kEffervescenceCensusColumnsGas] == 1.0 &&
            half.census[ext::kEffervescenceCensusColumnsBorrowed] == 1.0 &&
            half.census[ext::kEffervescenceCensusColumnsVacuum] == 1.0,
        "the census sorts the three columns into gas, borrowed and vacuum, one pass");
  Check(half.census[ext::kEffervescenceCensusFizzGas] == 0.0 &&
            half.census[ext::kEffervescenceCensusFizzBorrowed] == 0.0 &&
            half.census[ext::kEffervescenceCensusFizzVacuum] == 2.0,
        "and counts both vacuum-capped cells as the only fizzing cells");
  Check(std::fabs(half.census[ext::kEffervescenceCensusKgVacuum] - half.released) < 1e-9,
        "and its kilograms are exactly what went out on the released stream");

  // (b) Twice saturation: now all three fizz, and the gap column must release EXACTLY what the
  // gas column does -- the borrowed pressure is the neighbour's, not an approximation of it.
  const Result twice = run(2.0f);
  printf("  load 2x: released gas %.6f g, borrowed %.6f g, vacuum %.6f g\n",
         twice.by_col[0] * 1000.0f, twice.by_col[1] * 1000.0f, twice.by_col[2] * 1000.0f);
  Check(twice.by_col[0] > 0.0f, "twice-saturated liquid under gas fizzes");
  Check(twice.by_col[1] == twice.by_col[0],
        "and the gap column releases exactly what the gas column does");
  Check(twice.by_col[2] > twice.by_col[0],
        "and the vacuum column more than either");
  Check(twice.census[ext::kEffervescenceCensusFizzBorrowed] == 2.0 &&
            std::fabs(twice.census[ext::kEffervescenceCensusKgBorrowed] - twice.by_col[1]) < 1e-6,
        "and the census books the gap column's release as borrowed");
}

// ---------------------------------------------------------------- a thin pocket reads its column
//
// What the DISSOLVE rig's census found behind 2207 of 2210 false fizzes: the cell capping the
// water held a few GRAMS of CO2 -- the pocket the falling water had vacated, which ONI's gas
// spreading had not yet refilled from the full cells right above it -- and read alone it put the
// surface at a few hundred pascals. The surface now pools the gas column above the cap
// (`FizzSurfacePressure`, Stationeers' `Room.CacheRoomData` on a bounded column). Three two-cell
// columns of the same liquid, same load, differing only above the surface:
//
//   x 1  POCKET   7.9 g of CO2 in the cap, 1.8 kg in every cell above it
//   x 4  FULL     1.8 kg in the cap and above
//   x 7  SEALED   7.9 g in the cap and a solid lid right above it: the pocket IS the gas
void TestEffervescenceThinPocket() {
  printf("\n=== C3 effervescence: a thin pocket is read with the gas column above it ===\n");
  ElementTable table = MakeCondensationTable();
  constexpr uint16_t kGasCO2 = 1, kLiq = 2, kSolid = 4;
  constexpr int32_t W = 9, H = 14;
  constexpr float kT = 300.0f, kGasKg = 1.8f, kPocketKg = 0.0079f, kLiqKg = 1000.0f;
  constexpr float kH = 3.3e-4f, kVantHoff = 2400.0f, kMolar = 0.044f;
  const float henry = kH * std::exp(kVantHoff * (1.0f / kT - 1.0f / g_tunables.fizz_reference_k));
  const float full_pa = kGasKg / kMolar * g_tunables.gas_constant_r * kT;

  World w;
  w.Allocate(W, H);
  auto set = [&](int x, int y, uint16_t el, float kg) {
    PhaseEntry& e = w.Phase(w.Padded(static_cast<size_t>(x + y * W)));
    e.element = el;
    e.mass = kg;
    e.temperature = kT;
  };
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      if (y < 2) set(x, y, kSolid, 1000.0f);
      else set(x, y, kGasCO2, kGasKg);
    }
  }
  const int cols[3] = {1, 4, 7};
  const float top_pa = full_pa + 0.5f * kLiqKg * g_tunables.standard_gravity;
  const float load = 0.5f * henry * top_pa * kMolar * 1.0f;  // half of saturation under FULL gas
  float* d = w.DissolvedMassData();
  for (int c : cols) {
    for (int y = 0; y < 2; ++y) {
      set(c, y, kLiq, kLiqKg);
      d[w.Padded(static_cast<size_t>(c + y * W)) * ext::kDissolvedGasLanes] = load;
    }
  }
  set(1, 2, kGasCO2, kPocketKg);
  set(7, 2, kGasCO2, kPocketKg);
  set(7, 3, kSolid, 1000.0f);
  // Past the pooling bound of the pocket column: 1000x the gas, which must not be counted.
  for (int y = 2 + g_tunables.fizz_surface_column_cells; y < H; ++y) {
    set(1, y, kGasCO2, 1000.0f * kGasKg);
  }
  w.ExtCells().NoteNonzero(w.DissolvedMassProperty());

  World::Effervescence& cfg = w.MutableEffervescenceConfig();
  cfg.enabled = true;
  cfg.margin = 0.10f;
  cfg.rate_per_second = 0.10f;
  cfg.period_seconds = 1.0f;
  cfg.min_release_kg = 0.0f;
  cfg.lane_henry[0] = kH;
  cfg.lane_vant_hoff_k[0] = kVantHoff;
  cfg.lane_molar_kg[0] = kMolar;
  cfg.solvent_count = 1;
  cfg.solvent_element[0] = kLiq;
  cfg.solvent_factor[0] = 1.0f;
  cfg.solvent_density[0] = 1000.0f;

  // The readings themselves, against the ideal gas by hand.
  const float* dm = w.DissolvedMassData();
  const float pocket_alone = FizzGasPressure(w, cfg, table, w.Padded(1 + 2 * W), dm);
  // `FizzSurfacePressure` takes PADDED coordinates, as the kernel's walk does.
  auto pooled_at = [&](int x, int y) {
    const size_t p = w.Padded(static_cast<size_t>(x + y * W));
    const size_t pw = static_cast<size_t>(w.PaddedWidth());
    return FizzSurfacePressure(w, cfg, table, static_cast<int32_t>(p % pw),
                               static_cast<int32_t>(p / pw), dm);
  };
  const float pocket_pooled = pooled_at(1, 2);
  const float full_pooled = pooled_at(4, 2);
  const float sealed_pooled = pooled_at(7, 2);
  const double want_pooled =
      (kPocketKg + (g_tunables.fizz_surface_column_cells - 1) * static_cast<double>(kGasKg)) /
          kMolar *
      g_tunables.gas_constant_r * kT / g_tunables.fizz_surface_column_cells;
  printf("  pocket alone %.1f Pa, pooled %.1f Pa (want %.1f); full %.1f Pa (want %.1f); "
         "sealed %.1f Pa\n",
         pocket_alone, pocket_pooled, want_pooled, full_pooled, full_pa, sealed_pooled);
  Check(std::fabs(pocket_pooled - want_pooled) < 1e-4 * want_pooled,
        "a thin pocket pools with the gas above it: sum(n T) R / sum(V) over the column, and "
        "no further than the bound -- the 1000x gas past it is not counted");
  Check(full_pooled == FizzGasPressure(w, cfg, table, w.Padded(4 + 2 * W), dm),
        "a uniform column pools to exactly its single cell's reading");
  Check(sealed_pooled == pocket_alone,
        "and a pocket under a solid lid pools with nothing -- the pocket is all the gas there is");

  StepEffervescence(&w, table, 1.0f);
  float by_col[3] = {0.0f, 0.0f, 0.0f};
  for (const ext::LiquidPayloadReleased& r : w.PayloadReleased()) {
    const int x = r.cell % W;
    const int k = x == 1 ? 0 : x == 4 ? 1 : x == 7 ? 2 : -1;
    if (k >= 0) by_col[k] += r.amount;
  }
  printf("  load 0.5x of full: released pocket %.4f g, full %.4f g, sealed %.4f g\n",
         by_col[0] * 1000.0f, by_col[1] * 1000.0f, by_col[2] * 1000.0f);
  Check(by_col[1] == 0.0f, "half-saturated liquid under a full column holds its gas");
  Check(by_col[0] == 0.0f,
        "and so does the same liquid under a THIN POCKET with that column above it -- the "
        "DISSOLVE rig's false fizz");
  Check(by_col[2] > 0.0f,
        "but under a sealed pocket that really is a few grams of gas, it degasses");
}

// ---------------------------------------------------------------- the free surface outgasses
//
// A bubble needs the dissolved gas to beat the TOTAL pressure; a free surface only
// needs it to beat the PARTIAL pressure of that gas above. Three two-cell columns of the same
// liquid, each holding CO2 at HALF of what the total pressure keeps in solution -- so none of
// them fizzes -- under the same total pressure of different gas:
//
//   x 1  HELIUM   no CO2 above: the top cell outgasses all of its excess, which is all of it
//   x 4  CO2      the gas above is all CO2: half-saturated, nothing to give
//   x 7  MIXTURE  pure CO2 held in the MIXTURE layer, the cell itself reading vacuum (a promoted
//                 room), walled in so no neighbour can lend it a pressure: 0.29 read this as a
//                 vacuum surface and fizzed it; it must now read exactly like x 4
void TestEffervescenceSurface() {
  printf("\n=== C3 effervescence: a free surface trades gas with the gas above it ===\n");
  ElementTable table = MakeCondensationTable();
  constexpr uint16_t kGasCO2 = 1, kLiq = 2, kSolid = 4, kHelium = 5;
  constexpr int32_t W = 9, H = 8;
  constexpr float kT = 300.0f, kGasKg = 1.8f, kLiqKg = 1000.0f;
  constexpr float kH = 3.3e-4f, kVantHoff = 2400.0f, kMolar = 0.044f, kRate = 0.01f;
  const float henry = kH * std::exp(kVantHoff * (1.0f / kT - 1.0f / g_tunables.fizz_reference_k));
  const float full_pa = kGasKg / kMolar * g_tunables.gas_constant_r * kT;
  const float helium_kg = kGasKg / kMolar * 0.004f;  // the same moles, so the same pressure
  const float top_pa = full_pa + 0.5f * kLiqKg * g_tunables.standard_gravity;
  const float load = 0.5f * henry * top_pa * kMolar * 1.0f;
  const double leave = 1.0 - std::exp(-static_cast<double>(kRate));
  const size_t lanes = static_cast<size_t>(ext::kDissolvedGasLanes);

  // Columns x = 1 (helium above), 4 (CO2 in the grid above) and 7 (CO2 in the mixture layer of a
  // promoted room above), each two liquid cells under six gas cells.
  struct Options {
    bool long_form = true;
    float load_scale = 1.0f;     // 0: nothing dissolved anywhere
    float co2_cap_kg = kGasKg;   // the CO2 column's cap cell alone
    float co2_cap_k = kT;
  };
  struct Result {
    float top[3] = {0.0f, 0.0f, 0.0f};
    float bottom[3] = {0.0f, 0.0f, 0.0f};
    int surface_records = 0;
    int absorbed_records = 0;
    int fizz_records = 0;
    float mix_pa = 0.0f;
    float co2_cap_kg = 0.0f;
    float mix_cap_kg = 0.0f;
    float lane_top[3] = {0.0f, 0.0f, 0.0f};
    float liquid_top_k[3] = {0.0f, 0.0f, 0.0f};
    double ledger_kg = 0.0;
    double ledger_kj = 0.0;
    double census[ext::kEffervescenceCensusFields] = {};
  };
  auto run = [&](const Options& o) -> Result {
    Result out;
    World w;
    w.Allocate(W, H);
    auto set = [&](int x, int y, uint16_t el, float kg) {
      PhaseEntry& e = w.Phase(w.Padded(static_cast<size_t>(x + y * W)));
      e.element = el;
      e.mass = kg;
      e.temperature = kT;
    };
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        if (y < 2 || x == 6 || x == 8) set(x, y, kSolid, 1000.0f);
        else if (x <= 2) set(x, y, kHelium, helium_kg);
        else if (x <= 5) set(x, y, kGasCO2, kGasKg);
        else set(x, y, kVacuum, 0.0f);  // x 7: the facade of a promoted room
      }
    }
    for (int y = 2; y < H; ++y) {
      const size_t p = w.Padded(static_cast<size_t>(7 + y * W));
      w.MutableGasSpecies(p, 0) = kGasCO2;
      w.MutableGasMass(p, 0) = kGasKg;
      w.MutableGasOccupiedMask(p) = 1;
    }
    PhaseEntry& co2_cap = w.Phase(w.Padded(static_cast<size_t>(4 + 2 * W)));
    co2_cap.mass = o.co2_cap_kg;
    co2_cap.temperature = o.co2_cap_k;
    const int cols[3] = {1, 4, 7};
    float* d = w.DissolvedMassData();
    for (int c : cols) {
      for (int y = 0; y < 2; ++y) {
        set(c, y, kLiq, kLiqKg);
        d[w.Padded(static_cast<size_t>(c + y * W)) * lanes] = load * o.load_scale;
      }
    }
    if (o.load_scale > 0.0f) w.ExtCells().NoteNonzero(w.DissolvedMassProperty());

    World::Effervescence& cfg = w.MutableEffervescenceConfig();
    cfg.enabled = true;
    cfg.margin = 0.10f;
    cfg.rate_per_second = 0.10f;
    cfg.period_seconds = 1.0f;
    cfg.min_release_kg = 0.0f;
    cfg.lane_henry[0] = kH;
    cfg.lane_vant_hoff_k[0] = kVantHoff;
    cfg.lane_molar_kg[0] = kMolar;
    cfg.solvent_count = 1;
    cfg.solvent_element[0] = kLiq;
    cfg.solvent_factor[0] = 1.0f;
    cfg.solvent_density[0] = 1000.0f;
    if (o.long_form) {
      cfg.surface_rate_per_second = kRate;
      cfg.surface_min_release_kg = 0.0f;
      cfg.lane_element[0] = kGasCO2;
    }
    out.mix_pa = FizzGasPressure(w, cfg, table, w.Padded(7 + 2 * W), w.DissolvedMassData());

    StepEffervescence(&w, table, 1.0f);
    for (const ext::LiquidPayloadReleased& r : w.PayloadReleased()) {
      const int x = r.cell % W;
      const int y = r.cell / W;
      const int k = x == 1 ? 0 : x == 4 ? 1 : x == 7 ? 2 : -1;
      if (k < 0) continue;
      (y == 1 ? out.top : out.bottom)[k] += r.amount;
      if (r.reason == ext::kPayloadReleasedSurface) ++out.surface_records;
      if (r.reason == ext::kPayloadAbsorbedSurface) ++out.absorbed_records;
      if (r.reason == ext::kPayloadReleasedEffervescence) ++out.fizz_records;
    }
    out.co2_cap_kg = w.Phase(w.Padded(static_cast<size_t>(4 + 2 * W))).mass;
    out.mix_cap_kg = w.GasMass(w.Padded(static_cast<size_t>(7 + 2 * W)), 0);
    for (int k = 0; k < 3; ++k) {
      const size_t top = w.Padded(static_cast<size_t>(cols[k] + 1 * W));
      out.lane_top[k] = w.DissolvedMassData()[top * lanes];
      out.liquid_top_k[k] = w.Phase(top).temperature;
    }
    out.ledger_kg = w.Books().dissolved_surface;
    out.ledger_kj = w.EnergyBooks().dissolved_surface;
    for (int i = 0; i < ext::kEffervescenceCensusFields; ++i) {
      out.census[i] = w.EffervescenceConfig().census[i];
    }
    return out;
  };

  // What the CO2 columns can hold at the surface: H p M V at the pooled CO2 pressure, which is
  // the whole column's (the cells all hold the same gas at the same temperature).
  const double capacity = static_cast<double>(henry) * full_pa * kMolar * 1.0;
  const double deficit_want = (capacity - load) * leave;

  const Result on = run(Options{});
  const double want = static_cast<double>(load) * leave;
  printf("  mixture cap reads %.1f Pa (a CO2 cell reads %.1f Pa)\n", on.mix_pa, full_pa);
  printf("  top cells: helium %+.4f g (want %+.4f g), CO2 %+.4f g, mixture %+.4f g (want %+.4f g "
         "each); bottom cells %.4f/%.4f/%.4f g; %d surface, %d absorbed, %d fizz records\n",
         on.top[0] * 1000.0f, want * 1000.0, on.top[1] * 1000.0f, on.top[2] * 1000.0f,
         -deficit_want * 1000.0, on.bottom[0] * 1000.0f, on.bottom[1] * 1000.0f,
         on.bottom[2] * 1000.0f, on.surface_records, on.absorbed_records, on.fizz_records);
  Check(std::fabs(on.mix_pa - full_pa) < 1e-4f * full_pa,
        "a cell whose gas is in the MIXTURE layer reads that gas's pressure, not vacuum's");
  Check(on.fizz_records == 0,
        "half-saturated liquid fizzes under none of the three -- including the promoted room, "
        "which 0.29 read as a vacuum surface");
  Check(std::fabs(on.top[0] - want) < 1e-5 * want,
        "under helium the top cell outgasses (kg - H p_CO2 M V)(1 - e^-k) with p_CO2 = 0");
  Check(std::fabs(-on.top[1] - deficit_want) < 1e-4 * deficit_want &&
            std::fabs(-on.top[2] - deficit_want) < 1e-4 * deficit_want,
        "under CO2 -- in the grid or in the mixture -- it ABSORBS (H p_CO2 M V - kg)(1 - e^-k), "
        "published as a negative record");
  Check(on.absorbed_records == 2 && on.surface_records == 1,
        "two absorbed records (reason 11) and one released (reason 10)");
  Check(std::fabs((kGasKg - on.co2_cap_kg) + on.top[1]) < 1e-6f &&
            std::fabs((kGasKg - on.mix_cap_kg) + on.top[2]) < 1e-6f,
        "the cap cell gave up exactly what the surface took -- the vanilla cell and the mixture "
        "slot alike");
  Check(std::fabs(on.lane_top[1] - (load - on.top[1])) < 1e-6f &&
            std::fabs(on.lane_top[2] - (load - on.top[2])) < 1e-6f,
        "and the lane holds exactly that much more");
  Check(on.bottom[0] == 0.0f && on.bottom[1] == 0.0f && on.bottom[2] == 0.0f,
        "and only the TOP cell exchanges: the cell below it waits for mixing to bring it up");
  Check(std::fabs(on.ledger_kg - static_cast<double>(on.top[1])) < 1e-9,
        "the mass ledger charges the vanilla cell's loss at dissolved_surface, and not the "
        "mixture's, which field 0 never counted");
  Check(on.census[ext::kEffervescenceCensusSurfaces] == 3.0 &&
            on.census[ext::kEffervescenceCensusSurfaceReleases] == 1.0 &&
            std::fabs(on.census[ext::kEffervescenceCensusSurfaceKg] - on.top[0]) < 1e-9 &&
            on.census[ext::kEffervescenceCensusSurfaceUptakes] == 2.0 &&
            std::fabs(on.census[ext::kEffervescenceCensusSurfaceUptakeKg] +
                      (on.top[1] + on.top[2])) < 1e-6 &&
            on.census[ext::kEffervescenceCensusSurfaceUptakeHeatKJ] == 0.0,
        "the census counts three surfaces, one release, two uptakes, their kilograms, and no "
        "heat -- everything is at one temperature");

  // A THIN CAP: the column above pushes as hard as ever, but the cell touching the surface holds
  // 4 g. Half of it, and never under 2 g: the surface takes 2 g, not the 6.7 g it wanted.
  Options thin;
  thin.co2_cap_kg = 0.004f;
  const Result t = run(thin);
  printf("  thin 4 g cap: absorbed %.4f g, cap left %.4f g\n", -t.top[1] * 1000.0f,
         t.co2_cap_kg * 1000.0f);
  Check(std::fabs(-t.top[1] - 0.002f) < 1e-7f && std::fabs(t.co2_cap_kg - 0.002f) < 1e-7f,
        "a thin cap gives up at most half of what it holds, and never drops under 2 g");

  // A HOT CAP: the absorbed gas carries m c (T_cap - T_liquid) into the liquid, and both ledgers
  // see the grid's own change.
  Options hot;
  hot.co2_cap_k = 350.0f;
  const Result h = run(hot);
  const double hot_kg = -static_cast<double>(h.top[1]);
  const double heat = hot_kg * 0.846 * (350.0 - kT);
  const double rise = heat / (static_cast<double>(kLiqKg) * 2.0);
  const double grid_kj = (static_cast<double>(h.liquid_top_k[1]) - kT) * kLiqKg * 2.0 -
                         hot_kg * 0.846 * 350.0;
  printf("  hot 350 K cap: absorbed %.4f g, liquid %+.6f K (want %+.6f), heat %.5f kJ, energy "
         "ledger %.5f kJ (grid change %.5f)\n", hot_kg * 1000.0,
         h.liquid_top_k[1] - kT, rise, h.census[ext::kEffervescenceCensusSurfaceUptakeHeatKJ],
         h.ledger_kj, grid_kj);
  Check(hot_kg > 0.0 && std::fabs(h.census[ext::kEffervescenceCensusSurfaceUptakeHeatKJ] - heat) <
                            1e-9 + 1e-5 * heat,
        "gas from a hotter cap pays m c_gas (T_cap - T_liquid) into the liquid");
  Check(std::fabs((h.liquid_top_k[1] - kT) - rise) < 2e-5,
        "and the liquid warms by exactly that over its own heat capacity (to a float step)");
  Check(std::fabs(h.ledger_kj - grid_kj) < 1e-6,
        "and the energy ledger's dissolved_surface is the grid's own change: the gas cell's "
        "m c T leaving, the heat landing");

  // NOTHING DISSOLVED ANYWHERE: absorption is how plain liquid starts holding gas, so the pass
  // cannot wait for a lane to be non-zero first.
  Options plain;
  plain.load_scale = 0.0f;
  const Result pl = run(plain);
  const double plain_want = capacity * leave;
  printf("  plain liquid: helium %+.4f g, CO2 %+.4f g, mixture %+.4f g (want %+.4f g)\n",
         pl.top[0] * 1000.0f, pl.top[1] * 1000.0f, pl.top[2] * 1000.0f, -plain_want * 1000.0);
  Check(pl.top[0] == 0.0f && std::fabs(-pl.top[1] - plain_want) < 1e-4 * plain_want &&
            std::fabs(-pl.top[2] - plain_want) < 1e-4 * plain_want,
        "liquid holding nothing absorbs from the CO2 above it, and nothing from the helium");

  Options short_form;
  short_form.long_form = false;
  const Result off = run(short_form);
  Check(off.surface_records == 0 && off.absorbed_records == 0 && off.top[0] == 0.0f &&
            off.top[1] == 0.0f && off.census[ext::kEffervescenceCensusSurfaces] == 0.0,
        "and the short form of the message leaves the exchange off, both ways");
}

int main(int argc, char** argv) {
  setvbuf(stdout, nullptr, _IONBF, 0);
  bool perf = false;
  const char* goldens_path = "GOLDENS.txt";
  bool record_goldens = false;
  const char* noncanonical = nullptr;
  char noncanonical_buf[256];
  for (int i = 1; i < argc; ++i) {
    // Whitelist, as in the other two suites. `--perf` runs a different body entirely and
    // executes no Check() at all, so it must not be allowed to assert -- or to re-record.
    if (!noncanonical &&
        strcmp(argv[i], "--goldens") != 0 &&
        strcmp(argv[i], "--record-goldens") != 0) {
      if (i + 1 < argc && argv[i + 1][0] != '-') {
        snprintf(noncanonical_buf, sizeof(noncanonical_buf), "%s %s", argv[i], argv[i + 1]);
      } else {
        snprintf(noncanonical_buf, sizeof(noncanonical_buf), "%s", argv[i]);
      }
      noncanonical = noncanonical_buf;
    }
    if (!strcmp(argv[i], "--goldens") && i + 1 < argc) goldens_path = argv[++i];
    if (!strcmp(argv[i], "--record-goldens")) record_goldens = true;
    if (!strcmp(argv[i], "--perf")) perf = true;
  }
  if (perf) {
    RunPerfBenchmark(200, 200, 300);  // 40,000 cells
    RunRoomPerfBenchmark(200, 200, 24, 300);  // walled multi-room variant
    RunMultiRateBenchmark(200, 200, 24, 300);  // mixing_every_n_ticks cadence
  } else {
    TestExtMessageTable();
    TestExtApiHeader();
    TestRaycast();
    TestSinglePairEqualizes();
    TestTwoSpeciesIndependent();
    TestMixPairCarriesHeat();
    TestLiquidMeetsMixture();
    TestTemperatureAffectsPressure();
    TestProjectionMultiPhaseRules();
    TestCellCondensationRule();
    TestCellBoilRule();
    TestFieldSolver();
    TestLatentEnergyOfTransition();
    TestEffervescenceCap();
  TestEffervescenceThinPocket();
  TestEffervescenceSurface();
  }
  printf("\n%s\n", g_fail_count == 0 ? "done: all PASS" : "done: FAILURES PRESENT");
  const bool goldens_ok = goldens::CheckCount(goldens_path, "gastest.checks", g_check_count,
                                              record_goldens, noncanonical);
  return (g_fail_count == 0 && goldens_ok) ? 0 : 1;
}
