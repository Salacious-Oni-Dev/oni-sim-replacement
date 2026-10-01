// The gas-mixture math over the per-cell SoA storage in `world.h` (`gas_species_`/`gas_mass_`/
// occupancy/dirty). Header-only, same convention as the rest of `sim/`; tested standalone by
// `driver/src/gastest.cpp`.
//
// Scope: atmosphere-only. Nothing here assumes solid/liquid state; `MixPair` is meant for two
// open cells, not a pipe segment or a chunk.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "../abi/gas_mixture_abi.h"
#include "tunables.h"
#include "world.h"

namespace oni_sim::gas {

// Linear scan of a cell's <=8 slots for `species`. -1 if absent. Cheap enough at N=8 that a
// hash/sorted structure would cost more than it saves — same reasoning `world.h`'s comments
// give for keeping `kMaxSpeciesPerCell` fixed and small.
inline int FindSlot(const World& w, size_t cell, uint16_t species) {
  for (int s = 0; s < kMaxSpeciesPerCell; ++s) {
    if (w.GasSpecies(cell, s) == species) return s;
  }
  return -1;
}

// Finds `species`'s existing slot, or claims the first empty one and marks it occupied.
// Returns -1 only if the cell already holds `kMaxSpeciesPerCell` *other* distinct species —
// the cost of a fixed slot count; callers decide what to do with a full cell.
inline int FindOrClaimSlot(World& w, size_t cell, uint16_t species) {
  const int existing = FindSlot(w, cell, species);
  if (existing >= 0) return existing;
  for (int s = 0; s < kMaxSpeciesPerCell; ++s) {
    if (w.GasSpecies(cell, s) == kEmptySpecies) {
      w.MutableGasSpecies(cell, s) = species;
      w.MutableGasOccupiedMask(cell) =
          static_cast<uint8_t>(w.GasOccupiedMask(cell) | (1u << s));
      return s;
    }
  }
  return -1;
}

// Test/tooling helper — adds `mass` kg of `species` to `cell`, claiming a slot if needed.
// No message wraps this yet; direct-call only, same as how gastest.cpp drives it.
inline void InjectSpecies(World& w, size_t cell, uint16_t species, float mass) {
  const int slot = FindOrClaimSlot(w, cell, species);
  if (slot < 0) return;  // cell's 8 slots are full of other species; dropped, not asserted
  w.MutableGasMass(cell, slot) += mass;
  w.MutableGasDirty(cell) = 1;
}

// Symmetric counterpart to InjectSpecies, used by extractor-style buildings (a vanilla-mass
// release point --
// see abi/sim_abi_ext.h's kConvertToVanillaMass). Removes up to `mass` kg of `species` from
// `cell`, clamped to whatever's actually in that slot -- returns the amount actually removed,
// which a caller needing a conserving transfer must use instead of the amount it asked for.
// Frees the slot (species id reset to kEmptySpecies, occupied bit cleared) once its mass
// reaches zero, so FindOrClaimSlot can reuse it later, symmetric with InjectSpecies claiming
// one on the way in.
inline float RemoveSpecies(World& w, size_t cell, uint16_t species, float mass) {
  const int slot = FindSlot(w, cell, species);
  if (slot < 0 || !(mass > 0.0f)) return 0.0f;
  const float have = w.GasMass(cell, slot);
  const float removed = (mass < have) ? mass : have;
  const float remaining = have - removed;
  w.MutableGasMass(cell, slot) = remaining;
  if (remaining <= 0.0f) {
    w.MutableGasMass(cell, slot) = 0.0f;
    w.MutableGasSpecies(cell, slot) = kEmptySpecies;
    w.MutableGasOccupiedMask(cell) =
        static_cast<uint8_t>(w.GasOccupiedMask(cell) & ~(1u << slot));
  }
  w.MutableGasDirty(cell) = 1;
  return removed;
}

// `ElementTable::At(...).molarMass` is g/mol (Klei's own raw content-data units, mirrored
// verbatim off the real Element table -- confirmed against the managed
// ElementLoader reading the SAME asset, e.g. CarbonDioxide=44.01). Every mass this codebase
// carries (World::GasMass, a tank/pipe's own storage) is kg. `massKg / molarMass_gPerMol`
// is therefore dimensionally wrong by exactly 1000x (kg/(g/mol) = 1000*mol, not mol) --
// `kGramsPerKilogram` makes the required g<->kg conversion explicit at every mole
// computation in this file instead of leaving it an invisible unit mismatch.
constexpr float kGramsPerKilogram = 1000.0f;

// WHICH molar mass to divide by, and why this file is not consistent about it on purpose.
//
// Klei's `Element::molarMass` is the ATOMIC mass for the diatomic gases, not the molecular
// mass of the molecule the element models -- Oxygen 15.9994 instead of 31.9988, Hydrogen
// 1.00794 instead of 2.01588 (measured against the shipped elements/gas.yaml; the
// full table and the reasoning live on ElementTable::MolecularMassOf in sim/world.h). Every
// mole COUNT, and so every pressure this project reported for a diatomic, was exactly 2x
// wrong because of it.
//
// So the two functions that count moles -- CellMoles and MolesFromSpeciesList, the ones that
// feed CellPressure and the SIM_ComputeGasPressure export -- use the corrected
// `table.MolecularMassOf(idx)`.
//
// The two functions that MIX -- MixPair and EqualizeSingleSpecies -- deliberately keep
// reading the raw `.molarMass` field. This is not an oversight. In both of them the molar
// mass cancels out of the answer analytically: pressures go as `1/M`, so the pressure delta
// goes as `1/M`, so the mole delta goes as `1/M`, and the mass actually transferred is
// `dn * M`, which is independent of M. Substituting a different M there cannot change the
// physics, only the float rounding of an intermediate -- which would break byte-identity with
// the reference results for zero physical gain. That class of error moves READINGS, never
// transferred mass.

inline float CellMoles(const World& w, const ElementTable& table, size_t cell) {
  float moles = 0.0f;
  const uint8_t mask = w.GasOccupiedMask(cell);
  for (int s = 0; s < kMaxSpeciesPerCell; ++s) {
    if (!(mask & (1u << s))) continue;
    const float molar = table.MolecularMassOf(w.GasSpecies(cell, s));
    if (molar > 0.0f) moles += w.GasMass(cell, s) * kGramsPerKilogram / molar;
  }
  return moles;
}

// PV = nRT -> P = nRT / V. `CellVolumeM3` is a placeholder constant, not a real per-cell
// volume field — the rest of the sim tracks mass per tile, not volume, so there is nothing
// to read yet. Revisit once a real volume source exists (e.g. tile geometry); until then every
// cell is treated as the same fixed volume, which is enough to test conservation and
// equalization behavior, just not absolute pressure values. The tunable `CellVolumeM3`
// (tunables.def); the gas constant is `GasConstantR`, whose default is the ABI's
// `kGasConstantR` (abi/gas_mixture_abi.h).
static_assert(kTunableStock.gas_constant_r == kGasConstantR,
              "tunables.def: GasConstantR must default to the ABI's kGasConstantR");

// Generalized PV=nRT/V, decoupled from any cell -- the same formula CellPressure uses below,
// factored out so a caller with its own container (a tank, a pipe) reuses this exact physics
// instead of re-deriving it in managed code. State (a tank's mass, a pipe's contents) stays
// managed, matching vanilla's own design (conduits.h's header
// comment: "the sim never sees [pipe contents] move"; a real Gas Reservoir's own Storage is
// plain managed code too) -- what belongs natively is the physics formula itself, not a
// parallel managed reimplementation of it.
// `gasConstantR` defaults to the live table; a kernel calling this per cell passes its own
// hoisted copy (tunables.h, THE ONE RULE).
inline float PressureFromMoles(float moles, float temperatureK, float volumeM3,
                               float gasConstantR = g_tunables.gas_constant_r) {
  if (volumeM3 <= 0.0f) return 0.0f;
  return moles * gasConstantR * temperatureK / volumeM3;
}

// Dalton's-law mole sum over an arbitrary species/mass list -- the same per-species loop
// CellMoles runs above, generalized off `World`/a cell index so a caller's own container can
// reuse it. `species`/`massKg` are parallel arrays, `count` entries each.
inline float MolesFromSpeciesList(const ElementTable& table, const uint16_t* species,
                                   const float* massKg, int32_t count) {
  float moles = 0.0f;
  for (int32_t i = 0; i < count; ++i) {
    const float molar = table.MolecularMassOf(species[i]);
    if (molar > 0.0f) moles += massKg[i] * kGramsPerKilogram / molar;
  }
  return moles;
}

inline float CellPressure(const World& w, const ElementTable& table, size_t cell,
                          const Tunables& tun = g_tunables) {
  const float n = CellMoles(w, table, cell);
  const float t = w.Phase(cell).temperature;
  return PressureFromMoles(n, t, tun.cell_volume_m3, tun.gas_constant_r);
}

// Generalizes MixPair's own per-species equalization step (below) to two independent
// containers with their own volumes and temperatures, instead of two cells that always share
// CellVolumeM3. Added: a tank plumbed to a pipe with no valve between them is
// exactly this -- one connected system that should equalize toward a shared pressure, not an
// active pump moving mass at a fixed rate. As in Stationeers, directionality belongs in explicit
// valve BUILDINGS, never in a storage tank -- this is the physics a valve-free connection has.
//
// massA/tempA/volumeA and massB/tempB/volumeB describe each side's current holding of ONE
// shared species (molarMass); `rate` damps the step the same way MixPair's own rate does --
// MixPair's own dmass carries a fixed extra *0.5 on top of `rate`, so even rate=1.0 only closes
// HALF the pressure gap per call, converging geometrically over repeated calls rather than
// snapping to equilibrium in one step (verified in vftest.cpp: a single rate=1.0 call on an
// 8kg/0kg symmetric pair leaves 6kg/2kg, not 4kg/4kg -- intentional, same "run and iterate"
// philosophy MixPair's own doc comment already states, not a bug in this generalization).
// Derivation: moving `dn` moles
// from the higher-pressure side to the lower changes each side's own pressure by
// `dn*R*T/V` (its own T/V, not a shared one) in opposite directions, so closing the gap `dp`
// exactly takes `dn_full = dp / (R * (tempA/volumeA + tempB/volumeB))` -- the symmetric-volume
// case (V_a=V_b=V) collapses to MixPair's own `dp*V/(R*avg_t)` up to the same 0.5 damping
// factor MixPair applies separately, confirming this is the same physics, not a new formula.
//
// Returns the mass that should move INTO side A (from B) this step -- positive means A gains,
// negative means A loses to B, magnitude already clamped to whichever side is actually the
// source and floored at the same `MinTransferMass` MixPair uses (a transfer this small isn't
// worth a caller round-tripping a write for). Pure function: no World, no per-cell state, not
// even the element table (molarMass is a direct argument) -- a caller with its own container
// (this native gas-mixture layer never sees) supplies everything.
inline float EqualizeSingleSpecies(float molarMass, float massA, float tempA, float volumeA,
                                    float massB, float tempB, float volumeB, float rate) {
  if (molarMass <= 0.0f || volumeA <= 0.0f || volumeB <= 0.0f) return 0.0f;
  // An export's helper, one call per query: it reads the live table directly.
  const float gas_constant = g_tunables.gas_constant_r;
  const float min_transfer_mass = g_tunables.min_transfer_mass;
  const float pa =
      PressureFromMoles(massA * kGramsPerKilogram / molarMass, tempA, volumeA, gas_constant);
  const float pb =
      PressureFromMoles(massB * kGramsPerKilogram / molarMass, tempB, volumeB, gas_constant);
  const float dp = pa - pb;
  if (dp == 0.0f) return 0.0f;

  const float denom = gas_constant * (tempA / volumeA + tempB / volumeB);
  if (denom <= 0.0f) return 0.0f;
  // dn_full is in real moles here (dp is now a real Pa value); molarMass is g/mol, so
  // dn_full*molarMass is GRAMS -- divide back by kGramsPerKilogram to get the kg this
  // function returns.
  const float dn_full = std::fabs(dp) / denom;
  float dmass = dn_full * molarMass / kGramsPerKilogram * rate * g_tunables.gas_equalize_damping;

  if (dp > 0.0f) {
    // A is the higher-pressure side -- mass flows OUT of A, into B.
    dmass = std::min(dmass, massA);
    if (dmass <= min_transfer_mass) return 0.0f;
    return -dmass;
  }
  // B is the higher-pressure side -- mass flows INTO A.
  dmass = std::min(dmass, massB);
  if (dmass <= min_transfer_mass) return 0.0f;
  return dmass;
}

// Adiabatic tank-fill temperature: the compression heat of pumping gas into a store.
//
// `CalculateCombinedTemperature` (sim/emitters.h) is a plain mass-weighted calorimetry mix,
// correct for two masses merging at the SAME pressure/volume (two piles of material touching,
// a dispenser emptying into open space) but WRONG for gas being pumped from a source into an
// existing store against resistance: the incoming gas does real compression work, so it must
// arrive carrying its full enthalpy (Cp*T), not just its bare internal energy (Cv*T). The real
// open-system energy balance for filling a rigid container (a standard ideal-gas "tank filling"
// result) is:
//   massA*tempA + massB*(tempB * gammaMix) = (massA+massB) * tempFinal
// i.e. the exact same mass-weighted mix `CalculateCombinedTemperature` already computes, with
// the INCOMING side's temperature pre-scaled by gammaMix = Cp/Cv before averaging -- reduces to
// plain calorimetry exactly when gammaMix == 1.0 (an open valve / free mixing with no
// compression work, EqualizeSingleSpecies's own use case above).
//
// The game's Element table has no gamma (it never models gas compression; its specific heat
// makes no Cv/Cp distinction), and Stationeers' pumps do not model compression heating either,
// so this is a standard textbook energy balance rather than a port.
//
// gammaMix is a mixture-wide constant supplied by the caller, not looked up per species here --
// per-species gamma is Mod 2's territory (material properties), same placeholder pattern as
// LiquidCompressor's PlaceholderLiquidDensityKgM3. Reimplements CalculateCombinedTemperature's
// own clamped-mix shape directly (rather than including emitters.h, which pulls in
// buildings/disease/physics -- this header stays standalone-testable by gastest.exe, same
// convention as the rest of this file) -- structurally identical by construction, verified in
// vftest.cpp against the gammaMix==1.0 case matching plain mass-weighted mixing exactly.
inline float AdiabaticFillTemperature(float gammaMix, float massA, float tempA, float massB,
                                       float tempB) {
  const float scaled_temp_b = tempB * gammaMix;
  const float total = massA + massB;
  if (total <= 0.0f) return 0.0f;
  const float mix = (massB * scaled_temp_b + massA * tempA) / total;
  const float lo = tempA < scaled_temp_b ? tempA : scaled_temp_b;
  const float hi = tempA < scaled_temp_b ? scaled_temp_b : tempA;
  return mix > hi ? hi : (mix < lo ? lo : mix);
}

// One explicit-Euler mixing step between two adjacent cells, atmosphere-only, first pass —
// not tuned, not the final algorithm, just enough to prove the SoA storage can carry and
// conserve a real multi-species exchange. For every species present in either cell, moves
// mass from the higher-partial-pressure side toward the lower, proportional to that
// species' own pressure delta (Dalton's law: each species' partial pressure equalizes
// independently of the others sharing the cell).
//
// `rate` in (0, 1]: fraction of the one-step equalizing transfer to actually apply. 1.0
// would fully equalize a species pair in isolation; run below 1.0 and iterate when multiple
// species/multiple neighbor pairs interact in the same tick -- a damped rate rather than an
// exact solve every tick, as Stationeers' gas mixing does.
inline void MixPair(World& w, const ElementTable& table, size_t a, size_t b, float rate,
                    const Tunables& tun = g_tunables) {
  // Union of species present in either cell, collected once so a species crossing over to
  // a previously-absent side mid-loop can't perturb the walk.
  uint16_t species_union[kMaxSpeciesPerCell * 2];
  int union_n = 0;
  auto collect = [&](size_t cell) {
    const uint8_t mask = w.GasOccupiedMask(cell);
    for (int s = 0; s < kMaxSpeciesPerCell; ++s) {
      if (!(mask & (1u << s))) continue;
      const uint16_t sp = w.GasSpecies(cell, s);
      bool seen = false;
      for (int i = 0; i < union_n; ++i) {
        if (species_union[i] == sp) { seen = true; break; }
      }
      if (!seen) species_union[union_n++] = sp;
    }
  };
  collect(a);
  collect(b);

  const float ta = w.Phase(a).temperature;
  const float tb = w.Phase(b).temperature;
  const float avg_t = (ta + tb) * 0.5f;
  if (avg_t <= 0.0f) return;

  // Heat moves with the mass. A cell holds one shared temperature (`PhaseEntry.temperature`)
  // over its vanilla mass and every slot, so a species leaving a cell takes `dmass * c * T` of
  // that cell's heat with it and the source's temperature does not change; the receiving
  // cell's temperature is its new heat over its new heat capacity. Stationeers does the same:
  // gas always carries its energy with it. Moving mass alone would create heat from nothing
  // (`vftest`'s blocked-gas arm checks it).
  //
  // Double precision, and only a side that RECEIVED mass is rewritten: recomputing a side that
  // only gave mass up would round its temperature for no physical reason.
  auto heat_capacity = [&](size_t cell) {
    const PhaseEntry& c = w.Phase(cell);
    double hc = c.mass > 0.0f
                    ? static_cast<double>(c.mass) * table.At(c.element).specificHeatCapacity
                    : 0.0;
    const uint8_t mask = w.GasOccupiedMask(cell);
    for (int s = 0; s < kMaxSpeciesPerCell; ++s) {
      if (!(mask & (1u << s))) continue;
      hc += static_cast<double>(w.GasMass(cell, s)) *
            table.At(w.GasSpecies(cell, s)).specificHeatCapacity;
    }
    return hc;
  };
  double hc_a = heat_capacity(a);
  double hc_b = heat_capacity(b);
  double energy_a = hc_a * ta;
  double energy_b = hc_b * tb;
  bool a_received = false;
  bool b_received = false;

  for (int i = 0; i < union_n; ++i) {
    const uint16_t sp = species_union[i];
    const float molar = table.At(sp).molarMass;
    if (molar <= 0.0f) continue;

    const int sa = FindSlot(w, a, sp);
    const int sb = FindSlot(w, b, sp);
    const float mass_a = sa >= 0 ? w.GasMass(a, sa) : 0.0f;
    const float mass_b = sb >= 0 ? w.GasMass(b, sb) : 0.0f;
    const float pa =
        (mass_a * kGramsPerKilogram / molar) * tun.gas_constant_r * ta / tun.cell_volume_m3;
    const float pb =
        (mass_b * kGramsPerKilogram / molar) * tun.gas_constant_r * tb / tun.cell_volume_m3;
    const float dp = pa - pb;
    if (dp == 0.0f) continue;

    const bool a_is_source = dp > 0.0f;
    const size_t src = a_is_source ? a : b;
    const size_t dst = a_is_source ? b : a;
    const int src_slot = a_is_source ? sa : sb;
    const float src_mass = a_is_source ? mass_a : mass_b;
    if (src_slot < 0 || src_mass <= 0.0f) continue;

    // Convert the pressure delta back to a mole delta via the ideal-gas law, then to a
    // mass delta via this species' own molar mass (g/mol -> kg needs /kGramsPerKilogram, same
    // fix as EqualizeSingleSpecies above). Halved because the transfer moves both sides toward
    // the midpoint, not one side all the way to the other's current value.
    const float dn = std::fabs(dp) * tun.cell_volume_m3 / (tun.gas_constant_r * avg_t);
    float dmass = dn * molar / kGramsPerKilogram * rate * tun.gas_equalize_damping;
    dmass = std::min(dmass, src_mass);
    // Below this, treat the pair as equilibrated rather than paying a write (and marking
    // both cells dirty) for a transfer too small to matter. Without this cutoff, continuous
    // float math never actually reaches zero — a diffusion front asymptotically approaches
    // equal pressure but keeps moving some nonzero amount every tick forever, which means
    // `dirty` never clears and nothing can ever sleep. Found by measuring, not guessed:
    // the first version of this function had no floor and gastest.cpp's --perf benchmark
    // showed only 3.4% of a 40k-cell grid asleep after 300 ticks and a 1.07x "speedup" that
    // was really just skipping the small tail of cells the diffusion front hadn't reached
    // yet — see gastest.cpp's RunPerfBenchmark for the measurement. 1e-6 kg is a first
    // guess, not tuned against a real gameplay mass scale. The tunable `MinTransferMass`.
    if (dmass <= tun.min_transfer_mass) continue;
    // Never INTO a cell holding liquid (or anything condensed). The mixture shares
    // `PhaseEntry.temperature`, so a pond cell given slots would share its temperature with
    // the gas, and every liquid kernel refuses a cell carrying slots (`IsMixtureOwnedCell`):
    // without this line a promoted room's pond would stop flowing cell by cell as the mixture
    // reached it. Giving is still allowed, which drains any slots such a
    // cell already holds.
    if (table.Phase(w.Phase(dst).element) >= kStateLiquid) continue;

    const int dst_slot = FindOrClaimSlot(w, dst, sp);
    if (dst_slot < 0) continue;  // destination's 8 slots are full of other species

    w.MutableGasMass(src, src_slot) -= dmass;
    w.MutableGasMass(dst, dst_slot) += dmass;
    const double moved_hc = static_cast<double>(dmass) * table.At(sp).specificHeatCapacity;
    const double moved_energy = moved_hc * (a_is_source ? ta : tb);
    if (a_is_source) {
      hc_a -= moved_hc;
      energy_a -= moved_energy;
      hc_b += moved_hc;
      energy_b += moved_energy;
      b_received = true;
    } else {
      hc_b -= moved_hc;
      energy_b -= moved_energy;
      hc_a += moved_hc;
      energy_a += moved_energy;
      a_received = true;
    }
    w.MutableGasDirty(src) = 1;
    w.MutableGasDirty(dst) = 1;
    // A real transfer means this pair is not equilibrated — clear sleeping on both ends so
    // a stimulus into a sleeping cell wakes it immediately, same tick. Going TO sleep is
    // caller policy (see MixGridDirtySleep's doc comment and gastest.cpp's stable-tick
    // counter) — this function only ever wakes, never puts a cell down.
    w.MutableGasSleeping(src) = 0;
    w.MutableGasSleeping(dst) = 0;
  }

  // The mixture sits outside the vanilla energy ledger, but a cell's vanilla mass shares the
  // temperature written here, so whatever that write does to the grid's own energy
  // (`GridCellEnergy`) is heat crossing from the mixture into the grid and is booked as such
  // -- the same boundary `PromoteRoomAndAbsorbGas` books in the other direction. It is zero
  // for a promoted cell, which holds no vanilla mass.
  auto settle = [&](size_t cell, double hc, double energy) {
    if (!(hc > 0.0)) return;
    const double grid_before = GridCellEnergy(w, table, cell);
    w.Phase(cell).temperature = static_cast<float>(energy / hc);
    w.NoteEmittedEnergy(GridCellEnergy(w, table, cell) - grid_before);
  };
  if (a_received) settle(a, hc_a, energy_a);
  if (b_received) settle(b, hc_b, energy_b);
}

// One mixing pass over every horizontal+vertical neighbor pair in a `width` x `height` game
// grid, unconditionally. The naive baseline the sleep/dirty pass below is measured against —
// see gastest.cpp's perf benchmark.
inline void MixGridNaive(World& w, const ElementTable& table, int32_t width, int32_t height,
                          float rate) {
  const Tunables tun = g_tunables;  // tunables.h, THE ONE RULE
  for (int32_t y = 0; y < height; ++y) {
    for (int32_t x = 0; x < width; ++x) {
      const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(width) + x;
      const size_t p = w.Padded(idx);
      if (x + 1 < width) MixPair(w, table, p, w.Padded(idx + 1), rate, tun);
      if (y + 1 < height) {
        MixPair(w, table, p, w.Padded(idx + static_cast<size_t>(width)), rate, tun);
      }
    }
  }
}

// Same sweep, but skips a pair outright when BOTH cells are already asleep — a temporal-
// coherence/sleep skip. `MixPair` clears
// `sleeping` automatically the instant a real transfer touches a cell, so a sleeping cell
// wakes the same tick a neighbor disturbs it; this function never decides to put a cell TO
// sleep, only whether to skip one that already is — the threshold/hysteresis policy for
// falling asleep is the caller's (see gastest.cpp's stable-tick counter). Kept as a
// standalone function, not folded into `MixGridNaive` with a flag, so the naive path stays
// the actual unconditional baseline with nothing to accidentally short-circuit.
inline void MixGridDirtySleep(World& w, const ElementTable& table, int32_t width,
                               int32_t height, float rate) {
  const Tunables tun = g_tunables;  // tunables.h, THE ONE RULE
  for (int32_t y = 0; y < height; ++y) {
    for (int32_t x = 0; x < width; ++x) {
      const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(width) + x;
      const size_t p = w.Padded(idx);
      const bool p_asleep = w.GasSleeping(p) != 0;
      if (x + 1 < width) {
        const size_t pr = w.Padded(idx + 1);
        if (!(p_asleep && w.GasSleeping(pr))) MixPair(w, table, p, pr, rate, tun);
      }
      if (y + 1 < height) {
        const size_t pd = w.Padded(idx + static_cast<size_t>(width));
        if (!(p_asleep && w.GasSleeping(pd))) MixPair(w, table, p, pd, rate, tun);
      }
    }
  }
}

}  // namespace oni_sim::gas
