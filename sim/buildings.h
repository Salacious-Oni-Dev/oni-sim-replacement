// Buildings, as far as the sim is concerned.
//
// A building is not a cell. It is a rectangle of cells plus a lump of heat that lives
// outside the grid: one temperature, one heat capacity, one conductivity. Every substep it
// trades heat with each cell it covers, adds whatever its machinery is producing, and hands
// the game back a single number. That is the whole of `AddBuildingHeatExchange`, and it is
// the busiest message the game sends after the per-frame ones — 11,671 calls in four
// minutes of play.
//
// Two components:
//
//   * `BuildingHeatExchange`           building <-> the cells under it
//   * `BuildingToBuildingHeatExchange` building <-> building in contact
//
// Both run inside `SimData::UpdateComponents`, which `SimBase::UpdateData`
// calls once per substep after the fluid and disease kernels and before `PostProcessCell`.
// The components are updated in the order they sit in `SimData`, which puts cell exchange
// before building-to-building exchange.

#pragma once

#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

#include "../abi/sim_abi.h"
#include "../abi/sim_abi_ext.h"
#include "gas_mixture.h"
#include "physics.h"
#include "world.h"

namespace oni_sim {

// The rate at which a building trades heat with anything, folded into the transfer as a
// bare multiplier, used by both components. The tunable `BuildingTransferRate`
// (tunables.def).

// `CellSOA::GetInsulationValue` is `insulation * insulation * 1/255^2`, on the
// byte the game sends for the cell. So a normal cell (255) passes heat at full rate and an
// insulated tile attenuates by the *square* of its value. Only the cell exchange applies it;
// building-to-building exchange has no cell to read it from.
//
// The cell exchange never applies the two separately: `BuildingHeatExchange::Update` inlines
// `GetInsulationValue` and folds `BuildingTransferRate` into its constant, so the literal in
// the DLL is the product, and the raw `insulation * insulation` is what gets
// multiplied by it. Splitting them back apart would change the rounding, so it is kept as
// the one constant Klei actually stores.
// (`kInsulationScale`, the 1/255^2 on its own, lives in `physics.h` — cell-to-cell conduction
// needs it unfolded, and this file has no use for it once the product exists.)
// The product is `CellTransferScale` (tunables.h): Klei's stored literal while
// `BuildingTransferRate` is at its default, recomputed once anyone moves it.

// `SimData::constructor` writes 0.001f to both scales, and `SetDebugProperties`
// overwrites them every frame with whatever the game's debug panel holds. They are not debug
// values in any useful sense: they are the constants that set the absolute rate, and the
// game sends 3,819 of those messages a session.
inline constexpr float kDefaultBuildingTemperatureScale = 0.001f;

// Every temperature the sim writes is checked against this and 0. Klei logs and refuses
// rather than clamping, which matters: a building that goes out of range keeps its old
// temperature rather than being pinned to the bound. The tunable `MaxTemperature`
// (tunables.def), a ceiling.

// A handle is `slot | (version << 24)`, and everything that indexes by handle masks first.
inline constexpr int32_t kHandleIndexMask = 0x00ffffff;

// Every clamp in this file has SSE min-then-max semantics, not a comparison — see
// `ClampSS` in world.h for why that is a different function. It matters at three of the sites
// below, all of them a 0/0 that Klei lands on `hi` and a comparison clamp would keep as a NaN:
// `ApplyBuildingEnergy` on a building whose `CellCount() * per_cell_heat_capacity` is zero
// (applied at the start of each frame), the building's own average at the
// end of the cell sweep when its extents are empty, and
// `BuildingToBuildingHeatExchange`'s `1/self_hc`, which — unlike the `other_hc`
// on the far side of the same loop — has no positive guard in front of it.
inline float Clamp(float v, float lo, float hi) { return ClampSS(v, lo, hi); }
inline float Abs(float v) { return v < 0.0f ? -v : v; }

// Klei's `BuildingHeatExchangeData`, 44 bytes, built by `InitializeFromMessageData`.
// Three of its eleven fields are derived rather than sent, and the derivation
// is the interesting part:
//
//   * `heat_capacity` is `mass * element.specificHeatCapacity`, the whole building's.
//   * `per_cell_heat_capacity` is that divided by the cell count, and it is the one the
//     per-cell exchange actually uses — each cell trades against a building of one n'th the
//     size, and the n results are averaged back at the end. Building-to-building exchange
//     uses the undivided one instead.
//   * `thermal_conductivity` is the message's value *times* the element's, so a building's
//     conductivity is a multiplier on its material rather than a replacement for it.
//
// The extents are stored **padded** — the constructor adds one to each of minX/minY/maxX/maxY
// — so they index our storage directly, and the region test compares padded to padded.
struct BuildingHeatExchangeData {
  float temperature = 0.0f;
  float overheat_temperature = 0.0f;
  float operating_kilowatts = 0.0f;
  // Framework extension, abi/sim_abi_ext.h's kSetBuildingWasteHeatKilowatts. A second,
  // independent operating rate summed with the one above. Not part of Klei's record and
  // not in any Klei message: it defaults to 0 and a world that never sends the extension
  // message behaves byte-identically. See the ABI header for why the power -> heat rule
  // gets its own field instead of overloading `operating_kilowatts`.
  float waste_heat_kilowatts = 0.0f;
  // Framework extension, abi/sim_abi_ext.h's kSetBuildingExhaust. Klei's
  // `ExhaustKilowattsWhenActive`, moved into the sim as a rate: the sim does the whole of
  // `StructureTemperatureComponents.ExhaustHeat` itself, including the 1.5 kg mass factor and
  // the ceiling, and puts the part that cannot be delivered into this building's own body
  // instead of destroying it. 0 until the extension message is sent.
  float exhaust_kilowatts = 0.0f;
  // The ceiling Klei passes to ExhaustHeat. Normally `Overheatable.OverheatTemperature`,
  // which defaults to 348.15 K -- not 10000 K, which is why an ordinary machine in a hot room
  // delivers nothing in vanilla.
  float exhaust_max_temperature = 0.0f;
  // Framework extension, abi/sim_abi_ext.h's kSetBuildingRadiation. Emissivity, 0..1, and 0
  // by default so radiation is inert until something asks for it.
  float radiation_factor = 0.0f;
  // Radiating area in square metres; <= 0 means the building's own CellCount().
  float radiation_area_m2 = 0.0f;
  // Framework extension, abi/sim_abi_ext.h's kSetBuildingConvection. Stationeers'
  // ConvectionFactor, 0 by default so the term is inert until something asks for it.
  float convection_factor = 0.0f;
  // Convecting area in square metres; <= 0 means the building's own CellCount().
  float convection_area_m2 = 0.0f;
  // How far the convection term reaches, in cells. 0 is the building's own footprint, which is
  // what an ordinary building gets; a radiator asks for more so that it exchanges with the ROOM
  // rather than pinning the one cell it stands in. Clamped to kMaxBuildingConvectionReach.
  int32_t convection_reach = 0;
  float heat_capacity = 0.0f;
  float per_cell_heat_capacity = 0.0f;
  float thermal_conductivity = 0.0f;
  // The element's high transition temperature. A building
  // does not melt at its overheat temperature, it melts at its material's.
  float melt_temperature = 0.0f;
  int32_t min_x = 0, min_y = 0, max_x = 0, max_y = 0;

  int32_t CellCount() const { return (max_x - min_x) * (max_y - min_y); }
};

// A building this one is touching, and how many cells they share. `cells_in_contact` is
// stored by `BuildingToBuildingHeatExchange::Add` and then **never read** — the exchange
// rate does not depend on how much of the two buildings touch. Kept because the game sends
// it and a future build may want it.
struct InContactBuilding {
  int32_t handle = -1;
  int32_t cells_in_contact = 0;
};

struct BuildingToBuildingData {
  int32_t self = -1;
  std::vector<InContactBuilding> contacts;
};

// Klei's `CompactedVector`: a dense array plus a handle indirection, so iteration is linear
// and handles survive removals. A handle is `index | (version << 24)`; the version byte is
// bumped on release so a stale handle is detected rather than silently aliasing a new object.
//
// Removal is a swap with the last element, which is what "compacted" means and which decides
// **iteration order** — after a building is removed the last one takes its place, so update
// order is not registration order. That is observable, so it is copied rather than tidied.
template <typename T>
class CompactedVector {
 public:
  int32_t Add(const T& value) {
    int32_t handle;
    if (!free_.empty()) {
      handle = free_.back();
      free_.pop_back();
      index_[static_cast<size_t>(handle & kHandleIndexMask)] =
          static_cast<int32_t>(data_.size());
    } else {
      handle = static_cast<int32_t>(index_.size()) & kHandleIndexMask;
      index_.push_back(static_cast<int32_t>(data_.size()));
      version_.push_back(0);
    }
    data_.push_back(value);
    handles_.push_back(handle);
    return handle;
  }

  bool Valid(int32_t handle) const {
    if (handle < 0) return false;
    const size_t slot = static_cast<size_t>(handle & kHandleIndexMask);
    if (slot >= index_.size()) return false;
    if (version_[slot] != static_cast<uint8_t>(handle >> 24)) return false;
    return index_[slot] >= 0;
  }

  T* Get(int32_t handle) {
    if (!Valid(handle)) return nullptr;
    return &data_[static_cast<size_t>(index_[static_cast<size_t>(handle & kHandleIndexMask)])];
  }
  const T* Get(int32_t handle) const {
    return const_cast<CompactedVector*>(this)->Get(handle);
  }

  void Remove(int32_t handle) {
    if (!Valid(handle)) return;
    const size_t slot = static_cast<size_t>(handle & kHandleIndexMask);
    const size_t at = static_cast<size_t>(index_[slot]);
    const size_t last = data_.size() - 1;
    if (at != last) {
      data_[at] = data_[last];
      handles_[at] = handles_[last];
      index_[static_cast<size_t>(handles_[at] & kHandleIndexMask)] = static_cast<int32_t>(at);
    }
    data_.pop_back();
    handles_.pop_back();
    index_[slot] = -1;
    version_[slot] = static_cast<uint8_t>(version_[slot] + 1);
    free_.push_back(static_cast<int32_t>(slot) |
                    (static_cast<int32_t>(version_[slot]) << 24));
  }

  void Clear() {
    data_.clear();
    handles_.clear();
    index_.clear();
    version_.clear();
    free_.clear();
  }

  std::vector<T>& Data() { return data_; }
  const std::vector<T>& Data() const { return data_; }
  const std::vector<int32_t>& Handles() const { return handles_; }
  size_t SlotCount() const { return index_.size(); }

  // The rest of the table, for `sim/component_state.h`. A component's registry is live state
  // that no save blob carries and that `AllocateCells` clears, so a restore has to be able to
  // put back not just the objects but the handle table around them: iteration order is
  // observable (see the class comment above -- removal swaps with the last element), a stale
  // handle must keep being detected as stale, and the next `Add` has to reuse the same freed
  // slot the original run's next `Add` would have. Data() and Handles() alone cannot express
  // any of that, which is why these three are exposed rather than the pair being enough.
  const std::vector<int32_t>& Index() const { return index_; }
  const std::vector<uint8_t>& Version() const { return version_; }
  const std::vector<int32_t>& Free() const { return free_; }

  // Replace the whole table. Rejects anything self-inconsistent rather than half-applying it:
  // a caller handing back a truncated or reordered save would otherwise produce a registry
  // that looks valid and iterates wrongly.
  bool Restore(std::vector<T> data, std::vector<int32_t> handles, std::vector<int32_t> index,
               std::vector<uint8_t> version, std::vector<int32_t> free) {
    if (data.size() != handles.size()) return false;
    if (index.size() != version.size()) return false;
    for (size_t i = 0; i < handles.size(); ++i) {
      const size_t slot = static_cast<size_t>(handles[i] & kHandleIndexMask);
      if (slot >= index.size()) return false;
      if (index[slot] != static_cast<int32_t>(i)) return false;
      if (version[slot] != static_cast<uint8_t>(handles[i] >> 24)) return false;
    }
    for (const int32_t h : free) {
      const size_t slot = static_cast<size_t>(h & kHandleIndexMask);
      if (slot >= index.size()) return false;
      if (index[slot] != -1) return false;
    }
    data_ = std::move(data);
    handles_ = std::move(handles);
    index_ = std::move(index);
    version_ = std::move(version);
    free_ = std::move(free);
    return true;
  }

 private:
  std::vector<T> data_;
  std::vector<int32_t> handles_;   // parallel to data_
  std::vector<int32_t> index_;     // handle slot -> index into data_, -1 if free
  std::vector<uint8_t> version_;   // handle slot -> current version byte
  std::vector<int32_t> free_;      // released handles, already version-bumped, LIFO
};

// Everything the two components hand back to the game in one frame.
//
// The three int lists are `MeltedInfo`, which is a bare handle. `temperatures` is indexed by
// handle slot rather than appended to, because Klei resizes it to the slot count at the top
// of every update and writes each building into its own slot — a building that was skipped
// this substep keeps the value it had, which is what makes a skipped building read as
// "unchanged" rather than as absent.
struct BuildingEvents {
  std::vector<BuildingTemperatureInfo> temperatures;
  std::vector<int32_t> overheated;
  std::vector<int32_t> no_longer_overheated;
  std::vector<int32_t> melted;

  void ClearFrame() {
    overheated.clear();
    no_longer_overheated.clear();
    melted.clear();
  }
};

// The whole of a building's state outside the grid.
struct BuildingState {
  CompactedVector<BuildingHeatExchangeData> exchange;
  CompactedVector<BuildingToBuildingData> contact;
  BuildingEvents events;
  // `SetDebugProperties`, which despite the name arrives every single frame.
  float temperature_scale = kDefaultBuildingTemperatureScale;
  float to_building_temperature_scale = kDefaultBuildingTemperatureScale;
  // One byte per handle slot: has this building's framework waste heat been charged yet
  // during the current substep? Cleared at region 0 of every substep and set by the first
  // region that actually reaches the building. See StepBuildingHeatExchange's note on why
  // the framework's rate is once-per-substep while Klei's own operating_kilowatts stays
  // once-per-region.
  // One byte per handle slot, a bitfield: which of the framework's once-per-substep charges
  // has this building already taken during the current substep? Cleared at region 0 of every
  // substep and set by the first region that actually reaches the building. See
  // StepBuildingHeatExchange's note on why the framework's rates are once-per-substep while
  // Klei's own operating_kilowatts stays once-per-region.
  //   bit 0 — waste heat (kSetBuildingWasteHeatKilowatts)
  //   bit 1 — exhaust + radiation + convection (kSetBuildingExhaust, kSetBuildingRadiation,
  //           kSetBuildingConvection)
  std::vector<uint8_t> waste_heat_charged;

  void Clear() {
    exchange.Clear();
    contact.Clear();
    events = BuildingEvents{};
  }
};

// --------------------------------------------------------------------- registration

// `BuildingHeatExchange::Register` / `InitializeFromMessageData`. The message is the same 44
// bytes for Add and Modify; only the first field differs in meaning (a callback index on the
// way in, the handle to modify on the way back).
inline BuildingHeatExchangeData BuildingDataFromMessage(
    const ElementTable& table, uint16_t elem_idx, float mass, float temperature,
    float thermal_conductivity, float overheat_temperature, float operating_kilowatts,
    int32_t min_x, int32_t min_y, int32_t max_x, int32_t max_y) {
  const Element& e = table.At(elem_idx);
  BuildingHeatExchangeData d;
  d.temperature = temperature;
  d.overheat_temperature = overheat_temperature;
  d.operating_kilowatts = operating_kilowatts;
  d.heat_capacity = e.specificHeatCapacity * mass;
  const int32_t cells = (max_x - min_x) * (max_y - min_y);
  d.per_cell_heat_capacity =
      d.heat_capacity / static_cast<float>(cells > 0 ? cells : 1);
  d.thermal_conductivity = e.thermalConductivity * thermal_conductivity;
  d.melt_temperature = e.highTemp;
  // Padded, so the cell loop indexes storage directly.
  d.min_x = min_x + 1;
  d.min_y = min_y + 1;
  d.max_x = max_x + 1;
  d.max_y = max_y + 1;
  return d;
}

// The thermal energy one building record is holding, in joules. Not part of Klei's ABI --
// it is the energy ledger's unit of account for the four sites that add, replace, adjust or
// drop a building record (World::EnergyLedger). A building carries its own energy in and
// out of the sim exactly the way emitted and consumed mass does, and until this existed
// that showed up as a first-tick jump the size of the whole building.
inline double BuildingStoredEnergy(const BuildingHeatExchangeData& d) {
  return static_cast<double>(d.per_cell_heat_capacity) *
         static_cast<double>(d.CellCount()) * static_cast<double>(d.temperature);
}

// `BuildingHeatExchange::Modify`. The replacement is wholesale — the message
// carries a fresh temperature too, so the game can teleport a building's heat — and the only
// thing carried over from the old record is one event: a building that *was* over its
// overheat temperature and is not any more says so.
inline void ModifyBuilding(BuildingState* state, int32_t handle,
                           const BuildingHeatExchangeData& next) {
  BuildingHeatExchangeData* d = state->exchange.Get(handle);
  if (!d) return;
  if (d->overheat_temperature <= d->temperature &&
      next.temperature < d->overheat_temperature) {
    state->events.no_longer_overheated.push_back(handle);
  }
  // The framework's waste-heat rate is carried over, the same way the event above is.
  // Klei's Modify is wholesale and the game sends one whenever a building's payload is
  // dirty -- every operational change, every temperature edit -- so a plain replacement
  // would zero the field at unpredictable moments and the power -> heat rule would come and
  // go with no symptom. Nothing in any Klei message can set this field, so carrying it is
  // never wrong.
  const float carried_waste_heat = d->waste_heat_kilowatts;
  const float carried_exhaust = d->exhaust_kilowatts;
  const float carried_exhaust_max = d->exhaust_max_temperature;
  const float carried_radiation = d->radiation_factor;
  const float carried_radiation_area = d->radiation_area_m2;
  const float carried_convection = d->convection_factor;
  const float carried_convection_area = d->convection_area_m2;
  const int32_t carried_convection_reach = d->convection_reach;
  *d = next;
  d->waste_heat_kilowatts = carried_waste_heat;
  d->exhaust_kilowatts = carried_exhaust;
  d->exhaust_max_temperature = carried_exhaust_max;
  d->radiation_factor = carried_radiation;
  d->radiation_area_m2 = carried_radiation_area;
  d->convection_factor = carried_convection;
  d->convection_area_m2 = carried_convection_area;
  d->convection_reach = carried_convection_reach;
}

// `ModifyBuildingEnergy`, applied where the frame's messages are drained
// rather than inside either component's update. It is how the game injects a discrete lump
// of heat with its own bounds — a duplicant warming a bed, a dock charging a suit.
//
// The bounds widen to include the building's current temperature rather than clamping it,
// and a result outside 0..10000 K is **refused rather than clamped**: Klei logs it and
// leaves the building where it was.
inline void ApplyBuildingEnergy(BuildingHeatExchangeData* d, float delta_kj,
                                float min_temperature, float max_temperature) {
  const float lo = min_temperature < d->temperature ? min_temperature : d->temperature;
  const float hi = max_temperature > d->temperature ? max_temperature : d->temperature;
  const float max_t = g_tunables.max_temperature;  // a message handler, not a kernel
  if (lo < 0.0f || hi > max_t) return;
  const float total_hc = static_cast<float>(d->CellCount()) * d->per_cell_heat_capacity;
  const float next = Clamp(d->temperature + delta_kj / total_hc, lo, hi);
  if (next > 0.0f && next < max_t) d->temperature = next;
}

// --------------------------------------------------------------- the cell exchange

// `BuildingHeatExchange::Update`, step for step.
//
// Per cell under the building:
//
//   Q = srcHC * (T_cell - T_building) * (k_elem * k_building) * insulation
//       * BuildingTransferRate * dt * scale
//
// where `srcHC` is the heat capacity of whichever side is *hotter* — the cell's mass times
// its specific heat if the cell is warmer, the building's per-cell capacity if the building
// is. That asymmetry is not a symmetry bug on Klei's part: it makes a hot building shed heat
// into a thin gas at the gas's rate and a hot gas heat the building at the building's, which
// is why machines in vacuum barely cool.
//
// Both sides then move by Q over their own heat capacity, each clamped into the interval the
// two temperatures already spanned. If the clamps would leave them crossed over — the
// building ending up hotter than the cell it was heating — the pair is put at their
// mass-weighted equilibrium instead and Q is recomputed from where the cell actually landed.
// So a single substep can equalise a pair outright but never overshoot it.
//
// The building's own temperature moves by the *sum* of those energies over `n * per-cell
// capacity`, clamped to the min and max temperature seen anywhere in the sweep, and only
// then does `operating_kilowatts * dt` go in — outside the clamp, which is what lets a
// machine heat itself above everything around it.
// Klei's `StructureTemperatureComponents.MAX_PRESSURE`, 1.5f. The cell mass at which a
// building's exhaust is delivered in full; below it, vanilla scales delivery by
// `Mathf.Min(Grid.Mass[cell], 1.5f) / 1.5f` on the managed side and drops the remainder on the
// floor. The scaling is reproduced here; the dropping is not.
// The tunable `ExhaustMaxPressure` (tunables.def).

// Stefan-Boltzmann, in kW/(m^2 K^4) rather than W, because every other rate in this sim is a
// kilowatt and every energy a kilojoule.
//
// This is the one part of Stationeers' radiative model that could NOT be ported: its
// `AtmosphereHelper.GetRadiatedHeat` / `CalculateEntropy` evaluate the flux from
// `AtmosphericsManager.EntropyCurve`, a hand-authored Unity AnimationCurve whose keyframes
// live in a scene asset rather than in code. The closed form below is the law that curve is a
// tuned approximation of. Everything else about the model -- the one-way test, the sink lerp,
// the overshoot clamp -- is reproduced structurally from that function.
// The tunable `StefanBoltzmannKW` (tunables.def), a double.

// Stationeers' `Chemistry.ArmstrongLimit`, in pascals (`kGasConstantR` is J/(mol K) and
// `CellVolumeM3` is cubic metres, so PressureFromMoles returns pascals).
//
// 6.3 kPa is the real-world Armstrong limit -- the pressure below which water boils at body
// temperature -- and Stationeers uses it for EXACTLY the job it does here: `Device.
// OnAtmosphericTick` gates a device's waste heat on `Atmosphere.IsAboveArmstrong()`, its own
// test for "there is no medium here to dump into". Reused rather than picked.
//
// WHY NOT `ExhaustMaxPressure`. The first version of the radiation term reused Klei's 1.5 kg
// exhaust saturation as the opacity threshold, on the reasoning that it avoided inventing a
// constant. That reasoning was wrong -- the two quantities are unrelated -- and a live run
// measured the consequence: against a real 234-cycle colony, 5290 of 8717 buildings were fully
// covered but the mean exposure was still 0.204, because ordinary breathable base air weighs
// around 1 kg per cell and 1.5 kg is roughly one atmosphere of oxygen. The colony shed 37 MJ
// over 35 s of sim time -- ~1.1 MW -- out of cells that were not remotely empty. Only 72 cells
// of ~13000 held under 10 g. Pressure against the Armstrong limit puts the boundary where the
// medium actually stops being one.
// The tunable `ArmstrongLimitPa` (tunables.def), a double.

// How exposed a cell leaves a building to the environment: 1 in vacuum, 0 once there is a real
// medium, ramping linearly in PRESSURE (not mass) between them so the answer is the same
// physical statement for a light gas and a heavy one.
//
// A cell whose element has no molar mass -- a solid, a liquid -- is opaque. That is not a
// special case so much as the ordinary one: a building embedded in rock is surrounded by
// matter, and ONI's conduction already owns that exchange.
inline double CellExposure(const PhaseEntry& c, const ElementTable& table,
                           const Tunables& tun = g_tunables) {
  if (!(c.mass > 0.0f)) return 1.0;
  const float molar = table.MolecularMassOf(c.element);
  if (!(molar > 0.0f)) return 0.0;
  const float moles = c.mass * gas::kGramsPerKilogram / molar;
  const double p = static_cast<double>(
      gas::PressureFromMoles(moles, c.temperature, tun.cell_volume_m3, tun.gas_constant_r));
  if (p >= tun.armstrong_limit_pa) return 0.0;
  if (p <= 0.0) return 1.0;
  return 1.0 - p / tun.armstrong_limit_pa;
}

// NOT KLEI'S: the CONVECTION policy, and the building convection term
// (kSetBuildingConvection), following Stationeers' pipe <-> world convection: the exchange is
// scaled by max(clamp01(P / OneAtmosphere), clamp01(LiquidVolumeRatio / 0.001)), once for the
// pipe's contents and once for the world cell. The same two numbers as the framework's
// PipeHeatExchange.OneAtmospherePa and LiquidVolumeRatioForFullExchange.
// The tunables `ConvectionOneAtmospherePa` and `ConvectionLiquidVolumeRatioForFull`
// (tunables.def).

inline float ConvectionClamp01(float x) { return x > 0.0f ? (x < 1.0f ? x : 1.0f) : 0.0f; }

// One side's HeatExchangeRatio() from its gas pressure and its liquid volume ratio.
inline float HeatExchangeRatio(float pressure_pa, float liquid_volume_ratio,
                               const Tunables& tun = g_tunables) {
  const float gas = ConvectionClamp01(pressure_pa / tun.convection_one_atmosphere_pa);
  const float liquid =
      ConvectionClamp01(liquid_volume_ratio / tun.convection_liquid_volume_ratio_for_full);
  return gas > liquid ? gas : liquid;
}

// Liquid with no `sim.liquid_density` counts as water-dense, as the conduit kernel's
// FallbackConduitLiquidDensityKgM3 does (same value; conduits.h keeps its own name for its own
// two rules).
// The tunable `FallbackCellLiquidDensityKgM3` (tunables.def).

// One cell's HeatExchangeRatio() from its dominant element, mass and temperature -- the same
// reading `CellExposure` makes of a cell for radiation. A gas cell by its pressure over the sim's
// cell volume, a liquid cell by its liquid volume ratio, vacuum 0, an element the table does not
// know 1 (left to Klei's arithmetic rather than guessed as insulating).
//
// `solid_ratio` is the one thing the two callers disagree on, and both are deliberate. The
// conduit CONVECTION policy passes 1: an ONI pipe can run through a tile, and there it touches
// matter (conduits.h, CellHeatExchangeRatio). The building convection term passes 0, as
// Stationeers' `!CanContainAtmos` does: the building's exchange with a tile is Klei's
// conduction sweep, and this term must not be a second copy of it.
inline float CellHeatExchangeRatioOf(const ElementTable& table, uint16_t element, float mass,
                                     float temperature, float solid_ratio,
                                     const Tunables& tun = g_tunables) {
  if (!(mass > 0.0f)) return 0.0f;
  if (element >= table.Count()) return 1.0f;
  switch (table.Phase(element)) {
    case kStateGas: {
      const float molar = table.MolecularMassOf(element);
      if (!(molar > 0.0f)) return 0.0f;
      const float moles = mass * gas::kGramsPerKilogram / molar;
      return HeatExchangeRatio(
          gas::PressureFromMoles(moles, temperature, tun.cell_volume_m3, tun.gas_constant_r), 0.0f,
          tun);
    }
    case kStateLiquid: {
      const float density = table.LiquidDensityOf(element);
      const float litres_ratio =
          mass / (density > 0.0f ? density : tun.fallback_cell_liquid_density_kg_m3) /
          tun.cell_volume_m3;
      return HeatExchangeRatio(0.0f, litres_ratio, tun);
    }
    case kStateSolid:
      return solid_ratio;
    default:
      return 0.0f;
  }
}

// The temperature write `ApplyCellEnergy` performs (simdll.cpp), factored out so the exhaust
// path and the `ModifyCellEnergy` message handler cannot drift apart -- they are the same
// arithmetic, and vanilla exhaust reaches the cells through that very message.
//
// Returns whether the write was REACHED, and reports through `*landed_kj` the kilojoules that
// actually landed -- which is NOT `kilojoules`: a vacuum cell, a cell under 0.001 kg, and a
// cell already at or above `max_temperature` each refuse part or all of the lump.
//
// The two results are separate on purpose and the distinction is load-bearing. Klei runs
// `DoStateTransition` and marks the cell dirty on every path that reaches the write, INCLUDING
// the ones where the write moves the temperature by exactly zero -- zero kilojoules against a
// live ceiling is one, and `diffsim --scenario cellenergy` covers it. Collapsing "reached the
// write" into "landed something" would silently drop that transition. Returning only the
// kilojoules was the first shape of this function and it had exactly that bug.
//
// Deliberately does not run the transition itself -- the caller decides, because Klei's
// message drain does it and the building kernel's own sweep does it separately.
//
// `carry` (the `CellEnergyCarry` tunable; null is Klei's arithmetic, bit for bit) is the cell's
// rounding remainder. It is added to the payment going in, and on the way out it holds what the
// float temperature write did not land. A payment the ceiling or the range clamp cut short, or
// one refused outright, leaves it at zero: those are refusals, and the caller books them as such.
inline bool DeliverCellEnergy(PhaseEntry* c, const Element& e, float kilojoules,
                              float max_temperature, double* landed_kj,
                              const Tunables& tun = g_tunables, double* carry = nullptr) {
  *landed_kj = 0.0;
  const double owed = carry ? static_cast<double>(kilojoules) + *carry : 0.0;
  if (carry) *carry = 0.0;
  if ((e.state & kStateMask) == 0) return false;
  const float mass = c->mass;
  if (!(mass > tun.cell_energy_min_mass_kg) || !(max_temperature > 0.0f)) return false;
  const float t = c->temperature;
  float next = t;
  if (t <= max_temperature) next = max_temperature;
  const float asked = carry ? static_cast<float>(owed) : kilojoules;
  const float raised = t + asked / (mass * e.specificHeatCapacity);
  if (raised <= next) next = raised;
  if (next <= 0.0f || next > tun.max_temperature) {
    if (tun.max_temperature <= next) next = tun.max_temperature;
    if (next <= tun.cell_energy_min_temperature) next = tun.cell_energy_min_temperature;
  }
  if (!(next > 0.0f)) return false;
  c->temperature = next;
  *landed_kj = static_cast<double>(mass) * static_cast<double>(e.specificHeatCapacity) *
               (static_cast<double>(next) - static_cast<double>(t));
  if (carry && next == raised) *carry = owed - *landed_kj;
  return true;
}

inline void StepBuildingHeatExchange(World* w, const ElementTable& table,
                                     BuildingState* state, float dt,
                                     std::vector<StateChangeOre>* ores, size_t ri,
                                     const std::vector<uint8_t>* sky = nullptr,
                                     const std::vector<uint8_t>* beam = nullptr,
                                     const uint8_t* visible = nullptr,
                                     bool debug_editing = false,
                                     std::vector<SpawnFallingLiquidInfo>* falling = nullptr) {
  BuildingEvents& ev = state->events;
  // `SimData::UpdateComponents` sits inside `UpdateData`'s region loop, so
  // this runs once per region and a building inside two of them exchanges twice. The
  // temperature report is indexed by slot rather than appended, so it is cleared on the
  // first region only and a building reached twice reports the second result.
  if (ri == 0) ev.temperatures.assign(state->exchange.SlotCount(), BuildingTemperatureInfo{});
  // Same lifetime as the temperature report above, and cleared in the same place: region 0
  // is the first region of every substep (simdll.cpp runs `for substep { for region }`).
  if (ri == 0) state->waste_heat_charged.assign(state->exchange.SlotCount(), 0);
  const Tunables tun = g_tunables;  // tunables.h, THE ONE RULE
  const PhaseRules phase_rules;     // reads the transition tunables once, here
  const float cell_transfer = CellTransferScale(tun);

  // THE PLANET THIS REGION IS ON (abi/sim_abi_ext.h's kSetWorldEnvironment, Mod 3). Resolved
  // once per call, not per building: `Game.UnsafeSim200ms` builds one active region per
  // discovered world, so every building in this call shares one environment, and asking per
  // building would be the same rectangle walk repeated a thousand times.
  //
  // A world with no record gets the fallback record, which is all zeroes unless a caller set one
  // -- so every world that predates this, and the whole offline suite, takes the branch below
  // that does nothing.
  const World::Environment& env = w->EnvironmentOfWorld(w->WorldIndexOfRegion(ri));
  // Which exposure the solar term below reads: the direct beam on a world the sim has a sun for
  // (`kSetWorldSun`, Mod 3's sun path), the sky view everywhere else. Radiation needs no field at
  // all -- `CellExposure` is what it reads -- so this is the only place a building sees the beam.
  World::SunDirection sun_dir;
  const std::vector<uint8_t>* light =
      (beam != nullptr && sky != nullptr && beam->size() == sky->size() &&
       w->SunOfWorld(w->WorldIndexOfRegion(ri), &sun_dir))
          ? beam
          : sky;
  // 0 at night, 1 at noon, and 0 during an eclipse because `TimeOfDay` already zeroes the lux
  // for one. Formed here rather than sent per tick because `NewGameFrame` carries the lux
  // every frame for free -- see World::RegionSunlight.
  float sun_fraction = 0.0f;
  if (env.full_sun_lux > 0.0f && env.peak_irradiance_w_m2 > 0.0f) {
    sun_fraction = w->RegionSunlight(ri) / env.full_sun_lux;
    if (sun_fraction < 0.0f) sun_fraction = 0.0f;
    if (sun_fraction > 1.0f) sun_fraction = 1.0f;
  }

  const int32_t pw = w->PaddedWidth();
  std::vector<PhaseEntry>& cells = w->Phases();
  const std::vector<uint8_t>& insulation = w->Insulation();
  std::vector<BuildingHeatExchangeData>& all = state->exchange.Data();
  const std::vector<int32_t>& handles = state->exchange.Handles();

  for (size_t b = 0; b < all.size(); ++b) {
    BuildingHeatExchangeData& d = all[b];
    const int32_t handle = handles[b];

    // Extents against the active region, not cells against it. A building hanging over the
    // edge is skipped whole and does not even report a temperature.
    if (!w->ExtentsInRegion(ri, d.min_x - 1, d.min_y - 1, d.max_x - 1, d.max_y - 1)) {
      continue;
    }

    // ------------------------------------------------------------------ framework block
    // Exhaust delivery, the bounce, and radiation. All three are framework extensions
    // (abi/sim_abi_ext.h) with no vanilla counterpart, all three are ONCE PER SUBSTEP for the
    // same reason the waste-heat rate is -- see the long note further down -- and all three
    // are inert until the extension message is sent, so byte-identity is untouched.
    //
    // Placed BEFORE the conduction sweep on purpose. Vanilla's exhaust arrives as
    // `ModifyCellEnergy` messages, which drain at the top of the frame before any kernel
    // looks at a cell; running the delivery here is the closest a per-substep rate gets to
    // that ordering, and it means the conduction sweep below sees the cells as the exhaust
    // left them, exactly as vanilla's does.
    {
      const size_t fw_slot = static_cast<size_t>(handle & kHandleIndexMask);
      const bool wants_exhaust = d.exhaust_kilowatts != 0.0f;
      const bool wants_radiation = d.radiation_factor > 0.0f;
      const bool wants_convection = d.convection_factor > 0.0f;
      if ((wants_exhaust || wants_radiation || wants_convection) &&
          fw_slot < state->waste_heat_charged.size() &&
          (state->waste_heat_charged[fw_slot] & 0x2) == 0) {
        state->waste_heat_charged[fw_slot] |= 0x2;
        const float n_cells = static_cast<float>(d.CellCount());
        const float body_hc = d.per_cell_heat_capacity * n_cells;

        // ---- exhaust ----
        //
        // Klei's arithmetic, reproduced: the same per-cell split, the same 1.5 kg mass
        // factor, the same ceiling, the same three refusals inside DeliverCellEnergy. The
        // ONLY change is the last two lines -- what vanilla drops on the floor goes into the
        // building's own body instead.
        if (wants_exhaust && n_cells > 0.0f) {
          const double asked = static_cast<double>(dt) *
                               static_cast<double>(d.exhaust_kilowatts);
          const double per_cell = asked / static_cast<double>(n_cells);
          double delivered = 0.0;
          for (int32_t y = d.min_y; y < d.max_y; ++y) {
            for (int32_t x = d.min_x; x < d.max_x; ++x) {
              const size_t i = static_cast<size_t>(y) * pw + x;
              PhaseEntry& c = cells[i];
              const float m = c.mass;
              const float factor =
                  (m < tun.exhaust_max_pressure ? m : tun.exhaust_max_pressure) /
                  tun.exhaust_max_pressure;
              double landed = 0.0;
              DeliverCellEnergy(&c, table.At(c.element),
                                static_cast<float>(per_cell * factor),
                                d.exhaust_max_temperature, &landed, tun);
              delivered += landed;
            }
          }
          const double bounced = asked - delivered;
          if (body_hc > 0.0f) {
            d.temperature += static_cast<float>(bounced / static_cast<double>(body_hc));
            // The whole rate is an inflow: the building's operation invented it, exactly as
            // `operating_kilowatts` does, and all of it is now inside the sim -- part in the
            // cells, part in the body.
            w->NoteBuildingExhaust(asked);
            w->NoteBuildingExhaustBounced(bounced);
          } else {
            // No body to absorb it: a bypassed building, or one registered with no mass.
            // This is the one case the fix cannot rescue, so it is charged to the deletion
            // bucket rather than quietly folded into the inflow.
            w->NoteBuildingExhaust(delivered);
            w->NoteCellEnergyRefused(bounced);
          }
        }

        // ---- convection ----
        //
        // abi/sim_abi_ext.h's kSetBuildingConvection: Stationeers-style convection between
        // the building's body and each footprint cell,
        //
        //   q = 100 W/(m^2 K) * area/n * factor * cell HeatExchangeRatio() * (T_body - T_cell) * dt
        //
        // with the body's own ratio 1 (it is a solid) and a solid cell's 0 (Klei's conduction
        // sweep below owns tiles). Limited to the pair's equilibrium, as Stationeers'
        // `GasMixture.TransferEnergyTo` limits every exchange, so it can equalise the two and
        // never push either past the other.
        //
        // Placed before radiation because Stationeers convects first and radiates second
        // (`PipeRadiator.OnAtmosphericTick`). Stationeers then subtracts any positive convection
        // from the radiated heat; that is not reproduced, because here radiation is already
        // scaled to zero by the same medium that carries convection (CellExposure), so the two
        // cannot both be large in one cell.
        //
        // Energy moves between two bodies the sim already holds, so the ledger sees only the
        // float rounding of the two writes, charged to the same bucket as Klei's conduction:
        // (what the body gained) - (what the cells lost).
        if (wants_convection && body_hc > 0.0f && n_cells > 0.0f) {
          const double area = d.convection_area_m2 > 0.0f
                                  ? static_cast<double>(d.convection_area_m2)
                                  : static_cast<double>(n_cells);
          // THE REACH, and why a radiator needs one. Everything below would otherwise run over
          // the footprint alone, and a building pushing kilowatts into the one cell it stands in
          // pins that cell within a tick or two: after that the exchange is limited by how fast
          // ONI's own cell-to-cell conduction drains that cell, not by this term at all.
          // Measured on the RADIATOR rig: a radiator at Stationeers' own factor shed
          // 63.6 kW against a plain Radiant Pipe's 61.8 kW (1.03x), and raising the factor a
          // HUNDREDFOLD moved that only to 69.3 kW (1.14x) -- the factor was never the limit.
          // Spreading the same total conductance over the open cells around the building is what
          // makes it a room device: a radiator exchanges with the room, a pipe with its own tile
          // only.
          int32_t reach = d.convection_reach;
          if (reach < 0) reach = 0;
          if (reach > tun.max_building_convection_reach) reach = tun.max_building_convection_reach;
          const int32_t lo_x = d.min_x - reach < 1 ? 1 : d.min_x - reach;
          const int32_t lo_y = d.min_y - reach < 1 ? 1 : d.min_y - reach;
          const int32_t hi_x = d.max_x + reach > pw - 1 ? pw - 1 : d.max_x + reach;
          const int32_t ph = w->PaddedHeight();
          const int32_t hi_y = d.max_y + reach > ph - 1 ? ph - 1 : d.max_y + reach;

          // FIRST PASS: how many cells can take any of it. A cell with no ratio -- vacuum, or a
          // solid tile ONI's conduction already owns -- is not counted, so a radiator in a
          // half-buried alcove puts its whole area into the cells that are actually there rather
          // than losing it to the rock.
          int32_t participating = 0;
          for (int32_t y = lo_y; y < hi_y; ++y) {
            for (int32_t x = lo_x; x < hi_x; ++x) {
              const PhaseEntry& c = cells[static_cast<size_t>(y) * pw + x];
              if (CellHeatExchangeRatioOf(table, c.element, c.mass, c.temperature, 0.0f, tun) >
                  0.0f) {
                ++participating;
              }
            }
          }

          if (participating > 0) {
            const double k_cell_kw =
                static_cast<double>(tun.building_convection_w_per_m2k) * 0.001 *
                static_cast<double>(d.convection_factor) * area /
                static_cast<double>(participating);
            const double bhc = static_cast<double>(body_hc);
            double body_gained = 0.0;
            double cells_lost = 0.0;
            for (int32_t y = lo_y; y < hi_y; ++y) {
              for (int32_t x = lo_x; x < hi_x; ++x) {
                PhaseEntry& c = cells[static_cast<size_t>(y) * pw + x];
                if (!(c.mass > 0.0f)) continue;
                const float cell_hc_f = c.mass * table.At(c.element).specificHeatCapacity;
                if (!(cell_hc_f > 0.0f)) continue;
                const float ratio =
                    CellHeatExchangeRatioOf(table, c.element, c.mass, c.temperature, 0.0f, tun);
                if (!(ratio > 0.0f)) continue;
                const double chc = static_cast<double>(cell_hc_f);
                const double tb = static_cast<double>(d.temperature);
                const double tc = static_cast<double>(c.temperature);
                // Positive is body -> cell.
                double kj = k_cell_kw * static_cast<double>(ratio) * (tb - tc) *
                            static_cast<double>(dt);
                const double t_eq = (bhc * tb + chc * tc) / (bhc + chc);
                const double limit = (tb - t_eq) * bhc;
                if (std::fabs(kj) > std::fabs(limit)) kj = limit;
                if (!(kj != 0.0)) continue;
                // THE CELL IS WRITTEN FIRST AND THE BODY IS DEBITED BY WHAT LANDED, not by what
                // was proposed. A cell's temperature is a float: a heavy cell -- a full tile of
                // water is ~4200 kJ/K -- can round a small transfer away to nothing, and taking
                // it off the body anyway would delete that energy silently. Found by this term's
                // first vftest run: a water cell gained exactly 0.000000 kJ while the body cooled
                // by 0.98 kJ. What the cell cannot take, the body keeps.
                //
                // The rounding runs both ways -- a heavy cell can also land slightly MORE than
                // was proposed, which is the 1.8% the same arm's water case sits above the closed
                // form by -- and debiting what landed is what keeps the pair conservative either
                // way. The proposal is a rate; the transfer is what a float could record.
                const float new_cell = static_cast<float>(tc + kj / chc);
                const double landed = (static_cast<double>(new_cell) - tc) * chc;
                if (!(landed != 0.0)) continue;
                const float new_body = static_cast<float>(tb - landed / bhc);
                cells_lost -= landed;
                body_gained += (static_cast<double>(new_body) - tb) * bhc;
                c.temperature = new_cell;
                d.temperature = new_body;
                TransitionCell(w, table, x, y, ores, phase_rules, nullptr, visible, debug_editing,
                               falling);
              }
            }
            w->NoteBuildingExchange(body_gained - cells_lost);
          }
        }

        // ---- radiation ----
        //
        // Follows the structure of Stationeers' radiative exchange; see
        // abi/sim_abi_ext.h's kSetBuildingRadiation for the original alongside this. Three
        // properties carried over verbatim, because each one is load-bearing:
        //
        //   * ONE-WAY. It only ever sheds. A body colder than the sink is left alone rather
        //     than warmed, so this can never become a back-door heat source.
        //   * THE LOCAL MEDIUM SUPPRESSES IT, and this is the one place the port deviates
        //     from Stationeers on purpose. Stationeers lerps the SINK toward the local
        //     medium; reproducing that verbatim was tried and is wrong for ONI, because the
        //     flux is then sigma*A*(T_body^4 - T_cell^4) -- a second building <-> cell heat
        //     path running alongside the conduction sweep below, with the energy leaving the
        //     world instead of reaching the cell. It is exactly the magic heat deletion this
        //     work exists to remove, with a label on it.
        //
        //     Measured, not argued: against a real 234-cycle colony that form radiated
        //     55750 kJ over 32 s of sim time against 1380 kJ of exhaust -- roughly 1.7 MW of
        //     continuous cooling leaving the world. The offline arm passed only because the
        //     body and the room were at exactly the same temperature, which is the degenerate
        //     case.
        //
        //     Stationeers gets away with the lerp because CalculateThingEntropy is its
        //     device's ONLY exchange with the room -- its things have no conduction. ONI's
        //     buildings do. So the medium fraction scales the FLUX toward zero here rather
        //     than retargeting it: a cell holding a real medium is opaque and radiation is
        //     exactly 0, a vacuum is transparent and radiation is the only outlet left. That
        //     is also the more physical statement of the two.
        //
        //     "A real medium" is a PRESSURE test against Stationeers' own Armstrong limit --
        //     see CellExposure. Reusing Klei's 1.5 kg exhaust saturation for it, which the
        //     first version did, called ordinary breathable base air 20% vacuum and leaked
        //     ~1.1 MW out of a real colony.
        //   * AN OVERSHOOT CLAMP. Stationeers clamps to half the body's TOTAL energy, which
        //     in ONI's units is enormous and would never bite; the meaningful guard, and the
        //     one its `GasMixture.TransferEnergyTo` primitive actually implements elsewhere,
        //     is half the energy *above the sink*. That is what is used here, and it is the
        //     one deliberate deviation.
        //
        // The saturation mass for the lerp is Klei's own `MAX_PRESSURE`: the mass at which
        // vanilla considers a cell able to take a building's full exhaust is the same mass at
        // which it is able to carry the heat away, and reusing it avoids inventing a constant.
        if (wants_radiation && body_hc > 0.0f && n_cells > 0.0f) {
          const float t_body = d.temperature;
          // PER WORLD. `SinkTemperatureForWorld` answers with this
          // world's own sink if it has one and the single global scalar
          // (`kSetEnvironmentTemperature`) if it does not, which is what leaves every caller
          // and every vftest arm written before this unchanged.
          const float t_sink = w->SinkTemperatureForWorld(w->WorldIndexOfRegion(ri));
          // How much of the building has nothing between it and the environment, averaged
          // over its footprint. See CellExposure for why the threshold is a pressure against
          // Stationeers' Armstrong limit rather than a mass against Klei's exhaust saturation
          // -- that mistake cost a live run and about 1.1 MW of invisible colony cooling.
          double exposure_sum = 0.0;
          for (int32_t y = d.min_y; y < d.max_y; ++y) {
            for (int32_t x = d.min_x; x < d.max_x; ++x) {
              exposure_sum += CellExposure(cells[static_cast<size_t>(y) * pw + x], table, tun);
            }
          }
          const double exposure = exposure_sum / static_cast<double>(n_cells);
          if (exposure > 0.0 && t_body > t_sink) {
            const double area = d.radiation_area_m2 > 0.0f
                                    ? static_cast<double>(d.radiation_area_m2)
                                    : static_cast<double>(n_cells);
            const double tb = static_cast<double>(t_body);
            const double ts = static_cast<double>(t_sink);
            const double flux_kw = tun.stefan_boltzmann_kw *
                                   static_cast<double>(d.radiation_factor) * area * exposure *
                                   (tb * tb * tb * tb - ts * ts * ts * ts);
            double kj = flux_kw * static_cast<double>(dt);
            const double half_excess = 0.5 * static_cast<double>(body_hc) * (tb - ts);
            if (kj > half_excess) kj = half_excess;
            if (kj > 0.0) {
              d.temperature = t_body - static_cast<float>(kj / static_cast<double>(body_hc));
              w->NoteRadiatedToEnvironment(kj);
            }
          }
        }

        // SOLAR ABSORPTION -- the same boundary, inwards (kSetWorldEnvironment, Mod 3).
        //
        // KIRCHHOFF'S LAW IS WHY THIS NEEDS NO NEW PER-BUILDING FIELD. A grey body absorbs and
        // emits with the same coefficient, so the building's `radiation_factor` -- its
        // emissivity, already set by kSetBuildingRadiation -- is also its absorptivity. A
        // building that radiates well warms well in the sun, and a building that cannot do
        // one cannot do the other. Stationeers instead carries a per-prefab
        // `SolarHeatingScale` (0.1 on an ordinary thing, 2 on a Medium Radiator, 50 on the
        // Large Extendable) which lets a prefab be a perfect absorber and a perfect insulator
        // at once; that is a tuning knob, and this is the physics.
        //
        // TWO EXPOSURES, AND BOTH ARE REQUIRED. `CellExposure` says there is no medium in the
        // way, which is what radiation needs. Sunlight additionally needs a clear column to
        // the sky, or a building in a sealed vacuum cellar would sunbathe through bedrock.
        // That second test is the sunlight texture the sim already computes per world per
        // frame (`ComputeSunlight`), read here one frame stale -- which at 255 steps of a
        // shadow and a 200 ms frame is not a distinction anything can observe. On a world with
        // a sun that texture is the direct beam instead (`light`, above), so a building in the
        // shadow of a cliff is in the shade even with open sky over it.
        if (sun_fraction > 0.0f && d.radiation_factor > 0.0f && body_hc > 0.0f &&
            n_cells > 0.0f && light != nullptr && !light->empty()) {
          const int32_t gw = w->GameWidth();
          const int32_t gh = w->GameHeight();
          double lit_sum = 0.0;
          for (int32_t y = d.min_y; y < d.max_y; ++y) {
            for (int32_t x = d.min_x; x < d.max_x; ++x) {
              const int32_t gx = x - 1;
              const int32_t gy = y - 1;
              if (gx < 0 || gy < 0 || gx >= gw || gy >= gh) continue;
              const size_t gi = static_cast<size_t>(gy) * static_cast<size_t>(gw) +
                                static_cast<size_t>(gx);
              if (gi >= light->size()) continue;
              // The texture is exposure * 255 truncated, so a fully lit cell reads 255.
              const double lit = static_cast<double>((*light)[gi]) / 255.0;
              if (lit <= 0.0) continue;
              lit_sum += lit * CellExposure(cells[static_cast<size_t>(y) * pw + x], table, tun);
            }
          }
          if (lit_sum > 0.0) {
            const double area = d.radiation_area_m2 > 0.0f
                                    ? static_cast<double>(d.radiation_area_m2)
                                    : static_cast<double>(n_cells);
            // Irradiance is W/m^2 and the sim works in kilowatts and kilojoules.
            const double flux_kw = 0.001 * static_cast<double>(env.peak_irradiance_w_m2) *
                                   static_cast<double>(sun_fraction) *
                                   static_cast<double>(d.radiation_factor) * area *
                                   (lit_sum / static_cast<double>(n_cells));
            const double kj = flux_kw * static_cast<double>(dt);
            if (kj > 0.0) {
              d.temperature =
                  d.temperature + static_cast<float>(kj / static_cast<double>(body_hc));
              w->NoteAbsorbedFromEnvironment(kj);
            }
          }
        }
      }
    }

    const float t_building = d.temperature;
    if (d.per_cell_heat_capacity > 0.0f && t_building >= 0.0f) {
      const float building_hc = d.per_cell_heat_capacity;
      float accumulated = 0.0f;
      float seen_min = t_building, seen_max = t_building;
      // Energy the cells actually gave up this sweep, in joules and in double. Kept
      // alongside `accumulated` (which is what the building *claims* to have received)
      // because the two are not the same number -- see the ledger charge below.
      double cells_lost = 0.0;

      for (int32_t y = d.min_y; y < d.max_y; ++y) {
        for (int32_t x = d.min_x; x < d.max_x; ++x) {
          const size_t i = static_cast<size_t>(y) * pw + x;
          PhaseEntry& c = cells[i];
          if (c.mass <= 0.0f) continue;
          const Element& e = table.At(c.element);

          const float ins = static_cast<float>(insulation[i]);
          const float insulation_factor = ins * ins;
          const float cell_hc = c.mass * e.specificHeatCapacity;
          // NEUTRONIUM. `Unobtanium` ships `specificHeatCapacity: 0`, so a 2000 kg tile of it
          // passes the mass guard above with a heat capacity of exactly zero, and the
          // `t_cell - q / cell_hc` below is a 0/0. That poisons the CELL with NaN; on the next
          // sweep its temperature is NaN, `delta` is NaN, `source_hc` takes the building's
          // branch, `accumulated` is NaN, and the BUILDING follows. Worse than the unreadable
          // number: `NaN > overheat_temperature` is false, so every temperature gate on that
          // building then fails silently and nothing reports it.
          //
          // Skipping is the physical answer as well as the safe one. A cell that can hold no
          // heat can neither give nor take any, so the correct energy transfer is exactly the
          // zero this `continue` produces -- and neutronium's thermal conductivity is 0 as
          // well, so `q` was already going to be zero on the branch that does not divide.
          //
          // This is NOT a general float-safety guard bolted onto Klei's arithmetic, and it is
          // deliberately written as the narrow one. Every element in the game except this one
          // has a nonzero specific heat, so for every cell a player can ever mine, build or
          // create this test cannot fire and the sweep below is byte-identical to Klei's.
          //
          // It has to exist because neutronium does. The player cannot mine or make it, but
          // the world is full of it: it closes off the sides and the bottom of the asteroid
          // and it is the floor geysers sit on. Anything with a structure temperature that
          // ends up against one of those tiles -- directly, or through an `OverrideExtents`
          // that reaches a row below itself, as the Steam Turbine does -- hits
          // this. The material is left alone and this clause carries it.
          if (!(cell_hc > 0.0f)) continue;
          const float t_cell = c.temperature;
          const float delta = t_cell - t_building;

          // The multiply order is Klei's, not algebra's. Every one of these is float32 and
          // reassociating them moves the last bits, which is the difference between a
          // scenario that scores zero and one that scores 1e-5 and has to be argued about.
          const float source_hc = delta >= 0.0f ? cell_hc : building_hc;
          float q = source_hc * delta;
          q *= e.thermalConductivity * d.thermal_conductivity;
          q *= insulation_factor;
          q *= cell_transfer;
          q *= dt * state->temperature_scale;

          const float hi = t_building > t_cell ? t_building : t_cell;
          const float lo = t_building < t_cell ? t_building : t_cell;
          if (hi > seen_max) seen_max = hi;
          if (lo < seen_min) seen_min = lo;

          float new_cell = Clamp(t_cell - q / cell_hc, lo, hi);
          const float new_building = Clamp(t_building + q / building_hc, lo, hi);
          // NOT `< 0.0f`: in the game an unordered compare takes the equilibrium branch. Spelled as
          // the negation of `>= 0` so a NaN product lands on that branch the way it does in
          // the DLL, instead of falling through and leaving the NaN in the cell.
          if (!((new_building - new_cell) * (t_building - t_cell) >= 0.0f)) {
            // The two would have crossed. Put them where they were always going to end up.
            // Klei divides once and scales each side, rather than dividing the sum.
            const float inv_sum = 1.0f / (building_hc + cell_hc);
            const float equilibrium =
                Clamp(building_hc * inv_sum * t_building + inv_sum * cell_hc * t_cell, lo,
                      hi);
            new_cell = equilibrium;
            q = (equilibrium - t_building) * building_hc;
          }

          // Measured before TransitionCell runs, and with the heat capacity the cell had
          // on the way in: once the cell has boiled it is a different element with a
          // different specific heat, and that energy belongs to the phase-change site, not
          // to this one.
          cells_lost += (static_cast<double>(t_cell) - static_cast<double>(new_cell)) *
                        static_cast<double>(cell_hc);
          c.temperature = new_cell;
          // Klei runs the transition test on the cell here, inside the sweep, so a cell a
          // building has just boiled is already the new element before the next building
          // reads it.
          TransitionCell(w, table, x, y, ores, phase_rules, nullptr, visible, debug_editing,
                         falling);
          accumulated += q;
        }
      }

      const float n = static_cast<float>(d.CellCount());
      const float total_hc = n * building_hc;
      float next = Clamp(t_building + accumulated / total_hc, seen_min, seen_max);
      // The energy ledger's second building bucket, and it exists because this exchange
      // **does not conserve energy** -- Klei's algorithm, faithfully reproduced, not a bug
      // introduced here.
      //
      // Each cell moves by `Clamp(t_cell - q/cell_hc, lo, hi)` and the building moves by
      // `Clamp(t_building + sum(q)/total_hc, seen_min, seen_max)`. Two independent clamps
      // over two different intervals: whenever either bites, what the cells gave up and
      // what the building received are different numbers, and the difference is created or
      // destroyed outright. The cross-over guard recomputes `q` from where the cell landed,
      // which fixes that one pair, but nothing reconciles the building's own clamp against
      // the sum.
      //
      // Charged as (what the building gained) - (what the cells lost), so a positive value
      // is energy the sim invented. Measured before `operating_kilowatts` goes in, so the
      // two building buckets stay separable: one is the power grid, this one is the
      // algorithm.
      w->NoteBuildingExchange(
          (static_cast<double>(next) - static_cast<double>(t_building)) *
              static_cast<double>(total_hc) -
          cells_lost);
      // Operating heat is added *after* the clamp, so a machine can drive itself past
      // everything it touches. This is where an overheating aquatuner comes from.
      //
      // THE TWO RATES ARE APPLIED ON DIFFERENT SCHEDULES, DELIBERATELY.
      //
      // `operating_kilowatts` is Klei's and is applied ONCE PER REGION, because this whole
      // function runs once per region and that is what vanilla does -- a building covered by
      // two regions gets its self-heat twice. Measured, not assumed: `diffsim --scenario
      // buildinglap` charges +160 kJ where the identical machine under one region charges
      // +80, and the game's own DLL agrees with this one at 301.01685 K to the last digit.
      // Reproducing that exactly is the point of a compatible sim, so it is left alone.
      //
      // `waste_heat_kilowatts` is the framework's (abi/sim_abi_ext.h) and is applied ONCE
      // PER SUBSTEP, on whichever region reaches the building first. It is a new field with
      // no vanilla behaviour to preserve, and a derived power -> heat rule whose output
      // depended on a region layout the player can neither see nor choose would not be a
      // rule -- the same wattage would produce different heat in two colonies laid out
      // differently. This is also what Stationeers does: its waste-heat charge runs once
      // per tick over a flat list of devices, with no spatial partitioning.
      //
      // Byte-identity is untouched either way: `waste_heat_kilowatts` is 0 until the
      // extension message is sent, and 0 applied once is the same as 0 applied twice.
      float rate = d.operating_kilowatts;
      const size_t charge_slot = static_cast<size_t>(handle & kHandleIndexMask);
      bool charge_waste = false;
      if (d.waste_heat_kilowatts != 0.0f &&
          charge_slot < state->waste_heat_charged.size() &&
          (state->waste_heat_charged[charge_slot] & 0x1) == 0) {
        state->waste_heat_charged[charge_slot] |= 0x1;
        charge_waste = true;
        rate += d.waste_heat_kilowatts;
      }
      next += (dt * rate) / total_hc;
      // The energy ledger's one charged bucket (World::EnergyLedger). This line is the
      // *only* place electrical energy enters the sim's thermal state: vanilla self-heat
      // and the framework's derived power -> heat rule both arrive as `operating_kilowatts`
      // on this same field. Charged unconditionally -- including when the building is
      // about to be reported overheated or melted -- because the energy went in either way
      // and a bucket that skipped those cases would hide exactly the runs worth looking at.
      //
      // `dt` is seconds and `operating_kilowatts` is kW, so the product is kJ, which is the
      // ledger's unit throughout: ONI's `specificHeatCapacity` is kJ/(kg K), so grid,
      // building, conduit and chunk energies are all kJ as well. This charge was briefly
      // written with a *1000 "to convert to joules", and the ledger caught it on its first
      // run -- `buildingrun` reported a charge of 80000 against 80 actually landing
      // (50 ticks x 0.2 s x 8 kW). Recorded here because the first thing the instrument
      // found was a bug in its own author's code.
      if (d.operating_kilowatts != 0.0f) {
        w->NoteBuildingOperating(static_cast<double>(dt) *
                                 static_cast<double>(d.operating_kilowatts));
      }
      // Charged to its own bucket, never folded into the one above. Keeping the framework's
      // derived power -> heat rule separable from Klei's hand-tuned self-heat for the whole
      // run is most of the reason the field exists at all: it is what lets the ledger answer
      // "how much of this colony's heat came from the power grid" rather than just "how much
      // heat appeared".
      if (charge_waste) {
        w->NoteBuildingWasteHeat(static_cast<double>(dt) *
                                 static_cast<double>(d.waste_heat_kilowatts));
      }

      if (next >= d.overheat_temperature) ev.overheated.push_back(handle);
      if (t_building >= d.overheat_temperature && next < d.overheat_temperature) {
        ev.no_longer_overheated.push_back(handle);
      }
      if (next >= d.melt_temperature) ev.melted.push_back(handle);
      d.temperature = next;
    }

    const size_t slot = static_cast<size_t>(handle & kHandleIndexMask);
    if (slot < ev.temperatures.size()) {
      ev.temperatures[slot].handle = handle;
      ev.temperatures[slot].temperature = d.temperature;
    }
  }
}

// --------------------------------------------------- the building-to-building exchange

// `BuildingToBuildingHeatExchange::Update`.
//
// The same shape as the cell exchange with three differences, all of them load-bearing:
//
//   * it uses the **whole** building's heat capacity on both sides, not the per-cell one;
//   * there is no insulation term, because there is no cell to read one from;
//   * the clamp interval is computed once from *every* building in the group, in a first
//     pass, rather than per pair — so a chain of touching buildings cannot ping-pong heat
//     past the coldest and hottest members of the chain.
//
// The energy is also clamped by how far *both* sides could actually move, `min` of the two,
// with the sign put back afterwards. `cells_in_contact` plays no part.
inline void StepBuildingToBuilding(World* w, BuildingState* state, float dt, size_t ri) {
  BuildingEvents& ev = state->events;
  const float transfer_rate = g_tunables.building_transfer_rate;  // tunables.h, THE ONE RULE
  std::vector<BuildingToBuildingData>& groups = state->contact.Data();

  for (BuildingToBuildingData& g : groups) {
    BuildingHeatExchangeData* self = state->exchange.Get(g.self);
    if (!self) continue;
    if (!w->ExtentsInRegion(ri, self->min_x - 1, self->min_y - 1, self->max_x - 1,
                            self->max_y - 1)) {
      continue;
    }

    float seen_min = self->temperature, seen_max = self->temperature;
    for (const InContactBuilding& c : g.contacts) {
      const BuildingHeatExchangeData* other = state->exchange.Get(c.handle);
      if (!other) continue;
      if (other->temperature < seen_min) seen_min = other->temperature;
      if (other->temperature > seen_max) seen_max = other->temperature;
    }

    const float self_hc = self->heat_capacity;
    for (const InContactBuilding& c : g.contacts) {
      BuildingHeatExchangeData* other = state->exchange.Get(c.handle);
      if (!other) continue;
      const float other_hc = other->heat_capacity;
      if (other_hc <= 0.0f) continue;

      const float t_other = other->temperature;
      const float t_self = self->temperature;
      const float delta = t_other - t_self;
      const float source_hc = delta >= 0.0f ? other_hc : self_hc;
      float q = other->thermal_conductivity * self->thermal_conductivity;
      q *= delta * source_hc;
      q = (dt * state->to_building_temperature_scale) * q;
      q *= transfer_rate;

      const float inv_self = 1.0f / self_hc;
      const float inv_other = 1.0f / other_hc;
      const float self_room =
          Abs(Clamp(t_self + inv_self * q, seen_min, seen_max) - t_self) * self_hc;
      const float other_room =
          Abs(Clamp(t_other - inv_other * q, seen_min, seen_max) - t_other) * other_hc;
      // SSE min with the OTHER side's room first, so a NaN yields `self_room`. The operands
      // are in the game's order rather than the readable one for exactly that reason.
      float energy = MinSS(other_room, self_room);
      if (q < 0.0f) energy = -energy;

      float new_self = t_self + inv_self * energy;
      float new_other = t_other - inv_other * energy;
      // The unordered-compare case again, and this is the branch that decides what a
      // group owner with no heat capacity does. `1 / self_hc` is an infinity, `q` is zero
      // because `source_hc` is that same zero, and `inf * 0` makes `new_self` a NaN; the
      // unordered compare then sends the pair here and both ends come out at the weighted
      // equilibrium, which is the other building's temperature. Written `< 0.0f` the branch
      // is skipped and the NaN is published — `diffsim --scenario b2bzerohc`, klei 300.00000
      // against mine `nan`.
      if (!((new_self - new_other) * (t_self - t_other) >= 0.0f)) {
        const float equilibrium =
            Clamp((t_other * other_hc + t_self * self_hc) / (other_hc + self_hc), seen_min,
                  seen_max);
        new_self = equilibrium;
        new_other = equilibrium;
      }

      if (new_other >= other->overheat_temperature) ev.overheated.push_back(c.handle);
      if (other->overheat_temperature <= t_other &&
          new_other < other->overheat_temperature) {
        ev.no_longer_overheated.push_back(c.handle);
      }
      other->temperature = new_other;
      self->temperature = new_self;

      // Both sides publish immediately rather than at the end of the sweep, because a
      // building may appear in several contact groups in the same substep.
      auto publish = [&](int32_t handle, float temperature) {
        const size_t slot =
            static_cast<size_t>(handle & kHandleIndexMask);
        if (slot < ev.temperatures.size()) {
          ev.temperatures[slot].handle = handle;
          ev.temperatures[slot].temperature = temperature;
        }
      };
      publish(c.handle, new_other);
      publish(g.self, new_self);
    }
  }
}

}  // namespace oni_sim
