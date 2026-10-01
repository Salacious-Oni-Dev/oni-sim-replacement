// ConduitTemperatureManager.
//
// The contents of a pipe are not cells. They live in the game's own `ConduitFlow` grid and
// the sim never sees them move; what the sim owns is one number per conduit — the
// temperature of whatever is inside it — and one job: exchange heat between that number and
// the *building* the conduit is, then tell the game when the contents froze or boiled.
//
// So Klei's half of this file has no world, no grid and no neighbours (the network extension,
// below,
// is handed the game's own runs and adjacency by the managed side). It is a flat list of
// independent two-body problems, driven once per 200 ms from the game thread rather than from
// the sim frame, and it talks to the rest of the sim through exactly one message
// (`ModifyBuildingEnergy`) — the same message a duplicant warming a bed sends. That is the
// whole coupling: the conduit reads the building temperature the last frame published and
// pays back the energy it took as a message the next frame drains.
//
// Everything below follows the game's arithmetic exactly rather than a reasoned-out version,
// because the arithmetic is unusual in two ways that a plausible-looking rewrite gets wrong:
//
//   * The exchange is limited by *both* sides independently and then the smaller limit wins
//     (`min(limB, limC)`), rather than one clamp on the result.
//   * When the transfer would still cross the two temperatures over each other, the pair is
//     dropped onto its exact equilibrium instead. Not clamped to it — computed.
//
// Verification note: no `diffsim` scenario can reach any of this. The suite drives the cell
// grid, and conduit contents are a separate world that only the game populates. What can be
// checked offline is the two-body arithmetic against hand-worked cases and the handle
// bookkeeping against `CompactedVector`; what cannot be checked without the game is whether
// the game's own conduit flow agrees about which handle is which.

#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "../abi/sim_abi.h"
#include "../abi/sim_abi_ext.h"
#include "buildings.h"
#include "gas_mixture.h"
#include "phase_change.h"
#include "world.h"

namespace oni_sim {

// `MinSS`/`MaxSS` live in `world.h` now — this file was where they were first needed (conduit
// temperatures do go NaN in practice, a zero heat capacity divides); the definition and the
// explanation live in the one header everything includes.

// The exchange rate. Klei folds the pipe's contact area into a single constant
// rather than deriving it from the building's footprint the way cell-to-building exchange
// does, so a conduit is the same size to the thermal model no matter what building holds it.
// The tunable `ConduitContactArea` (tunables.def), as is every number in this block.
// Dt arrives in seconds and the conductivities are per-kilosecond
// (`ConduitTransferRate`).
// A conduit does not freeze at its contents' transition
// temperature, it freezes three degrees past it. The margin is what stops a pipe sitting
// exactly on the boundary from reporting a phase change every single update
// (`ConduitTransitionMargin`).
// A heat capacity at or below this is treated as "no body here"
// and the pair is skipped, and an energy transfer at or below the other is not worth a
// message (`MinConduitHeatCapacity`, `MinConduitEnergy`).
// Handles are `slot | (version << 24)`; a slot this large is a corrupt or
// released handle rather than a real one, and the entry is skipped without even writing an
// output temperature for it.
inline constexpr int32_t kMaxConduitSlot = 0x100000;

// ---------------------------------------------------------------------------------------
// NOT KLEI'S: the per-network read model, mixing and phase decisions -- the network
// extension. Everything marked as the network extension in this file belongs to it.
//
// Klei's manager knows every conduit's temperature and heat capacity but not WHERE the
// conduit is or WHICH run it belongs to: `Allocate` files the conduit's type and index on the
// managed side only. The game's `UtilityNetworkManager` already partitions every conduit layer
// into runs, and `ConduitFlow.RebuildConnections` re-allocates EVERY temperature handle of a
// type on each topology change (it frees them all in `SOAInfo.Clear` and adds them back one per
// connected conduit), so the managed side pushes `handle -> cell + network + neighbours` in one
// call right after each rebuild (`SIM_ConduitNetworkBind`) and the native side keys everything
// below off that. No union-find and no bridge semantics live here -- the partition is the
// game's `UtilityNetworkManager`'s and the adjacency is `ConduitFlow`'s own connection table.
//
// EVERYTHING BELOW IS INERT UNTIL A BIND ARRIVES. An unbound conduit is updated by exactly the
// Klei arithmetic and nothing else, which is what keeps `diffsim --conduits` bit-exact against
// Klei's own DLL: that harness never binds.

// The game's `ConduitType` numbering: None, Gas, Liquid, Solid. Solid conduits never allocate a
// temperature handle, so only the first two can be bound.
inline constexpr int32_t kConduitTypeGas = 1;
inline constexpr int32_t kConduitTypeLiquid = 2;
// `UtilityNetwork.id` is an index into the manager's own network list. The bound is there so a
// corrupt id in a bind cannot size a per-network array to gigabytes.
inline constexpr int32_t kMaxConduitNetworks = 1 << 20;
inline constexpr int32_t kKnownConduitPolicyFlags =
    ONI_CONDUIT_POLICY_MIX | ONI_CONDUIT_POLICY_PHASE | ONI_CONDUIT_POLICY_CONVECTION;
// The MIX coupling's ceiling. A conduit has at most four links, and each moves it by at most
// `k` of the gap to that neighbour, so `4k <= 1` is what keeps every mixed temperature between
// the temperatures it was mixed from. See `MixNetworks`. The tunable `MaxConduitMixFraction`
// (tunables.def), a ceiling whose default is the ABI's `ONI_CONDUIT_MIX_FRACTION_MAX`.
static_assert(kTunableStock.max_conduit_mix_fraction == ONI_CONDUIT_MIX_FRACTION_MAX,
              "tunables.def: MaxConduitMixFraction must default to the ABI ceiling");
inline constexpr int32_t kConduitNeighbourSlots = ONI_CONDUIT_NEIGHBOUR_SLOTS;
// Conduit liquid with no `sim.liquid_density` is counted as water-dense, exactly as
// OniFramework's PipeNetworkFacade.LiquidDensityOf falls back; trapped liquid with none is not
// counted at all, exactly as PipeMatterFacade.TrappedLiquidLitres skips it. Two rules, because
// the managed code this decision moved out of has two, and matching it is the whole point.
// The tunable `FallbackConduitLiquidDensityKgM3` (tunables.def).

// The CONVECTION policy's ratio helpers (`HeatExchangeRatio`, `CellHeatExchangeRatioOf`) live in
// buildings.h, because the building convection term (kSetBuildingConvection) reads the same ratio.

// The cells of the frame the GAME is holding: the `GameDataUpdate` the last `PrepareGameData`
// (or `Start`) handed back, whose three arrays the worker cannot be writing, because it fills
// the other of the two published frames (simdll.cpp's `PublishedFrame`). `Update` runs on the
// game thread while the next frame runs on the worker, so the live world is not readable from
// it; this is. Game cells, `count` of them. Null arrays mean the game holds no frame yet.
struct ConduitCellFrame {
  const uint16_t* element = nullptr;
  const float* mass = nullptr;
  const float* temperature = nullptr;
  size_t count = 0;
};

// The cell side of the CONVECTION policy, from the published dominant element and mass --
// the same reading `CellExposure` (buildings.h) makes of a cell for radiation. A gas cell by its
// pressure over the sim's cell volume, a liquid cell by its liquid volume ratio, vacuum 0.
//
// A SOLID CELL IS 1, and that is the one deliberate departure from Stationeers, whose
// `CalculateConvection` returns 0 when the grid `!CanContainAtmos`. A Stationeers pipe cannot be
// inside a wall; an ONI pipe can run through a tile, where it touches matter rather than
// convecting with air, and ONI's conduction already owns that exchange (the building's own cell
// exchange, `buildings.h`). Returning 0 there would switch off a working ONI heat exchanger
// that no Stationeers rule describes.
//
// A cell this frame does not cover is 1: the side that cannot be read is left to Klei's
// arithmetic rather than guessed as insulating.
inline float CellHeatExchangeRatio(const ElementTable& table, const ConduitCellFrame& cells,
                                   int32_t cell, const Tunables& tun) {
  if (cell < 0 || static_cast<size_t>(cell) >= cells.count) return 1.0f;
  return CellHeatExchangeRatioOf(table, cells.element[cell], cells.mass[cell],
                                 cells.temperature[cell], 1.0f, tun);
}

// `EvaporationTemperatureClampedK` used to be defined here. It moved to `world.h` when the
// cell condensation rule (`TransitionCell`, physics.h) became its second caller
// and the include graph -- conduits.h -> buildings.h -> physics.h -- put the two callers on
// opposite sides of it. Same function, same clamp; the note on why the clamp is kept rather
// than bypassed is at the definition.

// `ConduitTemperatureManager::Data`. The first 36 bytes are the game's fields; the tail
// after `high_temp` is ours. Nothing outside this file depends on the layout — the game
// never sees it.
struct ConduitTemperatureData {
  float temperature = 0.0f;              // +0x00, the contents
  float contents_conductivity = 0.0f;    // +0x04, the element's, unscaled
  float contents_heat_capacity = 0.0f;   // +0x08, mass * the element's specific heat
  int32_t structure_handle = -1;         // +0x0c, into the building temperature array
  float conduit_heat_capacity = 0.0f;    // +0x10, the pipe's own, sent by the game
  float conduit_conductivity = 0.0f;     // +0x14, likewise
  uint8_t insulated = 0;                 // +0x18, `def.ThermalConductivity < 1`
  float low_temp = 0.0f;                 // +0x1c, 0 when the element has no low transition
  float high_temp = 0.0f;                // +0x20, FLT_MAX when it has no high transition

  // ---- Not Klei's: the network extension. ----
  //
  // Klei's `Add` and `Set` are both handed the contents' mass and element and keep only
  // `mass * specificHeat`, so the manager could say how much heat a pipe held but not what
  // was in it. These two are the other half of that product, from the same arguments.
  float contents_mass = 0.0f;             // kg, as last sent by `Add` / `Set`
  uint16_t element_idx = 0;               // ElementTable index of the contents
  // The run this conduit belongs to, from the most recent `Bind` of its type. `conduit_type`
  // 0 means unbound, and an unbound entry is invisible to every aggregate and every policy.
  uint8_t conduit_type = 0;
  int32_t cell = -1;                      // game cell
  int32_t network = -1;                   // `UtilityNetwork.id`
  // The temperature handles of the conduits this one is piped to -- left, right, up, down, -1
  // for none -- from the same bind. What the MIX policy exchanges heat along.
  int32_t neighbours[kConduitNeighbourSlots] = {-1, -1, -1, -1};
};

// What `ConduitTemperatureManager_Update` hands back. The game reads it through a pointer
// that stays valid until the next call, so the three arrays are members rather than locals.
#pragma pack(push, 4)
struct ConduitTemperatureUpdateData {
  int32_t numEntries = 0;
  float* temperatures = nullptr;
  int32_t numFrozenHandles = 0;
  int32_t* frozenHandles = nullptr;
  int32_t numMeltedHandles = 0;
  int32_t* meltedHandles = nullptr;
};
#pragma pack(pop)

// The energy the contents took from the building has to go back as a message rather than as
// a direct write, because `Update` runs on the game thread between sim frames while the
// building temperatures belong to the sim. Klei calls `SimFrameManager::HandleMessage`,
// which enqueues; so does this, through the caller's sink.
using ConduitEnergySink = void (*)(void* ctx, const ModifyBuildingEnergyMessage& msg);

class ConduitTemperatures {
 public:
  // The element is sent as a *hash*, not an index, and is resolved here once —
  // the conduit keeps the element's numbers, not its identity, so an element table reload
  // does not reach conduits already registered.
  int32_t Add(const ElementTable& table, float temperature, float mass, int32_t element_hash,
              int32_t structure_handle, float conduit_heat_capacity,
              float conduit_conductivity, bool insulated) {
    const uint16_t element_idx = table.IndexOfHash(element_hash);
    const Element& e = table.At(element_idx);
    ConduitTemperatureData d;
    // A temperature past the sim's ceiling is refused and replaced with the
    // element's default rather than clamped, so a corrupt save loads cold instead of
    // exploding. Klei logs it; there is nothing to log to here.
    d.temperature =
        temperature > g_tunables.max_temperature ? e.defaultValues.temperature : temperature;
    d.contents_conductivity = e.thermalConductivity;
    d.contents_heat_capacity = mass * e.specificHeatCapacity;
    d.structure_handle = structure_handle;
    d.conduit_heat_capacity = conduit_heat_capacity;
    d.conduit_conductivity = conduit_conductivity;
    d.insulated = insulated ? 1 : 0;
    SetTransitions(&d, e);
    d.contents_mass = mass;          // ours
    d.element_idx = element_idx;     // ours
    aggregates_dirty_.store(true, std::memory_order_relaxed);
    return vec_.Add(d);
  }

  // Only the four contents fields move; the conduit's own heat capacity and
  // conductivity were fixed when the pipe was built and are deliberately not re-sent.
  void Set(const ElementTable& table, int32_t handle, float temperature, float mass,
           int32_t element_hash) {
    ConduitTemperatureData* d = vec_.Get(handle);
    if (!d) return;
    const uint16_t element_idx = table.IndexOfHash(element_hash);
    const Element& e = table.At(element_idx);
    d->temperature =
        temperature > g_tunables.max_temperature ? e.defaultValues.temperature : temperature;
    d->contents_heat_capacity = mass * e.specificHeatCapacity;
    d->contents_conductivity = e.thermalConductivity;
    SetTransitions(d, e);
    // `Set` is called from the game's job threads during `ConduitFlow.EndFrame`, one
    // handle per call and never two calls on one handle, so the entry writes do not race each
    // other. The dirty flag is the one piece of shared state they touch, hence the atomic.
    d->contents_mass = mass;
    d->element_idx = element_idx;
    aggregates_dirty_.store(true, std::memory_order_relaxed);
  }

  // Removal does not free the slot. It neuters the entry — a negative conduit
  // heat capacity fails the `MinConduitHeatCapacity` test in `Update`, so the pair is
  // skipped from the next update on — and queues the handle for release a frame later.
  //
  // The delay is not caution about threads. The game holds handles across a frame boundary
  // and would otherwise be able to see a slot recycled underneath a handle it is still
  // carrying; the version byte would catch that, but the temperature array it indexes by
  // slot would already have the new conduit's value in it.
  void Remove(int32_t handle) {
    if (ConduitTemperatureData* d = vec_.Get(handle)) {
      d->conduit_heat_capacity = -1.0f;
      d->conduit_conductivity = -1.0f;
      d->structure_handle = -1;
      // A removed conduit leaves its run at once rather than a frame later with its
      // slot, so no aggregate ever counts a pipe the game has already let go of.
      d->conduit_type = 0;
      d->cell = -1;
      d->network = -1;
      for (int32_t& n : d->neighbours) n = -1;
    }
    aggregates_dirty_.store(true, std::memory_order_relaxed);
    release_queue_[release_parity_].push_back(handle);
  }

  void Clear() {
    vec_.Clear();
    out_temperatures_.clear();
    frozen_.clear();
    melted_.clear();
    release_queue_[0].clear();
    release_queue_[1].clear();
    release_parity_ = 0;
    update_ = ConduitTemperatureUpdateData{};
    for (TypeBinding& t : types_) t = TypeBinding{};
    deferred_.clear();
    for (auto& store : trapped_) store.clear();
    proposals_.clear();
    convecting_structures_.clear();
    aggregates_dirty_.store(true, std::memory_order_relaxed);
  }

  // abi/sim_abi_ext.h's kSetBuildingConvection, the conduit-side half. A building that convects
  // with its cells through the building term owns that cell ratio there, so its own conduit must
  // not apply it a second time on the contents <-> pipe leg -- and in vacuum the cell ratio is 0,
  // which would leave a radiator's contents unable to warm the body that is supposed to radiate
  // them away. Recorded by the FULL handle rather than its index, so a slot reused by an ordinary
  // pipe does not inherit a demolished radiator's flag.
  //
  // Game thread, like every other entry point on this class: the send records it (simdll.cpp's
  // SIM_HandleMessage) and `Update` reads it, both outside the worker's frame.
  void SetStructureConvection(int32_t structure_handle, bool on) {
    if (structure_handle < 0) return;
    if (on) {
      convecting_structures_.insert(structure_handle);
    } else {
      convecting_structures_.erase(structure_handle);
    }
  }

  bool StructureConvects(int32_t structure_handle) const {
    return structure_handle >= 0 &&
           convecting_structures_.find(structure_handle) != convecting_structures_.end();
  }

  // Called once per sim frame from `Sim::Main`. The parity flips
  // first and the buffer that comes up is the one filled *before* the previous flip, which
  // is where the one frame of delay comes from.
  //
  // Klei calls it on the sim thread; ours calls it on the GAME thread, from `PrepareGameData`
  // just before each frame starts (`ReleaseConduitHandlesForNextFrame` in simdll.cpp). This
  // class is not synchronised and every other entry point is a game-thread export, so the
  // release is the one call that must not run on the worker: it compacts `vec_` and would
  // move entries underneath an `Update` or a `Set` in flight.
  void ReleaseQueuedHandles() {
    release_parity_ = (release_parity_ + 1) & 1;
    std::vector<int32_t>& due = release_queue_[release_parity_];
    for (int32_t handle : due) vec_.Remove(handle);
    due.clear();
  }

  //
  // `cells` is the CONVECTION policy's only addition to the signature, and nothing reads it
  // unless a run carries that policy; every caller that omits it gets Klei's update.
  const ConduitTemperatureUpdateData* Update(const ElementTable& table, float dt,
                                             const BuildingTemperatureInfo* building_temperatures,
                                             ConduitEnergySink sink, void* sink_ctx,
                                             const ConduitCellFrame& cells = ConduitCellFrame{}) {
    // Indexed by handle slot, not by dense position, and only ever grown: a conduit that is
    // skipped this update keeps the temperature it had rather than reading as absent.
    out_temperatures_.resize(vec_.SlotCount(), 0.0f);
    const Tunables tun = g_tunables;  // tunables.h, THE ONE RULE
    frozen_.clear();
    melted_.clear();
    deferred_.clear();
    proposals_.clear();

    std::vector<ConduitTemperatureData>& data = vec_.Data();
    const std::vector<int32_t>& handles = vec_.Handles();

    // CONVECTION policy. Its run side is one number per run, read before any conduit moves --
    // the state the update starts from, as Stationeers reads its network's atmosphere once per
    // tick. Same real-update condition as the mix below.
    const bool convect = dt > 0.0f && building_temperatures != nullptr &&
                         cells.element != nullptr && cells.mass != nullptr &&
                         cells.temperature != nullptr &&
                         (types_[kConduitTypeGas].any_convection ||
                          types_[kConduitTypeLiquid].any_convection);
    if (convect) PrepareConvection(table);

    for (size_t i = 0; i < data.size(); ++i) {
      ConduitTemperatureData& d = data[i];
      const size_t out = static_cast<size_t>(handles[i] & kHandleIndexMask);
      if (out >= out_temperatures_.size()) continue;

      // No building temperatures at all means the game has not published a frame yet. Klei
      // still walks the list and copies every temperature straight through, so the array the
      // game reads is populated on the very first update rather than left at zero.
      if (!building_temperatures) {
        out_temperatures_[out] = d.temperature;
        continue;
      }

      const int32_t slot = d.structure_handle & kHandleIndexMask;
      // A dead or corrupt structure handle writes *nothing*, not even the unchanged
      // temperature — the one path in this loop that leaves the output slot alone.
      if (slot >= kMaxConduitSlot) continue;

      // Either body missing, or a building the sim has never warmed, and the pair is skipped
      // with its temperature copied through unchanged.
      //
      // These are spelled as `constant >= field` rather than `field <= constant` on purpose:
      // that is the direction the game compares in, and it decides what a NaN heat capacity
      // does. An unordered compare does *not* skip the pair —
      // the reverse of what the natural spelling would do.
      if (tun.min_conduit_heat_capacity >= d.contents_heat_capacity) {
        out_temperatures_[out] = d.temperature;
        continue;
      }
      if (tun.min_conduit_heat_capacity >= d.conduit_heat_capacity) {
        out_temperatures_[out] = d.temperature;
        continue;
      }
      const float building_t = building_temperatures[slot].temperature;
      // An unordered compare skips, so a NaN building temperature skips. Written as the
      // negation of `>` to keep that.
      if (!(building_t > 0.0f)) {
        out_temperatures_[out] = d.temperature;
        continue;
      }

      const float contents_t = d.temperature;
      const float hc_contents = d.contents_heat_capacity;
      const float hc_conduit = d.conduit_heat_capacity;

      // Insulated pipes take the *minimum* of the two conductivities, plain
      // ones take the mean. Same shape as cell insulation in `physics.h`: below the
      // threshold the formula changes shape rather than scale, so an insulated pipe is held
      // to its weaker side instead of being slowed by a factor.
      const float k = d.insulated
                          ? MinSS(d.contents_conductivity, d.conduit_conductivity)
                          : (d.contents_conductivity + d.conduit_conductivity) * 0.5f;
      const float rate = k * (contents_t - building_t) * tun.conduit_contact_area;
      float energy = rate * dt * tun.conduit_transfer_rate;
      // CONVECTION policy. Scales the proposal only: the sign still comes from `rate`, and the
      // limits, the equilibrium rescue and the building's message below are Klei's, so a ratio
      // of 1 is Klei's update exactly and a ratio of 0 moves nothing and bills nothing.
      if (convect && ConvectsIn(d)) {
        // The cell ratio is the building convection term's when the pipe's own building has one
        // (SetStructureConvection): one ratio, on one leg. The run's ratio applies either way --
        // it is Stationeers' internal-atmosphere side, which no building term replaces.
        energy *= RunConvectionRatio(d) * (StructureConvects(d.structure_handle)
                                               ? 1.0f
                                               : CellHeatExchangeRatio(table, cells, d.cell, tun));
      }

      const float lo = MinSS(contents_t, building_t);
      const float hi = MaxSS(contents_t, building_t);
      const float inv_contents = 1.0f / hc_contents;
      const float inv_conduit = 1.0f / hc_conduit;

      // How much energy each side could give up or take before it would pass the other one.
      // Both are computed from the *same* proposed transfer and the smaller wins, which is
      // what makes a small pipe against a huge building move by the pipe's limit and not by
      // some average of the two.
      float reach_contents = contents_t - inv_contents * energy;
      reach_contents = MinSS(reach_contents, hi);
      reach_contents = MaxSS(reach_contents, lo);
      const float limit_contents = std::fabs(reach_contents - contents_t) * hc_contents;

      float reach_conduit = building_t + inv_conduit * energy;
      reach_conduit = MinSS(reach_conduit, hi);
      reach_conduit = MaxSS(reach_conduit, lo);
      const float limit_conduit = std::fabs(reach_conduit - building_t) * hc_conduit;

      // The sign comes from the *unlimited* rate, not from either limit, because both limits
      // went through `fabs`. NaN takes the negative branch, as in the game.
      const float sign = rate >= 0.0f ? 1.0f : -1.0f;
      float transferred = MinSS(limit_conduit, limit_contents) * sign;
      const float delta = -transferred;

      float next = MaxSS(contents_t + inv_contents * delta, 0.0f);
      const float next_building = MaxSS(building_t - inv_conduit * delta, 0.0f);

      // If the two ends came out on opposite sides of where they started, the
      // limits above did not hold and the honest answer is the temperature the pair would
      // reach with all the energy shared. Klei computes it rather than clamping to one side,
      // so an overshooting pair lands on its equilibrium in one update instead of ringing.
      // An unordered compare runs the equilibrium branch, so a NaN product runs it rather
      // than falling through it.
      if (!((next - next_building) * (contents_t - building_t) >= 0.0f)) {
        const float inv_total = 1.0f / (hc_contents + hc_conduit);
        next = inv_total * hc_contents * contents_t + inv_total * hc_conduit * building_t;
      }

      // `_fdtest`, testing for NaN only — an infinity is allowed through. A
      // NaN result is refused rather than corrected: the conduit keeps the temperature it
      // had and, because the transfer is zeroed too, no energy is billed to the building.
      if (next != next) {
        next = d.temperature;
        transferred = 0.0f;
      } else {
        d.temperature = next;
      }
      out_temperatures_[out] = next;

      // The building is paid in energy, not in temperature, and the bounds ride along so the
      // building's own handler can refuse a result outside them. `min`/`max` are the two
      // starting temperatures, so the building can never be pushed past where the contents
      // began — the same envelope the limits above enforced on this side.
      if (sink && std::fabs(transferred) > tun.min_conduit_energy) {
        ModifyBuildingEnergyMessage msg{};
        msg.handle = d.structure_handle;
        msg.deltaKJ = (contents_t - next) * hc_contents;
        msg.minTemperature = lo;
        msg.maxTemperature = hi;
        sink(sink_ctx, msg);
      }

      // A conduit in a run that mixes is checked for a phase transition AFTER the
      // run has mixed, against the temperature the game will actually be handed, rather than
      // here against one the mix is about to overwrite. Nothing else in this loop differs for
      // it: the exchange with its own building, the energy billed and the output written are
      // all the Klei arithmetic above.
      if (MixesIn(d)) {
        deferred_.push_back(static_cast<int32_t>(i));
        continue;
      }

      // Reported as handles, with the version byte still on, because the game looks the
      // conduit up in its own table and wants a stale handle to fail rather than alias.
      if (next < d.low_temp - tun.conduit_transition_margin) {
        frozen_.push_back(handles[i]);
      } else if (next > d.high_temp + tun.conduit_transition_margin) {
        melted_.push_back(handles[i]);
      }
    }

    // The mix runs only on a real update: `dt == 0` is `RebuildConnections`
    // publishing temperatures for handles it has just allocated, and no building temperatures
    // means the game has not published a frame yet -- the same case Klei copies straight
    // through above.
    if (dt > 0.0f && building_temperatures != nullptr) MixNetworks();
    for (const int32_t i : deferred_) {
      const ConduitTemperatureData& d = data[static_cast<size_t>(i)];
      const float t = d.temperature;
      if (t < d.low_temp - tun.conduit_transition_margin) {
        frozen_.push_back(handles[static_cast<size_t>(i)]);
      } else if (t > d.high_temp + tun.conduit_transition_margin) {
        melted_.push_back(handles[static_cast<size_t>(i)]);
      }
    }
    aggregates_dirty_.store(true, std::memory_order_relaxed);
    // Decided against the temperatures the game is about to be handed -- after the
    // exchange, after the mix -- and on the same real-update condition as the mix.
    if (dt > 0.0f && building_temperatures != nullptr &&
        (types_[kConduitTypeGas].any_phase || types_[kConduitTypeLiquid].any_phase)) {
      ProposePhaseChanges(table, dt);
    }

    update_.numEntries = static_cast<int32_t>(out_temperatures_.size());
    update_.temperatures = out_temperatures_.empty() ? nullptr : out_temperatures_.data();
    update_.numFrozenHandles = static_cast<int32_t>(frozen_.size());
    update_.frozenHandles = frozen_.empty() ? nullptr : frozen_.data();
    update_.numMeltedHandles = static_cast<int32_t>(melted_.size());
    update_.meltedHandles = melted_.empty() ? nullptr : melted_.data();
    return &update_;
  }

  // ---------------------------------------------------------------- network extension, public
  //
  // All of these run on the game thread, like every other entry point of this class (see
  // `ReleaseQueuedHandles`), and none of them changes the Klei arithmetic for an unbound
  // conduit.

  // Replaces every binding of one conduit type with the one given: entry `i` puts
  // `handles[i]` in network `network_ids[i]` at `cells[i]`, piped to the conduits whose
  // handles are `neighbour_handles[4i .. 4i+3]` (left, right, up, down; -1 for none). Called by
  // the managed side once per `ConduitFlow.onConduitsRebuilt`, which is the moment the handles
  // it names were allocated.
  //
  // The neighbours are stored as given and checked when used, not here: a link counts only
  // while both ends are bound to the same run of the same type (see `MixNetworks`). That is
  // also what makes a neighbour that is later removed drop out without a rebind -- `Remove`
  // unbinds it, and the game rebinds on the rebuild the removal triggers anyway.
  //
  // Returns how many handles were bound, which is less than `count` for a handle this manager
  // does not hold (stale, or already removed) or a network id out of range; both are skipped
  // rather than refusing the whole bind, because the rest of the run is still right. Returns
  // -1, and changes nothing, for an unknown type, a negative count, a null array with a
  // non-zero count, or a volume that is not a positive finite number.
  //
  // Every policy of the type is dropped, because network ids are re-assigned on each rebuild
  // and a policy keyed to the old numbering would land on whichever run inherited the number.
  int32_t Bind(int32_t conduit_type, const int32_t* handles, const int32_t* cells,
               const int32_t* network_ids, const int32_t* neighbour_handles, int32_t count,
               float volume_per_conduit_m3, float max_mass_per_conduit_kg) {
    if (conduit_type != kConduitTypeGas && conduit_type != kConduitTypeLiquid) return -1;
    if (count < 0) return -1;
    if (count > 0 && (!handles || !cells || !network_ids || !neighbour_handles)) return -1;
    if (!(volume_per_conduit_m3 > 0.0f) || !std::isfinite(volume_per_conduit_m3)) return -1;
    if (!(max_mass_per_conduit_kg > 0.0f) || !std::isfinite(max_mass_per_conduit_kg)) return -1;

    for (ConduitTemperatureData& d : vec_.Data()) {
      if (d.conduit_type != conduit_type) continue;
      d.conduit_type = 0;
      d.cell = -1;
      d.network = -1;
      for (int32_t& n : d.neighbours) n = -1;
    }
    int32_t bound = 0;
    int32_t max_network = -1;
    for (int32_t i = 0; i < count; ++i) {
      const int32_t network = network_ids[i];
      if (network < 0 || network >= kMaxConduitNetworks) continue;
      ConduitTemperatureData* d = vec_.Get(handles[i]);
      // `Remove` neuters with a negative conduit heat capacity and the slot stays valid until
      // the next frame's release, so a removed handle still resolves here. It is not a pipe.
      if (!d || d->conduit_heat_capacity < 0.0f) continue;
      d->conduit_type = static_cast<uint8_t>(conduit_type);
      d->cell = cells[i];
      d->network = network;
      const int32_t* from = neighbour_handles + static_cast<size_t>(i) * kConduitNeighbourSlots;
      for (int32_t s = 0; s < kConduitNeighbourSlots; ++s) d->neighbours[s] = from[s];
      ++bound;
      if (network > max_network) max_network = network;
    }

    TypeBinding& t = types_[conduit_type];
    ++t.generation;
    t.network_count = max_network + 1;
    t.volume_per_conduit_m3 = volume_per_conduit_m3;
    t.max_mass_per_conduit_kg = max_mass_per_conduit_kg;
    t.policy.assign(static_cast<size_t>(t.network_count), NetworkPolicy{});
    t.any_mix = false;
    t.any_phase = false;
    t.any_convection = false;
    t.convection_ratio.clear();
    aggregates_dirty_.store(true, std::memory_order_relaxed);
    return bound;
  }

  // Sets what the sim does to one bound run of its own accord. `flags` is a mask of
  // `ONI_CONDUIT_POLICY_*`; 0 turns everything off. Refused (false, nothing changed) for an
  // unknown type, a network outside the current bind, a flag bit this build does not know --
  // so a caller asking for a behaviour this DLL lacks finds out instead of silently getting
  // none -- or a parameter out of range for a flag that is set: MIX needs a coupling in
  // (0, MaxConduitMixFraction], PHASE a positive finite rate and a non-negative finite
  // remainder.
  bool SetPolicy(int32_t conduit_type, int32_t network, int32_t flags, float mix_fraction,
                 float phase_rate_per_second, float phase_min_remainder_kg) {
    if (conduit_type != kConduitTypeGas && conduit_type != kConduitTypeLiquid) return false;
    TypeBinding& t = types_[conduit_type];
    if (network < 0 || network >= t.network_count) return false;
    if ((flags & ~kKnownConduitPolicyFlags) != 0) return false;
    const bool mix = (flags & ONI_CONDUIT_POLICY_MIX) != 0;
    const bool phase = (flags & ONI_CONDUIT_POLICY_PHASE) != 0;
    if (mix && !(mix_fraction > 0.0f && mix_fraction <= g_tunables.max_conduit_mix_fraction)) {
      return false;
    }
    if (phase && (!(phase_rate_per_second > 0.0f) || !std::isfinite(phase_rate_per_second) ||
                  !(phase_min_remainder_kg >= 0.0f) || !std::isfinite(phase_min_remainder_kg))) {
      return false;
    }
    NetworkPolicy& policy = t.policy[static_cast<size_t>(network)];
    policy.flags = flags;
    policy.mix_fraction = mix ? mix_fraction : 0.0f;
    policy.phase_rate_per_second = phase ? phase_rate_per_second : 0.0f;
    policy.phase_min_remainder_kg = phase ? phase_min_remainder_kg : 0.0f;
    t.any_mix = false;
    t.any_phase = false;
    t.any_convection = false;
    for (const NetworkPolicy& other : t.policy) {
      t.any_mix = t.any_mix || (other.flags & ONI_CONDUIT_POLICY_MIX) != 0;
      t.any_phase = t.any_phase || (other.flags & ONI_CONDUIT_POLICY_PHASE) != 0;
      t.any_convection = t.any_convection || (other.flags & ONI_CONDUIT_POLICY_CONVECTION) != 0;
    }
    aggregates_dirty_.store(true, std::memory_order_relaxed);
    return true;
  }

  // The side-car mirror. One entry per cell per type, replaced outright; a mass at or below
  // zero removes it. Keyed by cell rather than by handle so that it survives every rebind --
  // the managed store it mirrors is keyed the same way, for the same reason.
  bool TrappedSet(const ElementTable& table, int32_t conduit_type, int32_t cell,
                  int32_t element_idx, float mass_kg, float temperature_k) {
    if (conduit_type != kConduitTypeGas && conduit_type != kConduitTypeLiquid) return false;
    if (cell < 0 || !std::isfinite(mass_kg) || !std::isfinite(temperature_k)) return false;
    std::unordered_map<int32_t, Trapped>& store = trapped_[conduit_type];
    if (!(mass_kg > 0.0f)) {
      store.erase(cell);
    } else {
      if (element_idx < 0 || element_idx >= table.Count()) return false;
      store[cell] = Trapped{static_cast<uint16_t>(element_idx), mass_kg, temperature_k};
    }
    aggregates_dirty_.store(true, std::memory_order_relaxed);
    return true;
  }

  int32_t TrappedClear(int32_t conduit_type) {
    if (conduit_type != kConduitTypeGas && conduit_type != kConduitTypeLiquid) return -1;
    const int32_t n = static_cast<int32_t>(trapped_[conduit_type].size());
    trapped_[conduit_type].clear();
    aggregates_dirty_.store(true, std::memory_order_relaxed);
    return n;
  }

  // The decisions of the last update. See SIM_ConduitPhaseProposals.
  const std::vector<OniConduitPhaseProposal>& PhaseProposals() const { return proposals_; }

  // The whole of one bound run, summed. Returns false for an unknown type or a network outside
  // the current bind, and in that case writes a zeroed struct with `networkId` = -1 (the
  // caller's buffer is always written; see the ABI comment). The sums are rebuilt at most once
  // per change to the conduit list, for every run of both types in one pass, so asking about
  // many runs between two changes costs one walk.
  bool Aggregate(const ElementTable& table, int32_t conduit_type, int32_t network,
                 OniConduitNetworkAggregate* out) {
    if (!out) return false;
    memset(out, 0, sizeof(*out));
    out->networkId = -1;
    if (conduit_type != kConduitTypeGas && conduit_type != kConduitTypeLiquid) return false;
    const TypeBinding& t = types_[conduit_type];
    if (network < 0 || network >= t.network_count) return false;
    RefreshAggregates(table);

    const NetworkAccum& a = t.accum[static_cast<size_t>(network)];
    const NetworkPolicy& policy = t.policy[static_cast<size_t>(network)];
    out->networkId = network;
    out->conduitType = conduit_type;
    out->generation = t.generation;
    out->policyFlags = policy.flags;
    out->conduitCount = a.conduits;
    out->filledCount = a.filled;
    out->speciesCount = a.species_count;
    out->speciesOverflow = static_cast<int32_t>(a.overflow.size());
    out->mixFraction = policy.mix_fraction;
    out->volumeM3 = static_cast<float>(static_cast<double>(a.conduits) * t.volume_per_conduit_m3);
    out->totalMassKg = static_cast<float>(a.mass);
    out->totalMoles = static_cast<float>(a.moles);
    out->heatCapacityKJPerK = static_cast<float>(a.heat_capacity);
    out->temperatureK = a.heat_capacity > 0.0 ? static_cast<float>(a.energy / a.heat_capacity)
                                              : 0.0f;
    out->massWeightedTemperatureK =
        a.mass > 0.0 ? static_cast<float>(a.mass_temperature / a.mass) : 0.0f;
    // Gas only, and through the same two functions `SIM_ComputeGasPressure` uses, so a run's
    // pressure and a managed caller's pressure for the same contents are one formula. A
    // liquid run's pressure is its headspace, which depends on gas the conduit itself cannot
    // hold (OniFramework's PipeMatterFacade); it is 0 here rather than a guess.
    if (conduit_type == kConduitTypeGas && a.moles > 0.0 && out->volumeM3 > 0.0f &&
        out->temperatureK > 0.0f) {
      out->pressurePa = gas::PressureFromMoles(out->totalMoles, out->temperatureK, out->volumeM3);
    }
    for (int32_t k = 0; k < a.species_count; ++k) {
      out->species[k] = a.species[k];
      out->massBySpeciesKg[k] = static_cast<float>(a.species_mass[k]);
    }
    out->trappedGasKg = static_cast<float>(a.trapped_gas_mass);
    out->trappedLiquidKg = static_cast<float>(a.trapped_liquid_mass);
    out->trappedSolidKg = static_cast<float>(a.trapped_solid_mass);
    out->liquidVolumeM3 = static_cast<float>(a.liquid_volume_m3);
    out->phaseRatePerSecond = policy.phase_rate_per_second;
    out->linkCount = a.links;
    if (conduit_type == kConduitTypeLiquid) out->headspacePressurePa = HeadspacePressurePa(t, a);
    return true;
  }

  // For tests and diagnostics: the binding one handle currently carries. False for a handle
  // this manager does not hold.
  bool BindingOf(int32_t handle, int32_t* conduit_type, int32_t* cell, int32_t* network) const {
    const ConduitTemperatureData* d = vec_.Get(handle);
    if (!d) return false;
    if (conduit_type) *conduit_type = d->conduit_type;
    if (cell) *cell = d->cell;
    if (network) *network = d->network;
    return true;
  }

  // ------------------------------------------------------------- end of network extension, public

  // Read-only access to the registered conduits, for the energy ledger
  // (SIM_DebugEnergyLedger). Pipe contents hold real thermal energy that is not in the
  // grid and not in a building temperature, so a ledger that ignored them would read a
  // pipe filling from a vent as energy vanishing.
  const CompactedVector<ConduitTemperatureData>& Registered() const { return vec_; }

 private:
  // An element with no transition on a side cannot leave on
  // that side, and the sentinels say so in the units the test uses: 0 K, which nothing
  // reaches, and FLT_MAX, which nothing exceeds. Storing the transition temperatures rather
  // than the element index is what makes the update loop free of table lookups.
  static void SetTransitions(ConduitTemperatureData* d, const Element& e) {
    d->low_temp = e.lowTempTransitionIdx == 0xFFFF ? 0.0f : e.lowTemp;
    d->high_temp = e.highTempTransitionIdx == 0xFFFF ? 3.4028234663852886e+38f : e.highTemp;
  }

  CompactedVector<ConduitTemperatureData> vec_;
  std::vector<float> out_temperatures_;
  std::vector<int32_t> frozen_;
  std::vector<int32_t> melted_;
  std::vector<int32_t> release_queue_[2];
  int32_t release_parity_ = 0;
  ConduitTemperatureUpdateData update_{};

  // ---------------------------------------------------------------- network extension, private

  struct NetworkPolicy {
    int32_t flags = 0;
    float mix_fraction = 0.0f;
    float phase_rate_per_second = 0.0f;
    float phase_min_remainder_kg = 0.0f;
  };

  struct Trapped {
    uint16_t element_idx = 0;
    float mass = 0.0f;
    float temperature = 0.0f;
  };

  // Sums in double: a run is thousands of conduits and the answer is compared against a
  // managed walk of the same contents in a different order.
  struct NetworkAccum {
    int32_t conduits = 0;
    int32_t filled = 0;
    int32_t links = 0;
    int32_t species_count = 0;
    double mass = 0.0;
    double moles = 0.0;
    double heat_capacity = 0.0;
    double energy = 0.0;
    double mass_temperature = 0.0;
    uint16_t species[ONI_CONDUIT_NET_MAX_SPECIES] = {};
    double species_mass[ONI_CONDUIT_NET_MAX_SPECIES] = {};
    // Distinct species that found no slot. Empty for every run a vanilla conduit can build:
    // one conduit holds one element, and a run holding more than eight at once is hand-made.
    std::vector<uint16_t> overflow;
    // The side-car's matter standing in this run's conduits, by phase, and what the headspace
    // pressure of a liquid run is built from.
    double trapped_gas_mass = 0.0;
    double trapped_liquid_mass = 0.0;
    double trapped_solid_mass = 0.0;
    double trapped_gas_moles = 0.0;
    double trapped_gas_mass_temperature = 0.0;
    double liquid_volume_m3 = 0.0;
  };

  // One pipe-to-pipe link of a mixing run, as dense positions in `vec_`, found once per update.
  struct MixLink {
    size_t a = 0;
    size_t b = 0;
  };

  struct TypeBinding {
    int32_t generation = 0;
    int32_t network_count = 0;
    float volume_per_conduit_m3 = 0.0f;
    float max_mass_per_conduit_kg = 0.0f;
    bool any_mix = false;
    bool any_phase = false;
    bool any_convection = false;
    std::vector<NetworkPolicy> policy;
    std::vector<NetworkAccum> accum;
    // CONVECTION policy: each run's own HeatExchangeRatio(), refilled at the top of every
    // update that convects. 1 for a run without the policy, which nothing reads.
    std::vector<float> convection_ratio;
  };

  bool ConvectsIn(const ConduitTemperatureData& d) const {
    if (d.conduit_type != kConduitTypeGas && d.conduit_type != kConduitTypeLiquid) return false;
    const TypeBinding& t = types_[d.conduit_type];
    if (!t.any_convection || d.network < 0 || d.network >= t.network_count) return false;
    return (t.policy[static_cast<size_t>(d.network)].flags & ONI_CONDUIT_POLICY_CONVECTION) != 0;
  }

  float RunConvectionRatio(const ConduitTemperatureData& d) const {
    const TypeBinding& t = types_[d.conduit_type];
    const size_t n = static_cast<size_t>(d.network);
    return n < t.convection_ratio.size() ? t.convection_ratio[n] : 1.0f;
  }

  // The run side of the CONVECTION policy: Stationeers' network `Atmosphere.
  // HeatExchangeRatio()`, from the same sums the aggregate and the PHASE policy read. A gas
  // run's pressure is its contents' (RunPressurePa); a liquid run's is its headspace's, which
  // only matters once its liquid falls under a thousandth of the run's volume. Liquid standing
  // in either kind of run counts toward the liquid ratio.
  void PrepareConvection(const ElementTable& table) {
    const Tunables tun = g_tunables;  // tunables.h, THE ONE RULE
    RefreshAggregates(table);
    for (int32_t type = kConduitTypeGas; type <= kConduitTypeLiquid; ++type) {
      TypeBinding& t = types_[type];
      t.convection_ratio.assign(static_cast<size_t>(t.network_count), 1.0f);
      if (!t.any_convection) continue;
      for (int32_t n = 0; n < t.network_count; ++n) {
        const size_t idx = static_cast<size_t>(n);
        if ((t.policy[idx].flags & ONI_CONDUIT_POLICY_CONVECTION) == 0) continue;
        if (idx >= t.accum.size()) continue;
        const NetworkAccum& a = t.accum[idx];
        const double capacity = static_cast<double>(a.conduits) * t.volume_per_conduit_m3;
        const float pressure =
            type == kConduitTypeGas ? RunPressurePa(t, a) : HeadspacePressurePa(t, a);
        const float liquid_ratio =
            capacity > 0.0 ? static_cast<float>(a.liquid_volume_m3 / capacity) : 0.0f;
        t.convection_ratio[idx] = HeatExchangeRatio(pressure, liquid_ratio, tun);
      }
    }
  }

  bool MixesIn(const ConduitTemperatureData& d) const {
    if (d.conduit_type != kConduitTypeGas && d.conduit_type != kConduitTypeLiquid) return false;
    const TypeBinding& t = types_[d.conduit_type];
    if (!t.any_mix || d.network < 0 || d.network >= t.network_count) return false;
    return (t.policy[static_cast<size_t>(d.network)].flags & ONI_CONDUIT_POLICY_MIX) != 0;
  }

  // A conduit takes part in its run's mix when it holds something that can carry heat, is a
  // live pipe, and has a temperature to share. An empty conduit is skipped rather than
  // averaged in at zero heat capacity -- which would change nothing -- so that it keeps the
  // temperature Klei's copy-through gave it instead of being handed the run's.
  static bool MixParticipant(const ConduitTemperatureData& d, float min_heat_capacity) {
    return min_heat_capacity < d.contents_heat_capacity &&
           min_heat_capacity < d.conduit_heat_capacity && std::isfinite(d.temperature);
  }

  static bool ListsNeighbour(const ConduitTemperatureData& d, int32_t handle) {
    for (const int32_t n : d.neighbours) {
      if (n == handle) return true;
    }
    return false;
  }

  // The one rule for which end owns a link, shared by the mix and the aggregate's `linkCount`
  // so the count reports exactly the edges the mix uses. Neighbour slot `s` of the conduit at
  // dense position `i` is a link it owns when it names a live conduit bound to the same run of
  // the same type -- not itself, not a handle an earlier slot already named -- and, when that
  // conduit names it back, sits at a higher dense position (the lower end owns a link both ends
  // list). Returns the partner, or null. Says nothing about contents: the mix further requires
  // both ends to be participants, which is symmetric, so ownership still resolves each link to
  // exactly one end.
  const ConduitTemperatureData* OwnedLink(size_t i, int32_t s) const {
    const std::vector<ConduitTemperatureData>& data = vec_.Data();
    const ConduitTemperatureData& a = data[i];
    const int32_t self = vec_.Handles()[i];
    const int32_t n = a.neighbours[s];
    if (n < 0 || n == self) return nullptr;
    for (int32_t r = 0; r < s; ++r) {
      if (a.neighbours[r] == n) return nullptr;
    }
    const ConduitTemperatureData* b = vec_.Get(n);
    if (!b || b->conduit_type != a.conduit_type || b->network != a.network) return nullptr;
    const size_t j = static_cast<size_t>(b - data.data());
    if (j < i && ListsNeighbour(*b, self)) return nullptr;
    return b;
  }

  // AXIAL MIXING: heat moves between conduits that are piped to each other, and only those.
  //
  // WHY NOT STATIONEERS' WHOLE-NETWORK MIX. A Stationeers pipe network is one atmosphere with
  // one temperature, and that works there because the devices in a refrigeration loop split its two sides into
  // separate networks. ONI runs are split the same way -- pumps and valves have ports, and a
  // port ends a run -- but a player's run is free to cross rooms with nothing in between, and
  // a refrigeration loop's liquid line can be ONE network that crosses the hot room as a
  // sub-cool leg, climbs through rock and crosses the cold room. Without the sub-cool leg such
  // a liquid reaches the cold room carrying 37.9 MJ of sensible heat against the 7.6 MJ its
  // boiling absorbs, and the cold room warms 43 K. Averaging the run pulls both legs to one
  // temperature and puts that failure back. A temperature per conduit with exchange between neighbours
  // keeps the gradient along a run, as a real pipe has, and still levels a run left alone.
  //
  // THE EXCHANGE. For every link between two participating conduits i and j of one run,
  //
  //     q = k (T_i - T_j) C_i C_j / (C_i + C_j),   T_i -= q / C_i,   T_j += q / C_j,
  //
  // all from the temperatures as they stand when this starts (Jacobi, not Gauss-Seidel), so
  // the order conduits sit in `vec_` changes nothing. `C_i C_j / (C_i + C_j)` is the pair's
  // reduced heat capacity: at k = 1 one link alone would carry the pair to its common
  // temperature, so k is the fraction of that gap a link closes per 200 ms update. Each link is
  // taken once and subtracts exactly what it adds, so the run's contents energy is conserved --
  // exactly in the double accumulation, to float rounding in what is stored -- and no energy
  // crosses to or from a building: that exchange has already happened by the time this runs.
  //
  // WHY k <= 0.25. One link moves T_i by k C_j / (C_i + C_j) < k of the gap to T_j, and a
  // conduit has at most four links, so the weights on its neighbours sum below 4k <= 1 and
  // T_i' is a weighted average of T_i and its neighbours' temperatures. Nothing overshoots or
  // rings, and no conduit leaves the range its neighbourhood started in -- so in particular,
  // mixing cannot carry a conduit past a temperature that neither it nor any neighbour was
  // already past. Only a corrupt bind can hand a conduit more than four links (the game's
  // connections are one per side); for that case each link's k is scaled by four over its
  // busier end's link count, which keeps the same bound without refusing the run.
  //
  // WHICH LINKS. A conduit's neighbours come from `ConduitFlow.SOAInfo.GetConduitConnections`,
  // which the game builds from each conduit's OWN `UtilityConnections` mask, so a link can be
  // listed from one side only. A link counts once: from the conduit at the lower dense position
  // when both ends list it, from the only end that lists it otherwise (`OwnedLink`, which the
  // aggregate's `linkCount` also uses). Both ends must be bound to the same run of the same
  // type, and both must be participants.
  //
  // A conduit whose building handle is dead is still mixed and its output slot written, which
  // Klei's loop would have left alone. That only applies inside a run a caller asked to mix.
  void MixNetworks() {
    const float min_hc = g_tunables.min_conduit_heat_capacity;  // tunables.h, THE ONE RULE
    // The ceiling is checked when `SetPolicy` stores a fraction, and applied again here, at use:
    // `MaxConduitMixFraction` is a live tunable, so a policy stored under a higher ceiling must
    // not outlive a lowered one.
    // Bit-exact while the ceiling has not moved, since every stored fraction is already under it.
    const float max_mix = g_tunables.max_conduit_mix_fraction;
    std::vector<ConduitTemperatureData>& data = vec_.Data();
    const std::vector<int32_t>& handles = vec_.Handles();
    mix_links_.clear();
    mix_degree_.assign(data.size(), 0);
    for (size_t i = 0; i < data.size(); ++i) {
      const ConduitTemperatureData& a = data[i];
      if (!MixesIn(a) || !MixParticipant(a, min_hc)) continue;
      for (int32_t s = 0; s < kConduitNeighbourSlots; ++s) {
        const ConduitTemperatureData* b = OwnedLink(i, s);
        if (!b || !MixParticipant(*b, min_hc)) continue;
        const size_t j = static_cast<size_t>(b - data.data());
        mix_links_.push_back(MixLink{i, j});
        ++mix_degree_[i];
        ++mix_degree_[j];
      }
    }
    if (mix_links_.empty()) return;

    mix_delta_.assign(data.size(), 0.0);
    for (const MixLink& link : mix_links_) {
      const ConduitTemperatureData& a = data[link.a];
      const ConduitTemperatureData& b = data[link.b];
      const TypeBinding& t = types_[a.conduit_type];
      double k = static_cast<double>(
          std::min(t.policy[static_cast<size_t>(a.network)].mix_fraction, max_mix));
      const int32_t busier = std::max(mix_degree_[link.a], mix_degree_[link.b]);
      if (busier > kConduitNeighbourSlots) {
        k *= static_cast<double>(kConduitNeighbourSlots) / static_cast<double>(busier);
      }
      const double ca = static_cast<double>(a.contents_heat_capacity);
      const double cb = static_cast<double>(b.contents_heat_capacity);
      const double gap = static_cast<double>(a.temperature) - static_cast<double>(b.temperature);
      const double q = k * gap * (ca * cb / (ca + cb));
      mix_delta_[link.a] -= q;
      mix_delta_[link.b] += q;
    }
    for (size_t i = 0; i < data.size(); ++i) {
      if (mix_delta_[i] == 0.0) continue;
      ConduitTemperatureData& d = data[i];
      const float next = static_cast<float>(
          static_cast<double>(d.temperature) +
          mix_delta_[i] / static_cast<double>(d.contents_heat_capacity));
      if (!std::isfinite(next)) continue;
      d.temperature = next;
      const size_t out = static_cast<size_t>(handles[i] & kHandleIndexMask);
      if (out < out_temperatures_.size()) out_temperatures_[out] = next;
    }
  }

  // The sums are stale when the conduit list changed (the dirty flag) OR when an element input
  // they were computed from changed (the table's stamp): a density or molecular mass written
  // after a run was summed changes its liquid volume or its moles without touching a conduit.
  void RefreshAggregates(const ElementTable& table) {
    const bool dirty = aggregates_dirty_.exchange(false, std::memory_order_relaxed);
    const uint64_t stamp = table.ConduitAggregateInputsStamp();
    if (!dirty && stamp == aggregates_stamp_) return;
    aggregates_stamp_ = stamp;
    RebuildAggregates(table);
  }

  void RebuildAggregates(const ElementTable& table) {
    const float fallback_density = g_tunables.fallback_conduit_liquid_density_kg_m3;
    for (int32_t type = kConduitTypeGas; type <= kConduitTypeLiquid; ++type) {
      TypeBinding& t = types_[type];
      t.accum.assign(static_cast<size_t>(t.network_count), NetworkAccum{});
    }
    const std::vector<ConduitTemperatureData>& data = vec_.Data();
    for (size_t i = 0; i < data.size(); ++i) {
      const ConduitTemperatureData& d = data[i];
      if (d.conduit_type != kConduitTypeGas && d.conduit_type != kConduitTypeLiquid) continue;
      TypeBinding& t = types_[d.conduit_type];
      if (d.network < 0 || d.network >= t.network_count) continue;
      NetworkAccum& a = t.accum[static_cast<size_t>(d.network)];
      ++a.conduits;
      for (int32_t s = 0; s < kConduitNeighbourSlots; ++s) {
        if (OwnedLink(i, s)) ++a.links;
      }
      AccumulateTrapped(table, d, &a);
      const double t_k = static_cast<double>(d.temperature);
      const bool finite_t = std::isfinite(d.temperature);
      if (d.contents_heat_capacity > 0.0f && finite_t) {
        a.heat_capacity += static_cast<double>(d.contents_heat_capacity);
        a.energy += static_cast<double>(d.contents_heat_capacity) * t_k;
      }
      if (!(d.contents_mass > 0.0f)) continue;
      ++a.filled;
      const double mass = static_cast<double>(d.contents_mass);
      a.mass += mass;
      if (finite_t) a.mass_temperature += mass * t_k;
      const float molar = table.MolecularMassOf(d.element_idx);
      if (molar > 0.0f) a.moles += mass * gas::kGramsPerKilogram / static_cast<double>(molar);
      if (table.Phase(d.element_idx) == kStateLiquid) {
        const float density = table.LiquidDensityOf(d.element_idx);
        a.liquid_volume_m3 += mass / static_cast<double>(density > 0.0f ? density
                                                         : fallback_density);
      }
      int32_t slot = -1;
      for (int32_t k = 0; k < a.species_count; ++k) {
        if (a.species[k] == d.element_idx) {
          slot = k;
          break;
        }
      }
      if (slot < 0 && a.species_count < ONI_CONDUIT_NET_MAX_SPECIES) {
        slot = a.species_count++;
        a.species[slot] = d.element_idx;
      }
      if (slot >= 0) {
        a.species_mass[slot] += mass;
      } else {
        bool seen = false;
        for (uint16_t o : a.overflow) seen = seen || o == d.element_idx;
        if (!seen) a.overflow.push_back(d.element_idx);
      }
    }
  }

  void AccumulateTrapped(const ElementTable& table, const ConduitTemperatureData& d,
                         NetworkAccum* a) const {
    const std::unordered_map<int32_t, Trapped>& store = trapped_[d.conduit_type];
    if (store.empty()) return;
    const auto it = store.find(d.cell);
    if (it == store.end()) return;
    const Trapped& m = it->second;
    if (!(m.mass > 0.0f) || m.element_idx >= table.Count()) return;
    const double mass = static_cast<double>(m.mass);
    switch (table.Phase(m.element_idx)) {
      case kStateGas: {
        a->trapped_gas_mass += mass;
        a->trapped_gas_mass_temperature += mass * static_cast<double>(m.temperature);
        const float molar = table.MolecularMassOf(m.element_idx);
        if (molar > 0.0f) {
          a->trapped_gas_moles += mass * gas::kGramsPerKilogram / static_cast<double>(molar);
        }
        break;
      }
      case kStateLiquid: {
        a->trapped_liquid_mass += mass;
        const float density = table.LiquidDensityOf(m.element_idx);
        if (density > 0.0f) a->liquid_volume_m3 += mass / static_cast<double>(density);
        break;
      }
      case kStateSolid:
        a->trapped_solid_mass += mass;
        break;
      default:
        break;
    }
  }

  // PipeMatterFacade.TryGetLiquidHeadspacePressurePa, natively: the trapped gas at its
  // mass-weighted temperature over whatever the liquid does not fill. 0 with no gas or no room.
  static float HeadspacePressurePa(const TypeBinding& t, const NetworkAccum& a) {
    const double capacity = static_cast<double>(a.conduits) * t.volume_per_conduit_m3;
    const double headspace = capacity - a.liquid_volume_m3;
    if (!(headspace > 0.0) || !(a.trapped_gas_mass > 0.0) || !(a.trapped_gas_moles > 0.0)) {
      return 0.0f;
    }
    const float temperature = static_cast<float>(a.trapped_gas_mass_temperature / a.trapped_gas_mass);
    return gas::PressureFromMoles(static_cast<float>(a.trapped_gas_moles), temperature,
                                  static_cast<float>(headspace));
  }

  static float RunPressurePa(const TypeBinding& t, const NetworkAccum& a) {
    if (!(a.moles > 0.0) || !(a.heat_capacity > 0.0)) return 0.0f;
    const float temperature = static_cast<float>(a.energy / a.heat_capacity);
    const float volume = static_cast<float>(static_cast<double>(a.conduits) * t.volume_per_conduit_m3);
    if (!(temperature > 0.0f) || !(volume > 0.0f)) return 0.0f;
    return gas::PressureFromMoles(static_cast<float>(a.moles), temperature, volume);
  }

  // PRESSURE-DRIVEN PHASE CHANGE, DECIDED HERE AND CARRIED OUT BY THE MANAGED SIDE. This is
  // OniFramework's PipeMatterFacade.TickNetworkPhaseChange and CondenseHeadspaceGas with the
  // mass movement taken out: the same pressure, the same clamped vapour curve, the same
  // `phase::ComputePhaseChangeStep` the managed code already reached through
  // SIM_ComputePhaseChangeStep, and the same order of checks, so a run under the PHASE policy
  // converts what the managed pass would have. Two deliberate differences, both recorded in
  // the ABI comment: the cadence is this 200 ms update rather than a caller's own sweep, and a
  // run's pressure is taken at its heat-capacity-weighted temperature (identical for the
  // one-element runs a vanilla conduit builds).
  //
  // THE CADENCE IS NOT A DETAIL. A managed sweep runs once per REAL second, so between sweeps a
  // line drifts off its curve (a refrigeration loop's hot line can sit 61.5 K below its dew
  // point). This update holds a tile ON the curve, so anything deciding the reverse direction
  // elsewhere, at the slow cadence, fights it. Hence the fourth kind below
  // (ProposeTrappedEvaporation): the gas run's return leg is decided here too.
  //
  // Every decision reads the state as this update left it and nothing here writes any of it,
  // so the proposals are independent: a boil and a headspace condensation on one tile are both
  // judged against the pre-update headspace pressure, as the managed pass judged them.
  void ProposePhaseChanges(const ElementTable& table, float dt) {
    RefreshAggregates(table);
    for (const ConduitTemperatureData& d : vec_.Data()) {
      if (d.conduit_type != kConduitTypeGas && d.conduit_type != kConduitTypeLiquid) continue;
      const TypeBinding& t = types_[d.conduit_type];
      if (!t.any_phase || d.network < 0 || d.network >= t.network_count) continue;
      const NetworkPolicy& policy = t.policy[static_cast<size_t>(d.network)];
      if ((policy.flags & ONI_CONDUIT_POLICY_PHASE) == 0) continue;
      const NetworkAccum& a = t.accum[static_cast<size_t>(d.network)];

      if (d.conduit_type == kConduitTypeGas) {
        const float pressure = RunPressurePa(t, a);
        if (pressure > 0.0f) {
          ProposeContents(table, d, policy, pressure, dt, kStateGas, kStateLiquid,
                          ONI_CONDUIT_PHASE_CONDENSE_CONTENTS);
          ProposeTrappedEvaporation(table, d, t, policy, pressure, dt);
        }
        continue;
      }
      // A liquid run's pressure may legitimately be zero -- a line holding no gas at all -- and
      // zero is an answer, not a failure: the clamp resolves it to the freezing point, so an
      // unpressurised line boils, which is the behaviour Stationeers warns players about.
      const float headspace = HeadspacePressurePa(t, a);
      ProposeContents(table, d, policy, headspace, dt, kStateLiquid, kStateGas,
                      ONI_CONDUIT_PHASE_BOIL_CONTENTS);
      if (headspace > 0.0f) ProposeHeadspace(table, d, t, policy, headspace, dt);
    }
  }

  void ProposeContents(const ElementTable& table, const ConduitTemperatureData& d,
                       const NetworkPolicy& policy, float pressure, float dt, uint8_t source_state,
                       uint8_t product_state, int32_t kind) {
    if (!(d.contents_mass > 0.0f) || d.element_idx >= table.Count()) return;
    if (table.Phase(d.element_idx) != source_state) return;
    const Element& e = table.At(d.element_idx);
    const uint16_t product = source_state == kStateGas ? e.lowTempTransitionIdx
                                                       : e.highTempTransitionIdx;
    if (product == 0xFFFF || product >= table.Count() || table.Phase(product) != product_state) {
      return;
    }
    ElementTable::PhaseCurve curve;
    if (!table.PhaseCurveOf(d.element_idx, &curve)) return;
    const float boundary = EvaporationTemperatureClampedK(curve, pressure);
    // Condensing needs the gas BELOW its dew point; boiling needs the liquid ABOVE its boiling
    // point. Same boundary, opposite side.
    if (source_state == kStateGas ? !(d.temperature < boundary) : !(d.temperature > boundary)) {
      return;
    }
    if (!(curve.latent_j_per_kg > 0.0f)) return;
    const phase::PhaseChangeStep step = phase::ComputePhaseChangeStep(
        d.contents_mass, d.temperature, boundary, curve.latent_j_per_kg,
        e.specificHeatCapacity * gas::kGramsPerKilogram, dt, policy.phase_rate_per_second,
        policy.phase_min_remainder_kg);
    if (!(step.converted_mass_kg > 0.0f)) return;
    OniConduitPhaseProposal p{};
    p.conduitType = d.conduit_type;
    p.networkId = d.network;
    p.cell = d.cell;
    p.kind = kind;
    p.sourceElementIdx = d.element_idx;
    p.productElementIdx = product;
    p.sourceMassKg = d.contents_mass;
    p.sourceTemperatureK = d.temperature;
    p.convertedMassKg = step.converted_mass_kg;
    p.boundaryK = boundary;
    p.pressurePa = pressure;
    p.latentHeatJPerKg = curve.latent_j_per_kg;
    proposals_.push_back(p);
  }

  // CondenseHeadspaceGas: standing gas above its dew point at the headspace pressure goes back
  // into the conduit as liquid -- but only into a conduit that is empty or already carrying
  // that liquid (ConduitContents is one element), and only as far as the conduit's capacity.
  void ProposeHeadspace(const ElementTable& table, const ConduitTemperatureData& d,
                        const TypeBinding& t, const NetworkPolicy& policy, float pressure,
                        float dt) {
    const std::unordered_map<int32_t, Trapped>& store = trapped_[kConduitTypeLiquid];
    const auto it = store.find(d.cell);
    if (it == store.end()) return;
    const Trapped& m = it->second;
    if (!(m.mass > 0.0f) || m.element_idx >= table.Count()) return;
    if (table.Phase(m.element_idx) != kStateGas) return;
    const Element& g = table.At(m.element_idx);
    const uint16_t liquid = g.lowTempTransitionIdx;
    if (liquid == 0xFFFF || liquid >= table.Count() || table.Phase(liquid) != kStateLiquid) return;
    ElementTable::PhaseCurve curve;
    if (!table.PhaseCurveOf(m.element_idx, &curve)) return;
    const float boundary = EvaporationTemperatureClampedK(curve, pressure);
    if (!(m.temperature < boundary)) return;
    if (!(curve.latent_j_per_kg > 0.0f)) return;
    const bool has_contents = d.contents_mass > 0.0f && !table.IsVacuum(d.element_idx);
    if (has_contents && d.element_idx != liquid) return;
    const float headroom = t.max_mass_per_conduit_kg - (has_contents ? d.contents_mass : 0.0f);
    if (!(headroom > 0.0f)) return;
    const phase::PhaseChangeStep step = phase::ComputePhaseChangeStep(
        m.mass, m.temperature, boundary, curve.latent_j_per_kg,
        g.specificHeatCapacity * gas::kGramsPerKilogram, dt, policy.phase_rate_per_second,
        policy.phase_min_remainder_kg);
    const float converted = std::min(step.converted_mass_kg, headroom);
    if (!(converted > 0.0f)) return;
    OniConduitPhaseProposal p{};
    p.conduitType = kConduitTypeLiquid;
    p.networkId = d.network;
    p.cell = d.cell;
    p.kind = ONI_CONDUIT_PHASE_CONDENSE_HEADSPACE;
    p.sourceElementIdx = m.element_idx;
    p.productElementIdx = liquid;
    p.sourceMassKg = m.mass;
    p.sourceTemperatureK = m.temperature;
    p.convertedMassKg = converted;
    p.boundaryK = boundary;
    p.pressurePa = pressure;
    p.latentHeatJPerKg = curve.latent_j_per_kg;
    proposals_.push_back(p);
  }

  // THE REVERSE OF CONDENSE_CONTENTS: liquid standing in a gas conduit going back into it as
  // gas once the run's pressure puts the dew point below the tile. With the two directions decided
  // in two places -- this update condensing, a once-a-second managed pass returning the tile
  // whenever the conduit reads warm -- a tile held on its dew point is found a hair above,
  // returned, and condensed again, over and over. Deciding both directions here, against ONE boundary, is what the liquid run already
  // had (BOIL_CONTENTS and CONDENSE_HEADSPACE).
  //
  // WHICH TEMPERATURE: the conduit's contents when it has any, the standing liquid's own when it
  // has none -- the managed pass's rule, and the temperature CONDENSE_CONTENTS reads, so a tile
  // is never proposed to condense and to evaporate in the same update. The step is the same
  // `phase::ComputePhaseChangeStep` on the standing liquid's own mass and heat capacity, the
  // way CONDENSE_HEADSPACE runs it on the standing gas's; the boundary and latent heat are the
  // VAPOUR's curve, the one condensation used. Only into a conduit that is empty or already
  // carrying that vapour (ConduitContents is one element), and only as far as its capacity.
  void ProposeTrappedEvaporation(const ElementTable& table, const ConduitTemperatureData& d,
                                 const TypeBinding& t, const NetworkPolicy& policy, float pressure,
                                 float dt) {
    const std::unordered_map<int32_t, Trapped>& store = trapped_[kConduitTypeGas];
    if (store.empty()) return;
    const auto it = store.find(d.cell);
    if (it == store.end()) return;
    const Trapped& m = it->second;
    if (!(m.mass > 0.0f) || m.element_idx >= table.Count()) return;
    if (table.Phase(m.element_idx) != kStateLiquid) return;
    const Element& l = table.At(m.element_idx);
    const uint16_t vapour = l.highTempTransitionIdx;
    if (vapour == 0xFFFF || vapour >= table.Count() || table.Phase(vapour) != kStateGas) return;
    ElementTable::PhaseCurve curve;
    if (!table.PhaseCurveOf(vapour, &curve)) return;
    const float boundary = EvaporationTemperatureClampedK(curve, pressure);
    const bool has_contents = d.contents_mass > 0.0f && !table.IsVacuum(d.element_idx);
    const float judged = has_contents ? d.temperature : m.temperature;
    if (!(judged > boundary)) return;
    if (!(curve.latent_j_per_kg > 0.0f)) return;
    if (has_contents && d.element_idx != vapour) return;
    const float headroom = t.max_mass_per_conduit_kg - (has_contents ? d.contents_mass : 0.0f);
    if (!(headroom > 0.0f)) return;
    const phase::PhaseChangeStep step = phase::ComputePhaseChangeStep(
        m.mass, judged, boundary, curve.latent_j_per_kg,
        l.specificHeatCapacity * gas::kGramsPerKilogram, dt, policy.phase_rate_per_second,
        policy.phase_min_remainder_kg);
    const float converted = std::min(step.converted_mass_kg, headroom);
    if (!(converted > 0.0f)) return;
    OniConduitPhaseProposal p{};
    p.conduitType = kConduitTypeGas;
    p.networkId = d.network;
    p.cell = d.cell;
    p.kind = ONI_CONDUIT_PHASE_EVAPORATE_TRAPPED;
    p.sourceElementIdx = m.element_idx;
    p.productElementIdx = vapour;
    p.sourceMassKg = m.mass;
    p.sourceTemperatureK = judged;
    p.convertedMassKg = converted;
    p.boundaryK = boundary;
    p.pressurePa = pressure;
    p.latentHeatJPerKg = curve.latent_j_per_kg;
    proposals_.push_back(p);
  }

  TypeBinding types_[3];
  // Conduits in mixing runs that reached the end of this update's exchange loop, by dense
  // position, waiting for their transition check. Cleared at the top of every update.
  std::vector<int32_t> deferred_;
  // `MixNetworks`' scratch, kept between updates only so it is not reallocated each time:
  // this update's links, how many links each dense position has, and each one's energy change.
  std::vector<MixLink> mix_links_;
  std::vector<int32_t> mix_degree_;
  std::vector<double> mix_delta_;
  std::unordered_map<int32_t, Trapped> trapped_[3];
  // Sim handles of the buildings that convect with their cells themselves. See
  // SetStructureConvection.
  std::unordered_set<int32_t> convecting_structures_;
  std::vector<OniConduitPhaseProposal> proposals_;
  std::atomic<bool> aggregates_dirty_{true};
  uint64_t aggregates_stamp_ = 0;
};

}  // namespace oni_sim
