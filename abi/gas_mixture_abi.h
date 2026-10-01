// The per-cell gas mixture -- the SoA contract. Both the SimDLL's own mixing kernel
// and the managed P/Invoke facade build against these shapes, so changing them
// means changing both sides in lockstep.
//
// The mixture sits beside sim/world.h's single-element PhaseEntry (element, mass,
// temperature), which the game still reads; the projection reconciles the two.
#pragma once
#include <cstdint>

namespace oni_sim::gas {

// Max distinct gas species allowed to coexist in one cell. Fixed at compile
// time, not dynamic — SoA lanes need a known stride for alignment/SIMD, and
// a bitmask needs a fixed width. 8 covers vanilla's real breathable-gas set
// (O2, CO2, H2, N2, Cl2, natural gas, steam, contaminated O2) with room for
// Mod 3's Mars-only additions without a layout change.
constexpr int kMaxSpeciesPerCell = 8;

// 0 is reserved as "empty slot" in species[] and occupied_mask — matches
// vanilla's own convention that SimHashes has no valid gas id of 0.
constexpr uint16_t kEmptySpecies = 0;

// Ideal-gas-law constant, J/(mol*K).
constexpr float kGasConstantR = 8.3144f;

// One cell's gas mixture. This struct documents the SHAPE only — it is
// never instantiated per-cell in game memory. Real storage is N parallel
// arrays (see GasMixtureSoA below), one flat array per field, each
// kMaxSpeciesPerCell * cellCount elements long, laid out
// [slot0 x all cells][slot1 x all cells]...[slot7 x all cells] so a SIMD
// pass over one species slot across many cells stays contiguous — the
// same reason World already keeps one std::vector<T> per field instead of
// a single std::vector<PhaseEntry>-shaped-for-mixtures.
struct GasMixtureCell {
  // Species identity per slot, kEmptySpecies (0) if unused.
  uint16_t species[kMaxSpeciesPerCell];
  // Partial mass (kg) per slot, parallel to species[]. Molar mass for
  // pressure math comes from the existing vanilla Element table via
  // species[i] — not duplicated here.
  float mass[kMaxSpeciesPerCell];
  // Bit i set iff species[i] != kEmptySpecies. Redundant with species[]
  // but kept explicit: an occupancy/dirty check tests one uint8 instead of
  // walking up to 8 slots.
  uint8_t occupied_mask;
  // Shared cell temperature (K) — ONE value for the whole mixture. Mixed
  // species in a single tile are assumed to reach thermal equilibrium far
  // faster than the gas-mixing tick rate, so conduction/radiation stay a
  // per-cell cost, not per-species -- the same shape as a Stationeers
  // atmosphere (one temperature, several gases).
  float temperature;
  // Mixture changed since the last mixing pass ran — gates the per-cell
  // mixing/pressure work. Temporal-coherence dirty flag, per cell (not per
  // slot).
  uint8_t dirty;
  // Room-pooled and equilibrated with its neighbors — skip this cell's
  // pressure/mixing work until a boundary stimulus (dirty neighbor, new
  // injection, opened door) wakes it. Distinct from `dirty`: a sleeping
  // room's cells are never individually dirty either, but the flag is kept
  // separate so waking logic can distinguish "never checked" from
  // "checked, found stable, parked."
  uint8_t sleeping;
};

// Total bytes per cell in the SoA lane group above, for capacity/memory
// budgeting before the real vectors exist:
//   species: 8 * 2  = 16
//   mass:    8 * 4  = 32
//   occupied_mask:    1
//   temperature:      4
//   dirty:            1
//   sleeping:         1
//   -----------------------
//   = 55 bytes/cell (vs. vanilla PhaseEntry's 8 bytes/cell: element u16 +
//   mass f32 + temperature f32). ~7x the working set per cell before any
//   room-pooling/sleep skip is applied — so the working set
//   grows several times over a single-element cell. Sleep/pooling exist to keep the HOT
//   (touched-this-tick) working set close to vanilla's, not to shrink the
//   cold storage footprint.
constexpr int kBytesPerCellUnpacked =
    kMaxSpeciesPerCell * (2 + 4) + 1 + 4 + 1 + 1;
static_assert(kBytesPerCellUnpacked == 55, "layout size changed silently");

// Room-graph pooling, a technique of its own rather than part of `sleeping`.
// The game's room concept (RoomProber/CavityInfo) lives in managed code, not
// in the SimDLL, so the pooling is a flood fill over the grid's solid mask
// done here.
struct RoomPoolState {
  uint32_t room_id;          // index into the room table, 0 = unassigned
  uint8_t all_cells_sleeping; // every cell in this room currently sleeping
};

// Multi-rate tick counters -- separate cadences for separate cost classes.
// The values are starting points, to be tuned by measurement.
struct TickRates {
  uint8_t mixing_every_n_ticks = 1;     // fast: pressure/mixing
  uint8_t convection_every_n_ticks = 3; // moderate: convective heat
  uint8_t radiant_every_n_ticks = 10;   // slow: radiant loss
};

// The gate every cadence in TickRates shares: a subsystem clocked at
// `every_n_ticks` runs on tick 0, `every_n_ticks`, `2*every_n_ticks`, ...
// `every_n_ticks == 0` is treated as "never runs" rather than dividing by
// zero, since a caller could plausibly zero a rate to disable a subsystem
// outright. Only `mixing_every_n_ticks` has a real kernel to gate today (sim/gas_mixture.h
// / sim/gas_rooms.h's mixing pass) — convection and radiant loss have no
// kernel yet, so their rates stay declared-but-unconsumed until those
// subsystems exist. Don't build stub kernels just to exercise this gate;
// measure it against the one real consumer there is.
inline bool ShouldTick(uint64_t tick_index, uint8_t every_n_ticks) {
  if (every_n_ticks == 0) return false;
  return tick_index % every_n_ticks == 0;
}

}  // namespace oni_sim::gas
