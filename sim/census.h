// Per-kernel cell accounting: how many cells a sweep LOOKED AT, and how many it actually
// changed.
//
// WHY THIS IS NOT PART OF THE PROFILER. `Profiler` (sim/simdll.cpp) answers "where did the
// frame go" in milliseconds, and a millisecond is the wrong unit for the largest class of
// defect a sim like this ships: a sweep that walks the entire grid to touch a handful of
// cells. A timing says a kernel is slow and can never say that it formed 98,304 visits to do
// 400 cells of work.
// That ratio is the diagnostic, and it is a COUNT. Two numbers per kernel make it visible,
// and -- because a count is exact where a time is noisy -- assertable, which is what turns
// this from a reporting tool into a gate. See `vftest --census`.
//
// ALWAYS COMPILED, ALWAYS COUNTING. `ONI_CONDUCTION_CENSUS` (sim/physics.h) is the prototype
// this generalises, and it is `#ifdef`-gated off, so it has never once run in a shipped
// build or in CI -- instrumentation nobody can switch on from a normal build is
// instrumentation that rots. The cost here is bounded by construction:
//
//   * `ONI_EXAMINED` is added ONCE PER ROW, never per cell -- one integer add per row of a
//     sweep whose row body is dozens of loads and a branch tree.
//   * `ONI_ACTIVE` sits on a branch the kernel ALREADY takes and already predicts, at the
//     point where the cell's state is written.
//   * Nothing here is read inside a kernel, so no load is added to any inner loop.
//
// MEASURED, six runs each side, `asteroid` at 256x384 clamped, 60 ticks, against a build of
// the same tree with none of this in it: the frame's best min goes 6.792 -> 6.807 ms (+0.2 %)
// and its best median 7.118 -> 7.199 (+1.1 %). The min is the steadiest statistic and the
// medians' own run-to-run spread is under 2 %, so the reading is "a fifth of a percent on the
// number that holds still, inside the noise on the one that does not" -- not "free".
//
// COUNT IT IN THE LOOP, NOT FROM THE LOOP BOUNDS. `examined` could be computed as the
// rectangle's area with one multiply per invocation and no per-row add at all. It is
// deliberately not, and the reason is the whole point of the counter: a number derived from
// the same `r.x0`/`r.x1` the loop is driven by AGREES WITH THAT LOOP EVEN WHEN THE LOOP IS
// WRONG. It would have reported a perfect ratio for the whole-grid sweeps that this exists
// to catch. The counter has to ride the iteration it is describing.
//
// THREADING. One plain global, incremented only by the thread running the frame. The sim
// runs its frame on a worker thread and every reader here arrives through
// `SIM_HandleMessage`, which calls `WaitIdle()` before any immediate message -- so a read
// happens with no frame in flight, and that is a guarantee of the message path rather than
// an assumption about timing. `bench` is single-threaded and calls the kernels directly.
//
// SLOT NAMES LIVE HERE, not in the profiler, because `bench` links the kernels directly out
// of `sim/` and never sees `simdll.cpp` at all. Both readers -- the offline harness and the
// live DLL -- need the same names for the same slots or their output cannot be compared,
// which is exactly the drift a performance baseline cannot survive.
#pragma once

#include <cstdint>

namespace oni_sim {
namespace census {

// One entry per timed kernel. `Profiler::Slot` (sim/simdll.cpp) is this enum -- it was moved
// here rather than duplicated, so a slot added to one is a slot added to both and the
// timings and the counts can never come to disagree about what a row means.
enum Slot {
  kDrainQueue,
  kConduction,
  kStateChange,
  kGasPressure,
  kGasDisplacement,
  kFlow,
  // Layer C mixing (sim/payload_mix.h). Counted as a cell sweep: it walks its region's
  // rectangle exactly once, and `changes` against `examined` is the number worth reading --
  // a pond with a gradient in it is the only place this kernel writes anything at all.
  kPayloadMix,
  kPostProcess,
  kDisease,
  kZeroMassless,
  kElementFlow,
  kRadiation,
  // The generic field solver. Counted, but NOT a cell sweep, and the two are
  // separate facts. `IsCellSweep` is a whitelist of the plain region sweeps, used for the
  // per-invocation bound assertion; a phase whose cost is a per-source raster plus a ray per
  // emitted pixel plus an optional decay pass has no single rectangle to be held to, which is
  // why its bound is `kBoundNone`. What it does have is a ratio worth reading: `examined`
  // here counts every cell the walks stepped on, RAY STEPS INCLUDED, against `changes`, the
  // cells a value was actually written into. That quotient is the multiplier worth
  // pricing -- for a point source it is the average ray length, and it is the number
  // that says whether a field is a sweep or a raster.
  kFields,
  kElementChunk,
  kBuildingHeat,
  kBuildingToBuilding,
  kVolumeFractions,
  // Layer C3 (sim/effervescence.h). A cell sweep over the WHOLE padded grid, column by column,
  // once per `kSetEffervescence` period rather than once per substep; `changes` is the number of
  // cells that fizzed.
  kEffervescence,
  kWorldEnvironment,
  kProject,
  kTextures,
  kFrame,
  kSlotCount
};

inline const char* Name(int s) {
  switch (s) {
    case kDrainQueue: return "DrainQueue";
    case kWorldEnvironment: return "StepWorldEnvironment";
    case kConduction: return "StepConduction";
    case kStateChange: return "StepStateChange";
    case kGasPressure: return "StepGasPressure";
    case kGasDisplacement: return "StepGasDisplacement";
    case kFlow: return "StepFlow";
    case kPayloadMix: return "StepPayloadMixing";
    case kPostProcess: return "StepPostProcess";
    case kDisease: return "StepDisease";
    case kZeroMassless: return "ZeroMasslessCells";
    case kElementFlow: return "StepElementFlow";
    case kRadiation: return "StepRadiationEmitters";
    case kFields: return "StepFields";
    case kElementChunk: return "StepElementChunks";
    case kBuildingHeat: return "StepBuildingHeatExchange";
    case kBuildingToBuilding: return "StepBuildingToBuilding";
    case kVolumeFractions: return "TickRoomPooledMixing";
    case kEffervescence: return "StepEffervescence";
    case kProject: return "Project";
    case kTextures: return "FillPropertyTextures";
    case kFrame: return "whole frame";
    default: return "?";
  }
}

// WHICH SLOTS THE RATIO MEANS ANYTHING FOR. A cell sweep walks a rectangle of the grid, so
// "examined" is cells and the invariant below applies to it. The rest walk LISTS -- the
// message queue, the registered buildings, the element chunks, the radiation emitters --
// where the population is whatever the game registered and a ratio against the grid says
// nothing at all. Those slots are left uncounted rather than counted in a unit that does not
// compare, because a column that mixes cells and list entries is worse than an empty one.
//
// `kElementFlow` is on the list side despite the name: it times the element and disease
// EMITTERS and CONSUMERS, which walk the components the game registered, not the grid.
//
// `kFrame` is the sum row and sweeps nothing itself.
inline bool IsCellSweep(int s) {
  switch (s) {
    case kConduction:
    case kStateChange:
    case kGasPressure:
    case kGasDisplacement:
    case kFlow:
    case kPayloadMix:
    case kPostProcess:
    case kDisease:
    case kZeroMassless:
    case kEffervescence:
    case kProject:
    case kTextures:
      return true;
    default:
      return false;
  }
}

// WHAT BOUNDS A SWEEP'S ITERATION -- declared per kernel, with the reason, so that "this
// kernel looked at more cells than its region" can be an assertion instead of a warning.
//
// The first version of this file flagged every sweep whose `examined` exceeded the region
// budget, and on a quarter-region asteroid that fired on FIVE kernels, four of them behaving
// exactly as they are documented to. A warning that is usually wrong is a warning nobody
// acts on -- so the bound is a property of the kernel, stated here next to why it is what it
// is, and the check is against the kernel's OWN bound.
enum Bound {
  // Walks its region's half-open rectangle and nothing else. The strictest bound, and the one
  // every plain region sweep is held to.
  kBoundRegion,
  // Walks the region grown one cell on the far edges -- `World::PaddedRegionInclusive`. The
  // world's top row is post-processed, and `sunlit` is the scenario that says so.
  kBoundRegionInclusive,
  // Not bounded by the region at all. Three different reasons, none of them a defect:
  // `StepGasDisplacement` and `StepLiquidDisplacement` transcribe Klei's `min(y0, 3)` slip,
  // which plainly means `max` and starts the sweep near the bottom of the world however high
  // the region begins; `Project` is driven by dirty rectangles that other kernels marked, not
  // by the region; `FillPropertyTextures` and `ZeroMasslessCells` have no region input at all.
  // The bound is still real -- a sweep may not exceed the padded grid per invocation -- and it
  // is the bound that catches a kernel walking the grid once per CELL rather than once.
  kBoundGrid,
  // No bound this file can state. `StepFields` is the only one: a field's cost is a decay
  // pass over the region, plus a raster per source, plus a Bresenham walk PER EMITTED PIXEL
  // for a point source -- so a correct run legitimately examines many times its region's
  // area, and a rectangle it may not exceed would be a number invented to have one. The
  // counts are still collected and still reported; what is dropped is the assertion, and it
  // is dropped explicitly rather than by being quietly excluded from `IsCellSweep`.
  kBoundNone
};

inline Bound SweepBound(int s) {
  switch (s) {
    case kFields: return kBoundNone;
    case kPostProcess: return kBoundRegionInclusive;
    case kGasDisplacement:
    case kFlow:
    case kZeroMassless:
    case kEffervescence:
    case kProject:
    case kTextures:
      return kBoundGrid;
    default:
      return kBoundRegion;
  }
}

inline const char* BoundName(Bound b) {
  switch (b) {
    case kBoundRegion: return "region";
    case kBoundRegionInclusive: return "region+1";
    case kBoundGrid: return "grid";
    case kBoundNone: return "-";
  }
  return "?";
}

struct Counters {
  // Cells the sweep's own loop stepped over. Incremented per ROW, by the row's width, from
  // inside the loop that walks the rows.
  uint64_t examined = 0;
  // State changes this sweep ANNOUNCED. For every kernel that moves substance this is a call
  // to `World::TouchSubstance` -- the sim's own "this cell changed" signal, the one the game
  // itself consumes -- counted at that single site rather than at eleven hand-picked branches,
  // so it is the kernel's definition of a change and not this file's opinion of one.
  // `StepConduction` is the exception and says so where it is counted: it moves temperature
  // and announces nothing, so it counts the driving cell of a pair that transferred.
  //
  // IT IS A COUNT OF CHANGES, NOT OF DISTINCT CELLS, and the difference is not pedantry: a
  // gas sweep can announce the same cell as a source and again as a destination inside one
  // invocation, so `changes` may legitimately exceed `examined` and NOTHING HERE ASSERTS
  // OTHERWISE. Dedup would need a per-kernel bitmap over the padded grid, which costs more
  // than the sweep it is describing. `changes / examined` is therefore a work DENSITY, not a
  // fraction, and reads as one.
  uint64_t changes = 0;
  // Cells inside the swept rectangle that the kernel stepped over in a RUN rather than one
  // at a time -- a kernel-internal early-out that skips whole blocks without forming their
  // work. Only `StepConduction`'s quiet-tile test does this today; every other sweep leaves
  // it at zero. It is counted where the skip happens, once per run and not once per cell, so
  // `examined - skipped` is the number of cells the sweep actually stood on. Without it that
  // optimisation is invisible: the rectangle is still fully examined, and how much of it the
  // tile test bought could only be inferred from a stopwatch.
  uint64_t skipped = 0;
  // Times the kernel was entered. A kernel runs once per region per substep, so this is the
  // denominator for "cells per invocation" -- and it is NOT `Profiler::calls`, which only
  // counts while the profiler is switched on.
  uint64_t invocations = 0;
};

// One extra bucket past the last slot: work announced while no sweep is running at all --
// `ModifyCell` from a queued message, an emitter, a building. It is not noise to be dropped.
// Without somewhere to go those announcements would be charged to whichever kernel ran last,
// which is the sort of quiet misattribution that makes a profile agree with itself and lie.
inline Counters g_counts[kSlotCount + 1];

// The sweep currently running on this thread, or `kSlotCount` for none. Set by `ONI_SWEEP`,
// which is a scope and not an assignment, so a kernel that returns early still hands the slot
// back. Saved and restored rather than reset, so nesting -- which no kernel does today -- would
// be counted correctly rather than silently clearing the outer sweep.
inline int g_slot = kSlotCount;

// Grid cells available to be swept, as of the last `SetActiveRegions`. Recorded here rather
// than derived by the reader because the reader (`vftest`, the live profiler) has no World.
// `region_cells` is the sum of the active rectangles' areas -- what a correct sweep is
// allowed to look at once per invocation -- and `grid_cells` the padded grid, which is what
// a sweep that ignored its region would look at instead. The gap between those two numbers
// is the defect class this file exists for.
inline uint64_t g_region_cells = 0;
inline uint64_t g_region_cells_incl = 0;
inline uint64_t g_grid_cells = 0;
inline uint64_t g_regions = 0;

// The cells one invocation of this kernel is allowed to step over, by its declared bound.
inline uint64_t BudgetPerInvocation(int s) {
  switch (SweepBound(s)) {
    case kBoundRegion: return g_region_cells;
    case kBoundRegionInclusive: return g_region_cells_incl;
    case kBoundGrid: return g_grid_cells;
    // Never reached: `WithinBudget` returns early for a slot that is not a cell sweep, and
    // `kBoundNone` belongs to exactly one such slot. The grid is the widest bound there is,
    // so if a future caller ever does ask, it gets the answer that asserts the least.
    case kBoundNone: return g_grid_cells;
  }
  return g_grid_cells;
}

// The one assertion this file is for: a sweep did not step over more cells than its declared
// bound allows, per invocation. Exact integer arithmetic on exact counts -- no tolerance, no
// noise, and nothing to re-record when a machine changes. A kernel that regresses to walking
// the grid trips its own `kBoundRegion` and names itself.
inline bool WithinBudget(int s) {
  const Counters& c = g_counts[s];
  if (!IsCellSweep(s) || c.invocations == 0) return true;
  return c.examined <= BudgetPerInvocation(s) * c.invocations;
}

// A row width, clamped. Written as a function rather than folded into the macro so the
// argument is evaluated exactly once: `r.x1 - r.x0` is cheap, but a macro that evaluates its
// argument twice fails at the first call site that passes something with a side
// effect. The clamp is not defensive noise -- an empty rectangle is representable, and an
// unclamped negative would land in a uint64 as ~1.8e19 and read as a catastrophic scan.
inline void NoteExamined(int slot, int64_t n) {
  if (n > 0) g_counts[slot].examined += static_cast<uint64_t>(n);
}

inline void NoteSkipped(int slot, int64_t n) {
  if (n > 0) g_counts[slot].skipped += static_cast<uint64_t>(n);
}

// A ROW'S WORTH of announced changes at once. Same clamp and same reason as `NoteExamined`,
// and it exists for the same reason the row-granular examined counter does: a kernel whose
// body is small enough that one global read-modify-write per cell shows up in its timing has
// to accumulate in a register and charge once. `StepFieldDecay` (sim/fields.h) is the case
// that forced it -- a per-cell increment there cost 71 % of the pass.
inline void NoteChanges(int slot, int64_t n) {
  if (n > 0) g_counts[slot].changes += static_cast<uint64_t>(n);
}

inline void Reset() {
  for (int s = 0; s <= kSlotCount; ++s) g_counts[s] = Counters();
}

// Charge one announced change to whatever sweep is running. The single call site is
// `World::TouchSubstance`; everything else that wants to count uses `ONI_CHANGED` with an
// explicit slot.
inline void NoteChange() { ++g_counts[g_slot].changes; }

struct SweepScope {
  int prev;
  explicit SweepScope(int s) : prev(g_slot) {
    g_slot = s;
    ++g_counts[s].invocations;
  }
  ~SweepScope() { g_slot = prev; }
  SweepScope(const SweepScope&) = delete;
  SweepScope& operator=(const SweepScope&) = delete;
};

}  // namespace census
}  // namespace oni_sim

// Row-granular, and the argument is the row's width. Written as a macro rather than an
// inline function so that the slot name at the call site reads as the kernel it is inside.
#define ONI_EXAMINED(slot, n) \
  ::oni_sim::census::NoteExamined(::oni_sim::census::slot, (n))

// One announced change, charged to a named slot. Only for kernels that write something
// `TouchSubstance` does not cover -- conduction's temperature, the projection's published
// bytes. Everything that moves substance is counted for free at `World::TouchSubstance`.
#define ONI_CHANGED(slot) (++::oni_sim::census::g_counts[::oni_sim::census::slot].changes)

// A run of cells the kernel skipped without visiting. Counted once per run.
#define ONI_SKIPPED(slot, n) \
  ::oni_sim::census::NoteSkipped(::oni_sim::census::slot, (n))

// A row's worth of announced changes, charged to a named slot in one add. For a kernel whose
// inner loop is too cheap to carry `ONI_CHANGED` per cell -- see `NoteChanges`.
#define ONI_CHANGED_N(slot, n) \
  ::oni_sim::census::NoteChanges(::oni_sim::census::slot, (n))

// The kernel was entered. A SCOPE, declared at the top of the function: it counts the
// invocation and owns `g_slot` for the duration, so `TouchSubstance` calls made anywhere
// underneath -- including out of a helper three files away -- are charged to this kernel
// without that helper knowing anything about the census.
#define ONI_SWEEP(slot) \
  ::oni_sim::census::SweepScope oni_sweep_scope_(::oni_sim::census::slot)
