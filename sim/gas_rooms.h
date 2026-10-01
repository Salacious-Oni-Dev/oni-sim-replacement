// Room pooling for the gas mixture: the part of the mixing cost the dirty/sleep pass in
// gas_mixture.h does not remove.
//
// The game's own room detection lives in managed code, so this is a flood fill of its own
// over the grid.
//
// A "room" is a maximal set of open (non-gas-impermeable) cells connected 4-way. Two open
// cells on opposite sides of a solid wall are never in the same room, and this file never
// builds a mixing pair between them — which is also the only wall-awareness gas mixing has
// anywhere in this codebase: gas_mixture.h's MixPair/MixGridNaive/MixGridDirtySleep have no
// solid check at all. Building the pair list from open-cell connectivity gets that check for
// free instead of adding it to the pair sweeps directly.
//
// The idea beyond plain dirty/sleep: MixGridDirtySleep still visits every cell of the grid
// every tick and tests GasSleeping on both sides of every pair — cheap per pair, but still
// O(grid) every tick even when most rooms have had nothing happen in them for hundreds of
// ticks. A fully-asleep room can be skipped as one block: this file keeps one awake-cell
// counter per room, updated incrementally as individual cells wake/sleep (never rescanned),
// and the room-pooled sweep tests that single counter once per room instead of walking every
// interior cell of a room nothing is happening in.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <queue>
#include <utility>
#include <vector>

#include "../abi/gas_mixture_abi.h"
#include "../abi/sim_abi_ext.h"
#include "gas_mixture.h"
#include "world.h"

namespace oni_sim::gas {

// WHY `IsOpenCell` TESTS THE ELEMENT AS WELL AS THE FLAG. `Properties()`'s `kGasImpermeable` bit is only ever written by `ApplyCellProperties`
// (simdll.cpp), which fires off a distinct message real gameplay (digging, vanilla tile
// placement) happens to always pair with a solidity change -- it is a cached side-table,
// not derived from the element live. `SimMessages.ReplaceElement`, the instant-paint
// primitive mods use to author terrain (a full PhaseEntry replace: element/mass/temperature),
// never sends that second message. A wall painted this way has the right solid *element*,
// so vanilla gas flow stays contained by it, but a never-set `kGasImpermeable` bit. Reading
// the bit alone, the flood fill walks straight through painted walls and can merge separate
// cisterns into one room covering the whole map -- hundreds of thousands of pairs, re-swept
// every mixing tick while any cell in it is awake, which stalls the game.
//
// So the cached bit is not trusted alone. A cell is open only if it is BOTH not marked
// gas-impermeable AND not currently occupying a solid element -- the second check reads
// the same live PhaseEntry/ElementTable data every other solid test in this codebase
// already uses (see e.g. `ZeroMasslessCells`'s `table.Phase(...) == kStateSolid`), so it
// can never go stale independently of the element itself. This is strictly more
// conservative than the old check (either signal being "closed" now closes the cell), so
// it can only ever shrink a room, never grow one past what real gas-impermeability data
// would already justify.
inline bool IsOpenCell(const World& w, const ElementTable& table, size_t padded) {
  if (w.Properties()[padded] & kGasImpermeable) return false;
  return !table.IsSolid(w.Phase(padded).element);
}

// Built by BuildRoomGraph over a `width` x `height` game grid, and rebuilt (see
// simdll.cpp's StepPhysics, `World::RoomsDirty`) whenever a solid boundary opens or closes
// under a live world -- natural phase transitions (`TransitionCell`, physics.h) and
// digging/building (`ApplyCellProperties`, simdll.cpp) both mark that flag. Because the graph
// is rebuilt, `owned` cannot live only on the RoomGraph: the persistent source of truth is a
// per-cell World field (see `World::MutableRoomPromoted`).
// `owned`: 0 = vanilla-owned, 1 = promoted to the new engine, vanilla's own gas kernels
// gated off for this room's cells (see IsRoomOwned's callers throughout sim/physics.h,
// sim/emitters.h and simdll.cpp's ApplyMassEmission/ApplyModifyCell/ApplyMassConsumption).
// Every fresh RoomGraph starts every room at 0 here; BuildRoomGraph itself re-derives the
// real values from World::RoomPromoted right after room_of is fully assigned, so this
// struct's own default is never the last word once a room has ever been promoted.
struct RoomGraph {
  struct Pair { size_t a, b; };

  std::vector<int32_t> room_of;             // padded cell -> room id, -1 if not open
  std::vector<int32_t> awake_count;         // per room: cells not currently GasSleeping
  std::vector<int32_t> room_size;           // per room: total open-cell count
  std::vector<uint8_t> owned;               // per room: 0 = vanilla, 1 = new-engine-owned
  std::vector<Pair> interior_pairs;         // grouped by room, contiguous
  std::vector<size_t> interior_room_start;  // interior_pairs[start[r], start[r+1]) is room r
  std::vector<Pair> boundary_pairs;         // crosses a room boundary; always evaluated

  // THE ROOM'S OWN CELLS, same contiguous-blocks-plus-starts shape as interior_pairs above,
  // and added for the same kind of reason: a caller that wants to walk one room should not
  // have to walk the grid to find it. `room_of` answers "which room is this cell in" and
  // there was no answer at all to the reverse until this pair of vectors existed, so summing
  // a room meant a full O(grid) pass bucketed by room id -- affordable once, absurd per
  // query. BuildRoomGraph's flood fill already visits a room's cells consecutively and to
  // completion before starting the next room, so this costs one push_back per open cell in a
  // pass that was happening anyway, and one int32 per open cell of memory.
  //
  // Entries are PADDED indices, matching interior_pairs/boundary_pairs and `room_of`, not
  // game cell indices.
  std::vector<int32_t> room_cells;           // grouped by room, contiguous
  std::vector<size_t> room_cell_start;       // room_cells[start[r], start[r+1]) is room r

  int32_t RoomCount() const { return static_cast<int32_t>(room_size.size()); }
};

// Trivial accessors, not inlined into call sites directly, so the
// per-kernel gates all read this one way. Bounds are
// the caller's responsibility, same convention as this file's other room
// accessors (e.g. `RoomOnCellSlept`'s `r >= 0` check) rather than asserting
// here.
inline bool IsRoomOwned(const RoomGraph& g, int32_t room_id) {
  return room_id >= 0 && g.owned[static_cast<size_t>(room_id)] != 0;
}
inline void SetRoomOwned(RoomGraph& g, int32_t room_id, bool owned_by_new_engine) {
  if (room_id < 0) return;
  g.owned[static_cast<size_t>(room_id)] = owned_by_new_engine ? 1 : 0;
}

// THE LIQUID KERNELS' GATE, and deliberately NOT the same predicate as `IsRoomOwned` above.
//
// The four gas-side gates (`StepConduction`, `StepGasPressure`, `StepGasDisplacement`,
// `sim/emitters.h`) ask only "is this cell's room promoted", and for gas that is the right
// question: `ApplyPromoteRoom` promotes a room and the caller converts its gas into the
// mixture, after which `ClearCell` leaves every one of those cells at vacuum/zero mass
// (sim/physics.h) -- the vanilla-frozen state. Room promotion and mixture ownership coincide.
//
// They do NOT coincide for liquid, and asking the room question there would be a regression
// rather than a gate. `IsOpenCell` above is `!IsSolid`, so a cell holding water is an open
// cell and `ApplyPromoteRoom` promotes it along with the rest of the room -- but nothing
// converts liquid into the mixture layer (`ApplyMassEmission` lets liquids fall through
// unchanged; the mixture layer is atmosphere-only). So a promoted room's pond still holds real mass in vanilla's own
// `PhaseEntry`, and gating the liquid kernels on room promotion alone would freeze that pond
// in place with NO engine simulating it -- water that stops falling and nothing that makes it
// fall again.
//
// So the liquid gate asks the narrower question that actually expresses the intent: does the
// NEW ENGINE hold mass in this cell? A cell the mixture occupies is protected; a vanilla pond
// inside the same promoted room keeps flowing exactly as it always did. `GasOccupiedMask` is
// always allocated (`World::Allocate`) and reads 0 for every world that never activates
// volume-fractions, so this returns false for all of them before it ever reaches the room
// lookup -- byte-identical, same construction as `IsRoomOwned`'s own `rooms == nullptr` fast
// path.
//
// The room test is kept as well as the mask test, not replaced by it: mixture occupancy
// without promotion is a real state (`ext::kInjectGasSpecies` activates volume-fractions and
// seeds a cell without promoting anything, which several `vftest` sections do), and vanilla
// still owns such a cell. Both must hold.
//
// **OCCUPANCY SPREADS, so "unowned" is a statement about now and not about forever.** Measured
// while building this gate's own negative control, not reasoned about: `MixRoomPooled` carries
// a species outward roughly one cell per tick, so a single `kInjectGasSpecies` into one end of
// a promoted five-cell room had made the next cell along mixture-owned within three ticks, and
// the arm that was supposed to show a vanilla pond flowing freely inside a promoted room
// failed because the pour was legitimately blocked by a cell that had become owned in the
// meantime. Nothing here is wrong -- a cell the mixture now holds mass in SHOULD be gated --
// but the consequence is worth stating plainly: over enough ticks, a promoted room with any
// mixture mass in it converges toward every GAS cell being owned.
//
// **A pond in a promoted room WAS reported frozen, and this gate was only half
// the reason.** The spread above also reached cells holding liquid, because `MixPair` gave to
// any open cell; a pond cell carrying slots is owned, and every liquid kernel refused it. And a
// liquid resting on owned gas could neither swap, fall nor flow into it. Both are closed: the
// mixture is never given to a cell holding liquid (`MixPair`), and liquid meeting mixture gas
// moves the gas aside (`EvictMixture` and the helpers after `RoomOnCellWoke` below) instead of
// stopping. This predicate still means "the new engine holds mass here"; what changed is that
// the liquid kernels now treat such a cell as gas to be displaced, not as a wall.
inline bool IsMixtureOwnedCell(const World& w, const RoomGraph* g, size_t padded) {
  if (g == nullptr) return false;
  if (padded >= g->room_of.size()) return false;
  if (w.GasOccupiedMask(padded) == 0) return false;
  return IsRoomOwned(*g, g->room_of[padded]);
}

// BFS label pass, 4-connected, over open cells only. Runs once per call, not once per tick —
// costs nothing next to a mixing sweep even at tens of thousands of cells.
inline RoomGraph BuildRoomGraph(const World& w, const ElementTable& table, int32_t width,
                                 int32_t height) {
  RoomGraph g;
  const size_t cells = static_cast<size_t>(width) * height;
  g.room_of.assign(w.PaddedCount(), -1);

  std::vector<uint8_t> visited(cells, 0);
  for (int32_t y0 = 0; y0 < height; ++y0) {
    for (int32_t x0 = 0; x0 < width; ++x0) {
      const size_t idx0 = static_cast<size_t>(y0) * static_cast<size_t>(width) + x0;
      if (visited[idx0]) continue;
      const size_t p0 = w.Padded(idx0);
      if (!IsOpenCell(w, table, p0)) {
        visited[idx0] = 1;
        continue;
      }

      const int32_t room_id = g.RoomCount();
      // One start per room, pushed BEFORE the flood fill that fills the block. Room ids are
      // assigned in this same order (room_id == RoomCount(), and room_size gains exactly one
      // entry per room below), so start[r] and room r stay in step without a second index.
      g.room_cell_start.push_back(g.room_cells.size());
      int32_t size = 0;
      std::queue<size_t> q;  // holds unpadded grid index
      q.push(idx0);
      visited[idx0] = 1;
      while (!q.empty()) {
        const size_t idx = q.front();
        q.pop();
        const int32_t x = static_cast<int32_t>(idx % static_cast<size_t>(width));
        const int32_t y = static_cast<int32_t>(idx / static_cast<size_t>(width));
        const size_t p = w.Padded(idx);
        g.room_of[p] = room_id;
        g.room_cells.push_back(static_cast<int32_t>(p));
        ++size;
        auto try_visit = [&](int32_t nx, int32_t ny) {
          if (nx < 0 || ny < 0 || nx >= width || ny >= height) return;
          const size_t nidx = static_cast<size_t>(ny) * static_cast<size_t>(width) + nx;
          if (visited[nidx]) return;
          const size_t np = w.Padded(nidx);
          visited[nidx] = 1;
          if (!IsOpenCell(w, table, np)) return;
          q.push(nidx);
        };
        try_visit(x - 1, y);
        try_visit(x + 1, y);
        try_visit(x, y - 1);
        try_visit(x, y + 1);
      }
      g.room_size.push_back(size);
      g.awake_count.push_back(size);  // everything starts awake, matching a fresh world's
                                       // GasSleeping default of 0 (world.h's Allocate)
      g.owned.push_back(0);           // every room starts vanilla-owned; nothing promotes yet
    }
  }

  // The terminator, so room r's block is always [start[r], start[r+1]) with no special case
  // for the last room -- the same shape interior_room_start uses below.
  g.room_cell_start.push_back(g.room_cells.size());

  // Re-derive `owned` from World's persistent per-cell record
  // (`room_promoted_`, set by ApplyPromoteRoom) rather than leaving it at the all-zero
  // default every fresh RoomGraph starts with. Without this, any rebuild -- geometry
  // changing under a live world, not just the very first BuildRoomGraph a game ever runs
  // -- would silently un-promote every room, since room ids are only stable between
  // rebuilds and `owned` used to be the room graph's own field with no memory past it. A
  // room merges or splits here exactly the way ordinary flood-fill connectivity already
  // does; this only decides whether the pieces come out promoted, by asking each of their
  // cells individually rather than trusting a room id that may not even be the same id as
  // before.
  for (size_t p = 0; p < g.room_of.size(); ++p) {
    const int32_t r = g.room_of[p];
    if (r >= 0 && w.RoomPromoted(p)) g.owned[static_cast<size_t>(r)] = 1;
  }

  // Second pass: classify every right/down neighbor pair (same walk MixGridNaive uses) as
  // interior (both cells same room) or boundary (different rooms). A pair with either side
  // not open is dropped entirely — never filed as boundary — since MixPair on a solid cell
  // would read/claim slots on a cell nothing ever seeds.
  std::vector<std::vector<RoomGraph::Pair>> by_room(static_cast<size_t>(g.RoomCount()));
  for (int32_t y = 0; y < height; ++y) {
    for (int32_t x = 0; x < width; ++x) {
      const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(width) + x;
      const size_t p = w.Padded(idx);
      const int32_t rp = g.room_of[p];
      if (rp < 0) continue;
      if (x + 1 < width) {
        const size_t pr = w.Padded(idx + 1);
        const int32_t rr = g.room_of[pr];
        if (rr == rp) by_room[static_cast<size_t>(rp)].push_back({p, pr});
        else if (rr >= 0) g.boundary_pairs.push_back({p, pr});
      }
      if (y + 1 < height) {
        const size_t pd = w.Padded(idx + static_cast<size_t>(width));
        const int32_t rd = g.room_of[pd];
        if (rd == rp) by_room[static_cast<size_t>(rp)].push_back({p, pd});
        else if (rd >= 0) g.boundary_pairs.push_back({p, pd});
      }
    }
  }
  g.interior_room_start.assign(by_room.size() + 1, 0);
  for (size_t r = 0; r < by_room.size(); ++r) {
    g.interior_room_start[r] = g.interior_pairs.size();
    for (const auto& pr : by_room[r]) g.interior_pairs.push_back(pr);
  }
  g.interior_room_start[by_room.size()] = g.interior_pairs.size();
  return g;
}

// Sleep is caller policy (same division MixPair/MixGridDirtySleep already use — MixPair only
// ever wakes, the caller decides when a cell has earned sleep). Call this the moment the
// caller flips a cell's GasSleeping to 1, so `awake_count` never needs a rescan.
inline void RoomOnCellSlept(RoomGraph& g, size_t padded) {
  const int32_t r = g.room_of[padded];
  if (r >= 0 && g.awake_count[static_cast<size_t>(r)] > 0) {
    --g.awake_count[static_cast<size_t>(r)];
  }
}
inline void RoomOnCellWoke(RoomGraph& g, size_t padded) {
  const int32_t r = g.room_of[padded];
  if (r >= 0) {
    const size_t ur = static_cast<size_t>(r);
    if (g.awake_count[ur] < g.room_size[ur]) ++g.awake_count[ur];
  }
}

// ------------------------------------------------------------------ liquid meets the mixture
//
// A cell whose PhaseEntry holds liquid (or anything condensed) must not also hold mixture
// slots. The mixture reuses `PhaseEntry.temperature` as its shared temperature, so a liquid in
// the same cell would have to share it, and every liquid kernel refuses such a cell
// (`IsMixtureOwnedCell`). Refusal alone is not enough, because two paths reached it:
// liquid over mixture gas could neither swap, fall nor flow into it, and `MixPair` spread the
// mixture into a promoted room's pond cells so the pond itself stopped moving. The helpers
// below are how liquid now gets through: the gas steps aside, as vanilla's `DisplaceGas` makes
// a vanilla gas step aside.
inline bool HoldsCondensedMass(const World& w, const ElementTable& table, size_t padded) {
  return table.Phase(w.Phase(padded).element) >= kStateLiquid;
}

inline void WakeMixtureCell(World& w, RoomGraph* g, size_t padded) {
  w.MutableGasDirty(padded) = 1;
  if (!w.GasSleeping(padded)) return;
  w.MutableGasSleeping(padded) = 0;
  if (g != nullptr) RoomOnCellWoke(*g, padded);
}

// Moves every mixture slot of `cell` into one neighbour of the same room, heat and all, and
// leaves `cell` with none. True when `cell` holds no slots afterwards (including when it held
// none to begin with); false when no neighbour could take them, and then nothing moved.
//
// The neighbour order is `DisplaceGas`'s: the four orthogonals starting at `rotation` (up,
// left, right, down), then the two upward diagonals. A neighbour qualifies when it is in the
// same room, holds nothing condensed, is not gas-impermeable, and has free slots for every
// species `cell` would bring it. Vacuum and mixture cells both qualify; so does a vanilla gas
// cell, which then carries both layers, a state `MixPair` already accounts for.
//
// The heat goes with the mass, as in `MixPair`: the slots leave at `cell`'s temperature, which
// does not change, and the neighbour settles to its new heat over its new heat capacity. The
// part of that settle landing on the neighbour's vanilla mass is booked on the ledger the same
// way `MixPair` books it.
inline bool EvictMixture(World& w, const ElementTable& table, RoomGraph* g, size_t cell,
                         uint16_t rotation) {
  const uint8_t mask = w.GasOccupiedMask(cell);
  if (mask == 0) return true;
  if (g == nullptr || cell >= g->room_of.size()) return false;
  const int32_t room = g->room_of[cell];
  if (room < 0) return false;
  const int32_t pw = w.PaddedWidth(), ph = w.PaddedHeight();
  const int32_t cx = static_cast<int32_t>(cell % static_cast<size_t>(pw));
  const int32_t cy = static_cast<int32_t>(cell / static_cast<size_t>(pw));
  const std::vector<uint8_t>& props = w.Properties();

  auto fits = [&](size_t c) {
    int free_slots = 0;
    for (int s = 0; s < kMaxSpeciesPerCell; ++s) {
      if (w.GasSpecies(c, s) == kEmptySpecies) ++free_slots;
    }
    for (int s = 0; s < kMaxSpeciesPerCell; ++s) {
      if (!(mask & (1u << s))) continue;
      if (FindSlot(w, c, w.GasSpecies(cell, s)) >= 0) continue;
      if (--free_slots < 0) return false;
    }
    return true;
  };
  size_t target = 0;
  bool have_target = false;
  auto consider = [&](int32_t x, int32_t y) {
    if (have_target) return;
    if (x < 1 || x >= pw - 1 || y < 1 || y >= ph - 1) return;
    const size_t c = static_cast<size_t>(y) * static_cast<size_t>(pw) + static_cast<size_t>(x);
    if (c >= g->room_of.size() || g->room_of[c] != room) return;
    if (HoldsCondensedMass(w, table, c)) return;
    if (props[c] & kGasImpermeable) return;
    if (!fits(c)) return;
    target = c;
    have_target = true;
  };
  const int32_t ox[4] = {0, -1, 1, 0};
  const int32_t oy[4] = {1, 0, 0, -1};
  for (int k = 0; k < 4; ++k) {
    const int r = (rotation + k) & 3;
    consider(cx + ox[r], cy + oy[r]);
  }
  for (int k = 0; k < 2; ++k) {
    const int r = (rotation + k) & 1;
    consider(r == 0 ? cx - 1 : cx + 1, cy + 1);
  }
  if (!have_target) return false;

  const PhaseEntry& to = w.Phase(target);
  double hc_to = to.mass > 0.0f
                     ? static_cast<double>(to.mass) * table.At(to.element).specificHeatCapacity
                     : 0.0;
  const uint8_t to_mask = w.GasOccupiedMask(target);
  for (int s = 0; s < kMaxSpeciesPerCell; ++s) {
    if (!(to_mask & (1u << s))) continue;
    hc_to += static_cast<double>(w.GasMass(target, s)) *
             table.At(w.GasSpecies(target, s)).specificHeatCapacity;
  }
  double energy_to = hc_to * static_cast<double>(to.temperature);
  const double t_from = static_cast<double>(w.Phase(cell).temperature);
  for (int s = 0; s < kMaxSpeciesPerCell; ++s) {
    if (!(mask & (1u << s))) continue;
    const uint16_t species = w.GasSpecies(cell, s);
    const float mass = w.GasMass(cell, s);
    const int slot = FindOrClaimSlot(w, target, species);  // `fits` guaranteed a slot
    w.MutableGasMass(target, slot) += mass;
    const double moved_hc = static_cast<double>(mass) * table.At(species).specificHeatCapacity;
    hc_to += moved_hc;
    energy_to += moved_hc * t_from;
    w.MutableGasMass(cell, s) = 0.0f;
    w.MutableGasSpecies(cell, s) = kEmptySpecies;
  }
  w.MutableGasOccupiedMask(cell) = 0;
  w.MutableGasDirty(cell) = 1;
  if (hc_to > 0.0) {
    const double grid_before = GridCellEnergy(w, table, target);
    w.Phase(target).temperature = static_cast<float>(energy_to / hc_to);
    w.NoteEmittedEnergy(GridCellEnergy(w, table, target) - grid_before);
  }
  WakeMixtureCell(w, g, target);
  w.MarkProjectDirty(cell);
  w.MarkProjectDirty(target);
  return true;
}

// Exchanges the mixture slots of two cells, for a swap of their PhaseEntries (`SwapCells`):
// the shared temperature travels inside the PhaseEntry, so the slots must travel with it or
// they would be left at the other cell's temperature. The two cells are neighbours in one
// room, so the room's awake count is unchanged by exchanging their sleep flags.
inline void SwapMixture(World& w, size_t a, size_t b) {
  for (int s = 0; s < kMaxSpeciesPerCell; ++s) {
    std::swap(w.MutableGasSpecies(a, s), w.MutableGasSpecies(b, s));
    std::swap(w.MutableGasMass(a, s), w.MutableGasMass(b, s));
  }
  std::swap(w.MutableGasOccupiedMask(a), w.MutableGasOccupiedMask(b));
  std::swap(w.MutableGasSleeping(a), w.MutableGasSleeping(b));
  w.MutableGasDirty(a) = 1;
  w.MutableGasDirty(b) = 1;
}

// The sweep that keeps the rule for every path the kernels do not see: a liquid a ModifyCell
// put on top of mixture gas, or a save written by an earlier build whose pond cells already carry
// slots. Once per mixing tick, before `TickRoomPooledMixing`; O(cells) mask reads, the same
// as that function's own two passes. A cell with nowhere to put its gas keeps it until a
// later tick finds room.
inline void EvictMixtureFromCondensedCells(World& w, const ElementTable& table, RoomGraph& g,
                                           uint16_t rotation) {
  for (size_t c = 0; c < w.GameCount(); ++c) {
    const size_t p = w.Padded(c);
    if (w.GasOccupiedMask(p) == 0) continue;
    if (!HoldsCondensedMass(w, table, p)) continue;
    EvictMixture(w, table, &g, p, rotation);
  }
}

// One pair, with the room's awake_count kept in sync: MixPair only ever clears GasSleeping
// (wakes), never sets it, so a 1->0 transition here is unambiguously a wake this pair caused.
inline void MixRoomPair(World& w, const ElementTable& table, RoomGraph& g, size_t a, size_t b,
                         float rate) {
  const bool a_was_asleep = w.GasSleeping(a) != 0;
  const bool b_was_asleep = w.GasSleeping(b) != 0;
  if (a_was_asleep && b_was_asleep) return;  // same per-pair skip MixGridDirtySleep uses
  MixPair(w, table, a, b, rate);
  if (a_was_asleep && !w.GasSleeping(a)) RoomOnCellWoke(g, a);
  if (b_was_asleep && !w.GasSleeping(b)) RoomOnCellWoke(g, b);
}

// A room with awake_count == 0 skips its entire interior-pair block with one counter read —
// no per-pair GasSleeping test, no per-cell array touch at all. Boundary pairs always run,
// since they are the only path a sleeping room can be woken through from outside it.
inline void MixRoomPooled(World& w, const ElementTable& table, RoomGraph& g, float rate) {
  for (int32_t r = 0; r < g.RoomCount(); ++r) {
    if (g.awake_count[static_cast<size_t>(r)] == 0) continue;
    const size_t begin = g.interior_room_start[static_cast<size_t>(r)];
    const size_t end = g.interior_room_start[static_cast<size_t>(r) + 1];
    for (size_t i = begin; i < end; ++i) {
      MixRoomPair(w, table, g, g.interior_pairs[i].a, g.interior_pairs[i].b, rate);
    }
  }
  for (const auto& pr : g.boundary_pairs) MixRoomPair(w, table, g, pr.a, pr.b, rate);
}

// One full mixing tick: reset dirty, run MixRoomPooled, then apply the sleep policy (a cell
// that produced no dirty flag for `sleep_threshold` consecutive ticks is parked; MixPair/
// MixRoomPair wake one immediately on a real transfer — see gastest.cpp's benchmark comments
// for the same division of responsibility). `stable` is a per-game-cell counter the caller
// owns and keeps across ticks, sized to `w.GameCount()`.
//
// Real integration point (sim/simdll.cpp's StepPhysics) and gastest.cpp's benchmarks both
// need this exact sequence; kept here as one function so the two can never quietly diverge in
// behavior — the benchmark's measured numbers are only meaningful if they measure what
// actually ships.
inline void TickRoomPooledMixing(World& w, const ElementTable& table, RoomGraph& g, float rate,
                                  std::vector<uint8_t>& stable, int sleep_threshold) {
  for (size_t c = 0; c < w.GameCount(); ++c) w.MutableGasDirty(w.Padded(c)) = 0;
  MixRoomPooled(w, table, g, rate);
  for (size_t c = 0; c < w.GameCount(); ++c) {
    const size_t p = w.Padded(c);
    if (w.GasDirty(p)) {
      stable[c] = 0;
    } else if (stable[c] < 255) {
      if (++stable[c] >= sleep_threshold) {
        if (!w.GasSleeping(p)) RoomOnCellSlept(g, p);
        w.MutableGasSleeping(p) = 1;
      }
    }
  }
}


// ------------------------------------------------------------------ the room, summed
//
// The read side of the room graph, and the reason `room_cells` above exists. `OniRoomAggregate`
// is defined in `abi/sim_ext_api.h` (the published C header) and aliased by `abi/sim_abi_ext.h`;
// the field-by-field contract lives there and is not restated here. What lives here is the
// arithmetic and the two decisions a reader of the struct cannot see:
//
//   * A CELL IS COUNTED IN THE PRESSURE MEAN ONLY IF IT HOLDS MIXTURE STATE. `CellPressure` is
//     PV=nRT over the mixture's own species slots, so a cell with an empty occupied-mask has no
//     mixture pressure -- not a pressure of zero. Averaging a zero in for it would make a room
//     read as emptier the less of it the mixture layer has been asked to own, which is exactly
//     backwards, so the denominator is `mixtureCellCount` and the caller is told what it was.
//
//   * TEMPERATURE IS WEIGHTED BY THE CELL'S WHOLE MASS, both layers. `PhaseEntry.temperature` is
//     one field per cell and the mixture layer blends incoming mass INTO it (see
//     ApplyInjectGasSpecies's mass-weighted blend) rather than keeping a second temperature, so
//     the honest weight for that one number is everything the cell holds. A massless cell is
//     skipped rather than contributing its temperature at weight zero.
//
// No allocation, no barrier of its own (the caller holds one), and O(cells in the room).
static_assert(ONI_ROOM_MAX_SPECIES >= kMaxSpeciesPerCell,
              "a room's composition needs room for at least one cell's worth of species");

inline bool ComputeRoomAggregate(const World& w, const ElementTable& table, const RoomGraph& g,
                                  int32_t room_id, OniRoomAggregate* out) {
  if (!out) return false;
  // Zeroed and marked no-room BEFORE anything can fail, so every refusal path below leaves the
  // same unmistakable struct rather than whatever the caller's stack happened to hold.
  std::memset(out, 0, sizeof(*out));
  out->roomId = -1;
  if (room_id < 0 || room_id >= g.RoomCount()) return false;
  const size_t r = static_cast<size_t>(room_id);
  // A graph built before `room_cells` existed, or one whose vectors disagree, answers no room
  // rather than indexing into a block that is not there.
  if (r + 1 >= g.room_cell_start.size() || r >= g.room_size.size() ||
      r >= g.awake_count.size() || r >= g.owned.size()) {
    return false;
  }

  out->roomId = room_id;
  out->cellCount = g.room_size[r];
  out->awakeCount = g.awake_count[r];
  out->owned = g.owned[r] != 0 ? 1 : 0;

  // Distinct-species overflow needs to remember what it has already counted, and it cannot
  // allocate to do it. This bound is deliberately far above anything reachable: vanilla has
  // about ten gases in total, and ONI_ROOM_MAX_SPECIES already holds sixteen. If a room ever
  // does hold more than `ONI_ROOM_MAX_SPECIES + kOverflowTracked` distinct species, the count
  // degrades to counting occurrences rather than distinct species -- stated here so that it is
  // read rather than discovered.
  constexpr int kOverflowTracked = 48;
  uint16_t overflow_seen[kOverflowTracked];
  int overflow_seen_count = 0;

  float weighted_temperature = 0.0f;
  float temperature_weight = 0.0f;
  float pressure_sum = 0.0f;

  const size_t begin = g.room_cell_start[r];
  const size_t end = g.room_cell_start[r + 1];
  for (size_t i = begin; i < end; ++i) {
    const size_t p = static_cast<size_t>(g.room_cells[i]);
    const PhaseEntry& phase = w.Phase(p);
    out->vanillaMassKg += phase.mass;
    float cell_mass = phase.mass;

    const uint8_t mask = w.GasOccupiedMask(p);
    if (mask != 0) {
      ++out->mixtureCellCount;
      pressure_sum += CellPressure(w, table, p);
      for (int slot = 0; slot < kMaxSpeciesPerCell; ++slot) {
        if (!(mask & (1u << slot))) continue;
        const uint16_t species = w.GasSpecies(p, slot);
        const float mass = w.GasMass(p, slot);
        out->mixtureMassKg += mass;
        cell_mass += mass;

        int found = -1;
        for (int k = 0; k < out->speciesCount; ++k) {
          if (out->species[k] == species) {
            found = k;
            break;
          }
        }
        if (found >= 0) {
          out->massBySpeciesKg[found] += mass;
          continue;
        }
        if (out->speciesCount < ONI_ROOM_MAX_SPECIES) {
          out->species[out->speciesCount] = species;
          out->massBySpeciesKg[out->speciesCount] = mass;
          ++out->speciesCount;
          continue;
        }
        // Out of slots. The mass still went into mixtureMassKg above -- it is never lost, only
        // unattributed -- and the species is counted once.
        bool already = false;
        for (int k = 0; k < overflow_seen_count; ++k) {
          if (overflow_seen[k] == species) {
            already = true;
            break;
          }
        }
        if (already) continue;
        if (overflow_seen_count < kOverflowTracked) {
          overflow_seen[overflow_seen_count++] = species;
        }
        ++out->speciesOverflow;
      }
    }

    if (cell_mass > 0.0f) {
      weighted_temperature += cell_mass * phase.temperature;
      temperature_weight += cell_mass;
    }
  }

  if (out->mixtureCellCount > 0) {
    out->meanPressurePa = pressure_sum / static_cast<float>(out->mixtureCellCount);
  }
  if (temperature_weight > 0.0f) {
    out->temperatureK = weighted_temperature / temperature_weight;
  }
  return true;
}

// ------------------------------------------------------------------ promote and absorb
//
// A room handed to the mixture WITH its gas (ext::kSetBlockedGasAddPolicy).
// `kPromoteRoom` on its own only flips the room's bit, and leaves converting the room's
// vanilla gas to whoever sent it. Every reader of a promoted room (breathing, the overlays,
// the mixing pass, `IsMixtureOwnedCell`) ignores the vanilla grid there, so a room promoted
// without its gas moved across reads as empty. When the SIM promotes a room on its own
// initiative, no managed caller is standing by to do that half, so it is done here, in the
// same step.
//
// Per open cell of the room:
//   * a GAS cell's whole vanilla mass moves into its own mixture slot for that element, and
//     the cell is left Vacuum at zero mass -- the vanilla-frozen shape `ClearCell` leaves in a
//     promoted cell. The temperature field is NOT cleared: it is the mixture's one shared
//     temperature (see gas_mixture_abi.h), so the gas keeps the temperature it had. Germs stay
//     in vanilla's fields, as they do for every promoted cell.
//   * a cell whose eight slots are already full of OTHER species keeps its vanilla gas where it
//     is. Nothing is deleted to make room.
//   * liquid and vacuum cells are untouched: promotion is atmosphere-only.
//
// The mass leaves the vanilla ledger through `consumed`, and its energy through
// `consumed` energy, the same two buckets `kRemoveVanillaMass` books -- the vanilla half of
// the managed `ConvertFromVanilla` pair this does natively. Returns the kilograms moved.
//
// Promotion is written to `World::RoomPromoted` per cell as well as to `owned`, for the reason
// `ApplyPromoteRoom` gives: a later geometry change rebuilds the graph from that per-cell
// record. The room's own cell list is walked, so this is O(room), not O(grid).
inline float PromoteRoomAndAbsorbGas(World& w, const ElementTable& table, RoomGraph& g,
                                     int32_t room) {
  if (room < 0 || room >= g.RoomCount()) return 0.0f;
  SetRoomOwned(g, room, true);
  const uint16_t vacuum = table.VacuumIndex();
  float moved = 0.0f;
  const size_t begin = g.room_cell_start[static_cast<size_t>(room)];
  const size_t end = g.room_cell_start[static_cast<size_t>(room) + 1];
  for (size_t i = begin; i < end; ++i) {
    const size_t p = static_cast<size_t>(g.room_cells[i]);
    w.MutableRoomPromoted(p) = 1;
    PhaseEntry& c = w.Phase(p);
    if (table.Phase(c.element) != kStateGas || !(c.mass > 0.0f)) continue;
    const int slot = FindOrClaimSlot(w, p, c.element);
    if (slot < 0) continue;
    const float m = c.mass;
    w.MutableGasMass(p, slot) += m;
    w.MutableGasDirty(p) = 1;
    w.NoteConsumed(m);
    w.NoteConsumedEnergy(-static_cast<double>(m) *
                         static_cast<double>(table.At(c.element).specificHeatCapacity) *
                         static_cast<double>(c.temperature));
    c.element = vacuum;
    c.mass = 0.0f;
    w.MarkProjectDirty(p);
    w.TouchSubstance(p);
    if (w.GasSleeping(p)) {
      w.MutableGasSleeping(p) = 0;
      RoomOnCellWoke(g, p);
    }
    moved += m;
  }
  return moved;
}

}  // namespace oni_sim::gas
