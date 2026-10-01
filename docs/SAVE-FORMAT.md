# The save blob

`SIM_BeginSave(&size, x, y)` returns a pointer to a blob the simulation still owns. The game
copies `size` bytes, stores them length-prefixed in the save file, and calls `SIM_EndSave`.
`SIM_HandleMessage(Load, size, blob)` takes it back and returns non-null on success.

**Everything depends on this round trip.** Worldgen runs 500 simulation frames per asteroid to
settle it, then calls `Sim.Save` and stores the result; `SaveLoader` feeds each blob back
through `Sim.Load` before a new game starts. A library that cannot round-trip the format cannot
begin a new game, let alone load an old one.

Implementation: `sim/saveblob.h`, and `World::ToBlob` / `World::FromBlob` in `sim/world.h`.
Tests: `driver/src/savefmt.cpp` (`--verify` decodes and re-encodes a synthetic blob and a real
one and requires both to come back byte-identical; `--roundtrip` saves, tears the simulation
down, boots a fresh one through the load path and compares every cell), plus the load arms in
`vftest` and `worldgen_test`.

## The game's format (version 15)

Little-endian throughout. **No padding anywhere**: the blob is byte-packed, unlike the `Pack = 4`
message payloads. No compression.

```
offset  size  field
     0     8  magic "SIMSAVE\0"
     8     4  int32   version           15
    12     4  int32   width             padded: game width + 2
    16     4  int32   height            padded: game height + 2
    20     4  int32   x                 the x passed to SIM_BeginSave
    24     4  int32   y                 the y passed to SIM_BeginSave
    28     1  uint8   unknown; zero in every blob seen
    29        cell[width * height]      16 bytes each
      +       disease[width * height]    8 bytes each
      +       backwall[width * height]  12 bytes each
```

```c
struct SaveCell {      // 16 bytes
  int32_t elementHash;   // SimHashes value, NOT a table index
  float   temperature;   // K
  float   mass;          // kg
  float   radiation;     // rads
};
struct SaveDisease {   // 8 bytes
  int32_t diseaseHash;   // 0 when the cell is clean
  int32_t count;
};
struct SaveBackwall {  // 12 bytes
  int32_t elementHash;
  float   mass;
  float   temperature;
};
```

The total size is exactly `29 + width × height × 36`. A 636 × 404 asteroid is 638 × 406 padded,
259,028 cells, 9,325,037 bytes.

Things worth knowing:

- **Elements are stored by hash, not by table index.** `elementHash` is the `SimHashes` value:
  granite is `-105943486`, oxygen `-1528777920`, vacuum `758759285`. This makes a save
  independent of the element table's order, so a mod that adds elements does not invalidate old
  saves. `diseaseHash` works the same way.
- **The grid is stored padded, border included.** Game cell `(x, y)` is at padded index
  `(y + 1) × width + (x + 1)`. `SaveBlob::Index` does the conversion.
- **Insulation, strength and cell properties are not saved.** The game re-applies them as
  buildings load.
- **Radiation is the fourth float of the cell record.** It is zero in a world without radiation.
- **The world seed and the random stream's position are not saved.**
- **Byte 28** is zero in every blob seen. It is preserved on re-encode rather than assumed.
- **A backwall with no element** (index 0xFFFF, which worldgen sends for a template cell without
  one) is kept as "no element" while the game runs and published as 0xFFFF, but the format has
  no way to say "no element", so it is saved as Vacuum and reloads as a massless Vacuum
  backwall.

## Versions

| version | written by | contents |
|---|---|---|
| 13 | older game builds | 21-byte header without `x`/`y`, 12-byte cells without radiation, the disease array, then 4 ignored bytes per cell; no backwall. The game does not read its border ring, and neither does this library |
| 14 | older game builds | read as 15 |
| 15 | the game, and this library for any world that uses no extension data | the layout above |
| 16, 17 | pre-release builds of this library | 15 plus fixed gas-mixture and room-promotion sections; read, no longer written |
| 18 | pre-release builds of this library | 15 plus the self-describing extension section (below); read, no longer written |
| 19 | this library, for a world with extension data | 18 plus the element palette (below) |

Version 12 and older are refused.

**A world with nothing to say writes plain version 15**, byte-identical to the game's own blob,
which the game's own library must keep accepting. `diffsim` checks both directions. The
extension section is added only once a `kSaved` property has actually been written.

A reader must reject an unknown version loudly. A silently misparsed blob produces a world
that is subtly wrong rather than an error.

### The extension section (version 18 and later)

After the backwall array, a count and one self-describing record per registered `kSaved`
property (see `EXT-REGISTRY.md`):

```
int32                      propertyCount
per property, a 68-byte header, then its bytes:
  char[48]  name           NUL-padded; the permanent on-disk key
  int32     type           ExtScalarType
  int32     arity          components per cell
  int32     stride         bytes per component
  int32     cellCount      must equal the blob's own width × height
  uint32    defaultBits    the property's registered default, per component
  uint8[]   bytes          cellCount × arity × stride, cell-major
```

Adding a property therefore never needs a new save version. `stride` is redundant with `type`
on purpose: it lets a reader skip a record whose type it does not know. `defaultBits` lets a
build that did not register a property (because its mod is uninstalled) carry the bytes
through and still fill any uncovered cell correctly. Records a build does not recognise are
carried through verbatim and written back out.

The gas-mixture layer (`sim.gas_species`, `sim.gas_mass`, the occupancy mask), room promotion
and `sim.dissolved_mass` are all first-party `kSaved` properties in this section.

### The element palette (version 19)

After the extension section:

```
int32     elementCount     the element table this blob was written against
int32[]   elementHash      one SimHashes id per table index, in index order
```

Cells are saved by hash, but the gas mixture's species are table **indices**, and the game sorts
its element table, so an element added or removed anywhere below a gas moves that gas's index.
Without the palette, gas saved under one table and loaded under another comes back as a
different gas or as Vacuum. `Load` maps every saved species index through the palette to the
index its element has now (`World::BuildElementRemap`). A species whose element the loading
table lacks is dropped (the slot is freed and its mass is gone), just as the game loads a cell
of an unknown element as Vacuum; the `Load` handler reports how many slots and kilograms. A
registered property of type `kExtElementIdx` goes through the same palette, and an element the
loading table lacks becomes `kExtNoElement` (0xFFFF).

A version 18 blob has no palette, and its indices are trusted as they are.

## The load path

Booting from a blob is a different message sequence from booting from cells. From
`SaveLoader.Load`:

```
SIM_Initialize
Elements_CreateTable
AllocateCells(width, height)      <- unpadded game dimensions
Disease_CreateTable
ClearUnoccupiedCells
Load(blob)                        <- once per asteroid
Start
```

`SimData_InitializeFromCells`, the worldgen path, needs neither `AllocateCells` nor
`ClearUnoccupiedCells`.

**Where a blob goes.** It is written at its own header `x, y` (the arguments given to
`SIM_BeginSave`) into the grid `AllocateCells` made, border ring and all. A fresh cluster sends
one blob per world, each saved by worldgen at its world's offset. `DefineWorldOffsets` plays no
part in placement.

**What it rewrites.** Every cell taken from the blob, in this order, as the game's own library
does:

| in the blob | after `Load` |
| --- | --- |
| hash `0x0280cf79` / `0x2391c22b` | read as `0x987dac06` / `0x4c76ae31`, two renamed elements |
| temperature NaN or ±infinite | 293 K |
| mass NaN | 100 kg |
| radiation not finite | 0 |
| a hash the element table does not hold | Vacuum, 0 kg, 0 K, 0 rads (a load, not a refusal) |
| any element at or below 0 K, with or without mass | exactly 293 K |
| radiation at or below 0 | 0 |
| Vacuum or Void, any mass and temperature | 0 kg, 0 K, 0 rads |
| more than 3 K outside its element's range | **one** state transition, unless a mod's phase rule refuses it (below) |

`SimData_InitializeFromCells` rewrites nothing. The 0 K rule matters because a world's border
ring is 0 K Neutronium, and on the cluster path that ring lands inside the playable grid.

An unknown element hash does not fail the load. A save that names an element this build does
not have (a removed mod's, say) opens with those cells empty rather than not opening at all.

**One rule the gas-mixture layer changes.** A Vacuum or Void cell that the mixture occupies keeps
the temperature it was saved with (a non-finite value, or one at or below 0 K, still becomes
293 K); its mass and radiation still go to zero. The mixture holds its gas in cells that are
Vacuum in the ordinary layer, and that temperature field is the mixture's temperature. A world
that never used the mixture has no such cell, so its load matches the game's.

**The load-time transition.** A cell more than 3 K outside its element's range moves across one
transition during `Load`: 3 K margin, 1.5 K overshoot, the low side first, no mass gate, no ore,
once. Here it is `LoadTimeStateTransitions` (`sim/physics.h`), a pass over the rectangle just
written. Each transition must also pass `CondensationAllowed` or `BoilingAllowed`, the phase
rules the frame uses. Both only ever refuse, and neither refuses anything for an element no mod
has described, so with no rules registered this is the game's behaviour exactly. It is a
separate pass, not part of the per-cell read, because the boiling rule reads the column above a
cell; rows go bottom-up, so every decision is made against the blob as saved. A Vacuum or Void
cell the mixture holds is skipped.

**Phase rules must reach the simulation before `Load`.** They are element attributes: not
saved, and discarded by every `SIM_Initialize`. A mod that pushes them only at `Game.OnSpawn` is
one `Load` too late, and its cells take the ungated transition. The framework API pushes them in
the registration window that opens before a save's `Load` and before worldgen's settle.

### Known differences from the game's library

Two differences in version 15 loads, neither expected to affect play, but both visible in a byte
comparison:

| where | this library | the game's |
| --- | --- | --- |
| the border ring | Neutronium at **293 K** after a load | 0 K |
| a cell whose disease hash is 0 but whose count is not | count kept | count 0 |

The first is the "at or below 0 K becomes 293 K" rule reaching the ring. Neutronium conducts
nothing.

## A blob is not a checkpoint

The format describes the **world**. It does not describe a **running simulation**, and the gap
matters to anything that restores a state and expects the same future to follow: a replay tool,
a rewind timeline, a test that loads a state and waits for a failure. Both `AllocateCells` and
`Load` reset everything below to fresh values.

To restore a run exactly, carry all of these:

| component | read with | restore with |
|---|---|---|
| the world | `SIM_BeginSave` | `Load` |
| the random stream | `SIM_DebugRandomState` | `kSetRandomState` |
| the scheduling counters | `SIM_DebugSchedulingState` | `kSetSchedulingState` |
| unstable-solid countdowns | `SIM_DebugStableTicks` | `kSetStableTicks` |
| disease growth remainders and infestation ages | `SIM_DebugDiseaseGrowth` | `kSetDiseaseGrowth` |
| the component registries | `SIM_DebugRegistryState` | `kSetRegistryState` |
| checkpoint-only extension properties | `SIM_DebugExtCellState` | `kSetExtCellState` |
| the radiation field | the published `radiation` array | `kSetCellRadiation` |
| the visibility mask | `SIM_DebugVisibilityState` | `kSetVisibilityState` |

and send `kSetLoadIsRestore` **before** the `Load`. It carries no state: it makes that one `Load`
skip the load-time state transition.

With the `CellEnergyCarry` tunable on, each cell's held remainder is not in any of these,
so a restored run starts with nothing held and can differ from the original by up to one
float temperature step per cell that was being paid.

Each of these was found because a restore that omitted it diverged from the original run:

- **Scheduling counters.** `displace_rotation` picks which of several equally good cells receives
  gas that a liquid pushes aside, and `pressure_dir` picks the gas sweep's column order. A
  restore with fresh counters runs the first physics frame in a different order.
- **Stable-tick countdowns.** Each unstable solid carries a one-byte countdown to falling.
  `Allocate` resets them all to the reroll sentinel, and the reroll is the only draw the unstable
  path makes on the random stream, so forgetting them moves sand on the wrong tick *and* puts the
  random stream out of step.
- **Disease growth.** A cell banks a remainder toward its next whole disease unit. The format has
  only the hash and the count, so a restored world regrows from zero. The gap widens with replay
  distance, so it does not show one frame after a checkpoint.
- **Component registries.** `AllocateCells` clears buildings, element chunks, element
  consumers and emitters, and radiation emitters. The game re-registers every handle it holds
  after a load; a tool driving the DLL directly cannot. **Re-sending the recorded registrations
  is not a substitute**: a registration is a handle on a body that evolves every substep
  (temperatures, emitter timers, accumulated state), so re-registering restores the boot-time
  bodies, and an older checkpoint is restored more wrongly.
- **Radiation.** `Load` clears radiation in Vacuum and Void cells, but a running world holds
  radiation there.
- **The visibility mask.** A frame reads the mask the game sent two `PrepareGameData` calls
  earlier, and every allocate and load zeroes it, so a replay restored into a zeroed mask refuses
  the falling liquid its run had handed over.
- **The load-time transition.** A run legitimately publishes cells outside their element's range:
  a supercooled drop that landed this substep freezes on the next. A `Load` moves such a cell one
  transition, so a restore without `kSetLoadIsRestore` starts from a state the run never held.

The registry blob is opaque, written and parsed by the simulation alone. The same state always
writes the same bytes, so the blob can be hashed and compared. It carries each
component's objects **and** the handle table around them: removal swaps with the last element,
so iteration order is observable; a stale handle must keep being detected as stale; and the next
`Add` after a restore must reuse the slot the original run's next `Add` would have. Left out,
deliberately: the radiation occlusion buffer (fully rewritten by every pass that reads it), the
per-frame event lists (outputs no kernel reads back), and the per-substep waste-heat flag.

### Using the restore messages

- **Send them straight after `Load`.** They are queued, so they drain at the start of the next
  frame, ahead of that frame's physics, which is where they belong.
- **Send the captured `skip_physics_frames` verbatim.** `Load` asks for one skipped frame. The
  captured value (0 for a checkpoint taken mid-run) overwrites that, so the frame steps physics
  and lands on the tick after the checkpoint's. Forcing 1 silently drops one physics frame from
  every replay.
- **Do not also replay the registrations.** Queued messages drain in a fixed category order, not
  in send order, and component registrations drain *after* `kSetRegistryState`, so replayed
  `Add`s would pile the boot colony on top of the restored one.
- **`kSetStableTicks` and `kSetDiseaseGrowth` are variable-length.** `kSetStableTicks` is an
  `int32 count` followed by one byte per **padded** cell. `kSetDiseaseGrowth` is an `int32 count`,
  then `count` floats, then `count` bytes. The getters report the length, so a caller never
  derives the padded count itself. A payload whose length disagrees with its count, or whose
  count disagrees with the loaded world, is refused whole rather than applied in part.
- **The published arrays lag the world by a frame.** A tool that compares a published array with
  a `SIM_Debug*` export, which reads the live state, is comparing two different instants.
- **Building-owned arrays start from defaults.** `properties`, `insulation` and `strengthInfo`
  are registrations `AllocateCells` drops and nothing re-sends, so a restored world shows the
  defaults until the game (or the tool) sends them again.

The scheduling message leaves out `first_frame` and `pipelined`, which are publication
bookkeeping: a restore cannot put a worker frame in flight.

### What does not need carrying

**The gas mixture's sleep flags.** Both ends of a pair sleep only after five consecutive ticks in
which every transfer between them fell below the mixer's minimum transfer. A transfer below that
floor is never applied, so nothing accumulates behind it, and nothing can write into a sleeping
pair without waking one end first. Skipping such a pair is exactly equal to running it, so a
restore that wakes every cell reproduces the run bit for bit. `SIM_DebugGasSleeping` exports the
flags so this can be checked.

None of the restore messages changes the simulation until it is sent.
