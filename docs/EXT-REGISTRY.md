# The extension registries

How a mod stores its own data in the simulation without an ABI change: per-cell properties, per-
element attributes, per-frame event streams, and fields built on per-cell properties.

Code: `abi/sim_abi_ext.h` (the blocks headed "THE PER-CELL EXTENSION REGISTRY", "per-element
attributes", "event streams" and "THE GENERIC FIELD SOLVER"), `sim/ext_registry.h`,
`sim/ext_elements.h`, `sim/ext_events.h`, `sim/ext_state.h`, `sim/fields.h`, and the handlers and
exports in `sim/simdll.cpp`. Tests: `driver/src/vftest.cpp` and `driver/src/gastest.cpp`.

---

## 1. What it replaces

Without a registry, every number a mod wants per cell costs a message id, a payload struct, a
member array in `World`, a line in `World::Allocate`, a handler, and, to survive a save, a new
save-format field and a new save version. Two further problems matter more than the edit count.

### 1.1 Persistence must be a stated decision

An ad-hoc array makes persistence an implicit choice that nobody has to state, review or notice,
and the first mod to rely on it finds its values gone after a save and load. So persistence is a
**required** field of registration with no default, and a registration that does not choose one
is refused.

### 1.2 Save versions are a global, ordered namespace

The game's format is version 15. If every saved array claimed the next integer, two mods that
each add a property independently would collide by construction, and neither could be written
without knowing about the other. The registry's save section is self-describing (§7), so adding a
property never needs a new save version.

---

## 2. Persistence classes

| class | meaning | in the save blob | in a checkpoint |
|---|---|---|---|
| `kSaved` | the simulation carries it across save and load | yes | no |
| `kRehydrated` | the owner re-pushes it after every load | no | yes |
| `kCheckpointOnly` | not saved, and not re-pushed by anyone, but needed to replay a run | no | yes |

A checkpoint (see `SAVE-FORMAT.md`) carries the two classes the save blob does not. `kSaved`
properties are not also carried in the checkpoint, because the checkpoint would then restore them
a second time and win.

`sim.thermal_mass_bonus` is `kCheckpointOnly`: the conduction kernel reads it, so a replay that
lacked it would conduct differently, and a tool driving the DLL directly has no mod to re-push it.

---

## 3. Registering a per-cell property

```c
ExtScalarType   kExtF32 | kExtU8 | kExtU16 | kExtI32 | kExtElementIdx   (4 is unused)
ExtPersistence  kSaved | kRehydrated | kCheckpointOnly                   // required
int32_t arity                                                            // components per cell
```

`kRegisterCellProperty` is an **immediate** message. Registration has to close before the first
`World::Allocate`, and both `AllocateCells` and `SimData_InitializeFromCells` are immediate, so a
queued registration would drain a frame after the world it was meant to size. It returns the
property index as a `const int32_t*`, or a negated `ExtRegisterResult`.

### 3.1 Naming

`<owner>.<property>`: both segments `[a-z0-9_]`, the owner 2–31 characters, the property 1–31,
and the whole name at most 47 characters, so it fits `name[48]` with its terminator. (31 + 1 + 31
is legal by segment and does not fit; it is refused with that reason.)

Every violation is a refusal with its own code, because "registration failed" is not actionable:

| refusal | code | why a refusal and not a repair |
|---|---|---|
| uppercase or a bad shape | `kExtRegisterBadName` | lower-casing would map `Contamination` and `contamination` onto one store |
| already registered | `kExtRegisterDuplicate` | logged naming **both** owners |
| `sim.` or `oni.` from a third party | `kExtRegisterReserved` | so a first-party key is visibly first-party in a blob |
| a name filling all 48 bytes | `kExtRegisterBadName` | refused, not truncated: the name is the permanent on-disk key |
| after the first allocate | `kExtRegisterClosed` | refused, not serviced by a resize: every per-cell array has the same length for the life of a world |
| a type or persistence outside the enums | `kExtRegisterBadType` | |
| arity below 1 or above `kExtMaxArity` | `kExtRegisterBadArity` | |
| the registry is full | `kExtRegisterFull` | |

The framework API composes the owner segment from the identity a mod already declares, so a mod
cannot register under another mod's prefix by accident. The native side enforces only the shape.

### 3.2 Arity: components per cell

A property can hold several components per cell, up to `kExtMaxArity` (64, a sanity bound, not
a design limit). The gas mixture's species and masses are arity 8.

**The layout is cell-major:** component `c` of cell `p` is at `(p × arity + c) × stride`.

- **`component` is required** on every write and read, never defaulted to 0. A default would let
  a caller write an arity-8 property without naming a lane and land silently on lane 0.
- **An out-of-range component is dropped, not clamped.** Folding component 8 onto lane 0 would
  corrupt a real value.
- **A default is per component.** An arity-8 property with a default starts with all eight lanes
  filled.

**If the property's shape on disk differs from the shape registered** (type, arity or stride),
the load refuses it by name rather than reinterpreting the bytes (§7.3).

---

## 4. `kRehydrated` is a claim, and the claim is checkable

`kRehydrated` means "the simulation must not carry this; I re-push it after every load". The
registry counts message writes per property since the last allocate, so
`SIM_ExtOutstandingRehydration` answers the question that matters after a load: which properties
was somebody supposed to re-push, and has not?

A checkpoint restore also counts: a property the checkpoint restored is not listed, even though
no message wrote it. That uses a separate `restored_by_checkpoint` flag, not the write counter,
because a restore is not a message write, and giving one field two meanings is how persistence
gaps start.

---

## 5. Exports

| export | returns |
|---|---|
| `SIM_ExtCellPropertyIndex(name)` | the index by name, or -1 |
| `SIM_ExtReadCellProperty(cell, prop, component, outBits)` | one component of one cell, as raw bits zero-extended from the stride |
| `SIM_ExtOutstandingRehydration(out, max)` | §4 |
| `SIM_ExtCellPropertyCount()` | how many properties are registered, so the registry can be enumerated |
| `SIM_ExtCellPropertyDescribe(idx, out)` | one registration (§10b); 1 on success, and 0 leaves `out` untouched |
| `SIM_ExtEventStreamIndex(name)` | a declared stream's index, or -1; stable for the life of the DLL |
| `SIM_ExtEventStreamCount()` | how many streams this DLL declares |
| `SIM_ExtEventStreamDescribe(idx, out)` | one stream's declaration (§11b) |
| `SIM_ExtPublishedProperties(frame, count)` | the per-frame property table (§10) |
| `SIM_ExtPublishedEvents(frame, count)` | the per-frame event table (§11) |
| `SIM_ExtElementAttributeIndex(name)`, `SIM_ExtElementAttribute(...)` | one element attribute (§12) |
| `SIM_ExtElementAttributeCount()`, `SIM_ExtElementAttributeDescribe(...)`, `SIM_ExtElementAttributeKeys(...)` | attribute discovery (§12b) |
| `SIM_ExtLiquidPayloadUnreported(prop, component, out)` | a liquid-carried amount that left the grid in a record nobody took (§14) |
| `SIM_ExtFieldCount()`, `SIM_ExtFieldIndex(propertyName)`, `SIM_ExtFieldDescribe(idx, out)` | field discovery (§15) |
| `SIM_DebugExtCellState(out, cap)` | the checkpoint blob (§9) |
| `SIM_DebugExtCellStateAll(out, cap)` | every class, for inspection (§9.2b) |

Extension properties are **not projected** into `GameDataUpdate`. The read-back exports and the
published table are the only way to observe a property directly.

---

## 6. `sim.thermal_mass_bonus`

The first property is first-party: `sim.thermal_mass_bonus`, F32, arity 1, `kCheckpointOnly`. It
adds to a cell's heat capacity in conduction. It is registered in `World`'s constructor, so it is
index 0 in every process and a mod's registration order cannot move it.

`kSetCellThermalMassBonus` is a compatibility alias that writes it through the registry.

The conduction kernel reads the bonus through a raw pointer cached in `World`, refreshed by
`Allocate` and by nothing else. That is safe because `Allocate` is the only thing that sizes
extension storage, and registration closes at the same moment.

---

## 7. The save section

A blob that carries extension data is version 18 or later, with one self-describing section
holding a count and one record per registered `kSaved` property. The byte layout is in
`SAVE-FORMAT.md`. In brief: a 48-byte name, then type, arity, stride, cell count and default,
then the bytes, cell-major.

- `stride` is stored even though `type` implies it, so that a reader can **skip** a record whose
  type it does not know.
- `persist` is **not** stored: it describes the registration, not the data, and a blob contains
  `kSaved` properties by definition.

Version 19 appends the element palette, one hash per element-table index, so that element
indices stored in the blob can be mapped to the loading table. `sim.gas_species` goes through it,
and so does any property of type **`kExtElementIdx`**: two bytes per component, like `kExtU16`,
but the type tells `Load` to rewrite every component through the palette. An element the loading
table lacks, or an index past the palette's end, becomes `kExtNoElement` (0xFFFF), and the `Load`
handler reports how many. The remap works from the record, not the registration, so an orphaned
record (§7.4) is remapped too: it is written back under the current table's palette on the next
save.

### 7.1 One section, not a chain of versions

The gas mixture and room promotion once had fixed sections of their own, at versions 16 and 17.
They are first-party `kSaved` properties now:

| property | type | arity |
|---|---|---|
| `sim.gas_occupied_mask` | u8 | 1 |
| `sim.gas_species` | u16 | 8 |
| `sim.gas_mass` | f32 | 8 |
| `sim.room_promoted` | u8 | 1 |
| `sim.dissolved_mass` | f32 | 8 |

Versions 16 and 17 are still read. They are not written.

Every section gate in the format is `>=`, never `==`, so **a version number is a cumulative
claim**. Stacking a new version on 16 and 17 would have meant writing their sections, empty or
not, into every later blob. `SaveBlobFixedSize` gates them `>= 16 && < 18`, and the `<` half is
what stops the chain growing.

### 7.2 Absent, not empty

A world with no gas mixture, no promoted room and no written extension property writes a plain
**version 15** blob, byte-identical to the game's own format, even though several `kSaved`
properties are registered in every world. `HasSavedData()` looks for any component that differs
from its registered default.

`diffsim` checks this from both sides:

```
  blobs are byte identical
  klei loads mine: accepted     <- the game's decoder, on this library's output
  mine loads klei: accepted
```

A version 18 blob with a count of zero would break that for every world in the game.

Once the section exists, every `kSaved` property goes into it, including those still at their
default: one rule for whether the section exists, not one per property.

### 7.3 The three mismatch cases

`ext_state::ResolveRecord` is shared by every load path, so they cannot disagree:

| case | meaning | what happens |
|---|---|---|
| the blob has a property this build did not register | a mod was **removed** | reported, not fatal; the bytes are carried through verbatim (§7.4) |
| this build registered a property the blob lacks | a mod was **added** | nothing: `Allocate` already filled it with its default |
| the type, arity or stride differs | a mod changed its own property's shape | **refused, naming the property**; never reinterpreted |

### 7.4 An uninstalled mod's data survives

A blob record that no registration claims is kept verbatim in `World::ext_orphans_` and written
back out on the next save. The case is a player turning a mod off for one session: without this,
that session's first save would destroy the mod's data permanently and silently.

- **An orphan forces the section to exist**, so re-saving does not drop it to write version 15.
- **`defaultBits` is in the record for this case.** A build that registered a property knows its
  default; a build that did not has no value but zero to fill a cell the blob does not cover.
- **Orphans are appended after the registered records**, which is stable across repeated round
  trips.

Saving again with the mod still absent is byte-identical, and reinstalling the mod recovers the
exact values it saved.

### 7.5 Both load paths

`FromBlob` restores a property wholesale. The sub-region path, where a small blob lands at an
offset inside a larger cluster grid, restores it **per cell**, because it maps the blob's local
index onto a different global one.

---

## 8. Old save versions

`tests/fixtures/` holds version 15, 16, 17 and 18 blobs written by earlier builds. `vftest` loads
each through the current reader and checks the contents: the version 16 mixture and the version
17 room promotion come back intact and re-save as the current version, and a world with nothing
to say still writes version 15 byte for byte. The fixtures matter because nothing in the tree can
write versions 16 and 17 any more: a test that generated its own old blobs would be checking the
new writer against the new reader.

---

## 9. The checkpoint component

`kSetExtCellState` / `SIM_DebugExtCellState`; the format is in `sim/ext_state.h`. The
authoritative list of what a complete checkpoint contains is the `kSetExtCellState` block in
`abi/sim_abi_ext.h`, and `SAVE-FORMAT.md` explains each item.

### 9.1 Why extension arrays belong in a checkpoint

The conduction kernel reads the thermal-mass bonus inside its pair loop. A property missing from
a replay does not read as missing: it silently produces a different result.

### 9.2 Which classes travel

| class | in the save blob | in the checkpoint |
|---|---|---|
| `kSaved` | yes | no |
| `kRehydrated` | no | yes |
| `kCheckpointOnly` | no | yes |
| orphans | yes | no |

The two carried classes are handled identically. The difference between them is declarative: it
is what lets `SIM_ExtOutstandingRehydration` report who promised to re-push and has not.

A checkpoint record naming a property this build now registers as `kSaved` is **refused**, naming
the property. Otherwise the save blob and the checkpoint would both restore it.

### 9.2b Reading every class: `SIM_DebugExtCellStateAll`

The checkpoint blob answers "what must a checkpoint carry". A tool inspecting a world wants every
property, including the simulation's own `kSaved` ones. `SIM_DebugExtCellStateAll` serves all
three classes in the same format, with the same two-call sizing (call with capacity 0 to learn the
size). It is **read-only by construction**: no message accepts its output, and
`kSetExtCellState` refuses a `kSaved` record by name. Orphans are not included.

It costs a worker barrier and a full serialisation, so poll it deliberately rather than every
frame. The framework API exposes it as `SimExtCellState`.

### 9.3 Absent means "was default", never "do not touch"

Properties still entirely at their default are **omitted** from the checkpoint blob: a checkpoint
is often taken on a ring and paid for on every seek. On restore, a carried property the blob does
not mention is **reset to its default**, not left alone; leaving it would keep a value the
checkpoint never saw.

---

## 10. The per-frame publish table

The read-back exports cost a call per cell and a worker barrier per call. For a mod that reads a
whole array every frame, a per-frame table is the other shape:

```c
struct ExtPublishedProperty {
  char        name[48];
  int32_t     type, persist, arity, stride, cellCount, byteCount;
  const void* data;      // VALID FOR THIS TICK ONLY
};

const ExtPublishedProperty* SIM_ExtPublishedProperties(const GameDataUpdate* frame,
                                                       int32_t* count);
```

Bind once per tick and read through a span, with no call per cell.

### 10.1 Three decisions

**The data are copies, not the registry's storage.** With the frame on a worker thread (see
`THREADING.md`), the kernels are writing the registry while the game reads the frame it was
given, so each published property is copied into the alternating `PublishedFrame` like every
array `GameDataUpdate` names. That copy is what makes the pointer valid for the whole tick.

**Publishing is opt-in per property** (`kPublishCellProperty`), because the copy is paid every
frame. Nothing is published until a mod asks, and the cost lands on the mod that asked. The
subscription is queued, so it takes effect on the **next** frame. It survives an `Allocate` and a
load: it is a statement about the world, not about one allocation of it.

**The caller passes the frame it is holding.** A "give me the current table" signature would have
to read the most recently finished frame, which the worker replaces mid-tick. Resolving the table
from the frame pointer the game already holds is the only provably safe form. A frame this DLL
never published is a refusal: null, with `*count = 0`. A live frame with no subscriptions is
**non-null** with `*count = 0`. The two are different facts.

`GameDataUpdate` itself is untouched: its layout belongs to the game. Cell counts in the table
are **padded** cells.

### 10b. Describing a registration

The published table is a read window and carries no default. A consumer that renders a property
needs one, since zero is a legal value of every scalar type, and "which cells are still at the
default" is not answerable from the bytes alone. The registration comes from
`SIM_ExtCellPropertyDescribe`, fetched once per world:

```c
struct ExtCellPropertyDesc {          // 76 bytes
  char     name[48];
  int32_t  type, persist, arity, stride;
  int32_t  cellCount;                 // PADDED cells; 0 before World::Allocate
  uint32_t defaultBits;
  int32_t  published;                 // non-zero if subscribed
};
```

| | export | barrier | fetched |
|---|---|---|---|
| the bytes | `SIM_ExtPublishedProperties` | none | every tick |
| the registration | `SIM_ExtCellPropertyDescribe` | `WaitIdle` | once per world |

The barrier is needed because registration appends to the registry until the first allocate.
`cellCount` is 0 for a property registered but not yet sized, and `published` changes a frame
after `kPublishCellProperty` is sent, because that message is queued.

---

## 11. Per-frame event streams

`GameDataUpdate` carries ten per-frame event lists as count-and-pointer pairs. Its layout is the
game's, so an eleventh has nowhere to go. A registered stream has the same shape without the
fixed list: a name, a record stride and a published count-and-pointer pair. Adding a native event
type is one `Register` and one `Emit`.

### 11.1 Three decisions

**Streams are declared natively; there is no registration message.** An event is something the
simulation observed, so whatever produces it declares it, and every producer is a kernel inside
this DLL. A mod gets discovery (`SIM_ExtEventStreamIndex`), subscription (`kSubscribeEventStream`)
and reading (`SIM_ExtPublishedEvents`).

**Subscription gates collection, not only the copy.** An unsubscribed stream buffers nothing, so
`Emit` costs a load and a branch, and it is safe to put on a hot path. A record produced before
anyone subscribed cannot be retrieved afterwards.

**Records are cleared at the end of the publish, not at the start of the frame.** This DLL can
refuse a message on the game thread, between frames. Cleared at the start of the frame, as the
game's own lists are, that record would be wiped before any publish carried it. Clearing after
the copy means every record reaches exactly one published frame.

Each stream has a per-frame byte cap (`kExtStreamBytesPerFrame`, 1 MB) and a published `dropped`
count, so a runaway producer cannot grow a buffer without bound, and a truncated list says so.

A record reaches the game **two** ticks after it is produced, whichever path produced it:
`PrepareGameData` returns the frame the worker has already finished and only then starts the
next one.

### 11.2 `sim.message_refused`

This DLL drops a malformed message without applying it. The `sim.message_refused` stream makes
that visible:

| reason | where | covers |
|---|---|---|
| `kExtRefusalShortPayload` | the payload template every queued handler uses | a payload shorter than its struct |
| `kExtRefusalUnknownMessage` | the unknown-id path | the queued and the immediate paths |
| `kExtRefusalBadTarget` | the handlers named in the header | an unregistered property, an out-of-range cell or component, and the specific refusals listed at each message |

`kExtRefusalBadTarget` is reported only by the handlers the header lists. **Every other handler's
semantic refusal is still silent**, so no record does not always mean the message applied.

An `UnknownMessage` record is not always a fault. The game sends `SetWorldZones` on every load,
and this DLL deliberately does not implement it, because the game's own library produces the same
output with or without it. So a record for id `-457308393` is expected.

### 11b. Describing a stream

The published table lists only **subscribed** streams, so it cannot be the discovery path: "no
such stream" and "nobody has subscribed yet" would look the same. `SIM_ExtEventStreamCount` and
`SIM_ExtEventStreamDescribe` answer instead:

```c
struct ExtEventStreamDesc {
  char    name[48];     // NUL-terminated, as declared
  int32_t stride;       // bytes per record, fixed at declaration
  int32_t subscribed;   // non-zero if collecting; survives an Allocate and a load
  int32_t declaredIdx;  // the index SIM_ExtEventStreamIndex returns for `name`
};
```

A record's layout is not described here: `stride` gives the size, and decoding a record needs to
know which stream it came from (`ExtRefusedMessage` for `sim.message_refused`).

| what | export | barrier | how often |
|---|---|---|---|
| the records | `SIM_ExtPublishedEvents` | none | every tick |
| how many streams | `SIM_ExtEventStreamCount` | none | once |
| one declaration | `SIM_ExtEventStreamDescribe` | `WaitIdle` | once per world |

The count needs no barrier because the declarations are fixed inside `SIM_Initialize`, before the
worker exists. `Describe` does, because `subscribed` is written by the drain on the worker; read
without the barrier it can report the previous frame's answer.

**The flag changes one frame before the published table does.** A subscription drains during
tick N, so `subscribed` reads true on tick N, while the table tick N publishes was built from the
subscriptions at the start of that frame. The stream appears in the table on tick N + 1.

---

## 12. Per-element attributes

`kRegisterElementAttribute` and `kSetElementAttribute` store data per element rather than per
cell. Native kernels keep typed accessors for what they read on every cell; the open,
string-keyed table is for what the simulation only stores and hands back, or reads through a
typed accessor that resolves the index once.

### 12.1 Four ways it differs from the per-cell registry

Each follows from one fact: an element attribute is **content data**, not world state.

**Nothing is saved, and there is no persistence field.** The game's element table is reloaded
every session and is not in the save blob, and an attribute of an element is the same kind of
thing. Saving one would let a stale value from an old version of a mod outrank that mod's current
table. So every attribute is re-pushed by its owner after every load, and there is no field in
which to claim otherwise.

**There is no default, and "unset" is a real answer.** Per-element storage is sparse: most
elements have no value for most attributes. The fallback for an unset attribute is per-element
knowledge the registry does not hold (`sim.molecular_mass` falls back to the element's own molar
mass). `SIM_ExtElementAttribute` reports set or not, and leaves the caller's buffer untouched when
it is not. **A stored 0 is a value, not an absence**, so clearing is an explicit flag on the
message.

**Registration never closes.** Nothing here is sized to the world, so a mod that loads late can
still register, and reloading a world disturbs nothing. `kExtRegisterClosed` is unreachable on
this path.

**Both messages are immediate.** `SIM_HandleMessage` already waits for the worker before any
immediate message, so the write is serialised at no extra cost, and data pushed at load is
readable on the same tick.

Keys are `SimHashes` ids, not table indices, so a value survives the element table being reloaded
and means the same thing against any table.

### 12.2 `sim.molecular_mass`

The first attribute is first-party: `sim.molecular_mass`, g/mol. The game stores the atomic mass
for diatomic gases (oxygen 15.9994, hydrogen 1.00794), so the simulation seeds corrected values
for the four diatomic gases at construction. `kSetMolecularMass` is an alias that writes this
attribute. `ElementTable::MolecularMassOf` is its typed accessor: the index is resolved once, and
the per-cell pressure path uses no string lookup. Seeding happens once, at construction, and
reloading the element table does not clear anything a mod pushed.

### 12.3 What is and is not a refusal

Writes report refusals on `sim.message_refused`. Registrations do not: a registration returns its
refusal as the value the caller must read anyway to learn the index.

| case | answer |
|---|---|
| a short `kSetElementAttribute` payload | `kExtRefusalShortPayload` |
| a write to an unregistered attribute index | `kExtRefusalBadTarget` |
| a write to a component outside the declared arity | `kExtRefusalBadTarget` |
| a clear of an element that had no entry | **not a refusal**: the caller asked for unset and got unset |
| any registration refusal | the negated `ExtRegisterResult`, and a log line |

The clear case keeps an idempotent teardown loop from filling the stream with its own success.

### 12b. Discovering attributes

```c
struct ExtElementAttributeDesc {   // 80 bytes
  char     name[48];     // NUL-terminated, as registered
  int32_t  type;         // ExtScalarType
  int32_t  arity;        // components per element
  int32_t  stride;       // bytes per component
  int32_t  valueCount;   // ELEMENTS carrying a value, not the element count
  int32_t  declaredIdx;  // the index SIM_ExtElementAttributeIndex returns for `name`
  int32_t  firstParty;   // non-zero if the simulation registered it
  uint64_t writes;       // message writes since process start, clears included
};

int32_t SIM_ExtElementAttributeCount(void);
int32_t SIM_ExtElementAttributeDescribe(int32_t attrIdx, ExtElementAttributeDesc* out);
int32_t SIM_ExtElementAttributeKeys(int32_t attrIdx, int32_t* out, int32_t max);
```

There is deliberately no persistence field and no default (§12.1). `valueCount` shows the
sparseness. `writes` tells apart the two meanings of `valueCount == 0`: nobody has pushed the
attribute yet, or its owner pushed and then cleared everything.

`Keys` returns the **total** number of keys rather than the number copied, so one call with
`max` 0 sizes the buffer and a second fills it (`out` may be null only when `max` is 0). Keys come
back in ascending order. Walking the loaded element table instead would not do: a mod may hold a
value for a hash the loaded table has no row for, and that value would look absent.

None of the attribute exports takes a barrier. Element data is written only through immediate
messages, which already wait for the worker, and is read-only while a frame runs.

---

## 13. The managed API

The framework API (`oni-framework-api`) wraps all three registries:

| type | covers |
|---|---|
| `SimExtCellProperties` | registering, writing, publishing and reading per-cell properties; rehydration; discovery |
| `SimExtElementAttributes` | registering, writing and reading element attributes |
| `SimExtEventStreams` | subscribing, discovery, and decoding `ExtRefusedMessage` |
| `SimExtFrame` | the per-tick bind to the published property and event tables |
| `SimExtRegistrar` | *when* a mod may register |
| `SimExtRegistry` / `ExtOwner` | the name space, and who owns a prefix |

### 13.1 The registration window

Registration must close before the first `World::Allocate`, but a mod **cannot** register from
`UserMod2.OnLoad`: at that point there is no simulation. `SIM_Initialize` destroys any previous
simulation and creates a new one, and `SIM_HandleMessage` returns immediately when there is none.
The window opens at `SIM_Initialize` and closes at the allocate a few calls later. Because
`SIM_Initialize` runs on every load, **the window reopens on every load, and every registration
must be made again**. A property index is valid for one simulation instance, not for the process.

`SimExtRegistrar` opens the window at the right moment.

### 13.2 Two allocate routes

The game allocates with `Sim.AllocateCells`. Code that builds a world from cells uses
`SimMessages.SimDataInitializeFromCells`, a different message that allocates and seeds in one
step. A registrar that hooks only one of them misses the other half of worlds. `SimExtRegistrar`
hooks both.

---

## 14. Liquid-carried properties and dissolved gas

By default a property is **static**: its value stays on its cell whatever the cell's contents do.
One transport mode moves it with liquid.

- **`kSetCellPropertyTransport`** (queued, Store) sets an F32 property to
  `kTransportFollowsLiquidMass`; any other type is refused. Every liquid kernel then moves the
  property in proportion to the liquid it moves: flow, cell swaps, same-liquid density mixing,
  directional merge, pressure displacement and simple displacement.
- **When liquid leaves the grid, its share leaves on a stream**; nothing is simply dropped.
  - `sim.liquid_payload_released` (`LiquidPayloadReleased`, 24 bytes) carries a game cell and a
    reason: Cleared, PhaseChange, OffGas, Falling, Wisp, Replaced, Removed, Deleted, Massless or
    Effervescence.
  - `sim.liquid_payload_consumed` (`LiquidPayloadConsumed`, 24 bytes) covers element consumers
    and `MassConsumption`. A consumer with no callback releases its share instead.
  - A record no subscriber takes is added to the property's unreported total, which
    `SIM_ExtLiquidPayloadUnreported` reads back. The total counts since the DLL was initialised
    and is not saved.
- **`kAddCellPropertyAmount`** (queued, Store) adds to one component on the simulation's side of
  the queue. A read-modify-write through `kSetCellProperty` would lose one of two adds in flight.
  The add is refused whole, never clamped, in four cases: a result below zero, a non-finite
  amount, a property that is not F32, or a liquid-carried add into a cell with no liquid.
- **The first user** is the first-party `sim.dissolved_mass` (F32, `kSaved`, arity
  `kDissolvedGasLanes` = 8), which the simulation registers and makes liquid-carried itself. Which
  gas owns which lane is managed content. A world that dissolves nothing is byte-identical,
  because the transport code sits behind a gate that stays false until a value is written.

`vftest` checks that `grid + released + consumed + unreported == injected` on every tick.

`kSetPayloadMixing`, `kSetDissolvedTint` and `kSetEffervescence` build on the same property: how
fast a dissolved amount evens out across still water, how it tints the liquid, and when
supersaturated liquid releases it as bubbles. See `EXTENSION-POINTS.md` §3.2.

---

## 15. Fields: a property with a solver

A field **owns no storage**. It is an already-registered `kExtF32` arity-1 cell property with a
propagation rule attached. So it inherits everything above: naming, duplicate refusal,
persistence class, the save section (§7), the checkpoint (§9), the publish table (§10) and
rehydration accounting (§4). The rule and its sources are **not** saved. They are content, and
their owner re-pushes them after every load, as with element attributes.

- **`kRegisterField`** (immediate, Store) takes the property index, an optional attenuating
  element attribute with a fallback for elements where it is unset, a law, a combine mode, a
  decay mode and a clamp. It returns the field index, or a negated `ExtRegisterResult`
  (`kExtRegisterBadProperty`, `kExtRegisterBadAttribute` or `kExtRegisterBadRule`). It can arrive
  at any time. There is at most one field per property, and at most `kExtMaxFields` (32) in all.
  - Laws: `kAttenFlat`; `kAttenRadiationMass`, the radiation absorption law; and
    `kAttenLightMass`, the sunlight step, in which a solid takes its whole factor.
  - Combine: `kCombineTransmission` (as radiation: `t *= 1 - a`) or `kCombineExposure` (as
    sunlight: `e -= a`).
  - Decay: `kDecayNone` (the default; a field with no sources then runs no whole-grid pass) or
    `kDecayFactor`.
- **`kSetFieldSource`** (queued, Parameter) adds, replaces or removes one source of one field,
  keyed by the owner's own `sourceId`; `strength == 0` removes it. The kinds are `kSourcePoint`,
  `kSourceDirectional` and `kSourceElement`. A bad index, cell, world or kind is refused on
  `sim.message_refused`.
- **`StepFields`** runs after `StepRadiationEmitters`, and only when at least one field is not
  idle (see `EXTENSION-POINTS.md` §2.2).
- **Two invariants**, both checked in `gastest` against deliberately broken rules. A rule may read
  any cell but writes only inside the region being stepped, which keeps region-parallel stepping
  possible. And a rule never draws from the shared random stream, so registering a field cannot
  shift the game's own randomness.
- **Reference checks:** a field with the radiation law reproduces `RadiationAbsorptionAlongLine`
  bit for bit, and one with the light law reproduces `ComputeSunBeam`. The game's own radiation
  and sunlight are not moved onto the solver; they stay as they are and serve as the reference.
