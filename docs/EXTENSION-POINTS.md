# Extension points

What a mod may hand the simulation, when the simulation reads it, and what it may affect.

This page describes an ABI surface. The phase ordering in §2 and the per-message table in §3
are promises to mods, not descriptions of current behaviour that happen to be true today:
changing them is a breaking change, treated like changing a struct layout. `abi/sim_abi_ext.h`
is the machine-readable half, and the exports in §2.5 and §3.2 let a mod read both lists at
run time.

---

## 1. Why data, and not callbacks

The obvious design for a simulation API is a hook: a function pointer, or a managed delegate,
that the simulation calls from inside a step (`OnBeforeTemperatureTransfer`, say). This library
deliberately does not offer one. The cost is not implementation time:

1. **It ends the compatibility check.** Physics changes are accepted on one criterion: with no
   extension in use, `diffsim` finds no difference from the game's own library. A simulation
   that calls arbitrary third-party code cannot be compared against anything.
2. **It ends determinism.** Two players with identical colonies and identical mod lists diverge
   on any callback that reads the wall clock, depends on iteration order or reads uninitialised
   state. The game saves the grid, so divergence means a corrupt save, not a visual glitch.
3. **It rules out parallelism for good.** Stepping regions in parallel requires proving that a
   step has no cross-region side effects. Nothing can be proven about an opaque callback.
4. **Crashes land in native frames.** A mod's null dereference surfaces as an unattributed
   access violation inside `SimDLL`.

Instead, every extension is **declarative data that a native phase evaluates**: a mod
registers a parameter, a property or a source, and a named kernel reads it while stepping.
That is comparable, deterministic and parallelisable.

A published data ABI can be widened later. A published callback ABI cannot be withdrawn.

### 1.1 The cost of publishing an order

Naming the order is itself a commitment: once it is published, the order kernels run in is a
promise rather than an implementation detail. That is why §2 is as careful about what it
refuses to guarantee as about what it guarantees. The refusals keep region-parallel stepping,
variable substep counts and gated phases possible.

---

## 2. The frame, as a published ordering

### 2.1 The frame

One frame (`RunFrame` in `sim/simdll.cpp`):

```
ClearFrameEvents
DrainQueue                <- phase 0. Applies the messages of the tick BEFORE this one.
StepPhysics(elapsed)      <- runs 0..N substeps; the body of §2.2
UpdateBackwallTransitions
Project                   <- phase 24. What the game will see.
FillPropertyTextures
PromoteWorldOffsets
BuildUpdate
```

Conduit handles freed during the previous tick are released on the game thread just before
the frame starts (see `THREADING.md`).

Two ordering facts sit outside the phase list and matter just as much:

* **A message takes effect on the tick after it is sent.** The frame drains the queue of the
  tick before it. A caller that sends and then reads back in the same tick reads the old value.
* **Messages drain in category order, not arrival order**, as in the game's own library. A
  `ModifyCellEnergy` always lands before a `ModifyCell` from the same frame, whichever order the
  game sent them in.

### 2.2 The substep body

`StepPhysics` derives a substep count from the elapsed time and runs the body that many times.
Inside a substep, the body runs **once per region**, except where noted:

```
  per region:
     1  StepWorldEnvironment         no-op unless the world has a kSetWorldEnvironment record
     2  StepConduction
     3  StepStateChange
     4  StepGasPressure
     5  StepGasDisplacement          off the SAME snapshot as 4
     6  StepFlow
     7  StepLiquidDisplacement       off the SAME snapshot as 6
     8  StepPayloadMixing            gated: a liquid-carried property is nonzero and mixes
     9  StepDiseaseDiffusion         gated: disease active
    10  StepRadiationField
    11  StepElementConsumers
    12  StepElementEmitters
    13  StepRadiationEmitters
    14  StepFields                   gated: at least one non-idle field registered
    15  StepElementChunks
    16  StepBuildingHeatExchange
    17  StepBuildingToBuilding
    18  StepDiseaseEmitters
    19  StepPostProcess
    20  ZeroMasslessCells            WHOLE GRID, inside the per-region loop
    21  StepDiseasePostProcess       gated: disease active
  once per substep, outside the region loop:
    22  TickRoomPooledMixing         gated: the gas-mixture layer is active
    23  StepEffervescence            gated: kSetEffervescence enabled it
```

These are the indices `SIM_ExtPhaseDescribe` returns (`ONI_EXT_PHASE_LIST`, 25 phases, 0..24).

Phases 11–13 and 15–18 are the game's component list, in the game's order. The disease emitter
comes last, which is why it runs after both building exchanges rather than beside the other
emitters.

Phases that are not part of the game's own simulation:

- `StepWorldEnvironment` (1) applies a world's solar heating, ground re-radiation and surface
  atmosphere boundary (`sim/environment.h`). A world with no environment record returns at once.
- `StepPayloadMixing` (8) evens out a liquid-carried amount, such as dissolved gas, across the
  body of liquid that holds it (`sim/payload_mix.h`). It runs straight after the two liquid
  sweeps, so it reads the liquid after it has moved this substep.
- `StepFields` (14) is the generic field solver (`sim/fields.h`). It sits beside the radiation
  emitter because it is the general form of the same walk. A world with no field registered
  returns on its first line.
- `TickRoomPooledMixing` (22) is the gas-mixture layer (`GAS-MIXTURES.md`).
- `StepEffervescence` (23) releases dissolved gas from supersaturated liquid. It needs each
  cell's whole column, and a column crosses regions, so it runs once per substep.

`ZeroMasslessCells` (20) is the one phase whose scope is neither per region nor per substep: it
sits inside the region loop but sweeps the whole grid, so a world with N regions covers every
cell N times per substep. It has to run after post-processing; moving it earlier changes the
result. The ABI gives it a scope value of its own for that reason.

### 2.3 What is guaranteed

* The order above is the order phases run in, **within one substep, within one region**.
* **Pairwise order is the promise; the absolute index is not.** If A runs before B in one build,
  it runs before B in the next. A phase inserted between two others does not change that for
  any existing pair, but it shifts every index after it, and that is a MINOR version bump, not a
  MAJOR one. A mod that stored an index from one build and compares it against another has
  stored the wrong thing; §2.5 is how to ask, and the name is the key.
* Each phase's declared scope says how often it runs relative to a frame.

### 2.4 What is explicitly *not* guaranteed

Each omission is deliberate.

* **Order across regions.** Region A's conduction and region B's conduction have no defined
  order relative to each other, and nothing may depend on one. This keeps region-parallel
  stepping possible.
* **How many substeps a frame runs.** The count is derived from the elapsed time, with a
  remainder carried to the next frame. A frame may run zero, one or several. Anything that must
  happen once per frame belongs in the drain or the publish, not in a substep phase.
* **That a gated phase runs at all.** A world with no disease never runs phases 9 and 21; a world
  that never uses the gas-mixture layer never runs 22; a frame with no substeps runs none of
  1–23. "It did not run" is a normal outcome, not an error.
* **Any order within a phase.** The order a kernel visits cells in is its own business.

### 2.5 Reading the list at run time

```c
int32_t SIM_ExtPhaseCount(void);
int32_t SIM_ExtPhaseDescribe(int32_t phaseIdx, ExtPhaseDesc* out);
```

`ExtPhaseDesc` carries the kernel's name, its own index echoed back, an `ExtPhaseScope` and an
`ExtPhaseGate` bitmask:

| scope | meaning |
|---|---|
| `kPhaseScopeFrame` | once per frame |
| `kPhaseScopeSubstep` | once per substep, over the whole world |
| `kPhaseScopeRegion` | once per region per substep |
| `kPhaseScopeGridInRegion` | once per region per substep, over the whole grid (`ZeroMasslessCells`) |

| gate | meaning |
|---|---|
| `kPhaseGateNone` | runs every frame |
| `kPhaseGateSubstep` | runs only in a frame that has substeps |
| `kPhaseGateWorld` | also needs a world condition, such as disease being active |

An out-of-range index or a null pointer returns 0 and **leaves the buffer untouched**, so a
caller that ignores the return value does not find a plausible descriptor in it.

These two exports read no world state and take no worker barrier. They can be called **before
`SIM_AllocateCells`**, so a mod can check at load time that the phase list it was built against
matches the DLL it loaded. The framework API wraps them as `SimExtPhases`.

### 2.6 The list cannot drift

The enum, the name table in `simdll.cpp` and the `static_assert` all expand one macro,
`ONI_EXT_PHASE_LIST` in `abi/sim_abi_ext.h`. `driver/src/vftest.cpp` holds the other half: a
copy of the order **typed out by hand**, deliberately not generated from the macro, because a
check generated from its subject passes by construction. Swapping two phases in the macro, or
adding a phase to the step body without publishing it, fails `vftest` and names every index
that moved.

---

## 3. The messages

### 3.1 Four kinds of message

The extension message ids (`ONI1` onward) are not all one kind of thing. Each carries a
published class:

| class | count | what it is |
|---|---|---|
| **Parameter** | 17 | data a named phase reads while stepping |
| **Operation** | 3 | an action applied once during the drain |
| **Store** | 9 | the extensible storage: declare a per-cell or per-element array, write it, open a read window or an event stream |
| **Checkpoint** | 9 | restores run state a save blob cannot carry. Not an extension point at all |

So of 38 ids, 20 change the simulation's behaviour (Parameter and Operation), 9 are the storage
they are built on, and 9 are a checkpoint ABI that shares the header and the drain.

### 3.2 The table

`ONI_EXT_MESSAGE_LIST` in `abi/sim_abi_ext.h` carries every column below except "affects", and
`SIM_ExtMessageCount` / `SIM_ExtMessageDescribe` export it, so a mod can ask rather than read.
**The header is the authority; this table is the prose edition.** `driver/src/gastest.cpp`
holds a hand-typed copy that fails if the header changes without it.

Delivery is **Q** (queued: drains at the start of the next frame, §2.1) or **I** (immediate:
applied inside `SIM_HandleMessage`, on the calling thread, before the call returns).

| id | message | class | del. | read by | affects |
|---|---|---|---|---|---|
| ONI1 | `kSetCellThermalMassBonus` | Parameter | Q | `StepConduction` | per-cell heat capacity, on top of mass × specific heat |
| ONI2 | `kInjectGasSpecies` | Operation | Q | drain; then `TickRoomPooledMixing` | adds species mass to the gas-mixture layer |
| ONI3 | `kRemoveVanillaMass` | Operation | Q | drain | removes single-element mass from a cell |
| ONI4 | `kConvertToVanillaMass` | Operation | Q | drain | mixture mass to single-element mass, refunded on refusal |
| ONI5 | `kPromoteRoom` | Parameter | Q | the room gates in the gas kernels, the emitters and the liquid kernels | which rooms the mixture layer owns |
| ONI6 | `kSetInvertedGravityElement` | Parameter | Q | `StepFlow` | one element falls up |
| ONI7 | `kSetMolecularMass` | Parameter | **I** | the mole and pressure path | g/mol; an alias for the `sim.molecular_mass` element attribute |
| ONI8 | `kSetBuildingWasteHeatKilowatts` | Parameter | Q | `StepBuildingHeatExchange` | power-to-heat rate, summed with the operating rate |
| ONI9 | `kSetBuildingExhaust` | Parameter | Q | `StepBuildingHeatExchange` | exhaust kW and its ceiling; the undelivered remainder goes to the building's own body |
| ONI: | `kSetBuildingRadiation` | Parameter | Q | `StepBuildingHeatExchange` | emissivity and radiating area |
| ONI; | `kSetEnvironmentTemperature` | Parameter | Q | `StepBuildingHeatExchange` | the radiative sink temperature |
| ONI< | `kSetRandomState` | Checkpoint | Q | — | the RNG state every draw comes from |
| ONI= | `kSetSchedulingState` | Checkpoint | Q | — | substep carry, pressure direction, skipped frames |
| ONI> | `kSetStableTicks` | Checkpoint | Q | `StepPostProcess` | per-cell stability countdown |
| ONI? | `kSetDiseaseGrowth` | Checkpoint | Q | `StepDiseasePostProcess` | per-cell growth remainder and infestation age |
| ONI@ | `kSetRegistryState` | Checkpoint | Q | — | all four component registries, as one opaque blob |
| ONIA | `kRegisterCellProperty` | Store | **I** | — | declares a per-cell array; closed at the first `Allocate` |
| ONIB | `kSetCellProperty` | Store | Q | whoever registered it | writes one cell and component of one |
| ONIC | `kSetExtCellState` | Checkpoint | Q | — | restores the registered arrays |
| ONID | `kPublishCellProperty` | Store | Q | `Project` | opens a per-frame read window on a property |
| ONIE | `kSubscribeEventStream` | Store | Q | frame build | opens a per-tick event stream |
| ONIF | `kRegisterElementAttribute` | Store | **I** | — | declares a per-element attribute |
| ONIG | `kSetElementAttribute` | Store | **I** | whichever path reads that attribute | writes one element's value |
| ONIH | `kSetBuildingConvection` | Parameter | Q | `StepBuildingHeatExchange` | convection between a building body and its cells; also stops that building's own conduit applying the cell ratio twice |
| ONII | `kSetWorldEnvironment` | Parameter | Q | `StepWorldEnvironment` and `StepBuildingHeatExchange` | per-world planet: radiative sink temperature, solar flux and absorptivity, and the surface atmosphere boundary. Every field is inert at ≤ 0 |
| ONIJ | `kSetWorldSun` | Parameter | Q | the sun-beam pass in `textures.h`, then the solar terms | per-world sun direction; switches on a direct-beam exposure field beside the vertical sunlight texture. Read back with `SIM_ExtWorldSun` / `SIM_ExtCopySunBeam` |
| ONIK | `kSetCellRadiation` | Checkpoint | **I** | `StepRadiationField` | the whole radiation field, which a load clears in Vacuum and Void cells |
| ONIL | `kSetBlockedGasAddPolicy` | Parameter | Q | the `ModifyCell` drain | a blocked gas add promotes its room and mixes, instead of deleting the smaller mass as the game does; default is the game's behaviour |
| ONIM | `kSetCellPropertyTransport` | Store | Q | every liquid mover | makes an F32 cell property move with liquid. When liquid leaves the grid, its share leaves on `sim.liquid_payload_released` / `sim.liquid_payload_consumed` |
| ONIN | `kAddCellPropertyAmount` | Store | Q | drain | adds to one component of an F32 property. A negative result, a non-finite amount, or a liquid-carried add into a cell with no liquid is refused |
| ONIO | `kRegisterField` | Store | **I** | — | attaches a propagation rule to a registered F32 arity-1 cell property; returns the field index |
| ONIP | `kSetFieldSource` | Parameter | Q | `StepFields` | adds, replaces or removes one source of one field, keyed by the owner's own id; `strength == 0` removes |
| ONIQ | `kSetPayloadMixing` | Parameter | Q | `StepPayloadMixing` | how fast a liquid-carried amount evens out across still water: the fraction of a neighbour pair's concentration gap closed per substep. 0 switches mixing off; `-1` sets every liquid-carried property |
| ONIS | `kSetDissolvedTint` | Parameter | Q | `Project` (the liquid texture) | tints liquid by what is dissolved in it: one colour and weight per lane, and a strength |
| ONIU | `kSetEffervescence` | Parameter | Q | `StepEffervescence` | per-lane solubility data; supersaturated liquid releases gas on `sim.liquid_payload_released` for the managed side to spawn as bubbles |
| ONIV | `kSetTunable` | Parameter | **I** | every kernel, which copies the table when it starts | sets one row of `sim/tunables.def`, or restores every row to its default; returns 0 or the refusal reason |
| ONIW | `kSetVisibilityState` | Checkpoint | **I** | the falling-liquid hand-off | the visibility mask's three buffers, which a load zeroes |
| ONIX | `kSetLoadIsRestore` | Checkpoint | **I** | the next `Load` | marks the next `Load` as a checkpoint restore, which skips the load-time state transition. One-shot; carries no state |

`ONIR` and `ONIT` are reserved.

### 3.3 Why nine are immediate

The nine **I** rows are not an inconsistency.

- `kRegisterCellProperty`, `kRegisterElementAttribute` and `kRegisterField` **have** to be
  immediate. Registration closes at the first `Allocate`; a queued registration would drain after
  the array it declares had already been sized, and each returns the index that every later
  message needs.
- `kSetCellRadiation` is the one checkpoint component the game reads back in `GameDataUpdate`,
  so it must land before the next publish.
- `kSetVisibilityState` must land before the first frame after a `Load`, which publishes the
  restored world again and swaps the buffers it restores.
- `kSetLoadIsRestore` must be seen by the `Load` that follows it. `Load` is handled on the
  calling thread, so a queued flag would drain only after the `Load` it was meant for.
- `kSetTunable` must be able to set `SubstepSeconds`, which changes only while no world is
  allocated. A queued message drains inside a frame, and there is no frame without a world.
- `kSetMolecularMass` and `kSetElementAttribute` write element data that exports read without
  waiting for the worker. Written on the worker, that would be a race; immediate, it lands on
  the calling thread before the call returns.

The consequence for a caller: **a queued write is not visible to a read-back in the same tick;
an immediate one is.** `ExtMessageDesc::delivery` says which a message is.

---

## 4. What an extension point may and may not touch

### 4.1 May

* **Its own registered data**, at any arity, for any cell. That is what the registry is for.
* **One cell it addresses by index**, through the message that addresses it. Cell indices are
  game cells at the ABI boundary; the conversion to the padded grid happens inside, so a caller
  never has to know about the padding.
* **One building or one element it addresses by handle or hash.**

### 4.2 May not

* **Depend on cross-region order.** §2.4. This is the one that silently works on every
  single-region world and then fails.
* **Assume a phase ran.** A gated phase is skipped on ordinary frames, not only on broken ones.
  Anything that must hold every frame has to be re-established, not assumed carried over.
* **Assume same-tick visibility of a queued write.** §2.1 and §3.3.
* **Assume a refusal is loud.** An out-of-range cell, property or component index is dropped,
  by design and consistently: a property index comes back from a registration, so a bad one is a
  caller bug. Refusals that carry a reason are published on the `sim.message_refused` stream
  (`kExtRefusalShortPayload`, `kExtRefusalUnknownMessage`, `kExtRefusalBadTarget`); subscribe to
  it while developing.
* **Expect anything before the first `Allocate`**, except the phase and message tables (§2.5,
  §3.2).

### 4.3 The invariant every point holds

**Byte-identical until sent.** Every extension defaults to a value that makes the simulation
compute exactly what the game's own library computes. A world that sends none of these messages
produces the same output. A new extension point that cannot hold this invariant is not ready to
be added.

One exception, stated in the header at `kSetMolecularMass`: the molar masses of the four
diatomic gases are corrected natively (the game stores atomic masses for them), so for those
"never sent" means "keeps the corrected values". This affects pressure readings only; see
`GAS-MIXTURES.md`.

---

## 5. How to add one

1. **Decide the class** (§3.1): a parameter read while stepping, a drain-time operation, storage,
   or checkpoint state. If it is a callback, re-read §1.
2. **Pick the phase that reads it**, from §2.2, and say so in the header comment. If no existing
   phase is the right place, that is a much bigger change than a new message: the phase list is
   an ABI surface.
3. **Allocate the next `ONI*` id** in `abi/sim_abi_ext.h`, with its payload struct, a
   `static_assert` on its size, and a comment saying what happens when it is never sent. Add it to
   `ONI_EXT_MESSAGE_LIST` and to the hand-typed copy in `gastest`.
4. **Hold the byte-identical-until-sent invariant** (§4.3), and say in the comment which default
   delivers it.
5. **Choose the delivery deliberately.** Queued, unless registration order or a read-back race
   forces immediate (§3.3). Immediate means it runs on the caller's thread, so check what else
   touches that store.
6. **Add a test that can fail.** A green suite proves nothing about a field no test sends.
7. **Run `diffsim` against a build of the previous commit** and confirm the output is identical,
   not merely that it looks fine.
8. **Add the row to §3.2.**
9. **Bump `sim/VERSION`** (MINOR for an addition) in the same commit.

---

## 6. Design notes

### 6.1 Checkpoint ids are not extension points

`kSetRandomState`, `kSetSchedulingState`, `kSetStableTicks`, `kSetDiseaseGrowth`,
`kSetRegistryState`, `kSetExtCellState`, `kSetCellRadiation` and `kSetVisibilityState` restore
run state a save blob cannot carry, so that a replayed run resumes identically, and
`kSetLoadIsRestore` stops the restore's own `Load` from changing the world it loads. They extend
nothing. They share the header and the drain with the extension points, so the class is
published as a field, `ExtMessageDesc::messageClass`, rather than left to prose. A mod that
wants only the real extension points filters on it.

### 6.2 `kSetMolecularMass` and `sim.molecular_mass` are two names for one store

`kSetMolecularMass` (ONI7) is a convenience alias that writes the `sim.molecular_mass` element
attribute, which `kSetElementAttribute` (ONIG) can also write. A mod enumerating the surface sees
two entries for one capability. The alias stays because the framework API already uses it.

### 6.3 Delivery is published

Whether a message takes effect in this call or on the next tick changes a caller's code, not
just its vocabulary. So it is published: `ExtMessageDesc::delivery` is `kDeliveryQueued` or
`kDeliveryImmediate`, from the same `ONI_EXT_MESSAGE_LIST`. `SIM_ExtMessageCount` and
`SIM_ExtMessageDescribe` read no world state and can be called before `SIM_AllocateCells`, so a
mod can check at load time, which is the one moment it can still decline to load.

The guard is split. `gastest` holds a hand-typed list of every row; it needs no game data, so
it can run anywhere. `vftest` checks what only a loaded DLL can: that the table is exported,
that descriptors survive marshalling, that refusals leave a caller's buffer alone, and that
several descriptors match a delivery the suite has just observed.

### 6.4 One phase scope has a single member

`kPhaseScopeGridInRegion` describes only `ZeroMasslessCells`. Every world in the offline suite
has one region, so no test can distinguish it from `kPhaseScopeRegion` by behaviour. It is
pinned by name in `vftest` instead.

### 6.5 Reference values check themselves

The offline suites compute their own reference digests and compare them against
`driver/GOLDENS.txt`, exiting non-zero when one moves. `--record-goldens` is the only supported
way to change one. Documentation points at that file and never copies a value from it. See
`driver/README.md`.

---

## 7. See also

* `EXT-REGISTRY.md`: the storage mechanism the `Store` rows are built on.
* `GAS-MIXTURES.md`: the gas-mixture layer behind ONI2–ONI5.
* `LEDGERS.md`: the conservation ledgers several of the building parameters are charged to.
* `abi/sim_abi_ext.h`: `ONI_EXT_PHASE_LIST`, `ONI_EXT_MESSAGE_LIST` and every payload struct.
* `abi/sim_ext_api.h`: the same surface as a plain C header.
* `driver/src/gastest.cpp`: the hand-typed message table.
* `driver/src/vftest.cpp`: the phase-order guard, the message surface across the ABI, and every
  refusal.
