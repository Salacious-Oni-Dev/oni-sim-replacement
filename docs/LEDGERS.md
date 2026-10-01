# Conservation ledgers

The simulation keeps two sets of books: one for mass and one for energy. Each is a list of
cumulative counters, one per place in the code where mass or energy crosses the boundary of
what the simulation holds. Subtracting everything the counters explain from the actual total
leaves the **drift**: whatever nothing accounts for. Drift is the check. The counters exist to
be subtracted.

Neither ledger compares against the game's own library, so both keep working where a
cell-by-cell comparison cannot: in worlds no scenario covers, and wherever an extension changes
the physics on purpose.

Implementation: `World::Ledger` and `World::EnergyLedger` in `sim/world.h`, charged by the
`World::Note*` functions. Read through `SIM_DebugLedger` and `SIM_DebugEnergyLedger`. Checked by
`diffsim --ledger` and `diffsim --energy-ledger`.

## How the check works

```
drift = (total_now - net_now) - (total_then - net_then)
```

`net` is everything the counters say came in minus everything they say went out. The starting
total cancels, so only the unexplained part is left.

The pass bar is **1e-6 of the total**. Transfers are single-precision (`d.mass = d.mass +
moved` rounds), while the counters accumulate in double, so the books do not balance to the
last bit and zero is the wrong bar. Across the whole `diffsim` suite, 101 scenarios of 50 ticks
each, every scenario passes both ledgers; the worst energy drift is 2.5e-7 relative.

Three rules make the counters trustworthy:

- **One counter per call site, not per cause.** A single "something was deleted" counter would
  balance the books by construction and prove nothing. Split by site, a counter that moves in a
  scenario with no business moving it is a finding in itself.
- **Charge what the cell actually changed by, not what the code meant to move.** Every charge is
  measured across the write, before and after. `(a + b) - a` is not `b` once the two differ
  enough to round.
- **A counter that counts a deletion is not part of the sum.** Adding it would cancel the
  deletion and report conservation exactly where there is none.

Both ledgers are off by default in `diffsim`, because reading the totals walks the whole grid
once per tick.

## Reading them

```c
int SIM_DebugLedger(double* out, int count);        // mass, kg
int SIM_DebugEnergyLedger(double* out, int count);  // energy, kJ
```

Each fills up to `count` doubles and returns the number of fields the DLL knows about, so a
caller built against a longer list can tell. The field order is a contract: **fields are
appended, never reordered**. `kLedgerFields` and `kEnergyLedgerFields` in
`driver/src/diffsim.cpp` are the other half of it, with `LedgerNet` and `EnergyNet` showing
exactly how each field enters the sum.

Both exports wait for the frame worker (see `THREADING.md`), so they can be called from the game
thread at any time.

## Units

**Energy is in kilojoules throughout.** The game's `specificHeatCapacity` is kJ/(kg·K), so
`mass × specificHeatCapacity × temperature` is kJ, and so is the `heat capacity × temperature`
of every other container. A stray `× 1000` "to convert to joules" is the classic mistake here.

## The mass ledger

Field 0 is the grid's total mass. Every other field is a cumulative counter in kg.

| # | field | site | meaning | in the sum |
|---|---|---|---|---|
| 0 | `grid` | | total mass of every cell | total |
| 1 | `emitted` | `ApplyMassEmission` | the game added mass | + |
| 2 | `modified` | `ApplyModifyCell` | the game overwrote a cell (signed) | + |
| 3 | `consumed` | `ApplyMassConsumption` | the game removed mass | − |
| 4 | `dug` | `ApplyDig` | a cell's contents became an entity | − |
| 5 | `ore` | `TransitionCell` | transition ore, reported as `SpawnOreInfo` | − |
| 6 | `unstable` | `UnstableCheckBasic` | a falling solid, handed to the game or dropped | − |
| 7 | `sublimated` | `DoSublimation`, `OffGasTransfer` | the share lost to `sublimateEfficiency` | − |
| 8 | `wisp` | `EvaporateWisp` | gas under 0.001 kg, deleted where it stands | − |
| 9 | `thinliq` | liquid branch of `StepPostProcess` | liquid under 0.01 kg, deleted likewise | − |
| 10 | `cleared` | `ClearCell` | reached a cell that still held mass | − |
| 11 | `compconsumed` | `StepElementConsumers` | a native element consumer drained cells | − |
| 12 | `compemitted` | `EmitInto` | a native element emitter filled a cell | + |
| 13 | `emitore` | `TryEmit`, solid branch | ore **created** for the game; see below | excluded |
| 14 | `mover` | `DisplaceLiquidDirectional` | the destination's gain when a liquid cell absorbs another whole | + |
| 15 | `worldinit` | `ResizeAndInitializeVacuumCells` | a world opened: the border ring arriving, whatever stood in the rectangle leaving (signed) | + |
| 16 | `atmosboundary` | `StepWorldEnvironment` | gas a planetary surface boundary supplied or took back (signed) | + |
| 17 | `dissolvedsurface` | surface uptake | gas dissolved out of the cell above into a liquid surface (signed) | + |

Notes:

- **`emitore` is outside the balance.** A solid element emitter puts nothing into the grid and
  takes nothing out. It makes a lump of ore out of mass the building holds outside the
  simulation and hands it straight to the game. The counter exists so that "the simulation
  created this much ore" can be seen.
- **`mover` is needed at one site only.** Every other fluid mover subtracts from its own source,
  so both halves stay inside the grid total. `DisplaceLiquidDirectional` instead clears the
  source, which charges `cleared`, a counter for mass that left the grid, so the destination's
  gain has to be counted to offset it.
- Several of the deletions (`wisp`, `thinliq`, the share lost to sublimation efficiency, a
  falling solid with nowhere to go) are the game's own behaviour and are reproduced, not fixed.
  The ledger makes them visible; it does not remove them.

## The energy ledger

Energy lives in four containers, and only the first is the grid. The conserved quantity is
their **sum**: the commonest transfer in the simulation, a building exchanging heat with the
cell under it, moves energy between two of them and would read as drift against either alone.
Field 40 is a small fifth stock: `ModifyCellEnergy` payments that cells are holding under the
`CellEnergyCarry` tunable. It is not in the sum and not in `net`. `cellenergymsg` counts only
what landed in a cell, and a held payment lands with a later one, so leaving the stock out of
both sides keeps the books closed.

| # | field | container |
|---|---|---|
| 0 | `grid` | cells: `mass × specificHeatCapacity × temperature` |
| 1 | `buildings` | building bodies: `per_cell_heat_capacity × CellCount × temperature` |
| 2 | `conduits` | conduit contents: `contents_heat_capacity × temperature` |
| 3 | `chunks` | element chunks: `heat_capacity × temperature` |

The counters, all in kJ. "+" means added to `net`, "−" subtracted, and "excluded" means the
field is informational and not part of the sum. Signed counters are charged as the containers
see them, so energy leaving charges negative and the counter is still added.

| # | field | site | meaning | in the sum |
|---|---|---|---|---|
| 4 | `bldgoperating` | `StepBuildingHeatExchange` | `operating_kilowatts × dt`, the only place electrical energy enters thermal state | + |
| 5 | `bldgexchange` | `StepBuildingHeatExchange` | the building–cell exchange's rounding; the two sides are clamped independently | + |
| 6 | `bldgreg` | Add / Modify / Remove building | a building carrying its own energy in or out | + |
| 7 | `bldgenergymsg` | `ModifyBuildingEnergy` | discrete kJ delivered to a building, as landed | + |
| 8 | `bldgwasteheat` | `StepBuildingHeatExchange` | the `kSetBuildingWasteHeatKilowatts` rate × dt | + |
| 9 | `bldgcount` | | number of registered buildings | excluded |
| 10 | `simseconds` | | the simulation's accumulated substep time, in seconds | excluded |
| 11 | `cellenergymsg` | `ApplyCellEnergy` | `ModifyCellEnergy`, as landed | + |
| 12 | `cellenergyrefused` | `ApplyCellEnergy` | what `ModifyCellEnergy` asked for and did not deliver | excluded |
| 13 | `bldgexhaust` | `StepBuildingHeatExchange` | the `kSetBuildingExhaust` rate × dt | + |
| 14 | `bldgexhaustbounced` | `StepBuildingHeatExchange` | the part of `bldgexhaust` that went into the building's body instead of the cells | excluded (already in 13) |
| 15 | `radiated` | `StepBuildingHeatExchange` | building bodies radiating to the environment | − |
| 16 | `chunkreg` | Add / Set / Remove chunk | a chunk carrying its own energy in or out | + |
| 17 | `chunkenergymsg` | chunk `ModifyEnergy` | discrete kJ delivered to a chunk, as landed | + |
| 18 | `chunkadjuster` | `StepElementChunks` | the chunk temperature adjuster, which acts as an outside source | + |
| 19 | `chunkexchange` | `ExchangeChunkHeatWithCell` | the chunk–cell exchange's rounding | + |
| 20 | `chunkenergyrefused` | chunk `ModifyEnergy` | what a chunk message asked for and did not get | excluded |
| 21 | `cellmod` | `ModifyCell` and `cellmod.h` | the game writing matter, and its heat, into a cell | + |
| 22 | `displace` | `DoDisplacement` | the gas-displacement primitive's own loss | + |
| 23 | `conductclamp` | `StepConduction` | the [1 K, 10000 K] temperature clamp | + |
| 24 | `cleared` | `ClearCell` | energy of matter deleted there | + |
| 25 | `wisp` | `EvaporateWisp` | likewise | + |
| 26 | `thinliquid` | `StepPostProcess` | likewise | + |
| 27 | `unstable` | `UnstableCheckBasic` | likewise | + |
| 28 | `consumedenergy` | `ApplyMassConsumption` | energy of mass the game removed | + |
| 29 | `emittedenergy` | `ApplyMassEmission` | energy of mass the game added | + |
| 30 | `compemitenergy` | `EmitInto` | a native element emitter | + |
| 31 | `mover` | `StepFlow`, `StepGasPressure`, `StepGasDisplacement`, `DisplaceLiquidDirectional` | the fluid movers' transfer difference; see below | + |
| 32 | `worldinit` | `ResizeAndInitializeVacuumCells` | a world being opened | + |
| 33 | `sublimatedenergy` | `DoSublimation`, `OffGasTransfer` | sublimation and off-gassing | + |
| 34 | `phasechange` | `TransitionCell` | phase transitions; see below | + |
| 35 | `transitionore` | `TransitionCell` | energy leaving with transition ore | + |
| 36 | `absorbed` | `StepWorldEnvironment` | sunlight absorbed by a sky-exposed cell or building | + |
| 37 | `atmosboundaryenergy` | `StepWorldEnvironment` | heat carried by gas the surface boundary supplied or took | + |
| 38 | `cellradiated` | `StepWorldEnvironment` | a sky-exposed cell radiating to its sky | − |
| 39 | `dissolvedsurface` | surface uptake | the grid's change where a liquid surface dissolved gas | + |
| 40 | `cellenergycarry` | `ApplyCellEnergy` | `ModifyCellEnergy` payments cells are holding under the `CellEnergyCarry` tunable, not yet in a temperature (see [DRAIN.md](DRAIN.md)); a stock, and zero unless the tunable has been on in this world | excluded |

`radiated` and `cellradiated` are kept apart so that the colony's waste heat can be told apart
from the planet's own day and night balance.

`bldgcount` separates "no buildings registered" from "buildings registered with no heat
capacity", both of which read as a `buildings` total of 0. It matters because buildings are
registered by a queued message, so a paused game has no native building records at all.

`simseconds` exists because the simulation's clock and the game's `GameClock` do not advance
at the same rate. A rate the simulation charges per substep can only be compared with an
integral over the simulation's own time.

## Where the game's own physics does not conserve energy

With every counter in place, the ledger closes. Several of those counters record places where
the game's own simulation, reproduced faithfully, does not conserve energy. They are behaviour
to preserve for compatibility, and extensions can change them deliberately.

- **Phase change (`phasechange`)** is the largest. Temperature carries across a transition and
  specific heat does not. 1000 kg of water at 200 K becomes ice at 201.5 K; ice's specific heat
  is 2.05 against water's 4.179, so the cell's stored energy nearly halves in one step, where
  conserving it would have put the cell at 407 K. The 1.5 K overshoot is the whole of the game's
  latent heat. On the `boiling` scenario this is 1.8 % of the world's energy over 50 ticks.
- **Fluid movers (`mover`).** A transfer debits the source at the temperature it holds now and
  credits the destination at the temperature the source held when the substep began, the
  snapshot that makes a sweep order-independent. Mass is conserved; energy differs.
- **Gas displacement (`displace`)**: the displaced gas is mixed by mass rather than by heat
  capacity, and the destination's element is overwritten after the mix.
- **The conduction clamp (`conductclamp`)**: `StepConduction` clamps temperatures to
  [1 K, 10000 K], which adds energy at the cold end and removes it at the hot end.
- **Sublimation (`sublimatedenergy`)**: only `sublimateEfficiency` of what leaves the solid
  arrives, and the product is written at the solid's temperature with no mixing.
- **Exchange clamps (`bldgexchange`, `chunkexchange`)**: each side of a two-body exchange is
  clamped on its own interval, so what one gives and the other receives can differ. This stays
  at the rounding floor.
- **Deletions (`cleared`, `wisp`, `thinliquid`, `unstable`)**: matter deleted with nowhere to
  go takes its energy with it. Unlike `radiated`, these have no receiver.
- **The chunk temperature adjuster (`chunkadjuster`)** drives a chunk toward a target
  temperature and so behaves as an energy source from outside the simulation.

### Heat the game discards on delivery

`ModifyCellEnergy` is the message the game's `StructureTemperatureComponents.ExhaustHeat` uses to
put a building's exhaust heat into the cells it covers. Heat is lost on five paths:

| path | where | when it applies |
|---|---|---|
| delivery scaled by `min(cell mass, 1.5 kg) / 1.5 kg` | the game's managed code, before sending | any cell under 1.5 kg, and all of it in vacuum |
| vacuum or mass ≤ 0.001 kg | `ApplyCellEnergy` | whatever the scaling did not already remove |
| the cell is at or above the message's `maxTemperature` | `ApplyCellEnergy` | the game passes the building's overheat temperature, 348.15 K by default, so a machine in a hot room delivers nothing |
| the 10000 K clamp | `ApplyCellEnergy` | rare |
| the float rounding of the temperature write | `ApplyCellEnergy` | every payment small against the cell's heat capacity, in either sign; with `CellEnergyCarry` on it is held for the cell's next payment instead |

The native share of this is counted in `cellenergyrefused`. The managed scaling happens before
the message is sent, so no counter can see it.

`kSetBuildingExhaust` is the alternative. It moves the whole exhaust into the simulation as a
rate integrated over its own substeps. The delivery rules are the same, shared with the
`ModifyCellEnergy` handler through one function (`DeliverCellEnergy` in `sim/buildings.h`), and
the one change is that the undelivered remainder goes into the building's own body instead of
disappearing. `bldgexhaustbounced` reports how much was rerouted.

### Radiation to the environment

`kSetBuildingRadiation` lets a building body radiate to a sink, and `kSetWorldEnvironment` sets
the sink per world. Three properties make it safe to add to a game that already has conduction:

- it only ever sheds heat;
- the sink temperature is pulled toward the local medium's temperature, weighted by how much
  medium there is, so a body in a pressurised room at its own temperature radiates nothing;
- a step never overshoots: it moves at most half the body's energy above the sink.

The flux is Stefan–Boltzmann, `σ·A·(T⁴ − T_sink⁴)`.

The environment is an infinite reservoir: `radiated` leaves the four containers and does not
return. A finite reservoir is left to managed code.

## Not in either ledger

The planetary latent-heat accumulator (`SIM_ExtWorldLatentEnergy`) records the latent heat that
sky-exposed condensation and boiling put into or took out of a world's atmosphere. It changes no
cell, so nothing crosses the grid boundary and neither ledger charges it.
