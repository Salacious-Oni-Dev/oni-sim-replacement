# Gas mixtures

The game's grid holds one element per cell. This library adds an optional second layer in which
a cell can hold up to eight gas species at once, each with its own mass, sharing one
temperature. The layer is off until a mod uses it, and a world that never uses it runs
byte-identical to the game's own library.

Implementation: `abi/gas_mixture_abi.h` (the layout), `sim/gas_mixture.h` (the per-pair math),
`sim/gas_rooms.h` (rooms, pooling and ownership). Tested by `driver/gastest` and
`driver/vftest`.

## The layer

Each cell has, beside its ordinary single-element `PhaseEntry`:

| field | per cell | meaning |
|---|---|---|
| species | 8 × u16 | the element of each slot, 0 for an empty slot |
| mass | 8 × f32 | the mass of each slot, in kg |
| occupied mask | u8 | bit *i* set when slot *i* holds a species |
| dirty | u8 | the mixture changed during this mixing tick |
| sleeping | u8 | the cell has been stable long enough to be skipped |

The mixture has no temperature of its own. It shares `PhaseEntry.temperature` with whatever
the ordinary layer holds in the cell, on the assumption that gases in one cell reach thermal
equilibrium much faster than they mix with their neighbours. Stationeers' atmosphere has the
same shape: one temperature, several gases.

Storage is structure-of-arrays and cell-major. Species, mass, occupancy and room promotion are
registered as saved extension properties (see `EXT-REGISTRY.md`), so they round-trip through
the save blob with no format change of their own.

Eight slots is a fixed limit. When a cell already holds eight other species, an injection of a
ninth is dropped, not merged.

## Activation

The layer activates on the first `kInjectGasSpecies` or `kPromoteRoom` message a world receives,
or when a loaded save already carries mixture state. Activation builds the room graph and
starts the mixing tick. Until then the mixing branch in `StepPhysics` is one untaken `if`.

## Rooms

A room is a maximal set of open cells connected four-ways. A cell is open when it is neither
marked gas-impermeable nor holding a solid element. Both tests are needed: a wall painted with
`SimMessages.ReplaceElement` has a solid element but never gets the impermeable flag, and
trusting the flag alone would let a room leak through it.

The room graph lists each room's cells, the interior pairs of neighbouring cells within the
room, and the boundary pairs that cross between rooms. It is rebuilt whenever a solid boundary
opens or closes: a phase transition, a dig, or a building placed or removed.

## Mixing

Mixing runs once per substep, after every other kernel, over every room at once rather than per
region, because a room can span regions.

For each neighbouring pair, every species present on either side moves from the side with the
higher partial pressure toward the lower, in proportion to its own pressure difference. Each
species equalizes independently of the others (Dalton's law). The step is damped rather than
solved exactly, as in Stationeers' gas mixing, and repeated calls converge.

Heat moves with the mass. A species leaving a cell takes `mass × specific heat × T` of that
cell's energy with it, and the receiving cell's temperature is its new energy over its new heat
capacity. Moving mass without heat would create or destroy energy.

**Sleep.** A cell that has not changed for five consecutive mixing ticks is marked sleeping. A
pair of two sleeping cells is skipped. Each room keeps a count of its awake cells, updated as
cells wake and sleep, so a room with no awake cells skips its whole interior in one test.
Boundary pairs always run, because they are the only way a sleeping room can be woken from
outside.

**Pressure** is the ideal-gas law over the cell's species, `P = nRT / V`, with a fixed cell
volume of 1 m³. Mole counts use the molecular mass of each gas. The game's element data stores
the atomic mass for diatomic gases (oxygen 15.9994, hydrogen 1.00794), and a mole count taken
from it reads twice too high; `ElementTable::MolecularMassOf` corrects for this. The mixing step
uses the raw value, because there the molar mass cancels out of the mass transferred.

## Ownership: which engine simulates a cell

Injecting mixture mass into a cell does not take the cell away from the game's own gas kernels.
**Promoting** a room does. `kPromoteRoom` marks the room containing a cell as owned by the
mixture layer, and the ordinary gas kernels (conduction, gas pressure, gas displacement and the
emitters) then leave that room's cells alone. The caller converts the room's ordinary gas into
the mixture, after which those cells hold no ordinary gas at all.

Liquid is gated differently. Nothing converts liquid into the mixture, so a promoted room can
still contain an ordinary pond. The liquid kernels skip a cell only when the room is promoted
**and** the mixture actually holds mass in that cell. Because mixing spreads species outward
about one cell per tick, a promoted room with any mixture mass in it eventually has every cell
owned, and the two gates converge.

Promotion is stored per cell rather than per room id, because room ids change on every rebuild.
It is saved. There is no demotion message.

## Moving mass between the layers

| message | effect |
|---|---|
| `kInjectGasSpecies` | adds mass of a species to a cell's mixture, at a given temperature, blended into the cell's temperature by heat capacity (mass × specific heat) |
| `kRemoveVanillaMass` | removes mass from a cell's ordinary element; paired with an injection of the same mass, it is a conserving conversion into the mixture |
| `kConvertToVanillaMass` | removes up to the requested mass of a species from the mixture and adds exactly what came out to the cell's ordinary element, in one message |
| `kPromoteRoom` | hands a room to the mixture layer (above) |

`kConvertToVanillaMass` is one message rather than two on purpose. With a separate remove and
add, a request for more than the mixture holds would clamp the removal and still add the full
amount, creating mass. If the destination cell refuses the gas (a solid in the way, say), the
removed mass is put back into the mixture.

## What the game sees

The published `GameDataUpdate` is unchanged by the mixture. `Grid.Element[]`, `Grid.Mass[]` and
the rest still describe the ordinary layer only, so an unmodded reader never sees a mixture.
A mod that wants a cell's mixture reads it through the exports:

| export | returns |
|---|---|
| `SIM_GasDominantElement` | the species with the most mass in the cell, and the total mixture mass; 0xFFFF when the cell holds no mixture |
| `SIM_GasComposition` | every species in the cell with its mass |
| `SIM_GasMassBatchQuery` | mixture masses for a batch of cells in one call |
| `SIM_GasPressure` | the cell's mixture pressure |
| `SIM_RoomId`, `SIM_RoomAggregate` | the room a cell is in, and that room's summed composition, mean pressure and temperature |
| `SIM_ComputeGasPressure` | the same pressure law, for a mod's own container (a tank or a pipe) |

The intended bridge is a managed facade: a patch that intercepts a vanilla read such as
`Grid.Element[cell]` and, for a cell holding mixture state, answers with the dominant species
and the total mass. The framework API provides it.

`SIM_RoomAggregate` averages pressure only over the cells that hold mixture state, because a cell
with no mixture has no mixture pressure, not a pressure of zero. Its temperature is weighted by
each cell's whole mass, in both layers, since the two layers share one temperature.

## Containers outside the grid

A mod's own tanks and pipes hold their contents in managed code, as the game's reservoirs do.
The physics is still provided natively, as pure functions that touch no world state:

- `EqualizeSingleSpecies`: how much of one species should move between two containers of
  different volumes and temperatures that are connected with no valve between them.
- `AdiabaticFillTemperature`: the temperature of gas pumped into a rigid store. The incoming
  gas carries its enthalpy, so its temperature is scaled by the mixture's heat-capacity ratio
  before the mass-weighted mix. With a ratio of 1 this is ordinary calorimetry.
- `SIM_LiquidVolumeFromMass`, `SIM_EqualizeLiquidVolumeMass`: the liquid counterparts. Liquids
  equalize by fill fraction, not by pressure.

There is no per-cell liquid mixture. Gas dissolved in a liquid is carried as a liquid-carried
cell property (`sim.dissolved_mass`) rather than as a second liquid species; see
`EXTENSION-POINTS.md`.

## Limits

- Pressure uses a fixed 1 m³ cell volume.
- The mixing rate (0.2) and sleep threshold (5 ticks) are starting values, not yet tuned against
  gameplay.
- `TickRates` declares separate cadences for convection and radiant loss, but only mixing has a
  kernel; the other two cadences are unused.
- A cell holds at most eight species.
