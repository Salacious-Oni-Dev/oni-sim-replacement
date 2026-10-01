# Conduit temperatures

The contents of a pipe are not cells. They live in the game's own `ConduitFlow` grid, the
simulation never sees them move, and nothing about them appears in a `GameDataUpdate`. What the
simulation owns is one temperature per conduit, and one job: exchange heat between that
temperature and the building the conduit is, then tell the game when the contents froze or
boiled. Every cooling loop, aquatuner and liquid radiator in the game goes through this.

This is the only subsystem with no grid and no neighbours in the vanilla case: a flat list of
independent two-body problems, driven once per 200 ms from the game thread rather than from the
simulation frame.

On top of that vanilla behaviour, the simulation can be told which run each conduit belongs to.
A bound run can then be summed, and given optional policies (heat mixing along the run,
pressure-driven phase change, convection). A conduit that no caller has bound runs the vanilla
arithmetic unchanged.

Implementation: `sim/conduits.h` (`ConduitTemperatures`). Checked by `driver/diffsim --conduits`,
which calls the conduit exports of both libraries directly and compares every return value for
bit equality, and by `driver/vftest` for the run extensions.

## The coupling

One message each way, and neither is a direct write.

- **In:** the game passes `Game.Instance.simData.buildingTemperatures`, the array the last
  simulation frame published, to `ConduitTemperatureManager_Update`. Each conduit reads its
  building's temperature from it by handle slot.
- **Out:** the energy the contents took from the building is paid back as a
  `ModifyBuildingEnergy` message, queued for the next frame. Applying it directly would warm a
  building in the middle of a frame it is part of.

So a conduit always exchanges against a building temperature one frame old, and the building
always learns about it one frame late. That is the game's design.

## Data

Per conduit:

| field | value |
| --- | --- |
| `temperature` | the contents' temperature |
| `contents_conductivity` | the contents element's `thermalConductivity`, unscaled |
| `contents_heat_capacity` | `mass * element.specificHeatCapacity` |
| `structure_handle` | the building, as an index into the building temperature array |
| `conduit_heat_capacity` | the pipe's own, sent by the game |
| `conduit_conductivity` | the pipe's own, sent by the game |
| `insulated` | `def.ThermalConductivity < 1` |
| `low_temp` | the element's `lowTemp`, or 0 when it has no low transition |
| `high_temp` | the element's `highTemp`, or `FLT_MAX` when it has no high transition |

The simulation also keeps each conduit's contents mass and element, and, once bound, its cell,
run id and piped neighbours.

`Add` resolves the element once, from its hash, and keeps its numbers rather than its identity;
an element table reload does not reach conduits already registered. `Set` updates only the
contents (temperature, mass, element); the pipe's own heat capacity and conductivity are fixed
when it is built.

A temperature above the 10000 K ceiling is refused on both `Add` and `Set` and replaced with the
element's **default** temperature, not clamped, so a corrupt save loads cold rather than
exploding.

Handles are `slot | (version << 24)`.

## The exchange

Per conduit, per update:

```
k        = insulated ? min(k_contents, k_conduit) : (k_contents + k_conduit) * 0.5
rate     = k * (T_contents - T_building) * 50           // contact area
energy   = rate * dt * 0.001
lo, hi   = min(T_contents, T_building), max(T_contents, T_building)
limit_c  = |clamp(T_contents - energy / hc_c, lo, hi) - T_contents| * hc_c
limit_b  = |clamp(T_building + energy / hc_b, lo, hi) - T_building| * hc_b
moved    = min(limit_b, limit_c) * (rate >= 0 ? 1 : -1)
T_c'     = max(T_contents - moved / hc_c, 0)
T_b'     = max(T_building + moved / hc_b, 0)
if (T_c' - T_b') * (T_contents - T_building) < 0:        // they crossed over
    T_c' = (hc_c * T_contents + hc_b * T_building) / (hc_c + hc_b)
```

Here `hc_c` is the contents' heat capacity and `hc_b` the conduit's own. The building is paid
`(T_contents - T_c') * hc_c` kJ, bounded to `[lo, hi]`, when the transfer exceeds 1e-6 kJ.

Three properties of this formula matter:

- **Insulated pipes change the shape of the formula, not its scale.** The minimum of the two
  conductivities replaces the mean, so an insulated pipe is held to its weaker side rather than
  slowed by a constant factor. Cell insulation follows the same rule.
- **Both sides are limited independently and the smaller limit wins.** `limit_c` and `limit_b`
  are computed from the same proposed transfer. A small amount of water against a large
  building moves by the water's limit.
- **An overshoot lands on its exact equilibrium.** If the two ends come out on opposite sides of
  where they started, the contents are set to the temperature the pair would share. An
  overshooting pair settles in one update rather than oscillating.

### Skips

In order:

- no building temperatures at all (the game has not published a frame yet): every temperature
  is copied through unchanged;
- a structure handle whose slot is at or above `0x100000`: **nothing** is written, not even the
  unchanged temperature, so the output slot keeps its previous value;
- a contents or conduit heat capacity at or below 1e-4, or a building temperature not above 0:
  the temperature is copied through unchanged.

### NaN

The comparisons are written so that a NaN behaves as it does in the game's library:

- a NaN heat capacity is **not** skipped;
- a NaN building temperature **is** skipped;
- `min` and `max` return their second operand when either is NaN;
- a NaN product in the crossover test takes the equilibrium branch;
- a NaN result is refused: the conduit keeps its previous temperature and nothing is billed to
  the building. An infinity is not refused.

## Transitions

A conduit **freezes** below `low_temp - 3` and **melts** above `high_temp + 3`. The three-degree
margin stops a pipe sitting exactly on its transition from reporting a phase change every
update. Frozen and melted conduits are reported as handles with the version byte intact, so the
game's lookup fails on a stale handle rather than aliasing another conduit.

The two sentinels (0 for no low transition, `FLT_MAX` for no high one) cannot be observed:
elements with no low transition already carry `lowTemp = 0`, and elements with no high
transition carry `highTemp = 10000`, which is also the simulation's temperature ceiling.

## Removal is deferred by a frame

`Remove` does not free the slot. It neuters the entry (a conduit heat capacity of -1 fails the
1e-4 test from the next update on), takes it out of its run at once, and queues the handle. The
queue is released once per frame, from `PrepareGameData` on the game thread, with a parity flip,
so a handle comes back one full frame later.

The delay exists because the game holds handles across a frame boundary. The version byte would
catch a recycled slot, but the temperature array is indexed by slot and would already hold the
new conduit's value.

## Conduit runs

A run is one of the game's own conduit networks: `UtilityNetworkManager`'s connected
components, bridges and broken segments included. The managed side names them.

- **`SIM_ConduitNetworkBind`** replaces every binding of one conduit type (1 gas, 2 liquid):
  for each conduit, its handle, its cell, its `UtilityNetwork.id` and the handles of up to four
  piped neighbours (left, right, up, down), plus the per-conduit volume and the per-conduit
  capacity. The framework re-sends it after every `ConduitFlow` rebuild. Each bind increments a
  **generation**; a run id is only meaningful with its generation, because the game renumbers
  networks whenever a conduit is built, broken or removed.
- **`SIM_ConduitNetworkAggregate`** returns one run summed: conduit count, filled count, total
  mass, moles, heat capacity, composition (up to 8 species, with an overflow count, never
  silent), two temperatures and pressure. `temperatureK` is heat-capacity-weighted, the
  temperature the contents would settle at if mixed; `massWeightedTemperatureK` is the
  mass-weighted mean. `pressurePa` is a gas run's ideal-gas pressure over its volume, computed
  as `SIM_ComputeGasPressure` does; a liquid run reports its headspace pressure separately. Sums
  are rebuilt at most once per change, for every run at once.
- **`SIM_ConduitTrappedSet` / `SIM_ConduitTrappedClear`** mirror standing matter that a conduit
  cannot represent (condensate in a gas line, headspace gas in a liquid line), keyed by cell so
  it survives every rebind. It is reported in the aggregate's `trapped*` fields, separately from
  the conduit contents.

Mass and composition lag the game's own grid by at most one conduit frame, because the game
publishes contents at the end of each conduit frame. Temperature is the simulation's own.

## Run policies

`SIM_ConduitNetworkPolicy` sets flags on one run of the current bind. All three are off by
default, and a run with none is updated by the vanilla arithmetic alone.

**MIX (`0x1`)** exchanges heat along the run after each update's conduit-to-building exchange.
Every piped pair where both conduits hold something exchanges

```
q = k (T_i - T_j) C_i C_j / (C_i + C_j),     0 < k <= 0.25
```

with every link computed from the same starting temperatures. Energy is conserved within the
run, nothing crosses to a building, and with at most four links and `k <= 0.25` every result is
a weighted average of its neighbours, so nothing overshoots. This is deliberately not
Stationeers' one temperature per network: an ONI run can cross rooms with no device in between,
and levelling it whole would erase any gradient along it. Trapped matter is not mixed. A mixing
conduit's freeze and melt test runs after the mix, against the temperature the game will be
given.

**PHASE (`0x2`)** decides pressure-driven phase change once per update, after the mix. Gas in a
gas run condenses when the run's pressure puts its dew point above its temperature; liquid
standing in a gas run evaporates when it puts the dew point below. Liquid in a liquid run boils
when the headspace pressure puts its boiling point below its temperature; headspace gas
condenses back. The boundary comes from the element's phase curve, and the amount per update is
a bounded step at the policy's rate (`phaseRatePerSecond`, with
`phaseMinRemainderKg` below which the whole source converts).

**The simulation only decides.** Conduit contents belong to `ConduitFlow` and trapped matter to
the managed side-car, so each decision is published as an `OniConduitPhaseProposal`
(`SIM_ConduitPhaseProposals`) with its kind, cell, source and product elements, the source state
it was decided against, the converted mass, the boundary, the pressure and the latent heat. The
managed side applies it, or refuses one that has gone stale. Each update replaces the previous
proposals.

| kind | run | what moves |
| --- | --- | --- |
| `CONDENSE_CONTENTS` (0) | gas | conduit gas becomes standing liquid |
| `BOIL_CONTENTS` (1) | liquid | conduit liquid becomes standing gas |
| `CONDENSE_HEADSPACE` (2) | liquid | standing gas becomes conduit liquid, clamped to the conduit's headroom |
| `EVAPORATE_TRAPPED` (3) | gas | standing liquid becomes conduit gas, clamped to the conduit's headroom |

**CONVECTION (`0x4`)** scales each conduit's exchange with its building by two of Stationeers'
`HeatExchangeRatio()`s (`AtmosphereHelper.CalculateConvection`), keeping ONI's own coefficient:

- the pipe side: `max(clamp01(P / 1 atm), clamp01(liquid volume / run volume / 0.001))`, where
  `P` is the run's gas pressure, or a liquid run's headspace pressure;
- the cell side: the same ratio for the conduit's own cell, read from the frame the game holds.
  A vacuum cell is 0. A **solid** cell is 1, where Stationeers uses 0: a pipe embedded in a tile
  touches matter, and ONI's conduction owns that exchange.

So a vacuum on either side stops the exchange, low-pressure gas slows it in proportion, and a
trace of liquid saturates it. The limits, the equilibrium rule and the building's energy
message are unchanged. A building that registers its own convection (`kSetBuildingConvection`)
owns the cell ratio, and its conduits do not apply it a second time. Before the game holds any
frame, the cell side cannot be read and the update is the vanilla one.

## Threading

Every conduit entry point runs on the game thread, outside the simulation worker's frame,
including the handle release. `Set` is called from the game's job threads during
`ConduitFlow.EndFrame`, one handle per call. See [THREADING.md](THREADING.md).

## Verification

No grid scenario reaches any of this, so `diffsim --conduits` is a second harness. It hands both
libraries the same table of building temperatures, runs no frame, and compares every export's
return value for **bit equality**. Its twelve conduits cover one branch each: plain, insulated,
an overshooting pair, a massless conduit, an orphan structure handle, a temperature past the
ceiling, freezing, boiling, one inside the three-degree margin, and two elements with a
transition on one side only. Removing any single branch of the kernel makes it fail, except
the two sentinels, which are unobservable as described above.

`diffsim --conduits-live` runs real buildings so the `ModifyBuildingEnergy` round trip is
exercised. It aligns the two libraries by frame rather than by tick and is informational.
