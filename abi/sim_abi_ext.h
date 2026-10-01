// Extension messages — NOT part of sim_abi.h. sim_abi.h is the game's own message ABI,
// generated from the player's install by tools/gen_sim_abi.py; everything in this file is
// new, added by this project, and has no vanilla counterpart. Keeping the two files
// separate means regenerating sim_abi.h for a future game update can never clobber this one.
//
// Message ids here are plain framework-chosen constants, not Klei string
// hashes — there is no Klei source for them to match. Picked in a range
// (0x4F4E4900.."ONI\0"-prefixed) that reads unmistakably as ours if it ever
// turns up in a log next to a real SimMessageHash value.
//
// First entry, added for Mod 1 (Physical Thermodynamics): a way to
// give an individual cell extra heat capacity that vanilla's per-element
// table cannot express. Reuses oni_sim::SetCellFloatValueMessage's layout
// (cellIdx, value) — same wire shape as SetInsulationValue/SetStrengthValue,
// just a new id and a new landing spot in the sim (see
// World::MutableThermalMassBonus, physics.h's `conduct` lambda).
//
// THE PUBLISHED C VIEW OF THIS SAME ABI IS `abi/sim_ext_api.h`, and it is included below.
// That file is what a third party binds against: plain C, no namespaces, every added export
// declared, and nothing of Klei's in it. THIS file is the C++ view -- the same surface plus
// the reasoning for every id, which is why it is ten times longer.
//
// The two are kept in step mechanically, not by anyone remembering. The descriptor structs
// and the scheduling payload are DEFINED THERE and aliased here, because they appear in an
// export's signature and one definition cannot drift from itself. Everything else here is
// restated there and checked field by field -- size, offset and exact type -- at compile time
// by `driver/src/gastest.cpp`, which also fails if this file's message ids and enum values
// stop agreeing with that one's. `sim/simdll.cpp` gets the export declarations through this
// include, so a signature that drifts from what is published is a compile error at the
// definition. See sim_ext_api.h's header comment for the whole arrangement, and
// `driver/README.md` for the proof that each half of it fails when it should.
#pragma once
#include <cstddef>
#include <cstdint>

#include "sim_ext_api.h"

namespace oni_sim::ext {

// Adds `value` (J/K) to cell `cellIdx`'s heat capacity for conduction
// purposes, on top of mass*specificHeatCapacity. Negative values are legal
// (a cell can be made a worse thermal sink than its element alone would be)
// down to the point capacity would go negative for that cell's own mass;
// nothing currently clamps this, so a caller that wants a floor enforces it
// itself. Value replaces any previous bonus on that cell — it does not
// accumulate across repeated messages, same as SetInsulationValue/
// SetStrengthValue.
constexpr int32_t kSetCellThermalMassBonus = 0x4F4E4931;  // "ONI1"

// Second entry: the message that seeds the volume-fractions gas mixture
// (abi/gas_mixture_abi.h, sim/gas_mixture.h, sim/gas_rooms.h) with real mass. Injects
// `massKg` of vanilla element `speciesIdx` (an ElementTable index, not a SimHashes id) into
// cell `cellIdx`'s multi-gas mixture -- additive to whatever PhaseEntry already holds there,
// since this is a SEPARATE layer atop vanilla's single-element cells, not a replacement yet
// (see gas_mixture_abi.h's atmosphere-only scope note).
//
// The FIRST message any world receives on this id lazily activates mixing for that world:
// it builds the room graph from the cell properties at that instant and starts running the
// mixing pass once a substep (see simdll.cpp's StepPhysics). A world that never receives one
// never activates, and StepPhysics's mixing branch never runs -- same byte-identical-until-
// sent guarantee kSetCellThermalMassBonus makes, verified against the full offline suite.
//
// `temperatureK` (added, real bug found live): the temperature the incoming
// `massKg` arrives at. Blended into the cell's one shared PhaseEntry.temperature field
// weighted by HEAT CAPACITY, m*c, against whatever the cell already holds (vanilla
// PhaseEntry.mass plus any existing gas-mixture mass), so the cell's m*c*T after the add is
// the sum before plus the new gas's own. A mass-weighted blend would make or lose energy
// whenever the specific heats differ. Before this field existed, Inject left the shared temperature field
// completely untouched -- fine for same-cell conversions (ConvertFromVanilla's cell already
// has a real vanilla temperature to inherit), but wrong for a cross-cell transfer into an
// empty/vacuum destination: a sealed tank starts at vanilla's ZeroMasslessCells 0 K, nothing
// had ever set it otherwise, and CellPressure = nRT/V is then exactly 0 Pa no matter how much
// mass accumulates: a sealed reservoir holding 40,000 kg of injected O2 would read 0 Pa.
constexpr int32_t kInjectGasSpecies = 0x4F4E4932;  // "ONI2"

#pragma pack(push, 4)
struct InjectGasSpeciesMessage {
  int32_t cellIdx;
  int32_t speciesIdx;
  float massKg;
  float temperatureK;
};
#pragma pack(pop)
static_assert(sizeof(InjectGasSpeciesMessage) == 16, "InjectGasSpeciesMessage layout drift");

// Third entry: the symmetric counterpart to kInjectGasSpecies. Feeding vanilla atmosphere
// into the gas-mixture layer with kInjectGasSpecies alone duplicates matter rather than
// converting it; this takes the equivalent mass back out of vanilla's PhaseEntry. Removes `massKg` of whatever element already occupies cell `cellIdx`'s
// vanilla PhaseEntry, clamped so the cell's mass can never go negative -- the caller doesn't
// need to know the element, only how much to take, since a cell only ever holds one vanilla
// element at a time. Pairing one kInjectGasSpecies with one kRemoveVanillaMass of the same
// massKg on the same cell is a conserving transfer: nothing appears, nothing vanishes, it
// only moves from the single-element layer to the multi-species one.
//
// THE CELL'S LIQUID PAYLOAD GOES WITH THE MASS. Any
// `follows_liquid_mass` property is released in the same proportion as the mass
// removed, with reason kPayloadReleasedRemoved -- the same treatment the negative branch of
// AddRemoveSubstance gives it, because it is the same event: liquid leaving a cell by a
// message. Before that date this handler alone left the payload behind, so a caller taking
// most of a cell's liquid left all of its dissolved gas on the little that remained.
// RELEASED rather than handed to the caller, because this message names no consumer: there
// is no handle and no callback index for a consumed record to be keyed by. A caller that
// wants the payload CARRIED sends vanilla's `MassConsumption` with a callback index instead
// and reads it back on sim.liquid_payload_consumed (kPayloadConsumerMassConsumption).
constexpr int32_t kRemoveVanillaMass = 0x4F4E4933;  // "ONI3"

#pragma pack(push, 4)
struct RemoveVanillaMassMessage {
  int32_t cellIdx;
  float massKg;
};
#pragma pack(pop)
static_assert(sizeof(RemoveVanillaMassMessage) == 8, "RemoveVanillaMassMessage layout drift");

// Fourth entry: the reverse of kInjectGasSpecies+kRemoveVanillaMass
// combined into one atomic message rather than two separate ones. Two separate messages (one
// that removes from the mixture, one that adds to vanilla) would let a caller who requests
// more than the mixture actually holds have the removal clamp down while the addition still
// adds the full requested amount -- creating mass exactly the way this project's first
// mass-conservation bug did. One atomic native handler that removes first, then adds only
// however much actually came out, cannot have that gap by construction.
//
// Removes up to `massKg` of vanilla element `speciesIdx` from cell `cellIdx`'s gas mixture
// and adds however much actually came out to that cell's own vanilla PhaseEntry --
// `temperatureK` is the temperature to merge in (via the same CalculateCombinedTemperature
// vanilla's own MassEmission message uses), since the mixture layer doesn't track a
// per-species temperature of its own (see gas_mixture_abi.h's atmosphere-only scope note) and
// has no other temperature to offer. If the target cell already holds a different vanilla
// element, this reuses the exact same accept/displace contract ApplyMassEmission uses for
// every other building that puts matter into a cell (match, vacuum, or DisplaceGasFromDrain)
// rather than reinventing one; if that still refuses (e.g. a solid in the way), the removed
// amount is refunded back into the mixture layer unchanged, so this stays conserving even on
// refusal, not just on success.
constexpr int32_t kConvertToVanillaMass = 0x4F4E4934;  // "ONI4"

#pragma pack(push, 4)
struct ConvertToVanillaMassMessage {
  int32_t cellIdx;
  int32_t speciesIdx;
  float massKg;
  float temperatureK;
};
#pragma pack(pop)
static_assert(sizeof(ConvertToVanillaMassMessage) == 16,
              "ConvertToVanillaMassMessage layout drift");

// Fifth entry: the per-room promotion bit -- how the managed side sets gas_rooms.h's
// `RoomGraph.owned`, handing a room from vanilla's gas kernels to the mixture. Promotes whichever room currently contains `cellIdx` (via
// the same `vf_rooms.room_of` lookup ActivateVolumeFractions/InjectSpecies already use) to
// new-engine-owned. If volume-fractions isn't active for this world yet, this activates it
// first (same lazy-activation contract as kInjectGasSpecies), so a promotion message sent
// before any gas has ever been injected still resolves to a real room rather than being a
// no-op. Once promoted, the gas kernels (`StepConduction`, `StepGasPressure`, `StepGasDisplacement`
// and the emitters) skip the room's cells, and the liquid kernels skip the ones the mixture
// holds mass in (`gas::IsRoomOwned`, `gas::IsMixtureOwnedCell`). A world that never sends it is
// unaffected, the same "byte-identical until sent" guarantee every extension message here
// holds.
//
// Promote-only: there is no demote message. What should happen when the building that caused
// a promotion is deconstructed is still open.
constexpr int32_t kPromoteRoom = 0x4F4E4935;  // "ONI5"

#pragma pack(push, 4)
struct PromoteRoomMessage {
  int32_t cellIdx;
};
#pragma pack(pop)
static_assert(sizeof(PromoteRoomMessage) == 4, "PromoteRoomMessage layout drift");

// THE READ SIDE OF THE SAME ROOM: what a room actually CONTAINS. `SIM_DebugRoomOwned` answers
// one boolean and discards the id; without this, a managed consumer that wants a room's
// pressure or composition has to approximate the room from outside, per cell, through the
// per-cell exports -- and a radius-N diamond leaks through walls.
//
// DEFINED IN `abi/sim_ext_api.h`, not here, and this is an alias rather than a
// second copy: this type appears in an EXPORT'S SIGNATURE, and two definitions of
// a type a function takes cannot be checked against each other by any compiler.
// One definition, so there is nothing to drift. The field list and its comments
// are in that file; everything either side of this line is the reasoning, which
// stays here. See sim_ext_api.h's "HOW THE TWO HEADERS ARE KEPT IN STEP".
using RoomAggregate = ::OniRoomAggregate;
static_assert(sizeof(RoomAggregate) == 140, "RoomAggregate layout drift");

// THE SAME READ SIDE FOR A CONDUIT RUN. Defined in `abi/sim_ext_api.h`
// for the same reason as the room aggregate: it appears in an export's signature. The
// reasoning -- why the partition is pushed rather than derived, why the handles are re-bound
// after every rebuild, and what the MIX policy does -- is in `sim/conduits.h`, beside the code.
using ConduitNetworkAggregate = ::OniConduitNetworkAggregate;
static_assert(sizeof(ConduitNetworkAggregate) == 140, "ConduitNetworkAggregate layout drift");
static_assert(offsetof(ConduitNetworkAggregate, species) == 64, "species[] offset drift");
static_assert(offsetof(ConduitNetworkAggregate, massBySpeciesKg) == 80,
              "massBySpeciesKg[] offset drift");
static_assert(offsetof(ConduitNetworkAggregate, trappedGasKg) == 112, "trapped* offset drift");
static_assert(offsetof(ConduitNetworkAggregate, phaseRatePerSecond) == 132,
              "phaseRatePerSecond offset drift");
static_assert(offsetof(ConduitNetworkAggregate, linkCount) == 136, "linkCount offset drift");
using ConduitPhaseProposal = ::OniConduitPhaseProposal;
static_assert(sizeof(ConduitPhaseProposal) == 44, "ConduitPhaseProposal layout drift");
static_assert(offsetof(ConduitPhaseProposal, sourceMassKg) == 20, "proposal offset drift");
// The other half of that check -- that a room has slots for at least one cell's worth of
// species -- lives in `sim/gas_rooms.h`, which is the file that has both this constant and
// `gas::kMaxSpeciesPerCell` in scope. This header deliberately does not include the gas ABI.

// Sixth entry, for an "updraft water" showcase: a real physics change, not a per-frame managed-side swap hack --
// flips the vertical sign of one element's turn in StepFlow's liquid branch (physics.h), so
// that element falls up and pools under solid ceilings instead of on floors. Every other
// element's liquid turn, and this element's own gas/solid behaviour, is untouched.
//
// `elementIdx` is an ElementTable index (not a SimHashes id), same convention as
// kInjectGasSpecies. -1 clears the effect back to normal gravity for every element -- the
// default (World::InvertedGravityElement's sentinel 0xFFFF) is "no element", so a world that
// never sends this message computes byte-identical to vanilla, same guarantee every other
// message in this file holds. Only one element can be inverted at a time; sending this again
// replaces whichever element was inverted before rather than adding a second one.
constexpr int32_t kSetInvertedGravityElement = 0x4F4E4936;  // "ONI6"

#pragma pack(push, 4)
struct SetInvertedGravityElementMessage {
  int32_t elementIdx;  // -1 clears the effect
};
#pragma pack(pop)
static_assert(sizeof(SetInvertedGravityElementMessage) == 4,
              "SetInvertedGravityElementMessage layout drift");

// Seventh entry: the API half of the molecular-mass correction. Klei's
// element table stores ATOMIC mass for the diatomic gases (Oxygen 15.9994 rather than
// 31.9988, Hydrogen 1.00794 rather than 2.01588), which made every gas pressure this project
// reported for a diatomic exactly 2x high -- measured live, not assumed. The native side now
// seeds its own corrections for the four gases ONI actually ships wrong
// (ElementTable::SeedDefaultMolecularMasses, sim/world.h), so the offline driver and the test
// binaries are right standalone with no message at all.
//
// This message exists so the correction is not a hardcoded native secret. The framework
// pushes MaterialPropertyRegistry's MolecularMassGPerMol entries through it at load, which
// makes the managed registry the authority and means a THIRD-PARTY mod that registers a new
// gas in that registry gets a correct native pressure for it without touching the SimDLL at
// all: what the SimDLL changes is reachable through the API.
//
// `idHash` is a SimHashes value -- Klei's `Hash.SDBMLower` of the element id string, the same
// number `Element::id` carries. NOT an ElementTable index, unlike kInjectGasSpecies's
// `speciesIdx`: an index is only meaningful against one particular loaded table, and this
// override has to survive the table being reloaded. `gPerMol <= 0` removes an override, falling
// that element back to Klei's own figure.
//
// Only the mole/pressure path reads the result (gas_mixture.h's CellMoles and
// MolesFromSpeciesList). No vanilla kernel does, so a world that never sends this still
// computes byte-identically -- the same guarantee every other message in this file holds,
// except that here the DEFAULTS are already seeded, so "never sent" means "keeps the four
// native corrections", not "keeps Klei's wrong numbers".
//
// SINCE STAGE 5 this is a convenience alias for the `sim.molecular_mass` attribute of the
// per-element registry below, and it is IMMEDIATE rather than deferred -- the value lands
// before the call returns. Both halves of that are corrections, not conveniences: the store
// is now read by exports that take no worker barrier, so a message that wrote it from the
// worker thread would race, and a deferred write would cost a tick of latency.
constexpr int32_t kSetMolecularMass = 0x4F4E4937;  // "ONI7"

#pragma pack(push, 4)
struct SetMolecularMassMessage {
  int32_t idHash;  // SimHashes value, not an ElementTable index
  float gPerMol;   // <= 0 removes the override
};
#pragma pack(pop)
static_assert(sizeof(SetMolecularMassMessage) == 8, "SetMolecularMassMessage layout drift");

// Eighth entry, for the framework's power -> heat rule (OniFramework.PowerHeat).
//
// Sets a second, independent operating-heat rate on one registered building, summed with
// the game's own `OperatingKilowatts` at the same post-clamp line in StepBuildingHeatExchange.
// `handle` is the building's sim handle -- the value AddBuildingHeatExchange handed back,
// which the managed side reads as `StructureTemperaturePayload.simHandleCopy`.
//
// WHY A SEPARATE FIELD RATHER THAN REUSING operating_kilowatts. The managed rule could push
// its number by patching `StructureTemperaturePayload.OperatingKilowatts` so vanilla's own
// ModifyBuildingHeatExchange carries it. It was written that way first, and that shape has
// two real problems:
//
//   1. It lies to the sim. The sim then cannot tell vanilla's hand-tuned self-heat from the
//      derived power rule, so the energy ledger cannot either, and the whole point of the
//      ledger is to be able to say how much of a colony's heat came from the power grid.
//   2. The getter is small and has two call sites. If Mono inlines it, the Harmony detour is
//      bypassed and the rule silently does nothing -- a failure with no symptom.
//
// A field the sim owns has neither problem: the managed side sends a number when the number
// changes, the sim applies it every substep, and `bldgwasteheat` in the energy ledger keeps
// it separable from vanilla self-heat for the whole run.
//
// PRESERVED ACROSS ModifyBuildingHeatExchange. Klei's Modify is a wholesale replacement of
// the record, and the game sends one whenever a building's payload is dirty -- every
// operational change, every temperature edit. A naive replacement would silently zero this
// field at unpredictable moments, so ModifyBuilding carries it over explicitly, the same way
// it already carries the no-longer-overheated event.
//
// Byte-identical until sent, like every other message in this file: the field defaults to 0
// and adds 0 to the rate.
constexpr int32_t kSetBuildingWasteHeatKilowatts = 0x4F4E4938;  // "ONI8"

#pragma pack(push, 4)
struct SetBuildingWasteHeatKilowattsMessage {
  int32_t handle;   // the building's sim handle, not a game cell
  float kilowatts;  // replaces any previous value; 0 switches the rule off for this building
};
#pragma pack(pop)
static_assert(sizeof(SetBuildingWasteHeatKilowattsMessage) == 8,
              "SetBuildingWasteHeatKilowattsMessage layout drift");

// Ninth entry: building exhaust heat.
//
// Klei's `StructureTemperatureComponents.ExhaustHeat` is the widest heat-deletion channel in
// vanilla. It destroys energy on four separate paths (game build 744825):
//
//   1. managed, before any message exists: delivery is scaled by
//      `Mathf.Min(Grid.Mass[cell], 1.5f) / 1.5f` and the remainder is simply dropped;
//   2. native, ApplyCellEnergy: a vacuum cell (element state 0) refuses the whole lump;
//   3. native, ApplyCellEnergy: so does a cell under 0.001 kg;
//   4. native, ApplyCellEnergy: the `maxTemperature` ceiling, which is NOT 10000 K --
//      `OnSpawn` sets it from `Overheatable.OverheatTemperature`, `BuildingDef.Overheatable`
//      defaults to true and `BuildingDef.OverheatTemperature` defaults to 348.15 K, so an
//      ordinary machine in a room already above 75 C delivers nothing at all.
//
// This message moves the whole of that computation into the sim as a RATE the sim owns, in
// exactly the shape kSetBuildingWasteHeatKilowatts already proved out, and gives the
// undelivered remainder somewhere real to go: the building's own body. `handle` is the
// building's sim handle -- the value AddBuildingHeatExchange handed back, which the managed
// side reads as `StructureTemperaturePayload.simHandleCopy`.
//
// WHY A RATE AND NOT PER-CELL MESSAGES. Vanilla sends one ModifyCellEnergy per covered cell
// per 200 ms managed tick, so the exhaust of a busy colony is hundreds of messages a frame,
// and the energy is applied on the managed clock rather than the sim's. The first live run of
// the power -> heat rule proved those two clocks do not agree -- 34.2 s of sim substep time
// against 21.1 s of GameClock over one window -- so a lump computed against `dt` on the
// managed side is charged against the wrong interval. A rate the sim integrates over its own
// substeps has neither problem, and it costs one 12-byte message per change instead of N per
// tick.
//
// The managed side must SUPPRESS vanilla's own ExhaustHeat when it sends this, or the heat is
// applied twice. The framework's ExhaustHeat patch does that with a prefix that returns
// false, covering both of vanilla's callers (StructureTemperatureComponents.Sim200ms and
// SpaceHeater.AddExhaustHeat).
//
// The delivery arithmetic is Klei's, reproduced rather than improved: the same 1.5 kg mass
// factor, the same per-cell split, the same ceiling. The ONLY change is where the undelivered
// remainder ends up. Preserved across ModifyBuildingHeatExchange for the same reason the waste
// heat field is. Byte-identical until sent: both fields default to 0 and 0 kW exhausts nothing.
constexpr int32_t kSetBuildingExhaust = 0x4F4E4939;  // "ONI9"

#pragma pack(push, 4)
struct SetBuildingExhaustMessage {
  int32_t handle;         // the building's sim handle, not a game cell
  float kilowatts;        // 0 switches exhaust off for this building
  float maxTemperature;   // the ceiling Klei passes; normally Overheatable.OverheatTemperature
};
#pragma pack(pop)
static_assert(sizeof(SetBuildingExhaustMessage) == 12, "SetBuildingExhaustMessage layout drift");

// Tenth entry, and the reason the exhaust bounce above is not just a
// slower way to cook every machine in a vacuum: a radiative outlet for a building body.
//
// PORTED FROM STATIONEERS, structurally, from `AtmosphereHelper.CalculateEntropy` /
// `CalculateThingEntropy` (Assets.Scripts.Atmospherics). Its shape, which is what makes a
// radiation term safe to add to a game that already has conduction:
//
//   if (internal.T <= globalAtmosphere(grid).T) return 0;      // one-way: only ever sheds
//   Tref  = global.T < 1 ? 50 : global.T;                      // vacuum floor
//   Tsink = Tref;
//   if (world != null && Tref < world.T)                       // local air blends in,
//       Tsink = Lerp(Tref, world.T, clamp01(world.moles/...)); // weighted by ITS PRESSURE
//   return Clamp(radiationFactor * surfaceArea * internal.HeatExchangeRatio() * curve(dT),
//                0, internal.TotalEnergy / 2);                 // half-energy overshoot guard
//
// The load-bearing part is the sink lerp. In a pressurised room the sink temperature IS the
// room temperature, the gradient collapses, and radiation contributes ~nothing on top of the
// conduction ONI already runs. In vacuum the local term drops out, the sink falls back to the
// environment reference, and radiation is the only outlet left -- which is exactly the
// physical claim, and exactly the case the exhaust bounce creates.
//
// WHAT COULD NOT BE PORTED. Stationeers evaluates the flux from `AtmosphericsManager.
// EntropyCurve`, a hand-authored Unity AnimationCurve whose keyframes live in a scene asset
// rather than in code. That slot takes Stefan-Boltzmann here -- sigma * A * (T^4 - Tsink^4),
// with sigma in kW so the result is kJ/s like every other rate in this sim -- which is the law
// the curve is a tuned approximation of. Everything else above is reproduced as-is, including
// the one-way test and the half-energy clamp.
//
// `radiationFactor` is emissivity, 0..1, and DEFAULTS TO 0 -- radiation is off until something
// asks for it, so a world that never sends this message computes byte-identically, the same
// guarantee every other message in this file holds. Turning it on globally is a policy
// decision and therefore belongs to the API, not to a native default:
// the framework's OniFramework.Radiation owns it.
//
// `surfaceAreaM2` <= 0 means "use the building's own footprint", CellCount() square metres,
// which is the right answer for almost every building and saves the managed side computing it.
constexpr int32_t kSetBuildingRadiation = 0x4F4E493A;  // "ONI:"

#pragma pack(push, 4)
struct SetBuildingRadiationMessage {
  int32_t handle;         // the building's sim handle, not a game cell
  float radiationFactor;  // emissivity 0..1; 0 switches radiation off for this building
  float surfaceAreaM2;    // <= 0 means the building's own CellCount()
};
#pragma pack(pop)
static_assert(sizeof(SetBuildingRadiationMessage) == 12,
              "SetBuildingRadiationMessage layout drift");

// Eleventh entry, added alongside kSetBuildingRadiation: the far side of the
// radiative exchange.
//
// This is the sim's stand-in for a Stationeers planet's atmosphere, a real reservoir on the
// receiving end: radiated energy in Stationeers MOVES rather than disappearing. Heat must not
// be deleted, so that is reproduced here: energy radiated out of a building
// is charged to its own ledger bucket (World::EnergyLedger::radiated_to_environment) and is
// therefore a NAMED boundary crossing with a receiver, not a hole. Flagship 2's Dynamic Planet
// is what eventually gives that bucket a real body with a temperature that moves; until then
// the environment is an infinite reservoir at a fixed temperature, which is a stub, and is
// documented as one.
//
// `kelvin` <= 0 restores the default. The default is 0 and radiation is off by default anyway,
// so a world that never sends this is unaffected either way.
constexpr int32_t kSetEnvironmentTemperature = 0x4F4E493B;  // "ONI;"

#pragma pack(push, 4)
struct SetEnvironmentTemperatureMessage {
  float kelvin;
};
#pragma pack(pop)
static_assert(sizeof(SetEnvironmentTemperatureMessage) == 4,
              "SetEnvironmentTemperatureMessage layout drift");

// Twelfth entry, added for reproducibility. Sets the sim's random stream to an
// exact state, which is the one input to a run that is otherwise not reproducible.
//
// WHY THIS EXISTS. `AllocateCells` -- the message a main-game LOAD boots through -- seeds the
// stream from the wall clock:
//
//     g->world.SetRandomState(static_cast<uint32_t>(time(nullptr)));   // simdll.cpp
//
// That is Klei's own behaviour, faithfully reproduced (its `AllocateCells` passes
// `_time64(NULL)` where `SimData_InitializeFromCells` passes the world seed), and it is
// deliberately NOT "fixed" there: a save loaded twice is supposed to shuffle its gas
// differently. This message does not change that default. It gives a caller that wants a
// reproducible run a way to ask for one, and a world that never sends it behaves exactly as
// before -- the same byte-identical-until-sent guarantee every other id in this file makes.
//
// WHAT IT COSTS TO NOT HAVE IT, measured against a recorded 636x404 cluster world: two runs of the identical command, ten ticks each, diverged in 1,897
// element cells, 40,151 mass cells and 50,189 temperature cells. Runs that happened to
// complete inside the same wall-clock second agreed; runs that straddled a second boundary
// did not, which points at a wall-clock seed. The divergence starts wherever gas
// moves, because the shuffle is what draws from the stream (physics.h's `NextRandom`).
//
// The consequences reach past the visualizer, which is why this is a sim-level addition
// rather than a tool-level workaround:
//
//   - Any live rig whose result depends on gas is not reproducible across game loads, which
//     collides with the project's "one run is not a result, two consecutive green runs"
//     rule. That rule has been assuming a determinism that a loaded world does not have.
//   - A scrub timeline cannot restore a checkpoint and re-simulate: `rng_` is not in
//     `sim/saveblob.h`, so a save blob does not carry the stream position. Pairing a blob
//     with this message is what makes a checkpoint complete.
//   - A bug report of the form "load this save, wait, watch it go wrong" is reproducible
//     only if the stream is.
//
// PAIRS WITH `SIM_DebugRandomState`, which already exports the getter (simdll.cpp). Read the
// state, keep it beside the save blob, send it back to return to that point in the stream.
//
// TIMING. This is a queued message like every other id here, so it is drained by the frame
// AFTER the one it was sent during (`DrainQueue` runs at the top of `RunFrame`, before
// `StepPhysics`). Send it after the boot sequence and before the first frame whose physics
// should be reproducible; sending it mid-run is legal and simply re-points the stream from
// that frame on.
//
// NOT A SEED, A STATE. The value is written into the LCG's state verbatim -- `SetRandomState`
// does no scrambling, matching `SimData::constructor`, which stores its seed argument raw
// (world.h). So round-tripping a value read from `SIM_DebugRandomState` reproduces the stream
// exactly; there is no seed-to-state transform to account for. Every uint32 is a legal state,
// 0 included.
constexpr int32_t kSetRandomState = 0x4F4E493C;  // "ONI<"

#pragma pack(push, 4)
struct SetRandomStateMessage {
  uint32_t state;
};
#pragma pack(pop)
static_assert(sizeof(SetRandomStateMessage) == 4, "SetRandomStateMessage layout drift");

// ---------------------------------------------------------------------------------------
// SCHEDULING STATE -- the rest of what a save blob does not carry.
//
// `kSetRandomState` above is one half of a complete checkpoint; this is the other half.
// Without it, a checkpoint restored and replayed reproduces the original run for exactly ONE
// frame and then diverges -- 26 of 35 replayed ticks wrong, worst 10.537 kg of total mass, on
// a 40-tick sweep of a recorded world. The random stream is NOT the cause: on most diverged ticks the live LCG state
// read back identical and the cell arrays still differed, so the streams were in step and
// the physics was not.
//
// The cause is that both `AllocateCells` (simdll.cpp, the message a main-game LOAD boots
// through) and `Load` itself reset SimData scheduling state to its fresh-allocation values:
//
//     first_physics_substep = true      displace_rotation = 0
//     skip_physics_frames   = 1         pressure_dir      = -1
//     substep_carry         = 0         shuffle_dir       = -1
//     vf_active             = false
//
// A sim that has been running to tick T holds none of those values, and none of them are in
// the save format -- they belong to the SimData object, not to the world contents, which is
// exactly what `Sim::displace_rotation`'s own comment says of the two counters. So a restore
// puts the right world under the wrong schedule, and the first frame that actually runs
// physics runs it in the wrong order. `displace_rotation` picks which of several equally
// good cells receives displaced gas (`rotation & 3`, `rotation & 1` in physics.h) and
// `pressure_dir` picks the column order and the horizontal pairing, so "the wrong order" is
// not a cosmetic difference: it is a different, equally valid run.
//
// PAIRS WITH `SIM_DebugSchedulingState`, the getter export (simdll.cpp), the same way
// `kSetRandomState` pairs with `SIM_DebugRandomState`. Read the state, keep it beside the
// save blob and the stream state, send all three back to return to that point in the run.
//
// WHAT IS DELIBERATELY NOT HERE. `first_frame` and `pipelined` are publication bookkeeping
// rather than schedule: `first_frame` only forces a full `Project` instead of an incremental
// one, which is the correct thing to do after a restore and publishes the same arrays either
// way, and `pipelined` says whether a worker frame is in flight -- a value that would be a
// lie the moment it was restored, since restoring cannot put a frame in flight. Setting them
// from outside would be able to make the sim publish a frame that does not exist.
//
// TIMING. Queued like every other id here, so it is drained at the top of the frame AFTER
// the one it was sent during, ahead of that frame's physics (`DrainQueue` runs before
// `StepPhysics`). That is the right place for it: the frame that drains it is the first
// frame that runs under the restored schedule. Send it immediately after the `Load`, in the
// same position as `kSetRandomState`.
//
// SKIP_PHYSICS_FRAMES. Sending 0 here is what removes the skipped frame a `Load` inserts,
// so a restored run steps physics on the very next frame exactly as the original did. That
// skip is Klei's behaviour for a real load and stays the default; this only lets a caller
// that is reconstructing a run say it does not want it.
constexpr int32_t kSetSchedulingState = 0x4F4E493D;  // "ONI="

// DEFINED IN `abi/sim_ext_api.h`, not here, and this is an alias rather than a
// second copy: this type appears in an EXPORT'S SIGNATURE, and two definitions of
// a type a function takes cannot be checked against each other by any compiler.
// One definition, so there is nothing to drift. The field list and its comments
// are in that file; everything either side of this line is the reasoning, which
// stays here. See sim_ext_api.h's "HOW THE TWO HEADERS ARE KEPT IN STEP".
using SetSchedulingStateMessage = ::OniSetSchedulingStateMessage;
static_assert(sizeof(SetSchedulingStateMessage) == 20,
              "SetSchedulingStateMessage layout drift");

// ---------------------------------------------------------------- kSetStableTicks
//
// THE FOURTH PART OF A CHECKPOINT, and the last per-cell state a save blob does not carry.
//
// `World::stable_ticks_` mirrors the game's per-cell stability counter: one byte a cell, persistent across
// frames, holding an unstable solid's countdown to falling in its low five bits. The save
// format has no field for it -- `SaveBlob` carries cells, disease, backwall, gas and room
// promotion and nothing else -- and every `Allocate` re-assigns the whole array to the
// all-`0x1f` reroll sentinel, which every `Load` goes through. A restored world has
// therefore forgotten when each grain of sand was going to fall.
//
// That is not a cosmetic difference, and it is not confined to the cells that fall.
// `World::StableTicksRemaining` rerolls a sentinel cell from the random stream, and that
// reroll is the ONLY draw the unstable path makes. So a restore both drops sand on the
// wrong tick and pushes the stream to the wrong place, which is why a replay could still
// diverge with `kSetRandomState` and `kSetSchedulingState` both in place.
//
// MEASURED, on the 636x404 reference corpus at seed 12345, one replayed frame past a
// checkpoint: 2,234 of 256,944 cells differ, the total mass is off by +20.392861 kg in two
// cells of about 10 kg, and one of them holds a solid in the replay where the run has
// vacuum. The seven scheduling counters agree on both sides at that tick except
// `shuffle_dir`, which differs because the gas sweep flips it once per gas cell reached and
// one displaced solid changes the parity. Which seeds show it is a property of the world:
// the divergence needs an unstable solid inside the active region during the replayed
// frames, so at interval 10 over 40 ticks seeds 7, 42 and 999 reproduce and 3 and 12345 do
// not. Reading that as "short checkpoint intervals diverge" is a mistake this project made
// and then measured its way out of.
//
// PAIRS WITH `SIM_DebugStableTicks`, the getter export (simdll.cpp), which returns the
// array's length so a caller never has to derive the padded cell count itself:
//
//     const int32_t n = SIM_DebugStableTicks(nullptr, 0);
//     std::vector<uint8_t> buf(n);
//     SIM_DebugStableTicks(buf.data(), n);
//
// VARIABLE LENGTH, unlike every other message here: the header is followed by exactly
// `count` bytes, one per PADDED cell -- (width + 2) * (height + 2), the same indexing the
// rest of the sim uses -- and a payload whose length disagrees with its own `count`, or
// whose `count` disagrees with this world's, is rejected and logged rather than applied
// partially. Send it after the `Load` that restores the world it belongs to, in the same
// position as `kSetRandomState` and `kSetSchedulingState`: it is sized against the world
// that is loaded when it drains, not the one that was loaded when it was sent.
//
// Queued like the rest, so it is drained at the top of the frame after the one it was sent
// during, ahead of that frame's physics.
constexpr int32_t kSetStableTicks = 0x4F4E493E;  // "ONI>"

#pragma pack(push, 4)
struct SetStableTicksMessage {
  int32_t count;  // padded cells that follow this header, one byte each
  // uint8_t ticks[count];
};
#pragma pack(pop)
static_assert(sizeof(SetStableTicksMessage) == 4, "SetStableTicksMessage layout drift");

// The FIFTH thing a save blob does not carry, and the last one measured to matter:
// World::disease_accum_ (a float per padded cell -- the growth remainder a cell has banked
// toward its next whole disease unit) and World::disease_infest_ (a byte per padded cell --
// how long that cell has been infested). Payload is { int32 count }, then `count` floats,
// then `count` bytes. The second variable-length message in this set, and the only one
// carrying two arrays.
//
// The game's saved disease record is eight bytes -- a hash and a count -- so neither array is in the save
// format, and World::FromBlob zeroes both on every load to match. That is faithful and
// deliberately unchanged: a colony reloaded in the main game restarts its growth remainders
// at zero exactly as vanilla does. This message exists so a caller RECONSTRUCTING a run can
// opt out of that, the same bargain kSetStableTicks offers for the unstable solids.
//
// Measured, and it is not a corner case. A replay check that hashes only elementIdx, mass and
// temperature -- three of the twelve arrays GameDataUpdate publishes -- reports every tick
// reproducing bit for bit. Widened to all twelve, the same 40-tick sweep on the reference corpus (3,739 diseased cells across four disease
// types) failed 26 of 35 replayed ticks, in the disease arrays alone, with the random stream
// in step and the total mass bit-identical, at every seed tried. Exact stops and short
// replays still passed: a growth remainder restarted from zero takes several ticks to fall
// a whole unit behind the run it is replaying, which is what made this invisible to any
// check that only looked one frame past a checkpoint.
//
// ONE OF THE CHECKPOINT COMPONENTS. This line used to name the whole list and say "five
// things"; the list has grown twice since (kSetRegistryState, then kSetExtCellState) and the
// count sat here wrong through both. The authoritative list is at kSetExtCellState, the
// newest component, and every other mention in this file now points there instead of keeping
// its own copy -- a count repeated in three places is a count that is wrong in two of them.
//
// Pairs with the SIM_DebugDiseaseGrowth export, which is the getter and also reports the
// length, so a caller never derives the padded cell count itself.
//
// Queued, and sized against the world that is loaded when it drains: send it after the Load
// it belongs to. A length that is not that world's padded cell count is rejected and logged
// rather than partially applied -- half a restored growth field is worse than none, because
// the half that did not land is the zeroed state the caller is trying to get out of.
constexpr int32_t kSetDiseaseGrowth = 0x4F4E493F;  // "ONI?"

#pragma pack(push, 4)
struct SetDiseaseGrowthMessage {
  int32_t count;  // padded cells that follow this header
  // float   accum[count];
  // uint8_t infest[count];
};
#pragma pack(pop)
static_assert(sizeof(SetDiseaseGrowthMessage) == 4, "SetDiseaseGrowthMessage layout drift");

// ---------------------------------------------------------------------------------------
// kSetRegistryState -- the SIXTH checkpoint component: the four component registries.
//
// `AllocateCells` clears all four of them (`simdll.cpp`: buildings, chunks, flow, radiation)
// and no save blob carries any of it. In the game that is right and invisible, because the
// game re-registers every handle it holds after a load. A tool driving the DLL is not holding
// those handles, so for it a restored world is a world with no buildings, no element chunks,
// no emitters and no consumers in it.
//
// RE-SENDING THE ORIGINAL REGISTRATION MESSAGES IS NOT THE SAME THING, and that is measured
// rather than argued. A registration is a handle on a body that then evolves every substep, so replaying the boot
// records restores the topology at BOOT values and a checkpoint is restored with a registry
// stale by its own age. Shortening the checkpoint interval made it WORSE -- 3 of 17 replayed
// ticks reproducing at interval 2 against 9 at interval 10 -- which is the signature of stale
// state rather than of a long replay. Exact stops kept passing throughout, because a stop
// restores and publishes without stepping physics, so a stale registry never gets to act.
//
// The payload is the opaque blob `sim/registry_state.h` produces, and its header carries its
// own magic and version. A caller never parses it and never mirrors any of the sim's
// bookkeeping to build one, which is the reason for a blob rather than a set of typed
// messages: the typed shape needed the caller to recompute chunk heat capacities and building
// registration fields from the element table, and would still have left the flow accumulators
// and the radiation emitters' timers uncarried.
//
// ONE OF THE CHECKPOINT COMPONENTS -- for a while, the last of them. The complete list is at
// kSetExtCellState; see the note at kSetDiseaseGrowth for why it lives in exactly one place.
//
// Pairs with SIM_DebugRegistryState, which is the getter and reports the size, so a caller
// never has to guess a buffer length.
//
// Queued, so it drains at the top of the next frame ahead of that frame's physics: send it
// after the Load it belongs to. Applied whole or not at all -- a blob that is truncated, or
// from a build whose structs changed, is rejected and logged rather than leaving half a
// colony registered.
constexpr int32_t kSetRegistryState = 0x4F4E4940;  // "ONI@"

// No struct: the payload IS the blob, from its first magic byte to its last. The length the
// message carries is the blob's length.

// ---------------------------------------------------------------------------------------
// THE PER-CELL EXTENSION REGISTRY -- kRegisterCellProperty and kSetCellProperty.
//
// Every id above this line is a BESPOKE extension. Adding one number per cell costs an id
// here, a struct here, a `std::vector` member in `world.h`, a line in `World::Allocate`, a
// handler and a `DrainQueue` line in `simdll.cpp`, and -- if the number is meant to survive a
// save -- a field in `saveblob.h` plus a brand new `kSaveVersion` integer. This registry
// replaces all of that with one registration.
//
// The edit count is the least interesting reason it exists. Two others matter more.
//
// FIRST: THE PERSISTENCE EDIT IS INVISIBLE WHEN IT IS SKIPPED. `kSetCellThermalMassBonus`
// (the first id in this file) has no `saveblob.h` field and no save version. The gas-mixture
// triple has both. Nothing in either place records whether that difference was a decision or
// an omission, and the first mod to store something in a thermal-mass bonus would have found
// its values silently gone after a save/load. An ad-hoc array makes persistence an implicit
// choice that nobody has to state, review, or even notice. So `persist` below is a REQUIRED
// field of registration with no default: a registration that does not choose one is rejected.
//
// SECOND: THE SAVE VERSION IS A GLOBAL ORDERED NAMESPACE, AND IT DOES NOT SCALE PAST US.
// Today each saved array claims the next integer -- 15 is Klei's own, 16 the gas mixture, 17
// the room promotion. Two mods that each add a property independently collide by
// construction, and neither can be written without knowing about the other. That, not the
// boilerplate, is the real blocker for third-party extension data. A single self-describing
// section makes adding a property stop bumping the save version at all.
//
// A `kSaved` property is written into the save's self-describing extension section
// (saveblob.h).

// The scalar a property stores per cell -- per COMPONENT of a cell, once `arity` is above 1.
// One type-erased byte vector plus a stride, allocated from the same `World::Allocate` that
// sizes every other per-cell array, so an extension property is exactly as long as
// `insulation_` or `strength_` (times its arity) and stays that way for the life of the world.
enum ExtScalarType : int32_t {
  kExtF32 = 0,
  kExtU8 = 1,
  kExtU16 = 2,
  kExtI32 = 3,
  // 4 is deliberately unused: state-dump tools share this numbering and spend 4 on their
  // own u32 (the RGBA property textures), which has no per-cell extension equivalent.
  kExtElementIdx = 5,
};

// kExtElementIdx. Stored exactly like kExtU16 -- two bytes a
// component, the setter keeps the low two bytes -- but the type says what the bytes MEAN: an
// index into the element table. That is what lets a load fix it.
//
// The game sorts its element table, so an index saved under one set of mods names a different
// element under another (Mod 2's four elements land at 133 and move every gas). Since
// kSaveVersionElementPalette (19) the blob carries the table it was written against, and a
// `kSaved` record of this type is rewritten through that palette on load, on both load paths,
// whether or not a loaded mod registered it: an orphan carried for an uninstalled mod is
// remapped too, because the next save writes it under the CURRENT table's palette.
//
// kExtNoElement (0xFFFF, the sim's own "no element", as a no-element backwall) passes through
// untouched. An index whose element the loading table does not hold, or that is past the end of
// the palette, becomes kExtNoElement, and the `Load` handler reports how many components did.
// A blob with no palette (v18 and older) is trusted as saved.
//
// A kRehydrated or kCheckpointOnly property of this type is never remapped: neither travels in
// a save, and a checkpoint is restored against the table it was taken with.
constexpr uint16_t kExtNoElement = 0xFFFF;

// What a load owes this property. REQUIRED at registration; there is no default.
//
//   kSaved           written into the save blob and restored from it, like the gas mixture.
//   kRehydrated      the sim must NOT carry it; the registering mod re-pushes it after every
//                    load. This is a CLAIM, not a shrug -- the registry counts writes since
//                    the last allocate so it can say which properties were promised and have
//                    not arrived. (What `kSetCellThermalMassBonus` has always assumed without
//                    anywhere to say so.)
//   kCheckpointOnly  absent from an ordinary save, but carried in the checkpoint blob a
//                    replay harness restores -- the same family as kSetRandomState,
//                    kSetSchedulingState, kSetStableTicks, kSetDiseaseGrowth and
//                    kSetRegistryState. Extension arrays belong to that family and have never
//                    been counted in it: `physics.h`'s conduction kernel reads the thermal
//                    bonus, so a replay missing it conducts differently and diverges. A
//                    standalone tool driving the DLL loads no mods, so no owner exists
//                    there to re-push anything.
//
// kRehydrated and kCheckpointOnly are carried identically by the sim today. The distinction is
// declarative and kept on purpose: it is the only thing that can answer "after this load, which
// properties was somebody supposed to re-push and has not".
enum ExtPersistence : int32_t {
  kSaved = 0,
  kRehydrated = 1,
  kCheckpointOnly = 2,
};

// IMMEDIATE, not queued -- the one message in this file that is. Registration must close
// before the first `World::Allocate`, because every per-cell array in the world is the same
// length for the life of that world and everything in `world.h` assumes it. A queued message
// drains at the top of the next frame, which is long after the `AllocateCells` or
// `SimData_InitializeFromCells` that a load boots through. So this is handled on the calling
// thread the same way AllocateCells is, and a registration that arrives after the first
// allocate is REFUSED AND LOGGED rather than serviced by a resize.
//
// RETURNS the property index as a `const int32_t*` (the return of `SIM_HandleMessage`), or a
// pointer to a negative `ExtRegisterResult` on refusal. The index is what `kSetCellProperty`
// addresses, and it is stable for the life of the process. `SIM_ExtCellPropertyIndex` looks
// the same number up by name later, for a caller that did not keep it.
//
// NAMING is `<owner>.<property>` and it is a PERMANENT ON-DISK CONTRACT once stage 2 lands:
// the name is the key in the save blob, so renaming one orphans data in every save that has
// it. Rules, all refusals rather than repairs:
//
//   * exactly one '.', both segments `[a-z0-9_]`, owner 2-31 chars, property 1-31, and the
//     whole name at most 47 chars so it fits `name[48]` with its terminator. (The 48-byte
//     field is the on-disk width; the segment caps are the readability rule. A 31+31 name is
//     legal by segment and rejected by the total, with a message that says so.)
//   * LOWERCASE IS ENFORCED, NOT NORMALISED. Normalising would silently map `Contamination`
//     and `contamination` onto one store. Two names must never become one property.
//   * a duplicate is rejected NAMING BOTH OWNERS -- a collision that logs one name is a
//     collision nobody can act on.
//   * `sim.` and `oni.` are reserved for first-party properties, so a first-party key is
//     visibly first-party in a blob a third party is reading.
//
// The owner segment is not meant to be free text at the call site. The framework composes it
// from the identity the mod already declared to `FrameworkVersion.Require(major, minor,
// consumer)`, so a mod cannot register under another mod's prefix by accident. Native cannot
// see that -- it enforces only the shape above -- which is why the composition belongs in the
// managed wrapper and not here.
//
// Reverse-DNS (`com.author.mod.prop`) was considered and rejected: longer in a field stored
// per property in every blob, and it assumes authors own a domain.
constexpr int32_t kRegisterCellProperty = 0x4F4E4941;  // "ONIA"

#pragma pack(push, 4)
struct RegisterCellPropertyMessage {
  char     name[48];     // NUL-terminated; see the naming rules above
  int32_t  type;         // ExtScalarType
  int32_t  persist;      // ExtPersistence -- REQUIRED, no default
  int32_t  arity;        // components per cell, >= 1; see kExtMaxArity
  uint32_t defaultBits;  // reinterpreted per `type`; every COMPONENT of every cell starts here
};
#pragma pack(pop)
static_assert(sizeof(RegisterCellPropertyMessage) == 64,
              "RegisterCellPropertyMessage layout drift");

// ARITY -- components per cell, added for stage 2.
//
// Stage 1 stored one scalar per cell, which is what `thermal_mass_bonus_` is. The gas-mixture
// triple stage 2 has to absorb is not: `gas_occupied_mask_` is one `uint8` per cell, but
// `gas_species_` is eight `uint16` and `gas_mass_` eight `float`. A registry that cannot hold
// those cannot hold the largest piece of extension data this project has, so `arity` closes
// that rather than special-casing the gas layout inside the save format the registry exists to
// generalise.
//
// LAYOUT IS CELL-MAJOR: component `c` of cell `p` sits at `(p * arity + c) * stride`. That is
// byte-for-byte what `gas_species_[p * kMaxSpeciesPerCell + slot]` already does, so the
// migration is a move rather than a relayout -- one fewer variable in a change that has enough.
// (`gas_mixture_abi.h`'s header comment describes an aspirational slot-major SoA; the shipped
// implementation is cell-major, and this follows the implementation.)
//
// The cap is a sanity bound, not a design limit: it keeps a garbled `arity` from asking
// `World::Allocate` for an absurd allocation, and 8 is what the gas layer actually needs.
constexpr int32_t kExtMaxArity = 64;

// Refusal codes, returned negated (so -kExtRegisterDuplicate comes back). Distinct values
// because "registration failed" is not actionable and "that name is already taken by mod X" is.
enum ExtRegisterResult : int32_t {
  kExtRegisterBadName = 1,     // shape, charset, case or length
  kExtRegisterDuplicate = 2,   // already registered; the log names both owners
  kExtRegisterReserved = 3,    // `sim.` / `oni.` from a non-first-party caller
  kExtRegisterBadType = 4,     // type or persistence outside the enums
  kExtRegisterClosed = 5,      // arrived after the first Allocate
  kExtRegisterFull = 6,        // registry at its cap
  kExtRegisterUnsupported = 7, // kSaved before stage 2's serialiser exists
  kExtRegisterBadArity = 8,    // arity below 1 or above kExtMaxArity
  // The three below are kRegisterField's (the generic field solver, further down this file).
  // Three codes rather than one for the reason the eight above are eight: "registration
  // failed" is not actionable, and "that property is arity 4, and a field has to be arity 1"
  // is.
  kExtRegisterBadProperty = 9,   // no such property, or it is not kExtF32 arity 1
  kExtRegisterBadAttribute = 10, // attributeIdx is neither -1 nor a registered attribute
  kExtRegisterBadRule = 11,      // law, combine or decay outside its enum; a decay factor
                                 // outside [0, 1]; clampLo above clampHi
};

// Queued, like every other cell-scoped message here, so it lands on the frame AFTER the one it
// was sent during -- the same latency `SetInsulationValue` has.
//
// `valueBits` is reinterpreted per the property's registered type: bit-cast for kExtF32,
// truncated to the low byte / low two bytes for kExtU8 / kExtU16 / kExtElementIdx, taken
// verbatim for kExtI32. The value REPLACES whatever the component held; it does not accumulate, matching
// SetInsulationValue / SetStrengthValue / kSetCellThermalMassBonus.
//
// `component` addresses one lane of an arity > 1 property and is 0 for the scalar case. It is
// an explicit field rather than folded into `cellIdx` because a packed index would silently
// alias a real cell the moment a caller got the arity wrong.
//
// An out-of-range `cellIdx`, `propertyIdx` or `component` is dropped silently, the same way
// every other cell message in this file drops an invalid cell. A property index is not
// guessable -- it comes back from a registration -- so a bad one is a caller bug, not hostile
// input.
constexpr int32_t kSetCellProperty = 0x4F4E4942;  // "ONIB"

#pragma pack(push, 4)
struct SetCellPropertyMessage {
  int32_t  cellIdx;
  int32_t  propertyIdx;  // from kRegisterCellProperty / SIM_ExtCellPropertyIndex
  int32_t  component;    // 0 .. arity-1; 0 for a scalar property
  uint32_t valueBits;    // reinterpreted per the property's registered ExtScalarType
};
#pragma pack(pop)
static_assert(sizeof(SetCellPropertyMessage) == 16, "SetCellPropertyMessage layout drift");

// FIRST-PARTY PROPERTY, and the reason kSetCellThermalMassBonus still works. The thermal-mass
// bonus is no longer a `std::vector<float>` member of `World`; it is a registered property
// under the reserved `sim.` prefix, registered by `World` itself before any allocate. The old
// id is kept as a compatibility alias that writes through the registry to the same storage, so
// the original wire contract is unchanged and `physics.h`'s
// conduction kernel reads the same floats it always did.
constexpr char kThermalMassBonusProperty[] = "sim.thermal_mass_bonus";

// The four FIRST-PARTY kSaved properties, migrated off their `World` members by stage 2 so
// that `kSaveVersionExtensions` replaces the fixed v16 and v17 sections rather than being
// stacked on top of them. Registered by `World` itself, before any allocate, under the
// reserved `sim.` prefix.
//
// Their names are permanent on-disk keys from the moment the first v18 blob is written.
constexpr char kGasOccupiedMaskProperty[] = "sim.gas_occupied_mask";  // u8,  arity 1
constexpr char kGasSpeciesProperty[]      = "sim.gas_species";        // u16, arity kMaxSpeciesPerCell
constexpr char kGasMassProperty[]         = "sim.gas_mass";           // f32, arity kMaxSpeciesPerCell
constexpr char kRoomPromotedProperty[]    = "sim.room_promoted";      // u8,  arity 1

// Gas dissolved in a cell's
// liquid, in kg, one component per LANE. A lane is a soluble gas, and which gas owns which lane
// is content data that the managed side assigns and keeps stable (lane numbers are what a save
// stores). Lanes rather than per-cell species slots so that moving it with liquid is a pure
// proportional split with no slot matching. Registered `kSaved` and flagged
// `kTransportFollowsLiquidMass` by `World` itself; all zero in a world that dissolves nothing,
// so such a world's blob is unchanged.
constexpr char kDissolvedMassProperty[]   = "sim.dissolved_mass";     // f32, arity kDissolvedGasLanes
constexpr int32_t kDissolvedGasLanes = 8;

// ---------------------------------------------------------------------------------------
// kSetExtCellState -- the SEVENTH checkpoint component: the per-cell extension properties a
// save blob does not carry.
//
// THE AUTHORITATIVE CHECKPOINT LIST LIVES HERE, and nowhere else in this file:
//
//     A COMPLETE CHECKPOINT IS NINE THINGS, AND ITS RESTORE IS ONE MESSAGE MORE
//       0. kSetLoadIsRestore                  sent BEFORE the Load: skip the load-time transition
//       1. the save blob                      (SIM_BeginSave / Load)
//       2. kSetRandomState                    the random stream
//       3. kSetSchedulingState                the SimData scheduling counters
//       4. kSetStableTicks                    the unstable-solid countdowns
//       5. kSetDiseaseGrowth                  the per-cell disease-growth remainders
//       6. kSetRegistryState                  the four component registries
//       7. kSetExtCellState                   the kRehydrated + kCheckpointOnly properties
//       8. kSetCellRadiation                  the radiation field `Load` clears in Vacuum
//       9. kSetVisibilityState                the visibility mask, three buffers deep
//
// Every earlier component's comment used to carry its own copy of this list, and two of the
// three copies were stale. They point here now. If another is ever added, this list is the
// one place to change.
//
// WHY EXTENSION ARRAYS BELONG TO THIS FAMILY AT ALL. They are not inert storage the sim hands
// back: `physics.h`'s conduction kernel binds the thermal-mass bonus INSIDE the pair loop, so
// a property that is absent during a replay does not read as "missing", it silently produces a
// different conduction result. That is the exact divergence signature every one of the other
// six was found by -- an exact stop passing while a replay diverged, because a stop restores
// and publishes without stepping physics, so stale state never gets to act.
//
// AND THE CONSUMER IS NOT HYPOTHETICAL. A tool driving the DLL directly has no mods
// loaded, so there is no owner present to re-push a `kRehydrated` property after a restore.
// For that caller `kRehydrated` is indistinguishable from "lost" unless something carries it.
//
// WHICH PROPERTIES TRAVEL, and why not all of them:
//
//     kSaved            NO. The save blob already carries them, and the save blob is
//                       component 1. Carrying them here would double-restore them.
//     kRehydrated       YES. The owner promised to re-push after a load; a replay harness
//                       has no owner to keep that promise.
//     kCheckpointOnly   YES. An explicit statement that nobody will re-push it.
//
// The two are treated IDENTICALLY by this carrier, which is the point of keeping both: the
// difference is declarative -- it is what makes SIM_ExtOutstandingRehydration able to answer
// "who was supposed to re-push and has not" -- rather than mechanical.
//
// ALL-DEFAULT PROPERTIES ARE OMITTED FROM THE BLOB, AND A RESTORE RESETS THEM. Same bargain
// registry_state.h struck when it refused to carry `RadiationState::occlusion`: a megabyte a
// checkpoint to move a number nothing has written is a cost paid on every seek. A property
// absent from the blob is restored to its REGISTERED DEFAULT rather than left alone, so a
// restore puts back the world the checkpoint ran with instead of whatever the current world
// last had -- absent means "was default", never "do not touch".
//
// ORPHANS DO NOT TRAVEL EITHER. An uninstalled mod's carried bytes live in
// the save blob, which is component 1. Putting them here as well would restore them twice.
//
// The payload IS the blob, from its first magic byte to its last; there is no header struct in
// front of it, exactly as kSetRegistryState works. The blob carries its own magic and version
// and is applied whole or not at all: a record whose shape disagrees with what this build
// registered rejects the entire blob rather than reinterpreting old bytes as a new shape. A
// record naming a property no loaded mod registered is REPORTED and skipped, not fatal -- the
// same policy the save path uses, for the same reason.
//
// Queued, so it drains at the top of the next frame ahead of that frame's physics: send it
// after the Load it belongs to. Pairs with SIM_DebugExtCellState, which is the getter and
// reports the size, so a caller never has to guess a buffer length.
//
// SIM_DebugExtCellStateAll is a THIRD thing and not part of this pair: same format, but every
// registered property including kSaved, for inspection only. It has no setter, and this
// message refuses a kSaved record by name, so its blob is not a checkpoint and cannot be
// applied as one.
constexpr int32_t kSetExtCellState = 0x4F4E4943;  // "ONIC"

// No struct: the payload IS the blob. Its format is `sim/ext_state.h`'s and nobody else's.

// ---------------------------------------------------------------------------------------
// PER-FRAME PUBLISH DESCRIPTORS
// ---------------------------------------------------------------------------------------
//
// Every other way to read extension data out of this DLL is a per-call export, and every
// custom export opens with a worker-thread barrier. That is cheap for a probe reading one
// cell and wrong for anything reading a REGION every frame: the cost is paid per call, and
// the barrier is a stall hazard mid-frame.
//
// The registry already knows what exists, how long it is and what shape it has, so a
// descriptor table costs almost nothing to produce. It is published with the frame, exactly
// the way GameDataUpdate's own arrays are, and a caller binds it once per tick and reads
// through a span instead of calling across the boundary per cell.
//
// THE RULE THAT BITES, and it is the same rule Klei's arrays already live under:
//
//     THE POINTERS ARE VALID FOR THE TICK AND NOT ACROSS IT.
//
// Hold one past the PrepareGameData that produced it and it is a use-after-free waiting for
// the frame after next, because the two published frames alternate. Copy what you need, or
// re-bind every tick. That is why SIM_ExtPublishedProperties takes the frame you were handed
// rather than "the current one": the descriptors belong to a frame, so the only frame whose
// table you may read is the frame you are still holding. Asking for "the latest" would race
// the worker, which starts filling the other slot the moment the tick begins.
//
// OPT-IN, PER PROPERTY. A descriptor's bytes are COPIED into the published frame, which is
// what buys the tick-long lifetime under threading -- the registry's own storage is being
// written by kernels while the game reads. So publishing is a subscription a caller asks for
// (kPublishCellProperty) and not something every registered property gets: the gas-mixture
// triple alone is ~53 bytes per cell, and a build that reads none of it must not pay to copy
// it. Nothing is published until somebody asks. The cost lands on the mod that asked.
//
// A published property is NOT a checkpoint component and NOT a save section. It is a read
// window.
constexpr int32_t kPublishCellProperty = 0x4F4E4944;  // "ONID"

#pragma pack(push, 4)
struct PublishCellPropertyMessage {
  int32_t propertyIdx;  // from kRegisterCellProperty / SIM_ExtCellPropertyIndex
  int32_t enable;       // non-zero subscribes, zero unsubscribes
};
#pragma pack(pop)
static_assert(sizeof(PublishCellPropertyMessage) == 8,
              "PublishCellPropertyMessage layout drift");

// Queued like every other cell message here, so a subscription sent during tick N first
// appears in the table published by tick N+1. A caller that binds and finds nothing on the
// first tick has not hit a bug.
//
// The table itself: one of these per SUBSCRIBED property, in registration order, so an index
// into it is not a property index and must not be treated as one. Match on `name`.
//
// `byteCount` is `cellCount * arity * stride` and is what a span should be built from --
// derive nothing. `cellCount` is PADDED cells (the allocation's), not game cells, because
// that is what the storage is; the padding is the same padding every other per-cell array in
// this ABI carries.
// DEFINED IN `abi/sim_ext_api.h`, not here, and this is an alias rather than a
// second copy: this type appears in an EXPORT'S SIGNATURE, and two definitions of
// a type a function takes cannot be checked against each other by any compiler.
// One definition, so there is nothing to drift. The field list and its comments
// are in that file; everything either side of this line is the reasoning, which
// stays here. See sim_ext_api.h's "HOW THE TWO HEADERS ARE KEPT IN STEP".
using ExtPublishedProperty = ::OniExtPublishedProperty;
static_assert(sizeof(void*) == 8, "ExtPublishedProperty layout assumes a 64-bit build");
static_assert(sizeof(ExtPublishedProperty) == 80, "ExtPublishedProperty layout drift");

// const ExtPublishedProperty* SIM_ExtPublishedProperties(const GameDataUpdate* frame,
//                                                        int32_t* count);
//
// `frame` is the pointer PrepareGameData returned this tick. A null frame, or a frame that is
// not one of the two live publications, returns null with *count = 0 -- a refusal, not an
// empty table, because those are different facts and a caller that cannot tell them apart
// will bind garbage.

// ---------------------------------------------------------------------------------------
// THE REGISTRY AS METADATA
// ---------------------------------------------------------------------------------------
//
// `ExtPublishedProperty` above is a READ WINDOW: it says what the bytes are and where they
// live for this tick, and it deliberately carries no registered default, because a consumer
// reading a whole array does not need one to read it.
//
// A consumer that RENDERS one does. "Which cells are still at the default" is not a question
// the bytes answer by themselves -- zero is a legal value for all four scalar types -- so a
// visualizer that paints an extension property has to know the default before it can tell an
// untouched cell from a cell somebody wrote a zero into. Until this export existed the only
// place that number was reachable was `SIM_DebugExtCellStateAll`, which is a worker barrier
// and a full serialisation of the world: paying that to learn six 32-bit constants is the
// wrong trade, and paying it every frame is the trade the publish table was built to avoid.
//
// So: the descriptor table stays the per-frame path and stays barrier-free, and this is the
// metadata path. It is the registry's own record, not a frame's, so it answers the same for
// every tick of a world and a caller fetches it ONCE.
//
// `cellCount` is 0 before `World::Allocate` -- registered, not yet sized -- which is a real
// state a caller can observe, since registration closes at the first allocate and this export
// is readable before it.
//
// `published` is the subscription flag, and it is the one field here that can change between
// two calls: `kPublishCellProperty` is queued, so a caller that subscribes and immediately
// re-describes may still see 0. That is the same one-frame latency every queued message has,
// not a failure.
// DEFINED IN `abi/sim_ext_api.h`, not here, and this is an alias rather than a
// second copy: this type appears in an EXPORT'S SIGNATURE, and two definitions of
// a type a function takes cannot be checked against each other by any compiler.
// One definition, so there is nothing to drift. The field list and its comments
// are in that file; everything either side of this line is the reasoning, which
// stays here. See sim_ext_api.h's "HOW THE TWO HEADERS ARE KEPT IN STEP".
using ExtCellPropertyDesc = ::OniExtCellPropertyDesc;
static_assert(sizeof(ExtCellPropertyDesc) == 76, "ExtCellPropertyDesc layout drift");

// int32_t SIM_ExtCellPropertyCount(void);
// int32_t SIM_ExtCellPropertyDescribe(int32_t propertyIdx, ExtCellPropertyDesc* out);
//
// Count returns how many properties are registered, so the registry can be ENUMERATED rather
// than only probed by a name the caller already knew. Describe fills `out` for one index and
// returns 1, or returns 0 and leaves `out` untouched for an unregistered index, a null
// pointer, or no sim.
//
// BOTH TAKE THE WORKER BARRIER, deliberately, and that is not in tension with the paragraph
// above. Registration runs on the calling thread during a message drain and can append to the
// registry until the first allocate closes it, so reading the vector without the barrier is a
// read of a container somebody may be growing. The barrier is affordable here for exactly the
// reason it was not affordable in `SIM_ExtPublishedProperties`: this is fetched once per
// world, not once per frame.

// ---------------------------------------------------------------------------------------
// PER-FRAME EVENT STREAMS
// ---------------------------------------------------------------------------------------
//
// GameDataUpdate carries ten per-frame event lists as count+pointer pairs -- dig info, spawned
// ore, mass consumed, mass emitted, callbacks, unstable cells, and so on -- filled by the
// kernels, cleared at the top of every frame, and read by the game before the next one. The
// mechanism is Klei's and it works. What it cannot do is grow: GameDataUpdate's 496 bytes are
// a layout contract, so an eleventh event type has nowhere to go.
//
// A registered stream is the same shape with the hardcoding removed: a name, a record stride,
// and a published count+pointer pair. Adding a new native event type stops being an ABI change
// and becomes one Register call and one Emit call.
//
// WHO MAY DECLARE A STREAM: NATIVE CODE, AND ONLY NATIVE CODE. This is the one place the three
// registries deliberately do NOT match. A cell property is storage, so it makes sense for the
// mod that owns the data to declare it. An event stream is a thing the SIMULATION observed and
// is reporting, so it is declared by whatever produces it -- and everything that produces one
// is a kernel inside this DLL. There is no kRegisterEventStream message, on purpose: a mod
// pushing records into the sim so the sim can hand them back is a loopback with no consumer,
// and shipping the call would advertise a stream that stays empty forever.
//
// The surface a mod gets is DISCOVER, SUBSCRIBE, READ:
//
//     SIM_ExtEventStreamIndex("sim.message_refused")   -> stream index, or -1
//     kSubscribeEventStream { streamIdx, enable }      -> start/stop collecting
//     SIM_ExtPublishedEvents(frame, &count)            -> this tick's descriptor table
//
// SUBSCRIPTION IS THE COLLECTION GATE, not just a publish filter. An unsubscribed stream does
// not buffer a single record, so a stream nobody reads costs one predicted branch at each Emit
// site and no memory at all. That is stronger than stage 3's opt-in, where the data exists
// either way and only the copy is optional, and it is what makes it safe to put an Emit on a
// path that runs often.
//
// SAME LIFETIME RULE AS EVERY OTHER PUBLISHED POINTER:
//
//     THE POINTERS ARE VALID FOR THE TICK AND NOT ACROSS IT.
//
// Copy what you need before the next PrepareGameData. See kPublishCellProperty's block above
// for why the export takes the frame you are holding rather than "the current one".
//
// ONE DELIBERATE DIFFERENCE FROM KLEI'S TEN. Klei's vectors are cleared at the TOP of a frame.
// A registered stream's buffer is cleared at the END of the publish that carried it, once the
// copy has been taken. The reason is that this DLL can refuse a message on the game thread
// between frames (the immediate-message path), and a record produced there would be wiped by
// the next frame's opening clear before anything could publish it. Clearing after the copy
// means a record always reaches exactly one published frame, whichever side of the boundary
// it was produced on.
//
// A stream is NOT a checkpoint component and NOT a save section. Events describe one tick;
// there is nothing to restore.
constexpr int32_t kSubscribeEventStream = 0x4F4E4945;  // "ONIE"

#pragma pack(push, 4)
struct SubscribeEventStreamMessage {
  int32_t streamIdx;  // from SIM_ExtEventStreamIndex
  int32_t enable;     // non-zero subscribes, zero unsubscribes and drops the buffer
};
#pragma pack(pop)
static_assert(sizeof(SubscribeEventStreamMessage) == 8,
              "SubscribeEventStreamMessage layout drift");

// Queued like the rest, so a subscription sent during tick N first collects on tick N+1 and
// first appears in the table tick N+1 publishes.
//
// The table: one of these per SUBSCRIBED stream, in declaration order. An index into it is not
// a stream index -- match on `name`, exactly as with ExtPublishedProperty.
//
// `byteCount` is `count * stride`; build a span from it and derive nothing. `dropped` is how
// many records this stream refused THIS FRAME because it hit its per-frame byte cap: a
// non-zero value means the table is a prefix of what happened, and silently handing back a
// truncated list as if it were complete is the failure this field exists to prevent.
// DEFINED IN `abi/sim_ext_api.h`, not here, and this is an alias rather than a
// second copy: this type appears in an EXPORT'S SIGNATURE, and two definitions of
// a type a function takes cannot be checked against each other by any compiler.
// One definition, so there is nothing to drift. The field list and its comments
// are in that file; everything either side of this line is the reasoning, which
// stays here. See sim_ext_api.h's "HOW THE TWO HEADERS ARE KEPT IN STEP".
using ExtPublishedStream = ::OniExtPublishedStream;
static_assert(sizeof(void*) == 8, "ExtPublishedStream layout assumes a 64-bit build");
static_assert(sizeof(ExtPublishedStream) == 72, "ExtPublishedStream layout drift");

// int32_t SIM_ExtEventStreamIndex(const char* name);
//
// The declared stream's index, or -1. Stable for the life of the DLL instance: streams are
// declared once at SIM_Initialize and never added, removed or reordered, so a caller may
// resolve a name once instead of every tick.
//
// const ExtPublishedStream* SIM_ExtPublishedEvents(const GameDataUpdate* frame,
//                                                  int32_t* count);
//
// Same contract as SIM_ExtPublishedProperties: `frame` is the pointer PrepareGameData returned
// this tick, a null or stale frame is a refusal (null, *count = 0), and a live frame with no
// subscriptions is non-null with *count = 0.

// ---------------------------------------------------------------------------------------
// THE STREAM REGISTRY AS METADATA
// ---------------------------------------------------------------------------------------
//
// `SIM_ExtEventStreamIndex` above answers for a name the caller already knew. That is enough
// for a mod, which subscribes to the one stream it came for, and it is not enough for a
// GENERIC consumer -- a visualizer, an inspector, a route -- which has no name to ask with and
// whose whole job is to show what this DLL happens to declare.
//
// The published table is not the answer either, and this is the difference from stage 3b worth
// stating: `ExtPublishedStream` lists SUBSCRIBED streams only. A stream nobody has subscribed
// to is absent from it, which makes "this DLL declares no such stream" and "nobody is watching
// that stream yet" arrive identically -- and the second is the normal state of every stream
// before a client asks for it. So the published table can never be the discovery mechanism for
// the thing a client discovers in order to subscribe.
//
// COUNT DOES NOT TAKE THE WORKER BARRIER AND DESCRIBE DOES, which is a split stage 3b did not
// need, so both halves are worth stating.
//
// Count reads the size of a vector that is written once inside `SIM_Initialize`, before any
// thread exists, and never added to, removed from or reordered afterwards -- that promise is
// the one `SIM_ExtEventStreamIndex` already makes when it calls its index stable for the life
// of the DLL instance. Barriering an immutable container is a stall bought for nothing.
//
// Describe reads one field that is NOT immutable. `subscribed` is written by the
// `kSubscribeEventStream` handler, which runs inside the frame's message drain on the sim
// thread, so a barrier-free read races it -- and the failure is not a torn value (it is an
// aligned bool) but a STALE one: the drain for the tick the caller just ran has not
// necessarily happened yet, so the flag reports the previous frame's answer and the queued
// subscription appears to take two ticks instead of one. That was measured, not reasoned:
// `vftest` asserted the one-tick latency the ABI documents, and got two.
//
// So Describe waits, and it can afford to for the same reason stage 3b's could: this is
// fetched once per world, not once per frame. The per-frame path is `SIM_ExtPublishedEvents`,
// which stays barrier-free.
//
// `subscribed` is then the one field that can differ between two calls, for the same reason
// `published` can in stage 3b: `kSubscribeEventStream` is queued, so a caller that subscribes
// and immediately re-describes -- without a tick in between -- still sees 0. One frame of
// latency, not a failure.
// DEFINED IN `abi/sim_ext_api.h`, not here, and this is an alias rather than a
// second copy: this type appears in an EXPORT'S SIGNATURE, and two definitions of
// a type a function takes cannot be checked against each other by any compiler.
// One definition, so there is nothing to drift. The field list and its comments
// are in that file; everything either side of this line is the reasoning, which
// stays here. See sim_ext_api.h's "HOW THE TWO HEADERS ARE KEPT IN STEP".
using ExtEventStreamDesc = ::OniExtEventStreamDesc;
static_assert(sizeof(ExtEventStreamDesc) == 60, "ExtEventStreamDesc layout drift");

// int32_t SIM_ExtEventStreamCount(void);
// int32_t SIM_ExtEventStreamDescribe(int32_t streamIdx, ExtEventStreamDesc* out);
//
// Count returns how many streams this DLL declares. Describe fills `out` for one index and
// returns 1, or returns 0 and leaves `out` untouched for an out-of-range index, a null
// pointer, or no sim.
//
// `declaredIdx` is redundant with the index the caller passed in, on purpose: a descriptor
// that has been copied out of the table, put on a wire, or sorted by name is no longer beside
// the loop counter that produced it, and a client that has to re-derive the subscribe index by
// re-resolving the name has a second chance to get it wrong.
//
// A record's LAYOUT is not here and cannot be: `stride` says how big a record is, never what
// is in it. A stream is a contract between its producer and a consumer that knows the struct
// -- `ExtRefusedMessage` below is that struct for the first stream. A generic client can
// enumerate streams, subscribe, count records and hand back bytes without decoding any of it;
// decoding requires knowing which stream, and that is a name test, not a metadata field.

// ---------------------------------------------------------------------------------------
// THE FIRST DECLARED STREAM: "sim.message_refused", stride 16
// ---------------------------------------------------------------------------------------
//
// This DLL drops a malformed message in silence. There are 56 `if (!Payload(...)) return;`
// sites in simdll.cpp and an unknown-id path at either end of the message boundary, and every
// one of them returns without telling anybody -- so a mod cannot distinguish a write that
// landed from a write that was thrown away, and the symptom is a value that never changes.
//
// That is the hole this stream fills, and it is why it is the first one: it needs no new
// physics, it costs nothing while nobody is subscribed, and it makes the extension ABI
// answerable about its own failures.
//
// WHAT IS AND IS NOT REPORTED, stated exactly, because a diagnostic that is quietly partial is
// worse than none:
//
//   REPORTED  kExtRefusalShortPayload    -- the payload was smaller than the handler's struct.
//                                          One edit in the `Payload` template covers all 56.
//   REPORTED  kExtRefusalUnknownMessage  -- a message id this DLL does not handle, on either
//                                          the queued or the immediate path.
//   REPORTED  kExtRefusalBadTarget       -- wired at three messages ONLY. kSetCellProperty:
//                                          an unregistered property index, an out-of-range
//                                          cell, or a component outside the property's arity.
//                                          kAddCellPropertyAmount: the same, plus a property
//                                          that is not F32, a non-finite amount, a sum below
//                                          zero, or -- for a liquid-carried property -- a cell
//                                          that holds no liquid. kSetCellPropertyTransport: a property
//                                          that is not a registered F32.
//
//   NOT REPORTED: every other handler's own semantic refusal. Those are 50-odd separate early
//   returns with no shared chokepoint, and pretending otherwise would let a caller read "no
//   refusal record" as "the message was applied". Wiring one is a single Emit at the site.
enum ExtRefusalReason : int32_t {
  kExtRefusalShortPayload = 1,
  kExtRefusalUnknownMessage = 2,
  kExtRefusalBadTarget = 3,
  // `kSetTunable`'s five, each its own reason because a refusal that does not say which rule it
  // broke leaves the caller guessing.
  kExtRefusalNotFinite = 4,
  kExtRefusalOutOfRange = 5,
  kExtRefusalWorldLoaded = 6,
  kExtRefusalReservedBits = 7,
  kExtRefusalCrossField = 8,
};

#pragma pack(push, 4)
struct ExtRefusedMessage {
  int32_t messageId;      // the SimMessageHash / ext:: id as sent
  int32_t reason;         // ExtRefusalReason
  int32_t payloadBytes;   // bytes that arrived
  int32_t expectedBytes;  // bytes the handler needed; 0 when the reason is not about size
};
#pragma pack(pop)
static_assert(sizeof(ExtRefusedMessage) == 16, "ExtRefusedMessage layout drift");

// The name, spelled once. A caller resolving it with SIM_ExtEventStreamIndex should not be
// retyping a string literal that this header already owns.
constexpr char kStreamMessageRefused[] = "sim.message_refused";

// The declaration cap, and the per-frame byte cap a stream buffers before it starts counting
// drops instead of growing. Both are ABI values because `dropped` is only interpretable if the
// caller knows there is a ceiling at all.
constexpr int32_t kExtMaxEventStreams = 32;
constexpr int32_t kExtStreamBytesPerFrame = 1 << 20;

// ---------------------------------------------------------------------------------------
// The PER-ELEMENT attribute registry -- the third and last of the three extension
// registries, and the smallest.
//
// WHAT IT REPLACES. `kSetMolecularMass` above is a per-element side table with the key
// hardcoded to one attribute: a `std::vector<std::pair<int32_t, float>>` in
// `ElementTable`, one message id, one accessor, and no way for a mod to read the value back
// or to store an attribute of its own. Stage 4 removed a hardcoded list of ten event types;
// this removes a hardcoded list of one. `sim.molecular_mass` is now an ordinary registered
// attribute and `ElementTable::MolecularMassOf` an ordinary reader of it.
//
// THE LINE BETWEEN THIS AND A TYPED ACCESSOR (and it is not stylistic): a
// native kernel must never do a string lookup per cell per substep. So the STORAGE moves here
// and the ACCESSOR stays typed -- `MolecularMassOf` resolves the attribute index once and
// indexes it, exactly as it walked its own vector before. Promoting a third-party attribute to
// a typed one later is then a deliberate, reviewable change rather than an accident.
//
// FOUR WAYS THIS REGISTRY IS DELIBERATELY NOT SHAPED LIKE REGISTRY 1, each because per-element
// content data is not per-cell world state:
//
//   1. NO `persist` FIELD, AND NOTHING HERE IS EVER SAVED. Klei's element table is content
//      data reloaded from `StreamingAssets` every session and is not in the save blob; an
//      attribute of an element is the same kind of thing. Writing one into a save would let a
//      stale value from an old version of a mod outrank that mod's current table on load,
//      which is worse than not carrying it. The registering mod re-pushes after every load --
//      the `kRehydrated` contract, made unconditional because there is no alternative to
//      offer. Said here rather than left implicit, because an unstated persistence answer is
//      how per-cell state has been lost before.
//   2. NO `defaultBits`, AND "UNSET" IS A REAL ANSWER. Per-cell storage is dense, so every
//      cell has a value and a default is the only way to say what it is. Per-element storage
//      is SPARSE -- most elements have no opinion about most attributes -- and the fallback
//      for an unset attribute is per-element knowledge this registry does not have.
//      `sim.molecular_mass` falls back to Klei's own `Element::molarMass`, which is a
//      different number for every element. So a read reports set-or-not and the caller
//      decides; it never invents a value.
//   3. REGISTRATION NEVER CLOSES. Registry 1 closes at the first `World::Allocate` because a
//      late registration would leave one per-cell array shorter than the rest. Nothing here is
//      sized to the world, so a mod that loads late can still register, and a world reload
//      does not disturb what is already registered.
//   4. KEYED BY SimHashes id, NEVER BY ElementTable INDEX. An index is only meaningful against
//      one particular loaded table; a hash survives the table being reloaded. This is the same
//      choice `kSetMolecularMass` already documented -- and stage 5 is what makes the claim
//      true, because `ElementTable::Load` used to CLEAR every override it held. Keying was
//      never the bug; the clear was.
//
// BOTH MESSAGES ARE IMMEDIATE, not deferred, and `kRegisterElementAttribute` returns the
// attribute index the way `kRegisterCellProperty` does. `SIM_HandleMessage` already calls
// `WaitIdle()` before the immediate path, so an immediate write is synchronized against the
// worker at no extra cost -- and content data pushed at load should be readable on the same
// tick rather than one tick later. A deferred write would force every caller to push its
// values as early as possible and still lose a tick.
//
// BYTE-IDENTICAL UNTIL SENT, like everything else in this file, with the same footnote
// `kSetMolecularMass` carries: the four diatomic corrections are seeded natively, so "never
// sent" means "keeps the seeded corrections", not "keeps Klei's atomic figures".
constexpr int32_t kRegisterElementAttribute = 0x4F4E4946;  // "ONIF"

#pragma pack(push, 4)
struct RegisterElementAttributeMessage {
  char name[48];   // `<owner>.<attribute>`, same validator as a cell property's name
  int32_t type;    // ExtScalarType
  int32_t arity;   // components per element, 1..kExtMaxArity
};
#pragma pack(pop)
static_assert(sizeof(RegisterElementAttributeMessage) == 56,
              "RegisterElementAttributeMessage layout drift");

// Writes one component of one element's attribute, or clears the whole entry for that element.
//
// `valueBits` is reinterpreted per the attribute's registered type, exactly as
// `SetCellPropertyMessage::valueBits` is: bit-cast for kExtF32, truncated to the low byte /
// low two bytes for kExtU8 / kExtU16 / kExtElementIdx, taken verbatim for kExtI32. An
// attribute is never saved, so an element-index attribute needs no remap: the owner pushes it
// against the table that is loaded.
//
// CLEARING IS AN EXPLICIT FLAG, not a magic value. `kSetMolecularMass` spells "remove this
// override" as `gPerMol <= 0`, which works only because no real molar mass is negative; there
// is no such spare value for a general attribute, and inventing one per type is how a caller
// ends up unable to store a legitimate zero. When `clear` is non-zero, `component` and
// `valueBits` are ignored and the element's whole entry for this attribute goes away --
// back to unset, which is a different answer from zero.
constexpr int32_t kSetElementAttribute = 0x4F4E4947;  // "ONIG"

#pragma pack(push, 4)
struct SetElementAttributeMessage {
  int32_t attrIdx;      // from kRegisterElementAttribute / SIM_ExtElementAttributeIndex
  int32_t idHash;       // SimHashes value, the number `Element::id` carries
  int32_t component;    // 0 .. arity-1
  uint32_t valueBits;   // raw bits, reinterpreted per the registered type
  int32_t clear;        // non-zero removes this element's entry; component/valueBits ignored
};
#pragma pack(pop)
static_assert(sizeof(SetElementAttributeMessage) == 20,
              "SetElementAttributeMessage layout drift");

// The first first-party attribute, spelled once so a caller resolving it with
// SIM_ExtElementAttributeIndex is not retyping a literal this header already owns. f32,
// arity 1, g/mol. `kSetMolecularMass` still works and is now a convenience alias that writes
// this attribute -- kept because the framework already uses it and because a caller with one
// number to push should not have to register anything.
constexpr char kAttrMolecularMass[] = "sim.molecular_mass";
// Second and third, for native phase change in conduit runs. The curve holds the inputs of a
// Stationeers-style clamped evaporation temperature, pre-resolved by
// OniFramework's MaterialPropertyRegistry so the sim evaluates one power and one clamp and
// never re-derives a critical or freezing point. See ONI_ATTR_PHASE_CURVE for the layout.
constexpr char kAttrPhaseCurve[] = "sim.phase_curve";
constexpr int32_t kPhaseCurveArity = 5;
constexpr char kAttrLiquidDensity[] = "sim.liquid_density";
// Fourth and fifth, for the cell condensation rule. Together with `kAttrPhaseCurve` they are
// the rule's three inputs, tested in this order: can it condense, is the pressure above the
// minimum liquid pressure, is it below the saturation temperature. An element carrying all three is governed by that rule instead of
// Klei's fixed `lowTemp - kTransitionMargin`; an element missing any of them keeps Klei's,
// which is why every offline scenario stays byte-identical.
//
// They are SEPARATE attributes rather than two more slots on `kAttrPhaseCurve` because
// `kPhaseCurveArity` is a wire constant: widening it would be a MAJOR ABI break for a purely
// additive capability, and a curve written by an older framework would silently read short.
constexpr char kAttrMinLiquidPressure[] = "sim.min_liquid_pressure";
constexpr char kAttrCanCondense[] = "sim.can_condense";
// Sixth, for physical latent heats: the enthalpy of FUSION, J/kg, so the planetary latent accumulator can count a
// freeze and a melt as well as a condensation and a boil. `kAttrPhaseCurve`'s slot 4 is an
// enthalpy of VAPORIZATION and stays one. A separate attribute rather than a sixth curve slot for
// the reason given above: `kPhaseCurveArity` is a wire constant.
constexpr char kAttrLatentFusion[] = "sim.latent_fusion";

// The declaration cap, for the same reason `kExtMaxEventStreams` is one: the index is a wire
// value, `Find` walks the descriptors linearly, and a mod registering in a loop should hit a
// wall with a name in the log rather than an allocation failure.
constexpr int32_t kExtMaxElementAttributes = 64;

// ---------------------------------------------------------------------------------------
// Added for Mod 1's pipe radiators: CONVECTION BETWEEN A BUILDING BODY AND ITS
// CELLS, Stationeers' shape. Stationeers' radiator convection:
//
//     heat = 100 * (T_internal - T_world) * surfaceArea
//            * world.HeatExchangeRatio() * internal.HeatExchangeRatio()
//            * TickSpeedSeconds * convectionFactor                      // J per tick
//
// Here the internal side is the building's own body, a solid, so its ratio is 1; the world
// side is each footprint cell's `HeatExchangeRatio()` -- max(clamp01(P / 1 atm),
// clamp01(liquid volume ratio / 0.001)) -- with the area split evenly over the footprint. A
// solid cell is 0, as Stationeers' `!CanContainAtmos` is: ONI's own conduction sweep owns a
// building's exchange with the tiles it touches, and this term is not a second copy of it.
// The transfer is limited to the pair's equilibrium, which is what Stationeers'
// `GasMixture.TransferEnergyTo` does, so it moves heat and never creates or deletes any.
//
// It runs ALONGSIDE Klei's conduction, not instead of it, and once per substep like the other
// framework building terms (see StepBuildingHeatExchange). 0 by default, so a world that never
// sends this message computes byte-identically.
//
// ONE SIDE EFFECT ON THE CONDUIT KERNEL, and it is the reason this is a message about the
// building and not a new conduit policy. Under `ONI_CONDUIT_POLICY_CONVECTION` a pipe's
// contents <-> pipe exchange is scaled by the run's ratio AND the conduit cell's, which stands
// in for Stationeers' pipe <-> world convection. For a pipe whose building has a convection
// factor that cell ratio belongs to THIS term instead, and applying it on both legs would count
// it twice and, in vacuum, stop the radiator ever warming up -- so the radiator's radiation
// (kSetBuildingRadiation) would never see its contents' heat. The conduit kernel runs on the
// game thread, so the send itself records the building's handle there (simdll.cpp,
// QueueDeferredMessage); the building field is applied by the worker like any queued message.
//
// `surfaceAreaM2` <= 0 means the building's own CellCount() square metres, as for radiation.
// `convectionFactor` <= 0 switches the term off and removes the conduit-side record.
//
// REACH, AND WHY IT IS NOT ALWAYS THE FOOTPRINT. Stationeers' radiators convect with the room's
// atmosphere; ONI's cells are the room, and a building that pushes all of its heat into the one
// cell it stands in saturates that cell within a tick or two -- after which what limits the
// exchange is how fast ONI's own cell-to-cell conduction carries heat away, not the building at
// all. Measured on the RADIATOR rig: a radiator at Stationeers' own factor shed
// 63.6 kW against a plain Radiant Pipe's 61.8 kW, a 3% difference, because both had pinned their
// own cell. `reachCells` spreads the SAME total conductance over every open cell within n of the
// building, which is what makes a radiator a room device and leaves an ordinary pipe a tile
// device. 0 is the footprint alone and is what every building that does not ask gets.
constexpr int32_t kSetBuildingConvection = 0x4F4E4948;  // "ONIH"

#pragma pack(push, 4)
struct SetBuildingConvectionMessage {
  int32_t handle;          // the building's sim handle (StructureTemperaturePayload.simHandleCopy)
  float convectionFactor;  // Stationeers-style convection factor; <= 0 switches the term off
  float surfaceAreaM2;     // <= 0 means the building's own CellCount()
  int32_t reachCells;      // 0 = its own footprint; n = every open cell within n of it
};
#pragma pack(pop)
static_assert(sizeof(SetBuildingConvectionMessage) == 16,
              "SetBuildingConvectionMessage layout drift");

// The largest `reachCells` the sim will honour. A reach of 4 is 41 cells around a 1x1 building,
// walked every substep for every building that convects; beyond that the cost stops being
// trivial and a caller wanting a whole room wants a different mechanism than a box.
inline constexpr int32_t kMaxBuildingConvectionReach = 4;

// Stationeers' convection coefficient, `AtmosphereHelper.GetConvectionHeat`'s literal 100, in
// W/(m^2 K). In the sim's kilowatts that is 0.1 kW/(m^2 K).
constexpr float kBuildingConvectionWPerM2K = 100.0f;

// ---------------------------------------------------------------------------------------
// Twenty-fifth entry, added for Flagship 2 / Mod 3 (Dynamic Planet): the world a
// colony is standing on, as a thing the simulation knows about.
//
// WHAT THIS REPLACES. `kSetEnvironmentTemperature` (the eleventh entry, above) is ONE scalar for
// the whole grid. A cluster map is several asteroids sharing that grid, and the point of Mod 3 is
// that they are not the same place: one world's night is another's noon, and a radiator on a cold
// world is a different machine from the same radiator on a hot one. This message is that scalar
// made per world, plus the two inputs a planet needs that no vanilla message carries at all.
//
// The global scalar is NOT retired. It stays as the fallback for any world with no record of its
// own, which is what keeps every existing caller and every existing test unchanged: a world that
// receives no `kSetWorldEnvironment` behaves exactly as it did before this message existed.
//
// `worldIndex` INDEXES `DefineWorldOffsets`' LIST, which the game sends in
// `ClusterManager.Instance.WorldContainers` order --
// NOT the per-cell zone byte from `SetWorldZones`, which vanilla allocates and never reads, and
// not the active-region index, which skips undiscovered worlds (`Game.UnsafeSim200ms` builds one
// region per DISCOVERED `WorldContainer`, so the two lists diverge the moment a world is hidden).
// A negative `worldIndex` sets the record every world falls back to, which is how a
// single-asteroid game or an offline scenario configures itself without knowing any indices.
//
// THREE CAPABILITIES, ONE RECORD, EACH INERT UNTIL ITS OWN FIELD IS NON-ZERO:
//
//   1. `sinkKelvin` -- the radiative sink for buildings on this world, replacing the global
//      scalar for them. <= 0 falls back to the global.
//
//   2. SOLAR HEATING. `peakIrradianceWPerM2` is the flux at full sun and `fullSunLux` is the lux
//      that same world reports at full sun, so the sim forms the instantaneous fraction itself
//      from `NewGameFrame.currentSunlightIntensity` -- which it has been handed every frame since
//      forever and has never read (`sim/textures.h` records the measurement that Klei's sunlight
//      texture does not read it either). That costs the managed side nothing per tick, and it
//      tracks an eclipse for free, because `TimeOfDay.UpdateSunlightIntensity` already zeroes the
//      lux during one. `solarAbsorptivity` is how much of the incident flux a fully exposed cell
//      takes; 0 switches solar heating off.
//
//      A BUILDING'S SOLAR ABSORPTION NEEDS NO FIELD OF ITS OWN. Kirchhoff's law says a grey
//      body's absorptivity equals its emissivity at the same wavelength, so a building absorbs
//      sunlight with the `radiationFactor` it already radiates with (`kSetBuildingRadiation`).
//      Stationeers carries a separate per-prefab `SolarHeatingScale` -- 0.1 on an ordinary thing,
//      2 on a Medium Radiator, 50 on the Large Extendable -- which is a tuning knob, not a
//      physical quantity, and adding one here would let a building be a perfect absorber and a
//      perfect insulator at once. If a demonstration later needs that knob it is a field on the
//      radiation message, not a second concept.
//
//   3. THE SURFACE ATMOSPHERE BOUNDARY. A cell with an unobstructed path to the sky is driven
//      toward `surfacePressureKPa` of `boundaryElement` at `boundaryTemperatureK`, closing
//      `boundaryRate` of the gap per substep. This is what makes a planet's surface thin CO2
//      rather than vacuum. Exposure is the sunlight texture the sim already computes per world
//      per frame (`ComputeSunlight`): a byte of 255 is a clear column to space, and it is
//      already the exact cell set a planetary atmosphere touches, so the boundary needs no scan
//      of its own. `boundaryRate` <= 0, `surfacePressureKPa` <= 0 or `boundaryElement` < 0 all
//      switch the boundary off.
//
//      IT IS AN INFINITE SOURCE, BY DESIGN, and this is where the honest
//      accounting has to be explicit. Matter appearing at the boundary is charged to the mass
//      ledger as a named boundary crossing, exactly as radiated energy is charged to
//      `radiated_to_environment` -- it is a planet handing gas to a colony, not matter created
//      from nothing, but it IS matter entering the grid and the ledger says so. A Stationeers
//      planet does not debit the gas it hands out either.
//
// THE FINITE RESERVOIR IS DELIBERATELY NOT HERE. Heat that is never deleted, carried to
// planetary scale, wants a sink that warms up. Stationeers does not warm its planets either,
// presumably because a planet a colony can cook is a balancing problem rather than a physics
// one. The infinite sink is the default and a finite one is a policy built on top. The sim's part of that is that `sinkKelvin` is a per-world value a
// caller can move every tick -- which is all a managed reservoir needs in order to exist.
constexpr int32_t kSetWorldEnvironment = 0x4F4E4949;  // "ONII"

#pragma pack(push, 4)
struct SetWorldEnvironmentMessage {
  int32_t worldIndex;            // index into DefineWorldOffsets' list; < 0 = the fallback record
  float sinkKelvin;              // radiative sink for this world; <= 0 = the global scalar
  float peakIrradianceWPerM2;    // solar flux at full sun; <= 0 = no solar heating
  float fullSunLux;              // the lux this world reports at full sun; <= 0 = no solar heating
  float solarAbsorptivity;       // 0..1 of the incident flux a fully exposed cell takes; 0 = off
  float surfacePressureKPa;      // the boundary's target pressure; <= 0 = no boundary
  float boundaryTemperatureK;    // the boundary's target temperature; <= 0 = sinkKelvin
  float boundaryRate;            // 0..1 of the gap a fully exposed cell closes per substep
  int32_t boundaryElement;       // the element the boundary supplies; < 0 = no boundary
};
#pragma pack(pop)
static_assert(sizeof(SetWorldEnvironmentMessage) == 36,
              "SetWorldEnvironmentMessage layout drift");

// The most of the gap a boundary cell may close in one substep. A rate of 1 would make the
// boundary cell a constant rather than a boundary -- it would overwrite whatever a pump or a
// vent had just done to it within the same substep, and the surface would stop being somewhere
// the colony can act on. Clamped rather than rejected, because a caller asking for "instantly"
// means "as fast as you allow".
inline constexpr float kMaxWorldBoundaryRate = 0.5f;

// ---------------------------------------------------------------------------------------
// kSetWorldSun: THE DIRECTION A WORLD'S SUN SHINES FROM. Flagship 2 / Mod 3.
//
// `kSetWorldEnvironment` already carries how MUCH sun a world gets: the DLL forms the fraction
// from the lux `NewGameFrame` sends, and on a planet with a sun path that lux is
// `sin(elevation)` times the world's sunlight trait. What it cannot carry is where the light
// comes FROM, and on a grid of cliffs and overhangs that decides which cells are lit at all.
// This is that direction, and nothing else.
//
// `dirX`, `dirY`: the vector from a cell toward the sun, projected into the grid plane. +x is
// east -- the grid's x-max side -- and +y is up. Normalised on
// receipt, so only the direction matters; `dirY <= 0` is a sun at or below the horizon and
// lights nothing. The out-of-plane component is not sent: nothing in a two-dimensional grid can
// cast a shadow along it, and how much it dims a flat surface is already inside the lux.
//
// WHAT IT SWITCHES ON. Every frame, a world with a sun gets a second exposure field beside
// Klei's sunlight texture: the same absorption law, walked along this direction
// (`ComputeSunBeam`, sim/textures.h). The two planetary SOLAR terms -- cells in
// `StepWorldEnvironment`, buildings in `StepBuildingHeatExchange` -- read that beam instead of
// the vertical texture. Everything that means "open to the sky" keeps the vertical texture: the
// atmosphere boundary, a cell's radiation to the sky, the latent accumulator, and the texture
// the game itself is handed. So a world that is never sent this message is unchanged byte for
// byte, and one that is gets shade without losing its sky.
//
// Stationeers' equivalent is a raycast per panel along `OrbitalSimulation.WorldSunVector`
// (`SolarPanelArm.IsRaycastObscured`, plus `VoxelTerrain.OctreeRaycast`). A field rather than a
// raycast because ONI's consumers are cells, not a handful of panels.
//
// Read back with `SIM_ExtWorldSun`; the beam itself with `SIM_ExtCopySunBeam`. `enabled == 0`
// forgets the world's sun. Same index rules as `kSetWorldEnvironment`: `DefineWorldOffsets`'
// order, negative for the fallback. The record survives a world re-boot exactly as the
// environment record does. QUEUED.
constexpr int32_t kSetWorldSun = 0x4F4E494A;  // "ONIJ"

#pragma pack(push, 4)
struct SetWorldSunMessage {
  int32_t worldIndex;  // index into DefineWorldOffsets' list; < 0 = the fallback record
  float dirX;          // toward the sun, in the grid plane; + = east (x max)
  float dirY;          // toward the sun, in the grid plane; + = up; <= 0 = below the horizon
  int32_t enabled;     // 0 forgets the record
};
#pragma pack(pop)
static_assert(sizeof(SetWorldSunMessage) == 16, "SetWorldSunMessage layout drift");

// ---------------------------------------------------------------------------------------
// kSetCellRadiation -- the EIGHTH checkpoint component: the per-cell radiation field, whole.
//
// The save blob does carry radiation, but `Load` does not give all of it back. Klei's `Load`
// clears the radiation of every Vacuum and Void cell (`World::ReadLoadedCell`, measured against
// Klei's DLL by driver/src/worldgen_test), and a running world has radiation in Vacuum cells.
// That is faithful and deliberately unchanged: a colony reloaded in the main game loses it
// exactly as vanilla does. This message exists so a caller RECONSTRUCTING a run can opt out,
// the same bargain kSetDiseaseGrowth offers for the growth remainders.
//
// Measured. With `Load`'s rewrite of Vacuum/Void cells in place and no radiation state
// carried, a 30-seek timeline test on a recorded world has 2 of 4 exact stops and 17 of 26
// replayed ticks failing, in the radiation array alone, with the
// random stream in step and the mass bit-identical. Removing only the Vacuum/Void radiation
// clear brought all 30 back. The whole field travels rather than just the cells `Load`
// cleared, so this does not depend on which of `Load`'s rewrites a running world can reach.
//
// Payload is { int32 count }, then `count` floats, one per PADDED cell, cell-major.
//
// ONE OF THE CHECKPOINT COMPONENTS; the complete list is at kSetExtCellState.
//
// Pairs with the SIM_DebugCellRadiation export, which is the getter and reports the length.
//
// IMMEDIATE, the first checkpoint component that was (kSetVisibilityState is the second, for a
// reason of the same shape), and measured into it: queued, it restored
// every replayed tick and still failed both exact stops. The first `PrepareGameData` after a
// `Load` publishes its priming frame before a queued message drains, and radiation is the one
// component the game reads back in `GameDataUpdate` -- the others are unpublished, so a
// one-frame-late restore of them is invisible at an exact stop. Applied on the calling thread
// after `WaitIdle`, straight into `World::radiation_`, so send it after the Load it belongs to.
// A length that is not that world's padded cell count is rejected and logged rather than
// partially applied.
constexpr int32_t kSetCellRadiation = 0x4F4E494B;  // "ONIK"

#pragma pack(push, 4)
struct SetCellRadiationMessage {
  int32_t count;  // padded cells that follow this header
  // float radiation[count];
};
#pragma pack(pop)
static_assert(sizeof(SetCellRadiationMessage) == 4, "SetCellRadiationMessage layout drift");

// kSetBlockedGasAddPolicy -- what a gas add does when it has nowhere to go.
//
// Vanilla's `AddGas` (sim/cellmod.h) reaches `AddIntoBlockedCell` when the named
// cell holds a DIFFERENT gas that `DisplaceGas` cannot move and none of the left, right or up
// neighbours is vacuum or already this gas. That tail deletes whichever of the two masses is
// smaller: the incoming gas, or the pocket standing in its way. Measured live on the bubbles
// rig: a 200 g CO2 pop onto a 20 g methane pocket lost 40 g, all of the methane and 20 g of
// the CO2.
//
// `kBlockedGasAddPromoteAndMix` stops that deletion for the one message path that
// reaches it, a `ModifyCell` gas add (`SimMessages.ModifyMass`, bubble pops, dupe breath). The
// sim promotes the blocked cell's room (`kPromoteRoom`'s meaning), moves every gas cell of
// that room out of vanilla's grid and into the room's mixture, then adds the incoming gas to
// the blocked cell's mixture. The whole room, and not just the one cell, because a promoted
// room's vanilla gas is dead to everything that reads a promoted room (breathing, the
// overlays, the mixing pass): promoting it without moving the gas across would leave the
// room reading as vacuum.
//
// What it does NOT cover: a blocked LIQUID add (`AddLiquid` shares the same deleting tail and
// still deletes -- promotion is gas-only), and a gas add with no room to promote into (a cell
// the room graph does not count as open, which keeps vanilla's behaviour and so deletes).
//
// Default `kBlockedGasAddVanilla`, so the fork stays bit-exact against Klei until this is
// sent. Kept on the Sim, not the World: it survives an Allocate or a Load, and is not saved.
// A mod that wants it sends it once per game session, after the sim is initialised.
constexpr int32_t kSetBlockedGasAddPolicy = 0x4F4E494C;  // "ONIL"

enum BlockedGasAddPolicy : int32_t {
  kBlockedGasAddVanilla = 0,          // AddIntoBlockedCell: the smaller mass is deleted
  kBlockedGasAddPromoteAndMix = 1,    // promote the room, move its gas into the mixture, mix
};

#pragma pack(push, 4)
struct SetBlockedGasAddPolicyMessage {
  int32_t policy;  // a BlockedGasAddPolicy; any other value is refused and changes nothing
};
#pragma pack(pop)
static_assert(sizeof(SetBlockedGasAddPolicyMessage) == 4,
              "SetBlockedGasAddPolicyMessage layout drift");

// kSetCellPropertyTransport -- a cell property that is carried by liquid (Layer C).
//
// A registered F32 property flagged `kTransportFollowsLiquidMass` is an EXTENSIVE amount held by
// the cell's liquid: dissolved gas, a salt, a dye, a tracer. Every kernel that moves liquid mass
// moves the same share of every component of the property with it (the liquid sweep, a swap, a
// displacement, a merge). Nothing is created or destroyed by the ride:
//
//   * liquid taken by a CONSUMER (`ElementConsumer`, the `MassConsumption` message) hands its
//     share to the "sim.liquid_payload_consumed" stream, keyed by the consumer, so the managed
//     side can carry it on into the pipe or storage;
//   * liquid that stops existing in the grid (cleared, evaporated, changed phase, fell out as a
//     particle, replaced, removed or deleted by a message) hands its share to the
//     "sim.liquid_payload_released" stream, with the reason.
//
// A record the owner is not subscribed to is not dropped silently: its amount is added to the
// property's unreported total (`SIM_ExtLiquidPayloadUnreported`).
//
// `kTransportStatic` is every property's default and the pre-Layer C behaviour: the value stays
// on its cell whatever the cell's contents do. Only F32 properties can follow liquid; any other
// type is refused. First-party `sim.dissolved_mass` is flagged by the sim itself.
constexpr int32_t kSetCellPropertyTransport = 0x4F4E494D;  // "ONIM"

enum CellPropertyTransport : int32_t {
  kTransportStatic = 0,
  kTransportFollowsLiquidMass = 1,
};

#pragma pack(push, 4)
struct SetCellPropertyTransportMessage {
  int32_t propertyIdx;  // from kRegisterCellProperty / SIM_ExtCellPropertyIndex
  int32_t transport;    // a CellPropertyTransport
};
#pragma pack(pop)
static_assert(sizeof(SetCellPropertyTransportMessage) == 8,
              "SetCellPropertyTransportMessage layout drift");

// Why a liquid payload left the grid. See kSetCellPropertyTransport.
enum LiquidPayloadReleaseReason : int32_t {
  kPayloadReleasedCleared = 0,      // ClearCell: evaporation of a tiny cell, a displaced merge
  kPayloadReleasedPhaseChange = 1,  // the liquid froze or boiled (TransitionCell)
  kPayloadReleasedOffGas = 2,       // the liquid off-gassed part of itself (DoLiquidOffGas)
  kPayloadReleasedFalling = 3,      // the liquid left the grid as a falling particle
  kPayloadReleasedWisp = 4,         // a sub-gram wisp evaporated or merged away
  kPayloadReleasedReplaced = 5,     // a message replaced the liquid with another element
  kPayloadReleasedRemoved = 6,      // a message removed liquid mass (negative AddRemoveSubstance)
  kPayloadReleasedDeleted = 7,      // vanilla deleted the liquid (AddIntoBlockedCell)
  kPayloadReleasedMassless = 8,     // ZeroMasslessCells emptied the cell
  // The liquid did NOT leave: the gas did. Only `sim.dissolved_mass` is released for this
  // reason, by `StepEffervescence`, when a cell holds more dissolved gas than the pressure on it
  // can keep in solution (ext::kSetEffervescence). The cell named is still liquid, which is
  // what tells the managed side to put the gas back as BUBBLES rather than as a gas cell.
  kPayloadReleasedEffervescence = 9,
  // The liquid did NOT leave, and no bubble forms: the gas crossed the liquid's free SURFACE into
  // the gas above it, because the surface cell held more than the gas above's PARTIAL pressure of
  // that gas keeps in solution (`kSetEffervescence`'s surface exchange). The cell
  // named is the top liquid cell; the managed side puts the gas into the cell above it.
  kPayloadReleasedSurface = 10,
  // The reverse of kPayloadReleasedSurface, and NOT a release: the liquid's free surface
  // DISSOLVED gas out of the cell above it. The record's `amount` is NEGATIVE --
  // minus the kg that went into the lane -- so the stream's sum per lane stays "net out of the
  // liquid", and a reader that only acts on positive amounts (as older readers do) acts on
  // none of these. `temperatureK` is the gas cell's. The sim has already taken the gas out of
  // the grid; the managed side must not put anything anywhere. A record nobody hears lands on
  // the property's unreported total like any other, so that total is net too.
  kPayloadAbsorbedSurface = 11,
};

#pragma pack(push, 4)
struct LiquidPayloadReleased {
  int32_t cell;          // GAME cell the liquid was in, as every cell-addressed message
  int32_t propertyIdx;
  int32_t component;
  float amount;          // in the property's own unit (kg for sim.dissolved_mass)
  float temperatureK;    // the liquid's temperature at the moment it left
  int32_t reason;        // LiquidPayloadReleaseReason
};
#pragma pack(pop)
static_assert(sizeof(LiquidPayloadReleased) == 24, "LiquidPayloadReleased layout drift");

enum LiquidPayloadConsumerKind : int32_t {
  kPayloadConsumerElementConsumer = 0,  // `id` is the ElementConsumer's sim handle
  kPayloadConsumerMassConsumption = 1,  // `id` is the MassConsumption message's callbackIdx
};

#pragma pack(push, 4)
struct LiquidPayloadConsumed {
  int32_t kind;          // LiquidPayloadConsumerKind
  int32_t id;
  int32_t propertyIdx;
  int32_t component;
  float amount;
  float temperatureK;    // the consumed liquid's mixed temperature, as ConsumedMassInfo reports it
};
#pragma pack(pop)
static_assert(sizeof(LiquidPayloadConsumed) == 24, "LiquidPayloadConsumed layout drift");

// kAddCellPropertyAmount -- ADD to one component of an F32 cell property (Layer C).
//
// `kSetCellProperty` writes an absolute value, which is the wrong shape for an amount: a mod
// that dissolves 5 g into a cell has to read the lane, add, and write back, and the read comes
// from a published frame one tick behind the queued write -- so two adds in flight lose one.
// This adds on the sim's side of the queue instead. A result below zero is refused whole
// ("sim.message_refused", kExtRefusalBadTarget) rather than clamped: a clamp would quietly
// create the difference. F32 properties only. An add to a liquid-carried property
// (kTransportFollowsLiquidMass) in a cell that holds no liquid is refused the same way: the
// amount would have no liquid to ride and would sit there until some arrived.
constexpr int32_t kAddCellPropertyAmount = 0x4F4E494E;  // "ONIN"

#pragma pack(push, 4)
struct AddCellPropertyAmountMessage {
  int32_t cellIdx;      // game cell index, as kSetCellProperty
  int32_t propertyIdx;
  int32_t component;
  float amount;         // may be negative; the result may not be
};
#pragma pack(pop)
static_assert(sizeof(AddCellPropertyAmountMessage) == 16,
              "AddCellPropertyAmountMessage layout drift");

constexpr char kStreamLiquidPayloadReleased[] = "sim.liquid_payload_released";
constexpr char kStreamLiquidPayloadConsumed[] = "sim.liquid_payload_consumed";

// =======================================================================================
// THE GENERIC FIELD SOLVER. Phase: `kPhaseFields`.
//
// A FIELD IS A PROPERTY THAT HAS BEEN GIVEN A SOLVER. It is not new storage: it is an
// already-registered `kExtF32`, arity-1 per-cell property (kRegisterCellProperty) with a
// propagation rule attached to it by index. So a field inherits that registry's naming,
// duplicate refusal, persistence class, save blob, checkpoint, zero-copy publish
// (`SIM_ExtPublishedProperties`) and rehydration accounting, and this block adds exactly one
// thing: something that WRITES the field every substep.
//
// WHY THE RULE IS DATA. A sim which calls out to arbitrary code cannot be diffed
// against the shipped DLL, and `diffsim` byte-identity is what this project's physics is
// accepted on. A propagation rule is therefore a struct of numbers evaluated natively.
//
// THE TWO INVARIANTS A FIELD KERNEL HOLDS, both load-bearing rather than tidy:
//
//   * READ ANY CELL, WRITE ONLY INSIDE THE REGION RECT. Klei's own emitter already works
//     this way -- `tickConstant`'s rays read wherever the line goes and its writes are
//     clipped to the region. Ordering ACROSS regions is not guaranteed, precisely to keep
//     region-parallel stepping possible, and a
//     cross-region READ is compatible with that where a write is not. Diffusion is excluded
//     from this version for exactly this reason and not for effort.
//   * NEVER ADVANCE THE SHARED RANDOM STREAM. `tickConstant` draws from the world LCG for
//     its noise, and only for the outer three quarters of an emitter -- so the emitter's
//     GEOMETRY decides how many draws the world makes, and the gas shuffle downstream moves
//     when they do. A field doing that would mean any mod registering any field shifts
//     vanilla's random stream, turning a diffsim-green DLL divergent on load. There is no
//     noise term here. If one is ever wanted it gets a private LCG seeded from
//     (fieldIdx, cell, substep) and the shared stream stays untouched.
//
// VANILLA RADIATION AND THE SUNLIGHT TEXTURES ARE NOT PORTED ONTO THIS. They are the two
// existing attenuated-field walks this generalises, they are bit-exactness-gated against the
// shipped game, and in this version they stay exactly where they are -- which is what makes
// "a world with no registered field is byte-identical" the gate that says this phase is safe.
// They are instead the ORACLES: a field configured as radiation must reproduce
// `RadiationAbsorptionAlongLine`, and one configured as light must reproduce `ComputeSunBeam`.

// How one cell's attenuation is computed from the per-element attribute the field names.
// The three are the tree's own three, named -- not a menu invented here.
enum ExtFieldAttenuationLaw : int32_t {
  // The attribute alone, mass-free. Clamped to [0, 1].
  kAttenFlat = 0,
  // `RadiationAbsorption` (sim/radiation.h): scaled by RADIATION_CONSTRUCTED_FACTOR on a
  // built tile, otherwise mixed with the cell's mass against the world's radiation tuning
  // (max mass, base weight, density weight). Clamped to [0, 1].
  kAttenRadiationMass = 1,
  // `ComputeSunBeam`'s per-cell step (sim/textures.h): a SOLID takes the whole attribute
  // whatever it weighs, a fluid takes `min(mass, maxMass) / maxMass` of it. The solid half
  // is not an approximation -- it is what Klei's own DLL does.
  kAttenLightMass = 2,
};

// How attenuation combines along a walk. THESE TWO ARE NOT INTERCHANGEABLE and the tree
// contains one of each: radiation multiplies a transmission, sunlight subtracts from an
// exposure. A solver that offered only one would be wrong inside this repository.
enum ExtFieldCombine : int32_t {
  kCombineTransmission = 0,  // t *= (1 - a), the radiation walk
  kCombineExposure = 1,      // e -= a, floored at zero, the sunlight walk
};

// What happens to a field's existing value each substep, before sources are applied.
enum ExtFieldDecay : int32_t {
  // Nothing. The field holds what it was given until something writes it again -- and, with
  // no source registered either, the solver runs NO whole-grid pass for it at all. That is
  // the cheap case and it is the default.
  kDecayNone = 0,
  // `value *= decayKeep` every substep, then anything at or below `floorValue` snaps to 0.
  // Deliberately NOT Klei's radiation linger law (`have - have/linger`, with a special case
  // subtracting 1.0 when the quotient underflows); that shape is Klei's and is reproduced
  // where it belongs, in the radiation kernel, not inherited by everything downstream of it.
  kDecayFactor = 1,
};

// kRegisterField -- attach a solver to a per-cell property. IMMEDIATE, and it returns the
// field index as a `const int32_t*` (the return of `SIM_HandleMessage`), or a pointer to a
// negated `ExtRegisterResult` on refusal -- the same shape `kRegisterCellProperty` has.
//
// Immediate rather than queued for the reason the element-attribute writes are: a caller
// registers a field and then registers its sources, and a tick of latency between them is a
// race against exports that take no worker barrier. Unlike `kRegisterCellProperty` this
// CLOSES NOTHING and may arrive at any time -- it allocates no storage, because the property
// it names already did.
//
// One field per property. A second registration on the same property is a duplicate refusal.
constexpr int32_t kRegisterField = 0x4F4E494F;  // "ONIO"

#pragma pack(push, 4)
struct RegisterFieldMessage {
  int32_t propertyIdx;        // from kRegisterCellProperty; must be kExtF32 and arity 1
  int32_t attributeIdx;       // from kRegisterElementAttribute, or -1 for "no attenuation"
  float   attributeFallback;  // used for an element the attribute is UNSET for; unset is a
                              // real answer in registry 3 and no default may paper over it,
                              // so the fallback is declared here instead
  int32_t law;                // ExtFieldAttenuationLaw
  int32_t combine;            // ExtFieldCombine
  int32_t decayMode;          // ExtFieldDecay
  float   decayKeep;          // kDecayFactor: per-substep survival, [0, 1]
  float   floorValue;         // at or below this, the value snaps to 0
  float   clampLo;            // the field is clamped to [clampLo, clampHi] after every pass
  float   clampHi;
};
#pragma pack(pop)
static_assert(sizeof(RegisterFieldMessage) == 40, "RegisterFieldMessage layout drift");

// What kind of source feeds a field. Every one of these is ONE PASS with no read-after-write
// across cells, which is what lets them all hold the region-write invariant above.
enum ExtFieldSourceKind : int32_t {
  // A point emitter: an ellipse with a linear falloff and an optional cone, attenuated along
  // a ray from the emitter to each target. `tickConstant`'s shape, minus the noise. Cost is
  // O(area * ray length) -- the one expensive rule, and the one a custom radiation type needs.
  kSourcePoint = 0,
  // A parallel source arriving from a direction, swept in lanes across one world.
  // `ComputeSunBeam`'s shape. Cost is O(cells in the world), and it is what an attenuated
  // LIGHT channel wants -- the first consumer this was built for.
  kSourceDirectional = 1,
  // Every cell whose element is `target` emits `strength` per kilogram of that cell's mass,
  // into its own cell. The per-element source `StepRadiationField` runs, without Klei's
  // function-local 5x5 stencil -- that table is Klei's radiation shape, not a general one.
  kSourceElement = 2,
};

// kSetFieldSource -- add, replace or remove one source. Queued, like every other parameter.
//
// KEYED BY THE OWNER'S OWN `sourceId`, not by a handle the sim hands back, and that is a
// deliberate departure from Klei's `RadiationEmitter::Register`. Sources are never saved (a
// field's VALUES persist; its rule and its sources are content, re-pushed by their owner
// after every load, exactly as element attributes are). An owner-keyed registration makes
// that re-push idempotent instead of making it a handle-bookkeeping problem.
//
// `strength == 0` REMOVES the source, whatever else the message says. A source contributing
// nothing and a source that is not there are the same thing to the solver, and spelling the
// removal as a second message id would have meant two ways to say it.
//
// An unknown `fieldIdx`, an out-of-range cell or world, or a `kind` outside the enum is
// refused and reported on "sim.message_refused" (kExtRefusalBadTarget), not dropped silently:
// a field index is not guessable -- it comes back from a registration -- so a bad one is a
// caller bug worth surfacing.
constexpr int32_t kSetFieldSource = 0x4F4E4950;  // "ONIP"

#pragma pack(push, 4)
struct SetFieldSourceMessage {
  int32_t fieldIdx;       // from kRegisterField / SIM_ExtFieldIndex
  int32_t sourceId;       // the OWNER's id; re-sending the same one replaces that source
  int32_t kind;           // ExtFieldSourceKind
  float   strength;       // per substep, in the field's own unit; 0 removes the source
  int32_t target;         // kSourcePoint: GAME cell | kSourceDirectional: world index |
                          // kSourceElement: element id hash (SimHashes)
  int32_t radiusX;        // kSourcePoint only, in cells
  int32_t radiusY;        // kSourcePoint only, in cells
  float   coneDirection;  // kSourcePoint only, degrees; 0 is +x, as `inRadialRange`
  float   coneAngle;      // kSourcePoint only, degrees; 360 means no cone at all
  float   dirX;           // kSourceDirectional only: the direction the source arrives FROM,
  float   dirY;           // as `kSetWorldSun`'s pair. Need not be normalised.
};
#pragma pack(pop)
static_assert(sizeof(SetFieldSourceMessage) == 44, "SetFieldSourceMessage layout drift");

// A cap rather than unbounded growth, for `SIM_ExtFieldDescribe`'s linear walk and for the
// same reason the other two registries have one: a mod registering in a loop should hit a
// wall with a name in the log rather than an allocation failure.
constexpr int32_t kExtMaxFields = 32;
// Per field. A point source is the expensive rule; this is what stops one field from
// silently costing a hundred ellipse rasters a substep.
constexpr int32_t kExtMaxFieldSources = 64;

// DEFINED IN `abi/sim_ext_api.h`, not here, and this is an alias rather than a
// second copy: this type appears in an EXPORT'S SIGNATURE, and two definitions of
// a type a function takes cannot be checked against each other by any compiler.
// One definition, so there is nothing to drift. The field list and its comments
// are in that file; everything either side of this line is the reasoning, which
// stays here. See sim_ext_api.h's "HOW THE TWO HEADERS ARE KEPT IN STEP".
using ExtFieldDesc = ::OniExtFieldDesc;
static_assert(sizeof(ExtFieldDesc) == 96, "ExtFieldDesc layout drift");

// int32_t SIM_ExtFieldCount(void);
// int32_t SIM_ExtFieldIndex(const char* propertyName);
// int32_t SIM_ExtFieldDescribe(int32_t fieldIdx, ExtFieldDesc* out);
//
// The discovery path, in the shape registries 1, 2 and 3 already have: Count to enumerate,
// Index to probe by a name the caller already holds, Describe to read one row back.
//
// `SIM_ExtFieldIndex` takes the PROPERTY's name, not a name of its own, and that is the whole
// design restated as an export signature: a field has no name space, because a field is a
// property that has been given a solver. A second naming registry would be a second place for
// two mods to collide, and the property registry already refuses a duplicate.
//
// THERE IS DELIBERATELY NO READ EXPORT. A field's values are the property's values, which
// `SIM_ExtPublishedProperties` already hands back zero-copy and `SIM_ExtReadCellProperty`
// already reads one cell of. A `SIM_ExtReadField` would be a second way to ask exactly the
// same question, and the two would eventually disagree about padding, about arity, or about
// what an unallocated world returns.
//
// All three take the worker barrier, for the reason `SIM_ExtCellPropertyDescribe` gives: this
// is fetched once per world rather than once per frame, and `kSetFieldSource` drains on the
// worker, so a `sourceCount` read without one would be a count from the middle of a drain.

// kSetPayloadMixing -- how fast a liquid-carried amount EVENS OUT across a body of liquid
// (payload mixing).
//
// `kSetCellPropertyTransport` above makes an amount ride the liquid's mass. That is transport and
// nothing else: an amount goes where its water goes, and a cell whose water never moves never
// gains or loses any. So a pond carbonated by a column of bubbles kept the carbonation in the
// columns the bubbles rose through and nowhere else -- measured, with vents in columns
// 12-14 a sensor one column away read 0.000 g/kg, exactly zero and structurally so, while the
// pond's own mean understated the water a pump beside it drew by more than 2x.
//
// WHAT THE MISSING MECHANISM ACTUALLY IS. Not molecular diffusion: CO2 in water is about
// 1e-9 m2/s, which moves a gram across a one-metre cell in something like eleven days, and a
// kernel modelling it faithfully would be a kernel that does nothing. It is CONVECTION -- bulk
// motion of the water itself, which an ONI cell has no representation of because an ONI cell has
// one velocity and it is zero. A 1 m3 cell is far larger than every eddy inside it, so the honest
// model is the one every engineering text uses at that scale: an EFFECTIVE mixing rate between
// adjacent parcels, with the rate standing in for motion the grid cannot resolve.
//
// WHERE THE DEFAULT COMES FROM. `share` is the fraction of a neighbour pair's CONCENTRATION gap
// closed per pair per substep, and the default is 0.125 -- which is ONI's own house constant for
// a cell-to-cell spreading process (`kDiseaseDiffusionShare`, and the same 0.125 both gas sweeps
// and `StepLiquidDisplacement` move per pair). Reused rather than invented, and then CHECKED
// against the two mechanisms that actually stir a pond, both of which bracket it:
//
//   * a bubble plume, which is what an aerated vessel has: airlift circulation runs at roughly
//     0.2-0.5 m/s, so a 1 m cell turns over in 2-5 s, which at a 0.2 s substep is 0.04-0.10 of
//     the cell per substep;
//   * solutal convection with no bubbles at all, which is what a settled carbonated pond has:
//     our own solution model puts soda-strength water 0.27% denser than plain water
//     (`Solubility.SolutionDensityKgPerM3`), and sqrt(g * 0.0027 * 1 m) is 0.16 m/s, a 6 s
//     turnover, 0.033 per substep.
//
// So the range the physics supports is about 0.03-0.10 and the house constant sits just above it.
// It is kept anyway, and this is the PREFERENCE half of the decision rather than a derivation: a
// number a third faster than the fastest mechanism that justifies it is inside the error of every
// term feeding it, and matching the constant three other sweeps already use is worth more than a
// fourth number nobody can re-derive. A caller who disagrees sends this message.
//
// WHAT IT IS NOT. It is not stratification. Carbonated water is DENSER than plain water, so in
// reality the rich parcel sinks and a pond ends up richer at the bottom than at the top; this
// kernel is isotropic and mixes up exactly as readily as down, so it produces a UNIFORM pond
// rather than a stratified one. That is a smaller error than the one it removes -- uniform is
// wrong by the stratification gradient, columns-only was wrong by everything -- and it is listed
// as unbuilt rather than hidden.
//
// IT IS NOT SAVED, exactly as `kSetCellPropertyTransport` is not. A mixing rate is a statement
// about the property, which is re-registered from scratch every session, not about the world --
// so the sim re-applies its own default to `sim.dissolved_mass` when it registers it, and a mod
// that changed the rate re-sends this message the same way it re-sends its registration.
//
// STATIONEERS DOES NOT HAVE THIS QUESTION. An `Atmosphere` is one well-mixed `GasMixture` over
// one `VolumeLitres`, so every mole in it is uniform by construction and `AtmosphericsManager`
// moves whole mixtures between atmospheres rather than parcels within one. There is no
// sub-volume for a gradient to exist in, and so no rate to copy.
constexpr int32_t kSetPayloadMixing = 0x4F4E4951;  // "ONIQ"

// `SetPayloadMixingMessage::propertyIdx`: every liquid-following property at once. A caller
// tuning one property sends its index; a caller who wants the whole lane set moved sends this.
constexpr int32_t kPayloadMixingAllProperties = -1;

// What the sim gives `sim.dissolved_mass` at registration, so a world that sends no message at
// all still mixes. A property registered by a MOD defaults to zero -- the sim will not decide
// that somebody else's tracer is stirred.
constexpr float kPayloadMixingDefaultShare = 0.125f;

#pragma pack(push, 4)
struct SetPayloadMixingMessage {
  int32_t propertyIdx;  // from kRegisterCellProperty, or kPayloadMixingAllProperties
  float   share;        // fraction of a pair's concentration gap closed per pair per substep.
                        // 0 switches mixing off for the property. Clamped to 0..1 rather than
                        // refused: above 1 a pair would overshoot and oscillate, and a caller
                        // asking for "as fast as possible" means 1, not divergence.
};
#pragma pack(pop)
static_assert(sizeof(SetPayloadMixingMessage) == 8, "SetPayloadMixingMessage layout drift");

// ---------------------------------------------------------------------------------------
// kSetDissolvedTint -- WATER THAT LOOKS LIKE IT IS CARRYING SOMETHING (Layer C).
//
// Layer C has always been invisible. A pond at soda strength and a pond of plain water render
// byte-identically, because the liquid property texture's RGB is a walk of the ELEMENT's own
// `gradientColours` and a dissolved amount is not an element. The two mixture overlays added in
// the same session answer "what is in here" on demand; this answers it without the player having
// to ask, which is the difference between a simulation feature and a simulation feature somebody
// notices.
//
// WHY THE SIM AND NOT A SHADER. `Klei/Liquid` reads its per-cell colour from `_LiquidTex`, and
// `_LiquidTex` is `PropertyTextures.Property.Liquid`, which is `externalLiquidTex` -- a pointer
// THIS DLL fills (`UpdateLiquidPropertyTexture`, reproduced in sim/textures.h). So the tint needs
// no shader change, no bundle and no material: blending it into the bytes written here puts it
// under the waves, the caustics, the refraction and the freezing overrides for free, and every
// one of those keeps working because nothing downstream can tell the difference between a tinted
// water cell and a differently-coloured liquid.
//
// It is also the only cheap place to do it. The managed alternative is to copy the whole
// 4-bytes-per-cell texture out of native memory every frame, tint it and hand it back -- 1.5 MB
// of memcpy per frame on a 512x768 asteroid to touch the handful of cells that carry anything.
// The sweep here already visits exactly the cells that need it: `FillPropertyTextures` skips
// every cell whose element has not changed UNLESS it is liquid, and a liquid cell is recomputed
// every frame regardless, so the tint costs one extra read of the dissolved lanes on cells the
// loop was already rewriting.
//
// THE COLOURS COME FROM THE CALLER, and they have to. A lane is a managed-side assignment (which
// gas owns which lane is content data, per `kDissolvedMassProperty`), and the sim has no colour
// table for elements at all. So the message carries one colour per lane. `laneWeight` carries
// the other thing the sim cannot know: moles per kilogram, i.e. the reciprocal of the gas's
// molar mass, so that the blend below is by MOLE FRACTION rather than by mass fraction.
// Sending 1.0 for every lane is a deliberate, and documented, mass-fraction blend instead.
//
// THE STRENGTH IS A SEPARATE AXIS FROM THE HUE, and that is this project's own addition rather
// than Stationeers': an `Atmosphere` is one uniform volume whose colour only has to say WHAT is
// in it, while a grid cell's colour also has to say HOW MUCH, or a trace and a saturated cell
// paint the same. So `laneColour` decides the direction and `fullScaleGramsPerKg` with `maxBlend`
// decide the distance travelled.
//
// WHAT IT NEVER DOES: it never replaces the water's colour, only moves it. `maxBlend` is the
// fraction of the way to the mixture colour that a cell AT FULL SCALE travels, and it is clamped
// to 0..1 with a default well below 1, because water carrying dissolved gas is still water and a
// player has to be able to tell it from a different liquid at a glance.
//
// IT IS NOT SAVED, for the same reason `kSetPayloadMixing` is not: it is a statement about how to
// DRAW a property, made out of managed content data that is rebuilt from scratch every session.
// A world loaded with no message sent renders exactly as it did before this existed, because
// `enabled` defaults to 0.
constexpr int32_t kSetDissolvedTint = 0x4F4E4953;  // "ONIS"

// "ONIR" (0x4F4E4952) is deliberately SKIPPED: it is reserved for a message not in this
// release. A gap in this space costs nothing, and two messages sharing an id would be read as
// each other.

// What a caller who has not thought about it should send. 10 g/kg is where the chain actually
// operates -- carbon dioxide's Henry saturation in water is about 1.7 g/kg at one atmosphere and
// 293 K, and the carbonation vessel holds its pond at 6 g/kg under four -- so ordinary carbonated
// water lands about four fifths of the way up the ramp with headroom above it.
constexpr float kDissolvedTintDefaultFullScale = 10.0f;

// How far a cell at full scale moves toward the mixture's colour. 0.45: enough that a carbonated
// pond is unmistakably not plain water at a glance, little enough that it is unmistakably still
// water. Below about 0.25 the difference does not survive the wave and caustic layers drawn over
// it; above about 0.6 water starts reading as a different liquid.
constexpr float kDissolvedTintDefaultMaxBlend = 0.45f;

#pragma pack(push, 4)
struct SetDissolvedTintMessage {
  int32_t  enabled;              // 0 switches the tint off entirely, which is the default
  float    fullScaleGramsPerKg;  // grams of dissolved gas per kilogram of liquid at full strength
  float    maxBlend;             // 0..1, clamped; how far a full-strength cell moves
  // Per lane, indexed exactly as kDissolvedMassProperty's components are.
  float    laneWeight[kDissolvedGasLanes];  // moles per kilogram (1 / molar mass); 0 = ignore lane
  uint32_t laneColour[kDissolvedGasLanes];  // 0x00RRGGBB, the colour that lane pulls towards
};
#pragma pack(pop)
static_assert(sizeof(SetDissolvedTintMessage) == 76, "SetDissolvedTintMessage layout drift");

// ---------------------------------------------------------------------------------------
// kSetEffervescence -- SUPERSATURATED LIQUID FIZZES.
//
// A supersaturated cell nucleates bubbles when c > c_eq * (1 + margin), driven by warming, by
// the pressure above the water dropping, or by agitation. Without this, gas could leave water
// only if something carried the water away or a rising bubble happened to strip it, so a
// carbonated pond that warmed, or whose headspace was vented, would stay supersaturated
// indefinitely.
//
// THE CRITERION IS THE TOTAL GAS TENSION, NOT ONE GAS AT A TIME. A bubble nucleus holds every
// dissolved gas at once, so it can grow when the partial pressures of all of them together
// exceed the pressure on the liquid. For each lane with a Henry constant,
//
//     c_eq,i = H_i(T) * P * M_i * V          (kg the cell would hold if that gas were alone
//                                              at the full local pressure P)
//     S      = sum_i  kg_i / c_eq,i          (1 = exactly saturated)
//
// and the cell fizzes when S > 1 + margin. That is Henry's law summed -- `kg_i / (H_i M_i V)` is
// gas i's own equilibrium partial pressure -- over the local pressure. With one gas it is the
// single-gas `Solubility.SaturationFraction` the managed side already uses, exactly.
//
// P IS THE PRESSURE AT THE CELL'S CENTRE, the same number `Bubbles.PressureAtPa` gives the
// managed side: the surface gas pressure capping the liquid column (below), plus g times every
// kilogram of SOLUTION above the centre -- dissolved gas included, and half of the cell's own. A
// column capped by vacuum with no gas near it has no surface term. Depth matters, as it does in
// a real bottle: the bottom of a deep pond holds gas that its surface cannot.
//
// A COLUMN CAPPED BY A SOLID IS CONFINED AND NEVER FIZZES. A full, rigid, sealed vessel has no
// volume for a bubble, so its liquid's pressure rises to hold the gas in (a sealed bottle stays
// flat until it is opened); the column walk cannot see that pressure. The same rule keeps the gas
// in under an overhang, where water touching rock belongs to a pond whose free surface is
// elsewhere -- erring towards no fizz, never towards a fizz at a wrong pressure.
//
// THE SURFACE READS THE GAS ABOVE IT, POOLED. From the cap upward, up to 8 cells
// and stopping at the first solid or liquid, every cell's moles-times-temperature and volume are
// summed and one ideal-gas pressure is read off the sums: P = R * sum(n T) / sum(V). A cell's
// volume is its own tile plus whatever the liquid cell under it leaves empty (negative when that
// solution has swelled past a tile), and a vacuum cell adds volume and no moles. A cell's gas is
// its element's AND the mixture layer's (a promoted room keeps its gas in the mixture and its
// cells read vacuum, so reading the element alone would see a vacuum surface). This is
// Stationeers' `Room.CacheRoomData`, which sums a room's `GasMixture` and `Volume` and reads
// `IdealGas.Pressure` once, on a bounded column instead of a flood-filled room. WHY: a single
// cell is not a reading of the pressure on a surface. On the DISSOLVE rig the
// census put 2207 of 2210 false fizzes under a cap holding a few GRAMS of CO2 -- the pocket the
// falling water vacated, which ONI's slow gas spreading had not refilled from the 1.8 kg cells
// right above it -- read at a few hundred pascals, S 77.8. Pooled with the column above it, that
// pocket reads the headspace's pressure, as the physical gas it stands for would.
//
// A CAP THAT HOLDS NOTHING IS WEIGHED BY ITS NEIGHBOURS. The liquid kernel moves water a whole
// cell per substep and leaves the cell it vacated empty until gas flows back in, so a sloshing
// pond under a full atmosphere shows a vacuum cap for a few substeps at a time. So a cap that is
// not solid and holds no gas -- vacuum, or a liquid cell the kernel has emptied -- takes the
// highest of: the pooled column above and including it, the cell to its left, the cell to its
// right. Only a cap with no gas beside or above it is a real vacuum surface at 0 Pa, which is
// what keeps a pond open to space degassing. `SIM_DebugEffervescenceCensus` counts columns and
// fizzes by which case capped them, `kEffervescenceCensus*` below.
//
// V IS THE VOLUME THE SOLUTION OCCUPIES -- liquid mass over its density plus each dissolved
// lane's moles times its partial molar volume -- which is what `Solubility.EquilibriumKg`
// uses. A half-full cell holds half as much, not as much as a full one.
//
// WHAT FIZZES OUT. Every lane is scaled by the same factor, towards S = 1:
//
//     released_i = kg_i * (1 - 1/S) * (1 - exp(-rate * period))
//
// A lane with no Henry constant is not a gas this message describes: it is not counted in S
// and never released. A release smaller than `minReleaseKg` in total is skipped, so a cell only
// just past the threshold does not emit a trickle of records each too small to become a bubble.
// The records go out on "sim.liquid_payload_released" with reason
// `kPayloadReleasedEffervescence`, and the managed side spawns them as bubbles in the cell --
// which then rise, and on the way strip more gas out of any supersaturated water they cross
// (C4). A fizzing column feeds itself, which is the physics of a shaken bottle.
//
// WHY THE SIM, and not a managed pass. The criterion needs the whole column above each cell,
// every cell, on a cadence -- a managed sweep would have to read the dissolved property through
// the worker barrier or through a published copy one tick stale, and it could not see the
// liquid a substep had just moved. Here it is one top-down walk per column, carrying the column
// mass as it goes, so every cell costs O(1).
//
// THE DATA COMES FROM THE CALLER, exactly as the tint's does: which gas owns which lane is
// managed content (`kDissolvedMassProperty`), and so are the Henry constants (Sander), the van 't
// Hoff temperature terms, the partial molar volumes and which liquids are solvents at all. The
// sim reads molar masses of SURFACE gases from `sim.molecular_mass` through
// `ElementTable::MolecularMassOf`, which resolves them the same way the managed
// `AtmosphereFacade.MolarMassGPerMol` does.
//
// OFF UNTIL SENT. `enabled` is 0 by default and the phase returns on its first line, so a world
// nobody configures is byte-identical without it -- which is what keeps every diffsim golden.
//
// IT IS NOT SAVED, for the same reason `kSetPayloadMixing` and `kSetDissolvedTint` are not: it is
// content data rebuilt from scratch every session, and a mod re-sends it the way it re-sends its
// registrations.
//
// STATIONEERS HAS NO EQUIVALENT. Its `Atmosphere` holds liquids and gases in one `GasMixture` with
// no dissolved state between them, and no effervescence or nucleation code exists in its
// assemblies, so there is nothing here to match or to diverge from.
constexpr int32_t kSetEffervescence = 0x4F4E4955;  // "ONIU"

// "ONIT" (0x4F4E4954) is deliberately SKIPPED, the same way "ONIR" is: it is reserved for a
// message not in this release. See `kExtMessageReservedIds`.

// Fizz once the cell holds 10% more than the pressure on it can keep in solution. Real water
// in a vessel nucleates on its walls and on particles at small supersaturations (homogeneous
// nucleation would need hundreds of times saturation, which is why a clean glass of soda keeps
// most of its gas); 10% is well clear of the ~1% per metre by which pressure changes with
// depth, so settled water does not chatter between fizzing and not.
constexpr float kEffervescenceDefaultMargin = 0.10f;
// The fraction of the excess released per second: a time constant of 50 s. An opened, still
// bottle takes tens of minutes; a vessel that bubbles through its own depth, as a 1 m cell does,
// is far faster. What this is tuned for is that a depressurised pond visibly fizzes for a minute
// or two rather than venting in a single frame or never.
constexpr float kEffervescenceDefaultRatePerSecond = 0.02f;
// How often the pass runs, seconds of sim time. The release uses the elapsed time, so this sets
// the size of each burst of bubbles, not the rate: 1 s makes one bubble group per cell per second.
constexpr float kEffervescenceDefaultPeriodSeconds = 1.0f;
// 0.5 g: below this a cell's release is skipped rather than sent. See above.
constexpr float kEffervescenceDefaultMinReleaseKg = 5.0e-4f;
// How many solvent liquids one message can name.
constexpr int32_t kEffervescenceMaxSolvents = 8;

// `SIM_DebugEffervescenceCensus` fields, in order: one double each, cumulative since the sim
// was initialised. Columns are liquid runs the pass walked (counted once per pass); fizz counts are
// cells that released, and kg what they released, by what capped their column.
constexpr int32_t kEffervescenceCensusPasses = 0;
constexpr int32_t kEffervescenceCensusColumnsGas = 1;
constexpr int32_t kEffervescenceCensusColumnsBorrowed = 2;
constexpr int32_t kEffervescenceCensusColumnsVacuum = 3;
constexpr int32_t kEffervescenceCensusColumnsConfined = 4;
constexpr int32_t kEffervescenceCensusFizzGas = 5;
constexpr int32_t kEffervescenceCensusFizzBorrowed = 6;
constexpr int32_t kEffervescenceCensusFizzVacuum = 7;
constexpr int32_t kEffervescenceCensusKgGas = 8;
constexpr int32_t kEffervescenceCensusKgBorrowed = 9;
constexpr int32_t kEffervescenceCensusKgVacuum = 10;
// The fizz with the highest S since the sim was initialised, as the kernel saw it -- the ground
// truth a managed reading of the same cell cannot give, because the managed arrays are a frame
// behind the substep that decided. Cell and cap cell are GAME cell indices; the cap is the cell
// the surface pressure was read from (the neighbour, for a borrowed cap), -1 for vacuum.
constexpr int32_t kEffervescenceCensusWorstS = 11;
constexpr int32_t kEffervescenceCensusWorstCell = 12;
constexpr int32_t kEffervescenceCensusWorstPressurePa = 13;
constexpr int32_t kEffervescenceCensusWorstSurfacePa = 14;
constexpr int32_t kEffervescenceCensusWorstTemperatureK = 15;
constexpr int32_t kEffervescenceCensusWorstVolumeM3 = 16;
constexpr int32_t kEffervescenceCensusWorstDissolvedKg = 17;
constexpr int32_t kEffervescenceCensusWorstCapCell = 18;
constexpr int32_t kEffervescenceCensusWorstCapElement = 19;
constexpr int32_t kEffervescenceCensusWorstCapMassKg = 20;
constexpr int32_t kEffervescenceCensusWorstCapTemperatureK = 21;
// The surface exchange: liquid surfaces it examined (top cells under a gas cap,
// once per pass), surfaces that released, and the kg they released. `SIM_DebugEffervescenceCensus`
// copies min(capacity, fields), so a reader built for the 22-field census still reads its 22.
constexpr int32_t kEffervescenceCensusSurfaces = 22;
constexpr int32_t kEffervescenceCensusSurfaceReleases = 23;
constexpr int32_t kEffervescenceCensusSurfaceKg = 24;
// Surface uptake: surfaces that absorbed in a pass, the kg they absorbed, and the kJ
// the absorbed gas carried above the liquid's temperature and paid into it (signed: gas colder
// than the liquid pays negative).
constexpr int32_t kEffervescenceCensusSurfaceUptakes = 25;
constexpr int32_t kEffervescenceCensusSurfaceUptakeKg = 26;
constexpr int32_t kEffervescenceCensusSurfaceUptakeHeatKJ = 27;
constexpr int32_t kEffervescenceCensusFields = 28;

// THE SURFACE EXCHANGE. Fizzing needs the dissolved gas to beat the TOTAL
// pressure on the liquid, which is right for a bubble -- a bubble has to push the whole
// atmosphere aside to exist -- and wrong for the liquid's free surface, where gas leaves by
// diffusion and the only thing holding it in is the PARTIAL pressure of that same gas above.
// Carbonated water under air goes flat without a single bubble; before this it never did.
//
// Once per pass, at the TOP liquid cell of every column whose cap holds gas (a borrowed, vacuum
// or confined cap exchanges nothing; vacuum already fizzes hard), per described lane i with a
// known element:
//
//     p_i      = R * sum(n_i T) / sum(V)    over the same pooled column the surface pressure uses
//     c_eq,i   = H_i(T) * p_i * M_i * V      (V the top cell's solution volume)
//     released = (kg_i - c_eq,i) * (1 - exp(-surfaceRate * period))   when kg_i > c_eq,i
//
// on "sim.liquid_payload_released" with reason `kPayloadReleasedSurface`; the managed side puts it
// into the cap cell as gas. A cell releasing less than `surfaceMinReleaseKg` over all lanes
// releases nothing. Only the top cell exchanges -- gas deeper down reaches it by
// `kSetPayloadMixing`, as in a real pond -- and it runs BEFORE that cell's fizz test.
//
// BOTH DIRECTIONS. A lane holding LESS than c_eq,i absorbs
//
//     absorbed = (c_eq,i - kg_i) * (1 - exp(-surfaceRate * period))
//
// drawn from the CAP CELL alone -- its vanilla gas first, then its mixture-layer slot -- and at
// most half of what that cell holds of the gas per pass, never leaving a vanilla gas cell under
// 2 g. The pooled column sets the pressure; only the cell touching the surface gives up gas, and
// gas flow refills it. Published on the same stream as a NEGATIVE
// record with reason `kPayloadAbsorbedSurface`. The absorbed gas pays what it carried above the
// liquid's temperature, `m c_gas (T_cap - T_liquid)`, into the liquid cell, since a dissolved
// lane holds its gas at the liquid's temperature. Both ledgers charge the grid's own change at
// `dissolved_surface` (SIM_DebugLedger field 17, SIM_DebugEnergyLedger field 39).
//
// Absorption has its own copy of the threshold: fewer than `surfaceMinReleaseKg` over all lanes
// absorbs nothing. At the default rate and a 1 s period that is a 10 g deficit in the top cell,
// so oxygen under an ordinary 100 kPa oxygen atmosphere (~42 g per cell at saturation) is taken
// up, and oxygen or CO2 at air's partial pressures (under 10 g) is not.
//
// With the surface exchange on, the pass also runs when nothing is dissolved anywhere yet --
// absorption is how plain water starts holding gas -- so `DissolvedMaybeNonzero` no longer gates
// it on its own.
//
// THE RATE IS A GAME RATE, not a measured one. A real still pond's liquid-film transfer
// coefficient, ~1e-5 to 1e-4 m/s over a 1 m cell, is 1e-5 to 1e-4 of the excess per second: a
// carbonated pond would take days of game time to go flat. 1e-3 per second (~45% of the top
// cell's excess per 600 s cycle) is 10-100x that, chosen so it happens within a cycle or two.
constexpr float kEffervescenceDefaultSurfaceRatePerSecond = 1.0e-3f;
// 10 mg: a surface releasing less than this over all its lanes in one pass releases nothing.
constexpr float kEffervescenceDefaultSurfaceMinReleaseKg = 1.0e-5f;

#pragma pack(push, 4)
struct SetEffervescenceMessage {
  int32_t  enabled;          // 0 switches the pass off, which is the default
  float    margin;           // fizz when S > 1 + margin; clamped to >= 0
  float    ratePerSecond;    // fraction of the excess released per second; clamped to >= 0
  float    periodSeconds;    // sim seconds between passes; <= 0 means the default
  float    minReleaseKg;     // a cell releasing less than this in total releases nothing
  // Per lane, indexed exactly as kDissolvedMassProperty's components are. A lane whose Henry
  // constant is 0 is not described: it counts towards nothing here and never fizzes.
  float    laneHenryMolPerM3Pa[kDissolvedGasLanes];      // H at 298.15 K
  float    laneVantHoffK[kDissolvedGasLanes];            // d ln H / d(1/T); 0 = no T term
  float    laneMolarMassKgPerMol[kDissolvedGasLanes];    // real molecular mass
  float    lanePartialMolarVolumeM3PerMol[kDissolvedGasLanes];  // volume a mole adds; may be 0
  // The liquids gas dissolves in. A liquid not named here holds nothing in solution as far as
  // this pass is concerned, and its dissolved lanes -- if something put gas there -- never fizz.
  int32_t  solventCount;                                   // 0..kEffervescenceMaxSolvents
  int32_t  solventElementIdx[kEffervescenceMaxSolvents];   // element TABLE index
  float    solventFactor[kEffervescenceMaxSolvents];       // multiplies every H (salting out)
  float    solventDensityKgPerM3[kEffervescenceMaxSolvents];  // the PURE liquid's density
};
// The surface exchange's fields, APPENDED. A sender may send the 248-byte message alone (the
// exchange is then off) or this 288-byte one; a sim without the exchange reads the first 248
// bytes of the longer one and ignores the rest, because `Payload` refuses only a SHORT payload.
struct SetEffervescenceMessageV2 {
  SetEffervescenceMessage base;
  float    surfaceRatePerSecond;   // fraction of a surface's excess released per second; 0 = off
  float    surfaceMinReleaseKg;    // a surface releasing less than this in total releases nothing
  int32_t  laneElementIdx[kDissolvedGasLanes];  // element TABLE index of each lane's gas; -1 none
};
#pragma pack(pop)
static_assert(sizeof(SetEffervescenceMessage) == 248, "SetEffervescenceMessage layout drift");
static_assert(sizeof(SetEffervescenceMessageV2) == 288, "SetEffervescenceMessageV2 layout drift");

// ---------------------------------------------------------------------------------------
// kSetTunable -- EVERY FIXED NUMBER THE SIM RUNS ON, LIVE.
//
// `sim/tunables.def` is the table: one row per number, Klei's own included, each with the stock
// game's value as its default. This message sets one row, or with `tunableId == -1` restores every
// row to the build's defaults. The id is the row's position in the file, and rows are append-only,
// so an id never changes meaning.
//
// IMMEDIATE, not queued, and that is forced rather than chosen. `SubstepSeconds` is the clock
// contract with the managed game, so it changes only while no world is allocated. A queued message drains inside a frame, and there is no frame without a world, so
// a queued `SubstepSeconds` could never be applied at all. Immediate costs nothing the queue would
// have bought: `SIM_HandleMessage` waits for the worker before any immediate handler runs, and the
// conduit kernel runs on the game thread this arrives on, so nothing is stepping while a value
// changes, and every kernel copies the table when it starts (tunables.h, THE ONE RULE).
//
// It RETURNS, which a queued message could not: a pointer to an int32 holding 0 (applied) or the
// `ExtRefusalReason`. The refusal also goes on "sim.message_refused". Refused, each for its own
// reason: an id with no row (`kExtRefusalBadTarget`); `reserved` not 0, or the high 32 bits set on
// a 32-bit row (`kExtRefusalReservedBits`); a NaN or an infinity (`kExtRefusalNotFinite`); a value
// outside the row's [min, max] (`kExtRefusalOutOfRange`; a CEILING row's max is its stock value); `SubstepSeconds` while a world is allocated (`kExtRefusalWorldLoaded`); and a
// value that breaks a rule between two rows (`kExtRefusalCrossField`): an unstable solid's
// countdown roll, `(int)(32767 * StableTicksRerollScale) + StableTicksRerollBase`, must stay under
// 31, which is the "roll again" sentinel of its 5-bit field; and each clamp's floor must not
// exceed its ceiling (conduction, radiation). A reset is refused as a whole, and changes nothing,
// if it would move `SubstepSeconds` while a world is allocated.
//
// NOT SAVED, and not reset by `SIM_Shutdown`, `SIM_Initialize` or an `Allocate`: a mod
// re-sends after each launch. Last writer wins.
//
// STATIONEERS HAS NO EQUIVALENT: its constants are compiled in and its runtime knobs are
// per-device settings. This is a deliberate divergence: every number is tunable here.
constexpr int32_t kSetTunable = 0x4F4E4956;  // "ONIV"
constexpr int32_t kTunableResetAll = -1;
#pragma pack(push, 4)
struct SetTunableMessage {
  int32_t  tunableId;  // a row of sim/tunables.def, or kTunableResetAll
  uint32_t reserved;   // must be 0
  uint64_t valueBits;  // float / int32: the low 32 bits, high 32 zero; double: all 64
};
#pragma pack(pop)
static_assert(sizeof(SetTunableMessage) == 16, "SetTunableMessage layout drift");

// ---------------------------------------------------------------------------------------
// kSetVisibilityState -- the NINTH checkpoint component: the game's visibility mask, all three
// buffers deep.
//
// The sim keeps the visibility mask the game hands over with every `PrepareGameData` the way
// the game's own library does: one buffer a frame reads (`SimData::visibleGrid`)
// and the two `GameData` buffers `FrameSync` alternates, so a frame reads the mask the game sent
// two `PrepareGameData`s earlier. Two things read it and refuse a cell the player cannot see:
// `SpawnFallingLiquid`, and a solid element emitter's ore drop.
//
// AllocateCells, InitializeFromCells, Load and Start all zero the three buffers, as a new world
// in Klei's starts with zeroed ones. That is deliberately unchanged: a colony loaded in the game
// sees nothing for its first frames exactly as it does on vanilla, and the game sends a fresh
// mask every frame after. This message exists for a caller RECONSTRUCTING a run, where those
// zeroed frames are a difference from the run being reconstructed rather than a start.
//
// Without it, a replay restored from a checkpoint diverges from the run it replays: liquid the
// run handed over as falling liquid stays in the grid, because the frames after the restore read
// a zeroed mask. Removing only the three resets on the allocate and load paths also closes the
// gap; this message closes it without changing what a load does.
//
// Payload is the header below, then `count` bytes of the buffer a frame reads, then `count`
// bytes of `GameData` buffer 0, then `count` bytes of buffer 1: `3 * count` bytes after the
// header, one byte per GAME cell (not padded). A buffer the sim holds empty is written as zeros,
// which is what the sim reads from it. `simSlot` (0 or 1) names which `GameData` buffer the
// NEXT `PrepareGameData` does not write -- the sim-side one.
//
// ONE OF THE CHECKPOINT COMPONENTS; the complete list is at kSetExtCellState.
//
// Pairs with the SIM_DebugVisibilityState export, the getter, whose output is this message's
// payload byte for byte: a caller never assembles one.
//
// THE FIRST FRAME AFTER THE LOAD PUTS THEM WHERE THEY WERE. The getter reads the buffers after
// the captured frame was published, and a publication ends by swapping the frame's buffer with
// the sim-side one. The first `PrepareGameData` after a Load (or a Start) runs a frame on the
// calling thread that publishes the restored world again, swap included. So the sim stores the
// frame and sim-side buffers the other way round, and that publication swaps them back: from
// then on the buffers are the captured ones and the next physics frame reads what the run's did.
// A read-back between this message and that frame therefore shows those two exchanged.
//
// IMMEDIATE, like kSetCellRadiation, and for a reason of the same shape: the next
// `PrepareGameData` writes its own mask into one of these buffers BEFORE a queued message would
// drain, so a queued restore would overwrite the mask that call just sent. Applied on the
// calling thread after `WaitIdle`, so send it after the Load it belongs to and before the next
// frame. A `count` that is not the loaded world's game cell count, a `simSlot` that is not 0 or
// 1, or a length that is not the header plus `3 * count` is rejected and logged, never partly
// applied.
constexpr int32_t kSetVisibilityState = 0x4F4E4957;  // "ONIW"

#pragma pack(push, 4)
struct SetVisibilityStateMessage {
  int32_t count;    // game cells; each of the three buffers that follow is this many bytes
  int32_t simSlot;  // 0 or 1: the GameData buffer the next PrepareGameData does not write
  // uint8_t frame[count];   the mask the next frame reads
  // uint8_t game0[count];   GameData buffer 0
  // uint8_t game1[count];   GameData buffer 1
};
#pragma pack(pop)
static_assert(sizeof(SetVisibilityStateMessage) == 8, "SetVisibilityStateMessage layout drift");

// ---------------------------------------------------------------------------------------
// kSetLoadIsRestore -- the next Load is a checkpoint RESTORE, not a save being loaded, so it
// skips the load-time state transition.
//
// `Load` ends with the load-time state transition (`LoadTimeStateTransitions`, sim/physics.h):
// any cell outside its element's range by more than the 3 K margin takes one transition step
// with the 1.5 K overshoot. That is right for a save -- the game's own library does it, and a
// colony loaded in the game goes through it -- and wrong for a checkpoint, which is a frame of a run in progress.
// A run legitimately publishes cells out of range: `StepStateChange` runs before the liquid
// sweeps in a substep, so a drop of supercooled liquid that lands this substep is frozen by the
// next one, not this one. A restore that pushes such a cell through the load-time step starts
// its replay from a state the run never held.
//
// Measured with oni-sim-visualizer's `--timeline-test 30` on a live corpus holding liquid CO2
// falling into a cold cavity: the exact stop at tick 21 differed in exactly
// two cells, each 0.036 kg of LiquidCarbonDioxide at about 182 K in the run and
// SolidCarbonDioxide 1.5 K warmer after the restore, and 8 of 26 replayed ticks diverged (up to
// 5.4 kg) from there; seeds 3 and 11 the same. With the load-time pass skipped on the restore,
// all 30 seeks reproduce the run bit for bit on all three seeds.
//
// A ONE-SHOT FLAG. `restore` 1 arms it and 0 disarms it; the next Load, whether it succeeds or
// rejects its blob, clears it, so a flag armed for a restore that never happened cannot leak into
// a later real load. What the Load does is otherwise unchanged. Nothing else reads it: a game
// never sends it, so a save loads exactly as before, and diffsim and every golden hold.
//
// ONE OF THE CHECKPOINT MESSAGES (the list is at kSetExtCellState), and the only one sent BEFORE
// the Load rather than after it. IMMEDIATE for that reason: the Load is handled on the calling
// thread, and a queued message would drain in the first frame after it, too late. `reserved`
// must be 0; a `restore` other than 0 or 1, a non-zero `reserved` or a short payload is logged
// and changes nothing.
constexpr int32_t kSetLoadIsRestore = 0x4F4E4958;  // "ONIX"

#pragma pack(push, 4)
struct SetLoadIsRestoreMessage {
  int32_t restore;   // 1: the next Load is a restore; 0: it is not (disarm)
  int32_t reserved;  // must be 0
};
#pragma pack(pop)
static_assert(sizeof(SetLoadIsRestoreMessage) == 8, "SetLoadIsRestoreMessage layout drift");
// ---------------------------------------------------------------------------------------
// ENUMERATING the element attributes.
//
// The two lookup exports -- `SIM_ExtElementAttributeIndex` and
// `SIM_ExtElementAttribute` -- and both of them require the caller to ALREADY KNOW the name
// of the attribute and the id of the element. That is enough for a mod reading back what it
// itself pushed, and it is nothing at all for an external tool. THE DISCOVERY PATH CANNOT BE
// THE THING YOU DISCOVER THROUGH, so every registry can be enumerated from outside the DLL,
// and the attribute that shows why is the sim's own: `sim.molecular_mass` is registered in
// `World`'s constructor, natively, so a discovery path built out of a managed-side ledger of
// registrations would have been blind to the one attribute that exists in a stock build.
//
// TWO FIELDS REGISTRY 1'S DESCRIPTOR HAS AND THIS ONE MUST NOT, for the reasons stage 5
// already gives above and repeated here because a descriptor is where somebody would add
// them back: there is no `persist`, because nothing in this registry is ever saved; and there
// is no `defaultBits`, because per-element storage is sparse and UNSET IS A REAL ANSWER that
// no default may paper over. `valueCount` is how a caller sees the sparseness -- it counts
// ELEMENTS CARRYING A VALUE, not elements.
//
// `writes` is carried because it answers the question the sparseness raises. An attribute
// with `valueCount == 0` is either one nobody has pushed yet or one whose pusher ran and
// cleared everything, and `writes` (message writes since the process started, clears
// included) is the only thing that tells those apart. Stage 5 keeps the counter for exactly
// this question; it was simply not reachable.
// DEFINED IN `abi/sim_ext_api.h`, not here, and this is an alias rather than a
// second copy: this type appears in an EXPORT'S SIGNATURE, and two definitions of
// a type a function takes cannot be checked against each other by any compiler.
// One definition, so there is nothing to drift. The field list and its comments
// are in that file; everything either side of this line is the reasoning, which
// stays here. See sim_ext_api.h's "HOW THE TWO HEADERS ARE KEPT IN STEP".
using ExtElementAttributeDesc = ::OniExtElementAttributeDesc;
static_assert(sizeof(ExtElementAttributeDesc) == 80, "ExtElementAttributeDesc layout drift");

// int32_t SIM_ExtElementAttributeCount(void);
// int32_t SIM_ExtElementAttributeDescribe(int32_t attrIdx, ExtElementAttributeDesc* out);
//
// Count returns how many attributes are registered. Describe fills `out` for one index and
// returns 1, or returns 0 and leaves `out` untouched for an unregistered index, a null
// pointer, or no sim. `declaredIdx` is redundant with the index passed in for the same reason
// `ExtEventStreamDesc` carries one: a descriptor that has been copied onto a wire and sorted
// by name is no longer beside the loop counter that produced it.
//
// NEITHER TAKES THE WORKER BARRIER, and that is the stage 5 answer rather than a new one: the
// element table and the attributes hanging off it are content data, written on the calling
// thread through the immediate path that `SIM_HandleMessage` has already serialised against
// the worker, and read-only for the rest of the frame. `SIM_ExtElementAttributeIndex` and
// `SIM_ExtElementAttribute` say the same thing at greater length in simdll.cpp. Registry 1's
// descriptor exports DO take it because registration there appends to a vector until the
// first allocate closes it; nothing here is sized to the world and registration never closes.
//
// int32_t SIM_ExtElementAttributeKeys(int32_t attrIdx, int32_t* out, int32_t max);
//
// Fills `out` with the SimHashes ids that carry a value for this attribute, ASCENDING, and
// returns the total the attribute has -- which may exceed `max`, so the return value sizes the
// buffer rather than reporting the copy. `out` may be null when `max` is 0; that is the sizing
// call. Returns 0 for an unregistered index or no sim, which is indistinguishable from an
// attribute nobody has written, and deliberately so: both mean "there is nothing to read".
//
// THIS EXPORT IS WHY THE REGISTRY IS OBSERVABLE AT ALL, and probing every element of the
// loaded table instead would not do. A key is a SimHashes id, not a table index, precisely so
// it survives the table being reloaded (point 4 above) -- so a mod may legitimately hold a
// value for a hash that the currently loaded ElementTable has no row for, and a client that
// enumerates the managed element table and asks about each row would show that value as
// absent rather than as what it is: a value for an element this table does not have.

// =======================================================================================
// THE FRAME, AS A PUBLISHED ORDERING
// =======================================================================================
//
// Every extension in this file is DATA. Nothing here registers a function pointer: a callback into the step ends
// the `diffsim` acceptance test, ends determinism (and ONI saves the sim grid, so divergence
// is a corrupt save, not a glitch), forecloses region-parallel stepping permanently, and puts
// third-party crashes in native frames under our name. A data ABI can be widened later; a
// callback ABI cannot be withdrawn.
//
// But "it is data" only answers *what* a mod hands over. It does not answer *when the sim
// reads it*, and that question decides behaviour just as hard -- a value read in conduction
// and the same value read after flow are two different mechanics. That ordering existed and
// was correct long before this block; it simply was not published, so a mod author had to
// infer it from kernel names in a profiler dump. This is the published form.
//
// WHAT IS GUARANTEED
//
//   * The order below is the order the phases run in, within one substep, within one region.
//   * A phase's position relative to its neighbours is stable across builds. It is an ABI
//     surface: reordering these is a breaking change and gets the same treatment as changing
//     a struct layout.
//   * Each phase's declared scope (below) says how often it runs relative to a frame.
//
// WHAT IS EXPLICITLY *NOT* GUARANTEED, and each omission is load-bearing
//
//   * ORDER ACROSS REGIONS. Region A's conduction and region B's conduction have no defined
//     order relative to each other, and nothing may depend on one. This is the reservation
//     that keeps region-parallel stepping open -- the whole reason callbacks were declined.
//   * HOW MANY SUBSTEPS A FRAME RUNS. `SubstepsForFrame` derives it from elapsed time and
//     carries a remainder; a frame may run zero, one, or several. Anything that must happen
//     once per frame belongs in the drain or the publish, not in a substep phase.
//   * THAT A GATED PHASE RUNS AT ALL. See `kPhaseGate*`. A world with no disease never runs
//     the disease phases; a world that never sent `kInjectGasSpecies` never runs mixing.
//     "It did not run" is a normal outcome, not an error.
//   * ANY TIMING WITHIN A PHASE. Cell iteration order inside a kernel is that kernel's own
//     business and is not published.
//
// The phase list is exposed at runtime by `SIM_ExtPhaseCount` / `SIM_ExtPhaseDescribe` so a
// caller reads it rather than hardcoding it.
//
// This list is the ONE definition. The enum, the name table in simdll.cpp and the driver's
// drift check all expand this macro, so an edit that reorders phases cannot leave one of the
// three behind -- a failure mode a comment could only ask people not to hit.
//
// X(enumerator, "name", scope, gate)
#define ONI_EXT_PHASE_LIST(X)                                                                 \
  X(kPhaseDrain,               "DrainQueue",              kPhaseScopeFrame,        kPhaseGateNone)     \
  X(kPhaseWorldEnvironment,    "StepWorldEnvironment",    kPhaseScopeRegion,       kPhaseGateSubstep)  \
  X(kPhaseConduction,          "StepConduction",          kPhaseScopeRegion,       kPhaseGateSubstep)  \
  X(kPhaseStateChange,         "StepStateChange",         kPhaseScopeRegion,       kPhaseGateSubstep)  \
  X(kPhaseGasPressure,         "StepGasPressure",         kPhaseScopeRegion,       kPhaseGateSubstep)  \
  X(kPhaseGasDisplacement,     "StepGasDisplacement",     kPhaseScopeRegion,       kPhaseGateSubstep)  \
  X(kPhaseFlow,                "StepFlow",                kPhaseScopeRegion,       kPhaseGateSubstep)  \
  X(kPhaseLiquidDisplacement,  "StepLiquidDisplacement",  kPhaseScopeRegion,       kPhaseGateSubstep)  \
  X(kPhasePayloadMixing,       "StepPayloadMixing",       kPhaseScopeRegion,       kPhaseGateWorld)    \
  X(kPhaseDiseaseDiffusion,    "StepDiseaseDiffusion",    kPhaseScopeRegion,       kPhaseGateWorld)    \
  X(kPhaseRadiationField,      "StepRadiationField",      kPhaseScopeRegion,       kPhaseGateSubstep)  \
  X(kPhaseElementConsumers,    "StepElementConsumers",    kPhaseScopeRegion,       kPhaseGateSubstep)  \
  X(kPhaseElementEmitters,     "StepElementEmitters",     kPhaseScopeRegion,       kPhaseGateSubstep)  \
  X(kPhaseRadiationEmitters,   "StepRadiationEmitters",   kPhaseScopeRegion,       kPhaseGateSubstep)  \
  X(kPhaseFields,              "StepFields",              kPhaseScopeRegion,       kPhaseGateWorld)    \
  X(kPhaseElementChunks,       "StepElementChunks",       kPhaseScopeRegion,       kPhaseGateSubstep)  \
  X(kPhaseBuildingHeat,        "StepBuildingHeatExchange", kPhaseScopeRegion,      kPhaseGateSubstep)  \
  X(kPhaseBuildingToBuilding,  "StepBuildingToBuilding",  kPhaseScopeRegion,       kPhaseGateSubstep)  \
  X(kPhaseDiseaseEmitters,     "StepDiseaseEmitters",     kPhaseScopeRegion,       kPhaseGateSubstep)  \
  X(kPhasePostProcess,         "StepPostProcess",         kPhaseScopeRegion,       kPhaseGateSubstep)  \
  X(kPhaseZeroMassless,        "ZeroMasslessCells",       kPhaseScopeGridInRegion, kPhaseGateSubstep)  \
  X(kPhaseDiseasePostProcess,  "StepDiseasePostProcess",  kPhaseScopeRegion,       kPhaseGateWorld)    \
  X(kPhaseGasMixing,           "TickRoomPooledMixing",    kPhaseScopeSubstep,      kPhaseGateWorld)    \
  X(kPhaseEffervescence,       "StepEffervescence",       kPhaseScopeSubstep,      kPhaseGateWorld)    \
  X(kPhaseProject,             "Project",                 kPhaseScopeFrame,        kPhaseGateNone)

// How often a phase runs, relative to one call of `SIM_Update`.
enum ExtPhaseScope : int32_t {
  // Once per frame, outside the substep loop entirely. The drain and the publish.
  kPhaseScopeFrame = 0,
  // Once per substep, outside the per-region loop. A frame that runs three substeps runs
  // this three times, whatever the region count.
  kPhaseScopeSubstep = 1,
  // Once per region per substep -- the common case, and the shape `SimBase::UpdateData`
  // itself has.
  kPhaseScopeRegion = 2,
  // Sits inside the per-region loop but sweeps THE WHOLE GRID each time, so on a world with
  // N regions it covers every cell N times per substep. Called out separately rather than
  // filed under `kPhaseScopeRegion` because that difference is invisible on the
  // single-region worlds the offline suite runs, and is exactly the kind of thing a mod
  // author would otherwise discover in production. `ZeroMasslessCells` is the only one: a
  // mover may empty a cell outside the rectangle it is sweeping, because the destination
  // test is the active mask, which is the union of every region.
  kPhaseScopeGridInRegion = 3,
};

// Why a phase might not run on a given frame. A bitmask, so a phase can carry both.
enum ExtPhaseGate : int32_t {
  // Runs on every frame the sim steps at all.
  kPhaseGateNone = 0,
  // Skipped entirely on a frame that runs no substeps -- which is an ordinary frame, not an
  // error: a paused game, a frame whose elapsed time did not add up to a whole 0.2 s substep,
  // or the skipped frame a `Load` inserts.
  kPhaseGateSubstep = 1,
  // Additionally skipped by a world-state condition: disease switched off for the disease
  // phases, `kInjectGasSpecies` never sent for mixing. Implies `kPhaseGateSubstep`.
  kPhaseGateWorld = 3,
};

enum ExtPhase : int32_t {
#define ONI_EXT_PHASE_ENUM(e, n, s, g) e,
  ONI_EXT_PHASE_LIST(ONI_EXT_PHASE_ENUM)
#undef ONI_EXT_PHASE_ENUM
  kPhaseCount
};

// DEFINED IN `abi/sim_ext_api.h`, not here, and this is an alias rather than a
// second copy: this type appears in an EXPORT'S SIGNATURE, and two definitions of
// a type a function takes cannot be checked against each other by any compiler.
// One definition, so there is nothing to drift. The field list and its comments
// are in that file; everything either side of this line is the reasoning, which
// stays here. See sim_ext_api.h's "HOW THE TWO HEADERS ARE KEPT IN STEP".
using ExtPhaseDesc = ::OniExtPhaseDesc;
static_assert(sizeof(ExtPhaseDesc) == 44, "ExtPhaseDesc layout drift");

// int32_t SIM_ExtPhaseCount(void);
// int32_t SIM_ExtPhaseDescribe(int32_t phaseIdx, ExtPhaseDesc* out);
//
// Returns 1 and fills `out` for a valid index; returns 0 and LEAVES `out` UNTOUCHED for an
// out-of-range index or a null pointer, the same discipline the other Describe exports hold
// -- a caller that ignores the return value must not find a plausible descriptor sitting in
// its buffer.
//
// Unlike every other export in this file these two read no world state, take no worker
// barrier, and are callable BEFORE `SIM_AllocateCells` -- the table is compile-time. That is
// deliberate: a mod that wants to check the phase list it was built against still matches the
// DLL it loaded should be able to do so at load time, before a world exists.

// =======================================================================================
// THE MESSAGE SURFACE, AS A PUBLISHED TABLE
// =======================================================================================
//
// The block above publishes WHEN the sim reads a value. This one publishes WHEN A MESSAGE
// TAKES EFFECT, which is the other half of the same question and the one a caller gets
// wrong: whether a message applies on this call or at the top of the next frame should not be
// something a mod author discovers by hitting it.
//
// It is not hypothetical and the cost has already been paid once. `kSetMolecularMass` was
// MADE immediate because the tick of latency was a real defect -- the store it writes is
// read by exports that take no worker barrier, so a queued write from the worker thread was
// a race, and the latency forced every caller to push its values as early as possible.
//
// WHAT A CALLER GETS FROM THIS
//
//   * `delivery` -- QUEUED means a send-then-read-back in the same tick returns the OLD
//     value, because the message is copied onto a queue the next frame's `DrainQueue` empties
//     (`kPhaseDrain` above). IMMEDIATE means it has already been applied when
//     `SIM_HandleMessage` returns, on the calling thread, behind the worker barrier.
//   * `messageClass` -- what kind of thing the id is, per EXTENSION-POINTS.md 3.1. This is
//     the field that answers the OTHER audit finding (6.1): six of these ids are the
//     checkpoint ABI and extend nothing, so "23 extension points" overstates the surface by
//     more than half. `kMessageClassCheckpoint` says so in a field rather than in prose
//     nobody queries.
//   * `phase` -- the `ExtPhase` that reads what the message wrote, where exactly one does.
//     `kPhaseUnpublished` where that is not a single phase: several read it (`kPromoteRoom`
//     gates four kernels; `kSetMolecularMass` feeds every mole/pressure path), or the reader
//     is the caller's own code (`kSetCellProperty` is read by whoever registered the
//     property), or nothing reads it in a step at all (the checkpoints restore state and are
//     done). An `Operation` carries `kPhaseDrain`, because for an operation the drain IS the
//     effect -- it changes the grid once and what it leaves behind is ordinary world state
//     thereafter.
//
// WHAT IT DOES NOT DO. It adds no message, changes no payload and reads no world state. Like
// the phase pair it is two read-only exports over a compile-time table, so it is callable
// before `SIM_AllocateCells` and holds "byte-identical until sent" trivially.
//
// THE ONE DEFINITION, AND THE LIMIT OF WHAT A MACRO CAN CLOSE. The rows below drive the
// index enum, the descriptor table in simdll.cpp and the id-contiguity assert, so those
// three cannot drift apart. What the macro CANNOT catch on its own is a new `constexpr
// int32_t kFoo = 0x4F4E4948` added above without a row here -- which is why
// `kExtMessageIdLast` names the last constant and the assert below fails the build the
// moment the range and the row count disagree, and why `driver/src/gastest.cpp` holds a
// hand-typed second opinion. A check generated from its own subject passes by construction.
//
// X(enumerator, "name", id, class, delivery, phase-that-reads-it)
#define ONI_EXT_MESSAGE_LIST(X)                                                               \
  X(kSetCellThermalMassBonus,      "kSetCellThermalMassBonus",      kSetCellThermalMassBonus,      \
    kMessageClassParameter,  kDeliveryQueued,    kPhaseConduction)                            \
  X(kInjectGasSpecies,             "kInjectGasSpecies",             kInjectGasSpecies,             \
    kMessageClassOperation,  kDeliveryQueued,    kPhaseDrain)                                 \
  X(kRemoveVanillaMass,            "kRemoveVanillaMass",            kRemoveVanillaMass,            \
    kMessageClassOperation,  kDeliveryQueued,    kPhaseDrain)                                 \
  X(kConvertToVanillaMass,         "kConvertToVanillaMass",         kConvertToVanillaMass,         \
    kMessageClassOperation,  kDeliveryQueued,    kPhaseDrain)                                 \
  X(kPromoteRoom,                  "kPromoteRoom",                  kPromoteRoom,                  \
    kMessageClassParameter,  kDeliveryQueued,    kPhaseUnpublished)                           \
  X(kSetInvertedGravityElement,    "kSetInvertedGravityElement",    kSetInvertedGravityElement,    \
    kMessageClassParameter,  kDeliveryQueued,    kPhaseFlow)                                  \
  X(kSetMolecularMass,             "kSetMolecularMass",             kSetMolecularMass,             \
    kMessageClassParameter,  kDeliveryImmediate, kPhaseUnpublished)                           \
  X(kSetBuildingWasteHeatKilowatts,"kSetBuildingWasteHeatKilowatts",kSetBuildingWasteHeatKilowatts,\
    kMessageClassParameter,  kDeliveryQueued,    kPhaseBuildingHeat)                          \
  X(kSetBuildingExhaust,           "kSetBuildingExhaust",           kSetBuildingExhaust,           \
    kMessageClassParameter,  kDeliveryQueued,    kPhaseBuildingHeat)                          \
  X(kSetBuildingRadiation,         "kSetBuildingRadiation",         kSetBuildingRadiation,         \
    kMessageClassParameter,  kDeliveryQueued,    kPhaseBuildingHeat)                          \
  X(kSetEnvironmentTemperature,    "kSetEnvironmentTemperature",    kSetEnvironmentTemperature,    \
    kMessageClassParameter,  kDeliveryQueued,    kPhaseBuildingHeat)                          \
  X(kSetRandomState,               "kSetRandomState",               kSetRandomState,               \
    kMessageClassCheckpoint, kDeliveryQueued,    kPhaseUnpublished)                           \
  X(kSetSchedulingState,           "kSetSchedulingState",           kSetSchedulingState,           \
    kMessageClassCheckpoint, kDeliveryQueued,    kPhaseUnpublished)                           \
  X(kSetStableTicks,               "kSetStableTicks",               kSetStableTicks,               \
    kMessageClassCheckpoint, kDeliveryQueued,    kPhasePostProcess)                           \
  X(kSetDiseaseGrowth,             "kSetDiseaseGrowth",             kSetDiseaseGrowth,             \
    kMessageClassCheckpoint, kDeliveryQueued,    kPhaseDiseasePostProcess)                    \
  X(kSetRegistryState,             "kSetRegistryState",             kSetRegistryState,             \
    kMessageClassCheckpoint, kDeliveryQueued,    kPhaseUnpublished)                           \
  X(kRegisterCellProperty,         "kRegisterCellProperty",         kRegisterCellProperty,         \
    kMessageClassStore,      kDeliveryImmediate, kPhaseUnpublished)                           \
  X(kSetCellProperty,              "kSetCellProperty",              kSetCellProperty,              \
    kMessageClassStore,      kDeliveryQueued,    kPhaseUnpublished)                           \
  X(kSetExtCellState,              "kSetExtCellState",              kSetExtCellState,              \
    kMessageClassCheckpoint, kDeliveryQueued,    kPhaseUnpublished)                           \
  X(kPublishCellProperty,          "kPublishCellProperty",          kPublishCellProperty,          \
    kMessageClassStore,      kDeliveryQueued,    kPhaseProject)                               \
  X(kSubscribeEventStream,         "kSubscribeEventStream",         kSubscribeEventStream,         \
    kMessageClassStore,      kDeliveryQueued,    kPhaseUnpublished)                            \
  X(kRegisterElementAttribute,     "kRegisterElementAttribute",     kRegisterElementAttribute,     \
    kMessageClassStore,      kDeliveryImmediate, kPhaseUnpublished)                           \
  X(kSetElementAttribute,          "kSetElementAttribute",          kSetElementAttribute,          \
    kMessageClassStore,      kDeliveryImmediate, kPhaseUnpublished)                           \
  X(kSetBuildingConvection,        "kSetBuildingConvection",        kSetBuildingConvection,        \
    kMessageClassParameter,  kDeliveryQueued,    kPhaseBuildingHeat)                          \
  X(kSetWorldEnvironment,          "kSetWorldEnvironment",          kSetWorldEnvironment,          \
    kMessageClassParameter,  kDeliveryQueued,    kPhaseUnpublished)                           \
  X(kSetWorldSun,                  "kSetWorldSun",                  kSetWorldSun,                  \
    kMessageClassParameter,  kDeliveryQueued,    kPhaseUnpublished)                           \
  X(kSetCellRadiation,             "kSetCellRadiation",             kSetCellRadiation,             \
    kMessageClassCheckpoint, kDeliveryImmediate, kPhaseRadiationField)                        \
  X(kSetBlockedGasAddPolicy,       "kSetBlockedGasAddPolicy",       kSetBlockedGasAddPolicy,       \
    kMessageClassParameter,  kDeliveryQueued,    kPhaseDrain)                                 \
  X(kSetCellPropertyTransport,     "kSetCellPropertyTransport",     kSetCellPropertyTransport,     \
    kMessageClassStore,      kDeliveryQueued,    kPhaseUnpublished)                           \
  X(kAddCellPropertyAmount,        "kAddCellPropertyAmount",        kAddCellPropertyAmount,        \
    kMessageClassStore,      kDeliveryQueued,    kPhaseUnpublished)                           \
  X(kRegisterField,                "kRegisterField",                kRegisterField,                \
    kMessageClassStore,      kDeliveryImmediate, kPhaseUnpublished)                           \
  X(kSetFieldSource,               "kSetFieldSource",               kSetFieldSource,               \
    kMessageClassParameter,  kDeliveryQueued,    kPhaseFields)                                \
  X(kSetPayloadMixing,             "kSetPayloadMixing",             kSetPayloadMixing,             \
    kMessageClassParameter,  kDeliveryQueued,    kPhasePayloadMixing)                         \
  X(kSetDissolvedTint,             "kSetDissolvedTint",             kSetDissolvedTint,             \
    kMessageClassParameter,  kDeliveryQueued,    kPhaseProject)                               \
  X(kSetEffervescence,             "kSetEffervescence",             kSetEffervescence,             \
    kMessageClassParameter,  kDeliveryQueued,    kPhaseEffervescence)                         \
  X(kSetTunable,                   "kSetTunable",                   kSetTunable,                   \
    kMessageClassParameter,  kDeliveryImmediate, kPhaseUnpublished)                           \
  X(kSetVisibilityState,           "kSetVisibilityState",           kSetVisibilityState,           \
    kMessageClassCheckpoint, kDeliveryImmediate, kPhaseUnpublished)                           \
  X(kSetLoadIsRestore,             "kSetLoadIsRestore",             kSetLoadIsRestore,             \
    kMessageClassCheckpoint, kDeliveryImmediate, kPhaseUnpublished)

// When the message takes effect, and the only field here a caller's code shape depends on.
enum ExtMessageDelivery : int32_t {
  // Copied onto the game thread's queue and applied by the NEXT frame's `DrainQueue`. A
  // read-back in the same tick sees the value the message replaced, not the one it sent.
  // This is the default and 29 of the 38 ids hold it: these messages change the world, and a
  // world change that landed mid-frame would be a change the frame the game is currently
  // holding did not contain.
  kDeliveryQueued = 0,
  // Applied inside `SIM_HandleMessage`, on the calling thread, behind `WaitIdle()`, before
  // the call returns. A read-back in the same tick sees the new value. Nine ids: the THREE
  // registrations, which HAVE to be (registration closes at the first `SIM_AllocateCells`, so
  // a queued one would drain after the array it wanted to declare had been sized, and each
  // returns the index every later message needs); the two element-attribute writes
  // (`kSetMolecularMass`, `kSetElementAttribute`), which were made immediate after the tick of
  // latency turned out to be a race against exports that take no worker barrier; `kSetTunable`;
  // and the three checkpoint messages `kSetCellRadiation`, `kSetVisibilityState` and
  // `kSetLoadIsRestore`. Each of the last four has its reason at its own constant.
  kDeliveryImmediate = 1,
};

// What kind of thing an id is. Published because the count
// alone misleads: only the Parameter and Operation rows extend the simulation's behaviour.
enum ExtMessageClass : int32_t {
  // Data a named phase reads while stepping -- the shape the extension contract is written
  // around. 17 of 38.
  kMessageClassParameter = 0,
  // An action applied during the drain. Changes the grid once and is not read again. 3 of 38.
  kMessageClassOperation = 1,
  // The extensible-storage plumbing itself: declare an array, write it, open a read window or
  // an event stream. 9 of 38. These are the mechanism the Parameters are built on rather than
  // extensions in their own right.
  kMessageClassStore = 2,
  // NOT AN EXTENSION POINT. Restores run state a save blob cannot carry, so that a replayed
  // run resumes identically. 9 of 38, and with the Stores the reason "38 extension points"
  // overstates the surface: 20 of the 38 extend behaviour -- see EXTENSION-POINTS.md 6.1,
  // which asked for exactly this field rather than another paragraph.
  kMessageClassCheckpoint = 3,
};

// `ExtMessageDesc::phase` where no single published phase reads the message: several do, the
// reader is the caller's own code, or nothing reads it in a step at all. Deliberately one
// value rather than three -- `messageClass` already separates those cases, and a caller that
// has to switch on a sentinel to learn something the neighbouring field already told it is a
// worse ABI than one that says "ask the docs".
constexpr int32_t kPhaseUnpublished = -1;

enum ExtMessageIndex : int32_t {
#define ONI_EXT_MESSAGE_ENUM(e, n, i, c, d, p) kExtMsgIdx_##e,
  ONI_EXT_MESSAGE_LIST(ONI_EXT_MESSAGE_ENUM)
#undef ONI_EXT_MESSAGE_ENUM
  kExtMessageCount
};

// The id range this file allocates from. `kExtMessageIdLast` NAMES THE LAST CONSTANT ABOVE
// and is bumped in the same edit that adds one; the assert then fails the build until
// ONI_EXT_MESSAGE_LIST has a row for it. That is the whole reason the ids are contiguous --
// a gap would make the range and the row count disagree for a reason that is not drift, and
// the check would have to be weakened to a `>=` that catches nothing.
constexpr int32_t kExtMessageIdFirst = kSetCellThermalMassBonus;  // ONI1
constexpr int32_t kExtMessageIdLast = kSetLoadIsRestore;          // ONIX
// RESERVED IDS: "ONIR" and "ONIT" are allocated to messages that are not in this release.
// Without this term the contiguity rule below could only be met by leaving a message OUT of the
// list -- and a message with a handler and no row is refused as an unknown id, because the
// dispatcher is generated from the rows (vftest: "the dissolved-tint message reaches its
// handler"). A gap that is NAMED is a gap the assert still accounts for.
constexpr int32_t kExtMessageReservedIds = 2;
static_assert(kExtMessageCount + kExtMessageReservedIds ==
                  kExtMessageIdLast - kExtMessageIdFirst + 1,
              "an extension message id exists with no ONI_EXT_MESSAGE_LIST row, or a row "
              "exists for an id this file does not define -- the list is the one definition");

// ---------------------------------------------------------------- the list, asked questions
//
// THE DISPATCHER USED TO RESTATE THIS LIST TWICE, BY HAND, AND ONE OF THE COPIES CARRIED A
// COLUMN. `simdll.cpp` held a 23-line `KnownMessage` chain ("is this one of ours?") and a
// 19-term disjunction in `QueueDeferredMessage` ("does this one defer?"). The second is the
// dangerous one: its ABSENCES were the `delivery` column. An id left out deferred nothing and
// applied immediately, an id put in deferred -- and nothing anywhere compared either against the
// column that publishes the same fact to every mod through `SIM_ExtMessageDescribe`.
//
// That is the shape of bug this project has already paid for twice: a hand-maintained copy that
// agrees with its source until the day somebody edits one of them. `kSetExtCellState` went
// missing from a managed copy for two days for exactly this reason. Both are now GENERATED from
// the list, so the published column and the dispatcher's behaviour cannot disagree -- not because
// someone remembers to check, but because there is only one place the answer is written.
//
// Both are `constexpr` and fold to a jump table at every call site; the dispatcher pays nothing
// for the change. `driver/src/gastest.cpp` asserts them against a hand-typed expectation, which
// is the one place a hand-typed list is worth having: a check generated from its own subject
// passes by construction, so the check for a generator has to be written out by a person.

// Is this id one of ours at all? True for every row of ONI_EXT_MESSAGE_LIST and nothing else.
inline constexpr bool IsExtMessage(int32_t id) {
#define ONI_EXT_MESSAGE_IS_OURS(sym, name, wire, cls, delivery, phase) \
  if (id == (wire)) return true;
  ONI_EXT_MESSAGE_LIST(ONI_EXT_MESSAGE_IS_OURS)
#undef ONI_EXT_MESSAGE_IS_OURS
  return false;
}

// Does this id go on the deferred queue rather than being applied inside `SIM_HandleMessage`?
// This IS the `delivery` column, read directly. False for an id that is not ours -- a caller
// asking about someone else's message is asking whether WE will defer it, and we will not.
inline constexpr bool IsQueuedExtMessage(int32_t id) {
#define ONI_EXT_MESSAGE_IS_QUEUED(sym, name, wire, cls, delivery, phase) \
  if (id == (wire)) return (delivery) == kDeliveryQueued;
  ONI_EXT_MESSAGE_LIST(ONI_EXT_MESSAGE_IS_QUEUED)
#undef ONI_EXT_MESSAGE_IS_QUEUED
  return false;
}

// DEFINED IN `abi/sim_ext_api.h`, not here, and this is an alias rather than a
// second copy: this type appears in an EXPORT'S SIGNATURE, and two definitions of
// a type a function takes cannot be checked against each other by any compiler.
// One definition, so there is nothing to drift. The field list and its comments
// are in that file; everything either side of this line is the reasoning, which
// stays here. See sim_ext_api.h's "HOW THE TWO HEADERS ARE KEPT IN STEP".
using ExtMessageDesc = ::OniExtMessageDesc;
static_assert(sizeof(ExtMessageDesc) == 60, "ExtMessageDesc layout drift");

// kSetTunable's discovery half, defined in abi/sim_ext_api.h for the same one-definition reason.
// `SIM_ExtTunableCount`, `SIM_ExtTunableDescribe`, `SIM_ExtTunableIndex`, `SIM_ExtTunableGet`:
// callable before a world exists, and none waits on a frame, because the table is written only
// on the calling thread while the sim is idle (see kSetTunable).
using ExtTunableDesc = ::OniExtTunableDesc;
static_assert(sizeof(ExtTunableDesc) == 312, "ExtTunableDesc layout drift");

// int32_t SIM_ExtMessageCount(void);
// int32_t SIM_ExtMessageDescribe(int32_t msgIdx, ExtMessageDesc* out);
//
// Exactly parallel to the phase pair, and holding the same refusal discipline: returns 1 and
// fills `out` for a valid index; returns 0 and LEAVES `out` UNTOUCHED for an out-of-range
// index or a null pointer, so a caller that ignores the return value cannot find a plausible
// descriptor sitting in its buffer.
//
// Like the phase pair and unlike everything else in this file, these read no world state,
// take no worker barrier, and are callable BEFORE `SIM_AllocateCells` -- the table is
// compile-time. A mod that wants to check, at load time, that the DLL it got still delivers
// the message it was built against the way it was built to expect can do so before a world
// exists, which is the only point at which it can still decline to load.

}  // namespace oni_sim::ext
