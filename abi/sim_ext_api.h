/* sim_ext_api.h -- the C ABI of the extension surface this project adds to SimDLL.dll.
 *
 * WHAT THIS FILE IS. `SimDLL.dll` as Klei ships it exports 16 symbols and understands one
 * fixed set of messages. The replacement DLL this project builds
 * understands all of those, byte for byte, and then exports 56 more symbols and understands
 * 23 more message ids of its own. This header is that extra surface, and only that: the ids,
 * the payload layouts, the descriptor structs and every added export, in plain C.
 *
 * WHAT THIS FILE IS NOT. It does not declare Klei's own 16 exports, describe Klei's messages,
 * or reveal anything about the cell grid's internal data model -- none of which is ours to
 * publish. A consumer that needs those has the game's own `Sim.cs` for them.
 *
 * TERMS. "Layer C" is the LIQUID PAYLOAD: an F32 cell property flagged
 * kTransportFollowsLiquidMass (sim.dissolved_mass is the first-party one) holds an amount carried
 * by the cell's liquid. Every liquid kernel moves it in proportion with the mass it moves, ONIQ
 * mixes it through still liquid, and liquid that is consumed or leaves the grid hands its share
 * to the liquid-payload streams. "C3" is EFFERVESCENCE (ONIU): dissolved gas beyond what the
 * pressure on a cell can hold is released as bubbles. "Layer A" is those bubbles, simulated on
 * the managed side, and "C4" is a rising bubble taking gas from supersaturated water it crosses.
 * The labels are the names of the stages these features were built in.
 *
 * WHY IT IS C AND NOT C++. Everything here crosses a DLL boundary, so everything here has to
 * be expressible in the least demanding language on either side of it. A C header binds from
 * C, from C++, from C# `[DllImport]` and from anything else that can read a struct layout; a
 * C++ one binds from C++. `abi/sim_abi_ext.h` is the C++ view of the same ABI, carries the
 * long-form reasoning for every id, and takes its descriptor struct definitions from THIS
 * file -- see "HOW THE TWO HEADERS ARE KEPT IN STEP" below.
 *
 * PLATFORM. Windows x64, 4-byte struct packing throughout, little-endian, `float` is IEEE-754
 * binary32. Pointers are 8 bytes and the struct sizes asserted below assume it.
 *
 * STABILITY. Message ids are wire values and never change meaning. Struct layouts are append-
 * only in practice but are NOT versioned: check `SIM_Version()` at load time, and check the
 * one property most likely to break your code -- whether a message takes effect on this call
 * or on the next tick -- with `SIM_ExtMessageDescribe`, which exists precisely so that a
 * caller can ask instead of assuming. Both are callable before a world exists, which is the
 * last moment a mod can still decline to load.
 *
 * HOW THE TWO HEADERS ARE KEPT IN STEP, because a published ABI header that drifts from the
 * DLL is worse than none at all:
 *
 *   1. `sim/simdll.cpp` INCLUDES THIS FILE (via `abi/sim_abi_ext.h`), so every declaration
 *      below is in scope at the definition it describes. A parameter or return type that
 *      drifts is a compile error at the definition site, not a silent mismatch -- these are
 *      `extern "C"` functions, and C++ refuses to overload one.
 *   2. The descriptor structs and the enums used in an export's signature are DEFINED HERE
 *      and aliased by `abi/sim_abi_ext.h`, so there is one definition and nothing to drift.
 *   3. The message ids, payload layouts and enums that only ever travel as bytes are
 *      RESTATED here and checked against the C++ header field by field -- size, offset and
 *      exact type -- by `driver/src/gastest.cpp`, at compile time. Restated rather than
 *      shared on purpose: a check generated from its own subject passes by construction.
 *   4. `tools/check-ext-api-header.sh` diffs the functions declared here against the export
 *      table of the DLL that was actually built, in both directions, so an export added
 *      without publishing it -- or published without existing -- fails the build.
 *
 * Every one of those four has been proven to fail on purpose; the fault table is in
 * `driver/README.md`.
 */
#ifndef ONI_SIM_EXT_API_H
#define ONI_SIM_EXT_API_H

#include <stdint.h>
#ifndef __cplusplus
#include <stdbool.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------------------
 * LINKAGE.
 *
 * Declarations carry no storage attribute by default, which is what you want when you
 * resolve these with `GetProcAddress` -- the ordinary way to bind a DLL that replaces a
 * game's own. Define ONI_SIM_IMPORT_SHARED before including this if you link against an
 * import library instead, or define ONI_SIM_API yourself to something else entirely.
 */
#if !defined(ONI_SIM_API)
#if defined(ONI_SIM_IMPORT_SHARED) && defined(_WIN32)
#define ONI_SIM_API __declspec(dllimport)
#else
#define ONI_SIM_API
#endif
#endif

/* A compile-time assertion that works in C99, C11 and C++. The sizes below are part of the
 * ABI, so a consumer whose compiler packs differently finds out here rather than at the
 * first garbled read. */
#if defined(__cplusplus)
#define ONI_SIM_STATIC_ASSERT(cond, msg) static_assert(cond, msg)
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
#define ONI_SIM_STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)
#else
/* Pre-C11: a negative array bound. The paste needs two levels of macro so that __LINE__ is
 * expanded to a number before it is glued on -- with one level every assertion in the file
 * would declare the same typedef name and the second would be a redefinition. */
#define ONI_SIM_SA_CAT2(a, b) a##b
#define ONI_SIM_SA_CAT(a, b) ONI_SIM_SA_CAT2(a, b)
#define ONI_SIM_STATIC_ASSERT(cond, msg) \
  typedef char ONI_SIM_SA_CAT(oni_sim_static_assert_, __LINE__)[(cond) ? 1 : -1]
#endif

/* -------------------------------------------------------------------------------------
 * THE MESSAGE IDS.
 *
 * Sent with Klei's own `SIM_HandleMessage(id, length, payload)`. They are framework-chosen
 * constants rather than Klei string hashes -- there is no Klei source for them to match --
 * picked in a range that reads unmistakably as ours ("ONI1".."ONIG") if one ever turns up in
 * a log beside a real SimMessageHash.
 *
 * THE IDS ARE CONTIGUOUS AND THAT IS LOAD-BEARING: the DLL asserts that the number of rows in
 * its published message table equals ONI_MSG_LAST - ONI_MSG_FIRST + 1, so an id added without
 * a published descriptor fails its build.
 *
 * Six of the 23 are the CHECKPOINT ABI -- they restore run state a save blob cannot carry so
 * a replayed run resumes identically -- and extend nothing. They are here because they are
 * part of the wire surface, not because they are extension points. `messageClass` in
 * `OniExtMessageDesc` is the field that tells them apart.
 */
enum OniExtMessageId {
  ONI_MSG_SET_CELL_THERMAL_MASS_BONUS   = 0x4F4E4931, /* "ONI1" */
  ONI_MSG_INJECT_GAS_SPECIES            = 0x4F4E4932, /* "ONI2" */
  ONI_MSG_REMOVE_VANILLA_MASS           = 0x4F4E4933, /* "ONI3" */
  ONI_MSG_CONVERT_TO_VANILLA_MASS       = 0x4F4E4934, /* "ONI4" */
  ONI_MSG_PROMOTE_ROOM                  = 0x4F4E4935, /* "ONI5" */
  ONI_MSG_SET_INVERTED_GRAVITY_ELEMENT  = 0x4F4E4936, /* "ONI6" */
  ONI_MSG_SET_MOLECULAR_MASS            = 0x4F4E4937, /* "ONI7" */
  ONI_MSG_SET_BUILDING_WASTE_HEAT_KW    = 0x4F4E4938, /* "ONI8" */
  ONI_MSG_SET_BUILDING_EXHAUST          = 0x4F4E4939, /* "ONI9" */
  ONI_MSG_SET_BUILDING_RADIATION        = 0x4F4E493A, /* "ONI:" */
  ONI_MSG_SET_ENVIRONMENT_TEMPERATURE   = 0x4F4E493B, /* "ONI;" */
  ONI_MSG_SET_RANDOM_STATE              = 0x4F4E493C, /* "ONI<" checkpoint */
  ONI_MSG_SET_SCHEDULING_STATE          = 0x4F4E493D, /* "ONI=" checkpoint */
  ONI_MSG_SET_STABLE_TICKS              = 0x4F4E493E, /* "ONI>" checkpoint */
  ONI_MSG_SET_DISEASE_GROWTH            = 0x4F4E493F, /* "ONI?" checkpoint */
  ONI_MSG_SET_REGISTRY_STATE            = 0x4F4E4940, /* "ONI@" checkpoint */
  ONI_MSG_REGISTER_CELL_PROPERTY        = 0x4F4E4941, /* "ONIA" */
  ONI_MSG_SET_CELL_PROPERTY             = 0x4F4E4942, /* "ONIB" */
  ONI_MSG_SET_EXT_CELL_STATE            = 0x4F4E4943, /* "ONIC" checkpoint */
  ONI_MSG_PUBLISH_CELL_PROPERTY         = 0x4F4E4944, /* "ONID" */
  ONI_MSG_SUBSCRIBE_EVENT_STREAM        = 0x4F4E4945, /* "ONIE" */
  ONI_MSG_REGISTER_ELEMENT_ATTRIBUTE    = 0x4F4E4946, /* "ONIF" */
  ONI_MSG_SET_ELEMENT_ATTRIBUTE         = 0x4F4E4947, /* "ONIG" */
  ONI_MSG_SET_BUILDING_CONVECTION       = 0x4F4E4948, /* "ONIH" */
  ONI_MSG_SET_WORLD_ENVIRONMENT         = 0x4F4E4949, /* "ONII" */
  ONI_MSG_SET_WORLD_SUN                 = 0x4F4E494A, /* "ONIJ" */
  ONI_MSG_SET_CELL_RADIATION            = 0x4F4E494B, /* "ONIK" checkpoint */
  ONI_MSG_SET_BLOCKED_GAS_ADD_POLICY    = 0x4F4E494C, /* "ONIL" */
  ONI_MSG_SET_CELL_PROPERTY_TRANSPORT   = 0x4F4E494D, /* "ONIM" */
  ONI_MSG_ADD_CELL_PROPERTY_AMOUNT      = 0x4F4E494E, /* "ONIN" */
  ONI_MSG_REGISTER_FIELD                = 0x4F4E494F, /* "ONIO" immediate; returns the idx */
  ONI_MSG_SET_FIELD_SOURCE              = 0x4F4E4950, /* "ONIP" */
  ONI_MSG_SET_PAYLOAD_MIXING            = 0x4F4E4951, /* "ONIQ" */
  /* "ONIR" (0x4F4E4952) is reserved for a message not in this release. */
  ONI_MSG_SET_DISSOLVED_TINT            = 0x4F4E4953, /* "ONIS" */
  /* "ONIT" (0x4F4E4954) is reserved for a message not in this release. */
  ONI_MSG_SET_EFFERVESCENCE             = 0x4F4E4955, /* "ONIU" */
  ONI_MSG_SET_TUNABLE                   = 0x4F4E4956, /* "ONIV" immediate; returns a result */
  ONI_MSG_SET_VISIBILITY_STATE          = 0x4F4E4957, /* "ONIW" checkpoint; immediate */
  ONI_MSG_SET_LOAD_IS_RESTORE           = 0x4F4E4958, /* "ONIX" checkpoint; immediate; BEFORE Load */

  ONI_MSG_FIRST = ONI_MSG_SET_CELL_THERMAL_MASS_BONUS,
  ONI_MSG_LAST  = ONI_MSG_SET_LOAD_IS_RESTORE
};

/* Ids inside ONI_MSG_FIRST..ONI_MSG_LAST that this DLL does not define, because another line of
 * development owns them. The published table holds ONI_MSG_LAST - ONI_MSG_FIRST + 1 -
 * ONI_MSG_RESERVED_IDS rows; see `kExtMessageReservedIds` in sim_abi_ext.h for why the gaps are
 * counted rather than hidden. */
#define ONI_MSG_RESERVED_IDS 2

/* -------------------------------------------------------------------------------------
 * ENUMS.
 *
 * Every one of these travels in an `int32_t` struct field rather than as an enum, so the
 * enum's own storage size is not part of the ABI. The values are.
 */

/* The scalar a registered per-cell property or per-element attribute stores. */
enum OniExtScalarType {
  ONI_EXT_F32 = 0,
  ONI_EXT_U8  = 1,
  ONI_EXT_U16 = 2,
  ONI_EXT_I32 = 3,
  /* 4 is unused on purpose (tools that dump state use it for u32).
   * Stored as a u16 element-table index. A SAVED property of this type is rewritten on load
   * through the save's element palette, so it still names the same element after mods change
   * the table; an element the loading table lacks becomes ONI_EXT_NO_ELEMENT. */
  ONI_EXT_ELEMENT_IDX = 5
};

/* "No element" in an ONI_EXT_ELEMENT_IDX component. Passes through a load untouched. */
#define ONI_EXT_NO_ELEMENT 0xFFFF

/* What happens to a registered property across a save and a load. REQUIRED at registration
 * with no default: a property that does not choose one is refused, because a persistence
 * decision that nobody has to state is a decision nobody notices getting skipped. */
enum OniExtPersistence {
  ONI_PERSIST_SAVED           = 0, /* written into the save's self-describing section */
  ONI_PERSIST_REHYDRATED      = 1, /* the registering mod rebuilds it after a load */
  ONI_PERSIST_CHECKPOINT_ONLY = 2  /* carried only by SET_EXT_CELL_STATE, for replay */
};

/* Why a registration was refused. Reported in the DLL's log, and through the
 * `sim.message_refused` event stream. */
enum OniExtRegisterResult {
  ONI_EXT_REGISTER_BAD_NAME     = 1, /* shape, charset, case or length */
  ONI_EXT_REGISTER_DUPLICATE    = 2, /* already registered; the log names both owners */
  ONI_EXT_REGISTER_RESERVED     = 3, /* "sim." / "oni." from a non-first-party caller */
  ONI_EXT_REGISTER_BAD_TYPE     = 4, /* type or persistence outside the enums */
  ONI_EXT_REGISTER_CLOSED       = 5, /* arrived after the first AllocateCells */
  ONI_EXT_REGISTER_FULL         = 6, /* registry at its cap */
  ONI_EXT_REGISTER_UNSUPPORTED  = 7, /* declared persistence has no serialiser */
  ONI_EXT_REGISTER_BAD_ARITY    = 8, /* arity below 1 or above ONI_EXT_MAX_ARITY */
  /* The three below are ONI_REGISTER_FIELD's. */
  ONI_EXT_REGISTER_BAD_PROPERTY  = 9,  /* no such property, or not F32 arity 1 */
  ONI_EXT_REGISTER_BAD_ATTRIBUTE = 10, /* neither -1 nor a registered attribute */
  ONI_EXT_REGISTER_BAD_RULE      = 11  /* law/combine/decay outside its enum, a decay factor
                                        * outside [0,1], or clampLo above clampHi */
};

/* --- THE FIELD SOLVER'S THREE ENUMS. A field is a per-cell property that has
 * been given a propagation rule; `abi/sim_abi_ext.h` at ONI_REGISTER_FIELD carries the
 * contract and the two invariants. */

/* How one cell's attenuation is computed from the per-element attribute the field names. */
enum OniExtFieldAttenuationLaw {
  ONI_ATTEN_FLAT           = 0, /* the attribute alone, mass-free */
  ONI_ATTEN_RADIATION_MASS = 1, /* the sim's radiation law: constructed-tile factor, else
                                 * mixed with cell mass against the radiation tuning */
  ONI_ATTEN_LIGHT_MASS     = 2  /* the sim's sunlight law: a solid takes the whole attribute,
                                 * a fluid takes min(mass, maxMass)/maxMass of it */
};

/* How attenuation combines along a walk. Not interchangeable: the sim contains one of each. */
enum OniExtFieldCombine {
  ONI_COMBINE_TRANSMISSION = 0, /* t *= (1 - a) */
  ONI_COMBINE_EXPOSURE     = 1  /* e -= a, floored at zero */
};

/* What happens to a field's value each substep, before its sources are applied. */
enum OniExtFieldDecay {
  ONI_DECAY_NONE   = 0, /* nothing -- and with no source either, no whole-grid pass at all */
  ONI_DECAY_FACTOR = 1  /* value *= decayKeep, then snap to 0 at or below floorValue */
};

/* What kind of source feeds a field. */
enum OniExtFieldSourceKind {
  ONI_SOURCE_POINT       = 0, /* an ellipse with falloff and an optional cone, attenuated
                               * along a ray to each target. O(area * ray length) */
  ONI_SOURCE_DIRECTIONAL = 1, /* a parallel source swept in lanes across one world. O(cells) */
  ONI_SOURCE_ELEMENT     = 2  /* every cell of one element emits per kilogram, into itself */
};

/* Why a message was refused, as reported by the `sim.message_refused` stream. */
enum OniExtRefusalReason {
  ONI_EXT_REFUSAL_SHORT_PAYLOAD   = 1,
  ONI_EXT_REFUSAL_UNKNOWN_MESSAGE = 2,
  ONI_EXT_REFUSAL_BAD_TARGET      = 3, /* SET_TUNABLE: an id with no row */
  /* The five below are ONI_MSG_SET_TUNABLE's. */
  ONI_EXT_REFUSAL_NOT_FINITE      = 4, /* a NaN or an infinity */
  ONI_EXT_REFUSAL_OUT_OF_RANGE    = 5, /* outside the row's [min, max]; a CEILING row's max is
                                        * its stock value, so a ceiling cannot be raised */
  ONI_EXT_REFUSAL_WORLD_LOADED    = 6, /* a row that may change only while no world is
                                        * allocated (ONI_TUNABLE_FLAG_NO_WORLD) */
  ONI_EXT_REFUSAL_RESERVED_BITS   = 7, /* `reserved` not 0, or a 32-bit row's high 32 bits set */
  ONI_EXT_REFUSAL_CROSS_FIELD     = 8  /* in range alone, but breaks a rule between two rows */
};

/* How often a published phase runs, relative to one sim frame. */
enum OniExtPhaseScope {
  ONI_PHASE_SCOPE_FRAME           = 0, /* once per frame, outside the substep loop */
  ONI_PHASE_SCOPE_SUBSTEP         = 1, /* once per substep, outside the region loop */
  ONI_PHASE_SCOPE_REGION          = 2, /* once per region per substep -- the common case */
  ONI_PHASE_SCOPE_GRID_IN_REGION  = 3  /* inside the region loop, but sweeps the WHOLE grid
                                        * each time: on a world with N active regions it
                                        * covers every cell N times per substep. Called out
                                        * separately because the difference is invisible on
                                        * the single-region worlds most testing uses. */
};

/* Why a phase might not run on a given frame. A bitmask: a phase can carry both. */
enum OniExtPhaseGate {
  ONI_PHASE_GATE_NONE    = 0, /* runs on every frame the sim steps at all */
  ONI_PHASE_GATE_SUBSTEP = 1, /* skipped on a frame that runs no substeps -- an ordinary
                               * frame, not an error: a paused game, or elapsed time that
                               * did not add up to a whole substep */
  ONI_PHASE_GATE_WORLD   = 3  /* additionally skipped by a world-state condition. Implies
                               * ONI_PHASE_GATE_SUBSTEP. */
};

/* WHEN A MESSAGE TAKES EFFECT. The one property here that changes the SHAPE of a caller's
 * code rather than its vocabulary, which is why it is published rather than documented. */
enum OniExtMessageDelivery {
  /* Copied onto a queue and applied by the NEXT frame's drain. A read-back in the same tick
   * returns the value the message replaced, not the one it sent. 23 of the 28 ids. */
  ONI_DELIVERY_QUEUED    = 0,
  /* Applied inside SIM_HandleMessage, on the calling thread, before the call returns. A
   * read-back in the same tick sees the new value. 4 of the 23. */
  ONI_DELIVERY_IMMEDIATE = 1
};

/* What kind of thing an id is. Published because the count alone misleads: only the
 * Parameter and Operation rows extend the simulation's behaviour. */
enum OniExtMessageClass {
  ONI_MESSAGE_CLASS_PARAMETER  = 0, /* data a named phase reads while stepping. 8 of 23. */
  ONI_MESSAGE_CLASS_OPERATION  = 1, /* an action applied during the drain. 3 of 23. */
  ONI_MESSAGE_CLASS_STORE      = 2, /* the extensible-storage plumbing itself. 6 of 23. */
  ONI_MESSAGE_CLASS_CHECKPOINT = 3  /* NOT AN EXTENSION POINT: run state a save cannot
                                     * carry, so a replay resumes identically. 6 of 23. */
};

/* `OniExtMessageDesc::phase` where no single published phase reads the message. */
#define ONI_PHASE_UNPUBLISHED (-1)

/* Caps that are ABI values because a `dropped` or a refusal is only interpretable if the
 * caller knows there is a ceiling at all. */
#define ONI_EXT_MAX_ARITY               64
#define ONI_EXT_MAX_EVENT_STREAMS       32
#define ONI_EXT_STREAM_BYTES_PER_FRAME  (1 << 20)
#define ONI_EXT_MAX_ELEMENT_ATTRIBUTES  64
#define ONI_EXT_MAX_FIELDS              32
#define ONI_EXT_MAX_FIELD_SOURCES       64

/* Names the sim itself registers. Spelled here so a caller resolving one is not retyping a
 * string literal this ABI already owns. The `sim.` prefix is reserved: a registration using
 * it from a mod is refused with ONI_EXT_REGISTER_RESERVED. */
#define ONI_PROP_THERMAL_MASS_BONUS  "sim.thermal_mass_bonus"
#define ONI_PROP_GAS_OCCUPIED_MASK   "sim.gas_occupied_mask"
#define ONI_PROP_GAS_SPECIES         "sim.gas_species"
#define ONI_PROP_GAS_MASS            "sim.gas_mass"
#define ONI_PROP_ROOM_PROMOTED       "sim.room_promoted"
#define ONI_ATTR_MOLECULAR_MASS      "sim.molecular_mass"
/* f32, arity 5, keyed by the SOURCE phase's element: the gas for condensation, the liquid for
 * boiling. [0] A and [1] B of the vapour curve T = (P_kPa / A)^(1/B); [2] the freezing (triple)
 * temperature and [3] the critical temperature it is clamped between, K; [4] latent heat of
 * vaporization, J/kg. An element with no entry never changes phase in a conduit run.
 * Pushed by OniFramework from MaterialPropertyRegistry; empty on a sim nobody has told. */
#define ONI_ATTR_PHASE_CURVE         "sim.phase_curve"
/* f32, arity 1: a liquid's density, kg/m^3, for the liquid volume a conduit run occupies. */
#define ONI_ATTR_LIQUID_DENSITY      "sim.liquid_density"
/* f32, arity 1, Pa, keyed by the GAS: the pressure below which this substance has no liquid
 * phase at all, so a cell under it refuses to condense however cold it gets. The values follow
 * Stationeers: 6.3 kPa for most substances, CO2 517 kPa, SO2 1800, N2O 800, Ozone 250,
 * Silanol 516, Helium 0. This is the term that keeps 0.6 kPa Martian CO2 a gas at 210 K. */
#define ONI_ATTR_MIN_LIQUID_PRESSURE "sim.min_liquid_pressure"
/* f32, arity 1, 0 or 1, keyed by the GAS: whether this substance condenses at all.
 * As in Stationeers this is an explicit table, NOT derivable from the pressure floor -- Helium's floor is 0.0 Pa and only the table stops it condensing. Every
 * LIQUID in that table reads false, which is why the rule this gates is additionally scoped
 * to a gas whose low-temperature transition target is a liquid: a liquid carries the same
 * curve as its gas (OniFramework pushes one MaterialProperties to both halves of a family),
 * so "has a curve" alone would have caught freezing and stopped it. */
#define ONI_ATTR_CAN_CONDENSE        "sim.can_condense"
/* f32, arity 1, J/kg: the enthalpy of FUSION. OniFramework pushes one value to every phase of
 * a family, as it does the curve; the sim reads it off the LIQUID for a freeze or a melt and off
 * the GAS for a deposition or a sublimation (vaporization + fusion). Read by the planetary latent
 * accumulator only. An element with no entry contributes nothing to a transition that needs it. */
#define ONI_ATTR_LATENT_FUSION       "sim.latent_fusion"
#define ONI_STREAM_MESSAGE_REFUSED   "sim.message_refused"
/* Layer C: gas dissolved in a cell's liquid, kg, f32 arity ONI_DISSOLVED_GAS_LANES, carried by
 * liquid mass. Lane-to-gas assignment is the managed side's content data. */
#define ONI_PROP_DISSOLVED_MASS      "sim.dissolved_mass"
#define ONI_DISSOLVED_GAS_LANES      8
#define ONI_STREAM_LIQUID_PAYLOAD_RELEASED "sim.liquid_payload_released"
#define ONI_STREAM_LIQUID_PAYLOAD_CONSUMED "sim.liquid_payload_consumed"

/* -------------------------------------------------------------------------------------
 * MESSAGE PAYLOADS.
 *
 * One struct per id that takes a fixed payload. Three ids take none of these:
 * SET_REGISTRY_STATE and SET_EXT_CELL_STATE take an opaque blob whose format is the DLL's
 * own, and the two variable-length checkpoint messages below carry a count followed by that
 * many trailing elements.
 *
 * The long-form contract for each -- what it means, when it was added, what it refuses and
 * why -- is in `abi/sim_abi_ext.h` beside the same id. What is here is the wire layout.
 */
#pragma pack(push, 4)

/* ONI1. Adds `value` J/K to a cell's heat capacity for conduction, on top of
 * mass * specificHeatCapacity. Replaces any previous bonus on that cell rather than
 * accumulating. Same wire shape as Klei's SetInsulationValue / SetStrengthValue. */
typedef struct OniSetCellThermalMassBonusMessage {
  int32_t cellIdx;
  float   value;
} OniSetCellThermalMassBonusMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetCellThermalMassBonusMessage) == 8, "ONI1 layout drift");

/* ONI2. Injects mass into a cell's multi-gas mixture -- a layer ATOP vanilla's single-element
 * cell, additive to whatever is already there. The FIRST message any world receives on this
 * id lazily activates gas mixing for that world; a world that never receives one never
 * activates it. `speciesIdx` is an element-table index, not a SimHashes id. */
typedef struct OniInjectGasSpeciesMessage {
  int32_t cellIdx;
  int32_t speciesIdx;
  float   massKg;
  float   temperatureK; /* blended into the cell's shared temperature, mass-weighted */
} OniInjectGasSpeciesMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniInjectGasSpeciesMessage) == 16, "ONI2 layout drift");

/* ONI3. Takes mass out of a cell's vanilla contents, so a caller can pair it with ONI2 into
 * a conserving transfer instead of duplicating matter. */
typedef struct OniRemoveVanillaMassMessage {
  int32_t cellIdx;
  float   massKg;
} OniRemoveVanillaMassMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniRemoveVanillaMassMessage) == 8, "ONI3 layout drift");

/* ONI4. The reverse transfer -- mixture layer back into vanilla -- as ONE atomic native
 * operation rather than a remove-then-add pair, so it cannot create mass whatever the caller
 * asks for. */
typedef struct OniConvertToVanillaMassMessage {
  int32_t cellIdx;
  int32_t speciesIdx;
  float   massKg;
  float   temperatureK;
} OniConvertToVanillaMassMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniConvertToVanillaMassMessage) == 16, "ONI4 layout drift");

/* ONI5. Flips whichever room the cell belongs to from vanilla-owned to mixture-owned.
 * Promote-only: there is no demote id. */
typedef struct OniPromoteRoomMessage {
  int32_t cellIdx;
} OniPromoteRoomMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniPromoteRoomMessage) == 4, "ONI5 layout drift");

/* ONI6. One element falls up: the vertical sign of that element's turn in the liquid flow
 * kernel is inverted. -1 clears the effect. A real physics-kernel change. */
typedef struct OniSetInvertedGravityElementMessage {
  int32_t elementIdx;
} OniSetInvertedGravityElementMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetInvertedGravityElementMessage) == 4, "ONI6 layout drift");

/* ONI7. The molecular mass the sim divides by when it counts moles, keyed on a SimHashes id.
 * <= 0 removes the override. IMMEDIATE, and deliberately: the store is read by exports that
 * take no worker barrier, so a queued write from the worker thread was a race. */
typedef struct OniSetMolecularMassMessage {
  int32_t idHash;
  float   gPerMol;
} OniSetMolecularMassMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetMolecularMassMessage) == 8, "ONI7 layout drift");

/* ONI8. Power -> heat for one building, summed with Klei's own self-heat at the same
 * post-clamp line. `handle` is the building's sim handle, not a game cell. 0 switches the
 * rule off for that building. */
typedef struct OniSetBuildingWasteHeatKilowattsMessage {
  int32_t handle;
  float   kilowatts;
} OniSetBuildingWasteHeatKilowattsMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetBuildingWasteHeatKilowattsMessage) == 8,
                      "ONI8 layout drift");

/* ONI9. Exhaust heat as a rate the sim owns and integrates over its own substeps, with the
 * ceiling Klei passes. Heat above the ceiling goes to the building's own body. */
typedef struct OniSetBuildingExhaustMessage {
  int32_t handle;
  float   kilowatts;
  float   maxTemperature;
} OniSetBuildingExhaustMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetBuildingExhaustMessage) == 12, "ONI9 layout drift");

/* ONI:. Radiative loss for one building. `surfaceAreaM2` <= 0 means the building's own
 * cell count. */
typedef struct OniSetBuildingRadiationMessage {
  int32_t handle;
  float   radiationFactor; /* emissivity 0..1; 0 switches radiation off */
  float   surfaceAreaM2;
} OniSetBuildingRadiationMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetBuildingRadiationMessage) == 12, "ONI: layout drift");

/* ONI;. The radiative sink temperature the building radiation rule loses heat to. */
typedef struct OniSetEnvironmentTemperatureMessage {
  float kelvin;
} OniSetEnvironmentTemperatureMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetEnvironmentTemperatureMessage) == 4, "ONI; layout drift");

/* ONI<. CHECKPOINT: the RNG state every draw comes off. */
typedef struct OniSetRandomStateMessage {
  uint32_t state;
} OniSetRandomStateMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetRandomStateMessage) == 4, "ONI< layout drift");

/* ONI=. CHECKPOINT: the frame scheduler's own state. Also the OUT parameter of
 * SIM_DebugSchedulingState, which is why this struct is defined here rather than restated:
 * it appears in an export's signature. */
typedef struct OniSetSchedulingStateMessage {
  int32_t  skip_physics_frames;   /* frames to publish without stepping; a Load sets 1 */
  int32_t  pressure_dir;          /* negated once per substep */
  int32_t  shuffle_dir;           /* the gas shuffle's direction */
  float    substep_carry;         /* frame time left over from the last whole substep */
  uint16_t displace_rotation;     /* incremented once per substep */
  uint8_t  first_physics_substep; /* 0 or 1 */
  uint8_t  vf_active;             /* 0 or 1; whether the volume-fraction pass is running */
} OniSetSchedulingStateMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetSchedulingStateMessage) == 20, "ONI= layout drift");

/* ONI>. CHECKPOINT, VARIABLE LENGTH: this header, then `count` bytes -- one stability
 * countdown per padded cell. */
typedef struct OniSetStableTicksMessage {
  int32_t count;
  /* uint8_t ticks[count]; */
} OniSetStableTicksMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetStableTicksMessage) == 4, "ONI> layout drift");

/* ONI?. CHECKPOINT, VARIABLE LENGTH: this header, then `count` floats, then `count` bytes. */
typedef struct OniSetDiseaseGrowthMessage {
  int32_t count;
  /* float   accum[count];  */
  /* uint8_t infest[count]; */
} OniSetDiseaseGrowthMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetDiseaseGrowthMessage) == 4, "ONI? layout drift");

/* ONIA. Declares a per-cell array. IMMEDIATE, and it has to be: registration closes at the
 * first AllocateCells, so a queued registration would drain after the array it wanted to
 * declare had already been sized. `persist` is required and has no default. */
typedef struct OniRegisterCellPropertyMessage {
  char     name[48];    /* "<owner>.<property>", NUL-terminated, lower case */
  int32_t  type;        /* OniExtScalarType */
  int32_t  persist;     /* OniExtPersistence -- REQUIRED */
  int32_t  arity;       /* components per cell, 1..ONI_EXT_MAX_ARITY */
  uint32_t defaultBits; /* reinterpreted per `type`; every component starts here */
} OniRegisterCellPropertyMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniRegisterCellPropertyMessage) == 64, "ONIA layout drift");

/* ONIB. Writes one component of one cell of one registered property. */
typedef struct OniSetCellPropertyMessage {
  int32_t  cellIdx;
  int32_t  propertyIdx; /* from SIM_ExtCellPropertyIndex */
  int32_t  component;   /* 0 .. arity-1; 0 for a scalar property */
  uint32_t valueBits;   /* reinterpreted per the property's registered type */
} OniSetCellPropertyMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetCellPropertyMessage) == 16, "ONIB layout drift");

/* ONID. Opens or closes a per-frame read window on a registered property. */
typedef struct OniPublishCellPropertyMessage {
  int32_t propertyIdx;
  int32_t enable; /* non-zero subscribes, zero unsubscribes */
} OniPublishCellPropertyMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniPublishCellPropertyMessage) == 8, "ONID layout drift");

/* ONIE. Opens or closes a per-tick event stream. */
typedef struct OniSubscribeEventStreamMessage {
  int32_t streamIdx; /* from SIM_ExtEventStreamIndex */
  int32_t enable;    /* non-zero subscribes; zero unsubscribes and drops the buffer */
} OniSubscribeEventStreamMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSubscribeEventStreamMessage) == 8, "ONIE layout drift");

/* ONIF. Declares a per-SUBSTANCE attribute, keyed by SimHashes id rather than by cell.
 * IMMEDIATE, for the same reason ONIA is. */
typedef struct OniRegisterElementAttributeMessage {
  char    name[48]; /* "<owner>.<attribute>", same validator as a cell property */
  int32_t type;     /* OniExtScalarType */
  int32_t arity;    /* components per element, 1..ONI_EXT_MAX_ARITY */
} OniRegisterElementAttributeMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniRegisterElementAttributeMessage) == 56, "ONIF layout drift");

/* ONIG. Writes or clears one element's value for one registered attribute. IMMEDIATE. */
typedef struct OniSetElementAttributeMessage {
  int32_t  attrIdx;
  int32_t  idHash;    /* SimHashes value -- the number Element::id carries */
  int32_t  component; /* 0 .. arity-1 */
  uint32_t valueBits;
  int32_t  clear;     /* non-zero removes this element's entry */
} OniSetElementAttributeMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetElementAttributeMessage) == 20, "ONIG layout drift");

/* ONIH. Stationeers-shaped convection between one building's body and its footprint cells:
 * 100 W/(m^2 K) * surfaceAreaM2 * convectionFactor * each cell's HeatExchangeRatio(), split
 * evenly over the participating cells, limited to the pair's equilibrium. A solid cell is 0
 * (ONI's own conduction owns it). `reachCells` > 0 spreads that same total over every open cell
 * within n of the building instead of its footprint alone -- a radiator is a room device, a pipe
 * is a tile device. `surfaceAreaM2` <= 0 means the building's own cell count;
 * `convectionFactor` <= 0 switches it off. Sending it also makes the building's own conduit,
 * if it is one, skip the cell ratio of ONI_CONDUIT_POLICY_CONVECTION (the cell ratio belongs to
 * this term instead). QUEUED. */
typedef struct OniSetBuildingConvectionMessage {
  int32_t handle;           /* the building's sim handle, not a game cell */
  float   convectionFactor; /* Stationeers-style convection factor; <= 0 switches it off */
  float   surfaceAreaM2;
  int32_t reachCells;       /* 0 = its own footprint; n = every open cell within n of it */
} OniSetBuildingConvectionMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetBuildingConvectionMessage) == 16, "ONIH layout drift");

#define ONI_MAX_BUILDING_CONVECTION_REACH 4

#define ONI_BUILDING_CONVECTION_W_PER_M2K 100.0f

/* ONII. The planet a world is standing on -- Flagship 2 / Mod 3. Three capabilities in one
 * record, each inert until its own field is non-zero:
 *
 *   sinkKelvin            the radiative sink buildings on this world shed into, replacing the
 *                         single global ONI_MSG_SET_ENVIRONMENT_TEMPERATURE scalar for them.
 *                         <= 0 falls back to that global.
 *   peakIrradianceWPerM2  solar flux at full sun, with fullSunLux the lux the same world reports
 *   fullSunLux            at full sun -- the DLL forms the instantaneous fraction itself from the
 *                         lux NewGameFrame already carries per region, so an eclipse and the
 *                         day/night cycle need no per-tick call. solarAbsorptivity (0..1) is how
 *                         much of the incident flux a fully exposed CELL takes; a BUILDING uses
 *                         its own radiationFactor, because a grey body's absorptivity equals its
 *                         emissivity (Kirchhoff), so no second per-building field exists.
 *   surfacePressureKPa    a cell with a clear path to the sky is driven toward this pressure of
 *   boundaryTemperatureK  boundaryElement at this temperature, closing boundaryRate of the gap
 *   boundaryRate          per substep (clamped to ONI_MAX_WORLD_BOUNDARY_RATE). The planet is an
 *   boundaryElement       INFINITE SOURCE: it is never debited for what it supplies, and the mass
 *                         and energy that enter are charged to their own ledger buckets so the
 *                         crossing is named rather than hidden. < 0 element, or <= 0 pressure or
 *                         rate, switches the boundary off.
 *
 * worldIndex indexes DefineWorldOffsets' list -- the game sends it in ClusterManager
 * WorldContainers order -- NOT the per-cell zone byte and NOT the active-region index, which
 * skips undiscovered worlds. A negative worldIndex sets the record every world with none of its
 * own falls back to. QUEUED. */
typedef struct OniSetWorldEnvironmentMessage {
  int32_t worldIndex;          /* < 0 sets the fallback record */
  float   sinkKelvin;          /* <= 0 = the global environment temperature */
  float   peakIrradianceWPerM2;
  float   fullSunLux;
  float   solarAbsorptivity;   /* 0..1; 0 switches solar heating off */
  float   surfacePressureKPa;  /* <= 0 switches the boundary off */
  float   boundaryTemperatureK;/* <= 0 = sinkKelvin */
  float   boundaryRate;        /* 0..1 of the gap per substep; 0 switches the boundary off */
  int32_t boundaryElement;     /* < 0 switches the boundary off */
} OniSetWorldEnvironmentMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetWorldEnvironmentMessage) == 36, "ONII layout drift");

#define ONI_MAX_WORLD_BOUNDARY_RATE 0.5f

/* ONIJ. The direction a world's sun shines from -- Flagship 2 / Mod 3's sun path. (dirX, dirY) is
 * the vector from a cell toward the sun in the grid plane, +x east (the x-max side), +y up;
 * normalised on receipt, and dirY <= 0 lights nothing. A world with a sun gets a second exposure
 * field walked along that direction, and its SOLAR heating (cells and buildings) reads it; the
 * atmosphere boundary, sky radiation, the latent accumulator and the game's own sunlight texture
 * keep the vertical one, so a world never sent this is unchanged. enabled == 0 forgets the
 * record. Index rules as ONII. QUEUED. */
typedef struct OniSetWorldSunMessage {
  int32_t worldIndex;          /* < 0 sets the fallback record */
  float   dirX;                /* toward the sun; + = east (x max) */
  float   dirY;                /* toward the sun; + = up; <= 0 = below the horizon */
  int32_t enabled;             /* 0 forgets the record */
} OniSetWorldSunMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetWorldSunMessage) == 16, "ONIJ layout drift");

/* ONIK. CHECKPOINT, VARIABLE LENGTH: this header, then `count` floats, one per padded cell. The
 * whole radiation field, because Load clears radiation in Vacuum and Void cells as Klei's does
 * and a running world has some there. IMMEDIATE, so the first publication after the Load already
 * carries it; send it after the Load it belongs to. */
typedef struct OniSetCellRadiationMessage {
  int32_t count;
  /* float radiation[count]; */
} OniSetCellRadiationMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetCellRadiationMessage) == 4, "ONIK layout drift");

/* ONIL. What a ModifyCell gas add does when it is blocked: the target holds a different gas
 * that cannot be displaced and no left/right/up neighbour is vacuum or the same gas. Vanilla
 * deletes the smaller of the two masses. PROMOTE_AND_MIX promotes the cell's room, moves the
 * room's gas into the mixture, then mixes the new gas in. Not saved; send once per session. */
typedef enum OniBlockedGasAddPolicy {
  ONI_BLOCKED_GAS_ADD_VANILLA          = 0,
  ONI_BLOCKED_GAS_ADD_PROMOTE_AND_MIX  = 1
} OniBlockedGasAddPolicy;

typedef struct OniSetBlockedGasAddPolicyMessage {
  int32_t policy; /* OniBlockedGasAddPolicy; any other value is refused */
} OniSetBlockedGasAddPolicyMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetBlockedGasAddPolicyMessage) == 4, "ONIL layout drift");

/* One record of the `sim.message_refused` event stream: a message the DLL declined, and why.
 * The stream exists because a refusal that is only a log line is a refusal a mod cannot
 * react to. */
typedef struct OniExtRefusedMessage {
  int32_t messageId;     /* the id as sent */
  int32_t reason;        /* OniExtRefusalReason */
  int32_t payloadBytes;  /* bytes that arrived */
  int32_t expectedBytes; /* bytes the handler needed; 0 when the reason is not about size */
} OniExtRefusedMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniExtRefusedMessage) == 16, "refusal record layout drift");

/* Layer C. ONI_MSG_SET_CELL_PROPERTY_TRANSPORT: an F32 property whose values are amounts carried
 * by the cell's liquid, moved in proportion with liquid mass by every liquid kernel. */
typedef enum OniCellPropertyTransport {
  ONI_TRANSPORT_STATIC               = 0,
  ONI_TRANSPORT_FOLLOWS_LIQUID_MASS  = 1
} OniCellPropertyTransport;

typedef struct OniSetCellPropertyTransportMessage {
  int32_t propertyIdx;
  int32_t transport; /* OniCellPropertyTransport */
} OniSetCellPropertyTransportMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetCellPropertyTransportMessage) == 8, "ONIM layout drift");

/* ONI_MSG_ADD_CELL_PROPERTY_AMOUNT: add to one component of an F32 property; a result below
 * zero is refused whole. */
typedef struct OniAddCellPropertyAmountMessage {
  int32_t cellIdx;
  int32_t propertyIdx;
  int32_t component;
  float amount;
} OniAddCellPropertyAmountMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniAddCellPropertyAmountMessage) == 16, "ONIN layout drift");

/* ONIO. Attaches a propagation rule to a registered per-cell property, making it a FIELD.
 * IMMEDIATE, and it returns the field index (or a negated OniExtRegisterResult) -- because a
 * caller registers a field and then its sources, and a tick between them is a race. It closes
 * nothing and may arrive at any time: the property it names already did the allocating. */
typedef struct OniRegisterFieldMessage {
  int32_t propertyIdx;        /* must be an ONI_EXT_F32 property of arity 1 */
  int32_t attributeIdx;       /* the per-element attenuation attribute, or -1 for none */
  float   attributeFallback;  /* for an element that attribute is unset for */
  int32_t law;                /* OniExtFieldAttenuationLaw */
  int32_t combine;            /* OniExtFieldCombine */
  int32_t decayMode;          /* OniExtFieldDecay */
  float   decayKeep;          /* per-substep survival, [0,1], for ONI_DECAY_FACTOR */
  float   floorValue;         /* at or below this, the value snaps to 0 */
  float   clampLo;
  float   clampHi;
} OniRegisterFieldMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniRegisterFieldMessage) == 40, "ONIO layout drift");

/* ONIP. Adds, replaces or removes one source of a field. Keyed by the CALLER'S `sourceId`,
 * not by a handle the sim returns, so the re-push every load needs is idempotent. A
 * `strength` of zero removes the source whatever else the message says. */
typedef struct OniSetFieldSourceMessage {
  int32_t fieldIdx;
  int32_t sourceId;       /* yours; re-sending the same one replaces that source */
  int32_t kind;           /* OniExtFieldSourceKind */
  float   strength;       /* per substep; 0 removes */
  int32_t target;         /* POINT: game cell | DIRECTIONAL: world index | ELEMENT: id hash */
  int32_t radiusX;        /* POINT only */
  int32_t radiusY;        /* POINT only */
  float   coneDirection;  /* POINT only, degrees; 0 is +x */
  float   coneAngle;      /* POINT only, degrees; 360 means no cone */
  float   dirX;           /* DIRECTIONAL only: the direction it arrives FROM. Any length. */
  float   dirY;
} OniSetFieldSourceMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetFieldSourceMessage) == 44, "ONIP layout drift");

/* ONIQ. Layer C mixing: how fast a liquid-carried amount evens out across the body of liquid
 * holding it. `share` is the fraction of a neighbour pair's CONCENTRATION gap closed per pair
 * per substep; 0 switches mixing off for that property, and a value outside [0,1] is clamped
 * rather than refused. `propertyIdx` of -1 (ONI_PAYLOAD_MIXING_ALL_PROPERTIES) sets every
 * liquid-following property at once. Transport moves an amount with its water; this is what
 * spreads it through water that is not moving. */
typedef struct OniSetPayloadMixingMessage {
  int32_t propertyIdx;
  float   share;
} OniSetPayloadMixingMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetPayloadMixingMessage) == 8, "ONIQ layout drift");

/* ONIS. The dissolved tint: a liquid cell's rendered colour moves part of the way towards the
 * mole-weighted colour of what is dissolved in it. `laneWeight` is moles per kilogram (1 / molar
 * mass) and `laneColour` is 0x00RRGGBB, both per lane of "sim.dissolved_mass". A cell at
 * `fullScaleGramsPerKg` or more moves `maxBlend` (clamped 0..1) of the way. `enabled` 0, the
 * default, draws every liquid exactly as the element table colours it. */
typedef struct OniSetDissolvedTintMessage {
  int32_t  enabled;
  float    fullScaleGramsPerKg;
  float    maxBlend;
  float    laneWeight[ONI_DISSOLVED_GAS_LANES];
  uint32_t laneColour[ONI_DISSOLVED_GAS_LANES];
} OniSetDissolvedTintMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetDissolvedTintMessage) == 76, "ONIS layout drift");

/* ONIU. Effervescence (Layer C3): a liquid cell holding more dissolved gas than the pressure on
 * it can keep in solution releases the excess as bubbles. The cell fizzes when
 *     S = sum over described lanes of  kg_i / (H_i(T) * P * M_i * V)  >  1 + margin
 * with H_i(T) = laneHenryMolPerM3Pa * solventFactor * exp(laneVantHoffK * (1/T - 1/298.15)),
 * P the pressure at the cell's centre (surface gas plus the solution column above), and V the
 * volume the solution occupies. Every lane is released by kg_i * (1 - 1/S) * (1 - exp(-rate *
 * period)) once per `periodSeconds` of sim time, on "sim.liquid_payload_released" with reason
 * ONI_PAYLOAD_RELEASED_EFFERVESCENCE. A lane with laneHenryMolPerM3Pa 0 is not described and
 * never fizzes; a liquid not named as a solvent never fizzes; a liquid column capped by a solid is
 * confined and never fizzes. `enabled` 0 is the default. */
#define ONI_EFFERVESCENCE_MAX_SOLVENTS 8

/* SIM_DebugEffervescenceCensus field indices (one double each). See kEffervescenceCensus*. */
#define ONI_EFFERVESCENCE_CENSUS_PASSES            0
#define ONI_EFFERVESCENCE_CENSUS_COLUMNS_GAS       1
#define ONI_EFFERVESCENCE_CENSUS_COLUMNS_BORROWED  2
#define ONI_EFFERVESCENCE_CENSUS_COLUMNS_VACUUM    3
#define ONI_EFFERVESCENCE_CENSUS_COLUMNS_CONFINED  4
#define ONI_EFFERVESCENCE_CENSUS_FIZZ_GAS          5
#define ONI_EFFERVESCENCE_CENSUS_FIZZ_BORROWED     6
#define ONI_EFFERVESCENCE_CENSUS_FIZZ_VACUUM       7
#define ONI_EFFERVESCENCE_CENSUS_KG_GAS            8
#define ONI_EFFERVESCENCE_CENSUS_KG_BORROWED       9
#define ONI_EFFERVESCENCE_CENSUS_KG_VACUUM         10
/* The highest-S fizz since the sim was initialised, as the kernel saw it. Cells are game
 * indices; the cap cell is where the surface pressure was read, -1 for vacuum. */
#define ONI_EFFERVESCENCE_CENSUS_WORST_S           11
#define ONI_EFFERVESCENCE_CENSUS_WORST_CELL        12
#define ONI_EFFERVESCENCE_CENSUS_WORST_PRESSURE_PA 13
#define ONI_EFFERVESCENCE_CENSUS_WORST_SURFACE_PA  14
#define ONI_EFFERVESCENCE_CENSUS_WORST_TEMPERATURE 15
#define ONI_EFFERVESCENCE_CENSUS_WORST_VOLUME_M3   16
#define ONI_EFFERVESCENCE_CENSUS_WORST_DISSOLVED   17
#define ONI_EFFERVESCENCE_CENSUS_WORST_CAP_CELL    18
#define ONI_EFFERVESCENCE_CENSUS_WORST_CAP_ELEMENT 19
#define ONI_EFFERVESCENCE_CENSUS_WORST_CAP_MASS    20
#define ONI_EFFERVESCENCE_CENSUS_WORST_CAP_TEMP    21
#define ONI_EFFERVESCENCE_CENSUS_SURFACES          22
#define ONI_EFFERVESCENCE_CENSUS_SURFACE_RELEASES  23
#define ONI_EFFERVESCENCE_CENSUS_SURFACE_KG        24
#define ONI_EFFERVESCENCE_CENSUS_SURFACE_UPTAKES   25
#define ONI_EFFERVESCENCE_CENSUS_SURFACE_UPTAKE_KG 26
#define ONI_EFFERVESCENCE_CENSUS_SURFACE_UPTAKE_HEAT_KJ 27
#define ONI_EFFERVESCENCE_CENSUS_FIELDS            28
typedef struct OniSetEffervescenceMessage {
  int32_t enabled;
  float   margin;
  float   ratePerSecond;
  float   periodSeconds;
  float   minReleaseKg;
  float   laneHenryMolPerM3Pa[ONI_DISSOLVED_GAS_LANES];
  float   laneVantHoffK[ONI_DISSOLVED_GAS_LANES];
  float   laneMolarMassKgPerMol[ONI_DISSOLVED_GAS_LANES];
  float   lanePartialMolarVolumeM3PerMol[ONI_DISSOLVED_GAS_LANES];
  int32_t solventCount;
  int32_t solventElementIdx[ONI_EFFERVESCENCE_MAX_SOLVENTS];
  float   solventFactor[ONI_EFFERVESCENCE_MAX_SOLVENTS];
  float   solventDensityKgPerM3[ONI_EFFERVESCENCE_MAX_SOLVENTS];
} OniSetEffervescenceMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetEffervescenceMessage) == 248, "ONIU layout drift");
/* The surface exchange, appended. Send this 288-byte form to switch it on; the
 * 248-byte form leaves it off. At the top liquid cell under a gas cap, each lane holding more
 * than H_i(T) * p_i * M_i * V -- p_i the PARTIAL pressure of that gas in the pooled gas column
 * above -- releases (excess) * (1 - exp(-surfaceRatePerSecond * period)) into the cell above, with
 * reason ONI_PAYLOAD_RELEASED_SURFACE. It runs both ways: a lane holding LESS absorbs
 * (deficit) * (1 - exp(-surfaceRatePerSecond * period)) out of the cell above (at most half of what
 * that cell holds of the gas per pass), paying the heat the gas carried above the liquid's
 * temperature into the liquid, and publishes it as a NEGATIVE record with reason
 * ONI_PAYLOAD_ABSORBED_SURFACE. */
typedef struct OniSetEffervescenceMessageV2 {
  OniSetEffervescenceMessage base;
  float   surfaceRatePerSecond;
  float   surfaceMinReleaseKg;
  int32_t laneElementIdx[ONI_DISSOLVED_GAS_LANES];
} OniSetEffervescenceMessageV2;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetEffervescenceMessageV2) == 288, "ONIU v2 layout drift");

/* ONIV. Sets one of the sim's tunable numbers -- every fixed number the simulation runs on, Klei's
 * included, each with the stock game's value as its default -- or, with tunableId
 * ONI_TUNABLE_RESET_ALL, restores every one of them to the build's defaults.
 *
 * IMMEDIATE, and it RETURNS: SIM_HandleMessage returns a pointer to an int32_t holding 0 when the
 * value was applied, or the OniExtRefusalReason when it was not (the same refusal also goes on the
 * `sim.message_refused` stream). The pointer is DLL-owned static storage, valid until the next
 * message. The sim is idle while the write happens, so a value never changes inside a frame.
 *
 * Discover the rows with SIM_ExtTunableCount / SIM_ExtTunableDescribe / SIM_ExtTunableIndex, and
 * read the live value back with SIM_ExtTunableGet. A tunableId is the row's position in the table
 * and never changes: rows are only ever appended.
 *
 * NOT SAVED. The table survives SIM_Shutdown, SIM_Initialize and every AllocateCells, but not the
 * process: a mod re-sends what it wants after each launch. Two mods setting one row: the last
 * write wins. */
#define ONI_TUNABLE_RESET_ALL (-1)
typedef struct OniSetTunableMessage {
  int32_t  tunableId;  /* a row, or ONI_TUNABLE_RESET_ALL (valueBits then ignored) */
  uint32_t reserved;   /* must be 0 */
  uint64_t valueBits;  /* ONI_TUNABLE_F32 / _I32: the value's 32 bits, high 32 zero.
                        * ONI_TUNABLE_F64: all 64. Bits, so a float crosses exactly. */
} OniSetTunableMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetTunableMessage) == 16, "ONIV layout drift");

/* ONIW. CHECKPOINT, VARIABLE LENGTH: this header, then three buffers of `count` bytes, one byte per
 * game cell: the visibility mask the next frame reads, then the game-side buffers 0 and 1. The
 * sim reads the mask the game sent two PrepareGameData calls earlier, and every allocate, load
 * and Start zeroes all three, as Klei's does; this puts back the mask a checkpoint ran with.
 * `simSlot` (0 or 1) is the game-side buffer the next PrepareGameData does not write. Take the
 * whole payload from SIM_DebugVisibilityState and send it back unchanged. IMMEDIATE, so the next
 * PrepareGameData's own mask is not overwritten; send it after the Load it belongs to. */
typedef struct OniSetVisibilityStateMessage {
  int32_t count;
  int32_t simSlot;
  /* uint8_t frame[count]; uint8_t game0[count]; uint8_t game1[count]; */
} OniSetVisibilityStateMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetVisibilityStateMessage) == 8, "ONIW layout drift");

/* ONIX. CHECKPOINT: the next Load is a restore of a checkpoint, not a save, so it skips the
 * load-time state transition that moves every cell outside its element's range one step. A run
 * publishes such cells (a supercooled drop that landed this substep freezes on the next), and a
 * restore that transitions them starts its replay from a state the run never held. `restore` 1
 * arms, 0 disarms; the next Load clears it either way. `reserved` must be 0. IMMEDIATE, and the
 * one checkpoint message sent BEFORE the Load it belongs to. A game never sends it. */
typedef struct OniSetLoadIsRestoreMessage {
  int32_t restore;
  int32_t reserved;
} OniSetLoadIsRestoreMessage;
ONI_SIM_STATIC_ASSERT(sizeof(OniSetLoadIsRestoreMessage) == 8, "ONIX layout drift");
#define ONI_EFFERVESCENCE_DEFAULT_SURFACE_RATE_PER_SECOND (1.0e-3f)
#define ONI_EFFERVESCENCE_DEFAULT_SURFACE_MIN_RELEASE_KG  (1.0e-5f)
#define ONI_EFFERVESCENCE_DEFAULT_MARGIN            (0.10f)
#define ONI_EFFERVESCENCE_DEFAULT_RATE_PER_SECOND   (0.02f)
#define ONI_EFFERVESCENCE_DEFAULT_PERIOD_SECONDS    (1.0f)
#define ONI_EFFERVESCENCE_DEFAULT_MIN_RELEASE_KG    (5.0e-4f)
#define ONI_PAYLOAD_MIXING_ALL_PROPERTIES (-1)
#define ONI_PAYLOAD_MIXING_DEFAULT_SHARE  (0.125f)

typedef enum OniLiquidPayloadReleaseReason {
  ONI_PAYLOAD_RELEASED_CLEARED      = 0,
  ONI_PAYLOAD_RELEASED_PHASE_CHANGE = 1,
  ONI_PAYLOAD_RELEASED_OFF_GAS      = 2,
  ONI_PAYLOAD_RELEASED_FALLING      = 3,
  ONI_PAYLOAD_RELEASED_WISP         = 4,
  ONI_PAYLOAD_RELEASED_REPLACED     = 5,
  ONI_PAYLOAD_RELEASED_REMOVED      = 6,
  ONI_PAYLOAD_RELEASED_DELETED      = 7,
  ONI_PAYLOAD_RELEASED_MASSLESS     = 8,
  /* The liquid is still there; the GAS left it, because the cell held more than the pressure on
   * it can keep in solution (ONI_MSG_SET_EFFERVESCENCE). Spawn it as bubbles in the cell. */
  ONI_PAYLOAD_RELEASED_EFFERVESCENCE = 9,
  /* The liquid is still there; the gas crossed its free surface by diffusion, because the top
   * cell held more than the PARTIAL pressure of that gas above keeps in solution. Put it into
   * the cell above as gas, not as bubbles. */
  ONI_PAYLOAD_RELEASED_SURFACE = 10,
  /* NOT a release: the free surface DISSOLVED gas out of the cell above. `amount` is NEGATIVE
   * (minus the kg absorbed) and the sim has already moved the gas; do nothing but count it.
   * `temperatureK` is the gas cell's. */
  ONI_PAYLOAD_ABSORBED_SURFACE = 11
} OniLiquidPayloadReleaseReason;

/* One record of `sim.liquid_payload_released`: payload whose liquid left the grid. */
typedef struct OniLiquidPayloadReleased {
  int32_t cell;
  int32_t propertyIdx;
  int32_t component;
  float amount;
  float temperatureK;
  int32_t reason; /* OniLiquidPayloadReleaseReason */
} OniLiquidPayloadReleased;
ONI_SIM_STATIC_ASSERT(sizeof(OniLiquidPayloadReleased) == 24, "payload released layout drift");

typedef enum OniLiquidPayloadConsumerKind {
  ONI_PAYLOAD_CONSUMER_ELEMENT_CONSUMER = 0,
  ONI_PAYLOAD_CONSUMER_MASS_CONSUMPTION = 1
} OniLiquidPayloadConsumerKind;

/* One record of `sim.liquid_payload_consumed`: payload a consumer took with its liquid. */
typedef struct OniLiquidPayloadConsumed {
  int32_t kind; /* OniLiquidPayloadConsumerKind */
  int32_t id;
  int32_t propertyIdx;
  int32_t component;
  float amount;
  float temperatureK;
} OniLiquidPayloadConsumed;
ONI_SIM_STATIC_ASSERT(sizeof(OniLiquidPayloadConsumed) == 24, "payload consumed layout drift");

#pragma pack(pop)

/* -------------------------------------------------------------------------------------
 * THE FRAME POINTER.
 *
 * The two per-frame publish exports take the pointer the sim handed back for THIS tick.
 * Its contents are Klei's frame struct and are not part of this ABI; to a consumer of this
 * header it is an opaque handle, and passing anything else returns a refusal rather than
 * reading it.
 */
#ifdef __cplusplus
namespace oni_sim { struct GameDataUpdate; }
typedef oni_sim::GameDataUpdate OniSimFrame;
#else
typedef struct OniSimFrame OniSimFrame;
#endif

/* -------------------------------------------------------------------------------------
 * DESCRIPTORS.
 *
 * These are DEFINED HERE and aliased by the C++ header, because they appear in an export's
 * signature: two definitions of a type a function takes cannot be checked against each other
 * by any compiler, so there is one.
 *
 * Every Describe export holds the same refusal discipline: it returns 1 and fills `out` for a
 * valid index, and returns 0 and LEAVES `out` UNTOUCHED for an out-of-range index, a null
 * pointer or no sim. A caller that ignores the return value therefore cannot find a plausible
 * descriptor sitting in its buffer.
 */
#pragma pack(push, 4)

/* A published per-cell property, for the tick that is being published. */
typedef struct OniExtPublishedProperty {
  char        name[48];  /* NUL-terminated, as registered */
  int32_t     type;      /* OniExtScalarType */
  int32_t     persist;   /* OniExtPersistence */
  int32_t     arity;     /* components per cell; the layout is cell-major */
  int32_t     stride;    /* bytes per component, implied by `type` */
  int32_t     cellCount; /* PADDED cells */
  int32_t     byteCount; /* cellCount * arity * stride */
  const void* data;      /* VALID FOR THIS TICK ONLY */
} OniExtPublishedProperty;
ONI_SIM_STATIC_ASSERT(sizeof(void*) == 8, "this ABI assumes a 64-bit build");
ONI_SIM_STATIC_ASSERT(sizeof(OniExtPublishedProperty) == 80, "published property drift");

/* The registry as metadata: one registered per-cell property, whether published or not. */
typedef struct OniExtCellPropertyDesc {
  char     name[48];
  int32_t  type;        /* OniExtScalarType */
  int32_t  persist;     /* OniExtPersistence */
  int32_t  arity;
  int32_t  stride;
  int32_t  cellCount;   /* PADDED cells, or 0 before the world is allocated */
  uint32_t defaultBits; /* the registered default, per component */
  int32_t  published;   /* non-zero if subscribed to the per-frame table */
} OniExtCellPropertyDesc;
ONI_SIM_STATIC_ASSERT(sizeof(OniExtCellPropertyDesc) == 76, "cell property desc drift");

/* A published event stream, for the tick that is being published. */
typedef struct OniExtPublishedStream {
  char        name[48];
  int32_t     stride;    /* bytes per record, fixed at declaration */
  int32_t     count;     /* records this frame */
  int32_t     byteCount; /* count * stride */
  int32_t     dropped;   /* records lost to the per-frame cap; 0 in the normal case */
  const void* data;      /* VALID FOR THIS TICK ONLY */
} OniExtPublishedStream;
ONI_SIM_STATIC_ASSERT(sizeof(OniExtPublishedStream) == 72, "published stream drift");

/* One declared event stream, whether subscribed or not. */
typedef struct OniExtEventStreamDesc {
  char    name[48];
  int32_t stride;
  int32_t subscribed;  /* non-zero if collecting; survives an Allocate and a load */
  int32_t declaredIdx; /* the index SIM_ExtEventStreamIndex returns for `name` */
} OniExtEventStreamDesc;
ONI_SIM_STATIC_ASSERT(sizeof(OniExtEventStreamDesc) == 60, "event stream desc drift");

/* One registered per-element attribute. */
typedef struct OniExtElementAttributeDesc {
  char     name[48];
  int32_t  type;       /* OniExtScalarType */
  int32_t  arity;      /* components per element */
  int32_t  stride;
  int32_t  valueCount; /* ELEMENTS carrying a value -- not the element count */
  int32_t  declaredIdx;
  int32_t  firstParty; /* non-zero if the sim itself registered it, not a mod */
  uint64_t writes;     /* message writes since process start, clears included -- the only
                        * thing that separates "nobody has pushed one" from "the pusher ran
                        * and cleared everything" */
} OniExtElementAttributeDesc;
ONI_SIM_STATIC_ASSERT(sizeof(OniExtElementAttributeDesc) == 80, "attribute desc drift");

/* One registered FIELD -- a per-cell property that has been given a propagation rule (the
 * contract, the two invariants and the enum values are in `abi/sim_abi_ext.h` at
 * `kRegisterField`). There is no `data` pointer and no read export: a field's values live in
 * the property named here, which `SIM_ExtPublishedProperties` already publishes zero-copy,
 * and a second way to ask the same question would be a second way to get it wrong.
 *
 * `sourceCount` is here for the rehydration question the other registries answer with a write
 * counter. A field's rule and its sources are NEVER saved -- their owner re-pushes them after
 * every load -- so "which fields was somebody supposed to re-register, and has not" has to be
 * answerable from outside, or the claim that they are re-pushed is a claim nothing can check. */
typedef struct OniExtFieldDesc {
  char    property[48];      /* the per-cell property this field lives in */
  int32_t fieldIdx;          /* echoed back, as every Describe in this header does */
  int32_t propertyIdx;
  int32_t attributeIdx;      /* the per-element attenuation attribute, or -1 for none */
  float   attributeFallback; /* used for an element that attribute is unset for */
  int32_t law;               /* OniExtFieldAttenuationLaw */
  int32_t combine;           /* OniExtFieldCombine */
  int32_t decayMode;         /* OniExtFieldDecay */
  float   decayKeep;
  float   floorValue;
  float   clampLo;
  float   clampHi;
  int32_t sourceCount;       /* sources registered right now, 0..kExtMaxFieldSources */
} OniExtFieldDesc;
ONI_SIM_STATIC_ASSERT(sizeof(OniExtFieldDesc) == 96, "field desc drift");

/* One phase of the published frame ordering. The ordering is DATA rather than a callback
 * list, so the names and indices are discovered at runtime and are deliberately NOT
 * constants in this header. */
typedef struct OniExtPhaseDesc {
  char    name[32]; /* the kernel's own name, NUL-terminated */
  int32_t index;    /* echoed back: a descriptor copied onto a wire is no longer beside
                     * the loop counter that produced it */
  int32_t scope;    /* OniExtPhaseScope */
  int32_t gate;     /* OniExtPhaseGate bitmask */
} OniExtPhaseDesc;
ONI_SIM_STATIC_ASSERT(sizeof(OniExtPhaseDesc) == 44, "phase desc drift");

/* One extension message id, and the four things about it a caller cannot otherwise ask. */
typedef struct OniExtMessageDesc {
  char    name[40];     /* the constant's own name, NUL-terminated */
  int32_t index;        /* position in the table, echoed back */
  int32_t id;           /* the WIRE id -- what SIM_HandleMessage takes. Not the index: an
                         * id survives a row being inserted above it, an index does not */
  int32_t messageClass; /* OniExtMessageClass */
  int32_t delivery;     /* OniExtMessageDelivery */
  int32_t phase;        /* the phase index that reads it, or ONI_PHASE_UNPUBLISHED */
} OniExtMessageDesc;
ONI_SIM_STATIC_ASSERT(sizeof(OniExtMessageDesc) == 60, "message desc drift");

/* The scalar a tunable row holds, and so how its 64 bits read. */
enum OniTunableType {
  ONI_TUNABLE_F32 = 0,
  ONI_TUNABLE_I32 = 1,
  ONI_TUNABLE_F64 = 2
};

/* OniExtTunableDesc.flags. */
#define ONI_TUNABLE_FLAG_CEILING  0x1 /* max equals the stock value: it may be lowered, never
                                       * raised past what the stock game uses */
#define ONI_TUNABLE_FLAG_NO_WORLD 0x2 /* refused while a world is allocated (SubstepSeconds: the
                                       * clock contract with the managed game) */

/* One tunable row. Every *Bits field reads as `type` says. */
typedef struct OniExtTunableDesc {
  char     name[48];     /* e.g. "GasPressureCap", NUL-terminated */
  char     group[16];    /* "Heat", "Phase", "Gas", ... */
  char     origin[8];    /* "KLEI" (the stock game's value), "STN", "PHYS" or "OURS" */
  char     doc[192];     /* one line: what it does, and its stock value */
  int32_t  tunableId;    /* echoed back */
  int32_t  type;         /* OniTunableType */
  int32_t  flags;        /* ONI_TUNABLE_FLAG_* */
  int32_t  reserved;
  uint64_t defaultBits;  /* what ONI_TUNABLE_RESET_ALL restores: this build's default */
  uint64_t stockBits;    /* the row's own value, before any build-time override */
  uint64_t minBits;      /* inclusive */
  uint64_t maxBits;      /* inclusive */
} OniExtTunableDesc;
ONI_SIM_STATIC_ASSERT(sizeof(OniExtTunableDesc) == 312, "tunable desc drift");

/* THE LIVE PER-KERNEL TABLE, one row per timed kernel. Rows are in slot order, which is the
 * order `bench`'s offline table and the DLL's own `Player.log` report already use.
 *
 * TWO CLOCKS IN ONE ROW, and the difference matters to anyone reading it. `msTotal` and
 * `calls` accumulate only while the profiler is ARMED (SIM_DebugSetProfiler), and are 0 for
 * every row while it is not -- with it off, the sim reads no timer at all, which is the
 * property that lets the instrumentation exist in a shipping build. The four census numbers
 * are NOT gated and accumulate on every frame of every run, so a row can honestly carry
 * millions of examined cells and 0.0 ms. A zero `msTotal` therefore means "nobody armed the
 * profiler", never "this kernel did no work"; `invocations` is the field that answers that.
 *
 * `cellSweep` says whether the census numbers are in a unit that compares. A cell sweep walks
 * a rectangle of the grid; the rest walk LISTS -- the message queue, the registered
 * buildings, the element chunks, the radiation emitters -- where a ratio against the grid
 * says nothing, so `budgetPerInvocation` is 0 on those rows rather than a number that invites
 * a meaningless division.
 *
 * `withinBudget` is the assertion: `examined <= budgetPerInvocation * invocations`, exact
 * integer arithmetic with no clock in it. A kernel that regresses to walking the whole grid
 * trips its own bound and names itself. It is computed in the DLL because the rule that picks
 * each kernel's denominator is the DLL's, and a caller re-deriving it would be re-deriving a
 * per-kernel decision rather than reading one. It is 1 on rows where it means nothing. */
typedef struct OniProfileSlot {
  char    name[32];            /* the kernel's own name, NUL-terminated */
  double  msTotal;             /* cumulative since the last arm; 0.0 while disarmed */
  int64_t calls;               /* timed entries; 0 while disarmed */
  int64_t examined;            /* cells the sweep's loop stepped over -- ALWAYS counted */
  int64_t skipped;             /* cells skipped in a RUN by a kernel-internal early-out */
  int64_t changes;             /* state changes announced; a count of changes, not of cells */
  int64_t invocations;         /* times entered -- once per region per substep */
  int64_t budgetPerInvocation; /* cells one invocation may step over; 0 when !cellSweep */
  int32_t cellSweep;           /* non-zero when the four census numbers compare */
  int32_t withinBudget;        /* non-zero when examined <= budget * invocations */
} OniProfileSlot;
ONI_SIM_STATIC_ASSERT(sizeof(OniProfileSlot) == 96, "profile slot drift");

/* The table's header row: what the per-slot numbers are to be read against. */
typedef struct OniProfileSummary {
  int32_t enabled;              /* non-zero while the millisecond half is armed */
  int32_t frames;               /* frames since the last arm -- the denominator for msTotal */
  int64_t regionCells;          /* sum of the active rectangles' areas */
  int64_t regionCellsInclusive; /* the same, grown one cell on the far edges */
  int64_t gridCells;            /* the PADDED grid: what a sweep ignoring its region sees */
  int64_t regions;              /* active regions -- one per discovered world */
  int64_t outsideAnySweep;      /* changes announced with no sweep running: queued
                                 * ModifyCells, emitters, buildings. Its own field because
                                 * charging them to whichever kernel ran last is the
                                 * misattribution that makes a profile agree with itself */
} OniProfileSummary;
ONI_SIM_STATIC_ASSERT(sizeof(OniProfileSummary) == 48, "profile summary drift");

/* -------------------------------------------------------------------------------------
 * A WHOLE ROOM, SUMMED ONCE.
 *
 * The sim already knows which 4-connected open-cell component every cell belongs to: the
 * mixing kernel needs it, so `sim/gas_rooms.h` builds it and maintains it incrementally.
 * Until this struct existed the only thing published out of that graph was one boolean
 * (SIM_DebugRoomOwned), which threw the room id away -- so a caller that wanted a room's
 * pressure or its composition had to approximate the room itself, per cell, from outside.
 *
 * IT IS THE APPROXIMATION THIS REPLACES THAT MAKES IT WORTH HAVING, not only the crossing
 * count. A radius-N diamond around a vent is not a room: it leaks through walls, it truncates
 * a large room, and it takes in cells on the far side of a door. This is the real component,
 * wall-aware by construction -- including the correction recorded in `gas_rooms.h`'s own
 * header comment, that a cell is open only if it is BOTH not marked gas-impermeable AND not
 * currently holding a solid element. Managed code re-deriving rooms would have to re-derive
 * that fix or silently reintroduce the hang it was written for.
 *
 * WHAT `roomId` IS FOR. It is stable only between rebuilds of the room graph -- a room merges
 * or splits when geometry changes, and ids are re-assigned -- so it is a grouping key for the
 * frame you read it on, not a handle to store across a dig. Two cells reporting the same id
 * on the same frame ARE in the same room, which is what makes "ask once per room" possible.
 *
 * MIXTURE MASS AND VANILLA MASS ARE REPORTED SEPARATELY, and neither is the other's fallback.
 * A room cell can hold multi-gas mixture state, or vanilla's single PhaseEntry mass, or both:
 * they are two layers over one grid (see StepPhysics's vf block). `mixtureCellCount` says how
 * many of `cellCount` cells carry any mixture state at all, so a caller can tell "this room is
 * half outside the mixture layer" from "this room is empty" -- the distinction
 * SIM_GasDominantElement's single 0xFFFF sentinel deliberately cannot draw.
 *
 * `meanPressurePa` is averaged over `mixtureCellCount`, NOT over `cellCount`: a cell with no
 * mixture state has no mixture pressure, and averaging a zero in for it would report a room
 * as emptier the less of it the mixture layer has been asked to own. When
 * `mixtureCellCount == 0` it is 0.0f and means nothing.
 *
 * `temperatureK` is mass-weighted by each cell's TOTAL mass (vanilla plus mixture) because
 * `PhaseEntry.temperature` is one shared field per cell -- the mixture layer blends incoming
 * mass into it rather than keeping a second temperature. A room with no mass anywhere reports
 * 0.0f.
 *
 * COMPOSITION OVERFLOW IS COUNTED, NEVER SILENT. `ONI_ROOM_MAX_SPECIES` bounds a ROOM, while
 * the sim's own per-cell bound is smaller (8), so a large room can legitimately hold more
 * distinct species than there are slots here. Slots are filled in first-encountered order;
 * `speciesOverflow` is the number of distinct species that did not get one. Their mass is
 * still inside `mixtureMassKg`, so `mixtureMassKg` exceeding the sum of `massBySpeciesKg` is
 * the expected, detectable consequence rather than a lost kilogram.
 */
#define ONI_ROOM_MAX_SPECIES 16

typedef struct OniRoomAggregate {
  int32_t  roomId;           /* the room's id this frame, or -1 if the cell is in no room */
  int32_t  cellCount;        /* open cells in the room -- gas_rooms.h's room_size */
  int32_t  awakeCount;       /* of those, cells not currently gas-sleeping */
  int32_t  owned;            /* non-zero once ONI_MSG_PROMOTE_ROOM has promoted this room */
  int32_t  mixtureCellCount; /* of `cellCount`, cells holding any mixture state at all */
  int32_t  speciesCount;     /* entries written into species[] / massBySpeciesKg[] */
  int32_t  speciesOverflow;  /* distinct species that found no slot; their mass is still in
                              * mixtureMassKg */
  float    mixtureMassKg;    /* multi-gas mass summed over the room */
  float    vanillaMassKg;    /* PhaseEntry mass summed over the room -- the other layer */
  float    meanPressurePa;   /* mean over mixtureCellCount cells; 0.0f when that is 0 */
  float    temperatureK;     /* mass-weighted over vanilla + mixture mass; 0.0f if massless */
  uint16_t species[ONI_ROOM_MAX_SPECIES];       /* ElementTable indices, not SimHashes ids */
  float    massBySpeciesKg[ONI_ROOM_MAX_SPECIES];
} OniRoomAggregate;
ONI_SIM_STATIC_ASSERT(sizeof(OniRoomAggregate) == 140, "room aggregate drift");

/* -------------------------------------------------------------------------------------
 * A GRID RAYCAST.
 *
 * The first cell on a line that the caller says stops it, and how much of a radiation-like
 * quantity survives the cells on the way. Answered by the same cell walk the radiation field
 * uses (Klei's RadiationAbsorptionAlongLine), so what a sensor "sees" and what radiation reaches
 * never disagree about which cells lie in between. Both endpoints are visited.
 *
 * WHAT STOPS THE RAY is either mask, ORed:
 *   `phaseMask`    one bit per phase of the cell's element -- ONI_RAYCAST_PHASE_*. On a
 *                  multi-gas cell that is the dominant-element facade the rest of the vanilla
 *                  ABI reports.
 *   `propertyMask` tested against the cell's Sim.Cell.Properties byte -- ONI_CELL_PROPERTY_*,
 *                  which are the game's own Sim.Cell.Properties values.
 * and a cell carrying any bit of `ignorePropertyMask` never stops it, whatever the other two
 * say -- it still attenuates. That third mask is what line of sight needs: a Glass Tile is a
 * SOLID cell that vanilla marks TRANSPARENT, so "stopped by solids, but not by windows" is
 * phaseMask SOLID with ignorePropertyMask TRANSPARENT. A closed door needs nothing special: the
 * game swaps its cells to the door's own solid material, and back when it opens.
 * All zero means nothing stops it: the ray runs the whole line and `transmission` is Klei's
 * RadiationAbsorptionAlongLine(start, end), bit for bit.
 *
 * `transmission` is the product of (1 - radiation absorption) over the cells from the start up
 * to AND INCLUDING `hitCell` (all of them when nothing stopped the ray), clamped to [0, 1]. It
 * uses the radiation tuning the game has set (SetRadiationParams), or the sim's defaults when it
 * has set none. It is the only per-material attenuation the sim owns; it is not light.
 *
 * Cells are GAME cell indices, the same ones Grid uses. The walk is a straight line across the
 * whole grid: it does not know where one asteroid ends and the next begins, so a ray between two
 * worlds crosses the void between them.
 *
 * An invalid query (either cell outside the grid) is answered with hitCell = lastClearCell = -1,
 * cellsVisited = 0 and transmission = 0.0f, never left unwritten; cellsVisited == 0 is the tell,
 * since a valid ray always visits at least its start. */
#define ONI_RAYCAST_PHASE_VACUUM 0x1u
#define ONI_RAYCAST_PHASE_GAS    0x2u
#define ONI_RAYCAST_PHASE_LIQUID 0x4u
#define ONI_RAYCAST_PHASE_SOLID  0x8u

#define ONI_CELL_PROPERTY_GAS_IMPERMEABLE    0x01u
#define ONI_CELL_PROPERTY_LIQUID_IMPERMEABLE 0x02u
#define ONI_CELL_PROPERTY_SOLID_IMPERMEABLE  0x04u
#define ONI_CELL_PROPERTY_UNBREAKABLE        0x08u
#define ONI_CELL_PROPERTY_TRANSPARENT        0x10u
#define ONI_CELL_PROPERTY_OPAQUE             0x20u
#define ONI_CELL_PROPERTY_NOTIFY_ON_MELT     0x40u
#define ONI_CELL_PROPERTY_CONSTRUCTED_TILE   0x80u

typedef struct OniRaycastQuery {
  int32_t  startCell;
  int32_t  endCell;
  uint32_t phaseMask;     /* ONI_RAYCAST_PHASE_* */
  uint32_t propertyMask;        /* ONI_CELL_PROPERTY_* */
  uint32_t ignorePropertyMask;  /* ONI_CELL_PROPERTY_*: a cell with any of these never stops it */
} OniRaycastQuery;
ONI_SIM_STATIC_ASSERT(sizeof(OniRaycastQuery) == 20, "raycast query drift");

typedef struct OniRaycastResult {
  int32_t hitCell;        /* first cell from the start that stops the ray, or -1 */
  int32_t lastClearCell;  /* the cell just before hitCell on the start side; endCell when
                           * nothing stopped the ray; -1 when the start cell itself stopped it */
  int32_t cellsVisited;   /* cells from the start up to and including hitCell, or the whole
                           * line; 0 only for an invalid query */
  float   transmission;   /* see above; in [0, 1] */
} OniRaycastResult;
ONI_SIM_STATIC_ASSERT(sizeof(OniRaycastResult) == 16, "raycast result drift");

/* -------------------------------------------------------------------------------------
 * A WHOLE CONDUIT RUN, SUMMED ONCE.
 *
 * The same idea as the room aggregate above, for pipes. Klei's conduit temperature manager
 * is handed every conduit's temperature, mass and element by the game on every conduit frame,
 * and until this struct existed it kept only `mass * specificHeat` and never learned where a
 * conduit was or which run it belonged to. It now keeps the mass and the element, and the
 * managed side tells it which run each conduit is in (SIM_ConduitNetworkBind) every time the
 * game re-partitions a conduit layer. The partition is the GAME'S OWN --
 * `UtilityNetworkManager`'s connected components -- so a run here is exactly a run to every
 * vanilla system, bridges and broken segments included.
 *
 * WHAT A RUN CONTAINS IS WHAT `ConduitFlow` HOLDS, AS OF THE LAST CONDUIT FRAME, PLUS WHAT
 * STANDS IN IT. The game publishes contents to the sim at the end of each conduit frame, so
 * mass and composition here lag the game's own grid by at most one frame; temperature is the
 * sim's own, and is as fresh as the last 200 ms update. Standing matter the conduit itself
 * cannot represent -- OniFramework's PipeMatterFacade side-car, condensate in a gas line and
 * headspace gas in a liquid one -- is mirrored into the sim per cell (SIM_ConduitTrappedSet)
 * and reported in the `trapped*` fields, separately: `totalMassKg`, the composition, both
 * temperatures and `pressurePa` are the conduit contents alone.
 *
 * `networkId` IS `UtilityNetwork.id` AND IS ONLY MEANINGFUL WITH ITS `generation`. Ids are
 * re-assigned every time a conduit is built, broken or removed; `generation` counts binds of
 * this conduit type, and two answers with different generations are about different
 * numberings.
 *
 * TWO TEMPERATURES, deliberately. `temperatureK` is the heat-capacity-weighted mean -- the
 * temperature the run's contents would settle at if mixed, and the one pressure is computed
 * at. `massWeightedTemperatureK` is what a managed walk weighting by mass reports
 * (PipeNetworkFacade.TryReadNetwork). They are equal for a run of one element.
 *
 * `pressurePa` is a GAS run's ideal-gas pressure over `volumeM3`, through the same two
 * functions SIM_ComputeGasPressure uses. It is 0.0f for a liquid run, whose pressure is its
 * headspace: `headspacePressurePa` -- the trapped gas over whatever `volumeM3` the liquid
 * (`liquidVolumeM3`, conduit contents plus trapped liquid) does not fill. That is the exact
 * construction of OniFramework's PipeMatterFacade.TryGetLiquidHeadspacePressurePa, including
 * its two density rules: conduit liquid with no `sim.liquid_density` counts as water-dense
 * (1000 kg/m^3), trapped liquid with none is not counted.
 *
 * COMPOSITION OVERFLOW IS COUNTED, NEVER SILENT, exactly as for rooms: slots fill in
 * first-encountered order and `speciesOverflow` counts distinct species that found none. A
 * vanilla conduit holds one element at a time, so only a hand-built mixture ever overflows.
 */
#define ONI_CONDUIT_NET_MAX_SPECIES 8

/* ONI_CONDUIT_POLICY_* -- what the sim does to a bound run of its own accord, set with
 * SIM_ConduitNetworkPolicy. A run with no policy is updated by Klei's arithmetic alone.
 *
 * MIX: axial heat exchange along the run. After each 200 ms update's conduit-to-building
 * exchange, every pair of conduits piped to each other (the neighbours the bind named) and both
 * holding something exchanges q = k (T_i - T_j) C_i C_j / (C_i + C_j), with k = `mixFraction`
 * in (0, ONI_CONDUIT_MIX_FRACTION_MAX] -- the fraction of a pair's gap one link closes per
 * update. All links are computed from the same starting temperatures. The run's contents
 * energy is conserved; no energy crosses to or from any building; and with at most four links
 * per conduit and k <= 0.25 every mixed temperature is a weighted average of the temperatures
 * around it, so nothing overshoots. It is deliberately NOT Stationeers' one temperature per
 * network: an ONI run can cross rooms with no device in between, and levelling such a run
 * whole defeats any gradient along it (sim/conduits.h, MixNetworks, says which rig that broke).
 * Packets still move under the game's own ConduitFlow, carrying their temperature with them.
 * Trapped matter is not mixed: its temperature belongs to the managed side-car.
 *
 * PHASE: pressure-driven phase change, decided by the sim once per 200 ms update after the mix.
 * Gas in a gas run condenses when the run's pressure puts the gas's dew point above its
 * temperature, and liquid standing in a gas run evaporates back when it puts the dew point
 * below it; liquid in a liquid run boils when the headspace pressure puts its boiling point
 * below it, and headspace gas in a liquid run condenses back. Both directions of each run are
 * decided here, against one boundary, so no other decider has to close the loop at a different
 * cadence. The boundary is `sim.phase_curve`, the amount is phase::ComputePhaseChangeStep
 * at the policy's rate. THE SIM ONLY DECIDES: conduit contents are ConduitFlow's and trapped
 * matter is the side-car's, both managed, so each decision is published as an
 * OniConduitPhaseProposal (SIM_ConduitPhaseProposals) for the managed side to apply.
 *
 * CONVECTION: Stationeers' pipe-to-world convection shape on ONI's own coefficient. Each
 * conduit's exchange with its building -- Klei's k * dT * contact area * rate, k the mean (or,
 * insulated, the minimum) of the contents' and the pipe's conductivities, which is what keeps
 * pipe material meaningful -- is multiplied by two HeatExchangeRatio()s, Stationeers'
 * AtmosphereHelper.CalculateConvection:
 *   pipe side: the run's max(clamp01(P / 1 atm), clamp01(liquid volume / run volume / 0.001)),
 *              P the run's gas pressure (a liquid run's headspace pressure);
 *   cell side: the same ratio of the conduit's own cell, read from the frame the game is
 *              holding: a gas cell by its pressure over the sim's cell volume, a liquid cell by
 *              its liquid volume, vacuum 0. A SOLID cell is 1, not Stationeers' 0: a pipe
 *              embedded in a tile touches matter, and ONI's conduction owns that exchange.
 * So a vacuum on either side stops the exchange, a low-pressure gas side slows it in
 * proportion to its pressure, and a trace of liquid saturates it. The equilibrium limits and
 * the building's energy message are Klei's, unchanged. Before the game holds any frame (the
 * first update after a Start, an Alloc or a Load) the cell side cannot be read and the update
 * is Klei's alone. Takes no parameters. */
#define ONI_CONDUIT_POLICY_MIX 0x1
#define ONI_CONDUIT_POLICY_PHASE 0x2
#define ONI_CONDUIT_POLICY_CONVECTION 0x4
/* The MIX coupling's ceiling (see MIX above), and how many neighbour handles
 * SIM_ConduitNetworkBind takes per conduit: left, right, up, down. */
#define ONI_CONDUIT_MIX_FRACTION_MAX 0.25f
#define ONI_CONDUIT_NEIGHBOUR_SLOTS 4

typedef struct OniConduitNetworkAggregate {
  int32_t  networkId;          /* UtilityNetwork.id, or -1 when refused */
  int32_t  conduitType;        /* the game's ConduitType: 1 gas, 2 liquid */
  int32_t  generation;         /* binds of this conduit type so far -- the id's numbering */
  int32_t  policyFlags;        /* ONI_CONDUIT_POLICY_* currently in force on this run */
  int32_t  conduitCount;       /* bound conduits in the run */
  int32_t  filledCount;        /* of those, conduits holding any mass */
  int32_t  speciesCount;       /* entries written into species[] / massBySpeciesKg[] */
  int32_t  speciesOverflow;    /* distinct species with no slot; their mass is still counted */
  float    mixFraction;        /* the MIX policy's coupling k, 0.0f without it */
  float    volumeM3;           /* conduitCount * the per-conduit volume the bind declared */
  float    totalMassKg;
  float    totalMoles;         /* sum of mass / molecular mass, sim.molecular_mass-corrected */
  float    heatCapacityKJPerK; /* sum of every conduit's contents heat capacity */
  float    temperatureK;       /* heat-capacity-weighted; 0.0f when the run holds nothing */
  float    massWeightedTemperatureK;
  float    pressurePa;         /* gas runs only, see above; 0.0f otherwise */
  uint16_t species[ONI_CONDUIT_NET_MAX_SPECIES];     /* ElementTable indices */
  float    massBySpeciesKg[ONI_CONDUIT_NET_MAX_SPECIES];
  float    trappedGasKg;       /* standing matter mirrored from the side-car, by phase */
  float    trappedLiquidKg;
  float    trappedSolidKg;
  float    liquidVolumeM3;     /* conduit liquid + trapped liquid, see the density rules above */
  float    headspacePressurePa; /* liquid runs only; 0.0f with no trapped gas or no headspace */
  float    phaseRatePerSecond;  /* the PHASE policy's rate, 0.0f without it */
  int32_t  linkCount;          /* pipe-to-pipe links inside the run, each counted once: the
                                * edges MIX exchanges along. A run is connected, so a correct
                                * bind has at least conduitCount - 1 (exactly that when the run
                                * has no loop); fewer means the neighbours were bound wrong. */
} OniConduitNetworkAggregate;
ONI_SIM_STATIC_ASSERT(sizeof(OniConduitNetworkAggregate) == 140, "conduit aggregate drift");

/* ONE PHASE-CHANGE DECISION, for the managed side to carry out. `kind` names which matter moves
 * where:
 *   CONDENSE_CONTENTS  a gas run: `convertedMassKg` of the conduit's gas contents at `cell`
 *                      becomes `productElementIdx` liquid standing in that conduit
 *   BOIL_CONTENTS      a liquid run: the conduit's liquid contents become standing gas
 *   CONDENSE_HEADSPACE a liquid run: standing gas at `cell` becomes liquid conduit contents,
 *                      already clamped to the conduit's mass headroom
 *   EVAPORATE_TRAPPED  a gas run: liquid standing at `cell` becomes gas conduit contents, the
 *                      reverse of CONDENSE_CONTENTS, already clamped to the conduit's mass
 *                      headroom. Judged on the conduit's contents temperature when it has any
 *                      and on the standing liquid's own when it has none -- the same temperature
 *                      CONDENSE_CONTENTS judges, so one tile is never proposed both ways at once
 * `sourceMassKg` / `sourceTemperatureK` are the state the decision was made against, so an
 * applier can refuse one that has gone stale. `latentHeatJPerKg` is released on condensing and
 * absorbed on boiling or evaporating; where it is billed is the applier's decision. An applier
 * built before a kind existed should refuse it, which leaves that matter where it stands. */
#define ONI_CONDUIT_PHASE_CONDENSE_CONTENTS  0
#define ONI_CONDUIT_PHASE_BOIL_CONTENTS      1
#define ONI_CONDUIT_PHASE_CONDENSE_HEADSPACE 2
#define ONI_CONDUIT_PHASE_EVAPORATE_TRAPPED  3

typedef struct OniConduitPhaseProposal {
  int32_t  conduitType;
  int32_t  networkId;
  int32_t  cell;
  int32_t  kind;               /* ONI_CONDUIT_PHASE_* */
  uint16_t sourceElementIdx;
  uint16_t productElementIdx;
  float    sourceMassKg;
  float    sourceTemperatureK;
  float    convertedMassKg;
  float    boundaryK;          /* the pressure-adjusted transition temperature it crossed */
  float    pressurePa;         /* the run (or headspace) pressure that boundary came from */
  float    latentHeatJPerKg;
} OniConduitPhaseProposal;
ONI_SIM_STATIC_ASSERT(sizeof(OniConduitPhaseProposal) == 44, "conduit phase proposal drift");

#pragma pack(pop)

/* -------------------------------------------------------------------------------------
 * EXPORTS -- WHO THIS DLL IS.
 */

/* "MAJOR.MINOR.REV+SHA", with a trailing '*' for a build made from a dirty tree. The stock
 * SimDLL does not export this symbol, and that is the whole detection mechanism: call it,
 * catch the missing-entry-point error, and you have a stock DLL. Callable at any time,
 * including before SIM_Initialize -- it reads a compile-time literal, touches no sim state
 * and cannot be made to wait on a frame. The pointer is static storage owned by the DLL;
 * never free it. */
ONI_SIM_API const char* SIM_Version();

/* -------------------------------------------------------------------------------------
 * EXPORTS -- THE PUBLISHED FRAME ORDERING AND MESSAGE SURFACE.
 *
 * These four are the only exports here that read NO world state, take no worker barrier and
 * are callable BEFORE the world is allocated: both tables are compile-time. A mod checking at
 * load time that the DLL it got still orders its frame, and still delivers its messages, the
 * way it was built to expect can do so at the one moment it can still decline to load.
 */
ONI_SIM_API int32_t SIM_ExtPhaseCount();
ONI_SIM_API int32_t SIM_ExtPhaseDescribe(int32_t phaseIdx, OniExtPhaseDesc* out);
ONI_SIM_API int32_t SIM_ExtMessageCount();
ONI_SIM_API int32_t SIM_ExtMessageDescribe(int32_t msgIdx, OniExtMessageDesc* out);

/* -------------------------------------------------------------------------------------
 * EXPORTS -- THE TUNABLE TABLE (ONI_MSG_SET_TUNABLE writes it).
 *
 * Like the four above, callable before SIM_Initialize and before a world exists, and none of them
 * waits on a frame: the table is only ever written on the thread that sends the message, while
 * the sim is idle. Describe and Get return 1 and fill `out`, or 0 and leave it untouched for an
 * id outside [0, Count) or a null pointer.
 */
ONI_SIM_API int32_t SIM_ExtTunableCount();
ONI_SIM_API int32_t SIM_ExtTunableDescribe(int32_t tunableId, OniExtTunableDesc* out);
/* The id of the row with this exact name, or -1. */
ONI_SIM_API int32_t SIM_ExtTunableIndex(const char* name);
/* The row's LIVE value, as bits. */
ONI_SIM_API int32_t SIM_ExtTunableGet(int32_t tunableId, uint64_t* outBits);

/* -------------------------------------------------------------------------------------
 * EXPORTS -- REGISTRY 1: A NAMED ARRAY OVER THE GRID.
 */

/* The index a registered name has, or -1. Stable for the life of the world. */
ONI_SIM_API int32_t SIM_ExtCellPropertyIndex(const char* name);
ONI_SIM_API int32_t SIM_ExtCellPropertyCount();
ONI_SIM_API int32_t SIM_ExtCellPropertyDescribe(int32_t propertyIdx,
                                                OniExtCellPropertyDesc* out);
/* One component of one cell, as raw bits to reinterpret per the registered type. Returns 1
 * on success, 0 on any refusal, and leaves *outBits alone when it refuses. */
ONI_SIM_API int32_t SIM_ExtReadCellProperty(int32_t cellIdx, int32_t propertyIdx,
                                            int32_t component, uint32_t* outBits);
/* The properties registered as ONI_PERSIST_REHYDRATED that nobody has rehydrated since the
 * last load. A non-empty list is a mod that registered a property and then failed to refill
 * it -- a claim the registry checks rather than trusts. Returns the TOTAL, which may exceed
 * `max`; pass a null buffer with max 0 to size it. */
ONI_SIM_API int32_t SIM_ExtOutstandingRehydration(int32_t* out, int32_t max);
/* Layer C: how much of one component of a liquid-carried property left the grid in a record no
 * subscriber took. Returns 1 on success. */
ONI_SIM_API int32_t SIM_ExtLiquidPayloadUnreported(int32_t propertyIdx, int32_t component,
                                                   double* out);
/* The per-frame read window opened by ONI_MSG_PUBLISH_CELL_PROPERTY. `frame` is the pointer
 * the sim returned for THIS tick; a null frame, or one that is not a live publication,
 * returns null with *count = 0 -- a refusal, not an empty table, because a caller that
 * cannot tell those apart will bind garbage. */
ONI_SIM_API const OniExtPublishedProperty* SIM_ExtPublishedProperties(const OniSimFrame* frame,
                                                                     int32_t* count);

/* -------------------------------------------------------------------------------------
 * EXPORTS -- REGISTRY 2: A NAMED PROPERTY OF A SUBSTANCE.
 *
 * Keyed by SimHashes id, not by cell and not by element-table index. Sparse by construction:
 * an attribute holds values only for the elements somebody pushed one for.
 */
ONI_SIM_API int32_t SIM_ExtElementAttributeIndex(const char* name);
ONI_SIM_API int32_t SIM_ExtElementAttributeCount();
ONI_SIM_API int32_t SIM_ExtElementAttributeDescribe(int32_t attrIdx,
                                                    OniExtElementAttributeDesc* out);
ONI_SIM_API int32_t SIM_ExtElementAttribute(int32_t attrIdx, int32_t idHash,
                                            int32_t component, uint32_t* outBits);
/* The SimHashes ids this attribute holds a value for. Returns the TOTAL, which may exceed
 * `max`, so it sizes a buffer rather than reporting a copy; a null buffer with max 0 is the
 * sizing call. */
ONI_SIM_API int32_t SIM_ExtElementAttributeKeys(int32_t attrIdx, int32_t* out, int32_t max);

/* -------------------------------------------------------------------------------------
 * EXPORTS -- THE FIELD SOLVER. Discovery only: registration is a message
 * (`kRegisterField` / `kSetFieldSource`), and reading a field's values is reading the
 * per-cell property it lives in, which registry 1's exports above already do.
 *
 * Like the phase and message pairs, `SIM_ExtFieldCount` reads no world state and takes no
 * worker barrier, so it is callable before `SIM_AllocateCells`. A refusal LEAVES `out`
 * UNTOUCHED and returns 0, so a caller that ignores the return value does not find a
 * plausible descriptor sitting in it.
 */
ONI_SIM_API int32_t SIM_ExtFieldCount();
/* By the name of the PROPERTY the field lives in -- a field has no name of its own, because
 * inventing a second name space for the same object is how two names become one store. */
ONI_SIM_API int32_t SIM_ExtFieldIndex(const char* propertyName);
ONI_SIM_API int32_t SIM_ExtFieldDescribe(int32_t fieldIdx, OniExtFieldDesc* out);

/* -------------------------------------------------------------------------------------
 * EXPORTS -- THE PLANETARY LATENT ACCUMULATOR. Flagship 2 / Mod 3.
 *
 * The signed energy, in JOULES, that phase change in one world's sky-exposed cells has put
 * into that world's atmosphere (positive: a gas condensed) or taken out of it (negative: a
 * liquid boiled). Stationeers keeps the same store as
 * `PlanetaryAtmosphereSimulation.LatentEnergyOffset` and reads it back as a temperature
 * offset that is summed into ambient, which makes it the thermostat on runaway condensation:
 * condensing warms the planet, and a warmer planet stops condensing.
 *
 * THE DIVISOR IS NOT HERE, ON PURPOSE. Stationeers divides by `GetHeatCapacity()`, the sum of
 * four planetary reservoirs it simulates and we defer to Mod 4; ours is therefore a configured
 * constant, and a configured constant is a balance decision rather than physics. It lives in
 * `OniFramework.PlanetaryEnvironment`, next to the one the sibling accumulator (the colony's
 * radiated waste heat) already uses. The DLL supplies the joules and no policy.
 *
 * Only gas <-> liquid transitions in cells with a path to the sky are counted, weighted by the
 * same lit fraction the atmosphere boundary uses as its coupling, and only on a world that has
 * declared a surface atmosphere. A freeze or a melt contributes nothing: `sim.phase_curve`
 * carries an enthalpy of VAPORIZATION, which is the wrong number for fusion.
 *
 * CUMULATIVE SINCE ALLOCATE and not zeroed by reading, like both ledgers -- sample, wait and
 * subtract. Nothing serialises it, so it starts a load at zero, exactly as the environment
 * record it belongs to does. Returns 1 and fills `out`; 0 for a null pointer or no world. */
ONI_SIM_API int32_t SIM_ExtWorldLatentEnergy(int32_t worldIndex, double* out);

/* Mod 3's sun path. The direction ONIJ last left for `worldIndex` -- or the fallback, when that
 * world has none of its own -- read out of the DLL rather than out of the caller's memory, so a
 * gate can tell a sun the sim has from one a mod meant to send. Returns 1 and fills both
 * components (normalised); 0 for a null pointer, no sim, or no sun. */
ONI_SIM_API int32_t SIM_ExtWorldSun(int32_t worldIndex, float* dirX, float* dirY);

/* The direct-beam exposure field: one byte per GAME cell, in the layout and scale of the
 * ExposedToSunlight texture (exposure * 255, truncated), zero on every world without a sun.
 * Copied into `out` rather than pointed at, because the worker rewrites it every frame. Returns
 * the cell count and copies only when `capacity` is at least that, so a call with a null `out`
 * sizes the buffer; -1 with no sim. */
ONI_SIM_API int32_t SIM_ExtCopySunBeam(uint8_t* out, int32_t capacity);

/* -------------------------------------------------------------------------------------
 * EXPORTS -- REGISTRY 3: A NAMED PER-FRAME EVENT LIST THE SIM PRODUCES.
 *
 * Streams are DECLARED by the DLL, never by a message: the set never changes after
 * SIM_Initialize returns, which is what lets an index be resolved once instead of per tick.
 * A mod SUBSCRIBES to one it wants.
 */
ONI_SIM_API int32_t SIM_ExtEventStreamIndex(const char* name);
ONI_SIM_API int32_t SIM_ExtEventStreamCount();
ONI_SIM_API int32_t SIM_ExtEventStreamDescribe(int32_t streamIdx, OniExtEventStreamDesc* out);
ONI_SIM_API const OniExtPublishedStream* SIM_ExtPublishedEvents(const OniSimFrame* frame,
                                                                int32_t* count);

/* -------------------------------------------------------------------------------------
 * EXPORTS -- PHYSICS THE SIM WILL COMPUTE FOR YOU.
 *
 * Pure functions of their arguments: no world, no sim state, no barrier, safe before
 * SIM_Initialize and safe from any thread. They exist so that managed code deciding what to
 * ask the sim for uses the sim's OWN arithmetic to decide it, rather than a second
 * implementation that agrees until it does not.
 */
ONI_SIM_API float SIM_ComputeGasPressure(const uint16_t* species, const float* massKg,
                                         int32_t count, float temperatureK, float volumeM3);
ONI_SIM_API float SIM_CalculateCombinedTemperature(float massA, float tempA, float massB,
                                                   float tempB);
ONI_SIM_API float SIM_EqualizeSingleSpeciesMass(float molarMass, float massA, float tempA,
                                                float volumeA, float massB, float tempB,
                                                float volumeB, float rate);
ONI_SIM_API float SIM_AdiabaticFillTemperature(float gammaMix, float massA, float tempA,
                                               float massB, float tempB);
ONI_SIM_API float SIM_LiquidVolumeFromMass(float massKg, float densityKgM3);
ONI_SIM_API float SIM_EqualizeLiquidVolumeMass(float densityKgM3, float massA,
                                               float capacityA_m3, float massB,
                                               float capacityB_m3, float rate);
/* One step of a phase change: how much mass converts, and what the remainder's temperature
 * becomes once the latent heat has been paid. Both outputs are written on every call. */
ONI_SIM_API void SIM_ComputePhaseChangeStep(float massKg, float temperatureK, float thresholdK,
                                            float latentHeatJPerKg, float specificHeatCapacity,
                                            float dtSeconds, float conversionRatePerSecond,
                                            float minRemainderKg, float* outConvertedMassKg,
                                            float* outRemainingTemperatureK);

/* -------------------------------------------------------------------------------------
 * EXPORTS -- READING THE MULTI-GAS LAYER.
 *
 * A cell that mixing does not own answers as vanilla does, so a caller does not have to know
 * which is which before it asks.
 */
ONI_SIM_API float    SIM_GasPressure(int32_t cellIdx);
ONI_SIM_API uint16_t SIM_GasDominantElement(int32_t cellIdx, float* outMass);
/* Up to `maxSlots` species and their masses. Returns the number written. */
ONI_SIM_API int32_t  SIM_GasComposition(int32_t cellIdx, uint16_t* outSpecies, float* outMass,
                                        int32_t maxSlots);
/* The same question for many cells under ONE worker barrier. N per-cell reads are N chances
 * to block on an in-flight frame; one bulk read takes one. All four out-buffers are filled
 * for all `count` cells, or none is. */
ONI_SIM_API int32_t  SIM_GasMassBatchQuery(const int32_t* cellIndices, int32_t count,
                                           uint8_t* outOwned, uint16_t* outDominant,
                                           float* outMass);
/* Non-zero if the mixture layer owns the room this cell belongs to. */
ONI_SIM_API int32_t  SIM_DebugRoomOwned(int32_t cellIdx);
/* WHICH room, rather than whether it is promoted: the id `SIM_DebugRoomOwned` collapses away.
 * -1 when there is no room -- the cell is solid, out of range, or the mixture layer has never
 * been activated in this world, since the room graph is built for mixing and does not exist
 * before it.
 *
 * IT IS A SEPARATE EXPORT RATHER THAN A WIDER RETURN FROM SIM_DebugRoomOwned, and the reason
 * is not caution: that export's -1 / 0 / 1 already spends the value 0 on "vanilla-owned", so
 * returning an id through it would make room 0 promoted indistinguishable from any room not
 * promoted. There is no widening of it that keeps both answers.
 *
 * Cheap enough to call per cell -- one barrier and one array read -- which is what makes it
 * the right way to GROUP cells before asking SIM_RoomAggregate once per distinct room. */
ONI_SIM_API int32_t  SIM_RoomId(int32_t cellIdx);
/* The whole room a cell belongs to, summed in the sim: composition, mass in both layers, mean
 * pressure and mass-weighted temperature, under ONE worker barrier and with no allocation on
 * either side. Returns non-zero when `out` was filled.
 *
 * On any failure -- null `out`, no world, no mixture layer, an invalid cell, a cell in no room
 * -- it returns 0 AND writes a zeroed struct with `roomId` = -1, so a caller that ignores the
 * return value cannot read a plausible room out of it.
 *
 * COST IS O(cells in the room), walked natively with no per-cell crossing, and it is not
 * cached: ask once per room per tick and group with SIM_RoomId rather than calling it per
 * cell. A room is summed at the moment you ask, so two calls inside one frame agree. */
ONI_SIM_API int32_t  SIM_RoomAggregate(int32_t cellIdx, OniRoomAggregate* out);
/* Casts `count` rays, one per query, under ONE worker barrier, and writes one result per query
 * -- including for an invalid one, see OniRaycastQuery. Returns the number of VALID queries
 * answered, or -1 (writing nothing) for a null pointer, a negative count or no world. Cost is
 * O(length) per ray, walked natively; batch the rays a caller needs in a tick rather than
 * crossing once per ray. */
ONI_SIM_API int32_t  SIM_QueryRaycast(const OniRaycastQuery* queries, OniRaycastResult* results,
                                      int32_t count);
/* ---- Conduit runs. All three run on the GAME thread, like Klei's own
 * ConduitTemperatureManager_* exports, and none waits on the sim worker: conduit state belongs
 * to the game thread (the worker does not touch it). Do not call them from a ConduitFlow job.
 *
 * Replaces every binding of `conduitType` (1 gas, 2 liquid): entry i puts temperature handle
 * `handles[i]` -- the value ConduitTemperatureManager_Add returned -- in run `networkIds[i]`
 * at game cell `cells[i]`, piped to the conduits whose temperature handles are
 * `neighbourHandles[4i .. 4i+3]` (ONI_CONDUIT_NEIGHBOUR_SLOTS per conduit: left, right, up,
 * down, -1 for none -- `ConduitFlow.SOAInfo.GetConduitConnections` mapped through the same
 * handle table). A neighbour counts only while both ends are bound to the same run; the MIX
 * policy is the only thing that reads them. Call it after every `ConduitFlow.onConduitsRebuilt`,
 * because the game re-allocates every temperature handle of a type on every rebuild. Every
 * policy of the type is dropped with the old numbering. `maxMassPerConduitKg` is the game's
 * per-conduit capacity (ConduitFlow.MaxMass), which bounds a headspace condensation. Returns
 * the number of handles bound (a stale or removed handle, or an id outside [0, 2^20), is
 * skipped), or -1 with nothing changed for an unknown type, a negative count, a null array
 * with a non-zero count, or a volume or capacity that is not a positive finite number. */
ONI_SIM_API int32_t  SIM_ConduitNetworkBind(int32_t conduitType, const int32_t* handles,
                                            const int32_t* cells, const int32_t* networkIds,
                                            const int32_t* neighbourHandles, int32_t count,
                                            float volumePerConduitM3,
                                            float maxMassPerConduitKg);
/* Sets ONI_CONDUIT_POLICY_* flags on one run of the current bind; 0 clears them. CONVECTION
 * takes no parameter. MIX takes
 * `mixFraction` in (0, ONI_CONDUIT_MIX_FRACTION_MAX]; PHASE takes `phaseRatePerSecond` > 0 (Stationeers' shape is 0.1)
 * and `phaseMinRemainderKg` >= 0, below which a conversion takes the whole source rather than
 * leaving dust. Parameters of a flag that is not set are ignored. Returns 1 when applied and 0,
 * changing nothing, for an unknown type, a run outside the current bind, a flag bit this DLL
 * does not know, or a parameter out of range for a flag that is set. */
ONI_SIM_API int32_t  SIM_ConduitNetworkPolicy(int32_t conduitType, int32_t networkId,
                                              int32_t flags, float mixFraction,
                                              float phaseRatePerSecond,
                                              float phaseMinRemainderKg);
/* Mirrors one conduit tile's standing matter from the managed side-car, keyed by CELL (not by
 * handle, so it survives every rebind): `elementIdx` is an ElementTable index, and a mass <= 0
 * removes the entry. One entry per cell per type, replaced outright. Returns 1, or 0 with
 * nothing changed for an unknown type, a negative cell, an element index outside the table or a
 * non-finite mass or temperature. Trapped matter at a cell with no bound conduit is kept and
 * simply counted nowhere until a bind puts a conduit there. */
ONI_SIM_API int32_t  SIM_ConduitTrappedSet(int32_t conduitType, int32_t cell, int32_t elementIdx,
                                           float massKg, float temperatureK);
/* Drops every mirrored entry of one type, for a caller about to re-push its whole store.
 * Returns the number of entries dropped, or -1 for an unknown type. */
ONI_SIM_API int32_t  SIM_ConduitTrappedClear(int32_t conduitType);
/* The phase-change decisions of the most recent ConduitTemperatureManager_Update, which
 * replaces them wholesale -- an update with dt 0 or no building temperatures leaves none.
 * Copies up to `capacity` into `out` and returns the TOTAL, which may exceed `capacity`; a
 * null `out` with capacity 0 is the sizing call. Read them after each update: they are
 * decisions against the state that update saw, and the next update re-decides from scratch. */
ONI_SIM_API int32_t  SIM_ConduitPhaseProposals(OniConduitPhaseProposal* out, int32_t capacity);
/* The whole of one run. Writes min(outSize, sizeof(OniConduitNetworkAggregate)) bytes and
 * returns 1 when filled; returns 0 for a null `out`, an `outSize` smaller than 4, an unknown
 * type or a run outside the current bind, and still writes a zeroed prefix with `networkId`
 * -1. `outSize` exists so the struct can grow at its end without breaking a caller built
 * against this one. Sums are rebuilt at most once per change to the conduit list, for every
 * run at once, so asking about each run once per tick costs one walk. */
ONI_SIM_API int32_t  SIM_ConduitNetworkAggregate(int32_t conduitType, int32_t networkId,
                                                 OniConduitNetworkAggregate* out,
                                                 int32_t outSize);
ONI_SIM_API float    SIM_DebugGasMass(int32_t cellIdx, int32_t speciesIdx);

/* -------------------------------------------------------------------------------------
 * EXPORTS -- STATE READ-BACK, FOR TOOLING AND FOR CHECKPOINTS.
 *
 * These are the read halves of the checkpoint messages: what a replay harness dumps before a
 * run and sends back to resume one bit-identically. They read live world state, so unlike the
 * table exports above they take the worker barrier and need a world.
 *
 * Every one that fills a buffer returns the TOTAL it would have written, which may exceed
 * `capacity`, and a null buffer with capacity 0 is the sizing call.
 */
ONI_SIM_API uint32_t SIM_DebugRandomState();
ONI_SIM_API bool     SIM_DebugSchedulingState(OniSetSchedulingStateMessage* out);
ONI_SIM_API int32_t  SIM_DebugStableTicks(uint8_t* out, int32_t capacity);
ONI_SIM_API int32_t  SIM_DebugGasSleeping(uint8_t* out, int32_t capacity);
ONI_SIM_API int32_t  SIM_DebugDiseaseGrowth(float* accum, uint8_t* infest, int32_t capacity);
ONI_SIM_API int32_t  SIM_DebugCellRadiation(float* out, int32_t capacity);
/* The whole ONI_MSG_SET_VISIBILITY_STATE payload, header included, ready to send back. */
ONI_SIM_API int32_t  SIM_DebugVisibilityState(uint8_t* out, int32_t capacity);
ONI_SIM_API int32_t  SIM_DebugRegistryState(uint8_t* out, int32_t capacity);
/* The registered extension arrays as one opaque blob, to be handed back verbatim by
 * ONI_MSG_SET_EXT_CELL_STATE. The `All` form carries every class of registered array; the
 * short form predates it and carries fewer. */
ONI_SIM_API int32_t  SIM_DebugExtCellState(uint8_t* out, int32_t capacity);
ONI_SIM_API int32_t  SIM_DebugExtCellStateAll(uint8_t* out, int32_t capacity);
/* Layer C3's census: ONI_EFFERVESCENCE_CENSUS_FIELDS doubles, cumulative since the sim was
 * initialised -- passes, then columns, fizzing cells and kilograms released by what capped each
 * column (gas, a gap weighed by its gas neighbours, genuine vacuum, confined). */
ONI_SIM_API int32_t  SIM_DebugEffervescenceCensus(double* out, int32_t capacity);

/* THE FLOW ACCUMULATOR: the cells the sim moved mass through during the frame that just
 * finished -- game cell index, signed kilograms in x and y, and the element the flow texture's
 * own gate compares against. The only per-cell field in the sim that records a TRANSFER
 * rather than a state.
 *
 * IT RETURNS THE TOUCHED LIST RATHER THAN TAKING CELL INDICES, and that is the shape rather
 * than a convenience: the sim already records every cell it writes, so the sparse list IS the
 * answer -- a few hundred entries where an index-batch call would make a caller ask about
 * hundreds of thousands of cells to find them. A CELL ABSENT FROM THE LIST HAS ZERO FLOW;
 * that is the whole meaning of its absence. All four buffers are filled or none is. */
ONI_SIM_API int32_t  SIM_DebugCellFlow(int32_t* outCells, float* outX, float* outY,
                                       uint16_t* outElement, int32_t max);

/* -------------------------------------------------------------------------------------
 * EXPORTS -- THE LIVE PER-KERNEL PROFILE.
 *
 * Where a frame went, in a colony somebody actually built, as data. The offline harness
 * (`driver/bench`) times the same kernels on scenarios somebody wrote and never runs the
 * game's own frame; the DLL's backtick-key report prints the same table as English into
 * `Player.log`. This is the third reader and the only one a program can use.
 *
 * READING DOES NOT ZERO and arming an already-armed profiler is a no-op: the numbers are
 * cumulative since the last arm, so a live poller samples, waits, samples again and
 * subtracts. Anything else would let two readers silently destroy each other's window.
 *
 * Callable with no world: the profiler's state lives outside the sim on purpose, so that a
 * measurement survives a load, an allocate and a shutdown.
 */

/* Arms (non-zero) or disarms (0) the millisecond half; returns the PREVIOUS setting so a
 * caller can restore it. Arming zeroes the timings AND the census together. Disarming does
 * NOT write the report the backtick key writes -- a caller reading this table already has
 * every number that report would print. */
ONI_SIM_API int32_t SIM_DebugSetProfiler(int32_t enabled);
/* Returns 1 and fills `out`, or 0 and leaves it untouched for a null pointer. */
ONI_SIM_API int32_t SIM_DebugProfileSummary(OniProfileSummary* out);
/* Returns the TOTAL number of slots, which may exceed `max`; a null buffer with `max` 0 is
 * the sizing call. */
ONI_SIM_API int32_t SIM_DebugProfile(OniProfileSlot* out, int32_t max);

/* -------------------------------------------------------------------------------------
 * EXPORTS -- THE ENERGY LEDGER.
 *
 * Per-call-site books: where every joule the sim added or removed came from. Off by default
 * because it costs a frame to keep. `SIM_DebugSetEnergyLedger(1)` switches it on and returns
 * the previous setting.
 */
ONI_SIM_API int SIM_DebugSetEnergyLedger(int enabled);
ONI_SIM_API int SIM_DebugEnergyLedger(double* out, int count);
/* The mass ledger: the same question for matter rather than energy. */
ONI_SIM_API int SIM_DebugLedger(double* out, int count);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif  /* ONI_SIM_EXT_API_H */
