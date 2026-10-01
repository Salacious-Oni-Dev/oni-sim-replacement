#pragma once
// The sim's tunable numbers, generated from `tunables.def` (the only place any of them is
// written down). `kSetTunable` in abi/sim_abi_ext.h is the contract for changing them.
//
// THE ONE RULE, and the reason for it. The live table is one global, `g_tunables`. A kernel
// takes a copy of it on entry:
//
//     const Tunables tun = g_tunables;
//
// and every loop, and every per-cell helper the kernel calls (which takes `const Tunables&`),
// reads that copy -- never the global. The grid is `float*`, and under strict aliasing a store
// through a `float*` may, as far as GCC can tell, change any global `float`, so a loop that read
// `g_tunables` would reload it after every store. A local copy whose address never leaves the
// kernel cannot alias anything, so its fields live in registers exactly as the old `constexpr`
// immediates did.
//
// What may read `g_tunables` directly, because none of it is a per-cell sweep: a kernel's entry;
// a message handler or an export; a per-building or per-emitter helper; a per-frame setup; and
// a branch that only a registered element attribute can reach (the condensation and boiling
// rules), which no vanilla world ever enters. A helper with callers of both kinds takes
// `const Tunables& tun = g_tunables`, so a kernel passes its copy and everyone else gets the
// live table.
//
// The table is written only by `kSetTunable` (abi/sim_abi_ext.h), which is an IMMEDIATE message:
// `SIM_HandleMessage` has already waited for the worker to finish its frame before the handler
// runs, and the conduit kernel runs on the game thread the message arrives on. So nothing is
// stepping while a value changes, and a value never changes inside a substep.
//
// A value in a register gives the same bits as the immediate it replaces: the build has no
// fast-math and `-ffp-contract=off`, so GCC neither reassociates nor fuses around a constant.
// The one place a constant's *value* was folded at compile time is a derived constant; see
// `CellTransferScale` below.

#include <cstdint>

// The build-time override (sim/build.sh, `ONI_TUNABLES_CFG`): a generated header whose
// `ONI_TUNABLE_OVERRIDES(X)` lists `X(member, value)` pairs, applied to the build's defaults on
// top of the rows below. Absent in every ordinary build, so the ordinary build is unchanged.
#ifdef ONI_TUNABLES_OVERRIDE_HEADER
#include ONI_TUNABLES_OVERRIDE_HEADER
#endif

namespace oni_sim {

namespace tunables_detail {
template <typename T> struct Bits;
template <> struct Bits<float> {
  static constexpr float From(uint64_t b) {
    return __builtin_bit_cast(float, static_cast<uint32_t>(b));
  }
};
template <> struct Bits<double> {
  static constexpr double From(uint64_t b) { return __builtin_bit_cast(double, b); }
};
template <> struct Bits<int32_t> {
  static constexpr int32_t From(uint64_t b) { return static_cast<int32_t>(b); }
};
}  // namespace tunables_detail

// One ID per row, in row order: the id `kSetTunable` and the SIM_ExtTunable* exports take. The
// framework's `SimTunable` enum is generated from the same rows (tools/gen_tunables_cs.py).
enum class TunableId : int32_t {
#define TUNABLE(Name, field, type, bits, lo, hi, group, origin, doc) Name,
#define TUNABLE_AT(Name, field, index, type, bits, lo, hi, group, origin, doc) Name,
#include "tunables.def"
#undef TUNABLE
#undef TUNABLE_AT
  Count
};

inline constexpr int32_t kRadiationStencilSize = 25;

struct Tunables {
#define TUNABLE(Name, field, type, bits, lo, hi, group, origin, doc) \
  type field = tunables_detail::Bits<type>::From(bits);
#define TUNABLE_AT(Name, field, index, type, bits, lo, hi, group, origin, doc)
#include "tunables.def"
#undef TUNABLE
#undef TUNABLE_AT

  float radiation_stencil[kRadiationStencilSize] = {};

  constexpr Tunables() {
#define TUNABLE(Name, field, type, bits, lo, hi, group, origin, doc)
#define TUNABLE_AT(Name, field, index, type, bits, lo, hi, group, origin, doc) \
  field[index] = tunables_detail::Bits<type>::From(bits);
#include "tunables.def"
#undef TUNABLE
#undef TUNABLE_AT
  }
};

// The rows exactly as `tunables.def` writes them: Klei's value for every KLEI row, whatever the
// build was configured with. `CellTransferScale` compares against this, not against the build's
// defaults, because the folded literal it returns is Klei's and belongs to Klei's rate only.
inline constexpr Tunables kTunableStock{};

// The build's defaults: the stock rows with the build-time override applied. What `id = -1`
// restores, and what the table starts as. Equal to `kTunableStock` in every ordinary build.
inline constexpr Tunables MakeTunableDefaults() {
  Tunables t{};
#ifdef ONI_TUNABLE_OVERRIDES
#define ONI_TUNABLE_OVERRIDE_APPLY(member, value) t.member = value;
  ONI_TUNABLE_OVERRIDES(ONI_TUNABLE_OVERRIDE_APPLY)
#undef ONI_TUNABLE_OVERRIDE_APPLY
#endif
  return t;
}
inline constexpr Tunables kTunableDefaults = MakeTunableDefaults();

// Every default lies inside its own bounds, and every array row names a slot that exists. This
// checks the BUILD's defaults, so an override outside a row's range fails the build.
#define TUNABLE(Name, field, type, bits, lo, hi, group, origin, doc)                       \
  static_assert(kTunableDefaults.field >= static_cast<type>(lo) &&                         \
                    kTunableDefaults.field <= static_cast<type>(hi),                       \
                "tunables.def: default of " #Name " is outside [min, max]");
#define TUNABLE_AT(Name, field, index, type, bits, lo, hi, group, origin, doc)             \
  static_assert((index) >= 0 && (index) < kRadiationStencilSize,                           \
                "tunables.def: " #Name " indexes past its array");                         \
  static_assert(kTunableDefaults.field[index] >= static_cast<type>(lo) &&                  \
                    kTunableDefaults.field[index] <= static_cast<type>(hi),                \
                "tunables.def: default of " #Name " is outside [min, max]");
#include "tunables.def"
#undef TUNABLE
#undef TUNABLE_AT

// The live table. Starts as the build's defaults; written only by `kSetTunable`; read by kernel
// entries (THE ONE RULE above). A plain global rather than a member of the sim's state, so it
// survives `SIM_Shutdown`, `SIM_Initialize` and every `Allocate`: a value is set once and holds until it is set again or reset with `id = -1`.
inline Tunables g_tunables = kTunableDefaults;

// ---- By-ID access, for the message, the exports and the tests -------------------------------
//
// A row's value crosses the ABI as 64 raw bits: a float or an int32 in the low 32 with the high
// 32 zero, a double in all 64. Bits rather than a value so a float crosses exactly.
enum TunableType : int32_t { kTunableF32 = 0, kTunableI32 = 1, kTunableF64 = 2 };

namespace tunables_detail {
template <typename T> struct TypeOf;
template <> struct TypeOf<float> { static constexpr TunableType kType = kTunableF32; };
template <> struct TypeOf<int32_t> { static constexpr TunableType kType = kTunableI32; };
template <> struct TypeOf<double> { static constexpr TunableType kType = kTunableF64; };
constexpr uint64_t ToBits(float v) { return __builtin_bit_cast(uint32_t, v); }
constexpr uint64_t ToBits(int32_t v) { return static_cast<uint32_t>(v); }
constexpr uint64_t ToBits(double v) { return __builtin_bit_cast(uint64_t, v); }
}  // namespace tunables_detail

constexpr int32_t kTunableCount = static_cast<int32_t>(TunableId::Count);

// Reads row `id` of `t` as bits. False for an id outside the table.
inline bool TunableGetBits(const Tunables& t, int32_t id, uint64_t* out) {
  switch (static_cast<TunableId>(id)) {
#define TUNABLE(Name, field, type, bits, lo, hi, group, origin, doc) \
  case TunableId::Name: *out = tunables_detail::ToBits(t.field); return true;
#define TUNABLE_AT(Name, field, index, type, bits, lo, hi, group, origin, doc) \
  case TunableId::Name: *out = tunables_detail::ToBits(t.field[index]); return true;
#include "tunables.def"
#undef TUNABLE
#undef TUNABLE_AT
    default: return false;
  }
}

// Writes row `id` of `t` from bits, with no validation (the message handler validates first).
inline bool TunableSetBits(Tunables& t, int32_t id, uint64_t bits) {
  switch (static_cast<TunableId>(id)) {
#define TUNABLE(Name, field, type, bits_, lo, hi, group, origin, doc) \
  case TunableId::Name: t.field = tunables_detail::Bits<type>::From(bits); return true;
#define TUNABLE_AT(Name, field, index, type, bits_, lo, hi, group, origin, doc) \
  case TunableId::Name: t.field[index] = tunables_detail::Bits<type>::From(bits); return true;
#include "tunables.def"
#undef TUNABLE
#undef TUNABLE_AT
    default: return false;
  }
}

// One row's metadata. `lo`/`hi` are the row's bounds as bits of the row's own type.
struct TunableInfo {
  const char* name;
  const char* group;
  const char* origin;
  const char* doc;
  TunableType type;
  uint64_t stock_bits;
  uint64_t lo_bits;
  uint64_t hi_bits;
};

inline constexpr TunableInfo kTunableInfo[] = {
#define TUNABLE(Name, field, type, bits, lo, hi, group, origin, doc)                         \
  {#Name, #group, #origin, doc, tunables_detail::TypeOf<type>::kType,                        \
   tunables_detail::ToBits(kTunableStock.field), tunables_detail::ToBits(static_cast<type>(lo)), \
   tunables_detail::ToBits(static_cast<type>(hi))},
#define TUNABLE_AT(Name, field, index, type, bits, lo, hi, group, origin, doc)               \
  {#Name, #group, #origin, doc, tunables_detail::TypeOf<type>::kType,                        \
   tunables_detail::ToBits(kTunableStock.field[index]),                                      \
   tunables_detail::ToBits(static_cast<type>(lo)), tunables_detail::ToBits(static_cast<type>(hi))},
#include "tunables.def"
#undef TUNABLE
#undef TUNABLE_AT
};
static_assert(sizeof(kTunableInfo) / sizeof(kTunableInfo[0]) == static_cast<size_t>(kTunableCount),
              "kTunableInfo and TunableId disagree -- tunables.def is the one definition");

// The published descriptor carries the name and the doc line in fixed buffers (OniExtTunableDesc,
// abi/sim_ext_api.h). A row that outgrows them fails here rather than arriving truncated.
#define TUNABLE(Name, field, type, bits, lo, hi, group, origin, doc)                         \
  static_assert(sizeof(#Name) <= 48 && sizeof(doc) <= 192 && sizeof(#group) <= 16 &&         \
                    sizeof(#origin) <= 8,                                                    \
                "tunables.def: " #Name "'s name, doc, group or origin outgrows its descriptor");
#define TUNABLE_AT(Name, field, index, type, bits, lo, hi, group, origin, doc)               \
  TUNABLE(Name, field, type, bits, lo, hi, group, origin, doc)
#include "tunables.def"
#undef TUNABLE
#undef TUNABLE_AT

// `kCellTransfer`, Klei's building-to-cell scale, is `BuildingTransferRate * (1 / 255^2)`, and
// Klei stores the FOLDED product (the literal; see buildings.h). Folded and
// recomputed agree today, but that is a property of these two numbers, not of the arithmetic. So while the
// parent is at its default this returns Klei's stored literal, and once anyone moves the parent
// it recomputes: bit-exact by construction at the default, and what anyone would expect after.
constexpr float CellTransferScale(const Tunables& t) {
  if (__builtin_bit_cast(uint32_t, t.building_transfer_rate) ==
      __builtin_bit_cast(uint32_t, kTunableStock.building_transfer_rate)) {
    return 7.689349956763181e-08f;
  }
  return t.building_transfer_rate * (1.0f / (255.0f * 255.0f));
}

}  // namespace oni_sim
