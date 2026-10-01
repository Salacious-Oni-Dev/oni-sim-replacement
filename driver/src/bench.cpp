// Per-kernel timing for the replacement sim.
//
// The scored scenarios elsewhere are 384 cells against a real asteroid's 98,304; this times
// the kernels at real scale, so tuning starts from numbers.
//
// It calls the kernels directly out of `sim/` rather than going through the DLL. That is
// deliberate: the kernels are header-only, the DLL boundary is one call per *frame* and
// would hide the per-substep split entirely, and timing them in-process means the sim's
// own sources stay untouched by this pass. The flags match `sim/build.sh`, so the codegen
// is the codegen that ships.
//
// Usage:
//   bench.exe --corpus <corpus.bin> [--scenario asteroid|granite|all]
//             [--width W --height H] [--ticks N] [--dt SECONDS]
//             [--regions full|clamped|quarter|cluster:N]
//             [--buildings off|corpus|N]
//             [--fields off|idle|decay|point|beam|element|all]
//             [--radiation N]

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "../../sim/buildings.h"
#include "../../sim/fields.h"
#include "../../sim/physics.h"
#include "../../sim/projection.h"
#include "../../sim/radiation.h"
#include "../../sim/textures.h"
#include "../../sim/world.h"
#include "goldens.h"
#include "harness.h"
#include "simcheck.h"

using namespace oni_sim;
using oni_bench::SeedBuilding;
using oni_bench::SeedWorld;
using oni_bench::Tables;

// ------------------------------------------------------------------ allocation counting
//
// A steady-state sim frame should allocate NOTHING. Every kernel works over arrays the world
// already owns and over `static thread_local` scratch that reaches its size on the first
// substep and is reused by `assign`/`resize` afterwards; the projection's event lists grow to
// their high-water mark and stay there. So "allocations per frame" is not a number to graph
// over time, it is a BINARY FACT to assert -- and asserting it is the only way to notice a
// `std::vector` that someone gives a fresh local home to inside a loop, which costs a malloc
// per substep and shows up in a timing as a couple of percent that everyone argues about.
//
// Counted by replacing global operator new, which is legal exactly once per program and is a
// property of this harness rather than of the sim: the shipped DLL has no such override and
// pays nothing. It counts every allocation on any thread; `bench` is single-threaded.
//
// It cannot see an allocator the CRT itself makes behind `printf`, so the count is only ever
// read across the tick loop, never across the report.
namespace {
size_t g_alloc_count = 0;
}  // namespace

void* operator new(size_t n) {
  ++g_alloc_count;
  void* p = malloc(n ? n : 1);
  if (p == nullptr) abort();
  return p;
}
void operator delete(void* p) noexcept { free(p); }
void operator delete(void* p, size_t) noexcept { free(p); }
void* operator new[](size_t n) { return operator new(n); }
void operator delete[](void* p) noexcept { free(p); }
void operator delete[](void* p, size_t) noexcept { free(p); }

namespace {

using Clock = std::chrono::steady_clock;

// std::chrono::steady_clock on mingw is QueryPerformanceCounter, which is sub-microsecond
// — fine for a kernel that is expected to land in the hundreds of microseconds, and the
// reason the per-substep samples are kept individually rather than summed. The interesting
// number is the *worst* substep, not the mean one: a mean that fits in a frame and a
// worst case that does not still hitches the game once every N frames.
struct Samples {
  explicit Samples(const char* n) : name(n) {}
  const char* name;
  std::vector<double> ms;

  // Reserved up front so this harness's own bookkeeping cannot land in the allocation count.
  // Without it a `push_back` per tick per bucket reallocates about six times per bucket over a
  // 50-tick run, nine buckets deep, and the first version of the steady-state assertion below
  // duly reported 66 allocations on the granite floor -- a world in which nothing happens at
  // all. A counter whose largest contributor is the counter's own scaffolding measures the
  // scaffolding.
  void Reserve(size_t n) { ms.reserve(n); }
  void Add(double v) { ms.push_back(v); }
  double Total() const {
    double s = 0;
    for (double v : ms) s += v;
    return s;
  }
  double Quantile(double q) const {
    if (ms.empty()) return 0;
    std::vector<double> s = ms;
    std::sort(s.begin(), s.end());
    size_t i = static_cast<size_t>(q * (s.size() - 1) + 0.5);
    return s[i];
  }
  double Min() const { return ms.empty() ? 0 : *std::min_element(ms.begin(), ms.end()); }
  double Max() const { return ms.empty() ? 0 : *std::max_element(ms.begin(), ms.end()); }
};

template <typename F>
double TimeIt(F&& f) {
  const auto a = Clock::now();
  f();
  const auto b = Clock::now();
  return std::chrono::duration<double, std::milli>(b - a).count();
}

// What is actually in the world. A timing without this is unreadable: a number from a
// world that is 99 % solid says nothing about a world with water in it, and "realistic"
// is a claim that has to be checkable.
struct Composition {
  size_t solid = 0, liquid = 0, gas = 0, vacuum = 0, massless = 0;
};

Composition Describe(const World& w, const ElementTable& t) {
  Composition c;
  for (size_t i = 0; i < w.GameCount(); ++i) {
    const PhaseEntry& e = w.Phase(w.Padded(i));
    if (e.mass <= 0.0f) ++c.massless;
    switch (t.Phase(e.element)) {
      case kStateSolid: ++c.solid; break;
      case kStateLiquid: ++c.liquid; break;
      case kStateGas: ++c.gas; break;
      default: ++c.vacuum; break;
    }
  }
  return c;
}

// ------------------------------------------------------------------- active regions

// The game sends region bounds per asteroid every frame and clamps maxY to
// HeightInCells - 1, so `clamped` is what a real single-asteroid map actually looks like
// and `full` is the state a sim that has never been told otherwise is in. `quarter` is not
// realistic; it is here to price the active-region mechanism itself, which has never been
// quantified.
std::vector<int32_t> Regions(const char* mode, int w, int h) {
  if (!strcmp(mode, "full")) return {0, 0, w, h};
  if (!strcmp(mode, "quarter")) return {w / 4, h / 4, w / 2, h / 2};
  // `cluster:N` is the shape Spaced Out actually sends. `Game.UnsafeSim200ms` builds one
  // `SimActiveRegion` per DISCOVERED WORLD, so the region count is the asteroid count: 1 in
  // the base game and 5-12 in a late cluster save. Every other mode here sends exactly one,
  // which is why a per-region cost that scales with the region COUNT rather than with the
  // region's area could never be seen by any suite in this repository.
  //
  // The strips are disjoint and side by side because the cluster grid is: each world owns a
  // rectangle at its own `WorldOffset` and no two overlap. Disjoint is the case that matters
  // -- a pair never crosses a region edge (see `World::SetActiveRegions`), so the per-cell
  // results are the same however many strips the same grid is cut into, and any cost that
  // moves with N is overhead by construction rather than work.
  if (!strncmp(mode, "cluster:", 8)) {
    int n = atoi(mode + 8);
    if (n < 1) n = 1;
    std::vector<int32_t> out;
    for (int i = 0; i < n; ++i) {
      const int x0 = static_cast<int>(static_cast<int64_t>(w) * i / n);
      const int x1 = static_cast<int>(static_cast<int64_t>(w) * (i + 1) / n);
      out.push_back(x0);
      out.push_back(0);
      out.push_back(x1);
      out.push_back(h - 1);
    }
    return out;
  }
  return {0, 0, w, h - 1};  // clamped: what the game sends
}

// ------------------------------------------------------------------- the run

struct Result {
  Samples conduction{"StepConduction"};
  Samples state{"StepStateChange"};
  Samples sublimation{"StepPostProcess"};
  Samples pressure{"StepGasPressure"};
  Samples displace{"StepGasDisplacement"};
  Samples flow{"StepFlow"};
  Samples liqdisp{"StepLiquidDisplacement"};
  Samples bldheat{"StepBuildingHeatExchange"};
  Samples bldbld{"StepBuildingToBuilding"};
  Samples zeromassless{"ZeroMasslessCells"};
  Samples project{"Project"};
  Samples textures{"FillPropertyTextures"};
  // `--sunlight` only. The direct beam at four suns, one per frame each: overhead (which must
  // equal the sky view byte for byte), the steep and shallow halves of the walk, and a low
  // western sun so both directions of the column walk are paid for.
  Samples beam90{"SunBeam 90 (=sky)"};
  Samples beam60{"SunBeam 60 E rows"};
  Samples beam30{"SunBeam 30 E cols"};
  Samples beam15{"SunBeam 15 W cols"};
  // `--radiation N` and `--fields <mode>` only. Both sit inside the substep and inside the
  // substep TOTAL, unlike the four sun-beam rows above: these are phases the DLL really runs
  // in this position (sim/simdll.cpp, `StepRadiationField` then the emitters then
  // `kPhaseFields`), so a frame that has them must be priced with them. That is also why the
  // rows only appear when the flags do -- a zero row in every historical table would say the
  // phase was free when the truth is that it never ran.
  Samples radfield{"StepRadiationField"};
  Samples rademit{"StepRadiationEmitters"};
  Samples fields{"StepFields"};
  Samples substep{"substep (13 kernels)"};
  Samples frame{"frame (substeps + projection)"};
  Composition before, after;
};

// `--verify`: recompute the projection and the property textures from scratch every frame
// and compare, byte for byte, against the incremental ones. Both are caches now — `Project`
// copies only the pass-through cells that were written, `FillPropertyTextures` only the
// cells whose element moved or which hold liquid — and `diffsim` cannot check either on its
// own: it does not compare four of the projection arrays at all, and the texture diffs it
// does report against Klei move run to run with Klei's frame pacing. The question a cache
// needs answered is not "does this match Klei", it is "does this match what the code would
// have written without the cache", and that needs no Klei in the loop.
//
// Roughly triples the frame, so it is a correctness switch and never on while timing.
bool g_verify = false;

// ---------------------------------------------------------------- the building population
//
// `--buildings` is OFF by default and that is not timidity, it is what keeps numbers
// taken without buildings comparable with each other. The two
// building kernels write cell temperatures, so switching them on moves the census digest and
// the world-state digest of the scenarios they run in -- the goldens are keyed separately
// (`RegionKeySuffix` + `BuildingKeySuffix`) for exactly that reason, rather than rewritten.
//
// `off`     no buildings registered.
// `corpus`  every `AddBuildingHeatExchange` the corpus recorded -- 1529 for
//           `sim_corpus_744825.bin`. The canonical setting, and the one with a golden.
// `N`       that many, tiled out of the recorded population. See `ScaleBuildings`.
const char* g_buildings = nullptr;  // nullptr == off
// Ticks run before the measured ones, to put the world in the regime the measurement is
// about rather than the transient it starts in. A freshly
// seeded asteroid spends its first thousand ticks equilibrating, every idle-frame number
// taken over that stretch is an average of two different worlds, and the one an idle colony
// actually runs is the second.
int g_settle = 0;
std::vector<SeedBuilding> g_building_pool;
bool g_sunlight = false;
int g_verify_fails = 0;

// ------------------------------------------------------------------------- the fields
//
// `--fields` is OFF by default, for the reason `--buildings` is: numbers taken without it
// must stay comparable, and a flag that silently changed them would make the
// whole file incomparable with itself. On, it registers one extension property per RULE and
// gives each one a field, so a run prices the rule it names and nothing else.
//
// `off`      no field registered. `StepFields` is not called at all -- not "called and
//            returned on its first line", not called, exactly as it is not called for
//            radiation or the element chunks.
// `idle`     one field, no decay and no source: `Field::Idle()` is true. THE GATE, and the
//            only configuration that answers "what does a registered field cost a world that
//            is not using it".
// `decay`    one field with `kDecayFactor` and no source, pre-filled so every cell has a
//            value to decay. The one whole-region pass a field runs on its own account.
// `point`    one field with the RADIATION law and one point source: an ellipse raster with a
//            Bresenham walk per emitted pixel. The expensive rule, and the same shape
//            `TickConstant` has -- which is the comparison `--radiation` exists to make
//            possible in the same table.
// `beam`     one field with the LIGHT law and one directional source: parallel lanes across
//            the world, one visit per cell. The cheap rule, and the one the first named
//            consumer (attenuated light) wants.
// `element`  one field with one element source: a region sweep that emits per kilogram from
//            every cell of one element. Granite, because both scenarios are full of it.
// `all`      all five at once, which is the canonical setting and the only one with a golden.
//            A field is per property and the rules do not interact, so five fields stepped in
//            one phase cost what the five cost separately -- and the per-rule split comes
//            from running the single-rule modes, which are dials and are reported, not
//            asserted. Same rule `--buildings N` is under.
const char* g_fields = nullptr;  // nullptr == off
// Radiation emitters, and `0` is off. `--radiation N` also sets `SimData::radiationEnabled`,
// the byte that switches radiation on; radiation is free until it is set.
//
// THE EMITTERS MOVE THE WORLD'S RANDOM STREAM. `TickConstant` draws from the world LCG for
// the outer three quarters of its ellipse, so a run with emitters in it takes a different
// number of draws than one without and the gas shuffle downstream lands differently. The
// state digest therefore moves, legitimately, and that is why this has its own golden key
// rather than sharing one. It is also the precise reason `sim/fields.h` has no noise in it:
// a field that drew from the same stream would make any mod's registration a divergence.
//
// TWO VARIABLES, NOT ONE, because "the byte is set" and "there are emitters" are separate
// facts and `--radiation 0` is the configuration that separates them: it prices
// `StepRadiationField` -- the whole-region occlusion and decay pass, which runs on the byte
// alone -- with nothing feeding it. A single count could not say that.
bool g_radiation_on = false;
int g_radiation = 0;


// ------------------------------------------------------------------------- the census
//
// The cell counts sim/census.h collected while the timings above were being taken. They are
// printed as a separate table rather than as extra columns on the timing one because they
// answer a different question and are keyed differently: the timing table is this file's own
// sample buckets, the census is the kernels' own slots, and `StepFlow`/`StepLiquidDisplacement`
// are two rows there and one here.
//
// WHAT THE COLUMNS MEAN.
//   cells/frame   what the sweep's own loop stepped over, divided by the frames it ran in.
//   stood         the share of those it actually stood on -- 100 % unless the kernel has an
//                 internal run-skip, which today only `StepConduction`'s quiet tiles do.
//   changed       announced changes per cell examined. A DENSITY, not a fraction: a sweep may
//                 announce one cell twice (source and destination), so this can exceed 100 %.
//   of region     cells/frame against what this scenario's active regions asked for, per
//                 invocation. This is the column the whole file exists for. 100 % is a kernel
//                 doing exactly what it was told; a kernel far above it is scanning past its
//                 region, which is the defect the sweeps-walk-rectangles change fixed once and
//                 which nothing but a stopwatch could see coming back.
const char* g_run_json = nullptr;
const char* g_corpus_path = nullptr;
int g_census_fails = 0;
int g_alloc_fails = 0;
bool g_record_goldens = false;
// The multi-region shape the goldens describe. Twelve because that is the top of the range a
// late Spaced Out cluster actually reaches; the defect this gate exists for scales with it.
const char kCanonicalCluster[] = "cluster:12";
// The settle that reaches the plateau, measured rather than picked:
// conduction's changed share falls 58.4 % -> 39.7 -> 29.6 -> 15.9 -> 5.8 -> 2.5 over settles
// of 0, 250, 500, 1000, 2000 and 4000, and then stops -- 8000 reads 2.55 %, which is up, not
// down. Four thousand is where the transient has decayed and every further tick is paid for
// nothing.
inline constexpr int kCanonicalSettle = 4000;
// The one canonical field configuration: every rule at once. One key rather than six, on the
// principle this file already applies to `--buildings N` -- a configuration the gate runs is
// asserted, a dial somebody turns to read a curve is reported. `all` is the configuration the
// gate runs because it is the only one that would notice a rule going quiet.
const char kCanonicalFields[] = "all";
// And the one canonical emitter population. Eight is a dial in exactly the way 1529 buildings
// is not -- there is no recorded population to take it from, because no save this project has
// ever read had radiation on. What makes the number defensible is that the table reports the
// per-emitter cost beside the total, so the shape of the measurement does not depend on it;
// what makes it CANONICAL is only that the gate needs a key, and a key needs a fixed number.
inline constexpr int kCanonicalRadiation = 8;
int g_golden_fails = 0;

struct CensusRow {
  int slot;
  double per_frame;
  double stood_pct;
  double changed_pct;
  double region_pct;
  bool within;
};

std::vector<CensusRow> CensusRows(int ticks) {
  std::vector<CensusRow> rows;
  const double frames = ticks > 0 ? static_cast<double>(ticks) : 1.0;
  for (int sl = 0; sl < census::kSlotCount; ++sl) {
    const census::Counters& c = census::g_counts[sl];
    // ANY SLOT WITH COUNTS, not only the cell sweeps (`StepFields`, for example).
    // `IsCellSweep` still decides the ASSERTION (`WithinBudget`, and the `of region` column,
    // which has no meaning for a phase whose walks are rays and lanes); what it no longer
    // decides is whether a kernel that counted itself gets to be seen.
    if (c.examined == 0) continue;
    const double examined = static_cast<double>(c.examined);
    const double stood = static_cast<double>(c.examined - c.skipped);
    // Per INVOCATION, because a kernel runs once per region per substep and the region budget
    // is per invocation. Dividing the frame's total by the frame's budget instead would call
    // every kernel a scanner on any world that takes more than one substep a frame.
    const double asked =
        static_cast<double>(census::g_region_cells) * static_cast<double>(c.invocations);
    CensusRow r{};
    r.slot = sl;
    r.per_frame = examined / frames;
    r.stood_pct = 100.0 * stood / examined;
    r.changed_pct = 100.0 * static_cast<double>(c.changes) / examined;
    r.region_pct = census::IsCellSweep(sl) && asked > 0.0 ? 100.0 * examined / asked : 0.0;
    r.within = census::WithinBudget(sl);
    rows.push_back(r);
  }
  return rows;
}

// The census as one canonical line per kernel, digested. One golden key per scenario rather
// than four per kernel, for the reason `diffsim.all.digest` is one key: the printed table
// above IS the diff, and a suite that names forty numbers is a suite whose failure nobody
// reads. Counts are exact integers with no clock in them, so unlike a timing this can be
// pinned at all -- that is the whole reason the census is worth having as a gate and the
// milliseconds beside it never will be.
std::string CensusDigest() {
  goldens::Md5 md5;
  char line[256];
  for (int sl = 0; sl < census::kSlotCount; ++sl) {
    const census::Counters& c = census::g_counts[sl];
    // Same widening as `CensusRows`, and for the same reason: the digest and the table it
    // tells a reader to go and read must describe the same set of rows.
    if (c.examined == 0) continue;
    snprintf(line, sizeof(line), "%s %llu %llu %llu %llu\n", census::Name(sl),
             static_cast<unsigned long long>(c.examined),
             static_cast<unsigned long long>(c.skipped),
             static_cast<unsigned long long>(c.changes),
             static_cast<unsigned long long>(c.invocations));
    md5.Update(reinterpret_cast<const uint8_t*>(line), strlen(line));
  }
  return md5.HexDigest();
}

// Non-canonical means "this run is not the one the goldens describe", and it is reported
// rather than silently skipped -- the same rule goldens.h states for the suites. `--verify`
// is on the list because it is not merely an extra check: it toggles four cells' impermeable
// bit every tick, on purpose, so the projection's dirty rectangles are genuinely different
// and a census taken under it describes a different run.
const char* CensusNonCanonical(const char* regions, int width, int height, int ticks,
                               float dt) {
  if (g_verify) return "--verify";
  if (width != 256 || height != 384) return "a non-default grid";
  if (ticks != 50) return "a non-default tick count";
  if (dt < 0.199f || dt > 0.201f) return "a non-default dt";
  // TWO canonical region modes, not one. `clamped` is the base game -- one region. `cluster:12` is a Spaced Out save
  // with twelve discovered asteroids, and it is goldened separately rather than instead
  // because the two gate different things: a fault in the per-region snapshot refresh leaves
  // `clamped` bit-identical (region 0 always takes the whole grid) and moves `cluster:12`.
  // Proved by planting exactly that fault -- see `WorldStateDigest`.
  if (strcmp(regions, "clamped") != 0 && strcmp(regions, kCanonicalCluster) != 0) {
    return "a non-default region mode";
  }
  // TWO canonical building settings, for the same reason there are two canonical region
  // modes: `off` keeps its own key, `corpus` is the whole recorded base and is the one the buildings work is aimed at.
  // A hand-picked count is a dial, not a configuration -- it is reported and not asserted.
  if (g_buildings != nullptr && strcmp(g_buildings, "corpus") != 0) {
    return "a non-default building count";
  }
  // TWO canonical settles, on the same principle again: 0 keeps its own key, and `kCanonicalSettle` is the settled world. Any other
  // value is a dial for reading the settle curve, and a dial is reported, not asserted.
  if (g_settle != 0 && g_settle != kCanonicalSettle) return "a non-default settle";
  // TWO canonical field settings and two canonical emitter populations, for the third and
  // fourth time on the same principle: `off` keeps its own key, and one configuration of each is the one the gate runs. Everything else is a
  // dial for reading the per-rule cost curve.
  if (g_fields != nullptr && strcmp(g_fields, kCanonicalFields) != 0) {
    return "a non-default field configuration";
  }
  if (g_radiation_on && g_radiation != kCanonicalRadiation) {
    return "a non-default emitter population";
  }
  return nullptr;
}

// The suffix that keeps the two canonical region modes in separate golden keys.
const char* RegionKeySuffix(const char* regions) {
  return strcmp(regions, kCanonicalCluster) == 0 ? ".c12" : "";
}

// And the one that keeps the buildings-on run out of the buildings-off run's key.
const char* BuildingKeySuffix() {
  return g_buildings != nullptr && strcmp(g_buildings, "corpus") == 0 ? ".b" : "";
}

// And the one that keeps the settled run out of the transient run's key.
//
// A SETTLED GOLDEN IS THE POINT, not a bonus. A change which is wrong only after a world
// settles would pass every other suite here, because no scenario runs long enough to leave the transient. This key
// is that scenario. It costs the gate about half a minute and it is the only assertion here
// that describes the regime an idle colony actually runs in.
const char* SettleKeySuffix() {
  static char buf[16];
  if (g_settle != kCanonicalSettle) return "";
  snprintf(buf, sizeof(buf), ".s%d", kCanonicalSettle);
  return buf;
}

// And the one that keeps the run with a field solver in it out of the run without one.
//
// IT HAS TO EXIST EVEN THOUGH A FIELD WRITES NO CELL. A field's values live in an extension
// property, not in `PhaseEntry`, so `state.*` cannot see them and a fields run would share a
// key with the fields-off run while doing visibly different work. Two things do move: the
// census, because `StepFields` now has counts, and `field.*`, which is the new key below and
// the only one that reads what the solver actually wrote.
const char* FieldKeySuffix() {
  return g_fields != nullptr && strcmp(g_fields, kCanonicalFields) == 0 ? ".f" : "";
}

// And the one for the emitters, which unlike the fields DO move the world -- see `g_radiation`.
const char* RadiationKeySuffix() {
  return g_radiation_on && g_radiation == kCanonicalRadiation ? ".r" : "";
}

// The world itself, after the run: every game cell's element, mass and temperature, in
// padded order, digested. The census counts what the kernels LOOKED AT; this counts what
// they LEFT BEHIND, and the two catch different things -- a change that visits exactly the
// same cells and writes different numbers into them moves this and not the census.
//
// IT EXISTS BECAUSE diffsim IS NOT SENSITIVE ENOUGH ON ONE AXIS. Two faults were planted in
// the per-region snapshot work: deleting the refresh outright for every region
// after the first, which diffsim CATCHES, and stripping the snapshot rectangle's margins,
// which diffsim MISSES -- `--scenario all` passes with the recorded digest, 746 lines and
// 4802/4900/98, `regionsplit`, `regionadj` and `regionlap` included. A differential oracle
// reports what its scenarios provoke, and none of diffsim's worlds has a region whose halo
// reads cells its neighbour has just written. Twelve strips across one asteroid do. This key
// catches both, and needs no game content to do it.
//
// Bit patterns, not values: two floats that print the same are not the same float, and the
// point of a state digest is to fail on the ulp that a formatted comparison forgives.
std::string WorldStateDigest(const World& w) {
  goldens::Md5 md5;
  const std::vector<PhaseEntry>& cells = w.Phases();
  for (const CellWalk cw : w.GameCells()) {
    const PhaseEntry& c = cells[cw.padded];
    uint8_t buf[10];
    memcpy(buf, &c.element, 2);
    memcpy(buf + 2, &c.mass, 4);
    memcpy(buf + 6, &c.temperature, 4);
    md5.Update(buf, sizeof(buf));
  }
  return md5.HexDigest();
}

// Klei's own cell validator, run over the world the scenario left behind. `simcheck.h` has
// the transcription and what each class means; this is the walk.
//
// IT ANSWERS A QUESTION THE TWO DIGESTS ABOVE CANNOT. `census` and `state` are both
// *relative*: they say that something moved, never that what is there is wrong. A NaN
// temperature that has been in a kernel since the day it was written is a stable digest, and
// both keys go green on it forever: an `ElementConsumer` publishing a NaN can pass this whole
// gate and show only in the running game's DevTools overlay. This key is that overlay, offline, on every tick's
// worth of cells, with no game and no Klei DLL in the room. It needs no game content at all,
// so unlike `diffsim` it is a check any machine can run.
simcheck::Counts SimCheckCounts(const World& w, const ElementTable& t) {
  simcheck::Counts c;
  const std::vector<PhaseEntry>& cells = w.Phases();
  for (const CellWalk cw : w.GameCells()) {
    const PhaseEntry& p = cells[cw.padded];
    c.Add(t.At(p.element), p.mass, p.temperature);
  }
  return c;
}

// The key is `<kind>.<scenario>`; everything else -- the file, the re-record path, the
// non-canonical refusal, the wording of a failure -- is `goldens::CheckDigest`, shared with
// `vftest`. Two suites asserting digests must not drift
// apart in HOW they assert them.
bool CheckDigestGolden(const char* kind, const char* scenario_key, const std::string& actual,
                       const char* noncanonical, const char* what_moved) {
  char key[64];
  snprintf(key, sizeof(key), "%s.%s", kind, scenario_key);
  return goldens::CheckDigest("GOLDENS.txt", key, actual, g_record_goldens, noncanonical,
                              what_moved);
}

void ReportCensus(int ticks) {
  const std::vector<CensusRow> rows = CensusRows(ticks);
  if (rows.empty()) return;
  printf("  %-22s %12s %8s %9s %10s %9s\n", "census", "cells/frame", "stood", "changed",
         "of region", "bound");
  for (const CensusRow& r : rows) {
    // `of region` is informational and is EXPECTED to exceed 100 % for four of these kernels;
    // `bound` is the assertion. See `census::SweepBound` for why each kernel's is what it is.
    printf("  %-22s %12.0f %7.1f%% %8.2f%% %9.1f%% %9s%s\n", census::Name(r.slot),
           r.per_frame, r.stood_pct, r.changed_pct, r.region_pct,
           census::BoundName(census::SweepBound(r.slot)),
           r.within ? "" : "  ** OVER ITS DECLARED BOUND **");
  }
  const census::Counters& outside = census::g_counts[census::kSlotCount];
  if (outside.changes > 0) {
    printf("  %-22s %12s %8s %9.0f %10s\n", "(outside any sweep)", "-", "-",
           static_cast<double>(outside.changes) / (ticks > 0 ? ticks : 1), "changes/frame");
  }
  printf("  region budget %llu cells (%llu inclusive) over %llu region(s); padded grid %llu\n",
         static_cast<unsigned long long>(census::g_region_cells),
         static_cast<unsigned long long>(census::g_region_cells_incl),
         static_cast<unsigned long long>(census::g_regions),
         static_cast<unsigned long long>(census::g_grid_cells));
  for (const CensusRow& r : rows) {
    if (r.within) continue;
    // Non-zero exit, like `--verify`: a bound this file declares and does not enforce is a
    // comment. `g_census_fails` is checked in `main`.
    printf("  CENSUS FAIL: %s examined %llu cells over %llu invocation(s), bound %s allows "
           "%llu\n",
           census::Name(r.slot),
           static_cast<unsigned long long>(census::g_counts[r.slot].examined),
           static_cast<unsigned long long>(census::g_counts[r.slot].invocations),
           census::BoundName(census::SweepBound(r.slot)),
           static_cast<unsigned long long>(census::BudgetPerInvocation(r.slot) *
                                           census::g_counts[r.slot].invocations));
    ++g_census_fails;
  }
}

// The machine-readable half, and the provenance is the point of it.
//
// A number without the inputs that produced it is not a measurement, it is an
// anecdote -- so every record here carries the corpus it read, that corpus's md5, the scenario,
// the grid, the region mode and the tick count, and a reader that finds two files disagreeing
// can tell which of those moved.
std::string CorpusDigest(const char* path) {
  if (path == nullptr) return "";
  FILE* f = fopen(path, "rb");
  if (f == nullptr) return "";
  goldens::Md5 md5;
  std::vector<uint8_t> buf(1 << 16);
  for (;;) {
    const size_t got = fread(buf.data(), 1, buf.size(), f);
    if (got == 0) break;
    md5.Update(buf.data(), got);
  }
  fclose(f);
  return md5.HexDigest();
}

// THE TIMINGS ARE IN THE SAME RECORD AS THE COUNTS, not a second file, and that is the whole
// argument for this function's existence.
//
// Without a committed baseline and a machine-readable form, every comparison ends at "inside
// the noise", and a real regression gets blamed on the machine. A count and a timing taken from
// different runs cannot be reconciled afterwards -- the count says what the kernel iterated
// and the timing says what that cost, and the only way "it examines the same cells and takes
// longer" can ever be stated is if one record carries both.
//
// Every field of provenance is here for the same reason: the corpus and its md5, the grid, the
// region mode, the building population, the tick count and dt. A number without the inputs
// that produced it is an anecdote.
void WriteRunJson(const char* path, const char* label, const SeedWorld& seed,
                  const char* region_mode, int ticks, float dt,
                  size_t buildings_registered, size_t building_cells, const Result& r,
                  size_t alloc_first, size_t alloc_steady) {
  // Appended, one JSON object per line: a run that overwrote the last one would make the
  // before/after comparison this file exists for impossible to do with two commands.
  FILE* f = fopen(path, "a");
  if (f == nullptr) {
    printf("  json: could not open %s for append\n", path);
    return;
  }
  const std::vector<CensusRow> rows = CensusRows(ticks);
  fprintf(f,
          "{\"scenario\":\"%s\",\"width\":%d,\"height\":%d,\"regions\":\"%s\",\"ticks\":%d,"
          "\"settle\":%d,\"fields\":\"%s\",\"radiation\":%d,\"emitters\":%d,"
          "\"dt\":%.4f,\"corpus\":\"%s\",\"corpus_md5\":\"%s\","
          "\"region_cells\":%llu,\"grid_cells\":%llu,\"region_count\":%llu,"
          "\"buildings\":%llu,\"building_cells\":%llu,\"kernels\":[",
          label, seed.width, seed.height, region_mode, ticks, g_settle,
          g_fields ? g_fields : "off", g_radiation_on ? 1 : 0, g_radiation,
          static_cast<double>(dt),
          g_corpus_path ? g_corpus_path : "", CorpusDigest(g_corpus_path).c_str(),
          static_cast<unsigned long long>(census::g_region_cells),
          static_cast<unsigned long long>(census::g_grid_cells),
          static_cast<unsigned long long>(census::g_regions),
          static_cast<unsigned long long>(buildings_registered),
          static_cast<unsigned long long>(building_cells));
  for (size_t i = 0; i < rows.size(); ++i) {
    const census::Counters& c = census::g_counts[rows[i].slot];
    fprintf(f,
            "%s{\"name\":\"%s\",\"examined\":%llu,\"skipped\":%llu,\"changes\":%llu,"
            "\"invocations\":%llu}",
            i ? "," : "", census::Name(rows[i].slot),
            static_cast<unsigned long long>(c.examined),
            static_cast<unsigned long long>(c.skipped),
            static_cast<unsigned long long>(c.changes),
            static_cast<unsigned long long>(c.invocations));
  }
  fprintf(f, "],\"outside_sweep_changes\":%llu,\"timings\":[",
          static_cast<unsigned long long>(census::g_counts[census::kSlotCount].changes));
  // Every timed bucket, including the two sum rows. `med_ms` is the median over this run's
  // SUBSTEPS (or ticks, for the frame-level rows) and is the statistic the baseline tooling
  // aggregates across runs; `min_ms` is the steadiest single number this harness produces and
  // is carried so a reader can prefer it. `calls` is emitted because a bucket whose call count
  // moved is not comparable with the one it is being diffed against, however close the
  // milliseconds look.
  const Samples* all[] = {&r.conduction, &r.state,    &r.sublimation, &r.pressure,
                          &r.displace,   &r.flow,     &r.liqdisp,     &r.bldheat,
                          &r.bldbld,     &r.zeromassless,
                          &r.radfield,   &r.rademit,  &r.fields,
                          &r.project,    &r.textures, &r.substep,
                          &r.frame};
  for (size_t i = 0; i < sizeof(all) / sizeof(all[0]); ++i) {
    const Samples& b = *all[i];
    fprintf(f,
            "%s{\"name\":\"%s\",\"calls\":%zu,\"min_ms\":%.6f,\"med_ms\":%.6f,"
            "\"p95_ms\":%.6f,\"max_ms\":%.6f,\"total_ms\":%.6f}",
            i ? "," : "", b.name, b.ms.size(), b.Min(), b.Quantile(0.5), b.Quantile(0.95),
            b.Max(), b.Total());
  }
  fprintf(f, "],\"alloc_first_tick\":%llu,\"alloc_steady\":%llu}\n",
          static_cast<unsigned long long>(alloc_first),
          static_cast<unsigned long long>(alloc_steady));
  fclose(f);
  printf("  json: appended one record to %s\n", path);
}

// ---------------------------------------------------------------------- the field solver
//
// One extension property per RULE, one field on each, all registered by hand rather than
// sent as messages -- the same choice `--buildings` makes and for the same reason: `bench`
// links the kernels out of `sim/` and has no message queue to drain. The registration path
// is the one the DLL's own `kRegisterField` handler takes (`ext::FieldRegistry::Register`
// with a `RegisterFieldMessage`), so a field registered here is validated by the code that
// validates a mod's, and the only thing skipped is the queue.
//
// EVERY STRENGTH IS PICKED SO THAT NOTHING SATURATES IN FIFTY SUBSTEPS. A field clamped at
// `clampHi` stops moving once it gets there, and a run whose cells are all pinned at 1.0
// measures the raster while reporting that nothing changed -- which is the shape of number
// this harness exists not to produce. The per-field summary below asserts the other half of
// that: a field that is not idle and left no cell nonzero did no work, whatever its timing
// says.
struct FieldSetup {
  ext::FieldState state;
  // Parallel arrays, one entry per registered field, in registration order.
  std::vector<int32_t> properties;
  std::vector<const char*> rules;
  std::vector<bool> idle;
  // Each property's bytes as the run STARTED, so the report can say whether a field moved
  // rather than only whether it is nonzero. The difference is the whole assertion: the decay
  // field is pre-filled, so "it has nonzero cells" is true of it before a single tick runs
  // and proves nothing at all, and an idle field's claim is the mirror image -- it must come
  // out exactly as it went in.
  std::vector<std::vector<float>> seeded;
  int32_t decay_property = -1;
  bool wants_beam = false;
  bool Active() const { return !properties.empty(); }
};

// The five rule names `--fields` understands, plus the two words that are not rules.
bool ValidFieldMode(const char* mode) {
  static const char* kModes[] = {"off", "idle", "decay", "point", "beam", "element", "all"};
  for (size_t i = 0; i < sizeof(kModes) / sizeof(kModes[0]); ++i) {
    if (strcmp(mode, kModes[i]) == 0) return true;
  }
  return false;
}

bool FieldModeHas(const char* rule) {
  if (g_fields == nullptr) return false;
  if (strcmp(g_fields, kCanonicalFields) == 0) return true;
  return strcmp(g_fields, rule) == 0;
}

// STAGE ONE, and it has to run BEFORE the world is allocated. `CellPropertyRegistry` closes
// registration at the first allocate -- deliberately, so that a property is never a different
// length from `insulation_` -- so a field's storage has to be asked for while the world is
// still empty. The fields themselves are registered in stage two, after the cells exist,
// because a field is a rule bolted onto a property that is already there.
void RegisterFieldProperties(World* world, ElementTable* elements, FieldSetup* fs,
                             int32_t* rad_attribute, int32_t* light_attribute) {
  if (g_fields == nullptr) return;
  std::string err;
  ext::CellPropertyRegistry& props = world->ExtCells();
  auto declare = [&](const char* name) {
    const int32_t idx =
        props.Register(name, ext::kExtF32, ext::kRehydrated, 1, 0u, false, &err);
    if (idx < 0) printf("  fields: %s\n", err.c_str());
    return idx;
  };
  // The two attenuation tables, each a MIRROR of a column Klei's element table already
  // carries. That is not a shortcut -- it is the configuration `gastest`'s two oracles prove
  // equal to `RadiationAbsorptionAlongLine` and to `ComputeSunBeam`, so the cost measured
  // here is the cost of the walk those arms certify rather than of some cheaper stand-in
  // with a flat table in it.
  auto mirror = [&](const char* name, bool light) {
    ext::ElementAttributeRegistry& attrs = elements->MutableAttributes();
    const int32_t idx = attrs.Register(name, ext::kExtF32, 1, false, &err);
    if (idx < 0) {
      printf("  fields: %s\n", err.c_str());
      return idx;
    }
    for (int32_t i = 0; i < elements->Count(); ++i) {
      const Element& el = elements->At(static_cast<uint16_t>(i));
      const float v = light ? el.lightAbsorptionFactor : el.radiationAbsorptionFactor;
      uint32_t bits = 0;
      memcpy(&bits, &v, sizeof(bits));
      attrs.Write(idx, el.id, 0, bits);
    }
    return idx;
  };

  if (FieldModeHas("idle")) fs->properties.push_back(declare("bench.field_idle"));
  if (FieldModeHas("decay")) fs->properties.push_back(declare("bench.field_decay"));
  if (FieldModeHas("point")) {
    *rad_attribute = mirror("bench.rad_atten", false);
    fs->properties.push_back(declare("bench.field_point"));
  }
  if (FieldModeHas("beam")) {
    *light_attribute = mirror("bench.light_atten", true);
    fs->properties.push_back(declare("bench.field_beam"));
    fs->wants_beam = true;
  }
  if (FieldModeHas("element")) fs->properties.push_back(declare("bench.field_element"));
  fs->properties.erase(std::remove(fs->properties.begin(), fs->properties.end(), -1),
                       fs->properties.end());
}

// STAGE TWO: the rules, their sources, and the one pre-filled array.
//
// The order here is the order stage one declared the properties in, and it is also the order
// `StepFields` applies them -- registration order is the only order predictable from outside
// (sim/fields.h), so a run that reordered these would be a different measurement.
void RegisterFields(World* world, const ElementTable& elements, const Tables& tables,
                    FieldSetup* fs, int32_t rad_attribute, int32_t light_attribute) {
  if (!fs->Active()) return;
  const int32_t pw = world->PaddedWidth();
  std::string err;
  size_t next = 0;
  auto take = [&]() { return fs->properties[next++]; };
  auto add = [&](const ext::RegisterFieldMessage& m, const char* rule, bool is_idle) {
    const int32_t fi = fs->state.registry.Register(world->ExtCells(), elements.Attributes(),
                                                   m, &err);
    if (fi < 0) {
      printf("  fields: %s\n", err.c_str());
      return -1;
    }
    fs->rules.push_back(rule);
    fs->idle.push_back(is_idle);
    return fi;
  };
  ext::RegisterFieldMessage m{};
  m.attributeIdx = -1;
  m.attributeFallback = 0.0f;
  m.law = ext::kAttenFlat;
  m.combine = ext::kCombineTransmission;
  m.decayMode = ext::kDecayNone;
  m.decayKeep = 1.0f;
  m.floorValue = 0.0f;
  m.clampLo = 0.0f;
  m.clampHi = 1.0f;

  if (FieldModeHas("idle")) {
    ext::RegisterFieldMessage idle = m;
    idle.propertyIdx = take();
    add(idle, "idle (no decay, no source)", true);
  }
  if (FieldModeHas("decay")) {
    ext::RegisterFieldMessage decay = m;
    decay.propertyIdx = take();
    decay.decayMode = ext::kDecayFactor;
    // 0.999 a substep, from a pre-filled 0.5: after the fifty substeps this suite times, a
    // cell holds 0.476. It never reaches `floorValue`, which matters -- a decay pass whose
    // cells have all snapped to zero takes the `v == 0.0f` early-out on every one of them and
    // prices the empty case while claiming to price decay.
    decay.decayKeep = 0.999f;
    fs->decay_property = decay.propertyIdx;
    add(decay, "decay only (kDecayFactor 0.999)", false);
  }
  if (FieldModeHas("point")) {
    ext::RegisterFieldMessage point = m;
    point.propertyIdx = take();
    point.attributeIdx = rad_attribute;
    point.law = ext::kAttenRadiationMass;
    point.combine = ext::kCombineTransmission;
    const int32_t fi = add(point, "point source (radiation law, 24x24)", false);
    if (fi >= 0) {
      ext::FieldSource s;
      s.id = 1;
      s.kind = ext::kSourcePoint;
      s.strength = 0.01f;
      // The middle of the grid in PADDED coordinates -- the border ring is one cell on each
      // side, so a game cell (x, y) is padded (x + 1, y + 1).
      s.target = (world->GameHeight() / 2 + 1) * pw + world->GameWidth() / 2 + 1;
      s.radius_x = 24;
      s.radius_y = 24;
      s.cone_angle = 360.0f;
      fs->state.registry.SetSource(fi, s);
    }
  }
  if (FieldModeHas("beam")) {
    ext::RegisterFieldMessage beam = m;
    beam.propertyIdx = take();
    beam.attributeIdx = light_attribute;
    beam.law = ext::kAttenLightMass;
    beam.combine = ext::kCombineExposure;
    const int32_t fi = add(beam, "directional (light law, 60 deg)", false);
    if (fi >= 0) {
      ext::FieldSource s;
      s.id = 1;
      s.kind = ext::kSourceDirectional;
      s.strength = 0.01f;
      s.target = 0;  // the one world declared below
      // Sixty degrees, which takes the ROW branch of the lane sweep and not the trivial
      // straight-down case: a zero offset per row is a column walk, and a column walk is the
      // one shape of this kernel that proves the least.
      s.dir_x = 0.5f;
      s.dir_y = 0.8660254f;
      fs->state.registry.SetSource(fi, s);
    }
  }
  if (FieldModeHas("element")) {
    ext::RegisterFieldMessage elem = m;
    elem.propertyIdx = take();
    const int32_t fi = add(elem, "element source (granite, per kg)", false);
    if (fi >= 0) {
      ext::FieldSource s;
      s.id = 1;
      s.kind = ext::kSourceElement;
      // Per kilogram, and granite weighs 2000 kg a cell in both scenarios, so this is 0.002
      // a substep and 0.1 over the fifty -- under the clamp with an order of magnitude spare.
      s.strength = 0.000001f;
      s.target = tables.Need(oni_bench::kGranite, "Granite");
      fs->state.registry.SetSource(fi, s);
    }
  }

  // The decay field's cells, pre-filled. A decay rule applied to an all-zero array is a
  // whole-region sweep that early-outs on every cell; this is what makes the pass pay for
  // the arithmetic it is supposed to be pricing.
  if (fs->decay_property >= 0) {
    float* values = world->ExtCells().MutableF32(fs->decay_property);
    if (values != nullptr) {
      for (size_t p = 0; p < world->PaddedCount(); ++p) values[p] = 0.5f;
      world->ExtCells().NoteNonzero(fs->decay_property);
    }
  }

  // And the before picture, taken last so it includes the pre-fill.
  fs->seeded.resize(fs->properties.size());
  for (size_t i = 0; i < fs->properties.size(); ++i) {
    const float* values = world->ExtCells().F32(fs->properties[i]);
    if (values == nullptr) continue;
    fs->seeded[i].assign(values, values + world->PaddedCount());
  }
}

// Every registered field's values, over the game cells, as bit patterns. The new golden, and
// the ONLY one that can see what the solver wrote: a field lives in an extension property, so
// `WorldStateDigest` -- element, mass, temperature -- is blind to it by construction, and a
// fields run would otherwise share every key it has with a run that registered nothing.
std::string FieldValueDigest(const World& w, const FieldSetup& fs) {
  goldens::Md5 md5;
  for (size_t i = 0; i < fs.properties.size(); ++i) {
    const float* values = w.ExtCells().F32(fs.properties[i]);
    if (values == nullptr) continue;
    for (const CellWalk cw : w.GameCells()) {
      md5.Update(&values[cw.padded], sizeof(float));
    }
  }
  return md5.HexDigest();
}

// ------------------------------------------------------------------- the emitters
//
// A deterministic lattice of `kRadiationConstant` emitters, which is the type whose per-step
// work is `TickConstant`: an ellipse raster with a `RadiationAbsorptionAlongLine` walk per
// emitted pixel. That is the same shape `kSourcePoint` has, on purpose -- the two rows sit
// next to each other in the table below, and the comparison is the whole reason this flag and
// `--fields point` landed together.
//
// `emitRate` and `emitSpeed` are both zero, which is the steady case: `StepRadiationEmitters`
// takes its `rate_zero` path, pins the step count at one, and the emitter fires on every
// substep rather than on a duty cycle. A price taken on a duty cycle is a price divided by a
// number nobody stated.
void RegisterEmitters(World* world, RadiationState* rad, int count) {
  if (count <= 0) return;
  const int32_t pw = world->PaddedWidth();
  const int32_t gw = world->GameWidth(), gh = world->GameHeight();
  int cols = 1;
  while (cols * cols < count) ++cols;
  const int rows = (count + cols - 1) / cols;
  for (int i = 0; i < count; ++i) {
    const int cx = static_cast<int>(static_cast<int64_t>(gw) * (2 * (i % cols) + 1) /
                                    (2 * cols));
    const int cy = static_cast<int>(static_cast<int64_t>(gh) * (2 * (i / cols) + 1) /
                                    (2 * rows));
    RadiationEmitterData d;
    d.cell = (cy + 1) * pw + cx + 1;
    d.radius_x = 24;
    d.radius_y = 24;
    d.emit_rads = 1000.0f;
    d.emit_rate = 0.0f;
    d.emit_speed = 0.0f;
    d.emit_direction = 0.0f;
    d.emit_angle = 360.0f;
    d.emit_type = kRadiationConstant;
    rad->emitters.Add(d);
  }
}

void RunScenario(const char* label, const char* scenario_key, const SeedWorld& seed,
                 const Tables& tables, const char* region_mode, int ticks, float dt) {
  ElementTable elements;
  DiseaseTable diseases;
  if (!elements.Load(tables.elements.data(), tables.elements.size()) ||
      !diseases.Load(tables.diseases.data(), tables.diseases.size())) {
    printf("failed to load tables\n");
    return;
  }

  World world;
  // BEFORE the world is allocated, because `InitializeFromCells` closes extension
  // registration. See `RegisterFieldProperties`.
  FieldSetup fields;
  int32_t rad_attribute = -1, light_attribute = -1;
  RegisterFieldProperties(&world, &elements, &fields, &rad_attribute, &light_attribute);
  const std::vector<uint8_t> payload = oni_bench::WorldPayload(seed);
  if (!world.InitializeFromCells(payload.data(), payload.size(), elements, diseases)) {
    printf("failed to seed world\n");
    return;
  }
  const std::vector<int32_t> region = Regions(region_mode, seed.width, seed.height);
  world.SetActiveRegions(region.data(), region.size() / 4);
  // `--sunlight` declares one world covering the grid, which is what a real game sends and
  // what switches the sunlight texture on. Without it `ComputeSunlight` returns on its first
  // line, so the texture number here would not include a pass the live game always pays for.
  //
  // A DIRECTIONAL FIELD NEEDS ONE TOO, and that is not free: `StepFieldDirectional` resolves
  // its source against `World::WorldOffsets()`, so `--fields beam` has to declare a world to
  // have anywhere to shine from -- and declaring one ALSO switches `ComputeSunlight` on
  // inside `FillPropertyTextures`, whether or not `--sunlight` was passed. So the texture
  // timing and the texture census of a `beam` run are not comparable with an otherwise
  // identical run without it. That entanglement is stated rather than worked around: it is
  // real, it is the game's own (a world that has no declared asteroid has no sun either),
  // and the run is keyed separately anyway.
  if (g_sunlight || fields.wants_beam) {
    std::vector<World::WorldOffset> worlds(1);
    worlds[0].x = 0;
    worlds[0].y = 0;
    worlds[0].w = world.GameWidth();
    worlds[0].h = world.GameHeight();
    world.SetWorldOffsets(std::move(worlds));
    world.PromoteWorldOffsets();
  }
  RegisterFields(&world, elements, tables, &fields, rad_attribute, light_attribute);

  // The radiation byte, and the emitters that make setting it mean something. Both kernels return on their first line
  // without it, which is exactly why no number in that file describes them.
  RadiationState radiation;
  if (g_radiation_on) {
    world.SetRadiationEnabled(true);
    RegisterEmitters(&world, &radiation, g_radiation);
  }

  // The building population, registered straight into the component registry rather than
  // sent as messages: `bench` links the kernels out of `sim/` and has no message queue to
  // drain, exactly as it has none for the flow accumulator or the projection mark it
  // already repeats by hand. `BuildingDataFromMessage` is the same function the DLL's
  // `AddBuildingHeatExchange` handler calls, so a building registered here is built from
  // the recorded payload through the identical path -- the only thing skipped is the queue.
  BuildingState buildings;
  size_t buildings_dropped = 0, building_tiles = 0, buildings_wanted = 0;
  if (g_buildings != nullptr && !g_building_pool.empty()) {
    buildings_wanted = strcmp(g_buildings, "corpus") == 0
                           ? g_building_pool.size()
                           : static_cast<size_t>(atoi(g_buildings) < 0 ? 0 : atoi(g_buildings));
    const std::vector<SeedBuilding> placed = oni_bench::ScaleBuildings(
        g_building_pool, buildings_wanted, seed.width, seed.height, &buildings_dropped,
        &building_tiles);
    for (const SeedBuilding& b : placed) {
      buildings.exchange.Add(BuildingDataFromMessage(
          elements, b.elem, b.mass, b.temperature, b.thermal_conductivity,
          b.overheat_temperature, b.operating_kilowatts, b.min_x, b.min_y, b.max_x,
          b.max_y));
    }
  }

  ProjectionBuffers buffers;
  buffers.Allocate(world.GameCount());
  PropertyTextureBuffers textures;
  textures.Allocate(world.GameCount());
  std::vector<uint8_t> beam(world.GameCount(), 0);
  std::vector<float> beam_lanes;
  ProjectionEvents events;
  std::vector<StateChangeOre> ores;
  ProjectionBuffers ref_buffers;
  PropertyTextureBuffers ref_textures;
  if (g_verify) {
    ref_buffers.Allocate(world.GameCount());
    ref_textures.Allocate(world.GameCount());
  }

  Result r;
  r.before = Describe(world, elements);

  // The very first Project is the one that emits a change notification for every cell in
  // the world, so it is not comparable with the ones that follow; run it outside the
  // measurement the way the real first frame does.
  Project(world, elements, &buffers, &events, true);
  events.Clear();

  for (Samples* b : {&r.conduction, &r.state, &r.sublimation, &r.pressure, &r.displace,
                     &r.flow, &r.liqdisp, &r.bldheat, &r.bldbld, &r.zeromassless,
                     &r.project, &r.textures, &r.beam90, &r.beam60, &r.beam30, &r.beam15,
                     &r.radfield, &r.rademit, &r.fields, &r.substep, &r.frame}) {
    b->Reserve(static_cast<size_t>(ticks) * 4 + 8);
  }
  // Warm-up is not skipped here -- the FIRST tick legitimately allocates every scratch buffer
  // and every event list, and a suite that pretended otherwise would be asserting a lie. The
  // count is taken from the second tick onward, and the first tick's total is printed beside
  // it so the size of that warm-up is visible rather than hidden.
  size_t alloc_after_first_tick = 0;
  size_t alloc_at_start = 0;
  float carry = 0.0f;
  size_t substeps_run = 0;
  uint16_t rotation = 0;
  int32_t pressure_dir = -1;
  Composition settled = r.before;
  // NEGATIVE TICKS ARE THE SETTLE, and they run the same body rather than a copy of it.
  // `bench` already has to repeat `StepPhysics`'s frame by hand -- `ClearFlow`, the region
  // mark, `SnapshotFlowElements`, the building kernels' position -- and every one of those
  // is a line the two could drift on. A second loop would be a second place to drift, so
  // the settle is this loop with its recording switched off: `carry`, `rotation` and
  // `pressure_dir` run straight through, which is also what makes the settled state
  // reproducible rather than merely warm.
  for (int tick = -g_settle; tick < ticks; ++tick) {
    const bool measuring = tick >= 0;
    if (tick == 0) {
      // Zeroed HERE and not before the loop. Seeding the world and declaring the regions
      // both announce cell changes, and so does every settle tick; a scenario's census must
      // describe the ticks it reports and nothing else.
      census::Reset();
      settled = Describe(world, elements);
      alloc_at_start = g_alloc_count;
    }
    if (tick == 1) alloc_after_first_tick = g_alloc_count;
    const double frame_ms = TimeIt([&] {
      // `StepPhysics` clears the flow accumulator once a frame, outside the substep loop,
      // and a bench that skips it does not just mis-time the texture pass — it lets the
      // touched-cell list grow for the whole run and prices `FillPropertyTextures` at three
      // hundred times what it costs. This is the first line of the frame for the same
      // reason it is there.
      world.ClearFlow();
      const int substeps = SubstepsForFrame(dt, &carry);
      for (int s = 0; s < substeps; ++s) {
        // Same order as `StepPhysics`: gas pressure before liquid flow, and post-process
        // after both. The order is not free even for timing — each kernel sees a different
        // grid depending on where it sits, and on this world that changes how much work
        // its early-outs skip.
        pressure_dir = -pressure_dir;
        // One pass per region, which is what `StepPhysics` does -- its region loop wraps
        // every kernel in the substep, not one of them. This used to be hardcoded to region
        // 0 on the grounds that `--regions` only ever sent one; `--regions cluster:N` sends
        // N, so the loop is now real.
        //
        // A kernel's sample is the sum over the substep's regions, so it stays "what this
        // kernel cost this substep" whatever N is, and with N = 1 it is the straight-line
        // body it replaces. That is the sum a frame budget is spent out of; the per-region
        // breakdown is not what a frame cares about.
        //
        // The other thing `StepPhysics` does around this body that has to be repeated by
        // hand: the region's projection mark. Without it the only mark is the one
        // `StepGasDisplacement` posts for itself, which happens to cover the region on this
        // world — so `Project` would still verify clean while being timed against a bound
        // narrower than the one the DLL uses.
        double c = 0.0, t = 0.0, g = 0.0, d = 0.0, f = 0.0, l = 0.0, u = 0.0;
        double bh = 0.0, bb = 0.0, z = 0.0, rf = 0.0, re = 0.0, fd = 0.0;
        for (size_t ri = 0; ri < world.RegionCount(); ++ri) {
          const World::PaddedRect& pr = world.PaddedRegionInclusive(ri);
          world.MarkProjectDirtyRect(pr.x0, pr.y0, pr.x1, pr.y1);
          c += TimeIt([&] { StepConduction(&world, elements, ri); });
          t += TimeIt([&] { StepStateChange(&world, elements, &ores, ri); });
          g += TimeIt([&] { StepGasPressure(&world, elements, diseases, pressure_dir, ri); });
          d += TimeIt([&] {
            StepGasDisplacement(&world, elements, pressure_dir, ri, nullptr, &diseases);
          });
          // The germ table as well, so the liquid sweeps price the germs they carry.
          f += TimeIt([&] {
            StepFlow(&world, elements, s == 0, rotation, ri, nullptr, nullptr, nullptr, false,
                     &diseases);
          });
          // The second liquid sweep and the flow-texture snapshot that closes the liquid
          // section. `bench` does not call `StepPhysics`, so every one of these has to be
          // repeated here by hand or the A/B prices a kernel that never ran.
          l += TimeIt([&] {
            StepLiquidDisplacement(&world, elements, pressure_dir, rotation, ri, nullptr, &diseases);
            world.SnapshotFlowElements();
          });
          // `SimData::UpdateComponents` sits between the liquid section and
          // `PostProcessCell` in `UpdateData` (sim/simdll.cpp, kBuildingHeat at 2476), and
          // both building kernels live inside it. They go in the same place here or the
          // sweep below sees cells the buildings have not touched yet -- the same reason
          // `ClearFlow` and `SnapshotFlowElements` are repeated by hand above.
          //
          // Called unconditionally, buildings or not: with an empty registry both are the
          // empty-vector iteration they have always been, which is a real measurement of
          // what the buildings-off configuration costs rather than a branch around it.
          // THE THREE PHASES `bench` NEVER CALLED, in the positions `SimData::UpdateData`
          // puts them (sim/simdll.cpp): `StepRadiationField` immediately before
          // `UpdateComponents`, `StepRadiationEmitters` inside it, and `kPhaseFields`
          // immediately after the emitters -- which is where the published pairwise-order
          // contract says the field solver sits.
          //
          // All three are called UNCONDITIONALLY, buildings-style: with radiation off and no
          // field registered each returns on its first line, and that is a real measurement
          // of what the gate costs rather than a branch around it. It is also the number
          // the field solver's cost claim rests on.
          rf += TimeIt([&] {
            StepRadiationField(&world, elements, diseases, &radiation, ri);
          });
          re += TimeIt([&] {
            StepRadiationEmitters(&world, elements, &radiation, g_tunables.substep_seconds, ri);
          });
          fd += TimeIt([&] {
            ext::StepFields(&world, elements, &world.ExtCells(), elements.Attributes(),
                            &fields.state, ri);
          });
          bh += TimeIt([&] {
            StepBuildingHeatExchange(&world, elements, &buildings, g_tunables.substep_seconds,
                                     &ores, ri);
          });
          bb += TimeIt([&] {
            StepBuildingToBuilding(&world, &buildings, g_tunables.substep_seconds, ri);
          });
          u += TimeIt([&] {
            StepPostProcess(&world, elements, 0, nullptr, ri, nullptr, nullptr, false, nullptr,
                            {}, &diseases);
          });
          // THE WHOLE-GRID PASS. `SimData::UpdateData`
          // (sim/simdll.cpp) runs it here, inside the region loop, immediately after
          // `PostProcessCell` and before the disease growth sweep, unconditionally and over
          // the entire padded grid. Leaving it out would drop a full-grid kernel from every substep
          // -- about 2.5 % of a settled asteroid's substep, and a larger share of an idle frame.
          //
          // It changes no state on either scenario -- `state.*` does not move -- because a settled world has no massless non-solid cells left for it to
          // find. That is the point rather than a reprieve: it is a whole-grid scan that
          // almost never does anything, which is the shape of cost this project is trying to
          // price.
          z += TimeIt([&] { ZeroMasslessCells(&world, elements); });
        }
        if (measuring) {
          r.conduction.Add(c);
          r.state.Add(t);
          r.sublimation.Add(u);
          r.pressure.Add(g);
          r.displace.Add(d);
          r.flow.Add(f);
          r.liqdisp.Add(l);
          r.bldheat.Add(bh);
          r.bldbld.Add(bb);
          r.zeromassless.Add(z);
          r.radfield.Add(rf);
          r.rademit.Add(re);
          r.fields.Add(fd);
          r.substep.Add(c + t + g + d + u + f + l + bh + bb + z + rf + re + fd);
          ++substeps_run;
        }
        ++rotation;
        ores.clear();
      }
      if (g_verify && measuring) {
        // A cell whose element and mass never move but whose impermeable bit does is the
        // one input to the projection's solidity fast path that does not arrive with the
        // cell — it arrives as a `CellPropertiesMessage`, which no scenario in the suite
        // sends, so `kPrevRecheck` would otherwise go untested. Toggling one cell per tick
        // puts it under the full-recompute oracle below. Only under --verify: it is a write
        // into the world, and the timed runs must not have one.
        // Four cells a tick, spread across the grid, because most of an asteroid is
        // already solid and a cell that was solid anyway proves nothing.
        const size_t cells = world.GameCount();
        for (int k = 0; k < 4; ++k) {
          const size_t game = (static_cast<size_t>(tick) * 7919 + k * (cells / 4)) % cells;
          world.MutableProperties(world.Padded(game)) ^= kSolidImpermeable;
        }
      }
      const double pj = TimeIt([&] { Project(world, elements, &buffers, &events, false); });
      const double tx =
          TimeIt([&] { FillPropertyTextures(world, elements, buffers, &textures); });
      if (g_sunlight && measuring) {
        // Timed OUTSIDE the frame total on purpose: nothing live calls it yet, and folding it
        // into `FillPropertyTextures` would move a number earlier measurements are compared
        // against. The sun record is set on the fallback and cleared again, so no kernel above
        // ever runs with one.
        struct BeamCase {
          Samples* samples;
          float dir_x, dir_y;
        };
        const BeamCase cases[] = {{&r.beam90, 0.0f, 1.0f},
                                  {&r.beam60, 0.5f, 0.8660254f},
                                  {&r.beam30, 0.8660254f, 0.5f},
                                  {&r.beam15, -0.9659258f, 0.2588190f}};
        for (const BeamCase& c : cases) {
          World::SunDirection sun;
          sun.dir_x = c.dir_x;
          sun.dir_y = c.dir_y;
          world.SetWorldSun(-1, sun);
          c.samples->Add(TimeIt(
              [&] { ComputeSunBeam(world, elements, buffers, &beam_lanes, &beam); }));
          if (c.samples == &r.beam90 &&
              memcmp(beam.data(), textures.exposed_to_sun.data(), beam.size()) != 0) {
            size_t differ = 0;
            for (size_t i = 0; i < beam.size(); ++i) {
              if (beam[i] != textures.exposed_to_sun[i]) ++differ;
            }
            printf("  SUNBEAM tick %d: overhead beam differs from the sky view in %zu cells\n",
                   tick, differ);
            ++g_verify_fails;
          }
        }
        world.ClearWorldSun(-1);
      }
      if (measuring) {
        r.project.Add(pj);
        r.textures.Add(tx);
      }
      if (g_verify && measuring) {
        ProjectionEvents ref_events;
        Project(world, elements, &ref_buffers, &ref_events, true);
        ref_textures.primed = false;
        FillPropertyTextures(world, elements, ref_buffers, &ref_textures);
        auto same = [&](const char* what, const void* a, const void* b, size_t bytes) {
          if (memcmp(a, b, bytes) == 0) return;
          printf("  VERIFY tick %d: %s differs from a full recompute\n", tick, what);
          ++g_verify_fails;
        };
        same("element", buffers.element.data(), ref_buffers.element.data(),
             buffers.element.size() * 2);
        same("mass", buffers.mass.data(), ref_buffers.mass.data(),
             buffers.mass.size() * 4);
        same("temperature", buffers.temperature.data(), ref_buffers.temperature.data(),
             buffers.temperature.size() * 4);
        same("properties", buffers.properties.data(), ref_buffers.properties.data(),
             buffers.properties.size());
        same("insulation", buffers.insulation.data(), ref_buffers.insulation.data(),
             buffers.insulation.size());
        same("strength", buffers.strength.data(), ref_buffers.strength.data(),
             buffers.strength.size());
        same("radiation", buffers.radiation.data(), ref_buffers.radiation.data(),
             buffers.radiation.size() * 4);
        same("disease_idx", buffers.disease_idx.data(), ref_buffers.disease_idx.data(),
             buffers.disease_idx.size());
        same("disease_count", buffers.disease_count.data(),
             ref_buffers.disease_count.data(), buffers.disease_count.size() * 4);
        same("backwall_element", buffers.backwall_element.data(),
             ref_buffers.backwall_element.data(), buffers.backwall_element.size() * 2);
        same("backwall_mass", buffers.backwall_mass.data(),
             ref_buffers.backwall_mass.data(), buffers.backwall_mass.size() * 4);
        same("backwall_temperature", buffers.backwall_temperature.data(),
             ref_buffers.backwall_temperature.data(),
             buffers.backwall_temperature.size() * 4);
        // Solidity appears in none of the arrays above — it leaves the sim only as
        // `solidInfo` events — so without this line the projection's solidity fast path
        // has no oracle here at all. `previous` is where it is decided and kept, and a
        // full recompute writes exactly the same byte for every cell.
        same("previous", buffers.previous.data(), ref_buffers.previous.data(),
             buffers.previous.size());
        same("tex.flow", textures.flow.data(), ref_textures.flow.data(),
             textures.flow.size() * 4);
        same("tex.liquid", textures.liquid.data(), ref_textures.liquid.data(),
             textures.liquid.size());
        same("tex.liquidData", textures.liquid_data.data(), ref_textures.liquid_data.data(),
             textures.liquid_data.size());
        same("tex.materialData", textures.material_data.data(),
             ref_textures.material_data.data(), textures.material_data.size());
        same("tex.sunlight", textures.exposed_to_sun.data(),
             ref_textures.exposed_to_sun.data(), textures.exposed_to_sun.size());
      }
      events.Clear();
      // AND THE TOUCH BITMAP WITH THEM, as `ClearFrameEvents` (sim/simdll.cpp) does -- it clears
      // `projection_events` and `ClearSubstanceTouched()` on the same two lines, because a
      // cell's "a substance was written here" flag is a fact about ONE frame that
      // `CopySimDataToGame` drains and the next frame must earn again.
      //
      // WHAT IT COSTS TO LEAVE OUT. `Project` reads the bitmap per cell and publishes a
      // `SubstanceChangeInfo` for every set bit, so without the clear the projection
      // republishes the CUMULATIVE set of cells ever touched, every frame, for the whole
      // run. On a settled asteroid that is 39.4 % of the grid where the real sim publishes
      // 10.7 % -- a saturating number that looked like a steady state and was an
      // accumulator. Measured: `Project`'s census changes 39.43 % -> 10.68 % and its time
      // 1.392 -> 1.334 ms min. The timing barely moves; the COUNT is wrong by a factor of
      // four, and the count is what `census.*` asserts. "Project changes the same 39 % however
      // long you settle" is exactly the shape of steady-state evidence that turns out to be a
      // counter nobody reset.
      world.ClearSubstanceTouched();
    });
    if (measuring) r.frame.Add(frame_ms);
  }
  const size_t alloc_steady = g_alloc_count - alloc_after_first_tick;
  const size_t alloc_first = alloc_after_first_tick - alloc_at_start;
  r.after = Describe(world, elements);

  const size_t cells = world.GameCount();
  char settle_note[64] = "";
  if (g_settle > 0) snprintf(settle_note, sizeof(settle_note), "  settle=%d", g_settle);
  printf("\n=== %s  %dx%d = %zu cells  regions=%s%s  %d ticks @ dt=%.2f (%zu substeps) ===\n",
         label, seed.width, seed.height, cells, region_mode, settle_note, ticks, dt,
         substeps_run);
  printf("  composition  solid %zu  liquid %zu  gas %zu  vacuum %zu  (massless %zu)"
         "  ->  solid %zu  liquid %zu  gas %zu  vacuum %zu\n",
         r.before.solid, r.before.liquid, r.before.gas, r.before.vacuum, r.before.massless,
         r.after.solid, r.after.liquid, r.after.gas, r.after.vacuum);
  // The settled composition, when there is one, because the line above reads "seed ->
  // finish" and with a settle in the middle that hides the whole point: the measured ticks
  // start from the middle column, and how far it sits from the left one is how much of the
  // run was transient that nothing measured.
  if (g_settle > 0) {
    printf("  settled      solid %zu  liquid %zu  gas %zu  vacuum %zu"
           "   <- the state the %d measured ticks start from, after %d untimed ones\n",
           settled.solid, settled.liquid, settled.gas, settled.vacuum, ticks, g_settle);
  }
  // Printed whatever the setting, including "none": a table with two building rows in it
  // and no statement of how many buildings produced them is the shape of number that gets
  // quoted out of context later.
  if (buildings.exchange.Data().size() == 0) {
    printf("  buildings    none registered -- the two building kernels below are an "
           "iteration over an empty vector\n");
    // Asked for a population and got none: the recorded base does not fit on this grid, and
    // every timing below is the empty-vector one under a label that claims otherwise. Says so
    // rather than leaving the line above to be read as "--buildings off".
    if (buildings_wanted > 0) {
      printf("  buildings    ** %zu ASKED FOR, ALL %zu DROPPED OFF-GRID ** -- the recorded "
             "population needs a grid at least as large as its bounding box\n",
             buildings_wanted, buildings_dropped);
    }
  } else {
    size_t covered = 0;
    for (const BuildingHeatExchangeData& d : buildings.exchange.Data()) {
      covered += static_cast<size_t>(d.CellCount());
    }
    printf("  buildings    %zu registered (%zu asked for, %zu dropped off-grid), %zu cells "
           "covered, %.2f cells each, %zu placement%s\n",
           buildings.exchange.Data().size(), buildings_wanted, buildings_dropped, covered,
           static_cast<double>(covered) / static_cast<double>(buildings.exchange.Data().size()),
           building_tiles, building_tiles == 1 ? "" : "s");
  }
  // Printed whatever the setting, for the reason the buildings line is: a table with a
  // `StepFields` row in it and no statement of what was registered is a number nobody can
  // put back in context later.
  if (!fields.Active()) {
    printf("  fields       none registered -- the StepFields row below, if the flags asked "
           "for one, is the gate returning on its first line\n");
  } else {
    printf("  fields       %zu registered over %zu propert%s, %d emitter%s\n",
           fields.rules.size(), fields.properties.size(),
           fields.properties.size() == 1 ? "y" : "ies", g_radiation,
           g_radiation == 1 ? "" : "s");
    // WHAT EACH ONE LEFT BEHIND, and the assertion that goes with it. A timing taken over a
    // field that wrote into no cell at all is a timing of an early-out wearing the label of
    // a solver -- exactly the failure `--buildings` found when a whole recorded population
    // dropped off-grid and every building row kept its name. So a non-idle field that left
    // the world with no nonzero cell fails the run rather than printing a small number.
    for (size_t i = 0; i < fields.rules.size() && i < fields.properties.size(); ++i) {
      const float* values = world.ExtCells().F32(fields.properties[i]);
      size_t nonzero = 0, moved = 0;
      float peak = 0.0f;
      if (values != nullptr) {
        for (const CellWalk cw : world.GameCells()) {
          const float v = values[cw.padded];
          if (v != 0.0f) ++nonzero;
          if (v > peak) peak = v;
          if (i < fields.seeded.size() && cw.padded < fields.seeded[i].size() &&
              memcmp(&fields.seeded[i][cw.padded], &v, sizeof(float)) != 0) {
            ++moved;
          }
        }
      }
      printf("  fields       %-34s %8zu moved, %zu nonzero, peak %.6f\n", fields.rules[i],
             moved, nonzero, static_cast<double>(peak));
      // THE TWO HALVES OF THE SAME ASSERTION, and both are needed. A non-idle field that
      // moved no cell has a timing that measures an early-out under the label of a rule --
      // the `--buildings` failure again, where a whole population dropped off-grid and every
      // building row kept its name. An IDLE field that moved one has broken the gate
      // the field solver's cost argument rests on: a registered field
      // nobody is feeding must cost nothing AND do nothing.
      if (!fields.idle[i] && moved == 0) {
        printf("  FIELDS FAIL: \"%s\" is not idle and left every cell exactly as it found "
               "it. Its timing\n"
               "  FIELDS FAIL: below measures an early-out, not the rule it is labelled "
               "with.\n",
               fields.rules[i]);
        ++g_golden_fails;
      }
      if (fields.idle[i] && moved != 0) {
        printf("  FIELDS FAIL: \"%s\" is IDLE and moved %zu cells. `Field::Idle()` is the "
               "gate the whole\n"
               "  FIELDS FAIL: cost argument rests on, and a field nobody feeds must write "
               "nothing.\n",
               fields.rules[i], moved);
        ++g_golden_fails;
      }
      // And the saturation note: a field pinned at its clamp has stopped being a measurement
      // of anything that moves. Reported, not failed -- saturation is a property of the
      // strength and the tick count, and both of those are dials.
      if (!fields.idle[i] && peak >= 1.0f) {
        printf("  fields       ** \"%s\" reached its clamp; its cells stop moving from here "
               "and `changed` understates the work **\n",
               fields.rules[i]);
      }
    }
  }

  const double total = r.substep.Total() + r.project.Total() + r.textures.Total();
  printf("  %-22s %8s %9s %9s %9s %9s %8s\n", "kernel", "calls", "min ms", "med ms",
         "p95 ms", "max ms", "share");
  auto row = [&](const Samples& s, bool share) {
    printf("  %-22s %8zu %9.3f %9.3f %9.3f %9.3f %7.1f%%\n", s.name, s.ms.size(), s.Min(),
           s.Quantile(0.5), s.Quantile(0.95), s.Max(),
           share && total > 0 ? 100.0 * s.Total() / total : 0.0);
  };
  row(r.conduction, true);
  row(r.state, true);
  row(r.sublimation, true);
  row(r.pressure, true);
  row(r.displace, true);
  row(r.flow, true);
  row(r.liqdisp, true);
  row(r.bldheat, true);
  row(r.bldbld, true);
  row(r.zeromassless, true);
  // Only when they ran. A zero row is not the same statement as an absent one: the gated
  // call really does cost about nothing, and printing "0.000" for a phase no world in this
  // run had switched on would read as a measurement of the phase rather than of the gate.
  // The gate's own price is the `--fields off` / `--radiation 0` reading of these same three
  // buckets, which is why they are summed into the substep whatever the flags say.
  if (g_radiation_on) {
    row(r.radfield, true);
    row(r.rademit, true);
  }
  if (fields.Active()) row(r.fields, true);
  row(r.project, true);
  row(r.textures, true);
  if (g_sunlight) {
    row(r.beam90, false);
    row(r.beam60, false);
    row(r.beam30, false);
    row(r.beam15, false);
  }
  printf("  %-22s %8s %9s %9s %9s %9s\n", "-", "", "", "", "", "");
  row(r.substep, false);
  row(r.frame, false);

  // The budget: the DLL runs on the calling thread, so a substep's cost
  // lands inside the game's frame. At 1x a 0.2 s substep fires about every twelfth frame;
  // the constraint is that when it fires it fits inside one 16.7 ms frame.
  const double worst = r.frame.Max();
  printf("  worst frame %.3f ms against a 16.67 ms budget (%.1f%%)%s\n", worst,
         100.0 * worst / 16.67, worst > 16.67 ? "  ** OVER **" : "");
  printf("  cost per substep per 1000 cells: %.4f ms (median)\n",
         cells ? r.substep.Quantile(0.5) * 1000.0 / cells : 0.0);
  // NOT "zero", and the difference is a measurement rather than a compromise.
  //
  // Zero is right for `granite`, which is settled: nothing moves, so nothing's working set
  // grows and it allocates exactly nothing after its first tick. It is WRONG as a universal
  // claim, because the asteroid's gas is still expanding into vacuum (29,532 gas cells at the
  // start, 34,750 at tick 50), so `flow_touched_` and the projection's event lists genuinely
  // get bigger and a geometrically-grown vector must reallocate O(log n) times to follow them.
  //
  // What separates a growing buffer from a CHURNING one is how the count scales with the run.
  // Measured on the asteroid after the `SnapshotFlowElements` fix: 7 allocations over 49
  // ticks, 9 over 99, 9 over 199 -- flat. Before it: one per frame, so 49, 99 and 199. A
  // quarter of the tick count sits between those two regimes with an order of magnitude of
  // room on both sides, and it is a bound the churn cannot pass at any run length.
  //
  // Below 40 ticks the first tick's warm-up dominates and the ratio says nothing, so it is
  // reported and not asserted -- the same rule as a non-canonical golden.
  const bool alloc_checkable = ticks >= 40;
  const size_t alloc_budget = static_cast<size_t>(ticks) / 4;
  const bool alloc_ok = !alloc_checkable || alloc_steady <= alloc_budget;
  printf("  allocations: %zu on the first tick (%s), "
         "%zu over the %d after%s\n",
         alloc_first,
         // With a settle in front of it the first MEASURED tick is not the first tick the
         // buffers ever saw, so the usual explanation is a lie and the number should be
         // near zero. Saying which regime this is beats leaving a 0 to be read as a fault.
         g_settle > 0 ? "already warm -- the settle sized every buffer"
                      : "scratch and event lists reaching size",
         alloc_steady, ticks - 1,
         alloc_steady == 0
             ? "  -- steady state allocates nothing"
             : (!alloc_checkable ? "  (too few ticks to judge)"
                                 : (alloc_ok ? "  -- growth, not churn" : "  ** CHURNING **")));
  if (!alloc_ok) {
    printf("  ALLOC FAIL: %zu allocations over %d steady ticks, budget %zu. That is about one\n"
           "  ALLOC FAIL: per frame, which is a buffer being rebuilt rather than one growing.\n",
           alloc_steady, ticks - 1, alloc_budget);
    ++g_alloc_fails;
  }
  ReportCensus(ticks);
  if (g_run_json) {
    size_t covered = 0;
    for (const BuildingHeatExchangeData& d : buildings.exchange.Data()) {
      covered += static_cast<size_t>(d.CellCount());
    }
    WriteRunJson(g_run_json, label, seed, region_mode, ticks, dt,
                 buildings.exchange.Data().size(), covered, r, alloc_first, alloc_steady);
  }
  const char* noncanonical =
      CensusNonCanonical(region_mode, seed.width, seed.height, ticks, dt);
  char keyed[64];
  snprintf(keyed, sizeof(keyed), "%s%s%s%s%s%s", scenario_key, RegionKeySuffix(region_mode),
           BuildingKeySuffix(), SettleKeySuffix(), FieldKeySuffix(), RadiationKeySuffix());
  if (!CheckDigestGolden("census", keyed, CensusDigest(), noncanonical,
                         "A kernel changed how many cells it examines, skips or changes. "
                         "The census table printed above is the diff.")) {
    ++g_golden_fails;
  }
  if (!CheckDigestGolden("state", keyed, WorldStateDigest(world), noncanonical,
                         "The sim left the world in a different state -- some cell's element, "
                         "mass or temperature moved. This is a PHYSICS change, not a "
                         "bookkeeping one.")) {
    ++g_golden_fails;
  }
  // The field values, and only when there are some. `state.*` is element, mass and
  // temperature; a field writes none of the three, so without this key a change to any walk
  // in sim/fields.h would move nothing in this suite at all.
  if (fields.Active() &&
      !CheckDigestGolden("field", keyed, FieldValueDigest(world, fields), noncanonical,
                         "The field solver wrote different values. Some walk, attenuation "
                         "law, combine mode or clamp in sim/fields.h moved -- gastest's two "
                         "oracles say whether it is still RadiationAbsorptionAlongLine and "
                         "ComputeSunBeam; this key says it changed at all.")) {
    ++g_golden_fails;
  }
  const simcheck::Counts check = SimCheckCounts(world, elements);
  printf("\n  Klei's SimCheckErrorMap, over the world this run left behind:\n");
  check.Print("simcheck");
  for (int i = 0; i < simcheck::kClassCount; ++i) {
    const simcheck::Class cl = static_cast<simcheck::Class>(i);
    if (check.n[i]) printf("    %-8s %s\n", simcheck::Name(cl), simcheck::Meaning(cl));
  }
  // Yellow and blue are the two classes no world this project builds has any business
  // showing: a vacuum cell holding a temperature, or holding mass. They are a hard fail
  // rather than a pinned count, because unlike red they are not lit by the world border.
  if (check.VacuumFaults() != 0) {
    printf("  SIMCHECK FAIL: %llu vacuum cells hold a temperature or mass (yellow %llu, "
           "blue %llu).\n",
           static_cast<unsigned long long>(check.VacuumFaults()),
           static_cast<unsigned long long>(check.n[simcheck::kYellow]),
           static_cast<unsigned long long>(check.n[simcheck::kBlue]));
    ++g_golden_fails;
  }
  goldens::Md5 check_md5;
  const std::string check_line = check.Line();
  check_md5.Update(reinterpret_cast<const uint8_t*>(check_line.data()), check_line.size());
  if (!CheckDigestGolden("simcheck", keyed, check_md5.HexDigest(), noncanonical,
                         "Klei's own cell validator classifies this world differently. A red "
                         "that was not there is a NaN, a cell over 10 t or 10 000 K, or one "
                         "under 10 K; magenta and cyan are cells sitting past a phase "
                         "threshold. The counts printed above are the diff.")) {
    ++g_golden_fails;
  }
}

// The scratch copies both sweep kernels take at the top of every substep, timed on their
// own. They are not a guess at where the time goes — they are the exact operations
// `StepConduction` and `StepFlow` perform, on a buffer of the same size, so the numbers
// subtract cleanly from the kernel totals above.
//
// `StepFlow` copies the whole cell array (12 B/cell) and clears a byte per cell;
// `StepConduction` copies temperatures alone (4 B/cell).
void ProbeCopies(size_t cells, int reps) {
  std::vector<PhaseEntry> src(cells), dst;
  std::vector<float> temps(cells), tdst;
  std::vector<uint8_t> flags;
  Samples phase_copy("StepFlow start.assign"), temp_copy("StepConduction start gather"),
      temp_soa("  the same gather, packed"), clear("StepFlow swapped.assign");
  tdst.resize(cells);
  for (int i = 0; i < reps; ++i) {
    phase_copy.Add(TimeIt([&] { dst.assign(src.begin(), src.end()); }));
    // WHAT `StepConduction` ACTUALLY DOES: read one float out of each 12-byte PhaseEntry.
    // This used to read the packed `temps` array below instead, and so measured the thing
    // the kernel does NOT do -- it was named for the gather and priced the alternative.
    temp_copy.Add(TimeIt([&] {
      for (size_t k = 0; k < cells; ++k) tdst[k] = src[k].temperature;
    }));
    // The same gather if temperature were its own array -- i.e. the whole of what a
    // struct-of-arrays layout could buy on the one field-sparse pass in the sim. The gap
    // between these two rows is the entire prize, and it is reported next to the kernel
    // timings so nobody has to guess at it again.
    temp_soa.Add(TimeIt([&] {
      for (size_t k = 0; k < cells; ++k) tdst[k] = temps[k];
    }));
    clear.Add(TimeIt([&] { flags.assign(cells, 0); }));
  }
  printf("\n--- scratch-buffer probe (%zu cells, %d reps) ---\n", cells, reps);
  printf("  %-30s med %7.3f ms  max %7.3f ms  (%zu KB)\n", phase_copy.name,
         phase_copy.Quantile(0.5), phase_copy.Max(), cells * sizeof(PhaseEntry) / 1024);
  printf("  %-30s med %7.3f ms  max %7.3f ms  (%zu KB read as 12 B stride)\n",
         temp_copy.name, temp_copy.Quantile(0.5), temp_copy.Max(),
         cells * sizeof(PhaseEntry) / 1024);
  printf("  %-30s med %7.3f ms  max %7.3f ms  (%zu KB)\n", temp_soa.name,
         temp_soa.Quantile(0.5), temp_soa.Max(), cells * sizeof(float) / 1024);
  printf("  %-30s med %7.3f ms  max %7.3f ms  (%zu KB)\n", clear.name, clear.Quantile(0.5),
         clear.Max(), cells / 1024);
}

}  // namespace

int main(int argc, char** argv) {
  setvbuf(stdout, nullptr, _IONBF, 0);

  const char* corpus = nullptr;
  const char* scenario = "all";
  const char* regions = "clamped";
  const char* buildings_arg = "off";
  const char* settle_arg = "0";
  const char* fields_arg = "off";
  const char* radiation_arg = nullptr;
  int width = 256, height = 384, ticks = 50;
  float dt = 0.2f;
  for (int i = 1; i < argc; ++i) {
    if (!strcmp(argv[i], "--corpus") && i + 1 < argc) corpus = argv[++i];
    else if (!strcmp(argv[i], "--scenario") && i + 1 < argc) scenario = argv[++i];
    else if (!strcmp(argv[i], "--regions") && i + 1 < argc) regions = argv[++i];
    else if (!strcmp(argv[i], "--verify")) g_verify = true;
    else if (!strcmp(argv[i], "--sunlight")) g_sunlight = true;
    else if (!strcmp(argv[i], "--width") && i + 1 < argc) width = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--height") && i + 1 < argc) height = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--ticks") && i + 1 < argc) ticks = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--dt") && i + 1 < argc) dt = static_cast<float>(atof(argv[++i]));
    // `--census-json` is an older name for the same flag, kept as an alias: it names the same
    // file in the same format.
    else if ((!strcmp(argv[i], "--json") || !strcmp(argv[i], "--census-json")) && i + 1 < argc)
      g_run_json = argv[++i];
    else if (!strcmp(argv[i], "--record-goldens")) g_record_goldens = true;
    else if (!strcmp(argv[i], "--buildings") && i + 1 < argc) buildings_arg = argv[++i];
    else if (!strcmp(argv[i], "--fields") && i + 1 < argc) fields_arg = argv[++i];
    else if (!strcmp(argv[i], "--radiation") && i + 1 < argc) radiation_arg = argv[++i];
    else if (!strcmp(argv[i], "--settle") && i + 1 < argc) settle_arg = argv[++i];
  }
  if (!corpus) {
    printf("usage: bench.exe --corpus <corpus.bin> [--scenario asteroid|granite|all]\n"
           "       [--width W --height H] [--ticks N] [--dt SECONDS]\n"
           "       [--regions full|clamped|quarter|cluster:N] [--verify]\n"
           "       [--buildings off|corpus|N]  register a building population (default off)\n"
           "       [--fields <mode>]        off|idle|decay|point|beam|element|all (default\n"
           "                                off): one extension property per rule, each with\n"
           "                                a field on it\n"
           "       [--radiation N]          set SimData::radiationEnabled and register N\n"
           "                                constant emitters. Omit it for off; `0` sets the\n"
           "                                byte and registers nobody, which prices the\n"
           "                                radiation field pass on its own\n"
           "       [--settle N]             run N untimed ticks first, so the measured ones\n"
           "                                describe a settled world and not the transient\n"
           "       [--json <path>]          append one machine-readable record per scenario\n"
           "                                (per-kernel timings AND counts, with provenance)\n"
           "       [--record-goldens]       rewrite this suite's keys in driver/GOLDENS.txt\n");
    return 2;
  }

  g_corpus_path = corpus;
  Tables tables;
  if (!oni_bench::LoadTables(corpus, &tables)) return 1;
  printf("element table: %d elements\n", tables.count);

  if (strcmp(buildings_arg, "off") != 0) {
    // `atoi` answers 0 for "corpuss", "1e4" and "lots" alike, and a run that silently
    // registered nothing while its label claimed a population is the one outcome this whole
    // scenario exists to remove. Checked here rather than at the use site so it fails before
    // a single tick runs.
    if (strcmp(buildings_arg, "corpus") != 0) {
      const char* p = buildings_arg;
      for (; *p; ++p) {
        if (*p < '0' || *p > '9') break;
      }
      if (p == buildings_arg || *p != '\0' || atoi(buildings_arg) <= 0) {
        printf("--buildings takes `off`, `corpus`, or a positive count -- not \"%s\"\n",
               buildings_arg);
        return 2;
      }
    }
    g_buildings = buildings_arg;
    // Loud, not silent: a `--buildings` run that quietly registered nothing would report the
    // empty-vector timing under a label that claims a population, which is the one failure
    // mode this whole scenario exists to remove.
    if (!oni_bench::LoadBuildings(corpus, &g_building_pool)) {
      printf("--buildings: this corpus carries no AddBuildingHeatExchange messages\n");
      return 1;
    }
    printf("building pool: %zu recorded from the corpus\n", g_building_pool.size());
  }

  // Same digit check `--buildings` gets, and for the same reason: `atoi` answers 0 for
  // "lots", and a settle that silently did not happen reports the transient under a label
  // that claims a settled world. That is the exact failure this scenario exists to remove.
  {
    const char* p = settle_arg;
    for (; *p; ++p) {
      if (*p < '0' || *p > '9') break;
    }
    if (p == settle_arg || *p != '\0') {
      printf("--settle takes a non-negative tick count -- not \"%s\"\n", settle_arg);
      return 2;
    }
    g_settle = atoi(settle_arg);
  }

  // `--fields`: a misspelled mode is refused rather than silently read as `off`. The whole
  // point of a mode name is that it selects a rule, and a run that selected none while
  // printing a `StepFields` row is the "asked for a population, got none" failure again.
  if (strcmp(fields_arg, "off") != 0) {
    if (!ValidFieldMode(fields_arg)) {
      printf("--fields takes off|idle|decay|point|beam|element|all -- not \"%s\"\n",
             fields_arg);
      return 2;
    }
    g_fields = fields_arg;
  }

  // `--radiation`: the same digit check `--buildings` and `--settle` get, for the same
  // reason. A world with the byte set and no emitter in it is a legitimate configuration --
  // it prices `StepRadiationField` alone, which is the whole-region occlusion and decay
  // pass -- so `--radiation 0` is accepted, sets the byte, and registers nobody.
  if (radiation_arg != nullptr) {
    const char* p = radiation_arg;
    for (; *p; ++p) {
      if (*p < '0' || *p > '9') break;
    }
    if (p == radiation_arg || *p != '\0') {
      printf("--radiation takes a non-negative emitter count -- not \"%s\"\n", radiation_arg);
      return 2;
    }
    g_radiation = atoi(radiation_arg);
    g_radiation_on = true;
  }

  if (!strcmp(scenario, "all") || !strcmp(scenario, "granite")) {
    RunScenario("granite (floor: nothing to simulate)", "granite",
                oni_bench::UniformGranite(tables, width, height), tables, regions, ticks,
                dt);
  }
  if (!strcmp(scenario, "all") || !strcmp(scenario, "asteroid")) {
    RunScenario("asteroid (representative)", "asteroid",
                oni_bench::Asteroid(tables, width, height), tables, regions, ticks, dt);
  }
  ProbeCopies(static_cast<size_t>(width + 2) * (height + 2), ticks);
  if (g_alloc_fails > 0) {
    printf("\nallocations: %d scenario%s allocated once per frame. A steady-state frame works\n"
           "allocations: out of buffers that already exist; a malloc in the tick loop is a\n"
           "allocations: defect even when it is cheap, and it is invisible to every timing.\n",
           g_alloc_fails, g_alloc_fails == 1 ? "" : "s");
    return 1;
  }
  if (g_golden_fails > 0) return 1;
  if (g_census_fails > 0) {
    printf("\ncensus: %d kernel%s stepped over more cells than its declared bound allows\n",
           g_census_fails, g_census_fails == 1 ? "" : "s");
    return 1;
  }
  if (g_verify) {
    printf("\nverify: %d mismatch%s against a full recompute\n", g_verify_fails,
           g_verify_fails == 1 ? "" : "es");
    return g_verify_fails == 0 ? 0 : 1;
  }
  return 0;
}
