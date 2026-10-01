// End-to-end proof that the real message pipeline —
// ext::kInjectGasSpecies -> ApplyInjectGasSpecies -> StepPhysics's mixing branch — actually
// moves gas mass the way gastest.cpp already proved the standalone kernel does. gastest.cpp
// links sim/gas_mixture.h/gas_rooms.h directly and calls MixPair/MixRoomPooled by hand; this
// tool drives ONLY the DLL's own exported ABI, exactly as the real game would, plus one
// addition: SIM_DebugGasMass, a non-Klei debug export (same category as
// SIM_DebugRandomState/SIM_DebugLedger) that reads gas_species_/gas_mass_ back out, since no
// projection/facade publishes them through GameDataUpdate yet.
//
// Loads this repo's own SimDLL.dll by default. There is no Klei baseline for this
// message — this never compares against Klei's DLL, unlike diffsim/experiments.
//
// Usage: vftest.exe --corpus <corpus.bin> [--dll <path>]

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <vector>

#include "../../abi/gas_mixture_abi.h"
#include "../../abi/sim_abi_ext.h"
#include "goldens.h"
#include "simhost.h"

using namespace simhost;
using namespace oni_sim;

namespace {

int g_fail_count = 0;
// Arms that RAN. Asserted against `vftest.checks` in driver/GOLDENS.txt at the end of the
// run: a failing arm already fails the suite on its own, but an arm that stops executing --
// because a guard above it changed, or a fixture path moved -- is invisible to a pass/fail
// count and used to read as success.
long g_check_count = 0;
void Check(bool cond, const char* what) {
  ++g_check_count;
  printf("  %s: %s\n", cond ? "PASS" : "FAIL", what);
  if (!cond) ++g_fail_count;
}

// --dump-blob-dir. The extension save format folds kSaveVersionGasMixture (16) and
// kSaveVersionRoomPromotion (17) into one self-describing section at a new version, and its
// acceptance test is a v16/v17 blob loading through the new reader with identical cell
// contents. AFTER THAT CHANGE NOTHING IN THIS TREE CAN WRITE ONE: `World::ToBlob` is the only
// producer of either version, and the two arms below are the only callers that give it a world
// with something to say. `savefmt --dump` seeds a vanilla world and yields v15.
//
// So the fixtures have to be captured from a build that predates the writer change, and this
// flag is how. Off by default -- a test tool that writes files on every run is a test tool
// somebody turns off.
const char* g_dump_blob_dir = nullptr;
// Where the pinned pre-migration blobs live. See tests/fixtures/README.md for why they are
// files rather than something a test regenerates: nothing in this tree can write a v16 or a
// v17 blob any more, so a test that produced its own input would be checking the new writer
// against the new reader and would pass whatever the migration did to the old format.
const char* g_fixture_dir = "../tests/fixtures";

std::vector<uint8_t> ReadFixture(const char* name) {
  char path[512];
  snprintf(path, sizeof(path), "%s/%s", g_fixture_dir, name);
  std::vector<uint8_t> out;
  FILE* f = fopen(path, "rb");
  if (!f) {
    printf("  FAILED: could not open fixture %s\n", path);
    ++g_fail_count;
    return out;
  }
  fseek(f, 0, SEEK_END);
  const long n = ftell(f);
  fseek(f, 0, SEEK_SET);
  if (n > 0) {
    out.resize(static_cast<size_t>(n));
    if (fread(out.data(), 1, out.size(), f) != out.size()) {
      printf("  FAILED: short read of fixture %s\n", path);
      ++g_fail_count;
      out.clear();
    }
  }
  fclose(f);
  return out;
}

// The filename carries the blob's OWN version, read out of its header rather than taken from
// the call site. That is not cosmetic: stage 2 changed what these arms write from v16/v17 to
// v18, and a fixed stem would have silently overwritten the pinned v16/v17 fixtures -- the
// only copies of those formats that exist -- the first time anyone ran this flag on a current
// build. A name derived from the bytes cannot do that.
void DumpBlob(const char* stem, const std::vector<uint8_t>& blob) {
  if (!g_dump_blob_dir) return;
  int32_t version = -1;
  if (blob.size() > 12) memcpy(&version, blob.data() + 8, 4);
  char path[512];
  snprintf(path, sizeof(path), "%s/%s-v%d.blob", g_dump_blob_dir, stem, version);
  FILE* f = fopen(path, "wb");
  if (!f) {
    printf("  FAILED: could not open %s for writing\n", path);
    ++g_fail_count;
    return;
  }
  const size_t wrote = fwrite(blob.data(), 1, blob.size(), f);
  fclose(f);
  if (wrote != blob.size()) {
    printf("  FAILED: short write to %s (%zu of %zu bytes)\n", path, wrote, blob.size());
    ++g_fail_count;
    return;
  }
  printf("  dumped %zu bytes to %s\n", blob.size(), path);
}

// Bring the sim up for one arm, with the energy ledger's expensive buckets switched on.
//
// Added. Those buckets -- the ones that have to walk a cell's stored energy to
// find out what to charge -- are off by default now, because leaving them on cost 7.6 % of
// the frame in every build including a shipped one (see the note beside
// World::SetEnergyLedgerEnabled in sim/world.h). Several arms below assert
// against those buckets, so every arm has to ask for them, and has to ask BEFORE its first
// tick: a ledger switched on midway has counted part of a run's transfers against all of that
// run's drift, which looks like a conservation bug in the sim rather than the measurement
// error it is.
//
// Resolved per call rather than cached in a global. There are twenty-six arms, not
// twenty-six thousand, and one GetProcAddress apiece is not worth a global to avoid. A DLL
// without the export is a no-op and the arms that read the walked buckets then read zero,
// which is a visible FAIL rather than a silent wrong number.
static void InitSim() {
  sim_initialize(&DefaultMessageHandler);
  auto set = reinterpret_cast<int (*)(int)>(
      GetProcAddress(g_sim, "SIM_DebugSetEnergyLedger"));
  if (set) set(1);
}

float (*sim_debug_gas_mass)(int32_t, int32_t) = nullptr;
int (*sim_debug_energy_ledger)(double*, int) = nullptr;
int (*sim_debug_ledger)(double*, int) = nullptr;
// The two ledgers' published widths, as of the world-environment work. Both arrays are append
// only, so a number that is too small reads a prefix rather than garbage -- but the arms below
// index the LAST fields, so they have to be exact.
constexpr int kEnergyLedgerFields = 39;
constexpr int kMassLedgerFields = 17;
uint16_t (*sim_gas_dominant_element)(int32_t, float*) = nullptr;
int32_t (*sim_debug_room_owned)(int32_t) = nullptr;
int32_t (*sim_room_id)(int32_t) = nullptr;
int32_t (*sim_room_aggregate)(int32_t, OniRoomAggregate*) = nullptr;
int32_t (*sim_ext_property_index)(const char*) = nullptr;
int32_t (*sim_ext_read_cell_property)(int32_t, int32_t, int32_t, uint32_t*) = nullptr;
int32_t (*sim_ext_outstanding_rehydration)(int32_t*, int32_t) = nullptr;
int32_t (*sim_ext_cell_state)(uint8_t*, int32_t) = nullptr;
int32_t (*sim_ext_cell_state_all)(uint8_t*, int32_t) = nullptr;
const ext::ExtPublishedProperty* (*sim_ext_published)(const GameDataUpdate*, int32_t*) = nullptr;
int32_t (*sim_ext_property_count)() = nullptr;
int32_t (*sim_ext_property_describe)(int32_t, ext::ExtCellPropertyDesc*) = nullptr;
int32_t (*sim_ext_stream_index)(const char*) = nullptr;
const ext::ExtPublishedStream* (*sim_ext_events)(const GameDataUpdate*, int32_t*) = nullptr;
int32_t (*sim_ext_stream_count)() = nullptr;
int32_t (*sim_ext_stream_describe)(int32_t, ext::ExtEventStreamDesc*) = nullptr;
int32_t (*sim_ext_phase_count)() = nullptr;
int32_t (*sim_ext_phase_describe)(int32_t, ext::ExtPhaseDesc*) = nullptr;
// The message surface's two exports.
int32_t (*sim_ext_message_count)() = nullptr;
int32_t (*sim_ext_message_describe)(int32_t, ext::ExtMessageDesc*) = nullptr;
// The flow accumulator, published.
int32_t (*sim_debug_cell_flow)(int32_t*, float*, float*, uint16_t*, int32_t) = nullptr;
// The live per-kernel profile.
int32_t (*sim_debug_set_profiler)(int32_t) = nullptr;
int32_t (*sim_debug_profile_summary)(OniProfileSummary*) = nullptr;
int32_t (*sim_debug_profile)(OniProfileSlot*, int32_t) = nullptr;

// What the DLL's own published table says about one message's delivery, by WIRE id. Exists so
// an arm that has just DEMONSTRATED a delivery behaviourally can pin the descriptor to what it
// saw: a table claiming QUEUED next to a same-tick read-back that succeeded is wrong, and this
// suite is the only place that can notice, because gastest checks the table against a written
// opinion and never sends a message at all. Returns -1 if the id has no row.
int32_t DeclaredDelivery(int32_t wireId) {
  ext::ExtMessageDesc md{};
  const int32_t idx = wireId - ext::kExtMessageIdFirst;
  if (!sim_ext_message_describe || sim_ext_message_describe(idx, &md) != 1 || md.id != wireId)
    return -1;
  return md.delivery;
}
int32_t (*sim_ext_attr_index)(const char*) = nullptr;
int32_t (*sim_ext_attr)(int32_t, int32_t, int32_t, uint32_t*) = nullptr;
// The enumeration half of registry 2.
int32_t (*sim_ext_attr_count)() = nullptr;
int32_t (*sim_ext_attr_describe)(int32_t, ext::ExtElementAttributeDesc*) = nullptr;
int32_t (*sim_ext_attr_keys)(int32_t, int32_t*, int32_t) = nullptr;
int32_t (*sim_gas_mass_batch_query)(const int32_t*, int32_t, uint8_t*, uint16_t*,
                                     float*) = nullptr;
float (*sim_gas_pressure)(int32_t) = nullptr;
float (*sim_compute_gas_pressure)(const uint16_t*, const float*, int32_t, float,
                                   float) = nullptr;
float (*sim_calculate_combined_temperature)(float, float, float, float) = nullptr;
// Klei's seven conduit exports, driven directly the way diffsim's kernel pass does,
// plus the three that bind runs, set their policy and read them back.
int (*conduit_add)(float, float, int, int, float, float, int32_t) = nullptr;
int (*conduit_set)(int, float, float, int) = nullptr;
void (*conduit_remove)(int) = nullptr;
void (*conduit_clear)() = nullptr;
void* (*conduit_update)(float, void*) = nullptr;
int32_t (*sim_conduit_bind)(int32_t, const int32_t*, const int32_t*, const int32_t*,
                            const int32_t*, int32_t, float, float) = nullptr;
int32_t (*sim_conduit_policy)(int32_t, int32_t, int32_t, float, float, float) = nullptr;
int32_t (*sim_conduit_trapped_set)(int32_t, int32_t, int32_t, float, float) = nullptr;
int32_t (*sim_conduit_trapped_clear)(int32_t) = nullptr;
int32_t (*sim_conduit_proposals)(OniConduitPhaseProposal*, int32_t) = nullptr;
int32_t (*sim_conduit_aggregate)(int32_t, int32_t, OniConduitNetworkAggregate*,
                                 int32_t) = nullptr;
float (*sim_equalize_single_species_mass)(float, float, float, float, float, float, float,
                                           float) = nullptr;
float (*sim_liquid_volume_from_mass)(float, float) = nullptr;
float (*sim_equalize_liquid_volume_mass)(float, float, float, float, float, float) = nullptr;
float (*sim_adiabatic_fill_temperature)(float, float, float, float, float) = nullptr;
void (*sim_compute_phase_change_step)(float, float, float, float, float, float, float, float,
                                       float*, float*) = nullptr;

// Vacuum and Oxygen SimHashes, matching the constants already established elsewhere in this
// project (world.h's ElementTable::VacuumIndex, driver/src/diffsim.cpp's kOxygen) — reused
// rather than re-derived.
constexpr int32_t kVacuumHash = 758759285;
constexpr int32_t kOxygenHash = -1528777920;

struct World {
  int width = 0, height = 0;
  std::vector<uint16_t> element;
  std::vector<float> mass;
  std::vector<float> temperature;
  // Sim.Cell.Properties per cell, sent in the world payload the way a loaded save sends them.
  // Empty means zero everywhere, which is what most arms send.
  std::vector<uint8_t> properties;
  void Init(int w, int h, uint16_t fill_element, float fill_mass, float temp) {
    width = w;
    height = h;
    const size_t n = static_cast<size_t>(w) * h;
    element.assign(n, fill_element);
    mass.assign(n, fill_mass);
    temperature.assign(n, temp);
  }
  size_t Count() const { return element.size(); }
};

void SendWorld(const World& w) {
  Writer b;
  b.Put<int32_t>(w.width);
  b.Put<int32_t>(w.height);
  b.Put<uint32_t>(12345u);
  b.PutBool(false);  // radiation
  b.PutBool(true);   // headless
  for (size_t i = 0; i < w.Count(); ++i) {
    Cell c{};
    c.elementIdx = w.element[i];
    c.mass = w.mass[i];
    c.temperature = w.temperature[i];
    c.properties = w.properties.empty() ? 0 : w.properties[i];
    c.insulation = 255;
    b.PutRaw(&c, sizeof(Cell));
  }
  for (size_t i = 0; i < w.Count(); ++i) {
    DiseaseCell d{};
    d.diseaseIdx = 0xFF;
    b.PutRaw(&d, sizeof(DiseaseCell));
  }
  for (size_t i = 0; i < w.Count(); ++i) {
    SimBackwall bw{};
    bw.elementIdx = 0;
    b.PutRaw(&bw, sizeof(SimBackwall));
  }
  Send(SimMessageHash::SimData_InitializeFromCells, b);
}

// The extension registry (abi/sim_abi_ext.h, sim/ext_registry.h). Registration is the one
// IMMEDIATE extension message, so the index comes straight back as the return of
// SIM_HandleMessage rather than a frame later.
//
// `name` is copied WITHOUT forcing a terminator: a name that fills all 48 bytes stays
// unterminated on the wire, which is exactly the case the sim has to refuse rather than
// truncate (truncation would register a different on-disk key than the caller asked for).
//
// `arity` and `component` below default here but are REQUIRED on the sim's own Write/Read.
// The asymmetry is deliberate: the production API must not let a caller land on lane 0 without
// saying so, while a test helper with thirty arity-1 call sites reads better without the noise,
// and the arity arms state it explicitly where it matters.
int32_t RegisterProperty(const char* name, int32_t type, int32_t persist, uint32_t def_bits,
                         int32_t arity = 1) {
  ext::RegisterCellPropertyMessage m{};
  const size_t n = strlen(name);
  memcpy(m.name, name, n < sizeof(m.name) ? n : sizeof(m.name));
  m.type = type;
  m.persist = persist;
  m.arity = arity;
  m.defaultBits = def_bits;
  const void* r = sim_handle_message(ext::kRegisterCellProperty, static_cast<int>(sizeof(m)),
                                     reinterpret_cast<const uint8_t*>(&m));
  return r ? *static_cast<const int32_t*>(r) : -999;
}

void SetCellProperty(int32_t cell, int32_t property, uint32_t bits, int32_t component = 0) {
  ext::SetCellPropertyMessage m{};
  m.cellIdx = cell;
  m.propertyIdx = property;
  m.component = component;
  m.valueBits = bits;
  sim_handle_message(ext::kSetCellProperty, static_cast<int>(sizeof(m)),
                     reinterpret_cast<const uint8_t*>(&m));
}

uint32_t BitsOf(float v) {
  uint32_t bits = 0;
  memcpy(&bits, &v, sizeof(bits));
  return bits;
}

// The SEVENTH checkpoint component. Two-call sizing, exactly as
// SIM_DebugRegistryState works: the length depends on what has been registered and written,
// so the first call is the only way to learn it.
std::vector<uint8_t> ExtCheckpoint() {
  const int32_t n = sim_ext_cell_state(nullptr, 0);
  if (n <= 0) return {};
  std::vector<uint8_t> v(static_cast<size_t>(n));
  if (sim_ext_cell_state(v.data(), n) != n) return {};
  return v;
}

// The inspection twin: every registered property, all three classes. Same format, same
// two-call sizing, and NOT a checkpoint -- see SIM_DebugExtCellStateAll.
std::vector<uint8_t> ExtInspectAll() {
  const int32_t n = sim_ext_cell_state_all(nullptr, 0);
  if (n <= 0) return {};
  std::vector<uint8_t> v(static_cast<size_t>(n));
  if (sim_ext_cell_state_all(v.data(), n) != n) return {};
  return v;
}

void SendExtCheckpoint(const std::vector<uint8_t>& blob) {
  sim_handle_message(ext::kSetExtCellState, static_cast<int>(blob.size()), blob.data());
}

// A subscription, so it drains a frame later like every other cell message.
void PublishProperty(int32_t property, bool enable) {
  ext::PublishCellPropertyMessage m{property, enable ? 1 : 0};
  sim_handle_message(ext::kPublishCellProperty, static_cast<int>(sizeof(m)),
                     reinterpret_cast<const uint8_t*>(&m));
}

// Looks a property up in a frame's descriptor table BY NAME, because the table's order is
// registration order and an index into it is not a property index.
const ext::ExtPublishedProperty* FindPublished(const GameDataUpdate* frame, const char* name) {
  int32_t n = 0;
  const ext::ExtPublishedProperty* tab = sim_ext_published(frame, &n);
  if (tab == nullptr) return nullptr;
  for (int32_t i = 0; i < n; ++i) {
    if (strncmp(tab[i].name, name, sizeof(tab[i].name)) == 0) return &tab[i];
  }
  return nullptr;
}

// Queued, so it lands a frame later like the publish subscription above --
// and unlike that one it switches COLLECTION on, not just a copy.
void SubscribeStream(int32_t stream, bool enable) {
  ext::SubscribeEventStreamMessage m{stream, enable ? 1 : 0};
  sim_handle_message(ext::kSubscribeEventStream, static_cast<int>(sizeof(m)),
                     reinterpret_cast<const uint8_t*>(&m));
}

// By name, for the same reason `FindPublished` is: the table holds only SUBSCRIBED streams,
// so an index into it is not a stream index.
const ext::ExtPublishedStream* FindStream(const GameDataUpdate* frame, const char* name) {
  int32_t n = 0;
  const ext::ExtPublishedStream* tab = sim_ext_events(frame, &n);
  if (tab == nullptr) return nullptr;
  for (int32_t i = 0; i < n; ++i) {
    if (strncmp(tab[i].name, name, sizeof(tab[i].name)) == 0) return &tab[i];
  }
  return nullptr;
}

// The refusal records a frame is carrying, decoded. Returns an empty vector for a frame that
// is not carrying the stream at all, which is a different thing from a frame carrying zero
// records -- the arms below distinguish the two through `FindStream` rather than through this.
std::vector<ext::ExtRefusedMessage> Refusals(const GameDataUpdate* frame) {
  const ext::ExtPublishedStream* st = FindStream(frame, ext::kStreamMessageRefused);
  std::vector<ext::ExtRefusedMessage> out;
  if (st == nullptr || st->data == nullptr) return out;
  out.resize(static_cast<size_t>(st->count));
  if (st->count > 0) memcpy(out.data(), st->data, static_cast<size_t>(st->byteCount));
  return out;
}

// IMMEDIATE, unlike every subscription above: the registration returns the
// attribute index straight out of `SIM_HandleMessage`, with no tick of latency to wait out.
int32_t RegisterElementAttribute(const char* name, int32_t type, int32_t arity) {
  ext::RegisterElementAttributeMessage m{};
  strncpy(m.name, name, sizeof(m.name) - 1);
  m.type = type;
  m.arity = arity;
  const void* r = sim_handle_message(ext::kRegisterElementAttribute,
                                     static_cast<int>(sizeof(m)),
                                     reinterpret_cast<const uint8_t*>(&m));
  return r ? *static_cast<const int32_t*>(r) : 0;
}

// A registration whose name deliberately fills the field with no terminator, to reach the
// "refused rather than truncated" path the wire format cannot otherwise exercise.
int32_t RegisterElementAttributeUnterminated() {
  ext::RegisterElementAttributeMessage m{};
  memset(m.name, 'a', sizeof(m.name));
  m.type = ext::kExtF32;
  m.arity = 1;
  const void* r = sim_handle_message(ext::kRegisterElementAttribute,
                                     static_cast<int>(sizeof(m)),
                                     reinterpret_cast<const uint8_t*>(&m));
  return r ? *static_cast<const int32_t*>(r) : 0;
}

void SetElementAttributeBits(int32_t attr, int32_t id_hash, int32_t component, uint32_t bits,
                             bool clear = false) {
  ext::SetElementAttributeMessage m{attr, id_hash, component, bits, clear ? 1 : 0};
  sim_handle_message(ext::kSetElementAttribute, static_cast<int>(sizeof(m)),
                     reinterpret_cast<const uint8_t*>(&m));
}

void SetElementAttributeF32(int32_t attr, int32_t id_hash, float value) {
  uint32_t bits = 0;
  memcpy(&bits, &value, sizeof(bits));
  SetElementAttributeBits(attr, id_hash, 0, bits);
}

// Returns the stored float, or NaN for "unset" -- so an arm can distinguish an override of
// zero from no override at all, which is the whole point of the export's return value.
float ReadElementAttributeF32(int32_t attr, int32_t id_hash, int32_t component = 0) {
  uint32_t bits = 0;
  if (!sim_ext_attr(attr, id_hash, component, &bits)) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  float v = 0.0f;
  memcpy(&v, &bits, sizeof(v));
  return v;
}

bool HasRefusal(const std::vector<ext::ExtRefusedMessage>& v, int32_t id, int32_t reason) {
  for (const ext::ExtRefusedMessage& r : v) {
    if (r.messageId == id && r.reason == reason) return true;
  }
  return false;
}

const GameDataUpdate* Boot(const Tables& t, const World& w) {
  SendRaw(SimMessageHash::Elements_CreateTable, t.elements);
  SendRaw(SimMessageHash::Disease_CreateTable, t.diseases);
  SendWorld(w);
  return static_cast<const GameDataUpdate*>(SendEmpty(SimMessageHash::Start));
}

// `Tick` above sends `maxY = height - 1` because that is what the GAME sends: the world's top
// row is never simulated in a real one (`World::SetActiveRegions`, measured on `toprow`). This
// sends `maxY = height` instead, which the game never does, and it exists for exactly one arm
// -- the top-row arm at the end of this file, which needs the same world stepped both ways so
// that "an 8x1 world publishes no flow" can be pinned to the region's top edge rather than to
// anything else about a one-row world. Do not reach for it anywhere else: a region that
// includes the top row is not a configuration the sim is required to be correct under, and an
// arm that used it by accident would be asserting against a shape Klei never produces.
const GameDataUpdate* TickTopRowIncluded(const World& w, std::vector<uint8_t>* visible) {
  NewGameFrame f{};
  f.elapsedSeconds = 0.2f;
  f.minX = 0;
  f.minY = 0;
  f.maxX = w.width;
  f.maxY = w.height;
  sim_handle_message(static_cast<int32_t>(SimMessageHash::SimFrameManager_NewGameFrame),
                     sizeof(NewGameFrame), reinterpret_cast<const uint8_t*>(&f));
  return static_cast<const GameDataUpdate*>(
      sim_handle_message(static_cast<int32_t>(SimMessageHash::PrepareGameData),
                         static_cast<int>(visible->size()), visible->data()));
}

const GameDataUpdate* Tick(const World& w, std::vector<uint8_t>* visible) {
  NewGameFrame f{};
  f.elapsedSeconds = 0.2f;
  f.minX = 0;
  f.minY = 0;
  f.maxX = w.width;
  f.maxY = w.height - 1;
  sim_handle_message(static_cast<int32_t>(SimMessageHash::SimFrameManager_NewGameFrame),
                     sizeof(NewGameFrame), reinterpret_cast<const uint8_t*>(&f));
  return static_cast<const GameDataUpdate*>(
      sim_handle_message(static_cast<int32_t>(SimMessageHash::PrepareGameData),
                         static_cast<int>(visible->size()), visible->data()));
}

// `Tick` with a sun in the sky. The fifth int of Klei's active-region record is
// `WorldContainer.currentSunlightIntensity` in lux, and nothing in the sim read it before Mod 3
// (`World::RegionSunlight`) -- so every other helper here leaves it at zero, which is midnight,
// which is why every arm written before this one is unaffected by the solar terms.
const GameDataUpdate* TickSunlit(const World& w, std::vector<uint8_t>* visible, float lux) {
  NewGameFrame f{};
  f.elapsedSeconds = 0.2f;
  f.minX = 0;
  f.minY = 0;
  f.maxX = w.width;
  f.maxY = w.height - 1;
  f.currentSunlightIntensity = lux;
  sim_handle_message(static_cast<int32_t>(SimMessageHash::SimFrameManager_NewGameFrame),
                     sizeof(NewGameFrame), reinterpret_cast<const uint8_t*>(&f));
  return static_cast<const GameDataUpdate*>(
      sim_handle_message(static_cast<int32_t>(SimMessageHash::PrepareGameData),
                         static_cast<int>(visible->size()), visible->data()));
}

// One world covering the whole grid, which is what a single-asteroid game sends. Without this
// the sunlight texture stays zero everywhere (`sim/textures.h`: it is per world and there are no
// worlds until this message arrives), and with it every column is lit from the grid's top row
// down. Mod 3's two per-cell terms both key off that texture, so an arm that forgets this
// measures a planet in permanent shadow.
void DefineOneWorld(int width, int height) {
  Writer offsets;
  offsets.Put<int32_t>(1);
  offsets.Put<int32_t>(0);
  offsets.Put<int32_t>(0);
  offsets.Put<int32_t>(width);
  offsets.Put<int32_t>(height);
  Send(SimMessageHash::DefineWorldOffsets, offsets);
}

// The same frame with TWO overlapping regions, sent the way the game sends them: one
// NewGameFrame struct per region in a single message (see diffsim's own Tick). The substep
// body runs once per region, which is what makes a building covered by both interesting.
const GameDataUpdate* TickTwoRegions(const World& w, std::vector<uint8_t>* visible) {
  NewGameFrame frames[2] = {};
  for (int i = 0; i < 2; ++i) {
    frames[i].elapsedSeconds = 0.2f;
    frames[i].minY = 0;
    frames[i].maxY = w.height - 1;
  }
  // Overlapping on purpose, and both wide enough to cover the probe building at x=1..3.
  frames[0].minX = 0;
  frames[0].maxX = w.width;
  frames[1].minX = 0;
  frames[1].maxX = w.width;
  sim_handle_message(static_cast<int32_t>(SimMessageHash::SimFrameManager_NewGameFrame),
                     static_cast<int>(sizeof(frames)),
                     reinterpret_cast<const uint8_t*>(frames));
  return static_cast<const GameDataUpdate*>(
      sim_handle_message(static_cast<int32_t>(SimMessageHash::PrepareGameData),
                         static_cast<int>(visible->size()), visible->data()));
}

void InjectGas(int32_t cellIdx, int32_t speciesIdx, float massKg, float temperatureK) {
  ext::InjectGasSpeciesMessage m{cellIdx, speciesIdx, massKg, temperatureK};
  sim_handle_message(ext::kInjectGasSpecies, static_cast<int>(sizeof(m)),
                     reinterpret_cast<const uint8_t*>(&m));
}

bool Resolve(const Tables& t, int32_t hash, const char* name, uint16_t* out) {
  const int32_t i = t.IndexOf(hash);
  if (i < 0) {
    printf("  FAILED: element %s (hash %d) is not in the table\n", name, hash);
    return false;
  }
  *out = static_cast<uint16_t>(i);
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  const char* dll = "../sim/build/SimDLL.dll";
  const char* corpus = nullptr;
  const char* goldens_path = "GOLDENS.txt";
  bool record_goldens = false;
  const char* noncanonical = nullptr;
  char noncanonical_buf[256];
  for (int i = 1; i < argc; ++i) {
    // Canonicity is a whitelist, so a flag added later is non-canonical until somebody
    // deliberately says otherwise. `--dll` and `--dump-blob-dir` are the two that genuinely
    // change what runs; they are excluded by not being listed rather than by being named.
    // Every branch below consumes its own value with ++i, so this only ever sees flags.
    if (!noncanonical &&
        strcmp(argv[i], "--corpus") != 0 &&
        strcmp(argv[i], "--fixtures") != 0 &&
        strcmp(argv[i], "--goldens") != 0 &&
        strcmp(argv[i], "--record-goldens") != 0) {
      if (i + 1 < argc && argv[i + 1][0] != '-') {
        snprintf(noncanonical_buf, sizeof(noncanonical_buf), "%s %s", argv[i], argv[i + 1]);
      } else {
        snprintf(noncanonical_buf, sizeof(noncanonical_buf), "%s", argv[i]);
      }
      noncanonical = noncanonical_buf;
    }
    if (!strcmp(argv[i], "--goldens") && i + 1 < argc) goldens_path = argv[++i];
    if (!strcmp(argv[i], "--record-goldens")) record_goldens = true;
    if (!strcmp(argv[i], "--dll") && i + 1 < argc) dll = argv[++i];
    if (!strcmp(argv[i], "--corpus") && i + 1 < argc) corpus = argv[++i];
    if (!strcmp(argv[i], "--dump-blob-dir") && i + 1 < argc) g_dump_blob_dir = argv[++i];
    if (!strcmp(argv[i], "--fixtures") && i + 1 < argc) g_fixture_dir = argv[++i];
  }
  if (!corpus) {
    printf("usage: vftest --corpus <corpus.bin> [--dll path] [--dump-blob-dir dir] "
           "[--fixtures dir]\n"
           "       [--goldens P]  reference values (default GOLDENS.txt, i.e. run from\n"
           "                      driver/); the arm count is asserted and a mismatch exits 1\n"
           "       [--record-goldens]  rewrite it instead of asserting it\n");
    return 1;
  }
  if (!Bind(dll)) return 1;
  sim_debug_gas_mass = reinterpret_cast<float (*)(int32_t, int32_t)>(
      GetProcAddress(g_sim, "SIM_DebugGasMass"));
  if (!sim_debug_gas_mass) {
    printf("SIM_DebugGasMass not exported by %s\n", dll);
    return 1;
  }
  sim_gas_dominant_element = reinterpret_cast<uint16_t (*)(int32_t, float*)>(
      GetProcAddress(g_sim, "SIM_GasDominantElement"));
  if (!sim_gas_dominant_element) {
    printf("SIM_GasDominantElement not exported by %s\n", dll);
    return 1;
  }
  sim_debug_room_owned = reinterpret_cast<int32_t (*)(int32_t)>(
      GetProcAddress(g_sim, "SIM_DebugRoomOwned"));
  if (!sim_debug_room_owned) {
    printf("SIM_DebugRoomOwned not exported by %s\n", dll);
    return 1;
  }
  sim_room_id = reinterpret_cast<int32_t (*)(int32_t)>(GetProcAddress(g_sim, "SIM_RoomId"));
  if (!sim_room_id) {
    printf("SIM_RoomId not exported by %s\n", dll);
    return 1;
  }
  sim_room_aggregate = reinterpret_cast<int32_t (*)(int32_t, OniRoomAggregate*)>(
      GetProcAddress(g_sim, "SIM_RoomAggregate"));
  if (!sim_room_aggregate) {
    printf("SIM_RoomAggregate not exported by %s\n", dll);
    return 1;
  }
  sim_ext_property_index = reinterpret_cast<int32_t (*)(const char*)>(
      GetProcAddress(g_sim, "SIM_ExtCellPropertyIndex"));
  if (!sim_ext_property_index) {
    printf("SIM_ExtCellPropertyIndex not exported by %s\n", dll);
    return 1;
  }
  sim_ext_read_cell_property = reinterpret_cast<int32_t (*)(int32_t, int32_t, int32_t, uint32_t*)>(
      GetProcAddress(g_sim, "SIM_ExtReadCellProperty"));
  if (!sim_ext_read_cell_property) {
    printf("SIM_ExtReadCellProperty not exported by %s\n", dll);
    return 1;
  }
  sim_ext_outstanding_rehydration = reinterpret_cast<int32_t (*)(int32_t*, int32_t)>(
      GetProcAddress(g_sim, "SIM_ExtOutstandingRehydration"));
  if (!sim_ext_outstanding_rehydration) {
    printf("SIM_ExtOutstandingRehydration not exported by %s\n", dll);
    return 1;
  }
  sim_ext_cell_state = reinterpret_cast<int32_t (*)(uint8_t*, int32_t)>(
      GetProcAddress(g_sim, "SIM_DebugExtCellState"));
  if (!sim_ext_cell_state) {
    printf("SIM_DebugExtCellState not exported by %s\n", dll);
    return 1;
  }
  sim_ext_cell_state_all = reinterpret_cast<int32_t (*)(uint8_t*, int32_t)>(
      GetProcAddress(g_sim, "SIM_DebugExtCellStateAll"));
  if (!sim_ext_cell_state_all) {
    printf("SIM_DebugExtCellStateAll not exported by %s\n", dll);
    return 1;
  }
  sim_ext_published = reinterpret_cast<const ext::ExtPublishedProperty* (*)(
      const GameDataUpdate*, int32_t*)>(
      GetProcAddress(g_sim, "SIM_ExtPublishedProperties"));
  if (!sim_ext_published) {
    printf("SIM_ExtPublishedProperties not exported by %s\n", dll);
    return 1;
  }
  sim_ext_property_count = reinterpret_cast<int32_t (*)()>(
      GetProcAddress(g_sim, "SIM_ExtCellPropertyCount"));
  if (!sim_ext_property_count) {
    printf("SIM_ExtCellPropertyCount not exported by %s\n", dll);
    return 1;
  }
  sim_ext_property_describe = reinterpret_cast<int32_t (*)(int32_t, ext::ExtCellPropertyDesc*)>(
      GetProcAddress(g_sim, "SIM_ExtCellPropertyDescribe"));
  if (!sim_ext_property_describe) {
    printf("SIM_ExtCellPropertyDescribe not exported by %s\n", dll);
    return 1;
  }
  sim_ext_stream_index = reinterpret_cast<int32_t (*)(const char*)>(
      GetProcAddress(g_sim, "SIM_ExtEventStreamIndex"));
  if (!sim_ext_stream_index) {
    printf("SIM_ExtEventStreamIndex not exported by %s\n", dll);
    return 1;
  }
  sim_ext_events = reinterpret_cast<const ext::ExtPublishedStream* (*)(
      const GameDataUpdate*, int32_t*)>(GetProcAddress(g_sim, "SIM_ExtPublishedEvents"));
  if (!sim_ext_events) {
    printf("SIM_ExtPublishedEvents not exported by %s\n", dll);
    return 1;
  }
  sim_ext_stream_count = reinterpret_cast<int32_t (*)()>(
      GetProcAddress(g_sim, "SIM_ExtEventStreamCount"));
  if (!sim_ext_stream_count) {
    printf("SIM_ExtEventStreamCount not exported by %s\n", dll);
    return 1;
  }
  sim_ext_stream_describe = reinterpret_cast<int32_t (*)(int32_t, ext::ExtEventStreamDesc*)>(
      GetProcAddress(g_sim, "SIM_ExtEventStreamDescribe"));
  if (!sim_ext_stream_describe) {
    printf("SIM_ExtEventStreamDescribe not exported by %s\n", dll);
    return 1;
  }
  sim_ext_phase_count = reinterpret_cast<int32_t (*)()>(
      GetProcAddress(g_sim, "SIM_ExtPhaseCount"));
  if (!sim_ext_phase_count) {
    printf("SIM_ExtPhaseCount not exported by %s\n", dll);
    return 1;
  }
  sim_ext_phase_describe = reinterpret_cast<int32_t (*)(int32_t, ext::ExtPhaseDesc*)>(
      GetProcAddress(g_sim, "SIM_ExtPhaseDescribe"));
  if (!sim_ext_phase_describe) {
    printf("SIM_ExtPhaseDescribe not exported by %s\n", dll);
    return 1;
  }
  sim_debug_cell_flow =
      reinterpret_cast<int32_t (*)(int32_t*, float*, float*, uint16_t*, int32_t)>(
          GetProcAddress(g_sim, "SIM_DebugCellFlow"));
  if (!sim_debug_cell_flow) {
    printf("SIM_DebugCellFlow not exported by %s\n", dll);
    return 1;
  }
  sim_debug_set_profiler = reinterpret_cast<int32_t (*)(int32_t)>(
      GetProcAddress(g_sim, "SIM_DebugSetProfiler"));
  if (!sim_debug_set_profiler) {
    printf("SIM_DebugSetProfiler not exported by %s\n", dll);
    return 1;
  }
  sim_debug_profile_summary = reinterpret_cast<int32_t (*)(OniProfileSummary*)>(
      GetProcAddress(g_sim, "SIM_DebugProfileSummary"));
  if (!sim_debug_profile_summary) {
    printf("SIM_DebugProfileSummary not exported by %s\n", dll);
    return 1;
  }
  sim_debug_profile = reinterpret_cast<int32_t (*)(OniProfileSlot*, int32_t)>(
      GetProcAddress(g_sim, "SIM_DebugProfile"));
  if (!sim_debug_profile) {
    printf("SIM_DebugProfile not exported by %s\n", dll);
    return 1;
  }
  sim_ext_message_count = reinterpret_cast<int32_t (*)()>(
      GetProcAddress(g_sim, "SIM_ExtMessageCount"));
  if (!sim_ext_message_count) {
    printf("SIM_ExtMessageCount not exported by %s\n", dll);
    return 1;
  }
  sim_ext_message_describe = reinterpret_cast<int32_t (*)(int32_t, ext::ExtMessageDesc*)>(
      GetProcAddress(g_sim, "SIM_ExtMessageDescribe"));
  if (!sim_ext_message_describe) {
    printf("SIM_ExtMessageDescribe not exported by %s\n", dll);
    return 1;
  }
  sim_ext_attr_index = reinterpret_cast<int32_t (*)(const char*)>(
      GetProcAddress(g_sim, "SIM_ExtElementAttributeIndex"));
  if (!sim_ext_attr_index) {
    printf("SIM_ExtElementAttributeIndex not exported by %s\n", dll);
    return 1;
  }
  sim_ext_attr = reinterpret_cast<int32_t (*)(int32_t, int32_t, int32_t, uint32_t*)>(
      GetProcAddress(g_sim, "SIM_ExtElementAttribute"));
  if (!sim_ext_attr) {
    printf("SIM_ExtElementAttribute not exported by %s\n", dll);
    return 1;
  }
  sim_ext_attr_count = reinterpret_cast<int32_t (*)()>(
      GetProcAddress(g_sim, "SIM_ExtElementAttributeCount"));
  if (!sim_ext_attr_count) {
    printf("SIM_ExtElementAttributeCount not exported by %s\n", dll);
    return 1;
  }
  sim_ext_attr_describe = reinterpret_cast<int32_t (*)(int32_t, ext::ExtElementAttributeDesc*)>(
      GetProcAddress(g_sim, "SIM_ExtElementAttributeDescribe"));
  if (!sim_ext_attr_describe) {
    printf("SIM_ExtElementAttributeDescribe not exported by %s\n", dll);
    return 1;
  }
  sim_ext_attr_keys = reinterpret_cast<int32_t (*)(int32_t, int32_t*, int32_t)>(
      GetProcAddress(g_sim, "SIM_ExtElementAttributeKeys"));
  if (!sim_ext_attr_keys) {
    printf("SIM_ExtElementAttributeKeys not exported by %s\n", dll);
    return 1;
  }
  sim_gas_mass_batch_query = reinterpret_cast<int32_t (*)(
      const int32_t*, int32_t, uint8_t*, uint16_t*, float*)>(
      GetProcAddress(g_sim, "SIM_GasMassBatchQuery"));
  if (!sim_gas_mass_batch_query) {
    printf("SIM_GasMassBatchQuery not exported by %s\n", dll);
    return 1;
  }
  sim_gas_pressure = reinterpret_cast<float (*)(int32_t)>(
      GetProcAddress(g_sim, "SIM_GasPressure"));
  if (!sim_gas_pressure) {
    printf("SIM_GasPressure not exported by %s\n", dll);
    return 1;
  }
  sim_compute_gas_pressure = reinterpret_cast<float (*)(
      const uint16_t*, const float*, int32_t, float, float)>(
      GetProcAddress(g_sim, "SIM_ComputeGasPressure"));
  if (!sim_compute_gas_pressure) {
    printf("SIM_ComputeGasPressure not exported by %s\n", dll);
    return 1;
  }
  conduit_add = reinterpret_cast<int (*)(float, float, int, int, float, float, int32_t)>(
      GetProcAddress(g_sim, "ConduitTemperatureManager_Add"));
  conduit_set = reinterpret_cast<int (*)(int, float, float, int)>(
      GetProcAddress(g_sim, "ConduitTemperatureManager_Set"));
  conduit_remove = reinterpret_cast<void (*)(int)>(
      GetProcAddress(g_sim, "ConduitTemperatureManager_Remove"));
  conduit_clear = reinterpret_cast<void (*)()>(
      GetProcAddress(g_sim, "ConduitTemperatureManager_Clear"));
  conduit_update = reinterpret_cast<void* (*)(float, void*)>(
      GetProcAddress(g_sim, "ConduitTemperatureManager_Update"));
  if (!conduit_add || !conduit_set || !conduit_remove || !conduit_clear || !conduit_update) {
    printf("a ConduitTemperatureManager_* export is missing from %s\n", dll);
    return 1;
  }
  sim_conduit_bind = reinterpret_cast<int32_t (*)(int32_t, const int32_t*, const int32_t*,
                                                  const int32_t*, const int32_t*, int32_t, float,
                                                  float)>(
      GetProcAddress(g_sim, "SIM_ConduitNetworkBind"));
  sim_conduit_policy = reinterpret_cast<int32_t (*)(int32_t, int32_t, int32_t, float, float,
                                                    float)>(
      GetProcAddress(g_sim, "SIM_ConduitNetworkPolicy"));
  sim_conduit_trapped_set = reinterpret_cast<int32_t (*)(int32_t, int32_t, int32_t, float, float)>(
      GetProcAddress(g_sim, "SIM_ConduitTrappedSet"));
  sim_conduit_trapped_clear = reinterpret_cast<int32_t (*)(int32_t)>(
      GetProcAddress(g_sim, "SIM_ConduitTrappedClear"));
  sim_conduit_proposals = reinterpret_cast<int32_t (*)(OniConduitPhaseProposal*, int32_t)>(
      GetProcAddress(g_sim, "SIM_ConduitPhaseProposals"));
  if (!sim_conduit_trapped_set || !sim_conduit_trapped_clear || !sim_conduit_proposals) {
    printf("a SIM_ConduitTrapped* / SIM_ConduitPhaseProposals export is missing from %s\n", dll);
    return 1;
  }
  sim_conduit_aggregate = reinterpret_cast<int32_t (*)(int32_t, int32_t,
                                                       OniConduitNetworkAggregate*, int32_t)>(
      GetProcAddress(g_sim, "SIM_ConduitNetworkAggregate"));
  if (!sim_conduit_bind || !sim_conduit_policy || !sim_conduit_aggregate) {
    printf("a SIM_ConduitNetwork* export is missing from %s\n", dll);
    return 1;
  }
  sim_calculate_combined_temperature = reinterpret_cast<float (*)(float, float, float, float)>(
      GetProcAddress(g_sim, "SIM_CalculateCombinedTemperature"));
  if (!sim_calculate_combined_temperature) {
    printf("SIM_CalculateCombinedTemperature not exported by %s\n", dll);
    return 1;
  }
  sim_equalize_single_species_mass = reinterpret_cast<float (*)(
      float, float, float, float, float, float, float, float)>(
      GetProcAddress(g_sim, "SIM_EqualizeSingleSpeciesMass"));
  if (!sim_equalize_single_species_mass) {
    printf("SIM_EqualizeSingleSpeciesMass not exported by %s\n", dll);
    return 1;
  }
  sim_liquid_volume_from_mass = reinterpret_cast<float (*)(float, float)>(
      GetProcAddress(g_sim, "SIM_LiquidVolumeFromMass"));
  if (!sim_liquid_volume_from_mass) {
    printf("SIM_LiquidVolumeFromMass not exported by %s\n", dll);
    return 1;
  }
  sim_equalize_liquid_volume_mass = reinterpret_cast<float (*)(
      float, float, float, float, float, float)>(
      GetProcAddress(g_sim, "SIM_EqualizeLiquidVolumeMass"));
  if (!sim_equalize_liquid_volume_mass) {
    printf("SIM_EqualizeLiquidVolumeMass not exported by %s\n", dll);
    return 1;
  }
  sim_adiabatic_fill_temperature = reinterpret_cast<float (*)(
      float, float, float, float, float)>(
      GetProcAddress(g_sim, "SIM_AdiabaticFillTemperature"));
  if (!sim_adiabatic_fill_temperature) {
    printf("SIM_AdiabaticFillTemperature not exported by %s\n", dll);
    return 1;
  }
  sim_compute_phase_change_step = reinterpret_cast<void (*)(
      float, float, float, float, float, float, float, float, float*, float*)>(
      GetProcAddress(g_sim, "SIM_ComputePhaseChangeStep"));
  if (!sim_compute_phase_change_step) {
    printf("SIM_ComputePhaseChangeStep not exported by %s\n", dll);
    return 1;
  }

  Tables t{};
  if (!LoadTables(corpus, &t)) return 1;
  uint16_t vacuum = 0, oxygen = 0;
  if (!Resolve(t, kVacuumHash, "Vacuum", &vacuum)) return 1;
  if (!Resolve(t, kOxygenHash, "Oxygen", &oxygen)) return 1;
  printf("  Oxygen idx=%u molarMass=%f state=%u\n", oxygen, t.At(oxygen)->molarMass,
         t.At(oxygen)->state);

  printf("=== vftest: real message pipeline, gas mixing ===\n");
  InitSim();

  // ---------------------------------------------------------------------------------------
  // THE PUBLISHED PHASE ORDERING, AND ITS DRIFT GUARD.
  //
  // Deliberately FIRST, before `SIM_AllocateCells`: these two exports read a compile-time
  // table rather than world state, and "callable before a world exists" is a property the
  // header promises, so it is asserted where it can actually fail rather than asserted in
  // prose. Every other Describe export in this file would return 0 here.
  //
  // The literal list below is TYPED OUT BY HAND on purpose. Generating it from
  // ONI_EXT_PHASE_LIST would make this arm pass by construction and prove nothing; written
  // out independently, it is a second opinion, and reordering the step body without updating
  // the published list fails here with the offending index named. That is a rule nobody can
  // accidentally not-run.
  //
  // If this fails after a genuine, intended reordering of the step body, the fix is to update
  // BOTH this list and ONI_EXT_PHASE_LIST -- and to treat it as the ABI break it is, because
  // this order is published to third-party mods.
  {
    printf("--- published phase ordering ---\n");
    struct Expect { const char* name; int32_t scope; int32_t gate; };
    static const Expect kExpected[] = {
        {"DrainQueue",               ext::kPhaseScopeFrame,        ext::kPhaseGateNone},
        {"StepWorldEnvironment",     ext::kPhaseScopeRegion,       ext::kPhaseGateSubstep},
        {"StepConduction",           ext::kPhaseScopeRegion,       ext::kPhaseGateSubstep},
        {"StepStateChange",          ext::kPhaseScopeRegion,       ext::kPhaseGateSubstep},
        {"StepGasPressure",          ext::kPhaseScopeRegion,       ext::kPhaseGateSubstep},
        {"StepGasDisplacement",      ext::kPhaseScopeRegion,       ext::kPhaseGateSubstep},
        {"StepFlow",                 ext::kPhaseScopeRegion,       ext::kPhaseGateSubstep},
        {"StepLiquidDisplacement",   ext::kPhaseScopeRegion,       ext::kPhaseGateSubstep},
        {"StepPayloadMixing",        ext::kPhaseScopeRegion,       ext::kPhaseGateWorld},
        {"StepDiseaseDiffusion",     ext::kPhaseScopeRegion,       ext::kPhaseGateWorld},
        {"StepRadiationField",       ext::kPhaseScopeRegion,       ext::kPhaseGateSubstep},
        {"StepElementConsumers",     ext::kPhaseScopeRegion,       ext::kPhaseGateSubstep},
        {"StepElementEmitters",      ext::kPhaseScopeRegion,       ext::kPhaseGateSubstep},
        {"StepRadiationEmitters",    ext::kPhaseScopeRegion,       ext::kPhaseGateSubstep},
        {"StepFields",               ext::kPhaseScopeRegion,       ext::kPhaseGateWorld},
        {"StepElementChunks",        ext::kPhaseScopeRegion,       ext::kPhaseGateSubstep},
        {"StepBuildingHeatExchange", ext::kPhaseScopeRegion,       ext::kPhaseGateSubstep},
        {"StepBuildingToBuilding",   ext::kPhaseScopeRegion,       ext::kPhaseGateSubstep},
        {"StepDiseaseEmitters",      ext::kPhaseScopeRegion,       ext::kPhaseGateSubstep},
        {"StepPostProcess",          ext::kPhaseScopeRegion,       ext::kPhaseGateSubstep},
        {"ZeroMasslessCells",        ext::kPhaseScopeGridInRegion, ext::kPhaseGateSubstep},
        {"StepDiseasePostProcess",   ext::kPhaseScopeRegion,       ext::kPhaseGateWorld},
        {"TickRoomPooledMixing",     ext::kPhaseScopeSubstep,      ext::kPhaseGateWorld},
        {"StepEffervescence",        ext::kPhaseScopeSubstep,      ext::kPhaseGateWorld},
        {"Project",                  ext::kPhaseScopeFrame,        ext::kPhaseGateNone},
    };
    const int32_t expected_n = static_cast<int32_t>(sizeof(kExpected) / sizeof(kExpected[0]));

    Check(sim_ext_phase_count() == expected_n,
          "SIM_ExtPhaseCount agrees with the independently written list -- a phase added to "
          "the step body without being published fails here");
    printf("    (DLL reports %d phases, this test expects %d)\n", sim_ext_phase_count(),
           expected_n);

    bool order_ok = true, scopes_ok = true, gates_ok = true, index_ok = true;
    for (int32_t i = 0; i < expected_n && i < sim_ext_phase_count(); ++i) {
      ext::ExtPhaseDesc pd{};
      if (sim_ext_phase_describe(i, &pd) != 1) {
        printf("    phase %d: describe refused a valid index\n", i);
        order_ok = false;
        continue;
      }
      if (strcmp(pd.name, kExpected[i].name) != 0) {
        printf("    phase %d: DLL says \"%s\", this test expects \"%s\"\n", i, pd.name,
               kExpected[i].name);
        order_ok = false;
      }
      if (pd.scope != kExpected[i].scope) {
        printf("    phase %d (%s): scope %d, expected %d\n", i, pd.name, pd.scope,
               kExpected[i].scope);
        scopes_ok = false;
      }
      if (pd.gate != kExpected[i].gate) {
        printf("    phase %d (%s): gate %d, expected %d\n", i, pd.name, pd.gate,
               kExpected[i].gate);
        gates_ok = false;
      }
      if (pd.index != i) index_ok = false;
    }
    Check(order_ok, "every phase name matches, in order -- this is the ABI surface");
    Check(scopes_ok,
          "every phase's scope matches: how often it runs relative to a frame is published "
          "too, not just where it sits");
    Check(gates_ok,
          "every phase's gate matches -- \"this phase did not run\" is a normal outcome and "
          "callers are told which phases can do it");
    Check(index_ok,
          "each descriptor echoes its own index, for the same reason ExtEventStreamDesc does");

    // ZeroMasslessCells is the one phase whose scope is neither per-region nor per-substep,
    // and the offline suite runs single-region worlds where the difference is invisible.
    // Pinned by name so a future edit that files it under kPhaseScopeRegion -- which would
    // look right on every scenario we own -- has to argue with this line.
    ext::ExtPhaseDesc zm{};
    Check(sim_ext_phase_describe(ext::kPhaseZeroMassless, &zm) == 1 &&
              zm.scope == ext::kPhaseScopeGridInRegion,
          "ZeroMasslessCells is declared whole-grid-inside-the-region-loop, not per-region: "
          "with N regions it sweeps every cell N times a substep, and no world in this suite "
          "has more than one region to notice");

    // Refusals, both ends and a null, with the buffer left alone -- the discipline stage 3b
    // and stage 4b both hold, and for the same reason: a caller that ignores the return value
    // must not find a plausible descriptor sitting in its buffer.
    ext::ExtPhaseDesc untouched{};
    memset(&untouched, 0x5A, sizeof(untouched));
    Check(sim_ext_phase_describe(-1, &untouched) == 0, "a negative phase index is refused");
    Check(untouched.scope == 0x5A5A5A5A, "and the buffer is untouched by the refusal");
    Check(sim_ext_phase_describe(sim_ext_phase_count(), &untouched) == 0,
          "one past the end is out of range too");
    Check(untouched.scope == 0x5A5A5A5A, "still untouched");
    Check(sim_ext_phase_describe(0, nullptr) == 0, "a null buffer is refused, not written");
  }

  // ---------------------------------------------------------------------------------------
  // THE PUBLISHED MESSAGE SURFACE, ACROSS THE ABI BOUNDARY.
  //
  // Deliberately here, beside the arm above and BEFORE `SIM_AllocateCells`, because the header
  // promises these two are callable before a world exists and that is a promise worth failing
  // on rather than asserting in prose. Every other Describe export in this file returns 0 at
  // this point.
  //
  // THIS ARM IS NOT A SECOND COPY OF THE TABLE. `driver/src/gastest.cpp` owns the table's
  // CONTENTS -- a hand-typed list of all 23 rows, in the one suite public CI can run, since
  // checking a compile-time table needs no corpus. What only a loaded DLL can check is what
  // is checked here: that the table is actually exported, that a descriptor survives the
  // marshalling with its packing intact, and that the refusals leave a caller's buffer alone.
  // Comparing the DLL's answers to `ext::` constants tests the plumbing, not the opinion --
  // the driver is compiled against the same header the DLL is.
  {
    printf("--- message surface across the ABI ---\n");
    Check(sim_ext_message_count() == ext::kExtMessageCount,
          "SIM_ExtMessageCount agrees with the header this driver was built against -- a DLL "
          "and a mod that disagree here disagree about what the wire ids mean");
    printf("    (DLL reports %d messages, this header declares %d)\n", sim_ext_message_count(),
           ext::kExtMessageCount);

    bool index_ok = true, id_ok = true, name_ok = true, enum_ok = true;
    // The next id the list should hold. It steps over exactly the ids `kExtMessageReservedIds`
    // counts -- "ONIR" and "ONIT", reserved for messages not in this release -- and over nothing else.
    int32_t want_id = ext::kExtMessageIdFirst;
    int32_t stepped_over = 0;
    for (int32_t i = 0; i < sim_ext_message_count(); ++i) {
      ext::ExtMessageDesc md{};
      if (sim_ext_message_describe(i, &md) != 1) {
        printf("    message %d: describe refused a valid index\n", i);
        index_ok = false;
        continue;
      }
      if (md.index != i) index_ok = false;
      // The id is the wire number and the index is a position in a list, and the whole point
      // of publishing both is that they are different things. Contiguity is the header's own
      // documented property, checked here through the boundary rather than assumed.
      while (md.id != want_id && (want_id == 0x4F4E4952 || want_id == 0x4F4E4954)) {
        ++want_id;
        ++stepped_over;
      }
      if (md.id != want_id) {
        printf("    message %d (%s): id 0x%08X, expected 0x%08X\n", i, md.name, md.id, want_id);
        id_ok = false;
      }
      ++want_id;
      // A name that arrived without its terminator would read off the end of the descriptor
      // in every caller that printed it, and a packing mistake is exactly how that happens.
      bool terminated = false;
      for (size_t c = 0; c < sizeof(md.name); ++c) {
        if (md.name[c] == '\0') { terminated = true; break; }
      }
      if (!terminated || md.name[0] == '\0') name_ok = false;
      if (md.messageClass < ext::kMessageClassParameter ||
          md.messageClass > ext::kMessageClassCheckpoint) enum_ok = false;
      if (md.delivery != ext::kDeliveryQueued && md.delivery != ext::kDeliveryImmediate)
        enum_ok = false;
      if (md.phase != ext::kPhaseUnpublished &&
          (md.phase < 0 || md.phase >= ext::kPhaseCount)) enum_ok = false;
    }
    Check(index_ok, "every index describes and each descriptor echoes its own index");
    if (stepped_over != ext::kExtMessageReservedIds) id_ok = false;
    Check(id_ok,
          "every wire id survives the boundary and the ids are contiguous, save for the "
          "reserved ids the header names");
    Check(name_ok, "every name comes back non-empty and NUL-terminated inside its 40 bytes");
    Check(enum_ok,
          "every class, delivery and phase is a value the header defines -- a descriptor "
          "carrying an out-of-range enum is a packing fault, not a new kind of message");

    // The two fields a caller reads by NAME rather than by walking, spot-checked through the
    // boundary. kSetElementAttribute is one of the four immediate ids and the arm much
    // further down proves that behaviourally; this is the same fact read off the ABI.
    ext::ExtMessageDesc ea{};
    Check(sim_ext_message_describe(
              ext::kSetElementAttribute - ext::kExtMessageIdFirst, &ea) == 1 &&
              ea.id == ext::kSetElementAttribute &&
              ea.delivery == ext::kDeliveryImmediate,
          "kSetElementAttribute is exported as IMMEDIATE, which is the one field of this "
          "table a caller's code shape depends on");
    ext::ExtMessageDesc tm{};
    Check(sim_ext_message_describe(
              ext::kSetCellThermalMassBonus - ext::kExtMessageIdFirst, &tm) == 1 &&
              tm.delivery == ext::kDeliveryQueued && tm.phase == ext::kPhaseConduction,
          "kSetCellThermalMassBonus is QUEUED and names StepConduction -- the two published "
          "tables joined up, which is the whole shape of this item");

    // Refusals, both ends and a null, with the buffer left alone. Same discipline every other
    // Describe in this file holds, for the same reason.
    ext::ExtMessageDesc untouched{};
    memset(&untouched, 0x5A, sizeof(untouched));
    Check(sim_ext_message_describe(-1, &untouched) == 0, "a negative message index is refused");
    Check(untouched.delivery == 0x5A5A5A5A, "and the buffer is untouched by the refusal");
    Check(sim_ext_message_describe(sim_ext_message_count(), &untouched) == 0,
          "one past the end is out of range too");
    Check(untouched.delivery == 0x5A5A5A5A, "still untouched");
    Check(sim_ext_message_describe(0, nullptr) == 0, "a null buffer is refused, not written");
  }

  // NOT a true vacuum: a real cell always carries some vanilla PhaseEntry mass, and
  // ZeroMasslessCells (a real vanilla kernel, unrelated to volume-fractions) zeroes
  // PhaseEntry.temperature every substep for any cell it considers massless. Since
  // gas_mixture_abi.h deliberately reuses PhaseEntry.temperature as the mixture's shared
  // temperature, a cell that is massless on the vanilla side but holds real gas-mixture mass
  // would have its shared temperature stomped to 0 every substep. Giving every cell a small real vanilla mass avoids
  // that entirely and is also the more representative case: a real ONI cell already has an
  // atmosphere: the volume-fractions layer adds species diversity on top of it, not into a
  // true void.
  World w;
  w.Init(5, 1, oxygen, 1.0f, 300.0f);
  Boot(t, w);

  // One end cell gets a real injection through the message path; the rest of the 5-cell
  // line starts empty. Same shape as gastest.cpp's TestSinglePairEqualizes, but driven
  // entirely through SIM_HandleMessage/StepPhysics rather than calling MixPair by hand.
  InjectGas(0, oxygen, 10.0f, 300.0f);

  std::vector<uint8_t> visible;
  // 0.2s/frame, well past the message-latency delay (effective next frame) and enough
  // substeps for a rate=0.2 mixing pass to visibly spread across 5 cells — gastest.cpp's
  // standalone test needed ~200 direct MixPair calls at this rate to fully equalize two
  // cells; this checks for real spread, not full equalization, so far fewer ticks suffice.
  for (int i = 0; i < 60; ++i) Tick(w, &visible);

  double total = 0.0;
  bool reached_far_cell = false;
  for (int c = 0; c < 5; ++c) {
    const float m = sim_debug_gas_mass(c, oxygen);
    printf("  cell %d: %.4f kg O2\n", c, m);
    total += m;
    if (c == 4 && m > 0.01f) reached_far_cell = true;
  }

  Check(std::fabs(total - 10.0f) < 1e-2, "mass conserved across the real message pipeline");
  Check(reached_far_cell,
        "gas injected at cell 0 reaches cell 4 through real substep-driven mixing");

  // Save/load round-trip. This world has real gas-mixture state (HasGasMixtureData
  // true), so ToBlob should write kSaveVersionGasMixture (16), not the legacy 15 -- checked
  // directly off the header bytes rather than assumed.
  printf("\n=== vftest: save/load round-trip ===\n");
  const std::vector<uint8_t> blob = Save(0, 0);
  int32_t blob_version = 0;
  Check(blob.size() > 12, "save blob is non-trivial");
  if (blob.size() > 12) {
    memcpy(&blob_version, blob.data() + 8, 4);
  }
  printf("  blob: %zu bytes, version=%d\n", blob.size(), blob_version);
  // The claim is that a world with real gas-mixture state does not write a LEGACY blob. The gas triple is a registered
  // kSaved extension property now, so it travels in the self-describing section instead of the
  // fixed v16 one. The other half of the migration -- an actual v16 blob still loading with
  // identical contents -- is a separate arm against the pinned fixtures, further down; one
  // assertion could not have covered both.
  Check(blob_version == 19,
        "a world with real gas-mixture state saves at kSaveVersionElementPalette (19), the gas "
        "triple travelling as registered kSaved properties rather than a fixed v16 section, "
        "with the element table its species indices were written against");
  DumpBlob("gas", blob);

  // Record the pre-load masses, then reload the very blob just saved and confirm they
  // survive exactly -- FromBlob followed by ActivateVolumeFractions (see the Load handler).
  double before_load = 0.0;
  for (int c = 0; c < 5; ++c) before_load += sim_debug_gas_mass(c, oxygen);

  sim_handle_message(static_cast<int32_t>(SimMessageHash::Load),
                     static_cast<int>(blob.size()), blob.data());

  double after_load = 0.0;
  for (int c = 0; c < 5; ++c) after_load += sim_debug_gas_mass(c, oxygen);
  printf("  before load: %.4f kg total, after load: %.4f kg total\n", before_load, after_load);
  Check(std::fabs(after_load - before_load) < 1e-2,
        "gas-mixture mass survives a real save/load round-trip");

  // The dominant-element facade's native half. Fresh DLL lifetime, one cell with
  // two species at very different masses -- SIM_GasDominantElement should report the heavier
  // one (Oxygen) as dominant and the TOTAL across both as outMass, not just the dominant
  // share. A second, never-injected cell in the same world checks the "no gas-mixture state"
  // contract: 0xFFFF, which is what the facade's C# caller reads as "fall back to vanilla."
  printf("\n=== vftest: dominant-element facade ===\n");
  sim_shutdown();
  InitSim();
  uint16_t co2 = 0;
  if (!Resolve(t, simhost::kCarbonDioxide, "CarbonDioxide", &co2)) return 1;
  // Five cells, not two: cell 4 needs to stay genuinely out of mixing's reach after just one
  // tick, to test the "no gas-mixture state" sentinel honestly (the mixing test above shows a
  // 5-cell line takes ~60 ticks at this rate to visibly reach the far end -- one tick isn't
  // close).
  World facade_world;
  facade_world.Init(5, 1, oxygen, 1.0f, 300.0f);
  Boot(t, facade_world);

  // Before any injection: the whole world is still "never activated," so every cell must
  // report the fall-back-to-vanilla sentinel.
  float pre_inject_mass = -1.0f;
  const uint16_t pre_inject = sim_gas_dominant_element(0, &pre_inject_mass);
  Check(pre_inject == 0xFFFF && pre_inject_mass == 0.0f,
        "a never-activated world's cells report the fall-back-to-vanilla sentinel");

  InjectGas(0, oxygen, 8.0f, 300.0f);
  InjectGas(0, co2, 2.0f, 300.0f);
  Tick(facade_world, &visible);  // lands the injections and runs exactly one mixing pass

  float facade_mass = -1.0f;
  const uint16_t dominant = sim_gas_dominant_element(0, &facade_mass);
  printf("  cell 0: dominant species idx=%u (oxygen idx=%u), total mass=%.4f kg\n", dominant,
         oxygen, facade_mass);
  Check(dominant == oxygen, "dominant-element facade picks the heavier species (Oxygen)");
  Check(facade_mass > 0.0f && facade_mass <= 10.0f,
        "facade reports cell 0's own total, not more than what was injected");

  // Conservation check has to sum every cell -- one mixing pass has already moved some mass
  // from cell 0 into cell 1, so cell 0 alone no longer holds the full 10 kg injected.
  double line_total = 0.0;
  for (int c = 0; c < 5; ++c) {
    line_total += sim_debug_gas_mass(c, oxygen) + sim_debug_gas_mass(c, co2);
  }
  printf("  line total (all 5 cells, O2+CO2): %.4f kg\n", line_total);
  Check(std::fabs(line_total - 10.0f) < 1e-2,
        "facade's total mass and the line's actual mass agree (conservation)");

  // NOT a sentinel case: MixRoomPooled's sweep is sequential (Gauss-Seidel-style, not
  // double-buffered), so within a *single* tick a pair's update is immediately visible to
  // the next pair in the same sweep -- pair (0,1) moves mass into cell 1, then pair (1,2)
  // reads that already-updated cell 1, and so on down the line. A small disturbance can
  // therefore ripple across an entire small open room in one tick, which is real and correct
  // (not a bug), just not the "genuinely untouched" case the sentinel check above needed --
  // that check has to run before any injection at all, which is what it does now.
  float far_mass = -1.0f;
  const uint16_t far_dominant = sim_gas_dominant_element(4, &far_mass);
  printf("  cell 4 (far from injection, same tick): dominant=%u, mass=%.6f\n", far_dominant,
         far_mass);
  Check(far_dominant == oxygen && far_mass > 0.0f && far_mass < facade_mass,
        "a same-tick trickle reaches the far cell via the sequential sweep, smaller than "
        "cell 0's own remaining share");

  printf("\n=== vftest: SIM_GasMassBatchQuery (overlay-coloring half) ===\n");
  {
    const int32_t cells[6] = {0, 1, 2, 3, 4, 999};
    uint8_t owned[6];
    uint16_t dominant[6];
    float mass[6];
    const int32_t written = sim_gas_mass_batch_query(cells, 6, owned, dominant, mass);
    Check(written == 6, "batch query writes every requested slot, including the invalid one");
    bool all_unowned = true;
    for (int i = 0; i < 5; ++i) all_unowned = all_unowned && owned[i] == 0;
    Check(all_unowned,
          "an active-but-not-yet-promoted room reports owned=0 for every real cell (fall back "
          "to vanilla)");
    Check(owned[5] == 0 && dominant[5] == 0xFFFF && mass[5] == 0.0f,
          "an invalid cell index reports the same owned=0/sentinel answer, not a crash");
  }

  // Room promotion: ext::kPromoteRoom should flip RoomGraph.owned for whichever room a cell
  // belongs to. SIM_DebugRoomOwned shows that from outside the DLL; the kernel gates are
  // tested further down. Reuses facade_world -- volume-fractions is already active there, all 5 cells in
  // one open room from BuildRoomGraph's flood-fill.
  printf("\n=== vftest: room promotion (ext::kPromoteRoom) ===\n");
  Check(sim_debug_room_owned(0) == 0, "a room starts vanilla-owned (0) before any promotion");
  ext::PromoteRoomMessage promote{0};
  sim_handle_message(ext::kPromoteRoom, static_cast<int>(sizeof(promote)),
                     reinterpret_cast<const uint8_t*>(&promote));
  // Same one-tick message latency every other ext:: message here has: this is deferred to the worker thread's queue, not applied
  // synchronously by SIM_HandleMessage itself -- SIM_DebugRoomOwned's WaitIdle() only waits
  // for whatever frame is already in flight, it doesn't force-drain a message sent this
  // instant. One real Tick is required before the effect is observable, same as InjectGas
  // above (found by this test's first real run: without the Tick, both checks below failed).
  Tick(facade_world, &visible);
  Check(sim_debug_room_owned(0) == 1, "kPromoteRoom flips cell 0's room to owned (1)");
  Check(sim_debug_room_owned(4) == 1,
        "promotion is per-ROOM, not per-cell -- cell 4 in the same open room reports owned too");
  Check(sim_debug_room_owned(999) == -1,
        "an invalid cell reports the no-room sentinel (-1), not a false positive");

  printf("\n=== vftest: SIM_GasMassBatchQuery reflects promotion "
         "===\n");
  {
    const int32_t cells[6] = {0, 1, 2, 3, 4, 999};
    uint8_t owned[6];
    uint16_t dominant[6];
    float mass[6];
    const int32_t written = sim_gas_mass_batch_query(cells, 6, owned, dominant, mass);
    Check(written == 6, "batch query still writes every slot after promotion");
    bool all_owned = true;
    for (int i = 0; i < 5; ++i) all_owned = all_owned && owned[i] == 1;
    Check(all_owned, "every real cell in the promoted room now reports owned=1");
    Check(owned[5] == 0, "the invalid cell is untouched by promotion, still owned=0");
    // Cross-check against the already-trusted single-cell facade, not a second source of truth.
    float single_mass = -1.0f;
    const uint16_t single_dominant = sim_gas_dominant_element(0, &single_mass);
    Check(dominant[0] == single_dominant && std::fabs(mass[0] - single_mass) < 1e-6f,
          "batch's cell-0 dominant/mass agree exactly with SIM_GasDominantElement's own answer");
    // Zero-count and null-pointer guards -- the "touches nothing, caller does the whole batch
    // vanilla-style" contract this export documents.
    Check(sim_gas_mass_batch_query(cells, 0, owned, dominant, mass) == 0,
          "count<=0 returns 0 and touches nothing");
    Check(sim_gas_mass_batch_query(nullptr, 6, owned, dominant, mass) == 0,
          "a null cellIndices pointer returns 0 rather than crashing");
  }

  // The room graph's READ side. Everything above
  // this point can see one boolean per cell; these two exports publish which room a cell is in
  // and what the whole room contains. Still on facade_world -- 5x1, one open room, promoted,
  // 8 kg O2 + 2 kg CO2 injected at cell 0 and mixed by the ticks above -- so every number here
  // is cross-checkable against the per-cell exports this is meant to replace.
  printf("\n=== vftest: SIM_RoomId / SIM_RoomAggregate ===\n");
  {
    const int32_t room0 = sim_room_id(0);
    Check(room0 >= 0, "a cell in an open room reports a real room id, not the -1 sentinel");
    Check(sim_room_id(4) == room0,
          "the far end of the same open room reports the SAME id -- the property that makes it "
          "a grouping key rather than a per-cell label");
    Check(sim_room_id(999) == -1,
          "an invalid cell reports the no-room sentinel (-1), the same one SIM_DebugRoomOwned "
          "uses");

    OniRoomAggregate agg;
    memset(&agg, 0xCD, sizeof(agg));  // poison: every field below has to be WRITTEN, not left
    Check(sim_room_aggregate(0, &agg) != 0, "the aggregate is filled for a cell in a room");
    printf("  room %d: %d cells (%d awake, %d with mixture), owned=%d, mixture=%.4f kg, "
           "vanilla=%.4f kg, %.2f Pa mean, %.2f K, %d species (+%d overflow)\n",
           agg.roomId, agg.cellCount, agg.awakeCount, agg.mixtureCellCount, agg.owned,
           agg.mixtureMassKg, agg.vanillaMassKg, agg.meanPressurePa, agg.temperatureK,
           agg.speciesCount, agg.speciesOverflow);

    Check(agg.roomId == room0, "the aggregate names the same room SIM_RoomId does");
    Check(agg.cellCount == 5,
          "the room is the whole 5x1 world -- every cell holds a gas element, so the flood fill "
          "finds one room, not five");
    Check(agg.owned == 1,
          "the aggregate reports the promotion SIM_DebugRoomOwned reports, from the same bit");
    Check(agg.awakeCount >= 0 && agg.awakeCount <= agg.cellCount,
          "awakeCount is a subset of the room, never more than it holds");
    Check(agg.mixtureCellCount > 0 && agg.mixtureCellCount <= agg.cellCount,
          "at least one cell carries mixture state after the injections, and never more cells "
          "than the room has");
    Check(agg.speciesOverflow == 0,
          "two species fit in ONI_ROOM_MAX_SPECIES with room to spare, so nothing overflows");
    Check(agg.speciesCount == 2,
          "the room's composition is the two injected species, counted once each across all "
          "five cells rather than once per cell");

    // The cross-check that matters: every number above is re-derived from the per-cell exports
    // this aggregate exists to replace. If the two ever disagree, one of them is wrong, and a
    // caller switching from the old shape to the new one would see a silent change of answer.
    double per_cell_mixture = 0.0;
    double per_cell_oxygen = 0.0;
    double per_cell_co2 = 0.0;
    double pressure_sum = 0.0;
    int pressure_cells = 0;
    for (int c = 0; c < 5; ++c) {
      float cell_total = -1.0f;
      const uint16_t dom = sim_gas_dominant_element(c, &cell_total);
      if (dom != 0xFFFF) per_cell_mixture += cell_total;
      per_cell_oxygen += sim_debug_gas_mass(c, oxygen);
      per_cell_co2 += sim_debug_gas_mass(c, co2);
      const float p = sim_gas_pressure(c);
      if (p >= 0.0f) {
        pressure_sum += p;
        ++pressure_cells;
      }
    }
    printf("  per-cell re-derivation: %.6f kg mixture (O2 %.6f + CO2 %.6f), %d cells with "
           "pressure, mean %.4f Pa\n",
           per_cell_mixture, per_cell_oxygen, per_cell_co2, pressure_cells,
           pressure_cells > 0 ? pressure_sum / pressure_cells : 0.0);
    Check(std::fabs(per_cell_mixture - agg.mixtureMassKg) < 1e-3,
          "the room's mixture mass equals the sum of its cells' own totals, read one at a time");
    Check(std::fabs(agg.mixtureMassKg - 10.0f) < 1e-2,
          "and that sum is the 10 kg injected -- the aggregate conserves what the line holds");
    Check(pressure_cells == agg.mixtureCellCount,
          "mixtureCellCount counts exactly the cells SIM_GasPressure answers for, which is what "
          "makes it the honest denominator for the mean");
    Check(pressure_cells > 0 &&
              std::fabs(pressure_sum / pressure_cells - agg.meanPressurePa) < 1e-2,
          "the mean pressure is the mean of those cells' own SIM_GasPressure answers");

    double oxygen_slot = -1.0, co2_slot = -1.0;
    for (int i = 0; i < agg.speciesCount; ++i) {
      if (agg.species[i] == oxygen) oxygen_slot = agg.massBySpeciesKg[i];
      if (agg.species[i] == co2) co2_slot = agg.massBySpeciesKg[i];
    }
    Check(oxygen_slot >= 0.0 && co2_slot >= 0.0,
          "both injected species appear in the composition by their ElementTable index");
    Check(std::fabs(oxygen_slot - per_cell_oxygen) < 1e-3 &&
              std::fabs(co2_slot - per_cell_co2) < 1e-3,
          "each species' room total equals the sum of SIM_DebugGasMass over the room's cells");
    double slot_sum = 0.0;
    for (int i = 0; i < agg.speciesCount; ++i) slot_sum += agg.massBySpeciesKg[i];
    Check(std::fabs(slot_sum - agg.mixtureMassKg) < 1e-3,
          "with no overflow the composition sums to the room's whole mixture mass -- the "
          "equality that stops being true, detectably, when speciesOverflow is non-zero");

    // The other layer. Init() gave every cell 1 kg of vanilla oxygen at 300 K and nothing here
    // has touched it, so vanillaMassKg is a real second number rather than a copy of the first.
    Check(std::fabs(agg.vanillaMassKg - 5.0f) < 1e-3,
          "vanillaMassKg is the PhaseEntry mass the room holds -- five cells of 1 kg -- and is "
          "reported separately from the mixture layer, not folded into it");
    Check(agg.temperatureK > 250.0f && agg.temperatureK < 350.0f,
          "the mass-weighted temperature is the 300 K everything in this world was created and "
          "injected at, not a zero from an unweighted average over massless cells");

    // ONE ROOM, ONE ANSWER. Asking from the far end of the same room must produce a
    // byte-identical struct: this is the property that lets a caller group cells by room id and
    // ask once, and it is exactly what a per-cell approximation like a radius-N diamond cannot
    // give -- two vents in one room would read two different "rooms".
    OniRoomAggregate from_far;
    memset(&from_far, 0xCD, sizeof(from_far));
    Check(sim_room_aggregate(4, &from_far) != 0, "the aggregate is filled from the far cell too");
    Check(memcmp(&agg, &from_far, sizeof(agg)) == 0,
          "every cell of a room answers with a byte-identical aggregate -- one room, one answer");

    // Refusals. The contract is that a caller ignoring the return value cannot read a plausible
    // room out of the struct, so the failure path has to WRITE, not merely return.
    OniRoomAggregate refused;
    memset(&refused, 0xCD, sizeof(refused));
    Check(sim_room_aggregate(999, &refused) == 0, "an invalid cell is refused");
    Check(refused.roomId == -1,
          "and the refusal WRITES the no-room sentinel over the caller's buffer rather than "
          "leaving whatever was in it");
    bool refused_zeroed = refused.cellCount == 0 && refused.mixtureCellCount == 0 &&
                          refused.speciesCount == 0 && refused.speciesOverflow == 0 &&
                          refused.owned == 0 && refused.mixtureMassKg == 0.0f &&
                          refused.vanillaMassKg == 0.0f && refused.meanPressurePa == 0.0f &&
                          refused.temperatureK == 0.0f;
    Check(refused_zeroed, "every other field of a refused aggregate is zero, not poison");
    Check(sim_room_aggregate(0, nullptr) == 0,
          "a null out-pointer is refused rather than written through");
  }

  // Conduit RUNS. Klei's conduit temperature manager was handed every conduit's mass
  // and element and kept only their product; now it keeps both, learns which run each conduit
  // is in from a bind the managed side sends after every topology rebuild, sums a run on
  // request, and can mix a run's temperature on the sim's own 200 ms update. None of that is
  // reachable through a message, so this drives the exports directly, the way diffsim's
  // `--conduits` kernel pass drives Klei's seven -- and that pass, which never binds, is the
  // witness that an unbound conduit still gets exactly Klei's arithmetic.
  //
  // Every conduit gets its own building slot whose temperature is set EQUAL to the conduit's.
  // That makes the conduit-to-building exchange an exact no-op (zero rate, zero transfer,
  // `next` == the starting temperature) while still reaching the end of Klei's loop, where the
  // phase-transition check lives -- so every temperature change below is the mix and nothing
  // else, and the transition check still runs.
  printf("\n=== vftest: conduit runs -- SIM_ConduitNetworkBind / Policy / Aggregate "
         "===\n");
  {
#pragma pack(push, 4)
    struct ConduitUpdate {
      int32_t numEntries;
      float* temperatures;
      int32_t numFrozenHandles;
      int32_t* frozenHandles;
      int32_t numMeltedHandles;
      int32_t* meltedHandles;
    };
    struct BuildingTemp {
      int32_t handle;
      float temperature;
    };
#pragma pack(pop)
    constexpr int32_t kGas = 1, kLiquid = 2;
    constexpr float kGasVolume = 0.01f, kLiquidVolume = 0.02f;
    constexpr float kGasMax = 1.0f, kLiquidMax = 10.0f;  // ConduitFlow.MaxMass, vanilla
    constexpr float kConduitHC = 1000.0f, kConduitK = 1.0f;
    auto slot = [](int32_t h) { return static_cast<size_t>(h & 0x00FFFFFF); };
    constexpr int32_t kBuildings = 20;
    BuildingTemp bt[kBuildings];
    for (int32_t i = 0; i < kBuildings; ++i) bt[i] = BuildingTemp{i, 300.0f};

    conduit_clear();
    // Run 0 (gas): 1 kg O2 at 300 K, 0.5 kg CO2 at 360 K and one empty conduit, piped in that
    // order. Run 1 (gas): 2 kg O2 at 250 K. Run 2 (gas): 1 kg O2 at 85 K -- past oxygen's
    // condensation margin -- piped to 1 kg O2 at 300 K. Run 3 (gas): a chain of five 1 kg O2
    // conduits, the first at 400 K and the rest at 300 K. Run 4 (gas): a 0.01 kg O2 hub at 400 K
    // that five 10 kg O2 spokes at 300 K all name as a neighbour -- a bind the game could never
    // send, since a conduit has four sides. Liquid run 0: 10 kg water at 290 K.
    const int32_t h_o2 = conduit_add(300.0f, 1.0f, kOxygenHash, 0, kConduitHC, kConduitK, 0);
    const int32_t h_co2 =
        conduit_add(360.0f, 0.5f, simhost::kCarbonDioxide, 1, kConduitHC, kConduitK, 0);
    const int32_t h_empty = conduit_add(300.0f, 0.0f, kVacuumHash, 2, kConduitHC, kConduitK, 0);
    const int32_t h_run1 = conduit_add(250.0f, 2.0f, kOxygenHash, 3, kConduitHC, kConduitK, 0);
    const int32_t h_cold = conduit_add(85.0f, 1.0f, kOxygenHash, 4, kConduitHC, kConduitK, 0);
    const int32_t h_warm = conduit_add(300.0f, 1.0f, kOxygenHash, 5, kConduitHC, kConduitK, 0);
    const int32_t h_water = conduit_add(290.0f, 10.0f, kWater, 6, kConduitHC, kConduitK, 0);
    constexpr int32_t kChain = 5, kSpokes = 5;
    int32_t h_chain[kChain];
    for (int32_t i = 0; i < kChain; ++i) {
      const float t0 = i == 0 ? 400.0f : 300.0f;
      h_chain[i] = conduit_add(t0, 1.0f, kOxygenHash, 8 + i, kConduitHC, kConduitK, 0);
      bt[8 + i].temperature = t0;
    }
    const int32_t h_hub = conduit_add(400.0f, 0.01f, kOxygenHash, 13, kConduitHC, kConduitK, 0);
    bt[13].temperature = 400.0f;
    int32_t h_spoke[kSpokes];
    for (int32_t i = 0; i < kSpokes; ++i) {
      h_spoke[i] = conduit_add(300.0f, 10.0f, kOxygenHash, 14 + i, kConduitHC, kConduitK, 0);
    }
    bt[1].temperature = 360.0f;
    bt[3].temperature = 250.0f;
    bt[4].temperature = 85.0f;
    bt[6].temperature = 290.0f;

    OniConduitNetworkAggregate agg;
    memset(&agg, 0xCD, sizeof(agg));
    Check(sim_conduit_aggregate(kGas, 0, &agg, sizeof(agg)) == 0 && agg.networkId == -1,
          "before any bind there are no runs: the aggregate is refused and still writes -1");

    // Neighbours are four per conduit -- left, right, up, down -- as SOAInfo.GetConduitConnections
    // names them. Run 0's O2 also names run 1's conduit (up): a neighbour in another run, which
    // must not count. Run 2's link is listed from the cold side only, as the game does when one
    // conduit's connection mask has a side its neighbour's lacks -- and listed there twice, which
    // the game never does, so that a link named in two slots is shown to count once.
    constexpr int32_t kGasBound = 7 + kChain + 1 + kSpokes;
    int32_t gas_handles[kGasBound];
    int32_t gas_cells[kGasBound];
    int32_t gas_nets[kGasBound];
    int32_t gas_neigh[kGasBound * 4];
    for (int32_t& n : gas_neigh) n = -1;
    const int32_t first7_handles[] = {h_o2, h_co2, h_empty, h_run1, h_cold, h_warm, 0x00ABCDEF};
    const int32_t first7_cells[] = {10, 11, 12, 20, 30, 31, 99};
    const int32_t first7_nets[] = {0, 0, 0, 1, 2, 2, 0};
    for (int32_t i = 0; i < 7; ++i) {
      gas_handles[i] = first7_handles[i];
      gas_cells[i] = first7_cells[i];
      gas_nets[i] = first7_nets[i];
    }
    gas_neigh[0 * 4 + 1] = h_co2;    // O2 -> right: CO2
    gas_neigh[0 * 4 + 2] = h_run1;   // O2 -> up: run 1's conduit, another run
    gas_neigh[1 * 4 + 0] = h_o2;     // CO2 -> left: O2
    gas_neigh[1 * 4 + 1] = h_empty;  // CO2 -> right: the empty conduit
    gas_neigh[2 * 4 + 0] = h_co2;    // empty -> left: CO2
    gas_neigh[4 * 4 + 1] = h_warm;   // cold -> right: warm; warm names nothing back
    gas_neigh[4 * 4 + 3] = h_warm;   // cold -> down: warm again
    for (int32_t i = 0; i < kChain; ++i) {
      const int32_t at = 7 + i;
      gas_handles[at] = h_chain[i];
      gas_cells[at] = 50 + i;
      gas_nets[at] = 3;
      if (i > 0) gas_neigh[at * 4 + 0] = h_chain[i - 1];
      if (i < kChain - 1) gas_neigh[at * 4 + 1] = h_chain[i + 1];
    }
    gas_handles[7 + kChain] = h_hub;
    gas_cells[7 + kChain] = 60;
    gas_nets[7 + kChain] = 4;
    for (int32_t i = 0; i < kSpokes; ++i) {
      const int32_t at = 7 + kChain + 1 + i;
      gas_handles[at] = h_spoke[i];
      gas_cells[at] = 61 + i;
      gas_nets[at] = 4;
      gas_neigh[at * 4 + 0] = h_hub;
    }
    Check(sim_conduit_bind(kGas, gas_handles, gas_cells, gas_nets, gas_neigh, kGasBound,
                           kGasVolume, kGasMax) == kGasBound - 1,
          "every real handle is bound and a handle this manager never issued is skipped, "
          "rather than refusing the whole run");
    const int32_t liquid_handles[] = {h_water};
    const int32_t liquid_cells[] = {40};
    const int32_t liquid_nets[] = {0};
    const int32_t liquid_neigh[] = {-1, -1, -1, -1};
    Check(sim_conduit_bind(kLiquid, liquid_handles, liquid_cells, liquid_nets, liquid_neigh, 1,
                           kLiquidVolume, kLiquidMax) == 1,
          "the liquid layer binds separately, with its own numbering");

    Check(sim_conduit_bind(3, gas_handles, gas_cells, gas_nets, gas_neigh, 7, kGasVolume,
                           kGasMax) == -1 &&
              sim_conduit_bind(kGas, gas_handles, gas_cells, gas_nets, gas_neigh, -1,
                               kGasVolume, kGasMax) == -1 &&
              sim_conduit_bind(kGas, nullptr, gas_cells, gas_nets, gas_neigh, 7, kGasVolume,
                               kGasMax) == -1 &&
              sim_conduit_bind(kGas, gas_handles, gas_cells, gas_nets, nullptr, 7, kGasVolume,
                               kGasMax) == -1 &&
              sim_conduit_bind(kGas, gas_handles, gas_cells, gas_nets, gas_neigh, 7, 0.0f,
                               kGasMax) == -1 &&
              sim_conduit_bind(kGas, gas_handles, gas_cells, gas_nets, gas_neigh, 7,
                               kGasVolume, 0.0f) == -1,
          "a solid-conduit type, a negative count, a null handle array, a null neighbour "
          "array, a zero volume and a zero capacity are each refused with -1");

    memset(&agg, 0xCD, sizeof(agg));
    Check(sim_conduit_aggregate(kGas, 0, &agg, sizeof(agg)) == 1, "gas run 0 is summed");
    const double hc_o2 = 1.0 * t.SpecificHeat(oxygen);
    const double hc_co2 = 0.5 * t.SpecificHeat(co2);
    const double t_eq = (hc_o2 * 300.0 + hc_co2 * 360.0) / (hc_o2 + hc_co2);
    printf("  run 0: %d conduits (%d filled), %.4f kg, %.4f mol, %.3f K (mass-weighted %.3f K), "
           "%.2f Pa over %.3f m3, generation %d, %d species (+%d)\n",
           agg.conduitCount, agg.filledCount, agg.totalMassKg, agg.totalMoles,
           agg.temperatureK, agg.massWeightedTemperatureK, agg.pressurePa, agg.volumeM3,
           agg.generation, agg.speciesCount, agg.speciesOverflow);
    Check(agg.networkId == 0 && agg.conduitType == kGas,
          "the aggregate names the run and the type it was asked for");
    Check(agg.generation == 1,
          "generation counts accepted binds of the gas type: one, because the six refused "
          "binds above changed nothing at all");
    Check(agg.conduitCount == 3 && agg.filledCount == 2,
          "three conduits are bound to run 0, and the empty one is counted but not as filled");
    Check(std::fabs(agg.totalMassKg - 1.5f) < 1e-5f,
          "the run holds the 1 kg of O2 and 0.5 kg of CO2 it was given -- the mass Klei's "
          "manager used to multiply away");
    double o2_slot = -1.0, co2_slot = -1.0;
    for (int32_t k = 0; k < agg.speciesCount; ++k) {
      if (agg.species[k] == oxygen) o2_slot = agg.massBySpeciesKg[k];
      if (agg.species[k] == co2) co2_slot = agg.massBySpeciesKg[k];
    }
    Check(agg.speciesCount == 2 && agg.speciesOverflow == 0 &&
              std::fabs(o2_slot - 1.0) < 1e-5 && std::fabs(co2_slot - 0.5) < 1e-5,
          "the composition names both elements by ElementTable index, with their own masses "
          "-- the sim can now say WHAT is in a pipe");
    Check(std::fabs(agg.massWeightedTemperatureK - 320.0f) < 1e-3f,
          "the mass-weighted temperature is (1*300 + 0.5*360) / 1.5 = 320 K");
    Check(std::fabs(agg.temperatureK - t_eq) < 1e-3,
          "the heat-capacity-weighted temperature is the run's mixing equilibrium, from each "
          "element's own specific heat");
    Check(std::fabs(agg.volumeM3 - 3.0f * kGasVolume) < 1e-7f,
          "the run's volume is its conduit count times the volume the bind declared");
    const uint16_t species2[] = {oxygen, co2};
    const float masses2[] = {1.0f, 0.5f};
    const float expected_p =
        sim_compute_gas_pressure(species2, masses2, 2, agg.temperatureK, agg.volumeM3);
    Check(expected_p > 0.0f && std::fabs(agg.pressurePa - expected_p) < 1e-4f * expected_p,
          "the run's pressure is SIM_ComputeGasPressure's answer for the same contents -- one "
          "formula, not a second copy of it");

    OniConduitNetworkAggregate run1;
    Check(sim_conduit_aggregate(kGas, 1, &run1, sizeof(run1)) == 1 && run1.conduitCount == 1 &&
              std::fabs(run1.totalMassKg - 2.0f) < 1e-5f &&
              std::fabs(run1.temperatureK - 250.0f) < 1e-3f,
          "run 1 is its own run: one conduit, 2 kg, 250 K, nothing of run 0 in it");
    int32_t links[5] = {-1, -1, -1, -1, -1};
    for (int32_t r = 0; r < 5; ++r) {
      OniConduitNetworkAggregate each;
      if (sim_conduit_aggregate(kGas, r, &each, sizeof(each)) == 1) links[r] = each.linkCount;
    }
    printf("  links per gas run: %d %d %d %d %d\n", links[0], links[1], links[2], links[3],
           links[4]);
    Check(links[0] == 2 && links[1] == 0 && links[2] == 1 && links[3] == kChain - 1 &&
              links[4] == kSpokes,
          "linkCount counts each link inside a run once: run 0's two (the empty conduit is "
          "piped in even though it holds nothing, and O2's neighbour in run 1 is not in run 0), "
          "run 2's one-sided link named twice, the chain's four and the hub's five");
    OniConduitNetworkAggregate water;
    Check(sim_conduit_aggregate(kLiquid, 0, &water, sizeof(water)) == 1 &&
              water.conduitCount == 1 && std::fabs(water.totalMassKg - 10.0f) < 1e-4f &&
              water.pressurePa == 0.0f && std::fabs(water.volumeM3 - kLiquidVolume) < 1e-7f,
          "a liquid run sums its mass and declares its own volume, and reports no pressure "
          "rather than a guess at its headspace");

    OniConduitNetworkAggregate refused;
    memset(&refused, 0xCD, sizeof(refused));
    Check(sim_conduit_aggregate(kGas, 5, &refused, sizeof(refused)) == 0 &&
              refused.networkId == -1 && refused.conduitCount == 0 &&
              refused.totalMassKg == 0.0f,
          "a run outside the current bind is refused, and the refusal writes a zeroed struct "
          "with networkId -1 rather than leaving poison");
    unsigned char small[sizeof(OniConduitNetworkAggregate)];
    memset(small, 0xCD, sizeof(small));
    const int32_t small_ok = sim_conduit_aggregate(kGas, 0, reinterpret_cast<OniConduitNetworkAggregate*>(small), 4);
    int32_t small_id = -2;
    memcpy(&small_id, small, 4);
    bool tail_untouched = true;
    for (size_t b = 4; b < sizeof(small); ++b) tail_untouched = tail_untouched && small[b] == 0xCD;
    Check(small_ok == 1 && small_id == 0 && tail_untouched,
          "a caller declaring a 4-byte buffer gets exactly the networkId and not one byte past "
          "it -- what lets the struct grow at its end");
    Check(sim_conduit_aggregate(kGas, 0, &agg, 3) == 0 &&
              sim_conduit_aggregate(kGas, 0, nullptr, sizeof(agg)) == 0,
          "a buffer too small for networkId, or a null one, is refused");

    // Set moves the aggregate: the dirty flag is what makes a cached sum honest.
    conduit_set(h_co2, 360.0f, 0.25f, simhost::kCarbonDioxide);
    Check(sim_conduit_aggregate(kGas, 0, &agg, sizeof(agg)) == 1 &&
              std::fabs(agg.totalMassKg - 1.25f) < 1e-5f,
          "a Set that halves the CO2 is visible in the very next aggregate");
    conduit_set(h_co2, 360.0f, 0.5f, simhost::kCarbonDioxide);

    // Policy refusals, then the mix.
    const float nan = std::numeric_limits<float>::quiet_NaN();
    Check(sim_conduit_policy(kGas, 5, ONI_CONDUIT_POLICY_MIX, 0.25f, 0.0f, 0.0f) == 0 &&
              sim_conduit_policy(kGas, 0, 0x80, 0.25f, 0.0f, 0.0f) == 0 &&
              sim_conduit_policy(kGas, 0, ONI_CONDUIT_POLICY_MIX, 0.0f, 0.0f, 0.0f) == 0 &&
              sim_conduit_policy(kGas, 0, ONI_CONDUIT_POLICY_MIX, 0.3f, 0.0f, 0.0f) == 0 &&
              sim_conduit_policy(kGas, 0, ONI_CONDUIT_POLICY_MIX, 1.0f, 0.0f, 0.0f) == 0 &&
              sim_conduit_policy(kGas, 0, ONI_CONDUIT_POLICY_MIX, nan, 0.0f, 0.0f) == 0 &&
              sim_conduit_policy(4, 0, ONI_CONDUIT_POLICY_MIX, 0.25f, 0.0f, 0.0f) == 0,
          "a policy for a run outside the bind, an unknown flag bit, a coupling of 0, 0.3, 1.0 "
          "or NaN, and an unknown type are each refused -- 0.3 and 1.0 because four links at "
          "more than 0.25 could carry a conduit past every neighbour it has");

    // What one link exchanges, written out from the formula in sim/conduits.h's MixNetworks so
    // that every expectation below is derived rather than recorded.
    auto link_q = [](double k, double ta, double ca, double tb, double cb) {
      return k * (ta - tb) * (ca * cb / (ca + cb));
    };

    // First update: run 2 has NO policy, so its 85 K conduit freezes exactly as Klei would
    // report it. Run 0 mixes.
    Check(sim_conduit_policy(kGas, 0, ONI_CONDUIT_POLICY_MIX, 0.25f, 0.0f, 0.0f) == 1,
          "MIX at the 0.25 ceiling is accepted on run 0");
    const ConduitUpdate* u = static_cast<const ConduitUpdate*>(conduit_update(0.2f, bt));
    const double q0 = link_q(0.25, 300.0, hc_o2, 360.0, hc_co2);
    const double pair_o2 = 300.0 - q0 / hc_o2;
    const double pair_co2 = 360.0 + q0 / hc_co2;
    const double before = hc_o2 * 300.0 + hc_co2 * 360.0;
    const double after = hc_o2 * u->temperatures[slot(h_o2)] + hc_co2 * u->temperatures[slot(h_co2)];
    printf("  after one update: O2 %.4f K, CO2 %.4f K (one link predicts %.4f / %.4f K), "
           "empty %.4f K, run 1 %.4f K, water %.4f K; contents energy %.6f -> %.6f kJ\n",
           u->temperatures[slot(h_o2)], u->temperatures[slot(h_co2)], pair_o2, pair_co2,
           u->temperatures[slot(h_empty)], u->temperatures[slot(h_run1)],
           u->temperatures[slot(h_water)], before, after);
    Check(std::fabs(u->temperatures[slot(h_o2)] - pair_o2) < 1e-3 &&
              std::fabs(u->temperatures[slot(h_co2)] - pair_co2) < 1e-3,
          "run 0's two filled conduits exchange exactly one link's worth -- a quarter of the "
          "reduced-heat-capacity gap, each moving by its own heat capacity -- and it is counted "
          "once although both of them name the link");
    Check(std::fabs(after - before) < 1e-6 * before,
          "and the run's contents energy is exactly what it was: the mix moves heat between "
          "packets and creates or deletes none");
    Check(u->temperatures[slot(h_empty)] == 300.0f,
          "the empty conduit, though piped to the CO2, takes no part -- it holds nothing to "
          "carry heat");
    Check(u->temperatures[slot(h_run1)] == 250.0f && u->temperatures[slot(h_water)] == 290.0f,
          "a neighbour in another run is not a link: run 1's conduit, which run 0's O2 names, "
          "is untouched, and so is the liquid layer");
    bool cold_frozen = false;
    for (int32_t k = 0; k < u->numFrozenHandles; ++k) cold_frozen = cold_frozen || u->frozenHandles[k] == h_cold;
    Check(cold_frozen,
          "without a policy, run 2's 85 K oxygen is reported frozen, exactly as Klei reports it");
    Check(u->temperatures[slot(h_chain[0])] == 400.0f &&
              u->temperatures[slot(h_chain[1])] == 300.0f && u->temperatures[slot(h_hub)] == 400.0f,
          "runs 3 and 4 have no policy yet and do not move");
    Check(sim_conduit_aggregate(kGas, 0, &agg, sizeof(agg)) == 1 &&
              agg.policyFlags == ONI_CONDUIT_POLICY_MIX && agg.mixFraction == 0.25f &&
              std::fabs(agg.temperatureK - t_eq) < 1e-3,
          "the aggregate reports the policy in force, and the run's heat-capacity-weighted "
          "temperature is exactly where it was: a conserving mix cannot move it");

    // Second update: run 2 mixes too. Its transition is checked AFTER the mix, against the
    // temperature the game will actually be handed.
    Check(sim_conduit_policy(kGas, 2, ONI_CONDUIT_POLICY_MIX, 0.25f, 0.0f, 0.0f) == 1,
          "MIX on run 2");
    bt[0].temperature = u->temperatures[slot(h_o2)];
    bt[1].temperature = u->temperatures[slot(h_co2)];
    u = static_cast<const ConduitUpdate*>(conduit_update(0.2f, bt));
    // Equal heat capacities: the reduced one is C/2, so each end moves k/2 of the gap.
    const double cold_mixed = 85.0 + 0.125 * (300.0 - 85.0);
    const double warm_mixed = 300.0 - 0.125 * (300.0 - 85.0);
    const float o2_freezes_below = t.At(oxygen)->lowTemp - 3.0f;
    printf("  run 2: cold %.4f K, warm %.4f K (predicted %.4f / %.4f K); oxygen is reported "
           "frozen below %.2f K\n",
           u->temperatures[slot(h_cold)], u->temperatures[slot(h_warm)], cold_mixed, warm_mixed,
           o2_freezes_below);
    Check(85.0f < o2_freezes_below && cold_mixed > o2_freezes_below,
          "precondition: 85 K oxygen is past its condensation margin and the mixed temperature "
          "is not, so the next check can tell a deferred transition check from an early one");
    cold_frozen = false;
    for (int32_t k = 0; k < u->numFrozenHandles; ++k) cold_frozen = cold_frozen || u->frozenHandles[k] == h_cold;
    Check(std::fabs(u->temperatures[slot(h_cold)] - cold_mixed) < 1e-3 &&
              std::fabs(u->temperatures[slot(h_warm)] - warm_mixed) < 1e-3,
          "a link listed by ONE end only still exchanges, and exactly once -- the game builds "
          "each conduit's connections from its own mask, so this is a shape it can send -- and "
          "naming it in a second slot does not make it two links");
    Check(!cold_frozen,
          "and the 85 K conduit is NOT reported frozen: its transition check saw the mixed "
          "temperature, not the 85 K the mix overwrote");

    // dt == 0 is RebuildConnections publishing fresh handles; it must not mix.
    conduit_set(h_o2, 300.0f, 1.0f, kOxygenHash);
    conduit_set(h_co2, 360.0f, 0.5f, simhost::kCarbonDioxide);
    bt[0].temperature = 300.0f;
    bt[1].temperature = 360.0f;
    u = static_cast<const ConduitUpdate*>(conduit_update(0.0f, bt));
    Check(u->temperatures[slot(h_o2)] == 300.0f && u->temperatures[slot(h_co2)] == 360.0f,
          "a dt = 0 update does not mix");
    u = static_cast<const ConduitUpdate*>(conduit_update(0.2f, nullptr));
    Check(u->temperatures[slot(h_o2)] == 300.0f && u->temperatures[slot(h_co2)] == 360.0f,
          "nor does an update with no building temperatures -- the game has not published a "
          "frame, and Klei copies straight through");

    // LOCALITY: the reason MIX is axial. Run 3 is a chain of five equal conduits with all the
    // heat at one end. Stationeers' whole-network mix would put every conduit at 320 K in one
    // update; the axial exchange moves only the two conduits that share the hot link, and the
    // three beyond it must not move by a single bit.
    Check(sim_conduit_policy(kGas, 3, ONI_CONDUIT_POLICY_MIX, 0.25f, 0.0f, 0.0f) == 1,
          "MIX on run 3, the five-conduit chain");
    const double hc_one = 1.0 * t.SpecificHeat(oxygen);
    u = static_cast<const ConduitUpdate*>(conduit_update(0.2f, bt));
    printf("  chain after one update: %.4f %.4f %.4f %.4f %.4f K\n",
           u->temperatures[slot(h_chain[0])], u->temperatures[slot(h_chain[1])],
           u->temperatures[slot(h_chain[2])], u->temperatures[slot(h_chain[3])],
           u->temperatures[slot(h_chain[4])]);
    Check(std::fabs(u->temperatures[slot(h_chain[0])] - 387.5) < 1e-3 &&
              std::fabs(u->temperatures[slot(h_chain[1])] - 312.5) < 1e-3,
          "the hot end and its neighbour close an eighth of their 100 K gap each (k/2 for "
          "equal heat capacities): 387.5 K and 312.5 K");
    Check(u->temperatures[slot(h_chain[2])] == 300.0f &&
              u->temperatures[slot(h_chain[3])] == 300.0f &&
              u->temperatures[slot(h_chain[4])] == 300.0f,
          "and the three conduits beyond them are exactly where they were: heat travels one "
          "link per update, so a gradient along a run survives the mix");

    // Left alone, the chain still levels. Each building is held at its conduit's temperature
    // so the only thing moving heat is the mix, and the energy must hold the whole way.
    const double chain_energy = hc_one * (400.0 + 4.0 * 300.0);
    bool chain_bounded = true;
    for (int32_t step = 0; step < 400; ++step) {
      for (int32_t i = 0; i < kChain; ++i) {
        bt[8 + i].temperature = u->temperatures[slot(h_chain[i])];
      }
      u = static_cast<const ConduitUpdate*>(conduit_update(0.2f, bt));
      for (int32_t i = 0; i < kChain; ++i) {
        const double ti = u->temperatures[slot(h_chain[i])];
        chain_bounded = chain_bounded && ti >= 300.0 - 1e-4 && ti <= 400.0 + 1e-4;
      }
    }
    double chain_after = 0.0, chain_spread = 0.0;
    for (int32_t i = 0; i < kChain; ++i) {
      const double ti = u->temperatures[slot(h_chain[i])];
      chain_after += hc_one * ti;
      chain_spread = std::max(chain_spread, std::fabs(ti - 320.0));
    }
    printf("  chain after 400 more updates: within %.6f K of 320 K; energy %.6f -> %.6f kJ\n",
           chain_spread, chain_energy, chain_after);
    Check(chain_spread < 1e-2,
          "left alone, the chain settles on its one equilibrium, 320 K -- axial mixing levels a "
          "run, it just does it along the pipe instead of all at once");
    Check(std::fabs(chain_after - chain_energy) < 1e-5 * chain_energy,
          "and after 401 updates the chain holds the energy it started with, to float rounding");
    Check(chain_bounded,
          "no conduit ever left the 300-400 K range it started in: with k <= 0.25 every mixed "
          "temperature is a weighted average of its neighbourhood, so nothing overshoots");

    // THE DEGREE GUARD. Run 4's hub is named by five spokes, each a thousand times its heat
    // capacity. Unscaled, five links at 0.25 would move the hub nearly 1.25 times its gap, past
    // the spokes; with k scaled by 4/5 it moves just under the gap and lands inside the range.
    Check(sim_conduit_policy(kGas, 4, ONI_CONDUIT_POLICY_MIX, 0.25f, 0.0f, 0.0f) == 1,
          "MIX on run 4, the corrupt five-link hub");
    const double hc_hub = 0.01 * t.SpecificHeat(oxygen);
    const double hc_spoke = 10.0 * t.SpecificHeat(oxygen);
    const double q_scaled = link_q(0.25 * 4.0 / 5.0, 300.0, hc_spoke, 400.0, hc_hub);
    const double q_unscaled = link_q(0.25, 300.0, hc_spoke, 400.0, hc_hub);
    const double hub_expected = 400.0 + kSpokes * q_scaled / hc_hub;
    const double spoke_expected = 300.0 - q_scaled / hc_spoke;
    const double hub_unscaled = 400.0 + kSpokes * q_unscaled / hc_hub;
    u = static_cast<const ConduitUpdate*>(conduit_update(0.2f, bt));
    printf("  hub %.4f K (predicted %.4f K; unscaled it would be %.4f K), spoke %.6f K\n",
           u->temperatures[slot(h_hub)], hub_expected, hub_unscaled,
           u->temperatures[slot(h_spoke[0])]);
    Check(hub_unscaled < 300.0,
          "precondition: without the guard this hub would overshoot the spokes it mixes with");
    bool spokes_ok = true;
    double hub_energy_after = hc_hub * u->temperatures[slot(h_hub)];
    for (int32_t i = 0; i < kSpokes; ++i) {
      spokes_ok = spokes_ok &&
                  std::fabs(u->temperatures[slot(h_spoke[i])] - spoke_expected) < 1e-4;
      hub_energy_after += hc_spoke * u->temperatures[slot(h_spoke[i])];
    }
    const double hub_energy_before = hc_hub * 400.0 + kSpokes * hc_spoke * 300.0;
    Check(std::fabs(u->temperatures[slot(h_hub)] - hub_expected) < 1e-3 && spokes_ok &&
              u->temperatures[slot(h_hub)] >= 300.0f,
          "with more than four links, each link's k is scaled by four over the link count, and "
          "the hub stays between the temperatures it mixed from");
    Check(std::fabs(hub_energy_after - hub_energy_before) < 1e-6 * hub_energy_before,
          "and the guard, being a smaller k, still conserves the run's energy");

    // Remove leaves the run at once; a rebind bumps the generation and drops every policy.
    conduit_remove(h_co2);
    Check(sim_conduit_aggregate(kGas, 0, &agg, sizeof(agg)) == 1 && agg.conduitCount == 2 &&
              std::fabs(agg.totalMassKg - 1.0f) < 1e-5f,
          "a removed conduit leaves its run immediately, not a frame later with its slot");
    Check(agg.linkCount == 0,
          "and takes both of run 0's links with it: the O2 and the empty conduit were each "
          "piped only to the CO2");
    const float o2_alone = u->temperatures[slot(h_o2)];
    bt[0].temperature = o2_alone;
    u = static_cast<const ConduitUpdate*>(conduit_update(0.2f, bt));
    Check(u->temperatures[slot(h_o2)] == o2_alone,
          "and it leaves the mix just as fast: run 0's O2 still names it, but with its only "
          "filled neighbour removed it has nothing to exchange with and does not move");
    const int32_t rebind_handles[] = {h_o2, h_co2, h_empty};
    const int32_t rebind_cells[] = {10, 11, 12};
    const int32_t rebind_nets[] = {0, 0, 0};
    const int32_t rebind_neigh[] = {-1, h_co2, -1, -1, h_o2, h_empty, -1, -1, h_co2, -1, -1, -1};
    Check(sim_conduit_bind(kGas, rebind_handles, rebind_cells, rebind_nets, rebind_neigh, 3,
                           kGasVolume, kGasMax) == 2,
          "a rebind naming the removed handle skips it: it resolves until the next frame's "
          "release, but it is not a pipe");
    Check(sim_conduit_aggregate(kGas, 0, &agg, sizeof(agg)) == 1 && agg.generation == 2 &&
              agg.policyFlags == 0 &&
              sim_conduit_aggregate(kGas, 1, &agg, sizeof(agg)) == 0,
          "the rebind bumps the generation, drops run 0's policy with the old numbering, and "
          "run 1 -- not in the new bind -- no longer exists");

    conduit_clear();
    Check(sim_conduit_aggregate(kLiquid, 0, &agg, sizeof(agg)) == 0,
          "ConduitTemperatureManager_Clear takes every binding with it");
  }

  // The third part: PRESSURE-DRIVEN PHASE CHANGE IN A RUN, decided by the sim. The
  // managed PipeMatterFacade.TickNetworkPhaseChange / CondenseHeadspaceGas made these decisions
  // per run from outside; under the PHASE policy the sim makes them on its own 200 ms update and
  // publishes each as a proposal, because the mass it would move belongs to ConduitFlow and to
  // the managed side-car. Every expected number below is re-derived here from the same inputs
  // -- the vapour curve, the ideal-gas pressure through SIM_ComputeGasPressure, and the step
  // through SIM_ComputePhaseChangeStep -- so a proposal that disagrees names its own defect.
  printf("\n=== vftest: conduit runs -- PHASE policy, trapped mirror, proposals ===\n");
  {
#pragma pack(push, 4)
    struct BuildingTemp {
      int32_t handle;
      float temperature;
    };
#pragma pack(pop)
    constexpr int32_t kGas = 1, kLiquid = 2;
    constexpr float kGasVolume = 0.01f, kLiquidVolume = 0.02f;
    constexpr float kGasMax = 1.0f, kLiquidMax = 10.0f;
    constexpr float kHC = 1000.0f, kK = 1.0f;
    constexpr float kRate = 0.1f, kMinRemainder = 0.01f;
    uint16_t steam = 0, water = 0;
    if (!Resolve(t, simhost::kSteam, "Steam", &steam)) return 1;
    if (!Resolve(t, kWater, "Water", &water)) return 1;

    // Water's Stationeers curve as MaterialPropertyRegistry holds it, resolved the way the
    // framework will push it: A, B, the curve at 6300 Pa (freezing) and at 6 MPa (critical),
    // and 8000 J/mol over water's 18.01528 g/mol.
    const double kA = 3.8782059839e-19, kB = 7.90030107708;
    auto curve_at = [&](double pa) { return std::pow(pa / 1000.0 / kA, 1.0 / kB); };
    const float freezing_k = static_cast<float>(curve_at(6300.0));
    const float critical_k = static_cast<float>(curve_at(6000000.0));
    const float latent = 8000.0f / 18.01528f * 1000.0f;
    auto clamped = [&](float pa) {
      if (!(pa > 0.0f)) return static_cast<double>(freezing_k);
      return std::min(std::max(curve_at(pa), static_cast<double>(freezing_k)),
                      static_cast<double>(critical_k));
    };
    const int32_t curve_attr = sim_ext_attr_index(ext::kAttrPhaseCurve);
    const int32_t density_attr = sim_ext_attr_index(ext::kAttrLiquidDensity);
    Check(curve_attr >= 0 && density_attr >= 0, "both phase-change attributes resolve by name");
    const float curve[5] = {static_cast<float>(kA), static_cast<float>(kB), freezing_k, critical_k,
                            latent};
    for (int32_t id : {static_cast<int32_t>(simhost::kSteam), static_cast<int32_t>(kWater)}) {
      for (int32_t k = 0; k < 5; ++k) {
        uint32_t bits = 0;
        memcpy(&bits, &curve[k], 4);
        SetElementAttributeBits(curve_attr, id, k, bits);
      }
    }
    SetElementAttributeF32(density_attr, kWater, 998.0f);

    conduit_clear();
    BuildingTemp bt[8];
    for (int32_t i = 0; i < 8; ++i) bt[i] = BuildingTemp{i, 300.0f};
    // Gas run 0 (PHASE): 2 x 1 kg steam at 380 K -- about 17 MPa in 20 L, past water's critical
    // pressure, so its dew point is clamped to the critical temperature and it condenses.
    // Gas run 1 (no policy): the same steam; must propose nothing. Gas run 2 (PHASE): 1 g of
    // steam at 380 K in one conduit and a second, EMPTY conduit, a few kPa over the two -- above
    // its dew point, so it must not condense, and water standing in either conduit evaporates.
    const int32_t g0a = conduit_add(380.0f, 1.0f, simhost::kSteam, 0, kHC, kK, 0);
    const int32_t g0b = conduit_add(380.0f, 1.0f, simhost::kSteam, 1, kHC, kK, 0);
    const int32_t g1 = conduit_add(380.0f, 1.0f, simhost::kSteam, 2, kHC, kK, 0);
    const int32_t g2 = conduit_add(380.0f, 0.001f, simhost::kSteam, 3, kHC, kK, 0);
    const int32_t g2e = conduit_add(380.0f, 0.0f, simhost::kSteam, 7, kHC, kK, 0);
    // Liquid run 0 (PHASE): 5 kg water at 350 K, no headspace gas -- an unpressurised line, so
    // its boiling point is its freezing point and it boils. Liquid run 1 (PHASE): 5 kg water at
    // 300 K with 0.5 kg of steam at 290 K standing in it; the headspace pressure puts the
    // boiling point far above 300 K and the dew point far above 290 K, so the water stays and
    // the steam condenses back. Liquid run 2 (PHASE): the same, with 9.9 kg of water -- the
    // condensation is clamped to the 0.1 kg the conduit still has room for.
    const int32_t l0 = conduit_add(350.0f, 5.0f, kWater, 4, kHC, kK, 0);
    const int32_t l1 = conduit_add(300.0f, 5.0f, kWater, 5, kHC, kK, 0);
    const int32_t l2 = conduit_add(300.0f, 9.9f, kWater, 6, kHC, kK, 0);
    bt[0].temperature = bt[1].temperature = bt[2].temperature = bt[3].temperature = 380.0f;
    bt[4].temperature = 350.0f;
    const int32_t gh[] = {g0a, g0b, g1, g2, g2e};
    const int32_t gc[] = {100, 101, 110, 120, 121};
    const int32_t gn[] = {0, 0, 1, 2, 2};
    const int32_t lh[] = {l0, l1, l2};
    const int32_t lc[] = {200, 210, 220};
    const int32_t ln[] = {0, 1, 2};
    // No neighbours: nothing in this arm mixes.
    int32_t no_neigh[5 * 4];
    for (int32_t& n : no_neigh) n = -1;
    Check(sim_conduit_bind(kGas, gh, gc, gn, no_neigh, 5, kGasVolume, kGasMax) == 5 &&
              sim_conduit_bind(kLiquid, lh, lc, ln, no_neigh, 3, kLiquidVolume, kLiquidMax) == 3,
          "three gas runs and three liquid runs bind");

    // The mirror.
    const float nan = std::numeric_limits<float>::quiet_NaN();
    Check(sim_conduit_trapped_set(3, 210, steam, 0.5f, 290.0f) == 0 &&
              sim_conduit_trapped_set(kLiquid, -1, steam, 0.5f, 290.0f) == 0 &&
              sim_conduit_trapped_set(kLiquid, 210, 60000, 0.5f, 290.0f) == 0 &&
              sim_conduit_trapped_set(kLiquid, 210, steam, nan, 290.0f) == 0 &&
              sim_conduit_trapped_set(kLiquid, 210, steam, 0.5f, nan) == 0,
          "the mirror refuses a solid type, a negative cell, an element outside the table and a "
          "non-finite mass or temperature");
    Check(sim_conduit_trapped_set(kLiquid, 210, steam, 0.5f, 290.0f) == 1 &&
              sim_conduit_trapped_set(kLiquid, 220, steam, 0.5f, 290.0f) == 1,
          "standing steam is mirrored into liquid runs 1 and 2, by cell");
    // Standing WATER in gas conduits, for EVAPORATE_TRAPPED:
    // in the condensing run 0, where it must stay; in the run with no policy; and in both
    // conduits of run 2 -- under 380 K steam, and in the empty conduit at its own 350 K.
    Check(sim_conduit_trapped_set(kGas, 100, water, 0.5f, 380.0f) == 1 &&
              sim_conduit_trapped_set(kGas, 110, water, 0.5f, 380.0f) == 1 &&
              sim_conduit_trapped_set(kGas, 120, water, 0.5f, 300.0f) == 1 &&
              sim_conduit_trapped_set(kGas, 121, water, 0.3f, 350.0f) == 1,
          "standing water is mirrored into gas runs 0, 1 and 2, by cell");

    OniConduitNetworkAggregate agg;
    Check(sim_conduit_aggregate(kLiquid, 1, &agg, sizeof(agg)) == 1, "liquid run 1 is summed");
    const double liquid_m3 = 5.0 / 998.0;
    const double headspace_m3 = kLiquidVolume - liquid_m3;
    const uint16_t steam_species[] = {steam};
    const float steam_mass[] = {0.5f};
    const float expected_headspace = sim_compute_gas_pressure(
        steam_species, steam_mass, 1, 290.0f, static_cast<float>(headspace_m3));
    printf("  liquid run 1: %.4f kg water, trapped gas %.4f kg, liquid %.6f m3, headspace "
           "%.0f Pa (expected %.0f)\n",
           agg.totalMassKg, agg.trappedGasKg, agg.liquidVolumeM3, agg.headspacePressurePa,
           expected_headspace);
    Check(std::fabs(agg.trappedGasKg - 0.5f) < 1e-6f && agg.trappedLiquidKg == 0.0f &&
              agg.totalMassKg == 5.0f,
          "the trapped steam is reported apart from the conduit's own 5 kg of water");
    Check(std::fabs(agg.liquidVolumeM3 - liquid_m3) < 1e-7,
          "the liquid volume uses sim.liquid_density (998 kg/m3), not the water fallback");
    Check(expected_headspace > 0.0f &&
              std::fabs(agg.headspacePressurePa - expected_headspace) < 1e-4f * expected_headspace,
          "the headspace pressure is the trapped steam over the room the water leaves -- "
          "PipeMatterFacade.TryGetLiquidHeadspacePressurePa's construction, natively");
    Check(agg.pressurePa == 0.0f, "and a liquid run still reports no ideal-gas pressure of its own");
    SetElementAttributeBits(density_attr, kWater, 0, 0, /*clear=*/true);
    Check(sim_conduit_aggregate(kLiquid, 1, &agg, sizeof(agg)) == 1 &&
              std::fabs(agg.liquidVolumeM3 - 5.0 / 1000.0) < 1e-7,
          "with no density registered, CONDUIT liquid counts as water-dense (1000 kg/m3) -- the "
          "fallback PipeNetworkFacade.LiquidDensityOf uses");
    SetElementAttributeF32(density_attr, kWater, 998.0f);

    // Policy refusals for PHASE.
    Check(sim_conduit_policy(kGas, 0, ONI_CONDUIT_POLICY_PHASE, 0.0f, 0.0f, kMinRemainder) == 0 &&
              sim_conduit_policy(kGas, 0, ONI_CONDUIT_POLICY_PHASE, 0.0f, nan, kMinRemainder) == 0 &&
              sim_conduit_policy(kGas, 0, ONI_CONDUIT_POLICY_PHASE, 0.0f, kRate, -1.0f) == 0,
          "PHASE is refused with a zero or NaN rate, or a negative remainder");
    Check(sim_conduit_policy(kGas, 0, ONI_CONDUIT_POLICY_PHASE, 0.0f, kRate, kMinRemainder) == 1 &&
              sim_conduit_policy(kGas, 2, ONI_CONDUIT_POLICY_PHASE, 0.0f, kRate, kMinRemainder) == 1 &&
              sim_conduit_policy(kLiquid, 0, ONI_CONDUIT_POLICY_PHASE, 0.0f, kRate, kMinRemainder) == 1 &&
              sim_conduit_policy(kLiquid, 1, ONI_CONDUIT_POLICY_PHASE, 0.0f, kRate, kMinRemainder) == 1 &&
              sim_conduit_policy(kLiquid, 2, ONI_CONDUIT_POLICY_PHASE, 0.0f, kRate, kMinRemainder) == 1,
          "PHASE is accepted on gas runs 0 and 2 and on all three liquid runs");

    OniConduitNetworkAggregate gas0, gas2;
    Check(sim_conduit_aggregate(kGas, 0, &gas0, sizeof(gas0)) == 1 &&
              sim_conduit_aggregate(kGas, 2, &gas2, sizeof(gas2)) == 1 &&
              gas0.policyFlags == ONI_CONDUIT_POLICY_PHASE && gas0.phaseRatePerSecond == kRate,
          "the aggregate reports the PHASE policy and its rate");

    conduit_update(0.2f, bt);
    OniConduitPhaseProposal props[16];
    memset(props, 0xCD, sizeof(props));
    const int32_t total = sim_conduit_proposals(props, 16);
    Check(sim_conduit_proposals(nullptr, 0) == total, "the sizing call reports the same total");
    printf("  %d proposals after one update; gas run 0 at %.0f Pa (dew point %.2f K), gas run 2 at "
           "%.0f Pa (dew point %.2f K)\n",
           total, gas0.pressurePa, clamped(gas0.pressurePa), gas2.pressurePa,
           clamped(gas2.pressurePa));
    for (int32_t i = 0; i < total && i < 16; ++i) {
      printf("    type %d run %d cell %d kind %d: %u -> %u, %.6f of %.4f kg at %.2f K, boundary "
             "%.3f K at %.0f Pa\n",
             props[i].conduitType, props[i].networkId, props[i].cell, props[i].kind,
             props[i].sourceElementIdx, props[i].productElementIdx, props[i].convertedMassKg,
             props[i].sourceMassKg, props[i].sourceTemperatureK, props[i].boundaryK,
             props[i].pressurePa);
    }
    auto find = [&](int32_t type, int32_t cell, int32_t kind) -> const OniConduitPhaseProposal* {
      for (int32_t i = 0; i < total && i < 16; ++i) {
        if (props[i].conduitType == type && props[i].cell == cell && props[i].kind == kind) {
          return &props[i];
        }
      }
      return nullptr;
    };
    Check(total == 7,
          "seven decisions: two condensations in gas run 0, one boil in liquid run 0, one "
          "headspace return in each of liquid runs 1 and 2, and water evaporating in both "
          "conduits of gas run 2 -- nothing from the run with no policy, the steam above its dew "
          "point, the water below its boiling point, or the water standing where steam condenses");

    const OniConduitPhaseProposal* c0 = find(kGas, 100, ONI_CONDUIT_PHASE_CONDENSE_CONTENTS);
    const OniConduitPhaseProposal* c1 = find(kGas, 101, ONI_CONDUIT_PHASE_CONDENSE_CONTENTS);
    Check(c0 && c1, "both steam conduits of gas run 0 are proposed to condense");
    if (c0 && c1) {
      Check(c0->sourceElementIdx == steam && c0->productElementIdx == water &&
                c0->networkId == 0 && c0->sourceMassKg == 1.0f && c0->sourceTemperatureK == 380.0f,
            "steam becomes water -- the element's own low transition, not a name guessed at");
      Check(std::fabs(c0->pressurePa - gas0.pressurePa) < 1e-3f * gas0.pressurePa,
            "the pressure behind the decision is the run's own, the one the aggregate reports");
      Check(gas0.pressurePa > 6000000.0f && std::fabs(c0->boundaryK - critical_k) < 1e-3f,
            "past the critical pressure the dew point is clamped to the critical temperature");
      float expected_kg = -1.0f, remaining_k = 0.0f;
      sim_compute_phase_change_step(1.0f, 380.0f, c0->boundaryK, latent,
                                    t.SpecificHeat(steam) * 1000.0f, 0.2f, kRate, kMinRemainder,
                                    &expected_kg, &remaining_k);
      Check(expected_kg > 0.0f && c0->convertedMassKg == expected_kg && c1->convertedMassKg == expected_kg,
            "the amount is exactly SIM_ComputePhaseChangeStep's for the same state at dt 0.2 -- "
            "the same phase-change step the managed pass called");
      Check(c0->latentHeatJPerKg == latent, "and it carries the latent heat it will release");
    }
    Check(find(kGas, 110, ONI_CONDUIT_PHASE_CONDENSE_CONTENTS) == nullptr,
          "gas run 1 has no policy and proposes nothing, though it holds the same steam");
    Check(clamped(gas2.pressurePa) < 380.0 &&
              find(kGas, 120, ONI_CONDUIT_PHASE_CONDENSE_CONTENTS) == nullptr,
          "gas run 2's thin steam is above its own dew point and does not condense");

    // EVAPORATE_TRAPPED, the gas run's return leg. Judged on the conduit's contents when it has
    // any (cell 120: the 380 K steam, not the 300 K water) and on the water's own temperature
    // when it has none (cell 121: 350 K), against the VAPOUR's curve at the run's pressure --
    // the boundary condensation uses -- with the step run on the standing water's own mass and
    // heat capacity.
    Check(find(kGas, 100, ONI_CONDUIT_PHASE_EVAPORATE_TRAPPED) == nullptr,
          "water standing in gas run 0 stays: its conduit's steam is below the dew point, the "
          "same tile is condensing, and one tile is never proposed both ways");
    Check(find(kGas, 110, ONI_CONDUIT_PHASE_EVAPORATE_TRAPPED) == nullptr,
          "gas run 1 has no policy, so its standing water is left to whoever else decides");
    const OniConduitPhaseProposal* e0 = find(kGas, 120, ONI_CONDUIT_PHASE_EVAPORATE_TRAPPED);
    const OniConduitPhaseProposal* e1 = find(kGas, 121, ONI_CONDUIT_PHASE_EVAPORATE_TRAPPED);
    Check(e0 && e1, "the water in both conduits of gas run 2 is proposed to evaporate");
    if (e0 && e1) {
      Check(e0->sourceElementIdx == water && e0->productElementIdx == steam &&
                e0->networkId == 2 && e0->sourceMassKg == 0.5f && e1->sourceMassKg == 0.3f,
            "water becomes steam -- the element's own high transition -- and the decision "
            "carries the standing mass it was made against");
      Check(std::fabs(e0->pressurePa - gas2.pressurePa) < 1e-3f * gas2.pressurePa &&
                std::fabs(e0->boundaryK - clamped(gas2.pressurePa)) < 1e-3f,
            "the boundary is the vapour's dew point at the run's own pressure");
      Check(e0->sourceTemperatureK == 380.0f && e1->sourceTemperatureK == 350.0f,
            "judged on the contents' 380 K where the conduit holds steam, and on the water's own "
            "350 K where it holds nothing");
      float expected0 = -1.0f, expected1 = -1.0f, remaining_k = 0.0f;
      sim_compute_phase_change_step(0.5f, 380.0f, e0->boundaryK, latent,
                                    t.SpecificHeat(water) * 1000.0f, 0.2f, kRate, kMinRemainder,
                                    &expected0, &remaining_k);
      sim_compute_phase_change_step(0.3f, 350.0f, e1->boundaryK, latent,
                                    t.SpecificHeat(water) * 1000.0f, 0.2f, kRate, kMinRemainder,
                                    &expected1, &remaining_k);
      Check(expected0 > 0.0f && e0->convertedMassKg == expected0 &&
                expected1 > 0.0f && e1->convertedMassKg == expected1,
            "the amounts are SIM_ComputePhaseChangeStep's for the standing water at dt 0.2");
      Check(e0->latentHeatJPerKg == latent, "and it carries the latent heat it will absorb");
    }

    const OniConduitPhaseProposal* b0 = find(kLiquid, 200, ONI_CONDUIT_PHASE_BOIL_CONTENTS);
    Check(b0 && b0->sourceElementIdx == water && b0->productElementIdx == steam &&
              b0->pressurePa == 0.0f && std::fabs(b0->boundaryK - freezing_k) < 1e-3f,
          "liquid run 0 has no headspace gas, so its boiling point is its freezing point and its "
          "350 K water boils -- the unpressurised line Stationeers warns about");
    Check(find(kLiquid, 210, ONI_CONDUIT_PHASE_BOIL_CONTENTS) == nullptr,
          "liquid run 1's water, under its headspace pressure, does not boil");

    const OniConduitPhaseProposal* h1 = find(kLiquid, 210, ONI_CONDUIT_PHASE_CONDENSE_HEADSPACE);
    const OniConduitPhaseProposal* h2 = find(kLiquid, 220, ONI_CONDUIT_PHASE_CONDENSE_HEADSPACE);
    Check(h1 && h1->sourceElementIdx == steam && h1->productElementIdx == water &&
              std::fabs(h1->pressurePa - expected_headspace) < 1e-3f * expected_headspace &&
              std::fabs(h1->convertedMassKg - 0.5f) < 1e-6f,
          "liquid run 1's standing steam condenses back -- all 0.5 kg, because what the step "
          "would leave is below the remainder floor");
    Check(h2 && std::fabs(h2->convertedMassKg - 0.1f) < 1e-5f,
          "liquid run 2's is clamped to the 0.1 kg of room its conduit has left");

    // The mirror survives a rebind (keyed by cell), and removal and clear behave.
    Check(sim_conduit_bind(kLiquid, lh, lc, ln, no_neigh, 3, kLiquidVolume, kLiquidMax) == 3 &&
              sim_conduit_aggregate(kLiquid, 1, &agg, sizeof(agg)) == 1 &&
              std::fabs(agg.trappedGasKg - 0.5f) < 1e-6f,
          "a rebind keeps the mirrored matter: it is keyed by cell, and so is the side-car");
    Check(sim_conduit_trapped_set(kLiquid, 210, steam, 0.0f, 290.0f) == 1 &&
              sim_conduit_aggregate(kLiquid, 1, &agg, sizeof(agg)) == 1 &&
              agg.trappedGasKg == 0.0f && agg.headspacePressurePa == 0.0f,
          "a zero mass removes the entry, and the headspace pressure goes with it");
    Check(sim_conduit_trapped_clear(kLiquid) == 1 && sim_conduit_trapped_clear(3) == -1,
          "TrappedClear drops the one remaining entry and refuses an unknown type");
    Check(sim_conduit_trapped_clear(kGas) == 4, "and the gas store's four go the same way");

    conduit_update(0.0f, bt);
    Check(sim_conduit_proposals(nullptr, 0) == 0,
          "a dt = 0 update (RebuildConnections publishing new handles) leaves no proposals");
    conduit_clear();
    SetElementAttributeBits(density_attr, kWater, 0, 0, /*clear=*/true);
    for (int32_t id : {static_cast<int32_t>(simhost::kSteam), static_cast<int32_t>(kWater)}) {
      SetElementAttributeBits(curve_attr, id, 0, 0, /*clear=*/true);
    }
  }

  // The CONVECTION conduit policy: Stationeers' pipe-to-world shape on Klei's
  // coefficient. Every expectation is derived from a same-cell run WITHOUT the policy (Klei's
  // own update, measured here rather than recomputed) times the two ratios, and each ratio from
  // SIM_ComputeGasPressure on the frame the game holds -- so nothing below restates the
  // arithmetic it is checking.
  printf("\n=== vftest: conduit CONVECTION policy -- pipe and cell HeatExchangeRatio ===\n");
  sim_shutdown();
  InitSim();
  {
#pragma pack(push, 4)
    struct ConduitUpdate {
      int32_t numEntries;
      float* temperatures;
      int32_t numFrozenHandles;
      int32_t* frozenHandles;
      int32_t numMeltedHandles;
      int32_t* meltedHandles;
    };
    struct BuildingTemp {
      int32_t handle;
      float temperature;
    };
#pragma pack(pop)
    constexpr int32_t kGas = 1, kLiquid = 2;
    constexpr float kGasVolume = 0.01f, kLiquidVolume = 0.02f;
    constexpr float kGasMax = 1.0f, kLiquidMax = 10.0f;
    // 1 kJ/K, not the 1000 the phase-change arms use: Klei's building-side limit is
    // |reach - T| * C with reach a float near 300 K, so a pipe that big quantises a quarter-kJ
    // transfer to whole ulps of 300 K (3e-5 K x 1000 = 0.03 kJ) and the limit, not the ratio,
    // decides the answer. Found by this arm's first run: 0.2441 kJ where 0.2560 was proposed.
    constexpr float kConduitHC = 1.0f, kConduitK = 1.0f;
    constexpr float kOneAtm = 101324.99694824219f;
    auto slot = [](int32_t h) { return static_cast<size_t>(h & 0x00FFFFFF); };

    uint16_t cv_granite = 0, cv_water = 0;
    if (!Resolve(t, kGranite, "Granite", &cv_granite)) return 1;
    if (!Resolve(t, simhost::kWater, "Water", &cv_water)) return 1;

    // One row of sealed pockets in granite, every pocket walled off from the next, so the
    // frames stepped below move nothing: (1,2) vacuum, (3,2) 2 kg oxygen (well over one
    // atmosphere in the sim's 1 m3 cell), (5,2) 0.65 kg oxygen (about half of one), (7,2) water,
    // (9,2) granite itself.
    constexpr int kCW = 11, kCH = 5;
    constexpr int kVac = 2 * kCW + 1, kFull = 2 * kCW + 3, kHalf = 2 * kCW + 5,
                  kWet = 2 * kCW + 7, kRock = 2 * kCW + 9;
    World cv_world;
    cv_world.Init(kCW, kCH, cv_granite, 2000.0f, 300.0f);
    cv_world.element[kVac] = vacuum;
    cv_world.mass[kVac] = 0.0f;
    cv_world.element[kFull] = oxygen;
    cv_world.mass[kFull] = 2.0f;
    cv_world.element[kHalf] = oxygen;
    cv_world.mass[kHalf] = 0.65f;
    cv_world.element[kWet] = cv_water;
    cv_world.mass[kWet] = 1000.0f;
    std::vector<uint8_t> cv_visible(static_cast<size_t>(kCW * kCH), 1);
    Boot(t, cv_world);
    Tick(cv_world, &cv_visible);
    const GameDataUpdate* held = Tick(cv_world, &cv_visible);
    Check(held != nullptr && held->mass[kVac] == 0.0f && held->elementIdx[kFull] == oxygen &&
              held->elementIdx[kHalf] == oxygen && held->elementIdx[kWet] == cv_water &&
              held->elementIdx[kRock] == cv_granite,
          "the held frame shows the five pockets as built");

    auto cell_ratio = [&](int cell) {
      const uint16_t sp[] = {oxygen};
      const float m[] = {held->mass[cell]};
      const float p = sim_compute_gas_pressure(sp, m, 1, held->temperature[cell], 1.0f);
      return std::min(std::max(p / kOneAtm, 0.0f), 1.0f);
    };
    const float r_full = cell_ratio(kFull);
    const float r_half = cell_ratio(kHalf);
    printf("  cell ratios from the held frame: 2 kg O2 %.6f, 0.65 kg O2 %.6f\n", r_full, r_half);
    Check(r_full == 1.0f && r_half > 0.45f && r_half < 0.55f,
          "2 kg of oxygen saturates the cell ratio and 0.65 kg is about half an atmosphere");

    // Gas runs, one conduit each, every building at 300 K. 1 kg O2 at 400 K in 10 L is far past
    // one atmosphere, so its run ratio is 1; the 1 g conduit's is its pressure over 1 atm.
    constexpr int32_t kRuns = 9;
    BuildingTemp bt[kRuns + 3];
    for (int32_t i = 0; i < kRuns + 3; ++i) bt[i] = BuildingTemp{i, 300.0f};
    conduit_clear();
    const float gas_mass[kRuns] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.001f, 1.0f, 0.001f, 1.0f};
    const int32_t gas_cell[kRuns] = {kVac, kFull, kHalf, kWet, kRock, kFull, kFull, kFull, kHalf};
    int32_t gh[kRuns], gn[kRuns], gneigh[kRuns * 4];
    for (int32_t i = 0; i < kRuns; ++i) {
      gh[i] = conduit_add(400.0f, gas_mass[i], kOxygenHash, i, kConduitHC, kConduitK, 0);
      gn[i] = i;
    }
    for (int32_t& n : gneigh) n = -1;
    // Liquid: 1 kg water at 350 K (a twentieth of the 20 L conduit, so its own ratio saturates)
    // over the vacuum pocket (convecting), over the half pocket (convecting), and over the half
    // pocket without the policy. 1 kg rather than 10 so its drop is many ulps of 350 K.
    const int32_t lh[3] = {
        conduit_add(350.0f, 1.0f, simhost::kWater, kRuns + 0, kConduitHC, kConduitK, 0),
        conduit_add(350.0f, 1.0f, simhost::kWater, kRuns + 1, kConduitHC, kConduitK, 0),
        conduit_add(350.0f, 1.0f, simhost::kWater, kRuns + 2, kConduitHC, kConduitK, 0)};
    const int32_t lc[3] = {kVac, kHalf, kHalf};
    const int32_t ln[3] = {0, 1, 2};
    int32_t lneigh[12];
    for (int32_t& n : lneigh) n = -1;
    Check(sim_conduit_bind(kGas, gh, gas_cell, gn, gneigh, kRuns, kGasVolume, kGasMax) == kRuns &&
              sim_conduit_bind(kLiquid, lh, lc, ln, lneigh, 3, kLiquidVolume, kLiquidMax) == 3,
          "nine one-conduit gas runs and three liquid runs bind");

    // Runs 6, 7 and liquid 2 are the controls: no policy, Klei's update alone.
    bool accepted = true;
    for (int32_t r : {0, 1, 2, 3, 4, 5, 8}) {
      accepted = accepted &&
                 sim_conduit_policy(kGas, r, ONI_CONDUIT_POLICY_CONVECTION, 0.0f, 0.0f, 0.0f) == 1;
    }
    accepted = accepted &&
               sim_conduit_policy(kLiquid, 0, ONI_CONDUIT_POLICY_CONVECTION, 0.0f, 0.0f, 0.0f) == 1 &&
               sim_conduit_policy(kLiquid, 1, ONI_CONDUIT_POLICY_CONVECTION, 0.0f, 0.0f, 0.0f) == 1;
    Check(accepted, "CONVECTION is accepted with no parameters, on gas and liquid runs alike");
    Check(sim_conduit_policy(kGas, 8, ONI_CONDUIT_POLICY_CONVECTION | ONI_CONDUIT_POLICY_MIX,
                             0.0f, 0.0f, 0.0f) == 0,
          "combined with MIX it still needs MIX's coupling, and is refused without one");
    OniConduitNetworkAggregate agg;
    Check(sim_conduit_aggregate(kGas, 1, &agg, sizeof(agg)) == 1 &&
              agg.policyFlags == ONI_CONDUIT_POLICY_CONVECTION,
          "the aggregate reports CONVECTION in force");

    const ConduitUpdate* u = static_cast<const ConduitUpdate*>(conduit_update(0.2f, bt));
    auto drop = [&](int32_t h, float from) { return from - u->temperatures[slot(h)]; };
    const double klei_full = drop(gh[6], 400.0f);
    const double klei_small = drop(gh[7], 400.0f);
    const double klei_liquid = drop(lh[2], 350.0f);
    printf("  Klei's own drops: 1 kg %.6f K, 1 g %.6f K, water %.6f K\n", klei_full, klei_small,
           klei_liquid);
    Check(klei_full > 0.01 && klei_small > 0.01 && klei_liquid > 0.01,
          "the controls move, so every ratio below scales something measurable");

    Check(u->temperatures[slot(gh[0])] == 400.0f,
          "a pipe over vacuum does not convect: the cell ratio is 0, the conduit keeps its "
          "temperature and bills its building nothing");
    Check(u->temperatures[slot(gh[1])] == u->temperatures[slot(gh[6])],
          "over a cell past one atmosphere, with a run past one atmosphere, both ratios are 1 "
          "and the update is Klei's to the bit");
    printf("  half-atmosphere cell: gas drop %.6f K (Klei %.6f x %.6f = %.6f), water drop %.6f K "
           "(Klei %.6f x %.6f = %.6f)\n",
           drop(gh[2], 400.0f), klei_full, r_half, klei_full * r_half, drop(lh[1], 350.0f),
           klei_liquid, r_half, klei_liquid * r_half);
    Check(std::fabs(drop(gh[2], 400.0f) - klei_full * r_half) < 1e-3 * klei_full,
          "over half an atmosphere the exchange is Klei's times the cell's pressure ratio");
    Check(u->temperatures[slot(gh[3])] == u->temperatures[slot(gh[6])],
          "over a liquid cell the ratio saturates (a thousandth of the cell's volume is enough)");
    Check(u->temperatures[slot(gh[4])] == u->temperatures[slot(gh[6])],
          "inside a solid tile the ratio is 1, not Stationeers' 0: a pipe through a tile touches "
          "matter, and ONI's conduction owns that exchange");
    const uint16_t sp[] = {oxygen};
    const float small_mass[] = {0.001f};
    const float r_small =
        std::min(sim_compute_gas_pressure(sp, small_mass, 1, 400.0f, kGasVolume) / kOneAtm, 1.0f);
    // The 1 g control is no guide here: its heat capacity is so small that Klei's own limit
    // lands it on the building's temperature (the whole 100 K). The PROPOSAL is what the ratio
    // scales, and it is the 1 kg conduit's -- same conductivities, same gap -- so the expected
    // drop is the 1 kg drop, times the ratio, over a thousandth of the heat capacity.
    const double expected_small = klei_full * (1.0 / 0.001) * r_small;
    printf("  1 g run ratio %.6f: drop %.6f K against the 1 kg proposal x ratio / (1 g's heat "
           "capacity) = %.6f K (Klei's own 1 g drop, limited: %.6f K)\n",
           r_small, drop(gh[5], 400.0f), expected_small, klei_small);
    Check(r_small > 0.05f && r_small < 0.5f && expected_small < 100.0 &&
              std::fabs(drop(gh[5], 400.0f) - expected_small) < 1e-3 * expected_small,
          "a run under one atmosphere convects in proportion to its own pressure");
    Check(u->temperatures[slot(lh[0])] == 350.0f,
          "a full liquid pipe over vacuum does not convect either: its own ratio is 1, the "
          "cell's is 0");
    Check(std::fabs(drop(lh[1], 350.0f) - klei_liquid * r_half) < 1e-3 * klei_liquid,
          "a liquid run's own ratio saturates, so over half an atmosphere it is Klei's times "
          "the cell's ratio alone");

    // The conduit-side half of ext::kSetBuildingConvection. A building that
    // convects with its own cells owns that cell ratio; its conduit must not apply it a second
    // time, and over vacuum -- where the cell ratio is 0 -- applying it would leave a radiator's
    // contents unable to reach the body that is supposed to radiate them away. Run 0 is the pipe
    // over the vacuum pocket, which measured exactly 0 above.
    {
      ext::SetBuildingConvectionMessage bc{};
      bc.handle = 0;  // run 0's structure handle, the vacuum pocket's pipe
      bc.convectionFactor = 1.0f;
      bc.surfaceAreaM2 = 0.0f;
      sim_handle_message(ext::kSetBuildingConvection, static_cast<int>(sizeof(bc)),
                         reinterpret_cast<const uint8_t*>(&bc));
      conduit_set(gh[0], 400.0f, 1.0f, kOxygenHash);
      conduit_set(gh[6], 400.0f, 1.0f, kOxygenHash);
      u = static_cast<const ConduitUpdate*>(conduit_update(0.2f, bt));
      const double with_building = drop(gh[0], 400.0f);
      printf("  radiator pipe over vacuum: drop %.6f K against Klei's own %.6f K\n",
             with_building, klei_full);
      Check(std::fabs(with_building - klei_full) < 1e-6 * klei_full,
            "a pipe whose building convects for itself keeps the run ratio and drops the cell "
            "ratio, so over vacuum it is Klei's update instead of nothing");

      bc.convectionFactor = 0.0f;
      sim_handle_message(ext::kSetBuildingConvection, static_cast<int>(sizeof(bc)),
                         reinterpret_cast<const uint8_t*>(&bc));
      conduit_set(gh[0], 400.0f, 1.0f, kOxygenHash);
      u = static_cast<const ConduitUpdate*>(conduit_update(0.2f, bt));
      Check(u->temperatures[slot(gh[0])] == 400.0f,
            "and switching the building term off puts the cell ratio back: vacuum moves nothing "
            "again");
    }

    // A policy cleared is Klei's update again.
    Check(sim_conduit_policy(kGas, 2, 0, 0.0f, 0.0f, 0.0f) == 1, "clearing run 2's policy");
    conduit_set(gh[2], 400.0f, 1.0f, kOxygenHash);
    conduit_set(gh[6], 400.0f, 1.0f, kOxygenHash);
    u = static_cast<const ConduitUpdate*>(conduit_update(0.2f, bt));
    Check(u->temperatures[slot(gh[2])] == u->temperatures[slot(gh[6])],
          "with the policy cleared the half-atmosphere run is Klei's update exactly");
    conduit_clear();
  }

  // Prove the actual point of promotion, not just that the bit flips: a kernel that reads
  // `RoomGraph.owned`. This is that kernel, StepGasPressure,
  // and the property under test is "a promoted room's mass stops moving via vanilla pressure
  // flow." Fresh world, not facade_world: needs a clean, controllable vanilla mass gradient
  // (facade_world's vanilla PhaseEntry mass was never touched by the dominant-facade checks
  // above, only its separate gas-mixture SoA layer was).
  printf("\n=== vftest: promotion gates vanilla pressure flow "
         "===\n");
  sim_shutdown();
  InitSim();
  // 5 wide x 3 tall, working in the middle row (y=1): the gravity test above found live that
  // a too-small world can put the only real gas cells on a border row that physics.h's
  // region math treats specially rather than as an ordinary interior row -- staying off row 0 and column 0 sidesteps that
  // here rather than re-deriving which edge case applies to a horizontal sweep specifically.
  constexpr int kW = 5, kCellA = 1 * kW + 1, kCellB = 1 * kW + 2;  // (1,1) and (2,1)
  World gate_world;
  gate_world.Init(kW, 3, oxygen, 0.0f, 300.0f);
  gate_world.mass[kCellA] = 4.0f;  // neighbour starts at 0 kg -- same element, so still "gas"
                                    // state per the element table, not "vacuum," which is
                                    // what lets vanilla's ordinary same-element flow apply.
  const GameDataUpdate* gv = Boot(t, gate_world);
  std::vector<uint8_t> gate_visible;

  // A trivial injection is the only thing that flips vf_active true (ApplyInjectGasSpecies,
  // simdll.cpp:776) -- ApplyPromoteRoom also self-activates, but landing this first keeps
  // the sequencing identical to every other section in this file. Same one-tick latency as
  // everywhere else; land it with a Tick before sending the promotion.
  InjectGas(kCellA, oxygen, 0.001f, 300.0f);
  // Two ticks, not one: the first frame does nothing, usually. Klei's worker thread races
  // PrepareGameData after Start/Load, so the frame that lands Boot()/the injection runs no
  // physics at all. Nothing specific to this gate; one extra tick of head start before the real assertions sidesteps it, same
  // as every other section in this file already does.
  Tick(gate_world, &gate_visible);
  gv = Tick(gate_world, &gate_visible);
  Check(gv && gv->mass[kCellB] > 0.0f,
        "sanity check: before promotion, vanilla pressure already moved mass A -> B on its "
        "own -- the gate below has something real to prevent");
  const float pre_promote_massA = gv->mass[kCellA];
  const float pre_promote_massB = gv->mass[kCellB];
  Check(pre_promote_massA > pre_promote_massB,
        "a real gradient still remains at promotion time -- an ungated kernel would keep "
        "narrowing it next tick");

  ext::PromoteRoomMessage gate_promote{kCellA};
  sim_handle_message(ext::kPromoteRoom, static_cast<int>(sizeof(gate_promote)),
                     reinterpret_cast<const uint8_t*>(&gate_promote));
  gv = Tick(gate_world, &gate_visible);
  Check(sim_debug_room_owned(kCellA) == 1,
        "cell A's room is promoted in the gate-test world too");

  // One more tick with zero messages in flight: if StepGasPressure's promotion gate works,
  // vanilla mass must be frozen exactly where it was the instant promotion landed --
  // regardless of whether the gate started applying the same tick as the promotion message
  // or the one after, which this loop doesn't need to distinguish.
  const float at_promote_massA = gv->mass[kCellA];
  const float at_promote_massB = gv->mass[kCellB];
  gv = Tick(gate_world, &gate_visible);
  Check(gv && gv->mass[kCellA] == at_promote_massA && gv->mass[kCellB] == at_promote_massB,
        "once promoted, vanilla pressure flow no longer moves mass between cells A and B, "
        "even though a real gradient remains between them");
  gv = Tick(gate_world, &gate_visible);
  Check(gv && gv->mass[kCellA] == at_promote_massA && gv->mass[kCellB] == at_promote_massB,
        "still frozen two ticks after promotion, not just one -- this isn't a one-tick "
        "coincidence");

  // Conduction: same property as the pressure gate above, different kernel --
  // "a promoted room's mass stops moving via vanilla pressure flow" becomes "a promoted
  // room's cells stop exchanging heat via vanilla conduction." `ApplyPromoteRoom` self-
  // activates volume-fractions on its own (simdll.cpp), so unlike the pressure test above
  // this one skips the `InjectGas` warm-up and promotes directly -- one fewer moving part,
  // same guarantee.
  printf("\n=== vftest: promotion gates vanilla conduction ===\n");
  sim_shutdown();
  InitSim();
  // Same 5x3 layout, same middle-row cells, same reason (avoid the border-row region quirk
  // the gravity test's own "Height 7, not 6" comment names) -- both cells hold real Oxygen
  // mass so neither trips the kernel's own vacuum early-out (`a.mass <= 0.0f`), and start
  // 100 K apart, far past the 1 K dead zone, so an ungated kernel has plenty to equalize.
  constexpr int kCW = 5, kCondA = 1 * kCW + 1, kCondB = 1 * kCW + 2;  // (1,1) and (2,1)
  World cond_world;
  cond_world.Init(kCW, 3, oxygen, 1.0f, 300.0f);
  cond_world.temperature[kCondA] = 400.0f;
  const GameDataUpdate* gc = Boot(t, cond_world);
  std::vector<uint8_t> cond_visible;
  gc = Tick(cond_world, &cond_visible);  // the first frame does nothing
  gc = Tick(cond_world, &cond_visible);  // head start, same as every other section here.
  Check(gc && gc->temperature[kCondB] > 300.0f,
        "sanity check: before promotion, vanilla conduction already warmed cell B on its own "
        "-- the gate below has something real to prevent");
  Check(gc && gc->temperature[kCondA] > gc->temperature[kCondB],
        "a real gradient still remains at promotion time");

  ext::PromoteRoomMessage cond_promote{kCondA};
  sim_handle_message(ext::kPromoteRoom, static_cast<int>(sizeof(cond_promote)),
                     reinterpret_cast<const uint8_t*>(&cond_promote));
  gc = Tick(cond_world, &cond_visible);
  Check(sim_debug_room_owned(kCondA) == 1, "cell A's room is promoted in the conduction-gate world too");

  const float at_promote_tempA = gc->temperature[kCondA];
  const float at_promote_tempB = gc->temperature[kCondB];
  gc = Tick(cond_world, &cond_visible);
  Check(gc && gc->temperature[kCondA] == at_promote_tempA && gc->temperature[kCondB] == at_promote_tempB,
        "once promoted, vanilla conduction no longer exchanges heat between cells A and B, "
        "even though a real gradient remains between them");
  gc = Tick(cond_world, &cond_visible);
  Check(gc && gc->temperature[kCondA] == at_promote_tempA && gc->temperature[kCondB] == at_promote_tempB,
        "still frozen two ticks after promotion, not just one -- this isn't a one-tick "
        "coincidence");

  // The other half of the gas gate: `StepGasPressure` refuses a pair of
  // two *different* gases outright (`sn == sc` check in `transfer`), so it is
  // `StepGasDisplacement` -- a separate kernel, gated separately -- that actually moves
  // oxygen and CO2 past each other. Gating pressure alone would leave this path wide open
  // for vanilla to overwrite a promoted cell. Geometry is diffsim.cpp's own `gasmix`
  // scenario's "straight displacement" triple (src/dst/beyond in a row, y=6, x=5..7),
  // reused rather than re-derived -- a Granite background is load-bearing here, not
  // decoration: it keeps the oxygen source cell from also feeding a vanilla *pressure* flow
  // out its other side into vacuum, which would confound what this test is isolating.
  printf("\n=== vftest: promotion gates vanilla gas displacement "
         "===\n");
  sim_shutdown();
  InitSim();
  uint16_t disp_granite = 0;
  if (!Resolve(t, kGranite, "Granite", &disp_granite)) return 1;
  constexpr int kDW = 24, kDH = 16;
  constexpr int kSrc = 6 * kDW + 5, kDst = 6 * kDW + 6, kBeyond = 6 * kDW + 7;  // (5,6)(6,6)(7,6)
  World disp_world;
  disp_world.Init(kDW, kDH, disp_granite, 2000.0f, 293.15f);
  disp_world.element[kSrc] = oxygen;
  disp_world.mass[kSrc] = 3.0f;
  disp_world.temperature[kSrc] = 310.0f;
  disp_world.element[kDst] = co2;
  disp_world.mass[kDst] = 1.0f;
  disp_world.temperature[kDst] = 290.0f;
  disp_world.element[kBeyond] = co2;
  disp_world.mass[kBeyond] = 1.0f;
  disp_world.temperature[kBeyond] = 290.0f;
  const GameDataUpdate* gd = Boot(t, disp_world);
  std::vector<uint8_t> disp_visible;
  gd = Tick(disp_world, &disp_visible);  // the first frame does nothing: head
  gd = Tick(disp_world, &disp_visible);  // start, same as every other section here.
  Check(gd && gd->elementIdx[kDst] == oxygen,
        "sanity check: before promotion, vanilla displacement already evicted cell B's CO2 "
        "into cell C and moved oxygen in behind it -- the gate below has something real to "
        "prevent");
  Check(gd && gd->elementIdx[kBeyond] == co2 && gd->mass[kBeyond] > 1.0f,
        "cell C received cell B's evicted CO2 on top of its own");

  // Fresh geometry for the promoted case -- the "before" world above is already displaced
  // and is not a useful starting point to promote.
  sim_shutdown();
  InitSim();
  World disp_gated;
  disp_gated.Init(kDW, kDH, disp_granite, 2000.0f, 293.15f);
  disp_gated.element[kSrc] = oxygen;
  disp_gated.mass[kSrc] = 3.0f;
  disp_gated.temperature[kSrc] = 310.0f;
  disp_gated.element[kDst] = co2;
  disp_gated.mass[kDst] = 1.0f;
  disp_gated.temperature[kDst] = 290.0f;
  disp_gated.element[kBeyond] = co2;
  disp_gated.mass[kBeyond] = 1.0f;
  disp_gated.temperature[kBeyond] = 290.0f;
  const GameDataUpdate* ge = Boot(t, disp_gated);
  std::vector<uint8_t> disp_gated_visible;

  ext::PromoteRoomMessage disp_promote{kSrc};
  sim_handle_message(ext::kPromoteRoom, static_cast<int>(sizeof(disp_promote)),
                     reinterpret_cast<const uint8_t*>(&disp_promote));
  ge = Tick(disp_gated, &disp_gated_visible);
  Check(sim_debug_room_owned(kSrc) == 1, "the source cell's room is promoted in the gated world too");

  const uint16_t at_promote_elemDst = ge->elementIdx[kDst];
  const float at_promote_massDst = ge->mass[kDst];
  const float at_promote_massBeyond = ge->mass[kBeyond];
  ge = Tick(disp_gated, &disp_gated_visible);
  Check(ge && ge->elementIdx[kDst] == at_promote_elemDst && ge->mass[kDst] == at_promote_massDst &&
            ge->mass[kBeyond] == at_promote_massBeyond,
        "once the source is promoted, vanilla displacement no longer evicts cell B or feeds "
        "cell C, even though the same pressure difference remains");
  ge = Tick(disp_gated, &disp_gated_visible);
  Check(ge && ge->elementIdx[kDst] == at_promote_elemDst && ge->mass[kDst] == at_promote_massDst &&
            ge->mass[kBeyond] == at_promote_massBeyond,
        "still frozen two ticks after promotion, not just one");

  // Consumers and emitters: `ElementConsumer`/`ElementEmitter` -- what GasPump, GasFilter,
  // and virtually every other gas-handling building use -- both funnel through
  // exactly two SimHashes, `MassConsumption` and `MassEmission`. This proves the emission
  // half: `ApplyMassEmission`, for a promoted cell receiving a gas, redirects into the
  // mixture layer (`gas::InjectSpecies`) instead of vanilla's single-element `PhaseEntry`.
  printf("\n=== vftest: promoted cell redirects vanilla gas emission into the mixture ===\n");
  sim_shutdown();
  InitSim();
  uint16_t emit_granite = 0;
  if (!Resolve(t, kGranite, "Granite", &emit_granite)) return 1;
  constexpr int kEW = 5, kEmitCell = 1 * kEW + 1;  // (1,1), same off-border convention as above
  // Granite everywhere except the one cell under test, punched out to vacuum: unlike the
  // pressure/conduction gates above, this section reads its result back through
  // `GameDataUpdate` rather than a WaitIdle'd debug export, so ordinary vanilla pressure flow
  // into an open neighbour would otherwise start dispersing the emitted gas before the read
  // -- confounding "did it arrive" with "how much has already flowed away," the same reason
  // the displacement test above insists on a Granite background. Sealing off every neighbour
  // removes that variable instead of tolerating it.
  World emit_before;
  emit_before.Init(kEW, 3, emit_granite, 2000.0f, 300.0f);
  emit_before.element[kEmitCell] = vacuum;
  emit_before.mass[kEmitCell] = 0.0f;
  const GameDataUpdate* geb = Boot(t, emit_before);
  std::vector<uint8_t> emit_before_visible;
  geb = Tick(emit_before, &emit_before_visible);
  geb = Tick(emit_before, &emit_before_visible);

  MassEmissionMessage sane{};
  sane.cellIdx = kEmitCell;
  sane.callbackIdx = -1;
  sane.mass = 2.0f;
  sane.temperature = 350.0f;
  sane.diseaseCount = 0;
  sane.elementIdx = co2;
  sane.diseaseIdx = 0xFF;
  sim_handle_message(static_cast<int32_t>(SimMessageHash::MassEmission), sizeof(sane),
                     reinterpret_cast<const uint8_t*>(&sane));
  // Two ticks, not one -- the sim's publish lag:
  // a message queued during tick N is processed during N+1, but the frame a given
  // `PrepareGameData` *publishes* is the one *before* the one it just queued. `sim_debug_room_owned`/`sim_debug_gas_mass` elsewhere in
  // this file dodge that lag with their own `WaitIdle()`; reading through the published
  // `GameDataUpdate`, as this check does, does not get to.
  geb = Tick(emit_before, &emit_before_visible);
  geb = Tick(emit_before, &emit_before_visible);
  Check(geb && geb->elementIdx[kEmitCell] == co2 && geb->mass[kEmitCell] > 1.9f,
        "sanity check: before promotion, a MassEmission message lands in vanilla PhaseEntry "
        "normally -- the redirect below has something real to bypass");

  sim_shutdown();
  InitSim();
  World emit_gated;
  emit_gated.Init(kEW, 3, emit_granite, 2000.0f, 300.0f);
  emit_gated.element[kEmitCell] = vacuum;
  emit_gated.mass[kEmitCell] = 0.0f;
  const GameDataUpdate* geg = Boot(t, emit_gated);
  std::vector<uint8_t> emit_gated_visible;

  ext::PromoteRoomMessage emit_promote{kEmitCell};
  sim_handle_message(ext::kPromoteRoom, static_cast<int>(sizeof(emit_promote)),
                     reinterpret_cast<const uint8_t*>(&emit_promote));
  geg = Tick(emit_gated, &emit_gated_visible);
  Check(sim_debug_room_owned(kEmitCell) == 1, "the emission target's room is promoted");

  const uint16_t before_vanilla_elem = geg->elementIdx[kEmitCell];
  const float before_vanilla_mass = geg->mass[kEmitCell];

  MassEmissionMessage gated{};
  gated.cellIdx = kEmitCell;
  gated.callbackIdx = -1;
  gated.mass = 2.0f;
  gated.temperature = 350.0f;
  gated.diseaseCount = 0;
  gated.elementIdx = co2;
  gated.diseaseIdx = 0xFF;
  sim_handle_message(static_cast<int32_t>(SimMessageHash::MassEmission), sizeof(gated),
                     reinterpret_cast<const uint8_t*>(&gated));
  // One tick here, not two: unlike the "before" sanity check above, this assertion is
  // "nothing changed," which the publish lag can't falsify either way -- a stale published
  // frame and a fresh one both show no change when nothing moved. `sim_debug_gas_mass` right
  // below is the one making a positive claim, and it sidesteps the lag on its own (WaitIdle).
  geg = Tick(emit_gated, &emit_gated_visible);
  Check(geg && geg->elementIdx[kEmitCell] == before_vanilla_elem &&
            geg->mass[kEmitCell] == before_vanilla_mass,
        "once promoted, the same MassEmission message leaves vanilla PhaseEntry untouched -- "
        "no vanilla element or mass change at all");
  Check(sim_debug_gas_mass(kEmitCell, co2) >= 1.9f,
        "the emitted mass landed in the mixture layer instead, at the real cell index");
  // The gas brings its own heat. The room is the emission cell alone (granite all round at
  // 300 K), empty before the message, so the cell's temperature after it is the emission's
  // 350 K less whatever two ticks of conduction to the granite took. A path that dropped
  // `temperature` would leave the gas at the empty cell's 300 K.
  geg = Tick(emit_gated, &emit_gated_visible);
  printf("  promoted emission of 2 kg CO2 at 350 K: cell reads %.3f K\n",
         geg ? geg->temperature[kEmitCell] : 0.0f);
  Check(geg && geg->temperature[kEmitCell] > 340.0f && geg->temperature[kEmitCell] <= 350.0f,
        "and it arrived at its own temperature, blended by heat capacity -- an emitter feeding "
        "a promoted room no longer creates or destroys the difference");

  // kInjectGasSpecies blends by HEAT CAPACITY. Same sealed, promoted cell: 2 kg
  // of CO2 at 300 K, then 1 kg of oxygen at 400 K. The cell's m*c*T after the second injection
  // is the two gases' own, so it reads (2 c_CO2 300 + c_O2 400) / (2 c_CO2 + c_O2). A mass
  // weighting reads 333.33 K, and with CO2's specific heat under oxygen's that is energy the
  // injection destroyed.
  {
    sim_shutdown();
    InitSim();
    World inj;
    inj.Init(kEW, 3, emit_granite, 2000.0f, 300.0f);
    inj.element[kEmitCell] = vacuum;
    inj.mass[kEmitCell] = 0.0f;
    const GameDataUpdate* gi2 = Boot(t, inj);
    std::vector<uint8_t> inj_visible;
    ext::PromoteRoomMessage inj_promote{kEmitCell};
    sim_handle_message(ext::kPromoteRoom, static_cast<int>(sizeof(inj_promote)),
                       reinterpret_cast<const uint8_t*>(&inj_promote));
    gi2 = Tick(inj, &inj_visible);
    InjectGas(kEmitCell, co2, 2.0f, 300.0f);
    gi2 = Tick(inj, &inj_visible);
    InjectGas(kEmitCell, oxygen, 1.0f, 400.0f);
    gi2 = Tick(inj, &inj_visible);
    gi2 = Tick(inj, &inj_visible);
    const double c_co2 = t.At(co2)->specificHeatCapacity;
    const double c_o2 = t.At(oxygen)->specificHeatCapacity;
    const double want = (2.0 * c_co2 * 300.0 + c_o2 * 400.0) / (2.0 * c_co2 + c_o2);
    const float got = gi2 ? gi2->temperature[kEmitCell] : 0.0f;
    printf("  2 kg CO2 at 300 K, then 1 kg O2 at 400 K: cell reads %.4f K, heat-capacity "
           "blend %.4f K, mass blend 333.3333 K\n", got, want);
    Check(std::fabs(got - want) < 0.01,
          "an injection blends its temperature in by heat capacity: the cell holds exactly the "
          "two gases' own m*c*T");
  }

  // ModifyCell is a THIRD gas-writing path: CO2Manager.SpawnBreath (a dupe's
  // exhaled CO2 puff) calls SimMessages.ModifyMass, which just calls ModifyCell directly,
  // not through ElementEmitter/MassEmission at all. Same Granite-island geometry as the
  // emission test above, same reasoning.
  printf("\n=== vftest: promoted cell redirects a ModifyCell gas-add into the mixture "
         "(ModifyCell path) ===\n");
  sim_shutdown();
  InitSim();
  uint16_t mc_granite = 0;
  if (!Resolve(t, kGranite, "Granite", &mc_granite)) return 1;
  constexpr int kMCW = 5, kMCCell = 1 * kMCW + 1;
  World mc_before;
  mc_before.Init(kMCW, 3, mc_granite, 2000.0f, 300.0f);
  mc_before.element[kMCCell] = vacuum;
  mc_before.mass[kMCCell] = 0.0f;
  const GameDataUpdate* gmb = Boot(t, mc_before);
  std::vector<uint8_t> mc_before_visible;
  gmb = Tick(mc_before, &mc_before_visible);
  gmb = Tick(mc_before, &mc_before_visible);

  ModifyCellMessage mc_sane{};
  mc_sane.cellIdx = kMCCell;
  mc_sane.callbackIdx = -1;
  mc_sane.temperature = 310.0f;
  mc_sane.mass = 0.03f;  // a plausible single CO2-breath-puff mass
  mc_sane.diseaseCount = 0;
  mc_sane.elementIdx = co2;
  mc_sane.replaceType = 0;  // kReplaceNone (sim/cellmod.h) -- not visible to this ABI-only tool
  mc_sane.diseaseIdx = 0xFF;
  mc_sane.addSubType = 0;
  sim_handle_message(static_cast<int32_t>(SimMessageHash::ModifyCell), sizeof(mc_sane),
                     reinterpret_cast<const uint8_t*>(&mc_sane));
  gmb = Tick(mc_before, &mc_before_visible);
  gmb = Tick(mc_before, &mc_before_visible);
  Check(gmb && gmb->elementIdx[kMCCell] == co2 && gmb->mass[kMCCell] > 0.025f,
        "sanity check: before promotion, a ModifyCell gas-add lands in vanilla PhaseEntry "
        "normally -- the redirect below has something real to bypass");

  sim_shutdown();
  InitSim();
  World mc_gated;
  mc_gated.Init(kMCW, 3, mc_granite, 2000.0f, 300.0f);
  mc_gated.element[kMCCell] = vacuum;
  mc_gated.mass[kMCCell] = 0.0f;
  const GameDataUpdate* gmg = Boot(t, mc_gated);
  std::vector<uint8_t> mc_gated_visible;

  ext::PromoteRoomMessage mc_promote{kMCCell};
  sim_handle_message(ext::kPromoteRoom, static_cast<int>(sizeof(mc_promote)),
                     reinterpret_cast<const uint8_t*>(&mc_promote));
  gmg = Tick(mc_gated, &mc_gated_visible);
  Check(sim_debug_room_owned(kMCCell) == 1, "the ModifyCell target's room is promoted");

  const uint16_t mc_before_vanilla_elem = gmg->elementIdx[kMCCell];
  const float mc_before_vanilla_mass = gmg->mass[kMCCell];

  ModifyCellMessage mc_gated_msg{};
  mc_gated_msg.cellIdx = kMCCell;
  mc_gated_msg.callbackIdx = -1;
  mc_gated_msg.temperature = 310.0f;
  mc_gated_msg.mass = 0.03f;
  mc_gated_msg.diseaseCount = 0;
  mc_gated_msg.elementIdx = co2;
  mc_gated_msg.replaceType = 0;
  mc_gated_msg.diseaseIdx = 0xFF;
  mc_gated_msg.addSubType = 0;
  sim_handle_message(static_cast<int32_t>(SimMessageHash::ModifyCell), sizeof(mc_gated_msg),
                     reinterpret_cast<const uint8_t*>(&mc_gated_msg));
  gmg = Tick(mc_gated, &mc_gated_visible);
  Check(gmg && gmg->elementIdx[kMCCell] == mc_before_vanilla_elem &&
            gmg->mass[kMCCell] == mc_before_vanilla_mass,
        "once promoted, the same ModifyCell gas-add leaves vanilla PhaseEntry untouched -- "
        "no vanilla element or mass change at all");
  Check(sim_debug_gas_mass(kMCCell, co2) > 0.025f,
        "the CO2 landed in the mixture layer instead, at the real cell index -- exactly the "
        "user's manual-test scenario (dupe breath in a promoted room)");

  // ext::kSetBlockedGasAddPolicy. A ModifyCell gas add onto a DIFFERENT gas that
  // cannot be displaced, with no vacuum or same-gas neighbour to the left, right or above,
  // reaches the game's add into a blocked cell, which deletes the smaller mass. Found live on the
  // bubbles rig (200 g of CO2 onto a 20 g methane pocket lost 40 g).
  //
  // Geometry: a two-cell room walled in Granite, hydrogen at A and oxygen at B beside it.
  // CO2 into A: DisplaceGas cannot move the hydrogen (B is not hydrogen and not vacuum), and
  // A's three neighbours are granite, oxygen, granite. Three arms, each on a fresh sim:
  // vanilla (proves the geometry really reaches the deleting tail), the policy on the same
  // add, and the policy on an add that is NOT blocked (must stay vanilla).
  printf("\n=== vftest: a blocked ModifyCell gas add mixes instead of deleting "
         "(kSetBlockedGasAddPolicy) ===\n");
  {
    uint16_t bg_hydrogen = 0;
    if (!Resolve(t, simhost::kHydrogen, "Hydrogen", &bg_hydrogen)) return 1;
    constexpr int kBW = 6, kBA = 1 * kBW + 1, kBB = 1 * kBW + 2;
    constexpr float kH2 = 0.05f, kO2 = 1.0f, kCO2 = 0.2f;
    auto make = [&](World* w, uint16_t a_element, float a_mass) {
      w->Init(kBW, 3, mc_granite, 2000.0f, 300.0f);
      w->element[kBA] = a_element;
      w->mass[kBA] = a_mass;
      w->element[kBB] = oxygen;
      w->mass[kBB] = kO2;
    };
    auto add_co2 = [&](int32_t cell) {
      ModifyCellMessage mc{};
      mc.cellIdx = cell;
      mc.callbackIdx = -1;
      mc.temperature = 350.0f;
      mc.mass = kCO2;
      mc.diseaseCount = 0;
      mc.elementIdx = co2;
      mc.replaceType = 0;  // kReplaceNone
      mc.diseaseIdx = 0xFF;
      mc.addSubType = 0;
      sim_handle_message(static_cast<int32_t>(SimMessageHash::ModifyCell), sizeof(mc),
                         reinterpret_cast<const uint8_t*>(&mc));
    };
    auto set_policy = [&](int32_t policy) {
      ext::SetBlockedGasAddPolicyMessage pm{policy};
      sim_handle_message(ext::kSetBlockedGasAddPolicy, static_cast<int>(sizeof(pm)),
                         reinterpret_cast<const uint8_t*>(&pm));
    };

    // Arm 1: vanilla. The hydrogen is emptied and only the remainder of the CO2 goes in.
    sim_shutdown();
    InitSim();
    World bv;
    make(&bv, bg_hydrogen, kH2);
    const GameDataUpdate* gb = Boot(t, bv);
    std::vector<uint8_t> bv_visible;
    gb = Tick(bv, &bv_visible);
    add_co2(kBA);
    gb = Tick(bv, &bv_visible);
    gb = Tick(bv, &bv_visible);
    Check(gb && gb->elementIdx[kBA] == co2 && std::fabs(gb->mass[kBA] - (kCO2 - kH2)) < 1e-4f,
          "vanilla, the geometry reaches AddIntoBlockedCell: A is CO2 at 0.15 kg -- the "
          "0.05 kg of hydrogen and 0.05 kg of the CO2 are gone");

    // Arm 2: the policy. Queued, and drained before ModifyCell in the same frame, so it is
    // sent in the same tick as the add on purpose.
    sim_shutdown();
    InitSim();
    World bm;
    make(&bm, bg_hydrogen, kH2);
    gb = Boot(t, bm);
    std::vector<uint8_t> bm_visible;
    gb = Tick(bm, &bm_visible);
    set_policy(ext::kBlockedGasAddPromoteAndMix);
    add_co2(kBA);
    gb = Tick(bm, &bm_visible);
    gb = Tick(bm, &bm_visible);
    Check(sim_debug_room_owned(kBA) == 1 && sim_debug_room_owned(kBB) == 1,
          "the blocked add promoted the two-cell room it landed in");
    Check(gb && gb->elementIdx[kBA] == vacuum && gb->mass[kBA] == 0.0f &&
              gb->elementIdx[kBB] == vacuum && gb->mass[kBB] == 0.0f,
          "both cells' vanilla gas moved into the mixture: vanilla's grid reads Vacuum/0 at A "
          "and at B, the promoted-cell shape");
    const float h2_sum = sim_debug_gas_mass(kBA, bg_hydrogen) + sim_debug_gas_mass(kBB, bg_hydrogen);
    const float o2_sum = sim_debug_gas_mass(kBA, oxygen) + sim_debug_gas_mass(kBB, oxygen);
    const float co2_sum = sim_debug_gas_mass(kBA, co2) + sim_debug_gas_mass(kBB, co2);
    printf("    mixture over the room: H2 %.6f  O2 %.6f  CO2 %.6f kg\n", h2_sum, o2_sum, co2_sum);
    Check(std::fabs(h2_sum - kH2) < 1e-6f && std::fabs(o2_sum - kO2) < 1e-6f &&
              std::fabs(co2_sum - kCO2) < 1e-6f,
          "nothing deleted: the room's mixture holds all 0.05 kg of hydrogen, all 1 kg of "
          "oxygen and all 0.2 kg of CO2");
    // Energy, not mass, decides the blend: 0.05 kg of hydrogen at 300 K against 0.2 kg of CO2
    // at 350 K. A mass-weighted mix would say 340 K; weighted by m*c it is 329.25 K. The
    // mixing pass runs in the same ticks and carries species between A and B, so A's
    // temperature is no longer the add's alone to decide; the room's m*c*T is. It checks the
    // add and the mixing pass together: a mass-weighted add would leave the room ~3.1 kJ high,
    // and a mixing pass that moved mass without its heat would leave it 3.45 kJ high.
    //
    // Read only once the room has stopped moving mass. `sim_debug_gas_mass` reads the live
    // mixture, but the temperatures come from the GameDataUpdate `PrepareGameData` hands back,
    // which is the frame it just collected -- one frame behind. While species are still
    // crossing, the two disagree by that frame's transfer: measured, 397.83 kJ two
    // ticks after the add, falling to 396.75 twelve ticks later as the transfers shrank.
    for (int k = 0; k < 120; ++k) gb = Tick(bm, &bm_visible);
    {
      const double c_h2 = t.SpecificHeat(bg_hydrogen), c_o2 = t.SpecificHeat(oxygen),
                   c_co2 = t.SpecificHeat(co2);
      const double want_a =
          (kH2 * c_h2 * 300.0 + kCO2 * c_co2 * 350.0) / (kH2 * c_h2 + kCO2 * c_co2);
      auto hc = [&](int32_t cell) {
        return sim_debug_gas_mass(cell, bg_hydrogen) * c_h2 +
               sim_debug_gas_mass(cell, oxygen) * c_o2 + sim_debug_gas_mass(cell, co2) * c_co2;
      };
      const double room_energy = hc(kBA) * gb->temperature[kBA] + hc(kBB) * gb->temperature[kBB];
      const double add_energy = (kH2 * c_h2 + kCO2 * c_co2) * want_a + kO2 * c_o2 * 300.0;
      printf("    per cell: A H2 %.6f O2 %.6f CO2 %.6f | B H2 %.6f O2 %.6f CO2 %.6f kg\n",
             sim_debug_gas_mass(kBA, bg_hydrogen), sim_debug_gas_mass(kBA, oxygen),
             sim_debug_gas_mass(kBA, co2), sim_debug_gas_mass(kBB, bg_hydrogen),
             sim_debug_gas_mass(kBB, oxygen), sim_debug_gas_mass(kBB, co2));
      printf("    A %.4f K, B %.4f K; the add alone would leave A at %.4f K\n",
             gb->temperature[kBA], gb->temperature[kBB], want_a);
      printf("    room m*c*T after mixing %.6f kJ against %.6f kJ right after the add\n",
             room_energy, add_energy);
      Check(sim_debug_gas_mass(kBA, oxygen) > 0.0f && sim_debug_gas_mass(kBB, co2) > 0.0f,
            "the mixing pass ran: oxygen reached A and CO2 reached B");
      Check(gb && std::fabs(room_energy - add_energy) < 0.01,
            "energy conserved through the add AND 122 ticks of mixing: the room's m*c*T equals "
            "the m*c-weighted add's, within 0.01 kJ of ~397 kJ");
    }

    // Arm 3: the policy on, and an add that is not blocked (A already holds CO2, so it is a
    // plain merge). Nothing may be promoted and the gas lands in vanilla's grid as always.
    sim_shutdown();
    InitSim();
    World bn;
    make(&bn, co2, 0.1f);
    gb = Boot(t, bn);
    std::vector<uint8_t> bn_visible;
    gb = Tick(bn, &bn_visible);
    set_policy(ext::kBlockedGasAddPromoteAndMix);
    add_co2(kBA);
    gb = Tick(bn, &bn_visible);
    gb = Tick(bn, &bn_visible);
    Check(sim_debug_room_owned(kBA) != 1,
          "an add that is not blocked promotes nothing, policy or no policy");
    Check(gb && gb->elementIdx[kBA] == co2 && std::fabs(gb->mass[kBA] - 0.3f) < 1e-4f &&
              gb->elementIdx[kBB] == oxygen,
          "and it lands in vanilla's grid exactly as Klei's merge does: A is CO2 at 0.3 kg");
  }

  // `ApplyMassConsumption` services a direct
  // `SimMessages.ConsumeMass` send -- what `GasBreatherFromWorldProvider.ConsumeGas`
  // (breathing) actually sends -- and shares `RemoveMassFromCell`'s promotion gate with
  // `StepElementConsumers` below. Same Granite-island geometry as the emission test: an
  // isolated cell so ordinary vanilla flow can't confound "how much did this message remove."
  printf("\n=== vftest: promoted cell redirects vanilla gas consumption into the mixture "
         "(VANILLA-REPLACEMENT items 4/5, ApplyMassConsumption) ===\n");
  sim_shutdown();
  InitSim();
  constexpr int kCW2 = 5, kConsumeCell = 1 * kCW2 + 1;
  World consume_before;
  consume_before.Init(kCW2, 3, emit_granite, 2000.0f, 300.0f);
  consume_before.element[kConsumeCell] = oxygen;
  consume_before.mass[kConsumeCell] = 5.0f;
  const GameDataUpdate* gcb = Boot(t, consume_before);
  std::vector<uint8_t> consume_before_visible;
  gcb = Tick(consume_before, &consume_before_visible);
  gcb = Tick(consume_before, &consume_before_visible);

  MassConsumptionMessage csane{};
  csane.cellIdx = kConsumeCell;
  csane.callbackIdx = -1;
  csane.mass = 2.0f;
  csane.elementIdx = oxygen;
  csane.radius = 1;
  csane.height = 0;
  sim_handle_message(static_cast<int32_t>(SimMessageHash::MassConsumption), sizeof(csane),
                     reinterpret_cast<const uint8_t*>(&csane));
  // Two ticks, same publish lag as the emission sanity check above.
  gcb = Tick(consume_before, &consume_before_visible);
  gcb = Tick(consume_before, &consume_before_visible);
  Check(gcb && gcb->mass[kConsumeCell] > 2.9f && gcb->mass[kConsumeCell] < 3.1f,
        "sanity check: before promotion, a MassConsumption message drains vanilla PhaseEntry "
        "normally (5 kg - 2 kg = 3 kg) -- the redirect below has something real to bypass");

  sim_shutdown();
  InitSim();
  World consume_gated;
  consume_gated.Init(kCW2, 3, emit_granite, 2000.0f, 300.0f);
  consume_gated.element[kConsumeCell] = oxygen;
  consume_gated.mass[kConsumeCell] = 5.0f;
  const GameDataUpdate* gcg = Boot(t, consume_gated);
  std::vector<uint8_t> consume_gated_visible;

  ext::PromoteRoomMessage consume_promote{kConsumeCell};
  sim_handle_message(ext::kPromoteRoom, static_cast<int>(sizeof(consume_promote)),
                     reinterpret_cast<const uint8_t*>(&consume_promote));
  gcg = Tick(consume_gated, &consume_gated_visible);
  Check(sim_debug_room_owned(kConsumeCell) == 1, "the consumption target's room is promoted");

  // Seed the mixture with real gas to consume -- vanilla mass at this cell stays 5 kg
  // throughout (InjectSpecies never touches PhaseEntry).
  InjectGas(kConsumeCell, oxygen, 4.0f, 300.0f);
  gcg = Tick(consume_gated, &consume_gated_visible);
  const float before_vanilla_mass_c = gcg->mass[kConsumeCell];

  MassConsumptionMessage cgated{};
  cgated.cellIdx = kConsumeCell;
  cgated.callbackIdx = -1;
  cgated.mass = 2.0f;
  cgated.elementIdx = oxygen;
  cgated.radius = 1;
  cgated.height = 0;
  sim_handle_message(static_cast<int32_t>(SimMessageHash::MassConsumption), sizeof(cgated),
                     reinterpret_cast<const uint8_t*>(&cgated));
  gcg = Tick(consume_gated, &consume_gated_visible);
  Check(gcg && gcg->mass[kConsumeCell] == before_vanilla_mass_c,
        "once promoted, the same MassConsumption message leaves vanilla PhaseEntry untouched");
  Check(sim_debug_gas_mass(kConsumeCell, oxygen) > 1.9f &&
            sim_debug_gas_mass(kConsumeCell, oxygen) < 2.1f,
        "the consumed mass came out of the mixture layer instead (4 kg injected - 2 kg "
        "consumed = 2 kg)");

  // The *other* consumption path, `StepElementConsumers` -- the per-substep native port of
  // the `ElementConsumer` component GasPump/GasFilter use (see `GasPumpConfig`). `configuration = kConsumeAllGas` is
  // GasPump's own setting, which is what makes `AnyInputCellHasState`'s fix load-bearing here
  // rather than `RemoveMassFromCell`'s alone: this consumer never names an element up front,
  // it has to be told one exists to look for by the reachability scan.
  printf("\n=== vftest: promoted cell redirects a GasPump-shaped ElementConsumer into the "
         "mixture (StepElementConsumers) ===\n");
  sim_shutdown();
  InitSim();
  constexpr int kCW3 = 5, kPumpCell = 1 * kCW3 + 1;
  World pump_world;
  pump_world.Init(kCW3, 3, emit_granite, 2000.0f, 300.0f);
  pump_world.element[kPumpCell] = vacuum;
  pump_world.mass[kPumpCell] = 0.0f;
  const GameDataUpdate* gp = Boot(t, pump_world);
  std::vector<uint8_t> pump_visible;
  gp = Tick(pump_world, &pump_visible);
  gp = Tick(pump_world, &pump_visible);

  ext::PromoteRoomMessage pump_promote{kPumpCell};
  sim_handle_message(ext::kPromoteRoom, static_cast<int>(sizeof(pump_promote)),
                     reinterpret_cast<const uint8_t*>(&pump_promote));
  gp = Tick(pump_world, &pump_visible);
  Check(sim_debug_room_owned(kPumpCell) == 1, "the GasPump's own cell room is promoted");

  InjectGas(kPumpCell, co2, 6.0f, 300.0f);
  gp = Tick(pump_world, &pump_visible);  // land the injection before the consumer looks for it

  AddElementConsumerMessage add{};
  add.cellIdx = kPumpCell;
  add.callbackIdx = 42;  // any non -1 id -- read the real handle back below, not assumed
  add.radius = 1;        // depth 1: reaches exactly its own cell, same off-by-one as Flood's
                         // depth test everywhere else in this file (depth 0 would flood nothing,
                         // not even the starting cell).
  // 2 == kConsumeAllGas (sim/emitters.h) -- not visible here, this tool links the DLL's
  // exported ABI only, not the sim headers (see this file's own top comment); GasPumpConfig.cs
  // is the reason this specific value, not a made-up one.
  add.configuration = 2;
  add.elementIdx = 0;                  // unused for AllGas
  sim_handle_message(static_cast<int32_t>(SimMessageHash::AddElementConsumer), sizeof(add),
                     reinterpret_cast<const uint8_t*>(&add));
  // Two ticks, same publish lag as every other read through
  // `GameDataUpdate` in this file -- `componentStateChangedMessages` is published the same
  // way `mass[]`/`elementIdx[]` are, one frame behind the one that actually processed it.
  gp = Tick(pump_world, &pump_visible);
  gp = Tick(pump_world, &pump_visible);
  int32_t consumer_handle = -1;
  if (gp != nullptr) {
    for (int32_t i = 0; i < gp->numComponentStateChangedMessages; ++i) {
      if (gp->componentStateChangedMessages[i].callbackIdx == 42) {
        consumer_handle = gp->componentStateChangedMessages[i].simHandle;
      }
    }
  }
  Check(consumer_handle != -1, "AddElementConsumer registered and returned a real handle");

  SetElementConsumerDataMessage setdata{};
  setdata.handle = consumer_handle;
  setdata.cell = kPumpCell;
  setdata.consumptionRate = 1.0f;  // 1 kg/s -- Register seeds the rate at 0, inert until this
  sim_handle_message(static_cast<int32_t>(SimMessageHash::SetElementConsumerData),
                     sizeof(setdata), reinterpret_cast<const uint8_t*>(&setdata));

  const float mixture_before_pump = sim_debug_gas_mass(kPumpCell, co2);
  gp = Tick(pump_world, &pump_visible);
  gp = Tick(pump_world, &pump_visible);
  gp = Tick(pump_world, &pump_visible);
  const float mixture_after_pump = sim_debug_gas_mass(kPumpCell, co2);
  Check(mixture_after_pump < mixture_before_pump,
        "a GasPump-shaped ElementConsumer (kConsumeAllGas) drew real mass out of a promoted "
        "cell's mixture -- AnyInputCellHasState recognized it as a gas source despite "
        "vanilla's own (frozen, Vacuum) element sitting there");
  Check(gp && gp->elementIdx[kPumpCell] == vacuum && gp->mass[kPumpCell] == 0.0f,
        "vanilla PhaseEntry at the cell was never touched by any of this -- still Vacuum, "
        "still zero mass, exactly as promotion left it");

  // Real physics change, not a managed-side hack: ext::kSetInvertedGravityElement flips the
  // vertical sign of one element's own turn in StepFlow's liquid branch (physics.h), rather
  // than swapping cells from managed C# every frame on top of an untouched vanilla kernel. This proves the actual kernel branches the other way, not that a managed loop
  // fakes the appearance of it.
  //
  // A width=1 column removes every horizontal neighbour, so the swap-vs-spawn permeability
  // probe in physics.h (`permeable(x+1,y)` etc.) always reads "walled in on both sides" and
  // takes the deterministic whole-cell SwapCells branch rather than the SpawnFallingLiquid
  // branch -- headless test harnesses never accept a spawn (see physics.h's own comment on the
  // `pair` scenario), so a wider world would need many ticks of pressure-driven pour instead
  // of one clean swap per tick. Cell 5 is Granite, standing in for a solid ceiling -- height 7,
  // not 6, so row 5 is still a real interior row (see the "clearing the toggle" world below for
  // why: `Tick()`'s NewGameFrame excludes the topmost row from ever being a valid turn).
  printf("\n=== vftest: real reversed-gravity physics (ext::kSetInvertedGravityElement) ===\n");
  sim_shutdown();
  InitSim();
  uint16_t water = 0, granite = 0;
  if (!Resolve(t, kWater, "Water", &water)) return 1;
  if (!Resolve(t, kGranite, "Granite", &granite)) return 1;

  World grav;
  grav.Init(1, 7, oxygen, 1.0f, 300.0f);
  grav.element[0] = water;
  grav.mass[0] = 500.0f;
  grav.element[5] = granite;
  grav.mass[5] = 2000.0f;
  const GameDataUpdate* gg = Boot(t, grav);

  ext::SetInvertedGravityElementMessage inv{static_cast<int32_t>(water)};
  sim_handle_message(ext::kSetInvertedGravityElement, static_cast<int>(sizeof(inv)),
                     reinterpret_cast<const uint8_t*>(&inv));

  std::vector<uint8_t> grav_visible;
  for (int i = 0; i < 5 && gg; ++i) gg = Tick(grav, &grav_visible);
  Check(gg && gg->elementIdx[4] == water && gg->mass[4] > 400.0f,
        "water rose from cell 0 to cell 4, four cells up, under an inverted-gravity toggle");
  Check(gg && gg->elementIdx[0] != water,
        "cell 0 no longer holds water -- it rose out, it didn't duplicate");
  Check(gg && gg->elementIdx[5] == granite,
        "the granite ceiling at cell 5 is untouched -- water never crosses a solid");

  // One more tick: water sitting directly under the ceiling must stay there, not vanish into
  // or fall back through the cells it just rose out of -- the "pools on the roof" property,
  // checked at the kernel level rather than assumed from the swap logic alone.
  gg = Tick(grav, &grav_visible);
  Check(gg && gg->elementIdx[4] == water && gg->mass[4] > 400.0f,
        "water pools at cell 4, directly under the solid ceiling, instead of falling back");

  // Clearing the toggle (-1) must restore ordinary vanilla gravity immediately, on a fresh
  // world -- spot-checked by confirming a new water cell now falls instead of rising.
  //
  // Height 7, not 6: found live by this test's first run. `Tick()`'s NewGameFrame sets
  // `maxY = height - 1`, and StepFlow's region derived from that treats the topmost row as
  // outside the interior -- never a valid turn source, only ever a destination. The earlier
  // rise test never surfaced this because its water started at row 0 and only ever needed
  // rows 1-4 as valid interior turns; row 5 (its ceiling) being unreachable was consistent
  // with "blocked by solid granite" either way. Seeding water at the true top row here (5, in
  // a 6-row world) left it permanently un-sourced -- frozen from tick 0, not falling one cell
  // in five ticks. One extra row of headroom (row 6, never touched) keeps row 5 a real
  // interior source, symmetric to the rise test's use of rows 0-4.
  ext::SetInvertedGravityElementMessage clear{-1};
  sim_handle_message(ext::kSetInvertedGravityElement, static_cast<int>(sizeof(clear)),
                     reinterpret_cast<const uint8_t*>(&clear));
  sim_shutdown();
  InitSim();
  World fall;
  fall.Init(1, 7, oxygen, 1.0f, 300.0f);
  fall.element[5] = water;
  fall.mass[5] = 500.0f;
  fall.element[0] = granite;
  fall.mass[0] = 2000.0f;
  const GameDataUpdate* gf = Boot(t, fall);
  std::vector<uint8_t> fall_visible;
  for (int i = 0; i < 5 && gf; ++i) gf = Tick(fall, &fall_visible);
  Check(gf && gf->elementIdx[1] == water,
        "with the toggle cleared, a fresh world's water falls normally again (down to cell 1, "
        "just above the floor at cell 0)");

  // The read-side gap: a mixture-occupied cell's shared
  // PhaseEntry.temperature must survive vanilla mass draining to zero. Two prior fixes
  // (ZeroMasslessCells, ClearCell) each looked sufficient and each still left this failing
  // -- StepPostProcess's gas branch calls EvaporateWisp BEFORE ClearCell ever gets a turn,
  // and it has the same bug. Third guard site, same GasOccupiedMask condition.
  printf("\n=== vftest: mixture-occupied cell keeps its temperature when vanilla mass drains "
         "to zero ===\n");
  sim_shutdown();
  InitSim();
  // Granite-isolated, same pattern as the ModifyCell addendum test above -- a lone gas cell
  // with granite on every side has no gas-state neighbour, so GasShuffle/DoDensityDisplacement
  // can never refill the drained cell from next door. Without this the first real run of this
  // test failed its own "mass is exactly zero" sanity check: a same-element, same-temperature
  // neighbour (the original all-oxygen world) is a real, ordinary vanilla density-displacement
  // candidate, and it occasionally won the coin flip and swapped a full PhaseEntry back in --
  // correct vanilla behaviour, just not what this test needed to isolate.
  uint16_t temp_granite = 0;
  if (!Resolve(t, kGranite, "Granite", &temp_granite)) return 1;
  constexpr int kTW = 5, kTCell = 1 * kTW + 1;
  World temp_world;
  // Granite at the SAME temperature as the gas cell -- no gradient, so conduction (ungated
  // here; this test isn't about conduction) can't drift the reading before the drain even runs.
  temp_world.Init(kTW, 3, temp_granite, 2000.0f, 350.0f);
  temp_world.element[kTCell] = oxygen;
  temp_world.mass[kTCell] = 2.0f;
  const GameDataUpdate* gt = Boot(t, temp_world);
  std::vector<uint8_t> temp_visible;
  gt = Tick(temp_world, &temp_visible);  // "first frame does nothing" head start
  gt = Tick(temp_world, &temp_visible);
  Check(gt && gt->temperature[kTCell] > 349.0f,
        "sanity check: vanilla temperature reads back before anything is touched");

  // Puts real mass in the gas-mixture layer at this cell -- GasOccupiedMask's bit for
  // oxygen's species slot goes up as a side effect (gas_mixture.h's AddSpecies), which is
  // the condition all three guards check. Not a room promotion -- this bug is about mixture
  // occupancy, not room ownership, and the guard code reads GasOccupiedMask, not RoomGraph.
  InjectGas(kTCell, oxygen, 1.0f, 350.0f);  // matches temp_world's own fill temp -- no-op blend, isolates this test to the drain behavior
  gt = Tick(temp_world, &temp_visible);  // one-tick message latency, same as InjectGas above
  Check(sim_debug_gas_mass(kTCell, oxygen) > 0.0f,
        "the cell now holds real gas-mixture mass (GasOccupiedMask should be set)");

  // Drain the vanilla mass to exactly zero. ApplyRemoveVanillaMass only ever touches
  // PhaseEntry.mass -- element stays Oxygen, not Vacuum -- so the next StepPostProcess sees
  // a "gas" cell (by element) at zero mass: EvaporateWisp's mass < 0.001f branch, then its
  // mass < 1e-9f branch, same path a real duplicant fully exhaling a cell's contents (or a
  // pump draining one) would take.
  const float vanilla_mass_here = gt->mass[kTCell];
  ext::RemoveVanillaMassMessage drain{kTCell, vanilla_mass_here};
  sim_handle_message(ext::kRemoveVanillaMass, static_cast<int>(sizeof(drain)),
                     reinterpret_cast<const uint8_t*>(&drain));
  gt = Tick(temp_world, &temp_visible);  // message queued this tick
  gt = Tick(temp_world, &temp_visible);  // lands on the NEXT tick (one-tick latency, same as
                                          // every other ext:: message here)
  Check(gt && gt->mass[kTCell] == 0.0f,
        "vanilla mass at the cell is now exactly zero -- EvaporateWisp/ClearCell both had a "
        "real target to fire on");
  Check(gt && gt->elementIdx[kTCell] == vacuum,
        "EvaporateWisp actually ran and cleared the element to Vacuum -- confirms this is the "
        "same code path being tested, not a no-op");

  // A few more ticks: EvaporateWisp runs every tick a gas-state cell's mass stays under the
  // wisp threshold, so if the guard were missing this would fail on the very next tick and
  // stay failed, not recover.
  for (int i = 0; i < 3 && gt; ++i) gt = Tick(temp_world, &temp_visible);
  Check(gt && gt->temperature[kTCell] > 349.0f,
        "temperature at the drained cell is UNCHANGED (still ~350 K), not zeroed -- the "
        "shared PhaseEntry.temperature field survives because the cell still holds real "
        "gas-mixture mass");
  Check(sim_debug_gas_mass(kTCell, oxygen) > 0.0f,
        "the gas-mixture mass itself is of course untouched by any of this -- it was never "
        "vanilla's to begin with");

  // The room graph must not be a build-once snapshot: it has to be rebuilt when a wall is dug
  // or built after the graph exists. Real choke point: ChangeCellProperties (kGasImpermeable) is the
  // actual vanilla message both digging and building send.
  printf("\n=== vftest: room graph reacts to a wall built mid-session "
         "(digging/building path) ===\n");
  sim_shutdown();
  InitSim();
  // A 1x5 open corridor down the middle row of a 7x3 world (columns 0 and 6 are dead space,
  // same margin the gate/conduction tests above keep). Promoting the whole corridor as one
  // room, then building a wall through its middle cell, should split it into two rooms that
  // BOTH still report owned -- not one owned and one orphaned, which is what RoomGraph.owned
  // silently resetting to all-zero on every rebuild would look like from the outside.
  constexpr int kRW = 7, kRRowY = 1;
  auto rr = [kRW](int x) { return kRRowY * kRW + x; };
  World room_world;
  room_world.Init(kRW, 3, oxygen, 1.0f, 300.0f);
  const GameDataUpdate* gr = Boot(t, room_world);
  std::vector<uint8_t> room_visible;
  gr = Tick(room_world, &room_visible);
  gr = Tick(room_world, &room_visible);
  (void)gr;

  ext::PromoteRoomMessage room_promote{rr(1)};
  sim_handle_message(ext::kPromoteRoom, static_cast<int>(sizeof(room_promote)),
                     reinterpret_cast<const uint8_t*>(&room_promote));
  Tick(room_world, &room_visible);
  Check(sim_debug_room_owned(rr(1)) == 1 && sim_debug_room_owned(rr(5)) == 1,
        "the whole open corridor promotes as one room, both ends included");

  // The real vanilla message digging/building both send. `properties = 1` is
  // world.h's kGasImpermeable -- not visible to this ABI-only tool by name, so spelled out
  // as the literal it actually is, same as ModifyCellMessage's `replaceType = 0` above.
  CellPropertiesMessage wall{};
  wall.cellIdx = rr(3);
  wall.callbackIdx = -1;
  wall.properties = 1;  // kGasImpermeable
  wall.set = 1;
  sim_handle_message(static_cast<int32_t>(SimMessageHash::ChangeCellProperties), sizeof(wall),
                     reinterpret_cast<const uint8_t*>(&wall));
  // One tick: DrainQueue applies the property change and marks RoomsDirty, then StepPhysics
  // (same RunFrame, same tick) sees it dirty and rebuilds before this Tick() call returns --
  // sim_debug_room_owned WaitIdle()s and reads the live RoomGraph directly, not a published
  // GameDataUpdate, so it needs none of the extra publish-lag tick the mass/temperature
  // checks above did.
  Tick(room_world, &room_visible);
  Check(sim_debug_room_owned(rr(3)) == -1,
        "the walled cell itself is no longer part of any room -- IsOpenCell now says closed, "
        "and the graph noticed without anyone sending a fresh promotion");
  Check(sim_debug_room_owned(rr(1)) == 1 && sim_debug_room_owned(rr(5)) == 1,
        "BOTH halves of the split corridor are STILL promoted -- World::RoomPromoted "
        "survived a rebuild that gave them new, different room ids than before the wall went "
        "up");

  // Second choke point: a natural phase transition
  // crossing the solid boundary (TransitionCell, physics.h) can open or close a cell just as
  // much as digging can -- ice melting into water is the same "was closed, now open" case as
  // the corridor test above, just reached a different way. ModifyCellEnergy is the real
  // vanilla message that heats a cell without moving mass (a running machine's waste heat, a
  // duplicant's body heat) and, per ApplyCellEnergy's own comment, calls TransitionCell
  // directly -- used here only because it lands the transition in one message instead of
  // waiting on ordinary per-substep conduction to cross the threshold over many ticks.
  printf("\n=== vftest: room graph reacts to a natural phase transition "
         "(TransitionCell path) ===\n");
  sim_shutdown();
  InitSim();
  uint16_t ice = 0;
  if (!Resolve(t, simhost::kIce, "Ice", &ice)) return 1;
  uint16_t ice_granite = 0;
  if (!Resolve(t, kGranite, "Granite", &ice_granite)) return 1;
  uint16_t ice_oxygen = 0;
  if (!Resolve(t, kOxygenHash, "Oxygen", &ice_oxygen)) return 1;
  // Granite everywhere except two cells: real Ice (solid -- closed, per IsOpenCell) and a
  // real Oxygen cell far away in the same otherwise-solid world, needed only to have
  // somewhere open to activate volume-fractions from (an all-granite world has nowhere to
  // send a first InjectGas/PromoteRoom at all). Same isolation reasoning as the read-side test.
  constexpr int kIW = 9, kIceCell = 1 * kIW + 1, kIO2Cell = 1 * kIW + 7;
  World ice_world;
  ice_world.Init(kIW, 3, ice_granite, 2000.0f, 250.0f);
  ice_world.element[kIceCell] = ice;
  ice_world.mass[kIceCell] = 5.0f;
  // Granite at the same 250 K as the ice -- no gradient, so ordinary conduction can't nudge
  // this towards a transition threshold before the message does it deliberately (same
  // reasoning as the read-side test's granite-isolated cell).
  ice_world.element[kIO2Cell] = ice_oxygen;
  ice_world.mass[kIO2Cell] = 1.0f;
  ice_world.temperature[kIO2Cell] = 300.0f;
  const GameDataUpdate* gi = Boot(t, ice_world);
  std::vector<uint8_t> ice_visible;
  gi = Tick(ice_world, &ice_visible);
  gi = Tick(ice_world, &ice_visible);
  (void)gi;

  InjectGas(kIO2Cell, ice_oxygen, 0.001f, 300.0f);  // only to activate volume-fractions
  Tick(ice_world, &ice_visible);
  Check(sim_debug_room_owned(kIceCell) == -1,
        "sanity check: the solid ice cell starts with no room at all -- IsOpenCell already "
        "says closed before anything melts");
  Check(sim_debug_room_owned(kIO2Cell) == 0,
        "sanity check: the real open cell elsewhere got a room of its own (vanilla-owned, "
        "not promoted)");

  ModifyCellEnergyMessage melt{};
  melt.cellIdx = kIceCell;
  melt.kilojoules = 1.0e9f;    // absurdly large on purpose -- only the ceiling matters
  melt.maxTemperature = 500.0f;  // comfortably past any real ice/water transition threshold
  melt.id = -1;
  sim_handle_message(static_cast<int32_t>(SimMessageHash::ModifyCellEnergy), sizeof(melt),
                     reinterpret_cast<const uint8_t*>(&melt));
  Tick(ice_world, &ice_visible);  // same one-tick reasoning as the wall test above
  Check(sim_debug_room_owned(kIceCell) != -1,
        "the melted cell now has a real room -- TransitionCell's solid-crossing check "
        "marked the graph dirty and StepPhysics rebuilt it, with no promotion message "
        "involved at all");

  // Promotion survives save/load (saveblob.h + world.h: kSaveVersionRoomPromotion=17,
  // World::room_promoted_ round-tripped).
  // BuildRoomGraph already re-derives RoomGraph.owned from World::RoomPromoted on every
  // rebuild (gas_rooms.h) -- so persistence only had to get the byte
  // itself across a save/load, not build any new rebuild path. Uses a full process restart
  // (shutdown + reinit + resend only the element/disease tables, no Boot/Start) rather than
  // calling Load on the still-live instance the gas-mixture round-trip test above
  // does -- proves this doesn't secretly depend on any in-memory state surviving between the
  // save and the load.
  printf("\n=== vftest: room promotion survives save/load ===\n");
  sim_shutdown();
  InitSim();
  World promo_world;
  promo_world.Init(5, 1, oxygen, 1.0f, 300.0f);
  Boot(t, promo_world);
  InjectGas(0, oxygen, 5.0f, 300.0f);
  Tick(promo_world, &visible);  // lands the injection, activates volume-fractions
  ext::PromoteRoomMessage promo_msg{0};
  sim_handle_message(ext::kPromoteRoom, static_cast<int>(sizeof(promo_msg)),
                     reinterpret_cast<const uint8_t*>(&promo_msg));
  Tick(promo_world, &visible);  // lands the promotion (one-tick latency, same as every ext:: message)
  Check(sim_debug_room_owned(0) == 1, "room is promoted before the save");

  const std::vector<uint8_t> promo_blob = Save(0, 0);
  int32_t promo_version = -1;
  if (promo_blob.size() > 12) memcpy(&promo_version, promo_blob.data() + 8, 4);
  printf("  promoted-room blob: %zu bytes, version=%d\n", promo_blob.size(), promo_version);
  Check(promo_version == 19,
        "a world with a promoted room saves at kSaveVersionElementPalette (19) too -- one "
        "section for both, which is what stopped the save-version chain growing");
  DumpBlob("promoted-room", promo_blob);

  sim_shutdown();
  InitSim();
  SendRaw(SimMessageHash::Elements_CreateTable, t.elements);
  SendRaw(SimMessageHash::Disease_CreateTable, t.diseases);
  Check(sim_debug_room_owned(0) == -1,
        "sanity check: a brand-new sim instance has no room at all yet");
  sim_handle_message(static_cast<int32_t>(SimMessageHash::Load),
                     static_cast<int>(promo_blob.size()), promo_blob.data());
  Check(sim_debug_room_owned(0) == 1,
        "promotion survives a full save/load round-trip across a fresh sim instance");
  Check(sim_debug_room_owned(4) == 1,
        "promotion survives per-ROOM, not per-cell -- cell 4 in the same room reports owned "
        "too, straight off BuildRoomGraph re-deriving owned from the restored per-cell bit");

  // `Load` takes Vacuum's heat away, as Klei's does (World::ReadLoadedCell) -- and a cell whose
  // gas the mixture holds IS vanilla Vacuum, with the mixture's temperature in the same field.
  // Without World::KeepMixtureTemperature, every such cell would load at 0 K and 0 Pa and
  // stay there. Both load paths, plus a control: a Vacuum cell the mixture does not occupy,
  // saved at a temperature, still loads at 0 K.
  printf("\n=== vftest: a mixture-held Vacuum cell keeps its temperature through Load ===\n");
  {
    uint16_t granite_mix = 0;
    if (!Resolve(t, kGranite, "Granite", &granite_mix)) return 1;
    constexpr int32_t kMixCell = 1, kPlainCell = 3, kMixW = 4;
    sim_shutdown();
    InitSim();
    World mix_world;
    mix_world.Init(kMixW, 1, granite_mix, 1000.0f, 300.0f);
    mix_world.element[kMixCell] = oxygen;
    mix_world.mass[kMixCell] = 1.0f;
    mix_world.temperature[kMixCell] = 250.0f;
    Boot(t, mix_world);
    std::vector<uint8_t> mix_visible;
    InjectGas(kMixCell, oxygen, 5.0f, 250.0f);
    Tick(mix_world, &mix_visible);
    ext::RemoveVanillaMassMessage mix_drain{kMixCell, 1.0f};
    sim_handle_message(ext::kRemoveVanillaMass, static_cast<int>(sizeof(mix_drain)),
                       reinterpret_cast<const uint8_t*>(&mix_drain));
    const GameDataUpdate* mu = nullptr;
    for (int i = 0; i < 3; ++i) mu = Tick(mix_world, &mix_visible);
    const float mix_pressure = sim_gas_pressure(kMixCell);
    Check(mu && mu->elementIdx[kMixCell] == vacuum && mu->mass[kMixCell] == 0.0f &&
              mu->temperature[kMixCell] == 250.0f && mix_pressure > 0.0f,
          "sanity check: before the save the cell is vanilla Vacuum at 250 K holding mixture "
          "O2 under real pressure");
    std::vector<uint8_t> mix_blob = Save(0, 0);
    // The control, written into the blob: the padded row is 1, the padded column cell + 1.
    const size_t plain_at = 29 + static_cast<size_t>(1 * (kMixW + 2) + kPlainCell + 1) * 16;
    const float plain_temp = 280.0f;
    if (mix_blob.size() > plain_at + 16) {
      memcpy(mix_blob.data() + plain_at, &kVacuumHash, 4);
      memcpy(mix_blob.data() + plain_at + 4, &plain_temp, 4);
    }

    sim_shutdown();
    InitSim();
    SendRaw(SimMessageHash::Elements_CreateTable, t.elements);
    SendRaw(SimMessageHash::Disease_CreateTable, t.diseases);
    SendRaw(SimMessageHash::Load, mix_blob);
    mu = static_cast<const GameDataUpdate*>(SendEmpty(SimMessageHash::Start));
    Check(mu && mu->elementIdx[kMixCell] == vacuum && mu->temperature[kMixCell] == 250.0f &&
              sim_gas_pressure(kMixCell) == mix_pressure,
          "saved-game path: the mixture-held Vacuum cell loads at 250 K and the same pressure");
    Check(mu && mu->elementIdx[kPlainCell] == vacuum && mu->temperature[kPlainCell] == 0.0f,
          "saved-game path: a Vacuum cell the mixture does not hold still loads at 0 K");

    // The cluster path: the same blob, re-headed to (2,1) inside a 10 x 4 allocation.
    constexpr int32_t kMixOffX = 2, kMixOffY = 1, kMixClusterW = 10, kMixClusterH = 4;
    std::vector<uint8_t> mix_cluster_blob = mix_blob;
    memcpy(mix_cluster_blob.data() + 20, &kMixOffX, 4);
    memcpy(mix_cluster_blob.data() + 24, &kMixOffY, 4);
    sim_shutdown();
    InitSim();
    SendRaw(SimMessageHash::Elements_CreateTable, t.elements);
    {
      Writer alloc;
      alloc.Put<int32_t>(kMixClusterW);
      alloc.Put<int32_t>(kMixClusterH);
      Send(SimMessageHash::AllocateCells, alloc);
    }
    SendRaw(SimMessageHash::Disease_CreateTable, t.diseases);
    {
      Writer offsets;
      for (int32_t v : {1, kMixOffX, kMixOffY, kMixW, 1}) offsets.Put<int32_t>(v);
      Send(SimMessageHash::DefineWorldOffsets, offsets);
    }
    SendEmpty(SimMessageHash::ClearUnoccupiedCells);
    SendRaw(SimMessageHash::Load, mix_cluster_blob);
    mu = static_cast<const GameDataUpdate*>(SendEmpty(SimMessageHash::Start));
    const int32_t cluster_mix = kMixOffY * kMixClusterW + kMixOffX + kMixCell;
    const int32_t cluster_plain = kMixOffY * kMixClusterW + kMixOffX + kPlainCell;
    Check(mu && mu->elementIdx[cluster_mix] == vacuum && mu->temperature[cluster_mix] == 250.0f &&
              sim_gas_pressure(cluster_mix) == mix_pressure,
          "cluster path: the mixture-held Vacuum cell loads at 250 K and the same pressure");
    Check(mu && mu->elementIdx[cluster_plain] == vacuum && mu->temperature[cluster_plain] == 0.0f,
          "cluster path: a Vacuum cell the mixture does not hold still loads at 0 K");
  }

  // Klei's load-time state transition (`DoLoadTimeStateTransition`) moves a cell
  // more than 3 K out of its element's range across one transition. This fork runs it as a pass
  // after the blob is down (`LoadTimeStateTransitions`, sim/physics.h), gated by the same phase
  // rules the frame's transition asks. `worldgen_test` checks the ungated rule against Klei;
  // this checks the gate. ONE BLOB, TWO LOADS, differing only in whether the rules were pushed
  // between `Elements_CreateTable` and `Load` -- which is the window the framework pushes them
  // in. Three cells: Steam at 300 K (condenses under Klei's rule), Water at 400 K (boils), and
  // Granite at 5000 K, which no rule describes and which must melt in both loads -- the witness
  // that the pass ran at all in the load where the other two stayed put.
  printf("\n=== vftest: the load-time state transition, gated by the phase rules ===\n");
  {
    uint16_t lt_granite = 0, lt_water = 0, lt_steam = 0;
    if (!Resolve(t, kGranite, "Granite", &lt_granite)) return 1;
    if (!Resolve(t, simhost::kWater, "Water", &lt_water)) return 1;
    if (!Resolve(t, simhost::kSteam, "Steam", &lt_steam)) return 1;
    constexpr int32_t kLtW = 5, kLtH = 3;
    constexpr int32_t kSteamCell = 1, kWaterCell = 2, kGraniteCell = 3;  // all on row 0
    sim_shutdown();
    InitSim();
    World lt_world;
    lt_world.Init(kLtW, kLtH, lt_granite, 1000.0f, 300.0f);
    Boot(t, lt_world);
    std::vector<uint8_t> lt_blob = Save(0, 0);
    auto put = [&](int32_t game_cell, int32_t hash, float temperature, float mass) {
      const int32_t gx = game_cell % kLtW, gy = game_cell / kLtW;
      const size_t at = 29 + static_cast<size_t>((gy + 1) * (kLtW + 2) + gx + 1) * 16;
      if (lt_blob.size() < at + 16) return;
      memcpy(lt_blob.data() + at, &hash, 4);
      memcpy(lt_blob.data() + at + 4, &temperature, 4);
      memcpy(lt_blob.data() + at + 8, &mass, 4);
    };
    put(kSteamCell, static_cast<int32_t>(simhost::kSteam), 300.0f, 1.0f);
    put(kWaterCell, static_cast<int32_t>(simhost::kWater), 400.0f, 100.0f);
    put(kGraniteCell, static_cast<int32_t>(kGranite), 5000.0f, 100.0f);

    // a = 5e-4, b = 1 is the boil arm's curve: t_sat clamps to the 647 K critical temperature
    // at any pressure this cell has, so 400 K water is refused. `sim.can_condense` = 0 on Steam
    // is the plainest condensation refusal there is.
    const float lt_curve[5] = {5.0e-4f, 1.0f, 273.15f, 647.0f, 2256000.0f};
    for (int run = 0; run < 2; ++run) {
      const bool with_rules = run == 1;
      sim_shutdown();
      InitSim();
      SendRaw(SimMessageHash::Elements_CreateTable, t.elements);
      SendRaw(SimMessageHash::Disease_CreateTable, t.diseases);
      if (with_rules) {
        const int32_t curve_attr = sim_ext_attr_index(ext::kAttrPhaseCurve);
        const int32_t condense_attr = sim_ext_attr_index(ext::kAttrCanCondense);
        Check(curve_attr >= 0 && condense_attr >= 0,
              "sim.phase_curve and sim.can_condense resolve by name before any Load");
        for (int32_t k = 0; k < 5; ++k) {
          uint32_t bits = 0;
          memcpy(&bits, &lt_curve[k], 4);
          SetElementAttributeBits(curve_attr, static_cast<int32_t>(simhost::kSteam), k, bits);
        }
        SetElementAttributeF32(condense_attr, static_cast<int32_t>(simhost::kSteam), 0.0f);
      }
      SendRaw(SimMessageHash::Load, lt_blob);
      const GameDataUpdate* lu =
          static_cast<const GameDataUpdate*>(SendEmpty(SimMessageHash::Start));
      if (lu == nullptr) {
        Check(false, "the blob loads and starts");
        continue;
      }
      printf("  %-20s steam cell %u at %.2f K, water cell %u at %.2f K, granite cell %u at "
             "%.2f K  (water %u, steam %u, granite %u)\n",
             with_rules ? "rules pushed" : "control, no rules", lu->elementIdx[kSteamCell],
             static_cast<double>(lu->temperature[kSteamCell]), lu->elementIdx[kWaterCell],
             static_cast<double>(lu->temperature[kWaterCell]), lu->elementIdx[kGraniteCell],
             static_cast<double>(lu->temperature[kGraniteCell]), lt_water, lt_steam, lt_granite);
      Check(lu->elementIdx[kGraniteCell] != lt_granite &&
                lu->temperature[kGraniteCell] == 4998.5f && lu->mass[kGraniteCell] == 100.0f,
            with_rules ? "WITNESS: Granite at 5000 K, which no rule describes, still takes Klei's "
                         "load-time transition in the load where the rules are present"
                       : "Granite at 5000 K takes Klei's load-time transition (4998.5 K)");
      if (with_rules) {
        Check(lu->elementIdx[kSteamCell] == lt_steam && lu->temperature[kSteamCell] == 300.0f &&
                  lu->mass[kSteamCell] == 1.0f,
              "WITH THE RULES Steam at 300 K loads as saved: sim.can_condense = 0 refuses the "
              "load-time condensation exactly as it refuses the frame's");
        Check(lu->elementIdx[kWaterCell] == lt_water && lu->temperature[kWaterCell] == 400.0f &&
                  lu->mass[kWaterCell] == 100.0f,
              "WITH THE RULES Water at 400 K loads as saved: its saturation temperature is above "
              "its own, so the load-time boil is refused");
      } else {
        Check(lu->elementIdx[kSteamCell] == lt_water && lu->temperature[kSteamCell] == 301.5f,
              "CONTROL: with no rules Steam at 300 K loads as Water at 301.5 K, as in Klei");
        Check(lu->elementIdx[kWaterCell] == lt_steam && lu->temperature[kWaterCell] == 398.5f,
              "CONTROL: with no rules Water at 400 K loads as Steam at 398.5 K, as in Klei");
      }
    }
  }

  printf("\n=== vftest: SIM_GasPressure (real PV=nRT, Mod 1's compressor/tank mechanic) ===\n");
  sim_shutdown();
  InitSim();
  // A single isolated cell -- no neighbor for MixRoomPooled to spread mass into, so every kg
  // injected stays put for the life of the test. Keeps the expected-pressure arithmetic below
  // exact instead of having to account for a same-tick trickle the way the dominant-element
  // section above does.
  World pressure_world;
  pressure_world.Init(1, 1, oxygen, 1.0f, 300.0f);
  Boot(t, pressure_world);

  // Sentinel: -1 Pa, not 0 -- 0 Pa is a real, legitimate answer for an occupied-but-cold or
  // massless mixture (see SIM_GasPressure's own doc comment), so it can't double as "no data"
  // the way SIM_GasDominantElement's 0xFFFF can for a mass total.
  Check(sim_gas_pressure(0) == -1.0f,
        "a never-activated cell reports the -1 sentinel, not 0 Pa");
  Check(sim_gas_pressure(999) == -1.0f, "an out-of-range cell reports the -1 sentinel");

  const float kPressureTestTempK = 300.0f;  // pressure_world.Init's own fill temperature --
  // kInjectGasSpecies only adds mass to the mixture layer, it never touches Phase().temperature
  // (gas_mixture_abi.h's shared-temperature field), so the cell's vanilla init temperature is
  // exactly what CellPressure reads back.
  constexpr float kCellVolumeM3 = 1.0f;  // mirrors gas_mixture.h's own placeholder constant --
  // not included directly (this file stays ABI-only, see its top comment), so restated here
  // the same way kVacuumHash/kOxygenHash above already mirror constants from elsewhere.
  // NOT `t.At(oxygen)->molarMass`, and that is the whole point of this oracle. Klei's element
  // table stores the ATOMIC mass for the diatomic gases -- Oxygen 15.9994, not O2's 31.9988
  // (measured against the shipped elements/gas.yaml; full table on
  // ElementTable::MolecularMassOf in sim/world.h). Reading the table here would make the
  // oracle share the SimDLL's own bug and agree with it for the wrong reason, which is exactly
  // how the earlier 1000x unit error in this same expression survived undetected. An oracle
  // that derives its expectation from the thing under test is not an oracle. So: state O2's
  // real molecular mass independently, from chemistry, and let the DLL be checked against it.
  constexpr float kMolarMassO2GPerMol = 31.9988f;
  const float molar_mass = kMolarMassO2GPerMol;

  InjectGas(0, oxygen, 4.0f, kPressureTestTempK);
  Tick(pressure_world, &visible);
  float pressure_4kg = sim_gas_pressure(0);
  // molarMass is g/mol (Klei's own raw content-data units); the mass this test injects is
  // kg, so the mole count needs the same g<->kg conversion gas_mixture.h's own
  // kGramsPerKilogram makes explicit. An oracle that divided kg directly by g/mol would be off
  // by 1000x, and would agree with a sim that had the same bug.
  float expected_4kg =
      (4.0f * 1000.0f / molar_mass) * gas::kGasConstantR * kPressureTestTempK / kCellVolumeM3;
  printf("  4 kg O2 in a 1-cell world: pressure=%.1f Pa, expected=%.1f Pa\n", pressure_4kg,
         expected_4kg);
  Check(std::fabs(pressure_4kg - expected_4kg) < expected_4kg * 0.01f,
        "SIM_GasPressure matches an independently-computed PV=nRT for a known mass/temp");

  // The actual compressor/tank demonstration: doubling the mass packed into the SAME
  // fixed-volume cell should double the pressure, exactly like a real compressor concentrating
  // a room's gas into a smaller sealed tank -- kCellVolumeM3 is the same fixed placeholder for
  // every cell (gas_mixture.h's own comment), so two cells holding different moles at the same
  // temperature are directly pressure-comparable, which is what makes this mechanic legible.
  InjectGas(0, oxygen, 4.0f, kPressureTestTempK);  // same cell, same volume, now 8 kg total
  Tick(pressure_world, &visible);
  float pressure_8kg = sim_gas_pressure(0);
  printf("  8 kg O2 in the same 1-cell world: pressure=%.1f Pa (%.2fx the 4 kg reading)\n",
         pressure_8kg, pressure_8kg / pressure_4kg);
  Check(std::fabs(pressure_8kg - 2.0f * pressure_4kg) < pressure_4kg * 0.01f,
        "doubling the mass packed into one fixed-volume cell doubles its pressure");

  printf(
      "\n=== vftest: SIM_ComputeGasPressure (generalized PV=nRT/V, no cell needed) ===\n");
  // Why it is exported: a mod's tank or pipe needs this exact formula for its own contents
  // (an isolated tank's storage, a gas pipe's contents), and should call the sim's rather
  // than keep a managed copy. Both the tank and a pipe are containers with real
  // mass/temperature but NO cell/world backing at all (matches real vanilla's own design --
  // sim/conduits.h's header comment says the sim never sees pipe contents move, and a real Gas
  // Reservoir's own Storage is plain managed code too), so this has to work with zero
  // `g->world` involvement, unlike SIM_GasPressure above.
  //
  // Cross-check against SIM_GasPressure itself first: the SAME 8 kg O2 / 300 K state already
  // sitting in cell 0 above, read two different ways, must agree exactly -- proves this isn't
  // a second, subtly-different formula, it's the same one.
  {
    const uint16_t one_species[] = {oxygen};
    const float one_mass[] = {8.0f};
    float via_generalized = sim_compute_gas_pressure(one_species, one_mass, 1,
                                                       kPressureTestTempK, kCellVolumeM3);
    printf("  SIM_GasPressure(cell)=%.4f Pa, SIM_ComputeGasPressure(same state)=%.4f Pa\n",
           pressure_8kg, via_generalized);
    Check(std::fabs(via_generalized - pressure_8kg) < pressure_8kg * 0.0001f,
          "SIM_ComputeGasPressure agrees exactly with SIM_GasPressure for the same "
          "mass/temp/volume -- one formula, not two");
  }

  // The actual point of generalizing it: a TWO-species mix (Dalton's law), which a single real
  // cell's SIM_GasPressure can't cleanly isolate without a full mixing setup. A tank holding
  // both O2 and CO2 sums their independent partial pressures.
  {
    uint16_t co2 = 0;
    if (!Resolve(t, simhost::kCarbonDioxide, "CarbonDioxide", &co2)) return 1;
    const uint16_t two_species[] = {oxygen, co2};
    const float two_mass[] = {2.0f, 3.0f};
    // Same independently-stated values as expected_4kg above -- see its comment for why this
    // must not read the element table. CO2 is one of the entries Klei already has right for
    // the molecule it names (44.01), so it is stated here at the same literal the table
    // happens to carry; O2 is not.
    const float molar_o2 = kMolarMassO2GPerMol;
    constexpr float kMolarMassCO2GPerMol = 44.01f;
    const float molar_co2 = kMolarMassCO2GPerMol;
    // Same g<->kg fix as expected_4kg above.
    const float expected_moles = (2.0f * 1000.0f / molar_o2) + (3.0f * 1000.0f / molar_co2);
    const float expected_two_species =
        expected_moles * gas::kGasConstantR * kPressureTestTempK / kCellVolumeM3;
    float via_two_species = sim_compute_gas_pressure(two_species, two_mass, 2,
                                                       kPressureTestTempK, kCellVolumeM3);
    printf("  2 kg O2 + 3 kg CO2, one tank: pressure=%.1f Pa, expected=%.1f Pa\n",
           via_two_species, expected_two_species);
    Check(std::fabs(via_two_species - expected_two_species) < expected_two_species * 0.01f,
          "a two-species tank sums independent partial pressures (Dalton's law), matching an "
          "independently-computed expectation");
  }

  // A real container (a tank) with a SMALLER volume than a normal cell's placeholder should
  // read a proportionally higher pressure for the identical mass/temperature -- the whole
  // reason this takes an explicit volume instead of always assuming kCellVolumeM3 the way
  // SIM_GasPressure does.
  {
    const uint16_t one_species[] = {oxygen};
    const float one_mass[] = {8.0f};
    float half_volume = sim_compute_gas_pressure(one_species, one_mass, 1, kPressureTestTempK,
                                                  kCellVolumeM3 * 0.5f);
    Check(std::fabs(half_volume - 2.0f * pressure_8kg) < pressure_8kg * 0.01f,
          "halving the container's volume doubles the pressure for the same mass/temp");
  }

  // Sentinels: -1 Pa (never a real pressure), same collapse SIM_GasPressure itself uses --
  // 0 Pa stays a legitimate answer this can't also mean "no data."
  {
    const uint16_t one_species[] = {oxygen};
    const float one_mass[] = {8.0f};
    const float zero_mass[] = {0.0f};
    Check(sim_compute_gas_pressure(nullptr, one_mass, 1, kPressureTestTempK, kCellVolumeM3) ==
              -1.0f,
          "a null species pointer reports the -1 sentinel");
    Check(sim_compute_gas_pressure(one_species, one_mass, 0, kPressureTestTempK,
                                    kCellVolumeM3) == -1.0f,
          "a zero count reports the -1 sentinel");
    Check(sim_compute_gas_pressure(one_species, one_mass, 1, kPressureTestTempK, 0.0f) ==
              -1.0f,
          "a zero volume reports the -1 sentinel");
    Check(sim_compute_gas_pressure(one_species, zero_mass, 1, kPressureTestTempK,
                                    kCellVolumeM3) == -1.0f,
          "an all-zero mass list (zero moles) reports the -1 sentinel, same as no data");
  }

  printf(
      "\n=== vftest: SIM_CalculateCombinedTemperature (same real formula as "
      "ApplyInjectGasSpecies's own blend) ===\n");
  // Why it is exported, same as SIM_ComputeGasPressure above: a mod's tank needs this exact
  // mass-weighted-mean-clamped formula, and should call the sim's rather than keep a copy.
  {
    // Hand-worked case: 4 kg @ 300 K mixed with 4 kg @ 400 K -> equal mass, so the mean is
    // the midpoint, 350 K -- easy to eyeball as correct before trusting the clamp cases below.
    float equal_mix = sim_calculate_combined_temperature(4.0f, 300.0f, 4.0f, 400.0f);
    Check(std::fabs(equal_mix - 350.0f) < 0.01f,
          "equal masses at 300K/400K blend to the exact midpoint, 350K");

    // Mass-weighted, not a plain average: 9 kg @ 300 K (dominant) mixed with 1 kg @ 400 K
    // should land close to 300K, not 350K.
    float weighted_mix = sim_calculate_combined_temperature(9.0f, 300.0f, 1.0f, 400.0f);
    float expected_weighted = (9.0f * 300.0f + 1.0f * 400.0f) / 10.0f;
    printf("  9kg@300K + 1kg@400K: got %.2fK, expected %.2fK\n", weighted_mix,
           expected_weighted);
    Check(std::fabs(weighted_mix - expected_weighted) < 0.01f,
          "a 9:1 mass ratio blends close to the dominant mass's own temperature, matching the "
          "hand-computed weighted mean");

    // Zero total mass is the formula's own explicit guard (emitters.h: "if (total <= 0.0f)
    // return 0.0f") -- not a real physical case, but a real code path this test should still
    // exercise rather than only ever calling it with plausible masses.
    Check(sim_calculate_combined_temperature(0.0f, 300.0f, 0.0f, 400.0f) == 0.0f,
          "zero total mass returns 0, the formula's own explicit guard");
  }

  printf(
      "\n=== vftest: SIM_EqualizeSingleSpeciesMass (tank<->pipe share pressure, no implicit "
      "valve) ===\n");
  // Real motivation: a tank plumbed to a pipe with no valve between them should behave as ONE
  // connected system sharing pressure, not an active pump moving mass at a fixed rate -- the
  // user's own correction, real Stationeers keeps directionality in explicit valve buildings.
  {
    // Deliberately the RAW table field here, unlike the two pressure oracles above:
    // SIM_EqualizeSingleSpeciesMass takes molarMass as a direct argument and the value cancels
    // out of its answer analytically (pressures go as 1/M, so the mole delta goes as 1/M, and
    // the transferred mass is dn*M). Which number is passed cannot change what this test
    // asserts, so it stays on the field the mixing path itself uses. See gas_mixture.h's
    // "WHICH molar mass" comment.
    const float molar = t.At(oxygen)->molarMass;

    // Symmetric volumes: same built-in extra 0.5 damping MixPair itself has (see this
    // function's own doc comment) means even rate=1.0 only closes HALF the gap per call, not
    // all of it -- by design, matching MixPair's own "run and iterate" philosophy rather than
    // snapping to equilibrium in one step. Verify convergence over repeated calls instead of a
    // single-call exact match.
    {
      float mass_a = 8.0f, mass_b = 0.0f;
      const float temp = 300.0f, vol = 1.0f;

      float delta1 = sim_equalize_single_species_mass(molar, mass_a, temp, vol, mass_b, temp,
                                                        vol, 1.0f);
      printf("  8kg/0kg, equal 1m^3 volumes, rate=1.0, call 1: delta into A = %.4f kg\n",
             delta1);
      Check(delta1 < 0.0f, "mass flows OUT of the higher-pressure side (A) at rate=1.0");
      Check(std::fabs(delta1 - (-2.0f)) < 0.01f,
            "one call moves HALF the full-equalization amount (4kg), not all of it -- same "
            "built-in damping MixPair's own dmass*0.5 already has, not a bug in this function");
      mass_a += delta1;
      mass_b -= delta1;
      Check(mass_a > mass_b,
            "after one call the gap has narrowed (8/0 -> 6/2) but not closed -- still A > B");

      // Repeated calls (same shape as this tank polling once a second) should converge toward
      // an even split -- geometric convergence, same as letting MixPair run for several ticks.
      for (int iter = 0; iter < 30; ++iter) {
        float d = sim_equalize_single_species_mass(molar, mass_a, temp, vol, mass_b, temp, vol,
                                                     1.0f);
        mass_a += d;
        mass_b -= d;
      }
      float pa = sim_compute_gas_pressure(&oxygen, &mass_a, 1, temp, vol);
      float pb = sim_compute_gas_pressure(&oxygen, &mass_b, 1, temp, vol);
      printf("  after 30 more calls: A=%.4fkg (%.1fPa), B=%.4fkg (%.1fPa)\n", mass_a, pa,
             mass_b, pb);
      Check(std::fabs(pa - pb) < 1.0f,
            "repeated calls converge equal-volume sides to (very nearly) the same pressure");
      Check(std::fabs(mass_a - mass_b) < 0.01f,
            "repeated calls converge equal volumes at equal temperature to an even split");
    }

    // Asymmetric volumes: converged state should give the smaller-volume side LESS mass than
    // the larger one at the same final (equal) pressure -- P=nRT/V means less V needs less n
    // for the same P. This is the case a fixed-rate pull could never get right regardless of
    // tuning, since it has no notion of either side's volume at all.
    {
      float mass_a = 10.0f, mass_b = 0.0f;
      const float temp = 300.0f;
      const float vol_a = 1.0f;   // tank
      const float vol_b = 4.0f;   // a bigger pipe/container
      for (int iter = 0; iter < 40; ++iter) {
        float d = sim_equalize_single_species_mass(molar, mass_a, temp, vol_a, mass_b, temp,
                                                     vol_b, 1.0f);
        mass_a += d;
        mass_b -= d;
      }
      float pa = sim_compute_gas_pressure(&oxygen, &mass_a, 1, temp, vol_a);
      float pb = sim_compute_gas_pressure(&oxygen, &mass_b, 1, temp, vol_b);
      printf("  10kg/0kg, 1m^3 vs 4m^3, after 40 calls: A=%.4fkg (%.1fPa), B=%.4fkg (%.1fPa)\n",
             mass_a, pa, mass_b, pb);
      Check(std::fabs(pa - pb) < 1.0f,
            "asymmetric volumes still converge to the same final pressure");
      Check(mass_a < mass_b,
            "the smaller-volume side ends up holding less mass at the same shared pressure "
            "(P=nRT/V) -- the actual reason this couldn't just be MixPair reused as-is");
    }

    // Damping: half the rate should move (very close to) half the mass of a full-rate step,
    // same linear-in-rate behavior MixPair's own rate parameter has.
    {
      float mass_a = 8.0f, mass_b = 0.0f;
      const float temp = 300.0f, vol = 1.0f;
      float full = sim_equalize_single_species_mass(molar, mass_a, temp, vol, mass_b, temp, vol,
                                                      1.0f);
      float half = sim_equalize_single_species_mass(molar, mass_a, temp, vol, mass_b, temp, vol,
                                                      0.5f);
      printf("  rate=1.0 delta=%.4f, rate=0.5 delta=%.4f\n", full, half);
      Check(std::fabs(half - full * 0.5f) < 0.001f,
            "the transfer scales linearly with rate, same as MixPair's own damping");
    }

    // Sentinels: no transfer for a nonsensical molar mass or a non-positive volume on either
    // side, same "degrade to inert, never garbage" contract every other pure function here has.
    Check(sim_equalize_single_species_mass(0.0f, 8.0f, 300.0f, 1.0f, 0.0f, 300.0f, 1.0f, 1.0f) ==
              0.0f,
          "a non-positive molar mass returns 0, no transfer");
    Check(sim_equalize_single_species_mass(molar, 8.0f, 300.0f, 0.0f, 0.0f, 300.0f, 1.0f, 1.0f) ==
              0.0f,
          "a non-positive volume on side A returns 0, no transfer");
    Check(sim_equalize_single_species_mass(molar, 8.0f, 300.0f, 1.0f, 0.0f, 300.0f, 0.0f, 1.0f) ==
              0.0f,
          "a non-positive volume on side B returns 0, no transfer");
    Check(sim_equalize_single_species_mass(molar, 4.0f, 300.0f, 1.0f, 4.0f, 300.0f, 1.0f, 1.0f) ==
              0.0f,
          "two sides already at equal pressure return 0, no transfer");
  }

  printf(
      "\n=== vftest: SIM_LiquidVolumeFromMass / SIM_EqualizeLiquidVolumeMass (liquid "
      "tank<->pipe share fill fraction, not ideal-gas pressure) ===\n");
  // Why: liquids are tracked by VOLUME, not partial pressure (Stationeers equalizes liquid
  // volume ratio the same way) -- reusing SIM_EqualizeSingleSpeciesMass's ideal-gas math
  // for an incompressible liquid would be the wrong physics reused for the wrong material.
  {
    // Water: real Element::density, ~1000 kg/m^3.
    const float density = 1000.0f;

    Check(std::fabs(sim_liquid_volume_from_mass(1000.0f, density) - 1.0f) < 0.001f,
          "1000kg of a 1000 kg/m^3 liquid occupies 1 m^3");
    Check(sim_liquid_volume_from_mass(1000.0f, 0.0f) == 0.0f,
          "a non-positive density returns 0 volume, not a divide-by-zero");

    // Symmetric capacities: same built-in extra 0.5 damping EqualizeSingleSpecies has (see
    // that function's own doc comment, mirrored here) -- one rate=1.0 call closes HALF the
    // fill-fraction gap, not all of it.
    {
      float mass_a = 8.0f, mass_b = 0.0f;
      const float cap = 0.01f;  // 10 L tank, matches Chemistry.PipeVolume's order of magnitude

      float delta1 =
          sim_equalize_liquid_volume_mass(density, mass_a, cap, mass_b, cap, 1.0f);
      printf("  8kg/0kg, equal 0.01m^3 capacities, rate=1.0, call 1: delta into A = %.4f kg\n",
             delta1);
      Check(delta1 < 0.0f, "mass flows OUT of the fuller side (A) at rate=1.0");
      Check(std::fabs(delta1 - (-2.0f)) < 0.01f,
            "one call moves HALF the full-equalization amount (4kg), not all of it -- same "
            "built-in damping the gas version has");
      mass_a += delta1;
      mass_b -= delta1;
      Check(mass_a > mass_b, "after one call the gap has narrowed but not closed");

      for (int iter = 0; iter < 30; ++iter) {
        float d = sim_equalize_liquid_volume_mass(density, mass_a, cap, mass_b, cap, 1.0f);
        mass_a += d;
        mass_b -= d;
      }
      printf("  after 30 more calls: A=%.4fkg, B=%.4fkg\n", mass_a, mass_b);
      Check(std::fabs(mass_a - mass_b) < 0.01f,
            "repeated calls converge equal capacities to an even split");
    }

    // Asymmetric capacities: converged state should give the smaller-capacity side LESS mass
    // than the larger one, at the same final FILL FRACTION -- the case a fixed-rate pull could
    // never get right since it has no notion of either side's capacity at all.
    {
      float mass_a = 10.0f, mass_b = 0.0f;
      const float cap_a = 0.01f;  // tank
      const float cap_b = 0.04f;  // a bigger pipe run
      for (int iter = 0; iter < 40; ++iter) {
        float d = sim_equalize_liquid_volume_mass(density, mass_a, cap_a, mass_b, cap_b, 1.0f);
        mass_a += d;
        mass_b -= d;
      }
      float ratio_a = sim_liquid_volume_from_mass(mass_a, density) / cap_a;
      float ratio_b = sim_liquid_volume_from_mass(mass_b, density) / cap_b;
      printf(
          "  10kg/0kg, 0.01m^3 vs 0.04m^3, after 40 calls: A=%.4fkg (%.1f%% full), "
          "B=%.4fkg (%.1f%% full)\n",
          mass_a, ratio_a * 100.0f, mass_b, ratio_b * 100.0f);
      Check(std::fabs(ratio_a - ratio_b) < 0.01f,
            "asymmetric capacities still converge to the same fill fraction");
      Check(mass_a < mass_b,
            "the smaller-capacity side ends up holding less mass at the same fill fraction -- "
            "the actual reason this needs its own kernel, not the gas one reused");
    }

    // Damping: half the rate should move half the mass of a full-rate step.
    {
      float mass_a = 8.0f, mass_b = 0.0f;
      const float cap = 0.01f;
      float full = sim_equalize_liquid_volume_mass(density, mass_a, cap, mass_b, cap, 1.0f);
      float half = sim_equalize_liquid_volume_mass(density, mass_a, cap, mass_b, cap, 0.5f);
      printf("  rate=1.0 delta=%.4f, rate=0.5 delta=%.4f\n", full, half);
      Check(std::fabs(half - full * 0.5f) < 0.001f,
            "the transfer scales linearly with rate");
    }

    // Sentinels: no transfer for a non-positive density or a non-positive capacity on either
    // side, same "degrade to inert, never garbage" contract every other pure function here has.
    Check(sim_equalize_liquid_volume_mass(0.0f, 8.0f, 0.01f, 0.0f, 0.01f, 1.0f) == 0.0f,
          "a non-positive density returns 0, no transfer");
    Check(sim_equalize_liquid_volume_mass(density, 8.0f, 0.0f, 0.0f, 0.01f, 1.0f) == 0.0f,
          "a non-positive capacity on side A returns 0, no transfer");
    Check(sim_equalize_liquid_volume_mass(density, 8.0f, 0.01f, 0.0f, 0.0f, 1.0f) == 0.0f,
          "a non-positive capacity on side B returns 0, no transfer");
    Check(sim_equalize_liquid_volume_mass(density, 4.0f, 0.01f, 4.0f, 0.01f, 1.0f) == 0.0f,
          "two sides already at equal fill fraction return 0, no transfer");
  }

  {
    printf("\n=== vftest: SIM_AdiabaticFillTemperature (compression heat) ===\n");

    // gammaMix == 1.0 must reduce EXACTLY to plain mass-weighted calorimetry
    // (SIM_CalculateCombinedTemperature) -- the whole justification for calling this "the same
    // formula, generalized" rather than a brand-new one.
    {
      float adiabatic = sim_adiabatic_fill_temperature(1.0f, 4.0f, 300.0f, 4.0f, 400.0f);
      float plain = sim_calculate_combined_temperature(4.0f, 300.0f, 4.0f, 400.0f);
      Check(std::fabs(adiabatic - plain) < 1e-3f,
            "gammaMix=1.0 matches plain calorimetry exactly (350 K expected)");
    }

    // gammaMix > 1.0 (real compression: incoming gas arrives carrying enthalpy, not just its
    // own bare temperature) must push the result ABOVE what plain calorimetry alone would give
    // -- the entire physical point of this function existing.
    {
      float adiabatic = sim_adiabatic_fill_temperature(1.333f, 4.0f, 300.0f, 4.0f, 300.0f);
      float plain = sim_calculate_combined_temperature(4.0f, 300.0f, 4.0f, 300.0f);
      Check(adiabatic > plain,
            "compression (gammaMix>1) raises the result above plain mass-weighted mixing "
            "for the same two inputs");
      // Hand-computed: (4*300 + 4*(300*1.333)) / 8 = (1200 + 1599.6) / 8 = 349.95
      Check(std::fabs(adiabatic - 349.95f) < 0.1f,
            "matches the hand-derived tank-filling energy balance exactly");
    }

    // A tiny trickle into a large existing store should barely move the temperature, same
    // intuition as plain calorimetry -- compression heating doesn't break that limit.
    {
      float adiabatic =
          sim_adiabatic_fill_temperature(1.333f, 1000.0f, 300.0f, 0.01f, 300.0f);
      Check(std::fabs(adiabatic - 300.0f) < 0.5f,
            "a trickle into a huge existing mass barely moves the shared temperature");
    }

    // Sentinels: same "degrade to inert, never garbage" contract every pure function here has.
    Check(sim_adiabatic_fill_temperature(1.333f, 0.0f, 300.0f, 0.0f, 300.0f) == 0.0f,
          "zero total mass on both sides returns 0, not NaN/garbage");
    Check(sim_adiabatic_fill_temperature(1.333f, 8.0f, 300.0f, 0.0f, 999.0f) == 300.0f,
          "zero incoming mass leaves the existing temperature completely untouched");
  }

  {
    printf("\n=== vftest: SIM_ComputePhaseChangeStep (real phase change) ===\n");

    // Rising (evaporating/boiling): 10 kg at 210 K crossing a 200 K threshold, cheap enough
    // sensible heat (specificHeatCapacity=100) against an expensive latent cost
    // (latentHeatJPerKg=5000) that only 2 kg is affordable at all -- hand-derived:
    // availableEnergyJ = 10*100*10 = 10000 J, affordable = 10000/5000 = 2 kg.
    {
      float converted = -1.0f, remaining = -1.0f;
      sim_compute_phase_change_step(10.0f, 210.0f, 200.0f, 5000.0f, 100.0f, 1.0f, 1.0f, 0.0f,
                                     &converted, &remaining);
      Check(std::fabs(converted - 2.0f) < 1e-3f,
            "rising: converts exactly the energy-affordable 2 kg at rate=1, dt=1");
      // remainingMassKg=8, energyForConversionJ=10000, deltaTempK=10000/(8*100)=12.5 -- would
      // put the remainder at 197.5 K, PAST the threshold, so it must clamp to 200 K exactly.
      Check(std::fabs(remaining - 200.0f) < 1e-3f,
            "rising: remaining temperature never overshoots past the threshold (clamped)");
    }

    // Exactly at the threshold: no driving force, nothing should convert.
    {
      float converted = -1.0f, remaining = -1.0f;
      sim_compute_phase_change_step(10.0f, 200.0f, 200.0f, 5000.0f, 100.0f, 1.0f, 1.0f, 0.0f,
                                     &converted, &remaining);
      Check(converted == 0.0f, "temperatureK == thresholdK converts nothing");
      Check(remaining == 200.0f, "temperatureK == thresholdK leaves temperature unchanged");
    }

    // minRemainderKg floor: only 1.25 kg is energy-affordable (availableEnergyJ=10*50*5=2500,
    // /2000 J/kg = 1.25 kg), and 10-1.25=8.75 kg would be left -- below a minRemainderKg of 9,
    // so this converts the FULL affordable amount even though conversionRatePerSecond=0.1
    // would otherwise throttle it to 0.125 kg. This is the "don't leave physically meaningless
    // dust behind" floor, same motivation as Stationeers' own MINIMUM_WORLD_VALID_TOTAL_MOLES.
    {
      float converted = -1.0f, remaining = -1.0f;
      sim_compute_phase_change_step(10.0f, 205.0f, 200.0f, 2000.0f, 50.0f, 1.0f, 0.1f, 9.0f,
                                     &converted, &remaining);
      Check(std::fabs(converted - 1.25f) < 1e-3f,
            "minRemainderKg floor converts the full affordable amount, ignoring the rate "
            "throttle");
    }

    // Falling (condensing/freezing): symmetric to the rising case, temperature below the
    // threshold, remaining mass warms toward (and clamps at) the threshold instead of cooling.
    {
      float converted = -1.0f, remaining = -1.0f;
      sim_compute_phase_change_step(10.0f, 190.0f, 200.0f, 5000.0f, 100.0f, 1.0f, 1.0f, 0.0f,
                                     &converted, &remaining);
      Check(std::fabs(converted - 2.0f) < 1e-3f,
            "falling: converts exactly the energy-affordable 2 kg, symmetric to rising");
      Check(std::fabs(remaining - 200.0f) < 1e-3f,
            "falling: remaining temperature clamps at the threshold from below, not past it");
    }

    // Sentinels: same "degrade to inert, never garbage" contract every pure function here has.
    {
      float converted = -1.0f, remaining = -1.0f;
      sim_compute_phase_change_step(0.0f, 250.0f, 200.0f, 5000.0f, 100.0f, 1.0f, 1.0f, 0.0f,
                                     &converted, &remaining);
      Check(converted == 0.0f && remaining == 250.0f,
            "zero mass converts nothing and leaves temperature untouched");
    }
    {
      float converted = -1.0f, remaining = -1.0f;
      sim_compute_phase_change_step(10.0f, 250.0f, 200.0f, 0.0f, 100.0f, 1.0f, 1.0f, 0.0f,
                                     &converted, &remaining);
      Check(converted == 0.0f, "zero latent heat converts nothing (no valid cost to pay)");
    }
    {
      float converted = -1.0f, remaining = -1.0f;
      sim_compute_phase_change_step(10.0f, 250.0f, 200.0f, 5000.0f, 0.0f, 1.0f, 1.0f, 0.0f,
                                     &converted, &remaining);
      Check(converted == 0.0f, "zero specific heat capacity converts nothing");
    }
  }

  // Real root cause of "fresh worldgen crashes under our custom SimDLL" (Grid.InitializeCells
  // -> ArgumentOutOfRangeException) -- see World::LoadIntoCluster's own doc
  // comment. Reproduces SaveLoader.LoadFromWorldGen's managed call sequence: AllocateCells at
  // the CLUSTER's size (bigger than any one world), DefineWorldOffsets for two worlds,
  // ClearUnoccupiedCells, then one Load per world (each sized to just that world and saved at
  // its own offset, which is where Klei puts it), all with zero ticks in between. The fuller
  // check, which also passes against Klei's DLL, is driver/src/worldgen_test.cpp.
  printf("\n=== vftest: fresh multi-world worldgen no longer corrupts the cluster grid "
         "(Sim.Start -> Grid.InitializeCells crash) ===\n");

  // Build each small world's own blob exactly the way a real per-world worldgen blob looks --
  // a tiny, fully-populated, single-element world, saved at its own offset and captured as raw
  // bytes.
  constexpr int32_t kClusterW = 20, kClusterH = 10;
  constexpr int32_t kWorldAX = 2, kWorldAY = 2;   // world A's 5x3 sits at game (2,2)-(6,4)
  constexpr int32_t kWorldBX = 12, kWorldBY = 2;  // world B's 5x3 sits at game (12,2)-(16,4)
  sim_shutdown();
  InitSim();
  World worldA;
  worldA.Init(5, 3, oxygen, 2.0f, 320.0f);
  Boot(t, worldA);
  const std::vector<uint8_t> world_a_blob = Save(kWorldAX, kWorldAY);
  int32_t world_a_version = -1;
  if (world_a_blob.size() > 12) memcpy(&world_a_version, world_a_blob.data() + 8, 4);
  Check(world_a_version == 15, "world A's own blob is the plain legacy shape (kSaveVersion)");

  sim_shutdown();
  InitSim();
  World worldB;
  worldB.Init(5, 3, co2, 3.0f, 310.0f);
  Boot(t, worldB);
  const std::vector<uint8_t> world_b_blob = Save(kWorldBX, kWorldBY);

  // Now the real cluster sequence: allocate a canvas big enough to hold both worlds with real
  // gaps on every side, define both worlds' offsets, clear to vacuum, then load each world's
  // blob into its own sub-rectangle -- no ticks anywhere in this block, matching
  // LoadFromWorldGen exactly.

  sim_shutdown();
  InitSim();
  SendRaw(SimMessageHash::Elements_CreateTable, t.elements);
  SendRaw(SimMessageHash::Disease_CreateTable, t.diseases);
  {
    Writer alloc;
    alloc.Put<int32_t>(kClusterW);
    alloc.Put<int32_t>(kClusterH);
    Send(SimMessageHash::AllocateCells, alloc);
  }
  {
    Writer offsets;
    offsets.Put<int32_t>(2);  // two worlds
    offsets.Put<int32_t>(kWorldAX);
    offsets.Put<int32_t>(kWorldAY);
    offsets.Put<int32_t>(5);
    offsets.Put<int32_t>(3);
    offsets.Put<int32_t>(kWorldBX);
    offsets.Put<int32_t>(kWorldBY);
    offsets.Put<int32_t>(5);
    offsets.Put<int32_t>(3);
    Send(SimMessageHash::DefineWorldOffsets, offsets);
  }
  SendEmpty(SimMessageHash::ClearUnoccupiedCells);
  SendRaw(SimMessageHash::Load, world_a_blob);
  SendRaw(SimMessageHash::Load, world_b_blob);

  // The exact read Grid.InitializeCells performs, managed-side: elementIdx[i] for every game
  // cell in the WHOLE cluster, trusted as a valid index into the real element table with no
  // bounds check of its own (that trust is exactly what threw ArgumentOutOfRangeException).
  const GameDataUpdate* cluster_gdu =
      static_cast<const GameDataUpdate*>(SendEmpty(SimMessageHash::Start));
  Check(cluster_gdu != nullptr, "Start returns a real GameDataUpdate for the cluster-sized world");

  bool every_index_valid = true;
  for (int32_t i = 0; i < kClusterW * kClusterH; ++i) {
    if (cluster_gdu->elementIdx[i] >= static_cast<uint16_t>(t.count)) {
      every_index_valid = false;
      printf("  cell %d: elementIdx=%u is OUT OF RANGE (table has %d entries) -- this is "
             "exactly what threw ArgumentOutOfRangeException in Grid.InitializeCells\n",
             i, cluster_gdu->elementIdx[i], t.count);
      break;
    }
  }
  Check(every_index_valid,
        "every cell in the full cluster-sized grid resolves to a valid element index -- the "
        "exact read that used to crash");

  auto GameIndexAt = [&](int32_t x, int32_t y) { return y * kClusterW + x; };

  Check(cluster_gdu->elementIdx[GameIndexAt(kWorldAX, kWorldAY)] == oxygen &&
        std::fabs(cluster_gdu->mass[GameIndexAt(kWorldAX, kWorldAY)] - 2.0f) < 1e-3f,
        "world A's own cell (2,2) survived with its real element and mass, not overwritten "
        "by world B's later Load");
  Check(cluster_gdu->elementIdx[GameIndexAt(kWorldBX, kWorldBY)] == co2 &&
        std::fabs(cluster_gdu->mass[GameIndexAt(kWorldBX, kWorldBY)] - 3.0f) < 1e-3f,
        "world B's own cell (12,2) has its real element and mass -- the bug this fixes made "
        "this the ONLY thing left standing, with world A's data already discarded");

  // A real gap cell, covered by neither world's offset rectangle -- exactly the "space between
  // planets" ClearUnoccupiedCells exists to give a sane default to.
  Check(cluster_gdu->elementIdx[GameIndexAt(9, 5)] == vacuum &&
        cluster_gdu->mass[GameIndexAt(9, 5)] == 0.0f,
        "an unoccupied cell between the two worlds is real Vacuum, not an arbitrary element 0");

  // A world that never activates volume-fractions (never sent kInjectGasSpecies) must still
  // save at the legacy version -- the "byte-identical until used" guarantee, checked on the save path specifically.
  sim_shutdown();
  InitSim();
  World plain;
  plain.Init(3, 1, oxygen, 1.0f, 300.0f);
  Boot(t, plain);
  for (int i = 0; i < 3; ++i) Tick(plain, &visible);
  const std::vector<uint8_t> plain_blob = Save(0, 0);
  int32_t plain_version = -1;
  if (plain_blob.size() > 12) memcpy(&plain_version, plain_blob.data() + 8, 4);
  printf("  plain (never-activated) blob: %zu bytes, version=%d\n", plain_blob.size(),
         plain_version);
  Check(plain_version == 15,
        "a world that never activates volume-fractions still saves at the legacy version");
  // Pinned too, and it is the most important of the three. The v16/v17 fixtures prove stage 2
  // can still READ what earlier builds wrote; this one proves stage 2 did not change what a
  // world with nothing to say WRITES. That is the "byte-identical to Klei's own format"
  // guarantee, and comparing against a pinned file is a stronger check than re-deriving the
  // expected bytes from the code that produced them.
  DumpBlob("plain", plain_blob);

  // Real live report: a player's sealed multi-cell tank room (confirmed sealed
  // via the vanilla Oxygen overlay -- no bleed) drained an injected gas-mixture mass to
  // exactly 0 over "a few seconds" after the pump stopped sending any further messages.
  // Reproduce the exact shape: source room | solid wall | multi-cell tank room | solid wall,
  // one injection cycle (matching one real Compressor.cs pump tick), then MANY ticks with
  // zero further messages -- if total tank-room mass is conserved (just redistributed across
  // the room's own interior cells by MixRoomPooled), that's expected; if it measurably
  // shrinks with nothing removing it, that's the real bug.
  printf("\n=== vftest: sealed multi-cell tank room holds its mass with no further messages "
         "(real live report) ===\n");
  sim_shutdown();
  InitSim();
  uint16_t granite_leak = 0;
  if (!Resolve(t, kGranite, "Granite", &granite_leak)) return 1;
  constexpr int32_t kLeakW = 10;
  constexpr int32_t kLeakSource = 0, kLeakWallL = 1, kLeakTankStart = 2, kLeakTankEnd = 5,
                     kLeakWallR = 6;  // tank room is cells 2..5 inclusive, 4 cells
  constexpr int32_t kLeakUnrelated = 8;  // far cell, unrelated open space (cells 7..9)
  World leak_world;
  leak_world.Init(kLeakW, 1, oxygen, 1.0f, 300.0f);
  leak_world.element[kLeakWallL] = granite_leak;
  leak_world.mass[kLeakWallL] = 2000.0f;
  leak_world.element[kLeakWallR] = granite_leak;
  leak_world.mass[kLeakWallR] = 2000.0f;
  for (int32_t c = kLeakTankStart; c <= kLeakTankEnd; ++c) {
    leak_world.element[c] = oxygen;
    leak_world.mass[c] = 0.001f;  // trace vanilla mass, same reasoning as the pressure test:
                                   // a true 0-mass cell starts at vanilla's ZeroMasslessCells
                                   // 0 K, which is not what this test is checking.
  }
  const GameDataUpdate* gl = Boot(t, leak_world);
  std::vector<uint8_t> leak_visible;
  gl = Tick(leak_world, &leak_visible);
  gl = Tick(leak_world, &leak_visible);

  ext::PromoteRoomMessage promote_source{kLeakSource};
  sim_handle_message(ext::kPromoteRoom, static_cast<int>(sizeof(promote_source)),
                     reinterpret_cast<const uint8_t*>(&promote_source));
  ext::PromoteRoomMessage promote_tank{kLeakTankStart};
  sim_handle_message(ext::kPromoteRoom, static_cast<int>(sizeof(promote_tank)),
                     reinterpret_cast<const uint8_t*>(&promote_tank));
  gl = Tick(leak_world, &leak_visible);
  Check(sim_debug_room_owned(kLeakSource) == 1, "the source room is promoted");
  Check(sim_debug_room_owned(kLeakTankStart) == 1, "the tank room is promoted");

  auto TotalTankMass = [&]() {
    float total = 0.0f;
    for (int32_t c = kLeakTankStart; c <= kLeakTankEnd; ++c) {
      total += sim_debug_gas_mass(c, oxygen);
    }
    return total;
  };

  // Several simulated pump cycles, not one -- Compressor.cs's real intake runs once/second
  // for as long as the pump is powered, each cycle re-sending PromoteRoom on BOTH cells
  // (GasMixtureTankComponent.Update() every second regardless of pump state, plus the
  // intake's own PromoteRoom(sourceCell) call while it's active) before each Tick.
  const float leak_temp = gl->temperature[kLeakSource];
  for (int cycle = 0; cycle < 6; ++cycle) {
    ext::RemoveVanillaMassMessage leak_remove{kLeakSource, 5.0f};
    sim_handle_message(ext::kRemoveVanillaMass, static_cast<int>(sizeof(leak_remove)),
                       reinterpret_cast<const uint8_t*>(&leak_remove));
    InjectGas(kLeakTankStart, oxygen, 5.0f, leak_temp);
    sim_handle_message(ext::kPromoteRoom, static_cast<int>(sizeof(promote_source)),
                       reinterpret_cast<const uint8_t*>(&promote_source));
    sim_handle_message(ext::kPromoteRoom, static_cast<int>(sizeof(promote_tank)),
                       reinterpret_cast<const uint8_t*>(&promote_tank));
    gl = Tick(leak_world, &leak_visible);
  }
  const float mass_right_after_injection = TotalTankMass();
  printf("  total tank-room gas-mixture mass right after %d pump cycles: %.4f kg\n", 6,
         mass_right_after_injection);
  Check(mass_right_after_injection > 20.0f,
        "the injected mass actually accumulated in the tank room");

  // Now "the pump stops," matching Compressor.cs exactly: no more RemoveVanillaMass/Inject,
  // but GasMixtureTankComponent.Update() keeps firing PromoteRoom(tank.Cell) every second,
  // forever, regardless of the pump. 25 ticks at 0.2s/tick = 5 real seconds, matching the
  // "a few seconds" timescale reported live; PromoteRoom re-sent every 5th tick (~1 real
  // second), matching its own 1-second interval.
  for (int i = 0; i < 25; ++i) {
    if (i % 5 == 0) {
      sim_handle_message(ext::kPromoteRoom, static_cast<int>(sizeof(promote_tank)),
                         reinterpret_cast<const uint8_t*>(&promote_tank));
    }
    gl = Tick(leak_world, &leak_visible);
  }
  const float mass_after_idle = TotalTankMass();
  printf("  total tank-room gas-mixture mass after 25 idle ticks (5 real seconds) with the "
         "pump stopped but PromoteRoom still firing every second: %.4f kg\n",
         mass_after_idle);
  Check(mass_after_idle > mass_right_after_injection * 0.99f,
        "a sealed multi-cell tank room's total gas-mixture mass survives idle ticks with no "
        "further injection -- it should redistribute across the room's own cells, not shrink");

  // Real colonies never sit still: a duplicant digging or a door cycling ANYWHERE on the map
  // sends ChangeCellProperties (kGasImpermeable) and marks World::RoomsDirty, forcing a FULL
  // BuildRoomGraph rebuild on the very next tick -- not just for the room that changed, for
  // EVERY room. Does an unrelated rebuild elsewhere disturb an already-sealed, already-idle
  // tank room's mass?
  printf("  probing: does an UNRELATED room-graph rebuild elsewhere on the map disturb the "
         "tank room's already-idle mass?\n");
  CellPropertiesMessage unrelated_wall{};
  unrelated_wall.cellIdx = kLeakUnrelated;
  unrelated_wall.callbackIdx = -1;
  unrelated_wall.properties = 1;  // kGasImpermeable
  unrelated_wall.set = 1;
  sim_handle_message(static_cast<int32_t>(SimMessageHash::ChangeCellProperties),
                     sizeof(unrelated_wall), reinterpret_cast<const uint8_t*>(&unrelated_wall));
  gl = Tick(leak_world, &leak_visible);  // forces the rebuild (RoomsDirty -> BuildRoomGraph)
  unrelated_wall.set = 0;  // and back open again, like a door cycling
  sim_handle_message(static_cast<int32_t>(SimMessageHash::ChangeCellProperties),
                     sizeof(unrelated_wall), reinterpret_cast<const uint8_t*>(&unrelated_wall));
  gl = Tick(leak_world, &leak_visible);  // forces a second rebuild
  const float mass_after_unrelated_rebuild = TotalTankMass();
  printf("  total tank-room gas-mixture mass after an unrelated rebuild cycle: %.4f kg\n",
         mass_after_unrelated_rebuild);
  Check(mass_after_unrelated_rebuild > mass_after_idle * 0.99f,
        "an unrelated room-graph rebuild elsewhere on the map does not disturb the "
        "already-sealed, already-idle tank room's mass");
  Check(sim_debug_room_owned(kLeakTankStart) == 1,
        "the tank room is still promoted after the unrelated rebuild");

  // ------------------------------------------------------------------------------------
  // ext::kSetBuildingWasteHeatKilowatts -- the power -> heat rule's native field.
  //
  // Here rather than in diffsim for the reason every extension message is: it is
  // byte-identical until sent, so no scenario in the offline suite exercises it, and an
  // extension message nothing sends is an extension message nothing tests. That is not
  // hypothetical in this repo -- QueueDeferredMessage silently dropped EVERY extension
  // message for weeks, and only a vftest arm could have caught it.
  printf("\n=== vftest: building waste heat (ext::kSetBuildingWasteHeatKilowatts) ===\n");
  {
    sim_debug_energy_ledger = reinterpret_cast<int (*)(double*, int)>(
        GetProcAddress(g_sim, "SIM_DebugEnergyLedger"));
    Check(sim_debug_energy_ledger != nullptr, "SIM_DebugEnergyLedger is exported");

    World bw;
    // Insulated granite everywhere so the building's own conduction to the cells under it
    // is small next to the waste heat, and the arithmetic below is about the one field
    // being tested rather than about the exchange.
    bw.Init(5, 3, oxygen, 1.0f, 300.0f);
    Boot(t, bw);
    std::vector<uint8_t> bvis;

    // Klei's own per-frame SetDebugProperties. Without it both scales sit at the
    // constructor default and every building transfer runs at a different rate than the
    // game's -- the same reason diffsim's SendScales exists.
    DebugProperties scales{};
    scales.buildingTemperatureScale = 0.001f;
    scales.buildingToBuildingTemperatureScale = 0.001f;
    sim_handle_message(static_cast<int32_t>(SimMessageHash::SetDebugProperties),
                       sizeof(scales), reinterpret_cast<const uint8_t*>(&scales));

    AddBuildingHeatExchangeMessage add{};
    add.callbackIdx = -1;
    add.elemIdx = oxygen;   // any element with a real specific heat; the mass is what counts
    add.mass = 400.0f;
    add.temperature = 300.0f;
    add.thermalConductivity = 0.0f;  // isolate the field under test from cell exchange
    add.overheatTemperature = 1.0e9f;
    add.operatingKilowatts = 0.0f;
    add.minX = 1;
    add.minY = 1;
    add.maxX = 3;
    add.maxY = 2;
    sim_handle_message(static_cast<int32_t>(SimMessageHash::AddBuildingHeatExchange),
                       sizeof(add), reinterpret_cast<const uint8_t*>(&add));

    // TWO ticks, not one. A queued message lands on the tick AFTER it is sent, so the
    // first tick only enqueues the Add and the building does not exist yet when the update
    // for it is published. Reading the handle after one tick got -1 here, which then made
    // every later assertion in this arm meaningless -- including one that passed, off a
    // 0 K baseline.
    Tick(bw, &bvis);
    const GameDataUpdate* u = Tick(bw, &bvis);
    Check(u != nullptr && u->numBuildingTemperatures == 1,
          "the probe building registered and publishes a temperature");
    int32_t handle = -1;
    float t_before = 0.0f;
    if (u && u->numBuildingTemperatures == 1) {
      handle = u->buildingTemperatures[0].handle;
      t_before = u->buildingTemperatures[0].temperature;
    }
    // Nothing below means anything without a live handle, and a run that carried on would
    // report passes it had not earned.
    Check(handle >= 0, "the probe building has a usable sim handle");

    // Baseline: no waste heat sent yet, so nothing may have been charged.
    double led[16] = {0};
    const int nfields = sim_debug_energy_ledger ? sim_debug_energy_ledger(led, 16) : 0;
    Check(nfields >= 9, "the energy ledger publishes the waste-heat field");
    Check(led[8] == 0.0, "no waste heat is charged before the message is ever sent");
    // Field 1 is the building reservoir: heat_capacity * temperature. Captured here, while
    // the building is still at its registered temperature, so its heat capacity can be
    // recovered by division below.
    const double baseline_building = led[1];

    // Send it, then run a known number of frames at a known dt.
    ext::SetBuildingWasteHeatKilowattsMessage wh{};
    wh.handle = handle;
    wh.kilowatts = 10.0f;
    sim_handle_message(ext::kSetBuildingWasteHeatKilowatts, static_cast<int>(sizeof(wh)),
                       reinterpret_cast<const uint8_t*>(&wh));

    const int kFrames = 20;
    for (int i = 0; i < kFrames; ++i) u = Tick(bw, &bvis);
    const float t_after = (u && u->numBuildingTemperatures == 1)
                              ? u->buildingTemperatures[0].temperature
                              : t_before;
    sim_debug_energy_ledger(led, 16);
    printf("  building %.4f K -> %.4f K, bldgwasteheat %.4f kJ\n", t_before, t_after, led[8]);

    // Not a magic threshold. The building's total heat capacity is recoverable from the
    // ledger itself -- field 1 is heat_capacity * temperature -- so the rise the charged
    // energy should have produced is arithmetic, and asserting THAT is far stronger than
    // asserting "it went up a bit". The building's thermalConductivity is 0, so no energy
    // leaves to the cells and the two numbers have to agree outright.
    //
    // Written first as `t_after > t_before + 1.0f`, which failed: 40 kJ into 402 kJ/K is
    // 0.0995 K, and the threshold was picked without doing the division. The sim was right
    // and the assertion was wrong.
    const double total_hc = t_before > 0.0f ? baseline_building / t_before : 0.0;
    // ONE FRAME of lag between the two readings, and it is the threading model rather than
    // an error. `SIM_DebugEnergyLedger` calls WaitIdle() and therefore sees the frame that
    // just finished; the pointer `PrepareGameData` hands back is the PREVIOUS completed
    // frame's update, because the sim frame runs on a Win32 worker. So the published
    // temperature is always exactly one frame behind the ledger.
    //
    // Measured, not assumed: the gap is 0.004989 K over 20 frames and 0.005005 K over 40 --
    // constant in absolute terms, which is what a fixed one-frame offset looks like and
    // what a proportional error does not.
    const double per_frame = 0.2 * 10.0;  // dt seconds * kW = kJ
    const double expected_rise = total_hc > 0.0 ? (led[8] - per_frame) / total_hc : 0.0;
    printf("  total heat capacity %.4f kJ/K, expected rise %.6f K (ledger minus the one "
           "frame the published update lags by), actual %.6f K\n",
           total_hc, expected_rise, static_cast<double>(t_after - t_before));
    Check(total_hc > 0.0 && expected_rise > 0.0 &&
              std::fabs((t_after - t_before) - expected_rise) < 0.01 * expected_rise,
          "the building's temperature rise matches the energy the ledger says went in");
    // The message lands on the frame AFTER it is sent, so the
    // charge covers kFrames-1 frames, not kFrames. Checked as a band rather than an
    // equality because the frame the message lands on is the thing being asserted, and
    // pinning it exactly would make this test about latency instead.
    Check(led[8] > per_frame * (kFrames - 2) && led[8] < per_frame * (kFrames + 1),
          "the waste-heat bucket charges dt * kW per frame, once the message has landed");

    // The preservation test, and the subtle one. Klei's ModifyBuildingHeatExchange is a
    // WHOLESALE replacement of the record and the game sends one whenever a building's
    // payload is dirty -- every operational change, every temperature edit. If the field
    // were not carried across, the rule would switch itself off at unpredictable moments
    // with no symptom at all.
    const double before_modify = led[8];
    ModifyBuildingHeatExchangeMessage mod{};
    mod.callbackIdx = handle;   // the handle rides in callbackIdx on this message
    mod.elemIdx = add.elemIdx;
    mod.mass = add.mass;
    mod.temperature = t_after;
    mod.thermalConductivity = add.thermalConductivity;
    mod.overheatTemperature = add.overheatTemperature;
    mod.operatingKilowatts = 0.0f;
    mod.minX = add.minX;
    mod.minY = add.minY;
    mod.maxX = add.maxX;
    mod.maxY = add.maxY;
    sim_handle_message(static_cast<int32_t>(SimMessageHash::ModifyBuildingHeatExchange),
                       sizeof(mod), reinterpret_cast<const uint8_t*>(&mod));
    for (int i = 0; i < kFrames; ++i) Tick(bw, &bvis);
    sim_debug_energy_ledger(led, 16);
    printf("  bldgwasteheat after a wholesale Modify: %.4f kJ (was %.4f)\n", led[8],
           before_modify);
    Check(led[8] > before_modify + per_frame * (kFrames - 2),
          "a wholesale ModifyBuildingHeatExchange does not wipe the waste-heat rate");

    // And 0 switches it off, which is how the managed side stops a building that has gone
    // inactive rather than by unregistering anything.
    wh.kilowatts = 0.0f;
    sim_handle_message(ext::kSetBuildingWasteHeatKilowatts, static_cast<int>(sizeof(wh)),
                       reinterpret_cast<const uint8_t*>(&wh));
    for (int i = 0; i < 3; ++i) Tick(bw, &bvis);
    const double after_off = led[8];
    sim_debug_energy_ledger(led, 16);
    for (int i = 0; i < kFrames; ++i) Tick(bw, &bvis);
    double led2[16] = {0};
    sim_debug_energy_ledger(led2, 16);
    printf("  bldgwasteheat after sending 0: %.4f kJ, then %.4f kJ\n", led[8], led2[8]);
    Check(led2[8] == led[8] && led[8] >= after_off,
          "sending 0 kW stops the charge entirely");
  }

  // ------------------------------------------------------------------------------------
  // The region asymmetry. the game's `OperatingKilowatts` is applied once per REGION; the
  // framework's waste_heat_kilowatts is applied once per SUBSTEP. This arm is the
  // regression test for both halves of that at once, which is the only way to be sure the
  // fix did not simply switch vanilla's behaviour off as well.
  //
  // Why it matters: a derived power -> heat rule whose output depended on a region layout
  // the player can neither see nor choose would not be a rule -- the same wattage would
  // make different heat in two differently laid-out colonies. Measured live at a 1.57-1.67x
  // multiplier on a real 234-cycle save before this fix.
  printf("\n=== vftest: waste heat is once per substep, operating heat once per region ===\n");
  {
    World rw;
    rw.Init(5, 3, oxygen, 1.0f, 300.0f);
    Boot(t, rw);
    std::vector<uint8_t> rvis;

    DebugProperties rscales{};
    rscales.buildingTemperatureScale = 0.001f;
    rscales.buildingToBuildingTemperatureScale = 0.001f;
    sim_handle_message(static_cast<int32_t>(SimMessageHash::SetDebugProperties),
                       sizeof(rscales), reinterpret_cast<const uint8_t*>(&rscales));

    // Buildings live outside the world and survive a Boot, so the earlier arm's probe is
    // still registered and this world has TWO. Record the handles that already exist and
    // pick out the one that is new, rather than assuming this is the only building --
    // assuming it left rhandle at -1, the waste-heat message addressed nothing, and the arm
    // reported a bldgwasteheat of exactly 0 that looked like the feature being broken.
    std::vector<int32_t> pre_existing;
    {
      const GameDataUpdate* pu = Tick(rw, &rvis);
      if (pu != nullptr) {
        for (int i = 0; i < pu->numBuildingTemperatures; ++i) {
          pre_existing.push_back(pu->buildingTemperatures[i].handle);
        }
      }
    }

    AddBuildingHeatExchangeMessage radd{};
    radd.callbackIdx = -1;
    radd.elemIdx = oxygen;
    radd.mass = 400.0f;
    radd.temperature = 300.0f;
    radd.thermalConductivity = 0.0f;
    radd.overheatTemperature = 1.0e9f;
    // 4 kW of Klei's own operating heat AND, below, 10 kW of the framework's. Both on the
    // same building, so one run answers both questions and neither can be confused for the
    // other.
    radd.operatingKilowatts = 4.0f;
    radd.minX = 1;
    radd.minY = 1;
    radd.maxX = 3;
    radd.maxY = 2;
    sim_handle_message(static_cast<int32_t>(SimMessageHash::AddBuildingHeatExchange),
                       sizeof(radd), reinterpret_cast<const uint8_t*>(&radd));

    Tick(rw, &rvis);
    const GameDataUpdate* ru = Tick(rw, &rvis);
    int32_t rhandle = -1;
    if (ru != nullptr) {
      for (int i = 0; i < ru->numBuildingTemperatures && rhandle < 0; ++i) {
        const int32_t h = ru->buildingTemperatures[i].handle;
        bool seen = false;
        for (size_t j = 0; j < pre_existing.size(); ++j) {
          if (pre_existing[j] == h) {
            seen = true;
            break;
          }
        }
        if (!seen) rhandle = h;
      }
    }
    Check(rhandle >= 0, "the two-region probe building has a usable sim handle");

    ext::SetBuildingWasteHeatKilowattsMessage rwh{};
    rwh.handle = rhandle;
    rwh.kilowatts = 10.0f;
    sim_handle_message(ext::kSetBuildingWasteHeatKilowatts, static_cast<int>(sizeof(rwh)),
                       reinterpret_cast<const uint8_t*>(&rwh));
    Tick(rw, &rvis);  // let it land

    // The earlier arm's building had its waste heat set to 0 already, but say so explicitly
    // for every pre-existing handle: this arm reads GLOBAL ledger buckets, so any other
    // building still charging would be silently folded into its numbers.
    for (size_t j = 0; j < pre_existing.size(); ++j) {
      ext::SetBuildingWasteHeatKilowattsMessage off{};
      off.handle = pre_existing[j];
      off.kilowatts = 0.0f;
      sim_handle_message(ext::kSetBuildingWasteHeatKilowatts, static_cast<int>(sizeof(off)),
                         reinterpret_cast<const uint8_t*>(&off));
    }
    Tick(rw, &rvis);

    double base[16] = {0};
    sim_debug_energy_ledger(base, 16);

    // Ten frames with ONE region, then ten with TWO. Same building, same rates, same dt.
    const int kRegionFrames = 10;
    for (int i = 0; i < kRegionFrames; ++i) Tick(rw, &rvis);
    double one[16] = {0};
    sim_debug_energy_ledger(one, 16);

    for (int i = 0; i < kRegionFrames; ++i) TickTwoRegions(rw, &rvis);
    double two[16] = {0};
    sim_debug_energy_ledger(two, 16);

    const double op_one = one[4] - base[4];
    const double op_two = two[4] - one[4];
    const double waste_one = one[8] - base[8];
    const double waste_two = two[8] - one[8];
    printf("  one region : bldgoperating %+.4f kJ, bldgwasteheat %+.4f kJ\n", op_one, waste_one);
    printf("  two regions: bldgoperating %+.4f kJ, bldgwasteheat %+.4f kJ\n", op_two, waste_two);

    // Klei's half: must still double. If this stops doubling, the fix has changed vanilla
    // behaviour, which is exactly what it must not do.
    Check(op_one > 0.0 && op_two > op_one * 1.8 && op_two < op_one * 2.2,
          "Klei's operating heat still doubles under two regions -- vanilla parity intact");
    // The framework's half: must NOT double.
    Check(waste_one > 0.0 && waste_two > waste_one * 0.9 && waste_two < waste_one * 1.1,
          "the framework's waste heat does NOT double under two regions -- once per substep");
  }

  // ------------------------------------------------------------------------------------
  // ext::kSetBuildingExhaust -- Klei's ExhaustHeat, as a rate the sim owns, with the part
  // that cannot be delivered going into the building's own body instead of being destroyed.
  //
  // The three arms are the three ways vanilla deletes it: a vacuum cell, a thin cell, and a
  // cell already at or above the ceiling. Each asserts BOTH halves -- that the delivery rules
  // are still Klei's, and that the remainder is conserved -- because a fix that quietly
  // stopped honouring the mass factor would look identical on the conservation number alone.
  printf("\n=== vftest: exhaust heat is bounced, not deleted (ext::kSetBuildingExhaust) ===\n");
  {
    // One 2x1 building. Three worlds, identical except for what is under it.
    struct ExhaustCase {
      const char* name;
      uint16_t fill_element;
      float fill_mass;
      float fill_temperature;
      float ceiling;
      bool expect_full_bounce;
    };
    const ExhaustCase cases[] = {
        // Vacuum: element state 0, refused outright. Everything bounces.
        {"vacuum", vacuum, 0.0f, 300.0f, 1000.0f, true},
        // A full cell well under the ceiling: Klei's mass factor saturates at 1.5 kg, so
        // delivery is whole and the bounce is ~0.
        {"full cell below the ceiling", oxygen, 20.0f, 300.0f, 1000.0f, false},
        // The ceiling Klei actually passes is the building's overheat temperature, 348.15 K
        // by default. A cell already past it takes nothing at all -- this is the path that
        // bites in every hot room in a real colony, and the one the vacuum framing misses.
        {"cell already above the ceiling", oxygen, 20.0f, 400.0f, 348.15f, true},
    };

    for (const ExhaustCase& c : cases) {
      // UNIFORM worlds, not a patch of two cells inside a different medium. Written the
      // other way first and all three arms were wrong for the same reason: the two ticks
      // it takes a building to register are two ticks of gas physics, so the pocket of
      // vacuum under the building had already been filled by its neighbours before the
      // exhaust ever ran, and the "cell above the ceiling" pocket had already started
      // equalising downwards. The sim was right and the world was not.
      World ew;
      ew.Init(5, 3, c.fill_element, c.fill_mass, c.fill_temperature);
      Boot(t, ew);
      std::vector<uint8_t> evis;

      DebugProperties scales{};
      scales.buildingTemperatureScale = 0.001f;
      scales.buildingToBuildingTemperatureScale = 0.001f;
      sim_handle_message(static_cast<int32_t>(SimMessageHash::SetDebugProperties),
                         sizeof(scales), reinterpret_cast<const uint8_t*>(&scales));

      // Read the building reservoir BEFORE this building exists. Field 1 is the sum over
      // EVERY registered building, and buildings outlive a world Boot, so the arms above
      // this one are still in that total -- dividing it by this building's temperature gives
      // a heat capacity that is not this building's. That was the first shape of the check
      // below and it failed for exactly that reason. The difference across the Add is this
      // building's own energy, and nothing else's.
      double pre[16] = {0};
      sim_debug_energy_ledger(pre, 16);

      AddBuildingHeatExchangeMessage add{};
      add.callbackIdx = -1;
      add.elemIdx = oxygen;
      add.mass = 400.0f;
      add.temperature = 300.0f;
      add.thermalConductivity = 0.0f;  // isolate the exhaust path from the conduction sweep
      add.overheatTemperature = 1.0e9f;
      add.operatingKilowatts = 0.0f;
      add.minX = 1;
      add.minY = 1;
      add.maxX = 3;
      add.maxY = 2;
      sim_handle_message(static_cast<int32_t>(SimMessageHash::AddBuildingHeatExchange),
                         sizeof(add), reinterpret_cast<const uint8_t*>(&add));

      // Two ticks: a queued message lands the tick AFTER it is sent. Buildings also outlive
      // a world Boot, so the handle is taken from this update rather than assumed to be the
      // only one -- the two-region arm below learned that the hard way.
      Tick(ew, &evis);
      const GameDataUpdate* u = Tick(ew, &evis);
      int32_t handle = -1;
      float t_before = 0.0f;
      if (u && u->numBuildingTemperatures >= 1) {
        handle = u->buildingTemperatures[u->numBuildingTemperatures - 1].handle;
        t_before = u->buildingTemperatures[u->numBuildingTemperatures - 1].temperature;
      }
      Check(handle >= 0, "the exhaust probe building has a usable sim handle");
      if (handle < 0) continue;

      double base[16] = {0};
      const int nfields = sim_debug_energy_ledger(base, 16);
      Check(nfields >= 16, "the energy ledger publishes the exhaust and radiation fields");
      const double baseline_building = base[1] - pre[1];

      ext::SetBuildingExhaustMessage ex{};
      ex.handle = handle;
      ex.kilowatts = 8.0f;  // a Coal Generator's ExhaustKilowattsWhenActive
      ex.maxTemperature = c.ceiling;
      sim_handle_message(ext::kSetBuildingExhaust, static_cast<int>(sizeof(ex)),
                         reinterpret_cast<const uint8_t*>(&ex));

      const int kFrames = 20;
      for (int i = 0; i < kFrames; ++i) u = Tick(ew, &evis);
      const float t_after = (u && u->numBuildingTemperatures >= 1)
                                ? u->buildingTemperatures[u->numBuildingTemperatures - 1]
                                      .temperature
                                : t_before;
      double led[16] = {0};
      sim_debug_energy_ledger(led, 16);
      const double charged = led[13] - base[13];
      const double bounced = led[14] - base[14];
      const double refused = led[12] - base[12];
      printf("  %-30s bldgexhaust %+8.4f kJ, bounced %+8.4f kJ, refused %+8.4f kJ, "
             "body %.4f -> %.4f K\n",
             c.name, charged, bounced, refused, t_before, t_after);

      // The rate is integrated over the sim's own substeps, so the expected charge is
      // dt * kW * frames, give or take the one frame of latency a queued message costs.
      const double expected = 0.2 * 8.0 * kFrames;
      Check(charged > expected * (1.0 - 1.5 / kFrames) && charged < expected * 1.01,
            "the exhaust bucket charges dt * kW over the run");
      // Nothing is destroyed on any of the three paths: the building has a body, so the
      // whole rate is accounted and the deletion bucket never moves.
      Check(refused == 0.0, "no exhaust energy reaches the deletion bucket");

      if (c.expect_full_bounce) {
        Check(bounced > charged * 0.99,
              "everything vanilla would have deleted lands in the building body instead");
      } else {
        Check(bounced < charged * 0.05,
              "a full cell below the ceiling still takes delivery, as vanilla does");
      }

      // The body's rise is arithmetic, not a threshold: this building's share of field 1 is
      // heat_capacity * temperature, so the heat capacity divides straight out of it.
      const double body_hc = baseline_building / static_cast<double>(t_before);
      const double expected_rise = bounced / body_hc;
      const double actual_rise = static_cast<double>(t_after) - static_cast<double>(t_before);
      Check(std::fabs(actual_rise - expected_rise) < 0.01 + std::fabs(expected_rise) * 0.02,
            "the body's temperature rise equals the bounced energy over its heat capacity");

      // Leave the rate off so the next case starts clean -- buildings outlive a Boot.
      ext::SetBuildingExhaustMessage off{};
      off.handle = handle;
      off.kilowatts = 0.0f;
      off.maxTemperature = c.ceiling;
      sim_handle_message(ext::kSetBuildingExhaust, static_cast<int>(sizeof(off)),
                         reinterpret_cast<const uint8_t*>(&off));
      Tick(ew, &evis);
      Tick(ew, &evis);
    }
  }

  // ------------------------------------------------------------------------------------
  // ext::kSetBuildingRadiation -- the outlet that stops the bounce above from simply cooking
  // every machine in a vacuum. Ported structurally from Stationeers'
  // AtmosphereHelper.CalculateEntropy; see abi/sim_abi_ext.h.
  //
  // The two arms are the whole claim: radiation is the ONLY outlet in vacuum, and it is
  // ~nothing in a pressurised room. The second is the safety property -- a radiation term
  // that cooled a full colony would be a new heat-deletion channel, not a fix for one.
  printf("\n=== vftest: building radiation (ext::kSetBuildingRadiation) ===\n");
  {
    struct RadiationCase {
      const char* name;
      uint16_t cell_element;
      float cell_mass;
      float cell_temperature;
      // Fraction of the vacuum case's flux this case should shed: 1 - min(mass/1.5, 1) per
      // cell. Exact, not a band -- the exposure factor is a straight multiplier.
      double expected_exposure;
    };
    const RadiationCase cases[] = {
        {"vacuum", vacuum, 0.0f, 400.0f, 1.0},
        // THE CASE THAT CAUGHT THE REAL BUG. Written first with the room at the body's own
        // temperature, which passed for the wrong reason: any form of the sink term gives
        // zero when the two are equal. A COLD room under a HOT body is the case a real colony
        // is full of, and the first implementation -- which lerped the sink toward the local
        // temperature, following Stationeers verbatim -- radiated hard here. Against a real
        // 234-cycle save that came to 55750 kJ over 32 s of sim time against 1380 kJ of
        // exhaust: a second building <-> cell heat path with the energy leaving the world.
        {"full room, body 100 K hotter than it", oxygen, 20.0f, 300.0f, 0.0},
        // Partial exposure, the middle of the range that neither extreme above tests.
        //
        // The mass is derived from the law, not picked: exposure ramps in PRESSURE against
        // Stationeers' 6.3 kPa Armstrong limit, so half exposure is 3.15 kPa, which for oxygen
        // (31.9988 g/mol) at 300 K is
        //     n = 3150 / (8.3144 * 300) = 1.2626 mol  ->  0.04040 kg.
        // Calibrated at 0.75 kg first, against the earlier mass-based ramp; that is 58 kPa and
        // is now correctly opaque, which is exactly the regression this arm should catch if the
        // threshold ever drifts back.
        {"thin cell at half the Armstrong limit, body 100 K hotter", oxygen, 0.04040f, 300.0f,
         0.5001},
    };

    for (const RadiationCase& c : cases) {
      // Uniform, for the same reason the exhaust worlds above are.
      World rw;
      rw.Init(5, 3, c.cell_element, c.cell_mass, c.cell_temperature);
      Boot(t, rw);
      std::vector<uint8_t> rvis;

      DebugProperties scales{};
      scales.buildingTemperatureScale = 0.001f;
      scales.buildingToBuildingTemperatureScale = 0.001f;
      sim_handle_message(static_cast<int32_t>(SimMessageHash::SetDebugProperties),
                         sizeof(scales), reinterpret_cast<const uint8_t*>(&scales));

      AddBuildingHeatExchangeMessage add{};
      add.callbackIdx = -1;
      add.elemIdx = oxygen;
      add.mass = 400.0f;
      add.temperature = 400.0f;  // hot body, so there is a gradient to shed across --
                                 // and, in two of the three cases, hotter than the room too
      add.thermalConductivity = 0.0f;
      add.overheatTemperature = 1.0e9f;
      add.operatingKilowatts = 0.0f;
      add.minX = 1;
      add.minY = 1;
      add.maxX = 3;
      add.maxY = 2;
      sim_handle_message(static_cast<int32_t>(SimMessageHash::AddBuildingHeatExchange),
                         sizeof(add), reinterpret_cast<const uint8_t*>(&add));

      Tick(rw, &rvis);
      const GameDataUpdate* u = Tick(rw, &rvis);
      int32_t handle = -1;
      float t_before = 0.0f;
      if (u && u->numBuildingTemperatures >= 1) {
        handle = u->buildingTemperatures[u->numBuildingTemperatures - 1].handle;
        t_before = u->buildingTemperatures[u->numBuildingTemperatures - 1].temperature;
      }
      Check(handle >= 0, "the radiation probe building has a usable sim handle");
      if (handle < 0) continue;

      double base[16] = {0};
      sim_debug_energy_ledger(base, 16);
      Check(base[15] == 0.0, "nothing is radiated before the message is ever sent");

      // Stationeers' own fallback when its planetary atmosphere is effectively absent.
      ext::SetEnvironmentTemperatureMessage env{};
      env.kelvin = 50.0f;
      sim_handle_message(ext::kSetEnvironmentTemperature, static_cast<int>(sizeof(env)),
                         reinterpret_cast<const uint8_t*>(&env));

      ext::SetBuildingRadiationMessage rad{};
      rad.handle = handle;
      rad.radiationFactor = 1.0f;  // a black body, so the arithmetic below is checkable
      rad.surfaceAreaM2 = 0.0f;    // use the building's own footprint
      sim_handle_message(ext::kSetBuildingRadiation, static_cast<int>(sizeof(rad)),
                         reinterpret_cast<const uint8_t*>(&rad));

      const int kFrames = 20;
      for (int i = 0; i < kFrames; ++i) u = Tick(rw, &rvis);
      const float t_after = (u && u->numBuildingTemperatures >= 1)
                                ? u->buildingTemperatures[u->numBuildingTemperatures - 1]
                                      .temperature
                                : t_before;
      double led[16] = {0};
      sim_debug_energy_ledger(led, 16);
      const double radiated = led[15] - base[15];
      printf("  %-46s radiated %+10.6f kJ, body %.4f -> %.4f K\n", c.name, radiated,
             t_before, t_after);

      // Not a threshold: sigma * A * exposure * (T^4 - Tsink^4) * dt over the run, with
      // A = 2 cells and Tsink = the environment temperature. The medium scales the FLUX --
      // it does not retarget the sink -- so a full cell is opaque and sheds exactly nothing
      // while a vacuum is transparent and sheds the lot.
      const double tb = static_cast<double>(t_before);
      const double ts = 50.0;
      const double per_second =
          5.670374419e-11 * 2.0 * (tb * tb * tb * tb - ts * ts * ts * ts) * c.expected_exposure;
      const double expected = per_second * 0.2 * kFrames;
      if (c.expected_exposure <= 0.0) {
        Check(radiated == 0.0,
              "a body sealed in a full cell radiates NOTHING, however much hotter it is than "
              "the room -- conduction already owns that exchange");
        Check(t_after >= t_before,
              "and it does not cool: nothing left the body on this path");
      } else {
        Check(radiated > 0.0, "an exposed hot body sheds heat to the environment");
        // Bounded from above by the closed form at the body's STARTING temperature, because
        // the body cools as it sheds and the flux falls across the run. Stated rather than
        // padded into a band that would hide a real discrepancy.
        Check(radiated <= expected * 1.001 && radiated > expected * 0.99,
              "what was radiated matches sigma * A * exposure * (T^4 - Tsink^4) over the run");
        Check(t_after < t_before, "the body actually cooled by what it radiated");
      }

      ext::SetBuildingRadiationMessage roff{};
      roff.handle = handle;
      roff.radiationFactor = 0.0f;
      roff.surfaceAreaM2 = 0.0f;
      sim_handle_message(ext::kSetBuildingRadiation, static_cast<int>(sizeof(roff)),
                         reinterpret_cast<const uint8_t*>(&roff));
      Tick(rw, &rvis);
      Tick(rw, &rvis);
    }
  }

  // ------------------------------------------------------------------------------------
  // ext::kSetBuildingConvection -- Stationeers-style convection between a building's body and its cells, for Mod 1's pipe radiators.
  //
  // Every expectation is the closed form 100 W/(m^2 K) * area * factor * cell ratio * dT * dt,
  // measured from the GRID's own energy rather than from either temperature, so neither heat
  // capacity has to be restated here. Klei's conduction is switched off for the probe
  // (thermalConductivity 0) so the only path between body and cell is this one.
  printf("\n=== vftest: building convection (ext::kSetBuildingConvection) ===\n");
  {
    struct ConvectionCase {
      const char* name;
      uint16_t cell_element;
      float cell_mass;
      // Expected fraction of the full-exchange flux: the cell's HeatExchangeRatio().
      // -1 means "derive it from the sim's own pressure", for the half-atmosphere case.
      double expected_ratio;
      // Each case is sized so that what moves is resolvable at BOTH ends: a float cell
      // temperature and a float body temperature. A full tile of water is ~4200 kJ/K and
      // rounds a gas-sized transfer away to nothing, so it gets a hundred times the factor
      // and a body heavy enough that its own rounding stays small against it.
      float factor;
      int frames;
      float body_mass;
      // Fill the world with granite and put the case's element in the building's own two cells
      // only. The water case needs it: an open pool evaporates at its surface, the latent heat
      // cools the tiles under it, and the widening gap made the body shed 1.8% more than the
      // closed form says. A pocket in rock neither evaporates nor flows.
      bool pocket;
      // How far the run may sit from the closed form, and it is NOT slack: both ends of the
      // exchange are floats. A light cell WARMS as it takes the heat, which narrows the gap and
      // puts the run just under the form; a heavy cell cannot resolve one write at all, so each
      // one rounds to the nearest ulp of its temperature and lands slightly more or less than
      // was proposed. A full water tile is ~4200 kJ/K and half an ulp of 300 K is 0.06 kJ
      // against a 2 kJ write, which is the whole of the 1.8% this case sits high by.
      double tol;
    };
    uint16_t bc_granite = 0, bc_water = 0;
    if (!Resolve(t, kGranite, "Granite", &bc_granite)) return 1;
    if (!Resolve(t, simhost::kWater, "Water", &bc_water)) return 1;
    const ConvectionCase cases[] = {
        {"vacuum", vacuum, 0.0f, 0.0, 0.01f, 20, 4.0f, false, 0.015},
        {"2 kg oxygen (past one atmosphere)", oxygen, 2.0f, 1.0, 0.01f, 20, 4.0f, false, 0.015},
        {"0.65 kg oxygen (about half an atmosphere)", oxygen, 0.65f, -1.0, 0.01f, 20, 4.0f, false, 0.015},
        // A FULL tile, not a puddle: 2 kg of water in a 5x3 world flows out of the building's
        // own cells within a tick and leaves it convecting with the vacuum it left behind,
        // which is what the first run of this arm measured (0.000000 kJ against a ratio of 1).
        {"water pocket", bc_water, 1000.0f, 1.0, 1.0f, 4, 400.0f, true, 0.03},
        // The one place this term and the conduit policy disagree, and both are deliberate:
        // a solid cell convects nothing here, because Klei's conduction sweep already owns a
        // building's exchange with the tiles it sits in.
        {"granite tile", bc_granite, 2000.0f, 0.0, 0.01f, 20, 4.0f, false, 0.015},
    };

    constexpr float kBodyK = 400.0f, kCellK = 300.0f;
    constexpr float kOneAtmPa = 101324.99694824219f;

    for (const ConvectionCase& c : cases) {
      World cw;
      if (c.pocket) {
        cw.Init(5, 3, bc_granite, 2000.0f, kCellK);
        // The building's own two cells, game coordinates (1,1) and (2,1).
        for (int cell : {1 * 5 + 1, 1 * 5 + 2}) {
          cw.element[cell] = c.cell_element;
          cw.mass[cell] = c.cell_mass;
        }
      } else {
        cw.Init(5, 3, c.cell_element, c.cell_mass, kCellK);
      }
      Boot(t, cw);
      std::vector<uint8_t> cvis;

      DebugProperties scales{};
      scales.buildingTemperatureScale = 0.001f;
      scales.buildingToBuildingTemperatureScale = 0.001f;
      sim_handle_message(static_cast<int32_t>(SimMessageHash::SetDebugProperties),
                         sizeof(scales), reinterpret_cast<const uint8_t*>(&scales));

      AddBuildingHeatExchangeMessage add{};
      add.callbackIdx = -1;
      add.elemIdx = oxygen;
      // A light body wherever the transfer is small: at 400 kg the body is ~400 kJ/K, and one
      // float ulp of 400 K is then 0.012 kJ -- a third of a tick's transfer in the gas cases,
      // so its own rounding would swamp the conservation check below. 4 kg puts an ulp at
      // 1e-4 kJ. The water case moves a hundred times as much and takes the heavy body.
      add.mass = c.body_mass;
      add.temperature = kBodyK;
      add.thermalConductivity = 0.0f;  // Klei's conduction off: this term is the only path
      add.overheatTemperature = 1.0e9f;
      add.operatingKilowatts = 0.0f;
      add.minX = 1;
      add.minY = 1;
      add.maxX = 3;
      add.maxY = 2;  // two cells, so the footprint area is 2 m^2
      sim_handle_message(static_cast<int32_t>(SimMessageHash::AddBuildingHeatExchange),
                         sizeof(add), reinterpret_cast<const uint8_t*>(&add));

      Tick(cw, &cvis);
      const GameDataUpdate* u = Tick(cw, &cvis);
      int32_t handle = -1;
      float t_before = 0.0f;
      if (u && u->numBuildingTemperatures >= 1) {
        handle = u->buildingTemperatures[u->numBuildingTemperatures - 1].handle;
        t_before = u->buildingTemperatures[u->numBuildingTemperatures - 1].temperature;
      }
      Check(handle >= 0, "the convection probe building has a usable sim handle");
      if (handle < 0) continue;

      // A CONTROL RUN FIRST: the same frames with the term still off. Klei's conduction is
      // switched off for this probe and nothing else touches the body, so the body must not
      // move at all -- which is what makes the body's own temperature the measurement below.
      for (int i = 0; i < c.frames; ++i) u = Tick(cw, &cvis);
      const float t_control = (u && u->numBuildingTemperatures >= 1)
                                  ? u->buildingTemperatures[u->numBuildingTemperatures - 1]
                                        .temperature
                                  : t_before;
      Check(t_control == t_before,
            "the body does not move before the message is sent: this term is the only path "
            "between it and its cells");
      t_before = t_control;

      ext::SetBuildingConvectionMessage conv{};
      conv.handle = handle;
      conv.convectionFactor = c.factor;
      conv.surfaceAreaM2 = 0.0f;  // the building's own footprint, 2 m^2
      conv.reachCells = 0;        // the footprint alone; the reach arm below is the other case
      sim_handle_message(ext::kSetBuildingConvection, static_cast<int>(sizeof(conv)),
                         reinterpret_cast<const uint8_t*>(&conv));

      // ONE TICK FOR THE QUEUE, AND THE WINDOW OPENS AFTER IT. The message is delivered
      // QUEUED (abi/sim_abi_ext.h's delivery column), so the frame it is sent in still runs
      // with the old factor. Counting that frame in the expectation is what the first version
      // of this arm did, and it read as a flat 5% shortfall -- 0.758 kJ over 20 frames where
      // 19 of them convected.
      u = Tick(cw, &cvis);
      t_before = (u && u->numBuildingTemperatures >= 1)
                     ? u->buildingTemperatures[u->numBuildingTemperatures - 1].temperature
                     : t_before;
      double base[16] = {0};
      sim_debug_energy_ledger(base, 16);

      for (int i = 0; i < c.frames; ++i) u = Tick(cw, &cvis);
      const float t_after = (u && u->numBuildingTemperatures >= 1)
                                ? u->buildingTemperatures[u->numBuildingTemperatures - 1]
                                      .temperature
                                : t_before;
      double led[16] = {0};
      sim_debug_energy_ledger(led, 16);
      // MEASURED ON THE BODY, not on the grid. A world of full water tiles moves its own
      // energy about (flow, the thin-liquid and displacement buckets) whether or not anything
      // is convecting, and reading the grid's total called that drift convected heat: 16.53 kJ
      // against a closed form of 16.00. The body has exactly one path in this probe, which is
      // the term under test.
      const double body_hc =
          static_cast<double>(c.body_mass) * static_cast<double>(t.SpecificHeat(oxygen));
      const double body_lost = (static_cast<double>(t_before) - static_cast<double>(t_after)) *
                               body_hc;
      const double ledger_drift = led[5] - base[5];

      double ratio = c.expected_ratio;
      if (ratio < 0.0) {
        const uint16_t sp[] = {oxygen};
        const float m[] = {c.cell_mass};
        ratio = std::min(
            std::max(sim_compute_gas_pressure(sp, m, 1, kCellK, 1.0f) / kOneAtmPa, 0.0f), 1.0f);
        Check(ratio > 0.45 && ratio < 0.55,
              "0.65 kg of oxygen is about half an atmosphere, so its ratio is about a half");
      }
      // 100 W/(m^2 K) is 0.1 kW/(m^2 K); 2 m^2 of area over 2 cells is 1 m^2 each, and both
      // cells are identical, so the whole footprint moves 0.1 * factor * ratio * dT kW.
      const double expected = 0.1 * static_cast<double>(c.factor) * 2.0 * ratio *
                              (static_cast<double>(kBodyK) - static_cast<double>(kCellK)) * 0.2 *
                              c.frames;
      printf("  %-42s ratio %.4f, body gave up %+9.6f kJ (closed form %+9.6f), %.4f -> "
             "%.4f K, its cells %.4f / %.4f K, ledger %+.6f kJ\n",
             c.name, ratio, body_lost, expected, t_before, t_after,
             u ? u->temperature[1 * 5 + 1] : 0.0f, u ? u->temperature[1 * 5 + 2] : 0.0f,
             ledger_drift);

      if (ratio <= 0.0) {
        Check(body_lost == 0.0 && t_after == t_before,
              "a cell with no ratio -- vacuum, or a solid tile ONI's conduction already owns --"
              " convects nothing, and the body does not move");
      } else {
        // Bounded by the closed form at the STARTING gap, which the run can only fall short
        // of: the body cools and the cell warms as the heat moves, so the gap narrows.
        Check(body_lost > expected * (1.0 - c.tol) && body_lost < expected * (1.0 + c.tol),
              "the body gave up 100 W/(m^2 K) * area * factor * ratio * dT over the run");
        Check(t_after < t_before, "and it is the body that cooled, not the room");
        // The two writes are floats, so the bucket sees their rounding and nothing else. A
        // term that created or destroyed energy outright would show up here as a multiple of
        // what moved, not a rounding of it.
        // One per cent, and it is the BODY's float ulp that sets it: the cell is written first
        // and the body is debited by what actually landed, so the only residual left is the
        // rounding of the body's own write -- 1e-4 kJ per write at 4 kJ/K, 0.012 kJ at
        // 400 kJ/K, tens of writes per run. A term that created or destroyed energy outright
        // would show a multiple of what moved here, not a rounding of it.
        Check(std::fabs(ledger_drift) < 1e-2 * body_lost,
              "the exchange conserves energy: the body lost what the cells gained, to the "
              "rounding of one float write");
      }

      // Switching it off stops it dead, which is also what a demolished radiator does.
      ext::SetBuildingConvectionMessage off{};
      off.handle = handle;
      off.convectionFactor = 0.0f;
      off.surfaceAreaM2 = 0.0f;
      sim_handle_message(ext::kSetBuildingConvection, static_cast<int>(sizeof(off)),
                         reinterpret_cast<const uint8_t*>(&off));
      // One tick for the queued clear to be drained and take effect, then two that must move
      // nothing.
      Tick(cw, &cvis);
      u = Tick(cw, &cvis);
      const float t_settled = (u && u->numBuildingTemperatures >= 1)
                                  ? u->buildingTemperatures[u->numBuildingTemperatures - 1]
                                        .temperature
                                  : 0.0f;
      Tick(cw, &cvis);
      u = Tick(cw, &cvis);
      const float t_off = (u && u->numBuildingTemperatures >= 1)
                              ? u->buildingTemperatures[u->numBuildingTemperatures - 1].temperature
                              : 0.0f;
      Check(t_off == t_settled,
            "with the factor cleared the term is inert again: the body stops moving, whatever "
            "the world around it is doing");
    }

    // REACH: the same conductance, spread over the cells around the building rather than the
    // ones under it. The invariant is that spreading changes WHERE the heat goes, not HOW MUCH:
    // with no cell anywhere near saturating, a reach of 1 must move the same total as a reach of
    // 0 and must warm cells outside the footprint that a reach of 0 leaves untouched.
    {
      const int kReachFrames = 10;
      double moved[2] = {0.0, 0.0};
      float centre[2] = {0.0f, 0.0f};
      float outside[2] = {0.0f, 0.0f};
      for (int pass = 0; pass < 2; ++pass) {
        World rw;
        rw.Init(7, 5, oxygen, 2.0f, kCellK);
        Boot(t, rw);
        std::vector<uint8_t> rvis;

        DebugProperties scales{};
        scales.buildingTemperatureScale = 0.001f;
        scales.buildingToBuildingTemperatureScale = 0.001f;
        sim_handle_message(static_cast<int32_t>(SimMessageHash::SetDebugProperties),
                           sizeof(scales), reinterpret_cast<const uint8_t*>(&scales));

        AddBuildingHeatExchangeMessage add{};
        add.callbackIdx = -1;
        add.elemIdx = oxygen;
        add.mass = 40.0f;
        add.temperature = kBodyK;
        add.thermalConductivity = 0.0f;
        add.overheatTemperature = 1.0e9f;
        add.operatingKilowatts = 0.0f;
        add.minX = 3;
        add.minY = 2;
        add.maxX = 4;
        add.maxY = 3;  // one cell, at the middle of the world
        sim_handle_message(static_cast<int32_t>(SimMessageHash::AddBuildingHeatExchange),
                           sizeof(add), reinterpret_cast<const uint8_t*>(&add));

        Tick(rw, &rvis);
        const GameDataUpdate* u = Tick(rw, &rvis);
        int32_t handle = -1;
        float t_before = 0.0f;
        if (u && u->numBuildingTemperatures >= 1) {
          handle = u->buildingTemperatures[u->numBuildingTemperatures - 1].handle;
          t_before = u->buildingTemperatures[u->numBuildingTemperatures - 1].temperature;
        }
        if (handle < 0) continue;

        ext::SetBuildingConvectionMessage conv{};
        conv.handle = handle;
        conv.convectionFactor = 0.01f;
        conv.surfaceAreaM2 = 1.0f;
        conv.reachCells = pass;  // 0 then 1
        sim_handle_message(ext::kSetBuildingConvection, static_cast<int>(sizeof(conv)),
                           reinterpret_cast<const uint8_t*>(&conv));
        u = Tick(rw, &rvis);  // one tick for the queue, as above
        t_before = (u && u->numBuildingTemperatures >= 1)
                       ? u->buildingTemperatures[u->numBuildingTemperatures - 1].temperature
                       : t_before;
        for (int i = 0; i < kReachFrames; ++i) u = Tick(rw, &rvis);
        const float t_after = (u && u->numBuildingTemperatures >= 1)
                                  ? u->buildingTemperatures[u->numBuildingTemperatures - 1]
                                        .temperature
                                  : t_before;
        moved[pass] = (static_cast<double>(t_before) - static_cast<double>(t_after)) *
                      static_cast<double>(add.mass) * static_cast<double>(t.SpecificHeat(oxygen));
        // The building's own cell, and the one to the left of it.
        centre[pass] = u ? u->temperature[2 * 7 + 3] : 0.0f;
        outside[pass] = u ? u->temperature[2 * 7 + 2] : 0.0f;
      }
      printf("  reach 0: moved %+9.6f kJ, its own cell %.4f K, the next one %.4f K (gap "
             "%.4f)\n  reach 1: moved %+9.6f kJ, its own cell %.4f K, the next one %.4f K (gap "
             "%.4f)\n",
             moved[0], centre[0], outside[0], centre[0] - outside[0], moved[1], centre[1],
             outside[1], centre[1] - outside[1]);
      Check(moved[0] > 0.0 && moved[1] > 0.0, "the body sheds in both passes");
      // NOT an equality: spreading the same conductance over more cells means each of them warms
      // less, which keeps the gradient wider and moves MORE, not the same. The first version of
      // this arm asserted equality and measured 12% -- that 12% is the effect, at a scale where
      // nothing is close to saturation. On the RADIATOR rig, where the one cell does saturate,
      // the same mechanism is the whole difference between a radiator and a pipe.
      Check(moved[1] > moved[0] * 1.05,
            "a reach sheds MORE than the footprint alone, and by more than a rounding: nine cells "
            "sharing the same conductance each warm about a ninth as much, so the gradient the "
            "term works against stays wider");

      // WHAT THIS ARM DELIBERATELY DOES NOT ASSERT, and why it is the rig's job instead. The
      // point of a reach is the temperature PROFILE -- heat into the room rather than into one
      // tile -- and at this scale that is unmeasurable here: seven by five cells of loose gas
      // mix by flow and conduct between themselves far faster than 0.2 kJ of input can separate
      // them, and the first version of this arm duly measured the building's own cell and its
      // neighbour at the same 300.0398 K with no reach at all. What a reach changes shows at a
      // larger scale: a kilowatt radiator, where one cell pins within a tick and the room is
      // what limits everything after that.
    }

    // The equilibrium limit, which is what keeps this from being an oscillator: a factor big
    // enough to propose far more than the gap holds must still stop at the shared temperature.
    {
      World cw;
      cw.Init(5, 3, oxygen, 2.0f, kCellK);
      Boot(t, cw);
      std::vector<uint8_t> cvis;

      DebugProperties scales{};
      scales.buildingTemperatureScale = 0.001f;
      scales.buildingToBuildingTemperatureScale = 0.001f;
      sim_handle_message(static_cast<int32_t>(SimMessageHash::SetDebugProperties),
                         sizeof(scales), reinterpret_cast<const uint8_t*>(&scales));

      AddBuildingHeatExchangeMessage add{};
      add.callbackIdx = -1;
      add.elemIdx = oxygen;
      add.mass = 4.0f;  // a light body, so the shared temperature is well clear of both starts
      add.temperature = kBodyK;
      add.thermalConductivity = 0.0f;
      add.overheatTemperature = 1.0e9f;
      add.operatingKilowatts = 0.0f;
      add.minX = 1;
      add.minY = 1;
      add.maxX = 3;
      add.maxY = 2;
      sim_handle_message(static_cast<int32_t>(SimMessageHash::AddBuildingHeatExchange),
                         sizeof(add), reinterpret_cast<const uint8_t*>(&add));

      Tick(cw, &cvis);
      const GameDataUpdate* u = Tick(cw, &cvis);
      int32_t handle = -1;
      float t_before = 0.0f;
      if (u && u->numBuildingTemperatures >= 1) {
        handle = u->buildingTemperatures[u->numBuildingTemperatures - 1].handle;
        t_before = u->buildingTemperatures[u->numBuildingTemperatures - 1].temperature;
      }
      if (handle >= 0) {
        ext::SetBuildingConvectionMessage conv{};
        conv.handle = handle;
        conv.convectionFactor = 1.0e6f;  // absurd on purpose
        conv.surfaceAreaM2 = 0.0f;
        sim_handle_message(ext::kSetBuildingConvection, static_cast<int>(sizeof(conv)),
                           reinterpret_cast<const uint8_t*>(&conv));
        for (int i = 0; i < 4; ++i) u = Tick(cw, &cvis);
        const float t_after = (u && u->numBuildingTemperatures >= 1)
                                  ? u->buildingTemperatures[u->numBuildingTemperatures - 1]
                                        .temperature
                                  : t_before;
        const float cell_after = u ? u->temperature[1 * 5 + 1] : 0.0f;
        printf("  runaway factor: body %.4f -> %.4f K, its first cell %.4f K\n", t_before,
               t_after, cell_after);
        Check(t_after < t_before && t_after > kCellK - 0.5f,
              "a proposal far larger than the gap stops at the pair's equilibrium instead of "
              "overshooting past the cell");
        Check(cell_after > kCellK && cell_after <= kBodyK,
              "and the cell it warmed stayed between the two starting temperatures");
      }
    }
  }

  // ================================================================================
  // Extension registry (abi/sim_abi_ext.h, sim/ext_registry.h). Everything below drives the DLL's own exported ABI only --
  // no internal access -- which is the same thing a mod would have.
  //
  // These arms exist because the registry's whole value is in what it REFUSES. A registry
  // that stores values correctly and accepts a second `mod2.contamination` from a different
  // owner, or a name that differs only in case, or a registration that arrives after the
  // world is already sized, has silently produced the class of bug it was written to
  // prevent. So most of what follows asserts a rejection and its exact reason code.
  printf("\n=== vftest: extension registry -- registration ===\n");
  sim_shutdown();
  InitSim();

  // The first-party property is registered by World's own constructor, before anything can
  // send a message, and it is index 0 in every process so a mod's registration order cannot
  // shift it.
  const int32_t tmb = sim_ext_property_index("sim.thermal_mass_bonus");
  printf("  sim.thermal_mass_bonus index=%d\n", tmb);
  Check(tmb == 0, "the first-party thermal-mass bonus is registered before any message, at index 0");
  Check(sim_ext_property_index("mod2.never_registered") == -1,
        "an unregistered name looks up as -1, not as some other property");

  const int32_t contamination =
      RegisterProperty("mod2.contamination", ext::kExtF32, ext::kRehydrated, 0u);
  printf("  mod2.contamination index=%d\n", contamination);
  Check(contamination > 0, "a well-formed third-party registration is accepted");
  Check(sim_ext_property_index("mod2.contamination") == contamination,
        "and is findable by name at the index it returned");

  // A default that is not zero, to prove Allocate's per-cell fill and not just its assign(0).
  const int32_t flag = RegisterProperty("mod3.flag", ext::kExtU8, ext::kRehydrated, 7u);
  Check(flag > 0, "a u8 property with a non-zero default is accepted");

  // ---- the refusals. Each one is a distinct code, because "registration failed" is not
  // actionable and "that name is already taken by mod2" is.
  Check(RegisterProperty("mod2.contamination", ext::kExtF32, ext::kRehydrated, 0u) ==
            -static_cast<int32_t>(ext::kExtRegisterDuplicate),
        "a duplicate name is refused as a DUPLICATE (the log names both owners)");
  Check(RegisterProperty("sim.sneaky", ext::kExtF32, ext::kRehydrated, 0u) ==
            -static_cast<int32_t>(ext::kExtRegisterReserved),
        "the reserved sim. owner is refused to a third-party registration");
  Check(RegisterProperty("oni.sneaky", ext::kExtF32, ext::kRehydrated, 0u) ==
            -static_cast<int32_t>(ext::kExtRegisterReserved),
        "and so is oni.");
  Check(RegisterProperty("Mod2.Contamination", ext::kExtF32, ext::kRehydrated, 0u) ==
            -static_cast<int32_t>(ext::kExtRegisterBadName),
        "UPPERCASE IS REFUSED, NOT DOWN-CASED -- normalising would map two names onto one store");
  Check(sim_ext_property_index("Mod2.Contamination") == -1,
        "and the rejected name did not quietly become the existing lowercase one");
  Check(RegisterProperty("contamination", ext::kExtF32, ext::kRehydrated, 0u) ==
            -static_cast<int32_t>(ext::kExtRegisterBadName),
        "a name with no owner segment is refused");
  Check(RegisterProperty("mod2.sub.prop", ext::kExtF32, ext::kRehydrated, 0u) ==
            -static_cast<int32_t>(ext::kExtRegisterBadName),
        "a name with two separators is refused");
  Check(RegisterProperty("m.x", ext::kExtF32, ext::kRehydrated, 0u) ==
            -static_cast<int32_t>(ext::kExtRegisterBadName),
        "a one-character owner is refused (owner is 2-31 chars)");
  Check(RegisterProperty("mod2.has space", ext::kExtF32, ext::kRehydrated, 0u) ==
            -static_cast<int32_t>(ext::kExtRegisterBadName),
        "a name outside [a-z0-9_] is refused");
  // 48 bytes of name with no room for a terminator. The sim refuses it rather than
  // truncating to 47, because the name is the permanent on-disk key and a truncated key
  // is a different property than the caller asked for.
  {
    char full[64];
    memset(full, 'a', sizeof(full));
    full[4] = '.';
    full[48] = '\0';  // 48 chars, so the 48-byte wire field holds no terminator
    Check(RegisterProperty(full, ext::kExtF32, ext::kRehydrated, 0u) ==
              -static_cast<int32_t>(ext::kExtRegisterBadName),
          "a name that fills the whole 48-byte field is refused, not silently truncated");
  }
  Check(RegisterProperty("mod2.badtype", 99, ext::kRehydrated, 0u) ==
            -static_cast<int32_t>(ext::kExtRegisterBadType),
        "a scalar type outside the enum is refused");
  Check(RegisterProperty("mod2.badpersist", ext::kExtF32, 99, 0u) ==
            -static_cast<int32_t>(ext::kExtRegisterBadType),
        "a persistence outside the enum is refused -- there is no default to fall back to");
  // Through stage 1 this asserted that kSaved was REFUSED with kExtRegisterUnsupported, and
  // that refusal was the point: accepting it with no serialiser behind it would have handed a
  // caller a property that silently behaved like kRehydrated. Stage 2 built the serialiser, so
  // the assertion is inverted rather than deleted -- and the acceptance is only meaningful
  // because the round-trip arms further down actually put bytes through a blob.
  Check(RegisterProperty("mod2.saved", ext::kExtF32, ext::kSaved, 0u) > 0,
        "kSaved registers now that stage 2's self-describing save section exists");

  // ---- storage: the values, the defaults, and the stride.
  printf("\n=== vftest: extension registry -- storage ===\n");
  World ext_world;
  ext_world.Init(4, 1, oxygen, 1.0f, 300.0f);
  Boot(t, ext_world);
  std::vector<uint8_t> ext_vis;

  uint32_t bits = 0;
  Check(sim_ext_read_cell_property(0, flag, 0, &bits) == 1 && bits == 7u,
        "an untouched cell reads back the property's registered default, not zero");
  Check(sim_ext_read_cell_property(0, contamination, 0, &bits) == 1 && bits == 0u,
        "and a zero-default property reads back zero");

  SetCellProperty(2, contamination, BitsOf(1234.5f));
  SetCellProperty(2, flag, 0x1234ABCDu);  // truncates to the low byte for a u8
  Tick(ext_world, &ext_vis);  // a queued message lands on the frame AFTER it is sent

  float read_back = 0.0f;
  Check(sim_ext_read_cell_property(2, contamination, 0, &bits) == 1, "the f32 write is readable");
  memcpy(&read_back, &bits, sizeof(read_back));
  printf("  cell 2 mod2.contamination = %.4f\n", read_back);
  Check(read_back == 1234.5f, "an f32 property round-trips through the generic setter exactly");
  Check(sim_ext_read_cell_property(2, flag, 0, &bits) == 1 && bits == 0xCDu,
        "a u8 property keeps only its low byte -- one stride per cell, not four");
  Check(sim_ext_read_cell_property(1, contamination, 0, &bits) == 1 && bits == 0u,
        "and the neighbouring cell was not touched: the stride indexes cells, not bytes");

  Check(sim_ext_read_cell_property(0, 999, 0, &bits) == 0,
        "reading an unregistered property index fails rather than returning garbage");
  Check(sim_ext_read_cell_property(9999, contamination, 0, &bits) == 0,
        "and so does reading a cell outside the world");

  // ---- registration closes at the first allocate. This is the invariant the whole of
  // world.h leans on: every per-cell array is the same length for the life of a world.
  Check(RegisterProperty("mod4.late", ext::kExtF32, ext::kRehydrated, 0u) ==
            -static_cast<int32_t>(ext::kExtRegisterClosed),
        "a registration arriving after the world is allocated is REFUSED, not serviced by a "
        "resize");
  Check(sim_ext_property_index("mod4.late") == -1, "and the late name really is not registered");

  // ---- the kRehydrated claim, made checkable. `mod3.flag` has been written; `mod2` has
  // too. A third kRehydrated property that nobody has written is what the list is for.
  printf("\n=== vftest: extension registry -- outstanding rehydration ===\n");
  sim_shutdown();
  InitSim();
  const int32_t promised = RegisterProperty("mod5.promised", ext::kExtF32, ext::kRehydrated, 0u);
  const int32_t pushed = RegisterProperty("mod5.pushed", ext::kExtF32, ext::kRehydrated, 0u);
  Check(promised > 0 && pushed > 0, "two kRehydrated properties registered");
  World reh_world;
  reh_world.Init(4, 1, oxygen, 1.0f, 300.0f);
  Boot(t, reh_world);
  std::vector<uint8_t> reh_vis;

  int32_t outstanding[8] = {};
  int32_t n_out = sim_ext_outstanding_rehydration(outstanding, 8);
  printf("  straight after the allocate: %d outstanding\n", n_out);
  Check(n_out == 2,
        "straight after an allocate, EVERY kRehydrated property is outstanding -- nobody has "
        "re-pushed anything yet");

  SetCellProperty(0, pushed, BitsOf(1.0f));
  Tick(reh_world, &reh_vis);
  n_out = sim_ext_outstanding_rehydration(outstanding, 8);
  printf("  after one write to mod5.pushed: %d outstanding\n", n_out);
  Check(n_out == 1 && outstanding[0] == promised,
        "a property its owner has re-pushed drops off the list, and the one nobody pushed is "
        "still named -- which is the whole difference between a kRehydrated claim and a "
        "comment saying the same thing");
  // The first-party property is kCheckpointOnly, not kRehydrated: nothing re-pushes it, the
  // checkpoint carries it. It must never appear on this list however long it goes unwritten.
  Check(n_out == 1,
        "sim.thermal_mass_bonus is kCheckpointOnly and never appears as outstanding, however "
        "long nothing writes it");

  // ---- the migration itself: the old bespoke id still lands in the registered property.
  printf("\n=== vftest: extension registry -- kSetCellThermalMassBonus compatibility ===\n");
  Writer tb;
  tb.Put<int32_t>(1);
  tb.Put<float>(50000.0f);
  sim_handle_message(ext::kSetCellThermalMassBonus, static_cast<int>(tb.bytes.size()),
                     tb.bytes.data());
  Tick(reh_world, &reh_vis);
  Check(sim_ext_read_cell_property(1, tmb, 0, &bits) == 1, "the aliased write is readable");
  memcpy(&read_back, &bits, sizeof(read_back));
  printf("  cell 1 sim.thermal_mass_bonus = %.1f J/K\n", read_back);
  Check(read_back == 50000.0f,
        "the original kSetCellThermalMassBonus id still lands, byte for byte, in the "
        "registered property that replaced its member vector");
  Check(sim_ext_read_cell_property(0, tmb, 0, &bits) == 1 && bits == 0u,
        "and only in the cell it addressed");

  // ---- ARITY. Stage 2 added components-per-cell because the gas-mixture triple the save
  // format has to absorb is not one scalar per cell: gas_occupied_mask_ is one uint8, but
  // gas_species_ is eight uint16 and gas_mass_ eight float. Everything below is about the
  // layout being CELL-MAJOR -- component c of cell p at (p * arity + c) -- because that is
  // byte-for-byte what gas_species_ already does, and a relayout would be a second variable
  // in a migration that has enough.
  printf("\n=== vftest: extension registry -- arity ===\n");
  sim_shutdown();
  InitSim();
  const int32_t lanes = RegisterProperty("mod6.lanes", ext::kExtU16, ext::kRehydrated, 0u,
                                         /*arity=*/8);
  const int32_t scalar = RegisterProperty("mod6.scalar", ext::kExtF32, ext::kRehydrated, 0u);
  Check(lanes > 0 && scalar > 0, "an arity-8 u16 property and an arity-1 f32 both register");

  Check(RegisterProperty("mod6.zero", ext::kExtF32, ext::kRehydrated, 0u, /*arity=*/0) ==
            -static_cast<int32_t>(ext::kExtRegisterBadArity),
        "arity 0 is refused -- a property with no components is not a property");
  Check(RegisterProperty("mod6.negative", ext::kExtF32, ext::kRehydrated, 0u, /*arity=*/-1) ==
            -static_cast<int32_t>(ext::kExtRegisterBadArity),
        "and so is a negative arity, which is what a garbled field looks like");
  Check(RegisterProperty("mod6.huge", ext::kExtF32, ext::kRehydrated, 0u,
                         ext::kExtMaxArity + 1) ==
            -static_cast<int32_t>(ext::kExtRegisterBadArity),
        "and so is one above kExtMaxArity, before it reaches World::Allocate as a size");
  Check(RegisterProperty("mod6.atcap", ext::kExtF32, ext::kRehydrated, 0u,
                         ext::kExtMaxArity) > 0,
        "kExtMaxArity itself is accepted -- the cap is inclusive");

  World ar_world;
  ar_world.Init(4, 1, oxygen, 1.0f, 300.0f);
  Boot(t, ar_world);
  std::vector<uint8_t> ar_vis;

  // Write a distinct value into every lane of one cell, and nothing into its neighbours.
  for (int32_t c = 0; c < 8; ++c) {
    SetCellProperty(2, lanes, static_cast<uint32_t>(0x1000 + c), c);
  }
  Tick(ar_world, &ar_vis);

  bool all_lanes = true;
  for (int32_t c = 0; c < 8; ++c) {
    uint32_t lane_bits = 0;
    if (sim_ext_read_cell_property(2, lanes, c, &lane_bits) != 1 ||
        lane_bits != static_cast<uint32_t>(0x1000 + c)) {
      printf("  lane %d read back 0x%X, expected 0x%X\n", c, lane_bits, 0x1000 + c);
      all_lanes = false;
    }
  }
  Check(all_lanes, "all eight lanes of one cell hold their own value and do not overwrite "
                   "each other");

  // The lane/cell aliasing check that actually matters. Cell 2 lane 0 and cell 1 lane 4 are
  // NOT the same slot, and an off-by-one in the cell-major index is exactly what would make
  // them one. Cell 1 was never written, so every lane of it must still read zero.
  bool neighbour_clean = true;
  for (int32_t c = 0; c < 8; ++c) {
    uint32_t lane_bits = 0xFFFF;
    if (sim_ext_read_cell_property(1, lanes, c, &lane_bits) != 1 || lane_bits != 0u) {
      printf("  neighbour cell 1 lane %d = 0x%X, expected 0\n", c, lane_bits);
      neighbour_clean = false;
    }
  }
  Check(neighbour_clean,
        "writing every lane of cell 2 leaves cell 1 untouched -- the cell-major stride is "
        "arity lanes wide, not one");

  uint32_t lane_bits = 0;
  Check(sim_ext_read_cell_property(2, lanes, 8, &lane_bits) == 0,
        "reading component 8 of an arity-8 property is refused, not wrapped to lane 0");
  Check(sim_ext_read_cell_property(2, lanes, -1, &lane_bits) == 0,
        "and so is a negative component");
  Check(sim_ext_read_cell_property(2, scalar, 1, &lane_bits) == 0,
        "component 1 of an arity-1 property is refused too -- a scalar has exactly one lane");

  // A write to an out-of-range component must be DROPPED, not folded onto a valid lane. This
  // is the one that would silently corrupt a real gas cell.
  SetCellProperty(2, lanes, 0xBEEFu, 8);
  SetCellProperty(2, lanes, 0xBEEFu, -1);
  Tick(ar_world, &ar_vis);
  bool still_intact = true;
  for (int32_t c = 0; c < 8; ++c) {
    uint32_t after = 0;
    sim_ext_read_cell_property(2, lanes, c, &after);
    if (after != static_cast<uint32_t>(0x1000 + c)) still_intact = false;
  }
  Check(still_intact,
        "a write to an out-of-range component is dropped and lands on NO lane -- not folded "
        "onto lane 0, which is the failure that would quietly corrupt a gas cell");

  // A nonzero default is a PER-COMPONENT default: an arity-8 property starts with eight
  // filled lanes, not one filled and seven zero.
  sim_shutdown();
  InitSim();
  const int32_t defaulted = RegisterProperty("mod6.defaulted", ext::kExtU16, ext::kRehydrated,
                                             0x2A2Au, /*arity=*/4);
  Check(defaulted > 0, "an arity-4 property with a nonzero default registers");
  World def_world;
  def_world.Init(3, 1, oxygen, 1.0f, 300.0f);
  Boot(t, def_world);
  bool all_defaulted = true;
  for (int32_t cell = 0; cell < 3; ++cell) {
    for (int32_t c = 0; c < 4; ++c) {
      uint32_t d = 0;
      if (sim_ext_read_cell_property(cell, defaulted, c, &d) != 1 || d != 0x2A2Au) {
        printf("  cell %d lane %d = 0x%X, expected 0x2A2A\n", cell, c, d);
        all_defaulted = false;
      }
    }
  }
  Check(all_defaulted,
        "a nonzero default fills EVERY component of every cell, not just lane 0 -- the "
        "default is a per-component value");

  // ---- kSaved and the self-describing v18 section. Stage 1 refused kSaved outright rather
  // than accepting it and behaving like kRehydrated; this is what earns the acceptance.
  printf("\n=== vftest: extension registry -- kSaved and the v18 section ===\n");
  sim_shutdown();
  InitSim();
  const int32_t saved_f = RegisterProperty("mod7.depth", ext::kExtF32, ext::kSaved, 0u);
  const int32_t saved_l = RegisterProperty("mod7.lanes", ext::kExtU16, ext::kSaved, 0u,
                                           /*arity=*/4);
  Check(saved_f > 0 && saved_l > 0, "two kSaved properties register");
  World sv_world;
  sv_world.Init(4, 1, oxygen, 1.0f, 300.0f);
  Boot(t, sv_world);
  std::vector<uint8_t> sv_vis;

  // ABSENT, NOT EMPTY. Registering kSaved properties and never writing one must not change
  // what this world saves -- it still has nothing to say. This is the guarantee Klei's own
  // decoder depends on (diffsim asserts `klei loads mine: accepted`), and a v18 blob with a
  // zero count would break it for every world in the game.
  std::vector<uint8_t> quiet_blob = Save(0, 0);
  int32_t quiet_version = -1;
  if (quiet_blob.size() > 12) memcpy(&quiet_version, quiet_blob.data() + 8, 4);
  printf("  registered-but-unwritten blob: %zu bytes, version=%d\n", quiet_blob.size(),
         quiet_version);
  Check(quiet_version == 15,
        "a world that registered kSaved properties and wrote none still saves at the LEGACY "
        "version -- the section is absent, not empty");

  SetCellProperty(1, saved_f, BitsOf(42.5f));
  for (int32_t c = 0; c < 4; ++c) {
    SetCellProperty(2, saved_l, static_cast<uint32_t>(0x300 + c), c);
  }
  Tick(sv_world, &sv_vis);

  const std::vector<uint8_t> sv_blob = Save(0, 0);
  int32_t sv_version = -1;
  if (sv_blob.size() > 12) memcpy(&sv_version, sv_blob.data() + 8, 4);
  printf("  written blob: %zu bytes, version=%d\n", sv_blob.size(), sv_version);
  Check(sv_version == 19,
        "one written kSaved cell promotes the blob to the extension section, palette and all "
        "(kSaveVersionElementPalette, 19)");
  Check(sv_blob.size() > quiet_blob.size(),
        "and the blob actually grew by a section rather than only changing its version word");

  // Round-trip through a FRESH sim instance, which is the only way to prove the bytes came out
  // of the blob rather than out of memory that survived. The properties have to be registered
  // again first, exactly as a mod would on load -- and the indices must come back the same,
  // because registration order is what the section's order is.
  sim_shutdown();
  InitSim();
  const int32_t rt_f = RegisterProperty("mod7.depth", ext::kExtF32, ext::kSaved, 0u);
  const int32_t rt_l = RegisterProperty("mod7.lanes", ext::kExtU16, ext::kSaved, 0u,
                                        /*arity=*/4);
  Check(rt_f == saved_f && rt_l == saved_l,
        "the same registrations in the same order yield the same indices");
  SendRaw(SimMessageHash::Elements_CreateTable, t.elements);
  SendRaw(SimMessageHash::Disease_CreateTable, t.diseases);
  sim_handle_message(static_cast<int32_t>(SimMessageHash::Load),
                     static_cast<int>(sv_blob.size()), sv_blob.data());

  uint32_t rt_bits = 0;
  float rt_f32 = 0.0f;
  Check(sim_ext_read_cell_property(1, rt_f, 0, &rt_bits) == 1, "the f32 property reads back");
  memcpy(&rt_f32, &rt_bits, sizeof(rt_f32));
  printf("  after load: mod7.depth[1] = %.1f\n", rt_f32);
  Check(rt_f32 == 42.5f, "a kSaved f32 survives a save/load across a fresh sim instance");

  bool rt_lanes = true;
  for (int32_t c = 0; c < 4; ++c) {
    uint32_t lb = 0;
    if (sim_ext_read_cell_property(2, rt_l, c, &lb) != 1 ||
        lb != static_cast<uint32_t>(0x300 + c)) {
      printf("  lane %d came back 0x%X, expected 0x%X\n", c, lb, 0x300 + c);
      rt_lanes = false;
    }
  }
  Check(rt_lanes,
        "and so does every lane of a kSaved arity-4 property, in the right lane -- the "
        "cell-major layout survives the blob");
  uint32_t untouched = 0xFFFF;
  Check(sim_ext_read_cell_property(0, rt_f, 0, &untouched) == 1 && untouched == 0u,
        "a cell nobody wrote comes back at its default, not at a neighbour's value");

  // A world that registers the property with a DIFFERENT SHAPE than the blob carries must be
  // refused and told which property, not handed reinterpreted bytes. This is the third
  // mismatch case in EXT-REGISTRY.md 3.2, and it is the one that silently produces garbage if
  // it is got wrong.
  sim_shutdown();
  InitSim();
  const int32_t wrong = RegisterProperty("mod7.depth", ext::kExtU16, ext::kSaved, 0u);
  Check(wrong > 0, "the same name registers with a different type in a fresh instance");
  RegisterProperty("mod7.lanes", ext::kExtU16, ext::kSaved, 0u, /*arity=*/4);
  SendRaw(SimMessageHash::Elements_CreateTable, t.elements);
  SendRaw(SimMessageHash::Disease_CreateTable, t.diseases);
  sim_handle_message(static_cast<int32_t>(SimMessageHash::Load),
                     static_cast<int>(sv_blob.size()), sv_blob.data());
  uint32_t after_refusal = 0xABCD;
  const int32_t refused_read = sim_ext_read_cell_property(1, wrong, 0, &after_refusal);
  printf("  after a shape-mismatch load: read=%d bits=0x%X\n", refused_read, after_refusal);
  Check(refused_read != 1 || after_refusal == 0u,
        "a property whose type changed since the save is REFUSED, not reinterpreted -- the "
        "cell holds its default, never the old bytes read as the new type");

  // A blob carrying a property nobody registered must still LOAD. A player who uninstalls one
  // mod must not find every save unopenable.
  sim_shutdown();
  InitSim();
  SendRaw(SimMessageHash::Elements_CreateTable, t.elements);
  SendRaw(SimMessageHash::Disease_CreateTable, t.diseases);
  sim_handle_message(static_cast<int32_t>(SimMessageHash::Load),
                     static_cast<int>(sv_blob.size()), sv_blob.data());
  Check(sim_ext_property_index("mod7.depth") == -1,
        "with no mod registering it, the name really is unknown to this instance");
  Check(sim_debug_gas_mass(0, oxygen) >= 0.0f,
        "and the blob still LOADED -- uninstalling a mod must not make a save unopenable");

  // ---- STAGE 2.4: the orphan really SURVIVES, not merely fails to crash.
  //
  // This is the case of a player toggling a mod off for one session. Without the side list,
  // that session's first save destroys the mod's data permanently and silently. Nothing fails
  // without this until it has already cost somebody a colony, which is why the assertion is
  // about the bytes coming back rather than about the load returning.
  printf("\n=== vftest: extension registry -- orphaned properties survive ===\n");
  const std::vector<uint8_t> orphan_blob = Save(0, 0);
  int32_t orphan_version = -1;
  if (orphan_blob.size() > 12) memcpy(&orphan_version, orphan_blob.data() + 8, 4);
  printf("  re-saved with the mod absent: %zu bytes, version=%d (original %zu)\n",
         orphan_blob.size(), orphan_version, sv_blob.size());
  Check(orphan_version == 19,
        "a world carrying orphaned properties still saves at v19 -- an orphan forces the "
        "section to exist even when this build has nothing of its own to put in it");
  // NOT the same size as the blob it came from, and that is correct rather than a leak. The
  // original carried all six kSaved records -- the two the mod owned plus the four first-party
  // ones, which go in whole once the section exists even when they are all-default. With the
  // mod gone, nothing this build registered has anything to say, so only the orphans force the
  // section. Absent-not-empty still applies to the registered half.
  Check(orphan_blob.size() < sv_blob.size(),
        "the re-saved blob is SMALLER: the four all-default first-party records are correctly "
        "omitted, while the orphans are not");

  // The property that actually matters over time: saving again with the mod still absent must
  // produce the same bytes. An orphan that degraded a little on each cycle would look fine in
  // any single round-trip test and still lose the data over a few sessions.
  const std::vector<uint8_t> orphan_again = Save(0, 0);
  Check(orphan_again.size() == orphan_blob.size() &&
            memcmp(orphan_again.data(), orphan_blob.data(), orphan_blob.size()) == 0,
        "and saving AGAIN with the mod still absent is byte-identical -- carried data does not "
        "decay across repeated save cycles, which no single round trip would have caught");

  // Now put the mod back and confirm the values are the ones it saved, not defaults.
  sim_shutdown();
  InitSim();
  const int32_t back_f = RegisterProperty("mod7.depth", ext::kExtF32, ext::kSaved, 0u);
  const int32_t back_l = RegisterProperty("mod7.lanes", ext::kExtU16, ext::kSaved, 0u,
                                          /*arity=*/4);
  SendRaw(SimMessageHash::Elements_CreateTable, t.elements);
  SendRaw(SimMessageHash::Disease_CreateTable, t.diseases);
  sim_handle_message(static_cast<int32_t>(SimMessageHash::Load),
                     static_cast<int>(orphan_blob.size()), orphan_blob.data());
  uint32_t back_bits = 0;
  float back_f32 = 0.0f;
  Check(sim_ext_read_cell_property(1, back_f, 0, &back_bits) == 1, "the property reads back");
  memcpy(&back_f32, &back_bits, sizeof(back_f32));
  printf("  mod reinstalled: mod7.depth[1] = %.1f (saved 42.5)\n", back_f32);
  Check(back_f32 == 42.5f,
        "REINSTALLING the mod recovers the exact value it saved, across a session in which "
        "the mod was absent and the world was saved again -- which is the entire point of the "
        "opaque side list");
  bool back_lanes = true;
  for (int32_t c = 0; c < 4; ++c) {
    uint32_t lb = 0;
    if (sim_ext_read_cell_property(2, back_l, c, &lb) != 1 ||
        lb != static_cast<uint32_t>(0x300 + c)) {
      back_lanes = false;
    }
  }
  Check(back_lanes, "and every lane of the arity-4 property came back in its own lane");

  // ---- THE FORMAT MIGRATION'S ACCEPTANCE TEST: a v16 and
  // a v17 blob, written by the build before this change, loading through the new reader with
  // identical cell contents.
  //
  // These come from tests/fixtures/ rather than from a world this test builds, because nothing
  // in this tree can write either version any more -- ToBlob emits kSaveVersionExtensions now.
  // A test that regenerated its own input would be checking the new writer against the new
  // reader and would pass whatever the migration did to the old format. That is the entire
  // reason the fixtures were captured before the writer changed.
  printf("\n=== vftest: legacy v15/v16/v17 blobs still load ===\n");

  const std::vector<uint8_t> fx16 = ReadFixture("gas-v16.blob");
  int32_t fx16_version = -1;
  if (fx16.size() > 12) memcpy(&fx16_version, fx16.data() + 8, 4);
  Check(fx16_version == 16, "the pinned gas fixture really is a v16 blob");

  sim_shutdown();
  InitSim();
  SendRaw(SimMessageHash::Elements_CreateTable, t.elements);
  SendRaw(SimMessageHash::Disease_CreateTable, t.diseases);
  sim_handle_message(static_cast<int32_t>(SimMessageHash::Load),
                     static_cast<int>(fx16.size()), fx16.data());
  double fx16_total = 0.0;
  for (int c = 0; c < 5; ++c) fx16_total += sim_debug_gas_mass(c, oxygen);
  printf("  v16 fixture restored %.4f kg O2 across 5 cells\n", fx16_total);
  Check(std::fabs(fx16_total - 10.0) < 1e-2,
        "a v16 blob written before the migration still restores its full gas mixture -- the "
        "fixed section's bytes reach the registered property that replaced it");
  // ...and re-saving it now writes the NEW format, which is the migration completing rather
  // than merely tolerating the old one.
  const std::vector<uint8_t> resaved = Save(0, 0);
  int32_t resaved_version = -1;
  if (resaved.size() > 12) memcpy(&resaved_version, resaved.data() + 8, 4);
  printf("  re-saved as version %d (%zu bytes, was %zu)\n", resaved_version, resaved.size(),
         fx16.size());
  Check(resaved_version == 19,
        "and re-saving that world writes kSaveVersionElementPalette -- a legacy blob is UPGRADED "
        "on its next save, not carried forward at its old version forever");

  // Round-trip the upgraded blob to prove the contents actually survived both formats, not
  // just the first read.
  sim_shutdown();
  InitSim();
  SendRaw(SimMessageHash::Elements_CreateTable, t.elements);
  SendRaw(SimMessageHash::Disease_CreateTable, t.diseases);
  sim_handle_message(static_cast<int32_t>(SimMessageHash::Load),
                     static_cast<int>(resaved.size()), resaved.data());
  double upgraded_total = 0.0;
  for (int c = 0; c < 5; ++c) upgraded_total += sim_debug_gas_mass(c, oxygen);
  printf("  v16 -> v19 -> load restored %.4f kg O2\n", upgraded_total);
  Check(std::fabs(upgraded_total - fx16_total) < 1e-4,
        "v16 -> v19 -> load preserves the mixture to the digit: the round trip THROUGH the new "
        "format loses nothing the old one carried");

  const std::vector<uint8_t> fx17 = ReadFixture("promoted-room-v17.blob");
  int32_t fx17_version = -1;
  if (fx17.size() > 12) memcpy(&fx17_version, fx17.data() + 8, 4);
  Check(fx17_version == 17, "the pinned promotion fixture really is a v17 blob");

  sim_shutdown();
  InitSim();
  SendRaw(SimMessageHash::Elements_CreateTable, t.elements);
  SendRaw(SimMessageHash::Disease_CreateTable, t.diseases);
  Check(sim_debug_room_owned(0) == -1, "sanity: the fresh instance has no room at all yet");
  sim_handle_message(static_cast<int32_t>(SimMessageHash::Load),
                     static_cast<int>(fx17.size()), fx17.data());
  Check(sim_debug_room_owned(0) == 1,
        "a v17 blob's room promotion survives the migration -- the byte that had its own save "
        "version is a registered kSaved property now, and reads back the same");
  Check(sim_debug_room_owned(4) == 1,
        "and still per-ROOM rather than per-cell: cell 4 in the same room reports owned too");

  // The v15 fixture is the one that guards the guarantee rather than the migration: a world
  // with nothing to say must still write exactly what Klei writes. Comparing against a pinned
  // file is stronger than re-deriving the expected bytes from the code that produced them.
  const std::vector<uint8_t> fx15 = ReadFixture("plain-v15.blob");
  sim_shutdown();
  InitSim();
  World plain2;
  plain2.Init(3, 1, oxygen, 1.0f, 300.0f);
  Boot(t, plain2);
  std::vector<uint8_t> plain2_vis;
  for (int i = 0; i < 3; ++i) Tick(plain2, &plain2_vis);
  const std::vector<uint8_t> plain2_blob = Save(0, 0);
  printf("  plain world now: %zu bytes; pinned v15 fixture: %zu bytes\n", plain2_blob.size(),
         fx15.size());
  Check(plain2_blob.size() == fx15.size() &&
            memcmp(plain2_blob.data(), fx15.data(), fx15.size()) == 0,
        "a world with nothing to say still writes the pinned v15 blob BYTE FOR BYTE -- four "
        "kSaved properties are registered in every world now, and registering them changed "
        "nothing about what such a world saves");

  // The same migration arms for kSaveVersionExtensions (18), pinned before the
  // writer moved to 19: nothing in this tree writes a v18 blob any more.
  printf("\n=== vftest: legacy v18 blobs still load ===\n");
  {
    const std::vector<uint8_t> fx18 = ReadFixture("gas-v18.blob");
    int32_t fx18_version = -1;
    if (fx18.size() > 12) memcpy(&fx18_version, fx18.data() + 8, 4);
    Check(fx18_version == 18, "the pinned gas fixture really is a v18 blob");
    sim_shutdown();
    InitSim();
    SendRaw(SimMessageHash::Elements_CreateTable, t.elements);
    SendRaw(SimMessageHash::Disease_CreateTable, t.diseases);
    sim_handle_message(static_cast<int32_t>(SimMessageHash::Load),
                       static_cast<int>(fx18.size()), fx18.data());
    double fx18_total = 0.0;
    for (int c = 0; c < 5; ++c) fx18_total += sim_debug_gas_mass(c, oxygen);
    printf("  v18 fixture restored %.4f kg O2 across 5 cells\n", fx18_total);
    Check(std::fabs(fx18_total - 10.0) < 1e-2,
          "a v18 blob, which has no element palette, still restores its gas mixture with its "
          "species indices taken as saved");

    const std::vector<uint8_t> fx18r = ReadFixture("promoted-room-v18.blob");
    sim_shutdown();
    InitSim();
    SendRaw(SimMessageHash::Elements_CreateTable, t.elements);
    SendRaw(SimMessageHash::Disease_CreateTable, t.diseases);
    sim_handle_message(static_cast<int32_t>(SimMessageHash::Load),
                       static_cast<int>(fx18r.size()), fx18r.data());
    Check(sim_debug_room_owned(0) == 1 && sim_debug_room_owned(4) == 1,
          "a v18 blob's room promotion still loads");
  }

  // kSaveVersionElementPalette. The mixture's species are element-table INDICES and the game
  // sorts its table, so an element added or removed below a gas moves it: Mod 2's four elements
  // land at 133, and every gas is above that. Before the palette, Oxygen saved with those four
  // and loaded without them came back as Hydrogen at 16x the pressure. These arms build that
  // table here -- four elements inserted at 133, every index field above it shifted -- and move
  // a save across it in both directions.
  printf("\n=== vftest: the element palette -- mixture species survive a changed table ===\n");
  {
    constexpr int kInsertAt = 133, kInserted = 4;
    std::vector<uint8_t> plus4;
    {
      const int n = t.count;
      std::vector<Element> e(static_cast<size_t>(n));
      memcpy(e.data(), t.elements.data() + 4, static_cast<size_t>(n) * sizeof(Element));
      auto shift = [&](uint16_t& v) {
        if (v != 0xFFFF && v >= kInsertAt) v = static_cast<uint16_t>(v + kInserted);
      };
      for (Element& x : e) {
        shift(x.lowTempTransitionIdx);
        shift(x.highTempTransitionIdx);
        shift(x.elementsTableIdx);
        shift(x.sublimateIndex);
        shift(x.convertIndex);
      }
      std::vector<Element> out;
      for (int i = 0; i < n; ++i) {
        if (i == kInsertAt) {
          for (int j = 0; j < kInserted; ++j) {
            Element c = e[static_cast<size_t>(kInsertAt)];
            c.id = 0x5EED0000 + j;
            c.lowTempTransitionIdx = c.highTempTransitionIdx = 0xFFFF;
            c.sublimateIndex = c.convertIndex = 0xFFFF;
            c.elementsTableIdx = static_cast<uint16_t>(kInsertAt + j);
            out.push_back(c);
          }
        }
        out.push_back(e[static_cast<size_t>(i)]);
      }
      const int32_t cnt = static_cast<int32_t>(out.size());
      plus4.resize(4 + out.size() * sizeof(Element));
      memcpy(plus4.data(), &cnt, 4);
      memcpy(plus4.data() + 4, out.data(), out.size() * sizeof(Element));
    }
    const std::vector<uint8_t> klei_table(t.elements.begin(),
                                          t.elements.begin() + 4 + t.count * sizeof(Element));
    const int32_t oxygen_plus4 = oxygen + kInserted, co2_plus4 = co2 + kInserted;
    const int32_t mod_element = kInsertAt + 1;  // one of the four, in the table that has them

    // Saves one cell holding `a` and optionally `b` (5 kg and 2 kg) under `save_table`, loads
    // it under `load_table`, and reads what the cell holds and the load's own report.
    struct Moved {
      int32_t version = -1;
      size_t blob_size = 0;
      float before_pressure = -1.0f, after_pressure = -1.0f;
      uint16_t after_dominant = 0xFFFF;
      float after_total = 0.0f, after_a = 0.0f, after_b = 0.0f;
    };
    auto move = [&](const std::vector<uint8_t>& save_table, const std::vector<uint8_t>& load_table,
                    int32_t a, int32_t b, int32_t a_after, int32_t b_after) {
      Moved m;
      sim_shutdown();
      InitSim();
      SendRaw(SimMessageHash::Elements_CreateTable, save_table);
      SendRaw(SimMessageHash::Disease_CreateTable, t.diseases);
      // The world is Oxygen UNDER THE TABLE IT IS SAVED WITH. Klei's index for Oxygen names a
      // different element in `plus4` -- a gas whose low transition is at 1738 K -- and this
      // one-cell world is never simulated, so that cell sat 1438 K out of range until Load's
      // own transition (`LoadTimeStateTransitions`) condensed it and moved the mixture's shared
      // temperature by the 1.5 K overshoot. The arm is about species, not about that.
      World pw;
      pw.Init(1, 1, save_table.size() == plus4.size() ? oxygen_plus4 : oxygen, 1.0f, 300.0f);
      SendWorld(pw);
      SendEmpty(SimMessageHash::Start);
      std::vector<uint8_t> pv;
      Tick(pw, &pv);
      InjectGas(0, a, 5.0f, 300.0f);
      if (b >= 0) InjectGas(0, b, 2.0f, 300.0f);
      for (int i = 0; i < 3; ++i) Tick(pw, &pv);
      m.before_pressure = sim_gas_pressure(0);
      const std::vector<uint8_t> blob = Save(0, 0);
      m.blob_size = blob.size();
      if (blob.size() > 12) memcpy(&m.version, blob.data() + 8, 4);
      sim_shutdown();
      InitSim();
      SendRaw(SimMessageHash::Elements_CreateTable, load_table);
      SendRaw(SimMessageHash::Disease_CreateTable, t.diseases);
      sim_handle_message(static_cast<int32_t>(SimMessageHash::Load),
                         static_cast<int>(blob.size()), blob.data());
      m.after_dominant = sim_gas_dominant_element(0, &m.after_total);
      m.after_pressure = sim_gas_pressure(0);
      if (a_after >= 0) m.after_a = sim_debug_gas_mass(0, a_after);
      if (b_after >= 0) m.after_b = sim_debug_gas_mass(0, b_after);
      return m;
    };

    const Moved same = move(klei_table, klei_table, oxygen, -1, oxygen, -1);
    printf("  unchanged table: v%d, %zu bytes, %.0f Pa -> %.0f Pa\n", same.version,
           same.blob_size, same.before_pressure, same.after_pressure);
    Check(same.version == 19 && same.after_dominant == oxygen && same.after_a == 5.0f &&
              same.after_pressure == same.before_pressure,
          "control: an unchanged table round-trips the mixture exactly");

    const Moved grow = move(klei_table, plus4, oxygen, -1, oxygen_plus4, -1);
    printf("  saved without the four, loaded with them: dominant %u (Oxygen is %d), %.0f -> %.0f Pa\n",
           grow.after_dominant, oxygen_plus4, grow.before_pressure, grow.after_pressure);
    Check(grow.after_dominant == oxygen_plus4 && grow.after_a == 5.0f &&
              grow.after_pressure == grow.before_pressure,
          "Oxygen saved before four elements sort in below it loads as Oxygen at its new index, "
          "at the same pressure");

    const Moved shrink = move(plus4, klei_table, co2_plus4, -1, co2, -1);
    printf("  saved with the four, loaded without: dominant %u (CO2 is %d), %.0f -> %.0f Pa\n",
           shrink.after_dominant, co2, shrink.before_pressure, shrink.after_pressure);
    Check(shrink.after_dominant == co2 && shrink.after_a == 5.0f &&
              shrink.after_pressure == shrink.before_pressure,
          "CO2 saved with them loads as CO2 without them -- it used to load as 5 kg of Vacuum");

    const Moved gone = move(plus4, klei_table, oxygen_plus4, mod_element, oxygen, -1);
    printf("  a species whose element is gone: dominant %u, total %.3f kg, O2 %.3f kg\n",
           gone.after_dominant, gone.after_total, gone.after_a);
    Check(gone.after_dominant == oxygen && gone.after_a == 5.0f && gone.after_total == 5.0f,
          "a species whose element the loading table does not have is dropped, and the other "
          "species in the same cell keep their slot and their mass");

    Check(same.blob_size > 0 && grow.blob_size + 4 * static_cast<size_t>(kInserted) ==
                                    shrink.blob_size,
          "the palette is one hash per element: the same world saved against a table four "
          "elements longer is sixteen bytes bigger");

    // kExtElementIdx. Without it, the palette fixed sim.gas_species and nothing
    // else: a mod's own kSaved property holding element indices loaded as saved, so the same
    // table change would turn its Oxygen into whatever sorted into Oxygen's old slot. The type is
    // what tells a load which records are indices, and it has to reach orphans too -- an
    // uninstalled mod's bytes are re-saved under the NEW table's palette.
    printf("\n=== vftest: kExtElementIdx -- a mod's saved element indices survive a changed "
           "table ===\n");
    constexpr uint32_t kNoEl = 0xFFFFu;
    constexpr uint16_t kPastPalette = 0x7000;
    constexpr const char* kOre = "mod12.ore";
    auto boot_tables = [&](const std::vector<uint8_t>& table) {
      sim_shutdown();
      InitSim();
      SendRaw(SimMessageHash::Elements_CreateTable, table);
      SendRaw(SimMessageHash::Disease_CreateTable, t.diseases);
    };
    auto register_ore = [&]() {
      return RegisterProperty(kOre, ext::kExtElementIdx, ext::kSaved, kNoEl, /*arity=*/3);
    };
    // A 2x1 world under `table`, saved at header (x, y): game cell 0 holds {a, b, no element},
    // game cell 1 holds `c1` in component 0.
    auto save_ore = [&](const std::vector<uint8_t>& table, uint16_t a, uint16_t b, uint16_t c1,
                        int32_t x, int32_t y) {
      boot_tables(table);
      const int32_t prop = register_ore();
      World ow;
      ow.Init(2, 1, table.size() == plus4.size() ? oxygen_plus4 : oxygen, 1.0f, 300.0f);
      SendWorld(ow);
      SendEmpty(SimMessageHash::Start);
      std::vector<uint8_t> v;
      Tick(ow, &v);
      SetCellProperty(0, prop, a, 0);
      SetCellProperty(0, prop, b, 1);
      SetCellProperty(1, prop, c1, 0);
      Tick(ow, &v);  // kSetCellProperty is queued: it lands on this frame
      return Save(x, y);
    };
    struct OreCells {
      bool loaded = false, read = false;
      uint32_t c0[3] = {0, 0, 0};
      uint32_t c1 = 0;
    };
    // Reads cells `g0` and `g1` of the registered property, if `prop` is one.
    auto read_ore = [&](int32_t prop, int32_t g0, int32_t g1, OreCells* o) {
      o->read = prop >= 0 && sim_ext_read_cell_property(g0, prop, 0, &o->c0[0]) == 1 &&
                sim_ext_read_cell_property(g0, prop, 1, &o->c0[1]) == 1 &&
                sim_ext_read_cell_property(g0, prop, 2, &o->c0[2]) == 1 &&
                sim_ext_read_cell_property(g1, prop, 0, &o->c1) == 1;
    };
    auto load_whole = [&](const std::vector<uint8_t>& table, const std::vector<uint8_t>& blob,
                          bool registered, int32_t g0 = 0) {
      OreCells o;
      boot_tables(table);
      const int32_t prop = registered ? register_ore() : -1;
      o.loaded = sim_handle_message(static_cast<int32_t>(SimMessageHash::Load),
                                    static_cast<int>(blob.size()), blob.data()) != nullptr;
      if (registered) read_ore(prop, g0, g0 + 1, &o);
      return o;
    };
    auto show = [](const char* what, const OreCells& o) {
      printf("  %s: loaded %d read %d, cell 0 {%u, %u, %u}, cell 1 %u\n", what, o.loaded, o.read,
             o.c0[0], o.c0[1], o.c0[2], o.c1);
    };
    auto is = [](const OreCells& o, uint32_t a, uint32_t b, uint32_t c1) {
      return o.loaded && o.read && o.c0[0] == a && o.c0[1] == b && o.c0[2] == kNoEl &&
             o.c1 == c1;
    };
    const uint16_t ox = static_cast<uint16_t>(oxygen), cd = static_cast<uint16_t>(co2);
    const uint16_t ox4 = static_cast<uint16_t>(oxygen_plus4);
    const uint16_t cd4 = static_cast<uint16_t>(co2_plus4);
    const uint16_t gone_el = static_cast<uint16_t>(mod_element);

    {
      boot_tables(klei_table);
      Check(RegisterProperty("mod12.hole", 4, ext::kSaved, 0u) == -ext::kExtRegisterBadType,
            "type 4 is not a scalar type: the gap before kExtElementIdx is refused, not "
            "silently read as something");
      const int32_t desc_prop = register_ore();
      ext::ExtCellPropertyDesc od{};
      Check(desc_prop >= 0 && sim_ext_property_describe(desc_prop, &od) == 1 &&
                od.type == ext::kExtElementIdx && od.stride == 2 && od.arity == 3 &&
                od.defaultBits == kNoEl,
            "a kExtElementIdx property registers and describes as two bytes a component");
    }

    const std::vector<uint8_t> ore_klei = save_ore(klei_table, ox, cd, cd, 0, 0);
    int32_t ore_version = -1;
    if (ore_klei.size() > 12) memcpy(&ore_version, ore_klei.data() + 8, 4);
    const OreCells ore_same = load_whole(klei_table, ore_klei, true);
    show("unchanged table", ore_same);
    Check(ore_version == 19 && is(ore_same, ox, cd, cd),
          "control: element indices round-trip unchanged under an unchanged table");

    const OreCells ore_grow = load_whole(plus4, ore_klei, true);
    show("saved without the four, loaded with them", ore_grow);
    Check(is(ore_grow, ox4, cd4, cd4),
          "Oxygen and CO2 saved in a mod's property before four elements sort in below them "
          "load at their new indices, and no element stays no element");

    const std::vector<uint8_t> ore_plus4 = save_ore(plus4, cd4, gone_el, kPastPalette, 0, 0);
    const OreCells ore_shrink = load_whole(klei_table, ore_plus4, true);
    show("saved with the four, loaded without", ore_shrink);
    Check(is(ore_shrink, cd, kNoEl, kNoEl),
          "CO2 comes back as CO2; an element the loading table lacks, and an index past the end "
          "of the palette, both come back as no element");

    // The orphan: saved with the mod under the four, loaded and re-saved under Klei's table
    // WITHOUT it, then loaded with it again under Klei's table. Carried verbatim, its bytes
    // would be plus4 indices written under Klei's palette, which the last load reads as an
    // unchanged table. (Reloading under the four instead would pass with NO remap anywhere --
    // the two missing rewrites cancel -- and the first version of this arm did exactly that.)
    const std::vector<uint8_t> ore_orphan_src = save_ore(plus4, ox4, cd4, cd4, 0, 0);
    const OreCells ore_orphaned = load_whole(klei_table, ore_orphan_src, false);
    const std::vector<uint8_t> ore_orphan_resaved = Save(0, 0);
    int32_t resaved_version = -1;
    if (ore_orphan_resaved.size() > 12) memcpy(&resaved_version, ore_orphan_resaved.data() + 8, 4);
    const OreCells ore_back = load_whole(klei_table, ore_orphan_resaved, true);
    show("orphaned under Klei's table, re-saved, loaded with the mod again", ore_back);
    Check(ore_orphaned.loaded && resaved_version == 19 && is(ore_back, ox, cd, cd),
          "an uninstalled mod's element indices are remapped while they wait as an orphan, so "
          "a save made without the mod gives them back as the elements they were");

    // The cluster path, `LoadIntoCluster`: a blob smaller than the allocation, placed at its
    // own header, restored per cell. Registered, then orphaned and re-saved whole.
    constexpr int32_t kOreClusterW = 8, kOreClusterH = 4, kOreX = 2, kOreY = 1;
    const std::vector<uint8_t> ore_part = save_ore(plus4, cd4, gone_el, ox4, kOreX, kOreY);
    auto load_part = [&](bool registered) {
      OreCells o;
      boot_tables(klei_table);
      const int32_t prop = registered ? register_ore() : -1;
      Writer alloc;
      alloc.Put<int32_t>(kOreClusterW);
      alloc.Put<int32_t>(kOreClusterH);
      Send(SimMessageHash::AllocateCells, alloc);
      SendEmpty(SimMessageHash::ClearUnoccupiedCells);
      o.loaded = sim_handle_message(static_cast<int32_t>(SimMessageHash::Load),
                                    static_cast<int>(ore_part.size()), ore_part.data()) != nullptr;
      const int32_t g0 = kOreY * kOreClusterW + kOreX;
      if (registered) read_ore(prop, g0, g0 + 1, &o);
      return o;
    };
    const OreCells ore_cluster = load_part(true);
    show("one world of a cluster, saved with the four, loaded without", ore_cluster);
    Check(is(ore_cluster, cd, kNoEl, ox),
          "the cluster load path remaps a registered property's element indices per cell");

    const OreCells ore_cluster_orphan = load_part(false);
    const std::vector<uint8_t> ore_cluster_resaved = Save(0, 0);
    const OreCells ore_cluster_back =
        load_whole(klei_table, ore_cluster_resaved, true, kOreY * kOreClusterW + kOreX);
    show("cluster orphan re-saved whole, loaded with the mod again", ore_cluster_back);
    Check(ore_cluster_orphan.loaded && is(ore_cluster_back, cd, kNoEl, ox),
          "and remaps an orphan's per-cell bytes the same way");
  }

  // The EIGHTH checkpoint component. Load clears radiation in Vacuum cells, as Klei's does;
  // ext::kSetCellRadiation puts the field back, and it is IMMEDIATE, so the very next
  // publication carries it. Without it a replay from a checkpoint loses radiation.
  printf("\n=== vftest: kSetCellRadiation -- the radiation Load clears comes back ===\n");
  {
    auto debug_cell_radiation = reinterpret_cast<int32_t (*)(float*, int32_t)>(
        GetProcAddress(g_sim, "SIM_DebugCellRadiation"));
    Check(debug_cell_radiation != nullptr, "SIM_DebugCellRadiation is exported");
    if (debug_cell_radiation != nullptr) {
      constexpr int32_t kRadCell = 1, kRadW = 3;
      sim_shutdown();
      InitSim();
      World rad_world;
      rad_world.Init(kRadW, 1, vacuum, 0.0f, 0.0f);
      Boot(t, rad_world);
      std::vector<uint8_t> rad_visible;
      CellRadiationModification add{};
      add.cellIdx = kRadCell;
      add.radiationDelta = 50.0f;
      add.callbackIdx = -1;
      sim_handle_message(static_cast<int32_t>(SimMessageHash::CellRadiationModification),
                         sizeof(add), reinterpret_cast<const uint8_t*>(&add));
      const GameDataUpdate* ru = nullptr;
      for (int i = 0; i < 2; ++i) ru = Tick(rad_world, &rad_visible);
      const float held = ru ? ru->radiation[kRadCell] : 0.0f;
      const int32_t n = debug_cell_radiation(nullptr, 0);
      std::vector<float> field(n > 0 ? static_cast<size_t>(n) : 0);
      if (n > 0) debug_cell_radiation(field.data(), n);
      printf("  vacuum cell holds %.4f rads before the save; field of %d padded cells\n", held, n);
      Check(held > 0.0f && ru && ru->elementIdx[kRadCell] == vacuum,
            "sanity check: a Vacuum cell holds radiation in a running world");
      const std::vector<uint8_t> rad_blob = Save(0, 0);

      sim_shutdown();
      InitSim();
      SendRaw(SimMessageHash::Elements_CreateTable, t.elements);
      SendRaw(SimMessageHash::Disease_CreateTable, t.diseases);
      ru = static_cast<const GameDataUpdate*>(sim_handle_message(
          static_cast<int32_t>(SimMessageHash::Load), static_cast<int>(rad_blob.size()),
          rad_blob.data()));
      ru = static_cast<const GameDataUpdate*>(SendEmpty(SimMessageHash::Start));
      Check(ru && ru->radiation[kRadCell] == 0.0f,
            "Load clears it, as Klei's Load does -- the reason the component exists");

      std::vector<uint8_t> payload(sizeof(ext::SetCellRadiationMessage) + field.size() * 4);
      const int32_t count = static_cast<int32_t>(field.size());
      memcpy(payload.data(), &count, 4);
      if (!field.empty()) memcpy(payload.data() + 4, field.data(), field.size() * 4);
      sim_handle_message(ext::kSetCellRadiation, static_cast<int>(payload.size()),
                         payload.data());
      ru = Tick(rad_world, &rad_visible);
      Check(ru && ru->radiation[kRadCell] == held,
            "kSetCellRadiation puts it back, and the first publication after it already "
            "carries the value -- it is immediate");
    }
  }

  // The NINTH checkpoint component. A frame reads the visibility mask the game sent two
  // PrepareGameData calls earlier, and every allocate and load zeroes the three buffers, as the
  // game's new world starts. A replay restored into those zeroed buffers refuses the falling
  // liquid its run had handed over, so it diverges from the run it replays.
  // ext::kSetVisibilityState puts the three buffers back. Not headless, or nothing spawns at all.
  printf("\n=== vftest: kSetVisibilityState -- the visibility mask a load zeroes comes back ===\n");
  {
    auto debug_visibility = reinterpret_cast<int32_t (*)(uint8_t*, int32_t)>(
        GetProcAddress(g_sim, "SIM_DebugVisibilityState"));
    Check(debug_visibility != nullptr, "SIM_DebugVisibilityState is exported");
    uint16_t vis_water = 0;
    if (debug_visibility != nullptr && Resolve(t, kWater, "Water", &vis_water)) {
      constexpr int32_t kVisW = 5, kVisH = 6, kVisWater = 3 * kVisW + 2;
      World vis_world;
      vis_world.Init(kVisW, kVisH, oxygen, 1.0f, 300.0f);
      vis_world.element[kVisWater] = vis_water;
      vis_world.mass[kVisWater] = 500.0f;
      vis_world.temperature[kVisWater] = 300.0f;
      // diffsim `hang`'s shape: the water's left neighbour is open and the cell under that is
      // granite, so the permeability probe sends it down the spawn branch every substep rather
      // than swapping it with the gas below. It stands still until it may be handed over.
      uint16_t vis_granite = 0;
      if (!Resolve(t, kGranite, "Granite", &vis_granite)) return 1;
      vis_world.element[kVisWater - kVisW - 1] = vis_granite;
      vis_world.mass[kVisWater - kVisW - 1] = 2000.0f;
      sim_shutdown();
      InitSim();
      SendRaw(SimMessageHash::Elements_CreateTable, t.elements);
      SendRaw(SimMessageHash::Disease_CreateTable, t.diseases);
      {
        Writer b;
        b.Put<int32_t>(vis_world.width);
        b.Put<int32_t>(vis_world.height);
        b.Put<uint32_t>(12345u);
        b.PutBool(false);  // radiation
        b.PutBool(false);  // headless: the one thing this arm cannot run without
        for (size_t i = 0; i < vis_world.Count(); ++i) {
          Cell c{};
          c.elementIdx = vis_world.element[i];
          c.mass = vis_world.mass[i];
          c.temperature = vis_world.temperature[i];
          c.insulation = 255;
          b.PutRaw(&c, sizeof(Cell));
        }
        for (size_t i = 0; i < vis_world.Count(); ++i) {
          DiseaseCell d{};
          d.diseaseIdx = 0xFF;
          b.PutRaw(&d, sizeof(DiseaseCell));
        }
        for (size_t i = 0; i < vis_world.Count(); ++i) {
          SimBackwall bw{};
          b.PutRaw(&bw, sizeof(SimBackwall));
        }
        Send(SimMessageHash::SimData_InitializeFromCells, b);
        SendEmpty(SimMessageHash::Start);
      }
      std::vector<uint8_t> all_seen(vis_world.Count(), 1);
      auto read_state = [&]() {
        const int32_t n = debug_visibility(nullptr, 0);
        std::vector<uint8_t> v(n > 0 ? static_cast<size_t>(n) : 0);
        if (n > 0) debug_visibility(v.data(), n);
        return v;
      };
      // Frames until the water is handed over as falling liquid, at most `limit`; -1 if never.
      auto frames_to_spawn = [&](int limit) {
        for (int i = 1; i <= limit; ++i) {
          const GameDataUpdate* u = Tick(vis_world, &all_seen);
          if (u && u->numSpawnFallingLiquidInfo > 0) return i;
        }
        return -1;
      };

      // One frame with the whole world seen: that frame still reads the zeroed mask, so the
      // water is still there, and the game-side buffers now hold the mask the next frames read.
      const GameDataUpdate* first = Tick(vis_world, &all_seen);
      Check(first && first->numSpawnFallingLiquidInfo == 0 &&
                first->elementIdx[kVisWater] == vis_water,
            "sanity check: the first frame reads a zeroed mask and hands nothing over");
      const std::vector<uint8_t> vis_blob = Save(0, 0);
      const std::vector<uint8_t> vis_state = read_state();
      // The scheduling counters too, as a replay sends them: a Load skips the physics of its
      // first frame, and the captured counters (skip 0) cancel that. Without them both restores
      // lose a frame to the skip and the mask's two frames of lag hide behind it.
      auto debug_sched = reinterpret_cast<bool (*)(ext::SetSchedulingStateMessage*)>(
          GetProcAddress(g_sim, "SIM_DebugSchedulingState"));
      ext::SetSchedulingStateMessage vis_sched{};
      const bool vis_sched_ok = debug_sched != nullptr && debug_sched(&vis_sched);
      Check(vis_sched_ok, "sanity check: the scheduling counters are readable");
      const size_t header = sizeof(ext::SetVisibilityStateMessage);
      Check(vis_state.size() == header + 3 * vis_world.Count(),
            "the getter's payload is the header plus three buffers of one byte per game cell");
      size_t seen_bytes = 0;
      for (size_t i = header; i < vis_state.size(); ++i) seen_bytes += vis_state[i] != 0;
      printf("  checkpoint after one seen frame: %zu bytes, %zu of them non-zero\n",
             vis_state.size(), seen_bytes);
      Check(seen_bytes > 0, "sanity check: the mask the game sent is in the buffers");

      auto restore = [&](bool with_state) {
        const int32_t dims[2] = {kVisW, kVisH};
        sim_handle_message(static_cast<int32_t>(SimMessageHash::AllocateCells), sizeof(dims),
                           reinterpret_cast<const uint8_t*>(dims));
        sim_handle_message(static_cast<int32_t>(SimMessageHash::Load),
                           static_cast<int>(vis_blob.size()), vis_blob.data());
        sim_handle_message(ext::kSetSchedulingState, static_cast<int>(sizeof(vis_sched)),
                           reinterpret_cast<const uint8_t*>(&vis_sched));
        if (with_state) {
          sim_handle_message(ext::kSetVisibilityState, static_cast<int>(vis_state.size()),
                             vis_state.data());
        }
      };
      const int ref = frames_to_spawn(12);

      restore(false);
      const std::vector<uint8_t> zeroed = read_state();
      size_t zeroed_bytes = 0;
      for (size_t i = header; i < zeroed.size(); ++i) zeroed_bytes += zeroed[i] != 0;
      Check(zeroed.size() == vis_state.size() && zeroed_bytes == 0,
            "a Load zeroes all three buffers, as Klei's new world starts -- the reason the "
            "component exists");
      const int without = frames_to_spawn(12);

      // Stored with the frame buffer and the sim-side one exchanged, which the restore's first
      // publication swaps back (abi header); read back before that frame, that is the layout.
      std::vector<uint8_t> stored = vis_state;
      {
        const size_t n = vis_world.Count();
        int32_t slot = 0;
        memcpy(&slot, vis_state.data() + 4, 4);
        uint8_t* frame_buf = stored.data() + header;
        uint8_t* sim_side = stored.data() + header + n * (1 + static_cast<size_t>(slot));
        std::swap_ranges(frame_buf, frame_buf + n, sim_side);
      }
      restore(true);
      Check(read_state() == stored,
            "kSetVisibilityState stores the captured buffers, slot included, with the frame and "
            "sim-side ones exchanged for the publication that follows a Load");
      const int with = frames_to_spawn(12);
      printf("  frames to the first falling-liquid record: run %d, restored without the mask "
             "%d, with it %d\n", ref, without, with);
      Check(ref > 0 && with > 0 && without > 0,
            "sanity check: the water is handed over in all three runs");
      Check(with == ref,
            "restored with the mask, the water is handed over on the same frame as the run it "
            "was restored from");
      Check(with < without,
            "and restored without it, later -- the frames a zeroed mask cost are the replay "
            "divergence this closes");

      // Refused whole: a count that is not this world's, and a slot that is not 0 or 1.
      restore(true);
      std::vector<uint8_t> bad = vis_state;
      int32_t bad_slot = 2;
      memcpy(bad.data() + 4, &bad_slot, 4);
      sim_handle_message(ext::kSetVisibilityState, static_cast<int>(bad.size()), bad.data());
      std::vector<uint8_t> short_one(vis_state.begin(), vis_state.end() - 1);
      sim_handle_message(ext::kSetVisibilityState, static_cast<int>(short_one.size()),
                         short_one.data());
      Check(read_state() == stored,
            "a bad slot and a short payload are refused without touching the buffers");
    }
  }

  // ---- A CHECKPOINT RESTORE SKIPS THE LOAD-TIME TRANSITION.
  //
  // Every Load ends with the load-time state transition, which moves a cell outside its
  // element's range one step with the 1.5 K overshoot. A run publishes such cells -- a
  // supercooled drop that landed this substep freezes on the next -- so a restore that pushed
  // them through the step replayed from a state the run never held. ext::kSetLoadIsRestore,
  // sent before the Load, skips the pass for that one Load. The cell is read off the frame that
  // publishes the Load, which runs no physics (a Load sets one skipped physics frame), so what
  // it shows is what the Load wrote.
  printf("\n=== vftest: kSetLoadIsRestore -- a restore skips the load-time transition ===\n");
  {
    uint16_t lr_water = 0, lr_ice = 0;
    if (Resolve(t, kWater, "Water", &lr_water) && Resolve(t, kIce, "Ice", &lr_ice)) {
      constexpr int32_t kLrW = 5, kLrH = 5, kLrCell = 2 * kLrW + 2;
      constexpr float kLrTemp = 200.0f;  // 73 K under Water's freezing point, far past the margin
      World lr_world;
      lr_world.Init(kLrW, kLrH, oxygen, 1.0f, 300.0f);
      lr_world.element[kLrCell] = lr_water;
      lr_world.mass[kLrCell] = 100.0f;
      lr_world.temperature[kLrCell] = kLrTemp;
      sim_shutdown();
      InitSim();
      SendRaw(SimMessageHash::Elements_CreateTable, t.elements);
      SendRaw(SimMessageHash::Disease_CreateTable, t.diseases);
      {
        Writer b;
        b.Put<int32_t>(lr_world.width);
        b.Put<int32_t>(lr_world.height);
        b.Put<uint32_t>(12345u);
        b.PutBool(false);  // radiation
        b.PutBool(true);   // headless
        for (size_t i = 0; i < lr_world.Count(); ++i) {
          Cell c{};
          c.elementIdx = lr_world.element[i];
          c.mass = lr_world.mass[i];
          c.temperature = lr_world.temperature[i];
          c.insulation = 255;
          b.PutRaw(&c, sizeof(Cell));
        }
        for (size_t i = 0; i < lr_world.Count(); ++i) {
          DiseaseCell d{};
          d.diseaseIdx = 0xFF;
          b.PutRaw(&d, sizeof(DiseaseCell));
        }
        for (size_t i = 0; i < lr_world.Count(); ++i) {
          SimBackwall bw{};
          b.PutRaw(&bw, sizeof(SimBackwall));
        }
        // InitializeFromCells rewrites nothing (docs/SAVE-FORMAT.md), so the supercooled water
        // is in the grid exactly as sent, and the blob saved from it holds it the same way.
        Send(SimMessageHash::SimData_InitializeFromCells, b);
      }
      const std::vector<uint8_t> lr_blob = Save(0, 0);
      Check(!lr_blob.empty(), "sanity check: the supercooled world saves");

      auto send_restore = [&](int32_t restore, int32_t reserved) {
        ext::SetLoadIsRestoreMessage m{};
        m.restore = restore;
        m.reserved = reserved;
        sim_handle_message(ext::kSetLoadIsRestore, static_cast<int>(sizeof(m)),
                           reinterpret_cast<const uint8_t*>(&m));
      };
      struct Seen {
        bool ok = false;
        uint16_t element = 0;
        float temperature = 0.0f;
      };
      auto load = [&](const std::vector<uint8_t>& blob) {
        Seen r;
        const int32_t dims[2] = {kLrW, kLrH};
        sim_handle_message(static_cast<int32_t>(SimMessageHash::AllocateCells), sizeof(dims),
                           reinterpret_cast<const uint8_t*>(dims));
        if (!sim_handle_message(static_cast<int32_t>(SimMessageHash::Load),
                                static_cast<int>(blob.size()), blob.data())) {
          return r;
        }
        std::vector<uint8_t> unseen(lr_world.Count(), 0);
        const GameDataUpdate* u = Tick(lr_world, &unseen);
        if (u && u->elementIdx && u->temperature) {
          r.ok = true;
          r.element = u->elementIdx[kLrCell];
          r.temperature = u->temperature[kLrCell];
        }
        return r;
      };
      auto describe = [&](const char* what, const Seen& r) {
        printf("  %-40s %s at %.3f K\n", what,
               !r.ok ? "(no update)" : r.element == lr_water ? "Water"
                                     : r.element == lr_ice   ? "Ice"
                                                             : "other",
               r.temperature);
      };

      const Seen plain = load(lr_blob);
      describe("a plain Load:", plain);
      Check(plain.ok && plain.element == lr_ice && plain.temperature == kLrTemp + 1.5f,
            "a plain Load freezes the supercooled water with the 1.5 K overshoot, as Klei's "
            "does -- the positive control, and what a save still gets");

      send_restore(1, 0);
      const Seen restored = load(lr_blob);
      describe("a Load after kSetLoadIsRestore:", restored);
      Check(restored.ok && restored.element == lr_water && restored.temperature == kLrTemp,
            "after kSetLoadIsRestore the Load leaves the cell as it was saved");

      const Seen after = load(lr_blob);
      describe("the Load after that:", after);
      Check(after.ok && after.element == lr_ice,
            "the flag is one-shot: the next Load transitions again");

      send_restore(1, 0);
      send_restore(0, 0);
      const Seen disarmed = load(lr_blob);
      Check(disarmed.ok && disarmed.element == lr_ice, "restore 0 disarms it");

      send_restore(2, 0);
      const Seen bad_value = load(lr_blob);
      send_restore(1, 7);
      const Seen bad_reserved = load(lr_blob);
      Check(bad_value.ok && bad_value.element == lr_ice && bad_reserved.ok &&
                bad_reserved.element == lr_ice,
            "a restore of 2 and a non-zero reserved are refused and arm nothing");

      // A restore whose blob is rejected must not leave the flag armed for the next real load.
      send_restore(1, 0);
      const std::vector<uint8_t> garbage(16, 0xAB);
      sim_handle_message(static_cast<int32_t>(SimMessageHash::Load),
                         static_cast<int>(garbage.size()), garbage.data());
      const Seen after_reject = load(lr_blob);
      Check(after_reject.ok && after_reject.element == lr_ice,
            "a Load that rejects its blob still consumes the flag");
    }
  }

  // ---- THE SEVENTH CHECKPOINT COMPONENT.
  //
  // A save blob carries kSaved and nothing else. kRehydrated and kCheckpointOnly are the two
  // classes that survive only if something else carries them, and a tool driving the DLL
  // with no mods loaded has no owner present to re-push either. Every one of the
  // other six components was found the same way: an exact stop kept passing while a replay
  // diverged, because a stop restores and publishes without stepping physics, so stale state
  // never gets to act. These arms are the deliberate version of that test.
  printf("\n=== vftest: extension registry -- the seventh checkpoint component ===\n");
  sim_shutdown();
  InitSim();
  const int32_t cp_reh = RegisterProperty("mod8.promise", ext::kExtF32, ext::kRehydrated, 0u);
  const int32_t cp_only = RegisterProperty("mod8.carried", ext::kExtU16, ext::kCheckpointOnly,
                                           0u, /*arity=*/3);
  const int32_t cp_saved = RegisterProperty("mod8.persisted", ext::kExtF32, ext::kSaved, 0u);
  const int32_t cp_quiet = RegisterProperty("mod8.quiet", ext::kExtF32, ext::kCheckpointOnly,
                                            BitsOf(7.5f));
  Check(cp_reh > 0 && cp_only > 0 && cp_saved > 0 && cp_quiet > 0,
        "one property of each persistence class registers, plus one that stays at its default");
  World cp_world;
  cp_world.Init(4, 1, oxygen, 1.0f, 300.0f);
  Boot(t, cp_world);
  std::vector<uint8_t> cp_vis;

  const std::vector<uint8_t> cp_empty = ExtCheckpoint();
  printf("  nothing written yet: checkpoint is %zu bytes\n", cp_empty.size());
  Check(cp_empty.size() >= 12,
        "a world with nothing to carry still returns a HEADER, never zero bytes -- a zero is "
        "indistinguishable from a DLL that predates the export, and a caller would then "
        "silently carry six components instead of seven");

  SetCellProperty(1, cp_reh, BitsOf(11.25f));
  for (int32_t c = 0; c < 3; ++c) {
    SetCellProperty(2, cp_only, static_cast<uint32_t>(0x500 + c), c);
  }
  SetCellProperty(3, cp_saved, BitsOf(99.5f));
  Tick(cp_world, &cp_vis);

  const std::vector<uint8_t> cp_blob = ExtCheckpoint();
  const std::vector<uint8_t> cp_save = Save(0, 0);
  printf("  after writes: checkpoint %zu bytes, save blob %zu bytes\n", cp_blob.size(),
         cp_save.size());
  Check(cp_blob.size() > cp_empty.size(),
        "writing a kRehydrated and a kCheckpointOnly property grows the checkpoint");

  // THE CENTRAL EXCLUSION. The save blob is checkpoint component 1, so carrying a kSaved
  // property here as well would restore it twice -- and the second restore would win, which is
  // the same value only for as long as the two agree. Asserted on the bytes rather than on
  // behaviour, because a double restore of a value that happens to match looks like a pass.
  const std::string cp_text(reinterpret_cast<const char*>(cp_blob.data()), cp_blob.size());
  Check(cp_text.find("mod8.promise") != std::string::npos,
        "the checkpoint names the kRehydrated property");
  Check(cp_text.find("mod8.carried") != std::string::npos,
        "and the kCheckpointOnly one");
  Check(cp_text.find("mod8.persisted") == std::string::npos,
        "and NOT the kSaved one -- the save blob already carries it, and restoring it from "
        "both would double-restore it");
  Check(cp_text.find("mod8.quiet") == std::string::npos,
        "nor a property still at its registered default: a megabyte a checkpoint to move a "
        "number nothing wrote is paid on every seek, which is the trade registry_state.h "
        "already made for RadiationState::occlusion");
  Check(cp_text.find("sim.gas_mass") == std::string::npos &&
            cp_text.find("sim.room_promoted") == std::string::npos,
        "and none of the four first-party kSaved properties either");
  Check(cp_text.find("sim.thermal_mass_bonus") == std::string::npos,
        "sim.thermal_mass_bonus is kCheckpointOnly but nothing has written it, so it is "
        "omitted too -- the rule is what the property CONTAINS, not who owns it");

  // THE INSPECTION TWIN, and the same assertion inverted. "What must a checkpoint carry" and
  // "what is in this cell" are different questions, and the properties the first must exclude
  // are exactly the ones the sim itself writes -- so a tool asking the second through the
  // first sees a mod's property and not the sim's own. Asserted on the bytes for the same
  // reason the exclusion above is.
  const std::vector<uint8_t> all_blob = ExtInspectAll();
  const std::string all_text(reinterpret_cast<const char*>(all_blob.data()), all_blob.size());
  printf("  inspection blob: %zu bytes against the checkpoint's %zu\n", all_blob.size(),
         cp_blob.size());
  Check(all_blob.size() > cp_blob.size(),
        "SIM_DebugExtCellStateAll returns MORE than the checkpoint, because it adds the class "
        "the checkpoint must leave out");
  Check(all_text.find("mod8.persisted") != std::string::npos,
        "and it names the kSaved property, which is the whole point: the checkpoint may not "
        "carry it and an inspecting tool most wants to see it");
  Check(all_text.find("mod8.promise") != std::string::npos &&
            all_text.find("mod8.carried") != std::string::npos,
        "without losing either carried class -- all three travel here");
  Check(all_text.find("mod8.quiet") == std::string::npos,
        "a property still at its registered default is omitted from this one too: the same "
        "trade, and the same consequence for a reader -- absent cannot be told from "
        "registered-but-untouched");
  Check(all_blob.size() >= 12 && memcmp(all_blob.data(), cp_blob.data(), 8) == 0,
        "same magic and same version as the checkpoint blob -- one format, one decoder, so a "
        "reader of either cannot drift from the sim");

  // ---- restore into a fresh instance. The save blob restores kSaved; the checkpoint restores
  // the other two. Both halves are sent, in the order a replay harness sends them.
  sim_shutdown();
  InitSim();
  const int32_t r_reh = RegisterProperty("mod8.promise", ext::kExtF32, ext::kRehydrated, 0u);
  const int32_t r_only = RegisterProperty("mod8.carried", ext::kExtU16, ext::kCheckpointOnly,
                                          0u, /*arity=*/3);
  const int32_t r_saved = RegisterProperty("mod8.persisted", ext::kExtF32, ext::kSaved, 0u);
  RegisterProperty("mod8.quiet", ext::kExtF32, ext::kCheckpointOnly, BitsOf(7.5f));
  Check(r_reh == cp_reh && r_only == cp_only && r_saved == cp_saved,
        "the same registrations in the same order yield the same indices");
  SendRaw(SimMessageHash::Elements_CreateTable, t.elements);
  SendRaw(SimMessageHash::Disease_CreateTable, t.diseases);
  sim_handle_message(static_cast<int32_t>(SimMessageHash::Load),
                     static_cast<int>(cp_save.size()), cp_save.data());

  uint32_t cp_bits = 0;
  float cp_f32 = 0.0f;
  Check(sim_ext_read_cell_property(1, r_reh, 0, &cp_bits) == 1 && cp_bits == 0u,
        "straight after the Load the kRehydrated property is at its default -- the save blob "
        "does NOT carry it, which is the gap the seventh component exists to close");
  int32_t cp_out[8] = {};
  int32_t cp_n = sim_ext_outstanding_rehydration(cp_out, 8);
  printf("  after Load alone: %d outstanding rehydration(s)\n", cp_n);
  Check(cp_n == 1 && cp_out[0] == r_reh,
        "and the registry SAYS SO: exactly the property somebody owes a re-push for is named");

  SendExtCheckpoint(cp_blob);
  Tick(cp_world, &cp_vis);

  Check(sim_ext_read_cell_property(1, r_reh, 0, &cp_bits) == 1, "the property reads back");
  memcpy(&cp_f32, &cp_bits, sizeof(cp_f32));
  printf("  after the checkpoint: mod8.promise[1] = %.2f\n", cp_f32);
  Check(cp_f32 == 11.25f,
        "the checkpoint restores a kRehydrated property no save blob carried and no owner "
        "re-pushed");
  bool cp_lanes = true;
  for (int32_t c = 0; c < 3; ++c) {
    uint32_t lb = 0;
    if (sim_ext_read_cell_property(2, r_only, c, &lb) != 1 ||
        lb != static_cast<uint32_t>(0x500 + c)) {
      printf("  lane %d came back 0x%X, expected 0x%X\n", c, lb, 0x500 + c);
      cp_lanes = false;
    }
  }
  Check(cp_lanes, "and every lane of a kCheckpointOnly arity-3 property, in its own lane");
  Check(sim_ext_read_cell_property(3, r_saved, 0, &cp_bits) == 1, "the kSaved property reads");
  memcpy(&cp_f32, &cp_bits, sizeof(cp_f32));
  Check(cp_f32 == 99.5f,
        "and the kSaved property still holds what the SAVE BLOB put there -- the checkpoint "
        "neither carried it nor clobbered it");

  cp_n = sim_ext_outstanding_rehydration(cp_out, 8);
  printf("  after the checkpoint: %d outstanding rehydration(s)\n", cp_n);
  Check(cp_n == 0,
        "and the outstanding list is now EMPTY. A harness that restored the property "
        "correctly must not still be told it is missing -- a diagnostic that fires on every "
        "seek is the same as no diagnostic");

  // ---- ABSENT MEANS "WAS DEFAULT", NOT "LEAVE IT ALONE". The blob omits an all-default
  // property; a restore into a world where somebody has since written that property has to put
  // the default back, or the restore silently keeps a value the checkpoint never saw.
  printf("\n=== vftest: checkpoint restore RESETS what the blob omitted ===\n");
  const int32_t stale = sim_ext_property_index("mod8.quiet");
  Check(stale > 0, "mod8.quiet is registered in this instance");
  SetCellProperty(0, stale, BitsOf(1234.0f));
  Tick(cp_world, &cp_vis);
  Check(sim_ext_read_cell_property(0, stale, 0, &cp_bits) == 1 && cp_bits == BitsOf(1234.0f),
        "a value is written into a property the checkpoint blob does not mention");
  SendExtCheckpoint(cp_blob);
  Tick(cp_world, &cp_vis);
  Check(sim_ext_read_cell_property(0, stale, 0, &cp_bits) == 1 && cp_bits == BitsOf(7.5f),
        "restoring the checkpoint puts that property back to its REGISTERED DEFAULT -- absent "
        "from the blob means the checkpoint ran with the default, never 'do not touch'");
  Check(sim_ext_read_cell_property(1, r_reh, 0, &cp_bits) == 1 && cp_bits == BitsOf(11.25f),
        "while the property the blob DID carry is unchanged by the second restore");

  // ---- refusals. Every one of these must leave the registry exactly as it was, which is what
  // `registry_state::Load` promises for the sixth component and what this promises for the
  // seventh. Asserted by reading a live value back after each attempt, not by the return code.
  printf("\n=== vftest: checkpoint blob refusals leave the world alone ===\n");
  std::vector<uint8_t> bad = cp_blob;
  bad[0] ^= 0xFFu;
  SendExtCheckpoint(bad);
  Tick(cp_world, &cp_vis);
  Check(sim_ext_read_cell_property(1, r_reh, 0, &cp_bits) == 1 && cp_bits == BitsOf(11.25f),
        "a blob with the wrong magic is rejected and changes nothing");

  bad = cp_blob;
  const uint32_t bogus_version = 0xDEADu;
  memcpy(bad.data() + 4, &bogus_version, 4);
  SendExtCheckpoint(bad);
  Tick(cp_world, &cp_vis);
  Check(sim_ext_read_cell_property(1, r_reh, 0, &cp_bits) == 1 && cp_bits == BitsOf(11.25f),
        "an unknown blob version is rejected rather than misread -- a reader that guessed "
        "would restore garbage into a physics input");

  bad.assign(cp_blob.begin(), cp_blob.begin() + static_cast<long>(cp_blob.size() / 2));
  SendExtCheckpoint(bad);
  Tick(cp_world, &cp_vis);
  Check(sim_ext_read_cell_property(1, r_reh, 0, &cp_bits) == 1 && cp_bits == BitsOf(11.25f),
        "and so is a truncated one, at the half-way point rather than at a record boundary");

  SendExtCheckpoint(std::vector<uint8_t>());
  Tick(cp_world, &cp_vis);
  Check(sim_ext_read_cell_property(1, r_reh, 0, &cp_bits) == 1 && cp_bits == BitsOf(11.25f),
        "an empty payload is reported, not applied");

  // A SHAPE MISMATCH is the one that silently produces garbage if it is got wrong: the same
  // name registered with a different arity, and old bytes reinterpreted as the new shape.
  sim_shutdown();
  InitSim();
  const int32_t sh_reh = RegisterProperty("mod8.promise", ext::kExtF32, ext::kRehydrated, 0u);
  RegisterProperty("mod8.carried", ext::kExtU16, ext::kCheckpointOnly, 0u, /*arity=*/5);
  World sh_world;
  sh_world.Init(4, 1, oxygen, 1.0f, 300.0f);
  Boot(t, sh_world);
  std::vector<uint8_t> sh_vis;
  SendExtCheckpoint(cp_blob);
  Tick(sh_world, &sh_vis);
  Check(sim_ext_read_cell_property(1, sh_reh, 0, &cp_bits) == 1 && cp_bits == 0u,
        "a checkpoint whose record disagrees with this build's arity is refused WHOLE -- the "
        "property that would have matched is left at its default rather than half restored");

  // A property nobody registered is REPORTED and skipped, not fatal: the same policy the save
  // path uses, because a caller that dropped one mod must still get its checkpoint back.
  sim_shutdown();
  InitSim();
  const int32_t partial = RegisterProperty("mod8.promise", ext::kExtF32, ext::kRehydrated, 0u);
  World pt_world;
  pt_world.Init(4, 1, oxygen, 1.0f, 300.0f);
  Boot(t, pt_world);
  std::vector<uint8_t> pt_vis;
  SendExtCheckpoint(cp_blob);
  Tick(pt_world, &pt_vis);
  Check(sim_ext_read_cell_property(1, partial, 0, &cp_bits) == 1 && cp_bits == BitsOf(11.25f),
        "an unregistered record is skipped and the rest of the blob still applies -- dropping "
        "one mod must not cost the checkpoint");

  // A property RECLASSIFIED to kSaved between builds. The blob carries it because the build
  // that wrote the checkpoint thought nobody else would; this build's save blob carries it
  // too. Applying both would restore it twice, and the checkpoint would win -- the same value
  // only for as long as the two agree. Refused, with the property named.
  //
  // This arm exists because the guard is otherwise a rule that has never been made to fire.
  sim_shutdown();
  InitSim();
  const int32_t rc_reh = RegisterProperty("mod8.promise", ext::kExtF32, ext::kRehydrated, 0u);
  const int32_t rc_now_saved = RegisterProperty("mod8.carried", ext::kExtU16, ext::kSaved, 0u,
                                                /*arity=*/3);
  Check(rc_reh > 0 && rc_now_saved > 0,
        "the same two names register, but one of them is kSaved in this build");
  World rc_world;
  rc_world.Init(4, 1, oxygen, 1.0f, 300.0f);
  Boot(t, rc_world);
  std::vector<uint8_t> rc_vis;
  SendExtCheckpoint(cp_blob);
  Tick(rc_world, &rc_vis);
  Check(sim_ext_read_cell_property(1, rc_reh, 0, &cp_bits) == 1 && cp_bits == 0u,
        "a checkpoint carrying a property this build now calls kSaved is refused WHOLE -- the "
        "save blob answers for that property, and a second restore of it would silently win");

  // ---- THE PER-FRAME PUBLISH DESCRIPTOR TABLE.
  //
  // Every other route into extension data is a per-call export that opens with the worker
  // barrier. This is the one that is bound once per tick and read as a span, and the two
  // things worth testing are the ones a caller can get wrong: WHICH frame a table belongs to,
  // and WHEN a subscription starts.
  printf("\n=== vftest: extension registry -- per-frame publish descriptors ===\n");
  sim_shutdown();
  InitSim();
  const int32_t pb_f32 = RegisterProperty("mod9.window", ext::kExtF32, ext::kCheckpointOnly, 0u);
  const int32_t pb_vec = RegisterProperty("mod9.triple", ext::kExtU16, ext::kRehydrated, 0u,
                                          /*arity=*/3);
  const int32_t pb_quiet = RegisterProperty("mod9.unwatched", ext::kExtF32, ext::kSaved, 0u);
  Check(pb_f32 > 0 && pb_vec > 0 && pb_quiet > 0, "three properties register for stage 3");
  World pb_world;
  pb_world.Init(8, 4, oxygen, 1.0f, 300.0f);
  Boot(t, pb_world);
  std::vector<uint8_t> pb_vis;

  const GameDataUpdate* pb_frame = Tick(pb_world, &pb_vis);
  int32_t pb_n = -1;
  const ext::ExtPublishedProperty* pb_tab = sim_ext_published(pb_frame, &pb_n);
  Check(pb_tab != nullptr && pb_n == 0,
        "a live frame nobody has subscribed against returns a NON-NULL table with count 0 -- "
        "'nothing published' and 'that is not a frame I published' are different facts, and a "
        "caller that cannot tell them apart binds garbage");

  int32_t pb_reject = -1;
  Check(sim_ext_published(nullptr, &pb_reject) == nullptr && pb_reject == 0,
        "a null frame is refused with null and count 0");
  GameDataUpdate pb_fake{};
  pb_reject = -1;
  Check(sim_ext_published(&pb_fake, &pb_reject) == nullptr && pb_reject == 0,
        "a GameDataUpdate that is not one of the two live publications is refused -- the "
        "descriptors belong to a frame, so a frame this DLL never published has no table");

  // THE ROTATION. A subscription is a queued message like every other cell message here, so
  // it takes effect on the frame AFTER the one that was already in flight. Asserted rather
  // than worked around: a caller that binds on the tick it subscribed and sees nothing has
  // not found a bug, and this is where that is written down.
  PublishProperty(pb_f32, true);
  PublishProperty(pb_vec, true);
  const GameDataUpdate* pb_same = Tick(pb_world, &pb_vis);
  sim_ext_published(pb_same, &pb_n);
  Check(pb_n == 0, "a subscription sent during a tick is NOT in that tick's table");
  const GameDataUpdate* pb_next = Tick(pb_world, &pb_vis);
  sim_ext_published(pb_next, &pb_n);
  Check(pb_n == 2, "it is in the next one, and only the two subscribed properties are there");
  Check(FindPublished(pb_next, "mod9.unwatched") == nullptr,
        "the property nobody subscribed to is absent -- publishing is opt-in because the copy "
        "is per frame, and a build that reads none of it must not pay to copy it");

  const ext::ExtPublishedProperty* pb_d = FindPublished(pb_next, "mod9.triple");
  Check(pb_d != nullptr, "a descriptor is found BY NAME (the table's order is registration "
                         "order, so an index into it is not a property index)");
  if (pb_d != nullptr) {
    printf("  mod9.triple: type %d persist %d arity %d stride %d cells %d bytes %d\n",
           pb_d->type, pb_d->persist, pb_d->arity, pb_d->stride, pb_d->cellCount,
           pb_d->byteCount);
    Check(pb_d->type == ext::kExtU16 && pb_d->persist == ext::kRehydrated && pb_d->arity == 3 &&
              pb_d->stride == 2,
          "the descriptor reports the registered shape, not a guess");
    Check(pb_d->byteCount == pb_d->cellCount * pb_d->arity * pb_d->stride,
          "byteCount is exactly cellCount * arity * stride -- a span is built from it and "
          "nothing is derived by the caller");
    Check(pb_d->cellCount > 8 * 4,
          "cellCount is PADDED cells, which is what the storage is, not the 32 game cells");
    Check(pb_d->data != nullptr, "the data pointer is non-null");
  }

  // THE COPY IS FRESH EVERY FRAME. Written through the ordinary cell message, so this also
  // shows the table reflects the state AFTER that frame's queue drained rather than the state
  // the previous table was built from.
  // Two ticks, not one, and for the SAME reason the subscription needed two: a message sent
  // during tick N drains at the top of tick N+1. The first version of this arm ticked once
  // and failed, which is the rotation doing exactly what the rest of this file documents.
  SetCellProperty(5, pb_f32, BitsOf(11.25f));
  Tick(pb_world, &pb_vis);
  const GameDataUpdate* pb_w1 = Tick(pb_world, &pb_vis);
  const ext::ExtPublishedProperty* pb_v1 = FindPublished(pb_w1, "mod9.window");
  bool pb_saw_1125 = false, pb_saw_2250 = false;
  if (pb_v1 != nullptr) {
    const float* f = static_cast<const float*>(pb_v1->data);
    for (int32_t i = 0; i < pb_v1->byteCount / 4; ++i) {
      if (f[i] == 11.25f) pb_saw_1125 = true;
      if (f[i] == 22.5f) pb_saw_2250 = true;
    }
  }
  Check(pb_saw_1125 && !pb_saw_2250, "a value written before the tick is in that tick's table");

  SetCellProperty(5, pb_f32, BitsOf(22.5f));
  Tick(pb_world, &pb_vis);
  const GameDataUpdate* pb_w2 = Tick(pb_world, &pb_vis);
  const ext::ExtPublishedProperty* pb_v2 = FindPublished(pb_w2, "mod9.window");
  pb_saw_1125 = false;
  pb_saw_2250 = false;
  if (pb_v2 != nullptr) {
    const float* f = static_cast<const float*>(pb_v2->data);
    for (int32_t i = 0; i < pb_v2->byteCount / 4; ++i) {
      if (f[i] == 11.25f) pb_saw_1125 = true;
      if (f[i] == 22.5f) pb_saw_2250 = true;
    }
  }
  Check(pb_saw_2250 && !pb_saw_1125,
        "and the overwrite replaces it -- the bytes are copied per frame, not a pointer to "
        "the registry handed out once");

  // THE ALTERNATION, which is the whole reason the ABI says the pointers die with the tick.
  // CONSECUTIVE ticks, deliberately: two ticks apart is the same slot again, and an arm that
  // took its two frames two ticks apart would be asserting the opposite of what it claims.
  const GameDataUpdate* pb_alt_a = Tick(pb_world, &pb_vis);
  const ext::ExtPublishedProperty* pb_alt_da = FindPublished(pb_alt_a, "mod9.window");
  const GameDataUpdate* pb_alt_b = Tick(pb_world, &pb_vis);
  const ext::ExtPublishedProperty* pb_alt_db = FindPublished(pb_alt_b, "mod9.window");
  Check(pb_alt_a != pb_alt_b && pb_alt_da != nullptr && pb_alt_db != nullptr &&
            pb_alt_da != pb_alt_db,
        "consecutive frames publish into DIFFERENT slots, descriptor table included -- a "
        "pointer held across a tick is a use-after-free waiting for the frame after next");

  PublishProperty(pb_f32, false);
  Tick(pb_world, &pb_vis);
  const GameDataUpdate* pb_off = Tick(pb_world, &pb_vis);
  sim_ext_published(pb_off, &pb_n);
  Check(pb_n == 1 && FindPublished(pb_off, "mod9.window") == nullptr &&
            FindPublished(pb_off, "mod9.triple") != nullptr,
        "unsubscribing drops one property and leaves the other");

  // A SUBSCRIPTION SURVIVES A LOAD. It is a statement about the world, not about one
  // allocation of it, and a caller re-subscribing after every load would find an empty table
  // on exactly the tick a load makes interesting.
  const std::vector<uint8_t> pb_save = Save(0, 0);
  SendRaw(SimMessageHash::Load, pb_save);
  const GameDataUpdate* pb_loaded = Tick(pb_world, &pb_vis);
  const ext::ExtPublishedProperty* pb_after = FindPublished(pb_loaded, "mod9.triple");
  Check(pb_after != nullptr, "the subscription is still there after a load");
  if (pb_after != nullptr) {
    Check(pb_after->byteCount == pb_after->cellCount * pb_after->arity * pb_after->stride,
          "and the descriptor is re-sized from the reallocated storage rather than kept");
  }

  // ---- PER-FRAME EVENT STREAMS.
  //
  // Same publish shape as the descriptor table above, with two differences that are the whole
  // point and are what these arms are about: a stream is declared by NATIVE code and never by
  // a message, and a subscription gates COLLECTION rather than only the copy. The producer
  // under test is "sim.message_refused", which turns 56 silent early returns into records.
  printf("\n=== vftest: extension registry -- per-frame event streams ===\n");

  const int32_t es_idx = sim_ext_stream_index(ext::kStreamMessageRefused);
  Check(es_idx >= 0,
        "the first-party stream is declared at SIM_Initialize and resolves by name");
  Check(sim_ext_stream_index("mod9.nosuchstream") == -1,
        "an undeclared name resolves to -1 -- and there is no way for a caller to declare "
        "one, because an event is something the SIM observed and every producer is native");

  const GameDataUpdate* es_quiet = Tick(pb_world, &pb_vis);
  int32_t es_n = -1;
  Check(sim_ext_events(es_quiet, &es_n) != nullptr && es_n == 0,
        "a live frame with no stream subscribed returns a NON-NULL table with count 0");
  es_n = -1;
  Check(sim_ext_events(nullptr, &es_n) == nullptr && es_n == 0,
        "a null frame is refused with null and count 0");
  GameDataUpdate es_fake{};
  es_n = -1;
  Check(sim_ext_events(&es_fake, &es_n) == nullptr && es_n == 0,
        "a GameDataUpdate this DLL never published has no stream table either");

  // THE COLLECTION GATE, and this is the arm that distinguishes stage 4 from stage 3. A
  // refusal that happens while nobody is subscribed is not buffered anywhere, so subscribing
  // afterwards cannot retrieve it. That is the property that makes it safe to put an Emit on
  // a path that runs every frame.
  const uint8_t es_short[4] = {1, 2, 3, 4};
  sim_handle_message(ext::kSetCellProperty, 4, es_short);
  Tick(pb_world, &pb_vis);
  Tick(pb_world, &pb_vis);
  SubscribeStream(es_idx, true);
  Tick(pb_world, &pb_vis);
  const GameDataUpdate* es_on = Tick(pb_world, &pb_vis);
  const ext::ExtPublishedStream* es_st = FindStream(es_on, ext::kStreamMessageRefused);
  Check(es_st != nullptr,
        "once subscribed the stream appears in the table (one frame later, the same rotation "
        "every queued message here lives under)");
  Check(es_st != nullptr && es_st->count == 0,
        "and it is EMPTY -- a refusal that happened before anybody subscribed was never "
        "collected, because subscription gates collection and not just the copy");
  if (es_st != nullptr) {
    printf("  sim.message_refused: stride %d count %d bytes %d dropped %d\n", es_st->stride,
           es_st->count, es_st->byteCount, es_st->dropped);
    Check(es_st->stride == static_cast<int32_t>(sizeof(ext::ExtRefusedMessage)) &&
              es_st->byteCount == es_st->count * es_st->stride && es_st->dropped == 0,
          "the descriptor reports the declared stride, byteCount == count * stride, and no "
          "drops -- a truncated list must never look like a complete one");
  }

  // A SHORT PAYLOAD IS NOW REPORTED. This is the 56-site case: one edit in the `Payload`
  // template, and a mod can finally tell a write that landed from one that was thrown away.
  sim_handle_message(ext::kSetCellProperty, 4, es_short);
  const GameDataUpdate* es_p1 = Tick(pb_world, &pb_vis);
  Check(Refusals(es_p1).empty(),
        "the refusal is NOT in the tick the message was sent during -- it is produced by the "
        "drain, and the drain runs a frame later");
  const GameDataUpdate* es_p2 = Tick(pb_world, &pb_vis);
  const std::vector<ext::ExtRefusedMessage> es_recs = Refusals(es_p2);
  Check(es_recs.size() == 1, "it is in the next one, exactly once");
  if (es_recs.size() == 1) {
    Check(es_recs[0].messageId == ext::kSetCellProperty &&
              es_recs[0].reason == ext::kExtRefusalShortPayload &&
              es_recs[0].payloadBytes == 4 &&
              es_recs[0].expectedBytes ==
                  static_cast<int32_t>(sizeof(ext::SetCellPropertyMessage)),
          "and it names the message, the reason, what arrived and what was needed -- "
          "\"something was dropped\" without those four numbers is not actionable");
  }

  const GameDataUpdate* es_p3 = Tick(pb_world, &pb_vis);
  Check(Refusals(es_p3).empty(),
        "records describe ONE tick: the buffer is cleared by the publish that carried them, "
        "so the next frame starts empty rather than accumulating");

  // THE ONE HANDLER-LEVEL REFUSAL THAT IS WIRED. A well-formed message aimed at a property
  // that was never registered used to vanish without trace, which is precisely the mistake
  // the registry exists to make visible.
  SetCellProperty(0, 999, 0u);
  Tick(pb_world, &pb_vis);
  const GameDataUpdate* es_bad = Tick(pb_world, &pb_vis);
  Check(HasRefusal(Refusals(es_bad), ext::kSetCellProperty, ext::kExtRefusalBadTarget),
        "a full-length kSetCellProperty naming an unregistered property is reported as "
        "kExtRefusalBadTarget rather than silently dropped");

  SetCellProperty(0, pb_vec, 0u, 7);
  Tick(pb_world, &pb_vis);
  const GameDataUpdate* es_lane = Tick(pb_world, &pb_vis);
  Check(HasRefusal(Refusals(es_lane), ext::kSetCellProperty, ext::kExtRefusalBadTarget),
        "so is a write to lane 7 of an arity-3 property -- landing on lane 0 instead would "
        "return a plausible number, which is the worst kind of wrong");

  // AN UNKNOWN MESSAGE ID, AND THE REASON THE CLEAR MOVED. This one is refused on the GAME
  // thread, between frames, on the immediate-message path -- so it is emitted when nothing is
  // running. Klei's ten vectors are cleared at the TOP of a frame; a stream cleared there
  // would wipe this record before any publish could carry it, and the one diagnostic whose
  // whole job is to stop a silent drop would silently drop it. Clearing at the END of the
  // publish instead is what makes this arm pass, and it is the only reason that rule exists.
  const int32_t es_bogus_id = 0x5A5A5A01;
  const uint8_t es_junk[4] = {0, 0, 0, 0};
  sim_handle_message(es_bogus_id, 4, es_junk);
  Tick(pb_world, &pb_vis);
  const GameDataUpdate* es_unk = Tick(pb_world, &pb_vis);
  Check(HasRefusal(Refusals(es_unk), es_bogus_id, ext::kExtRefusalUnknownMessage),
        "an unknown id refused between frames on the GAME thread still reaches a published "
        "frame -- it survives the frame boundary it was produced across");
  // This arm first asserted ONE tick, on the reasoning that nothing had queued the message,
  // and failed. It is two, and the ABI is right: `PrepareGameData` hands back
  // `last_published` -- the frame the worker finished -- and only THEN kicks the next one, so
  // every published fact costs one tick of publish latency whichever path produced it. The
  // immediate path is not a tick faster than the queued one; it skips the drain, not the
  // pipeline.

  // UNSUBSCRIBING STOPS THE COLLECTOR, not just the copy.
  SubscribeStream(es_idx, false);
  Tick(pb_world, &pb_vis);
  Tick(pb_world, &pb_vis);
  sim_handle_message(ext::kSetCellProperty, 4, es_short);
  Tick(pb_world, &pb_vis);
  const GameDataUpdate* es_off = Tick(pb_world, &pb_vis);
  es_n = -1;
  Check(sim_ext_events(es_off, &es_n) != nullptr && es_n == 0 &&
            FindStream(es_off, ext::kStreamMessageRefused) == nullptr,
        "after unsubscribing the stream is gone from the table and the refusal it would have "
        "carried was never collected");

  // ---- PER-ELEMENT ATTRIBUTES.
  //
  // The third registry, and the one that is not per-cell and not per-frame. Its storage is
  // SPARSE, nothing in it is ever saved, registration never closes, and both of its messages
  // are immediate -- four differences from the per-cell registry, each because an element
  // attribute is content data rather than world state. The migration under test is
  // `sim.molecular_mass`, which until today was a private vector on `ElementTable` with one
  // hardcoded key, a write-only message, and no read-back path at all.
  printf("\n=== vftest: extension registry -- per-element attributes ===\n");

  const int32_t ea_mol = sim_ext_attr_index(ext::kAttrMolecularMass);
  Check(ea_mol >= 0,
        "the first-party attribute sim.molecular_mass resolves by name -- the molecular-mass "
        "override is an ordinary registered attribute now, not a hardcoded vector");
  Check(sim_ext_attr_index("mod9.nosuchattribute") == -1,
        "an unregistered name resolves to -1");

  // THE FOUR SEEDED DIATOMIC CORRECTIONS, read back through the public export for the first
  // time. Before stage 5 there was no way to ask the sim what molar mass it was using.
  Check(ReadElementAttributeF32(ea_mol, kOxygenHash) == 31.9988f,
        "Oxygen's seeded correction reads back as O2's 31.9988, not Klei's atomic 15.9994");
  Check(ReadElementAttributeF32(ea_mol, simhost::kHydrogen) == 2.01588f,
        "Hydrogen's seeded correction reads back as H2's 2.01588");

  // UNSET IS NOT ZERO, and this is the arm the missing `defaultBits` field exists for. CO2's
  // Klei figure is already right for the molecule, so it has no override -- and the right
  // answer to "what is the override" is "there is none", never 0, because the fallback is
  // `Element::molarMass` and only `MolecularMassOf` knows that.
  uint32_t ea_bits = 0xDEADBEEF;
  Check(sim_ext_attr(ea_mol, simhost::kCarbonDioxide, 0, &ea_bits) == 0 &&
            ea_bits == 0xDEADBEEF,
        "an element with no override reports UNSET and leaves the caller's buffer alone -- "
        "unset is a different answer from an override of zero");

  // The ONI7 alias still works, writes the same attribute, and is IMMEDIATE. These two arms
  // failed first, asserting after one `Tick` on the assumption that the message was still
  // queued -- which it was, and which was a race: this store is now read by exports that take
  // no worker barrier, so a message writing it from the worker thread had to move. It did,
  // and the arms below assert the stronger property with no tick at all.
  ext::SetMolecularMassMessage ea_alias{simhost::kCarbonDioxide, 44.01f};
  sim_handle_message(ext::kSetMolecularMass, static_cast<int>(sizeof(ea_alias)),
                     reinterpret_cast<const uint8_t*>(&ea_alias));
  Check(ReadElementAttributeF32(ea_mol, simhost::kCarbonDioxide) == 44.01f,
        "kSetMolecularMass still works, now writes sim.molecular_mass, and lands before the "
        "call returns -- a convenience alias, not a second store and not a deferred one");
  ea_alias.gPerMol = 0.0f;
  sim_handle_message(ext::kSetMolecularMass, static_cast<int>(sizeof(ea_alias)),
                     reinterpret_cast<const uint8_t*>(&ea_alias));
  Check(sim_ext_attr(ea_mol, simhost::kCarbonDioxide, 0, nullptr) == 0,
        "and its <= 0 spelling of \"remove this override\" clears the entry rather than "
        "storing a zero");

  // THE ONLY ARM THAT CAN SEE THIS. If `ElementTable::Load` cleared every override, a second
  // element-table load in one session would silently throw away whatever a mod had pushed,
  // even though the values are keyed on the SimHashes id precisely so they survive it.
  SetElementAttributeF32(ea_mol, simhost::kGranite, 101.5f);
  Check(ReadElementAttributeF32(ea_mol, simhost::kGranite) == 101.5f,
        "a pushed molecular mass lands immediately -- the element-attribute path is the "
        "immediate path, so there is no tick of latency to wait out");
  // And the published table says so too. The line above is the behaviour; this is
  // the claim a mod reads before it writes the code that depends on the behaviour. Checked
  // together on purpose -- a descriptor that drifts from what the sim actually does is worse
  // than no descriptor, because it is believed.
  Check(DeclaredDelivery(ext::kSetElementAttribute) == ext::kDeliveryImmediate,
        "and SIM_ExtMessageDescribe publishes kSetElementAttribute as IMMEDIATE -- the arm "
        "above is what that word means");
  SendRaw(SimMessageHash::Elements_CreateTable, t.elements);
  Check(ReadElementAttributeF32(ea_mol, simhost::kGranite) == 101.5f,
        "and it SURVIVES the element table being reloaded -- the claim MaterialProperties.cs "
        "was already making and the code was not keeping");
  Check(ReadElementAttributeF32(ea_mol, kOxygenHash) == 31.9988f,
        "while the seeded defaults are still there after that reload, not seeded twice or "
        "cleared away");
  SetElementAttributeBits(ea_mol, simhost::kGranite, 0, 0, /*clear=*/true);

  // ---- THIRD-PARTY REGISTRATION.
  const int32_t ea_mod = RegisterElementAttribute("mod9.reactivity", ext::kExtF32, 1);
  Check(ea_mod >= 0, "a third-party attribute registers and the index comes back immediately");
  Check(sim_ext_attr_index("mod9.reactivity") == ea_mod,
        "and resolves to the same index by name");
  Check(RegisterElementAttribute("mod9.reactivity", ext::kExtF32, 1) ==
            -static_cast<int32_t>(ext::kExtRegisterDuplicate),
        "a duplicate name is refused, not silently re-registered");
  Check(RegisterElementAttribute("sim.sneaky", ext::kExtF32, 1) ==
            -static_cast<int32_t>(ext::kExtRegisterReserved),
        "the sim. owner is reserved against the message path, the same as for a cell property");
  Check(RegisterElementAttribute("mod9.BadCase", ext::kExtF32, 1) ==
            -static_cast<int32_t>(ext::kExtRegisterBadName),
        "uppercase is rejected rather than down-cased -- one name, one store");
  Check(RegisterElementAttribute("nodotatall", ext::kExtF32, 1) ==
            -static_cast<int32_t>(ext::kExtRegisterBadName),
        "a name with no owner segment is refused");
  Check(RegisterElementAttribute("mod9.bogustype", 99, 1) ==
            -static_cast<int32_t>(ext::kExtRegisterBadType),
        "a type outside the enum is refused");
  Check(RegisterElementAttribute("mod9.bogusarity", ext::kExtF32, 0) ==
            -static_cast<int32_t>(ext::kExtRegisterBadArity),
        "arity 0 is refused");
  Check(RegisterElementAttributeUnterminated() ==
            -static_cast<int32_t>(ext::kExtRegisterBadName),
        "a name field with no terminator is REFUSED rather than truncated -- the name is how "
        "every later write and read finds the attribute again");

  // ---- WRITE / READ, INCLUDING THE ZERO THAT IS NOT UNSET.
  SetElementAttributeF32(ea_mod, simhost::kGranite, 2.5f);
  Check(ReadElementAttributeF32(ea_mod, simhost::kGranite) == 2.5f,
        "a third-party attribute round-trips through the registry with no tick in between");
  SetElementAttributeF32(ea_mod, simhost::kCarbonDioxide, 0.0f);
  ea_bits = 0xDEADBEEF;
  Check(sim_ext_attr(ea_mod, simhost::kCarbonDioxide, 0, &ea_bits) == 1 && ea_bits == 0,
        "an explicitly stored 0.0 reads back as SET with the value 0 -- the registry can hold "
        "a legitimate zero, which is why clearing is a flag and not a magic value");
  Check(sim_ext_attr(ea_mod, kOxygenHash, 0, nullptr) == 0,
        "and an element nobody wrote is still unset in the same attribute");

  // ---- ARITY.
  const int32_t ea_vec = RegisterElementAttribute("mod9.triple", ext::kExtI32, 3);
  Check(ea_vec >= 0, "an arity-3 attribute registers");
  SetElementAttributeBits(ea_vec, simhost::kGranite, 2, 77);
  ea_bits = 0xDEADBEEF;
  Check(sim_ext_attr(ea_vec, simhost::kGranite, 2, &ea_bits) == 1 && ea_bits == 77,
        "writing lane 2 alone lands in lane 2");
  Check(sim_ext_attr(ea_vec, simhost::kGranite, 0, &ea_bits) == 1 && ea_bits == 0,
        "and the lanes nobody wrote read back as SET and zero, not as garbage -- a partially "
        "written entry is still a whole entry");
  Check(sim_ext_attr(ea_vec, simhost::kGranite, 3, nullptr) == 0,
        "lane 3 of an arity-3 attribute is refused, not folded onto lane 0");

  // ---- CLEAR REMOVES THE WHOLE ENTRY, every lane at once.
  SetElementAttributeBits(ea_vec, simhost::kGranite, 0, 0, /*clear=*/true);
  Check(sim_ext_attr(ea_vec, simhost::kGranite, 2, nullptr) == 0 &&
            sim_ext_attr(ea_vec, simhost::kGranite, 0, nullptr) == 0,
        "clearing removes every component of that element at once, back to unset");

  // ---- REFUSALS REACH STAGE 4'S STREAM, and the two cases that are NOT refusals do not.
  SubscribeStream(es_idx, true);
  Tick(pb_world, &pb_vis);
  Tick(pb_world, &pb_vis);

  const uint8_t ea_short[4] = {9, 9, 9, 9};
  sim_handle_message(ext::kSetElementAttribute, 4, ea_short);
  Tick(pb_world, &pb_vis);
  const GameDataUpdate* ea_f1 = Tick(pb_world, &pb_vis);
  Check(HasRefusal(Refusals(ea_f1), ext::kSetElementAttribute, ext::kExtRefusalShortPayload),
        "a short kSetElementAttribute payload is a recorded refusal, not a silent return");

  SetElementAttributeF32(999, simhost::kGranite, 1.0f);
  Tick(pb_world, &pb_vis);
  const GameDataUpdate* ea_f2 = Tick(pb_world, &pb_vis);
  Check(HasRefusal(Refusals(ea_f2), ext::kSetElementAttribute, ext::kExtRefusalBadTarget),
        "so is a write to an unregistered attribute index");

  SetElementAttributeBits(ea_vec, simhost::kGranite, 7, 1);
  Tick(pb_world, &pb_vis);
  const GameDataUpdate* ea_f3 = Tick(pb_world, &pb_vis);
  Check(HasRefusal(Refusals(ea_f3), ext::kSetElementAttribute, ext::kExtRefusalBadTarget),
        "and so is a write to lane 7 of an arity-3 attribute");

  // Clearing an element that had no entry asks for it to be unset, and it is unset. That is
  // the caller getting what they asked for, so it is deliberately NOT a refusal -- otherwise
  // an idempotent teardown loop would fill the diagnostic stream with its own success.
  SetElementAttributeBits(ea_mod, kVacuumHash, 0, 0, /*clear=*/true);
  Tick(pb_world, &pb_vis);
  const GameDataUpdate* ea_f4 = Tick(pb_world, &pb_vis);
  Check(!HasRefusal(Refusals(ea_f4), ext::kSetElementAttribute, ext::kExtRefusalBadTarget),
        "clearing an element that had no entry is not a refusal -- the caller asked for unset "
        "and got unset");

  // A REGISTRATION refusal is NOT streamed, and that asymmetry is deliberate: registration
  // hands its refusal straight back as a return value the caller must already read to learn
  // the index, so a stream record would be a second copy of an answer nobody missed.
  RegisterElementAttribute("mod9.reactivity", ext::kExtF32, 1);
  Tick(pb_world, &pb_vis);
  const GameDataUpdate* ea_f5 = Tick(pb_world, &pb_vis);
  Check(!HasRefusal(Refusals(ea_f5), ext::kRegisterElementAttribute,
                    ext::kExtRefusalBadTarget) &&
            !HasRefusal(Refusals(ea_f5), ext::kRegisterElementAttribute,
                        ext::kExtRefusalUnknownMessage),
        "a refused registration is answered by its return value and does not also become a "
        "stream record");

  // ---- THE REGISTRY AS METADATA.
  //
  // The descriptor table above is a read window and carries no registered default, on purpose.
  // A consumer that PAINTS a property needs one anyway -- zero is a legal value for all four
  // scalar types, so "still at the default" is not a question the bytes answer by themselves --
  // and the only other place that number lived was SIM_DebugExtCellStateAll, a worker barrier
  // and a full serialisation of the world. These two exports are the metadata half.
  printf("\n=== vftest: extension registry -- describe and enumerate ===\n");
  sim_shutdown();
  InitSim();

  // Distinct in every field the descriptor reports, so an offset wrong by one lands on a
  // value that cannot be mistaken for the right one. That is the failure mode a marshalling
  // layer actually has, and a table of zeroes would hide it.
  const int32_t md_a = RegisterProperty("mod10.warm", ext::kExtF32, ext::kSaved, BitsOf(293.15f));
  const int32_t md_b = RegisterProperty("mod10.flags", ext::kExtU8, ext::kRehydrated, 7u,
                                        /*arity=*/2);
  const int32_t md_c = RegisterProperty("mod10.count", ext::kExtI32, ext::kCheckpointOnly,
                                        0xFFFFFFFFu);
  Check(md_a >= 0 && md_b >= 0 && md_c >= 0, "three properties register for stage 3b");

  Check(sim_ext_property_count() >= 3,
        "the registry can be ENUMERATED -- SIM_ExtCellPropertyIndex answers only for a name "
        "the caller already had, which is no use to a client discovering what a world holds");

  // BEFORE the allocate. Registration is open until the first world is created, so this is a
  // state a caller can really observe, and a cellCount of 0 is the honest answer rather than
  // a guess at what the world will be.
  ext::ExtCellPropertyDesc md{};
  Check(sim_ext_property_describe(md_a, &md) == 1 && md.cellCount == 0,
        "a property described before World::Allocate reports cellCount 0 -- registered, not "
        "yet sized");

  World md_world;
  md_world.Init(8, 4, oxygen, 1.0f, 300.0f);
  Boot(t, md_world);
  std::vector<uint8_t> md_vis;
  Tick(md_world, &md_vis);

  Check(sim_ext_property_describe(md_a, &md) == 1, "mod10.warm describes");
  printf("  mod10.warm: type %d persist %d arity %d stride %d cells %d default 0x%08x pub %d\n",
         md.type, md.persist, md.arity, md.stride, md.cellCount, md.defaultBits, md.published);
  Check(strcmp(md.name, "mod10.warm") == 0, "the name comes back NUL-terminated and whole");
  Check(md.type == ext::kExtF32 && md.persist == ext::kSaved && md.arity == 1 && md.stride == 4,
        "type, persistence, arity and stride are the registered ones");
  Check(md.defaultBits == BitsOf(293.15f),
        "and the REGISTERED DEFAULT round-trips as raw bits -- the whole reason this export "
        "exists, since the published descriptor deliberately omits it");
  Check(md.cellCount == 10 * 6,
        "cellCount is PADDED cells (the storage's), matching ExtPublishedProperty and the save "
        "blob, not the 32 game cells");
  Check(md.published == 0, "nothing is published until somebody subscribes");

  Check(sim_ext_property_describe(md_b, &md) == 1 && md.type == ext::kExtU8 && md.arity == 2 &&
            md.stride == 1 && md.defaultBits == 7u && md.persist == ext::kRehydrated,
        "an arity-2 u8 property reports its own shape and its own default, not the previous "
        "call's");
  Check(sim_ext_property_describe(md_c, &md) == 1 && md.type == ext::kExtI32 &&
            md.defaultBits == 0xFFFFFFFFu && md.persist == ext::kCheckpointOnly,
        "and an i32 default is carried VERBATIM -- -1 is a legal default and must not be "
        "clamped, sign-extended or reinterpreted on the way out");

  ext::ExtCellPropertyDesc md_untouched{};
  md_untouched.type = 0x5A5A5A5A;
  Check(sim_ext_property_describe(-1, &md_untouched) == 0 &&
            md_untouched.type == 0x5A5A5A5A,
        "an out-of-range index returns 0 and LEAVES THE BUFFER ALONE -- a caller that ignores "
        "the return must not find a plausible descriptor sitting in it");
  Check(sim_ext_property_describe(sim_ext_property_count(), &md_untouched) == 0,
        "one past the end is out of range too");
  Check(sim_ext_property_describe(md_a, nullptr) == 0, "a null buffer is refused, not written");

  // THE SUBSCRIPTION FLAG, which is the one field here that changes between two calls. It is
  // what lets a route ask "is this already published" without binding a frame first.
  PublishProperty(md_a, true);
  Check(sim_ext_property_describe(md_a, &md) == 1 && md.published == 0,
        "kPublishCellProperty is QUEUED, so a describe immediately after subscribing still "
        "reports 0 -- the same one-frame latency every cell message has, not a failure");
  Check(DeclaredDelivery(ext::kPublishCellProperty) == ext::kDeliveryQueued,
        "and the delivery table publishes kPublishCellProperty as QUEUED -- the latency the arm "
        "above just demonstrated, exported instead of left in a comment");
  Tick(md_world, &md_vis);
  Check(sim_ext_property_describe(md_a, &md) == 1 && md.published == 1,
        "and it reports 1 on the next tick");
  Check(sim_ext_property_describe(md_b, &md) == 1 && md.published == 0,
        "the property nobody subscribed to still reports 0");
  PublishProperty(md_a, false);
  Tick(md_world, &md_vis);
  Check(sim_ext_property_describe(md_a, &md) == 1 && md.published == 0,
        "unsubscribing clears it");

  // ---- THE STREAM REGISTRY AS METADATA.
  //
  // Runs on the sim the registry-metadata arms left booted, deliberately: the declaration set is written inside
  // SIM_Initialize and is immutable afterwards, so there is nothing about a fresh sim these
  // arms would see differently, and a shutdown here would cost a boot for no assertion.
  //
  // The claim being gated is DISCOVERY. `SIM_ExtEventStreamIndex` answers only for a name the
  // caller already typed, and the published table carries subscribed streams only -- so the
  // arm that matters most is the pair at the end, where a stream is describable while it is
  // absent from the published table. That is the state every stream is in before anybody has
  // asked for it, and it is the state a generic client has to be able to see.
  printf("\n=== vftest: event streams -- describe and enumerate ===\n");
  // Three since Layer C: the refusal stream plus the two liquid-payload streams,
  // declared in that order.
  Check(sim_ext_stream_count() == 3,
        "this DLL declares exactly three event streams; a fourth landing without this arm "
        "being updated should be a failing test, not a silent pass");

  ext::ExtEventStreamDesc sd{};
  Check(sim_ext_stream_describe(0, &sd) == 1, "stream 0 describes");
  Check(strcmp(sd.name, ext::kStreamMessageRefused) == 0,
        "and it is the refusal stream, by the name the header owns");
  Check(sd.stride == static_cast<int32_t>(sizeof(ext::ExtRefusedMessage)),
        "stride is the declared record size, 16, not a byte count of anything this frame");
  Check(sd.declaredIdx == 0, "declaredIdx is the index that was passed in");
  Check(sim_ext_stream_index(sd.name) == sd.declaredIdx,
        "and it round-trips through SIM_ExtEventStreamIndex -- which is the whole point of "
        "carrying it: a descriptor that has been copied onto a wire is no longer beside the "
        "loop counter that produced it");
  Check(sd.subscribed == 0, "nothing has subscribed yet");

  // Out of range, both ends, and the buffer left alone. Same discipline as stage 3b: a caller
  // that ignores the return value must not find a plausible descriptor sitting in the buffer.
  ext::ExtEventStreamDesc sd_untouched{};
  memset(&sd_untouched, 0x5A, sizeof(sd_untouched));
  Check(sim_ext_stream_describe(-1, &sd_untouched) == 0, "a negative index is refused");
  Check(sd_untouched.stride == 0x5A5A5A5A,
        "and the buffer is untouched by the refusal");
  Check(sim_ext_stream_describe(sim_ext_stream_count(), &sd_untouched) == 0,
        "one past the end is out of range too");
  Check(sd_untouched.stride == 0x5A5A5A5A, "still untouched");
  Check(sim_ext_stream_describe(0, nullptr) == 0, "a null buffer is refused, not written");

  // THE SUBSCRIPTION FLAG, and the one-frame latency it inherits from the queued message.
  SubscribeStream(0, true);
  Check(sim_ext_stream_describe(0, &sd) == 1 && sd.subscribed == 0,
        "kSubscribeEventStream is QUEUED, so a describe immediately after subscribing still "
        "reports 0");
  Check(DeclaredDelivery(ext::kSubscribeEventStream) == ext::kDeliveryQueued,
        "and the delivery table publishes kSubscribeEventStream as QUEUED");
  const GameDataUpdate* sd_frame = Tick(md_world, &md_vis);
  Check(sim_ext_stream_describe(0, &sd) == 1 && sd.subscribed == 1,
        "and it reports 1 on the next tick");
  // THE DESCRIPTOR FLIPS ONE FRAME BEFORE THE PUBLISHED TABLE DOES, and that is not a wobble
  // to paper over -- it is the ABI's own sequence made visible. The drain that sets the flag
  // runs inside tick N; the table tick N publishes was built from what was subscribed when the
  // frame began. So the stream is `subscribed` on tick N and IN THE TABLE on tick N+1, which
  // is exactly what `kSubscribeEventStream`'s "first collects on tick N+1" means. A client
  // that subscribes, sees the flag, and concludes the bytes are already there is the caller
  // this arm exists to contradict.
  Check(FindStream(sd_frame, ext::kStreamMessageRefused) == nullptr,
        "the frame whose own drain set the flag does NOT yet carry the stream");
  sd_frame = Tick(md_world, &md_vis);
  Check(FindStream(sd_frame, ext::kStreamMessageRefused) != nullptr,
        "the next frame does, with zero records -- present and empty, which is a different "
        "fact from absent");

  SubscribeStream(0, false);
  sd_frame = Tick(md_world, &md_vis);
  Check(sim_ext_stream_describe(0, &sd) == 1 && sd.subscribed == 0,
        "unsubscribing clears the flag on the tick that drains it");
  sd_frame = Tick(md_world, &md_vis);
  Check(FindStream(sd_frame, ext::kStreamMessageRefused) == nullptr,
        "and the frame after that drops it out of the published table -- so the published "
        "table alone cannot tell an undeclared stream from an unsubscribed one, which is "
        "exactly why Describe exists");
  Check(sim_ext_stream_describe(0, &sd) == 1 && strcmp(sd.name, ext::kStreamMessageRefused) == 0,
        "while Describe still names it: declared and unsubscribed is a visible state");

  // ---- THE ELEMENT-ATTRIBUTE REGISTRY AS METADATA.
  //
  // Runs on the sim the two metadata arms above left booted. The claim being gated is the same one, for
  // the last of the three registries: `SIM_ExtElementAttributeIndex` answers only for a name
  // the caller already typed, and `SIM_ExtElementAttribute` needs an element id as well, so
  // before these three exports a tool handed this DLL could not find out that registry 2 holds
  // anything at all -- not even the sim's own `sim.molecular_mass`.
  //
  // THE SPARSENESS IS WHAT THE ARMS ARE ABOUT. A per-cell property has a value for every cell;
  // an attribute has one for a handful of elements out of two hundred. So `valueCount` and the
  // key list are the descriptor's real content, and the arms below assert them across a write,
  // a second write to the same element, and a clear -- because those three move `valueCount`,
  // `writes` and `keys` in three different combinations.
  printf("\n=== vftest: element attributes -- describe and enumerate ===\n");

  Check(sim_ext_attr_count() == 6,
        "a fresh sim declares exactly six element attributes -- sim.molecular_mass, "
        "sim.phase_curve, sim.liquid_density, sim.min_liquid_pressure, "
        "sim.can_condense and sim.latent_fusion -- all registered "
        "NATIVELY in World's constructor, which is why a "
        "discovery path built out of a managed-side ledger of registrations would see none at all");

  ext::ExtElementAttributeDesc ad{};
  Check(sim_ext_attr_describe(0, &ad) == 1, "attribute 0 describes");
  Check(strcmp(ad.name, ext::kAttrMolecularMass) == 0,
        "and it is sim.molecular_mass, by the name the header owns");
  Check(ad.firstParty == 1,
        "flagged first-party: the sim registered it, not a mod, and the \"sim.\" owner is "
        "reserved so no mod could have");
  Check(ad.type == ext::kExtF32 && ad.arity == 1 && ad.stride == 4,
        "f32, arity 1, four bytes a component");
  Check(ad.declaredIdx == 0, "declaredIdx is the index that was passed in");
  Check(sim_ext_attr_index(ad.name) == ad.declaredIdx,
        "and it round-trips through SIM_ExtElementAttributeIndex -- carried for the reason "
        "ExtEventStreamDesc carries one: a descriptor copied onto a wire is no longer beside "
        "the loop counter that produced it");
  Check(ad.valueCount == 4,
        "four elements carry a value: the diatomic corrections the sim seeds for itself. "
        "valueCount counts ELEMENTS WITH A VALUE, never elements");
  Check(ad.writes >= 4, "and the seeding shows up in the write counter");

  // The two phase-change attributes. Registered first-party at fixed indices because a native kernel reads them
  // (the conduit-run phase change), and deliberately NOT seeded: the vapour curve is the managed
  // registry's data, so a sim nobody has told has no phase change in conduit runs -- vanilla.
  ext::ExtElementAttributeDesc ad_curve{};
  Check(sim_ext_attr_describe(1, &ad_curve) == 1 &&
            strcmp(ad_curve.name, ext::kAttrPhaseCurve) == 0 && ad_curve.firstParty == 1 &&
            ad_curve.type == ext::kExtF32 && ad_curve.arity == ext::kPhaseCurveArity &&
            ad_curve.valueCount == 0,
        "attribute 1 is sim.phase_curve: first-party, f32, arity 5, and empty until the managed "
        "registry pushes a curve");
  ext::ExtElementAttributeDesc ad_density{};
  Check(sim_ext_attr_describe(2, &ad_density) == 1 &&
            strcmp(ad_density.name, ext::kAttrLiquidDensity) == 0 && ad_density.firstParty == 1 &&
            ad_density.type == ext::kExtF32 && ad_density.arity == 1 &&
            ad_density.valueCount == 0,
        "attribute 2 is sim.liquid_density: first-party, f32, arity 1, likewise empty");
  ext::ExtElementAttributeDesc ad_fusion{};
  Check(sim_ext_attr_describe(5, &ad_fusion) == 1 &&
            strcmp(ad_fusion.name, ext::kAttrLatentFusion) == 0 && ad_fusion.firstParty == 1 &&
            ad_fusion.type == ext::kExtF32 && ad_fusion.arity == 1 && ad_fusion.valueCount == 0,
        "attribute 5 is sim.latent_fusion: first-party, f32, arity 1, empty until pushed");

  // Distinct in every field the descriptor reports, so an offset wrong by one lands on a value
  // that cannot be mistaken for the right one -- the failure mode a marshalling layer actually
  // has, and one a table of identical attributes would hide. Same discipline as the registry-metadata arm.
  const int32_t ad_a = RegisterElementAttribute("mod11.latent", ext::kExtF32, 1);
  const int32_t ad_b = RegisterElementAttribute("mod11.curve", ext::kExtI32, 3);
  // 6 and 7: a mod's attributes land
  // after whatever the sim registered natively, so these indices move whenever the sim gains
  // one. That is exactly why a caller resolves an index by NAME through
  // SIM_ExtElementAttributeIndex instead of hardcoding it, and this arm moving is the reminder
  // rather than a break.
  Check(ad_a == 6 && ad_b == 7, "two mod attributes register, at the next two indices");
  Check(sim_ext_attr_count() == 8, "and the count follows them -- registration never closes");

  Check(sim_ext_attr_describe(ad_b, &ad) == 1, "the arity-3 i32 attribute describes");
  Check(strcmp(ad.name, "mod11.curve") == 0 && ad.type == ext::kExtI32 && ad.arity == 3 &&
            ad.stride == 4 && ad.declaredIdx == ad_b,
        "with its own name, type, arity and index -- not the previous attribute's");
  Check(ad.firstParty == 0,
        "and NOT first-party, which is the field that tells a client whose contract it is "
        "reading: the sim's, or some mod's");
  Check(ad.valueCount == 0 && ad.writes == 0,
        "registered and never written: no elements, no writes. That pair is the whole reason "
        "`writes` is on the descriptor -- see the clear below, where valueCount returns to 0 "
        "and writes does not");

  // KEYS. The sizing call first, because a client that cannot size its buffer cannot use the
  // export at all.
  Check(sim_ext_attr_keys(ad_b, nullptr, 0) == 0,
        "an attribute nobody has written has no keys, and asking costs nothing");

  SetElementAttributeBits(ad_b, kOxygenHash, 0, 11u);
  SetElementAttributeBits(ad_b, kOxygenHash, 2, 33u);
  SetElementAttributeBits(ad_b, simhost::kCarbonDioxide, 1, 22u);
  Check(sim_ext_attr_describe(ad_b, &ad) == 1 && ad.valueCount == 2 && ad.writes == 3,
        "three writes across two elements: valueCount counts the ELEMENTS and writes counts "
        "the messages, and the two are deliberately not the same number");

  int32_t ad_keys[8];
  memset(ad_keys, 0x5A, sizeof(ad_keys));
  Check(sim_ext_attr_keys(ad_b, nullptr, 0) == 2,
        "the sizing call reports the total with no buffer at all");
  Check(sim_ext_attr_keys(ad_b, ad_keys, 8) == 2, "and the filling call reports the same total");
  Check(ad_keys[0] < ad_keys[1],
        "the two keys come back ASCENDING as signed ids -- the registry stores them sorted for "
        "its own lookup, so this is a copy and not a sort, and a client diffing two of these "
        "does not have to sort first");
  Check((ad_keys[0] == kOxygenHash && ad_keys[1] == simhost::kCarbonDioxide) ||
            (ad_keys[0] == simhost::kCarbonDioxide && ad_keys[1] == kOxygenHash),
        "and they are the two elements that were written, by SimHashes id -- an id and never a "
        "table index, so a value survives the element table being reloaded");
  Check(ad_keys[2] == 0x5A5A5A5A, "nothing was written past the total");

  // A SHORT BUFFER TRUNCATES AND STILL REPORTS THE TOTAL, which is what makes the two-call
  // pattern work: the return value sizes the buffer rather than reporting the copy.
  int32_t ad_one[2];
  memset(ad_one, 0x5A, sizeof(ad_one));
  Check(sim_ext_attr_keys(ad_b, ad_one, 1) == 2,
        "a buffer with room for one of two keys still returns 2 -- the total, not the copy");
  Check(ad_one[0] == ad_keys[0] && ad_one[1] == 0x5A5A5A5A,
        "it copied the first key and stopped at the caller's max");

  // THE CLEAR. valueCount goes back to where it started; writes does not, and that asymmetry
  // is the whole answer to "did nobody push this, or did the pusher clear it".
  SetElementAttributeBits(ad_b, kOxygenHash, 0, 0, /*clear=*/true);
  Check(sim_ext_attr_describe(ad_b, &ad) == 1 && ad.valueCount == 1 && ad.writes == 4,
        "clearing an element drops it out of valueCount and INCREMENTS writes");
  Check(sim_ext_attr_keys(ad_b, ad_keys, 8) == 1, "and out of the key list");
  SetElementAttributeBits(ad_b, ad_keys[0], 0, 0, /*clear=*/true);
  Check(sim_ext_attr_describe(ad_b, &ad) == 1 && ad.valueCount == 0 && ad.writes == 5,
        "cleared back to empty, and the descriptor now says what no cell-property descriptor "
        "ever has to: nobody holds a value, but somebody did -- writes is 5, not 0");

  // Out of range, both ends, a null buffer, and the buffer left alone. Same discipline as
  // stages 3b and 4b: a caller that ignores the return value must not find a plausible
  // descriptor sitting in its buffer.
  ext::ExtElementAttributeDesc ad_untouched{};
  memset(&ad_untouched, 0x5A, sizeof(ad_untouched));
  Check(sim_ext_attr_describe(-1, &ad_untouched) == 0, "a negative index is refused");
  Check(ad_untouched.arity == 0x5A5A5A5A, "and the buffer is untouched by the refusal");
  Check(sim_ext_attr_describe(sim_ext_attr_count(), &ad_untouched) == 0,
        "one past the end is out of range too");
  Check(ad_untouched.arity == 0x5A5A5A5A, "still untouched");
  Check(sim_ext_attr_describe(0, nullptr) == 0, "a null buffer is refused, not written");

  memset(ad_keys, 0x5A, sizeof(ad_keys));
  Check(sim_ext_attr_keys(-1, ad_keys, 8) == 0, "keys for a negative index is 0");
  Check(sim_ext_attr_keys(sim_ext_attr_count(), ad_keys, 8) == 0,
        "and 0 one past the end -- indistinguishable from an attribute nobody has written, "
        "deliberately: both mean there is nothing to read");
  Check(ad_keys[0] == 0x5A5A5A5A, "with the caller's buffer left alone either way");

  // =========================================================================================
  // SIM_DebugCellFlow, the flow accumulator published.
  //
  // The flow accumulator is the only per-cell field in the sim that records a TRANSFER rather
  // than a state, and until this export nothing outside the DLL could read it: it is written
  // by `World::AddFlow` and read by `FillFlowTexture` and by nothing else. What comes back is
  // the raw signed kilograms moved in the frame that just finished, NOT the texture's
  // normalised number -- see the export's own comment for why that difference is deliberate.
  //
  // A fresh world on purpose. Every earlier scenario in this file is uniform-mass or
  // gas-mixture driven, and vanilla `StepGasPressure` moves nothing across a flat gradient,
  // so an arm asserting "flow is non-empty" against one of those would be asserting a
  // coincidence. This one loads a heavy end cell and lets ordinary vanilla physics do it.
  printf("\n=== vftest: SIM_DebugCellFlow -- the flow accumulator, published ===\n");
  {
    // Before a world exists. The export reads world state and takes the barrier, unlike the
    // phase and message tables, so "no world" is a refusal rather than an empty answer -- and
    // the two are indistinguishable from outside, deliberately: both mean nothing moved.
    sim_shutdown();
    Check(sim_debug_cell_flow(nullptr, nullptr, nullptr, nullptr, 0) == 0,
          "with no sim there is no flow to report, and asking does not crash");

    InitSim();
    World fw;
    // 8x4 AND NOT 8x1, and the reason is vanilla behaviour rather than a preference or a
    // defect. A one-row world reports ZERO flow on every one of
    // ten ticks, for this reason.
    //
    // The game sends `maxY = height - 1` -- `Tick` above does the same because that is what
    // the game does, and `World::SetActiveRegions` says why: the world's top row is never
    // simulated in a real game, and a replacement that steps it anyway drifts from Klei on
    // every world whose top row is not uniform (`toprow` is the scenario that measured it).
    // On a ONE-ROW world the only row IS the top row. `PaddedRegion` then comes out
    // `{1,1,9,1}` -- y0 >= y1, an empty driving rectangle -- while `PaddedRegionInclusive`
    // still covers the row at `{1,1,9,2}`. Every kernel that takes `PaddedRegion`, the gas
    // pressure sweep included, iterates nothing; `StepPostProcess`, the one sweep that takes
    // the inclusive rectangle, still runs. Nothing that writes the flow accumulator runs at
    // all, so the accumulator is empty -- correctly.
    //
    // Measured three ways. Same scenario at heights 1..5, six ticks each: 8x1 reports
    // 0,0,0,0,0,0 and 8x2 reports 4,7,10,11,15,14. Re-sent with `maxY = height` instead, the
    // same 8x1 world reports 3,5,6,7,8,8 and every height >= 2 is UNCHANGED -- which is the
    // control, since dropping the top row can only matter to a world whose top row is its
    // only one. And a standalone harness that links `StepGasPressure` directly and runs it
    // over the default full-interior region records 3 cells on tick 0 of an 8x1 world, so
    // the kernel is not the thing that is height-sensitive.
    //
    // TWO THINGS ARE TRUE AT ONCE about the original one-row attempt: the `fw.mass[2 * 8 + 2]`
    // below is also out of range on an 8x1 world (`mass` holds 8 floats there), so that
    // attempt planted no gradient either. Planting it correctly still reports zero.
    //
    // The by-catch is worth more than the answer: on the 8x1 world the 20 kg does eventually
    // move -- a whole-cell swap one column left, several ticks in -- with the flow count
    // still 0. That is `GasShuffle` inside `StepPostProcess`, which is the inclusive sweep,
    // and it never touches the accumulator. "Mass moved" and "flow was published" are not
    // the same predicate, exactly as for a gas pair that only ever moves diagonally
    // (physics.h: Klei's diagonal call is the one of the three with no accumulate after it).
    fw.Init(8, 4, oxygen, 1.0f, 300.0f);
    fw.mass[2 * 8 + 2] = 20.0f;  // the gradient vanilla pressure actually moves mass across
    Boot(t, fw);
    std::vector<uint8_t> fvis;

    // ONE tick, not sixty. The accumulator holds ONE FRAME of transfers -- `ClearFlow` runs
    // at the top of the physics frame, after the skip test -- so what is read here is the
    // frame that just finished and nothing before it. An arm that ticked sixty times and then
    // read would be asserting the same thing about the sixtieth frame while looking like it
    // was asserting something cumulative.
    Tick(fw, &fvis);

    const int32_t sizing = sim_debug_cell_flow(nullptr, nullptr, nullptr, nullptr, 0);
    printf("  after one tick: %d cell(s) moved mass\n", sizing);
    Check(sizing > 0,
          "a world with a real mass gradient reports flow after one tick -- the accumulator "
          "is reachable from outside the DLL for the first time");
    Check(sizing <= 32,
          "and no more entries than the world has cells: the list is cells, not transfers, "
          "and AddFlow records a cell once however many times it writes it");

    std::vector<int32_t> fc(64, -1);
    std::vector<float> fx(64, 12345.0f), fy(64, 12345.0f);
    std::vector<uint16_t> fe(64, 0x1234);
    const int32_t got = sim_debug_cell_flow(fc.data(), fx.data(), fy.data(), fe.data(), 64);
    Check(got == sizing,
          "the sizing call and the filling call agree -- the return is the TOTAL either way, "
          "the same discipline SIM_ExtElementAttributeKeys holds");

    bool cells_ok = true, moved_ok = false, untouched_tail = true;
    for (int32_t i = 0; i < got; ++i) {
      if (fc[i] < 0 || fc[i] >= 32) cells_ok = false;
      if (fx[i] != 0.0f || fy[i] != 0.0f) moved_ok = true;
    }
    for (size_t i = static_cast<size_t>(got); i < fc.size(); ++i) {
      if (fc[i] != -1 || fx[i] != 12345.0f || fe[i] != 0x1234) untouched_tail = false;
    }
    Check(cells_ok,
          "every cell index is a valid GAME cell -- the accumulator is indexed by PADDED cell "
          "and the conversion is the export's job, not the caller's");
    Check(moved_ok, "at least one reported cell has a non-zero transfer");
    Check(untouched_tail,
          "and the export writes exactly `got` entries, leaving the rest of the caller's "
          "buffers alone");

    // WHY THE PIN IS GEOMETRY AND NOT THE FLOW TEXTURE, which would otherwise be the obvious
    // independent oracle: `FillFlowTexture` computes the same two differences, but the frame
    // the driver gets back on the FIRST tick publishes a flow texture of all zeros while the
    // accumulator read here is already full -- measured, 4 non-zero texture floats against 17
    // recorded cells on the next tick and 40 against 29 the tick after, so the texture is
    // real but is not the frame this call's accumulator belongs to. A cross-check between the
    // two would be comparing a frame against its neighbour. Not a defect and not chased
    // further; recorded so the next person does not spend the afternoon on it.
    //
    // THE SLOT ARITHMETIC, PINNED BY GEOMETRY. x is `[0] - [1]` and y is `[3] - [2]`;
    // crossing those two pairs is the single most likely mistake this export can make, and no
    // self-consistent check can see it. The world is built so the grid itself answers: one
    // heavy cell at (2,2) on an otherwise uniform 8x4, and the first frame pushes mass into
    // its four orthogonal neighbours and nowhere else. So the two HORIZONTAL neighbours must
    // report an x component and no y, and the two VERTICAL ones the reverse. Swap the pairs
    // in the export and all four of these fail at once.
    //
    // Their signs must also be opposite within each axis -- (1,2) and (3,2) received mass
    // moving in opposite directions -- which is what a duplicated slot or an absolute value
    // would break while leaving the axis test passing.
    auto find = [&](int32_t cell, float* x, float* y) {
      for (int32_t i = 0; i < got; ++i) {
        if (fc[i] != cell) continue;
        *x = fx[i];
        *y = fy[i];
        return true;
      }
      return false;
    };
    const int32_t src = 2 * 8 + 2, left = 2 * 8 + 1, right = 2 * 8 + 3;
    const int32_t up = 1 * 8 + 2, down = 3 * 8 + 2;
    float lx = 0, ly = 0, rx = 0, ry = 0, ux = 0, uy = 0, dx = 0, dy = 0, sx0 = 0, sy0 = 0;
    const bool all_four =
        find(left, &lx, &ly) && find(right, &rx, &ry) && find(up, &ux, &uy) &&
        find(down, &dx, &dy);
    Check(all_four,
          "the heavy cell's four orthogonal neighbours are all on the list after one frame -- "
          "the geometry the two checks below read the slot mapping out of");
    Check(all_four && lx != 0.0f && ly == 0.0f && rx != 0.0f && ry == 0.0f,
          "the two HORIZONTAL neighbours report an x component and no y -- x is [0]-[1]");
    Check(all_four && uy != 0.0f && ux == 0.0f && dy != 0.0f && dx == 0.0f,
          "the two VERTICAL neighbours report a y component and no x -- y is [3]-[2], and "
          "crossing the two pairs fails both of these at once");
    Check(all_four && ((lx > 0.0f) != (rx > 0.0f)) && ((uy > 0.0f) != (dy > 0.0f)),
          "and the signs are opposite within each axis: the sign is the direction the mass "
          "went, so a duplicated slot or an absolute value shows up here");

    // THE SOURCE CELL NETS TO ZERO AND IS STILL ON THE LIST, which is not a curiosity -- it
    // is the difference between "cells the accumulator wrote" and "cells with non-zero net
    // flow", and a consumer that filters on the latter would silently drop the cell the
    // transfer came FROM. It gave equally in all four directions, so [0]-[1] and [3]-[2] both
    // cancel exactly.
    Check(find(src, &sx0, &sy0) && sx0 == 0.0f && sy0 == 0.0f,
          "the cell the mass left is on the list with a net flow of zero -- the list is cells "
          "AddFlow wrote, not cells whose net is non-zero, and the two are different sets");

    // TRUNCATION. `max` smaller than the total fills `max` entries and still returns the
    // total, so a caller sizes its buffer from the return rather than from the copy. A caller
    // that treated the return as "how many I got" would walk off the end of its own array,
    // which is why this is checked rather than documented.
    std::vector<int32_t> tc(1, -1);
    std::vector<float> ttx(1, 0.0f), tty(1, 0.0f);
    std::vector<uint16_t> te(1, 0);
    const int32_t trunc = sim_debug_cell_flow(tc.data(), ttx.data(), tty.data(), te.data(), 1);
    Check(trunc == got,
          "a buffer too small still returns the TOTAL, not the number copied -- the return "
          "sizes the buffer");
    Check(tc[0] == fc[0], "and the entries it did write are the first ones, in list order");

    // THE CADENCE, which is the property most likely to be quietly broken by a future change
    // to where ClearFlow sits. One frame of transfers, not a running total.
    Tick(fw, &fvis);
    std::vector<int32_t> sc(64, -1);
    std::vector<float> sx(64, 0.0f), sy(64, 0.0f);
    std::vector<uint16_t> se(64, 0);
    const int32_t second = sim_debug_cell_flow(sc.data(), sx.data(), sy.data(), se.data(), 64);
    bool differs = second != got;
    for (int32_t i = 0; i < second && i < got && !differs; ++i) {
      if (sc[i] != fc[i] || sx[i] != fx[i]) differs = true;
    }
    Check(differs,
          "the second tick reports its OWN frame, not the first frame's numbers again -- the "
          "accumulator is cleared at the top of each physics frame and this is the only arm "
          "that would notice if it stopped being");

    // The refusals. A null in any output pointer collapses to the sizing call rather than
    // writing a partial answer into the pointers that ARE valid, because a caller that passed
    // three good buffers and one null has a bug and half-filling three of them hides it.
    Check(sim_debug_cell_flow(nullptr, sx.data(), sy.data(), se.data(), 64) == second,
          "a null cell buffer collapses to the sizing call rather than filling the others");
    std::vector<int32_t> nc(4, -7);
    Check(sim_debug_cell_flow(nc.data(), nullptr, sy.data(), se.data(), 4) == second,
          "and so does a null value buffer");
    Check(nc[0] == -7, "with the buffers that WERE valid left untouched");
    Check(sim_debug_cell_flow(sc.data(), sx.data(), sy.data(), se.data(), -1) == second,
          "a negative max is the sizing call too, not a wrap-around length");

    // AND THE ARM THAT ACTUALLY PINS THE CLEAR. "The two frames differ" is satisfied by an
    // accumulator that never clears at all, because the touched list would simply keep
    // growing -- proven by planting exactly that fault, which the check above passed. What
    // only a cleared buffer can do is report FEWER cells than the frame before it. A world
    // settling from one heavy cell reaches every cell and then oscillates (5, 17, 26, 29, 32,
    // 31, 32, ... measured), so a decrease shows up within a handful of ticks; without the
    // clear the list is monotonically non-decreasing and this can never pass.
    int32_t prev = second;
    bool decreased = false;
    for (int i = 0; i < 12 && !decreased; ++i) {
      Tick(fw, &fvis);
      const int32_t now = sim_debug_cell_flow(nullptr, nullptr, nullptr, nullptr, 0);
      if (now < prev) decreased = true;
      prev = now;
    }
    Check(decreased,
          "the reported cell count goes DOWN at least once over twelve ticks -- an "
          "accumulator that is never cleared cannot do that, and \"the two frames differ\" "
          "alone does not notice");
  }

  // THE TOP-ROW RULE -- why the arm above is 8x4 and not 8x1, asserted.
  //
  // A one-row world publishes no flow, and that is vanilla rather than a defect in the export
  // or in the kernel. The game sends `maxY = height - 1` because the world's top row is never
  // simulated in a real game (`World::SetActiveRegions`; `toprow` measured it). On a ONE-ROW
  // world the only row IS the top row, so `PaddedRegion` comes out `{1,1,9,1}` -- y0 >= y1, an
  // empty driving rectangle -- while `PaddedRegionInclusive` still covers the row at
  // `{1,1,9,2}`. Every kernel that writes the flow accumulator takes the first rectangle and
  // iterates nothing; `StepPostProcess`, the one sweep that takes the second, still runs and
  // never calls `AddFlow`.
  //
  // THREE CHECKS AND NOT ONE, AND THE COUNT IS THE POINT. "An 8x1 world reports zero" is
  // satisfied by an export that has been broken into returning zero for everything, which is
  // the same failure the cadence arm above already walked into once -- a claim that a planted
  // fault cannot break is not a check. So the zero is stated together with the two things that
  // give it its meaning:
  //
  //   (a) 8x1 under the region the GAME sends            -> no flow, on any tick
  //   (b) THE SAME 8x1 world with the top row included   -> flow
  //   (c) 8x2 under BOTH regions                         -> flow either way
  //
  // (b) is what turns (a) from "the export is dead" into "the region has no driving row": one
  // input differs, the top edge, and it is decisive. (c) is the control that stops the whole
  // thing being read as "dropping the top row breaks flow" -- dropping it is invisible unless
  // the top row is the world's only row, which is exactly the claim.
  //
  // Ten ticks rather than one for (a), and that is deliberate: `GasShuffle` inside
  // `StepPostProcess` does eventually move mass on this world -- a whole-cell swap, measured --
  // and it never calls `AddFlow`. Summing over ten ticks therefore says something stronger than
  // "the first frame was quiet": mass moved and nothing was published, which is the caveat
  // the ABI documentation carries. The swap's timing comes off the random stream, so it is not
  // asserted; the zero is.
  printf("\n=== vftest: SIM_DebugCellFlow -- a one-row world has no driving row ===\n");
  {
    auto flow_over = [&](int height, bool include_top_row) {
      sim_shutdown();
      InitSim();
      World tw;
      tw.Init(8, height, oxygen, 1.0f, 300.0f);
      // Row 0 in both worlds, so the seed is the same cell of the same shape either way and
      // the only thing that differs between the four runs is the world's height and the
      // region's top edge. `2 * 8 + 2` -- the 8x4 arm's index -- is out of range on an 8x1
      // world, which is the second fault the original one-row attempt carried.
      tw.mass[2] = 20.0f;
      Boot(t, tw);
      std::vector<uint8_t> tvis;
      int32_t total = 0;
      for (int k = 0; k < 10; ++k) {
        if (include_top_row) {
          TickTopRowIncluded(tw, &tvis);
        } else {
          Tick(tw, &tvis);
        }
        total += sim_debug_cell_flow(nullptr, nullptr, nullptr, nullptr, 0);
      }
      return total;
    };

    const int32_t one_row_game = flow_over(1, false);
    const int32_t one_row_top = flow_over(1, true);
    const int32_t two_row_game = flow_over(2, false);
    const int32_t two_row_top = flow_over(2, true);
    printf("  ten ticks, cells reported: 8x1 game-region %d, 8x1 top-row-included %d, "
           "8x2 game-region %d, 8x2 top-row-included %d\n",
           one_row_game, one_row_top, two_row_game, two_row_top);

    Check(one_row_game == 0,
          "a ONE-ROW world reports no flow on any of ten ticks under the region the game "
          "actually sends -- its only row is the top row, which never drives");
    Check(one_row_top > 0,
          "and the SAME world flows as soon as the top row is inside the driving rectangle -- "
          "which is what makes the zero above the region and not a dead export");
    Check(two_row_game > 0 && two_row_top > 0,
          "an 8x2 world flows under BOTH regions -- dropping the top row is invisible unless "
          "the top row is the world's only row, which is the whole claim");
  }

  // ------------------------------------------------------------------------------------
  // The live per-kernel profile, published.
  //
  // `bench` times these same kernels offline against a corpus and never runs the game's own
  // frame; the backtick key times them in a real colony and prints the answer as English into
  // two log files. `SIM_DebugProfile` is the third reader and the only one a program can use,
  // which is what `DebugInspectorServer`'s `/profile` route serves and what a mod asking
  // whether its own additions cost a frame has to be able to call.
  //
  // WHAT THESE ARMS ARE FOR, given that a timing cannot be goldened. Milliseconds move with
  // the machine, so nothing here asserts a duration -- what it asserts is the SHAPE of the
  // table and the two rules that make it readable:
  //
  //   * the census counts on every frame of every run, ARMED OR NOT, while the milliseconds
  //     count only while armed. That asymmetry is published, and a consumer
  //     reading a 0.0 ms row has to be able to tell "nobody armed it" from "this kernel did
  //     no work" -- so the arms below prove a row can honestly carry thousands of examined
  //     cells and no time at all.
  //   * READING DOES NOT ZERO, and arming an already-armed profiler does not either. Both are
  //     what let a live poller sample-wait-sample, and both are invisible to any single read.
  //
  // The summary's four geometry fields are exact integers and ARE asserted exactly, and they
  // land on the same fact the arm above this one guards: the game sends `maxY = height - 1`,
  // so an 8x4 world's driving rectangle is 8x3 while its inclusive one is 8x4. A region
  // convention that changed would move both arms, from two directions.
  printf("\n=== vftest: SIM_DebugProfile -- the live per-kernel table ===\n");
  {
    sim_shutdown();
    InitSim();
    World pw;
    pw.Init(8, 4, oxygen, 1.0f, 300.0f);
    // One heavy cell so the gas kernels have something to do; the same seed the flow arms
    // use, for the same reason -- a world where nothing moves gives every sweep a change
    // count of zero and makes half of this table untestable.
    pw.mass[2 * 8 + 2] = 20.0f;
    Boot(t, pw);
    std::vector<uint8_t> pvis;

    const int32_t slots = sim_debug_profile(nullptr, 0);
    printf("  slots %d\n", slots);
    Check(slots > 0, "the sizing call (null buffer, max 0) returns a slot count");

    // The short-buffer half of the same discipline. A sentinel in the row past the cut,
    // because "returns the total" and "writes only what fits" are two claims and a suite
    // that checks the first and not the second has not checked the refusal at all.
    std::vector<OniProfileSlot> rows(static_cast<size_t>(slots) + 1);
    memset(rows.data(), 0xEE, rows.size() * sizeof(OniProfileSlot));
    const int32_t short_total = sim_debug_profile(rows.data(), 3);
    Check(short_total == slots,
          "a buffer too small still returns the TOTAL, not the number copied -- the return "
          "sizes the buffer, the SIM_ExtElementAttributeKeys discipline");
    Check(static_cast<unsigned char>(rows[3].name[0]) == 0xEE,
          "and it writes exactly `max` rows: the fourth is untouched with max 3");

    Check(sim_debug_profile_summary(nullptr) == 0,
          "SIM_DebugProfileSummary refuses a null pointer with 0 rather than writing one");

    // ---- the census counts with the profiler OFF, and this is the published asymmetry ----
    for (int k = 0; k < 5; ++k) Tick(pw, &pvis);
    memset(rows.data(), 0, rows.size() * sizeof(OniProfileSlot));
    sim_debug_profile(rows.data(), slots);
    OniProfileSummary sum{};
    Check(sim_debug_profile_summary(&sum) == 1, "the summary fills its out-parameter");

    int64_t examined_off = 0;
    double ms_off = 0.0;
    int64_t calls_off = 0;
    bool every_row_named = true;
    bool budget_matches_sweep = true;
    bool all_within_budget = true;
    bool saw_conduction = false;
    for (int32_t i = 0; i < slots; ++i) {
      const OniProfileSlot& r = rows[static_cast<size_t>(i)];
      examined_off += r.examined;
      ms_off += r.msTotal;
      calls_off += r.calls;
      if (r.name[0] == '\0' || r.name[sizeof(r.name) - 1] != '\0') every_row_named = false;
      if ((r.budgetPerInvocation > 0) != (r.cellSweep != 0)) budget_matches_sweep = false;
      if (!r.withinBudget) all_within_budget = false;
      if (!strcmp(r.name, "StepConduction")) saw_conduction = true;
    }
    // The examined figure here is NOT this world's five ticks. Nothing has ever armed the
    // profiler in this process, and `census::Reset` runs only on an arm, so the counters have
    // been accumulating across every arm in this suite since it started -- hundreds of worlds,
    // most of them larger than this one. That is correct and worth stating: the census has no
    // window of its own, and a consumer that wants one takes two reads and subtracts, exactly
    // as it must for the milliseconds.
    printf("  profiler OFF, census cumulative since process start: examined %lld, ms %.4f, "
           "calls %lld, enabled %d, frames %d\n",
           static_cast<long long>(examined_off), ms_off,
           static_cast<long long>(calls_off), sum.enabled, sum.frames);

    Check(sum.enabled == 0 && sum.frames == 0,
          "the profiler is OFF and has counted no frames -- nothing armed it");
    Check(ms_off == 0.0 && calls_off == 0,
          "so every row's milliseconds and call count are zero: with the profiler off the "
          "sim reads no timer at all, which is what lets it exist in a shipping build");
    Check(examined_off > 0,
          "and the CENSUS counted anyway -- it is not gated on the profiler, so a 0.0 ms row "
          "means 'nobody armed it' and never 'this kernel did no work'");
    Check(every_row_named, "every row carries a NUL-terminated kernel name");
    Check(saw_conduction,
          "and the names are the sim's own: StepConduction is one of them, so a consumer can "
          "find a kernel by the name bench and the log file already use");
    Check(budget_matches_sweep,
          "budgetPerInvocation is set on exactly the cell sweeps and 0 on the rows that walk "
          "LISTS -- a ratio against the grid says nothing about the message queue");
    // **This assertion is position-dependent, and that is a hazard rather than a design.**
    // `examined` is cumulative since process start (see the note above) while
    // `budgetPerInvocation` is derived from the world loaded RIGHT NOW, so the ratio it checks
    // only means anything because every arm that runs before this point happens to leave it
    // under the bound. Adding a section ABOVE this one that ticks a world wider than this
    // one's region pushes the cumulative average over the current budget and fails this check
    // with nothing whatsoever wrong in the sim -- which is exactly what happened when the
    // liquid-gate arms were first written above it (25,760 examined over 821 invocations
    // against a budget of 24). New sections go BELOW this one until the census grows a window
    // of its own.
    Check(all_within_budget,
          "and no kernel stepped over more cells than its declared bound allows");

    // ---- the summary's geometry, exact, and it lands on the top-row rule ----
    printf("  summary geometry: grid %lld, region %lld, region+1 %lld, regions %lld\n",
           static_cast<long long>(sum.gridCells), static_cast<long long>(sum.regionCells),
           static_cast<long long>(sum.regionCellsInclusive),
           static_cast<long long>(sum.regions));
    Check(sum.gridCells == 10 * 6,
          "gridCells is the PADDED grid -- 8x4 game cells plus the one-cell border ring");
    Check(sum.regions == 1, "one active region, because this world is one asteroid");
    Check(sum.regionCells == 8 * 3,
          "the driving rectangle is 8x3 and not 8x4: the game sends maxY = height - 1, so the "
          "world's top row is never swept -- the same rule the one-row arm above guards");
    Check(sum.regionCellsInclusive == 8 * 4,
          "and the INCLUSIVE rectangle does cover it, which is why StepPostProcess reaches a "
          "row the driving kernels never do");

    // ---- arming makes the milliseconds appear, and only the milliseconds ----
    Check(sim_debug_set_profiler(1) == 0,
          "SIM_DebugSetProfiler returns the PREVIOUS setting, so a caller can restore it");
    // Read with no tick in between, so the only thing that can have changed the census is the
    // arm itself. Comparing two accumulated windows instead would depend on how many frames
    // the boot ran, which is not what this arm is about.
    std::vector<OniProfileSlot> zeroed(static_cast<size_t>(slots));
    sim_debug_profile(zeroed.data(), slots);
    int64_t examined_zeroed = 0;
    for (int32_t i = 0; i < slots; ++i) examined_zeroed += zeroed[static_cast<size_t>(i)].examined;
    Check(examined_zeroed == 0,
          "arming ZEROED the census along with the timings -- the two are read as one table, "
          "so a cell count accumulated over a different window than the milliseconds beside "
          "it would be worse than no cell count at all");
    for (int k = 0; k < 5; ++k) Tick(pw, &pvis);
    memset(rows.data(), 0, rows.size() * sizeof(OniProfileSlot));
    sim_debug_profile(rows.data(), slots);
    OniProfileSummary armed{};
    sim_debug_profile_summary(&armed);
    double ms_on = 0.0;
    int64_t calls_on = 0, examined_on = 0;
    for (int32_t i = 0; i < slots; ++i) {
      ms_on += rows[static_cast<size_t>(i)].msTotal;
      calls_on += rows[static_cast<size_t>(i)].calls;
      examined_on += rows[static_cast<size_t>(i)].examined;
    }
    printf("  armed, 5 more ticks: ms %.4f, calls %lld, examined %lld, frames %d\n", ms_on,
           static_cast<long long>(calls_on), static_cast<long long>(examined_on),
           armed.frames);
    Check(armed.enabled == 1, "the summary reports the profiler as armed");
    Check(armed.frames == 5,
          "and counts exactly the five frames since arming -- one per PrepareGameData, not "
          "one per substep");
    Check(calls_on > 0 && ms_on > 0.0,
          "the timings are now being taken: kernels report calls and a non-zero duration");
    Check(examined_on > 0,
          "and the census restarted from that zero and counted the five frames since");

    // ---- reading does not zero, and re-arming an armed profiler does not either ----
    for (int k = 0; k < 5; ++k) Tick(pw, &pvis);
    std::vector<OniProfileSlot> later(static_cast<size_t>(slots));
    sim_debug_profile(later.data(), slots);
    double ms_later = 0.0;
    int64_t examined_later = 0;
    for (int32_t i = 0; i < slots; ++i) {
      ms_later += later[static_cast<size_t>(i)].msTotal;
      examined_later += later[static_cast<size_t>(i)].examined;
    }
    Check(examined_later > examined_on && ms_later >= ms_on,
          "a read does not zero: the numbers are cumulative since the arm, which is what "
          "lets a live poller sample, wait, sample and subtract");

    Check(sim_debug_set_profiler(1) == 1,
          "arming an ALREADY-armed profiler reports it was already on");
    std::vector<OniProfileSlot> rearmed(static_cast<size_t>(slots));
    sim_debug_profile(rearmed.data(), slots);
    int64_t examined_rearmed = 0;
    for (int32_t i = 0; i < slots; ++i) examined_rearmed += rearmed[static_cast<size_t>(i)].examined;
    Check(examined_rearmed == examined_later,
          "and it is a NO-OP rather than a re-arm -- zeroing here would silently discard a "
          "window somebody else armed, halfway through it, with no way for either to notice");

    // ---- disarming stops one clock and not the other ----
    Check(sim_debug_set_profiler(0) == 1, "disarming reports the previous setting too");
    for (int k = 0; k < 5; ++k) Tick(pw, &pvis);
    std::vector<OniProfileSlot> after(static_cast<size_t>(slots));
    sim_debug_profile(after.data(), slots);
    OniProfileSummary off{};
    sim_debug_profile_summary(&off);
    double ms_after = 0.0;
    int64_t examined_after = 0;
    for (int32_t i = 0; i < slots; ++i) {
      ms_after += after[static_cast<size_t>(i)].msTotal;
      examined_after += after[static_cast<size_t>(i)].examined;
    }
    printf("  disarmed, 5 more ticks: ms %.4f (was %.4f), examined %lld (was %lld)\n",
           ms_after, ms_later, static_cast<long long>(examined_after),
           static_cast<long long>(examined_later));
    Check(off.enabled == 0, "the summary reports the profiler as disarmed");
    Check(ms_after == ms_later,
          "the millisecond half stopped exactly where it was -- disarming does not clear the "
          "window it measured, and does not write the log-file report the backtick key does");
    Check(examined_after > examined_later,
          "while the census kept counting through the disarm, which is the same asymmetry "
          "read from the other end");
  }

  // ------------------------------------------------------------------------------------
  // GOLDEN STATE FOR THE DEVIATIONS `diffsim` CANNOT SEE.
  //
  // `diffsim` answers exactly one question: does this DLL match Klei's, bit for bit, over a
  // corpus of real frames. It is the strongest oracle this project has and it is structurally
  // BLIND to everything the project adds on purpose -- because every one of those additions is,
  // by definition, a difference from Klei. An extension message that landed a degree hotter
  // than it used to, or stopped landing at all, leaves `diffsim.all.digest` exactly where it
  // was. So does the whole multi-gas layer. Two oracles, two questions.
  //
  // WHAT A KEY HERE PINS. One deviation, driven the way a mod drives it -- through the
  // published message ids -- against a fixed world for a fixed number of ticks, digested over
  // the state the frame PUBLISHES: every game cell's element, mass and temperature, plus every
  // building temperature. Bit patterns and not printed values, for `bench`'s reason: two floats
  // that format the same are not the same float, and the point of a state golden is to fail on
  // the ulp a formatted comparison forgives.
  //
  // THREE THINGS ARE CHECKED BESIDE THE GOLDENS, AND THE GOLDENS ARE WORTHLESS WITHOUT THEM.
  //
  //   (a) THE FIXTURE IS DETERMINISTIC. The baseline is run TWICE and the two digests must be
  //       equal. A golden over a fixture that wanders is not a gate, it is a tripwire that
  //       fires on Tuesdays, and the only way to know which one this is, is to run it twice.
  //   (b) EVERY DEVIATION ACTUALLY MOVES THE WORLD. Each digest must differ from the baseline's.
  //       This is the arm that fires when a message stops being delivered -- which has happened
  //       in this repo: `QueueDeferredMessage` silently dropped EVERY extension message for
  //       weeks. A golden alone would not have caught it; the recorded value would
  //       simply have become the vanilla one on the day it was recorded.
  //   (c) THE FIVE DEVIATIONS ARE FIVE. All the digests are pairwise distinct, which is what
  //       fails if two branches of the fixture send the same message -- the copy-paste this
  //       shape of test invites, and one that (b) alone passes happily.
  //
  // A claim no planted fault can break is not a check. `state.dev.vanilla` on its own is satisfied by a
  // sim that ignores every extension message ever sent to it.
  // ------------------------------------------------------------------------------------
  // ext::kSetWorldEnvironment -- the planet a world is on.
  //
  // Three capabilities and they are tested separately, because they share only an input:
  // the per-world radiative sink, solar absorption (building and cell), and the surface
  // atmosphere boundary.
  //
  // EVERY ARM DEFINES A WORLD FIRST. The two per-cell terms key off the sunlight texture, which
  // is per world and stays zero until `DefineWorldOffsets` arrives -- an arm that forgot it
  // would measure a planet in permanent shadow and pass by measuring nothing.
  // ---------------------------------------------------------------- the LIQUID promotion gate
  //
  // Gating only the two GAS movers would leave a real hole, because volume-fractions was
  // designed atmosphere-only and liquids fall through it unchanged. These arms prove the hole
  // is real rather than theoretical:
  //
  //   * a promoted cell sits at vacuum / zero mass in vanilla's grid (`ClearCell`),
  //   * `StepFlow`'s `permeable` is `Phase(element) < kStateLiquid`, so vacuum qualifies,
  //   * `move_into` refuses a LIQUID destination and sends a GAS one through `DisplaceGas`,
  //     but a VACUUM destination passes both guards and takes `d.element` plus the mass.
  //
  // So vanilla liquid poured straight into a promoted cell and overwrote the
  // `PhaseEntry.temperature` field the mixture layer uses as its shared temperature.
  //
  // The gate is `gas::IsMixtureOwnedCell`, NOT `IsRoomOwned`: promotion and mixture ownership
  // coincide for gas and do NOT for liquid, so asking the room question would freeze an
  // ordinary pond that no engine is simulating. The third arm is a negative control that
  // proves we did not do that -- without it, the first two would pass just as happily under
  // the wrong predicate.
  //
  // **The water is created by a `MassEmission` message AFTER the gate is already live**, and
  // that is not a stylistic choice. Seeding it in the world instead loses a race that cost a
  // debugging round to see: `PrepareGameData` publishes the frame BEFORE the one it queues,
  // so a run that seeds 400 kg of water and then promotes has already
  // drained the cell by the time the first published frame can be read, and the arm ends up
  // measuring message latency rather than the gate. Emitting into a world that has already
  // settled removes the race completely. A liquid emission is the right instrument as well as
  // a convenient one: `ApplyMassEmission` redirects only GASES into the mixture
  // (`emitters.h`'s `t.Phase(element) == kStateGas`), so water lands in vanilla's own
  // `PhaseEntry` exactly as a vent would put it there.
  printf("\n=== vftest: promotion gates the vanilla liquid flow mover (StepFlow) ===\n");
  sim_shutdown();
  InitSim();
  uint16_t lq_granite = 0, lq_water = 0, lq_vacuum = 0;
  if (!Resolve(t, simhost::kGranite, "Granite", &lq_granite)) return 1;
  if (!Resolve(t, simhost::kWater, "Water", &lq_water)) return 1;
  if (!Resolve(t, simhost::kVacuum, "Vacuum", &lq_vacuum)) return 1;
  constexpr int kLW = 24, kLH = 16;
  // A vertical run of five open cells. `kLMid` is where the water is emitted; `kLC1` and
  // `kLC3` are its two neighbours; `kLC0` and `kLC4` are far enough away to be somewhere the
  // negative control can put mixture mass without standing in the water's path.
  constexpr int kLCol = 8;
  constexpr int kLC0 = 4 * kLW + kLCol, kLC1 = 5 * kLW + kLCol, kLMid = 6 * kLW + kLCol,
                kLC3 = 7 * kLW + kLCol, kLC4 = 8 * kLW + kLCol;
  auto carve_column = [&](World* wd) {
    wd->Init(kLW, kLH, lq_granite, 2000.0f, 293.15f);
    for (int c : {kLC0, kLC1, kLMid, kLC3, kLC4}) {
      wd->element[c] = lq_vacuum;
      wd->mass[c] = 0.0f;
      wd->temperature[c] = 293.15f;
    }
  };
  auto emit_water = [&](int32_t cell) {
    MassEmissionMessage m{};
    m.cellIdx = cell;
    m.callbackIdx = -1;
    m.mass = 400.0f;
    m.temperature = 290.0f;
    m.diseaseCount = 0;
    m.elementIdx = lq_water;
    sim_handle_message(static_cast<int32_t>(SimMessageHash::MassEmission), sizeof(m),
                       reinterpret_cast<const uint8_t*>(&m));
  };

  // Ungated baseline: no promotion, no injection. Water spreads out of the middle cell.
  World lq_base;
  carve_column(&lq_base);
  const GameDataUpdate* lg = Boot(t, lq_base);
  std::vector<uint8_t> lq_visible;
  for (int i = 0; i < 3; ++i) lg = Tick(lq_base, &lq_visible);
  emit_water(kLMid);
  // `kLC1` is the cell BELOW `kLMid` -- the water pours through it and on down to `kLC0`, so
  // the end state is not where the evidence is. What every arm here measures is the same
  // thing: did water EVER occupy the neighbour, on any tick. An end-state check would call a
  // gate that holds for one tick and then lets go a pass, which is exactly the failure the
  // first version of this section hid.
  const int kLCells[5] = {kLC0, kLC1, kLMid, kLC3, kLC4};
  float base_final[5] = {0, 0, 0, 0, 0};
  bool base_reached_c1 = false;
  for (int i = 0; i < 6; ++i) {
    lg = Tick(lq_base, &lq_visible);
    if (lg->mass[kLC1] > 0.0f) base_reached_c1 = true;
  }
  for (int c = 0; c < 5; ++c) base_final[c] = lg ? lg->mass[kLCells[c]] : -1.0f;
  printf("  ungated: column %.3f %.3f %.3f %.3f %.3f, water reached the cell below: %s\n",
         base_final[0], base_final[1], base_final[2], base_final[3], base_final[4],
         base_reached_c1 ? "yes" : "no");
  Check(base_reached_c1,
        "sanity check: with no gate, emitted water pours out of the middle cell through the "
        "neighbour below it -- the gate below has something real to prevent");

  // Gated: promote the column and give BOTH of the water's neighbours real mixture mass, so
  // `IsMixtureOwnedCell` is true for them, and let all of that settle BEFORE any water
  // exists. The middle cell is promoted too and is deliberately left without mixture mass --
  // it is still vanilla's, and it still tries.
  sim_shutdown();
  InitSim();
  World lq_gated;
  carve_column(&lq_gated);
  const GameDataUpdate* lgg = Boot(t, lq_gated);
  std::vector<uint8_t> lq_gated_visible;
  ext::PromoteRoomMessage lq_promote{kLMid};
  sim_handle_message(ext::kPromoteRoom, static_cast<int>(sizeof(lq_promote)),
                     reinterpret_cast<const uint8_t*>(&lq_promote));
  InjectGas(kLC1, oxygen, 2.0f, 293.15f);
  InjectGas(kLC3, oxygen, 2.0f, 293.15f);
  for (int i = 0; i < 3; ++i) lgg = Tick(lq_gated, &lq_gated_visible);
  Check(sim_debug_room_owned(kLMid) == 1, "the water cell's room is promoted in the gated world");
  Check(sim_debug_gas_mass(kLC1, oxygen) > 0.0f && sim_debug_gas_mass(kLC3, oxygen) > 0.0f,
        "and both of its neighbours really are mixture-owned before any water exists");

  emit_water(kLMid);
  // Liquid over mixture gas must not hang there: refusing the mixture-owned neighbour would
  // hold the water up for good. The gas steps aside (`EvictMixture`) and the water
  // pours, so what is checked is the other half of the original concern: the mixture is not
  // overwritten. Every tick after the emission, not just the last one: the oxygen's total
  // holds, and no cell ever carries water and mixture gas at once.
  //
  // The second claim pairs each PUBLISHED grid with the mixture read on the tick BEFORE it.
  // `sim_debug_gas_mass` reads the live state, and the frame `Tick` returns is the one before
  // the one it queued (docs/THREADING.md). Pairing a frame with its own tick's reads measured
  // the lag, not the sim: tick 1 published water in `kLC1` while the live oxygen there was
  // what had risen into it after the water had already moved on to `kLC0`.
  const int kLAll[5] = {kLC0, kLC1, kLMid, kLC3, kLC4};
  float worst_neighbour = 0.0f;
  float o2_lo = 1e9f, o2_hi = -1e9f;
  int shared_ticks = 0;
  float live_prev[5];
  for (int c = 0; c < 5; ++c) live_prev[c] = sim_debug_gas_mass(kLAll[c], oxygen);
  for (int i = 0; i < 12; ++i) {
    lgg = Tick(lq_gated, &lq_gated_visible);
    const float n = lgg->mass[kLC1] + lgg->mass[kLC3];
    if (n > worst_neighbour) worst_neighbour = n;
    float o2 = 0.0f;
    bool shared = false;
    for (int c = 0; c < 5; ++c) {
      if (live_prev[c] > 0.0f && lgg->elementIdx[kLAll[c]] == lq_water) shared = true;
      live_prev[c] = sim_debug_gas_mass(kLAll[c], oxygen);
      o2 += live_prev[c];
    }
    if (o2 < o2_lo) o2_lo = o2;
    if (o2 > o2_hi) o2_hi = o2;
    if (shared) ++shared_ticks;
  }
  printf("  gated:   mid %.3f kg, most the neighbours ever held %.3f kg; mixture O2 %.6f..%.6f "
         "kg; ticks with water and mixture in one cell: %d\n",
         lgg ? lgg->mass[kLMid] : -1.0f, worst_neighbour, o2_lo, o2_hi, shared_ticks);
  Check(worst_neighbour > 0.0f,
        "the water pours through the mixture-owned cells: the mixture gas steps aside instead "
        "of holding the liquid up");
  Check(std::fabs(o2_lo - 4.0f) < 1e-4f && std::fabs(o2_hi - 4.0f) < 1e-4f,
        "and the mixture's 4 kg of oxygen is all still there on every tick -- moved, not "
        "overwritten or deleted");
  Check(shared_ticks == 0,
        "and no published tick has a cell holding water and mixture gas at once");

  // A LIQUID ADDED ONTO MIXTURE GAS by ModifyCell -- a falling drop landing, a building's
  // liquid output, a debug spawn. Klei's `AddLiquid` reads a mixture cell as vacuum and would
  // write the water's temperature over the one the mixture shares, so the gas is moved aside
  // first (`AddLiquidBesideMixture`). The liquid sweeps would move the gas out of the water's
  // cell a substep later either way, so where the gas ends up cannot tell the two apart; its
  // HEAT can. One species, so each cell's pressure over its mass is proportional to its
  // temperature and sum(P) / sum(m) over the column tracks the oxygen's mean temperature.
  // Conduction does not run inside a promoted room, so nothing else can move that heat.
  printf("\n=== vftest: a liquid added onto mixture gas does not take the gas's heat ===\n");
  sim_shutdown();
  InitSim();
  World lq_add;
  carve_column(&lq_add);
  const GameDataUpdate* lga = Boot(t, lq_add);
  std::vector<uint8_t> lq_add_visible;
  ext::PromoteRoomMessage lq_add_promote{kLMid};
  sim_handle_message(ext::kPromoteRoom, static_cast<int>(sizeof(lq_add_promote)),
                     reinterpret_cast<const uint8_t*>(&lq_add_promote));
  InjectGas(kLC3, oxygen, 1.0f, 400.0f);
  for (int i = 0; i < 40; ++i) lga = Tick(lq_add, &lq_add_visible);
  auto heat_ratio = [&](float* total_mass) {
    double p = 0.0, m = 0.0;
    for (int c : kLAll) {
      const float cm = sim_debug_gas_mass(c, oxygen);
      if (!(cm > 0.0f)) continue;
      p += sim_gas_pressure(c);
      m += cm;
    }
    if (total_mass) *total_mass = static_cast<float>(m);
    return m > 0.0 ? p / m : 0.0;
  };
  float add_mass_before = 0.0f, add_mass_after = 0.0f;
  const double ratio_before = heat_ratio(&add_mass_before);
  const float top_share = sim_debug_gas_mass(kLC4, oxygen) / add_mass_before;
  ModifyCellMessage lq_drop{};
  lq_drop.cellIdx = kLC4;
  lq_drop.callbackIdx = -1;
  lq_drop.temperature = 290.0f;
  lq_drop.mass = 5.0f;
  lq_drop.diseaseCount = 0;
  lq_drop.elementIdx = lq_water;
  lq_drop.replaceType = 0;  // kReplaceNone
  lq_drop.diseaseIdx = 0xFF;
  lq_drop.addSubType = 0;
  sim_handle_message(static_cast<int32_t>(SimMessageHash::ModifyCell), sizeof(lq_drop),
                     reinterpret_cast<const uint8_t*>(&lq_drop));
  bool water_landed = false;
  for (int i = 0; i < 6; ++i) {
    lga = Tick(lq_add, &lq_add_visible);
    for (int c : kLAll) {
      if (lga->elementIdx[c] == lq_water) water_landed = true;
    }
  }
  const double ratio_after = heat_ratio(&add_mass_after);
  printf("  top cell held %.1f%% of the oxygen; P/m %.4f -> %.4f (%.3f%%), O2 %.6f -> %.6f kg, "
         "water landed: %s\n",
         100.0f * top_share, ratio_before, ratio_after,
         100.0 * (ratio_after - ratio_before) / ratio_before, add_mass_before, add_mass_after,
         water_landed ? "yes" : "no");
  Check(water_landed && top_share > 0.05f,
        "setup: the water really landed, on a cell holding a real share of the mixture");
  Check(std::fabs(add_mass_after - add_mass_before) < 1e-5f,
        "the oxygen's mass is all still there after the water lands on it");
  Check(std::fabs(ratio_after - ratio_before) < 1e-3 * ratio_before,
        "and so is its heat: the 290 K water did not overwrite the 400 K gas's temperature");

  // THE NEGATIVE CONTROL, and the reason the gate is `IsMixtureOwnedCell` rather than
  // `IsRoomOwned`. Same promoted room, but the mixture mass goes into a cell at the far end
  // that the water is not trying to enter. Under a room-promotion predicate this water would
  // freeze; under the ownership predicate it must behave exactly like the ungated world.
  printf("\n=== vftest: a vanilla pond inside a promoted room still flows (the gate is "
         "ownership, not promotion) ===\n");
  sim_shutdown();
  InitSim();
  World lq_ctrl;
  carve_column(&lq_ctrl);
  const GameDataUpdate* lgc = Boot(t, lq_ctrl);
  std::vector<uint8_t> lq_ctrl_visible;
  ext::PromoteRoomMessage lq_ctrl_promote{kLMid};
  sim_handle_message(ext::kPromoteRoom, static_cast<int>(sizeof(lq_ctrl_promote)),
                     reinterpret_cast<const uint8_t*>(&lq_ctrl_promote));
  // `kLC4` is the TOP of the column, two cells above the water and in the opposite direction
  // from the pour. The choice matters and was measured, not assumed: injecting at `kLC0`
  // instead made this arm fail, because room-pooled mixing had carried the species one cell
  // per tick into `kLC1` -- the cell directly below the water -- by the time the water
  // existed, so the pour was legitimately blocked and the control was measuring mixing
  // diffusion rather than the gate. That is a real property of an ownership predicate and it
  // is recorded on `IsMixtureOwnedCell`: occupancy SPREADS, so "unowned" is a statement about
  // now, not forever.
  //
  // And the mass is 4 mg, not 2 kg, because 2 kg stopped holding still. Until
  // MixPair carried heat, oxygen that crossed into a promoted empty cell kept that cell's
  // 0 K -- ZeroMasslessCells had zeroed it while it held nothing -- so it had no pressure and
  // went no further: measured, 0.72 kg sat in `kLC3` at 0 K and never reached `kLMid`. This
  // arm passed on that heat deletion. With the heat carried, `kLC3` reads 293.15 K and the
  // oxygen reaches `kLMid` within two ticks, so any real mass anywhere in this five-cell
  // column spreads to the water. The first transfer out of a full cell into an empty one is a
  // fifth of its mass (measured above: 2 kg sent 0.4 kg), so 4 mg would send 0.8 mg, under
  // MixPair's 1e-6 kg transfer floor: `kLC4` stays mixture-owned and nothing else becomes so.
  InjectGas(kLC4, oxygen, 4e-6f, 293.15f);
  for (int i = 0; i < 3; ++i) lgc = Tick(lq_ctrl, &lq_ctrl_visible);
  Check(sim_debug_room_owned(kLMid) == 1, "the room really is promoted in the control world too");
  Check(sim_debug_gas_mass(kLC4, oxygen) > 0.0f,
        "and the far cell really holds mixture gas -- one cell of the room is owned");
  Check(sim_debug_gas_mass(kLMid, oxygen) == 0.0f && sim_debug_gas_mass(kLC1, oxygen) == 0.0f,
        "and the mixture has NOT reached the water's cell or the one below it, so the pour is "
        "genuinely unobstructed");
  emit_water(kLMid);
  bool ctrl_reached_c1 = false;
  for (int i = 0; i < 6; ++i) {
    lgc = Tick(lq_ctrl, &lq_ctrl_visible);
    if (lgc->mass[kLC1] > 0.0f) ctrl_reached_c1 = true;
  }
  bool ctrl_matches = ctrl_reached_c1 == base_reached_c1;
  for (int c = 0; c < 5; ++c) {
    if (lgc->mass[kLCells[c]] != base_final[c]) ctrl_matches = false;
  }
  printf("  control: column %.3f %.3f %.3f %.3f %.3f (ungated %.3f %.3f %.3f %.3f %.3f)\n",
         lgc->mass[kLC0], lgc->mass[kLC1], lgc->mass[kLMid], lgc->mass[kLC3], lgc->mass[kLC4],
         base_final[0], base_final[1], base_final[2], base_final[3], base_final[4]);
  Check(ctrl_matches,
        "a promoted room whose cells the mixture does NOT own flows exactly as it did before "
        "promotion, cell for cell -- the liquid gate did not freeze an ordinary pond");

  // ---------------------------------------------------------------- the other liquid mover
  //
  // `StepLiquidDisplacement` is to `StepFlow` what `StepGasDisplacement` is to
  // `StepGasPressure`: the separate mechanism that moves two DIFFERENT liquids past each
  // other, which the flow mover refuses outright (`move_into` returns false for a destination
  // holding a different liquid). Gating only the flow mover would leave a promoted cell open
  // to being overwritten by displacement instead of by a pour -- exactly the hole the gas gate
  // avoids by gating both gas kernels together.
  //
  // The second liquid is found by scanning the real element table for a liquid that is not
  // water, rather than by writing a SimHashes constant from memory -- a constant written that
  // way has been wrong before in this project (see the diatomic molar-mass work).
  printf("\n=== vftest: promotion gates vanilla liquid displacement "
         "(StepLiquidDisplacement) ===\n");
  uint16_t other_liquid = 0xFFFF;
  for (int32_t i = 0; i < t.count; ++i) {
    const Element* e = t.At(i);
    if ((e->state & 3) == 2 && static_cast<uint16_t>(i) != lq_water) {
      other_liquid = static_cast<uint16_t>(i);
      break;
    }
  }
  Check(other_liquid != 0xFFFF,
        "the element table carries a second liquid to displace water against");
  if (other_liquid != 0xFFFF) {
    // The three-cell run the kernel needs, in a row: [other][dst][src]. The water is emitted
    // into `src` once the gate is live, for the same anti-race reason as the arm above, and
    // it has to outweigh `start[other].mass + start[dst].mass` for the `available` test to
    // pass at all.
    constexpr int kDRow = 6;
    constexpr int kDOther = kDRow * kLW + 5, kDDst = kDRow * kLW + 6, kDSrc = kDRow * kLW + 7;
    auto carve_row = [&](World* wd) {
      wd->Init(kLW, kLH, lq_granite, 2000.0f, 293.15f);
      for (int c : {kDOther, kDDst, kDSrc}) {
        wd->element[c] = other_liquid;
        wd->mass[c] = 1.0f;
        wd->temperature[c] = 290.0f;
      }
    };
    auto emit_water_at = [&](int32_t cell) {
      MassEmissionMessage m{};
      m.cellIdx = cell;
      m.callbackIdx = -1;
      m.mass = 900.0f;
      m.temperature = 290.0f;
      m.diseaseCount = 0;
      m.elementIdx = lq_water;
      sim_handle_message(static_cast<int32_t>(SimMessageHash::MassEmission), sizeof(m),
                         reinterpret_cast<const uint8_t*>(&m));
    };

    sim_shutdown();
    InitSim();
    World dsp_base;
    carve_row(&dsp_base);
    const GameDataUpdate* dg = Boot(t, dsp_base);
    std::vector<uint8_t> dsp_visible;
    for (int i = 0; i < 3; ++i) dg = Tick(dsp_base, &dsp_visible);
    emit_water_at(kDSrc);
    bool base_displaced = false;
    for (int i = 0; i < 6; ++i) {
      dg = Tick(dsp_base, &dsp_visible);
      if (dg->elementIdx[kDDst] == lq_water) base_displaced = true;
    }
    printf("  ungated: dst ended on element %u (water is %u), displaced at some point: %s\n",
           dg ? dg->elementIdx[kDDst] : 0u, lq_water, base_displaced ? "yes" : "no");
    Check(base_displaced,
          "sanity check: with no gate, vanilla liquid displacement pushes the other liquid "
          "aside and moves water into its cell");

    sim_shutdown();
    InitSim();
    World dsp_gated;
    carve_row(&dsp_gated);
    const GameDataUpdate* dgg = Boot(t, dsp_gated);
    std::vector<uint8_t> dsp_gated_visible;
    ext::PromoteRoomMessage dsp_promote{kDDst};
    sim_handle_message(ext::kPromoteRoom, static_cast<int>(sizeof(dsp_promote)),
                       reinterpret_cast<const uint8_t*>(&dsp_promote));
    InjectGas(kDDst, oxygen, 2.0f, 293.15f);
    for (int i = 0; i < 3; ++i) dgg = Tick(dsp_gated, &dsp_gated_visible);
    Check(sim_debug_room_owned(kDDst) == 1 && sim_debug_gas_mass(kDDst, oxygen) > 0.0f,
          "the displacement destination is promoted AND mixture-owned before any water exists");
    emit_water_at(kDSrc);
    bool gated_displaced = false;
    for (int i = 0; i < 6; ++i) {
      dgg = Tick(dsp_gated, &dsp_gated_visible);
      if (dgg->elementIdx[kDDst] == lq_water) gated_displaced = true;
    }
    printf("  gated:   dst ended on element %u, displaced at any point: %s\n",
           dgg ? dgg->elementIdx[kDDst] : 0u, gated_displaced ? "yes" : "no");
    Check(!gated_displaced,
          "once the destination is mixture-owned, vanilla displacement never moves water into "
          "it on any tick -- all three cells of a displacement are gated, as in "
          "StepGasDisplacement");
  }

  printf("\n=== vftest: world environment (ext::kSetWorldEnvironment) ===\n");
  {
    sim_debug_ledger =
        reinterpret_cast<int (*)(double*, int)>(GetProcAddress(g_sim, "SIM_DebugLedger"));
    Check(sim_debug_ledger != nullptr, "SIM_DebugLedger is exported");
    uint16_t we_granite = 0, we_co2 = 0, we_water = 0;
    if (!Resolve(t, kGranite, "Granite", &we_granite)) return 1;
    if (!Resolve(t, simhost::kCarbonDioxide, "CarbonDioxide", &we_co2)) return 1;
    if (!Resolve(t, simhost::kWater, "Water", &we_water)) return 1;

    auto send_env = [](const ext::SetWorldEnvironmentMessage& m) {
      sim_handle_message(ext::kSetWorldEnvironment, static_cast<int>(sizeof(m)),
                         reinterpret_cast<const uint8_t*>(&m));
    };

    // ---- 1. the sink is per world, and falls back to the global when it is not set ---------
    //
    // Three runs of one probe: the global scalar alone, a world record that overrides it, and a
    // world record with sinkKelvin 0 that must fall back. A hotter sink sheds less, and the
    // third run has to land exactly on the first -- "exactly", because a fallback that quietly
    // used 0 K instead would shed MORE and a >= test would pass on it.
    {
      struct SinkCase {
        const char* name;
        bool send_world;
        float world_sink;
      };
      const SinkCase cases[] = {
          {"global 50 K only", false, 0.0f},
          {"world record 350 K", true, 350.0f},
          {"world record with sink 0 (must fall back to the global)", true, 0.0f},
      };
      double shed[3] = {0, 0, 0};
      for (int ci = 0; ci < 3; ++ci) {
        World ew;
        ew.Init(5, 5, vacuum, 0.0f, 100.0f);
        Boot(t, ew);
        DefineOneWorld(ew.width, ew.height);
        std::vector<uint8_t> evis;

        DebugProperties scales{};
        scales.buildingTemperatureScale = 0.001f;
        scales.buildingToBuildingTemperatureScale = 0.001f;
        sim_handle_message(static_cast<int32_t>(SimMessageHash::SetDebugProperties),
                           sizeof(scales), reinterpret_cast<const uint8_t*>(&scales));

        AddBuildingHeatExchangeMessage add{};
        add.callbackIdx = -1;
        add.elemIdx = oxygen;
        add.mass = 100.0f;
        add.temperature = 400.0f;
        add.thermalConductivity = 0.0f;
        add.overheatTemperature = 1.0e9f;
        add.minX = 1;
        add.minY = 1;
        add.maxX = 2;
        add.maxY = 2;
        sim_handle_message(static_cast<int32_t>(SimMessageHash::AddBuildingHeatExchange),
                           sizeof(add), reinterpret_cast<const uint8_t*>(&add));

        Tick(ew, &evis);
        const GameDataUpdate* u = Tick(ew, &evis);
        int32_t handle = -1;
        if (u && u->numBuildingTemperatures >= 1) {
          handle = u->buildingTemperatures[u->numBuildingTemperatures - 1].handle;
        }
        Check(handle >= 0, "the world-environment sink probe building has a usable sim handle");
        if (handle < 0) continue;

        ext::SetEnvironmentTemperatureMessage global{};
        global.kelvin = 50.0f;
        sim_handle_message(ext::kSetEnvironmentTemperature, static_cast<int>(sizeof(global)),
                           reinterpret_cast<const uint8_t*>(&global));
        if (cases[ci].send_world) {
          ext::SetWorldEnvironmentMessage env{};
          env.worldIndex = 0;
          env.sinkKelvin = cases[ci].world_sink;
          send_env(env);
        }
        ext::SetBuildingRadiationMessage rad{};
        rad.handle = handle;
        rad.radiationFactor = 1.0f;
        rad.surfaceAreaM2 = 0.0f;
        sim_handle_message(ext::kSetBuildingRadiation, static_cast<int>(sizeof(rad)),
                           reinterpret_cast<const uint8_t*>(&rad));

        // One tick for the queue -- all three messages are queued, so the frame they are sent
        // in still runs with the old values.
        u = Tick(ew, &evis);
        const float t_before = (u && u->numBuildingTemperatures >= 1)
                                   ? u->buildingTemperatures[u->numBuildingTemperatures - 1]
                                         .temperature
                                   : 0.0f;
        for (int i = 0; i < 5; ++i) u = TickSunlit(ew, &evis, 0.0f);
        const float t_after = (u && u->numBuildingTemperatures >= 1)
                                  ? u->buildingTemperatures[u->numBuildingTemperatures - 1]
                                        .temperature
                                  : 0.0f;
        shed[ci] = static_cast<double>(t_before) - static_cast<double>(t_after);
        printf("  %-55s body %.4f -> %.4f K (shed %.4f K)\n", cases[ci].name, t_before, t_after,
               shed[ci]);
      }
      Check(shed[0] > 0.0, "a building radiating against the global 50 K sink cools");
      Check(shed[1] > 0.0 && shed[1] < shed[0],
            "a world whose own sink is 350 K sheds LESS than the same building against the "
            "global 50 K -- the per-world record is what the term read");
      Check(shed[2] == shed[0],
            "a world record with sinkKelvin 0 falls back to the global EXACTLY: a fallback "
            "that used 0 K instead would shed more, and this is what catches it");
    }

    // ---- 2. solar absorption on a building, and Kirchhoff ---------------------------------
    //
    // The sink is set EQUAL to the body temperature so the radiation term cannot fire
    // (`t_body > t_sink` is strict), which leaves absorption as the only path and makes the
    // body's own rise the measurement.
    //
    // Closed form per substep: 0.001 * irradiance * sunFraction * radiationFactor * area * dt,
    // with area the building's own cell count and dt one substep. The factor IS the
    // absorptivity -- that is the Kirchhoff claim, and the half-emissivity case is what tests
    // it rather than merely asserting it.
    {
      struct SolarCase {
        const char* name;
        float lux;        // what the region reports this frame
        float factor;     // emissivity, and therefore absorptivity
        bool expect_rise;
      };
      const SolarCase cases[] = {
          {"full sun, black body", 80000.0f, 1.0f, true},
          {"full sun, half emissivity", 80000.0f, 0.5f, true},
          {"night", 0.0f, 1.0f, false},
      };
      constexpr float kFullSunLux = 80000.0f;
      constexpr float kIrradiance = 600.0f;  // W/m^2, about Mars at the top of its atmosphere
      constexpr float kBodyK = 300.0f;
      constexpr int kFrames = 10;
      double rise[3] = {0, 0, 0};
      for (int ci = 0; ci < 3; ++ci) {
        World ew;
        ew.Init(5, 5, vacuum, 0.0f, 100.0f);
        Boot(t, ew);
        DefineOneWorld(ew.width, ew.height);
        std::vector<uint8_t> evis;

        DebugProperties scales{};
        scales.buildingTemperatureScale = 0.001f;
        scales.buildingToBuildingTemperatureScale = 0.001f;
        sim_handle_message(static_cast<int32_t>(SimMessageHash::SetDebugProperties),
                           sizeof(scales), reinterpret_cast<const uint8_t*>(&scales));

        AddBuildingHeatExchangeMessage add{};
        add.callbackIdx = -1;
        add.elemIdx = oxygen;
        add.mass = 10.0f;
        add.temperature = kBodyK;
        add.thermalConductivity = 0.0f;
        add.overheatTemperature = 1.0e9f;
        add.minX = 1;
        add.minY = 1;
        add.maxX = 2;
        add.maxY = 2;  // one cell, so the area is 1 m^2
        sim_handle_message(static_cast<int32_t>(SimMessageHash::AddBuildingHeatExchange),
                           sizeof(add), reinterpret_cast<const uint8_t*>(&add));

        Tick(ew, &evis);
        const GameDataUpdate* u0 = Tick(ew, &evis);
        int32_t handle = -1;
        if (u0 && u0->numBuildingTemperatures >= 1) {
          handle = u0->buildingTemperatures[u0->numBuildingTemperatures - 1].handle;
        }
        Check(handle >= 0, "the solar probe building has a usable sim handle");
        if (handle < 0) continue;

        ext::SetWorldEnvironmentMessage env{};
        env.worldIndex = 0;
        env.sinkKelvin = kBodyK;  // equal, so radiation cannot fire and absorption is alone
        env.peakIrradianceWPerM2 = kIrradiance;
        env.fullSunLux = kFullSunLux;
        send_env(env);
        ext::SetBuildingRadiationMessage rad{};
        rad.handle = handle;
        rad.radiationFactor = cases[ci].factor;
        rad.surfaceAreaM2 = 0.0f;
        sim_handle_message(ext::kSetBuildingRadiation, static_cast<int>(sizeof(rad)),
                           reinterpret_cast<const uint8_t*>(&rad));

        const GameDataUpdate* u = TickSunlit(ew, &evis, cases[ci].lux);
        const float t_before = (u && u->numBuildingTemperatures >= 1)
                                   ? u->buildingTemperatures[u->numBuildingTemperatures - 1]
                                         .temperature
                                   : 0.0f;
        for (int i = 0; i < kFrames; ++i) u = TickSunlit(ew, &evis, cases[ci].lux);
        const float t_after = (u && u->numBuildingTemperatures >= 1)
                                  ? u->buildingTemperatures[u->numBuildingTemperatures - 1]
                                        .temperature
                                  : 0.0f;
        rise[ci] = static_cast<double>(t_after) - static_cast<double>(t_before);
        // The body's heat capacity: 10 kg of oxygen's specific heat, read from the sim rather
        // than restated, by dividing the kilojoules the closed form says arrived.
        const double body_hc = 10.0 * static_cast<double>(t.At(oxygen)->specificHeatCapacity);
        const double expected_kj = 0.001 * static_cast<double>(kIrradiance) *
                                   (static_cast<double>(cases[ci].lux) /
                                    static_cast<double>(kFullSunLux)) *
                                   static_cast<double>(cases[ci].factor) * 1.0 * 0.2 * kFrames;
        printf("  %-30s body %.4f -> %.4f K (rise %.5f K, closed form %.5f K)\n", cases[ci].name,
               t_before, t_after, rise[ci], expected_kj / body_hc);
        if (cases[ci].expect_rise) {
          const double expected_k = expected_kj / body_hc;
          Check(rise[ci] > expected_k * 0.98 && rise[ci] < expected_k * 1.02,
                "a sunlit building warms by the closed form 0.001 * irradiance * sunFraction * "
                "factor * area * dt");
        } else {
          Check(rise[ci] == 0.0,
                "a building on the night side absorbs exactly nothing -- TimeOfDay zeroes the "
                "lux and the sim forms the fraction from it, so night needs no code");
        }
      }
      Check(rise[0] > 0.0 && rise[1] > 0.0 &&
                rise[1] > rise[0] * 0.48 && rise[1] < rise[0] * 0.52,
            "KIRCHHOFF: halving the building's EMISSIVITY halves what it absorbs, because a "
            "grey body's absorptivity is its emissivity -- this is why no second per-building "
            "field exists");
    }

    // ---- 3. solar heating of a cell, and the ledger --------------------------------------
    //
    // A granite tile under an open column. `ComputeSunlight` writes a cell's exposure BEFORE
    // that cell absorbs the beam and a solid then stops it, so the first solid in a column is
    // lit and everything below it is dark -- which is exactly "the ground warms in the sun".
    {
      // BURIED, WITH ONE SHAFT, AND THE REASON IS THE LEDGER ASSERTION BELOW. Every building
      // any earlier arm in this file registered is still registered -- `AllocateCells` does not
      // clear the building registry -- and the world record this arm sends applies to world 0,
      // which is all of them. In an open world those buildings absorb sunlight too and the
      // bucket delta is theirs as much as this cell's: measured at 7.685 kJ against this
      // cell's 1.085 kJ, which is six leftover probes at their own factors, exactly.
      //
      // So the world is solid rock with a single vacuum shaft. Rock's light absorption is 1, so
      // every buried cell reads 0 and every leftover building sits in rock at exposure 0. The
      // only thing in this world that can absorb anything is the tile at the bottom of the
      // shaft, which makes the bucket delta a statement about that tile alone.
      World ew;
      ew.Init(5, 5, we_granite, 500.0f, 100.0f);
      const int lit_cell = 1 * 5 + 2;    // game (2,1): the tile at the bottom of the shaft
      const int dark_cell = 0 * 5 + 2;   // game (2,0): under it, in its shadow
      for (int yy = 2; yy <= 4; ++yy) {  // the shaft itself, open to the sky
        ew.element[yy * 5 + 2] = vacuum;
        ew.mass[yy * 5 + 2] = 0.0f;
      }
      ew.mass[lit_cell] = 100.0f;
      ew.temperature[lit_cell] = 200.0f;
      ew.mass[dark_cell] = 100.0f;
      ew.temperature[dark_cell] = 200.0f;
      Boot(t, ew);
      DefineOneWorld(ew.width, ew.height);
      std::vector<uint8_t> evis;

      constexpr float kFullSunLux = 80000.0f;
      constexpr float kIrradiance = 600.0f;
      constexpr float kAbsorptivity = 0.9f;
      constexpr int kFrames = 10;

      ext::SetWorldEnvironmentMessage env{};
      env.worldIndex = 0;
      // The sink is pinned to the tiles' own 200 K so the OUTWARD term (arm 5) cannot fire and
      // absorption is alone in the measurement -- the same isolation arm 2 uses on a building,
      // and it is needed here for the same reason: `solarAbsorptivity` is the emissivity too,
      // so declaring one declares both. `t_body > t_sink` is strict, and what the tile gains
      // above 200 K it re-radiates at about 4e-5 kJ over these ten frames, which is four orders
      // of magnitude under the tolerance below.
      env.sinkKelvin = 200.0f;
      env.peakIrradianceWPerM2 = kIrradiance;
      env.fullSunLux = kFullSunLux;
      env.solarAbsorptivity = kAbsorptivity;
      send_env(env);

      // Two ticks: one for the queued message, one for the sunlight texture, which is computed
      // at Project and is therefore one frame behind the offsets that define the world.
      TickSunlit(ew, &evis, kFullSunLux);
      const GameDataUpdate* eu = TickSunlit(ew, &evis, kFullSunLux);
      double base[kEnergyLedgerFields] = {0};
      const bool have_ledger =
          sim_debug_energy_ledger &&
          sim_debug_energy_ledger(base, kEnergyLedgerFields) >= kEnergyLedgerFields;
      const float lit_before = eu ? eu->temperature[lit_cell] : 0.0f;
      const float dark_before = eu ? eu->temperature[dark_cell] : 0.0f;
      for (int i = 0; i < kFrames; ++i) eu = TickSunlit(ew, &evis, kFullSunLux);
      const float lit_after = eu ? eu->temperature[lit_cell] : 0.0f;
      const float dark_after = eu ? eu->temperature[dark_cell] : 0.0f;

      const double hc = 100.0 * static_cast<double>(t.At(we_granite)->specificHeatCapacity);
      const double expected_kj =
          0.001 * static_cast<double>(kIrradiance) * static_cast<double>(kAbsorptivity) * 0.2 *
          kFrames;
      // MEASURED AGAINST THE TILE IN ITS OWN SHADOW, not against its own starting temperature.
      // Both tiles are 100 kg of granite that started at 200 K in the same rock; the only
      // difference between them is that one is lit. Their absolute temperatures are not the
      // measurement and must not be: this world still holds every building every earlier arm
      // registered, and those exchange heat with the cells they sit in, so both tiles drift
      // together by many kelvin over ten frames. The first version of this arm asserted on the
      // lit tile's own rise and read -15.5 K.
      const double expected_k = expected_kj / hc;
      const double differential = static_cast<double>(lit_after) - static_cast<double>(dark_after);
      const double started_apart =
          static_cast<double>(lit_before) - static_cast<double>(dark_before);
      printf("  lit tile %.4f -> %.4f K, shadowed tile %.4f -> %.4f K; lit is %.5f K warmer "
             "(closed form %.5f)\n",
             lit_before, lit_after, dark_before, dark_after, differential - started_apart,
             expected_k);
      Check(differential - started_apart > expected_k * 0.8 &&
                differential - started_apart < expected_k * 1.2,
            "a sunlit ground tile pulls away from the tile in its own shadow by the closed "
            "form -- the day/night swing is emergent, not scripted");
      if (have_ledger) {
        double now[kEnergyLedgerFields] = {0};
        sim_debug_energy_ledger(now, kEnergyLedgerFields);
        const double charged = now[36] - base[36];
        printf("  absorbed_from_environment charged %+.5f kJ over %d frames\n", charged, kFrames);
        // THE EXACT MEASUREMENT, and the reason the world is buried: the only cell in it that
        // can absorb anything is the one at the bottom of the shaft, so this bucket is that
        // tile's absorption and nothing else. It is also what says the shadowed tile takes
        // nothing -- a factor-1 solid takes the whole beam, so the surface is lit and the
        // interior is not.
        Check(charged > expected_kj * 0.98 && charged < expected_kj * 1.02,
              "and it is CHARGED, to the closed form: sunlight is a named boundary crossing "
              "with a star on the far side, not energy the sim invented");
      }
    }

    // ---- 4. the surface atmosphere boundary -----------------------------------------------
    //
    // A vacuum cell open to the sky fills toward the planet's pressure; a solid and a liquid
    // are refused; a cell holding somebody else's gas keeps its mass and takes the planet's
    // temperature. Every one of those is a decision the kernel makes explicitly, and three of
    // them are refusals -- the kind a test that only looked at the happy path would miss.
    {
      constexpr float kSurfaceKPa = 2.47f;   // Stationeers' Mars, near enough
      constexpr float kSurfaceK = 240.0f;
      constexpr float kRate = 0.25f;
      constexpr int kFrames = 20;

      // THREE SHAFTS IN SOLID ROCK, one per case, and the shafts are what make the refusals
      // testable at all. The boundary only reaches a cell with a path to the sky, and a cell
      // with a path to the sky is a cell whose gas can move -- so a "the mass did not change"
      // assertion on a single open cell measures ONI's own flow, not this kernel. The first
      // run of this arm did exactly that and read 1.0 kg of oxygen as 0.52 kg. Each case
      // therefore gets its own sealed column and the assertions are on COLUMN TOTALS, which
      // flow inside a column cannot change and this kernel can.
      World ew;
      ew.Init(7, 6, we_granite, 500.0f, 260.0f);
      const int kVacX = 1, kLiqX = 3, kGasX = 5;
      auto cell_at = [](int x, int y) { return y * 7 + x; };
      for (int x : {kVacX, kLiqX, kGasX}) {
        for (int y = 1; y <= 5; ++y) {
          ew.element[cell_at(x, y)] = vacuum;
          ew.mass[cell_at(x, y)] = 0.0f;
          ew.temperature[cell_at(x, y)] = 0.0f;
        }
      }
      // Cold water, well under its boiling point at this pressure, at the bottom of its shaft.
      ew.element[cell_at(kLiqX, 1)] = we_water;
      ew.mass[cell_at(kLiqX, 1)] = 500.0f;
      ew.temperature[cell_at(kLiqX, 1)] = 280.0f;
      // Somebody's oxygen at the bottom of the third.
      ew.element[cell_at(kGasX, 1)] = oxygen;
      ew.mass[cell_at(kGasX, 1)] = 1.0f;
      ew.temperature[cell_at(kGasX, 1)] = 300.0f;
      Boot(t, ew);
      DefineOneWorld(ew.width, ew.height);
      std::vector<uint8_t> evis;

      ext::SetWorldEnvironmentMessage env{};
      env.worldIndex = 0;
      env.surfacePressureKPa = kSurfaceKPa;
      env.boundaryTemperatureK = kSurfaceK;
      env.boundaryRate = kRate;
      env.boundaryElement = we_co2;
      send_env(env);

      // A column's total of one element, which is the quantity flow inside a shaft cannot
      // change and the boundary can.
      auto column_mass = [&](const GameDataUpdate* u, int x, uint16_t elem) {
        double total = 0.0;
        if (!u) return total;
        for (int y = 0; y < 6; ++y) {
          const int ci2 = cell_at(x, y);
          if (u->elementIdx[ci2] == elem) total += static_cast<double>(u->mass[ci2]);
        }
        return total;
      };

      TickSunlit(ew, &evis, 0.0f);
      const GameDataUpdate* eu = TickSunlit(ew, &evis, 0.0f);
      double mbase[kMassLedgerFields] = {0};
      const bool have_mass =
          sim_debug_ledger && sim_debug_ledger(mbase, kMassLedgerFields) >= kMassLedgerFields;
      const double water_before = column_mass(eu, kLiqX, we_water);
      const double oxygen_before = column_mass(eu, kGasX, oxygen);
      const float solid_mass_before = eu ? eu->mass[cell_at(0, 1)] : 0.0f;
      for (int i = 0; i < kFrames; ++i) eu = TickSunlit(ew, &evis, 0.0f);

      // The target the kernel is driving toward, recomputed here from PV = nRT rather than
      // read back from the sim, so this is an independent opinion and not an echo.
      //
      // STATED, not read from the table under test -- the same rule the pressure oracle
      // earlier in this file is built on. CO2 is not diatomic so Klei's own field happens to
      // be right here, but an oracle that derives its expectation from the thing it is
      // checking is not an oracle, and that is exactly how a 1000x unit error once survived.
      constexpr double kMolarMassCO2GPerMol = 44.0095;
      const double moles = static_cast<double>(kSurfaceKPa) * 1000.0 * 1.0 / (8.3144 * kSurfaceK);
      const double target = moles * kMolarMassCO2GPerMol / 1000.0;
      const float vac_mass = eu ? eu->mass[cell_at(kVacX, 1)] : 0.0f;
      printf("  vacuum shaft filled to %.6f kg/cell against a PV=nRT target of %.6f kg at "
             "%.2f kPa / %.0f K\n",
             vac_mass, target, kSurfaceKPa, kSurfaceK);
      Check(static_cast<double>(vac_mass) > target * 0.8 &&
                static_cast<double>(vac_mass) <= target * 1.05,
            "a sky-exposed vacuum cell fills toward the planet's own pressure, and stops there "
            "rather than running away");
      Check(eu && eu->elementIdx[cell_at(kVacX, 1)] == we_co2,
            "and it fills with the planet's element, not with whatever was nearest");

      Check(eu && eu->mass[cell_at(0, 1)] == solid_mass_before,
            "a SOLID tile is the planet's crust, not its atmosphere: the boundary does not "
            "push gas into rock");

      const double water_after = column_mass(eu, kLiqX, we_water);
      const double oxygen_after = column_mass(eu, kGasX, oxygen);
      printf("  water column %.4f -> %.4f kg, oxygen column %.6f -> %.6f kg\n", water_before,
             water_after, oxygen_before, oxygen_after);
      Check(eu && eu->elementIdx[cell_at(kLiqX, 1)] == we_water,
            "the cell of water is still water: the boundary does not push gas into a liquid");
      Check(water_after > water_before * 0.999,
            "and not a gram of it is destroyed -- a boundary that overwrote a liquid would "
            "read as matter vanishing, which is the failure this refusal exists to prevent");
      Check(oxygen_after > oxygen_before * 0.999,
            "a colony's OXYGEN survives contact with the planet in full: the planet does not "
            "get to overwrite somebody else's gas and call it a boundary condition");
      const float foreign_t = eu ? eu->temperature[cell_at(kGasX, 1)] : 0.0f;
      printf("  the oxygen's own cell: 300.00 -> %.2f K (planet at %.0f K)\n", foreign_t,
             kSurfaceK);
      Check(foreign_t < 300.0f && foreign_t >= kSurfaceK,
            "but it IS in contact with the planet: its temperature is pulled toward the "
            "planet's and never past it");

      if (have_mass) {
        double mnow[kMassLedgerFields] = {0};
        sim_debug_ledger(mnow, kMassLedgerFields);
        const double charged = mnow[16] - mbase[16];
        printf("  atmosphere_boundary charged %+.6f kg\n", charged);
        Check(charged > 0.0,
              "the matter the planet supplied is CHARGED to its own ledger bucket: an infinite "
              "source is a decision, and a ledger that did not say so would be reporting "
              "conservation it does not have");
      }
    }

    // ---- 5. the ground radiates back, and Kirchhoff on a CELL ------------------------------
    //
    // Arm 3's twin, outwards. Without this term the solar one is a RATCHET: a lit tile gains
    // and never gives back, so a planet's crust cooks over a few hundred cycles. With it, a
    // surface temperature is a balance rather than a number somebody typed, which is the whole
    // claim Mod 3 makes about a day.
    //
    // ISOLATED FROM ARM 3'S TERM BY LEAVING THE SUN OUT: `peakIrradianceWPerM2` and
    // `fullSunLux` stay 0, so `wants_solar` is false and only the outward term runs. The
    // sunlight TEXTURE is unaffected by that -- it is geometry, not brightness
    // (`currentSunlightIntensity` is measurably not an input to it, see sim/textures.h), so
    // the shaft is still lit and the buried rock is still dark.
    {
      struct RadCase {
        const char* name;
        float absorptivity;  // and therefore the emissivity -- that is the claim under test
        float sink;
        bool expect_cool;
      };
      const RadCase cases[] = {
          {"grey ground, 100 K sky", 0.9f, 100.0f, true},
          {"half emissivity, same sky", 0.45f, 100.0f, true},
          {"sky HOTTER than the ground", 0.9f, 500.0f, false},
      };
      constexpr float kGroundK = 400.0f;
      constexpr double kSigmaKW = 5.670374419e-11;  // stated, not read from the code under test
      double charged[3] = {0, 0, 0};
      double expected[3] = {0, 0, 0};
      for (int ci = 0; ci < 3; ++ci) {
        // BURIED, WITH ONE SHAFT, for arm 3's reason and one more. The buckets are per world
        // and every building an earlier arm registered is still registered, but those charge
        // `radiated_to_environment` (field 15) and this asserts on `cell_radiated_to_environment`
        // (field 38) -- the two are separate buckets precisely so the colony's waste heat and
        // the planet's own ground cannot be confused for one another. The burial is still what
        // makes this cell the only cell in the world that can radiate at all.
        //
        // THE WHOLE WORLD IS AT 400 K, which is the one thing this arm needs that arm 3 did
        // not: conduction must not move the tile between the frame the temperature is read and
        // the frame the flux is computed from it, or the closed form is evaluated at the wrong
        // temperature. Uniform means conduction has nothing to do.
        World ew;
        ew.Init(5, 5, we_granite, 500.0f, kGroundK);
        const int lit_cell = 1 * 5 + 2;  // game (2,1): the tile at the bottom of the shaft
        for (int yy = 2; yy <= 4; ++yy) {
          ew.element[yy * 5 + 2] = vacuum;
          ew.mass[yy * 5 + 2] = 0.0f;
        }
        ew.mass[lit_cell] = 100.0f;
        Boot(t, ew);
        DefineOneWorld(ew.width, ew.height);
        std::vector<uint8_t> evis;

        ext::SetWorldEnvironmentMessage env{};
        env.worldIndex = 0;
        env.sinkKelvin = cases[ci].sink;
        env.solarAbsorptivity = cases[ci].absorptivity;
        send_env(env);

        // One tick for the queued message, one for the sunlight texture (computed at Project,
        // so it is one frame behind the offsets that define the world).
        TickSunlit(ew, &evis, 0.0f);
        const GameDataUpdate* eu = TickSunlit(ew, &evis, 0.0f);
        double base[kEnergyLedgerFields] = {0};
        const bool have_ledger =
            sim_debug_energy_ledger &&
            sim_debug_energy_ledger(base, kEnergyLedgerFields) >= kEnergyLedgerFields;
        const double t_before = eu ? static_cast<double>(eu->temperature[lit_cell]) : 0.0;

        // ONE frame, deliberately. The flux goes as T^4, so over many frames the closed form
        // would have to be integrated rather than evaluated; over one substep it is exact,
        // because the kernel runs at the TOP of the substep and therefore sees precisely the
        // temperature published at the end of the last one.
        eu = TickSunlit(ew, &evis, 0.0f);
        const double t_after = eu ? static_cast<double>(eu->temperature[lit_cell]) : 0.0;

        const double ts = static_cast<double>(cases[ci].sink);
        expected[ci] = cases[ci].expect_cool
                           ? kSigmaKW * static_cast<double>(cases[ci].absorptivity) *
                                 (t_before * t_before * t_before * t_before - ts * ts * ts * ts) *
                                 0.2
                           : 0.0;
        if (have_ledger) {
          double now[kEnergyLedgerFields] = {0};
          sim_debug_energy_ledger(now, kEnergyLedgerFields);
          charged[ci] = now[38] - base[38];
        }
        printf("  %-32s tile %.4f -> %.4f K, cell_radiated charged %+.6f kJ (closed form "
               "%.6f)\n",
               cases[ci].name, t_before, t_after, charged[ci], expected[ci]);
        if (!have_ledger) continue;
        if (cases[ci].expect_cool) {
          Check(t_after < t_before,
                "a sky-exposed ground tile hotter than its sky COOLS -- the solar term is no "
                "longer a one-way ratchet");
          Check(charged[ci] > expected[ci] * 0.98 && charged[ci] < expected[ci] * 1.02,
                "and it is charged to its own bucket at sigma * absorptivity * (T^4 - Tsink^4) "
                "* dt, with the lit fraction as the view factor");
        } else {
          Check(t_after == static_cast<double>(static_cast<float>(t_before)),
                "a ground tile COLDER than its sky is left alone: the sky is a sink, never a "
                "back-door source -- only the star may add energy, and that is metered apart");
          Check(charged[ci] == 0.0,
                "and nothing at all is charged for it");
        }
      }
      Check(charged[0] > 0.0 && charged[1] > 0.0 && charged[1] > charged[0] * 0.48 &&
                charged[1] < charged[0] * 0.52,
            "KIRCHHOFF ON A CELL: halving the world's solar ABSORPTIVITY halves what its "
            "ground EMITS, because for a grey body they are one number -- which is why the "
            "message carries no separate emissivity field");
    }
  }

  printf("\n=== vftest: the planetary latent accumulator (SIM_ExtWorldLatentEnergy) ===\n");
  {
    // Stationeers' planetary atmosphere accumulates the energy its own atmosphere's phase
    // changes release, and reads it back as a temperature offset summed into ambient. This is
    // the half of that the DLL owns: the joules, per world, and no policy.
    //
    // FIVE RUNS OF THE SAME WORLD, because a counter that only ever goes up proves nothing
    // about what it is counting. One counts, one is refused for having no planet, one is
    // refused for having no sky, one runs the transition BACKWARDS and must come out
    // negative, and one pushes `sim.latent_fusion` so the freeze that follows the
    // condensation is billed as well.
    auto send_env = [](const ext::SetWorldEnvironmentMessage& m) {
      sim_handle_message(ext::kSetWorldEnvironment, static_cast<int>(sizeof(m)),
                         reinterpret_cast<const uint8_t*>(&m));
    };
    auto sim_ext_latent = reinterpret_cast<int32_t (*)(int32_t, double*)>(
        GetProcAddress(g_sim, "SIM_ExtWorldLatentEnergy"));
    Check(sim_ext_latent != nullptr, "SIM_ExtWorldLatentEnergy is exported");
    if (sim_ext_latent != nullptr) {
      uint16_t la_granite = 0, la_steam = 0, la_water = 0, la_ice = 0;
      if (!Resolve(t, kGranite, "Granite", &la_granite)) return 1;
      if (!Resolve(t, simhost::kSteam, "Steam", &la_steam)) return 1;
      if (!Resolve(t, simhost::kWater, "Water", &la_water)) return 1;
      if (!Resolve(t, simhost::kIce, "Ice", &la_ice)) return 1;

      // STATED, not read out of the table under test. 2.256e6 J/kg is water's real enthalpy of
      // vaporization; what matters to this arm is only that the sim multiplies by the number it
      // was given, which an oracle that re-read the attribute could not tell.
      const float kLatentJPerKg = 2256000.0f;
      const float curve[5] = {1.0f, 1.0f, 273.15f, 647.0f, kLatentJPerKg};
      // PUSHED AFTER EVERY `Boot`, NOT ONCE HERE. `Elements_CreateTable` rebuilds the element
      // table and the attribute registry hanging off it, so a curve written before a boot is a
      // curve the run under test does not have -- which is exactly how this arm first read a
      // flat zero.
      auto push_curve = [&]() {
        const int32_t curve_attr = sim_ext_attr_index(ext::kAttrPhaseCurve);
        Check(curve_attr >= 0, "sim.phase_curve resolves by name");
        if (curve_attr < 0) return;
        for (int32_t id : {static_cast<int32_t>(simhost::kSteam),
                           static_cast<int32_t>(simhost::kWater)}) {
          for (int32_t k = 0; k < 5; ++k) {
            uint32_t bits = 0;
            memcpy(&bits, &curve[k], 4);
            SetElementAttributeBits(curve_attr, id, k, bits);
          }
        }
      };
      // `sim.can_condense` is deliberately NOT pushed. The condensation rule and this
      // accumulator are separate claims about the same substance, and an arm that pushed both
      // would not be able to say which of the two it had measured.
      printf("  element indices: steam %u water %u ice %u granite %u\n", la_steam, la_water,
             la_ice, la_granite);

      // Physical latent heats, D3: water's enthalpy of fusion, stated for the same reason.
      const float kFusionJPerKg = 333550.0f;
      auto push_fusion = [&]() {
        const int32_t fusion_attr = sim_ext_attr_index(ext::kAttrLatentFusion);
        Check(fusion_attr >= 0, "sim.latent_fusion resolves by name");
        if (fusion_attr < 0) return;
        uint32_t bits = 0;
        memcpy(&bits, &kFusionJPerKg, 4);
        for (int32_t id : {static_cast<int32_t>(simhost::kSteam),
                           static_cast<int32_t>(simhost::kWater),
                           static_cast<int32_t>(simhost::kIce)}) {
          SetElementAttributeBits(fusion_attr, id, 0, bits);
        }
      };
      struct LatentCase {
        const char* name;
        bool send_env;      // does this world declare a surface atmosphere
        bool shaded;        // is the transitioning cell under rock
        bool boil;          // liquid -> gas instead of gas -> liquid
        bool fusion;        // is sim.latent_fusion pushed as well as the curve
      };
      // THE NO-PLANET RUN GOES FIRST, and that ordering is load-bearing rather than tidy: a
      // `kSetWorldEnvironment` record SURVIVES a re-boot of the world in this harness (nothing
      // in the allocate path clears `World::world_environments_`), so a run that follows one
      // which sent a record is not a world without a record at all. Written down because the
      // arm read a full charge on a world it believed had no planet, and that was the harness
      // telling the truth about the DLL's lifecycle rather than a bug in the kernel.
      const LatentCase cases[] = {
          {"condensation on a world that is NOT a planet", false, false, false, false},
          {"sky-exposed steam condensing on a planet", true, false, false, false},
          {"the same condensation under rock, out of the sky", true, true, false, false},
          {"sky-exposed water BOILING on a planet", true, false, true, false},
          {"the same condensation, with sim.latent_fusion", true, false, false, true},
      };
      const int kW = 5, kH = 5;
      auto la_cell = [&](int x, int y) { return y * kW + x; };
      for (const LatentCase& c : cases) {
        // Granite everywhere: the heat bath that drives the transition, and the walls that stop
        // the fluid wandering off to transition somewhere with a different sky.
        World lw;
        // 600 K for the boil run, not more: at 1200 K the granite itself does something
        // (a 120 kg lump of another element appears in the pocket) and the run stops being
        // about water. Water boils at 372.15 K; 600 K is bath enough.
        lw.Init(kW, kH, la_granite, 1000.0f, c.boil ? 600.0f : 100.0f);
        // The top row is never simulated (`World::SetActiveRegions`), so it is emptied rather
        // than used: a solid there would break the beam and shade the cell under test.
        for (int x = 0; x < kW; ++x) {
          lw.element[la_cell(x, kH - 1)] = vacuum;
          lw.mass[la_cell(x, kH - 1)] = 0.0f;
          lw.temperature[la_cell(x, kH - 1)] = 0.0f;
        }
        // THE PROBE CELL STARTS EMPTY AND THE FLUID IS INJECTED AFTER THE WARM-UP TICKS, which
        // is the whole reason this arm is shaped this way. The sunlight texture is computed at
        // Project and read a frame later, so a cell set up in the world and left to conduct
        // transitions on tick 1 -- before any sky exists -- and the accumulator correctly reads
        // zero for a reason that has nothing to do with what is being tested. Ask the first
        // version of this arm, which read a flat zero four times.
        const int probe = c.boil ? la_cell(2, 1) : (c.shaded ? la_cell(2, 2) : la_cell(2, 3));
        for (int y = (c.boil ? 1 : (c.shaded ? 2 : 3)); y <= kH - 2; ++y) {
          if (c.shaded && y > 2) break;  // leave the rock cap that shades the probe
          lw.element[la_cell(2, y)] = vacuum;
          lw.mass[la_cell(2, y)] = 0.0f;
          lw.temperature[la_cell(2, y)] = 0.0f;
        }
        Boot(t, lw);
        DefineOneWorld(lw.width, lw.height);
        push_curve();
        if (c.fusion) push_fusion();
        std::vector<uint8_t> lvis;

        if (c.send_env) {
          ext::SetWorldEnvironmentMessage env{};
          env.worldIndex = 0;
          // Pressure and element declare that this world HAS an atmosphere, which is the gate.
          // `boundaryRate` and `solarAbsorptivity` stay 0 so that the boundary and the two
          // radiative terms do nothing at all: the only physics in this run is conduction and
          // the state change it drives.
          env.surfacePressureKPa = 2.47f;
          env.boundaryElement = la_water;
          send_env(env);
        }

        // Three ticks before anything is injected: one for the queued environment record, one
        // for the sunlight texture (computed at Project, so a frame behind the world offsets
        // that define it), one to be sure.
        for (int i = 0; i < 3; ++i) TickSunlit(lw, &lvis, 0.0f);
        double base = 0.0;
        Check(sim_ext_latent(0, &base) == 1, "the accumulator reads back for world 0");

        const float inject_mass = c.boil ? 100.0f : 1.0f;
        ModifyCellMessage inject{};
        inject.cellIdx = probe;
        inject.callbackIdx = -1;
        inject.temperature = c.boil ? 350.0f : 400.0f;
        inject.mass = inject_mass;
        inject.diseaseCount = 0;
        inject.elementIdx = c.boil ? la_water : la_steam;
        inject.replaceType = 1;  // kReplaceElement (sim/cellmod.h)
        inject.diseaseIdx = 0xFF;
        inject.addSubType = 0;
        sim_handle_message(static_cast<int32_t>(SimMessageHash::ModifyCell), sizeof(inject),
                           reinterpret_cast<const uint8_t*>(&inject));

        const GameDataUpdate* lu = TickSunlit(lw, &lvis, 0.0f);
        const uint16_t elem_before = lu ? lu->elementIdx[probe] : 0;
        for (int i = 0; i < 200; ++i) lu = TickSunlit(lw, &lvis, 0.0f);
        double now = 0.0;
        sim_ext_latent(0, &now);
        const double charged = now - base;
        const uint16_t elem_after = lu ? lu->elementIdx[probe] : 0;
        const double expected =
            (c.boil ? -1.0 : 1.0) * static_cast<double>(inject_mass) *
            (static_cast<double>(kLatentJPerKg) + (c.fusion ? static_cast<double>(kFusionJPerKg)
                                                            : 0.0));
        printf("  %-52s %.3f kg %u -> %u, charged %+.6g J (a full conversion is %+.6g J)\n",
               c.name, static_cast<double>(inject_mass), elem_before, elem_after, charged,
               expected);

        Check(elem_after != elem_before,
              "the run's cell really did change phase -- without that every claim below is "
              "a claim about a transition that never happened");
        if (!c.send_env || c.shaded) {
          Check(charged == 0.0,
                c.send_env ? "a transition with rock between it and the sky is NOT the planet's "
                             "atmosphere and is not counted"
                           : "a world that has declared no atmosphere accumulates nothing, "
                             "however much of it changes phase");
          continue;
        }
        if (c.boil) {
          Check(charged < 0.0,
                "SIGNED: a liquid boiling TAKES energy out of the planet's atmosphere, so the "
                "accumulator falls -- a counter that only rises is measuring transitions, not "
                "latent heat");
          Check(charged > expected * 1.001 && charged < expected * 0.999,
                "and it is the cell's whole mass times the latent heat it was given, to within "
                "float rounding");
        } else {
          Check(charged > 0.0, "a gas condensing PUTS energy into the planet's atmosphere");
          Check(elem_after == la_ice,
                "the probe went on to FREEZE after it condensed, which is the control this run "
                "carries for free");
          if (c.fusion) {
            Check(charged > expected * 0.999 && charged < expected * 1.001,
                  "WITH sim.latent_fusion the freeze is billed too: the accumulator holds the "
                  "whole mass times vaporization PLUS fusion, the fusion read off the water");
          } else {
            Check(charged < expected * 1.001,
                  "and the freeze contributed nothing: sim.phase_curve carries an enthalpy of "
                  "VAPORIZATION, which is the wrong number for fusion, and with no "
                  "sim.latent_fusion pushed the accumulator holds the condensation alone");
          }
        }
      }

      // The export's own contract, which no run above can see.
      double twice_a = 0.0, twice_b = 0.0;
      sim_ext_latent(0, &twice_a);
      sim_ext_latent(0, &twice_b);
      Check(twice_a == twice_b,
            "READING DOES NOT ZERO: two readers sample, wait and subtract, and cannot destroy "
            "each other's window");
      Check(sim_ext_latent(0, nullptr) == 0, "a null out pointer is refused, not written");
      double absent = 12345.0;
      Check(sim_ext_latent(7, &absent) == 1 && absent == 0.0,
            "a world nobody has counted a transition for answers 0 rather than failing -- "
            "'no transitions' is a real answer");
    }
  }

  printf("\n=== vftest: the cell boil rule, through the DLL ===\n");
  {
    // `gastest` asserts `BoilingAllowed` and the ambient walk against
    // the header; this asserts the same rule reaching a running sim through the message path --
    // a curve pushed by `kSetElementAttribute`, the gate hoisted per region in
    // `StepStateChange`, and a cell that does or does not change element over 200 ticks.
    //
    // TWO RUNS OF ONE SCENE, DIFFERING ONLY IN THE CURVE. A pocket of water in a 600 K granite
    // bath: hot enough that Klei's fixed `highTemp + 3 K` fires, which the control proves by
    // boiling it away. The rule run pushes a curve whose saturation temperature at this cell's
    // ambient pressure is above the water's temperature, and the water must still be water at
    // the end. Anything that made the transition impossible for another reason would take the
    // control down with it, which is what the control is for.
    uint16_t br_granite = 0, br_water = 0, br_steam = 0;
    if (!Resolve(t, kGranite, "Granite", &br_granite)) return 1;
    if (!Resolve(t, simhost::kWater, "Water", &br_water)) return 1;
    if (!Resolve(t, simhost::kSteam, "Steam", &br_steam)) return 1;

    // a = 5e-4, b = 1 makes the curve `t_sat = P_kPa / 5e-4`, so the ~0.49 kPa a 100 kg water
    // tile presses on itself clamps t_sat straight to the critical temperature. The numbers are
    // chosen to make the REFUSAL unambiguous at a pressure this pocket really has; the physical
    // curve is Mod 2's business and is asserted where Mod 2's numbers live.
    const float br_curve[5] = {5.0e-4f, 1.0f, 273.15f, 647.0f, 2256000.0f};
    const int kW = 5, kH = 5;
    auto br_cell = [&](int x, int y) { return y * kW + x; };
    for (int run = 0; run < 2; ++run) {
      const bool with_rule = run == 1;
      World bw;
      bw.Init(kW, kH, br_granite, 1000.0f, 600.0f);
      for (int x = 0; x < kW; ++x) {
        bw.element[br_cell(x, kH - 1)] = vacuum;
        bw.mass[br_cell(x, kH - 1)] = 0.0f;
        bw.temperature[br_cell(x, kH - 1)] = 0.0f;
      }
      for (int y = 1; y <= kH - 2; ++y) {
        bw.element[br_cell(2, y)] = vacuum;
        bw.mass[br_cell(2, y)] = 0.0f;
        bw.temperature[br_cell(2, y)] = 0.0f;
      }
      Boot(t, bw);
      // Pushed after the Boot, never before it: `Elements_CreateTable` rebuilds the attribute
      // registry, as the latent arm above records.
      if (with_rule) {
        const int32_t curve_attr = sim_ext_attr_index(ext::kAttrPhaseCurve);
        Check(curve_attr >= 0, "sim.phase_curve resolves by name");
        for (int32_t id : {static_cast<int32_t>(simhost::kSteam),
                           static_cast<int32_t>(simhost::kWater)}) {
          for (int32_t k = 0; k < 5; ++k) {
            uint32_t bits = 0;
            memcpy(&bits, &br_curve[k], 4);
            SetElementAttributeBits(curve_attr, id, k, bits);
          }
        }
      }
      std::vector<uint8_t> bvis;
      for (int i = 0; i < 2; ++i) Tick(bw, &bvis);

      ModifyCellMessage inject{};
      inject.cellIdx = br_cell(2, 1);
      inject.callbackIdx = -1;
      inject.temperature = 350.0f;
      inject.mass = 100.0f;
      inject.diseaseCount = 0;
      inject.elementIdx = br_water;
      inject.replaceType = 1;  // kReplaceElement
      inject.diseaseIdx = 0xFF;
      inject.addSubType = 0;
      sim_handle_message(static_cast<int32_t>(SimMessageHash::ModifyCell), sizeof(inject),
                         reinterpret_cast<const uint8_t*>(&inject));

      const GameDataUpdate* bu = Tick(bw, &bvis);
      const uint16_t before = bu ? bu->elementIdx[br_cell(2, 1)] : 0;
      for (int i = 0; i < 200; ++i) bu = Tick(bw, &bvis);
      const uint16_t after = bu ? bu->elementIdx[br_cell(2, 1)] : 0;
      const float t_after = bu ? bu->temperature[br_cell(2, 1)] : 0.0f;
      // `before` is the cell one tick after the inject was SENT, which is the tick before it
      // lands -- a queued message takes effect on the tick after arrival -- so it reads as the
      // granite the pocket was cut from. It is printed rather than asserted for that reason.
      printf("  %-28s %u -> %u at %.1f K  (water %u, steam %u)\n",
             with_rule ? "curve registered" : "control, no curve", before, after,
             static_cast<double>(t_after), br_water, br_steam);
      if (with_rule) {
        Check(after == br_water,
              "WITH THE RULE the water is still water after 200 ticks in a 600 K bath: its "
              "saturation temperature at this cell's ambient pressure is above its own, and "
              "the boil Klei's fixed threshold wanted is refused");
        // Klei's threshold for water is `highTemp + TransitionMargin` = 375.15 K. The run
        // measures 432 K; 380 is the claim being made, which is "past it", not the reading.
        Check(t_after > 380.0f,
              "...and it is not that the cell stayed cold -- it is well past Klei's boiling "
              "point of 375.15 K and simply does not boil");
      } else {
        Check(after == br_steam,
              "CONTROL: the same cell with no curve registered boils away exactly as vanilla "
              "does, which is what makes the run above a claim about the RULE");
      }
    }
  }

  printf("\n=== vftest: the direct beam (ext::kSetWorldSun, SIM_ExtWorldSun, "
         "SIM_ExtCopySunBeam) ===\n");
  {
    // Flagship 2 / Mod 3's sun path. A world given a sun direction gets a second
    // exposure field walked along it, and its SOLAR terms read that instead of Klei's vertical
    // sky view; everything that means "open to the sky" keeps the vertical one.
    //
    // ONE WORLD, ONE PILLAR, TWO TILES, AND THE SUN MOVED ACROSS IT. A granite pillar stands in
    // the middle of a vacuum world with a ground tile on each side of it, both under open sky.
    // With the sun low in the east the western tile is in the pillar's shadow and the eastern
    // one is lit; with the sun low in the west it is the other way round. Nothing else in the
    // world changes between the two windows, so a tile that warms in the first and falls behind
    // in the second can only be following the beam -- which is the positive control a single
    // "the shaded tile stayed cold" window could not be: a term that was never switched on
    // leaves a shaded tile cold too.
    auto sim_ext_world_sun = reinterpret_cast<int32_t (*)(int32_t, float*, float*)>(
        GetProcAddress(g_sim, "SIM_ExtWorldSun"));
    auto sim_ext_copy_beam = reinterpret_cast<int32_t (*)(uint8_t*, int32_t)>(
        GetProcAddress(g_sim, "SIM_ExtCopySunBeam"));
    Check(sim_ext_world_sun != nullptr, "SIM_ExtWorldSun is exported");
    Check(sim_ext_copy_beam != nullptr, "SIM_ExtCopySunBeam is exported");
    uint16_t sb_granite = 0;
    if (!Resolve(t, kGranite, "Granite", &sb_granite)) return 1;
    uint16_t sb_glass = 0;
    if (!Resolve(t, kGlass, "Glass", &sb_glass)) return 1;
    uint16_t sb_ice = 0;
    if (!Resolve(t, kIce, "Ice", &sb_ice)) return 1;
    if (sim_ext_world_sun != nullptr && sim_ext_copy_beam != nullptr) {
      auto send_sun = [](int32_t world, float dir_x, float dir_y, int32_t enabled) {
        ext::SetWorldSunMessage m{};
        m.worldIndex = world;
        m.dirX = dir_x;
        m.dirY = dir_y;
        m.enabled = enabled;
        sim_handle_message(ext::kSetWorldSun, static_cast<int>(sizeof(m)),
                           reinterpret_cast<const uint8_t*>(&m));
      };
      auto send_env = [](const ext::SetWorldEnvironmentMessage& m) {
        sim_handle_message(ext::kSetWorldEnvironment, static_cast<int>(sizeof(m)),
                           reinterpret_cast<const uint8_t*>(&m));
      };
      constexpr int kW = 16;
      constexpr int kH = 12;
      constexpr float kFullSunLux = 80000.0f;
      constexpr float kIrradiance = 600.0f;
      constexpr float kAbsorptivity = 0.9f;
      constexpr int kFrames = 10;
      const int32_t cells = kW * kH;
      auto at = [](int x, int y) { return y * kW + x; };

      World sw;
      sw.Init(kW, kH, vacuum, 0.0f, 200.0f);
      // The pillar: column 8, rows 0..7, granite. A solid takes its whole light absorption
      // whatever it weighs, and granite's is 1, so the pillar is opaque.
      for (int y = 0; y <= 7; ++y) {
        sw.element[at(8, y)] = sb_granite;
        sw.mass[at(8, y)] = 2000.0f;
        sw.temperature[at(8, y)] = 200.0f;
      }
      const int west_tile = at(5, 0);
      const int east_tile = at(12, 0);
      for (int c : {west_tile, east_tile}) {
        sw.element[c] = sb_granite;
        sw.mass[c] = 100.0f;
        sw.temperature[c] = 200.0f;
      }
      // Two panes high in open sky, clear of every lane the low suns below take to the tiles:
      // glass (light absorption 0.1) and ice (0.33333). A solid takes its whole factor whatever
      // it weighs -- `sunglass` in diffsim, measured against Klei -- so the cell below each one
      // reads 255 * 0.9 = 229 and 255 * 2/3 = 170.
      const int glass_pane = at(2, 10);
      const int ice_pane = at(14, 10);
      sw.element[glass_pane] = sb_glass;
      sw.mass[glass_pane] = 800.0f;
      sw.temperature[glass_pane] = 200.0f;
      sw.element[ice_pane] = sb_ice;
      sw.mass[ice_pane] = 1000.0f;
      sw.temperature[ice_pane] = 200.0f;
      Boot(t, sw);
      DefineOneWorld(sw.width, sw.height);
      std::vector<uint8_t> svis;
      std::vector<uint8_t> beam(static_cast<size_t>(cells), 0);
      auto frames = [&](int n) {
        const GameDataUpdate* u = nullptr;
        for (int i = 0; i < n; ++i) u = TickSunlit(sw, &svis, kFullSunLux);
        return u;
      };
      auto all_zero = [&]() {
        for (uint8_t b : beam) {
          if (b != 0) return false;
        }
        return true;
      };

      // A clean planet for world 0. Earlier arms leave their records behind (a record survives
      // a re-boot), and one of them declares a surface atmosphere this arm's vacuum must not get.
      ext::SetWorldEnvironmentMessage env{};
      env.worldIndex = 0;
      send_env(env);

      // ---- 1. no sun: nothing to read back, and a field of zeros ---------------------------
      frames(2);
      float sx = 9.0f, sy = 9.0f;
      Check(sim_ext_world_sun(0, &sx, &sy) == 0,
            "a world never sent kSetWorldSun reports no sun, so a gate can tell 'no shade' from "
            "'no mechanism'");
      Check(sim_ext_copy_beam(nullptr, 0) == cells,
            "SIM_ExtCopySunBeam with no buffer returns the game cell count, so a caller can size one");
      std::fill(beam.begin(), beam.end(), static_cast<uint8_t>(0xAB));
      sim_ext_copy_beam(beam.data(), 3);
      Check(beam[0] == 0xAB && beam[static_cast<size_t>(cells) - 1] == 0xAB,
            "a buffer too small for the field is not written at all");
      sim_ext_copy_beam(beam.data(), cells);
      Check(all_zero(), "with no sun anywhere the direct beam is zero everywhere");

      // ---- 2. a low eastern sun: the western tile is in shade under open sky ---------------
      env.sinkKelvin = 200.0f;  // the tiles' own temperature: the outward term stays out of it
      env.peakIrradianceWPerM2 = kIrradiance;
      env.fullSunLux = kFullSunLux;
      env.solarAbsorptivity = kAbsorptivity;
      send_env(env);
      send_sun(0, 0.8660254f * 3.0f, 0.5f * 3.0f, 1);
      const GameDataUpdate* su = frames(2);
      Check(sim_ext_world_sun(0, &sx, &sy) == 1 && std::fabs(sx - 0.8660254f) < 1e-4f &&
                std::fabs(sy - 0.5f) < 1e-4f,
            "the sun the DLL holds is the one sent, normalised on receipt");
      sim_ext_copy_beam(beam.data(), cells);
      const uint8_t* sky =
          su ? static_cast<const uint8_t*>(su->propertyTextureExposedToSunlight) : nullptr;
      Check(sky != nullptr, "the frame publishes the sunlight texture");
      if (sky != nullptr) {
        printf("  sun low in the east: west tile sky %u beam %u, east tile sky %u beam %u\n",
               sky[west_tile], beam[static_cast<size_t>(west_tile)], sky[east_tile],
               beam[static_cast<size_t>(east_tile)]);
        Check(sky[west_tile] == 255 && beam[static_cast<size_t>(west_tile)] == 0,
              "SHADE UNDER OPEN SKY: with the sun low in the east the pillar shadows the western "
              "tile, whose column to the sky is still clear");
        Check(sky[east_tile] == 255 && beam[static_cast<size_t>(east_tile)] == 255,
              "the eastern tile, with nothing between it and the sun, is in full beam");
      }
      double east_gain_e = 0.0, west_gain_e = 0.0;
      {
        const GameDataUpdate* u0 = frames(1);
        const float e0 = u0 ? u0->temperature[east_tile] : 0.0f;
        const float w0 = u0 ? u0->temperature[west_tile] : 0.0f;
        const GameDataUpdate* u1 = frames(kFrames);
        east_gain_e = static_cast<double>(u1 ? u1->temperature[east_tile] : 0.0f) - e0;
        west_gain_e = static_cast<double>(u1 ? u1->temperature[west_tile] : 0.0f) - w0;
      }

      // ---- 3. the sun moves to the west: the shadow swaps sides ----------------------------
      send_sun(0, -0.8660254f, 0.5f, 1);
      frames(2);
      sim_ext_copy_beam(beam.data(), cells);
      Check(beam[static_cast<size_t>(west_tile)] == 255 &&
                beam[static_cast<size_t>(east_tile)] == 0,
            "THE POSITIVE CONTROL: with the sun low in the west the same pillar shadows the "
            "EASTERN tile and the western one is lit -- the shade follows the sun, not the grid");
      double east_gain_w = 0.0, west_gain_w = 0.0;
      {
        const GameDataUpdate* u0 = frames(1);
        const float e0 = u0 ? u0->temperature[east_tile] : 0.0f;
        const float w0 = u0 ? u0->temperature[west_tile] : 0.0f;
        const GameDataUpdate* u1 = frames(kFrames);
        east_gain_w = static_cast<double>(u1 ? u1->temperature[east_tile] : 0.0f) - e0;
        west_gain_w = static_cast<double>(u1 ? u1->temperature[west_tile] : 0.0f) - w0;
      }
      // Ten frames of one-substep frames, both tiles 100 kg of granite, sun fraction 1, beam
      // 255: the same closed form arm 3 of the world-environment block holds a lit cell to.
      const double hc = 100.0 * static_cast<double>(t.At(sb_granite)->specificHeatCapacity);
      const double expected_k = 0.001 * static_cast<double>(kIrradiance) *
                                static_cast<double>(kAbsorptivity) * 0.2 * kFrames / hc;
      const double lead_e = east_gain_e - west_gain_e;
      const double lead_w = west_gain_w - east_gain_w;
      printf("  eastern sun: east tile +%.5f K, west +%.5f K; western sun: west +%.5f K, "
             "east +%.5f K (closed form %.5f)\n",
             east_gain_e, west_gain_e, west_gain_w, east_gain_w, expected_k);
      Check(lead_e > expected_k * 0.8 && lead_e < expected_k * 1.2,
            "the tile in the eastern sun's beam out-warms the one in the pillar's shadow by the "
            "closed form, although both are under the same open sky");
      Check(lead_w > expected_k * 0.8 && lead_w < expected_k * 1.2,
            "and with the western sun the lead changes hands by the same amount: solar heating "
            "reads the beam, not the sky view");

      // ---- 4. overhead, the beam IS Klei's texture --------------------------------------
      send_sun(0, 0.0f, 1.0f, 1);
      su = frames(3);
      sim_ext_copy_beam(beam.data(), cells);
      sky = su ? static_cast<const uint8_t*>(su->propertyTextureExposedToSunlight) : nullptr;
      int differ = 0;
      for (int32_t i = 0; sky != nullptr && i < cells; ++i) {
        if (beam[static_cast<size_t>(i)] != sky[i]) ++differ;
      }
      Check(sky != nullptr && differ == 0,
            "with the sun overhead the direct beam equals the sky view in every cell: a lane "
            "offset of zero is a column");
      const uint8_t under_glass = beam[static_cast<size_t>(glass_pane - kW)];
      const uint8_t under_ice = beam[static_cast<size_t>(ice_pane - kW)];
      printf("  overhead: beam under the glass pane %u, under the ice pane %u\n", under_glass,
             under_ice);
      Check(under_glass == 229 && under_ice == 170,
            "LIGHT THROUGH A SOLID: under a glass pane the beam reads 229 and under an ice pane "
            "170 -- a solid takes its whole light absorption (0.1, 0.33333), not the whole beam");

      // ---- 5. below the horizon, and forgetting ----------------------------------------
      send_sun(0, 0.5f, -0.2f, 1);
      frames(2);
      sim_ext_copy_beam(beam.data(), cells);
      Check(sim_ext_world_sun(0, &sx, &sy) == 1 && sy < 0.0f && all_zero(),
            "a sun below the horizon is still a sun the DLL holds, and it lights nothing");
      send_sun(0, 0.0f, 0.0f, 1);
      frames(1);
      Check(sim_ext_world_sun(0, &sx, &sy) == 0,
            "a direction with no length is no sun, not a NaN walked into the beam");
      send_sun(0, 0.0f, 1.0f, 1);
      frames(1);
      send_sun(0, 0.0f, 1.0f, 0);
      frames(2);
      sim_ext_copy_beam(beam.data(), cells);
      Check(sim_ext_world_sun(0, &sx, &sy) == 0 && all_zero(),
            "enabled = 0 forgets the sun, and the frame after puts the field back to zero");

      // Leave world 0 as the next arm expects to find it.
      send_env(ext::SetWorldEnvironmentMessage{});
      frames(1);
    }
  }

  // Liquid-carried payloads. A property
  // flagged kTransportFollowsLiquidMass is an extensive amount carried by the liquid in its
  // cell: it moves when the liquid moves, and when the liquid leaves the grid its share leaves
  // on one of two streams, or -- if nobody took the record -- on the unreported total. So there
  // is ONE invariant, checked on every tick below rather than at the end:
  //
  //     on the grid + released + consumed + unreported == injected
  //
  // Checked against the pipeline's one-frame lag: a read through the
  // exports after PGD(n) sees the state after frame n, and PGD(n+1) is what hands back frame
  // n's records. So each tick's records close the PREVIOUS tick's grid read, never its own.
  //
  // One world, three things in it, each walled off from the others:
  //   * a basin: four cells of water on the floor of a ten-cell trench, free to spread;
  //   * eight single-cell pockets, one per way liquid leaves the grid, plus a control;
  //   * three extra properties -- a mod's F32 tracer switched to follow the liquid, a mod's
  //     F32 left static, and a U8 that the transport message must refuse.
  printf("\n=== vftest: Layer C -- a liquid-carried payload is conserved at every exit ===\n");
  {
    sim_shutdown();
    InitSim();
    auto sim_payload_unreported = reinterpret_cast<int32_t (*)(int32_t, int32_t, double*)>(
        GetProcAddress(g_sim, "SIM_ExtLiquidPayloadUnreported"));
    Check(sim_payload_unreported != nullptr, "SIM_ExtLiquidPayloadUnreported is exported");
    uint16_t pl_granite = 0, pl_water = 0, pl_vacuum = 0;
    if (!Resolve(t, simhost::kGranite, "Granite", &pl_granite)) return 1;
    if (!Resolve(t, simhost::kWater, "Water", &pl_water)) return 1;
    if (!Resolve(t, simhost::kVacuum, "Vacuum", &pl_vacuum)) return 1;

    const int32_t dm = sim_ext_property_index(ext::kDissolvedMassProperty);
    ext::ExtCellPropertyDesc dm_desc{};
    Check(dm > 0 && sim_ext_property_describe(dm, &dm_desc) == 1 &&
              dm_desc.type == ext::kExtF32 && dm_desc.persist == ext::kSaved &&
              dm_desc.arity == ext::kDissolvedGasLanes,
          "sim.dissolved_mass is registered first-party before any message: F32, saved, one "
          "lane per dissolved gas");
    // Registration closes at the first allocate, so the mod properties go in before Boot.
    const int32_t tracer = RegisterProperty("mod13.tracer", ext::kExtF32, ext::kSaved, 0u);
    const int32_t fixed = RegisterProperty("mod13.fixed", ext::kExtF32, ext::kSaved, 0u);
    const int32_t flag = RegisterProperty("mod13.flag", ext::kExtU8, ext::kSaved, 0u);
    Check(tracer > 0 && fixed > 0 && flag > 0, "the three mod properties register");

    constexpr int kPW = 20, kPH = 12;
    constexpr int kBasinY = 1, kPocketY = 8;
    auto at = [](int x, int y) { return static_cast<int32_t>(y * kPW + x); };
    // The pockets, by what is done to each. kBoil is 50 kg rather than 500 so one message can
    // take it past boiling, and it carries its payload on lane 3, not lane 0.
    enum Pocket { kReplace, kRemove, kConsume, kConsumeNoId, kConsumer, kControl, kUnheard,
                  kBoil, kPockets };
    auto pocket = [&](int i) { return at(2 + 2 * i, kPocketY); };

    World pw;
    pw.Init(kPW, kPH, pl_granite, 2000.0f, 300.0f);
    for (int y = kBasinY; y < kBasinY + 4; ++y) {
      for (int x = 1; x <= 10; ++x) {
        pw.element[at(x, y)] = pl_vacuum;
        pw.mass[at(x, y)] = 0.0f;
      }
    }
    for (int x = 1; x <= 4; ++x) {
      pw.element[at(x, kBasinY)] = pl_water;
      pw.mass[at(x, kBasinY)] = 1000.0f;
    }
    for (int i = 0; i < kPockets; ++i) {
      pw.element[pocket(i)] = pl_water;
      pw.mass[pocket(i)] = i == kBoil ? 50.0f : 500.0f;
      pw.temperature[pocket(i)] = i == kBoil ? 360.0f : 300.0f;
    }

    auto add = [&](int32_t cell, int32_t prop, int32_t lane, float amount) {
      ext::AddCellPropertyAmountMessage m{cell, prop, lane, amount};
      sim_handle_message(ext::kAddCellPropertyAmount, static_cast<int>(sizeof(m)),
                         reinterpret_cast<const uint8_t*>(&m));
    };
    auto transport = [&](int32_t prop, int32_t mode) {
      ext::SetCellPropertyTransportMessage m{prop, mode};
      sim_handle_message(ext::kSetCellPropertyTransport, static_cast<int>(sizeof(m)),
                         reinterpret_cast<const uint8_t*>(&m));
    };
    auto readf = [&](int32_t cell, int32_t prop, int32_t lane) {
      uint32_t bits = 0;
      if (!sim_ext_read_cell_property(cell, prop, lane, &bits)) {
        return std::numeric_limits<float>::quiet_NaN();
      }
      float v = 0.0f;
      memcpy(&v, &bits, sizeof(v));
      return v;
    };
    auto grid_sum = [&](int32_t prop, int32_t lane) {
      double s = 0.0;
      for (int32_t c = 0; c < kPW * kPH; ++c) s += readf(c, prop, lane);
      return s;
    };
    auto unreported = [&](int32_t prop, int32_t lane) {
      double u = 0.0;
      if (sim_payload_unreported == nullptr || !sim_payload_unreported(prop, lane, &u)) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      return u;
    };

    std::vector<ext::LiquidPayloadReleased> released;
    std::vector<ext::LiquidPayloadConsumed> consumed;
    std::vector<ext::ExtRefusedMessage> refused;
    int32_t consumer_handle = -1;
    auto absorb = [&](const GameDataUpdate* fr) {
      if (fr == nullptr) return;
      if (const ext::ExtPublishedStream* st = FindStream(fr, ext::kStreamLiquidPayloadReleased)) {
        const auto* r = static_cast<const ext::LiquidPayloadReleased*>(st->data);
        for (int32_t i = 0; r != nullptr && i < st->count; ++i) released.push_back(r[i]);
      }
      if (const ext::ExtPublishedStream* st = FindStream(fr, ext::kStreamLiquidPayloadConsumed)) {
        const auto* r = static_cast<const ext::LiquidPayloadConsumed*>(st->data);
        for (int32_t i = 0; r != nullptr && i < st->count; ++i) consumed.push_back(r[i]);
      }
      for (const ext::ExtRefusedMessage& r : Refusals(fr)) refused.push_back(r);
      for (int32_t i = 0; i < fr->numComponentStateChangedMessages; ++i) {
        if (fr->componentStateChangedMessages[i].callbackIdx == 4242) {
          consumer_handle = fr->componentStateChangedMessages[i].simHandle;
        }
      }
    };
    auto released_sum = [&](int32_t prop, int32_t lane, int32_t cell, int32_t reason) {
      double s = 0.0;
      for (const ext::LiquidPayloadReleased& r : released) {
        if (r.propertyIdx == prop && r.component == lane && (cell < 0 || r.cell == cell) &&
            (reason < 0 || r.reason == reason)) {
          s += r.amount;
        }
      }
      return s;
    };
    auto consumed_sum = [&](int32_t prop, int32_t lane, int32_t kind, int32_t id) {
      double s = 0.0;
      for (const ext::LiquidPayloadConsumed& r : consumed) {
        if (r.propertyIdx == prop && r.component == lane && (kind < 0 || r.kind == kind) &&
            (id == -2 || r.id == id)) {
          s += r.amount;
        }
      }
      return s;
    };

    // What was put in, per (property, lane), and the closure check against it.
    struct Account {
      int32_t prop;
      int32_t lane;
      double injected;
      double grid_prev;
      double unreported_prev;
      double worst;
    };
    std::vector<Account> accounts = {
        {dm, 0, 4 * 1.0 + 7 * 2.0, 0, 0, 0},
        {dm, 3, 2.0, 0, 0, 0},
        {tracer, 0, 4 * 10.0, 0, 0, 0},
    };
    bool closing = false;
    int closed_ticks = 0;
    std::vector<uint8_t> pl_visible;
    const GameDataUpdate* pf = nullptr;
    auto step = [&]() {
      pf = Tick(pw, &pl_visible);
      absorb(pf);
      for (Account& a : accounts) {
        const double books = released_sum(a.prop, a.lane, -1, -1) +
                             consumed_sum(a.prop, a.lane, -1, -2);
        if (closing) {
          const double err = std::fabs(a.grid_prev + books + a.unreported_prev - a.injected) /
                             a.injected;
          if (err > a.worst) a.worst = err;
        }
        a.grid_prev = grid_sum(a.prop, a.lane);
        a.unreported_prev = unreported(a.prop, a.lane);
      }
      if (closing) ++closed_ticks;
    };

    pf = Boot(t, pw);
    SubscribeStream(sim_ext_stream_index(ext::kStreamMessageRefused), true);
    SubscribeStream(sim_ext_stream_index(ext::kStreamLiquidPayloadReleased), true);
    SubscribeStream(sim_ext_stream_index(ext::kStreamLiquidPayloadConsumed), true);
    transport(tracer, ext::kTransportFollowsLiquidMass);
    transport(flag, ext::kTransportFollowsLiquidMass);  // a U8: refused
    for (int x = 1; x <= 4; ++x) {
      add(at(x, kBasinY), dm, 0, 1.0f);
      add(at(x, kBasinY), tracer, 0, 10.0f);
    }
    add(at(1, kBasinY), fixed, 0, 5.0f);
    for (int i = 0; i < kPockets; ++i) {
      add(pocket(i), dm, i == kBoil ? 3 : 0, 2.0f);
    }
    // Seven adds the sim must refuse, and none of which may move a single bit: a sum below
    // zero, a NaN, a property that is not F32, a cell off the grid, a lane past the property's
    // arity, and a liquid-carried amount into a cell with no liquid (solid, then vacuum).
    add(at(1, kBasinY), dm, 1, -1.0f);
    add(at(1, kBasinY), dm, 2, std::numeric_limits<float>::quiet_NaN());
    add(at(1, kBasinY), flag, 0, 1.0f);
    add(kPW * kPH, dm, 0, 1.0f);
    add(at(1, kBasinY), dm, ext::kDissolvedGasLanes, 1.0f);
    add(at(3, kPocketY), dm, 0, 1.0f);  // the granite between two pockets: no liquid to ride
    add(at(8, kBasinY + 2), tracer, 0, 1.0f);  // vacuum above the basin: no liquid either
    for (int i = 0; i < 3; ++i) step();
    closing = true;

    int refused_adds = 0, refused_transport = 0;
    for (const ext::ExtRefusedMessage& r : refused) {
      if (r.reason != ext::kExtRefusalBadTarget) continue;
      if (r.messageId == ext::kAddCellPropertyAmount) ++refused_adds;
      if (r.messageId == ext::kSetCellPropertyTransport) ++refused_transport;
    }
    printf("  refusals: %d adds, %d transport\n", refused_adds, refused_transport);
    Check(refused_adds == 7,
          "kAddCellPropertyAmount refuses, on the refusal stream, a sum below zero, a NaN, a "
          "non-F32 property, an off-grid cell, a lane past the arity, and a liquid-carried "
          "amount into a solid and a vacuum cell -- seven of seven");
    Check(refused_transport == 1,
          "kSetCellPropertyTransport refuses to make a U8 property follow the liquid: an amount "
          "carried by mass has to be divisible");
    Check(grid_sum(dm, 1) == 0.0 && grid_sum(dm, 2) == 0.0 &&
              grid_sum(dm, ext::kDissolvedGasLanes - 1) == 0.0,
          "and the refused adds left their lanes at exactly zero");
    Check(readf(pocket(kControl), dm, 0) == 2.0f && readf(pocket(kBoil), dm, 3) == 2.0f,
          "an accepted add lands exactly, on the lane it named");

    // ---- the pocket exits. Each message goes to its own pocket on the same tick.
    ModifyCellMessage replace{};
    replace.cellIdx = pocket(kReplace);
    replace.callbackIdx = -1;
    replace.temperature = 300.0f;
    replace.mass = 500.0f;
    replace.elementIdx = pl_granite;
    replace.replaceType = 1;  // kReplaceElement (sim/cellmod.h)
    replace.diseaseIdx = 0xFF;
    sim_handle_message(static_cast<int32_t>(SimMessageHash::ModifyCell), sizeof(replace),
                       reinterpret_cast<const uint8_t*>(&replace));
    ModifyCellMessage remove{};
    remove.cellIdx = pocket(kRemove);
    remove.callbackIdx = -1;
    remove.temperature = 300.0f;
    remove.mass = -100.0f;
    remove.elementIdx = pl_water;
    remove.replaceType = 0;
    remove.diseaseIdx = 0xFF;
    sim_handle_message(static_cast<int32_t>(SimMessageHash::ModifyCell), sizeof(remove),
                       reinterpret_cast<const uint8_t*>(&remove));
    for (int i : {static_cast<int>(kConsume), static_cast<int>(kConsumeNoId)}) {
      MassConsumptionMessage mc{};
      mc.cellIdx = pocket(i);
      mc.callbackIdx = i == kConsume ? 77 : -1;
      mc.mass = 100.0f;
      mc.elementIdx = pl_water;
      mc.radius = 1;
      mc.height = 0;
      sim_handle_message(static_cast<int32_t>(SimMessageHash::MassConsumption), sizeof(mc),
                         reinterpret_cast<const uint8_t*>(&mc));
    }
    AddElementConsumerMessage ec{};
    ec.cellIdx = pocket(kConsumer);
    ec.callbackIdx = 4242;
    ec.radius = 1;
    ec.configuration = 0;  // kConsumeSpecificElement (sim/emitters.h)
    ec.elementIdx = pl_water;
    sim_handle_message(static_cast<int32_t>(SimMessageHash::AddElementConsumer), sizeof(ec),
                       reinterpret_cast<const uint8_t*>(&ec));
    ModifyCellEnergyMessage boil{};
    boil.cellIdx = pocket(kBoil);
    boil.kilojoules = 1.0e9f;
    boil.maxTemperature = 420.0f;
    boil.id = -1;
    sim_handle_message(static_cast<int32_t>(SimMessageHash::ModifyCellEnergy), sizeof(boil),
                       reinterpret_cast<const uint8_t*>(&boil));
    for (int i = 0; i < 3; ++i) step();
    Check(consumer_handle != -1, "the ElementConsumer registered and returned a real handle");
    SetElementConsumerDataMessage rate{};
    rate.handle = consumer_handle;
    rate.cell = pocket(kConsumer);
    rate.consumptionRate = 5.0f;
    sim_handle_message(static_cast<int32_t>(SimMessageHash::SetElementConsumerData),
                       sizeof(rate), reinterpret_cast<const uint8_t*>(&rate));
    for (int i = 0; i < 14; ++i) step();

    // ---- the unreported total. Unsubscribe the released stream, let that land, then replace
    // one more pocket: its payload has nowhere to be reported, so it must be counted.
    SubscribeStream(sim_ext_stream_index(ext::kStreamLiquidPayloadReleased), false);
    for (int i = 0; i < 2; ++i) step();
    const double unheard_before = unreported(dm, 0);
    replace.cellIdx = pocket(kUnheard);
    sim_handle_message(static_cast<int32_t>(SimMessageHash::ModifyCell), sizeof(replace),
                       reinterpret_cast<const uint8_t*>(&replace));
    for (int i = 0; i < 3; ++i) step();
    const double unheard = unreported(dm, 0) - unheard_before;
    SubscribeStream(sim_ext_stream_index(ext::kStreamLiquidPayloadReleased), true);
    for (int i = 0; i < 3; ++i) step();

    // The concentration snapshot: payload read now is the state after frame n, and the NEXT
    // tick returns frame n's masses, so the two describe the same instant.
    std::vector<float> dm_now(kPW * kPH), tr_now(kPW * kPH);
    for (int32_t c = 0; c < kPW * kPH; ++c) {
      dm_now[c] = readf(c, dm, 0);
      tr_now[c] = readf(c, tracer, 0);
    }
    const float consumer_left = readf(pocket(kConsumer), dm, 0);
    const float consume_left = readf(pocket(kConsume), dm, 0);
    const float consume_noid_left = readf(pocket(kConsumeNoId), dm, 0);
    const float remove_left = readf(pocket(kRemove), dm, 0);
    step();
    const GameDataUpdate* snap = pf;

    for (const Account& a : accounts) {
      printf("  closure prop %d lane %d: injected %.6f, worst relative error %.3g over %d ticks\n",
             a.prop, a.lane, a.injected, a.worst, closed_ticks);
    }
    bool closes = true;
    for (const Account& a : accounts) closes = closes && a.worst < 1e-5;
    Check(closes && closed_ticks > 20,
          "ON EVERY TICK: grid + released + consumed + unreported equals what was injected, for "
          "the dissolved-gas lanes and for a mod's own tracer (to 1e-5 relative)");

    // The basin: the payload rode the liquid downstream, at the liquid's own concentration.
    bool reached = false;
    for (int x = 5; x <= 10; ++x) reached = reached || dm_now[at(x, kBasinY)] > 0.0f;
    double worst_conc = 0.0, worst_ratio = 0.0;
    int wet = 0;
    for (int y = kBasinY; y < kBasinY + 4; ++y) {
      for (int x = 1; x <= 10; ++x) {
        const int32_t c = at(x, y);
        if (snap == nullptr || snap->elementIdx[c] != pl_water || snap->mass[c] < 1.0f) continue;
        ++wet;
        const double conc = dm_now[c] / snap->mass[c];
        worst_conc = std::max(worst_conc, std::fabs(conc / 1.0e-3 - 1.0));
        if (dm_now[c] > 0.0f) {
          worst_ratio = std::max(worst_ratio, std::fabs(tr_now[c] / dm_now[c] / 10.0 - 1.0));
        }
      }
    }
    printf("  basin: %d wet cells, payload downstream: %s, worst concentration error %.3g, "
           "worst tracer ratio error %.3g\n",
           wet, reached ? "yes" : "no", worst_conc, worst_ratio);
    Check(reached, "StepFlow carried dissolved gas into cells that started dry");
    Check(wet > 4 && worst_conc < 1e-3,
          "and every wet cell holds it at the source water's concentration (1 g per kg): it "
          "moves with the mass, in proportion");
    Check(worst_ratio < 1e-3,
          "a mod's own F32, switched to follow the liquid, rides with it identically -- the "
          "transport is generic, not special to dissolved gas");
    Check(readf(at(1, kBasinY), fixed, 0) == 5.0f && grid_sum(fixed, 0) == 5.0,
          "a property left static stays in its cell while the water it sat in moved away");

    // The pockets, one exit each.
    const double rep = released_sum(dm, 0, pocket(kReplace), ext::kPayloadReleasedReplaced);
    Check(std::fabs(rep - 2.0) < 1e-6 && readf(pocket(kReplace), dm, 0) == 0.0f,
          "REPLACED: a ModifyCell that turns the water to granite releases all 2 kg, reason "
          "Replaced, at that cell");
    const double rem = released_sum(dm, 0, pocket(kRemove), ext::kPayloadReleasedRemoved);
    printf("  removed %.6f, left %.6f\n", rem, remove_left);
    Check(std::fabs(rem - 0.4) < 1e-5 && std::fabs(remove_left - 1.6) < 1e-5,
          "REMOVED: taking 100 kg of 500 releases a fifth of the payload and leaves the rest");
    const double con77 = consumed_sum(dm, 0, ext::kPayloadConsumerMassConsumption, 77);
    const double take77 = snap ? (500.0 - snap->mass[pocket(kConsume)]) / 500.0 : -1.0;
    printf("  MassConsumption cb 77: consumed %.6f (liquid share %.6f), left %.6f\n", con77,
           take77, consume_left);
    Check(take77 > 0.0 && std::fabs(con77 - 2.0 * take77) < 1e-5 &&
              std::fabs(con77 + consume_left - 2.0) < 1e-5,
          "CONSUMED: a MassConsumption with a callback hands its share to the consumed stream, "
          "keyed MassConsumption + its callbackIdx");
    const double noid = released_sum(dm, 0, pocket(kConsumeNoId), ext::kPayloadReleasedRemoved);
    Check(noid > 0.0 && std::fabs(noid + consume_noid_left - 2.0) < 1e-5 &&
              consumed_sum(dm, 0, -1, -1) == 0.0,
          "and one with no callback has nobody to hand it to, so it is RELEASED at the cell "
          "(reason Removed) rather than consumed by id -1");
    const double ecs = consumed_sum(dm, 0, ext::kPayloadConsumerElementConsumer,
                                    consumer_handle);
    const double ec_mass = snap ? snap->mass[pocket(kConsumer)] : -1.0;
    printf("  ElementConsumer %d: consumed %.6f, left %.6f in %.3f kg\n", consumer_handle, ecs,
           consumer_left, ec_mass);
    Check(ecs > 0.0 && std::fabs(ecs + consumer_left - 2.0) < 1e-5 &&
              std::fabs(consumer_left / ec_mass / (2.0 / 500.0) - 1.0) < 1e-3,
          "CONSUMED: an ElementConsumer drawing water takes the payload with it, keyed by its "
          "sim handle, and leaves the rest at the water's concentration");
    Check(readf(pocket(kControl), dm, 0) == 2.0f,
          "CONTROL: the pocket nothing touched still holds exactly 2 kg");
    double boil_t = 0.0;
    double boiled = 0.0;
    for (const ext::LiquidPayloadReleased& r : released) {
      if (r.cell == pocket(kBoil) && r.propertyIdx == dm && r.component == 3 &&
          r.reason == ext::kPayloadReleasedPhaseChange) {
        boiled += r.amount;
        boil_t = r.temperatureK;
      }
    }
    printf("  boiled %.6f at %.2f K\n", boiled, boil_t);
    Check(std::fabs(boiled - 2.0) < 1e-6 && boil_t > 370.0 &&
              readf(pocket(kBoil), dm, 3) == 0.0f,
          "PHASE CHANGE: water boiled to steam releases its whole payload, on the lane it was "
          "on, at the temperature it boiled at");
    printf("  unreported while unsubscribed: %.6f\n", unheard);
    Check(std::fabs(unheard - 2.0) < 1e-6 &&
              released_sum(dm, 0, pocket(kUnheard), -1) == 0.0,
          "UNREPORTED: a release with nobody subscribed is not lost -- it is counted, exactly, "
          "on SIM_ExtLiquidPayloadUnreported");
  }

  // Payload mixing (sim/payload_mix.h). Transport, tested above, moves an amount with its
  // water. This is the other half: what spreads it through water that is NOT moving. Without
  // it, a pond carbonated by a column of bubbles holds the carbonation in those columns and
  // reads exactly 0.000 g/kg one tile away, for any pressure and any length of run.
  //
  // One sealed world, run twice -- once at the default share and once with mixing switched off
  // by message -- because "it spread" only means something beside a run where it did not.
  printf("\n=== vftest: Layer C -- a dissolved amount mixes through still water ===\n");
  {
    constexpr int kMW = 20, kMH = 8;
    constexpr int kPondY = 2, kPondX0 = 1, kPondX1 = 10;   // the long pond, one row
    constexpr int kStackX = 14, kStackY = 2;               // the unequal-mass pair, stacked
    constexpr int kTicks = 600;
    auto mat = [](int x, int y) { return static_cast<int32_t>(y * kMW + x); };

    // What each run leaves behind, so the two can be compared as data rather than in prose.
    struct MixRun {
      double grid_sum;        // total dissolved mass on the grid at the end
      double first;           // the cell it was injected into
      double last;            // the far end of the pond
      double lo, hi, mean;    // concentration spread across the pond, kg per kg
      double stack_lo, stack_hi;  // the unequal-mass pair, kg per kg, which must not move
      int32_t refused_bad;    // refusals of a kSetPayloadMixing naming no property
    };

    auto run = [&](bool mixing) -> MixRun {
      sim_shutdown();
      InitSim();
      uint16_t mx_granite = 0, mx_water = 0;
      MixRun out{};
      if (!Resolve(t, simhost::kGranite, "Granite", &mx_granite)) return out;
      if (!Resolve(t, simhost::kWater, "Water", &mx_water)) return out;
      const int32_t dmx = sim_ext_property_index(ext::kDissolvedMassProperty);

      World mw;
      // Solid everywhere else, so every pond below is sealed: no cell can flow, and anything
      // that arrives at the far end arrived by mixing and by nothing else.
      mw.Init(kMW, kMH, mx_granite, 2000.0f, 300.0f);
      for (int x = kPondX0; x <= kPondX1; ++x) {
        mw.element[mat(x, kPondY)] = mx_water;
        mw.mass[mat(x, kPondY)] = 1000.0f;   // full, so the liquid mover has nowhere to pour
        mw.temperature[mat(x, kPondY)] = 300.0f;
      }
      // THE UNEQUAL-MASS PAIR, and it is the check that this kernel mixes CONCENTRATION rather
      // than amount. A full cell under a fifth-full one is already where water settles, so the
      // liquid mover leaves it alone; both cells are seeded at the same 5 grams per kilogram
      // but hold five times different AMOUNTS. A kernel equalising amounts would push a
      // kilogram and a half upwards into the small cell and call the result mixed.
      mw.element[mat(kStackX, kStackY)] = mx_water;
      mw.mass[mat(kStackX, kStackY)] = 1000.0f;
      mw.temperature[mat(kStackX, kStackY)] = 300.0f;
      mw.element[mat(kStackX, kStackY + 1)] = mx_water;
      mw.mass[mat(kStackX, kStackY + 1)] = 200.0f;
      mw.temperature[mat(kStackX, kStackY + 1)] = 300.0f;

      std::vector<uint8_t> mvis;
      Boot(t, mw);
      SubscribeStream(sim_ext_stream_index(ext::kStreamMessageRefused), true);

      auto mixing_msg = [&](int32_t prop, float share) {
        ext::SetPayloadMixingMessage m{prop, share};
        sim_handle_message(ext::kSetPayloadMixing, static_cast<int>(sizeof(m)),
                           reinterpret_cast<const uint8_t*>(&m));
      };
      if (!mixing) mixing_msg(dmx, 0.0f);
      // A property index nobody registered, sent in both runs: a rate for something that does
      // not exist is a caller naming a thing, not a caller asking for too much of one, so it
      // is refused rather than clamped.
      mixing_msg(9999, 0.5f);

      auto add = [&](int32_t cell, float amount) {
        ext::AddCellPropertyAmountMessage m{cell, dmx, 0, amount};
        sim_handle_message(ext::kAddCellPropertyAmount, static_cast<int>(sizeof(m)),
                           reinterpret_cast<const uint8_t*>(&m));
      };
      add(mat(kPondX0, kPondY), 10.0f);
      add(mat(kStackX, kStackY), 5.0f);          // 1000 kg at 5 g/kg
      add(mat(kStackX, kStackY + 1), 1.0f);      // 200 kg at the same 5 g/kg

      for (int i = 0; i < kTicks; ++i) {
        const GameDataUpdate* fr = Tick(mw, &mvis);
        for (const ext::ExtRefusedMessage& r : Refusals(fr)) {
          if (r.messageId == ext::kSetPayloadMixing) ++out.refused_bad;
        }
      }

      auto readm = [&](int32_t cell) {
        uint32_t bits = 0;
        if (!sim_ext_read_cell_property(cell, dmx, 0, &bits)) {
          return std::numeric_limits<float>::quiet_NaN();
        }
        float v = 0.0f;
        memcpy(&v, &bits, sizeof(v));
        return v;
      };
      for (int32_t c = 0; c < kMW * kMH; ++c) out.grid_sum += readm(c);
      out.first = readm(mat(kPondX0, kPondY));
      out.last = readm(mat(kPondX1, kPondY));
      out.lo = 1e30;
      out.hi = -1e30;
      for (int x = kPondX0; x <= kPondX1; ++x) {
        const double conc = readm(mat(x, kPondY)) / 1000.0;
        out.lo = std::min(out.lo, conc);
        out.hi = std::max(out.hi, conc);
        out.mean += conc;
      }
      out.mean /= static_cast<double>(kPondX1 - kPondX0 + 1);
      out.stack_lo = readm(mat(kStackX, kStackY)) / 1000.0;
      out.stack_hi = readm(mat(kStackX, kStackY + 1)) / 200.0;
      return out;
    };

    const MixRun on = run(true);
    const MixRun off = run(false);

    printf("  mixing ON  after %d ticks: grid %.6f kg, near end %.6f, far end %.6f, "
           "concentration %.6g..%.6g (mean %.6g)\n",
           kTicks, on.grid_sum, on.first, on.last, on.lo, on.hi, on.mean);
    printf("  mixing OFF after %d ticks: grid %.6f kg, near end %.6f, far end %.6f\n",
           kTicks, off.grid_sum, off.first, off.last);

    // Conservation first, because a kernel that spreads an amount by inventing it spreads it
    // beautifully. 16 kg went in: 10 into the pond, 5 and 1 into the stacked pair.
    Check(std::fabs(on.grid_sum - 16.0) < 1.6e-4 && std::fabs(off.grid_sum - 16.0) < 1.6e-4,
          "MIXING CONSERVES: the grid holds exactly what was injected, mixing on or off (to "
          "1e-5 relative)");

    Check(off.last == 0.0f && std::fabs(off.first - 10.0) < 1e-5,
          "WITH MIXING OFF the far end of a sealed pond is still exactly zero after 600 ticks "
          "-- which is what row 37 measured in the game, reproduced here as a control");

    Check(on.last > 0.0 && on.lo > 0.0,
          "WITH MIXING ON every cell of the pond holds some, including the far end ten tiles "
          "from where it was injected into still water");

    const double spread = on.mean > 0.0 ? (on.hi - on.lo) / on.mean : 1.0;
    printf("  pond spread (hi-lo)/mean: %.4f\n", spread);
    Check(spread < 0.15,
          "and the pond is close to UNIFORM: at the default 0.125 share the length of it "
          "evens out inside 600 substeps, which is the behaviour a sensor anywhere in a "
          "carbonated pond needs");

    printf("  unequal-mass pair, mixing on: %.6g and %.6g kg per kg\n", on.stack_lo,
           on.stack_hi);
    Check(std::fabs(on.stack_lo - 5.0e-3) < 1e-6 && std::fabs(on.stack_hi - 5.0e-3) < 1e-6,
          "CONCENTRATION, NOT AMOUNT: a 1000 kg cell and a 200 kg cell already at the same "
          "grams-per-kilogram exchange nothing, though one holds five times the amount");

    Check(on.refused_bad > 0 && off.refused_bad > 0,
          "kSetPayloadMixing naming a property that does not exist is refused on the stream, "
          "not clamped into one that does");
  }

  // Effervescence (ext::kSetEffervescence; sim/effervescence.h). Without it, supersaturated
  // water simply stays supersaturated: nothing but a passing bubble can take gas out of it. This arm is the physics of a bottle:
  //
  //   A  a deep column loaded uniformly -- its SURFACE is past the threshold and its BOTTOM is
  //      not, because the water above the bottom cell presses it into holding more;
  //   B  the same column loaded to just under the threshold -- the control, it must never fizz;
  //   C  a cold shallow column and
  //   D  a warm one, loaded with the SAME kilograms: warm water holds less (van 't Hoff), so D
  //      fizzes and C does not;
  //   E  water sealed under granite with no gas above it, loaded far past what any atmosphere
  //      would hold -- a full rigid vessel has no room for a bubble, so it stays flat.
  //
  // Run three times: with the message, without it, and with it but naming no solvent. Nothing
  // puts the released gas back -- that is the managed side's job -- so every released gram is on
  // the stream and conservation is "grid + released == injected" exactly.
  printf("\n=== vftest: Layer C3 -- supersaturated water fizzes, and only where it should ===\n");
  {
    constexpr int kFW = 17, kFH = 12;
    constexpr int kColA = 2, kColB = 5, kColC = 8, kColD = 11, kColE = 14;
    constexpr int kDeepY0 = 2, kDeepY1 = 7;       // A and B: six rows, CO2 cap at y 8
    constexpr int kShallowY0 = 2, kShallowY1 = 3;  // C and D: two rows, CO2 cap at y 4
    constexpr float kTempAB = 300.0f, kTempC = 285.0f, kTempD = 320.0f;
    constexpr float kCapPa = 101325.0f;
    constexpr int kTicks = 1500;                  // 300 s of sim time
    constexpr float kMargin = 0.10f, kRate = 0.10f, kPeriod = 1.0f, kMinRelease = 1.0e-4f;
    // Carbon dioxide on lane 0, as `DissolvedGas`'s lane table has it; Sander's constants, the
    // same numbers `Solubility` carries.
    constexpr float kH = 3.3e-4f, kVantHoff = 2400.0f, kMolar = 0.04401f;
    auto fcell = [](int x, int y) { return static_cast<int32_t>(y * kFW + x); };

    // What the sim should compute, written out again here so the arm has an opinion of its own.
    // Partial molar volume is sent as 0, so V is the water's volume alone.
    auto henry = [&](float t) {
      return kH * std::exp(kVantHoff * (1.0f / t - 1.0f / 298.15f));
    };
    auto cap_kg = [&](float t) { return kCapPa * kMolar / (8.3144f * t); };
    auto centre_pa = [&](float cap_pa, int rows_above) {
      return cap_pa + (static_cast<float>(rows_above) * 1000.0f + 500.0f) * 9.80665f;
    };
    auto capacity = [&](float t, float pa) { return henry(t) * pa * kMolar * 1.0f; };

    // Loads. A: 1.53x the TOP cell's capacity, which is 1.05x the BOTTOM cell's. B: 0.95x the
    // top's STATIC capacity. C and D: 0.9x of the COLD column's top capacity.
    //
    // WHY B IS 0.95 AND NOT 1.05, which is what its first version used and failed on. The static
    // figures assume six full 1000 kg cells, and ONI's water does not stay that way: under its
    // own column it compresses downwards (measured: 1040.52 kg at the bottom, 899.24 at the top
    // after 1500 ticks), and the tenth of a cubic metre the top cell gives up becomes VOLUME for
    // the CO2 cap above it (`Bubbles.GasVolumeM3`). The cap expands to 1.1 m3 and its pressure
    // falls about 9%, so 1.05x on paper was ~1.15x in the running sim -- past the margin, and
    // the sim was right to fizz it. 0.95x on paper is ~1.05x once the cap has expanded.
    const float top_ab = capacity(kTempAB, centre_pa(kCapPa, 0));
    const float bottom_ab = capacity(kTempAB, centre_pa(kCapPa, kDeepY1 - kDeepY0));
    const float load_a = 1.53f * top_ab;
    const float load_b = 0.95f * top_ab;
    const float top_c = capacity(kTempC, centre_pa(kCapPa, 0));
    const float top_d = capacity(kTempD, centre_pa(kCapPa, 0));
    const float load_cd = 0.9f * top_c;
    printf("  capacities: A/B top %.4f kg, bottom %.4f kg; C top %.4f, D top %.4f\n", top_ab,
           bottom_ab, top_c, top_d);
    printf("  loads: A %.4f (S top %.3f, bottom %.3f), B %.4f (S top %.3f), C and D %.4f "
           "(S %.3f cold, %.3f warm)\n",
           load_a, load_a / top_ab, load_a / bottom_ab, load_b, load_b / top_ab, load_cd,
           load_cd / top_c, load_cd / top_d);

    enum FizzMode { kFizzOff, kFizzOn, kFizzNoSolvent };
    struct FizzRun {
      double injected = 0.0, grid = 0.0, released = 0.0;
      int32_t records = 0, bad_cell = 0, bad_temp = 0, late = 0;
      double by_col[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
      double top_a = 0.0, bottom_a = 0.0;
      double max_s_a = 0.0;
    };

    auto run = [&](FizzMode mode) -> FizzRun {
      sim_shutdown();
      InitSim();
      FizzRun out{};
      uint16_t f_granite = 0, f_water = 0, f_co2 = 0;
      if (!Resolve(t, simhost::kGranite, "Granite", &f_granite)) return out;
      if (!Resolve(t, simhost::kWater, "Water", &f_water)) return out;
      if (!Resolve(t, simhost::kCarbonDioxide, "CarbonDioxide", &f_co2)) return out;
      const int32_t dmf = sim_ext_property_index(ext::kDissolvedMassProperty);

      World fw;
      fw.Init(kFW, kFH, f_granite, 2000.0f, 300.0f);
      auto column = [&](int x, int y0, int y1, float temp) {
        for (int y = y0; y <= y1; ++y) {
          fw.element[fcell(x, y)] = f_water;
          fw.mass[fcell(x, y)] = 1000.0f;
          fw.temperature[fcell(x, y)] = temp;
        }
        fw.element[fcell(x, y1 + 1)] = f_co2;
        fw.mass[fcell(x, y1 + 1)] = cap_kg(temp);
        fw.temperature[fcell(x, y1 + 1)] = temp;
      };
      column(kColA, kDeepY0, kDeepY1, kTempAB);
      column(kColB, kDeepY0, kDeepY1, kTempAB);
      column(kColC, kShallowY0, kShallowY1, kTempC);
      column(kColD, kShallowY0, kShallowY1, kTempD);
      // E: two rows of water and NO cap -- the granite `Init` filled the world with stays above.
      for (int y = kShallowY0; y <= kShallowY1; ++y) {
        fw.element[fcell(kColE, y)] = f_water;
        fw.mass[fcell(kColE, y)] = 1000.0f;
        fw.temperature[fcell(kColE, y)] = kTempAB;
      }

      std::vector<uint8_t> fvis;
      Boot(t, fw);
      SubscribeStream(sim_ext_stream_index(ext::kStreamLiquidPayloadReleased), true);

      if (mode != kFizzOff) {
        ext::SetEffervescenceMessage m{};
        m.enabled = 1;
        m.margin = kMargin;
        m.ratePerSecond = kRate;
        m.periodSeconds = kPeriod;
        m.minReleaseKg = kMinRelease;
        m.laneHenryMolPerM3Pa[0] = kH;
        m.laneVantHoffK[0] = kVantHoff;
        m.laneMolarMassKgPerMol[0] = kMolar;
        if (mode == kFizzOn) {
          m.solventCount = 1;
          m.solventElementIdx[0] = f_water;
          m.solventFactor[0] = 1.0f;
          m.solventDensityKgPerM3[0] = 1000.0f;
        }
        sim_handle_message(ext::kSetEffervescence, static_cast<int>(sizeof(m)),
                           reinterpret_cast<const uint8_t*>(&m));
      }
      auto add = [&](int32_t cell, float amount) {
        ext::AddCellPropertyAmountMessage m{cell, dmf, 0, amount};
        sim_handle_message(ext::kAddCellPropertyAmount, static_cast<int>(sizeof(m)),
                           reinterpret_cast<const uint8_t*>(&m));
        out.injected += amount;
      };
      for (int y = kDeepY0; y <= kDeepY1; ++y) {
        add(fcell(kColA, y), load_a);
        add(fcell(kColB, y), load_b);
      }
      for (int y = kShallowY0; y <= kShallowY1; ++y) {
        add(fcell(kColC, y), load_cd);
        add(fcell(kColD, y), load_cd);
        add(fcell(kColE, y), 5.0f * top_ab);
      }

      auto col_of = [&](int32_t cell) {
        const int x = cell % kFW;
        return x == kColA ? 0 : x == kColB ? 1 : x == kColC ? 2 : x == kColD ? 3
             : x == kColE ? 4 : -1;
      };
      auto temp_of_col = [&](int col) {
        return col == 2 ? kTempC : col == 3 ? kTempD : kTempAB;
      };
      for (int i = 0; i < kTicks; ++i) {
        const GameDataUpdate* fr = Tick(fw, &fvis);
        if (fr != nullptr && i == kTicks - 1 && mode == kFizzOn) {
          printf("  column B after %d ticks, kg bottom to top, cap last:", kTicks);
          for (int y = kDeepY0; y <= kDeepY1 + 1; ++y) printf(" %.2f", fr->mass[fcell(kColB, y)]);
          printf("\n");
        }
        const ext::ExtPublishedStream* st =
            fr != nullptr ? FindStream(fr, ext::kStreamLiquidPayloadReleased) : nullptr;
        const auto* r = st != nullptr ? static_cast<const ext::LiquidPayloadReleased*>(st->data)
                                      : nullptr;
        for (int32_t k = 0; r != nullptr && k < st->count; ++k) {
          if (r[k].propertyIdx != dmf || r[k].reason != ext::kPayloadReleasedEffervescence) {
            continue;
          }
          ++out.records;
          out.released += r[k].amount;
          const int col = col_of(r[k].cell);
          if (col < 0 || fw.element[static_cast<size_t>(r[k].cell)] != f_water) {
            ++out.bad_cell;
            continue;
          }
          out.by_col[col] += r[k].amount;
          if (std::fabs(r[k].temperatureK - temp_of_col(col)) > 2.0f) ++out.bad_temp;
          if (i >= kTicks - 50) ++out.late;
          if (r[k].cell == fcell(kColA, kDeepY1)) out.top_a += r[k].amount;
          if (r[k].cell == fcell(kColA, kDeepY0)) out.bottom_a += r[k].amount;
        }
      }

      auto readf = [&](int32_t cell) {
        uint32_t bits = 0;
        if (!sim_ext_read_cell_property(cell, dmf, 0, &bits)) return 0.0f;
        float v = 0.0f;
        memcpy(&v, &bits, sizeof(v));
        return v;
      };
      for (int32_t c = 0; c < kFW * kFH; ++c) out.grid += readf(c);
      // Column A's worst remaining supersaturation, against the static pressures.
      for (int y = kDeepY0; y <= kDeepY1; ++y) {
        const double s = readf(fcell(kColA, y)) /
                         capacity(kTempAB, centre_pa(kCapPa, kDeepY1 - y));
        out.max_s_a = std::max(out.max_s_a, s);
      }
      return out;
    };

    const FizzRun on = run(kFizzOn);
    const FizzRun off = run(kFizzOff);
    const FizzRun nosol = run(kFizzNoSolvent);

    printf("  ON: injected %.4f kg, grid %.4f, released %.4f in %d records "
           "(A %.4f, B %.4f, C %.4f, D %.4f, E %.4f); A top %.4f, A bottom %.4f; A's worst S "
           "now %.4f; %d in the last 50 ticks\n",
           on.injected, on.grid, on.released, on.records, on.by_col[0], on.by_col[1],
           on.by_col[2], on.by_col[3], on.by_col[4], on.top_a, on.bottom_a, on.max_s_a,
           on.late);
    printf("  OFF: released %.4f in %d records; NO SOLVENT: released %.4f in %d records\n",
           off.released, off.records, nosol.released, nosol.records);

    Check(std::fabs(on.grid + on.released - on.injected) < 1e-4 * on.injected,
          "EFFERVESCENCE CONSERVES: what is left dissolved plus what was released as "
          "effervescence is exactly what was injected (1e-4 relative)");
    Check(on.records > 0 && on.by_col[0] > 0.0,
          "a column loaded past 1 + margin at its surface FIZZES: effervescence records "
          "appear, on the dissolved property, with reason kPayloadReleasedEffervescence");
    Check(on.bad_cell == 0,
          "every effervescence record names a cell that is STILL WATER -- the gas left, the "
          "liquid did not, which is what tells the managed side to make bubbles");
    Check(on.bad_temp == 0,
          "and carries the temperature of the water it left");
    Check(on.top_a > 0.0 && on.bottom_a == 0.0,
          "DEPTH HOLDS GAS: the same load fizzes at the top of a six-metre column and not at its "
          "bottom, which ~54 kPa of water above it keeps in solution");
    Check(on.by_col[1] == 0.0,
          "a column loaded to just inside the margin at its surface, once its own water has "
          "settled, never fizzes");
    Check(on.by_col[3] > 0.0 && on.by_col[2] == 0.0,
          "WARM WATER HOLDS LESS: the same kilograms fizz out of water at 320 K and stay in "
          "water at 285 K (van 't Hoff)");
    Check(on.by_col[4] == 0.0,
          "CONFINED LIQUID STAYS FLAT: water sealed under a solid with no gas above it holds "
          "five times an atmosphere's worth and releases none -- a full rigid vessel has no room "
          "for a bubble");
    Check(on.late == 0 && on.max_s_a <= 1.0 + kMargin + 0.02,
          "IT SETTLES: after 300 s nothing is still fizzing, and no cell of the deep column is "
          "left past the threshold");
    Check(off.records == 0 && std::fabs(off.grid - off.injected) < 1e-4 * off.injected,
          "WITHOUT kSetEffervescence nothing fizzes at all, and every gram is still dissolved "
          "-- the pass is off until a caller turns it on");
    Check(nosol.records == 0,
          "a liquid that is not named as a solvent never fizzes, whatever it holds");
  }

  // THE TINT MESSAGE IS DISPATCHED AT ALL. `kSetDissolvedTint` ("ONIS") once had
  // a handler and no `ONI_EXT_MESSAGE_LIST` row, and the dispatcher is generated
  // from that list -- so the question of whether the sim ever saw the message is a question
  // about the list, not about the handler, and no arm had asked it. A message the dispatcher
  // does not know is refused on `sim.message_refused` as `kExtRefusalUnknownMessage`, which is
  // what this watches for.
  printf("\n=== vftest: the dissolved-tint message reaches its handler ===\n");
  {
    sim_shutdown();
    InitSim();
    uint16_t tg_granite = 0;
    if (!Resolve(t, simhost::kGranite, "Granite", &tg_granite)) return 1;
    World tw;
    tw.Init(8, 8, tg_granite, 2000.0f, 300.0f);
    std::vector<uint8_t> tvis;
    Boot(t, tw);
    SubscribeStream(sim_ext_stream_index(ext::kStreamMessageRefused), true);
    // The subscription is itself a QUEUED message, while an id the dispatcher does not know is
    // refused synchronously inside `SIM_HandleMessage`. Without a tick between them the refusal
    // is emitted into a stream nobody is subscribed to yet, and this arm reads zero whatever
    // the dispatcher does -- which is exactly what its first version did.
    Tick(tw, &tvis);
    ext::SetDissolvedTintMessage m{};
    m.enabled = 1;
    m.fullScaleGramsPerKg = 10.0f;
    m.maxBlend = 0.45f;
    sim_handle_message(ext::kSetDissolvedTint, static_cast<int>(sizeof(m)),
                       reinterpret_cast<const uint8_t*>(&m));
    // THE POSITIVE CONTROL. An id in our range that nothing defines, sent the same way. Without
    // it a zero below could as easily mean "the refusal stream never fires" as "the tint was
    // dispatched".
    constexpr int32_t kNobodysId = 0x4F4E497F;  // "ONI\x7F", never allocated
    sim_handle_message(kNobodysId, static_cast<int>(sizeof(m)),
                       reinterpret_cast<const uint8_t*>(&m));
    int32_t unknown = 0, control = 0;
    for (int i = 0; i < 3; ++i) {
      const GameDataUpdate* fr = Tick(tw, &tvis);
      for (const ext::ExtRefusedMessage& r : Refusals(fr)) {
        if (r.reason != static_cast<int32_t>(ext::kExtRefusalUnknownMessage)) continue;
        if (r.messageId == ext::kSetDissolvedTint) ++unknown;
        if (r.messageId == kNobodysId) ++control;
      }
    }
    printf("  refused as an unknown message: kSetDissolvedTint %d time(s), the control id %d\n",
           unknown, control);
    Check(control > 0,
          "CONTROL: an id nothing defines IS refused as an unknown message, so the stream this "
          "arm watches does fire");
    Check(unknown == 0,
          "kSetDissolvedTint is DISPATCHED: the sim does not refuse it as an unknown message id");
  }

  bool deviation_goldens_ok = true;
  printf("\n=== vftest: golden state for the deviations diffsim cannot see ===\n");
  {
    // Resolved here rather than reused: the suite's earlier `co2` is a local of another block.
    uint16_t dev_co2 = 0;
    if (!Resolve(t, simhost::kCarbonDioxide, "CarbonDioxide", &dev_co2)) return 1;
    // The deviations, by the ext message each one is driven with. `kDevNone` is the control:
    // the same world, the same building, the same ticks, and no extension message at all.
    enum Deviation {
      kDevNone = 0,
      kDevWasteHeat,     // ext::kSetBuildingWasteHeatKilowatts -- the power -> heat rule
      kDevExhaust,       // ext::kSetBuildingExhaust -- exhaust heat bounced, not deleted
      kDevMixture,       // ext::kInjectGasSpecies -- the multi-gas layer and its enthalpy rule
      kDevRadiation,     // ext::kSetBuildingRadiation -- a radiative outlet for a building body
      kDevEnvTemp,       // the same radiator against a WARMER sink; see below
      kDevThermalMass,   // ext::kSetCellThermalMassBonus -- Mod 2's per-cell material property
    };

    // WHY `envtemp` IS `radiation` WITH A DIFFERENT SINK, AND NOT A CASE OF ITS OWN. Sending
    // `kSetEnvironmentTemperature` on its own changes nothing that any digest can see, and
    // that is correct rather than a defect: `World::EnvironmentTemperature` has exactly one
    // reader in the sim (`sim/buildings.h`, the `t_sink` of a radiating body), so an
    // environment temperature with nothing radiating into it is a number the frame never
    // consults. The first version of this fixture sent it alone and check (b) caught it --
    // which is precisely what (b) is for, and worth more than the golden it was guarding.

    auto state_of = [&](int deviation) -> std::string {
      sim_shutdown();
      InitSim();
      World dw;
      // Small enough to run six times without costing the suite anything, big enough that the
      // gas kernels have somewhere to move mass to. Uniform, so the ONLY thing that differs
      // between the six runs is the one message sent below.
      // THE CELL MASS IS DERIVED, NOT PICKED, and it is the one number in this fixture that
      // had to be. Radiation is suppressed by the local medium on purpose (`sim/buildings.h`):
      // a cell holding real gas is OPAQUE and the flux is exactly 0, which is what stops the
      // radiative outlet from becoming a second, invisible building-to-cell heat path that
      // deletes energy -- measured at ~1.7 MW of continuous cooling on a real 234-cycle colony
      // before it was fixed. So a building buried in ordinary breathable air radiates NOTHING,
      // correctly, and the first two versions of this fixture (1 kg everywhere, then a vacuum
      // stripe that the surrounding gas refilled within a tick) produced a radiation case
      // bit-identical to the control. Check (b) caught both.
      //
      // Exposure ramps in PRESSURE against Stationeers' 6.3 kPa Armstrong limit, so half
      // exposure is 3.15 kPa, which for oxygen (31.9988 g/mol) at 300 K is
      //     n = 3150 / (8.3144 * 300) = 1.2626 mol  ->  0.04040 kg,
      // the same derivation the behavioural radiation arm above uses. Half rather than full,
      // so that a change in EITHER direction of the threshold moves this digest.
      dw.Init(8, 6, oxygen, 0.04040f, 300.0f);
      // One hot cell, identical in all seven runs. The thermal-mass bonus adds J/K to a cell's
      // heat capacity FOR CONDUCTION, so on a world where every cell is already at the same
      // temperature it changes the rate of a transfer that is not happening -- check (b) caught
      // that too. Its neighbour at (5,3) is where the bonus goes.
      dw.temperature[3 * 8 + 4] = 400.0f;
      Boot(t, dw);
      std::vector<uint8_t> dvis;

      // Klei's own per-frame scales. Sent identically in every run: without them both
      // building scales sit at the constructor default and every building transfer runs at a
      // different rate than the game's -- the reason `diffsim`'s SendScales exists.
      DebugProperties scales{};
      scales.buildingTemperatureScale = 0.001f;
      scales.buildingToBuildingTemperatureScale = 0.001f;
      sim_handle_message(static_cast<int32_t>(SimMessageHash::SetDebugProperties),
                         sizeof(scales), reinterpret_cast<const uint8_t*>(&scales));

      // A building in every run, including the control, because two of the deviations are
      // building fields and a fixture that only registers one when it is about to be used
      // would be comparing worlds that differ by more than the message under test.
      AddBuildingHeatExchangeMessage add{};
      add.callbackIdx = -1;
      add.elemIdx = oxygen;
      // A LIGHT, HOT BODY, and both numbers are chosen for sensitivity rather than realism.
      // The digest's resolution is one ulp of a published float32, so the question a fixture
      // has to answer is whether a small change in the RULE moves the state by more than that.
      // At 400 kg and 320 K it does not: a 0.1 % change in the radiative flux moved the body by
      // ~1e-5 K over 30 ticks, under the ~3e-5 K spacing of floats near 320, and the radiation
      // goldens PASSED against a planted fault. 40 kg multiplies the temperature swing by ten
      // and 400 K multiplies the radiated flux by 2.4 (it goes as T^4), which puts the same
      // fault three orders of magnitude clear of the floor. Found by planting it, not by
      // arithmetic first.
      add.mass = 40.0f;
      add.temperature = 400.0f;
      // A REAL conductivity here, unlike the behavioural arms above, which zero it to isolate
      // one field. This golden wants the opposite: the deviation has to reach the GRID, or a
      // digest over published cells cannot see it.
      add.thermalConductivity = 1.0f;
      add.overheatTemperature = 1.0e9f;
      add.operatingKilowatts = 0.0f;
      add.minX = 1;
      add.minY = 1;
      add.maxX = 3;
      add.maxY = 2;
      sim_handle_message(static_cast<int32_t>(SimMessageHash::AddBuildingHeatExchange),
                         sizeof(add), reinterpret_cast<const uint8_t*>(&add));

      // Two ticks: a queued message lands the tick AFTER it is sent, so the building does not
      // exist until the second frame and its handle cannot be read before then.
      Tick(dw, &dvis);
      const GameDataUpdate* u = Tick(dw, &dvis);
      int32_t handle = -1;
      if (u && u->numBuildingTemperatures >= 1) {
        handle = u->buildingTemperatures[u->numBuildingTemperatures - 1].handle;
      }

      switch (deviation) {
        case kDevNone:
          break;
        case kDevWasteHeat: {
          ext::SetBuildingWasteHeatKilowattsMessage m{};
          m.handle = handle;
          m.kilowatts = 10.0f;
          sim_handle_message(ext::kSetBuildingWasteHeatKilowatts, static_cast<int>(sizeof(m)),
                             reinterpret_cast<const uint8_t*>(&m));
          break;
        }
        case kDevExhaust: {
          ext::SetBuildingExhaustMessage m{};
          m.handle = handle;
          m.kilowatts = 10.0f;
          m.maxTemperature = 1000.0f;
          sim_handle_message(ext::kSetBuildingExhaust, static_cast<int>(sizeof(m)),
                             reinterpret_cast<const uint8_t*>(&m));
          break;
        }
        case kDevMixture:
          // Two species into one cell, which is the whole point of the layer: vanilla ONI
          // cannot hold two gases in a cell at all, and the enthalpy rule (sim/gas_mixture.h)
          // is what decides the blend's temperature when the second one arrives.
          InjectGas(4 * 8 + 6, oxygen, 2.0f, 350.0f);
          InjectGas(4 * 8 + 6, dev_co2, 1.0f, 280.0f);
          break;
        case kDevRadiation:
        case kDevEnvTemp: {
          // The sink first, so the radiator below is already reading the right one on its
          // first frame. `kDevRadiation` leaves it at the default 0 K -- deep space, the sink
          // a radiator sees when nobody has said otherwise -- and `kDevEnvTemp` puts a
          // planetary atmosphere at 200 K in its place, which is the whole gameplay point of
          // the field: the same machine sheds heat at a different rate on a different planet.
          if (deviation == kDevEnvTemp) {
            ext::SetEnvironmentTemperatureMessage env{};
            env.kelvin = 200.0f;
            sim_handle_message(ext::kSetEnvironmentTemperature, static_cast<int>(sizeof(env)),
                               reinterpret_cast<const uint8_t*>(&env));
          }
          ext::SetBuildingRadiationMessage rad{};
          rad.handle = handle;
          rad.radiationFactor = 1.0f;  // a black body, so the difference is as large as it gets
          rad.surfaceAreaM2 = 0.0f;    // the building's own footprint
          sim_handle_message(ext::kSetBuildingRadiation, static_cast<int>(sizeof(rad)),
                             reinterpret_cast<const uint8_t*>(&rad));
          break;
        }
        case kDevThermalMass: {
          // The published C type, since `abi/sim_abi_ext.h` declares no C++ alias for this
          // one -- it is the oldest extension id and predates that convention.
          OniSetCellThermalMassBonusMessage m{};
          // The neighbour of the fixture's hot cell, so there is a gradient for the bonus to
          // slow down. Out in the uniform field it changed nothing at all, and check (b)
          // caught that.
          m.cellIdx = 3 * 8 + 5;
          m.value = 50.0f;
          sim_handle_message(ext::kSetCellThermalMassBonus, static_cast<int>(sizeof(m)),
                             reinterpret_cast<const uint8_t*>(&m));
          break;
        }
        default:
          break;
      }

      // Long enough for the deviation to propagate off the cell or building it lands on, short
      // enough that six of these do not show up in the suite's runtime.
      const GameDataUpdate* last = nullptr;
      for (int k = 0; k < 30; ++k) last = Tick(dw, &dvis);

      goldens::Md5 md5;
      if (last) {
        for (size_t i = 0; i < dw.Count(); ++i) {
          uint8_t buf[10];
          memcpy(buf, &last->elementIdx[i], 2);
          memcpy(buf + 2, &last->mass[i], 4);
          memcpy(buf + 6, &last->temperature[i], 4);
          md5.Update(buf, sizeof(buf));
        }
        // The building's own temperature is state this project changes on purpose too, and
        // two of the deviations reach it FIRST -- a run that hashed only cells would see the
        // waste-heat case a frame or two later than it happened, and would see nothing at all
        // if the building's conduction to the grid were ever switched off.
        for (int32_t b = 0; b < last->numBuildingTemperatures; ++b) {
          uint8_t buf[8];
          memcpy(buf, &last->buildingTemperatures[b].handle, 4);
          memcpy(buf + 4, &last->buildingTemperatures[b].temperature, 4);
          md5.Update(buf, sizeof(buf));
        }
      }
      return md5.HexDigest();
    };

    const std::string vanilla = state_of(kDevNone);
    const std::string vanilla_again = state_of(kDevNone);
    const std::string waste_heat = state_of(kDevWasteHeat);
    const std::string exhaust = state_of(kDevExhaust);
    const std::string mixture = state_of(kDevMixture);
    const std::string radiation = state_of(kDevRadiation);
    const std::string env_temp = state_of(kDevEnvTemp);
    const std::string thermal_mass = state_of(kDevThermalMass);

    printf("  vanilla      %s\n", vanilla.c_str());
    printf("  wasteheat    %s\n", waste_heat.c_str());
    printf("  exhaust      %s\n", exhaust.c_str());
    printf("  mixture      %s\n", mixture.c_str());
    printf("  radiation    %s\n", radiation.c_str());
    printf("  envtemp      %s\n", env_temp.c_str());
    printf("  thermalmass  %s\n", thermal_mass.c_str());

    // (a) the fixture is deterministic, so a golden over it is a gate and not a tripwire.
    Check(vanilla == vanilla_again,
          "the same world stepped the same number of ticks twice produces the SAME digest -- "
          "without this a golden here would be a tripwire that fires on Tuesdays");

    // (b) every deviation actually moves the world. This is the arm that fires when a message
    //     stops being DELIVERED, which a recorded digest on its own cannot notice.
    const std::string* devs[6] = {&waste_heat, &exhaust,  &mixture,
                                  &radiation,  &env_temp, &thermal_mass};
    const char* names[6] = {"wasteheat", "exhaust", "mixture",
                            "radiation", "envtemp", "thermalmass"};
    bool all_moved = true;
    for (int i = 0; i < 6; ++i) {
      if (*devs[i] == vanilla) {
        printf("  the %s deviation left the world identical to vanilla\n", names[i]);
        all_moved = false;
      }
    }
    Check(all_moved,
          "every one of the six deviations changes the published world against the vanilla "
          "control -- an extension message that stopped being delivered fails HERE, where a "
          "recorded digest alone would simply have been re-recorded as the vanilla one");

    // (c) five deviations, five outcomes: the check that fails when two branches of the
    //     fixture send the same message.
    bool all_distinct = true;
    for (int i = 0; i < 6; ++i) {
      for (int j = i + 1; j < 6; ++j) {
        if (*devs[i] == *devs[j]) {
          printf("  %s and %s produced the SAME state\n", names[i], names[j]);
          all_distinct = false;
        }
      }
    }
    Check(all_distinct,
          "and the six are six: no two deviations produce the same world, which is what fails "
          "if the fixture ever sends one message twice under two names -- and what says the "
          "environment temperature really does change what a radiator sheds");

    // The goldens themselves. One key per deviation rather than one over all six, so a failure
    // NAMES the rule that moved -- the reason `bench` keys its census per scenario.
    struct DevKey { const char* key; const std::string* digest; const char* moved; };
    const DevKey keys[7] = {
      {"state.dev.vanilla", &vanilla,
       "the CONTROL moved: something changed in vanilla stepping, not in a deviation. Check "
       "diffsim first -- if it is green, the change is in a kernel diffsim's scenarios do not "
       "reach"},
      {"state.dev.wasteheat", &waste_heat,
       "the power -> heat rule (ext::kSetBuildingWasteHeatKilowatts) produces different state"},
      {"state.dev.exhaust", &exhaust,
       "the exhaust bounce (ext::kSetBuildingExhaust) produces different state"},
      {"state.dev.mixture", &mixture,
       "the multi-gas layer or its enthalpy rule (ext::kInjectGasSpecies, sim/gas_mixture.h) "
       "produces different state"},
      {"state.dev.radiation", &radiation,
       "the radiative outlet (ext::kSetBuildingRadiation) produces different state"},
      {"state.dev.envtemp", &env_temp,
       "the same radiator against a 200 K sink instead of 0 K "
       "(ext::kSetEnvironmentTemperature) produces different state"},
      {"state.dev.thermalmass", &thermal_mass,
       "the per-cell thermal mass bonus (ext::kSetCellThermalMassBonus) produces different "
       "state"},
    };
    for (int i = 0; i < 7; ++i) {
      if (!goldens::CheckDigest(goldens_path, keys[i].key, *keys[i].digest, record_goldens,
                                noncanonical, keys[i].moved)) {
        deviation_goldens_ok = false;
      }
    }
  }

  // ------------------------------------------------------------------------------------
  // Golden RESULTS for the pure physics formulas.
  //
  // The deviations above are world state. These are not: `SIM_ComputePhaseChangeStep` and its
  // six neighbours are pure functions with no World, no message and no dispatch entry, which
  // is exactly why they are the hardest thing here to gate. A behavioural arm on a formula
  // compares against a tolerance, and a tolerance is a hole the width of the tolerance; the
  // formulas are also the part of this project a flagship mod's numbers hang off, so an
  // arithmetic change nobody meant is a balance change nobody sees.
  //
  // A swept input grid, digested over the OUTPUT BIT PATTERNS, closes that: it fails on the
  // last bit of the last mantissa, and it costs one line of GOLDENS.txt. The inputs are
  // deliberately spread across the interesting cases rather than sampled uniformly -- both
  // directions of every gradient, a zero mass, a zero rate, a threshold sitting exactly on the
  // temperature -- because a grid that only visits the easy middle of a formula pins the part
  // that was never going to move.
  //
  // PHASE CHANGE is here rather than above because there is
  // no world state to digest: the STATE (which species a tank holds, how much) correctly stays
  // managed, and only the formula is native. See sim/phase_change.h's own header.
  bool formula_goldens_ok = true;
  printf("\n=== vftest: golden results for the pure physics formulas ===\n");
  {
    // Resolved here rather than reused: the suite's `co2` is a local of an earlier block, and
    // a formula golden that silently fell back to element 0 would still hash to something.
    uint16_t formula_co2 = 0;
    if (!Resolve(t, simhost::kCarbonDioxide, "CarbonDioxide", &formula_co2)) return 1;
    goldens::Md5 md5;
    auto feed = [&](float v) {
      // Bit patterns, not values, for the same reason the state digest uses them.
      md5.Update(&v, sizeof(v));
    };

    // Deliberately irregular: a geometric-ish spread with the awkward values kept in.
    const float masses[6] = {0.0f, 0.001f, 1.0f, 20.0f, 1000.0f, 100000.0f};
    const float temps[6] = {1.0f, 100.0f, 273.15f, 300.0f, 1234.5f, 5000.0f};

    for (int mi = 0; mi < 6; ++mi) {
      for (int ti = 0; ti < 6; ++ti) {
        const float m = masses[mi];
        const float k = temps[ti];

        // Phase change: the threshold walks THROUGH the temperature, so every call site sees
        // the below/at/above branches rather than only the one the fixture happened to pick.
        for (int th = 0; th < 3; ++th) {
          const float threshold = k + (th - 1) * 50.0f;
          float converted = 0.0f, remaining = 0.0f;
          sim_compute_phase_change_step(m, k, threshold, 334000.0f, 4.179f, 0.2f, 0.1f, 0.001f,
                                        &converted, &remaining);
          feed(converted);
          feed(remaining);
        }

        feed(sim_calculate_combined_temperature(m, k, 5.0f, 300.0f));
        feed(sim_equalize_single_species_mass(0.032f, m, k, 1.0f, 2.0f, 300.0f, 1.0f, 0.5f));
        feed(sim_adiabatic_fill_temperature(1.4f, m, k, 1.0f, 300.0f));
        feed(sim_liquid_volume_from_mass(m, 1000.0f));
        feed(sim_equalize_liquid_volume_mass(1000.0f, m, 1.0f, 2.0f, 1.0f, 0.5f));

        // Gas pressure takes arrays plus one temperature and one volume for the whole cell,
        // so it is fed a two-species mixture whose second half is held fixed -- the sweep is
        // over the first, which is what the loop is varying.
        const uint16_t species[2] = {oxygen, formula_co2};
        const float massKg[2] = {m, 1.0f};
        feed(sim_compute_gas_pressure(species, massKg, 2, k, 1.0f));
      }
    }

    const std::string digest = md5.HexDigest();
    printf("  %d input rows over 7 formulas: %s\n", 36, digest.c_str());
    if (!goldens::CheckDigest(goldens_path, "formulas.digest", digest, record_goldens,
                              noncanonical,
                              "one of the pure physics formulas (sim/phase_change.h, "
                              "gas_mixture.h, liquid_mixture.h) returns different numbers. "
                              "diffsim cannot see this: none of these is called by a vanilla "
                              "kernel")) {
      formula_goldens_ok = false;
    }
  }

  // The grid raycast, through the DLL. gastest proves the walk against Klei's own
  // RadiationAbsorptionAlongLine over thousands of rays with the kernel compiled in; what only
  // the export can get wrong is everything around it -- GAME cell indices in and out (the sim
  // walks PADDED cells, and a missed conversion is off by one row and one column), the batch,
  // and a refusal that has to WRITE its result rather than leave the caller's buffer alone.
  printf("\n=== vftest: SIM_QueryRaycast ===\n");
  {
    auto sim_query_raycast =
        reinterpret_cast<int32_t (*)(const OniRaycastQuery*, OniRaycastResult*, int32_t)>(
            GetProcAddress(g_sim, "SIM_QueryRaycast"));
    Check(sim_query_raycast != nullptr, "SIM_QueryRaycast is exported");
    uint16_t rc_granite = 0, rc_water = 0, rc_glass = 0;
    if (!Resolve(t, simhost::kGranite, "Granite", &rc_granite)) return 1;
    if (!Resolve(t, kGlass, "Glass", &rc_glass)) return 1;
    if (!Resolve(t, simhost::kWater, "Water", &rc_water)) return 1;
    if (sim_query_raycast != nullptr) {
      // A vacuum world with one row of oxygen across it, water at x = 4, granite at x = 8 and a
      // window at x = 10 in that row -- glass, marked Transparent the way vanilla's Glass Tile
      // marks it. Nothing is stepped: the query reads the world as sent, properties included.
      constexpr int kW = 12, kH = 8, kRow = 3;
      auto at = [](int x, int y) { return y * kW + x; };
      World rw;
      rw.Init(kW, kH, vacuum, 0.0f, 300.0f);
      for (int x = 0; x < kW; ++x) {
        rw.element[at(x, kRow)] = oxygen;
        rw.mass[at(x, kRow)] = 1.0f;
      }
      rw.element[at(4, kRow)] = rc_water;
      rw.mass[at(4, kRow)] = 1000.0f;
      rw.element[at(8, kRow)] = rc_granite;
      rw.mass[at(8, kRow)] = 1000.0f;
      rw.element[at(10, kRow)] = rc_glass;
      rw.mass[at(10, kRow)] = 100.0f;
      rw.properties.assign(rw.Count(), 0);
      rw.properties[at(10, kRow)] = ONI_CELL_PROPERTY_TRANSPARENT | ONI_CELL_PROPERTY_SOLID_IMPERMEABLE;
      Boot(t, rw);

      const uint32_t kSolid = ONI_RAYCAST_PHASE_SOLID;
      const uint32_t kWet = ONI_RAYCAST_PHASE_LIQUID | ONI_RAYCAST_PHASE_SOLID;
      const uint32_t kSeeThrough = ONI_CELL_PROPERTY_TRANSPARENT;
      const OniRaycastQuery q[] = {
          {at(0, kRow), at(11, kRow), kSolid, 0u, 0u},     // 0: east along the row
          {at(11, kRow), at(0, kRow), kSolid, 0u, 0u},     // 1: west, from the high end: window
          {at(0, kRow), at(11, kRow), kWet, 0u, 0u},       // 2: stopped by the water first
          {at(0, kRow), at(11, kRow), 0u, 0u, 0u},         // 3: nothing stops it
          {at(11, kRow), at(0, kRow), 0u, 0u, 0u},         // 4: the same, cast back
          {at(8, 0), at(8, 7), kSolid, 0u, 0u},            // 5: up column 8 through the granite
          {at(0, kRow), at(0, 7), ONI_RAYCAST_PHASE_VACUUM, 0u, 0u},  // 6: out of the oxygen row
          {at(8, kRow), at(0, kRow), kSolid, 0u, 0u},      // 7: from inside the granite
          {999, at(0, 0), kSolid, 0u, 0u},                 // 8: invalid start
          {at(0, 0), -1, kSolid, 0u, 0u},                  // 9: invalid end
          {at(11, kRow), at(0, kRow), kSolid, 0u, kSeeThrough},  // 10: west, seeing through glass
          {at(0, kRow), at(11, kRow), 0u, ONI_CELL_PROPERTY_SOLID_IMPERMEABLE, 0u},  // 11
      };
      constexpr int32_t kQueries = static_cast<int32_t>(sizeof(q) / sizeof(q[0]));
      OniRaycastResult r[kQueries];
      memset(r, 0xCD, sizeof(r));  // poison: every result has to be WRITTEN
      const int32_t answered = sim_query_raycast(q, r, kQueries);
      for (int32_t i = 0; i < kQueries; ++i) {
        printf("  ray %d: %d -> %d  hit %d  lastClear %d  visited %d  T %.6f\n", i,
               q[i].startCell, q[i].endCell, r[i].hitCell, r[i].lastClearCell,
               r[i].cellsVisited, r[i].transmission);
      }
      Check(answered == 10, "one call answers the whole batch and counts the ten valid rays");
      Check(r[0].hitCell == at(8, kRow) && r[0].lastClearCell == at(7, kRow) &&
                r[0].cellsVisited == 9,
            "a ray cast east along the row stops on the granite, in GAME cells, with the cell "
            "before it as its last clear cell and nine cells visited");
      Check(r[1].hitCell == at(10, kRow) && r[1].lastClearCell == at(11, kRow) &&
                r[1].cellsVisited == 2,
            "cast west from the far end, solids stop it at the window first -- the high-end "
            "start, answered from the end of Klei's walk");
      Check(r[10].hitCell == at(8, kRow) && r[10].lastClearCell == at(9, kRow) &&
                r[10].cellsVisited == 4 && r[10].transmission < 1.0f,
            "and ignoring Transparent it sees through the window to the granite behind it, "
            "still dimmed by the glass it passed -- the property byte arrived in GAME cell "
            "order from the world payload and is read at the right cell");
      Check(r[11].hitCell == at(10, kRow) && r[11].lastClearCell == at(9, kRow),
            "a property mask alone stops a ray at the one cell carrying the bit, and on no "
            "cell whose element merely happens to be solid");
      Check(r[2].hitCell == at(4, kRow) && r[2].lastClearCell == at(3, kRow) &&
                r[2].cellsVisited == 5,
            "with liquid added to the stop mask the water stops it first");
      Check(r[3].hitCell == -1 && r[3].lastClearCell == at(11, kRow) && r[3].cellsVisited == 12,
            "with nothing to stop it the ray reaches its end and visits the whole row");
      Check(r[2].transmission < 1.0f && r[2].transmission >= r[0].transmission &&
                r[0].transmission >= r[3].transmission && r[3].transmission > 0.0f,
            "the transmission falls as the ray goes further through water and granite: the "
            "prefix to the water keeps at least what the prefix to the granite keeps, which "
            "keeps at least what the whole row does");
      uint32_t there = 0, back = 0;
      memcpy(&there, &r[3].transmission, 4);
      memcpy(&back, &r[4].transmission, 4);
      Check(there == back && r[4].lastClearCell == at(0, kRow),
            "an unstopped ray and the same ray cast back are the same float, bit for bit -- "
            "Klei's walk is one walk whichever end names it");
      Check(r[5].hitCell == at(8, kRow) && r[5].lastClearCell == at(8, kRow - 1) &&
                r[5].cellsVisited == kRow + 1,
            "the vertical walk finds the granite too, one row up per step");
      Check(r[6].hitCell == at(0, kRow + 1) && r[6].lastClearCell == at(0, kRow) &&
                r[6].cellsVisited == 2,
            "a vacuum stop mask stops a ray the moment it leaves the oxygen row");
      Check(r[7].hitCell == at(8, kRow) && r[7].lastClearCell == -1 && r[7].cellsVisited == 1,
            "a ray cast from inside the granite stops on its own start, with no clear cell");
      bool refused_written = true;
      for (int i = 8; i < 10; ++i) {  // rays 8 and 9 are the invalid pair
        refused_written = refused_written && r[i].hitCell == -1 && r[i].lastClearCell == -1 &&
                          r[i].cellsVisited == 0 && r[i].transmission == 0.0f;
      }
      Check(refused_written,
            "an invalid start or end is answered with -1, -1, 0 visited and 0.0 transmission "
            "WRITTEN over the caller's buffer, and is not counted in the return");

      OniRaycastResult untouched;
      memset(&untouched, 0xCD, sizeof(untouched));
      const int32_t null_q = sim_query_raycast(nullptr, &untouched, 1);
      const int32_t null_r = sim_query_raycast(q, nullptr, 1);
      const int32_t negative = sim_query_raycast(q, &untouched, -1);
      OniRaycastResult poison;
      memset(&poison, 0xCD, sizeof(poison));
      Check(null_q == -1 && null_r == -1 && negative == -1 &&
                memcmp(&untouched, &poison, sizeof(poison)) == 0,
            "a null pointer or a negative count returns -1 and writes nothing");
      Check(sim_query_raycast(q, r, 0) == 0, "an empty batch answers nothing and says so");
    }
  }

  // THE TUNABLE TABLE (ext::kSetTunable, sim/tunables.def). LAST in this file on purpose: the
  // table survives SIM_Shutdown, so an arm that tuned something and
  // then failed before its reset would change every arm after it. Each run below resets on its
  // way in and on its way out.
  //
  //   1. the discovery exports describe every row, before a world exists;
  //   2. set and read back one row of each type, and reset;
  //   3. every refusal, by its own reason, on the return value AND on "sim.message_refused",
  //      with the live value unchanged after each;
  //   4. the substep rule: settable with no world, refused with one;
  //   5. a reset restores a run bit-identical to one that never tuned;
  //   6. one POSITIVE CONTROL per group: the tuned value visibly changes what the sim does.
  //      A row whose knob is not wired passes 1-5 and fails here.
  printf("\n=== vftest: tunables -- kSetTunable, the four exports, a positive control per "
         "group ===\n");
  {
    auto tun_count = reinterpret_cast<int32_t (*)()>(GetProcAddress(g_sim, "SIM_ExtTunableCount"));
    auto tun_describe = reinterpret_cast<int32_t (*)(int32_t, OniExtTunableDesc*)>(
        GetProcAddress(g_sim, "SIM_ExtTunableDescribe"));
    auto tun_index =
        reinterpret_cast<int32_t (*)(const char*)>(GetProcAddress(g_sim, "SIM_ExtTunableIndex"));
    auto tun_get = reinterpret_cast<int32_t (*)(int32_t, uint64_t*)>(
        GetProcAddress(g_sim, "SIM_ExtTunableGet"));
    auto eq_gas = reinterpret_cast<float (*)(float, float, float, float, float, float, float,
                                             float)>(
        GetProcAddress(g_sim, "SIM_EqualizeSingleSpeciesMass"));
    auto eq_liquid = reinterpret_cast<float (*)(float, float, float, float, float, float)>(
        GetProcAddress(g_sim, "SIM_EqualizeLiquidVolumeMass"));
    Check(tun_count && tun_describe && tun_index && tun_get && eq_gas && eq_liquid,
          "the four tunable exports (and the two equalize exports the mixture control uses) "
          "are exported");
    if (!(tun_count && tun_describe && tun_index && tun_get && eq_gas && eq_liquid)) return 1;

    auto fbits = [](float v) { uint32_t b = 0; memcpy(&b, &v, 4); return static_cast<uint64_t>(b); };
    auto dbits = [](double v) { uint64_t b = 0; memcpy(&b, &v, 8); return b; };
    auto id = [&](const char* name) {
      const int32_t i = tun_index(name);
      if (i < 0) printf("  no tunable named %s\n", name);
      return i;
    };
    auto set = [&](int32_t tid, uint64_t bits, uint32_t reserved = 0) -> int32_t {
      ext::SetTunableMessage m{tid, reserved, bits};
      const void* r = sim_handle_message(ext::kSetTunable, static_cast<int>(sizeof(m)),
                                         reinterpret_cast<const uint8_t*>(&m));
      return r ? *static_cast<const int32_t*>(r) : -999;
    };
    auto reset = [&]() { return set(ext::kTunableResetAll, 0); };
    auto live = [&](int32_t tid) { uint64_t b = ~0ull; tun_get(tid, &b); return b; };

    // ---- 1. discovery, with no world allocated.
    sim_shutdown();
    InitSim();
    reset();
    const int32_t n = tun_count();
    bool echo_ok = true, index_ok = true, live_ok = true, stock_ok = true, strings_ok = true;
    int32_t ceilings = 0, no_world = 0;
    std::vector<uint64_t> defaults(n > 0 ? static_cast<size_t>(n) : 0);
    for (int32_t i = 0; i < n; ++i) {
      OniExtTunableDesc d{};
      if (tun_describe(i, &d) != 1 || d.tunableId != i) echo_ok = false;
      if (d.name[sizeof(d.name) - 1] != '\0' || d.name[0] == '\0' || d.doc[0] == '\0' ||
          d.group[0] == '\0' || d.origin[0] == '\0') {
        strings_ok = false;
      }
      if (tun_index(d.name) != i) index_ok = false;
      defaults[static_cast<size_t>(i)] = d.defaultBits;
      if (live(i) != d.defaultBits) live_ok = false;
      if (d.stockBits != d.defaultBits) stock_ok = false;
      if (d.flags & ONI_TUNABLE_FLAG_CEILING) ++ceilings;
      if (d.flags & ONI_TUNABLE_FLAG_NO_WORLD) ++no_world;
    }
    printf("  %d rows, %d ceilings, %d frozen while a world exists\n", n, ceilings, no_world);
    Check(n >= 129, "the table publishes every row (at least 129)");
    Check(echo_ok && strings_ok,
          "every row describes, echoes its id, and carries a name, group, origin and doc line");
    Check(index_ok, "SIM_ExtTunableIndex finds every row by its own name, and names are unique");
    Check(live_ok, "every live value starts at the row's default");
    Check(stock_ok, "an ordinary build's defaults are the stock rows (no build-time override)");
    Check(ceilings == 8 && no_world == 1,
          "eight CEILING rows (max is the stock value) and one row frozen while a world exists "
          "(SubstepSeconds)");
    OniExtTunableDesc untouched{};
    untouched.type = 0x5A5A5A5A;
    uint64_t untouched_bits = 0x5A5A5A5A5A5A5A5Aull;
    Check(tun_describe(-1, &untouched) == 0 && tun_describe(n, &untouched) == 0 &&
              tun_describe(0, nullptr) == 0 && untouched.type == 0x5A5A5A5A,
          "describe refuses -1, Count and a null buffer, and writes nothing");
    Check(tun_get(-1, &untouched_bits) == 0 && tun_get(n, &untouched_bits) == 0 &&
              tun_get(0, nullptr) == 0 && untouched_bits == 0x5A5A5A5A5A5A5A5Aull &&
              tun_index("NoSuchTunable") == -1 && tun_index(nullptr) == -1,
          "get refuses the same and writes nothing; an unknown name is -1");
    // Found by id rather than by `DeclaredDelivery`, which indexes `id - first` and so is off by
    // the reserved ids ("ONIR", "ONIT") for every message after them.
    int32_t tunable_delivery = -1;
    for (int32_t i = 0; i < sim_ext_message_count(); ++i) {
      ext::ExtMessageDesc md{};
      if (sim_ext_message_describe(i, &md) == 1 && md.id == ext::kSetTunable) {
        tunable_delivery = md.delivery;
      }
    }
    Check(tunable_delivery == ext::kDeliveryImmediate,
          "kSetTunable is published as IMMEDIATE, which the same-call read-backs below rely on");

    // ---- 2 and 4 (no world half). Every type, read back in the same call.
    const int32_t t_substep = id("SubstepSeconds");
    const int32_t t_cap = id("GasPressureCap");
    const int32_t t_shift = id("ConductionTileShift");
    const int32_t t_sb = id("StefanBoltzmannKW");
    const int32_t t_stencil = id("RadiationStencil07");
    Check(set(t_substep, fbits(0.1f)) == 0 && live(t_substep) == fbits(0.1f),
          "with no world allocated SubstepSeconds is settable, and reads back");
    Check(set(t_cap, fbits(0.25f)) == 0 && live(t_cap) == fbits(0.25f), "a float row sets");
    Check(set(t_shift, 5) == 0 && live(t_shift) == 5u, "an int row sets");
    Check(set(t_sb, dbits(6.0e-11)) == 0 && live(t_sb) == dbits(6.0e-11),
          "a double row sets, all 64 bits");
    Check(set(t_stencil, fbits(0.5f)) == 0 && live(t_stencil) == fbits(0.5f),
          "a stencil row sets");
    Check(reset() == 0, "a reset with no world is applied");
    bool reset_ok = true;
    for (int32_t i = 0; i < n; ++i) {
      if (live(i) != defaults[static_cast<size_t>(i)]) reset_ok = false;
    }
    Check(reset_ok, "and every row is back at its default");

    // ---- 3 and 4 (world half). A world, the refusal stream, and every reason.
    uint16_t tn_granite = 0, tn_water = 0, tn_sand = 0, tn_steam = 0;
    if (!Resolve(t, simhost::kGranite, "Granite", &tn_granite)) return 1;
    if (!Resolve(t, simhost::kWater, "Water", &tn_water)) return 1;
    if (!Resolve(t, simhost::kSand, "Sand", &tn_sand)) return 1;
    if (!Resolve(t, simhost::kSteam, "Steam", &tn_steam)) return 1;
    {
      World rw;
      rw.Init(6, 6, tn_granite, 1000.0f, 300.0f);
      Boot(t, rw);
      std::vector<uint8_t> rvis(rw.Count(), 1);
      // The subscription is itself a queued message: sent now, drained by the frame after the
      // next one, so two ticks before anything is refused.
      SubscribeStream(sim_ext_stream_index(ext::kStreamMessageRefused), true);
      Tick(rw, &rvis);
      Tick(rw, &rvis);
      const int32_t t_maxt = id("MaxTemperature");
      const int32_t t_base = id("StableTicksRerollBase");
      const int32_t t_cmin = id("ConductionMinTemperature");
      const int32_t t_cmax = id("ConductionMaxTemperature");
      struct Refusal {
        const char* what;
        int32_t got;
        int32_t want;
        int32_t row;
      };
      const float nan = std::numeric_limits<float>::quiet_NaN();
      std::vector<Refusal> r;
      r.push_back({"an id with no row", set(n, fbits(1.0f)), ext::kExtRefusalBadTarget, -1});
      r.push_back({"a negative id other than -1", set(-2, 0), ext::kExtRefusalBadTarget, -1});
      r.push_back({"a NaN", set(t_cap, fbits(nan)), ext::kExtRefusalNotFinite, t_cap});
      r.push_back({"an infinite double",
                   set(t_sb, dbits(std::numeric_limits<double>::infinity())),
                   ext::kExtRefusalNotFinite, t_sb});
      r.push_back({"a value past the row's max", set(t_cap, fbits(1.5f)),
                   ext::kExtRefusalOutOfRange, t_cap});
      r.push_back({"a value under the row's min", set(t_shift, static_cast<uint32_t>(0)),
                   ext::kExtRefusalOutOfRange, t_shift});
      r.push_back({"a CEILING raised past its stock value", set(t_maxt, fbits(10001.0f)),
                   ext::kExtRefusalOutOfRange, t_maxt});
      r.push_back({"reserved not 0", set(t_cap, fbits(0.2f), 1u), ext::kExtRefusalReservedBits,
                   t_cap});
      r.push_back({"a 32-bit row's high 32 bits set", set(t_cap, fbits(0.2f) | (1ull << 32)),
                   ext::kExtRefusalReservedBits, t_cap});
      r.push_back({"SubstepSeconds moved with a world allocated", set(t_substep, fbits(0.1f)),
                   ext::kExtRefusalWorldLoaded, t_substep});
      r.push_back({"a countdown roll reaching the 31 sentinel (base 28 + 3)",
                   set(t_base, 28), ext::kExtRefusalCrossField, t_base});
      bool returns_ok = true;
      for (const Refusal& x : r) {
        if (x.got != x.want) {
          printf("  %s: returned %d, expected %d\n", x.what, x.got, x.want);
          returns_ok = false;
        }
      }
      Check(returns_ok, "every refusal returns its own reason");
      // A floor above its ceiling is inside each row's own range, so only the pair rule can
      // refuse it -- and it must, whichever of the two is the one that moves.
      Check(set(t_cmax, fbits(400.0f)) == 0 &&
                set(t_cmin, fbits(500.0f)) == ext::kExtRefusalCrossField &&
                set(t_cmin, fbits(100.0f)) == 0 &&
                set(t_cmax, fbits(50.0f)) == ext::kExtRefusalCrossField,
          "floor-above-ceiling is refused whichever of the pair moves");
      Check(set(t_substep, fbits(0.2f)) == 0,
            "re-sending SubstepSeconds' current value with a world allocated is accepted (a "
            "config replayed after a load does not fail on it)");
      Check(set(t_base, 27) == 0 && live(t_base) == 27u,
            "base 27 + 3 = 30 is the largest roll the 5-bit field holds, and is accepted");
      bool unchanged = true;
      for (int32_t row : {t_cap, t_sb, t_maxt, t_shift, t_substep}) {
        if (live(row) != defaults[static_cast<size_t>(row)]) unchanged = false;
      }
      Check(unchanged, "no refused message changed a live value");
      {
        ext::SetTunableMessage m{t_cap, 0, fbits(0.2f)};
        const void* res = sim_handle_message(ext::kSetTunable, 8,
                                             reinterpret_cast<const uint8_t*>(&m));
        Check(res && *static_cast<const int32_t*>(res) == ext::kExtRefusalShortPayload &&
                  live(t_cap) == defaults[static_cast<size_t>(t_cap)],
              "a short payload is refused and changes nothing");
      }
      // The frame the next tick returns was built before these refusals (the worker runs one
      // frame ahead of the game), so their records ride the one after it. Collect both.
      std::vector<ext::ExtRefusedMessage> rec = Refusals(Tick(rw, &rvis));
      const std::vector<ext::ExtRefusedMessage> rec2 = Refusals(Tick(rw, &rvis));
      rec.insert(rec.end(), rec2.begin(), rec2.end());
      bool stream_ok = true;
      for (int32_t reason : {ext::kExtRefusalBadTarget, ext::kExtRefusalNotFinite,
                             ext::kExtRefusalOutOfRange, ext::kExtRefusalReservedBits,
                             ext::kExtRefusalWorldLoaded, ext::kExtRefusalCrossField,
                             ext::kExtRefusalShortPayload}) {
        if (!HasRefusal(rec, ext::kSetTunable, reason)) {
          printf("  no stream record for reason %d\n", reason);
          stream_ok = false;
        }
      }
      Check(stream_ok, "and each reason is on sim.message_refused as well");
      Check(set(ext::kTunableResetAll, 0) == 0 &&
                live(t_base) == defaults[static_cast<size_t>(t_base)] &&
                live(t_cmax) == defaults[static_cast<size_t>(t_cmax)],
            "a reset with a world allocated is applied (SubstepSeconds is at its default, so "
            "the reset does not move it)");
      Check(set(t_substep, fbits(0.2f)) == 0 && reset() == 0, "and leaves the table clean");
    }

    // ---- 5. a reset restores a run bit-identical to one that never tuned.
    auto mixed_run = [&](bool tune_first, bool reset_after_tuning) -> uint64_t {
      reset();
      sim_shutdown();
      InitSim();
      constexpr int kW = 12, kH = 12;
      auto at = [](int x, int y) { return y * kW + x; };
      World w;
      w.Init(kW, kH, vacuum, 0.0f, 300.0f);
      for (int i = 0; i < kW; ++i) {
        w.element[at(i, 0)] = w.element[at(i, kH - 1)] = w.element[at(0, i)] =
            w.element[at(kW - 1, i)] = tn_granite;
        w.mass[at(i, 0)] = w.mass[at(i, kH - 1)] = w.mass[at(0, i)] = w.mass[at(kW - 1, i)] =
            1000.0f;
      }
      for (int x = 1; x < kW - 1; ++x) {
        w.element[at(x, 1)] = tn_water;
        w.mass[at(x, 1)] = 400.0f + 40.0f * x;
        w.temperature[at(x, 1)] = 290.0f + 3.0f * x;
        w.element[at(x, 6)] = oxygen;
        w.mass[at(x, 6)] = 0.5f + 0.2f * x;
      }
      w.element[at(3, 8)] = tn_sand;
      w.mass[at(3, 8)] = 800.0f;
      w.element[at(8, 4)] = tn_granite;
      w.mass[at(8, 4)] = 500.0f;
      w.temperature[at(8, 4)] = 450.0f;
      if (tune_first) {
        set(id("GasPressureCap"), fbits(0.25f));
        set(id("TransitionMargin"), fbits(0.0f));
        set(id("StableTicksRerollBase"), 10);
        set(id("MinConductionDelta"), fbits(5.0f));
        set(id("LiquidHorizontalRate"), fbits(0.5f));
        if (reset_after_tuning) reset();
      }
      Boot(t, w);
      std::vector<uint8_t> vis(w.Count(), 1);
      const GameDataUpdate* f = nullptr;
      for (int i = 0; i < 40; ++i) f = Tick(w, &vis);
      uint64_t h = 1469598103934665603ull;
      auto mix = [&](const void* p, size_t bytes) {
        const uint8_t* b = static_cast<const uint8_t*>(p);
        for (size_t k = 0; k < bytes; ++k) h = (h ^ b[k]) * 1099511628211ull;
      };
      if (f != nullptr) {
        mix(f->elementIdx, w.Count() * 2);
        mix(f->mass, w.Count() * 4);
        mix(f->temperature, w.Count() * 4);
      }
      reset();
      return h;
    };
    const uint64_t never = mixed_run(false, false);
    const uint64_t tuned = mixed_run(true, false);
    const uint64_t restored = mixed_run(true, true);
    printf("  40-tick digests: never tuned %016llx, tuned %016llx, tuned then reset %016llx\n",
           static_cast<unsigned long long>(never), static_cast<unsigned long long>(tuned),
           static_cast<unsigned long long>(restored));
    Check(tuned != never, "sanity check: the tuned run is a different run");
    Check(restored == never,
          "a reset restores a run bit-identical to one that never tuned (nothing but the "
          "message moves the table)");

    // ---- 6. the positive controls. Each is `probe(tuned)`, run twice, compared.
    auto fresh = [&]() {
      reset();
      sim_shutdown();
      InitSim();
    };
    auto boxed = [&](int size, uint16_t fill, float fill_mass, float temp) {
      World w;
      w.Init(size, size, fill, fill_mass, temp);
      for (int i = 0; i < size; ++i) {
        for (int c : {i, i * size, (size - 1) * size + i, i * size + size - 1}) {
          w.element[c] = tn_granite;
          w.mass[c] = 1000.0f;
        }
      }
      return w;
    };

    // Heat: MinConductionDelta, and the ConductionMaxTemperature ceiling LOWERED live.
    auto heat = [&](int mode) {
      fresh();
      World w;
      w.Init(9, 9, tn_granite, 2000.0f, 300.0f);
      w.temperature[4 * 9 + 4] = 340.0f;
      if (mode == 1) set(id("MinConductionDelta"), fbits(100.0f));
      if (mode == 2) set(id("ConductionMaxTemperature"), fbits(320.0f));
      Boot(t, w);
      std::vector<uint8_t> vis(w.Count(), 1);
      const GameDataUpdate* f = nullptr;
      for (int i = 0; i < 3; ++i) f = Tick(w, &vis);
      const float v = f ? f->temperature[4 * 9 + 4] : -1.0f;
      reset();
      return v;
    };
    const float heat0 = heat(0), heat1 = heat(1), heat2 = heat(2);
    printf("  heat: hot cell after 3 ticks %.4f K default, %.4f K MinConductionDelta 100, "
           "%.4f K ConductionMaxTemperature 320\n", heat0, heat1, heat2);
    Check(heat0 < 340.0f && heat1 == 340.0f,
          "Heat: MinConductionDelta 100 stops a 40 K pair exchanging at all");
    Check(heat2 <= 320.0f && heat0 > 320.0f,
          "Heat: a lowered ConductionMaxTemperature ceiling clamps a conducting cell to it");

    // Phase: TransitionMargin. Water 2 K past its boiling line, inside the default 3 K margin.
    auto phase = [&](bool tuned_run) {
      fresh();
      World w;
      w.Init(7, 7, tn_granite, 2000.0f, 374.5f);
      w.element[3 * 7 + 3] = tn_water;
      w.mass[3 * 7 + 3] = 1000.0f;
      if (tuned_run) set(id("TransitionMargin"), fbits(0.0f));
      Boot(t, w);
      std::vector<uint8_t> vis(w.Count(), 1);
      const GameDataUpdate* f = nullptr;
      for (int i = 0; i < 2; ++i) f = Tick(w, &vis);
      const uint16_t e = f ? f->elementIdx[3 * 7 + 3] : 0xFFFF;
      reset();
      return e;
    };
    const uint16_t phase0 = phase(false), phase1 = phase(true);
    printf("  phase: water at 374.5 K is element %u by default, %u with TransitionMargin 0 "
           "(water %u, steam %u)\n", phase0, phase1, tn_water, tn_steam);
    Check(phase0 == tn_water && phase1 == tn_steam,
          "Phase: TransitionMargin 0 boils water the default 3 K margin holds");

    // Gas: GasPressureCap. Oxygen released into a vacuum box. The cap bounds one pair's transfer
    // at a share of the giving cell's START mass; oxygen's own flow rate stays under the stock
    // 0.125 here (10 kg -> 2.8 kg over eight neighbours), so RAISING it changes nothing and this
    // LOWERS it until it binds. Measured before it was written that way: 0.25 moved exactly what
    // 0.125 did.
    auto gas = [&](bool tuned_run) {
      fresh();
      World w = boxed(9, vacuum, 0.0f, 300.0f);
      w.element[4 * 9 + 4] = oxygen;
      w.mass[4 * 9 + 4] = 10.0f;
      if (tuned_run) set(id("GasPressureCap"), fbits(0.05f));
      Boot(t, w);
      std::vector<uint8_t> vis(w.Count(), 1);
      const GameDataUpdate* f = nullptr;
      for (int i = 0; i < 2; ++i) f = Tick(w, &vis);
      const float v = f ? f->mass[4 * 9 + 4] : -1.0f;
      reset();
      return v;
    };
    const float gas0 = gas(false), gas1 = gas(true);
    printf("  gas: source cell after its first moving tick %.4f kg default, %.4f kg "
           "GasPressureCap 0.05\n", gas0, gas1);
    Check(gas0 < 10.0f && gas1 > gas0,
          "Gas: a GasPressureCap low enough to bind keeps more gas in the source");

    // Liquid: LiquidHorizontalRate. A water column on the floor of a vacuum box.
    auto liquid = [&](bool tuned_run) {
      fresh();
      World w = boxed(9, vacuum, 0.0f, 300.0f);
      w.element[1 * 9 + 4] = tn_water;
      w.mass[1 * 9 + 4] = 1000.0f;
      if (tuned_run) set(id("LiquidHorizontalRate"), fbits(0.0f));
      Boot(t, w);
      std::vector<uint8_t> vis(w.Count(), 1);
      const GameDataUpdate* f = nullptr;
      for (int i = 0; i < 3; ++i) f = Tick(w, &vis);
      const float v = f ? f->mass[1 * 9 + 4] : -1.0f;
      reset();
      return v;
    };
    const float liq0 = liquid(false), liq1 = liquid(true);
    printf("  liquid: source cell after 3 ticks %.4f kg default, %.4f kg LiquidHorizontalRate "
           "0\n", liq0, liq1);
    Check(liq0 < 1000.0f && liq1 > liq0,
          "Liquid: LiquidHorizontalRate 0 keeps water from spreading sideways as it does by "
          "default");

    // Rng: StableTicksRerollBase. Sand over a vacuum shaft; the tick it leaves its cell.
    auto rng = [&](bool tuned_run) {
      fresh();
      World w = boxed(9, vacuum, 0.0f, 300.0f);
      w.element[7 * 9 + 4] = tn_sand;
      w.mass[7 * 9 + 4] = 500.0f;
      if (tuned_run) set(id("StableTicksRerollBase"), 25);
      Boot(t, w);
      std::vector<uint8_t> vis(w.Count(), 1);
      int left = -1;
      for (int i = 0; i < 60 && left < 0; ++i) {
        const GameDataUpdate* f = Tick(w, &vis);
        if (f && f->elementIdx[7 * 9 + 4] != tn_sand) left = i;
      }
      reset();
      return left;
    };
    const int rng0 = rng(false), rng1 = rng(true);
    printf("  rng: unsupported sand leaves its cell at tick %d by default, %d with "
           "StableTicksRerollBase 25\n", rng0, rng1);
    Check(rng0 >= 0 && rng1 >= 25 && rng1 - rng0 >= 15,
          "Rng: a larger StableTicksRerollBase makes unsupported sand wait that much longer");

    // Mixture: GasEqualizeDamping and LiquidEqualizeDamping, through the two pure exports. 1.0
    // is twice 0.5, and doubling is exact in float, so the tuned result is exactly twice.
    reset();
    const float eg0 = eq_gas(32.0f, 2.0f, 300.0f, 1.0f, 1.0f, 300.0f, 1.0f, 0.2f);
    const float el0 = eq_liquid(1000.0f, 800.0f, 1.0f, 400.0f, 1.0f, 0.2f);
    set(id("GasEqualizeDamping"), fbits(1.0f));
    set(id("LiquidEqualizeDamping"), fbits(1.0f));
    const float eg1 = eq_gas(32.0f, 2.0f, 300.0f, 1.0f, 1.0f, 300.0f, 1.0f, 0.2f);
    const float el1 = eq_liquid(1000.0f, 800.0f, 1.0f, 400.0f, 1.0f, 0.2f);
    reset();
    printf("  mixture: gas equalize %.6f -> %.6f kg, liquid %.6f -> %.6f kg at damping 1.0\n",
           eg0, eg1, el0, el1);
    Check(eg0 != 0.0f && eg1 == 2.0f * eg0 && el0 != 0.0f && el1 == 2.0f * el0,
          "Mixture: damping 1.0 moves exactly twice what the default 0.5 moves, gas and liquid");

    // Building: CellEnergyMinMassKg. ModifyCellEnergy into half a kilogram of oxygen in vacuum.
    auto building = [&](bool tuned_run) {
      fresh();
      World w = boxed(9, vacuum, 0.0f, 300.0f);
      w.element[4 * 9 + 4] = oxygen;
      w.mass[4 * 9 + 4] = 0.5f;
      if (tuned_run) set(id("CellEnergyMinMassKg"), fbits(1.0f));
      Boot(t, w);
      std::vector<uint8_t> vis(w.Count(), 1);
      ModifyCellEnergyMessage m{};
      m.cellIdx = 4 * 9 + 4;
      m.kilojoules = 50.0f;
      m.maxTemperature = 1000.0f;
      m.id = -1;
      sim_handle_message(static_cast<int32_t>(SimMessageHash::ModifyCellEnergy), sizeof(m),
                         reinterpret_cast<const uint8_t*>(&m));
      const GameDataUpdate* f = nullptr;
      for (int i = 0; i < 2; ++i) f = Tick(w, &vis);
      float hottest = 0.0f;
      for (size_t c = 0; f && c < w.Count(); ++c) {
        if (f->elementIdx[c] == oxygen) hottest = std::max(hottest, f->temperature[c]);
      }
      reset();
      return hottest;
    };
    const float bld0 = building(false), bld1 = building(true);
    printf("  building: hottest oxygen after 50 kJ %.2f K default, %.2f K CellEnergyMinMassKg "
           "1\n", bld0, bld1);
    Check(bld0 > 350.0f && bld1 < 301.0f,
          "Building: CellEnergyMinMassKg 1 refuses the energy the default 0.001 kg gate lands");

    // Building: CellEnergyCarry. A thousand 0.1 kJ ModifyCellEnergy payments into one
    // 1000 kg water cell at 283.15 K. Each is under one float temperature step (~0.13 kJ there),
    // so Klei's write lands a whole step every time and the books show the difference as
    // refused. With the row on, the cell carries the remainder: nothing is refused, and what
    // landed plus what the carry still holds is exactly what was paid. The ceiling arm pays into
    // a cell the message's own ceiling stops: that stays a refusal, and the carry holds nothing.
    struct CarryRun { double asked, landed, refused, held; };
    auto carry_run = [&](bool on, float ceiling) {
      fresh();
      World w = boxed(9, tn_water, 1000.0f, 283.15f);
      if (on) set(id("CellEnergyCarry"), 1);
      Boot(t, w);
      std::vector<uint8_t> vis(w.Count(), 1);
      double b[41] = {0}, a[41] = {0};
      const int published = sim_debug_energy_ledger ? sim_debug_energy_ledger(b, 41) : 0;
      ModifyCellEnergyMessage m{};
      m.cellIdx = 4 * 9 + 4;
      m.kilojoules = 0.1f;
      m.maxTemperature = ceiling;
      m.id = -1;
      for (int i = 0; i < 1000; ++i) {
        sim_handle_message(static_cast<int32_t>(SimMessageHash::ModifyCellEnergy), sizeof(m),
                           reinterpret_cast<const uint8_t*>(&m));
      }
      Tick(w, &vis);
      if (published >= 41) sim_debug_energy_ledger(a, 41);
      reset();
      return CarryRun{1000.0 * static_cast<double>(m.kilojoules), a[11] - b[11], a[12] - b[12],
                      a[40] - b[40]};
    };
    const bool ledger41 = sim_debug_energy_ledger && sim_debug_energy_ledger(nullptr, 0) >= 41;
    Check(ledger41, "the energy ledger publishes field 40, the energy CellEnergyCarry holds");
    if (ledger41) {
      const CarryRun off = carry_run(false, 10000.0f), on = carry_run(true, 10000.0f);
      const CarryRun cap = carry_run(true, 283.16f);
      printf("  carry off: asked %.6f landed %.6f refused %+.6f held %+.6f kJ\n", off.asked,
             off.landed, off.refused, off.held);
      printf("  carry on:  asked %.6f landed %.6f refused %+.6f held %+.6f kJ\n", on.asked,
             on.landed, on.refused, on.held);
      printf("  ceiling:   asked %.6f landed %.6f refused %+.6f held %+.6f kJ\n", cap.asked,
             cap.landed, cap.refused, cap.held);
      Check(std::fabs(off.refused) > 1.0 && off.held == 0.0,
            "Building: with CellEnergyCarry off, Klei's float write mis-lands small payments by "
            "more than 1 kJ in 100 (the defect the row exists for)");
      Check(std::fabs(on.refused) < 1e-6 && std::fabs(on.landed + on.held - on.asked) < 1e-6 &&
                std::fabs(on.held) < 0.2,
            "Building: with CellEnergyCarry on, nothing is refused and landed + held is exactly "
            "what was paid, the carry under one float step");
      Check(cap.refused > 1.0 && cap.held == 0.0 &&
                std::fabs(cap.landed + cap.refused - cap.asked) < 1e-6,
            "Building: a payment the message's ceiling stops is still refused, and the carry "
            "holds none of it");
    }

    // The next three need a world the shared `SendWorld` does not send: radiation ON, germs, or
    // the property textures (not headless). Same payload, three switches.
    auto send_world_ex = [&](const World& w, bool radiation, bool headless, int32_t germ_cell,
                             int32_t germs) {
      Writer b;
      b.Put<int32_t>(w.width);
      b.Put<int32_t>(w.height);
      b.Put<uint32_t>(12345u);
      b.PutBool(radiation);
      b.PutBool(headless);
      for (size_t i = 0; i < w.Count(); ++i) {
        Cell c{};
        c.elementIdx = w.element[i];
        c.mass = w.mass[i];
        c.temperature = w.temperature[i];
        c.insulation = 255;
        b.PutRaw(&c, sizeof(Cell));
      }
      for (size_t i = 0; i < w.Count(); ++i) {
        DiseaseCell d{};
        d.diseaseIdx = static_cast<int32_t>(i) == germ_cell ? 0 : 0xFF;
        d.elementCount = static_cast<int32_t>(i) == germ_cell ? germs : 0;
        b.PutRaw(&d, sizeof(DiseaseCell));
      }
      for (size_t i = 0; i < w.Count(); ++i) {
        SimBackwall bw{};
        b.PutRaw(&bw, sizeof(SimBackwall));
      }
      Send(SimMessageHash::SimData_InitializeFromCells, b);
      return static_cast<const GameDataUpdate*>(SendEmpty(SimMessageHash::Start));
    };
    auto boot_ex = [&](const World& w, bool radiation, bool headless, int32_t germ_cell,
                       int32_t germs) {
      SendRaw(SimMessageHash::Elements_CreateTable, t.elements);
      SendRaw(SimMessageHash::Disease_CreateTable, t.diseases);
      return send_world_ex(w, radiation, headless, germ_cell, germs);
    };

    // Disease: DiseaseDiffusionShare. Germs in one granite cell; do its neighbours get any?
    auto disease = [&](bool tuned_run) {
      fresh();
      World w;
      w.Init(9, 9, tn_granite, 1000.0f, 300.0f);
      if (tuned_run) set(id("DiseaseDiffusionShare"), fbits(0.0f));
      // Well over granite's minDiffusionCount (1,000,000): at exactly the threshold, the first
      // tick's die-off leaves the cell under it and nothing diffuses whatever the share.
      boot_ex(w, false, true, 4 * 9 + 4, 100000000);
      std::vector<uint8_t> vis(w.Count(), 1);
      const GameDataUpdate* f = nullptr;
      for (int i = 0; i < 3; ++i) f = Tick(w, &vis);
      int64_t spread = 0;
      for (int c : {4 * 9 + 3, 4 * 9 + 5, 3 * 9 + 4, 5 * 9 + 4}) {
        if (f) spread += f->diseaseCount[c];
      }
      reset();
      return spread;
    };
    const int64_t dis0 = disease(false), dis1 = disease(true);
    printf("  disease: germs in the four neighbours after 3 ticks: %lld default, %lld "
           "DiseaseDiffusionShare 0\n", static_cast<long long>(dis0), static_cast<long long>(dis1));
    Check(dis0 > 0 && dis1 == 0, "Disease: DiseaseDiffusionShare 0 stops germs spreading");

    // Radiation: the MaxRadiation ceiling, lowered live, clamps a field it did not before.
    auto radiation = [&](bool tuned_run) {
      fresh();
      World w;
      w.Init(9, 9, vacuum, 0.0f, 300.0f);
      if (tuned_run) set(id("MaxRadiation"), fbits(10.0f));
      boot_ex(w, true, true, -1, 0);
      std::vector<uint8_t> vis(w.Count(), 1);
      CellRadiationModification add{};
      add.cellIdx = 4 * 9 + 4;
      add.radiationDelta = 100000.0f;
      add.callbackIdx = -1;
      sim_handle_message(static_cast<int32_t>(SimMessageHash::CellRadiationModification),
                         sizeof(add), reinterpret_cast<const uint8_t*>(&add));
      const GameDataUpdate* f = nullptr;
      for (int i = 0; i < 2; ++i) f = Tick(w, &vis);
      const float v = f ? f->radiation[4 * 9 + 4] : -1.0f;
      reset();
      return v;
    };
    const float field_default = radiation(false), field_capped = radiation(true);
    printf("  radiation: cell after 1e5 rads %.2f default, %.2f MaxRadiation 10\n", field_default, field_capped);
    Check(field_default > 10.0f && field_capped <= 10.0f && field_capped >= 0.0f,
          "Radiation: a lowered MaxRadiation ceiling clamps the field to it");

    // Texture: FillAlphaScale. The liquid property texture of a half-full water cell.
    auto texture = [&](bool tuned_run) {
      fresh();
      World w = boxed(9, vacuum, 0.0f, 300.0f);
      for (int x = 1; x < 8; ++x) {
        w.element[1 * 9 + x] = tn_water;
        w.mass[1 * 9 + x] = 300.0f;
      }
      if (tuned_run) set(id("FillAlphaScale"), fbits(0.002f));
      boot_ex(w, false, false, -1, 0);
      std::vector<uint8_t> vis(w.Count(), 1);
      const GameDataUpdate* f = nullptr;
      for (int i = 0; i < 3; ++i) f = Tick(w, &vis);
      std::vector<uint8_t> tex;
      if (f && f->propertyTextureLiquid) {
        const uint8_t* p = static_cast<const uint8_t*>(f->propertyTextureLiquid);
        tex.assign(p, p + w.Count() * 4);
      }
      reset();
      return tex;
    };
    const std::vector<uint8_t> tex0 = texture(false), tex1 = texture(true);
    const size_t probe = (1 * 9 + 4) * 4;
    printf("  texture: liquid texel of a 300 kg water cell %s default, %s FillAlphaScale "
           "0.002\n", tex0.empty() ? "(none)" : "published", tex1.empty() ? "(none)" : "published");
    Check(!tex0.empty() && !tex1.empty() && tex0 != tex1 &&
              memcmp(tex0.data() + probe, tex1.data() + probe, 4) != 0,
          "Texture: FillAlphaScale changes the liquid texture a water cell publishes");

    reset();
    printf("  NOT covered by a positive control here, each for its reason: ConductionTileShift "
           "(perf only: no observable behaviour by design); the Fizz group (effervescence is off "
           "unless configured, so its rows need the effervescence arm's setup; covered by the "
           "framework's live arms)\n");
  }

  sim_shutdown();
  printf("\n%s\n", g_fail_count == 0 ? "done: all PASS" : "done: FAILURES PRESENT");
  const bool goldens_ok = goldens::CheckCount(goldens_path, "vftest.checks", g_check_count,
                                              record_goldens, noncanonical);
  return (g_fail_count == 0 && goldens_ok && deviation_goldens_ok && formula_goldens_ok) ? 0 : 1;
}
