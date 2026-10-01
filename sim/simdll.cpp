// The replacement SimDLL.
//
// Sixteen exports and one message queue, matching Klei's C ABI exactly. What is here is
// the *shape* of the sim: boot, message dispatch, the projection contract, save and
// load, plus conduction and flow. What is deliberately not here yet: state changes and
// element interactions, so water heated past 372 K stays water.
//
// That is a real milestone rather than a placeholder: it is what lets the differential
// harness (driver/src/diffsim.cpp) compare us against Klei tick for tick, and it means
// every divergence it reports from now on is a physics divergence rather than an ABI
// one. A world already at equilibrium should match Klei exactly today.
//
// Not installed over the game. Build with sim/build.sh; the harness loads it directly.

// First, and before <windows.h>: it pulls in <winsock2.h>, which must precede the Winsock 1
// declarations <windows.h> would otherwise bring in.
#include "kprofiler.h"

#include <windows.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <cstring>
#include <string>
#include <vector>

#include "../abi/sim_abi.h"
#include "../abi/sim_abi_ext.h"
#include "buildings.h"
#include "cellmod.h"
#include "chunks.h"
#include "disease.h"
#include "conduits.h"
#include "emitters.h"
#include "environment.h"
#include "fields.h"
#include "radiation.h"
#include "raycast.h"
#include "payload_mix.h"
#include "effervescence.h"
#include "physics.h"
#include "projection.h"
#include "ext_events.h"
#include "ext_state.h"
#include "registry_state.h"
#include "saveblob.h"
#include "textures.h"
#include "version.h"
#include "world.h"
// The gas-mixture layer: gas_mixture.h/gas_rooms.h wired into the DLL's own substep and
// message pipeline.
#include "gas_mixture.h"
#include "gas_rooms.h"
#include "liquid_mixture.h"
#include "phase_change.h"

using namespace oni_sim;

namespace {

using GameMessageHandler = int (*)(int, void*);

// ------------------------------------------------------------- the published frame
//
// The game reads the arrays a `GameDataUpdate` names for the whole of its tick, and once the
// sim runs on its own thread it is filling the *next* frame while that read is happening. So
// what the update points at cannot be the sim's own vectors: `Project` and
// `FillPropertyTextures` are incremental and keep theirs across frames on purpose, which is
// exactly what makes them the wrong thing to hand out.
//
// Klei's answer is two whole `SimData` objects, `CopySimDataToGame` filling one and the game
// side swapping the pointers. This is the same answer with one
// allocation instead of forty: every array the update names is copied into `bytes` and the
// update's pointer patched to the copy. Two of these alternate, so the frame the game is
// reading is never the frame the worker is filling.
//
// `bytes` is never shrunk and never re-zeroed — `Begin` only resets the write cursor — so
// after the first frame a publication is one pass of memcpy into a buffer already the right
// size. That pass is the price of threading and it is paid on the worker, off the frame.
struct PublishedFrame {
  std::vector<uint8_t> bytes;
  std::vector<std::pair<void**, size_t>> fixups;
  size_t used = 0;
  GameDataUpdate update{};
  // Published properties. Not reachable through `update` -- `GameDataUpdate` is Klei's struct and
  // its 496 bytes are a layout contract we do not get to extend -- so it hangs off the frame
  // and `SIM_ExtPublishedProperties` resolves it from the frame pointer the game is holding.
  // Same buffer, same lifetime, same alternation: the two rules the ABI documents.
  std::vector<ext::ExtPublishedProperty> ext;
  // Published event streams, here for exactly the same reason: one descriptor per SUBSCRIBED
  // event stream, resolved from the frame pointer by `SIM_ExtPublishedEvents`.
  std::vector<ext::ExtPublishedStream> streams;

  void Begin() {
    used = 0;
    fixups.clear();
    ext.clear();
    streams.clear();
    memset(&update, 0, sizeof(update));
  }

  // The pointer cannot be handed out here: a later `Put` may still grow `bytes` and move it.
  // Offsets are recorded instead and resolved once, in `Finish`.
  template <typename T, typename V>
  void Put(T*& slot, const std::vector<V>& src) {
    const size_t off = (used + 7u) & ~static_cast<size_t>(7);
    const size_t n = src.size() * sizeof(V);
    if (off + n > bytes.size()) bytes.resize(off + n);
    if (n != 0) memcpy(bytes.data() + off, src.data(), n);
    used = off + n;
    fixups.emplace_back(reinterpret_cast<void**>(&slot), off);
  }

  // Same contract as `Put`, for a source that is not a `std::vector<V>` and a slot that is
  // not a member of `update`. The caller must have finished sizing whatever holds `slot`
  // before calling: the fixup is an address, and a vector that reallocates afterwards takes
  // its elements with it.
  void PutBytes(const void*& slot, const void* src, size_t n) {
    const size_t off = (used + 7u) & ~static_cast<size_t>(7);
    if (off + n > bytes.size()) bytes.resize(off + n);
    if (n != 0) memcpy(bytes.data() + off, src, n);
    used = off + n;
    fixups.emplace_back(const_cast<void**>(&slot), off);
  }

  void Finish() {
    for (const auto& f : fixups) *f.first = bytes.data() + f.second;
  }
};

// The numbers the frame uses that have no other home are tunables (sim/tunables.def): the room
// pooled-mixing rate, cadence and the substeps a room must stay still before it sleeps (ours,
// the first guess gastest's benchmarks were measured with, never tuned against gameplay), and
// Klei's world-border cell, written over every border cell at init: 9999 kg at 0 K.

struct Sim {
  GameMessageHandler callback = nullptr;
  ElementTable elements;
  DiseaseTable diseases;
  World world;
  ProjectionBuffers buffers;
  ProjectionEvents projection_events;
  PropertyTextureBuffers textures;
  // Buildings live outside the grid and outside the world, so they survive a load the way
  // Klei's do: the save blob has no room for them and the game re-registers every one.
  BuildingState buildings;
  // Element chunks: matter the game holds outside the grid. Same lifetime as buildings —
  // not in the save blob, re-registered by the game after every load.
  ElementChunkState chunks;
  // Element consumers and emitters: the two components that move matter between a building
  // and the grid. First and second in `SimData`'s component list, so they run before chunks.
  ElementFlowState flow;
  // The radiation emitter: third in the component list, and the only owner of the
  // `radiation` array other than the game itself.
  RadiationState radiation;
  // The generic field solver. Not a fifth registry and not a fourth component
  // list -- a field owns no storage at all. It is a registered arity-1 F32 cell property in
  // `world.ExtCells()` plus a propagation rule attached to it by index, so persistence, the
  // save blob, the checkpoint, the zero-copy publish and the managed read path all come from
  // registry 1 unchanged. What lives here is the rule, its sources, and the two scratch
  // buffers the kernels reuse.
  //
  // Deliberately NOT cleared by AllocateCells, unlike the four registries above. Those hold
  // handles the game re-registers after every load; a field's rule and its sources belong to
  // the mod that pushed them, which has no load hook telling it to push them again.
  ext::FieldState fields;
  // The game's visibility mask, three buffers deep the way Klei's is. `visible` is
  // `SimData::visibleGrid`, what a frame reads: the solid emitter's ore drop and the falling
  // liquid both refuse a cell the player cannot see. `visible_game` is the `visibleGrid` of
  // the two `GameData` buffers `FrameSync` alternates. `PrepareGameData` copies the payload
  // into the game-side one and flips which is which (`GameSyncFunction` writes the
  // game-side buffer, then `FrameSync::GameSync` swaps the pair); the end of every frame
  // swaps `visible` with the sim-side one (`GameData::swapVisibleGrid` at the tail of
  // `CopySimDataToGame`). So a frame sees the mask the game sent two `PrepareGameData`s
  // earlier, and the first frames of a world see a zeroed one and hand nothing over.
  std::vector<uint8_t> visible;
  std::vector<uint8_t> visible_game[2];
  int visible_sim_slot = 0;
  bool debug_editing = false;
  // ext::kSetLoadIsRestore's one-shot flag: the next Load is a checkpoint restore and skips
  // the load-time state transition. Cleared by that Load whether or not it succeeds.
  bool load_is_restore = false;
  // Conduit contents are not part of the world either, and unlike buildings they are driven
  // from the game thread between frames rather than from the frame itself.
  ConduitTemperatures conduits;

  bool started = false;
  bool first_frame = true;
  int skip_physics_frames = 1;
  float elapsed_seconds = 0.0f;
  // Leftover frame time that did not add up to a whole substep, carried forward.
  float substep_carry = 0.0f;
  // Klei spares one massless cell per region from being zeroed, but only the first time.
  bool first_physics_substep = true;
  // The game's substep counter: a uint16 zeroed at construction and incremented at the very
  // end of every substep. `DisplaceGas` starts
  // its neighbour scan at it, so it decides which of several equally good cells receives the
  // gas a liquid pushes aside. It belongs to the SimData object, not to the world contents:
  // it is *not* reset by InitializeFromCells or by a load, only by an Alloc.
  uint16_t displace_rotation = 0;
  // The game's sweep direction: a signed 1 negated at the top of every substep, so it
  // alternates once per substep. It is the column order the gas
  // pressure sweep walks in *and* the horizontal neighbour each cell pairs with, and it
  // therefore also picks which of the two upward diagonals exists this substep. Same
  // lifetime as `displace_rotation`: SimData state, not world contents.
  int32_t pressure_dir = -1;

  // Per-frame event lists. GameDataUpdate carries count+pointer pairs into these, and
  // the game reads them before the next call, so clearing them at the top of each frame
  // is safe and is what keeps a stale event from being replayed.
  // Transition ores, collected per substep and drained into spawn_ore_info at the end of
  // the frame — a frame can run several substeps and each of them may produce drops.
  std::vector<StateChangeOre> state_change_ores;

  std::vector<SpawnOreInfo> dig_info;
  std::vector<SpawnOreInfo> spawn_ore_info;
  std::vector<MassConsumedCallback> mass_consumed;
  std::vector<DiseaseConsumptionCallback> disease_consumed;
  std::vector<MassEmittedCallback> mass_emitted;
  std::vector<CallbackInfo> callbacks;
  // Unstable solids the post-process sweep handed to the game this frame. Produced by a
  // kernel rather than by the projection, so unlike `substanceChangeInfo` it cannot be
  // recovered by comparing two frames — it has to be collected as it happens.
  std::vector<UnstableCellInfo> unstable_cells;
  // `SimEvents::spawnLiquidInfo`: liquid the flow sweep handed to the game as falling
  // particles this frame, in the order it was handed over. See `StepFlow`.
  std::vector<SpawnFallingLiquidInfo> falling_liquid;
  // `SimEvents::worldDamageInfo`: one record per wall cell an over-full liquid broke this
  // frame, source cell alongside. See `DoPressureBreak`.
  std::vector<WorldDamageInfo> world_damage;
  std::vector<BackwallShouldTransitionInfo> backwall_transitions;
  std::vector<ComponentStateChangedMessage> component_state;

  // Event streams: the same idea with the list of ten unhardcoded. Declared natively at
  // `SIM_Initialize` (see `RegisterFirstPartyStreams`), collected only while somebody is
  // subscribed, published as descriptors beside the ten above, and cleared at the END of the
  // publish rather than the top of the frame -- `ext_events.h`'s `ClearFrame` says why.
  ext::EventStreamRegistry ext_events;
  // Resolved once at declaration so the refusal path is an index compare and not a string
  // lookup. -1 until `RegisterFirstPartyStreams` runs, which is what makes `EmitRefusal`
  // safe to call from anywhere including before the streams exist.
  int32_t stream_refused = -1;
  // Layer C: the two liquid-payload streams (abi/sim_abi_ext.h, kSetCellPropertyTransport), and
  // what a frame could not hand over because nobody was subscribed or the stream was full.
  // Indexed `propertyIdx * kExtMaxArity + component`. Never reset: it is a running total, the
  // payload counterpart of the mass ledger's buckets, so "released but unheard" has a number.
  int32_t stream_payload_released = -1;
  int32_t stream_payload_consumed = -1;
  std::vector<double> payload_unreported =
      std::vector<double>(static_cast<size_t>(ext::CellPropertyRegistry::kMaxProperties) *
                              static_cast<size_t>(ext::kExtMaxArity),
                          0.0);

  // Two published frames, alternating: the worker fills one while the game reads the other.
  PublishedFrame pub[2];
  int pub_slot = 0;
  // What the last completed frame published. Written by whichever thread ran that frame and
  // read by the game thread only after it has waited for the worker, so that wait is the
  // synchronisation and there is nothing else to lock.
  GameDataUpdate* last_published = nullptr;
  // The frame the GAME is holding: what the last `PrepareGameData` (or `Start`) handed back,
  // and the game-cell count its arrays were published at. Game thread only -- set where the
  // frame is handed over, read by `ConduitTemperatureManager_Update`, which runs on the game
  // thread while the worker fills the OTHER published frame -- so its three cell arrays are
  // readable there without a barrier. Null after an Alloc or a Load until the next hand-over,
  // because a frame published for the old world is sized for it. The CONVECTION conduit
  // policy's cell side (sim/conduits.h) is the one reader.
  const GameDataUpdate* game_frame = nullptr;
  size_t game_frame_cells = 0;
  // False until a frame has been published *into* the pipeline. The first `PrepareGameData`
  // after a Start, an Alloc or a Load runs its frame on the calling thread and primes it;
  // every one after that returns the frame the worker finished during the game's tick.
  bool pipelined = false;
  std::vector<uint8_t> save_blob;

  // Messages arrive between frames and take effect when the frame runs, exactly as they
  // do in Klei's sim. Applying them on receipt would let the game observe a half-stepped
  // world, and it is also how the real sim behaves — a probe that saves before ticking
  // sees none of them.
  struct Pending {
    int32_t id;
    std::vector<uint8_t> payload;
  };
  // Klei's `SimFrameManager::currentFrame`: everything that has arrived since the last
  // `NewGameFrame`. `SimFrameManager::HandleMessage` puts every queued message
  // into it, whatever the message is.
  std::vector<Pending> queue;
  // And the frame that is actually being processed. `NewGameFrame` closes `currentFrame` into
  // `queuedFrames` (`SimFrameManager::NewFrame`) and `BeginFrameProcessing`
  // moves that to `activeFrames` — but the frame the game *observes* through a given
  // `PrepareGameData` is the one before it, because the sim runs on its own thread and the
  // call publishes what has already finished rather than waiting for what it just queued.
  //
  // So a message sent during tick N takes effect on tick N+1, always, and that is a
  // property of the boundary rather than of the harness's waits. Measured directly:
  // `diffsim --scenario chunkenergy --chunks` sends 5000 kJ before tick 3 and Klei's chunk
  // does not move until tick 4, twice over, on both of its energy messages.
  //
  // Nothing had ever tested it. Every scenario in the suite sends its messages before tick 1
  // and `skip_physics_frames` covers that one case by running no substep; the chunk probes
  // are the first thing here to send a message to a component that is already running.
  std::vector<Pending> active;

  int32_t next_handle = 0;
  std::vector<std::pair<int32_t, int32_t>> unknown_messages;  // id -> count

  // False until the first ext::kInjectGasSpecies message
  // this world ever receives — see ApplyInjectGasSpecies. Kept in Sim, not World: gas_rooms.h
  // needs World (for the SoA accessors), so World cannot also hold a RoomGraph without a
  // header cycle. vf_rooms is rebuilt whenever its size no longer matches the world's
  // PaddedCount, which self-heals across an Allocate/resize/load without a separate hook.
  bool vf_active = false;
  gas::RoomGraph vf_rooms;
  std::vector<uint8_t> vf_stable;  // per game cell, sized to World::GameCount()
  uint64_t vf_tick = 0;
  // The mixing cadence is the tunable `RoomMixingEveryNTicks`. `gas::TickRates`' other two
  // cadences have no kernel yet, so they are not held here: a cadence nothing reads is not one.

  // ext::kSetBlockedGasAddPolicy. Here and not on World for the reason the header gives: it is
  // a session setting, so an Allocate or a Load must not reset it, and it is never saved.
  int32_t blocked_gas_add_policy = ext::kBlockedGasAddVanilla;
};

Sim* g = nullptr;

// ------------------------------------------------------------------ the sim thread
//
// Klei's sim runs on its own thread and meets the game at a strict
// alternating rendezvous over a double buffer, so a sim frame overlaps the game's *render*
// rather than the game's sim call. Nothing about what gets published changes — the same
// bytes in the same frame order — which is why the test for this is `diffsim` coming back
// unchanged rather than any new golden.
//
// Raw Win32 rather than <thread>: this toolchain is `x86_64-w64-mingw32-g++` with the win32
// thread model, where the standard threading headers are not dependable, and two auto-reset
// events are the whole protocol anyway.
GameDataUpdate* RunFrame();

// The mask a frame reads: `Sim::visible`, zeroed to the grid's size when it is not a full
// mask, because Klei's buffers start zeroed. Never null, so an unseen cell is refused.
const uint8_t* FrameVisible() {
  if (g->visible.size() != g->world.GameCount()) g->visible.assign(g->world.GameCount(), 0);
  return g->visible.data();
}

// A new world: all three visibility buffers start zeroed again.
void ResetVisibility() {
  g->visible.clear();
  g->visible_game[0].clear();
  g->visible_game[1].clear();
  g->visible_sim_slot = 0;
}

struct FrameWorker {
  HANDLE thread = nullptr;
  HANDLE work = nullptr;  // game -> sim: run a frame
  HANDLE done = nullptr;  // sim -> game: the frame is published
  // Game thread only, and true exactly between a `Kick` and the `WaitIdle` that collects it.
  bool busy = false;
  // Set once, before the thread starts, and read by both after that.
  bool enabled = false;
  volatile long quit = 0;
};
FrameWorker g_worker;

DWORD WINAPI WorkerMain(LPVOID) {
  for (;;) {
    WaitForSingleObject(g_worker.work, INFINITE);
    if (g_worker.quit != 0) break;
    RunFrame();
    SetEvent(g_worker.done);
  }
  return 0;
}

// Everything that is not a queued message touches state the worker owns, so it waits here
// first. With no frame in flight this is one predicted branch.
void WaitIdle() {
  if (!g_worker.busy) return;
  WaitForSingleObject(g_worker.done, INFINITE);
  g_worker.busy = false;
}

void Kick() {
  g_worker.busy = true;
  SetEvent(g_worker.work);
}

void StartWorker() {
  g_worker = FrameWorker();
  // An escape hatch for bisecting a divergence: `sim_nothread.on` in the working directory puts
  // the frame back on the calling thread *and* takes the pipeline out with it, restoring the
  // arrangement every golden in the suite was taken under.
  if (FILE* marker = fopen("sim_nothread.on", "rb")) {
    fclose(marker);
    return;
  }
  g_worker.work = CreateEventA(nullptr, FALSE, FALSE, nullptr);
  g_worker.done = CreateEventA(nullptr, FALSE, FALSE, nullptr);
  if (!g_worker.work || !g_worker.done) return;
  g_worker.thread = CreateThread(nullptr, 0, WorkerMain, nullptr, 0, nullptr);
  if (!g_worker.thread) return;
  g_worker.enabled = true;
}

void StopWorker() {
  WaitIdle();
  if (g_worker.thread != nullptr) {
    g_worker.quit = 1;
    SetEvent(g_worker.work);
    WaitForSingleObject(g_worker.thread, INFINITE);
    CloseHandle(g_worker.thread);
  }
  if (g_worker.work != nullptr) CloseHandle(g_worker.work);
  if (g_worker.done != nullptr) CloseHandle(g_worker.done);
  g_worker = FrameWorker();
}

void Report(const char* text) {
  if (!g || !g->callback) return;
  // GameHandledMessages.ReportMessage: the payload's first pointer-sized field is a
  // char* the game turns into a log line.
  struct {
    const char* message;
    const char* callstack;
  } msg{text, nullptr};
  g->callback(0, &msg);
}

// One record onto "sim.message_refused" -- see that stream's block in
// `abi/sim_abi_ext.h` for what is and is not reported.
//
// Costs a load and two predicted branches when nobody is subscribed, which is the normal case
// and is why this is safe to put on the message path at all. It deliberately reports nothing
// back: a refusal is already a thing that went wrong, and a caller that had to check whether
// reporting it worked would need a second diagnostic for the first one.
// Layer C. Moves this frame's liquid-payload records out of the World and onto their streams.
// A record a stream will not take -- unsubscribed, or past the per-frame cap -- is not lost
// without trace: its amount goes on the property's unreported total.
void FlushLiquidPayloadRecords() {
  auto unheard = [](int32_t prop, int32_t component, float amount) {
    if (prop < 0 || component < 0 || component >= ext::kExtMaxArity) return;
    const size_t k = static_cast<size_t>(prop) * static_cast<size_t>(ext::kExtMaxArity) +
                     static_cast<size_t>(component);
    if (k < g->payload_unreported.size()) g->payload_unreported[k] += amount;
  };
  std::vector<ext::LiquidPayloadReleased>& rel = g->world.PayloadReleased();
  for (const ext::LiquidPayloadReleased& r : rel) {
    if (!g->ext_events.Emit(g->stream_payload_released, &r, sizeof(r))) {
      unheard(r.propertyIdx, r.component, r.amount);
    }
  }
  rel.clear();
  std::vector<ext::LiquidPayloadConsumed>& con = g->world.PayloadConsumed();
  for (const ext::LiquidPayloadConsumed& r : con) {
    if (!g->ext_events.Emit(g->stream_payload_consumed, &r, sizeof(r))) {
      unheard(r.propertyIdx, r.component, r.amount);
    }
  }
  con.clear();
}

void EmitRefusal(int32_t message_id, ext::ExtRefusalReason reason, int32_t got, int32_t want) {
  if (g == nullptr || g->stream_refused < 0) return;
  if (!g->ext_events.Subscribed(g->stream_refused)) return;
  ext::ExtRefusedMessage rec{};
  rec.messageId = message_id;
  rec.reason = static_cast<int32_t>(reason);
  rec.payloadBytes = got;
  rec.expectedBytes = want;
  g->ext_events.Emit(g->stream_refused, &rec, sizeof(rec));
}

// `payload_bytes` is what arrived, for the refusal record. It is not used for the tally --
// that has always been per id -- and defaults to 0 for a caller that genuinely has no length
// to report.
void NoteUnknown(int32_t id, size_t payload_bytes = 0) {
  EmitRefusal(id, ext::kExtRefusalUnknownMessage, static_cast<int32_t>(payload_bytes), 0);
  for (auto& p : g->unknown_messages) {
    if (p.first == id) {
      ++p.second;
      return;
    }
  }
  g->unknown_messages.emplace_back(id, 1);
}

// ------------------------------------------------------------------ live profiler
//
// `ToggleProfiler` is a message *into* the sim — the game sends it from the backtick key
// (`DebugHandler`, `Sim.SIM_HandleMessage(-409964931, 0, null)`), and Klei answers it with a
// `kprofiler` statically linked into their DLL. This answers it with per-kernel timings,
// because there is otherwise no way to see where a frame goes in a colony somebody actually
// built: `bench` runs offline, on scenarios somebody wrote, and does not call `StepPhysics`
// at all. These are the only numbers this project can take from a real world.
//
// Off costs one predicted branch per kernel call and nothing else — no timer is read. State
// lives outside `Sim` on purpose, so a toggle survives a load, an Alloc and a Shutdown; a
// profiling run that ended when the player reloaded would be useless.
struct Profiler {
  // The slot enum and its names live in `sim/census.h`, not here. `bench` links the kernels
  // directly out of `sim/` and never sees this file, so a private copy of the list would give
  // the offline harness and the live DLL two sets of names for one set of kernels.
  using Slot = census::Slot;
  // Enumerators are not imported by the type alias, and every `PROF(kFoo)` in this file names
  // them unqualified through the macro. One line each rather than a second enum, so adding a
  // kernel is still a single edit in census.h -- and a slot added there and forgotten here is
  // a compile error at its first use, not a silently missing row.
  static constexpr Slot kDrainQueue = census::kDrainQueue;
  static constexpr Slot kConduction = census::kConduction;
  static constexpr Slot kStateChange = census::kStateChange;
  static constexpr Slot kGasPressure = census::kGasPressure;
  static constexpr Slot kGasDisplacement = census::kGasDisplacement;
  static constexpr Slot kFlow = census::kFlow;
  static constexpr Slot kPayloadMix = census::kPayloadMix;
  static constexpr Slot kPostProcess = census::kPostProcess;
  static constexpr Slot kDisease = census::kDisease;
  static constexpr Slot kZeroMassless = census::kZeroMassless;
  static constexpr Slot kElementFlow = census::kElementFlow;
  static constexpr Slot kRadiation = census::kRadiation;
  static constexpr Slot kFields = census::kFields;
  static constexpr Slot kElementChunk = census::kElementChunk;
  static constexpr Slot kBuildingHeat = census::kBuildingHeat;
  static constexpr Slot kBuildingToBuilding = census::kBuildingToBuilding;
  static constexpr Slot kVolumeFractions = census::kVolumeFractions;
  static constexpr Slot kEffervescence = census::kEffervescence;
  static constexpr Slot kWorldEnvironment = census::kWorldEnvironment;
  static constexpr Slot kProject = census::kProject;
  static constexpr Slot kTextures = census::kTextures;
  static constexpr Slot kFrame = census::kFrame;
  static constexpr int kSlotCount = census::kSlotCount;
  static const char* Name(int s) { return census::Name(s); }

  bool on = false;
  int64_t freq = 0;
  double ms[kSlotCount] = {};
  int64_t calls[kSlotCount] = {};
  int frames = 0;

  // A scope, not a start/stop pair: several of the kernels below sit inside a region loop
  // that an early return could leave.
  struct Scope {
    int slot;
    int64_t t0;
    bool kprofiler;  // a kprofiler section was opened, so one must be closed
    explicit Scope(int s);
    ~Scope();
    Scope(const Scope&) = delete;
    Scope& operator=(const Scope&) = delete;
  };
};

Profiler g_prof;

// The same kernel scopes feed Klei's kprofiler (`sim/kprofiler.h`) while a capture is running,
// so a trace taken from the game shows the sim worker's kernels nested in its frame. Names are
// recorded once; the worker names itself the first time it records under a session.
void KprofilerBeginKernel(int slot) {
  static uint64_t ids[Profiler::kSlotCount];
  static uint64_t category = 0;
  static std::atomic<bool> named{false};
  if (!named.load(std::memory_order_relaxed)) {
    for (int i = 0; i < Profiler::kSlotCount; ++i) {
      ids[i] = kprof::G().strings.Record(Profiler::Name(i));
    }
    category = kprof::G().strings.Record("SimDLL");
    named.store(true);
  }
  static thread_local int named_thread_session = -1;
  const int session = kprof::G().session.load(std::memory_order_relaxed);
  if (named_thread_session != session) {
    named_thread_session = session;
    kprof::SetThreadInfo(0, kprof::G().strings.Record("SimDLL frame worker"), category);
  }
  kprof::BeginSection(ids[slot], category, -1);
}

Profiler::Scope::Scope(int s) : slot(g_prof.on ? s : -1), t0(0), kprofiler(kprof::Running()) {
  if (kprofiler) KprofilerBeginKernel(s);
  if (slot < 0) return;
  LARGE_INTEGER t;
  QueryPerformanceCounter(&t);
  t0 = t.QuadPart;
}

Profiler::Scope::~Scope() {
  if (kprofiler) kprof::EndSection(-1);
  if (slot < 0) return;
  LARGE_INTEGER t;
  QueryPerformanceCounter(&t);
  g_prof.ms[slot] += static_cast<double>(t.QuadPart - t0) * 1000.0 /
                     static_cast<double>(g_prof.freq);
  ++g_prof.calls[slot];
}

#define PROF(slot) Profiler::Scope prof_scope_##slot(Profiler::slot)

// Both halves on purpose. `Report` goes through the game's own message handler and lands in
// `Player.log`, which is the only channel out of a sim DLL that needs no file at all — but a
// log the player has to scroll is a bad place to compare two builds, so the same lines are
// appended to `sim_profile.log` in the process working directory (the game root, where
// `PerformanceCaptureData.json` lands too). A failed fopen is silent: an unwritable install
// must not cost the caller its timings.
void EmitProfile() {
  FILE* f = fopen("sim_profile.log", "a");
  char line[192];

  auto emit = [&](const char* text) {
    Report(text);
    if (f) fprintf(f, "%s\n", text);
  };

  const double frames = g_prof.frames > 0 ? static_cast<double>(g_prof.frames) : 1.0;
  snprintf(line, sizeof(line), "sim profiler: %d frames, %.3f ms/frame",
           g_prof.frames, g_prof.ms[Profiler::kFrame] / frames);
  emit(line);

  for (int s = 0; s < Profiler::kSlotCount; ++s) {
    if (g_prof.calls[s] == 0) continue;
    if (s == Profiler::kFrame) continue;
    // Per *frame*, not per call: a kernel runs once per region per substep, so its call
    // count is the wrong denominator for anything a 16.67 ms budget is compared against.
    snprintf(line, sizeof(line), "  %-26s %8.3f ms/frame  %8.4f ms/call  (%lld calls)",
             Profiler::Name(s), g_prof.ms[s] / frames,
             g_prof.ms[s] / static_cast<double>(g_prof.calls[s]),
             static_cast<long long>(g_prof.calls[s]));
    emit(line);
    // The census row, immediately under the timing it belongs to and only for the sweeps it
    // means anything for (sim/census.h). Cells per frame, and the two ratios that say why the
    // milliseconds above are what they are: how much of what the sweep looked at it changed,
    // and how much of what it looked at its own region actually asked for. A kernel at 100 %
    // of the region is doing what it was told; one far above it is scanning.
    const census::Counters& c = census::g_counts[s];
    if (!census::IsCellSweep(s) || c.examined == 0) continue;
    const double stood = static_cast<double>(c.examined - c.skipped);
    const double asked = static_cast<double>(census::g_region_cells) *
                         (static_cast<double>(c.invocations) / frames);
    snprintf(line, sizeof(line),
             "      cells %10.0f/frame  stood %5.1f%%  changed %6.2f%%  of region %6.1f%%",
             static_cast<double>(c.examined) / frames,
             c.examined ? 100.0 * stood / static_cast<double>(c.examined) : 0.0,
             c.examined ? 100.0 * static_cast<double>(c.changes) /
                              static_cast<double>(c.examined)
                        : 0.0,
             asked > 0.0 ? 100.0 * (static_cast<double>(c.examined) / frames) / asked : 0.0);
    emit(line);
  }
  // Work announced with no sweep running: queued `ModifyCell`s, emitters, buildings. Its own
  // row rather than folded into a kernel's, because charging it to one would be a lie and
  // dropping it would make the change counts fail to add up to the frame's actual traffic.
  if (census::g_counts[census::kSlotCount].changes > 0) {
    snprintf(line, sizeof(line), "  %-26s %10.1f changes/frame (messages, emitters, buildings)",
             "outside any sweep",
             static_cast<double>(census::g_counts[census::kSlotCount].changes) / frames);
    emit(line);
  }
  if (f) {
    fprintf(f, "\n");  // one blank line per run: the file is appended to, not replaced
    fclose(f);
  }
}

// Arming, factored out of `ToggleProfiler` because there are now two ways in -- the backtick
// key and `SIM_DebugSetProfiler` -- and two copies of "zero the timings and the census in the
// same breath" is exactly the drift that would let a live reading and a keyboard reading
// describe two different windows while looking like one table.
void ArmProfiler() {
  LARGE_INTEGER f;
  QueryPerformanceFrequency(&f);
  g_prof.freq = f.QuadPart ? f.QuadPart : 1;
  for (int s = 0; s < Profiler::kSlotCount; ++s) {
    g_prof.ms[s] = 0.0;
    g_prof.calls[s] = 0;
  }
  // The census is zeroed with the timings, not separately: the two are read as one table and
  // a cell count accumulated over a different window than the milliseconds beside it is worse
  // than no cell count at all.
  census::Reset();
  g_prof.frames = 0;
  g_prof.on = true;
}

// The message carries no payload, so there is no "start" and "stop" to distinguish: the
// first press arms and zeroes, the second reports and disarms. Zeroing on arm rather than on
// disarm is what makes a second run comparable to the first.
void ToggleProfiler() {
  if (g_prof.on) {
    g_prof.on = false;
    EmitProfile();
    return;
  }
  ArmProfiler();
  Report("sim profiler: on (press again to report)");
}

// ------------------------------------------------------------------ queued messages

template <typename T>
bool Payload(const Sim::Pending& p, T* out) {
  if (p.payload.size() < sizeof(T)) {
    // This covers all 56 `if (!Payload(...)) return;` sites: a short payload is reported, not
    // silently dropped, so a mod can tell a write that landed from one that was thrown away.
    EmitRefusal(p.id, ext::kExtRefusalShortPayload, static_cast<int32_t>(p.payload.size()),
                static_cast<int32_t>(sizeof(T)));
    return false;
  }
  memcpy(out, p.payload.data(), sizeof(T));
  return true;
}

// The same check for a message that has not been queued — the immediate path holds raw bytes
// rather than a `Sim::Pending`. Short payloads are refused rather than zero-filled: a caller
// that sent the wrong struct should hear about it, not get defaults.
template <typename T>
bool PayloadBytes(const uint8_t* msg, size_t len, T* out) {
  if (msg == nullptr || len < sizeof(T)) return false;
  memcpy(out, msg, sizeof(T));
  return true;
}

// Forward-declared: defined below alongside ApplyMassEmission, the handler it was written
// for. ApplyModifyCell (just above that definition in the file) needs it too, for the same
// promoted-gas-cell redirect -- see this addendum's comment at its ApplyModifyCell call site.
inline bool IsPromotedGasCell(const gas::RoomGraph* rooms, size_t cell, const ElementTable& table,
                              uint16_t elementIdx);
// Defined with the volume-fractions handlers below; the blocked-gas promotion needs it too.
void ActivateVolumeFractions();

// Promotion's rooms pointer, under the one guard every gate in this file uses: the graph is
// only trusted when it is sized for this world.
const gas::RoomGraph* LiveRooms() {
  return (g->vf_active && g->vf_rooms.room_of.size() == g->world.PaddedCount()) ? &g->vf_rooms
                                                                                : nullptr;
}

// A gas add that lands in a promoted cell's mixture. Shared by ModifyCell's two mixture paths
// -- the cell was already promoted, or the add was blocked and the room was promoted for it.
//
// The incoming temperature is blended into the cell's one shared temperature field first,
// over everything already there (vanilla PhaseEntry plus every mixture slot), WEIGHTED BY HEAT
// CAPACITY, m*c, so the cell's sum of m*c*T after the add is the sum before plus the new gas's
// own m*c*T: the energy the add brings is the energy that lands. That is Stationeers'
// accounting -- gas adds its energy, never a temperature -- and `ApplyInjectGasSpecies` does
// the same, through the shared BlendGasTemperatureIntoCell below. Without the blend, a hot or
// cold gas add into a promoted room would arrive at the room's temperature and its energy
// difference would be lost.
// The blend itself, shared with ApplyInjectGasSpecies: the cell's one temperature field takes
// `mass` of `element` at `temperature`, weighted by m*c over the vanilla PhaseEntry and every
// mixture slot. Call it BEFORE the mass is added.
void BlendGasTemperatureIntoCell(size_t cell, uint16_t element, float mass, float temperature) {
  const PhaseEntry& here = g->world.Phase(cell);
  double existing_hc =
      static_cast<double>(here.mass) * g->elements.At(here.element).specificHeatCapacity;
  const uint8_t mask = g->world.GasOccupiedMask(cell);
  for (int s = 0; s < gas::kMaxSpeciesPerCell; ++s) {
    if (!(mask & (1u << s))) continue;
    existing_hc += static_cast<double>(g->world.GasMass(cell, s)) *
                   g->elements.At(g->world.GasSpecies(cell, s)).specificHeatCapacity;
  }
  const double incoming_hc =
      static_cast<double>(mass) * g->elements.At(element).specificHeatCapacity;
  if (existing_hc + incoming_hc > 0.0) {
    g->world.Phase(cell).temperature = static_cast<float>(
        (existing_hc * here.temperature + incoming_hc * temperature) /
        (existing_hc + incoming_hc));
  }
}

void MixGasIntoPromotedCell(size_t cell, uint16_t element, float mass, float temperature,
                            uint8_t disease_idx, int32_t disease_count) {
  BlendGasTemperatureIntoCell(cell, element, mass, temperature);
  gas::InjectSpecies(g->world, cell, element, mass);
  // Same wake-on-arrival rule every other redirect here follows.
  if (g->world.GasSleeping(cell)) {
    g->world.MutableGasSleeping(cell) = 0;
    gas::RoomOnCellWoke(g->vf_rooms, cell);
  }
  // Disease stays vanilla's concern, same as ApplyMassEmission's redirect.
  if (disease_idx != 0xFF) {
    AddDiseaseToCell(&g->world, g->diseases, cell, disease_idx, disease_count);
  }
  // No NoteEmitted: this mass never touched the vanilla ledger, same as every other
  // mixture-layer entry point.
}

// ext::kSetBlockedGasAddPolicy's other half: the add was blocked, so promote the cell's room,
// absorb its gas, and mix the new gas in. False when there is no room to promote -- the cell
// is not open to the room graph -- and the caller then falls back to Klei's tail.
//
// The graph is brought up to date first. `ActivateVolumeFractions` builds it when the world
// has never had one; a pending geometry change (`RoomsDirty`) would otherwise be picked up at
// the next frame, after this promotion had already been written against the old room ids.
bool PromoteAndMixBlockedGas(size_t cell, uint16_t element, float mass, float temperature,
                             uint8_t disease_idx, int32_t disease_count) {
  if (!g->vf_active || g->vf_rooms.room_of.size() != g->world.PaddedCount()) {
    ActivateVolumeFractions();
  } else if (g->world.RoomsDirty()) {
    g->vf_rooms = gas::BuildRoomGraph(g->world, g->elements, g->world.GameWidth(),
                                       g->world.GameHeight());
    g->world.MutableRoomsDirty() = false;
  }
  const int32_t room = g->vf_rooms.room_of[cell];
  if (room < 0) return false;
  gas::PromoteRoomAndAbsorbGas(g->world, g->elements, g->vf_rooms, room);
  MixGasIntoPromotedCell(cell, element, mass, temperature, disease_idx, disease_count);
  return true;
}

// A ModifyCell liquid add (a falling drop landing, a building's liquid output, a debug spawn).
// Klei's `AddLiquid` sees a mixture cell as vacuum and would write its temperature over the
// one the mixture shares, so the mixture gas steps aside first, as the liquid sweeps make it
// do. When no neighbour can take it, the add goes ahead and the gas stays until the next
// mixing tick moves it (`EvictMixtureFromCondensedCells`); the cell's one temperature is then
// blended over both layers by heat capacity, so the gas neither gains nor loses heat.
void AddLiquidBesideMixture(const CellModContext& cm, size_t cell, uint16_t element, float mass,
                            float temperature, uint8_t disease_idx, int32_t disease_count) {
  gas::RoomGraph* rooms = LiveRooms() != nullptr ? &g->vf_rooms : nullptr;
  if (!gas::IsMixtureOwnedCell(g->world, rooms, cell) ||
      gas::EvictMixture(g->world, g->elements, rooms, cell, g->displace_rotation)) {
    AddLiquid(cm, cell, element, mass, temperature, disease_idx, disease_count);
    return;
  }
  double slots_hc = 0.0;
  const uint8_t mask = g->world.GasOccupiedMask(cell);
  for (int s = 0; s < gas::kMaxSpeciesPerCell; ++s) {
    if (!(mask & (1u << s))) continue;
    slots_hc += static_cast<double>(g->world.GasMass(cell, s)) *
                g->elements.At(g->world.GasSpecies(cell, s)).specificHeatCapacity;
  }
  const double slots_t = g->world.Phase(cell).temperature;
  AddLiquid(cm, cell, element, mass, temperature, disease_idx, disease_count);
  const PhaseEntry& after = g->world.Phase(cell);
  const double cell_hc =
      after.mass > 0.0f
          ? static_cast<double>(after.mass) * g->elements.At(after.element).specificHeatCapacity
          : 0.0;
  if (cell_hc + slots_hc > 0.0) {
    g->world.Phase(cell).temperature = static_cast<float>(
        (cell_hc * after.temperature + slots_hc * slots_t) / (cell_hc + slots_hc));
  }
}

void ApplyModifyCell(const Sim::Pending& p) {
  ModifyCellMessage m{};
  if (!Payload(p, &m) || !g->world.ValidGameCell(m.cellIdx)) return;
  const size_t cell = g->world.Padded(static_cast<size_t>(m.cellIdx));
  // The projection walks rectangles now, and a message can write a cell no kernel drove.
  // The paths below mark the further cells they reach; this covers the named one, which
  // some of them leave untouched.
  g->world.MarkProjectDirty(cell);

  const CellModContext cm{&g->world, &g->elements, &g->diseases, g->displace_rotation};

  // The clamp. It is a *conditional* clamp — the values are only touched when
  // the message is already out of range, so an in-range negative-mass message keeps its
  // temperature. Klei reports it to the crash handler on the way through and carries on.
  float temperature = m.temperature;
  float mass = m.mass;
  const float max_t = g_tunables.cellmod_max_temperature;  // a message handler, not a kernel
  if ((temperature <= 0.0f && mass > 0.0f) || temperature > max_t) {
    if (temperature >= max_t) temperature = max_t;
    if (temperature <= 0.0f) temperature = 0.0f;
    if (mass <= 0.0f) {
      mass = 0.0f;
      temperature = 0.0f;
    }
  }

  switch (m.replaceType) {
    case kReplaceNone: {
      if (m.elementIdx >= static_cast<uint16_t>(g->elements.Count())) break;
      // Dispatched on the phase of the element in the **message**, not the one in the cell.
      switch (g->elements.Phase(m.elementIdx)) {
        case kStateGas:
          if (mass > 0.0f) {
            // ModifyCell is a THIRD gas-writing path, distinct from the MassConsumption/
            // MassEmission pair (ElementConsumer/ElementEmitter). A duplicant's exhaled CO2
            // arrives this way: `CO2Manager.SpawnBreath` calls `SimMessages.ModifyMass`, which
            // calls `ModifyCell` directly -- not an ElementEmitter -- so without this it would
            // sit in a promoted room as a small vanilla pocket instead of mixing. Same redirect as `ApplyMassEmission`'s, scoped to exactly
            // the branch that matches it (`kReplaceNone`, gas, adding mass) -- `kReplaceElement`/
            // `kReplaceAndDisplace` and the mass<=0 removal branch below are NOT covered by
            // this redirect.
            if (IsPromotedGasCell(LiveRooms(), cell, g->elements, m.elementIdx)) {
              MixGasIntoPromotedCell(cell, m.elementIdx, mass, temperature, m.diseaseIdx,
                                     m.diseaseCount);
            } else if (g->blocked_gas_add_policy == ext::kBlockedGasAddPromoteAndMix) {
              // ext::kSetBlockedGasAddPolicy. Klei's AddGas runs unchanged up to the point it
              // would delete, and hands the blocked case back instead.
              bool blocked = false;
              AddGas(cm, cell, m.elementIdx, mass, temperature, m.diseaseIdx, m.diseaseCount,
                     &blocked);
              if (blocked && !PromoteAndMixBlockedGas(cell, m.elementIdx, mass, temperature,
                                                      m.diseaseIdx, m.diseaseCount)) {
                AddIntoBlockedCell(cm, cell, m.elementIdx, mass, temperature, m.diseaseIdx,
                                   m.diseaseCount, true);
              }
            } else {
              AddGas(cm, cell, m.elementIdx, mass, temperature, m.diseaseIdx, m.diseaseCount);
            }
          } else {
            RemoveFromCell(cm, cell, kStateGas, mass);
          }
          break;
        case kStateLiquid:
          if (mass > 0.0f) {
            AddLiquidBesideMixture(cm, cell, m.elementIdx, mass, temperature, m.diseaseIdx,
                                   m.diseaseCount);
          } else if (mass < 0.0f) {
            RemoveFromCell(cm, cell, kStateLiquid, mass);
          }
          break;
        case kStateSolid:
          if (mass > 0.0f) {
            AddSolid(cm, cell, m.elementIdx, mass, temperature, m.diseaseIdx, m.diseaseCount,
                     m.addSubType);
          } else if (mass < 0.0f) {
            RemoveFromCell(cm, cell, kStateSolid, mass);
          }
          break;
        default:
          // Vacuum. reports "Invalid replacement type" and does nothing else,
          // so a message that adds Vacuum to a cell is silently dropped.
          break;
      }
      break;
    }
    case kReplaceElement:
      ReplaceElementAt(cm, cell, m.elementIdx, mass, temperature, m.diseaseIdx,
                       m.diseaseCount);
      break;
    case kReplaceAndDisplace:
      ReplaceAndDisplaceElementAt(cm, cell, m.elementIdx, mass, temperature, m.diseaseIdx,
                                  m.diseaseCount);
      break;
    default:
      break;
  }

  // `!= -1`, not `>= 0`:. Every other negative index is pushed and handed back
  // to the game, which is the sort of thing that only matters once.
  if (m.callbackIdx != -1) g->callbacks.push_back(CallbackInfo{m.callbackIdx});
}

void ApplyDig(const Sim::Pending& p) {
  DigMessage m{};
  if (!Payload(p, &m) || !g->world.ValidGameCell(m.cellIdx)) return;
  const size_t cell = g->world.Padded(static_cast<size_t>(m.cellIdx));
  // The projection walks rectangles now, and a message can write a cell no kernel drove.
  g->world.MarkProjectDirty(cell);
  PhaseEntry& e = g->world.Phase(cell);
  if (!g->elements.IsSolid(e.element) || e.mass <= 0.0f) return;
  if (g->world.Properties()[cell] & kUnbreakable) return;

  // Digging hands the cell's mass to the game as ore and empties the cell. The measured
  // behaviour of Klei's sim is that this is exact: 48000 kg dug produced 48000 kg of
  // ore, so anything less here is a bug, not a design choice.
  SpawnOreInfo ore{};
  ore.cellIdx = m.cellIdx;
  ore.elemIdx = e.element;
  ore.mass = e.mass;
  ore.temperature = e.temperature;
  ore.diseaseIdx = g->world.DiseaseIdx()[cell];
  ore.diseaseCount = g->world.Disease()[cell].count;
  if (!m.skipEvent) g->dig_info.push_back(ore);

  // Charged whether or not the event was skipped: `skipEvent` withholds the announcement,
  // not the mass, and the cell is emptied either way.
  g->world.NoteDug(e.mass);
  e = PhaseEntry{};
  g->world.MutableDisease(cell) = SaveDisease{};
  g->world.MutableDiseaseIdx(cell) = 0xFF;
  if (m.callbackIdx >= 0) g->callbacks.push_back(CallbackInfo{m.callbackIdx});
}

// `ProcessMassEmission`. One building putting a fixed bite of matter into one
// cell, with no component and no flood — the electrolyzer and everything like it.
//
// This used to be a twenty-line approximation that refused anything landing on a different
// element and reported the message's own numbers back on the refusal. Four things were wrong
// with that, and the first is the one that matters in play:
//
//   * **Klei displaces what is in the way.** A gas emission into a cell holding a different
//     gas calls `DisplaceGas`, a liquid emission calls `DisplaceLiquid`, and the emission
//     goes ahead if either succeeds. Only a solid in the way, or a displacement with nowhere
//     to go, is a refusal.
//   * A refusal reports **nothing** — element `0xffff`, zero mass, zero temperature — rather
//     than an echo of the request. The `emitted` flag is the only field that carries meaning.
//   * The disease on the message is added to the cell, through the same merge every other
//     germ writer uses.
//   * The temperature mix is `CalculateCombinedTemperature`, so it is clamped to the two
//     inputs; the old open mix could overshoot on a large mass ratio.
//
// And the callback test is `!= -1`, not `>= 0`, as in `ModifyCell`.
//
// Buildings do not each need their own patch or native equivalent: `ElementConsumer`/
// `ElementEmitter` -- the shared
// components GasPump, GasFilter, and virtually every other gas-handling building actually use,
// rather than each building writing `Grid.Mass` itself -- send exactly two SimHashes,
// `MassConsumption` and `MassEmission`, both already handled by exactly these two functions.
// So "many buildings" is two shared native handlers. `ApplyMassEmission`'s half: for a *promoted* cell
// receiving a *gas*, redirect straight into the mixture layer (`gas::InjectSpecies`) instead
// of vanilla's single-element `PhaseEntry`, which the flow kernels' gates already stop treating
// as that room's real reservoir. Liquids fall through unchanged -- the mixture layer is
// atmosphere-only, so a promoted room has no liquid
// layer to redirect into.
inline bool IsPromotedGasCell(const gas::RoomGraph* rooms, size_t cell, const ElementTable& table,
                              uint16_t elementIdx) {
  if (rooms == nullptr || cell >= rooms->room_of.size()) return false;
  if (table.Phase(elementIdx) != kStateGas) return false;
  return gas::IsRoomOwned(*rooms, rooms->room_of[cell]);
}

void ApplyMassEmission(const Sim::Pending& p) {
  MassEmissionMessage m{};
  if (!Payload(p, &m) || !g->world.ValidGameCell(m.cellIdx)) return;
  const size_t cell = g->world.Padded(static_cast<size_t>(m.cellIdx));
  // The projection walks rectangles now, and a message can write a cell no kernel drove.
  g->world.MarkProjectDirty(cell);

  // Promoted-cell redirect, checked first: same `rooms` guard every other gate uses
  // (`vf_active && vf_rooms.room_of.size() == world.PaddedCount()`), computed here rather than
  // reusing the per-substep hoisted one -- this handler runs from the message-drain path, not
  // inside the region loop that pointer is scoped to.
  const gas::RoomGraph* rooms =
      (g->vf_active && g->vf_rooms.room_of.size() == g->world.PaddedCount()) ? &g->vf_rooms
                                                                              : nullptr;
  if (IsPromotedGasCell(rooms, cell, g->elements, m.elementIdx)) {
    // The same landing ModifyCell's redirect uses: the emission's temperature is blended into
    // the cell by heat capacity before the gas joins the mixture, so the energy it brings is
    // the energy that lands; injecting the mass alone would create or destroy the difference
    // whenever the emitter's temperature is not the room's. MixGasIntoPromotedCell also wakes a
    // sleeping cell and adds the germs to the vanilla fields (disease is not redirected).
    MixGasIntoPromotedCell(cell, m.elementIdx, m.mass, m.temperature, m.diseaseIdx,
                           m.diseaseCount);
    // No `NoteEmitted` call: this mass never touched the vanilla ledger, exactly like
    // `ApplyInjectGasSpecies` -- the ledger's "total mass" check sums vanilla PhaseEntry mass
    // only, and mass entering the mixture layer was never part of that total to begin with.
    if (m.callbackIdx != -1) {
      MassEmittedCallback cb{};
      cb.callbackIdx = m.callbackIdx;
      cb.elemIdx = m.elementIdx;
      cb.suceeded = 1;
      cb.diseaseIdx = m.diseaseIdx;
      cb.mass = m.mass;
      cb.temperature = m.temperature;
      cb.diseaseCount = m.diseaseCount;
      g->mass_emitted.push_back(cb);
    }
    return;
  }

  const CellModContext cm{&g->world, &g->elements, &g->diseases, g->displace_rotation};

  PhaseEntry& e = g->world.Phase(cell);
  const uint16_t vacuum = g->elements.VacuumIndex();
  const uint16_t was = e.element;
  bool ok = was == m.elementIdx || was == vacuum;
  if (!ok) {
    const uint8_t phase = g->elements.Phase(m.elementIdx);
    if (phase == kStateGas) ok = DisplaceGasFromDrain(cm, cell, was);
    else if (phase == kStateLiquid) ok = DisplaceLiquid(cm, cell, was);
  }

  if (!ok) {
    if (m.callbackIdx != -1) {
      MassEmittedCallback cb{};
      cb.callbackIdx = m.callbackIdx;
      cb.elemIdx = 0xFFFF;
      cb.suceeded = 0;
      cb.diseaseIdx = 0xFF;
      cb.mass = 0.0f;
      cb.temperature = 0.0f;
      cb.diseaseCount = 0;
      g->mass_emitted.push_back(cb);
    }
    return;
  }

  // Re-read: a displacement moved the cell's contents out from under the reference.
  PhaseEntry& c = g->world.Phase(cell);
  const uint16_t before_element = c.element;
  const float before = c.mass;
  // Energy ledger: measured across the mix AND the element write below, because
  // `CalculateCombinedTemperature` clamps and the element under the mass can change in the
  // same breath -- neither of which the message's own numbers would show.
  const double energy_before = GridCellEnergy(g->world, g->elements, cell);
  c.temperature = CalculateCombinedTemperature(c.mass, c.temperature, m.mass, m.temperature);
  c.mass = before + m.mass;
  g->world.NoteEmitted(c.mass - before);
  if (m.diseaseIdx != 0xFF) {
    AddDiseaseToCell(&g->world, g->diseases, cell, m.diseaseIdx, m.diseaseCount);
  }
  // The element is written and announced only when it actually changes. Klei checks against
  // the element the cell held *before* the displacement, not after.
  if (before_element != m.elementIdx) {
    c.element = m.elementIdx;
    g->world.TouchSubstance(cell);
  }
  g->world.NoteEmittedEnergy(GridCellEnergy(g->world, g->elements, cell) - energy_before);

  if (m.callbackIdx != -1) {
    MassEmittedCallback cb{};
    cb.callbackIdx = m.callbackIdx;
    cb.elemIdx = m.elementIdx;
    cb.suceeded = 1;
    cb.diseaseIdx = m.diseaseIdx;
    cb.mass = m.mass;
    cb.temperature = m.temperature;
    cb.diseaseCount = m.diseaseCount;
    g->mass_emitted.push_back(cb);
  }
}

// `ProcessMassConsumption`. The other half of the same pair, and it is not a
// single-cell operation at all: the message carries a radius and a height, and Klei runs the
// **same drain the element consumer runs** over either a flood or a rectangle.
//
//   * `height == 0` floods outward `radius` steps, through solids if the element is a solid
//     and through everything else if it is not.
//   * `height != 0` walks a `radius x height` rectangle whose top-left corner is the cell.
//
// So the old handler — take from one cell, report the cell's temperature — was wrong about
// the extent, about the disease it hands back, and about the temperature, which is the
// mass-weighted mix of every cell it drained rather than any one cell's.
void ApplyMassConsumption(const Sim::Pending& p) {
  MassConsumptionMessage m{};
  if (!Payload(p, &m) || !g->world.ValidGameCell(m.cellIdx)) return;
  const size_t cell = g->world.Padded(static_cast<size_t>(m.cellIdx));
  const int32_t pw = g->world.PaddedWidth();
  const int32_t x = static_cast<int32_t>(cell % static_cast<size_t>(pw));
  const int32_t y = static_cast<int32_t>(cell / static_cast<size_t>(pw));

  ConsumedMassInfo info{};
  info.simHandle = -1;
  info.removedElemIdx = m.elementIdx;
  info.diseaseIdx = 0xFF;
  info.mass = 0.0f;
  info.temperature = 0.0f;
  info.diseaseCount = 0;

  // This handler services a direct
  // `SimMessages.ConsumeMass` send -- what `GasBreatherFromWorldProvider.ConsumeGas`
  // (breathing) uses -- as opposed to `StepElementConsumers`, the separate native component
  // path GasPump/GasFilter actually tick through every substep (gated above, in `StepPhysics`,
  // with its own locally-hoisted `rooms`). This handler runs from the message-drain path, not
  // that per-region loop, so it computes its own pointer, same as `ApplyMassEmission`.
  // `FloodRemoved`/`RectangularRemoved` share `RemoveMassFromCell`'s promotion gate with
  // `StepElementConsumers` now, so passing a real pointer here is what actually engages it for
  // this call site -- the shared function's own gate does nothing for a caller still passing
  // `nullptr`.
  const gas::RoomGraph* rooms =
      (g->vf_active && g->vf_rooms.room_of.size() == g->world.PaddedCount()) ? &g->vf_rooms
                                                                              : nullptr;
  float remaining = m.mass;
  g->world.PayloadBeginConsume();
  if (m.height == 0) {
    FloodRemoved(&g->world, g->elements, g->diseases, &remaining, m.elementIdx, &info, x, y,
                 m.radius, &g->flow.scratch, rooms);
  } else {
    RectangularRemoved(&g->world, g->elements, g->diseases, &remaining, m.elementIdx, &info,
                       x, y, m.radius, m.height, rooms);
  }
  // Layer C: the payload the removal took, keyed to the callback that will carry the liquid on.
  g->world.PayloadEndConsume(ext::kPayloadConsumerMassConsumption, m.callbackIdx,
                             info.temperature,
                             static_cast<size_t>(y) * static_cast<size_t>(g->world.PaddedWidth()) +
                                 static_cast<size_t>(x));
  g->world.NoteConsumed(info.mass);

  if (m.callbackIdx != -1) {
    MassConsumedCallback cb{};
    cb.callbackIdx = m.callbackIdx;
    cb.elemIdx = info.removedElemIdx;
    cb.diseaseIdx = info.diseaseIdx;
    cb.mass = info.mass;
    cb.temperature = info.temperature;
    cb.diseaseCount = info.diseaseCount;
    g->mass_consumed.push_back(cb);
  }
}

// The two byte-array writes `SimFrameManager::ProcessFrame` does before anything else,
// (insulation) and (strength). Both are `(char)(int)v`:
// truncation toward zero into the low byte, with **no rounding and no clamp** — and only
// the insulation one scales by 255. Strength is stored exactly as it arrives.
//
// This used to round (`* 255.0f + 0.5f`), clamp to [0, 1], and scale strength by 255 as
// well, all three of which are wrong, and a green suite said nothing about any of it
// because **no scenario had ever sent either message** — the insulation coverage the suite
// does have arrives in the world payload instead. Exactly the hole `Cell.properties` sat in.
//
// The strength half is still unobservable: the byte is projected but nothing in
// `GameDataUpdate` publishes it and no kernel of ours reads it, so `setinsul` scores the
// insulation half only and the strength half is untested.
void ApplyCellFloat(const Sim::Pending& p, bool insulation) {
  SetCellFloatValueMessage m{};
  if (!Payload(p, &m) || !g->world.ValidGameCell(m.cellIdx)) return;
  const size_t cell = g->world.Padded(static_cast<size_t>(m.cellIdx));
  const float scaled = insulation ? m.value * 255.0f : m.value;
  const uint8_t byte = static_cast<uint8_t>(static_cast<int32_t>(scaled));
  if (insulation) g->world.MutableInsulation(cell) = byte;
  else g->world.MutableStrength(cell) = byte;
}

// Framework extension (abi/sim_abi_ext.h), not a Klei message. Same payload shape as
// SetInsulationValue/SetStrengthValue, but the value lands unscaled — it is a J/K float,
// not a byte — in the `sim.thermal_mass_bonus` extension property, which physics.h's
// conduction kernel folds into a cell's heat capacity. See sim_abi_ext.h for the contract.
//
// A COMPATIBILITY ALIAS. The storage behind this is a registered property of the extension
// registry; the wire contract is the original one: same id, same payload, same floats
// reaching the same kernel. The property states what a load owes it.
void ApplyThermalMassBonus(const Sim::Pending& p) {
  SetCellFloatValueMessage m{};
  if (!Payload(p, &m) || !g->world.ValidGameCell(m.cellIdx)) return;
  const size_t cell = g->world.Padded(static_cast<size_t>(m.cellIdx));
  uint32_t bits = 0;
  memcpy(&bits, &m.value, sizeof(bits));
  g->world.ExtCells().Write(g->world.ThermalMassBonusProperty(), cell, /*component=*/0, bits);
}

// Framework extension (abi/sim_abi_ext.h): the generic per-cell property write, the one
// message that serves every registered property instead of one message per property.
//
// A bad `propertyIdx` is dropped the same way a bad `cellIdx` is. The index is not guessable
// — it comes back from a registration — so a wrong one is a caller bug rather than hostile
// input, and there is no frame-rate-safe way to shout about it from inside a drain.
void ApplyCellProperty(const Sim::Pending& p) {
  ext::SetCellPropertyMessage m{};
  if (!Payload(p, &m)) return;
  // The ONLY handler-level refusal wired to "sim.message_refused". It earns
  // it because this is the extension ABI's own write path -- a mod writing to a property it
  // never registered, or to a lane outside the arity it declared, is the exact mistake this
  // registry exists to make visible, and it was silent until now. `Write` returns false for
  // an unregistered index, an out-of-range cell and an out-of-range component alike, so one
  // check covers all three. Every OTHER handler's semantic refusal is still silent, and the
  // ABI block says so rather than letting "no record" be read as "applied".
  if (!g->world.ValidGameCell(m.cellIdx)) {
    EmitRefusal(p.id, ext::kExtRefusalBadTarget, static_cast<int32_t>(p.payload.size()), 0);
    return;
  }
  const size_t cell = g->world.Padded(static_cast<size_t>(m.cellIdx));
  if (!g->world.ExtCells().Write(m.propertyIdx, cell, m.component, m.valueBits)) {
    EmitRefusal(p.id, ext::kExtRefusalBadTarget, static_cast<int32_t>(p.payload.size()), 0);
  }
}

// Layer C (abi/sim_abi_ext.h, kAddCellPropertyAmount). Read, add and write on the sim's side
// of the queue, so two adds in flight both land.
void ApplyAddCellPropertyAmount(const Sim::Pending& p) {
  ext::AddCellPropertyAmountMessage m{};
  if (!Payload(p, &m)) return;
  const int32_t got = static_cast<int32_t>(p.payload.size());
  if (!g->world.ValidGameCell(m.cellIdx)) {
    EmitRefusal(p.id, ext::kExtRefusalBadTarget, got, 0);
    return;
  }
  ext::CellPropertyRegistry& reg = g->world.ExtCells();
  const ext::CellPropertyRegistry::Property* prop = reg.At(m.propertyIdx);
  const size_t cell = g->world.Padded(static_cast<size_t>(m.cellIdx));
  uint32_t bits = 0;
  if (prop == nullptr || prop->type != ext::kExtF32 || !std::isfinite(m.amount) ||
      !reg.Read(m.propertyIdx, cell, m.component, &bits)) {
    EmitRefusal(p.id, ext::kExtRefusalBadTarget, got, 0);
    return;
  }
  // A liquid-carried amount exists only where there is liquid to carry it. One added to a gas,
  // solid or vacuum cell would sit there until some liquid arrived and then ride away with it,
  // so it is refused here rather than stranded.
  if (prop->follows_liquid_mass &&
      g->elements.Phase(g->world.Phase(cell).element) != kStateLiquid) {
    EmitRefusal(p.id, ext::kExtRefusalBadTarget, got, 0);
    return;
  }
  float value = 0.0f;
  memcpy(&value, &bits, 4);
  const float next = value + m.amount;
  if (next < 0.0f) {
    EmitRefusal(p.id, ext::kExtRefusalBadTarget, got, 0);
    return;
  }
  memcpy(&bits, &next, 4);
  reg.Write(m.propertyIdx, cell, m.component, bits);
}

// abi/sim_abi_ext.h, kSetFieldSource. Add, replace or remove one source of one
// field. Queued, because it is a parameter of a phase and a parameter that landed mid-substep
// would make the same frame read two different source lists in two different regions.
//
// THE TARGET IS RESOLVED HERE, not in the kernel, and the three kinds resolve to three
// different things: a point source's `target` is a GAME cell and is stored padded, a
// directional source's is a world index and is stored verbatim, and an element source's is a
// SimHashes id and is stored as the element-table index. Resolving at the edge means the
// kernels index arrays with a number that was already checked, and it means a bad target is
// reported to the caller on the frame they sent it rather than being silently skipped every
// substep forever.
//
// A removal (`strength == 0`) still has to name a valid field, but its target is not checked:
// the owner is withdrawing a source, and refusing that because the cell it used to sit in has
// since gone out of range would leave the source in place, which is the opposite of what was
// asked.
void ApplySetFieldSource(const Sim::Pending& p) {
  ext::SetFieldSourceMessage m{};
  if (!Payload(p, &m)) return;
  const int32_t got = static_cast<int32_t>(p.payload.size());
  const bool removing = m.strength == 0.0f;

  ext::FieldSource s;
  s.id = m.sourceId;
  s.kind = m.kind;
  s.strength = m.strength;
  s.radius_x = m.radiusX;
  s.radius_y = m.radiusY;
  s.cone_direction = m.coneDirection;
  s.cone_angle = m.coneAngle;
  s.dir_x = m.dirX;
  s.dir_y = m.dirY;

  if (!removing) {
    switch (m.kind) {
      case ext::kSourcePoint:
        if (!g->world.ValidGameCell(m.target)) {
          EmitRefusal(p.id, ext::kExtRefusalBadTarget, got, 0);
          return;
        }
        s.target = static_cast<int32_t>(g->world.Padded(static_cast<size_t>(m.target)));
        break;
      case ext::kSourceDirectional:
        if (m.target < 0 ||
            static_cast<size_t>(m.target) >= g->world.WorldOffsets().size()) {
          EmitRefusal(p.id, ext::kExtRefusalBadTarget, got, 0);
          return;
        }
        s.target = m.target;
        break;
      case ext::kSourceElement:
        // The hash is resolved to a table index once, here. An element the table does not hold
        // is a refusal and not a silent no-op, because a mod that mistyped a SimHashes constant
        // would otherwise see a source that registers cleanly and never emits -- which is the
        // exact failure `oni-diatomic-molar-mass-fix` cost a correction over.
        if (!g->elements.HasHash(m.target)) {
          EmitRefusal(p.id, ext::kExtRefusalBadTarget, got, 0);
          return;
        }
        s.target = g->elements.IndexOfHash(m.target);
        break;
      default:
        EmitRefusal(p.id, ext::kExtRefusalBadTarget, got, 0);
        return;
    }
  }

  // An unknown field, or a field whose source list is full. Reported rather than dropped: a
  // field index comes back from a registration, so a bad one is a caller bug.
  if (!g->fields.registry.SetSource(m.fieldIdx, s)) {
    EmitRefusal(p.id, ext::kExtRefusalBadTarget, got, 0);
  }
}

// A subscription, not a write: it says "copy this property into every
// published frame from now on", and the copy itself happens in `BuildUpdate`.
//
// Queued like the write above, so it lands one frame later and the first table a caller sees
// after subscribing is the NEXT one. That is not a wart to work around -- it is the same
// rotation every other message in this file lives under, and a caller that binds on the tick
// it subscribed and finds its property missing has not found a bug.
//
// An unknown index is dropped silently. `ApplyCellProperty` no longer is -- stage 4 gave it
// a frame-rate-safe way to shout from inside a drain ("sim.message_refused") -- but a
// subscription is not a write, and a caller that asked to publish a property it never
// registered has a bug its own registration call already reported.
void ApplyPublishCellProperty(const Sim::Pending& p) {
  ext::PublishCellPropertyMessage m{};
  if (!Payload(p, &m)) return;
  g->world.ExtCells().SetPublished(m.propertyIdx, m.enable != 0);
}

// Also a subscription, and a stronger one: an unsubscribed stream does not
// COLLECT, so this switches a producer on rather than only switching a copy on. Queued for the
// same reason as the one above -- it changes what the next frame gathers and publishes, and
// applying it on receipt would change what the frame the game is currently reading contained.
void ApplySubscribeEventStream(const Sim::Pending& p) {
  ext::SubscribeEventStreamMessage m{};
  if (!Payload(p, &m)) return;
  g->ext_events.Subscribe(m.streamIdx, m.enable != 0);
}

// (Re)builds the room graph from whatever Properties()
// holds right now (a later wall built or removed marks `RoomsDirty`, and StepPhysics rebuilds
// the graph then), sizes the per-cell sleep counter,
// and marks the world active. Shared by two callers: ApplyInjectGasSpecies (the first
// message a world ever receives on that id) and the Load handler (a loaded blob that already
// carries gas-mixture state) — kept as one function so the two can never quietly
// diverge in what "activated" means.
void ActivateVolumeFractions() {
  g->vf_rooms = gas::BuildRoomGraph(g->world, g->elements, g->world.GameWidth(),
                                     g->world.GameHeight());
  g->vf_stable.assign(g->world.GameCount(), 0);
  g->vf_tick = 0;
  g->vf_active = true;
}

// Framework extension (abi/sim_abi_ext.h). See kInjectGasSpecies's comment there for the
// full contract. Everything here is a no-op path for a world that never receives this
// message, which is what keeps StepPhysics's mixing branch byte-identical to vanilla until
// it does.
void ApplyInjectGasSpecies(const Sim::Pending& p) {
  ext::InjectGasSpeciesMessage m{};
  if (!Payload(p, &m) || !g->world.ValidGameCell(m.cellIdx)) return;
  if (m.speciesIdx < 0 || m.speciesIdx >= g->elements.Count()) return;
  if (!g->vf_active || g->vf_rooms.room_of.size() != g->world.PaddedCount()) {
    ActivateVolumeFractions();
  }
  const size_t cell = g->world.Padded(static_cast<size_t>(m.cellIdx));
  // The projection is row-dirty driven (World::MarkProjectRows), and a handler that writes
  // the world outside a kernel has to say so or the published arrays keep showing the old
  // value until some unrelated kernel happens to dirty that row. Every vanilla mutating path
  // already does this (cellmod.h, emitters.h, physics.h), and so does this message's own
  // reverse, ApplyConvertToVanillaMass -- without it Grid.Mass and Grid.Temperature would
  // lag the managed side after every injection: the published mass of an injected cell would
  // stay at its pre-injection value for TWO further frames, until physics reached the row. The temperature write below is the part of this
  // handler that needs it; the mixture arrays are not published at all yet.
  g->world.MarkProjectDirty(cell);
  // Blend the incoming mass's temperature into the cell's one shared PhaseEntry.temperature
  // field before adding the mass — see kInjectGasSpecies's own doc comment (abi/sim_abi_ext.h)
  // for why this exists. Existing mass is vanilla PhaseEntry.mass plus whatever the mixture
  // layer already holds here, since both share that one temperature field.
  //
  // Weighted by heat capacity, m*c. A mass weighting (CalculateCombinedTemperature) would
  // create or destroy energy whenever the incoming gas and what is already in the cell have
  // different specific heats.
  if (m.massKg > 0.0f) {
    BlendGasTemperatureIntoCell(cell, static_cast<uint16_t>(m.speciesIdx), m.massKg,
                                m.temperatureK);
  }
  gas::InjectSpecies(g->world, cell, static_cast<uint16_t>(m.speciesIdx), m.massKg);
  // InjectSpecies only ever sets `dirty`, never clears `sleeping` (see gas_mixture.h) — a
  // message landing in an already-asleep room has to wake it explicitly, or the mixing pass
  // keeps skipping that room's interior until some other stimulus reaches it.
  if (g->world.GasSleeping(cell)) {
    g->world.MutableGasSleeping(cell) = 0;
    gas::RoomOnCellWoke(g->vf_rooms, cell);
  }
}

// Framework extension (abi/sim_abi_ext.h). The symmetric counterpart to
// ApplyInjectGasSpecies -- see kRemoveVanillaMass's comment there for why this exists.
// Takes mass directly out of the cell's vanilla PhaseEntry, clamped to what's actually
// there, and books it through the same ledger bucket ApplyMassConsumption uses (`consumed`):
// this is mass leaving the vanilla layer, same as any other consumer, just via a
// single-cell direct write instead of MassConsumption's flood/rectangle. A caller pairing
// this with kInjectGasSpecies for the same massKg on the same cell makes the pair a
// conserving transfer rather than a duplication.
void ApplyRemoveVanillaMass(const Sim::Pending& p) {
  ext::RemoveVanillaMassMessage m{};
  if (!Payload(p, &m) || !g->world.ValidGameCell(m.cellIdx)) return;
  if (!(m.massKg > 0.0f)) return;
  const size_t cell = g->world.Padded(static_cast<size_t>(m.cellIdx));
  PhaseEntry& c = g->world.Phase(cell);
  const float before = c.mass;
  const float removed = (m.massKg < before) ? m.massKg : before;
  if (!(removed > 0.0f)) return;
  // Layer C: THE PAYLOAD RIDES THE MASS, AND THIS HANDLER USED TO LET IT STAY BEHIND.
  //
  // Every other path that takes liquid out of a cell moves or releases the cell's
  // `follows_liquid_mass` properties in the same proportion -- `ApplyAddRemoveSubstance`'s
  // own negative branch does exactly this, four lines of it, at cellmod.h:677. This one did
  // not, so a caller that pumped 97 of a cell's 100 kg away left ALL of the dissolved gas on
  // the 3 kg that remained: a concentration that rises without bound, from a handler whose
  // whole contract is "mass leaves this cell". Found by reading this function
  // while chasing why Mod 1's pump carried no gas.
  //
  // RELEASED, NOT CONSUMED, because this message names no consumer: it is a bare cell write,
  // with no handle and no callback for a record to be keyed by. A caller that wants the
  // payload CARRIED rather than given back to the world sends vanilla's `MassConsumption`
  // with a callback index instead, which routes it (kPayloadConsumerMassConsumption) -- that
  // is what OniFramework's `DissolvedCargo.ConsumeToConduit` does, and why this handler is
  // now the fallback rather than the way. `kPayloadReleasedRemoved` is the same reason code
  // the negative-AddRemoveSubstance path uses, because it is the same event.
  //
  // Before the subtraction, because the fraction is over the mass that was holding it.
  g->world.PayloadRelease(cell, removed < before ? removed / before : 1.0f, c.temperature,
                          ext::kPayloadReleasedRemoved);
  c.mass -= removed;
  // Same reason as ApplyInjectGasSpecies's own MarkProjectDirty above, and the same
  // measurement: without it the published mass of the cell this just took matter out of
  // stays at its old value until an unrelated kernel dirties the row.
  g->world.MarkProjectDirty(cell);
  g->world.NoteConsumed(removed);
  // Energy ledger: mass only, at the cell's own temperature and element -- same shape as
  // RemoveMassFromCell's vanilla branch, which is the bucket this shares.
  g->world.NoteConsumedEnergy(-static_cast<double>(removed) *
                              static_cast<double>(
                                  g->elements.At(c.element).specificHeatCapacity) *
                              static_cast<double>(c.temperature));
}

// Framework extension (abi/sim_abi_ext.h). The reverse of kInjectGasSpecies+
// kRemoveVanillaMass, done as one atomic operation rather than two messages -- see
// kConvertToVanillaMass's own comment for why that matters. Removes from the mixture first,
// then only ever adds however much actually came out, so this can't create mass regardless of
// what the caller asked for.
void ApplyConvertToVanillaMass(const Sim::Pending& p) {
  ext::ConvertToVanillaMassMessage m{};
  if (!Payload(p, &m) || !g->world.ValidGameCell(m.cellIdx)) return;
  if (m.speciesIdx < 0 || m.speciesIdx >= g->elements.Count()) return;
  if (!(m.massKg > 0.0f)) return;
  const size_t cell = g->world.Padded(static_cast<size_t>(m.cellIdx));

  const float removed =
      gas::RemoveSpecies(g->world, cell, static_cast<uint16_t>(m.speciesIdx), m.massKg);
  if (!(removed > 0.0f)) return;

  // Same accept/displace contract ApplyMassEmission uses for every other building that puts
  // matter into a cell (match, vacuum, or DisplaceGasFromDrain) -- reused, not reinvented.
  g->world.MarkProjectDirty(cell);
  const CellModContext cm{&g->world, &g->elements, &g->diseases, g->displace_rotation};
  PhaseEntry& c = g->world.Phase(cell);
  const uint16_t vacuum = g->elements.VacuumIndex();
  const uint16_t was = c.element;
  bool ok = was == static_cast<uint16_t>(m.speciesIdx) || was == vacuum;
  if (!ok && g->elements.Phase(static_cast<uint16_t>(m.speciesIdx)) == kStateGas) {
    ok = DisplaceGasFromDrain(cm, cell, was);
  }
  if (!ok) {
    // Refused -- e.g. a solid in the way with nowhere to displace into. Refund the mixture
    // side so this stays conserving even on refusal, not just on success.
    gas::InjectSpecies(g->world, cell, static_cast<uint16_t>(m.speciesIdx), removed);
    return;
  }

  const double energy_before = GridCellEnergy(g->world, g->elements, cell);
  c.temperature = CalculateCombinedTemperature(c.mass, c.temperature, removed, m.temperatureK);
  c.element = static_cast<uint16_t>(m.speciesIdx);
  c.mass += removed;
  g->world.NoteEmitted(removed);
  g->world.NoteEmittedEnergy(GridCellEnergy(g->world, g->elements, cell) - energy_before);
}

// Framework extension (abi/sim_abi_ext.h). See kPromoteRoom's comment there for the full
// contract. Ensures volume-fractions is active (same lazy-activation as ApplyInjectGasSpecies)
// so a promotion sent standalone still resolves to a real room graph, then flips
// `RoomGraph.owned` for whichever room `cellIdx` currently belongs to. No kernel reads that
// flag yet, so this is inert for any world that never sends it -- same "byte-identical until
// sent" guarantee as every other message in this file.
void ApplyPromoteRoom(const Sim::Pending& p) {
  ext::PromoteRoomMessage m{};
  if (!Payload(p, &m) || !g->world.ValidGameCell(m.cellIdx)) return;
  if (!g->vf_active || g->vf_rooms.room_of.size() != g->world.PaddedCount()) {
    ActivateVolumeFractions();
  }
  const size_t cell = g->world.Padded(static_cast<size_t>(m.cellIdx));
  const int32_t room = g->vf_rooms.room_of[cell];
  gas::SetRoomOwned(g->vf_rooms, room, true);
  // Persist promotion per cell, not just on this RoomGraph
  // instance's `owned` array -- a geometry change later rebuilds `g->vf_rooms` from scratch
  // (`World::RoomsDirty`, StepPhysics), and BuildRoomGraph re-derives `owned` from exactly
  // this per-cell record (gas_rooms.h). Without this, promoting a room and then digging
  // anywhere else in the world would silently un-promote it the next time the graph
  // rebuilds -- the room id it used to have is not guaranteed to mean anything afterward.
  // O(grid) scan, same cost class as BuildRoomGraph itself; promotion is a rare player/debug
  // action, not a per-tick one, so this is not a hot path.
  if (room >= 0) {
    for (size_t p = 0; p < g->vf_rooms.room_of.size(); ++p) {
      if (g->vf_rooms.room_of[p] == room) g->world.MutableRoomPromoted(p) = 1;
    }
  }
}

// Framework extension (abi/sim_abi_ext.h). See kSetBlockedGasAddPolicy's comment there. An
// unknown policy is refused rather than clamped: a caller asking for a behaviour this build
// does not have must not silently get a different one.
// Framework extension (abi/sim_abi_ext.h). See kSetCellPropertyTransport's comment there.
void ApplySetCellPropertyTransport(const Sim::Pending& p) {
  ext::SetCellPropertyTransportMessage m{};
  if (!Payload(p, &m)) return;
  if (m.transport != ext::kTransportStatic && m.transport != ext::kTransportFollowsLiquidMass) {
    Report("kSetCellPropertyTransport: unknown transport");
    return;
  }
  if (!g->world.ExtCells().SetFollowsLiquidMass(
          m.propertyIdx, m.transport == ext::kTransportFollowsLiquidMass)) {
    EmitRefusal(ext::kSetCellPropertyTransport, ext::kExtRefusalBadTarget,
                static_cast<int32_t>(p.payload.size()), 0);
    return;
  }
  g->world.RefreshLiquidPayload();
}

// Framework extension (abi/sim_abi_ext.h). See kSetPayloadMixing's comment there. `share` is
// CLAMPED rather than refused, in `SetMixShare`: above 1 a pair would overshoot its own
// equilibrium and ring, and a caller asking for "as fast as you can" means 1. A bad
// `propertyIdx` IS refused, because that is a caller naming something that does not exist
// rather than a caller asking for too much of something that does.
void ApplySetPayloadMixing(const Sim::Pending& p) {
  ext::SetPayloadMixingMessage m{};
  if (!Payload(p, &m)) return;
  ext::CellPropertyRegistry& reg = g->world.ExtCells();
  if (m.propertyIdx == ext::kPayloadMixingAllProperties) {
    for (int32_t i = 0; i < reg.Count(); ++i) {
      const ext::CellPropertyRegistry::Property* q = reg.At(i);
      // Only the properties the kernel would ever look at. Setting a rate on a property that
      // does not follow liquid would be a number that changes nothing, stored where a later
      // `kSetCellPropertyTransport` could silently bring it to life.
      if (q != nullptr && q->follows_liquid_mass) reg.SetMixShare(i, m.share);
    }
  } else if (!reg.SetMixShare(m.propertyIdx, m.share)) {
    EmitRefusal(ext::kSetPayloadMixing, ext::kExtRefusalBadTarget,
                static_cast<int32_t>(p.payload.size()), 0);
    return;
  }
  // The hoisted list carries a copy of the rate -- see `World::PayloadProperty`.
  g->world.RefreshLiquidPayload();
}

// Framework extension (abi/sim_abi_ext.h). See kSetDissolvedTint's comment there.
//
// Nothing here can be refused: every field is either a flag, a clamped scalar or per-lane
// content data, and a caller who sends nonsense gets a tint that looks wrong rather than a world
// that behaves wrong. The colours are unpacked once, here, so the per-cell path in
// `FillPropertyTextures` never touches a packed word.
void ApplySetDissolvedTint(const Sim::Pending& p) {
  ext::SetDissolvedTintMessage m{};
  if (!Payload(p, &m)) return;
  World::DissolvedTint& t = g->world.MutableDissolvedTintConfig();
  t.enabled = m.enabled != 0;
  t.full_scale_g_per_kg = m.fullScaleGramsPerKg > 0.0f ? m.fullScaleGramsPerKg
                                                       : ext::kDissolvedTintDefaultFullScale;
  float blend = m.maxBlend;
  if (!(blend > 0.0f)) blend = 0.0f;
  if (blend > 1.0f) blend = 1.0f;
  t.max_blend = blend;
  for (int i = 0; i < ext::kDissolvedGasLanes; ++i) {
    const float wgt = m.laneWeight[i];
    t.lane_weight[i] = wgt > 0.0f ? wgt : 0.0f;
    const uint32_t c = m.laneColour[i];
    t.lane_r[i] = static_cast<float>((c >> 16) & 0xFFu) * (1.0f / 255.0f);
    t.lane_g[i] = static_cast<float>((c >> 8) & 0xFFu) * (1.0f / 255.0f);
    t.lane_b[i] = static_cast<float>(c & 0xFFu) * (1.0f / 255.0f);
  }
  // The liquid texture caches on "the element did not change", and a liquid cell is exempt from
  // that cache already, so the next sweep repaints every liquid cell with the new tint without
  // anything else being told. A non-liquid cell is unaffected by this message in any case.
}

// Framework extension (abi/sim_abi_ext.h). See kSetEffervescence's comment there.
//
// Refuses nothing, for the tint's reason: every field is a flag, a clamped scalar or per-lane
// content, and a caller who sends nonsense gets water that fizzes wrongly rather than a world that
// breaks. The one guard with teeth is the solvent count, which is clamped so a bad count cannot
// read past the arrays.
void ApplySetEffervescence(const Sim::Pending& p) {
  ext::SetEffervescenceMessage m{};
  if (!Payload(p, &m)) return;
  World::Effervescence& e = g->world.MutableEffervescenceConfig();
  e.enabled = m.enabled != 0;
  e.margin = m.margin > 0.0f ? m.margin : 0.0f;
  e.rate_per_second = m.ratePerSecond > 0.0f ? m.ratePerSecond : 0.0f;
  e.period_seconds = m.periodSeconds > 0.0f ? m.periodSeconds
                                            : ext::kEffervescenceDefaultPeriodSeconds;
  e.min_release_kg = m.minReleaseKg > 0.0f ? m.minReleaseKg : 0.0f;
  for (int i = 0; i < ext::kDissolvedGasLanes; ++i) {
    e.lane_henry[i] = m.laneHenryMolPerM3Pa[i] > 0.0f ? m.laneHenryMolPerM3Pa[i] : 0.0f;
    e.lane_vant_hoff_k[i] = std::isfinite(m.laneVantHoffK[i]) ? m.laneVantHoffK[i] : 0.0f;
    e.lane_molar_kg[i] = m.laneMolarMassKgPerMol[i] > 0.0f ? m.laneMolarMassKgPerMol[i] : 0.0f;
    e.lane_pmv_m3[i] =
        m.lanePartialMolarVolumeM3PerMol[i] > 0.0f ? m.lanePartialMolarVolumeM3PerMol[i] : 0.0f;
  }
  int32_t n = m.solventCount;
  if (n < 0) n = 0;
  if (n > ext::kEffervescenceMaxSolvents) n = ext::kEffervescenceMaxSolvents;
  e.solvent_count = 0;
  for (int32_t s = 0; s < n; ++s) {
    const int32_t idx = m.solventElementIdx[s];
    if (idx < 0 || idx > 0xFFFF) continue;
    e.solvent_element[e.solvent_count] = static_cast<uint16_t>(idx);
    e.solvent_factor[e.solvent_count] = m.solventFactor[s] > 0.0f ? m.solventFactor[s] : 0.0f;
    e.solvent_density[e.solvent_count] =
        m.solventDensityKgPerM3[s] > 0.0f ? m.solventDensityKgPerM3[s] : 0.0f;
    ++e.solvent_count;
  }
  // The surface exchange, if the caller sent the long form; the short form switches it off.
  e.surface_rate_per_second = 0.0f;
  e.surface_min_release_kg = ext::kEffervescenceDefaultSurfaceMinReleaseKg;
  for (int i = 0; i < ext::kDissolvedGasLanes; ++i) e.lane_element[i] = 0xFFFF;
  if (p.payload.size() >= sizeof(ext::SetEffervescenceMessageV2)) {
    ext::SetEffervescenceMessageV2 v{};
    memcpy(&v, p.payload.data(), sizeof(v));
    e.surface_rate_per_second = v.surfaceRatePerSecond > 0.0f ? v.surfaceRatePerSecond : 0.0f;
    e.surface_min_release_kg = v.surfaceMinReleaseKg > 0.0f ? v.surfaceMinReleaseKg : 0.0f;
    for (int i = 0; i < ext::kDissolvedGasLanes; ++i) {
      const int32_t idx = v.laneElementIdx[i];
      e.lane_element[i] = idx >= 0 && idx < 0xFFFF ? static_cast<uint16_t>(idx) : uint16_t{0xFFFF};
    }
  }
  // A re-send restarts the period, so a caller that switches the pass on sees its first burst a
  // full period later, the same as a load.
  e.elapsed_seconds = 0.0f;
}

void ApplySetBlockedGasAddPolicy(const Sim::Pending& p) {
  ext::SetBlockedGasAddPolicyMessage m{};
  if (!Payload(p, &m)) return;
  if (m.policy != ext::kBlockedGasAddVanilla && m.policy != ext::kBlockedGasAddPromoteAndMix) {
    Report("kSetBlockedGasAddPolicy: unknown policy");
    return;
  }
  g->blocked_gas_add_policy = m.policy;
}

// Framework extension (abi/sim_abi_ext.h). See kSetInvertedGravityElement's comment there.
// -1 maps to World's own "no element" sentinel (0xFFFF); any other value is stored as-is,
// with no range check against the real element table -- an out-of-range index simply never
// matches any `elem` StepFlow compares it against (physics.h), so it is inert rather than
// unsafe. Only one element can be inverted at a time; the new value replaces the old one.
void ApplySetInvertedGravityElement(const Sim::Pending& p) {
  ext::SetInvertedGravityElementMessage m{};
  if (!Payload(p, &m)) return;
  g->world.SetInvertedGravityElement(
      m.elementIdx < 0 ? uint16_t{0xFFFF} : static_cast<uint16_t>(m.elementIdx));
}

// Framework extension (abi/sim_abi_ext.h). Re-points the random stream, which is the one
// input to a run that `AllocateCells` deliberately takes from the wall clock. Written raw:
// `SetRandomState` does no scrambling, so a value read back from `SIM_DebugRandomState`
// round-trips exactly. Every uint32 is a legal state, so there is nothing to validate and
// nothing to reject.
void ApplySetRandomState(const Sim::Pending& p) {
  ext::SetRandomStateMessage m{};
  if (!Payload(p, &m)) return;
  g->world.SetRandomState(m.state);
}

// Framework extension (abi/sim_abi_ext.h). The other half of "a save blob does not carry
// everything a run needs to continue": the SimData scheduling counters. Both AllocateCells
// and Load reset these to their fresh-allocation values, so a restored world runs its first
// physics frame in a different order than the run it was captured from -- measured as 26 of
// 35 replayed ticks diverging with the random stream still in step.
//
// Applied verbatim, like SetRandomState: these are states, not seeds, and every value a
// getter can return is a legal one to send back. The one guard is on vf_active, which is
// not self-contained -- turning the volume-fraction pass on without a room graph sized for
// this world would run it against a stale one -- so the rebuild is requested rather than
// assumed, and StepPhysics does it at the top of the next frame it runs.
void ApplySetSchedulingState(const Sim::Pending& p) {
  ext::SetSchedulingStateMessage m{};
  if (!Payload(p, &m)) return;
  g->skip_physics_frames = m.skip_physics_frames;
  g->pressure_dir = m.pressure_dir;
  g->substep_carry = m.substep_carry;
  g->displace_rotation = m.displace_rotation;
  g->first_physics_substep = m.first_physics_substep != 0;
  g->world.SetShuffleDir(m.shuffle_dir);
  const bool want_vf = m.vf_active != 0;
  if (want_vf && !g->vf_active) g->world.MutableRoomsDirty() = true;
  g->vf_active = want_vf;
}

// Framework extension (abi/sim_abi_ext.h). The third thing a Load throws away, after the
// random stream and the scheduling counters: World::stable_ticks_, one byte a cell, holding
// how long each unstable solid has left before it falls. No save format field holds it and
// every Allocate re-assigns the array to the all-0x1f reroll sentinel, so a restored world
// rerolls every unstable solid it reaches -- which drops sand on the wrong tick AND moves
// the random stream, since that reroll is the only draw the unstable path makes.
//
// The only variable-length message in the extension set, so both length checks are real:
// the payload has to agree with its own count, and the count has to agree with this world's
// padded cell count. A mismatch is logged and dropped rather than applied to the part that
// fits -- the tail would stay at the sentinel, which is the state the caller is trying to
// leave, and a half-restored run looks restored without being it.
void ApplySetStableTicks(const Sim::Pending& p) {
  ext::SetStableTicksMessage m{};
  if (!Payload(p, &m)) return;
  if (m.count <= 0) {
    Report("kSetStableTicks: non-positive count");
    return;
  }
  const size_t count = static_cast<size_t>(m.count);
  if (p.payload.size() != sizeof(m) + count) {
    Report("kSetStableTicks: payload length does not match its own count");
    return;
  }
  if (!g->world.SetStableTicks(p.payload.data() + sizeof(m), count)) {
    Report("kSetStableTicks: count is not this world's padded cell count");
  }
}

// Framework extension (abi/sim_abi_ext.h). The fifth checkpoint component: both per-cell
// disease growth arrays, in one payload. (This comment used to be the waste-heat handler's,
// copied down with the function below it and never corrected.)
void ApplySetDiseaseGrowth(const Sim::Pending& p) {
  ext::SetDiseaseGrowthMessage m{};
  if (!Payload(p, &m)) return;
  if (m.count <= 0) {
    Report("kSetDiseaseGrowth: non-positive count");
    return;
  }
  const size_t count = static_cast<size_t>(m.count);
  // Both arrays, in one payload, checked against their own header before a byte is read.
  if (p.payload.size() != sizeof(m) + count * sizeof(float) + count) {
    Report("kSetDiseaseGrowth: payload length does not match its own count");
    return;
  }
  const uint8_t* accum = p.payload.data() + sizeof(m);
  const uint8_t* infest = accum + count * sizeof(float);
  if (!g->world.SetDiseaseGrowth(reinterpret_cast<const float*>(accum), infest, count)) {
    Report("kSetDiseaseGrowth: count is not this world's padded cell count");
  }
}

// Framework extension (abi/sim_abi_ext.h). The sixth checkpoint component: all four component
// registries, as the opaque blob sim/registry_state.h produces. Sixth of SEVEN since stage 2b
// -- ApplySetExtCellState below is the seventh. See that header for what
// travels, what is deliberately left out, and why re-sending the original registration
// messages is not the same thing.
//
// The payload IS the blob -- there is no header struct in front of it -- and registry_state
// checks its own magic, version, per-element sizes and total length before it moves anything
// into place, so a rejection here leaves the registries exactly as they were.
void ApplySetRegistryState(const Sim::Pending& p) {
  if (p.payload.empty()) {
    Report("kSetRegistryState: empty payload");
    return;
  }
  if (!registry_state::Load(p.payload.data(), p.payload.size(), &g->buildings, &g->chunks,
                             &g->flow, &g->radiation)) {
    Report("kSetRegistryState: blob rejected, registries left unchanged");
  }
}

// Framework extension (abi/sim_abi_ext.h). The SEVENTH checkpoint component: the per-cell
// extension properties a save blob does not carry -- `kRehydrated` and `kCheckpointOnly`. See
// that header's kSetExtCellState block for the complete checkpoint list, which lives there and
// only there, and `sim/ext_state.h` for the format and the whole-or-nothing rule.
//
// The payload IS the blob. A record naming a property this build did not register is reported
// and skipped rather than fatal, so a caller that dropped a mod still gets its checkpoint back;
// anything else -- a bad magic, an unknown version, a shape that disagrees with what this build
// registered -- rejects the whole blob and leaves the registry exactly as it was.
void ApplySetExtCellState(const Sim::Pending& p) {
  if (p.payload.empty()) {
    Report("kSetExtCellState: empty payload");
    return;
  }
  std::string error;
  std::vector<std::string> reported;
  if (!g->world.ExtCheckpointFromBlob(p.payload.data(), p.payload.size(), &error, &reported)) {
    Report(("kSetExtCellState: " + error + "; extension properties left unchanged").c_str());
    return;
  }
  // Skipped, not silent. A caller that never hears these cannot tell "the checkpoint had
  // nothing extra" from "I forgot to load the mod that owns it".
  for (const std::string& name : reported) {
    Report(("kSetExtCellState: blob carries extension property \"" + name +
            "\", which no loaded mod registered; that record was skipped").c_str());
  }
}

void ApplySetBuildingWasteHeat(const Sim::Pending& p) {
  ext::SetBuildingWasteHeatKilowattsMessage m{};
  if (!Payload(p, &m)) return;
  if (BuildingHeatExchangeData* d = g->buildings.exchange.Get(m.handle)) {
    d->waste_heat_kilowatts = m.kilowatts;
  }
}

// Framework extension (abi/sim_abi_ext.h). Klei's `ExhaustKilowattsWhenActive`, as a rate the
// sim owns and integrates over its own substeps. Same stale-handle policy as above.
void ApplySetBuildingExhaust(const Sim::Pending& p) {
  ext::SetBuildingExhaustMessage m{};
  if (!Payload(p, &m)) return;
  if (BuildingHeatExchangeData* d = g->buildings.exchange.Get(m.handle)) {
    d->exhaust_kilowatts = m.kilowatts;
    d->exhaust_max_temperature = m.maxTemperature;
  }
}

// Framework extension (abi/sim_abi_ext.h). Emissivity and radiating area for one building.
void ApplySetBuildingRadiation(const Sim::Pending& p) {
  ext::SetBuildingRadiationMessage m{};
  if (!Payload(p, &m)) return;
  if (BuildingHeatExchangeData* d = g->buildings.exchange.Get(m.handle)) {
    d->radiation_factor = m.radiationFactor;
    d->radiation_area_m2 = m.surfaceAreaM2;
  }
}

// Framework extension (abi/sim_abi_ext.h). Stationeers-shaped convection between one building's
// body and its footprint cells. The conduit-side half of this message is recorded at SEND time in
// `SIM_HandleMessage`, because the conduit kernel runs on the game thread; this half is the
// building field, applied by the worker like any other queued parameter.
void ApplySetBuildingConvection(const Sim::Pending& p) {
  ext::SetBuildingConvectionMessage m{};
  if (!Payload(p, &m)) return;
  if (BuildingHeatExchangeData* d = g->buildings.exchange.Get(m.handle)) {
    d->convection_factor = m.convectionFactor;
    d->convection_area_m2 = m.surfaceAreaM2;
    d->convection_reach = m.reachCells;
  }
}

// Framework extension (abi/sim_abi_ext.h). The temperature of the reservoir radiating bodies
// shed into -- this sim's stand-in for a planetary atmosphere, unless a world environment
// supplies one.
void ApplySetEnvironmentTemperature(const Sim::Pending& p) {
  ext::SetEnvironmentTemperatureMessage m{};
  if (!Payload(p, &m)) return;
  g->world.SetEnvironmentTemperature(m.kelvin > 0.0f ? m.kelvin : 0.0f);
}

// Framework extension (abi/sim_abi_ext.h). The per-world version of the above, plus the two
// planetary inputs no vanilla message carries: how much sun a world gets, and what its surface
// atmosphere is held at. Flagship 2 / Mod 3.
void ApplySetWorldEnvironment(const Sim::Pending& p) {
  ext::SetWorldEnvironmentMessage m{};
  if (!Payload(p, &m)) return;
  World::Environment e{};
  e.sink_kelvin = m.sinkKelvin > 0.0f ? m.sinkKelvin : 0.0f;
  e.peak_irradiance_w_m2 = m.peakIrradianceWPerM2 > 0.0f ? m.peakIrradianceWPerM2 : 0.0f;
  e.full_sun_lux = m.fullSunLux > 0.0f ? m.fullSunLux : 0.0f;
  // Absorptivity is a fraction and a caller asking for more than one is asking for a cell that
  // absorbs more than arrives. Clamped rather than refused: the term stays physical and the
  // caller's intent (as much as possible) is still honoured.
  e.solar_absorptivity = m.solarAbsorptivity <= 0.0f
                             ? 0.0f
                             : (m.solarAbsorptivity > 1.0f ? 1.0f : m.solarAbsorptivity);
  e.surface_pressure_kpa = m.surfacePressureKPa > 0.0f ? m.surfacePressureKPa : 0.0f;
  e.boundary_temperature_k = m.boundaryTemperatureK > 0.0f ? m.boundaryTemperatureK : 0.0f;
  e.boundary_rate = m.boundaryRate <= 0.0f
                        ? 0.0f
                        : (m.boundaryRate > g_tunables.max_world_boundary_rate
                               ? g_tunables.max_world_boundary_rate
                               : m.boundaryRate);
  // An element index the table does not hold would index off the end of it every substep, so it
  // is refused here rather than guarded in the kernel.
  e.boundary_element =
      (m.boundaryElement >= 0 && m.boundaryElement < g->elements.Count()) ? m.boundaryElement
                                                                            : -1;
  g->world.SetWorldEnvironment(m.worldIndex, e);
}

// Framework extension (abi/sim_abi_ext.h). The direction a world's sun shines from, which is
// what switches its direct beam on. Flagship 2 / Mod 3's sun path.
void ApplySetWorldSun(const Sim::Pending& p) {
  ext::SetWorldSunMessage m{};
  if (!Payload(p, &m)) return;
  if (m.enabled == 0) {
    g->world.ClearWorldSun(m.worldIndex);
    return;
  }
  // A direction with no length, or one that is not finite, has nowhere to shine from, and would
  // put a NaN lane offset into the beam walk. Taken as "no sun" here rather than guarded in the
  // kernel, which runs every frame.
  const float len2 = m.dirX * m.dirX + m.dirY * m.dirY;
  if (!(len2 > 0.0f) || !std::isfinite(len2)) {
    g->world.ClearWorldSun(m.worldIndex);
    return;
  }
  const float inv = 1.0f / std::sqrt(len2);
  World::SunDirection s;
  s.dir_x = m.dirX * inv;
  s.dir_y = m.dirY * inv;
  g->world.SetWorldSun(m.worldIndex, s);
}

// `ProcessCellEnergyModifications`. The third busiest message in the live
// census and the only one of the top five that is not a component: it is how everything
// game-side that heats or cools a cell without moving mass reaches the grid — a running
// machine's waste heat, a duplicant's body heat, a radiant pipe.
//
// Four gates, and the two in the middle are the ones a guess gets wrong:
//
//   * the cell's element must have a **phase**, `state & 3`. A Vacuum cell is refused
//     before its mass is even read, so energy poured into empty space is discarded rather
//     than accumulated.
//   * `mass > 0.001f`, strictly, against the same 1 g floor the rest of the sim uses.
//   * `maxTemperature > 0.0f`. A message with no ceiling does **nothing at all** — the
//     ceiling is not optional and zero is not "unlimited".
//
// The ceiling itself is `max(temperature, maxTemperature)`, not `maxTemperature`: a cell
// already hotter than the ceiling is not cooled back down to it, it is simply left where
// it is. So the message can only ever push a cell *towards* the ceiling from below, and
// negative kilojoules cool without limit.
//
// Klei then clamps a result outside `(0, 10000]` into `[1, 10000]` and shouts on stderr,
// and skips the write entirely if the result is not positive.
void ApplyCellEnergy(const Sim::Pending& p) {
  ModifyCellEnergyMessage m{};
  if (!Payload(p, &m)) return;
  // Energy ledger. Past this point the message is a real request for `m.kilojoules` of heat
  // at a real cell, and every branch below that returns without landing them has destroyed
  // them. Charged on each branch separately rather than once at the bottom so the bucket
  // stays attached to the reason -- this is the site behind `ExhaustHeat`, which is the
  // widest heat-deletion channel in vanilla, and "how much" is useless without "where".
  if (!g->world.ValidGameCell(m.cellIdx)) {
    g->world.NoteCellEnergyRefused(m.kilojoules);
    return;
  }
  const size_t cell = g->world.Padded(static_cast<size_t>(m.cellIdx));
  PhaseEntry& c = g->world.Phase(cell);
  const Element& e = g->elements.At(c.element);
  // The arithmetic lives in buildings.h's DeliverCellEnergy, shared with the building exhaust
  // path (abi/sim_abi_ext.h's kSetBuildingExhaust) so the two cannot drift apart -- vanilla
  // exhaust reaches the cells through this very message, and a fix that reproduced the
  // delivery rules approximately would be worse than no fix at all. It refuses on three
  // paths: a vacuum cell (element state 0), a cell under 0.001 kg, and the maxTemperature
  // ceiling -- which is NOT 10000 K for most buildings but their overheat temperature.
  //
  // `landed` is measured over the temperature write alone. Deliberately not measured across
  // the `DoStateTransition` below: a transition swaps the element under the mass, and whatever
  // that does to the cell's energy belongs to the phase-change site, not this one.
  //
  // With `CellEnergyCarry` on (tunables.def) the cell's rounding remainder rides along: what
  // the write did not land is held for the cell's next payment and booked as held, not refused.
  double* carry = g_tunables.cell_energy_carry != 0 ? g->world.CellEnergyCarry(cell) : nullptr;
  const double carried_before = carry ? *carry : 0.0;
  double landed = 0.0;
  const bool reached_write = DeliverCellEnergy(&c, e, m.kilojoules, m.maxTemperature, &landed,
                                               g_tunables, carry);
  const double carried_delta = carry ? *carry - carried_before : 0.0;
  g->world.NoteCellEnergyCarry(carried_delta);
  g->world.NoteCellEnergyMsg(landed);
  g->world.NoteCellEnergyRefused(static_cast<double>(m.kilojoules) - landed - carried_delta);
  if (!reached_write) return;
  // A message can write a cell no kernel drove, and `Project` walks rectangles.
  g->world.MarkProjectDirty(cell);
  // Klei calls `DoStateTransition` on the cell here, in the drain, before any kernel of
  // this frame has looked at it — so a cell this message boils is already steam when the
  // first substep runs.
  const int32_t pw = g->world.PaddedWidth();
  const int32_t y = static_cast<int32_t>(cell / static_cast<size_t>(pw));
  const int32_t x = static_cast<int32_t>(cell - static_cast<size_t>(y) * pw);
  if (TransitionCell(&g->world, g->elements, x, y, &g->state_change_ores, PhaseRules{},
                     nullptr, FrameVisible(), g->debug_editing, &g->falling_liquid)) {
    g->world.TouchSubstance(cell);
  }
}

void ApplyCellProperties(const Sim::Pending& p) {
  CellPropertiesMessage m{};
  if (!Payload(p, &m) || !g->world.ValidGameCell(m.cellIdx)) return;
  const size_t cell = g->world.Padded(static_cast<size_t>(m.cellIdx));
  if (m.set) g->world.MutableProperties(cell) |= m.properties;
  else g->world.MutableProperties(cell) &= static_cast<uint8_t>(~m.properties);
  // This is the real vanilla message digging and building both
  // send (kGasImpermeable, gas_rooms.h's IsOpenCell), so it is one of the two real choke
  // points for a room-graph-affecting geometry change (the other is TransitionCell,
  // physics.h, for natural phase transitions). Marking dirty unconditionally on any
  // kGasImpermeable touch, even a same-state one, is deliberate -- this message is rare next
  // to a mixing tick, so a spurious rebuild once in a while costs nothing, and checking
  // "did the bit actually flip" would need reading MutableProperties before the write above,
  // which is more code for a saving that doesn't matter here.
  if (m.properties & kGasImpermeable) g->world.MutableRoomsDirty() = true;
  if (m.callbackIdx >= 0) g->callbacks.push_back(CallbackInfo{m.callbackIdx});
}

void ApplyDiseaseModification(const Sim::Pending& p) {
  CellDiseaseModification m{};
  if (!Payload(p, &m) || !g->world.ValidGameCell(m.cellIdx)) return;
  const size_t cell = g->world.Padded(static_cast<size_t>(m.cellIdx));
  if (m.diseaseIdx != 0xFF) {
    g->world.MutableDiseaseIdx(cell) = m.diseaseIdx;
    g->world.MutableDisease(cell).diseaseHash = g->diseases.HashOf(m.diseaseIdx);
  }
  int64_t count = g->world.Disease()[cell].count + static_cast<int64_t>(m.diseaseCount);
  if (count < 0) count = 0;
  g->world.MutableDisease(cell).count = static_cast<int32_t>(count);
  if (count == 0) {
    g->world.MutableDiseaseIdx(cell) = 0xFF;
    g->world.MutableDisease(cell).diseaseHash = 0;
  }
}

// `ProcessConsumeDisease`. The disease *consumer*, and it is not a component
// update at all — `DiseaseConsumer` has only `Register` and `Unregister`, and no entry in
// `SimData`'s component list. The consuming is done by a per-frame message, drained by
// `ProcessFrame` immediately **before** the inline disease-modification loop.
//
// Per record: take `percentToConsume` of the cell's germs, rounded with a **+0.5f** before
// the truncation, capped at `maxToConsume`; subtract it; and if that leaves the cell below
// one germ, clear all four disease fields the way `ClearCell` does.
//
// The callback carries the disease index the cell has **afterwards** — 0xFF when the cell was
// empty to begin with, and 0xFF again when this call emptied it — together with the amount
// actually taken, which is 0 in both of those cases.
void ApplyConsumeDisease(const Sim::Pending& p) {
  ConsumeDiseaseMessage m{};
  if (!Payload(p, &m) || !g->world.ValidGameCell(m.gameCell)) return;
  const size_t cell = g->world.Padded(static_cast<size_t>(m.gameCell));
  uint8_t reported_idx = 0xFF;
  int32_t taken = 0;
  if (g->world.DiseaseIdx(cell) != 0xFF) {
    const int32_t count = g->world.Disease()[cell].count;
    taken = static_cast<int32_t>(static_cast<float>(count) * m.percentToConsume + 0.5f);
    if (taken > m.maxToConsume) taken = m.maxToConsume;
    g->world.MutableDisease(cell).count = count - taken;
    if (g->world.Disease()[cell].count < 1) {
      g->world.MutableDiseaseIdx(cell) = 0xFF;
      g->world.MutableDisease(cell).count = 0;
      g->world.MutableDisease(cell).diseaseHash = 0;
      g->world.MutableDiseaseInfest(cell) = 0;
      g->world.MutableDiseaseAccum(cell) = 0.0f;
    }
    reported_idx = g->world.DiseaseIdx(cell);
  }
  if (m.callbackIdx != -1) {
    DiseaseConsumptionCallback cb{};
    cb.callbackIdx = m.callbackIdx;
    cb.diseaseIdx = reported_idx;
    cb.diseaseCount = taken;
    g->disease_consumed.push_back(cb);
  }
}

// `ProcessCellRadiationChanges`, first half.
//
// The callback does not go into `callbackInfo` — it goes into its own array,
// `radiationConsumedCallbacks`, and it carries the cell and an
// amount as well as the index. The amount is **the delta that was applied**, except when
// the delta drove the cell to zero or below, in which case it is the rads the cell *had*:
// a consumer that asks for more than is there is told how much it actually got.
void ApplyRadiationModification(const Sim::Pending& p) {
  CellRadiationModification m{};
  if (!Payload(p, &m) || !g->world.ValidGameCell(m.cellIdx)) return;
  const size_t cell = g->world.Padded(static_cast<size_t>(m.cellIdx));
  const float before = g->world.Radiation()[cell];
  float reported = m.radiationDelta;
  float v = before + reported;
  if (v <= 0.0f) {
    v = 0.0f;
    reported = before;
  }
  g->world.MutableRadiation(cell) = v;
  if (m.callbackIdx != -1) {
    g->radiation.consumed.push_back(
        ConsumedRadiationCallback{m.callbackIdx, m.cellIdx, reported});
  }
}

// `ProcessCellRadiationChanges`, second half. Five of the six type codes land somewhere;
// type 1 is not in Klei's chain at all and is silently dropped.
void ApplyRadiationParams(const Sim::Pending& p) {
  RadiationParamsModification m{};
  if (!Payload(p, &m)) return;
  g->world.SetRadiationParam(m.RadiationParamsType, m.value);
}

void ApplyBackwall(const Sim::Pending& p) {
  SetBackwallDataMsg m{};
  if (!Payload(p, &m) || !g->world.ValidGameCell(m.gameCell)) return;
  const size_t cell = g->world.Padded(static_cast<size_t>(m.gameCell));
  g->world.MutableBackwall(cell) =
      {g->elements.BackwallHash(m.elementIdx), m.mass, m.temperature};
}

// Component messages are drained **by type**, in the order `SimFrameManager::ProcessFrame`
// runs them, not in arrival order: every Add, then every Modify and energy
// change, then every Remove. The game registers a building and then immediately modifies it,
// so arrival order and type order genuinely differ on the frame a building is placed.
//
// Every Add reports the handle it allocated back through `componentStateChangedMessages`,
// which is the only way the game learns it — `SIM_HandleMessage` returns null for these.
void DrainBuildingQueue() {
  auto each = [](SimMessageHash id, void (*fn)(const Sim::Pending&)) {
    for (const Sim::Pending& p : g->active) {
      if (p.id == static_cast<int32_t>(id)) fn(p);
    }
  };

  each(SimMessageHash::SetDebugProperties, [](const Sim::Pending& p) {
    DebugProperties m{};
    if (!Payload(p, &m)) return;
    g->buildings.temperature_scale = m.buildingTemperatureScale;
    g->buildings.to_building_temperature_scale = m.buildingToBuildingTemperatureScale;
    // The third field. Nothing read it until the solid element emitter needed it: it is
    // what lets the sandbox tools drop ore into a cell the player has not uncovered.
    g->debug_editing = m.isDebugEditing != 0;
  });

  each(SimMessageHash::AddBuildingHeatExchange, [](const Sim::Pending& p) {
    AddBuildingHeatExchangeMessage m{};
    if (!Payload(p, &m)) return;
    const BuildingHeatExchangeData added = BuildingDataFromMessage(
        g->elements, m.elemIdx, m.mass, m.temperature, m.thermalConductivity,
        m.overheatTemperature, m.operatingKilowatts, m.minX, m.minY, m.maxX, m.maxY);
    // Energy ledger: a registered building brings its own thermal energy into the sim.
    g->world.NoteBuildingRegistered(BuildingStoredEnergy(added));
    const int32_t handle = g->buildings.exchange.Add(added);
    if (m.callbackIdx >= 0) {
      g->component_state.push_back(ComponentStateChangedMessage{m.callbackIdx, handle});
    }
  });

  each(SimMessageHash::ModifyBuildingHeatExchange, [](const Sim::Pending& p) {
    ModifyBuildingHeatExchangeMessage m{};
    // `callbackIdx` carries the handle on this message rather than a callback. The field is
    // reused, not misnamed: `SimMessages.ModifyBuildingHeatExchange` writes `sim_handle`
    // into it.
    if (!Payload(p, &m)) return;
    // Wholesale replacement, and the message carries a fresh temperature, so this can
    // teleport a building's heat. Charged as the real before/after difference -- reading
    // the record back out afterwards rather than trusting the message, because Modify is a
    // no-op on a dead handle.
    const BuildingHeatExchangeData* before = g->buildings.exchange.Get(m.callbackIdx);
    const double stored_before = before ? BuildingStoredEnergy(*before) : 0.0;
    ModifyBuilding(&g->buildings, m.callbackIdx,
                   BuildingDataFromMessage(g->elements, m.elemIdx, m.mass, m.temperature,
                                           m.thermalConductivity, m.overheatTemperature,
                                           m.operatingKilowatts, m.minX, m.minY, m.maxX,
                                           m.maxY));
    if (const BuildingHeatExchangeData* after = g->buildings.exchange.Get(m.callbackIdx)) {
      g->world.NoteBuildingRegistered(BuildingStoredEnergy(*after) - stored_before);
    }
  });

  each(SimMessageHash::ModifyBuildingEnergy, [](const Sim::Pending& p) {
    ModifyBuildingEnergyMessage m{};
    if (!Payload(p, &m)) return;
    if (BuildingHeatExchangeData* d = g->buildings.exchange.Get(m.handle)) {
      // What actually landed, not what was asked for: ApplyBuildingEnergy clamps, and
      // refuses outright when the bounds are out of range.
      const double stored_before = BuildingStoredEnergy(*d);
      ApplyBuildingEnergy(d, m.deltaKJ, m.minTemperature, m.maxTemperature);
      g->world.NoteBuildingEnergyMsg(BuildingStoredEnergy(*d) - stored_before);
    }
  });

  each(SimMessageHash::RemoveBuildingHeatExchange, [](const Sim::Pending& p) {
    RemoveBuildingHeatExchangeMessage m{};
    if (!Payload(p, &m)) return;
    // Energy ledger: the record's energy leaves the sim with it.
    if (const BuildingHeatExchangeData* d = g->buildings.exchange.Get(m.handle)) {
      g->world.NoteBuildingRegistered(-BuildingStoredEnergy(*d));
    }
    g->buildings.exchange.Remove(m.handle);
    if (m.callbackIdx >= 0) g->callbacks.push_back(CallbackInfo{m.callbackIdx});
  });

  // The two building-to-building hashes are crossed over relative to their names, and this
  // is not a transcription slip: `SimMessages.RegisterBuildingToBuildingHeatExchange` sends
  // `AddBuildingToBuildingHeatExchange`, and `SimMessages.AddBuildingToBuildingHeatExchange`
  // sends `AddInContactBuildingToBuildingToBuildingHeatExchange`. Register creates the
  // contact group and hands back its own handle; Add puts a neighbour into an existing one.
  each(SimMessageHash::AddBuildingToBuildingHeatExchange, [](const Sim::Pending& p) {
    RegisterBuildingToBuildingHeatExchangeMessage m{};
    if (!Payload(p, &m)) return;
    if (!g->buildings.exchange.Valid(m.structureTemperatureHandler)) return;
    BuildingToBuildingData d;
    d.self = m.structureTemperatureHandler;
    const int32_t handle = g->buildings.contact.Add(d);
    if (m.callbackIdx >= 0) {
      g->component_state.push_back(ComponentStateChangedMessage{m.callbackIdx, handle});
    }
  });

  each(SimMessageHash::RemoveBuildingInContactFromBuildingToBuildingHeatExchange,
       [](const Sim::Pending& p) {
         RemoveBuildingInContactFromBuildingToBuildingHeatExchangeMessage m{};
         if (!Payload(p, &m)) return;
         BuildingToBuildingData* d = g->buildings.contact.Get(m.selfHandler);
         if (!d) return;
         for (size_t i = 0; i < d->contacts.size(); ++i) {
           if (d->contacts[i].handle != m.buildingNoLongerInContactHandler) continue;
           d->contacts.erase(d->contacts.begin() + static_cast<ptrdiff_t>(i));
           break;
         }
       });

  each(SimMessageHash::AddInContactBuildingToBuildingToBuildingHeatExchange,
       [](const Sim::Pending& p) {
         AddBuildingToBuildingHeatExchangeMessage m{};
         if (!Payload(p, &m)) return;
         BuildingToBuildingData* d = g->buildings.contact.Get(m.selfHandler);
         if (!d) return;
         d->contacts.push_back(
             InContactBuilding{m.buildingInContactHandle, m.cellsInContact});
       });

  each(SimMessageHash::RemoveBuildingToBuildingHeatExchange, [](const Sim::Pending& p) {
    RemoveBuildingToBuildingHeatExchangeMessage m{};
    if (!Payload(p, &m)) return;
    g->buildings.contact.Remove(m.selfHandler);
    if (m.callbackIdx >= 0) g->callbacks.push_back(CallbackInfo{m.callbackIdx});
  });
}

// The chunk half of `SimFrameManager::ProcessFrame` plus the whole of
// `ProcessElementChunkMessages`. Same by-type drain the buildings get, and the
// order matters for the same reason: the game registers a chunk and modifies it in the same
// frame.
//
//   Add -> Move -> SetData -> Energy -> Adjuster -> Remove
//
// Note where `Move` sits. A chunk registered and moved in one frame lands where the move
// says; a chunk moved and then removed never exchanges from the new cell at all.
void DrainChunkQueue() {
  auto each = [](SimMessageHash id, void (*fn)(const Sim::Pending&)) {
    for (const Sim::Pending& p : g->active) {
      if (p.id == static_cast<int32_t>(id)) fn(p);
    }
  };

  each(SimMessageHash::AddElementChunk, [](const Sim::Pending& p) {
    AddElementChunkMessage m{};
    if (!Payload(p, &m)) return;
    const ElementChunkData added = ChunkDataFromMessage(g->elements, g->world, m);
    // Energy ledger: a registered chunk brings its own thermal energy into the sim, exactly
    // as a registered building does.
    g->world.NoteChunkRegistered(ChunkStoredEnergy(added));
    const int32_t handle = g->chunks.chunks.Add(added);
    if (m.callbackIdx != -1) {
      g->component_state.push_back(ComponentStateChangedMessage{m.callbackIdx, handle});
    }
  });

  each(SimMessageHash::MoveElementChunk, [](const Sim::Pending& p) {
    MoveElementChunkMessage m{};
    if (!Payload(p, &m)) return;
    MoveChunk(&g->chunks, g->world, m);
  });

  each(SimMessageHash::SetElementChunkData, [](const Sim::Pending& p) {
    SetElementChunkDataMessage m{};
    if (!Payload(p, &m)) return;
    // Wholesale replacement of temperature AND heat capacity, so this can teleport a lump's
    // heat. Charged as the real before/after difference, read back out of the record rather
    // than taken from the message, because Modify is a no-op on a dead handle.
    const ElementChunkData* before = g->chunks.chunks.Get(m.handle);
    const double stored_before = before ? ChunkStoredEnergy(*before) : 0.0;
    ModifyChunk(&g->chunks, m);
    if (const ElementChunkData* after = g->chunks.chunks.Get(m.handle)) {
      g->world.NoteChunkRegistered(ChunkStoredEnergy(*after) - stored_before);
    }
  });

  each(SimMessageHash::ModifyElementChunkEnergy, [](const Sim::Pending& p) {
    ModifyElementChunkEnergyMessage m{};
    if (!Payload(p, &m)) return;
    ModifyChunkEnergy(g->world, &g->chunks, m);
  });

  each(SimMessageHash::ModifyChunkTemperatureAdjuster, [](const Sim::Pending& p) {
    ModifyElementChunkAdjusterMessage m{};
    if (!Payload(p, &m)) return;
    ModifyChunkAdjuster(&g->chunks, m);
  });

  // A removal reports `-1` back through the same channel an add reports its handle on, which
  // is how the game learns the handle it was holding is now dead.
  each(SimMessageHash::RemoveElementChunk, [](const Sim::Pending& p) {
    RemoveElementChunkMessage m{};
    if (!Payload(p, &m)) return;
    // Energy ledger: the record's energy leaves the sim with it.
    if (const ElementChunkData* d = g->chunks.chunks.Get(m.handle)) {
      g->world.NoteChunkRegistered(-ChunkStoredEnergy(*d));
    }
    g->chunks.chunks.Remove(m.handle);
    if (m.callbackIdx != -1) {
      g->component_state.push_back(ComponentStateChangedMessage{m.callbackIdx, -1});
    }
  });
}

// The element consumer and the element emitter, in the order `SimFrameManager::ProcessFrame`
// walks them: both come after the chunk block, and within each component it is
// register, then modify, then unregister.
//
// `SetElementConsumerData` is Klei's `ModifyElementConsumerMessage` under a different name —
// the hash the game sends is `SetElementConsumerData` and the struct it carries is the modify
// payload, handle plus cell plus rate.
void DrainElementFlowQueue() {
  auto each = [](SimMessageHash id, void (*fn)(const Sim::Pending&)) {
    for (const Sim::Pending& p : g->active) {
      if (p.id == static_cast<int32_t>(id)) fn(p);
    }
  };

  each(SimMessageHash::AddElementConsumer, [](const Sim::Pending& p) {
    AddElementConsumerMessage m{};
    if (!Payload(p, &m)) return;
    if (!g->world.ValidGameCell(m.cellIdx)) return;
    ElementConsumerData d{};
    // `Register` seeds the rate at zero and leaves it there: a consumer does nothing at all
    // until a `SetElementConsumerData` gives it one.
    d.consumption_rate = 0.0f;
    d.cell = static_cast<int32_t>(g->world.Padded(static_cast<size_t>(m.cellIdx)));
    d.element = m.elementIdx;
    d.max_depth = m.radius;
    d.configuration = m.configuration;
    d.offset_idx = 0;
    const int32_t handle = g->flow.consumers.Add(d);
    if (m.callbackIdx != -1) {
      g->component_state.push_back(ComponentStateChangedMessage{m.callbackIdx, handle});
    }
  });

  each(SimMessageHash::SetElementConsumerData, [](const Sim::Pending& p) {
    SetElementConsumerDataMessage m{};
    if (!Payload(p, &m)) return;
    ElementConsumerData* d = g->flow.consumers.Get(m.handle);
    if (d == nullptr || !g->world.ValidGameCell(m.cell)) return;
    // Klei writes the cell first and the rate second, and writes nothing else — the
    // element, the radius and the configuration are fixed at registration.
    d->cell = static_cast<int32_t>(g->world.Padded(static_cast<size_t>(m.cell)));
    d->consumption_rate = m.consumptionRate;
  });

  each(SimMessageHash::RemoveElementConsumer, [](const Sim::Pending& p) {
    RemoveElementConsumerMessage m{};
    if (!Payload(p, &m)) return;
    g->flow.consumers.Remove(m.handle);
    if (m.callbackIdx != -1) {
      g->component_state.push_back(ComponentStateChangedMessage{m.callbackIdx, -1});
    }
  });

  each(SimMessageHash::AddElementEmitter, [](const Sim::Pending& p) {
    AddElementEmitterMessage m{};
    if (!Payload(p, &m)) return;
    // Every field but the pressure ceiling and the two callbacks comes from `Register`'s own
    // constants, which are `ElementEmitterData`'s defaults: interval FLT_MAX, mass zero,
    // temperature -1, blocked state 0xff.
    ElementEmitterData d{};
    d.max_pressure = m.maxPressure;
    d.blocked_cb = m.onBlockedCB;
    d.unblocked_cb = m.onUnblockedCB;
    const int32_t handle = g->flow.emitters.Add(d);
    // And the emitter's report slot is blanked at registration, before it has ever run.
    const size_t slot = static_cast<size_t>(handle & kHandleIndexMask);
    if (slot >= g->flow.emitted.size()) g->flow.emitted.resize(slot + 1);
    EmittedMassInfo& e = g->flow.emitted[slot];
    e.elemIdx = 0xFFFF;
    e.diseaseIdx = 0xFF;
    e.mass = 0.0f;
    e.temperature = 0.0f;
    e.diseaseCount = 0;
    if (m.callbackIdx != -1) {
      g->component_state.push_back(ComponentStateChangedMessage{m.callbackIdx, handle});
    }
  });

  each(SimMessageHash::ModifyElementEmitter, [](const Sim::Pending& p) {
    ModifyElementEmitterMessage m{};
    if (!Payload(p, &m)) return;
    ElementEmitterData* d = g->flow.emitters.Get(m.handle);
    if (d == nullptr || !g->world.ValidGameCell(m.cellIdx)) return;
    d->elapsed_time = 0.0f;
    d->emit_interval = m.emitInterval;
    d->emit_mass = m.emitMass;
    d->emit_temperature = m.emitTemperature;
    d->max_pressure = m.maxPressure;
    d->emit_disease_count = m.diseaseCount;
    d->cell = static_cast<int32_t>(g->world.Padded(static_cast<size_t>(m.cellIdx)));
    d->element = m.elementIdx;
    d->max_depth = m.maxDepth;
    d->offset_idx = 0;
    d->disease_idx = m.diseaseIdx;
    // The one conditional write in the whole function,: an emitter modified
    // to carry no mass has its blocked state forced back to the third value, so the next
    // update fires a blocked callback whatever it fired last time.
    if (!(m.emitMass > 0.0f)) d->blocked_state = 0xFF;
  });

  // The disease emitter. `Register` leaves a slot with no cell and no disease,
  // so a registered-but-unmodified emitter does nothing at all.
  each(SimMessageHash::AddDiseaseEmitter, [](const Sim::Pending& p) {
    AddDiseaseEmitterMessage m{};
    if (!Payload(p, &m)) return;
    const int32_t handle = g->flow.disease_emitters.Add(DiseaseEmitterData{});
    const size_t slot = static_cast<size_t>(handle & kHandleIndexMask);
    if (slot >= g->flow.disease_emitted.size()) g->flow.disease_emitted.resize(slot + 1);
    // `UpdateDataListOnly` writes exactly this into every slot, so it is what
    // an emitter that has never fired reports.
    g->flow.disease_emitted[slot].diseaseIdx = 0xFF;
    g->flow.disease_emitted[slot].count = 0;
    if (m.callbackIdx != -1) {
      g->component_state.push_back(ComponentStateChangedMessage{m.callbackIdx, handle});
    }
  });

  // `DiseaseEmitter::Modify`. Five writes and **no reset of `elapsedTime`** —
  // the element emitter's `Modify` zeroes its clock and this one does not, so a re-modified
  // disease emitter keeps counting from where it was.
  each(SimMessageHash::ModifyDiseaseEmitter, [](const Sim::Pending& p) {
    ModifyDiseaseEmitterMessage m{};
    if (!Payload(p, &m)) return;
    DiseaseEmitterData* d = g->flow.disease_emitters.Get(m.handle);
    if (d == nullptr || !g->world.ValidGameCell(m.gameCell)) return;
    d->emit_interval = m.emitInterval;
    d->disease_idx = m.diseaseIdx;
    d->range = m.maxDepth;
    d->emit_count = m.emitCount;
    d->cell = static_cast<int32_t>(g->world.Padded(static_cast<size_t>(m.gameCell)));
  });

  each(SimMessageHash::RemoveDiseaseEmitter, [](const Sim::Pending& p) {
    RemoveDiseaseEmitterMessage m{};
    if (!Payload(p, &m)) return;
    g->flow.disease_emitters.Remove(m.handle);
    if (m.callbackIdx != -1) {
      g->component_state.push_back(ComponentStateChangedMessage{m.callbackIdx, -1});
    }
  });

  each(SimMessageHash::RemoveElementEmitter, [](const Sim::Pending& p) {
    RemoveElementEmitterMessage m{};
    if (!Payload(p, &m)) return;
    g->flow.emitters.Remove(m.handle);
    if (m.callbackIdx != -1) {
      g->component_state.push_back(ComponentStateChangedMessage{m.callbackIdx, -1});
    }
  });
}

// The radiation emitter's three messages, drained after the element emitter's and before
// the disease emitter's, which is where `ProcessFrame` puts them.
//
// Unlike the element emitter, `Register` takes the whole configuration from the message —
// there is no inert default state to modify away from. The one thing it does invent is the
// clamp: `emitSpeed` is stored as `min(emitSpeed, emitRads)` here and as
// `min(emitRate, emitSpeed)` in `Modify`, which is a real asymmetry in the game.
void DrainRadiationQueue() {
  auto each = [](SimMessageHash id, void (*fn)(const Sim::Pending&)) {
    for (const Sim::Pending& p : g->active) {
      if (p.id == static_cast<int32_t>(id)) fn(p);
    }
  };

  each(SimMessageHash::AddRadiationEmitter, [](const Sim::Pending& p) {
    AddRadiationEmitterMessage m{};
    if (!Payload(p, &m)) return;
    if (!g->world.ValidGameCell(m.cell)) return;
    RadiationEmitterData d{};
    d.cell = static_cast<int32_t>(g->world.Padded(static_cast<size_t>(m.cell)));
    d.radius_x = static_cast<uint16_t>(m.emitRadiusX);
    d.radius_y = static_cast<uint16_t>(m.emitRadiusY);
    d.emit_rads = m.emitRads;
    d.emit_rate = m.emitRate;
    d.emit_speed = m.emitSpeed <= m.emitRads ? m.emitSpeed : m.emitRads;
    d.emit_direction = m.emitDirection;
    d.emit_angle = m.emitAngle;
    d.emit_timer = 0.0f;
    d.emit_step_timer = 0.0f;
    d.emit_type = m.emitType;
    d.emit_step = 0;
    const int32_t handle = g->radiation.emitters.Add(d);
    if (m.callbackIdx != -1) {
      g->component_state.push_back(ComponentStateChangedMessage{m.callbackIdx, handle});
    }
  });

  each(SimMessageHash::ModifyRadiationEmitter, [](const Sim::Pending& p) {
    ModifyRadiationEmitterMessage m{};
    if (!Payload(p, &m)) return;
    RadiationEmitterData* d = g->radiation.emitters.Get(m.handle);
    if (d == nullptr || !g->world.ValidGameCell(m.cell)) return;
    // The three pieces of running state — both timers and the phase — are **not** touched:
    // a modified emitter carries on mid-sweep.
    d->cell = static_cast<int32_t>(g->world.Padded(static_cast<size_t>(m.cell)));
    d->radius_x = static_cast<uint16_t>(m.emitRadiusX);
    d->radius_y = static_cast<uint16_t>(m.emitRadiusY);
    d->emit_rads = m.emitRads;
    d->emit_rate = m.emitRate;
    d->emit_speed = m.emitRate <= m.emitSpeed ? m.emitRate : m.emitSpeed;
    d->emit_direction = m.emitDirection;
    d->emit_angle = m.emitAngle;
    d->emit_type = m.emitType;
  });

  each(SimMessageHash::RemoveRadiationEmitter, [](const Sim::Pending& p) {
    RemoveRadiationEmitterMessage m{};
    if (!Payload(p, &m)) return;
    g->radiation.emitters.Remove(m.handle);
    if (m.callbackIdx != -1) {
      g->component_state.push_back(ComponentStateChangedMessage{m.callbackIdx, -1});
    }
  });
}

// One pass over the frame's messages, for one hash. The buildings and the chunks have had
// their own copy of this since they were written; it is at file scope now because the cell
// messages need it too. See `DrainQueue`.
void EachActive(SimMessageHash id, void (*fn)(const Sim::Pending&)) {
  for (const Sim::Pending& p : g->active) {
    if (p.id == static_cast<int32_t>(id)) fn(p);
  }
}

bool KnownMessage(int32_t id) {
  // GENERATED FROM `ONI_EXT_MESSAGE_LIST`, not restated. This was a 23-line chain of `if (id ==
  // ext::...) return true;` -- one line per row of a list that already exists, kept in step by
  // hand. Framework extension ids live outside SimMessageHash's (Klei's) numbering entirely, so
  // they are still checked before the switch below rather than folded into it as fake
  // enumerators; what changed is that the set is now read from the one place it is defined.
  if (ext::IsExtMessage(id)) return true;
  switch (static_cast<SimMessageHash>(id)) {
    case SimMessageHash::ModifyCell:
    case SimMessageHash::Dig:
    case SimMessageHash::MassEmission:
    case SimMessageHash::MassConsumption:
    case SimMessageHash::SetInsulationValue:
    case SimMessageHash::SetStrengthValue:
    case SimMessageHash::ModifyCellEnergy:
    case SimMessageHash::ChangeCellProperties:
    case SimMessageHash::ConsumeDisease:
    case SimMessageHash::CellDiseaseModification:
    case SimMessageHash::CellRadiationModification:
    case SimMessageHash::ModifyBackwallData:
    case SimMessageHash::SetDebugProperties:
    case SimMessageHash::AddBuildingHeatExchange:
    case SimMessageHash::ModifyBuildingHeatExchange:
    case SimMessageHash::ModifyBuildingEnergy:
    case SimMessageHash::RemoveBuildingHeatExchange:
    case SimMessageHash::AddBuildingToBuildingHeatExchange:
    case SimMessageHash::AddInContactBuildingToBuildingToBuildingHeatExchange:
    case SimMessageHash::RemoveBuildingInContactFromBuildingToBuildingHeatExchange:
    case SimMessageHash::RemoveBuildingToBuildingHeatExchange:
    case SimMessageHash::AddElementChunk:
    case SimMessageHash::MoveElementChunk:
    case SimMessageHash::SetElementChunkData:
    case SimMessageHash::ModifyElementChunkEnergy:
    case SimMessageHash::ModifyChunkTemperatureAdjuster:
    case SimMessageHash::RemoveElementChunk:
    case SimMessageHash::AddElementConsumer:
    case SimMessageHash::SetElementConsumerData:
    case SimMessageHash::RemoveElementConsumer:
    case SimMessageHash::AddElementEmitter:
    case SimMessageHash::ModifyElementEmitter:
    case SimMessageHash::RemoveElementEmitter:
    case SimMessageHash::AddDiseaseEmitter:
    case SimMessageHash::ModifyDiseaseEmitter:
    case SimMessageHash::RemoveDiseaseEmitter:
    case SimMessageHash::AddRadiationEmitter:
    case SimMessageHash::ModifyRadiationEmitter:
    case SimMessageHash::RemoveRadiationEmitter:
    case SimMessageHash::RadiationParamsModification:
      return true;
    default:
      return false;
  }
}

// **Klei does not drain in arrival order.** `SIM_HandleMessage` sorts each message into a
// per-category vector on `SimFrameInfo`, and `SimFrameManager::ProcessFrame`
// then walks those vectors in a fixed order that has nothing to do with the order the game
// sent them in. The order below is that function read top to bottom:
//
//   insulation, strength, `ProcessCellEnergyModifications`,
//   `ProcessCellProperties`, `ProcessMassConsumption`, `ProcessMassEmission`,
//   `ProcessConsumeDisease`, the inline disease-modification loop,
//   `ProcessCellRadiationChanges`, `ProcessDigPoints`, `ProcessCellModifications`,
//   then the components.
//
// Arrival order would be wrong, and a suite cannot tell unless a scenario sends two
// categories in one frame — see `msgorder`, which sends a
// `ModifyCell` and a `ModifyCellEnergy` at one cell and is worth 519 K when this is wrong.
// In live play the game sends thousands of messages a frame across a dozen categories, so
// arrival order would have been wrong on nearly every frame.
//
// Cost is one pass per category rather than one pass total. Eleven linear scans of a vector
// that holds a few thousand small structs is not measurable next to a substep, and bucketing
// on the way in would put the sort in `SIM_HandleMessage`, which runs on the game's thread.
void DrainQueue() {
  EachActive(SimMessageHash::SetInsulationValue,
             [](const Sim::Pending& p) { ApplyCellFloat(p, true); });
  EachActive(SimMessageHash::SetStrengthValue,
             [](const Sim::Pending& p) { ApplyCellFloat(p, false); });
  // Framework extension — see abi/sim_abi_ext.h. EachActive filters `g->active` by exact
  // int32 id match, so an extension id outside SimMessageHash's range works unmodified.
  EachActive(static_cast<SimMessageHash>(ext::kSetCellThermalMassBonus), &ApplyThermalMassBonus);
  EachActive(static_cast<SimMessageHash>(ext::kSetCellProperty), &ApplyCellProperty);
  EachActive(static_cast<SimMessageHash>(ext::kPublishCellProperty), &ApplyPublishCellProperty);
  EachActive(static_cast<SimMessageHash>(ext::kSubscribeEventStream), &ApplySubscribeEventStream);
  EachActive(static_cast<SimMessageHash>(ext::kInjectGasSpecies), &ApplyInjectGasSpecies);
  EachActive(static_cast<SimMessageHash>(ext::kRemoveVanillaMass), &ApplyRemoveVanillaMass);
  EachActive(static_cast<SimMessageHash>(ext::kConvertToVanillaMass), &ApplyConvertToVanillaMass);
  EachActive(static_cast<SimMessageHash>(ext::kPromoteRoom), &ApplyPromoteRoom);
  EachActive(static_cast<SimMessageHash>(ext::kSetBlockedGasAddPolicy),
             &ApplySetBlockedGasAddPolicy);
  EachActive(static_cast<SimMessageHash>(ext::kSetCellPropertyTransport),
             &ApplySetCellPropertyTransport);
  EachActive(static_cast<SimMessageHash>(ext::kSetPayloadMixing), &ApplySetPayloadMixing);
  EachActive(static_cast<SimMessageHash>(ext::kSetDissolvedTint), &ApplySetDissolvedTint);
  EachActive(static_cast<SimMessageHash>(ext::kSetEffervescence), &ApplySetEffervescence);
  EachActive(static_cast<SimMessageHash>(ext::kAddCellPropertyAmount),
             &ApplyAddCellPropertyAmount);
  EachActive(static_cast<SimMessageHash>(ext::kSetFieldSource), &ApplySetFieldSource);
  EachActive(static_cast<SimMessageHash>(ext::kSetInvertedGravityElement),
             &ApplySetInvertedGravityElement);
  EachActive(static_cast<SimMessageHash>(ext::kSetBuildingWasteHeatKilowatts),
             &ApplySetBuildingWasteHeat);
  EachActive(static_cast<SimMessageHash>(ext::kSetBuildingExhaust), &ApplySetBuildingExhaust);
  EachActive(static_cast<SimMessageHash>(ext::kSetBuildingRadiation),
             &ApplySetBuildingRadiation);
  EachActive(static_cast<SimMessageHash>(ext::kSetBuildingConvection),
             &ApplySetBuildingConvection);
  EachActive(static_cast<SimMessageHash>(ext::kSetEnvironmentTemperature),
             &ApplySetEnvironmentTemperature);
  EachActive(static_cast<SimMessageHash>(ext::kSetWorldEnvironment),
             &ApplySetWorldEnvironment);
  EachActive(static_cast<SimMessageHash>(ext::kSetWorldSun), &ApplySetWorldSun);
  EachActive(static_cast<SimMessageHash>(ext::kSetRandomState), &ApplySetRandomState);
  EachActive(static_cast<SimMessageHash>(ext::kSetSchedulingState),
             &ApplySetSchedulingState);
  EachActive(static_cast<SimMessageHash>(ext::kSetStableTicks), &ApplySetStableTicks);
  EachActive(static_cast<SimMessageHash>(ext::kSetDiseaseGrowth), &ApplySetDiseaseGrowth);
  EachActive(static_cast<SimMessageHash>(ext::kSetRegistryState), &ApplySetRegistryState);
  EachActive(static_cast<SimMessageHash>(ext::kSetExtCellState), &ApplySetExtCellState);
  EachActive(SimMessageHash::ModifyCellEnergy, &ApplyCellEnergy);
  EachActive(SimMessageHash::ChangeCellProperties, &ApplyCellProperties);
  EachActive(SimMessageHash::MassConsumption, &ApplyMassConsumption);
  EachActive(SimMessageHash::MassEmission, &ApplyMassEmission);
  EachActive(SimMessageHash::ConsumeDisease, &ApplyConsumeDisease);
  EachActive(SimMessageHash::CellDiseaseModification, &ApplyDiseaseModification);
  EachActive(SimMessageHash::CellRadiationModification, &ApplyRadiationModification);
  EachActive(SimMessageHash::RadiationParamsModification, &ApplyRadiationParams);
  EachActive(SimMessageHash::Dig, &ApplyDig);
  EachActive(SimMessageHash::ModifyCell, &ApplyModifyCell);
  // The backwall is the one cell message with no home in `ProcessFrame` — the game's
  // backwall data is not one of the vectors that function walks, and where Klei applies it
  // has not been found. It is last here because nothing else reads it.
  EachActive(SimMessageHash::ModifyBackwallData, &ApplyBackwall);

  // Then the components, in `SimData`'s own order.
  DrainBuildingQueue();
  DrainChunkQueue();
  DrainElementFlowQueue();
  DrainRadiationQueue();

  for (const Sim::Pending& p : g->active) {
    if (!KnownMessage(p.id)) NoteUnknown(p.id, p.payload.size());
  }
  g->active.clear();
}

// ------------------------------------------------------------------ the frame

// The backwall is inert on Klei's side — no kernel moves its mass or its temperature,
// measured — and the one thing the sim does with it is tell the game, every frame, which
// backwalls have left their own element's range. The swap itself is the game's, and it
// comes back as a `ModifyBackwallData`.
//
// The test, from `diffsim --scenario backwall`: a granite backwall at 1000 K announces
// (`highTemp` 942) while the same backwall at 500 K and at 100 K does not; water announces
// on both sides of 272.5 / 372.5; a massless backwall never announces at any temperature;
// and a Vacuum backwall at 20000 K never announces either. That last one is why the range
// alone is not the rule — Vacuum's `highTemp` is 0, so a pure range test would fire — its
// high transition points back at Vacuum, so there is nowhere for it to go.
//
// **Level-triggered, not edge-triggered**: Klei announces the same 40 cells on all 49
// frames of that run, 1960 announcements, for as long as they stay out of range. It read
// as a one-shot at first because `diffsim` stops comparing at the first divergent tick.
//
// So the list is a property of the world rather than of the frame, and it is kept as one:
// the backwall only ever changes when something writes it, so the set is rebuilt from the
// cells written since the last frame and published whole every frame.
bool BackwallShouldTransition(const World& w, const ElementTable& t, size_t p) {
  const SaveBackwall& b = w.Backwall()[p];
  if (b.mass <= 0.0f || !t.HasHash(b.elementHash)) return false;
  const uint16_t idx = t.IndexOfHash(b.elementHash);
  const Element& e = t.At(idx);
  const bool high = b.temperature > e.highTemp &&
                    e.highTempTransitionIdx != 0xFFFF && e.highTempTransitionIdx != idx;
  const bool low = b.temperature < e.lowTemp &&
                   e.lowTempTransitionIdx != 0xFFFF && e.lowTempTransitionIdx != idx;
  return high || low;
}

void UpdateBackwallTransitions() {
  World& w = g->world;
  if (!w.Allocated()) return;
  if (!w.BackwallDirtyAll() && w.BackwallDirty().empty()) return;
  const ElementTable& t = g->elements;
  std::vector<BackwallShouldTransitionInfo>& out = g->backwall_transitions;
  if (w.BackwallDirtyAll()) {
    // Ascending game-cell order, which is the order Klei announces them in.
    out.clear();
    for (size_t game = 0; game < w.GameCount(); ++game) {
      if (BackwallShouldTransition(w, t, w.Padded(game))) {
        out.push_back(BackwallShouldTransitionInfo{static_cast<int32_t>(game)});
      }
    }
  } else {
    for (uint32_t p : w.BackwallDirty()) {
      const int64_t game = w.GameIndex(p);
      if (!w.ValidGameCell(game)) continue;
      const BackwallShouldTransitionInfo key{static_cast<int32_t>(game)};
      // Kept sorted, so a cell a message writes lands where a full rescan would have put
      // it rather than at the end.
      auto at = std::lower_bound(out.begin(), out.end(), key,
                                 [](const BackwallShouldTransitionInfo& a,
                                    const BackwallShouldTransitionInfo& b) {
                                   return a.gameCell < b.gameCell;
                                 });
      const bool present = at != out.end() && at->gameCell == key.gameCell;
      if (BackwallShouldTransition(w, t, p)) {
        if (!present) out.insert(at, key);
      } else if (present) {
        out.erase(at);
      }
    }
  }
  w.ClearBackwallDirty();
}

void ClearFrameEvents() {
  g->state_change_ores.clear();
  g->dig_info.clear();
  g->spawn_ore_info.clear();
  g->mass_consumed.clear();
  g->disease_consumed.clear();
  g->mass_emitted.clear();
  g->callbacks.clear();
  g->radiation.consumed.clear();
  g->unstable_cells.clear();
  g->falling_liquid.clear();
  g->world_damage.clear();
  g->world.ClearCellMelted();
  g->component_state.clear();
  g->projection_events.Clear();
  // Cleared per frame, not per substep: Klei's `SimEvents` vectors live across the substeps
  // of one `PrepareGameData` and are drained by `CopySimDataToGame`, so a cell touched in
  // the first substep is still announced if the second one leaves it alone.
  if (g->world.Allocated()) g->world.ClearSubstanceTouched();
  // `temperatures` is deliberately not cleared: it is indexed by handle and rebuilt in full
  // by every substep that runs, so a frame with no physics still reports the last value each
  // building had rather than an empty list.
  g->buildings.events.ClearFrame();
  // `info` is deliberately not cleared here, for the same reason `temperatures` is not: it
  // is indexed by handle and carries a running `deltaKJ` that only a substep-free frame
  // resets. `melted` is a per-frame list and does get cleared.
  g->chunks.ClearFrame();
}

// Physics: conduction, state changes, sublimation and flow, in that order.
//
// The order is measured, not chosen. Conduction runs before the transition pass — a cell
// walked across its boundary by a hot neighbour is never observed above the boundary and
// still unchanged. The transition pass runs before flow — water on the floor of a sealed
// vacuum shaft has already lost mass upward on the first frame it reads as steam.
// Sublimation runs last of the three, which fell out of an element whose sublimation
// product freezes at room temperature and so oscillates every frame.
//
// The frame's elapsed time is not a timestep — it decides how many fixed 0.2 s substeps to
// run, with the remainder carried into the next frame. Measured: 0.05 s, 0.15 s and 0.2 s
// frames all give identical results and a 0.4 s frame gives exactly what two 0.2 s frames
// give.
//
// The first frame after Start or Load does no physics. That is not a guard, it is
// measured: Klei's sim transfers nothing on its first PrepareGameData and starts on the
// second, and an off-by-one here shows up as a permanent one-tick lead over the whole
// world.
// Returns the number of frames processed, which is what `GameDataUpdate` reports. Always
// one here, and deliberately so: Klei's count is *not* a fixed function of the call. Its sim
// runs on a worker thread and a `PrepareGameData` that finds two frames queued runs both and
// reports 2. Measured with `diffsim`, which now prints the count — roughly one call in
// fifteen. Reproducing that would mean reproducing the scheduler, so this reports the honest
// one-in-one-out and the harness treats a mismatch as a Klei-side event rather than a defect.
int StepPhysics(float elapsed) {
  if (g->skip_physics_frames > 0) {
    --g->skip_physics_frames;
    // `Sim::Main` takes the `UpdateComponentsDataListOnly` branch on a tick that runs no
    // frame, so the components still report — and for chunks that branch is
    // the only thing that ever zeroes `deltaKJ`. On *this* tick they report nothing at all,
    // because `RunFrame` has not drained the queue either: see the note there.
    UpdateChunkDataList(&g->chunks);
    UpdateEmitterDataList(&g->flow);
    return 1;
  }
  // A paused game, not a startup skip: `Sim::Main` takes this same
  // `UpdateComponentsDataListOnly` branch whenever `dt <= 0`, which is what
  // the game sends every render frame while paused — it keeps calling PrepareGameData with
  // no speed throttling between calls, since there is nothing to throttle. Reported bug:
  // this branch didn't exist here, so a paused frame fell into `SubstepsForFrame`, which
  // forces at least one substep even at dt = 0 (that "always at least one" rule is measured
  // and correct for a genuinely short *running* frame, e.g. dt = 0.05 at the slowest game
  // speed — it just does not apply to dt <= 0, which is a different condition and never a
  // running frame). The result was one full substep per paused render frame, uncapped by
  // any game speed, which is the fastest the sim can possibly run — confirmed against Klei
  // with `diffsim --scenario paused`, which sends dt = 0.0 every tick: Klei's cells sit at
  // the seed temperature all 20 ticks, ours drifted up to 55 K. Fixed the same way the
  // startup skip already is, since it is the identical branch: no substep, no flow, just
  // republish with a zeroed `deltaKJ`.
  if (elapsed <= 0.0f) {
    UpdateChunkDataList(&g->chunks);
    UpdateEmitterDataList(&g->flow);
    return 1;
  }
  // Conduction runs *before* flow within a substep, and the order is measured rather than
  // chosen. A 20 kg pocket of gas about to spill into an empty cell loses exactly the heat
  // a single 20 kg cell would lose, and the two cells then read the same temperature —
  // which only happens if the whole pocket conducts as one lump first and the transfer
  // splits the result afterwards. Flowing first leaves the two cells 8 K apart.
  // The flow accumulator holds one **frame** of transfers, not one substep. Where Klei
  // clears it has not been found — `SimBase::UpdateData` does not, and `UpdateFlowTexture`
  // only reads — but the cadence is measured twice over. `traceflow` publishes a different
  // set of cells on each of its first four ticks and nothing carries over, so it is not
  // cumulative; and `substeps`, the one scenario that sends `dt = 1.0` and gets five
  // substeps in a frame, is exact only if all five accumulate into the same buffer.
  //
  // It sits after the skip test, not before it, so a skipped physics frame leaves the
  // buffer alone. That is what Klei does by construction: it does not call `UpdateData`.
  g->world.ClearFlow();
  // Whether any of the disease work below has anything to do. See `RefreshDiseaseActive`.
  g->world.RefreshDiseaseActive();
  // The room graph is a snapshot (gas_rooms.h's own file
  // comment), and geometry can change during play -- digging, building
  // (`ApplyCellProperties`), or a natural phase transition crossing a solid boundary
  // (`TransitionCell`, physics.h) -- any of which sets `World::RoomsDirty`. Checked once a
  // FRAME, here, not once a substep or once a region: several substeps can run in one frame
  // (`SubstepsForFrame` below), and rebuilding the same graph several times over for one
  // real change would be pure waste. `BuildRoomGraph` itself re-derives `owned` from
  // `World::RoomPromoted` (gas_rooms.h), so a promoted room stays promoted across a rebuild
  // even if it split or merged with a neighbour. Deliberately NOT routed through
  // `ActivateVolumeFractions` -- that also resets `vf_stable`/`vf_tick`, which have nothing
  // to do with room connectivity and would just be wasted busywork on every ordinary dig.
  if (g->vf_active && g->world.RoomsDirty()) {
    g->vf_rooms = gas::BuildRoomGraph(g->world, g->elements, g->world.GameWidth(),
                                       g->world.GameHeight());
    g->world.MutableRoomsDirty() = false;
  }
  const int substeps = SubstepsForFrame(elapsed, &g->substep_carry);
  // The frame's own copy of the tunables (sim/tunables.h, THE ONE RULE), read once: nothing may
  // change the table while the substeps below run.
  const Tunables frame_tun = g_tunables;
  const float substep_seconds = frame_tun.substep_seconds;
  for (int i = 0; i < substeps; ++i) {
    g->pressure_dir = -g->pressure_dir;
    // The whole substep body runs **once per region**, which is `SimBase::UpdateData`'s own
    // shape: is one loop over the region list at stride 0x18 and everything from
    // the conduction dispatch to `Disease::PostProcess` sits inside it. Only
    // three things are outside — the pressure-direction negation, the outer
    // `CellSOA::CopyFrom` and the substep counter — and they are
    // the three that stay outside this loop as well.
    //
    // With one region this is exactly what running each sweep over the region list gave, so
    // every scenario that predates the region probes is unchanged; `regionadj` and
    // `regionlap` are the two that can tell the orderings apart.
    // One charge per substep, before the region loop -- the energy ledger's clock. Deliberately
    // outside the loop below: the substep body runs once per region, and counting time per
    // region is exactly the confusion this field exists to resolve.
    g->world.NoteSimSeconds(substep_seconds);
    for (size_t ri = 0; ri < g->world.RegionCount(); ++ri) {
      // What this region's substep can leave for `Project` to publish. The inclusive
      // rectangle is the widest any of the kernels below drives from — `StepPostProcess`
      // and the far end of a gas-pressure pair use it — and `ProjectReach` covers
      // how far past its own loop each of them can write. `StepGasDisplacement` is the one
      // exception, because its rows are clamped *away* from the region rather than towards
      // it, so it marks its own.
      const World::PaddedRect& pr = g->world.PaddedRegionInclusive(ri);
      g->world.MarkProjectDirtyRect(pr.x0, pr.y0, pr.x1, pr.y1);
      // The promotion gate, hoisted once per region rather than
      // recomputed per kernel: pass the live RoomGraph only when it's actually in sync with
      // this world (same guard `ApplyPromoteRoom`/`SIM_DebugRoomOwned` use) -- a stale or
      // not-yet-built graph gates nothing rather than gating by a wrong cell mapping.
      // `vf_active` off (every world with no promoted room) means this is always nullptr, so
      // every gated kernel's default-nullptr fast path is exactly what runs -- byte-identical
      // to vanilla. Every kernel below that has a promotion gate (`StepConduction`,
      // `StepGasPressure` and `StepGasDisplacement`, the liquid kernels and the consumers)
      // reads this same pointer, not one recomputed per call site.
      // Not const: the two liquid sweeps move mixture gas out of a liquid's way, and
      // waking the cell it lands in updates the room's awake count.
      gas::RoomGraph* rooms =
          (g->vf_active && g->vf_rooms.room_of.size() == g->world.PaddedCount()) ? &g->vf_rooms
                                                                                  : nullptr;
      // The planet, before anything else touches the cells (sim/environment.h, Mod 3). It is
      // a boundary condition for the substep: establish what the world's surface is being held
      // at, then let this substep's own physics act on it. Inert on every world that has not
      // been sent a `kSetWorldEnvironment`, which is all of them until Mod 3 sends one.
      {
        PROF(kWorldEnvironment);
        StepWorldEnvironment(&g->world, g->elements, ri, substep_seconds,
                             &g->textures.exposed_to_sun, &g->textures.sun_beam);
      }
      { PROF(kConduction); StepConduction(&g->world, g->elements, ri, rooms); }
      {
        PROF(kStateChange);
        // The sky vector is the same one `StepWorldEnvironment` just read, and it is passed
        // for the same set of cells: the planetary latent accumulator counts a gas/liquid
        // transition only where the planet's own atmosphere reaches. Null-safe and inert on
        // every world with no environment record.
        StepStateChange(&g->world, g->elements, &g->state_change_ores, ri,
                        &g->textures.exposed_to_sun, FrameVisible(), g->debug_editing,
                        &g->falling_liquid);
      }
      // Gas and liquid are two sweeps, not one, and gas goes first. `SimBase::UpdateData`
      // runs the `UpdatePressure` sweep and only then `UpdateLiquid`,
      // with a fresh `CellSOA::CopyFrom` between them — so liquid never sees a
      // grid gas is halfway through moving across, and gas never sees one a liquid has
      // already fallen through.
      {
        PROF(kGasPressure);
        // `rooms` computed once above, before `StepConduction` -- see that comment.
        StepGasPressure(&g->world, g->elements, g->diseases, g->pressure_dir, ri, rooms);
      }
      // The second gas sweep, off the *same* snapshot as the first — `UpdateData` does not
      // take a fresh `CellSOA::CopyFrom` until, after both. It has to come
      // straight after `StepGasPressure`, which is what owns that snapshot.
      {
        PROF(kGasDisplacement);
        // `rooms` computed once above, before `StepConduction` -- see that comment. This is
        // the other half of pressure-driven flow between cells: gating only
        // `StepGasPressure` would leave a promoted cell's mass open to vanilla eviction via
        // displacement instead.
        StepGasDisplacement(&g->world, g->elements, g->pressure_dir, ri, rooms, &g->diseases);
      }
      {
        PROF(kFlow);
        // `rooms` computed once above, before `StepConduction`. The liquid half of the
        // promotion gate: these two sweeps are the only movers left that
        // could still write a promoted cell, and the hole was real -- a promoted cell reads
        // as vacuum in vanilla's grid, which `StepFlow`'s own permeability test accepts as a
        // place to pour. They gate on `gas::IsMixtureOwnedCell`, not `IsRoomOwned`, so an
        // ordinary vanilla pond inside a promoted room keeps flowing; see that function.
        StepFlow(&g->world, g->elements, g->first_physics_substep, g->displace_rotation, ri,
                 rooms, &g->falling_liquid, FrameVisible(),
                 g->debug_editing, &g->diseases);
        // The second liquid sweep, off the *same* snapshot as the first — `UpdateData` runs
        // it inline between `UpdateLiquid` and the `CellSOA::CopyFrom`.
        // It has to come straight after `StepFlow`, which owns that snapshot
        // and the flow accumulator this sweep reads back.
        StepLiquidDisplacement(&g->world, g->elements, g->pressure_dir, g->displace_rotation,
                               ri, rooms, &g->diseases);
        // The fourth and last `CellSOA::CopyFrom` of the substep,. Two readers:
        // the flow texture's gate, so only the elements the texture can ask about are recorded
        // here (see `World::SnapshotFlowElements`), and the germ pair sweep, which retakes
        // the germ half itself (`StepDiseaseDiffusion`).
        g->world.SnapshotFlowElements();
      }
      {
        // Layer C mixing, and it goes AFTER both liquid sweeps rather than between them. The
        // question this kernel answers is "what does a settled pond look like", so it should
        // see the grid the water has finished moving through this substep -- mixing the
        // pre-flow arrangement would carbonate cells the water has already left. It has its
        // own `PROF` block rather than sharing `kFlow`'s because it is not a Klei kernel
        // sharing a Klei snapshot: it is ours, it is skippable, and a row that says how much
        // a mixing pass costs is exactly what decides whether to switch it off.
        PROF(kPayloadMix);
        StepPayloadMixing(&g->world, g->elements, ri);
      }
      // Post-process runs **after** flow, not before it. `SimBase::UpdateData` calls
      // `UpdatePressure`, then `UpdateLiquid`, then
      // `PostProcessCell`, in that order, and it is a real difference rather
      // than bookkeeping: the shuffle and `DoDensityDisplacement` see the grid a fluid cell
      // has already moved through, so the cell a liquid vacated is gas again by the time the
      // sweep reaches it and draws its number. Running it first cost one draw per falling
      // liquid cell per substep and put the whole world onto a different random stream.
      // The germ pair sweep sits between the liquid sweep and `PostProcessCell`, which is
      // where `UpdateData` puts it: is after `UpdateLiquid` and
      // before `PostProcessCell`. It reads the copy Klei takes after both
      // liquid sweeps, which it retakes itself, so it has to stay downstream of
      // them.
      if (g->world.DiseaseActive()) {
        PROF(kDisease);
        StepDiseaseDiffusion(&g->world, g->elements, g->diseases, ri);
      }
      // The components run after the fluid kernels and before the substep counter turns over,
      // which is where `SimData::UpdateComponents` sits inside `SimBase::UpdateData`. The
      // order is the order they sit in `SimData`: element consumer, element emitter,
      // radiation emitter, element chunk, building cell exchange, building-to-building,
      // disease emitter. Only the disease emitter is still missing.
      // The radiation field: decay, the cosmic occlusion column walk, and the three
      // sources. It sits immediately before `SimData::UpdateComponents` in `UpdateData`,
      // which puts it ahead of the emitter that feeds it.
      {
        PROF(kRadiation);
        StepRadiationField(&g->world, g->elements, g->diseases, &g->radiation, ri);
      }
      {
        PROF(kElementFlow);
        // `rooms` computed once above, before `StepConduction` -- see that comment.
        // `StepElementConsumers` is the real per-substep native port of the `ElementConsumer`
        // component GasPump/GasFilter/etc. actually use -- distinct from `ApplyMassConsumption` below, which only services a direct
        // `SimMessages.ConsumeMass` send (what dupe breathing uses). Both share
        // `RemoveMassFromCell`'s promotion gate now, but each needs its OWN call site updated
        // to actually pass a real pointer -- gating the shared function alone does nothing
        // for a caller that still passes `nullptr`.
        StepElementConsumers(&g->world, g->elements, g->diseases, &g->flow, substep_seconds, ri,
                             rooms);
        StepElementEmitters(&g->world, g->elements, g->diseases, &g->flow,
                            g->displace_rotation, substep_seconds, &g->spawn_ore_info,
                            &g->callbacks, FrameVisible(),
                            g->debug_editing, ri);
      }
      {
        PROF(kRadiation);
        StepRadiationEmitters(&g->world, g->elements, &g->radiation, substep_seconds, ri);
      }
      // `kPhaseFields`. Placed immediately after the radiation emitters because that is where
      // it belongs by kind -- it is the generic form of the walk that kernel does -- and
      // because the published contract is PAIRWISE ORDER, not an absolute index. A stock world registers no
      // field, so `StepFields` returns on its first line and this costs a call.
      {
        PROF(kFields);
        ext::StepFields(&g->world, g->elements, &g->world.ExtCells(),
                        g->elements.Attributes(), &g->fields, ri);
      }
      {
        PROF(kElementChunk);
        StepElementChunks(&g->world, g->elements, &g->chunks, substep_seconds, ri);
      }
      {
        PROF(kBuildingHeat);
        StepBuildingHeatExchange(&g->world, g->elements, &g->buildings, substep_seconds,
                                 &g->state_change_ores, ri, &g->textures.exposed_to_sun,
                                 &g->textures.sun_beam, FrameVisible(), g->debug_editing,
                                 &g->falling_liquid);
      }
      {
        PROF(kBuildingToBuilding);
        StepBuildingToBuilding(&g->world, &g->buildings, substep_seconds, ri);
      }
      // The disease emitter is **seventh and last** in `SimData`'s component list
      // (constructor), so it runs after both building exchanges. It is the
      // only component that creates germs.
      {
        PROF(kElementFlow);
        StepDiseaseEmitters(&g->world, g->elements, g->diseases, &g->flow, substep_seconds,
                            ri);
      }
      {
        PROF(kPostProcess);
        const CellModContext cm{&g->world, &g->elements, &g->diseases, g->displace_rotation};
        StepPostProcess(&g->world, g->elements, g->displace_rotation, &g->unstable_cells, ri,
                        &g->falling_liquid, FrameVisible(), g->debug_editing, &g->world_damage,
                        OverfullDisplace{&cm, &DoOverfullDisplace}, &g->diseases);
      }
      // Only now are the cells the substep emptied turned into vacuum. Klei has no such
      // sweep at all — it clears a cell at the moment a mover empties it, or leaves the
      // element in place until `PostProcessCell` reaches it and `Evaporate`s it — so the
      // pass has to sit *after* post-process or the sweep loses a gas cell, its two draws
      // and one negation of the shuffle stride. See the note above the function.
      //
      // It stays a whole-grid pass inside the per-region loop rather than a per-region one:
      // a mover may empty a cell outside the rectangle it is sweeping, because the
      // destination test is the active mask and that is the union of every region.
      { PROF(kZeroMassless); ZeroMasslessCells(&g->world, g->elements); }
      // `Disease::PostProcess` is the last call inside `UpdateData`'s region
      // loop — after the components, after `PostProcessCell` — so the growth sweep sees a
      // cell at the temperature and mass the whole substep left it at.
      if (g->world.DiseaseActive()) {
        PROF(kDisease);
        StepDiseasePostProcess(&g->world, g->elements, g->diseases,
                               g->world.RadiationEnabled(), ri);
      }
    }
    // The new multi-gas mixing layer, additive on top of
    // everything above rather than a replacement for it — PhaseEntry and vanilla's
    // single-element cells are untouched by this block, which only reads/writes
    // gas_species_/gas_mass_/gas_occupied_mask_/gas_dirty_/gas_sleeping_. Runs once per
    // substep, not once per region: a room can span region boundaries, and
    // TickRoomPooledMixing already walks every room in the world regardless of which
    // region this pass is active over.
    //
    // `vf_active` starts false and only activation sets it, so a world that never receives
    // kInjectGasSpecies (or a promotion) takes the `if` here and nothing else.
    if (g->vf_active) {
      PROF(kVolumeFractions);
      if (gas::ShouldTick(g->vf_tick,
                          static_cast<uint8_t>(frame_tun.room_mixing_every_n_ticks))) {
        // No liquid cell keeps mixture slots past a mixing tick. The liquid sweeps move
        // them aside when they meet them; this catches every other way they arrive (a
        // ModifyCell liquid over mixture gas, or a save whose ponds already carry slots).
        if (g->vf_rooms.room_of.size() == g->world.PaddedCount()) {
          gas::EvictMixtureFromCondensedCells(g->world, g->elements, g->vf_rooms,
                                              g->displace_rotation);
        }
        // Both first guesses, from gastest.cpp's room-mixing benchmarks (see their definition).
        gas::TickRoomPooledMixing(g->world, g->elements, g->vf_rooms, frame_tun.room_mixing_rate,
                                  g->vf_stable, frame_tun.room_mixing_sleep_threshold);
      }
      ++g->vf_tick;
    }
    // `kPhaseEffervescence`: supersaturated liquid gives gas back as bubbles. Once per
    // substep and outside the region loop, because its criterion needs each cell's whole column
    // and a column crosses regions. Last among the substep phases, so it sees the grid every
    // mover has finished with; returns on its first line unless `kSetEffervescence` enabled it.
    {
      PROF(kEffervescence);
      StepEffervescence(&g->world, g->elements, substep_seconds);
    }
    g->first_physics_substep = false;
    ++g->displace_rotation;
  }
  // A transition that names a transition ore hands that share of the cell's mass to the
  // game rather than keeping it in the grid, so it has to be announced or the mass is
  // simply gone. Klei reports it in the same list as a dug-out cell.
  for (const StateChangeOre& o : g->state_change_ores) {
    SpawnOreInfo ore{};
    ore.cellIdx = o.game_cell;
    ore.elemIdx = o.element;
    ore.mass = o.mass;
    ore.temperature = o.temperature;
    ore.diseaseIdx = o.disease_idx;
    ore.diseaseCount = o.disease_count;
    g->spawn_ore_info.push_back(ore);
  }
  g->state_change_ores.clear();
  return 1;
}

// The `spawnOreInfo` export, which is not a copy: `SimBase::CopySimDataToGame` **sorts** the
// sim's list and then **coalesces** it (0x180032... , the `_Sort_unchecked` at the top of the
// spawn-ore block and the run-merge loop after it).
//
// Two ore drops that name the same cell *and* the same element become one entry carrying
// their combined mass and a mass-weighted temperature clamped to the two — through
// `FastCalculateCombinedTemperature`, with the cooler side passed first. `diseaseIdx` and
// `diseaseCount` are **not** merged: the first entry of a run keeps its own and the rest are
// discarded.
//
// Nothing in this project needed it until a solid element emitter turned up, because that is
// the first thing that can drop several lumps of ore in one frame. It applies to transition
// ore as well, which is why it lives here rather than in the emitter: the sort is what makes
// the list's *order* a property of the world rather than of the kernel that filled it.
//
// `digInfo` is a separate vector and is **not** sorted, even though it holds the same struct.
void PublishSpawnOre() {
  std::vector<SpawnOreInfo>& v = g->spawn_ore_info;
  if (v.size() < 2) return;
  std::sort(v.begin(), v.end(), [](const SpawnOreInfo& a, const SpawnOreInfo& b) {
    if (a.cellIdx != b.cellIdx) return a.cellIdx < b.cellIdx;
    return a.elemIdx < b.elemIdx;
  });
  size_t out = 0;
  for (size_t i = 1; i < v.size(); ++i) {
    SpawnOreInfo& head = v[out];
    const SpawnOreInfo& next = v[i];
    if (head.cellIdx != next.cellIdx || head.elemIdx != next.elemIdx) {
      v[++out] = next;
      continue;
    }
    const float total = head.mass + next.mass;
    if (total > 0.0f) {
      const bool next_cooler = next.temperature < head.temperature;
      const float m_a = next_cooler ? next.mass : head.mass;
      const float t_a = next_cooler ? next.temperature : head.temperature;
      const float m_b = next_cooler ? head.mass : next.mass;
      const float t_b = next_cooler ? head.temperature : next.temperature;
      head.temperature = FastCalculateCombinedTemperature(m_a, t_a, m_b, t_b);
    } else {
      head.temperature = 0.0f;
    }
    head.mass = head.mass + next.mass;
  }
  v.resize(out + 1);
}

// ---------------------------------------------------------------- the in-game digest log
//
// A golden that needs no mod, no Klei and no written scenario. Once per published frame it
// hashes the arrays the game is about to read and appends one line to `sim_digest.log`, next
// to the DLL.
//
// Two jobs. It is the **determinism** oracle: the same save run twice must produce the same
// digests line for line, and when the sim moves onto its own thread the
// digests must still match the ones taken while it was synchronous — which is the whole
// argument that threading changed nothing. And it carries `SIM_DebugLedger`'s mass drift,
// which is the one correctness instrument that works on a real 234-cycle colony where no
// golden exists.
//
// Off unless a marker file `sim_digest.on` sits beside the DLL, because hashing four arrays
// over a quarter of a million cells is not free and would otherwise pollute every frame-time
// measurement taken in the live game.
FILE* g_digest = nullptr;
bool g_digest_checked = false;
uint64_t g_digest_frame = 0;

uint64_t DigestBytes(uint64_t h, const void* p, size_t n) {
  const uint8_t* b = static_cast<const uint8_t*>(p);
  for (size_t i = 0; i < n; ++i) {
    h ^= b[i];
    h *= 1099511628211ULL;  // FNV-1a
  }
  return h;
}

void WriteDigest(const GameDataUpdate& u, int frames) {
  if (!g_digest_checked) {
    g_digest_checked = true;
    if (FILE* marker = fopen("sim_digest.on", "rb")) {
      fclose(marker);
      g_digest = fopen("sim_digest.log", "w");
    }
  }
  if (!g_digest) return;
  const size_t n = g->world.GameCount();
  uint64_t h = 1469598103934665603ULL;
  h = DigestBytes(h, u.elementIdx, n * sizeof(uint16_t));
  h = DigestBytes(h, u.mass, n * sizeof(float));
  h = DigestBytes(h, u.temperature, n * sizeof(float));
  h = DigestBytes(h, u.diseaseCount, n * sizeof(int32_t));
  // The mass ledger, the same numbers `SIM_DebugLedger` exports: the grid total and the net
  // of every sanctioned flow. `grid - net` is the drift, and what matters is whether it
  // *moves* from frame to frame — on a real colony there is no golden to check against, so a
  // drift that stays put is the strongest correctness statement available.
  const World::Ledger& l = g->world.Books();
  // Per **call site**, in `SIM_DebugLedger`'s order minus its field 0. The buckets are
  // cumulative, so what goes in the log is each one's *delta* for this frame: a drift step
  // is only diagnosable next to the list of sites that ran on the frame it stepped. Sites
  // that did nothing are left out, which keeps the line short on the overwhelming majority
  // of frames where nothing crosses the boundary at all.
  static const char* const kBucketNames[] = {
      "emitted", "modified", "consumed", "dug",  "ore",     "unstable",  "sublimated",
      "wisp",    "thin_liq", "cleared",  "comp_consumed", "comp_emitted", "emitter_ore",
      "mover",   "world_init", "atmos_boundary", "dissolved_surface"};
  const double buckets[] = {l.emitted,   l.modified,   l.consumed,          l.dug,
                            l.ore,       l.unstable,   l.sublimated,        l.wisp,
                            l.thin_liquid, l.cleared,  l.component_consumed,
                            l.component_emitted, l.emitter_ore, l.mover, l.world_init,
                            l.atmosphere_boundary, l.dissolved_surface};
  const int kBuckets = static_cast<int>(sizeof(buckets) / sizeof(buckets[0]));
  static double prev_buckets[sizeof(buckets) / sizeof(buckets[0])] = {};
  static double prev_drift = 0.0;

  // Gains minus losses, and the *same* formula as `LedgerNet` in `driver/src/diffsim.cpp`
  // — the two have to agree or the offline suite and the live game are measuring different
  // quantities under the same name. `emitter_ore` is deliberately not in the sum: a solid
  // element emitter hands the game a lump of ore made out of mass the building holds
  // outside the sim, so it is neither a grid gain nor a grid loss.
  // `atmosphere_boundary` and `dissolved_surface` are here as they are in `LedgerNet`; without
  // them a planet's sky exchange and surface uptake would read as digest drift.
  const double net = l.emitted + l.modified + l.component_emitted + l.mover + l.world_init +
                     l.atmosphere_boundary + l.dissolved_surface -
                     (l.consumed + l.dug + l.ore + l.unstable + l.sublimated + l.wisp +
                      l.thin_liquid + l.cleared + l.component_consumed);
  const double grid = g->world.TotalGridMass();
  const double drift = grid - net;

  fprintf(g_digest,
          "frame %llu frames %d cells %zu digest %016llx grid %.6f net %.6f drift %.6f "
          "ddrift %.6f",
          static_cast<unsigned long long>(g_digest_frame++), frames, n,
          static_cast<unsigned long long>(h), grid, net, drift, drift - prev_drift);
  prev_drift = drift;
  for (int i = 0; i < kBuckets; ++i) {
    const double d = buckets[i] - prev_buckets[i];
    prev_buckets[i] = buckets[i];
    if (d != 0.0) fprintf(g_digest, " %s %.6f", kBucketNames[i], d);
  }
  fputc('\n', g_digest);
  fflush(g_digest);
}

GameDataUpdate* BuildUpdate(int frames) {
  // `CopySimDataToGame` ends by swapping the frame's visibility buffer with the sim-side
  // `GameData`'s. See `Sim::visible`.
  std::swap(g->visible, g->visible_game[g->visible_sim_slot]);
  // The slot the worker is allowed to fill, and it advances on every publication rather than
  // only on the threaded ones — a Start followed by the first `PrepareGameData` publishes
  // twice in a row with the game holding the first pointer in between.
  PublishedFrame& p = g->pub[g->pub_slot];
  g->pub_slot ^= 1;
  p.Begin();
  GameDataUpdate& u = p.update;
  u.numFramesProcessed = frames;

  ProjectionBuffers& b = g->buffers;
  p.Put(u.elementIdx, b.element);
  p.Put(u.temperature, b.temperature);
  p.Put(u.mass, b.mass);
  p.Put(u.properties, b.properties);
  p.Put(u.insulation, b.insulation);
  p.Put(u.strengthInfo, b.strength);
  p.Put(u.radiation, b.radiation);
  p.Put(u.diseaseIdx, b.disease_idx);
  p.Put(u.diseaseCount, b.disease_count);
  p.Put(u.backwallElement, b.backwall_element);
  p.Put(u.backwallMass, b.backwall_mass);
  p.Put(u.backwallTemperature, b.backwall_temperature);
  p.Put(u.accumulatedFlow, b.accumulated_flow);

  u.numSubstanceChangeInfo = static_cast<int32_t>(g->projection_events.substance.size());
  p.Put(u.substanceChangeInfo, g->projection_events.substance);
  u.numSolidInfo = static_cast<int32_t>(g->projection_events.solid.size());
  p.Put(u.solidInfo, g->projection_events.solid);
  u.numSolidSubstanceChangeInfo =
      static_cast<int32_t>(g->projection_events.solid_substance.size());
  p.Put(u.solidSubstanceChangeInfo, g->projection_events.solid_substance);
  u.numLiquidChangeInfo = static_cast<int32_t>(g->projection_events.liquid.size());
  p.Put(u.liquidChangeInfo, g->projection_events.liquid);
  u.numDigInfo = static_cast<int32_t>(g->dig_info.size());
  p.Put(u.digInfo, g->dig_info);
  PublishSpawnOre();
  u.numSpawnOreInfo = static_cast<int32_t>(g->spawn_ore_info.size());
  p.Put(u.spawnOreInfo, g->spawn_ore_info);
  u.numMassConsumedCallbacks = static_cast<int32_t>(g->mass_consumed.size());
  p.Put(u.massConsumedCallbacks, g->mass_consumed);
  u.numMassEmittedCallbacks = static_cast<int32_t>(g->mass_emitted.size());
  p.Put(u.massEmittedCallbacks, g->mass_emitted);
  u.numCallbackInfo = static_cast<int32_t>(g->callbacks.size());
  p.Put(u.callbackInfo, g->callbacks);
  u.numUnstableCellInfo = static_cast<int32_t>(g->unstable_cells.size());
  p.Put(u.unstableCellInfo, g->unstable_cells);
  u.numCellMeltedInfos = static_cast<int32_t>(g->world.CellMelted().size());
  p.Put(u.cellMeltedInfos, g->world.CellMelted());
  u.numWorldDamageInfo = static_cast<int32_t>(g->world_damage.size());
  p.Put(u.worldDamageInfo, g->world_damage);
  u.numSpawnFallingLiquidInfo = static_cast<int32_t>(g->falling_liquid.size());
  p.Put(u.spawnFallingLiquidInfo, g->falling_liquid);
  u.numBackwallShouldTransitionInfos =
      static_cast<int32_t>(g->backwall_transitions.size());
  p.Put(u.backwallShouldTransitionInfos, g->backwall_transitions);
  u.numComponentStateChangedMessages =
      static_cast<int32_t>(g->component_state.size());
  p.Put(u.componentStateChangedMessages, g->component_state);
  // Filled by `ProcessCellRadiationChanges`, not by the emitters: the sim tells the game how
  // many rads each `CellRadiationModification` with a callback actually moved.
  u.numRadiationConsumedCallbacks = static_cast<int32_t>(g->radiation.consumed.size());
  p.Put(u.radiationConsumedCallbacks, g->radiation.consumed);

  // The two element components. `consumed` is a list of what the pumps took this frame and
  // is handed over whole; `emitted` is indexed by handle slot and is only partly reset, so
  // both go through a published copy the reset cannot walk over.
  PublishElementFlow(&g->flow, g->elements.VacuumIndex());
  u.numDiseaseConsumptionCallbacks = static_cast<int32_t>(g->disease_consumed.size());
  p.Put(u.diseaseConsumptionCallbacks, g->disease_consumed);
  u.numDiseaseEmittedInfos = static_cast<int32_t>(g->flow.published_disease_emitted.size());
  p.Put(u.diseaseEmittedInfos, g->flow.published_disease_emitted);
  u.numRemovedMassEntries = static_cast<int32_t>(g->flow.published_consumed.size());
  p.Put(u.removedMassEntries, g->flow.published_consumed);
  u.numEmittedMassEntries = static_cast<int32_t>(g->flow.published_emitted.size());
  p.Put(u.emittedMassEntries, g->flow.published_emitted);

  // Element chunks. `info` is indexed by handle slot, `melted` is a bare handle each.
  // `PublishChunkInfo` hands out a copy and zeroes the accumulator behind it, which is what
  // makes `deltaKJ` one frame's energy rather than the running total the substep builds.
  PublishChunkInfo(&g->chunks);
  u.numElementChunkInfos = static_cast<int32_t>(g->chunks.published.size());
  p.Put(u.elementChunkInfos, g->chunks.published);
  u.numElementChunkMeltedInfos = static_cast<int32_t>(g->chunks.melted.size());
  p.Put(u.elementChunkMeltedInfos, g->chunks.melted);

  // Buildings. `buildingTemperatures` is the one the game reads every frame — it is how a
  // machine's temperature gets back into `StructureTemperatureComponents` at all. The other
  // three are `MeltedInfo`, a bare handle each.
  BuildingEvents& be = g->buildings.events;
  u.numBuildingTemperatures = static_cast<int32_t>(be.temperatures.size());
  p.Put(u.buildingTemperatures, be.temperatures);
  u.numBuildingOverheatInfos = static_cast<int32_t>(be.overheated.size());
  p.Put(u.buildingOverheatInfos, be.overheated);
  u.numBuildingNoLongerOverheatedInfos =
      static_cast<int32_t>(be.no_longer_overheated.size());
  p.Put(u.buildingNoLongerOverheatedInfos, be.no_longer_overheated);
  u.numBuildingMeltedInfos = static_cast<int32_t>(be.melted.size());
  p.Put(u.buildingMeltedInfos, be.melted);

  // These are raw buffers the game feeds to Texture2D.LoadRawTextureData, not handles.
  // A null here is not "no texture", it is a null dereference inside Unity, so they are
  // always allocated and always published.
  p.Put(u.propertyTextureFlow, g->textures.flow);
  p.Put(u.propertyTextureLiquid, g->textures.liquid);
  p.Put(u.propertyTextureLiquidData, g->textures.liquid_data);
  p.Put(u.propertyTextureMaterialData, g->textures.material_data);
  p.Put(u.propertyTextureExposedToSunlight, g->textures.exposed_to_sun);

  // The per-frame descriptor table, one entry per SUBSCRIBED property. The
  // vector is sized in full before the first `PutBytes`, because a fixup is the address of
  // `d.data` and a vector that grows afterwards moves every one of them.
  {
    const ext::CellPropertyRegistry& reg = g->world.ExtCells();
    const int32_t subscribed = reg.PublishedCount();
    p.ext.assign(static_cast<size_t>(subscribed), ext::ExtPublishedProperty{});
    size_t slot = 0;
    for (int32_t idx = 0; idx < reg.Count() && slot < p.ext.size(); ++idx) {
      if (!reg.Published(idx)) continue;
      const ext::CellPropertyRegistry::Property* prop = reg.At(idx);
      if (prop == nullptr) continue;
      ext::ExtPublishedProperty& d = p.ext[slot++];
      memset(&d, 0, sizeof(d));
      const size_t n = prop->name.size() < sizeof(d.name) - 1 ? prop->name.size()
                                                             : sizeof(d.name) - 1;
      memcpy(d.name, prop->name.c_str(), n);
      d.type = prop->type;
      d.persist = prop->persist;
      d.arity = prop->arity;
      d.stride = static_cast<int32_t>(ext::ScalarStride(prop->type));
      d.cellCount = static_cast<int32_t>(reg.Cells());
      const std::vector<uint8_t>& src = reg.Bytes(idx);
      d.byteCount = static_cast<int32_t>(src.size());
      // THE COPY IS THE POINT, not an inefficiency to optimise away later. The registry's
      // storage is what the kernels write, and under threading they are writing it while the
      // game reads this. Copying into the frame is what makes "valid for the tick" true.
      p.PutBytes(d.data, src.data(), src.size());
    }
  }

  // One descriptor per SUBSCRIBED stream. Same sizing rule as the block above
  // and for the same reason -- a fixup holds the address of `d.data`, so the vector must be at
  // its final size before the first `PutBytes` into it.
  FlushLiquidPayloadRecords();
  {
    ext::EventStreamRegistry& streams = g->ext_events;
    p.streams.assign(static_cast<size_t>(streams.SubscribedCount()),
                     ext::ExtPublishedStream{});
    size_t slot = 0;
    for (int32_t idx = 0; idx < streams.Count() && slot < p.streams.size(); ++idx) {
      if (!streams.Subscribed(idx)) continue;
      const ext::EventStreamRegistry::Stream* st = streams.At(idx);
      if (st == nullptr) continue;
      ext::ExtPublishedStream& d = p.streams[slot++];
      memset(&d, 0, sizeof(d));
      const size_t n = st->name.size() < sizeof(d.name) - 1 ? st->name.size()
                                                            : sizeof(d.name) - 1;
      memcpy(d.name, st->name.c_str(), n);
      d.stride = st->stride;
      d.count = streams.Records(idx);
      d.byteCount = static_cast<int32_t>(st->bytes.size());
      d.dropped = st->dropped;
      p.PutBytes(d.data, st->bytes.data(), st->bytes.size());
    }
  }

  // Only now do the offsets become pointers: every `Put` above may have moved the buffer.
  p.Finish();
  WriteDigest(u, frames);
  g->last_published = &u;
  // And this is the line the whole "cleared after the publish, not at the top
  // of the frame" rule comes down to. The frame owns its own copy of every record by now, so
  // dropping the registry's is safe -- and doing it HERE rather than in `ClearFrameEvents`
  // is what lets a record produced between frames, on the game thread's immediate-message
  // path, still reach a published frame. See `ext_events.h`'s `ClearFrame`.
  g->ext_events.ClearFrame();
  return &u;
}

// Once per frame start, on the game thread, immediately before the frame begins -- the one
// release Klei makes at the top of `Sim::Main`. Called from `PrepareGameData`
// on both of its paths: before the synchronous `RunFrame` of the first call after a Start, an
// Alloc or a Load (or of every call under `sim_nothread.on`), and before each `Kick`. Exactly
// one call per frame, so the parity in `ConduitTemperatures` flips exactly as often as it did
// when this lived inside `RunFrame`, and a handle the game removes during tick N is still
// released at the start of frame N + 1.
//
// WHY NOT INSIDE THE FRAME. `RunFrame` runs on the sim worker; the conduit exports run on the
// game thread and are not behind `WaitIdle` (they would stall every conduit call on the
// in-flight frame). A release on the worker compacted the conduit vector concurrently with
// `ConduitTemperatureManager_Update` iterating it and `_Set` writing through a pointer into it
// -- and `ConduitFlow.RebuildConnections` removes EVERY handle of a conduit type, so the queue
// is non-empty after every pipe placed or removed.
void ReleaseConduitHandlesForNextFrame() { g->conduits.ReleaseQueuedHandles(); }

GameDataUpdate* RunFrame() {
  if (!g->world.Allocated()) return nullptr;
  PROF(kFrame);
  if (g_prof.on) ++g_prof.frames;
  ClearFrameEvents();
  // Klei releases the conduit handles removed during the previous frame HERE,
  // immediately before `BeginFrameProcessing` drains the queue. Ours does not, and the move
  // is a threading fix rather than a reordering: this function runs on the sim worker, while
  // every `ConduitTemperatureManager_*` export runs on the game thread (and `Set` on the
  // game's job threads) straight after `PrepareGameData` kicks this frame. Releasing here
  // compacted the conduit vector underneath an `Update` walking it. The release now happens
  // in `ReleaseConduitHandlesForNextFrame`, on the game thread, at the one point that is
  // still "the start of this frame" as the game can observe it -- nothing the game does can
  // land between that call and this frame starting. See `sim/conduits.h`'s `Remove`.
  // The frame drains the messages of the tick *before* it, not its own. See `Sim::active`.
  // The rotation that fills `active` is not here any more: it belongs to the game thread now
  // and happens at `PrepareGameData`, once this frame is collected and before the next one is
  // handed over. Doing it here would mean touching `g->queue` while the game is pushing onto
  // it. The mapping is unchanged — a message still waits exactly one frame.
  { PROF(kDrainQueue); DrainQueue(); }
  const int frames = StepPhysics(g->elapsed_seconds);
  UpdateBackwallTransitions();
  {
    PROF(kProject);
    Project(g->world, g->elements, &g->buffers, &g->projection_events, g->first_frame);
  }
  {
    PROF(kTextures);
    FillPropertyTextures(g->world, g->elements, g->buffers, &g->textures);
  }
  // See `World::PromoteWorldOffsets`: the frame that received the offsets publishes without
  // them, exactly as Klei's does.
  g->world.PromoteWorldOffsets();
  g->first_frame = false;
  return BuildUpdate(frames);
}

}  // namespace

extern "C" {

// WHO THIS DLL IS. Not part of Klei's ABI and never called by the game -- the caller is
// OniFramework.SimVersion, which P/Invokes it and puts the answer on the build watermark so a
// running game states which sim it has loaded instead of leaving it to be inferred.
//
// The stock SimDLL does not export this symbol, and that is the whole detection mechanism:
// the managed side calls it, catches EntryPointNotFoundException, and reports STOCK. There is
// therefore nothing to add to a vanilla DLL and no version negotiation to get wrong.
//
// Callable at any time, including before SIM_Initialize and after SIM_Shutdown: it reads a
// string literal baked in at compile time (version.h) and touches no sim state, so it does
// not take the worker lock and cannot be made to wait on a frame. The returned pointer is
// static storage owned by the DLL -- the caller marshals it and never frees it.
__declspec(dllexport) const char* SIM_Version() {
  return oni_sim::kVersionString;
}

// Every event stream this DLL publishes is declared here, once, at
// `SIM_Initialize` -- there is no message that reaches `EventStreamRegistry::Register` and
// there is not meant to be (see `kSubscribeEventStream` in the ABI header). Declaring them all
// in one function is what makes `SIM_ExtEventStreamIndex` able to promise a stable index: the
// set never changes after this returns, so a caller resolves a name once.
//
// A failure here is a bug in this file, not in a mod, so it is reported and then tolerated --
// the stream simply does not exist, `Find` returns -1, and every `EmitRefusal` becomes a
// no-op. Taking the DLL down over a diagnostic that failed to declare itself would be worse
// than the diagnostic being missing.
void RegisterFirstPartyStreams() {
  std::string error;
  g->stream_refused = g->ext_events.Register(
      ext::kStreamMessageRefused, static_cast<int32_t>(sizeof(ext::ExtRefusedMessage)), &error);
  if (g->stream_refused < 0) Report(error.c_str());
  g->stream_payload_released = g->ext_events.Register(
      ext::kStreamLiquidPayloadReleased,
      static_cast<int32_t>(sizeof(ext::LiquidPayloadReleased)), &error);
  if (g->stream_payload_released < 0) Report(error.c_str());
  g->stream_payload_consumed = g->ext_events.Register(
      ext::kStreamLiquidPayloadConsumed,
      static_cast<int32_t>(sizeof(ext::LiquidPayloadConsumed)), &error);
  if (g->stream_payload_consumed < 0) Report(error.c_str());
}


__declspec(dllexport) void SIM_Initialize(GameMessageHandler callback) {
  StopWorker();
  delete g;
  g = new Sim();
  g->callback = callback;
  // The first-party extension properties register in World's constructor, which has no
  // logger. This is the first moment there is one. Empty on every healthy build; a typo in
  // `kThermalMassBonusProperty` or a mistake in ext_registry.h's validator says so here
  // rather than presenting later as a conduction kernel reading a null array.
  if (!g->world.ExtRegistrationError().empty()) {
    Report(g->world.ExtRegistrationError().c_str());
  }
  RegisterFirstPartyStreams();
  StartWorker();
}

__declspec(dllexport) void SIM_Shutdown() {
  // Before `delete g`, not after: the worker dereferences it on every frame.
  StopWorker();
  delete g;
  g = nullptr;
}

// The deferred messages, split out of the dispatch below for the sim thread's sake. These
// change the world and must land on a frame boundary, so all they do here is copy bytes onto
// `g->queue` — which belongs to the game thread alone. The worker reads `g->active`, and the
// rotation between the two happens at `PrepareGameData` with the worker stopped.
//
// Everything *else* touches state the worker owns and has to wait for it, and waiting on each
// of the thousands of these the game sends per frame would give back exactly the overlap
// threading buys. That is why the split exists at all.
bool QueueDeferredMessage(int sim_msg_id, size_t len, const uint8_t* msg) {
  // Same reasoning as KnownMessage: these are cell-scoped messages like SetInsulationValue,
  // so they defer to the worker thread the same way.
  //
  // Queued extension ids fall through into the same push_back the vanilla cell messages use
  // below; returning "queued" before reaching it would silently drop them (vftest's
  // real-pipeline test injects 10 kg of O2 and reads it back).
  // GENERATED FROM `ONI_EXT_MESSAGE_LIST`'s `delivery` COLUMN, so which ids are deferred and
  // what `SIM_ExtMessageDescribe` publishes to mods come from one definition.
  const bool ext_message = ext::IsQueuedExtMessage(sim_msg_id);
  if (!ext_message)
  switch (static_cast<SimMessageHash>(sim_msg_id)) {
    case SimMessageHash::ModifyCell:
    case SimMessageHash::Dig:
    case SimMessageHash::MassEmission:
    case SimMessageHash::MassConsumption:
    case SimMessageHash::SetInsulationValue:
    case SimMessageHash::SetStrengthValue:
    case SimMessageHash::ModifyCellEnergy:
    case SimMessageHash::ChangeCellProperties:
    case SimMessageHash::ConsumeDisease:
    case SimMessageHash::CellDiseaseModification:
    case SimMessageHash::CellRadiationModification:
    case SimMessageHash::ModifyBackwallData:
    // The building components. `SetDebugProperties` is in this list because it carries the
    // two scales the components multiply every transfer by, and the game sends it once per
    // frame — 3,819 times in one recorded session.
    case SimMessageHash::SetDebugProperties:
    case SimMessageHash::AddBuildingHeatExchange:
    case SimMessageHash::ModifyBuildingHeatExchange:
    case SimMessageHash::ModifyBuildingEnergy:
    case SimMessageHash::RemoveBuildingHeatExchange:
    case SimMessageHash::AddBuildingToBuildingHeatExchange:
    case SimMessageHash::AddInContactBuildingToBuildingToBuildingHeatExchange:
    case SimMessageHash::RemoveBuildingInContactFromBuildingToBuildingHeatExchange:
    case SimMessageHash::RemoveBuildingToBuildingHeatExchange:
    // Element chunks: matter the game holds outside the grid, and the busiest thing it
    // sends after the per-frame messages — 12,194 calls over these five hashes in a
    // 38-second capture.
    case SimMessageHash::AddElementChunk:
    case SimMessageHash::MoveElementChunk:
    case SimMessageHash::SetElementChunkData:
    case SimMessageHash::ModifyElementChunkEnergy:
    case SimMessageHash::ModifyChunkTemperatureAdjuster:
    case SimMessageHash::RemoveElementChunk:
    // The two element components. Third and fourth busiest component groups in the census,
    // and the pair that moves matter between a building and the grid.
    case SimMessageHash::AddElementConsumer:
    case SimMessageHash::SetElementConsumerData:
    case SimMessageHash::RemoveElementConsumer:
    case SimMessageHash::AddElementEmitter:
    case SimMessageHash::ModifyElementEmitter:
    case SimMessageHash::RemoveElementEmitter:
    case SimMessageHash::AddDiseaseEmitter:
    case SimMessageHash::ModifyDiseaseEmitter:
    case SimMessageHash::RemoveDiseaseEmitter:
    // The radiation emitter, third in the component list. `RadiationParamsModification` is
    // in this list rather than with the cell messages because it *is* one: the same
    // function drains it, straight after the per-cell rads changes.
    case SimMessageHash::AddRadiationEmitter:
    case SimMessageHash::ModifyRadiationEmitter:
    case SimMessageHash::RemoveRadiationEmitter:
    case SimMessageHash::RadiationParamsModification:
      break;

    default:
      return false;
  }
  Sim::Pending p;
  p.id = sim_msg_id;
  if (msg != nullptr && len != 0) p.payload.assign(msg, msg + len);
  g->queue.push_back(std::move(p));
  return true;
}

// ---------------------------------------------------------------------------------------
// `ext::kSetTunable` (abi/sim_abi_ext.h has the contract; tunables.h the table).
// ---------------------------------------------------------------------------------------

// One row's own rules: the bits' shape, finiteness, and the row's [min, max]. 0 when the value
// passes, else the `ExtRefusalReason`.
int32_t CheckTunableRow(int32_t id, uint64_t bits) {
  const TunableInfo& info = kTunableInfo[id];
  using tunables_detail::Bits;
  switch (info.type) {
    case kTunableF32: {
      if ((bits >> 32) != 0) return ext::kExtRefusalReservedBits;
      const float v = Bits<float>::From(bits);
      if (!std::isfinite(v)) return ext::kExtRefusalNotFinite;
      if (!(v >= Bits<float>::From(info.lo_bits) && v <= Bits<float>::From(info.hi_bits))) {
        return ext::kExtRefusalOutOfRange;
      }
      return 0;
    }
    case kTunableI32: {
      if ((bits >> 32) != 0) return ext::kExtRefusalReservedBits;
      const int32_t v = Bits<int32_t>::From(bits);
      if (v < Bits<int32_t>::From(info.lo_bits) || v > Bits<int32_t>::From(info.hi_bits)) {
        return ext::kExtRefusalOutOfRange;
      }
      return 0;
    }
    case kTunableF64: {
      const double v = Bits<double>::From(bits);
      if (!std::isfinite(v)) return ext::kExtRefusalNotFinite;
      if (!(v >= Bits<double>::From(info.lo_bits) && v <= Bits<double>::From(info.hi_bits))) {
        return ext::kExtRefusalOutOfRange;
      }
      return 0;
    }
  }
  return ext::kExtRefusalBadTarget;
}

// The rules between rows, which no single row's [min, max] can state. Checked against the
// whole candidate table, so it does not matter which of a pair is sent first -- only that the
// table is never left breaking one.
const char* TunableCrossFieldViolation(const Tunables& t) {
  // `World::StableTicksRemaining`: the countdown roll is `(int)(r * scale) + base` for a 15-bit
  // `r`, stored in a 5-bit field whose 31 means "roll again". Evaluated exactly as the kernel
  // does it, at the largest `r`.
  const int32_t roll =
      static_cast<int32_t>(32767.0f * t.stable_ticks_reroll_scale) + t.stable_ticks_reroll_base;
  if (roll >= static_cast<int32_t>(World::kStableTicksReroll)) {
    return "StableTicksRerollBase + (int)(32767 * StableTicksRerollScale) must stay under 31, "
           "the countdown field's roll-again sentinel";
  }
  if (!(t.conduction_min_temperature <= t.conduction_max_temperature)) {
    return "ConductionMinTemperature must not exceed ConductionMaxTemperature";
  }
  if (!(t.min_radiation <= t.max_radiation)) {
    return "MinRadiation must not exceed MaxRadiation";
  }
  return nullptr;
}

// Row flags as the descriptor publishes them (OniExtTunableDesc.flags).
int32_t TunableFlags(int32_t id) {
  int32_t flags = 0;
  if (kTunableInfo[id].hi_bits == kTunableInfo[id].stock_bits) flags |= ONI_TUNABLE_FLAG_CEILING;
  if (id == static_cast<int32_t>(TunableId::SubstepSeconds)) flags |= ONI_TUNABLE_FLAG_NO_WORLD;
  return flags;
}

// Returns 0 (applied) or the refusal reason, and reports a refusal both ways.
int32_t ApplySetTunable(const ext::SetTunableMessage& m, int32_t payload_bytes) {
  char line[320];
  auto refuse = [&](int32_t reason, const char* why) {
    EmitRefusal(ext::kSetTunable, static_cast<ext::ExtRefusalReason>(reason), payload_bytes, 0);
    const char* name = (m.tunableId >= 0 && m.tunableId < kTunableCount)
                           ? kTunableInfo[m.tunableId].name
                           : (m.tunableId == ext::kTunableResetAll ? "reset" : "?");
    snprintf(line, sizeof(line), "kSetTunable: %s (id %d) refused: %s", name, m.tunableId, why);
    Report(line);
    return reason;
  };
  if (m.reserved != 0) return refuse(ext::kExtRefusalReservedBits, "reserved is not 0");
  const bool world = g->world.Allocated();
  const int32_t substep = static_cast<int32_t>(TunableId::SubstepSeconds);
  uint64_t live_substep = 0, want_substep = 0;
  TunableGetBits(g_tunables, substep, &live_substep);

  Tunables candidate = g_tunables;
  if (m.tunableId == ext::kTunableResetAll) {
    candidate = kTunableDefaults;
  } else {
    if (m.tunableId < 0 || m.tunableId >= kTunableCount) {
      return refuse(ext::kExtRefusalBadTarget, "no such row");
    }
    const int32_t own = CheckTunableRow(m.tunableId, m.valueBits);
    if (own == ext::kExtRefusalReservedBits) {
      return refuse(own, "a 32-bit row with its high 32 bits set");
    }
    if (own == ext::kExtRefusalNotFinite) return refuse(own, "not finite");
    if (own == ext::kExtRefusalOutOfRange) {
      return refuse(own, (TunableFlags(m.tunableId) & ONI_TUNABLE_FLAG_CEILING) != 0
                             ? "outside [min, max]; a CEILING may be lowered, never raised"
                             : "outside [min, max]");
    }
    if (own != 0) return refuse(own, "not a value this row can hold");
    TunableSetBits(candidate, m.tunableId, m.valueBits);
  }
  // Decision 1: the substep is the clock contract with the managed game, fixed while a world
  // exists. Re-sending the value it already has is not a change and is accepted, so a config
  // replayed after a load does not fail on it.
  TunableGetBits(candidate, substep, &want_substep);
  if (world && want_substep != live_substep) {
    return refuse(ext::kExtRefusalWorldLoaded,
                  "SubstepSeconds changes only while no world is allocated");
  }
  if (const char* why = TunableCrossFieldViolation(candidate)) {
    return refuse(ext::kExtRefusalCrossField, why);
  }
  g_tunables = candidate;
  return 0;
}

void* HandleImmediateMessage(int sim_msg_id, size_t len, const uint8_t* msg) {
  // Framework extension (abi/sim_abi_ext.h), handled before the switch for the same reason
  // KnownMessage checks the extension ids before its own: they live outside SimMessageHash's
  // numbering entirely and folding them in as fake enumerators would put project-invented
  // values into Klei's ABI enum.
  //
  // ONE OF THE THREE IMMEDIATE EXTENSION MESSAGES (stage 5 added the two element-attribute
  // ones below). Registration must close before the first `World::Allocate`, and both
  // `AllocateCells` and `SimData_InitializeFromCells` are immediate; a queued registration
  // would drain a frame after the world it was meant to size. So this is applied on the
  // calling thread, which `SIM_HandleMessage` has already made safe with `WaitIdle()`.
  //
  // Returns the property index, or a negated ExtRegisterResult. Static storage because the
  // return crosses the DLL boundary as a `void*` and the caller reads it before the next
  // message — the same lifetime every other pointer this function returns has.
  if (sim_msg_id == ext::kRegisterCellProperty) {
    static int32_t result = 0;
    ext::RegisterCellPropertyMessage m{};
    if (!PayloadBytes(msg, len, &m)) {
      result = -static_cast<int32_t>(ext::kExtRegisterBadName);
      Report("kRegisterCellProperty: payload too short");
      return &result;
    }
    // A name that fills all 48 bytes with no terminator is REFUSED, not truncated. Truncating
    // would register a different name than the caller asked for, and the name is the
    // permanent on-disk key for that property's data — so the quiet repair would show up
    // later as a save whose section nobody claims. This is also the only way the ">47
    // characters" rule is reachable over the wire, since a longer name cannot be transmitted
    // in the field at all.
    if (m.name[sizeof(m.name) - 1] != '\0') {
      result = -static_cast<int32_t>(ext::kExtRegisterBadName);
      Report("kRegisterCellProperty: name field is not NUL-terminated (a property name is at "
             "most 47 characters); refused rather than truncated, because the name is the "
             "on-disk key");
      return &result;
    }
    std::string error;
    result = g->world.ExtCells().Register(m.name, m.type, m.persist, m.arity, m.defaultBits,
                                          /*first_party=*/false, &error);
    if (result < 0) Report(error.c_str());
    return &result;
  }

  // abi/sim_abi_ext.h, kRegisterField. Attach a propagation rule to a cell
  // property that is already registered. Immediate for the same reason kRegisterCellProperty
  // is: the caller needs the index back to send it a single source, and a registration that
  // drained a frame later would hand every caller a sequencing problem that has no good
  // answer.
  //
  // It takes a property INDEX rather than a name because the property must already exist --
  // a field is not a new kind of storage, it is a solver bolted onto storage somebody else
  // registered, and the index is what that registration returned. This is also why there is
  // no name-collision check here: the property registry already refused the duplicate name
  // before this message could be sent.
  //
  // Returns the field index, or a negated ExtRegisterResult, in static storage -- same
  // lifetime rule as the two registrations above.
  if (sim_msg_id == ext::kRegisterField) {
    static int32_t result = 0;
    ext::RegisterFieldMessage m{};
    if (!PayloadBytes(msg, len, &m)) {
      result = -static_cast<int32_t>(ext::kExtRegisterBadRule);
      Report("kRegisterField: payload too short");
      return &result;
    }
    std::string error;
    result = g->fields.registry.Register(g->world.ExtCells(), g->elements.Attributes(), m,
                                         &error);
    if (result < 0) Report(error.c_str());
    return &result;
  }

  // The EIGHTH checkpoint component: the whole radiation field, which `Load` does not give back
  // in Vacuum and Void cells (World::ReadLoadedCell). Immediate rather than queued because it is
  // the one component the game reads back in GameDataUpdate, and the first PrepareGameData
  // after a Load publishes before a queued message drains (abi/sim_abi_ext.h). The caller
  // already waited for the worker (SIM_HandleMessage), so the world is ours to write.
  if (sim_msg_id == ext::kSetCellRadiation) {
    ext::SetCellRadiationMessage m{};
    if (!PayloadBytes(msg, len, &m)) {
      EmitRefusal(sim_msg_id, ext::kExtRefusalShortPayload, static_cast<int32_t>(len),
                  static_cast<int32_t>(sizeof(m)));
      Report("kSetCellRadiation: payload too short");
      return nullptr;
    }
    // The rest is logged rather than refused through the event stream, as kSetDiseaseGrowth
    // logs its own: the refusal codes are a published list, and these are not on it.
    if (m.count <= 0 || len != sizeof(m) + static_cast<size_t>(m.count) * sizeof(float)) {
      Report("kSetCellRadiation: payload length does not match its own count");
      return nullptr;
    }
    const size_t count = static_cast<size_t>(m.count);
    // Copied out rather than cast in place: nothing promises the float lane's alignment.
    std::vector<float> values(count);
    memcpy(values.data(), msg + sizeof(m), count * sizeof(float));
    if (!g->world.Allocated() || !g->world.SetRadiationField(values.data(), count)) {
      Report("kSetCellRadiation: count is not this world's padded cell count");
    }
    return nullptr;
  }

  // The NINTH checkpoint component: the visibility mask, all three buffers. Every allocate, load
  // and Start zeroes them (ResetVisibility), as Klei's new world starts with zeroed ones.
  // Immediate because the next PrepareGameData writes one of these buffers before a queued
  // message would drain, and the restore would then overwrite the mask that call sent
  // (abi/sim_abi_ext.h). The caller already waited for the worker (SIM_HandleMessage).
  if (sim_msg_id == ext::kSetVisibilityState) {
    ext::SetVisibilityStateMessage m{};
    if (!PayloadBytes(msg, len, &m)) {
      EmitRefusal(sim_msg_id, ext::kExtRefusalShortPayload, static_cast<int32_t>(len),
                  static_cast<int32_t>(sizeof(m)));
      Report("kSetVisibilityState: payload too short");
      return nullptr;
    }
    // Logged rather than refused through the event stream, as kSetCellRadiation logs its own.
    if (m.count <= 0 || (m.simSlot != 0 && m.simSlot != 1) ||
        len != sizeof(m) + 3 * static_cast<size_t>(m.count)) {
      Report("kSetVisibilityState: payload length or slot does not match its own header");
      return nullptr;
    }
    if (!g->world.Allocated() || static_cast<size_t>(m.count) != g->world.GameCount()) {
      Report("kSetVisibilityState: count is not this world's game cell count");
      return nullptr;
    }
    // The frame buffer and the sim-side buffer go in SWAPPED. The getter reads them after the
    // captured frame was published, and publishing ends with that swap (BuildUpdate). The first
    // PrepareGameData after a Load or a Start runs a frame on the calling thread that
    // publishes the restored world again, swap included, so a verbatim restore would swap them
    // once more than the run did -- measured by vftest's arm, where the handed-over water came a
    // frame late until this was done. After that publication the buffers are the captured ones.
    const size_t count = static_cast<size_t>(m.count);
    const uint8_t* bytes = msg + sizeof(m);
    const uint8_t* frame = bytes;
    const uint8_t* game[2] = {bytes + count, bytes + 2 * count};
    const int s = m.simSlot;
    g->visible.assign(game[s], game[s] + count);
    g->visible_game[s].assign(frame, frame + count);
    g->visible_game[1 - s].assign(game[1 - s], game[1 - s] + count);
    g->visible_sim_slot = s;
    return nullptr;
  }

  // The one checkpoint message sent BEFORE its Load: that Load is a restore, so it skips the
  // load-time state transition (abi/sim_abi_ext.h). Immediate because the
  // Load is handled on the calling thread and a queued message would drain after it.
  if (sim_msg_id == ext::kSetLoadIsRestore) {
    ext::SetLoadIsRestoreMessage m{};
    if (!PayloadBytes(msg, len, &m)) {
      EmitRefusal(sim_msg_id, ext::kExtRefusalShortPayload, static_cast<int32_t>(len),
                  static_cast<int32_t>(sizeof(m)));
      Report("kSetLoadIsRestore: payload too short");
      return nullptr;
    }
    if ((m.restore != 0 && m.restore != 1) || m.reserved != 0) {
      Report("kSetLoadIsRestore: restore is not 0 or 1, or reserved is not 0");
      return nullptr;
    }
    g->load_is_restore = m.restore == 1;
    return nullptr;
  }

  // MOVED TO THE IMMEDIATE PATH BY STAGE 5, and the move is a fix rather than a tidy-up.
  // `kSetMolecularMass` writes the same store the two messages below write and the two
  // exports below read -- and those reads take no `WaitIdle()`, on the documented ground that
  // the element table is written on the calling thread and read-only for the rest of the
  // frame. That ground was false for as long as this one message drained on the WORKER: a
  // game-thread read of a molar mass could race a worker-thread write of it. It was only ever
  // in the deferred list because every extension message was cell-scoped when the list was
  // written, and this one is not. Immediate is also strictly better for its caller: the
  // value lands before the call returns instead of a tick later.
  if (sim_msg_id == ext::kSetMolecularMass) {
    ext::SetMolecularMassMessage m{};
    if (!PayloadBytes(msg, len, &m)) {
      EmitRefusal(sim_msg_id, ext::kExtRefusalShortPayload, static_cast<int32_t>(len),
                  static_cast<int32_t>(sizeof(m)));
      Report("kSetMolecularMass: payload too short");
      return nullptr;
    }
    // A pure table edit: no world access, no cell touched, nothing scheduled. The next
    // CellMoles/MolesFromSpeciesList call simply divides by a different number.
    g->elements.SetMolecularMass(m.idHash, m.gPerMol);
    return nullptr;
  }

  // abi/sim_abi_ext.h at kRegisterElementAttribute. Immediate for a
  // different reason than the registration above: an element attribute is CONTENT DATA pushed
  // at load, not world state that has to land on a frame boundary, and a caller that pushes a
  // value and then reads a pressure on the same tick should see the value it pushed.
  // `kSetMolecularMass`'s one-tick deferral is exactly why Mod 1 has to push it "first thing
  // in OnSpawn deliberately"; this path has no such requirement to document.
  //
  // Returning the index, or a negated ExtRegisterResult, in static storage — same lifetime
  // rule as kRegisterCellProperty above.
  if (sim_msg_id == ext::kRegisterElementAttribute) {
    static int32_t result = 0;
    ext::RegisterElementAttributeMessage m{};
    if (!PayloadBytes(msg, len, &m)) {
      result = -static_cast<int32_t>(ext::kExtRegisterBadName);
      Report("kRegisterElementAttribute: payload too short");
      return &result;
    }
    // Refused rather than truncated, for the same reason the cell-property name is: a
    // truncated name registers something other than what the caller asked for, and the name
    // is how every later message and every read-back finds this attribute again.
    if (m.name[sizeof(m.name) - 1] != '\0') {
      result = -static_cast<int32_t>(ext::kExtRegisterBadName);
      Report("kRegisterElementAttribute: name field is not NUL-terminated (an attribute name "
             "is at most 47 characters); refused rather than truncated");
      return &result;
    }
    std::string error;
    result = g->elements.MutableAttributes().Register(m.name, m.type, m.arity,
                                                      /*first_party=*/false, &error);
    if (result < 0) Report(error.c_str());
    return &result;
  }

  // The write. No return value: a refusal here goes to the `sim.message_refused` stream
  // (stage 4) rather than to the caller, because unlike a registration this is sent in bulk
  // at load and a per-message return the caller has to check is a return the caller will not
  // check. A registration refusal stays a return value precisely because there is exactly one
  // of them per attribute and its result is the index everything else needs.
  if (sim_msg_id == ext::kSetElementAttribute) {
    ext::SetElementAttributeMessage m{};
    if (!PayloadBytes(msg, len, &m)) {
      EmitRefusal(sim_msg_id, ext::kExtRefusalShortPayload, static_cast<int32_t>(len),
                  static_cast<int32_t>(sizeof(m)));
      Report("kSetElementAttribute: payload too short");
      return nullptr;
    }
    ext::ElementAttributeRegistry& attrs = g->elements.MutableAttributes();
    const bool ok = m.clear != 0 ? attrs.Clear(m.attrIdx, m.idHash)
                                 : attrs.Write(m.attrIdx, m.idHash, m.component, m.valueBits);
    // A clear of an element that had no entry reports false and is NOT a refusal — the
    // caller asked for it to be unset and it is unset. Only a bad attribute index or a
    // component outside the registered arity is a refusal, so the two are separated here
    // rather than folded into one `if (!ok)`.
    if (!ok && (m.clear == 0 || !attrs.Valid(m.attrIdx))) {
      EmitRefusal(sim_msg_id, ext::kExtRefusalBadTarget, static_cast<int32_t>(len),
                  static_cast<int32_t>(sizeof(m)));
    }
    return nullptr;
  }

  // Tunables. Immediate, and it returns its result -- see `ext::kSetTunable` for why both.
  if (sim_msg_id == ext::kSetTunable) {
    static int32_t result = 0;
    ext::SetTunableMessage m{};
    if (!PayloadBytes(msg, len, &m)) {
      EmitRefusal(sim_msg_id, ext::kExtRefusalShortPayload, static_cast<int32_t>(len),
                  static_cast<int32_t>(sizeof(m)));
      Report("kSetTunable: payload too short");
      result = ext::kExtRefusalShortPayload;
      return &result;
    }
    result = ApplySetTunable(m, static_cast<int32_t>(len));
    return &result;
  }

  switch (static_cast<SimMessageHash>(sim_msg_id)) {
    case SimMessageHash::Elements_CreateTable:
      // msg_length overshoots for this message — the managed side sends the stream's
      // grown capacity, roughly twice the real payload. Parse the count and trust that,
      // never the length.
      if (!g->elements.Load(msg, len)) Report("element table rejected");
      return nullptr;

    case SimMessageHash::Disease_CreateTable:
      if (!g->diseases.Load(msg, len)) Report("disease table rejected");
      return nullptr;

    // `SimData::ResizeAndInitializeVacuumCells`. This is how a world is opened
    // in the grid: a rectangle of Vacuum, walled in by a one-cell ring of Unobtanium, with
    // the backwall of both set to Vacuum.
    //
    // The name only describes the second half. What it actually writes, measured field by
    // field against Klei on `vacrect` — the "240 tonnes" that made the first attempt look
    // wrong is the ring, and it is 24 cells of 9999 kg:
    //
    //   * the **ring**, one cell outside the rectangle on every side: element Unobtanium,
    //     mass a flat **9999 kg** (a constant in the function — Unobtanium's own table row
    //     carries 10000 and 20000 and neither of them lands here), temperature 0, and the
    //     two disease fields cleared. Its **radiation is left alone**;
    //   * the **rectangle**: element Vacuum, mass 0, temperature 0, the two disease fields
    //     cleared, and radiation zeroed as well. Not the infestation counter and not the
    //     growth accumulator, which is what separates this from `ClearCell`;
    //   * the **backwall** over ring and rectangle together: element Vacuum, mass 0,
    //     temperature 0.
    //
    // The radiation split is the load-bearing measurement and it is the one a guess gets
    // wrong: a ring cell keeps its rads and a rectangle cell loses them, which is only
    // visible because `vacrect` now seeds germs and rads in **three** zones — inside, on
    // the ring, and outside as the control.
    //
    // `gridSizeX`/`gridSizeY` are the resize the name promises. The game sends the grid it
    // already has whenever the grid is not actually changing size, which is every case
    // observed so far, so a differing pair is reported rather than acted on.
    case SimMessageHash::SimData_ResizeAndInitializeVacuumCells: {
      int32_t v[6] = {0, 0, 0, 0, 0, 0};
      if (len < sizeof(v)) return nullptr;
      memcpy(v, msg, sizeof(v));
      if (v[0] != g->world.GameWidth() || v[1] != g->world.GameHeight()) {
        Report("ResizeAndInitializeVacuumCells: grid resize is not implemented");
        return nullptr;
      }
      const int32_t rw = v[2], rh = v[3], rx = v[4], ry = v[5];
      if (rw <= 0 || rh <= 0) return nullptr;
      const uint16_t vacuum = g->elements.VacuumIndex();
      const uint16_t border = g->elements.UnobtaniumIndex();
      constexpr int32_t kVacuumHash = 758759285;
      const int32_t vacuum_hash = g->elements.HasHash(kVacuumHash) ? kVacuumHash : 0;

      auto each = [&](int32_t x0, int32_t y0, int32_t x1, int32_t y1, auto&& fn) {
        for (int32_t y = y0; y <= y1; ++y) {
          if (y < 0 || y >= g->world.GameHeight()) continue;
          for (int32_t x = x0; x <= x1; ++x) {
            if (x < 0 || x >= g->world.GameWidth()) continue;
            const size_t game = static_cast<size_t>(y) * g->world.GameWidth() +
                                static_cast<size_t>(x);
            fn(g->world.Padded(game), game);
          }
        }
      };

      // The ring and the rectangle in one walk, because the ring write is the rectangle
      // write with a different element and without the radiation clear — Klei fills the
      // whole expanded rectangle with the border and then clears the inside of it.
      each(rx - 1, ry - 1, rx + rw, ry + rh, [&](size_t cell, size_t game) {
        g->world.MarkProjectDirty(cell);
        g->world.TouchSubstance(cell);
        // Both ledgers: a wholesale overwrite, so whatever the cell held leaves the sim
        // here and 9999 kg of border arrives in its place. Unobtanium carries
        // specificHeatCapacity 0, so the energy "after" side is exactly zero and that charge
        // is minus the cell's whole energy -- while the mass charge is strongly positive,
        // the two pulling opposite ways at the same write.
        const double init_before = GridCellEnergy(g->world, g->elements, cell);
        const float init_mass_before = g->world.Phase(cell).mass;
        PhaseEntry& p = g->world.Phase(cell);
        g->world.PayloadZero(cell);
        p.element = border;
        p.mass = g_tunables.world_border_mass_kg;
        p.temperature = g_tunables.world_border_temperature_k;
        g->world.NoteWorldInitEnergy(GridCellEnergy(g->world, g->elements, cell) - init_before);
        g->world.NoteWorldInit(p.mass - init_mass_before);
        g->world.MutableDiseaseIdx(cell) = 0xFF;
        SaveDisease& d = g->world.MutableDisease(cell);
        d.diseaseHash = 0;
        d.count = 0;
        SaveBackwall& b = g->world.MutableBackwall(cell);
        b.elementHash = vacuum_hash;
        b.mass = 0.0f;
        b.temperature = 0.0f;
        // The projection's memory of the cell moves with the cell. `Project` announces a
        // `solidInfo` when a cell's solidity differs from `buffers.previous`, and the ring
        // turns twenty-four gas cells into a solid border — yet Klei announces **none** of
        // them, on this frame or any later one. So the message updates the published side
        // as well as the live grid, and the ring is solid and massive in both before the
        // next projection ever looks at it.
        //
        // Only `previous` is written, not `buffers.element`: the substance announcements
        // *are* published for these cells (78 of them, matching Klei cell for cell), and
        // they are driven by the element diff against that buffer. Solidity is the one
        // memory the message resets.
        if (game < g->buffers.previous.size()) {
          g->buffers.previous[game] = static_cast<uint8_t>(kPrevSolid | kPrevHadMass);
        }
      });
      each(rx, ry, rx + rw - 1, ry + rh - 1, [&](size_t cell, size_t game) {
        const double init_before = GridCellEnergy(g->world, g->elements, cell);
        const float init_mass_before = g->world.Phase(cell).mass;
        PhaseEntry& p = g->world.Phase(cell);
        g->world.PayloadZero(cell);
        p.element = vacuum;
        p.mass = 0.0f;
        p.temperature = 0.0f;
        g->world.NoteWorldInitEnergy(GridCellEnergy(g->world, g->elements, cell) - init_before);
        // Not routed through `ClearCell`: this is an init, not a physics clear, and it does
        // not touch the infestation counter or the growth accumulator that `ClearCell` does.
        // Charging it to `cleared` would put a world-open under a physics bucket.
        g->world.NoteWorldInit(-init_mass_before);
        g->world.SetRadiation(cell, 0.0f);
        // Same reset, the other way up: the cleared rectangle is neither solid nor massive.
        if (game < g->buffers.previous.size()) g->buffers.previous[game] = 0;
      });
      // The rectangle is not only cleared, it is *registered*: from the next published frame
      // the sunlight sweep runs over it and it reads back fully lit, and a solid dropped into
      // it later shadows the column below itself. See `World::AddWorldOffset`.
      g->world.AddWorldOffset(World::WorldOffset{rx, ry, rw, rh});
      return nullptr;
    }

    case SimMessageHash::DefineWorldOffsets: {
      // `DefineWorldOffsets`: an int32 count, then four int32s per entry into
      // a 16-byte `SimData::WorldOffsetData`. The sim keeps them for one reason we know of —
      // the sunlight texture is computed per world, and stays zero until this arrives.
      int32_t count = 0;
      if (len >= 4) memcpy(&count, msg, 4);
      std::vector<World::WorldOffset> v;
      if (count > 0 && len >= static_cast<size_t>(4 + count * 16)) {
        v.resize(static_cast<size_t>(count));
        for (int32_t i = 0; i < count; ++i) {
          const uint8_t* p = msg + 4 + static_cast<size_t>(i) * 16;
          memcpy(&v[static_cast<size_t>(i)].x, p, 4);
          memcpy(&v[static_cast<size_t>(i)].y, p + 4, 4);
          memcpy(&v[static_cast<size_t>(i)].w, p + 8, 4);
          memcpy(&v[static_cast<size_t>(i)].h, p + 12, 4);
        }
      }
      g->world.SetWorldOffsets(std::move(v));
      return nullptr;
    }

    case SimMessageHash::AllocateCells: {
      int32_t w = 0, h = 0;
      if (len >= 8) {
        memcpy(&w, msg, 4);
        memcpy(&h, msg + 4, 4);
      }
      if (w > 0 && h > 0) {
        g->world.Allocate(w, h);
        g->buffers.Allocate(g->world.GameCount());
        g->textures.Allocate(g->world.GameCount());
        g->first_frame = true;
        g->pipelined = false;
        ResetVisibility();
        g->game_frame = nullptr;
        g->game_frame_cells = 0;
        g->skip_physics_frames = 1;
        g->first_physics_substep = true;
        g->displace_rotation = 0;
        g->pressure_dir = -1;
        // This path seeds the random stream from the wall clock: `AllocateCells` passes
        // `_time64(NULL)` as the constructor's seed argument, where `InitializeFromCells`
        // passes the world seed. A save loaded twice therefore shuffles its gas
        // differently, which is Klei's behaviour and not something to "fix".
        g->world.SetRandomState(static_cast<uint32_t>(time(nullptr)));
        g->world.SetShuffleDir(-1);
        // An Alloc builds a fresh SimData in Klei's sim, and the components live in it. Every
        // building handle the game holds is stale after this and the game re-registers them.
        g->buildings.Clear();
        g->chunks.Clear();
        g->flow.Clear();
        g->radiation.Clear();
        // A fresh SimData has nothing in the new layer
        // either (Allocate above already zeroed it). A stale `true` here from a *previous*
        // world would otherwise leave StepPhysics's mixing block running TickRoomPooledMixing
        // against a room graph sized for dimensions this Allocate may have just changed.
        g->vf_active = false;
      }
      return nullptr;
    }

    case SimMessageHash::SimData_InitializeFromCells:
      if (!g->world.InitializeFromCells(msg, len, g->elements, g->diseases)) {
        Report("SimData_InitializeFromCells rejected");
        return nullptr;
      }
      g->buffers.Allocate(g->world.GameCount());
      g->textures.Allocate(g->world.GameCount());
      g->first_frame = true;
      g->pipelined = false;
      ResetVisibility();
      g->game_frame = nullptr;
      g->game_frame_cells = 0;
      g->skip_physics_frames = 1;
      g->first_physics_substep = true;
      // Same reasoning as AllocateCells above — InitializeFromCells calls Allocate too.
      g->vf_active = false;
      return nullptr;

    case SimMessageHash::Load: {
      // ext::kSetLoadIsRestore is one-shot: this Load consumes it, including on the paths below
      // that reject the blob, so a flag armed for a restore that failed never reaches a save.
      const bool is_restore = g->load_is_restore;
      g->load_is_restore = false;
      SaveBlob blob;
      std::string error;
      if (!DecodeSaveBlob(msg, len, &blob, &error)) {
        Report(("save blob rejected: " + error).c_str());
        return nullptr;  // the game turns a null return into a load failure
      }
      // A blob that covers the whole allocation at (0,0) is a saved game: reload through
      // `FromBlob`, as always. A smaller one is one world of a fresh cluster and goes down at
      // its own header `x, y` (`World::LoadIntoCluster`). One that does not fit falls back to
      // `FromBlob`; what Klei does there is not measured.
      const bool whole = !g->world.Allocated() ||
                         (blob.x == 0 && blob.y == 0 &&
                          blob.GameWidth() == g->world.GameWidth() &&
                          blob.GameHeight() == g->world.GameHeight());
      const bool fits = g->world.Allocated() && blob.x >= 0 && blob.y >= 0 &&
                        blob.x + blob.GameWidth() <= g->world.GameWidth() &&
                        blob.y + blob.GameHeight() <= g->world.GameHeight();
      const bool ok = (!whole && fits)
                          ? g->world.LoadIntoCluster(blob, g->elements, g->diseases, &error)
                          : g->world.FromBlob(blob, g->elements, g->diseases, &error);
      if (!ok) {
        Report(("save blob rejected: " + error).c_str());
        return nullptr;
      }
      // Klei's step 6, `DoLoadTimeStateTransition`, as a pass over what was just written: it
      // has to pass the mods' phase rules, and the boil rule reads the column above a cell.
      // After the mixture layer is back, because a mixture-held Vacuum cell is exempt. See
      // `LoadTimeStateTransitions`, sim/physics.h.
      //
      // NOT ON A CHECKPOINT RESTORE (ext::kSetLoadIsRestore). A checkpoint is a frame of a
      // run in progress, and a run publishes cells out of range -- a supercooled drop that landed
      // this substep freezes on the next -- so pushing them through this step starts the replay
      // from a state the run never held.
      if (!is_restore) LoadTimeStateTransitions(&g->world, g->elements);
      g->buffers.Allocate(g->world.GameCount());
      g->textures.Allocate(g->world.GameCount());
      g->first_frame = true;
      g->pipelined = false;
      ResetVisibility();
      g->game_frame = nullptr;
      g->game_frame_cells = 0;
      g->skip_physics_frames = 1;
      g->first_physics_substep = true;
      // FromBlob already restored gas_species_/gas_mass_/
      // gas_occupied_mask_ if the blob carried them; this is what turns that restored data
      // back into an active mixing pass, the same way ApplyInjectGasSpecies activates a
      // fresh world's first message. A legacy blob (no gas section, the common case) leaves
      // HasGasMixtureData false, so this stays the plain reset — no stale room graph from
      // whatever world was loaded before this one.
      //
      // Room promotion survives save/load: also activate on
      // HasAnyRoomPromoted alone, because a room can be promoted with its own gas-mixture mass
      // already fully consumed/vented by save time (HasGasMixtureData false) while
      // room_promoted_ (just restored by FromBlob, above) still says otherwise. Either way,
      // ActivateVolumeFractions's BuildRoomGraph call re-derives RoomGraph.owned straight from
      // World::RoomPromoted (gas_rooms.h) — this is the only place that restored bit needs to
      // reach the live room graph.
      if (g->world.LoadDroppedSpeciesSlots() > 0) {
        // Not a failure: the element is gone from this table, and Klei drops a cell whose
        // element is unknown too (World::RemapSpecies). Said out loud because it is mass.
        char note[160];
        snprintf(note, sizeof(note),
                 "load dropped %d gas-mixture species slot(s), %.6f kg, whose element is not in "
                 "this element table",
                 g->world.LoadDroppedSpeciesSlots(), g->world.LoadDroppedSpeciesKg());
        Report(note);
      }
      if (g->world.LoadDroppedElementIndices() > 0) {
        // Same cause, in a registered or orphaned kExtElementIdx property: each such component
        // now holds kExtNoElement. Not a failure either.
        char note[160];
        snprintf(note, sizeof(note),
                 "load set %d saved element-index component(s) to no element: their element is "
                 "not in this element table",
                 g->world.LoadDroppedElementIndices());
        Report(note);
      }
      if (g->world.HasGasMixtureData() || g->world.HasAnyRoomPromoted()) {
        ActivateVolumeFractions();
      } else {
        g->vf_active = false;
      }
      // Non-null means accepted. The game only tests for null, and there is no frame to
      // hand it — this is the empty publication the next `PrepareGameData` will overwrite.
      return &g->pub[g->pub_slot].update;
    }

    case SimMessageHash::ClearUnoccupiedCells: {
      // A fresh cluster sends this before its per-world `Load`s, and the gaps between worlds
      // are never written by any of them. Klei leaves a gap cell as Vacuum with no mass or
      // heat, backwall included (driver/src/worldgen_test against Klei's DLL).
      // Allocate's zeroed cells would otherwise be element index 0, which is a real element.
      if (!g->world.Allocated()) return nullptr;
      const uint16_t vacuum = g->elements.VacuumIndex();
      const size_t n = g->world.PaddedCount();
      for (size_t i = 0; i < n; ++i) {
        g->world.Phase(i) = {vacuum, 0.0f, 0.0f};
        g->world.SetRadiation(i, 0.0f);
        g->world.MutableBackwall(i) = {World::kVacuumHash, 0.0f, 0.0f};
      }
      return nullptr;
    }

    case SimMessageHash::Start: {
      if (!g->world.Allocated()) {
        Report("Start before the world was allocated");
        return nullptr;
      }
      g->started = true;
      ClearFrameEvents();
      Project(g->world, g->elements, &g->buffers, &g->projection_events, true);
      // The property textures are deliberately *not* filled here. Klei's are still all
      // zero after Start and only get populated by PrepareGameData — filling them early
      // showed up as a tick-0 divergence in every liquid cell.
      //
      // The backwall is the same story and was found the same way, once a scenario finally
      // sent a non-empty `SimBackwall`: Klei's Start update carries 0xFFFF for every
      // backwall element and zero mass and temperature, and only the first real frame fills
      // them in. Undoing the copy `Project` just made is cheaper than teaching it a mode,
      // and the dirty-all makes frame 1 redo it.
      g->buffers.backwall_element.assign(g->buffers.backwall_element.size(), 0xFFFF);
      g->buffers.backwall_mass.assign(g->buffers.backwall_mass.size(), 0.0f);
      g->buffers.backwall_temperature.assign(g->buffers.backwall_temperature.size(), 0.0f);
      g->world.MarkStaticDirtyAll();
      g->first_frame = false;
      // Start publishes on this thread and puts nothing in flight, so the first
      // `PrepareGameData` after it primes the pipeline.
      g->pipelined = false;
      ResetVisibility();
      GameDataUpdate* started = BuildUpdate(0);
      g->game_frame = started;
      g->game_frame_cells = started ? g->world.GameCount() : 0;
      return started;
    }

    case SimMessageHash::SimFrameManager_NewGameFrame: {
      // The payload is `sizeof(NewGameFrame) * activeRegions.Count`, not one struct: the
      // game sends one region per active area and a multi-asteroid world sends several.
      // Reading only the first would leave every asteroid but one frozen.
      const size_t count = len / sizeof(NewGameFrame);
      if (count == 0) return nullptr;
      std::vector<int32_t> regions;
      std::vector<float> cosmic;
      std::vector<float> sunlight;
      regions.reserve(count * 4);
      cosmic.reserve(count);
      sunlight.reserve(count);
      for (size_t i = 0; i < count; ++i) {
        NewGameFrame f{};
        memcpy(&f, msg + i * sizeof(NewGameFrame), sizeof(f));
        if (i == 0) g->elapsed_seconds = f.elapsedSeconds;
        regions.push_back(f.minX);
        regions.push_back(f.minY);
        regions.push_back(f.maxX);
        regions.push_back(f.maxY);
        // The sixth int of Klei's `activeRegions` record. Ignored until the radiation field
        // went in, because nothing read it.
        cosmic.push_back(f.currentCosmicRadiationIntensity);
        // The fifth int, and until Mod 3 nothing read it either -- see World::RegionSunlight.
        sunlight.push_back(f.currentSunlightIntensity);
      }
      g->world.SetActiveRegions(regions.data(), count, cosmic.data(), sunlight.data());
      return nullptr;
    }

    case SimMessageHash::PrepareGameData: {
      // The pipeline. `docs/THREADING.md`: the frame handed back here is the one the worker
      // started at the *previous* `PrepareGameData` — which is Klei's arrangement, and is why
      // the rotation below sits on this side of the boundary rather than inside the frame.
      // The first call after a Start, an Alloc or a Load has nothing in flight and runs its
      // frame here, on this thread, exactly as the whole sim used to.
      GameDataUpdate* out = nullptr;
      if (g_worker.enabled && g->pipelined) {
        out = g->last_published;
      } else {
        ReleaseConduitHandlesForNextFrame();
        out = RunFrame();
        g->pipelined = true;
      }
      // The payload is the game's visibility mask, one byte per game cell. It is copied into
      // the game-side buffer AFTER the frame, as `GameSync` does once the sim thread has
      // finished one, and the pair then trades places. See `Sim::visible`.
      if (msg != nullptr && len > 0) {
        std::vector<uint8_t>& dst = g->visible_game[1 - g->visible_sim_slot];
        dst.assign(msg, msg + static_cast<size_t>(len));
        g->visible_sim_slot = 1 - g->visible_sim_slot;
      }
      if (out == nullptr) return nullptr;
      g->game_frame = out;
      g->game_frame_cells = g->world.GameCount();
      // A message sent during this tick takes effect one frame later: the next frame drains
      // `active`, and `active` is what `queue` held when the frame before it was collected.
      // Same rotation as before, one thread further out.
      g->active.swap(g->queue);
      g->queue.clear();
      if (g_worker.enabled) {
        ReleaseConduitHandlesForNextFrame();
        Kick();
      }
      return out;
    }

    // Immediate, not queued: it changes nothing about the world, and a toggle that only took
    // effect on the next frame would time a frame the player did not ask for.
    case SimMessageHash::ToggleProfiler:
      ToggleProfiler();
      return nullptr;

    default:
      NoteUnknown(sim_msg_id, len);
      return nullptr;
  }
}

__declspec(dllexport) void* SIM_HandleMessage(int sim_msg_id, int msg_length,
                                              const uint8_t* msg) {
  if (!g) return nullptr;
  const size_t len = msg_length > 0 ? static_cast<size_t>(msg_length) : 0;
  // kSetBuildingConvection's conduit-side half, and the one message with an effect on this
  // thread. The conduit kernel runs on the game thread between frames (see the
  // ConduitTemperatureManager exports), so a record it reads has to be written here rather than
  // by the worker's drain; the building field itself is queued like every other parameter. See
  // abi/sim_abi_ext.h for why a radiator's conduit must not apply the cell ratio twice.
  if (sim_msg_id == ext::kSetBuildingConvection && msg != nullptr &&
      len >= sizeof(ext::SetBuildingConvectionMessage)) {
    ext::SetBuildingConvectionMessage m{};
    memcpy(&m, msg, sizeof(m));
    g->conduits.SetStructureConvection(m.handle, m.convectionFactor > 0.0f);
  }
  // The fast path, and the only one that must not block: it touches the game thread's own
  // queue and nothing else.
  if (QueueDeferredMessage(sim_msg_id, len, msg)) return nullptr;
  WaitIdle();
  return HandleImmediateMessage(sim_msg_id, len, msg);
}

__declspec(dllexport) void* SIM_HandleMessages(int sim_msg_id, int msg_length,
                                               int msg_count, const uint8_t* msg) {
  if (!g || msg_count <= 0) return nullptr;
  void* last = nullptr;
  for (int i = 0; i < msg_count; ++i) {
    last = SIM_HandleMessage(sim_msg_id, msg_length,
                             msg + static_cast<size_t>(i) * msg_length);
  }
  return last;
}

__declspec(dllexport) uint8_t* SIM_BeginSave(int* size, int x, int y) {
  WaitIdle();
  if (!g || !g->world.Allocated()) {
    if (size) *size = 0;
    return nullptr;
  }
  g->save_blob = EncodeSaveBlob(g->world.ToBlob(x, y, g->elements));
  if (size) *size = static_cast<int>(g->save_blob.size());
  return g->save_blob.data();
}

__declspec(dllexport) void SIM_EndSave() {
  if (g) g->save_blob.clear();
}

// Not part of Klei's ABI — the game never calls it. `diffsim` uses it to read the random
// stream's position out of both sims and compare draw counts directly, which is the only
// way to tell "we take the wrong number of draws" apart from "we take the right number in
// the wrong places".
__declspec(dllexport) uint32_t SIM_DebugRandomState() {
  WaitIdle();
  return g ? g->world.RandomState() : 0u;
}

// Also not part of Klei's ABI, and the getter half of `ext::kSetSchedulingState`. Fills the
// same struct the message takes, so a caller reads it here, keeps it beside the save blob
// and the random state, and sends all three back to resume a run exactly where it was.
// Returns false and touches nothing when there is no sim or no destination, so a caller on
// a stock SimDLL -- where this symbol does not exist at all -- and a caller on a
// not-yet-initialised one behave the same way.
__declspec(dllexport) bool SIM_DebugSchedulingState(oni_sim::ext::SetSchedulingStateMessage* out) {
  if (!g || !out) return false;
  WaitIdle();
  out->skip_physics_frames = g->skip_physics_frames;
  out->pressure_dir = g->pressure_dir;
  out->shuffle_dir = g->world.ShuffleDir();
  out->substep_carry = g->substep_carry;
  out->displace_rotation = g->displace_rotation;
  out->first_physics_substep = g->first_physics_substep ? 1u : 0u;
  out->vf_active = g->vf_active ? 1u : 0u;
  return true;
}

// Also not part of Klei's ABI: Layer C3's census (`World::Effervescence::census`), in
// `ext::kEffervescenceCensus*` order. Returns the field count ALWAYS -- the discoverable-size
// contract of `SIM_DebugStableTicks` -- and copies the first min(capacity, count) fields. Cumulative since the sim was initialised; a reader takes deltas.
// Returns 0 with no sim.
__declspec(dllexport) int32_t SIM_DebugEffervescenceCensus(double* out, int32_t capacity) {
  if (!g) return 0;
  WaitIdle();
  // Copies min(capacity, fields): the census only ever grows at the end, so a reader built for
  // an older, shorter census still reads its prefix.
  const int32_t n = ext::kEffervescenceCensusFields;
  if (out != nullptr && capacity > 0) {
    const World::Effervescence& e = g->world.EffervescenceConfig();
    const int32_t k = capacity < n ? capacity : n;
    for (int32_t i = 0; i < k; ++i) out[i] = e.census[i];
  }
  return n;
}

// Also not part of Klei's ABI, and the getter half of `ext::kSetStableTicks`. Returns the
// number of bytes the sim holds -- one per padded cell -- ALWAYS, and copies them only when
// `out` is non-null and `capacity` is at least that number. So the size is discoverable
// without a buffer, and a caller never has to derive the padded cell count from the world
// dimensions:
//
//     const int32_t n = SIM_DebugStableTicks(nullptr, 0);
//     std::vector<uint8_t> buf(n);
//     SIM_DebugStableTicks(buf.data(), n);
//
// Returns 0 with no sim, which is also what a caller sees for a world that has not been
// allocated -- there is nothing to read in either case.
__declspec(dllexport) int32_t SIM_DebugStableTicks(uint8_t* out, int32_t capacity) {
  if (!g) return 0;
  WaitIdle();
  const std::vector<uint8_t>& v = g->world.StableTicks();
  const int32_t n = static_cast<int32_t>(v.size());
  if (out && n > 0 && capacity >= n) memcpy(out, v.data(), static_cast<size_t>(n));
  return n;
}

// Not part of Klei's ABI, same two-call sizing idiom as SIM_DebugStableTicks above and the
// same one byte per PADDED cell: World::gas_sleeping_, the volume-fractions mixing layer's
// per-cell sleep bit (1 = parked, 0 = awake). Read-only, and deliberately WITHOUT a
// kSetGasSleeping counterpart, which is the whole point of exporting it.
//
// Every Allocate resets the array to all-awake and every Load goes through one, so a restored
// world has forgotten which cells were parked -- exactly the shape of the gap that
// ext::kSetStableTicks had to close for the unstable solids. This one does not need closing:
// MixRoomPooled skips a pair only when BOTH ends sleep, and both only sleep after five
// consecutive ticks in which every transfer between them fell under MixPair's
// MinTransferMass floor. A sub-floor transfer is never applied, so nothing accumulates
// behind the floor, and nothing can write into such a pair without waking one end first --
// skipping it is exactly equal to running it. Measured rather than argued: seeding
// 612 cells across three species and running a 120-tick scrub sweep with the mixture layer in the
// digest, all 113 replayed ticks reproduced the run bit for bit.
//
// It is exported so that conclusion stays checkable by looking instead of by reasoning, and
// so a caller that ever does find a divergence here can see the array rather than infer it.
__declspec(dllexport) int32_t SIM_DebugGasSleeping(uint8_t* out, int32_t capacity) {
  if (!g) return 0;
  WaitIdle();
  const std::vector<uint8_t>& v = g->world.GasSleepingArray();
  const int32_t n = static_cast<int32_t>(v.size());
  if (out && n > 0 && capacity >= n) memcpy(out, v.data(), static_cast<size_t>(n));
  return n;
}

// The getter half of ext::kSetDiseaseGrowth -- see that id's comment for why the two arrays
// it reads are not in any save. Both come back through one call because they are one state:
// World::disease_accum_ (a float per padded cell) into `accum`, World::disease_infest_ (a
// byte per padded cell) into `infest`. Same two-call sizing as the exports above -- pass
// nulls to learn the count, then buffers of that size:
//
//     const int32_t n = SIM_DebugDiseaseGrowth(nullptr, nullptr, 0);
//     std::vector<float> accum(n);
//     std::vector<uint8_t> infest(n);
//     SIM_DebugDiseaseGrowth(accum.data(), infest.data(), n);
//
// Either pointer may be null on its own, so a caller wanting one array does not have to
// allocate the other. Returns 0 with no sim and with no allocated world alike -- there is
// nothing to read in either case.
__declspec(dllexport) int32_t SIM_DebugDiseaseGrowth(float* accum, uint8_t* infest,
                                                     int32_t capacity) {
  if (!g) return 0;
  WaitIdle();
  const std::vector<float>& a = g->world.DiseaseAccum();
  const std::vector<uint8_t>& f = g->world.DiseaseInfest();
  const int32_t n = static_cast<int32_t>(a.size());
  if (n <= 0 || f.size() != a.size()) return n;
  if (capacity >= n) {
    if (accum) memcpy(accum, a.data(), static_cast<size_t>(n) * sizeof(float));
    if (infest) memcpy(infest, f.data(), static_cast<size_t>(n));
  }
  return n;
}

// The getter half of ext::kSetCellRadiation: World::radiation_, one float per PADDED cell.
// Same two-call sizing as SIM_DebugDiseaseGrowth -- pass null to learn the count, then a buffer
// of that size. Returns 0 with no sim and with no allocated world alike.
__declspec(dllexport) int32_t SIM_DebugCellRadiation(float* out, int32_t capacity) {
  if (!g) return 0;
  WaitIdle();
  const std::vector<float>& r = g->world.Radiation();
  const int32_t n = static_cast<int32_t>(r.size());
  if (n > 0 && out && capacity >= n) memcpy(out, r.data(), static_cast<size_t>(n) * sizeof(float));
  return n;
}

// The getter half of ext::kSetVisibilityState: the whole payload, header included, so a caller
// sends back exactly what it got. A buffer held empty is written as zeros, which is what a frame
// reads from it (FrameVisible). Same two-call sizing as SIM_DebugCellRadiation; 0 with no sim
// and with no allocated world alike.
__declspec(dllexport) int32_t SIM_DebugVisibilityState(uint8_t* out, int32_t capacity) {
  if (!g) return 0;
  WaitIdle();
  if (!g->world.Allocated()) return 0;
  const size_t count = g->world.GameCount();
  if (count == 0) return 0;
  const size_t total = sizeof(ext::SetVisibilityStateMessage) + 3 * count;
  const int32_t n = static_cast<int32_t>(total);
  if (!out || capacity < n) return n;
  ext::SetVisibilityStateMessage m{};
  m.count = static_cast<int32_t>(count);
  m.simSlot = g->visible_sim_slot;
  memcpy(out, &m, sizeof(m));
  const std::vector<uint8_t>* bufs[3] = {&g->visible, &g->visible_game[0], &g->visible_game[1]};
  uint8_t* dst = out + sizeof(m);
  for (const std::vector<uint8_t>* b : bufs) {
    if (b->size() == count) {
      memcpy(dst, b->data(), count);
    } else {
      memset(dst, 0, count);
    }
    dst += count;
  }
  return n;
}

// The getter half of ext::kSetRegistryState, and the sixth checkpoint component's read side.
// Serializes all four component registries -- buildings, element chunks, the element/disease
// flow components, and the radiation emitters -- into one opaque blob whose format is
// `sim/registry_state.h`'s and nobody else's.
//
// Same two-call sizing as the exports above, except that the size is not derivable from the
// world's dimensions at all here: it depends on how much the game has registered. So the
// first call is not a convenience, it is the only way to learn the length:
//
//     const int32_t n = SIM_DebugRegistryState(nullptr, 0);
//     std::vector<uint8_t> buf(n);
//     SIM_DebugRegistryState(buf.data(), n);
//
// The blob is built either way, because its length is not known before it is built. That is
// two serializations for one read, and it is deliberate: the registries are small next to the
// world (a few hundred kilobytes against a nine-megabyte save on a full colony) and a caller
// that could ask for a size without paying for it would still have to handle the size
// changing between the two calls.
//
// Returns 0 with no sim. Never returns a partial blob: if `capacity` is short, nothing is
// copied and the required size comes back, exactly as the other exports do.
__declspec(dllexport) int32_t SIM_DebugRegistryState(uint8_t* out, int32_t capacity) {
  if (!g) return 0;
  WaitIdle();
  std::vector<uint8_t> blob;
  registry_state::Save(g->buildings, g->chunks, g->flow, g->radiation, &blob);
  const int32_t n = static_cast<int32_t>(blob.size());
  if (out && n > 0 && capacity >= n) memcpy(out, blob.data(), static_cast<size_t>(n));
  return n;
}

// The getter half of ext::kSetExtCellState, and the SEVENTH checkpoint component's read side.
// Serialises the per-cell extension properties a save blob does not carry -- `kRehydrated` and
// `kCheckpointOnly`, never `kSaved` and never an orphan, both of which the save blob already
// holds. Format is `sim/ext_state.h`'s and nobody else's.
//
// Same two-call sizing as SIM_DebugRegistryState, and for the same reason: the length depends
// on what has been registered and written, not on the world's dimensions.
//
//     const int32_t n = SIM_DebugExtCellState(nullptr, 0);
//     std::vector<uint8_t> buf(n);
//     SIM_DebugExtCellState(buf.data(), n);
//
// ALWAYS RETURNS AT LEAST A HEADER, even with nothing to carry. A zero would be
// indistinguishable from a DLL that predates this export, and a caller checking for the
// component would then silently carry six things instead of seven -- which is exactly how the
// fifth and sixth components stayed missing for two milestones each.
__declspec(dllexport) int32_t SIM_DebugExtCellState(uint8_t* out, int32_t capacity) {
  if (!g) return 0;
  WaitIdle();
  std::vector<uint8_t> blob;
  g->world.ExtCheckpointToBlob(&blob);
  const int32_t n = static_cast<int32_t>(blob.size());
  if (out && n > 0 && capacity >= n) memcpy(out, blob.data(), static_cast<size_t>(n));
  return n;
}

// The INSPECTION twin of the export above: EVERY registered per-cell extension property, all
// three persistence classes, in the same format and with the same two-call sizing.
//
// READ-ONLY BY CONSTRUCTION. There is no message that accepts this blob -- `kSetExtCellState`
// takes the checkpoint's, and `ext_state::Load` refuses a `kSaved` record by name with a
// reason, so feeding this one back cannot half-apply even by mistake.
//
// It exists because "what is in this cell" and "what must a checkpoint carry" are different
// questions, and only the second had an answer. The properties a checkpoint must NOT carry --
// the gas mixture triple and `sim.room_promoted`, because the save blob already holds them --
// are exactly the ones the sim itself writes and a tool most wants to see.
//
// The same two omissions apply as to the checkpoint blob, and a caller has to be told: an
// orphan (an uninstalled mod's carried bytes) is not a registered property
// and is not here, and a property at its registered default in every cell is skipped, so
// "absent" cannot be told from "registered but untouched".
__declspec(dllexport) int32_t SIM_DebugExtCellStateAll(uint8_t* out, int32_t capacity) {
  if (!g) return 0;
  WaitIdle();
  std::vector<uint8_t> blob;
  g->world.ExtInspectToBlob(&blob);
  const int32_t n = static_cast<int32_t>(blob.size());
  if (out && n > 0 && capacity >= n) memcpy(out, blob.data(), static_cast<size_t>(n));
  return n;
}

// The per-frame descriptor table. See `kPublishCellProperty` in
// abi/sim_abi_ext.h for the contract; the two things that matter are repeated here because
// this is the function people will read instead of the header.
//
// NO `WaitIdle()`, DELIBERATELY, and it is the whole point of the stage. Every other custom
// export in this file opens with the worker barrier; this one cannot need it, because it
// reads a frame that has already been published and is not the one the worker is filling.
// Adding a barrier here later would quietly give back what the stage was for.
//
// THAT SAFETY IS WHY `frame` IS A PARAMETER. The obvious signature -- "give me the current
// table" -- would have to read `last_published`, which the worker reassigns mid-tick, so the
// caller could be handed the slot being overwritten. The frame the game is holding is the
// only frame whose table is provably still intact, so the caller passes it and this resolves
// the slot from it. A frame pointer that is neither live publication is a refusal.
//
// A refusal returns null with *count = 0. A LIVE frame with no subscriptions returns a
// non-null pointer with *count = 0 -- different facts, distinguishable, which is why the
// empty case cannot just be `p.ext.data()` on an empty vector.
static const ext::ExtPublishedProperty kNoPublishedProperties{};

__declspec(dllexport) const ext::ExtPublishedProperty* SIM_ExtPublishedProperties(
    const GameDataUpdate* frame, int32_t* count) {
  if (count) *count = 0;
  if (!g || frame == nullptr) return nullptr;
  for (int i = 0; i < 2; ++i) {
    const PublishedFrame& p = g->pub[i];
    if (&p.update != frame) continue;
    if (count) *count = static_cast<int32_t>(p.ext.size());
    return p.ext.empty() ? &kNoPublishedProperties : p.ext.data();
  }
  return nullptr;
}

// The event-stream half. See `kSubscribeEventStream` in
// `abi/sim_abi_ext.h` for the whole design; the two exports below are its read side.
//
// No `WaitIdle()`, and unlike the two publish exports the reason is not the frame pointer:
// the declaration set is written once, inside `SIM_Initialize`, before the worker thread
// exists, and nothing adds, removes or reorders a stream afterwards. Reading it is reading
// immutable state.
__declspec(dllexport) int32_t SIM_ExtEventStreamIndex(const char* name) {
  if (!g || name == nullptr) return -1;
  return g->ext_events.Find(std::string(name));
}

// Same three-way answer as `SIM_ExtPublishedProperties`: null + 0 is a refusal (no sim, null
// frame, or a frame that is not one of the two live publications), non-null + 0 is a live
// frame with nothing subscribed.
static const ext::ExtPublishedStream kNoPublishedStreams{};

__declspec(dllexport) const ext::ExtPublishedStream* SIM_ExtPublishedEvents(
    const GameDataUpdate* frame, int32_t* count) {
  if (count) *count = 0;
  if (!g || frame == nullptr) return nullptr;
  for (int i = 0; i < 2; ++i) {
    const PublishedFrame& p = g->pub[i];
    if (&p.update != frame) continue;
    if (count) *count = static_cast<int32_t>(p.streams.size());
    return p.streams.empty() ? &kNoPublishedStreams : p.streams.data();
  }
  return nullptr;
}

// The stream registry as metadata. `SIM_ExtEventStreamIndex` answers only
// for a name the caller already has, and the published table above lists SUBSCRIBED streams
// only, so neither one lets a generic client find out what this DLL declares. See
// `ExtEventStreamDesc` in `abi/sim_abi_ext.h`.
//
// Count takes no `WaitIdle()`, for the reason spelled out above `SIM_ExtEventStreamIndex`:
// the declaration set is written once inside `SIM_Initialize` and is immutable afterwards.
__declspec(dllexport) int32_t SIM_ExtEventStreamCount() {
  if (!g) return 0;
  return g->ext_events.Count();
}

// Describe DOES take it, and the difference is `subscribed`. That flag is written by
// `ApplySubscribeEventStream` inside `DrainQueue`, on the sim thread, during the frame -- so a
// barrier-free read here does not tear, it goes STALE: the drain for the tick the caller just
// ran may not have happened, and a subscription the ABI documents as landing in one tick
// appears to take two. `vftest`'s stage 4b arms asserted one and measured two, which is how
// this was found rather than argued.
//
// Affordable for the same reason stage 3b's barrier is: this is the metadata path, fetched
// once per world. `SIM_ExtPublishedEvents` is the per-frame path and stays barrier-free.
__declspec(dllexport) int32_t SIM_ExtEventStreamDescribe(int32_t streamIdx,
                                                         ext::ExtEventStreamDesc* out) {
  if (!g || out == nullptr) return 0;
  WaitIdle();
  const ext::EventStreamRegistry::Stream* s = g->ext_events.At(streamIdx);
  if (s == nullptr) return 0;

  ext::ExtEventStreamDesc d{};
  const size_t n = s->name.size() < sizeof(d.name) - 1 ? s->name.size() : sizeof(d.name) - 1;
  memcpy(d.name, s->name.data(), n);
  d.stride = s->stride;
  d.subscribed = s->subscribed ? 1 : 0;
  d.declaredIdx = streamIdx;
  *out = d;
  return 1;
}

// abi/sim_abi_ext.h at kRegisterElementAttribute. The read-back half: the
// per-element registry stores what a mod pushes and hands it back, and without these two
// exports "hands back" would be a claim rather than a capability -- which is exactly the state
// `kSetMolecularMass` was in, write-only, from the day it was added.
//
// NO `WaitIdle()` ON EITHER, deliberately, and the reason is the one `SIM_ComputeGasPressure`
// already relies on two hundred lines below: the element table and the attributes hanging off
// it are content data, written on the calling thread through the immediate path (which
// `SIM_HandleMessage` has already serialised against the worker) and read-only for the rest of
// the frame. A barrier here would buy nothing and would put a stall on a path a UI can poll.
//
// -1 for an unknown name, the same sentinel `SIM_ExtEventStreamIndex` uses.
__declspec(dllexport) int32_t SIM_ExtElementAttributeIndex(const char* name) {
  if (!g || name == nullptr) return -1;
  return g->elements.Attributes().Find(std::string(name));
}

// Reads one component of one element's attribute, by SimHashes id. Returns 0 for "no value",
// which covers a bad attribute index, a component outside the registered arity, and -- the
// common case -- an element this attribute simply says nothing about. `outBits` is untouched
// unless the answer is 1, so a caller that ignores the return reads its own initialiser rather
// than a stale value from the previous call.
//
// UNSET IS NOT ZERO, and this signature is where that matters most. `sim.molecular_mass` has
// no entry for the two hundred elements nobody has corrected, and the right answer for those
// is Klei's own `Element::molarMass`, which this export deliberately does not substitute --
// the caller asking about an override wants to know whether there is one.
__declspec(dllexport) int32_t SIM_ExtElementAttribute(int32_t attrIdx, int32_t idHash,
                                                      int32_t component, uint32_t* outBits) {
  if (!g) return 0;
  uint32_t bits = 0;
  if (!g->elements.Attributes().Read(attrIdx, idHash, component, &bits)) return 0;
  if (outBits) *outBits = bits;
  return 1;
}

// The planetary latent accumulator, in joules, signed, for
// one world. Returns 1 and fills `out`, or 0 for a null pointer or no world -- and 1 with a
// zero for the ordinary case of a world that is not a planet, because "no transitions counted"
// is a real answer and not a failure.
//
// WHAT IT IS NOT: it is not a temperature, and the DLL will not turn it into one. Stationeers
// divides its own `LatentEnergyOffset` by `PlanetaryAtmosphereSimulation.GetHeatCapacity()`
// (`:298`) -- the summed heat capacity of four planetary reservoirs. We defer those reservoirs
// to Mod 4, so the divisor is a configured constant, and a configured constant is a balance
// decision: it lives in `OniFramework.PlanetaryEnvironment` beside the one its sibling
// accumulator already uses, not in here. See `World::NoteWorldLatentEnergy`.
//
// `WaitIdle()`, unlike the two attribute exports above: this number is written by the worker
// inside `StepStateChange`, so a barrier-free read is a read of a half-written frame.
//
// CUMULATIVE AND NOT ZEROED BY READING, like both ledgers -- sample, wait, subtract.
__declspec(dllexport) int32_t SIM_ExtWorldLatentEnergy(int32_t worldIndex, double* out) {
  if (!out) return 0;
  WaitIdle();
  if (!g) return 0;
  *out = g->world.LatentEnergyOfWorld(worldIndex);
  return 1;
}

// Mod 3's sun path, read back. `kSetWorldSun` is queued, so a mod that has sent one cannot tell
// from its own memory whether the DLL applied it -- and a gate that asserts shade moved has to
// know the mechanism is in the sim before a zero means anything. `WaitIdle()` because the
// record is written by `DrainQueue` on the worker.
__declspec(dllexport) int32_t SIM_ExtWorldSun(int32_t worldIndex, float* dirX, float* dirY) {
  if (!dirX || !dirY) return 0;
  WaitIdle();
  if (!g) return 0;
  World::SunDirection s;
  if (!g->world.SunOfWorld(worldIndex, &s)) return 0;
  *dirX = s.dir_x;
  *dirY = s.dir_y;
  return 1;
}

// The direct beam, copied. Not a pointer into `g->textures`, because `FillPropertyTextures`
// rewrites the buffer every frame on the worker and a pointer would be a read of a half-written
// field the moment the next frame starts. One copy of a byte per cell is 393 KB on a 512x768
// asteroid, which a caller on a 200 ms sweep pays for once.
__declspec(dllexport) int32_t SIM_ExtCopySunBeam(uint8_t* out, int32_t capacity) {
  WaitIdle();
  if (!g) return -1;
  const std::vector<uint8_t>& beam = g->textures.sun_beam;
  const int32_t n = static_cast<int32_t>(beam.size());
  if (out != nullptr && capacity >= n && n > 0) {
    memcpy(out, beam.data(), static_cast<size_t>(n));
  }
  return n;
}

// The enumeration half, and the reason the two exports above are not it:
// both of them answer only for a name and an element the caller already has. A tool handed a
// SimDLL it did not build has neither, so like the per-cell registry and the event streams,
// this registry has a discovery path. See `ExtElementAttributeDesc` in `abi/sim_abi_ext.h` for
// the descriptor and for the two fields it deliberately does not carry.
//
// NONE OF THE THREE TAKES `WaitIdle()`, for the same reason as the lookups:
// the element table and the attributes hanging off it are content data, written on the calling
// thread through the immediate path that `SIM_HandleMessage` has already serialised against
// the worker, and read-only for the rest of the frame. Stage 4b's Describe DOES take the
// barrier and the difference is real: `subscribed` there is written inside `DrainQueue` on the
// sim thread, so a barrier-free read went stale. Nothing in this descriptor is written from a
// kernel -- `writes` and `keys` both move only under a message.
__declspec(dllexport) int32_t SIM_ExtElementAttributeCount() {
  if (!g) return 0;
  return g->elements.Attributes().Count();
}

// Fills `out` for one attribute and returns 1, or returns 0 and leaves `out` untouched for an
// unregistered index, a null pointer, or no sim. The untouched half matters: a caller that
// ignores the return value reads its own initialiser rather than the previous attribute's
// name, which is the discipline stage 3b and stage 4b both hold.
__declspec(dllexport) int32_t SIM_ExtElementAttributeDescribe(int32_t attrIdx,
                                                              ext::ExtElementAttributeDesc* out) {
  if (!g || out == nullptr) return 0;
  const ext::ElementAttributeRegistry& r = g->elements.Attributes();
  const ext::ElementAttributeRegistry::Attribute* a = r.At(attrIdx);
  if (a == nullptr) return 0;

  ext::ExtElementAttributeDesc d{};
  const size_t n = a->name.size() < sizeof(d.name) - 1 ? a->name.size() : sizeof(d.name) - 1;
  memcpy(d.name, a->name.data(), n);
  d.type = a->type;
  d.arity = a->arity;
  d.stride = static_cast<int32_t>(a->stride);
  d.valueCount = static_cast<int32_t>(a->keys.size());
  d.declaredIdx = attrIdx;
  d.firstParty = a->first_party ? 1 : 0;
  d.writes = a->writes;
  *out = d;
  return 1;
}

// The keys an attribute holds a value for, ascending, as SimHashes ids. Returns the TOTAL the
// attribute has rather than the number copied, so one call with `max` 0 sizes the buffer and a
// second one fills it; `out` may be null exactly when `max` is 0.
//
// Ascending because the registry stores them that way (`ElementAttributeRegistry::Insert`
// keeps `keys` sorted so a lookup is a `lower_bound`), so this is a copy and not a sort -- and
// because a caller diffing two of these against each other should not have to sort first.
__declspec(dllexport) int32_t SIM_ExtElementAttributeKeys(int32_t attrIdx, int32_t* out,
                                                          int32_t max) {
  if (!g) return 0;
  const ext::ElementAttributeRegistry::Attribute* a = g->elements.Attributes().At(attrIdx);
  if (a == nullptr) return 0;
  const int32_t total = static_cast<int32_t>(a->keys.size());
  if (out != nullptr && max > 0) {
    const int32_t copy = max < total ? max : total;
    memcpy(out, a->keys.data(), static_cast<size_t>(copy) * sizeof(int32_t));
  }
  return total;
}

// Also not part of Klei's ABI, and there is no Klei-side equivalent to read: this is the
// conservation ledger, which is a check against the sim's
// own arithmetic rather than against the other sim.
//
// A flat array of doubles rather than the struct, so the driver does not have to include
// `world.h` to read it. The order below is the contract, and `kLedgerFields` in
// diffsim.cpp is the other half of it — append only, never reorder. Returns the number of
// fields the DLL knows about, so a driver built against a longer list can tell.
//
// Field 0 is the only one that costs anything: it walks the grid. The rest are counters
// the sim has been keeping all along.
__declspec(dllexport) int SIM_DebugLedger(double* out, int count) {
  WaitIdle();
  const int fields = 18;
  if (!out || !g) return fields;
  const World::Ledger& l = g->world.Books();
  const double v[fields] = {g->world.TotalGridMass(),
                            l.emitted,
                            l.modified,
                            l.consumed,
                            l.dug,
                            l.ore,
                            l.unstable,
                            l.sublimated,
                            l.wisp,
                            l.thin_liquid,
                            l.cleared,
                            l.component_consumed,
                            l.component_emitted,
                            l.emitter_ore,
                            l.mover,
                            l.world_init,
                            l.atmosphere_boundary,
                            l.dissolved_surface};
  const int n = count < fields ? count : fields;
  for (int i = 0; i < n; ++i) out[i] = v[i];
  return fields;
}

// Switch the expensive half of the energy ledger on. Added, after a perf pass
// measured what leaving it on had been costing.
//
// The cheap buckets -- the ones whose call site already holds the kilojoules it is charging --
// are always counted and are unaffected by this. What this turns on is the buckets that have
// to FIND their number by walking a cell's stored energy before and after the change:
// `MoverCharge` in the fluid sweeps and `CellEnergyCharge` in ModifyCell. `MoverCharge` alone
// was 0.63 ms of a 2.32 ms StepGasPressure and 7.6 % of the frame, in every build including a
// shipped one, for an instrument nothing in a shipped build reads.
//
// CALL IT BEFORE THE FIRST TICK. Switching the ledger on midway counts part of a run's
// transfers against all of that run's drift, which reads as a conservation bug in the sim
// rather than as the measurement error it is. `diffsim --energy-ledger` and `vftest` both call
// it immediately after loading the DLL, which is the only correct moment.
//
// Returns the previous setting, so a caller can restore it; ignores `g == nullptr` the same
// way the other debug exports do, by answering as though nothing is on.
__declspec(dllexport) int SIM_DebugSetEnergyLedger(int enabled) {
  if (!g) return 0;
  WaitIdle();
  const bool was = g->world.EnergyLedgerEnabled();
  g->world.SetEnergyLedgerEnabled(enabled != 0);
  return was ? 1 : 0;
}

// The energy counterpart of SIM_DebugLedger, added for the power -> heat work.
// Same contract: a flat array of doubles, append only, never reorder, and the driver's
// `kEnergyLedgerFields` in diffsim.cpp is the other half of it. Returns the number of fields
// the DLL knows about.
//
// Kilojoules throughout -- ONI's specificHeatCapacity is kJ/(kg K). Fields 0..3 are the four places energy actually lives, published
// separately rather than summed, because the commonest transfer in the sim -- a building
// exchanging with the cell under it -- moves energy between two of them and would read as
// drift against any one alone. The driver adds them up; the DLL does not, so a driver can
// also watch one reservoir on its own.
//
// Field 40 is informational, a stock rather than a flow: the energy `CellEnergyCarry`
// (tunables.def) holds for cells whose ModifyCellEnergy payments have not yet moved their
// temperature. It is in no container (fields 0..3) and in no inflow: field 11 books only what
// landed, field 12 what was refused, and the held remainder sits between them until a later
// payment lands it, so adding it to the stock without also adding its delta to the inflows would
// read the carry as drift. Zero unless the row is or has been on: switching the row off leaves
// what is held where it is, and the carry is not saved.
//
// Fields 0..3 walk their containers and are the only ones that cost anything. The rest are
// counters the sim keeps as it goes, one per call site that moves energy across the boundary
// of those four containers. Each was added where a scenario's drift said a site was
// uncharged, not from a list guessed by reading the kernels: a guessed list balances the books
// by construction and proves nothing (World::Ledger's "per call site, not per cause" note).
__declspec(dllexport) int SIM_DebugEnergyLedger(double* out, int count) {
  WaitIdle();
  const int fields = 41;
  if (!out || !g) return fields;

  double building = 0.0;
  // Counted as well as summed, and published as its own field. A building total of 0 kJ is
  // ambiguous on its own -- it means either "no buildings are registered" or "they are all
  // registered with no heat capacity", which are completely different faults. The first live
  // run of the power -> heat rule hit exactly that ambiguity against a real colony.
  double building_count = 0.0;
  for (const BuildingHeatExchangeData& d : g->buildings.exchange.Data()) {
    building_count += 1.0;
    if (d.per_cell_heat_capacity <= 0.0f) continue;
    building += static_cast<double>(d.per_cell_heat_capacity) *
                static_cast<double>(d.CellCount()) * static_cast<double>(d.temperature);
  }

  double conduit = 0.0;
  for (const ConduitTemperatureData& d : g->conduits.Registered().Data()) {
    conduit += static_cast<double>(d.contents_heat_capacity) *
               static_cast<double>(d.temperature);
  }

  double chunk = 0.0;
  for (const ElementChunkData& d : g->chunks.chunks.Data()) {
    chunk += static_cast<double>(d.heat_capacity) * static_cast<double>(d.temperature);
  }

  const World::EnergyLedger& l = g->world.EnergyBooks();
  const double v[fields] = {g->world.TotalGridEnergy(g->elements), building, conduit, chunk,
                            l.building_operating, l.building_exchange,
                            l.building_registered, l.building_energy_msg,
                            l.building_waste_heat, building_count, l.sim_seconds,
                            l.cell_energy_msg,     l.cell_energy_refused,
                            l.building_exhaust,    l.building_exhaust_bounced,
                            l.radiated_to_environment,
                            l.chunk_registered,    l.chunk_energy_msg,
                            l.chunk_adjuster,      l.chunk_exchange,
                            l.chunk_energy_refused, l.cell_modified, l.displaced,
                            l.conduction_clamp,     l.cleared,
                            l.wisp,                 l.thin_liquid,
                            l.unstable,             l.consumed,
                            l.emitted,              l.component_emitted,
                            l.mover,                l.world_init,
                            l.sublimated,           l.phase_change,
                            l.transition_ore,       l.absorbed_from_environment,
                            l.atmosphere_boundary,  l.cell_radiated_to_environment,
                            l.dissolved_surface,
                            g->world.CellEnergyCarryHeld()};
  const int n = count < fields ? count : fields;
  for (int i = 0; i < n; ++i) out[i] = v[i];
  return fields;
}

// Not part of Klei's ABI, same category as SIM_DebugRandomState/SIM_DebugLedger above.
// The only way an external driver — an opaque LoadLibrary
// handle, same access the real game has — can read gas_species_/gas_mass_ back out, since
// GameDataUpdate does not publish them. Returns the mass (kg) of `speciesIdx` (an
// ElementTable index) currently held in game cell `cellIdx`'s mixture, or 0 if the world
// isn't allocated, the cell is out of range, or that species isn't present there.
__declspec(dllexport) float SIM_DebugGasMass(int32_t cellIdx, int32_t speciesIdx) {
  WaitIdle();
  if (!g || !g->world.Allocated() || !g->world.ValidGameCell(cellIdx)) return 0.0f;
  const size_t cell = g->world.Padded(static_cast<size_t>(cellIdx));
  const int slot = gas::FindSlot(g->world, cell, static_cast<uint16_t>(speciesIdx));
  return slot >= 0 ? g->world.GasMass(cell, slot) : 0.0f;
}

// The per-cell flow read: the data exists (`flow_`, four floats per cell), and this is its
// export.
//
// WHAT IT PUBLISHES. The flow accumulator: one FRAME of mass transfers, in kg, signed, where
// the sign is the direction the mass went. `World::AddFlow` writes four slots a cell and
// `UpdateFlowTexture` takes `[0] - [1]` for x and `[3] - [2]` for y, so both ends of a
// transfer land on the same signed value; those two differences are what this hands back,
// which makes it the same field the vanilla `Flow` texture draws rather than a second
// derivation of it. It is the only per-cell field in the sim that records a transfer instead
// of a state, and it was invisible from outside.
//
// WHY IT RETURNS THE TOUCHED LIST RATHER THAN TAKING CELL INDICES. Two reasons, and neither
// is convenience. `World::AddFlow` already records every cell it writes -- the recording is
// what keeps a 6 MB buffer affordable on a 512x768 asteroid -- so the sparse list IS the
// answer, a few hundred entries where an index-batch API would make the caller ask about
// hundreds of thousands of cells to find them. And §18 of the assessment measured the other
// half: `WaitIdle` returns immediately when the worker is idle, so it is not a guaranteed
// per-call stall, but N per-cell reads are N opportunities to block on an in-flight frame.
// One bulk read takes one barrier. A CELL THAT IS NOT IN THE LIST HAS ZERO FLOW; that is the
// whole meaning of its absence, and a caller asking about one cell should ask that way.
//
// WHEN TO CALL IT. Between frames. `ClearFlow` runs at the top of the physics frame, after
// the skip test, so a read taken between frames sees the frame that just finished -- exactly
// what the texture pass saw. A read taken mid-frame sees a partial accumulation. A skipped
// physics frame leaves the buffer alone, by construction, because Klei does not call
// `UpdateData` on one.
//
// WHAT IT DOES NOT PUBLISH, and this is a scope line rather than an oversight. §11 asks for
// "velocity, pressure delta". The velocity half is here. THE PRESSURE DELTA IS NOT, because
// it does not exist: nothing in the sim stores a previous frame's pressure, so publishing a
// delta would mean adding a per-cell frame-over-frame array -- new state, new save-size
// question, new clear cadence -- which is not "only the export is missing" and should be
// decided on its own merits. `SIM_GasPressure` already answers instantaneous pressure per
// cell for a mixture-owned cell.
//
// THE SIZING CALL. Returns the TOTAL number of cells the accumulator wrote, which may exceed
// `max` -- the return value sizes the buffer rather than reporting the copy, the same
// discipline `SIM_ExtElementAttributeKeys` holds. Pass null buffers with `max` 0 to ask only
// for the count. Entries are written in the order the cells were first touched, which is the
// order the texture pass walks them; it is not sorted and callers must not assume it is.
//
// IT PUBLISHES THE RAW TRANSFER, NOT THE TEXTURE'S NUMBER, and the difference is deliberate.
// `FillFlowTexture` (sim/textures.h) multiplies by `1.0f / max(mass, 1.0f)` -- so a gas cell
// publishes its raw hundredths of a kilogram and a 1000 kg water cell publishes a thousandth
// of its transfer -- and zeroes the cell entirely when the snapshot element and the live
// element disagree. That scale is a rendering normalisation for a shader, not a physical
// quantity; a debug consumer asking what moved wants kilograms. Anything that needs the
// texture's own value should read the texture.
//
// `outElement` is the OTHER half of that gate, and is the part that is otherwise invisible:
// `World::SnapshotFlowElements`, taken at the substep's fourth and last `CellSOA::CopyFrom`,
// which is the element the texture compares against. The live element it is compared TO is
// already published every frame through the ordinary ABI, so a caller that wants to reproduce
// the gate joins this against data it already has. The snapshot is taken inside the region
// loop, so late entries on the touched list can have no snapshot yet; those get 0xFFFF rather
// than a stale or invented element, because "this cell moved mass and the texture has not
// decided about it" is a different fact from any element id and must not be confusable with
// one.
__declspec(dllexport) int32_t SIM_DebugCellFlow(int32_t* outCells, float* outX, float* outY,
                                                uint16_t* outElement, int32_t max) {
  WaitIdle();
  if (!g || !g->world.Allocated()) return 0;
  const std::vector<World::FlowCell>& touched = g->world.FlowTouched();
  const std::vector<float>& accum = g->world.FlowAccum();
  const std::vector<uint16_t>& elems = g->world.FlowElements();
  const int32_t total = static_cast<int32_t>(touched.size());
  if (max <= 0 || !outCells || !outX || !outY || !outElement) return total;
  const int32_t n = max < total ? max : total;
  for (int32_t i = 0; i < n; ++i) {
    const size_t padded = static_cast<size_t>(touched[static_cast<size_t>(i)].padded);
    const float* f = &accum[padded * 4];
    outCells[i] = static_cast<int32_t>(touched[static_cast<size_t>(i)].game);
    outX[i] = f[0] - f[1];
    outY[i] = f[3] - f[2];
    outElement[i] = static_cast<size_t>(i) < elems.size() ? elems[static_cast<size_t>(i)]
                                                          : static_cast<uint16_t>(0xFFFF);
  }
  return total;
}

// ------------------------------------------------------------------ the live profile
//
// THE PER-KERNEL TABLE, AS DATA. Two other readers exist and neither one is a program. `bench` times these kernels OFFLINE, on scenarios somebody wrote, against a
// recorded corpus, and never calls `StepPhysics` at all. `ToggleProfiler` times them in a
// colony somebody actually built and then prints the answer as ENGLISH into `Player.log` and
// `sim_profile.log`, where the only way to compare two builds is to read two logs. These
// three exports are the third reader: the same table as DATA, while the game is running.
// A debug HTTP route can serve it to `curl`, but the general case is a mod asking whether its
// own additions cost a frame -- which it must be able to ask without patching the DLL.
//
// TWO CLOCKS, AND ONLY ONE OF THEM NEEDS ARMING. The milliseconds are gated on `g_prof.on`
// -- off, a `PROF` scope reads no timer at all, which is the property that lets the profiler
// exist in a shipping build. THE CENSUS IS NOT GATED (sim/census.h): the counts accumulate
// on every frame of every run, armed or not, so `SIM_DebugProfile` answers with real
// examined/changed numbers from a DLL nobody has switched anything on in. That asymmetry is
// published rather than smoothed over, because it is the difference between a row whose
// `msTotal` is 0.0 for want of arming and one whose kernel genuinely never ran.
//
// READING DOES NOT ZERO. The numbers are cumulative since the last arm, which is what makes a
// live poller work at all: sample, wait, sample again, subtract. A read that zeroed would
// make two pollers -- or a poller and the backtick key -- silently destroy each other's
// window, and there is no way for either to notice.
//
// WHY THE STATE LIVES OUTSIDE `Sim`. It always has (see `Profiler` above): a toggle survives
// a load, an Alloc and a Shutdown, because a profiling run that ended when the player
// reloaded would be useless. These exports inherit that, and it is why they
// answer with `g == nullptr` instead of refusing -- there is no world, but the table is still
// the honest record of the frames that did run. It is also why they take no barrier in that
// case: with no sim there is no worker thread and no frame to be in flight.

// Arms or disarms the millisecond half, and returns the PREVIOUS setting so a caller can put
// it back the way it found it.
//
// ARMING AN ALREADY-ARMED PROFILER IS A NO-OP, deliberately, and this is the one place the
// export is not simply the keyboard path with a different trigger: `ArmProfiler` zeroes, and
// zeroing a window somebody armed from the backtick key would silently discard their
// measurement halfway through it. A caller that genuinely wants a fresh window disarms first.
//
// DISARMING DOES NOT REPORT. `ToggleProfiler`'s second press calls `EmitProfile`, which
// appends to `Player.log` and to `sim_profile.log`; this does not, because a caller reading
// through `SIM_DebugProfile` already HAS every number that report would print and does not
// need it restated into two files it cannot read back. The keyboard path is unchanged.
__declspec(dllexport) int32_t SIM_DebugSetProfiler(int32_t enabled) {
  if (g) WaitIdle();
  const bool was = g_prof.on;
  if (enabled != 0) {
    if (!was) ArmProfiler();
  } else {
    g_prof.on = false;
  }
  return was ? 1 : 0;
}

// The table's header row: what the per-slot numbers below are to be read against. Returns 1
// and fills `out`, or 0 and leaves it untouched for a null pointer -- the `Describe`
// discipline, so a caller that ignores the return cannot find a plausible summary sitting in
// its buffer.
//
// `regionCells` and its two companions are the denominators. `census::BudgetPerInvocation`
// picks between them per kernel by that kernel's DECLARED bound, and the export publishes the
// choice it made per row rather than making the caller re-derive the rule; these three are
// here so that the caller can still show the reader what the budget was made of.
__declspec(dllexport) int32_t SIM_DebugProfileSummary(OniProfileSummary* out) {
  if (!out) return 0;
  if (g) WaitIdle();
  out->enabled = g_prof.on ? 1 : 0;
  out->frames = g_prof.frames;
  out->regionCells = static_cast<int64_t>(census::g_region_cells);
  out->regionCellsInclusive = static_cast<int64_t>(census::g_region_cells_incl);
  out->gridCells = static_cast<int64_t>(census::g_grid_cells);
  out->regions = static_cast<int64_t>(census::g_regions);
  // The bucket past the last slot: work announced while no sweep was running -- a queued
  // `ModifyCell`, an emitter, a building. Its own field rather than folded into a kernel's,
  // for the reason `EmitProfile` gives it its own row: charging it to whichever kernel ran
  // last is the sort of quiet misattribution that makes a profile agree with itself and lie.
  out->outsideAnySweep =
      static_cast<int64_t>(census::g_counts[census::kSlotCount].changes);
  return 1;
}

// One row per timed kernel, in slot order, which is `census::Slot` order and therefore the
// order `bench`'s table and `EmitProfile`'s lines already use -- the three readers of this
// table must not disagree about what row 4 means.
//
// THE SIZING CALL. Returns the TOTAL number of slots, which may exceed `max`: the return
// sizes the buffer rather than reporting the copy, the discipline `SIM_ExtElementAttributeKeys`
// and `SIM_DebugCellFlow` already hold. A null buffer with `max` 0 asks only for the count.
//
// `msTotal` and `calls` are cumulative since the last arm and are 0 for every slot while the
// profiler is off. `examined`, `skipped`, `changes` and `invocations` are the census and are
// live either way. `cellSweep` says whether the four census numbers are in a unit that
// compares -- a cell sweep walks a rectangle of the grid; the rest walk LISTS (the message
// queue, the registered buildings, the element chunks, the radiation emitters) where a ratio
// against the grid says nothing at all -- and `budgetPerInvocation` is 0 on those rows for the
// same reason. `withinBudget` is `census::WithinBudget`, the one assertion the census file
// exists for, computed HERE rather than left to the caller: it is exact integer arithmetic
// over counts the DLL owns, and a consumer that re-derived it from the published numbers would
// be re-deriving the rule that decides which denominator each row takes.
__declspec(dllexport) int32_t SIM_DebugProfile(OniProfileSlot* out, int32_t max) {
  if (g) WaitIdle();
  const int32_t total = census::kSlotCount;
  if (max <= 0 || !out) return total;
  const int32_t n = max < total ? max : total;
  for (int32_t i = 0; i < n; ++i) {
    const census::Counters& c = census::g_counts[i];
    OniProfileSlot& r = out[i];
    memset(&r, 0, sizeof(r));
    const char* name = census::Name(i);
    // Truncated rather than refused: a name too long for the field is a bug in census.h, not
    // a runtime condition a caller can do anything about, and a silently unterminated string
    // handed across an ABI is worse than a short one.
    strncpy(r.name, name, sizeof(r.name) - 1);
    r.msTotal = g_prof.ms[i];
    r.calls = g_prof.calls[i];
    r.examined = static_cast<int64_t>(c.examined);
    r.skipped = static_cast<int64_t>(c.skipped);
    r.changes = static_cast<int64_t>(c.changes);
    r.invocations = static_cast<int64_t>(c.invocations);
    r.cellSweep = census::IsCellSweep(i) ? 1 : 0;
    r.budgetPerInvocation =
        r.cellSweep ? static_cast<int64_t>(census::BudgetPerInvocation(i)) : 0;
    r.withinBudget = census::WithinBudget(i) ? 1 : 0;
  }
  return total;
}

// Debug-only, same category as SIM_DebugGasMass: ext::kPromoteRoom's effect, seen from outside
// the DLL. The gated kernels read the same bit via their own `rooms` pointer (see the
// per-region `rooms` computation above); this export is the read-only,
// WaitIdle-synchronized path for test/debug tooling and any future managed-side query that
// needs the answer outside the sim thread, not a second source of truth. Returns -1 if the
// cell has no room (volume-fractions never activated, or the cell isn't open), else 0
// (vanilla-owned, the default for every room today) or 1 (promoted).
__declspec(dllexport) int32_t SIM_DebugRoomOwned(int32_t cellIdx) {
  WaitIdle();
  if (!g || !g->world.Allocated() || !g->world.ValidGameCell(cellIdx)) return -1;
  if (!g->vf_active || g->vf_rooms.room_of.size() != g->world.PaddedCount()) return -1;
  const size_t cell = g->world.Padded(static_cast<size_t>(cellIdx));
  const int32_t room = g->vf_rooms.room_of[cell];
  if (room < 0) return -1;
  return gas::IsRoomOwned(g->vf_rooms, room) ? 1 : 0;
}

// WHICH room, not whether it is promoted -- the id SIM_DebugRoomOwned above discards.
//
// It is a SECOND export rather than a wider return from SIM_DebugRoomOwned, and that is forced
// rather than chosen: that export already spends the value 0 on "vanilla-owned", so an id
// returned through it would make room 0-promoted indistinguishable from any room not promoted.
// There is no widening of it that keeps both answers, and its callers (GasMixtureFacade,
// vftest) read the 0/1 meaning today.
//
// -1 whenever there is no room to name: no world, an invalid cell, the mixture layer never
// activated in this world (the room graph is built for mixing and does not exist before it), or
// a solid cell. The id is stable only between rebuilds of the room graph -- geometry changing
// merges and splits rooms and re-assigns ids, exactly as BuildRoomGraph's own comment says --
// so it is a grouping key for the frame you read it on, never a handle to store across a dig.
__declspec(dllexport) int32_t SIM_RoomId(int32_t cellIdx) {
  WaitIdle();
  if (!g || !g->world.Allocated() || !g->world.ValidGameCell(cellIdx)) return -1;
  if (!g->vf_active || g->vf_rooms.room_of.size() != g->world.PaddedCount()) return -1;
  return g->vf_rooms.room_of[g->world.Padded(static_cast<size_t>(cellIdx))];
}

// THE WHOLE ROOM, SUMMED IN THE SIM. The field contract is in abi/sim_ext_api.h with the struct; the arithmetic
// is gas::ComputeRoomAggregate in sim/gas_rooms.h. This function is the barrier and the
// lookups, and deliberately nothing else -- the sum is testable offline without a DLL because
// it lives in the header rather than here.
//
// WHY IT EXISTS AT ALL, since a caller can read every cell it touches one at a time: the sim
// already maintains the room graph for its own mixing kernel. Re-deriving a room from outside
// -- walking a radius-3 diamond of 25 cells with SIM_DebugRoomOwned, then SIM_GasComposition and
// SIM_GasPressure on the survivors, allocating an array per cell -- costs up to 75 boundary
// crossings and 25 allocations per vent per 200 ms tick, to produce a mean pressure and a
// mass-by-species sum. This is one crossing and no allocation.
//
// And it answers a DIFFERENT, better question. A diamond is not a room: it leaks through walls,
// truncates a large room, and takes in cells behind a door. This is the real 4-connected
// open-cell component, wall-aware by construction -- including the stale-kGasImpermeable
// correction recorded in gas_rooms.h's own header, which managed code re-deriving rooms would
// have to re-derive or silently reintroduce.
//
// NOT CACHED, on purpose. The cost is O(cells in the room) walked natively, and a cache would
// need an invalidation rule covering every kernel that writes gas mass -- a much larger claim
// than the item makes. Group with SIM_RoomId and ask once per room per tick.
__declspec(dllexport) int32_t SIM_RoomAggregate(int32_t cellIdx, OniRoomAggregate* out) {
  WaitIdle();
  if (!out) return 0;
  // Every refusal below still leaves a zeroed struct with roomId == -1, so a caller that
  // ignores the return value cannot read a plausible room out of it. ComputeRoomAggregate
  // writes that shape itself; these two paths return before reaching it.
  memset(out, 0, sizeof(*out));
  out->roomId = -1;
  if (!g || !g->world.Allocated() || !g->world.ValidGameCell(cellIdx)) return 0;
  if (!g->vf_active || g->vf_rooms.room_of.size() != g->world.PaddedCount()) return 0;
  const size_t cell = g->world.Padded(static_cast<size_t>(cellIdx));
  const int32_t room = g->vf_rooms.room_of[cell];
  return gas::ComputeRoomAggregate(g->world, g->elements, g->vf_rooms, room, out) ? 1 : 0;
}

// A grid raycast: sim/raycast.h, which runs Klei's own
// radiation ray walk with a stop condition added. Batched because its callers are sensors and
// beams, which come in dozens per tick, and each ray is only O(length) -- the barrier and the
// crossing would otherwise be most of the cost.
//
// Every query gets a written result, valid or not, so a caller that skips the return value
// cannot read a plausible hit out of a stale buffer. The return counts the valid ones.
__declspec(dllexport) int32_t SIM_QueryRaycast(const OniRaycastQuery* queries,
                                               OniRaycastResult* results, int32_t count) {
  WaitIdle();
  if (!queries || !results || count < 0) return -1;
  if (!g || !g->world.Allocated()) return -1;
  const World& w = g->world;
  const int32_t pw = w.PaddedWidth();
  auto game_cell = [&w](size_t padded) -> int32_t {
    return padded == RaycastHit::kNoCell ? -1 : static_cast<int32_t>(w.GameIndex(padded));
  };
  int32_t answered = 0;
  for (int32_t i = 0; i < count; ++i) {
    const OniRaycastQuery& q = queries[i];
    OniRaycastResult& out = results[i];
    if (!w.ValidGameCell(q.startCell) || !w.ValidGameCell(q.endCell)) {
      out.hitCell = -1;
      out.lastClearCell = -1;
      out.cellsVisited = 0;
      out.transmission = 0.0f;
      continue;
    }
    const size_t a = w.Padded(static_cast<size_t>(q.startCell));
    const size_t b = w.Padded(static_cast<size_t>(q.endCell));
    const RaycastHit r = Raycast(w, g->elements, static_cast<int32_t>(a % pw),
                                 static_cast<int32_t>(a / pw), static_cast<int32_t>(b % pw),
                                 static_cast<int32_t>(b / pw), q.phaseMask, q.propertyMask,
                                 q.ignorePropertyMask);
    out.hitCell = game_cell(r.hit);
    out.lastClearCell = game_cell(r.last_clear);
    out.cellsVisited = r.visited;
    out.transmission = r.transmission;
    ++answered;
  }
  return answered;
}

// Not part of Klei's ABI, but not a debug export either: this is the native half of a
// dominant-element facade plus selective Harmony patches, rather than a full managed-side
// overhaul. A C# P/Invoke wrapper
// intercepts a vanilla read like Grid.Element[cell] and, for a cell this reports as
// gas-mixture-active, substitutes this answer instead of trusting GameDataUpdate's own
// single-element array — which still publishes whatever PhaseEntry says, untouched, exactly
// as it always has (see StepPhysics's vf block comment: the two layers are still separate).
//
// Returns the ElementTable index of whichever species holds the most mass in cell
// `cellIdx`'s gas mixture, or 0xFFFF if the cell holds no gas-mixture state at all (the
// facade's caller reads that as "fall back to the normal vanilla value"). `outMass`, if
// non-null, receives the TOTAL mass summed across every species in the mixture — not just
// the dominant one's share — which is what a vanilla mass reader actually wants to see. A
// weighted-average answer is deliberately not built; dominant-only is the default until
// something concrete needs the other.
__declspec(dllexport) uint16_t SIM_GasDominantElement(int32_t cellIdx, float* outMass) {
  WaitIdle();
  if (outMass) *outMass = 0.0f;
  if (!g || !g->world.Allocated() || !g->world.ValidGameCell(cellIdx)) return 0xFFFF;
  const size_t cell = g->world.Padded(static_cast<size_t>(cellIdx));
  const uint8_t mask = g->world.GasOccupiedMask(cell);
  if (mask == 0) return 0xFFFF;
  uint16_t dominant = 0xFFFF;
  float dominant_mass = -1.0f;
  float total = 0.0f;
  for (int s = 0; s < gas::kMaxSpeciesPerCell; ++s) {
    if (!(mask & (1u << s))) continue;
    const float m = g->world.GasMass(cell, s);
    total += m;
    if (m > dominant_mass) {
      dominant_mass = m;
      dominant = g->world.GasSpecies(cell, s);
    }
  }
  if (outMass) *outMass = total;
  return dominant;
}

// Same category as SIM_GasDominantElement above (not Klei's ABI, not a debug export): the
// rest of that function's own loop, published instead of collapsed to a single winner. Pipes carrying real
// mixed gas can't be shown by a dominant-only read: two Gas Element
// Sensors sitting in one mixing room, one over O2 and one over CO2, only look "mixed" if a
// caller can see both species survive in one cell, not just whichever currently outweighs the
// other.
//
// Fills outSpecies/outMass (parallel arrays, caller-owned, each at least maxSlots long) with
// every species actually occupying cellIdx's mixture, up to maxSlots entries, and returns how
// many were written. maxSlots is always clamped to kMaxSpeciesPerCell (8) even if the caller
// passes more, since the mixture never holds more slots than that regardless. Returns 0 (and
// touches neither array) if the world isn't allocated, the cell is out of range, or the cell
// holds no gas-mixture state.
__declspec(dllexport) int32_t SIM_GasComposition(int32_t cellIdx, uint16_t* outSpecies,
                                                  float* outMass, int32_t maxSlots) {
  WaitIdle();
  if (!g || !g->world.Allocated() || !g->world.ValidGameCell(cellIdx)) return 0;
  if (!outSpecies || !outMass || maxSlots <= 0) return 0;
  const size_t cell = g->world.Padded(static_cast<size_t>(cellIdx));
  const uint8_t mask = g->world.GasOccupiedMask(cell);
  if (mask == 0) return 0;
  const int32_t cap =
      (maxSlots < gas::kMaxSpeciesPerCell) ? maxSlots : gas::kMaxSpeciesPerCell;
  int32_t written = 0;
  for (int s = 0; s < gas::kMaxSpeciesPerCell && written < cap; ++s) {
    if (!(mask & (1u << s))) continue;
    outSpecies[written] = g->world.GasSpecies(cell, s);
    outMass[written] = g->world.GasMass(cell, s);
    ++written;
  }
  return written;
}

// Not part of Klei's ABI, not a debug export: the batch counterpart to
// SIM_GasDominantElement/SIM_GasComposition above, for overlay colouring.
// PropertyTextures.UpdateSolidLiquidGasMass reads Grid.Element/Grid.Mass for EVERY cell in the active world on an
// ISim200ms tick -- one WaitIdle-synchronized single-cell call per cell at that scale would
// mean tens of thousands of sim-thread rendezvous every 200ms. This export does ONE
// WaitIdle for an entire caller-chosen batch of cells instead.
//
// Contract (frozen here, before any managed caller exists yet -- same
// "design before code" discipline as RoomGraph.owned itself, and before any texture-side
// Harmony/rendering hook is attempted):
//   * cellIndices/count: any list of game cell indices, in any order, caller's choice of
//     batch size -- no assumption about rows, rects, or chunk shape, so a caller can match
//     whatever granularity UpdateTextureThreaded already chunks into (or the whole visible
//     world in one call) without this contract changing.
//   * outOwned[i] is 1 iff cellIndices[i]'s ROOM has been promoted (gas::IsRoomOwned), NOT
//     merely "this cell currently holds mixture mass" -- a promoted room legitimately sitting
//     at zero real gas is still a real answer a caller must trust over vanilla's stale
//     nonzero Grid.Mass, so promotion status and current mass are deliberately reported as
//     two separate things instead of collapsing "no mass" and "not promoted" together the way
//     SIM_GasDominantElement's 0xFFFF sentinel does.
//   * outDominant[i]/outMass[i] are only meaningful where outOwned[i] == 1; on every other
//     index they are written as 0xFFFF/0.0f but the caller has no reason to read them --
//     outOwned[i] == 0 means "fall back to vanilla Grid.Element/Grid.Mass for this cell,"
//     the same collapse every other facade method in this category uses.
//   * Returns the number of cells actually written (== count on success), or 0 if count <= 0,
//     any pointer is null, or the world isn't allocated -- in which case NO array is touched
//     at all, so a caller can treat a 0 return as "nothing here is promoted yet, do the whole
//     batch the vanilla way" without inspecting individual entries.
__declspec(dllexport) int32_t SIM_GasMassBatchQuery(const int32_t* cellIndices, int32_t count,
                                                     uint8_t* outOwned, uint16_t* outDominant,
                                                     float* outMass) {
  WaitIdle();
  if (!cellIndices || !outOwned || !outDominant || !outMass || count <= 0) return 0;
  if (!g || !g->world.Allocated()) return 0;
  const bool rooms_valid =
      g->vf_active && g->vf_rooms.room_of.size() == g->world.PaddedCount();
  for (int32_t i = 0; i < count; ++i) {
    outOwned[i] = 0;
    outDominant[i] = 0xFFFF;
    outMass[i] = 0.0f;
    const int32_t cellIdx = cellIndices[i];
    if (!g->world.ValidGameCell(cellIdx)) continue;
    if (!rooms_valid) continue;
    const size_t cell = g->world.Padded(static_cast<size_t>(cellIdx));
    const int32_t room = g->vf_rooms.room_of[cell];
    if (room < 0 || !gas::IsRoomOwned(g->vf_rooms, room)) continue;
    outOwned[i] = 1;
    const uint8_t mask = g->world.GasOccupiedMask(cell);
    uint16_t dominant = 0xFFFF;
    float dominant_mass = -1.0f;
    float total = 0.0f;
    for (int s = 0; s < gas::kMaxSpeciesPerCell; ++s) {
      if (!(mask & (1u << s))) continue;
      const float m = g->world.GasMass(cell, s);
      total += m;
      if (m > dominant_mass) {
        dominant_mass = m;
        dominant = g->world.GasSpecies(cell, s);
      }
    }
    outDominant[i] = dominant;
    outMass[i] = total;
  }
  return count;
}

// Not part of Klei's ABI, not a debug export: read-only exposure of gas_mixture.h's own
// CellPressure (PV = nRT over kMaxSpeciesPerCell slots, Dalton's law summed across species) --
// the exact math MixPair already runs internally to find each pair's pressure delta, published
// here instead of staying trapped inside the mixing kernel. A promoted
// tank cell fed more moles than a single cell "should" hold (CellVolumeM3 is a fixed placeholder
// volume, same for every cell -- see CellPressure's own comment) reads a visibly higher pressure
// here than an ordinary room's cells.
//
// Returns the cell's total mixture pressure in Pa, or -1.0f if the world isn't allocated, the
// cell is out of range, or the cell holds no gas-mixture state at all (mirrors
// SIM_GasDominantElement's "no data" collapse, just with a sentinel a pressure value can never
// legitimately hit instead of 0xFFFF -- 0 Pa is a real, valid answer for an occupied-but-cold or
// massless mixture, so it can't double as the sentinel the way it can for a mass total).
__declspec(dllexport) float SIM_GasPressure(int32_t cellIdx) {
  WaitIdle();
  if (!g || !g->world.Allocated() || !g->world.ValidGameCell(cellIdx)) return -1.0f;
  const size_t cell = g->world.Padded(static_cast<size_t>(cellIdx));
  if (g->world.GasOccupiedMask(cell) == 0) return -1.0f;
  return gas::CellPressure(g->world, g->elements, cell);
}

// Not part of Klei's ABI, not a debug export: the same PV=nRT/V physics as SIM_GasPressure
// above (gas::PressureFromMoles/MolesFromSpeciesList, sim/gas_mixture.h), generalized off any
// particular cell so a caller with its OWN container -- a tank, a pipe -- gets pressure from
// the one native formula instead of re-deriving it in managed code. The state itself (a tank's mass, a pipe's contents)
// correctly stays managed -- that matches real vanilla's own design, not a shortcut -- but the
// physics formula does not need a second, hand-copied implementation just because its owner
// happens to live outside `g->world`.
//
// `species`/`massKg` are parallel arrays, `count` entries each (Dalton's law: summed as
// independent partial pressures, same as CellPressure). No `g->world` access and no WaitIdle()
// needed -- this never touches mutable per-frame sim state, only the element table, which is
// loaded once (`g->elements.Load`, at world/save load) and read-only for the rest of the
// process, the same assumption sim/conduits.h's own unsynchronized `table.At(...)` calls
// already rely on.
//
// Returns -1.0f (never a real pressure, mirrors SIM_GasPressure's own sentinel) for a null
// world/pointer, a non-positive count or volume, or a species list that sums to zero moles --
// 0 Pa is a real, legitimate answer this can't reuse as "no data."
__declspec(dllexport) float SIM_ComputeGasPressure(const uint16_t* species, const float* massKg,
                                                     int32_t count, float temperatureK,
                                                     float volumeM3) {
  if (!g || !species || !massKg || count <= 0 || volumeM3 <= 0.0f) return -1.0f;
  const float moles = gas::MolesFromSpeciesList(g->elements, species, massKg, count);
  if (moles <= 0.0f) return -1.0f;
  return gas::PressureFromMoles(moles, temperatureK, volumeM3);
}

// Not part of Klei's ABI, not a debug export: publishes emitters.h's own
// CalculateCombinedTemperature (the game's, reproduced 1:1) for the same reason
// SIM_ComputeGasPressure above exists: a managed caller blending temperatures should call the
// native original rather than keep a copy. Pure function, no `g->world` access, no WaitIdle() needed -- it never
// touches sim state at all, only its four arguments.
__declspec(dllexport) float SIM_CalculateCombinedTemperature(float massA, float tempA,
                                                                float massB, float tempB) {
  return CalculateCombinedTemperature(massA, tempA, massB, tempB);
}

// Not part of Klei's ABI, not a debug export: publishes gas_mixture.h's own
// EqualizeSingleSpecies (see its own doc comment for the physics and why it exists) --
// generalizes the native mixing kernel's MixPair equalization step to two containers with
// independent volumes, added for a real design fix: a tank plumbed to a pipe with
// no valve between them should share one pressure, not have the tank act as its own implicit
// active pump. Pure function, no `g` access at all -- doesn't even need the element table,
// since molarMass arrives as a direct argument.
__declspec(dllexport) float SIM_EqualizeSingleSpeciesMass(float molarMass, float massA,
                                                            float tempA, float volumeA,
                                                            float massB, float tempB,
                                                            float volumeB, float rate) {
  return gas::EqualizeSingleSpecies(molarMass, massA, tempA, volumeA, massB, tempB, volumeB,
                                     rate);
}

// Not part of Klei's ABI, not a debug export: publishes gas_mixture.h's own
// AdiabaticFillTemperature (see its own doc comment for the full derivation and why neither
// vanilla ONI nor Stationeers already solve this). Pure function, no `g` access at all.
__declspec(dllexport) float SIM_AdiabaticFillTemperature(float gammaMix, float massA,
                                                           float tempA, float massB,
                                                           float tempB) {
  return gas::AdiabaticFillTemperature(gammaMix, massA, tempA, massB, tempB);
}

// Not part of Klei's ABI, not a debug export: liquid_mixture.h's VolumeFromMass, for a mod's
// own liquid tank/pipe reading -- what fraction of its capacity it currently holds. Pure
// function, no `g` access, no WaitIdle() needed.
__declspec(dllexport) float SIM_LiquidVolumeFromMass(float massKg, float densityKgM3) {
  return liquid::VolumeFromMass(massKg, densityKgM3);
}

// Not part of Klei's ABI, not a debug export: publishes liquid_mixture.h's own
// EqualizeLiquidVolume (see its own doc comment for the physics -- Stationeers' real
// volume-ratio model, checked before designing this, not gas_mixture.h's ideal-gas pressure
// reused wrong). Added for Mod 1's liquid tank/pipe mechanic, mirroring
// SIM_EqualizeSingleSpeciesMass's role for gas. Pure function, no `g` access at all.
__declspec(dllexport) float SIM_EqualizeLiquidVolumeMass(float densityKgM3, float massA,
                                                            float capacityA_m3, float massB,
                                                            float capacityB_m3, float rate) {
  return liquid::EqualizeLiquidVolume(densityKgM3, massA, capacityA_m3, massB, capacityB_m3,
                                       rate);
}

// Not part of Klei's ABI, not a debug export: publishes phase_change.h's own
// ComputePhaseChangeStep -- see that header's own comment for the derivation (vanilla
// TransitionCell has no latent heat; this follows Stationeers' state-change accounting on a
// mass-based tank). Pure function, no `g`
// access at all -- every input arrives as a direct argument, same as this file's other
// gas_mixture.h/liquid_mixture.h exports.
//
// outConvertedMassKg/outRemainingTemperatureK are out-params rather than a packed return value
// so this stays a plain extern "C" float-in/float-out signature P/Invoke can bind directly,
// same convention as every other multi-result export in this file.
__declspec(dllexport) void SIM_ComputePhaseChangeStep(float massKg, float temperatureK,
                                                        float thresholdK, float latentHeatJPerKg,
                                                        float specificHeatCapacity,
                                                        float dtSeconds,
                                                        float conversionRatePerSecond,
                                                        float minRemainderKg,
                                                        float* outConvertedMassKg,
                                                        float* outRemainingTemperatureK) {
  phase::PhaseChangeStep step = phase::ComputePhaseChangeStep(
      massKg, temperatureK, thresholdK, latentHeatJPerKg, specificHeatCapacity, dtSeconds,
      conversionRatePerSecond, minRemainderKg);
  if (outConvertedMassKg) *outConvertedMassKg = step.converted_mass_kg;
  if (outRemainingTemperatureK) *outRemainingTemperatureK = step.remaining_temperature_k;
}

// ------------------------------------------------------- extension registry (sim_abi_ext.h)
//
// Not part of Klei's ABI. Two exports, both of them things a registration message alone
// cannot do: look an index up by name after the fact, and read a value back.
//
// The readback is not a convenience. Extension properties are NOT projected into
// `GameDataUpdate` — nothing on the managed side reads a per-cell extension value through the
// normal channel — so without this there is no way for a caller, or a test, to observe that a
// write landed except through whatever physics happens to consume it. `sim.thermal_mass_bonus`
// has a kernel that consumes it; a property registered by a mod may have nothing yet.

// The index a registration returned, looked up by name, for a caller that did not keep it (or
// that is checking whether somebody else already registered the name). -1 if unregistered.
__declspec(dllexport) int32_t SIM_ExtCellPropertyIndex(const char* name) {
  if (!g || name == nullptr) return -1;
  return g->world.ExtCells().Find(std::string(name));
}

// =======================================================================================
// The frame's phase ordering, as a published table.
// =======================================================================================
//
// See `ONI_EXT_PHASE_LIST` in abi/sim_abi_ext.h for the guarantee this publishes and, just
// as importantly, the three things it deliberately does NOT guarantee (order across regions,
// substeps per frame, that a gated phase runs at all).
//
// The table is generated from the same macro the enum is, so the two cannot drift apart --
// which matters more here than in most places, because a phase list that has quietly fallen
// one entry behind the step body is worse than no phase list: it is confidently wrong, and a
// mod author has no way to tell from the outside. `driver/src/vftest.cpp` holds the other
// half of that check, a literal copy of the order that fails on purpose if the step body is
// reordered without the list.
//
// NEITHER export touches `g`, and neither calls `WaitIdle()`. Every other export in this file
// does both because every other one reads world state; this one reads a static array, so a
// caller can enumerate the phases before `SIM_AllocateCells` has run. See the header.
// No anonymous namespace: this sits inside the `extern "C"` block that opens above, and a
// namespace-definition is not allowed in a linkage-specification. `static` gives the table
// internal linkage instead, which is all it needed the namespace for.
struct PhaseTableEntry {
  const char* name;
  int32_t scope;
  int32_t gate;
};
static const PhaseTableEntry kPhaseTable[] = {
#define ONI_EXT_PHASE_ROW(e, n, s, gt) {n, ext::s, ext::gt},
    ONI_EXT_PHASE_LIST(ONI_EXT_PHASE_ROW)
#undef ONI_EXT_PHASE_ROW
};
static_assert(sizeof(kPhaseTable) / sizeof(kPhaseTable[0]) ==
                  static_cast<size_t>(ext::kPhaseCount),
              "phase table and ExtPhase disagree -- ONI_EXT_PHASE_LIST is the one definition");

__declspec(dllexport) int32_t SIM_ExtPhaseCount() { return ext::kPhaseCount; }

__declspec(dllexport) int32_t SIM_ExtPhaseDescribe(int32_t phaseIdx, ext::ExtPhaseDesc* out) {
  if (out == nullptr || phaseIdx < 0 || phaseIdx >= ext::kPhaseCount) return 0;
  const PhaseTableEntry& e = kPhaseTable[phaseIdx];
  ext::ExtPhaseDesc d{};
  // Bounded even though the names are literals in a header we own: the one way an over-long
  // name could arrive is a future edit to the macro that forgets the 31-character budget, and
  // a truncated name is a better failure than a smashed caller buffer.
  const size_t n = strlen(e.name) < sizeof(d.name) - 1 ? strlen(e.name) : sizeof(d.name) - 1;
  memcpy(d.name, e.name, n);
  d.index = phaseIdx;
  d.scope = e.scope;
  d.gate = e.gate;
  *out = d;
  return 1;
}

// =======================================================================================
// The message surface, as a published table.
// =======================================================================================
//
// See `ONI_EXT_MESSAGE_LIST` in abi/sim_abi_ext.h for what these publish and why. The short
// version: whether a message takes effect on this call or at the top of the next frame
// decides the CALLER's code shape, so it is exported and enforced rather than only written.
//
// The table is generated from the same macro the index enum is, for the reason the phase
// table is: a published surface that has quietly fallen a row behind is worse than no
// published surface, because it is confidently wrong and a caller cannot tell from outside.
// `driver/src/gastest.cpp` holds the hand-typed second opinion -- deliberately in the suite
// public CI can run, since this table needs no game content to check.
//
// NEITHER export touches `g`, and neither calls `WaitIdle()`. Same as the phase pair and for
// the same reason: a static array, readable before `SIM_AllocateCells`, so a mod can check
// the DLL it loaded at the one moment it can still decline to load.
struct MessageTableEntry {
  const char* name;
  int32_t id;
  int32_t messageClass;
  int32_t delivery;
  int32_t phase;
};
static const MessageTableEntry kMessageTable[] = {
#define ONI_EXT_MESSAGE_ROW(e, n, i, c, d, p) {n, ext::i, ext::c, ext::d, ext::p},
    ONI_EXT_MESSAGE_LIST(ONI_EXT_MESSAGE_ROW)
#undef ONI_EXT_MESSAGE_ROW
};
static_assert(sizeof(kMessageTable) / sizeof(kMessageTable[0]) ==
                  static_cast<size_t>(ext::kExtMessageCount),
              "message table and ExtMessageIndex disagree -- ONI_EXT_MESSAGE_LIST is the one "
              "definition");

__declspec(dllexport) int32_t SIM_ExtMessageCount() { return ext::kExtMessageCount; }

__declspec(dllexport) int32_t SIM_ExtMessageDescribe(int32_t msgIdx, ext::ExtMessageDesc* out) {
  if (out == nullptr || msgIdx < 0 || msgIdx >= ext::kExtMessageCount) return 0;
  const MessageTableEntry& e = kMessageTable[msgIdx];
  ext::ExtMessageDesc d{};
  // Bounded for the same reason SIM_ExtPhaseDescribe bounds its copy: the names are literals
  // in a header we own, so the only way an over-long one arrives is a future edit to the
  // macro that forgets the 39-character budget, and a truncated name beats a smashed buffer.
  const size_t n = strlen(e.name) < sizeof(d.name) - 1 ? strlen(e.name) : sizeof(d.name) - 1;
  memcpy(d.name, e.name, n);
  d.index = msgIdx;
  d.id = e.id;
  d.messageClass = e.messageClass;
  d.delivery = e.delivery;
  d.phase = e.phase;
  *out = d;
  return 1;
}

// =======================================================================================
// The tunable table, readable. See `kSetTunable` in abi/sim_abi_ext.h.
// =======================================================================================
//
// No `g`, no `WaitIdle()`: `g_tunables` is written only by the immediate `kSetTunable`, on the
// thread that sends it and while the sim is idle, so a read from that thread cannot race it, and
// the metadata is compile-time. All four work before `SIM_Initialize`.
__declspec(dllexport) int32_t SIM_ExtTunableCount() { return kTunableCount; }

__declspec(dllexport) int32_t SIM_ExtTunableDescribe(int32_t tunableId, ext::ExtTunableDesc* out) {
  if (out == nullptr || tunableId < 0 || tunableId >= kTunableCount) return 0;
  const TunableInfo& info = kTunableInfo[tunableId];
  ext::ExtTunableDesc d{};
  // Every string is bounded at compile time (tunables.h), so these never truncate; bounded here
  // anyway, for the reason every other describe export bounds its copy.
  auto copy = [](char* dst, size_t cap, const char* src) {
    const size_t n = strlen(src) < cap - 1 ? strlen(src) : cap - 1;
    memcpy(dst, src, n);
  };
  copy(d.name, sizeof(d.name), info.name);
  copy(d.group, sizeof(d.group), info.group);
  copy(d.origin, sizeof(d.origin), info.origin);
  copy(d.doc, sizeof(d.doc), info.doc);
  d.tunableId = tunableId;
  d.type = info.type;
  d.flags = TunableFlags(tunableId);
  TunableGetBits(kTunableDefaults, tunableId, &d.defaultBits);
  d.stockBits = info.stock_bits;
  d.minBits = info.lo_bits;
  d.maxBits = info.hi_bits;
  *out = d;
  return 1;
}

__declspec(dllexport) int32_t SIM_ExtTunableIndex(const char* name) {
  if (name == nullptr) return -1;
  for (int32_t i = 0; i < kTunableCount; ++i) {
    if (strcmp(kTunableInfo[i].name, name) == 0) return i;
  }
  return -1;
}

__declspec(dllexport) int32_t SIM_ExtTunableGet(int32_t tunableId, uint64_t* outBits) {
  if (outBits == nullptr || tunableId < 0 || tunableId >= kTunableCount) return 0;
  return TunableGetBits(g_tunables, tunableId, outBits) ? 1 : 0;
}

// The registry as metadata. See `ExtCellPropertyDesc` in
// abi/sim_abi_ext.h for the whole argument; the short version is that the per-frame
// descriptor table carries no registered default on purpose, and a client that PAINTS an
// extension property needs one to tell an untouched cell from a written zero.
//
// Enumerable, not merely probeable: `SIM_ExtCellPropertyIndex` answers only for a name the
// caller already had, which is no use to a visualizer that is trying to discover what this
// world registered.
__declspec(dllexport) int32_t SIM_ExtCellPropertyCount() {
  if (!g) return 0;
  WaitIdle();
  return g->world.ExtCells().Count();
}

// Fills `out` and returns 1, or returns 0 leaving `out` untouched for an unregistered index,
// a null pointer, or no sim.
//
// The barrier is affordable here and was not in `SIM_ExtPublishedProperties`, for the reason
// stated in the ABI header: registration appends to the registry until the first allocate
// closes it, and this is fetched once per world rather than once per frame.
__declspec(dllexport) int32_t SIM_ExtCellPropertyDescribe(int32_t propertyIdx,
                                                          ext::ExtCellPropertyDesc* out) {
  if (!g || out == nullptr) return 0;
  WaitIdle();
  const ext::CellPropertyRegistry& reg = g->world.ExtCells();
  const ext::CellPropertyRegistry::Property* p = reg.At(propertyIdx);
  if (p == nullptr) return 0;

  ext::ExtCellPropertyDesc d{};
  // Truncation cannot happen -- `ValidPropertyName` caps a name at 47 characters precisely so
  // it fits this field with its terminator -- but the copy is bounded anyway, because the one
  // place a name could arrive over-long is a future registration path that forgot the rule.
  const size_t n = p->name.size() < sizeof(d.name) - 1 ? p->name.size() : sizeof(d.name) - 1;
  memcpy(d.name, p->name.data(), n);
  d.type = p->type;
  d.persist = p->persist;
  d.arity = p->arity;
  d.stride = static_cast<int32_t>(ext::ScalarStride(p->type));
  // PADDED cells, matching `ExtPublishedProperty::cellCount` and the save blob's records, and
  // 0 before `World::Allocate` has sized anything.
  d.cellCount = static_cast<int32_t>(reg.Cells());
  d.defaultBits = p->default_bits;
  d.published = p->published ? 1 : 0;
  *out = d;
  return 1;
}

// Reads one component of one cell of one property back as raw bits, zero-extended from the
// property's stride: bit-cast for kExtF32, the low byte / low two bytes for kExtU8 / kExtU16,
// verbatim for kExtI32. Returns 1 on success, 0 for an unregistered property, an out-of-range
// cell or component, or a world that has not been allocated.
//
// `component` is 0 for a scalar (arity 1) property. It is a parameter rather than an implied 0
// for the same reason `SetCellPropertyMessage` carries one: reading lane 0 of an arity-8
// property when you meant lane 3 returns a plausible number, which is the worst kind of wrong.
//
// `cellIdx` is a GAME cell, like every other cell-addressed message and export in this file;
// the padding conversion happens here so a caller never has to know the world is padded.
__declspec(dllexport) int32_t SIM_ExtReadCellProperty(int32_t cellIdx, int32_t propertyIdx,
                                                      int32_t component, uint32_t* outBits) {
  if (!g || outBits == nullptr || !g->world.ValidGameCell(cellIdx)) return 0;
  WaitIdle();
  const size_t cell = g->world.Padded(static_cast<size_t>(cellIdx));
  return g->world.ExtCells().Read(propertyIdx, cell, component, outBits) ? 1 : 0;
}

// ---------------------------------------------------------------------------------------
// The field discovery path. Count / Index / Describe, the same three exports
// registries 1, 2 and 3 have, and DELIBERATELY NO READ: a field's values are the property's
// values, which `SIM_ExtPublishedProperties` already publishes zero-copy and
// `SIM_ExtReadCellProperty` already reads one cell of. See abi/sim_abi_ext.h at
// `kExtMaxFieldSources` for the whole argument.

__declspec(dllexport) int32_t SIM_ExtFieldCount() {
  if (!g) return 0;
  WaitIdle();
  return static_cast<int32_t>(g->fields.registry.Data().size());
}

// Takes the PROPERTY's name, because a field has no name of its own. Returns the field index,
// or -1 for an unregistered name, a property with no field attached, a null pointer or no sim.
__declspec(dllexport) int32_t SIM_ExtFieldIndex(const char* propertyName) {
  if (!g || propertyName == nullptr) return -1;
  WaitIdle();
  const int32_t property_idx = g->world.ExtCells().Find(std::string(propertyName));
  if (property_idx < 0) return -1;
  return g->fields.registry.FindByProperty(property_idx);
}

// Fills `out` and returns 1, or returns 0 leaving `out` untouched for an unregistered index,
// a null pointer, or no sim.
__declspec(dllexport) int32_t SIM_ExtFieldDescribe(int32_t fieldIdx, ext::ExtFieldDesc* out) {
  if (!g || out == nullptr) return 0;
  WaitIdle();
  const std::vector<ext::Field>& fields = g->fields.registry.Data();
  if (fieldIdx < 0 || static_cast<size_t>(fieldIdx) >= fields.size()) return 0;
  const ext::Field& f = fields[static_cast<size_t>(fieldIdx)];

  ext::ExtFieldDesc d{};
  // The property's name rather than an index alone, so a caller can match a field to the
  // property table it already has without a second lookup. A field whose property has somehow
  // gone away describes with an empty name rather than refusing -- the row still exists.
  const ext::CellPropertyRegistry::Property* p = g->world.ExtCells().At(f.property);
  if (p != nullptr) {
    const size_t n = p->name.size() < sizeof(d.property) - 1 ? p->name.size()
                                                             : sizeof(d.property) - 1;
    memcpy(d.property, p->name.data(), n);
  }
  d.fieldIdx = fieldIdx;
  d.propertyIdx = f.property;
  d.attributeIdx = f.attribute;
  d.attributeFallback = f.fallback;
  d.law = f.law;
  d.combine = f.combine;
  d.decayMode = f.decay_mode;
  d.decayKeep = f.decay_keep;
  d.floorValue = f.floor_value;
  d.clampLo = f.clamp_lo;
  d.clampHi = f.clamp_hi;
  d.sourceCount = static_cast<int32_t>(f.sources.size());
  *out = d;
  return 1;
}

// Layer C (kSetCellPropertyTransport). How much of one component of a liquid-carried property
// left the grid in a record no subscriber took, since the DLL was initialised. Returns 0 with no
// sim or an index out of range.
__declspec(dllexport) int32_t SIM_ExtLiquidPayloadUnreported(int32_t propertyIdx,
                                                             int32_t component, double* out) {
  if (!g || out == nullptr || propertyIdx < 0 || component < 0 ||
      component >= ext::kExtMaxArity) {
    return 0;
  }
  WaitIdle();
  const size_t k = static_cast<size_t>(propertyIdx) * static_cast<size_t>(ext::kExtMaxArity) +
                   static_cast<size_t>(component);
  if (k >= g->payload_unreported.size()) return 0;
  *out = g->payload_unreported[k];
  return 1;
}

// The properties whose owner declared `kRehydrated` — "the sim must not carry this, I re-push
// it after every load" — and has not written a single cell since the world was allocated.
//
// This is what turns that declaration from a comment into a claim somebody can check. Fills
// `out` with up to `max` property indices and returns how many there are in total (which may
// exceed `max`, so a caller that cares can size and ask again). Returns 0 with no world.
//
// Stage 2b is what makes it load-bearing, and corrected what this comment used to claim. The
// save blob restores `kSaved`; the extension checkpoint (ext::kSetExtCellState) restores BOTH
// `kRehydrated` and `kCheckpointOnly`. So a caller that carries all seven checkpoint components
// sees this list empty, and a caller that carries only the save blob sees exactly the set
// somebody owes a re-push for.
__declspec(dllexport) int32_t SIM_ExtOutstandingRehydration(int32_t* out, int32_t max) {
  if (!g) return 0;
  WaitIdle();
  const std::vector<int32_t> v = g->world.ExtCells().OutstandingRehydration();
  const int32_t n = static_cast<int32_t>(v.size());
  if (out != nullptr) {
    for (int32_t i = 0; i < n && i < max; ++i) out[i] = v[static_cast<size_t>(i)];
  }
  return n;
}

__declspec(dllexport) void SIM_DebugCrash() {
  volatile int* p = nullptr;
  *p = 0;
}

__declspec(dllexport) char* SYSINFO_Acquire() {
  static char info[] = "oni-sim-replacement";
  return info;
}

__declspec(dllexport) void SYSINFO_Release() {}

// kprofiler: the sixteen exports `Klei/KProfilerPlugin.cs` declares, and the profiler behind
// them, `sim/kprofiler.h`. Signatures are the managed declarations -- `string` under the default
// `CharSet.Ansi` is `const char*`, `ulong` is `uint64_t`, `long` is `int64_t`. Inert until
// something loads the plugin and starts a capture: every recording call returns on one atomic
// load while the state is not "running". Nothing in the shipped game calls these (the managed
// call sites are compiled out); the development install's devpatch does.
__declspec(dllexport) void kprofiler_load_plugin() { kprof::LoadPlugin(); }
__declspec(dllexport) void kprofiler_unload_plugin() { kprof::UnloadPlugin(); }
__declspec(dllexport) void kprofiler_start_http_control_listener(int port) {
  kprof::StartHttpControlListener(port);
}
__declspec(dllexport) void kprofiler_start_http_data_sender(int port) {
  kprof::ReplaceBroadcaster(new kprof::HttpBroadcaster(port));
}
__declspec(dllexport) void kprofiler_start_file_data_sender(const char* filename) {
  kprof::ReplaceBroadcaster(new kprof::FileBroadcaster(filename));
}
__declspec(dllexport) void kprofiler_flush_data_sender() { kprof::FlushDataSender(); }
__declspec(dllexport) void kprofiler_stop_data_sender() { kprof::ReplaceBroadcasterQuietly(nullptr); }
__declspec(dllexport) void kprofiler_start_profiling() { kprof::StartProfiling(); }
__declspec(dllexport) void kprofiler_stop_profiling(int broadcast_info) {
  kprof::StopProfiling(broadcast_info != 0);
}
__declspec(dllexport) uint64_t kprofile_record_string(const char* str) {
  return str == nullptr ? 0 : kprof::G().strings.Record(str);
}
__declspec(dllexport) uint64_t kprofiler_get_thread_uid() { return kprof::ThreadUid(); }
__declspec(dllexport) void kprofiler_set_thread_info(uint64_t thread_id, uint64_t name,
                                                     uint64_t category) {
  kprof::SetThreadInfo(thread_id, name, category);
}
__declspec(dllexport) void kprofiler_begin_section(uint64_t name, uint64_t category,
                                                   int64_t gc_alloc_count) {
  kprof::BeginSection(name, category, gc_alloc_count);
}
__declspec(dllexport) void kprofiler_end_section(int64_t gc_alloc_count) {
  kprof::EndSection(gc_alloc_count);
}
__declspec(dllexport) void kprofiler_ping(uint64_t name, uint64_t category, double value) {
  kprof::Ping(name, category, value);
}
__declspec(dllexport) void kprofiler_counter(uint64_t name, double value) {
  kprof::Counter(name, value);
}

// ConduitTemperatureManager, `sim/conduits.h`. These run on the *game* thread between sim
// frames, not inside a frame, which is why they touch `g->conduits` directly instead of
// queueing a message the way every cell-facing export does.
__declspec(dllexport) void ConduitTemperatureManager_Initialize() {
  if (g) g->conduits.Clear();
}
__declspec(dllexport) void ConduitTemperatureManager_Shutdown() {
  if (g) g->conduits.Clear();
}

__declspec(dllexport) int ConduitTemperatureManager_Add(
    float contents_temperature, float contents_mass, int contents_element_hash,
    int conduit_structure_temperature_handle, float conduit_heat_capacity,
    float conduit_thermal_conductivity, int32_t conduit_insulated) {
  if (!g) return 0;
  return g->conduits.Add(g->elements, contents_temperature, contents_mass,
                         contents_element_hash, conduit_structure_temperature_handle,
                         conduit_heat_capacity, conduit_thermal_conductivity,
                         conduit_insulated != 0);
}

__declspec(dllexport) void ConduitTemperatureManager_Remove(int handle) {
  if (g) g->conduits.Remove(handle);
}

// Klei returns the handle it was given rather than a new one; the game ignores the result.
__declspec(dllexport) int ConduitTemperatureManager_Set(int handle, float contents_temperature,
                                                        float contents_mass,
                                                        int contents_element_hash) {
  if (g) {
    g->conduits.Set(g->elements, handle, contents_temperature, contents_mass,
                    contents_element_hash);
  }
  return handle;
}

__declspec(dllexport) void ConduitTemperatureManager_Clear() {
  if (g) g->conduits.Clear();
}

// The conduit run read model and its policies, `sim/conduits.h`. Game thread, no
// `WaitIdle`, for the same reason as the seven Klei exports above -- and that reason holds without
// exception, because the handle release runs in `PrepareGameData`, not on the worker (see `ReleaseConduitHandlesForNextFrame`).
__declspec(dllexport) int32_t SIM_ConduitNetworkBind(int32_t conduitType, const int32_t* handles,
                                                     const int32_t* cells,
                                                     const int32_t* networkIds,
                                                     const int32_t* neighbourHandles,
                                                     int32_t count, float volumePerConduitM3,
                                                     float maxMassPerConduitKg) {
  if (!g) return -1;
  return g->conduits.Bind(conduitType, handles, cells, networkIds, neighbourHandles, count,
                          volumePerConduitM3, maxMassPerConduitKg);
}

__declspec(dllexport) int32_t SIM_ConduitNetworkPolicy(int32_t conduitType, int32_t networkId,
                                                       int32_t flags, float mixFraction,
                                                       float phaseRatePerSecond,
                                                       float phaseMinRemainderKg) {
  if (!g) return 0;
  return g->conduits.SetPolicy(conduitType, networkId, flags, mixFraction, phaseRatePerSecond,
                               phaseMinRemainderKg)
             ? 1
             : 0;
}

__declspec(dllexport) int32_t SIM_ConduitTrappedSet(int32_t conduitType, int32_t cell,
                                                    int32_t elementIdx, float massKg,
                                                    float temperatureK) {
  if (!g) return 0;
  return g->conduits.TrappedSet(g->elements, conduitType, cell, elementIdx, massKg, temperatureK)
             ? 1
             : 0;
}

__declspec(dllexport) int32_t SIM_ConduitTrappedClear(int32_t conduitType) {
  if (!g) return -1;
  return g->conduits.TrappedClear(conduitType);
}

// Total returned, up to `capacity` copied: the sizing contract every buffer-filling read-back
// in this file uses.
__declspec(dllexport) int32_t SIM_ConduitPhaseProposals(OniConduitPhaseProposal* out,
                                                        int32_t capacity) {
  if (!g) return 0;
  const std::vector<OniConduitPhaseProposal>& all = g->conduits.PhaseProposals();
  const int32_t total = static_cast<int32_t>(all.size());
  if (out && capacity > 0) {
    const int32_t n = total < capacity ? total : capacity;
    memcpy(out, all.data(), static_cast<size_t>(n) * sizeof(OniConduitPhaseProposal));
  }
  return total;
}

// The caller's buffer is always written when it is large enough to hold `networkId`, so a
// caller that ignores the return value reads -1 rather than whatever was there. The struct is
// built whole and then copied out by the size the caller declared, which is what lets it grow
// at the end without breaking a caller compiled against this shape.
__declspec(dllexport) int32_t SIM_ConduitNetworkAggregate(int32_t conduitType, int32_t networkId,
                                                          OniConduitNetworkAggregate* out,
                                                          int32_t outSize) {
  if (!out || outSize < static_cast<int32_t>(sizeof(int32_t))) return 0;
  OniConduitNetworkAggregate full;
  memset(&full, 0, sizeof(full));
  full.networkId = -1;
  const bool filled = g && g->conduits.Aggregate(g->elements, conduitType, networkId, &full);
  const size_t n = static_cast<size_t>(outSize) < sizeof(full) ? static_cast<size_t>(outSize)
                                                                 : sizeof(full);
  memcpy(out, &full, n);
  return filled ? 1 : 0;
}

__declspec(dllexport) void* ConduitTemperatureManager_Update(float dt,
                                                             void* building_conductivity_data) {
  if (!g) return nullptr;
  // The energy owed to each building goes onto the same queue the game's own messages land
  // on, in the order the conduits are walked, and is drained by the next frame. Applying it
  // here would let a building be warmed in the middle of a frame it is already part of.
  const auto sink = [](void* ctx, const ModifyBuildingEnergyMessage& m) {
    Sim* s = static_cast<Sim*>(ctx);
    Sim::Pending p;
    p.id = static_cast<int32_t>(SimMessageHash::ModifyBuildingEnergy);
    p.payload.resize(sizeof(m));
    memcpy(p.payload.data(), &m, sizeof(m));
    s->queue.push_back(std::move(p));
  };
  const auto* temperatures = static_cast<const BuildingTemperatureInfo*>(building_conductivity_data);
  // The cells of the frame the game holds, for the CONVECTION policy's cell side. See
  // `game_frame`: that frame is not the one the worker is writing, so no barrier is needed.
  ConduitCellFrame cells;
  if (g->game_frame != nullptr) {
    cells.element = g->game_frame->elementIdx;
    cells.mass = g->game_frame->mass;
    cells.temperature = g->game_frame->temperature;
    cells.count = g->game_frame_cells;
  }
  return const_cast<ConduitTemperatureUpdateData*>(
      g->conduits.Update(g->elements, dt, temperatures, sink, g, cells));
}

}  // extern "C"

BOOL APIENTRY DllMain(HMODULE, DWORD, LPVOID) { return TRUE; }
