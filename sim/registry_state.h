// The sixth checkpoint component: the four component registries, whole.
//
// Sixth of SEVEN. `sim/ext_state.h` is the seventh -- the per-cell
// extension properties a save blob does not carry -- and `abi/sim_abi_ext.h`'s
// kSetExtCellState block holds the complete list, which is now the only copy of it.
//
// A save blob is cells. It carries no part of what the game registered with the sim, and
// `AllocateCells` -- the message every load goes through -- clears all four registries
// outright (`simdll.cpp`, the AllocateCells case: `buildings.Clear()`, `chunks.Clear()`,
// `flow.Clear()`, `radiation.Clear()`). In the game that is correct and invisible, because
// the game re-registers every handle it holds after a load. A tool driving the DLL cannot:
// it is not holding those handles, and a corpus records the registrations once, at boot.
//
// **Re-sending the recorded registrations is not the same thing.** A registration is a handle on a body that then evolves every substep --
// `d->temperature = r.t_b` in `chunks.h` and its twin in `buildings.h`, `elapsed_time` and
// `offset_idx` on the emitters and consumers, `emit_timer`/`emit_step` on the radiation
// emitters -- so replaying the boot records restores the topology at BOOT values. A
// checkpoint is then restored with a registry stale by however many ticks old it is, and
// shortening the checkpoint interval makes it worse rather than better, which is the
// signature of a stale-state bug rather than a replay-length one.
//
// So the registries are carried whole instead: the objects, and the handle table around
// them. The handle table is not bookkeeping that can be rebuilt. `CompactedVector`'s removal
// swaps with the last element, so **iteration order is observable**; a stale handle has to
// keep being detected as stale; and the next `Add` after a restore has to reuse the same
// freed slot the original run's next `Add` would have. All five vectors travel.
//
// WHAT IS DELIBERATELY NOT HERE, and why:
//
//   * `RadiationState::occlusion`, one float per padded cell. It looks like carried state --
//     its own comment says it is allocated once and kept -- but `StepRadiation` **fully
//     rewrites** every cell of the region it walks, from the row below it in the same pass,
//     with 1.0 at the region's edge. Nothing reads a value written by an earlier frame, so
//     it is derived, not carried, and putting it in would add a megabyte a checkpoint to
//     move a number that the next substep overwrites.
//   * The per-frame event lists: `BuildingEvents`, `ElementChunkState::info`/`published`/
//     `melted`, `ElementFlowState::consumed`/`published_*`, `RadiationState::consumed`.
//     These are what the sim hands the game at the end of a frame. They are outputs; no
//     kernel reads them back, and a restore that is followed by a frame regenerates them.
//     `emitted` and `disease_emitted` ARE carried despite being outputs, because they are
//     indexed by handle slot and survive across frames with only part of themselves reset by
//     the export -- the cheap, faithful choice for two arrays a few entries long.
//   * `BuildingState::temperature_scale` and `to_building_temperature_scale` ARE carried.
//     `BuildingState::Clear` leaves them alone, so an Allocate does not lose them -- but a
//     restore should put back the values the checkpoint ran with rather than whatever the
//     current world last had, and the game re-sends them every frame anyway.
//   * `ElementFlowState::scratch`, which is flood-fill scratch reallocated per use.
//
// The format is one self-describing byte stream, produced and consumed by this file alone.
// A caller -- a viewer, the framework, a future live attach -- never parses it and never
// mirrors any of the sim's bookkeeping to build it, which is the whole reason it exists as a
// blob rather than as a set of typed messages: the alternative shape needed the caller to
// recompute chunk heat capacities and building registration fields, and would still have
// left the flow and radiation state uncarried.

#pragma once

#include <cstdint>
#include <cstring>
#include <utility>
#include <vector>

#include "buildings.h"
#include "chunks.h"
#include "emitters.h"
#include "radiation.h"

namespace oni_sim {
namespace registry_state {

// "ONIS" -- the component-state blob, distinct from any save blob.
inline constexpr uint32_t kMagic = 0x53494E4Fu;
// Bump on any layout change. A reader refusing an unknown version is the point: a blob is
// only ever produced and consumed by the same build in the intended use, and silently
// misreading one would corrupt a registry rather than fail.
// Version 2: `BuildingHeatExchangeData` gained kSetBuildingConvection's two fields, so a
// v1 blob's building records are a different size. `WriteTable` carries the element size as well,
// so a stale blob would be refused either way; the version says which change refused it.
inline constexpr uint32_t kVersion = 2u;

// ------------------------------------------------------------------------- the byte stream

class Writer {
 public:
  explicit Writer(std::vector<uint8_t>* out) : out_(out) {}

  void U32(uint32_t v) { Raw(&v, sizeof(v)); }
  void I32(int32_t v) { Raw(&v, sizeof(v)); }
  void F32(float v) { Raw(&v, sizeof(v)); }

  // A length-prefixed run of trivially copyable elements. The element size travels with it so
  // a reader can reject a blob written by a build whose struct grew, which is the failure a
  // bare count would let through.
  template <typename T>
  void Pod(const std::vector<T>& v) {
    I32(static_cast<int32_t>(v.size()));
    I32(static_cast<int32_t>(sizeof(T)));
    if (!v.empty()) Raw(v.data(), v.size() * sizeof(T));
  }

 private:
  void Raw(const void* p, size_t n) {
    const size_t at = out_->size();
    out_->resize(at + n);
    memcpy(out_->data() + at, p, n);
  }
  std::vector<uint8_t>* out_;
};

class Reader {
 public:
  Reader(const uint8_t* p, size_t n) : p_(p), end_(p + n) {}

  bool ok() const { return ok_; }

  uint32_t U32() { uint32_t v = 0; Raw(&v, sizeof(v)); return v; }
  int32_t I32() { int32_t v = 0; Raw(&v, sizeof(v)); return v; }
  float F32() { float v = 0.0f; Raw(&v, sizeof(v)); return v; }

  template <typename T>
  std::vector<T> Pod() {
    std::vector<T> v;
    const int32_t n = I32();
    const int32_t elem = I32();
    if (!ok_ || n < 0 || elem != static_cast<int32_t>(sizeof(T))) {
      ok_ = false;
      return v;
    }
    const size_t bytes = static_cast<size_t>(n) * sizeof(T);
    if (static_cast<size_t>(end_ - p_) < bytes) {
      ok_ = false;
      return v;
    }
    v.resize(static_cast<size_t>(n));
    if (bytes) memcpy(v.data(), p_, bytes);
    p_ += bytes;
    return v;
  }

  // Everything has been consumed and nothing overran. Checked at the end rather than trusted:
  // a blob with trailing bytes is a blob from a different layout that happened to parse.
  bool AtEnd() const { return ok_ && p_ == end_; }

 private:
  void Raw(void* dst, size_t n) {
    if (!ok_ || static_cast<size_t>(end_ - p_) < n) {
      ok_ = false;
      return;
    }
    memcpy(dst, p_, n);
    p_ += n;
  }
  const uint8_t* p_;
  const uint8_t* end_;
  bool ok_ = true;
};

// ------------------------------------------------------------------- one registry at a time

// A CompactedVector of trivially copyable T: the objects, then the four vectors of the handle
// table. See the note at the top of this file for why the table is not rebuildable.
template <typename T>
inline void WriteTable(Writer& w, const CompactedVector<T>& cv) {
  w.Pod(cv.Data());
  w.Pod(cv.Handles());
  w.Pod(cv.Index());
  w.Pod(cv.Version());
  w.Pod(cv.Free());
}

template <typename T>
inline bool ReadTable(Reader& r, CompactedVector<T>* cv) {
  std::vector<T> data = r.Pod<T>();
  std::vector<int32_t> handles = r.Pod<int32_t>();
  std::vector<int32_t> index = r.Pod<int32_t>();
  std::vector<uint8_t> version = r.Pod<uint8_t>();
  std::vector<int32_t> free = r.Pod<int32_t>();
  if (!r.ok()) return false;
  return cv->Restore(std::move(data), std::move(handles), std::move(index),
                     std::move(version), std::move(free));
}

// `BuildingToBuildingData` owns a vector, so it cannot travel as a POD block. Written as the
// table's four index vectors plus one length-prefixed contact list per live entry, in data
// order -- which is the order `Handles()` is in, so the two line up on the way back.
inline void WriteContactTable(Writer& w, const CompactedVector<BuildingToBuildingData>& cv) {
  const std::vector<BuildingToBuildingData>& data = cv.Data();
  w.I32(static_cast<int32_t>(data.size()));
  for (const BuildingToBuildingData& d : data) {
    w.I32(d.self);
    w.Pod(d.contacts);
  }
  w.Pod(cv.Handles());
  w.Pod(cv.Index());
  w.Pod(cv.Version());
  w.Pod(cv.Free());
}

inline bool ReadContactTable(Reader& r, CompactedVector<BuildingToBuildingData>* cv) {
  const int32_t n = r.I32();
  if (!r.ok() || n < 0) return false;
  std::vector<BuildingToBuildingData> data;
  data.resize(static_cast<size_t>(n));
  for (int32_t i = 0; i < n; ++i) {
    data[static_cast<size_t>(i)].self = r.I32();
    data[static_cast<size_t>(i)].contacts = r.Pod<InContactBuilding>();
    if (!r.ok()) return false;
  }
  std::vector<int32_t> handles = r.Pod<int32_t>();
  std::vector<int32_t> index = r.Pod<int32_t>();
  std::vector<uint8_t> version = r.Pod<uint8_t>();
  std::vector<int32_t> free = r.Pod<int32_t>();
  if (!r.ok()) return false;
  return cv->Restore(std::move(data), std::move(handles), std::move(index),
                     std::move(version), std::move(free));
}

// ------------------------------------------------------------------------------ the whole

inline void Save(const BuildingState& buildings, const ElementChunkState& chunks,
                 const ElementFlowState& flow, const RadiationState& radiation,
                 std::vector<uint8_t>* out) {
  out->clear();
  Writer w(out);
  w.U32(kMagic);
  w.U32(kVersion);

  w.F32(buildings.temperature_scale);
  w.F32(buildings.to_building_temperature_scale);
  WriteTable(w, buildings.exchange);
  WriteContactTable(w, buildings.contact);

  WriteTable(w, chunks.chunks);

  WriteTable(w, flow.consumers);
  WriteTable(w, flow.emitters);
  WriteTable(w, flow.disease_emitters);
  w.Pod(flow.emitted);
  w.Pod(flow.disease_emitted);

  WriteTable(w, radiation.emitters);
}

// Applies the blob or changes nothing. The registries are decoded into locals first and moved
// into place only once every one of them has parsed, so a blob that is truncated or from
// another layout leaves a working world rather than half a colony.
inline bool Load(const uint8_t* p, size_t n, BuildingState* buildings,
                 ElementChunkState* chunks, ElementFlowState* flow,
                 RadiationState* radiation) {
  if (!p || !buildings || !chunks || !flow || !radiation) return false;
  Reader r(p, n);
  if (r.U32() != kMagic) return false;
  if (r.U32() != kVersion) return false;

  const float temperature_scale = r.F32();
  const float to_building_temperature_scale = r.F32();
  if (!r.ok()) return false;

  CompactedVector<BuildingHeatExchangeData> exchange;
  CompactedVector<BuildingToBuildingData> contact;
  CompactedVector<ElementChunkData> chunk_table;
  CompactedVector<ElementConsumerData> consumers;
  CompactedVector<ElementEmitterData> emitters;
  CompactedVector<DiseaseEmitterData> disease_emitters;
  CompactedVector<RadiationEmitterData> radiation_emitters;

  if (!ReadTable(r, &exchange)) return false;
  if (!ReadContactTable(r, &contact)) return false;
  if (!ReadTable(r, &chunk_table)) return false;
  if (!ReadTable(r, &consumers)) return false;
  if (!ReadTable(r, &emitters)) return false;
  if (!ReadTable(r, &disease_emitters)) return false;
  std::vector<EmittedMassInfo> emitted = r.Pod<EmittedMassInfo>();
  std::vector<DiseaseEmittedInfo> disease_emitted = r.Pod<DiseaseEmittedInfo>();
  if (!ReadTable(r, &radiation_emitters)) return false;
  if (!r.AtEnd()) return false;

  buildings->temperature_scale = temperature_scale;
  buildings->to_building_temperature_scale = to_building_temperature_scale;
  buildings->exchange = std::move(exchange);
  buildings->contact = std::move(contact);
  chunks->chunks = std::move(chunk_table);
  flow->consumers = std::move(consumers);
  flow->emitters = std::move(emitters);
  flow->disease_emitters = std::move(disease_emitters);
  flow->emitted = std::move(emitted);
  flow->disease_emitted = std::move(disease_emitted);
  radiation->emitters = std::move(radiation_emitters);
  return true;
}

}  // namespace registry_state
}  // namespace oni_sim
