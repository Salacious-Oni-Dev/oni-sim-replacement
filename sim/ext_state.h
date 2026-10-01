// The SEVENTH checkpoint component: the per-cell extension properties a save blob does not
// carry. `ext::kSetExtCellState` is the setter, `SIM_DebugExtCellState` the getter, and the
// ABI header's block at that id is where the complete checkpoint list lives.
//
// WHY THIS IS A SEPARATE HEADER. `ext_registry.h` knows the shape of a property's storage and
// nothing about any blob layout; `saveblob.h` knows the layout and nothing about the registry;
// neither includes the other, and that separation is worth keeping. This file is the one place
// that includes both, and it is the only place that knows how to turn one into the other.
//
// WHAT TRAVELS. `kRehydrated` and `kCheckpointOnly`, and nothing else:
//
//   * `kSaved` is already in the save blob, which is checkpoint component 1. Carrying it here
//     as well would restore it twice -- from the blob, then from this -- and the second
//     restore would win, which is the same value only for as long as the two agree.
//   * Orphans (an uninstalled mod's carried bytes) live in the save blob for
//     the same reason and are excluded for the same reason.
//
// The two classes that DO travel are handled identically here. That is the point of keeping
// both: the difference is declarative -- it is what lets `OutstandingRehydration` answer "who
// promised to re-push and has not" -- rather than mechanical.
//
// ALL-DEFAULT PROPERTIES ARE OMITTED, AND ABSENT MEANS "WAS DEFAULT". `registry_state.h` made
// the same trade when it refused to carry `RadiationState::occlusion`: a checkpoint is taken
// on a ring and paid for on every seek, and `sim.thermal_mass_bonus` alone is 4 bytes a cell
// (about 1 MB on a 256,944-cell world), often for an array nothing has written. The consequence is the half that has to be got right: `Load` RESETS every
// selected property the blob did not mention back to its registered default. A restore has to
// put back the world the checkpoint ran with, not merge into whatever the current world last
// had, and "leave it alone" would silently keep a value the checkpoint never saw.
//
// APPLIED WHOLE OR NOT AT ALL, like `registry_state::Load`: every record is resolved before
// any of them is applied. A record whose shape disagrees with what this build registered
// rejects the entire blob rather than reinterpreting old bytes as a new shape; a record naming
// a property no loaded mod registered is reported and skipped, which is the same policy the
// save path uses and for the same reason (a player who uninstalls one mod must not find every
// checkpoint unusable).
#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "ext_registry.h"
#include "saveblob.h"

namespace oni_sim {
namespace ext_state {

// "ONIE" -- the extension-state blob, distinct from a save blob and from registry_state's
// "ONIS". A reader that refuses an unknown magic or version is the point: silently misreading
// one would restore garbage into a physics input rather than fail.
inline constexpr uint32_t kMagic = 0x45494E4Fu;
inline constexpr uint32_t kVersion = 1u;

// magic(4) + version(4), before the section `saveblob.h` encodes.
inline constexpr size_t kHeaderSize = 8;

// The persistence classes a checkpoint carries. Named once, here, so the setter and the getter
// cannot drift apart -- and so that "which classes" is a fact with a location rather than two
// initialiser lists that happen to match today.
inline std::vector<int32_t> CarriedProperties(const ext::CellPropertyRegistry& reg) {
  return reg.PropertiesWithPersistence({ext::kRehydrated, ext::kCheckpointOnly});
}

// EVERY registered property, all three classes. Not a checkpoint and never used as one --
// `SaveAll` below is read-only, has no setter, and `Load` refuses a `kSaved` record by name
// anyway, so a blob from here cannot be fed back in even by mistake.
//
// It exists because "what is in this cell" and "what does a checkpoint have to carry" are
// different questions with different answers, and until this there was only a way to ask the
// second. A tool inspecting a world wants the gas mixture and the promoted-room flag as much
// as it wants a mod's rehydrated property -- more, since those are the ones the sim itself
// writes -- and those are exactly the ones a checkpoint must NOT carry.
inline std::vector<int32_t> AllProperties(const ext::CellPropertyRegistry& reg) {
  return reg.PropertiesWithPersistence({ext::kSaved, ext::kRehydrated, ext::kCheckpointOnly});
}

// Turns registry storage into blob records. `skip_default` is what makes a save blob include
// every kSaved property once its section exists (one rule for whether the section is there,
// not one rule per property) while a checkpoint omits the ones with nothing in them.
inline void CollectRecords(const ext::CellPropertyRegistry& reg,
                           const std::vector<int32_t>& idxs, bool skip_default,
                           std::vector<SaveExtProperty>* out) {
  for (int32_t idx : idxs) {
    const ext::CellPropertyRegistry::Property* prop = reg.At(idx);
    if (prop == nullptr) continue;
    if (skip_default && !reg.DiffersFromDefault(idx)) continue;
    SaveExtProperty e;
    e.name = prop->name;
    e.type = prop->type;
    e.arity = prop->arity;
    e.stride = static_cast<int32_t>(ext::ScalarStride(prop->type));
    e.cell_count = static_cast<int32_t>(reg.Cells());
    e.default_bits = prop->default_bits;
    e.bytes = reg.Bytes(idx);
    out->push_back(std::move(e));
  }
}

// Matches one blob record to a registered property, or explains why it cannot be. Returns:
//
//    >= 0  the property index; the record's shape matches what this build registered
//     -1   no loaded mod registered this name -- REPORTED, not fatal
//     -2   the name is registered but the SHAPE disagrees -- fatal
//
// Shared by the save path (`World::FromBlob` and its sub-region twin) and the checkpoint, so
// the three can never disagree about what a mismatch is. The sub-region path once lacked the
// gas/room restore entirely and the bug was silent and total, which is the argument for one
// resolver rather than three.
inline int32_t ResolveRecord(const ext::CellPropertyRegistry& reg, const SaveExtProperty& e,
                             std::string* error) {
  const int32_t idx = reg.Find(e.name);
  if (idx < 0) {
    if (error) {
      *error = "blob carries extension property \"" + e.name +
               "\", which no loaded mod registered";
    }
    return -1;
  }
  const ext::CellPropertyRegistry::Property* prop = reg.At(idx);
  if (prop == nullptr) return -1;
  if (e.type != prop->type || e.arity != prop->arity ||
      e.stride != static_cast<int32_t>(ext::ScalarStride(prop->type))) {
    if (error) {
      *error = "extension property \"" + e.name +
               "\" has a different shape in this blob than the build registered (blob: type " +
               std::to_string(e.type) + " arity " + std::to_string(e.arity) + " stride " +
               std::to_string(e.stride) + "; build: type " + std::to_string(prop->type) +
               " arity " + std::to_string(prop->arity) + "); refusing rather than "
               "reinterpreting it";
    }
    return -2;
  }
  return idx;
}

// ------------------------------------------------------------------------------ the whole

// Always produces a blob, even when there is nothing to carry: a caller that got an empty
// vector back could not tell "no extension data" from "this DLL does not have the export".
// An eight-byte header with a zero count says the first unambiguously.
inline void Save(const ext::CellPropertyRegistry& reg, std::vector<uint8_t>* out) {
  std::vector<SaveExtProperty> records;
  CollectRecords(reg, CarriedProperties(reg), /*skip_default=*/true, &records);
  // Sized by hand rather than through `SaveExtSectionSize`, which returns 0 for an empty list
  // because a save blob omits the whole section in that case. A checkpoint always writes the
  // count, so the four bytes are unconditional here.
  size_t sz = kHeaderSize + 4;
  for (const SaveExtProperty& e : records) sz += SaveExtRecordSize(e);
  out->assign(sz, 0);
  memcpy(out->data(), &kMagic, 4);
  memcpy(out->data() + 4, &kVersion, 4);
  EncodeExtSection(records, out->data() + kHeaderSize);
}

// Every registered property, in the same format `Save` writes, for INSPECTION only. See
// `AllProperties` for why this exists separately, and `SIM_DebugExtCellStateAll` for the
// export that serves it.
//
// `skip_default` stays true, exactly as the checkpoint does: a property nothing has written
// is `cell_count * arity * stride` bytes of the same value, which on the reference corpus is
// about a megabyte to say "nothing here". The consequence is the same one the checkpoint
// has, and the reader has to be told: an absent property is one that was never registered OR
// one that is at its default everywhere, and this format cannot distinguish them.
inline void SaveAll(const ext::CellPropertyRegistry& reg, std::vector<uint8_t>* out) {
  std::vector<SaveExtProperty> records;
  CollectRecords(reg, AllProperties(reg), /*skip_default=*/true, &records);
  size_t sz = kHeaderSize + 4;
  for (const SaveExtProperty& e : records) sz += SaveExtRecordSize(e);
  out->assign(sz, 0);
  memcpy(out->data(), &kMagic, 4);
  memcpy(out->data() + 4, &kVersion, 4);
  EncodeExtSection(records, out->data() + kHeaderSize);
}

// Applies the blob or changes nothing. See the header for the whole-or-nothing rule and for
// why a property the blob omits is RESET rather than left alone.
//
// `reported` collects the names of records this build did not register: skipped, not fatal,
// but a caller that never hears about them cannot tell a mod it forgot to load from a
// checkpoint that simply had nothing extra in it.
inline bool Load(const uint8_t* p, size_t n, ext::CellPropertyRegistry* reg,
                 std::string* error, std::vector<std::string>* reported = nullptr) {
  auto fail = [&](const std::string& why) {
    if (error) *error = why;
    return false;
  };
  if (p == nullptr || reg == nullptr) return fail("kSetExtCellState: null argument");
  if (n < kHeaderSize) return fail("kSetExtCellState: blob shorter than its header");
  uint32_t magic = 0, version = 0;
  memcpy(&magic, p, 4);
  memcpy(&version, p + 4, 4);
  if (magic != kMagic) return fail("kSetExtCellState: bad magic");
  if (version != kVersion) {
    return fail("kSetExtCellState: unknown blob version " + std::to_string(version));
  }

  const uint8_t* const end = p + n;
  std::vector<SaveExtProperty> records;
  const uint8_t* q = DecodeExtSection(p + kHeaderSize, end, reg->Cells(), /*allow_empty=*/true,
                                      &records, error);
  if (q == nullptr) return false;
  if (q != end) return fail("kSetExtCellState: blob has bytes after its last record");

  // PASS 1 -- resolve everything before applying anything. A shape mismatch anywhere has to
  // leave the world exactly as it was, not half restored.
  std::vector<int32_t> resolved(records.size(), -1);
  for (size_t i = 0; i < records.size(); ++i) {
    std::string why;
    const int32_t idx = ResolveRecord(*reg, records[i], &why);
    if (idx == -2) return fail(why);
    if (idx == -1) {
      if (reported != nullptr) reported->push_back(records[i].name);
      continue;
    }
    // A record for a class this checkpoint does not carry is a blob from a build whose
    // registration changed persistence under the same name. Refused rather than applied: it
    // would double-restore a kSaved property the save blob has already put back.
    const ext::CellPropertyRegistry::Property* prop = reg->At(idx);
    if (prop != nullptr && prop->persist == ext::kSaved) {
      return fail("extension property \"" + records[i].name +
                  "\" is kSaved in this build, and a checkpoint must not restore it -- the "
                  "save blob already has");
    }
    resolved[i] = idx;
  }

  // PASS 2 -- apply. Every carried property the blob did NOT mention goes back to its default
  // first, so "absent" means "was default" rather than "leave whatever is there".
  std::vector<bool> mentioned(static_cast<size_t>(reg->Count()), false);
  for (size_t i = 0; i < records.size(); ++i) {
    if (resolved[i] >= 0) mentioned[static_cast<size_t>(resolved[i])] = true;
  }
  for (int32_t idx : CarriedProperties(*reg)) {
    if (mentioned[static_cast<size_t>(idx)]) continue;
    reg->FillDefault(idx);
    reg->MarkRestoredByCheckpoint(idx, true);
  }
  for (size_t i = 0; i < records.size(); ++i) {
    const int32_t idx = resolved[i];
    if (idx < 0) continue;
    if (!reg->LoadBytes(idx, records[i].bytes.data(), records[i].bytes.size())) {
      // Pass 1 checked the shape and DecodeExtSection checked the cell count against
      // `reg->Cells()`, so reaching here means the two disagree about something neither
      // checked -- worth a refusal with a name rather than a silent skip.
      return fail("extension property \"" + records[i].name +
                  "\" did not fit this world's storage");
    }
    reg->MarkRestoredByCheckpoint(idx, true);
  }
  return true;
}

}  // namespace ext_state
}  // namespace oni_sim
