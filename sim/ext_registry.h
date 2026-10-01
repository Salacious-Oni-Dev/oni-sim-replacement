// The per-cell extension property registry.
//
// One registration replaces what every id above `kRegisterCellProperty` in
// `abi/sim_abi_ext.h` had to spell out by hand: a `std::vector` member in `world.h`, a line
// in `World::Allocate`, a handler in `simdll.cpp`, and -- for a property meant to outlive a
// save -- a `saveblob.h` field plus a new `kSaveVersion` integer. See that header's
// "THE PER-CELL EXTENSION REGISTRY" block for why the edit count is the least interesting
// part of the argument.
//
// STORAGE is one type-erased byte vector per property plus a stride, sized by the same
// `World::Allocate` that sizes `insulation_` and `strength_`. So an extension property is
// exactly `PaddedCount()` entries long, allocated at the same moment as every other per-cell
// array, and stays that length for the life of the world -- the invariant the rest of
// `world.h` already assumes everywhere.
//
// TYPE ERASURE, AND WHY THE HOT PATH DOES NOT PAY FOR IT. `physics.h`'s conduction kernel
// reads the thermal-mass bonus as a raw `const float*` hoisted out of the pair loop, exactly
// as it read `ThermalMassBonus().data()` before this existed. `F32()` below is that hoist:
// one bounds check and one `.data()`, once per kernel invocation, and the inner loop indexes
// a plain float array. A `std::vector<uint8_t>`'s buffer comes from `::operator new` and is
// therefore aligned for any fundamental type, so the reinterpret is well-defined for every
// scalar this registry stores.
//
// REGISTRATION CLOSES AT THE FIRST ALLOCATE, and a late registration is refused rather than
// serviced by a resize. Growing the set mid-world would either leave one array shorter than
// the rest or force a reallocation that invalidates a pointer a kernel is already holding.
// Mods register at load; worlds are allocated later; the ordering is not a burden in
// practice. Refusing loudly is what makes it not a burden in theory either.
//
// WRITE COUNTING is not diagnostics for its own sake. `kRehydrated` is a CLAIM by the
// registering mod that it re-pushes the property after every load, and the counter is the
// only thing that can turn that claim into an answerable question: "which properties was
// somebody supposed to re-push, and has not". Stage 2b consumes it. It costs nothing --
// extension properties are written by messages, never by kernels, so this counts message
// traffic and never touches the physics inner loop.
#pragma once

#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <string>
#include <vector>

#include "../abi/sim_abi_ext.h"

namespace oni_sim::ext {

// Bytes per cell for each scalar. 0 means "not a scalar type", which is how the validator
// rejects a `type` field outside the enum.
inline size_t ScalarStride(int32_t type) {
  switch (type) {
    case kExtF32: return 4;
    case kExtU8: return 1;
    case kExtU16: return 2;
    case kExtI32: return 4;
    case kExtElementIdx: return 2;
    default: return 0;
  }
}

inline bool ValidPersistence(int32_t p) {
  return p == kSaved || p == kRehydrated || p == kCheckpointOnly;
}

// `<owner>.<property>`, both segments [a-z0-9_], owner 2-31, property 1-31, whole name at
// most 47 chars so it fits `RegisterCellPropertyMessage::name[48]` with its terminator.
//
// The total-length rule is not redundant with the segment rules: 31 + 1 + 31 is legal by
// segment and does not fit the field. It is rejected here, with a reason, rather than
// truncated into a different name than the caller asked for.
//
// LOWERCASE IS ENFORCED, NOT NORMALISED -- see the header. An uppercase letter is a rejection,
// because down-casing it would map two distinct names onto one store.
inline bool ValidPropertyName(const std::string& name, std::string* why) {
  auto fail = [&](const char* msg) {
    if (why) *why = msg;
    return false;
  };
  if (name.empty()) return fail("empty name");
  if (name.size() > 47) return fail("name longer than 47 characters");
  const size_t dot = name.find('.');
  if (dot == std::string::npos) return fail("no '.' separator; names are <owner>.<property>");
  if (name.find('.', dot + 1) != std::string::npos) return fail("more than one '.' separator");
  const std::string owner = name.substr(0, dot);
  const std::string prop = name.substr(dot + 1);
  if (owner.size() < 2 || owner.size() > 31) return fail("owner segment must be 2-31 chars");
  if (prop.empty() || prop.size() > 31) return fail("property segment must be 1-31 chars");
  for (char c : name) {
    if (c == '.') continue;
    const bool ok = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_';
    if (!ok) {
      if (c >= 'A' && c <= 'Z') return fail("uppercase is rejected, not down-cased");
      return fail("illegal character; segments are [a-z0-9_]");
    }
  }
  return true;
}

// `sim.` and `oni.` belong to this project, so a first-party key is visibly first-party in a
// blob a third party is reading.
inline bool IsReservedOwner(const std::string& name) {
  return name.compare(0, 4, "sim.") == 0 || name.compare(0, 4, "oni.") == 0;
}

// The `<owner>` half of a registered name. Free rather than a private static of the per-cell
// registry since stage 5, because the per-element registry (`ext_elements.h`) shares this
// namespace and reports duplicate collisions the same way.
inline std::string OwnerOf(const std::string& name) {
  const size_t dot = name.find('.');
  return dot == std::string::npos ? name : name.substr(0, dot);
}

class CellPropertyRegistry {
 public:
  // A cap rather than unbounded growth: the index is an ABI value handed back over a message
  // return, the descriptors are walked linearly by `Find`, and a mod loop registering in a
  // loop should hit a wall with a name in the log rather than an allocation failure.
  static constexpr int32_t kMaxProperties = 64;

  struct Property {
    std::string name;
    int32_t type = kExtF32;
    int32_t persist = kRehydrated;
    // Components per cell, >= 1. Cell-major: component `c` of cell `p` lives at
    // `(p * arity + c) * stride`. See kExtMaxArity in the ABI header for why that layout.
    int32_t arity = 1;
    uint32_t default_bits = 0;
    bool first_party = false;
    // Message writes since the last `Allocate`. See the header: this is what makes a
    // `kRehydrated` claim checkable.
    uint64_t writes_since_allocate = 0;
    // Set when the extension checkpoint (stage 2b, `sim/ext_state.h`) restored this property
    // from a blob. A SEPARATE FLAG rather than a bump of the counter above: that counter means
    // "how many message writes have landed", and a restore is not a message write. Giving one
    // field two meanings is how `thermal_mass_bonus_`'s persistence gap started.
    //
    // It exists because `OutstandingRehydration` would otherwise be useless to the caller it
    // was built for. A replay harness that restored a `kRehydrated` property CORRECTLY would
    // still be told it was outstanding, so the list would fire on every seek and mean nothing.
    bool restored_by_checkpoint = false;
    // This property is copied into every published frame's descriptor table.
    // A SUBSCRIPTION, not a property of the data -- so unlike the two fields above it
    // SURVIVES an `Allocate`. A caller that asked to read a window into the world asked about
    // the world, not about one allocation of it, and making it re-subscribe after every load
    // would mean a silent empty table on exactly the tick a load makes interesting.
    bool published = false;
    // This property is an EXTENSIVE amount carried
    // by a cell's liquid, and the liquid kernels move it with the liquid's mass -- the share of a
    // cell's liquid that leaves takes the same share of every component with it. F32 only.
    // Survives an `Allocate` for the reason `published` does: it describes what the data IS,
    // not one allocation of it. Set by `ext::kSetCellPropertyTransport` or first-party code.
    bool follows_liquid_mass = false;
    // Conservative "anything nonzero here?" for a follows-liquid property, so a world that never
    // wrote one pays a load and a branch per liquid transfer and nothing else. Set by every write
    // and every load; cleared only by `Allocate`. Never cleared by a kernel that happens to drive
    // the data back to zero -- being wrong in that direction costs a few flops, not correctness.
    bool maybe_nonzero = false;
    // Layer C mixing (`ext::kSetPayloadMixing`). The fraction of a neighbour pair's
    // CONCENTRATION gap `StepPayloadMixing` closes per pair per substep. Zero is "do not mix",
    // which is every property's default; the sim gives its own
    // `sim.dissolved_mass` `ext::kPayloadMixingDefaultShare` at registration and gives a mod's
    // property nothing, because whether somebody else's tracer is stirred is not ours to decide.
    // Survives an `Allocate` for the reason `follows_liquid_mass` and `published` do: it
    // describes what the data IS, not one allocation of it.
    float mix_share = 0.0f;
  };

  // Returns the property index (>= 0), or a NEGATED `ExtRegisterResult` on refusal. `error`
  // is filled with a human-readable reason on every refusal -- including, for a duplicate,
  // the name of the mod that already holds it, because a collision that logs one name is a
  // collision nobody can act on.
  int32_t Register(const std::string& name, int32_t type, int32_t persist, int32_t arity,
                   uint32_t default_bits, bool first_party, std::string* error) {
    auto refuse = [&](ExtRegisterResult code, const std::string& msg) {
      if (error) *error = msg;
      return -static_cast<int32_t>(code);
    };
    std::string why;
    if (!ValidPropertyName(name, &why)) {
      return refuse(kExtRegisterBadName, "extension property \"" + name + "\": " + why);
    }
    if (ScalarStride(type) == 0) {
      return refuse(kExtRegisterBadType,
                    "extension property \"" + name + "\": unknown scalar type " +
                        std::to_string(type));
    }
    if (arity < 1 || arity > kExtMaxArity) {
      return refuse(kExtRegisterBadArity,
                    "extension property \"" + name + "\": arity must be 1.." +
                        std::to_string(kExtMaxArity) + ", not " + std::to_string(arity));
    }
    if (!ValidPersistence(persist)) {
      return refuse(kExtRegisterBadType,
                    "extension property \"" + name + "\": persistence is required and must be "
                    "kSaved, kRehydrated or kCheckpointOnly, not " + std::to_string(persist));
    }
    // `kSaved` was refused through stage 1, because accepting it with no serialiser behind it
    // would have given a caller a property that silently behaved like `kRehydrated` -- exactly
    // the failure the registry exists to make impossible. Stage 2 built the serialiser
    // (`kSaveVersionExtensions` in saveblob.h, `World::ToBlob`/`FromBlob`), so it is accepted
    // now. `kExtRegisterUnsupported` is kept in the ABI enum rather than renumbered: the
    // refusal codes are wire values.
    if (!first_party && IsReservedOwner(name)) {
      return refuse(kExtRegisterReserved,
                    "extension property \"" + name + "\": the \"sim.\" and \"oni.\" owners are "
                    "reserved for first-party properties");
    }
    if (closed_) {
      return refuse(kExtRegisterClosed,
                    "extension property \"" + name + "\": registration closed at the first "
                    "world allocate; register before the world is created");
    }
    const int32_t existing = Find(name);
    if (existing >= 0) {
      return refuse(kExtRegisterDuplicate,
                    "extension property \"" + name + "\" is already registered by \"" +
                        OwnerOf(props_[static_cast<size_t>(existing)].name) +
                        "\"; the second registration came from \"" + OwnerOf(name) + "\"");
    }
    if (static_cast<int32_t>(props_.size()) >= kMaxProperties) {
      return refuse(kExtRegisterFull,
                    "extension property \"" + name + "\": registry is full (" +
                        std::to_string(kMaxProperties) + " properties)");
    }
    Property p;
    p.name = name;
    p.type = type;
    p.persist = persist;
    p.arity = arity;
    p.default_bits = default_bits;
    p.first_party = first_party;
    props_.push_back(p);
    storage_.emplace_back();
    return static_cast<int32_t>(props_.size()) - 1;
  }

  int32_t Find(const std::string& name) const {
    for (size_t i = 0; i < props_.size(); ++i) {
      if (props_[i].name == name) return static_cast<int32_t>(i);
    }
    return -1;
  }

  int32_t Count() const { return static_cast<int32_t>(props_.size()); }
  bool Closed() const { return closed_; }
  bool Valid(int32_t idx) const {
    return idx >= 0 && static_cast<size_t>(idx) < props_.size();
  }
  const Property* At(int32_t idx) const {
    return Valid(idx) ? &props_[static_cast<size_t>(idx)] : nullptr;
  }

  // Sizes every registered property to `padded_count` cells at its registered default, and
  // closes registration. Called only from `World::Allocate`, alongside every other per-cell
  // array, so an extension property is never a different length from `insulation_`.
  void Allocate(size_t padded_count) {
    cells_ = padded_count;
    closed_ = true;
    for (size_t i = 0; i < props_.size(); ++i) {
      Property& p = props_[i];
      const size_t stride = ScalarStride(p.type);
      const size_t lanes = padded_count * static_cast<size_t>(p.arity);
      storage_[i].assign(lanes * stride, 0);
      // `assign` has already written every scalar's zero bit pattern, float included, so the
      // common case is done. Only a nonzero default needs the second pass -- calling
      // `FillDefault` unconditionally would memset ten megabytes twice on a full-colony load
      // for no change in the bytes.
      if (p.default_bits != 0) FillDefault(static_cast<int32_t>(i));
      // A fresh allocate is a fresh world: whatever a `kRehydrated` owner pushed into the
      // previous one says nothing about this one, and neither does a checkpoint restored
      // into it.
      p.writes_since_allocate = 0;
      p.restored_by_checkpoint = false;
      p.maybe_nonzero = false;
    }
  }

  // Layer C. Flags an F32 property as carried by liquid mass (see `Property::follows_liquid_mass`).
  // Refuses any other type: only an amount can be split in proportion, and an element index or a
  // flag byte halved on its way out of a cell is not a smaller one of the same thing.
  bool SetFollowsLiquidMass(int32_t idx, bool follows) {
    if (!Valid(idx)) return false;
    Property& p = props_[static_cast<size_t>(idx)];
    if (p.type != kExtF32) return false;
    p.follows_liquid_mass = follows;
    return true;
  }

  // Layer C mixing. Clamped rather than refused -- see `ext::SetPayloadMixingMessage::share`.
  // Refuses a property that is not F32 for the same reason `SetFollowsLiquidMass` does: a mixing
  // rate is a rate at which two AMOUNTS even out, and an element index halfway between two cells
  // is not a third element.
  bool SetMixShare(int32_t idx, float share) {
    if (!Valid(idx)) return false;
    Property& p = props_[static_cast<size_t>(idx)];
    if (p.type != kExtF32) return false;
    p.mix_share = share < 0.0f ? 0.0f : (share > 1.0f ? 1.0f : share);
    return true;
  }

  // Marks a property as possibly holding data. Called by kernels that CREATE an amount in a
  // follows-liquid property (the Layer C exchange), which bypass `Write`.
  void NoteNonzero(int32_t idx) {
    if (Valid(idx)) props_[static_cast<size_t>(idx)].maybe_nonzero = true;
  }

  // Rewrites every component of every cell with the property's registered default.
  //
  // Two callers, and the second is the reason this is a function rather than a loop inside
  // `Allocate`: a checkpoint restore has to reset a property the checkpoint OMITTED, because
  // the blob omits a property that was all-default and "absent" therefore means "was default",
  // never "leave whatever is there". See `sim/ext_state.h`.
  bool FillDefault(int32_t idx) {
    if (!Valid(idx)) return false;
    const Property& p = props_[static_cast<size_t>(idx)];
    const size_t stride = ScalarStride(p.type);
    std::vector<uint8_t>& bytes = storage_[static_cast<size_t>(idx)];
    // The common case is a zero default (every scalar's zero bit pattern, float included), and
    // `memset` is what the whole-array case wants. Anything else is filled per COMPONENT -- a
    // default is a per-component value, not a per-cell one, so an arity-8 property with a
    // nonzero default gets all eight lanes set to it.
    if (p.default_bits == 0) {
      if (!bytes.empty()) memset(bytes.data(), 0, bytes.size());
      return true;
    }
    for (size_t off = 0; off + stride <= bytes.size(); off += stride) {
      memcpy(bytes.data() + off, &p.default_bits, stride);
    }
    return true;
  }

  size_t Cells() const { return cells_; }

  // Writes the low `stride` bytes of `bits` into one COMPONENT of `cell`, which is the whole
  // of the reinterpretation for all four scalar types on a little-endian target: a bit-cast
  // float, a truncated u8/u16, or an i32 taken verbatim. Returns false for an unregistered
  // property, an out-of-range cell or an out-of-range component, so a caller can tell a
  // dropped message from an applied one.
  //
  // `component` is REQUIRED rather than defaulted to 0. A default would let a caller write to
  // an arity-8 property without naming a lane and silently land on lane 0 -- which is the
  // shape of bug this registry was built to stop, not one to introduce into it.
  //
  // Deliberately does NOT call `World::MarkStatic`. Extension properties have no texture and
  // no projection into `GameDataUpdate` -- nothing on the managed side reads a per-cell
  // extension value back -- so marking them would dirty a static texture for a value that
  // never reaches one. `MutableThermalMassBonus` made the same choice before this existed.
  bool Write(int32_t idx, size_t cell, int32_t component, uint32_t bits) {
    if (!Valid(idx) || cell >= cells_) return false;
    Property& p = props_[static_cast<size_t>(idx)];
    if (component < 0 || component >= p.arity) return false;
    const size_t stride = ScalarStride(p.type);
    const size_t lane = cell * static_cast<size_t>(p.arity) + static_cast<size_t>(component);
    memcpy(storage_[static_cast<size_t>(idx)].data() + lane * stride, &bits, stride);
    ++p.writes_since_allocate;
    if (bits != p.default_bits) p.maybe_nonzero = true;
    return true;
  }

  // Zero-extends the low `stride` bytes back out. The mirror of `Write`, and the only way a
  // caller can read an extension property back: they are not projected.
  bool Read(int32_t idx, size_t cell, int32_t component, uint32_t* out) const {
    if (!Valid(idx) || cell >= cells_ || out == nullptr) return false;
    const Property& p = props_[static_cast<size_t>(idx)];
    if (component < 0 || component >= p.arity) return false;
    const size_t stride = ScalarStride(p.type);
    const size_t lane = cell * static_cast<size_t>(p.arity) + static_cast<size_t>(component);
    uint32_t bits = 0;
    memcpy(&bits, storage_[static_cast<size_t>(idx)].data() + lane * stride, stride);
    *out = bits;
    return true;
  }

  int32_t Arity(int32_t idx) const {
    return Valid(idx) ? props_[static_cast<size_t>(idx)].arity : 0;
  }

  // The kernel hoist: the base of the whole float array. Null for an unregistered index, a
  // non-float property, or a world that has not been allocated yet -- every one of which is a
  // caller bug rather than a state a kernel should be asked to handle, so the callers check
  // once and keep the pointer.
  //
  // For an arity-1 property this is indexed `[cell]`, exactly as `ThermalMassBonus().data()`
  // was. For arity > 1 it is `[cell * arity + component]`, which is what `gas_mass_`'s own
  // call sites already write.
  const float* F32(int32_t idx) const {
    if (!Valid(idx) || props_[static_cast<size_t>(idx)].type != kExtF32) return nullptr;
    const std::vector<uint8_t>& bytes = storage_[static_cast<size_t>(idx)];
    if (bytes.empty()) return nullptr;
    return reinterpret_cast<const float*>(bytes.data());
  }

  // The MUTABLE hoists, and the reason `World`'s gas/room accessors cost exactly what they
  // did as member vectors: each is fetched once by `World::Allocate` into a cached raw
  // pointer, and every accessor indexes that. Null under the same three conditions `F32` is.
  //
  // These do not count writes. `writes_since_allocate` exists to make a `kRehydrated` claim
  // checkable, and a kernel writing through a hoisted pointer is not the thing that claim is
  // about -- these four properties are `kSaved`, which the blob answers for.
  float* MutableF32(int32_t idx) {
    if (!Valid(idx) || props_[static_cast<size_t>(idx)].type != kExtF32) return nullptr;
    std::vector<uint8_t>& bytes = storage_[static_cast<size_t>(idx)];
    return bytes.empty() ? nullptr : reinterpret_cast<float*>(bytes.data());
  }
  uint16_t* MutableU16(int32_t idx) {
    if (!Valid(idx) || props_[static_cast<size_t>(idx)].type != kExtU16) return nullptr;
    std::vector<uint8_t>& bytes = storage_[static_cast<size_t>(idx)];
    return bytes.empty() ? nullptr : reinterpret_cast<uint16_t*>(bytes.data());
  }
  uint8_t* MutableU8(int32_t idx) {
    if (!Valid(idx) || props_[static_cast<size_t>(idx)].type != kExtU8) return nullptr;
    std::vector<uint8_t>& bytes = storage_[static_cast<size_t>(idx)];
    return bytes.empty() ? nullptr : bytes.data();
  }

  uint64_t WritesSinceAllocate(int32_t idx) const {
    return Valid(idx) ? props_[static_cast<size_t>(idx)].writes_since_allocate : 0;
  }

  // The properties whose owner promised to re-push after a load and has not: no message write
  // since the allocate, AND no extension checkpoint restored it. `kSaved` is excluded because
  // the save blob answers for it; `kCheckpointOnly` because it is an explicit statement that
  // nobody was ever going to re-push it, which is not an outstanding anything.
  //
  // The checkpoint clause is what keeps this answerable rather than merely true. Stage 2b's
  // carrier restores `kRehydrated` properties wholesale (`sim/ext_state.h`), and without the
  // clause a harness that restored one correctly would still be listed -- a diagnostic that
  // fires on every seek is the same as no diagnostic.
  std::vector<int32_t> OutstandingRehydration() const {
    std::vector<int32_t> out;
    for (size_t i = 0; i < props_.size(); ++i) {
      const Property& p = props_[i];
      if (p.persist == kRehydrated && p.writes_since_allocate == 0 && !p.restored_by_checkpoint) {
        out.push_back(static_cast<int32_t>(i));
      }
    }
    return out;
  }

  // Set by the extension checkpoint's restore, and by nothing else. See `restored_by_checkpoint`
  // above for why it is not a bump of the write counter.
  bool MarkRestoredByCheckpoint(int32_t idx, bool restored) {
    if (!Valid(idx)) return false;
    props_[static_cast<size_t>(idx)].restored_by_checkpoint = restored;
    return true;
  }
  bool RestoredByCheckpoint(int32_t idx) const {
    return Valid(idx) && props_[static_cast<size_t>(idx)].restored_by_checkpoint;
  }

  // ------------------------------------------------------- publish subscriptions (stage 3)
  //
  // The registry only records who asked. What that costs -- a copy into the published frame,
  // paid once per frame per subscriber -- belongs to `simdll.cpp`, which owns the frame.
  bool SetPublished(int32_t idx, bool publish) {
    if (!Valid(idx)) return false;
    props_[static_cast<size_t>(idx)].published = publish;
    return true;
  }
  bool Published(int32_t idx) const {
    return Valid(idx) && props_[static_cast<size_t>(idx)].published;
  }
  // Deliberately a COUNT and not a list of indices. `BuildUpdate` walks this every single
  // frame, and a query that returned a `std::vector` would put a heap allocation on the
  // publish path of every frame including the overwhelmingly common one where nobody has
  // subscribed to anything.
  int32_t PublishedCount() const {
    int32_t n = 0;
    for (const Property& p : props_) {
      if (p.published) ++n;
    }
    return n;
  }

  // ---------------------------------------------------------------- serialisation
  //
  // Everything below exists for `World::ToBlob`/`FromBlob`. It is deliberately raw: the
  // registry knows the shape of a property's storage and nothing about the blob's layout, and
  // `saveblob.h` knows the layout and nothing about the registry. Neither includes the other.

  const std::vector<uint8_t>& Bytes(int32_t idx) const { return storage_[static_cast<size_t>(idx)]; }

  // Bytes one property occupies: cells * arity * stride. Zero before the allocate.
  size_t ByteCount(int32_t idx) const {
    if (!Valid(idx)) return 0;
    const Property& p = props_[static_cast<size_t>(idx)];
    return cells_ * static_cast<size_t>(p.arity) * ScalarStride(p.type);
  }

  // Restores a property's storage wholesale from a blob. Refuses a length that does not match
  // what this build's registration says the property is, rather than reading a prefix of it --
  // a mod that changed its own arity or type between versions is a case to report, not to
  // reinterpret. Does NOT touch `writes_since_allocate`: a load is not a re-push, and a
  // `kSaved` property is not on the outstanding list anyway.
  bool LoadBytes(int32_t idx, const uint8_t* src, size_t n) {
    if (!Valid(idx) || src == nullptr) return false;
    if (n != ByteCount(idx)) return false;
    memcpy(storage_[static_cast<size_t>(idx)].data(), src, n);
    props_[static_cast<size_t>(idx)].maybe_nonzero = true;
    return true;
  }

  // Bytes one CELL occupies in a property: arity * stride. The unit the sub-region load path
  // copies in, because that path maps a blob's local cell index onto a different global one and
  // so cannot restore a property wholesale.
  size_t CellChunk(int32_t idx) const {
    if (!Valid(idx)) return 0;
    const Property& p = props_[static_cast<size_t>(idx)];
    return static_cast<size_t>(p.arity) * ScalarStride(p.type);
  }

  // Copies one cell's worth of a property in from a foreign buffer. `src` must point at
  // `CellChunk(idx)` readable bytes.
  bool WriteCellChunk(int32_t idx, size_t cell, const uint8_t* src) {
    if (!Valid(idx) || cell >= cells_ || src == nullptr) return false;
    const size_t chunk = CellChunk(idx);
    memcpy(storage_[static_cast<size_t>(idx)].data() + cell * chunk, src, chunk);
    props_[static_cast<size_t>(idx)].maybe_nonzero = true;
    return true;
  }

  // True when any component of any cell differs from the property's registered default.
  //
  // This is what keeps a v18 section ABSENT rather than empty. It is the direct generalisation
  // of `World::HasGasMixtureData`, which scans `gas_occupied_mask_` for the same reason, and it
  // is a save-time scan for the same reason that one is: saving is not a hot path, and a live
  // flag would have to be kept in sync by every write in the sim.
  bool DiffersFromDefault(int32_t idx) const {
    if (!Valid(idx)) return false;
    const Property& p = props_[static_cast<size_t>(idx)];
    const size_t stride = ScalarStride(p.type);
    const std::vector<uint8_t>& bytes = storage_[static_cast<size_t>(idx)];
    for (size_t off = 0; off + stride <= bytes.size(); off += stride) {
      uint32_t bits = 0;
      memcpy(&bits, bytes.data() + off, stride);
      if (bits != p.default_bits) return true;
    }
    return false;
  }

  // The properties in any of `classes`, in REGISTRATION ORDER -- which is the order they
  // appear in a blob, and the reason `World`'s constructor has a comment telling you not to
  // reorder its five first-party registrations.
  //
  // This was `SavedProperties()` through stage 2. Stage 2b needs the same walk over a
  // different subset (`{kRehydrated, kCheckpointOnly}` for the checkpoint carrier), so the
  // selector is parameterised rather than copied: one walk, two callers, and a third subset
  // costs an argument instead of a function.
  std::vector<int32_t> PropertiesWithPersistence(std::initializer_list<int32_t> classes) const {
    std::vector<int32_t> out;
    for (size_t i = 0; i < props_.size(); ++i) {
      for (int32_t c : classes) {
        if (props_[i].persist == c) {
          out.push_back(static_cast<int32_t>(i));
          break;
        }
      }
    }
    return out;
  }

  // Whether this world has anything to put in a save's extension section. False for a world
  // that registered kSaved properties and never wrote one -- which must still write a blob at
  // the legacy version, byte-identical to Klei's own format.
  bool HasSavedData() const {
    for (size_t i = 0; i < props_.size(); ++i) {
      if (props_[i].persist == kSaved &&
          DiffersFromDefault(static_cast<int32_t>(i))) {
        return true;
      }
    }
    return false;
  }

 private:
  std::vector<Property> props_;
  // Parallel to `props_`: one type-erased byte vector each. Split rather than a member of
  // `Property` so a descriptor walk (`Find`, `OutstandingRehydration`) does not drag the
  // storage through cache.
  std::vector<std::vector<uint8_t>> storage_;
  size_t cells_ = 0;
  bool closed_ = false;
};

}  // namespace oni_sim::ext
