// The per-element extension attribute registry, the third of the extension registries.
//
// One registration replaces what `kSetMolecularMass` had to spell out by hand: a
// `std::vector<std::pair<int32_t, float>>` member on `ElementTable`, a message id, a setter,
// a seeder, and no read-back path at all. See `abi/sim_abi_ext.h` at
// `kRegisterElementAttribute` for the contract and for the four ways this is deliberately not
// shaped like the per-cell registry.
//
// STORAGE IS SPARSE, which is the difference that drives everything else. A per-cell property
// has a value for every cell because every cell exists; an attribute of an element usually has
// a value for a handful of elements and none for the other two hundred. So each attribute
// carries a sorted key list plus a packed byte array, "unset" is a first-class answer rather
// than a default, and there is no `Allocate` and therefore no moment at which registration
// has to close.
//
// KEYS ARE SimHashes IDS, NOT TABLE INDICES, and the key list is kept sorted so a lookup is a
// `lower_bound` rather than the linear walk the molecular-mass vector used. That matters
// because `ElementTable::MolecularMassOf` is called once per cell by the mole/pressure path:
// the old walk was over however many masses the managed registry had pushed, which is not four
// once a third-party mod registers gases of its own.
//
// NOTHING HERE IS SAVED. Element attributes are content data, re-pushed by their owner after
// every load, exactly like the element table they hang off -- the ABI header argues why, and
// the absence of a `persist` field is that argument made unforgeable rather than documented.
#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "../abi/sim_abi_ext.h"
#include "ext_registry.h"

namespace oni_sim::ext {

class ElementAttributeRegistry {
 public:
  static constexpr int32_t kMaxAttributes = kExtMaxElementAttributes;

  struct Attribute {
    std::string name;
    int32_t type = kExtF32;
    // Components per element, >= 1. Element-major: component `c` of the element in slot `s`
    // lives at `(s * arity + c) * stride`, the same layout and the same reasoning as the
    // per-cell registry's `(p * arity + c) * stride`.
    int32_t arity = 1;
    size_t stride = 4;
    bool first_party = false;
    // Sorted SimHashes ids, one per element that has a value. `bytes` is parallel to it:
    // `keys.size() * arity * stride` long, and the two are resized together or not at all.
    std::vector<int32_t> keys;
    std::vector<uint8_t> bytes;
    // Message writes since the process started. The per-cell registry counts these to make a
    // `kRehydrated` claim checkable; here every attribute is unconditionally rehydrated, so
    // this exists for the same question asked without the persistence class: "was anybody
    // supposed to push this, and did they".
    uint64_t writes = 0;
  };

  // Returns the attribute index (>= 0), or a NEGATED `ExtRegisterResult` on refusal, with
  // `error` filled in on every refusal -- including which mod already holds a duplicate
  // name, because a collision that names one side is a collision nobody can act on.
  //
  // `kExtRegisterClosed` is unreachable here on purpose: there is no `Allocate` to close
  // against. The code stays in the enum because refusal codes are wire values.
  int32_t Register(const std::string& name, int32_t type, int32_t arity, bool first_party,
                   std::string* error) {
    auto refuse = [&](ExtRegisterResult code, const std::string& msg) {
      if (error) *error = msg;
      return -static_cast<int32_t>(code);
    };
    std::string why;
    // Deliberately the SAME validator the per-cell registry uses. One `<owner>.<property>`
    // namespace across both registries means a mod's name means the same thing wherever it
    // appears, and a reader who has learned one has learned the other.
    if (!ValidPropertyName(name, &why)) {
      return refuse(kExtRegisterBadName, "element attribute \"" + name + "\": " + why);
    }
    const size_t stride = ScalarStride(type);
    if (stride == 0) {
      return refuse(kExtRegisterBadType, "element attribute \"" + name +
                                             "\": unknown scalar type " + std::to_string(type));
    }
    if (arity < 1 || arity > kExtMaxArity) {
      return refuse(kExtRegisterBadArity, "element attribute \"" + name + "\": arity must be 1.." +
                                              std::to_string(kExtMaxArity) + ", not " +
                                              std::to_string(arity));
    }
    if (!first_party && IsReservedOwner(name)) {
      return refuse(kExtRegisterReserved,
                    "element attribute \"" + name + "\": the \"sim.\" and \"oni.\" owners are "
                    "reserved for first-party attributes");
    }
    const int32_t existing = Find(name);
    if (existing >= 0) {
      return refuse(kExtRegisterDuplicate,
                    "element attribute \"" + name + "\" is already registered by \"" +
                        OwnerOf(attrs_[static_cast<size_t>(existing)].name) +
                        "\"; the second registration came from \"" + OwnerOf(name) + "\"");
    }
    if (static_cast<int32_t>(attrs_.size()) >= kMaxAttributes) {
      return refuse(kExtRegisterFull, "element attribute \"" + name + "\": registry is full (" +
                                          std::to_string(kMaxAttributes) + " attributes)");
    }
    Attribute a;
    a.name = name;
    a.type = type;
    a.arity = arity;
    a.stride = stride;
    a.first_party = first_party;
    attrs_.push_back(a);
    return static_cast<int32_t>(attrs_.size()) - 1;
  }

  int32_t Find(const std::string& name) const {
    for (size_t i = 0; i < attrs_.size(); ++i) {
      if (attrs_[i].name == name) return static_cast<int32_t>(i);
    }
    return -1;
  }

  int32_t Count() const { return static_cast<int32_t>(attrs_.size()); }
  bool Valid(int32_t idx) const {
    return idx >= 0 && static_cast<size_t>(idx) < attrs_.size();
  }
  const Attribute* At(int32_t idx) const {
    return Valid(idx) ? &attrs_[static_cast<size_t>(idx)] : nullptr;
  }

  // Number of elements that have a value for this attribute. Not the element count.
  int32_t ValueCount(int32_t idx) const {
    const Attribute* a = At(idx);
    return a ? static_cast<int32_t>(a->keys.size()) : 0;
  }

  uint64_t Writes(int32_t idx) const {
    const Attribute* a = At(idx);
    return a ? a->writes : 0;
  }

  bool Has(int32_t idx, int32_t id_hash) const {
    const Attribute* a = At(idx);
    return a != nullptr && Slot(*a, id_hash) >= 0;
  }

  // Writes one component. An element with no entry yet gets one, zero-filled across every
  // component, and then this component set -- so a partially-written arity-3 attribute reads
  // back as "set, with zeros in the components nobody wrote", never as garbage.
  bool Write(int32_t idx, int32_t id_hash, int32_t component, uint32_t bits) {
    if (!Valid(idx)) return false;
    Attribute& a = attrs_[static_cast<size_t>(idx)];
    if (component < 0 || component >= a.arity) return false;
    const int32_t slot = Insert(a, id_hash);
    uint8_t* dst = a.bytes.data() +
                   (static_cast<size_t>(slot) * static_cast<size_t>(a.arity) +
                    static_cast<size_t>(component)) *
                       a.stride;
    // Little-endian truncation, byte for byte the same reinterpretation
    // `CellPropertyRegistry::Write` performs on `SetCellPropertyMessage::valueBits`.
    memcpy(dst, &bits, a.stride);
    ++a.writes;
    return true;
  }

  // Removes an element's whole entry -- every component at once, back to unset. Returns false
  // when the attribute index is bad or the element had no entry, so a caller can tell "cleared
  // something" from "there was nothing there".
  bool Clear(int32_t idx, int32_t id_hash) {
    if (!Valid(idx)) return false;
    Attribute& a = attrs_[static_cast<size_t>(idx)];
    const int32_t slot = Slot(a, id_hash);
    if (slot < 0) return false;
    const size_t chunk = static_cast<size_t>(a.arity) * a.stride;
    const size_t at = static_cast<size_t>(slot);
    a.keys.erase(a.keys.begin() + static_cast<ptrdiff_t>(at));
    a.bytes.erase(a.bytes.begin() + static_cast<ptrdiff_t>(at * chunk),
                  a.bytes.begin() + static_cast<ptrdiff_t>((at + 1) * chunk));
    ++a.writes;
    return true;
  }

  // False means UNSET (or a bad index/component), and `out` is left alone. There is no default
  // to fall back to -- see the ABI header: the fallback for an unset element attribute is
  // per-element knowledge this registry does not hold.
  bool Read(int32_t idx, int32_t id_hash, int32_t component, uint32_t* out) const {
    const Attribute* a = At(idx);
    if (a == nullptr || component < 0 || component >= a->arity) return false;
    const int32_t slot = Slot(*a, id_hash);
    if (slot < 0) return false;
    uint32_t bits = 0;
    const uint8_t* src = a->bytes.data() +
                         (static_cast<size_t>(slot) * static_cast<size_t>(a->arity) +
                          static_cast<size_t>(component)) *
                             a->stride;
    memcpy(&bits, src, a->stride);
    if (out) *out = bits;
    return true;
  }

  // The typed convenience the hot path uses. `MolecularMassOf` resolves its attribute index
  // once and calls this per cell; there is no string anywhere on that path.
  bool ReadF32(int32_t idx, int32_t id_hash, float* out) const {
    uint32_t bits = 0;
    if (!Read(idx, id_hash, 0, &bits)) return false;
    float v = 0.0f;
    memcpy(&v, &bits, sizeof(v));
    if (out) *out = v;
    return true;
  }

  bool WriteF32(int32_t idx, int32_t id_hash, float value) {
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof(bits));
    return Write(idx, id_hash, 0, bits);
  }

 private:
  static int32_t Slot(const Attribute& a, int32_t id_hash) {
    const auto it = std::lower_bound(a.keys.begin(), a.keys.end(), id_hash);
    if (it == a.keys.end() || *it != id_hash) return -1;
    return static_cast<int32_t>(it - a.keys.begin());
  }

  static int32_t Insert(Attribute& a, int32_t id_hash) {
    const auto it = std::lower_bound(a.keys.begin(), a.keys.end(), id_hash);
    const size_t at = static_cast<size_t>(it - a.keys.begin());
    if (it != a.keys.end() && *it == id_hash) return static_cast<int32_t>(at);
    const size_t chunk = static_cast<size_t>(a.arity) * a.stride;
    a.keys.insert(a.keys.begin() + static_cast<ptrdiff_t>(at), id_hash);
    a.bytes.insert(a.bytes.begin() + static_cast<ptrdiff_t>(at * chunk), chunk, 0);
    return static_cast<int32_t>(at);
  }

  std::vector<Attribute> attrs_;
};

}  // namespace oni_sim::ext
