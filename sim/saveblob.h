// The SimDLL save blob: read, write, and validate.
//
// SIM_BeginSave hands the game an opaque byte blob and SIM_HandleMessage(Load, ...)
// takes it back. Worldgen round-trips through that pair once per asteroid *before a new
// game can start*, so this is not a persistence detail — nothing boots without it.
//
// The layout here was established by driving the game's SimDLL offline and changing one
// field at a time (driver/src/savefmt.cpp), then checked against a real 9.3 MB blob saved
// by the game.
//
// A replacement sim writes and reads its own blobs, so it does not have to keep this
// format. It does have to read it, or every existing save is dead.

#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "../abi/gas_mixture_abi.h"

namespace oni_sim {

// Little-endian, no padding anywhere, no compression. Sizes are asserted below rather
// than assumed, because the game's own message structs are Pack = 4 and mixing the two
// conventions is silent corruption.
#pragma pack(push, 1)

struct SaveCell {
  int32_t elementHash;  // SimHashes value, not an index into the element table
  float temperature;    // K
  float mass;           // kg
  float radiation;      // rads
};

struct SaveDisease {
  int32_t diseaseHash;  // 0 when the cell is clean
  int32_t count;
};

struct SaveBackwall {
  int32_t elementHash;
  float mass;
  float temperature;
};

// The gas-mixture section. Our own extension, not the game's — appended only when
// `version >= kSaveVersionGasMixture`, which `World::ToBlob` only sets when the world
// actually holds gas-mixture state (see its comment). One entry per cell, same order as
// SaveCell. `dirty`/`sleeping` (abi/gas_mixture_abi.h's GasMixtureCell) are deliberately
// absent: both are transient per-substep bookkeeping, not state — a freshly loaded world's
// cells start dirty-clear and awake regardless, the same way a freshly Allocated one does.
struct SaveGasCell {
  uint8_t occupied_mask;
  uint16_t species[gas::kMaxSpeciesPerCell];
  float mass[gas::kMaxSpeciesPerCell];
};

#pragma pack(pop)

static_assert(sizeof(SaveCell) == 16, "SaveCell layout drift");
static_assert(sizeof(SaveDisease) == 8, "SaveDisease layout drift");
static_assert(sizeof(SaveBackwall) == 12, "SaveBackwall layout drift");
static_assert(sizeof(SaveGasCell) == 1 + gas::kMaxSpeciesPerCell * (2 + 4),
              "SaveGasCell layout drift");

inline constexpr char kSaveMagic[8] = {'S', 'I', 'M', 'S', 'A', 'V', 'E', '\0'};
inline constexpr int32_t kSaveVersion = 15;  // game build 744825, Klei's own format
// A world that has never activated gas-mixture state still writes and reads exactly
// kSaveVersion/the legacy layout — byte-identical to the game's blob. Only a world that actually
// has something to say about the new layer pays for the extra section, at this version.
inline constexpr int32_t kSaveVersionGasMixture = 16;
// Room promotion, so it survives save/load. Our own extension again,
// appended only when `version >= kSaveVersionRoomPromotion`, which `World::ToBlob` only sets
// when the world actually has a promoted room (see its comment). A blob at this version
// always also carries the full gas section (kSaveVersionRoomPromotion > kSaveVersionGasMixture,
// and DecodeSaveBlob's gas-section read is gated on `>= kSaveVersionGasMixture`, not `==`) —
// so ToBlob fills `gas` unconditionally whenever it fills `room_promoted`, even for a world
// whose gas-mixture layer happens to be empty at save time. Restoring the byte is all this
// needs: `World::FromBlob` only has to set `room_promoted_` before the Load handler's
// `ActivateVolumeFractions` call, because `BuildRoomGraph` already re-derives `RoomGraph.owned`
// from `World::RoomPromoted` on every rebuild (gas_rooms.h) — there is no second copy of the
// promotion bit anywhere else that also needs restoring.
inline constexpr int32_t kSaveVersionRoomPromotion = 17;
// The SELF-DESCRIBING EXTENSION SECTION, and the last save version this format should ever
// need to add.
//
// Every version above is a fixed section at a fixed offset, which is why each one cost a new
// integer. That namespace is global, ordered and monotonic: two mods that each add a per-cell
// property independently claim the same next integer and collide by construction, and neither
// can be written without knowing about the other. That -- not the boilerplate -- is the real
// blocker for third-party extension data (abi/sim_abi_ext.h says the same thing at length).
//
// This section inverts it. It carries a COUNT and then one self-describing record per
// registered `kSaved` property, so adding a property stops bumping the save version at all.
//
//   int32                      propertyCount
//   per property, 68-byte header then its bytes:
//     char[48]  name           NUL-padded; the permanent on-disk key (see sim_abi_ext.h)
//     int32     type           ExtScalarType
//     int32     arity          components per cell
//     int32     stride         bytes per component -- redundant with `type` ON PURPOSE
//     int32     cellCount      must equal the blob's own w*h
//     uint32    defaultBits    the property's registered default, per component
//     uint8[]   bytes          cellCount * arity * stride, cell-major
//
// `defaultBits` is here for the ORPHAN case and would otherwise be unnecessary: a build that
// registered the property knows its own default. A build that did NOT register it -- because
// the mod is uninstalled -- has to carry the bytes through anyway, and without this it would
// have no value but zero to fill a cell the blob does not cover. Four bytes per property to
// keep an uninstalled mod's data correct rather than merely present.
//
// `stride` is stored even though `type` implies it, because it is what lets a reader SKIP a
// record whose `type` it does not recognise. Without it, one unknown type from a newer build
// makes the rest of the section unparseable; with it, an unknown record can be carried through
// verbatim, which is what a mod uninstalled for one session needs (see EXT-REGISTRY.md).
//
// `persist` is deliberately NOT stored. It is a property of the registration, not of the data:
// a blob contains `kSaved` properties by definition, and storing it would invite a mod to
// change its own persistence between versions and leave the reader to adjudicate.
//
// ABSENT, NOT EMPTY, when there is nothing to say. A world with no gas mixture, no promoted
// room and no written extension property writes a plain v15 blob byte-identical to the game's own
// format, which the game's own DLL has to keep accepting (`diffsim` asserts `klei loads mine: accepted`). A v18 blob carrying a
// zero count would break that for every world in the game.
inline constexpr int32_t kSaveVersionExtensions = 18;

// THE ELEMENT PALETTE: v18 plus one tail after the extension section, and the reason a version
// was added after the one above promised to be the last.
//
//   int32     elementCount     the element table this blob was written against
//   int32[]   elementHash      one SimHashes id per table index, in index order
//
// Cells have always been saved by element HASH, as Klei saves them. The gas mixture's species
// (`sim.gas_species`) are table INDICES, and `ElementLoader` sorts: an element added or removed
// anywhere below a gas moves that gas's index (Mod 2's four elements land at 133; every gas in
// the game's table is at 177-208). Without it, 5 kg of mixture Oxygen saved with those four
// elements and loaded without them would come back as Hydrogen at 16x the pressure, and CO2 as
// 5 kg of Vacuum. The palette is what lets `Load` turn a
// saved index back into the element it meant.
//
// A version and not a record, because a record in the section above is per cell by
// construction (`cellCount` must equal the grid) and this is one table per blob: about 850
// bytes against the megabytes a per-cell hash would cost. It is written whenever the
// extension section is, so a world with nothing to say still writes Klei's own v15.
//
// A v18 blob has no palette, and its indices are trusted as they are, which is exactly right
// when the table has not changed since it was written and exactly as wrong as before when it
// has. There is nothing better to do with it: the table it was written against is not
// recorded anywhere.
inline constexpr int32_t kSaveVersionElementPalette = 19;
// Bounds a corrupt palette count before it becomes an allocation. Element indices are u16.
inline constexpr int32_t kSaveMaxPaletteElements = 0xFFFF;

// One registered kSaved property as it appears in a blob. `bytes` is cell-major:
// component `c` of cell `p` at `(p * arity + c) * stride`.
struct SaveExtProperty {
  std::string name;
  int32_t type = 0;
  int32_t arity = 1;
  int32_t stride = 0;
  int32_t cell_count = 0;
  uint32_t default_bits = 0;
  std::vector<uint8_t> bytes;
};

// Fixed size of one record's header, before its bytes.
inline constexpr size_t kSaveExtRecordHeader = 48 + 4 + 4 + 4 + 4 + 4;
// A blob is not allowed to claim more properties than the registry can hold. Bounds a
// corrupt count before it becomes an allocation.
inline constexpr int32_t kSaveExtMaxProperties = 64;

// magic(8) + version(4) + width(4) + height(4) + x(4) + y(4) + one byte that is zero in
// every blob observed, including a real 9.3 MB one from the game.
inline constexpr size_t kSaveHeaderSize = 29;

// OLDER KLEI BLOBS, read-only. Every blob the current game writes is kSaveVersion, but a save
// from an older build carries the sim state in the format of that build, and Klei's DLL
// (build 744825) still loads them. Measured by feeding a real 2020 blob (save 7.17,
// game build 420700, "Euphoria v22.3") and edited copies of it to Klei's DLL offline
// (driver/src/savefmt.cpp --load-blob) and diffing what it saved back out:
//
//   v13  magic(8) version(4) width(4) height(4) one byte = 21-byte header, no x/y.
//        cells  {elementHash, temperature, mass} 12 bytes each, no radiation
//        disease 8 bytes each, as now
//        then 4 bytes a cell (-1.0 inside, 0.0 on the ring in the real blob) that Klei
//        ignores: a 5.0 written there loaded as 0 rads.
//        No backwall section: every cell loads as {Vacuum, 0 kg, 0 K}.
//        THE RING IS NOT READ: a ring cell edited in the blob kept the value AllocateCells
//        gave it, which is why the top row came back 9999 kg of Vacuum rather than the
//        blob's Unobtanium.
//   v14  the v15 layout. A v14-stamped copy of a v15 blob loads identically to the v15 one
//        (the five cells that differed were a second load-time transition). Whether a v14
//        blob's ring is read is NOT measured: the test blob's ring equalled AllocateCells'.
//   v12  accepted by Klei in a layout that is neither of the above; not read here.
//
// Nothing ever writes these: a world loaded from one saves back as kSaveVersion, as Klei's does.
inline constexpr int32_t kSaveVersionKleiNoBackwall = 13;
inline constexpr size_t kSaveHeaderSizeV13 = 21;
inline constexpr size_t kSaveCellSizeV13 = 12;
inline constexpr size_t kSaveUnreadSizeV13 = 4;
inline constexpr int32_t kSaveBackwallVacuumHash = 758759285;

struct SaveBlob {
  int32_t version = kSaveVersion;
  // Padded dimensions: the sim keeps a one-cell border ring, so these are the game's
  // grid size plus two in each direction and the arrays cover the border as well.
  int32_t width = 0;
  int32_t height = 0;
  int32_t x = 0;  // world offset, straight from SIM_BeginSave's arguments
  int32_t y = 0;
  uint8_t trailing = 0;  // header byte 28, purpose unknown, always zero so far
  // False only for a decoded kSaveVersionKleiNoBackwall blob, whose ring Klei does not read;
  // World::FromBlob then keeps the ring its own Allocate laid down.
  bool ring_from_blob = true;

  std::vector<SaveCell> cells;
  std::vector<SaveDisease> disease;
  std::vector<SaveBackwall> backwall;
  // Same length as `cells` when `version >=
  // kSaveVersionGasMixture`; empty otherwise (the common case — see kSaveVersionGasMixture's
  // comment).
  std::vector<SaveGasCell> gas;
  // Same length as `cells` when `version >=
  // kSaveVersionRoomPromotion`; empty otherwise. One byte per cell, straight off
  // `World::RoomPromoted` — no packing, no room-index indirection, so a room split or merged
  // by a later `BuildRoomGraph` rebuild still resolves correctly (the per-cell bit is the
  // source of truth, not a stored room id).
  std::vector<uint8_t> room_promoted;
  // Non-empty only when `version >=
  // kSaveVersionExtensions`, and a v18 blob is only written when there is at least one
  // registered kSaved property with something in it -- see kSaveVersionExtensions' comment on
  // why the section is absent rather than empty.
  std::vector<SaveExtProperty> ext;
  // kSaveVersionElementPalette: the element hash at each table index when this blob was written.
  // Non-empty exactly when `version >= kSaveVersionElementPalette`.
  std::vector<int32_t> element_palette;

  size_t Count() const { return static_cast<size_t>(width) * height; }
  int32_t GameWidth() const { return width - 2; }
  int32_t GameHeight() const { return height - 2; }

  // Padded index of a cell in the game's own coordinates.
  size_t Index(int32_t game_x, int32_t game_y) const {
    return static_cast<size_t>(game_y + 1) * width + (game_x + 1);
  }
};

// The FIXED prefix: every section whose length is a function of the version and the grid.
//
// Through kSaveVersionRoomPromotion this was the whole blob, and a blob whose size did not
// equal it exactly was rejected. kSaveVersionExtensions adds a VARIABLE tail, so from v18 on
// this is a lower bound rather than an equality -- see DecodeSaveBlob, which keeps the exact
// check for every legacy version and requires the tail to consume the remainder exactly.
inline size_t SaveBlobFixedSize(int32_t version, int32_t width, int32_t height) {
  const size_t n = static_cast<size_t>(width) * height;
  size_t sz =
      kSaveHeaderSize + n * (sizeof(SaveCell) + sizeof(SaveDisease) + sizeof(SaveBackwall));
  // The v16 and v17 fixed sections exist ONLY at v16 and v17. From kSaveVersionExtensions on,
  // the same data travels as registered `kSaved` properties inside the self-describing
  // section, so a v18 blob does not carry them at a fixed offset at all.
  //
  // THE `<` HALF OF THIS TEST IS LOAD-BEARING, and getting it wrong is not a size bug, it is a
  // crash: every gate in this file is `>=`, so a version number is a cumulative claim. Left as
  // a bare `>=`, `EncodeSaveBlob` would copy `n * sizeof(SaveGasCell)` bytes out of an empty
  // `b.gas` for the ordinary v18 world that has extension data and no gas mixture.
  if (version >= kSaveVersionGasMixture && version < kSaveVersionExtensions) {
    sz += n * sizeof(SaveGasCell);
  }
  if (version >= kSaveVersionRoomPromotion && version < kSaveVersionExtensions) {
    sz += n * sizeof(uint8_t);
  }
  return sz;
}

// Bytes one extension record occupies, header included.
inline size_t SaveExtRecordSize(const SaveExtProperty& p) {
  return kSaveExtRecordHeader + p.bytes.size();
}

// Bytes the whole extension section occupies, count word included. Zero when there are no
// properties -- a blob below kSaveVersionExtensions, or one that had nothing to say.
inline size_t SaveExtSectionSize(const std::vector<SaveExtProperty>& ext) {
  if (ext.empty()) return 0;
  size_t sz = 4;
  for (const SaveExtProperty& p : ext) sz += SaveExtRecordSize(p);
  return sz;
}

// Total size of a blob, fixed prefix plus whatever tail it carries.
inline size_t SaveBlobSize(const SaveBlob& b);

// ---------------------------------------------------------------------- the extension section
//
// Factored out of EncodeSaveBlob/DecodeSaveBlob so the SEVENTH checkpoint
// component (`sim/ext_state.h`, `ext::kSetExtCellState`) writes the same records into a
// different container instead of growing a second, subtly different parser. The section is:
//
//     int32 count, then `count` records of { name[48], type, arity, stride, cellCount,
//     defaultBits, bytes[cellCount * arity * stride] }
//
// and it appears verbatim in a v18 save blob's tail and in a checkpoint blob's body.

// Writes the section at `p` and returns the pointer just past it. `p` must have
// `SaveExtSectionSize(ext)` writable bytes.
inline uint8_t* EncodeExtSection(const std::vector<SaveExtProperty>& ext, uint8_t* p) {
  const int32_t count = static_cast<int32_t>(ext.size());
  memcpy(p, &count, 4);
  p += 4;
  for (const SaveExtProperty& e : ext) {
    // NUL-PADDED, not just NUL-terminated: the tail of the field is part of the bytes a save
    // is compared against, and leaving it uninitialised would make two blobs with the same
    // content differ. `name` is at most 47 chars (sim_abi_ext.h's rule), so the copy always
    // leaves at least one zero byte.
    memset(p, 0, 48);
    memcpy(p, e.name.data(), e.name.size() < 48 ? e.name.size() : 48);
    memcpy(p + 48, &e.type, 4);
    memcpy(p + 52, &e.arity, 4);
    memcpy(p + 56, &e.stride, 4);
    memcpy(p + 60, &e.cell_count, 4);
    memcpy(p + 64, &e.default_bits, 4);
    p += kSaveExtRecordHeader;
    if (!e.bytes.empty()) memcpy(p, e.bytes.data(), e.bytes.size());
    p += e.bytes.size();
  }
  return p;
}

// Reads the section starting at `p`, which must not run past `end`. Returns the pointer just
// past the section, or nullptr with `error` filled.
//
// Everything here is bounds-checked against the REMAINING bytes before it is used as a length:
// this is the only section whose size comes out of the blob rather than out of the grid, so a
// truncated or hostile blob can claim any count and any stride it likes.
//
// `expect_cells` is the grid the section has to agree with, and `allow_empty` says whether a
// zero count is legal -- it is not in a save blob (see the caller) and is in a checkpoint.
inline const uint8_t* DecodeExtSection(const uint8_t* p, const uint8_t* end,
                                       size_t expect_cells, bool allow_empty,
                                       std::vector<SaveExtProperty>* out, std::string* error) {
  auto fail = [&](const char* why) -> const uint8_t* {
    if (error) *error = why;
    return nullptr;
  };
  if (out == nullptr || p == nullptr || end < p) return fail("extension section: bad arguments");
  if (end - p < 4) return fail("extension section is truncated before its count");
  int32_t count = 0;
  memcpy(&count, p, 4);
  p += 4;
  if (count < 0) return fail("extension section claims a negative property count");
  if (count == 0 && !allow_empty) {
    return fail("extension section claims no properties; such a blob should not be at "
                "kSaveVersionExtensions at all");
  }
  if (count > kSaveExtMaxProperties) {
    return fail("extension section claims more properties than the registry can hold");
  }
  out->clear();
  out->resize(static_cast<size_t>(count));
  for (int32_t i = 0; i < count; ++i) {
    SaveExtProperty& e = (*out)[static_cast<size_t>(i)];
    if (static_cast<size_t>(end - p) < kSaveExtRecordHeader) {
      return fail("extension record header runs past the end of the blob");
    }
    char raw[49] = {};
    memcpy(raw, p, 48);
    raw[48] = '\0';  // a name that fills all 48 bytes is unterminated on disk too
    e.name.assign(raw);
    memcpy(&e.type, p + 48, 4);
    memcpy(&e.arity, p + 52, 4);
    memcpy(&e.stride, p + 56, 4);
    memcpy(&e.cell_count, p + 60, 4);
    memcpy(&e.default_bits, p + 64, 4);
    p += kSaveExtRecordHeader;

    if (e.name.empty()) return fail("extension record has an empty name");
    if (e.arity < 1 || e.arity > 64) return fail("extension record has an implausible arity");
    if (e.stride < 1 || e.stride > 8) return fail("extension record has an implausible stride");
    if (static_cast<size_t>(e.cell_count) != expect_cells) {
      return fail("extension record's cell count does not match the blob's own grid");
    }
    // 64-bit product before the comparison: three int32s that each pass their own bound can
    // still multiply past 2^31.
    const uint64_t bytes = static_cast<uint64_t>(e.cell_count) *
                           static_cast<uint64_t>(e.arity) * static_cast<uint64_t>(e.stride);
    if (bytes > static_cast<uint64_t>(end - p)) {
      return fail("extension record claims more bytes than the blob has left");
    }
    e.bytes.resize(static_cast<size_t>(bytes));
    memcpy(e.bytes.data(), p, static_cast<size_t>(bytes));
    p += static_cast<size_t>(bytes);
  }
  return p;
}

// Decode. Returns false and fills `error` rather than reading past the end of a blob
// that does not describe itself consistently — a truncated save is the one input this
// is guaranteed to meet eventually.
inline bool DecodeSaveBlob(const uint8_t* data, size_t size, SaveBlob* out,
                           std::string* error) {
  auto fail = [&](const char* why) {
    if (error) *error = why;
    return false;
  };
  if (size < kSaveHeaderSize) return fail("blob shorter than the header");
  if (memcmp(data, kSaveMagic, sizeof(kSaveMagic)) != 0) return fail("bad magic");

  SaveBlob b;
  memcpy(&b.version, data + 8, 4);
  if (b.version < kSaveVersionKleiNoBackwall) return fail("blob version older than 13 is not readable");
  if (b.version == kSaveVersionKleiNoBackwall) {
    // See kSaveVersionKleiNoBackwall. Decoded straight into the current shape and stamped
    // kSaveVersion, so nothing downstream needs to know this blob was ever older.
    if (size < kSaveHeaderSizeV13) return fail("blob shorter than the v13 header");
    memcpy(&b.width, data + 12, 4);
    memcpy(&b.height, data + 16, 4);
    b.trailing = data[20];
    if (b.width <= 2 || b.height <= 2) return fail("degenerate grid dimensions");
    if (static_cast<int64_t>(b.width) * b.height > (1 << 28)) {
      return fail("grid dimensions implausibly large");
    }
    const size_t n = b.Count();
    if (size != kSaveHeaderSizeV13 +
                    n * (kSaveCellSizeV13 + sizeof(SaveDisease) + kSaveUnreadSizeV13)) {
      return fail("size does not match w*h for v13");
    }
    b.cells.resize(n);
    b.disease.resize(n);
    b.backwall.assign(n, SaveBackwall{kSaveBackwallVacuumHash, 0.0f, 0.0f});
    const uint8_t* p = data + kSaveHeaderSizeV13;
    for (size_t i = 0; i < n; ++i, p += kSaveCellSizeV13) {
      memcpy(&b.cells[i], p, kSaveCellSizeV13);
      b.cells[i].radiation = 0.0f;
    }
    memcpy(b.disease.data(), p, n * sizeof(SaveDisease));
    b.version = kSaveVersion;
    b.ring_from_blob = false;
    *out = std::move(b);
    return true;
  }
  memcpy(&b.width, data + 12, 4);
  memcpy(&b.height, data + 16, 4);
  memcpy(&b.x, data + 20, 4);
  memcpy(&b.y, data + 24, 4);
  b.trailing = data[28];

  if (b.width <= 2 || b.height <= 2) return fail("degenerate grid dimensions");
  // 2^31 bytes of cells is already absurd; this only has to reject nonsense that would
  // overflow the size computation below.
  if (static_cast<int64_t>(b.width) * b.height > (1 << 28)) {
    return fail("grid dimensions implausibly large");
  }
  const size_t fixed = SaveBlobFixedSize(b.version, b.width, b.height);
  if (b.version >= kSaveVersionExtensions) {
    // From v18 the blob has a variable tail, so the fixed prefix is a lower bound. The tail
    // is then required to consume the remainder EXACTLY, below -- the strictness moves, it
    // does not relax.
    if (size < fixed) return fail("blob shorter than its own fixed sections");
  } else if (size != fixed) {
    return fail("size does not match w*h for this version");
  }

  const size_t n = b.Count();
  b.cells.resize(n);
  b.disease.resize(n);
  b.backwall.resize(n);
  const uint8_t* p = data + kSaveHeaderSize;
  memcpy(b.cells.data(), p, n * sizeof(SaveCell));
  p += n * sizeof(SaveCell);
  memcpy(b.disease.data(), p, n * sizeof(SaveDisease));
  p += n * sizeof(SaveDisease);
  memcpy(b.backwall.data(), p, n * sizeof(SaveBackwall));
  p += n * sizeof(SaveBackwall);
  // Absent from every blob at kSaveVersion (15) — the game's own format this sim has to keep reading forever — and present only at
  // kSaveVersionGasMixture (16) or above, which only our own EncodeSaveBlob ever writes.
  if (b.version >= kSaveVersionGasMixture && b.version < kSaveVersionExtensions) {
    b.gas.resize(n);
    memcpy(b.gas.data(), p, n * sizeof(SaveGasCell));
    p += n * sizeof(SaveGasCell);
  }
  // Absent below kSaveVersionRoomPromotion (17) — every blob at
  // kSaveVersionGasMixture (16) and below, including every real game save at kSaveVersion (15).
  if (b.version >= kSaveVersionRoomPromotion && b.version < kSaveVersionExtensions) {
    b.room_promoted.resize(n);
    memcpy(b.room_promoted.data(), p, n * sizeof(uint8_t));
    p += n * sizeof(uint8_t);
  }
  // The self-describing extension section.
  if (b.version >= kSaveVersionExtensions) {
    const uint8_t* const end = data + size;
    // `allow_empty=false`: a v18 blob exists precisely because something had to be written
    // into it, so a zero count is either corruption or a writer that should have emitted v17.
    // The checkpoint carrier passes true, because a world with nothing to carry still has to
    // produce a blob a caller can hand back.
    p = DecodeExtSection(p, end, n, /*allow_empty=*/false, &b.ext, error);
    if (p == nullptr) return false;
    if (b.version >= kSaveVersionElementPalette) {
      if (end - p < 4) return fail("element palette is truncated before its count");
      int32_t count = 0;
      memcpy(&count, p, 4);
      p += 4;
      if (count < 1 || count > kSaveMaxPaletteElements) {
        return fail("element palette claims an implausible element count");
      }
      if (static_cast<size_t>(end - p) < static_cast<size_t>(count) * 4) {
        return fail("element palette runs past the end of the blob");
      }
      b.element_palette.resize(static_cast<size_t>(count));
      memcpy(b.element_palette.data(), p, static_cast<size_t>(count) * 4);
      p += static_cast<size_t>(count) * 4;
    }
    // The tail has to account for every remaining byte. A blob with slack after its last
    // record is not a blob this writer produced.
    if (p != end) return fail("extension section does not consume the rest of the blob");
  }

  *out = std::move(b);
  return true;
}

inline size_t SaveBlobPaletteSize(const SaveBlob& b);

inline std::vector<uint8_t> EncodeSaveBlob(const SaveBlob& b) {
  std::vector<uint8_t> out(SaveBlobFixedSize(b.version, b.width, b.height) +
                           SaveExtSectionSize(b.ext) + SaveBlobPaletteSize(b));
  memcpy(out.data(), kSaveMagic, sizeof(kSaveMagic));
  memcpy(out.data() + 8, &b.version, 4);
  memcpy(out.data() + 12, &b.width, 4);
  memcpy(out.data() + 16, &b.height, 4);
  memcpy(out.data() + 20, &b.x, 4);
  memcpy(out.data() + 24, &b.y, 4);
  out[28] = b.trailing;

  const size_t n = b.Count();
  uint8_t* p = out.data() + kSaveHeaderSize;
  memcpy(p, b.cells.data(), n * sizeof(SaveCell));
  p += n * sizeof(SaveCell);
  memcpy(p, b.disease.data(), n * sizeof(SaveDisease));
  p += n * sizeof(SaveDisease);
  memcpy(p, b.backwall.data(), n * sizeof(SaveBackwall));
  p += n * sizeof(SaveBackwall);
  if (b.version >= kSaveVersionGasMixture && b.version < kSaveVersionExtensions) {
    memcpy(p, b.gas.data(), n * sizeof(SaveGasCell));
    p += n * sizeof(SaveGasCell);
  }
  if (b.version >= kSaveVersionRoomPromotion && b.version < kSaveVersionExtensions) {
    memcpy(p, b.room_promoted.data(), n * sizeof(uint8_t));
    p += n * sizeof(uint8_t);
  }
  if (!b.ext.empty()) p = EncodeExtSection(b.ext, p);
  if (b.version >= kSaveVersionElementPalette) {
    const int32_t count = static_cast<int32_t>(b.element_palette.size());
    memcpy(p, &count, 4);
    p += 4;
    if (count > 0) memcpy(p, b.element_palette.data(), static_cast<size_t>(count) * 4);
    p += static_cast<size_t>(count) * 4;
  }
  return out;
}

inline size_t SaveBlobPaletteSize(const SaveBlob& b) {
  return b.version >= kSaveVersionElementPalette ? 4 + b.element_palette.size() * 4 : 0;
}

inline size_t SaveBlobSize(const SaveBlob& b) {
  return SaveBlobFixedSize(b.version, b.width, b.height) + SaveExtSectionSize(b.ext) +
         SaveBlobPaletteSize(b);
}

}  // namespace oni_sim
