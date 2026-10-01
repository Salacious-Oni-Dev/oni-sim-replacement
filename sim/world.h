// The replacement sim's world state.
//
// Storage is padded exactly the way Klei's is — a one-cell border ring, so the save blob
// is a straight serialisation and neighbour loops never need a bounds test. The game,
// however, indexes `GameDataUpdate` arrays by *unpadded* game cell, so every projection
// has to convert. Getting that wrong shifts the whole world by one row and looks like
// corruption rather than an off-by-one.
//
// Cells hold a list of phases rather than one element. Today that list is always length
// one, which is what makes this "vanilla-equivalent"; the volume-fraction model fills it
// in later. The projection is already written against the list, so it does not change.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "../abi/gas_mixture_abi.h"

#include "../abi/sim_abi.h"
#include "census.h"
#include "ext_elements.h"
#include "tunables.h"
#include "ext_registry.h"
#include "ext_state.h"
#include "saveblob.h"

namespace oni_sim {

// Element.state packs the phase into the low two bits and flags above it.
enum : uint8_t {
  kStateMask = 3,
  kStateVacuum = 0,
  kStateGas = 1,
  kStateLiquid = 2,
  kStateSolid = 3,
  kStateUnbreakable = 4,
  kStateUnstable = 8,
};

// Cell.properties bits, from Sim.Cell.Properties.
enum : uint8_t {
  kGasImpermeable = 1,
  kLiquidImpermeable = 2,
  kSolidImpermeable = 4,
  kUnbreakable = 8,
  kTransparent = 0x10,
  kOpaque = 0x20,
  kNotifyOnMelt = 0x40,
  // The bit the radiation absorption walk tests, and the only reader of it in the sim:
  // a constructed tile absorbs `RADIATION_CONSTRUCTED_FACTOR` of its element's factor
  // instead of the mass-weighted mix every other cell gets. Klei tests it as the sign of
  // the signed byte, which is what makes 0x80 and "negative" the same question.
  kConstructedTile = 0x80,
};

// ------------------------------------------------------------- SSE min/max, spelled out
//
// SSE scalar min is `dst = (dst < src) ? dst : src`. On finite operands that is `std::min`
// with the arguments in this order; on a NaN it is the opposite of it. Per the Intel SDM, if
// *either* operand is a NaN the instruction returns `src` outright — so a game clamp written as
// SSE min-then-max doubles as a NaN scrubber and lands on the bound, while the C++ it reads as
// (`if (v > hi) v = hi;`, or `std::min(v, hi)`) compares false against the NaN and keeps it.
// SSE max has the same asymmetry.
//
// That is not a theoretical difference. Klei's sim divides without guarding the denominator
// in at least a dozen places, and every one of those is a 0/0 the moment some mass or heat
// capacity is exactly zero — which the game supplies routinely (neutronium's specific heat is
// 0, a cell of the right element can hold no mass, a building can register with no cells).
// Writing one of those clamps as comparisons gives a NaN where the game gives a bound: the
// element consumer's temperature (`emitters.h`) and the germ fraction in `UpdatePressure`
// (`physics.h`) are two places it shows.
//
// The `ClampSS` argument order is the game's own: min against `hi` first, then max against
// `lo`, which is what makes a NaN come out as `hi` rather than as `lo`.
inline float MinSS(float dst, float src) { return dst < src ? dst : src; }
inline float MaxSS(float dst, float src) { return dst > src ? dst : src; }
inline double MinSD(double dst, double src) { return dst < src ? dst : src; }
inline double MaxSD(double dst, double src) { return dst > src ? dst : src; }
inline float ClampSS(float v, float lo, float hi) { return MaxSS(MinSS(v, hi), lo); }

// One substance in a cell. A single-element cell has exactly one of these; a
// volume-fraction cell will have several and they must stay sorted by element index so
// the dominant-phase tie-break is deterministic.
struct PhaseEntry {
  uint16_t element = 0;
  float mass = 0.0f;
  float temperature = 0.0f;
};

// Per disease, per element. Eight fields, and the order is the order the game writes them
// in — the order `Disease::Disease` fills its eight per-element vectors, reading
// one entry of each per element per record. Six floats,
// one int and one byte is the 29-byte stride the old size-only parser had already measured.
//
// Which field is which came from the two kernels rather than from a name: `GetDiffusionScale`
// returns vector 3 and gates on vectors 6 and 7, and `PostProcess`
// picks vector 1 in range, vector 2 over, and vector 0 under. That is exactly
// Klei's own `ElemGrowthInfo` declaration order.
struct ElemGrowthInfo {
  float underPopulationDeathRate = 0.0f;
  float populationHalfLife = 0.0f;
  float overPopulationHalfLife = 0.0f;
  float diffusionScale = 0.0f;
  float minCountPerKG = 0.0f;
  float maxCountPerKG = 0.0f;
  int32_t minDiffusionCount = 0;
  uint8_t minDiffusionInfestationTickCount = 0;
};

// One disease. 0x150 bytes in the DLL; the four `RangeInfo`s are `{minViable, minGrowth,
// maxGrowth, maxViable}` and are read in ascending order by the temperature sweep.
//
// The two pressure ranges are parsed because the game sends them and the stride depends on
// them, but **nothing in the sim reads them**. `PostProcess` looks at temperature and at the
// per-element population bounds and at nothing else; pressure lives entirely game-side.
struct DiseaseInfo {
  int32_t id_hash = 0;
  float strength = 0.0f;
  float temperature_range[4] = {0, 0, 0, 0};
  float temperature_half_lives[4] = {0, 0, 0, 0};
  float pressure_range[4] = {0, 0, 0, 0};
  float pressure_half_lives[4] = {0, 0, 0, 0};
  float radiation_kill_rate = 0.0f;
  std::vector<ElemGrowthInfo> growth;
};

// The id hashes turn a cell's disease index into the hash the save format stores and back;
// the rest is the growth and half-life data the two disease kernels run on.
class DiseaseTable {
 public:
  bool Load(const uint8_t* data, size_t size) {
    // 0 int32 diseaseCount | 4 int32 elementCount | per disease: KleiString name,
    // int32 idHashCode, ... The record is variable length because of the name and the
    // per-element growth table, so walk it rather than indexing.
    hashes_.clear();
    infos_.clear();
    if (size < 8) return false;
    int32_t disease_count = 0, element_count = 0;
    memcpy(&disease_count, data, 4);
    memcpy(&element_count, data + 4, 4);
    if (disease_count < 0 || element_count < 0) return false;
    size_t off = 8;
    for (int32_t i = 0; i < disease_count; ++i) {
      int32_t name_len = 0;
      if (off + 4 > size) return false;
      memcpy(&name_len, data + off, 4);
      off += 4;
      if (name_len < 0 || off + static_cast<size_t>(name_len) + 4 > size) return false;
      off += static_cast<size_t>(name_len);
      int32_t hash = 0;
      memcpy(&hash, data + off, 4);
      hashes_.push_back(hash);
      // Everything after the name, in the order `Disease::Disease` reads it. The record is
      // walked twice — once to keep the hash list this class has always published, once to
      // fill the info — and the two agree by construction because both walk `off`.
      if (off + kDiseaseFixedSize +
              static_cast<size_t>(element_count) * kElemGrowthInfoSize >
          size) {
        return false;
      }
      DiseaseInfo info;
      size_t p = off;
      auto f32 = [&]() {
        float v = 0.0f;
        memcpy(&v, data + p, 4);
        p += 4;
        return v;
      };
      auto i32 = [&]() {
        int32_t v = 0;
        memcpy(&v, data + p, 4);
        p += 4;
        return v;
      };
      info.id_hash = i32();
      info.strength = f32();
      for (int k = 0; k < 4; ++k) info.temperature_range[k] = f32();
      for (int k = 0; k < 4; ++k) info.temperature_half_lives[k] = f32();
      for (int k = 0; k < 4; ++k) info.pressure_range[k] = f32();
      for (int k = 0; k < 4; ++k) info.pressure_half_lives[k] = f32();
      info.radiation_kill_rate = f32();
      info.growth.resize(static_cast<size_t>(element_count));
      for (int32_t e = 0; e < element_count; ++e) {
        ElemGrowthInfo& g = info.growth[static_cast<size_t>(e)];
        g.underPopulationDeathRate = f32();
        g.populationHalfLife = f32();
        g.overPopulationHalfLife = f32();
        g.diffusionScale = f32();
        g.minCountPerKG = f32();
        g.maxCountPerKG = f32();
        g.minDiffusionCount = i32();
        g.minDiffusionInfestationTickCount = data[p];
        p += 1;
      }
      infos_.push_back(std::move(info));
      // The fixed part that follows the name, then the per-element growth table.
      //
      // Both sizes were guessed the first time round and both were wrong, which made this
      // function return false on the *real* game's table — the DLL reported "disease table
      // rejected" and every disease hash silently became 0. It went unnoticed because
      // disease is stubbed and no scenario has an infected cell in it.
      //
      // Recovered by walking the real 31,220-byte payload: the five records are Food
      // Poisoning (14), Slimelung (9), Floral Scent (12), Zombie Spores (13) and
      // Radioactive Contaminants (24), and with a 76-byte fixed part and a **29-byte**
      // growth record they land on 6250 / 12487 / 18727 / 24968 and end on exactly 31220.
      // Every one of those matches an observed name offset. 29 is not a mistake for 32:
      // this is one of the byte-packed serialisations, 7 floats plus a trailing byte, and
      // a stride scan over the payload picks 29 out by a factor of seven over any other
      // stride.
      off += kDiseaseFixedSize;
      off += static_cast<size_t>(element_count) * kElemGrowthInfoSize;
      if (off > size) return false;
    }
    return true;
  }

  int32_t Count() const { return static_cast<int32_t>(hashes_.size()); }
  int32_t HashOf(uint8_t idx) const {
    return idx < hashes_.size() ? hashes_[idx] : 0;
  }
  uint8_t IndexOfHash(int32_t hash) const {
    for (size_t i = 0; i < hashes_.size(); ++i) {
      if (hashes_[i] == hash) return static_cast<uint8_t>(i);
    }
    return 0xFF;
  }

  // Null for an index the table does not hold, which is what a world loaded before the
  // table arrives looks like. Both kernels check.
  const DiseaseInfo* Get(uint8_t idx) const {
    return idx < infos_.size() ? &infos_[idx] : nullptr;
  }

 private:
  // Measured, not derived — see Load. The fixed part covers idHashCode and everything up
  // to the growth table; the individual fields in it are not needed here, only its size.
  static constexpr size_t kDiseaseFixedSize = 76;
  static constexpr size_t kElemGrowthInfoSize = 29;
  std::vector<int32_t> hashes_;
  std::vector<DiseaseInfo> infos_;
};

class ElementTable {
 public:
  // Registers the first-party element attributes and seeds their defaults ONCE, at
  // construction rather than at every `Load` -- see `SeedDefaultMolecularMasses` for why the
  // difference is a bug fix and not a refactor.
  ElementTable() {
    std::string error;
    molecular_mass_attr_ = attributes_.Register(ext::kAttrMolecularMass, ext::kExtF32,
                                                /*arity=*/1, /*first_party=*/true, &error);
    // Registered here for the same reason as the molecular mass: a native kernel
    // (the conduit run phase change in `sim/conduits.h`) reads them, so the index has to be
    // fixed before any mod can register anything. Unlike the molecular mass they are NOT
    // seeded -- the curve is the managed registry's data, and a sim nobody has told simply has
    // no phase change in conduit runs, which is the vanilla behaviour.
    phase_curve_attr_ = attributes_.Register(ext::kAttrPhaseCurve, ext::kExtF32,
                                             ext::kPhaseCurveArity, /*first_party=*/true, &error);
    liquid_density_attr_ = attributes_.Register(ext::kAttrLiquidDensity, ext::kExtF32,
                                                /*arity=*/1, /*first_party=*/true, &error);
    // Registered here for the same reason the curve is: `TransitionCell`
    // (physics.h) reads them on the state-change path, so the indices have to be fixed before
    // any mod can register anything. Not seeded either -- an element the managed side has
    // never described has no condensation rule and keeps Klei's fixed threshold, which is the
    // vanilla behaviour and the reason the offline gate is safe by construction.
    min_liquid_pressure_attr_ = attributes_.Register(ext::kAttrMinLiquidPressure, ext::kExtF32,
                                                     /*arity=*/1, /*first_party=*/true, &error);
    can_condense_attr_ = attributes_.Register(ext::kAttrCanCondense, ext::kExtF32,
                                              /*arity=*/1, /*first_party=*/true, &error);
    // Physical latent heats (D3). Read by `LatentEnergyOfTransition` (physics.h) inside the
    // planetary accumulator only, so it is registered here for the same fixed-index reason and
    // left unseeded for the same vanilla-by-default reason as the four above.
    latent_fusion_attr_ = attributes_.Register(ext::kAttrLatentFusion, ext::kExtF32,
                                               /*arity=*/1, /*first_party=*/true, &error);
    SeedDefaultMolecularMasses();
  }

  // The table arrives as int32 count followed by that many Element records.
  bool Load(const uint8_t* data, size_t size) {
    if (size < 4) return false;
    int32_t n = 0;
    memcpy(&n, data, 4);
    if (n <= 0 || 4 + static_cast<size_t>(n) * sizeof(Element) > size) return false;
    elements_.resize(n);
    memcpy(elements_.data(), data + 4, static_cast<size_t>(n) * sizeof(Element));
    by_hash_.clear();
    ++generation_;
    return true;
  }

  int32_t Count() const { return static_cast<int32_t>(elements_.size()); }
  // Bumped by every `Load`. Anything that caches a value derived from the table keys its
  // cache on this as well as on the table's address, because a table can be reloaded in
  // place and the address alone would not notice.
  uint32_t Generation() const { return generation_; }
  bool Empty() const { return elements_.empty(); }
  const Element& At(uint16_t i) const { return elements_[i < Count() ? i : 0]; }

  // A backwall index past the end of the table is "no element", not element 0: worldgen sends
  // 0xFFFF for every template cell that leaves `backwallElement` unset. Klei keeps the raw
  // index, publishes 0xFFFF and saves it as Vacuum. Stored here as hash 0, which no element has.
  static constexpr int32_t kNoElementHash = 0;
  int32_t BackwallHash(uint16_t i) const {
    return i < Count() ? elements_[i].id : kNoElementHash;
  }

  // Saves key elements by SimHashes value, not by index, so this lookup is on the
  // critical path of every load — and it is not only a load-time lookup: `Project` asks
  // for the backwall element of every cell in the world, every frame. The index used to
  // be an unsorted vector walked linearly, which is 212 compares for a hash the table does
  // not hold, and a cell with no backwall carries hash 0, which the table never holds.
  // Sorted plus `lower_bound` instead. Duplicate hashes keep resolving to the *lowest*
  // index, the way a linear scan from the front did, because the sort is by (hash, index).
  uint16_t IndexOfHash(int32_t hash) const {
    const std::pair<int32_t, uint16_t>* e = Find(hash);
    return e ? e->second : 0;  // vacuum-ish fallback; callers check HasHash first
  }
  bool HasHash(int32_t hash) const { return Find(hash) != nullptr; }

  // Klei reports a cell that flow has emptied as Vacuum, not as a massless cell of
  // whatever used to be there — a shaft a drop of water has just left reads back as
  // element 211 with zero mass, not as water.
  uint16_t VacuumIndex() const {
    constexpr int32_t kVacuumHash = 758759285;
    return HasHash(kVacuumHash) ? IndexOfHash(kVacuumHash) : static_cast<uint16_t>(0);
  }

  // The Void index. Void is the element whose cells destroy whatever is pushed into
  // them: `UpdatePressure` compares the destination's element against this index
  // and, when it matches, zeroes the cell's mass and temperature instead of
  // filling it. No `diffsim` scenario carries one, so when the table has no Void this
  // returns an index no element can hold and the comparison never fires.
  uint16_t VoidIndex() const {
    constexpr int32_t kVoidHash = -1456075980;
    return HasHash(kVoidHash) ? IndexOfHash(kVoidHash) : static_cast<uint16_t>(0xFFFF);
  }

  // The world border. `SimData::ResizeAndInitializeVacuumCells` rings the rectangle it
  // opens with this at a flat 9999 kg and 0 K — measured against Klei on `vacrect`, and
  // 9999 is a constant in the function rather than anything in Unobtanium's own row (its
  // table entry carries 10000 and 20000, and neither is what lands in the cell).
  uint16_t UnobtaniumIndex() const {
    constexpr int32_t kUnobtaniumHash = 1838482828;
    return HasHash(kUnobtaniumHash) ? IndexOfHash(kUnobtaniumHash)
                                    : static_cast<uint16_t>(0xFFFF);
  }

  uint8_t Phase(uint16_t i) const { return At(i).state & kStateMask; }
  bool IsSolid(uint16_t i) const { return Phase(i) == kStateSolid; }
  bool IsVacuum(uint16_t i) const { return Phase(i) == kStateVacuum; }
  float SpecificHeat(uint16_t i) const { return At(i).specificHeatCapacity; }

  // ---------------------------------------------------------------- molecular mass
  //
  // `Element::molarMass` is Klei's own content-data field and this project mirrors it
  // verbatim, because every vanilla kernel that reads it has to keep reading exactly what
  // vanilla read. But Klei stores the **atomic** mass for the diatomic gases, not the
  // molecular mass of the molecule the element actually represents (measured
  // against the shipped `StreamingAssets/elements/gas.yaml`, not assumed):
  //
  //     Oxygen              15.9994    should be 31.9988   (O2)
  //     ContaminatedOxygen  15.9994    should be 31.9988   (O2 by mass, same molecule)
  //     Hydrogen             1.00794   should be  2.01588  (H2)
  //     ChlorineGas         34.453     should be 70.906    (Cl2; Klei's own figure is also
  //                                                         a typo of atomic Cl, 35.453)
  //
  // Everything else in that file is already right for the molecule it names -- CarbonDioxide
  // 44.01, Methane 16.044, Steam 18.01528, Propane 44.1, Helium 4, and the metal vapours,
  // which really are monatomic. Deliberately NOT corrected: SulfurGas (32) and PhosphorusGas
  // (30.973762). Real sulfur and phosphorus vapour are S8/S2 and P4 depending on temperature,
  // so there is no single right answer to substitute, and inventing one would be a guess.
  //
  // The consequence, found live: every gas pressure this project reported for a diatomic was
  // exactly 2x high, because `moles = mass / molarMass` used the atomic figure. 6 kg of O2 in
  // 120 L at 293.2 K read 7.62 MPa where the real answer is 3.81 MPa.
  //
  // This accessor is the corrected one, and ONLY the mole/pressure functions in
  // sim/gas_mixture.h call it (CellMoles, MolesFromSpeciesList). Every vanilla kernel keeps
  // reading `At(i).molarMass` directly and is bit-for-bit unaffected, which is what keeps the
  // 823-scenario byte-identity gate meaningful. gas_mixture.h's own MIXING math
  // (MixPair, EqualizeSingleSpecies) also keeps the raw field on purpose -- see the comment
  // there; the molar mass cancels out of the transferred mass analytically, so switching it
  // would perturb float rounding for exactly zero physical gain.
  //
  // SINCE STAGE 5 the storage is the per-element attribute registry (`sim/ext_elements.h`,
  // `sim.molecular_mass`) rather than a private vector, and this is the typed accessor the
  // design's own rule keeps: a native kernel must not do a string lookup per cell per substep,
  // so the index is resolved once at construction and this indexes it. A miss means the
  // element has no override at all, which is a different answer from an override of zero, and
  // is exactly why the registry reports set-or-not instead of substituting a default -- the
  // fallback below is per-element and only this function knows it.
  float MolecularMassOf(uint16_t i) const {
    const Element& e = At(i);
    float v = 0.0f;
    if (attributes_.ReadF32(molecular_mass_attr_, e.id, &v)) return v;
    return e.molarMass;
  }

  // The registry itself, for the generic `kRegisterElementAttribute` / `kSetElementAttribute`
  // path and the two exports that read it back. Third-party attributes live here beside the
  // first-party one and are stored, handed back, and read by no kernel at all.
  const ext::ElementAttributeRegistry& Attributes() const { return attributes_; }
  ext::ElementAttributeRegistry& MutableAttributes() { return attributes_; }
  int32_t MolecularMassAttribute() const { return molecular_mass_attr_; }

  // The vapour curve of the element at table index `i`, as `sim.phase_curve` holds
  // it; false when the element has no complete entry. All five components are required, so a
  // half-written curve reads as absent rather than as a curve with zeros in it.
  struct PhaseCurve {
    float a = 0.0f;
    float b = 0.0f;
    float freezing_k = 0.0f;
    float critical_k = 0.0f;
    float latent_j_per_kg = 0.0f;
  };
  bool PhaseCurveOf(uint16_t i, PhaseCurve* out) const {
    if (i >= elements_.size()) return false;
    const int32_t id = elements_[i].id;
    float v[ext::kPhaseCurveArity];
    for (int32_t k = 0; k < ext::kPhaseCurveArity; ++k) {
      uint32_t bits = 0;
      if (!attributes_.Read(phase_curve_attr_, id, k, &bits)) return false;
      memcpy(&v[k], &bits, sizeof(float));
    }
    if (out) {
      out->a = v[0];
      out->b = v[1];
      out->freezing_k = v[2];
      out->critical_k = v[3];
      out->latent_j_per_kg = v[4];
    }
    return true;
  }
  // `sim.liquid_density` for the element at index `i`, kg/m^3; 0 when it has none. Callers
  // choose their own fallback, because the managed code this mirrors uses two different ones.
  float LiquidDensityOf(uint16_t i) const {
    if (i >= elements_.size()) return 0.0f;
    float v = 0.0f;
    return attributes_.ReadF32(liquid_density_attr_, elements_[i].id, &v) && v > 0.0f ? v : 0.0f;
  }
  // `sim.min_liquid_pressure` for the element at index `i`, Pa; 0 when it has none. Zero and
  // absent are deliberately the same answer here: a floor of zero admits every pressure, which
  // is exactly what an element with no floor should do.
  //
  // Helium is the case that names the rule. Stationeers lists 0.0 for it but floors every entry
  // at 6.3 kPa, so its minimum liquid pressure is 6.3 kPa, not zero. Either way the floor is not what stops it: 6.3 kPa
  // is a pressure every breathable atmosphere clears. `CanCondenseOf` stops it, or nothing does.
  float MinLiquidPressureOf(uint16_t i) const {
    if (i >= elements_.size()) return 0.0f;
    float v = 0.0f;
    return attributes_.ReadF32(min_liquid_pressure_attr_, elements_[i].id, &v) && v > 0.0f ? v
                                                                                          : 0.0f;
  }
  // `sim.can_condense`, as a bool. UNSET IS NOT FALSE-BUT-PRESENT: `has` reports whether the
  // managed side ever described this element, and the condensation rule requires a present
  // entry before it takes over from Klei's threshold. An element explicitly written 0 (Helium)
  // is described AND refuses; an element never written is undescribed and keeps vanilla.
  bool CanCondenseOf(uint16_t i, bool* has = nullptr) const {
    if (has) *has = false;
    if (i >= elements_.size()) return false;
    float v = 0.0f;
    if (!attributes_.ReadF32(can_condense_attr_, elements_[i].id, &v)) return false;
    if (has) *has = true;
    return v != 0.0f;
  }
  // `sim.latent_fusion` for the element at index `i`, J/kg, into `out`; false when it has
  // none. Absent is reported rather than read as zero, because the one caller has to tell "no
  // fusion enthalpy was ever described" from a value, and refuses the transition on the former.
  bool LatentFusionOf(uint16_t i, float* out) const {
    if (i >= elements_.size()) return false;
    float v = 0.0f;
    if (!attributes_.ReadF32(latent_fusion_attr_, elements_[i].id, &v)) return false;
    if (out) *out = v;
    return true;
  }
  int32_t PhaseCurveAttribute() const { return phase_curve_attr_; }
  int32_t LiquidDensityAttribute() const { return liquid_density_attr_; }
  int32_t MinLiquidPressureAttribute() const { return min_liquid_pressure_attr_; }
  int32_t CanCondenseAttribute() const { return can_condense_attr_; }
  int32_t LatentFusionAttribute() const { return latent_fusion_attr_; }

  // O(1), and the reason `StepStateChange` is not measurably slower than it was.
  //
  // The condensation rule (`CondensationAllowed`, physics.h) sits inside a branch that has
  // already fired, so it looked free -- and measured +0.010 ms on a 0.147 ms kernel anyway,
  // consistently, across every bench configuration. The cost is per CELL THAT CROSSES its
  // low-temperature threshold, of which a settled world has more than the branch's rarity
  // suggests, and each one was paying a binary search through an attribute nobody had written.
  //
  // `keys` is the sorted list of elements that have a value, so an empty one means no mod has
  // described a single element and the whole rule is inert. That is the state of every offline
  // scenario and of any game running without OniFramework, and it costs a load and a compare.
  bool AnyCondensationRules() const {
    const ext::ElementAttributeRegistry::Attribute* a = attributes_.At(can_condense_attr_);
    return a != nullptr && !a->keys.empty();
  }

  // The same O(1) question for `sim.phase_curve`, and it gates the planetary latent
  // accumulator (`StepStateChange`) the way the one above gates the condensation rule.
  //
  // A SEPARATE TEST AND NOT A REUSE OF IT. The two attributes are pushed together by Mod 2
  // today, but they are different claims -- an element can carry a vapour curve and no
  // condensation rule -- and gating one on the other would make the accumulator silently
  // depend on which OTHER attribute a mod happened to write.
  bool AnyPhaseCurves() const {
    const ext::ElementAttributeRegistry::Attribute* a = attributes_.At(phase_curve_attr_);
    return a != nullptr && !a->keys.empty();
  }
  // And for `sim.latent_fusion`, which widens the accumulator's gate: a table carrying fusion
  // enthalpies and no curve at all still has freezes worth counting.
  bool AnyLatentFusion() const {
    const ext::ElementAttributeRegistry::Attribute* a = attributes_.At(latent_fusion_attr_);
    return a != nullptr && !a->keys.empty();
  }

  // Changes whenever anything a conduit run's cached sums were computed from changes on this
  // side: a table reload, or a write or clear of `sim.molecular_mass` (moles) or
  // `sim.liquid_density` (liquid volume). Each term is a monotonic counter, so their sum moves
  // on every change and never returns to an old value. Found by vftest: a density cleared after
  // a run was summed left the cached liquid volume stale.
  uint64_t ConduitAggregateInputsStamp() const {
    uint64_t stamp = generation_;
    if (const ext::ElementAttributeRegistry::Attribute* a = attributes_.At(molecular_mass_attr_)) {
      stamp += a->writes;
    }
    if (const ext::ElementAttributeRegistry::Attribute* a = attributes_.At(liquid_density_attr_)) {
      stamp += a->writes;
    }
    return stamp;
  }

  // The API half: what the SimDLL changes is reachable through the API. The framework pushes
  // MaterialPropertyRegistry's MolecularMassGPerMol values through
  // ext::kSetMolecularMass at load, so a third-party mod that registers a new gas there gets
  // a correct native pressure for it without touching this file. `gPerMol <= 0` removes an
  // override and falls the element back to Klei's own figure. Replaces, never accumulates,
  // same contract as every other extension message.
  void SetMolecularMass(int32_t idHash, float gPerMol) {
    if (gPerMol > 0.0f) {
      attributes_.WriteF32(molecular_mass_attr_, idHash, gPerMol);
    } else {
      attributes_.Clear(molecular_mass_attr_, idHash);
    }
  }

  // Seeded at construction, so the native sim is correct standalone (the offline driver and the
  // test binaries never send an extension message) and the managed push is a refinement
  // rather than a requirement. Hashes are SimHashes values, the same numbering `Element::id`
  // carries. A synthetic test table whose ids are 0/1/2 matches none of these and is
  // therefore left exactly as it was, which is why the offline suite stays byte-identical.
  //
  // The hashes are Klei's own `Hash.SDBMLower(name)` of the element id string. Computed, then
  // the computation itself verified against the three SimHashes constants already hardcoded in
  // this class -- Vacuum 758759285, Unobtanium 1838482828, Void -1456075980 -- which it
  // reproduces exactly, so these are derived from a checked oracle rather than recalled.
  //
  // CALLED ONCE, FROM THE CONSTRUCTOR, NOT FROM `Load`. It used to be called by `Load` and to
  // open with `molecular_overrides_.clear()`, which meant a second element-table load in one
  // session silently threw away every mass a mod had pushed -- while `MaterialProperties.cs`'s
  // own comment claimed the opposite in so many words ("keyed on the SimHashes id ... so the
  // override survives the sim's element table being reloaded"). The keying was always right;
  // the clear was the bug. Nothing here depends on the table's contents -- these are SimHashes
  // ids, which mean the same thing against any table -- so seeding once and never clearing
  // makes the documented claim true. The offline suite is unaffected either way: a synthetic
  // test table's ids are 0/1/2 and match none of these, and a real table reloaded twice ends
  // up with the same four defaults it had before.
  void SeedDefaultMolecularMasses() {
    SetMolecularMass(-1528777920, 31.9988f);   // Oxygen              (O2)
    SetMolecularMass(721531317, 31.9988f);     // ContaminatedOxygen  (O2)
    SetMolecularMass(-1046145888, 2.01588f);   // Hydrogen            (H2)
    SetMolecularMass(-1324664829, 70.906f);    // ChlorineGas         (Cl2)
  }

 private:
  const std::pair<int32_t, uint16_t>* Find(int32_t hash) const {
    if (by_hash_.empty()) BuildIndex();
    const auto it = std::lower_bound(
        by_hash_.begin(), by_hash_.end(), hash,
        [](const std::pair<int32_t, uint16_t>& p, int32_t h) { return p.first < h; });
    if (it == by_hash_.end() || it->first != hash) return nullptr;
    return &*it;
  }
  void BuildIndex() const {
    by_hash_.reserve(elements_.size());
    for (size_t i = 0; i < elements_.size(); ++i) {
      by_hash_.emplace_back(elements_[i].id, static_cast<uint16_t>(i));
    }
    std::sort(by_hash_.begin(), by_hash_.end());
  }
  std::vector<Element> elements_;
  mutable std::vector<std::pair<int32_t, uint16_t>> by_hash_;
  // The per-element attribute registry, holding `sim.molecular_mass` plus
  // whatever a mod registers. Replaces the bespoke `(SimHashes id, g/mol)` vector this class
  // used to walk linearly -- the registry keeps its keys sorted, which matters once a
  // third-party mod has pushed masses for gases of its own and the walk is no longer over four
  // entries.
  ext::ElementAttributeRegistry attributes_;
  // Resolved once, in the constructor. `MolecularMassOf` indexes with it per cell; no string
  // lookup ever reaches the mole/pressure path.
  int32_t molecular_mass_attr_ = -1;
  int32_t phase_curve_attr_ = -1;          // conduit run phase change
  int32_t liquid_density_attr_ = -1;       // conduit run phase change
  int32_t min_liquid_pressure_attr_ = -1;  // cell condensation rule
  int32_t can_condense_attr_ = -1;         // cell condensation rule
  int32_t latent_fusion_attr_ = -1;        // physical latent heats
  uint32_t generation_ = 0;
};

// The clamped evaporation temperature, as the framework's
// MaterialPropertyRegistry.TryGetEvaporationTemperatureClampedK computes it (Stationeers' form): the raw curve
// `(P_kPa / A)^(1/B)` clamped to [freezing, critical], and the freezing point itself for a
// pressure the curve cannot evaluate. Double inside, as the managed original is.
//
// Lives here because it has two callers on opposite sides of the include graph: the conduit
// run phase change (conduits.h) and the cell condensation rule (`TransitionCell`, physics.h).
// `conduits.h` includes `buildings.h` includes `physics.h`, so the definition sits in the
// header both see -- beside the `PhaseCurve` it takes.
//
// THE CLAMP IS SAFE TO KEEP. Stationeers uses the clamped form at every state change, and the
// reason it is safe is an identity in its tables: each element's freezing constant is DEFINED as this curve
// evaluated at that element's own `MinLiquidPressure` --
//
//     FREEZING_TEMPERATURE_NITROGEN_K           = EvaporationTemperature(N2,  6.3 kPa)
//     TRIPLE_POINT_TEMPERATURE_CARBON_DIOXIDE_K = EvaporationTemperature(CO2, 517 kPa)
//
// -- and the curve rises monotonically in P, so `raw(P) >= freezing_k` exactly when
// `P >= MinLiquidPressure`, which is the condition the caller has already tested. The lower
// clamp is therefore unreachable behind the pressure gate rather than merely unlikely: it
// moves the answer only over 0.6 .. 516.9 kPa for CO2, every one of which the floor refuses
// first. (Water is the one element whose freezing point is hand-pinned at 273.15 K instead of
// derived, and there the lower clamp does fire, over a 0.57 kPa band -- where raising a
// sub-freezing saturation temperature to the freezing point is the behaviour you want.)
//
// The UPPER clamp, meanwhile, is load-bearing in the other direction. At 8 MPa the raw curve
// puts CO2's saturation temperature at 272.7 K, above its 266.3 K critical temperature, so a
// cell at 270 K would condense into a liquid that cannot exist. Clamped, it correctly refuses.
inline float EvaporationTemperatureClampedK(const ElementTable::PhaseCurve& c, float pressure_pa) {
  if (!(pressure_pa > 0.0f) || !(c.a > 0.0f) || c.b == 0.0f) return c.freezing_k;
  const double raw = std::pow(static_cast<double>(pressure_pa) / 1000.0 / static_cast<double>(c.a),
                              1.0 / static_cast<double>(c.b));
  const double clamped = std::min(std::max(raw, static_cast<double>(c.freezing_k)),
                                  static_cast<double>(c.critical_k));
  return static_cast<float>(clamped);
}

// One game cell, carrying both of the indices every whole-grid loop needs.
//
// `World::Padded` is a 64-bit division by a width that is only known at run time, and the
// two loops that walk the whole grid — `Project` and `FillPropertyTextures` — were calling
// it once per cell. On a 512x768 grid that was 0.44 ms in each of them, a third of the
// projection and most of the texture pass, spent recomputing an index that advances by one
// per cell and by two more at the end of every row (`padded - game` is `2y + gw + 3`).
// Walking it costs an add and a loop-carried compare instead.
struct CellWalk {
  size_t game;
  size_t padded;
};

class GameCellRange {
 public:
  GameCellRange(int32_t game_width, size_t count, int32_t padded_width)
      : game_width_(game_width), count_(count), padded_width_(padded_width) {}

  class Iterator {
   public:
    Iterator(size_t game, size_t padded, int32_t game_width)
        : game_(game), padded_(padded), x_(0), game_width_(game_width) {}
    CellWalk operator*() const { return CellWalk{game_, padded_}; }
    Iterator& operator++() {
      ++game_;
      ++padded_;
      if (++x_ == game_width_) {
        x_ = 0;
        padded_ += 2;  // skip this row's right border and the next row's left
      }
      return *this;
    }
    bool operator!=(const Iterator& other) const { return game_ != other.game_; }

   private:
    size_t game_;
    size_t padded_;
    int32_t x_;
    int32_t game_width_;
  };

  Iterator begin() const {
    return Iterator(0, static_cast<size_t>(padded_width_) + 1, game_width_);
  }
  Iterator end() const { return Iterator(count_, 0, game_width_); }

 private:
  int32_t game_width_;
  size_t count_;
  int32_t padded_width_;
};

// The two `CellSOA::CopyFrom` snapshots a substep takes. Both live here rather than beside
// the kernels that fill them, because the flow texture reads one of them after the substep
// is over.
//
// The snapshot the *two* gas sweeps share. `SimBase::UpdateData` takes one
// and then runs both the pressure sweep and the displacement sweep off it — there is no
// second copy between them — so `StepGasDisplacement` reads a `start` that predates every
// transfer `StepGasPressure` made, and the two must be called in that order with nothing in
// between.
//
// It is *not* what the flow texture gates on — that is `LiquidSweepStart` below, and
// swapping the two was tried and put nineteen scenarios wrong.
inline std::vector<PhaseEntry>& GasSweepStart() {
  static thread_local std::vector<PhaseEntry> start;
  return start;
}

// The `CellSOA::CopyFrom` — the one a substep takes between the two gas
// sweeps and `UpdateLiquid`, and the last one it takes at all. `StepFlow` prices its
// transfers against it, and `UpdateFlowTexture` reads it a second time after the substep is
// over: a cell whose element has changed since publishes no flow at all.
inline std::vector<PhaseEntry>& LiquidSweepStart() {
  static thread_local std::vector<PhaseEntry> start;
  return start;
}

// What a cell's disease looked like at the same instant. `Disease::UpdateCells`
// reads both cells of a pair out of the snapshot and accumulates into the live grid
// through `AddDiseaseToCell`, which is the gas-pressure
// pattern: the amount a pair moves is independent of the order the sweep visits pairs in,
// but a cell can receive from all four of its neighbours in one sweep.
//
// It is filled beside `LiquidSweepStart` and not at the top of the disease sweep, because
// the snapshot the disease sweep reads is the one taken — *before*
// `UpdateLiquid`, not after it.
struct DiseaseEntry {
  uint8_t idx = 0xFF;
  int32_t count = 0;
  // The infestation age goes in the snapshot too, because `Disease::GetDiffusionScale` gates
  // on it and reads it from `SimData::cells` like everything else it reads.
  uint8_t infest = 0;
};
inline std::vector<DiseaseEntry>& DiseaseSweepStart() {
  static thread_local std::vector<DiseaseEntry> start;
  return start;
}

// The same shape, one substep-phase earlier: `UpdatePressure`'s inline germ transfer
// reads the *count* it prices its fraction against from a snapshot too —
// `Disease::AddDiseaseToCell`'s live grid is distinct from the snapshot the divide reads,
// matching the mass split (`start[c].mass` vs `cells[c].mass`) in the same function. Reading
// live instead cascades a source cell's germs across an entire
// vacuum room in one substep — the destination of one pair becomes a fresh, undrained
// source for its own next pair before the snapshot would have reset it — which `germvac`'s
// tick-2 output does not show (Klei moves exactly one row, not the whole chamber). Filled
// once per `StepGasPressure` call, the same cadence `GasSweepStart` uses.
inline std::vector<DiseaseEntry>& GasDiseaseSweepStart() {
  static thread_local std::vector<DiseaseEntry> start;
  return start;
}

// ------------------------------------------------------- refreshing a snapshot per region
//
// Klei takes `CellSOA::CopyFrom` once per SUBSTEP, four times in all, and then runs its cell
// tasks off that one copy. This sim takes one per REGION instead, because the region loop in `StepPhysics`
// wraps every kernel rather than sitting inside one. On a single-region world those are the
// same thing. On a multi-region one they are not, and the difference is R copies of the
// whole grid where Klei makes one.
//
// THAT IS NOT A ROUNDING ERROR. `Game.UnsafeSim200ms` builds one `SimActiveRegion` per
// DISCOVERED WORLD, so a Spaced Out cluster save sends one region per asteroid -- 5 to 12
// in a late game, against 1 in the base game, which is the only shape any suite here had
// ever run. Measured with `bench --regions cluster:N`, which cuts one asteroid into N
// disjoint strips so the CELL WORK IS IDENTICAL (the census's cells/frame does not move by a
// single cell from N=1 to N=12) and only the region count changes:
//
//   regions      1        2        4        8       12
//   StepFlow  0.163    0.363    0.610    0.847    1.203 ms   <- 7.4x for the same sweep
//   pressure  1.588    1.779    2.015    2.346    2.659
//   frame     7.123    7.623    8.187    8.880    9.961      <- +40 %, zero extra physics
//
// `StepFlow` copies 1.19 MB per region; twelve of those is 14.3 MB of traffic per substep,
// and 1.04 ms at ~14 GB/s is exactly the gap. A real cluster is worse than this table, which
// holds the GRID fixed: the game's grid grows with the asteroid count too, so the true shape
// is R copies of an R-times-larger grid.
//
// THE FIX, AND WHY IT IS SHAPED LIKE THIS. Region 0 still copies the whole grid, so a
// single-region world -- the base game, and every scenario in every suite -- executes
// literally the code it did before and cannot regress. Regions 1..R-1 refresh only the
// rectangle their own sweeps can read. The values inside that rectangle are the same ones a
// whole-grid copy would have written; the cells outside it keep region 0's copy from earlier
// in this same substep, which is what Klei's single copy would have left there anyway.
//
// THE MARGIN IS 3 because that is the widest reach any reader of these snapshots has. The
// displacement sweeps clamp x to `r.x0 - 3 .. r.x1 + 3` and read a `beyond` cell one past
// their own bound, and `StepGasPressure` reads its far end out of the INCLUSIVE region, one
// row above `r.y1`. The y range starts at row 0 rather than at the region because of Klei's
// `min(y0, 3)` -- the displacement sweeps deliberately begin near the bottom of the GRID
// whatever the region says, which `StepGasDisplacement` documents in full. A rectangle that
// is too big is safe and still O(region); one that is too small is a stale read, which is
// why the margins here are generous rather than tight.
struct SnapshotRect {
  int32_t x0, y0, x1, y1;
};
inline SnapshotRect SnapshotReadRect(int32_t pw, int32_t ph, int32_t rx0, int32_t ry0,
                                     int32_t rx1, int32_t ry1) {
  (void)ry0;
  SnapshotRect q;
  q.x0 = rx0 - 3 > 0 ? rx0 - 3 : 0;
  q.y0 = 0;
  q.x1 = rx1 + 3 < pw ? rx1 + 3 : pw;
  q.y1 = ry1 + 3 < ph ? ry1 + 3 : ph;
  return q;
}

// Whether this region has to take the whole grid. Region 0 always does -- it is Klei's copy,
// and it is what every single-region world has always executed -- and so does the first call
// on a world whose grid has changed size, so that no cell outside any region is ever left
// holding a default-constructed entry.
inline bool SnapshotNeedsWholeGrid(size_t have, size_t n, size_t ri) {
  return ri == 0 || have != n;
}

// The rows of `q`, as half-open [begin, end) index runs into a padded grid.
template <class Row>
inline void ForEachSnapshotRow(int32_t pw, const SnapshotRect& q, Row row) {
  if (q.x1 <= q.x0) return;
  for (int32_t y = q.y0; y < q.y1; ++y) {
    const size_t base = static_cast<size_t>(y) * static_cast<size_t>(pw);
    row(base + static_cast<size_t>(q.x0), base + static_cast<size_t>(q.x1));
  }
}

// The world. Padded dimensions internally, unpadded on the way out.
class World {
 public:
  // First-party extension properties are registered HERE, before anything can allocate a
  // world, for two reasons. Registration closes at the first `Allocate`, so a lazier moment
  // would be a moment too late; and doing it in the constructor pins the first-party indices
  // ahead of every third-party one, so `sim.thermal_mass_bonus` is index 0 in every process
  // and a mod's registration order cannot shift it.
  //
  // It is `kCheckpointOnly` (it was registered before `kSaved` existed and has not been
  // reclassified) rather than `kRehydrated`: `physics.h`'s conduction kernel READS this array, which makes a
  // replay that is missing it conduct differently and diverge, and a standalone tool driving
  // the DLL loads no mods and so has no owner to re-push anything. That is the same
  // argument that put the random state, the scheduling counters, the stable ticks, the
  // disease growth and the component registries in the checkpoint.
  //
  // BEHAVIOUR TODAY IS IDENTICAL TO THE MEMBER VECTOR'S: nothing carries this array. No
  // ordinary save did before and none does now, and the extension checkpoint that would
  // honour `kCheckpointOnly` is stage 2b and does not exist yet. The declaration is still
  // the point — it is the difference between a gap nobody stated and a gap with a name,
  // and stage 2b is the stage that has to go looking for exactly this list.
  World() {
    // The only ways this can fail are a typo in the name constant and a mistake in
    // ext_registry.h's validator — both build-time bugs, neither of which world.h has a
    // logger to shout about. So the reason is KEPT rather than dropped on the floor, and
    // `simdll.cpp` reports it once at initialisation. A first-party registration that
    // silently did not happen would present later as a null `ThermalMassBonusData()`, which
    // is a much longer walk back to the same typo.
    thermal_mass_bonus_prop_ = ext_cells_.Register(
        ext::kThermalMassBonusProperty, ext::kExtF32, ext::kCheckpointOnly, /*arity=*/1, 0u,
        /*first_party=*/true, &ext_registration_error_);
    if (thermal_mass_bonus_prop_ >= 0) ext_registration_error_.clear();
    // The four kSaved first-party properties that older save versions carried as fixed
    // sections. Registration order IS blob order, so it is also the
    // order a v18 section reads back in; do not reorder these without understanding that.
    constexpr int32_t kSlots = oni_sim::gas::kMaxSpeciesPerCell;
    gas_occupied_mask_prop_ = ext_cells_.Register(
        ext::kGasOccupiedMaskProperty, ext::kExtU8, ext::kSaved, /*arity=*/1, 0u,
        /*first_party=*/true, &ext_registration_error_);
    gas_species_prop_ = ext_cells_.Register(
        ext::kGasSpeciesProperty, ext::kExtU16, ext::kSaved, kSlots,
        static_cast<uint32_t>(oni_sim::gas::kEmptySpecies), /*first_party=*/true,
        &ext_registration_error_);
    gas_mass_prop_ = ext_cells_.Register(
        ext::kGasMassProperty, ext::kExtF32, ext::kSaved, kSlots, 0u, /*first_party=*/true,
        &ext_registration_error_);
    room_promoted_prop_ = ext_cells_.Register(
        ext::kRoomPromotedProperty, ext::kExtU8, ext::kSaved, /*arity=*/1, 0u,
        /*first_party=*/true, &ext_registration_error_);
    if (gas_occupied_mask_prop_ >= 0 && gas_species_prop_ >= 0 && gas_mass_prop_ >= 0 &&
        room_promoted_prop_ >= 0) {
      ext_registration_error_.clear();
    }
    // Layer C: dissolved gas, AFTER the four above so their blob order is unchanged. Carried by
    // liquid from the start -- the sim owns this property, so it does not wait for a message.
    dissolved_mass_prop_ = ext_cells_.Register(
        ext::kDissolvedMassProperty, ext::kExtF32, ext::kSaved, ext::kDissolvedGasLanes, 0u,
        /*first_party=*/true, &ext_registration_error_);
    if (dissolved_mass_prop_ >= 0) {
      ext_cells_.SetFollowsLiquidMass(dissolved_mass_prop_, true);
      // And it MIXES from the start, for the same reason it is transported from the start: the
      // sim owns this property, so it does not wait for a message to behave the way dissolved
      // gas behaves. A mod that wants a different rate, or none, sends `ext::kSetPayloadMixing`.
      ext_cells_.SetMixShare(dissolved_mass_prop_, ext::kPayloadMixingDefaultShare);
    }
  }

  // Empty unless a first-party registration above was refused. See the constructor.
  const std::string& ExtRegistrationError() const { return ext_registration_error_; }

  void Allocate(int32_t game_width, int32_t game_height) {
    // Seeding and loading write the static arrays through the members directly rather than
    // through the Mutable* accessors, so they say "all of it" by hand.
    static_dirty_.clear();
    static_dirty_all_ = true;
    game_width_ = game_width;
    game_height_ = game_height;
    // `AllocateCells` builds a *new* `SimData`, so the five radiation tunables go back to
    // the constructor's defaults with it — a params message does not survive a reallocation.
    radiation_linger_rate_ = 1.1f;
    radiation_max_mass_ = 2000.0f;
    radiation_base_weight_ = 0.3f;
    radiation_density_weight_ = 0.7f;
    radiation_constructed_factor_ = 0.8f;
    width_ = game_width + 2;
    height_ = game_height + 2;
    const size_t n = PaddedCount();
    phases_.assign(n, {});
    cell_energy_carry_.clear();
    cell_energy_carry_held_ = 0.0;
    radiation_.assign(n, 0.0f);
    disease_.assign(n, SaveDisease{});
    disease_idx_.assign(n, 0xFF);
    disease_accum_.assign(n, 0.0f);
    disease_infest_.assign(n, 0);
    backwall_.assign(n, SaveBackwall{});
    properties_.assign(n, 0);
    // 255, not 0. `CellSOA::GetInsulationValue` squares this over 255^2, so 0 zeroes solid
    // conduction outright and 255 is the game's own "not insulated".
    // `InitializeFromCells` overwrites every one of these explicitly per cell right after
    // Allocate, so this default only reaches daylight on the path that doesn't: `FromBlob`,
    // where the save carries no insulation byte at all (confirmed against SIM_BeginSave's own
    // 36-byte cell record) and the only other writer is the game's Insulator/Door components,
    // which only ever touch cells a building sits on. A cell nothing wrote to needs the neutral
    // value, not the minimum one.
    insulation_.assign(n, 255);
    strength_.assign(n, 0);
    // Every registered extension property, sized and defaulted in one call, and
    // registration closes here. See ext_registry.h: this is the line that replaced a
    // per-property `assign` and is the reason adding one no longer edits this function.
    ext_cells_.Allocate(n);
    ext_orphans_.clear();
    // The cached hoists. `ext_cells_.Allocate` is the only thing that sizes extension storage
    // and registration closed at the same moment, so these are refreshed here and nowhere
    // else -- on the line after the allocation, which is the one place a future change could
    // otherwise dangle one of them.
    thermal_mass_bonus_data_ = ext_cells_.F32(thermal_mass_bonus_prop_);
    gas_occupied_mask_data_ = ext_cells_.MutableU8(gas_occupied_mask_prop_);
    gas_species_data_ = ext_cells_.MutableU16(gas_species_prop_);
    gas_mass_data_ = ext_cells_.MutableF32(gas_mass_prop_);
    room_promoted_data_ = ext_cells_.MutableU8(room_promoted_prop_);
    dissolved_mass_data_ = ext_cells_.MutableF32(dissolved_mass_prop_);
    RefreshLiquidPayload();
    payload_released_.clear();
    payload_consumed_.clear();
    // The gas-mixture per-cell state, sized and zeroed.
    gas_dirty_.assign(n, 0);
    gas_sleeping_.assign(n, 0);

    rooms_dirty_ = false;
    active_.assign(n, 1);
    active_incl_.assign(n, 1);
    // A world that has never been sent a region behaves as though the whole grid were in
    // play, and the rectangles have to say the same thing the masks do.
    padded_.assign(1, {1, 1, width_ - 1, height_ - 1});
    padded_incl_ = padded_;
    region_cosmic_.assign(1, 0.0f);
    region_sunlight_.assign(1, 0.0f);
    substance_touched_.assign(n, 0);
    stable_ticks_.assign(n, kStableTicksReroll);
    flow_.assign(n * 4, 0.0f);
    flow_touched_.clear();
    project_row_x0_.assign(static_cast<size_t>(height_), kNoDirtyRow);
    project_row_x1_.assign(static_cast<size_t>(height_), -1);
    project_y0_ = height_;
    project_y1_ = -1;
    project_dirty_all_ = true;
    ledger_ = Ledger{};
    energy_ledger_ = EnergyLedger{};
    allocated_ = true;
  }

  // ------------------------------------------------------------------- the flow accumulator
  //
  // Four floats a cell, in padded cell indices, and the only per-cell
  // field in the sim that records a *transfer* rather than a state. Nothing reads it but
  // `UpdateFlowTexture`, which takes `[0] - [1]` for the published x and
  // `[3] - [2]` for the published y — so both ends of a transfer land on the same signed
  // value, and the sign is the direction the mass went in.
  //
  // Every write goes through `AddFlow`, which records the cell as well. The recording is
  // what keeps the buffer affordable: it is 6 MB on a 512x768 asteroid and both the clear
  // and the texture pass would otherwise be whole-grid walks for a field that is zero
  // wherever nothing is moving — the solid two thirds of a real world, always.
  //
  // A write of exactly zero is dropped rather than recorded. Klei performs it — the refused
  // half of a pressure pair still runs the accumulate with the returned 0.0 — but adding
  // zero cannot change a float, and every cell in the world takes one of those every
  // substep, so recording them would put the whole grid on the list and undo the point.
  std::vector<float>& FlowAccum() { return flow_; }
  const std::vector<float>& FlowAccum() const { return flow_; }
  // `{padded, game}` for each recorded cell. The texture pass has neither index and needs
  // both — it reads the accumulator and the cell by padded index and writes the texture by
  // game index — and recovering one from the other is `GameIndex`, a 64-bit division by a
  // run-time width. Every caller already has both for free, so they are carried.
  //
  // Worth 0.03 ms of the texture pass's 0.34 on an asteroid, and no more: the division was
  // the obvious suspect and the cost is really the scattered traffic either side of it.
  // Kept because it is measurably faster and the callers pay nothing, not because it was
  // the fix it was expected to be.
  struct FlowCell {
    uint32_t padded;
    uint32_t game;
  };
  const std::vector<FlowCell>& FlowTouched() const { return flow_touched_; }
  void AddFlow(size_t cell, size_t game_cell, int slot, float amount) {
    if (amount == 0.0f) return;
    float* f = &flow_[cell * 4];
    // Recorded once, on the write that takes the cell off zero. A gas cell on a live
    // asteroid takes four or five of these a substep and the texture pass wants it once.
    if (f[0] == 0.0f && f[1] == 0.0f && f[2] == 0.0f && f[3] == 0.0f) {
      flow_touched_.push_back(
          FlowCell{static_cast<uint32_t>(cell), static_cast<uint32_t>(game_cell)});
    }
    f[static_cast<size_t>(slot)] += amount;
  }
  void ClearFlow() {
    for (const FlowCell& c : flow_touched_) {
      float* f = &flow_[static_cast<size_t>(c.padded) * 4];
      f[0] = 0.0f;
      f[1] = 0.0f;
      f[2] = 0.0f;
      f[3] = 0.0f;
    }
    flow_touched_.clear();
    flow_element_.clear();
  }

  // What the flow texture's gate compares against.
  //
  // `UpdateFlowTexture` publishes nothing for a cell whose `updatedCells`
  // element differs from its `cells` element, and the `cells` copy it reads is the substep's
  // **fourth and last** `CellSOA::CopyFrom`, — after `UpdateLiquid` *and*
  // after the liquid displacement pass, not the copy that `LiquidSweepStart`
  // holds. The two were indistinguishable for as long as nothing between them changed an
  // element, and the comment on `LiquidSweepStart` calling "the last one it
  // takes at all" was simply wrong.
  //
  // So the gate is really "did anything change this cell's element *after* the liquid
  // section" — the disease sweep, the components, `PostProcessCell`, `ZeroMasslessCells` —
  // and a cell the liquid mover poured into publishes its flow rather than being silenced.
  // That was `sunliquid`'s last divergence: one cell, (12,1), where water arrived during
  // `UpdateLiquid` and our texture read `-0` because the scale was zero.
  //
  // Recorded per flow-touched cell rather than as a grid snapshot: the texture only ever
  // asks about cells the accumulator wrote, so this is a few hundred entries on an asteroid
  // where a `CellSOA::CopyFrom` is a megabyte. Every substep overwrites it, so what survives
  // to publish time is the last substep's — which is Klei's copy.
  void SnapshotFlowElements() {
    const size_t n = flow_touched_.size();
    // GROW GEOMETRICALLY, EXPLICITLY. `ClearFlow` empties this vector every frame, and a
    // `resize(n)` on an EMPTY vector defeats the library's doubling: libstdc++ takes
    // `max(size() + delta, 2 * size())`, which with `size() == 0` is exactly `n`. So capacity
    // landed on precisely this frame's count, next frame's count was a hundred cells larger,
    // and it reallocated -- 65 KB, once per new high-water mark, which with capacity set to
    // exactly the mark means every incremental growth rather than every doubling. Measured on
    // the asteroid: 56 allocations over 49 ticks, 98 over 99, 98 over 199 -- it stops when the
    // world stops opening new space, and 9 is what the same growth costs with doubling
    // restored. Found by the steady-state allocation assertion in `bench`
    // (search "allocations:"), which is the only thing here that could have found it: it costs
    // about a microsecond, so no timing was ever going to show it, and the census counts cells
    // rather than mallocs. Reserving before the resize restores the doubling and leaves the
    // clear-every-frame semantics exactly as they were.
    if (flow_element_.capacity() < n) flow_element_.reserve(n * 2);
    flow_element_.resize(n);
    for (size_t i = 0; i < n; ++i) {
      flow_element_[i] = phases_[flow_touched_[i].padded].element;
    }
  }
  const std::vector<uint16_t>& FlowElements() const { return flow_element_; }

  // Which cells a kernel wrote a substance into this frame.
  //
  // This is not a diff and it is not a set of cells whose element changed. Klei records it
  // by hand: `SimEvents::ChangeSubstance` pushes `{ gameCell, 0xffff, 0xffff }`
  // — the element indices are *not* captured at the call — and `SimBase::CopySimDataToGame`
  // fills `oldElemIdx` from the previously published element array and `newElemIdx` from the
  // one it is publishing,. A cell swapped with a neighbour
  // holding the same gas is therefore announced as `183 -> 183`, and that is the bulk of the
  // traffic in any room with a shuffle in it.
  //
  // Klei sorts the list by cell and runs `std::unique` over it before publishing (the
  // `_Buffered_rotate_unchecked<SubstanceChangeInfo>` in `.text` is `std::stable_sort`'s
  // merge, and is the unique pass walking backwards). A flag per cell scanned in
  // game-cell order at projection time gives sorted-and-deduplicated for free, which is why
  // this is a bitmap and not a list — and it means the *order* kernels touch cells in cannot
  // leak into the output.
  //
  // `ChangeSubstance` does one other thing on the way out, on **both** exits — the one that
  // pushed and the one whose game index fell outside the world — and it is the reason this
  // and `MarkUnstableDirty` are the same call: both set the cell's unstable bits (`|= 0x1f`).
  // Writing a substance into a cell restarts that cell's
  // unstable-solid countdown.
  void TouchSubstance(size_t padded) {
    // THE CENSUS'S ONE CHANGE SITE (sim/census.h). This is the sim's own "this cell changed"
    // announcement -- what the game consumes, and what every substance-moving kernel already
    // calls -- so charging the count here rather than at a branch inside each kernel means the
    // counter cannot drift from the behaviour it claims to describe. It is charged to whatever
    // sweep is currently running, which `ONI_SWEEP` owns; a call from outside any sweep (a
    // queued `ModifyCell`, an emitter) lands in the out-of-sweep bucket rather than on the
    // kernel that happened to run last.
    ::oni_sim::census::NoteChange();
    substance_touched_[padded] = 1;
    stable_ticks_[padded] |= kStableTicksReroll;
  }
  bool SubstanceTouched(size_t padded) const { return substance_touched_[padded] != 0; }

  // `SimEvents::cellMeltedInfo`, the game cells whose tile melted this frame, in the order the
  // transitions ran. Klei's only writer is `DoStateTransition`'s high branch: a cell that heats past its melting point while its property byte
  // carries `kNotifyOnMelt` is reported, and the game destroys the tile that set the bit and
  // raises "building melted". Kept here, beside the touch bitmap, rather than threaded through
  // `TransitionCell`'s four callers, because every one of them reaches the report in Klei.
  // A record of one frame, like the touch bitmap: cleared by the frame's event reset.
  void NoteCellMelted(int32_t game) { cell_melted_.push_back(game); }
  const std::vector<int32_t>& CellMelted() const { return cell_melted_; }
  void ClearCellMelted() { cell_melted_.clear(); }
  void ClearSubstanceTouched() {
    std::fill(substance_touched_.begin(), substance_touched_.end(), 0);
  }

  // One byte a cell, **persistent across frames** — unlike the touch
  // bitmap above, which is a record of one frame. The low five bits are an unstable solid's
  // countdown to falling and the high three are never written by anything.
  //
  // `0x1f` is not a count of 31. It is the sentinel `SimData::GetStableTicksRemaining`
  // reads as "roll a fresh one", and rolling is the **only** place in the
  // unstable path that touches the random stream. Everything else about falling sand is
  // deterministic; this one byte decides which cells draw and when, which is why it has to
  // be exact rather than approximately right.
  //
  // The array starts out all-sentinel: a world that has just loaded has had nothing written
  // into any of its cells, so every unstable solid the active region reaches rolls before it
  // moves. Measured — see `sand` in `diffsim` — not assumed.
  static constexpr uint8_t kStableTicksReroll = 0x1f;
  void MarkUnstableDirty(size_t padded) { stable_ticks_[padded] |= kStableTicksReroll; }

  // `GetStableTicksRemaining` verbatim, including the two things about it that look like
  // mistakes and are not. It **returns the decremented value**, so a cell that reads 1 this
  // substep reads 0 and falls on the next one rather than this one. And on the reroll path
  // it returns the whole seeded byte while storing only its low five bits — which is the
  // same number here, because the seed is `(int)(r * 3/32767) + 3` and so is 3, 4, 5 or 6.
  //
  // `scale` and `base` are the tunables `StableTicksRerollScale` (default bits 0x38C00180,
  // 3.0f / 32767.0f) and `StableTicksRerollBase` (3), passed in by the post-process kernel
  // that hoisted them (tunables.h). Their product must stay under the sentinel: a roll of 31
  // would read back as "roll again".
  uint8_t StableTicksRemaining(size_t padded, float scale, int32_t base) {
    uint8_t& b = stable_ticks_[padded];
    const uint8_t v = static_cast<uint8_t>(b & kStableTicksReroll);
    if (v == kStableTicksReroll) {
      // The same `rand()` the shuffle draws from, and the same 1/32767 scale, times three.
      const uint32_t s = NextRandomState();
      const float f = static_cast<float>((s >> 16) & 0x7FFFu) * scale;
      const uint8_t rolled = static_cast<uint8_t>(static_cast<int32_t>(f) + base);
      b = static_cast<uint8_t>((b & 0xE0u) | (rolled & kStableTicksReroll));
      return rolled;
    }
    if (v == 0) return 0;
    const uint8_t next = static_cast<uint8_t>(v - 1);
    b = static_cast<uint8_t>((b & 0xE0u) | next);
    return next;
  }

  // The whole countdown array, for `ext::kSetStableTicks` and `SIM_DebugStableTicks`.
  // Exposed wholesale rather than a cell at a time because a caller that wants it wants all
  // of it: this is the one piece of per-cell state no save carries, so restoring a run to a
  // point in its past means writing the entire array back, not patching cells.
  const std::vector<uint8_t>& StableTicks() const { return stable_ticks_; }

  // Rejects a length that is not this world's padded cell count rather than writing what
  // fits: a short array would leave the tail at the reroll sentinel, which is exactly the
  // state the caller is trying to get rid of, and a silent partial restore is worse than a
  // refused one because the run would look restored and would not be.
  bool SetStableTicks(const uint8_t* src, size_t count) {
    if (!src || count != stable_ticks_.size()) return false;
    memcpy(stable_ticks_.data(), src, count);
    return true;
  }

  // A flag set by `AllocateCells` and `SimData_InitializeFromCells`. It decides
  // which half of the unstable path runs: the game gets an `unstableCellInfo` and an emptied
  // cell, so that it can spawn a falling-object entity, and an offline sim gets the fall
  // simulated in the grid instead. `diffsim` sends 1; the game sends 0 in normal play and 1
  // from worldgen.
  bool Headless() const { return headless_; }

  // The active region, from `NewGameFrame`.
  //
  // The sim does not step the whole grid. Each frame the game sends one or more regions and
  // only cells inside them are simulated — measured by putting a hot cell one row under the
  // bound and watching it exchange heat downward and not upward. A pair whose far side is
  // outside the region exchanges *nothing*: the same cell cooled 22.6 K with four active
  // neighbours and 17.1 K with three, which is exactly three quarters.
  //
  // This matters more than it looks. The game clamps `maxY` to `HeightInCells - 1`, so the
  // world's top row is never simulated in a real game either, and a replacement that steps
  // it anyway drifts from Klei in every world whose top row is not uniform. It went
  // unnoticed here for as long as it did because every scenario's top row was identical
  // granite; the first scenario with something interesting up there found it immediately.
  //
  // Cells default to active, so a sim that has not been told otherwise behaves as though
  // the whole grid were in play rather than freezing.
  void SetActiveRegions(const int32_t* regions, size_t count,
                        const float* cosmic = nullptr, const float* sunlight = nullptr) {
    if (!allocated_ || count == 0) return;
    active_.assign(PaddedCount(), 0);
    active_incl_.assign(PaddedCount(), 0);
    regions_.clear();
    padded_.clear();
    padded_incl_.clear();
    region_cosmic_.clear();
    region_sunlight_.clear();
    for (size_t r = 0; r < count; ++r) {
      const int32_t* q = regions + r * 4;
      // Kept as rectangles as well as as a mask. The per-cell kernels want the mask; the
      // building components want the rectangle, because Klei tests a building's *extents*
      // against the region rather than testing its cells (`BuildingHeatExchange::Update`,
      //), and a building straddling the edge is skipped whole.
      regions_.push_back({q[0], q[1], q[2], q[3]});
      const int32_t min_x = q[0] < 0 ? 0 : q[0];
      const int32_t min_y = q[1] < 0 ? 0 : q[1];
      const int32_t max_x = q[2] > game_width_ ? game_width_ : q[2];
      const int32_t max_y = q[3] > game_height_ ? game_height_ : q[3];
      for (int32_t y = min_y; y < max_y; ++y) {
        for (int32_t x = min_x; x < max_x; ++x) {
          active_[static_cast<size_t>(y + 1) * width_ + (x + 1)] = 1;
        }
      }
      const int32_t max_xi = q[2] >= game_width_ ? game_width_ - 1 : q[2];
      const int32_t max_yi = q[3] >= game_height_ ? game_height_ - 1 : q[3];
      for (int32_t y = min_y; y <= max_yi; ++y) {
        for (int32_t x = min_x; x <= max_xi; ++x) {
          active_incl_[static_cast<size_t>(y + 1) * width_ + (x + 1)] = 1;
        }
      }
      // The same two masks as rectangles, in padded coordinates and half-open, so a sweep
      // can iterate exactly the cells the mask would have accepted instead of walking the
      // grid and asking. `PaddedRect(x0,y0,x1,y1)` and `Active(padded)` are two spellings
      // of one set by construction: everything below is the loop bounds above, shifted by
      // the border ring. The inclusive variant is the same rectangle with its far edge one
      // cell further out, which is what `max_xi`/`max_yi` mean.
      padded_.push_back({min_x + 1, min_y + 1, max_x + 1, max_y + 1});
      region_cosmic_.push_back(cosmic == nullptr ? 0.0f : cosmic[r]);
      region_sunlight_.push_back(sunlight == nullptr ? 0.0f : sunlight[r]);
      padded_incl_.push_back({min_x + 1, min_y + 1, max_xi + 2, max_yi + 2});
    }
    // What a correct sweep is allowed to look at, once, per invocation -- and what a sweep
    // that ignored its regions would look at instead. Recorded here because this is the only
    // place both numbers are known at once, and because the readers (`vftest`, the live
    // profiler) hold no `World` to ask. Rectangles may overlap, so this is a sum of areas and
    // not a count of distinct cells: it is an upper bound on the correct answer, which is the
    // safe direction for a bound a test refuses on.
    ::oni_sim::census::g_regions = static_cast<uint64_t>(padded_.size());
    ::oni_sim::census::g_grid_cells = static_cast<uint64_t>(PaddedCount());
    uint64_t area = 0;
    for (const PaddedRect& q : padded_) {
      if (q.x1 > q.x0 && q.y1 > q.y0) {
        area += static_cast<uint64_t>(q.x1 - q.x0) * static_cast<uint64_t>(q.y1 - q.y0);
      }
    }
    ::oni_sim::census::g_region_cells = area;
    uint64_t area_incl = 0;
    for (const PaddedRect& q : padded_incl_) {
      if (q.x1 > q.x0 && q.y1 > q.y0) {
        area_incl += static_cast<uint64_t>(q.x1 - q.x0) * static_cast<uint64_t>(q.y1 - q.y0);
      }
    }
    ::oni_sim::census::g_region_cells_incl = area_incl;
  }
  bool Active(size_t padded) const { return active_[padded] != 0; }

  // The same regions, read **inclusively** and in game coordinates.
  //
  // `NewGameFrame` sends `maxY = height - 1`, and the mask above treats that half-open, so
  // the world's top row is never conducted — which is measured and load-bearing: making the
  // mask inclusive instead puts eight scenarios tens of kelvin out. The gas pressure sweep
  // does *not* agree with it. `sunlit` is the only scenario that can tell, because it is the
  // only one whose top row holds something other than uniform border: Klei announces a
  // substance change for every vacuum cell in that row on every frame, and does so whether
  // the harness sends `maxY = height - 1` or `height`, so Klei is clamping internally and
  // then including the clamped row. Nothing in the suite has *movable* gas up there, so this
  // is the announce reproducing exactly rather than a claim about the flow: the two are
  // indistinguishable here, and a scenario with gas on the top row would separate them.
  // Kept as a second mask rather than a rectangle scan: `StepGasPressure` asks this once
  // per *pair*, and walking the region list there cost 0.5 ms a frame on the asteroid.
  bool ActiveInclusive(size_t padded) const { return active_incl_[padded] != 0; }
  const std::vector<uint8_t>& ActiveMask() const { return active_; }

  // The two masks above, as rectangles a sweep can iterate.
  //
  // Padded coordinates, half-open on both axes, already clamped to the interior — so
  // `for (y = r.y0; y < r.y1; ++y) for (x = r.x0; x < r.x1; ++x)` visits exactly the cells
  // `Active` accepts, in exactly the order a full-grid scan would visit them, and a sweep
  // that switches from one to the other is unchanged on any world with a single region.
  //
  // **Klei's structure is region-outer, and since the multi-region refactor so is this.**
  // `SimBase::UpdateData` is one loop over the region list at stride 0x18, and
  // the *entire* frame body sits inside it — conduction, both gas sweeps, liquid,
  // post-process, the components, right down to `Disease::PostProcess`. Klei
  // runs the whole physics frame once per region; two overlapping regions get the frame run
  // twice over their overlap. So `StepPhysics` owns the loop and every sweep now takes the
  // index of the region it is running over, rather than each sweep walking the list itself.
  // The two orderings agree exactly when there is one region, which is every scenario in the
  // suite that predates the region probes and every single-asteroid game; `regionadj` and
  // `regionlap` are the two that can tell them apart.
  struct PaddedRect {
    int32_t x0, y0, x1, y1;
  };
  const std::vector<PaddedRect>& PaddedRegions() const { return padded_; }
  const std::vector<PaddedRect>& PaddedRegionsInclusive() const { return padded_incl_; }

  // How many times the frame body runs. Always at least one: a world that has never been
  // sent a region carries a single rectangle covering the whole interior, which is what
  // makes "no region" and "one region spelled out" the same run — `regionone` is the
  // control that says so.
  //
  // `padded_`, `padded_incl_` and `regions_` are three spellings of one list and are pushed
  // together, except that `regions_` is empty in the defaulted case where the other two hold
  // the whole-world rectangle. Index with this and go through the accessors below.
  size_t RegionCount() const { return padded_.size(); }
  const PaddedRect& PaddedRegion(size_t ri) const { return padded_[ri]; }
  // `NewGameFrame` carries a sunlight and a cosmic-radiation intensity **per region**, and
  // Klei's `activeRegions` record is six ints wide because of it. The cosmic figure is the
  // one the sim reads: every unoccluded cell gains `intensity / RADIATION_LINGER_RATE` of it
  // per substep. A world that has never been sent a frame has none.
  float RegionCosmic(size_t ri) const {
    return ri < region_cosmic_.size() ? region_cosmic_[ri] : 0.0f;
  }
  // The FIFTH int of the same record. The game's own sunlight texture does not read it
  // (measured -- `sim/textures.h` lights a world's top row at 255 with this sent as zero). It is `WorldContainer.currentSunlightIntensity` in
  // lux: `TimeOfDay.UpdateSunlightIntensity` sets it to `sin(pi * cycleFraction / dayFraction)`
  // times that world's own `sunlight` fixed trait, zeroes it at night and during an eclipse, and
  // `Game.UnsafeSim200ms` copies it into the region record for every DISCOVERED world. So the
  // game has been telling the sim where each world is in its own day, every frame, for free.
  // Mod 3's solar heating is what finally reads it.
  float RegionSunlight(size_t ri) const {
    return ri < region_sunlight_.size() ? region_sunlight_[ri] : 0.0f;
  }
  const PaddedRect& PaddedRegionInclusive(size_t ri) const { return padded_incl_[ri]; }

  // Game coordinates, half-open in x and y, exactly as `NewGameFrame` sends them.
  //
  // A world that has never been sent a region reports none, and the building components
  // read that as "everything is in play" — the same default the mask uses.
  struct Rect {
    int32_t min_x, min_y, max_x, max_y;
  };
  const std::vector<Rect>& Regions() const { return regions_; }

  // The region the frame body is currently running over, in game coordinates. The defaulted
  // case has no `regions_` entry to hand back, so it reports the whole world — which is the
  // rectangle `padded_` holds for that case, read back without the border ring.
  Rect Region(size_t ri) const {
    if (regions_.empty()) return Rect{0, 0, game_width_, game_height_};
    return regions_[ri];
  }

  // A building exchanges heat only if its **extents** sit inside the region, rather than
  // cell by cell (`BuildingHeatExchange::Update`), so one straddling an edge is
  // skipped whole. Since the components run inside `UpdateData`'s region loop
  // (`SimData::UpdateComponents`) the question is asked of one region at a
  // time, and a building inside two of them exchanges twice.
  bool ExtentsInRegion(size_t ri, int32_t min_x, int32_t min_y, int32_t max_x,
                       int32_t max_y) const {
    if (regions_.empty()) return true;
    const Rect& r = regions_[ri];
    return min_x >= r.min_x && min_y >= r.min_y && max_x <= r.max_x && max_y <= r.max_y;
  }

  bool Allocated() const { return allocated_; }
  // The worlds a cluster map is made of, from `DefineWorldOffsets`: four
  // int32s each, in game coordinates. A single-asteroid game sends one; an offline harness
  // that never sends the message has none, and the sunlight texture stays zero.
  struct WorldOffset {
    int32_t x = 0, y = 0, w = 0, h = 0;
  };
  const std::vector<WorldOffset>& WorldOffsets() const { return world_offsets_; }
  // `DefineWorldOffsets` is an immediate message — Klei acts on it inside the handler — but
  // the sunlight texture it feeds does **not** light up on the frame the message arrives:
  // Klei publishes zeros for that frame and the geometry from the next one. Measured, not
  // assumed: `sunlight` sends the offsets before tick 1, and Klei's texture is zero at tick 1
  // and filled from tick 2. So the offsets are staged here and promoted once the frame that
  // received them has published.
  void SetWorldOffsets(std::vector<WorldOffset> v) { pending_world_offsets_ = std::move(v); }
  // `SimData::ResizeAndInitializeVacuumCells` opens a rectangle and that rectangle is a
  // *world*: the sunlight sweep lights it on the next published frame, and a solid dropped
  // into it afterwards casts a shadow down its own column — which is what says the message
  // registers geometry rather than writing 255 into the texture. Measured on `vacrect`.
  //
  // It appends rather than replaces. No scenario has both a `DefineWorldOffsets` and this
  // message, so which one Klei does is **not** established; appending is the reading that
  // matches the name — the message opens *a* world, it does not redefine the cluster.
  //
  // It takes effect **immediately**, unlike `DefineWorldOffsets`, and that is measured too:
  // staged, our rectangle lit at tick 5 against Klei's tick 4, and every later tick agreed.
  // So the two messages do not share the one-frame delay, and the delay is not a property of
  // "world geometry" — whatever produces it for `DefineWorldOffsets` does not apply here. The
  // live list and the pending one both take the rectangle so a promote cannot drop it.
  void AddWorldOffset(const WorldOffset& o) {
    if (pending_world_offsets_.empty()) pending_world_offsets_ = world_offsets_;
    pending_world_offsets_.push_back(o);
    world_offsets_.push_back(o);
  }
  void PromoteWorldOffsets() {
    if (pending_world_offsets_.empty()) return;
    world_offsets_ = pending_world_offsets_;
  }

  int32_t GameWidth() const { return game_width_; }
  int32_t GameHeight() const { return game_height_; }
  size_t GameCount() const {
    return static_cast<size_t>(game_width_) * game_height_;
  }
  int32_t PaddedWidth() const { return width_; }
  int32_t PaddedHeight() const { return height_; }
  size_t PaddedCount() const { return static_cast<size_t>(width_) * height_; }

  // Game cell index -> padded index. The game numbers cells row-major over the
  // unpadded grid; we store the border, so every row shifts by one and every cell
  // shifts by one more.
  // Every game cell in order, with its padded index carried alongside — see `CellWalk`.
  // Identical order to `for (size_t i = 0; i < GameCount(); ++i)` with `Padded(i)`, and the
  // same answer for every cell, without the division.
  GameCellRange GameCells() const {
    return GameCellRange(game_width_, GameCount(), width_);
  }

  size_t Padded(size_t game_cell) const {
    const size_t y = game_cell / static_cast<size_t>(game_width_);
    const size_t x = game_cell - y * static_cast<size_t>(game_width_);
    return (y + 1) * static_cast<size_t>(width_) + (x + 1);
  }
  bool ValidGameCell(int64_t c) const {
    return c >= 0 && static_cast<size_t>(c) < GameCount();
  }
  // The other direction, spelled the way Klei spells it — `cell / width` and `cell % width`
  // for the row and column, then `(width - 2) * (y - 1) + (x - 1)`. Every event that names a
  // cell computes it inline; `ChangeSubstance` is the shortest copy of it.
  // A border cell comes back negative or past the end, which is what callers test.
  int64_t GameIndex(size_t padded) const {
    const int64_t y = static_cast<int64_t>(padded) / width_;
    const int64_t x = static_cast<int64_t>(padded) % width_;
    return (y - 1) * game_width_ + (x - 1);
  }

  PhaseEntry& Phase(size_t padded) { return phases_[padded]; }
  const PhaseEntry& Phase(size_t padded) const { return phases_[padded]; }

  std::vector<PhaseEntry>& Phases() { return phases_; }
  const std::vector<PhaseEntry>& Phases() const { return phases_; }

  // The seven arrays below are the ones `Project` copies straight through to the game with
  // no physics in between: properties, insulation, strength, radiation, the two disease
  // fields and the three backwall fields. On a 512x768 grid, copying all of them for every
  // cell cost 1.5 ms of a 3.4 ms projection, every frame, to reproduce bytes that were
  // already there — the buffers persist between frames and the game re-reads the same
  // pointers, so an unwritten array is not a stale array, it is the same array.
  //
  // So every write to one is recorded by cell, and `Project` copies exactly the cells
  // recorded. That is only sound if *every* writer records, which is why the mutable
  // spellings are named apart from the const ones rather than overloaded on constness: an
  // overload set would hand a mutable reference to `w->Properties()` on a non-const
  // `World*` — which is what several sweeps in physics.h do to *read* it — and the writer
  // set would silently include every reader. Named apart, the compiler finds the writers.
  //
  // They are also per cell rather than per array, and that is not cosmetic either: the
  // first version of this recorded a single "something changed" epoch, and the sublimation
  // tail writes `Disease()[n].count -= 0` on every subliming cell of every substep, so a
  // world with any sublimation in it re-copied everything anyway. A whole-array hook is a
  // hook on the wrong thing.
  float& MutableRadiation(size_t p) { return MarkStatic(p), radiation_[p]; }
  // The radiation field is rewritten for **every cell of the region on every substep** — the
  // decay pass writes each cell whether or not the value moved — so going through
  // `MutableRadiation` would mark the whole grid static-dirty every frame and hand `Project`
  // a full copy of seven arrays it does not need. Writing through this instead marks only
  // the cells whose value actually changed, which is the same guarantee the projection's
  // own comment makes: each copy is either performed or provably redundant.
  void SetRadiation(size_t p, float v) {
    if (radiation_[p] == v) return;
    MarkStatic(p);
    radiation_[p] = v;
  }
  SaveDisease& MutableDisease(size_t p) { return MarkStatic(p), disease_[p]; }
  SaveBackwall& MutableBackwall(size_t p) {
    MarkBackwall(p);
    return MarkStatic(p), backwall_[p];
  }
  uint8_t& MutableProperties(size_t p) { return MarkStatic(p), properties_[p]; }
  uint8_t& MutableInsulation(size_t p) { return MarkStatic(p), insulation_[p]; }
  uint8_t& MutableStrength(size_t p) { return MarkStatic(p), strength_[p]; }
  // Framework extension storage (not from Klei — see abi/sim_abi_ext.h). Every per-cell
  // extension property lives here, addressed by the index its registration returned.
  //
  // Writes do NOT call MarkStatic, unlike Insulation/Strength: extension properties have no
  // texture and no projection into `GameDataUpdate`, so there is nothing for a dirty mark to
  // refresh. That was `MutableThermalMassBonus`'s choice before the registry existed and it is
  // ext_registry.h's now, in one place instead of one per property.
  ext::CellPropertyRegistry& ExtCells() { return ext_cells_; }
  const ext::CellPropertyRegistry& ExtCells() const { return ext_cells_; }

  // ------------------------------------------------- the seventh checkpoint component
  //
  // `ext::kSetExtCellState` / `SIM_DebugExtCellState`. Thin forwarders: the format and the
  // whole-or-nothing rule live in `sim/ext_state.h`, and the ABI header's block at
  // `kSetExtCellState` carries the complete checkpoint list. They are here rather than called
  // straight from `simdll.cpp` for the same reason `ToBlob`/`FromBlob` are: the registry is
  // the world's, and nothing outside reaches into it.
  void ExtCheckpointToBlob(std::vector<uint8_t>* out) const {
    ext_state::Save(ext_cells_, out);
  }

  // The INSPECTION twin: every registered property, all three persistence classes, in the
  // same format. Deliberately a second entry point rather than a flag on the one above --
  // the checkpoint's class selection is load-bearing (carrying a `kSaved` property would
  // restore it twice, once from the save blob and once from here) and a boolean argument
  // would put that decision at every call site instead of in `ext_state.h`.
  void ExtInspectToBlob(std::vector<uint8_t>* out) const {
    ext_state::SaveAll(ext_cells_, out);
  }

  bool ExtCheckpointFromBlob(const uint8_t* p, size_t n, std::string* error,
                             std::vector<std::string>* reported = nullptr) {
    return ext_state::Load(p, n, &ext_cells_, error, reported);
  }

  uint8_t& MutableDiseaseIdx(size_t p) { return MarkStatic(p), disease_idx_[p]; }

  // The gas-mixture layer (abi/gas_mixture_abi.h). A world that never activates it computes
  // byte-identical to vanilla. Slot layout is
  // SoA: `gas_species_`/`gas_mass_` are `PaddedCount() * kMaxSpeciesPerCell` long, cell-major
  // (all of one cell's slots together); a cell-minor, SIMD-friendly layout is a possible
  // later optimisation. Deliberately reuses `phases_[p].temperature` as the mixture's shared
  // temperature rather than adding a second field (see gas_mixture_abi.h's `GasMixtureCell`
  // comment on why one shared temperature, not per-species) — no `gas_temperature_` array here.
  //
  // These four are registered `kSaved` extension properties -- see the constructor. The
  // indexing is plain, because the registry's storage is cell-major with the
  // same stride: `(p * arity + slot) * sizeof(T)`, and the save format needs no new
  // `kSaveVersion` integer per array (saveblob.h).
  uint16_t& MutableGasSpecies(size_t p, int slot) {
    return gas_species_data_[p * oni_sim::gas::kMaxSpeciesPerCell + static_cast<size_t>(slot)];
  }
  float& MutableGasMass(size_t p, int slot) {
    return gas_mass_data_[p * oni_sim::gas::kMaxSpeciesPerCell + static_cast<size_t>(slot)];
  }
  uint8_t& MutableGasOccupiedMask(size_t p) { return gas_occupied_mask_data_[p]; }
  uint8_t& MutableGasDirty(size_t p) { return gas_dirty_[p]; }
  uint8_t& MutableGasSleeping(size_t p) { return gas_sleeping_[p]; }
  // Promotion's real source of truth, per padded cell rather
  // than per room-graph-id. RoomGraph.owned (gas_rooms.h) is a fast per-tick lookup derived
  // FROM this on every rebuild, not the other way around -- room ids are only stable between
  // rebuilds, and a rebuild (see RoomsDirty below) can split or merge rooms at any time a
  // solid boundary opens or closes. Same Allocate-time lifecycle as gas_occupied_mask_: sized
  // and zeroed there and untouched by a room-graph rebuild, so promotion survives an in-session
  // geometry change. It is a registered kSaved property (save version 17 and later), so it
  // survives a save and load too.
  uint8_t& MutableRoomPromoted(size_t p) { return room_promoted_data_[p]; }

  // ---------------------------------------------------------------- Layer C: liquid payload
  //
  // Every F32 extension property flagged `follows_liquid_mass` (ext::kSetCellPropertyTransport)
  // is an amount held by a cell's liquid. The primitives below are the ONLY way a kernel moves
  // one, and each conserves exactly: an amount leaves one place and arrives at another, or it
  // becomes a record (released / consumed) that the frame publishes.
  //
  // THE ZERO-COST GATE. `PayloadActive()` is false in every world that never wrote such a
  // property, and each primitive returns on it first. Nothing here writes a vanilla field, so a
  // world with payload is still bit-exact in every array Klei's DLL has.
  struct PayloadProperty {
    int32_t idx;
    int32_t arity;
    float* data;
    // Hoisted out of the registry with the pointer, and for the same reason: `StepPayloadMixing`
    // reads it once per property per PAIR, and a pair is two cells, so the registry lookup would
    // be the dominant cost of a kernel whose actual work is two multiplies.
    float mix_share;
  };

  // Rebuilds the hoisted list from the registry's flags. After `Allocate` and after a transport
  // message; nothing else changes the set.
  void RefreshLiquidPayload() {
    payload_props_.clear();
    size_t lanes = 0;
    for (int32_t i = 0; i < ext_cells_.Count(); ++i) {
      const ext::CellPropertyRegistry::Property* p = ext_cells_.At(i);
      if (p == nullptr || !p->follows_liquid_mass) continue;
      float* data = ext_cells_.MutableF32(i);
      if (data == nullptr) continue;
      payload_props_.push_back({i, p->arity, data, p->mix_share});
      lanes += static_cast<size_t>(p->arity);
    }
    payload_capture_.assign(lanes, 0.0f);
  }

  const std::vector<PayloadProperty>& PayloadProperties() const { return payload_props_; }

  bool PayloadActive() const {
    for (const PayloadProperty& p : payload_props_) {
      if (ext_cells_.At(p.idx)->maybe_nonzero) return true;
    }
    return false;
  }

  // Moves `fraction` of `src`'s payload to `dst`. `fraction >= 1` moves all of it exactly, so a
  // cell that gave away all its liquid keeps no rounding residue.
  void PayloadMove(size_t src, size_t dst, float fraction) {
    if (!PayloadActive() || src == dst || !(fraction > 0.0f)) return;
    for (const PayloadProperty& p : payload_props_) {
      float* s = p.data + src * static_cast<size_t>(p.arity);
      float* d = p.data + dst * static_cast<size_t>(p.arity);
      for (int32_t c = 0; c < p.arity; ++c) {
        if (s[c] == 0.0f) continue;
        const float m = fraction >= 1.0f ? s[c] : s[c] * fraction;
        s[c] -= m;
        d[c] += m;
      }
    }
  }

  void PayloadSwap(size_t a, size_t b) {
    if (!PayloadActive()) return;
    for (const PayloadProperty& p : payload_props_) {
      float* x = p.data + a * static_cast<size_t>(p.arity);
      float* y = p.data + b * static_cast<size_t>(p.arity);
      for (int32_t c = 0; c < p.arity; ++c) std::swap(x[c], y[c]);
    }
  }

  // Evens two neighbouring cells' payload out, one step towards equal CONCENTRATION -- the
  // Layer C mixing primitive (`ext::kSetPayloadMixing`, `sim/payload_mix.h`). The caller has
  // already established that both cells hold the same liquid element and that both masses are
  // positive; this does the arithmetic and nothing else.
  //
  // CONCENTRATION, NOT AMOUNT, and that is the whole of the physics in here. Two cells holding
  // the same grams per kilogram are already mixed however different their masses are, and a
  // kernel that equalised AMOUNTS would drive gas out of a full cell into the half-empty one
  // beside it and call the result uniform. So the target for `a` is its share of the pair's
  // total by MASS, and what moves is `share` of the distance to it.
  //
  // IT CONSERVES THE WAY `PayloadMove` DOES, by construction rather than by tolerance: one
  // number `m` is computed, subtracted from one lane and added to the other, so the only error
  // is the rounding of those two floats and neither cell can go negative -- `m` is clamped to
  // what the giving cell actually holds before anything is written.
  void PayloadMixPair(size_t a, size_t b, float mass_a, float mass_b) {
    if (!PayloadActive() || a == b) return;
    const float total_mass = mass_a + mass_b;
    if (!(total_mass > 0.0f)) return;
    const float share_a = mass_a / total_mass;
    for (const PayloadProperty& p : payload_props_) {
      if (!(p.mix_share > 0.0f)) continue;
      float* x = p.data + a * static_cast<size_t>(p.arity);
      float* y = p.data + b * static_cast<size_t>(p.arity);
      for (int32_t c = 0; c < p.arity; ++c) {
        const float qa = x[c];
        const float qb = y[c];
        if (qa == 0.0f && qb == 0.0f) continue;
        // What `a` would hold if the pair were uniform, minus what it holds now.
        const float gap = (qa + qb) * share_a - qa;
        if (gap > 0.0f) {
          float m = gap * p.mix_share;
          if (m > qb) m = qb;
          if (!(m > 0.0f)) continue;
          y[c] -= m;
          x[c] += m;
        } else if (gap < 0.0f) {
          float m = -gap * p.mix_share;
          if (m > qa) m = qa;
          if (!(m > 0.0f)) continue;
          x[c] -= m;
          y[c] += m;
        }
      }
    }
  }

  // True when at least one liquid-following property is both nonzero somewhere and set to mix.
  // The whole-kernel gate: a world with dissolved gas but mixing switched off, and a world with
  // mixing on but nothing dissolved, both pay one walk of a list that is normally one entry long.
  bool PayloadMixingActive() const {
    for (const PayloadProperty& p : payload_props_) {
      if (p.mix_share > 0.0f && ext_cells_.At(p.idx)->maybe_nonzero) return true;
    }
    return false;
  }

  // `fraction` of `cell`'s payload leaves the grid for `reason`
  // (ext::LiquidPayloadReleaseReason). Becomes a record; the frame publishes it.
  void PayloadRelease(size_t cell, float fraction, float temperature_k, int32_t reason) {
    if (!PayloadActive() || !(fraction > 0.0f)) return;
    for (const PayloadProperty& p : payload_props_) {
      float* s = p.data + cell * static_cast<size_t>(p.arity);
      for (int32_t c = 0; c < p.arity; ++c) {
        if (s[c] == 0.0f) continue;
        const float m = fraction >= 1.0f ? s[c] : s[c] * fraction;
        s[c] -= m;
        ext::LiquidPayloadReleased r{};
        r.cell = static_cast<int32_t>(GameIndex(cell));
        r.propertyIdx = p.idx;
        r.component = c;
        r.amount = m;
        r.temperatureK = temperature_k;
        r.reason = reason;
        payload_released_.push_back(r);
      }
    }
  }

  // A consumer's removal. `PayloadConsume` accumulates every cell's share the consumer took;
  // `PayloadEndConsume` turns the total into one record per nonzero component, keyed by the
  // consumer, exactly as the ConsumedMassInfo it rides beside is one record per consumer.
  void PayloadBeginConsume() {
    if (PayloadActive()) std::fill(payload_capture_.begin(), payload_capture_.end(), 0.0f);
  }
  void PayloadConsume(size_t cell, float fraction) {
    if (!PayloadActive() || !(fraction > 0.0f)) return;
    size_t base = 0;
    for (const PayloadProperty& p : payload_props_) {
      float* s = p.data + cell * static_cast<size_t>(p.arity);
      for (int32_t c = 0; c < p.arity; ++c) {
        if (s[c] == 0.0f) continue;
        const float m = fraction >= 1.0f ? s[c] : s[c] * fraction;
        s[c] -= m;
        payload_capture_[base + static_cast<size_t>(c)] += m;
      }
      base += static_cast<size_t>(p.arity);
    }
  }
  // `id < 0` is a consumer nobody will hear back from (a `MassConsumption` message sent with
  // no callback): the liquid is gone and so is its route, so the payload is RELEASED at
  // `fallback_cell` instead, where the removal was centred.
  void PayloadEndConsume(int32_t kind, int32_t id, float temperature_k, size_t fallback_cell) {
    if (!PayloadActive()) return;
    size_t base = 0;
    for (const PayloadProperty& p : payload_props_) {
      for (int32_t c = 0; c < p.arity; ++c) {
        const float m = payload_capture_[base + static_cast<size_t>(c)];
        if (m == 0.0f) continue;
        if (id < 0) {
          ext::LiquidPayloadReleased rel{};
          rel.cell = static_cast<int32_t>(GameIndex(fallback_cell));
          rel.propertyIdx = p.idx;
          rel.component = c;
          rel.amount = m;
          rel.temperatureK = temperature_k;
          rel.reason = ext::kPayloadReleasedRemoved;
          payload_released_.push_back(rel);
          continue;
        }
        ext::LiquidPayloadConsumed r{};
        r.kind = kind;
        r.id = id;
        r.propertyIdx = p.idx;
        r.component = c;
        r.amount = m;
        r.temperatureK = temperature_k;
        payload_consumed_.push_back(r);
      }
      base += static_cast<size_t>(p.arity);
    }
  }

  // World (re)initialisation of a region: the cell is reset, not drained, and the ledger books
  // the reset as `world init` rather than as any physical flow. The payload goes with it the
  // same way, silently -- releasing it would spawn bubbles into a region that is being wiped.
  void PayloadZero(size_t cell) {
    if (!PayloadActive()) return;
    for (const PayloadProperty& p : payload_props_) {
      float* s = p.data + cell * static_cast<size_t>(p.arity);
      for (int32_t c = 0; c < p.arity; ++c) s[c] = 0.0f;
    }
  }

  // This frame's records. The frame publish moves them into the event streams and clears them.
  std::vector<ext::LiquidPayloadReleased>& PayloadReleased() { return payload_released_; }
  std::vector<ext::LiquidPayloadConsumed>& PayloadConsumed() { return payload_consumed_; }

  int32_t DissolvedMassProperty() const { return dissolved_mass_prop_; }
  float* DissolvedMassData() { return dissolved_mass_data_; }
  // The read-only half, for the liquid property texture. `FillPropertyTextures` is handed a
  // `const World&` on purpose -- it is a renderer and must not be able to move a gram -- so it
  // cannot use the mutable accessor above.
  const float* DissolvedMassRead() const { return dissolved_mass_data_; }

  // HOW A CARRIED AMOUNT IS DRAWN (ext::kSetDissolvedTint). Not physics: this is the only piece
  // of state in `World` that exists purely so the liquid property texture can be tinted by what
  // is dissolved in a cell, and nothing but `FillPropertyTextures` reads it. It lives here
  // rather than beside the renderer because it arrives as a message and the message handlers
  // already have the world; `enabled` is false until one arrives, so a build nobody configures
  // renders exactly as it did before this existed.
  struct DissolvedTint {
    bool enabled = false;
    float full_scale_g_per_kg = ext::kDissolvedTintDefaultFullScale;
    float max_blend = ext::kDissolvedTintDefaultMaxBlend;
    float lane_weight[ext::kDissolvedGasLanes] = {};
    // Unpacked once, on receipt, so the per-cell path never touches a packed word.
    float lane_r[ext::kDissolvedGasLanes] = {};
    float lane_g[ext::kDissolvedGasLanes] = {};
    float lane_b[ext::kDissolvedGasLanes] = {};
  };

  const DissolvedTint& DissolvedTintConfig() const { return dissolved_tint_; }
  DissolvedTint& MutableDissolvedTintConfig() { return dissolved_tint_; }

  // LAYER C3: WHEN DISSOLVED GAS COMES BACK OUT (ext::kSetEffervescence, sim/effervescence.h).
  // Content data, not world state: pushed by the managed side each session, never saved, and
  // `enabled` false until it arrives, so an unconfigured world never runs the pass.
  struct Effervescence {
    bool enabled = false;
    float margin = ext::kEffervescenceDefaultMargin;
    float rate_per_second = ext::kEffervescenceDefaultRatePerSecond;
    float period_seconds = ext::kEffervescenceDefaultPeriodSeconds;
    float min_release_kg = ext::kEffervescenceDefaultMinReleaseKg;
    float lane_henry[ext::kDissolvedGasLanes] = {};
    float lane_vant_hoff_k[ext::kDissolvedGasLanes] = {};
    float lane_molar_kg[ext::kDissolvedGasLanes] = {};
    float lane_pmv_m3[ext::kDissolvedGasLanes] = {};
    int32_t solvent_count = 0;
    uint16_t solvent_element[ext::kEffervescenceMaxSolvents] = {};
    float solvent_factor[ext::kEffervescenceMaxSolvents] = {};
    float solvent_density[ext::kEffervescenceMaxSolvents] = {};
    // The surface exchange (`ext::SetEffervescenceMessageV2`). Off -- rate 0 --
    // unless the caller sent the long form of the message; `lane_element` 0xFFFF is "no gas".
    float surface_rate_per_second = 0.0f;
    float surface_min_release_kg = ext::kEffervescenceDefaultSurfaceMinReleaseKg;
    uint16_t lane_element[ext::kDissolvedGasLanes] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
                                                      0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
    // Sim seconds since the pass last ran. Not saved: a load restarts the period, which moves
    // one burst of bubbles by less than a second and nothing else.
    float elapsed_seconds = 0.0f;
    // What the pass has done, by what capped each column it looked at, since the sim was
    // initialised. `kSetEffervescence` does not touch these, so a re-send does not zero them; a
    // reader takes deltas. Read by `SIM_DebugEffervescenceCensus`, in `ext::kEffervescenceCensus*`
    // order. Never saved.
    //
    //   gas       the cell above the liquid holds gas: its own pressure is the surface's
    //   borrowed  the cell above holds nothing, but a neighbour of it holds gas: a transient gap
    //             the liquid kernel left, weighed at that neighbour's pressure
    //   vacuum    the cell above holds nothing and so do its neighbours: genuine vacuum, 0 Pa
    //   confined  a solid (or the top of the world) caps the column: never fizzes
    //
    // `columns_*` count liquid runs per pass, `fizz_*` the cells that released, `kg_*` what they
    // released; `surface*` the same three for the surface exchange.
    double census[ext::kEffervescenceCensusFields] = {};
  };
  const Effervescence& EffervescenceConfig() const { return effervescence_; }
  Effervescence& MutableEffervescenceConfig() { return effervescence_; }

  // Whether `sim.dissolved_mass` can hold anything at all -- the cheap whole-pass gate.
  bool DissolvedMaybeNonzero() const {
    if (dissolved_mass_prop_ < 0 || dissolved_mass_data_ == nullptr) return false;
    const ext::CellPropertyRegistry::Property* p = ext_cells_.At(dissolved_mass_prop_);
    return p != nullptr && p->maybe_nonzero;
  }

  // Takes `kg` of lane `lane` out of `cell`'s dissolved gas and publishes it as released for
  // `reason` -- the one-lane form of `PayloadRelease`, for a kernel that decides lane by lane.
  // Clamped to what the lane holds, so it cannot drive a lane negative; returns what it moved.
  float DissolvedRelease(size_t cell, int32_t lane, float kg, float temperature_k,
                         int32_t reason) {
    if (dissolved_mass_data_ == nullptr || lane < 0 || lane >= ext::kDissolvedGasLanes ||
        !(kg > 0.0f)) {
      return 0.0f;
    }
    float* s = dissolved_mass_data_ + cell * static_cast<size_t>(ext::kDissolvedGasLanes);
    const float m = kg < s[lane] ? kg : s[lane];
    if (!(m > 0.0f)) return 0.0f;
    s[lane] -= m;
    ext::LiquidPayloadReleased r{};
    r.cell = static_cast<int32_t>(GameIndex(cell));
    r.propertyIdx = dissolved_mass_prop_;
    r.component = lane;
    r.amount = m;
    r.temperatureK = temperature_k;
    r.reason = reason;
    payload_released_.push_back(r);
    return m;
  }
  // The reverse of `DissolvedRelease`: puts `kg` into lane `lane` of `cell`'s dissolved gas and
  // publishes it on the same stream as a NEGATIVE release for `reason` (`kPayloadAbsorbedSurface`),
  // so a reader can count what went in without a second stream. The caller has already taken
  // the mass from wherever it came from. Returns what it added.
  float DissolvedAbsorb(size_t cell, int32_t lane, float kg, float temperature_k,
                        int32_t reason) {
    if (dissolved_mass_data_ == nullptr || lane < 0 || lane >= ext::kDissolvedGasLanes ||
        !(kg > 0.0f)) {
      return 0.0f;
    }
    float* s = dissolved_mass_data_ + cell * static_cast<size_t>(ext::kDissolvedGasLanes);
    s[lane] += kg;
    ext_cells_.NoteNonzero(dissolved_mass_prop_);
    ext::LiquidPayloadReleased r{};
    r.cell = static_cast<int32_t>(GameIndex(cell));
    r.propertyIdx = dissolved_mass_prop_;
    r.component = lane;
    r.amount = -kg;
    r.temperatureK = temperature_k;
    r.reason = reason;
    payload_released_.push_back(r);
    return kg;
  }
  // Set by any kernel or message handler that can change whether IsOpenCell(gas_rooms.h)'s
  // answer for some cell just changed -- a solid boundary opening or closing. Checked once a
  // tick (StepPhysics), not polled continuously: geometry changes (digging, building,
  // natural phase transitions) are rare next to a mixing tick, so a dirty flag costs nothing
  // until something real happens, unlike an every-tick full-grid rescan would.
  bool RoomsDirty() const { return rooms_dirty_; }
  bool& MutableRoomsDirty() { return rooms_dirty_; }

  // The cells written since the last projection, in write order and with duplicates. Empty
  // with `StaticDirtyAll()` set means "all of them" — the state a fresh or freshly loaded
  // world is in, and the state the list collapses to once it is longer than the full copy
  // would be. `Project` is the only consumer and clears it; the list is bookkeeping about
  // what a *cache* still owes, not world state, which is why clearing it is const.
  const std::vector<uint32_t>& StaticDirty() const { return static_dirty_; }
  bool StaticDirtyAll() const { return static_dirty_all_; }
  // Forced by `Start`, which publishes the backwall arrays the way Klei does — untouched —
  // and then owes the next projection a full re-copy to fill them in.
  void MarkStaticDirtyAll() {
    static_dirty_all_ = true;
    static_dirty_.clear();
  }

  // The backwalls written since the last frame checked them, in write order. The backwall
  // never changes by itself — no kernel of Klei's touches it, measured — so the set of
  // cells that could newly need a transition is exactly the set something wrote, which is
  // a load (all of them) or a `ModifyBackwallData`.
  const std::vector<uint32_t>& BackwallDirty() const { return backwall_dirty_; }
  bool BackwallDirtyAll() const { return backwall_dirty_all_; }
  void ClearBackwallDirty() {
    backwall_dirty_.clear();
    backwall_dirty_all_ = false;
  }
  void ClearStaticDirty() const {
    static_dirty_.clear();
    static_dirty_all_ = false;
  }

  // ---------------------------------------- what the projection still has to look at
  //
  // `Project` publishes element, mass, temperature and solidity for every game cell, and it
  // walked all of them every frame. It does not have to: the buffers persist between frames
  // and the game re-reads the same pointers, so a cell nothing wrote is not a stale cell,
  // it is the same cell. The set it has to re-derive is the set something *could* have
  // written since the last projection.
  //
  // That set is recorded here as a rectangle per producer: each substep marks the region it
  // is about to drive, and the four message handlers that write a phase entry mark their one
  // cell. Marking is dilated by `ProjectReach` on the way in, because a kernel can write
  // past the rectangle it drives from — the pair sweeps test the far end separately, and
  // `StepGasDisplacement` moves a cell's contents two cells along. Four is a bound, not a
  // derivation: it is one more than the furthest write in the tree, and `bench --verify`
  // is what proves it, by rebuilding every cell from the world and comparing bytes.
  //
  // Kept as one x-span per row rather than as a list of rectangles, and that is the whole
  // reason this shape was chosen. `Project` emits `substanceChangeInfo` in increasing game
  // index, which is how it reproduces Klei's sort and `std::unique` without doing either.
  // A row-major walk over per-row spans is still increasing; a walk over two rectangles in
  // turn is not, and would have to sort afterwards.
  //
  // It is the tunable `ProjectReach` (tunables.def), whose minimum is this 4: it may be
  // raised, never lowered. Read here, once per rectangle, never per cell.

  void MarkProjectDirtyAll() const { project_dirty_all_ = true; }

  // One cell, undilated — a property write changes that cell's solidity and nothing else.
  void MarkProjectDirty(size_t padded) const {
    const int32_t y = static_cast<int32_t>(padded / static_cast<size_t>(width_));
    const int32_t x =
        static_cast<int32_t>(padded - static_cast<size_t>(y) * static_cast<size_t>(width_));
    MarkProjectRows(y, y, x, x);
  }

  // Padded and half-open, the way every kernel spells its own loop bounds.
  void MarkProjectDirtyRect(int32_t x0, int32_t y0, int32_t x1, int32_t y1) const {
    if (x1 <= x0 || y1 <= y0) return;
    const int32_t reach = g_tunables.project_reach;
    MarkProjectRows(y0 - reach, y1 - 1 + reach, x0 - reach, x1 - 1 + reach);
  }

  bool ProjectDirtyAll() const { return project_dirty_all_; }
  int32_t ProjectDirtyY0() const { return project_y0_; }
  int32_t ProjectDirtyY1() const { return project_y1_; }
  int32_t ProjectRowX0(int32_t y) const { return project_row_x0_[static_cast<size_t>(y)]; }
  int32_t ProjectRowX1(int32_t y) const { return project_row_x1_[static_cast<size_t>(y)]; }

  // Only the rows that were marked are reset, so a frame that moved one cell pays for one
  // row rather than for the grid.
  void ClearProjectDirty() const {
    for (int32_t y = project_y0_; y <= project_y1_; ++y) {
      project_row_x0_[static_cast<size_t>(y)] = kNoDirtyRow;
      project_row_x1_[static_cast<size_t>(y)] = -1;
    }
    project_y0_ = height_;
    project_y1_ = -1;
    project_dirty_all_ = false;
  }

  // ------------------------------------------------------------------------- the ledger
  //
  // The conservation check, the one piece of verification that survives the gas mixture
  // intact. `diffsim` compares cell against cell, and a mixed cell has no game cell to compare
  // against; a ledger
  // needs no oracle at all.
  //
  // Mass is conserved by every kernel except a short list of places that export it to the
  // game or destroy it on purpose. So the check is
  //
  //   (total now - total at load) == (what came in) - (what went out)
  //
  // and everything the list does not name lands in the driver's `drift` column. Drift is
  // the pass/fail; the buckets are only there to be subtracted.
  //
  // Buckets are per **call site**, not per cause. One "cleared" bucket that every deleting
  // site charged would balance the books by construction and prove nothing. Split, each
  // site has to say what it claims to be doing, and a bucket that moves in a scenario with
  // no business moving it is itself a finding.
  //
  // Charging is one `+=` at events that already do far more work than that, so it is always
  // on and not gated on anything. The grid total is O(cells) and is *not* — it is computed
  // only when the driver asks, through `SIM_DebugLedger`.
  struct Ledger {
    // Mass the game moved across the boundary, all four of them signed the way the driver
    // reads them: `emitted` and `modified` are additions, the rest are removals.
    double emitted = 0;      // MassEmission
    double modified = 0;     // ModifyCell, which can go either way
    double consumed = 0;     // MassConsumption
    double dug = 0;          // DigInfo — the cell's contents become an entity
    double ore = 0;          // transition ore, handed over as SpawnOreInfo
    double unstable = 0;     // a falling solid, handed to the game or dropped through
    // Mass the physics destroys on purpose. Klei's behaviour in every case, measured, and
    // the reason §2b's original two-item list is not the whole story.
    double sublimated = 0;   // taken * (1 - sublimateEfficiency), sublimation and off-gas
    double wisp = 0;         // gas under 0.001 kg, deleted where it stands
    double thin_liquid = 0;  // liquid under 0.01 kg, likewise
    double cleared = 0;      // `ClearCell` reached a cell that still held mass
    // The two components that move mass across the boundary on their own, without a
    // message per transfer. Separate buckets because they are separate call sites: an
    // emitter that puts mass in the grid and a `MassEmission` that does the same thing are
    // different code and a shared bucket would hide either one going wrong.
    double component_consumed = 0;  // ElementConsumer::Update
    double component_emitted = 0;   // ElementEmitter::Emit
    double emitter_ore = 0;         // a solid emitter, which spawns ore instead of filling a cell
    // Mass a physics mover carried from one cell to another **inside** the grid. Charged
    // only where the move's two halves land in different buckets, which is one site:
    // `DisplaceLiquidDirectional` absorbs the source into the destination whole and then
    // calls `ClearCell`, and `ClearCell` is the single authority for `cleared` -- a bucket
    // for mass that left the grid. Nothing left the grid here; it moved one cell. This is
    // the destination's gain, and it is what cancels that.
    //
    // The other fluid movers do not appear here. They subtract from the source themselves
    // rather than clearing it, so both halves are already inside the grid total and there
    // is nothing to charge. The energy ledger's `mover` is charged at all four of them
    // because energy does *not* conserve across a transfer -- the destination is credited
    // at the substep's snapshot temperature -- where the mass does.
    double mover = 0;
    // Mass a world-init message put into the grid or took out of it wholesale.
    // `SimData_ResizeAndInitializeVacuumCells` opens a world by overwriting a rectangle to
    // vacuum and walling it in with a ring of Unobtanium at a flat 9999 kg a cell -- so the
    // ring is 24 cells of mass arriving from outside the sim, and the rectangle is whatever
    // stood there leaving it. Both are real boundary crossings and neither goes through any
    // of the message channels above, which is why they need their own bucket rather than
    // `modified`: the buckets are per call site, and this is a different site.
    //
    // Signed like `modified`, because the two halves pull opposite ways.
    double world_init = 0;
    // The planetary surface boundary (abi/sim_abi_ext.h's kSetWorldEnvironment, Mod 3):
    // matter a world's own atmosphere put into a sky-exposed cell, or took back out of one.
    //
    // A REAL BOUNDARY CROSSING WITH A RECEIVER, and named so that the infinite-source decision
    // is visible rather than hidden. The planet is not depleted by what it hands over (user
    // decision; Stationeers does the same and its debit line is dead code behind a
    // hard-coded `false`) -- so from the grid's point of view this mass arrives from outside,
    // and a ledger that did not say so would be reporting conservation it does not have.
    //
    // Signed like `modified`: positive is mass entering the grid.
    double atmosphere_boundary = 0;
    // Surface uptake (sim/effervescence.h `FizzSurface`): gas a liquid's free
    // surface dissolved out of the vanilla gas cell above it, into `sim.dissolved_mass`, which
    // `TotalGridMass` does not walk. Signed like `modified`, so it only ever charges negative.
    // Gas taken from the MIXTURE layer is not charged, for the reason `consumed` gives: field 0
    // cannot see that container either.
    double dissolved_surface = 0;
  };

  // ------------------------------------------------------------------ the energy ledger
  //
  // The mass ledger above has a zero-drift guarantee; without an energy ledger, "heat is never
  // deleted" would be an assertion no instrument could contradict. This is the same idea in
  // kilojoules, and it is deliberately built the same way: buckets per **call
  // site**, drift is everything the buckets do not explain, and the check needs no oracle.
  //
  // It is built instrument-first: buckets are added by *running* it and reading which
  // scenarios drift, rather than by guessing a list of sites from a read of the kernels. A
  // guessed bucket list would balance the books by construction and prove nothing, which is
  // what the mass ledger's "per call site, not per cause" note is about.
  //
  // Energy lives in four places, and only the first is the grid: cells, building
  // temperatures, conduit contents and element chunks. `SIM_DebugEnergyLedger` publishes
  // all four separately, because the commonest transfer in the whole sim -- a building
  // exchanging with the cell under it -- moves energy between two of them and would read
  // as drift against the grid alone.
  struct EnergyLedger {
    // Electrical energy the game's power grid pushed into a building's own temperature,
    // via `operating_kilowatts` in StepBuildingHeatExchange. Vanilla's own self-heat and
    // the framework's derived power -> heat rule both arrive through this one field, so
    // this bucket is the whole of "the colony's machines heated themselves this run".
    //
    // Signed: `operating_kilowatts` may be negative (nothing forbids it, and a building
    // that cools itself would send one), so this can go down as well as up.
    double building_operating = 0;
    // What Klei's building <-> cell exchange creates or destroys outright. Not a boundary
    // crossing at all -- both sides are inside the sim -- but a genuine non-conservation
    // in the algorithm itself: the cell side and the building side are clamped
    // independently over different intervals, so what one gives up and the other receives
    // are different numbers whenever either clamp bites. Positive means the sim invented
    // energy. Charged at the site in StepBuildingHeatExchange, which spells out the
    // arithmetic.
    double building_exchange = 0;
    // Energy a building record carried in or out of the sim by being registered, replaced
    // or dropped. A building holds real thermal energy (heat capacity x temperature), so
    // registering one is an inflow in exactly the way emitted mass is, and `ModifyBuilding`
    // is a wholesale replacement that can teleport a building's heat. Charged as the actual
    // before/after difference at each site rather than as what the message asked for.
    double building_registered = 0;
    // `ModifyBuildingEnergy`: the discrete-kilojoule side door -- Klei's
    // GameComps.StructureTemperatures.ProduceEnergy, and the conduit energy sink. This is
    // the channel the aquatuner and the steam turbine account their moved heat through, so
    // it is kept apart from `building_operating` (the power grid) on purpose: one is a
    // transfer the game has already accounted elsewhere, the other is new energy.
    //
    // Charged as what actually landed, not as the message's deltaKJ: ApplyBuildingEnergy
    // clamps, and refuses outright when the bounds are out of range.
    double building_energy_msg = 0;
    // The framework's derived power -> heat rule (abi/sim_abi_ext.h's
    // kSetBuildingWasteHeatKilowatts), kept apart from `building_operating` so a run can be
    // asked how much of its heat came from the power grid rather than only how much heat
    // appeared. Zero on any world that never sends the extension message.
    double building_waste_heat = 0;
    // `ModifyCellEnergy`: Klei's `StructureTemperatureComponents.ExhaustHeat` channel, and
    // the only way a message puts heat straight into a grid cell. Charged as what actually
    // landed -- the cell's `mass * specificHeatCapacity * dT` across the temperature write
    // only, deliberately NOT across the `DoStateTransition` that follows it, because a
    // transition changes the element under the mass and its non-conservation belongs to the
    // phase-change site rather than to this one.
    double cell_energy_msg = 0;
    // What the same message asked for and did NOT get: `kilojoules` minus what landed.
    //
    // Not a container and not a boundary crossing, so it is NOT part of the conserved sum --
    // it is the deletion itself, made countable. `ExhaustHeat` scales delivery by
    // `min(Grid.Mass, 1.5) / 1.5` on the managed side before the message is even sent, and
    // then `ApplyCellEnergy` refuses outright in vacuum, under 0.001 kg, and whenever the
    // cell already sits at or above the message's `maxTemperature` (the building's own
    // overheat temperature). Every one of those paths destroys energy silently; this bucket
    // is what makes the amount visible instead of inferred.
    //
    // Signed the way the message is: a negative-exhaust cooler (Ice-Cooled Fan, Massive Heat
    // Sink) that is refused shows up negative here.
    double cell_energy_refused = 0;
    // A building's exhaust heat -- Klei's `ExhaustKilowattsWhenActive`, moved into the sim as
    // a rate by abi/sim_abi_ext.h's kSetBuildingExhaust. New energy the building's operation
    // invents, exactly like `operating_kilowatts`, so it is an inflow and belongs in the
    // conserved sum. Charged as `dt * exhaust_kilowatts` MINUS whatever had nowhere to go at
    // all (see below), so it is what actually entered the sim rather than what was asked for.
    double building_exhaust = 0;
    // Diagnostic, NOT part of the conserved sum: how much of the above could not be delivered
    // to the cells and was rerouted into the building's own body instead. This is precisely
    // the number vanilla destroys -- the mass factor, the vacuum gate, the thin-cell gate and
    // the overheat ceiling, added up -- so it is what says whether the fix is doing anything.
    double building_exhaust_bounced = 0;
    // Energy radiated out of a building body to the environment (abi/sim_abi_ext.h's
    // kSetBuildingRadiation / kSetEnvironmentTemperature). A NAMED boundary crossing with a
    // receiver, not a deletion: Stationeers hands the same energy to
    // `PlanetaryAtmosphereSimulation`, and Flagship 2's Dynamic Planet is what eventually
    // gives this bucket a real body whose temperature moves. Until then the far side is an
    // infinite reservoir at a fixed temperature and this bucket is the entire record of what
    // crossed. Signed as an OUTFLOW: positive means energy left the sim.
    double radiated_to_environment = 0;
    // The same boundary crossed INWARDS: sunlight absorbed by a building body or a grid cell
    // (abi/sim_abi_ext.h's kSetWorldEnvironment). Flagship 2 / Mod 3.
    //
    // It is an INFLOW and it belongs in the conserved sum for exactly the reason its outward
    // twin does: a star is a receiver on the far side of the boundary, not nothing, and the sim
    // has no business inventing energy without saying where it came from. A world that never
    // receives a `kSetWorldEnvironment` with a non-zero irradiance never charges this, which is
    // every world that existed before Mod 3 and every offline scenario.
    //
    // Signed as an inflow: positive means energy entered the sim. So the pair
    // `absorbed_from_environment - radiated_to_environment` is the planet's net radiative
    // budget, which is the one number a Dynamic Planet rig actually wants to watch.
    double absorbed_from_environment = 0;
    // The energy half of the planetary surface boundary (kSetWorldEnvironment, Mod 3) -- the
    // heat that arrives inside the matter the boundary supplies, plus the heat the boundary
    // moves in or out of a cell it is holding at the planet's temperature. Its mass twin is
    // `Ledger::atmosphere_boundary`, and the two are charged at the same call site from the
    // same before/after difference, so a cell whose contents the boundary changed is charged
    // once for the mass and once for the energy rather than having either inferred.
    //
    // Signed as an inflow: positive means energy entered the sim.
    double atmosphere_boundary = 0;
    // What the GROUND shed into its own sky (kSetWorldEnvironment, Mod 3) --
    // `absorbed_from_environment`'s twin outwards for a grid cell, exactly as
    // `radiated_to_environment` is for a building body.
    //
    // KIRCHHOFF IS WHY THIS EXISTS AND WHY IT NEEDS NO SECOND COEFFICIENT. A grey body absorbs
    // and emits with the same number, so the world's `solar_absorptivity` is also the
    // emissivity of every cell it lets the sun reach. Without this bucket the solar term is a
    // one-way ratchet: a sunlit tile gains about 0.05 K per cycle and never gives any of it
    // back, so a planet's crust cooks over a few hundred cycles with nothing in the books to
    // say why. With it, a surface temperature is the balance of two flows rather than a number
    // somebody chose, which is the whole claim Mod 3 makes about a day.
    //
    // KEPT SEPARATE FROM `radiated_to_environment` ON PURPOSE. The framework's finite-reservoir
    // policy (`OniFramework.PlanetaryEnvironment.ReservoirHeatCapacityKJPerK`) warms the planet
    // by what the COLONY rejects into it, and the planet's own crust radiating its own daylight
    // back at its own sky is not that. One bucket for both would have a bare, unvisited Mars
    // heating itself in proportion to how much of it the player had uncovered.
    //
    // Signed as an OUTFLOW: positive means energy left the sim.
    double cell_radiated_to_environment = 0;
    // Surface uptake: the grid's own change where a liquid surface dissolved gas
    // from the cell above -- the vanilla gas cell's `m c_gas T_gas` leaving, plus the heat that
    // gas carried above the liquid's temperature landing in the liquid. What stays behind in
    // `sim.dissolved_mass` is the gas at the liquid's temperature, which no container here
    // walks. Signed like `cell_modified` (the grid's view); mixture-layer gas charges only the
    // liquid's heat, since its `m c T` was never in field 0.
    double dissolved_surface = 0;
    // `MassConsumption` and the native element consumers (emitters.h's RemoveMassFromCell):
    // matter, and the heat in it, taken out of a grid cell and handed to a building. A real
    // boundary crossing with a receiver on the other side, not a deletion -- the callback
    // carries the mass and the temperature back to the game, which is what a Gas Pump's
    // contents are. Named to match the mass ledger's `consumed` so the two divide.
    //
    // Only the vanilla `PhaseEntry` half is charged. The promoted-room branch takes its mass
    // out of the gas-mixture layer, which `TotalGridEnergy` does not walk, so charging it
    // would invent an outflow from a container field 0 cannot see.
    double consumed = 0;
    // `MassEmission` and the framework's ConvertToVanillaMass: the same crossing inwards.
    // Charged as the cell's real before/after change rather than as the message's own
    // `mass x temperature`, because `CalculateCombinedTemperature` clamps the mix and the
    // element under the mass can change in the same breath.
    double emitted = 0;
    // Natural phase change -- `TransitionCell`, the single choke point every transition in
    // the sim goes through, the per-tick sweep and `BuildingHeatExchange`'s direct per-cell
    // call alike.
    //
    // This is the largest non-conservation in the whole sim and it is Klei's design, not an
    // accounting gap: **temperature carries across a transition and specific heat does
    // not.** 1000 kg of water at 200 K becomes ice at 201.5 K, and ice's specific heat is
    // 2.05 against water's 4.179, so the cell's internal energy nearly halves in one step.
    // Conserving energy instead would have put that cell at 407 K. The 1.5 K overshoot is
    // the whole of vanilla's latent heat.
    //
    // Naming it is the point. `sim/phase_change.h` is the framework's real latent-heat
    // model, and until this bucket existed there was no measurement saying how much energy
    // the vanilla rule invents or destroys for it to be compared against.
    double phase_change = 0;
    // The transition ore a few elements drop: mass handed to the game at the post-transition
    // temperature. Kept apart from `phase_change` because it is an ordinary boundary
    // crossing with a receiver -- the game gets a real lump -- and lumping it in would make
    // the number above stop meaning "what the temperature-carrying rule costs".
    double transition_ore = 0;
    // Sublimation and liquid off-gassing (physics.h's `DoSublimation` and its off-gas twin).
    // Two things happen at once here and neither conserves energy: only
    // `sublimateEfficiency` of what leaves the solid arrives as gas, and the product is
    // written at the SOLID's temperature with no mixing at all -- unlike every other
    // delivery in the sim, which mixes. So a hot rock off-gassing into a cold pocket hands
    // that pocket its own temperature outright.
    //
    // Charged in two pieces per transfer, the source's and the destination's, so the
    // `DisplaceGas` that may sit between them keeps its own bucket rather than being folded
    // into this one.
    double sublimated = 0;
    // `SimData_ResizeAndInitializeVacuumCells`: the game opening a rectangle of the world.
    // Both halves are wholesale overwrites -- an Unobtanium ring at 9999 kg and 0 K, and a
    // vacuum interior at nothing -- so whatever those cells were holding leaves the sim with
    // no receiver at all. Charged as each cell's real before/after change, which is very
    // nearly "minus everything that was there": Unobtanium's specificHeatCapacity is 0 and
    // vacuum has no mass, so both writes land on exactly zero energy.
    //
    // Not a leak to be fixed. This is world generation, and the matter it overwrites never
    // belonged to a running colony.
    double world_init = 0;
    // The fluid movers -- `StepFlow`'s liquid sweep above all -- and the one thing about them
    // that does not conserve energy: a transfer debits the SOURCE at the temperature it holds
    // now and credits the DESTINATION at the temperature the source held when the substep
    // began (`start[src].temperature`). A cell that has already given a slice away this
    // substep has moved, so the two ends of the same transfer disagree about how much heat
    // travelled with the mass.
    //
    // Klei's, and load-bearing rather than incidental: the snapshot is what makes a sweep's
    // result independent of the order the cells are visited in. Charged as the pair's real
    // before/after change so the amount is countable instead of appearing as drift.
    double mover = 0;
    // The native ElementEmitter component (emitters.h's EmitInto), kept apart from `emitted`
    // the same way the mass ledger keeps `component_emitted` apart from `emitted`: one is a
    // building sending a message, the other is a component the sim ticks itself. Charged as
    // the cell's real before/after change, because the element is overwritten and the
    // temperature mix is clamped.
    //
    // The solid arm has no charge and needs none: it hands the game a lump of ore and never
    // touches the grid, so that energy never entered the container field 0 walks.
    double component_emitted = 0;
    // The four places matter -- and the energy in it -- leaves the grid without going
    // anywhere, one per site and named the same way the mass ledger names them, so the two
    // can be divided into each other: `cleared / cleared` is the mean
    // `specificHeatCapacity x temperature` of everything that was deleted at that site.
    //
    // All four are OUTFLOWS with no receiver at all -- unlike `radiated_to_environment`,
    // which crosses a named boundary -- so they are exactly the heat deletion this
    // sim sets out to remove, made countable. They are signed as the containers see them
    // (negative: energy left), which is why they are ADDED to the conserved sum rather than
    // subtracted.
    //
    // `cleared` is charged inside `ClearCell` itself, so it is the single authority for
    // every caller that reaches it; the guards in cellmod.h and the displacement pair below
    // are all measured to stop short of it rather than to double-count it.
    double cleared = 0;
    // A gas cell under 1 mg, zeroed by `ZeroMasslessCells` -- but only when a neighbour
    // holding at least 1 kg of gas is there to be merged into, which is the gate that makes
    // this a rounding cleanup rather than a leak.
    double wisp = 0;
    // A liquid cell under 0.01 kg, zeroed by `Evaporate`. The pool that finished draining.
    double thin_liquid = 0;
    // A solid cell handed to the game as a falling entity -- and charged when it is NOT
    // handed over too, because either way the grid stops holding it.
    double unstable = 0;
    // `StepConduction`'s [1 K, 10000 K] clamp (physics.h). Klei's, and a real energy source
    // and sink rather than an accounting gap: a cell that a message or a kernel left below
    // 1 K is pushed back up to 1 K with nothing paying for the difference, and a cell above
    // 10000 K is pushed back down with the excess destroyed. The clamp is what `modifycell`'s
    // sub-kelvin probes read back as 1 K on the next tick.
    //
    // Positive means the sim invented energy, which is the usual direction: the cold end of
    // this clamp is reached far more often than the hot one.
    double conduction_clamp = 0;
    // `DoDisplacement` (physics.h), the gas-displacement primitive shared by the liquid
    // mover, post-process and the message drain. It does NOT conserve energy, and for two
    // reasons that are both Klei's: `AddMassAndUpdateTemperature` mixes the pair's
    // temperatures weighted by MASS rather than by heat capacity, and the destination's
    // element is overwritten with the source's AFTER that mix -- so whenever the two cells
    // hold different elements the specific heat capacity under the merged mass changes and
    // nothing pays for it.
    //
    // Kept out of `cell_modified` deliberately even though `ModifyCell` reaches it: three
    // unrelated callers do, and a bucket that lumped them together could not say which one
    // moved.
    double displaced = 0;
    // `ModifyCell` and everything it dispatches into (cellmod.h): the game writing matter
    // straight into a grid cell, which carries that matter's thermal energy across the
    // boundary with it. The exact counterpart of the mass ledger's `modified`, and charged at
    // the same call sites, so the two can be read against each other.
    //
    // This bucket is NOT only "mass times heat capacity times temperature of what arrived".
    // Several of these paths rewrite a cell's ELEMENT without touching its mass -- a
    // displaced liquid neighbour taking the incoming element, `AddIntoBlockedCell` blanking a
    // blocker, `AddSolid`'s overwrite -- and the specific heat capacity changes underneath
    // the same kilograms. That is a genuine energy change with no mass change at all, and
    // measuring the cell's stored energy before and after (rather than the message's own
    // numbers) is what makes it countable.
    double cell_modified = 0;
    // Energy an element-chunk record carried in or out of the sim by being registered,
    // replaced or dropped. Exactly the building_registered story one container over: a chunk
    // is `heat_capacity x temperature` of real thermal energy that the game hands the sim and
    // takes back again, and `SetElementChunkData` is a wholesale replacement of BOTH those
    // fields, so it can teleport a lump's heat the way ModifyBuilding teleports a building's.
    //
    // This is the bucket that dominates the chunk scenarios: before it existed every one of
    // them drifted by very close to its own total chunk energy, arriving at tick 1 -- the same
    // registration signature the building drift turned out to have.
    double chunk_registered = 0;
    // `ModifyElementChunkEnergy`: the discrete-kilojoule side door for chunks, the counterpart
    // of building_energy_msg. Charged as what actually LANDED rather than as the message's
    // deltaKJ, because ElementChunk::ModifyEnergy is not a straight add: it floors the
    // resulting temperature at zero (so a large negative delta is silently truncated, which
    // CREATES energy relative to what was asked) and it drops the message entirely for a chunk
    // with no heat capacity. Reading the record back out is the only way to see either.
    double chunk_energy_msg = 0;
    // What the same message asked for and did NOT get, the counterpart of
    // cell_energy_refused and NOT part of the conserved sum for the same reason: it is the
    // discrepancy itself, made countable, and including it would make the discrepancy cancel
    // out and the ledger would report conservation exactly where there is none.
    //
    // Two paths, and they are opposite in sign. ElementChunk::ModifyEnergy floors the
    // resulting temperature at 0 K without ceilinging it, so a withdrawal larger than the
    // chunk holds is truncated -- the game asks for a removal it does not get, which RETAINS
    // energy rather than deleting it. And a chunk with no heat capacity swallows the message
    // whole. `diffsim --scenario chunkenergy` exercises the first deliberately: a -1e9 kJ
    // withdrawal against a 10 kg lump of water moves ~1.8e4 kJ and discards the rest.
    double chunk_energy_refused = 0;
    // The chunk temperature adjuster (`ModifyChunkTemperatureAdjuster`), charged where it
    // acts, in StepElementChunks. This one is a genuine boundary crossing rather than an
    // accounting gap: the adjuster is a SOURCE, not a body -- it drives the chunk through the
    // same two-body solver every substep and then throws its own new temperature away, so
    // whatever the chunk gained came from nowhere inside the sim. That is what a Thermo
    // Regulator's contents or a duplicant's body heat looks like from in here.
    //
    // Signed: an adjuster colder than the chunk is a sink and charges negative.
    double chunk_adjuster = 0;
    // What the chunk <-> cell exchange creates or destroys outright, the counterpart of
    // building_exchange and non-conservation in Klei's own algorithm rather than a bug here.
    // CalculateTemperatureExchangePrecise clamps each side to the pair's equilibrium
    // INDEPENDENTLY (`if (equilibrium <= next_a) next_a = equilibrium;` against
    // `if (next_b <= equilibrium) next_b = equilibrium;`), so whenever either clamp bites the
    // energy one body gives up and the other receives are different numbers.
    //
    // Charged as (cell after + chunk after) - (cell before + chunk before) across both the
    // cell call and the ground-tile call, so positive means the sim invented energy.
    double chunk_exchange = 0;
    // Not an energy: the sim's own accumulated substep time, in seconds, charged once per
    // substep and NOT once per region. Added because the first live run of the
    // power -> heat rule charged 1.70x what the managed side pushed, the rate was proven
    // identical on both sides, and that left the elapsed time as the only remaining
    // suspect -- with no instrument able to say whether the sim and the game agreed about
    // how much time had passed. Now there is one.
    double sim_seconds = 0;
  };

  void NoteBuildingOperating(double kilojoules) const {
    energy_ledger_.building_operating += kilojoules;
  }
  void NoteBuildingExchange(double kilojoules) const {
    energy_ledger_.building_exchange += kilojoules;
  }
  void NoteBuildingRegistered(double kilojoules) const {
    energy_ledger_.building_registered += kilojoules;
  }
  void NoteBuildingEnergyMsg(double kilojoules) const {
    energy_ledger_.building_energy_msg += kilojoules;
  }
  void NoteBuildingWasteHeat(double kilojoules) const {
    energy_ledger_.building_waste_heat += kilojoules;
  }
  void NoteCellEnergyMsg(double kilojoules) const {
    energy_ledger_.cell_energy_msg += kilojoules;
  }
  void NoteCellEnergyRefused(double kilojoules) const {
    energy_ledger_.cell_energy_refused += kilojoules;
  }
  // The per-cell remainder behind the `CellEnergyCarry` tunable (tunables.def). Allocated on the
  // first carrying payment rather than with the world, so a world that never turns the row on
  // holds nothing. Not saved: what a save drops is at most one float temperature step's worth
  // of energy per cell that received a payment.
  double* CellEnergyCarry(size_t cell) {
    if (cell_energy_carry_.size() != PaddedCount()) cell_energy_carry_.assign(PaddedCount(), 0.0);
    return &cell_energy_carry_[cell];
  }
  // Energy the carries hold: paid by a message, not yet in any cell's temperature. A stock the
  // ledger publishes beside the grid's own, so grid + held = what the messages delivered.
  void NoteCellEnergyCarry(double delta_kj) { cell_energy_carry_held_ += delta_kj; }
  double CellEnergyCarryHeld() const { return cell_energy_carry_held_; }
  void NoteBuildingExhaust(double kilojoules) const {
    energy_ledger_.building_exhaust += kilojoules;
  }
  void NoteBuildingExhaustBounced(double kilojoules) const {
    energy_ledger_.building_exhaust_bounced += kilojoules;
  }
  void NoteRadiatedToEnvironment(double kilojoules) const {
    energy_ledger_.radiated_to_environment += kilojoules;
  }
  void NoteAbsorbedFromEnvironment(double kilojoules) const {
    energy_ledger_.absorbed_from_environment += kilojoules;
  }
  void NoteAtmosphereBoundaryEnergy(double kilojoules) const {
    energy_ledger_.atmosphere_boundary += kilojoules;
  }
  void NoteCellRadiatedToEnvironment(double kilojoules) const {
    energy_ledger_.cell_radiated_to_environment += kilojoules;
  }
  void NotePhaseChange(double kilojoules) const {
    energy_ledger_.phase_change += kilojoules;
  }
  void NoteTransitionOre(double kilojoules) const {
    energy_ledger_.transition_ore += kilojoules;
  }
  void NoteSublimatedEnergy(double kilojoules) const {
    energy_ledger_.sublimated += kilojoules;
  }
  void NoteWorldInitEnergy(double kilojoules) const {
    energy_ledger_.world_init += kilojoules;
  }
  void NoteMoverEnergy(double kilojoules) const { energy_ledger_.mover += kilojoules; }
  void NoteComponentEmittedEnergy(double kilojoules) const {
    energy_ledger_.component_emitted += kilojoules;
  }
  void NoteConsumedEnergy(double kilojoules) const { energy_ledger_.consumed += kilojoules; }
  void NoteEmittedEnergy(double kilojoules) const { energy_ledger_.emitted += kilojoules; }
  void NoteClearedEnergy(double kilojoules) const { energy_ledger_.cleared += kilojoules; }
  void NoteWispEnergy(double kilojoules) const { energy_ledger_.wisp += kilojoules; }
  void NoteThinLiquidEnergy(double kilojoules) const {
    energy_ledger_.thin_liquid += kilojoules;
  }
  void NoteUnstableEnergy(double kilojoules) const { energy_ledger_.unstable += kilojoules; }
  void NoteConductionClamp(double kilojoules) const {
    energy_ledger_.conduction_clamp += kilojoules;
  }
  void NoteDisplaced(double kilojoules) const { energy_ledger_.displaced += kilojoules; }
  void NoteCellModified(double kilojoules) const {
    energy_ledger_.cell_modified += kilojoules;
  }
  void NoteChunkRegistered(double kilojoules) const {
    energy_ledger_.chunk_registered += kilojoules;
  }
  void NoteChunkEnergyMsg(double kilojoules) const {
    energy_ledger_.chunk_energy_msg += kilojoules;
  }
  void NoteChunkEnergyRefused(double kilojoules) const {
    energy_ledger_.chunk_energy_refused += kilojoules;
  }
  void NoteChunkAdjuster(double kilojoules) const {
    energy_ledger_.chunk_adjuster += kilojoules;
  }
  void NoteChunkExchange(double kilojoules) const {
    energy_ledger_.chunk_exchange += kilojoules;
  }
  void NoteSimSeconds(double seconds) const { energy_ledger_.sim_seconds += seconds; }
  void NoteDissolvedSurfaceEnergy(double kilojoules) const {
    energy_ledger_.dissolved_surface += kilojoules;
  }
  const EnergyLedger& EnergyBooks() const { return energy_ledger_; }

  // ---------------------------------------------------- whether the ledger costs anything
  //
  // The buckets divide into two kinds, and only one of them is worth switching off.
  //
  // Most `Note*Energy` sites already hold the number they are charging -- the power grid's
  // own kilowatts, the exhaust the message carried, the flux radiation just computed -- and
  // cost one `+=`. Those stay on always; a branch would cost more than the addition.
  //
  // The rest have to FIND the number, by walking a cell's stored energy before and after the
  // thing they are charging. `MoverCharge` (physics.h) is the worst of them: it is
  // constructed on every attempted fluid move, and the gas-pressure sweep attempts a great
  // many. Measured, it cost 0.63 ms of a 2.32 ms `StepGasPressure` and 7.6 % of
  // the whole frame -- a debug instrument that was, until this flag, on in every build
  // including a shipped one.
  //
  // So this gates the walks and nothing else. A shipped DLL takes one predictable branch per
  // move instead of four to six energy walks; `diffsim --energy-ledger` and `vftest` turn it
  // on and get exactly the books they had before, to the bit.
  //
  // IT MUST BE SET BEFORE THE FIRST TICK. A ledger switched on halfway through has counted
  // half a run's transfers against a whole run's drift, which reads as a conservation bug in
  // the sim rather than as the measurement error it is. Nothing here can detect that, so the
  // callers do it at load.
  bool EnergyLedgerEnabled() const { return energy_ledger_enabled_; }
  void SetEnergyLedgerEnabled(bool enabled) { energy_ledger_enabled_ = enabled; }

  void NoteEmitted(float kg) const { ledger_.emitted += kg; }
  void NoteModified(float kg) const { ledger_.modified += kg; }
  void NoteAtmosphereBoundary(double kg) const { ledger_.atmosphere_boundary += kg; }
  void NoteDissolvedSurface(double kg) const { ledger_.dissolved_surface += kg; }
  void NoteConsumed(float kg) const { ledger_.consumed += kg; }
  void NoteDug(float kg) const { ledger_.dug += kg; }
  void NoteOre(float kg) const { ledger_.ore += kg; }
  void NoteUnstable(float kg) const { ledger_.unstable += kg; }
  void NoteSublimated(float kg) const { ledger_.sublimated += kg; }
  void NoteWisp(float kg) const { ledger_.wisp += kg; }
  void NoteThinLiquid(float kg) const { ledger_.thin_liquid += kg; }
  void NoteCleared(float kg) const { ledger_.cleared += kg; }
  void NoteComponentConsumed(float kg) const { ledger_.component_consumed += kg; }
  void NoteComponentEmitted(float kg) const { ledger_.component_emitted += kg; }
  void NoteEmitterOre(float kg) const { ledger_.emitter_ore += kg; }
  void NoteMover(float kg) const { ledger_.mover += kg; }
  void NoteWorldInit(float kg) const { ledger_.world_init += kg; }
  const Ledger& Books() const { return ledger_; }

  // Game cells only, which is what the game's own total would be. A mover that wrote into
  // the border ring would take mass out of this sum and show up as drift — which is the
  // right answer, because that mass is gone as far as the game is concerned.
  //
  // Summed in double over floats: the grid is 400k cells and a naive float accumulator
  // loses the low bits of a 1 kg cell long before the last row.
  double TotalGridMass() const {
    double total = 0.0;
    for (const CellWalk cw : GameCells()) total += phases_[cw.padded].mass;
    return total;
  }

  // The energy counterpart of `TotalGridMass`. This is the same construction as `TotalGridMass`, one
  // level more expensive: a cell's energy is `mass * specificHeatCapacity * temperature`,
  // so it needs the element table that mass did not.
  //
  // Kilojoules -- ONI's `specificHeatCapacity` is kJ/(kg K), so this is kJ and so is every
  // other energy figure in the ledger. The numbers are still large (a full world runs to
  // ~1e8 kJ), which is why the accumulator is double and why the drift that matters is a
  // *relative* one: a float accumulator would lose a whole cell's worth of energy in the
  // low bits before the first row finished.
  //
  // Cells only. Buildings, conduits and element chunks hold energy of their own outside
  // the grid; `SIM_DebugEnergyLedger` sums those separately so that a transfer between a
  // building and the cell under it -- which is conserving, and very common -- does not
  // read as drift.
  double TotalGridEnergy(const ElementTable& table) const {
    double total = 0.0;
    for (const CellWalk cw : GameCells()) {
      const PhaseEntry& c = phases_[cw.padded];
      if (c.mass <= 0.0f) continue;
      total += static_cast<double>(c.mass) *
               static_cast<double>(table.At(c.element).specificHeatCapacity) *
               static_cast<double>(c.temperature);
    }
    return total;
  }

  const std::vector<float>& Radiation() const { return radiation_; }
  const std::vector<SaveDisease>& Disease() const { return disease_; }
  const std::vector<SaveBackwall>& Backwall() const { return backwall_; }
  const std::vector<uint8_t>& Properties() const { return properties_; }

  // The random stream and the shuffle's stride. `SetRandomState` is what makes a gas world
  // reproducible: the game stores the seed argument as the stream state raw, with no
  // scramble, so the world seed *is* the first state.
  uint32_t RandomState() const { return rng_; }
  void SetRandomState(uint32_t s) { rng_ = s; }
  uint32_t NextRandomState() {
    rng_ = rng_ * 214013u + 2531011u;
    return rng_;
  }
  int32_t ShuffleDir() const { return shuffle_dir_; }
  void SetShuffleDir(int32_t d) { shuffle_dir_ = d; }
  const std::vector<uint8_t>& Insulation() const { return insulation_; }
  const std::vector<uint8_t>& Strength() const { return strength_; }
  // The conduction kernel's hoist, and the whole of what `thermal_mass_bonus_` was. Extra
  // heat capacity in J/K, folded into a cell's conduction math alongside its element
  // mass*specificHeat. Default 0 on every cell, so a world that never sends
  // kSetCellThermalMassBonus computes byte-identical to vanilla — kept by a registered
  // property.
  //
  // A CACHED RAW POINTER, refreshed by `Allocate` and by nothing else. `ExtCells().F32()` is
  // the general lookup and would do here — it is hoisted out of the pair loop, so it is paid
  // once per kernel invocation — but the conduction kernel is 10 % of the frame and this
  // costs one member load instead of a bounds check, a type check and two vector
  // indirections. Measured on the granite bench, 50 ticks, median of three runs.
  //
  // WHY IT CANNOT DANGLE. `ext::CellPropertyRegistry::Allocate` is the only thing that sizes
  // extension storage, `World::Allocate` is its only caller, and it refreshes this pointer on
  // the very next line. Registration closes at that same moment, so no later `Register` can
  // reallocate the buffer behind it. Any future code that resizes extension storage outside
  // `Allocate` has to refresh this too — which is why the two lines sit together.
  const float* ThermalMassBonusData() const { return thermal_mass_bonus_data_; }
  int32_t ThermalMassBonusProperty() const { return thermal_mass_bonus_prop_; }
  // Framework extension (not from Klei — see abi/sim_abi_ext.h, kSetInvertedGravityElement).
  // One element index whose liquid turn in StepFlow (physics.h) runs with its vertical sign
  // flipped -- the element falls up and pools under ceilings instead of on floors. Sentinel
  // 0xFFFF ("no element") is the default and matches every existing ElementTable slot never
  // being a valid index that collides with it, so a world that never sends the message
  // computes byte-identical to vanilla -- same contract as ThermalMassBonus above.
  uint16_t InvertedGravityElement() const { return inverted_gravity_element_; }
  void SetInvertedGravityElement(uint16_t e) { inverted_gravity_element_ = e; }

  // Framework extension (not from Klei — see abi/sim_abi_ext.h,
  // kSetEnvironmentTemperature). The temperature of the reservoir a radiating building body
  // sheds into, and the sim's stand-in for Stationeers'
  // `PlanetaryAtmosphereSimulation`. 0 until something sets it, which together with a
  // radiation factor that is also 0 by default means radiation is inert on any world that
  // never asks for it.
  //
  // Stationeers' own fallback when its planetary atmosphere is effectively absent is 50 K
  // (`CalculateEntropy`: `global.T < 1 ? 50 : global.T`); choosing the number for ONI is a
  // policy decision and belongs to the API rather than to a native default.
  float EnvironmentTemperature() const { return environment_temperature_; }
  void SetEnvironmentTemperature(float k) { environment_temperature_ = k; }

  // Framework extension (not from Klei -- see abi/sim_abi_ext.h, kSetWorldEnvironment). The
  // planet a world IS, for the three terms that need one: the radiative sink a building sheds
  // into, the sunlight it absorbs, and the atmosphere its surface is held at. Flagship 2 / Mod 3.
  //
  // Every field is zero by default and every term is inert at zero, so a world that receives no
  // record behaves exactly as it did before this existed -- the same contract
  // `ThermalMassBonus` and `InvertedGravityElement` make, and the reason the whole offline suite
  // is unaffected.
  struct Environment {
    float sink_kelvin = 0.0f;             // <= 0 falls back to EnvironmentTemperature()
    float peak_irradiance_w_m2 = 0.0f;    // solar flux at full sun
    float full_sun_lux = 0.0f;            // the lux this world reports at full sun
    float solar_absorptivity = 0.0f;      // 0..1 of the incident flux a bare cell takes
    float surface_pressure_kpa = 0.0f;    // the boundary's target pressure
    float boundary_temperature_k = 0.0f;  // <= 0 falls back to the sink
    float boundary_rate = 0.0f;           // 0..1 of the gap closed per substep
    int32_t boundary_element = -1;        // < 0 = no boundary
  };

  // `world` indexes `DefineWorldOffsets`' list. A negative index sets the fallback record, which
  // is what a single-asteroid game and every offline scenario use -- neither knows an index, and
  // requiring one would make the common case the awkward one.
  void SetWorldEnvironment(int32_t world, const Environment& e) {
    if (world < 0) {
      default_environment_ = e;
      return;
    }
    if (static_cast<size_t>(world) >= world_environments_.size()) {
      world_environments_.resize(static_cast<size_t>(world) + 1);
    }
    world_environments_[static_cast<size_t>(world)] = e;
    world_environment_set_.resize(world_environments_.size(), 0);
    world_environment_set_[static_cast<size_t>(world)] = 1;
  }

  // A world with no record of its own gets the fallback, NOT a zeroed record. That distinction is
  // the whole reason `world_environment_set_` exists: resizing the vector to reach index 3 would
  // otherwise silently give worlds 0..2 an all-zero environment and switch off terms a caller had
  // configured globally.
  const Environment& EnvironmentOfWorld(int32_t world) const {
    if (world >= 0 && static_cast<size_t>(world) < world_environments_.size() &&
        world_environment_set_[static_cast<size_t>(world)] != 0) {
      return world_environments_[static_cast<size_t>(world)];
    }
    return default_environment_;
  }

  // THE SUN A WORLD IS UNDER, as a direction. Mod 3's sun path.
  //
  // A unit vector from a cell TOWARD the sun, projected into the grid plane: +x is east, which
  // is the grid's x-max side, and +y is up. `dir_y <= 0` is a sun at
  // or below the horizon, and nothing is lit. The out-of-plane component is not carried because
  // nothing in a two-dimensional grid can cast a shadow along it; how much it dims a flat surface
  // is already inside the lux the game sends every frame (`sin(elevation)` times the world's
  // sunlight trait), so all the sim needs from the sun is where the beam comes FROM.
  //
  // Kept apart from `Environment` on purpose. The sky view `ComputeSunlight` publishes and the
  // direct beam are different questions -- a cell under an overhang is open to the sky and in the
  // shade -- and a world that never receives a sun record keeps the vertical texture for every
  // term, byte for byte. Same fallback contract as the environment: a negative index sets the
  // record every world without its own falls back to.
  struct SunDirection {
    float dir_x = 0.0f;
    float dir_y = 0.0f;
  };

  void SetWorldSun(int32_t world, const SunDirection& s) {
    if (world < 0) {
      default_sun_ = s;
      default_sun_set_ = true;
      return;
    }
    if (static_cast<size_t>(world) >= world_suns_.size()) {
      world_suns_.resize(static_cast<size_t>(world) + 1);
      world_sun_set_.resize(world_suns_.size(), 0);
    }
    world_suns_[static_cast<size_t>(world)] = s;
    world_sun_set_[static_cast<size_t>(world)] = 1;
  }

  // Forgets a world's sun, so its beam goes back to being nobody's business. A negative index
  // forgets the fallback.
  void ClearWorldSun(int32_t world) {
    if (world < 0) {
      default_sun_ = SunDirection{};
      default_sun_set_ = false;
      return;
    }
    if (static_cast<size_t>(world) < world_sun_set_.size()) {
      world_sun_set_[static_cast<size_t>(world)] = 0;
    }
  }

  // False when the world has no sun of its own and there is no fallback: every beam consumer then
  // reads the sky view instead, which is what keeps a world without a record unchanged.
  bool SunOfWorld(int32_t world, SunDirection* out) const {
    if (world >= 0 && static_cast<size_t>(world) < world_sun_set_.size() &&
        world_sun_set_[static_cast<size_t>(world)] != 0) {
      *out = world_suns_[static_cast<size_t>(world)];
      return true;
    }
    if (default_sun_set_) {
      *out = default_sun_;
      return true;
    }
    return false;
  }

  bool AnyWorldSun() const {
    if (default_sun_set_) return true;
    for (uint8_t set : world_sun_set_) {
      if (set != 0) return true;
    }
    return false;
  }

  // The sink a building on `world` radiates into: its own if it has one, else the one global
  // scalar `kSetEnvironmentTemperature` sets. Keeping the global as the fallback is what leaves
  // every existing caller and every existing vftest arm unchanged.
  float SinkTemperatureForWorld(int32_t world) const {
    const Environment& e = EnvironmentOfWorld(world);
    return e.sink_kelvin > 0.0f ? e.sink_kelvin : environment_temperature_;
  }

  // ------------------------------------------------- the planetary latent accumulator
  //
  // A Stationeers planet keeps one signed planet-wide store of the energy its own atmosphere's
  // phase changes have released or absorbed, and reads it back as a temperature that is summed
  // into ambient alongside the solar, greenhouse, density and weather terms. That is the
  // thermostat: condensing
  // warms the planet, which stops the condensing.
  //
  // THIS HOLDS THE ENERGY AND NOTHING ELSE. It is not divided by anything here and it changes
  // no cell. Which heat capacity it is read against, and whether it is read at all, is the
  // framework's `PlanetaryEnvironment` policy -- the same split the sibling accumulator
  // already uses, where the sim charges `radiated_to_environment` at the site the energy left
  // a body and `ReservoirHeatCapacityKJPerK` decides what that means for a planet. A balance
  // constant does not belong in a kernel.
  //
  // JOULES, AND SIGNED: positive is energy the planet's atmosphere GAINED (gas condensing to
  // liquid), negative is energy it gave up (liquid boiling back to gas). `StepStateChange` is
  // the only writer -- see its own comment for which transitions count and why the
  // building-driven ones do not.
  //
  // CUMULATIVE SINCE THE WORLD WAS ALLOCATED, never reset by a read, exactly like the two
  // ledgers: a reader samples, waits and subtracts, and two readers cannot destroy each
  // other's window. Nothing serialises it, so it starts a load at zero -- the same contract
  // `Environment` itself makes, where a mod re-sends its planets after a load.
  void NoteWorldLatentEnergy(int32_t world, double joules) {
    if (world < 0 || joules == 0.0) return;
    if (static_cast<size_t>(world) >= world_latent_energy_j_.size()) {
      world_latent_energy_j_.resize(static_cast<size_t>(world) + 1, 0.0);
    }
    world_latent_energy_j_[static_cast<size_t>(world)] += joules;
  }
  // 0 for a world that has never had a transition counted, which is every world in every
  // offline scenario and every world with no environment record.
  double LatentEnergyOfWorld(int32_t world) const {
    if (world < 0 || static_cast<size_t>(world) >= world_latent_energy_j_.size()) return 0.0;
    return world_latent_energy_j_[static_cast<size_t>(world)];
  }

  // Which world a region belongs to, or -1.
  //
  // `Game.UnsafeSim200ms` builds one active region per DISCOVERED `WorldContainer`, and its
  // rectangle is exactly that world's `WorldOffset`..`WorldOffset + WorldSize`. So the match is
  // an equality on the rectangle, not a containment test -- and it is deliberately not the
  // region's own index, because an undiscovered world contributes an offset and no region, which
  // makes the two lists disagree from that point on. A region that matches nothing (an offline
  // scenario that sends regions but no world offsets, which is most of the suite) reports -1 and
  // gets the fallback record.
  int32_t WorldIndexOfRegion(size_t ri) const {
    if (world_offsets_.empty() || regions_.empty() || ri >= regions_.size()) return -1;
    const Rect& r = regions_[ri];
    for (size_t w = 0; w < world_offsets_.size(); ++w) {
      const WorldOffset& o = world_offsets_[w];
      if (o.x == r.min_x && o.y == r.min_y && o.x + o.w == r.max_x && o.y + o.h == r.max_y) {
        return static_cast<int32_t>(w);
      }
    }
    // Not an exact rectangle. Fall back to "which world contains this region's lower-left
    // corner", so a game that ever clips a region to something smaller than its world still
    // resolves rather than silently losing its planet.
    for (size_t w = 0; w < world_offsets_.size(); ++w) {
      const WorldOffset& o = world_offsets_[w];
      if (r.min_x >= o.x && r.min_y >= o.y && r.min_x < o.x + o.w && r.min_y < o.y + o.h) {
        return static_cast<int32_t>(w);
      }
    }
    return -1;
  }

  // Which world a GAME cell belongs to, or -1. The rectangle test the sunlight sweep already
  // does, exposed for the building terms, which know a cell and not a region.
  int32_t WorldIndexOfGameCell(int32_t gx, int32_t gy) const {
    for (size_t w = 0; w < world_offsets_.size(); ++w) {
      const WorldOffset& o = world_offsets_[w];
      if (gx >= o.x && gy >= o.y && gx < o.x + o.w && gy < o.y + o.h) {
        return static_cast<int32_t>(w);
      }
    }
    return -1;
  }
  uint16_t GasSpecies(size_t p, int slot) const {
    return gas_species_data_[p * oni_sim::gas::kMaxSpeciesPerCell + static_cast<size_t>(slot)];
  }
  float GasMass(size_t p, int slot) const {
    return gas_mass_data_[p * oni_sim::gas::kMaxSpeciesPerCell + static_cast<size_t>(slot)];
  }
  uint8_t GasOccupiedMask(size_t p) const { return gas_occupied_mask_data_[p]; }
  uint8_t GasDirty(size_t p) const { return gas_dirty_[p]; }
  uint8_t GasSleeping(size_t p) const { return gas_sleeping_[p]; }
  // The whole array, one byte per PADDED cell, for SIM_DebugGasSleeping. Read-only and with
  // no setter counterpart, unlike stable_ticks_: `Allocate` resets this to all-awake and every
  // `Load` goes through one, but that is measured to cost nothing but a few ticks of skipped
  // optimisation. A pair is skipped only when BOTH ends sleep, and both only sleep after five
  // consecutive ticks in which every transfer between them fell under MixPair's
  // MinTransferMass floor; a sub-floor transfer is never applied, so nothing accumulates, and
  // nothing can write into such a pair without waking one end first. Skipping it is therefore
  // exactly equal to running it, which is why a restored world that has forgotten every
  // sleeping bit still reproduces the run it came from.
  const std::vector<uint8_t>& GasSleepingArray() const { return gas_sleeping_; }
  uint8_t RoomPromoted(size_t p) const { return room_promoted_data_[p]; }

  // --------------------------------------------------------------- seeding

  // SimData_InitializeFromCells: unpadded arrays of Sim.Cell / DiseaseCell /
  // SimBackwall, in that order, after the header.
  bool InitializeFromCells(const uint8_t* data, size_t size, const ElementTable& table,
                           const DiseaseTable& diseases) {
    size_t off = 0;
    auto take = [&](void* dst, size_t n) {
      if (off + n > size) return false;
      memcpy(dst, data + off, n);
      off += n;
      return true;
    };
    int32_t w = 0, h = 0;
    uint32_t seed = 0;
    uint8_t radiation_enabled = 0, headless = 0;
    if (!take(&w, 4) || !take(&h, 4) || !take(&seed, 4) || !take(&radiation_enabled, 1) ||
        !take(&headless, 1)) {
      return false;
    }
    if (w <= 0 || h <= 0) return false;
    Allocate(w, h);
    radiation_enabled_ = radiation_enabled != 0;
    headless_ = headless != 0;
    // The seed this message carries is the random stream, verbatim. The game stores it as the
    // stream state and nothing else ever writes it, so
    // a world's whole gas shuffle is determined here. The *other* constructor call site,
    // `AllocateCells`, passes `_time64(NULL)` instead — a loaded save is not reproducible.
    rng_ = seed;
    shuffle_dir_ = -1;

    const size_t n = GameCount();
    for (size_t i = 0; i < n; ++i) {
      Cell c{};
      if (!take(&c, sizeof(Cell))) return false;
      const size_t p = Padded(i);
      phases_[p] = {c.elementIdx, c.mass, c.temperature};
      properties_[p] = c.properties;
      insulation_[p] = c.insulation;
      strength_[p] = c.strengthInfo;
    }
    for (size_t i = 0; i < n; ++i) {
      DiseaseCell d{};
      if (!take(&d, sizeof(DiseaseCell))) return false;
      const size_t p = Padded(i);
      // 0xFF means "no disease". The save format keys diseases by hash, the game data
      // update by index, so both are kept and neither is derived on the fly.
      disease_idx_[p] = d.diseaseIdx;
      disease_[p].diseaseHash = (d.diseaseIdx == 0xFF) ? 0 : diseases.HashOf(d.diseaseIdx);
      disease_[p].count = (d.diseaseIdx == 0xFF) ? 0 : d.elementCount;
      // The game writes zero into both of these, but the payload carries them and the sim
      // owns them from here on, so take what was sent rather than assuming the zero.
      disease_infest_[p] = d.reservedInfestationTickCount;
      disease_accum_[p] = d.reservedAccumulatedError;
    }
    for (size_t i = 0; i < n; ++i) {
      SimBackwall b{};
      if (!take(&b, sizeof(SimBackwall))) return false;
      const size_t p = Padded(i);
      backwall_[p] = {table.BackwallHash(b.elementIdx), b.mass, b.temperature};
    }
    // A freshly loaded world owes the game a transition check on every backwall it just
    // took; Klei announces its whole out-of-range set on the first frame after a load.
    backwall_dirty_all_ = true;
    SealBorder(table);
    return true;
  }

  // --------------------------------------------------------------- save / load

  // True the instant any cell holds a real species — cheap
  // enough to scan at save time (not a hot path) rather than maintaining a live flag that
  // every InjectSpecies/mixing write would have to keep in sync.
  bool HasGasMixtureData() const {
    if (gas_occupied_mask_data_ == nullptr) return false;
    for (size_t i = 0, n = PaddedCount(); i < n; ++i) {
      if (gas_occupied_mask_data_[i] != 0) return true;
    }
    return false;
  }

  // Room promotion has to survive save/load. Independent of
  // HasGasMixtureData: ApplyPromoteRoom only requires vf_active, not that the promoted room's
  // own cells still hold nonzero gas-mixture mass at save time (a room can be promoted and
  // then have its injected gas fully consumed/vented before the next save). ToBlob and the
  // Load handler both need to know about a promoted room even when this is true and
  // HasGasMixtureData() is false.
  bool HasAnyRoomPromoted() const {
    if (room_promoted_data_ == nullptr) return false;
    for (size_t i = 0, n = PaddedCount(); i < n; ++i) {
      if (room_promoted_data_[i] != 0) return true;
    }
    return false;
  }

  // Matches one blob record to a registered property, or explains why it cannot be. Shared by
  // the two SAVE load paths so they can never disagree about what a mismatch is -- the
  // sub-region path once lacked the gas/room restore entirely, and the bug was silent and
  // total (see its comment). Returns:
  //
  //    >= 0  the property index; the record's shape matches what this build registered
  //     -1   no loaded mod registered this name -- REPORTED, not fatal. A player who
  //          uninstalls one mod must not find every save unopenable; stage 2.4's orphan side
  //          list is what carries the bytes through instead of dropping them.
  //     -2   the name is registered but the SHAPE disagrees -- fatal. A mod that changed its
  //          own property's type or arity between versions gets a refusal naming the property,
  //          never a reinterpretation of the old bytes as the new shape.
  //
  // Stage 2b moved the body to `ext_state::ResolveRecord` so the checkpoint carrier resolves
  // records by exactly the same rules; this adds the sentence about the save path's orphan
  // handling, which is the only part that differs between the two callers.
  int32_t ResolveExtRecord(const SaveExtProperty& e, std::string* error) const {
    const int32_t idx = ext_state::ResolveRecord(ext_cells_, e, error);
    if (idx == -1 && error != nullptr) {
      *error += "; its data is carried verbatim and written back unchanged";
    }
    return idx;
  }

  // Finds or creates this world's carrier for an unregistered property, sized to THIS world's
  // grid and pre-filled with the record's own `defaultBits`. Null for a record whose shape
  // cannot be honoured. See ToBlob's stage 2.4 comment for why orphans are carried at all.
  SaveExtProperty* EnsureOrphan(const SaveExtProperty& e) {
    for (SaveExtProperty& o : ext_orphans_) {
      if (o.name == e.name) {
        return (o.arity == e.arity && o.stride == e.stride) ? &o : nullptr;
      }
    }
    const size_t chunk = static_cast<size_t>(e.arity) * static_cast<size_t>(e.stride);
    if (e.arity < 1 || e.stride < 1 || chunk == 0) return nullptr;
    if (ext_orphans_.size() >= static_cast<size_t>(kSaveExtMaxProperties)) return nullptr;
    SaveExtProperty o;
    o.name = e.name;
    o.type = e.type;
    o.arity = e.arity;
    o.stride = e.stride;
    o.cell_count = static_cast<int32_t>(PaddedCount());
    o.default_bits = e.default_bits;
    o.bytes.assign(PaddedCount() * chunk, 0);
    if (e.default_bits != 0) {
      const size_t stride = static_cast<size_t>(e.stride);
      for (size_t off = 0; off + stride <= o.bytes.size(); off += stride) {
        memcpy(o.bytes.data() + off, &o.default_bits, stride);
      }
    }
    ext_orphans_.push_back(std::move(o));
    return &ext_orphans_.back();
  }

  static constexpr int32_t kVacuumHash = 758759285;

  // `Load` rewrites every cell it takes, on the saved-game path and the cluster path alike;
  // `SimData_InitializeFromCells` does not. This is Klei's `CellRead` followed by
  // the rest of `Load`'s per-cell loop, in Klei's order, and `driver/src/worldgen_test` checks each step against Klei's DLL:
  //
  //   1. Two renamed element hashes are mapped to their new names before the lookup.
  //   2. A temperature that is not finite becomes 293 K, a mass that is not finite 100 kg, and
  //      radiation that is not finite 0.
  //   3. A hash the table does not hold loads as Vacuum with no mass, heat or radiation.
  //   4. A temperature at or below 0 K becomes exactly 293 K, massless or not, and radiation at
  //      or below 0 becomes 0.
  //   5. Vacuum and Void keep no mass, heat or radiation.
  //
  // A world's own border ring is 0 K Neutronium under 9999 kg of 0 K Vacuum, and on the
  // cluster path that ring lands inside the playable grid, so step 4 is what keeps it from
  // freezing the gas next to it.
  //
  // KLEI'S STEP 6 IS NOT HERE, AND IT IS NOT SKIPPED. `DoLoadTimeStateTransition`
  // moves a cell that is more than 3 K out of its element's range across one transition. In this
  // fork that transition has to pass the same `PhaseRules` gates the frame's does, or every load
  // would condense or boil every cell a mod's rule is holding -- and `BoilingAllowed` reads the
  // column ABOVE the cell, which this per-cell reader cannot see. So it runs as a separate pass
  // once the whole blob is down: `LoadTimeStateTransitions` (sim/physics.h), called from the
  // `Load` handler over `LastLoadRect()`. With no rules registered it is Klei's step 6 exactly
  // (`driver/src/worldgen_test` checks it on both DLLs).
  //
  // Step 3 never refuses an unknown hash: the game does not, and a refused load is a colony the player cannot open, for one missing element.
  static constexpr int32_t kLegacyHashA = 0x0280cf79;
  static constexpr int32_t kLegacyHashATo = static_cast<int32_t>(0x987dac06);
  static constexpr int32_t kLegacyHashB = 0x2391c22b;
  static constexpr int32_t kLegacyHashBTo = 0x4c76ae31;

  static void ReadLoadedCell(const SaveCell& in, const ElementTable& table, PhaseEntry* out,
                             float* radiation) {
    int32_t hash = in.elementHash;
    if (hash == kLegacyHashA) {
      hash = kLegacyHashATo;
    } else if (hash == kLegacyHashB) {
      hash = kLegacyHashBTo;
    }
    float temperature = std::isfinite(in.temperature) ? in.temperature : 293.0f;
    float mass = std::isfinite(in.mass) ? in.mass : 100.0f;
    float rad = std::isfinite(in.radiation) ? in.radiation : 0.0f;
    uint16_t element;
    if (table.HasHash(hash)) {
      element = table.IndexOfHash(hash);
    } else {
      element = table.VacuumIndex();
      mass = 0.0f;
      temperature = 0.0f;
      rad = 0.0f;
    }
    if (temperature <= 0.0f) temperature = 293.0f;
    if (rad <= 0.0f) rad = 0.0f;
    if (element == table.VoidIndex() || element == table.VacuumIndex()) {
      temperature = 0.0f;
      mass = 0.0f;
      rad = 0.0f;
    }
    *out = {element, mass, temperature};
    *radiation = rad;
  }

  SaveBlob ToBlob(int32_t x, int32_t y, const ElementTable& table) const {
    SaveBlob b;
    b.width = width_;
    b.height = height_;
    b.x = x;
    b.y = y;
    const size_t n = PaddedCount();
    b.cells.resize(n);
    b.disease = disease_;
    b.backwall = backwall_;
    for (SaveBackwall& bw : b.backwall) {
      if (bw.elementHash == ElementTable::kNoElementHash) bw.elementHash = kVacuumHash;
    }
    for (size_t i = 0; i < n; ++i) {
      b.cells[i].elementHash = table.At(phases_[i].element).id;
      b.cells[i].temperature = phases_[i].temperature;
      b.cells[i].mass = phases_[i].mass;
      b.cells[i].radiation = radiation_[i];
    }
    // Everything the v16 and v17 fixed sections carry -- the gas-mixture triple and the room-promotion byte -- is a registered kSaved
    // extension property now, so this writes ONE self-describing section at
    // kSaveVersionExtensions instead of two fixed ones at two versions.
    //
    // ABSENT, NOT EMPTY. `HasSavedData()` is false for a world that registered kSaved
    // properties and never wrote one, and such a world writes the plain kSaveVersion blob,
    // byte-identical to Klei's own format, which `diffsim` checks from BOTH sides (`klei loads mine: accepted`). A v18 blob
    // carrying a zero count would break that for every world in the game.
    //
    // WHY v18 REPLACES v16/v17 RATHER THAN STACKING ON THEM: every section gate in saveblob.h is `>=`, never `==`, so a version number is
    // a CUMULATIVE CLAIM and not a label. Setting `b.version = 18` while leaving the gas and
    // room sections to the old code would make `EncodeSaveBlob` copy n * sizeof(SaveGasCell) bytes
    // out of an empty `b.gas` -- for a world that has extension data but no gas mixture, which
    // is the ordinary case. Adding a version on top of that chain means paying for every
    // section below it whether or not you have anything to put there. Folding them in is what
    // stops the chain growing, and it is the whole point of the self-describing section.
    //
    // Once the section exists, EVERY kSaved property goes into it, including any that happen
    // to be all-default -- one rule for whether the section is there, not one rule per
    // property. That is what the old code already did when a promoted room forced v17 with an
    // empty gas-mixture layer.
    //
    // `skip_default=false` here and true for the checkpoint (`ext_state::Save`): once the save
    // section exists EVERY kSaved property goes into it, all-default ones included, because
    // one rule for whether the section is there beats one rule per property. A checkpoint is
    // taken on a ring and pays for its size on every seek, so it makes the other trade.
    if (ext_cells_.HasSavedData()) {
      b.version = kSaveVersionExtensions;
      ext_state::CollectRecords(ext_cells_,
                                ext_cells_.PropertiesWithPersistence({ext::kSaved}),
                                /*skip_default=*/false, &b.ext);
    }
    // The opaque side list. Properties this build did not register, carried
    // verbatim from the blob that was loaded and written straight back out.
    //
    // The case is a player uninstalling a mod for one session. Without this, that session's
    // first save destroys the mod's data permanently -- silently, and for everyone who ever
    // toggles a mod off to test something. It is the one part of this stage nothing fails
    // without until it has already cost somebody a colony, which is exactly why it is here
    // rather than on a list.
    //
    // Orphans are appended AFTER the registered properties, which is stable across repeated
    // round-trips: once an orphan is at the end it stays there. They also force the section to
    // exist even when no registered property has anything to say -- a blob that carried
    // orphans was a v18 blob, and re-saving it must not drop them to write a v15.
    if (!ext_orphans_.empty()) {
      b.version = kSaveVersionExtensions;
      for (const SaveExtProperty& o : ext_orphans_) b.ext.push_back(o);
    }
    // Whenever the section is written, so is the table its element indices mean something
    // against (saveblob.h, kSaveVersionElementPalette). A world with nothing to say never gets
    // here and still writes Klei's v15.
    if (!b.ext.empty() && table.Count() > 0) {
      b.version = kSaveVersionElementPalette;
      b.element_palette.resize(static_cast<size_t>(table.Count()));
      for (int32_t i = 0; i < table.Count(); ++i) {
        b.element_palette[static_cast<size_t>(i)] = table.At(static_cast<uint16_t>(i)).id;
      }
    }
    return b;
  }

  // Loading resolves element *hashes*, so a table whose ordering changed since the save
  // was written still produces the right materials. A hash the table does not hold loads as
  // Vacuum, as it does in Klei's `CellRead`; only a missing table is refused.
  bool FromBlob(const SaveBlob& b, const ElementTable& table,
                const DiseaseTable& diseases, std::string* error) {
    if (b.width <= 2 || b.height <= 2) {
      if (error) *error = "blob has degenerate dimensions";
      return false;
    }
    if (table.Empty()) {
      if (error) *error = "Load before Elements_CreateTable";
      return false;
    }
    Allocate(b.GameWidth(), b.GameHeight());
    load_dropped_species_slots_ = 0;
    load_dropped_species_kg_ = 0.0;
    const size_t n = PaddedCount();
    if (b.cells.size() != n || b.disease.size() != n || b.backwall.size() != n) {
      if (error) *error = "blob arrays do not match its own dimensions";
      return false;
    }
    // A v13 blob's ring is not read by Klei (saveblob.h, kSaveVersionKleiNoBackwall): its
    // ring is the one AllocateCells laid down, which is `SealBorder`'s, applied below once
    // the backwall is in. Measured byte-identical to Klei on a real v13 blob.
    const auto from_blob = [&](size_t i) {
      if (b.ring_from_blob) return true;
      const size_t x = i % static_cast<size_t>(width_), y = i / static_cast<size_t>(width_);
      return x != 0 && y != 0 && x + 1 != static_cast<size_t>(width_) &&
             y + 1 != static_cast<size_t>(height_);
    };
    for (size_t i = 0; i < n; ++i) {
      if (from_blob(i)) ReadLoadedCell(b.cells[i], table, &phases_[i], &radiation_[i]);
    }
    load_rect_ = {0, 0, width_, height_};
    if (b.ring_from_blob) {
      disease_ = b.disease;
    } else {
      for (size_t i = 0; i < n; ++i) {
        if (from_blob(i)) disease_[i] = b.disease[i];
      }
    }
    backwall_ = b.backwall;
    if (!b.ring_from_blob) SealBorder(table);
    backwall_dirty_all_ = true;
    disease_idx_.assign(n, 0xFF);
    // The blob carries the hash and the count and nothing else, so a reload starts every
    // cell's growth remainder and infestation age from zero. That is what Klei's own save
    // does — `SaveDisease` is eight bytes there too.
    disease_accum_.assign(n, 0.0f);
    disease_infest_.assign(n, 0);
    for (size_t i = 0; i < n; ++i) {
      if (disease_[i].diseaseHash != 0) {
        disease_idx_[i] = diseases.IndexOfHash(disease_[i].diseaseHash);
      }
    }
    // Insulation, strength and properties are deliberately absent from the blob; the
    // game re-applies them as buildings load. Leaving them zeroed matches Klei.
    //
    // THE LEGACY READ PATH, and it is permanent. v16 and v17 blobs were written by earlier
    // builds of this sim; nothing writes one any more (ToBlob emits kSaveVersionExtensions instead), and this has to keep
    // reading them forever. `Allocate` above already sized every registered property and
    // filled it with its default, which is exactly right for a legacy kSaveVersion blob that
    // carries none of this.
    //
    // The destinations are registry storage now rather than member vectors, so this is where
    // the format migration actually happens: bytes that arrived in a fixed section leave in a
    // self-describing one. `dirty`/`sleeping` are in neither (see SaveGasCell's comment) and
    // stay at Allocate's zeroed default -- a freshly loaded gas cell starts dirty-clear and
    // awake, same as a freshly allocated one.
    if (b.version >= kSaveVersionGasMixture && b.gas.size() == n &&
        gas_occupied_mask_data_ != nullptr) {
      for (size_t i = 0; i < n; ++i) {
        gas_occupied_mask_data_[i] = b.gas[i].occupied_mask;
        for (int s = 0; s < oni_sim::gas::kMaxSpeciesPerCell; ++s) {
          const size_t idx = i * oni_sim::gas::kMaxSpeciesPerCell + static_cast<size_t>(s);
          gas_species_data_[idx] = b.gas[i].species[s];
          gas_mass_data_[idx] = b.gas[i].mass[s];
        }
      }
    }
    // Same, for the room-promotion byte. No RoomGraph rebuild happens here: the caller (the
    // Load handler) does that afterward via ActivateVolumeFractions, which reads this back out
    // through BuildRoomGraph -- see kSaveVersionRoomPromotion's comment for why that is the
    // only place the bit needs restoring.
    if (b.version >= kSaveVersionRoomPromotion && b.room_promoted.size() == n &&
        room_promoted_data_ != nullptr) {
      memcpy(room_promoted_data_, b.room_promoted.data(), n);
    }
    // The self-describing section. `Allocate` above
    // already sized every registered property and filled it with its default, which is exactly
    // the right answer for a property this build registered that the blob does not carry -- a
    // mod ADDED since the save. See ResolveExtRecord for the other two mismatch cases.
    //
    // A kExtElementIdx record is rewritten through the palette on its way in, registered or
    // orphaned alike (abi/sim_abi_ext.h, kExtElementIdx).
    load_dropped_element_indices_ = 0;
    const ElementRemap remap = BuildElementRemap(b, table);
    for (const SaveExtProperty& e : b.ext) {
      const int32_t idx = ResolveExtRecord(e, error);
      if (idx == -1) {
        // Stage 2.4: keep it so the next save writes it back. Gated on the record covering
        // this world's grid, which it does on this path by construction (the decoder already
        // rejected any record whose cell_count disagreed with the blob, and `Allocate` above
        // sized this world to that same blob). Verbatim except for element indices: the next
        // save writes this table's palette, not the one these bytes were saved against.
        if (static_cast<size_t>(e.cell_count) == n) {
          ext_orphans_.push_back(e);
          if (IsElementIndexRecord(e)) {
            SaveExtProperty& o = ext_orphans_.back();
            RemapElementIndices(o.bytes.data(), o.bytes.size() / 2, remap);
          }
        }
        continue;
      }
      if (idx == -2) return false;
      if (IsElementIndexRecord(e) && remap.active) {
        std::vector<uint8_t> bytes = e.bytes;
        RemapElementIndices(bytes.data(), bytes.size() / 2, remap);
        if (ext_cells_.LoadBytes(idx, bytes.data(), bytes.size())) continue;
      } else if (ext_cells_.LoadBytes(idx, e.bytes.data(), e.bytes.size())) {
        continue;
      }
      {
        if (error) {
          *error = "extension property \"" + e.name + "\" carries " +
                   std::to_string(e.bytes.size()) + " bytes, but this world needs " +
                   std::to_string(ext_cells_.ByteCount(idx));
        }
        return false;
      }
    }
    // After the mixture layer is back, not before: occupancy is what decides both. The remap
    // goes first, because a species whose element is gone leaves its cell unoccupied, and an
    // unoccupied Vacuum cell has no heat to keep.
    for (size_t i = 0; i < n; ++i) {
      RemapSpecies(i, remap);
      KeepMixtureTemperature(i, b.cells[i], table);
    }
    return true;
  }

  // The gas mixture's species are element-table INDICES, and a table can change between a save
  // and its load -- `ElementLoader` sorts, so any element added or removed below a gas moves
  // it. A blob from kSaveVersionElementPalette on carries the hash at every index it was
  // written against, and this turns each saved index back into the element it meant.
  //
  // A species whose element this table does not hold is DROPPED: slot freed, mass gone. That
  // is Klei's own answer for a cell whose element is unknown (it loads as Vacuum,
  // `ReadLoadedCell`), and there is nothing to simulate an element with that has no row. It is
  // counted rather than silent: the `Load` handler reports it (`LoadDroppedSpeciesSlots`).
  //
  // The same map serves every saved `kExtElementIdx` record (`RemapElementIndices`), where a
  // gone element becomes `ext::kExtNoElement` instead: a registry property has no mass to drop
  // and no slot to free, and what "no element" means to it is its owner's business.
  //
  // A v18 blob or older has no palette and is trusted as saved -- `active` stays false.
  // `identity` is the common case, the table unchanged, and costs one pass over the palette.
  static constexpr uint16_t kElementGone = ext::kExtNoElement;
  struct ElementRemap {
    std::vector<uint16_t> to;
    bool active = false;
    bool identity = true;
  };
  static ElementRemap BuildElementRemap(const SaveBlob& b, const ElementTable& table) {
    ElementRemap r;
    if (b.element_palette.empty()) return r;
    r.active = true;
    r.to.resize(b.element_palette.size());
    for (size_t i = 0; i < b.element_palette.size(); ++i) {
      const int32_t hash = b.element_palette[i];
      const uint16_t to = table.HasHash(hash) ? table.IndexOfHash(hash) : kElementGone;
      r.to[i] = to;
      if (to != i) r.identity = false;
    }
    return r;
  }
  void RemapSpecies(size_t cell, const ElementRemap& r) {
    if (!r.active || r.identity || gas_occupied_mask_data_ == nullptr) return;
    uint8_t mask = gas_occupied_mask_data_[cell];
    if (mask == 0) return;
    constexpr int kSlots = oni_sim::gas::kMaxSpeciesPerCell;
    for (int s = 0; s < kSlots; ++s) {
      if (!(mask & (1u << s))) continue;
      const size_t lane = cell * kSlots + static_cast<size_t>(s);
      const uint16_t from = gas_species_data_[lane];
      const uint16_t to = from < r.to.size() ? r.to[from] : kElementGone;
      if (to != kElementGone) {
        gas_species_data_[lane] = to;
        continue;
      }
      ++load_dropped_species_slots_;
      load_dropped_species_kg_ += gas_mass_data_[lane];
      gas_species_data_[lane] = oni_sim::gas::kEmptySpecies;
      gas_mass_data_[lane] = 0.0f;
      mask = static_cast<uint8_t>(mask & ~(1u << s));
    }
    gas_occupied_mask_data_[cell] = mask;
  }
  // A saved record whose bytes are element indices. The stride is checked as well as the type
  // because an orphan's shape comes from the blob alone, with no registration to vouch for it.
  static bool IsElementIndexRecord(const SaveExtProperty& e) {
    return e.type == ext::kExtElementIdx && e.stride == 2;
  }
  // Rewrites `components` two-byte element indices in place through the palette. Unlike
  // `RemapSpecies` this does not skip an identity map: an index past the end of the palette
  // named no element when it was saved either, and it comes out as `kExtNoElement` whether or
  // not the table moved, so the rule does not depend on what else a player changed.
  // `kExtNoElement` passes through, and is not counted.
  void RemapElementIndices(uint8_t* bytes, size_t components, const ElementRemap& r) {
    if (!r.active) return;
    for (size_t c = 0; c < components; ++c) {
      uint16_t from = 0;
      memcpy(&from, bytes + c * 2, 2);
      if (from == ext::kExtNoElement) continue;
      const uint16_t to = from < r.to.size() ? r.to[from] : kElementGone;
      if (to == kElementGone) ++load_dropped_element_indices_;
      if (to != from) memcpy(bytes + c * 2, &to, 2);
    }
  }
  // The padded rectangle the last `FromBlob` or `LoadIntoCluster` wrote, half-open: the cells
  // `LoadTimeStateTransitions` (sim/physics.h) visits. Empty until a load has happened.
  PaddedRect LastLoadRect() const { return load_rect_; }
  // True for a Vacuum or Void cell whose gas the mixture layer holds. Its temperature is the
  // mixture's (`KeepMixtureTemperature`), not an element's, so no element range applies to it.
  bool MixtureHeldVacuum(size_t cell, const ElementTable& table) const {
    if (gas_occupied_mask_data_ == nullptr || gas_occupied_mask_data_[cell] == 0) return false;
    const uint16_t element = phases_[cell].element;
    return element == table.VacuumIndex() || element == table.VoidIndex();
  }
  // What the last `FromBlob` or `LoadIntoCluster` dropped for want of an element (see above).
  int32_t LoadDroppedSpeciesSlots() const { return load_dropped_species_slots_; }
  double LoadDroppedSpeciesKg() const { return load_dropped_species_kg_; }
  // Saved `kExtElementIdx` components the last load turned into `kExtNoElement`, registered
  // properties and orphans together.
  int32_t LoadDroppedElementIndices() const { return load_dropped_element_indices_; }

  // THE ONE PLACE THIS LOAD IS NOT KLEI'S, AND IT CANNOT BE. `ReadLoadedCell` takes Vacuum's
  // heat away, which is right for Klei's Vacuum and wrong for this fork's: a cell whose gas the
  // mixture layer holds is vanilla Vacuum with no mass (`ZeroMasslessCells`, `ClearCell` and
  // `Evaporate` all leave it that way on purpose), and `PhaseEntry.temperature` is the
  // mixture's own shared temperature (abi/gas_mixture_abi.h). Zeroing it would delete the heat of
  // every such cell on every load and park it at 0 K, where `CellPressure` reads 0 Pa and
  // `MixPair`'s `avg_t <= 0` guard stops it mixing: 5 kg of mixture O2 at 250 K would load as
  // 0 K / 0 Pa and stay there. So an occupied Vacuum or Void cell keeps the temperature it was saved with, under the
  // same two rewrites every other cell gets (not finite, or at or below 0 K, becomes 293 K).
  // Its mass and radiation still go to zero: the vanilla layer really is empty.
  //
  // A world that never activated the mixture has no occupied cell, so vanilla saves load
  // exactly as Klei loads them; `worldgen_test` runs against Klei's DLL and still passes.
  void KeepMixtureTemperature(size_t cell, const SaveCell& in, const ElementTable& table) {
    if (gas_occupied_mask_data_ == nullptr || gas_occupied_mask_data_[cell] == 0) return;
    const uint16_t element = phases_[cell].element;
    if (element != table.VacuumIndex() && element != table.VoidIndex()) return;
    const float t = std::isfinite(in.temperature) ? in.temperature : 293.0f;
    phases_[cell].temperature = t > 0.0f ? t : 293.0f;
  }

  // A fresh game allocates the whole cluster once and then sends one `Load` per world, each
  // blob sized to its own world (`SaveLoader.LoadFromWorldGen`). `FromBlob` reallocates to the
  // blob, so the second world shrank the grid under the game's fixed `Grid.WidthInCells` and
  // `Grid.InitializeCells` read past the end of `elementIdx`.
  //
  // Klei places each blob at **its own header** `x, y` -- the arguments worldgen passed to
  // `SIM_BeginSave` -- and not by `DefineWorldOffsets`: two blobs saved at (0,0) land on top of
  // each other at the origin even with both worlds' offsets defined (`driver/src/worldgen_test`,
  // run against Klei's DLL). The padded blob goes down unshifted, so
  // its border ring sits one cell outside the world's game rectangle, inside the cluster. Pairing
  // blobs with offsets by arrival order only agrees while the two lists happen to match.
  bool LoadIntoCluster(const SaveBlob& b, const ElementTable& table,
                       const DiseaseTable& diseases, std::string* error) {
    if (b.width <= 2 || b.height <= 2) {
      if (error) *error = "blob has degenerate dimensions";
      return false;
    }
    if (table.Empty()) {
      if (error) *error = "Load before Elements_CreateTable";
      return false;
    }
    const size_t n = static_cast<size_t>(b.width) * static_cast<size_t>(b.height);
    load_dropped_species_slots_ = 0;
    load_dropped_species_kg_ = 0.0;
    load_dropped_element_indices_ = 0;
    const ElementRemap remap = BuildElementRemap(b, table);
    if (b.cells.size() != n || b.disease.size() != n || b.backwall.size() != n) {
      if (error) *error = "blob arrays do not match its own dimensions";
      return false;
    }
    load_rect_ = {std::max(b.x, 0), std::max(b.y, 0), std::min(b.x + b.width, width_),
                  std::min(b.y + b.height, height_)};
    if (load_rect_.x1 < load_rect_.x0) load_rect_.x1 = load_rect_.x0;
    if (load_rect_.y1 < load_rect_.y0) load_rect_.y1 = load_rect_.y0;
    for (int32_t ly = 0; ly < b.height; ++ly) {
      // `b.width`/`b.height` are PADDED dims and `b.cells` is a padded-order dump, so local
      // (0,0) is the world's own border ring. Bound against the destination's padded grid.
      const int32_t gy = b.y + ly;
      if (gy < 0 || gy >= height_) continue;
      for (int32_t lx = 0; lx < b.width; ++lx) {
        const int32_t gx = b.x + lx;
        if (gx < 0 || gx >= width_) continue;
        const size_t li = static_cast<size_t>(ly) * static_cast<size_t>(b.width) +
                           static_cast<size_t>(lx);
        const size_t gi = static_cast<size_t>(gy) * static_cast<size_t>(width_) +
                           static_cast<size_t>(gx);
        ReadLoadedCell(b.cells[li], table, &phases_[gi], &radiation_[gi]);
        disease_[gi] = b.disease[li];
        backwall_[gi] = b.backwall[li];
        disease_idx_[gi] = 0xFF;
        disease_accum_[gi] = 0.0f;
        disease_infest_[gi] = 0;
        if (disease_[gi].diseaseHash != 0) {
          disease_idx_[gi] = diseases.IndexOfHash(disease_[gi].diseaseHash);
        }
        // This project's own save-format extensions, written into the sub-rectangle the same
        // way everything above is. This block used to be absent, with a comment saying a blob
        // reaching this path is always the legacy vanilla shape (kSaveVersion, 15) because only
        // worldgen produces one — but a save/restore cycle is an allocate-then-load too. A
        // whole-grid blob at (0,0) goes to `FromBlob` (simdll.cpp's `Load` handler); any smaller blob with extensions
        // still comes here, and this stays for it.
        //
        // Without the extension read below, the same blob that a bare `Load` restores in full
        // (479.420492 kg of an injected mixture) restores 0.000000 kg through `AllocateCells` +
        // `Load`, while still reporting a successful load. Silent, and total.
        //
        // Gated on the blob's own version and its own array length, the same two conditions
        // `FromBlob` uses, so a legacy blob still leaves `Allocate`'s zeroed "never activated"
        // default untouched -- which is what the old comment was right about.
        if (b.version >= kSaveVersionGasMixture && b.gas.size() == n &&
            gas_occupied_mask_data_ != nullptr) {
          gas_occupied_mask_data_[gi] = b.gas[li].occupied_mask;
          for (int s = 0; s < oni_sim::gas::kMaxSpeciesPerCell; ++s) {
            const size_t dst = gi * oni_sim::gas::kMaxSpeciesPerCell + static_cast<size_t>(s);
            gas_species_data_[dst] = b.gas[li].species[s];
            gas_mass_data_[dst] = b.gas[li].mass[s];
          }
        }
        if (b.version >= kSaveVersionRoomPromotion && b.room_promoted.size() == n &&
            room_promoted_data_ != nullptr) {
          room_promoted_data_[gi] = b.room_promoted[li];
        }
        // The self-describing section, restored PER CELL because this path
        // maps the blob's own local index onto a different global one. `FromBlob`'s wholesale
        // LoadBytes would put a sub-region's bytes at the wrong cells entirely.
        //
        // This block exists for the same reason the two above it do, and the comment above
        // says what happens when it does not: a v18 blob coming through allocate-then-load
        // would restore a full 22 MB section into nothing, silently, while reporting success.
        for (const SaveExtProperty& e : b.ext) {
          const int32_t pidx = ResolveExtRecord(e, error);
          if (pidx == -2) continue;  // shape mismatch: reported by the resolver
          if (pidx == -1) {
            // Stage 2.4 on the sub-region path, and this is NOT the exotic case: the comment
            // above explains that an ordinary allocate-then-load save/restore comes through
            // here. An orphan dropped here is a mod's data destroyed on a normal reload.
            //
            // Destination-sized rather than blob-sized, because this path places a small blob
            // inside a larger grid. Cells the blob does not cover keep the property's own
            // registered default, which is why `defaultBits` is in the record at all -- a
            // build that did not register the property has no other way to know it.
            SaveExtProperty* o = EnsureOrphan(e);
            if (o == nullptr) continue;
            const size_t chunk = static_cast<size_t>(e.arity) * static_cast<size_t>(e.stride);
            if (chunk == 0 || (li + 1) * chunk > e.bytes.size()) continue;
            if ((gi + 1) * chunk > o->bytes.size()) continue;
            memcpy(o->bytes.data() + gi * chunk, e.bytes.data() + li * chunk, chunk);
            if (IsElementIndexRecord(e)) {
              RemapElementIndices(o->bytes.data() + gi * chunk, chunk / 2, remap);
            }
            continue;
          }
          const size_t chunk = ext_cells_.CellChunk(pidx);
          if (chunk == 0 || (li + 1) * chunk > e.bytes.size()) continue;
          if (IsElementIndexRecord(e) && remap.active) {
            // One cell's components, at most kExtMaxArity * 2 bytes: the resolver matched this
            // record's arity to the registration, and registration caps it.
            uint8_t cell_bytes[ext::kExtMaxArity * 2];
            if (chunk > sizeof(cell_bytes)) continue;
            memcpy(cell_bytes, e.bytes.data() + li * chunk, chunk);
            RemapElementIndices(cell_bytes, chunk / 2, remap);
            ext_cells_.WriteCellChunk(pidx, gi, cell_bytes);
            continue;
          }
          ext_cells_.WriteCellChunk(pidx, gi, e.bytes.data() + li * chunk);
        }
        RemapSpecies(gi, remap);
        KeepMixtureTemperature(gi, b.cells[li], table);
      }
    }
    backwall_dirty_all_ = true;
    return true;
  }

  bool RadiationEnabled() const { return radiation_enabled_; }
  void SetRadiationEnabled(bool v) { radiation_enabled_ = v; }

  // The five tunables `ProcessCellRadiationChanges` writes from a `RadiationParamsModification`
  // and `SimData::SimData` seeds. The defaults are the
  // constructor's own, matched against the shipped library rather than guessed, and a world
  // that never receives the message runs on them forever.
  //
  // Type 1 is missing from Klei's `if` chain: the message can carry it and nothing happens.
  float RadiationLingerRate() const { return radiation_linger_rate_; }
  float RadiationMaxMass() const { return radiation_max_mass_; }
  float RadiationBaseWeight() const { return radiation_base_weight_; }
  float RadiationDensityWeight() const { return radiation_density_weight_; }
  float RadiationConstructedFactor() const { return radiation_constructed_factor_; }
  void SetRadiationParam(int32_t type, float v) {
    switch (type) {
      case 0: radiation_linger_rate_ = v; break;
      case 2: radiation_base_weight_ = v; break;
      case 3: radiation_density_weight_ = v; break;
      case 4: radiation_constructed_factor_ = v; break;
      case 5: radiation_max_mass_ = v; break;
      default: break;
    }
  }
  uint8_t DiseaseIdx(size_t padded) const { return disease_idx_[padded]; }
  const std::vector<uint8_t>& DiseaseIdx() const { return disease_idx_; }

  // The two per-cell disease fields the *sim* owns and the game never sees. They are the
  // `reservedAccumulatedError` and `reservedInfestationTickCount` slots of `Sim.DiseaseCell`
  // — "reserved" because the game writes zero into both and only reads them back — and
  // `SimData::ClearCell` zeroes them along with everything else.
  //
  // No `MarkStatic`: neither is projected, so a write to one owes the game nothing.
  float& MutableDiseaseAccum(size_t p) { return disease_accum_[p]; }
  const std::vector<float>& DiseaseAccum() const { return disease_accum_; }
  uint8_t& MutableDiseaseInfest(size_t p) { return disease_infest_[p]; }
  const std::vector<uint8_t>& DiseaseInfest() const { return disease_infest_; }

  // ext::kSetDiseaseGrowth. The two arrays above are per-cell growth state that Klei's save
  // format has no field for -- SaveDisease is a hash and a count -- so `FromBlob` zeroes both
  // on every load, and a replay from a restored world therefore runs its disease growth from
  // zero while the run it is replaying does not. Both are written together or neither is:
  // an infestation age without the growth remainder that produced it is not a state.
  // ext::kSetCellRadiation, the eighth checkpoint component: the whole radiation field back,
  // after a `Load` that cleared it in Vacuum and Void cells. Through `SetRadiation`, so only the
  // cells whose value actually changes are marked for the projection.
  bool SetRadiationField(const float* values, size_t count) {
    if (!values || count != radiation_.size()) return false;
    for (size_t p = 0; p < count; ++p) SetRadiation(p, values[p]);
    return true;
  }

  bool SetDiseaseGrowth(const float* accum, const uint8_t* infest, size_t count) {
    if (!accum || !infest || count != disease_accum_.size()) return false;
    memcpy(disease_accum_.data(), accum, count * sizeof(float));
    memcpy(disease_infest_.data(), infest, count);
    return true;
  }

  // Whether any cell is infected at all. Recomputed once a frame rather than tracked per
  // write: a scan for "any byte that is not 0xFF" over the grid costs microseconds, and a
  // flag maintained by hand at every writer is a flag that goes stale at the one writer
  // nobody thought of. Every scenario in the suite but the disease ones answers false here
  // and skips the snapshot, the pair sweep and the growth sweep entirely.
  // Answered once a frame and cached, because the sweeps that ask are inside the substep
  // and region loops. Refreshing at the top of the frame is sound rather than merely cheap:
  // nothing creates disease mid-frame except a kernel moving germs that were already
  // somewhere, so a frame that starts clean stays clean.
  void RefreshDiseaseActive() { disease_active_ = AnyDisease(); }
  bool DiseaseActive() const { return disease_active_; }

  bool AnyDisease() const {
    for (uint8_t v : disease_idx_) {
      if (v != 0xFF) return true;
    }
    return false;
  }

 private:
  // Past a sixteenth of the grid the list stops being cheaper than the copy it replaces,
  // and it also stops being bounded, so it collapses to "all".
  void MarkBackwall(size_t padded) {
    if (backwall_dirty_all_) return;
    if (backwall_dirty_.size() >= PaddedCount() / 16) {
      backwall_dirty_all_ = true;
      backwall_dirty_.clear();
      return;
    }
    backwall_dirty_.push_back(static_cast<uint32_t>(padded));
  }

  void MarkStatic(size_t padded) {
    if (static_dirty_all_) return;
    if (static_dirty_.size() >= PaddedCount() / 16) {
      static_dirty_all_ = true;
      static_dirty_.clear();
      return;
    }
    static_dirty_.push_back(static_cast<uint32_t>(padded));
  }

  // Inclusive on both axes, clamped to the interior. An empty row is one whose `x1` is
  // below its `x0`, which is the state `kNoDirtyRow` and -1 leave it in.
  void MarkProjectRows(int32_t y0, int32_t y1, int32_t x0, int32_t x1) const {
    if (project_dirty_all_) return;
    if (y0 < 1) y0 = 1;
    if (x0 < 1) x0 = 1;
    if (y1 > height_ - 2) y1 = height_ - 2;
    if (x1 > width_ - 2) x1 = width_ - 2;
    if (y1 < y0 || x1 < x0) return;
    for (int32_t y = y0; y <= y1; ++y) {
      const size_t row = static_cast<size_t>(y);
      if (x0 < project_row_x0_[row]) project_row_x0_[row] = x0;
      if (x1 > project_row_x1_[row]) project_row_x1_[row] = x1;
    }
    if (y0 < project_y0_) project_y0_ = y0;
    if (y1 > project_y1_) project_y1_ = y1;
  }

  // The border ring is not a real part of the world; it exists so neighbour loops have
  // something impassable to hit.
  //
  // Klei fills it with **Neutronium at 9999 kg and 0 K, except the top row, which is
  // Vacuum** — the world is open to space at the top and sealed everywhere else. Both
  // the mass and the choice of element were recovered by saving an identically seeded
  // world from Klei's sim and from this one and diffing the blobs, which is the only way
  // to observe the ring: the game cannot address it. 9999 kg of vacuum in the top row is
  // contradictory on its face, which is precisely why it is copied rather than derived.
  void SealBorder(const ElementTable& table) {
    constexpr int32_t kNeutroniumHash = 1838482828;
    constexpr int32_t kVacuumHash = 758759285;
    const uint16_t wall = table.HasHash(kNeutroniumHash)
                              ? table.IndexOfHash(kNeutroniumHash)
                              : static_cast<uint16_t>(0);
    const uint16_t space = table.HasHash(kVacuumHash) ? table.IndexOfHash(kVacuumHash)
                                                      : static_cast<uint16_t>(0);
    // The border's backwall is Vacuum everywhere, including behind the Neutronium ring.
    const int32_t vacuum_hash = table.HasHash(kVacuumHash) ? kVacuumHash : 0;
    auto seal = [&](size_t p, uint16_t element) {
      phases_[p] = {element, 9999.0f, 0.0f};
      properties_[p] = kSolidImpermeable | kUnbreakable;
      backwall_[p] = {vacuum_hash, 0.0f, 0.0f};
    };
    // Row index height_ - 1 is the highest y, which is the top of the world.
    for (int32_t x = 0; x < width_; ++x) {
      seal(static_cast<size_t>(x), wall);
      seal(static_cast<size_t>(height_ - 1) * width_ + x, space);
    }
    for (int32_t y = 0; y < height_ - 1; ++y) {
      seal(static_cast<size_t>(y) * width_, wall);
      seal(static_cast<size_t>(y) * width_ + (width_ - 1), wall);
    }
  }

  bool allocated_ = false;
  std::vector<WorldOffset> world_offsets_, pending_world_offsets_;
  int32_t game_width_ = 0, game_height_ = 0;
  int32_t width_ = 0, height_ = 0;
  bool radiation_enabled_ = false;
  bool headless_ = false;
  float radiation_linger_rate_ = 1.1f;
  float radiation_max_mass_ = 2000.0f;
  float radiation_base_weight_ = 0.3f;
  float radiation_density_weight_ = 0.7f;
  float radiation_constructed_factor_ = 0.8f;

  // The state of the game's `rand()`, and the gas shuffle's tie-break stride. Both are set
  // once at construction and never touched by anything else; the stride starts at -1, as it
  // does in the game.
  uint32_t rng_ = 0;
  int32_t shuffle_dir_ = -1;

  std::vector<PhaseEntry> phases_;
  std::vector<float> radiation_;
  std::vector<SaveDisease> disease_;
  std::vector<SaveBackwall> backwall_;
  std::vector<uint32_t> backwall_dirty_;
  bool backwall_dirty_all_ = false;
  std::vector<uint16_t> flow_element_;
  std::vector<uint8_t> properties_;
  std::vector<uint8_t> insulation_;
  std::vector<uint8_t> strength_;
  // Framework extension storage, all of it (abi/sim_abi_ext.h, kRegisterCellProperty).
  // `thermal_mass_bonus_` used to be a `std::vector<float>` right here; it is a registered
  // property under the reserved `sim.` prefix now, and the next one costs no member at all.
  ext::CellPropertyRegistry ext_cells_;
  int32_t thermal_mass_bonus_prop_ = -1;  // index of kThermalMassBonusProperty, set in ctor
  std::string ext_registration_error_;  // see World::World
  const float* thermal_mass_bonus_data_ = nullptr;  // see ThermalMassBonusData
  // The four kSaved first-party properties. Indices set in the constructor,
  // pointers refreshed by `Allocate` and by nothing else.
  int32_t gas_occupied_mask_prop_ = -1;
  int32_t gas_species_prop_ = -1;
  int32_t gas_mass_prop_ = -1;
  int32_t room_promoted_prop_ = -1;
  uint8_t* gas_occupied_mask_data_ = nullptr;
  uint16_t* gas_species_data_ = nullptr;
  float* gas_mass_data_ = nullptr;
  uint8_t* room_promoted_data_ = nullptr;
  // Layer C. See the "liquid payload" block above.
  int32_t dissolved_mass_prop_ = -1;
  float* dissolved_mass_data_ = nullptr;
  DissolvedTint dissolved_tint_;
  Effervescence effervescence_;
  std::vector<PayloadProperty> payload_props_;
  std::vector<float> payload_capture_;
  std::vector<ext::LiquidPayloadReleased> payload_released_;
  std::vector<ext::LiquidPayloadConsumed> payload_consumed_;
  // Properties a loaded blob carried that no registration in THIS build
  // claims -- an uninstalled mod's data, held verbatim so the next save writes it back rather
  // than destroying it. Cleared by `Allocate`: a fresh world inherits nobody's orphans.
  std::vector<SaveExtProperty> ext_orphans_;
  int32_t load_dropped_species_slots_ = 0;
  double load_dropped_species_kg_ = 0.0;
  int32_t load_dropped_element_indices_ = 0;
  PaddedRect load_rect_{0, 0, 0, 0};
  uint16_t inverted_gravity_element_ = 0xFFFF;  // framework extension, see InvertedGravityElement
  float environment_temperature_ = 0.0f;  // framework extension, see EnvironmentTemperature
  bool rooms_dirty_ = false;
  // `dirty` and `sleeping` stay plain members on purpose. Both are transient per-substep
  // bookkeeping rather than state -- SaveGasCell never carried either, and a freshly loaded
  // world starts dirty-clear and awake exactly as a freshly allocated one does. Registering
  // them would mean declaring a persistence for something that has none.
  std::vector<uint8_t> gas_dirty_;
  std::vector<uint8_t> gas_sleeping_;
  std::vector<uint8_t> disease_idx_;
  std::vector<float> disease_accum_;
  std::vector<uint8_t> disease_infest_;
  bool disease_active_ = false;
  std::vector<uint8_t> active_;
  std::vector<uint8_t> active_incl_;
  std::vector<float> region_cosmic_;
  std::vector<float> region_sunlight_;
  std::vector<Environment> world_environments_;
  std::vector<double> world_latent_energy_j_;
  std::vector<uint8_t> world_environment_set_;
  Environment default_environment_;
  std::vector<SunDirection> world_suns_;
  std::vector<uint8_t> world_sun_set_;
  SunDirection default_sun_;
  bool default_sun_set_ = false;
  std::vector<PaddedRect> padded_;
  std::vector<PaddedRect> padded_incl_;
  std::vector<uint8_t> substance_touched_;
  std::vector<int32_t> cell_melted_;
  std::vector<uint8_t> stable_ticks_;
  std::vector<float> flow_;
  std::vector<FlowCell> flow_touched_;
  std::vector<Rect> regions_;
  mutable std::vector<uint32_t> static_dirty_;
  mutable bool static_dirty_all_ = true;

  // The projection's per-row dirty spans. `project_y0_ > project_y1_` means nothing is
  // marked; a marked row always has `project_row_x1_ >= project_row_x0_`.
  static constexpr int32_t kNoDirtyRow = INT32_MAX;
  mutable std::vector<int32_t> project_row_x0_;
  mutable std::vector<int32_t> project_row_x1_;
  mutable int32_t project_y0_ = 0;
  mutable int32_t project_y1_ = -1;
  mutable bool project_dirty_all_ = true;

  // Cumulative since the world was allocated, which is also when the driver takes the
  // baseline total the buckets are subtracted from.
  mutable Ledger ledger_;
  mutable EnergyLedger energy_ledger_;
  std::vector<double> cell_energy_carry_;
  double cell_energy_carry_held_ = 0.0;
  // Not cleared by Allocate(): the flag is a property of how this process was launched, not
  // of the world it is holding, and a reallocation midway through a run must not silently
  // stop charging the buckets the run is being judged on.
  bool energy_ledger_enabled_ = false;
};

// -------------------------------------------------- the per-region snapshot refreshes
//
// THESE ARE FUNCTIONS, NOT BRANCHES INSIDE THE SWEEPS, and that is a measured decision
// rather than a tidiness one. Written inline in `StepFlow`, the two-way branch below cost
// that kernel **0.156 -> 0.184 ms on a SINGLE-region world** -- the path where the branch is
// not even taken and the code executed is the same `assign` it always was. 0.028 ms over a
// 194,312-cell sweep is 0.14 ns a cell, which is the sweep being compiled differently, not
// work being done: the extra body pushed `StepFlow` past some inlining or register budget.
// Proved by ablation -- removing only that branch, with every other part of this change
// still in place, put the kernel back at 0.155 -- and the cost did not move when the
// rectangle computation was short-circuited, so it was the code's presence and not its
// execution. Behind a call the sweeps are byte-for-byte the kernels they were.
//
// `ONI_SNAPSHOT_NOINLINE` is the other half of the same finding. Behind a plain call GCC
// inlined these back into the sweeps and the 0.028 ms came straight back; refusing the
// inline put `StepFlow` at 0.154-0.156 again, which is the number it had before any of this
// existed. The copy is a memmove over a megabyte -- there is nothing to gain by inlining it
// and a measured amount to lose.
//
// See `SnapshotReadRect` above for what the rectangle is and why the margins are what they
// are.
#if defined(__GNUC__)
#define ONI_SNAPSHOT_NOINLINE __attribute__((noinline))
#else
#define ONI_SNAPSHOT_NOINLINE
#endif

ONI_SNAPSHOT_NOINLINE inline void RefreshPhaseSnapshot(std::vector<PhaseEntry>& snap,
                                 const std::vector<PhaseEntry>& cells, size_t ri, int32_t pw,
                                 int32_t ph, int32_t rx0, int32_t ry0, int32_t rx1,
                                 int32_t ry1) {
  if (SnapshotNeedsWholeGrid(snap.size(), cells.size(), ri)) {
    snap.assign(cells.begin(), cells.end());
    return;
  }
  const SnapshotRect q = SnapshotReadRect(pw, ph, rx0, ry0, rx1, ry1);
  ForEachSnapshotRow(pw, q, [&](size_t b, size_t e) {
    std::copy(cells.begin() + static_cast<ptrdiff_t>(b),
              cells.begin() + static_cast<ptrdiff_t>(e),
              snap.begin() + static_cast<ptrdiff_t>(b));
  });
}

// The disease half of the same copy. Gathered a field at a time out of three separate
// arrays rather than copied, which is why it is a loop and not a `std::copy`.
ONI_SNAPSHOT_NOINLINE inline void RefreshDiseaseSnapshot(std::vector<DiseaseEntry>& snap, const World& w, size_t ri,
                                   int32_t pw, int32_t ph, int32_t rx0, int32_t ry0,
                                   int32_t rx1, int32_t ry1) {
  const size_t n = w.PaddedCount();
  auto gather = [&](size_t b, size_t e) {
    for (size_t i = b; i < e; ++i) {
      snap[i].idx = w.DiseaseIdx(i);
      snap[i].count = w.Disease()[i].count;
      snap[i].infest = w.DiseaseInfest()[i];
    }
  };
  if (SnapshotNeedsWholeGrid(snap.size(), n, ri)) {
    snap.resize(n);
    gather(0, n);
    return;
  }
  ForEachSnapshotRow(pw, SnapshotReadRect(pw, ph, rx0, ry0, rx1, ry1), gather);
}

// `StepConduction`'s snapshot is temperature only -- 4 bytes a cell against 12 -- and its
// readers are the tile summary and the pair sweep, neither of which looks past the region
// grown by one. So this one takes its own rectangle rather than `SnapshotReadRect`'s
// margins, which exist for the displacement sweeps' much longer reach.
ONI_SNAPSHOT_NOINLINE inline void RefreshTemperatureSnapshot(std::vector<float>& snap,
                                       const std::vector<PhaseEntry>& cells, size_t ri,
                                       int32_t pw, int32_t ph, int32_t rx0, int32_t ry0,
                                       int32_t rx1, int32_t ry1) {
  const size_t n = cells.size();
  auto gather = [&](size_t b, size_t e) {
    for (size_t i = b; i < e; ++i) snap[i] = cells[i].temperature;
  };
  if (SnapshotNeedsWholeGrid(snap.size(), n, ri)) {
    snap.resize(n);
    gather(0, n);
    return;
  }
  SnapshotRect q;
  q.x0 = rx0;
  q.y0 = ry0;
  q.x1 = rx1 + 1 < pw ? rx1 + 1 : pw;
  q.y1 = ry1 + 1 < ph ? ry1 + 1 : ph;
  ForEachSnapshotRow(pw, q, gather);
}

// The thermal energy one grid cell is holding, in kilojoules, or 0 for a cell the energy
// ledger cannot see.
//
// This MUST agree exactly with `World::TotalGridEnergy` above -- that walk is field 0 of the
// ledger and every per-cell charge in the sim is measured against it, so any difference
// between the two formulas would read as permanent, unexplainable drift rather than as the
// bug in the instrument that it would be. Two things follow from that and neither is
// cosmetic:
//
//   * the `mass <= 0` skip is copied, so a negative-mass cell contributes zero to both
//     sides rather than a negative number to one of them;
//   * the padding ring contributes nothing, because `TotalGridEnergy` walks `GameCells()`
//     only. Charging a write into the border would invent drift out of nothing.
//     `GameIndex` comes back negative or past the end for exactly those cells, which is the
//     test Klei's own event code uses.
inline double GridCellEnergy(const World& w, const ElementTable& table, size_t padded) {
  if (!w.ValidGameCell(w.GameIndex(padded))) return 0.0;
  const PhaseEntry& c = w.Phase(padded);
  if (c.mass <= 0.0f) return 0.0;
  return static_cast<double>(c.mass) *
         static_cast<double>(table.At(c.element).specificHeatCapacity) *
         static_cast<double>(c.temperature);
}

}  // namespace oni_sim
