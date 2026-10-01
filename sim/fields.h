// THE GENERIC FIELD SOLVER: a per-cell scalar, a propagation rule and per-material
// attenuation, all declared as parameters rather than written as code. The ABI contract is
// in `abi/sim_abi_ext.h` at `kRegisterField`. Published as the phase `StepFields`.
//
// A FIELD IS A PROPERTY THAT HAS BEEN GIVEN A SOLVER. It owns no storage: it is a registered
// `kExtF32`, arity-1 per-cell property (`ext_registry.h`) plus a rule attached to it by index.
// Everything a field needs except the stepping already existed -- naming and duplicate
// refusal, the persistence class, the save blob, the checkpoint, the zero-copy publish, the
// managed read and write paths, and the rehydration accounting. This file adds the one thing
// that was missing: something that writes the field every substep.
//
// IT IS NOT A NEW SET OF WALKS EITHER. Every rule here is one of the tree's own, re-cut to
// take its attenuation from a parameter:
//
//   * `kSourcePoint`       is `tickConstant` (radiation.h:473) -- an ellipse with a linear
//                          falloff and an optional cone, attenuated along a ray to each
//                          target -- minus the noise (see NOISE below).
//   * `kSourceDirectional` is `ComputeSunBeam` (textures.h:337) -- parallel lanes swept
//                          across one world -- in padded coordinates against the live grid
//                          rather than in game coordinates against the projection, because
//                          this runs inside the substep and that runs at publish time.
//   * `kSourceElement`     is the per-element source of `StepRadiationField` without Klei's
//                          function-local 5x5 stencil, which is radiation's shape and not a
//                          general one.
//   * `kAttenRadiationMass` is `RadiationAbsorption`; `kAttenLightMass` is `ComputeSunBeam`'s
//                          per-cell step. Both are bit-exactness-gated against the shipped
//                          game, and `gastest` asserts that a field configured as either one
//                          reproduces it -- which is what makes copying the walk safe.
//
// THE TWO INVARIANTS, both load-bearing:
//
// READ ANY CELL, WRITE ONLY INSIDE THE REGION RECT. Klei's own emitter already works this
// way. Ordering ACROSS regions is not guaranteed, which keeps region-parallel stepping
// possible, and a cross-region READ is
// compatible with that where a write is not. Every write in this file goes through
// `FieldAdd`, which takes the rect; that is deliberate, so that a new rule cannot forget.
//
// NOISE, AND WHY THERE IS NONE. `tickConstant` draws from the world LCG, and only for the
// outer three quarters of its ellipse -- so an emitter's GEOMETRY decides how many draws the
// world makes, and the gas shuffle downstream of it moves when they do. A field doing that
// would mean any mod registering any field shifts vanilla's random stream, which turns a
// diffsim-green DLL divergent the moment a mod loads. Nothing in this file calls
// `World::NextRandomState()`, and a future noise term gets a private LCG instead.
//
// THE ATTENUATION TABLE IS DENSE AND REBUILT PER PASS. The per-element attribute registry is
// SPARSE and keyed by SimHashes id, so reading it is a `lower_bound` -- fine per element,
// wrong per cell. `FieldAttenuation` resolves the whole table into one float per element-table
// index before a pass starts (a couple of hundred lookups) and the kernels index it directly.
#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "../abi/sim_abi_ext.h"
#include "census.h"
#include "ext_elements.h"
#include "ext_registry.h"
#include "radiation.h"
#include "world.h"

namespace oni_sim::ext {

// One registered source of one field. `target` is stored RESOLVED: a point source keeps the
// padded cell, an element source keeps the element table index, a directional source keeps the
// world index -- so the per-substep pass never re-does a conversion or a hash lookup.
struct FieldSource {
  int32_t id = 0;
  int32_t kind = kSourcePoint;
  float strength = 0.0f;
  int32_t target = 0;
  int32_t radius_x = 0;
  int32_t radius_y = 0;
  float cone_direction = 0.0f;
  float cone_angle = 360.0f;
  float dir_x = 0.0f;
  float dir_y = 0.0f;
};

struct Field {
  int32_t property = -1;
  int32_t attribute = -1;
  float fallback = 0.0f;
  int32_t law = kAttenFlat;
  int32_t combine = kCombineTransmission;
  int32_t decay_mode = kDecayNone;
  float decay_keep = 1.0f;
  float floor_value = 0.0f;
  float clamp_lo = 0.0f;
  float clamp_hi = 1.0f;
  std::vector<FieldSource> sources;

  // Nothing to do this substep: no decay to apply and nothing feeding it. The field still
  // holds its values and a mod can still read and write them -- it simply is not stepped, and
  // the whole-grid pass never runs, so a registered but idle field costs nothing per substep.
  bool Idle() const { return decay_mode == kDecayNone && sources.empty(); }
};

class FieldRegistry {
 public:
  static constexpr int32_t kMaxFields = kExtMaxFields;
  static constexpr int32_t kMaxSources = kExtMaxFieldSources;

  // Returns the field index (>= 0), or a NEGATED `ExtRegisterResult`. `error` is filled on
  // every refusal, with the reason spelled out -- three distinct codes rather than one,
  // because "registration failed" is not actionable and "that property is arity 4" is.
  int32_t Register(const CellPropertyRegistry& props, const ElementAttributeRegistry& attrs,
                   const RegisterFieldMessage& m, std::string* error) {
    auto refuse = [&](ExtRegisterResult code, const std::string& msg) {
      if (error) *error = msg;
      return -static_cast<int32_t>(code);
    };
    const CellPropertyRegistry::Property* p = props.At(m.propertyIdx);
    if (p == nullptr) {
      return refuse(kExtRegisterBadProperty,
                    "field: no cell property with index " + std::to_string(m.propertyIdx));
    }
    if (p->type != kExtF32 || p->arity != 1) {
      return refuse(kExtRegisterBadProperty,
                    "field \"" + p->name + "\": a field must live in an F32 property of arity "
                    "1; this one is type " + std::to_string(p->type) + " arity " +
                        std::to_string(p->arity));
    }
    if (m.attributeIdx != -1 && attrs.At(m.attributeIdx) == nullptr) {
      return refuse(kExtRegisterBadAttribute,
                    "field \"" + p->name + "\": no element attribute with index " +
                        std::to_string(m.attributeIdx) + " (use -1 for no attenuation)");
    }
    if (m.law != kAttenFlat && m.law != kAttenRadiationMass && m.law != kAttenLightMass) {
      return refuse(kExtRegisterBadRule,
                    "field \"" + p->name + "\": unknown attenuation law " +
                        std::to_string(m.law));
    }
    if (m.combine != kCombineTransmission && m.combine != kCombineExposure) {
      return refuse(kExtRegisterBadRule,
                    "field \"" + p->name + "\": unknown combine mode " +
                        std::to_string(m.combine));
    }
    if (m.decayMode != kDecayNone && m.decayMode != kDecayFactor) {
      return refuse(kExtRegisterBadRule,
                    "field \"" + p->name + "\": unknown decay mode " +
                        std::to_string(m.decayMode));
    }
    // A NaN fails both of these, which is the right answer for it.
    if (!(m.decayKeep >= 0.0f && m.decayKeep <= 1.0f)) {
      return refuse(kExtRegisterBadRule,
                    "field \"" + p->name + "\": decay factor must be in [0, 1]");
    }
    if (!(m.clampLo <= m.clampHi)) {
      return refuse(kExtRegisterBadRule,
                    "field \"" + p->name + "\": clamp low is above clamp high");
    }
    const int32_t existing = FindByProperty(m.propertyIdx);
    if (existing >= 0) {
      return refuse(kExtRegisterDuplicate,
                    "field \"" + p->name + "\" already has a solver (field index " +
                        std::to_string(existing) + "); one field per property");
    }
    if (static_cast<int32_t>(fields_.size()) >= kMaxFields) {
      return refuse(kExtRegisterFull, "field \"" + p->name + "\": registry is full (" +
                                          std::to_string(kMaxFields) + " fields)");
    }
    Field f;
    f.property = m.propertyIdx;
    f.attribute = m.attributeIdx;
    f.fallback = m.attributeFallback;
    f.law = m.law;
    f.combine = m.combine;
    f.decay_mode = m.decayMode;
    f.decay_keep = m.decayKeep;
    f.floor_value = m.floorValue;
    f.clamp_lo = m.clampLo;
    f.clamp_hi = m.clampHi;
    fields_.push_back(f);
    return static_cast<int32_t>(fields_.size()) - 1;
  }

  int32_t Count() const { return static_cast<int32_t>(fields_.size()); }
  bool Valid(int32_t idx) const {
    return idx >= 0 && static_cast<size_t>(idx) < fields_.size();
  }
  const Field* At(int32_t idx) const {
    return Valid(idx) ? &fields_[static_cast<size_t>(idx)] : nullptr;
  }
  std::vector<Field>& Data() { return fields_; }
  const std::vector<Field>& Data() const { return fields_; }

  int32_t FindByProperty(int32_t property_idx) const {
    for (size_t i = 0; i < fields_.size(); ++i) {
      if (fields_[i].property == property_idx) return static_cast<int32_t>(i);
    }
    return -1;
  }

  // Add, replace or remove one source. `strength == 0` removes -- a source contributing
  // nothing and a source that is not there are the same thing to the solver. Returns false
  // for an unknown field or a full source list, which the caller reports as a refusal rather
  // than dropping: a field index is not guessable, so a bad one is a caller bug.
  bool SetSource(int32_t field_idx, const FieldSource& s) {
    if (!Valid(field_idx)) return false;
    std::vector<FieldSource>& list = fields_[static_cast<size_t>(field_idx)].sources;
    for (size_t i = 0; i < list.size(); ++i) {
      if (list[i].id != s.id) continue;
      if (s.strength == 0.0f) {
        list.erase(list.begin() + static_cast<long>(i));
      } else {
        list[i] = s;
      }
      return true;
    }
    if (s.strength == 0.0f) return true;  // removing one that is not there is not an error
    if (static_cast<int32_t>(list.size()) >= kMaxSources) return false;
    list.push_back(s);
    return true;
  }

  // A world going away takes nothing with it: a field's rule and its sources belong to the mod
  // that registered them, not to the world, and the mod re-pushes them after a load. So there
  // is deliberately no `Clear()` called from `World::Allocate` here -- that would silently
  // discard a registration its mod had every reason to think was still in force.

 private:
  std::vector<Field> fields_;
};

// ---------------------------------------------------------------------------- attenuation

// One float per element-table index, resolved once per pass. `Build` is cheap (a couple of
// hundred sparse lookups); the kernels then index it with the cell's element.
struct FieldAttenuation {
  std::vector<float> by_element;

  void Build(const ElementTable& t, const ElementAttributeRegistry& attrs, const Field& f) {
    const int32_t n = t.Count();
    by_element.assign(static_cast<size_t>(n < 0 ? 0 : n), f.fallback);
    if (f.attribute < 0) {
      // No attenuation at all: every cell is perfectly transparent, whatever the fallback
      // says. A field with no attribute asked for no attenuation, not for the fallback
      // applied everywhere.
      for (size_t i = 0; i < by_element.size(); ++i) by_element[i] = 0.0f;
      return;
    }
    for (int32_t i = 0; i < n; ++i) {
      float v = f.fallback;
      // UNSET IS A REAL ANSWER in registry 2, and the fallback is what the field declared for
      // it. Only an element carrying a value overrides it.
      attrs.ReadF32(f.attribute, t.At(static_cast<uint16_t>(i)).id, &v);
      by_element[static_cast<size_t>(i)] = v;
    }
  }
};

// The per-cell absorbed fraction, by the field's law. Clamped to [0, 1] with SSE min/max
// semantics (`ClampSS`), not with comparisons -- the radiation law can produce a NaN when
// the world's radiation max mass is zero, and the game's clamp returns 1.0 there where a
// comparison clamp would hand the walk a NaN.
inline float FieldAbsorbed(const World& w, const ElementTable& t, const FieldAttenuation& a,
                           const Field& f, size_t cell) {
  const uint16_t elem = w.Phases()[cell].element;
  float factor = elem < a.by_element.size() ? a.by_element[elem] : 0.0f;
  // The transparent-element shortcut, and it is DELIBERATELY NOT TAKEN FOR THE RADIATION LAW.
  // For the other two a zero factor provably produces a zero absorption, so skipping the
  // arithmetic changes nothing. The radiation law's arithmetic is Klei's, and Klei's is not
  // zero there in every case: with `RadiationMaxMass()` zero -- a tuning value the game SETS
  // over the ABI and can legitimately leave at zero -- the mass term is `0 * inf`, a NaN, and
  // `ClampSS` returns 1.0 for a NaN where this shortcut would return 0.0. That is the whole
  // difference between "a transparent cell" and "an opaque one", so the law that claims to BE
  // `RadiationAbsorption` takes the long path unconditionally and the gastest arm asserting
  // the two are equal is entitled to assert it without a precondition.
  if (factor == 0.0f && f.law != kAttenRadiationMass) return 0.0f;
  switch (f.law) {
    case kAttenRadiationMass: {
      // `RadiationAbsorption` (radiation.h:164) with the field's attribute in place of
      // `radiationAbsorptionFactor`. The property byte's sign bit is the constructed-tile
      // test, spelled exactly as it is there.
      if (static_cast<int8_t>(w.Properties()[cell]) < 0) {
        factor = factor * w.RadiationConstructedFactor();
      } else {
        factor = (w.Phases()[cell].mass / w.RadiationMaxMass()) * factor *
                     w.RadiationDensityWeight() +
                 factor * w.RadiationBaseWeight();
      }
      return ClampSS(factor, 0.0f, 1.0f);
    }
    case kAttenLightMass: {
      // `ComputeSunBeam`'s per-cell step (textures.h:364). A SOLID takes the whole factor
      // whatever it weighs -- that is Klei's own behaviour, measured on `sunglass`, not a
      // simplification -- and a fluid takes its filled fraction of it.
      const Element& el = t.At(elem);
      if ((el.state & kStateMask) == kStateSolid) return ClampSS(factor, 0.0f, 1.0f);
      const float mass = w.Phases()[cell].mass;
      if (!(el.maxMass > 0.0f) || !(mass > 0.0f)) return 0.0f;
      const float full = mass > el.maxMass ? el.maxMass : mass;
      return ClampSS(full / el.maxMass * factor, 0.0f, 1.0f);
    }
    default:
      return ClampSS(factor, 0.0f, 1.0f);
  }
}

// One step of a walk: fold this cell's absorption into the running carrier. The two modes are
// the tree's own two and they are NOT interchangeable -- radiation multiplies a transmission,
// sunlight subtracts from an exposure. Both start at 1 and stay in [0, 1].
inline void FieldCarry(const Field& f, float absorbed, float* carrier) {
  if (f.combine == kCombineExposure) {
    *carrier -= absorbed;
    if (*carrier <= 0.0f) *carrier = 0.0f;
  } else {
    *carrier = (1.0f - absorbed) * *carrier;
  }
}

// THE ONE WRITE PATH. Takes the rect so that a rule cannot forget the invariant, clamps to the
// field's declared range, and snaps to zero at the floor. `x`/`y` are padded coordinates.
inline void FieldAdd(float* values, const World::PaddedRect& r, int32_t pw, const Field& f,
                     int32_t x, int32_t y, float amount) {
  if (x < r.x0 || y < r.y0 || x >= r.x1 || y >= r.y1) return;
  const size_t cell = static_cast<size_t>(y) * static_cast<size_t>(pw) +
                      static_cast<size_t>(x);
  float v = values[cell] + amount;
  if (v <= f.floor_value) v = 0.0f;
  if (v < f.clamp_lo) v = f.clamp_lo;
  if (v > f.clamp_hi) v = f.clamp_hi;
  values[cell] = v;
  // A FIELD ANNOUNCES NOTHING, so the census cannot pick its changes up at
  // `World::TouchSubstance` the way it does for every kernel that moves substance -- a field
  // writes an extension property and the game reads it zero-copy. Counted here instead, at
  // the one write path.
  //
  // IT COUNTS WRITES PERFORMED, NOT BITS MOVED, and that is the same definition `census.h`
  // states for every other slot: `changes` is what a sweep ANNOUNCED, a count of announcements
  // and not of distinct cells, which is why it may legitimately exceed `examined`. A source
  // re-writing a cell already pinned at `clamp_hi` did the work either way, and the reader is
  // told about saturation separately -- `bench`'s per-field summary says when a field has
  // reached its clamp.
  //
  // The first version compared the stored bits before and after instead. It was exact and it
  // was WRONG TO SHIP: measured on the decay pass, which is the cheapest rule and the one
  // whose body a per-cell compare actually shows up in, 0.056 ms a substep became 0.096 --
  // a 71 % tax on the kernel the counter exists to describe. `census.h` is explicit that its
  // cost is bounded by construction, and an instrumentation that changes the answer is not
  // instrumentation.
  ONI_CHANGED(kFields);
}

// ------------------------------------------------------------------------------ the walks

// Klei's Bresenham loop header, for the THIRD time in this tree (`RadiationAbsorptionAlongLine`
// has it, `raycast.h` has it) and deliberately copied rather than shared: the other two are
// gated bit-for-bit against the shipped game, and a refactor that touched them to serve this
// would put that gate at risk for tidiness. What keeps the copy honest is `gastest`, which
// asserts that this function with the radiation law equals `RadiationAbsorptionAlongLine` over
// a few hundred random lines. Change one and the others have to change with it, and the suite
// will say so.
//
// Both endpoints are visited, as in Klei's walk: a source buried in lead attenuates itself.
inline float FieldAttenuationAlongLine(const World& w, const ElementTable& t,
                                       const FieldAttenuation& a, const Field& f, int32_t x0,
                                       int32_t y0, int32_t x1, int32_t y1) {
  const float fx0 = static_cast<float>(x0), fy0 = static_cast<float>(y0);
  const float fx1 = static_cast<float>(x1), fy1 = static_cast<float>(y1);
  const bool along_x = std::fabs(fy1 - fy0) <= std::fabs(fx1 - fx0);
  const float major_a = along_x ? fx0 : fy0;
  const float major_b = along_x ? fx1 : fy1;
  const float minor_a = along_x ? fy0 : fx0;
  const float minor_b = along_x ? fy1 : fx1;

  float from = minor_a, to = minor_b;
  if (major_b < major_a) {
    from = minor_b;
    to = minor_a;
  }
  const float lo = major_b <= major_a ? major_b : major_a;
  const float hi = major_a <= major_b ? major_b : major_a;
  const float span = hi - lo;
  const float delta = std::fabs(to - from);
  float err = span * 0.5f;
  const int32_t step = from < to ? 1 : -1;
  const int32_t pw = w.PaddedWidth();

  float carrier = 1.0f;
  int32_t minor = static_cast<int32_t>(from);
  const int32_t last = static_cast<int32_t>(hi);
  // COUNTED IN A LOCAL AND CHARGED ONCE, which is the only way to obey both of `census.h`'s
  // rules here. It says the counter has to RIDE THE ITERATION it describes -- a number derived
  // from the loop's own bounds agrees with the loop even when the loop is wrong, which is the
  // defect the whole file exists to catch -- and it says `ONI_EXAMINED` is added once per ROW
  // and never once per cell. A walk has no rows, so the step counter is a register increment
  // in the loop and one global add after it: the count moves when the iteration moves, and the
  // census still costs one add per walk rather than one per step. Proved by planting exactly
  // that fault: a ray shortened by one step leaves a bounds-derived count identical.
  int64_t stepped = 0;
  for (int32_t major = static_cast<int32_t>(lo); major <= last; ++major) {
    ++stepped;
    const int32_t x = along_x ? major : minor;
    const int32_t y = along_x ? minor : major;
    const size_t cell = static_cast<size_t>(y) * static_cast<size_t>(pw) +
                        static_cast<size_t>(x);
    FieldCarry(f, FieldAbsorbed(w, t, a, f, cell), &carrier);
    const float next = err - delta;
    err = next < 0.0f ? next + span : next;
    if (next < 0.0f) minor += step;
  }
  ONI_EXAMINED(kFields, stepped);
  if (carrier >= 1.0f) carrier = 1.0f;
  if (carrier <= 0.0f) carrier = 0.0f;
  return carrier;
}

// ---- kSourcePoint. `tickConstant` (radiation.h:473) without the noise and with the field's
// own attenuation. The scan box is Klei's ellipse test done honestly -- `2*radiusY + 1` rows
// rather than Klei's `2*radiusX`, because that square-in-radiusX box is a Klei quirk
// documented in radiation.h and inheriting it would clip a tall field for no reason.
inline void StepFieldPoint(const World& w, const ElementTable& t, const FieldAttenuation& a,
                           const Field& f, const FieldSource& s, float* values,
                           const World::PaddedRect& r) {
  const int32_t pw = w.PaddedWidth(), ph = w.PaddedHeight();
  const int32_t rx = s.radius_x, ry = s.radius_y;
  if (rx <= 0 || ry <= 0) return;
  const int32_t cy = s.target / pw, cx = s.target % pw;
  const float frx = static_cast<float>(rx), fry = static_cast<float>(ry);
  const float rx2 = frx * frx, ry2 = fry * fry;

  // The row the loop below actually steps over, clamped the way the loop's own `continue`s
  // clamp it, counted once per row. The per-cell tests inside -- the region rect, the cone,
  // the ellipse, the falloff -- are work this scan formed and then threw away; they are cells
  // examined, not cells skipped, and `ONI_SKIPPED` is reserved for a RUN the kernel steps
  // over without visiting (see `census.h`), which this is not.
  const int32_t scan_x0 = cx - rx < 1 ? 1 : cx - rx;
  const int32_t scan_x1 = cx + rx >= pw ? pw - 1 : cx + rx;
  const int64_t scan_row = scan_x1 >= scan_x0 ? scan_x1 - scan_x0 + 1 : 0;

  for (int32_t y = cy - ry; y <= cy + ry; ++y) {
    if (y <= 0 || y >= ph) continue;
    ONI_EXAMINED(kFields, scan_row);
    for (int32_t x = cx - rx; x <= cx + rx; ++x) {
      if (x <= 0 || x >= pw) continue;
      // The region test is on the WRITE, and it is the only one: the ray below reads whatever
      // it crosses, region or not.
      if (x < r.x0 || y < r.y0 || x >= r.x1 || y >= r.y1) continue;
      if (!InRadialRange(cx, cy, x, y, s.cone_angle, s.cone_direction)) continue;
      const float dx = static_cast<float>(x - cx);
      const float dy = static_cast<float>(y - cy);
      if (!((dy * dy) / ry2 + (dx * dx) / rx2 <= 1.0f)) continue;
      // Klei's falloff, which is a diamond rather than a radius: linear in |dx| along the
      // major axis and tapered by |dy| with the cross term that keeps the corners finite.
      const float inv_rx = 1.0f / frx;
      const float falloff = (1.0f - std::fabs(dx) * inv_rx) -
                            (std::fabs(dy) - std::fabs(dy * dx) * inv_rx) / fry;
      if (!(falloff > 0.0f)) continue;
      const float carrier = FieldAttenuationAlongLine(w, t, a, f, cx, cy, x, y);
      FieldAdd(values, r, pw, f, x, y, carrier * falloff * s.strength);
    }
  }
}

// ---- kSourceDirectional. `ComputeSunBeam`'s lane sweep (textures.h:337), in PADDED
// coordinates over the live grid.
//
// THE RAYS ARE LANES, NOT RAYCASTS. Every ray entering a world is parallel, so a cell belongs
// to exactly one of them -- the one that crossed the world's source-side edge at a position
// the cell's distance from that edge fixes. Walking away from the source and keeping one
// carrier per lane visits each cell once and interpolates nothing, which is both the cheapest
// walk and the one whose zero-offset case IS a column.
//
// The sweep walks the WHOLE world, including the parts outside the region rect, because a
// lane's carrier depends on everything it has already crossed; only the writes are clipped.
// That is the read-anywhere/write-in-region invariant in its clearest form.
inline void StepFieldDirectional(const World& w, const ElementTable& t,
                                 const FieldAttenuation& a, const Field& f,
                                 const FieldSource& s, float* values,
                                 const World::PaddedRect& r, std::vector<float>* lanes) {
  const std::vector<World::WorldOffset>& worlds = w.WorldOffsets();
  if (s.target < 0 || static_cast<size_t>(s.target) >= worlds.size()) return;
  // At or below the horizon nothing arrives, and a NaN fails this test too.
  if (!(s.dir_y > 0.0f)) return;

  const World::WorldOffset& world = worlds[static_cast<size_t>(s.target)];
  const int32_t gw = w.GameWidth(), gh = w.GameHeight();
  const int32_t gx0 = world.x < 0 ? 0 : world.x;
  const int32_t gx1 = world.x + world.w > gw ? gw : world.x + world.w;
  const int32_t gy0 = world.y < 0 ? 0 : world.y;
  const int32_t gy1 = world.y + world.h > gh ? gh : world.y + world.h;
  if (gx1 <= gx0 || gy1 <= gy0) return;
  // Game to padded is a shift of one in each axis. The world rectangle is game-space; every
  // cell walked below is padded.
  const int32_t x0 = gx0 + 1, x1 = gx1 + 1, y0 = gy0 + 1, y1 = gy1 + 1;
  const int32_t width = x1 - x0, height = y1 - y0;
  const int32_t pw = w.PaddedWidth();

  // One cell, one lane: write what arrives, then take what the cell absorbs. A lane that has
  // reached zero stays there.
  auto pass = [&](float& carrier, int32_t x, int32_t y) {
    if (carrier <= 0.0f) return;
    FieldAdd(values, r, pw, f, x, y, carrier * s.strength);
    const size_t cell = static_cast<size_t>(y) * static_cast<size_t>(pw) +
                        static_cast<size_t>(x);
    FieldCarry(f, FieldAbsorbed(w, t, a, f, cell), &carrier);
  };

  const float ax = s.dir_x < 0.0f ? -s.dir_x : s.dir_x;
  if (ax <= s.dir_y) {
    // Rows, top down. `slope` is how far toward the source a lane's entry point lies per row
    // climbed, so a cell `k` rows below the top sits on the lane that entered at x + k*slope.
    const double slope = static_cast<double>(s.dir_x) / static_cast<double>(s.dir_y);
    const int32_t last = static_cast<int32_t>(std::floor(slope * (height - 1) + 0.5));
    const int32_t omin = last < 0 ? last : 0;
    const int32_t omax = last > 0 ? last : 0;
    lanes->assign(static_cast<size_t>(width + omax - omin), 1.0f);
    for (int32_t y = y1 - 1; y >= y0; --y) {
      ONI_EXAMINED(kFields, width);
      const int32_t offset =
          static_cast<int32_t>(std::floor(slope * (y1 - 1 - y) + 0.5)) - omin;
      for (int32_t x = x0; x < x1; ++x) {
        pass((*lanes)[static_cast<size_t>(x - x0 + offset)], x, y);
      }
    }
  } else {
    // Columns, from the source's side inwards. A cell `k` columns in from that edge sits on
    // the lane that entered it `k * drop` rows higher. `drop` is positive by the `dir_y > 0`
    // test above, so there is no negative offset to account for here.
    const double drop = static_cast<double>(s.dir_y) / static_cast<double>(ax);
    const int32_t omax = static_cast<int32_t>(std::floor(drop * (width - 1) + 0.5));
    lanes->assign(static_cast<size_t>(height + omax), 1.0f);
    const bool from_right = s.dir_x > 0.0f;
    for (int32_t k = 0; k < width; ++k) {
      // A column is this branch's row: one add per lane walked, same rule as above.
      ONI_EXAMINED(kFields, height);
      const int32_t x = from_right ? x1 - 1 - k : x0 + k;
      const int32_t offset = static_cast<int32_t>(std::floor(drop * k + 0.5));
      for (int32_t y = y0; y < y1; ++y) {
        pass((*lanes)[static_cast<size_t>(y - y0 + offset)], x, y);
      }
    }
  }
}

// ---- kSourceElement. Every cell of one element emits per kilogram of its own mass, into its
// own cell. No stencil: Klei's 5x5 weight map is radiation's shape, and a general solver that
// inherited it would be spraying a shape nobody asked for.
inline void StepFieldElement(const World& w, const Field& f, const FieldSource& s,
                             float* values, const World::PaddedRect& r) {
  const int32_t pw = w.PaddedWidth();
  const uint16_t elem = static_cast<uint16_t>(s.target);
  for (int32_t y = r.y0; y < r.y1; ++y) {
    ONI_EXAMINED(kFields, r.x1 - r.x0);
    for (int32_t x = r.x0; x < r.x1; ++x) {
      const size_t cell = static_cast<size_t>(y) * static_cast<size_t>(pw) +
                          static_cast<size_t>(x);
      if (w.Phases()[cell].element != elem) continue;
      const float mass = w.Phases()[cell].mass;
      if (!(mass > 0.0f)) continue;
      FieldAdd(values, r, pw, f, x, y, mass * s.strength);
    }
  }
}

// ---- decay. The only whole-region pass a field runs on its own account, which is why
// `kDecayNone` is the default and why `Field::Idle()` exists.
inline void StepFieldDecay(const World& w, const Field& f, float* values,
                           const World::PaddedRect& r) {
  const int32_t pw = w.PaddedWidth();
  for (int32_t y = r.y0; y < r.y1; ++y) {
    ONI_EXAMINED(kFields, r.x1 - r.x0);
    // COUNTED HERE AND NOT BY `FieldAdd`, because decay is the one write in this file that
    // does not go through it: it replaces a value rather than adding to one, so there is
    // nothing to clip against the rect (the loop IS the rect) and no amount to clamp beyond
    // what the three lines below do. The first version of this census left the count to
    // `FieldAdd` alone, and duly reported a decay-only field as changing ZERO cells while it
    // moved every cell of the region every substep -- a whole-region sweep whose own numbers
    // said it did nothing.
    //
    // A LOCAL, ADDED ONCE PER ROW, and it rides the `v == 0.0f` branch the kernel was already
    // taking rather than adding a test of its own -- which is `census.h`'s rule in both its
    // halves: one global add per row, and a counter that sits on a branch the kernel already
    // predicts. What it counts is what `FieldAdd` counts, cells written, so the two writers
    // in this file report one slot in one unit.
    //
    // Measured, because this file's history is the argument for measuring it: the version
    // that compared each cell's bits before and after cost 0.096 ms a substep against 0.056
    // for the pass with no counter at all. This one is inside the noise of 0.056.
    int64_t written = 0;
    for (int32_t x = r.x0; x < r.x1; ++x) {
      const size_t cell = static_cast<size_t>(y) * static_cast<size_t>(pw) +
                          static_cast<size_t>(x);
      float v = values[cell];
      if (v == 0.0f) continue;
      ++written;
      v = v * f.decay_keep;
      if (v <= f.floor_value) v = 0.0f;
      if (v < f.clamp_lo) v = f.clamp_lo;
      if (v > f.clamp_hi) v = f.clamp_hi;
      values[cell] = v;
    }
    ONI_CHANGED_N(kFields, written);
  }
}

// Scratch the phase owns, so a substep allocates nothing once it has run once.
struct FieldState {
  FieldRegistry registry;
  FieldAttenuation attenuation;
  std::vector<float> lanes;
};

// THE PHASE. `kPhaseFields`, region-scoped, gated: a world with no registered field, or one
// whose every field is idle, does no work beyond the loop header.
//
// Decay first, then sources in registration order -- the order a caller registered them in is
// the order they are applied, which is the only one that is predictable from outside.
inline void StepFields(World* w, const ElementTable& t, CellPropertyRegistry* props,
                       const ElementAttributeRegistry& attrs, FieldState* state, size_t ri) {
  // AT THE TOP, ahead of the gate, because the gate is the thing the count is asked about:
  // a stock world registers no field, and "the phase was entered 500 times and examined no
  // cells" is the shape that says the early-out is doing its job. A scope, so the writes
  // `FieldAdd` makes three functions down are charged here without those functions knowing
  // anything about the census.
  ONI_SWEEP(kFields);
  std::vector<Field>& fields = state->registry.Data();
  if (fields.empty()) return;
  const World::PaddedRect& r = w->PaddedRegion(ri);
  if (r.x1 <= r.x0 || r.y1 <= r.y0) return;

  for (size_t i = 0; i < fields.size(); ++i) {
    Field& f = fields[i];
    if (f.Idle()) continue;
    float* values = props->MutableF32(f.property);
    if (values == nullptr) continue;  // the world has not been allocated yet

    if (f.decay_mode != kDecayNone) StepFieldDecay(*w, f, values, r);
    if (f.sources.empty()) continue;

    state->attenuation.Build(t, attrs, f);
    for (size_t si = 0; si < f.sources.size(); ++si) {
      const FieldSource& s = f.sources[si];
      switch (s.kind) {
        case kSourcePoint:
          StepFieldPoint(*w, t, state->attenuation, f, s, values, r);
          break;
        case kSourceDirectional:
          StepFieldDirectional(*w, t, state->attenuation, f, s, values, r, &state->lanes);
          break;
        case kSourceElement:
          StepFieldElement(*w, f, s, values, r);
          break;
        default:
          break;
      }
    }
    // A source wrote through `FieldAdd`, which clamps, so there is no trailing clamp pass
    // here -- a second whole-grid sweep for a bound every write already held.
    props->NoteNonzero(f.property);
  }
}

}  // namespace oni_sim::ext
