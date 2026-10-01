// Physics kernels.
//
// Every constant and ordering here was established by measuring the game's own sim, and is
// checked against it by `diffsim`.

#pragma once

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstring>
#include <vector>

#include "census.h"
#include "disease.h"
#include "gas_rooms.h"
#include "tunables.h"
#include "world.h"

namespace oni_sim {



// The sim does not run on the frame's elapsed time. It runs a **fixed 0.2 s substep**, and
// a longer frame runs more of them.
//
// Measured directly: a frame of 0.05 s, 0.15 s and 0.2 s all produce exactly the same
// result, and a frame of 0.4 s produces exactly the result of two 0.2 s frames. A rate
// cannot do that. The leftover time carries into the next frame — at dt = 0.3 the sim
// alternates one substep and two.
//
// This corrected an error in the conduction kernel, which was written as `dQ = k*dT*dt`
// with the frame's dt. That is right only because the game happens to send 0.2 s; on any
// other frame length it was wrong, and it was flow's dt sweep that exposed it.
// The tunable `SubstepSeconds` (tunables.def).

// How many substeps a frame of `elapsed` seconds runs, carrying the remainder.
//
// A frame always runs at least one, even when it is shorter than a substep: at dt = 0.05
// the sim still steps every frame rather than every fourth. Whether Klei clamps the
// elapsed time up to 0.2 or forces a minimum of one substep cannot be told apart from
// outside, and the two agree on every frame length the game actually sends.
inline int SubstepsForFrame(float elapsed, float* carry) {
  const float substep = g_tunables.substep_seconds;  // once a frame, outside every kernel
  *carry += elapsed;
  int n = static_cast<int>(*carry / substep);
  if (n <= 0) {
    *carry = 0.0f;
    return 1;
  }
  *carry -= n * substep;
  return n;
}

// Measured against isolated cell pairs in a vacuum world (nothing else can conduct):
//
//   dQ = sqrt(k_a * k_b) * (T_a - T_b) * dt        kilojoules
//
// per orthogonal neighbour pair, per frame. Three things about it that are not obvious
// and were all wrong in my first guesses:
//
//   * The pair conductivity is the **geometric mean**, not the minimum and not the
//     arithmetic mean. granite(3.39)+sandstone(2.9) moved 62.707 kJ against a 100 K
//     difference, implying 3.1354; sqrt(3.39*2.9) = 3.1354. granite+copper(60) implied
//     14.2616 against sqrt(3.39*60) = 14.2618. Both to five significant figures.
//   * It is **completely independent of mass**. The same pair moved an identical
//     67.794 kJ with the receiving cell at 250, 1000 and 2000 kg. Mass only decides how
//     far the temperature moves afterwards, through the heat capacity.
//   * There is no area or distance term at all — no dx, no cell size.
//
// Cell insulation is modelled, and it is not a simple attenuation — see
// `PairConductivityInsulated` below and `kInsulationScale`.
//
// Conduction has a dead zone: pairs closer together than this exchange nothing at all.
//
// Measured by walking an isolated pair's temperature difference down. At 2 K it transfers
// every frame; at exactly 1 K it transfers once and then stops, because that single
// transfer drops the difference to 0.9983; at 0.5 K and below it never transfers at all.
//
// This is not a rounding artefact, it is load-bearing behaviour. It is why a copper block
// in granite keeps its interior at *exactly* its starting temperature while its edges
// cool, and it is why temperatures in ONI settle a fraction of a degree apart and never
// finish equalising. A replacement without it equalises everything smoothly and drifts
// away from Klei within a handful of frames.
// The tunable `MinConductionDelta` (tunables.def).

// Conduction skips whole tiles whose temperatures cannot span the dead zone. 16 is a guess to
// be measured, not a result: wide enough that the summary costs little per cell, narrow enough
// that one live cell does not hold a large area open. The edge is the tunable
// `ConductionTileShift` (tunables.def); `StepConduction` sizes its tile arrays on every call,
// so a new shift needs no reallocation of its own. `kHugeTemperature` is the scan's sentinel,
// not a quantity, and stays a constant.
inline constexpr float kHugeTemperature = 1e30f;

// A cell's surface-area multiplier is chosen by the phase of the cell it is touching, and
// **both cells' multipliers apply**.
//
//   k_pair = sqrt(k_a * k_b) * SAM_a[phase of b] * SAM_b[phase of a]
//
// Measured after the first conduction pass wrongly left fluids out. Oxygen carries
// solidSurfaceAreaMultiplier 25, and one oxygen cell sealed in granite conducted at
// exactly 25x the geometric mean — not 5x, which is what folding the 25 into oxygen's own
// conductivity before taking the square root would give. A 3x3 water pocket, where the
// centre cell touches only water and both sides contribute their liquid multiplier of 25,
// conducted at 625x. That is the measurement that says the two multiply rather than the
// larger one winning.
inline float SurfaceAreaMultiplier(const Element& e, uint8_t other_phase) {
  switch (other_phase) {
    case kStateSolid: return e.solidSurfaceAreaMultiplier;
    case kStateLiquid: return e.liquidSurfaceAreaMultiplier;
    case kStateGas: return e.gasSurfaceAreaMultiplier;
    default: return 1.0f;
  }
}

// The geometric mean is not computed as a square root. The game takes `logf` of the *second*
// cell's conductivity, then of the first, adds them in that order, halves the sum and calls
// `expf`:
//
//   k = expf(0.5f * (logf(k_b) + logf(k_a)))
//
// which is `sqrt(k_a * k_b)` to within a last bit or two and not always to within zero. That
// sounds like a detail worth rounding away and it is not: `GasShuffle` keeps the *coolest*
// candidate, so one ulp of temperature anywhere in a gas room eventually picks a different
// cell to swap, and from there the two sims are running different worlds. With a square root,
// every scenario with gas and a gradient in it is one ulp adrift within three ticks.
//
// The `logf`/`expf` pair is only reached when both cells are at full insulation, which is
// why this function does not read insulation at all: see `PairConductivityInsulated`.
inline float PairConductivityUncached(const ElementTable& table, uint16_t a, uint16_t b) {
  const Element& ea = table.At(a);
  const Element& eb = table.At(b);
  const float ka = ea.thermalConductivity;
  const float kb = eb.thermalConductivity;
  if (ka <= 0.0f || kb <= 0.0f) return 0.0f;
  // Each cell's multiplier is taken against the other cell's phase, and the pair is
  // multiplied together *before* it is applied to the mean, as the game does.
  const float sa = SurfaceAreaMultiplier(ea, table.Phase(b));
  const float sb = SurfaceAreaMultiplier(eb, table.Phase(a));
  const float geo = std::exp(0.5f * (std::log(kb) + std::log(ka)));
  return geo * (sb * sa);
}

// The geometric mean above is spelled `exp(0.5 * (log kb + log ka))` because that is what
// the game computes, and the last bit of `heatblock` depends on it — `sqrt(ka * kb)` is the
// same number in exact arithmetic and a different float. That spelling is also expensive:
// **7.6 ms of a 10 ms conduction sweep on the asteroid**, two `logf` and an `expf` on every
// pair that clears the dead zone, ~200,000 of them a substep. A settled world hides it,
// because it rejects every pair before reaching this.
//
// The value depends on nothing but the two element indices, and a world has ten or so
// elements in it. So compute each ordered pair once and read it back. This is bit-exact by
// construction rather than by measurement: same function, same inputs, called fewer times.
// The table below is worth 60 % of the substep.
// `Bind` once per sweep and `Get` per pair, rather than going through the `thread_local`
// every time: `thread_local` on mingw is *emulated* TLS and every access is a function
// call, which at 200,000 pairs a substep was itself worth 1.9 ms — more than half of what
// the cache saves. The sweep holds a reference for the length of the call, the same way it
// already holds one to the `start` gather.
class PairConductivityCache {
 public:
  void Bind(const ElementTable& table) {
    if (table_ == &table && generation_ == table.Generation() && count_ == table.Count()) {
      return;
    }
    table_ = &table;
    generation_ = table.Generation();
    count_ = table.Count();
    k_.assign(static_cast<size_t>(count_) * static_cast<size_t>(count_), kUncomputed);
  }

  float Get(const ElementTable& table, uint16_t a, uint16_t b) const {
    if (static_cast<int32_t>(a) >= count_ || static_cast<int32_t>(b) >= count_) {
      return PairConductivityUncached(table, a, b);
    }
    float& slot = k_[static_cast<size_t>(a) * static_cast<size_t>(count_) + b];
    if (slot == kUncomputed) slot = PairConductivityUncached(table, a, b);
    return slot;
  }

 private:
  static constexpr float kUncomputed = -1.0f;  // every real value is 0 or positive
  const ElementTable* table_ = nullptr;
  uint32_t generation_ = 0;
  int32_t count_ = 0;
  mutable std::vector<float> k_;
};

inline PairConductivityCache& ThreadPairConductivity() {
  static thread_local PairConductivityCache cache;
  return cache;
}

// The convenience spelling, for anything that is not the conduction sweep.
inline float PairConductivity(const ElementTable& table, uint16_t a, uint16_t b) {
  PairConductivityCache& cache = ThreadPairConductivity();
  cache.Bind(table);
  return cache.Get(table, a, b);
}

// Cell insulation. Before the pair is formed at all, each cell's conductivity is
// scaled by its own `(insulation / 255)^2` — the constant is 1/255^2, read as
// 1.537870048196055e-05, and the byte is squared *before* it is scaled, so an insulated tile
// attenuates by the square of its value and not by the value.
//
// The part that makes this more than a multiplier: if *either* factor comes out below 1.0,
// the game abandons the geometric mean entirely and takes `min(f_a*k_a, f_b*k_b)` instead.
// The test is on the two raw factors, not on the scaled conductivities. So one insulated
// cell drags the whole pair down to the weaker side rather than to a mean, which is the
// difference between an insulated tile slowing heat down and an insulated tile stopping it.
//
// Both branches read the scaled conductivities. The mean branch is only ever reached with
// both factors at exactly 1.0, where `1.0f * k == k` bit for bit, which is what lets the
// element-pair cache above stay valid and bit-exact for every uninsulated pair in the world.
inline constexpr float kInsulationScale = 1.0f / (255.0f * 255.0f);

inline float InsulationFactor(uint8_t insulation) {
  const float v = static_cast<float>(insulation);
  return v * v * kInsulationScale;
}

// A plain minimum, then the same surface-area product the mean branch
// applies. The `<= 0` guard is not the game's — it would reach `min(0, ...)` and get zero,
// and so would the caller's own `k <= 0` rejection, so it changes nothing and is kept only
// to read the same as the uncached function above.
inline float PairConductivityInsulated(const ElementTable& table, uint16_t a, uint16_t b,
                                       float fa, float fb) {
  const Element& ea = table.At(a);
  const Element& eb = table.At(b);
  const float ka = ea.thermalConductivity;
  const float kb = eb.thermalConductivity;
  if (ka <= 0.0f || kb <= 0.0f) return 0.0f;
  const float sa = SurfaceAreaMultiplier(ea, table.Phase(b));
  const float sb = SurfaceAreaMultiplier(eb, table.Phase(a));
  return std::min(fa * ka, fb * kb) * (sb * sa);
}


// The pair exchange. The game does all of it in **double**.
// Its caller orders the pair so the cooler cell goes in first, so `dT` is never
// negative.
//
//   Teq = (Tc*Cc + Th*Ch) / (Cc + Ch)
//   q   = min(dT*k*dt, (dT*0.25)*Cc, (dT*0.25)*Ch)      -- dt is (double)0.2f
//   Tc' = min(Tc + q/Cc, Teq)
//   Th' = max(Th + (-1.0/Ch)*q, Teq)
//
// Three things here are not the obvious spelling and all three move the last bit. The
// quarter clamp is `(dT*0.25)*C` rather than `dT*(0.25*C)`; the two capacities are separate
// candidates in a three-way minimum rather than one `min(Cc, Ch)` term; and the hot side is
// `Th + (-1/Ch)*q`, a reciprocal and a multiply, where the cold side is a real division.
// The clamp to the equilibrium temperature is the piece that has no counterpart at all in a
// naive `dQ/C` step: it is what stops a pair crossing over, and the quarter is a second,
// looser guard in front of it.
struct PairExchange {
  float cold, hot;
};
// `dt` is the substep widened to double, which at the default is (double)0.2f =
// 0.20000000298023224, Klei's constant. Widening a float is exact, so the
// default is that constant to the bit.
inline PairExchange ExchangeTemperatures(double tc, double cc, double th, double ch,
                                         float k_pair, double dt) {
  const double delta = th - tc;
  const double teq = (tc * cc + th * ch) / (cc + ch);
  const double quarter = delta * 0.25;
  double q = delta * static_cast<double>(k_pair) * dt;
  const double qc = quarter * cc, qh = quarter * ch;
  // A minimum over the three in this order, where a tie keeps the earlier, as in the game.
  if (qc < q) q = qc;
  if (qh < q) q = qh;
  double lo = tc + q / cc;
  if (teq < lo) lo = teq;
  double hi = th + (-1.0 / ch) * q;
  if (hi < teq) hi = teq;
  PairExchange out;
  out.cold = static_cast<float>(lo);
  out.hot = static_cast<float>(hi);
  return out;
}

inline float HeatCapacity(const ElementTable& table, const PhaseEntry& e) {
  return e.mass * table.SpecificHeat(e.element);
}

// Charges the change in stored energy of two or three grid cells to the energy ledger's
// `mover` bucket over this guard's own scope.
//
// Scoped rather than written out at each site because the fluid sweeps do not have a single
// exit: the gas pressure sweep alone returns from four places -- a non-vacuum destination, a
// Void destination that destroys the delivery outright, and two early rejections -- and the
// element write that decides how much energy the delivered mass is even worth happens at the
// last of them. A hand-written before/after pair would have to be correct at every one.
class MoverCharge {
 public:
  static constexpr size_t kNone = static_cast<size_t>(-1);
  //
  // `Sum()` walks two or three cells, and this guard is constructed on every attempted move,
  // so both ends are gated on `World::EnergyLedgerEnabled()` -- see the note beside it in
  // world.h. `on_` is latched at construction rather than re-read in the destructor: a run
  // that switched the flag between the two would charge a delta against a `before_` that was
  // never taken, which is a worse failure than not charging at all.
  MoverCharge(World* w, const ElementTable& table, size_t a, size_t b, size_t c = kNone)
      : w_(w),
        table_(table),
        a_(a),
        b_(b),
        c_(c),
        on_(w->EnergyLedgerEnabled()),
        before_(on_ ? Sum() : 0.0) {}
  ~MoverCharge() {
    if (on_) w_->NoteMoverEnergy(Sum() - before_);
  }
  MoverCharge(const MoverCharge&) = delete;
  MoverCharge& operator=(const MoverCharge&) = delete;

 private:
  double Sum() const {
    double total = GridCellEnergy(*w_, table_, a_) + GridCellEnergy(*w_, table_, b_);
    if (c_ != kNone) total += GridCellEnergy(*w_, table_, c_);
    return total;
  }
  World* w_;
  const ElementTable& table_;
  size_t a_, b_, c_;
  bool on_;
  double before_;
};

// `AddMassAndUpdateTemperature`, the game's name for the one function every "pour some of this
// cell into that one" path in the sim goes through. `UpdateLiquid`'s mover, `DoDisplacement`
// (and so `DisplaceGas`), `DoSublimation` and the three `Add*` entry points all call it, so
// they all share its arithmetic and there is no reason for any of them to spell it out
// differently here.
//
//   total = dst.mass + amount
//   dst.temperature = clamp((dst.mass*dst.temperature + src_temp*amount) / total,
//                           min(dst.temperature, src_temp), max(...))
//
// The clamp is the part that matters. Without it a mix of two
// cells at the *same* temperature does not come back to that temperature — the quotient
// rounds — and a world with no gradient in it at all still drifts a last bit per transfer.
// One ulp is enough: `GasShuffle` keeps the coolest candidate, so it eventually swaps a
// different pair of cells and the two sims stop agreeing about anything.
inline void AddMassAndUpdateTemperature(PhaseEntry* dst, float amount, float src_temp) {
  const float total = dst->mass + amount;
  if (total > 0.0f) {
    const float mix = (dst->mass * dst->temperature + src_temp * amount) / total;
    const float lo = dst->temperature < src_temp ? dst->temperature : src_temp;
    const float hi = dst->temperature < src_temp ? src_temp : dst->temperature;
    dst->temperature = mix < lo ? lo : (mix > hi ? hi : mix);
    dst->mass = total;
  } else {
    // Nothing there and nothing arriving, so the temperature is cleared rather than left
    // behind.
    //
    // The mass write lives in the branch above, not after the `if`, as in the game: this
    // branch zeroes the temperature and **returns without touching the mass at all**.
    //
    // Nothing in the sim can reach it with anything to show for it. Every caller pours a
    // non-negative amount into a cell, so `total` can only come out at zero, and at zero the
    // three writes this branch skips would all have written what was already there. That was
    // checked rather than assumed: `cellmodliq`'s sealed-rock probe drives the branch on
    // every run — the incoming liquid weighs exactly what the rock it displaced weighed — and
    // ablating either half of it scores 0.000000 K and 0.000000 kg. Kept because it is the
    // game's behaviour, and labelled as unreachable.
    dst->temperature = 0.0f;
  }
}

// One conduction pass over the grid.
//
// Neighbour pairs are visited once each (right and up from every cell), and both sides of
// a pair are updated from the temperatures at the *start* of the frame. Reading updated
// values mid-sweep would make the result depend on iteration order, which is the classic
// way a diffusion kernel stops being symmetric and starts drifting.
// Temporary instrumentation, compiled only when `ONI_CONDUCTION_CENSUS` is defined — which
// `sim/build.sh` never does, because printing from inside the DLL kills the sim thread. It
// prints from a static destructor so `bench` needs no changes to report it.
#ifdef ONI_CONDUCTION_CENSUS
#include <cstdio>
struct ConductionCensus {
  unsigned long long substeps = 0, visits = 0, calls = 0, inactive = 0, deadzone = 0,
                     vacuum = 0, no_k = 0, no_cap = 0, writes = 0;
  ~ConductionCensus() {
    if (substeps == 0 || calls == 0) return;
    const double s = static_cast<double>(substeps);
    const double c = static_cast<double>(calls) / s;
    auto row = [&](const char* name, unsigned long long v) {
      std::printf("  %-24s %10.0f   %5.1f %%\n", name, v / s, 100.0 * v / calls);
    };
    std::printf("\nconduction census, per substep over %llu substeps\n", substeps);
    std::printf("  %-24s %10.0f\n", "driving cells visited", visits / s);
    std::printf("  %-24s %10.0f\n", "conduct calls", c);
    row("rejected: inactive", inactive);
    row("rejected: dead zone", deadzone);
    row("rejected: vacuum", vacuum);
    row("rejected: k <= 0", no_k);
    row("rejected: heat capacity", no_cap);
    row("reach the write", writes);
  }
};
inline ConductionCensus g_conduction_census;
#define ONI_CC(field) (++::oni_sim::g_conduction_census.field)
#else
#define ONI_CC(field) ((void)0)
#endif

// `rooms`: the same promotion gate as `StepGasPressure`. `nullptr` means "no gate" -- the
// check inside `conduct` below short-circuits on `rooms == nullptr` before touching `room_of`,
// so this is byte-for-byte the vanilla kernel until a real `RoomGraph` is passed. When it is, a pair with either end in a promoted room exchanges nothing here --
// vanilla conduction neither heats nor cools a promoted cell, in either direction. It does
// not make conduction read the mixture's own thermal state for that cell; it only stops
// vanilla conduction from writing that cell's temperature once the room owns it.
inline void StepConduction(World* w, const ElementTable& table, size_t ri,
                           const gas::RoomGraph* rooms = nullptr) {
  ONI_SWEEP(kConduction);
  ONI_CC(substeps);
  const World::PaddedRect& r = w->PaddedRegion(ri);
  const int32_t pw = w->PaddedWidth(), ph = w->PaddedHeight();
  std::vector<PhaseEntry>& cells = w->Phases();
  const Tunables tun = g_tunables;  // tunables.h, THE ONE RULE
  const int32_t tile_shift = tun.conduction_tile_shift;
  const int32_t tile = 1 << tile_shift;
  const float min_delta = tun.min_conduction_delta;
  const float clamp_lo = tun.conduction_min_temperature;
  const float clamp_hi = tun.conduction_max_temperature;
  const double dt = static_cast<double>(tun.substep_seconds);

  // Temperatures as they were at the start of the frame.
  static thread_local std::vector<float> start;

  // The gather also summarises each tile, because the census says the sweep below refuses
  // 99.9 % of the pairs it forms on a settled world and still pays to form them. See
  // Measured: the gather moves 389 KB in 0.019 ms against a sweep that costs 1.15, so
  // a min/max folded into it is nearly free and buys the right to refuse whole tiles.
  // Region 0 gathers the grid; a later region gathers only what its own pairs and the tile
  // summary below can read, which is its rectangle grown by one. See `SnapshotReadRect`
  // (world.h) -- this snapshot is the cheapest of the four (4 bytes a cell against 12) and
  // still cost 11 % of the kernel across a twelve-region cluster.
  RefreshTemperatureSnapshot(start, cells, ri, pw, ph, r.x0, r.y0, r.x1, r.y1);

  const int32_t tiles_x = (pw + tile - 1) >> tile_shift;
  const int32_t tiles_y = (ph + tile - 1) >> tile_shift;
  const size_t tile_count = static_cast<size_t>(tiles_x) * static_cast<size_t>(tiles_y);
  static thread_local std::vector<float> tile_min, tile_max;
  tile_min.assign(tile_count, kHugeTemperature);
  tile_max.assign(tile_count, -kHugeTemperature);

  // Summarised over the rectangle, not the grid, and that is the difference between a win
  // and a regression: on a `quarter` region a whole-grid summary costs more than the sweep it
  // is trying to shorten (0.162 -> 0.258 ms, measured). The cells a rectangle's pairs read are
  // the rectangle grown by one to the right and one up, so that is the area summarised, and
  // anything outside it is never read by a pair this can skip. Tiles this region does not
  // reach keep their empty `+huge`/`-huge` pair, which reads as quiet and is never consulted:
  // the sweep below only ever looks up tiles it is standing in.
  // Tile-major, with the running min and max in registers: a per-cell read-modify-write into
  // the two tile arrays costs more than the reads it is summarising.
  //
  // THIS LOOP DOES NOT VECTORISE, and this comment used to say it did. `-fopt-info-vec-all`
  // reports "couldn't vectorize loop ... Analysis failed with vector mode V4SF"
  // for the `x` loop below: it is the inner of three consecutive nested loops, which GCC
  // refuses as a nest. Three loops in the whole of `sim/` vectorise and none of them is this
  // one. The claim was plausible -- it IS a straight run over contiguous floats -- and it was
  // never checked; the flag that checks it costs one build.
  {
    const int32_t sy1 = std::min(r.y1 + 1, ph), sx1 = std::min(r.x1 + 1, pw);
    for (int32_t ty = r.y0 >> tile_shift; ty <= (sy1 - 1) >> tile_shift;
         ++ty) {
      const int32_t y0 = std::max(r.y0, ty << tile_shift);
      const int32_t y1 = std::min(sy1, (ty + 1) << tile_shift);
      for (int32_t tx = r.x0 >> tile_shift; tx <= (sx1 - 1) >> tile_shift;
           ++tx) {
        const int32_t x0 = std::max(r.x0, tx << tile_shift);
        const int32_t x1 = std::min(sx1, (tx + 1) << tile_shift);
        float lo = kHugeTemperature, hi = -kHugeTemperature;
        for (int32_t y = y0; y < y1; ++y) {
          const float* p = &start[static_cast<size_t>(y) * pw];
          for (int32_t x = x0; x < x1; ++x) {
            const float t = p[x];
            lo = std::min(lo, t);
            hi = std::max(hi, t);
          }
        }
        const size_t ti = static_cast<size_t>(ty) * tiles_x + tx;
        if (lo < tile_min[ti]) tile_min[ti] = lo;
        if (hi > tile_max[ti]) tile_max[ti] = hi;
      }
    }
  }

  // A tile is quiet when nothing its pairs can reach spans the dead zone. A cell's pairs go
  // right and up by one, so they reach into the tile to the right and the tile below; taking
  // those two neighbours whole is a superset of the cells actually touched, which makes the
  // test conservative — it can only ever refuse to skip.
  static thread_local std::vector<uint8_t> quiet;
  quiet.assign(tile_count, 0);
  for (int32_t ty = 0; ty < tiles_y; ++ty) {
    for (int32_t tx = 0; tx < tiles_x; ++tx) {
      const size_t ti = static_cast<size_t>(ty) * tiles_x + tx;
      float lo = tile_min[ti], hi = tile_max[ti];
      if (tx + 1 < tiles_x) {
        lo = std::min(lo, tile_min[ti + 1]);
        hi = std::max(hi, tile_max[ti + 1]);
      }
      if (ty + 1 < tiles_y) {
        lo = std::min(lo, tile_min[ti + tiles_x]);
        hi = std::max(hi, tile_max[ti + tiles_x]);
      }
      quiet[ti] = (hi - lo) < min_delta ? 1 : 0;
    }
  }

  PairConductivityCache& pair_k = ThreadPairConductivity();
  pair_k.Bind(table);
  const uint8_t* insulation = w->Insulation().data();
  // The extension registry's hoist (sim/ext_registry.h). Was `ThermalMassBonus().data()` off
  // a member vector; it is one bounds check and one `.data()` off a registered property now,
  // paid once per kernel invocation, and the inner loop below still indexes a plain float
  // array. Null only if the world has not allocated, which cannot be true here.
  const float* thermal_bonus = w->ThermalMassBonusData();

  // RETURNS whether this pair moved any heat, so the caller can count the DRIVING CELL as
  // active exactly once however many of its two pairs transferred. Counting inside here
  // instead would count PAIRS, and a cell forms two of them -- `active` would be allowed to
  // exceed `examined`, and the one invariant every other sweep obeys (a kernel cannot change
  // more cells than it looked at) would need an exception carved for this kernel. An
  // assertion with an exception in it is an assertion that stops being read.
  auto conduct = [&](size_t ia, size_t ib) -> bool {
    // A pair straddling the edge of the region exchanges nothing at all, which is measured
    // rather than assumed — see World::SetActiveRegions. That used to be a membership test
    // on both ends; it is now the loop bound below, which is the same thing for one region
    // and *not* the same thing for two. See the note on the sweep.
    ONI_CC(calls);
    // Promotion gate: checked before the dead-zone test so
    // a promoted pair is refused for the right reason even when it would also have failed the
    // dead zone -- correctness first, the ordering below this point is what is tuned for
    // rejection speed. `rooms == nullptr` (the default, every existing call site) short-circuits
    // before either `room_of` lookup, so a world that never promotes anything pays one pointer
    // compare here and nothing else.
    if (rooms != nullptr &&
        ((ia < rooms->room_of.size() && gas::IsRoomOwned(*rooms, rooms->room_of[ia])) ||
         (ib < rooms->room_of.size() && gas::IsRoomOwned(*rooms, rooms->room_of[ib])))) {
      return false;
    }
    // The dead-zone test goes first because it is the cheapest and by far the most
    // selective: it reads two floats out of `start` and rejects most pairs in a world that
    // is anywhere near settled. Every test in this function is a pure read with no side
    // effect, so their order decides only how much work a rejected pair costs, never which
    // pairs transfer — the suite is bit-identical across this change.
    //
    // It used to run last, after `PairConductivity` had already done two element-table
    // lookups, two surface-area branches and a square root for a pair that was about to be
    // discarded. Measured at 256x384: 2.60 -> 2.25 ms median on a world with live
    // gradients everywhere (13 %), and 1.63 -> 1.20 ms on a settled one (27 %), which is
    // the shape to expect — the more of the map is at rest, the more pairs this rejects
    // before doing any arithmetic. A real base sits nearer the settled end.
    const float delta = start[ia] - start[ib];
    if (delta > -min_delta && delta < min_delta) { ONI_CC(deadzone); return false; }

    PhaseEntry& a = cells[ia];
    PhaseEntry& b = cells[ib];
    // Vacuum conducts nothing. Measured: a solid pair isolated in a vacuum world
    // exchanges heat only with each other, never with the vacuum around them.
    if (a.mass <= 0.0f || b.mass <= 0.0f) { ONI_CC(vacuum); return false; }

    // Insulation is per cell, so it cannot live in the element-pair cache, and it cannot be
    // applied to the cached result either: below 1.0 the formula changes shape, not scale.
    // It is read after the dead zone and the vacuum test because those two reject most pairs
    // in a settled world before any of this arithmetic happens.
    //
    // The game's own test is on the factors, each against 1.0, and this is on the
    // bytes against 255, which is the same test: the factor is `v*v/255^2`, so 255 gives
    // exactly 1.0 (the suite proves it, since every scenario before this one sent 255 and
    // matched the geometric-mean branch) and 254, the next value down, gives 0.992. Spelling
    // it on the bytes keeps two int-to-float converts and four multiplies off the path that
    // every uninsulated pair in the world takes. Asteroid conduction, median of 50: 2.507 ms
    // computing both factors, 2.453 comparing the bytes, 2.427 with an ablation that does not
    // read the array at all. So the whole feature costs 1 % of the kernel and half of what is
    // left is the two scattered byte loads, which is the floor.
    const uint8_t ia_ins = insulation[ia], ib_ins = insulation[ib];
    const float k = (ia_ins == 255 && ib_ins == 255)
                        ? pair_k.Get(table, a.element, b.element)
                        : PairConductivityInsulated(table, a.element, b.element,
                                                    InsulationFactor(ia_ins),
                                                    InsulationFactor(ib_ins));
    // Neither of these two rejections may return: Klei has no early-out here at all. A pair
    // with no conductivity between it runs the whole exchange, gets zero back, and still
    // reaches the clamp at the bottom — which is the only reason a cell that a message left
    // below 1 K ever comes back up. `modifycell` is what found this: a `ModifyCell` writing
    // 0.5 K into a fully insulated granite cell reads 1 K from Klei on the very next tick,
    // and 0.5 from us for as long as this was a `return`. Insulation 0 makes `k` zero, so
    // every pair in that world took this exit and the clamp below had never once run in a
    // scenario that could tell.
    bool exchange = k > 0.0f;
    if (!exchange) ONI_CC(no_k);
    // The two rejections are not the same shape, and `vacrect` is what separates them. A
    // pair with no conductivity still reaches the clamp (above); a pair where either end has
    // **no heat capacity** does not — it leaves both temperatures exactly as it found them.
    // The witness is the Unobtanium ring `ResizeAndInitializeVacuumCells` writes: its element
    // carries `specificHeatCapacity` 0 *and* `thermalConductivity` 0, the message leaves it
    // at 0 K, and Klei reads back 0.000000 for fifty ticks while a clamp outside this guard
    // reads back 1. `modifycell`'s insulated granite is the other side of the same test —
    // there the capacity is 0.79 and only `k` is zero, and Klei does clamp.
    //
    // The capacities are computed for the no-conductivity pairs too, because the clamp needs
    // the answer whether or not the exchange happens. That is the rare arm: `k` is positive
    // for all but insulated and zero-conductivity pairs, so this costs the common path
    // nothing it was not already paying.
    // Framework extension: `thermal_bonus` is 0 on every cell unless something has sent
    // `kSetCellThermalMassBonus` (see abi/sim_abi_ext.h), so a world that never touches
    // the extension adds exactly 0.0f here — same bits as vanilla. The array
    // behind the pointer is a registered extension property.
    const float ca = HeatCapacity(table, a) + thermal_bonus[ia];
    const float cb = HeatCapacity(table, b) + thermal_bonus[ib];
    const bool has_capacity = ca > 0.0f && cb > 0.0f;
    if (!has_capacity) {
      if (exchange) ONI_CC(no_cap);
      exchange = false;
    }
    if (exchange) ONI_CC(writes);

    // The quarter clamp and the equilibrium clamp both live in `ExchangeTemperatures`. A
    // single pair may move the lighter side by at most a **quarter** of the temperature
    // difference: one kilogram of oxygen with a single 400 K granite neighbour, whose
    // unclamped step would carry it to 442 K, lands on exactly 325.00000, and the same cell
    // with four hot neighbours lands on exactly 400.00000 — four quarters. A quarter is one
    // neighbour's share of four, which is what makes the bound safe.
    //
    // Klei orders the pair by temperature and hands the cooler cell in first.
    if (exchange) {
    const bool a_hot = delta > 0.0f;
    const PairExchange x =
        a_hot ? ExchangeTemperatures(start[ib], cb, start[ia], ca, k, dt)
              : ExchangeTemperatures(start[ia], ca, start[ib], cb, k, dt);
    const float new_a = a_hot ? x.hot : x.cold;
    const float new_b = a_hot ? x.cold : x.hot;

    // The new temperature is turned back into a *difference* against the frame's
    // starting temperature and that difference is added to the live grid, so a cell that has
    // already conducted with one neighbour keeps both contributions. Then the live value is
    // clamped to [1 K, 10000 K] — the tunables
    // `ConductionMinTemperature` / `ConductionMaxTemperature` — which is also the pair of
    // constants the function's own asserts are written against.
    a.temperature = new_a - start[ia] + a.temperature;
    b.temperature = new_b - start[ib] + b.temperature;
    }
    // Guarded so the store stays off the hot path. The clamp is idempotent and all but a
    // handful of cells in any world are already inside the range, so testing first turns
    // this from two writes per surviving pair into none. `has_capacity` is the outer guard
    // and not `exchange` — see the note above it.
    if (has_capacity) {
      // Energy ledger: this clamp is a source at one end and a sink at the other, and
      // nothing anywhere pays for either. Charged with `HeatCapacity` rather than with the
      // `ca`/`cb` above, deliberately -- those carry `thermal_bonus`, which is a framework
      // extension the ledger's grid total (World::TotalGridEnergy) does not know about, and
      // charging against a capacity field 0 cannot see would read as permanent drift.
      //
      // Only mass and temperature enter here and only the temperature moves, so the change
      // is exactly `mass * specificHeatCapacity * dT` -- the same product field 0 sums.
      if (a.temperature < clamp_lo || a.temperature > clamp_hi) {
        const float clamped = std::max(clamp_lo, std::min(clamp_hi, a.temperature));
        w->NoteConductionClamp(static_cast<double>(HeatCapacity(table, a)) *
                               (static_cast<double>(clamped) -
                                static_cast<double>(a.temperature)));
        a.temperature = clamped;
      }
      if (b.temperature < clamp_lo || b.temperature > clamp_hi) {
        const float clamped = std::max(clamp_lo, std::min(clamp_hi, b.temperature));
        w->NoteConductionClamp(static_cast<double>(HeatCapacity(table, b)) *
                               (static_cast<double>(clamped) -
                                static_cast<double>(b.temperature)));
        b.temperature = clamped;
      }
    }
    return exchange;
  };

  // The border ring is not part of the world and must not act as an infinite heat sink,
  // so pairs are formed only between interior cells.
  //
  // The bounds are this region's rectangle rather than the whole grid, and the driving cell
  // is the only end this can drop: `conduct` rejects a pair unless *both* ends are active,
  // so a cell outside every rectangle can never be the near end of an accepted pair and
  // visiting it was pure loss. Restricting a row-major scan to a rectangle leaves the
  // accepted pairs in the same order as well as the same set, which is what makes this
  // bit-exact rather than merely equivalent — the temperature write is an accumulation
  // with a clamp on each step, so the order is load-bearing.
  {
    for (int32_t y = r.y0; y < r.y1; ++y) {
      ONI_EXAMINED(kConduction, r.x1 - r.x0);
      const size_t trow = static_cast<size_t>(y >> tile_shift) * tiles_x;
      for (int32_t x = r.x0; x < r.x1;) {
        // A quiet tile holds no pair that can clear the dead zone, so the whole run of
        // columns it covers is skipped without forming them. Every test in `conduct` is a
        // pure read with no side effect, so this cannot change which pairs transfer, in what
        // order, or by how much — the row-major order of the pairs that remain is untouched.
        const int32_t tx = x >> tile_shift;
        if (quiet[trow + static_cast<size_t>(tx)]) {
          const int32_t next = (tx + 1) << tile_shift;
          ONI_SKIPPED(kConduction, std::min(next, r.x1) - x);
          x = next;
          continue;
        }
        const size_t i = static_cast<size_t>(y) * pw + x;
        ONI_CC(visits);
        // Both ends inside **this** rectangle, which is the whole of the region test now.
        // The rectangle is already clamped to the interior, so `r.x1 <= pw - 1` and the
        // border guard this replaces is subsumed rather than dropped.
        //
        // With one region it is the old `Active` pair test exactly: a far end outside the
        // rectangle is a far end outside the only region there is. With two it is not, and
        // `regionadj` is what says which one Klei does. Two regions that meet at a column
        // have a seam pair that neither sweep forms — the left region stops one short of it
        // and the right region's cells only ever pair rightward and upward — so the seam
        // does not conduct at all. Measured: with the mask test the last gas column of the
        // left region conducts into the wall and ends 34.7 K cold against Klei; with this it
        // does not, and the scenario is exact.
        // `|` and not `||`: both pairs must be formed, and short-circuiting the second one
        // the moment the first transferred would silently delete half this kernel's work.
        bool moved = false;
        if (x + 1 < r.x1) moved = conduct(i, i + 1);
        if (y + 1 < r.y1) moved = conduct(i, i + static_cast<size_t>(pw)) | moved;
        // Conduction is the one sweep that announces nothing -- it moves temperature, and
        // `TouchSubstance` is about substance -- so it is also the one that counts its own
        // changes, and it counts CELLS: the driving cell of a pair that transferred, once,
        // however many of its two pairs did.
        if (moved) ONI_CHANGED(kConduction);
        ++x;
      }
    }
  }
}

// --------------------------------------------------------------------- state changes
//
// Every element carries a low and a high transition. Both fire on temperature alone, and
// all three constants below were measured on samples sealed inside granite *at the
// sample's own temperature*, so the pair difference is zero, conduction's 1 K dead zone is
// never reached, and nothing but the transition can move the cell.
//
//   T > highTemp + 3   ->  highTempTransitionIdx, and the new temperature is T - 1.5
//   T < lowTemp  - 3   ->  lowTempTransitionIdx,  and the new temperature is T + 1.5
//
// The 3 K margin is the surprise. `T > highTemp` is wrong: water at 372.55 K, granite at
// 943 K and a third element one degree over its own threshold all sat still for three
// ticks. Sweeping the boundary at 0.05 K puts it between +3.00 (never fires) and +3.05
// (always fires), identically for water at 372.5 K, granite at 942 K and a solid at
// 272.5 K, and identically at 1000 kg, 5 kg and 0.5 kg. So it is a fixed number of kelvin,
// not a fraction of the threshold and not an energy.
//
// The margin and the adjustment are two halves of the same mechanism: after a transition
// the cell sits exactly 1.5 K past the threshold in its new phase, which is half the 3 K
// it would need to cross back. Without that, a cell straddling a boundary would flip every
// substep forever.
//
// Temperature carries across; the specific heats do not. 1000 kg of water at 200 K becomes
// ice at 201.5 K, and ice's specific heat is 2.05 against water's 4.179, so the cell's
// internal energy nearly halves. That is not a defect in the measurement — energy
// conservation would have put it at 407 K. Klei conserves temperature and lets the energy
// jump, and the 1.5 K nudge is the whole of its latent heat.
//
// Both are tunables, `TransitionMargin` and `TransitionOvershoot` (tunables.def), and reach
// `TransitionCell` through `PhaseRules`.

// Klei's literals from the kernels below are tunables (tunables.def). Each is an immediate at
// its own site in the stock DLL, so each is its own row even where two share a value: nothing
// shows they come from one constant, and folding them would let one knob move two Klei
// behaviours. By the Klei function they belong to:
//
//   `DoStateTransition`, small freeze   SmallFreezeOreRatio (via PhaseRules)
//   `EvaporateWisp`                     GasWispMassKg
//   `GasShuffle`                        GasShuffleSkipGate, GasShuffleCandidateCoin
//   `DoDensityDisplacement`             GasDisplacementProbability, LiquidDisplacementProbability
//   `SpawnFallingLiquid`                FallingLiquidTemperatureWindow
//   `DoPartialMelt`                     PartialMeltKg, PartialMeltMargin, PartialMeltGasMargin
//   `DoSublimation`                     SublimationCellCapKg
//   `DoLiquidOffGas`                    OffGasCellCapKg, OffGasAmountCapKg, OffGasResidualKg
//   `Evaporate` in post-process         ThinLiquidKg
//   `UpdateLiquid`                      LiquidPourShare, LiquidUpwardShare,
//                                       LiquidViscosityBypassRatio

// 65535, not 0, is the element table's "no such element". Index 0 is a real element, so
// testing against 0 makes every element look like it has a transition.
inline constexpr uint16_t kNoElement = 0xFFFF;

// A transition that has to hand mass to the game rather than keep it in the grid.
//
// Some elements name a *transition ore*: a second element that a fixed fraction of the
// mass becomes. Measured on two of them in both directions — 1000 kg of element 31 at its
// threshold left 800 kg of the main product in the cell and produced a 200 kg ore drop
// against `highTempTransitionOreMassConversion` of 0.2, and an element with 0.68 left
// 320 kg and dropped 680 kg. So the conversion is the ore's share, not the product's, and
// both come out at the post-transition temperature.
//
// The small-freeze rule (below `TransitionCell`) hands over a whole cell, germs and all, so a
// record also carries the cell's disease. A transition ore's share carries its part of the
// cell's germs, the same share of the count, which the cell pays.
struct StateChangeOre {
  int32_t game_cell;
  uint16_t element;
  float mass;
  float temperature;
  uint8_t disease_idx = 0xFF;
  int32_t disease_count = 0;
};

// One state-change pass.
//
// Ordering inside a substep is **conduction, then this, then flow**, and both halves were
// measured rather than chosen:
//
//   * Against conduction: a cell walked across its boundary by a hot neighbour is never
//     observed above the boundary and still unchanged. It read 375.48965 K as water on one
//     frame and 374.00754 K as steam on the next — and 374.00754 + 1.5 is 375.50754, which
//     is exactly 375.48965 plus that frame's conduction step. The substep that carries the
//     cell over converts it. A gas cooled from 606 K to its surroundings in a single
//     substep by four granite neighbours condensed at the *surrounding* temperature, not
//     its own, which says the same thing a second way.
//   * Against flow: 5 kg of water at 500 K on the floor of a sealed vacuum shaft cannot
//     move — the floor and walls are granite and it is far under maxMass. The first frame
//     it reads as steam it has already lost half a kilogram upward. So the transition runs
//     first and the new gas flows the same substep.
//
// One transition per cell per substep, not a loop: ice seeded at 500 K, which is above both
// ice's threshold and the threshold of the water it becomes, took two frames to reach steam
// at 0.2 s a frame and one frame at 0.4 s. Two substeps, two transitions.
// ------------------------------------------------------------- condensation
//
// The gate on the game's low-temperature branch, following Stationeers' gas state change. Its
// three tests, in order:
//
//     if the element cannot condense                  -> refuse
//     if pressure < the element's minimum liquid pressure -> refuse
//     if temperature >= saturation temperature(pressure)  -> refuse
//
// THIS CAN ONLY SUBTRACT A TRANSITION, NEVER ADD ONE. Letting the saturation temperature
// replace the game's fixed `lowTemp - TransitionMargin` outright (so a cell at 2 MPa and 240 K
// condenses instead of waiting for 214 K) would oscillate unless the liquid's boil side moved
// with it: an element's `highTemp` is a one-atmosphere boiling point, so CO2 condensed at 2 MPa
// would be 44 K above it and boil straight back the next substep. `BoilingAllowed` below is the
// pressure-aware boil side.
//
// What this gate alone guarantees: Mars' 0.6 kPa surface CO2 is four orders of magnitude under
// CO2's 517 kPa minimum liquid pressure, so the rule REFUSES the condensation the game's fixed
// threshold would make every night, and refusing cannot oscillate.
struct CondensationRule {
  ElementTable::PhaseCurve curve;
  float min_liquid_pressure_pa = 0.0f;
  bool can_condense = false;
};

// True when `gas_element` carries a COMPLETE ruleset and its low-temperature transition really
// is a gas condensing into a liquid.
//
// The phase test is not decoration. OniFramework registers one `MaterialProperties` against
// both halves of a family (`MaterialPropertyRegistry.RegisterFamily(props, gasId, liquidId)`),
// so a liquid carries the same `sim.phase_curve` as its gas -- and every liquid in Stationeers'
// `CanCondense` table reads false. Keying this on "has a curve" alone would therefore have
// caught the FREEZING branch of every curve-carrying liquid and refused it outright: liquid
// nitrogen would have stopped turning into ice.
inline bool CondensationRuleOf(const ElementTable& table, uint16_t gas_element,
                               uint16_t low_target, CondensationRule* out) {
  if (table.Phase(gas_element) != kStateGas) return false;
  if (static_cast<int32_t>(low_target) >= table.Count()) return false;
  if (table.Phase(low_target) != kStateLiquid) return false;
  bool described = false;
  const bool can = table.CanCondenseOf(gas_element, &described);
  // UNSET IS NOT ZERO. An element the managed side has never described has no rule and keeps
  // Klei's threshold untouched -- which is what makes the 823-scenario byte-identity gate safe
  // by construction, since no offline scenario registers an element attribute at all.
  if (!described) return false;
  ElementTable::PhaseCurve curve;
  if (!table.PhaseCurveOf(gas_element, &curve)) return false;
  if (out) {
    out->curve = curve;
    out->min_liquid_pressure_pa = table.MinLiquidPressureOf(gas_element);
    out->can_condense = can;
  }
  return true;
}

// The gate itself. Returns true to let Klei's branch proceed -- including for every element
// that carries no rule, which is every element in every vanilla scenario.
// `rules_present` is `ElementTable::AnyCondensationRules()`, HOISTED OUT OF THE CALLER'S LOOP
// rather than read here. Reading it per cell -- three dependent loads through the attribute
// registry -- cost half of a +0.010 ms regression on `StepStateChange` all by itself; see the
// accessor's comment. `StepStateChange` evaluates it once per region and hands
// it down; callers with one cell to transition (buildings.h) let it default and pay the read.
inline bool CondensationAllowed(const ElementTable& table, const PhaseEntry& c,
                                uint16_t low_target, bool rules_present) {
  if (!rules_present) return true;
  CondensationRule rule;
  if (!CondensationRuleOf(table, c.element, low_target, &rule)) return true;
  if (!rule.can_condense) return false;
  const float molar = table.MolecularMassOf(c.element);
  // No molar mass means no pressure can be computed. Deferring to Klei beats inventing one.
  if (!(molar > 0.0f)) return true;
  // The cell holds one element, so its partial pressure IS its pressure. A mixture-owned cell
  // never reaches here at all -- promotion leaves it at vacuum/zero mass in `PhaseEntry` and
  // `TransitionCell` returns on `mass <= 0` before this is called.
  const float pressure =
      gas::PressureFromMoles(c.mass * gas::kGramsPerKilogram / molar, c.temperature,
                             g_tunables.cell_volume_m3);
  if (pressure < rule.min_liquid_pressure_pa) return false;
  return c.temperature < EvaporationTemperatureClampedK(rule.curve, pressure);
}

// ------------------------------------------------------------------ the boil side
//
// THE PRESSURE A LIQUID CELL BOILS AGAINST. Pascals, absolute.
//
// Condensing CO2 at 2 MPa yields liquid at 241.5 K; the curve puts its saturation temperature
// at that pressure at 243.37 K, so the boil is refused and no condense/boil flip-flop starts.
//
// The pressure is that of the GAS the liquid opens into, as in Stationeers, where a liquid
// boils against the pressure of the gas sharing its space and never against its own. The walk
// below climbs the column the liquid sits in and reads the gas at the top.
//
// On top of that sits a hydrostatic term, which Stationeers has no analogue for: a Stationeers
// liquid has no geometry, while an ONI liquid is a column of cells with a real depth.
//
// THE HYDROSTATIC TERM IS A MASS, NOT A DEPTH, and that is exact rather than a shortcut.
// P = rho*g*h over a column of cross-section A is (m/V)*g*h = m*g*h/(A*h) = m*g/A, and an ONI
// cell is 1 m x 1 m x 1 m, so A = 1 m^2 and the pressure a column adds is simply the mass
// above it times g. A full 1000 kg water tile is 9.81 kPa, which is the right answer for a
// metre of water. The cell's OWN mass counts half: the pressure being asked for is the one at
// the cell, and half its mass lies below its own mid-depth.
//
// GRAVITY IS A CONSTANT HERE. A per-world g would belong in `World::Environment`, whose
// message is fixed-size, so it waits for that message's next major revision. On Mars
// (3.71 m/s^2) a column presses about 2.6x less than this says.
//
// PERF: +0.003 ms on `StepStateChange`. With no attributes registered `PhaseRules::boiling` is
// false and `BoilingAllowed` returns on its first compare, so the walk below is only ever
// entered by a cell whose element carries a curve AND whose temperature has already crossed
// the game's fixed threshold. Keep that ordering.
// Gravity is the tunable `StandardGravity` (tunables.def), m/s^2.

inline float AmbientPressureAtLiquidCellPa(const World& w, const ElementTable& table,
                                           int32_t x, int32_t y) {
  const int32_t pw = w.PaddedWidth();
  const std::vector<PhaseEntry>& cells = w.Phases();
  // Half its own mass, because the pressure wanted is the one at this cell and not the one
  // under it. See the note above.
  double column_kg = 0.5 * static_cast<double>(cells[static_cast<size_t>(y) * pw + x].mass);
  float gas_pa = 0.0f;
  // Game rows only. Padded row `GameHeight()` is the top of the world and row `GameHeight()+1`
  // is the border ring, which is storage rather than a cell anything is in. A column that
  // reaches the top without meeting gas opens on nothing and gets no gas term -- which is the
  // honest answer for a world with no atmosphere, and on a world with one the boundary has
  // already put its gas in those cells, so reading the cell IS reading the sky.
  const int32_t top = w.GameHeight();
  for (int32_t yy = y + 1; yy <= top; ++yy) {
    const PhaseEntry& c = cells[static_cast<size_t>(yy) * pw + x];
    if (c.mass <= 0.0f) break;  // vacuum: the column is open and nothing presses on it
    const uint8_t phase = table.Phase(c.element);
    if (phase == kStateLiquid) {
      column_kg += static_cast<double>(c.mass);
      continue;
    }
    if (phase == kStateGas) {
      const float molar = table.MolecularMassOf(c.element);
      // No molar mass means no pressure can be computed from a mass. Leaving the term at zero
      // is what makes an unanswerable question defer to Klei rather than invent a number.
      if (molar > 0.0f) {
        gas_pa = gas::PressureFromMoles(c.mass * gas::kGramsPerKilogram / molar, c.temperature,
                                        g_tunables.cell_volume_m3);
      }
    }
    // Gas, solid or anything else: the liquid column ends here either way. A SOLID CEILING
    // CONTRIBUTES NOTHING, on purpose -- a sealed liquid tile has no ambient to evaluate and
    // no gas to build one, so it keeps its hydrostatic term and nothing else. For any element
    // whose freezing point sits below Klei's boiling threshold -- which is every ordinary one,
    // water at 273.15 K against 372.15 K included -- the lower clamp then puts t_sat under
    // Klei's constant and the vanilla threshold goes on governing, which is the conservative
    // answer to a question the cell cannot answer.
    break;
  }
  // A registered-rule path only (see `BoilingAllowed`), so it reads the live table directly.
  return gas_pa + static_cast<float>(column_kg * static_cast<double>(g_tunables.standard_gravity));
}

// The boil gate, and the mirror of `CondensationAllowed` above in every respect that matters:
// it returns true to LET THE GAME'S BRANCH PROCEED, it is an `&&` on a branch that has already
// fired, and it can therefore only ever SUBTRACT a transition. `rules_present` is
// `ElementTable::AnyPhaseCurves()`, hoisted out of the caller's loop for the reason that
// accessor's comment records.
//
// IT IS GATED ON THE PHASE CURVE AND NOT ON `sim.can_condense`. Whether a substance can
// evaporate is the same fact as whether it is a liquid, which is the phase test two lines
// down, so there is no second attribute here: an element with a vapour curve and a
// liquid->gas transition has a boiling point that moves with pressure, and that is all this
// needs to know.
//
// THE CURVE IS READ OFF THE GAS MEMBER, the same way `CondensationRuleOf` and
// `LatentEnergyOfTransition` read it. `RegisterFamily` pushes one record to every phase of a
// family so the liquid carries it too, but one rule for which element answers a family
// question beats two.
inline bool BoilingAllowed(const World& w, const ElementTable& table, const PhaseEntry& c,
                           int32_t x, int32_t y, uint16_t high_target, bool rules_present) {
  if (!rules_present) return true;
  if (table.Phase(c.element) != kStateLiquid) return true;
  if (static_cast<int32_t>(high_target) >= table.Count()) return true;
  if (table.Phase(high_target) != kStateGas) return true;
  ElementTable::PhaseCurve curve;
  // UNSET IS NOT ZERO, as everywhere else in this registry. An element nobody has described
  // keeps Klei's fixed threshold untouched, which is what makes the byte-identity gate safe by
  // construction: no offline scenario registers an element attribute at all.
  if (!table.PhaseCurveOf(high_target, &curve)) return true;
  const float ambient = AmbientPressureAtLiquidCellPa(w, table, x, y);
  return c.temperature > EvaporationTemperatureClampedK(curve, ambient);
}

// Which of the two pressure rules the caller has already established are worth asking about.
//
// One struct rather than two bools because both are the same kind of thing -- an O(1) "has any
// mod described anything at all" read, hoisted out of a per-cell loop -- and because a
// defaulted aggregate lets `buildings.h`'s direct callers stay exactly as they were. The two
// flags are SEPARATE TESTS AND NOT A REUSE OF ONE ANOTHER, for the reason
// `ElementTable::AnyPhaseCurves` records: an element can carry a vapour curve and no
// condensation rule, and gating either on the other would make one rule silently depend on
// which other attribute a mod happened to write.
struct PhaseRules {
  bool condensation = true;  // ElementTable::AnyCondensationRules()
  bool boiling = true;       // ElementTable::AnyPhaseCurves()
  // The tunables `TransitionCell` reads, carried here so that a kernel builds this once per
  // region and its cell loop never reads `g_tunables` (tunables.h). A caller with one cell to
  // transition default-constructs it and reads the live table then.
  float margin = g_tunables.transition_margin;
  float overshoot = g_tunables.transition_overshoot;
  float small_freeze_ore_ratio = g_tunables.small_freeze_ore_ratio;
  float ore_min_kg = g_tunables.transition_ore_min_kg;
};

// `CellAccessor::ClearDisease`, and `CellSOA::ClearDisease` is the
// same four stores. All four disease fields, including the two the sim owns rather than the
// save.
inline void ClearCellDisease(World* w, size_t cell) {
  w->MutableDiseaseIdx(cell) = 0xFF;
  w->MutableDisease(cell) = SaveDisease{};
  w->MutableDiseaseInfest(cell) = 0;
  w->MutableDiseaseAccum(cell) = 0.0f;
}

// `CellSOA::ModifyDiseaseCount`: add `delta` to the cell's germ count and, once
// the count is below one, `ClearCellDisease`. There is no zero-delta shortcut: a cell left
// holding fewer than one germ is cleared whatever was added.
inline void ModifyCellDiseaseCount(World* w, size_t cell, int32_t delta) {
  w->MutableDisease(cell).count += delta;
  if (w->Disease()[cell].count < 1) ClearCellDisease(w, cell);
}

// One cell's transition test, which is Klei's `DoStateTransition`.
//
// This is a free function rather than the body of the loop below because the loop is not
// its only caller: `BuildingHeatExchange::Update` calls `DoStateTransition` on every cell
// it has just changed the temperature of, in the same substep and before any other kernel
// sees the cell. See `sim/buildings.h`.
//
// `x`, `y` are padded coordinates. Returns true if the cell changed element.
// `out_previous`, when given, receives the element the cell held BEFORE a transition -- and
// only when this returns true. It exists so that `StepStateChange` can classify a transition
// without loading the cell itself first: this function already computes the value as
// `was_element` below, and a caller that re-read it would pay a load per cell in the sim's
// hottest small kernel to learn something the callee already knows. Defaulted, so the direct
// `buildings.h` caller is untouched.
//
// `visible` and `debug_editing` are for the two rules that hand the whole cell to the game,
// the small freeze and the condensation drop: the game has to be able to see the cell to take
// it, and a null mask sees nothing, as Klei's zeroed buffers do. `falling` is the condensation
// drop's list; null (bench, the tests) never drops.
inline void ClearCell(World* w, const ElementTable& table, size_t cell, int32_t payload_reason);

inline bool TransitionCell(World* w, const ElementTable& table, int32_t x, int32_t y,
                           std::vector<StateChangeOre>* ores,
                           PhaseRules rules = PhaseRules{},
                           uint16_t* out_previous = nullptr,
                           const uint8_t* visible = nullptr, bool debug_editing = false,
                           std::vector<SpawnFallingLiquidInfo>* falling = nullptr) {
  const int32_t pw = w->PaddedWidth();
  const size_t i = static_cast<size_t>(y) * pw + x;
  PhaseEntry& c = w->Phases()[i];
  if (c.mass <= 0.0f) return false;
  const Element& e = table.At(c.element);

  uint16_t target = kNoElement;
  float new_temperature = c.temperature;
  int32_t ore_id = 0;
  float ore_share = 0.0f;
  bool cooling = false;
  if (c.temperature > e.highTemp + rules.margin &&
      e.highTempTransitionIdx != kNoElement &&
      BoilingAllowed(*w, table, c, x, y, e.highTempTransitionIdx, rules.boiling)) {
    target = e.highTempTransitionIdx;
    new_temperature = c.temperature - rules.overshoot;
    ore_id = e.highTempTransitionOreID;
    ore_share = e.highTempTransitionOreMassConversion;
  } else if (c.temperature < e.lowTemp - rules.margin &&
             e.lowTempTransitionIdx != kNoElement &&
             CondensationAllowed(table, c, e.lowTempTransitionIdx, rules.condensation)) {
    target = e.lowTempTransitionIdx;
    new_temperature = c.temperature + rules.overshoot;
    cooling = true;
    ore_id = e.lowTempTransitionOreID;
    ore_share = e.lowTempTransitionOreMassConversion;
  }
  if (target == kNoElement || static_cast<int32_t>(target) >= table.Count()) return false;

  // This is the single choke point every natural phase
  // transition in the sim goes through -- both StepStateChange's per-tick sweep and
  // BuildingHeatExchange::Update's direct per-cell call (see this function's own file
  // comment above) end up here. `IsOpenCell` (gas_rooms.h) keys off `table.IsSolid`, so any
  // transition that crosses the solid boundary either way (ice melting to water, water
  // freezing to ice, rock sublimating away) can flip a cell's room-graph membership. Marked
  // here rather than at each caller so neither caller has to remember to.
  const uint16_t was_element = c.element;
  if (out_previous) *out_previous = was_element;
  // Layer C: liquid that freezes or boils stops carrying its payload. C3 turns this into real
  // degassing (bubbles on freezing, off-gas ahead of the boil); until then it is released here.
  if (table.Phase(was_element) == kStateLiquid && table.Phase(target) != kStateLiquid) {
    w->PayloadRelease(i, 1.0f, c.temperature, ext::kPayloadReleasedPhaseChange);
  }
  // Energy ledger: measured across the whole transition, because the temperature moves by
  // the overshoot, the mass can drop by an ore share and the element -- and with it the
  // specific heat -- is replaced, and only the cell itself can say what the three together
  // came to. The ore's own share is charged separately below and subtracted back out here.
  const double transition_before = GridCellEnergy(*w, table, i);
  double ore_energy = 0.0;
  c.temperature = new_temperature;
  // The ore share is spawned only above 1 g (`0.001f < share * mass`, both branches; the
  // tunable `TransitionOreMinKg`), and it takes the same share of the cell's germs,
  // `(int)((float)count * share)`, of the cell's germ type, which the cell then pays through
  // `ModifyDiseaseCount`.
  if (ore_id != 0 && table.HasHash(ore_id) && rules.ore_min_kg < ore_share * c.mass) {
    const float ore_mass = ore_share * c.mass;
    const int32_t ore_germs =
        static_cast<int32_t>(static_cast<float>(w->Disease()[i].count) * ore_share);
    if (ores) {
      // The ore is announced at the *unpadded* game cell, because every event in
      // GameDataUpdate is indexed the way the game indexes cells, not the way storage
      // is laid out.
      ores->push_back({(y - 1) * w->GameWidth() + (x - 1), table.IndexOfHash(ore_id),
                       ore_mass, new_temperature, w->DiseaseIdx(i), ore_germs});
    }
    ModifyCellDiseaseCount(w, i, -ore_germs);
    const double ore_before = GridCellEnergy(*w, table, i);
    c.mass -= ore_mass;
    // Charged whether or not `ores` was supplied: the cell loses the mass either way, and a
    // caller that passes null has simply thrown the announcement away.
    w->NoteOre(ore_mass);
    ore_energy = GridCellEnergy(*w, table, i) - ore_before;
    w->NoteTransitionOre(ore_energy);
  }
  // THE MELT REPORT, `DoStateTransition`'s high branch after the ore share and before
  // `ChangeSubstance`: a cell whose property byte carries `kNotifyOnMelt` (0x40, which every
  // constructed tile sets through `SimCellOccupier`) is announced in `cellMeltedInfo` at its
  // game cell, if that falls inside the world, and its unstable countdown is re-armed. The
  // game destroys the tile and clears the bit; the sim clears nothing. Only the high branch
  // tests the bit, so a flagged cell that freezes or condenses is never reported.
  if (!cooling && (w->Properties()[i] & kNotifyOnMelt) != 0) {
    const int32_t game = (y - 1) * w->GameWidth() + (x - 1);
    if (game >= 0 && static_cast<size_t>(game) < w->GameCount()) w->NoteCellMelted(game);
    w->MarkUnstableDirty(i);
  }
  // THE SMALL-FREEZE RULE, `DoStateTransition`'s low branch after the ore share: a cell that
  // freezes into a solid while holding no more than 80% of that solid's default mass
  // (`mass / defaultValues.mass <= 0.8`, against the target element's own default mass)
  // does not become a solid cell. The whole cell goes to the game as ore
  // of the solid, at the post-transition temperature and with the cell's germs, and the cell
  // is cleared. `SpawnOre` is called with `force = isDebugEditing`, so it refuses a cell the
  // game cannot see; a refused cell freezes in place like any other. Water freezing in a
  // shallow pool drops ice debris this way rather than leaving thin ice tiles.
  if (cooling && c.mass > 0.0f && table.IsSolid(target) && ores != nullptr &&
      c.mass / table.At(target).defaultValues.mass <= rules.small_freeze_ore_ratio) {
    const int32_t game = (y - 1) * w->GameWidth() + (x - 1);
    if (game >= 0 && static_cast<size_t>(game) < w->GameCount() &&
        (debug_editing || (visible != nullptr && visible[static_cast<size_t>(game)] != 0))) {
      ores->push_back({game, target, c.mass, c.temperature, w->DiseaseIdx(i),
                       w->Disease()[i].count});
      const double whole_before = GridCellEnergy(*w, table, i);
      w->NoteOre(c.mass);
      c.mass = 0.0f;
      ClearCell(w, table, i, ext::kPayloadReleasedCleared);
      const double whole = -whole_before;
      w->NoteTransitionOre(whole);
      w->NotePhaseChange(GridCellEnergy(*w, table, i) - transition_before - ore_energy - whole);
      return true;
    }
  }
  // THE CONDENSATION DROP, the same low branch when the target is NOT a solid: a gas that
  // condenses goes to the game whole, as a falling-liquid record, and the cell is cleared. It is
  // vanilla's condensation rain -- steam on a cold ceiling drips instead of leaving a water tile
  // -- and it is written inline in `DoStateTransition`, not through
  // `SimEvents::SpawnFallingLiquid`, so its gates are its own, in Klei's order:
  //
  //   * the sim is not headless, which is why every offline suite, and
  //     Klei's own DLL run headless, condense in place;
  //   * the cell below is not liquid-impermeable (`kLiquidImpermeable` is clear). Nothing asks
  //     whether it is solid, so a pocket on a natural granite floor still drips;
  //   * the game cell is inside the world and visible, or the game is in debug editing;
  //   * the new temperature (after the overshoot) is inside the target's own range widened by
  //     the transition margin each side, `lowTemp - 3 <= T <= highTemp + 3`. Klei loads 3.0
  //     once at the top of the function and uses the same register for the margin and for this
  //     window, so it is `rules.margin` here and not `FallingLiquidTemperatureWindow`.
  //
  // The record carries the target, the mass left after any ore share, the new temperature and
  // the cell's germs. Klei does not test the mass, so a cell an ore share emptied would still
  // push a zero-mass record; no element's condensation carries an ore, and this does the same.
  // The ledger prices what left as the liquid the game will hold, like `StepFlow`'s spawn.
  if (cooling && falling != nullptr && !w->Headless() && !table.IsSolid(target) &&
      !(w->Properties()[i - static_cast<size_t>(pw)] & kLiquidImpermeable)) {
    const int32_t game = (y - 1) * w->GameWidth() + (x - 1);
    const Element& te = table.At(target);
    if (game >= 0 && static_cast<size_t>(game) < w->GameCount() &&
        (debug_editing || (visible != nullptr && visible[static_cast<size_t>(game)] != 0)) &&
        te.lowTemp - rules.margin <= c.temperature &&
        c.temperature <= te.highTemp + rules.margin) {
      SpawnFallingLiquidInfo info{};
      info.cellIdx = game;
      info.elemIdx = target;
      info.diseaseIdx = w->DiseaseIdx(i);
      info.mass = c.mass;
      info.temperature = c.temperature;
      info.diseaseCount = w->Disease()[i].count;
      falling->push_back(info);
      const double particle = static_cast<double>(c.mass) *
                              static_cast<double>(te.specificHeatCapacity) * c.temperature;
      w->NoteUnstable(c.mass);
      w->NoteUnstableEnergy(-particle);
      c.mass = 0.0f;
      ClearCell(w, table, i, ext::kPayloadReleasedFalling);
      w->NotePhaseChange(particle - transition_before - ore_energy);
      return true;
    }
  }
  // A conversion of 1.0 hands the whole cell to the game and leaves nothing behind.
  if (c.mass <= 0.0f) {
    c.element = table.VacuumIndex();
    c.mass = 0.0f;
    c.temperature = 0.0f;
  } else {
    c.element = target;
  }
  w->NotePhaseChange(GridCellEnergy(*w, table, i) - transition_before - ore_energy);
  if (table.IsSolid(was_element) != table.IsSolid(c.element)) {
    w->MutableRoomsDirty() = true;
  }
  return true;
}

// KLEI'S LOAD-TIME STATE TRANSITION, `DoLoadTimeStateTransition`, gated by the
// same two pressure rules `TransitionCell` asks. Called by the `Load` handler once the blob is
// down, over the rectangle it wrote (`World::LastLoadRect`).
//
// KLEI'S RULE, AND IT IS NOT `TransitionCell`'s. Same 3 K margin and 1.5 K overshoot, but:
//
//   * no mass gate: a massless Granite cell at 5000 K loads as massless Magma;
//   * no transition ore: the whole mass stays in the cell and nothing is announced;
//   * the low side is tested first (a cell is never out of range on both, so this is order,
//     not behaviour);
//   * one step: Ice at 5000 K loads as Water at 4998.5 K and the first frame does the rest.
//
// Nothing is charged to the energy ledger or the room graph: a load is not a frame, the ledger
// has no entry for anything else `Load` rewrites, and activation rebuilds the room graph.
//
// WHY A PASS AND NOT A STEP OF `World::ReadLoadedCell`. Two reasons, and the second is the one
// that forces it. The rules (`CondensationAllowed`, `BoilingAllowed`) live here, above the World
// they read; and `BoilingAllowed` reads the column above the cell, which a per-cell reader has
// not written yet. Rows go bottom-up, so every cell above the one being decided is still as
// loaded: each decision is made against the blob, never against a neighbour's transition, and
// the result does not depend on visiting order. With no rule registered each gate returns true
// on its first compare and the pass is Klei's loop exactly, cell for cell.
//
// THE RULES MUST ALREADY BE HERE, and that is the managed side's job, not this function's. They
// are element attributes (`sim.phase_curve`, `sim.can_condense`, `sim.min_liquid_pressure`), are
// not saved, and die with every `SIM_Initialize`. A mod that pushes them only at `Game.OnSpawn`
// -- after `Load` -- gets Klei's ungated transition on every cell its rules would hold. The
// framework's `MaterialPropertyRegistry.PushToSimAtEveryOpening` pushes them in the
// registration window, before any `Load` or worldgen settle.
//
// A MIXTURE-HELD VACUUM CELL IS SKIPPED. Klei's Vacuum always arrives at 0 K; this fork's
// carries the mixture's temperature (`World::KeepMixtureTemperature`), which is no element's.
// Vacuum's `highTemp` is 0 K and its high transition is itself, so without the skip a mixture
// at 250 K would load as Vacuum at 248.5 K, and lose another 1.5 K on every load after.
inline void LoadTimeStateTransitions(World* w, const ElementTable& table) {
  const World::PaddedRect r = w->LastLoadRect();
  const int32_t pw = w->PaddedWidth();
  const PhaseRules rules{table.AnyCondensationRules(), table.AnyPhaseCurves()};
  const uint16_t count = static_cast<uint16_t>(table.Count());
  for (int32_t y = r.y0; y < r.y1; ++y) {
    for (int32_t x = r.x0; x < r.x1; ++x) {
      const size_t i = static_cast<size_t>(y) * pw + x;
      PhaseEntry& c = w->Phases()[i];
      if (c.element >= count) continue;
      if (w->MixtureHeldVacuum(i, table)) continue;
      const Element& e = table.At(c.element);
      if (c.temperature < e.lowTemp - rules.margin && e.lowTempTransitionIdx != kNoElement &&
          e.lowTempTransitionIdx < count) {
        if (CondensationAllowed(table, c, e.lowTempTransitionIdx, rules.condensation)) {
          c.temperature += rules.overshoot;
          c.element = e.lowTempTransitionIdx;
        }
      } else if (c.temperature > e.highTemp + rules.margin &&
                 e.highTempTransitionIdx != kNoElement && e.highTempTransitionIdx < count) {
        if (BoilingAllowed(*w, table, c, x, y, e.highTempTransitionIdx, rules.boiling)) {
          c.temperature -= rules.overshoot;
          c.element = e.highTempTransitionIdx;
        }
      }
    }
  }
}

// The latent energy one transition just released into, or took out of, the planet's
// atmosphere. Joules, signed: positive for a transition towards the denser phase.
//
// EVERY LATENT HEAT IS READ OFF THE PAIR'S NON-SOLID MEMBER, both ways round:
//
//   | transition      | sign | latent heat                         | read off   |
//   |-----------------|------|-------------------------------------|------------|
//   | gas -> liquid   |  +   | vaporization (`sim.phase_curve` [4]) | the gas    |
//   | liquid -> gas   |  -   | vaporization                        | the gas    |
//   | liquid -> solid |  +   | fusion (`sim.latent_fusion`)        | the liquid |
//   | solid -> liquid |  -   | fusion                              | the liquid |
//   | gas -> solid    |  +   | vaporization + fusion               | the gas    |
//   | solid -> gas    |  -   | vaporization + fusion               | the gas    |
//
// `MaterialPropertyRegistry.RegisterFamily` pushes one record to every phase of a family, so
// any member would answer; resolving through the non-solid one generalises what
// `CondensationRuleOf` above already does with the gas, and one rule for which element answers
// a family question beats two. A direct gas <-> solid step charges the sum of
// the heats of vaporization and fusion, as Stationeers does for gas that deposits straight
// to ice.
//
// A TRANSITION WHOSE LATENT HEAT IS NOT FULLY DESCRIBED CONTRIBUTES NOTHING. A sublimation on an
// element with a curve and no fusion entry is not billed its vaporization half: that would be a
// guess at a number nobody supplied, and it would also change what an older framework, which
// pushes curves and no fusion, sees from this DLL.
//
// THE MASS IS WHAT ENDED UP IN THE CELL, not what was in it before. They differ only when the
// transition hands an ore share to the game; when one does, the lump has left the cell at its
// own temperature and its latent heat went with it.
inline double LatentEnergyOfTransition(const World& w, const ElementTable& table, size_t i,
                                       uint16_t was, int32_t x, int32_t y, int32_t gw,
                                       int32_t gh, const std::vector<uint8_t>& sky) {
  const PhaseEntry& c = w.Phases()[i];
  const uint8_t was_phase = table.Phase(was);
  const uint8_t now_phase = table.Phase(c.element);
  double sign = 0.0;
  uint16_t source = kNoElement;  // the non-solid member the latent heat is read off
  bool vaporization = false;
  bool fusion = false;
  if (was_phase == kStateGas && now_phase == kStateLiquid) {
    sign = 1.0;  source = was;  vaporization = true;
  } else if (was_phase == kStateLiquid && now_phase == kStateGas) {
    sign = -1.0;  source = c.element;  vaporization = true;
  } else if (was_phase == kStateLiquid && now_phase == kStateSolid) {
    sign = 1.0;  source = was;  fusion = true;
  } else if (was_phase == kStateSolid && now_phase == kStateLiquid) {
    sign = -1.0;  source = c.element;  fusion = true;
  } else if (was_phase == kStateGas && now_phase == kStateSolid) {
    sign = 1.0;  source = was;  vaporization = true;  fusion = true;
  } else if (was_phase == kStateSolid && now_phase == kStateGas) {
    sign = -1.0;  source = c.element;  vaporization = true;  fusion = true;
  } else {
    return 0.0;
  }
  if (!(c.mass > 0.0f)) return 0.0;
  // The same padded-to-game index and the same one-frame-stale sunlight texture
  // `StepWorldEnvironment` reads, for the same reason: it is already computed, already per
  // world, and 255 steps of shadow at a 200 ms frame is not a distinction anything in the
  // game can observe.
  const int32_t gx = x - 1;
  const int32_t gy = y - 1;
  if (gx < 0 || gy < 0 || gx >= gw || gy >= gh) return 0.0;
  const size_t gi = static_cast<size_t>(gy) * static_cast<size_t>(gw) +
                    static_cast<size_t>(gx);
  if (gi >= sky.size()) return 0.0;
  const uint8_t lit_byte = sky[gi];
  if (lit_byte == 0) return 0.0;
  double latent_j_per_kg = 0.0;
  if (vaporization) {
    ElementTable::PhaseCurve curve;
    if (!table.PhaseCurveOf(source, &curve)) return 0.0;
    if (!(curve.latent_j_per_kg > 0.0f)) return 0.0;
    latent_j_per_kg += static_cast<double>(curve.latent_j_per_kg);
  }
  if (fusion) {
    float fusion_j_per_kg = 0.0f;
    if (!table.LatentFusionOf(source, &fusion_j_per_kg)) return 0.0;
    if (!(fusion_j_per_kg > 0.0f)) return 0.0;
    latent_j_per_kg += static_cast<double>(fusion_j_per_kg);
  }
  return sign * static_cast<double>(c.mass) * latent_j_per_kg *
         (static_cast<double>(lit_byte) / 255.0);
}

// THE PLANETARY LATENT ACCUMULATOR'S ONLY WRITER.
//
// In Stationeers a planet's latent-energy offset is fed only by phase changes inside the
// planet's OWN reservoirs -- its global gas mix, its clouds, its ice caps -- and nothing a base
// does feeds it.
//
// This sim has no planetary reservoirs, so the closest thing to "the planet's own atmosphere"
// is THE SKY-EXPOSED CELLS -- which is not an approximation
// chosen here but the same set `sim/environment.h`'s boundary already holds at the planet's
// pressure and composition. So a transition counts when all of this is true:
//
//   * the world declares a surface atmosphere (a `kSetWorldEnvironment` boundary). A world
//     that is not a planet has no atmosphere for latent heat to land in;
//   * the cell has a path to the sky, weighted by the same lit fraction the boundary uses as
//     its coupling. Steam condensing three metres under water is not warming the planet, and
//     the beam has already been attenuated by exactly the medium that should attenuate it;
//   * the transition's latent heat is fully described: `sim.phase_curve`'s enthalpy of
//     VAPORIZATION for gas <-> liquid, `sim.latent_fusion` for liquid <-> solid, both for a
//     direct gas <-> solid step (`LatentEnergyOfTransition` above). A melt or a freeze on
//     an element with no fusion entry contributes nothing.
//
// AND IT IS THIS KERNEL ONLY, not `TransitionCell` itself, even though that function is the
// choke point every transition in the sim goes through. `buildings.h` calls it directly on a
// cell a machine has just changed the temperature of, and that is a colony transition, not a
// planetary one, and Stationeers does not count those either.
//
// NOTHING HERE TOUCHES A CELL. The accumulator is a number beside the grid, so the whole of
// this is invisible to the byte-identity gate by construction, and inert on every world with
// no environment record -- which is every world in every offline scenario.
inline void StepStateChange(World* w, const ElementTable& table,
                            std::vector<StateChangeOre>* ores, size_t ri,
                            const std::vector<uint8_t>* sky = nullptr,
                            const uint8_t* visible = nullptr, bool debug_editing = false,
                            std::vector<SpawnFallingLiquidInfo>* falling = nullptr) {
  ONI_SWEEP(kStateChange);
  const World::PaddedRect& r = w->PaddedRegion(ri);
  const int32_t pw = w->PaddedWidth();
  // Once per region, not once per cell. Both false for every world nobody has pushed element
  // attributes to, which short-circuits each rule to a register compare.
  const bool any_phase_curves = table.AnyPhaseCurves();
  const PhaseRules rules{table.AnyCondensationRules(), any_phase_curves};
  // The latent accumulator's gate, hoisted the same way and for the same reason. A world with
  // no boundary declared is not a planet, and the per-cell work below never starts. It shares
  // `AnyPhaseCurves()` with the boil rule above -- one read, two consumers, and they ask the
  // same question of the same attribute -- and adds `AnyLatentFusion()`, because a table that
  // describes only fusion enthalpies still has freezes to count.
  const int32_t latent_world = w->WorldIndexOfRegion(ri);
  const World::Environment& env = w->EnvironmentOfWorld(latent_world);
  const bool wants_latent = latent_world >= 0 && sky != nullptr && !sky->empty() &&
                            env.boundary_element >= 0 && env.surface_pressure_kpa > 0.0f &&
                            (any_phase_curves || table.AnyLatentFusion());
  const int32_t gw = w->GameWidth();
  const int32_t gh = w->GameHeight();
  double latent_j = 0.0;
  // The rectangle, not the grid: the `Active` test this replaces was the loop's only gate,
  // so the visited set and its order are unchanged. See `World::PaddedRegions`.
  {
    for (int32_t y = r.y0; y < r.y1; ++y) {
      ONI_EXAMINED(kStateChange, r.x1 - r.x0);
      for (int32_t x = r.x0; x < r.x1; ++x) {
        const size_t i = static_cast<size_t>(y) * pw + x;
        // `DoStateTransition` announces, on the transitioning cell only, and
        // only once it has got past the `mass > 0` gate — a massless cell
        // is never announced by this kernel.
        // ACTIVE == the cell changed element. `TransitionCell` returning true is exactly that
        // claim already: it is the condition the kernel itself uses to decide whether the
        // substance moved, so the counter cannot drift from the behaviour it describes.
        // No explicit change count: `TouchSubstance` is the census's change site, and this
        // kernel already calls it on exactly the cells that transitioned.
        // EXACTLY ONE CALL SITE, and that is a measured constraint rather than a style
        // preference. `TransitionCell` is large and inlined; two calls to it inside this
        // loop body took the kernel from 0.163 ms to 0.465 ms on the asteroid scenario, and
        // splitting the latent variant into a second loop in this same function was worse
        // still at 0.40 ms. Hence `out_previous`: the callee hands back what it already
        // knows, so the planet's accounting costs a stack slot the optimiser can delete on
        // the path that ignores it, and nothing per cell.
        uint16_t was = kNoElement;
        if (!TransitionCell(w, table, x, y, ores, rules, &was, visible, debug_editing, falling)) {
          continue;
        }
        w->TouchSubstance(i);
        if (wants_latent) {
          latent_j += LatentEnergyOfTransition(*w, table, i, was, x, y, gw, gh, *sky);
        }
      }
    }
  }
  if (latent_j != 0.0) w->NoteWorldLatentEnergy(latent_world, latent_j);
}

// ------------------------------------------------------------------------ sublimation
//
// The other way an element changes, and the only one that is not driven by temperature.
// Five elements in the table carry a `sublimateIndex`; two of them at probability 1.0,
// which makes those two reproducible offline.
//
// Measured on a solid with rate 0.4, efficiency 0.5 and probability 1.0, sealed in granite
// with a varying number of empty neighbours:
//
//   * **Per free neighbour**, not per cell. One free neighbour took 0.08 kg a frame off
//     the source; four free neighbours took 0.32 kg. Each neighbour received 0.04 kg.
//   * The amount is `sublimateRate * 0.2`, i.e. a rate per second against the substep, and
//     it does not depend on the source's mass at all — 100 kg and 50 kg both lost 0.08 kg.
//   * `sublimateEfficiency` is a **loss**: 0.08 kg leaves the source and 0.04 kg arrives.
//     The rest is destroyed. This is the one place in the sim measured to not conserve
//     mass, and it is deliberate on Klei's part — it is how a kilogram of Oxylite becomes
//     half a kilogram of oxygen.
//   * Fully sealed in solid, nothing happens at all. The source does not lose mass it
//     cannot place.
//
// It runs *after* the transition pass. The proof is an accident of the table: this
// element's product is a liquid that freezes below 300 K, so at room temperature the
// product cell oscillates — every frame the transition pass turns the 0.04 kg into a solid,
// and every frame the sublimation pass displaces that solid as an ore drop and refills the
// cell with 0.04 kg of liquid. The cell reads a steady 0.04 kg with one ore drop a frame,
// which only happens in that order.
//
// `sublimateProbability < 1` used to be skipped, on the grounds that a coin toss the
// harness cannot predict turns any scenario containing one into noise. That is no longer
// true: the coin is Klei's `rand()`, the stream is reproducible, and this kernel now draws
// from it. See "the post-process sweep" below for why the draws matter beyond sublimation.
//
// The shape above has one subtlety: **the free cell drives this, not the solid**. `DoSublimation` is called from `PostProcessCell`'s gas and vacuum
// branches only, it bails unless the *free* cell holds less than 1.8 kg, and it scans that
// cell's four neighbours looking for solids. Liquids never sublimate anything. Every
// measurement in the comment above is a measurement of the same thing seen from the other
// side, so none of them changes; what changes is which cells draw, and when.

// ------------------------------------------------------------- the post-process sweep
//
// `SimBase::UpdateData`'s second pass: `for y: for x: PostProcessCell(y*width + x)`, plain
// and sequential, over the same active region every other kernel uses. It is where
// sublimation, density displacement and the random gas shuffle all live, and they have to
// share one function because they share **one random stream**, the game's `rand()`. Getting
// the shuffle right is not only a matter of getting its rule right; it is a matter of every
// cell drawing exactly as many numbers as the game's does, in the same order. A draw skipped anywhere
// shifts every later cell in the world onto the wrong number.
//
// So the branch structure below is Klei's, including the parts whose *effects* are not
// modelled — a branch that consumes no draws still has to be taken.
//
//   state & 3 == 0  vacuum  ->  DoSublimation, unless property bit 0 is set
//   state & 3 == 1  gas     ->  negate the stride, then the main path, then DoSublimation
//   state & 3 == 2  liquid  ->  its own branch; draws once for off-gassing (not modelled)
//   state & 3 == 3  solid   ->  nothing at all unless it is Unstable, and then no draws
//
// The gas main path, in order: a mass gate at 1e-9 and another at 0.001 that both skip
// straight to sublimation without drawing; `DoDensityDisplacement` (one conditional draw);
// four `DoPartialMelt` calls that never draw; the shuffle (one gate draw, up to three
// candidate draws); `DoSublimation` (one draw per solid neighbour). Any of the first three
// returning true skips the rest and goes straight to sublimation.

// `((state >> 16) & 0x7FFF) * 3.0518509e-05`. `scale` is the tunable `RandomScale`, whose
// default is written as its bits (0x38000100) because it is the float *nearest* 1/32767 rather
// than the quotient, and the difference decides coin flips at the 0.5 and 0.9 gates.
inline float NextRandom(World* w, float scale) {
  const uint32_t s = w->NextRandomState();
  return static_cast<float>((s >> 16) & 0x7FFFu) * scale;
}

// `DoDensityDisplacement`. The draw happens only when the cell below has the
// **same state byte** — the whole byte, so Unstable counts — which is what made a fit
// assuming a fixed number of draws per cell fail. Probability 0.99 from the gas branch and
// 0.30 from the liquid one.
//
// Note it selects on **temperature**, not mass: same element, and the cell below hotter,
// swaps the two. This is Klei's convection, and it is the second thing here that was
// written down as "mass" from measurement and turned out to be temperature.
//
// `src_element` is the element the *caller* captured, which is not always the element the
// cell now holds: Klei loads the source's `ElementPostProcessData` pointer once at the top
// of `PostProcessCell` and passes it down, so a cell that `Evaporate` has since cleared to
// vacuum is still tested and moved as the gas it used to be. Only the same-element test
// reads the live grid; the state byte, the molar mass and the liquid-equalise test all read
// the captured copy.
// `CellSOA::SwapCells`. Every per-cell field the sim owns, which is the phase
// triple and all four disease fields — not the static ones, which belong to the map rather
// than to the matter standing in it.
//
// It lives here rather than in `cellmod.h`, where it was written, because the five swap sites
// **below** in this file need it: they had all been spelling it `std::swap` on the phase
// entry alone and dropping the disease on the floor. Between two cells of the same element
// the phase triple is identical, so that omission was invisible in every field the harness
// compares except the germs — and both germ scenarios are solid granite, so no swap site in
// the suite had ever run with a germ under it.
inline void SwapCells(World* w, size_t a, size_t b) {
  std::vector<PhaseEntry>& cells = w->Phases();
  const PhaseEntry pa = cells[a];
  cells[a] = cells[b];
  cells[b] = pa;
  // Layer C: a liquid payload is the liquid's, so it goes where the liquid goes.
  w->PayloadSwap(a, b);
  const uint8_t di = w->DiseaseIdx(a);
  w->MutableDiseaseIdx(a) = w->DiseaseIdx(b);
  w->MutableDiseaseIdx(b) = di;
  const SaveDisease d = w->Disease()[a];
  w->MutableDisease(a) = w->Disease()[b];
  w->MutableDisease(b) = d;
  const uint8_t inf = w->MutableDiseaseInfest(a);
  w->MutableDiseaseInfest(a) = w->MutableDiseaseInfest(b);
  w->MutableDiseaseInfest(b) = inf;
  const float acc = w->MutableDiseaseAccum(a);
  w->MutableDiseaseAccum(a) = w->MutableDiseaseAccum(b);
  w->MutableDiseaseAccum(b) = acc;
  w->MarkProjectDirty(a);
  w->MarkProjectDirty(b);
}

inline bool DoDensityDisplacement(World* w, const ElementTable& table, size_t cell,
                                  size_t below, float probability, uint16_t src_element,
                                  const Tunables& tun) {
  std::vector<PhaseEntry>& cells = w->Phases();
  const Element& e = table.At(src_element);
  const Element& b = table.At(cells[below].element);
  if (b.state != e.state) return false;
  if (NextRandom(w, tun.random_scale) <= probability) return false;
  if (e.molarMass > b.molarMass) {
    SwapCells(w, cell, below);
    // Announced. The convection swap at the bottom of this function is **not**: it is a bare
    // `SwapCells` with nothing after it but the return. The game is inconsistent here — the
    // shuffle announces its same-element swaps and this one does not — so the inconsistency
    // has to be copied rather than tidied.
    w->TouchSubstance(cell);
    w->TouchSubstance(below);
    return true;
  }
  if (cells[below].element != cells[cell].element) return false;
  if (cells[below].temperature <= cells[cell].temperature) return false;
  if ((e.state & kStateMask) == kStateLiquid) {
    // Liquid equalises instead of swapping: both cells take the combined temperature and
    // no mass moves. `FastCalculateCombinedTemperature` is a mass-weighted
    // mean **clamped back between its two inputs**, the same shape as
    // `AddMassAndUpdateTemperature`'s clamp and for the same reason — the division can
    // land a last bit outside the interval its own operands span, and one ulp here is
    // enough to change which pair `GasShuffle` swaps three ticks later.
    const float t0 = cells[cell].temperature;
    const float t1 = cells[below].temperature;
    const float m0 = cells[cell].mass;
    const float m1 = cells[below].mass;
    const float mix = (m0 * t0 + m1 * t1) / (m0 + m1);
    // An unordered compare keeps `t1`; the second test keeps `mix` only when `mix >= t0`.
    // Reached only when `t1 > t0`, so this is a clamp to `[t0, t1]` written the long way.
    const float t = !(t1 >= mix) ? t1 : (mix >= t0 ? mix : t0);
    cells[cell].temperature = t;
    cells[below].temperature = t;
    // Layer C: the two cells have just mixed, so their payloads mix too, split by mass. This is
    // the convective turnover that keeps a pond from staying stratified. Moves only between
    // the pair, so the total is unchanged.
    if (w->PayloadActive() && m0 + m1 > 0.0f) {
      const float share_below = m1 / (m0 + m1);
      for (const World::PayloadProperty& p : w->PayloadProperties()) {
        float* a = p.data + cell * static_cast<size_t>(p.arity);
        float* bb = p.data + below * static_cast<size_t>(p.arity);
        for (int32_t k = 0; k < p.arity; ++k) {
          const float total = a[k] + bb[k];
          if (total == 0.0f) continue;
          bb[k] = total * share_below;
          a[k] = total - bb[k];
        }
      }
    }
    return true;
  }
  SwapCells(w, cell, below);
  return true;
}

// ------------------------------------------------------------------ the gas wisp path
//
// The first thing a gas cell does, before `DoDensityDisplacement` and before the shuffle,
// is ask whether it still counts as gas at all. Three thresholds:
//
//   mass < 1e-9   `Evaporate` outright, no questions asked.
//   mass < 0.001   a wisp: look at the four neighbours, and if any of them
//                                is a gas holding at least 1.0 kg, the wisp
//                                is `Evaporate`d too. Otherwise it stays.
//
// `Evaporate` is `SimData::ClearCell` plus a `ChangeSubstance` event, and
// `ClearCell` writes the vacuum element, zero mass and **zero temperature**.
// The wisp's mass is not given to the neighbour that justified deleting it — it is simply
// gone. That is Klei, not an approximation here: a falling liquid leaks a little gas mass
// out of the world on every substep.
//
// Two things about this path that decide the random stream rather than the physics, both
// the reason it is modelled at all:
//
//  1. **It never draws and it never exits.** Both evaporation sites fall straight through
//     into `DoDensityDisplacement` — there is no `jmp` past it. The cell
//     goes on to take its ordinary turn as though nothing had happened.
//  2. **The cell it leaves behind is vacuum, and the cell above it can see that.** This is
//     the whole reason the path matters: a vacuum cell is not the same state byte as a gas
//     one, so the neighbour above spends no `DoDensityDisplacement` draw next time round.
//     Leaving the wisp in place as gas made our sim draw *more* often than Klei's on every
//     scenario with a liquid falling through an atmosphere.
//
// The four neighbours are `cell - dir`, `cell + dir`, below and above — the whole
// 4-neighbourhood, so the stride only decides the order, and since no candidate draws or
// mutates anything the order cannot be observed.
inline void EvaporateWisp(World* w, const ElementTable& table, size_t cell, int32_t dir,
                          int32_t pw, const Tunables& tun) {
  std::vector<PhaseEntry>& cells = w->Phases();
  const float mass = cells[cell].mass;
  if (mass >= tun.gas_wisp_mass_kg) return;
  if (mass >= 1e-9f) {
    const ptrdiff_t offsets[4] = {-static_cast<ptrdiff_t>(dir), dir,
                                  -static_cast<ptrdiff_t>(pw), pw};
    bool merged = false;
    for (int k = 0; k < 4 && !merged; ++k) {
      const size_t n = static_cast<size_t>(static_cast<ptrdiff_t>(cell) + offsets[k]);
      if (cells[n].mass < 1.0f) continue;
      if ((table.At(cells[n].element).state & kStateMask) == kStateGas) merged = true;
    }
    if (!merged) return;
  }
  w->NoteWisp(mass);
  w->NoteWispEnergy(-GridCellEnergy(*w, table, cell));
  w->PayloadRelease(cell, 1.0f, cells[cell].temperature, ext::kPayloadReleasedWisp);
  cells[cell].element = table.VacuumIndex();
  cells[cell].mass = 0.0f;
  // A mixture-occupied cell keeps its temperature, as in ZeroMasslessCells and ClearCell:
  // this runs from StepPostProcess's gas branch BEFORE ClearCell ever gets a turn, so a
  // promoted cell's vanilla mass draining below the wisp threshold reaches this first. Same
  // guard, same reasoning -- GasOccupiedMask is always allocated and reads 0 for any world
  // that never activates volume-fractions, so this stays byte-identical there.
  if (w->GasOccupiedMask(cell) == 0) {
    cells[cell].temperature = 0.0f;
  }
  // `Evaporate` is `ClearCell` followed by an unconditional
  // `ChangeSubstance`, so every cell either evaporation path deletes is
  // announced. The gas branch reaches it.
  w->TouchSubstance(cell);
}

// The shuffle. One gate draw at 0.9, then three candidates in a fixed order carrying a
// "must be denser" flag on the third.
//
// Two details decide the stream. The temperature test comes **before** the candidate's own
// draw, so a candidate rejected on temperature consumes nothing; and `best` starts at
// FLT_MAX and only ever tightens, so the scan keeps the coolest candidate that also passes
// its coin flip. The third candidate is the cell below and additionally needs the source to
// be strictly heavier by molar mass — which is the whole reason two cells of the same gas
// never density-sort.
inline void GasShuffle(World* w, const ElementTable& table, size_t cell, int32_t dir,
                       int32_t pw, uint16_t src_element, const Tunables& tun) {
  if (NextRandom(w, tun.random_scale) <= tun.gas_shuffle_skip_gate) return;
  std::vector<PhaseEntry>& cells = w->Phases();
  // The captured element, for the same reason `DoDensityDisplacement` takes one.
  const Element& src = table.At(src_element);
  const ptrdiff_t offsets[3] = {dir, -dir, -static_cast<ptrdiff_t>(pw)};
  const bool needs_denser[3] = {false, false, true};

  float best = FLT_MAX;
  size_t winner = 0;
  bool have = false;
  for (int k = 0; k < 3; ++k) {
    const size_t n = static_cast<size_t>(static_cast<ptrdiff_t>(cell) + offsets[k]);
    const float t = cells[n].temperature;
    if (t > best) continue;
    if (NextRandom(w, tun.random_scale) <= tun.gas_shuffle_candidate_coin) continue;
    const Element& ne = table.At(cells[n].element);
    if ((ne.state & kStateMask) != kStateGas) continue;
    if (needs_denser[k] && !(src.molarMass > ne.molarMass)) continue;
    best = t;
    winner = n;
    have = true;
  }
  if (have) {
    SwapCells(w, cell, winner);
    // Winner first and then the cell itself. The order does not survive publication — the
    // game sorts the list by cell index — but both entries do,
    // and they are the single largest source of substance traffic in any room with gas in
    // it. Almost all of them read `183 -> 183`.
    w->TouchSubstance(winner);
    w->TouchSubstance(cell);
  }
}

// ------------------------------------------------------------------- clearing and shoving
//
// `SimData::ClearCell`: vacuum element, no mass, no temperature, no disease.
// It does not announce — every caller that wants the cell announced does so itself, which
// is why `Evaporate` below is two lines rather than one.
inline void ClearCell(World* w, const ElementTable& table, size_t cell,
                      int32_t payload_reason = ext::kPayloadReleasedCleared) {
  std::vector<PhaseEntry>& cells = w->Phases();
  // Layer C. A caller that means the liquid to go somewhere moves the payload first; whatever is
  // still here when the cell is cleared left the grid with it.
  w->PayloadRelease(cell, 1.0f, cells[cell].temperature, payload_reason);
  // Most callers reach here having already moved the cell's mass somewhere, so this charges
  // nothing. The ones that do not are the point: `cleared` is where a deletion nobody
  // thought of shows up, instead of quietly becoming drift with no name on it.
  if (cells[cell].mass > 0.0f) {
    w->NoteCleared(cells[cell].mass);
    // Energy ledger, same rule and the same site as the mass above, so the two divide into
    // each other. Charged HERE rather than at the callers deliberately: this is the one
    // place every clearing path passes through, and every guard elsewhere in the sim
    // (cellmod.h's CellEnergyCharge, DoDisplacement's pair) is placed to stop short of it
    // so that nothing is counted twice.
    w->NoteClearedEnergy(-GridCellEnergy(*w, table, cell));
  }
  cells[cell].element = table.VacuumIndex();
  cells[cell].mass = 0.0f;
  // The gas-mixture-occupancy check, as in ZeroMasslessCells (`Evaporate` calls this from
  // `PostProcessCell` once a gas cell's vanilla mass falls under 1e-9). `GasOccupiedMask` is
  // always allocated (World::Allocate) and reads 0 for every world that never activates
  // volume-fractions, so this is byte-identical there. Only the temperature is guarded --
  // element/mass/disease still clear normally even for a mixture-occupied cell, since
  // vacuum/zero-mass IS the correct vanilla-frozen state for a promoted gas cell; only
  // stomping its shared temperature field to 0 K was ever wrong.
  if (w->GasOccupiedMask(cell) == 0) {
    cells[cell].temperature = 0.0f;
  }
  w->MutableDiseaseIdx(cell) = 0xFF;
  w->MutableDisease(cell) = SaveDisease{};
  // The two sim-owned disease fields go with it:, the last two
  // stores in `ClearCell`. Leaving the growth remainder behind would let a cleared cell
  // hand a fraction of a germ to whatever moves in next.
  w->MutableDiseaseInfest(cell) = 0;
  w->MutableDiseaseAccum(cell) = 0.0f;
}

// `Evaporate` is `ClearCell` plus one unconditional `ChangeSubstance`.
inline void Evaporate(World* w, const ElementTable& table, size_t cell) {
  ClearCell(w, table, cell);
  w->TouchSubstance(cell);
}

// `DisplaceGas`. The game calls it from many places, and this project reaches three
// of them: the liquid mover, `DoSublimation` and the liquid off-gas block. It empties a gas
// cell into a neighbour so the caller can have it, and reports whether that was possible —
// a caller that gets `false` back must abandon whatever it was going to write.
//
// It tries the four orthogonal neighbours in the fixed order **up, left, right, down**,
// then the two upward diagonals, and it begins each of those two scans at a **rotating
// offset**, the game's substep counter: a `uint16` that starts at zero and is incremented once
// at the very end of each substep, so during substep *N* the orthogonal scan starts at `N % 4` and the
// diagonal scan at `N % 2`. `rotation` is ours, stepped in lockstep.
//
// The two scans do not test the same thing. An orthogonal neighbour is taken if it holds the
// same gas **or is vacuum**; a diagonal is taken only if it holds the same gas, and
// additionally only if the cell horizontally between it and the source is not solid
// — there has to be a way round the corner. Both scans then require the
// candidate's own `GasImpermeable` bit to be clear, and a
// candidate that fails any test is skipped rather than ending the scan.
//
// Two gates of its own, before any of that: the cell must hold mass and must
// be **gas**. The first is why a mover that has already
// emptied a cell this substep cannot displace it again.
//
// `start` and `region_gated` are not Klei's — its only bound is the array's own length. They
// exist because every measurement behind the liquid path was taken with the liquid sweep's
// `interior` in place, and nothing yet separates the two readings. The sublimation callers
// pass `nullptr` and `false`, as the game does.
// `rooms`: the LIQUID promotion gate, and it is optional here for a reason worth stating.
// This helper is reached from four places -- the liquid flow mover (`StepFlow`), the liquid
// pressure displacement (`DisplaceLiquidDirectional`), `DoSublimation` and the liquid off-gas
// block. Only the first two pass a graph today, because only those two are the liquid kernels
// this commit gates. The other two still pass `nullptr` and so can still choose a
// mixture-owned cell as the target a displaced gas lands in; that is a KNOWN REMAINING PATH,
// named here rather than silently closed, because closing it belongs to sublimation rather
// than to the liquid gate.
// `diseases`: the displaced gas takes its germs with it. `DoDisplacement` hands the
// whole cell's live germ type and count to `AddMassAndUpdateTemperature`, which merges them
// into the cell the gas lands in; `ClearCell` then empties the source's. A caller with no
// table (the scaffolding drivers) still clears them, which is what every caller did before.
inline bool DisplaceGas(World* w, const ElementTable& table, size_t dst, uint16_t rotation,
                        const std::vector<PhaseEntry>* start, bool region_gated,
                        const gas::RoomGraph* rooms = nullptr,
                        const DiseaseTable* diseases = nullptr) {
  std::vector<PhaseEntry>& cells = w->Phases();
  const std::vector<uint8_t>& props = w->Properties();
  const int32_t pw = w->PaddedWidth(), ph = w->PaddedHeight();
  const int32_t dst_x = static_cast<int32_t>(dst % static_cast<size_t>(pw));
  const int32_t dst_y = static_cast<int32_t>(dst / static_cast<size_t>(pw));

  // Promotion gate, before anything else: a cell the mixture owns is neither emptied by this
  // helper nor used as somewhere to put gas. `rooms == nullptr` (every caller that does not
  // pass one, and every world that never promotes) short-circuits inside
  // `IsMixtureOwnedCell` before touching `room_of`.
  if (gas::IsMixtureOwnedCell(*w, rooms, dst)) return false;
  const PhaseEntry& d = cells[dst];
  if (!(d.mass > 0.0f)) return false;
  if (table.Phase(d.element) != kStateGas) return false;

  size_t target = 0;
  bool have_target = false;
  auto solid_at = [&](size_t c) { return table.Phase(cells[c].element) == kStateSolid; };
  auto consider = [&](int32_t cx, int32_t cy, bool allow_vacuum) {
    if (have_target) return;
    if (cx < 1 || cx >= pw - 1 || cy < 1 || cy >= ph - 1) return;
    const size_t c = static_cast<size_t>(cy) * pw + cx;
    // The one membership test in the file that is still the mask rather than a rectangle.
    // `DisplaceGas` is reached from the liquid mover as well as from post-process, so it has
    // no single rectangle to be handed, and no scenario exercises it at a region seam — the
    // two-room worlds have nothing that sublimates. Left as the union deliberately: making it
    // a rectangle would be a guess with no detector behind it. If a probe ever puts a
    // sublimating solid on a seam, this is the line it will be about.
    if (region_gated && !w->Active(c)) return;
    // The target half of the same gate. Without it a displaced gas lands in a promoted cell
    // by the `allow_vacuum` arm below -- a mixture-owned cell reads as vacuum in vanilla's
    // grid on purpose (`ClearCell`), so it would otherwise look like the ideal place to put
    // something.
    if (gas::IsMixtureOwnedCell(*w, rooms, c)) return;
    if (solid_at(c)) return;
    if (start != nullptr && table.Phase((*start)[c].element) == kStateSolid) return;
    const PhaseEntry& p = cells[c];
    // Klei compares *element indices* — same gas, or the vacuum element — and that is not
    // the same test as "holds no mass". A cell this sweep has already emptied to make room
    // for liquid has zero mass but is not vacuum, and accepting it put the displaced gas
    // straight back under the water: `pour` had 127 kg where Klei had 125, the extra two
    // kilograms being oxygen merged into the water cell by the end-of-sweep add, while Klei
    // had carried the same 3 kg three cells up. That one test is worth 185 kg on `pour`
    // over 50 ticks.
    if (!(p.element == d.element || (allow_vacuum && table.IsVacuum(p.element)))) return;
    if (props[c] & kGasImpermeable) return;
    target = c;
    have_target = true;
  };

  const int32_t ox[4] = {0, -1, 1, 0};
  const int32_t oy[4] = {1, 0, 0, -1};
  for (int k = 0; k < 4; ++k) {
    const int r = (rotation + k) & 3;
    consider(dst_x + ox[r], dst_y + oy[r], true);
  }
  // The upward diagonals, which is where the gas goes when a falling slug has sealed every
  // orthogonal way out: measured on `pair` in oxygen, where the kilogram displaced from
  // under the water turns up one cell up and to the side.
  for (int k = 0; k < 2; ++k) {
    const int r = (rotation + k) & 1;
    const int32_t side = r == 0 ? dst_x - 1 : dst_x + 1;
    if (side < 1 || side >= pw - 1) continue;
    if (solid_at(static_cast<size_t>(dst_y) * pw + side)) continue;
    consider(side, dst_y + 1, false);
  }
  if (!have_target) return false;

  // `DisplaceGas` hands the pair to `DoDisplacement`, which is one call to
  // `AddMassAndUpdateTemperature` with the whole of the displaced cell as the
  // amount — so the clamp applies here too — then copies the element across, clears the
  // source and announces **both** ends,. The cell the gas
  // arrives in almost always already held the same gas, so it reads `183 -> 183` and is
  // invisible in the grid; it is only visible in the message list.
  // Energy ledger: this pair does not conserve energy, and both reasons are Klei's.
  // `AddMassAndUpdateTemperature` mixes the two temperatures weighted by MASS rather than by
  // heat capacity, and `onward.element` is overwritten with the source's straight afterwards
  // -- so whenever the two cells hold different elements the specific heat capacity under the
  // merged mass changes and nothing pays for it. Charged over both cells together, across the
  // whole transfer including the source's clear, since only the pair's sum is meaningful.
  const double pair_before =
      GridCellEnergy(*w, table, target) + GridCellEnergy(*w, table, dst);
  PhaseEntry& onward = cells[target];
  PhaseEntry& moving = cells[dst];
  const float onward_before = onward.mass;
  AddMassAndUpdateTemperature(&onward, moving.mass, moving.temperature);
  // The disease half of the same call: the moving cell's live germs, all of
  // them, merged into the cell it lands in. Its total is positive (the moving cell holds
  // mass, gated above), so this is the merge branch, never the clear one.
  if (diseases != nullptr && w->DiseaseActive()) {
    AddDiseaseToCell(w, *diseases, target, w->DiseaseIdx(dst), w->Disease()[dst].count);
  }
  onward.element = moving.element;
  // Debit the source with what actually arrived before blanking it. `ClearCell` would zero
  // the field either way, so this changes nothing the sim can observe — but the ledger
  // reads the field, and left alone it would charge this whole transfer to `cleared` as if
  // the gas had been deleted. What survives the subtraction is the rounding of
  // `onward.mass + moving.mass`, which is a real if microscopic loss and belongs there.
  moving.mass -= onward.mass - onward_before;
  // Charged before the clear, not after: whatever the subtraction above left standing in the
  // source is a real deletion and belongs to `cleared`, which `ClearCell` charges itself.
  w->NoteDisplaced(GridCellEnergy(*w, table, target) + GridCellEnergy(*w, table, dst) -
                   pair_before);
  ClearCell(w, table, dst);
  w->TouchSubstance(target);
  w->TouchSubstance(dst);
  return true;
}

// `SimEvents::SpawnFallingLiquid`: hand `mass` of `element` to the game as a
// falling-liquid particle at `at`, and report whether it was taken. It takes nothing out of
// the grid itself; each caller does that on a true return, and does something else on a
// false one. It refuses, in Klei's order:
//
//   * when the sim is headless (the flag `UnstableCheckBasic` reads).
//     Every offline suite sends headless, which is why none of them could see this path;
//   * when the cell below `at` is liquid-impermeable (`kLiquidImpermeable`);
//   * when `at` is not a game cell;
//   * when the game cannot see `at` and is not in debug editing (`visibleGrid`, as the
//     solid emitter's ore drop reads it);
//   * when `temperature` is outside the element's own liquid range, widened by 3 K each
//     side: `lowTemp - 3 <= T <= highTemp + 3`.
//
// The record is pushed in call order, and `CopySimDataToGame` hands the list over by swap:
// unlike `spawnOreInfo` it is neither sorted nor merged. Two sweeps call it: the liquid mover
// (`StepFlow`) and post-process's partial melt (`DoPartialMelt`), in that order within a
// substep. A null `falling` never spawns, which is what `bench` passes.
inline bool SpawnFallingLiquid(World* w, const ElementTable& table,
                               std::vector<SpawnFallingLiquidInfo>* falling,
                               const uint8_t* visible, bool debug_editing, size_t at,
                               uint16_t element, float mass, float temperature,
                               uint8_t disease_idx, int32_t disease_count,
                               const Tunables& tun) {
  if (falling == nullptr || w->Headless()) return false;
  const std::vector<uint8_t>& props = w->Properties();
  if (props[at - static_cast<size_t>(w->PaddedWidth())] & kLiquidImpermeable) return false;
  const int64_t game = w->GameIndex(at);
  if (game < 0 || static_cast<size_t>(game) >= w->GameCount()) return false;
  if (!debug_editing && visible != nullptr && visible[static_cast<size_t>(game)] == 0) {
    return false;
  }
  const Element& fe = table.At(element);
  if (!(fe.lowTemp - tun.falling_liquid_temperature_window <= temperature)) return false;
  if (!(temperature <= fe.highTemp + tun.falling_liquid_temperature_window)) return false;
  SpawnFallingLiquidInfo info{};
  info.cellIdx = static_cast<int32_t>(game);
  info.elemIdx = element;
  info.diseaseIdx = disease_idx;
  info.mass = mass;
  info.temperature = temperature;
  info.diseaseCount = disease_count;
  falling->push_back(info);
  return true;
}

// `DoPartialMelt`: a hot gas cell melts 5 kg off the face of a solid beside it
// when it holds enough heat to do so, without waiting for the whole solid to cross its
// transition. `PostProcessCell` calls it for the cell below, left, right and above, in that
// order, and the first to return true ends the run. Every read is of the LIVE grid.
//
// It does nothing unless, in Klei's order:
//
//   * `gas` is gas or vacuum (`state & 3 <= 1`) and `solid` is solid (`== 3`);
//   * the solid holds more than 5 kg (strictly: `5 >= mass` refuses);
//   * the solid has neither `kUnbreakable` (8) nor `kNotifyOnMelt` (0x40) set;
//   * `T_gas > highTemp + 3` and `T_solid < highTemp - 3`, both strict;
//   * the solid has a high transition, and that element is a liquid;
//   * the gas can pay for it: `(T_gas - (gas.lowTemp + 6)) * (m_gas * c_gas)` is at least
//     `needed = (highTemp + 3 - T_solid) * c_liquid * 5`. The float operations are grouped
//     exactly as Klei's, because `needed` and the gas's heat capacity are reused below.
//
// Then it asks the game to take 5 kg of the liquid at `highTemp + 3` as a falling particle
// at the GAS cell, with the solid's germ type and `(int)(5 / m_solid * germs)` of its germs.
//
//   * Taken: the gas cools to `(C T_gas - needed) / C`, the solid loses 5 kg and those germs
//     (a bare subtract, not `ModifyDiseaseCount`), and a solid left at or under FLT_MIN is
//     `Evaporate`d. Nothing else is announced.
//   * Refused (always, headless): the gas is `DisplaceGas`'d out of the way, and if that
//     fails nothing happens and the call returns false. Otherwise the gas cell BECOMES the
//     liquid, 5 kg at `highTemp + 3`, the solid loses 5 kg (its germs stay), the same
//     `Evaporate` test runs, and the gas cell is announced. Klei writes the cooled gas
//     temperature into the cell first and then overwrites it; only the second store is kept.
//
// Neither branch conserves energy, both for Klei's reasons: the 5 kg changes specific heat
// as it melts, and the refusal branch spends nothing of the gas's heat at all. The ledger
// charges the difference to `phase_change`, and the particle that leaves to `unstable`.
inline bool DoPartialMelt(World* w, const ElementTable& table, size_t gas, size_t solid,
                          uint16_t rotation, std::vector<SpawnFallingLiquidInfo>* falling,
                          const uint8_t* visible, bool debug_editing, const Tunables& tun,
                          const DiseaseTable* diseases = nullptr) {
  std::vector<PhaseEntry>& cells = w->Phases();
  const std::vector<uint8_t>& props = w->Properties();
  const Element& s = table.At(cells[solid].element);
  const Element& g = table.At(cells[gas].element);
  if ((g.state & kStateMask) > kStateGas) return false;
  if ((s.state & kStateMask) != kStateSolid) return false;
  if (tun.partial_melt_kg >= cells[solid].mass) return false;
  if (props[solid] & (kUnbreakable | kNotifyOnMelt)) return false;
  const float melt_t = s.highTemp + tun.partial_melt_margin;
  const float t_gas = cells[gas].temperature;
  if (!(t_gas > melt_t)) return false;
  const float t_solid = cells[solid].temperature;
  if (!(t_solid < s.highTemp - tun.partial_melt_margin)) return false;
  const uint16_t liquid = s.highTempTransitionIdx;
  if (liquid == 0xFFFF) return false;
  const Element& l = table.At(liquid);
  const float needed = (melt_t - t_solid) * l.specificHeatCapacity * tun.partial_melt_kg;
  const float gas_c = cells[gas].mass * g.specificHeatCapacity;
  if (!((t_gas - (g.lowTemp + tun.partial_melt_gas_margin)) * gas_c >= needed)) return false;
  if ((l.state & kStateMask) != kStateLiquid) return false;
  const float cooled = (gas_c * t_gas - needed) / gas_c;
  const int32_t germs = static_cast<int32_t>((tun.partial_melt_kg / cells[solid].mass) *
                                             static_cast<float>(w->Disease()[solid].count));

  const double before = GridCellEnergy(*w, table, gas) + GridCellEnergy(*w, table, solid);
  if (SpawnFallingLiquid(w, table, falling, visible, debug_editing, gas, liquid,
                         tun.partial_melt_kg, melt_t, w->DiseaseIdx(solid), germs, tun)) {
    cells[gas].temperature = cooled;
    cells[solid].mass -= tun.partial_melt_kg;
    w->MutableDisease(solid).count -= germs;
    // What left the grid, priced the way the game will hold it: 5 kg of the liquid.
    const double particle = static_cast<double>(tun.partial_melt_kg) *
                            static_cast<double>(l.specificHeatCapacity) * melt_t;
    w->NoteUnstable(tun.partial_melt_kg);
    w->NoteUnstableEnergy(-particle);
    w->NotePhaseChange(GridCellEnergy(*w, table, gas) + GridCellEnergy(*w, table, solid) +
                       particle - before);
    if (cells[solid].mass <= FLT_MIN) {
      Evaporate(w, table, solid);
      w->MutableRoomsDirty() = true;
    }
    return true;
  }
  // `DisplaceGas` charges its own move to `displaced`, so the phase change is priced from
  // after it.
  if (!DisplaceGas(w, table, gas, rotation, nullptr, false, nullptr, diseases)) return false;
  const double moved = GridCellEnergy(*w, table, gas) + GridCellEnergy(*w, table, solid);
  cells[gas].mass = tun.partial_melt_kg;
  cells[gas].temperature = melt_t;
  cells[gas].element = liquid;
  cells[solid].mass -= tun.partial_melt_kg;
  w->NotePhaseChange(GridCellEnergy(*w, table, gas) + GridCellEnergy(*w, table, solid) - moved);
  if (cells[solid].mass <= FLT_MIN) {
    Evaporate(w, table, solid);
    w->MutableRoomsDirty() = true;
  }
  w->TouchSubstance(gas);
  return true;
}

// `DoSublimation`, driven by the free cell. Bails without drawing unless the
// cell holds less than 1.8 kg; then walks up, right, left, down and draws once for every
// neighbour that is solid and has a real element, acting when the draw is at or under that
// element's `sublimateProbability`.
// The neighbours are read straight out of the padded grid with no region test and no edge
// test of any kind: Klei's only bound is the array's own length. That matters more than it
// looks — the border ring is Neutronium, so it is *solid*, so every gas cell along the edge
// of the world draws four times here where an implementation that skipped the ring would
// draw fewer, and one skipped draw moves the whole rest of the world onto the wrong number.
// This was the first defect this kernel had, and it showed as our sim making no swaps at
// all rather than as slightly wrong ones.
inline bool DoSublimation(World* w, const ElementTable& table, size_t cell, int32_t pw,
                          uint16_t rotation, bool caller_free, const Tunables& tun,
                          const DiseaseTable* diseases = nullptr) {
  std::vector<PhaseEntry>& cells = w->Phases();
  if (cells[cell].mass >= tun.sublimation_cell_cap_kg) return false;

  // Both of these are captured **once, before the loop**, and neither is re-read as the
  // loop changes the cell: the cell's element, and the free
  // cell's `ElementPostProcessData` which is the caller's fourth argument. So the second
  // solid neighbour to sublimate into a vacuum cell is still told the cell is vacuum, even
  // though the first one has already filled it — and it therefore takes the write path
  // below rather than the merge path, overwriting the temperature the first one set.
  const uint16_t cell_element0 = cells[cell].element;
  // `caller_free` is **not** derived from that element. Klei reads the element live,
  // but the phase it branches on is the caller's fourth
  // argument — the `ElementPostProcessData` `PostProcessCell` captured at the top of the
  // cell's turn, before `EvaporateWisp`, `DoDensityDisplacement` and the shuffle ran. So a
  // gas cell that one of those emptied is still treated as gas here, takes the displacement
  // branch, and `DisplaceGas` then refuses it for having no mass — which means no product
  // is written at all. Deriving the flag from the live element instead writes the product
  // into the emptied cell and puts `subldisp` a kilogram out by tick 22.

  bool did = false;
  const ptrdiff_t offsets[4] = {pw, 1, -1, -static_cast<ptrdiff_t>(pw)};
  for (int k = 0; k < 4; ++k) {
    const size_t n = static_cast<size_t>(static_cast<ptrdiff_t>(cell) + offsets[k]);
    const Element& src = table.At(cells[n].element);
    if ((src.state & kStateMask) != kStateSolid) continue;
    // **This test is before the draw, and that is the whole difference between a world
    // that tracks Klei and one that does not.** A solid with no sublimate target consumes
    // nothing from the stream, so a room walled in granite draws exactly as often as the
    // same room in a vacuum. Putting the draw first cost every cell after the first wall
    // its place in the sequence.
    if (src.sublimateIndex == kNoElement ||
        static_cast<int32_t>(src.sublimateIndex) >= table.Count()) {
      continue;
    }
    if (NextRandom(w, tun.random_scale) > src.sublimateProbability) continue;
    // Past the draw the transfer is the one that was measured from the solid's side:
    // `sublimateRate * 0.2` leaves it per free neighbour, `sublimateEfficiency` of that
    // arrives, and the remainder is destroyed.
    PhaseEntry& s = cells[n];
    const float portion = src.sublimateRate * tun.substep_seconds;
    const float held = s.mass;
    float taken = portion < held ? portion : held;

    // **The solid is consumed where it stands when one more portion would not fit in what
    // is left of it** — `mass - taken >= portion`, and the whole block
    // when it is not. The tile becomes the sublimate at `efficiency` of its own
    // mass, keeping its temperature, and nothing is handed to the free cell at all: the
    // announce is on the *solid*, and the loop skips the tail entirely by
    // jumping to. This is how a seam of oxylite ends rather than thinning
    // forever, and it was the largest single thing missing from this kernel.
    if (!(held - taken >= portion)) {
      const double stands_before = GridCellEnergy(*w, table, n);
      s.element = src.sublimateIndex;
      s.mass = held * src.sublimateEfficiency;
      w->NoteSublimated(held - s.mass);
      // Both the element and the mass change and the temperature is kept, so the cell's own
      // before/after is the only honest measure of what that came to.
      w->NoteSublimatedEnergy(GridCellEnergy(*w, table, n) - stands_before);
      w->TouchSubstance(n);
      did = true;
      continue;
    }

    // The disease that rides along is the fraction of the solid's count that the transfer
    // takes, truncated — and recomputed if the merge path
    // below clamps the amount.
    const float count_n = static_cast<float>(w->Disease()[n].count);
    int32_t disease_share = static_cast<int32_t>((taken / held) * count_n);

    // What the free cell actually gains, measured at each of the three ways this ends. It
    // cannot be measured across the whole block: the `DisplaceGas` below empties that cell
    // into a neighbour first, and that is a transfer, not a loss.
    float gained = 0.0f;

    if (cell_element0 == src.sublimateIndex) {
      // The free cell already holds this gas, so the transfer is an ordinary add and the
      // 1.8 kg gate is applied a second time, to what *arrives*: scales the
      // amount down so the cell lands exactly on 1.8 kg rather than past it.
      float arriving = taken * src.sublimateEfficiency;
      const float room = tun.sublimation_cell_cap_kg - cells[cell].mass;
      if (arriving > room) {
        taken = (room / arriving) * taken;
        arriving = room;
        disease_share = static_cast<int32_t>((taken / held) * count_n);
      }
      // This path delivers through the same `AddMassAndUpdateTemperature` as
      // everything else, clamp included, and with it the solid's germ share, of the solid's
      // live germ type, merged by `Disease::AddDiseaseToCell`.
      const float dst_before = cells[cell].mass;
      const double dst_energy_before = GridCellEnergy(*w, table, cell);
      AddMassAndUpdateTemperature(&cells[cell], arriving, s.temperature);
      if (diseases != nullptr && cells[cell].mass > 0.0f) {
        AddDiseaseToCell(w, *diseases, cell, w->DiseaseIdx(n), disease_share);
      }
      gained = cells[cell].mass - dst_before;
      w->NoteSublimatedEnergy(GridCellEnergy(*w, table, cell) - dst_energy_before);
      did = true;
    } else {
      // The free cell holds something else. If it was **vacuum** the product is simply
      // written over it. If it held a gas, that gas is shoved aside first, and **if it cannot be
      // shoved the product is not written at all** —.
      const bool room_made =
          caller_free ||
          DisplaceGas(w, table, cell, rotation, nullptr, false, nullptr, diseases);
      // What happens on failure is the one genuinely odd branch in the kernel,:
      // if nothing has sublimated yet this call the neighbour is skipped
      // outright, but if something *has*, the tail below still runs — so the solid loses
      // its mass and no cell gains it. This is the game's behaviour, kept on purpose.
      if (!room_made && !did) continue;
      if (room_made) {
        // Note the mass is a plain `+=` and the temperature is a plain copy
        // from the solid: no mixing, unlike every other delivery in the sim.
        const double dst_energy_before = GridCellEnergy(*w, table, cell);
        cells[cell].element = src.sublimateIndex;
        const float dst_before = cells[cell].mass;
        cells[cell].mass += taken * src.sublimateEfficiency;
        gained = cells[cell].mass - dst_before;
        cells[cell].temperature = s.temperature;
        w->NoteSublimatedEnergy(GridCellEnergy(*w, table, cell) - dst_energy_before);
        // The germs are written, not merged: the solid's live type and its share. The hash
        // is the save's copy of the type and follows it.
        w->MutableDiseaseIdx(cell) = w->DiseaseIdx(n);
        w->MutableDisease(cell).count = disease_share;
        if (diseases != nullptr) {
          w->MutableDisease(cell).diseaseHash =
              w->DiseaseIdx(n) == 0xFF ? 0 : diseases->HashOf(w->DiseaseIdx(n));
        }
        // On the free cell that took the sublimate.
        w->TouchSubstance(cell);
        did = true;
      }
    }

    // The shared tail, reached by both paths above. The solid pays whatever
    // the transfer decided, and is deleted outright once there is nothing left of it.
    const double solid_before = GridCellEnergy(*w, table, n);
    s.mass -= taken;
    w->NoteSublimatedEnergy(GridCellEnergy(*w, table, n) - solid_before);
    // Whatever left the solid and did not arrive. On the failure branch above that is the
    // whole of `taken` — the game's own behaviour, kept on purpose, as the comment there
    // says — and it is exactly the kind of thing the ledger exists to keep visible.
    w->NoteSublimated(taken - gained);
    // The shared tail: the solid pays its germ share, and under one germ `ClearDisease` resets all
    // four fields, the germ type included (this used to zero the count and hash and
    // leave the type behind).
    ModifyCellDiseaseCount(w, n, -disease_share);
    // The game also accumulates the removed mass per cell here. Nothing this sim
    // publishes reads it.
    // Under 1e-9 kg the solid is `Evaporate`d, which announces it a second
    // time in the same call.
    if (1e-9f >= s.mass) Evaporate(w, table, n);
  }
  return did;
}

// ------------------------------------------------------------------- liquid off-gassing
//
// `ElementPostProcessData` is a compact per-element copy of the element table, built once in
// `CreateElementsTable`. Which field holds what matters, because the off-gas path and
// `DoSublimation` read the same fields while meaning different things by them:
//
//   sublimateIndex     convertIndex          state
//   sublimateRate      sublimateEfficiency   sublimateProbability
//   offGasProbability                        molarMass
//   strength           maxMass               minHorizontalFlow
//   sublimateFX
//
// So a liquid off-gasses into its own `sublimateIndex`, gates on `sublimateProbability`,
// loses `offGasProbability * mass` per substep (despite the name — it is a rate, not a
// probability, and it is capped at 1 kg) and delivers `sublimateEfficiency` of that.
//
// The transfer itself is, shared by both exits of the block. Source loses the
// full amount, destination gains `amount * efficiency` — the same deliberate mass loss
// `DoSublimation` has — and the destination's temperature becomes the mass-weighted mean
// **clamped to the two input temperatures**, which the plain quotient is not when the two
// masses differ by enough to lose the low bits.
//
// Germs, in Klei's order. When `carry` is set, `from` pays
// `(int)((float)count * (amount / mass))` of its live count through `ModifyDiseaseCount`, and
// if `to` then holds a different germ type it is cleared and takes `from`'s (after the
// debit, so a source emptied of germs hands on 0xFF). A `to` that changes element is cleared
// again. Last, `to` gains the share through `ModifyDiseaseCount`, which clears a cell left
// under one germ even when nothing was carried. On the convert-in-place path `from` and `to`
// are one cell, so the count it pays is the count it gets back, under whatever type the two
// clears left: Klei's own arithmetic, reproduced rather than tidied.
inline void OffGasTransfer(World* w, const ElementTable& table, const Element& src,
                           size_t from, size_t to, float amount, bool carry,
                           const DiseaseTable* diseases = nullptr) {
  std::vector<PhaseEntry>& cells = w->Phases();
  // Energy ledger. Two cells, or one on the convert-in-place path -- counting `to` twice
  // when it IS `from` would double every number this charges.
  const bool same_cell = from == to;
  const double pair_before = GridCellEnergy(*w, table, from) +
                             (same_cell ? 0.0 : GridCellEnergy(*w, table, to));
  double cleared_here = 0.0;
  // `from` and `to` are the same cell on the convert-in-place path, so the source debit has
  // to be read back out of the cell rather than assumed to be `amount`.
  const float from_before = cells[from].mass;
  // Layer C: a liquid that off-gasses part of itself loses that share of its payload; when it
  // converts in place the cell stops being that liquid, and all of it goes.
  if (from_before > 0.0f) {
    w->PayloadRelease(from, same_cell ? 1.0f : amount / from_before, cells[from].temperature,
                      ext::kPayloadReleasedOffGas);
  }
  cells[from].mass -= amount;
  const float debited = from_before - cells[from].mass;
  int32_t germs = 0;
  if (carry) {
    germs = static_cast<int32_t>(static_cast<float>(w->Disease()[from].count) *
                                 (amount / from_before));
    ModifyCellDiseaseCount(w, from, -germs);
    if (w->DiseaseIdx(to) != w->DiseaseIdx(from)) {
      ClearCellDisease(w, to);
      w->MutableDiseaseIdx(to) = w->DiseaseIdx(from);
      if (diseases != nullptr) {
        w->MutableDisease(to).diseaseHash =
            w->DiseaseIdx(from) == 0xFF ? 0 : diseases->HashOf(w->DiseaseIdx(from));
      }
    }
  }
  if (cells[to].element != src.sublimateIndex) {
    // The destination held something else and it is overwritten, not displaced. The caller
    // shoves a gas aside first where it can; anything still here is destroyed.
    w->PayloadRelease(to, 1.0f, cells[to].temperature, ext::kPayloadReleasedCleared);
    w->NoteCleared(cells[to].mass);
    // Charged here rather than in `ClearCell`, because this path does not call it -- it
    // open-codes the same three writes. Kept in the same bucket so "what was destroyed
    // outright" stays one number wherever it happened.
    cleared_here = -GridCellEnergy(*w, table, to);
    w->NoteClearedEnergy(cleared_here);
    cells[to].element = src.sublimateIndex;
    ClearCellDisease(w, to);
    cells[to].mass = 0.0f;
  }
  const float arriving = amount * src.sublimateEfficiency;
  const float t_src = cells[from].temperature;
  const float t_dst = cells[to].temperature;
  const float m_dst = cells[to].mass;
  const float total = m_dst + arriving;
  float t = 0.0f;
  if (total > 0.0f) {
    t = (t_dst * m_dst + t_src * arriving) / total;
    const float lo = t_src < t_dst ? t_src : t_dst;
    const float hi = t_src < t_dst ? t_dst : t_src;
    if (t < lo) t = lo;
    if (t > hi) t = hi;
  }
  cells[to].temperature = t;
  cells[to].mass = m_dst + arriving;
  ModifyCellDiseaseCount(w, to, germs);
  // The efficiency loss. `debited` has to be read before the overwrite above, because on
  // the in-place path `from` and `to` are the same cell and that overwrite has already
  // charged the residual to `cleared`.
  w->NoteSublimated(debited - arriving);
  // The efficiency loss in energy, plus the temperature copy: the pair's real change, less
  // whatever the overwrite above already charged to `cleared`.
  w->NoteSublimatedEnergy(GridCellEnergy(*w, table, from) +
                          (same_cell ? 0.0 : GridCellEnergy(*w, table, to)) - pair_before -
                          cleared_here);
  // On the cell the gas arrives in. Like the two in `DoSublimation` this
  // placement is unverified by a scenario: only five elements in the table carry
  // a `sublimateIndex` and water is not one of them, so no suite scenario off-gasses.
  w->TouchSubstance(to);
}

// The block. **Only one neighbour is tested — the cell above** — which was
// the open question. It does not loop.
//
// The gate order is what the random stream cares about, and it is not the order the physics
// suggests: mass of the cell above under 1.8 kg, then the liquid's own `sublimateIndex`
// against 0xFFFF, then the cell above being gas or vacuum, and only *then* the draw. The
// permeability test on the cell above sits **after** the draw and before the comparison, so
// a liquid under a sealed tile still consumes its number.
//
// The upshot for the stream is quieter than it looks: five elements in the table carry a
// `sublimateIndex` and water is not one of them, so an ordinary pool draws nothing here.
// What a pool does draw is `DoDensityDisplacement`, once per cell, at 0.30.
inline void DoLiquidOffGas(World* w, const ElementTable& table, size_t cell, int32_t pw,
                           uint16_t rotation, const Tunables& tun,
                           const DiseaseTable* diseases = nullptr) {
  std::vector<PhaseEntry>& cells = w->Phases();
  const std::vector<uint8_t>& props = w->Properties();
  const size_t above = cell + static_cast<size_t>(pw);
  const size_t below = cell - static_cast<size_t>(pw);

  if (!(tun.off_gas_cell_cap_kg > cells[above].mass)) return;
  const Element& src = table.At(cells[cell].element);
  if (src.sublimateIndex == kNoElement ||
      static_cast<int32_t>(src.sublimateIndex) >= table.Count()) {
    return;
  }
  if ((table.At(cells[above].element).state & kStateMask) > kStateGas) return;

  const float roll = NextRandom(w, tun.random_scale);
  if (props[above] & kGasImpermeable) return;
  if (!(roll < src.sublimateProbability)) return;

  // The amount comes off this cell unless the cell below holds *more* of the same element,
  // in which case the column's bottom pays instead and the flag is remembered — the whole
  // point of it is the leftover handling further down.
  float amount = src.offGasProbability * cells[cell].mass;
  if (amount > tun.off_gas_amount_cap_kg) amount = tun.off_gas_amount_cap_kg;
  bool from_below = false;
  if (cells[below].element == cells[cell].element && cells[below].mass > cells[cell].mass) {
    from_below = true;
    amount = src.offGasProbability * cells[below].mass;
    if (amount > tun.off_gas_amount_cap_kg) amount = tun.off_gas_amount_cap_kg;
  }

  if (cells[cell].mass - amount >= tun.off_gas_residual_kg) {
    if (!(amount > 0.0f)) return;
    const uint16_t up = cells[above].element;
    if (up != src.sublimateIndex && up != table.VacuumIndex()) {
      // Klei shoves whatever is up there aside with `DisplaceGas` and carries
      // on when that works; when it does not, the whole block is abandoned
      // and the liquid keeps its mass. The draw is already spent either way, so the stream
      // stays aligned whichever branch is taken.
      if (!DisplaceGas(w, table, above, rotation, nullptr, false, nullptr, diseases)) return;
    }
    // Room is measured against the same 1.8 kg the gate used, and against the *delivered*
    // mass rather than the amount taken.
    const float delivered = amount * src.sublimateEfficiency;
    const float room = tun.off_gas_cell_cap_kg - cells[above].mass;
    if (delivered > room) amount *= room / delivered;
    // Germs ride along only when the cell above holds none or the same type, read
    // after `DisplaceGas` has emptied it.
    const bool carry = w->DiseaseIdx(cell) == w->DiseaseIdx(above) ||
                       w->DiseaseIdx(above) == 0xFF;
    OffGasTransfer(w, table, src, cell, above, amount, carry, diseases);
    return;
  }

  // Too little would be left to stand as a liquid, so the cell converts where it is: the
  // whole of it becomes gas in place, and any shortfall is then drawn out of the cell below.
  const float held = cells[cell].mass;
  OffGasTransfer(w, table, src, cell, cell, held, true, diseases);
  const float excess = amount - held;
  if (excess > 0.0f && from_below) {
    // The same carry rule, against the cell as the in-place conversion left it.
    const bool carry = w->DiseaseIdx(below) == w->DiseaseIdx(cell) ||
                       w->DiseaseIdx(cell) == 0xFF;
    OffGasTransfer(w, table, src, below, cell, excess, carry, diseases);
  }
}

// ---------------------------------------------------------------- unstable solids
//
// Sand, regolith and the rest of the "falls when nothing holds it up" list. This is the
// last kernel in the sim that draws from the random stream, and the only reason it matters
// enough to model is that draw: everything the fall itself does is deterministic, but a
// world with one grain of sand in it that skips a draw puts every gas cell after it onto a
// different number.
//
// `PostProcessCell`'s solid branch is two tests and then nothing: `state & 0xb == 0xb`
// — phase 3 *and* bit 3, Unstable — and then a saved-options bit picking
// `DoUnstableCheckWithDiagonals` over `DoUnstableCheckBasic`.
// That bit is `SimSavedOptions.ENABLE_DIAGONAL_FALLING_SAND`, set by
// `SimMessages.SetSavedOptionValue`, and **nothing in `Assembly-CSharp` calls that method**
// — the whole diagonal kernel is shipped dead. The byte starts at zero and only
// `SetSavedOptions` can change it, so basic is what the game runs and what is modelled here.
//
// `DoUnstableCheckBasic` reads exactly one neighbour, the cell below, and bails on three
// things without ever consulting the counter — each time **resetting** it to the sentinel,
// so a tile that has been propped up for an hour still waits its full roll after the prop
// is dug out:
//
//   the cell below is a solid
//   the cell below is Void, hash 0xa9360b34
//   the cell below is SolidImpermeable, property bit 2
//
// It is worth being precise about what is *not* in that list. There is no test that the
// falling cell is unsupported from the sides, no test on the cell's own mass, and no test
// on what is below the cell below. A single sand tile with gas under it falls, and it falls
// through liquid as readily as through vacuum.
inline void UnstableFallSwap(World* w, const ElementTable& table, size_t cell, int32_t pw) {
  std::vector<PhaseEntry>& cells = w->Phases();
  // `HeadlessUnstableFallSimpleSwap`. Not one swap — a loop that walks the
  // cell all the way down until a solid stops it, in a single call, inside one substep.
  // There is no draw anywhere in it and no re-check of the guards above: only the phase of
  // the next cell down matters once the fall has started.
  size_t cur = cell;
  while (cur >= static_cast<size_t>(pw)) {
    const size_t below = cur - static_cast<size_t>(pw);
    if (table.Phase(cells[below].element) == kStateSolid) {
      // On the cell that did *not* move. Klei announces the resting place as
      // well as every cell the fall passed through.
      w->TouchSubstance(cur);
      return;
    }
    // `CellSOA` swaps seven arrays here and the cell properties are not one of them, so a
    // tile falling past a wall keeps the wall's permeability where it was.
    SwapCells(w, below, cur);
    // The inlined `ChangeSubstance` — same game-index
    // arithmetic, same push, same `|= 0x1f`. It names the cell the material just left, so
    // across iterations both ends of every step get announced.
    w->TouchSubstance(cur);
    cur = below;
  }
}

inline void UnstableCheckBasic(World* w, const ElementTable& table, size_t cell, int32_t pw,
                               std::vector<UnstableCellInfo>* unstable, const Tunables& tun) {
  std::vector<PhaseEntry>& cells = w->Phases();
  const std::vector<uint8_t>& props = w->Properties();
  const size_t below = cell - static_cast<size_t>(pw);
  const uint16_t below_element = cells[below].element;

  if (table.Phase(below_element) == kStateSolid || below_element == table.VoidIndex() ||
      (props[below] & kSolidImpermeable) != 0) {
    // Held up, so the countdown goes back to the sentinel — and note this is
    // the *only* exit that writes it. A cell that is merely still counting leaves it alone.
    w->MarkUnstableDirty(cell);
    return;
  }

  // The draw, and the only one in this kernel. It happens on every visit to an unsupported
  // unstable solid, not only on the visit that moves it.
  if (w->StableTicksRemaining(cell, tun.stable_ticks_reroll_scale,
                              tun.stable_ticks_reroll_base) != 0) {
    return;
  }

  if (w->Headless()) {
    UnstableFallSwap(w, table, cell, pw);
    return;
  }

  // The game's half. The cell is handed over as an `unstableCellInfo` and then emptied, and
  // the grid does not move it at all — whatever falls is an entity from here on. `Evaporate`
  // is `SimData::ClearCell` plus one `ChangeSubstance`, and `ClearCell` writes
  // the vacuum element, zero mass, zero temperature and a disease index of 0xff.
  //
  // Klei reads the five cell fields — disease count, disease
  // index, temperature, mass, element — **before** `Evaporate` clears them, and writes
  // `fallingInfo` as a literal 0. Reading them after the clear would leave
  // the cell index right and every number zero, which is why the harness compares the whole
  // struct and not just the count. Run `diffsim --scenario sand --gameside --messages`.
  if (unstable) {
    UnstableCellInfo info{};
    info.cellIdx = static_cast<int32_t>(w->GameIndex(cell));
    info.elemIdx = cells[cell].element;
    info.fallingInfo = 0;
    info.diseaseIdx = w->DiseaseIdx(cell);
    info.mass = cells[cell].mass;
    info.temperature = cells[cell].temperature;
    info.diseaseCount = w->Disease()[cell].count;
    unstable->push_back(info);
  }
  // Charged with `unstable` null as well. Without it the solid falls through the grid
  // rather than becoming an entity, but either way the cells stop holding it.
  w->NoteUnstable(cells[cell].mass);
  w->NoteUnstableEnergy(-GridCellEnergy(*w, table, cell));
  // Layer C: the cell's contents leave the grid as a falling particle, handed to the game.
  w->PayloadRelease(cell, 1.0f, cells[cell].temperature, ext::kPayloadReleasedFalling);
  cells[cell].element = table.VacuumIndex();
  cells[cell].mass = 0.0f;
  cells[cell].temperature = 0.0f;
  w->MutableDiseaseIdx(cell) = 0xFF;
  w->MutableDisease(cell) = SaveDisease{};
  w->TouchSubstance(cell);
}

// `DoPressureBreak`: over-full liquid pushing on a thin wall. `PostProcessCell`
// calls it only when a liquid cell holds more than its element's `maxMass`, with
// `p = mass / maxMass`, for the cell below, left, right and above in that order; the first
// that returns true ends the cell's turn, which also skips the off-gas draw after it. Every
// read is of the LIVE grid, and nothing here draws.
//
// It walks up to three cells from `cell` in direction `step`:
//
//   * it stops at the first cell that is not solid;
//   * a solid whose element is `kStateUnbreakable` (state bit 4) or whose cell has
//     `kUnbreakable` (property bit 8) refuses the whole break;
//   * each solid adds to a resistance that starts at 1:
//         ((m / maxMass) * hi + (1 - hi)) * strength * lo * 0.25  (+ 0.1 when hi == 0)
//     where `hi` is bit 7 of the cell's strength byte (`SetStrength`'s weight) and `lo` its
//     low seven bits, and `strength`/`maxMass` are the solid element's. The floats are grouped
//     as Klei's: the term, then the 0.1, then the running sum. The
//     0.1 is the float 0x3dccccd0, one ulp off 0.1f;
//   * a resistance above `p` refuses the break there and then.
//
// Only a wall one or two cells thick breaks (three never does), and only while
// `resistance < p`. If the cell past the wall holds liquid, that liquid's `m / maxMass` comes
// off `p` first -- a wall with water on both sides holds -- and the test is taken again.
// A break pushes one `{gameCell(wall), gameCell(source)}` record per wall cell, nearest first,
// skipping any pair outside the game's cells, and returns true whether or not anything was
// pushed. The game applies the damage (`WorldDamage.ApplyDamage`, 1/1200 per record), leaks
// 1 kg through, damages the building in the tile and destroys the tile at full damage. A null
// `damage` still breaks and still ends the cell's turn, which is what `bench` passes.
inline bool DoPressureBreak(World* w, const ElementTable& table, size_t cell, float p,
                            ptrdiff_t step, std::vector<WorldDamageInfo>* damage,
                            const Tunables& tun) {
  const std::vector<PhaseEntry>& cells = w->Phases();
  const std::vector<uint8_t>& props = w->Properties();
  const std::vector<uint8_t>& strength = w->Strength();
  // The tunables `PressureBreakUnbuiltFloor` (default bits 0x3dccccd0),
  // `PressureBreakBaseResistance` (1) and `PressureBreakStrengthScale` (0.25).
  const float unbuilt_floor = tun.pressure_break_unbuilt_floor;

  float resistance = tun.pressure_break_base_resistance;
  int walls = 0;
  size_t at = cell;
  for (int k = 0; k < 3; ++k) {
    at = static_cast<size_t>(static_cast<ptrdiff_t>(at) + step);
    const Element& e = table.At(cells[at].element);
    if ((e.state & kStateMask) != kStateSolid) break;
    if (e.state & kStateUnbreakable) return false;
    if (props[at] & kUnbreakable) return false;
    ++walls;
    const uint8_t s = strength[at];
    const float hi = static_cast<float>(s >> 7);
    const float lo = static_cast<float>(s & 0x7f);
    const float floor = hi >= 1.0f ? 0.0f : unbuilt_floor;
    float term = cells[at].mass / e.maxMass;
    term *= hi;
    term += 1.0f - hi;
    term *= e.strength;
    term *= lo;
    term *= tun.pressure_break_strength_scale;
    term += floor;
    resistance += term;
    if (p < resistance) return false;
  }
  if (walls != 1 && walls != 2) return false;
  if (!(resistance < p)) return false;
  // `at` is the cell the walk stopped on: past the wall when it was one or two thick.
  const Element& past = table.At(cells[at].element);
  if ((past.state & kStateMask) == kStateLiquid && cells[at].mass > 0.0f) {
    p -= cells[at].mass / past.maxMass;
  }
  if (!(resistance < p)) return false;
  if (damage != nullptr) {
    const int64_t source = w->GameIndex(cell);
    const int64_t n = static_cast<int64_t>(w->GameCount());
    size_t wall = cell;
    for (int k = 0; k < walls; ++k) {
      wall = static_cast<size_t>(static_cast<ptrdiff_t>(wall) + step);
      const int64_t game = w->GameIndex(wall);
      if (game < 0 || game >= n || source < 0 || source >= n) continue;
      WorldDamageInfo info{};
      info.gameCell = static_cast<int32_t>(game);
      info.damageSourceOffset = static_cast<int32_t>(source);
      damage->push_back(info);
    }
  }
  return true;
}

// The over-full `DisplaceLiquid` that follows the four breaks lives in `cellmod.h`
// (`DoOverfullDisplace`), because it needs `DisplaceLiquid` and the disease table, and
// `cellmod.h` includes this file rather than the other way round. A caller that has both
// hands it in; one that does not (`bench`) leaves it null and the step does not run.
struct CellModContext;
struct OverfullDisplace {
  const CellModContext* cm = nullptr;
  bool (*fn)(const CellModContext& cm, size_t cell) = nullptr;
};

// `falling`, `visible` and `debug_editing` are `DoPartialMelt`'s, passed on to
// `SpawnFallingLiquid` exactly as `StepFlow` passes them.
inline void StepPostProcess(World* w, const ElementTable& table, uint16_t rotation,
                            std::vector<UnstableCellInfo>* unstable, size_t ri,
                            std::vector<SpawnFallingLiquidInfo>* falling = nullptr,
                            const uint8_t* visible = nullptr, bool debug_editing = false,
                            std::vector<WorldDamageInfo>* damage = nullptr,
                            OverfullDisplace overfull = {},
                            const DiseaseTable* diseases = nullptr) {
  ONI_SWEEP(kPostProcess);
  const World::PaddedRect& r = w->PaddedRegionInclusive(ri);
  const int32_t pw = w->PaddedWidth();
  std::vector<PhaseEntry>& cells = w->Phases();
  const std::vector<uint8_t>& props = w->Properties();
  const Tunables tun = g_tunables;  // tunables.h, THE ONE RULE

  // The rectangle, read **inclusively** — the same set the `ActiveInclusive` test this
  // replaces accepted, in the same order. See `World::PaddedRegions`.
  {
    for (int32_t y = r.y0; y < r.y1; ++y) {
      ONI_EXAMINED(kPostProcess, r.x1 - r.x0);
      for (int32_t x = r.x0; x < r.x1; ++x) {
        const size_t i = static_cast<size_t>(y) * pw + x;
        // Inclusive, like the far end of a gas-pressure pair and unlike everything else —
        // the world's top row is post-processed. Measured on `toprow`, where a 0.0009 kg
        // wisp on row `height - 1` next to an 8.8 kg cell is evaporated by Klei and was
        // left standing here; with the half-open mask the scenario ends 0.000894 kg heavier
        // than Klei's and with it the totals agree to the last bit. `toprow`'s draw counts
        // agree tick for tick too, so the shuffle runs up there as well and draws the same
        // number of times — it is the whole sweep that is inclusive, not one branch of it.
        // That is why this sweep takes its rectangle from `PaddedRegionInclusive` and every
        // other one takes it from `PaddedRegion`.
        const uint8_t state = table.At(cells[i].element).state;
        switch (state & kStateMask) {
          case kStateVacuum:
            if (!(props[i] & kGasImpermeable)) {
              DoSublimation(w, table, i, pw, rotation, true, tun, diseases);
            }
            break;

          case kStateGas: {
            // The stride flips on **every** gas cell reached, before any mass gate, and it is
            // the only thing that ever writes the stride.
            const int32_t dir = -w->ShuffleDir();
            w->SetShuffleDir(dir);
            // Klei captures the source's `ElementPostProcessData` here, once, and every step
            // below is handed that captured pointer rather than re-reading the grid. It
            // matters because the very next line can clear the cell to vacuum.
            const uint16_t src_element = cells[i].element;
            // A wisp or an empty cell is deleted outright. Neither draws and neither exits —
            // both evaporation sites fall straight into `DoDensityDisplacement`.
            EvaporateWisp(w, table, i, dir, pw, tun);
            const size_t below = i - static_cast<size_t>(pw);
            const bool consumed = DoDensityDisplacement(w, table, i, below,
                                                        tun.gas_displacement_probability,
                                                        src_element, tun);
            // Then the four `DoPartialMelt` calls, on the cell below, left, right and above,
            // stopping at the first that melts. None of them draws, but one that melts skips
            // the shuffle and with it the shuffle's draws. A liquid-impermeable gas cell
            // skips the four calls and goes straight to the shuffle.
            if (!consumed) {
              const bool melted =
                  !(props[i] & kLiquidImpermeable) &&
                  (DoPartialMelt(w, table, i, below, rotation, falling, visible, debug_editing,
                                 tun, diseases) ||
                   DoPartialMelt(w, table, i, i - 1, rotation, falling, visible, debug_editing,
                                 tun, diseases) ||
                   DoPartialMelt(w, table, i, i + 1, rotation, falling, visible, debug_editing,
                                 tun, diseases) ||
                   DoPartialMelt(w, table, i, i + static_cast<size_t>(pw), rotation, falling,
                                 visible, debug_editing, tun, diseases));
              if (!melted) GasShuffle(w, table, i, dir, pw, src_element, tun);
            }
            DoSublimation(w, table, i, pw, rotation, false, tun, diseases);
            break;
          }

          case kStateLiquid: {
            // Under 0.01 kg the cell is not a liquid any more: `Evaporate` clears it to the
            // vacuum element at zero mass and zero temperature and nothing else runs. No draw.
            if (cells[i].mass < tun.thin_liquid_kg) {
              w->NoteThinLiquid(cells[i].mass);
              w->NoteThinLiquidEnergy(-GridCellEnergy(*w, table, i));
              cells[i].element = table.VacuumIndex();
              cells[i].mass = 0.0f;
              cells[i].temperature = 0.0f;
              // The liquid branch's `Evaporate`. This is where a drained
              // pool cell's `172 -> 211` comes from.
              w->TouchSubstance(i);
              break;
            }
            // Same kernel as the gas branch, at 0.30 instead of 0.99, and reached with no mass
            // gate in front of it. This is where a pool's draws actually go.
            const size_t below = i - static_cast<size_t>(pw);
            if (DoDensityDisplacement(w, table, i, below, tun.liquid_displacement_probability,
                                      cells[i].element, tun)) {
              break;
            }
            // Over `maxMass`: the four `DoPressureBreak` calls, below, left, right and above,
            // then above 1.5 * maxMass the over-full `DisplaceLiquid`. Four
            // `DoPartialHeatTransition` calls on the same four cells follow in the game and are
            // not modelled. None of them draws a random number, so leaving them out keeps the
            // random sequence in step -- but each can end the cell's
            // turn, which skips the off-gas draw below.
            {
              const Element& le = table.At(cells[i].element);
              if (cells[i].mass > le.maxMass) {
                const float p = cells[i].mass / le.maxMass;
                const ptrdiff_t spw = static_cast<ptrdiff_t>(pw);
                if (DoPressureBreak(w, table, i, p, -spw, damage, tun) ||
                    DoPressureBreak(w, table, i, p, -1, damage, tun) ||
                    DoPressureBreak(w, table, i, p, 1, damage, tun) ||
                    DoPressureBreak(w, table, i, p, spw, damage, tun)) {
                  break;
                }
                if (overfull.fn != nullptr && overfull.fn(*overfull.cm, i)) break;
              }
            }
            DoLiquidOffGas(w, table, i, pw, rotation, tun, diseases);
            break;
          }

          default:
            // Solid. Everything but an unstable one returns having done nothing and drawn
            // nothing — the branch still has to be taken, but there is nothing in it.
            if ((state & kStateUnstable) != 0) {
              UnstableCheckBasic(w, table, i, pw, unstable, tun);
            }
            break;
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------- flow
//
// Three separate laws, one per phase pairing, all measured on pockets of fluid sealed
// inside solid granite so that nothing could enter or leave the experiment.
//
//   gas + gas, same element:    dm = element.flow * (m_src - m_dst)
//                               no cap, no minimum, and gravity plays no part — the
//                               vertical case is identical to the horizontal one.
//
//   liquid, horizontal:         dm = min(0.25 * (m_src - m_dst), m_src, viscosity)
//                               skipped entirely if dm < minHorizontalFlow
//
//   liquid, down:               dm = min(viscosity, m_src,
//                                        0.5 * max(0, max(m_src * 1.01, maxMass) - m_dst))
//                               skipped if m_src > minVerticalFlow and dm < minVerticalFlow
//
//   liquid, up:                 c  = max(m_dst * 1.01, maxMass)
//                               dm = 0.5 * max(0, m_src - c), and additionally capped by
//                               viscosity and m_src unless m_src >= 2c; skipped at 0.01
//
// The downward law is **one expression, not two branches**. Reading it as "pour into the
// room below, or else apply the pressure gradient once the cell below is full" is right
// almost everywhere and wrong exactly where a settled pool lives — just under maxMass,
// where the pressure arm has already overtaken the room arm. That mistake cost `pool`
// 177 kg; the `max` inside is worth every one of them. The game computes
// `max(mass*1.01, maxMass)` in one step.
//
// The 1.01 is a real, load-bearing constant: a settled column of liquid holds exactly 1%
// more mass in each cell than the one above it. It was recovered from three different
// starting totals that all settled to a below/above ratio of 1.0100 to five figures.
//
// Different elements do not mix at all. Two gases at different pressures in adjacent cells
// stay exactly as they are, forever — this is the one-element-per-cell defect in its
// purest form, and the reason this project exists.
//
// Both are tunables, `LiquidHorizontalRate` (0.25) and `LiquidPressureRatio` (1.01), in
// tunables.def.

// The upward branch is gated on a hard-coded 0.01 rather than on the element's
// minVerticalFlow — the two happen to be equal for water, so only an element with a
// different minVerticalFlow can tell them apart.
// The tunable `LiquidMinUpwardFlow` (tunables.def).

// Which of two elements sinks past the other.
//
// Measured on vertical pairs: a liquid always sinks through vacuum and through gas, no
// matter how little of it there is; two liquids swap iff the upper one has the larger
// **molarMass** — not the larger mass, and not the fuller cell. Water (18.0) under a
// liquid of molar mass 400 swapped in every ratio tried, including 900 kg of the upper
// one against 100 kg of the lower.
//
// Two gases never swap, even with the heavier molar mass on top. So ONI's gas layering
// does not come from this kernel.
//
// **Not wired into `StepFlow` any more**, and kept because the measurement is real and
// will be needed. `UpdateLiquid` handles only the liquid-into-vacuum-or-gas case and
// explicitly refuses to swap two different liquids — it tests the lower cell's phase and
// bails out to the horizontal spread when it is liquid or solid. The molar-mass rule
// therefore lives in `DoDensityDisplacement`, a separate kernel that also draws from the
// RNG and is not modelled yet. There is no scenario for two liquids in `diffsim` to score
// it with, which is exactly why it should not be guessed at from here.
inline bool SinksPast(const ElementTable& t, uint16_t above, uint16_t below) {
  const uint8_t pa = t.Phase(above), pb = t.Phase(below);
  if (pa != kStateLiquid) return false;      // only liquids sink; gases never swap
  if (pb == kStateSolid) return false;
  if (pb == kStateVacuum || pb == kStateGas) return true;
  return t.At(above).molarMass > t.At(below).molarMass;
}

// ------------------------------------------------------------------------ gas pressure
//
// Gas is **its own sweep**, run before the liquid one, and that is the shape of the whole
// substep rather than a detail of this kernel. `SimBase::UpdateData` is:
//
//   CellSOA::CopyFrom          start <- live
//                conduction and state change, dispatched to worker tasks, then joined
//   CellSOA::CopyFrom          start <- live, again
//   UpdatePressure   x3        the gas sweep, three neighbours per cell
//   DoGasPressureDisplacement  a second gas sweep
//   CellSOA::CopyFrom          start <- live
//   UpdateLiquid               the liquid sweep
//   CellSOA::CopyFrom          start <- live
//                Disease::UpdateCells, SimData::UpdateComponents
//   PostProcessCell            the random stream
//
// so `start` is refreshed three times and gas never sees a grid a liquid has moved through
// in the same substep. This kernel is the `UpdatePressure` sweep; `StepFlow` below is now
// the liquid one alone.
//
// **Three neighbours, not four.** A cell pairs with `cell + hdir`, with `cell + width`,
// and with `cell + width + hdir`. `hdir` is a signed 1 that is negated at the top of every
// substep exactly the way the shuffle stride is, and it is both the column order the sweep walks in and the horizontal
// neighbour it picks. Every unordered orthogonal pair is therefore still visited once, and
// so is **one** of the two upward diagonals — which one alternates with the substep. The
// opposite diagonal is never visited at all.
//
// The diagonal is skipped unless there is a way round the corner: the states of the two
// cells between it and the source are OR'd together and the pair is dropped when the
// result is 3, which catches a solid on either side and also a gas/liquid pinch.
//
// Direction is decided by the sign of the transfer, not by which cell the sweep is on, so
// a light cell reached first still gets filled by its heavier neighbour.
//
// The cap is the piece that makes a freshly emptied cell fill from **one** neighbour. A
// destination in the vacuum state has the source's element written into it, and every gate
// here re-reads the live element and compares it against the start-of-sweep one, so the
// instant a vacuum cell takes its first delivery it is closed to every remaining pair this
// substep. Klei's cell holds 0.24 kg from one donor where a gather from all four would
// hold 0.96.
//
// The tunable `GasPressureCap` (tunables.def). The same 0.125 in `DoGasPressureDisplacement`
// and `DoLiquidPressureDisplacement` is two further rows of its own.

// `rooms`: the promotion gate. `nullptr` means "no gate" -- this kernel then runs as
// vanilla, byte-for-byte, since the two new
// `IsRoomOwned` checks below both short-circuit on `rooms == nullptr` before touching
// `room_of`. When a live `RoomGraph` is passed and a cell's room has been promoted
// (`SetRoomOwned`, see `ApplyPromoteRoom`), this vanilla kernel neither drives mass out
// of that cell nor accepts a delivery into it -- the room's own room-pooled mixing
// (`gas_mixture.h`/`gas_rooms.h`) is what owns its mass from here on, not this sweep.
// This is the "vanilla must stop moving mass here" half of promotion. StepConduction,
// StepGasDisplacement and the element emitters carry the same gate; the liquid kernels use
// the narrower `IsMixtureOwnedCell` (gas_rooms.h).
inline void StepGasPressure(World* w, const ElementTable& table, const DiseaseTable& diseases,
                            int32_t hdir, size_t ri, const gas::RoomGraph* rooms = nullptr) {
  ONI_SWEEP(kGasPressure);
  const World::PaddedRect& r = w->PaddedRegion(ri);
  const int32_t pw = w->PaddedWidth();
  const size_t gw = static_cast<size_t>(w->GameWidth());
  std::vector<PhaseEntry>& cells = w->Phases();
  const float pressure_cap = g_tunables.gas_pressure_cap;  // tunables.h, THE ONE RULE
  const std::vector<uint8_t>& props = w->Properties();
  const uint16_t voidelem = table.VoidIndex();

  // One region's share of `CellSOA::CopyFrom`, not the whole grid once per region -- see
  // `SnapshotReadRect` in world.h for the measurement that made this worth changing and for
  // why region 0 still copies everything. `StepGasDisplacement` reads this same snapshot and
  // reaches further than this sweep does, which is what the rectangle's margins are for.
  std::vector<PhaseEntry>& start = GasSweepStart();
  RefreshPhaseSnapshot(start, cells, ri, pw, w->PaddedHeight(), r.x0, r.y0, r.x1, r.y1);
  std::vector<DiseaseEntry>& dstart = GasDiseaseSweepStart();
  if (w->DiseaseActive()) {
    RefreshDiseaseSnapshot(dstart, *w, ri, pw, w->PaddedHeight(), r.x0, r.y0, r.x1, r.y1);
  }

  // Two region tests, not one, and the difference between them is exactly the world's top
  // row. A cell **drives** only if it is in the ordinary half-open region, the same one
  // every other sweep uses; a cell may be the far end of a pair if it is in the region read
  // **inclusively**, one row further up.
  //
  // Both halves are measured. `toprow` puts a ten-kilogram-to-nothing gradient along row
  // `height - 1` and Klei does not level it — so the top row does not drive — while the
  // cells below it pull mass down out of that row on the same tick, and `sunlit` shows Klei
  // announcing top-row cells as the far end of a zero-mass vacuum pair — so the top row is
  // reachable. Using the inclusive mask for both looks identical on every scenario that has
  // no gas up there, which is why it stood until `toprow` existed.
  // Only the far end still needs a bound of its own: the driving loop below walks the
  // half-open rectangle itself. Both are this region's rectangle rather than the mask, which
  // is the same set for one region and a different one for two — the mask is the *union*, so
  // it lets a cell drive into a neighbour that belongs to some other region entirely. Klei
  // never asks which region a cell is in; it asks what it is standing on. Both rectangles
  // are already clamped to the interior, so the border test they replace is subsumed.
  const World::PaddedRect& ri_far = w->PaddedRegionInclusive(ri);
  auto interior = [&](int32_t x, int32_t y) {
    return x >= ri_far.x0 && x < ri_far.x1 && y >= ri_far.y0 && y < ri_far.y1;
  };

  // One pair, which is `UpdatePressure` itself. `c` is the cell the sweep is
  // standing on; whether it gives or receives falls out of the sign.
  //
  // `axis` is which pair of the flow accumulator this neighbour writes: 0 for the
  // horizontal one, 1 for the vertical one, and -1 for the diagonal, which writes nothing
  // at all. That is not an omission — Klei's diagonal call is the one of the
  // three with no accumulate after it, so a cell whose gas only ever moves diagonally
  // publishes no flow.
  auto transfer = [&](size_t c, size_t gc, uint16_t ec, uint8_t sc, float rate_c, int32_t nx,
                      int32_t ny, int axis) {
    if (!interior(nx, ny)) return;
    const size_t nb = static_cast<size_t>(ny) * pw + nx;
    if (props[nb] & kGasImpermeable) return;
    // Promotion gate: refuse the pair outright if the neighbour's room has been handed to
    // the new engine -- a promoted cell takes no delivery via this sweep, in either
    // direction (the driving cell's own gate below covers the case where *it* is the
    // promoted one).
    if (rooms != nullptr && nb < rooms->room_of.size() &&
        gas::IsRoomOwned(*rooms, rooms->room_of[nb])) {
      return;
    }
    const uint16_t en = start[nb].element;
    // The gate that closes a cell for the rest of the substep once something has changed
    // its element -- a fill from vacuum, or a state change earlier in the frame.
    if (cells[nb].element != en) return;
    const uint8_t sn = table.Phase(en);
    if (sn > kStateGas) return;
    // Same element, or a vacuum/gas pair. Two *different* gases are refused outright: this
    // kernel never mixes them, and what does move them past each other is
    // `DoGasPressureDisplacement`, which is not modelled.
    if (ec != en && sn == sc) return;

    // The driving difference comes out of `start`; everything below is written straight
    // into the live grid. A cell that has already given to one neighbour still offers its
    // full start mass to the next, but the mass it has left is what actually leaves.
    const float mc = start[c].mass, mn = start[nb].mass;
    // The rate belongs to the cell the sweep is on when that cell is a gas, and to the
    // neighbour when it is not -- vacuum has no flow rate to lend.
    const float rate = sc == kStateGas ? rate_c : table.At(en).flow;
    float amount = rate * (mc - mn);
    if (amount > mc * pressure_cap) amount = mc * pressure_cap;
    if (amount < mn * -pressure_cap) amount = mn * -pressure_cap;

    // The accumulate Klei does with `UpdatePressure`'s return value, for the
    // horizontal pair and for the vertical one. It is the *signed capped*
    // amount and not the mass that actually left, which are the same number here: the cap
    // is `0.125 * start.mass` on either side, so the clamp against the source's own mass
    // below can never bite.
    //
    // The slot each end writes is chosen so that the texture's `[0] - [1]` and `[3] - [2]`
    // both come out as `-step * amount`, the same value in the cell that gave and the cell
    // that took. `hdir` is in it because the horizontal step is `hdir` rather than 1.
    if (axis >= 0) {
      const size_t gn = static_cast<size_t>(ny - 1) * gw + static_cast<size_t>(nx - 1);
      if (axis == 0) {
        const float h = static_cast<float>(hdir) * amount;
        w->AddFlow(c, gc, 1, h);
        w->AddFlow(nb, gn, 0, -h);
      } else {
        w->AddFlow(c, gc, 2, amount);
        w->AddFlow(nb, gn, 3, -amount);
      }
    }

    const bool forward = amount >= 0.0f;
    const size_t src = forward ? c : nb;
    const size_t dst = forward ? nb : c;
    const uint16_t src_element = forward ? ec : en;
    const uint16_t dst_element = forward ? en : ec;
    const uint8_t dst_state = forward ? sn : sc;
    const float src_start_mass = forward ? mc : mn;

    float moved = forward ? amount : -amount;
    if (moved > src_start_mass) moved = src_start_mass;

    // Energy ledger, over the whole rest of the lambda -- the element write that decides
    // what the delivered mass is worth is at the very bottom, past three more returns.
    // `ts` below is the source's LIVE temperature, so both ends agree about the heat
    // travelling with the mass; what does not conserve is that `moved` is capped against
    // `src_start_mass`, the SNAPSHOT, while the source's debit is floored at zero -- a cell
    // that has already given mass away this sweep can be asked for more than it still has.
    // That, the temperature clamp, a destination that stays vacuum, and the Void branch that
    // destroys the delivery outright are what this collects.
    const MoverCharge mover_charge(w, table, dst, src);
    PhaseEntry& d = cells[dst];
    const float ts = cells[src].temperature;
    const float total = d.mass + moved;
    if (total <= 0.0f) {
      d.temperature = 0.0f;
    } else {
      // The mass-weighted mean, then clamped back between the two temperatures. The clamp
      // is Klei's, and it is not redundant in float: a mix of a large cold
      // cell and a tiny hot one can round outside the pair.
      const float mix = (d.temperature * d.mass + ts * moved) / total;
      const float lo = d.temperature < ts ? d.temperature : ts;
      const float hi = d.temperature < ts ? ts : d.temperature;
      d.temperature = mix < lo ? lo : (mix > hi ? hi : mix);
    }
    d.mass = total;

    PhaseEntry& s = cells[src];
    s.mass = s.mass - moved > 0.0f ? s.mass - moved : 0.0f;

    // `UpdatePressure`'s germ transfer.
    // Unlike `start`, which this whole function prices mass against, disease is read and
    // written **live** here -- there is no dedicated disease snapshot for the gas pass the
    // way `StepFlow` takes one for the liquid pass (see `DiseaseSweepStart` there). A germ
    // this call moves is visible to the very next pair the sweep processes, same substep.
    //
    // `frac` is priced against `src_start_mass`, the same start-of-substep mass `moved` was
    // capped against above, not the live (already-decremented) mass. The game does not guard
    // this division: it divides raw, then clamps the result to [0, 1] with SSE min/max before
    // multiplying by the count. For zero masses the division is IEEE NaN, and SSE min/max do
    // not propagate a NaN the way `std::min`/`std::max` would: if the *first* operand is NaN
    // the instruction returns the *second*. `min(NaN, 1.0)` is therefore `1.0`, and the clamp
    // turns "priced against zero mass" into "move the whole count" -- the clean, undecayed
    // whole-count relocation the game shows (`germvac`). A `src_start_mass > 0` guard would
    // skip exactly the "stranded duplicant in true vacuum" case.
    if (w->DiseaseActive()) {
      // `src_count` prices against the snapshot (`GasDiseaseSweepStart`), not the live grid,
      // as the game does. Reading live would let a cell that has just *received* germs this
      // same call become, a few iterations later in the same horizontal sweep, a fresh source
      // for its own next pair, chaining a single seed across the entire room in one substep.
      // The game moves exactly one row (`germvac`, tick 2).
      //
      // `src_idx`, unlike `src_count`, reads the **live** grid. This is the gate that limits a
      // single source to one real transfer per substep even though this block runs three
      // times (once per neighbour direction): the first hit that fully drains the source also
      // runs `ClearDisease`, which sets the *live* idx to 0xFF. The second and third hits then
      // read that 0xFF back as `src_idx`, and `AddDiseaseToCell`'s merge rule
      // (`MergeDiseaseCounts`, `idx_b == 0xFF`) treats an incoming 0xFF as "keep the resident,
      // drop the newcomer" -- so the call still runs but is a no-op. Reading idx from the
      // snapshot would hand the destination a full-strength duplicate on every hit, tripling
      // the count instead of moving it once. The source empties out and stays cleared, which
      // is why the game's diseaseIdx reads 0xFF there afterwards.
      const int32_t src_count = dstart.empty() ? 0 : dstart[src].count;
      const uint8_t src_idx = w->DiseaseIdx(src);
      float frac = moved / src_start_mass;
      // SSE min/max semantics: a NaN *first* operand resolves to the *second* operand, so the
      // NaN case is handled explicitly rather than left to the comparison.
      frac = std::isnan(frac) ? 1.0f : (frac < 1.0f ? frac : 1.0f);
      frac = frac > 0.0f ? frac : 0.0f;
      const int32_t amount = static_cast<int32_t>(frac * static_cast<float>(src_count));
      AddDiseaseToCell(w, diseases, dst, src_idx, amount);
      // The *live* count is decremented cumulatively, on every hit, as in the game -- not
      // overwritten from
      // the snapshot constant. With a full drain on hit one this collapses to the same result
      // either way, but a partial-frac transfer (non-vacuum neighbours) needs the true
      // cumulative live value or the second and third hits price against a stale number.
      const int32_t remaining = w->Disease()[src].count - amount;
      if (remaining < 1) {
        w->MutableDiseaseIdx(src) = 0xFF;
        w->MutableDisease(src).count = 0;
        w->MutableDisease(src).diseaseHash = 0;
        w->MutableDiseaseInfest(src) = 0;
        w->MutableDiseaseAccum(src) = 0.0f;
      } else {
        w->MutableDisease(src).count = remaining;
      }
    }

    // Only a destination that *was* in the vacuum state takes an element, and this is the
    // write the live-versus-start gate above is reading.
    if (dst_state != kStateVacuum) return;
    if (dst_element != voidelem) {
      d.element = src_element;
      // This is the single largest source of substance traffic in a world with
      // any vacuum in it, and it fires on pairs where **nothing moves**: two adjacent vacuum
      // cells transfer `flow * (0 - 0)` = 0, the destination is still vacuum-state, and it
      // is announced anyway as `211 -> 211`. A sealed vacuum pocket therefore reports every
      // one of its cells on every physics frame forever. That is not a wasteful accident to
      // be tidied up — it is what `voidbox` measures, and it is why the vacuum branch of
      // `PostProcessCell` is *not* the source, which cost a long detour to establish.
      w->TouchSubstance(dst);
      return;
    }
    // Void: the mass is not delivered, it is destroyed.
    d.mass = 0.0f;
    d.temperature = 0.0f;
  };

  // The rectangles, not the grid. `x_first`/`x_last` are per-rectangle because the column
  // order is `hdir`-signed: ascending from `x0`, or descending from `x1 - 1`.
  //
  // **The two directions are not symmetric, and that is measured.** Ascending covers
  // `[x0, x1)`; descending covers `(x0, x1)` — it stops one short and never drives the
  // region's own first column. The asymmetry is invisible on any region that starts at
  // `min_x = 0`, because the column it drops is then the world's first column, which is
  // border in every scenario that existed before the region probes; that is why this stood
  // as `r.x0 - 1` for as long as nothing sent an inset region. `edgecolx` separates them:
  // two gas columns against a shell with `min_x = 1`, where the dropped column is live gas
  // and the descending walk driving it costs 0.12 kg over 30 ticks. With `r.x0` the whole
  // region suite is exact and all 31 scenarios that predate it are unchanged.
  {
    const int32_t x_first = hdir > 0 ? r.x0 : r.x1 - 1;
    const int32_t x_last = hdir > 0 ? r.x1 : r.x0;
    for (int32_t y = r.y0; y < r.y1; ++y) {
      ONI_EXAMINED(kGasPressure, r.x1 - r.x0);
      for (int32_t x = x_first; x != x_last; x += hdir) {
        const size_t c = static_cast<size_t>(y) * pw + x;
        if (props[c] & kGasImpermeable) continue;
        // Promotion gate: a promoted cell never drives this vanilla sweep either --
        // `transfer`'s own gate above stops it receiving, this stops it giving.
        if (rooms != nullptr && c < rooms->room_of.size() &&
            gas::IsRoomOwned(*rooms, rooms->room_of[c])) {
          continue;
        }
        const uint16_t ec = start[c].element;
        if (cells[c].element != ec) continue;
        const uint8_t sc = table.Phase(ec);
        if (sc > kStateGas) continue;
        const float rate_c = table.At(ec).flow;
        const size_t gc = static_cast<size_t>(y - 1) * gw + static_cast<size_t>(x - 1);

        // Klei re-tests the cell between pairs, and only the live-element test can have
        // changed: a vacuum cell that has just been filled stops here.
        transfer(c, gc, ec, sc, rate_c, x + hdir, y, 0);
        if (cells[c].element != ec) continue;
        transfer(c, gc, ec, sc, rate_c, x, y + 1, 1);
        if (cells[c].element != ec) continue;
        const uint8_t up = table.Phase(start[c + static_cast<size_t>(pw)].element);
        const uint8_t side = table.Phase(start[static_cast<size_t>(
            static_cast<ptrdiff_t>(c) + hdir)].element);
        if (((up | side) & kStateMask) == kStateSolid) continue;
        transfer(c, gc, ec, sc, rate_c, x + hdir, y + 1, -1);
      }
    }
  }
}

// -------------------------------------------------------------------- gas displacement
//
// The second gas sweep, part of `SimBase::UpdateData` and calling
// `DoGasPressureDisplacement` four times per cell. It is the only thing that
// moves two *different* gases past each other: the pressure sweep above refuses that pair
// outright, so without this one an oxygen pocket and a carbon dioxide pocket sit against
// each other forever.
//
// A pair is a run of three cells in a straight line -- source, destination, and the cell
// one further out, called the cell "beyond" here. The destination's **whole
// contents** are shoved out into `beyond` and the destination is cleared; the source then
// puts a slice of itself into the hole it just made. Hence the name: nothing is exchanged,
// the middle cell is evicted.
//
// The four directions are visited in a fixed order -- down, `-hdir`, `+hdir`, up -- at call
// sites. Unlike the pressure sweep,
// the columns are walked in ascending x whatever `hdir` is; `hdir` only picks which
// horizontal neighbour is tried first.
//
// The gates, in Klei's order (the call site tests the first three against `start`, the
// callee re-tests some of them against the live grid):
//
//   1. `start` says the source is a gas.
//   2. `start` says the destination is a gas, and a *different* element from the source.
//   3. The destination is not gas-impermeable.
//   4. `start.mass[src] > start.mass[dst]`  -- pressure, out of the snapshot.
//   5. The source and the destination both still hold their `start` element, and the
//      destination still has mass. This is the same live-versus-start gate the pressure
//      sweep uses, and it is why a cell that has already been displaced this sweep is
//      closed to the rest of it.
//   6. `beyond` holds either the destination's own element or Vacuum, and is not
//      gas-impermeable. There is nowhere else for the evicted gas to go.
//
// The transfer is `min(live.mass[src], 0.125 * start.mass[src])` -- the same 0.125 as the
// pressure sweep's cap, at the same constant, but here it is the whole rule
// rather than a clamp on a rate. That is where the oscillating pair
// gets its 0.25 kg from: 0.125 of 2.0 kg, every substep, in whichever direction `hdir` is
// pointing.
//
// The destination's temperature afterwards is the source's *start* temperature, copied
// rather than mixed, because the destination was emptied first. `beyond` does get a
// mass-weighted mix, with the same clamp as everywhere else.
//
// **The sweep insets itself three cells**, where every other sweep insets
// one: x runs over `[max(x0, 3), min(x1, width - 3))`. That is enough room for `beyond`.
// The y clamp is `min(y0, 3)` where it plainly means `max` -- Klei's own copy-paste slip,
// which we reproduce -- so the sweep starts at row 0 of a full-grid region and would index
// two rows below the grid. Klei gets away with it because a world's bottom rows are solid
// border and gate 1 stops there; a scenario with gas on row 0 or 1 would walk off the front
// of the arrays in Klei and be silently skipped here.
//
// Germs: the destination's germs go with it into `beyond` and the
// destination is cleared of them (`DoDisplacement`), and then an eighth of the source's START
// count, `(int)(0.125 * start.diseaseCount[src])`, follows the source's gas into `dst`, the
// source paying the same number. `diseases` carries the table; with none, nothing moves. The
// promotion gate below refuses a promoted cell before any of it, so germs riding on gas need
// nothing further here.
//
// `rooms`: the same promotion gate as `StepGasPressure`/`StepConduction`, needed here
// because this is the *other* half of pressure-driven flow between cells --
// the mechanism that moves two different gases past each other by evicting `dst` into
// `beyond` and pulling `src` in behind it. Gating only `StepGasPressure` and leaving this
// kernel alone would still let vanilla overwrite a promoted cell's mass/element/temperature
// outright, just via displacement instead of ordinary flow. `nullptr` (every existing call
// site) is the untouched fast path, exactly as the other two gated kernels.
inline void StepGasDisplacement(World* w, const ElementTable& table, int32_t hdir,
                                size_t ri, const gas::RoomGraph* rooms = nullptr,
                                const DiseaseTable* diseases = nullptr) {
  ONI_SWEEP(kGasDisplacement);
  const int32_t pw = w->PaddedWidth();
  const int32_t gw = w->GameWidth(), gh = w->GameHeight();
  std::vector<PhaseEntry>& cells = w->Phases();
  const float displacement_share = g_tunables.gas_pressure_displacement_share;  // THE ONE RULE
  const float germ_share = g_tunables.gas_displacement_germ_share;
  const std::vector<uint8_t>& props = w->Properties();
  std::vector<PhaseEntry>& start = GasSweepStart();
  // `StepGasPressure` owns the snapshot; without it there is nothing to sweep.
  if (start.size() != cells.size()) return;
  const uint16_t vacuum = table.VacuumIndex();
  // The germ snapshot `StepGasPressure` took for this region (its own, not the liquid
  // sweep's), which Klei's `cells` still is here. Gated like the gas sweep's transfer,
  // on a world that holds germs at all.
  const std::vector<DiseaseEntry>& dstart = GasDiseaseSweepStart();
  const bool have_dstart = dstart.size() == w->PaddedCount();
  const bool carry_germs = diseases != nullptr && w->DiseaseActive();

  // `DoGasPressureDisplacement` itself, minus the call site's three `start` gates.
  auto displace = [&](size_t src, size_t gsrc, uint16_t ec, size_t dst, size_t gdst,
                      size_t beyond) {
    // Promotion gate, checked first (correctness independent of the ordering below): all
    // three participating cells matter here, not just two -- `dst` and `beyond` both get
    // overwritten, `src` gets drained, so any one of the three being promoted must refuse
    // the whole displacement.
    if (rooms != nullptr &&
        ((src < rooms->room_of.size() && gas::IsRoomOwned(*rooms, rooms->room_of[src])) ||
         (dst < rooms->room_of.size() && gas::IsRoomOwned(*rooms, rooms->room_of[dst])) ||
         (beyond < rooms->room_of.size() &&
          gas::IsRoomOwned(*rooms, rooms->room_of[beyond])))) {
      return;
    }
    if (!(start[src].mass > start[dst].mass)) return;
    if (cells[src].element != ec) return;
    const uint16_t ed = start[dst].element;
    if (cells[dst].element != ed) return;
    if (!(cells[dst].mass > 0.0f)) return;
    const uint16_t eb = cells[beyond].element;
    if (eb != ed && eb != vacuum) return;
    if (props[beyond] & kGasImpermeable) return;

    // Energy ledger: three cells move here, not two -- the destination empties into
    // `beyond`, then the source moves in behind it -- so the charge spans all three. The
    // destination is refilled at `start[src].temperature`, the SNAPSHOT, while the source is
    // debited live, which is the same disagreement the liquid mover has.
    const MoverCharge mover_charge(w, table, dst, beyond, src);
    // `DoDisplacement`: the destination empties into `beyond`.
    PhaseEntry& d = cells[dst];
    PhaseEntry& b = cells[beyond];
    const float total = b.mass + d.mass;
    if (total > 0.0f) {
      const float mix = (b.temperature * b.mass + d.temperature * d.mass) / total;
      const float lo = b.temperature < d.temperature ? b.temperature : d.temperature;
      const float hi = b.temperature < d.temperature ? d.temperature : b.temperature;
      b.temperature = mix < lo ? lo : (mix > hi ? hi : mix);
    }
    b.mass = total;
    b.element = d.element;
    // The disease half of `DoDisplacement`'s `AddMassAndUpdateTemperature`: the destination's
    // live germs, all of them, merged into `beyond`. `total` is positive, since `d.mass` was
    // gated above.
    if (carry_germs) {
      AddDiseaseToCell(w, *diseases, beyond, w->DiseaseIdx(dst), w->Disease()[dst].count);
    }
    // `SimData::ClearCell`: Vacuum, no mass, no temperature, and no germs, all
    // four disease fields.
    d.element = vacuum;
    d.mass = 0.0f;
    d.temperature = 0.0f;
    ClearCellDisease(w, dst);

    // And the source moves in behind it.
    float moved = displacement_share * start[src].mass;
    if (moved > cells[src].mass) moved = cells[src].mass;
    d.mass += moved;
    d.temperature = start[src].temperature;
    d.element = ec;
    PhaseEntry& s = cells[src];
    s.mass = s.mass - moved > 0.0f ? s.mass - moved : 0.0f;

    // The disease slice, the last thing `DoGasPressureDisplacement` does: an eighth of
    // the source's START germ count, `(int)((float)count * 0.125f)` (the tunable
    // `GasDisplacementGermShare`), of its START germ type, merged into `dst`; the source loses
    // the same number with no germ-type test. The liquid
    // sweep's slice is the same code (`StepLiquidDisplacement`).
    if (carry_germs) {
      const uint8_t germ_idx = have_dstart ? dstart[src].idx : w->DiseaseIdx(src);
      const int32_t germ_count = have_dstart ? dstart[src].count : w->Disease()[src].count;
      const int32_t germs = static_cast<int32_t>(static_cast<float>(germ_count) * germ_share);
      AddDiseaseToCell(w, *diseases, dst, germ_idx, germs);
      ModifyCellDiseaseCount(w, src, -germs);
    }

    // And its three twins. **All
    // four directions write the y pair**, the two horizontal ones included, so a gas shoved
    // sideways by this sweep publishes as vertical flow. That is Klei's copy-paste and not
    // a reading error: the four sites are byte-for-byte the same shape, and `traceflow`
    // measures a purely horizontal displacement arriving in `flow.g`.
    //
    // `beyond` gets nothing, even though it is the cell that took the destination's whole
    // contents. Only the source and the destination are written.
    w->AddFlow(src, gsrc, 3, -moved);
    w->AddFlow(dst, gdst, 2, moved);

    // Three announcements for two cells. `DoDisplacement` announces both ends of its move,
    // and then `DoGasPressureDisplacement` announces the
    // destination again as the source moves in behind it; the duplicate
    // disappears in the publish-time `unique`. `beyond` reads unchanged whenever it already
    // held the destination's gas, which is the usual case and is where `gasmix`'s
    // `207 -> 207` pairs come from.
    w->TouchSubstance(beyond);
    w->TouchSubstance(dst);
  };

  auto try_dir = [&](size_t c, uint16_t ec, int32_t x, int32_t y, int32_t dx, int32_t dy) {
    // Klei has no such test and reads off the end of its arrays when the region reaches
    // the bottom rows; see the note above.
    const int32_t bx = x + 2 * dx, by = y + 2 * dy;
    if (bx < 0 || bx >= gw || by < 0 || by >= gh) return;
    const ptrdiff_t off = static_cast<ptrdiff_t>(dy) * pw + dx;
    const size_t dst = static_cast<size_t>(static_cast<ptrdiff_t>(c) + off);
    const size_t beyond = static_cast<size_t>(static_cast<ptrdiff_t>(c) + 2 * off);
    const uint16_t ed = start[dst].element;
    if (ed == ec) return;
    if (props[dst] & kGasImpermeable) return;
    if (table.Phase(ed) != kStateGas) return;
    const size_t gsrc = static_cast<size_t>(y) * static_cast<size_t>(gw) +
                        static_cast<size_t>(x);
    const size_t gdst = static_cast<size_t>(y + dy) * static_cast<size_t>(gw) +
                        static_cast<size_t>(x + dx);
    displace(c, gsrc, ec, dst, gdst, beyond);
  };

  // The loop bounds are the region rectangle, not the active mask: Klei iterates each
  // rectangle in turn and never tests a cell for membership, so overlapping regions sweep
  // their overlap twice. This is the one sweep that wants the rectangle in *game*
  // coordinates, because its clamps are written against `gw`/`gh`.
  //
  // **The inset is 2.** A bound of 3 with a half-open row loop looks equally plausible, and
  // four measurements rule it out, each of them a three-cell displacement placed so that
  // exactly one clamp can refuse it:
  //
  //   * `tracerow` / `traceflow` -- source on row `height - 3` of a 12-high world: fires.
  //   * `tracetall` -- the same row of a 16-high world, so the bound is measured from the
  //     top of the world rather than being a fixed row: fires.
  //   * `tracewall` -- source on the first interior column, pushing right: fires.
  //   * `tracewall` -- source on the last interior column, pushing left: fires.
  //
  // The constant here is what all four probes measure, and the suite agrees with it
  // everywhere else.
  {
    const World::Rect r = w->Region(ri);
    const int32_t x0 = r.min_x > 2 ? r.min_x : 2;
    int32_t x1 = r.max_x < gw - 2 ? r.max_x : gw - 2;
    const int32_t y0 = r.min_y < 2 ? r.min_y : 2;
    const int32_t y1 = r.max_y < gh - 2 ? r.max_y : gh - 2;
    // The row is also capped at the region's own width, which the clamps
    // above can only have shortened. Kept because the game does it.
    if (x1 - x0 > r.max_x - r.min_x) x1 = x0 + (r.max_x - r.min_x);
    // The one kernel whose loop is not bounded by its region: `min(min_y, 2)` puts its
    // first row near the bottom of the world however high the region starts, so it can
    // publish rows the region never covers, and `Project`'s bound has to hear about it.
    // Game coordinates, and the body works on `(x + 1, y + 1)`.
    w->MarkProjectDirtyRect(x0 + 1, y0 + 1, x1 + 1, y1 + 1);
    for (int32_t y = y0; y < y1; ++y) {
      // The rectangle this sweep actually walks is NOT its region -- see the comment above
      // the clamps, `min(min_y, 2)` puts its first row near the bottom of the world however
      // high the region starts. So this is the one sweep whose `examined` is expected to
      // exceed the region's area, and the census is where that stops being a comment and
      // becomes a number.
      ONI_EXAMINED(kGasDisplacement, x1 - x0);
      for (int32_t x = x0; x < x1; ++x) {
        const size_t c = static_cast<size_t>(y + 1) * pw + (x + 1);
        const uint16_t ec = start[c].element;
        if (table.Phase(ec) != kStateGas) continue;
        try_dir(c, ec, x, y, 0, -1);
        try_dir(c, ec, x, y, -hdir, 0);
        try_dir(c, ec, x, y, hdir, 0);
        try_dir(c, ec, x, y, 0, 1);
      }
    }
  }
}

// One flow substep.
//
// The update scheme is neither pure Jacobi nor pure Gauss-Seidel, and it took a probe with
// liquid in the *middle* of a channel to see which: a cell pushing to both its left and
// its right gave 125 kg left and only 93.75 kg right, which is 0.25 of what it had *after*
// the first push. So a cell spends its own mass as it goes, but reads every neighbour's
// mass as it was at the start of the substep. Reading neighbours live instead produces
// visibly different numbers three cells along.
// `rotation` is the game's substep counter: zero at construction, incremented
// once at the end of every substep, and read by `DisplaceGas` to decide where the gas a
// liquid pushes out of a cell ends up. It is the only piece of sim state outside the grid
// that this kernel reads.
// `rooms`, the liquid half of the promotion gate. The predicate is `gas::IsMixtureOwnedCell`, NOT `IsRoomOwned` -- see that function's
// comment for why the liquid kernels ask a narrower question than the gas ones.
//
// What it is actually preventing, which is a live path and not a hypothetical: a promoted cell
// sits at vacuum/zero mass in vanilla's grid (`ClearCell`), `permeable` below is
// `Phase(element) < kStateLiquid` so vacuum qualifies, and `move_into`'s guards refuse a
// LIQUID destination and send a GAS one through `DisplaceGas` -- but a VACUUM destination
// passes both and takes `d.element = elem` plus the mass. So without this gate vanilla liquid
// pours straight into a promoted cell and overwrites the `PhaseEntry.temperature` field the
// mixture layer uses as its shared temperature (`world.h`'s SoA comment).
//
// Closing it by refusal alone would freeze liquid in place: water on top of mixture gas would
// hang in mid-air for good. So the gate no longer stops at refusal. Wherever
// liquid meets a mixture-owned cell, the mixture gas moves aside first -- into a gas
// neighbour of its room (`gas::EvictMixture`), or up past the liquid on a swap
// (`gas::SwapMixture`) -- and the liquid then moves exactly as it would over vanilla gas. Only
// gas with nowhere to go still stops the liquid, as vanilla's own `DisplaceGas` does.
//
// `nullptr` (the default, and the entire offline suite) is the untouched fast path.
// `falling`, `visible` and `debug_editing` are `SpawnFallingLiquid`'s: where a liquid that
// leaves the grid as a falling particle is reported, and the game's visibility mask and debug
// flag that gate it. A null `falling` never spawns, which is what `bench` passes.
inline void StepFlow(World* w, const ElementTable& table, bool first_physics,
                     uint16_t rotation, size_t ri, gas::RoomGraph* rooms = nullptr,
                     std::vector<SpawnFallingLiquidInfo>* falling = nullptr,
                     const uint8_t* visible = nullptr, bool debug_editing = false,
                     const DiseaseTable* diseases = nullptr) {
  ONI_SWEEP(kFlow);
  const World::PaddedRect& r = w->PaddedRegion(ri);
  const int32_t pw = w->PaddedWidth();
  const size_t gw = static_cast<size_t>(w->GameWidth());
  std::vector<PhaseEntry>& cells = w->Phases();
  const Tunables tun = g_tunables;  // tunables.h, THE ONE RULE

  const std::vector<uint8_t>& props = w->Properties();

  // `world.h` owns this one, because the flow texture reads it again after the substep is
  // over. See `LiquidSweepStart`.
  // One region's share, not the whole grid once per region -- `SnapshotReadRect` (world.h)
  // carries the numbers. This is the snapshot that cost the most: 1.19 MB copied per region,
  // and `StepFlow` went 0.163 -> 1.203 ms across a twelve-region cluster doing identical
  // cell work. `StepLiquidDisplacement` reads it after this sweep and reaches three cells
  // wider, which is what sets the margin.
  std::vector<PhaseEntry>& start = LiquidSweepStart();
  RefreshPhaseSnapshot(start, cells, ri, pw, w->PaddedHeight(), r.x0, r.y0, r.x1, r.y1);

  // The disease half of the same `CellSOA::CopyFrom`: the germ type and count every liquid
  // transfer this substep is priced against, here and in `StepLiquidDisplacement`. The pair
  // sweep does NOT read this one: Klei copies the grid again after both liquid sweeps
  // and `Disease::UpdateCells` reads that copy, so `StepDiseaseDiffusion`
  // retakes it. Before liquid carried germs the two copies held the same counts.
  if (w->DiseaseActive()) {
    std::vector<DiseaseEntry>& dstart = DiseaseSweepStart();
    RefreshDiseaseSnapshot(dstart, *w, ri, pw, w->PaddedHeight(), r.x0, r.y0, r.x1, r.y1);
  } else {
    DiseaseSweepStart().clear();
  }
  const std::vector<DiseaseEntry>& dstart = DiseaseSweepStart();
  const bool have_dstart = dstart.size() == w->PaddedCount();
  // Germs ride on every transfer the mover makes: `UpdateLiquid` hands
  // `UpdateNeighbourLiquidMass` the source's START germ type and a share of its running count,
  // and the mover merges them into the destination with the mass. Gated like the gas
  // sweep's transfer, on a world that holds germs at all; a caller with no table (the
  // scaffolding drivers) moves none.
  const bool carry_germs = diseases != nullptr && w->DiseaseActive();

  // `SpawnFallingLiquid`, bound to this sweep's list, mask and flag.
  auto spawn_falling = [&](size_t at, uint16_t element, float mass, float temperature,
                           uint8_t disease_idx, int32_t disease_count) -> bool {
    return SpawnFallingLiquid(w, table, falling, visible, debug_editing, at, element, mass,
                              temperature, disease_idx, disease_count, tun);
  };

  // Inflow is written **straight into the live grid**, the way the game writes it: the mover
  // ends in `AddMassAndUpdateTemperature` against the live grid, and the receiving cell holds the new mass from that instant on.
  //
  // Deferring it to the end of the sweep looks equivalent, because the measurement it exists
  // to satisfy is about the *amount* rather than the grid: on a
  // settled pool in vacuum the leftmost and the rightmost wet cell each push exactly
  // 0.25 x 125 kg outward, even though the right-hand one has already been handed another
  // 125 kg by its left neighbour in the same sweep. Pricing against the live mass would make
  // that cell push 0.25 x 250 kg and put twice as much water one cell further along the floor.
  //
  // The game gets that result while writing live, because a cell's turn opens by taking its
  // **start** mass and decrementing that copy as it spends. Inflow it received earlier in
  // the sweep is in the grid but not in the copy. `src_mass` below is that copy, and it is
  // what every transfer is priced against.
  //
  // Deferring is not equivalent, because the mover reads the destination's element out of
  // the live grid. See `move_into`.

  // "Interior" now also means "inside the region being stepped": mass may not move into a
  // cell this pass is not stepping, any more than heat may. The rectangle rather than the
  // mask, for the same reason conduction takes it — the mask is the union of every region
  // and would let mass cross a seam that Klei's per-region sweep never reaches across. It is
  // already clamped to the interior, so the border test it replaces is subsumed.
  auto interior = [&](int32_t x, int32_t y) {
    return x >= r.x0 && x < r.x1 && y >= r.y0 && y < r.y1;
  };

  // **One** sweep, bottom-up and left-to-right, one turn per cell. There is no separate
  // swap pass: Klei's `UpdateLiquid` handles the whole-cell swap, the downward pour and the
  // sideways spread for a single cell in a single call, and returns the moment it swaps.
  //
  // That structure matters, because it is what produces the alternating-column pattern that
  // a wide sheet of liquid falls in. See the permeability probes below.
  //
  // The rectangle, not the grid: `interior` accepted exactly these cells, and it stays
  // for the *destination* test, which is a different question. See `World::PaddedRegions`.
  {
    for (int32_t y = r.y0; y < r.y1; ++y) {
      ONI_EXAMINED(kFlow, r.x1 - r.x0);
      for (int32_t x = r.x0; x < r.x1; ++x) {
        const size_t src = static_cast<size_t>(y) * pw + x;
        const size_t gsrc = static_cast<size_t>(y - 1) * gw + static_cast<size_t>(x - 1);
        const uint16_t elem = start[src].element;
        const uint8_t phase = table.Phase(elem);
        // Liquid only. Gas moved out to `StepGasPressure`, which is a separate sweep with a
        // separate `start` buffer, three neighbours instead of four and a cap this one has
        // never had.
        if (phase != kStateLiquid) continue;
        // Promotion gate, giving half. A liquid cell that also carries mixture slots is a
        // real state: a save from an earlier build can hold one in a promoted room's pond cells,
        // and a ModifyCell can still put liquid on top of
        // mixture gas. The slots step aside into a gas neighbour of the same room first
        // (`EvictMixture`); only a cell whose gas has nowhere to go is refused, and it tries
        // again next substep.
        if (gas::IsMixtureOwnedCell(*w, rooms, src) &&
            !gas::EvictMixture(*w, table, rooms, src, rotation)) {
          continue;
        }
        const Element& e = table.At(elem);
        // Liquid spends its own mass as it goes: a 500 kg water cell pushing both ways gave
        // 125 kg left and then only 93.75 kg right, which is a quarter of what remained
        // after the first push. So it reads `cells[src].mass`, which `give` has already
        // decremented, while every *neighbour* it reads comes out of `start`.
        //
        // Gas is the other way round on both counts and now lives in `StepGasPressure`: it
        // offers its start mass to every neighbour in turn, and it writes its deliveries
        // into the live grid rather than deferring them.

        // The cell's mass as the substep began, less whatever it has already
        // spent this turn. Not `cells[src].mass`, which also carries anything the sweep has
        // pushed into it since — see the note on inflow above.
        float src_mass = start[src].mass;
        // Klei's running germ count for the same turn (`local_res8`): the start count, less
        // each share a transfer took. Every transfer prices its germs against it, the mover's
        // and the falling-liquid record's alike.
        const uint8_t src_germ_idx = have_dstart ? dstart[src].idx : w->DiseaseIdx(src);
        int32_t src_germs = have_dstart ? dstart[src].count : w->Disease()[src].count;

        // May this cell offer `dst` anything at all, and what mass does the receiving side
        // count as in the flow formula? These are the *pricing* gates, and every one of them
        // that reads a neighbour reads it out of `start`.
        //
        // Only left, right and up come through here; the downward pour has its own two gates,
        // below. All three that do are gated identically: the
        // same four tests in the same order. A destination holding a *different* element must
        // be vacuum or gas, must not be solid in the live grid, must not be
        // liquid-impermeable, and counts as zero mass. So a liquid over `maxMass` rises into
        // gas and displaces it, which the `squeeze` scenario measures: Klei lifts 250 kg of a
        // 1,500 kg cell into the oxygen above it on the third tick and carries the oxygen up
        // the shaft.
        //
        // Whether the transfer then *lands* is not decided here — that is `move_into`, and it
        // reads the live grid rather than this one.
        auto receiving_mass = [&](size_t dst, float* out) -> bool {
          const PhaseEntry& d = start[dst];
          const uint8_t dphase = table.Phase(d.element);
          if (d.element != elem && dphase > kStateGas) return false;
          // Solidity is re-tested against the *live* grid as well: a cell that froze earlier
          // in this substep is closed for the rest of it.
          if (table.Phase(cells[dst].element) == kStateSolid) return false;
          // And the destination's own permeability, which is a property bit rather than a
          // phase: the `kLiquidImpermeable` bit, read off the *start* grid,
          // in every one of the three directions that reach this test.
          if (props[dst] & kLiquidImpermeable) return false;
          // The neighbour's mass as it was at the *start* of the substep, and only if it
          // held this same liquid then — a test against the start
          // element's phase, with any other case reading as zero. Reading it live
          // looks harmless and is not: a cell whose left and right neighbours both push into
          // it ends up with 1.1227 kg where Klei has exactly 1.1400, because the second
          // pusher sees the first pusher's contribution. Reading the live value downward
          // alone costs `pool` 88 kg, and in all four directions 236 kg.
          *out = dphase == kStateLiquid && d.element == elem ? d.mass : 0.0f;
          return true;
        };

        // Empty `dst` into a neighbour so this liquid can have it, and report whether that
        // was possible. This is Klei's mover — the one every direction's
        // transfer goes through — and the point at which it is called is load-bearing:
        // `UpdateLiquid` computes the transfer, tests it against the element's minimum
        // and against zero, and
        // **only then** calls the mover. A transfer the minimum rejects therefore displaces
        // nothing at all.
        //
        // Running the displacement first, from inside the probe above, is what `pour` was
        // reading. The leading cell of a spreading sheet holds a fraction of a kilogram, so a
        // quarter of it is under water's 0.1 kg minimum horizontal flow and the transfer is
        // dropped — but the gas had already been shoved out of the cell and merged with the
        // gas above it, leaving a vacuum the water never arrived in. On the 20x10 `pour` world
        // that shows at (1,1) on the eighth frame: the game has 1.1271 kg of oxygen there and
        // the wrong order leaves none, with the same 1.1271 kg sitting one cell higher.
        // The kernel itself is `DisplaceGas` above, shared with `DoSublimation` and the
        // liquid off-gas block. Two things are specific to this caller and are passed in as
        // arguments: it is handed the **live** grid, so the gas it looks for is the gas in the cell now and not the
        // gas the substep started with — and every measurement behind this path was taken
        // with the sweep's `start` snapshot and its `interior` region test in place.
        auto displace_gas = [&](size_t dst) -> bool {
          return DisplaceGas(w, table, dst, rotation, &start, true, rooms, diseases);
        };

        // Hand `amount` to `dst`, and report whether `dst` would take it. This is Klei's
        // mover, and **every one of the four directions goes through it** —
        // down, left, right, up.
        // It is the only place a liquid transfer is ever written.
        //
        // Two things about it.
        //
        // **It keys off the live destination element**, and everything after turns on that
        // value:
        // equal to the source element, mass is simply added whatever phase `start` thought
        // the cell was; a *liquid* is refused outright; a *gas* is put through `DisplaceGas`
        // and the transfer is abandoned if that fails. So the pricing gates above may pass a
        // transfer that the mover then refuses, and that is not an edge case — it is what a
        // wide sheet of liquid does on its first frame, on every second column.
        //
        // **The downward pour goes through it as well.** That is the whole of the `falling`
        // and `liquid` divergence. A sheet three rows deep drops its 1st, 3rd and 5th columns
        // into the gas below on the first frame; the cells they vacated now hold that gas,
        // in the live grid, while `start` still says water. The row above then prices a
        // 50 kg pour against `start` — same element, so it is allowed — and Klei's mover
        // finds oxygen underneath it and asks `DisplaceGas` to move it. Where the oxygen has
        // somewhere to go, the pour lands and the oxygen turns up next door; where the column
        // is walled in by its own sheet, `DisplaceGas` fails and **the pour does not happen
        // at all**. Measured on `falling`, second frame, row 11: Klei leaves 1.0 kg of oxygen
        // in the interior columns and 50 kg of water in the edge column, whose displaced
        // kilogram lands at (5,11) and makes it 2.0. We poured into every one of them and
        // read 51 kg across the row.
        //
        // Not modelled: the Void element, which the mover empties the
        // transfer into and reports success. No scenario has one.
        auto move_into = [&](size_t dst, float amount, int32_t germs) -> bool {
          // Promotion gate, receiving half, and the one that closes the real hole described
          // on this kernel. Every one of the four directions goes through this mover -- down,
          // left, right and up -- so one test here covers all of them. The mixture gas steps
          // aside first, as a vanilla gas does through `displace_gas` below; with nowhere to
          // go, the transfer is refused, and that costs the source nothing: `give` subtracts
          // only after the mover returns true.
          if (gas::IsMixtureOwnedCell(*w, rooms, dst) &&
              !gas::EvictMixture(*w, table, rooms, dst, rotation)) {
            return false;
          }
          PhaseEntry& d = cells[dst];
          if (d.element != elem) {
            const uint8_t dphase = table.Phase(d.element);
            if (dphase == kStateLiquid) return false;
            if (dphase == kStateGas && !displace_gas(dst)) return false;
            d.element = elem;
            // The destination changes element, so the mover announces it.
            // Only on this branch: a transfer into a cell that already holds the same
            // liquid adds mass and says nothing.
            w->TouchSubstance(dst);
          }
          // `AddMassAndUpdateTemperature`, against the source's temperature as
          // the substep began. Same element on both sides by the time we get here, so the
          // specific heats cancel and this is a plain mass-weighted mean — and then the same
          // clamp back between the two temperatures that `UpdatePressure` uses,.
          // It is not redundant in float and it is not cosmetic: a
          // 900 kg water cell shedding a slice into a 1 kg gas cell rounds a ten-thousandth of
          // a kelvin outside the pair, and one ulp of temperature is enough to flip the
          // `t > best` scan in `GasShuffle` and swap a different pair of cells.
          // Energy ledger: the destination is credited against the source's temperature as
          // the SUBSTEP began, while `give` below debits the source at the temperature it
          // holds NOW. The two are the same number only for a cell that has not already
          // moved this substep, so the pair does not conserve. Charged over both ends.
          const double dst_before = GridCellEnergy(*w, table, dst);
          AddMassAndUpdateTemperature(&d, amount, start[src].temperature);
          w->NoteMoverEnergy(GridCellEnergy(*w, table, dst) - dst_before);
          // The disease half of the same call: the source's START germ type and the share
          // `give` priced, merged into the destination by `Disease::AddDiseaseToCell`.
          if (carry_germs) AddDiseaseToCell(w, *diseases, dst, src_germ_idx, germs);
          return true;
        };

        // The germs a transfer of `amount` takes with it, against the running mass and count:
        // `(int)((amount / mass) * (float)count)`, computed before the transfer.
        auto germ_share = [&](float amount) -> int32_t {
          return static_cast<int32_t>((amount / src_mass) * static_cast<float>(src_germs));
        };

        // The sideways falling-liquid branch, (left) and (right). A
        // cell standing on something solid that pushes sideways into a cell with an open
        // cell under it does not fill that cell: the slice goes over the edge as a falling
        // particle. The two gates are `IsSolid` on the cell below the source, read off the
        // START grid (solid, or liquid-impermeable), and `IsLiquidPermeable` on the cell below
        // the destination, read off the LIVE grid. If the spawn is refused, the caller falls
        // back to the ordinary mover with the same amount, which is why this was a provable
        // no-op under `diffsim`.
        //
        // On success, Klei runs the mover's common tail without the mover: the source spends
        // the slice, its germs lose their share when the live cell still holds the germ
        // species the turn began with, and the flow slot records the transfer. Nothing is
        // written to `dst`.
        auto fall_sideways = [&](size_t dst, int slot, float amount) -> bool {
          // A particle falls down in the game, so an element whose gravity is inverted
          // (`kSetInvertedGravityElement`) never takes this branch.
          if (elem == w->InvertedGravityElement() || amount <= 0.0f || amount > src_mass) {
            return false;
          }
          const size_t src_below = src - static_cast<size_t>(pw);
          if (table.Phase(start[src_below].element) != kStateSolid &&
              !(props[src_below] & kLiquidImpermeable)) {
            return false;
          }
          const size_t dst_below = dst - static_cast<size_t>(pw);
          if ((props[dst_below] & kLiquidImpermeable) ||
              table.Phase(cells[dst_below].element) >= kStateLiquid) {
            return false;
          }
          const int32_t share = germ_share(amount);
          if (!spawn_falling(dst, elem, amount, start[src].temperature, src_germ_idx, share)) {
            return false;
          }
          src_germs -= share;
          src_mass -= amount;
          const double src_before = GridCellEnergy(*w, table, src);
          if (cells[src].mass > 0.0f) {
            w->PayloadRelease(src, amount >= cells[src].mass ? 1.0f : amount / cells[src].mass,
                              cells[src].temperature, ext::kPayloadReleasedFalling);
          }
          cells[src].mass -= amount;
          if (cells[src].mass < 0.0f) cells[src].mass = 0.0f;
          // The source loses the share only while its live germ type is still the one the
          // turn began with (the live `diseaseIdx` against the snapshot's).
          if (w->DiseaseIdx(src) == src_germ_idx) ModifyCellDiseaseCount(w, src, -share);
          // The slice left the grid for the game, the way a falling solid does.
          w->NoteUnstable(amount);
          w->NoteUnstableEnergy(GridCellEnergy(*w, table, src) - src_before);
          w->AddFlow(src, gsrc, slot, amount);
          return true;
        };

        // Spend `amount` on `dst`, if `dst` will have it. Klei subtracts from the source only
        // after the mover has returned true, and leaves it alone otherwise
        // — so a refused transfer costs the source nothing.
        auto give = [&](size_t dst, int slot, float amount) -> bool {
          if (amount <= 0.0f) return false;
          if (amount > src_mass) amount = src_mass;
          if (amount <= 0.0f) return false;
          // Priced before the mover runs, against the running mass and count, and handed to
          // it; the source pays only once the mover has taken it.
          const int32_t share = germ_share(amount);
          if (!move_into(dst, amount, share)) return false;
          src_germs -= share;
          if (carry_germs && w->DiseaseIdx(src) == src_germ_idx) {
            ModifyCellDiseaseCount(w, src, -share);
          }
          src_mass -= amount;
          const double src_before = GridCellEnergy(*w, table, src);
          // Layer C: the share of the source's liquid that left takes the same share of its
          // payload, priced against the LIVE mass it is leaving, so a cell that gives in several
          // directions in one substep gives each share of what is actually still there.
          if (cells[src].mass > 0.0f) {
            w->PayloadMove(src, dst, amount >= cells[src].mass ? 1.0f : amount / cells[src].mass);
          }
          cells[src].mass -= amount;
          if (cells[src].mass < 0.0f) cells[src].mass = 0.0f;
          w->NoteMoverEnergy(GridCellEnergy(*w, table, src) - src_before);
          // The flow accumulator, and the liquid mover writes it differently from the gas
          // sweeps in two ways. **Only the cell that gave is written** — the destination
          // records nothing — and the slot is a plain function of the direction rather than
          // of a signed amount, because `amount` here is a magnitude. One site per
          // direction, in the order the directions are tried: down,
          // left, right, up, and all four sit *after* the mover's
          // success check, so a refused transfer records nothing either.
          //
          // The source, not the destination, is measured rather than read: the four sites
          // decrement the source's mass and write the flow at the cell whose turn it is, not
          // at the destination. Writing it at the destination puts every value one row below
          // Klei's when `pool` is dumped.
          //
          // The signs that come out of it do not agree with the gas sweeps and are not
          // meant to be reconciled: a liquid falling publishes negative y at the cell it
          // left, and a gas rising publishes negative y at both ends. Each is transcribed
          // from its own site.
          w->AddFlow(src, gsrc, slot, amount);
          return true;
        };

        // ------------------------------------------------------------------ liquid
        //
        // Klei's order is down, left, right, up, and each direction is skipped once the cell
        // has nothing left to give. The cell's own outflow is subtracted as it goes, so the
        // running mass here is `src_mass`; nothing has been added to it, because a cell reads
        // its own mass once, out of `start`, and spends down from there.
        if (phase == kStateLiquid) {
          // Framework extension (not from Klei — abi/sim_abi_ext.h,
          // kSetInvertedGravityElement). `vy` is the sign of "the direction this element's
          // liquid turn treats as down": -1 (vanilla, every element by default) or +1 for the
          // one element currently toggled, in which case this whole turn runs mirrored —
          // "below" means the cell **above**, and vice versa. `fall_slot`/`rise_slot` keep the
          // flow-accumulator (and therefore the flow *texture*) tied to true geometric
          // direction rather than to "gravity", so an inverted element's flow animation still
          // points the way the mass actually moved instead of mislabelling up as down.
          const bool grav_inverted = elem == w->InvertedGravityElement();
          const int32_t vy = grav_inverted ? 1 : -1;
          const int fall_slot = grav_inverted ? 3 : 2;
          const int rise_slot = grav_inverted ? 2 : 3;

          // A cell a liquid can move *through*. Klei's `IsLiquidPermeable` answers no for
          // anything liquid or solid, so a cell holding water is not permeable — and that,
          // read off the **live** grid, is the whole of the alternating-column pattern that
          // a falling sheet of liquid shows. Column 1 swaps down; column 2 then looks left,
          // finds the hole column 1 left behind (permeable) and the water column 1 became
          // (not permeable), and takes the falling-particle branch, which does nothing here.
          // Column 3 looks left at column 2, which is still water, and swaps. And so on.
          //
          // This replaces an "every swap strides past the next column" rule that reproduced
          // the same pattern by construction and could not explain anything else.
          //
          // It tests the cell's liquid-impermeable property bit first, which
          // this probe skipped until the falling-liquid path was built: a closed door or a
          // mesh-blocked cell beside a falling liquid is not a place it can fall past.
          auto permeable = [&](int32_t cx, int32_t cy) {
            if (!interior(cx, cy)) return false;
            const size_t c = static_cast<size_t>(cy) * pw + cx;
            if (props[c] & kLiquidImpermeable) return false;
            return table.Phase(cells[c].element) < kStateLiquid;
          };

          bool turn_over = false;
          if (interior(x, y + vy)) {
            const size_t below = static_cast<size_t>(static_cast<ptrdiff_t>(src) + vy * pw);
            const uint16_t below_elem = start[below].element;
            const uint8_t below_phase = table.Phase(below_elem);
            // Two gates, and only two: the cell below must not be solid **in the live grid**,
            // and it must not be liquid-impermeable.
            // Everything after that turns
            // on the element the substep began with.
            if (table.Phase(cells[below].element) != kStateSolid &&
                !(props[below] & kLiquidImpermeable)) {
              if (below_elem == elem) {
                // Same element below: pour. One expression, not two branches — the room
                // under `maxMass` and the 1% pressure gradient are the two arms of a single
                // `max`, and reading them as alternatives is wrong exactly where a settled
                // pool lives. Measured on `pool`: a 992.525 kg cell over a 997.475 kg one
                // moves 2.4876 kg, the pressure arm; the room arm gives 1.2625 kg.
                const float dm = below_phase == kStateLiquid ? start[below].mass : 0.0f;
                const float sm = src_mass;
                float amount =
                    std::max(0.0f, std::max(sm * tun.liquid_pressure_ratio, e.maxMass) - dm) *
                    tun.liquid_pour_share;
                amount = std::min({amount, sm, e.viscosity});
                // The minimum is only enforced on a cell that holds more than the minimum,
                // so the last dribble of a nearly empty cell still drains.
                if (sm > e.minVerticalFlow && amount < e.minVerticalFlow) amount = 0.0f;
                // Through the mover, exactly like the other three directions — and this is
                // where the cell below being *live* gas rather than *start* water gets the
                // pour cancelled. A false return does not end the turn: skips
                // the transfer and drops straight into the leftward branch.
                give(below, fall_slot, amount);
              } else if (below_phase <= kStateGas) {
                // Vacuum or gas below, so the cell either falls into it whole or is handed
                // to the game as a falling-liquid particle. Which one is decided by four
                // permeability probes, in this order.
                bool swap = false;
                if (permeable(x - 1, y) && !permeable(x - 1, y + vy)) {
                  swap = false;                       // spawn: open to the left, blocked below-left
                } else if (!permeable(x + 1, y)) {
                  swap = true;                        // walled in on the right
                } else if (permeable(x + 1, y + vy)) {
                  swap = true;                        // open below-right
                }
                // The swap branch writes both cells directly and is the one transfer in
                // this kernel that does NOT go through `move_into`, so it needs the gate
                // spelled out again. A mixture-owned cell below reads as vacuum, so
                // `below_phase <= kStateGas` is exactly how it arrives here. Refused, the cell
                // would neither swap nor fall and would stand there for good. So the swap
                // takes the mixture slots up with the PhaseEntry that carries their
                // temperature, so the gas rises past the liquid as a vanilla gas does, and
                // the spawn branch runs as it would over any gas: the falling particle
                // leaves `src` and does not touch `below`.
                const bool mixture_below = gas::IsMixtureOwnedCell(*w, rooms, below);
                if (swap) {
                  SwapCells(w, src, below);
                  if (mixture_below) {
                    gas::SwapMixture(*w, src, below);
                    gas::WakeMixtureCell(*w, rooms, src);
                  }
                  // `UpdateLiquid` announces both ends of the swap, and
                  // — the same shape as the gas shuffle. The spawn branch
                  // below announces only the one cell, but under `diffsim` it never empties the cell, so
                  // there is nothing there to announce.
                  w->TouchSubstance(src);
                  w->TouchSubstance(below);
                } else if (!grav_inverted &&
                           spawn_falling(src, elem, src_mass, start[src].temperature,
                                         src_germ_idx, src_germs)) {
                  // The spawn branch. The game took the whole cell as a falling
                  // particle: the record carries the START mass, temperature and germs, and
                  // the live cell is then cleared to vacuum outright, which
                  // also drops anything a neighbour pushed into it earlier in this sweep.
                  // That is Klei's, and the ledger charges what really left.
                  w->NoteUnstable(cells[src].mass);
                  w->NoteUnstableEnergy(-GridCellEnergy(*w, table, src));
                  w->PayloadRelease(src, 1.0f, cells[src].temperature,
                                    ext::kPayloadReleasedFalling);
                  cells[src].element = table.VacuumIndex();
                  cells[src].mass = 0.0f;
                  cells[src].temperature = 0.0f;
                  w->MutableDiseaseIdx(src) = 0xFF;
                  w->MutableDisease(src) = SaveDisease{};
                  w->TouchSubstance(src);
                }
                // Either way the cell's turn is over. On the spawn branch Klei asks the game
                // to take the liquid as a falling particle and empties the cell only if the
                // game accepts. A headless sim never accepts -- the headless flag makes
                // `SpawnFallingLiquid` return false before doing anything -- so under
                // `diffsim` the cell simply stands still for a substep. That is measured:
                // in `pair`, Klei leaves two 1,000 kg cells exactly where they are for one
                // tick. In the game it is not headless, and a cell that stood still here
                // forever was liquid hanging over gas.
                turn_over = true;
              }
            }
          }
          if (turn_over) continue;

          // Sideways, left before right, then up. Same acceptance test each time: a
          // neighbour holding a *different* element must be vacuum or gas to be entered at
          // all, and the mass it counts as is zero unless it holds this same liquid.
          const int dx[3] = {-1, 1, 0};
          const int dy[3] = {0, 0, -vy};
          // Which flow slot each of the three writes, in the same order. See `give`.
          const int slot[3] = {0, 1, rise_slot};
          for (int k = 0; k < 3; ++k) {
            if (src_mass <= 0.0f) break;
            const int32_t nx = x + dx[k], ny = y + dy[k];
            if (!interior(nx, ny)) continue;
            const size_t dst = static_cast<size_t>(ny) * pw + nx;
            float dm = 0.0f;
            if (!receiving_mass(dst, &dm)) continue;
            const float sm = src_mass;
            float amount = 0.0f;
            if (dy[k] == 0) {
              amount = std::min({tun.liquid_horizontal_rate * (sm - dm), sm, e.viscosity});
              // minHorizontalFlow is a minimum on the *transfer*, not on the difference: a
              // 0.02 kg difference never moves, because a quarter of it is 0.005 and the
              // minimum is 0.01, while a 0.1 kg difference does.
              if (amount < e.minHorizontalFlow) amount = 0.0f;
            } else {
              // Up, which is the only way liquid rises: everything over the greater of
              // maxMass and the neighbour's 1% share, halved. The viscosity cap is skipped
              // for a cell holding more than twice that — Klei really does let a wildly
              // over-pressured cell move more than its viscosity in one substep.
              const float ceiling = std::max(dm * tun.liquid_pressure_ratio, e.maxMass);
              amount = std::max(0.0f, sm - ceiling) * tun.liquid_upward_share;
              if (sm < ceiling * tun.liquid_viscosity_bypass_ratio) {
                amount = std::min({amount, sm, e.viscosity});
              }
              if (amount <= tun.liquid_min_upward_flow) amount = 0.0f;
            }
            // Both minimums are behind us, so this is where Klei calls the mover, and the
            // mover is what displaces. A cell it cannot empty is a transfer that does not
            // happen: a false return falls straight through to the next
            // direction, which is what a false return from `give` amounts to here. The two
            // sideways directions first offer the slice as a falling particle; a refusal
            // goes to the mover with the same amount.
            if (dy[k] == 0 && amount > 0.0f && fall_sideways(dst, slot[k], amount)) continue;
            give(dst, slot[k], amount);
          }
          continue;
        }

      }
    }
  }

  (void)first_physics;
}

// ----------------------------------------------------- liquid pressure displacement
//
// The liquid twin of `StepGasDisplacement`, and the only thing in the sim that moves two
// **different** liquids past each other. The flow mover above refuses that pair outright
// (`move_into` returns false for a destination holding a different liquid), so without this
// sweep a pool of water and a pool of some other liquid sit against each other forever —
// which is exactly what `sunliquid` measured: Klei turns a whole column of water into the
// heavier liquid over one tick and this sim left it alone.
//
// It is a second pass over the region, run in `SimBase::UpdateData` immediately after
// the `UpdateLiquid` loop and before the `CellSOA::CopyFrom`,
// so it reads the **same snapshot** `StepFlow` took and has to run straight after it.
//
// Three directions, not four,: `-hdir`, `+hdir`
// and **up**. There is no downward site — gravity is the flow mover's job. The up site
// carries a gate the horizontal ones do not: the source's snapshot mass must be at least
// its element's `maxMass`, so a cell has to be full before it will shove a different liquid
// upwards. `hdir` is `SimData::iterateDirection`, the same field
// `StepGasPressure` uses, negated once per substep.
//
// The shape of one displacement is a run of three cells in a straight line — source,
// destination, and the cell one further out, called `other` here:
//
//   1. the destination is not `LiquidImpermeable`, holds a **different** element from the
//      source, and that element is a liquid;
//   2. neither end has already been rewritten this sweep: both still hold the element the
//      snapshot recorded;
//   3. `snapshot.mass[other] + snapshot.mass[dst] < available`, where `available` is the
//      source's snapshot mass **minus everything the flow accumulator says has already
//      crossed its four faces this substep**. That is the only place in the sim that reads
//      the flow accumulator back as state rather than writing it, and it is why this sweep
//      has to sit downstream of `StepFlow` rather than beside it;
//   4. `DisplaceLiquidDirectional` gets the destination out of the way — see below — and
//      only if that succeeds does anything move.
//
// What then moves is **an eighth of `available`**, the same 0.125 the gas sweeps use, and
// the destination takes the source's *snapshot* temperature rather than a mix, because it
// was emptied first.
// `rooms`: handed down to `DisplaceGas` below so the gas this helper shoves aside cannot be
// parked in a mixture-owned cell. The two cells this helper itself writes -- `src` and `dst`
// -- are already gated by `StepLiquidDisplacement`'s own three-cell check before it is
// called, so there is no second test for them here.
inline bool DisplaceLiquidDirectional(World* w, const ElementTable& table, uint16_t rotation,
                                      const std::vector<PhaseEntry>* start, size_t src,
                                      size_t dst, const gas::RoomGraph* rooms = nullptr,
                                      const DiseaseTable* diseases = nullptr) {
  std::vector<PhaseEntry>& cells = w->Phases();
  const std::vector<uint8_t>& props = w->Properties();
  // `<= 0`, so a cell another displacement has already emptied is refused.
  if (!(cells[src].mass > 0.0f)) return false;
  if (props[dst] & kLiquidImpermeable) return false;
  const uint16_t ed = cells[dst].element;
  const uint8_t phase = table.Phase(ed);
  if (phase < kStateLiquid) {
    // Gas or vacuum in the way: shove it aside and **swap** the two cells whole. Klei
    // asserts the destination is empty afterwards (`updateliquid.cpp:428`), which is why
    // `DisplaceGas` refusing is fatal to the whole displacement rather than something to
    // work around. A vacuum destination fails here — `DisplaceGas` wants mass and a gas —
    // so this sweep cannot push a liquid into a hole.
    if (!DisplaceGas(w, table, dst, rotation, start, true, rooms, diseases)) return false;
    SwapCells(w, src, dst);
    w->TouchSubstance(src);
    w->TouchSubstance(dst);
    return true;
  }
  if (phase != kStateLiquid) return false;
  // The same liquid on both sides: the destination absorbs the source whole and the source
  // is cleared. Different liquids and the displacement is refused, which is what stops a
  // three-element sandwich from shuffling forever.
  if (cells[src].element != ed) return false;
  // Both ledgers: the destination absorbs the source whole and `ClearCell` then charges the
  // source's whole mass and energy to `cleared` -- but nothing left the grid, it moved one
  // cell. Each ledger's `mover` bucket carries the destination's gain, which is what cancels
  // that. This is the only site in the sim where the mass ledger needs one: every other
  // fluid mover subtracts from its own source instead of clearing it, so both halves stay
  // inside the grid total and neither is charged.
  {
    // The guard deliberately stops short of the `ClearCell` below. The two halves of this
    // transfer belong to different buckets: the destination's gain is a move, and the
    // source's loss is `cleared`, which `ClearCell` charges itself. Measuring across both
    // would net to zero here and leave `cleared`'s side charged twice -- which is exactly
    // what it did on the first attempt, and `sunliquid` said so immediately.
    const MoverCharge mover_charge(w, table, dst, src);
    const float dst_before = cells[dst].mass;
    AddMassAndUpdateTemperature(&cells[dst], cells[src].mass, cells[src].temperature);
    // The disease half of the same call: the source's LIVE germ type and count, all of them,
    // merged into the destination before `ClearCell` empties the source.
    if (diseases != nullptr && w->DiseaseActive()) {
      AddDiseaseToCell(w, *diseases, dst, w->DiseaseIdx(src), w->Disease()[src].count);
    }
    // Layer C: all of the source's liquid went into `dst`, so all of its payload goes too,
    // before the ClearCell below would release it.
    w->PayloadMove(src, dst, 1.0f);
    // The float add rounds, so this is the destination's *measured* gain rather than
    // `cells[src].mass`. `cleared` is charged from the source's own float, and the pair of
    // them differing by a rounding step is the ledger doing its job rather than a leak.
    w->NoteMover(cells[dst].mass - dst_before);
    w->MarkProjectDirty(dst);
  }
  ClearCell(w, table, src);
  // One announcement, and it is the **source**, not the cell that gained the mass — the
  // gas arm above announces both. is reached with Klei's local still holding
  // `param_3`. Transcribed rather than tidied.
  w->TouchSubstance(src);
  return true;
}

// `rooms`: the same liquid promotion gate `StepFlow` takes, and needed here for the same
// reason the gas side gates `StepGasDisplacement` as well as `StepGasPressure` -- this is the
// *other* mechanism that moves liquid between cells, the one that shoves two DIFFERENT
// liquids past each other. Gating only the flow mover would leave a promoted cell open to
// being overwritten by displacement instead of by an ordinary pour.
inline void StepLiquidDisplacement(World* w, const ElementTable& table, int32_t hdir,
                                   uint16_t rotation, size_t ri,
                                   gas::RoomGraph* rooms = nullptr,
                                   const DiseaseTable* diseases = nullptr) {
  // Shares `kFlow`, because the CALL SITE does: this runs inside the same `PROF(kFlow)` block
  // as `StepFlow` (sim/simdll.cpp), so counting it separately would produce a cell count with
  // no time to divide it by. One slot, two sweeps, and `invocations` says so -- it is twice
  // the region count per substep here, not once.
  ONI_SWEEP(kFlow);
  const int32_t pw = w->PaddedWidth(), ph = w->PaddedHeight();
  const float liquid_share = g_tunables.liquid_pressure_displacement_share;  // THE ONE RULE
  const float germ_share = g_tunables.liquid_displacement_germ_share;
  std::vector<PhaseEntry>& cells = w->Phases();
  const std::vector<uint8_t>& props = w->Properties();
  std::vector<PhaseEntry>& start = LiquidSweepStart();
  // `StepFlow` owns the snapshot; without it there is nothing to sweep.
  if (start.size() != cells.size()) return;
  const std::vector<float>& flow = w->FlowAccum();
  // The germ snapshot `StepFlow` took for this region, which Klei's `cells` still is here.
  const std::vector<DiseaseEntry>& dstart = DiseaseSweepStart();
  const bool have_dstart = dstart.size() == w->PaddedCount();
  const bool carry_germs = diseases != nullptr && w->DiseaseActive();

  // `DoLiquidPressureDisplacement`.
  auto displace = [&](size_t src, size_t gsrc, uint16_t ec, size_t dst, size_t other) -> float {
    // Promotion gate, checked first so it is independent of the ordering below: all THREE
    // participating cells matter, exactly as in `StepGasDisplacement`. `dst` is overwritten,
    // `other` receives whatever `dst` held, and `src` is drained. A liquid cell among them
    // that carries mixture slots has them moved aside first (`EvictMixture`), and is
    // refused only when they have nowhere to go. An `other` holding mixture gas over vacuum
    // is refused outright: it reads as vacuum, and `DisplaceLiquidDirectional` refuses a
    // vacuum destination anyway, so moving its gas first would move it for nothing.
    auto clear_of_mixture = [&](size_t c) {
      if (!gas::IsMixtureOwnedCell(*w, rooms, c)) return true;
      if (!gas::HoldsCondensedMass(*w, table, c)) return false;
      return gas::EvictMixture(*w, table, rooms, c, rotation);
    };
    if (!clear_of_mixture(src) || !clear_of_mixture(dst) || !clear_of_mixture(other)) {
      return 0.0f;
    }
    if (props[dst] & kLiquidImpermeable) return 0.0f;
    const uint16_t ed = start[dst].element;
    if (ec == ed) return 0.0f;
    if (cells[src].element != ec) return 0.0f;
    if (cells[dst].element != ed) return 0.0f;
    if (table.Phase(ed) != kStateLiquid) return 0.0f;
    const float* f = &flow[src * 4];
    // Klei's order, and it is a float sum so the order is load-bearing: y, x, z, w.
    const float available = start[src].mass - (f[1] + f[0] + f[2] + f[3]);
    if (!(start[other].mass + start[dst].mass < available)) return 0.0f;
    if (!DisplaceLiquidDirectional(w, table, rotation, &start, dst, other, rooms, diseases)) {
      return 0.0f;
    }
    const float moved = available * liquid_share;
    PhaseEntry& d = cells[dst];
    d.mass += moved;
    d.temperature = start[src].temperature;
    d.element = ec;
    w->TouchSubstance(dst);
    PhaseEntry& s = cells[src];
    // Layer C: `dst`'s own liquid (and payload) went to `other` above; `src` now sends the
    // share of its liquid that moved, and the same share of its payload.
    if (s.mass > 0.0f) w->PayloadMove(src, dst, moved >= s.mass ? 1.0f : moved / s.mass);
    s.mass = s.mass - moved > 0.0f ? s.mass - moved : 0.0f;
    w->MarkProjectDirty(src);
    (void)gsrc;
    // The disease slice, the last thing the function does: an eighth of the source's START germ
    // count, `(int)((float)count * 0.125f)` (the tunable `LiquidDisplacementGermShare`), of its
    // START germ type, merged into `dst`; the source loses the same number with no germ-type
    // test, unlike the mover's.
    if (carry_germs) {
      const uint8_t germ_idx = have_dstart ? dstart[src].idx : w->DiseaseIdx(src);
      const int32_t germ_count = have_dstart ? dstart[src].count : w->Disease()[src].count;
      const int32_t germs = static_cast<int32_t>(static_cast<float>(germ_count) * germ_share);
      AddDiseaseToCell(w, *diseases, dst, germ_idx, germs);
      ModifyCellDiseaseCount(w, src, -germs);
    }
    return moved;
  };

  // The rectangle, inset three cells on every side so the `other` cell has somewhere to be
  // — in **sim** coordinates against `SimData::width` and
  // `::height`, where the gas twin's clamps are in game coordinates. The `min(y0, 3)` is
  // Klei's own copy-paste slip, the same one the gas sweep carries: it plainly means `max`,
  // so the sweep starts near the bottom of the world however high the region begins.
  const World::PaddedRect& r = w->PaddedRegion(ri);
  const int32_t x0 = r.x0 > 3 ? r.x0 : 3;
  int32_t x1 = r.x1 < pw - 3 ? r.x1 : pw - 3;
  const int32_t y0 = r.y0 < 3 ? r.y0 : 3;
  const int32_t y1 = r.y1 < ph - 3 ? r.y1 : ph - 3;
  // The row is also capped at the region's own width, which the clamps above
  // can only have shortened.
  if (x1 - x0 > r.x1 - r.x0) x1 = x0 + (r.x1 - r.x0);
  if (y0 >= y1) return;
  w->MarkProjectDirtyRect(x0, y0, x1, y1);
  const size_t gw = static_cast<size_t>(w->GameWidth());
  for (int32_t y = y0; y < y1; ++y) {
    // Klei's `min(y0, 3)` slip, transcribed above: the same not-bounded-by-its-region shape
    // as the gas twin, and counted for the same reason.
    ONI_EXAMINED(kFlow, x1 - x0);
    for (int32_t x = x0; x < x1; ++x) {
      const size_t c = static_cast<size_t>(y) * pw + x;
      // The source element and the three neighbour tests below read the SNAPSHOT: the
      // call site's element pointer is `SimData::cells`, the copy `UpdateData` takes before
      // `UpdateLiquid`, not `updatedCells`. Reading the live grid made a cell that
      // `StepFlow` had just turned into another liquid a source of that liquid, and it
      // displaced a neighbour Klei leaves alone (`germliq`, tick 20).
      const uint16_t ec = start[c].element;
      if (table.Phase(ec) != kStateLiquid) continue;
      const size_t gsrc = static_cast<size_t>(y - 1) * gw + static_cast<size_t>(x - 1);
      // The flow bookkeeping is transcribed site by site and does not reconcile with
      // itself: both horizontal sites write slot 0 of the source and slot 1 of the cell to
      // the source's **left**, whichever way the displacement went, and the vertical site
      // writes slot 3 of the source and slot 2 of the cell above. Klei's, not ours.
      const size_t left = c - 1;
      const size_t up = c + static_cast<size_t>(pw);
      {
        const size_t dst = static_cast<size_t>(static_cast<ptrdiff_t>(c) - hdir);
        if (start[dst].element != ec) {
          const size_t other = static_cast<size_t>(static_cast<ptrdiff_t>(c) - 2 * hdir);
          const float moved = displace(c, gsrc, ec, dst, other);
          w->AddFlow(c, gsrc, 0, moved);
          w->AddFlow(left, gsrc - 1, 1, -moved);
        }
      }
      {
        const size_t dst = static_cast<size_t>(static_cast<ptrdiff_t>(c) + hdir);
        if (start[dst].element != ec) {
          const size_t other = static_cast<size_t>(static_cast<ptrdiff_t>(c) + 2 * hdir);
          const float moved = displace(c, gsrc, ec, dst, other);
          w->AddFlow(c, gsrc, 0, -moved);
          w->AddFlow(left, gsrc - 1, 1, moved);
        }
      }
      if (start[up].element != ec) {
        // The gate the horizontal sites do not have: the source must be at least as full as
        // its element's `maxMass`, and this one reads the LIVE element and mass
        // (`updatedCells`), unlike the neighbour test just above it.
        if (!(table.At(cells[c].element).maxMass > cells[c].mass)) {
          const size_t other = c + 2 * static_cast<size_t>(pw);
          const float moved = displace(c, gsrc, ec, up, other);
          w->AddFlow(c, gsrc, 3, moved);
          w->AddFlow(up, gsrc + gw, 2, -moved);
        }
      }
    }
  }
}

// A cell emptied by flow would otherwise keep its element and read back as massless water
// at whatever temperature it had. Klei reports vacuum at 0 K — and does it on the first
// *physics* frame, not on load, which is why a freshly seeded vacuum shaft still reads
// 293.15 K for one tick before dropping to zero.
//
// **This runs after `StepPostProcess`, and the order is the rule, not bookkeeping.** There
// is no sweep in the DLL that clears massless cells: `SimData::ClearCell`
// has ten call sites and every one of them is a mover emptying a specific cell or
// `Evaporate` deleting one. So a cell the gas kernels drain keeps its
// element for the rest of the substep, and `PostProcessCell` still sees a **gas** cell
// there. That cell negates the shuffle stride, falls through the 1e-9 wisp gate into
// `Evaporate` without exiting, and then takes its ordinary turn — a `DoDensityDisplacement`
// draw and a shuffle gate draw — before ending up as vacuum anyway.
//
// Zeroing before post-process instead loses those two draws and, worse, loses one negation
// of `dir`, which inverts the stride for every gas cell after it in the
// sweep and turns a swap with the left neighbour into a swap with the right one. In the
// `falling` and `liquid` scenarios that shows as three adjacent whole-cell transpositions
// and two missing draws, with the grid and the random stream otherwise still bit-identical.
//
// Klei spares exactly one massless cell per connected region from the zeroing: a 4x10
// vacuum shaft produced 39 change notifications for its 40 cells, and three disconnected
// pockets produced three survivors, one apiece. That was reproduced, implemented, and then
// **deliberately removed again**, because which cell survives is not stable: the same
// scenario on the same DLL spared the cell on one run and zeroed it on the next. It looks
// like the seam between worker-thread chunks rather than a rule, and modelling a race is
// worse than not modelling it. So every massless cell is zeroed here, deterministically,
// and `diffsim` reports massless-cell temperature separately from the temperature of cells
// that actually hold something.
inline void ZeroMasslessCells(World* w, const ElementTable& table) {
  ONI_SWEEP(kZeroMassless);
  std::vector<PhaseEntry>& cells = w->Phases();
  const uint16_t vacuum = table.VacuumIndex();
  const size_t n = w->PaddedCount();
  // THE WHOLE PADDED GRID, once per substep, regardless of any region. Counted in one go rather than per row
  // because there are no rows here: the loop is flat over the grid, and pretending otherwise
  // to make the counter look like its neighbours would be describing a loop that is not there.
  ONI_EXAMINED(kZeroMassless, static_cast<int64_t>(n));
  // The hint is worth 2.5x and it is the only one in the sim, so it needs its reason written
  // down. This is the ONLY kernel here whose hot
  // path is its *skip*: every other sweep stands on essentially every cell it examines, so
  // GCC's default layout -- fall through into the body, branch away to continue -- is already
  // right for them. Here the body is the rare case, and without the hint GCC compiles this
  // `continue` as a TAKEN forward branch over the twenty-one instructions below it, so the hot
  // path costs two taken branches spanning about a hundred bytes and runs at 1.98 cycles a
  // cell. With it the cold body moves out of line, the loop closes in about twenty-four
  // bytes, and the same seven instructions run at 0.79 -- 0.681 ms to 0.276 at twelve
  // regions, which is 21 % of an idle twelve-region substep for one token.
  //
  // `__builtin_expect` rather than C++20's [[likely]] because this tree is -std=c++17, and
  // it is a hint with no semantics: a compiler that does not have it loses the speed and
  // nothing else. Verified byte-identical across the whole local gate, diffsim included.
  for (size_t i = 0; i < n; ++i) {
    if (__builtin_expect(cells[i].mass > 0.0f, 1)) continue;
    if (table.Phase(cells[i].element) == kStateSolid) continue;
    // A cell can be
    // vanilla-massless (PhaseEntry.mass == 0) while still holding real mass in the new
    // gas-mixture layer (gas_mass_/gas_occupied_mask_), which reuses this same
    // PhaseEntry.temperature field as its own shared temperature (gas_mixture_abi.h).
    // Without this check, this pass would set that temperature to 0 K every substep for
    // such a cell, and MixPair's `avg_t <= 0.0f` guard would then refuse to mix it. `gas_occupied_mask_` is always allocated
    // (World::Allocate assigns it alongside every other per-cell array) and reads 0 for
    // every world that never activates volume-fractions, so this is byte-identical there, and only changes behavior for a cell the
    // mixture actually occupies.
    if (w->GasOccupiedMask(i) != 0) continue;
    // Writes the grid directly and announces nothing, so it counts its own changes -- the
    // same exception `StepConduction` makes, for the same reason.
    ONI_CHANGED(kZeroMassless);
    // Layer C, on the rare branch only: every liquid drain moves or releases its payload as the
    // mass goes, so this should find none. If one ever does, it is released, not leaked.
    w->PayloadRelease(i, 1.0f, cells[i].temperature, ext::kPayloadReleasedMassless);
    cells[i].element = vacuum;
    cells[i].mass = 0.0f;
    cells[i].temperature = 0.0f;
  }
}

}  // namespace oni_sim
