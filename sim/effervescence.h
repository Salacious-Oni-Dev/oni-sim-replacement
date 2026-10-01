// ================================================================================================
// EFFERVESCENCE -- supersaturated liquid gives its excess gas back as bubbles.
//
// The contract, the criterion and every default are at `ext::kSetEffervescence` in
// abi/sim_abi_ext.h.
//
// WHY. Gas enters water when a bubble dissolves (`DissolvedGas.Add`), and without this pass it
// could leave only if something carried the water away or a rising bubble stripped it. Water
// that became supersaturated -- warmed, or had the pressure on it dropped -- would simply stay
// that way, and a depressurised carbonated pond would never fizz.
//
// WHAT IT MODELS. Heterogeneous nucleation, as a rate: once the total gas tension of a cell
// passes `1 + margin` times the pressure on it, a fixed fraction per second of the excess comes
// out, every dissolved gas in proportion. Not bubble growth, not nucleation-site density, not
// agitation. The bubbles themselves are the managed side's (Layer A), and they carry on the
// physics from there: they rise, and on the way they strip more gas from any supersaturated
// water they cross (C4), which is why a fizzing column accelerates.
//
// WHAT IT CANNOT DISTURB. Nothing vanilla. It reads `PhaseEntry` and writes only the
// `sim.dissolved_mass` lanes and the released-record list, and it returns on its first line
// unless `kSetEffervescence` enabled it AND something has been dissolved somewhere -- so every
// diffsim golden, and every world that never sent the message, is byte-identical.
//
// WHY A COLUMN WALK, ONCE PER SUBSTEP AND OUTSIDE THE REGION LOOP. The criterion needs the
// pressure at each cell, and that is the whole column above it. Walking every column top-down
// and carrying the column's mass makes each cell O(1); running it inside the region loop would
// read columns that cross into regions not yet stepped. The walk visits the whole padded grid,
// so it is also throttled: it only runs once `periodSeconds` of sim time has gone by.
// ================================================================================================
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "census.h"
#include "gas_mixture.h"
#include "physics.h"
#include "tunables.h"
#include "world.h"

namespace oni_sim {

// The constants the managed side's `Bubbles` and `Solubility` use, spelled identically so a
// cell the two sides look at gets the same answer. R, g and the cell volume are the sim's own
// tunables `GasConstantR`, `StandardGravity` and `CellVolumeM3`, so they cannot drift apart;
// the rest are tunables of their own (tunables.def), each named after its managed mirror:
//   FizzReferenceK                  Solubility.ReferenceTemperatureK
//   FizzMinimumGasVolumeM3          Bubbles.MinimumGasVolumeM3
//   FizzSurfaceColumnCells          Bubbles.SurfaceColumnCells
//   FizzFallbackSolventDensityKgM3  the density a solvent with no registered liquid density
//                                   and no default mass is read at
// Every helper below takes the kernel's hoisted copy (tunables.h, THE ONE RULE); a direct
// caller outside the kernel lets it default to the live table.

// A liquid's pure density, kg/m3, resolved exactly as `Bubbles.LiquidDensityKgPerM3` resolves it:
// the configured solvent's density, else `sim.liquid_density`, else the element's default cell
// mass, else water's 1000.
inline float FizzLiquidDensity(const World::Effervescence& cfg, const ElementTable& table,
                               uint16_t element, const Tunables& tun = g_tunables) {
  for (int32_t s = 0; s < cfg.solvent_count; ++s) {
    if (cfg.solvent_element[s] == element && cfg.solvent_density[s] > 0.0f) {
      return cfg.solvent_density[s];
    }
  }
  const float registered = table.LiquidDensityOf(element);
  if (registered > 0.0f) return registered;
  const float default_mass = table.At(element).defaultValues.mass;
  return default_mass > 0.0f ? default_mass : tun.fizz_fallback_solvent_density_kg_m3;
}

// The dissolved lanes of `cell`, summed, and the volume they add to its solution.
inline void FizzDissolved(const World::Effervescence& cfg, const float* lanes, float* mass_kg,
                          float* volume_m3) {
  float m = 0.0f;
  float v = 0.0f;
  for (int32_t i = 0; i < ext::kDissolvedGasLanes; ++i) {
    const float kg = lanes[i];
    if (!(kg > 0.0f)) continue;
    m += kg;
    if (cfg.lane_molar_kg[i] > 0.0f && cfg.lane_pmv_m3[i] > 0.0f) {
      v += kg / cfg.lane_molar_kg[i] * cfg.lane_pmv_m3[i];
    }
  }
  *mass_kg = m;
  *volume_m3 = v;
}

// How far up a column the surface pressure pools, in cells (`FizzSurfacePressure`). Mirrored as
// `Bubbles.SurfaceColumnCells`. The tunable `FizzSurfaceColumnCells` (tunables.def).

// A pooled gas reading: moles-times-kelvin and volume summed over some cells, and the same sum
// per dissolved-gas lane (the moles of that lane's gas alone), for partial pressures.
struct FizzPool {
  double moles_k = 0.0;
  double volume_m3 = 0.0;
  double lane_moles_k[ext::kDissolvedGasLanes] = {};
};

// Adds one non-liquid, non-solid cell to `pool`. False -- and nothing added -- for a solid, a
// liquid, or a cell outside the grid: the cells a pooled reading stops at. Vacuum is open, adding
// volume and no moles.
//
// THE GAS IS VANILLA'S CELL AND THE MIXTURE LAYER'S, BOTH. A promoted room keeps its gas in the
// mixture slots and leaves the cell's `PhaseEntry` at vacuum (`ClearCell`, sim/physics.h), so
// reading the element alone would put every promoted room at 0 Pa -- a vacuum surface, which
// degasses hard. The two stores are disjoint, so summing them counts nothing twice; the mixture
// shares the cell's one temperature (abi/gas_mixture_abi.h). 0.29 read the element alone.
//
// The volume is the cell's own tile plus whatever the liquid cell below it leaves empty
// (`Bubbles.GasVolumeM3`); that term goes negative when that cell's solution has swelled past a
// tile, which squeezes the gas.
inline bool FizzAddCell(const World& w, const World::Effervescence& cfg,
                        const ElementTable& table, size_t cell, const float* dissolved,
                        FizzPool* pool, const Tunables& tun = g_tunables) {
  if (cell >= w.PaddedCount()) return false;
  const PhaseEntry& g = w.Phase(cell);
  const uint8_t phase = table.Phase(g.element);
  if (phase != kStateGas && phase != kStateVacuum) return false;
  const double t = g.temperature > 1.0f ? g.temperature : 1.0f;
  auto add_gas = [&](uint16_t element, float mass) {
    if (!(mass > 0.0f)) return;
    const float molar_kg = table.MolecularMassOf(element) * 0.001f;
    if (!(molar_kg > 0.0f)) return;
    const double n_t = static_cast<double>(mass) / molar_kg * t;
    pool->moles_k += n_t;
    for (int32_t i = 0; i < ext::kDissolvedGasLanes; ++i) {
      if (cfg.lane_element[i] == element) pool->lane_moles_k[i] += n_t;
    }
  };
  if (phase == kStateGas) add_gas(g.element, g.mass);
  const uint8_t mask = w.GasOccupiedMask(cell);
  for (int s = 0; s < gas::kMaxSpeciesPerCell; ++s) {
    if (mask & (1u << s)) add_gas(w.GasSpecies(cell, s), w.GasMass(cell, s));
  }
  double volume = tun.cell_volume_m3;
  const size_t pw = static_cast<size_t>(w.PaddedWidth());
  if (cell >= pw) {
    const size_t below = cell - pw;
    const PhaseEntry& l = w.Phase(below);
    if (table.Phase(l.element) == kStateLiquid && l.mass > 0.0f) {
      const float rho = FizzLiquidDensity(cfg, table, l.element, tun);
      float lanes_kg = 0.0f;
      float lanes_m3 = 0.0f;
      FizzDissolved(cfg, dissolved + below * static_cast<size_t>(ext::kDissolvedGasLanes),
                    &lanes_kg, &lanes_m3);
      const float solution_m3 = (rho > 0.0f ? l.mass / rho : 0.0f) + lanes_m3;
      volume += tun.cell_volume_m3 - solution_m3;
    }
  }
  pool->volume_m3 += volume;
  return true;
}

// Ideal-gas pressure of pooled moles-kelvin over a pooled volume, Pa, the volume floored as
// `Bubbles.MinimumGasVolumeM3` floors it. Double throughout, so one cell pooled alone and four
// identical cells pooled together read the same pressure to the bit.
inline float FizzPooledPressure(double moles_k, double volume_m3,
                                const Tunables& tun = g_tunables) {
  if (!(moles_k > 0.0)) return 0.0f;
  const double v = volume_m3 > tun.fizz_minimum_gas_volume_m3 ? volume_m3
                                                              : tun.fizz_minimum_gas_volume_m3;
  return static_cast<float>(moles_k * tun.gas_constant_r / v);
}

// The gas pressure of `gas_cell` alone, Pa. Zero for anything that holds no gas.
inline float FizzGasPressure(const World& w, const World::Effervescence& cfg,
                             const ElementTable& table, size_t gas_cell, const float* dissolved,
                             const Tunables& tun = g_tunables) {
  FizzPool pool;
  if (!FizzAddCell(w, cfg, table, gas_cell, dissolved, &pool, tun)) return 0.0f;
  return FizzPooledPressure(pool.moles_k, pool.volume_m3, tun);
}

// THE SURFACE PRESSURE IS THE GAS COLUMN'S, NOT ONE CELL'S. See `ext::kSetEffervescence`, "THE
// SURFACE READS THE GAS ABOVE IT, POOLED". From the cap at (x, cap_y) upward, up to
// `FizzSurfaceColumnCells` cells, stopping at the first solid or liquid or the border ring:
// every cell's moles and volume summed, and one ideal-gas pressure read off the sums -- the way
// Stationeers reads a room's pressure, on a bounded column instead of a room.
inline void FizzSurfacePool(const World& w, const World::Effervescence& cfg,
                            const ElementTable& table, int32_t x, int32_t cap_y,
                            const float* dissolved, FizzPool* pool,
                            const Tunables& tun = g_tunables) {
  const size_t pw = static_cast<size_t>(w.PaddedWidth());
  // The top padded row is the border ring, not a game cell, and its empty volume is no room's.
  const int32_t top = w.PaddedHeight() - 1;
  for (int32_t k = 0; k < tun.fizz_surface_column_cells; ++k) {
    const int32_t y = cap_y + k;
    if (y >= top) break;
    const size_t c = static_cast<size_t>(y) * pw + static_cast<size_t>(x);
    if (!FizzAddCell(w, cfg, table, c, dissolved, pool, tun)) break;
  }
}

inline float FizzSurfacePressure(const World& w, const World::Effervescence& cfg,
                                 const ElementTable& table, int32_t x, int32_t cap_y,
                                 const float* dissolved, const Tunables& tun = g_tunables) {
  FizzPool pool;
  FizzSurfacePool(w, cfg, table, x, cap_y, dissolved, &pool, tun);
  return FizzPooledPressure(pool.moles_k, pool.volume_m3, tun);
}

// What caps a liquid column, and the surface pressure that follows. See `ext::kSetEffervescence`,
// "A CAP THAT HOLDS NOTHING IS WEIGHED BY ITS NEIGHBOURS" and "THE SURFACE READS THE GAS ABOVE IT,
// POOLED". `read_from` is the cap cell for a gas cap, the neighbour whose reading won for a
// borrowed one.
enum FizzCap { kFizzCapGas, kFizzCapBorrowed, kFizzCapVacuum, kFizzCapConfined };

inline FizzCap FizzClassifyCap(const World& w, const World::Effervescence& cfg,
                               const ElementTable& table, int32_t x, int32_t cap_y,
                               const float* dissolved, float* surface_pa, size_t* read_from,
                               FizzPool* pool, const Tunables& tun = g_tunables) {
  *surface_pa = 0.0f;
  *read_from = static_cast<size_t>(-1);
  *pool = FizzPool{};
  const int32_t pw = w.PaddedWidth();
  const int32_t ph = w.PaddedHeight();
  if (cap_y >= ph) return kFizzCapConfined;
  const size_t cap = static_cast<size_t>(cap_y) * static_cast<size_t>(pw) +
                     static_cast<size_t>(x);
  if (table.Phase(w.Phase(cap).element) == kStateSolid) return kFizzCapConfined;
  FizzSurfacePool(w, cfg, table, x, cap_y, dissolved, pool, tun);
  const float column = FizzPooledPressure(pool->moles_k, pool->volume_m3, tun);
  if (FizzGasPressure(w, cfg, table, cap, dissolved, tun) > 0.0f) {
    *surface_pa = column;
    *read_from = cap;
    return kFizzCapGas;
  }
  // Holds no gas. The pooled column above it, or the cell left or right, highest wins: the gap
  // refills from whichever side pushes hardest, and erring high errs towards not fizzing.
  float best = column;
  size_t best_cell = column > 0.0f ? cap + static_cast<size_t>(pw) : static_cast<size_t>(-1);
  auto consider = [&](size_t n) {
    const float p = FizzGasPressure(w, cfg, table, n, dissolved, tun);
    if (p > best) {
      best = p;
      best_cell = n;
    }
  };
  if (x > 0) consider(cap - 1);
  if (x + 1 < pw) consider(cap + 1);
  if (best > 0.0f) {
    *surface_pa = best;
    *read_from = best_cell;
    return kFizzCapBorrowed;
  }
  return kFizzCapVacuum;
}

// One cell, at pressure `pressure_pa`. Releases if it is past the threshold; returns the kg
// released.
inline float FizzCell(World* w, const World::Effervescence& cfg, const ElementTable& table,
                      size_t c, float pressure_pa, float kept_fraction, float* tension_out,
                      float* volume_out, const Tunables& tun = g_tunables) {
  const PhaseEntry& e = w->Phase(c);
  int32_t solvent = -1;
  for (int32_t s = 0; s < cfg.solvent_count; ++s) {
    if (cfg.solvent_element[s] == e.element) {
      solvent = s;
      break;
    }
  }
  if (solvent < 0 || !(cfg.solvent_factor[solvent] > 0.0f)) return 0.0f;
  if (!(pressure_pa > 0.0f) || !(e.temperature > 0.0f)) return 0.0f;
  float* lanes = w->DissolvedMassData() + c * static_cast<size_t>(ext::kDissolvedGasLanes);
  float lanes_kg = 0.0f;
  float lanes_m3 = 0.0f;
  FizzDissolved(cfg, lanes, &lanes_kg, &lanes_m3);
  const float rho = FizzLiquidDensity(cfg, table, e.element, tun);
  const float volume_m3 = (rho > 0.0f ? e.mass / rho : 0.0f) + lanes_m3;
  if (!(volume_m3 > 0.0f)) return 0.0f;

  // S = sum_i kg_i / (H_i(T) * P * M_i * V), over the lanes this message describes.
  const float inv_t = 1.0f / e.temperature - 1.0f / tun.fizz_reference_k;
  float tension = 0.0f;
  for (int32_t i = 0; i < ext::kDissolvedGasLanes; ++i) {
    const float kg = lanes[i];
    if (!(kg > 0.0f) || !(cfg.lane_henry[i] > 0.0f) || !(cfg.lane_molar_kg[i] > 0.0f)) continue;
    float h = cfg.lane_henry[i] * cfg.solvent_factor[solvent];
    if (cfg.lane_vant_hoff_k[i] != 0.0f) h *= std::exp(cfg.lane_vant_hoff_k[i] * inv_t);
    const float capacity_kg = h * pressure_pa * cfg.lane_molar_kg[i] * volume_m3;
    if (!(capacity_kg > 0.0f)) continue;
    tension += kg / capacity_kg;
  }
  *tension_out = tension;
  *volume_out = volume_m3;
  if (!(tension > 1.0f + cfg.margin)) return 0.0f;

  // Every described lane scaled towards S = 1, by the share the elapsed period lets out.
  const float fraction = (1.0f - 1.0f / tension) * (1.0f - kept_fraction);
  if (!(fraction > 0.0f)) return 0.0f;
  float total = 0.0f;
  for (int32_t i = 0; i < ext::kDissolvedGasLanes; ++i) {
    if (lanes[i] > 0.0f && cfg.lane_henry[i] > 0.0f && cfg.lane_molar_kg[i] > 0.0f) {
      total += lanes[i] * fraction;
    }
  }
  if (!(total >= cfg.min_release_kg)) return 0.0f;
  float released = 0.0f;
  for (int32_t i = 0; i < ext::kDissolvedGasLanes; ++i) {
    if (lanes[i] > 0.0f && cfg.lane_henry[i] > 0.0f && cfg.lane_molar_kg[i] > 0.0f) {
      released += w->DissolvedRelease(c, i, lanes[i] * fraction, e.temperature,
                                      ext::kPayloadReleasedEffervescence);
    }
  }
  return released;
}

// Gas of `element` a surface may draw from `cap` in one pass: at most half of what the cell holds
// of it, and never so much that a vanilla gas cell falls under `FizzUptakeKeepKg` -- the sim's
// own 1 g wisp floor, doubled, so an uptake can never be what turns a thin cell into a wisp.
// Both are tunables, `FizzUptakeKeepKg` (0.002 kg) and `FizzUptakeMaxShare` (0.5).
inline void FizzCapStores(const World& w, size_t cap, uint16_t element, float* facade_kg,
                          float* mixture_kg, const Tunables& tun = g_tunables) {
  *facade_kg = 0.0f;
  *mixture_kg = 0.0f;
  const PhaseEntry& g = w.Phase(cap);
  if (g.element == element && g.mass > tun.fizz_uptake_keep_kg) {
    const float share = g.mass * tun.fizz_uptake_max_share;
    const float above_floor = g.mass - tun.fizz_uptake_keep_kg;
    *facade_kg = share < above_floor ? share : above_floor;
  }
  const int slot = gas::FindSlot(w, cap, element);
  if (slot >= 0) {
    const float m = w.GasMass(cap, slot);
    if (m > 0.0f) *mixture_kg = m * tun.fizz_uptake_max_share;
  }
}

// What one surface did in one pass.
struct FizzSurfaceResult {
  float released_kg = 0.0f;   // out of the liquid, on the stream as kPayloadReleasedSurface
  float absorbed_kg = 0.0f;   // into the liquid, out of the cap cell
  double heat_kj = 0.0;       // what the absorbed gas carried above the liquid's temperature
};

// THE SURFACE EXCHANGE, at the top liquid cell `c` of a column whose cap cell `cap` holds gas,
// `pool` being that cap's pooled column. See `ext::kEffervescenceDefaultSurfaceRatePerSecond`'s
// comment. For each described lane with a known element, against `c_eq = H(T) p_i M V` with p_i
// the PARTIAL pressure of its gas in the pooled column:
//
//   * holding more than c_eq, it releases `(kg - c_eq) * (1 - kept)`;
//   * holding less, it ABSORBS `(c_eq - kg) * (1 - kept)`, drawn out of the CAP
//     CELL alone -- vanilla gas first, then the mixture layer -- and limited to what
//     `FizzCapStores` lets it take. The pooled column sets how hard the gas pushes in; only the
//     cell touching the surface gives it up, and gas flow refills that cell from above.
//
// The two directions share one threshold: a surface whose releases, or whose absorptions,
// total less than `surface_min_release_kg` over all lanes does nothing in that direction.
//
// HEAT. A dissolved lane carries no heat of its own, so dissolved gas is at its liquid's
// temperature. The absorbed gas leaves the cap at the cap's temperature, so what it carried
// above the liquid's, `m c_gas (T_cap - T_liquid)`, is paid into the liquid cell -- the rule
// `Bubbles.Exchange` follows for a dissolving bubble. Both ledgers charge the
// grid's own change at `dissolved_surface`.
inline FizzSurfaceResult FizzSurface(World* w, const World::Effervescence& cfg,
                                     const ElementTable& table, size_t c, size_t cap,
                                     const FizzPool& pool, float kept_fraction,
                                     const Tunables& tun = g_tunables) {
  FizzSurfaceResult result;
  PhaseEntry& e = w->Phase(c);
  int32_t solvent = -1;
  for (int32_t s = 0; s < cfg.solvent_count; ++s) {
    if (cfg.solvent_element[s] == e.element) {
      solvent = s;
      break;
    }
  }
  if (solvent < 0 || !(cfg.solvent_factor[solvent] > 0.0f) || !(e.temperature > 0.0f)) {
    return result;
  }
  float* lanes = w->DissolvedMassData();
  if (lanes == nullptr) return result;
  lanes += c * static_cast<size_t>(ext::kDissolvedGasLanes);
  float lanes_kg = 0.0f;
  float lanes_m3 = 0.0f;
  FizzDissolved(cfg, lanes, &lanes_kg, &lanes_m3);
  const float rho = FizzLiquidDensity(cfg, table, e.element, tun);
  const float volume_m3 = (rho > 0.0f ? e.mass / rho : 0.0f) + lanes_m3;
  if (!(volume_m3 > 0.0f)) return result;
  const double gas_v =
      pool.volume_m3 > tun.fizz_minimum_gas_volume_m3 ? pool.volume_m3
                                                      : tun.fizz_minimum_gas_volume_m3;
  const float inv_t = 1.0f / e.temperature - 1.0f / tun.fizz_reference_k;
  const float leave = 1.0f - kept_fraction;
  float out[ext::kDissolvedGasLanes] = {};
  float in_facade[ext::kDissolvedGasLanes] = {};
  float in_mixture[ext::kDissolvedGasLanes] = {};
  float total_out = 0.0f;
  float total_in = 0.0f;
  for (int32_t i = 0; i < ext::kDissolvedGasLanes; ++i) {
    const float kg = lanes[i];
    const uint16_t element = cfg.lane_element[i];
    if (!(cfg.lane_henry[i] > 0.0f) || !(cfg.lane_molar_kg[i] > 0.0f) || element == 0xFFFF) {
      continue;
    }
    float h = cfg.lane_henry[i] * cfg.solvent_factor[solvent];
    if (cfg.lane_vant_hoff_k[i] != 0.0f) h *= std::exp(cfg.lane_vant_hoff_k[i] * inv_t);
    const float partial_pa =
        static_cast<float>(pool.lane_moles_k[i] * tun.gas_constant_r / gas_v);
    const float capacity_kg = h * partial_pa * cfg.lane_molar_kg[i] * volume_m3;
    const float excess = kg - capacity_kg;
    if (excess > 0.0f) {
      out[i] = excess * leave;
      total_out += out[i];
    } else if (excess < 0.0f) {
      float want = -excess * leave;
      float facade_kg = 0.0f;
      float mixture_kg = 0.0f;
      FizzCapStores(*w, cap, element, &facade_kg, &mixture_kg, tun);
      in_facade[i] = want < facade_kg ? want : facade_kg;
      want -= in_facade[i];
      in_mixture[i] = want < mixture_kg ? want : mixture_kg;
      total_in += in_facade[i] + in_mixture[i];
    }
  }
  if (total_out > 0.0f && total_out >= cfg.surface_min_release_kg) {
    for (int32_t i = 0; i < ext::kDissolvedGasLanes; ++i) {
      if (out[i] > 0.0f) {
        result.released_kg += w->DissolvedRelease(c, i, out[i], e.temperature,
                                                  ext::kPayloadReleasedSurface);
      }
    }
  }
  if (!(total_in > 0.0f) || total_in < cfg.surface_min_release_kg) return result;

  // Take the gas out of the cap. The cap keeps its element and its temperature; only mass moves,
  // exactly as a consumer takes it (emitters.h `RemoveMassFromCell`).
  PhaseEntry& g = w->Phase(cap);
  const float cap_k = g.temperature;
  double heat_kj = 0.0;
  double facade_kg_total = 0.0;
  double facade_kj_total = 0.0;
  for (int32_t i = 0; i < ext::kDissolvedGasLanes; ++i) {
    const uint16_t element = cfg.lane_element[i];
    float taken = 0.0f;
    if (in_facade[i] > 0.0f && g.element == element) {
      g.mass -= in_facade[i];
      taken += in_facade[i];
      facade_kg_total += in_facade[i];
      facade_kj_total += static_cast<double>(in_facade[i]) *
                         static_cast<double>(table.SpecificHeat(element)) *
                         static_cast<double>(cap_k);
    }
    if (in_mixture[i] > 0.0f) {
      taken += gas::RemoveSpecies(*w, cap, element, in_mixture[i]);
    }
    if (!(taken > 0.0f)) continue;
    result.absorbed_kg +=
        w->DissolvedAbsorb(c, i, taken, cap_k, ext::kPayloadAbsorbedSurface);
    heat_kj += static_cast<double>(taken) * static_cast<double>(table.SpecificHeat(element)) *
               (static_cast<double>(cap_k) - static_cast<double>(e.temperature));
  }
  if (!(result.absorbed_kg > 0.0f)) return result;
  w->MarkProjectDirty(cap);

  // The heat the gas brought above the liquid's temperature, into the liquid.
  const double liquid_c = static_cast<double>(table.SpecificHeat(e.element));
  const double capacity = static_cast<double>(e.mass) * liquid_c;
  const double liquid_before = capacity * static_cast<double>(e.temperature);
  if (capacity > 0.0 && heat_kj != 0.0) {
    double next = static_cast<double>(e.temperature) + heat_kj / capacity;
    if (next < 1.0) next = 1.0;
    if (next > 10000.0) next = 10000.0;
    e.temperature = static_cast<float>(next);
    w->MarkProjectDirty(c);
  }
  const double liquid_after = capacity * static_cast<double>(e.temperature);
  result.heat_kj = heat_kj;
  w->NoteDissolvedSurface(-facade_kg_total);
  w->NoteDissolvedSurfaceEnergy((liquid_after - liquid_before) - facade_kj_total);
  return result;
}

// The pass. `dt` is the substep's length in sim seconds; the walk itself runs once per
// `period_seconds` of accumulated time and releases by the whole elapsed interval.
inline void StepEffervescence(World* w, const ElementTable& table, float dt) {
  World::Effervescence& cfg = w->MutableEffervescenceConfig();
  const Tunables tun = g_tunables;  // tunables.h, THE ONE RULE
  // Surface uptake can put gas into water that holds none, so with the surface exchange
  // on, "nothing dissolved anywhere" no longer means there is nothing to do.
  if (!cfg.enabled || w->DissolvedMassData() == nullptr) return;
  if (!w->DissolvedMaybeNonzero() && !(cfg.surface_rate_per_second > 0.0f)) return;
  cfg.elapsed_seconds += dt;
  const float period = cfg.period_seconds > 0.0f ? cfg.period_seconds
                                                 : ext::kEffervescenceDefaultPeriodSeconds;
  if (cfg.elapsed_seconds < period) return;
  const float elapsed = cfg.elapsed_seconds;
  cfg.elapsed_seconds = 0.0f;
  const bool surfacing = cfg.surface_rate_per_second > 0.0f;
  if (!(cfg.rate_per_second > 0.0f) && !surfacing) return;
  // The share of the excess that STAYS dissolved over the interval; what leaves is 1 minus it.
  const float kept = std::exp(-cfg.rate_per_second * elapsed);
  const float kept_surface = std::exp(-cfg.surface_rate_per_second * elapsed);

  ONI_SWEEP(kEffervescence);
  cfg.census[ext::kEffervescenceCensusPasses] += 1.0;
  const int32_t pw = w->PaddedWidth();
  const int32_t ph = w->PaddedHeight();
  const float* dissolved = w->DissolvedMassData();
  for (int32_t x = 0; x < pw; ++x) {
    ONI_EXAMINED(kEffervescence, ph);
    bool in_run = false;
    bool confined = false;
    FizzCap cap_kind = kFizzCapVacuum;
    size_t read_from = static_cast<size_t>(-1);
    float surface_pa = 0.0f;
    FizzPool pool;
    float column_kg = 0.0f;
    // Top down: +width is the cell above, so the highest padded row is the top of the world.
    for (int32_t y = ph - 1; y >= 0; --y) {
      const size_t c = static_cast<size_t>(y) * static_cast<size_t>(pw) + static_cast<size_t>(x);
      const PhaseEntry& e = w->Phase(c);
      if (table.Phase(e.element) != kStateLiquid || !(e.mass > 0.0f)) {
        in_run = false;
        continue;
      }
      if (!in_run) {
        in_run = true;
        column_kg = 0.0f;
        // A COLUMN CAPPED BY A SOLID IS CONFINED, and confined liquid does not fizz. A full,
        // rigid, sealed vessel has no volume for a bubble to grow into, so its liquid's pressure
        // rises to whatever keeps the gas in -- a sealed bottle stays flat until it is opened. The
        // column walk cannot see that pressure, and reading the column alone would give the top
        // cell of such a vessel a few kilopascals and fizz it violently. The same rule errs the
        // same way under an overhang, where water touching the rock belongs to a pond whose free
        // surface is somewhere else: it keeps the gas in rather than fizz it at a wrong pressure.
        // Any other cap is read as the gas column above it, pooled, so a thin pocket the falling
        // water left reads the headspace it belongs to. A cap of VACUUM is not confinement --
        // liquid under vacuum degasses hard -- but a cap holding no gas is usually a gap the
        // liquid kernel left for a substep in a pond under a full atmosphere, so it is weighed
        // by its gas neighbours first and only read as 0 Pa when it has none
        // (`FizzClassifyCap`).
        cap_kind = FizzClassifyCap(*w, cfg, table, x, y + 1, dissolved, &surface_pa, &read_from,
                                   &pool, tun);
        confined = cap_kind == kFizzCapConfined;
        cfg.census[ext::kEffervescenceCensusColumnsGas + static_cast<int>(cap_kind)] += 1.0;
        // The free surface, if gas rests on it, trades with that gas first -- before this cell's
        // fizz test, which then sees what is left.
        if (surfacing && cap_kind == kFizzCapGas) {
          cfg.census[ext::kEffervescenceCensusSurfaces] += 1.0;
          const FizzSurfaceResult r =
              FizzSurface(w, cfg, table, c, read_from, pool, kept_surface, tun);
          if (r.released_kg > 0.0f) {
            ONI_CHANGED(kEffervescence);
            cfg.census[ext::kEffervescenceCensusSurfaceReleases] += 1.0;
            cfg.census[ext::kEffervescenceCensusSurfaceKg] += r.released_kg;
          }
          if (r.absorbed_kg > 0.0f) {
            ONI_CHANGED(kEffervescence);
            cfg.census[ext::kEffervescenceCensusSurfaceUptakes] += 1.0;
            cfg.census[ext::kEffervescenceCensusSurfaceUptakeKg] += r.absorbed_kg;
            cfg.census[ext::kEffervescenceCensusSurfaceUptakeHeatKJ] += r.heat_kj;
          }
        }
      }
      float lanes_kg = 0.0f;
      float lanes_m3 = 0.0f;
      FizzDissolved(cfg, dissolved + c * static_cast<size_t>(ext::kDissolvedGasLanes),
                    &lanes_kg, &lanes_m3);
      const float solution_kg = e.mass + lanes_kg;
      // At the cell's centre: the column above it plus half of itself, as
      // `Bubbles.PressureAtPa` weighs a position half-way up a cell.
      if (lanes_kg > 0.0f && !confined) {
        const float pressure = surface_pa + (column_kg + 0.5f * solution_kg) * tun.standard_gravity;
        float tension = 0.0f;
        float volume = 0.0f;
        const float released =
            FizzCell(w, cfg, table, c, pressure, kept, &tension, &volume, tun);
        if (released > 0.0f) {
          ONI_CHANGED(kEffervescence);
          const int k = static_cast<int>(cap_kind);  // never confined here
          double* n = cfg.census;
          n[ext::kEffervescenceCensusFizzGas + k] += 1.0;
          n[ext::kEffervescenceCensusKgGas + k] += released;
          if (tension > n[ext::kEffervescenceCensusWorstS]) {
            const bool has_cap = read_from != static_cast<size_t>(-1);
            const PhaseEntry& cap = w->Phase(has_cap ? read_from : c);
            n[ext::kEffervescenceCensusWorstS] = tension;
            n[ext::kEffervescenceCensusWorstCell] = static_cast<double>(w->GameIndex(c));
            n[ext::kEffervescenceCensusWorstPressurePa] = pressure;
            n[ext::kEffervescenceCensusWorstSurfacePa] = surface_pa;
            n[ext::kEffervescenceCensusWorstTemperatureK] = e.temperature;
            n[ext::kEffervescenceCensusWorstVolumeM3] = volume;
            n[ext::kEffervescenceCensusWorstDissolvedKg] = lanes_kg;
            n[ext::kEffervescenceCensusWorstCapCell] =
                has_cap ? static_cast<double>(w->GameIndex(read_from)) : -1.0;
            n[ext::kEffervescenceCensusWorstCapElement] = has_cap ? cap.element : -1.0;
            n[ext::kEffervescenceCensusWorstCapMassKg] = has_cap ? cap.mass : 0.0;
            n[ext::kEffervescenceCensusWorstCapTemperatureK] = has_cap ? cap.temperature : 0.0;
          }
        }
      }
      column_kg += solution_kg;
    }
  }
}

}  // namespace oni_sim
