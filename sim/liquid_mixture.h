#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "tunables.h"

// Pure liquid-volume math for a mod's own storage (a tank plumbed to a liquid pipe) -- the
// liquid counterpart to gas_mixture.h's pressure/equalization exports, added for
// Mod 1's liquid tank/pipe mechanic (mirroring the gas compressor/tank work).
//
// NOT modeled the way gas is. As in Stationeers, liquids are not ideal-gas-pressure objects:
// they are tracked by VOLUME, and equalization evens out the volume RATIO (the fraction of a
// container's capacity that is liquid), never a partial pressure. Reusing gas_mixture.h's
// PressureFromMoles/EqualizeSingleSpecies for liquid would apply ideal-gas math to an
// incompressible fluid.
//
// This project has no native per-cell liquid-mixture layer yet (no liquid_mixture_abi.h,
// no SoA storage in World) -- these two functions don't need one. Like gas_mixture.h's
// EqualizeSingleSpecies, they're pure functions over a mod's own container state (mass +
// fixed volume capacity + a caller-supplied density), no simulation state touched at all.
namespace liquid {

// Volume (m^3) that `massKg` of a liquid with density `densityKgM3` (kg/m^3, e.g.
// Element::density -- already real vanilla per-element data, no new table needed) occupies.
// Returns 0 for a non-positive density.
inline float VolumeFromMass(float massKg, float densityKgM3) {
  if (densityKgM3 <= 0.0f) return 0.0f;
  return massKg / densityKgM3;
}

// Generalizes Stationeers-style volume-ratio equalization to two independent containers with
// their own volume CAPACITIES (not a shared fixed cell volume) -- the same shape
// gas_mixture.h's EqualizeSingleSpecies generalizes MixPair to, just built on volume ratio
// instead of partial pressure. A tank plumbed to a liquid pipe with no valve between them is
// one connected system that should equalize toward the same FILL FRACTION, not an active pump
// moving mass at a fixed rate -- the same reasoning as the gas case: directionality belongs in an explicit valve
// building, never baked into a storage tank.
//
// massA/capacityA_m3 and massB/capacityB_m3 describe each side's current holding of ONE
// shared liquid species (densityKgM3) and its own fixed volume capacity. `rate` damps the
// step the same way EqualizeSingleSpecies's own `rate` does -- a fixed extra *0.5 on top of
// it, converging geometrically over repeated calls rather than snapping to equal fill in one
// step, so a caller polling at the same cadence Mod 1's gas tank already uses gets the same
// multi-second convergence feel, not a sudden jump.
//
// Derivation: ratioA = (massA/density)/capacityA, ratioB = (massB/density)/capacityB. Moving
// `dm` mass from B into A changes ratioA by +dm/(density*capacityA) and ratioB by
// -dm/(density*capacityB), so closing the gap `dr` fully takes
// `dm_full = dr / ((1/capacityA + 1/capacityB) / density)` -- the volume-ratio analogue of
// EqualizeSingleSpecies's own `dn_full = dp / (R*(T_a/V_a + T_b/V_b))`.
//
// Returns the mass that should move INTO side A (from B) this step -- positive means A gains,
// negative means A loses to B, 0.0f if already equal, capacities are non-positive, the density
// is non-positive, or the computed transfer is below a fixed minimum (same
// MinTransferMass=1e-6f floor as EqualizeSingleSpecies, for the same reason: continuous float
// math never hits exact zero, so an unfloored transfer never lets either side settle/sleep).
inline float EqualizeLiquidVolume(float densityKgM3, float massA, float capacityA_m3,
                                   float massB, float capacityB_m3, float rate) {
  if (densityKgM3 <= 0.0f || capacityA_m3 <= 0.0f || capacityB_m3 <= 0.0f) return 0.0f;
  const float ratioA = VolumeFromMass(massA, densityKgM3) / capacityA_m3;
  const float ratioB = VolumeFromMass(massB, densityKgM3) / capacityB_m3;
  const float dr = ratioA - ratioB;
  if (dr == 0.0f) return 0.0f;

  const float denom = (1.0f / capacityA_m3 + 1.0f / capacityB_m3) / densityKgM3;
  if (denom <= 0.0f) return 0.0f;
  const float dm_full = std::fabs(dr) / denom;
  // The tunables `LiquidEqualizeDamping` and `MinTransferMass` (the latter shared with the gas
  // layer). An export's helper, one call per query, so it reads the live table directly.
  float dmass = dm_full * rate * oni_sim::g_tunables.liquid_equalize_damping;
  const float min_transfer_mass = oni_sim::g_tunables.min_transfer_mass;

  if (dr > 0.0f) {
    // A is the fuller side -- mass flows OUT of A, into B.
    dmass = std::min(dmass, massA);
    if (dmass <= min_transfer_mass) return 0.0f;
    return -dmass;
  }
  // B is the fuller side -- mass flows INTO A.
  dmass = std::min(dmass, massB);
  if (dmass <= min_transfer_mass) return 0.0f;
  return dmass;
}

}  // namespace liquid
