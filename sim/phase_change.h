#pragma once

#include <algorithm>
#include <cmath>

// Latent-heat phase change for a mod's own container, such as a compressor tank holding a
// gas/liquid blend of one species.
//
// The game's own cell transition (`TransitionCell` in sim/physics.h) carries temperature across
// a phase change and charges a flat 1.5 K overshoot; neither the game's `Element` data nor the
// sim has a latent heat. A tank that condenses or boils needs the energy to be real: the heat
// released by condensation, or spent by evaporation, has to come from or go to the contents.
//
// The model follows Stationeers' state-change accounting: a per-substance latent heat, a
// gradual conversion bounded by what the source phase's own sensible heat can pay for, and a
// "convert the rest" floor so a step never leaves a meaningless remainder. It is adapted to kg
// rather than mol, because the tanks this serves track kg. The thresholds are the game's own
// per-element transition temperatures; the latent-heat table is supplied by the caller (the
// framework's phase-change facade holds the values and their sources).
//
// This is a pure function, like its neighbours in gas_mixture.h and liquid_mixture.h: which
// species a tank holds and how much stays in managed code, as a vanilla reservoir's storage
// does, and only the physics formula lives natively.
namespace oni_sim::phase {

// One step's worth of phase-change conversion for a single species crossing a single
// threshold (boiling/evaporating if temperatureK is ABOVE thresholdK, condensing/freezing if
// BELOW it -- the caller decides which threshold applies, same as TransitionCell's own
// two-branch high/low check).
struct PhaseChangeStep {
  // Mass (kg) that should move to the other phase this step. 0 if nothing should convert
  // (deltaT is zero, or every input is degenerate).
  float converted_mass_kg;
  // The SOURCE mass's own temperature after paying (or receiving) this step's latent-heat
  // cost -- always clamped so a single step can never cross back past thresholdK: a step
  // cannot overshoot equilibrium.
  float remaining_temperature_k;
};

// massKg/temperatureK describe the SOURCE phase's own current holding (e.g. the gas that
// might condense). thresholdK is the real vanilla transition temperature for this species at
// this phase boundary (Element.highTemp for a liquid's boiling point, Element.lowTemp for a
// gas's condensation point -- caller-supplied so a future pressure-adjusted threshold can
// simply be computed by the caller and handed in unchanged; this function has no opinion on
// where thresholdK came from).
//
// latentHeatJPerKg/specificHeatCapacity are both J/(kg*K)-scale per-kg quantities (not J/mol
// -- this tank tracks kg, so the caller converts once, at the table, not per call).
//
// dtSeconds/conversionRatePerSecond together bound how much of the "affordable" mass
// (whatever the source's own sensible heat could pay for) actually converts this step --
// `conversionRatePerSecond=1` over a 1-second step converts the FULL affordable amount in one
// call; a 10%-per-step conversion is a rate of 0.1 with dtSeconds=1. minRemainderKg is the
// "convert it all, don't leave dust" floor, as a caller-chosen mass.
inline PhaseChangeStep ComputePhaseChangeStep(float massKg, float temperatureK,
                                               float thresholdK, float latentHeatJPerKg,
                                               float specificHeatCapacity, float dtSeconds,
                                               float conversionRatePerSecond,
                                               float minRemainderKg) {
  PhaseChangeStep result{0.0f, temperatureK};
  if (massKg <= 0.0f || latentHeatJPerKg <= 0.0f || specificHeatCapacity <= 0.0f) return result;

  const float deltaT = temperatureK - thresholdK;
  if (deltaT == 0.0f) return result;
  const bool rising = deltaT > 0.0f;  // true: boiling/evaporating; false: condensing/freezing

  // Energy this mass's own sensible heat can supply (or must shed) getting back to the
  // threshold, compared against the full conversion cost.
  const float availableEnergyJ = massKg * specificHeatCapacity * std::fabs(deltaT);
  float affordableMassKg = std::min(massKg, availableEnergyJ / latentHeatJPerKg);
  if (affordableMassKg <= 0.0f) return result;

  float convertMassKg;
  if (massKg - affordableMassKg <= minRemainderKg) {
    // Converting only the energy-affordable amount would leave physically meaningless dust
    // behind -- take it all in one step.
    convertMassKg = affordableMassKg;
  } else {
    const float rate = std::clamp(conversionRatePerSecond * dtSeconds, 0.0f, 1.0f);
    convertMassKg = affordableMassKg * rate;
  }
  convertMassKg = std::min(convertMassKg, massKg);
  if (convertMassKg <= 0.0f) return result;

  // The converting mass pays for its own transition out of the remaining mass's sensible
  // heat -- energy conserved by construction: whatever the conversion consumed (rising) or
  // released (falling) shows up as exactly that much temperature change in what's left.
  const float energyForConversionJ = convertMassKg * latentHeatJPerKg;
  const float remainingMassKg = massKg - convertMassKg;
  float remainingTemperatureK = temperatureK;
  if (remainingMassKg > 0.0f) {
    const float deltaTempK = energyForConversionJ / (remainingMassKg * specificHeatCapacity);
    remainingTemperatureK = rising ? temperatureK - deltaTempK : temperatureK + deltaTempK;
    // Never overshoot back past the threshold from this one step's cost alone.
    remainingTemperatureK =
        rising ? std::max(remainingTemperatureK, thresholdK) : std::min(remainingTemperatureK, thresholdK);
  }

  result.converted_mass_kg = convertMassKg;
  result.remaining_temperature_k = remainingTemperatureK;
  return result;
}

}  // namespace oni_sim::phase
