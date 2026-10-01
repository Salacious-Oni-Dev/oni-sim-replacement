// The planet a world is standing on -- Flagship 2 / Mod 3 (Dynamic Planet).
//
// This is the one kernel in the sim with no counterpart in the game at all. Vanilla ONI has
// no concept of an ambient environment at all: the surface of an asteroid is vacuum, and vacuum
// there is the absence of a cell's contents rather than a declared place with a temperature, a
// pressure and a composition. `abi/sim_abi_ext.h`'s `kSetWorldEnvironment` declares that place per
// world, and this is what reads it.
//
// THREE TERMS, and they are separate physics that happen to share an input:
//
//   1. SOLAR HEATING. A cell the sun reaches absorbs `absorptivity * irradiance * lit` watts per
//      square metre of its own face. This is the honest source of a planet's day/night swing:
//      nothing scripts the temperature, the ground warms while the sun is up and radiates while
//      it is not. The energy is charged to `absorbed_from_environment`, which is
//      `radiated_to_environment`'s twin inwards, so the pair is a world's net radiative budget.
//
//   1b. RADIATIVE LOSS FROM THE GROUND, which is term 1 run backwards and is not optional.
//      KIRCHHOFF'S LAW: a grey body absorbs and emits with the same coefficient, so the world's
//      `solar_absorptivity` is also the emissivity of every cell the sun can reach, and a cell
//      that can take sunlight in can shed heat out through the same clear column. Without it the
//      solar term is a RATCHET -- a lit tile gains about 0.05 K per cycle and never gives any of
//      it back, and a planet's crust cooks over a few hundred cycles with nothing in the books to
//      say why. With it the two flows balance (at Mars' 318 W/m2 and 0.75 absorptivity, near
//      280 K at noon) and a surface temperature is something the world arrives at rather than
//      something somebody typed. Charged to its OWN bucket, `cell_radiated_to_environment`, and
//      not to the building one -- see World::EnergyLedger for why the finite-reservoir policy
//      needs those two apart.
//
//   2. THE SURFACE ATMOSPHERE BOUNDARY. A cell with a clear path to the sky is driven toward the
//      world's declared pressure, composition and temperature. This is what makes Mars' surface
//      2.5 kPa of CO2 rather than vacuum, and it is what makes the atmosphere a resource a
//      machine can actually draw on.
//
// WHAT COUNTS AS "THE SKY", AND WHY IT COSTS NOTHING. Both terms key off the sunlight texture the
// sim already computes per world per frame (`sim/textures.h`, `ComputeSunlight`): each world's own
// top row starts at full exposure, the beam descends absorbing through fluids, and a SOLID stops
// it dead. So a non-zero byte means "no solid between this cell and space", which is exactly the
// set of cells a planetary atmosphere touches -- already computed, already per world, and needing
// no scan of its own. The byte is one frame stale, which at a 200 ms frame and 255 steps of shadow
// is not a distinction anything in the game can observe.
//
// The lit FRACTION is used as a weight rather than as a yes/no, which falls out of the same
// arithmetic: a cell under three metres of water still has a path to the sky in the topological
// sense, but it should not be held at the planet's pressure, and the beam has already been
// attenuated by exactly the medium that should be attenuating the coupling.
//
// THE BOUNDARY IS AN INFINITE SOURCE, BY DESIGN, AND THE LEDGER SAYS SO.
// Matter that appears here is charged to `Ledger::atmosphere_boundary` and the heat inside it to
// `EnergyLedger::atmosphere_boundary`, both from the same before/after difference at the same call
// site. That is the whole difference between a boundary condition and a leak: the sim is not
// pretending this mass was always there. Stationeers' planetary atmosphere behaves the same way:
// the gas it hands out is never debited from the planet.
//
// EVERY TERM IS INERT AT ZERO. A world with no environment record, which is every world that
// existed before this file and every offline scenario, takes the early return on the first line
// of the loop body and the kernel is a no-op -- the same byte-identical-until-asked contract
// `kSetCellThermalMassBonus` and `kSetInvertedGravityElement` make.
#ifndef ONI_SIM_ENVIRONMENT_H_
#define ONI_SIM_ENVIRONMENT_H_

#include <cstdint>
#include <vector>

#include "buildings.h"
#include "gas_mixture.h"
#include "world.h"

namespace oni_sim {

// The mass of `element` that fills one cell at `pressure_kpa` and `temperature_k`, from PV = nRT.
// Returns 0 for an element with no molecular mass -- a solid or a liquid, neither of which this
// boundary supplies.
inline float BoundaryTargetMass(const ElementTable& table, int32_t element, float pressure_kpa,
                                float temperature_k, const Tunables& tun) {
  if (element < 0 || pressure_kpa <= 0.0f || temperature_k <= 0.0f) return 0.0f;
  const float molar = table.MolecularMassOf(static_cast<uint16_t>(element));
  if (!(molar > 0.0f)) return 0.0f;
  // `PressureFromMoles` returns pascals over `CellVolumeM3` cubic metres, so this is its
  // inverse in the same units: moles = P * V / (R * T), with the kPa the message carries
  // converted to Pa.
  const double moles = static_cast<double>(pressure_kpa) * 1000.0 *
                       static_cast<double>(tun.cell_volume_m3) /
                       (static_cast<double>(tun.gas_constant_r) *
                        static_cast<double>(temperature_k));
  return static_cast<float>(moles * static_cast<double>(molar) /
                            static_cast<double>(gas::kGramsPerKilogram));
}

// One region's worth of all three terms. Runs at the TOP of the substep, before `StepConduction`: the
// planet's state is a boundary condition established for the substep, and the sim's own physics
// then acts on it within that same substep rather than a substep later.
inline void StepWorldEnvironment(World* w, const ElementTable& table, size_t ri, float dt,
                                 const std::vector<uint8_t>* sky,
                                 const std::vector<uint8_t>* beam = nullptr) {
  const int32_t world = w->WorldIndexOfRegion(ri);
  const World::Environment& env = w->EnvironmentOfWorld(world);
  const Tunables tun = g_tunables;  // tunables.h, THE ONE RULE

  const bool wants_solar = env.solar_absorptivity > 0.0f && env.peak_irradiance_w_m2 > 0.0f &&
                           env.full_sun_lux > 0.0f;
  // The sink a CELL radiates into, which is the same one a building on this world already uses:
  // this world's own record if it has one, the single global scalar (`kSetEnvironmentTemperature`)
  // if it does not. Gated on the world's absorptivity and nothing else, because by Kirchhoff that
  // one number is the emissivity as well -- a world that declares no absorptivity has declared
  // that its ground does not couple to its sky in either direction, which is every world that
  // existed before Mod 3.
  const float cell_sink = w->SinkTemperatureForWorld(world);
  const bool wants_cell_radiation = env.solar_absorptivity > 0.0f && cell_sink > 0.0f;
  const bool wants_boundary = env.boundary_rate > 0.0f && env.surface_pressure_kpa > 0.0f &&
                              env.boundary_element >= 0;
  if (!wants_solar && !wants_cell_radiation && !wants_boundary) return;
  if (sky == nullptr || sky->empty()) return;
  // The direct beam replaces the sky view for solar heating only, and only on a world the sim
  // has been given a sun for. Every other world -- all of them before Mod 3's sun path -- reads
  // the sky view for all three terms, exactly as before.
  World::SunDirection sun;
  const bool use_beam = beam != nullptr && beam->size() == sky->size() &&
                        w->SunOfWorld(world, &sun);

  // 0 at night, 1 at noon, 0 during an eclipse -- `TimeOfDay.UpdateSunlightIntensity` already
  // zeroes the lux for one, so an eclipse needs no code here.
  float sun_fraction = 0.0f;
  if (wants_solar) {
    sun_fraction = w->RegionSunlight(ri) / env.full_sun_lux;
    if (sun_fraction < 0.0f) sun_fraction = 0.0f;
    if (sun_fraction > 1.0f) sun_fraction = 1.0f;
  }
  const double solar_kw_per_cell = wants_solar
                                       ? 0.001 * static_cast<double>(env.peak_irradiance_w_m2) *
                                             static_cast<double>(sun_fraction) *
                                             static_cast<double>(env.solar_absorptivity)
                                       : 0.0;

  const float boundary_t =
      env.boundary_temperature_k > 0.0f ? env.boundary_temperature_k : env.sink_kelvin;
  const float target_mass =
      wants_boundary
          ? BoundaryTargetMass(table, env.boundary_element, env.surface_pressure_kpa, boundary_t,
                               tun)
          : 0.0f;
  const uint16_t bel = static_cast<uint16_t>(env.boundary_element);
  const bool boundary_live = wants_boundary && target_mass > 0.0f && boundary_t > 0.0f;

  std::vector<PhaseEntry>& cells = w->Phases();
  const int32_t pw = w->PaddedWidth();
  const int32_t gw = w->GameWidth();
  const int32_t gh = w->GameHeight();
  const World::PaddedRect& pr = w->PaddedRegion(ri);

  double mass_charge = 0.0;
  double boundary_energy_charge = 0.0;
  double solar_charge = 0.0;
  double cell_radiated_charge = 0.0;

  for (int32_t y = pr.y0; y < pr.y1; ++y) {
    for (int32_t x = pr.x0; x < pr.x1; ++x) {
      const int32_t gx = x - 1;
      const int32_t gy = y - 1;
      if (gx < 0 || gy < 0 || gx >= gw || gy >= gh) continue;
      const size_t gi = static_cast<size_t>(gy) * static_cast<size_t>(gw) +
                        static_cast<size_t>(gx);
      if (gi >= sky->size()) continue;
      // TWO EXPOSURES on a world with a sun (`kSetWorldSun`): `lit` is the sky view, and it
      // still governs everything that means "open to the sky" -- term 1b's view factor and term
      // 2's boundary. `sun_lit` is the direct beam, and only term 1 reads it. A cell can have
      // either without the other: under an overhang with the sun low it has sky and no beam,
      // and at the foot of that overhang's far side it can have beam and no sky.
      const uint8_t lit_byte = (*sky)[gi];
      const uint8_t beam_byte = use_beam ? (*beam)[gi] : lit_byte;
      if (lit_byte == 0 && beam_byte == 0) continue;
      const double lit = static_cast<double>(lit_byte) / 255.0;
      const double sun_lit = static_cast<double>(beam_byte) / 255.0;

      PhaseEntry& c = cells[static_cast<size_t>(y) * pw + x];

      // ---- 1. solar heating -------------------------------------------------------------
      //
      // The ground is what this warms. A gas cell holds grams and barely moves; the first solid
      // the beam meets holds hundreds of kilograms and is written with the exposure it had
      // BEFORE it absorbed the beam (`ComputeSunlight` writes then breaks), so a planet's
      // surface tile is lit and its interior is not. That is the whole day/night mechanism.
      if ((wants_solar || wants_cell_radiation) && c.mass > 0.0f) {
        const float shc = table.At(c.element).specificHeatCapacity;
        const double hc = static_cast<double>(c.mass) * static_cast<double>(shc);
        if (shc > 0.0f && hc > 0.0) {
          if (wants_solar) {
            const double kj = solar_kw_per_cell * sun_lit * static_cast<double>(dt);
            if (kj > 0.0) {
              // Written the way every other float-precision transfer in this sim is: propose,
              // write, and charge what LANDED. A heavy tile can round a small gain away
              // entirely, and charging the proposal would put energy in the ledger that is not
              // in the grid.
              const double before = static_cast<double>(c.temperature);
              const float after = static_cast<float>(before + kj / hc);
              const double landed = (static_cast<double>(after) - before) * hc;
              if (landed != 0.0) {
                c.temperature = after;
                solar_charge += landed;
              }
            }
          }

          // ---- 1b. the same window, outwards ---------------------------------------------
          //
          // Three properties held in common with the building term in `buildings.h`, and each
          // one is load-bearing for the same reason there:
          //
          //   * ONE-WAY. A cell colder than the sink is left alone rather than warmed. The sky
          //     is not allowed to become a back-door heat source; only the star is, and that is
          //     term 1, which is metered separately.
          //   * THE VIEW FACTOR IS THE LIT FRACTION, the same weight term 1 uses. What can be
          //     seen can be radiated to: a tile under three metres of water has a topological
          //     path to space and almost no radiative coupling through it, and the beam has
          //     already been attenuated by exactly the medium that should attenuate this.
          //   * AN OVERSHOOT CLAMP at half the energy above the sink, which is what stops a
          //     light cell at a large excess from being driven past the sink and oscillating.
          //     Same clamp, same argument, as the building term's.
          //
          // The area is one square metre because a cell's face is one square metre -- ONI's
          // grid is metres and `CellVolumeM3` is 1 by default.
          if (wants_cell_radiation) {
            const double tb = static_cast<double>(c.temperature);
            const double ts = static_cast<double>(cell_sink);
            if (tb > ts) {
              const double flux_kw = tun.stefan_boltzmann_kw *
                                     static_cast<double>(env.solar_absorptivity) * lit *
                                     (tb * tb * tb * tb - ts * ts * ts * ts);
              double kj = flux_kw * static_cast<double>(dt);
              const double half_excess = 0.5 * hc * (tb - ts);
              if (kj > half_excess) kj = half_excess;
              if (kj > 0.0) {
                const float after = static_cast<float>(tb - kj / hc);
                const double left = (tb - static_cast<double>(after)) * hc;
                if (left != 0.0) {
                  c.temperature = after;
                  cell_radiated_charge += left;
                }
              }
            }
          }
        }
      }

      // ---- 2. the surface atmosphere boundary --------------------------------------------
      if (!boundary_live) continue;
      // A solid tile is the planet's crust, not its atmosphere. ONI's conduction already owns
      // what happens inside it, and the beam stops here anyway.
      if (c.mass > 0.0f && table.IsSolid(c.element)) continue;
      // A liquid is not this boundary's business either: pushing gas into a cell of water is
      // not a boundary condition, it is a bug with a ledger entry.
      if (c.mass > 0.0f && table.Phase(c.element) == kStateLiquid) continue;

      // Clamped at use as well as when `kSetWorldEnvironment` stores it: `MaxWorldBoundaryRate`
      // is a live ceiling, and a rate stored under a higher one must not outlive a lowered one.
      // Bit-exact while the ceiling has not moved.
      const double rate =
          static_cast<double>(std::min(env.boundary_rate, tun.max_world_boundary_rate)) * lit;
      if (!(rate > 0.0)) continue;

      const double mass_before = static_cast<double>(c.mass);
      const float shc_before = c.mass > 0.0f ? table.At(c.element).specificHeatCapacity : 0.0f;
      const double energy_before =
          mass_before * static_cast<double>(shc_before) * static_cast<double>(c.temperature);

      const bool empty = !(c.mass > 0.0f) || table.IsVacuum(c.element);
      if (empty || c.element == bel) {
        // THE ORDINARY CASE: vacuum, or the planet's own gas. Drive both mass and temperature
        // toward the planet's, closing `rate` of the gap.
        if (empty) {
          c.element = bel;
          // An empty cell has no temperature worth blending -- vanilla's ZeroMasslessCells
          // leaves it at 0 K, and blending against that would deliver the planet's gas at half
          // its temperature.
          c.temperature = boundary_t;
          c.mass = static_cast<float>(rate * static_cast<double>(target_mass));
        } else {
          c.mass = static_cast<float>(mass_before +
                                      rate * (static_cast<double>(target_mass) - mass_before));
          c.temperature = static_cast<float>(static_cast<double>(c.temperature) +
                                             rate * (static_cast<double>(boundary_t) -
                                                     static_cast<double>(c.temperature)));
        }
      } else {
        // THE CELL HOLDS SOMETHING ELSE -- oxygen a colony vented, say. The planet does not get
        // to overwrite it: that would delete the colony's matter and hide the deletion inside a
        // boundary. It is still in CONTACT with it, so the temperature is pulled toward the
        // planet's and the mass is left alone. Vanilla's own flow is what disperses the gas.
        c.temperature = static_cast<float>(
            static_cast<double>(c.temperature) +
            rate * (static_cast<double>(boundary_t) - static_cast<double>(c.temperature)));
      }

      const float shc_after = c.mass > 0.0f ? table.At(c.element).specificHeatCapacity : 0.0f;
      const double energy_after =
          static_cast<double>(c.mass) * static_cast<double>(shc_after) *
          static_cast<double>(c.temperature);
      mass_charge += static_cast<double>(c.mass) - mass_before;
      boundary_energy_charge += energy_after - energy_before;
    }
  }

  if (solar_charge != 0.0) w->NoteAbsorbedFromEnvironment(solar_charge);
  if (cell_radiated_charge != 0.0) w->NoteCellRadiatedToEnvironment(cell_radiated_charge);
  if (mass_charge != 0.0) w->NoteAtmosphereBoundary(mass_charge);
  if (boundary_energy_charge != 0.0) w->NoteAtmosphereBoundaryEnergy(boundary_energy_charge);
}

}  // namespace oni_sim

#endif  // ONI_SIM_ENVIRONMENT_H_
