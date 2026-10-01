// ================================================================================================
// LAYER C MIXING -- a liquid-carried amount evens out across the body of liquid holding it.
//
// WHAT THIS CLOSES. `kSetCellPropertyTransport` makes an amount ride the liquid's mass, and that
// is transport and nothing else: the amount goes where its water goes, so a cell whose water
// never moves never gains or loses any. A pond carbonated by a column of bubbles therefore kept
// the carbonation in the columns the bubbles rose through and nowhere else. With vents in columns 12-14 a sensor at column 15 read 0.000 g/kg -- exactly zero,
// structurally, for any pressure and any length of run -- while the pond's own mean understated
// the water a pump beside it drew by more than a factor of two. A player expects a sensor
// anywhere in a carbonated pond to read the carbonation, and this pass is what makes it so.
//
// WHAT IT MODELS. Not molecular diffusion. CO2 in water is about 1e-9 m2/s, which crosses a
// one-metre cell in something like eleven days; a kernel modelling it faithfully would be a
// kernel that does nothing. What actually evens a real vessel out is CONVECTION -- bulk motion of
// the water, driven by a bubble plume where there is one and by density differences where there
// is not -- and an ONI cell has no representation of that, because an ONI cell has one velocity
// and it is zero. A 1 m3 cell is far larger than every eddy inside it, so this is the model every
// engineering text reaches for at that scale: an effective exchange rate between adjacent
// parcels, with the rate standing in for motion the grid cannot resolve. The rate, where it comes
// from and why it is 0.125 are at `ext::kSetPayloadMixing`.
//
// WHAT IT DOES NOT MODEL. Stratification. Carbonated water is denser than plain water -- 0.27% at
// soda strength, by our own `Solubility.SolutionDensityKgPerM3` -- so in reality the rich parcel
// sinks and a settled pond ends up richer at the bottom. This kernel is isotropic: it mixes
// upwards exactly as readily as downwards and leaves a UNIFORM pond. That is a smaller error than
// the one it removes (uniform is wrong by the stratification gradient; columns-only was wrong by
// everything), and it is stated here rather than hidden.
//
// WHAT IT CANNOT DISTURB. Nothing vanilla. This kernel reads `Phase().element` and `Phase().mass`
// and writes only extension-property lanes, so every array Klei's DLL has is untouched and a
// world with no payload skips the whole sweep on `PayloadMixingActive()`. It is not a port of
// anything: `SimBase::UpdateData` has no such pass, which is exactly why row 37 existed.
// ================================================================================================
#pragma once

#include <cstddef>
#include <cstdint>

#include "world.h"

#include "census.h"

namespace oni_sim {

// One neighbour pair. Both halves must be the SAME liquid element with mass on them.
//
// SAME ELEMENT, not merely both liquid. Two different liquids do not share a cell in this grid
// and do not mix across one either -- `StepFlow`'s mover refuses the pair outright and
// `StepLiquidDisplacement` swaps them past each other rather than blending them. A dissolved
// amount that crossed from water into brine would be gas leaving a solvent it is soluble in for
// one it may not be, at a Henry constant nothing here knows, so the conservative gate is the one
// the liquid kernels already hold to.
inline void PayloadMixPairCells(World* w, const ElementTable& elements, size_t a, size_t b) {
  const PhaseEntry& pa = w->Phase(a);
  const PhaseEntry& pb = w->Phase(b);
  if (pa.element != pb.element) return;
  if (elements.Phase(pa.element) != kStateLiquid) return;
  if (!(pa.mass > 0.0f) || !(pb.mass > 0.0f)) return;
  w->PayloadMixPair(a, b, pa.mass, pb.mass);
}

// The pair sweep, in the shape `StepDiseaseDiffusion` already has: every cell in the region
// against the one to its right and the one above it, so every unordered orthogonal pair inside
// the region is visited exactly once.
//
// READS LIVE, NOT A SNAPSHOT, and unlike the disease sweep that is a choice rather than a
// transcription. There is no vanilla kernel here to be bit-exact against, so the only question is
// whether reading a cell a previous pair already wrote can misbehave -- and it cannot: every
// transfer moves a fraction of a remaining gap towards equality and is clamped to what the giving
// cell holds, so the pass is monotone, cannot overshoot and cannot go negative whatever order it
// visits in. What order DOES change is the rate, slightly: a cell mixed with its left neighbour
// before its right one carries a little of the left one's gas into the right pair in the same
// substep, which spreads a gradient marginally faster than a snapshot pass would. That is
// strictly closer to convection, which does not wait a substep either.
inline void StepPayloadMixing(World* w, const ElementTable& elements, size_t ri) {
  ONI_SWEEP(kPayloadMix);
  if (!w->PayloadMixingActive()) return;
  const World::PaddedRect& r = w->PaddedRegion(ri);
  const int32_t pw = w->PaddedWidth();
  for (int32_t y = r.y0; y < r.y1; ++y) {
    ONI_EXAMINED(kPayloadMix, r.x1 - r.x0);
    for (int32_t x = r.x0; x < r.x1; ++x) {
      const size_t c = static_cast<size_t>(y) * pw + x;
      // The cheap gate first and once: a cell that is not liquid pairs with nothing, and on an
      // ordinary map most of the grid is not liquid.
      const PhaseEntry& here = w->Phase(c);
      if (elements.Phase(here.element) != kStateLiquid || !(here.mass > 0.0f)) continue;
      PayloadMixPairCells(w, elements, c, c + 1);
      PayloadMixPairCells(w, elements, c, c + static_cast<size_t>(pw));
    }
  }
}

}  // namespace oni_sim
