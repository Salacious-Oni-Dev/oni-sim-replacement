// A grid raycast: the first cell on a line that a caller says stops it, and how much of a
// radiation-like quantity survives the cells in between. Published as `SIM_QueryRaycast` (sim/simdll.cpp,
// abi/sim_ext_api.h).
//
// IT IS NOT A NEW WALK. It visits exactly the cells `RadiationAbsorptionAlongLine`
// (sim/radiation.h) visits, by running that function's own loop, so a
// sensor asking "can I see that cell" and the radiation field asking "how much reaches that
// cell" can never disagree about which cells lie between. That is the property a second,
// tidier Bresenham would have lost: Klei's walk always runs from the LOW end of the major axis
// to the high end, whichever end the caller calls the start, and a walk run the other way picks
// different cells at every half-step tie.
//
// So a ray cast from the high end is answered from Klei's order backwards, in ONE pass and with
// no allocation: the first stopping cell counted from the start is the LAST one in Klei's order,
// and the transmission up to it is the product over the cells from there to the end. The running
// product is restarted at every stopping cell so that, when the pass finishes, it holds exactly
// that suffix.
//
// THE TRANSMISSION IS KLEI'S PRODUCT, BIT FOR BIT, WHEN NOTHING STOPS THE RAY. Every product
// here is taken in Klei's order, over Klei's per-cell factor (`RadiationAbsorption`), then
// clamped the way Klei clamps it -- so with both masks zero the answer IS
// `RadiationAbsorptionAlongLine(start, end)`, which gastest asserts over a few hundred random
// lines rather than trusting this paragraph. When something does stop it, the product runs over
// the start-side cells up to AND INCLUDING the stopping cell (the ray is attenuated by what it
// hit), and in Klei's order, which for a high-end start means from the stopping cell outwards.
//
// Both endpoints are visited, as in Klei's walk: a ray cast from inside a tile starts inside
// that tile, and it is the caller's to decide whether that cell stopping it is an answer or a
// reason to pick a different start. `lastClear` exists so that "how far did it get" does not
// need a second query.
#pragma once

#include <cstddef>
#include <cstdint>

#include "radiation.h"
#include "world.h"

namespace oni_sim {

struct RaycastHit {
  static constexpr size_t kNoCell = static_cast<size_t>(-1);
  size_t hit = kNoCell;         // padded index of the first stopping cell from the start
  size_t last_clear = kNoCell;  // padded index of the cell just before it, start-side; the
                                // end cell when nothing stopped the ray; kNoCell when the
                                // start cell itself stopped it
  int32_t visited = 0;          // cells from the start up to and including `hit`, or all of them
  float transmission = 0.0f;    // Klei's product over those same cells, clamped to [0, 1]
};

// Does this cell stop the ray? `phase_mask` holds one bit per phase, `1 << (state & 3)` --
// vacuum, gas, liquid, solid -- read off the cell's element, which on a mixture cell is the
// dominant-element facade exactly as everything else in the vanilla ABI sees it.
// `property_mask` is tested against the cell's `Sim.Cell.Properties` byte, so a caller can stop
// on what the game has SET on a cell (impermeability, a constructed tile) as well as on what is
// in it.
//
// `ignore_mask` overrides both: a cell carrying any of those property bits never stops the ray
// (it still attenuates it). It exists for the case two ORed masks cannot say, and it is the
// commonest one -- line of sight. A Glass Tile is a SOLID cell of glass, and vanilla marks it
// `Transparent` (`GlassTileConfig` sets `SimCellOccupier.setTransparent`), so "stopped by
// solids, except windows" is phase SOLID with ignore TRANSPARENT.
inline bool RaycastStops(const World& w, const ElementTable& t, size_t cell, uint32_t phase_mask,
                         uint32_t property_mask, uint32_t ignore_mask) {
  const uint8_t props = w.Properties()[cell];
  if ((props & ignore_mask) != 0) return false;
  const uint32_t phase = t.Phase(w.Phases()[cell].element);
  return ((phase_mask >> phase) & 1u) != 0 || (props & property_mask) != 0;
}

// Padded coordinates in, padded cells out. The loop header is `RadiationAbsorptionAlongLine`'s,
// line for line; change one and the other has to change with it, which gastest will say.
inline RaycastHit Raycast(const World& w, const ElementTable& t, int32_t x0, int32_t y0,
                          int32_t x1, int32_t y1, uint32_t phase_mask, uint32_t property_mask,
                          uint32_t ignore_mask) {
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

  // Klei swaps the endpoints exactly when `major_b < major_a`; that is the whole definition of
  // "the caller's start is the high end".
  const bool reversed = major_b < major_a;

  RaycastHit r;
  float transmission = 1.0f;
  int32_t index = 0;             // position in Klei's order
  int32_t last_stop_index = -1;  // reversed only: the latest stopping cell in Klei's order
  bool clear_pending = false;    // reversed only: the next cell is the one before the stop
  size_t previous = RaycastHit::kNoCell;
  size_t first = RaycastHit::kNoCell;  // Klei's first cell: the caller's END when reversed

  int32_t minor = static_cast<int32_t>(from);
  const int32_t last = static_cast<int32_t>(hi);
  for (int32_t major = static_cast<int32_t>(lo); major <= last; ++major, ++index) {
    const int32_t x = along_x ? major : minor;
    const int32_t y = along_x ? minor : major;
    const size_t cell = static_cast<size_t>(y) * static_cast<size_t>(pw) +
                        static_cast<size_t>(x);
    if (index == 0) first = cell;
    const bool stops = RaycastStops(w, t, cell, phase_mask, property_mask, ignore_mask);

    if (reversed) {
      if (clear_pending) {
        r.last_clear = cell;
        clear_pending = false;
      }
      if (stops) {
        transmission = 1.0f;  // only the suffix from the latest stop outwards is the answer
        last_stop_index = index;
        r.hit = cell;
        r.last_clear = RaycastHit::kNoCell;
        clear_pending = true;
      }
    }
    transmission = (1.0f - RadiationAbsorption(w, t, cell)) * transmission;

    if (!reversed && stops) {
      r.hit = cell;
      r.last_clear = previous;
      r.visited = index + 1;
      break;
    }
    previous = cell;

    const float next = err - delta;
    err = next < 0.0f ? next + span : next;
    if (next < 0.0f) minor += step;
  }

  const int32_t count = index;  // cells in the whole line when the loop ran to the end
  if (reversed) {
    if (last_stop_index >= 0) {
      r.visited = count - last_stop_index;
    } else {
      // Nothing stopped it, so the ray reached the caller's end -- which, reversed, is the
      // cell Klei's order visits FIRST.
      r.visited = count;
      r.last_clear = first;
    }
  } else if (r.hit == RaycastHit::kNoCell) {
    r.visited = count;
    r.last_clear = previous;
  }

  if (transmission >= 1.0f) transmission = 1.0f;
  if (transmission <= 0.0f) transmission = 0.0f;
  r.transmission = transmission;
  return r;
}

}  // namespace oni_sim
