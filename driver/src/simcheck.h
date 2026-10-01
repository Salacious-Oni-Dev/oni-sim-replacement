// Klei's own per-cell sanity check, ported out of the game and run offline.
//
// WHY THIS EXISTS. The shipped game already carries a cell validator: the `SimCheckErrorMap`
// overlay in `SimDebugView`, reachable from the DevTools' Sim Debug window. It paints every
// cell by whether its element, mass and temperature are self-consistent, and Klei wrote it
// for exactly the faults a sim rewrite produces. In the game it is read by opening the
// DevTools and looking at a screen; this runs the same check offline.
//
// Nothing else offline asks that question. `bench` digests the world's bit patterns and
// `diffsim` compares two sims against each other, and a NaN that both sims produce, or that
// one sim produces in a scenario diffsim does not run, is invisible to both. This header is
// that question, asked on every cell of every offline run, with no game and no Klei DLL
// needed.
//
// IT IS A TRANSCRIPTION, NOT AN INTERPRETATION. `Classify` below is
// `SimDebugView.GetSimCheckErrorMapColour` statement for statement, and `StateChangeProximity`
// is `GetStateChangeColour`'s `t`. Three managed-to-sim mappings are the only liberty taken:
//
//   Grid.Element[cell].IsVacuum   ->  (state & kStateMask) == kStateVacuum
//                                     (`Element.IsVacuum` is `(state & State.Solid) == 0`,
//                                     and `State.Solid` is 3, the state mask -- so it reads
//                                     the two state bits, not the solid bit)
//   Grid.Pressure[cell]           ->  mass * 101.3f
//                                     (`Grid.PressureIndexer` is exactly `mass[i] * 101.3f`,
//                                     so the green rule's `mass < 1 && pressure < 1` is
//                                     really `mass < 1/101.3 = 0.009872 kg`; the second
//                                     clause is the binding one and the first never fires
//                                     alone)
//   element.highTempTransition != null -> highTempTransitionIdx != 0xFFFF
//                                     (`ElementLoader` leaves the managed field null when the
//                                     target hash resolves to nothing; the sim table writes
//                                     0xFFFF for the same elements -- `conduits.h` already
//                                     relies on that spelling)
//
// WHAT A COLOUR MEANS, AND WHAT IT DOES NOT. Klei's thresholds are Klei's, and three of the
// seven classes are lit by worlds that are working exactly as intended:
//
//   red      NaN mass, NaN temperature, mass > 10 000 kg, temperature > 10 000 K, or a
//            non-vacuum cell under 10 K. **The neutronium world border is 20 000 kg a cell**,
//            so every sealed world reads thousands of red cells that are not faults. Read a
//            MOVE in the count, not the count.
//   yellow   vacuum holding a temperature
//   blue     vacuum holding mass
//   gray     vacuum, clean
//   green    effectively massless gas (see the pressure mapping above)
//   magenta  more than 3 K above the element's highTemp, with a transition to take
//   cyan     within 3 K of the element's lowTemp, with a transition to take
//   black    nothing to say
//
// Magenta and cyan are states Mod 1 and Mod 3 create on purpose (a superheated working
// fluid, a 220 K Martian sky held gaseous by a condensation floor). They are pinned here
// because a MOVE in them is worth seeing, not because they are wrong.
//
// The counts are pinned as goldens rather than asserted as zero for the same reason.
#ifndef ONI_DRIVER_SIMCHECK_H
#define ONI_DRIVER_SIMCHECK_H

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "../../abi/sim_abi.h"

namespace simcheck {

using oni_sim::Element;

enum Class : uint8_t {
  kBlack = 0,
  kRed,
  kYellow,
  kBlue,
  kGray,
  kGreen,
  kMagenta,
  kCyan,
  kClassCount,
};

inline const char* Name(Class c) {
  switch (c) {
    case kBlack: return "black";
    case kRed: return "red";
    case kYellow: return "yellow";
    case kBlue: return "blue";
    case kGray: return "gray";
    case kGreen: return "green";
    case kMagenta: return "magenta";
    case kCyan: return "cyan";
    default: return "?";
  }
}

// What each class is evidence OF, in one phrase, so a `bench` table can be read without this
// file open beside it.
inline const char* Meaning(Class c) {
  switch (c) {
    case kBlack: return "ordinary";
    case kRed: return "NaN / >10t / >10000K / <10K -- and the world border";
    case kYellow: return "vacuum with a temperature";
    case kBlue: return "vacuum with mass";
    case kGray: return "vacuum, clean";
    case kGreen: return "mass < 0.00987 kg";
    case kMagenta: return "over highTemp + 3, transition pending";
    case kCyan: return "under lowTemp + 3, transition pending";
    default: return "?";
  }
}

// `SimDebugView.GetSimCheckErrorMapColour`, statement for statement.
inline Class Classify(const Element& e, float mass, float temperature) {
  if (std::isnan(mass) || std::isnan(temperature) || mass > 10000.0f ||
      temperature > 10000.0f) {
    return kRed;
  }
  const bool vacuum = (e.state & 3) == 0;
  if (vacuum) {
    if (temperature != 0.0f) return kYellow;
    return mass == 0.0f ? kGray : kBlue;
  }
  if (temperature < 10.0f) return kRed;
  if (mass < 1.0f && mass * 101.3f < 1.0f) return kGreen;
  if (temperature > e.highTemp + 3.0f && e.highTempTransitionIdx != 0xFFFF) return kMagenta;
  if (temperature < e.lowTemp + 3.0f && e.lowTempTransitionIdx != 0xFFFF) return kCyan;
  return kBlack;
}

// `SimDebugView.GetStateChangeColour`'s lerp parameter: 0 well away from either threshold,
// 1 at one of them. Klei paints black-to-red over it; offline it is a number, and a cell over
// 0.5 is inside 2.5 % of a phase boundary.
//
// Vacuum returns 0 (Klei returns black without computing anything), and so does an element
// whose lowTemp or highTemp is zero -- those are the divisors and the game's own code
// divides by them unguarded. That is the only place this deviates: a 0/0 here would put a NaN
// into a diagnostic whose whole job is to find NaNs.
inline float StateChangeProximity(const Element& e, float temperature) {
  if ((e.state & 3) == 0) return 0.0f;
  const float lo_scale = e.lowTemp * 0.05f;
  const float hi_scale = e.highTemp * 0.05f;
  if (!(lo_scale > 0.0f) || !(hi_scale > 0.0f)) return 0.0f;
  const float a = std::fabs(temperature - e.lowTemp) / lo_scale;
  const float b = std::fabs(temperature - e.highTemp) / hi_scale;
  const float t = 1.0f - (a < b ? a : b);
  return t > 0.0f ? t : 0.0f;
}

struct Counts {
  uint64_t n[kClassCount];
  uint64_t statechange;  // cells with StateChangeProximity >= 0.5
  uint64_t cells;

  Counts() { Reset(); }
  void Reset() {
    memset(n, 0, sizeof(n));
    statechange = 0;
    cells = 0;
  }
  void Add(const Element& e, float mass, float temperature) {
    ++cells;
    ++n[Classify(e, mass, temperature)];
    if (StateChangeProximity(e, temperature) >= 0.5f) ++statechange;
  }
  // The classes that are a fault wherever they appear on a world this project builds.
  // Deliberately NOT red: the border is red in every sealed world. Yellow and blue are
  // vacuum holding something, which no kernel should ever leave behind.
  uint64_t VacuumFaults() const { return n[kYellow] + n[kBlue]; }

  // One canonical line, for digesting. Order is the enum's, so a new class appends.
  std::string Line() const {
    char buf[256];
    int off = 0;
    for (int i = 0; i < kClassCount; ++i) {
      off += snprintf(buf + off, sizeof(buf) - static_cast<size_t>(off), "%s%llu",
                      i ? " " : "", static_cast<unsigned long long>(n[i]));
    }
    snprintf(buf + off, sizeof(buf) - static_cast<size_t>(off), " sc %llu of %llu",
             static_cast<unsigned long long>(statechange),
             static_cast<unsigned long long>(cells));
    return std::string(buf);
  }

  // The human-readable form, as a string rather than a print, because `diffsim` routes every
  // line of its transcript through a `#define printf` that digests it -- and a header included
  // above that define would quietly write past the digest. Two runs whose simcheck lines
  // differ have to fail the golden, not merely look different on a terminal.
  std::string Describe(const char* label) const {
    char buf[512];
    int off = snprintf(buf, sizeof(buf), "  %-13s %8llu cells", label,
                       static_cast<unsigned long long>(cells));
    for (int i = 0; i < kClassCount; ++i) {
      if (!n[i]) continue;
      off += snprintf(buf + off, sizeof(buf) - static_cast<size_t>(off), "  %s %llu",
                      Name(static_cast<Class>(i)), static_cast<unsigned long long>(n[i]));
    }
    snprintf(buf + off, sizeof(buf) - static_cast<size_t>(off), "  statechange %llu",
             static_cast<unsigned long long>(statechange));
    return std::string(buf);
  }

  void Print(const char* label) const { printf("%s\n", Describe(label).c_str()); }
};

}  // namespace simcheck

#endif  // ONI_DRIVER_SIMCHECK_H
