// Shared world seeding for the offline tools.
//
// `diffsim.cpp` grew its own copies of all of this while it was the only tool that needed
// to build a world; `bench.cpp` needs exactly the same pieces, so they live here. diffsim
// has not been migrated onto this header yet — it is the correctness oracle and this pass
// is supposed to change nothing it can see. Migrating it is a separate, mechanical change.
//
// The scenario builders below are the *other* reason this file exists: a world big enough
// for cache behaviour to exist has to be generated rather than typed out, and it has to be
// generated the same way every run or two timings cannot be compared.

#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <utility>
#include <vector>

#include "../../abi/sim_abi.h"

namespace oni_bench {

using oni_sim::Cell;
using oni_sim::DiseaseCell;
using oni_sim::Element;
using oni_sim::SimBackwall;
using oni_sim::SimMessageHash;

// Element ids are `Hash.SDBMLower(name)` over the element's *code* name, which is how a
// save survives the table being reordered. Verified against the six hashes diffsim had
// hard-coded: all six reproduce exactly.
constexpr int32_t SdbmLower(const char* s) {
  uint32_t h = 0;
  for (const char* p = s; *p; ++p) {
    const char c = (*p >= 'A' && *p <= 'Z') ? static_cast<char>(*p - 'A' + 'a') : *p;
    h = static_cast<uint32_t>(c) + (h << 6) + (h << 16) - h;
  }
  return static_cast<int32_t>(h);
}

enum : int32_t {
  kVacuum = SdbmLower("Vacuum"),
  kOxygen = SdbmLower("Oxygen"),
  kCarbonDioxide = SdbmLower("CarbonDioxide"),
  kHydrogen = SdbmLower("Hydrogen"),
  kSteam = SdbmLower("Steam"),
  kWater = SdbmLower("Water"),
  kMagma = SdbmLower("Magma"),
  kGranite = SdbmLower("Granite"),
  kSandStone = SdbmLower("SandStone"),
  kIgneousRock = SdbmLower("IgneousRock"),
  kObsidian = SdbmLower("Obsidian"),
  kCopper = SdbmLower("Copper"),
  kCuprite = SdbmLower("Cuprite"),
  kDirt = SdbmLower("Dirt"),
  kSand = SdbmLower("Sand"),
  kIce = SdbmLower("Ice"),
  kAlgae = SdbmLower("Algae"),
};

// ------------------------------------------------------------------- element table

struct Tables {
  std::vector<uint8_t> elements, diseases;
  int32_t count = 0;

  const Element* At(int32_t i) const {
    return reinterpret_cast<const Element*>(elements.data() + 4 +
                                            static_cast<size_t>(i) * sizeof(Element));
  }
  int32_t IndexOf(int32_t hash) const {
    for (int32_t i = 0; i < count; ++i) {
      if (At(i)->id == hash) return i;
    }
    return -1;
  }
  // A scenario that silently substitutes a missing element stops meaning what it says, so
  // this is loud on purpose.
  uint16_t Need(int32_t hash, const char* name) const {
    const int32_t i = IndexOf(hash);
    if (i < 0) {
      printf("element %s (hash %d) is not in this corpus\n", name, hash);
      return 0;
    }
    return static_cast<uint16_t>(i);
  }
};

inline bool LoadTables(const char* path, Tables* out) {
  FILE* f = fopen(path, "rb");
  if (!f) {
    printf("cannot open corpus %s\n", path);
    return false;
  }
  for (;;) {
    int32_t h[4];
    if (fread(h, sizeof(h), 1, f) != 1) break;
    const size_t bytes = static_cast<size_t>(h[2]) * (h[3] > 0 ? h[3] : 1);
    std::vector<uint8_t> payload(bytes);
    if (bytes && fread(payload.data(), 1, bytes, f) != bytes) break;
    if (h[1] == static_cast<int32_t>(SimMessageHash::Elements_CreateTable) &&
        out->elements.empty()) {
      out->elements = std::move(payload);
    } else if (h[1] == static_cast<int32_t>(SimMessageHash::Disease_CreateTable) &&
               out->diseases.empty()) {
      out->diseases = std::move(payload);
    }
    if (!out->elements.empty() && !out->diseases.empty()) break;
  }
  fclose(f);
  if (out->elements.size() < 4 || out->diseases.empty()) return false;
  memcpy(&out->count, out->elements.data(), 4);
  return out->count > 0;
}

// ------------------------------------------------------------------- world seeding

struct SeedWorld {
  int width = 0, height = 0;
  std::vector<uint16_t> element;
  std::vector<float> mass, temperature;
  // 255 is "no insulation". It stays the default here so
  // those scenarios keep sending exactly the bytes they used to.
  std::vector<uint8_t> insulation;

  int Cell(int x, int y) const { return y * width + x; }
  size_t Count() const { return element.size(); }
  void Init(int w, int h, uint16_t e, float m, float t) {
    width = w;
    height = h;
    const size_t n = static_cast<size_t>(w) * h;
    element.assign(n, e);
    mass.assign(n, m);
    temperature.assign(n, t);
    insulation.assign(n, 255);
  }
  void Set(int x, int y, uint16_t e, float m, float t) {
    if (x < 0 || y < 0 || x >= width || y >= height) return;
    const int c = Cell(x, y);
    element[c] = e;
    mass[c] = m;
    temperature[c] = t;
  }
  void SetInsulation(int x, int y, uint8_t v) {
    if (x < 0 || y < 0 || x >= width || y >= height) return;
    insulation[Cell(x, y)] = v;
  }
};

// SimData_InitializeFromCells' payload: header, then unpadded Sim.Cell / DiseaseCell /
// SimBackwall arrays.
inline std::vector<uint8_t> WorldPayload(const SeedWorld& w, bool headless_mode = true) {
  std::vector<uint8_t> b;
  auto put = [&](const void* p, size_t n) {
    const uint8_t* s = static_cast<const uint8_t*>(p);
    b.insert(b.end(), s, s + n);
  };
  const int32_t width = w.width, height = w.height;
  const uint32_t seed = 12345u;
  const uint8_t radiation = 0;
  const uint8_t headless = headless_mode ? 1 : 0;
  put(&width, 4);
  put(&height, 4);
  put(&seed, 4);
  put(&radiation, 1);
  put(&headless, 1);
  for (size_t i = 0; i < w.Count(); ++i) {
    Cell c{};
    c.elementIdx = w.element[i];
    c.mass = w.mass[i];
    c.temperature = w.temperature[i];
    c.insulation = w.insulation.empty() ? 255 : w.insulation[i];
    put(&c, sizeof(Cell));
  }
  for (size_t i = 0; i < w.Count(); ++i) {
    DiseaseCell d{};
    d.diseaseIdx = 0xFF;
    put(&d, sizeof(DiseaseCell));
  }
  for (size_t i = 0; i < w.Count(); ++i) {
    SimBackwall s{};
    put(&s, sizeof(SimBackwall));
  }
  return b;
}

// ------------------------------------------------------------------- scenarios

// A uniform block of granite. This is the **cheapest possible** world at any size:
// conduction sees a zero temperature difference at every pair and bails at the dead zone,
// and every cell fails StepFlow's phase check. It is here to bound the measurement from
// below, not to be representative of anything.
inline SeedWorld UniformGranite(const Tables& t, int width, int height) {
  SeedWorld w;
  w.Init(width, height, t.Need(kGranite, "Granite"), 2000.0f, 300.0f);
  return w;
}

// A deterministic stand-in for a real asteroid.
//
// Shape, and why each part is here:
//
//   * ~60 % solid, banded granite / sandstone / igneous with ore veins, on a mild depth
//     temperature gradient — enough element variety that `PairConductivity` actually
//     looks things up rather than hitting the same pair every time.
//   * Carved caverns holding gas at several pressures. Two gases, so the "different
//     elements never mix" path is exercised alongside the same-element flow path.
//   * Standing water with a free surface at the bottom of some caverns: liquid flow,
//     liquid/gas boundaries and the whole-cell swap path.
//   * A magma pocket under one water pool, which keeps a handful of cells transitioning
//     for the whole run instead of transitioning once on tick 1 and going quiet.
//   * An open gas column in the upper third with a pressure gradient and vacuum at the
//     very top, so the world is not uniformly dense.
//
// Everything is placed from a fixed-seed xorshift, so two runs generate the identical
// world and two timings are comparable.
inline SeedWorld Asteroid(const Tables& t, int width, int height) {
  const uint16_t granite = t.Need(kGranite, "Granite");
  const uint16_t sandstone = t.Need(kSandStone, "SandStone");
  const uint16_t igneous = t.Need(kIgneousRock, "IgneousRock");
  const uint16_t copper = t.Need(kCuprite, "Cuprite");
  const uint16_t dirt = t.Need(kDirt, "Dirt");
  const uint16_t oxygen = t.Need(kOxygen, "Oxygen");
  const uint16_t co2 = t.Need(kCarbonDioxide, "CarbonDioxide");
  const uint16_t water = t.Need(kWater, "Water");
  const uint16_t magma = t.Need(kMagma, "Magma");
  const uint16_t vacuum = t.Need(kVacuum, "Vacuum");

  uint32_t rng = 0x9E3779B9u;
  auto next = [&]() {
    rng ^= rng << 13;
    rng ^= rng >> 17;
    rng ^= rng << 5;
    return rng;
  };
  auto frand = [&]() { return static_cast<float>(next() & 0xFFFFFF) / 16777216.0f; };
  auto irand = [&](int lo, int hi) {
    return lo + static_cast<int>(next() % static_cast<uint32_t>(hi - lo + 1));
  };

  SeedWorld w;
  w.Init(width, height, granite, 2000.0f, 300.0f);

  const int surface = height * 62 / 100;  // solid below, open above

  // ------------------------------------------------------------------ solid bulk
  for (int y = 0; y < surface; ++y) {
    // Depth gradient: hotter at the bottom, which also keeps conduction awake — a world
    // that starts isothermal exercises nothing but the 1 K dead zone.
    const float base = 290.0f + 60.0f * (1.0f - static_cast<float>(y) / surface);
    for (int x = 0; x < width; ++x) {
      // Bands rather than per-cell noise: real strata are contiguous, and contiguity is
      // exactly what decides cache behaviour in the neighbour loops.
      const int band = (y / 9 + x / 31) % 4;
      uint16_t e = granite;
      float m = 2000.0f;
      switch (band) {
        case 0: e = granite; m = 2000.0f; break;
        case 1: e = sandstone; m = 1800.0f; break;
        case 2: e = igneous; m = 2200.0f; break;
        default: e = dirt; m = 1600.0f; break;
      }
      w.Set(x, y, e, m, base + 4.0f * frand());
    }
  }
  // Ore veins: small blobs of a high-conductivity metal ore, which is where conduction
  // stops being uniform.
  for (int v = 0; v < (width * surface) / 3000; ++v) {
    const int cx = irand(2, width - 3), cy = irand(2, surface - 3);
    const int r = irand(1, 3);
    for (int y = cy - r; y <= cy + r; ++y) {
      for (int x = cx - r; x <= cx + r; ++x) {
        if ((x - cx) * (x - cx) + (y - cy) * (y - cy) <= r * r) {
          w.Set(x, y, copper, 1800.0f, 300.0f + 10.0f * frand());
        }
      }
    }
  }

  // ------------------------------------------------------------------ caverns
  const int caverns = (width * surface) / 2600;
  for (int c = 0; c < caverns; ++c) {
    const int cx = irand(6, width - 7);
    const int cy = irand(6, surface - 7);
    const int rx = irand(3, 9), ry = irand(2, 6);
    // Gas pressure varies cavern to cavern: two cells of the same gas at different
    // pressures is the case the flow kernel spends its time on.
    const bool is_co2 = (next() & 3) == 0;
    const uint16_t gas = is_co2 ? co2 : oxygen;
    const float pressure = 0.4f + 2.6f * frand();
    const float gas_t = 285.0f + 25.0f * frand();
    // Half the caverns hold standing water in their lower third, giving a free surface.
    const bool flooded = (next() & 1) != 0;
    const int water_top = cy - ry + (2 * ry) / 3;
    for (int y = cy - ry; y <= cy + ry; ++y) {
      for (int x = cx - rx; x <= cx + rx; ++x) {
        const float dx = static_cast<float>(x - cx) / rx;
        const float dy = static_cast<float>(y - cy) / ry;
        if (dx * dx + dy * dy > 1.0f) continue;
        if (x < 1 || y < 1 || x >= width - 1 || y >= surface) continue;
        if (flooded && y <= water_top) {
          w.Set(x, y, water, 950.0f + 60.0f * frand(), 295.0f + 5.0f * frand());
        } else {
          w.Set(x, y, gas, pressure, gas_t);
        }
      }
    }
    // One cavern in six gets magma directly under its floor. Water above rock above magma
    // keeps boiling for the whole run rather than settling after a few ticks.
    if (flooded && (c % 6) == 0) {
      for (int x = cx - rx / 2; x <= cx + rx / 2; ++x) {
        w.Set(x, cy - ry - 1, magma, 1800.0f, 1900.0f);
        w.Set(x, cy - ry - 2, magma, 1800.0f, 1950.0f);
      }
    }
  }

  // ------------------------------------------------------------------ open space
  for (int y = surface; y < height; ++y) {
    // Thinning atmosphere, then vacuum in the top rows.
    const float f = static_cast<float>(y - surface) / (height - surface);
    for (int x = 0; x < width; ++x) {
      if (f > 0.75f) {
        w.Set(x, y, vacuum, 0.0f, 0.0f);
      } else if (((x / 17) & 1) && f > 0.35f) {
        w.Set(x, y, co2, 1.2f * (1.0f - f), 290.0f + 6.0f * frand());
      } else {
        w.Set(x, y, oxygen, 1.9f * (1.0f - f) + 0.05f, 292.0f + 6.0f * frand());
      }
    }
  }
  // A rock ceiling over part of the map, so the open region is not one unobstructed box.
  for (int x = 0; x < width; ++x) {
    if (((x / 23) & 1) == 0) continue;
    const int y = surface + 3 + (x / 23) % 5;
    w.Set(x, y, granite, 2000.0f, 295.0f);
  }
  return w;
}

// ------------------------------------------------------- the building population

// WHY THIS IS READ OUT OF THE CORPUS RATHER THAN INVENTED.
//
// `StepBuildingHeatExchange` and `StepBuildingToBuilding` are the one pair of kernels whose
// cost scales with what the PLAYER built rather than with the size of the map. Pricing them needs a
// realistic population, and "realistic" is exactly the word that turns a benchmark into an
// anecdote when the numbers behind it are guessed.
//
// They are not guessed. The corpus `bench` already loads its element table from is a
// recording of the real message stream, and a save load re-registers the whole base:
// `sim_corpus_744825.bin` carries **1529 `AddBuildingHeatExchange` messages in its first
// 1540 records**, callback indices 0..1528 with no repeat, against exactly ONE
// `RemoveBuildingHeatExchange` in the entire file. That is a real ONI base as the game
// itself described it, and it comes with the element, mass, temperature, conductivity and
// extents of every building in it.
//
// What the recording says, and none of it is what a synthesized population would have been:
//
//   * **mean footprint 1.27 cells** -- 1396 of the 1529 are 1x1, the largest is 5x3. The
//     cost of these kernels is driven by the building COUNT and the per-region extent
//     test, not by area.
//   * **302 footprints carry more than one building.** Co-located, not rebuilt: a wire and
//     a pipe share a tile in ONI, and the pairs read `elements [13, 14]` over and over.
//     Deduplicating by footprint would have thrown away a fifth of the base.
//   * `operating_kilowatts` is 0.0 for all 1529, and so is `overheat_temperature`.
//   * mass 1..400 kg (median 20), conductivity 0.01..2.0 (median 1.0), temperature
//     246.8..320.2 K.
//
// Element indices are indices into the corpus's OWN element table, which is the table
// `bench` loads from the same file, so they need no translation and cannot drift.
struct SeedBuilding {
  uint16_t elem = 0;
  float mass = 0.0f;
  float temperature = 0.0f;
  float thermal_conductivity = 0.0f;
  float overheat_temperature = 0.0f;
  float operating_kilowatts = 0.0f;
  // Game coordinates, half-open, exactly as the message carries them:
  // `cells = (max_x - min_x) * (max_y - min_y)`. `BuildingDataFromMessage` is what pads
  // them, and it is the only thing that should.
  int32_t min_x = 0, min_y = 0, max_x = 0, max_y = 0;
};

// Every `AddBuildingHeatExchange` in the corpus, in the order the game sent them.
//
// The single `RemoveBuildingHeatExchange` is NOT replayed: the message carries a sim handle
// the game allocated, and nothing in the file maps a callback index onto one. So this is an
// upper bound on the live population by exactly one building out of 1529, and it is the
// conservative direction for a benchmark -- it can only price the kernels high.
inline bool LoadBuildings(const char* path, std::vector<SeedBuilding>* out) {
  FILE* f = fopen(path, "rb");
  if (!f) {
    printf("cannot open corpus %s\n", path);
    return false;
  }
  out->clear();
  for (;;) {
    int32_t h[4];
    if (fread(h, sizeof(h), 1, f) != 1) break;
    const size_t stride = static_cast<size_t>(h[2]);
    const size_t count = static_cast<size_t>(h[3] > 0 ? h[3] : 1);
    const size_t bytes = stride * count;
    std::vector<uint8_t> payload(bytes);
    if (bytes && fread(payload.data(), 1, bytes, f) != bytes) break;
    if (h[1] != static_cast<int32_t>(SimMessageHash::AddBuildingHeatExchange)) continue;
    if (stride != sizeof(oni_sim::AddBuildingHeatExchangeMessage)) continue;
    for (size_t i = 0; i < count; ++i) {
      oni_sim::AddBuildingHeatExchangeMessage m{};
      memcpy(&m, payload.data() + i * stride, sizeof(m));
      SeedBuilding b;
      b.elem = m.elemIdx;
      b.mass = m.mass;
      b.temperature = m.temperature;
      b.thermal_conductivity = m.thermalConductivity;
      b.overheat_temperature = m.overheatTemperature;
      b.operating_kilowatts = m.operatingKilowatts;
      b.min_x = m.minX;
      b.min_y = m.minY;
      b.max_x = m.maxX;
      b.max_y = m.maxY;
      out->push_back(b);
    }
  }
  fclose(f);
  return !out->empty();
}

// How a recorded population is stretched to an arbitrary count, and why it is stretched
// rather than randomised.
//
// The recorded base is one mid-game colony. A late-game one has several times as many
// buildings, and the whole point of this scenario is to find out where the two kernels go
// as that number grows -- so there has to be a dial. Turning it must not invent new
// buildings, because the distributions above are the part that is measured; it may only
// place more copies of the ones the recording actually contains.
//
// So: the population is tiled. Copy 0 sits at the coordinates the game recorded, byte for
// byte. Every further copy is translated by a whole number of HALF bbox widths and heights,
// and only offsets that keep the entire bounding box inside the grid are used -- a building
// indexed outside the padded array is an out-of-bounds read, not a slow benchmark.
//
// Half steps rather than whole ones, deliberately: 1529 buildings covering 1937 cells
// inside a 125x91 bounding box is a 17% fill, so whole-box tiling wastes five sixths of the
// grid before it runs out of room. Overlap is not an artifact either -- 302 of the recorded
// footprints already carry more than one building, because ONI stacks a wire, a pipe and a
// tile on one cell. A denser tiling is a denser base, which is what the dial is for.
//
// `dropped` counts buildings that fell outside the grid and were skipped (only possible on a
// grid smaller than the recorded bounding box); `tiles` reports how many distinct offsets
// the placement had available, so a run that wrapped and started stacking says so.
inline std::vector<SeedBuilding> ScaleBuildings(const std::vector<SeedBuilding>& base,
                                                size_t target, int width, int height,
                                                size_t* dropped, size_t* tiles) {
  std::vector<SeedBuilding> out;
  *dropped = 0;
  *tiles = 0;
  if (base.empty() || target == 0) return out;

  int32_t bx0 = base[0].min_x, by0 = base[0].min_y;
  int32_t bx1 = base[0].max_x, by1 = base[0].max_y;
  for (const SeedBuilding& b : base) {
    if (b.min_x < bx0) bx0 = b.min_x;
    if (b.min_y < by0) by0 = b.min_y;
    if (b.max_x > bx1) bx1 = b.max_x;
    if (b.max_y > by1) by1 = b.max_y;
  }
  const int32_t step_x = (bx1 - bx0) / 2 > 0 ? (bx1 - bx0) / 2 : 1;
  const int32_t step_y = (by1 - by0) / 2 > 0 ? (by1 - by0) / 2 : 1;

  // (0, 0) first, so copy 0 is the recording untouched; the rest in a fixed order, so two
  // runs at the same count place the same buildings in the same cells.
  std::vector<std::pair<int32_t, int32_t>> offsets;
  offsets.push_back({0, 0});
  for (int32_t dy = -by0; dy + by1 <= height; dy += step_y) {
    if (dy + by0 < 0) continue;
    for (int32_t dx = -bx0; dx + bx1 <= width; dx += step_x) {
      if (dx + bx0 < 0) continue;
      if (dx == 0 && dy == 0) continue;
      offsets.push_back({dx, dy});
    }
  }
  *tiles = offsets.size();

  out.reserve(target);
  for (size_t i = 0; i < target; ++i) {
    const SeedBuilding& src = base[i % base.size()];
    const std::pair<int32_t, int32_t>& o = offsets[(i / base.size()) % offsets.size()];
    SeedBuilding b = src;
    b.min_x += o.first;
    b.max_x += o.first;
    b.min_y += o.second;
    b.max_y += o.second;
    if (b.min_x < 0 || b.min_y < 0 || b.max_x > width || b.max_y > height ||
        b.max_x <= b.min_x || b.max_y <= b.min_y) {
      ++*dropped;
      continue;
    }
    out.push_back(b);
  }
  return out;
}

}  // namespace oni_bench
