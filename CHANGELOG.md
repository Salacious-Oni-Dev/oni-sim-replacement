# Changelog

Notable changes to this repository, newest first. All SDK repositories share one version
number per release; see [Compatibility](https://github.com/Salacious-Oni-Dev/oni-sdk-docs/blob/main/guides/compatibility.md).

## 0.1.0-alpha.1 (2026-10-01)

First public release. Supported game build: 744825.

- A replacement `SimDLL.dll` that accepts every message the game sends to its own library and
  publishes the same data back. With no mod using the extensions it aims to behave like the
  game's own library; `diffsim` compares the two scenario by scenario.
- The simulation frame runs on a worker thread of its own, overlapped with the game's managed
  code and rendering, as the game's own library does.
- An extension surface for mods, declared in `abi/`: per-cell gas mixtures, registries for
  extra per-cell properties and per-element attributes, per-frame event streams, phase change
  inside conduit runs, a grid raycast, and mass and energy ledgers. Nothing changes until a mod
  asks for it.
- Gas-mixture temperatures blend by heat capacity (mass × specific heat). Injecting a gas with
  `kInjectGasSpecies`, and a `MassEmission` into a room the gas-mixture layer has taken over
  ([promoted](docs/GAS-MIXTURES.md#ownership-which-engine-simulates-a-cell)), both land at a
  temperature that neither makes nor loses energy. Only a mod that uses gas mixtures promotes a
  room, so the game alone never reaches the second path.
- Checkpoint messages that restore the run state a save cannot carry: the random stream, the
  scheduling counters, the unstable-solid countdowns, the disease-growth remainders, the
  component registries, per-cell extension properties, the radiation field and the visibility
  mask. A restore can skip the load-time state transition (`kSetLoadIsRestore`), so a replayed
  run resumes identically. The same world always produces the same checkpoint bytes.
- Every fixed number the simulation runs on is in one table, `sim/tunables.def`, with the
  game's own values as defaults. A mod can read and set each one while the game runs.
- `CellEnergyCarry`, a tunable for the rounding of `ModifyCellEnergy`. The game writes each
  payment as one float temperature step, so a payment that is small against the cell's heat
  capacity lands too much or too little by the same amount every time. With the row on, each
  cell keeps the remainder and adds it to its next payment; a payment cut short by the overheat
  ceiling or the temperature clamp is still refused. It is off by default, and off is the
  game's own arithmetic, bit for bit. [Energy ledger](docs/LEDGERS.md) field 40 reports the
  energy the carries hold.
- `SIM_Version()` reports the library's version, so a mod or a bug report can tell which
  library ran.
- The game's in-library event profiler, KProfiler, with the game's capture format and HTTP
  control routes on `127.0.0.1`. It also records the simulation's own kernels.
  `tools/kprofile2chrome.py` converts a capture for Chrome's trace viewer.
- The build reads the game's message layouts from an installed copy and checks the size of
  every one, so a game update that changes a layout fails the build.
- Offline tools in `driver/` that run the library with no game process (`gastest`, `diffsim`,
  `vftest`, `bench` and others), and a passthrough library in `shim/` that records the game's
  messages into a test corpus.
- A Windows installer: it refuses unlisted game builds, verifies the package, keeps a checked
  backup of the game's own library, and restores it on uninstall.
- `sim\build.ps1` builds the library on Windows in PowerShell, without WSL or bash. Releases
  are built with `sim/build.sh`; a Windows build is a development build (see the README).
