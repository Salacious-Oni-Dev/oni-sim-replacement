# Offline test tools

These tools drive the SimDLL with no game process running. They are cross-compiled for
Windows x64 with mingw-w64. On WSL they run directly through Windows interop; on other Linux
systems, run them with Wine.

```sh
cd driver
ONI_GAME=/path/to/OxygenNotIncluded ./build.sh   # ONI_GAME is only needed for the first build
```

The first build generates `abi/sim_abi.h` from your own game install (see `tools/gen_sim_abi.py`).

## The tools

| tool | needs | what it checks |
|---|---|---|
| `gastest` | nothing | Compiles the sim's kernels into itself and tests them on synthetic worlds: gas mixing, phase change, fields, raycasts, the published ABI tables. Run this one first; it needs no game files. |
| `diffsim` | a corpus, the game's `SimDLL.dll`, a built `sim/build/SimDLL.dll` | Loads the game's SimDLL and this one into one process, seeds both identically, steps them in lockstep and compares every game-visible array after every tick. It also cross-loads save blobs in both directions. |
| `vftest` | a corpus, a built SimDLL | Drives the built DLL through its real message pipeline and checks the extension surface: the gas mixture, the extension registries, save/load round trips, published streams. |
| `bench` | a corpus | Times the sim's kernels on recorded worlds, and digests the resulting world state. |
| `worldgen_test` | a corpus, a SimDLL | Checks the world-generation load path step by step. |
| `replay`, `experiments`, `savefmt`, `driver` | a corpus | Smaller investigation tools: replay a recorded boot, run conservation experiments, inspect the save-blob format. |
| `kproftest` | a SimDLL | Exercises the profiler exports. |

A tool that needs arguments prints its usage when run without them.

Tools that load the game's own SimDLL look for it as `SimDLL_orig.dll` in the current
directory. Copy it there from `<install>/OxygenNotIncluded_Data/Plugins/x86_64/SimDLL.dll`, or
pass its path: `--klei <path>` for `diffsim`, `--dll <path>` for `replay`, `experiments` and
`savefmt`, and as the first argument for `driver`.

## Recording a corpus

Most tools replay a *corpus*: the real messages the game sent to its SimDLL, recorded
from your own game. The passthrough shim in `shim/` records one:

1. Build it with `shim/build.sh`.
2. In `<install>/OxygenNotIncluded_Data/Plugins/x86_64/`, rename the game's `SimDLL.dll` to
   `SimDLL_orig.dll` and put the built shim there as `SimDLL.dll`.
3. Start the game and load a save as the first thing you do; do not start a new game first.
   The shim records the first simulation session, and a new game's first session is world
   generation. The shim forwards every call to the original and writes `sim_corpus.bin` beside
   it, containing the complete boot sequence plus one sample of every message seen after it.
4. Put the original `SimDLL.dll` back.
5. Move `sim_corpus.bin` out of the folder, and delete `sim_notes.log`, `sim_shim.log` and
   `sim_timing.log`.

Record with no mods installed. A corpus recorded with mods contains messages a stock game
never sends.

## The goldens check themselves

Every suite checks its own reference values against **`GOLDENS.txt`** and exits non-zero when
they change. `GOLDENS.txt` explains what each key means.

**How the `diffsim` digest is taken.** `diffsim` hashes the bytes *it* printed, through one
wrapper that every print goes through. The two loaded SimDLLs print on stderr, and some of
that output changes from run to run (thread ids, for example); none of it goes through the
wrapper. So the digest excludes that output by construction, and the by-hand equivalent is
exact with no filter at all:

```sh
./build/diffsim.exe --corpus sim_corpus.bin --scenario all > out.txt   # NOT 2>&1
md5sum out.txt   # == diffsim.all.digest
wc -l  out.txt   # == diffsim.all.lines
```

**Changing a golden.** Never by hand. `--record-goldens` rewrites that suite's own keys, keeps
every comment in the file where it was, and prints what it wrote. The resulting diff of
`GOLDENS.txt` is what a reviewer reads.

Goldens recorded against one corpus hold only for that corpus. With a corpus you recorded
yourself, record your own goldens first. After that, the suites catch any change you make.

## A pitfall

`diffsim` loads a prebuilt `sim/build/SimDLL.dll`. After editing `sim/*.h`, rebuild the DLL
with `sim/build.sh`, or pass `--mine <dll>`, before trusting a `diffsim` result. Otherwise it
tests the DLL that was already there, and passes.
