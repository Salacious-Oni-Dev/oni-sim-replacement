# oni-sim-replacement

A replacement for `SimDLL.dll`, the native simulation library of Oxygen Not Included.

It is an independently implemented simulation library focused on behavioral compatibility with
the game's simulation interfaces and behavior. Its implementation is informed by analysis of the
shipped game's observable behavior and public interfaces. This repository contains its own
source code and does not include or distribute Klei's source code, game binaries, or
proprietary game assets.

Where a comment names one of the game's own functions, types or globals (written as Klei's
`UpdateLiquid`, for example), the name is the one in `SimDLL.pdb`, the symbol file Klei ships
with the game beside `SimDLL.dll`, or in the game's managed code.

This is one part of the SDK. The managed framework API that mods call is in
[oni-framework-api](https://github.com/Salacious-Oni-Dev/oni-framework-api), the example mods
built on it are in [oni-flagship-mods](https://github.com/Salacious-Oni-Dev/oni-flagship-mods),
and the guides, including
[installing and removing](https://github.com/Salacious-Oni-Dev/oni-sdk-docs/blob/main/guides/installing.md),
are in [oni-sdk-docs](https://github.com/Salacious-Oni-Dev/oni-sdk-docs).

**Status: alpha.** Interfaces can still change between releases.

**Windows only.** The simulation library is a Windows DLL, so the SDK needs the game's Windows
version.

## What it does

- **Runs the game's simulation.** It accepts every message the game sends to its own SimDLL and
  publishes the same data back. With no mod using the extensions, it aims to behave exactly
  like the game's own library. The `diffsim` tool compares the two, scenario by scenario, and
  reports any known difference as `KNOWN GAP`.
- **Adds an extension surface** for mods: per-cell gas mixtures (several gases in one cell),
  registries for extra per-cell properties and per-element attributes, per-frame event streams,
  phase change inside conduit runs, a grid raycast, and mass and energy ledgers. Nothing changes
  until a mod asks for it.

The extension surface is declared in `abi/`:

| file | contents |
|---|---|
| `abi/sim_ext_api.h` | the C ABI of every added export and message, in plain C |
| `abi/sim_abi_ext.h` | the extension message ids and payload layouts |
| `abi/gas_mixture_abi.h` | the per-cell gas mixture layout |

## What gets replaced, and how to undo it

Installing replaces one file in the game:

    <install>/OxygenNotIncluded_Data/Plugins/x86_64/SimDLL.dll

No other game file is changed; a backup of the original and a small record of what was
installed are kept beside it. To go back to the original, put Klei's `SimDLL.dll` back, or use
Steam's **Verify integrity of game files**, which restores it. A game update also restores it.
To use the library again after an update, install a release that supports the new game build,
or rebuild from source (see below), because the build checks the game's message layouts.

Because you are replacing a native library, you should be able to check what you run. The full
source is here, and you can build the DLL yourself.

## Building

Requirements:
- mingw-w64 (`x86_64-w64-mingw32-g++`)
- bash, or Windows PowerShell (see Building on Windows below)
- Python 3.8 or later
- an installed copy of the game

On Linux or WSL:

```sh
ONI_GAME=/path/to/OxygenNotIncluded ./sim/build.sh
```

The output is `sim/build/SimDLL.dll`. `ONI_GAME` is needed only for the first build.

The game's own message structs are part of the game, so this repository does not carry them.
On the first build, `tools/gen_sim_abi.py` reads their layouts from your installed
`Assembly-CSharp.dll` and writes `abi/sim_abi.h`. It reads only .NET metadata: type names,
field types, packing and enum constants. Every generated struct carries a size check, so a game
update that changes a layout fails the build instead of corrupting messages at run time. To
regenerate after an update, delete `abi/sim_abi.h` and build again with `ONI_GAME` set.

### Building on Windows

`sim\build.ps1` does what `sim/build.sh` does, in Windows PowerShell, without WSL or bash. It
builds `sim\build\SimDLL.dll`, and on the first build it generates `abi\sim_abi.h` from your
game install.

#### Tools

Install these once, from PowerShell, then open a new PowerShell window so they are on `PATH`:

```powershell
winget install BrechtSanders.WinLibs.POSIX.UCRT   # g++ (MinGW-w64)
winget install Python.Python.3.12                 # Python 3, for the first build only
winget install Git.Git                            # optional: puts the commit in the version stamp
```

If `python` opens the Microsoft Store instead of running, that is the Store placeholder, not
Python: install Python with the `winget` line above. Without Git the build still works, and the
version ends in `+unknown`.

Windows does not run PowerShell scripts by default. Either allow your own scripts once:

```powershell
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
```

or start each build with `powershell -ExecutionPolicy Bypass -File <script>` as shown below.

#### Build

```powershell
git clone https://github.com/Salacious-Oni-Dev/oni-sim-replacement.git
cd oni-sim-replacement
powershell -ExecutionPolicy Bypass -File sim\build.ps1 -OniGame "C:\Program Files (x86)\Steam\steamapps\common\OxygenNotIncluded"
```

`-OniGame` is the folder that contains `OxygenNotIncluded_Data`. You can set it once for the
session instead: `$env:ONI_GAME = "C:\Program Files (x86)\Steam\steamapps\common\OxygenNotIncluded"`.
It is only needed while `abi\sim_abi.h` does not exist yet.

The last line names the build, for example `built sim/build/SimDLL.dll  (0.1.1+1a2b3c4)`. A
trailing `*` means the working tree had uncommitted changes.

The Windows build adds `-static` to the compiler flags so that `SimDLL.dll` does not depend on
`libwinpthread-1.dll`, which the WinLibs compiler would otherwise link dynamically and which the
game does not ship. The other flags are those of `sim/build.sh`.

Releases of `SimDLL.dll` are built with `sim/build.sh`. A Windows build links a different C
runtime (UCRT rather than msvcrt), and its results have not been compared with the release
build's tick for tick, so treat it as a development build: use it to work on the code, and
play with the released `SimDLL.dll`.

## Installing

Most players get the library through the framework mod, `oni-framework-api`. The framework
carries a built `SimDLL.dll` and, with the player's consent, puts it in place each time the game
starts and puts the game's own back at quit, using the same `supported-builds.txt` as the
installer below. It leaves alone a library installed with the installer or by hand. The rest of
this section is for installing the library without it.

On Windows, use the installer. A release zip holds it together with a built `SimDLL.dll`; to
make the same zip from your own build, run `installer/package.sh` after `sim/build.sh` (with
`ONI_GAME` set, it also checks that the game you built against is a supported build).

1. Close the game.
2. Double-click `install.cmd`. To remove the library again, double-click `uninstall.cmd`.

The installer finds the game through Steam, or takes `-GamePath`. It:

- refuses any game build it does not list in `installer/supported-builds.txt`, which holds the
  SHA-256 of the game's own `SimDLL.dll` for each supported build;
- checks the packaged `SimDLL.dll` against `SHA256SUMS`;
- keeps the game's own library as `SimDLL.dll.vanilla`, and checks the copy byte for byte before
  replacing anything;
- records what it installed in `SimDLL.dll.sdk`, so that uninstalling never restores an older
  build's backup over a game update.

`install.ps1 -Status` reports what is installed and changes nothing. `installer/README.txt` is
the full description, and ships in the zip.

If you build against a game build that is not listed, add a line for it to
`installer/supported-builds.txt`: the SHA-256 of that build's own `SimDLL.dll`, then its build
number.

To install by hand instead:

1. Close the game.
2. In `<install>/OxygenNotIncluded_Data/Plugins/x86_64/`, rename `SimDLL.dll` to
   `SimDLL.dll.vanilla`.
3. Copy `sim/build/SimDLL.dll` into that folder.

To uninstall by hand, delete the copied `SimDLL.dll` and rename `SimDLL.dll.vanilla` back, or
verify the game files in Steam. `uninstall.cmd` also undoes a manual install of the library
from the same release.

## Testing

`driver/` holds the offline test tools. They run the DLL with no game process. `gastest` needs
nothing but the build. The other tools replay a corpus of real game messages, which you record
from your own game with the passthrough shim in `shim/`. See `driver/README.md`.

## Documentation

| document | covers |
|---|---|
| [Extension points](docs/EXTENSION-POINTS.md) | the frame's published phase order, every extension message, and the rules an extension follows |
| [Extension registries](docs/EXT-REGISTRY.md) | per-cell properties, per-element attributes, event streams and fields: storing a mod's own data in the simulation |
| [Gas mixtures](docs/GAS-MIXTURES.md) | the optional layer that lets a cell hold several gases at once |
| [Projection](docs/PROJECTION.md) | how simulation state becomes the arrays and textures the game reads each frame |
| [Save format](docs/SAVE-FORMAT.md) | the save blob, its versions, what a load does, and what a checkpoint needs beyond it |
| [Conservation ledgers](docs/LEDGERS.md) | the mass and energy books, and where the game's own physics does not conserve |
| [Threading](docs/THREADING.md) | the frame worker, which calls wait for it, and why it does not change results |
| [KProfiler](docs/KPROFILER.md) | the in-DLL event profiler, its byte stream, and how to take and read a capture |
| [Physics](docs/PHYSICS.md) | conduction, gas and liquid flow, the post-process pass and its random stream, state changes, sublimation, and building heat exchange |
| [Quirks](docs/QUIRKS.md) | measured behaviour of the game's simulation that is surprising or easy to get wrong, and the one deliberate difference |
| [Message drain](docs/DRAIN.md) | when a message takes effect, the order categories drain in, and the cell-energy, insulation and strength handlers |
| [ModifyCell](docs/CELLMOD.md) | the add, remove and replace paths, the deletion tail, and the liquid displacement primitives |
| [Emitters and consumers](docs/EMITTERS.md) | element and disease emitters and consumers, `MassConsumption`, `MassEmission` and `ConsumeDisease` |
| [Element chunks](docs/CHUNKS.md) | heat exchange between off-grid matter and the cell it sits in |
| [Disease](docs/DISEASE.md) | how germs spread, grow, die, and travel with the matter that carries them |
| [Radiation](docs/RADIATION.md) | the radiation field, its decay and sources, and radiation emitters |
| [Conduit temperatures](docs/CONDUITS.md) | pipe contents against their building, conduit runs, and the optional run policies |
| [Cluster messages](docs/CLUSTER.md) | world zones, world offsets, the sunlight texture, and opening a new world in the grid |
| [Backwall](docs/BACKWALL.md) | the material behind a cell and when the simulation reports it out of range |

## Layout

| path | contents |
|---|---|
| `sim/` | the simulation library |
| `abi/` | the extension ABI headers (`sim_abi.h` is generated, not committed) |
| `shim/` | a passthrough SimDLL that records the game's messages into a test corpus |
| `driver/` | offline test tools and their reference results (`GOLDENS.txt`) |
| `tests/fixtures/` | save blobs from earlier builds, used to test loading old saves |
| `tools/` | the header generators, a header consistency check, and a KProfiler capture reader |
| `installer/` | the Windows installer, its launchers, and the script that packages it with a build |

## License

Mozilla Public License 2.0, see `LICENSE`. If you distribute a modified build of this DLL, you
must publish the source of the files you changed. The license applies file by file, so a mod
that loads or calls the DLL is not affected by it.

## Credits

Oxygen Not Included is developed and published by Klei Entertainment. This project is not
affiliated with or endorsed by Klei.

Development of this project uses AI coding assistants. All changes are reviewed and released
by the maintainer.
