Replacement simulation library for Oxygen Not Included
======================================================

This package replaces one file in the game:

    <game>\OxygenNotIncluded_Data\Plugins\x86_64\SimDLL.dll

That library runs the world simulation. Replacing it is what lets mods built on the framework
use simulation features the game's own library does not have. Nothing else in the game is
changed. The framework and the mods are installed separately, as ordinary mods.

The source of this library is public, under the MPL-2.0:
    https://github.com/Salacious-Oni-Dev/oni-sim-replacement


Before installing: check the download
-------------------------------------

Compare the SHA-256 of SimDLL.dll with the one published on the release page. In PowerShell:

    Get-FileHash .\SimDLL.dll -Algorithm SHA256

If the two differ, do not install the file. The installer also checks SimDLL.dll against
SHA256SUMS in this folder, which catches a damaged download.


Installing
----------

1. Close the game.
2. Double-click install.cmd.

The installer finds the game through Steam. If it cannot, run it from PowerShell with the
game's folder:

    .\install.ps1 -GamePath "D:\SteamLibrary\steamapps\common\OxygenNotIncluded"

It then:

- checks that your game build is one this release supports (supported-builds.txt). On any other
  build it stops without changing anything;
- keeps the game's own library as SimDLL.dll.vanilla, and checks that the copy is identical;
- puts the replacement in place and checks it.

If Windows refuses to write to the game's folder, right-click install.cmd and choose
"Run as administrator".


Uninstalling
------------

1. Close the game.
2. Double-click uninstall.cmd.

This puts the game's own SimDLL.dll back and removes the backup. Steam can do the same: in the
game's Properties, open Installed Files and choose "Verify integrity of game files".

Keep a copy of any save made while the replacement was installed. A save in which a mod stored
extension data uses a newer save format, which the game's own library is not built to read.


After a game update
-------------------

A game update puts the game's own SimDLL.dll back, so the replacement is simply gone and the
game runs unmodified. Install again only with a release that supports the new build; this
installer refuses a build it does not list.


Other options
-------------

    .\install.ps1 -Status      show what is installed, and change nothing
    .\install.ps1 -Force       install on a build that is not listed (not recommended)
