#!/usr/bin/env bash
# Assemble the end-user package: installer/dist/oni-sim-replacement-<version>.zip
#
#   installer/package.sh
#
# Takes sim/build/SimDLL.dll as built (run sim/build.sh first) and puts it beside install.ps1,
# the two .cmd launchers, supported-builds.txt and README.txt, with SHA256SUMS and VERSION
# written here. The .cmd and .txt files get CRLF line endings, for Notepad and cmd.exe.
#
# The library is built against the message layouts of the game at ONI_GAME. When ONI_GAME is
# set, this refuses to package unless that game's own SimDLL.dll is a build listed in
# supported-builds.txt, so a package cannot list a build it was not built against.
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
root=$(dirname "$here")
die() { echo "package: $*" >&2; exit 1; }

dll=$root/sim/build/SimDLL.dll
[ -f "$dll" ] || die "$dll is missing; run sim/build.sh first"

version=$(sed -n '/^[^#[:space:]]/{p;q}' "$root/sim/VERSION")
[ -n "$version" ] || die "sim/VERSION has no version line"

if [ -n "${ONI_GAME:-}" ]; then
  plugins=$ONI_GAME/OxygenNotIncluded_Data/Plugins/x86_64
  found=
  # The game's own library may sit under the name the installer or a manual install left it.
  for f in SimDLL.dll SimDLL.dll.vanilla SimDLL_orig.dll; do
    [ -f "$plugins/$f" ] || continue
    h=$(sha256sum < "$plugins/$f" | cut -d' ' -f1)
    if grep -q "^$h " "$here/supported-builds.txt"; then found=$f; break; fi
  done
  [ -n "$found" ] || die "no SimDLL.dll under $plugins is a build listed in supported-builds.txt"
  echo "built against: $plugins/$found ($(grep "^$h " "$here/supported-builds.txt" | cut -d' ' -f2))"
else
  echo "package: ONI_GAME is not set, so the supported build list is not checked" >&2
fi

name=oni-sim-replacement-$version
out=$here/dist/$name
rm -rf "$out" "$out.zip"
mkdir -p "$out"

cp "$dll" "$out/SimDLL.dll"
cp "$here/install.ps1" "$out/"
for f in install.cmd uninstall.cmd README.txt supported-builds.txt; do
  sed 's/\r$//; s/$/\r/' "$here/$f" > "$out/$f"
done
printf '%s\r\n' "$version" > "$out/VERSION"
(cd "$out" && sha256sum SimDLL.dll > SHA256SUMS)

(cd "$here/dist" && zip -qr "$name.zip" "$name")
echo "packaged $out.zip"
cat "$out/SHA256SUMS"
