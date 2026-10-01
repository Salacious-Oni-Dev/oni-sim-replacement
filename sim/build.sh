#!/usr/bin/env bash
# Build the replacement SimDLL. This does NOT install it over the game — the harness in
# driver/ loads it by path, which is the only thing that should be running it until the
# physics exists.
set -euo pipefail

cd "$(dirname "$0")"
mkdir -p build

# abi/sim_abi.h describes the game's own message structs, so it is generated from the player's
# install rather than distributed. Set ONI_GAME to the game directory for the first build.
if [ ! -f ../abi/sim_abi.h ]; then
  : "${ONI_GAME:?abi/sim_abi.h is missing: set ONI_GAME to your Oxygen Not Included install directory}"
  python3 ../tools/gen_sim_abi.py "$ONI_GAME" -o ../abi/sim_abi.h
fi

# THE VERSION STAMP. MAJOR.MINOR is the one hand-edited part (VERSION, beside this script);
# the revision and the commit come from git so they cannot drift from the history, and a dirty
# working tree gets a trailing `*` so a build made from uncommitted edits says so on screen.
#
# Everything here degrades to a literal rather than failing: this script is the only thing that
# compiles simdll.cpp, and a missing git or a shallow checkout must not be able to stop a build.
# `git rev-list --count` is taken from this repository.
BASE=$(grep -v '^#' VERSION | tr -d '[:space:]')
REV=$(git rev-list --count HEAD 2>/dev/null || echo 0)
SHA=$(git rev-parse --short=7 HEAD 2>/dev/null || echo unknown)
DIRTY=""
[ -n "$(git status --porcelain 2>/dev/null)" ] && DIRTY="*"
# THE BUILD-TIME TUNABLES OVERRIDE (sim/tunables.def, sim/tunables_cfg.py). Unset in every
# ordinary build, which is then the table's own defaults, byte for byte. Set, it names a config of
# `Name = value` lines that become this build's defaults -- what the table starts as and what a
# reset restores -- and the version says so, so a tuned DLL is never mistaken for a stock one.
TUNE_FLAGS=()
TUNED=""
if [ -n "${ONI_TUNABLES_CFG:-}" ]; then
  python3 tunables_cfg.py tunables.def "$ONI_TUNABLES_CFG" build/tunables_override.h
  TUNE_FLAGS=(-DONI_TUNABLES_OVERRIDE_HEADER="\"$(pwd)/build/tunables_override.h\"")
  TUNED=".tuned-$(sha256sum "$ONI_TUNABLES_CFG" | cut -c1-8)"
fi
VERSION="${BASE}.${REV}+${SHA}${TUNED}${DIRTY}"
echo "version: $VERSION"

# Optimisation flags, chosen by measurement: -O3 and a baseline ISA.
#
#   -O3                  inlining and unrolling, not vectorisation. GCC emits zero packed FP
#                        arithmetic here at any -O level, so this is scheduling work, not SIMD.
#                        StepStateChange -54%, Project -51% on granite, StepFlow -9%.
#   -march=x86-64-v2     for exactly one instruction: SSE4.1's blendv, which turns a branch in
#                        StepGasPressure's inner loop into a branchless select. -15% on that
#                        kernel, and -mssse3 and -mpopcnt each buy nothing, so SSE4.1 is the
#                        whole of it. v2 is a 2008-era baseline; v3 measured no faster and
#                        regressed FillPropertyTextures 23%, so the wider baseline is free.
#   -ffp-contract=off    A GUARD, not an optimisation. It is a no-op at v2, which has no FMA.
#                        At v3 or -march=native GCC contracts a*b+c into an FMA, the rounding
#                        moves, and diffsim goes from 0 differing lines to a 39.6 K divergence
#                        and FAILURES. Anyone widening -march must keep this flag.
#
# diffsim --scenario all --ticks 50 is byte-identical to the -O2 build these replaced:
# 0 differing lines, blobs byte identical, crossload accepted both ways.
x86_64-w64-mingw32-g++ -O3 -march=x86-64-v2 -ffp-contract=off -std=c++17 -shared \
  -static-libgcc -static-libstdc++ \
  -Wall -Wextra \
  -DONI_SIM_VERSION_STRING="\"$VERSION\"" \
  "${TUNE_FLAGS[@]}" \
  -o build/SimDLL.dll simdll.cpp \
  -Wl,--out-implib,build/libSimDLL.a \
  -lws2_32

echo "built sim/build/SimDLL.dll  ($VERSION)"
