#!/usr/bin/env bash
# Does `abi/sim_ext_api.h` still describe the DLL that was actually built?
#
#   tools/check-ext-api-header.sh [--dll PATH]
#
# A published ABI header that has drifted from its DLL is worse than no header at all: a
# consumer compiles clean and reads garbage. Three of the four things that keep this one
# honest are compiler errors and cost nothing to run (see the header's own "HOW THE TWO
# HEADERS ARE KEPT IN STEP"). This is the fourth, and it is the one no compiler can do,
# because a MISSING declaration is not an error anywhere -- an export added to
# `sim/simdll.cpp` and never published simply is not mentioned, and nothing notices.
#
# So this diffs the two lists in BOTH directions:
#
#   * declared here but not exported  -- the header promises a symbol GetProcAddress will
#                                        not find, which fails at the consumer's first call.
#   * exported but not declared here  -- the surface grew and the published view did not,
#                                        which is how a header becomes a stale document.
#
# The right-hand list is read from the BUILT DLL's export table rather than from a grep of
# the source, because the export table is what a consumer actually binds against and a grep is
# a guess about what the compiler did with the source.
#
# Klei's own 32 exports are excluded by name: the 16 of the sim ABI and the 16 kprofiler exports
# `Klei/KProfilerPlugin.cs` declares (sim/kprofiler.h). They are not ours to
# publish, they are a closed contract, and the list
# is spelled out here rather than pattern-matched so that adding an export that happens to look
# like one of Klei's cannot hide in it.
#
# It also compiles the header standalone, as C and as C++, because "C-compatible" is a claim
# and nothing else in this tree compiles a line of C.
set -uo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
HDR="$ROOT/abi/sim_ext_api.h"
DLL="$ROOT/sim/build/SimDLL.dll"
while [ $# -gt 0 ]; do
  case "$1" in
    --dll) DLL=$2; shift 2 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

OBJDUMP=${OBJDUMP:-x86_64-w64-mingw32-objdump}
CC=${CC:-x86_64-w64-mingw32-gcc}
CXX=${CXX:-x86_64-w64-mingw32-g++}

[ -f "$HDR" ] || { echo "no header at $HDR" >&2; exit 2; }
[ -f "$DLL" ] || { echo "no DLL at $DLL -- run sim/build.sh first" >&2; exit 2; }

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
fail=0

# Klei's closed contract: the 16 symbols the game itself calls, and the 16 kprofiler symbols its
# KProfilerPlugin declares. Not published by us.
cat > "$TMP/klei" <<'KLEI'
SIM_Initialize
SIM_Shutdown
SIM_HandleMessage
SIM_HandleMessages
SIM_BeginSave
SIM_EndSave
SIM_DebugCrash
SYSINFO_Acquire
SYSINFO_Release
ConduitTemperatureManager_Initialize
ConduitTemperatureManager_Shutdown
ConduitTemperatureManager_Add
ConduitTemperatureManager_Remove
ConduitTemperatureManager_Set
ConduitTemperatureManager_Clear
ConduitTemperatureManager_Update
kprofiler_load_plugin
kprofiler_unload_plugin
kprofiler_start_http_control_listener
kprofiler_start_http_data_sender
kprofiler_start_file_data_sender
kprofiler_flush_data_sender
kprofiler_stop_data_sender
kprofiler_start_profiling
kprofiler_stop_profiling
kprofile_record_string
kprofiler_get_thread_uid
kprofiler_set_thread_info
kprofiler_begin_section
kprofiler_end_section
kprofiler_ping
kprofiler_counter
KLEI
sort -o "$TMP/klei" "$TMP/klei"

# What the header declares: every line that opens with the linkage macro, up to the name.
grep -o '^ONI_SIM_API[^(]*[* ]\([A-Za-z_][A-Za-z0-9_]*\)(' "$HDR" \
  | sed 's/.*[* ]\([A-Za-z_][A-Za-z0-9_]*\)($/\1/' | sort -u > "$TMP/declared"

# What the DLL exports, from its name pointer table.
"$OBJDUMP" -p "$DLL" \
  | sed -n '/\[Ordinal\/Name Pointer\] Table/,$p' \
  | awk '/^\t\[/ {print $NF}' | sort -u > "$TMP/exported_all"
comm -23 "$TMP/exported_all" "$TMP/klei" > "$TMP/exported"

n_decl=$(wc -l < "$TMP/declared")
n_exp=$(wc -l < "$TMP/exported")
n_klei=$(comm -12 "$TMP/exported_all" "$TMP/klei" | wc -l)
echo "header declares $n_decl exports; DLL exports $n_exp of ours plus $n_klei of Klei's"

if [ "$n_klei" -ne 32 ]; then
  echo "FAIL: expected Klei's 32 exports in the DLL, found $n_klei -- one of them has been"
  echo "      renamed or dropped, which breaks the game's own contract before it breaks ours"
  comm -13 "$TMP/exported_all" "$TMP/klei" | sed 's/^/        missing: /'
  fail=1
fi

if ! comm -23 "$TMP/declared" "$TMP/exported" | grep -q . ; then :; else
  echo "FAIL: declared in abi/sim_ext_api.h but NOT exported by the DLL --"
  echo "      a consumer's GetProcAddress returns null for each of these:"
  comm -23 "$TMP/declared" "$TMP/exported" | sed 's/^/        /'
  fail=1
fi

if ! comm -13 "$TMP/declared" "$TMP/exported" | grep -q . ; then :; else
  echo "FAIL: exported by the DLL but NOT declared in abi/sim_ext_api.h --"
  echo "      the surface grew and the published view of it did not:"
  comm -13 "$TMP/declared" "$TMP/exported" | sed 's/^/        /'
  fail=1
fi

# "C-compatible" is a claim. C99 is the floor on purpose: the header's own static assertion
# falls back to a negative array bound below C11, and that fallback is only ever exercised
# here.
echo '#include "abi/sim_ext_api.h"' > "$TMP/probe.c"
cp "$TMP/probe.c" "$TMP/probe.cpp"
for std in c99 c11 c17; do
  if "$CC" -x c -std=$std -pedantic -Wall -Wextra -Werror -I"$ROOT" \
        -c "$TMP/probe.c" -o "$TMP/probe_$std.o" 2>"$TMP/err_$std"; then
    echo "ok: header compiles standalone as $std"
  else
    echo "FAIL: header does not compile as $std"; sed 's/^/        /' "$TMP/err_$std"; fail=1
  fi
done
if "$CXX" -x c++ -std=c++17 -pedantic -Wall -Wextra -Werror -I"$ROOT" \
      -c "$TMP/probe.cpp" -o "$TMP/probe_cpp.o" 2>"$TMP/err_cpp"; then
  echo "ok: header compiles standalone as c++17"
else
  echo "FAIL: header does not compile as c++17"; sed 's/^/        /' "$TMP/err_cpp"; fail=1
fi

[ $fail -eq 0 ] && echo "EXT API HEADER: declared surface == exported surface" \
                || echo "EXT API HEADER: FAILURES above"
exit $fail
