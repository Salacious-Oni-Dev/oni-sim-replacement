// The custom SimDLL's own identity, as a string it can hand back to managed code.
//
// WHY THIS EXISTS. Nothing in the running game could previously tell you which SimDLL it had
// loaded. A stale DLL is invisible: it exports the same names, answers the same messages, and
// differs only in behaviour -- which is exactly the thing under test when you care. That cost a
// full debugging round on this project (a regression chased through two rigs and three theories
// before the deployed DLL turned out to predate the fix), and the run archives grew an
// `simdll: <md5>` provenance line specifically to make the comparison controlled afterwards.
// This is the same answer moved one step earlier: the DLL says who it is, on screen, before
// anyone has to reason about it.
//
// HOW IT IS SET. build.sh passes -DONI_SIM_VERSION_STRING="0.1.12+abc1234" -- MAJOR.MINOR from
// the VERSION file beside this header, REV from `git rev-list --count HEAD`, build metadata
// from the short commit, plus a trailing `*` when the working tree was dirty. Identical source
// therefore yields an identical DLL, which is what keeps the md5 in a run's provenance.txt a
// valid "did anything actually change" signal. A version counter that ticked once per build
// would have destroyed that property, which is why there is not one.
//
// THE FALLBACK IS DELIBERATELY UGLY. A build that reaches here without the -D did not come
// from build.sh, and the honest thing for it to report is that it does not know rather than a
// plausible-looking number that nothing can be traced back to.
#pragma once

#ifndef ONI_SIM_VERSION_STRING
#define ONI_SIM_VERSION_STRING "0.0.0+nostamp"
#endif

namespace oni_sim {

// The full version string, static storage, valid for the life of the process. Returned
// straight out of SIM_Version() as a `const char*` for the managed side to marshal; it is a
// string literal, so there is nothing to free and no ownership to negotiate.
constexpr const char* kVersionString = ONI_SIM_VERSION_STRING;

}  // namespace oni_sim
