# Floating-point settings that make NFR-02 achievable.
#
# NFR-02 promises bit-identical results across runs on a fixed (platform,
# compiler, flag set). Three compiler behaviours break that promise, and all
# three are off by default in some compilers and on by default in others, so
# they are set explicitly rather than assumed:
#
#   -ffp-contract=off    stops the compiler fusing a*b+c into an FMA. Fusing is
#                        not a rounding-neutral transformation, and whether it
#                        happens depends on optimisation level and on which
#                        expressions the vectoriser got to first. GCC defaults
#                        this to "fast" for C++, so it must be turned off.
#   -ffast-math          licenses reassociation and finite-math assumptions.
#                        Never permitted, not even in release builds.
#   -march=native        makes the binary depend on the machine that compiled
#                        it, which destroys the reference-hash comparison that
#                        the determinism CI job is built on.
#
# Cross-platform bit-identity is explicitly *not* promised (NFR-03); libm is
# not correctly rounded and differs between platforms. What this file buys is
# reproducibility within one (platform, compiler, flags) triple, which is what
# the replay and leaderboard claims actually rest on.

set(SLIPX_FORBIDDEN_FLAGS "-ffast-math" "-funsafe-math-optimizations"
                          "-march=native" "-mtune=native" "-Ofast" "/fp:fast")

# Fail loudly rather than silently producing a build whose results cannot be
# compared against the reference hashes.
function(slipx_assert_determinism_flags)
  set(_all "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${_build_type}}")
  foreach(_cfg RELEASE DEBUG RELWITHDEBINFO MINSIZEREL)
    set(_all "${_all} ${CMAKE_CXX_FLAGS_${_cfg}}")
  endforeach()
  foreach(_bad IN LISTS SLIPX_FORBIDDEN_FLAGS)
    string(FIND "${_all}" "${_bad}" _pos)
    if(NOT _pos EQUAL -1)
      message(FATAL_ERROR
        "SlipX: ${_bad} is present in CMAKE_CXX_FLAGS. It breaks NFR-02 "
        "(bit-identical replay) and is not permitted in any build "
        "configuration. Remove it or build a fork that does not claim "
        "determinism.")
    endif()
  endforeach()
endfunction()

# Applied to every SlipX target, not only the core: an inlined header from the
# core compiled inside slipx_sim must round the same way it does inside the
# core's own translation units.
function(slipx_apply_determinism_flags target)
  if(MSVC)
    target_compile_options(${target} PUBLIC /fp:precise)
  else()
    target_compile_options(${target} PUBLIC -ffp-contract=off)
  endif()
endfunction()
