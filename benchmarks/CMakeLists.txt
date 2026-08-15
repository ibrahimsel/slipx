# The performance benchmarks (M5.9).
#
# A plain executable with no benchmark-library dependency: it has to run in CI
# on five platforms without anybody installing anything, and what it measures
# is coarse enough that a statistical harness would mostly be measuring
# itself.
#
# The ctest case around it checks that it runs and that nothing has regressed
# by an order of magnitude. A tighter gate would fail on a busy shared runner
# for no reason, and a benchmark that cries wolf is one people learn to ignore.

add_executable(slipx_bench slipx_bench.cpp)
target_link_libraries(slipx_bench PRIVATE slipx::sim slipx::scene slipx::sense)
target_compile_definitions(slipx_bench PRIVATE
  SLIPX_EXAMPLE_TRACK_DIR="${CMAKE_SOURCE_DIR}/examples/tracks/paddock_stadium")
slipx_apply_determinism_flags(slipx_bench)

if(SLIPX_BUILD_TESTS AND BUILD_TESTING)
  add_test(NAME Benchmarks.Run COMMAND slipx_bench)
  # Ten times the P1 targets. The point is to catch somebody putting an
  # allocation in step(), not to police a shared runner's mood.
  set_tests_properties(Benchmarks.Run PROPERTIES TIMEOUT 600)
endif()
