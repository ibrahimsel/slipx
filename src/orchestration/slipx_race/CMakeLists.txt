# slipx_race: race control (ADR-0046).
#
# Above slipx_sim and slipx_scene, below the bindings: the referee needs the
# simulation and the track at once, and neither of those should know what a
# rulebook is. Implements the pinned RoboRacer ruleset revision stated in
# include/slipx/race/ruleset.hpp.

add_library(slipx_race
  src/event_stream.cpp
  src/head_to_head.cpp
  src/leaderboard.cpp
  src/obstacle_test.cpp
  src/ruleset.cpp
  src/starts.cpp
  src/time_trial.cpp
  src/wrong_way.cpp
)
add_library(slipx::race ALIAS slipx_race)

target_include_directories(slipx_race PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(slipx_race PUBLIC slipx::sim slipx::scene)
target_compile_features(slipx_race PUBLIC cxx_std_17)
slipx_apply_determinism_flags(slipx_race)

if(NOT MSVC)
  target_compile_options(slipx_race PRIVATE -Wall -Wextra -Wpedantic -Wshadow
                                            -Wconversion -Wdouble-promotion)
  if(SLIPX_WERROR)
    target_compile_options(slipx_race PRIVATE -Werror)
  endif()
endif()

set_target_properties(slipx_race PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  EXPORT_NAME race
)

install(TARGETS slipx_race EXPORT slipxTargets)
install(DIRECTORY include/slipx DESTINATION include)

# The canned deterministic match, written out as an event stream. The pytest
# suite runs it and reads the file back with the reference mcap library,
# which is what holds the "one format, not two" claim.
add_executable(slipx_race_demo tools/race_demo.cpp)
target_link_libraries(slipx_race_demo PRIVATE slipx::race)
target_compile_definitions(slipx_race_demo PRIVATE
  SLIPX_EXAMPLE_TRACK_DIR="${CMAKE_SOURCE_DIR}/examples/tracks/paddock_stadium")
slipx_apply_determinism_flags(slipx_race_demo)
install(TARGETS slipx_race_demo EXPORT slipxTargets)

# The CI leaderboard harness: a seeded round-robin whose standings are
# computed from the event streams read back from disk, so a batch is
# reproducible from its manifest and seeds alone.
add_executable(slipx_leaderboard tools/leaderboard_main.cpp)
target_link_libraries(slipx_leaderboard PRIVATE slipx::race)
target_compile_definitions(slipx_leaderboard PRIVATE
  SLIPX_EXAMPLE_TRACK_DIR="${CMAKE_SOURCE_DIR}/examples/tracks/paddock_stadium")
slipx_apply_determinism_flags(slipx_leaderboard)

if(SLIPX_BUILD_TESTS AND BUILD_TESTING)
  add_subdirectory(tests)
  # The harness end to end, the way CI would run it (mirrors Benchmarks.Run).
  add_test(NAME Leaderboard.Run
           COMMAND slipx_leaderboard ${CMAKE_CURRENT_BINARY_DIR}/leaderboard_run)
  set_tests_properties(Leaderboard.Run PROPERTIES TIMEOUT 600)
endif()
