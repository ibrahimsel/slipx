# slipx_race: race control (ADR-0046).
#
# Above slipx_sim and slipx_scene, below the bindings: the referee needs the
# simulation and the track at once, and neither of those should know what a
# rulebook is. Implements the pinned RoboRacer ruleset revision stated in
# include/slipx/race/ruleset.hpp.

add_library(slipx_race
  src/head_to_head.cpp
  src/obstacle_test.cpp
  src/ruleset.cpp
  src/starts.cpp
  src/time_trial.cpp
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

if(SLIPX_BUILD_TESTS AND BUILD_TESTING)
  add_subdirectory(tests)
endif()
