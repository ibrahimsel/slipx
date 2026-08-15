# slipx_sense: sensor models.
#
# Depends on slipx_core and nothing else, and in particular NOT on
# slipx_scene: the two are siblings and neither includes the other
# (ADR-0037). A sensor model is about timing, noise and dropouts; what a ray
# hits arrives as a function the caller supplies, so the same LiDAR can be
# pointed at a track, at an occupancy grid, or at a circle in a test whose
# answer has a closed form.
#
# It also owns the seeded generator, because sensing is the lowest C++ layer
# above the core that needs randomness. slipx_sim includes it from here.

add_library(slipx_sense
  src/encoder.cpp
  src/imu.cpp
  src/lidar.cpp
)
add_library(slipx::sense ALIAS slipx_sense)

target_include_directories(slipx_sense PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(slipx_sense PUBLIC slipx::core)
target_compile_features(slipx_sense PUBLIC cxx_std_17)
slipx_apply_determinism_flags(slipx_sense)

if(NOT MSVC)
  target_compile_options(slipx_sense PRIVATE -Wall -Wextra -Wpedantic -Wshadow
                                             -Wconversion -Wdouble-promotion)
  if(SLIPX_WERROR)
    target_compile_options(slipx_sense PRIVATE -Werror)
  endif()
endif()

set_target_properties(slipx_sense PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  EXPORT_NAME sense
)

install(TARGETS slipx_sense EXPORT slipxTargets)
install(DIRECTORY include/slipx DESTINATION include)

if(SLIPX_BUILD_TESTS AND BUILD_TESTING)
  add_subdirectory(tests)
endif()
