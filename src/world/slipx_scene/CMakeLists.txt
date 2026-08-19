# slipx_scene: the track.
#
# Depends on slipx_core and nothing else. It is allowed I/O and allocation,
# which the core is not: reading a centreline is its job precisely so that the
# core never has to know what a file is.
#
# It does not parse the track manifest. The manifest arrives as a plain
# struct, the way VehicleParams does, so this component never learns what YAML
# is and the dependency direction holds by construction (ADR-0003, ADR-0034).

add_library(slipx_scene
  src/broadphase.cpp
  src/centreline.cpp
  src/lap.cpp
  src/projection.cpp
  src/raycast.cpp
  src/track.cpp
)
add_library(slipx::scene ALIAS slipx_scene)

target_include_directories(slipx_scene PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(slipx_scene PUBLIC slipx::core)
target_compile_features(slipx_scene PUBLIC cxx_std_17)
slipx_apply_determinism_flags(slipx_scene)

if(NOT MSVC)
  target_compile_options(slipx_scene PRIVATE -Wall -Wextra -Wpedantic -Wshadow
                                             -Wconversion -Wdouble-promotion)
  if(SLIPX_WERROR)
    target_compile_options(slipx_scene PRIVATE -Werror)
  endif()
endif()

set_target_properties(slipx_scene PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  EXPORT_NAME scene
)

install(TARGETS slipx_scene EXPORT slipxTargets)
install(DIRECTORY include/slipx DESTINATION include)

if(SLIPX_BUILD_TESTS AND BUILD_TESTING)
  add_subdirectory(tests)
endif()
