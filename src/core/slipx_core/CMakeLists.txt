# slipx_core: vehicle dynamics.
#
# target_link_libraries is empty and must stay empty. CORE-01 and NFR-06 make
# this the one file in the repository where adding a dependency is a design
# decision rather than a convenience, and tools/dep_lint.py fails CI on any
# include that reaches upward or outward from here.

add_library(slipx_core
  src/l0_kinematic.cpp
  src/l1_bicycle.cpp
  src/params.cpp
  src/vehicle_model.cpp
)
add_library(slipx::core ALIAS slipx_core)

target_include_directories(slipx_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(slipx_core PUBLIC cxx_std_17)
slipx_apply_determinism_flags(slipx_core)

if(NOT MSVC)
  target_compile_options(slipx_core PRIVATE -Wall -Wextra -Wpedantic
                                            -Wshadow -Wconversion
                                            -Wdouble-promotion)
  if(SLIPX_WERROR)
    target_compile_options(slipx_core PRIVATE -Werror)
  endif()
endif()

set_target_properties(slipx_core PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  EXPORT_NAME core
)

install(TARGETS slipx_core EXPORT slipxTargets)
install(DIRECTORY include/slipx DESTINATION include)

if(SLIPX_BUILD_TESTS AND BUILD_TESTING)
  add_subdirectory(tests)
endif()
