# slipx_sim: orchestration.
#
# Depends on slipx_core and nothing else. It is allowed I/O and allocation,
# which the core is not: writing a manifest is its job precisely so that the
# core never has to know what a file is.

# Captured at configure time and hashed into every manifest (SIM-06, NFR-02).
find_package(Git QUIET)
set(SLIPX_GIT_SHA "unknown")
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
  # --verify, and the result is checked: in a repository with no commits yet,
  # rev-parse prints the literal string "HEAD" and exits non-zero, which would
  # otherwise be recorded as though it were a real revision.
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --verify HEAD
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    OUTPUT_VARIABLE _slipx_git_sha
    RESULT_VARIABLE _slipx_git_result
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
  if(_slipx_git_result EQUAL 0 AND NOT _slipx_git_sha STREQUAL "")
    set(SLIPX_GIT_SHA "${_slipx_git_sha}")
  endif()
  # A dirty tree is recorded, not hidden: a published result whose source
  # cannot be recovered from its SHA has to say so on its face.
  execute_process(
    COMMAND ${GIT_EXECUTABLE} status --porcelain --untracked-files=no
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    OUTPUT_VARIABLE _slipx_git_dirty
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
  if(NOT _slipx_git_dirty STREQUAL "" AND NOT SLIPX_GIT_SHA STREQUAL "unknown")
    set(SLIPX_GIT_SHA "${SLIPX_GIT_SHA}-dirty")
  endif()
endif()

string(TOUPPER "${CMAKE_BUILD_TYPE}" _slipx_build_type_upper)
set(SLIPX_EFFECTIVE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${_slipx_build_type_upper}} -ffp-contract=off")
string(STRIP "${SLIPX_EFFECTIVE_CXX_FLAGS}" SLIPX_EFFECTIVE_CXX_FLAGS)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/slipx/sim/build_info.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/generated/slipx/sim/build_info.hpp
  @ONLY)

add_library(slipx_sim
  src/libc_identity.cpp
  src/manifest.cpp
  src/sensor_rig.cpp
  src/simulation.cpp
)
add_library(slipx::sim ALIAS slipx_sim)

target_include_directories(slipx_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
  $<INSTALL_INTERFACE:include>
)
# slipx::sense for the seeded generator, which lives at the lowest layer
# above the core that needs it (ADR-0037).
target_link_libraries(slipx_sim PUBLIC slipx::core slipx::sense)
target_compile_features(slipx_sim PUBLIC cxx_std_17)
slipx_apply_determinism_flags(slipx_sim)

if(NOT MSVC)
  target_compile_options(slipx_sim PRIVATE -Wall -Wextra -Wpedantic -Wshadow
                                           -Wconversion -Wdouble-promotion)
  if(SLIPX_WERROR)
    target_compile_options(slipx_sim PRIVATE -Werror)
  endif()
endif()

set_target_properties(slipx_sim PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  EXPORT_NAME sim
)

# The determinism conformance binary (NFR-02) and the P0 exit gate.
add_executable(slipx_conformance tools/conformance.cpp)
target_link_libraries(slipx_conformance PRIVATE slipx::sim)
slipx_apply_determinism_flags(slipx_conformance)

install(TARGETS slipx_sim slipx_conformance EXPORT slipxTargets)
install(DIRECTORY include/slipx DESTINATION include)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/generated/slipx/sim/build_info.hpp
  DESTINATION include/slipx/sim)

if(SLIPX_BUILD_TESTS AND BUILD_TESTING)
  add_subdirectory(tests)
endif()
