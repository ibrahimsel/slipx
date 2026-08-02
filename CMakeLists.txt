cmake_minimum_required(VERSION 3.20)

# PROJECT_VERSION is the numeric triple only: CMake cannot express a PEP 440
# pre-release suffix. The full string, suffix included, lives in
# include/slipx/version.hpp and is what reaches the run manifest and PyPI.
# tools/version_check.py holds the two in agreement.
project(slipx
  VERSION 0.1.0
  DESCRIPTION "Vehicle dynamics for 1/10-scale autonomous racecars"
  LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(SlipxDeterminism)
include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)          # no GNU dialect; the core has to port
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

slipx_assert_determinism_flags()

option(SLIPX_BUILD_TESTS  "Build the C++ test suites"        ON)
option(SLIPX_BUILD_SIM    "Build slipx_sim"                  ON)
option(SLIPX_BUILD_PYTHON "Build the pybind11 extension"     OFF)
option(SLIPX_WERROR       "Treat warnings as errors"         OFF)

# CORE-01 is verified by construction, not by inspection: this option drops
# every layer above the core from the build, so the "builds and passes its full
# test suite with slipx_schema absent" claim is something CI can execute.
option(SLIPX_CORE_ONLY "Configure slipx_core and nothing else" OFF)
if(SLIPX_CORE_ONLY)
  set(SLIPX_BUILD_SIM OFF)
  set(SLIPX_BUILD_PYTHON OFF)
endif()

if(SLIPX_BUILD_TESTS AND BUILD_TESTING)
  include(SlipxTesting)
endif()

add_subdirectory(src/core/slipx_core)

if(SLIPX_BUILD_SIM)
  add_subdirectory(src/orchestration/slipx_sim)
endif()

if(SLIPX_BUILD_PYTHON)
  add_subdirectory(src/bindings/slipx)
endif()

message(STATUS "SlipX ${PROJECT_VERSION}: build type ${CMAKE_BUILD_TYPE}, "
               "sim=${SLIPX_BUILD_SIM} python=${SLIPX_BUILD_PYTHON} "
               "tests=${SLIPX_BUILD_TESTS}")
