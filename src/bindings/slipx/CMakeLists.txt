# The pybind11 extension module.
#
# Built by scikit-build-core when a wheel is made, and directly by CMake when
# somebody is working in the repository. Both paths produce the same _slipx
# extension from the same sources; there is no separate setup.py build.

find_package(pybind11 CONFIG QUIET)

if(NOT pybind11_FOUND)
  # Ask the interpreter where its pybind11 is. This is how scikit-build-core
  # supplies the build-time dependency, and it also picks up a pip- or
  # distribution-installed pybind11 when building in place.
  find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
  execute_process(
    COMMAND ${Python3_EXECUTABLE} -m pybind11 --cmakedir
    OUTPUT_VARIABLE _pybind11_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _pybind11_result
    ERROR_QUIET)
  if(NOT _pybind11_result EQUAL 0)
    message(FATAL_ERROR
      "SlipX: SLIPX_BUILD_PYTHON is ON but pybind11 was not found. Install it "
      "(pip install pybind11) or configure with -DSLIPX_BUILD_PYTHON=OFF. The "
      "C++ core does not need it.")
  endif()
  find_package(pybind11 CONFIG REQUIRED PATHS ${_pybind11_dir} NO_DEFAULT_PATH)
endif()

pybind11_add_module(_slipx src/bindings.cpp)
target_link_libraries(_slipx PRIVATE slipx::sim)

# The determinism flags apply here as much as anywhere: a header from the core
# inlined into the extension must round exactly as it does in the library, or
# the hash a Python user computes will not match the one CI publishes (NFR-02).
slipx_apply_determinism_flags(_slipx)

# Where the extension lands.
#
# Under scikit-build-core, into the slipx package inside the wheel. Otherwise,
# next to the package sources in the tree, so that PYTHONPATH pointed at
# src/bindings/slipx finds a working package without an install step. The
# in-tree copy is gitignored.
if(DEFINED SKBUILD)
  install(TARGETS _slipx LIBRARY DESTINATION slipx)
else()
  set_target_properties(_slipx PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/slipx)
endif()
