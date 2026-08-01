# GoogleTest, found once at the top level.
#
# find_package's imported targets are visible only in the directory that called
# it and its children, so locating GoogleTest inside one package's tests
# directory leaves it invisible to every sibling package. It is done here
# instead, and every test subdirectory uses slipx_add_test().
#
# GoogleTest is a test-only dependency. It never appears in slipx_core's link
# line and therefore does not touch the CORE-01 claim; the dependency lint
# (NFR-06) checks that rather than taking it on trust.

find_package(GTest QUIET)

if(NOT GTest_FOUND)
  message(STATUS "slipx: GoogleTest not found locally, fetching it")
  include(FetchContent)
  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

include(GoogleTest)

# slipx_add_test(<name> <library>)
#
# Compiles <name>.cpp from the calling directory against <library> and
# registers every TEST() in it with CTest individually, so a failure names the
# case rather than the binary.
function(slipx_add_test name library)
  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name} PRIVATE ${library} GTest::gtest
                                        GTest::gtest_main)
  # Applied to the test binary too: a header from the core inlined into a test
  # must round exactly as it does inside the library, or the test is measuring
  # a different build than the one being shipped (NFR-02).
  slipx_apply_determinism_flags(${name})
  gtest_discover_tests(${name})
endfunction()
