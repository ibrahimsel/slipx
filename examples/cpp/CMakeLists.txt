# The reference stack: a wall follower and a pure pursuit controller.
#
# In examples/ rather than in a component, because nothing in the library
# depends on them and nothing should. They exist to validate the simulator,
# which they do by driving a lap of the shipped track in the test below; a
# competitive stack is somebody else's project and this is explicitly not one.
#
# This is the only place in the tree that links slipx_scene and slipx_sense
# together, which is the point of ADR-0037: the two never name each other, and
# something above both hands the scene's raycast to the sensor as a function.

add_library(slipx_reference_stack INTERFACE)
target_include_directories(slipx_reference_stack INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(slipx_reference_stack INTERFACE
  slipx::scene slipx::sense slipx::core)

if(SLIPX_BUILD_TESTS AND BUILD_TESTING)
  add_subdirectory(tests)
endif()
