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

# The ghost race: twenty cars driving the track at once, which needs the
# orchestrator as well. It links slipx::sim on top of the stack above, and it
# is a separate target so the reference stack keeps its narrower dependencies.
add_library(slipx_ghost_race INTERFACE)
target_include_directories(slipx_ghost_race INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(slipx_ghost_race INTERFACE
  slipx_reference_stack slipx::sim)

add_executable(slipx_ghost_race_main ghost_race_main.cpp)
set_target_properties(slipx_ghost_race_main PROPERTIES OUTPUT_NAME slipx_ghost_race)
target_link_libraries(slipx_ghost_race_main PRIVATE slipx_ghost_race)
target_compile_definitions(slipx_ghost_race_main PRIVATE
  SLIPX_EXAMPLE_TRACK_DIR="${CMAKE_SOURCE_DIR}/examples/tracks/paddock_stadium")
slipx_apply_determinism_flags(slipx_ghost_race_main)

if(SLIPX_BUILD_TESTS AND BUILD_TESTING)
  add_subdirectory(tests)
endif()
