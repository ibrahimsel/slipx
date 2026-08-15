# The end-to-end lap. CI runs it headlessly, which is the M5.8 exit condition.

slipx_add_test(test_reference_stack slipx_reference_stack)
target_compile_definitions(test_reference_stack PRIVATE
  SLIPX_EXAMPLE_TRACK_DIR="${CMAKE_SOURCE_DIR}/examples/tracks/paddock_stadium")

# Twenty of them at once, orchestrated. Also headless, also in CI.
slipx_add_test(test_ghost_race slipx_ghost_race)
target_compile_definitions(test_ghost_race PRIVATE
  SLIPX_EXAMPLE_TRACK_DIR="${CMAKE_SOURCE_DIR}/examples/tracks/paddock_stadium")
