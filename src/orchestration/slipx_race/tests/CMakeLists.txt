# slipx_race tests: one scenario suite per race procedure, which is the
# milestone's own acceptance bar.

slipx_add_test(test_starts slipx::race)
slipx_add_test(test_time_trial slipx::race)
slipx_add_test(test_obstacle_test slipx::race)
slipx_add_test(test_head_to_head slipx::race)
slipx_add_test(test_event_stream slipx::race)
slipx_add_test(test_leaderboard slipx::race)

foreach(_target test_starts test_time_trial test_obstacle_test
        test_head_to_head test_event_stream test_leaderboard)
  target_compile_definitions(${_target} PRIVATE
    SLIPX_EXAMPLE_TRACK_DIR="${CMAKE_SOURCE_DIR}/examples/tracks/paddock_stadium")
endforeach()
