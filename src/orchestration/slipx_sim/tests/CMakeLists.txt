# slipx_sim tests. SIM-01 through SIM-09 as they stand at P0; the lockstep
# barrier with acknowledgements and a timeout policy (SIM-05) is P3.

slipx_add_test(test_rng slipx::sim)
slipx_add_test(test_hash slipx::sim)
slipx_add_test(test_manifest slipx::sim)
slipx_add_test(test_simulation slipx::sim)
slipx_add_test(test_determinism slipx::sim)
slipx_add_test(test_snapshot slipx::sim)
slipx_add_test(test_events slipx::sim)
slipx_add_test(test_contact_sim slipx::sim)
slipx_add_test(test_barrier slipx::sim)
slipx_add_test(test_sensor_rig slipx::sim)
slipx_add_test(test_track_world slipx::sim)
target_compile_definitions(test_track_world PRIVATE
  SLIPX_EXAMPLE_TRACK_DIR="${CMAKE_SOURCE_DIR}/examples/tracks/paddock_stadium")
