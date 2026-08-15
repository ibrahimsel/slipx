# slipx_scene tests: the centreline parser, the surface to tyre pairing, and
# the projection that lap counting and track limits are both built on.

slipx_add_test(test_centreline slipx::scene)
slipx_add_test(test_track slipx::scene)
slipx_add_test(test_lap slipx::scene)

# The shipped track, so that "a track loads" is asserted against the file
# people actually get rather than against a string in a test. It is the
# generated one: SlipX redistributes no third-party geometry (ADR-0035), so
# there is no real venue in the tree to point at, and there is deliberately
# no network in a test.
foreach(_target test_track test_lap)
  target_compile_definitions(${_target} PRIVATE
    SLIPX_EXAMPLE_TRACK_DIR="${CMAKE_SOURCE_DIR}/examples/tracks/paddock_stadium")
endforeach()
