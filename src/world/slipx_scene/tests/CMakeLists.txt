# slipx_scene tests, first slice: the centreline parser and the surface to
# tyre pairing. Lap counting and track limits are the next slice and have no
# tests here yet.

slipx_add_test(test_centreline slipx::scene)
slipx_add_test(test_track slipx::scene)

# The shipped track, so that "a track loads" is asserted against the file
# people actually get rather than against a string in a test. It is the
# generated one: SlipX redistributes no third-party geometry (ADR-0035), so
# there is no real venue in the tree to point at, and there is deliberately
# no network in a test.
target_compile_definitions(test_track PRIVATE
  SLIPX_EXAMPLE_TRACK_DIR="${CMAKE_SOURCE_DIR}/examples/tracks/paddock_stadium")
