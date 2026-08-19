# slipx_core test suite.
#
# The four layers of SRS 7 map onto these files:
#
#   analytical   test_analytical.cpp    closed-form cases with known answers
#   invariant    test_invariants.cpp    symmetry, dissipation, determinism
#   cross-tier   test_cross_tier.cpp    L0 and L1 agree in the linear limit
#   empirical    deferred to P2 and delegated to the registry (SRS 7)
#
# plus the requirement-level tests that are not physics: test_conventions
# (CORE-17), test_no_alloc (CORE-01, CORE-03) and test_integrator (CORE-13).

slipx_add_test(test_math slipx::core)
slipx_add_test(test_params slipx::core)
slipx_add_test(test_conventions slipx::core)
slipx_add_test(test_integrator slipx::core)
slipx_add_test(test_analytical slipx::core)
slipx_add_test(test_invariants slipx::core)
slipx_add_test(test_contact slipx::core)
slipx_add_test(test_cross_tier slipx::core)
slipx_add_test(test_no_alloc slipx::core)
