// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// CORE-01: no dynamic allocation inside step. Verified, not asserted in prose.
//
// The global operator new below counts allocations while a flag is armed. It
// is a blunt instrument and that is the point: anything that allocates, at any
// depth, through any library, is caught. A std::function stored in a model, a
// std::vector scratch buffer, a std::string built for a log message, an
// exception object: all of them show up here rather than in a user's 20-agent
// run as an unexplained latency spike.
//
// The requirement matters because of who the core is for. Somebody embedding
// slipx_core in a real-time loop, or running 20 agents in lockstep, cannot
// have the physics step reaching for the allocator; that is a shared,
// unbounded-latency resource and it is exactly what makes N instances stop
// being trivially parallel.

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdlib>
#include <new>

#include "slipx/vehicle_model.hpp"
#include "test_support.hpp"

namespace {

// Not atomic and not thread-safe: the counter is armed only around
// single-threaded step loops. Making it atomic would add a synchronised
// operation to every allocation in the test binary for no benefit.
bool g_armed = false;
std::size_t g_allocations = 0;

struct AllocationGuard {
  AllocationGuard() {
    g_allocations = 0;
    g_armed = true;
  }
  ~AllocationGuard() { g_armed = false; }
  std::size_t count() const { return g_allocations; }
};

}  // namespace

// Replacing the global allocation functions is the only portable way to see
// every allocation regardless of which library made it.
void* operator new(std::size_t n) {
  if (g_armed) ++g_allocations;
  if (n == 0) n = 1;
  void* p = std::malloc(n);
  if (p == nullptr) throw std::bad_alloc();
  return p;
}

void* operator new[](std::size_t n) { return ::operator new(n); }

void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

namespace {

using namespace slipx_test;
using slipx::DriveInput;
using slipx::Integrator;
using slipx::StepDiagnostics;
using slipx::Tier;
using slipx::VehicleModel;
using slipx::VehicleState;

constexpr double kDt = kDefaultDt;

// The counter has to be believed before it can be used as evidence.
TEST(AllocationCounter, ActuallyCountsAllocations) {
  std::size_t seen = 0;
  {
    AllocationGuard guard;
    volatile double* p = new double[64];
    p[0] = 1.0;
    seen = guard.count();
    delete[] p;
  }
  EXPECT_GE(seen, 1u);
}

TEST(AllocationCounter, IsDisarmedOutsideTheGuard) {
  { AllocationGuard guard; }
  const std::size_t before = g_allocations;
  volatile double* p = new double[64];
  p[0] = 1.0;
  delete[] p;
  EXPECT_EQ(g_allocations, before) << "the guard must not leak its arming";
}

class NoAlloc
    : public ::testing::TestWithParam<std::tuple<Tier, Integrator>> {};

TEST_P(NoAlloc, StepAllocatesNothing) {
  const Tier tier = std::get<0>(GetParam());
  const Integrator integ = std::get<1>(GetParam());

  // Construction may allocate: it makes a model on the heap. Only the step
  // loop is under test, so the model is built before the guard is armed.
  auto model = VehicleModel::create(tier, reference_params(), integ);
  VehicleState s = travelling(5.0);
  StepDiagnostics diag;

  std::size_t without_diagnostics = 0;
  std::size_t with_diagnostics = 0;
  {
    AllocationGuard guard;
    for (int i = 0; i < 2000; ++i) {
      model->step(s, DriveInput{0.08, 1.0}, kDt, nullptr);
    }
    without_diagnostics = guard.count();
  }
  {
    AllocationGuard guard;
    for (int i = 0; i < 2000; ++i) {
      model->step(s, DriveInput{-0.08, -1.0}, kDt, &diag);
    }
    with_diagnostics = guard.count();
  }

  // Assertions outside the guarded regions: EXPECT_EQ itself allocates on
  // failure, and a test that allocated while explaining that nothing
  // allocated would be its own counterexample.
  EXPECT_EQ(without_diagnostics, 0u);
  EXPECT_EQ(with_diagnostics, 0u) << "diagnostics must be a write, not a build";
}

// Saturated, clipped and standstill paths are the ones most likely to acquire
// an allocation later, because they are where error reporting gets added.
TEST_P(NoAlloc, SaturatedAndDegenerateStepsAllocateNothing) {
  const auto p = reference_params();
  auto model = VehicleModel::create(std::get<0>(GetParam()), p,
                                    std::get<1>(GetParam()));
  StepDiagnostics diag;
  std::size_t count = 0;
  {
    AllocationGuard guard;
    VehicleState s = at_rest();
    for (int i = 0; i < 500; ++i) {          // from standstill
      model->step(s, DriveInput{9.9, 99.0}, kDt, &diag);
    }
    for (int i = 0; i < 500; ++i) {          // full lock, full brakes
      model->step(s, DriveInput{-9.9, -99.0}, kDt, &diag);
    }
    for (int i = 0; i < 500; ++i) {          // against the top-speed clip
      model->step(s, DriveInput{0.0, 99.0}, kDt, &diag);
    }
    count = guard.count();
  }
  EXPECT_EQ(count, 0u);
}

INSTANTIATE_TEST_SUITE_P(
    TiersAndIntegrators, NoAlloc,
    ::testing::Combine(::testing::Values(Tier::L0_Kinematic, Tier::L1_Bicycle),
                       ::testing::Values(Integrator::kRK4,
                                         Integrator::kSemiImplicitEuler)));

// try_create is the path a caller in a constrained environment uses, and it
// must be able to report a refusal without building a string to do it. The
// messages are literals precisely so that this holds.
TEST(NoAllocFactory, RefusalReportsWithoutAllocating) {
  auto bad = reference_params();
  bad.mass = -1.0;

  std::size_t count = 0;
  const char* reason = nullptr;
  {
    AllocationGuard guard;
    auto model = VehicleModel::try_create(Tier::L1_Bicycle, bad,
                                          Integrator::kRK4, &reason);
    (void)model;
    count = guard.count();
  }
  EXPECT_EQ(count, 0u);
  EXPECT_NE(reason, nullptr);
}

}  // namespace
