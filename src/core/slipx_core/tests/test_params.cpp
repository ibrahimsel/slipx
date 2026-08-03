// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// validate() and the factory's refusal behaviour.
//
// The distinction being tested is the architectural one: the core rejects
// parameter sets that describe no possible object, and says nothing about
// whether the car is a legal RoboRacer entry. That question belongs to
// slipx_schema (SCH-03, SCH-04), which the core is not allowed to know exists
// (CORE-01).

#include <gtest/gtest.h>

#include <cstring>

#include "slipx/params.hpp"
#include "slipx/vehicle_model.hpp"
#include "test_support.hpp"

namespace {

using namespace slipx_test;
using slipx::Tier;
using slipx::VehicleModel;
using slipx::VehicleParams;

TEST(Validate, AcceptsTheReferenceSet) {
  EXPECT_EQ(slipx::validate(reference_params()), nullptr);
}

TEST(Validate, AcceptsTheStructDefaults) {
  // The defaults must themselves be a usable car, because "fill in a struct
  // and go" is the whole embedding story (CORE-01).
  EXPECT_EQ(slipx::validate(VehicleParams{}), nullptr);
}

TEST(Validate, RejectsImpossibleMassAndInertia) {
  auto p = reference_params();
  p.mass = 0.0;
  EXPECT_NE(slipx::validate(p), nullptr);

  p = reference_params();
  p.mass = -1.0;
  EXPECT_NE(slipx::validate(p), nullptr);

  p = reference_params();
  p.izz = 0.0;
  EXPECT_NE(slipx::validate(p), nullptr);
}

TEST(Validate, RejectsDegenerateGeometry) {
  auto p = reference_params();
  p.lf = 0.0;  // zero wheelbase divides by zero in every steering equation
  EXPECT_NE(slipx::validate(p), nullptr);

  p = reference_params();
  p.lr = -0.1;
  EXPECT_NE(slipx::validate(p), nullptr);

  p = reference_params();
  p.wheel_radius = 0.0;
  EXPECT_NE(slipx::validate(p), nullptr);
}

// The most common failure is not a wrong number, it is a right number in the
// wrong unit. Two checks exist purely to catch it.
TEST(Validate, CatchesUnitErrors) {
  auto p = reference_params();
  p.h_cog = 60.0;  // millimetres entered as if they were metres
  const char* why = slipx::validate(p);
  ASSERT_NE(why, nullptr);
  EXPECT_NE(std::strstr(why, "metres"), nullptr) << why;

  p = reference_params();
  p.steer_max = 23.0;  // degrees entered as radians
  why = slipx::validate(p);
  ASSERT_NE(why, nullptr);
  EXPECT_NE(std::strstr(why, "degrees"), nullptr) << why;
}

TEST(Validate, DecelMaxIsAMagnitude) {
  auto p = reference_params();
  p.decel_max = -12.0;  // a plausible mistake, given it is a deceleration
  const char* why = slipx::validate(p);
  ASSERT_NE(why, nullptr);
  EXPECT_NE(std::strstr(why, "magnitude"), nullptr) << why;
}

TEST(Validate, RejectsZeroSlipAngleFloor) {
  auto p = reference_params();
  p.v_eps = 0.0;  // reintroduces the standstill singularity
  EXPECT_NE(slipx::validate(p), nullptr);
}

// Zero relaxation length is a division by zero inside relaxation_rate rather
// than a tyre that responds instantly, so it is rejected at the boundary
// instead of being guarded for in the numerical path (CORE-07).
TEST(Validate, RejectsANonPositiveRelaxationLength) {
  auto p = reference_params();
  p.tyre_front.relax_length = 0.0;
  const char* why = slipx::validate(p);
  ASSERT_NE(why, nullptr);
  EXPECT_NE(std::strstr(why, "relax_length"), nullptr) << why;

  p.tyre_front.relax_length = -0.08;
  EXPECT_NE(slipx::validate(p), nullptr);
}

TEST(Validate, SaysNothingAboutCompetitionLegality) {
  // A car twice the legal RoboRacer length. The core builds it without
  // complaint: dimensional legality is SCH-03's job, and somebody embedding
  // the core in their own simulator wants the physics without the rulebook.
  auto p = reference_params();
  p.lf = 0.6;
  p.lr = 0.6;
  p.track_front = 0.9;
  p.track_rear = 0.9;
  EXPECT_EQ(slipx::validate(p), nullptr);
  EXPECT_NE(VehicleModel::create(Tier::L1_Bicycle, p), nullptr);
}

TEST(Factory, TryCreateReportsWhyRatherThanThrowing) {
  auto p = reference_params();
  p.mass = -1.0;

  const char* reason = nullptr;
  auto model = VehicleModel::try_create(Tier::L1_Bicycle, p,
                                        slipx::Integrator::kRK4, &reason);
  EXPECT_EQ(model, nullptr);
  ASSERT_NE(reason, nullptr);
  EXPECT_NE(std::strstr(reason, "mass"), nullptr) << reason;
}

TEST(Factory, CreateThrowsOnBadParameters) {
  auto p = reference_params();
  p.c_alpha_f = 0.0;
  EXPECT_THROW(VehicleModel::create(Tier::L1_Bicycle, p),
               std::invalid_argument);
}

// An unimplemented tier is an error, never a silent substitution: a trajectory
// labelled L3 that is actually L2 is worse than no trajectory at all
// (ADR-0005). L2 landed in P1 and is checked below to be a real tier rather
// than the refusal this case used to assert.
TEST(Factory, UnimplementedTiersAreRefusedNotSubstituted) {
  const char* reason = nullptr;
  EXPECT_EQ(VehicleModel::try_create(Tier::L3_Extended, reference_params(),
                                     slipx::Integrator::kRK4, &reason),
            nullptr);
  ASSERT_NE(reason, nullptr);
  EXPECT_NE(std::strstr(reason, "L3"), nullptr) << reason;

  EXPECT_THROW(VehicleModel::create(Tier::L3_Extended, reference_params()),
               std::invalid_argument);
}

// And L2 is genuinely L2, not L1 wearing the label. The tier it reports and
// the size of its state vector are both checked, because the second is the one
// that could not be faked by a substitution.
TEST(Factory, L2IsBuiltAndIsNotL1) {
  auto model = VehicleModel::create(Tier::L2_DoubleTrack, reference_params());
  ASSERT_NE(model, nullptr);
  EXPECT_EQ(model->tier(), Tier::L2_DoubleTrack);

  auto l1 = VehicleModel::create(Tier::L1_Bicycle, reference_params());
  EXPECT_GT(model->state_dimension(), l1->state_dimension());
}

TEST(Factory, ModelRemembersHowItWasBuilt) {
  auto p = reference_params();
  p.mass = 4.25;
  auto model = VehicleModel::create(Tier::L1_Bicycle, p,
                                    slipx::Integrator::kSemiImplicitEuler);

  EXPECT_EQ(model->tier(), Tier::L1_Bicycle);
  EXPECT_EQ(model->integrator(), slipx::Integrator::kSemiImplicitEuler);
  EXPECT_DOUBLE_EQ(model->params().mass, 4.25);
  EXPECT_EQ(model->state_dimension(), 6u);

  auto l0 = VehicleModel::create(Tier::L0_Kinematic, p);
  EXPECT_EQ(l0->state_dimension(), 4u);
  EXPECT_EQ(l0->integrator(), slipx::Integrator::kRK4) << "RK4 is the default";
}

// The parameters are copied at construction, not referenced. A caller's struct
// going out of scope must not turn a running simulation into a dangling read.
TEST(Factory, ParametersAreCopiedNotAliased) {
  std::unique_ptr<VehicleModel> model;
  {
    auto p = reference_params();
    p.mass = 7.5;
    model = VehicleModel::create(Tier::L1_Bicycle, p);
    p.mass = 999.0;  // mutating the caller's copy must not reach the model
  }
  EXPECT_DOUBLE_EQ(model->params().mass, 7.5);
}

TEST(Provenance, DefaultsToProvisional) {
  // NFR-08: nothing may present an unmeasured parameter set as measured, and
  // the safe default is the weakest claim.
  EXPECT_EQ(VehicleParams{}.provenance, slipx::Provenance::kProvisional);
  EXPECT_EQ(reference_params().provenance, slipx::Provenance::kProvisional);
}

}  // namespace
