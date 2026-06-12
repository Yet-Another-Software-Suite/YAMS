// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Unit tests for LQRConfig and LQRController — pure math, no hardware required.

#include <frc/system/plant/DCMotor.h>
#include <gtest/gtest.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <cmath>
#include <variant>

#include "yams/math/LQRConfig.hpp"
#include "yams/math/LQRController.hpp"

namespace yams::test {

using namespace math;

// ── Config helpers ────────────────────────────────────────────────────────────

static LQRConfig FlywheelConfig() {
  return LQRConfig{}
      .WithFlywheelSystem(frc::DCMotor::NEO(1), 0.00233, 1.0)
      .WithQElems({3.0})
      .WithRElems({12.0})
      .WithStateStdDevs({3.0})
      .WithMeasurementStdDevs({0.01});
}

static LQRConfig ArmConfig() {
  return LQRConfig{}
      .WithArmSystem(frc::DCMotor::NEO(1), 0.5, 50.0)
      .WithQElems({0.01745, 0.1745})
      .WithRElems({12.0})
      .WithStateStdDevs({0.01745, 0.1745})
      .WithMeasurementStdDevs({0.0001});
}

static LQRConfig ElevatorConfig() {
  return LQRConfig{}
      .WithElevatorSystem(frc::DCMotor::NEO(1), 5.0, 0.0254, 10.0)
      .WithQElems({0.01, 0.1})
      .WithRElems({12.0})
      .WithStateStdDevs({0.01, 0.1})
      .WithMeasurementStdDevs({0.0001});
}

// ── LQRConfig: type accessors ─────────────────────────────────────────────────

TEST(LQRConfigTest, FlywheelSystemSetsType) {
  EXPECT_EQ(FlywheelConfig().GetType(), LQRConfig::LQRType::FLYWHEEL);
}

TEST(LQRConfigTest, ArmSystemSetsType) {
  EXPECT_EQ(ArmConfig().GetType(), LQRConfig::LQRType::ARM);
}

TEST(LQRConfigTest, ElevatorSystemSetsType) {
  EXPECT_EQ(ElevatorConfig().GetType(), LQRConfig::LQRType::ELEVATOR);
}

// ── LQRConfig: period and voltage ────────────────────────────────────────────

TEST(LQRConfigTest, DefaultPeriodIs20ms) {
  EXPECT_NEAR(FlywheelConfig().GetPeriod().value(), 0.020, 1e-9);
}

TEST(LQRConfigTest, WithPeriodStored) {
  EXPECT_NEAR(FlywheelConfig().WithPeriod(10_ms).GetPeriod().value(), 0.010, 1e-9);
}

TEST(LQRConfigTest, DefaultMaxVoltageIs12V) {
  EXPECT_NEAR(FlywheelConfig().GetMaxVoltage().value(), 12.0, 1e-9);
}

TEST(LQRConfigTest, WithMaxVoltageStored) {
  EXPECT_NEAR(FlywheelConfig().WithMaxVoltage(9_V).GetMaxVoltage().value(), 9.0, 1e-9);
}

// ── LQRConfig: loop variant type ─────────────────────────────────────────────

TEST(LQRConfigTest, FlywheelGetLoopIsLoop1) {
  EXPECT_TRUE(std::holds_alternative<LQRConfig::Loop1>(FlywheelConfig().GetLoop()));
}

TEST(LQRConfigTest, ArmGetLoopIsLoop2) {
  EXPECT_TRUE(std::holds_alternative<LQRConfig::Loop2>(ArmConfig().GetLoop()));
}

TEST(LQRConfigTest, ElevatorGetLoopIsLoop2) {
  EXPECT_TRUE(std::holds_alternative<LQRConfig::Loop2>(ElevatorConfig().GetLoop()));
}

// ── LQRController: type and config accessors ─────────────────────────────────

TEST(LQRControllerTest, FlywheelGetType) {
  EXPECT_EQ(LQRController{FlywheelConfig()}.GetType(), LQRConfig::LQRType::FLYWHEEL);
}

TEST(LQRControllerTest, ArmGetType) {
  EXPECT_EQ(LQRController{ArmConfig()}.GetType(), LQRConfig::LQRType::ARM);
}

TEST(LQRControllerTest, ElevatorGetType) {
  EXPECT_EQ(LQRController{ElevatorConfig()}.GetType(), LQRConfig::LQRType::ELEVATOR);
}

TEST(LQRControllerTest, GetConfigIsPresent) {
  EXPECT_TRUE(LQRController{FlywheelConfig()}.GetConfig().has_value());
}

TEST(LQRControllerTest, GetConfigPreservesType) {
  LQRController ctrl{ArmConfig()};
  ASSERT_TRUE(ctrl.GetConfig().has_value());
  EXPECT_EQ(ctrl.GetConfig()->GetType(), LQRConfig::LQRType::ARM);
}

// ── LQRController: flywheel angular velocity ─────────────────────────────────

TEST(LQRControllerTest, Flywheel_BelowSetpoint_PositiveOutput) {
  LQRController ctrl{FlywheelConfig()};
  EXPECT_GT(ctrl.Calculate(0_rad_per_s, 200_rad_per_s).value(), 0.0);
}

TEST(LQRControllerTest, Flywheel_AboveSetpoint_NegativeOutput) {
  LQRController ctrl{FlywheelConfig()};
  // Warm the state estimate up toward 300 rad/s, then request 100 rad/s.
  for (int i = 0; i < 5; ++i) ctrl.Calculate(300_rad_per_s, 300_rad_per_s);
  EXPECT_LT(ctrl.Calculate(300_rad_per_s, 100_rad_per_s).value(), 0.0);
}

TEST(LQRControllerTest, Flywheel_ClampedToMaxVoltage) {
  LQRController ctrl{FlywheelConfig().WithMaxVoltage(12_V)};
  EXPECT_LE(std::abs(ctrl.Calculate(0_rad_per_s, 1e6_rad_per_s).value()), 12.0 + 1e-6);
}

TEST(LQRControllerTest, Flywheel_OutputIsFinite) {
  LQRController ctrl{FlywheelConfig()};
  EXPECT_TRUE(std::isfinite(ctrl.Calculate(0_rad_per_s, 100_rad_per_s).value()));
}

TEST(LQRControllerTest, Flywheel_ResetToSetpoint_ReducesOutput) {
  // Without reset the Kalman state starts at 0, so error = 100 → large output.
  LQRController ctrl_far{FlywheelConfig()};
  auto out_far = ctrl_far.Calculate(0_rad_per_s, 100_rad_per_s);

  // After resetting state to the setpoint, error ≈ 0 → smaller output.
  LQRController ctrl_near{FlywheelConfig()};
  ctrl_near.Reset(0_rad, 100_rad_per_s);
  auto out_near = ctrl_near.Calculate(100_rad_per_s, 100_rad_per_s);

  EXPECT_LT(std::abs(out_near.value()), std::abs(out_far.value()));
}

// ── LQRController: arm angular position ──────────────────────────────────────

TEST(LQRControllerTest, Arm_BelowSetpoint_PositiveOutput) {
  LQRController ctrl{ArmConfig()};
  ctrl.Reset(0_rad, 0_rad_per_s);
  EXPECT_GT(ctrl.Calculate(0_rad, 1_rad, 0_rad_per_s).value(), 0.0);
}

TEST(LQRControllerTest, Arm_AboveSetpoint_NegativeOutput) {
  LQRController ctrl{ArmConfig()};
  ctrl.Reset(2_rad, 0_rad_per_s);
  EXPECT_LT(ctrl.Calculate(2_rad, 1_rad, 0_rad_per_s).value(), 0.0);
}

TEST(LQRControllerTest, Arm_ClampedToMaxVoltage) {
  LQRController ctrl{ArmConfig().WithMaxVoltage(12_V)};
  ctrl.Reset(0_rad, 0_rad_per_s);
  EXPECT_LE(std::abs(ctrl.Calculate(0_rad, 1000_rad, 0_rad_per_s).value()), 12.0 + 1e-6);
}

TEST(LQRControllerTest, Arm_OutputIsFinite) {
  LQRController ctrl{ArmConfig()};
  ctrl.Reset(0_rad, 0_rad_per_s);
  EXPECT_TRUE(std::isfinite(ctrl.Calculate(0_rad, 0.5_rad, 0_rad_per_s).value()));
}

TEST(LQRControllerTest, Arm_ResetToSetpoint_ReducesOutput) {
  LQRController ctrl_far{ArmConfig()};
  ctrl_far.Reset(0_rad, 0_rad_per_s);
  auto out_far = ctrl_far.Calculate(0_rad, 1_rad, 0_rad_per_s);

  LQRController ctrl_near{ArmConfig()};
  ctrl_near.Reset(1_rad, 0_rad_per_s);
  auto out_near = ctrl_near.Calculate(1_rad, 1_rad, 0_rad_per_s);

  EXPECT_LT(std::abs(out_near.value()), std::abs(out_far.value()));
}

// ── LQRController: elevator linear position ──────────────────────────────────

TEST(LQRControllerTest, Elevator_BelowSetpoint_PositiveOutput) {
  LQRController ctrl{ElevatorConfig()};
  ctrl.Reset(0_m, 0_mps);
  EXPECT_GT(ctrl.Calculate(0_m, 0.5_m, 0_mps).value(), 0.0);
}

TEST(LQRControllerTest, Elevator_AboveSetpoint_NegativeOutput) {
  LQRController ctrl{ElevatorConfig()};
  ctrl.Reset(1_m, 0_mps);
  EXPECT_LT(ctrl.Calculate(1_m, 0.5_m, 0_mps).value(), 0.0);
}

TEST(LQRControllerTest, Elevator_ClampedToMaxVoltage) {
  LQRController ctrl{ElevatorConfig().WithMaxVoltage(12_V)};
  ctrl.Reset(0_m, 0_mps);
  EXPECT_LE(std::abs(ctrl.Calculate(0_m, 10000_m, 0_mps).value()), 12.0 + 1e-6);
}

TEST(LQRControllerTest, Elevator_OutputIsFinite) {
  LQRController ctrl{ElevatorConfig()};
  ctrl.Reset(0_m, 0_mps);
  EXPECT_TRUE(std::isfinite(ctrl.Calculate(0_m, 0.5_m, 0_mps).value()));
}

TEST(LQRControllerTest, Elevator_ResetAffectsDirection) {
  LQRController ctrl{ElevatorConfig()};

  ctrl.Reset(0_m, 0_mps);
  auto out_below = ctrl.Calculate(0_m, 0.5_m, 0_mps);

  ctrl.Reset(1_m, 0_mps);
  auto out_above = ctrl.Calculate(1_m, 0.5_m, 0_mps);

  EXPECT_GT(out_below.value(), 0.0);
  EXPECT_LT(out_above.value(), 0.0);
}

// ── LQRController: linear flywheel (m/s) ─────────────────────────────────────

TEST(LQRControllerTest, LinearFlywheel_BelowSetpoint_PositiveOutput) {
  LQRController ctrl{FlywheelConfig()};
  EXPECT_GT(ctrl.Calculate(0_mps, 5_mps).value(), 0.0);
}

TEST(LQRControllerTest, LinearFlywheel_ClampedToMaxVoltage) {
  LQRController ctrl{FlywheelConfig().WithMaxVoltage(12_V)};
  EXPECT_LE(std::abs(ctrl.Calculate(0_mps, 1e6_mps).value()), 12.0 + 1e-6);
}

TEST(LQRControllerTest, LinearFlywheel_OutputIsFinite) {
  LQRController ctrl{FlywheelConfig()};
  EXPECT_TRUE(std::isfinite(ctrl.Calculate(0_mps, 5_mps).value()));
}

// ── LQRController: UpdateConfig ───────────────────────────────────────────────

TEST(LQRControllerTest, UpdateConfig_ChangesType) {
  LQRController ctrl{FlywheelConfig()};
  ASSERT_EQ(ctrl.GetType(), LQRConfig::LQRType::FLYWHEEL);
  ctrl.UpdateConfig(ArmConfig());
  EXPECT_EQ(ctrl.GetType(), LQRConfig::LQRType::ARM);
}

TEST(LQRControllerTest, UpdateConfig_NewMaxVoltageRespected) {
  LQRController ctrl{FlywheelConfig()};
  ctrl.UpdateConfig(FlywheelConfig().WithMaxVoltage(6_V));
  EXPECT_LE(std::abs(ctrl.Calculate(0_rad_per_s, 1e6_rad_per_s).value()), 6.0 + 1e-6);
}

}  // namespace yams::test
