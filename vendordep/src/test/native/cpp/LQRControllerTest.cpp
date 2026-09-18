// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Unit tests for LQRConfig and LQRController — pure math, no hardware required.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <variant>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/time.hpp>
#include <wpi/units/velocity.hpp>
#include <wpi/units/voltage.hpp>

#include "yams/math/LQRConfig.hpp"
#include "yams/math/LQRController.hpp"

namespace yams::test {

using namespace math;

// ── Config helpers ────────────────────────────────────────────────────────────

static LQRConfig FlywheelConfig() {
  return LQRConfig{}
      .WithFlywheelSystem(wpi::math::DCMotor::NEO(1), 0.00233, 1.0)
      .WithQElems({3.0})
      .WithRElems({12.0})
      .WithStateStdDevs({3.0})
      .WithMeasurementStdDevs({0.01});
}

static LQRConfig ArmConfig() {
  return LQRConfig{}
      .WithArmSystem(wpi::math::DCMotor::NEO(1), 0.5, 50.0)
      .WithQElems({0.01745, 0.1745})
      .WithRElems({12.0})
      .WithStateStdDevs({0.01745, 0.1745})
      .WithMeasurementStdDevs({0.0001});
}

static LQRConfig ElevatorConfig() {
  return LQRConfig{}
      .WithElevatorSystem(wpi::math::DCMotor::NEO(1), 5.0, 0.0254, 10.0)
      .WithQElems({0.01, 0.1})
      .WithRElems({12.0})
      .WithStateStdDevs({0.01, 0.1})
      .WithMeasurementStdDevs({0.0001});
}

// ── LQRConfig: type accessors ─────────────────────────────────────────────────

TEST_CASE("LQRConfig.FlywheelSystemSetsType", "[LQRConfig]") {
  CHECK(FlywheelConfig().GetType() == LQRConfig::LQRType::FLYWHEEL);
}

TEST_CASE("LQRConfig.ArmSystemSetsType", "[LQRConfig]") {
  CHECK(ArmConfig().GetType() == LQRConfig::LQRType::ARM);
}

TEST_CASE("LQRConfig.ElevatorSystemSetsType", "[LQRConfig]") {
  CHECK(ElevatorConfig().GetType() == LQRConfig::LQRType::ELEVATOR);
}

// ── LQRConfig: period and voltage ────────────────────────────────────────────

TEST_CASE("LQRConfig.DefaultPeriodIs20ms", "[LQRConfig]") {
  CHECK(FlywheelConfig().GetPeriod().value() == Catch::Approx(0.020).margin(1e-9));
}

TEST_CASE("LQRConfig.WithPeriodStored", "[LQRConfig]") {
  CHECK(FlywheelConfig().WithPeriod(10_ms).GetPeriod().value() == Catch::Approx(0.010).margin(1e-9));
}

TEST_CASE("LQRConfig.DefaultMaxVoltageIs12V", "[LQRConfig]") {
  CHECK(FlywheelConfig().GetMaxVoltage().value() == Catch::Approx(12.0).margin(1e-9));
}

TEST_CASE("LQRConfig.WithMaxVoltageStored", "[LQRConfig]") {
  CHECK(FlywheelConfig().WithMaxVoltage(9_V).GetMaxVoltage().value() ==
        Catch::Approx(9.0).margin(1e-9));
}

// ── LQRConfig: loop variant type ─────────────────────────────────────────────

TEST_CASE("LQRConfig.FlywheelGetLoopIsLoop1", "[LQRConfig]") {
  CHECK(std::holds_alternative<LQRConfig::Loop1>(FlywheelConfig().GetLoop()));
}

TEST_CASE("LQRConfig.ArmGetLoopIsLoop2", "[LQRConfig]") {
  CHECK(std::holds_alternative<LQRConfig::Loop2>(ArmConfig().GetLoop()));
}

TEST_CASE("LQRConfig.ElevatorGetLoopIsLoop2", "[LQRConfig]") {
  CHECK(std::holds_alternative<LQRConfig::Loop2>(ElevatorConfig().GetLoop()));
}

// ── LQRController: type and config accessors ─────────────────────────────────

TEST_CASE("LQRController.FlywheelGetType", "[LQRController]") {
  CHECK(LQRController{FlywheelConfig()}.GetType() == LQRConfig::LQRType::FLYWHEEL);
}

TEST_CASE("LQRController.ArmGetType", "[LQRController]") {
  CHECK(LQRController{ArmConfig()}.GetType() == LQRConfig::LQRType::ARM);
}

TEST_CASE("LQRController.ElevatorGetType", "[LQRController]") {
  CHECK(LQRController{ElevatorConfig()}.GetType() == LQRConfig::LQRType::ELEVATOR);
}

TEST_CASE("LQRController.GetConfigIsPresent", "[LQRController]") {
  CHECK(LQRController{FlywheelConfig()}.GetConfig().has_value());
}

TEST_CASE("LQRController.GetConfigPreservesType", "[LQRController]") {
  LQRController ctrl{ArmConfig()};
  REQUIRE(ctrl.GetConfig().has_value());
  CHECK(ctrl.GetConfig()->GetType() == LQRConfig::LQRType::ARM);
}

// ── LQRController: flywheel angular velocity ─────────────────────────────────

TEST_CASE("LQRController.Flywheel_BelowSetpoint_PositiveOutput", "[LQRController]") {
  LQRController ctrl{FlywheelConfig()};
  CHECK(ctrl.Calculate(0_rad_per_s, 200_rad_per_s).value() > 0.0);
}

TEST_CASE("LQRController.Flywheel_AboveSetpoint_NegativeOutput", "[LQRController]") {
  LQRController ctrl{FlywheelConfig()};
  // Warm the state estimate up toward 300 rad/s, then request 100 rad/s.
  for (int i = 0; i < 5; ++i) ctrl.Calculate(300_rad_per_s, 300_rad_per_s);
  CHECK(ctrl.Calculate(300_rad_per_s, 100_rad_per_s).value() < 0.0);
}

TEST_CASE("LQRController.Flywheel_ClampedToMaxVoltage", "[LQRController]") {
  LQRController ctrl{FlywheelConfig().WithMaxVoltage(12_V)};
  CHECK(std::abs(ctrl.Calculate(0_rad_per_s, 1e6_rad_per_s).value()) <= 12.0 + 1e-6);
}

TEST_CASE("LQRController.Flywheel_OutputIsFinite", "[LQRController]") {
  LQRController ctrl{FlywheelConfig()};
  CHECK(std::isfinite(ctrl.Calculate(0_rad_per_s, 100_rad_per_s).value()));
}

TEST_CASE("LQRController.Flywheel_ResetToSetpoint_ReducesOutput", "[LQRController]") {
  // Without reset the Kalman state starts at 0, so error = 100 → large output.
  LQRController ctrl_far{FlywheelConfig()};
  auto out_far = ctrl_far.Calculate(0_rad_per_s, 100_rad_per_s);

  // After resetting state to the setpoint, error ≈ 0 → smaller output.
  LQRController ctrl_near{FlywheelConfig()};
  ctrl_near.Reset(0_rad, 100_rad_per_s);
  auto out_near = ctrl_near.Calculate(100_rad_per_s, 100_rad_per_s);

  CHECK(std::abs(out_near.value()) < std::abs(out_far.value()));
}

// ── LQRController: arm angular position ──────────────────────────────────────

TEST_CASE("LQRController.Arm_BelowSetpoint_PositiveOutput", "[LQRController]") {
  LQRController ctrl{ArmConfig()};
  ctrl.Reset(0_rad, 0_rad_per_s);
  CHECK(ctrl.Calculate(0_rad, 1_rad, 0_rad_per_s).value() > 0.0);
}

TEST_CASE("LQRController.Arm_AboveSetpoint_NegativeOutput", "[LQRController]") {
  LQRController ctrl{ArmConfig()};
  ctrl.Reset(2_rad, 0_rad_per_s);
  CHECK(ctrl.Calculate(2_rad, 1_rad, 0_rad_per_s).value() < 0.0);
}

TEST_CASE("LQRController.Arm_ClampedToMaxVoltage", "[LQRController]") {
  LQRController ctrl{ArmConfig().WithMaxVoltage(12_V)};
  ctrl.Reset(0_rad, 0_rad_per_s);
  CHECK(std::abs(ctrl.Calculate(0_rad, 1000_rad, 0_rad_per_s).value()) <= 12.0 + 1e-6);
}

TEST_CASE("LQRController.Arm_OutputIsFinite", "[LQRController]") {
  LQRController ctrl{ArmConfig()};
  ctrl.Reset(0_rad, 0_rad_per_s);
  CHECK(std::isfinite(ctrl.Calculate(0_rad, 0.5_rad, 0_rad_per_s).value()));
}

TEST_CASE("LQRController.Arm_ResetToSetpoint_ReducesOutput", "[LQRController]") {
  LQRController ctrl_far{ArmConfig()};
  ctrl_far.Reset(0_rad, 0_rad_per_s);
  auto out_far = ctrl_far.Calculate(0_rad, 1_rad, 0_rad_per_s);

  LQRController ctrl_near{ArmConfig()};
  ctrl_near.Reset(1_rad, 0_rad_per_s);
  auto out_near = ctrl_near.Calculate(1_rad, 1_rad, 0_rad_per_s);

  CHECK(std::abs(out_near.value()) < std::abs(out_far.value()));
}

// ── LQRController: elevator linear position ──────────────────────────────────

TEST_CASE("LQRController.Elevator_BelowSetpoint_PositiveOutput", "[LQRController]") {
  LQRController ctrl{ElevatorConfig()};
  ctrl.Reset(0_m, 0_mps);
  CHECK(ctrl.Calculate(0_m, 0.5_m, 0_mps).value() > 0.0);
}

TEST_CASE("LQRController.Elevator_AboveSetpoint_NegativeOutput", "[LQRController]") {
  LQRController ctrl{ElevatorConfig()};
  ctrl.Reset(1_m, 0_mps);
  CHECK(ctrl.Calculate(1_m, 0.5_m, 0_mps).value() < 0.0);
}

TEST_CASE("LQRController.Elevator_ClampedToMaxVoltage", "[LQRController]") {
  LQRController ctrl{ElevatorConfig().WithMaxVoltage(12_V)};
  ctrl.Reset(0_m, 0_mps);
  CHECK(std::abs(ctrl.Calculate(0_m, 10000_m, 0_mps).value()) <= 12.0 + 1e-6);
}

TEST_CASE("LQRController.Elevator_OutputIsFinite", "[LQRController]") {
  LQRController ctrl{ElevatorConfig()};
  ctrl.Reset(0_m, 0_mps);
  CHECK(std::isfinite(ctrl.Calculate(0_m, 0.5_m, 0_mps).value()));
}

TEST_CASE("LQRController.Elevator_ResetAffectsDirection", "[LQRController]") {
  LQRController ctrl{ElevatorConfig()};

  ctrl.Reset(0_m, 0_mps);
  auto out_below = ctrl.Calculate(0_m, 0.5_m, 0_mps);

  ctrl.Reset(1_m, 0_mps);
  auto out_above = ctrl.Calculate(1_m, 0.5_m, 0_mps);

  CHECK(out_below.value() > 0.0);
  CHECK(out_above.value() < 0.0);
}

// ── LQRController: linear flywheel (m/s) ─────────────────────────────────────

TEST_CASE("LQRController.LinearFlywheel_BelowSetpoint_PositiveOutput", "[LQRController]") {
  LQRController ctrl{FlywheelConfig()};
  CHECK(ctrl.Calculate(0_mps, 5_mps).value() > 0.0);
}

TEST_CASE("LQRController.LinearFlywheel_ClampedToMaxVoltage", "[LQRController]") {
  LQRController ctrl{FlywheelConfig().WithMaxVoltage(12_V)};
  CHECK(std::abs(ctrl.Calculate(0_mps, 1e6_mps).value()) <= 12.0 + 1e-6);
}

TEST_CASE("LQRController.LinearFlywheel_OutputIsFinite", "[LQRController]") {
  LQRController ctrl{FlywheelConfig()};
  CHECK(std::isfinite(ctrl.Calculate(0_mps, 5_mps).value()));
}

// ── LQRController: UpdateConfig ───────────────────────────────────────────────

TEST_CASE("LQRController.UpdateConfig_ChangesType", "[LQRController]") {
  LQRController ctrl{FlywheelConfig()};
  REQUIRE(ctrl.GetType() == LQRConfig::LQRType::FLYWHEEL);
  ctrl.UpdateConfig(ArmConfig());
  CHECK(ctrl.GetType() == LQRConfig::LQRType::ARM);
}

TEST_CASE("LQRController.UpdateConfig_NewMaxVoltageRespected", "[LQRController]") {
  LQRController ctrl{FlywheelConfig()};
  ctrl.UpdateConfig(FlywheelConfig().WithMaxVoltage(6_V));
  CHECK(std::abs(ctrl.Calculate(0_rad_per_s, 1e6_rad_per_s).value()) <= 6.0 + 1e-6);
}

}  // namespace yams::test
