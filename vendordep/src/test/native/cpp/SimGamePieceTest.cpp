// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Mirrors Java GamePieceSimTest: a simulated mechanism carrying game pieces moves as the heavier
// mechanism would. Each test drives the simulation at a voltage worked out by hand from the physics,
// and checks the mechanism holds still, falls or speeds up as it should.

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/simulation/RoboRioSim.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/mass.hpp>
#include <wpi/units/moment_of_inertia.hpp>

#include "helpers/MockHardware.h"
#include "helpers/MotorControllerFactory.h"
#include "helpers/SchedulerHelper.h"
#include "yams/mechanisms/config/ArmConfig.hpp"
#include "yams/mechanisms/config/ElevatorConfig.hpp"
#include "yams/mechanisms/config/PivotConfig.hpp"
#include "yams/mechanisms/positional/Arm.hpp"
#include "yams/mechanisms/positional/Elevator.hpp"
#include "yams/mechanisms/positional/Pivot.hpp"

namespace yams::test {

using namespace motorcontrollers;
using namespace mechanisms;
using namespace mechanisms::config;

namespace {

constexpr double kGravity = 9.8;
// Low enough that the mechanism takes a few tenths of a second to reach its speed, so its inertia
// shows.
constexpr double kGearing = 10.0;
constexpr double kArmLength = 0.5;
constexpr double kArmMass = 2.0;
constexpr double kPieceMass = 1.0;
constexpr double kCarriageMass = 5.0;
constexpr double kDrumRadius = 0.02;
constexpr double kSimPeriod = 0.02;
// A uniform rod about its end, as WPILib's arm assumes.
constexpr double kArmMoi = kArmMass * kArmLength * kArmLength / 3.0;

const wpi::math::DCMotor kMotor = wpi::math::DCMotor::NEO(1);

struct SimGamePieceFixture {
  SimGamePieceFixture() {
    InitializeHardware();
    SchedulerHelper::Enable();
    SchedulerHelper::CancelAll();
    wpi::sim::RoboRioSim::SetVInVoltage(12_V);
  }
  ~SimGamePieceFixture() {
    TeardownHardware();
    SchedulerHelper::CancelAll();
  }
};

SmartMotorControllerConfig BaseConfig(const std::string& name) {
  return MakeBaseConfig(ProfileType::None, 0.0,
                        gearing::MechanismGearing{gearing::GearBox::FromReductionStages({kGearing})},
                        nullptr, name);
}

HardwareBundle Spark(SmartMotorControllerConfig cfg, const std::string& name) {
  auto bundle = MakeBundle({HardwareType::SparkMax, ProfileType::None, name}, cfg);
  bundle.smc->SetupSimulation();
  return bundle;
}

// Drive the simulation at a voltage for a time, as the motor controller would each loop.
void Drive(SimSupplier* sim, double volts, double seconds) {
  const int loops = static_cast<int>(std::lround(seconds / kSimPeriod));
  for (int loop = 0; loop < loops; loop++) {
    sim->SetMechanismStatorVoltage(wpi::units::volt_t{volts});
    sim->StarveUpdateSim();
    sim->UpdateSim();
  }
}

// The voltage that holds a torque at the mechanism with the motor stalled.
double StallVolts(double torque) {
  return torque * kMotor.R.value() / (kGearing * kMotor.Kt.value());
}

// A motor's speed at the mechanism, from rest, after a time at a voltage above what holds it: it rises
// to the voltage's free speed with a time constant in proportion to the inertia.
double SpeedAfter(double volts, double seconds, double moi) {
  const double freeSpeed = volts * kMotor.Kv.value() / kGearing;
  const double timeConstant =
      moi * kMotor.R.value() * kMotor.Kv.value() / (kGearing * kGearing * kMotor.Kt.value());
  return freeSpeed * (1 - std::exp(-seconds / timeConstant));
}

double Degrees(SimSupplier* sim) { return wpi::units::degree_t{sim->GetMechanismPosition()}.value(); }

double RadiansPerSecond(SimSupplier* sim) {
  return wpi::units::radians_per_second_t{sim->GetMechanismVelocity()}.value();
}

double Radians(SimSupplier* sim) { return wpi::units::radian_t{sim->GetMechanismPosition()}.value(); }

}  // namespace

TEST_CASE_METHOD(SimGamePieceFixture, "SimGamePieces.Arm", "[SimGamePieces]") {
  auto cfg = BaseConfig("GamePieceArm");
  cfg.WithMOI(wpi::units::kilogram_square_meter_t{kArmMoi}).WithStartingPosition(0_deg);
  auto bundle = Spark(cfg, "GamePieceArm");
  auto* armCfg = new ArmConfig;
  armCfg->WithArmLength(wpi::units::meter_t{kArmLength}).WithMinAngle(-100_deg).WithMaxAngle(100_deg);
  auto* arm = new positional::Arm(armCfg, bundle.smc);
  SimSupplier* sim = bundle.smc->GetSimSupplier();
  const double armTorque = kArmMass * kGravity * kArmLength / 2;

  SECTION("held level by the voltage that holds the arm and the game piece") {
    const double pieceTorque = kPieceMass * kGravity * kArmLength;
    Drive(sim, StallVolts(armTorque), 0.5);
    CHECK(std::abs(Degrees(sim)) < 0.5);

    // With the game piece at its end, the voltage that holds the empty arm no longer holds it.
    sim->AddGamePiece(wpi::units::kilogram_t{kPieceMass});
    Drive(sim, StallVolts(armTorque), 0.5);
    INFO("arm with a game piece at " << Degrees(sim) << " deg");
    CHECK(Degrees(sim) < -5);

    sim->SetMechanismPosition(0_deg);
    sim->SetMechanismVelocity(0_tps);
    const double holdVolts = StallVolts(armTorque + pieceTorque);
    Drive(sim, holdVolts, 0.5);
    CHECK(std::abs(Degrees(sim)) < 0.5);
    // The current is the motor's at that voltage, not at the voltage the simulation is given.
    CHECK(std::abs(sim->GetStatorCurrent().value() - holdVolts / kMotor.R.value()) <
          holdVolts / kMotor.R.value() * 0.03);

    // Without the game piece again, that voltage lifts the arm.
    sim->RemoveGamePiece();
    Drive(sim, holdVolts, 0.5);
    CHECK(Degrees(sim) > 5);
  }

  SECTION("accelerates as its moment of inertia with the game piece says") {
    const double pieceDistance = 0.3;
    const double withPiece = kArmMoi + kPieceMass * pieceDistance * pieceDistance;
    const double holdVolts = StallVolts(armTorque + kPieceMass * kGravity * pieceDistance);
    sim->AddGamePiece(wpi::units::kilogram_t{kPieceMass}, wpi::units::meter_t{pieceDistance});
    const double seconds = 0.1;
    Drive(sim, holdVolts + 1, seconds);
    const double expected = SpeedAfter(1, seconds, withPiece);
    INFO("arm's speed " << RadiansPerSecond(sim) << " rad/s, expected " << expected);
    CHECK(std::abs(RadiansPerSecond(sim) - expected) < expected * 0.03);
    // Without the game piece's inertia, the arm would be noticeably faster.
    CHECK(SpeedAfter(1, seconds, kArmMoi) > expected * 1.2);
  }

  SchedulerHelper::CancelAll();
  CloseBundle(bundle);
  delete arm;
}

TEST_CASE_METHOD(SimGamePieceFixture, "SimGamePieces.Elevator", "[SimGamePieces]") {
  auto cfg = BaseConfig("GamePieceElevator");
  cfg.WithMechanismCircumference(wpi::units::meter_t{2 * std::numbers::pi * kDrumRadius})
      .WithStartingPosition(1_m);
  auto bundle = Spark(cfg, "GamePieceElevator");
  auto* elevatorCfg = new ElevatorConfig;
  elevatorCfg->WithMinimumHeight(0_m).WithMaximumHeight(2_m).WithCarriageMass(
      wpi::units::kilogram_t{kCarriageMass});
  auto* elevator = new positional::Elevator(elevatorCfg, bundle.smc);
  SimSupplier* sim = bundle.smc->GetSimSupplier();
  const double carriageVolts = StallVolts(kCarriageMass * kGravity * kDrumRadius);
  const double loadedVolts = StallVolts((kCarriageMass + kPieceMass) * kGravity * kDrumRadius);
  const double start = Radians(sim);

  Drive(sim, carriageVolts, 0.5);
  CHECK(std::abs(Radians(sim) - start) < 0.05);

  // With the game piece, the same voltage lets the carriage fall, at the speed where the motor's back
  // EMF holds the game piece's extra weight.
  sim->AddGamePiece(wpi::units::kilogram_t{kPieceMass});
  Drive(sim, carriageVolts, 0.5);
  const double fallSpeed = kPieceMass * kGravity * kDrumRadius * kMotor.R.value() * kMotor.Kv.value() /
                           (kGearing * kGearing * kMotor.Kt.value()) * kDrumRadius;
  INFO("carriage falling at " << RadiansPerSecond(sim) * kDrumRadius << " m/s, expected "
                              << -fallSpeed);
  CHECK(std::abs(RadiansPerSecond(sim) * kDrumRadius + fallSpeed) < fallSpeed * 0.03);

  const double heldAt = Radians(sim);
  sim->SetMechanismVelocity(0_tps);
  Drive(sim, loadedVolts, 0.5);
  CHECK(std::abs(Radians(sim) - heldAt) < 0.05);

  SchedulerHelper::CancelAll();
  CloseBundle(bundle);
  delete elevator;
}

TEST_CASE_METHOD(SimGamePieceFixture, "SimGamePieces.Pivot", "[SimGamePieces]") {
  auto cfg = BaseConfig("GamePiecePivot");
  cfg.WithMOI(wpi::units::kilogram_square_meter_t{kArmMoi}).WithStartingPosition(0_deg);
  auto bundle = Spark(cfg, "GamePiecePivot");
  auto* pivotCfg = new PivotConfig;
  pivotCfg->WithMinAngle(-180_deg).WithMaxAngle(180_deg);
  auto* pivot = new positional::Pivot(pivotCfg, bundle.smc);
  bundle.smc->SetupSimulation();
  SimSupplier* sim = bundle.smc->GetSimSupplier();

  SECTION("speeds up as its moment of inertia with the game piece says") {
    const double pieceDistance = 0.4;
    const double withPiece = kArmMoi + kPieceMass * pieceDistance * pieceDistance;
    sim->AddGamePiece(wpi::units::kilogram_t{kPieceMass}, wpi::units::meter_t{pieceDistance});
    const double seconds = 0.3;
    Drive(sim, 2, seconds);
    const double expected = SpeedAfter(2, seconds, withPiece);
    INFO("pivot's speed " << RadiansPerSecond(sim) << " rad/s, expected " << expected);
    CHECK(std::abs(RadiansPerSecond(sim) - expected) < expected * 0.02);
    CHECK(SpeedAfter(2, seconds, kArmMoi) > expected * 1.2);
  }

  SECTION("needs the game piece's distance from the axis") {
    CHECK_THROWS_AS(sim->AddGamePiece(wpi::units::kilogram_t{kPieceMass}), std::logic_error);
    // Removing a game piece that is not there does nothing.
    sim->RemoveGamePiece();
  }

  SchedulerHelper::CancelAll();
  CloseBundle(bundle);
  delete pivot;
}

}  // namespace yams::test
