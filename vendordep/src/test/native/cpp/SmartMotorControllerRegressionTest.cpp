// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Regression tests for SmartMotorController / SmartMotorControllerConfig behaviour that differed
// from the Java reference. These tests only use API that also existed before the fixes, so each
// can be run against the old code to confirm it catches the bug.

#include <rev/SparkMax.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <memory>
#include <string>
#include <wpi/commands2/CommandScheduler.hpp>
#include <wpi/hardware/bus/CANPort.hpp>
#include <wpi/math/controller/ArmFeedforward.hpp>
#include <wpi/math/controller/ElevatorFeedforward.hpp>
#include <wpi/math/controller/SimpleMotorFeedforward.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/nt/NetworkTableInstance.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/mass.hpp>

#include "helpers/FakeSmartMotorController.h"
#include "helpers/MockHardware.h"
#include "helpers/MotorControllerFactory.h"
#include "helpers/TestSubsystem.h"
#include "yams/exceptions.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/motorcontrollers/SmartMotorControllerCommandRegistry.hpp"
#include "yams/motorcontrollers/local/SparkWrapper.hpp"

namespace yams::test {

using motorcontrollers::SmartMotorControllerCommandRegistry;
using Slot = SmartMotorControllerConfig::ClosedLoopControllerSlot;
using Verbosity = SmartMotorControllerConfig::TelemetryVerbosity;
using SimpleFF = wpi::math::SimpleMotorFeedforward<wpi::units::turns>;

namespace {

std::string UniqueName(const std::string& prefix) {
  static int count = 0;
  return prefix + std::to_string(count++);
}

std::shared_ptr<wpi::nt::NetworkTable> DataRoot() {
  return wpi::nt::NetworkTableInstance::GetDefault().GetTable("SMCRegression/Data");
}

std::shared_ptr<wpi::nt::NetworkTable> TuningRoot() {
  return wpi::nt::NetworkTableInstance::GetDefault().GetTable("SMCRegression/Tuning");
}

SimpleFF MakeSimpleFF(double kS, double kV, double kA) {
  return SimpleFF{wpi::units::volt_t{kS}, wpi::units::unit_t<SimpleFF::kv_unit>{kV},
                  wpi::units::unit_t<SimpleFF::ka_unit>{kA}};
}

wpi::math::ElevatorFeedforward MakeElevatorFF(double kG) {
  return wpi::math::ElevatorFeedforward{
      0_V, wpi::units::volt_t{kG},
      wpi::units::unit_t<wpi::math::ElevatorFeedforward::kv_unit>{0.0},
      wpi::units::unit_t<wpi::math::ElevatorFeedforward::ka_unit>{0.0}};
}

wpi::math::ArmFeedforward MakeArmFF(double kG) {
  return wpi::math::ArmFeedforward{0_V, wpi::units::volt_t{kG},
                                   wpi::units::unit_t<wpi::math::ArmFeedforward::kv_unit>{0.0},
                                   wpi::units::unit_t<wpi::math::ArmFeedforward::ka_unit>{0.0}};
}

}  // namespace

// ---- Item 1: the wrappers' config is the one the base class uses ----------------------------

TEST_CASE("SMCRegression.WrapperConfigUsedByBaseSafetyCheck", "[SMCRegression]") {
  InitializeHardware();
  // A NEO 550 with a stator current limit passes the base safety check only if the base class
  // reads the wrapper's config (it used to read its own default-constructed copy).
  auto spark = std::make_unique<rev::spark::SparkMax>(
      wpi::CANPort::CAN_S0, NextCanId(), rev::spark::SparkLowLevel::MotorType::kBrushless);
  SmartMotorControllerConfig cfg;
  cfg.WithMotorGearing(gearing::MechanismGearing{5.0}).WithStatorCurrentLimit(30_A);
  std::unique_ptr<motorcontrollers::local::SparkWrapper> smc;
  CHECK_NOTHROW(smc = std::make_unique<motorcontrollers::local::SparkWrapper>(
                    spark.get(), wpi::math::DCMotor::NEO550(1), &cfg));
  smc.reset();
  TeardownHardware();
}

// ---- Item 3: telemetry --------------------------------------------------------------------

TEST_CASE("SMCRegression.UpdateTelemetryDoesNotApplyTuning", "[SMCRegression]") {
  auto name = UniqueName("NoTuneOnUpdate");
  SmartMotorControllerConfig cfg;
  cfg.WithFeedback(1.0, 0.0, 0.0).WithTelemetry(name, Verbosity::HIGH);
  FakeSmartMotorController smc{&cfg};
  smc.SetupTelemetry(DataRoot(), TuningRoot());

  // A dashboard edit is only applied by the Live Tuning command, not by publishing telemetry.
  TuningRoot()->GetSubTable(name)->GetEntry("closedloop/feedback/kP").SetDouble(5.0);
  smc.UpdateTelemetry();
  smc.UpdateTelemetry();
  CHECK(smc.setKpCalls == 0);
  smc.Close();
}

TEST_CASE("SMCRegression.SetupTelemetryPublishesImmediately", "[SMCRegression]") {
  auto name = UniqueName("PublishOnSetup");
  SmartMotorControllerConfig cfg;
  cfg.WithTelemetry(name, Verbosity::LOW);
  FakeSmartMotorController smc{&cfg};
  smc.pos = 0.25_tr;
  smc.SetupTelemetry(DataRoot(), TuningRoot());
  CHECK(DataRoot()->GetSubTable(name)->GetEntry("mechanism/position").GetDouble(-1.0) ==
        Catch::Approx(0.25));
  smc.Close();
}

// ---- Item 4: Close() releases the Live Tuning registration --------------------------------

TEST_CASE("SMCRegression.CloseRemovesLiveTuningCommand", "[SMCRegression]") {
  TestSubsystem sub;
  sub.SetName(UniqueName("SMCRegressionLiveTuning"));
  SmartMotorControllerConfig cfg;
  cfg.WithFeedback(1.0, 0.0, 0.0)
      .WithSubsystem(&sub)
      .WithTelemetry(UniqueName("LiveTuningMotor"), Verbosity::HIGH);
  {
    FakeSmartMotorController smc{&cfg};
    smc.UpdateTelemetry();  // sets up telemetry with the default tables, registering Live Tuning
    REQUIRE(SmartMotorControllerCommandRegistry::CommandExists("Live Tuning", &sub));
    smc.Close();
    // The registered callback captures the controller, so it must not outlive it.
    CHECK_FALSE(SmartMotorControllerCommandRegistry::CommandExists("Live Tuning", &sub));
  }
  SmartMotorControllerCommandRegistry::RemoveCommands(&sub);
  wpi::cmd::CommandScheduler::GetInstance().UnregisterSubsystem(&sub);
}

TEST_CASE("SMCRegression.RemoveCommandsUnpublishesTunable", "[SMCRegression]") {
  TestSubsystem sub;
  auto subName = UniqueName("SMCRegressionRemoveTunable");
  sub.SetName(subName);
  SmartMotorControllerCommandRegistry::AddCommand("Live Tuning", &sub, [] {});
  auto table =
      wpi::nt::NetworkTableInstance::GetDefault().GetTable("Tuning")->GetSubTable(subName);
  REQUIRE_FALSE(table->GetSubTable("Live Tuning")->GetKeys().empty());
  SmartMotorControllerCommandRegistry::RemoveCommands(&sub);
  CHECK(table->GetSubTable("Live Tuning")->GetKeys().empty());
  wpi::cmd::CommandScheduler::GetInstance().UnregisterSubsystem(&sub);
}

// ---- Item 5: linear closed-loop mode -------------------------------------------------------

TEST_CASE("SMCRegression.LinearModeNeedsLinearIndicator", "[SMCRegression]") {
  SECTION("circumference alone is rotational") {
    SmartMotorControllerConfig cfg;
    cfg.WithMechanismCircumference(0.1_m);
    CHECK_FALSE(cfg.GetLinearClosedLoopControllerUse());
  }
  SECTION("elevator feedforward makes it linear") {
    SmartMotorControllerConfig cfg;
    cfg.WithMechanismCircumference(0.1_m).WithFeedforward(MakeElevatorFF(1.0));
    CHECK(cfg.GetLinearClosedLoopControllerUse());
  }
  SECTION("linear trapezoidal profile makes it linear") {
    SmartMotorControllerConfig cfg;
    cfg.WithMechanismCircumference(0.1_m).WithLinearTrapezoidProfile(1_mps, 1_mps_sq);
    CHECK(cfg.GetLinearClosedLoopControllerUse());
  }
  SECTION("linear indicator without circumference is rotational") {
    SmartMotorControllerConfig cfg;
    cfg.WithFeedforward(MakeElevatorFF(1.0));
    CHECK_FALSE(cfg.GetLinearClosedLoopControllerUse());
  }
}

// ---- Item 6: the profiled feedforward keeps its kA term ------------------------------------

TEST_CASE("SMCRegression.ProfiledFeedforwardIncludesKa", "[SMCRegression]") {
  SmartMotorControllerConfig cfg;
  cfg.WithTrapezoidProfile(1_tps, 2_tr_per_s_sq).WithFeedforward(MakeSimpleFF(0.0, 0.0, 1.0));
  FakeSmartMotorController smc{&cfg};
  smc.SetPosition(1_tr);
  smc.SetRunning(true);
  smc.IterateClosedLoopController();
  // From rest the profile accelerates at 2 rot/s^2, so kA = 1 V/(rot/s^2) gives 2 V.
  CHECK(smc.lastVoltage.value() == Catch::Approx(2.0).margin(0.05));
}

// ---- Item 7: continuous wrapping in the RoboRIO loop ---------------------------------------

TEST_CASE("SMCRegression.ContinuousWrappingTakesShortestPath", "[SMCRegression]") {
  SmartMotorControllerConfig cfg;
  cfg.WithFeedback(1.0, 0.0, 0.0).WithContinuousWrapping(-0.5_tr, 0.5_tr);
  FakeSmartMotorController smc{&cfg};
  smc.CreatePID();
  smc.pos = 0.45_tr;
  smc.SetPosition(-0.45_tr);
  smc.SetRunning(true);
  smc.IterateClosedLoopController();
  // -0.45 is 0.1 rotations ahead of 0.45 across the wrapping point.
  CHECK(smc.lastVoltage.value() == Catch::Approx(0.1).margin(1e-6));

  // The measurement wrapping past the boundary must not jump the controller.
  smc.pos = -0.48_tr;
  smc.IterateClosedLoopController();
  CHECK(smc.lastVoltage.value() == Catch::Approx(0.03).margin(1e-6));
}

// ---- Item 8: measurement limits are applied as mechanism limits ----------------------------

TEST_CASE("SMCRegression.MeasurementLimitsBecomeMechanismLimits", "[SMCRegression]") {
  SECTION("converted with the circumference") {
    SmartMotorControllerConfig cfg;
    cfg.WithMechanismCircumference(2_m).WithMeasurementLimits(0.5_m, 1_m);
    REQUIRE(cfg.GetMechanismLowerLimit().has_value());
    REQUIRE(cfg.GetMechanismUpperLimit().has_value());
    CHECK(cfg.GetMechanismLowerLimit()->value() == Catch::Approx(0.25));
    CHECK(cfg.GetMechanismUpperLimit()->value() == Catch::Approx(0.5));
  }
  SECTION("throws without circumference") {
    SmartMotorControllerConfig cfg;
    CHECK_THROWS_AS(cfg.WithMeasurementLimits(0_m, 1_m),
                    exceptions::SmartMotorControllerConfigurationException);
  }
}

// ---- Item 9: linear profiles ---------------------------------------------------------------

TEST_CASE("SMCRegression.LinearExponentialProfileIsFollowed", "[SMCRegression]") {
  SmartMotorControllerConfig cfg;
  cfg.WithFeedback(1.0, 0.0, 0.0)
      .WithMotorGearing(gearing::MechanismGearing{10.0})
      .WithExponentialProfile(12_V, wpi::math::DCMotor::KrakenX60(1), 5_kg, 0.05_m);
  FakeSmartMotorController smc{&cfg};
  smc.CreatePID();
  smc.SetPosition(1_m);
  smc.SetRunning(true);
  smc.IterateClosedLoopController();
  // The PID chases the profile's first step, not the 1 m goal (which would give 1 V).
  CHECK(std::abs(smc.lastVoltage.value()) < 0.5);
}

TEST_CASE("SMCRegression.LinearTrapezoidStateResetOnStart", "[SMCRegression]") {
  SmartMotorControllerConfig cfg;
  cfg.WithMechanismCircumference(1_m).WithLinearTrapezoidProfile(1_mps, 1_mps_sq);
  FakeSmartMotorController smc{&cfg};
  smc.CreateIdleClosedLoopThread();
  smc.pos = 2_tr;  // 2 m
  smc.StartClosedLoopController();
  auto state = smc.LinearTrapState();
  smc.StopClosedLoopController();
  REQUIRE(state.has_value());
  CHECK(state->position.value() == Catch::Approx(2.0));
}

// ---- Item 10: elevator and arm feedforward outside the position branch ---------------------

TEST_CASE("SMCRegression.FeedforwardAppliedForVelocitySetpoint", "[SMCRegression]") {
  SECTION("elevator") {
    SmartMotorControllerConfig cfg;
    cfg.WithMechanismCircumference(1_m).WithFeedforward(MakeElevatorFF(1.0));
    FakeSmartMotorController smc{&cfg};
    smc.SetVelocity(0_mps);
    smc.SetRunning(true);
    smc.IterateClosedLoopController();
    CHECK(smc.lastVoltage.value() == Catch::Approx(1.0));
  }
  SECTION("arm") {
    SmartMotorControllerConfig cfg;
    cfg.WithFeedforward(MakeArmFF(1.0));
    FakeSmartMotorController smc{&cfg};
    smc.SetVelocity(0_tps);
    smc.SetRunning(true);
    smc.IterateClosedLoopController();
    CHECK(smc.lastVoltage.value() == Catch::Approx(1.0));
  }
}

// ---- Item 11: conversions require a circumference ------------------------------------------

TEST_CASE("SMCRegression.ConvertFromMechanismRequiresCircumference", "[SMCRegression]") {
  SmartMotorControllerConfig cfg;
  CHECK_THROWS_AS(cfg.ConvertFromMechanism(1_tr),
                  exceptions::SmartMotorControllerConfigurationException);
  CHECK_THROWS_AS(cfg.ConvertFromMechanism(1_tps),
                  exceptions::SmartMotorControllerConfigurationException);
  cfg.WithMechanismCircumference(0.5_m);
  CHECK(cfg.ConvertFromMechanism(2_tr).value() == Catch::Approx(1.0));
}

// ---- Items 13, 14, 16: config defaults --------------------------------------------------------

TEST_CASE("SMCRegression.DefaultMomentOfInertia", "[SMCRegression]") {
  SmartMotorControllerConfig cfg;
  CHECK(cfg.GetMOI().value() == Catch::Approx(0.02));
}

TEST_CASE("SMCRegression.InversionIgnoredInSimulation", "[SMCRegression]") {
  SmartMotorControllerConfig cfg;
  cfg.WithMotorInverted(true).WithExternalEncoderInverted(true);
  REQUIRE(cfg.GetMotorInverted().has_value());
  CHECK_FALSE(*cfg.GetMotorInverted());
  REQUIRE(cfg.GetExternalEncoderInverted().has_value());
  CHECK_FALSE(*cfg.GetExternalEncoderInverted());
}

TEST_CASE("SMCRegression.DrumExponentialProfileSetsMaxInput", "[SMCRegression]") {
  SmartMotorControllerConfig cfg;
  cfg.WithExponentialProfile(10_V, wpi::math::DCMotor::KrakenX60(1), 5_kg, 0.05_m);
  REQUIRE(cfg.GetExponentialProfileMaxInput().has_value());
  CHECK(cfg.GetExponentialProfileMaxInput()->value() == Catch::Approx(10.0));
}

// ---- Item 15: config validation ------------------------------------------------------------

TEST_CASE("SMCRegression.ConfigValidation", "[SMCRegression]") {
  SECTION("continuous wrapping needs a closed-loop controller") {
    SmartMotorControllerConfig cfg;
    CHECK_THROWS_AS(cfg.WithContinuousWrapping(-0.5_tr, 0.5_tr),
                    exceptions::SmartMotorControllerConfigurationException);
  }
  SECTION("lower limit must be below upper limit") {
    SmartMotorControllerConfig cfg;
    CHECK_THROWS_AS(cfg.WithMechanismLimits(1_tr, 0_tr),
                    exceptions::SmartMotorControllerConfigurationException);
    CHECK_THROWS_AS(cfg.WithMechanismLimits(1_tr, 1_tr),
                    exceptions::SmartMotorControllerConfigurationException);
  }
  SECTION("negative zero offset wraps by one rotation") {
    SmartMotorControllerConfig cfg;
    cfg.WithExternalEncoderZeroOffset(-0.25_tr);
    REQUIRE(cfg.GetExternalEncoderZeroOffset().has_value());
    CHECK(cfg.GetExternalEncoderZeroOffset()->value() == Catch::Approx(0.75));
  }
  SECTION("subsystem may only be set once") {
    TestSubsystem a;
    TestSubsystem b;
    SmartMotorControllerConfig cfg;
    cfg.WithSubsystem(&a);
    CHECK_THROWS_AS(cfg.WithSubsystem(&b),
                    exceptions::SmartMotorControllerConfigurationException);
    wpi::cmd::CommandScheduler::GetInstance().UnregisterSubsystem(&a);
    wpi::cmd::CommandScheduler::GetInstance().UnregisterSubsystem(&b);
  }
  SECTION("unset subsystem throws") {
    SmartMotorControllerConfig cfg;
    CHECK_THROWS_AS(cfg.GetSubsystem(), exceptions::SmartMotorControllerConfigurationException);
  }
}

// ---- Item 21: exponential profile tuning defaults ------------------------------------------

TEST_CASE("SMCRegression.ExponentialProfileTuningDefaults", "[SMCRegression]") {
  auto name = UniqueName("ExpoDefaults");
  SmartMotorControllerConfig cfg;
  cfg.WithFeedback(1.0, 0.0, 0.0)
      .WithExponentialProfile(2.0, 0.5, 10_V)
      .WithTelemetry(name, Verbosity::HIGH);
  FakeSmartMotorController smc{&cfg};
  smc.SetupTelemetry(DataRoot(), TuningRoot());
  auto tuning = TuningRoot()->GetSubTable(name);
  CHECK(tuning->GetEntry("closedloop/motionprofile/kV").GetDouble(-1.0) == Catch::Approx(2.0));
  CHECK(tuning->GetEntry("closedloop/motionprofile/kA").GetDouble(-1.0) == Catch::Approx(0.5));
  CHECK(tuning->GetEntry("closedloop/motionprofile/maxInput").GetDouble(-1.0) ==
        Catch::Approx(10.0));
  smc.Close();
}

// ---- Item 22: encoder synchronization runs before the running check ------------------------

TEST_CASE("SMCRegression.SynchronizeBeforeRunningCheck", "[SMCRegression]") {
  SmartMotorControllerConfig cfg;
  FakeSmartMotorController smc{&cfg};
  smc.SetRunning(false);
  smc.IterateClosedLoopController();
  CHECK(smc.synchronizeCalls == 1);
}

}  // namespace yams::test
