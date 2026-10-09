// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Regression tests for SparkWrapper behaviour that differed from the Java reference
// (yams/java/yams/core/motorcontrollers/local/SparkWrapper.java). Values the wrapper computes are
// read back from the simulated SPARK's applied configuration (configAccessor) where possible.
// These tests only use API that also existed before the fixes, so each can be run against the old
// code to confirm it catches the bug.

#include <rev/SparkMax.h>
#include <rev/SplineEncoder.h>
#include <rev/sim/SparkAbsoluteEncoderSim.h>
#include <rev/sim/SparkRelativeEncoderSim.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <any>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <wpi/hardware/bus/CANPort.hpp>
#include <wpi/math/controller/ArmFeedforward.hpp>
#include <wpi/math/controller/SimpleMotorFeedforward.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/math/util/MathUtil.hpp>
#include <wpi/simulation/AlertSim.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/moment_of_inertia.hpp>
#include <wpi/units/temperature.hpp>
#include <wpi/units/time.hpp>
#include <wpi/units/voltage.hpp>

#include "helpers/FakeSmartMotorController.h"
#include "helpers/MockHardware.h"
#include "helpers/MotorControllerFactory.h"
#include "yams/exceptions.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/motorcontrollers/local/SparkWrapper.hpp"

namespace yams::test {

namespace {

using motorcontrollers::local::SparkWrapper;
using MotorMode = SmartMotorControllerConfig::MotorMode;
using SimpleFF = wpi::math::SimpleMotorFeedforward<wpi::units::turns>;
using ConfigException = exceptions::SmartMotorControllerConfigurationException;
using rev::spark::ClosedLoopSlot;

/** Motor rotations per mechanism rotation used by most tests. */
constexpr double kRatio = 12.0;

/**
 * CAN IDs on a bus no other test uses. REVLib's encoder sims look a SPARK's simulated encoders up
 * by device, so a CAN ID reused from an earlier test can resolve to that test's stale sim device.
 */
int NextSparkParityCanId() {
  static int next = 1;
  if (next >= kReservedCanIdStart) throw std::runtime_error("Out of SparkParity CAN IDs");
  return next++;
}

std::unique_ptr<rev::spark::SparkMax> MakeSparkMax() {
  return std::make_unique<rev::spark::SparkMax>(wpi::CANPort::CAN_S1, NextSparkParityCanId(),
                                                rev::spark::SparkLowLevel::MotorType::kBrushless);
}

/** Closed-loop config with a mechanism gearing of @p reduction motor rotations per rotation. */
SmartMotorControllerConfig BaseConfig(double reduction = kRatio) {
  SmartMotorControllerConfig cfg;
  cfg.WithMotorGearing(gearing::MechanismGearing{reduction})
      .WithStatorCurrentLimit(40_A)
      .WithZeroPower(MotorMode::BRAKE)
      .WithClosedLoopMode();
  return cfg;
}

SimpleFF MakeSimpleFF(double kS, double kV, double kA) {
  return SimpleFF{wpi::units::volt_t{kS}, wpi::units::unit_t<SimpleFF::kv_unit>{kV},
                  wpi::units::unit_t<SimpleFF::ka_unit>{kA}};
}

/** Wait for a (possibly asynchronously applied) SPARK value to reach @p expected. */
void CheckDevice(double expected, const std::function<double()>& actual, const std::string& what) {
  auto near = [&] {
    return std::abs(actual() - expected) <= 1e-4 * std::max(1.0, std::abs(expected));
  };
  for (int i = 0; i < 200 && !near(); i++) std::this_thread::sleep_for(std::chrono::milliseconds(5));
  INFO(what << ": expected " << expected << " on the SPARK but was " << actual());
  CHECK(near());
}

/** Reset the SPARK so later tests that reuse its CAN ID start from defaults. */
void ResetSpark(rev::spark::SparkMax& spark, bool alternateEncoder = false) {
  rev::spark::SparkMaxConfig defaults;
  // Once its alternate encoder is created, a SPARK MAX only accepts alternate encoder data port
  // configurations.
  if (alternateEncoder) defaults.alternateEncoder.SetSparkMaxDataPortConfig();
  spark.Configure(defaults, rev::ResetMode::kResetSafeParameters,
                  rev::PersistMode::kNoPersistParameters);
}

struct SparkParityFixture {
  SparkParityFixture() { InitializeHardware(); }
  ~SparkParityFixture() { TeardownHardware(); }
};

}  // namespace

// ---- P0: the SPARK runs in raw feedback sensor units ----------------------------------------

TEST_CASE_METHOD(SparkParityFixture, "SparkParity.GainsInFeedbackSensorUnits", "[SparkParity]") {
  auto spark = MakeSparkMax();
  auto cfg = BaseConfig();
  cfg.WithFeedback(2.4, 0.6, 0.12).WithFeedforward(MakeSimpleFF(0.1, 0.6, 0.06));
  {
    SparkWrapper smc{spark.get(), wpi::math::DCMotor::NEO(1), &cfg};
    auto& closedLoop = spark->configAccessor.closedLoop;
    // Volts per mechanism rotation -> duty cycle per motor rotation: / (12 V * 12).
    const double scale = 12.0 * kRatio;
    CheckDevice(2.4 / scale, [&] { return closedLoop.GetP(ClosedLoopSlot::kSlot0); }, "kP");
    // REVLib's simulated SPARK applies kI/kD per second, so they are not scaled by the 1 ms loop.
    CheckDevice(0.6 / scale, [&] { return closedLoop.GetI(ClosedLoopSlot::kSlot0); }, "kI");
    CheckDevice(0.12 / scale, [&] { return closedLoop.GetD(ClosedLoopSlot::kSlot0); }, "kD");
    // Volts per mechanism rotation/s -> volts per motor RPM.
    CheckDevice(0.1, [&] { return closedLoop.feedForward.getkS(ClosedLoopSlot::kSlot0); }, "kS");
    CheckDevice(0.6 / (kRatio * 60.0),
                [&] { return closedLoop.feedForward.getkV(ClosedLoopSlot::kSlot0); }, "kV");
    CheckDevice(0.06 / (kRatio * 60.0),
                [&] { return closedLoop.feedForward.getkA(ClosedLoopSlot::kSlot0); }, "kA");

    // The motor's encoder reports motor rotations; the wrapper converts to the mechanism.
    smc.SetEncoderPosition(0.25_tr);
    CheckDevice(0.25 * kRatio, [&] { return spark->GetEncoder().GetPosition().Get(); },
                "raw encoder position");
    CHECK(smc.GetMechanismPosition().value() == Catch::Approx(0.25).margin(1e-6));
  }
  ResetSpark(*spark);
}

TEST_CASE_METHOD(SparkParityFixture, "SparkParity.VoltageCompensationAppliedAndScalesGains",
                 "[SparkParity]") {
  auto spark = MakeSparkMax();
  auto cfg = BaseConfig();
  cfg.WithFeedback(2.0, 0.0, 0.0).WithVoltageCompensation(10_V);
  {
    SparkWrapper smc{spark.get(), wpi::math::DCMotor::NEO(1), &cfg};
    CheckDevice(1.0, [&] { return spark->configAccessor.GetVoltageCompensationEnabled() ? 1.0 : 0.0; },
                "voltage compensation enabled");
    CheckDevice(10.0, [&] { return spark->configAccessor.GetVoltageCompensation(); },
                "voltage compensation");
    CheckDevice(2.0 / (10.0 * kRatio),
                [&] { return spark->configAccessor.closedLoop.GetP(ClosedLoopSlot::kSlot0); },
                "kP scaled by the compensated voltage");
  }
  ResetSpark(*spark);
}

TEST_CASE_METHOD(SparkParityFixture, "SparkParity.ArmFeedforwardCosineRatio", "[SparkParity]") {
  auto spark = MakeSparkMax();
  auto cfg = BaseConfig();
  cfg.WithFeedback(1.0, 0.0, 0.0)
      .WithFeedforward(wpi::math::ArmFeedforward{
          0_V, 0.5_V, wpi::units::unit_t<wpi::math::ArmFeedforward::kv_unit>{0.0},
          wpi::units::unit_t<wpi::math::ArmFeedforward::ka_unit>{0.0}});
  {
    SparkWrapper smc{spark.get(), wpi::math::DCMotor::NEO(1), &cfg};
    auto& ff = spark->configAccessor.closedLoop.feedForward;
    CheckDevice(0.5, [&] { return ff.getkCos(ClosedLoopSlot::kSlot0); }, "kCos");
    // kCos needs the mechanism angle in rotations from the motor's rotations.
    CheckDevice(1.0 / kRatio, [&] { return ff.getkCosRatio(ClosedLoopSlot::kSlot0); },
                "kCosRatio");

    smc.SetKg(0.3);
    CheckDevice(0.3, [&] { return ff.getkCos(ClosedLoopSlot::kSlot0); }, "live kCos");
    CheckDevice(1.0 / kRatio, [&] { return ff.getkCosRatio(ClosedLoopSlot::kSlot0); },
                "live kCosRatio");
    CHECK(cfg.GetArmFeedforward(SmartMotorControllerConfig::ClosedLoopControllerSlot::SLOT_0)
              ->GetKg()
              .value() == Catch::Approx(0.3));
  }
  ResetSpark(*spark);
}

TEST_CASE_METHOD(SparkParityFixture, "SparkParity.SetpointsAndGainsSwitchUnits", "[SparkParity]") {
  auto spark = MakeSparkMax();
  auto cfg = BaseConfig();
  cfg.WithFeedback(2.4, 0.0, 0.0);
  {
    SparkWrapper smc{spark.get(), wpi::math::DCMotor::NEO(1), &cfg};
    auto& controller = spark->GetClosedLoopController();
    auto& closedLoop = spark->configAccessor.closedLoop;

    smc.SetVelocity(wpi::units::turns_per_second_t{1.0});
    CheckDevice(kRatio * 60.0, [&] { return controller.GetSetpoint().Get(); },
                "velocity setpoint in motor RPM");
    // Velocity error is in RPM, so the same YAMS gain is 60x smaller on the SPARK.
    CheckDevice(2.4 / (12.0 * kRatio * 60.0), [&] { return closedLoop.GetP(ClosedLoopSlot::kSlot0); },
                "velocity kP");

    smc.SetPosition(0.5_tr);
    CheckDevice(0.5 * kRatio, [&] { return controller.GetSetpoint().Get(); },
                "position setpoint in motor rotations");
    CheckDevice(2.4 / (12.0 * kRatio), [&] { return closedLoop.GetP(ClosedLoopSlot::kSlot0); },
                "position kP");
  }
  ResetSpark(*spark);
}

TEST_CASE_METHOD(SparkParityFixture, "SparkParity.MAXMotionConstraintsInFeedbackSensorRPM",
                 "[SparkParity]") {
  auto spark = MakeSparkMax();
  auto cfg = BaseConfig();
  cfg.WithFeedback(1.0, 0.0, 0.0)
      .WithTrapezoidProfile(wpi::units::turns_per_second_t{1.0},
                            wpi::units::turns_per_second_squared_t{2.0});
  {
    SparkWrapper smc{spark.get(), wpi::math::DCMotor::NEO(1), &cfg};
    auto& maxMotion = spark->configAccessor.closedLoop.maxMotion;
    CheckDevice(1.0 * kRatio * 60.0, [&] { return maxMotion.GetCruiseVelocity(ClosedLoopSlot::kSlot0); },
                "cruise velocity");
    CheckDevice(2.0 * kRatio * 60.0,
                [&] { return maxMotion.GetMaxAcceleration(ClosedLoopSlot::kSlot0); },
                "max acceleration");

    // Live profile setters keep the config and the SPARK in step.
    smc.SetMotionProfileMaxVelocity(wpi::units::turns_per_second_t{2.0});
    CHECK(cfg.GetTrapMaxVelocityTurns()->value() == Catch::Approx(2.0));
    CheckDevice(2.0 * kRatio * 60.0, [&] { return maxMotion.GetCruiseVelocity(ClosedLoopSlot::kSlot0); },
                "live cruise velocity");
    smc.SetMotionProfileMaxAcceleration(wpi::units::turns_per_second_squared_t{3.0});
    CHECK(cfg.GetTrapMaxAccelTurns()->value() == Catch::Approx(3.0));
    CHECK(cfg.GetTrapMaxVelocityTurns()->value() == Catch::Approx(2.0));
    CheckDevice(3.0 * kRatio * 60.0,
                [&] { return maxMotion.GetMaxAcceleration(ClosedLoopSlot::kSlot0); },
                "live max acceleration");
  }
  ResetSpark(*spark);
}

TEST_CASE_METHOD(SparkParityFixture, "SparkParity.SimulatedMAXMotionFollowsProfile",
                 "[SparkParity]") {
  auto spark = MakeSparkMax();
  auto cfg = BaseConfig();
  cfg.WithFeedback(20.0, 0.0, 0.0)
      .WithMOI(0.001_kg_sq_m)
      .WithTrapezoidProfile(wpi::units::turns_per_second_t{1.0},
                            wpi::units::turns_per_second_squared_t{2.0});
  {
    SparkWrapper smc{spark.get(), wpi::math::DCMotor::NEO(1), &cfg};
    smc.SetPosition(1_tr);
    smc.SimIterate();
    // REVLib's MAXMotion simulation does not follow the profile, so the wrapper sends the
    // profile's points with position control.
    CHECK(spark->GetClosedLoopController().GetControlType() ==
          rev::spark::SparkLowLevel::ControlType::kPosition);
    const double firstSetpoint = spark->GetClosedLoopController().GetSetpoint().Get();
    INFO("first profile setpoint " << firstSetpoint);
    CHECK(firstSetpoint > 0.0);
    CHECK(firstSetpoint < 0.1 * kRatio);
    for (int i = 0; i < 150; i++) smc.SimIterate();
    INFO("position after 3 s " << smc.GetMechanismPosition().value());
    CHECK(smc.GetMechanismPosition().value() == Catch::Approx(1.0).margin(0.05));
  }
  ResetSpark(*spark);
}

TEST_CASE_METHOD(SparkParityFixture, "SparkParity.SoftLimitsInFeedbackSensorRotations",
                 "[SparkParity]") {
  auto spark = MakeSparkMax();
  auto cfg = BaseConfig();
  cfg.WithFeedback(1.0, 0.0, 0.0).WithMechanismLimits(-0.5_tr, 0.75_tr);
  {
    SparkWrapper smc{spark.get(), wpi::math::DCMotor::NEO(1), &cfg};
    auto& softLimit = spark->configAccessor.softLimit;
    CheckDevice(-0.5 * kRatio, [&] { return softLimit.GetReverseSoftLimit(); }, "reverse");
    CheckDevice(0.75 * kRatio, [&] { return softLimit.GetForwardSoftLimit(); }, "forward");

    smc.SetMechanismLimits(-0.25_tr, 0.5_tr);
    CheckDevice(-0.25 * kRatio, [&] { return softLimit.GetReverseSoftLimit(); }, "live reverse");
    CheckDevice(0.5 * kRatio, [&] { return softLimit.GetForwardSoftLimit(); }, "live forward");
    smc.SetMechanismUpperLimit(0.6_tr);
    CheckDevice(0.6 * kRatio, [&] { return softLimit.GetForwardSoftLimit(); }, "upper");
    smc.SetMechanismLowerLimit(-0.4_tr);
    CheckDevice(-0.4 * kRatio, [&] { return softLimit.GetReverseSoftLimit(); }, "lower");
  }
  ResetSpark(*spark);
}

TEST_CASE_METHOD(SparkParityFixture, "SparkParity.ClosedLoopToleranceInFeedbackSensorRotations",
                 "[SparkParity]") {
  auto spark = MakeSparkMax();
  auto cfg = BaseConfig();
  cfg.WithFeedback(1.0, 0.0, 0.0).WithClosedLoopTolerance(0.01_tr);
  {
    SparkWrapper smc{spark.get(), wpi::math::DCMotor::NEO(1), &cfg};
    auto& closedLoop = spark->configAccessor.closedLoop;
    CheckDevice(0.01 * kRatio,
                [&] { return closedLoop.GetAllowedClosedLoopError(ClosedLoopSlot::kSlot0); },
                "allowed closed loop error");
    CheckDevice(0.01 * kRatio,
                [&] { return closedLoop.maxMotion.GetAllowedProfileError(ClosedLoopSlot::kSlot0); },
                "allowed profile error");
  }
  ResetSpark(*spark);
}

TEST_CASE_METHOD(SparkParityFixture, "SparkParity.SetMechanismGearingRewritesScaledValues",
                 "[SparkParity]") {
  auto spark = MakeSparkMax();
  auto cfg = BaseConfig();
  cfg.WithFeedback(2.4, 0.0, 0.0).WithMechanismLimits(-0.5_tr, 0.5_tr);
  {
    SparkWrapper smc{spark.get(), wpi::math::DCMotor::NEO(1), &cfg};
    smc.SetMechanismGearing(gearing::MechanismGearing{6.0});
    CheckDevice(2.4 / (12.0 * 6.0),
                [&] { return spark->configAccessor.closedLoop.GetP(ClosedLoopSlot::kSlot0); },
                "kP after the gearing change");
    CheckDevice(0.5 * 6.0, [&] { return spark->configAccessor.softLimit.GetForwardSoftLimit(); },
                "forward soft limit after the gearing change");
  }
  ResetSpark(*spark);
}

// ---- External encoders ------------------------------------------------------------------------

TEST_CASE_METHOD(SparkParityFixture, "SparkParity.AbsoluteEncoderRangeOffset", "[SparkParity]") {
  for (double discontinuity : {0.5, 1.0}) {
    auto spark = MakeSparkMax();
    auto cfg = BaseConfig(1.0);
    cfg.WithFeedback(1.0, 0.0, 0.0)
        .WithExternalEncoder(&spark->GetAbsoluteEncoder())
        .WithExternalEncoderDiscontinuityPoint(wpi::units::turn_t{discontinuity});
    {
      SparkWrapper smc{spark.get(), wpi::math::DCMotor::NEO(1), &cfg};
      // The range offset is the middle of the reported range: [-0.5, 0.5) -> 0, [0, 1) -> 0.5.
      CheckDevice(discontinuity - 0.5,
                  [&] { return spark->configAccessor.absoluteEncoder.GetRangeOffset(); },
                  "range offset for discontinuity point " + std::to_string(discontinuity));
    }
    ResetSpark(*spark);
  }
}

TEST_CASE_METHOD(SparkParityFixture, "SparkParity.ExternalFeedbackReadsExternalEncoder",
                 "[SparkParity]") {
  auto spark = MakeSparkMax();
  auto cfg = BaseConfig();
  // Two encoder rotations per mechanism rotation.
  cfg.WithFeedback(1.0, 0.0, 0.0)
      .WithExternalEncoder(&spark->GetAbsoluteEncoder())
      .WithExternalEncoderGearing(2.0);
  {
    SparkWrapper smc{spark.get(), wpi::math::DCMotor::NEO(1), &cfg};
    CHECK(spark->configAccessor.closedLoop.GetFeedbackSensor() ==
          rev::spark::FeedbackSensor::kAbsoluteEncoder);
    rev::spark::SparkAbsoluteEncoderSim absSim{spark.get()};
    absSim.SetPosition(0.4);
    CHECK(smc.GetMechanismPosition().value() == Catch::Approx(0.2).margin(1e-6));
    REQUIRE(smc.GetExternalEncoderPosition().has_value());
    CHECK(smc.GetExternalEncoderPosition()->value() == Catch::Approx(72.0).margin(1e-4));
  }
  ResetSpark(*spark);
}

TEST_CASE_METHOD(SparkParityFixture, "SparkParity.AbsoluteEncoderSeedsMotorEncoder",
                 "[SparkParity]") {
  auto spark = MakeSparkMax();
  rev::spark::SparkAbsoluteEncoderSim{spark.get()}.SetPosition(0.25);
  auto cfg = BaseConfig();
  cfg.WithFeedback(1.0, 0.0, 0.0).WithExternalEncoder(&spark->GetAbsoluteEncoder());
  {
    SparkWrapper smc{spark.get(), wpi::math::DCMotor::NEO(1), &cfg};
    // Without a starting position the motor's encoder starts from the absolute encoder, in motor
    // rotations.
    CheckDevice(0.25 * kRatio, [&] { return spark->GetEncoder().GetPosition().Get(); },
                "seeded motor encoder");
  }
  ResetSpark(*spark);
}

TEST_CASE_METHOD(SparkParityFixture, "SparkParity.SynchronizeRelativeEncoderFromAbsolute",
                 "[SparkParity]") {
  auto spark = MakeSparkMax();
  auto cfg = BaseConfig();
  cfg.WithFeedback(1.0, 0.0, 0.0)
      .WithExponentialProfile(12_V, wpi::math::DCMotor::NEO(1), 0.001_kg_sq_m)
      .WithExternalEncoder(&spark->GetAbsoluteEncoder())
      .WithFeedbackSynchronizationThreshold(0.01_tr);
  {
    SparkWrapper smc{spark.get(), wpi::math::DCMotor::NEO(1), &cfg};
    smc.StopClosedLoopController();
    rev::spark::SparkAbsoluteEncoderSim{spark.get()}.SetPosition(0.3);
    smc.SynchronizeRelativeEncoder();
    // The motor's encoder is more than the threshold away, so it is reseeded in motor rotations.
    CheckDevice(0.3 * kRatio, [&] { return spark->GetEncoder().GetPosition().Get(); },
                "synchronized motor encoder");
  }
  ResetSpark(*spark);
}

TEST_CASE_METHOD(SparkParityFixture, "SparkParity.SetEncoderPositionSendsAbsoluteZeroOffset",
                 "[SparkParity]") {
  auto spark = MakeSparkMax();
  auto cfg = BaseConfig(1.0);
  cfg.WithFeedback(1.0, 0.0, 0.0).WithExternalEncoder(&spark->GetAbsoluteEncoder());
  {
    SparkWrapper smc{spark.get(), wpi::math::DCMotor::NEO(1), &cfg};
    auto& absoluteEncoder = spark->configAccessor.absoluteEncoder;
    CheckDevice(0.0, [&] { return absoluteEncoder.GetZeroOffset(); }, "initial zero offset");
    const double target = -0.3;
    // Zero offsets are in [0, 1) rotations.
    const double expected =
        wpi::math::InputModulus(smc.GetMechanismPosition().value() - target, 0.0, 1.0);
    smc.SetEncoderPosition(wpi::units::turn_t{target});
    CheckDevice(expected, [&] { return absoluteEncoder.GetZeroOffset(); },
                "zero offset after SetEncoderPosition");
  }
  ResetSpark(*spark);
}

TEST_CASE_METHOD(SparkParityFixture, "SparkParity.ExternalGearingWithDiscontinuityRaisesAlert",
                 "[SparkParity]") {
  auto spark = MakeSparkMax();
  auto cfg = BaseConfig();
  cfg.WithFeedback(1.0, 0.0, 0.0)
      .WithExternalEncoder(&spark->GetAbsoluteEncoder())
      .WithExternalEncoderGearing(2.0)
      .WithExternalEncoderDiscontinuityPoint(0.5_tr);
  auto gearingAlertActive = [] {
    for (auto& alert : wpi::sim::AlertSim::GetActive()) {
      if (alert.level == wpi::util::Alert::Level::HIGH &&
          alert.text.find("gearing") != std::string::npos)
        return true;
    }
    return false;
  };
  {
    SparkWrapper smc{spark.get(), wpi::math::DCMotor::NEO(1), &cfg};
    CHECK(gearingAlertActive());
  }
  CHECK_FALSE(gearingAlertActive());
  ResetSpark(*spark);
}

TEST_CASE_METHOD(SparkParityFixture, "SparkParity.QuadratureAndDetachedEncoders",
                 "[SparkParity]") {
  SECTION("Alternate encoder") {
    auto spark = MakeSparkMax();
    auto cfg = BaseConfig();
    cfg.WithFeedback(1.0, 0.0, 0.0).WithExternalEncoder(&spark->GetAlternateEncoder());
    {
      std::unique_ptr<SparkWrapper> smc;
      REQUIRE_NOTHROW(smc = std::make_unique<SparkWrapper>(spark.get(),
                                                           wpi::math::DCMotor::NEO(1), &cfg));
      CHECK(spark->configAccessor.closedLoop.GetFeedbackSensor() ==
            rev::spark::FeedbackSensor::kAlternateOrExternalEncoder);
      smc->SetEncoderPosition(0.3_tr);
      CHECK(smc->GetMechanismPosition().value() == Catch::Approx(0.3).margin(1e-4));
    }
    ResetSpark(*spark, true);
  }
  SECTION("Detached encoder") {
    auto spark = MakeSparkMax();
    rev::detached::SplineEncoder encoder{wpi::CANPort::CAN_S1, NextSparkParityCanId()};
    auto cfg = BaseConfig();
    cfg.WithFeedback(1.0, 0.0, 0.0)
        .WithExternalEncoder(&encoder)
        .WithExternalEncoderDiscontinuityPoint(0.5_tr);
    {
      std::unique_ptr<SparkWrapper> smc;
      REQUIRE_NOTHROW(smc = std::make_unique<SparkWrapper>(spark.get(),
                                                           wpi::math::DCMotor::NEO(1), &cfg));
      CHECK(spark->configAccessor.closedLoop.GetFeedbackSensor() ==
            rev::spark::FeedbackSensor::kDetachedAbsoluteEncoder);
      // The simulated detached encoder reports within its range: 0.7 rotations -> -0.3.
      smc->SetEncoderPosition(0.7_tr);
      CHECK(smc->GetMechanismPosition().value() == Catch::Approx(-0.3).margin(1e-6));
    }
    ResetSpark(*spark);
  }
}

TEST_CASE_METHOD(SparkParityFixture, "SparkParity.ContinuousWrappingOnlyForAbsoluteFeedback",
                 "[SparkParity]") {
  SECTION("Motor encoder feedback wraps the setpoint instead") {
    auto spark = MakeSparkMax();
    auto cfg = BaseConfig(10.0);
    cfg.WithFeedback(1.0, 0.0, 0.0).WithContinuousWrapping(-0.5_tr, 0.5_tr);
    {
      SparkWrapper smc{spark.get(), wpi::math::DCMotor::NEO(1), &cfg};
      // The motor's encoder turns 10 times per mechanism rotation, so SPARK wrapping (every
      // sensor rotation) would be wrong.
      CheckDevice(0.0, [&] {
        return spark->configAccessor.closedLoop.GetPositionWrappingEnabled() ? 1.0 : 0.0;
      }, "position wrapping enabled");
      smc.SetPosition(0.9_tr);
      // Nearest equivalent of 0.9 rotations from 0 is -0.1 rotations = -1 motor rotation.
      CheckDevice(-1.0, [&] { return spark->GetClosedLoopController().GetSetpoint().Get(); },
                  "wrapped setpoint");
    }
    ResetSpark(*spark);
  }
  SECTION("Absolute encoder feedback wraps on the SPARK") {
    auto spark = MakeSparkMax();
    auto cfg = BaseConfig();
    cfg.WithFeedback(1.0, 0.0, 0.0)
        .WithExternalEncoder(&spark->GetAbsoluteEncoder())
        .WithContinuousWrapping(-0.5_tr, 0.5_tr);
    {
      SparkWrapper smc{spark.get(), wpi::math::DCMotor::NEO(1), &cfg};
      CheckDevice(1.0, [&] {
        return spark->configAccessor.closedLoop.GetPositionWrappingEnabled() ? 1.0 : 0.0;
      }, "position wrapping enabled");
    }
    ResetSpark(*spark);
  }
}

// ---- Validation -------------------------------------------------------------------------------

TEST_CASE_METHOD(SparkParityFixture, "SparkParity.UnsupportedOptionsThrow", "[SparkParity]") {
  auto spark = MakeSparkMax();
  auto cfg = BaseConfig();
  cfg.WithFeedback(1.0, 0.0, 0.0);
  SECTION("Supply current limit") {
    cfg.WithSupplyCurrentLimit(30_A);
    CHECK_THROWS_AS(SparkWrapper(spark.get(), wpi::math::DCMotor::NEO(1), &cfg), ConfigException);
  }
  SECTION("Relative encoder inversion") {
    cfg.WithEncoderInverted(true);
    CHECK_THROWS_AS(SparkWrapper(spark.get(), wpi::math::DCMotor::NEO(1), &cfg),
                    std::invalid_argument);
  }
  SECTION("Closed loop control period without a RoboRIO controller") {
    cfg.WithClosedLoopControlPeriod(10_ms);
    CHECK_THROWS_AS(SparkWrapper(spark.get(), wpi::math::DCMotor::NEO(1), &cfg), ConfigException);
  }
  SECTION("Closed loop maximum voltage without a RoboRIO controller") {
    cfg.WithClosedLoopMaxVoltage(6_V);
    CHECK_THROWS_AS(SparkWrapper(spark.get(), wpi::math::DCMotor::NEO(1), &cfg), ConfigException);
  }
  SECTION("Feedback synchronization threshold without a RoboRIO controller") {
    cfg.WithFeedbackSynchronizationThreshold(0.01_tr);
    CHECK_THROWS_AS(SparkWrapper(spark.get(), wpi::math::DCMotor::NEO(1), &cfg), ConfigException);
  }
  SECTION("Temperature cutoff without a profile") {
    cfg.WithTemperatureCutoff(wpi::units::celsius_t{80});
    CHECK_THROWS_AS(SparkWrapper(spark.get(), wpi::math::DCMotor::NEO(1), &cfg), ConfigException);
  }
  SECTION("Unknown follower type") {
    int notASpark = 0;
    cfg.WithFollowers({{std::any(&notASpark), false}});
    CHECK_THROWS_AS(SparkWrapper(spark.get(), wpi::math::DCMotor::NEO(1), &cfg),
                    std::invalid_argument);
  }
  ResetSpark(*spark);
}

TEST_CASE_METHOD(SparkParityFixture, "SparkParity.SupplyCurrentIsUnsupported", "[SparkParity]") {
  auto spark = MakeSparkMax();
  auto cfg = BaseConfig();
  cfg.WithFeedback(1.0, 0.0, 0.0);
  {
    SparkWrapper smc{spark.get(), wpi::math::DCMotor::NEO(1), &cfg};
    CHECK_FALSE(smc.GetSupplyCurrent().has_value());
  }
  ResetSpark(*spark);
}

TEST_CASE_METHOD(SparkParityFixture, "SparkParity.ResetPreviousConfig", "[SparkParity]") {
  auto spark = MakeSparkMax();
  rev::spark::SparkMaxConfig previous;
  previous.SmartCurrentLimit(33);
  spark->Configure(previous, rev::ResetMode::kResetSafeParameters,
                   rev::PersistMode::kNoPersistParameters);
  REQUIRE(spark->configAccessor.GetSmartCurrentLimit() == 33);

  SmartMotorControllerConfig cfg;
  cfg.WithMotorGearing(gearing::MechanismGearing{kRatio})
      .WithClosedLoopMode()
      .WithFeedback(1.0, 0.0, 0.0)
      .WithResetPreviousConfig(false);
  {
    SparkWrapper smc{spark.get(), wpi::math::DCMotor::NEO(1), &cfg};
    // Nothing in the YAMS config sets a current limit, so the SPARK keeps its previous one.
    CHECK(spark->configAccessor.GetSmartCurrentLimit() == 33);
  }
  ResetSpark(*spark);
}

// ---- Hardware details -------------------------------------------------------------------------

TEST_CASE_METHOD(SparkParityFixture, "SparkParity.MinionAdvancesCommutation", "[SparkParity]") {
  auto spark = MakeSparkMax();
  auto cfg = BaseConfig();
  cfg.WithFeedback(1.0, 0.0, 0.0);
  {
    SparkWrapper smc{spark.get(), wpi::math::DCMotor::Minion(1), &cfg};
    CheckDevice(120.0, [&] { return spark->configAccessor.GetAdvanceCommutation(); },
                "advance commutation");
  }
  ResetSpark(*spark);
}

TEST_CASE_METHOD(SparkParityFixture, "SparkParity.FollowersGetIdleModeAndAreCleared",
                 "[SparkParity]") {
  auto leader = MakeSparkMax();
  auto follower = MakeSparkMax();
  auto cfg = BaseConfig();
  cfg.WithFeedback(1.0, 0.0, 0.0).WithFollowers({{std::any(follower.get()), true}});
  {
    SparkWrapper smc{leader.get(), wpi::math::DCMotor::NEO(1), &cfg};
    CheckDevice(leader->GetDeviceId(),
                [&] { return follower->configAccessor.GetFollowerModeLeaderId(); }, "leader id");
    CheckDevice(1.0, [&] {
      return follower->configAccessor.GetIdleMode() == rev::spark::SparkBaseConfig::IdleMode::kBrake
                 ? 1.0
                 : 0.0;
    }, "follower brake mode");
    // Applying the config again must not reconfigure the followers.
    CHECK(cfg.GetFollowers().empty());
  }
  ResetSpark(*follower);
  ResetSpark(*leader);
}

// ---- Live setters -----------------------------------------------------------------------------

TEST_CASE_METHOD(SparkParityFixture, "SparkParity.LiveSettersScaleUpdateConfigAndForward",
                 "[SparkParity]") {
  auto spark = MakeSparkMax();
  SmartMotorControllerConfig followerCfg;
  FakeSmartMotorController looseFollower{&followerCfg};
  auto cfg = BaseConfig();
  cfg.WithFeedback(2.4, 0.0, 0.0)
      .WithFeedforward(MakeSimpleFF(0.0, 0.6, 0.0))
      .WithLooselyCoupledFollowers({&looseFollower});
  {
    SparkWrapper smc{spark.get(), wpi::math::DCMotor::NEO(1), &cfg};
    auto& closedLoop = spark->configAccessor.closedLoop;

    smc.SetKp(4.8);
    CheckDevice(4.8 / (12.0 * kRatio), [&] { return closedLoop.GetP(ClosedLoopSlot::kSlot0); },
                "live kP");
    CHECK(cfg.GetKp() == Catch::Approx(4.8));
    CHECK(looseFollower.setKpCalls == 1);

    smc.SetKv(1.2);
    CheckDevice(1.2 / (kRatio * 60.0),
                [&] { return closedLoop.feedForward.getkV(ClosedLoopSlot::kSlot0); }, "live kV");
    CHECK(cfg.GetSimpleFeedforward(SmartMotorControllerConfig::ClosedLoopControllerSlot::SLOT_0)
              ->GetKv()
              .value() == Catch::Approx(1.2));

    // Stopping the leader stops loosely coupled followers too.
    looseFollower.dutyCycle = 0.7;
    smc.SetDutyCycle(0.0);
    CHECK(looseFollower.dutyCycle == 0.0);
  }
  ResetSpark(*spark);
}

// ---- Simulation -------------------------------------------------------------------------------

TEST_CASE_METHOD(SparkParityFixture, "SparkParity.SimulationInputs", "[SparkParity]") {
  auto spark = MakeSparkMax();
  auto cfg = BaseConfig();
  cfg.WithFeedback(1.0, 0.0, 0.0);
  {
    SparkWrapper smc{spark.get(), wpi::math::DCMotor::NEO(1), &cfg};
    auto* supplier = smc.GetSimSupplier();
    REQUIRE(supplier != nullptr);

    // Voltage commands drive the mechanism simulation directly.
    smc.SetVoltage(6_V);
    CHECK(supplier->IsInputFed());
    CHECK(supplier->GetMechanismStatorVoltage().value() == Catch::Approx(6.0));
    for (int i = 0; i < 5; i++) {
      smc.SetVoltage(6_V);
      smc.SimIterate();
    }
    // Stator current comes from the mechanism simulation.
    CHECK(smc.GetStatorCurrent().value() == Catch::Approx(supplier->GetStatorCurrent().value()));

    // Simulated encoder velocity is set in motor RPM.
    smc.SetEncoderVelocity(wpi::units::turns_per_second_t{1.0});
    CHECK(rev::spark::SparkRelativeEncoderSim{spark.get()}.GetVelocity() ==
          Catch::Approx(kRatio * 60.0));
  }
  ResetSpark(*spark);
}

}  // namespace yams::test
