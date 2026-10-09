// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Regression tests for TalonFXWrapper / TalonFXSWrapper behaviour that differed from the Java
// reference (yams/java/yams/core/motorcontrollers/remote/TalonFX(S)Wrapper.java). Values the
// wrappers compute are read back from the simulated device's configuration. These tests only use
// API that also existed before the fixes, so each can be run against the old code to confirm it
// catches the bug. Tests of new API are in TalonWrapperApiTest.cpp.

#include <any>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <string>
#include <utility>
#include <vector>
#include <wpi/math/controller/ElevatorFeedforward.hpp>
#include <wpi/math/controller/SimpleMotorFeedforward.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/force.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/mass.hpp>
#include <wpi/units/temperature.hpp>
#include <wpi/units/time.hpp>
#include <wpi/units/voltage.hpp>

#include "helpers/FakeSmartMotorController.h"
#include "helpers/MockHardware.h"
#include "helpers/TalonTestHardware.h"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/motorcontrollers/remote/TalonFXSWrapper.hpp"
#include "yams/motorcontrollers/remote/TalonFXWrapper.hpp"

namespace yams::test {

namespace {

using motorcontrollers::remote::TalonFXSWrapper;
using motorcontrollers::remote::TalonFXWrapper;
using MotorMode = SmartMotorControllerConfig::MotorMode;
using Slot = SmartMotorControllerConfig::ClosedLoopControllerSlot;
using SimpleFF = wpi::math::SimpleMotorFeedforward<wpi::units::turns>;
using namespace ctre::phoenix6;

/** Motor rotations per mechanism rotation. */
constexpr double kRatio = 10.0;

SmartMotorControllerConfig BaseConfig() {
  SmartMotorControllerConfig cfg;
  cfg.WithMotorGearing(gearing::MechanismGearing{kRatio})
      .WithStatorCurrentLimit(40_A)
      .WithZeroPower(MotorMode::BRAKE)
      .WithClosedLoopMode();
  return cfg;
}

/** Elevator config: 0.2 m per mechanism rotation, gains per meter (linear closed loop). */
SmartMotorControllerConfig ElevatorConfig() {
  auto cfg = BaseConfig();
  using Elevator = wpi::math::ElevatorFeedforward;
  cfg.WithMechanismCircumference(0.2_m)
      .WithFeedback(10.0, 0.0, 1.0)
      .WithFeedforward(Elevator{0.1_V, 0.3_V, wpi::units::unit_t<Elevator::kv_unit>{2.0},
                                wpi::units::unit_t<Elevator::ka_unit>{0.5}});
  return cfg;
}

SimpleFF MakeSimpleFF(double kS, double kV, double kA) {
  return SimpleFF{wpi::units::volt_t{kS}, wpi::units::unit_t<SimpleFF::kv_unit>{kV},
                  wpi::units::unit_t<SimpleFF::ka_unit>{kA}};
}

const wpi::math::DCMotor kKraken = wpi::math::DCMotor::KrakenX60(1);
const wpi::math::DCMotor kNeo = wpi::math::DCMotor::NEO(1);

struct TalonParityFixture {
  TalonParityFixture() {
    InitializeHardware();
    TalonHardware().Reset();
  }
  ~TalonParityFixture() { TeardownHardware(); }

  TalonTestHardware& hw = TalonHardware();
};

}  // namespace

// ---- P0: gains ------------------------------------------------------------------------------

TEST_CASE_METHOD(TalonParityFixture, "TalonParity.LinearGainsPerMechanismRotation",
                 "[TalonParity]") {
  auto cfg = ElevatorConfig();
  TalonFXWrapper smc{&hw.fx, kKraken, &cfg};
  // 5 mechanism rotations per meter: per-meter gains are divided by 5.
  auto slot0 = ReadConfig(hw.fx).Slot0;
  CHECK(slot0.kP.value() == Catch::Approx(2.0).margin(1e-3));
  CHECK(slot0.kD.value() == Catch::Approx(0.2).margin(1e-3));
  CHECK(slot0.kV.value() == Catch::Approx(0.4).margin(1e-3));
  CHECK(slot0.kA.value() == Catch::Approx(0.1).margin(1e-3));
  CHECK(slot0.kG.value() == Catch::Approx(0.3).margin(1e-3));
  CHECK(slot0.GravityType == signals::GravityTypeValue::Elevator_Static);

  // Live setters convert too, and changing the circumference rewrites the gains.
  smc.SetKv(4.0);
  CHECK(ReadConfig(hw.fx).Slot0.kV.value() == Catch::Approx(0.8).margin(1e-3));
  smc.SetMechanismCircumference(0.4_m);
  CHECK(ReadConfig(hw.fx).Slot0.kP.value() == Catch::Approx(4.0).margin(1e-3));
}

TEST_CASE_METHOD(TalonParityFixture, "TalonParity.TalonFXSLinearGainsPerMechanismRotation",
                 "[TalonParity]") {
  auto cfg = ElevatorConfig();
  TalonFXSWrapper smc{&hw.fxs, kNeo, TalonFXSWrapper::MotorArrangement::NEO, &cfg};
  auto slot0 = ReadConfig(hw.fxs).Slot0;
  CHECK(slot0.kP.value() == Catch::Approx(2.0).margin(1e-3));
  CHECK(slot0.kV.value() == Catch::Approx(0.4).margin(1e-3));
}

TEST_CASE_METHOD(TalonParityFixture, "TalonParity.OnlyConfiguredSlotsWritten", "[TalonParity]") {
  configs::TalonFXConfiguration vendor;
  vendor.Slot1.kP = 5.0;
  vendor.Slot1.kV = 0.7;
  auto cfg = BaseConfig();
  cfg.WithVendorConfig(vendor).WithFeedback(2.0, 0.0, 0.0);
  TalonFXWrapper smc{&hw.fx, kKraken, &cfg};
  auto device = ReadConfig(hw.fx);
  CHECK(device.Slot0.kP.value() == Catch::Approx(2.0).margin(1e-3));
  // Slot 1 has no YAMS gains, so it keeps the vendor config's.
  CHECK(device.Slot1.kP.value() == Catch::Approx(5.0).margin(1e-3));
  CHECK(device.Slot1.kV.value() == Catch::Approx(0.7).margin(1e-3));
}

TEST_CASE_METHOD(TalonParityFixture, "TalonParity.KeepsDeviceConfigUnlessReset", "[TalonParity]") {
  configs::MotorOutputConfigs motorOutput;
  motorOutput.DutyCycleNeutralDeadband = 0.1;
  hw.fx.GetConfigurator().Apply(motorOutput);
  REQUIRE(ReadConfig(hw.fx).MotorOutput.DutyCycleNeutralDeadband.value() ==
          Catch::Approx(0.1).margin(1e-3));
  {
    auto cfg = BaseConfig();
    cfg.WithFeedback(1.0, 0.0, 0.0).WithResetPreviousConfig(false);
    TalonFXWrapper smc{&hw.fx, kKraken, &cfg};
    CHECK(ReadConfig(hw.fx).MotorOutput.DutyCycleNeutralDeadband.value() ==
          Catch::Approx(0.1).margin(1e-3));
  }
  {
    auto cfg = BaseConfig();
    // Resetting the previous configuration is the default.
    cfg.WithFeedback(1.0, 0.0, 0.0);
    TalonFXWrapper smc{&hw.fx, kKraken, &cfg};
    CHECK(ReadConfig(hw.fx).MotorOutput.DutyCycleNeutralDeadband.value() ==
          Catch::Approx(0.0).margin(1e-3));
  }
}

// ---- P0: TalonFXS commutation ---------------------------------------------------------------

TEST_CASE_METHOD(TalonParityFixture, "TalonParity.TalonFXSCommutationAndFeedback",
                 "[TalonParity]") {
  {
    auto cfg = BaseConfig();
    cfg.WithFeedback(1.0, 0.0, 0.0);
    TalonFXSWrapper smc{&hw.fxs, kNeo, TalonFXSWrapper::MotorArrangement::NEO, &cfg};
    auto device = ReadConfig(hw.fxs);
    CHECK(device.Commutation.MotorArrangement == signals::MotorArrangementValue::NEO_JST);
    CHECK(device.Commutation.AdvancedHallSupport == signals::AdvancedHallSupportValue::Disabled);
    // Without an external encoder the TalonFXS uses the motor's commutation sensor.
    CHECK(device.ExternalFeedback.ExternalFeedbackSensorSource ==
          signals::ExternalFeedbackSensorSourceValue::Commutation);
    CHECK(device.ExternalFeedback.SensorToMechanismRatio.value() ==
          Catch::Approx(kRatio).margin(1e-3));
    CHECK(device.ExternalFeedback.RotorToSensorRatio.value() == Catch::Approx(1.0).margin(1e-3));
  }
  {
    auto cfg = BaseConfig();
    cfg.WithFeedback(1.0, 0.0, 0.0);
    TalonFXSWrapper smc{&hw.fxs, wpi::math::DCMotor::Minion(1),
                        TalonFXSWrapper::MotorArrangement::Minion, &cfg};
    auto device = ReadConfig(hw.fxs);
    CHECK(device.Commutation.MotorArrangement == signals::MotorArrangementValue::Minion_JST);
    CHECK(device.Commutation.AdvancedHallSupport == signals::AdvancedHallSupportValue::Enabled);
  }
}

TEST_CASE_METHOD(TalonParityFixture, "TalonParity.TalonFXSEncoderInvertedSetsSensorPhase",
                 "[TalonParity]") {
  auto cfg = BaseConfig();
  cfg.WithFeedback(1.0, 0.0, 0.0).WithEncoderInverted(true);
  TalonFXSWrapper smc{&hw.fxs, kNeo, TalonFXSWrapper::MotorArrangement::NEO, &cfg};
  CHECK(ReadConfig(hw.fxs).ExternalFeedback.SensorPhase == signals::SensorPhaseValue::Opposed);
  smc.SetEncoderInverted(false);
  CHECK(ReadConfig(hw.fxs).ExternalFeedback.SensorPhase == signals::SensorPhaseValue::Aligned);
}

// ---- Motion Magic -------------------------------------------------------------------------

TEST_CASE_METHOD(TalonParityFixture, "TalonParity.TrapezoidProfilesUseMotionMagicRequests",
                 "[TalonParity]") {
  {
    auto cfg = BaseConfig();
    cfg.WithFeedback(1.0, 0.0, 0.0).WithTrapezoidProfile(1_tps, 2_tr_per_s_sq);
    TalonFXWrapper smc{&hw.fx, kKraken, &cfg};
    smc.SetVelocity(1_tps);
    CHECK(AppliedControlName(hw.fx) == "MotionMagicVelocityVoltage");
  }
  {
    auto cfg = BaseConfig();
    cfg.WithFeedback(1.0, 0.0, 0.0)
        .WithVelocityTrapezoidProfile(4_tr_per_s_sq,
                                      wpi::units::angular_jerk::turns_per_second_cubed_t{40});
    TalonFXWrapper smc{&hw.fx, kKraken, &cfg};
    smc.SetPosition(0.5_tr);
    CHECK(AppliedControlName(hw.fx) == "MotionMagicVoltage");
  }
}

TEST_CASE_METHOD(TalonParityFixture, "TalonParity.ProfileSettersUpdateConfig", "[TalonParity]") {
  {
    auto cfg = BaseConfig();
    cfg.WithFeedback(1.0, 0.0, 0.0).WithTrapezoidProfile(1_tps, 2_tr_per_s_sq);
    TalonFXWrapper smc{&hw.fx, kKraken, &cfg};
    smc.SetMotionProfileMaxVelocity(3_tps);
    CHECK(cfg.GetTrapMaxVelocityTurns()->value() == Catch::Approx(3.0).margin(1e-3));
    CHECK(cfg.GetTrapMaxAccelTurns()->value() == Catch::Approx(2.0).margin(1e-3));
    CHECK(ReadConfig(hw.fx).MotionMagic.MotionMagicCruiseVelocity.value() ==
          Catch::Approx(3.0).margin(1e-3));
  }
  {
    // A velocity profile stores the acceleration as its first constraint and the jerk second.
    auto cfg = BaseConfig();
    cfg.WithFeedback(1.0, 0.0, 0.0)
        .WithVelocityTrapezoidProfile(4_tr_per_s_sq,
                                      wpi::units::angular_jerk::turns_per_second_cubed_t{40});
    TalonFXSWrapper smc{&hw.fxs, kNeo, TalonFXSWrapper::MotorArrangement::NEO, &cfg};
    smc.SetMotionProfileMaxAcceleration(6_tr_per_s_sq);
    CHECK(cfg.GetTrapMaxVelocityTurns()->value() == Catch::Approx(6.0).margin(1e-3));
    CHECK(cfg.GetTrapMaxAccelTurns()->value() == Catch::Approx(40.0).margin(1e-3));
    CHECK(ReadConfig(hw.fxs).MotionMagic.MotionMagicAcceleration.value() ==
          Catch::Approx(6.0).margin(1e-3));
  }
}

TEST_CASE_METHOD(TalonParityFixture, "TalonParity.ExponentialProfileSetterUpdatesConfig",
                 "[TalonParity]") {
  {
    auto cfg = BaseConfig();
    cfg.WithFeedback(1.0, 0.0, 0.0).WithExponentialProfile(0.5, 0.05, 10_V);
    TalonFXWrapper smc{&hw.fx, kKraken, &cfg};
    smc.SetExponentialProfile(0.8, std::nullopt, 11_V);
    CHECK(cfg.GetExponentialProfileKV().value() == Catch::Approx(0.8).margin(1e-3));
    CHECK(cfg.GetExponentialProfileKA().value() == Catch::Approx(0.05).margin(1e-3));
    CHECK(cfg.GetExponentialProfileMaxInput()->value() == Catch::Approx(11.0).margin(1e-3));
    auto motionMagic = ReadConfig(hw.fx).MotionMagic;
    CHECK(motionMagic.MotionMagicExpo_kV.value() == Catch::Approx(0.8).margin(1e-3));
    CHECK(motionMagic.MotionMagicExpo_kA.value() == Catch::Approx(0.05).margin(1e-3));
  }
  {
    // Elevator profile: per-meter constraints, Motion Magic Expo gains per mechanism rotation.
    auto cfg = BaseConfig();
    cfg.WithFeedback(1.0, 0.0, 0.0).WithExponentialProfile(12_V, kKraken, 5_kg, 0.03_m);
    TalonFXWrapper smc{&hw.fx, kKraken, &cfg};
    const double kA = cfg.GetExponentialProfileKA().value();
    smc.SetExponentialProfile(1.0, std::nullopt, std::nullopt);
    CHECK(cfg.HasLinearExponentialProfile());
    CHECK(cfg.GetExponentialProfileKV().value() == Catch::Approx(1.0).margin(1e-3));
    CHECK(cfg.GetExponentialProfileKA().value() == Catch::Approx(kA).margin(1e-3));
    CHECK(ReadConfig(hw.fx).MotionMagic.MotionMagicExpo_kV.value() ==
          Catch::Approx(1.0).margin(1e-3));
  }
}

// ---- Soft limits and ramps ----------------------------------------------------------------

TEST_CASE_METHOD(TalonParityFixture, "TalonParity.SoftLimits", "[TalonParity]") {
  {
    // Soft limits are only enforced in closed loop mode.
    auto cfg = BaseConfig();
    cfg.WithOpenLoopMode().WithMechanismLimits(-1_tr, 1_tr);
    TalonFXWrapper smc{&hw.fx, kKraken, &cfg};
    auto limits = ReadConfig(hw.fx).SoftwareLimitSwitch;
    CHECK_FALSE(limits.ForwardSoftLimitEnable);
    CHECK_FALSE(limits.ReverseSoftLimitEnable);
    CHECK(limits.ForwardSoftLimitThreshold.value() == Catch::Approx(1.0).margin(1e-3));
  }
  {
    // SetMechanismLimits only moves the thresholds.
    auto cfg = BaseConfig();
    cfg.WithFeedback(1.0, 0.0, 0.0).WithMechanismLimits(-1_tr, 1_tr);
    TalonFXWrapper smc{&hw.fx, kKraken, &cfg};
    smc.SetMechanismLimitsEnabled(false);
    smc.SetMechanismLimits(-2_tr, 2_tr);
    auto limits = ReadConfig(hw.fx).SoftwareLimitSwitch;
    CHECK_FALSE(limits.ForwardSoftLimitEnable);
    CHECK_FALSE(limits.ReverseSoftLimitEnable);
    CHECK(limits.ForwardSoftLimitThreshold.value() == Catch::Approx(2.0).margin(1e-3));
    CHECK(limits.ReverseSoftLimitThreshold.value() == Catch::Approx(-2.0).margin(1e-3));
  }
  {
    // A measurement limit needs the other limit configured.
    auto cfg = BaseConfig();
    cfg.WithFeedback(1.0, 0.0, 0.0).WithMechanismCircumference(0.1_m);
    TalonFXWrapper smc{&hw.fx, kKraken, &cfg};
    smc.SetMeasurementUpperLimit(0.5_m);
    auto limits = ReadConfig(hw.fx).SoftwareLimitSwitch;
    CHECK_FALSE(limits.ForwardSoftLimitEnable);
    CHECK(limits.ForwardSoftLimitThreshold.value() == Catch::Approx(0.0).margin(1e-3));
  }
}

TEST_CASE_METHOD(TalonParityFixture, "TalonParity.RampsForAllOutputTypes", "[TalonParity]") {
  auto cfg = BaseConfig();
  cfg.WithFeedback(1.0, 0.0, 0.0).WithClosedLoopRampRate(0.3_s).WithOpenLoopRampRate(0.2_s);
  TalonFXWrapper smc{&hw.fx, kKraken, &cfg};
  auto device = ReadConfig(hw.fx);
  CHECK(device.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod.value() ==
        Catch::Approx(0.3).margin(1e-3));
  CHECK(device.ClosedLoopRamps.TorqueClosedLoopRampPeriod.value() ==
        Catch::Approx(0.3).margin(1e-3));
  CHECK(device.OpenLoopRamps.DutyCycleOpenLoopRampPeriod.value() ==
        Catch::Approx(0.2).margin(1e-3));
  CHECK(device.OpenLoopRamps.TorqueOpenLoopRampPeriod.value() == Catch::Approx(0.2).margin(1e-3));
  smc.SetClosedLoopRampRate(0.5_s);
  smc.SetOpenLoopRampRate(0.4_s);
  device = ReadConfig(hw.fx);
  CHECK(device.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod.value() ==
        Catch::Approx(0.5).margin(1e-3));
  CHECK(device.ClosedLoopRamps.VoltageClosedLoopRampPeriod.value() ==
        Catch::Approx(0.5).margin(1e-3));
  CHECK(device.ClosedLoopRamps.TorqueClosedLoopRampPeriod.value() ==
        Catch::Approx(0.5).margin(1e-3));
  CHECK(device.OpenLoopRamps.DutyCycleOpenLoopRampPeriod.value() ==
        Catch::Approx(0.4).margin(1e-3));
  CHECK(device.OpenLoopRamps.TorqueOpenLoopRampPeriod.value() == Catch::Approx(0.4).margin(1e-3));
  CHECK(cfg.GetClosedLoopRampRate()->value() == Catch::Approx(0.5).margin(1e-3));
}

// ---- Followers ----------------------------------------------------------------------------

TEST_CASE_METHOD(TalonParityFixture, "TalonParity.FollowersGetNeutralModeAndAreCleared",
                 "[TalonParity]") {
  {
    auto cfg = BaseConfig();
    cfg.WithFeedback(1.0, 0.0, 0.0).WithFollowers({{std::any{&hw.fxFollower}, true}});
    TalonFXWrapper smc{&hw.fx, kKraken, &cfg};
    CHECK(ReadConfig(hw.fxFollower).MotorOutput.NeutralMode == signals::NeutralModeValue::Brake);
    CHECK(AppliedControlName(hw.fxFollower) == "Follower");
    CHECK(cfg.GetFollowers().empty());
  }
  {
    auto cfg = BaseConfig();
    cfg.WithFeedback(1.0, 0.0, 0.0).WithFollowers({{std::any{42}, false}});
    CHECK_THROWS(TalonFXWrapper{&hw.fx, kKraken, &cfg});
  }
}

TEST_CASE_METHOD(TalonParityFixture, "TalonParity.LooseFollowersAndLiveSetters", "[TalonParity]") {
  SmartMotorControllerConfig followerCfg;
  FakeSmartMotorController looseFollower{&followerCfg};
  auto cfg = BaseConfig();
  cfg.WithFeedback(2.0, 0.0, 0.0)
      .WithFeedforward(MakeSimpleFF(0.0, 0.1, 0.0))
      .WithMechanismCircumference(0.1_m)
      .WithLooselyCoupledFollowers({&looseFollower});
  TalonFXSWrapper smc{&hw.fxs, kNeo, TalonFXSWrapper::MotorArrangement::NEO, &cfg};

  // Stopping the leader stops loosely coupled followers too.
  looseFollower.dutyCycle = 0.7;
  smc.SetDutyCycle(0.0);
  CHECK(looseFollower.dutyCycle == 0.0);

  // Live gains are kept in the config.
  smc.SetKp(4.8);
  CHECK(cfg.GetKp() == Catch::Approx(4.8).margin(1e-3));
  CHECK(ReadConfig(hw.fxs).Slot0.kP.value() == Catch::Approx(4.8).margin(1e-3));

  // The feedforward force is sent to the Talon and forwarded; a plain velocity clears it.
  smc.SetVelocity(1_tps, 10_N);
  CHECK(std::abs(std::stod(AppliedControlField(hw.fxs, "FeedForward"))) > 1e-6);
  CHECK(looseFollower.GetSetpointFeedforwardForce().value_or(0_N).value() ==
        Catch::Approx(10.0).margin(1e-3));
  smc.SetVelocity(1_tps);
  CHECK(std::stod(AppliedControlField(hw.fxs, "FeedForward")) == Catch::Approx(0.0).margin(1e-3));

  smc.SetClosedLoopSlot(Slot::SLOT_1);
  CHECK(looseFollower.GetClosedLoopControllerSlot() == Slot::SLOT_1);
}

// ---- Unsupported options ------------------------------------------------------------------

TEST_CASE_METHOD(TalonParityFixture, "TalonParity.UnsupportedOptionsThrow", "[TalonParity]") {
  auto expectThrow = [&](const std::string& what, auto&& configure) {
    auto cfg = BaseConfig();
    cfg.WithFeedback(1.0, 0.0, 0.0);
    configure(cfg);
    INFO(what);
    CHECK_THROWS(TalonFXWrapper{&hw.fx, kKraken, &cfg});
  };
  expectThrow("closed loop tolerance",
              [](auto& cfg) { cfg.WithClosedLoopTolerance(wpi::units::turn_t{0.01}); });
  expectThrow("closed loop control period",
              [](auto& cfg) { cfg.WithClosedLoopControlPeriod(5_ms); });
  expectThrow("temperature cutoff",
              [](auto& cfg) { cfg.WithTemperatureCutoff(wpi::units::celsius_t{70}); });
  expectThrow("feedback synchronization threshold",
              [](auto& cfg) { cfg.WithFeedbackSynchronizationThreshold(0.01_tr); });
  expectThrow("voltage compensation", [](auto& cfg) { cfg.WithVoltageCompensation(12_V); });
  expectThrow("encoder inverted", [](auto& cfg) { cfg.WithEncoderInverted(true); });
  expectThrow("slot 3 feedforward",
              [](auto& cfg) { cfg.WithFeedforward(MakeSimpleFF(0.1, 0.1, 0.0), Slot::SLOT_3); });
  expectThrow("unsupported control request",
              [](auto& cfg) { cfg.WithVendorControlRequest(controls::DutyCycleOut{0.0}); });

  {
    // The TalonFXS has three slots too; Java put slot 3 feedforwards in slot 2.
    auto cfg = BaseConfig();
    cfg.WithFeedback(1.0, 0.0, 0.0).WithFeedforward(MakeSimpleFF(0.1, 0.1, 0.0), Slot::SLOT_3);
    CHECK_THROWS(TalonFXSWrapper{&hw.fxs, kNeo, TalonFXSWrapper::MotorArrangement::NEO, &cfg});
  }
  {
    auto cfg = BaseConfig();
    cfg.WithFeedback(1.0, 0.0, 0.0);
    TalonFXWrapper smc{&hw.fx, kKraken, &cfg};
    CHECK_THROWS(smc.SetClosedLoopSlot(Slot::SLOT_3));
  }
}

TEST_CASE_METHOD(TalonParityFixture, "TalonParity.LinearUnitsNeedCircumference", "[TalonParity]") {
  auto cfg = BaseConfig();
  cfg.WithFeedback(1.0, 0.0, 0.0);
  TalonFXWrapper smc{&hw.fx, kKraken, &cfg};
  CHECK_THROWS(smc.GetMeasurementPosition());
  CHECK_THROWS(smc.GetMeasurementVelocity());
  CHECK_THROWS(smc.SetEncoderPosition(1_m));
  CHECK_THROWS(smc.SetPosition(1_m));
}

// ---- Setpoints ----------------------------------------------------------------------------

TEST_CASE_METHOD(TalonParityFixture, "TalonParity.TalonFXSSetpointsInOpenLoopMode",
                 "[TalonParity]") {
  auto cfg = BaseConfig();
  cfg.WithOpenLoopMode().WithFeedback(1.0, 0.0, 0.0);
  TalonFXSWrapper smc{&hw.fxs, kNeo, TalonFXSWrapper::MotorArrangement::NEO, &cfg};
  smc.SetPosition(0.5_tr);
  CHECK(AppliedControlName(hw.fxs) == "PositionVoltage");
}

// ---- External encoders --------------------------------------------------------------------

TEST_CASE_METHOD(TalonParityFixture, "TalonParity.CANcoderExternalEncoder", "[TalonParity]") {
  auto cfg = BaseConfig();
  cfg.WithFeedback(1.0, 0.0, 0.0)
      .WithExternalEncoder(std::any{&hw.cancoder})
      .WithExternalEncoderGearing(2.0)
      .WithExternalEncoderZeroOffset(0.1_tr)
      .WithExternalEncoderDiscontinuityPoint(1_tr)
      .WithStartingPosition(90_deg);
  TalonFXWrapper smc{&hw.fx, kKraken, &cfg};
  // The external encoder gives the position, so the starting position is not applied.
  CHECK(AlertActive("starting position is not applied"));

  configs::CANcoderConfiguration cancoderConfig;
  hw.cancoder.GetConfigurator().Refresh(cancoderConfig);
  CHECK(cancoderConfig.MagnetSensor.MagnetOffset.value() == Catch::Approx(0.1).margin(1e-3));

  // The simulated CANcoder turns twice per mechanism rotation and reads the zero offset.
  smc.GetSimSupplier()->SetMechanismPosition(0.3_tr);
  smc.SimIterate();
  CHECK(WaitFor(
      [&] { return std::abs(hw.cancoder.GetAbsolutePosition().GetValue().value() - 0.6) < 0.02; }));

  // Setting the encoder position sets the CANcoder, and its position (not its absolute
  // position) is reported.
  smc.SetEncoderPosition(1.3_tr);
  CHECK(WaitFor([&] {
    auto position = smc.GetExternalEncoderPosition();
    return position && std::abs(wpi::units::turn_t{*position}.value() - 2.6) < 0.02;
  }));
}

TEST_CASE_METHOD(TalonParityFixture, "TalonParity.CANdiNeedsPWMSource", "[TalonParity]") {
  auto cfg = BaseConfig();
  cfg.WithFeedback(1.0, 0.0, 0.0).WithExternalEncoder(std::any{&hw.candi});
  CHECK_THROWS(TalonFXWrapper{&hw.fx, kKraken, &cfg});
}

TEST_CASE_METHOD(TalonParityFixture, "TalonParity.ExternalEncoderOptionsWithoutEncoderAlert",
                 "[TalonParity]") {
  auto cfg = BaseConfig();
  cfg.WithFeedback(1.0, 0.0, 0.0)
      .WithExternalEncoderZeroOffset(0.1_tr)
      .WithExternalEncoderDiscontinuityPoint(1_tr);
  {
    TalonFXWrapper smc{&hw.fx, kKraken, &cfg};
    CHECK(AlertActive("zero offset is not supported without an external encoder"));
    CHECK(AlertActive("discontinuity point is not supported without an external encoder"));
  }
  CHECK_FALSE(AlertActive("zero offset is not supported without an external encoder"));
}

// ---- Simulation and gearing ----------------------------------------------------------------

TEST_CASE_METHOD(TalonParityFixture, "TalonParity.Simulation", "[TalonParity]") {
  {
    // A simulation motor chosen by the user is kept.
    auto cfg = BaseConfig();
    cfg.WithFeedback(1.0, 0.0, 0.0).WithSimMotor(wpi::math::DCMotor::Falcon500(1));
    TalonFXWrapper smc{&hw.fx, kKraken, &cfg};
    CHECK(cfg.GetSimMotor()->freeSpeed.value() ==
          Catch::Approx(wpi::math::DCMotor::Falcon500(1).freeSpeed.value()));
  }
  {
    // The simulation is built without explicit gearing (1:1).
    SmartMotorControllerConfig cfg;
    cfg.WithFeedback(1.0, 0.0, 0.0).WithClosedLoopMode();
    TalonFXSWrapper smc{&hw.fxs, kNeo, TalonFXSWrapper::MotorArrangement::NEO, &cfg};
    CHECK(smc.GetSimSupplier() != nullptr);
  }
  {
    // Status signals follow a non-default simulation period.
    auto cfg = BaseConfig();
    cfg.WithFeedback(1.0, 0.0, 0.0).WithSimulationPeriod(5_ms);
    TalonFXWrapper smc{&hw.fx, kKraken, &cfg};
    CHECK(hw.fx.GetPosition(false).GetAppliedUpdateFrequency().value() ==
          Catch::Approx(200.0).margin(1e-3));
  }
}

TEST_CASE_METHOD(TalonParityFixture, "TalonParity.SetMechanismGearingUpdatesSensorRatio",
                 "[TalonParity]") {
  auto cfg = BaseConfig();
  cfg.WithFeedback(1.0, 0.0, 0.0);
  TalonFXWrapper smc{&hw.fx, kKraken, &cfg};
  smc.SetMechanismGearing(gearing::MechanismGearing{20.0});
  CHECK(ReadConfig(hw.fx).Feedback.SensorToMechanismRatio.value() ==
        Catch::Approx(20.0).margin(1e-3));
}

}  // namespace yams::test
