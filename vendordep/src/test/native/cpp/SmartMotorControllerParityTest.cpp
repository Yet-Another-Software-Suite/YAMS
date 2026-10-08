// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Tests for SmartMotorController / SmartMotorControllerConfig / SimSupplier behaviour and API
// ported from the Java reference that need API added alongside the fixes.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>
#include <string>
#include <utility>
#include <wpi/math/controller/SimpleMotorFeedforward.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/math/system/Models.hpp>
#include <wpi/nt/NetworkTableInstance.hpp>
#include <wpi/simulation/DCMotorSim.hpp>
#include <wpi/simulation/RoboRioSim.hpp>
#include <wpi/simulation/SimHooks.hpp>
#include <wpi/simulation/SingleJointedArmSim.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/force.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/mass.hpp>
#include <wpi/units/moment_of_inertia.hpp>

#include "helpers/FakeSmartMotorController.h"
#include "helpers/MockHardware.h"
#include "yams/exceptions.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/math/LQRConfig.hpp"
#include "yams/motorcontrollers/simulation/ArmSimSupplier.hpp"
#include "yams/motorcontrollers/simulation/BatterySim.hpp"
#include "yams/motorcontrollers/simulation/DCMotorSimSupplier.hpp"
#include "yams/telemetry/SmartMotorControllerTelemetryConfig.hpp"

namespace yams::test {

using motorcontrollers::meters_per_second_cubed_t;
using motorcontrollers::simulation::ArmSimSupplier;
using motorcontrollers::simulation::BatterySim;
using motorcontrollers::simulation::DCMotorSimSupplier;
using Slot = SmartMotorControllerConfig::ClosedLoopControllerSlot;
using Verbosity = SmartMotorControllerConfig::TelemetryVerbosity;
using SimpleFF = wpi::math::SimpleMotorFeedforward<wpi::units::turns>;

namespace {

std::string UniqueName(const std::string& prefix) {
  static int count = 0;
  return prefix + std::to_string(count++);
}

std::shared_ptr<wpi::nt::NetworkTable> DataRoot() {
  return wpi::nt::NetworkTableInstance::GetDefault().GetTable("SMCParity/Data");
}

std::shared_ptr<wpi::nt::NetworkTable> TuningRoot() {
  return wpi::nt::NetworkTableInstance::GetDefault().GetTable("SMCParity/Tuning");
}

/** Fake controller exposing the protected software PID setup. */
class PIDFake : public FakeSmartMotorController {
 public:
  using FakeSmartMotorController::FakeSmartMotorController;
  void ConfigurePID() { ConfigureSoftwarePID(GetConfig()); }
  const std::optional<wpi::math::PIDController>& PID() const { return m_pid; }
};

wpi::sim::DCMotorSim MakeDCMotorSim() {
  auto motor = wpi::math::DCMotor::KrakenX60(1);
  return wpi::sim::DCMotorSim{
      wpi::math::Models::SingleJointedArmFromPhysicalConstants(motor, 0.01_kg_sq_m, 1.0), motor};
}

}  // namespace

// ---- Item 2: the simulation is stepped once per loop ----------------------------------------

TEST_CASE("SMCParity.SimSteppedOncePerLoop", "[SMCParity]") {
  InitializeHardware();
  SmartMotorControllerConfig cfg;
  cfg.WithMotorGearing(1.0);
  FakeSmartMotorController smc{&cfg};
  auto sim = MakeDCMotorSim();
  auto reference = MakeDCMotorSim();
  DCMotorSimSupplier supplier{sim, smc};

  for (int i = 0; i < 5; ++i) {
    // The motor controller feeds its output voltage; the mechanism and the motor controller both
    // update the simulation in the same loop, then the loop ends.
    supplier.SetMechanismStatorVoltage(6_V);
    supplier.UpdateSim();
    supplier.UpdateSim();
    supplier.StarveUpdateSim();

    reference.SetInputVoltage(6_V);
    reference.Update(20_ms);
  }
  CHECK(supplier.GetMechanismVelocity().value() ==
        Catch::Approx(wpi::units::turns_per_second_t{reference.GetAngularVelocity()}.value())
            .epsilon(1e-9));
  smc.Close();
  TeardownHardware();
}

// ---- Item 20: the simulation period --------------------------------------------------------

TEST_CASE("SMCParity.SupplierUsesSimulationPeriod", "[SMCParity]") {
  InitializeHardware();
  SmartMotorControllerConfig cfg;
  CHECK(cfg.GetSimulationPeriod().value() == Catch::Approx(0.02));
  cfg.WithMotorGearing(1.0).WithClosedLoopControlPeriod(10_ms).WithSimulationPeriod(5_ms);
  FakeSmartMotorController smc{&cfg};
  auto sim = MakeDCMotorSim();
  auto reference = MakeDCMotorSim();
  DCMotorSimSupplier supplier{sim, smc};
  supplier.SetMechanismStatorVoltage(12_V);
  supplier.UpdateSim();
  reference.SetInputVoltage(12_V);
  reference.Update(5_ms);
  CHECK(supplier.GetMechanismVelocity().value() ==
        Catch::Approx(wpi::units::turns_per_second_t{reference.GetAngularVelocity()}.value())
            .epsilon(1e-9));
  smc.Close();
  TeardownHardware();
}

// ---- Items 4 and 18: battery simulation ------------------------------------------------------

TEST_CASE("SMCParity.SupplierTracksFilteredSupplyCurrentUnderController", "[SMCParity]") {
  InitializeHardware();
  BatterySim::Reset();
  SmartMotorControllerConfig cfg;
  cfg.WithMotorGearing(1.0);
  FakeSmartMotorController smc{&cfg};
  auto sim = MakeDCMotorSim();
  DCMotorSimSupplier supplier{sim, smc};

  smc.dutyCycle = 0.5;
  // Fed input: the battery is not updated, so the supply current filter is still fresh.
  supplier.SetMechanismStatorVoltage(6_V);
  supplier.UpdateSim();
  supplier.StarveUpdateSim();
  const void* key = static_cast<motorcontrollers::SmartMotorController*>(&smc);
  CHECK_FALSE(BatterySim::HasCurrent(key));

  // Supply current is the duty cycle times the stator current, through a 0.1 s IIR filter.
  double stator = supplier.GetStatorCurrent().value();
  REQUIRE(stator > 1.0);
  double gain = 1.0 - std::exp(-0.02 / 0.1);
  CHECK(supplier.GetSupplyCurrent().value() == Catch::Approx(gain * 0.5 * stator).epsilon(1e-6));

  // Input not fed: the duty cycle is applied and the battery tracks the current under the SMC.
  supplier.UpdateSim();
  CHECK(BatterySim::HasCurrent(key));

  smc.Close();
  CHECK_FALSE(BatterySim::HasCurrent(key));
  BatterySim::Reset();
  TeardownHardware();
}

TEST_CASE("SMCParity.BatterySimReset", "[SMCParity]") {
  int a = 0;
  BatterySim::CalculateVoltage(&a, 10_A);
  REQUIRE(BatterySim::HasCurrent(&a));
  BatterySim::Reset();
  CHECK_FALSE(BatterySim::HasCurrent(&a));
}

// ---- Item 19: arm supplier acceleration ----------------------------------------------------

TEST_CASE("SMCParity.ArmSupplierReportsAcceleration", "[SMCParity]") {
  InitializeHardware();
  SmartMotorControllerConfig cfg;
  cfg.WithMotorGearing(10.0);
  FakeSmartMotorController smc{&cfg};
  auto motor = wpi::math::DCMotor::KrakenX60(1);
  wpi::sim::SingleJointedArmSim sim{motor,  10.0,  0.1_kg_sq_m, 0.5_m, -10_rad,
                                    10_rad, false, 0_rad};
  ArmSimSupplier supplier{sim, smc};

  wpi::units::turns_per_second_squared_t rotorAccel{0};
  for (int i = 0; i < 5; ++i) {
    supplier.SetMechanismStatorVoltage(12_V);
    supplier.UpdateSim();
    supplier.StarveUpdateSim();
    wpi::sim::StepTiming(20_ms);
    rotorAccel = supplier.GetRotorAcceleration();
  }
  CHECK(rotorAccel.value() > 0.0);
  CHECK(supplier.GetMechanismAcceleration().value() ==
        Catch::Approx(supplier.GetRotorAcceleration().value() / 10.0));
  smc.Close();
  TeardownHardware();
}

// ---- Items 7 and missing API: software PID configuration -----------------------------------

TEST_CASE("SMCParity.SoftwarePIDWrapsAndHasTolerance", "[SMCParity]") {
  SmartMotorControllerConfig cfg;
  cfg.WithFeedback(1.0, 0.0, 0.0)
      .WithContinuousWrapping(-0.5_tr, 0.5_tr)
      .WithClosedLoopTolerance(0.01_tr);
  PIDFake smc{&cfg};
  smc.CreatePID();
  smc.ConfigurePID();
  REQUIRE(smc.PID().has_value());
  CHECK(smc.PID()->IsContinuousInputEnabled());
  CHECK(smc.PID()->GetErrorTolerance() == Catch::Approx(0.01));
}

TEST_CASE("SMCParity.ContinuousWrappingSetpoint", "[SMCParity]") {
  SmartMotorControllerConfig cfg;
  CHECK(cfg.GetContinuousWrappingSetpoint(0.4_tr, -0.4_tr).value() == Catch::Approx(0.4));
  cfg.WithFeedback(1.0, 0.0, 0.0).WithContinuousWrapping(-0.5_tr, 0.5_tr);
  CHECK(cfg.GetContinuousWrappingSetpoint(0.4_tr, -0.4_tr).value() == Catch::Approx(-0.6));
  CHECK(cfg.GetContinuousWrappingSetpoint(0.1_tr, 3.0_tr).value() == Catch::Approx(3.1));
}

// ---- Item 12: zero power is optional -------------------------------------------------------

TEST_CASE("SMCParity.ZeroPowerIsOptional", "[SMCParity]") {
  SmartMotorControllerConfig cfg;
  CHECK_FALSE(cfg.GetZeroPower().has_value());
  cfg.WithZeroPower(SmartMotorControllerConfig::MotorMode::BRAKE);
  CHECK(cfg.GetZeroPower() == SmartMotorControllerConfig::MotorMode::BRAKE);
}

// ---- Item 17: velocity trapezoidal profiles --------------------------------------------------

TEST_CASE("SMCParity.VelocityTrapezoidProfileTakesAccelerationAndJerk", "[SMCParity]") {
  SECTION("angular") {
    SmartMotorControllerConfig cfg;
    cfg.WithVelocityTrapezoidProfile(2_tr_per_s_sq,
                                     wpi::units::angular_jerk::turns_per_second_cubed_t{10.0})
        .WithFeedforward(SimpleFF{0_V, wpi::units::unit_t<SimpleFF::kv_unit>{1.0},
                                  wpi::units::unit_t<SimpleFF::ka_unit>{0.0}});
    CHECK(cfg.GetVelocityTrapezoidalProfileInUse());
    CHECK(cfg.GetTrapMaxVelocityTurns()->value() == Catch::Approx(2.0));
    CHECK(cfg.GetTrapMaxAccelTurns()->value() == Catch::Approx(10.0));
    FakeSmartMotorController smc{&cfg};
    smc.SetVelocity(10_tps);
    smc.SetRunning(true);
    smc.IterateClosedLoopController();
    // The velocity setpoint ramps (jerk limited), so kV * setpoint is far below 10 V.
    CHECK(smc.lastVoltage.value() > 0.0);
    CHECK(smc.lastVoltage.value() < 0.1);
  }
  SECTION("linear") {
    SmartMotorControllerConfig cfg;
    cfg.WithMechanismCircumference(1_m)
        .WithFeedback(1.0, 0.0, 0.0)
        .WithVelocityTrapezoidProfile(2_mps_sq, meters_per_second_cubed_t{10.0});
    CHECK(cfg.GetLinearClosedLoopControllerUse());
    FakeSmartMotorController smc{&cfg};
    smc.CreatePID();
    smc.SetVelocity(1_mps);
    smc.SetRunning(true);
    smc.IterateClosedLoopController();
    // The PID chases the profiled velocity (m/s), not the 1 m/s goal.
    CHECK(smc.lastVoltage.value() > 0.0);
    CHECK(smc.lastVoltage.value() < 0.1);
  }
}

TEST_CASE("SMCParity.VelocityProfileJerkTuningDefault", "[SMCParity]") {
  auto name = UniqueName("JerkDefault");
  SmartMotorControllerConfig cfg;
  cfg.WithFeedback(1.0, 0.0, 0.0)
      .WithVelocityTrapezoidProfile(2_tr_per_s_sq,
                                    wpi::units::angular_jerk::turns_per_second_cubed_t{10.0})
      .WithTelemetry(name, Verbosity::HIGH);
  FakeSmartMotorController smc{&cfg};
  smc.SetupTelemetry(DataRoot(), TuningRoot());
  // 10 rot/s^3 is 600 RPM/s^2.
  CHECK(TuningRoot()->GetSubTable(name)->GetEntry("closedloop/motionprofile/maxJerk").GetDouble(
            -1.0) == Catch::Approx(600.0));
  smc.Close();
}

// ---- Missing API: feedforward force ----------------------------------------------------------

TEST_CASE("SMCParity.VelocityFeedforwardForceAddedInLoop", "[SMCParity]") {
  SmartMotorControllerConfig cfg;
  cfg.WithMechanismCircumference(0.1_m).WithMotorGearing(1.0);
  FakeSmartMotorController smc{&cfg};
  smc.SetVelocity(0_tps, 10_N);
  REQUIRE(smc.GetSetpointFeedforwardForce().has_value());
  CHECK(smc.GetSetpointFeedforwardForce()->value() == Catch::Approx(10.0));
  smc.SetRunning(true);
  smc.IterateClosedLoopController();

  auto motor = wpi::math::DCMotor::KrakenX60(1);
  double torque = 10.0 * (0.1 / (2.0 * std::numbers::pi));
  double expected = motor.Voltage(wpi::units::newton_meter_t{torque}, 0_rad_per_s).value();
  CHECK(expected > 0.0);
  CHECK(smc.lastVoltage.value() == Catch::Approx(expected));
  CHECK(cfg.ConvertToVoltage(motor, 10_N).value() == Catch::Approx(expected));
  CHECK(cfg.ConvertToCurrent(motor, 10_N).value() ==
        Catch::Approx(motor.Current(wpi::units::newton_meter_t{torque}).value()));

  SmartMotorControllerConfig noCircumference;
  CHECK_THROWS_AS(noCircumference.ConvertToVoltage(motor, 10_N),
                  exceptions::SmartMotorControllerConfigurationException);
}

TEST_CASE("SMCParity.SetpointForceTelemetry", "[SMCParity]") {
  auto name = UniqueName("ForceTelemetry");
  SmartMotorControllerConfig cfg;
  cfg.WithMechanismCircumference(0.1_m).WithTelemetry(name, Verbosity::HIGH);
  FakeSmartMotorController smc{&cfg};
  smc.SetupTelemetry(DataRoot(), TuningRoot());
  smc.SetVelocity(1_tps, 3_N);
  smc.UpdateTelemetry();
  CHECK(DataRoot()->GetSubTable(name)->GetEntry("closedloop/setpoint/force").GetDouble(-1.0) ==
        Catch::Approx(3.0));
  smc.Close();
}

// ---- Missing API: conversions ------------------------------------------------------------------

TEST_CASE("SMCParity.DistanceConversions", "[SMCParity]") {
  SmartMotorControllerConfig cfg;
  CHECK_THROWS_AS(cfg.ConvertToMechanism(1_m),
                  exceptions::SmartMotorControllerConfigurationException);
  cfg.WithMechanismCircumference(0.5_m);
  CHECK(cfg.ConvertToMechanism(1_m).value() == Catch::Approx(2.0));
  CHECK(cfg.ConvertToMechanism(1_mps).value() == Catch::Approx(2.0));
  CHECK(cfg.ConvertToMechanism(1_mps_sq).value() == Catch::Approx(2.0));
  CHECK(cfg.ConvertToMechanism(meters_per_second_cubed_t{1.0}).value() == Catch::Approx(2.0));
  CHECK(cfg.ConvertFromMechanism(2_tr_per_s_sq).value() == Catch::Approx(1.0));
  CHECK(cfg.ConvertFromMechanism(wpi::units::angular_jerk::turns_per_second_cubed_t{2.0}).value() ==
        Catch::Approx(1.0));
}

// ---- Missing API: config builders ----------------------------------------------------------

TEST_CASE("SMCParity.ConfigBuilders", "[SMCParity]") {
  SECTION("feedback synchronization threshold is rotational only") {
    SmartMotorControllerConfig cfg;
    cfg.WithFeedbackSynchronizationThreshold(0.01_tr);
    CHECK(cfg.GetFeedbackSynchronizationThreshold()->value() == Catch::Approx(0.01));
    SmartMotorControllerConfig linear;
    linear.WithMechanismCircumference(1_m);
    CHECK_THROWS_AS(linear.WithFeedbackSynchronizationThreshold(0.01_tr),
                    exceptions::SmartMotorControllerConfigurationException);
  }
  SECTION("closed-loop tolerance needs a PID") {
    SmartMotorControllerConfig cfg;
    CHECK_THROWS_AS(cfg.WithClosedLoopTolerance(0.01_tr),
                    exceptions::SmartMotorControllerConfigurationException);
    SmartMotorControllerConfig rotational;
    rotational.WithFeedback(1.0, 0.0, 0.0).WithMechanismCircumference(1_m);
    CHECK_THROWS_AS(rotational.WithClosedLoopTolerance(0.01_m),
                    exceptions::SmartMotorControllerConfigurationException);
    SmartMotorControllerConfig linear;
    linear.WithFeedback(1.0, 0.0, 0.0)
        .WithMechanismCircumference(2_m)
        .WithLinearClosedLoopController(true)
        .WithClosedLoopTolerance(0.02_m);
    CHECK(linear.GetClosedLoopTolerance()->value() == Catch::Approx(0.01));
  }
  SECTION("voltage compensation and reset previous config") {
    SmartMotorControllerConfig cfg;
    CHECK_FALSE(cfg.GetVoltageCompensation().has_value());
    CHECK(cfg.GetResetPreviousConfig());
    cfg.WithVoltageCompensation(11_V).WithResetPreviousConfig(false);
    CHECK(cfg.GetVoltageCompensation()->value() == Catch::Approx(11.0));
    CHECK_FALSE(cfg.GetResetPreviousConfig());
  }
  SECTION("telemetry with only a verbosity is named motor") {
    SmartMotorControllerConfig cfg;
    cfg.WithTelemetry(Verbosity::LOW);
    CHECK(cfg.GetTelemetryName() == "motor");
    CHECK(cfg.GetVerbosity() == Verbosity::LOW);
  }
  SECTION("linear closed-loop controller flag") {
    SmartMotorControllerConfig cfg;
    cfg.WithMechanismCircumference(1_m).WithLinearClosedLoopController(true);
    CHECK(cfg.GetLinearClosedLoopControllerUse());
    cfg.WithLinearClosedLoopController(false);
    CHECK_FALSE(cfg.GetLinearClosedLoopControllerUse());
  }
  SECTION("zero offset as a distance") {
    SmartMotorControllerConfig cfg;
    CHECK_THROWS_AS(cfg.WithExternalEncoderZeroOffset(0.1_m),
                    exceptions::SmartMotorControllerConfigurationException);
    cfg.WithMechanismCircumference(1_m).WithExternalEncoderZeroOffset(0.25_m);
    CHECK(cfg.GetExternalEncoderZeroOffset()->value() == Catch::Approx(0.25));
  }
  SECTION("gearing from a ratio and cascading stages") {
    SmartMotorControllerConfig cfg;
    CHECK_THROWS_AS(cfg.WithCascadingElevatorStages(2),
                    exceptions::SmartMotorControllerConfigurationException);
    cfg.WithMotorGearing(10.0);
    CHECK(cfg.GetMotorGearing()->GetMechanismToRotorRatio() == Catch::Approx(10.0));
    cfg.WithCascadingElevatorStages(2);
    // Same as Java: the gearing is divided by the stages via MechanismGearing::Div.
    gearing::MechanismGearing expected{10.0};
    expected.Div(2.0);
    CHECK(cfg.GetMotorGearing()->GetMechanismToRotorRatio() ==
          Catch::Approx(expected.GetMechanismToRotorRatio()));
  }
  SECTION("simulation LQR override") {
    SmartMotorControllerConfig cfg;
    cfg.WithFeedback(1.0, 0.0, 0.0);
    CHECK_FALSE(cfg.GetLQR(Slot::SLOT_0).has_value());
    cfg.WithSimClosedLoopController(math::LQRConfig{}
                                        .WithFlywheelSystem(wpi::math::DCMotor::NEO(1), 0.00233, 1.0)
                                        .WithQElems({3.0})
                                        .WithRElems({12.0})
                                        .WithStateStdDevs({3.0})
                                        .WithMeasurementStdDevs({0.01}));
    CHECK(cfg.GetLQR(Slot::SLOT_0).has_value());
    CHECK(cfg.GetSlotGains(Slot::SLOT_0).kP == Catch::Approx(0.0));
  }
  SECTION("clear followers") {
    SmartMotorControllerConfig cfg;
    int follower = 0;
    cfg.WithFollowers({{std::any{&follower}, false}});
    REQUIRE(cfg.GetFollowers().size() == 1);
    cfg.ClearFollowers();
    CHECK(cfg.GetFollowers().empty());
  }
  SECTION("new options are tracked for validation") {
    SmartMotorControllerConfig cfg;
    cfg.ResetValidationCheck();
    CHECK_THROWS_AS(cfg.ValidateBasicOptions(),
                    exceptions::SmartMotorControllerConfigurationException);
  }
}

// ---- Missing API: telemetry config, tuning and close hooks ---------------------------------

TEST_CASE("SMCParity.TelemetryConfigFromConfig", "[SMCParity]") {
  auto name = UniqueName("CustomTelemetry");
  SmartMotorControllerConfig cfg;
  telemetry::SmartMotorControllerTelemetryConfig telemetryConfig;
  telemetryConfig.WithMechanismPosition().WithCustom(telemetry::DoubleTelemetryField::RotorVelocity,
                                                     true);
  cfg.WithTelemetry(name, std::move(telemetryConfig));
  CHECK(cfg.GetVerbosity() == Verbosity::HIGH);
  REQUIRE(cfg.GetSmartControllerTelemetryConfig() != nullptr);
  FakeSmartMotorController smc{&cfg};
  smc.pos = 0.5_tr;
  smc.vel = 2_tps;
  smc.SetupTelemetry(DataRoot(), TuningRoot());
  auto data = DataRoot()->GetSubTable(name);
  CHECK(data->GetEntry("mechanism/position").GetDouble(-1.0) == Catch::Approx(0.5));
  CHECK(data->GetEntry("rotor/velocity").GetDouble(-1.0) == Catch::Approx(2.0));
  CHECK_FALSE(data->ContainsKey("motor/outputVoltage"));
  smc.Close();
}

TEST_CASE("SMCParity.ApplyTuningValues", "[SMCParity]") {
  auto name = UniqueName("ApplyTuning");
  SmartMotorControllerConfig cfg;
  cfg.WithFeedback(1.0, 0.0, 0.0).WithTelemetry(name, Verbosity::HIGH);
  FakeSmartMotorController smc{&cfg};
  smc.SetupTelemetry(DataRoot(), TuningRoot());
  CHECK(smc.TuningEnabled());
  TuningRoot()->GetSubTable(name)->GetEntry("closedloop/feedback/kP").SetDouble(5.0);
  smc.ApplyTuningValues();
  CHECK(smc.setKpCalls == 1);
  smc.Close();
}

TEST_CASE("SMCParity.CloseHooksRunOnce", "[SMCParity]") {
  SmartMotorControllerConfig cfg;
  int calls = 0;
  {
    FakeSmartMotorController smc{&cfg};
    smc.AddCloseHook([&calls] { ++calls; });
    smc.Close();
    smc.Close();
    CHECK(calls == 1);
  }
  CHECK(calls == 1);  // the destructor's Close() does not rerun it
}

TEST_CASE("SMCParity.SetMechanismGearingAndCircumference", "[SMCParity]") {
  SmartMotorControllerConfig cfg;
  FakeSmartMotorController smc{&cfg};
  smc.SetMechanismGearing(gearing::MechanismGearing{4.0});
  smc.SetMechanismCircumference(0.3_m);
  CHECK(cfg.GetMotorGearing()->GetMechanismToRotorRatio() == Catch::Approx(4.0));
  CHECK(cfg.GetMechanismCircumference()->value() == Catch::Approx(0.3));
}

}  // namespace yams::test
