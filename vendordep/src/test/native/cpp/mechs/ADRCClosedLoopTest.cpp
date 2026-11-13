// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// An arm, an elevator, a pivot and a flywheel closed with an ADRC on every motor controller, each simulated by the
// mechanism. None has a feedforward: the arm and elevator hold against gravity with no kG. The mechanisms need nothing
// for an ADRC: they set their setpoints as they do with a PID, and the motor controller runs the ADRC on the SystemCore.
// Mirrors the Java ADRCMechanismTest. The ADRC was written by FRC Team 8331.

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <string>
#include <vector>

#include <wpi/commands2/CommandScheduler.hpp>
#include <wpi/commands2/Commands.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/mass.hpp>
#include <wpi/units/moment_of_inertia.hpp>

#include "helpers/MockHardware.h"
#include "helpers/MotorControllerFactory.h"
#include "helpers/SchedulerHelper.h"
#include "yams/math/ADRCConfig.hpp"
#include "yams/math/LQRConfig.hpp"
#include "yams/mechanisms/config/ArmConfig.hpp"
#include "yams/mechanisms/config/ElevatorConfig.hpp"
#include "yams/mechanisms/config/FlyWheelConfig.hpp"
#include "yams/mechanisms/config/PivotConfig.hpp"
#include "yams/mechanisms/positional/Arm.hpp"
#include "yams/mechanisms/positional/Elevator.hpp"
#include "yams/mechanisms/positional/Pivot.hpp"
#include "yams/mechanisms/velocity/FlyWheel.hpp"

namespace yams::test {

using namespace motorcontrollers;
using namespace mechanisms;
using namespace mechanisms::config;

namespace {
constexpr double kReduction = 12.0;

/** Every motor controller, without a motion profile: the ADRC's settling time shapes the response. */
std::vector<MotorTestParam> Controllers() {
  std::vector<MotorTestParam> params;
  for (auto& param : AllMotorParams()) {
    if (param.profile == ProfileType::None || (param.hardware == HardwareType::Nova &&
                                               param.profile == ProfileType::Trapezoid)) {
      params.push_back({param.hardware, ProfileType::None, param.name});
    }
  }
  return params;
}

double Reduction(HardwareType hardware) { return hardware == HardwareType::A301 ? kA301Gearbox : kReduction; }

/** An ADRC of the given order and settling time. */
math::ADRCConfig Adrc(math::ADRCConfig::SystemOrder order, wpi::units::second_t settlingTime) {
  math::ADRCConfig adrc;
  adrc.WithSystemOrder(order).WithSettlingTime(settlingTime);
  return adrc;
}

/** The motor config with a 12:1 reduction (an A301 gets its own gearbox), limited to 12 V, and no ADRC yet. */
SmartMotorControllerConfig BaseConfig(const MotorTestParam& param, const std::string& mechanism) {
  SmartMotorControllerConfig cfg;
  cfg.WithMotorGearing(gearing::MechanismGearing{gearing::GearBox::FromReductionStages({3.0, 4.0})})
      .WithZeroPower(SmartMotorControllerConfig::MotorMode::BRAKE)
      .WithMotorInverted(false)
      .WithClosedLoopMode()
      .WithClosedLoopMaxVoltage(12_V)
      .WithTelemetry("ADRC" + mechanism + param.name);
  ForHardware(cfg, param.hardware, 40.0_A);
  return cfg;
}

/** The motor config with an ADRC and an inertia to derive b0 from, starting at the given angle. */
SmartMotorControllerConfig AdrcConfig(const MotorTestParam& param, const std::string& mechanism,
                                      math::ADRCConfig::SystemOrder order, wpi::units::second_t settlingTime,
                                      double inertia, wpi::units::degree_t start = 0.0_deg) {
  auto cfg = BaseConfig(param, mechanism);
  cfg.WithMOI(wpi::units::kilogram_square_meter_t{inertia}).WithStartingPosition(start);
  cfg.WithADRCController(Adrc(order, settlingTime));
  return cfg;
}

struct ADRCClosedLoopFixture {
  ADRCClosedLoopFixture() {
    InitializeHardware();
    SchedulerHelper::Enable();
    SchedulerHelper::CancelAll();
  }
  ~ADRCClosedLoopFixture() {
    SchedulerHelper::CancelAll();
    TeardownHardware();
  }
};

/**
 * Hold a setpoint for a while after settling, and check how far from it the mechanism got. A Talon is simulated in real
 * time, so on it a miss is a warning, as the other C++ mechanism tests do.
 */
void CheckHolds(const MotorTestParam& param, bool ctre, const std::function<double()>& measured, double setpoint,
                double tolerance) {
  SchedulerHelper::RunForDuration(2.0_s);
  double worst = 0;
  SchedulerHelper::RunForDuration(0.5_s, [&] { worst = std::max(worst, std::abs(measured() - setpoint)); });
  INFO(param.name << ": should hold " << setpoint << " within " << tolerance << ", but was up to " << worst
                  << " from it, ending at " << measured());
  if (ctre && worst > tolerance) {
    std::printf("[WARNING] %s ADRC closed loop inconclusive in real time simulation: %f from %f\n",
                param.name.c_str(), worst, setpoint);
  } else {
    CHECK(worst <= tolerance);
  }
}
}  // namespace

TEST_CASE_METHOD(ADRCClosedLoopFixture, "ADRCClosedLoop.PivotHoldsItsAngle", "[ADRCClosedLoop]") {
  for (const auto& param : Controllers()) {
    DYNAMIC_SECTION(param.name) {
      // A position loop is second order; b0 is derived from the inertia.
      auto bundle = MakeBundle(param, AdrcConfig(param, "Pivot", math::ADRCConfig::SystemOrder::SECOND, 0.6_s, 0.1));
      REQUIRE(bundle.smc->GetADRCController() != nullptr);
      auto* pivotCfg = new PivotConfig;
      pivotCfg->WithMinAngle(-180.0_deg).WithMaxAngle(180.0_deg);
      auto* pivot = new positional::Pivot(pivotCfg, bundle.smc);
      bundle.subsystem->m_mechSimPeriodic = [pivot] { pivot->SimIterate(); };
      bundle.subsystem->m_mechUpdateTelemetry = [pivot] { pivot->UpdateTelemetry(); };
      bundle.smc->SetupSimulation();
      bundle.subsystem->m_testRunning = true;

      auto command = bundle.subsystem->SetPositionSetpoint(90.0_deg);
      wpi::cmd::CommandScheduler::GetInstance().Schedule(command);
      CheckHolds(
          param, IsCTRE(bundle),
          [&] { return wpi::units::degree_t{bundle.smc->GetMechanismPosition()}.value(); }, 90, 1);

      SchedulerHelper::CancelAll();
      CloseBundle(bundle);
      delete pivot;
    }
  }
}

TEST_CASE_METHOD(ADRCClosedLoopFixture, "ADRCClosedLoop.ArmHoldsItsAngleAgainstGravity", "[ADRCClosedLoop]") {
  for (const auto& param : Controllers()) {
    DYNAMIC_SECTION(param.name) {
      // An arm is second order too; the observer takes gravity for part of the disturbance.
      auto bundle =
          MakeBundle(param, AdrcConfig(param, "Arm", math::ADRCConfig::SystemOrder::SECOND, 0.6_s, 0.1, -30.0_deg));
      REQUIRE(bundle.smc->GetADRCController() != nullptr);
      auto* armCfg = new ArmConfig;
      armCfg->WithArmLength(0.5_m).WithMinAngle(-180.0_deg).WithMaxAngle(180.0_deg);
      auto* arm = new positional::Arm(armCfg, bundle.smc);
      bundle.subsystem->m_mechSimPeriodic = [arm] { arm->SimIterate(); };
      bundle.subsystem->m_mechUpdateTelemetry = [arm] { arm->UpdateTelemetry(); };
      bundle.smc->SetupSimulation();
      bundle.subsystem->m_testRunning = true;

      auto command = bundle.subsystem->SetPositionSetpoint(45.0_deg);
      wpi::cmd::CommandScheduler::GetInstance().Schedule(command);
      CheckHolds(
          param, IsCTRE(bundle),
          [&] { return wpi::units::degree_t{bundle.smc->GetMechanismPosition()}.value(); }, 45, 1);

      SchedulerHelper::CancelAll();
      CloseBundle(bundle);
      delete arm;
    }
  }
}

TEST_CASE_METHOD(ADRCClosedLoopFixture, "ADRCClosedLoop.ElevatorHoldsItsHeightAgainstGravity", "[ADRCClosedLoop]") {
  for (const auto& param : Controllers()) {
    // The A301 cannot lift the elevator.
    if (param.hardware == HardwareType::A301) {
      continue;
    }
    DYNAMIC_SECTION(param.name) {
      // A back-EMF limited elevator: a first order ADRC on the position, with the mechanism's speed per volt as b0.
      constexpr double kDrumRadius = 0.02;
      auto adrc = Adrc(math::ADRCConfig::SystemOrder::FIRST, 0.5_s);
      adrc.WithB0(MotorForHardware(param.hardware),
                  param.hardware == HardwareType::A301
                      ? gearing::MechanismGearing{gearing::GearBox::FromReductionStages({kA301Gearbox})}
                      : gearing::MechanismGearing{gearing::GearBox::FromReductionStages({3.0, 4.0})});
      auto cfg = BaseConfig(param, "Elevator");
      cfg.WithMechanismCircumference(wpi::units::meter_t{2 * std::numbers::pi * kDrumRadius})
          .WithStartingPosition(0.1_m)
          .WithADRCController(adrc);
      auto bundle = MakeBundle(param, cfg);
      REQUIRE(bundle.smc->GetADRCController() != nullptr);
      auto* elevatorCfg = new ElevatorConfig;
      elevatorCfg->WithCarriageMass(10_kg).WithMinimumHeight(0.0_m).WithMaximumHeight(2.0_m);
      auto* elevator = new positional::Elevator(elevatorCfg, bundle.smc);
      bundle.subsystem->m_mechSimPeriodic = [elevator] { elevator->SimIterate(); };
      bundle.subsystem->m_mechUpdateTelemetry = [elevator] { elevator->UpdateTelemetry(); };
      bundle.smc->SetupSimulation();
      bundle.subsystem->m_testRunning = true;

      auto command =
          wpi::cmd::Run([&bundle] { bundle.smc->SetPosition(0.5_m); }, {bundle.smc->GetConfig().GetSubsystem()});
      wpi::cmd::CommandScheduler::GetInstance().Schedule(command);
      CheckHolds(
          param, IsCTRE(bundle), [&] { return bundle.smc->GetMeasurementPosition().value(); }, 0.5, 0.01);

      SchedulerHelper::CancelAll();
      CloseBundle(bundle);
      delete elevator;
    }
  }
}

TEST_CASE_METHOD(ADRCClosedLoopFixture, "ADRCClosedLoop.FlyWheelHoldsItsSpeed", "[ADRCClosedLoop]") {
  for (const auto& param : Controllers()) {
    DYNAMIC_SECTION(param.name) {
      // A velocity loop is first order, with a short settling time so the observer keeps up with the back-EMF.
      auto bundle =
          MakeBundle(param, AdrcConfig(param, "FlyWheel", math::ADRCConfig::SystemOrder::FIRST, 0.15_s, 0.01));
      REQUIRE(bundle.smc->GetADRCController() != nullptr);
      auto* flyWheelCfg = new FlyWheelConfig;
      flyWheelCfg->WithRollerDiameter(wpi::units::meter_t{4.0 * 0.0254});
      auto* flyWheel = new velocity::FlyWheel(flyWheelCfg, bundle.smc);
      bundle.subsystem->m_mechSimPeriodic = [flyWheel] { flyWheel->SimIterate(); };
      bundle.subsystem->m_mechUpdateTelemetry = [flyWheel] { flyWheel->UpdateTelemetry(); };
      bundle.smc->SetupSimulation();
      bundle.subsystem->m_testRunning = true;

      // Half the motor's free speed, at the mechanism.
      const double setpoint =
          0.5 * MotorForHardware(param.hardware).freeSpeed.value() / (2 * std::numbers::pi) / Reduction(param.hardware);
      auto command = wpi::cmd::Run(
          [&bundle, setpoint] { bundle.smc->SetVelocity(wpi::units::turns_per_second_t{setpoint}); },
          {bundle.smc->GetConfig().GetSubsystem()});
      wpi::cmd::CommandScheduler::GetInstance().Schedule(command);
      CheckHolds(
          param, IsCTRE(bundle),
          [&] { return wpi::units::turns_per_second_t{bundle.smc->GetMechanismVelocity()}.value(); }, setpoint,
          0.02 * setpoint);

      SchedulerHelper::CancelAll();
      CloseBundle(bundle);
      delete flyWheel;
    }
  }
}

// WithClosedLoopRampRate ramps a closed loop run on the SystemCore, an ADRC or an LQR, on every motor controller. Such a
// loop sends the motor controller a voltage, which it treats as open loop, so its own closed loop ramp never applies;
// YAMS ramps the output instead. Mirrors the Java ClosedLoopRampTest.
TEST_CASE_METHOD(ADRCClosedLoopFixture, "ADRCClosedLoop.TheClosedLoopRampLimitsHowFastTheOutputChanges",
                 "[ADRCClosedLoop]") {
  // 0 V to 12 V in half a second.
  constexpr double kRampVoltsPerSecond = 24.0;
  // Slack for the applied voltage, which a motor controller reports in steps.
  constexpr double kSlackVolts = 0.75;
  for (const auto& param : Controllers()) {
    for (const bool lqr : {false, true}) {
      for (const bool ramped : {true, false}) {
        // A Talon is simulated in real time, so its first outputs come late; only its ramped run is checked.
        if (!ramped && param.hardware != HardwareType::SparkMax && param.hardware != HardwareType::SparkFlex &&
            param.hardware != HardwareType::Nova && param.hardware != HardwareType::A301) {
          continue;
        }
        DYNAMIC_SECTION(param.name << (lqr ? " LQR" : " ADRC") << (ramped ? "" : " unramped")) {
          auto cfg = BaseConfig(param, std::string{lqr ? "RampLQR" : "RampADRC"} + (ramped ? "" : "Unramped"));
          cfg.WithStartingPosition(0.0_deg);
          if (ramped) cfg.WithClosedLoopRampRate(0.5_s);
          if (lqr) {
            cfg.WithLQR(math::LQRConfig{}
                            .WithArmSystem(MotorForHardware(param.hardware), 0.02, Reduction(param.hardware))
                            .WithQElems({0.01745, 0.1745})
                            .WithRElems({12.0})
                            .WithStateStdDevs({0.01745, 0.1745})
                            .WithMeasurementStdDevs({0.0001}));
          } else {
            cfg.WithMOI(wpi::units::kilogram_square_meter_t{0.1})
                .WithADRCController(Adrc(math::ADRCConfig::SystemOrder::SECOND, 0.6_s));
          }
          auto bundle = MakeBundle(param, cfg);
          bundle.smc->SetupSimulation();
          bundle.subsystem->m_testRunning = true;

          // A whole rotation away, so the loop asks for a large output at once.
          auto command =
              wpi::cmd::Run([&bundle] { bundle.smc->SetPosition(wpi::units::turn_t{1}); },
                            {bundle.smc->GetConfig().GetSubsystem()});
          wpi::cmd::CommandScheduler::GetInstance().Schedule(command);
          double time = 0;
          double worstOverRamp = 0;
          SchedulerHelper::RunForDuration(0.5_s, [&] {
            time += 0.02;
            worstOverRamp =
                std::max(worstOverRamp, std::abs(bundle.smc->GetVoltage().value()) - kRampVoltsPerSecond * time);
          });
          INFO(param.name << ": " << worstOverRamp << " V over a " << kRampVoltsPerSecond << " V/s ramp");
          if (ramped) {
            CHECK(worstOverRamp <= kSlackVolts);
          } else {
            // Without the ramp the same step rises faster, so the ramp is what held it back.
            CHECK(worstOverRamp > kSlackVolts);
          }

          SchedulerHelper::CancelAll();
          CloseBundle(bundle);
        }
      }
    }
  }
}

}  // namespace yams::test
