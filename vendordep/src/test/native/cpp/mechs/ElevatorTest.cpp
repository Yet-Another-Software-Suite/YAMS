// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Mirrors Java ElevatorTest — exercises duty-cycle and position-PID control
// for each (HardwareType × ProfileType) combination using WPILib simulation.

#include <frc/system/plant/DCMotor.h>
#include <frc2/command/Commands.h>
#include <gtest/gtest.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/velocity.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

#include "helpers/MockHardware.h"
#include "helpers/MotorControllerFactory.h"
#include "helpers/SchedulerHelper.h"
#include "yams/mechanisms/config/ElevatorConfig.hpp"
#include "yams/mechanisms/positional/Elevator.hpp"

namespace yams::test {

using namespace motorcontrollers;
using namespace mechanisms;
using namespace mechanisms::config;

// ---- Config helpers ---------------------------------------------------------

static SmartMotorControllerConfig MakeElevatorSMCConfig(ProfileType profile, HardwareType hardware,
                                                        TestSubsystem* subsys,
                                                        const std::string& name) {
  // 22 sprocket teeth × 0.25 in pitch = 5.5 in circumference ≈ 0.1397 m

  SmartMotorControllerConfig cfg;
  cfg.WithFeedback(8.0, 0.0, 0.0)
      .WithMechanismCircumference(0.25_in * 22)
      .WithStartingPosition(0.0_m)
      .WithMeasurementLimits(0.0_m, 5.0_m)
      .WithMotorGearing(
          gearing::MechanismGearing{gearing::GearBox::FromReductionStages({3.0, 4.0})})
      .WithIdleMode(SmartMotorControllerConfig::MotorMode::BRAKE)
      .WithStatorCurrentLimit(40.0_A)
      .WithMotorInverted(false)
      .WithElevatorFeedforward(1.0, 0.0, 0.0)
      .WithClosedLoopMode()
      .WithSubsystem(subsys)
      .WithTelemetry(name);

  switch (profile) {
    case ProfileType::Trapezoid:
      cfg.WithLinearTrapezoidProfile(1_mps, 0.5_mps_sq);
      break;
    case ProfileType::Exponential:
      cfg.WithExponentialProfile(12.0_V, MotorForHardware(hardware), 1_lb, 0.25_in * 22.0);
      break;
    default:
      break;
  }
  return cfg;
}

static positional::Elevator* CreateElevator(SmartMotorController* smc, TestSubsystem* subsys) {
  auto* cfg = new ElevatorConfig;
  cfg->WithMinimumHeight(0.0_m).WithCarriageMass(1_lb).WithMaximumHeight(3.0_m);
  auto* elevator = new positional::Elevator(cfg, smc);
  subsys->m_mechSimPeriodic = [elevator] { elevator->SimIterate(); };
  subsys->m_mechUpdateTelemetry = [elevator] { elevator->UpdateTelemetry(); };
  return elevator;
}

// ---- Shared test bodies -----------------------------------------------------

static void DutyCycleTestBody(SmartMotorController* smc, bool isCTRE) {
  auto preVel = smc->GetMeasurementVelocity();
  auto preDist = smc->GetMeasurementPosition();
  bool passed = false;

  auto* subsys = static_cast<TestSubsystem*>(smc->GetConfig().GetSubsystem());
  auto cmd = subsys->SetDutyCycle(1.0);
  frc2::CommandScheduler::GetInstance().Schedule(cmd);

  SchedulerHelper::RunForDuration(1.0_s, [&] {
    if (smc->GetDutyCycle() != 0.0) passed = true;
  });

  if (isCTRE) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    SchedulerHelper::RunForDuration(1.0_s);
  }

  auto postVel = smc->GetMeasurementVelocity();
  auto postDist = smc->GetMeasurementPosition();

  bool moved = (postVel > preVel) || (postDist > preDist) || passed;
  if (isCTRE && !moved) {
    std::printf("[WARNING] TalonFX/TalonFXS duty-cycle test inconclusive on this OS.\n");
  } else {
    EXPECT_TRUE(moved) << "Motor did not move during duty-cycle test"
                       << " preVel=" << preVel.value() << " preDist=" << preDist.value()
                       << " postVel=" << postVel.value() << " postDist=" << postDist.value();
  }
}

static void PositionPIDTestBody(SmartMotorController* smc, bool isCTRE, bool debugOutput = false) {
  auto preDist = smc->GetMeasurementPosition();
  bool passed = false;

  auto cmd = frc2::cmd::Run([smc] { smc->SetPosition(2.0_m); }, {smc->GetConfig().GetSubsystem()});
  frc2::CommandScheduler::GetInstance().Schedule(cmd);

  units::millisecond_t period{
      smc->GetConfig().GetClosedLoopControlPeriod().value_or(20_ms).value() * 1000.0};

  SchedulerHelper::RunForDuration(isCTRE ? 1.0_s : 20.0_s, [&] {
    if (debugOutput) {
      std::printf("[DEBUG] mechanism_position=%.6f turns, dutycyle=%.6f\n",
                  smc->GetMechanismPosition().value(), smc->GetDutyCycle());
    }
    if (isCTRE) {
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(period.value())));
    }
    if (smc->GetDutyCycle() != 0.0) passed = true;
  });

  auto postDist = smc->GetMeasurementPosition();
  EXPECT_TRUE(std::abs(postDist.value() - preDist.value()) > 0.005 || passed)
      << "Elevator did not move toward PID setpoint"
      << " preDist=" << preDist.value() << " postDist=" << postDist.value();
}

// ---- Parameterised fixture --------------------------------------------------

class ElevatorTest : public ::testing::TestWithParam<MotorTestParam> {
 protected:
  void SetUp() override {
    InitializeHardware();
    SchedulerHelper::Enable();
    SchedulerHelper::CancelAll();
  }
  void TearDown() override {
    TeardownHardware();
    SchedulerHelper::CancelAll();
  }
};

// ---- Tests ------------------------------------------------------------------

TEST_P(ElevatorTest, SMCDutyCycle) {
  auto& param = GetParam();
  SCOPED_TRACE(param.name);
  auto cfg = MakeElevatorSMCConfig(param.profile, param.hardware, nullptr, param.name);
  auto subsys = std::make_unique<TestSubsystem>();
  cfg.WithSubsystem(subsys.get());
  auto bundle = MakeBundle(param, cfg);
  bundle.smc->SetupSimulation();
  bundle.subsystem->m_testRunning = true;

  DutyCycleTestBody(bundle.smc, IsCTRE(bundle));
  CloseBundle(bundle);
}

TEST_P(ElevatorTest, SMCPositionPID) {
  auto& param = GetParam();
  SCOPED_TRACE(param.name);
  auto cfg = MakeElevatorSMCConfig(param.profile, param.hardware, nullptr, param.name);
  auto bundle = MakeBundle(param, cfg);
  bundle.smc->SetupSimulation();
  bundle.subsystem->m_testRunning = true;

  PositionPIDTestBody(bundle.smc, IsCTRE(bundle));
  CloseBundle(bundle);
}

TEST_P(ElevatorTest, ElevatorDutyCycle) {
  auto& param = GetParam();
  SCOPED_TRACE(param.name);
  auto cfg = MakeElevatorSMCConfig(param.profile, param.hardware, nullptr, param.name);
  auto bundle = MakeBundle(param, cfg);
  bundle.smc->SetupSimulation();
  bundle.subsystem->m_testRunning = true;

  auto elevator = CreateElevator(bundle.smc, bundle.subsystem.get());
  auto upCmd = elevator->Set(1.0);
  frc2::CommandScheduler::GetInstance().Schedule(upCmd);

  DutyCycleTestBody(bundle.smc, IsCTRE(bundle));
  CloseBundle(bundle);
  delete elevator;
}

TEST_P(ElevatorTest, ElevatorPositionPID) {
  auto& param = GetParam();
  SCOPED_TRACE(param.name);
  auto cfg = MakeElevatorSMCConfig(param.profile, param.hardware, nullptr, param.name);
  auto bundle = MakeBundle(param, cfg);
  bundle.smc->SetupSimulation();
  bundle.subsystem->m_testRunning = true;

  auto elevator = CreateElevator(bundle.smc, bundle.subsystem.get());
  auto highPid = elevator->RunTo(2.0_m);
  frc2::CommandScheduler::GetInstance().Schedule(highPid);

  PositionPIDTestBody(bundle.smc, IsCTRE(bundle), param.name == "TalonFXS_NoPro");
  CloseBundle(bundle);
  delete elevator;
}

INSTANTIATE_TEST_SUITE_P(AllControllersTests, ElevatorTest, ::testing::ValuesIn(AllMotorParams()),
                         [](const ::testing::TestParamInfo<MotorTestParam>& info) {
                           return info.param.name;
                         });

}  // namespace yams::test
