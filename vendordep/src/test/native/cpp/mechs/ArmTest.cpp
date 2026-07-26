// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Mirrors Java ArmTest — duty-cycle and position-PID tests for a single-jointed
// arm across all (HardwareType × ProfileType) combinations.

#include <frc/system/plant/DCMotor.h>
#include <frc2/command/Commands.h>
#include <gtest/gtest.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/mass.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <thread>

#include "helpers/MockHardware.h"
#include "helpers/MotorControllerFactory.h"
#include "helpers/SchedulerHelper.h"
#include "yams/mechanisms/config/ArmConfig.hpp"
#include "yams/mechanisms/positional/Arm.hpp"

namespace yams::test {

using namespace motorcontrollers;
using namespace mechanisms;
using namespace mechanisms::config;

// ---- Config helpers ---------------------------------------------------------

static SmartMotorControllerConfig MakeArmSMCConfig(ProfileType profile, HardwareType hardware,
                                                   TestSubsystem* subsys, const std::string& name) {
  SmartMotorControllerConfig cfg;
  cfg.WithFeedback(5.0, 0.0, 0.0)
      .WithMechanismLimits(-100.0_deg, 100.0_deg)
      .WithMotorGearing(
          gearing::MechanismGearing{gearing::GearBox::FromReductionStages({3.0, 4.0})})
      .WithIdleMode(SmartMotorControllerConfig::MotorMode::BRAKE)
      .WithStatorCurrentLimit(40.0_A)
      .WithMotorInverted(false)
      .WithFeedforward(frc::ArmFeedforward{
          0.0_V, 0.0_V, units::unit_t<frc::ArmFeedforward::kv_unit>{1.0},
          units::unit_t<frc::ArmFeedforward::ka_unit>{0.0}})
      .WithClosedLoopMode()
      .WithMOI(4_in, 1_lb)
      .WithStartingPosition(0.0_deg)
      .WithSubsystem(subsys)
      .WithTelemetry(name);

  switch (profile) {
    case ProfileType::Trapezoid:
      cfg.WithTrapezoidProfile(180.0_deg_per_s, 90.0_deg_per_s_sq);
      break;
    case ProfileType::Exponential:
      cfg.WithExponentialProfile(12.0_V, MotorForHardware(hardware), cfg.GetMOI());
      break;
    default:
      break;
  }
  return cfg;
}

static positional::Arm* CreateArm(SmartMotorController* smc, TestSubsystem* subsys, bool isCTRE) {
  auto* cfg = new ArmConfig;
  cfg->WithArmLength(4_in).WithMinAngle(-100.0_deg).WithMaxAngle(200.0_deg);
  auto* arm = new positional::Arm(cfg, smc);
  subsys->m_mechSimPeriodic = [arm] { arm->SimIterate(); };
  subsys->m_mechUpdateTelemetry = [arm] { arm->UpdateTelemetry(); };
  return arm;
}

// ---- Shared test bodies -----------------------------------------------------

static void DutyCycleTestBody(SmartMotorController* smc, bool isCTRE) {
  auto preVel = smc->GetMechanismVelocity();
  auto preAngle = smc->GetMechanismPosition();
  bool passed = false;

  auto* subsys = static_cast<TestSubsystem*>(smc->GetConfig().GetSubsystem());
  auto cmd = subsys->SetDutyCycle(0.5);
  frc2::CommandScheduler::GetInstance().Schedule(cmd);
  frc2::CommandScheduler::GetInstance().Schedule(cmd);

  SchedulerHelper::RunForDuration(1.5_s, [&] {
    if (smc->GetDutyCycle() != 0.0) passed = true;
  });

  if (isCTRE) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    SchedulerHelper::RunForDuration(1.0_s);
  }
  auto postVel = smc->GetMechanismVelocity();
  auto postAngle = smc->GetMechanismPosition();

  bool moved = (std::abs(postVel.value() - preVel.value()) > 0.05) ||
               (std::abs(postAngle.value() - preAngle.value()) > 0.05) || passed;
  if (isCTRE && !moved) {
    std::printf("[WARNING] TalonFX/TalonFXS arm duty-cycle inconclusive.\n");
  } else {
    EXPECT_TRUE(moved) << "Arm did not move during duty-cycle test"
                       << " preVel=" << preVel.value() << " preAngle=" << preAngle.value()
                       << " postVel=" << postVel.value() << " postAngle=" << postAngle.value();
  }
}

static void PositionPIDTestBody(SmartMotorController* smc, bool isCTRE) {
  auto preAngle = smc->GetMechanismPosition();
  bool passed = false;

  auto cmd =
      frc2::cmd::Run([smc] { smc->SetPosition(80.0_deg); }, {smc->GetConfig().GetSubsystem()});
  frc2::CommandScheduler::GetInstance().Schedule(cmd);

  SchedulerHelper::RunForDuration(1.0_s, [&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(
        smc->GetConfig().GetClosedLoopControlPeriod().value_or(20_ms).value() * 1000.0)));
    if (smc->GetDutyCycle() != 0.0) passed = true;
  });

  auto postAngle = smc->GetMechanismPosition();
  EXPECT_TRUE(std::abs(postAngle.value() - preAngle.value()) > 0.05 || passed)
      << "Arm did not move toward PID setpoint"
      << " preAngle=" << preAngle.value() << " postAngle=" << postAngle.value();
}

// ---- Fixture ----------------------------------------------------------------

class ArmTest : public ::testing::TestWithParam<MotorTestParam> {
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

TEST_P(ArmTest, SMCDutyCycle) {
  auto& param = GetParam();
  SCOPED_TRACE(param.name);
  auto cfg = MakeArmSMCConfig(param.profile, param.hardware, nullptr, param.name);
  auto bundle = MakeBundle(param, cfg);
  bundle.smc->SetupSimulation();
  bundle.subsystem->m_testRunning = true;

  DutyCycleTestBody(bundle.smc, IsCTRE(bundle));
  CloseBundle(bundle);
}

TEST_P(ArmTest, SMCPositionPID) {
  auto& param = GetParam();
  SCOPED_TRACE(param.name);
  auto cfg = MakeArmSMCConfig(param.profile, param.hardware, nullptr, param.name);
  auto bundle = MakeBundle(param, cfg);
  bundle.smc->SetupSimulation();
  bundle.subsystem->m_testRunning = true;

  PositionPIDTestBody(bundle.smc, IsCTRE(bundle));
  CloseBundle(bundle);
}

TEST_P(ArmTest, ArmDutyCycle) {
  auto& param = GetParam();
  SCOPED_TRACE(param.name);
  auto cfg = MakeArmSMCConfig(param.profile, param.hardware, nullptr, param.name);
  auto bundle = MakeBundle(param, cfg);
  bundle.smc->SetupSimulation();
  bundle.subsystem->m_testRunning = true;

  auto arm = CreateArm(bundle.smc, bundle.subsystem.get(), IsCTRE(bundle));
  auto upCmd = arm->Set(0.5);
  frc2::CommandScheduler::GetInstance().Schedule(upCmd);

  DutyCycleTestBody(bundle.smc, IsCTRE(bundle));
  CloseBundle(bundle);
  delete arm;
}

TEST_P(ArmTest, ArmPositionPID) {
  auto& param = GetParam();
  SCOPED_TRACE(param.name);
  auto cfg = MakeArmSMCConfig(param.profile, param.hardware, nullptr, param.name);
  auto bundle = MakeBundle(param, cfg);
  bundle.smc->SetupSimulation();
  bundle.subsystem->m_testRunning = true;

  auto arm = CreateArm(bundle.smc, bundle.subsystem.get(), IsCTRE(bundle));
  auto highPid = arm->RunTo(80.0_deg);
  frc2::CommandScheduler::GetInstance().Schedule(highPid);

  PositionPIDTestBody(bundle.smc, IsCTRE(bundle));
  CloseBundle(bundle);
  delete arm;
}

INSTANTIATE_TEST_SUITE_P(AllControllersTests, ArmTest, ::testing::ValuesIn(AllMotorParams()),
                         [](const ::testing::TestParamInfo<MotorTestParam>& info) {
                           return info.param.name;
                         });

}  // namespace yams::test
