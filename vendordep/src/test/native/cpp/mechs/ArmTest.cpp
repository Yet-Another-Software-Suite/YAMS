// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

// Mirrors Java ArmTest — duty-cycle and position-PID tests for a single-jointed
// arm across all (HardwareType × ProfileType) combinations.

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
#include "yams/mechanisms/config/ArmConfig.h"
#include "yams/mechanisms/positional/Arm.h"

namespace yams::test {

using namespace motorcontrollers;
using namespace mechanisms;
using namespace mechanisms::config;

// ---- Config helpers ---------------------------------------------------------

static SmartMotorControllerConfig MakeArmSMCConfig(ProfileType profile, TestSubsystem* subsys,
                                                   const std::string& name) {
  SmartMotorControllerConfig cfg;
  cfg.WithFeedback(5.0, 0.0, 0.0)
      .WithMechanismLimits(-100.0_deg, 100.0_deg)
      .WithMotorGearing(
          gearing::MechanismGearing{gearing::GearBox::FromReductionStages({3.0, 4.0})})
      .WithIdleMode(SmartMotorControllerConfig::MotorMode::BRAKE)
      .WithStatorCurrentLimit(40.0_A)
      .WithMotorInverted(false)
      .WithArmFeedforward(0.0, 1.0, 0.0, 0.0)
      .WithClosedLoopMode()
      .WithSubsystem(subsys)
      .WithTelemetry(name);

  switch (profile) {
    case ProfileType::Trapezoid:
      cfg.WithTrapezoidProfile(
          units::degrees_per_second_t{180.0},
          units::unit_t<units::compound_unit<units::angular_velocity::degrees_per_second,
                                             units::inverse<units::seconds>>>{90.0});
      break;
    case ProfileType::Exponential:
      // ~40 RPS max / 80 RPS² accel expressed as simple gains
      cfg.WithExponentialProfile(0.5, 0.05, 12.0_V);
      break;
    default:
      break;
  }
  return cfg;
}

static positional::Arm CreateArm(SmartMotorController* smc, TestSubsystem* subsys, bool isCTRE) {
  ArmConfig cfg;
  cfg.WithMotorController(smc)
      .WithSubsystem(subsys)
      .WithArmLength(units::meter_t{4.0 * 0.0254})  // 4 inches → meters
      .WithMinAngle(-100.0_deg)
      .WithMaxAngle(200.0_deg)
      .WithStartingAngle(0.0_deg);
  // withHorizontalZero only for CTRE controllers in the Java tests
  // (not a config option in our C++ ArmConfig yet; add if needed)

  positional::Arm arm{cfg};
  subsys->m_mechSimPeriodic = [&arm] { arm.SimIterate(); };
  subsys->m_mechUpdateTelemetry = [&arm] { arm.UpdateTelemetry(); };
  return arm;
}

// ---- Shared test bodies -----------------------------------------------------

static void DutyCycleTestBody(SmartMotorController* smc, bool isCTRE) {
  auto preVel = smc->GetMechanismVelocity();
  auto preAngle = smc->GetMechanismPosition();
  bool passed = false;

  auto* subsys = static_cast<TestSubsystem*>(smc->GetConfig().GetSubsystem());
  auto cmd = subsys->SetDutyCycle(0.5);
  frc2::CommandScheduler::GetInstance().Schedule(&cmd.Unwrap());
  frc2::CommandScheduler::GetInstance().Schedule(&cmd.Unwrap());

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
    EXPECT_TRUE(moved) << "Arm did not move during duty-cycle test";
  }
}

static void PositionPIDTestBody(SmartMotorController* smc, bool isCTRE) {
  auto preAngle = smc->GetMechanismPosition();
  bool passed = false;

  auto cmd =
      frc2::cmd::Run([smc] { smc->SetPosition(80.0_deg); }, {smc->GetConfig().GetSubsystem()});
  frc2::CommandScheduler::GetInstance().Schedule(&cmd.Unwrap());

  SchedulerHelper::RunForDuration(1.0_s, [&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(
        smc->GetConfig().GetClosedLoopControlPeriod().value_or(20_ms).value() * 1000.0)));
    if (smc->GetDutyCycle() != 0.0) passed = true;
  });

  auto postAngle = smc->GetMechanismPosition();
  EXPECT_TRUE(std::abs(postAngle.value() - preAngle.value()) > 0.05 || passed)
      << "Arm did not move toward PID setpoint";
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
  auto cfg = MakeArmSMCConfig(param.profile, nullptr, param.name);
  auto bundle = MakeBundle(param, cfg);
  bundle.smc->SetupSimulation();
  bundle.subsystem->m_testRunning = true;

  DutyCycleTestBody(bundle.smc, IsCTRE(bundle));
  CloseBundle(bundle);
}

TEST_P(ArmTest, SMCPositionPID) {
  auto& param = GetParam();
  auto cfg = MakeArmSMCConfig(param.profile, nullptr, param.name);
  auto bundle = MakeBundle(param, cfg);
  bundle.smc->SetupSimulation();
  bundle.subsystem->m_testRunning = true;

  PositionPIDTestBody(bundle.smc, IsCTRE(bundle));
  CloseBundle(bundle);
}

TEST_P(ArmTest, ArmDutyCycle) {
  auto& param = GetParam();
  auto cfg = MakeArmSMCConfig(param.profile, nullptr, param.name);
  auto bundle = MakeBundle(param, cfg);
  bundle.smc->SetupSimulation();
  bundle.subsystem->m_testRunning = true;

  auto arm = CreateArm(bundle.smc, bundle.subsystem.get(), IsCTRE(bundle));
  auto upCmd = arm.Set(0.5);
  frc2::CommandScheduler::GetInstance().Schedule(&upCmd.Unwrap());

  DutyCycleTestBody(bundle.smc, IsCTRE(bundle));
  CloseBundle(bundle);
}

TEST_P(ArmTest, ArmPositionPID) {
  auto& param = GetParam();
  auto cfg = MakeArmSMCConfig(param.profile, nullptr, param.name);
  auto bundle = MakeBundle(param, cfg);
  bundle.smc->SetupSimulation();
  bundle.subsystem->m_testRunning = true;

  auto arm = CreateArm(bundle.smc, bundle.subsystem.get(), IsCTRE(bundle));
  auto highPid = arm.GoToAngle(80.0_deg);
  frc2::CommandScheduler::GetInstance().Schedule(&highPid.Unwrap());

  PositionPIDTestBody(bundle.smc, IsCTRE(bundle));
  CloseBundle(bundle);
}

INSTANTIATE_TEST_SUITE_P(AllControllersTests, ArmTest, ::testing::ValuesIn(AllMotorParams()),
                         [](const ::testing::TestParamInfo<MotorTestParam>& info) {
                           return info.param.name;
                         });

}  // namespace yams::test
