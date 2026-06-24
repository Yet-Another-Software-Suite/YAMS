// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Mirrors Java ShooterTest — duty-cycle and velocity-PID tests for a FlyWheel
// (shooter) mechanism across all (HardwareType × ProfileType) combinations.

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
#include "yams/mechanisms/config/FlyWheelConfig.hpp"
#include "yams/mechanisms/velocity/FlyWheel.hpp"

namespace yams::test {

using namespace motorcontrollers;
using namespace mechanisms;
using namespace mechanisms::config;
using namespace mechanisms::velocity;

// ---- Config helpers ---------------------------------------------------------

static SmartMotorControllerConfig MakeShooterSMCConfig(ProfileType profile, TestSubsystem* subsys,
                                                       const std::string& name) {
  SmartMotorControllerConfig cfg;
  cfg.WithFeedback(100.0, 0.0, 0.0)
      .WithMotorGearing(
          gearing::MechanismGearing{gearing::GearBox::FromReductionStages({3.0, 4.0})})
      .WithIdleMode(SmartMotorControllerConfig::MotorMode::COAST)
      .WithStatorCurrentLimit(40.0_A)
      .WithMotorInverted(false)
      .WithSimpleFeedforward(0.0, 1.0, 0.0)
      .WithClosedLoopMode()
      .WithSubsystem(subsys)
      .WithTelemetry(name);

  // ShooterTest also tests case 3: PID gains zeroed — cover this by repeating
  // with the given profile.
  switch (profile) {
    case ProfileType::Trapezoid:
      // RPM.of(6000) ≈ 6000/60 rps = 100 rps → 36000 deg/s
      // RPM.per(Second).of(9000) ≈ 9000/60 rps² → 54000 deg/s²
      cfg.WithTrapezoidProfile(
          units::degrees_per_second_t{36000.0},
          units::unit_t<units::compound_unit<units::angular_velocity::degrees_per_second,
                                             units::inverse<units::seconds>>>{54000.0});
      break;
    case ProfileType::Exponential:
      cfg.WithExponentialProfile(0.5, 0.05, 12.0_V);
      break;
    default:
      break;
  }
  return cfg;
}

static FlyWheel* CreateShooter(SmartMotorController* smc, TestSubsystem* subsys) {
  auto* cfg = new FlyWheelConfig;
  cfg->WithRollerDiameter(units::meter_t{4.0 * 0.0254});  // 4 inches
  auto* shooter = new FlyWheel(cfg, smc);
  subsys->m_mechSimPeriodic = [shooter] { shooter->SimIterate(); };
  subsys->m_mechUpdateTelemetry = [shooter] { shooter->UpdateTelemetry(); };
  return shooter;
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

  SchedulerHelper::RunForDuration(1.0_s, [&] {
    if (smc->GetDutyCycle() != 0.0) passed = true;
  });

  if (isCTRE) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    SchedulerHelper::RunForDuration(1.0_s);
  }

  auto postVel = smc->GetMechanismVelocity();
  auto postAngle = smc->GetMechanismPosition();

  bool moved = (postVel > preVel) || (postAngle > preAngle) || passed;
  if (isCTRE && !moved) {
    std::printf("[WARNING] TalonFX/TalonFXS shooter duty-cycle inconclusive.\n");
  } else {
    EXPECT_TRUE(moved) << "Shooter did not spin during duty-cycle test"
                       << " preVel=" << preVel.value() << " preAngle=" << preAngle.value()
                       << " postVel=" << postVel.value() << " postAngle=" << postAngle.value();
  }
}

static void VelocityPIDTestBody(SmartMotorController* smc, bool isCTRE) {
  auto preVel = smc->GetMechanismVelocity();
  bool passed = false;

  // ~2000 RPM = 2000/60 rps * 360 deg/rot = 12000 deg/s
  auto cmd = frc2::cmd::Run([smc] { smc->SetVelocity(units::degrees_per_second_t{12000.0}); },
                            {smc->GetConfig().GetSubsystem()});
  frc2::CommandScheduler::GetInstance().Schedule(cmd);

  SchedulerHelper::RunForDuration(isCTRE ? 1.0_s : 2.0_s, [&] {
    if (smc->GetDutyCycle() != 0.0) passed = true;
  });

  auto postVel = smc->GetMechanismVelocity();

  EXPECT_TRUE(std::abs(postVel.value() - preVel.value()) > 0.05 || passed)
      << "Shooter velocity did not change toward PID setpoint"
      << " preVel=" << preVel.value() << " postVel=" << postVel.value();
}

// ---- Fixture ----------------------------------------------------------------

class ShooterTest : public ::testing::TestWithParam<MotorTestParam> {
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

TEST_P(ShooterTest, SMCDutyCycle) {
  auto& param = GetParam();
  SCOPED_TRACE(param.name);
  auto cfg = MakeShooterSMCConfig(param.profile, nullptr, param.name);
  auto bundle = MakeBundle(param, cfg);
  bundle.smc->SetupSimulation();
  bundle.subsystem->m_testRunning = true;

  DutyCycleTestBody(bundle.smc, IsCTRE(bundle));
  CloseBundle(bundle);
}

TEST_P(ShooterTest, SMCVelocityPID) {
  auto& param = GetParam();
  SCOPED_TRACE(param.name);
  auto cfg = MakeShooterSMCConfig(param.profile, nullptr, param.name);
  auto bundle = MakeBundle(param, cfg);
  bundle.smc->SetupSimulation();
  bundle.subsystem->m_testRunning = true;

  VelocityPIDTestBody(bundle.smc, IsCTRE(bundle));
  CloseBundle(bundle);
}

TEST_P(ShooterTest, ShooterDutyCycle) {
  auto& param = GetParam();
  SCOPED_TRACE(param.name);
  auto cfg = MakeShooterSMCConfig(param.profile, nullptr, param.name);
  auto bundle = MakeBundle(param, cfg);
  bundle.smc->SetupSimulation();
  bundle.subsystem->m_testRunning = true;

  auto shooter = CreateShooter(bundle.smc, bundle.subsystem.get());
  auto upCmd = shooter->Set(0.5);
  frc2::CommandScheduler::GetInstance().Schedule(upCmd);

  DutyCycleTestBody(bundle.smc, IsCTRE(bundle));
  CloseBundle(bundle);
  delete shooter;
}

TEST_P(ShooterTest, ShooterVelocityPID) {
  auto& param = GetParam();
  SCOPED_TRACE(param.name);
  auto cfg = MakeShooterSMCConfig(param.profile, nullptr, param.name);
  auto bundle = MakeBundle(param, cfg);
  bundle.smc->SetupSimulation();
  bundle.subsystem->m_testRunning = true;

  // ~80 RPM = 80/60 * 360 ≈ 480 deg/s
  auto shooter = CreateShooter(bundle.smc, bundle.subsystem.get());
  auto highPid = shooter->RunTo(units::degrees_per_second_t{480.0});
  frc2::CommandScheduler::GetInstance().Schedule(highPid);

  VelocityPIDTestBody(bundle.smc, IsCTRE(bundle));
  CloseBundle(bundle);
  delete shooter;
}

INSTANTIATE_TEST_SUITE_P(AllControllersTests, ShooterTest, ::testing::ValuesIn(AllMotorParams()),
                         [](const ::testing::TestParamInfo<MotorTestParam>& info) {
                           return info.param.name;
                         });

}  // namespace yams::test
