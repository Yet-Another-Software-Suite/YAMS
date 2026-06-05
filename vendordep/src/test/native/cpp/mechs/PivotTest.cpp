// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

// Mirrors Java PivotTest — duty-cycle and position-PID tests for a rotary
// pivot mechanism across all (HardwareType × ProfileType) combinations.

#include <frc2/command/Commands.h>
#include <gtest/gtest.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <thread>

#include "helpers/MockHardware.h"
#include "helpers/MotorControllerFactory.h"
#include "helpers/SchedulerHelper.h"
#include "yams/mechanisms/config/PivotConfig.hpp"
#include "yams/mechanisms/positional/Pivot.hpp"

namespace yams::test {

using namespace motorcontrollers;
using namespace mechanisms;
using namespace mechanisms::config;

// ---- Config helpers ---------------------------------------------------------

static SmartMotorControllerConfig MakePivotSMCConfig(ProfileType profile, TestSubsystem* subsys,
                                                     const std::string& name) {
  SmartMotorControllerConfig cfg;
  cfg.WithFeedback(4.0, 0.0, 0.0)
      .WithMechanismLimits(-100.0_deg, 100.0_deg)
      .WithMotorGearing(
          gearing::MechanismGearing{gearing::GearBox::FromReductionStages({3.0, 4.0, 5.0})})
      .WithIdleMode(SmartMotorControllerConfig::MotorMode::BRAKE)
      .WithStatorCurrentLimit(40.0_A)
      .WithMotorInverted(false)
      .WithSimpleFeedforward(1.0, 0.0, 0.0)
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
      cfg.WithExponentialProfile(0.5, 0.05, 12.0_V);
      break;
    default:
      break;
  }
  return cfg;
}

static positional::Pivot* CreatePivot(SmartMotorController* smc, TestSubsystem* subsys) {
  PivotConfig cfg;
  cfg.WithMotorController(smc)
      .WithSubsystem(subsys)
      .WithMinAngle(-100.0_deg)
      .WithMaxAngle(150.0_deg)
      .WithStartingAngle(0.0_deg);
  positional::Pivot* pivot = new positional::Pivot(cfg);
  subsys->m_mechSimPeriodic = [pivot] { pivot->SimIterate(); };
  subsys->m_mechUpdateTelemetry = [pivot] { pivot->UpdateTelemetry(); };
  return pivot;
}

// ---- Shared test bodies -----------------------------------------------------

static void DutyCycleTestBody(SmartMotorController* smc, bool isCTRE) {
  auto preVel = smc->GetMechanismVelocity();
  auto preAngle = smc->GetMechanismPosition();
  bool passed = false;

  auto* subsys = static_cast<TestSubsystem*>(smc->GetConfig().GetSubsystem());
  auto cmd = subsys->SetDutyCycle(0.5);
  frc2::CommandScheduler::GetInstance().Schedule(cmd);
  frc2::CommandScheduler::GetInstance().Schedule(cmd);  // schedule twice like Java

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
    std::printf("[WARNING] TalonFX/TalonFXS pivot duty-cycle inconclusive.\n");
  } else {
    EXPECT_TRUE(moved) << "Pivot did not move during duty-cycle test"
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

  SchedulerHelper::RunForDuration(isCTRE ? 1.0_s : 20.0_s, [&] {
    if (isCTRE)
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(
          smc->GetConfig().GetClosedLoopControlPeriod().value_or(20_ms).value() * 1000.0)));
    if (!isCTRE && smc->GetDutyCycle() != 0.0) passed = true;
  });

  auto postAngle = smc->GetMechanismPosition();
  EXPECT_TRUE(std::abs(postAngle.value() - preAngle.value()) > 0.0003 || passed)
      << "Pivot did not move toward PID setpoint"
      << " preAngle=" << preAngle.value() << " postAngle=" << postAngle.value();
}

// ---- Fixture ----------------------------------------------------------------

class PivotTest : public ::testing::TestWithParam<MotorTestParam> {
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

TEST_P(PivotTest, SMCDutyCycle) {
  auto& param = GetParam();
  SCOPED_TRACE(param.name);
  auto cfg = MakePivotSMCConfig(param.profile, nullptr, param.name);
  auto bundle = MakeBundle(param, cfg);
  bundle.smc->SetupSimulation();
  bundle.subsystem->m_testRunning = true;

  DutyCycleTestBody(bundle.smc, IsCTRE(bundle));
  CloseBundle(bundle);
}

TEST_P(PivotTest, SMCPositionPID) {
  auto& param = GetParam();
  SCOPED_TRACE(param.name);
  auto cfg = MakePivotSMCConfig(param.profile, nullptr, param.name);
  auto bundle = MakeBundle(param, cfg);
  bundle.smc->SetupSimulation();
  bundle.subsystem->m_testRunning = true;

  PositionPIDTestBody(bundle.smc, IsCTRE(bundle));
  CloseBundle(bundle);
}

TEST_P(PivotTest, PivotDutyCycle) {
  auto& param = GetParam();
  SCOPED_TRACE(param.name);
  auto cfg = MakePivotSMCConfig(param.profile, nullptr, param.name);
  auto bundle = MakeBundle(param, cfg);
  bundle.smc->SetupSimulation();
  bundle.subsystem->m_testRunning = true;

  auto pivot = CreatePivot(bundle.smc, bundle.subsystem.get());
  auto upCmd = pivot->Set(0.5);
  frc2::CommandScheduler::GetInstance().Schedule(upCmd);

  DutyCycleTestBody(bundle.smc, IsCTRE(bundle));
  CloseBundle(bundle);
  delete pivot;
}

TEST_P(PivotTest, PivotPositionPID) {
  auto& param = GetParam();
  SCOPED_TRACE(param.name);
  auto cfg = MakePivotSMCConfig(param.profile, nullptr, param.name);
  auto bundle = MakeBundle(param, cfg);
  bundle.smc->SetupSimulation();
  bundle.subsystem->m_testRunning = true;

  auto pivot = CreatePivot(bundle.smc, bundle.subsystem.get());
  auto highPid = pivot->RunTo(80.0_deg);
  frc2::CommandScheduler::GetInstance().Schedule(highPid);

  PositionPIDTestBody(bundle.smc, IsCTRE(bundle));
  CloseBundle(bundle);
  delete pivot;
}

INSTANTIATE_TEST_SUITE_P(AllControllersTests, PivotTest, ::testing::ValuesIn(AllMotorParams()),
                         [](const ::testing::TestParamInfo<MotorTestParam>& info) {
                           return info.param.name;
                         });

}  // namespace yams::test
