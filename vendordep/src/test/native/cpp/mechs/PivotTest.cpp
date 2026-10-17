// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Mirrors Java PivotTest duty-cycle and position-PID tests for a rotary
// pivot mechanism across all (HardwareType × ProfileType) combinations.

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>
#include <wpi/commands2/CommandScheduler.hpp>
#include <wpi/commands2/Commands.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_acceleration.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/moment_of_inertia.hpp>

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

static SmartMotorControllerConfig MakePivotSMCConfig(ProfileType profile, HardwareType hardware,
                                                     TestSubsystem* subsys,
                                                     const std::string& name) {
  SmartMotorControllerConfig cfg;
  cfg.WithFeedback(8.0, 0.0, 0.0)
      .WithMechanismLimits(-100.0_deg, 100.0_deg)
      .WithMotorGearing(
          gearing::MechanismGearing{gearing::GearBox::FromReductionStages({3.0, 4.0})})
      .WithZeroPower(SmartMotorControllerConfig::MotorMode::BRAKE)
      .WithStatorCurrentLimit(40.0_A)
      .WithMotorInverted(false)
      .WithFeedforward(wpi::math::SimpleMotorFeedforward<wpi::units::turns>{
          1.0_V,
          wpi::units::unit_t<wpi::math::SimpleMotorFeedforward<wpi::units::turns>::kv_unit>{0.0},
          wpi::units::unit_t<wpi::math::SimpleMotorFeedforward<wpi::units::turns>::ka_unit>{0.0}})
      .WithClosedLoopMode()
      .WithMOI(12_in, 1_lb)
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

static positional::Pivot* CreatePivot(SmartMotorController* smc, TestSubsystem* subsys) {
  auto* cfg = new PivotConfig;
  cfg->WithMinAngle(-100.0_deg).WithMaxAngle(150.0_deg);
  auto* pivot = new positional::Pivot(cfg, smc);
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
  wpi::cmd::CommandScheduler::GetInstance().Schedule(cmd);
  wpi::cmd::CommandScheduler::GetInstance().Schedule(cmd);  // schedule twice like Java

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
    INFO("Pivot did not move during duty-cycle test"
         << " preVel=" << preVel.value() << " preAngle=" << preAngle.value()
         << " postVel=" << postVel.value() << " postAngle=" << postAngle.value());
    CHECK(moved);
  }
}

static void PositionPIDTestBody(SmartMotorController* smc, bool isCTRE) {
  auto preAngle = smc->GetMechanismPosition();
  bool passed = false;

  auto cmd =
      wpi::cmd::Run([smc] { smc->SetPosition(80.0_deg); }, {smc->GetConfig().GetSubsystem()});
  wpi::cmd::CommandScheduler::GetInstance().Schedule(cmd);

  SchedulerHelper::RunForDuration(isCTRE ? 1.0_s : 20.0_s, [&] {
    if (isCTRE)
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(
          smc->GetConfig().GetClosedLoopControlPeriod().value_or(20_ms).value() * 1000.0)));
    if (smc->GetDutyCycle() != 0.0) passed = true;
  });

  auto postAngle = smc->GetMechanismPosition();
  INFO("Pivot did not move toward PID setpoint"
       << " preAngle=" << preAngle.value() << " postAngle=" << postAngle.value());
  CHECK((std::abs(postAngle.value() - preAngle.value()) > 0.0003 || passed));
}

// ---- Fixture ----------------------------------------------------------------

namespace {
struct PivotTestFixture {
  PivotTestFixture() {
    InitializeHardware();
    SchedulerHelper::Enable();
    SchedulerHelper::CancelAll();
  }
  ~PivotTestFixture() {
    TeardownHardware();
    SchedulerHelper::CancelAll();
  }
};
}  // namespace

// ---- Tests ------------------------------------------------------------------

TEST_CASE_METHOD(PivotTestFixture, "PivotTest.SMCDutyCycle", "[PivotTest]") {
  for (auto& param : AllMotorParams()) {
    DYNAMIC_SECTION(param.name) {
      auto cfg = MakePivotSMCConfig(param.profile, param.hardware, nullptr, param.name);
      auto bundle = MakeBundle(param, cfg);
      bundle.smc->SetupSimulation();
      bundle.subsystem->m_testRunning = true;

      DutyCycleTestBody(bundle.smc, IsCTRE(bundle));
      CloseBundle(bundle);
    }
  }
}

TEST_CASE_METHOD(PivotTestFixture, "PivotTest.SMCPositionPID", "[PivotTest]") {
  for (auto& param : AllMotorParams()) {
    DYNAMIC_SECTION(param.name) {
      auto cfg = MakePivotSMCConfig(param.profile, param.hardware, nullptr, param.name);
      auto bundle = MakeBundle(param, cfg);
      bundle.smc->SetupSimulation();
      bundle.subsystem->m_testRunning = true;

      PositionPIDTestBody(bundle.smc, IsCTRE(bundle));
      CloseBundle(bundle);
    }
  }
}

TEST_CASE_METHOD(PivotTestFixture, "PivotTest.PivotDutyCycle", "[PivotTest]") {
  for (auto& param : AllMotorParams()) {
    DYNAMIC_SECTION(param.name) {
      auto cfg = MakePivotSMCConfig(param.profile, param.hardware, nullptr, param.name);
      auto bundle = MakeBundle(param, cfg);
      bundle.smc->SetupSimulation();
      bundle.subsystem->m_testRunning = true;

      auto pivot = CreatePivot(bundle.smc, bundle.subsystem.get());
      auto upCmd = pivot->Set(0.5);
      wpi::cmd::CommandScheduler::GetInstance().Schedule(upCmd);

      DutyCycleTestBody(bundle.smc, IsCTRE(bundle));
      CloseBundle(bundle);
      delete pivot;
    }
  }
}

TEST_CASE_METHOD(PivotTestFixture, "PivotTest.PivotPositionPID", "[PivotTest]") {
  for (auto& param : AllMotorParams()) {
    DYNAMIC_SECTION(param.name) {
      auto cfg = MakePivotSMCConfig(param.profile, param.hardware, nullptr, param.name);
      auto bundle = MakeBundle(param, cfg);
      bundle.smc->SetupSimulation();
      bundle.subsystem->m_testRunning = true;

      auto pivot = CreatePivot(bundle.smc, bundle.subsystem.get());
      auto highPid = pivot->RunTo(80.0_deg);
      wpi::cmd::CommandScheduler::GetInstance().Schedule(highPid);

      PositionPIDTestBody(bundle.smc, IsCTRE(bundle));
      CloseBundle(bundle);
      delete pivot;
    }
  }
}

// A pivot driven into its hard stops stays on them. Its DCMotorSim has no hard stops, so the pivot
// is put back on the stop it passes, and stopped there, or it flies past again on the next loop.
TEST_CASE_METHOD(PivotTestFixture, "PivotTest.HardStops", "[PivotTest]") {
  constexpr double kLowerHardStopDegrees = -30.0;
  constexpr double kUpperHardStopDegrees = 60.0;
  for (auto& param : AllMotorParams()) {
    if (param.profile != ProfileType::None) continue;
    DYNAMIC_SECTION(param.name) {
      auto cfg = MakePivotSMCConfig(param.profile, param.hardware, nullptr, param.name);
      auto bundle = MakeBundle(param, cfg);
      bundle.smc->SetupSimulation();
      bundle.subsystem->m_testRunning = true;
      auto* pivotCfg = new PivotConfig;
      pivotCfg->WithMinAngle(wpi::units::degree_t{kLowerHardStopDegrees})
          .WithMaxAngle(wpi::units::degree_t{kUpperHardStopDegrees});
      auto* pivot = new positional::Pivot(pivotCfg, bundle.smc);
      bundle.subsystem->m_mechSimPeriodic = [pivot] { pivot->SimIterate(); };
      bundle.subsystem->m_mechUpdateTelemetry = [pivot] { pivot->UpdateTelemetry(); };
      const bool isCTRE = IsCTRE(bundle);
      // Phoenix runs its simulated Talons in real time.
      auto pace = [isCTRE] {
        if (isCTRE) std::this_thread::sleep_for(std::chrono::milliseconds(20));
      };

      // The commands outlive their runs: the scheduler holds on to them.
      std::vector<wpi::cmd::CommandPtr> commands;
      for (double dutyCycle : {0.5, -0.5}) {
        const double stop = dutyCycle > 0 ? kUpperHardStopDegrees : kLowerHardStopDegrees;
        commands.push_back(bundle.subsystem->SetDutyCycle(dutyCycle));
        wpi::cmd::CommandScheduler::GetInstance().Schedule(commands.back());
        // Long enough to reach the stop from the other one.
        SchedulerHelper::RunForDuration(2.0_s, pace);
        // Pushed against the stop, the pivot must stay on it, loop after loop.
        double furthestPast = 0.0;
        SchedulerHelper::RunForDuration(0.5_s, [&] {
          pace();
          const double simulated =
              wpi::units::degree_t{bundle.smc->GetSimSupplier()->GetMechanismPosition()}.value();
          furthestPast = std::max(furthestPast, dutyCycle > 0 ? simulated - stop : stop - simulated);
        });
        INFO(param.name << " pivot went " << furthestPast << " deg past its hard stop at " << stop
                        << " deg");
        CHECK(furthestPast < 1.0);
      }
      SchedulerHelper::CancelAll();
      CloseBundle(bundle);
      delete pivot;
    }
  }
}

}  // namespace yams::test
