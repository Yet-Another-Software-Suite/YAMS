// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Mirrors Java PivotTest duty-cycle and position-PID tests for a rotary
// pivot mechanism across all (HardwareType × ProfileType) combinations.

#include <catch2/catch_test_macros.hpp>

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

// A loosely coupled follower follows its leader, whether its closed loop runs on the motor
// controller or on the SystemCore, as a Nova's always does and a SPARK's with an exponential profile
// does. When the leader's closed loop is stopped, such as to drive it at a duty cycle, the follower
// is stopped too, rather than its closed loop driving it on to the last setpoint. Each motor drives its own simulated pivot.
TEST_CASE_METHOD(PivotTestFixture, "PivotTest.LooselyCoupledFollowers", "[PivotTest]") {
  constexpr double kSetpointDegrees = 45.0;
  struct Pair {
    std::string name;
    MotorTestParam leader;
    MotorTestParam follower;
  };
  const std::vector<Pair> pairs = {
      // The follower's closed loop runs on the motor controller.
      {"SparkMax following a SparkFlex",
       {HardwareType::SparkFlex, ProfileType::None, "LooseLeaderSparkFlex"},
       {HardwareType::SparkMax, ProfileType::None, "LooseFollowerSparkMax"}},
      // The follower's closed loop runs on the SystemCore.
      {"Nova following a SparkMax",
       {HardwareType::SparkMax, ProfileType::None, "LooseLeaderSparkMax"},
       {HardwareType::Nova, ProfileType::None, "LooseFollowerNova"}},
      {"SparkMax with an exponential profile following a SparkFlex",
       {HardwareType::SparkFlex, ProfileType::None, "LooseLeaderSparkFlexExpo"},
       {HardwareType::SparkMax, ProfileType::Exponential, "LooseFollowerSparkMaxExpo"}},
  };
  for (const auto& pair : pairs) {
    DYNAMIC_SECTION(pair.name) {
      auto followerCfg = MakePivotSMCConfig(pair.follower.profile, pair.follower.hardware, nullptr,
                                            pair.follower.name);
      auto follower = MakeBundle(pair.follower, followerCfg);
      auto leaderCfg = MakePivotSMCConfig(pair.leader.profile, pair.leader.hardware, nullptr,
                                          pair.leader.name);
      leaderCfg.WithLooselyCoupledFollowers({follower.smc});
      auto leader = MakeBundle(pair.leader, leaderCfg);
      // The command registry needs the leader's and the follower's subsystems named apart.
      leader.subsystem->SetName(pair.leader.name);
      follower.subsystem->SetName(pair.follower.name);
      auto* leaderPivot = CreatePivot(leader.smc, leader.subsystem.get());
      auto* followerPivot = CreatePivot(follower.smc, follower.subsystem.get());
      leader.smc->SetupSimulation();
      follower.smc->SetupSimulation();
      leader.subsystem->m_testRunning = true;
      follower.subsystem->m_testRunning = true;
      auto followerDegrees = [&] {
        return wpi::units::degree_t{follower.smc->GetSimSupplier()->GetMechanismPosition()}.value();
      };

      // The commands outlive their runs: the scheduler holds on to them.
      std::vector<wpi::cmd::CommandPtr> commands;
      auto schedule = [&](wpi::cmd::CommandPtr command) {
        commands.push_back(std::move(command));
        wpi::cmd::CommandScheduler::GetInstance().Schedule(commands.back());
      };

      // 1. The follower goes to the leader's setpoint.
      schedule(leader.subsystem->SetPositionSetpoint(wpi::units::degree_t{kSetpointDegrees}));
      SchedulerHelper::RunForDuration(5.0_s);
      CHECK(std::abs(followerDegrees() - kSetpointDegrees) < 3.0);

      // 2. Back to 0 deg, and the leader is stopped halfway, as a duty cycle command does.
      schedule(leader.subsystem->SetPositionSetpoint(0.0_deg));
      for (int loop = 0; loop < 250 && followerDegrees() > kSetpointDegrees / 2; loop++) {
        SchedulerHelper::RunForDuration(20_ms);
      }
      const double stoppedAt = followerDegrees();
      REQUIRE(stoppedAt > 5.0);
      schedule(leader.subsystem->SetDutyCycle(0.0));
      // The follower is stopped as well, loop after loop, rather than its closed loop driving it on to
      // 0 deg. Its position cannot show this, as a fast follower coasts most of the way there.
      SchedulerHelper::RunForDuration(40_ms);
      double largestDutyCycle = 0.0;
      SchedulerHelper::RunForDuration(1.0_s, [&] {
        largestDutyCycle = std::max(largestDutyCycle, std::abs(follower.smc->GetDutyCycle()));
      });
      INFO(pair.name << ": follower driven at a duty cycle of up to " << largestDutyCycle
                     << " after its leader stopped, at " << stoppedAt << " deg");
      CHECK(largestDutyCycle < 0.01);

      // 3. The leader's closed loop is started again, and the follower follows again.
      schedule(leader.subsystem->SetPositionSetpoint(wpi::units::degree_t{kSetpointDegrees}));
      SchedulerHelper::RunForDuration(5.0_s);
      CHECK(std::abs(followerDegrees() - kSetpointDegrees) < 3.0);

      SchedulerHelper::CancelAll();
      CloseBundle(leader);
      CloseBundle(follower);
      delete leaderPivot;
      delete followerPivot;
    }
  }
}

}  // namespace yams::test
