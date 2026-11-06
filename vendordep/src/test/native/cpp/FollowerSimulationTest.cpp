// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// How followers are simulated, mirroring the Java FollowerSimulationTest: an A301 followed by other
// A301Wrappers has every A301 in its motor model, and a follower reads its simulation; a motor
// controller's loosely coupled followers are in its mechanism's motor model, GetMechanismDCMotor().

#include <first/A301.h>

#include <any>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <memory>
#include <optional>
#include <stdexcept>
#include <wpi/hardware/bus/CANPort.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/units/current.hpp>
#include <wpi/units/voltage.hpp>

#include "helpers/MockHardware.h"
#include "yams/exceptions.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/motorcontrollers/YMotors.hpp"
#include "yams/motorcontrollers/local/A301Wrapper.hpp"
#include "yams/motorcontrollers/simulation/BatterySim.hpp"

namespace yams::test {

namespace {

using motorcontrollers::SmartMotorController;
using motorcontrollers::SmartMotorControllerConfig;
using motorcontrollers::local::A301Wrapper;
using motorcontrollers::simulation::BatterySim;

/** CAN IDs on a bus no other test uses. */
int NextCanId() {
  static int next = 1;
  if (next > 62) throw std::runtime_error("Out of FollowerSimulation CAN IDs");
  return next++;
}

std::unique_ptr<first::a301::A301> MakeA301() {
  return std::make_unique<first::a301::A301>(wpi::CANPort::CAN_S3, NextCanId());
}

/** An open loop config through the A301's 64:1 gearbox, or another reduction. */
SmartMotorControllerConfig OpenLoopConfig(double ratio = 64.0) {
  SmartMotorControllerConfig cfg;
  cfg.WithMotorGearing(gearing::MechanismGearing{ratio}).WithOpenLoopMode();
  return cfg;
}

/** A motor controller's battery key is its SmartMotorController address. */
const void* Key(SmartMotorController& smc) { return static_cast<const void*>(&smc); }

/** Whether the battery counts the motor controller's current. */
bool BatteryCounts(SmartMotorController& smc) {
  BatterySim::CalculateVoltage(Key(smc), wpi::units::ampere_t{50});
  const bool counted = BatterySim::HasCurrent(Key(smc));
  BatterySim::RemoveCurrent(Key(smc));
  return counted;
}

struct FollowerSimulationFixture {
  FollowerSimulationFixture() {
    InitializeHardware();
    BatterySim::Reset();
  }
  ~FollowerSimulationFixture() {
    BatterySim::Reset();
    TeardownHardware();
  }
};

}  // namespace

TEST_CASE_METHOD(FollowerSimulationFixture,
                 "FollowerSimulation.A301FollowersAreInTheLeadersMotorAndSimulation",
                 "[FollowerSimulation]") {
  // Two A301s on one mechanism, the follower facing the other way.
  auto followerA301 = MakeA301();
  auto followerCfg = OpenLoopConfig();
  A301Wrapper follower{followerA301.get(), &followerCfg};
  auto leaderA301 = MakeA301();
  auto leaderCfg = OpenLoopConfig();
  leaderCfg.WithFollowers({{std::any{&follower}, true}});
  std::optional<A301Wrapper> leader;
  leader.emplace(leaderA301.get(), &leaderCfg);

  const auto one = motorcontrollers::YMotors::A301();
  CHECK(leader->GetDCMotor().stallTorque.value() == Catch::Approx(2.0 * one.stallTorque.value()));
  CHECK(leader->GetDCMotor().freeSpeed.value() == Catch::Approx(one.freeSpeed.value()));

  // The follower is sent the leader's voltage, inverted, and reads the leader's simulation.
  leader->SetVoltage(wpi::units::volt_t{6});
  CHECK(follower.GetVoltage().value() == Catch::Approx(-6.0));
  for (int i = 0; i < 25; i++) leader->SimIterate();
  CHECK(leader->GetMechanismPosition().value() > 0.001);
  CHECK(follower.GetMechanismPosition().value() ==
        Catch::Approx(leader->GetMechanismPosition().value()));
  // Each A301 draws half of the current, and the battery counts it once, in the leader's.
  CHECK(follower.GetStatorCurrent().value() ==
        Catch::Approx(leader->GetStatorCurrent().value() / 2.0));
  CHECK_FALSE(BatteryCounts(follower));

  // Without its leader, the follower simulates itself again.
  leader.reset();
  CHECK(BatteryCounts(follower));
}

TEST_CASE_METHOD(FollowerSimulationFixture, "FollowerSimulation.OnlyAnA301WrapperCanFollowAnA301",
                 "[FollowerSimulation]") {
  auto leaderA301 = MakeA301();
  auto followerA301 = MakeA301();
  auto cfg = OpenLoopConfig();
  cfg.WithFollowers({{std::any{followerA301.get()}, false}});
  CHECK_THROWS_AS((A301Wrapper{leaderA301.get(), &cfg}),
                  exceptions::SmartMotorControllerConfigurationException);
}

TEST_CASE_METHOD(FollowerSimulationFixture,
                 "FollowerSimulation.MechanismMotorHasTheLooseFollowers", "[FollowerSimulation]") {
  // The follower is geared 32:1 and the leader 64:1, so the follower adds half an A301 at the
  // leader's rotor.
  auto followerA301 = MakeA301();
  auto followerCfg = OpenLoopConfig(32.0);
  A301Wrapper follower{followerA301.get(), &followerCfg};
  auto leaderA301 = MakeA301();
  auto leaderCfg = OpenLoopConfig(64.0);
  leaderCfg.WithLooselyCoupledFollowers({&follower});
  A301Wrapper leader{leaderA301.get(), &leaderCfg};

  const auto one = motorcontrollers::YMotors::A301();
  const auto mechanism = leader.GetMechanismDCMotor();
  CHECK(mechanism.stallTorque.value() == Catch::Approx(1.5 * one.stallTorque.value()));
  CHECK(mechanism.stallCurrent.value() == Catch::Approx(2.0 * one.stallCurrent.value()));
  CHECK(mechanism.freeSpeed.value() == Catch::Approx(one.freeSpeed.value()));
  // The leader's own motor model is still its own.
  CHECK(leader.GetDCMotor().stallTorque.value() == Catch::Approx(one.stallTorque.value()));

  // Without a mechanism, such as two flywheels on their own shafts, the follower draws its own
  // current.
  CHECK(BatteryCounts(follower));
}

}  // namespace yams::test
