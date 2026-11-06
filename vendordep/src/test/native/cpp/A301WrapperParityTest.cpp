// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Tests for A301Wrapper behaviour, mirroring the Java reference
// (yams/java/yams/core/motorcontrollers/local/A301Wrapper.java). The closed loop runs on the
// SystemCore and REVLib does not simulate the A301, so these tests step the wrapper's DCMotorSim
// with SimIterate().

#include <first/A301.h>

#include <any>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <wpi/hardware/bus/CANPort.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/simulation/RoboRioSim.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/current.hpp>
#include <wpi/units/time.hpp>
#include <wpi/units/voltage.hpp>

#include "helpers/MockHardware.h"
#include "yams/exceptions.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/motorcontrollers/local/A301Wrapper.hpp"

namespace yams::test {

namespace {

using motorcontrollers::SmartMotorControllerConfig;
using motorcontrollers::local::A301Wrapper;
using ConfigException = exceptions::SmartMotorControllerConfigurationException;

/** Motor rotations per mechanism rotation: the A301's 64:1 gearbox. */
constexpr double kRatio = 64.0;

/** CAN IDs on a bus no other test uses. */
int NextA301CanId() {
  static int next = 1;
  if (next > 62) throw std::runtime_error("Out of A301Parity CAN IDs");
  return next++;
}

std::unique_ptr<first::a301::A301> MakeA301() {
  return std::make_unique<first::a301::A301>(wpi::CANPort::CAN_S2, NextA301CanId());
}

/** Closed-loop config with no current limit, which the A301 does not have. */
SmartMotorControllerConfig BaseConfig() {
  SmartMotorControllerConfig cfg;
  cfg.WithMotorGearing(gearing::MechanismGearing{kRatio})
      .WithFeedback(10.0, 0.0, 0.0)
      .WithClosedLoopMode();
  return cfg;
}

struct A301ParityFixture {
  A301ParityFixture() { InitializeHardware(); }
  ~A301ParityFixture() { TeardownHardware(); }
};

}  // namespace

TEST_CASE_METHOD(A301ParityFixture, "A301Parity.RejectsWhatTheA301DoesNotHave", "[A301Parity]") {
  {
    INFO("current limits");
    auto a301 = MakeA301();
    auto cfg = BaseConfig();
    cfg.WithStatorCurrentLimit(40_A);
    CHECK_THROWS_AS((A301Wrapper{a301.get(), &cfg}), ConfigException);
  }
  {
    INFO("ramp rates");
    auto a301 = MakeA301();
    auto cfg = BaseConfig();
    cfg.WithOpenLoopRampRate(0.1_s);
    CHECK_THROWS_AS((A301Wrapper{a301.get(), &cfg}), ConfigException);
  }
  {
    INFO("voltage compensation");
    auto a301 = MakeA301();
    auto cfg = BaseConfig();
    cfg.WithVoltageCompensation(12_V);
    CHECK_THROWS_AS((A301Wrapper{a301.get(), &cfg}), ConfigException);
  }
  {
    INFO("vendor config");
    auto a301 = MakeA301();
    auto cfg = BaseConfig();
    cfg.WithVendorConfig(std::string{"not an A301 config"});
    CHECK_THROWS_AS((A301Wrapper{a301.get(), &cfg}), ConfigException);
  }
  {
    INFO("only the A301's own absolute encoder is an external encoder");
    auto a301 = MakeA301();
    auto other = MakeA301();
    auto cfg = BaseConfig();
    cfg.WithExternalEncoder(other.get()).WithUseExternalFeedbackEncoder(true);
    CHECK_THROWS_AS((A301Wrapper{a301.get(), &cfg}), std::invalid_argument);
  }
}

TEST_CASE_METHOD(A301ParityFixture, "A301Parity.ClosedLoopConvergesInSimulation", "[A301Parity]") {
  auto a301 = MakeA301();
  auto cfg = BaseConfig();
  A301Wrapper smc{a301.get(), &cfg};
  smc.SetPosition(wpi::units::turn_t{0.25});
  // Step the SystemCore's closed loop and the simulation together, 3 s in 20 ms steps.
  for (int i = 0; i < 150; i++) {
    smc.IterateClosedLoopController();
    smc.SimIterate();
  }
  INFO("position after 3 s " << smc.GetMechanismPosition().value() << " rotations");
  CHECK(smc.GetMechanismPosition().value() == Catch::Approx(0.25).margin(0.02));
}

TEST_CASE_METHOD(A301ParityFixture, "A301Parity.AbsoluteEncoderReadsInTheWrappingRange",
                 "[A301Parity]") {
  {
    INFO("[0, 1) without wrapping");
    auto a301 = MakeA301();
    auto cfg = BaseConfig();
    cfg.WithExternalEncoder(a301.get())
        .WithUseExternalFeedbackEncoder(true)
        .WithStartingPosition(wpi::units::turn_t{-0.2});
    A301Wrapper smc{a301.get(), &cfg};
    CHECK(smc.GetMechanismPosition().value() == Catch::Approx(0.8).margin(1e-6));
  }
  {
    INFO("[-0.5, 0.5) with wrapping");
    auto a301 = MakeA301();
    auto cfg = BaseConfig();
    cfg.WithExternalEncoder(a301.get())
        .WithUseExternalFeedbackEncoder(true)
        .WithWrapping(wpi::units::turn_t{-0.5}, wpi::units::turn_t{0.5})
        .WithStartingPosition(wpi::units::turn_t{-0.2});
    A301Wrapper smc{a301.get(), &cfg};
    CHECK(smc.GetMechanismPosition().value() == Catch::Approx(-0.2).margin(1e-6));
  }
}

TEST_CASE_METHOD(A301ParityFixture, "A301Parity.MotorIsTheA301s", "[A301Parity]") {
  // The A301's motor: 13,500 RPM free, and 5.27 Nm at peak power through 135:1.
  const auto motor = motorcontrollers::YMotors::A301();
  CHECK(wpi::units::revolutions_per_minute_t{motor.freeSpeed}.value() == Catch::Approx(13500.0));
  CHECK(motor.stallTorque.value() * 135.0 / 2.0 == Catch::Approx(5.27));

  auto a301 = MakeA301();
  auto cfg = BaseConfig();
  A301Wrapper smc{a301.get(), &cfg};
  CHECK(smc.GetDCMotor().freeSpeed.value() == Catch::Approx(motor.freeSpeed.value()));
}

TEST_CASE_METHOD(A301ParityFixture, "A301Parity.MotorCountModelsEveryA301OnTheMechanism",
                 "[A301Parity]") {
  // Three A301s on one mechanism have three times the torque, at the same free speed.
  const auto one = motorcontrollers::YMotors::A301();
  auto a301 = MakeA301();
  auto cfg = BaseConfig();
  A301Wrapper smc{a301.get(), 3, &cfg};
  CHECK(smc.GetDCMotor().stallTorque.value() == Catch::Approx(3.0 * one.stallTorque.value()));
  CHECK(smc.GetDCMotor().freeSpeed.value() == Catch::Approx(one.freeSpeed.value()));

  auto other = MakeA301();
  auto otherCfg = BaseConfig();
  CHECK_THROWS_AS((A301Wrapper{other.get(), 0, &otherCfg}), std::invalid_argument);
}

}  // namespace yams::test
