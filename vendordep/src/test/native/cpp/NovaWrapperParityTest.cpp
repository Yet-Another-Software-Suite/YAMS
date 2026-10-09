// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Tests for NovaWrapper behaviour, mirroring the Java reference
// (yams/java/yams/core/motorcontrollers/local/NovaWrapper.java). The Nova's closed loop runs on
// the SystemCore, so these tests step it with IterateClosedLoopController() and SimIterate().

#include <thrifty/canEncoder/CanEncoder.h>
#include <thrifty/nova/Nova.h>
#include <thrifty/nova/NovaConfig.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <any>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <wpi/math/controller/SimpleMotorFeedforward.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/simulation/RoboRioSim.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/time.hpp>
#include <wpi/units/voltage.hpp>

#include "helpers/MockHardware.h"
#include "helpers/MotorControllerFactory.h"
#include "yams/exceptions.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/motorcontrollers/local/NovaWrapper.hpp"

namespace yams::test {

namespace {

using motorcontrollers::local::NovaWrapper;
using MotorMode = SmartMotorControllerConfig::MotorMode;
using SimpleFF = wpi::math::SimpleMotorFeedforward<wpi::units::turns>;
using ConfigException = exceptions::SmartMotorControllerConfigurationException;
using FeedbackSensorType = thrifty::Motor::FeedbackSensorType;

/** Motor rotations per mechanism rotation used by most tests. */
constexpr double kRatio = 12.0;

/**
 * CAN IDs on a bus no other test uses, so a device does not start from the simulated state an
 * earlier test's device with the same ID left behind.
 */
int NextNovaParityCanId() {
  static int next = 0;
  if (next > 62) throw std::runtime_error("Out of NovaParity CAN IDs");
  return next++;
}

std::unique_ptr<thrifty::Nova> MakeNova() {
  return std::make_unique<thrifty::Nova>(1, NextNovaParityCanId(), thrifty::Motor::NEO);
}

std::unique_ptr<thrifty::CanEncoder> MakeCanEncoder() {
  return std::make_unique<thrifty::CanEncoder>(1, NextNovaParityCanId());
}

/** Closed-loop config with a mechanism gearing of @p reduction motor rotations per rotation. */
SmartMotorControllerConfig BaseConfig(double reduction = kRatio) {
  SmartMotorControllerConfig cfg;
  cfg.WithMotorGearing(gearing::MechanismGearing{reduction})
      .WithStatorCurrentLimit(40_A)
      .WithZeroPower(MotorMode::BRAKE)
      .WithClosedLoopMode();
  return cfg;
}

SimpleFF MakeSimpleFF(double kS, double kV, double kA) {
  return SimpleFF{wpi::units::volt_t{kS}, wpi::units::unit_t<SimpleFF::kv_unit>{kV},
                  wpi::units::unit_t<SimpleFF::ka_unit>{kA}};
}

/** Run the SystemCore closed loop and the simulation together for @p steps loops. */
void Step(NovaWrapper& smc, int steps) {
  for (int i = 0; i < steps; i++) {
    smc.IterateClosedLoopController();
    smc.SimIterate();
  }
}

struct NovaParityFixture {
  NovaParityFixture() { InitializeHardware(); }
  ~NovaParityFixture() { TeardownHardware(); }
};

}  // namespace

TEST_CASE_METHOD(NovaParityFixture, "NovaParity.ClosedLoopConvergesInSimulation",
                 "[NovaParity]") {
  auto nova = MakeNova();
  auto cfg = BaseConfig();
  cfg.WithFeedback(8.0, 0.0, 0.0).WithFeedforward(MakeSimpleFF(0.0, 1.0, 0.0));
  NovaWrapper smc{nova.get(), wpi::math::DCMotor::NEO(1), &cfg};
  smc.SetPosition(wpi::units::turn_t{80.0 / 360.0});
  Step(smc, 150);
  INFO("position after 3 s " << smc.GetMechanismPosition().value() * 360.0 << " deg");
  CHECK(smc.GetMechanismPosition().value() == Catch::Approx(80.0 / 360.0).margin(2.0 / 360.0));
}

TEST_CASE_METHOD(NovaParityFixture, "NovaParity.SimulationHoldsCommandedDutyCycle",
                 "[NovaParity]") {
  auto nova = MakeNova();
  auto cfg = BaseConfig();
  cfg.WithOpenLoopMode();
  NovaWrapper smc{nova.get(), wpi::math::DCMotor::NEO(1), &cfg};
  const double supplyVolts = wpi::sim::RoboRioSim::GetVInVoltage().value();
  smc.SetVoltage(6_V);
  const double dutyCycle = smc.GetDutyCycle();
  CHECK(dutyCycle == Catch::Approx(6.0 / supplyVolts));
  // The simulation holds the commanded output between commands, as the Nova does; reading it back
  // from the motor model would feed the motor's back-EMF in as input.
  for (int i = 0; i < 25; i++) smc.SimIterate();
  CHECK(smc.GetDutyCycle() == Catch::Approx(dutyCycle));
  // The Nova cannot apply more than its supply voltage.
  smc.SetVoltage(100_V);
  CHECK(smc.GetDutyCycle() == Catch::Approx(1.0));
  CHECK(smc.GetVoltage().value() <= wpi::sim::RoboRioSim::GetVInVoltage().value() + 1e-9);
  smc.SetDutyCycle(-3.0);
  CHECK(smc.GetDutyCycle() == Catch::Approx(-1.0));
}

TEST_CASE_METHOD(NovaParityFixture, "NovaParity.VendorConfigMustBeNovaConfigBatch",
                 "[NovaParity]") {
  {
    auto nova = MakeNova();
    auto cfg = BaseConfig();
    cfg.WithVendorConfig(std::string{"not a Nova config"});
    CHECK_THROWS_AS((NovaWrapper{nova.get(), wpi::math::DCMotor::NEO(1), &cfg}), ConfigException);
  }
  {
    auto nova = MakeNova();
    auto cfg = BaseConfig();
    cfg.WithVendorConfig(thrifty::NovaConfigBatch{thrifty::NovaConfig::TempThrottleEnable(true)});
    NovaWrapper smc{nova.get(), wpi::math::DCMotor::NEO(1), &cfg};
    CHECK(smc.GetMotorControllerConfig() != nullptr);
  }
}

TEST_CASE_METHOD(NovaParityFixture, "NovaParity.UnsupportedOptionsThrow", "[NovaParity]") {
  {
    INFO("the internal encoder cannot be inverted");
    auto nova = MakeNova();
    auto cfg = BaseConfig();
    cfg.WithEncoderInverted(true);
    CHECK_THROWS_AS((NovaWrapper{nova.get(), wpi::math::DCMotor::NEO(1), &cfg}),
                    std::invalid_argument);
  }
  {
    INFO("the Nova has one ramp rate");
    auto nova = MakeNova();
    auto cfg = BaseConfig();
    cfg.WithOpenLoopRampRate(0.1_s).WithClosedLoopRampRate(0.2_s);
    CHECK_THROWS_AS((NovaWrapper{nova.get(), wpi::math::DCMotor::NEO(1), &cfg}), ConfigException);
  }
  {
    INFO("matching open and closed loop ramp rates are accepted");
    auto nova = MakeNova();
    auto cfg = BaseConfig();
    cfg.WithOpenLoopRampRate(0.1_s).WithClosedLoopRampRate(0.1_s);
    CHECK_NOTHROW(NovaWrapper{nova.get(), wpi::math::DCMotor::NEO(1), &cfg});
  }
  {
    INFO("a zero offset needs an external encoder");
    auto nova = MakeNova();
    auto cfg = BaseConfig();
    cfg.WithExternalEncoderZeroOffset(wpi::units::turn_t{0.1});
    CHECK_THROWS_AS((NovaWrapper{nova.get(), wpi::math::DCMotor::NEO(1), &cfg}), ConfigException);
  }
  {
    INFO("a quadrature encoder has no zero offset");
    auto nova = MakeNova();
    auto cfg = BaseConfig();
    cfg.WithExternalEncoder(FeedbackSensorType::QUAD)
        .WithExternalEncoderZeroOffset(wpi::units::turn_t{0.1});
    CHECK_THROWS_AS((NovaWrapper{nova.get(), wpi::math::DCMotor::NEO(1), &cfg}), ConfigException);
  }
  {
    INFO("only data port and CAN encoders are external encoders");
    auto nova = MakeNova();
    auto cfg = BaseConfig();
    cfg.WithExternalEncoder(FeedbackSensorType::INTERNAL);
    CHECK_THROWS_AS((NovaWrapper{nova.get(), wpi::math::DCMotor::NEO(1), &cfg}),
                    std::invalid_argument);
  }
  {
    INFO("vendor control requests are not supported");
    auto nova = MakeNova();
    auto cfg = BaseConfig();
    cfg.WithVendorControlRequest(std::any{1});
    CHECK_THROWS_AS((NovaWrapper{nova.get(), wpi::math::DCMotor::NEO(1), &cfg}), ConfigException);
  }
}

TEST_CASE_METHOD(NovaParityFixture, "NovaParity.AbsoluteEncodersReportBelowDiscontinuityPoint",
                 "[NovaParity]") {
  for (const bool canEncoderUsed : {false, true}) {
    DYNAMIC_SECTION((canEncoderUsed ? "CAN encoder" : "data port absolute encoder")) {
      auto nova = MakeNova();
      auto canEncoder = MakeCanEncoder();
      auto cfg = BaseConfig();
      cfg.WithExternalEncoder(canEncoderUsed ? std::any{canEncoder.get()}
                                             : std::any{FeedbackSensorType::ABS})
          .WithUseExternalFeedbackEncoder(true)
          .WithExternalEncoderDiscontinuityPoint(wpi::units::turn_t{0.5});
      NovaWrapper smc{nova.get(), wpi::math::DCMotor::NEO(1), &cfg};
      smc.SetEncoderPosition(wpi::units::turn_t{0.7});
      // An absolute encoder reports angles within one rotation: [-0.5, 0.5) here.
      CHECK(smc.GetMechanismPosition().value() == Catch::Approx(-0.3).margin(1e-6));
      CHECK(smc.GetExternalEncoderMechanismPosition()->value() == Catch::Approx(-0.3).margin(1e-6));
      // The motor's encoder is not wrapped.
      CHECK(smc.GetRelativeMechanismPosition().value() == Catch::Approx(0.7).margin(1e-6));
    }
  }
}

TEST_CASE_METHOD(NovaParityFixture, "NovaParity.ExternalEncoderGearing", "[NovaParity]") {
  auto nova = MakeNova();
  auto cfg = BaseConfig();
  cfg.WithExternalEncoder(FeedbackSensorType::QUAD)
      .WithUseExternalFeedbackEncoder(true)
      .WithExternalEncoderGearing(2.0);
  NovaWrapper smc{nova.get(), wpi::math::DCMotor::NEO(1), &cfg};
  smc.SetEncoderPosition(wpi::units::turn_t{3.0});
  // A quadrature encoder is not wrapped, and reads in mechanism rotations through its gearing.
  CHECK(smc.GetMechanismPosition().value() == Catch::Approx(3.0).margin(1e-6));
  CHECK(smc.GetRotorPosition().value() == Catch::Approx(3.0 * kRatio).margin(1e-6));
}

TEST_CASE_METHOD(NovaParityFixture, "NovaParity.StartingPosition", "[NovaParity]") {
  auto nova = MakeNova();
  auto cfg = BaseConfig();
  cfg.WithStartingPosition(wpi::units::degree_t{45.0});
  NovaWrapper smc{nova.get(), wpi::math::DCMotor::NEO(1), &cfg};
  CHECK(smc.GetMechanismPosition().value() == Catch::Approx(0.125).margin(1e-6));
  CHECK(smc.GetRotorPosition().value() == Catch::Approx(0.125 * kRatio).margin(1e-6));
}

TEST_CASE_METHOD(NovaParityFixture, "NovaParity.FollowersMustBeNovasAndAreCleared",
                 "[NovaParity]") {
  {
    auto nova = MakeNova();
    auto cfg = BaseConfig();
    cfg.WithFollowers({{std::any{1}, false}});
    CHECK_THROWS_AS((NovaWrapper{nova.get(), wpi::math::DCMotor::NEO(1), &cfg}),
                    std::invalid_argument);
  }
  {
    auto nova = MakeNova();
    auto follower = MakeNova();
    auto cfg = BaseConfig();
    cfg.WithFollowers({{std::any{follower.get()}, true}});
    NovaWrapper smc{nova.get(), wpi::math::DCMotor::NEO(1), &cfg};
    // The followers keep following; applying the config again must not reconfigure them.
    CHECK(cfg.GetFollowers().empty());
  }
}

TEST_CASE_METHOD(NovaParityFixture, "NovaParity.LiveSettersUpdateConfig", "[NovaParity]") {
  auto nova = MakeNova();
  auto cfg = BaseConfig();
  cfg.WithFeedback(1.0, 0.0, 0.0).WithFeedforward(MakeSimpleFF(0.1, 1.0, 0.0));
  NovaWrapper smc{nova.get(), wpi::math::DCMotor::NEO(1), &cfg};
  smc.SetKp(2.5);
  smc.SetKd(0.25);
  smc.SetKs(0.3);
  smc.SetKv(1.5);
  const auto gains = cfg.GetSlotGains(SmartMotorControllerConfig::ClosedLoopControllerSlot::SLOT_0);
  CHECK(gains.kP == Catch::Approx(2.5));
  CHECK(gains.kD == Catch::Approx(0.25));
  const auto ff =
      cfg.GetSimpleFeedforward(SmartMotorControllerConfig::ClosedLoopControllerSlot::SLOT_0);
  REQUIRE(ff.has_value());
  CHECK(ff->GetKs().value() == Catch::Approx(0.3));
  CHECK(ff->GetKv().value() == Catch::Approx(1.5));
  CHECK_THROWS_AS(smc.SetEncoderInverted(true), std::runtime_error);
}

}  // namespace yams::test
