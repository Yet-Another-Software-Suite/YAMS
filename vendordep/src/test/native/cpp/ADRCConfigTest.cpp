// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// ADRCConfig on its own and in a SmartMotorControllerConfig: its values and the ones it refuses, deriving b0, building
// a controller, the slots and copies. Mirrors the Java ADRCConfigTest. The ADRC was written by FRC Team 8331.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <memory>
#include <numbers>
#include <stdexcept>

#include <wpi/math/controller/SimpleMotorFeedforward.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/moment_of_inertia.hpp>
#include <wpi/units/time.hpp>
#include <wpi/units/voltage.hpp>

#include "yams/exceptions.hpp"
#include "yams/math/ADRCConfig.hpp"
#include "yams/math/ADRCController.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"

namespace yams::test {

using namespace math;
using motorcontrollers::SmartMotorControllerConfig;
using Slot = SmartMotorControllerConfig::ClosedLoopControllerSlot;

namespace {
/** A motor config with a 12:1 reduction. */
SmartMotorControllerConfig SmcConfig() {
  SmartMotorControllerConfig config;
  config.WithMotorGearing(12.0);
  return config;
}

std::shared_ptr<ADRCConfig> Shared(const ADRCConfig& config) { return std::make_shared<ADRCConfig>(config); }
}  // namespace

TEST_CASE("ADRCConfig.UnsetValuesAreEmptyAndBuildWithTheirDefaults", "[ADRCConfig]") {
  ADRCConfig config;
  CHECK_FALSE(config.GetSystemOrder());
  CHECK_FALSE(config.GetSettlingTime());
  CHECK_FALSE(config.GetBandwidthScale());
  CHECK_FALSE(config.GetB0());
  CHECK_FALSE(config.GetInputDelay());
  CHECK_FALSE(config.IsCascaded());
  CHECK_FALSE(config.GetTdMethod());
  CHECK(config.GetTdParams().empty());

  config.WithSystemOrder(ADRCConfig::SystemOrder::FIRST).WithB0(10);
  auto adrc = ADRCConfig::BuildController(Shared(config), wpi::math::DCMotor::NEO(1), SmcConfig(), 20_ms);
  CHECK(adrc.GetSettlingTime() == ADRCConfig::kDefaultSettlingTime);
  CHECK(adrc.GetBandwidthScale() == ADRCConfig::kDefaultBandwidthScale);
}

TEST_CASE("ADRCConfig.KeepsEveryValueItIsGiven", "[ADRCConfig]") {
  ADRCConfig config;
  config.WithSystemOrder(ADRCConfig::SystemOrder::SECOND)
      .WithSettlingTime(0.6_s)
      .WithBandwidthScale(1.5)
      .WithB0(3)
      .WithInputDelay(40_ms)
      .WithCascaded(true)
      .WithTrackingDifferentiator(TrackingDifferentiator::Method::EULER, {0.5});
  CHECK(ADRCConfig::GetOrder(*config.GetSystemOrder()) == 2);
  CHECK(config.GetSettlingTime()->value() == 0.6);
  CHECK(*config.GetBandwidthScale() == 1.5);
  CHECK(*config.GetB0() == 3);
  CHECK(config.GetInputDelay()->value() == Catch::Approx(0.04));
  CHECK(config.IsCascaded());
  CHECK(*config.GetTdMethod() == TrackingDifferentiator::Method::EULER);
  CHECK(config.GetTdParams() == std::vector<double>{0.5});
  config.WithoutTrackingDifferentiator();
  CHECK_FALSE(config.GetTdMethod());
  CHECK(config.GetTdParams().empty());
}

TEST_CASE("ADRCConfig.RefusesValuesTheControllerCannotUse", "[ADRCConfig]") {
  ADRCConfig config;
  config.WithSystemOrder(ADRCConfig::SystemOrder::FIRST).WithSettlingTime(0.3_s);
  CHECK_THROWS_AS(config.WithSettlingTime(-1_s), std::invalid_argument);
  CHECK_THROWS_AS(config.WithSettlingTime(0_s), std::invalid_argument);
  CHECK_THROWS_AS(config.WithBandwidthScale(0), std::invalid_argument);
  CHECK_THROWS_AS(config.WithB0(0), std::invalid_argument);
  CHECK_THROWS_AS(config.WithB0(std::nan("")), std::invalid_argument);
  CHECK_THROWS_AS(config.WithInputDelay(-1_s), std::invalid_argument);
  // A refused value leaves the config as it was.
  CHECK(config.GetSettlingTime()->value() == 0.3);
  CHECK_FALSE(config.GetB0());
}

TEST_CASE("ADRCConfig.DerivesB0FromTheMotorGearingAndInertia", "[ADRCConfig]") {
  const auto motor = wpi::math::DCMotor::NEO(1);
  ADRCConfig config;
  config.WithSystemOrder(ADRCConfig::SystemOrder::SECOND);
  auto smc = SmcConfig();
  // The placeholder inertia is never used.
  CHECK_THROWS_AS(config.ResolveB0(motor, smc), exceptions::SmartMotorControllerConfigurationException);
  smc.WithMOI(wpi::units::kilogram_square_meter_t{0.1});
  const double expected = 12 * motor.Kt.value() / (2 * std::numbers::pi * motor.R.value() * 0.1);
  CHECK(config.ResolveB0(motor, smc) == Catch::Approx(expected));
  // A b0 that is set wins.
  config.WithB0(7.0);
  CHECK(config.ResolveB0(motor, smc) == 7.0);
}

TEST_CASE("ADRCConfig.AnElevatorIsFirstOrderWithItsSpeedPerVoltInMetersAsB0", "[ADRCConfig]") {
  const auto motor = wpi::math::DCMotor::NEO(1);
  STATIC_CHECK(ADRCConfig::IsChosenWhenBuilt(ADRCConfig::SystemOrder::ELEVATOR));
  STATIC_CHECK(ADRCConfig::RequiresLinearClosedLoop(ADRCConfig::SystemOrder::ELEVATOR));
  STATIC_CHECK_FALSE(ADRCConfig::IsChosenWhenBuilt(ADRCConfig::SystemOrder::FIRST));
  ADRCConfig config;
  config.WithSystemOrder(ADRCConfig::SystemOrder::ELEVATOR);
  // An elevator's closed loop must be linear.
  auto rotations = SmcConfig();
  rotations.WithMechanismCircumference(0.15_m);
  CHECK_THROWS_AS(config.ResolveB0(motor, rotations), exceptions::SmartMotorControllerConfigurationException);
  // Without an inertia, the motor's free speed per volt through a 12:1 reduction and a 0.15 m circumference.
  auto linear = SmcConfig();
  linear.WithMechanismCircumference(0.15_m).WithLinearClosedLoopController(true);
  CHECK(config.ResolveB0(motor, linear) == Catch::Approx(motor.Kv.value() / (2 * std::numbers::pi * 12) * 0.15));
  // A b0 that is set wins, and still needs a linear closed loop.
  config.WithB0(3);
  CHECK(config.ResolveB0(motor, linear) == 3);
  CHECK_THROWS_AS(config.ResolveB0(motor, SmcConfig()), exceptions::SmartMotorControllerConfigurationException);
}

// A position loop chosen when built: a 60:1 NEO pivot's back-EMF time constant, 6 ms, is under two 20 ms periods, so
// it is first order with its speed per volt; at 12:1 it is 160 ms, so it is second order with its acceleration per volt.
TEST_CASE("ADRCConfig.APositionLoopIsChosenFromTheBackEmfTimeConstant", "[ADRCConfig]") {
  const auto motor = wpi::math::DCMotor::NEO(1);
  STATIC_CHECK(ADRCConfig::SystemOrder::ARM == ADRCConfig::SystemOrder::POSITION);
  STATIC_CHECK(ADRCConfig::SystemOrder::PIVOT == ADRCConfig::SystemOrder::POSITION);
  STATIC_CHECK(ADRCConfig::GetOrder(ADRCConfig::SystemOrder::POSITION) == 0);
  auto geared = SmcConfig();
  geared.WithMotorGearing(60.0).WithMOI(wpi::units::kilogram_square_meter_t{0.1});
  ADRCConfig config;
  config.WithSystemOrder(ADRCConfig::SystemOrder::POSITION);
  auto first = ADRCConfig::BuildController(Shared(config), motor, geared, 20_ms);
  CHECK(first.GetSystemOrder() == 1);
  CHECK(first.GetB0() == Catch::Approx(motor.Kv.value() / (2 * std::numbers::pi * 60)));
  auto light = SmcConfig();
  light.WithMOI(wpi::units::kilogram_square_meter_t{0.1});
  auto second = ADRCConfig::BuildController(Shared(config), motor, light, 20_ms);
  CHECK(second.GetSystemOrder() == 2);
  CHECK(second.GetB0() == Catch::Approx(12 * motor.Kt.value() / (2 * std::numbers::pi * motor.R.value() * 0.1)));
  // It needs an inertia to choose, and is only chosen when built from a motor config.
  CHECK_THROWS_AS(ADRCConfig::BuildController(Shared(config), motor, SmcConfig(), 20_ms),
                  exceptions::SmartMotorControllerConfigurationException);
  CHECK_THROWS_AS(ADRCController{Shared(config)}, std::logic_error);
}

TEST_CASE("ADRCConfig.ALinearClosedLoopDerivesB0InMeters", "[ADRCConfig]") {
  const auto motor = wpi::math::DCMotor::NEO(1);
  ADRCConfig config;
  config.WithSystemOrder(ADRCConfig::SystemOrder::SECOND);
  auto smc = SmcConfig();
  smc.WithMOI(wpi::units::kilogram_square_meter_t{0.1}).WithLinearClosedLoopController(true);
  // A linear closed loop needs a circumference: without one, b0 stays in rotations.
  const double perRotation = 12 * motor.Kt.value() / (2 * std::numbers::pi * motor.R.value() * 0.1);
  CHECK(config.ResolveB0(motor, smc) == Catch::Approx(perRotation));
  smc.WithMechanismCircumference(0.5_m);
  const double expected = 12 * motor.Kt.value() / (2 * std::numbers::pi * motor.R.value() * 0.1) * 0.5;
  CHECK(config.ResolveB0(motor, smc) == Catch::Approx(expected));
}

TEST_CASE("ADRCConfig.BuildingNeedsASystemOrder", "[ADRCConfig]") {
  ADRCConfig config;
  config.WithB0(10);
  CHECK_THROWS_AS(ADRCConfig::BuildController(Shared(config), wpi::math::DCMotor::NEO(1), SmcConfig(), 20_ms),
                  exceptions::SmartMotorControllerConfigurationException);
}

TEST_CASE("ADRCConfig.AnADRCReplacesTheSlotsPIDAndFeedforward", "[ADRCConfig]") {
  auto smc = SmcConfig();
  smc.WithFeedback(1.0, 0.0, 0.0)
      .WithFeedforward(wpi::math::SimpleMotorFeedforward<wpi::units::turns>{
          0.1_V, wpi::units::unit_t<wpi::math::SimpleMotorFeedforward<wpi::units::turns>::kv_unit>{1.0},
          wpi::units::unit_t<wpi::math::SimpleMotorFeedforward<wpi::units::turns>::ka_unit>{0.0}});
  ADRCConfig adrc;
  adrc.WithSystemOrder(ADRCConfig::SystemOrder::FIRST);
  smc.WithADRCController(adrc);
  REQUIRE(smc.GetADRCController() != nullptr);
  CHECK(smc.GetKp() == 0.0);
  CHECK_FALSE(smc.GetSimpleFeedforward(Slot::SLOT_0));
  CHECK_FALSE(smc.GetArmFeedforward(Slot::SLOT_0));
  CHECK_FALSE(smc.GetElevatorFeedforward(Slot::SLOT_0));
  CHECK_FALSE(smc.GetLQR(Slot::SLOT_0));
  // Only its own slot.
  CHECK(smc.GetADRCController(Slot::SLOT_1) == nullptr);
  // A PID on the slot removes the ADRC.
  smc.WithFeedback(2.0, 0.0, 0.0);
  CHECK(smc.GetADRCController() == nullptr);
  CHECK(smc.GetKp() == 2.0);
  smc.WithADRCController(adrc).WithoutADRCController();
  CHECK(smc.GetADRCController() == nullptr);
}

TEST_CASE("ADRCConfig.TheMotorConfigReturnsItsOwnInstance", "[ADRCConfig]") {
  auto smc = SmcConfig();
  ADRCConfig adrc;
  adrc.WithSystemOrder(ADRCConfig::SystemOrder::FIRST).WithB0(3);
  smc.WithADRCController(adrc);
  CHECK(smc.GetADRCController() == smc.GetADRCController());
  smc.GetADRCController()->WithB0(5);
  CHECK(*smc.GetADRCController()->GetB0() == 5);
  CHECK(*adrc.GetB0() == 3);
}

TEST_CASE("ADRCConfig.ACopyOfTheMotorConfigGetsItsOwnADRCConfig", "[ADRCConfig]") {
  auto smc = SmcConfig();
  ADRCConfig adrc;
  adrc.WithSystemOrder(ADRCConfig::SystemOrder::FIRST).WithB0(3);
  smc.WithADRCController(adrc);
  SmartMotorControllerConfig copy = smc;
  CHECK(copy.GetADRCController() != smc.GetADRCController());
  copy.GetADRCController()->WithB0(9);
  CHECK(*smc.GetADRCController()->GetB0() == 3);
}

}  // namespace yams::test
