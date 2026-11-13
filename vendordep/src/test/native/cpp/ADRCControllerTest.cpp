// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// The ADRCController on its own, driving an ideal plant: a chain of `order` integrators, y^(order) = b u + d, with a
// constant disturbance d it is not told about. Mirrors the Java ADRCControllerTest. The ADRC was written by FRC Team
// 8331.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include <wpi/units/time.hpp>
#include <wpi/units/voltage.hpp>

#include "yams/math/ADRCConfig.hpp"
#include "yams/math/ADRCController.hpp"

namespace yams::test {

using namespace math;

namespace {
constexpr double kDt = 0.02;
/** Plant simulation substeps per control period. */
constexpr int kSubsteps = 20;

/** A chain of integrators driven by b u + d. */
struct Plant {
  std::vector<double> x;
  double b;
  double d;

  Plant(int order, double b, double d, double y0) : x(order, 0.0), b{b}, d{d} { x[0] = y0; }

  double Output() const { return x[0]; }

  /** Hold u for one control period. */
  void Step(double u) {
    const double h = kDt / kSubsteps;
    for (int s = 0; s < kSubsteps; ++s) {
      const double top = b * u + d;
      for (size_t i = 0; i + 1 < x.size(); ++i) x[i] += x[i + 1] * h;
      x.back() += top * h;
    }
  }
};

/** A config for a plant of order 1 through 4. */
std::shared_ptr<ADRCConfig> Config(int order) {
  const ADRCConfig::SystemOrder orders[] = {ADRCConfig::SystemOrder::FIRST, ADRCConfig::SystemOrder::SECOND,
                                            ADRCConfig::SystemOrder::THIRD, ADRCConfig::SystemOrder::FOURTH};
  auto config = std::make_shared<ADRCConfig>();
  config->WithSystemOrder(orders[order - 1]);
  return config;
}

/** An initialized controller for a config, with b0 10 unless the config sets one. */
ADRCController Initialized(const std::shared_ptr<ADRCConfig>& config) {
  ADRCController adrc{config};
  adrc.Initialize(10, wpi::units::second_t{kDt});
  return adrc;
}

ADRCController Controller(int order, double settlingTime, double b0) {
  auto config = Config(order);
  config->WithSettlingTime(wpi::units::second_t{settlingTime});
  ADRCController adrc{config};
  adrc.Initialize(b0, wpi::units::second_t{kDt});
  return adrc;
}

/** Run the loop, returning the largest error over the last second. */
double Run(ADRCController& adrc, Plant& plant, double setpoint, double seconds) {
  double worst = 0;
  const int steps = static_cast<int>(std::lround(seconds / kDt));
  for (int i = 0; i < steps; ++i) {
    plant.Step(adrc.Calculate(plant.Output(), setpoint).value());
    if (i >= steps - static_cast<int>(std::lround(1.0 / kDt))) {
      worst = std::max(worst, std::abs(plant.Output() - setpoint));
    }
  }
  return worst;
}

}  // namespace

TEST_CASE("ADRCController.ReachesItsSetpointAgainstAnUnknownDisturbance", "[ADRCController]") {
  const int order = GENERATE(1, 2, 3, 4);
  CAPTURE(order);
  auto adrc = Controller(order, 1.0, 10);
  Plant plant{order, 10, -3, 0};
  CHECK(Run(adrc, plant, 1.0, 6) < 1e-3);
  // The observer's last state is the disturbance it found.
  CHECK(adrc.GetEstimatedDisturbance() == Catch::Approx(-3).margin(0.05));
}

TEST_CASE("ADRCController.ReachesItsSetpointWithB0OffByAFactorOfTwo", "[ADRCController]") {
  const int order = GENERATE(1, 2);
  const double b0 = GENERATE(5.0, 20.0);
  CAPTURE(order, b0);
  auto adrc = Controller(order, 1.0, b0);
  Plant plant{order, 10, -3, 0};
  CHECK(Run(adrc, plant, 1.0, 6) < 1e-3);
}

TEST_CASE("ADRCController.SettlesWhenTheActuatorAppliesLessThanItIsAskedFor", "[ADRCController]") {
  auto adrc = Controller(2, 0.5, 10);
  Plant plant{2, 10, -3, 0};
  constexpr double kActuatorLimit = 3;
  double applied = 0;
  double worst = 0;
  for (int i = 0; i < 400; ++i) {
    const double asked = adrc.Calculate(plant.Output(), 1.0, wpi::units::volt_t{applied}).value();
    applied = std::clamp(asked, -kActuatorLimit, kActuatorLimit);
    plant.Step(applied);
    if (i >= 350) worst = std::max(worst, std::abs(plant.Output() - 1.0));
  }
  CHECK(worst < 1e-3);
}

TEST_CASE("ADRCController.ResetStartsTheObserverAtTheMeasurement", "[ADRCController]") {
  auto adrc = Controller(2, 0.5, 10);
  Plant plant{2, 10, -3, 0};
  Run(adrc, plant, 1.0, 2);
  adrc.Reset(0.25);
  CHECK(adrc.GetEstimatedStates() == std::vector<double>{0.25, 0, 0});
  CHECK(adrc.GetLastOutput().value() == 0);
}

TEST_CASE("ADRCController.ChangingTheConfigRetunesAtOnceAndKeepsTheEstimatedDisturbance", "[ADRCController]") {
  auto adrc = Controller(1, 0.5, 10);
  Plant plant{1, 10, -3, 0};
  Run(adrc, plant, 1.0, 3);
  const double disturbance = adrc.GetEstimatedDisturbance();
  adrc.GetConfig()->WithSettlingTime(0.25_s).WithBandwidthScale(2.0);
  CHECK(adrc.GetEstimatedDisturbance() == disturbance);
  CHECK(adrc.GetSettlingTime().value() == 0.25);
  CHECK(adrc.GetBandwidthScale() == 2.0);
  CHECK(adrc.GetBandwidth().value() == Catch::Approx(2.0 * 2 * 2.0 / 0.25));
  CHECK(adrc.GetObserverBandwidth().value() == Catch::Approx(4 * adrc.GetBandwidth().value()));
  adrc.GetConfig()->WithB0(12);
  CHECK(adrc.GetB0() == 12);
  CHECK(adrc.GetEstimatedDisturbance() == disturbance);
  CHECK(Run(adrc, plant, 1.0, 3) < 1e-3);
}

TEST_CASE("ADRCController.InitializeReadsTheConfigOverTheGivenB0", "[ADRCController]") {
  auto config = Config(1);
  config->WithSettlingTime(0.4_s).WithBandwidthScale(2).WithB0(5);
  ADRCController fromConfig{config};
  fromConfig.Initialize(10, wpi::units::second_t{kDt});
  CHECK(fromConfig.GetSettlingTime().value() == 0.4);
  CHECK(fromConfig.GetBandwidthScale() == 2);
  CHECK(fromConfig.GetB0() == 5);

  auto fromDefaults = Initialized(Config(1));
  CHECK(fromDefaults.GetSettlingTime() == ADRCConfig::kDefaultSettlingTime);
  CHECK(fromDefaults.GetBandwidthScale() == ADRCConfig::kDefaultBandwidthScale);
  CHECK(fromDefaults.GetB0() == 10);
}

TEST_CASE("ADRCController.InputDelayHoldsTheSetpointOnADelayedPlant", "[ADRCController]") {
  auto config = Config(1);
  config->WithSettlingTime(0.5_s).WithInputDelay(wpi::units::second_t{2 * kDt});
  auto adrc = Initialized(config);
  Plant plant{1, 10, -3, 0};
  double pending[] = {0, 0};
  double worst = 0;
  for (int i = 0; i < 300; ++i) {
    const double u = adrc.Calculate(plant.Output(), 1.0).value();
    plant.Step(pending[1]);
    pending[1] = pending[0];
    pending[0] = u;
    if (i >= 250) worst = std::max(worst, std::abs(plant.Output() - 1.0));
  }
  CHECK(worst < 1e-3);
}

TEST_CASE("ADRCController.TrackingDifferentiatorSmoothsTheSetpoint", "[ADRCController]") {
  const auto method = GENERATE(TrackingDifferentiator::Method::EULER, TrackingDifferentiator::Method::TOD,
                               TrackingDifferentiator::Method::LD, TrackingDifferentiator::Method::RED,
                               TrackingDifferentiator::Method::INTD);
  CAPTURE(static_cast<int>(method));
  auto config = Config(2);
  config->WithSettlingTime(0.5_s).WithTrackingDifferentiator(method);
  auto adrc = Initialized(config);
  REQUIRE(adrc.GetTrackingDifferentiator() != nullptr);
  Plant plant{2, 10, -3, 0};
  CHECK(Run(adrc, plant, 1.0, 15) < 1e-2);
}

TEST_CASE("ADRCController.CascadedObserverHoldsTheSetpoint", "[ADRCController]") {
  auto config = Config(1);
  config->WithSettlingTime(0.5_s).WithCascaded(true);
  auto adrc = Initialized(config);
  Plant plant{1, 10, -3, 0};
  CHECK(Run(adrc, plant, 1.0, 6) < 1e-3);
}

TEST_CASE("ADRCController.ACopyOfABuiltConfigTunesOnItsOwn", "[ADRCController]") {
  auto adrc = Controller(2, 0.5, 10);
  ADRCConfig copy{*adrc.GetConfig()};
  CHECK(copy.GetSettlingTime() == adrc.GetConfig()->GetSettlingTime());
  CHECK_THROWS_AS(copy.WithSystemOrder(ADRCConfig::SystemOrder::FIRST), std::logic_error);
  copy.WithSettlingTime(1_s);
  CHECK(adrc.GetSettlingTime().value() == 0.5);
  CHECK(copy.GetSettlingTime()->value() == 1.0);
}

TEST_CASE("ADRCController.RejectsInvalidArguments", "[ADRCController]") {
  CHECK_THROWS_AS(ADRCController{std::make_shared<ADRCConfig>()}, std::invalid_argument);
  ADRCController adrc{Config(1)};
  CHECK_FALSE(adrc.IsInitialized());
  CHECK_THROWS_AS(adrc.Calculate(0, 0), std::logic_error);
  CHECK_THROWS_AS(adrc.Reset(0), std::logic_error);
  CHECK_THROWS_AS(adrc.Initialize(10, 0_s), std::invalid_argument);
  CHECK_THROWS_AS(adrc.Initialize(0, wpi::units::second_t{kDt}), std::invalid_argument);
  adrc.Initialize(10, wpi::units::second_t{kDt});
  CHECK(adrc.IsInitialized());
  CHECK_THROWS_AS(adrc.Reset(std::vector<double>(3, 0.0), 0_V), std::invalid_argument);
  CHECK_THROWS_AS(adrc.GetConfig()->WithSystemOrder(ADRCConfig::SystemOrder::SECOND), std::logic_error);
  adrc.GetConfig()->WithSystemOrder(ADRCConfig::SystemOrder::FIRST);
}

TEST_CASE("ADRCController.TracksAMovingSetpointGivenItsRate", "[ADRCController]") {
  // A setpoint moving at a steady rate, as a motion profile's does while it cruises. Given where it is now and where it
  // will be next, the controller tracks it; told only the position, it trails it, by the rate divided by the bandwidth
  // for a first order loop and twice that for a second order loop.
  const int order = GENERATE(1, 2);
  const double rate = 0.5;
  const int steps = static_cast<int>(std::lround(4.0 / kDt));
  double lag[2] = {0, 0};
  for (int withRate = 0; withRate < 2; ++withRate) {
    ADRCController adrc = Controller(order, 0.5, 10);
    Plant plant{order, 10, 0, 0};
    for (int i = 0; i < steps; ++i) {
      const double now = rate * i * kDt;
      const double next = rate * (i + 1) * kDt;
      plant.Step(withRate == 1 ? adrc.Calculate(plant.Output(), ADRCController::Reference{now, rate},
                                                ADRCController::Reference{next, rate}, wpi::units::volt_t{std::nan("")})
                                     .value()
                               : adrc.Calculate(plant.Output(), now, wpi::units::volt_t{std::nan("")}).value());
    }
    // The plant has run one period past the last setpoint: compare it with where the setpoint is now.
    lag[withRate] = rate * steps * kDt - plant.Output();
  }
  const double bandwidth = 2.0 * (order + 1) / 0.5;
  CHECK(lag[0] == Catch::Approx(order * rate / bandwidth).margin(0.02));
  CHECK(std::abs(lag[1]) < 0.02 * lag[0]);
}

TEST_CASE("ADRCController.AStillReferenceIsThePlainSetpoint", "[ADRCController]") {
  // A still reference is the plain setpoint, and a non-finite rate is taken as zero.
  const int order = GENERATE(1, 2);
  ADRCController plain = Controller(order, 0.5, 10);
  ADRCController still = Controller(order, 0.5, 10);
  ADRCController nonFinite = Controller(order, 0.5, 10);
  const wpi::units::volt_t none{std::nan("")};
  for (int i = 0; i < 10; ++i) {
    const double expected = plain.Calculate(0.1 * i, 1.0, none).value();
    CHECK(still.Calculate(0.1 * i, ADRCController::Reference::Still(1.0), ADRCController::Reference::Still(1.0), none)
              .value() == Catch::Approx(expected).margin(1e-12));
    CHECK(nonFinite
              .Calculate(0.1 * i, ADRCController::Reference{1.0, std::nan("")},
                         ADRCController::Reference{1.0, std::numeric_limits<double>::infinity()}, none)
              .value() == Catch::Approx(expected).margin(1e-12));
  }
}

}  // namespace yams::test
