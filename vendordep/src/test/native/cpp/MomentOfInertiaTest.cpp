// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// A config's moment of inertia may only be set once, whichever setter sets it, but a copy of the
// config may set its own. Fetching the default one warns in the SMCConfig alert group. Mirrors the
// Java MomentOfInertiaTest.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <string>

#include <wpi/math/system/DCMotor.hpp>
#include <wpi/simulation/AlertSim.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/mass.hpp>
#include <wpi/units/moment_of_inertia.hpp>
#include <wpi/units/voltage.hpp>

#include "yams/exceptions.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"

namespace yams::test {

using motorcontrollers::SmartMotorControllerConfig;
using exceptions::SmartMotorControllerConfigurationException;

namespace {
SmartMotorControllerConfig Config() {
  SmartMotorControllerConfig config;
  config.WithMotorGearing(12.0);
  return config;
}

/** How many active SMCConfig default moment of inertia alerts there are for a telemetry name. */
long ActiveDefaultMOIAlerts(const std::string& name) {
  auto alerts = wpi::sim::AlertSim::GetActive();
  return std::count_if(alerts.begin(), alerts.end(), [&](const auto& alert) {
    return alert.group == "SMCConfig" && alert.id == name + "/DefaultMOI";
  });
}
}  // namespace

TEST_CASE("MomentOfInertiaTest.SettingItTwiceThrows", "[MomentOfInertia]") {
  {
    auto config = Config();
    config.WithMOI(0.1_kg_sq_m);
    CHECK_THROWS_AS(config.WithMOI(0.2_kg_sq_m), SmartMotorControllerConfigurationException);
  }
  {
    auto config = Config();
    config.WithMOI(0.45_m, 3.6_kg);
    CHECK_THROWS_AS(config.WithWheelMOI(0.1_m, 0.45_kg), SmartMotorControllerConfigurationException);
  }
  {
    auto config = Config();
    config.WithWheelMOI(0.1_m, 0.45_kg);
    CHECK_THROWS_AS(config.WithExponentialProfile(12_V, wpi::math::DCMotor::NEO(1), 0.1_kg_sq_m),
                    SmartMotorControllerConfigurationException);
  }
}

TEST_CASE("MomentOfInertiaTest.ACopyCanSetItAgain", "[MomentOfInertia]") {
  auto original = Config();
  original.WithMOI(0.1_kg_sq_m);
  auto copy = original.Clone();
  CHECK(copy.GetMOI().value() == Catch::Approx(0.1));
  CHECK(copy.IsMomentOfInertiaConfigured());

  copy.WithMOI(0.2_kg_sq_m);
  CHECK(copy.GetMOI().value() == Catch::Approx(0.2));
  CHECK(original.GetMOI().value() == Catch::Approx(0.1));
  CHECK_THROWS_AS(copy.WithMOI(0.3_kg_sq_m), SmartMotorControllerConfigurationException);
  CHECK_THROWS_AS(original.WithMOI(0.3_kg_sq_m), SmartMotorControllerConfigurationException);

  // A plain copy is a copy too.
  SmartMotorControllerConfig assigned = original;
  CHECK_NOTHROW(assigned.WithMOI(0.4_kg_sq_m));
}

TEST_CASE("MomentOfInertiaTest.FetchingTheDefaultWarnsUntilItIsSet", "[MomentOfInertia]") {
  auto config = Config();
  config.WithTelemetry("DefaultMOIAlertTest");
  CHECK(ActiveDefaultMOIAlerts("DefaultMOIAlertTest") == 0);

  config.GetMOI();
  CHECK(ActiveDefaultMOIAlerts("DefaultMOIAlertTest") == 1);

  config.WithMOI(0.1_kg_sq_m);
  config.GetMOI();
  CHECK(ActiveDefaultMOIAlerts("DefaultMOIAlertTest") == 0);
}

TEST_CASE("MomentOfInertiaTest.FetchingASetOneNeverWarns", "[MomentOfInertia]") {
  auto config = Config();
  config.WithTelemetry("SetMOIAlertTest").WithMOI(0.1_kg_sq_m);
  config.GetMOI();
  CHECK(ActiveDefaultMOIAlerts("SetMOIAlertTest") == 0);
}

TEST_CASE("MomentOfInertiaTest.NoWarningWithoutATelemetryName", "[MomentOfInertia]") {
  auto count = [] {
    auto alerts = wpi::sim::AlertSim::GetAll();
    return std::count_if(alerts.begin(), alerts.end(),
                         [](const auto& alert) { return alert.group == "SMCConfig"; });
  };
  auto before = count();
  Config().GetMOI();
  CHECK(count() == before);
}

}  // namespace yams::test
