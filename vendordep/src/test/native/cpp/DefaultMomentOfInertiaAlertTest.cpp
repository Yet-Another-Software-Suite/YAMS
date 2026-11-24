// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// A motor controller warns in simulation when it builds its simulation from the default moment of
// inertia, and not when one is set. Motor controllers that share a telemetry name, as a swerve
// drive's modules do, each get their own alert. Mirrors the Java DefaultMomentOfInertiaAlertTest.

#include <rev/SparkMax.h>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <wpi/hardware/bus/CANPort.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/simulation/AlertSim.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/mass.hpp>

#include "helpers/MockHardware.h"
#include "helpers/MotorControllerFactory.h"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/motorcontrollers/local/SparkWrapper.hpp"

namespace yams::test {

using motorcontrollers::SmartMotorControllerConfig;
using motorcontrollers::local::SparkWrapper;

namespace {
/** How many active default moment of inertia alerts there are for a telemetry name. */
long ActiveAlerts(const std::string& name) {
  auto alerts = wpi::sim::AlertSim::GetActive();
  return std::count_if(alerts.begin(), alerts.end(), [&](const auto& alert) {
    return alert.group == "YAMS" &&
           alert.id.ends_with("_DefaultMomentOfInertia_" + name);
  });
}

SmartMotorControllerConfig Config(const std::string& name) {
  SmartMotorControllerConfig cfg;
  cfg.WithMotorGearing(gearing::MechanismGearing{5.0}).WithTelemetry(name);
  return cfg;
}

std::unique_ptr<rev::spark::SparkMax> MakeSpark() {
  return std::make_unique<rev::spark::SparkMax>(wpi::CANPort::CAN_S0, NextCanId(),
                                                rev::spark::SparkLowLevel::MotorType::kBrushless);
}
}  // namespace

TEST_CASE("DefaultMomentOfInertiaAlertTest.WarnsOnlyForTheDefault", "[MomentOfInertia]") {
  InitializeHardware();
  auto unsetConfig = Config("SparkDefaultMOI");
  auto setConfig = Config("SparkSetMOI");
  setConfig.WithMOI(0.1_m, 0.45_kg);
  auto unsetSpark = MakeSpark();
  auto setSpark = MakeSpark();
  {
    SparkWrapper unset{unsetSpark.get(), wpi::math::DCMotor::NEO(1), &unsetConfig};
    SparkWrapper set{setSpark.get(), wpi::math::DCMotor::NEO(1), &setConfig};
    CHECK(ActiveAlerts("SparkDefaultMOI") == 1);
    CHECK(ActiveAlerts("SparkSetMOI") == 0);
  }
  TeardownHardware();
}

TEST_CASE("DefaultMomentOfInertiaAlertTest.MotorControllersThatShareATelemetryNameEachWarn",
          "[MomentOfInertia]") {
  InitializeHardware();
  // A swerve drive names every module's drive motor the same.
  auto firstConfig = Config("SparkDriveMotor");
  auto secondConfig = Config("SparkDriveMotor");
  auto firstSpark = MakeSpark();
  auto secondSpark = MakeSpark();
  {
    SparkWrapper first{firstSpark.get(), wpi::math::DCMotor::NEO(1), &firstConfig};
    SparkWrapper second{secondSpark.get(), wpi::math::DCMotor::NEO(1), &secondConfig};
    CHECK(ActiveAlerts("SparkDriveMotor") == 2);
  }
  TeardownHardware();
}

}  // namespace yams::test
