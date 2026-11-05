// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

// Mirrors Java MockHardwareExtension initialises the WPILib HAL and
// simulation environment so that motor-controller and command-scheduler code
// can run inside a Catch2 test case.

#include <wpi/hal/DriverStationTypes.hpp>
#include <wpi/hal/HAL.h>

#include <wpi/simulation/DriverStationSim.hpp>
#include <wpi/simulation/RoboRioSim.hpp>
#include <wpi/simulation/SimHooks.hpp>

namespace yams::test {

inline void InitializeHardware() {
  HAL_Initialize();
  wpi::sim::DriverStationSim::SetDsAttached(true);
  wpi::sim::DriverStationSim::SetRobotMode(wpi::hal::RobotMode::TELEOPERATED);
  wpi::sim::DriverStationSim::SetEnabled(true);
  wpi::sim::DriverStationSim::NotifyNewData();
  wpi::sim::StepTiming(0.0_s);
}

inline void TeardownHardware() {
  wpi::sim::RoboRioSim::ResetData();
  wpi::sim::DriverStationSim::ResetData();
  wpi::sim::DriverStationSim::NotifyNewData();
}

/**
 * Enables the simulated robot for as long as it exists, then resets it. A closed loop on the
 * SystemCore sends no voltage while the robot is disabled, so a test that runs it needs one.
 */
struct EnabledRobot {
  EnabledRobot() { InitializeHardware(); }
  ~EnabledRobot() { TeardownHardware(); }
  EnabledRobot(const EnabledRobot&) = delete;
  EnabledRobot& operator=(const EnabledRobot&) = delete;
};

}  // namespace yams::test
