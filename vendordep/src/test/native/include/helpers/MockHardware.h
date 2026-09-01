// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

// Mirrors Java MockHardwareExtension — initialises the WPILib HAL and
// simulation environment so that motor-controller and command-scheduler code
// can run inside Google Test.

#include <wpi/simulation/DriverStationSim.hpp>
#include <wpi/simulation/RoboRioSim.hpp>
#include <wpi/simulation/SimHooks.hpp>
#include <wpi/hal/HAL.h>

namespace yams::test {

inline void InitializeHardware() {
  HAL_Initialize(500, 0);
  wpi::sim::DriverStationSim::SetDsAttached(true);
  wpi::sim::DriverStationSim::SetRobotMode(HAL_ROBOT_MODE_TELEOPERATED);
  wpi::sim::DriverStationSim::SetEnabled(true);
  wpi::sim::DriverStationSim::NotifyNewData();
  wpi::sim::StepTiming(0.0_s);
}

inline void TeardownHardware() {
  wpi::sim::RoboRioSim::ResetData();
  wpi::sim::DriverStationSim::ResetData();
  wpi::sim::DriverStationSim::NotifyNewData();
}

}  // namespace yams::test
