// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

// Mirrors Java MockHardwareExtension — initialises the WPILib HAL and
// simulation environment so that motor-controller and command-scheduler code
// can run inside Google Test.

#include <frc/simulation/DriverStationSim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/simulation/SimHooks.h>
#include <hal/HAL.h>

namespace yams::test {

inline void InitializeHardware() {
  HAL_Initialize(500, 0);
  frc::sim::DriverStationSim::SetDsAttached(true);
  frc::sim::DriverStationSim::SetAutonomous(false);
  frc::sim::DriverStationSim::SetTest(false);
  frc::sim::DriverStationSim::SetEnabled(true);
  frc::sim::DriverStationSim::NotifyNewData();
  frc::sim::StepTiming(0.0_s);
}

inline void TeardownHardware() {
  frc::sim::RoboRioSim::ResetData();
  frc::sim::DriverStationSim::ResetData();
  frc::sim::DriverStationSim::NotifyNewData();
}

}  // namespace yams::test
