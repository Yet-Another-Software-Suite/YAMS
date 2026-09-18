// Copyright (c) 2025-2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.helpers;

import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.simulation.SimHooks;
import yams.core.motorcontrollers.simulation.BatterySim;

/**
 * JUnit 5 testing extension which ensures all WPILib foundational bits are
 * initialized to be able to run the scheduler.
 */
public final class MockHardwareExtension {
  public static void beforeAll() {
    initializeHardware();
  }

  public static void afterAll() {
    RoboRioSim.resetData();
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
    // Without this, a SmartMotorController that wasn't (or couldn't be) closed by an earlier
    // test leaves its last-known current draw permanently counted towards every later test's
    // shared battery voltage calculation for the rest of the JVM's lifetime.
    BatterySim.reset();
    //		HAL.releaseDSMutex();
  }

  private static void initializeHardware() {
    HAL.initialize();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    SimHooks.stepTiming(0.0); // Wait for Notifiers
  }
}
