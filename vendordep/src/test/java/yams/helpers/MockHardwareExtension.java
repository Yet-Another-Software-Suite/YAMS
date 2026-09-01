// Copyright (c) 2025-2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.helpers;

import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.simulation.SimHooks;

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
