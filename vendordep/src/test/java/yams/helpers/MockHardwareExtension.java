package yams.helpers;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;

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
		HAL.initialize(500, 0);
		DriverStationSim.setDsAttached(true);
		DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    SimHooks.stepTiming(0.0); // Wait for Notifiers
	}
}