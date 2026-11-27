// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import first.robot.Constants.OperatorConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.NiDsXboxControllerSim;
import org.wpilib.simulation.SimHooks;

/**
 * Holds the right bumper in teleop, in range of the target: the robot turns to face it, the flywheel spins up, and only
 * then does the feeder run. Letting go stops the feeder.
 */
class ShootOnTheMoveTest {
  private static final double kLoopSeconds = 0.02;

  private Robot robot;
  private Thread robotThread;
  private NiDsXboxControllerSim driver;
  private DoubleSubscriber feederVoltage;

  @BeforeEach
  void startRobot() {
    assertTrue(HAL.initialize());
    SimHooks.pauseTiming();
    DriverStationSim.resetData();
    robot = new Robot();
    robotThread = new Thread(robot::startCompetition);
    robotThread.start();
    SimHooks.waitForProgramStart();
    driver = new NiDsXboxControllerSim(OperatorConstants.DRIVER_CONTROLLER_PORT);
    driver.setButtonsAvailable(-1);
    driver.setButtonsMaximumIndex(32);
    feederVoltage = NetworkTableInstance.getDefault()
        .getDoubleTopic("/Mechanisms/Feeder/FeederMotor/motor/outputVoltage")
        .subscribe(Double.NaN);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    step(0.5);
  }

  @AfterEach
  void stopRobot() throws InterruptedException {
    feederVoltage.close();
    robot.endCompetition();
    robotThread.join();
    robot.close();
    SimHooks.resumeTiming();
  }

  @Test
  void feedsOnlyOnceTheShotIsReady() {
    // Nothing held: the feeder stays stopped.
    assertEquals(0, feederVoltage.get(), 1e-6, "the feeder runs with nothing held");

    // Hold the right bumper. In the first moments the robot is still turning to face the
    // target and the flywheel is still spinning up, so the shot is not ready yet.
    driver.setRightBumperButton(true);
    driver.notifyNewData();
    step(0.1);
    assertEquals(0, feederVoltage.get(), 1e-6, "the feeder runs before the shot is ready");

    // Held long enough to aim and reach speed, the feeder runs.
    step(4);
    assertTrue(feederVoltage.get() > 1, "the feeder runs once the shot is ready, at " + feederVoltage.get() + " V");

    // Let go: the aim ends, the shot is no longer ready, and the feeder stops.
    driver.setRightBumperButton(false);
    driver.notifyNewData();
    step(0.2);
    assertEquals(0, feederVoltage.get(), 1e-6, "the feeder keeps running after the bumper is let go");
  }

  private void step(double seconds) {
    for (int i = 0; i < Math.round(seconds / kLoopSeconds); i++) {
      SimHooks.stepTiming(kLoopSeconds);
    }
  }
}
