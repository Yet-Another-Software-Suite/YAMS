// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import first.robot.Constants.OperatorConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.networktables.BooleanSubscriber;
import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.NiDsXboxControllerSim;
import org.wpilib.simulation.SimHooks;

/** Drives the simulated robot cleanly and verifies skid telemetry publishes with no false skids. */
class SwerveSkidDetectionTest {
  private static final double LOOP_SECONDS = 0.02;

  private Robot robot;
  private Thread robotThread;
  private NiDsXboxControllerSim driver;
  private DoubleSubscriber ratioSub;
  private BooleanSubscriber skiddingSub;

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
    ratioSub = NetworkTableInstance.getDefault().getDoubleTopic("/SkidDetection/Ratio").subscribe(Double.NaN);
    skiddingSub = NetworkTableInstance.getDefault().getBooleanTopic("/SkidDetection/IsSkidding").subscribe(true);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    step(0.5);
  }

  @AfterEach
  void stopRobot() throws InterruptedException {
    ratioSub.close();
    skiddingSub.close();
    robot.endCompetition();
    robotThread.join();
    robot.close();
    SimHooks.resumeTiming();
  }

  @Test
  void steadyDrivingReportsNoFalseSkids() {
    driver.setLeftY(-0.8);
    driver.notifyNewData();
    step(1.5);
    assertTrue(Double.isFinite(ratioSub.get()), "Skid ratio should be published");
    assertFalse(skiddingSub.get(), "Clean steady driving should not flag a skid");
  }

  private void step(double seconds) {
    for (int i = 0; i < Math.round(seconds / LOOP_SECONDS); i++) {
      SimHooks.stepTiming(LOOP_SECONDS);
    }
  }
}
