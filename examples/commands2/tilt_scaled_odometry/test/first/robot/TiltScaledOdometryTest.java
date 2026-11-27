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
import org.wpilib.networktables.BooleanSubscriber;
import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.NiDsXboxControllerSim;
import org.wpilib.simulation.SimHooks;

/** Drives the simulated robot on flat ground and verifies unit scale and vision acceptance. */
class TiltScaledOdometryTest {
  private static final double LOOP_SECONDS = 0.02;

  private Robot robot;
  private Thread robotThread;
  private NiDsXboxControllerSim driver;
  private DoubleSubscriber scaleSub;
  private BooleanSubscriber visionAllowedSub;

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
    scaleSub = NetworkTableInstance.getDefault().getDoubleTopic("/TiltOdometry/Scale").subscribe(Double.NaN);
    visionAllowedSub = NetworkTableInstance.getDefault().getBooleanTopic("/TiltOdometry/VisionAllowed").subscribe(false);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    step(0.5);
  }

  @AfterEach
  void stopRobot() throws InterruptedException {
    scaleSub.close();
    visionAllowedSub.close();
    robot.endCompetition();
    robotThread.join();
    robot.close();
    SimHooks.resumeTiming();
  }

  @Test
  void flatDrivingPublishesUnitScaleAndAllowsVision() {
    driver.setLeftY(-0.6);
    driver.notifyNewData();
    step(1.0);
    assertEquals(1.0, scaleSub.get(), 1e-3, "Flat ground should publish scale = 1.0");
    assertTrue(visionAllowedSub.get(), "Flat ground should allow vision measurements");
  }

  private void step(double seconds) {
    for (int i = 0; i < Math.round(seconds / LOOP_SECONDS); i++) {
      SimHooks.stepTiming(LOOP_SECONDS);
    }
  }
}
