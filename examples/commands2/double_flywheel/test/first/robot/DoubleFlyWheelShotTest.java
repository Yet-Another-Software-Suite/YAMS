// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static first.robot.Constants.DoubleFlyWheelConstants.*;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.RPM;

import first.robot.Constants.OperatorConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.NiDsXboxControllerSim;
import org.wpilib.simulation.SimHooks;
import org.wpilib.units.measure.AngularVelocity;

/**
 * Holds the right bumper in Teleop: both wheels reach their shot speeds, within the ready tolerance, in a second,
 * and coast down when it is let go.
 */
class DoubleFlyWheelShotTest {
  private static final double kLoopSeconds = 0.02;

  private Robot robot;
  private Thread robotThread;
  private NiDsXboxControllerSim operator;

  @BeforeEach
  void startRobot() {
    assertTrue(HAL.initialize());
    SimHooks.pauseTiming();
    DriverStationSim.resetData();
    robot = new Robot();
    robotThread = new Thread(robot::startCompetition);
    robotThread.start();
    SimHooks.waitForProgramStart();
    operator = new NiDsXboxControllerSim(OperatorConstants.OPERATOR_CONTROLLER_PORT);
    operator.setButtonsAvailable(-1);
    operator.setButtonsMaximumIndex(32);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    step(0.2);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    step(0.5);
  }

  @AfterEach
  void stopRobot() throws InterruptedException {
    robot.endCompetition();
    robotThread.join();
    robot.close();
    SimHooks.resumeTiming();
  }

  @Test
  void bothWheelsReachTheirShotSpeeds() {
    operator.setRightBumperButton(true);
    operator.notifyNewData();
    step(1);
    assertNear(SHOT_LOWER_SPEED, robot.robotContainer.doubleFlyWheel.getLowerSpeed(), "lower wheel");
    assertNear(SHOT_UPPER_SPEED, robot.robotContainer.doubleFlyWheel.getUpperSpeed(), "upper wheel");

    // Held longer, both wheels stay at speed.
    step(2);
    assertNear(SHOT_LOWER_SPEED, robot.robotContainer.doubleFlyWheel.getLowerSpeed(), "lower wheel");
    assertNear(SHOT_UPPER_SPEED, robot.robotContainer.doubleFlyWheel.getUpperSpeed(), "upper wheel");

    // Let go: both wheels coast down.
    operator.setRightBumperButton(false);
    operator.notifyNewData();
    step(1);
    assertTrue(robot.robotContainer.doubleFlyWheel.getLowerSpeed().lt(SHOT_LOWER_SPEED.div(2)),
        "the lower wheel coasts down, but was at " + robot.robotContainer.doubleFlyWheel.getLowerSpeed().in(RPM) + " RPM");
    assertTrue(robot.robotContainer.doubleFlyWheel.getUpperSpeed().lt(SHOT_UPPER_SPEED.div(2)),
        "the upper wheel coasts down, but was at " + robot.robotContainer.doubleFlyWheel.getUpperSpeed().in(RPM) + " RPM");
  }

  private static void assertNear(AngularVelocity expected, AngularVelocity actual, String wheel) {
    assertTrue(actual.isNear(expected, READY_TOLERANCE),
        wheel + " within " + READY_TOLERANCE.in(RPM) + " RPM of " + expected.in(RPM) + " RPM, but was at "
            + actual.in(RPM) + " RPM");
  }

  private void step(double seconds) {
    for (int i = 0; i < Math.round(seconds / kLoopSeconds); i++) {
      SimHooks.stepTiming(kLoopSeconds);
    }
  }
}
