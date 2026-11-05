// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static first.robot.Constants.ArmConstants.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;

import org.junit.jupiter.api.Test;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;
import org.wpilib.units.measure.Angle;

/** Runs the robot in simulation and checks the A301 arm reaches each setpoint and stows. */
class ArmAutonomousTest {
  private static final double kLoopSeconds = 0.02;

  @Test
  void armReachesEachSetpoint() throws InterruptedException {
    assertTrue(HAL.initialize());
    SimHooks.pauseTiming();
    DriverStationSim.resetData();
    final Robot robot = new Robot();
    final Thread robotThread = new Thread(robot::startCompetition);
    robotThread.start();
    SimHooks.waitForProgramStart();
    try {
      DriverStationSim.setDsAttached(true);
      DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();

      // Teleop with nothing pressed: the default command stows the arm.
      step(3);
      assertAngle(robot, STOW, "stowed in teleop");

      // Each setpoint, driven the way a held button drives it.
      for (Angle setpoint : new Angle[] {LEVEL, HIGH, UP, LEVEL, STOW}) {
        CommandScheduler.getInstance().schedule(robot.robotContainer.arm.setAngle(setpoint));
        step(3);
        assertAngle(robot, setpoint, "held at " + setpoint.in(Degrees) + " degrees");
      }
    } finally {
      robot.endCompetition();
      robotThread.join();
      robot.close();
      SimHooks.resumeTiming();
    }
  }

  private static void assertAngle(Robot robot, Angle expected, String message) {
    final double degrees = robot.robotContainer.arm.getAngle().in(Degrees);
    System.out.printf("%s: %.1f degrees%n", message, degrees);
    assertEquals(expected.in(Degrees), degrees, TOLERANCE.in(Degrees), message);
  }

  private static void step(double seconds) {
    for (int i = 0; i < Math.round(seconds / kLoopSeconds); i++) {
      SimHooks.stepTiming(kLoopSeconds);
    }
  }
}
