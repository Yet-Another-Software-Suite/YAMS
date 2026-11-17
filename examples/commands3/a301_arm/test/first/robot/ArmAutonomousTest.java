// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static first.robot.Constants.ArmConstants.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;

import first.robot.opmodes.auto.ArmTour;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;
import org.wpilib.units.measure.Angle;

/** Runs the robot in simulation and checks the arm reaches each setpoint, stows, and tours them all. */
class ArmAutonomousTest {
  private static final double kLoopSeconds = 0.02;

  @Test
  void armReachesEachSetpoint() throws InterruptedException {
    assertTrue(HAL.initialize());
    DriverStationSim.resetData();
    final Robot robot = new Robot();
    SimHooks.pauseTiming();
    final Thread robotThread = new Thread(robot::startCompetition);
    robotThread.start();
    SimHooks.waitForProgramStart();
    try {
      DriverStationSim.setDsAttached(true);
      DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();

      // Enabled with nothing else running: the default command stows the arm.
      step(3);
      assertAngle(robot, STOW, "stowed");

      // Each setpoint, held the way a held button holds it.
      for (Angle setpoint : new Angle[] {LEVEL, HIGH, UP, LEVEL}) {
        final Command hold = robot.arm.setAngle(setpoint);
        Scheduler.getDefault().schedule(hold);
        step(3);
        assertAngle(robot, setpoint, "held at " + setpoint.in(Degrees) + " degrees");
        Scheduler.getDefault().cancel(hold);
      }

      // The autonomous tour visits every setpoint, ends, and leaves the arm stowed.
      final Command tour = ArmTour.tour(robot);
      Scheduler.getDefault().schedule(tour);
      double highest = Double.NEGATIVE_INFINITY;
      for (int i = 0; i < Math.round(25 / kLoopSeconds) && Scheduler.getDefault().isScheduledOrRunning(tour); i++) {
        SimHooks.stepTiming(kLoopSeconds);
        highest = Math.max(highest, robot.arm.getAngle().in(Degrees));
      }
      assertFalse(Scheduler.getDefault().isScheduledOrRunning(tour), "the tour ends");
      assertEquals(UP.in(Degrees), highest, 2 * TOLERANCE.in(Degrees), "the tour reaches its highest setpoint");
      assertAngle(robot, STOW, "stowed after the tour");
    } finally {
      robot.endCompetition();
      robotThread.join();
      robot.close();
      SimHooks.resumeTiming();
    }
  }

  private static void assertAngle(Robot robot, Angle expected, String message) {
    final double degrees = robot.arm.getAngle().in(Degrees);
    System.out.printf("%s: %.1f degrees%n", message, degrees);
    assertEquals(expected.in(Degrees), degrees, TOLERANCE.in(Degrees), message);
  }

  private static void step(double seconds) {
    for (int i = 0; i < Math.round(seconds / kLoopSeconds); i++) {
      SimHooks.stepTiming(kLoopSeconds);
    }
  }
}
