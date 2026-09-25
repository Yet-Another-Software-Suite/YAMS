// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.auto;

import static first.robot.Constants.FuelConstants.*;

import first.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.RobotState;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;
import org.wpilib.system.Timer;

/** The Everybot's example autonomous: drive for 3 seconds, then launch fuel for 10 seconds. */
@Autonomous
public class ExampleAuto implements OpMode {
  private final Trigger enabled = new Trigger(RobotState::isEnabled);

  /**
   * Creates the autonomous opmode. The routine starts once the robot is enabled and is canceled
   * when the opmode exits.
   *
   * @param robot The robot instance to control.
   */
  public ExampleAuto(Robot robot) {
    enabled.onTrue(exampleAuto(robot));
  }

  /**
   * Drives, then runs the launch sequence. Requires all three mechanisms for the whole routine,
   * like the original sequential command group.
   *
   * @param robot The robot instance to control.
   * @return the autonomous routine
   */
  private static Command exampleAuto(Robot robot) {
    return Command.requiring(robot.drive, robot.intakeLauncher, robot.indexer).executing(coroutine -> {
      // Drive at half speed for 3 seconds. The drive command is resent every loop to keep the
      // DifferentialDrive watchdog fed.
      var driveTimer = Timer.createStarted();
      while (!driveTimer.hasElapsed(3)) {
        robot.drive.setArcadeDrive(0.5, 0.0);
        coroutine.yield();
      }

      // Spin up the launcher for 0.75 second and then launch balls for 9.25 seconds, for a
      // total of 10 seconds. The drive stays stopped, still sent every loop for the watchdog.
      robot.intakeLauncher.setPower(LAUNCHING_LAUNCHER_PERCENT);
      robot.indexer.setPower(INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT);
      var launchTimer = Timer.createStarted();
      while (!launchTimer.hasElapsed(SPIN_UP_SECONDS)) {
        robot.drive.stop();
        coroutine.yield();
      }
      robot.indexer.setPower(INDEXER_LAUNCHING_PERCENT);
      while (!launchTimer.hasElapsed(10)) {
        robot.drive.stop();
        coroutine.yield();
      }
      robot.intakeLauncher.stop();
      robot.indexer.stop();
    }).whenCanceled(() -> {
      robot.drive.stop();
      robot.intakeLauncher.stop();
      robot.indexer.stop();
    }).named("ExampleAuto");
  }
}
