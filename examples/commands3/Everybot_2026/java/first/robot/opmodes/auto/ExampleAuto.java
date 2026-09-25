// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.auto;

import static org.wpilib.units.Units.Seconds;

import first.robot.Robot;
import first.robot.commands.FuelCommands;
import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.RobotState;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

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
    return Command.requiring(robot.drive, robot.intakeLauncher, robot.indexer)
        .executing(coroutine -> {
          // Drive at half speed for 3 seconds. autoDrive never ends on its own, so the timeout
          // controls how long it runs.
          coroutine.await(robot.drive.autoDrive(0.5, 0.0).withTimeout(Seconds.of(3)));
          // Spin up the launcher for 0.75 second and then launch balls for 9.25 seconds, for a
          // total of 10 seconds
          coroutine.await(FuelCommands.launchSequence(robot.intakeLauncher, robot.indexer)
              .withTimeout(Seconds.of(10)));
        })
        .named("ExampleAuto");
  }
}
