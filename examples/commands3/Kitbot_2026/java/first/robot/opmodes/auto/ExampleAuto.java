// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.auto;

import static org.wpilib.units.Units.Seconds;

import first.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.RobotState;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

/** The KitBot's example autonomous: drive forward briefly, then spin up and launch fuel. */
@Autonomous(name = "Example Auto")
public class ExampleAuto implements OpMode {
  private final Trigger enabled = new Trigger(RobotState::isEnabled);

  /**
   * Creates the autonomous opmode. This constructor is called automatically by the OpModeRobot
   * framework when the opmode is selected on the driver station.
   *
   * @param robot The robot instance to control.
   */
  public ExampleAuto(Robot robot) {
    // Run the routine once the robot is enabled. It is canceled when the opmode exits.
    enabled.onTrue(exampleAuto(robot));
  }

  // Example autonomous command which drives forward and then launches fuel.
  private static Command exampleAuto(Robot robot) {
    return Command.requiring(robot.drive, robot.feeder, robot.intakeLauncher)
        .executing(coroutine -> {
          // Drive backwards for .25 seconds. The driveArcade command factory
          // creates a command which does not end which allows us to control
          // the timing using withTimeout
          coroutine.await(robot.drive.driveArcade(() -> 0.5, () -> 0).withTimeout(Seconds.of(.25)));
          // Stop driving. The stop command ends immediately after commanding the motors to stop
          coroutine.await(robot.drive.stop());
          // Spin up the launcher for 1 second and then launch balls for 9 seconds, for a
          // total of 10 seconds
          coroutine.await(robot.fuel.spinUp().withTimeout(Seconds.of(1)));
          coroutine.await(robot.fuel.launch().withTimeout(Seconds.of(9)));
          // Stop running the launcher
          coroutine.await(robot.fuel.stopCommand());
        })
        .named("Example Auto");
  }
}
