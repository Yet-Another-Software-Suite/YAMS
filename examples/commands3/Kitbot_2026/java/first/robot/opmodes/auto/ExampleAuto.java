// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.auto;

import first.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.RobotState;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;
import org.wpilib.system.Timer;

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
          // Drive for .25 seconds. DifferentialDrive motor safety needs the output sent every
          // loop, so keep driving until the time is up
          Timer timer = new Timer();
          timer.start();
          while (!timer.hasElapsed(.25)) {
            robot.drive.arcadeDrive(0.5, 0);
            coroutine.yield();
          }
          // Spin up the launcher for 1 second and then launch balls for 9 seconds, for a
          // total of 10 seconds. The drive stays stopped, still sent every loop for motor safety
          robot.fuel.setSpinUp();
          timer.restart();
          while (!timer.hasElapsed(1)) {
            robot.drive.arcadeDrive(0, 0);
            coroutine.yield();
          }
          robot.fuel.setLaunch();
          timer.restart();
          while (!timer.hasElapsed(9)) {
            robot.drive.arcadeDrive(0, 0);
            coroutine.yield();
          }
          // Stop running the launcher
          robot.fuel.stop();
        })
        .named("Example Auto");
  }
}
