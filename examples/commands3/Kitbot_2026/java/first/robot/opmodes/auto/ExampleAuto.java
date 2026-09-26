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

  // Example autonomous command which drives forward and then launches fuel. It requires no
  // mechanism itself: each step runs a mechanism's own command, and a mechanism's default command
  // takes over as soon as its step is done. That keeps the drive stopped (and motor safety fed)
  // while launching, and stops the rollers when the routine ends.
  private static Command exampleAuto(Robot robot) {
    return Command.noRequirements(coroutine -> {
      // Drive for .25 seconds. Whichever command finishes first cancels the other, so the drive
      // command ends when the timer does
      coroutine.awaitAny(
          robot.drive.driveArcade(0.5, 0),
          Command.waitFor(Seconds.of(.25)).named("Example Auto.DriveTime"));
      // Spin up the launcher for 1 second and then launch balls for 9 seconds, for a
      // total of 10 seconds
      coroutine.awaitAny(
          robot.fuel.spinUpAndLaunch(),
          Command.waitFor(Seconds.of(10)).named("Example Auto.LaunchTime"));
    }).named("Example Auto");
  }
}
