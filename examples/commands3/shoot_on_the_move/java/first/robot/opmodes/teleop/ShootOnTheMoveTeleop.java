// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.teleop;

import static first.robot.Constants.ShotConstants.*;

import first.robot.Robot;
import first.robot.ShotCalculator;
import org.wpilib.command3.Trigger;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;
import yams.commands3.swerve.SwerveInputStream;

/**
 * Teleop: drive with the sticks, and hold the right bumper to aim, spin the flywheel to the shot's speed, and feed
 * only once the shot is ready. The bindings are created when this opmode is selected and removed when it exits.
 */
@Teleop(name = "Shoot On The Move Teleop")
public class ShootOnTheMoveTeleop implements OpMode {
  /**
   * Creates the teleop opmode. This constructor is called automatically by the OpModeRobot framework when the opmode
   * is selected on the driver station.
   *
   * @param robot The robot instance to control.
   */
  public ShootOnTheMoveTeleop(Robot robot) {
    // The left stick translates and the right stick's X axis rotates.
    // Pushing a stick forward or left reads negative, so negate each axis to match
    // WPILib's convention: +X forward, +Y left, counterclockwise positive.
    SwerveInputStream input = robot.drive.getChassisSpeedsSupplier(
            () -> -robot.driverController.getLeftY(),
            () -> -robot.driverController.getLeftX(),
            () -> -robot.driverController.getRightX())
        // While the right bumper is held, face the shot's target instead of turning with the
        // right stick. The left stick keeps driving.
        .withAim(() -> robot.shot().target(), robot.driverController.rightBumper())
        // Aim from where the robot will be when the game piece leaves it.
        .withAimLookahead(AIM_LOOKAHEAD);
    robot.drive.setDefaultCommand(robot.drive.driveFieldRelative(input));

    // While the right bumper is held, keep the flywheel at the speed the shot needs, which
    // changes as the robot moves.
    robot.driverController.rightBumper().whileTrue(robot.shooter.track(() -> robot.shot().flywheelSpeed()));

    // The shot is ready when the robot faces the aim heading, the flywheel is at the shot's
    // speed, and the robot is at a distance the tables cover. All three must hold for a tenth
    // of a second, so that one lucky loop does not fire a shot.
    Trigger ready = input.aimLock(AIM_TOLERANCE)
        .and(robot.shooter.atSpeed(() -> robot.shot().flywheelSpeed(), SPEED_TOLERANCE))
        .and(() -> ShotCalculator.inRange(robot.drive.getPose()))
        .debounce(READY_DEBOUNCE);

    // Feed only while the shot is ready. The moment it is not, the feeder stops.
    ready.whileTrue(robot.feeder.feed());

    // Press Y to make the way the robot faces now "forward".
    robot.driverController.y().onTrue(robot.drive.zeroGyro());
  }
}
