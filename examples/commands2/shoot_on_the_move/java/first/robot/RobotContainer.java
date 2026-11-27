// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static first.robot.Constants.ShotConstants.*;
import static org.wpilib.units.Units.Seconds;

import first.robot.Constants.OperatorConstants;
import first.robot.subsystems.FeederSubsystem;
import first.robot.subsystems.ShooterSubsystem;
import first.robot.subsystems.SwerveSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandNiDsXboxController;
import org.wpilib.command2.button.Trigger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import yams.commands2.swerve.SwerveInputStream;

/** The robot's subsystems and buttons. */
public class RobotContainer {
  private final SwerveSubsystem drive = new SwerveSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final FeederSubsystem feeder = new FeederSubsystem();

  private final CommandNiDsXboxController driverController =
      new CommandNiDsXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  /**
   * The shot for where the robot is, and how it is moving, right now. To try another strategy, change this one line:
   * {@code ShotCalculator.newton(...)} with the same arguments, or {@code ShotCalculator.standing(drive.getPose())}.
   */
  private ShotCalculator.Shot shot() {
    return ShotCalculator.recursive(drive.getPose(), drive.getFieldRelativeSpeed());
  }

  public RobotContainer() {
    // The left stick translates and the right stick's X axis rotates.
    // Pushing a stick forward or left reads negative, so negate each axis to match
    // WPILib's convention: +X forward, +Y left, counterclockwise positive.
    SwerveInputStream input = drive.getChassisSpeedsSupplier(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX())
        // While the right bumper is held, face the shot's target instead of turning with the
        // right stick. The left stick keeps driving.
        .withAim(() -> shot().target(), driverController.rightBumper())
        // Aim from where the robot will be when the game piece leaves it.
        .withAimLookahead(AIM_LOOKAHEAD);
    drive.setDefaultCommand(drive.driveFieldRelative(input));

    // With no button held, the flywheel coasts and the feeder is stopped.
    shooter.setDefaultCommand(shooter.set(0));
    feeder.setDefaultCommand(feeder.stop());

    // While the right bumper is held, keep the flywheel at the speed the shot needs, which
    // changes as the robot moves.
    driverController.rightBumper().whileTrue(shooter.track(() -> shot().flywheelSpeed()));

    // The shot is ready when the robot faces the aim heading, the flywheel is at the shot's
    // speed, and the robot is at a distance the tables cover. All three must hold for a tenth
    // of a second, so that one lucky loop does not fire a shot.
    Trigger ready = input.aimLock(AIM_TOLERANCE)
        .and(shooter.atSpeed(() -> shot().flywheelSpeed(), SPEED_TOLERANCE))
        .and(() -> ShotCalculator.inRange(drive.getPose()))
        .debounce(READY_DEBOUNCE.in(Seconds));

    // Feed only while the shot is ready. The moment it is not, the feeder stops.
    ready.whileTrue(feeder.feed());

    // Press Y to make the way the robot faces now "forward".
    driverController.y().onTrue(drive.zeroGyro());
  }

  /** Draws the real target and the aim point on the drive's field, to watch the lead in simulation. */
  public void showShotOnField() {
    drive.getField2d().getObject("Target").setPose(new Pose2d(ShotCalculator.TARGET, Rotation2d.ZERO));
    drive.getField2d().getObject("Aim point").setPose(shot().target());
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
