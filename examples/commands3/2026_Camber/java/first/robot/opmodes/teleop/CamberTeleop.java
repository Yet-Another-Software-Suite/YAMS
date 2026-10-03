// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.opmodes.teleop;

import first.robot.Constants.Shooter.Setpoints;
import first.robot.Constants.SwerveDrive;
import first.robot.Robot;
import first.robot.commands.Drive;
import first.robot.commands.ShooterCommands;
import first.robot.mechanisms.IndexerMechanism;
import first.robot.mechanisms.ShooterMechanism;
import first.robot.mechanisms.SwerveMechanism;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

/**
 * Teleop with the original bindings. The bindings are created in the constructor, so they only exist
 * while this opmode is selected. The default commands are set in {@link Robot}.
 */
@Teleop(name = "Camber Teleop")
public class CamberTeleop implements OpMode
{

  /**
   * Creates the teleop opmode. The OpModeRobot framework calls this when the opmode is selected on
   * the driver station.
   *
   * @param robot The robot instance to control.
   */
  public CamberTeleop(Robot robot)
  {
    CommandNiDsXboxController driverController   = robot.driverController;
    CommandNiDsXboxController operatorController = robot.operatorController;
    ShooterCommands           shooterCommands    = robot.shooterCommands;
    ShooterMechanism          shooter            = robot.shooter;
    IndexerMechanism          indexer            = robot.indexer;
    SwerveMechanism           drivebase          = robot.drivebase;

    // Shooting commands
    operatorController.a().whileTrue(shooterCommands.shootAndIndex(Setpoints.lowRPM));
    operatorController.b().whileTrue(shooterCommands.shootAndIndex(Setpoints.midRPM));
    operatorController.x().whileTrue(shooterCommands.shootAndIndex(Setpoints.high));
    operatorController.y().whileTrue(shooterCommands.shootAndIndex(Setpoints.maxRPM));
    operatorController.rightTrigger(0.3).whileTrue(shooterCommands.shootAndIndex(drivebase));
    // The D-pad triggers live on the generic HID in 2027.
    operatorController.getHID().povUp().whileTrue(indexer.setDutycycleCommand(-0.8));
    operatorController.getHID().povDown().whileTrue(indexer.setDutycycleCommand(0.8));
    operatorController.getHID().povLeft().whileTrue(shooter.setDutycycleCommand(-0.8));
    operatorController.getHID().povRight().whileTrue(shooter.setDutycycleCommand(0.8));

    // auto-aim
    driverController.leftTrigger(0.3).whileTrue(Drive.autoAim(drivebase, driverController));

    // Intake and outtake controls.
    // TODO: Tune later
    operatorController.rightBumper().whileTrue(shooterCommands.intake());
    operatorController.leftBumper().whileTrue(shooterCommands.outtake());

    // Prevents Swerve Drive from moving by making an X
    driverController.x().whileTrue(drivebase.lock());
    driverController.back().and(driverController.start()).onTrue(drivebase.zeroGyroWithAllianceCommand());
    // Reset odom on field to known points.
    driverController.getHID().povUp().onTrue(drivebase.resetOdometryCommand(SwerveDrive.Setpoints.robotPoseAtHub));
    driverController.getHID().povDown().onTrue(drivebase.resetOdometryCommand(SwerveDrive.Setpoints.robotPoseAtOutpost));
  }
}
