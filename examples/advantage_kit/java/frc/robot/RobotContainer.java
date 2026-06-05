// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer
{

  private final SwerveSubsystem   drive    = new SwerveSubsystem();
  private final ArmSubsystem      arm      = new ArmSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ShooterSubsystem  shooter  = new ShooterSubsystem();

  private final CommandXboxController xboxController = new CommandXboxController(0);

  public RobotContainer()
  {
    DriverStation.silenceJoystickConnectionWarning(true);
    drive.setDefaultCommand(drive.setRobotRelativeChassisSpeeds(drive.getChassisSpeedsSupplier(xboxController::getLeftY,
                                                                                               xboxController::getLeftX,
                                                                                               xboxController::getRightX)));
    arm.setDefaultCommand(arm.setAngle(Degrees.of(0)));
    elevator.setDefaultCommand(elevator.setHeight(Meters.of(0)));
    shooter.setDefaultCommand(shooter.set(0));
    configureBindings();
  }

  private void configureBindings()
  {

    xboxController.a().whileTrue(arm.setAngle(Degrees.of(20)));
    xboxController.b().whileTrue(elevator.setHeight(Meters.of(1)));
    xboxController.x().whileTrue(shooter.setVelocity(RPM.of(3000)));
    xboxController.leftBumper().whileTrue(drive.driveToPose(new Pose2d(Meters.of(3),
                                                                    Meters.of(3),
                                                                    Rotation2d.fromDegrees(30))));
    xboxController.rightBumper().whileTrue(drive.driveToPose(new Pose2d(Meters.of(5),
                                                                    Meters.of(6),
                                                                    Rotation2d.fromDegrees(70))));

  }

  public Command getAutonomousCommand()
  {
    return null;
  }
}