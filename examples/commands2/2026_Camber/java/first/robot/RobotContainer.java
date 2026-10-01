// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot;

import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import first.robot.Constants.Shooter.Setpoints;
import first.robot.Constants.SwerveDrive;
import first.robot.commands.AutoAimCommand;
import first.robot.commands.IntakeCommand;
import first.robot.commands.OuttakeCommand;
import first.robot.commands.ShootAndIndexCommand;
import first.robot.subsystems.IndexerSubsystem;
import first.robot.subsystems.ShooterSubsystem;
import first.robot.subsystems.SwerveSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.command2.button.CommandNiDsXboxController;
import org.wpilib.math.util.MathUtil;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunables;
import org.wpilib.units.measure.Angle;
import yams.commands2.swerve.SwerveInputStream;

public class RobotContainer
{

  private final ShooterSubsystem shooter   = new ShooterSubsystem();
  private final IndexerSubsystem indexer   = new IndexerSubsystem();
  private final SwerveSubsystem  drivebase = new SwerveSubsystem();
  private final CommandNiDsXboxController driverController   = new CommandNiDsXboxController(0);
  private final CommandNiDsXboxController operatorController = new CommandNiDsXboxController(1);
  Selectable<Command> autChooser;
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverController.getLeftY() * -1,
                                                                () -> driverController.getLeftX() * -1) // set to 0
                                                            .withControllerRotationAxis(() -> driverController.getRightX())
                                                            .deadband(.1)
                                                            .withScaleTranslation(.8)
                                                            .withAllianceRelativeControl();

  /// Testing SwerveInputStream to ensure that our swerve drive is capable of running in autonomous
  SwerveInputStream driveDirectAngle = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                            () -> driverController.getLeftY() * -1,
                                                            () -> driverController.getLeftX() * -1)
                                                        .withHeading(this::rightStickHeading)
                                                        .deadband(0.1)
                                                        .withScaleTranslation(.8).withHeadingControl(() -> true)
                                                        .withAllianceRelativeControl();

  // Heading picked with the right stick. YAGSL kept the last heading while the stick was inside
  // angleJoystickRadiusDeadband, starting at 0.
  private Angle lastStickHeading = Radians.of(0);

  public RobotContainer()
  {
    // Regular control is commented out below.
    drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveDirectAngle));
    // Test control is this.
    // drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveDirectAngle)); // Use this to test for the 8 steps.

    shooter.setDefaultCommand(shooter.setVelocityCommand(() -> Setpoints.maxRPM.times(Math.clamp(MathUtil.applyDeadband(
                                                                                                         -operatorController.getRightY(),
                                                                                                         0.1),
                                                                                                     0,
                                                                                                     1))));
    indexer.setDefaultCommand(indexer.setDutycycleCommand(0));
//
    //new EventTrigger("StartIntake").onTrue(new IntakeCommand(indexer, shooter));
    //new EventTrigger("StopIntake").onTrue(new OuttakeCommand(indexer, shooter));
//        NamedCommands.registerCommand("ShootBalls",
//                shooter.setVelocityCommand(Shooter.Setpoints.autonomousPeriodRPM)
//                        .withTimeout(Seconds.of(4)));
    NamedCommands.registerCommand("ShootBallsOdom",
                                  new ShootAndIndexCommand(indexer,
                                                           shooter,
                                                           drivebase).withTimeout(4));

    NamedCommands.registerCommand("ShootBalls",
                                  new ShootAndIndexCommand(indexer,
                                                           shooter,
                                                           Setpoints.autonomousPeriodRPM).withTimeout(4));

    NamedCommands.registerCommand("Stop", STOP());

    NamedCommands.registerCommand("StartIntake", new IntakeCommand(indexer, shooter));

    autChooser = AutoBuilder.buildAutoChooser("NO Auto");
    Tunables.publish("Auto Chooser", autChooser);

    configureBindings();
  }

  /** The right stick's heading, held while the stick is near the center. */
  private Angle rightStickHeading()
  {
    double x = driverController.getRightX();
    double y = driverController.getRightY();
    if (Math.hypot(x, y) > SwerveDrive.Modules.angleJoystickRadiusDeadband)
    {
      lastStickHeading = Radians.of(Math.atan2(x, y));
    }
    return lastStickHeading;
  }

  public Command STOP()
  {
    return shooter.setDutycycleCommand(0)
                  .alongWith(indexer.setDutycycleCommand(0)).withTimeout(Seconds.of(0.000001));
  }

  private void configureBindings()
  {
    Tunables.publish("Index Balls",
                     indexer.setDutycycleCommand(-1.0).onlyWhile(() -> shooter.isNear(RPM.of(25))).repeatedly());
    // Shooting commands
    operatorController.a().whileTrue(new ShootAndIndexCommand(indexer, shooter, Setpoints.lowRPM));
    operatorController.b().whileTrue(new ShootAndIndexCommand(indexer, shooter, Setpoints.midRPM));
    operatorController.x().whileTrue(new ShootAndIndexCommand(indexer, shooter, Setpoints.high));
    operatorController.y().whileTrue(new ShootAndIndexCommand(indexer, shooter, Setpoints.maxRPM));
    operatorController.rightTrigger(0.3).whileTrue(new ShootAndIndexCommand(indexer, shooter, drivebase));
    // The D-pad triggers live on the generic HID in 2027.
    operatorController.getHID().povUp().whileTrue(indexer.setDutycycleCommand(-0.8));
    operatorController.getHID().povDown().whileTrue(indexer.setDutycycleCommand(0.8));
    operatorController.getHID().povLeft().whileTrue(shooter.setDutycycleCommand(-0.8));
    operatorController.getHID().povRight().whileTrue(shooter.setDutycycleCommand(0.8));

    // auto-aim
    driverController.leftTrigger(0.3).whileTrue(new AutoAimCommand(drivebase, driveAngularVelocity, 0.4));

    // Intake and outtake controls.
    // TODO: Tune later
    operatorController.rightBumper().whileTrue(new IntakeCommand(indexer, shooter));
    operatorController.leftBumper().whileTrue(new OuttakeCommand(indexer, shooter));

    // Shooting commands

    // Prevents Swerve Drive from moving by making an X
    driverController.x().whileTrue(drivebase.lock());
    driverController.back().and(driverController.start()).onTrue(drivebase.zeroGyroWithAllianceCommand());
    // Reset odom on field to known points.
    driverController.getHID().povUp().onTrue(drivebase.resetOdometryCommand(SwerveDrive.Setpoints.robotPoseAtHub));
    driverController.getHID().povDown().onTrue(drivebase.resetOdometryCommand(SwerveDrive.Setpoints.robotPoseAtOutpost));


  }


  /** @return The drivetrain, for tests. */
  SwerveSubsystem getDrivebase()
  {
    return drivebase;
  }

  public Command getAutonomousCommand()
  {
    return drivebase.getAutonomousCommand("Left Auto");

  }
}
