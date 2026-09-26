// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import first.robot.Constants.OIConstants;
import first.robot.commands.Autos;
import first.robot.subsystems.DriveSubsystem;
import first.robot.commands.FuelCommands;
import first.robot.subsystems.ConveyorSubsystem;
import first.robot.subsystems.FeederSubsystem;
import first.robot.subsystems.IntakeSubsystem;
import first.robot.subsystems.ShooterSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.command2.button.CommandNiDsXboxController;
import org.wpilib.command2.button.Trigger;
import org.wpilib.tunable.Tunables;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem   m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem   m_intake     = new IntakeSubsystem();
  private final ConveyorSubsystem m_conveyor   = new ConveyorSubsystem();
  private final ShooterSubsystem  m_shooter    = new ShooterSubsystem();
  private final FeederSubsystem   m_feeder     = new FeederSubsystem();

  // The driver's controller
  private final CommandNiDsXboxController m_driverController =
      new CommandNiDsXboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        m_robotDrive.driveCommand(
            m_robotDrive.getInputStream(
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> -m_driverController.getRightX(),
                OIConstants.kDriveDeadband),
            true).withName("Robot Drive Default"));

    // Dashboard buttons for running individual mechanisms.
    Tunables.publish("Intake", FuelCommands.intake(m_intake, m_conveyor).withName("Intake - Intaking"));
    Tunables.publish("Extake", FuelCommands.extake(m_intake, m_conveyor).withName("Intake - Extaking"));

    Tunables.publish("Feeder", FuelCommands.feed(m_shooter, m_feeder).withName("Shooter - Feeding and Shooting"));
    Tunables.publish("Flywheel", m_shooter.runFlywheelCommand().withName("Shooter - Spinning up Flywheel"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * org.wpilib.command2.button.CommandGenericHID}'s subclasses for {@link
   * CommandNiDsXboxController Xbox}/{@link org.wpilib.command2.button.CommandPS4Controller
   * PS4} controllers or {@link org.wpilib.command2.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings()
  {
    // Left Stick Button -> Set swerve to X
    m_driverController.leftStick().whileTrue(m_robotDrive.setXCommand());

    // Start Button -> Zero swerve heading
    m_driverController.start().onTrue(m_robotDrive.zeroHeadingCommand());

    // Right Trigger -> Run fuel intake
    m_driverController
        .rightTrigger(OIConstants.kTriggerButtonThreshold)
        .whileTrue(FuelCommands.intake(m_intake, m_conveyor));

    // Left Trigger -> Run fuel intake in reverse
    m_driverController
        .leftTrigger(OIConstants.kTriggerButtonThreshold)
        .whileTrue(FuelCommands.extake(m_intake, m_conveyor));

    // Y Button -> Run intake and run the shooter flywheel and feeder
    m_driverController.y().toggleOnTrue(FuelCommands.shoot(m_shooter, m_feeder).alongWith(FuelCommands.intake(m_intake, m_conveyor)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_robotDrive);
  }
}
