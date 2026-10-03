// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static first.robot.Constants.OperatorConstants.*;

import first.robot.commands.ClimbDown;
import first.robot.commands.ClimbUp;
import first.robot.commands.Drive;
import first.robot.commands.Eject;
import first.robot.commands.ExampleAuto;
import first.robot.commands.Intake;
import first.robot.commands.LaunchSequence;
import first.robot.subsystems.CANDriveSubsystem;
import first.robot.subsystems.IndexerSubsystem;
import first.robot.subsystems.IntakeLauncherSubsystem;
import first.robot.subsystems.ClimberSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.command2.button.CommandNiDsXboxController;
import org.wpilib.command2.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final IntakeLauncherSubsystem intakeLauncherSubsystem = new IntakeLauncherSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  // The driver's controller
  private final CommandNiDsXboxController driverController = new CommandNiDsXboxController(
      DRIVER_CONTROLLER_PORT);

  // The operator's controller, by default it is setup to use a single controller
  private final CommandNiDsXboxController operatorController = new CommandNiDsXboxController(
      OPERATOR_CONTROLLER_PORT);

  // The autonomous routine. WPILib 2027 removed SendableChooser in favor of opmodes, and the
  // Everybot only ships one auto, so it is built once here.
  private final Command autonomousCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();

    autonomousCommand = new ExampleAuto(driveSubsystem, intakeLauncherSubsystem, indexerSubsystem);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link org.wpilib.command2.button.CommandGenericHID}'s subclasses
   * for {@link CommandNiDsXboxController Xbox}/
   * {@link org.wpilib.command2.button.CommandPS4Controller PS4}
   * controllers or
   * {@link org.wpilib.command2.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // While the left bumper on operator controller is held, intake Fuel
    driverController.leftBumper().whileTrue(new Intake(intakeLauncherSubsystem, indexerSubsystem));
    // While the right bumper on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    driverController.rightBumper().whileTrue(new LaunchSequence(intakeLauncherSubsystem, indexerSubsystem));
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    driverController.a().whileTrue(new Eject(intakeLauncherSubsystem, indexerSubsystem));
    // The D-pad triggers live on the generic HID in 2027.
    // While the down arrow on the directional pad is held it will unclimb the robot
    driverController.getHID().povDown().whileTrue(new ClimbDown(climberSubsystem));
    // While the up arrow on the directional pad is held it will climb the robot
    driverController.getHID().povUp().whileTrue(new ClimbUp(climberSubsystem));

    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value)
    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));

    intakeLauncherSubsystem.setDefaultCommand(intakeLauncherSubsystem.run(() -> intakeLauncherSubsystem.stop()));

    indexerSubsystem.setDefaultCommand(indexerSubsystem.run(() -> indexerSubsystem.stop()));

    climberSubsystem.setDefaultCommand(climberSubsystem.run(() -> climberSubsystem.stop()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autonomousCommand;
  }
}
