// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Unqualified Quokkas quokkas2025 (MIT, see LICENSE-UQ).

package first.robot;

import first.robot.Constants.ArmConstants;
import first.robot.Constants.OperatorConstants;
import first.robot.commands.Autos;
import first.robot.subsystems.ArmSubsystem;
import first.robot.subsystems.Climber;
import first.robot.subsystems.DriveTrain;
import first.robot.subsystems.Intake;
import org.wpilib.command2.Command;
import org.wpilib.command2.button.CommandNiDsXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_drive = new DriveTrain();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final Intake m_intake = new Intake();
  private final Climber m_climber = new Climber();

  private final CommandNiDsXboxController m_driverController =
      new CommandNiDsXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {

    // *** Drive bindings ***
    // Default behaviour (follow Y-axes of joysticks to implement tank drive)
    m_drive.setDefaultCommand(m_drive.driveTank(m_driverController::getLeftY, m_driverController::getRightY));


    // *** Arm bindings ***

    // Move to intake coral with Y
    m_driverController.y()
      .onTrue(m_arm.moveArmToPosition(ArmConstants.positionIntakeCoral));

    // Move to intake algae with X
    m_driverController.x()
      .onTrue(m_arm.moveArmToPosition(ArmConstants.positionIntakeAlgae));

    // Move to remove low-reef algae and dump L1 coral with B
    m_driverController.b()
      .onTrue(m_arm.moveArmToPosition(ArmConstants.positionRemoveAlgaeLow));

    // Move to remove high-reef algae with A
    m_driverController.a()
      .onTrue(m_arm.moveArmToPosition(ArmConstants.positionRemoveAlgaeHigh));

    // The D-pad triggers live on the generic HID in 2027.
    // Move to start climb with D-Pad Down
    m_driverController.getHID().povDown()
      .onTrue(m_arm.moveArmToPosition(ArmConstants.positionClimbStart));

    // Move to finish climb with D-Pad up
    m_driverController.getHID().povUp()
      .onTrue(m_arm.moveArmToPosition(ArmConstants.positionClimbEnd));


    // *** Intake bindings ***
    // Default behaviour (do nothing)
    m_intake.setDefaultCommand(m_intake.moveIntake(0.0));

    // Run intake with right bumper button
    m_driverController.rightBumper()
      .and(m_driverController.rightTrigger().negate())
      .whileTrue(m_intake.moveIntake(0.75));

    // Run intake in reverse with right trigger button
    m_driverController.rightTrigger()
      .and(m_driverController.rightBumper().negate())
      .whileTrue(m_intake.moveIntake(-0.75));


    // *** Climber bindings ***
    // Default behaviour (do nothing)
    m_climber.setDefaultCommand(m_climber.moveClimber(0.0));

    // Expand climber hooks with left bumper
    m_driverController.leftBumper()
      .and(m_driverController.leftTrigger().negate())
      .whileTrue(m_climber.moveClimber(0.5));

    // Retract climber hooks with left trigger
    m_driverController.leftTrigger()
      .and(m_driverController.leftBumper().negate())
      .whileTrue(m_climber.moveClimber(-0.5));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Autos.autoSideLeft(m_drive, m_arm, m_intake);
  }
}
