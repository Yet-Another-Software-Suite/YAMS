// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static first.robot.Constants.ArmConstants.*;
import static first.robot.Constants.OperatorConstants.*;

import first.robot.subsystems.ArmSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandNiDsXboxController;
import org.wpilib.units.measure.Angle;

/** Holds the arm and binds it to the operator's controller. */
public class RobotContainer {
  public final ArmSubsystem arm = new ArmSubsystem();

  private final CommandNiDsXboxController operatorController = new CommandNiDsXboxController(OPERATOR_CONTROLLER_PORT);

  public RobotContainer() {
    // With nothing else running, the arm holds its stowed angle.
    arm.setDefaultCommand(arm.setAngle(STOW).withName("Arm Stow"));
    configureBindings();
  }

  private void configureBindings() {
    // Hold a button to move the arm there; let go and it stows.
    operatorController.a().whileTrue(arm.setAngle(LEVEL).withName("Arm Level"));
    operatorController.b().whileTrue(arm.setAngle(HIGH).withName("Arm High"));
    operatorController.y().whileTrue(arm.setAngle(UP).withName("Arm Up"));
  }

  /**
   * Swing through every setpoint, waiting at each one, then stow. Enable autonomous to watch the
   * arm move without a controller.
   *
   * @return Autonomous {@link Command}.
   */
  public Command getAutonomousCommand() {
    return Commands.sequence(moveTo(LEVEL), moveTo(HIGH), moveTo(UP), moveTo(LEVEL), moveTo(STOW))
        .withName("Arm Tour");
  }

  /** Move to an angle, then hold it for a second. */
  private Command moveTo(Angle angle) {
    return arm.setAngle(angle).until(arm.isNear(angle)).withTimeout(3)
        .andThen(arm.setAngle(angle).withTimeout(1));
  }
}
