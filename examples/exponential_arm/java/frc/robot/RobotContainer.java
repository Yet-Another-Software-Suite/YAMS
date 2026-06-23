// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ExponentiallyProfiledArmSubsystem;

import static edu.wpi.first.units.Units.*;


/**
 * RobotContainer wires together all subsystems, default commands, and operator
 * bindings. The structure keeps hardware construction (subsystems) separate from
 * behavior (commands) and from the WPILib entry point (Robot.java).
 */
public class RobotContainer
{
  private ExponentiallyProfiledArmSubsystem arm = new ExponentiallyProfiledArmSubsystem();
  // Port 0 is the driver controller. Use port 1 for a separate operator controller.
  public CommandXboxController xboxController = new CommandXboxController(0);

  public RobotContainer()
  {
    // Suppress the "joystick not connected" warning in sim so the log is not spammed
    // when running without a physical controller plugged in.
    DriverStation.silenceJoystickConnectionWarning(true);

    // Default command: hold the arm in place with 0% duty cycle.
    // This keeps the motor active in BRAKE mode so the arm does not sag between
    // commands. Switching to setAngle(current position) would be more precise but
    // requires knowing the current position at startup before homing is run.
    arm.setDefaultCommand(arm.armCmd(0));
//    arm.setDefaultCommand(arm.setAngle(Degrees.of(0)));

    configureBindings();
  }


  private void configureBindings()
  {
    // Homing example (commented out): run at teleop start to seed the encoder.
    // Adjust the current threshold to match your physical arm's stall current.
    //RobotModeTriggers.teleop().onTrue(arm.homing(Amps.of(20)));

    // Button 1 (A): manual drive up at 50% duty cycle for open-loop testing.
    // Button 2 (B): manual drive down at 50% duty cycle.
    // whileTrue means the command runs while the button is held and cancels on release,
    // which returns control to the default command (0% duty cycle).
    xboxController.button(1).whileTrue(arm.armCmd(0.5));
    xboxController.button(2).whileTrue(arm.armCmd(-0.5));

    // Button 4 (Y): move to 30 deg -- stow / intake position.
    // Button 5 (left bumper): move to 80 deg -- scoring position.
    // These use the exponential profile; the arm accelerates quickly, then
    // settles smoothly without the sharp deceleration edge of a trapezoidal profile.
    // The command holds the setpoint until interrupted (whileTrue), so releasing
    // the button hands off back to the 0% duty-cycle default command.
    xboxController.button(4).whileTrue(arm.setAngle(Degrees.of(30)));
    xboxController.button(5).whileTrue(arm.setAngle(Degrees.of(80)));
  }


  public Command getAutonomousCommand()
  {
    return Commands.print("No autonomous command configured");
  }
}
