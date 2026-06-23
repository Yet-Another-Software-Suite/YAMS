// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DiffyMechSubsystem;

import static edu.wpi.first.units.Units.Degrees;

/**
 * Wires the differential mechanism subsystem to driver inputs.
 *
 * <p>Button layout demonstrates both control modes available on DiffyMechSubsystem:
 * <ul>
 *   <li>Closed-loop angle commands (buttons 1, 2) -- good for repeatable scored positions.</li>
 *   <li>Open-loop duty-cycle commands (buttons 3-5) -- useful for manual tuning or driver feel.</li>
 * </ul>
 *
 * <p>The default command holds the mechanism at zero duty cycle so the mechanism
 * is not back-driven when no button is pressed.  Switching to the commented-out
 * setAngle default would hold a fixed angle with PID once real feedforward is tuned.
 */
public class RobotContainer
{
  private DiffyMechSubsystem diffyMech = new DiffyMechSubsystem();
  // Controller port 0 -- primary driver.
  public CommandXboxController xboxController = new CommandXboxController(0);

  public RobotContainer()
  {
    // Suppress the "joystick not connected" DS warning during sim/bench testing.
    DriverStation.silenceJoystickConnectionWarning(true);
    // Open-loop zero keeps the motors energized in brake mode without a position target,
    // which prevents the wrist from sagging when no button is held.
    diffyMech.setDefaultCommand(diffyMech.set(0, 0));
//    diffyMech.setDefaultCommand(diffyMech.setAngle(Degrees.of(0), Degrees.of(0)));
    configureBindings();
  }


  private void configureBindings()
  {
    // Button 1: closed-loop to tilt=15 deg, twist=15 deg -- exercises both DOFs together
    //           to verify the differential kinematics decompose correctly.
    xboxController.button(1).whileTrue(diffyMech.setAngle(Degrees.of(15), Degrees.of(15)));

    // Button 2: larger setpoint to confirm profile limits (180 deg/s, 90 deg/s^2) are respected.
    xboxController.button(2).whileTrue(diffyMech.setAngle(Degrees.of(30), Degrees.of(45)));

    // Button 3: open-loop zero -- explicit stop in case the default command is swapped out.
    xboxController.button(3).whileTrue(diffyMech.set(0, 0));

    // Button 4: 50% tilt only -- isolates the same-direction motor coupling (both motors forward).
    xboxController.button(4).whileTrue(diffyMech.set(0.5, 0));

    // Button 5: 50% twist only -- isolates the opposing-direction coupling (motors contra-rotate).
    xboxController.button(5).whileTrue(diffyMech.set(0, 0.5));
  }


  public Command getAutonomousCommand()
  {
    return Commands.print("No autonomous command configured");
  }
}
