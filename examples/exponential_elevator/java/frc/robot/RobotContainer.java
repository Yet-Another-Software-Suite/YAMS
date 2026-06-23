// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ExponentiallyProfiledElevatorSubsystem;

import static edu.wpi.first.units.Units.Meters;


public class RobotContainer
{
  // Elevator subsystem owns the motor and profile controller.
  public ExponentiallyProfiledElevatorSubsystem elevator = new ExponentiallyProfiledElevatorSubsystem();
  // Port 0 is the first USB gamepad plugged into the Driver Station.
  public CommandXboxController xboxController = new CommandXboxController(0);

  public RobotContainer()
  {
    // Suppress the "joystick not connected" alert in simulation so it does not flood the DS log.
    DriverStation.silenceJoystickConnectionWarning(true);
    // elevCmd(0) holds duty cycle at zero, acting as a safe idle when no other command is scheduled.
    elevator.setDefaultCommand(elevator.elevCmd(0));
    configureBindings();
  }


  private void configureBindings()
  {
    // Homing is commented out here as a reminder of where to wire it in.
    // Uncomment and tune the current threshold (Amps) before competition.
    //RobotModeTriggers.teleop().onTrue(elevator.homing(Amps.of(1)));

    // Button 1 (A): move to 1 m -- scoring height.
    // whileTrue re-commands the profile every loop, so releasing the button drops back to the default.
    xboxController.button(1).whileTrue(elevator.setHeight(Meters.of(1)));
    // Button 2 (B): retract to 0 m -- stowed / intake height.
    xboxController.button(2).whileTrue(elevator.setHeight(Meters.of(0)));
    // Buttons 4 and 5 are manual override bindings for tuning; remove before competition.
    // Negative duty cycle drives the carriage down.
    xboxController.button(4).whileTrue(elevator.elevCmd(-0.5));
    // Positive duty cycle drives the carriage up.
    xboxController.button(5).whileTrue(elevator.elevCmd(0.5));
  }


  public Command getAutonomousCommand()
  {
    return Commands.print("No autonomous command configured");
  }
}
