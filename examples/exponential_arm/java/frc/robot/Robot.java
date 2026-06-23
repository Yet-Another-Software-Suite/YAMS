// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * Top-level robot class. TimedRobot calls each *Init() method once on mode
 * entry and each *Periodic() method at 50 Hz (every 20 ms) while that mode is
 * active. All subsystem logic lives in RobotContainer and its subsystems;
 * Robot.java is purely the WPILib entry point.
 */
public class Robot extends TimedRobot
{
    // Holds the scheduled autonomous command so it can be cancelled on teleop entry.
    private Command autonomousCommand;

    // RobotContainer is public so simulation harnesses can reach subsystems directly.
    public final RobotContainer robotContainer;


    public Robot()
    {
        // RobotContainer wires up subsystems, default commands, and button bindings.
        robotContainer = new RobotContainer();
    }


    @Override
    public void robotPeriodic()
    {
        // CommandScheduler.run() must be called every loop cycle. It processes the
        // command queue, evaluates triggers, and calls subsystem periodic() methods.
        // Removing this call will break all command-based behavior.
        CommandScheduler.getInstance().run();
    }


    @Override
    public void disabledInit() {}


    @Override
    public void disabledPeriodic() {}


    @Override
    public void disabledExit() {}


    @Override
    public void autonomousInit()
    {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // Only schedule if the container returned a real command.
        // Scheduling null would throw a NullPointerException.
        if (autonomousCommand != null)
        {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }


    @Override
    public void autonomousPeriodic() {}


    @Override
    public void autonomousExit() {}


    @Override
    public void teleopInit()
    {
        // Cancel the autonomous command so its subsystem requirements are released
        // before the driver starts issuing teleop commands. Without this, the arm
        // subsystem would stay locked to the auto command and ignore button presses.
        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        }
    }


    @Override
    public void teleopPeriodic() {}


    @Override
    public void teleopExit() {}


    @Override
    public void testInit()
    {
        // Cancel every running command when entering test mode so a prior teleop or
        // auto command does not interfere with test-mode routines.
        CommandScheduler.getInstance().cancelAll();
    }


    @Override
    public void testPeriodic() {}


    @Override
    public void testExit() {}
}
