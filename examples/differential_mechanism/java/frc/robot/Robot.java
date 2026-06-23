// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Top-level robot class for the differential mechanism example.
 *
 * <p>Follows the standard WPILib TimedRobot pattern.  All subsystem and command
 * setup lives in RobotContainer; this class only owns the lifecycle hooks and
 * drives the CommandScheduler loop.
 */
public class Robot extends TimedRobot
{
    private Command autonomousCommand;

    public final RobotContainer robotContainer;


    public Robot()
    {
        robotContainer = new RobotContainer();
    }


    @Override
    public void robotPeriodic()
    {
        // CommandScheduler.run() must be called every loop iteration (20 ms).
        // It polls buttons, executes scheduled commands, and calls subsystem periodic().
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
        // Cancel any autonomous command that is still running when tele-op starts.
        // Without this the auto command could keep driving motors while the driver has control.
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
        // Clear all running commands so test mode starts from a known state.
        CommandScheduler.getInstance().cancelAll();
    }


    @Override
    public void testPeriodic() {}


    @Override
    public void testExit() {}
}
