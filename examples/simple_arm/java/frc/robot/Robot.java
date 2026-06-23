// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Top-level robot class for the simple_arm example.
 *
 * <p>Follows the standard WPILib TimedRobot pattern: robotPeriodic() runs every
 * 20 ms and ticks the CommandScheduler, which dispatches all active commands
 * including the arm's default duty-cycle command and any button-triggered
 * setAngle commands.
 */
public class Robot extends TimedRobot
{
    // Holds a reference so autonomousInit() can cancel auto when teleop starts.
    private Command autonomousCommand;

    // robotContainer is public so simulation harnesses can reach subsystems directly.
    public final RobotContainer robotContainer;

    public Robot()
    {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic()
    {
        // Single point of execution for all scheduled commands.
        // Must run every loop even in disabled mode so telemetry keeps updating.
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
        // Cancel auto command when driver takes control; prevents auto from
        // continuing to run the arm into an unintended position.
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
        // Cancel everything before test mode so no stale commands carry over.
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
