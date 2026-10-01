// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.framework.TimedRobot;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.tunable.Tunables;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    private final RobotContainer m_robotContainer;
    private Command m_autonomousCommand;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
        // Instantiate our RobotContainer.  This will perform all our button bindings.
        m_robotContainer = new RobotContainer();
        Tunables.publish("CommandScheduler", CommandScheduler.getInstance());
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** @return The robot container, for tests. */
    RobotContainer getRobotContainer() {
        return m_robotContainer;
    }

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when teleop starts running.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void utilityInit() {
        // Cancels all running commands at the start of utility (formerly test) mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * Simulation starts with an unknown alliance, so the reef poses would not be mirrored. Default to
     * Blue 1; another alliance can still be picked in the simulated driver station.
     */
    @Override
    public void simulationInit() {
        if (DriverStationSim.getAllianceStationId() == AllianceStationID.UNKNOWN) {
            DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_1);
            DriverStationSim.notifyNewData();
        }
    }
}
