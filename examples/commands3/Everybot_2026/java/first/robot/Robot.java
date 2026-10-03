// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static first.robot.Constants.OperatorConstants.*;

import first.robot.mechanisms.CANDriveMechanism;
import first.robot.mechanisms.ClimberMechanism;
import first.robot.mechanisms.IndexerMechanism;
import first.robot.mechanisms.IntakeLauncherMechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.framework.OpModeRobot;

/**
 * The robot. Holds the mechanisms and controllers, sets their stop default commands, and runs the
 * scheduler. Button bindings live in the teleop opmode ({@code opmodes/teleop}) and the autonomous
 * routine in the autonomous opmode ({@code opmodes/auto}); {@link OpModeRobot} finds both by their
 * annotations.
 */
public class Robot extends OpModeRobot {
  // The robot's mechanisms
  public final CANDriveMechanism drive = new CANDriveMechanism();
  public final IntakeLauncherMechanism intakeLauncher = new IntakeLauncherMechanism();
  public final IndexerMechanism indexer = new IndexerMechanism();
  public final ClimberMechanism climber = new ClimberMechanism();

  // The driver's controller
  public final CommandNiDsXboxController driverController = new CommandNiDsXboxController(DRIVER_CONTROLLER_PORT);

  // The operator's controller, by default it is setup to use a single controller
  public final CommandNiDsXboxController operatorController = new CommandNiDsXboxController(OPERATOR_CONTROLLER_PORT);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Every mechanism stops when nothing else is using it. The teleop opmode replaces the drive
    // default with joystick driving while it is active.
    drive.setDefaultCommand(drive.idle());
    intakeLauncher.setDefaultCommand(intakeLauncher.idle());
    indexer.setDefaultCommand(indexer.idle());
    climber.setDefaultCommand(climber.idle());
  }

  /**
   * This function is called every 20 ms, no matter the mode. Runs the scheduler, then publishes
   * the YAMS telemetry that the subsystems used to publish in {@code periodic()}.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, and removing finished or interrupted
    // commands. This must be called from the robot's periodic block in order for anything in the
    // Command-based framework to work.
    Scheduler.getDefault().run();

    drive.updateTelemetry();
    intakeLauncher.updateTelemetry();
    indexer.updateTelemetry();
    climber.updateTelemetry();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    drive.simIterate();
    intakeLauncher.simIterate();
    indexer.simIterate();
    climber.simIterate();
  }
}
