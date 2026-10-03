// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static first.robot.Constants.OperatorConstants.*;

import first.robot.commands.FuelCommands;
import first.robot.mechanisms.CANDriveMechanism;
import first.robot.mechanisms.FeederMechanism;
import first.robot.mechanisms.IntakeLauncherMechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.framework.OpModeRobot;

/**
 * The robot. Holds the mechanisms, the fuel commands and the controllers. Teleop bindings live in
 * {@link first.robot.opmodes.teleop.KitBotTeleop} and the autonomous routine in
 * {@link first.robot.opmodes.auto.ExampleAuto}; {@link OpModeRobot} finds both through their
 * annotations. If you change the name of this class or the package after creating this project,
 * you must also update the Main.java file in the project.
 */
public class Robot extends OpModeRobot {
  // The robot's mechanisms
  public final CANDriveMechanism drive = new CANDriveMechanism();
  public final FeederMechanism feeder = new FeederMechanism();
  public final IntakeLauncherMechanism intakeLauncher = new IntakeLauncherMechanism();
  // Fuel actions that run both rollers together
  public final FuelCommands fuel = new FuelCommands(feeder, intakeLauncher);

  // The driver's controller
  public final CommandNiDsXboxController driverController = new CommandNiDsXboxController(
      DRIVER_CONTROLLER_PORT);

  // The operator's controller
  public final CommandNiDsXboxController operatorController = new CommandNiDsXboxController(
      OPERATOR_CONTROLLER_PORT);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Keep the drive stopped whenever nothing else is driving it. The teleop opmode replaces this
    // with joystick arcade drive while it is running.
    drive.setDefaultCommand(drive.idle());
    // Stop each roller whenever no fuel command is using it.
    feeder.setDefaultCommand(feeder.idle());
    intakeLauncher.setDefaultCommand(intakeLauncher.idle());
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and utility.
   */
  @Override
  public void robotPeriodic() {
    // Runs the scheduler. This is responsible for polling buttons, running scheduled commands,
    // and removing finished or interrupted commands. This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    Scheduler.getDefault().run();

    // Publish YAMS telemetry for every mechanism.
    drive.periodic();
    feeder.periodic();
    intakeLauncher.periodic();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    drive.simulationPeriodic();
    feeder.simulationPeriodic();
    intakeLauncher.simulationPeriodic();
  }
}
