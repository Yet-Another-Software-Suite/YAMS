// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import first.robot.Constants.OIConstants;
import first.robot.commands.FuelCommands;
import first.robot.mechanisms.ConveyorMechanism;
import first.robot.mechanisms.DriveMechanism;
import first.robot.mechanisms.FeederMechanism;
import first.robot.mechanisms.IntakeMechanism;
import first.robot.mechanisms.ShooterMechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.system.RobotController;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.Tunables;
import yams.commands3.telemetry.CommandTunable;

/**
 * The robot's mechanisms and driver controller live here. The teleop bindings are in
 * {@link first.robot.opmodes.teleop.DefaultTeleop} and the autonomous routine is in
 * {@link first.robot.opmodes.auto.ExampleAuto}; {@link OpModeRobot} finds both by their
 * annotations. If you change the name of this class or the package after creating this project,
 * you must also update the Main.java file in the project.
 */
public class Robot extends OpModeRobot
{
  // The robot's mechanisms are defined here...
  public final DriveMechanism    drive    = new DriveMechanism();
  public final IntakeMechanism   intake   = new IntakeMechanism();
  public final ConveyorMechanism conveyor = new ConveyorMechanism();
  public final ShooterMechanism  shooter  = new ShooterMechanism();
  public final FeederMechanism   feeder   = new FeederMechanism();

  // The driver's controller
  public final CommandNiDsXboxController driverController =
      new CommandNiDsXboxController(OIConstants.kDriverControllerPort);

  private final Scheduler scheduler = Scheduler.getDefault();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot()
  {
    // Dashboard buttons for running individual mechanisms.
    Tunables.publish("Intake", new CommandTunable(FuelCommands.intake(intake, conveyor)));
    Tunables.publish("Extake", new CommandTunable(FuelCommands.extake(intake, conveyor)));

    Tunables.publish("Feeder", new CommandTunable(FuelCommands.feed(shooter, feeder)));
    Tunables.publish("Flywheel", new CommandTunable(shooter.runFlywheel()));

    Tunables.publishDouble("Bat Voltage", RobotController::getBatteryVoltage, voltage -> {});
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and utility.
   */
  @Override
  public void robotPeriodic()
  {
    // Update odometry and YAMS telemetry first so commands work with up-to-date information, the
    // same order the v2 subsystem periodic() methods ran in.
    drive.updateTelemetry();
    intake.updateTelemetry();
    conveyor.updateTelemetry();
    shooter.updateTelemetry();
    feeder.updateTelemetry();

    // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, and removing finished or interrupted commands.
    // This must be called from the robot's periodic block in order for anything in the
    // Command-based framework to work.
    scheduler.run();

    Telemetry.log("Scheduler", scheduler, Scheduler.proto);
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic()
  {
    drive.simIterate();
    intake.simIterate();
    conveyor.simIterate();
    shooter.simIterate();
    feeder.simIterate();
  }
}
