// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot;

import first.robot.Constants.Shooter.Setpoints;
import first.robot.commands.ShooterCommands;
import first.robot.mechanisms.IndexerMechanism;
import first.robot.mechanisms.ShooterMechanism;
import first.robot.mechanisms.SwerveMechanism;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.math.util.MathUtil;
import org.wpilib.telemetry.Telemetry;

/**
 * Holds the robot's mechanisms, the controllers, and the commands shared by the opmodes in
 * {@code first.robot.opmodes}. OpModeRobot finds the {@code @Teleop} and {@code @Autonomous}
 * opmodes in that package and constructs the one selected on the driver station. This replaces the
 * original {@code RobotContainer}.
 */
public class Robot extends OpModeRobot
{

  public final ShooterMechanism          shooter            = new ShooterMechanism();
  public final IndexerMechanism          indexer            = new IndexerMechanism();
  public final SwerveMechanism           drivebase          = new SwerveMechanism();
  public final CommandNiDsXboxController driverController   = new CommandNiDsXboxController(0);
  public final CommandNiDsXboxController operatorController = new CommandNiDsXboxController(1);
  public final ShooterCommands           shooterCommands    = new ShooterCommands(indexer, shooter);

  private final Scheduler scheduler = Scheduler.getDefault();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code. Default commands set here apply in every opmode. The
   * drivetrain default is set by the teleop opmodes, so autos never read the sticks.
   */
  public Robot()
  {
    shooter.setDefaultCommand(shooter.setVelocityCommand(() -> Setpoints.maxRPM.times(Math.clamp(MathUtil.applyDeadband(
                                                                                                     -operatorController.getRightY(),
                                                                                                     0.1),
                                                                                                 0,
                                                                                                 1))));
    indexer.setDefaultCommand(indexer.idle());

    // if (isSimulation())
    // {
    DriverStationBackend.silenceJoystickConnectionAlert(true);
    // }
  }

  /**
   * This function is called every 20 ms, no matter the mode. The mechanism updates run first, as v2
   * subsystem periodic() methods did, then the scheduler.
   */
  @Override
  public void robotPeriodic()
  {
    shooter.periodic();
    indexer.periodic();
    drivebase.periodic();

    scheduler.run();
    Telemetry.log("Scheduler", scheduler, Scheduler.proto);
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic()
  {
    shooter.simulationPeriodic();
    indexer.simulationPeriodic();
    drivebase.simulationPeriodic();
  }
}
