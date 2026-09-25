// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;

import first.robot.commands.Drive;
import first.robot.mechanisms.ArmMechanism;
import first.robot.mechanisms.ElevatorMechanism;
import first.robot.mechanisms.ShooterMechanism;
import first.robot.mechanisms.SwerveMechanism;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * AdvantageKit robot using Commands v3.
 *
 * <p>This extends AdvantageKit's {@link LoggedRobot} instead of
 * {@link org.wpilib.framework.OpModeRobot}. AdvantageKit 27.0.0-alpha-5 ships no OpModeRobot
 * variant, {@code OpModeRobot.startCompetition()} is final, and the Logger hooks that wrap every
 * loop ({@code Logger.periodicBeforeUser()/periodicAfterUser()}) are package-private, so an
 * OpModeRobot cannot drive the replay loop. LoggedRobot is an {@code IterativeRobotBase}, the
 * same base as TimedRobot, so this follows WPILib's hatchbotcmdv3 example: Commands v3 run on the
 * default {@link Scheduler} from robotPeriodic() and button bindings are created in the
 * constructor, where they are globally scoped (active in every mode, like the v2 port).
 * Replay (including {@link #setUseTiming(boolean)}) keeps working unchanged.
 */
public class Robot extends LoggedRobot {
  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final Mode simMode     = Mode.SIM;
  /// Change to Mode.REPLAY to enable REPLAy.
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  private final Scheduler scheduler = Scheduler.getDefault();

  // Mechanisms are created after Logger.start() in the constructor so their hardware reads are
  // logged from the first loop.
  private final SwerveMechanism   drive;
  private final ArmMechanism      arm;
  private final ElevatorMechanism elevator;
  private final ShooterMechanism  shooter;

  private final CommandNiDsXboxController xboxController = new CommandNiDsXboxController(0);

  public Robot() {
    switch (currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    drive = new SwerveMechanism();
    arm = new ArmMechanism();
    elevator = new ElevatorMechanism();
    shooter = new ShooterMechanism();

    DriverStationBackend.silenceJoystickConnectionAlert(true);
    // The drive command also drives to these poses while the left or right bumper is held.
    drive.setDefaultCommand(Drive.teleop(drive, xboxController,
                                          new Pose2d(Meters.of(3), Meters.of(3), Rotation2d.fromDegrees(30)),
                                          new Pose2d(Meters.of(5), Meters.of(6), Rotation2d.fromDegrees(70))));
    arm.setDefaultCommand(arm.setAngle(Degrees.of(0)));
    elevator.setDefaultCommand(elevator.setHeight(Meters.of(0)));
    shooter.setDefaultCommand(shooter.set(0));
    configureBindings();
  }

  private void configureBindings()
  {
    xboxController.a().whileTrue(arm.setAngle(Degrees.of(20)));
    xboxController.b().whileTrue(elevator.setHeight(Meters.of(1)));
    xboxController.x().whileTrue(shooter.setVelocity(RPM.of(3000)));
  }

  @Override
  public void robotPeriodic() {
    // Commands v3 Mechanisms have no periodic() hook, so run what the v2 subsystems did in
    // periodic() here, before the scheduler, to match CommandScheduler ordering. Each call
    // updates and processes the AdvantageKit inputs so commands see replayed values.
    drive.periodic();
    arm.periodic();
    elevator.periodic();
    shooter.periodic();
    scheduler.run();
  }

  @Override
  public void simulationPeriodic() {
    drive.simulationPeriodic();
    arm.simulationPeriodic();
    elevator.simulationPeriodic();
    shooter.simulationPeriodic();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    // The v2 port offered no autonomous command.
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void utilityInit() {
    scheduler.cancelAll();
  }

  @Override
  public void utilityPeriodic() {
  }

  @Override
  public void utilityExit() {
  }
}
