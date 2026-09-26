// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import first.robot.commands.Drive;
import first.robot.mechanisms.ArmMechanism;
import first.robot.mechanisms.ArmMechanism.ArmConstants;
import first.robot.mechanisms.ElevatorMechanism;
import first.robot.mechanisms.IndexerMechanism;
import first.robot.mechanisms.ShooterMechanism;
import first.robot.mechanisms.SwerveMechanism;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;

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
 *
 * <p>Commands that use more than one mechanism ({@link #shoot}, {@link #scorePreloadAuto}) have no
 * requirements of their own and run each mechanism's command as a child, so a mechanism is only
 * owned while its child command runs and its default command resumes afterwards.
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
  private final IndexerMechanism  indexer;

  // Setpoints shared by the teleop bindings and the autonomous routine.
  private static final Angle           SCORE_ANGLE       = ArmConstants.SOME_ANGLE;
  private static final Distance        SCORE_HEIGHT      = Meters.of(1);
  private static final AngularVelocity SHOOT_SPEED       = RPM.of(3000);
  private static final AngularVelocity SHOOTER_TOLERANCE = RPM.of(100);
  private static final Angle           ARM_TOLERANCE     = Degrees.of(3);
  private static final Distance        HEIGHT_TOLERANCE  = Meters.of(0.05);

  private final Command autonomousCommand;

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
    indexer = new IndexerMechanism();

    DriverStationBackend.silenceJoystickConnectionAlert(true);
    // The drive command also drives to these poses while the left or right bumper is held.
    drive.setDefaultCommand(Drive.teleop(drive, xboxController,
                                          new Pose2d(Meters.of(3), Meters.of(3), Rotation2d.fromDegrees(30)),
                                          new Pose2d(Meters.of(5), Meters.of(6), Rotation2d.fromDegrees(70))));
    arm.setDefaultCommand(arm.setAngle(Degrees.of(0)));
    elevator.setDefaultCommand(elevator.setHeight(Meters.of(0)));
    shooter.setDefaultCommand(shooter.set(0));
    indexer.setDefaultCommand(indexer.idle());
    autonomousCommand = scorePreloadAuto();
    configureBindings();
  }

  private void configureBindings()
  {
    xboxController.a().whileTrue(arm.setAngle(SCORE_ANGLE));
    xboxController.b().whileTrue(elevator.setHeight(SCORE_HEIGHT));
    xboxController.x().whileTrue(shooter.setVelocity(SHOOT_SPEED));
    xboxController.rightTrigger().whileTrue(shoot(SHOOT_SPEED));
  }

  /**
   * Spin the shooter up and feed whenever it is at speed. The feed binding is created inside this
   * command, so it only exists while shooting; if the wheel drops out of tolerance after a shot,
   * feeding pauses until it recovers. Runs until canceled.
   *
   * @param speed Shooter speed.
   * @return {@link Command}
   */
  private Command shoot(AngularVelocity speed)
  {
    return Command.noRequirements(coroutine -> {
      coroutine.fork(shooter.setVelocity(speed));
      shooter.atSpeed(speed, SHOOTER_TOLERANCE).whileTrue(indexer.feed());
      coroutine.park();
    }).named("Shoot");
  }

  /**
   * Raise the arm and elevator to the scoring position, then shoot for two seconds. The arm and
   * elevator commands are forked so they keep holding the scoring position during the shot; they are
   * canceled (and the default commands resume) when the routine ends.
   *
   * @return {@link Command}
   */
  private Command scorePreloadAuto()
  {
    return Command.noRequirements(coroutine -> {
      coroutine.fork(arm.setAngle(SCORE_ANGLE), elevator.setHeight(SCORE_HEIGHT));
      // Give up waiting after two seconds so a mechanism that never settles cannot stall the auto.
      coroutine.waitUntil(arm.near(SCORE_ANGLE, ARM_TOLERANCE).and(elevator.near(SCORE_HEIGHT, HEIGHT_TOLERANCE)),
                          Seconds.of(2));
      coroutine.awaitAny(shoot(SHOOT_SPEED), Command.waitFor(Seconds.of(2)).named("Shot Time"));
    }).named("Score Preload Auto");
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
    indexer.periodic();
    scheduler.run();
  }

  @Override
  public void simulationPeriodic() {
    drive.simulationPeriodic();
    arm.simulationPeriodic();
    elevator.simulationPeriodic();
    shooter.simulationPeriodic();
    indexer.simulationPeriodic();
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
    // Scheduled from the global scope, so it is canceled by hand when autonomous ends.
    scheduler.schedule(autonomousCommand);
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
    scheduler.cancel(autonomousCommand);
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
