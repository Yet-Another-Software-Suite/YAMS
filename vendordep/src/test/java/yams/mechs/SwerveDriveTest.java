// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechs;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicLong;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import yams.exceptions.SwerveDriveConfigurationException;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.helpers.MockHardwareExtension;
import yams.helpers.TestWithScheduler;
import yams.math.AntiTipping;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.GyroAxis;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.config.SwerveModuleConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Tests a simulated four module {@link SwerveDrive} on SPARK MAXes: resetting the odometry, driving
 * to a pose and locking the wheels.
 */
public class SwerveDriveTest {
  /** Subsystem that updates and simulates the drive it holds. */
  private static class SwerveTestSubsystem extends SubsystemBase {
    SwerveDrive drive;

    @Override
    public void periodic() {
      if (drive != null) {
        drive.updateTelemetry();
      }
    }

    @Override
    public void simulationPeriodic() {
      if (drive != null) {
        drive.simIterate();
      }
    }
  }

  private static final Translation2d[] kLocations = {
    new Translation2d(Inches.of(10), Inches.of(10)),
    new Translation2d(Inches.of(10), Inches.of(-10)),
    new Translation2d(Inches.of(-10), Inches.of(10)),
    new Translation2d(Inches.of(-10), Inches.of(-10))
  };
  private static final String[] kNames = {"frontleft", "frontright", "backleft", "backright"};

  private static final int kFirstCanId = 90;

  private static SwerveTestSubsystem subsystem;
  private static final List<SparkMax> sparks = new ArrayList<>();
  private static final SmartMotorController[] drives = new SmartMotorController[4];
  private static final SmartMotorController[] azimuths = new SmartMotorController[4];
  private static int driveCount = 0;

  private SwerveDrive drive;
  private SwerveModule[] modules;

  @BeforeAll
  static void createHardware() {
    MockHardwareExtension.beforeAll();
    subsystem = new SwerveTestSubsystem();
    for (int i = 0; i < 4; i++) {
      // IDs above the ones DeviceCreator hands out, so the other tests keep the IDs they had.
      final SparkMax driveSpark = new SparkMax(kFirstCanId + 2 * i, MotorType.kBrushless);
      final SparkMax azimuthSpark = new SparkMax(kFirstCanId + 2 * i + 1, MotorType.kBrushless);
      sparks.add(driveSpark);
      sparks.add(azimuthSpark);
      drives[i] =
          new SparkWrapper(
              driveSpark,
              DCMotor.getNEO(1),
              new SmartMotorControllerConfig(subsystem)
                  .withWheelDiameter(Inches.of(4))
                  .withClosedLoopController(0.4, 0, 0)
                  .withGearing(new MechanismGearing(GearBox.fromReductionStages(6.75)))
                  .withStatorCurrentLimit(Amps.of(40))
                  .withTelemetry("SwerveDriveTest drive " + i, TelemetryVerbosity.LOW));
      azimuths[i] =
          new SparkWrapper(
              azimuthSpark,
              DCMotor.getNEO(1),
              new SmartMotorControllerConfig(subsystem)
                  .withClosedLoopController(3.8476, 0, 0)
                  .withContinuousWrapping(Radians.of(-Math.PI), Radians.of(Math.PI))
                  .withGearing(new MechanismGearing(GearBox.fromReductionStages(12.8)))
                  .withStatorCurrentLimit(Amps.of(40))
                  .withTelemetry("SwerveDriveTest azimuth " + i, TelemetryVerbosity.LOW));
    }
  }

  @AfterAll
  static void closeHardware() {
    for (int i = 0; i < 4; i++) {
      drives[i].close();
      azimuths[i].close();
    }
    sparks.forEach(SparkMax::close);
    MockHardwareExtension.afterAll();
  }

  /** Build a drive on the shared motors, with every module given the minimum velocity. */
  private void build(boolean minimumVelocity) {
    driveCount++;
    modules = new SwerveModule[4];
    for (int i = 0; i < 4; i++) {
      SwerveModuleConfig config =
          new SwerveModuleConfig(drives[i], azimuths[i])
              .withAbsoluteEncoder(() -> 0.0)
              .withTelemetry(kNames[i] + driveCount, TelemetryVerbosity.LOW)
              .withLocation(kLocations[i])
              .withOptimization(true);
      if (minimumVelocity) {
        config.withMinimumVelocity(MetersPerSecond.of(0.1));
      }
      modules[i] = new SwerveModule(config);
    }
    drive =
        new SwerveDrive(
            new SwerveDriveConfig(subsystem, modules)
                .withMaximumChassisSpeed(MetersPerSecond.of(4.5), DegreesPerSecond.of(540))
                .withStartingPose(new Pose2d())
                .withTranslationController(new PIDController(1.5, 0, 0))
                .withRotationController(new PIDController(4, 0, 0))
                .withTelemetry("SwerveDriveTest" + driveCount, TelemetryVerbosity.LOW));
    subsystem.drive = drive;
  }

  /** Simulated time in microseconds. It only ever goes forward, across tests. */
  private static final AtomicLong clockMicros = new AtomicLong(60_000_000);

  /**
   * Run the scheduler in 20 ms steps. TestWithScheduler restarts the clock from zero on every call,
   * which would make the drive's simulation see time go backward between calls.
   */
  private static void runFor(double seconds) {
    RobotController.setTimeSource(clockMicros::get);
    for (int i = 0; i < Math.round(seconds / 0.02); i++) {
      clockMicros.addAndGet(20_000);
      // Nothing else reads the simulated driver station, so the enabled state would go stale.
      DriverStationSim.notifyNewData();
      DriverStation.refreshData();
      CommandScheduler.getInstance().run();
      SimHooks.stepTimingAsync(0.02);
      try {
        Thread.sleep(2);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      }
    }
  }

  @BeforeEach
  void setUp() {
    // Other test classes disable the simulated driver station when they finish.
    MockHardwareExtension.beforeAll();
    TestWithScheduler.schedulerStart();
    TestWithScheduler.schedulerClear();
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    subsystem.drive = null;
  }

  @Test
  void resetOdometryAlignsTheGyroWithThePose() {
    build(false);
    drive.resetOdometry(new Pose2d(1, 2, Rotation2d.fromDegrees(90)));
    assertEquals(90, drive.getPose().getRotation().getDegrees(), 0.1);
    assertEquals(
        90,
        drive.getGyroAngle().in(Degrees),
        0.1,
        "the gyro should read the pose's heading, so field relative driving agrees with the pose");

    drive.zeroGyro();
    assertEquals(0, drive.getGyroAngle().in(Degrees), 0.1, "zeroGyro should make the gyro read 0");
    assertEquals(0, drive.getPose().getRotation().getDegrees(), 0.1);
    assertEquals(1, drive.getPose().getX(), 0.01, "zeroGyro should keep the position");
    assertEquals(2, drive.getPose().getY(), 0.01, "zeroGyro should keep the position");
  }

  @Test
  void driveToPoseSetpointIsProportionalToTheDistance() {
    build(false);
    drive.resetTranslationPID();
    drive.resetRotationPID();
    // kP of 1.5 and 5 m to go is 7.5 m/s toward the target, along (0.6, 0.8).
    final ChassisSpeeds speeds = drive.driveToPoseSetpoint(new Pose2d(3, 4, new Rotation2d()));
    assertEquals(4.5, speeds.vxMetersPerSecond, 0.01);
    assertEquals(6.0, speeds.vyMetersPerSecond, 0.01);
    assertEquals(0, speeds.omegaRadiansPerSecond, 0.01);
    // At the pose there is nowhere to go.
    final ChassisSpeeds atPose = drive.driveToPoseSetpoint(new Pose2d());
    assertEquals(0, atPose.vxMetersPerSecond, 1e-6);
    assertEquals(0, atPose.vyMetersPerSecond, 1e-6);
  }

  @Test
  void driveToPoseKeepsDrivingUntilInterruptedAndThenStops() {
    build(false);
    final Command command = drive.driveToPose(new Pose2d(30, 0, new Rotation2d()));
    TestWithScheduler.schedule(command);
    runFor(1.5);
    assertTrue(command.isScheduled(), "should run until interrupted");
    final double travelled = drive.getPose().getX();
    assertTrue(travelled > 0.5, "should keep driving toward the pose, but is at " + drive.getPose());
    command.cancel();
    for (SwerveModuleState state : drive.getStateFromRobotRelativeChassisSpeeds(new ChassisSpeeds())) {
      assertEquals(0, state.speedMetersPerSecond, 1e-6);
    }
    runFor(1.5);
    final double afterStop = drive.getPose().getX();
    runFor(0.5);
    assertEquals(afterStop, drive.getPose().getX(), 0.05, "should stop when interrupted");
  }

  @Test
  void lockPoseTurnsTheWheelsEvenWithAMinimumVelocity() {
    build(true);
    final Command lock = subsystem.run(drive::lockPose);
    TestWithScheduler.schedule(lock);
    // The wheels start at 0 degrees, 45 degrees from where the lock puts them.
    for (int waited = 0; waited < 20 && worstLockError() > 5; waited++) {
      runFor(0.5);
    }
    for (int i = 0; i < 4; i++) {
      assertEquals(
          0,
          lockError(i),
          5,
          kNames[i] + " should point along the line to the robot center when locked");
    }
  }

  /** How far module {@code i} is from pointing along the line to the robot center, in degrees. */
  private double lockError(int i) {
    final Angle expected = kLocations[i].getAngle().getMeasure();
    return MathUtil.inputModulus(
        modules[i].getState().angle.getMeasure().minus(expected).in(Degrees), -90, 90);
  }

  private double worstLockError() {
    double worst = 0;
    for (int i = 0; i < 4; i++) {
      worst = Math.max(worst, Math.abs(lockError(i)));
    }
    return worst;
  }

  @Test
  void exponentialProfileConstraintsReachTheMaximumVelocity() {
    final ExponentialProfile.Constraints constraints =
        ExponentialProfilePIDController.createConstraints(
            Volts.of(12), DegreesPerSecond.of(720), DegreesPerSecond.per(edu.wpi.first.units.Units.Second).of(2880));
    // 720 degrees per second is 2 rotations per second.
    assertEquals(2, constraints.maxVelocity(), 1e-6);
  }

  @Test
  void isNearChecksDistanceAndHeading() {
    build(false);
    drive.resetOdometry(new Pose2d(1, 2, Rotation2d.fromDegrees(90)));
    assertTrue(drive.isNear(new Pose2d(1.03, 2, Rotation2d.fromDegrees(92)), Centimeters.of(5), Degrees.of(3)));
    assertFalse(drive.isNear(new Pose2d(1.2, 2, Rotation2d.fromDegrees(90)), Centimeters.of(5), Degrees.of(3)));
    assertFalse(drive.isNear(new Pose2d(1, 2, Rotation2d.fromDegrees(100)), Centimeters.of(5), Degrees.of(3)));
  }

  @Test
  void driveToPoseWithTolerancesEndsAtThePoseAndStops() {
    build(false);
    final Pose2d pose = new Pose2d(0.75, 0, new Rotation2d());
    final Command command = drive.driveToPose(pose, Centimeters.of(10), Degrees.of(5));
    TestWithScheduler.schedule(command);
    runFor(0.1);
    assertTrue(command.isScheduled(), "should still be driving to the pose");
    runFor(6);
    assertFalse(command.isScheduled(), "should end once within the tolerances");
    assertTrue(
        drive.isNear(pose, Centimeters.of(15), Degrees.of(5)),
        "should have ended near the pose, but is at " + drive.getPose());
    final double afterStop = drive.getPose().getX();
    runFor(0.5);
    assertEquals(afterStop, drive.getPose().getX(), 0.05, "should have stopped at the pose");
  }

  @Test
  void theDriveRunsWithoutAutoAlignControllers() {
    final SwerveModule[] plainModules = new SwerveModule[4];
    driveCount++;
    for (int i = 0; i < 4; i++) {
      plainModules[i] =
          new SwerveModule(
              new SwerveModuleConfig(drives[i], azimuths[i])
                  .withAbsoluteEncoder(() -> 0.0)
                  .withTelemetry(kNames[i] + driveCount, TelemetryVerbosity.LOW)
                  .withLocation(kLocations[i]));
    }
    final SwerveDriveConfig config =
        new SwerveDriveConfig(subsystem, plainModules)
            .withTelemetry("SwerveDriveTest" + driveCount, TelemetryVerbosity.HIGH);
    assertFalse(config.hasTranslationPID());
    assertFalse(config.hasRotationPID());
    final SwerveDrive plain = new SwerveDrive(config);
    assertDoesNotThrow(plain::updateTelemetry);
    assertDoesNotThrow(plain::simIterate);
    assertDoesNotThrow(plain::resetTranslationPID);
    assertDoesNotThrow(plain::resetRotationPID);
    assertDoesNotThrow(() -> plain.setRobotRelativeChassisSpeeds(new ChassisSpeeds(1, 0, 0)));
    assertThrows(SwerveDriveConfigurationException.class, () -> plain.driveToPoseSetpoint(new Pose2d()));
  }

  @Test
  void antiTippingCorrectsOnlyPastTheThreshold() {
    final AntiTipping antiTipping = new AntiTipping(2.0, Math.toRadians(10), 0.5);
    final ChassisSpeeds flat = antiTipping.calculate(new Rotation3d(0, Math.toRadians(5), 0));
    assertEquals(0, flat.vxMetersPerSecond, 1e-9, "5 degrees is below the 10 degree threshold");
    // 12 degrees of pitch: 2.0 * sin(12 degrees) is 0.416 m/s, below the 0.5 m/s cap.
    final ChassisSpeeds tilted = antiTipping.calculate(new Rotation3d(0, Math.toRadians(12), 0));
    assertEquals(
        2.0 * Math.sin(Math.toRadians(12)),
        Math.hypot(tilted.vxMetersPerSecond, tilted.vyMetersPerSecond),
        1e-6);
    // Nose down (positive pitch) is falling forward, so the correction drives forward.
    assertTrue(tilted.vxMetersPerSecond > 0, "should drive toward the side the robot is falling to");
    // 40 degrees would be 1.29 m/s, which is capped.
    final ChassisSpeeds capped = antiTipping.calculate(new Rotation3d(Math.toRadians(40), 0, 0));
    assertEquals(0.5, Math.hypot(capped.vxMetersPerSecond, capped.vyMetersPerSecond), 1e-6);
  }

  @Test
  void gyroRotation3dAppliesTheAxisInversionAndOffset() {
    final Rotation3d[] reading = {new Rotation3d(0, 0, Math.toRadians(50))};
    final SwerveDriveConfig config = new SwerveDriveConfig().withGyroRotation3d(() -> reading[0]);
    assertEquals(50, config.getGyroAngle().in(Degrees), 1e-6);
    config.withGyroOffset(Degrees.of(20));
    assertEquals(30, config.getGyroAngle().in(Degrees), 1e-6, "the offset is subtracted");
    assertEquals(30, Math.toDegrees(config.getGyroRotation3d().getZ()), 1e-6);
    config.withGyroInverted(true);
    assertEquals(-70, config.getGyroAngle().in(Degrees), 1e-6, "inverted, then offset");
    // A gyro on its side, with its Y axis up: the robot turning is the pitch of the gyro.
    config.withGyroInverted(false).withGyroOffset(Degrees.of(0)).withGyroHeadingAxis(GyroAxis.PITCH);
    reading[0] = new Rotation3d(0, Math.toRadians(35), 0);
    assertEquals(35, Math.abs(config.getGyroAngle().in(Degrees)), 1e-6, "the pitch of the gyro is the heading");
    // A gyro given as an angle has no tilt.
    final SwerveDriveConfig angleOnly = new SwerveDriveConfig().withGyro(() -> Degrees.of(15));
    assertEquals(15, Math.toDegrees(angleOnly.getGyroRotation3d().getZ()), 1e-6);
    assertEquals(0, angleOnly.getGyroRotation3d().getX(), 1e-9);
  }

  @Test
  void moduleConfigHoldsTheCouplingRatioAndVelocityDeadband() {
    final SwerveModuleConfig config = new SwerveModuleConfig(drives[0], azimuths[0]);
    assertTrue(config.getCouplingRatio().isEmpty());
    config.withCouplingRatio(3.5);
    assertEquals(3.5, config.getCouplingRatio().orElseThrow().getOutputToInputConversionFactor(), 1e-9);
    config.withCouplingRatio(0);
    assertTrue(config.getCouplingRatio().isEmpty(), "0 means no coupling");
    assertTrue(config.getVelocityDeadband().isEmpty());
    config.withVelocityDeadband(MetersPerSecond.of(0.1));
    assertEquals(0.1, config.getVelocityDeadband().orElseThrow().in(MetersPerSecond), 1e-9);
  }

  @Test
  void inputStreamWithARotationAxisTurnsTheRobot() {
    build(false);
    final ChassisSpeeds speeds = SwerveInputStream.of(drive, () -> 0, () -> 0, () -> 0.5).get();
    assertTrue(speeds.omegaRadiansPerSecond > 0, "half stick should ask for rotation");
  }
}
