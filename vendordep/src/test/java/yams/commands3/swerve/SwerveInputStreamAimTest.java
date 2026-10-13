// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands3.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.system.DCMotor;
import org.wpilib.preferences.Preferences;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.config.SwerveDriveConfig;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.SwerveModuleConfig;
import yams.core.mechanisms.swerve.SwerveModule;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;

/**
 * Tests the aim options of {@link SwerveInputStream}: {@link SwerveInputStream#withAimLookahead},
 * {@link SwerveInputStream#withAimFeedforward}, {@link SwerveInputStream#withAimHeadingOffset} and
 * {@link SwerveInputStream#isAimLocked}. A Commands v3 trigger only updates when the scheduler polls it, so the
 * lock is read directly here.
 *
 * <p>The drive is a simulated four module drive with a proportional rotation controller of 1, so the angular velocity
 * it asks for is the heading error in radians. Its pose, heading and velocity are set by each test.
 */
public class SwerveInputStreamAimTest {
  private static final double kTolerance = 1e-9;

  /** Mechanism the drive's motor controllers belong to. */
  private static final Mechanism kMechanism = new Mechanism() {};

  private static final List<SmartMotorController> motorControllers = new ArrayList<>();
  private static SwerveDrive drive;

  /** Pose the drive reports. */
  private static Pose2d pose = new Pose2d();

  /** Field relative velocity the drive reports. */
  private static ChassisVelocities fieldVelocity = new ChassisVelocities();

  /** Whether the stream is aiming. */
  private boolean aiming;

  private static SwerveModule createModule(String name, double frontInches, double leftInches) {
    SmartMotorControllerConfig driveCfg =
        new SmartMotorControllerConfig(kMechanism)
            .withWheelDiameter(Inches.of(4))
            .withClosedLoopController(0.4, 0, 0)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(6.75)))
            .withTelemetry(name + "Drive", TelemetryVerbosity.LOW);
    SmartMotorControllerConfig azimuthCfg =
        new SmartMotorControllerConfig(kMechanism)
            .withClosedLoopController(50, 0, 0.5)
            .withContinuousWrapping(Radians.of(-Math.PI), Radians.of(Math.PI))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(12.8)))
            .withTelemetry(name + "Azimuth", TelemetryVerbosity.LOW);
    SmartMotorController driveMotor =
        new SparkWrapper(DeviceCreator.createSparkMax(), DCMotor.getNEO(1), driveCfg);
    SmartMotorController azimuthMotor =
        new SparkWrapper(DeviceCreator.createSparkMax(), DCMotor.getNEO(1), azimuthCfg);
    motorControllers.add(driveMotor);
    motorControllers.add(azimuthMotor);
    return new SwerveModule(
        new SwerveModuleConfig(driveMotor, azimuthMotor)
            .withAbsoluteEncoder(() -> Degrees.of(0))
            .withLocation(new Translation2d(Inches.of(frontInches), Inches.of(leftInches)))
            .withTelemetry(name, TelemetryVerbosity.LOW));
  }

  @BeforeAll
  static void createDrive() {
    MockHardwareExtension.beforeAll();
    drive =
        new SwerveDrive(
            new SwerveDriveConfig(
                    kMechanism,
                    createModule("SISAimTestFL", 12, 12),
                    createModule("SISAimTestFR", 12, -12),
                    createModule("SISAimTestBL", -12, 12),
                    createModule("SISAimTestBR", -12, -12))
                .withGyro(Rotation3d::new)
                .withStartingPose(new Pose2d())
                .withMaximumChassisSpeed(MetersPerSecond.of(4.5), DegreesPerSecond.of(540))
                .withRotationController(new PIDController(1, 0, 0))
                .withTelemetry("SwerveInputStreamAimTest", TelemetryVerbosity.LOW)) {
          @Override
          public Pose2d getPose() {
            return pose;
          }

          @Override
          public Rotation3d getGyroRotation3d() {
            return new Rotation3d(pose.getRotation());
          }

          @Override
          public ChassisVelocities getFieldRelativeSpeed() {
            return fieldVelocity;
          }

          @Override
          public ChassisVelocities getRobotRelativeSpeed() {
            return fieldVelocity.toRobotRelative(pose.getRotation());
          }
        };
  }

  @AfterAll
  static void closeDrive() {
    for (SmartMotorController smc : motorControllers) {
      smc.close();
      DeviceCreator.silence(smc);
    }
    motorControllers.clear();
    drive = null;
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  @BeforeEach
  void setUp() {
    MockHardwareExtension.beforeAll();
    pose = new Pose2d();
    fieldVelocity = new ChassisVelocities();
    aiming = true;
  }

  @AfterEach
  void tearDown() {
    MockHardwareExtension.afterAll();
  }

  /** A stream with centered sticks that aims at a field position while {@link #aiming} is true. */
  private SwerveInputStream aimAt(double x, double y) {
    return SwerveInputStream.of(drive, () -> 0, () -> 0, () -> 0)
        .withAim(() -> new Pose2d(x, y, new Rotation2d()), () -> aiming);
  }

  @Test
  void aimTurnsTowardTheTarget() {
    // The target is straight to the left, so the heading error is 90 degrees.
    assertEquals(Math.PI / 2, aimAt(0, 2).get().omega, kTolerance);
    // From a robot that already faces left, the same target needs no turn.
    pose = new Pose2d(0, 0, Rotation2d.fromDegrees(90));
    assertEquals(0, aimAt(0, 2).get().omega, kTolerance);
  }

  @Test
  void targetVectorIsFromTheRobotToTheTarget() {
    pose = new Pose2d(1, 1, Rotation2d.fromDegrees(45));
    Translation2d vector = aimAt(0, 0).getTargetVector(new Pose2d(3, 4, new Rotation2d()));
    assertEquals(2, vector.getX(), kTolerance);
    assertEquals(3, vector.getY(), kTolerance);
  }

  @Test
  void lookaheadMovesTheRobotAlongItsFieldVelocity() {
    fieldVelocity = new ChassisVelocities(2, 0, 0);
    SwerveInputStream stream = aimAt(1, 2).withAimLookahead(Seconds.of(0.5));
    // In half a second the robot is 1 m further along X, directly beside the target.
    Translation2d vector = stream.getTargetVector(new Pose2d(1, 2, new Rotation2d()));
    assertEquals(0, vector.getX(), kTolerance);
    assertEquals(2, vector.getY(), kTolerance);
    assertEquals(Math.PI / 2, stream.get().omega, kTolerance);
    // Without a lookahead the stream aims from where the robot is now.
    assertEquals(Math.atan2(2, 1), aimAt(1, 2).get().omega, kTolerance);
  }

  @Test
  void lookaheadIsFieldRelativeWhateverTheRobotFaces() {
    pose = new Pose2d(0, 0, Rotation2d.fromDegrees(90));
    fieldVelocity = new ChassisVelocities(2, 0, 0);
    Translation2d vector =
        aimAt(1, 2).withAimLookahead(Seconds.of(0.5)).getTargetVector(new Pose2d(1, 2, new Rotation2d()));
    assertEquals(0, vector.getX(), kTolerance);
    assertEquals(2, vector.getY(), kTolerance);
  }

  @Test
  void headingOffsetRotatesTheAimHeading() {
    // The target is straight ahead, so only the offset is left to turn.
    assertEquals(Math.PI / 2, aimAt(2, 0).withAimHeadingOffset(Rotation2d.fromDegrees(90)).get().omega, kTolerance);
    assertEquals(0, aimAt(2, 0).withAimHeadingOffset(Rotation2d.fromDegrees(90), () -> false).get().omega, kTolerance);
  }

  @Test
  void feedforwardIsAddedToTheRotation() {
    // kV of 0.5 on the controller's pi / 2 rad/s adds another pi / 4.
    SwerveInputStream stream = aimAt(0, 2).withAimFeedforward(new SimpleMotorFeedforward(0, 0.5, 0));
    assertEquals(Math.PI / 2 * 1.5, stream.get().omega, kTolerance);
    assertEquals(
        Math.PI / 2 * 1.5, stream.calculateAngularVelocity(Radians.of(Math.PI / 2)).in(RadiansPerSecond), kTolerance);
  }

  @Test
  void aimLockIsTrueOnlyWhileAimingAndFacingTheTarget() {
    SwerveInputStream ahead = aimAt(2, 0);
    assertFalse(ahead.isAimLocked(Degrees.of(1)), "Not locked before the stream is read");
    ahead.get();
    assertTrue(ahead.isAimLocked(Degrees.of(1)));

    SwerveInputStream left = aimAt(0, 2);
    left.get();
    assertFalse(left.isAimLocked(Degrees.of(1)));
    assertTrue(left.isAimLocked(Degrees.of(91)));

    aiming = false;
    ahead.get();
    assertFalse(ahead.isAimLocked(Degrees.of(1)), "Not locked once aiming stops");
  }

  @Test
  void cloneKeepsTheAimOptions() {
    fieldVelocity = new ChassisVelocities(2, 0, 0);
    SwerveInputStream clone =
        aimAt(1, 2)
            .withAimLookahead(Seconds.of(0.5))
            .withAimHeadingOffset(Rotation2d.fromDegrees(-90))
            .withAimFeedforward(new SimpleMotorFeedforward(0, 0.5, 0))
            .clone();
    // Lookahead puts the target at 90 degrees, and the offset takes that back to 0.
    assertEquals(0, clone.get().omega, kTolerance);
    assertEquals(
        Math.PI / 2 * 1.5, clone.calculateAngularVelocity(Radians.of(Math.PI / 2)).in(RadiansPerSecond), kTolerance);
  }
}
