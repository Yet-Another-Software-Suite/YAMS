// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radians;

import com.revrobotics.spark.SparkMax;
import java.util.ArrayList;
import java.util.List;
import java.util.SplittableRandom;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.system.DCMotor;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.preferences.Preferences;
import org.wpilib.system.Timer;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.config.SwerveDriveConfig;
import yams.commands2.swerve.SwerveDrive;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.SwerveModuleConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.SwerveDriveTelemetry;
import yams.core.telemetry.SwerveDriveTelemetryConfig;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.TestWithScheduler;

/**
 * Unit, seeded fuzz, and {@link SwerveDrive}-level integration tests for {@link SkidDetector},
 * {@link TiltOdometryScaler}, tilt-gated vision updates, and skid/tilt telemetry gating.
 */
class SwerveSkidAndTiltOdometryTest {
  private static final double EPS = 1e-6;
  private static final double HALF_TRACK_METERS = 0.30;
  private static final double MODULE_OFFSET_INCHES = 12.0;
  private static final double DRIVE_REDUCTION = 6.75;
  private static final double AZIMUTH_REDUCTION = 12.8;
  private static final double NOMINAL_VOLTAGE = 12.0;

  private static final Translation2d[] SQUARE_LOCATIONS =
      new Translation2d[] {
        new Translation2d(HALF_TRACK_METERS, HALF_TRACK_METERS),
        new Translation2d(HALF_TRACK_METERS, -HALF_TRACK_METERS),
        new Translation2d(-HALF_TRACK_METERS, HALF_TRACK_METERS),
        new Translation2d(-HALF_TRACK_METERS, -HALF_TRACK_METERS)
      };

  private static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(SQUARE_LOCATIONS);

  /** Lightweight shared SparkMax hardware fixture for SwerveDrive integration tests. */
  private static final class TestHardware {
    final SubsystemBase subsystem = new SubsystemBase() {};
    final List<SmartMotorController> controllers = new ArrayList<>();
    final SwerveModule[] modules;

    TestHardware() {
      modules =
          new SwerveModule[] {
            createModule("SkidFL", MODULE_OFFSET_INCHES, MODULE_OFFSET_INCHES),
            createModule("SkidFR", MODULE_OFFSET_INCHES, -MODULE_OFFSET_INCHES),
            createModule("SkidBL", -MODULE_OFFSET_INCHES, MODULE_OFFSET_INCHES),
            createModule("SkidBR", -MODULE_OFFSET_INCHES, -MODULE_OFFSET_INCHES)
          };
    }

    private SwerveModule createModule(String name, double xInches, double yInches) {
      double driveKv =
          NOMINAL_VOLTAGE / (DCMotor.getNEO(1).freeSpeed / (2.0 * Math.PI) / DRIVE_REDUCTION);
      SmartMotorControllerConfig driveCfg =
          new SmartMotorControllerConfig(subsystem)
              .withWheelDiameter(Inches.of(4))
              .withClosedLoopController(0.4, 0, 0)
              .withFeedforward(new SimpleMotorFeedforward(0, driveKv))
              .withGearing(new MechanismGearing(GearBox.fromReductionStages(DRIVE_REDUCTION)))
              .withStatorCurrentLimit(Amps.of(40))
              .withTelemetry(name + "Drive", TelemetryVerbosity.LOW);
      SmartMotorControllerConfig azimuthCfg =
          new SmartMotorControllerConfig(subsystem)
              .withClosedLoopController(50.0, 0, 0.5)
              .withContinuousWrapping(Radians.of(-Math.PI), Radians.of(Math.PI))
              .withGearing(new MechanismGearing(GearBox.fromReductionStages(AZIMUTH_REDUCTION)))
              .withStatorCurrentLimit(Amps.of(40))
              .withTelemetry(name + "Azimuth", TelemetryVerbosity.LOW);

      SmartMotorController driveSmc =
          new SparkWrapper(DeviceCreator.createSparkMax(), DCMotor.getNEO(1), driveCfg);
      SmartMotorController azimuthSmc =
          new SparkWrapper(DeviceCreator.createSparkMax(), DCMotor.getNEO(1), azimuthCfg);
      controllers.add(driveSmc);
      controllers.add(azimuthSmc);

      SwerveModuleConfig moduleCfg =
          new SwerveModuleConfig(driveSmc, azimuthSmc)
              .withAbsoluteEncoder(() -> Degrees.of(0))
              .withLocation(new Translation2d(Inches.of(xInches), Inches.of(yInches)))
              .withOptimization(true)
              .withCosineCompensation(true)
              .withTelemetry(name, TelemetryVerbosity.LOW);
      return new SwerveModule(moduleCfg);
    }

    void close() {
      CommandScheduler.getInstance().unregisterSubsystem(subsystem);
      for (SmartMotorController smc : controllers) {
        smc.close();
        DeviceCreator.silence(smc);
        if (smc.getMotorController() instanceof SparkMax spark) {
          spark.close();
        }
      }
    }
  }

  /** SwerveDrive subclass allowing deterministic 3D gyro tilt injection in simulation tests. */
  private static final class TiltingTestSwerveDrive extends SwerveDrive {
    private final AtomicReference<Rotation3d> m_attitude;

    TiltingTestSwerveDrive(SwerveDriveConfig config, AtomicReference<Rotation3d> attitude) {
      super(config);
      m_attitude = attitude;
    }

    @Override
    public Rotation3d getGyroRotation3d() {
      return m_attitude != null ? m_attitude.get() : Rotation3d.ZERO;
    }
  }

  private static final AtomicInteger DRIVE_COUNTER = new AtomicInteger();
  private static TestHardware sharedHardware;

  private static TestHardware hardware() {
    if (sharedHardware == null) {
      sharedHardware = new TestHardware();
    }
    return sharedHardware;
  }

  @BeforeEach
  void setUp() {
    MockHardwareExtension.beforeAll();
    TestWithScheduler.schedulerStart();
    TestWithScheduler.schedulerClear();
  }

  @AfterEach
  void tearDown() {
    TestWithScheduler.schedulerClear();
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  @AfterAll
  static void tearDownAll() {
    if (sharedHardware != null) {
      sharedHardware.close();
      sharedHardware = null;
    }
  }

  private static SwerveDriveConfig baseDriveConfig(TelemetryVerbosity verbosity) {
    TestHardware hw = hardware();
    return new SwerveDriveConfig(hw.subsystem, hw.modules)
        .withGyro(() -> Rotation3d.ZERO)
        .withStartingPose(new Pose2d())
        .withMaximumChassisSpeed(MetersPerSecond.of(4.5), DegreesPerSecond.of(540))
        .withTelemetry("SkidTiltTest" + DRIVE_COUNTER.incrementAndGet(), verbosity);
  }

  // ---- SkidDetector unit tests ---------------------------------------------------------------

  @Test
  void skidDetectorReportsNominalWhenStationaryOrPurelyTranslating() {
    SkidDetector detector = new SkidDetector(1.35);

    SwerveModuleVelocity[] stopped =
        KINEMATICS.toSwerveModuleVelocities(new ChassisVelocities(0.0, 0.0, 0.0));
    SkidDetector.SkidState stopState = detector.calculate(stopped, SQUARE_LOCATIONS, 0.0);
    assertEquals(1.0, stopState.skidRatio(), EPS);
    assertFalse(stopState.skidding());
    assertEquals(SkidDetector.NO_SLIPPING_MODULES, stopState.slippingModuleMask());

    SwerveModuleVelocity[] straight =
        KINEMATICS.toSwerveModuleVelocities(new ChassisVelocities(2.5, -1.2, 0.0));
    SkidDetector.SkidState straightState = detector.calculate(straight, SQUARE_LOCATIONS, 0.0);
    assertEquals(1.0, straightState.skidRatio(), 1e-4);
    assertFalse(straightState.skidding());
    assertEquals(SkidDetector.NO_SLIPPING_MODULES, straightState.slippingModuleMask());
  }

  @Test
  void skidDetectorReportsNominalDuringPureSpinAndCombinedMotion() {
    SkidDetector detector = new SkidDetector(1.35);

    double pureOmega = 3.0;
    SwerveModuleVelocity[] spin =
        KINEMATICS.toSwerveModuleVelocities(new ChassisVelocities(0.0, 0.0, pureOmega));
    SkidDetector.SkidState spinState = detector.calculate(spin, SQUARE_LOCATIONS, pureOmega);
    assertEquals(1.0, spinState.skidRatio(), EPS);
    assertFalse(spinState.skidding());

    double comboOmega = -2.2;
    SwerveModuleVelocity[] combo =
        KINEMATICS.toSwerveModuleVelocities(new ChassisVelocities(2.0, 1.5, comboOmega));
    SkidDetector.SkidState comboState = detector.calculate(combo, SQUARE_LOCATIONS, comboOmega);
    assertEquals(1.0, comboState.skidRatio(), 1e-4);
    assertFalse(comboState.skidding());
  }

  @Test
  void skidDetectorSuppressesLowSpeedNoiseSpreadBelowDeadband() {
    SkidDetector detector = new SkidDetector(1.35, MetersPerSecond.of(0.10));
    // 0.12 / 0.08 = 1.50 > 1.35, but spread is only 0.04 m/s (< 0.10 m/s deadband).
    SwerveModuleVelocity[] noisyCrawl =
        new SwerveModuleVelocity[] {
          new SwerveModuleVelocity(0.12, Rotation2d.ZERO),
          new SwerveModuleVelocity(0.08, Rotation2d.ZERO),
          new SwerveModuleVelocity(0.10, Rotation2d.ZERO),
          new SwerveModuleVelocity(0.09, Rotation2d.ZERO)
        };
    SkidDetector.SkidState state = detector.calculate(noisyCrawl, SQUARE_LOCATIONS, 0.0);
    assertFalse(state.skidding());
    assertEquals(1.0, state.skidRatio(), EPS);
    assertEquals(SkidDetector.NO_SLIPPING_MODULES, state.slippingModuleMask());
  }

  @Test
  void skidDetectorDetectsDualLeftWheelSlipWhenUsingGyroOmega() {
    SkidDetector detector = new SkidDetector(1.35);
    // Square 0.6 m chassis moving forward at 1.0 m/s with both left wheels (FL=0, BL=2)
    // slipping at 1.6 m/s while the robot is not rotating (gyro omega = 0).
    // With gyro omega = 0, skidRatio = 1.60 > 1.35 (whereas wheel-fit omega would mask it at
    // ~1.26).
    SwerveModuleVelocity[] leftSlip =
        new SwerveModuleVelocity[] {
          new SwerveModuleVelocity(1.6, Rotation2d.ZERO),
          new SwerveModuleVelocity(1.0, Rotation2d.ZERO),
          new SwerveModuleVelocity(1.6, Rotation2d.ZERO),
          new SwerveModuleVelocity(1.0, Rotation2d.ZERO)
        };
    SkidDetector.SkidState gyroState = detector.calculate(leftSlip, SQUARE_LOCATIONS, 0.0);
    assertTrue(gyroState.skidding());
    assertEquals(1.6, gyroState.skidRatio(), 1e-4);
    // Exact 2-vs-2 split has no majority consensus, so all 4 module bits are flagged.
    assertEquals(0b1111, gyroState.slippingModuleMask());
  }

  @Test
  void skidDetectorIsolatesSingleSlippingOrDraggingModuleInBitmask() {
    SkidDetector detector = new SkidDetector(1.35);

    // Module 3 spinning fast (wheelspin): only bit 3 should be set.
    SwerveModuleVelocity[] singleSpin =
        new SwerveModuleVelocity[] {
          new SwerveModuleVelocity(1.0, Rotation2d.ZERO),
          new SwerveModuleVelocity(1.0, Rotation2d.ZERO),
          new SwerveModuleVelocity(1.0, Rotation2d.ZERO),
          new SwerveModuleVelocity(1.8, Rotation2d.ZERO)
        };
    SkidDetector.SkidState spinState = detector.calculate(singleSpin, SQUARE_LOCATIONS, 0.0);
    assertTrue(spinState.skidding());
    assertEquals(1.8, spinState.skidRatio(), 1e-4);
    assertEquals(1 << 3, spinState.slippingModuleMask());
    assertFalse(spinState.isModuleSlipping(0));
    assertFalse(spinState.isModuleSlipping(1));
    assertFalse(spinState.isModuleSlipping(2));
    assertTrue(spinState.isModuleSlipping(3));

    // Module 1 stalled/dragging: only bit 1 should be set.
    SwerveModuleVelocity[] singleDrag =
        new SwerveModuleVelocity[] {
          new SwerveModuleVelocity(1.0, Rotation2d.ZERO),
          new SwerveModuleVelocity(0.2, Rotation2d.ZERO),
          new SwerveModuleVelocity(1.0, Rotation2d.ZERO),
          new SwerveModuleVelocity(1.0, Rotation2d.ZERO)
        };
    SkidDetector.SkidState dragState = detector.calculate(singleDrag, SQUARE_LOCATIONS, 0.0);
    assertTrue(dragState.skidding());
    assertEquals(5.0, dragState.skidRatio(), 1e-4);
    assertEquals(1 << 1, dragState.slippingModuleMask());
    assertTrue(dragState.isModuleSlipping(1));
    assertFalse(dragState.isModuleSlipping(0));
  }

  @Test
  void skidDetectorValidatesArguments() {
    assertThrows(IllegalArgumentException.class, () -> new SkidDetector(1.0));
    assertThrows(IllegalArgumentException.class, () -> new SkidDetector(Double.NaN));
    assertThrows(
        IllegalArgumentException.class, () -> new SkidDetector(1.35, MetersPerSecond.of(-0.1)));
    SkidDetector detector = new SkidDetector(1.35);
    assertThrows(
        IllegalArgumentException.class,
        () -> detector.calculate(new SwerveModuleVelocity[2], SQUARE_LOCATIONS, 0.0));
  }

  @Test
  void skidDetectorFuzzInvariantsHold() {
    SkidDetector detector = new SkidDetector(1.35);
    SplittableRandom rng = new SplittableRandom(20270278L);
    for (int trial = 0; trial < 300; trial++) {
      double vx = rng.nextDouble(-4.5, 4.5);
      double vy = rng.nextDouble(-4.5, 4.5);
      double omega = rng.nextDouble(-8.0, 8.0);
      SwerveModuleVelocity[] states =
          KINEMATICS.toSwerveModuleVelocities(new ChassisVelocities(vx, vy, omega));
      if (trial % 3 == 0) {
        int bad = rng.nextInt(4);
        states[bad] =
            new SwerveModuleVelocity(
                states[bad].velocity * rng.nextDouble(0.0, 3.0), states[bad].angle);
      }
      SkidDetector.SkidState state = detector.calculate(states, SQUARE_LOCATIONS, omega);
      assertTrue(Double.isFinite(state.skidRatio()));
      assertTrue(state.skidRatio() >= 1.0 - EPS);
      if (state.skidding()) {
        assertTrue(state.slippingModuleMask() != SkidDetector.NO_SLIPPING_MODULES);
      } else {
        assertEquals(SkidDetector.NO_SLIPPING_MODULES, state.slippingModuleMask());
      }
    }
  }

  // ---- TiltOdometryScaler unit tests ---------------------------------------------------------

  @Test
  void tiltOdometryScalerLinearRampAndCosineProjectionModes() {
    TiltOdometryScaler linear =
        new TiltOdometryScaler(
            Degrees.of(2), Degrees.of(25), TiltOdometryScaler.ScalingMode.LINEAR_RAMP);
    TiltOdometryScaler cosine =
        new TiltOdometryScaler(
            Degrees.of(2), Degrees.of(25), TiltOdometryScaler.ScalingMode.COSINE_PROJECTION);

    // Flat & yaw-invariant.
    Rotation3d flatYawed = new Rotation3d(0.0, 0.0, Math.toRadians(137.0));
    assertEquals(1.0, linear.computeScale(flatYawed), EPS);
    assertEquals(1.0, cosine.computeScale(flatYawed), EPS);

    // Midpoint (13.5 deg): linear = 0.5, cosine = cos(13.5 deg).
    Rotation3d midTilt = new Rotation3d(0.0, Math.toRadians(13.5), Math.toRadians(45.0));
    assertEquals(0.5, linear.computeScale(midTilt), 1e-4);
    assertEquals(Math.cos(Math.toRadians(13.5)), cosine.computeScale(midTilt), 1e-4);

    // Beyond zeroTrustTilt (30 deg): both = 0.0.
    Rotation3d steep = new Rotation3d(Math.toRadians(30.0), 0.0, 0.0);
    assertEquals(0.0, linear.computeScale(steep), EPS);
    assertEquals(0.0, cosine.computeScale(steep), EPS);
  }

  @Test
  void tiltOdometryScalerDropsOnlySlippingModuleDeltaWhileHealthyWheelsAdvance() {
    TiltOdometryScaler scaler = TiltOdometryScaler.passthrough();
    SwerveModulePosition[] p0 = positions(0.0, 0.0, 0.0, 0.0);
    scaler.reset(p0);

    // Tick 1: all 4 modules advance by +1.0 m, but module 3 is flagged as slipping.
    SwerveModulePosition[] p1 = positions(1.0, 1.0, 1.0, 1.0);
    SwerveModulePosition[] scaled1 = scaler.scalePositions(Rotation3d.ZERO, p1, 1 << 3);
    assertEquals(1.0, scaled1[0].distance, EPS);
    assertEquals(1.0, scaled1[1].distance, EPS);
    assertEquals(1.0, scaled1[2].distance, EPS);
    assertEquals(0.0, scaled1[3].distance, EPS);
    assertEquals(0.75, scaler.getLastScale(), EPS);

    // Tick 2: grip recovered (mask = 0), all modules advance by +0.5 m.
    SwerveModulePosition[] p2 = positions(1.5, 1.5, 1.5, 1.5);
    SwerveModulePosition[] scaled2 =
        scaler.scalePositions(Rotation3d.ZERO, p2, SkidDetector.NO_SLIPPING_MODULES);
    assertEquals(1.5, scaled2[0].distance, EPS);
    assertEquals(1.5, scaled2[1].distance, EPS);
    assertEquals(1.5, scaled2[2].distance, EPS);
    assertEquals(0.5, scaled2[3].distance, EPS);
    assertEquals(1.0, scaler.getLastScale(), EPS);
  }

  @Test
  void tiltOdometryScalerConfigProducesIndependentStatefulInstances() {
    TiltOdometryScaler.Config cfg =
        new TiltOdometryScaler.Config(
            Degrees.of(2), Degrees.of(25), TiltOdometryScaler.ScalingMode.COSINE_PROJECTION);
    TiltOdometryScaler a = new TiltOdometryScaler(cfg);
    TiltOdometryScaler b = new TiltOdometryScaler(cfg);

    a.reset(positions(0.0, 0.0, 0.0, 0.0));
    b.reset(positions(0.0, 0.0, 0.0, 0.0));

    SwerveModulePosition[] outA =
        a.scalePositions(
            new Rotation3d(0.0, Math.toRadians(30.0), 0.0), positions(2.0, 2.0, 2.0, 2.0), 0);
    SwerveModulePosition[] outB =
        b.scalePositions(Rotation3d.ZERO, positions(2.0, 2.0, 2.0, 2.0), 0);

    assertEquals(0.0, outA[0].distance, EPS);
    assertEquals(2.0, outB[0].distance, EPS);
  }

  @Test
  void tiltOdometryScalerValidatesConfigAndFuzzInvariants() {
    assertThrows(
        IllegalArgumentException.class,
        () -> new TiltOdometryScaler(Degrees.of(10), Degrees.of(10)));
    assertThrows(
        IllegalArgumentException.class,
        () -> new TiltOdometryScaler(Degrees.of(-1), Degrees.of(20)));
    assertThrows(
        IllegalArgumentException.class,
        () -> new TiltOdometryScaler(Degrees.of(2), Degrees.of(95)));

    SplittableRandom rng = new SplittableRandom(20270279L);
    for (TiltOdometryScaler.ScalingMode mode : TiltOdometryScaler.ScalingMode.values()) {
      TiltOdometryScaler scaler = new TiltOdometryScaler(Degrees.of(2), Degrees.of(25), mode);
      for (int i = 0; i < 200; i++) {
        Rotation3d att =
            new Rotation3d(
                rng.nextDouble(-Math.PI * 0.45, Math.PI * 0.45),
                rng.nextDouble(-Math.PI * 0.45, Math.PI * 0.45),
                rng.nextDouble(-Math.PI, Math.PI));
        double scale = scaler.computeScale(att);
        assertTrue(scale >= 0.0 && scale <= 1.0);
      }
    }
  }

  // ---- SwerveDrive-level integration tests ---------------------------------------------------

  @Test
  void skidOdometryRejectionWithoutSkidDetectionThrowsIllegalStateException() {
    SwerveDriveConfig cfg = baseDriveConfig(TelemetryVerbosity.LOW).withSkidOdometryRejection(true);
    assertThrows(IllegalStateException.class, () -> new SwerveDrive(cfg));
  }

  @Test
  void twoDrivesFromSameConfigDoNotShareTiltScalerState() {
    SwerveDriveConfig sharedCfg =
        baseDriveConfig(TelemetryVerbosity.LOW)
            .withTiltScaledOdometry(
                Degrees.of(2), Degrees.of(25), TiltOdometryScaler.ScalingMode.LINEAR_RAMP);
    AtomicReference<Rotation3d> tilted =
        new AtomicReference<>(new Rotation3d(0.0, Math.toRadians(30.0), 0.0));
    AtomicReference<Rotation3d> flat = new AtomicReference<>(Rotation3d.ZERO);

    TiltingTestSwerveDrive driveA = new TiltingTestSwerveDrive(sharedCfg, tilted);
    TiltingTestSwerveDrive driveB =
        new TiltingTestSwerveDrive(
            sharedCfg.withTelemetry(
                "SkidTiltSharedB" + DRIVE_COUNTER.incrementAndGet(), TelemetryVerbosity.LOW),
            flat);

    driveA.updateTelemetry();
    driveB.updateTelemetry();

    assertEquals(0.0, driveA.getTiltOdometryScale(), EPS);
    assertEquals(1.0, driveB.getTiltOdometryScale(), EPS);
  }

  @Test
  void resetOdometryAndZeroOdometryHeadingResetSkidAndTiltState() {
    SwerveDrive drive =
        new SwerveDrive(
            baseDriveConfig(TelemetryVerbosity.LOW)
                .withSkidDetection(1.35)
                .withSkidOdometryRejection(true)
                .withTiltScaledOdometry(Degrees.of(2), Degrees.of(25)));

    drive.updateTelemetry();
    Pose2d target = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(90.0));
    drive.resetOdometry(target);
    assertEquals(2.0, drive.getPose().getX(), 1e-3);
    assertEquals(3.0, drive.getPose().getY(), 1e-3);
    assertEquals(90.0, drive.getPose().getRotation().getDegrees(), 1e-2);
    assertEquals(1.0, drive.getSkidRatio(), EPS);
    assertFalse(drive.isSkidding());
    assertEquals(1.0, drive.getTiltOdometryScale(), EPS);

    drive.zeroOdometryHeading();
    assertEquals(0.0, drive.getPose().getRotation().getDegrees(), 1e-2);
    assertEquals(1.0, drive.getSkidRatio(), EPS);
    assertEquals(1.0, drive.getTiltOdometryScale(), EPS);
  }

  @Test
  void visionMeasurementIsRejectedWhileTiltedAndAcceptedWhenFlat() {
    AtomicReference<Rotation3d> attitude =
        new AtomicReference<>(new Rotation3d(0.0, Math.toRadians(15.0), 0.0));
    SwerveDriveConfig cfg =
        baseDriveConfig(TelemetryVerbosity.LOW).withVisionMaxTilt(Degrees.of(5));
    TiltingTestSwerveDrive drive = new TiltingTestSwerveDrive(cfg, attitude);
    drive.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.ZERO));
    drive.updateTelemetry();

    // Tilted at 15 deg (> 5 deg): both addVisionMeasurement overloads must reject the update.
    Pose2d visionPose = new Pose2d(5.0, 5.0, Rotation2d.ZERO);
    drive.addVisionMeasurement(visionPose, Timer.getTimestamp(), VecBuilder.fill(0.01, 0.01, 0.01));
    drive.addVisionMeasurement(visionPose, Timer.getTimestamp());
    assertEquals(0.0, drive.getPose().getX(), 1e-2);
    assertEquals(0.0, drive.getPose().getY(), 1e-2);

    // Flatten back to 0 deg (<= 5 deg): vision update must now pull the pose estimate toward (5,
    // 5).
    attitude.set(Rotation3d.ZERO);
    drive.updateTelemetry();
    drive.addVisionMeasurement(visionPose, Timer.getTimestamp(), VecBuilder.fill(0.01, 0.01, 0.01));
    assertTrue(drive.getPose().getX() > 1.0, "Expected flat vision update to shift pose X");
    assertTrue(drive.getPose().getY() > 1.0, "Expected flat vision update to shift pose Y");
  }

  @Test
  void telemetryFieldsAreDisabledWhenUnconfiguredAndPublishedWhenConfigured() {
    // 1. Unconfigured drive at HIGH verbosity: SkidRatio, IsSkidding, TiltOdometryScale must be
    // disabled.
    SwerveDriveTelemetryConfig unconfiguredTelCfg =
        new SwerveDriveTelemetryConfig().withTelemetryVerbosity(TelemetryVerbosity.HIGH);
    SwerveDrive plainDrive =
        new SwerveDrive(
            baseDriveConfig(TelemetryVerbosity.HIGH)
                .withTelemetry("PlainDrive" + DRIVE_COUNTER.incrementAndGet(), unconfiguredTelCfg));
    plainDrive.updateTelemetry();

    assertFalse(
        unconfiguredTelCfg
            .getDoubleFields(plainDrive)
            .get(SwerveDriveTelemetry.DoubleTelemetryField.SkidRatio)
            .isEnabled());
    assertFalse(
        unconfiguredTelCfg
            .getDoubleFields(plainDrive)
            .get(SwerveDriveTelemetry.DoubleTelemetryField.TiltOdometryScale)
            .isEnabled());
    assertFalse(
        unconfiguredTelCfg
            .getBoolFields()
            .get(SwerveDriveTelemetry.BooleanTelemetryField.IsSkidding)
            .isEnabled());

    // 2. Configured drive at HIGH verbosity: all three fields must be enabled and published.
    String configuredName = "ConfiguredDrive" + DRIVE_COUNTER.incrementAndGet();
    SwerveDriveTelemetryConfig configuredTelCfg =
        new SwerveDriveTelemetryConfig().withTelemetryVerbosity(TelemetryVerbosity.HIGH);
    SwerveDrive configuredDrive =
        new SwerveDrive(
            baseDriveConfig(TelemetryVerbosity.HIGH)
                .withSkidDetection(1.35)
                .withTiltScaledOdometry(
                    Degrees.of(2), Degrees.of(25), TiltOdometryScaler.ScalingMode.COSINE_PROJECTION)
                .withTelemetry(configuredName, configuredTelCfg));
    configuredDrive.updateTelemetry();

    assertTrue(
        configuredTelCfg
            .getDoubleFields(configuredDrive)
            .get(SwerveDriveTelemetry.DoubleTelemetryField.SkidRatio)
            .isEnabled());
    assertTrue(
        configuredTelCfg
            .getDoubleFields(configuredDrive)
            .get(SwerveDriveTelemetry.DoubleTelemetryField.TiltOdometryScale)
            .isEnabled());
    assertTrue(
        configuredTelCfg
            .getBoolFields()
            .get(SwerveDriveTelemetry.BooleanTelemetryField.IsSkidding)
            .isEnabled());

    NetworkTable nt =
        NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(configuredName);
    assertTrue(nt != null);
  }

  private static SwerveModulePosition[] positions(double d0, double d1, double d2, double d3) {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(d0, Rotation2d.ZERO),
      new SwerveModulePosition(d1, Rotation2d.ZERO),
      new SwerveModulePosition(d2, Rotation2d.ZERO),
      new SwerveModulePosition(d3, Rotation2d.ZERO)
    };
  }
}
