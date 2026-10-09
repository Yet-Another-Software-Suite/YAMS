// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechs;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.thrifty.nova.Nova;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.MathUtil;
import org.wpilib.preferences.Preferences;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.config.SwerveDriveConfig;
import yams.commands2.swerve.SwerveDrive;
import yams.core.exceptions.SwerveDriveConfigurationException;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.SwerveModuleConfig;
import yams.core.mechanisms.swerve.SwerveModule;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.local.NovaWrapper;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.motorcontrollers.remote.TalonFXSWrapper;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.TestWithScheduler;

/**
 * Four module swerve drive integration test, ported from the C++ SwerveDriveTest. Every drive reads
 * its attitude from a Pigeon2, and the tests that depend on the hardware run against each
 * {@link Layout}: a TalonFX and SPARK MAX drive both ways round, a Thrifty Nova drive, and drives
 * mixing SPARK MAX, SPARK Flex, TalonFXS, TalonFX and Nova motor controllers in differing
 * combinations.
 *
 * <p>In simulation the drive reads its heading from its own simulated gyro, so most tests cover how
 * {@link SwerveDrive} uses {@link SwerveDrive#getGyroRotation3d()}; {@link
 * #configReadsThePigeon2(Layout)} covers reading the configured Pigeon2.
 */
public class SwerveDriveTest {
  /** Module offset from the robot centre for a 24 in by 24 in square chassis. */
  private static final double kModuleOffsetInches = 12;

  private static final double kTolerance = 1e-6;

  /** Motor controllers a module's drive or azimuth motor can use. */
  enum Motor {
    SPARK_MAX,
    SPARK_FLEX,
    TALON_FXS,
    TALON_FX,
    NOVA
  }

  /**
   * Motor controllers of a drive's modules, in FL, FR, BL, BR order.
   *
   * @param name     Name shown in the test report.
   * @param drives   Drive motor controller of each module.
   * @param azimuths Azimuth motor controller of each module.
   */
  record Layout(String name, Motor[] drives, Motor[] azimuths) {
    static Layout uniform(String name, Motor drive, Motor azimuth) {
      return new Layout(
          name,
          new Motor[] {drive, drive, drive, drive},
          new Motor[] {azimuth, azimuth, azimuth, azimuth});
    }

    @Override
    public String toString() {
      return name;
    }
  }

  private static final Motor SMAX = Motor.SPARK_MAX;
  private static final Motor SFLEX = Motor.SPARK_FLEX;
  private static final Motor TFXS = Motor.TALON_FXS;
  private static final Motor TFX = Motor.TALON_FX;
  private static final Motor NOVA = Motor.NOVA;

  /** TalonFX drive motors and SPARK MAX azimuth motors. */
  private static final Layout kTalonFXDriveSparkMaxAzimuth =
      Layout.uniform("Pigeon2, TalonFX drive, SPARK MAX azimuth", TFX, SMAX);

  static Stream<Layout> layouts() {
    return Stream.of(
        kTalonFXDriveSparkMaxAzimuth,
        Layout.uniform("Pigeon2, SPARK MAX drive, TalonFX azimuth", SMAX, TFX),
        // Every motor controller as a drive and an azimuth, each module pairing two different ones.
        new Layout(
            "Pigeon2, mixed: SMAX/SFLEX, SFLEX/TFXS, TFXS/TFX, TFX/SMAX",
            new Motor[] {SMAX, SFLEX, TFXS, TFX},
            new Motor[] {SFLEX, TFXS, TFX, SMAX}),
        // The same pairs the other way round.
        new Layout(
            "Pigeon2, mixed: SFLEX/SMAX, TFXS/SFLEX, TFX/TFXS, SMAX/TFX",
            new Motor[] {SFLEX, TFXS, TFX, SMAX},
            new Motor[] {SMAX, SFLEX, TFXS, TFX}),
        // Each module one motor controller for both, every module a different one.
        new Layout(
            "Pigeon2, mixed: SMAX/SMAX, SFLEX/SFLEX, TFXS/TFXS, TFX/TFX",
            new Motor[] {SMAX, SFLEX, TFXS, TFX},
            new Motor[] {SMAX, SFLEX, TFXS, TFX}),
        Layout.uniform("Pigeon2, Nova drive, Nova azimuth", NOVA, NOVA),
        // A Nova paired with every other motor controller, as the drive and as the azimuth.
        new Layout(
            "Pigeon2, mixed: NOVA/SMAX, SFLEX/NOVA, NOVA/TFXS, TFX/NOVA",
            new Motor[] {NOVA, SFLEX, NOVA, TFX},
            new Motor[] {SMAX, NOVA, TFXS, NOVA}));
  }

  /** Subsystem that runs the drive's telemetry and simulation, like a robot's swerve subsystem. */
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

  /** Counter giving each drive its own telemetry names, so its tunables are not published twice. */
  private static int driveCount = 0;

  /**
   * A layout's devices and modules. Phoenix keeps simulating every CTRE device a run creates, even
   * after it is closed, and with a new set for every test the simulated CAN bus fills until Talon
   * data arrives late and the modules fall behind. So, like the C++ test, each layout's hardware is
   * created once for the whole class, and only the {@link SwerveDrive} is rebuilt per test.
   */
  private static final class Hardware {
    final SwerveTestSubsystem subsystem = new SwerveTestSubsystem();
    final Pigeon2 pigeon = DeviceCreator.createPigeon2();
    final List<SmartMotorController> motorControllers = new ArrayList<>();
    final SwerveModule[] modules;

    Hardware(Layout layout) {
      final String prefix = "L" + HARDWARE.size();
      modules =
          new SwerveModule[] {
            createModule(
                this,
                prefix + "FL",
                layout.drives()[0],
                layout.azimuths()[0],
                kModuleOffsetInches,
                kModuleOffsetInches),
            createModule(
                this,
                prefix + "FR",
                layout.drives()[1],
                layout.azimuths()[1],
                kModuleOffsetInches,
                -kModuleOffsetInches),
            createModule(
                this,
                prefix + "BL",
                layout.drives()[2],
                layout.azimuths()[2],
                -kModuleOffsetInches,
                kModuleOffsetInches),
            createModule(
                this,
                prefix + "BR",
                layout.drives()[3],
                layout.azimuths()[3],
                -kModuleOffsetInches,
                -kModuleOffsetInches)
          };
    }

    void close() {
      CommandScheduler.getInstance().unregisterSubsystem(subsystem);
      for (SmartMotorController smc : motorControllers) {
        smc.close();
        DeviceCreator.silence(smc);
        switch (smc.getMotorController()) {
          case SparkMax spark -> spark.close();
          case SparkFlex spark -> spark.close();
          case TalonFXS talon -> talon.close();
          case TalonFX talon -> talon.close();
          case Nova nova -> nova.close();
          default -> {}
        }
      }
      DeviceCreator.silence(pigeon);
      pigeon.close();
    }
  }

  /** Each layout's hardware, by layout name, created the first time a test uses the layout. */
  private static final Map<String, Hardware> HARDWARE = new LinkedHashMap<>();

  private SwerveTestSubsystem subsystem;
  private Pigeon2 pigeon;
  private SwerveModule[] modules;
  private SwerveDrive drive;

  /**
   * The motor each motor controller drives: a NEO, NEO Vortex, NEO on a TalonFXS, Kraken X60, or NEO
   * on a Nova.
   */
  private static DCMotor dcMotor(Motor motor) {
    return switch (motor) {
      case SPARK_MAX, TALON_FXS, NOVA -> DCMotor.getNEO(1);
      case SPARK_FLEX -> DCMotor.getNeoVortex(1);
      case TALON_FX -> DCMotor.getKrakenX60(1);
    };
  }

  private static SmartMotorController createMotor(
      Hardware hardware, Motor motor, SmartMotorControllerConfig config) {
    final SmartMotorController smc =
        switch (motor) {
          case SPARK_MAX ->
              new SparkWrapper(DeviceCreator.createSparkMax(), dcMotor(motor), config);
          case SPARK_FLEX ->
              new SparkWrapper(DeviceCreator.createSparkFlex(), dcMotor(motor), config);
          case TALON_FXS ->
              new TalonFXSWrapper(DeviceCreator.createTalonFXS(), dcMotor(motor), config);
          case TALON_FX ->
              new TalonFXWrapper(DeviceCreator.createTalonFX(), dcMotor(motor), config);
          case NOVA -> new NovaWrapper(DeviceCreator.createNova(), dcMotor(motor), config);
        };
    hardware.motorControllers.add(smc);
    return smc;
  }

  /** Drive gear reduction. */
  private static final double kDriveReduction = 6.75;

  /**
   * Drive velocity feedforward, volts per wheel rotation per second: 12 V over the motor's free
   * speed at the wheel. kV is a property of the motor; the PID gains below are the same on every
   * motor controller, as a team switching vendors would leave them.
   */
  private static double driveKV(Motor motor) {
    return 12.0 / (dcMotor(motor).freeSpeed / (2 * Math.PI) / kDriveReduction);
  }

  private static SwerveModule createModule(
      Hardware hardware,
      String name,
      Motor driveMotor,
      Motor azimuthMotor,
      double frontInches,
      double leftInches) {
    SmartMotorControllerConfig driveCfg =
        new SmartMotorControllerConfig(hardware.subsystem)
            .withWheelDiameter(Inches.of(4))
            .withClosedLoopController(0.4, 0, 0)
            .withFeedforward(new SimpleMotorFeedforward(0, driveKV(driveMotor)))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(kDriveReduction)))
            .withStatorCurrentLimit(Amps.of(40))
            .withTelemetry(name + "Drive", TelemetryVerbosity.LOW);
    SmartMotorControllerConfig azimuthCfg =
        new SmartMotorControllerConfig(hardware.subsystem)
            .withClosedLoopController(50, 0, 0.5)
            .withContinuousWrapping(Radians.of(-Math.PI), Radians.of(Math.PI))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(12.8)))
            .withStatorCurrentLimit(Amps.of(40))
            .withTelemetry(name + "Azimuth", TelemetryVerbosity.LOW);
    SwerveModuleConfig moduleCfg =
        new SwerveModuleConfig(
                createMotor(hardware, driveMotor, driveCfg),
                createMotor(hardware, azimuthMotor, azimuthCfg))
            .withAbsoluteEncoder(() -> Degrees.of(0))
            .withLocation(new Translation2d(Inches.of(frontInches), Inches.of(leftInches)))
            .withOptimization(true)
            .withCosineCompensation(true)
            .withTelemetry(name, TelemetryVerbosity.LOW);
    return new SwerveModule(moduleCfg);
  }

  /** A drive config for the layout's modules and Pigeon2, with no translation or rotation controller. */
  private SwerveDriveConfig baseConfig() {
    driveCount++;
    return new SwerveDriveConfig(subsystem, modules)
        // Phoenix's simulated Pigeon2 does not fill in its quaternion, which getRotation3d() reads,
        // so read the attitude from its roll, pitch and yaw.
        .withGyro(
            () ->
                new Rotation3d(
                    pigeon.getRoll().getValue(),
                    pigeon.getPitch().getValue(),
                    pigeon.getYaw().getValue()))
        .withStartingPose(new Pose2d())
        .withMaximumChassisSpeed(MetersPerSecond.of(4.5), DegreesPerSecond.of(540))
        .withTelemetry("SwerveDriveTest" + driveCount, TelemetryVerbosity.LOW);
  }

  /** Build a drive, with translation and rotation controllers, on the layout's hardware. */
  private void build(Layout layout) {
    final Hardware hardware = HARDWARE.computeIfAbsent(layout.name(), name -> new Hardware(layout));
    subsystem = hardware.subsystem;
    pigeon = hardware.pigeon;
    modules = hardware.modules;
    drive =
        new SwerveDrive(
            baseConfig()
                .withTranslationController(new PIDController(2, 0, 0))
                .withRotationController(new PIDController(4, 0, 0)));
    subsystem.drive = drive;
  }

  /** Run the scheduler, giving Phoenix, which simulates Talons in real time, each loop to catch up. */
  private static void runFor(double seconds) {
    TestWithScheduler.cycle(
        Seconds.of(seconds),
        () -> {
          try {
            Thread.sleep(20);
          } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
          }
        });
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
    if (drive != null) {
      // The hardware is shared with the next test: bring the wheels to a stop first.
      drive.setRobotRelativeChassisSpeeds(new ChassisVelocities());
      runFor(0.3);
      subsystem.drive = null;
    }
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  @AfterAll
  static void closeHardware() {
    HARDWARE.values().forEach(Hardware::close);
    HARDWARE.clear();
  }

  /** The drive's heading from its gyro, in degrees. */
  private double gyroHeadingDegrees() {
    return Math.toDegrees(drive.getGyroRotation3d().getZ());
  }

  // ---- Every layout --------------------------------------------------------------------------

  @ParameterizedTest(name = "{0}")
  @MethodSource("layouts")
  void telemetryAndSimRunWithoutCrash(Layout layout) {
    build(layout);
    assertDoesNotThrow(() -> runFor(0.5));
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("layouts")
  void configReadsThePigeon2(Layout layout) throws InterruptedException {
    build(layout);
    // The Pigeon2 simulation publishes its attitude in real time: wait for it to arrive.
    pigeon.getSimState().setRawYaw(Degrees.of(30));
    pigeon.getSimState().setPitch(Degrees.of(5));
    Rotation3d attitude = drive.getConfig().getGyroRotation3d();
    for (int attempt = 0;
        attempt < 100 && Math.abs(Math.toDegrees(attitude.getZ()) - 30) > 0.5;
        attempt++) {
      Thread.sleep(20);
      attitude = drive.getConfig().getGyroRotation3d();
    }
    assertEquals(30, Math.toDegrees(attitude.getZ()), 0.5, "yaw from the Pigeon2");
    assertEquals(5, Math.toDegrees(attitude.getY()), 0.5, "pitch from the Pigeon2");
  }

  /**
   * How far the modules are from moving as one rigid body: the chassis motion that best fits their
   * measured states is found with the kinematics, and this is the largest difference, as a velocity
   * vector, between a module's measured state and the state that motion gives it. Modules fighting
   * each other cannot all fit one motion, so this is large; a drive that drifts still fits one.
   */
  private double moduleDisagreement() {
    final SwerveModuleVelocity[] measured = drive.getModuleStates();
    final var kinematics = drive.getKinematics();
    final SwerveModuleVelocity[] rigid =
        kinematics.toSwerveModuleVelocities(kinematics.toChassisVelocities(measured));
    double worst = 0;
    for (int i = 0; i < measured.length; i++) {
      final Translation2d measuredVector =
          new Translation2d(measured[i].velocity, measured[i].angle);
      final Translation2d rigidVector = new Translation2d(rigid[i].velocity, rigid[i].angle);
      worst = Math.max(worst, measuredVector.getDistance(rigidVector));
    }
    return worst;
  }

  /**
   * What a drive did once its modules had turned to their first states.
   *
   * @param disagreement Worst {@link #moduleDisagreement()} after settling, in meters per second.
   * @param settledPose  Pose when the modules had settled.
   * @param pose         Pose at the end.
   * @param speed        Measured robot relative speed at the end.
   */
  private record DriveResult(
      double disagreement, Pose2d settledPose, Pose2d pose, ChassisVelocities speed) {}

  /** Seconds the modules get to turn to their first states before they are checked. */
  private static final double kSettleSeconds = 0.75;

  /** Drive with the given speeds for {@code seconds}, checking the modules after they settle. */
  private DriveResult drive(java.util.function.Supplier<ChassisVelocities> speeds, double seconds) {
    TestWithScheduler.schedule(drive.drive(speeds));
    final int settleLoops = (int) Math.round(kSettleSeconds / 0.02);
    final AtomicInteger loop = new AtomicInteger();
    final double[] worst = {0};
    final Pose2d[] settledPose = {null};
    TestWithScheduler.cycle(
        Seconds.of(seconds),
        () -> {
          try {
            Thread.sleep(20);
          } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
          }
          final int current = loop.incrementAndGet();
          if (current == settleLoops) {
            settledPose[0] = drive.getPose();
          } else if (current > settleLoops) {
            worst[0] = Math.max(worst[0], moduleDisagreement());
          }
        });
    return new DriveResult(
        worst[0], settledPose[0], drive.getPose(), drive.getRobotRelativeSpeed());
  }

  /** Largest velocity, in meters per second, a module may be off from the drive's rigid motion. */
  private static final double kMaxDisagreement = 0.25;

  @ParameterizedTest(name = "{0}")
  @MethodSource("layouts")
  void drivesForward(Layout layout) {
    build(layout);
    final DriveResult result = drive(() -> new ChassisVelocities(1, 0, 0), 2);
    System.out.printf("%s forward: %s%n", layout, result);
    assertTrue(result.pose().getX() > 1, "should drive forward, but is at " + result.pose());
    assertEquals(0, result.pose().getY(), 0.1, "should not drive sideways");
    assertEquals(0, result.pose().getRotation().getDegrees(), 5, "should not turn");
    assertEquals(1, result.speed().vx, 0.2, "should be driving forward at the commanded speed");
    assertTrue(
        result.disagreement() < kMaxDisagreement,
        "modules should not fight, but were " + result.disagreement() + " m/s apart");
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("layouts")
  void spinsInPlace(Layout layout) {
    build(layout);
    final DriveResult result = drive(() -> new ChassisVelocities(0, 0, 2), 2);
    System.out.printf("%s spin: %s%n", layout, result);
    final double movedSinceSettling =
        result.pose().getTranslation().getDistance(result.settledPose().getTranslation());
    final double turnedSinceSettling =
        result.pose().getRotation().minus(result.settledPose().getRotation()).getDegrees();
    assertEquals(
        2, result.speed().omega, 0.4, "should be spinning counterclockwise at the commanded speed");
    assertTrue(
        movedSinceSettling < 0.1, "should spin in place, but moved " + movedSinceSettling + " m");
    assertTrue(
        Math.abs(turnedSinceSettling) > 20,
        "should have kept turning, but turned " + turnedSinceSettling + " degrees");
    assertTrue(
        result.disagreement() < kMaxDisagreement,
        "modules should not fight, but were " + result.disagreement() + " m/s apart");
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("layouts")
  void translatesWhileSpinning(Layout layout) {
    build(layout);
    // Field relative toward +X while spinning: some drift off +X is expected, fighting is not.
    final DriveResult result =
        drive(
            () ->
                new ChassisVelocities(1, 0, 2)
                    .toRobotRelative(drive.getGyroRotation3d().toRotation2d()),
            2);
    System.out.printf("%s translate while spinning: %s%n", layout, result);
    final Translation2d travel =
        result.pose().getTranslation().minus(result.settledPose().getTranslation());
    assertTrue(
        travel.getX() > 0.5, "should translate toward +X while spinning, but went " + travel);
    assertEquals(
        2, result.speed().omega, 0.4, "should be spinning counterclockwise at the commanded speed");
    assertEquals(
        1,
        Math.hypot(result.speed().vx, result.speed().vy),
        0.3,
        "should be translating at the commanded speed");
    assertTrue(
        result.disagreement() < kMaxDisagreement,
        "modules should not fight, but were " + result.disagreement() + " m/s apart");
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("layouts")
  void resetOdometryAlignsGyro(Layout layout) {
    build(layout);
    // ResetOdometry sets the gyro to the pose's heading so field relative driving and heading
    // control, which use the gyro, agree with the reset pose. ZeroGyro resets both to 0 degrees.
    drive.resetOdometry(new Pose2d(1, 2, Rotation2d.fromDegrees(90)));
    assertEquals(90, gyroHeadingDegrees(), 0.1);
    assertEquals(90, drive.getPose().getRotation().getDegrees(), 0.1);

    drive.zeroGyro();
    assertEquals(0, gyroHeadingDegrees(), 0.1);
    assertEquals(0, drive.getPose().getRotation().getDegrees(), 0.1);
    assertEquals(1, drive.getPose().getX(), 0.01);
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("layouts")
  void lockPoseSetsXPattern(Layout layout) {
    build(layout);
    drive.lockPose();
    ChassisVelocities desired = drive.getDesiredChassisSpeeds();
    assertEquals(0, desired.vx, 1e-9);
    assertEquals(0, desired.vy, 1e-9);
    assertEquals(0, desired.omega, 1e-9);

    runFor(1);
    for (SwerveModule module : drive.getModules()) {
      double expected =
          module.getConfig().getLocation().orElseThrow().getAngle().orElseThrow().getDegrees();
      double actual = module.getState().angle.getDegrees();
      // Module optimization may reverse the wheel, so angles half a rotation apart are equivalent.
      double error = Math.IEEEremainder(actual - expected, 180);
      assertEquals(
          0, error, 10, module.getName() + " at " + actual + " should turn toward " + expected);
    }
  }

  // ---- One layout: these do not depend on the motor controllers --------------------------------

  @Test
  void configuredPIDControllersArePresent() {
    build(kTalonFXDriveSparkMaxAzimuth);
    assertEquals(2, drive.getConfig().getTranslationPID().orElseThrow().getP(), kTolerance);
    assertEquals(4, drive.getConfig().getRotationPID().orElseThrow().getP(), kTolerance);
  }

  @Test
  void pidControllersAreOptional() {
    build(kTalonFXDriveSparkMaxAzimuth);
    // A drive without translation or rotation controllers runs, and only drive to pose reports
    // that they are missing.
    SwerveDriveConfig cfg = baseConfig();
    assertFalse(cfg.getTranslationPID().isPresent());
    assertFalse(cfg.getRotationPID().isPresent());
    SwerveDrive plain = new SwerveDrive(cfg);
    assertDoesNotThrow(plain::updateTelemetry);
    assertDoesNotThrow(plain::simIterate);
    assertDoesNotThrow(plain::resetTranslationPID);
    assertDoesNotThrow(plain::resetRotationPID);
    assertThrows(
        SwerveDriveConfigurationException.class, () -> plain.driveToPoseSetpoint(new Pose2d()));
  }

  @Test
  void initialPoseIsTheStartingPose() {
    build(kTalonFXDriveSparkMaxAzimuth);
    Pose2d pose = drive.getPose();
    assertEquals(0, pose.getX(), 0.01);
    assertEquals(0, pose.getY(), 0.01);
    assertEquals(0, pose.getRotation().getDegrees(), 0.1);
    assertEquals(0, gyroHeadingDegrees(), 0.1);
  }

  @Test
  void resetOdometryMatchesPose() {
    build(kTalonFXDriveSparkMaxAzimuth);
    drive.resetOdometry(new Pose2d(3, 2, Rotation2d.fromDegrees(45)));
    Pose2d pose = drive.getPose();
    assertEquals(3, pose.getX(), 0.01);
    assertEquals(2, pose.getY(), 0.01);
    assertEquals(45, pose.getRotation().getDegrees(), 0.1);
  }

  @Test
  void gyroHeadingWrapsAtHalfARotation() {
    build(kTalonFXDriveSparkMaxAzimuth);
    // The heading is the gyro attitude's yaw, which wraps at half a rotation each way.
    drive.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(270)));
    assertEquals(-90, gyroHeadingDegrees(), 0.1);
    assertEquals(-90, drive.getPose().getRotation().getDegrees(), 0.1);
  }

  @Test
  void fieldRelativeSpeedsUseTheGyroHeading() {
    build(kTalonFXDriveSparkMaxAzimuth);
    // Facing left (90 degrees), driving toward the field's +X is driving to the robot's right.
    drive.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(90)));
    drive.setFieldRelativeChassisSpeeds(new ChassisVelocities(1, 0, 0));
    ChassisVelocities robotRelative = drive.getDesiredChassisSpeeds();
    assertEquals(0, robotRelative.vx, 1e-3);
    assertEquals(-1, robotRelative.vy, 1e-3);
    assertEquals(0, robotRelative.omega, 1e-3);
  }

  @Test
  void stateFromSpeedsForwardDrive() {
    build(kTalonFXDriveSparkMaxAzimuth);
    SwerveModuleVelocity[] states =
        drive.getStateFromRobotRelativeChassisSpeeds(new ChassisVelocities(1, 0, 0));
    for (int i = 0; i < states.length; i++) {
      assertEquals(
          1, states[i].velocity, 0.01, "module " + i + " speed should equal the commanded speed");
      assertEquals(0, states[i].angle.getDegrees(), 1, "module " + i + " should point forward");
    }
  }

  @Test
  void stateFromSpeedsPureRotation() {
    build(kTalonFXDriveSparkMaxAzimuth);
    SwerveModuleVelocity[] states =
        drive.getStateFromRobotRelativeChassisSpeeds(new ChassisVelocities(0, 0, 1));
    for (int i = 0; i < states.length; i++) {
      assertTrue(Math.abs(states[i].velocity) > 0, "module " + i + " should move to rotate");
      assertTrue(
          Math.abs(states[i].angle.getDegrees()) > 1,
          "module " + i + " should not point forward to rotate");
    }
  }

  @Test
  void setRobotRelativeSpeedsDoesNotCrash() {
    build(kTalonFXDriveSparkMaxAzimuth);
    assertDoesNotThrow(() -> drive.setRobotRelativeChassisSpeeds(new ChassisVelocities(1, 0, 0)));
    assertDoesNotThrow(() -> drive.setRobotRelativeChassisSpeeds(new ChassisVelocities()));
  }

  @Test
  void addVisionMeasurementDoesNotCrash() {
    build(kTalonFXDriveSparkMaxAzimuth);
    assertDoesNotThrow(() -> drive.addVisionMeasurement(new Pose2d(1, 1, new Rotation2d()), 0));
  }

  @Test
  void distanceFromPose() {
    build(kTalonFXDriveSparkMaxAzimuth);
    assertEquals(5, drive.getDistanceFromPose(new Pose2d(3, 4, new Rotation2d())).in(Meters), 0.01);
  }

  @Test
  void angleDifferenceFromPoseWrapsAtHalfARotation() {
    build(kTalonFXDriveSparkMaxAzimuth);
    drive.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(170)));
    double difference =
        drive
            .getAngleDifferenceFromPose(new Pose2d(0, 0, Rotation2d.fromDegrees(-170)))
            .in(Degrees);
    assertEquals(
        20, Math.abs(Math.toDegrees(MathUtil.angleModulus(Math.toRadians(difference)))), 0.1);
  }

  @Test
  void driveCommandCallsSpeedSupplier() {
    build(kTalonFXDriveSparkMaxAzimuth);
    AtomicInteger calls = new AtomicInteger();
    TestWithScheduler.schedule(
        drive.drive(
            () -> {
              calls.incrementAndGet();
              return new ChassisVelocities();
            }));
    runFor(0.1);
    assertTrue(calls.get() >= 1);
  }

  @Test
  void driveCommandRequiresTheSubsystem() {
    build(kTalonFXDriveSparkMaxAzimuth);
    Command command = drive.drive(ChassisVelocities::new);
    assertTrue(command.getRequirements().contains(subsystem));
    assertSame(subsystem, drive.getSubsystem());
  }

  @Test
  void secondDriveCommandInterruptsFirst() {
    build(kTalonFXDriveSparkMaxAzimuth);
    AtomicInteger firstCalls = new AtomicInteger();
    AtomicInteger secondCalls = new AtomicInteger();
    Command first =
        drive.drive(
            () -> {
              firstCalls.incrementAndGet();
              return new ChassisVelocities();
            });
    Command second =
        drive.drive(
            () -> {
              secondCalls.incrementAndGet();
              return new ChassisVelocities();
            });

    TestWithScheduler.schedule(first);
    runFor(0.04);
    int firstCallsAtInterrupt = firstCalls.get();
    assertNotEquals(0, firstCallsAtInterrupt, "the first command should run");

    TestWithScheduler.schedule(second);
    runFor(0.04);
    assertTrue(secondCalls.get() >= 1, "the second command should run after interrupting");
    assertEquals(firstCallsAtInterrupt, firstCalls.get(), "the first command should be cancelled");
  }
}
