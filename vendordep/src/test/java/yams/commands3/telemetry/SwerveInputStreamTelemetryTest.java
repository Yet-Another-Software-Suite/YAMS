// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands3.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.preferences.Preferences;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.config.SwerveDriveConfig;
import yams.commands3.swerve.SwerveDrive;
import yams.commands3.swerve.SwerveInputStream;
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
 * Tests live tuning of a {@link SwerveInputStream} through {@link SwerveInputStreamTelemetry}: the table shows the
 * stream's configuration, valid dashboard edits are applied to the stream and change its output, invalid edits are
 * replaced with the stream's value, and changes made in code are published without being overridden.
 *
 * <p>The drive is a simulated four module drive with a 4.5 m/s, 540 deg/s maximum chassis speed and a rotation
 * controller, built once for the class since the streams only read it. Each test uses its own
 * {@link NetworkTableInstance}; a second publisher on a topic stands in for the dashboard.
 */
public class SwerveInputStreamTelemetryTest {
  private static final double kTolerance = 1e-9;
  private static final double kConfigMaxLinear = 4.5;
  private static final double kConfigMaxAngular = DegreesPerSecond.of(540).in(RadiansPerSecond);

  /** Mechanism the drive's motor controllers belong to. */
  private static final Mechanism kMechanism = new Mechanism() {};
  private static final List<SmartMotorController> motorControllers = new ArrayList<>();
  private static SwerveDrive drive;

  private NetworkTableInstance instance;
  private NetworkTable table;
  private SwerveInputStreamTelemetry telemetry;

  /** Controller axes read by the stream. */
  private double forward;
  private double left;
  private double rotation;

  private static SwerveModule createModule(String name, double frontInches, double leftInches) {
    SmartMotorControllerConfig driveCfg = new SmartMotorControllerConfig(kMechanism)
        .withWheelDiameter(Inches.of(4))
        .withClosedLoopController(0.4, 0, 0)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(6.75)))
        .withTelemetry(name + "Drive", TelemetryVerbosity.LOW);
    SmartMotorControllerConfig azimuthCfg = new SmartMotorControllerConfig(kMechanism)
        .withClosedLoopController(50, 0, 0.5)
        .withContinuousWrapping(Radians.of(-Math.PI), Radians.of(Math.PI))
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(12.8)))
        .withTelemetry(name + "Azimuth", TelemetryVerbosity.LOW);
    SmartMotorController driveMotor = new SparkWrapper(DeviceCreator.createSparkMax(), DCMotor.getNEO(1), driveCfg);
    SmartMotorController azimuthMotor = new SparkWrapper(DeviceCreator.createSparkMax(), DCMotor.getNEO(1), azimuthCfg);
    motorControllers.add(driveMotor);
    motorControllers.add(azimuthMotor);
    return new SwerveModule(new SwerveModuleConfig(driveMotor, azimuthMotor)
                                .withAbsoluteEncoder(() -> Degrees.of(0))
                                .withLocation(new Translation2d(Inches.of(frontInches), Inches.of(leftInches)))
                                .withTelemetry(name, TelemetryVerbosity.LOW));
  }

  @BeforeAll
  static void createDrive() {
    MockHardwareExtension.beforeAll();
    drive = new SwerveDrive(
        new SwerveDriveConfig(kMechanism,
                              createModule("SISTelemetryTestFL", 12, 12),
                              createModule("SISTelemetryTestFR", 12, -12),
                              createModule("SISTelemetryTestBL", -12, 12),
                              createModule("SISTelemetryTestBR", -12, -12))
            .withGyro(Rotation3d::new)
            .withStartingPose(new Pose2d())
            .withMaximumChassisSpeed(MetersPerSecond.of(kConfigMaxLinear), DegreesPerSecond.of(540))
            .withRotationController(new PIDController(1, 0, 0))
            .withTelemetry("SwerveInputStreamTelemetryTest", TelemetryVerbosity.LOW));
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
    instance = NetworkTableInstance.create();
    table = instance.getTable("SwerveInputStream").getSubTable("test");
    forward = 0;
    left = 0;
    rotation = 0;
  }

  @AfterEach
  void tearDown() {
    if (telemetry != null) {
      telemetry.close();
      telemetry = null;
    }
    instance.close();
    MockHardwareExtension.afterAll();
  }

  private SwerveInputStream stream() {
    return new SwerveInputStream(drive, () -> forward, () -> left, () -> rotation);
  }

  private void attach(SwerveInputStream stream) {
    telemetry = new SwerveInputStreamTelemetry(stream, table);
  }

  private double published(String key) {
    return table.getDoubleTopic(key).subscribe(Double.NaN).get();
  }

  private boolean publishedBoolean(String key) {
    return table.getBooleanTopic(key).subscribe(false).get();
  }

  /** Edit a value as the dashboard would. */
  private void dashboard(String key, double value) {
    DoublePublisher publisher = table.getDoubleTopic(key).publish();
    publisher.set(value);
  }

  /** Edit a value as the dashboard would. */
  private void dashboard(String key, boolean value) {
    BooleanPublisher publisher = table.getBooleanTopic(key).publish();
    publisher.set(value);
  }

  @Test
  void publishesTheStreamConfiguration() {
    attach(stream()
               .withDeadband(0.1)
               .withScaleTranslation(0.8)
               .withScaleRotation(0.5)
               .withCubeTranslationControllerAxis()
               .withAllianceRelativeControl());

    assertEquals(0.1, published("deadband"), kTolerance);
    assertEquals(0.8, published("translationScale"), kTolerance);
    assertEquals(0.5, published("rotationScale"), kTolerance);
    assertEquals(kConfigMaxLinear, published("maxLinearVelocity"), kTolerance);
    assertEquals(kConfigMaxAngular, published("maxAngularVelocity"), kTolerance);
    assertTrue(publishedBoolean("translationCube"));
    assertFalse(publishedBoolean("rotationCube"));
    assertTrue(publishedBoolean("allianceRelative"));
    assertFalse(publishedBoolean("robotRelative"));
    assertEquals("ANGULAR_VELOCITY", table.getStringTopic("mode").subscribe("").get());
  }

  @Test
  void dashboardEditsAreAppliedToTheStream() {
    SwerveInputStream stream = stream();
    attach(stream);

    dashboard("deadband", 0.2);
    dashboard("translationScale", 0.6);
    dashboard("rotationScale", 0.4);
    dashboard("maxLinearVelocity", 3.0);
    dashboard("maxAngularVelocity", 5.0);
    dashboard("translationCube", true);
    dashboard("rotationCube", true);
    dashboard("allianceRelative", true);
    dashboard("robotRelative", true);
    telemetry.update();

    assertEquals(0.2, stream.getAxisDeadband(), kTolerance);
    assertEquals(0.6, stream.getTranslationAxisScale(), kTolerance);
    assertEquals(0.4, stream.getOmegaAxisScale(), kTolerance);
    assertEquals(3.0, stream.getMaximumChassisLinearVelocity().in(MetersPerSecond), kTolerance);
    assertEquals(5.0, stream.getMaximumChassisAngularVelocity().in(RadiansPerSecond), kTolerance);
    assertTrue(stream.isTranslationCubeEnabled());
    assertTrue(stream.isOmegaCubeEnabled());
    assertTrue(stream.isAllianceRelativeEnabled());
    assertTrue(stream.isRobotRelativeEnabled());

    // The edits stay applied on later loops.
    telemetry.update();
    assertEquals(0.2, stream.getAxisDeadband(), kTolerance);
    assertEquals(0.2, published("deadband"), kTolerance);
    assertTrue(stream.isRobotRelativeEnabled());

    // Turning a feature back off on the dashboard turns it off in the stream.
    dashboard("translationCube", false);
    telemetry.update();
    assertFalse(stream.isTranslationCubeEnabled());
  }

  @Test
  void tunedMaximumLinearVelocityOverridesTheDriveConfig() {
    SwerveInputStream stream = stream();
    attach(stream);
    forward = 1;
    assertEquals(kConfigMaxLinear, stream.get().vx, kTolerance);

    dashboard("maxLinearVelocity", 2.0);
    telemetry.update();
    assertEquals(2.0, stream.get().vx, kTolerance);
  }

  @Test
  void tunedMaximumAngularVelocityOverridesTheDriveConfig() {
    SwerveInputStream stream = stream();
    attach(stream);
    rotation = 1;
    assertEquals(kConfigMaxAngular, stream.get().omega, kTolerance);

    dashboard("maxAngularVelocity", 3.0);
    telemetry.update();
    assertEquals(3.0, stream.get().omega, kTolerance);
  }

  @Test
  void tunedDeadbandChangesTheOutput() {
    SwerveInputStream stream = stream();
    attach(stream);
    forward = 0.3;
    assertTrue(stream.get().vx > 0);

    dashboard("deadband", 0.5);
    telemetry.update();
    assertEquals(0, stream.get().vx, kTolerance);
  }

  @Test
  void tunedScalesChangeTheOutput() {
    SwerveInputStream stream = stream();
    attach(stream);
    forward = 1;
    rotation = 1;

    dashboard("translationScale", 0.5);
    dashboard("rotationScale", 0.25);
    telemetry.update();
    var velocities = stream.get();
    assertEquals(0.5 * kConfigMaxLinear, velocities.vx, kTolerance);
    assertEquals(0.25 * kConfigMaxAngular, velocities.omega, kTolerance);
  }

  @Test
  void tunedCubingChangesTheOutput() {
    SwerveInputStream stream = stream();
    attach(stream);
    rotation = 0.5;

    dashboard("rotationCube", true);
    telemetry.update();
    assertEquals(0.125 * kConfigMaxAngular, stream.get().omega, kTolerance);
  }

  @ParameterizedTest(name = "{0} = {1}")
  @CsvSource({
      "deadband, -0.1",
      "deadband, 1.0",
      "deadband, NaN",
      "translationScale, 0",
      "translationScale, 1.5",
      "rotationScale, -0.5",
      "rotationScale, NaN",
      "maxLinearVelocity, 0",
      "maxLinearVelocity, -1",
      "maxLinearVelocity, Infinity",
      "maxAngularVelocity, 0",
      "maxAngularVelocity, NaN",
  })
  void invalidDashboardValuesAreReplacedWithTheStreamValue(String key, double value) {
    attach(stream().withDeadband(0.1).withScaleTranslation(0.8).withScaleRotation(0.5));
    double before = published(key);

    dashboard(key, value);
    telemetry.update();

    assertEquals(before, published(key), kTolerance, "dashboard shows the stream's value again");
    telemetry.update();
    assertEquals(before, published(key), kTolerance, "the stream is unchanged");
  }

  @Test
  void codeChangesArePublishedAndNotOverridden() {
    SwerveInputStream stream = stream().withScaleTranslation(0.8);
    attach(stream);

    // E.g. a slow mode binding changing the scale while driving.
    stream.withScaleTranslation(0.4);
    telemetry.update();
    assertEquals(0.4, published("translationScale"), kTolerance);
    assertEquals(0.4, stream.getTranslationAxisScale(), kTolerance);

    stream.withScaleTranslation(0.8);
    telemetry.update();
    assertEquals(0.8, published("translationScale"), kTolerance);
    assertEquals(0.8, stream.getTranslationAxisScale(), kTolerance);
  }

  @Test
  void supplierControlledFeaturesAreNotOverridden() {
    boolean[] allianceRelative = {false};
    SwerveInputStream stream = stream().withAllianceRelativeControl(() -> allianceRelative[0]);
    attach(stream);

    allianceRelative[0] = true;
    telemetry.update();
    assertTrue(publishedBoolean("allianceRelative"));

    // The stream still follows the supplier: the telemetry did not replace it with a fixed value.
    allianceRelative[0] = false;
    assertFalse(stream.isAllianceRelativeEnabled());
    telemetry.update();
    assertFalse(publishedBoolean("allianceRelative"));
  }

  @Test
  void publishesTheCurrentMode() {
    boolean[] headingControl = {false};
    SwerveInputStream stream = SwerveInputStream.of(drive, () -> forward, () -> left)
        .withControllerHeadingAxis(() -> 0, () -> 1)
        .withHeadingControl(() -> headingControl[0]);
    attach(stream);

    headingControl[0] = true;
    stream.get();
    telemetry.update();
    assertEquals("HEADING", table.getStringTopic("mode").subscribe("").get());
  }

  @Test
  void closeStopsPublishing() {
    attach(stream());
    assertTrue(table.getTopic("deadband").exists());
    assertTrue(table.getTopic("mode").exists());

    telemetry.close();
    telemetry = null;
    assertFalse(table.getTopic("deadband").exists());
    assertFalse(table.getTopic("robotRelative").exists());
    assertFalse(table.getTopic("mode").exists());
  }

  @Test
  void withMaximumVelocityOverridesTheDriveConfig() {
    forward = 1;
    rotation = 1;
    assertEquals(kConfigMaxLinear, stream().get().vx, kTolerance, "defaults to the drive config");

    var velocities = stream()
        .withMaximumLinearVelocity(MetersPerSecond.of(2))
        .withMaximumAngularVelocity(RadiansPerSecond.of(1))
        .get();
    assertEquals(2, velocities.vx, kTolerance);
    assertEquals(1, velocities.omega, kTolerance);
  }
}
