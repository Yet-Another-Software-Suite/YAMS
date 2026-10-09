// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands2.telemetry;

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
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.preferences.Preferences;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.config.SwerveDriveConfig;
import yams.commands2.swerve.SwerveDrive;
import yams.commands2.swerve.SwerveInputStream;
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
 * Tests {@link SwerveInputStream#withTelemetry(String, TelemetryVerbosity)}: what each {@link TelemetryVerbosity}
 * publishes, that reading the stream publishes it, and live tuning at {@link TelemetryVerbosity#HIGH}: the tuning table
 * shows the stream's configuration, valid dashboard edits are applied to the stream while the {@code Live Tuning}
 * command runs and change its output, invalid edits are replaced with the stream's value, and changes made in code are
 * published without being overridden.
 *
 * <p>The drive is a simulated four module drive with a 4.5 m/s, 540 deg/s maximum chassis speed and a rotation
 * controller, built once for the class since the streams only read it. Each test names its stream uniquely; writing to
 * an entry from the test stands in for the dashboard.
 */
public class SwerveInputStreamTelemetryTest {
  private static final double kTolerance = 1e-9;
  private static final double kConfigMaxLinear = 4.5;
  private static final double kConfigMaxAngular = DegreesPerSecond.of(540).in(RadiansPerSecond);

  /** Subsystem the drive's motor controllers belong to. */
  private static final SubsystemBase kMechanism = new SubsystemBase() {};

  private static final List<SmartMotorController> motorControllers = new ArrayList<>();
  private static SwerveDrive drive;

  /** Numbers each test's stream name. */
  private static int streamCount = 0;

  /** Streams with telemetry, closed after each test. */
  private final List<SwerveInputStream> streams = new ArrayList<>();

  /** This test's stream name. */
  private String name;

  /** Controller axes read by the stream. */
  private double forward;

  private double left;
  private double rotation;

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
                    createModule("SISTelemetryTestFL", 12, 12),
                    createModule("SISTelemetryTestFR", 12, -12),
                    createModule("SISTelemetryTestBL", -12, 12),
                    createModule("SISTelemetryTestBR", -12, -12))
                .withGyro(Rotation3d::new)
                .withStartingPose(new Pose2d())
                .withMaximumChassisSpeed(
                    MetersPerSecond.of(kConfigMaxLinear), DegreesPerSecond.of(540))
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
    CommandScheduler.getInstance().unregisterSubsystem(kMechanism);
    drive = null;
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  @BeforeEach
  void setUp() {
    MockHardwareExtension.beforeAll();
    name = "SISTelemetryTest" + streamCount++;
    forward = 0;
    left = 0;
    rotation = 0;
  }

  @AfterEach
  void tearDown() {
    for (SwerveInputStream stream : streams) {
      stream.getTelemetry().ifPresent(SwerveInputStreamTelemetry::close);
    }
    streams.clear();
    MockHardwareExtension.afterAll();
  }

  private SwerveInputStream stream() {
    return SwerveInputStream.of(drive, () -> forward, () -> left, () -> rotation);
  }

  /** Enable the stream's telemetry under this test's name. */
  private SwerveInputStream withTelemetry(SwerveInputStream stream, TelemetryVerbosity verbosity) {
    streams.add(stream);
    return stream.withTelemetry(name, verbosity);
  }

  /** Apply dashboard edits, as one loop of the Live Tuning command does. */
  private static void tune(SwerveInputStream stream) {
    stream.getTelemetry().orElseThrow().applyTuningValues();
  }

  private void runLiveTuning(Command command) {
    CommandScheduler.getInstance().schedule(command);
    CommandScheduler.getInstance().run();
  }

  private void stopLiveTuning(Command command) {
    CommandScheduler.getInstance().cancel(command);
    CommandScheduler.getInstance().run();
  }

  private static NetworkTable dataTable(String streamName) {
    return NetworkTableInstance.getDefault().getTable("SwerveInputStream").getSubTable(streamName);
  }

  private static NetworkTable tuningTable(String streamName) {
    return NetworkTableInstance.getDefault()
        .getTable("Tuning")
        .getSubTable("SwerveInputStream")
        .getSubTable(streamName);
  }

  private double data(String key) {
    return dataTable(name).getEntry(key).getDouble(Double.NaN);
  }

  private String mode() {
    return dataTable(name).getEntry("mode").getString("");
  }

  private double published(String key) {
    return tuningTable(name).getEntry(key).getDouble(Double.NaN);
  }

  private boolean publishedBoolean(String key) {
    return tuningTable(name).getEntry(key).getBoolean(false);
  }

  /** Edit a tuning value as the dashboard would. */
  private void dashboard(String key, double value) {
    tuningTable(name).getEntry(key).setDouble(value);
  }

  /** Edit a tuning value as the dashboard would. */
  private void dashboard(String key, boolean value) {
    tuningTable(name).getEntry(key).setBoolean(value);
  }

  // ---- Verbosity
  // ---------------------------------------------------------------------------------

  @Test
  void lowPublishesOnlyTheMode() {
    SwerveInputStream stream = withTelemetry(stream().withDeadband(0.1), TelemetryVerbosity.LOW);

    assertEquals("ANGULAR_VELOCITY", mode());
    assertFalse(dataTable(name).getTopic("deadband").exists());
    assertTrue(tuningTable(name).getTopics().isEmpty());
    assertTrue(stream.getTelemetry().orElseThrow().getLiveTuningCommand().isEmpty());
  }

  @Test
  void midPublishesTheConfigurationReadOnly() {
    SwerveInputStream stream =
        withTelemetry(stream().withDeadband(0.1).withScaleTranslation(0.8), TelemetryVerbosity.MID);

    assertEquals(0.1, data("deadband"), kTolerance);
    assertEquals(0.8, data("translationScale"), kTolerance);
    assertEquals(kConfigMaxLinear, data("maxLinearVelocity"), kTolerance);
    assertTrue(tuningTable(name).getTopics().isEmpty());
    assertTrue(stream.getTelemetry().orElseThrow().getLiveTuningCommand().isEmpty());

    // Changes made in code are published when the stream is read.
    stream.withScaleTranslation(0.5);
    stream.get();
    assertEquals(0.5, data("translationScale"), kTolerance);
  }

  @Test
  void highPublishesTuningValuesAndTheLiveTuningCommand() {
    SwerveInputStream stream =
        withTelemetry(
            stream()
                .withDeadband(0.1)
                .withScaleTranslation(0.8)
                .withScaleRotation(0.5)
                .withCubeTranslationControllerAxis()
                .withAllianceRelativeControl(),
            TelemetryVerbosity.HIGH);

    assertEquals(0.1, data("deadband"), kTolerance);
    assertEquals(0.1, published("deadband"), kTolerance);
    assertEquals(0.8, published("translationScale"), kTolerance);
    assertEquals(0.5, published("rotationScale"), kTolerance);
    assertEquals(kConfigMaxLinear, published("maxLinearVelocity"), kTolerance);
    assertEquals(kConfigMaxAngular, published("maxAngularVelocity"), kTolerance);
    assertTrue(publishedBoolean("translationCube"));
    assertFalse(publishedBoolean("rotationCube"));
    assertTrue(publishedBoolean("allianceRelative"));
    assertFalse(publishedBoolean("robotRelative"));
    assertTrue(stream.getTelemetry().orElseThrow().getLiveTuningCommand().isPresent());
    assertFalse(
        tuningTable(name).getSubTable("Live Tuning").getTopics().isEmpty(),
        "Live Tuning on the dashboard");
  }

  @Test
  void readingTheStreamPublishesTheMode() {
    boolean[] headingControl = {false};
    SwerveInputStream stream =
        withTelemetry(
            SwerveInputStream.of(drive, () -> forward, () -> left)
                .withControllerHeadingAxis(() -> 0, () -> 1)
                .withHeadingControl(() -> headingControl[0]),
            TelemetryVerbosity.LOW);

    headingControl[0] = true;
    stream.get();
    assertEquals("HEADING", mode());
  }

  // ---- Live tuning
  // -------------------------------------------------------------------------------

  @Test
  void dashboardEditsApplyOnlyWhileLiveTuningRuns() {
    SwerveInputStream stream = withTelemetry(stream(), TelemetryVerbosity.HIGH);
    Command liveTuning = stream.getTelemetry().orElseThrow().getLiveTuningCommand().orElseThrow();

    dashboard("deadband", 0.2);
    stream.get();
    assertEquals(0, stream.getAxisDeadband(), kTolerance, "not applied before Live Tuning runs");

    runLiveTuning(liveTuning);
    assertEquals(0.2, stream.getAxisDeadband(), kTolerance, "applied while Live Tuning runs");

    stopLiveTuning(liveTuning);
    dashboard("deadband", 0.3);
    stream.get();
    assertEquals(0.2, stream.getAxisDeadband(), kTolerance, "not applied after Live Tuning stops");
  }

  @Test
  void dashboardEditsAreAppliedToTheStream() {
    SwerveInputStream stream = withTelemetry(stream(), TelemetryVerbosity.HIGH);

    dashboard("deadband", 0.2);
    dashboard("translationScale", 0.6);
    dashboard("rotationScale", 0.4);
    dashboard("maxLinearVelocity", 3.0);
    dashboard("maxAngularVelocity", 5.0);
    dashboard("translationCube", true);
    dashboard("rotationCube", true);
    dashboard("allianceRelative", true);
    dashboard("robotRelative", true);
    tune(stream);

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
    tune(stream);
    assertEquals(0.2, stream.getAxisDeadband(), kTolerance);
    assertEquals(0.2, published("deadband"), kTolerance);
    assertTrue(stream.isRobotRelativeEnabled());

    // Turning a feature back off on the dashboard turns it off in the stream.
    dashboard("translationCube", false);
    tune(stream);
    assertFalse(stream.isTranslationCubeEnabled());
  }

  @Test
  void tunedMaximumVelocitiesOverrideTheDriveConfig() {
    SwerveInputStream stream = withTelemetry(stream(), TelemetryVerbosity.HIGH);
    forward = 1;
    rotation = 1;
    assertEquals(kConfigMaxLinear, stream.get().vx, kTolerance);

    dashboard("maxLinearVelocity", 2.0);
    dashboard("maxAngularVelocity", 3.0);
    tune(stream);
    var velocities = stream.get();
    assertEquals(2.0, velocities.vx, kTolerance);
    assertEquals(3.0, velocities.omega, kTolerance);
  }

  @Test
  void tunedDeadbandChangesTheOutput() {
    SwerveInputStream stream = withTelemetry(stream(), TelemetryVerbosity.HIGH);
    forward = 0.3;
    assertTrue(stream.get().vx > 0);

    dashboard("deadband", 0.5);
    tune(stream);
    assertEquals(0, stream.get().vx, kTolerance);
  }

  @Test
  void tunedScalesChangeTheOutput() {
    SwerveInputStream stream = withTelemetry(stream(), TelemetryVerbosity.HIGH);
    forward = 1;
    rotation = 1;

    dashboard("translationScale", 0.5);
    dashboard("rotationScale", 0.25);
    tune(stream);
    var velocities = stream.get();
    assertEquals(0.5 * kConfigMaxLinear, velocities.vx, kTolerance);
    assertEquals(0.25 * kConfigMaxAngular, velocities.omega, kTolerance);
  }

  @Test
  void tunedCubingChangesTheOutput() {
    SwerveInputStream stream = withTelemetry(stream(), TelemetryVerbosity.HIGH);
    rotation = 0.5;

    dashboard("rotationCube", true);
    tune(stream);
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
    SwerveInputStream stream =
        withTelemetry(
            stream().withDeadband(0.1).withScaleTranslation(0.8).withScaleRotation(0.5),
            TelemetryVerbosity.HIGH);
    double before = published(key);

    dashboard(key, value);
    tune(stream);

    assertEquals(before, published(key), kTolerance, "dashboard shows the stream's value again");
    tune(stream);
    assertEquals(before, published(key), kTolerance, "the stream is unchanged");
  }

  @Test
  void codeChangesArePublishedAndNotOverridden() {
    SwerveInputStream stream =
        withTelemetry(stream().withScaleTranslation(0.8), TelemetryVerbosity.HIGH);

    // E.g. a slow mode binding changing the scale while driving.
    stream.withScaleTranslation(0.4);
    tune(stream);
    assertEquals(0.4, published("translationScale"), kTolerance);
    assertEquals(0.4, stream.getTranslationAxisScale(), kTolerance);

    stream.withScaleTranslation(0.8);
    tune(stream);
    assertEquals(0.8, published("translationScale"), kTolerance);
    assertEquals(0.8, stream.getTranslationAxisScale(), kTolerance);
  }

  @Test
  void supplierControlledFeaturesAreNotOverridden() {
    boolean[] allianceRelative = {false};
    SwerveInputStream stream =
        withTelemetry(
            stream().withAllianceRelativeControl(() -> allianceRelative[0]),
            TelemetryVerbosity.HIGH);

    allianceRelative[0] = true;
    tune(stream);
    assertTrue(publishedBoolean("allianceRelative"));

    // The stream still follows the supplier: the telemetry did not replace it with a fixed value.
    allianceRelative[0] = false;
    assertFalse(stream.isAllianceRelativeEnabled());
    tune(stream);
    assertFalse(publishedBoolean("allianceRelative"));
  }

  // ---- Lifecycle
  // ---------------------------------------------------------------------------------

  @Test
  void withTelemetryReplacesTheTelemetry() {
    SwerveInputStream stream = withTelemetry(stream(), TelemetryVerbosity.HIGH);
    String firstName = name;
    var first = stream.getTelemetry().orElseThrow();

    name = firstName + "Renamed";
    withTelemetry(stream, TelemetryVerbosity.HIGH);
    assertFalse(stream.getTelemetry().orElseThrow() == first);
    assertFalse(dataTable(firstName).getTopic("mode").exists(), "the first telemetry is closed");
    assertFalse(tuningTable(firstName).getTopic("deadband").exists());
    assertEquals("ANGULAR_VELOCITY", mode());
  }

  @Test
  void cloneHasNoTelemetry() {
    SwerveInputStream stream = withTelemetry(stream(), TelemetryVerbosity.HIGH);
    SwerveInputStream clone = stream.clone();

    assertTrue(clone.getTelemetry().isEmpty());
    assertTrue(stream.getTelemetry().isPresent());
    assertTrue(stream().getTelemetry().isEmpty(), "streams have no telemetry until withTelemetry");
  }

  @Test
  void closeStopsPublishing() {
    SwerveInputStream stream = withTelemetry(stream(), TelemetryVerbosity.HIGH);
    var telemetry = stream.getTelemetry().orElseThrow();
    Command liveTuning = telemetry.getLiveTuningCommand().orElseThrow();
    runLiveTuning(liveTuning);
    assertTrue(dataTable(name).getTopic("deadband").exists());

    telemetry.close();
    assertFalse(CommandScheduler.getInstance().isScheduled(liveTuning), "Live Tuning is canceled");
    assertFalse(dataTable(name).getTopic("mode").exists());
    assertFalse(dataTable(name).getTopic("deadband").exists());
    assertFalse(tuningTable(name).getTopic("deadband").exists());
    assertFalse(tuningTable(name).getTopic("robotRelative").exists());
    assertTrue(
        tuningTable(name).getSubTable("Live Tuning").getTopics().isEmpty(),
        "Live Tuning is removed");
  }

  @Test
  void withMaximumVelocityOverridesTheDriveConfig() {
    forward = 1;
    rotation = 1;
    assertEquals(kConfigMaxLinear, stream().get().vx, kTolerance, "defaults to the drive config");

    var velocities =
        stream()
            .withMaximumLinearVelocity(MetersPerSecond.of(2))
            .withMaximumAngularVelocity(RadiansPerSecond.of(1))
            .get();
    assertEquals(2, velocities.vx, kTolerance);
    assertEquals(1, velocities.omega, kTolerance);
  }
}
