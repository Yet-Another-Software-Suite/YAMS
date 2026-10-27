// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechs;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import com.revrobotics.spark.SparkMax;
import java.util.List;
import java.util.function.Consumer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.preferences.Preferences;
import org.wpilib.units.measure.Angle;
import org.wpilib.util.Pair;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.DoubleJointedArm;
import yams.commands2.telemetry.SmartMotorControllerCommandRegistry;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.mechanisms.config.DoubleJointedArmConfig;
import yams.core.mechanisms.config.DoubleJointedArmConfig.ExclusionZone;
import yams.core.mechanisms.config.DoubleJointedArmConfig.GoalInExclusionZone;
import yams.core.mechanisms.config.DoubleJointedArmConfig.UpperAngleReference;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;
import yams.helpers.TestWithScheduler;

/** A simulated double jointed arm moving around an exclusion zone, and its kinematics. */
public class DoubleJointedArmTest {
  private static final double kLowerLength = 0.6;
  private static final double kUpperLength = 0.5;
  private static final double kLowerMass = 1;
  private static final double kUpperMass = 0.5;

  /** Over the shoulder, so the arm can't swing over the top. */
  private static final ExclusionZone kAbove =
      new ExclusionZone(new Translation2d(-0.25, 0.15), new Translation2d(0.25, 2));

  private static int offset = 0;

  private SmartMotorControllerTestSubsystem subsystem;
  private SmartMotorController lower;
  private SmartMotorController upper;

  /** Whether the upper joint angle is from the lower arm, for the simulated arm's angles. */
  private boolean upperFromLowerArm = false;

  private SmartMotorController sparkMax(
      String name, double length, double mass, double start, double kP, double kG) {
    SmartMotorControllerConfig config =
        (SmartMotorControllerConfig)
            new SmartMotorControllerConfig(subsystem)
                .withClosedLoopController(kP, 0, kP / 50)
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4, 5)))
                .withZeroPower(MotorMode.BRAKE)
                .withStatorCurrentLimit(Amps.of(40))
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withStartingPosition(Degrees.of(start))
                .withMomentOfInertia(Meters.of(length), Kilograms.of(mass))
                .withTelemetry(name + "(" + offset + ")", TelemetryVerbosity.HIGH);
    if (kG != 0) {
      config.withFeedforward(new ArmFeedforward(0, kG, 0, 0));
    }
    return new SparkWrapper(DeviceCreator.createSparkMax(), DCMotor.getNEO(1), config);
  }

  private DoubleJointedArm arm(double start, ExclusionZone... zones) {
    return arm(start, start, 100, 0, 0, config -> {}, zones);
  }

  /**
   * A simulated arm on two SPARK MAXes.
   *
   * @param lowerStart Starting lower joint angle, in degrees.
   * @param upperStart Starting upper joint angle, in degrees.
   * @param kP         Both joints' kP.
   * @param lowerKg    The lower joint's kG, or 0 for no ArmFeedforward.
   * @param upperKg    The upper joint's kG, or 0 for no ArmFeedforward.
   * @param customize  Changes to the arm's config.
   * @param zones      Exclusion zones.
   */
  private DoubleJointedArm arm(
      double lowerStart,
      double upperStart,
      double kP,
      double lowerKg,
      double upperKg,
      Consumer<DoubleJointedArmConfig> customize,
      ExclusionZone... zones) {
    offset++;
    subsystem = new SmartMotorControllerTestSubsystem();
    lower = sparkMax("DJALower", kLowerLength, kLowerMass, lowerStart, kP, lowerKg);
    upper = sparkMax("DJAUpper", kUpperLength, kUpperMass, upperStart, kP, upperKg);
    subsystem.setSMC(lower);
    DoubleJointedArmConfig config =
        new DoubleJointedArmConfig(
                new ArmConfig()
                        .withLength(Meters.of(kLowerLength))
                        .withHardLimits(Degrees.of(-270), Degrees.of(270)),
                    lower,
                new ArmConfig()
                        .withLength(Meters.of(kUpperLength))
                        .withHardLimits(Degrees.of(-270), Degrees.of(270)),
                    upper)
            .withExclusionZoneClearance(Meters.of(0.03))
            .withPathTolerance(Degrees.of(3));
    for (ExclusionZone zone : zones) {
      config.withExclusionZone(zone.min(), zone.max());
    }
    customize.accept(config);
    upperFromLowerArm = config.getUpperAngleReference() == UpperAngleReference.LOWER_ARM;
    DoubleJointedArm arm = new DoubleJointedArm(config);
    subsystem.mechSimPeriodic = arm::simIterate;
    subsystem.mechUpdateTelemetry = arm::updateTelemetry;
    subsystem.testRunning = true;
    return arm;
  }

  /** Voltage holding a still joint against a torque on its NEO through the 60:1 gearing. */
  private static double holdingVolts(double torque) {
    DCMotor neo = DCMotor.getNEO(1);
    return torque / 60 / neo.Kt * neo.R;
  }

  /** Whether the simulated arm touches a zone, by sampling points along both segments and at the elbow. */
  private boolean simulatedArmTouches(ExclusionZone zone) {
    double lowerAngle = lower.getSimSupplier().orElseThrow().getMechanismPosition().in(Degrees);
    double upperAngle =
        upper.getSimSupplier().orElseThrow().getMechanismPosition().in(Degrees)
            + (upperFromLowerArm ? lowerAngle : 0);
    double elbowX = kLowerLength * Math.cos(Math.toRadians(lowerAngle));
    double elbowY = kLowerLength * Math.sin(Math.toRadians(lowerAngle));
    for (int k = 0; k <= 100; k++) {
      double t = k / 100.0;
      double[][] points = {
        {elbowX * t, elbowY * t},
        {
          elbowX + kUpperLength * Math.cos(Math.toRadians(upperAngle)) * t,
          elbowY + kUpperLength * Math.sin(Math.toRadians(upperAngle)) * t
        }
      };
      for (double[] point : points) {
        if (point[0] >= zone.min().getX()
            && point[0] <= zone.max().getX()
            && point[1] >= zone.min().getY()
            && point[1] <= zone.max().getY()) {
          return true;
        }
      }
    }
    return false;
  }

  @BeforeEach
  void beforeTest() {
    MockHardwareExtension.beforeAll();
    TestWithScheduler.schedulerStart();
    TestWithScheduler.schedulerClear();
  }

  @AfterEach
  void afterTest() {
    if (subsystem != null) {
      SmartMotorControllerCommandRegistry.removeCommands(subsystem);
      CommandScheduler.getInstance().unregisterSubsystem(subsystem);
    }
    for (SmartMotorController smc : new SmartMotorController[] {lower, upper}) {
      if (smc != null) {
        smc.close();
        DeviceCreator.silence(smc);
        ((SparkMax) smc.getMotorController()).close();
      }
    }
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
    TestWithScheduler.schedulerClear();
  }

  @Test
  void inverseKinematicsReachesThePosition() {
    DoubleJointedArm arm = arm(0);
    for (Translation2d target :
        new Translation2d[] {
          new Translation2d(0.7, 0.3), new Translation2d(-0.4, 0.6), new Translation2d(0.2, -0.8)
        }) {
      for (boolean invert : new boolean[] {false, true}) {
        Pair<Angle, Angle> angles = arm.getAnglesForPosition(target, invert);
        double x =
            kLowerLength * Math.cos(angles.getFirst().in(Degrees) * Math.PI / 180)
                + kUpperLength * Math.cos(angles.getSecond().in(Degrees) * Math.PI / 180);
        double y =
            kLowerLength * Math.sin(angles.getFirst().in(Degrees) * Math.PI / 180)
                + kUpperLength * Math.sin(angles.getSecond().in(Degrees) * Math.PI / 180);
        assertEquals(
            target.getX(), x, 1e-9, "x of " + target + (invert ? " with the elbow inverted" : ""));
        assertEquals(
            target.getY(), y, 1e-9, "y of " + target + (invert ? " with the elbow inverted" : ""));
      }
    }
  }

  /** Whether the arm at these angles, in degrees, touches a zone, by sampling points along both segments. */
  private static boolean touches(double lowerAngle, double upperAngle, ExclusionZone zone) {
    double elbowX = kLowerLength * Math.cos(Math.toRadians(lowerAngle));
    double elbowY = kLowerLength * Math.sin(Math.toRadians(lowerAngle));
    for (int k = 0; k <= 100; k++) {
      double t = k / 100.0;
      double[][] points = {
        {elbowX * t, elbowY * t},
        {
          elbowX + kUpperLength * Math.cos(Math.toRadians(upperAngle)) * t,
          elbowY + kUpperLength * Math.sin(Math.toRadians(upperAngle)) * t
        }
      };
      for (double[] point : points) {
        if (point[0] >= zone.min().getX()
            && point[0] <= zone.max().getX()
            && point[1] >= zone.min().getY()
            && point[1] <= zone.max().getY()) {
          return true;
        }
      }
    }
    return false;
  }

  @Test
  void theArmFoldsPastAnExclusionZone() {
    // A wall in front of the arm, which only the stretched out arm reaches.
    ExclusionZone wall =
        new ExclusionZone(new Translation2d(0.75, -0.3), new Translation2d(0.9, 0.3));
    DoubleJointedArm arm = arm(30, wall);
    // Moving both joints straight from 30 to -30 degrees would sweep the arm through the wall.
    boolean straightTouches = false;
    for (double angle = 30; angle >= -30; angle -= 1) {
      straightTouches |= touches(angle, angle, wall);
    }
    assertTrue(
        straightTouches, "the straight move goes through the wall, so the arm must go around it");
    TestWithScheduler.schedule(arm.setAngle(Degrees.of(-30), Degrees.of(-30)));

    boolean[] touched = {false};
    TestWithScheduler.cycle(Seconds.of(6), () -> touched[0] |= simulatedArmTouches(wall));

    assertFalse(touched[0], "the arm entered the exclusion zone");
    assertEquals(-30, arm.getLowerAngle().in(Degrees), 5);
    assertEquals(-30, arm.getUpperAngle().in(Degrees), 5);
  }

  @Test
  void theArmHoldsWhenTheGoalIsInAnExclusionZone() {
    DoubleJointedArm arm = arm(0, kAbove);
    assertTrue(arm.isInExclusionZone(Degrees.of(90), Degrees.of(90)));
    TestWithScheduler.schedule(arm.setAngle(Degrees.of(90), Degrees.of(90)));

    boolean[] touched = {false};
    TestWithScheduler.cycle(Seconds.of(2), () -> touched[0] |= simulatedArmTouches(kAbove));

    assertFalse(touched[0], "the arm entered the exclusion zone");
    assertEquals(0, arm.getLowerAngle().in(Degrees), 5);
  }

  /** The arm's NetworkTables table under a top level table. */
  private static NetworkTable table(String top, DoubleJointedArm arm) {
    return NetworkTableInstance.getDefault().getTable(top).getSubTable(arm.getName());
  }

  @Test
  void anElbowMotorOnTheLowerArmFoldsPastAnExclusionZone() {
    // The upper joint is from the lower arm: 0 is straight, so this starts and ends straight, 30
    // degrees up then down.
    ExclusionZone wall =
        new ExclusionZone(new Translation2d(0.75, -0.3), new Translation2d(0.9, 0.3));
    DoubleJointedArm arm =
        arm(
            30,
            0,
            100,
            0,
            0,
            config -> config.withUpperAngleReference(UpperAngleReference.LOWER_ARM),
            wall);
    assertEquals(30, arm.getUpperAngleFromHorizontal().in(Degrees), 0.5);
    TestWithScheduler.schedule(arm.setAngle(Degrees.of(-30), Degrees.of(0)));

    boolean[] touched = {false};
    TestWithScheduler.cycle(Seconds.of(6), () -> touched[0] |= simulatedArmTouches(wall));

    assertFalse(touched[0], "the arm entered the exclusion zone");
    assertEquals(-30, arm.getLowerAngle().in(Degrees), 5);
    assertEquals(0, arm.getUpperAngle().in(Degrees), 5);
  }

  @Test
  void kGIsCompensatedForTheUpperArm() throws InterruptedException {
    // The elbow motor on the lower arm, and each kG tuned with the arm straight out horizontally.
    double lowerKg =
        holdingVolts(
            (kLowerMass * kLowerLength / 2
                    + kUpperMass * kLowerLength
                    + kUpperMass * kUpperLength / 2)
                * 9.8);
    double upperKg = holdingVolts(kUpperMass * kUpperLength / 2 * 9.8);
    // A low kP, so the arm only holds still where kG holds it: uncompensated, each joint sags over
    // 2 degrees here.
    DoubleJointedArm arm =
        arm(
            0,
            0,
            10,
            lowerKg,
            upperKg,
            config -> config.withUpperAngleReference(UpperAngleReference.LOWER_ARM));
    // The lower arm at 60 degrees and the upper arm horizontal.
    TestWithScheduler.schedule(arm.setAngle(Degrees.of(60), Degrees.of(-60)));

    TestWithScheduler.cycle(Seconds.of(4));

    double lower = Math.toRadians(60);
    assertEquals(
        lowerKg * arm.getDynamics().getKgScale(0, lower, -lower),
        arm.getKg(0),
        lowerKg * 0.05,
        "the shoulder's kG holds the upper arm too");
    assertEquals(
        upperKg * 2,
        arm.getKg(1),
        upperKg * 0.05,
        "the elbow's kG cos(-60°) holds the horizontal upper arm");
    assertEquals(60, arm.getLowerAngle().in(Degrees), 1);
    assertEquals(-60, arm.getUpperAngle().in(Degrees), 1);
  }

  @Test
  void anElbowDrivenFromTheBaseKeepsItsKg() throws InterruptedException {
    double lowerKg =
        holdingVolts((kLowerMass * kLowerLength / 2 + kUpperMass * kLowerLength) * 9.8);
    double upperKg = holdingVolts(kUpperMass * kUpperLength / 2 * 9.8);
    DoubleJointedArm arm = arm(0, 0, 10, lowerKg, upperKg, config -> {});
    TestWithScheduler.schedule(arm.setAngle(Degrees.of(60), Degrees.of(0)));

    TestWithScheduler.cycle(Seconds.of(4));

    assertEquals(lowerKg, arm.getKg(0), 1e-12);
    assertEquals(upperKg, arm.getKg(1), 1e-12);
    assertEquals(60, arm.getLowerAngle().in(Degrees), 1);
    assertEquals(0, arm.getUpperAngle().in(Degrees), 1);
  }

  @Test
  void theArmMovesOnWhenGravityHoldsItFurtherFromAWaypointThanTheTolerance() {
    // Without kG the arm sags about 2 degrees, four times the tolerance, so it never gets within it
    // of a waypoint.
    ExclusionZone wall =
        new ExclusionZone(new Translation2d(0.75, -0.3), new Translation2d(0.9, 0.3));
    DoubleJointedArm arm =
        arm(30, 30, 100, 0, 0, config -> config.withPathTolerance(Degrees.of(0.5)), wall);
    TestWithScheduler.schedule(arm.setAngle(Degrees.of(-30), Degrees.of(-30)));

    boolean[] touched = {false};
    TestWithScheduler.cycle(Seconds.of(6), () -> touched[0] |= simulatedArmTouches(wall));

    assertFalse(touched[0], "the arm entered the exclusion zone");
    assertEquals(-30, arm.getLowerAngle().in(Degrees), 5);
    assertEquals(-30, arm.getUpperAngle().in(Degrees), 5);
  }

  @Test
  void aNullAngleKeepsItsSetpoint() throws InterruptedException {
    DoubleJointedArm arm = arm(0);
    TestWithScheduler.schedule(arm.setAngle(Degrees.of(10), Degrees.of(20)));
    TestWithScheduler.cycle(Seconds.of(1));

    TestWithScheduler.schedule(arm.setAngle(null, Degrees.of(40)));
    TestWithScheduler.cycle(Seconds.of(1));

    assertEquals(10, Math.toDegrees(arm.getPathFollower().getGoalLowerRadians()), 1e-9);
    assertEquals(40, Math.toDegrees(arm.getPathFollower().getGoalUpperRadians()), 1e-9);
  }

  @Test
  void telemetryPublishesTheArmAndLiveTunesThePathTolerance() throws InterruptedException {
    DoubleJointedArm arm =
        arm(
            0,
            0,
            100,
            0,
            0,
            config -> config.withTelemetry("DJATelemetry" + offset, TelemetryVerbosity.HIGH),
            kAbove);
    TestWithScheduler.schedule(arm.setAngle(Degrees.of(-30), Degrees.of(-30)));
    TestWithScheduler.cycle(Seconds.of(1));
    arm.updateTelemetry();

    NetworkTable data = table("Mechanisms", arm);
    NetworkTable tuning = table("Tuning", arm);
    assertTrue(data.getTopic("position").exists());
    assertFalse(data.getEntry("inExclusionZone").getBoolean(true));
    assertEquals(-30, data.getEntry("path/goal/lower").getDouble(Double.NaN), 1e-9);
    assertEquals(1, data.getEntry("path/waypoints").getDouble(Double.NaN), 1e-9);
    assertEquals(3, tuning.getEntry("path/tolerance").getDouble(Double.NaN), 1e-9);
    assertFalse(
        tuning.getSubTable("Live Tuning").getKeys().isEmpty(), "Live Tuning on the dashboard");

    var telemetry = arm.getArmTelemetry().orElseThrow();
    tuning.getEntry("path/tolerance").setDouble(4.5);
    telemetry.applyTuningValues(arm);
    assertEquals(4.5, arm.getPathTolerance().in(Degrees), 1e-9);

    // An invalid tolerance is replaced with the arm's.
    tuning.getEntry("path/tolerance").setDouble(-1);
    telemetry.applyTuningValues(arm);
    assertEquals(4.5, arm.getPathTolerance().in(Degrees), 1e-9);
    assertEquals(4.5, tuning.getEntry("path/tolerance").getDouble(Double.NaN), 1e-9);

    // A tolerance set in code is published.
    arm.setPathTolerance(Degrees.of(2));
    telemetry.applyTuningValues(arm);
    assertEquals(2, tuning.getEntry("path/tolerance").getDouble(Double.NaN), 1e-9);
    assertEquals(2, arm.getPathTolerance().in(Degrees), 1e-9);
  }

  @Test
  void aDynamicZoneInTheWayIsPlannedAround() throws InterruptedException {
    // Swinging from reaching forward to reaching back under the shoulder, when something moves in
    // under it.
    DoubleJointedArm arm = arm(0, kAbove);
    ExclusionZone under =
        new ExclusionZone(new Translation2d(-0.1, -1.5), new Translation2d(0.1, -0.15));
    TestWithScheduler.schedule(arm.setAngle(Degrees.of(-170), Degrees.of(-170)));
    TestWithScheduler.cycle(Seconds.of(0.04));
    assertTrue(
        arm.getLowerAngle().in(Degrees) > -30,
        "not near it yet: " + arm.getLowerAngle().in(Degrees));

    arm.setDynamicExclusionZones(List.of(under));
    boolean[] touched = {false};
    TestWithScheduler.cycle(
        Seconds.of(3),
        () -> touched[0] |= simulatedArmTouches(under) || simulatedArmTouches(kAbove));

    // Both ways round are blocked now, so the arm stops short of the new zone.
    assertFalse(touched[0], "the arm entered an exclusion zone");
    assertTrue(
        arm.getLowerAngle().in(Degrees) > -90,
        "the arm stopped before swinging under: " + arm.getLowerAngle().in(Degrees));

    // Clearing the zone lets the arm finish.
    arm.setDynamicExclusionZones(List.of());
    TestWithScheduler.cycle(Seconds.of(4));
    assertEquals(-170, arm.getLowerAngle().in(Degrees), 5);
    assertEquals(-170, arm.getUpperAngle().in(Degrees), 5);
  }

  @Test
  void aGoalInAZoneCanGoToTheNearestClearAngles() throws InterruptedException {
    DoubleJointedArm arm =
        arm(
            0,
            0,
            100,
            0,
            0,
            config -> config.withGoalInExclusionZone(GoalInExclusionZone.NEAREST_CLEAR),
            kAbove);
    TestWithScheduler.schedule(arm.setAngle(Degrees.of(90), Degrees.of(90)));

    boolean[] touched = {false};
    TestWithScheduler.cycle(Seconds.of(4), () -> touched[0] |= simulatedArmTouches(kAbove));

    assertFalse(touched[0], "the arm entered the exclusion zone");
    // A lower arm pointing up between about 31 and 149 degrees goes through the zone, so it goes up
    // to just under 31.
    assertTrue(
        arm.getLowerAngle().in(Degrees) > 5 && arm.getLowerAngle().in(Degrees) < 31,
        "the arm went up as far as it can: " + arm.getLowerAngle().in(Degrees));
  }
}
