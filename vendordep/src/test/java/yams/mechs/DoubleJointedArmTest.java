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
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;
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

  /** Over the shoulder, so the arm can't swing over the top. */
  private static final ExclusionZone kAbove =
      new ExclusionZone(new Translation2d(-0.25, 0.15), new Translation2d(0.25, 2));

  private static int offset = 0;

  private SmartMotorControllerTestSubsystem subsystem;
  private SmartMotorController lower;
  private SmartMotorController upper;

  private SmartMotorController sparkMax(String name, double length, double mass, double start) {
    SmartMotorControllerConfig config =
        (SmartMotorControllerConfig)
            new SmartMotorControllerConfig(subsystem)
                .withClosedLoopController(100, 0, 2)
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4, 5)))
                .withZeroPower(MotorMode.BRAKE)
                .withStatorCurrentLimit(Amps.of(40))
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withStartingPosition(Degrees.of(start))
                .withMomentOfInertia(Meters.of(length), Kilograms.of(mass))
                .withTelemetry(name + "(" + offset + ")", TelemetryVerbosity.HIGH);
    return new SparkWrapper(DeviceCreator.createSparkMax(), DCMotor.getNEO(1), config);
  }

  private DoubleJointedArm arm(double start, ExclusionZone... zones) {
    offset++;
    subsystem = new SmartMotorControllerTestSubsystem();
    lower = sparkMax("DJALower", kLowerLength, 1, start);
    upper = sparkMax("DJAUpper", kUpperLength, 0.5, start);
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
    DoubleJointedArm arm = new DoubleJointedArm(config);
    subsystem.mechSimPeriodic = arm::simIterate;
    subsystem.mechUpdateTelemetry = arm::updateTelemetry;
    subsystem.testRunning = true;
    return arm;
  }

  /** Whether the simulated arm touches a zone, by sampling points along both segments and at the elbow. */
  private boolean simulatedArmTouches(ExclusionZone zone) {
    double lowerAngle = lower.getSimSupplier().orElseThrow().getMechanismPosition().in(Degrees);
    double upperAngle = upper.getSimSupplier().orElseThrow().getMechanismPosition().in(Degrees);
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
}
