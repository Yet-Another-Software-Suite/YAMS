// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkMax;
import java.util.function.DoubleSupplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.preferences.Preferences;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.motorcontrollers.remote.TalonFXSWrapper;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;

/**
 * Tests that an {@link ArmFeedforward}'s kV and kA, which are per radian, reach a Talon's or a
 * SPARK's onboard feedforward per rotation, as the C++ wrappers already do. The other feedforwards
 * are per rotation already.
 */
public class ArmFeedforwardUnitsTest {
  /** Phoenix stores gains in fixed point, so 0.1 reads back as 0.0996. A 2π error is far larger. */
  private static final double kTolerance = 0.01;
  /** The radians in one rotation. */
  private static final double kRadiansPerRotation = Rotations.of(1).in(Radians);
  /** Motor rotations per mechanism rotation. */
  private static final double kGearRatio = 12;

  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
  }

  @AfterEach
  void endTest() {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  private static SmartMotorControllerConfig config(String name) {
    return new yams.commands2.config.SmartMotorControllerConfig()
        .withSubsystem(new SmartMotorControllerTestSubsystem())
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withStatorCurrentLimit(Amps.of(40))
        .withZeroPower(MotorMode.BRAKE)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(1, 0, 0)
        .withTelemetry("ArmFeedforwardUnitsTest " + name, TelemetryVerbosity.LOW);
  }

  private static Slot0Configs slot0(SmartMotorController smc) {
    final Slot0Configs slot0 = new Slot0Configs();
    if (smc.getMotorController() instanceof TalonFXS talonFXS) {
      DeviceCreator.refreshConfig(() -> talonFXS.getConfigurator().refresh(slot0, 1.0));
    } else {
      DeviceCreator.refreshConfig(() -> ((TalonFX) smc.getMotorController()).getConfigurator().refresh(slot0, 1.0));
    }
    return slot0;
  }

  private static void assertTalonArmGains(SmartMotorController smc, double kV, double kA, String when) {
    final Slot0Configs slot0 = slot0(smc);
    assertEquals(0.1, slot0.kS, kTolerance, when + ": kS is in volts");
    assertEquals(0.2, slot0.kG, kTolerance, when + ": kG is in volts");
    assertEquals(kV * kRadiansPerRotation, slot0.kV, kTolerance, when + ": kV per rotation");
    assertEquals(kA * kRadiansPerRotation, slot0.kA, kTolerance, when + ": kA per rotation");
  }

  @Test
  void talonFXGetsArmGainsPerRotation() {
    final var smc = new TalonFXWrapper(DeviceCreator.createTalonFX(), DCMotor.getKrakenX60(1),
        config("TalonFX").withFeedforward(new ArmFeedforward(0.1, 0.2, 1.0, 0.5)));
    assertTalonArmGains(smc, 1.0, 0.5, "TalonFX at startup");
    smc.setKv(2.0);
    smc.setKa(0.25);
    assertTalonArmGains(smc, 2.0, 0.25, "TalonFX after setKv and setKa");
    smc.setFeedforward(0.1, 3.0, 0.75, 0.2);
    assertTalonArmGains(smc, 3.0, 0.75, "TalonFX after setFeedforward");
  }

  @Test
  void talonFXSGetsArmGainsPerRotation() {
    final var smc = new TalonFXSWrapper(DeviceCreator.createTalonFXS(), DCMotor.getNEO(1),
        config("TalonFXS").withFeedforward(new ArmFeedforward(0.1, 0.2, 1.0, 0.5)));
    assertTalonArmGains(smc, 1.0, 0.5, "TalonFXS at startup");
    smc.setKv(2.0);
    smc.setKa(0.25);
    assertTalonArmGains(smc, 2.0, 0.25, "TalonFXS after setKv and setKa");
    smc.setFeedforward(0.1, 3.0, 0.75, 0.2);
    assertTalonArmGains(smc, 3.0, 0.75, "TalonFXS after setFeedforward");
  }

  @Test
  void talonFXKeepsSimpleGainsPerRotation() {
    final var smc = new TalonFXWrapper(DeviceCreator.createTalonFX(), DCMotor.getKrakenX60(1),
        config("TalonFX simple").withFeedforward(new SimpleMotorFeedforward(0.1, 1.0, 0.5)));
    final Slot0Configs slot0 = slot0(smc);
    assertEquals(1.0, slot0.kV, kTolerance, "a SimpleMotorFeedforward's kV is per rotation already");
    assertEquals(0.5, slot0.kA, kTolerance, "a SimpleMotorFeedforward's kA is per rotation already");
  }

  /** Wait for a SPARK's configuration, which is applied asynchronously and stored as floats. */
  private static void assertSpark(double expected, DoubleSupplier actual, String what) throws InterruptedException {
    assertTrue(
        AbsoluteEncoderCases.eventually(() -> Math.abs(actual.getAsDouble() - expected) <= 1e-4 * Math.max(1, Math.abs(expected))),
        what + ": expected " + expected + " on the SPARK but was " + actual.getAsDouble());
  }

  @Test
  void sparkGetsArmGainsPerRotation() throws InterruptedException {
    final SparkMax spark = DeviceCreator.createSparkMax();
    final var smc = new SparkWrapper(spark, DCMotor.getNEO(1),
        config("SPARK").withFeedforward(new ArmFeedforward(0.1, 0.2, 1.0, 0.5)));
    // The SPARK's kV and kA are per RPM of its encoder, which turns kGearRatio times per mechanism turn.
    final double rpmPerMechanismRotationPerSecond = kGearRatio * 60;
    final var feedForward = spark.configAccessor.closedLoop.feedForward;
    assertSpark(1.0 * kRadiansPerRotation / rpmPerMechanismRotationPerSecond, feedForward::getkV, "SPARK kV at startup");
    assertSpark(0.5 * kRadiansPerRotation / rpmPerMechanismRotationPerSecond, feedForward::getkA, "SPARK kA at startup");
    smc.setKv(2.0);
    assertSpark(2.0 * kRadiansPerRotation / rpmPerMechanismRotationPerSecond, feedForward::getkV, "SPARK kV after setKv");
    smc.setFeedforward(0.1, 3.0, 0.75, 0.2);
    assertSpark(3.0 * kRadiansPerRotation / rpmPerMechanismRotationPerSecond, feedForward::getkV, "SPARK kV after setFeedforward");
    assertSpark(0.75 * kRadiansPerRotation / rpmPerMechanismRotationPerSecond, feedForward::getkA, "SPARK kA after setFeedforward");
  }
}
