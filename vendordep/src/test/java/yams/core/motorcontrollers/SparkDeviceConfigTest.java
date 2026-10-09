// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Rotations;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.DoubleSupplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.system.DCMotor;
import org.wpilib.preferences.Preferences;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.telemetry.SmartMotorControllerCommandRegistry;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;

/**
 * Tests that values the {@link SparkWrapper} computes reach the SPARK's own configuration, in the
 * units the SPARK uses: soft limits in rotations of its feedback sensor, and the absolute encoder's
 * zero offset after {@link SparkWrapper#setEncoderPosition}.
 */
public class SparkDeviceConfigTest {
  /** 12 motor rotations per mechanism rotation. */
  private static final MechanismGearing kGearing =
      new MechanismGearing(GearBox.fromReductionStages(3, 4));

  private static final double kRatio = 12;
  private static final double kCircumferenceMeters = 0.2;

  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
  }

  @AfterEach
  void endTest() {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  private static SmartMotorControllerConfig baseConfig(String name, MechanismGearing gearing) {
    return new yams.commands2.config.SmartMotorControllerConfig()
        .withSubsystem(new SmartMotorControllerTestSubsystem())
        .withGearing(gearing)
        .withStatorCurrentLimit(Amps.of(40))
        .withZeroPower(MotorMode.BRAKE)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(1, 0, 0)
        .withTelemetry("SparkDeviceConfigTest " + name, yams.core.telemetry.enums.TelemetryVerbosity.LOW);
  }

  private static void close(SmartMotorController smc, SparkMax sparkMax) {
    SmartMotorControllerTestSubsystem subsys =
        (SmartMotorControllerTestSubsystem)
            ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem();
    SmartMotorControllerCommandRegistry.removeCommands(subsys);
    CommandScheduler.getInstance().unregisterSubsystem(subsys);
    subsys.close();
    smc.close();
    DeviceCreator.silence(smc);
    sparkMax.configure(
        new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    sparkMax.close();
  }

  /** Wait for an asynchronously applied SPARK parameter to reach the expected value. */
  private static void assertDevice(double expected, DoubleSupplier actual, String what)
      throws InterruptedException {
    assertTrue(
        AbsoluteEncoderCases.eventually(
            () -> Math.abs(actual.getAsDouble() - expected) <= 1e-4 * Math.max(1, Math.abs(expected))),
        what + ": expected " + expected + " on the SPARK but was " + actual.getAsDouble());
  }

  @Test
  void softLimitsAreInFeedbackSensorRotations() throws InterruptedException {
    final SparkMax sparkMax = DeviceCreator.createSparkMax();
    final SmartMotorController smc =
        new SparkWrapper(
            sparkMax,
            DCMotor.getNEO(1),
            baseConfig("angular", kGearing).withSoftLimits(Rotations.of(-0.5), Rotations.of(0.75)));
    ((SmartMotorControllerTestSubsystem)
            ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem())
        .setSMC(smc);
    try {
      final var softLimit = sparkMax.configAccessor.softLimit;
      // applyConfig
      assertDevice(-0.5 * kRatio, softLimit::getReverseSoftLimit, "applied reverse soft limit");
      assertDevice(0.75 * kRatio, softLimit::getForwardSoftLimit, "applied forward soft limit");

      smc.setMechanismLimits(Rotations.of(-0.25), Rotations.of(0.5));
      assertDevice(-0.25 * kRatio, softLimit::getReverseSoftLimit, "setMechanismLimits reverse");
      assertDevice(0.5 * kRatio, softLimit::getForwardSoftLimit, "setMechanismLimits forward");

      smc.setMechanismUpperLimit(Rotations.of(0.6));
      assertDevice(0.6 * kRatio, softLimit::getForwardSoftLimit, "setMechanismUpperLimit");

      smc.setMechanismLowerLimit(Rotations.of(-0.4));
      assertDevice(-0.4 * kRatio, softLimit::getReverseSoftLimit, "setMechanismLowerLimit");
    } finally {
      close(smc, sparkMax);
    }
  }

  @Test
  void linearSoftLimitsAreInFeedbackSensorRotations() throws InterruptedException {
    final SparkMax sparkMax = DeviceCreator.createSparkMax();
    final SmartMotorController smc =
        new SparkWrapper(
            sparkMax,
            DCMotor.getNEO(1),
            baseConfig("linear", kGearing)
                .withMechanismCircumference(Meters.of(kCircumferenceMeters))
                .withSoftLimits(Meters.of(-0.1), Meters.of(0.2)));
    ((SmartMotorControllerTestSubsystem)
            ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem())
        .setSMC(smc);
    try {
      final var softLimit = sparkMax.configAccessor.softLimit;
      assertDevice(
          -0.1 / kCircumferenceMeters * kRatio,
          softLimit::getReverseSoftLimit,
          "applied reverse soft limit");
      assertDevice(
          0.2 / kCircumferenceMeters * kRatio,
          softLimit::getForwardSoftLimit,
          "applied forward soft limit");

      smc.setMeasurementUpperLimit(Meters.of(0.3));
      assertDevice(
          0.3 / kCircumferenceMeters * kRatio,
          softLimit::getForwardSoftLimit,
          "setMeasurementUpperLimit");

      smc.setMeasurementLowerLimit(Meters.of(-0.05));
      assertDevice(
          -0.05 / kCircumferenceMeters * kRatio,
          softLimit::getReverseSoftLimit,
          "setMeasurementLowerLimit");
    } finally {
      close(smc, sparkMax);
    }
  }

  @Test
  void setEncoderPositionSendsTheAbsoluteEncoderZeroOffset() throws InterruptedException {
    final SparkMax sparkMax = DeviceCreator.createSparkMax();
    final SmartMotorController smc =
        new SparkWrapper(
            sparkMax,
            DCMotor.getNEO(1),
            baseConfig("zero offset", new MechanismGearing(GearBox.fromReductionStages(1)))
                .withExternalEncoder(sparkMax.getAbsoluteEncoder()));
    ((SmartMotorControllerTestSubsystem)
            ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem())
        .setSMC(smc);
    try {
      final var absoluteEncoder = sparkMax.configAccessor.absoluteEncoder;
      assertDevice(0, absoluteEncoder::getZeroOffset, "initial zero offset");

      final double target = -0.3;
      final double expected = smc.getMechanismPosition().in(Rotations) - target;
      smc.setEncoderPosition(Rotations.of(target));
      assertDevice(expected, absoluteEncoder::getZeroOffset, "zero offset after setEncoderPosition");
    } finally {
      close(smc, sparkMax);
    }
  }
}
