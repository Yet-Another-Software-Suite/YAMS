// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.revrobotics.encoder.DetachedEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.ValueSource;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.MathUtil;
import org.wpilib.preferences.Preferences;
import org.wpilib.units.measure.Angle;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.core.exceptions.SmartMotorControllerConfigurationException;
import yams.core.motorcontrollers.AbsoluteEncoderCases.Controller;
import yams.core.motorcontrollers.AbsoluteEncoderCases.Encoder;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;

/**
 * Tests {@link SmartMotorControllerConfig#withExternalEncoderZeroOffset}: the zero offset is the
 * angle the absolute encoder reads, before the offset, where the mechanism is at zero.
 *
 * <p>For every absolute external encoder on every motor controller ({@link AbsoluteEncoderCases})
 * and under every closed loop controller, the encoder is configured with a zero offset of 40°. In
 * simulation, the encoder reads the simulated mechanism's angle with the offset already removed, so
 * after going to 100° the simulated mechanism, the mechanism, and the encoder are all at 100°: the
 * offset is neither applied twice nor the wrong way round. An absolute encoder gives the angle
 * within a rotation, so the mechanism positions are compared within a rotation. A quadrature
 * encoder, which is not absolute, has no zero offset.
 */
public class ZeroOffsetTest {
  private static final Angle kTolerance = Degrees.of(5);
  private static final Angle kZeroOffset = Degrees.of(40);
  private static final Angle kSetpoint = Degrees.of(100);

  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
  }

  @AfterEach
  void endTest() {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  /** An encoder and a closed loop controller. */
  private record Case(Encoder encoder, Controller controller) {
    String name() {
      return encoder + " " + controller;
    }

    @Override
    public String toString() {
      return name();
    }
  }

  private static Stream<Case> createCases() {
    final List<Case> cases = new ArrayList<>();
    for (Encoder encoder : Encoder.values()) {
      for (Controller controller : Controller.values()) {
        cases.add(new Case(encoder, controller));
      }
    }
    return cases.stream();
  }

  /** Check that the encoder holds the zero offset, in its own terms. */
  private static void assertConfigured(SmartMotorController smc, String name) throws InterruptedException {
    final double offset = kZeroOffset.in(Rotations);
    final Object encoder = smc.getConfig().getExternalEncoder().orElseThrow();
    if (encoder instanceof CANcoder cancoder) {
      final CANcoderConfiguration config = new CANcoderConfiguration();
      cancoder.getConfigurator().refresh(config);
      assertEquals(offset, config.MagnetSensor.MagnetOffset, 1e-3, name + ": CANcoder magnet offset");
    } else if (encoder instanceof CANdi candi) {
      final CANdiConfiguration config = new CANdiConfiguration();
      candi.getConfigurator().refresh(config);
      assertEquals(offset, config.PWM1.AbsoluteSensorOffset, 1e-3, name + ": CANdi absolute sensor offset");
    } else if (encoder instanceof DetachedEncoder detachedEncoder) {
      assertEquals(offset, detachedEncoder.detachedEncoderAccessor.getDutyCycleOffset(), 1e-6, name + ": CAN encoder duty cycle offset");
    } else {
      final SparkBase spark = (SparkBase) smc.getMotorController();
      final double[] zeroOffset = new double[1];
      assertTrue(AbsoluteEncoderCases.eventually(() -> {
        zeroOffset[0] = spark instanceof SparkMax sparkMax
                        ? sparkMax.configAccessor.absoluteEncoder.getZeroOffset()
                        : ((SparkFlex) spark).configAccessor.absoluteEncoder.getZeroOffset();
        return Math.abs(zeroOffset[0] - offset) < 1e-6;
      }), name + ": SPARK absolute encoder zero offset expected " + offset + " but was " + zeroOffset[0]);
    }
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("createCases")
  void encoderReadsZeroWhereTheMechanismIsAtZero(Case testCase) throws InterruptedException {
    final String name = testCase.name();
    final SmartMotorController smc = AbsoluteEncoderCases.create(testCase.encoder(),
        AbsoluteEncoderCases.config("ZeroOffsetTest " + name, testCase.controller()),
        config -> config.withExternalEncoderZeroOffset(kZeroOffset));
    try {
      assertConfigured(smc, name);
      AbsoluteEncoderCases.runTo(smc, testCase.encoder(), kSetpoint, Seconds.of(2.5));

      final Angle simulated = smc.getSimSupplier().orElseThrow().getMechanismPosition();
      final Angle mechanism = smc.getMechanismPosition();
      final Angle absolute = AbsoluteEncoderCases.absoluteAngle(smc);
      // An absolute encoder gives the angle within a rotation, so the mechanism may have gone either
      // way round to it.
      assertTrue(Math.abs(MathUtil.inputModulus(simulated.minus(kSetpoint).in(Degrees), -180, 180)) < kTolerance.in(Degrees),
          name + ": expected the simulated mechanism at " + kSetpoint.in(Degrees) + "° but it was at " + simulated.in(Degrees) + "°");
      assertTrue(Math.abs(MathUtil.inputModulus(mechanism.minus(kSetpoint).in(Degrees), -180, 180)) < kTolerance.in(Degrees),
          name + ": expected the mechanism to read " + kSetpoint.in(Degrees) + "° but it read " + mechanism.in(Degrees) + "°");
      assertTrue(Math.abs(absolute.minus(kSetpoint).in(Degrees)) < kTolerance.in(Degrees),
          name + ": expected the encoder to read " + kSetpoint.in(Degrees) + "° but it read " + absolute.in(Degrees) + "°");
    } finally {
      AbsoluteEncoderCases.close(smc);
    }
  }

  @Test
  void negativeZeroOffsetIsTheSameAngleWithinARotation() {
    final SmartMotorControllerConfig config = new yams.commands2.config.SmartMotorControllerConfig().withExternalEncoderZeroOffset(Degrees.of(-10));
    assertEquals(350, config.getExternalEncoderZeroOffset().orElseThrow().in(Degrees), 1e-9);
  }

  @ParameterizedTest(name = "{0}")
  @ValueSource(strings = {"SparkMax", "SparkFlex"})
  void quadratureEncoderHasNoZeroOffset(String name) {
    final SparkBase spark = name.equals("SparkFlex") ? DeviceCreator.createSparkFlex() : DeviceCreator.createSparkMax();
    final SmartMotorControllerConfig config = ThroughBoreEncoderTest.withThroughBore(spark, ThroughBoreEncoderTest.Connection.QUADRATURE,
            AbsoluteEncoderCases.config("ZeroOffsetTest quadrature " + name, Controller.PID))
        .withUseExternalFeedbackEncoder(true)
        .withExternalEncoderZeroOffset(kZeroOffset);
    try {
      assertThrows(SmartMotorControllerConfigurationException.class,
          () -> new SparkWrapper(spark, name.equals("SparkFlex") ? DCMotor.getNeoVortex(1) : DCMotor.getNEO(1), config),
          name + ": a quadrature encoder has no zero offset");
    } finally {
      CommandScheduler.getInstance().unregisterSubsystem(config.getSubsystem());
      ThroughBoreEncoderTest.closeDevices(spark, null);
    }
  }
}
