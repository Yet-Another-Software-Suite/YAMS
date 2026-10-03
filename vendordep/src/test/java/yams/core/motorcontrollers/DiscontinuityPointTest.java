// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
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
import java.util.Optional;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.ValueSource;
import org.wpilib.math.util.MathUtil;
import org.wpilib.preferences.Preferences;
import org.wpilib.units.measure.Angle;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.core.exceptions.SmartMotorControllerConfigurationException;
import yams.core.motorcontrollers.AbsoluteEncoderCases.Controller;
import yams.core.motorcontrollers.AbsoluteEncoderCases.Encoder;
import yams.core.motorcontrollers.AbsoluteEncoderCases.RelativeFeedback;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;

/**
 * Tests {@link SmartMotorControllerConfig#withExternalEncoderDiscontinuityPoint}: an absolute
 * encoder with a discontinuity point of 0.5 rotations reports angles in [-0.5, 0.5) rotations, and
 * one of 1 rotation in [0, 1).
 *
 * <p>For every absolute external encoder on every motor controller ({@link AbsoluteEncoderCases}),
 * under every closed loop controller, and for both discontinuity points, the encoder is configured
 * with it, and the mechanism goes to an angle that only that range holds: -100° for 0.5 rotations,
 * 260° for 1 rotation. The encoder then reports that angle, inside its range; the mechanism may have
 * gone either way round to it, since the encoder gives the angle within a rotation. Only 0.5 and 1
 * rotation are accepted. Without an absolute encoder, on the motor's encoder or a quadrature
 * encoder, a SPARK rejects a discontinuity point, and a Talon ignores it.
 */
public class DiscontinuityPointTest {
  private static final Angle kTolerance = Degrees.of(5);

  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
  }

  @AfterEach
  void endTest() {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  /** An encoder, a closed loop controller, and a discontinuity point. */
  private record Case(Encoder encoder, Controller controller, Angle discontinuityPoint) {
    String name() {
      return encoder + " " + controller + " discontinuity point " + discontinuityPoint.in(Rotations) + " rotations";
    }

    /** An angle in the range the discontinuity point gives, outside the other one. */
    Angle setpoint() {
      return discontinuityPoint.isEquivalent(Rotations.of(0.5)) ? Degrees.of(-100) : Degrees.of(260);
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
        for (Angle discontinuityPoint : new Angle[] {Rotations.of(0.5), Rotations.of(1)}) {
          cases.add(new Case(encoder, controller, discontinuityPoint));
        }
      }
    }
    return cases.stream();
  }

  /** Check that the encoder holds the discontinuity point, in its own terms. */
  private static void assertConfigured(SmartMotorController smc, Case testCase) throws InterruptedException {
    final String name = testCase.name();
    final double point = testCase.discontinuityPoint().in(Rotations);
    final Object encoder = smc.getConfig().getExternalEncoder().orElseThrow();
    if (encoder instanceof CANcoder cancoder) {
      final CANcoderConfiguration config = new CANcoderConfiguration();
      DeviceCreator.refreshConfig(() -> cancoder.getConfigurator().refresh(config, 1.0));
      assertEquals(point, config.MagnetSensor.AbsoluteSensorDiscontinuityPoint, 1e-9, name + ": CANcoder discontinuity point");
    } else if (encoder instanceof CANdi candi) {
      final CANdiConfiguration config = new CANdiConfiguration();
      DeviceCreator.refreshConfig(() -> candi.getConfigurator().refresh(config, 1.0));
      assertEquals(point, config.PWM1.AbsoluteSensorDiscontinuityPoint, 1e-9, name + ": CANdi discontinuity point");
    } else if (encoder instanceof DetachedEncoder detachedEncoder) {
      assertEquals(point == 0.5, detachedEncoder.detachedEncoderAccessor.isDutyCycleZeroCentered(), name + ": CAN encoder zero-centered");
    } else {
      // REVLib's range offset is the middle of the range the encoder reports.
      final SparkBase spark = (SparkBase) smc.getMotorController();
      final double[] rangeOffset = new double[1];
      assertTrue(AbsoluteEncoderCases.eventually(() -> {
        rangeOffset[0] = spark instanceof SparkMax sparkMax
                         ? sparkMax.configAccessor.absoluteEncoder.getRangeOffset()
                         : ((SparkFlex) spark).configAccessor.absoluteEncoder.getRangeOffset();
        return Math.abs(rangeOffset[0] - (point - 0.5)) < 1e-6;
      }), name + ": SPARK absolute encoder range offset expected " + (point - 0.5) + " but was " + rangeOffset[0]);
    }
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("createCases")
  void encoderReportsAnglesBelowItsDiscontinuityPoint(Case testCase) throws InterruptedException {
    final String name = testCase.name();
    final SmartMotorController smc = AbsoluteEncoderCases.create(testCase.encoder(),
        AbsoluteEncoderCases.config("DiscontinuityPointTest " + name, testCase.controller()),
        config -> config.withExternalEncoderDiscontinuityPoint(testCase.discontinuityPoint()));
    try {
      assertConfigured(smc, testCase);
      AbsoluteEncoderCases.runTo(smc, testCase.encoder(), testCase.setpoint(), Seconds.of(2.5));

      final Angle simulated = smc.getSimSupplier().orElseThrow().getMechanismPosition();
      // An absolute encoder gives the angle within a rotation, so the mechanism may have gone either
      // way round to it.
      assertTrue(Math.abs(MathUtil.inputModulus(simulated.minus(testCase.setpoint()).in(Degrees), -180, 180)) < kTolerance.in(Degrees),
          name + ": expected the simulated mechanism at " + testCase.setpoint().in(Degrees) + "° but it was at " + simulated.in(Degrees) + "°");
      final Angle absolute = AbsoluteEncoderCases.absoluteAngle(smc);
      final double point = testCase.discontinuityPoint().in(Rotations);
      assertTrue(absolute.in(Rotations) >= point - 1 - 1e-6 && absolute.in(Rotations) <= point + 1e-6,
          name + ": expected the encoder to report an angle in [" + (point - 1) + ", " + point + ") rotations but it reported " + absolute.in(Rotations));
      assertTrue(Math.abs(absolute.minus(testCase.setpoint()).in(Degrees)) < kTolerance.in(Degrees),
          name + ": expected the encoder to report " + testCase.setpoint().in(Degrees) + "° but it reported " + absolute.in(Degrees) + "°");
    } finally {
      AbsoluteEncoderCases.close(smc);
    }
  }

  @ParameterizedTest(name = "{0} rotations")
  @ValueSource(doubles = {0.5, 1})
  void acceptsHalfAndWholeRotations(double rotations) {
    assertDoesNotThrow(() -> new yams.commands2.config.SmartMotorControllerConfig().withExternalEncoderDiscontinuityPoint(Rotations.of(rotations)));
  }

  @ParameterizedTest(name = "{0} rotations")
  @ValueSource(doubles = {0, 0.3, 0.75, 2, -0.5})
  void rejectsOtherDiscontinuityPoints(double rotations) {
    assertThrows(SmartMotorControllerConfigurationException.class,
        () -> new yams.commands2.config.SmartMotorControllerConfig().withExternalEncoderDiscontinuityPoint(Rotations.of(rotations)));
  }

  /** A motor controller without an absolute encoder, a closed loop controller, and a discontinuity point. */
  private record RelativeCase(RelativeFeedback feedback, Controller controller, Angle discontinuityPoint) {
    @Override
    public String toString() {
      return feedback + " " + controller + " discontinuity point " + discontinuityPoint.in(Rotations) + " rotations";
    }
  }

  private static Stream<RelativeCase> createRelativeCases() {
    final List<RelativeCase> cases = new ArrayList<>();
    for (RelativeFeedback feedback : RelativeFeedback.values()) {
      for (Controller controller : Controller.values()) {
        for (Angle discontinuityPoint : new Angle[] {Rotations.of(0.5), Rotations.of(1)}) {
          cases.add(new RelativeCase(feedback, controller, discontinuityPoint));
        }
      }
    }
    return cases.stream();
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("createRelativeCases")
  void withoutAnAbsoluteEncoder(RelativeCase testCase) {
    final String name = testCase.toString();
    final SmartMotorControllerConfig config = AbsoluteEncoderCases.config("DiscontinuityPointTest " + name, testCase.controller())
        .withExternalEncoderDiscontinuityPoint(testCase.discontinuityPoint());
    if (!testCase.feedback().talon()) {
      // A SPARK rejects the option without an absolute encoder to give it to.
      assertThrows(SmartMotorControllerConfigurationException.class, () -> AbsoluteEncoderCases.create(testCase.feedback(), config),
          name + ": a SPARK without an absolute encoder has no discontinuity point");
      return;
    }
    // A Talon alerts that the discontinuity point is not applied without an external encoder. Its
    // motor's encoder does not wrap: 260 degrees reads 260 degrees whatever the point.
    final Angle setpoint = Degrees.of(260);
    final SmartMotorController smc = AbsoluteEncoderCases.create(testCase.feedback(), config);
    try {
      AbsoluteEncoderCases.run(smc, true, Optional.of(setpoint), Seconds.of(2.5));
      final Angle simulated = smc.getSimSupplier().orElseThrow().getMechanismPosition();
      final Angle mechanism = smc.getMechanismPosition();
      assertTrue(Math.abs(simulated.minus(setpoint).in(Degrees)) < kTolerance.in(Degrees),
          name + ": expected the simulated mechanism at " + setpoint.in(Degrees) + " degrees but it was at " + simulated.in(Degrees));
      assertTrue(Math.abs(mechanism.minus(setpoint).in(Degrees)) < kTolerance.in(Degrees),
          name + ": expected the mechanism to read " + setpoint.in(Degrees) + " degrees but it read " + mechanism.in(Degrees));
    } finally {
      AbsoluteEncoderCases.close(smc);
    }
  }
}
