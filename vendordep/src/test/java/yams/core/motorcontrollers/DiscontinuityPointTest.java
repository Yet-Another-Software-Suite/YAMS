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
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
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
 * Tests {@link SmartMotorControllerConfig#withExternalEncoderDiscontinuityPoint}: an absolute
 * encoder with a discontinuity point of 0.5 rotations reports angles in [-0.5, 0.5) rotations, and
 * one of 1 rotation in [0, 1).
 *
 * <p>For every absolute external encoder on every motor controller ({@link AbsoluteEncoderCases}),
 * under every closed loop controller, and for both discontinuity points, the encoder is configured
 * with it, and the mechanism goes to an angle that only that range holds: -100° for 0.5 rotations,
 * 260° for 1 rotation. The encoder then reports that angle, inside its range; the mechanism may have
 * gone either way round to it, since the encoder gives the angle within a rotation. Only 0.5 and 1
 * rotation are accepted, and a quadrature encoder, which is not absolute, has no discontinuity
 * point.
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
      cancoder.getConfigurator().refresh(config);
      assertEquals(point, config.MagnetSensor.AbsoluteSensorDiscontinuityPoint, 1e-9, name + ": CANcoder discontinuity point");
    } else if (encoder instanceof CANdi candi) {
      final CANdiConfiguration config = new CANdiConfiguration();
      candi.getConfigurator().refresh(config);
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

  @ParameterizedTest(name = "{0}")
  @ValueSource(strings = {"SparkMax", "SparkFlex"})
  void quadratureEncoderHasNoDiscontinuityPoint(String name) {
    final SparkBase spark = name.equals("SparkFlex") ? DeviceCreator.createSparkFlex() : DeviceCreator.createSparkMax();
    final SmartMotorControllerConfig config = ThroughBoreEncoderTest.withThroughBore(spark, ThroughBoreEncoderTest.Connection.QUADRATURE,
            AbsoluteEncoderCases.config("DiscontinuityPointTest quadrature " + name, Controller.PID))
        .withUseExternalFeedbackEncoder(true)
        .withExternalEncoderDiscontinuityPoint(Rotations.of(0.5));
    try {
      assertThrows(SmartMotorControllerConfigurationException.class,
          () -> new SparkWrapper(spark, name.equals("SparkFlex") ? DCMotor.getNeoVortex(1) : DCMotor.getNEO(1), config),
          name + ": a quadrature encoder has no discontinuity point");
    } finally {
      CommandScheduler.getInstance().unregisterSubsystem(config.getSubsystem());
      ThroughBoreEncoderTest.closeDevices(spark, null);
    }
  }
}
