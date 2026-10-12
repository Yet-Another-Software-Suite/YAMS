// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.util.MathUtil;
import org.wpilib.preferences.Preferences;
import org.wpilib.units.measure.Angle;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.core.motorcontrollers.AbsoluteEncoderCases.Controller;
import yams.core.motorcontrollers.AbsoluteEncoderCases.Encoder;
import yams.helpers.MockHardwareExtension;

/**
 * Tests {@link SmartMotorControllerConfig#withExternalEncoderZeroOffset} together with
 * {@link SmartMotorControllerConfig#withExternalEncoderDiscontinuityPoint}: the zero offset moves
 * the encoder's zero, and the discontinuity point moves with it.
 *
 * <p>For every absolute external encoder on every motor controller ({@link AbsoluteEncoderCases})
 * and for both discontinuity points, the encoder is configured with a zero offset of 90° and the
 * mechanism goes to two angles. The encoder reports angles from one rotation below its
 * discontinuity point up to it, measured from where the mechanism is at zero: [-180°, 180°) for
 * 0.5 rotations and [0°, 360°) for 1 rotation.
 *
 * <p>If the discontinuity point stayed where it is without a zero offset, the offset would shift
 * that range by 90° instead, to [-270°, 90°) and [-90°, 270°). One angle of each case is outside
 * the shifted range, so it is only reported as itself when the discontinuity point has moved with
 * the zero: 120° for 0.5 rotations, which would otherwise read -240°, and 300° for 1 rotation, which
 * would otherwise read -60°.
 */
public class ZeroOffsetDiscontinuityPointTest {
  private static final Angle kTolerance = Degrees.of(5);
  private static final Angle kZeroOffset = Degrees.of(90);

  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
  }

  @AfterEach
  void endTest() {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  /** An encoder, a discontinuity point, and an angle to go to. */
  private record Case(Encoder encoder, Angle discontinuityPoint, Angle setpoint) {
    String name() {
      return encoder
          + " discontinuity point "
          + discontinuityPoint.in(Rotations)
          + " rotations at "
          + setpoint.in(Degrees)
          + " degrees";
    }

    @Override
    public String toString() {
      return name();
    }
  }

  private static Stream<Case> createCases() {
    final List<Case> cases = new ArrayList<>();
    for (Encoder encoder : Encoder.values()) {
      // 120 degrees is only in [-180, 180) when the discontinuity point moved with the zero offset.
      cases.add(new Case(encoder, Rotations.of(0.5), Degrees.of(120)));
      cases.add(new Case(encoder, Rotations.of(0.5), Degrees.of(-120)));
      // 300 degrees is only in [0, 360) when the discontinuity point moved with the zero offset.
      cases.add(new Case(encoder, Rotations.of(1), Degrees.of(300)));
      cases.add(new Case(encoder, Rotations.of(1), Degrees.of(30)));
    }
    return cases.stream();
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("createCases")
  void discontinuityPointMovesWithTheZeroOffset(Case testCase) {
    final String name = testCase.name();
    final SmartMotorController smc =
        AbsoluteEncoderCases.create(
            testCase.encoder(),
            AbsoluteEncoderCases.config("ZeroOffsetDiscontinuityPointTest " + name, Controller.PID),
            config ->
                config
                    .withExternalEncoderZeroOffset(kZeroOffset)
                    .withExternalEncoderDiscontinuityPoint(testCase.discontinuityPoint()));
    try {
      AbsoluteEncoderCases.runTo(smc, testCase.encoder(), testCase.setpoint(), Seconds.of(2.5));

      final Angle simulated = smc.getSimSupplier().orElseThrow().getMechanismPosition();
      // An absolute encoder gives the angle within a rotation, so the mechanism may have gone
      // either way round to it.
      assertTrue(
          Math.abs(
                  MathUtil.inputModulus(
                      simulated.minus(testCase.setpoint()).in(Degrees), -180, 180))
              < kTolerance.in(Degrees),
          name
              + ": expected the simulated mechanism at "
              + testCase.setpoint().in(Degrees)
              + "° but it was at "
              + simulated.in(Degrees)
              + "°");
      // The encoder reports the angle itself, not the same angle a rotation away: its range is
      // measured from the mechanism's zero, where the zero offset put the encoder's zero.
      final Angle absolute = AbsoluteEncoderCases.absoluteAngle(smc);
      assertTrue(
          Math.abs(absolute.minus(testCase.setpoint()).in(Degrees)) < kTolerance.in(Degrees),
          name
              + ": expected the encoder to report "
              + testCase.setpoint().in(Degrees)
              + "° but it reported "
              + absolute.in(Degrees)
              + "°");
    } finally {
      AbsoluteEncoderCases.close(smc);
    }
  }
}
