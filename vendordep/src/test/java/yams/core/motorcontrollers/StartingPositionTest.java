// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
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
import yams.core.motorcontrollers.AbsoluteEncoderCases.RelativeFeedback;
import yams.helpers.MockHardwareExtension;

/**
 * Tests a mechanism that starts away from zero, with and without an absolute encoder, on every motor
 * controller and under every closed loop controller.
 *
 * <ul>
 *   <li>Without an absolute encoder, the motor's encoder, or a SPARK's quadrature encoder, only
 *       counts from where it powers on, so the robot is placed at the
 *       {@link SmartMotorControllerConfig#withStartingPosition starting position}, 30°.
 *   <li>With an absolute encoder ({@link AbsoluteEncoderCases}), the encoder knows where the
 *       mechanism is: the simulated mechanism rests at 30°
 *       ({@link SmartMotorControllerConfig#withSimStartingPosition}) and no starting position is
 *       given.
 * </ul>
 *
 * <p>Either way, before moving the mechanism reads 30°, as does its encoder, and the simulated
 * mechanism is there. Going to 100° then moves the simulated mechanism the 70° to 100°.
 */
public class StartingPositionTest {
  private static final Angle kTolerance = Degrees.of(5);
  private static final Angle kStartingPosition = Degrees.of(30);
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

  /** What the mechanism's position comes from: an absolute encoder, or not. */
  private record Feedback(RelativeFeedback relative, Encoder absoluteEncoder) {
    boolean talon() {
      return absoluteEncoder != null ? absoluteEncoder.talon() : relative.talon();
    }

    @Override
    public String toString() {
      return absoluteEncoder != null ? absoluteEncoder.toString() : relative.toString();
    }
  }

  /** A feedback sensor and a closed loop controller. */
  private record Case(Feedback feedback, Controller controller) {
    @Override
    public String toString() {
      return feedback + " " + controller;
    }
  }

  private static Stream<Case> createCases() {
    final List<Case> cases = new ArrayList<>();
    final List<Feedback> feedbacks = new ArrayList<>();
    for (RelativeFeedback relative : RelativeFeedback.values()) {
      feedbacks.add(new Feedback(relative, null));
    }
    for (Encoder absoluteEncoder : Encoder.values()) {
      feedbacks.add(new Feedback(null, absoluteEncoder));
    }
    for (Feedback feedback : feedbacks) {
      for (Controller controller : Controller.values()) {
        cases.add(new Case(feedback, controller));
      }
    }
    return cases.stream();
  }

  private static SmartMotorController create(Case testCase, String name) {
    final SmartMotorControllerConfig config =
        AbsoluteEncoderCases.config(name, testCase.controller());
    final Feedback feedback = testCase.feedback();
    if (feedback.absoluteEncoder() != null) {
      return AbsoluteEncoderCases.create(
          feedback.absoluteEncoder(),
          config,
          cfg -> cfg.withSimStartingPosition(kStartingPosition));
    }
    return AbsoluteEncoderCases.create(
        feedback.relative(), config.withStartingPosition(kStartingPosition));
  }

  /** Difference between two angles, wrapped into [-180°, 180°). */
  private static double wrappedErrorDegrees(Angle actual, Angle expected) {
    return MathUtil.inputModulus(actual.in(Degrees) - expected.in(Degrees), -180, 180);
  }

  private static void assertNear(Angle actual, Angle expected, String what) {
    assertTrue(
        Math.abs(wrappedErrorDegrees(actual, expected)) < kTolerance.in(Degrees),
        what + " expected " + expected.in(Degrees) + "° but was " + actual.in(Degrees) + "°");
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("createCases")
  void startsAwayFromZero(Case testCase) {
    final String name = testCase.toString();
    final SmartMotorController smc = create(testCase, "StartingPositionTest " + name);
    try {
      // Let the simulation, and a Talon's fused sensor, settle where the mechanism starts.
      AbsoluteEncoderCases.run(smc, testCase.feedback().talon(), Optional.empty(), Seconds.of(0.5));
      assertNear(
          smc.getSimSupplier().orElseThrow().getMechanismPosition(),
          kStartingPosition,
          name + ": simulated mechanism at the start");
      assertNear(
          smc.getMechanismPosition(), kStartingPosition, name + ": mechanism reading at the start");
      smc.getExternalEncoderMechanismPosition()
          .ifPresent(
              encoder ->
                  assertNear(encoder, kStartingPosition, name + ": encoder reading at the start"));

      AbsoluteEncoderCases.run(
          smc, testCase.feedback().talon(), Optional.of(kSetpoint), Seconds.of(2.5));
      final Angle simulated = smc.getSimSupplier().orElseThrow().getMechanismPosition();
      // The mechanism started at 30°, so going to 100° moves it 70°, not a whole rotation more.
      assertTrue(
          Math.abs(simulated.minus(kSetpoint).in(Degrees)) < kTolerance.in(Degrees),
          name
              + ": expected the simulated mechanism at "
              + kSetpoint.in(Degrees)
              + "° but it was at "
              + simulated.in(Degrees)
              + "°");
      assertNear(smc.getMechanismPosition(), kSetpoint, name + ": mechanism reading after moving");
      smc.getExternalEncoderMechanismPosition()
          .ifPresent(
              encoder -> assertNear(encoder, kSetpoint, name + ": encoder reading after moving"));
    } finally {
      AbsoluteEncoderCases.close(smc);
    }
  }
}
