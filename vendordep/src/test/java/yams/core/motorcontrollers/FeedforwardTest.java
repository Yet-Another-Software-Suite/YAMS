// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;
import static yams.core.motorcontrollers.ClosedLoopCases.averageVoltage;
import static yams.core.motorcontrollers.ClosedLoopCases.baseConfig;
import static yams.core.motorcontrollers.ClosedLoopCases.close;
import static yams.core.motorcontrollers.ClosedLoopCases.create;

import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.controller.ElevatorFeedforward;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.preferences.Preferences;
import org.wpilib.simulation.RoboRioSim;
import yams.core.motorcontrollers.ClosedLoopCases.Controller;
import yams.helpers.MockHardwareExtension;

/**
 * Tests that kS, kV, kA and kG give the same voltage on every motor controller, in YAMS units: kS and
 * kG in volts, kV in volts per mechanism rotation per second and kA per rotation per second squared
 * (per meter for a linear mechanism), and an arm's kG times the cosine of its angle.
 *
 * <p>The SPARK and Talons apply the feedforward onboard and the Nova on the SystemCore. A simulated
 * SPARK follows a profile with position control and an arbitrary feedforward, since REVLib's
 * MAXMotion does not follow its profile in simulation, so a gain the SPARK also applies itself must
 * not be sent again, nor one it does not apply left out.
 *
 * <p>The mechanism is heavy enough that it barely moves while a gain is measured, and with no kP the
 * voltage is the feedforward's alone.
 */
public class FeedforwardTest {
  /**
   * Heavy enough that a volt barely moves it in a second. Each test asks for about a volt: held still,
   * a motor draws its voltage over its resistance, and a Kraken X60 reaches the 80 A current limit at
   * 2 V.
   */
  private static final double kMoi = 1000;

  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
    RoboRioSim.setVInVoltage(12.0);
  }

  @AfterEach
  void endTest() {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  /** Allowed difference from the expected voltage: 5%, and 50 mV of rounding and timing. */
  private static double tolerance(double expectedVolts) {
    return 0.05 * Math.abs(expectedVolts) + 0.05;
  }

  private static Stream<Controller> controllers() {
    return ClosedLoopCases.controllers();
  }

  /** Every motor controller, with and without a trapezoidal profile. */
  private static Stream<Arguments> controllersWithAndWithoutProfile() {
    return controllers().flatMap(c -> Stream.of(Arguments.of(c, true), Arguments.of(c, false)));
  }

  private static yams.commands2.config.SmartMotorControllerConfig config(
      String test, Controller controller) {
    return baseConfig("FeedforwardTest " + test + " " + controller)
        .withMomentOfInertia(KilogramSquareMeters.of(kMoi))
        .withStartingPosition(Rotations.of(0))
        .withClosedLoopController(0, 0, 0);
  }

  /** A slow profile, still speeding up for the first two seconds toward a distant goal. */
  private static yams.commands2.config.SmartMotorControllerConfig withSlowProfile(
      yams.commands2.config.SmartMotorControllerConfig cfg) {
    return cfg.withTrapezoidalProfile(RotationsPerSecond.of(2), RotationsPerSecondPerSecond.of(1));
  }

  /** Velocity control applies kS in the direction of the setpoint. */
  @ParameterizedTest(name = "{0}")
  @MethodSource("controllers")
  void staticGainWithVelocityControl(Controller controller) {
    final double kS = 0.5;
    final SmartMotorController smc =
        create(
            controller,
            config("kS velocity", controller).withFeedforward(new SimpleMotorFeedforward(kS, 0, 0)));
    try {
      final double volts =
          averageVoltage(controller, smc, 0.5, 0.8, () -> smc.setVelocity(RotationsPerSecond.of(1)));
      assertEquals(kS, volts, tolerance(kS), controller + ": kS");
    } finally {
      close(smc);
    }
  }

  /** Following a profile applies kS once, in the direction of travel. */
  @ParameterizedTest(name = "{0}")
  @MethodSource("controllers")
  void staticGainWhileFollowingAProfile(Controller controller) {
    final double kS = 0.5;
    final SmartMotorController smc =
        create(
            controller,
            withSlowProfile(config("kS profile", controller))
                .withFeedforward(new SimpleMotorFeedforward(kS, 0, 0)));
    try {
      final double volts =
          averageVoltage(controller, smc, 0.6, 1.2, () -> smc.setPosition(Rotations.of(10)));
      assertEquals(kS, volts, tolerance(kS), controller + ": kS, applied once");
    } finally {
      close(smc);
    }
  }

  /** Velocity control applies kV times the setpoint. */
  @ParameterizedTest(name = "{0}")
  @MethodSource("controllers")
  void velocityGainWithVelocityControl(Controller controller) {
    final double kV = 1;
    final double velocity = 1;
    final SmartMotorController smc =
        create(
            controller,
            config("kV velocity", controller).withFeedforward(new SimpleMotorFeedforward(0, kV, 0)));
    try {
      final double volts =
          averageVoltage(
              controller, smc, 0.5, 0.8, () -> smc.setVelocity(RotationsPerSecond.of(velocity)));
      assertEquals(kV * velocity, volts, tolerance(kV * velocity), controller + ": kV times the velocity");
    } finally {
      close(smc);
    }
  }

  /** Following a profile at its cruise velocity applies kV times that velocity. */
  @ParameterizedTest(name = "{0}")
  @MethodSource("controllers")
  void velocityGainWhileFollowingAProfile(Controller controller) {
    final double kV = 1;
    final double cruise = 1;
    final SmartMotorController smc =
        create(
            controller,
            config("kV profile", controller)
                .withTrapezoidalProfile(
                    RotationsPerSecond.of(cruise), RotationsPerSecondPerSecond.of(4))
                .withFeedforward(new SimpleMotorFeedforward(0, kV, 0)));
    try {
      // The profile reaches its cruise velocity after 0.25 s.
      final double volts =
          averageVoltage(controller, smc, 0.6, 1.2, () -> smc.setPosition(Rotations.of(10)));
      assertEquals(
          kV * cruise, volts, tolerance(kV * cruise), controller + ": kV times the cruise velocity");
    } finally {
      close(smc);
    }
  }

  /** Following a profile as it speeds up applies kA times its acceleration. */
  @ParameterizedTest(name = "{0}")
  @MethodSource("controllers")
  void accelerationGainWhileFollowingAProfile(Controller controller) {
    final double kA = 1;
    final double acceleration = 1;
    final SmartMotorController smc =
        create(
            controller,
            withSlowProfile(config("kA profile", controller))
                .withFeedforward(new SimpleMotorFeedforward(0, 0, kA)));
    try {
      final double volts =
          averageVoltage(controller, smc, 0.6, 1.5, () -> smc.setPosition(Rotations.of(10)));
      assertEquals(
          kA * acceleration,
          volts,
          tolerance(kA * acceleration),
          controller + ": kA times the profile's acceleration");
    } finally {
      close(smc);
    }
  }

  /** A linear mechanism's kV is volts per meter per second. */
  @ParameterizedTest(name = "{0}")
  @MethodSource("controllers")
  void linearVelocityGain(Controller controller) {
    final double kV = 5;
    final double velocity = 0.1;
    final SmartMotorController smc =
        create(
            controller,
            config("linear kV", controller)
                .withMechanismCircumference(Meters.of(0.2))
                .withFeedforward(new ElevatorFeedforward(0, 0, kV)));
    try {
      final double volts =
          averageVoltage(
              controller, smc, 0.5, 0.8, () -> smc.setVelocity(MetersPerSecond.of(velocity)));
      assertEquals(kV * velocity, volts, tolerance(kV * velocity), controller + ": kV times the velocity");
    } finally {
      close(smc);
    }
  }

  /** Holding an elevator applies kG. */
  @ParameterizedTest(name = "{0} profile {1}")
  @MethodSource("controllersWithAndWithoutProfile")
  void elevatorGravity(Controller controller, boolean profile) {
    final double kG = 0.8;
    var cfg =
        config("elevator kG " + profile, controller)
            .withMechanismCircumference(Meters.of(0.2))
            .withFeedforward(new ElevatorFeedforward(0, kG, 0));
    if (profile) {
      cfg = cfg.withTrapezoidalProfile(MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(1));
    }
    final SmartMotorController smc = create(controller, cfg);
    try {
      final double volts =
          averageVoltage(controller, smc, 0.5, 0.8, () -> smc.setPosition(Meters.of(0)));
      assertEquals(kG, volts, tolerance(kG), controller + ": kG");
    } finally {
      close(smc);
    }
  }

  private static Stream<Arguments> armAngles() {
    return controllers()
        .flatMap(
            c ->
                Stream.of(60.0, 150.0)
                    .flatMap(
                        angle ->
                            Stream.of(Arguments.of(c, angle, true), Arguments.of(c, angle, false))));
  }

  /** Holding an arm applies kG times the cosine of its angle from horizontal. */
  @ParameterizedTest(name = "{0} at {1} degrees, profile {2}")
  @MethodSource("armAngles")
  void armGravity(Controller controller, double degrees, boolean profile) {
    final double kG = 1;
    var cfg =
        config("arm kG " + degrees + " " + profile, controller)
            .withStartingPosition(Degrees.of(degrees))
            .withFeedforward(new ArmFeedforward(0, kG, 0));
    if (profile) {
      cfg = withSlowProfile(cfg);
    }
    final SmartMotorController smc = create(controller, cfg);
    try {
      final double expected = kG * Math.cos(Math.toRadians(degrees));
      final double volts =
          averageVoltage(controller, smc, 0.5, 0.8, () -> smc.setPosition(Degrees.of(degrees)));
      assertEquals(expected, volts, tolerance(expected), controller + ": kG times cos(angle)");
    } finally {
      close(smc);
    }
  }
}
