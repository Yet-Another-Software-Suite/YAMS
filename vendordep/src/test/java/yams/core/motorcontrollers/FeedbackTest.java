// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;
import static yams.core.motorcontrollers.ClosedLoopCases.averageVoltage;
import static yams.core.motorcontrollers.ClosedLoopCases.baseConfig;
import static yams.core.motorcontrollers.ClosedLoopCases.close;
import static yams.core.motorcontrollers.ClosedLoopCases.create;
import static yams.core.motorcontrollers.ClosedLoopCases.run;

import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.controller.ElevatorFeedforward;
import org.wpilib.preferences.Preferences;
import org.wpilib.simulation.RoboRioSim;
import yams.core.motorcontrollers.ClosedLoopCases.Controller;
import yams.helpers.MockHardwareExtension;

/**
 * Tests that kP, kI and kD give the same voltage on every motor controller, in YAMS units: volts per
 * mechanism rotation of error (per meter for a linear mechanism, per rotation per second for
 * velocity control), per rotation-second of accumulated error, and per rotation per second of the
 * error's rate of change.
 *
 * <p>The mechanism is heavy enough that it barely moves while a gain is measured, so the error stays
 * as it was set and each gain's voltage can be read from the motor controller.
 */
public class FeedbackTest {
  /**
   * Heavy enough that a volt barely moves it in a second. Each test asks for about a volt: held still,
   * a motor draws its voltage over its resistance, and a Kraken X60 reaches the 80 A current limit at
   * 2 V.
   */
  private static final double kMoi = 1000;

  /** Allowed difference from the expected voltage: 5%, and 50 mV of rounding and timing. */
  private static double tolerance(double expectedVolts) {
    return 0.05 * Math.abs(expectedVolts) + 0.05;
  }

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

  private static Stream<Controller> controllers() {
    return ClosedLoopCases.controllers();
  }

  private static yams.commands2.config.SmartMotorControllerConfig config(
      String test, Controller controller, double kP, double kI, double kD) {
    return baseConfig("FeedbackTest " + test + " " + controller)
        .withMomentOfInertia(KilogramSquareMeters.of(kMoi))
        .withStartingPosition(Rotations.of(0))
        .withClosedLoopController(kP, kI, kD);
  }

  /** kP volts per rotation of position error. */
  @ParameterizedTest(name = "{0}")
  @MethodSource("controllers")
  void positionProportionalGain(Controller controller) {
    final double kP = 5;
    final double error = 0.2;
    final SmartMotorController smc = create(controller, config("kP", controller, kP, 0, 0));
    try {
      final double volts =
          averageVoltage(controller, smc, 0.5, 0.8, () -> smc.setPosition(Rotations.of(error)));
      assertEquals(kP * error, volts, tolerance(kP * error), controller + ": kP times the error");
    } finally {
      close(smc);
    }
  }

  /** kP volts per rotation per second of velocity error. */
  @ParameterizedTest(name = "{0}")
  @MethodSource("controllers")
  void velocityProportionalGain(Controller controller) {
    final double kP = 1;
    final double velocity = 1;
    final SmartMotorController smc = create(controller, config("velocity kP", controller, kP, 0, 0));
    try {
      final double volts =
          averageVoltage(
              controller, smc, 0.5, 0.8, () -> smc.setVelocity(RotationsPerSecond.of(velocity)));
      assertEquals(
          kP * velocity, volts, tolerance(kP * velocity), controller + ": kP times the error");
    } finally {
      close(smc);
    }
  }

  /**
   * A linear mechanism's kP is volts per meter of error. An {@link ElevatorFeedforward}, even with no
   * gains, makes the closed loop linear.
   */
  @ParameterizedTest(name = "{0}")
  @MethodSource("controllers")
  void linearProportionalGain(Controller controller) {
    final double kP = 20;
    final double error = 0.05;
    final SmartMotorController smc =
        create(
            controller,
            config("linear kP", controller, kP, 0, 0)
                .withMechanismCircumference(Meters.of(0.2))
                .withFeedforward(new ElevatorFeedforward(0, 0, 0)));
    try {
      final double volts =
          averageVoltage(controller, smc, 0.5, 0.8, () -> smc.setPosition(Meters.of(error)));
      assertEquals(kP * error, volts, tolerance(kP * error), controller + ": kP times the error");
    } finally {
      close(smc);
    }
  }

  /**
   * kI volts per rotation-second of accumulated error: with a steady error, the voltage grows by kI
   * times the error each second.
   */
  @ParameterizedTest(name = "{0}")
  @MethodSource("controllers")
  void positionIntegralGain(Controller controller) {
    final double kI = 4;
    final double error = 0.25;
    final SmartMotorController smc = create(controller, config("kI", controller, 0, kI, 0));
    try {
      final double[] volts = {Double.NaN, Double.NaN};
      run(
          controller,
          smc,
          1.0,
          () -> smc.setPosition(Rotations.of(error)),
          time -> {
            if (Math.abs(time - 0.5) < 1e-6) {
              volts[0] = smc.getVoltage().in(Volts);
            } else if (Math.abs(time - 1.0) < 1e-6) {
              volts[1] = smc.getVoltage().in(Volts);
            }
          });
      final double perSecond = (volts[1] - volts[0]) / 0.5;
      assertEquals(
          kI * error,
          perSecond,
          0.1 * kI * error,
          controller + ": the voltage should grow by kI times the error each second, but went from "
              + volts[0] + " V at 0.5 s to " + volts[1] + " V at 1 s");
    } finally {
      close(smc);
    }
  }

  /**
   * kD volts per rotation per second of the error's rate of change: holding a setpoint while the
   * mechanism moves away from it at a steady velocity, the error shrinks at that velocity.
   */
  @ParameterizedTest(name = "{0}")
  @MethodSource("controllers")
  void positionDerivativeGain(Controller controller) {
    final double kD = 4;
    final double velocity = 0.25;
    final SmartMotorController smc = create(controller, config("kD", controller, 0, 0, kD));
    try {
      smc.getSimSupplier()
          .orElseThrow()
          .setMechanismVelocity(RotationsPerSecond.of(velocity));
      final double volts =
          averageVoltage(controller, smc, 0.5, 0.8, () -> smc.setPosition(Rotations.of(0)));
      assertEquals(
          -kD * velocity,
          volts,
          tolerance(kD * velocity),
          controller + ": kD times the error's rate of change");
    } finally {
      close(smc);
    }
  }
}
