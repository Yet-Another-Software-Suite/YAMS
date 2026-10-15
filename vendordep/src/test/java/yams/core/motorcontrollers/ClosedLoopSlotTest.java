// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeFalse;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static yams.core.motorcontrollers.ClosedLoopCases.averageVoltage;
import static yams.core.motorcontrollers.ClosedLoopCases.baseConfig;
import static yams.core.motorcontrollers.ClosedLoopCases.close;
import static yams.core.motorcontrollers.ClosedLoopCases.create;
import static yams.core.motorcontrollers.ClosedLoopCases.kRatio;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import java.util.function.DoubleSupplier;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.preferences.Preferences;
import org.wpilib.simulation.RoboRioSim;
import yams.core.motorcontrollers.ClosedLoopCases.Controller;
import yams.core.motorcontrollers.SmartMotorController.ClosedLoopControllerSlot;
import yams.helpers.MockHardwareExtension;

/**
 * Tests that every motor controller runs the gains of the closed loop slot selected, from each of its
 * slots: the SPARK and Nova have four, the Talons three. Each slot gets different gains, so running
 * another slot's gains gives the wrong voltage.
 *
 * <p>The mechanism is heavy enough that it barely moves while a gain is measured, and each gain asks
 * for at most a volt, below where any motor reaches its current limit held still.
 */
public class ClosedLoopSlotTest {
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

  private static Stream<Arguments> controllersAndSlots() {
    return ClosedLoopCases.controllersAndSlots();
  }

  /** kP for a slot: 2.5, 5, 7.5 and 10 volts per mechanism rotation. */
  private static double kP(ClosedLoopControllerSlot slot) {
    return 2.5 * (slot.ordinal() + 1);
  }

  /** kV for a slot: 0.25, 0.5, 0.75 and 1 volt per mechanism rotation per second. */
  private static double kV(ClosedLoopControllerSlot slot) {
    return 0.25 * (slot.ordinal() + 1);
  }

  /** kS for a slot: 0.05, 0.1, 0.15 and 0.2 volts. */
  private static double kS(ClosedLoopControllerSlot slot) {
    return 0.05 * (slot.ordinal() + 1);
  }

  /** A config with each of the controller's slots filled in, each with its own gains. */
  private static yams.commands2.config.SmartMotorControllerConfig config(
      String test, Controller controller, boolean feedforward) {
    final yams.commands2.config.SmartMotorControllerConfig config =
        baseConfig("ClosedLoopSlotTest " + test + " " + controller)
            .withMomentOfInertia(KilogramSquareMeters.of(kMoi))
            .withStartingPosition(Rotations.of(0));
    controller
        .slotStream()
        .forEach(
            slot -> {
              if (feedforward) {
                config
                    .withClosedLoopController(0, 0, 0, slot)
                    .withFeedforward(new SimpleMotorFeedforward(0, kV(slot), 0), slot);
              } else {
                config.withClosedLoopController(kP(slot), 0, 0, slot);
              }
            });
    return config;
  }

  /** Position control gives the selected slot's kP times the error. */
  @ParameterizedTest(name = "{0} {1}")
  @MethodSource("controllersAndSlots")
  void proportionalGainOfTheSelectedSlot(Controller controller, ClosedLoopControllerSlot slot) {
    final double error = 0.1;
    final SmartMotorController smc = create(controller, config("kP " + slot, controller, false));
    try {
      smc.setClosedLoopSlot(slot);
      final double expected = kP(slot) * error;
      final double volts =
          averageVoltage(controller, smc, 0.5, 0.8, () -> smc.setPosition(Rotations.of(error)));
      assertEquals(
          expected, volts, 0.05 * expected + 0.05, controller + " " + slot + ": kP times the error");
    } finally {
      close(smc);
    }
  }

  /**
   * Velocity control gives the selected slot's kV times the setpoint.
   *
   * <p>REVLib's simulated SPARK only applies kV in its slot 0 (REVLib 2027.0.0-alpha-7), so the
   * SPARK's other slots are skipped; see
   * https://github.com/wpilibsuite/SystemcoreTesting/issues/425. {@link
   * #gainsReachTheMatchingSparkSlot} checks those slots get the right kV.
   */
  @ParameterizedTest(name = "{0} {1}")
  @MethodSource("controllersAndSlots")
  void velocityGainOfTheSelectedSlot(Controller controller, ClosedLoopControllerSlot slot) {
    assumeFalse(
        controller.isSpark() && slot != ClosedLoopControllerSlot.SLOT_0,
        "REVLib's simulated SPARK only applies kV in slot 0");
    final double velocity = 1;
    final SmartMotorController smc = create(controller, config("kV " + slot, controller, true));
    try {
      smc.setClosedLoopSlot(slot);
      final double expected = kV(slot) * velocity;
      final double volts =
          averageVoltage(
              controller, smc, 0.5, 0.8, () -> smc.setVelocity(RotationsPerSecond.of(velocity)));
      assertEquals(
          expected,
          volts,
          0.05 * expected + 0.05,
          controller + " " + slot + ": kV times the velocity");
    } finally {
      close(smc);
    }
  }

  /** Wait for an asynchronously applied SPARK parameter to reach the expected value. */
  private static void assertDevice(double expected, DoubleSupplier actual, String what)
      throws InterruptedException {
    assertTrue(
        AbsoluteEncoderCases.eventually(
            () ->
                Math.abs(actual.getAsDouble() - expected)
                    <= 1e-4 * Math.max(1, Math.abs(expected))),
        what + ": expected " + expected + " on the SPARK but was " + actual.getAsDouble());
  }

  /**
   * Each YAMS slot's gains are written to the SPARK slot of the same number, in the SPARK's units:
   * kP per rotation of its encoder at a 12 V scale, kS in volts, and kV per motor RPM. A real SPARK
   * runs these on the robot, where simulation cannot check them.
   */
  @ParameterizedTest
  @EnumSource(ClosedLoopControllerSlot.class)
  void gainsReachTheMatchingSparkSlot(ClosedLoopControllerSlot slot) throws InterruptedException {
    final Controller spark = ClosedLoopCases.kControllers[0];
    final yams.commands2.config.SmartMotorControllerConfig config =
        baseConfig("ClosedLoopSlotTest SPARK gains " + slot)
            .withMomentOfInertia(KilogramSquareMeters.of(kMoi));
    for (ClosedLoopControllerSlot each : ClosedLoopControllerSlot.values()) {
      config
          .withClosedLoopController(kP(each), 0, 0, each)
          .withFeedforward(new SimpleMotorFeedforward(kS(each), kV(each), 0), each);
    }
    final SmartMotorController smc = create(spark, config);
    try {
      final SparkMax sparkMax = (SparkMax) smc.getMotorController();
      final ClosedLoopSlot sparkSlot = ClosedLoopSlot.values()[slot.ordinal()];
      final var closedLoop = sparkMax.configAccessor.closedLoop;
      assertDevice(kP(slot) / (12 * kRatio), () -> closedLoop.getP(sparkSlot), slot + " kP");
      assertDevice(kS(slot), () -> closedLoop.feedForward.getkS(sparkSlot), slot + " kS");
      assertDevice(
          kV(slot) / RPM.of(kRatio * 60).in(RPM),
          () -> closedLoop.feedForward.getkV(sparkSlot),
          slot + " kV");
    } finally {
      close(smc);
    }
  }
}
