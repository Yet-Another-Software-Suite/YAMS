// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeFalse;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;
import static org.wpilib.units.Units.Seconds;

import java.util.function.DoubleSupplier;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.preferences.Preferences;
import org.wpilib.simulation.RoboRioSim;
import yams.commands2.mechanisms.Arm;
import yams.commands2.mechanisms.Elevator;
import yams.commands2.mechanisms.FlyWheel;
import yams.commands2.mechanisms.Pivot;
import yams.core.math.ADRCConfig;
import yams.core.math.ADRCConfig.SystemOrder;
import yams.core.mechanisms.SmartMechanism;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.mechanisms.config.ElevatorConfig;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.mechanisms.config.PivotConfig;
import yams.core.motorcontrollers.ClosedLoopCases.Controller;
import yams.helpers.MockHardwareExtension;

/**
 * Each YAMS mechanism (arm, elevator, pivot and flywheel) on every motor controller, closed with an
 * ADRC and simulated by the mechanism itself. The mechanisms need nothing for ADRC: they set their
 * setpoints as they do with a PID, and the motor controller runs the ADRC.
 *
 * <p>{@link ADRCClosedLoopTest} explains each mechanism's ADRC settings.
 */
public class ADRCMechanismTest {
  private static final double kSettleSeconds = 2.0;
  private static final double kHoldSeconds = 0.5;

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

  private static ADRCConfig adrc(SystemOrder order, double settlingSeconds) {
    return new ADRCConfig()
        .withSystemOrder(order)
        .withSettlingTime(Seconds.of(settlingSeconds))
        .withMaxVoltage(Volts.of(12));
  }

  private static yams.commands2.config.SmartMotorControllerConfig baseConfig(String name) {
    return ClosedLoopCases.baseConfig("ADRCMechanismTest " + name);
  }

  /**
   * Command a setpoint through the mechanism, step the mechanism's simulation, and check it settles
   * there and holds it.
   */
  private static void assertSettles(Controller controller, SmartMechanism mechanism,
      Runnable command, DoubleSupplier measured, double setpoint, double tolerance) {
    final double[] worst = {0};
    ClosedLoopCases.run(
        controller, mechanism::simIterate, kSettleSeconds + kHoldSeconds, command, time -> {
          if (time >= kSettleSeconds) {
            worst[0] = Math.max(worst[0], Math.abs(measured.getAsDouble() - setpoint));
          }
        });
    final double end = measured.getAsDouble();
    System.out.println(controller + " " + mechanism.getClass().getSimpleName() + ": ended at " + end
        + " (setpoint " + setpoint + "), at most " + worst[0] + " from it while holding");
    assertTrue(worst[0] <= tolerance,
        controller + ": should hold " + setpoint + " within " + tolerance + ", but was " + worst[0]
            + " from it, ending at " + end);
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("controllers")
  void arm(Controller controller) {
    final SmartMotorController smc = ClosedLoopCases.create(controller,
        baseConfig("arm " + controller)
            .withMomentOfInertia(KilogramSquareMeters.of(0.1))
            .withStartingPosition(Degrees.of(-30))
            .withADRCController(adrc(SystemOrder.SECOND, 0.6)));
    try {
      final Arm arm = new Arm(new ArmConfig()
                                  .withLength(Meters.of(0.5))
                                  .withHardLimits(Degrees.of(-180), Degrees.of(180)),
          smc);
      assertSettles(controller, arm,
          ()
              -> arm.setMechanismPositionSetpoint(Degrees.of(45)),
          () -> arm.getAngle().in(Degrees), 45, 1);
    } finally {
      ClosedLoopCases.close(smc);
    }
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("controllers")
  void pivot(Controller controller) {
    final SmartMotorController smc = ClosedLoopCases.create(controller,
        baseConfig("pivot " + controller)
            .withMomentOfInertia(KilogramSquareMeters.of(0.1))
            .withStartingPosition(Degrees.of(0))
            .withADRCController(adrc(SystemOrder.SECOND, 0.6)));
    try {
      final Pivot pivot =
          new Pivot(new PivotConfig().withHardLimits(Degrees.of(-180), Degrees.of(180)), smc);
      assertSettles(controller, pivot,
          ()
              -> pivot.setMechanismPositionSetpoint(Degrees.of(90)),
          () -> pivot.getAngle().in(Degrees), 90, 1);
    } finally {
      ClosedLoopCases.close(smc);
    }
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("controllers")
  void flyWheel(Controller controller) {
    final double setpoint = 0.5 * controller.motor().freeSpeed / (2 * Math.PI) / controller.ratio();
    final SmartMotorController smc = ClosedLoopCases.create(controller,
        baseConfig("flywheel " + controller)
            .withMomentOfInertia(KilogramSquareMeters.of(0.01))
            .withADRCController(adrc(SystemOrder.FIRST, 0.15)));
    try {
      final FlyWheel flyWheel = new FlyWheel(new FlyWheelConfig().withDiameter(Inches.of(4)), smc);
      assertSettles(controller, flyWheel,
          ()
              -> flyWheel.setMechanismVelocitySetpoint(RotationsPerSecond.of(setpoint)),
          () -> flyWheel.getSpeed().in(RotationsPerSecond), setpoint, 0.02 * setpoint);
    } finally {
      ClosedLoopCases.close(smc);
    }
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("controllers")
  void elevator(Controller controller) {
    assumeFalse(controller.name().equals("A301"), "The A301 cannot lift the elevator");
    final double drumRadius = 0.02;
    final yams.commands2.config.SmartMotorControllerConfig cfg =
        baseConfig("elevator " + controller)
            .withMechanismCircumference(Meters.of(2 * Math.PI * drumRadius))
            // A first order ADRC on the position, with the mechanism's speed per volt as b0.
            .withADRCController(
                adrc(SystemOrder.FIRST, 0.5).withB0(controller.motor().Kv / (2 * Math.PI * controller.ratio())));
    cfg.withStartingPosition(cfg.convertToMechanism(Meters.of(0.1)));
    final SmartMotorController smc = ClosedLoopCases.create(controller, cfg);
    try {
      final Elevator elevator = new Elevator(new ElevatorConfig()
                                                 .withCarriageWeight(Kilograms.of(10))
                                                 .withHardLimits(Meters.of(0), Meters.of(2)),
          smc);
      assertSettles(controller, elevator,
          ()
              -> elevator.setMeasurementPositionSetpoint(Meters.of(0.5)),
          () -> elevator.getHeight().in(Meters), 0.5, 0.01);
    } finally {
      ClosedLoopCases.close(smc);
    }
  }
}
