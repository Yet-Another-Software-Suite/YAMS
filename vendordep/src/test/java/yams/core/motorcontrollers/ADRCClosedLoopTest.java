// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeFalse;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Rotations;
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
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.simulation.SingleJointedArmSim;
import yams.core.math.ADRCConfig;
import yams.core.math.ADRCConfig.SystemOrder;
import yams.core.motorcontrollers.ClosedLoopCases.Controller;
import yams.core.motorcontrollers.simulation.ArmSimSupplier;
import yams.core.motorcontrollers.simulation.ElevatorSimSupplier;
import yams.helpers.MockHardwareExtension;

/**
 * Every motor controller closing its loop with an ADRC: a flywheel's velocity, a pivot's and an
 * arm's angle, and an elevator's height.
 *
 * <p>The ADRC runs on the SystemCore for every motor controller, from the same config. Its {@code
 * b0} is derived from the motor, the gearing and the moment of inertia, and the configs have no
 * feedforward: the ADRC's observer estimates gravity, friction and back-EMF as one disturbance and
 * cancels it. So an arm or elevator that ends at its setpoint, with no kG, shows the observer
 * works.
 */
public class ADRCClosedLoopTest {
  /** How long each mechanism is given to settle at its setpoint. */
  private static final double kSettleSeconds = 2.0;

  /** How long it is then watched holding there. */
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

  /**
   * An ADRC config with no file in the deploy directory, so only the values set here apply.
   *
   * @param order        Plant order: 1 for a velocity, 2 for a position.
   * @param settlingTime Desired settling time, seconds.
   * @return The config.
   */
  static ADRCConfig adrc(SystemOrder order, double settlingSeconds) {
    return new ADRCConfig()
        .withSystemOrder(order)
        .withSettlingTime(Seconds.of(settlingSeconds));
  }

  /**
   * The {@code b0} {@link ADRCConfig} derives from the moment of inertia alone: mechanism rotations
   * per second squared per volt, G Kt / (2 pi R J).
   *
   * @param controller Motor controller, with its motor and gearing.
   * @param moi        Mechanism moment of inertia, kg m^2.
   * @return The derived {@code b0}.
   */
  private static double inertiaB0(Controller controller, double moi) {
    return controller.ratio() * controller.motor().Kt / (2 * Math.PI * controller.motor().R * moi);
  }

  private static yams.commands2.config.SmartMotorControllerConfig baseConfig(String name) {
    return ClosedLoopCases.baseConfig("ADRCClosedLoopTest " + name);
  }

  /**
   * Command a setpoint, and check the mechanism settles there and holds it.
   *
   * @param controller Motor controller under test.
   * @param smc        The motor controller.
   * @param command    Commands the setpoint.
   * @param measured   The measured value.
   * @param setpoint   The setpoint.
   * @param tolerance  How far from the setpoint it may be while holding it. A Talon, simulated in
   *                   real time, is held to it on average (see {@link ClosedLoopCases.HoldError}).
   */
  private static void assertSettles(Controller controller, SmartMotorController smc,
      Runnable command, DoubleSupplier measured, double setpoint, double tolerance) {
    final ClosedLoopCases.HoldError error = new ClosedLoopCases.HoldError(controller);
    ClosedLoopCases.run(controller, smc, kSettleSeconds + kHoldSeconds, command, time -> {
      if (time >= kSettleSeconds) {
        error.add(measured.getAsDouble() - setpoint);
      }
    });
    final double end = measured.getAsDouble();
    System.out.println(controller + ": ended at " + end + " (setpoint " + setpoint + "), " + error
        + " while holding");
    assertTrue(error.judged() <= tolerance,
        controller + ": should hold " + setpoint + " within " + tolerance + ", but was " + error
            + ", ending at " + end);
  }

  /** A flywheel's velocity loop: a first order plant. */
  @ParameterizedTest(name = "{0}")
  @MethodSource("controllers")
  void flywheelReachesItsVelocity(Controller controller) {
    // Half the motor's free speed, at the mechanism.
    final double setpoint = 0.5 * controller.motor().freeSpeed / (2 * Math.PI) / controller.ratio();
    final SmartMotorController smc = ClosedLoopCases.create(controller,
        baseConfig("flywheel " + controller)
            .withMomentOfInertia(KilogramSquareMeters.of(0.01))
            // The observer runs at four times the closed loop bandwidth. It has to be faster than
            // the motor's back-EMF, which it estimates as part of the disturbance: about 220 rad/s
            // for a Kraken on this flywheel. A 0.15 s settling time puts it at 213 rad/s.
            .withClosedLoopControllerMaximumVoltage(Volts.of(12))
            .withADRCController(adrc(SystemOrder.FIRST, 0.15)));
    try {
      assertSettles(controller, smc,
          ()
              -> smc.setVelocity(RotationsPerSecond.of(setpoint)),
          () -> smc.getMechanismVelocity().in(RotationsPerSecond), setpoint, 0.02 * setpoint);
    } finally {
      ClosedLoopCases.close(smc);
    }
  }

  /** A pivot with no gravity, such as a turret: a second order plant. */
  @ParameterizedTest(name = "{0}")
  @MethodSource("controllers")
  void pivotReachesItsAngle(Controller controller) {
    final double setpoint = Degrees.of(90).in(Rotations);
    final SmartMotorController smc = ClosedLoopCases.create(controller,
        baseConfig("pivot " + controller)
            .withMomentOfInertia(KilogramSquareMeters.of(0.1))
            .withStartingPosition(Rotations.of(0))
            .withClosedLoopControllerMaximumVoltage(Volts.of(12))
            .withADRCController(adrc(SystemOrder.PIVOT, 0.6)));
    try {
      assertSettles(controller, smc,
          ()
              -> smc.setPosition(Rotations.of(setpoint)),
          () -> smc.getMechanismPosition().in(Rotations), setpoint, Degrees.of(1).in(Rotations));
    } finally {
      ClosedLoopCases.close(smc);
    }
  }

  /** An arm under gravity, with no kG: the observer has to find and cancel gravity. */
  @ParameterizedTest(name = "{0}")
  @MethodSource("controllers")
  void armReachesItsAngleAgainstGravity(Controller controller) {
    final double moi = 0.1;
    final double length = 0.5;
    final double start = Degrees.of(-30).in(Rotations);
    final double setpoint = Degrees.of(45).in(Rotations);
    final SmartMotorController smc = ClosedLoopCases.create(controller,
        baseConfig("arm " + controller)
            .withMomentOfInertia(KilogramSquareMeters.of(moi))
            .withStartingPosition(Rotations.of(start))
            .withClosedLoopControllerMaximumVoltage(Volts.of(12))
            .withADRCController(adrc(SystemOrder.ARM, 0.6)));
    try {
      final SingleJointedArmSim sim = new SingleJointedArmSim(controller.motor(),
          controller.ratio(), moi, length, Degrees.of(-180).in(Radians),
          Degrees.of(180).in(Radians), true, Rotations.of(start).in(Radians));
      smc.setSimSupplier(new ArmSimSupplier(sim, smc));
      smc.setupSimulation();
      assertSettles(controller, smc,
          ()
              -> smc.setPosition(Rotations.of(setpoint)),
          () -> smc.getMechanismPosition().in(Rotations), setpoint, Degrees.of(1).in(Rotations));
    } finally {
      ClosedLoopCases.close(smc);
    }
  }

  /**
   * An elevator under gravity, with no kG, closed in meters.
   *
   * <p>An elevator's back-EMF, not its inertia, sets how fast it moves: it reaches the speed its
   * voltage gives it within a few milliseconds, far quicker than a 50 Hz loop can act on. So a
   * second order ADRC with the inertia's {@code b0} has an observer too slow for the disturbance,
   * and {@code SystemOrder.ELEVATOR}, a first order ADRC on its position with its speed per volt as
   * {@code b0}, fits it.
   */
  @ParameterizedTest(name = "{0}")
  @MethodSource("controllers")
  void elevatorReachesItsHeightAgainstGravity(Controller controller) {
    // Lifting 10 kg at the speeds an ADRC asks for takes more than the A301 motor's 28 W peak.
    assumeFalse(controller.name().equals("A301"), "The A301 cannot lift the elevator");
    final double mass = 10;
    final double drumRadius = 0.02;
    final double start = 0.1;
    final double setpoint = 0.5;
    final yams.commands2.config.SmartMotorControllerConfig cfg =
        baseConfig("elevator " + controller)
            .withMechanismCircumference(Meters.of(2 * Math.PI * drumRadius))
            .withLinearClosedLoopController(true)
            .withMomentOfInertia(KilogramSquareMeters.of(mass * drumRadius * drumRadius))
            .withClosedLoopControllerMaximumVoltage(Volts.of(12))
            // An elevator: first order on its position, in meters, with its speed per volt as b0.
            .withADRCController(adrc(SystemOrder.ELEVATOR, 0.5));
    cfg.withStartingPosition(cfg.convertToMechanism(Meters.of(start)));
    final SmartMotorController smc = ClosedLoopCases.create(controller, cfg);
    try {
      final ElevatorSim sim = new ElevatorSim(
          controller.motor(), controller.ratio(), mass, drumRadius, 0, 2, true, start, 0.0, 0.0);
      smc.setSimSupplier(new ElevatorSimSupplier(sim, smc));
      smc.setupSimulation();
      assertSettles(controller, smc,
          ()
              -> smc.setPosition(Meters.of(setpoint)),
          () -> smc.getMeasurementPosition().in(Meters), setpoint, 0.01);
    } finally {
      ClosedLoopCases.close(smc);
    }
  }
}
