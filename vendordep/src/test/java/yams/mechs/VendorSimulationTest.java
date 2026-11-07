// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechs;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.preferences.Preferences;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.util.Pair;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.Pivot;
import yams.commands2.telemetry.SmartMotorControllerCommandRegistry;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.PivotConfig;
import yams.core.motorcontrollers.SimSupplier;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.motorcontrollers.remote.TalonFXSWrapper;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.motorcontrollers.simulation.BatterySim;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;
import yams.helpers.TestWithScheduler;

/**
 * The motor controllers with a vendor simulation, REV's SparkSim for a SPARK and Phoenix's for a
 * Talon, are simulated through it, on a simulated pivot:
 *
 * <ul>
 *   <li>The motor controller's encoder, read from its vendor simulation, follows the pivot.
 *   <li>The pivot is given the output the vendor simulation applies.
 *   <li>The vendor simulation limits the current, so a lower stator current limit applies less
 *       output and moves the pivot less.
 *   <li>A PID, a feedforward and an exponential profile each take the pivot to the setpoint, with
 *       no follower, a tightly coupled follower or a loosely coupled follower, and the vendor
 *       simulation's duty cycle, voltage, current and velocity react while they do, the loosely
 *       coupled follower's too.
 * </ul>
 */
public class VendorSimulationTest {
  private static final MechanismGearing GEARING =
      new MechanismGearing(GearBox.fromReductionStages(3, 4, 5));
  private static final double SETPOINT_DEGREES = 45;

  private static int offset = 0;

  private static SmartMotorControllerConfig config(String label, ControlMode mode, double currentLimitAmps) {
    return config(label, mode, currentLimitAmps, 4, 1);
  }

  /**
   * A config for a pivot with a moment of inertia of a mass at a radius.
   *
   * @param radiusInches Radius of the mass.
   * @param massPounds Mass on the pivot.
   */
  private static SmartMotorControllerConfig config(
      String label, ControlMode mode, double currentLimitAmps, double radiusInches, double massPounds) {
    SmartMotorControllerTestSubsystem subsys = new SmartMotorControllerTestSubsystem();
    subsys.setName(label + "[" + offset + "]");
    SmartMotorControllerConfig cfg = (SmartMotorControllerConfig) new SmartMotorControllerConfig()
        .withGearing(GEARING)
        .withZeroPower(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(currentLimitAmps))
        .withMotorInverted(false)
        .withControlMode(mode)
        .withMomentOfInertia(Inches.of(radiusInches), Pounds.of(massPounds))
        .withStartingPosition(Degrees.of(0))
        .withSubsystem(subsys)
        .withTelemetry(label + "[" + offset + "]", TelemetryVerbosity.HIGH);
    if (mode == ControlMode.CLOSED_LOOP) {
      cfg.withClosedLoopController(40, 0, 0);
    }
    return cfg;
  }

  /**
   * Phoenix runs a Talon's simulation and status signals in real time, while the tests step the
   * robot's time faster, so a Talon's loops are paced at the real loop period.
   */
  private static boolean s_realTime = false;

  /** Every loop of a cycle, paced at 20 ms of real time for a Talon. */
  private static Runnable paced(Runnable everyLoop) {
    return () -> {
      if (s_realTime) {
        try {
          Thread.sleep(20);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
        }
      }
      everyLoop.run();
    };
  }

  private static Stream<Arguments> vendorSimulated() {
    return Stream.of(
        Arguments.of("SPARK MAX", (Function<SmartMotorControllerConfig, SmartMotorController>)
            cfg -> new SparkWrapper(DeviceCreator.createSparkMax(), DCMotor.getNEO(1), cfg)),
        Arguments.of("SPARK Flex", (Function<SmartMotorControllerConfig, SmartMotorController>)
            cfg -> new SparkWrapper(DeviceCreator.createSparkFlex(), DCMotor.getNeoVortex(1), cfg)),
        // Phoenix runs a Talon's simulation in real time.
        Arguments.of("TalonFX", (Function<SmartMotorControllerConfig, SmartMotorController>)
            cfg -> new TalonFXWrapper(DeviceCreator.createTalonFX(), DCMotor.getKrakenX60(1), cfg)),
        Arguments.of("TalonFXS", (Function<SmartMotorControllerConfig, SmartMotorController>)
            cfg -> new TalonFXSWrapper(DeviceCreator.createTalonFXS(), DCMotor.getNEO(1), cfg)));
  }

  /** A pivot on the motor controller, simulated and pumped by its test subsystem. */
  private static SmartMotorControllerTestSubsystem pivot(SmartMotorController smc) {
    SmartMotorControllerTestSubsystem subsys = subsystem(smc);
    subsys.setSMC(smc);
    Pivot pivot = new Pivot(new PivotConfig().withHardLimits(Degrees.of(-180), Degrees.of(180)), smc);
    subsys.mechSimPeriodic = pivot::simIterate;
    subsys.mechUpdateTelemetry = pivot::updateTelemetry;
    subsys.testRunning = true;
    return subsys;
  }

  private static SmartMotorControllerTestSubsystem subsystem(SmartMotorController smc) {
    return (SmartMotorControllerTestSubsystem) ((SmartMotorControllerConfig) smc.getConfig()).getSubsystem();
  }

  private static SimSupplier mechanism(SmartMotorController smc) {
    return smc.getSimSupplier().orElseThrow();
  }

  private static void close(SmartMotorController smc) {
    // A command still running would command the closed motor controller.
    CommandScheduler.getInstance().cancelAll();
    SmartMotorControllerTestSubsystem subsys = subsystem(smc);
    SmartMotorControllerCommandRegistry.removeCommands(subsys);
    CommandScheduler.getInstance().unregisterSubsystem(subsys);
    // The subsystem closes the motor controller it pumps, so give it one if no test did.
    subsys.setSMC(smc);
    subsys.close();
    smc.close();
    DeviceCreator.silence(smc);
    Object motor = smc.getMotorController();
    if (motor instanceof SparkMax spark) {
      spark.close();
    } else if (motor instanceof SparkFlex spark) {
      spark.close();
    } else if (motor instanceof TalonFX talon) {
      talon.close();
    } else if (motor instanceof TalonFXS talon) {
      talon.close();
    }
  }

  /**
   * Drive the pivot at a voltage for a time, in one cycle, as each restarts the robot's clock.
   *
   * @return The pivot's angle in degrees after the time.
   */
  private static double runAtVoltage(SmartMotorController smc, double volts, double seconds, Runnable everyLoop)
      throws InterruptedException {
    smc.setVoltage(Volts.of(volts));
    TestWithScheduler.cycle(Seconds.of(seconds), paced(everyLoop));
    return mechanism(smc).getMechanismPosition().in(Degrees);
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("vendorSimulated")
  void vendorEncoderFollowsTheMechanism(
      String name, Function<SmartMotorControllerConfig, SmartMotorController> create) throws InterruptedException {
    SmartMotorController smc = create.apply(config("Encoder(" + name + ")", ControlMode.OPEN_LOOP, 40));
    s_realTime = smc instanceof TalonFXWrapper || smc instanceof TalonFXSWrapper;
    try {
      pivot(smc);
      double degrees = runAtVoltage(smc, 2, 0.5, () -> {});
      assertTrue(degrees > 10, name + ": pivot did not move, at " + degrees + "°");
      // The encoder the motor controller reports comes from its vendor simulation.
      assertEquals(degrees, smc.getMechanismPosition().in(Degrees), 1, name + ": encoder position");
      double velocity = mechanism(smc).getMechanismVelocity().in(DegreesPerSecond);
      assertEquals(velocity, smc.getMechanismVelocity().in(DegreesPerSecond), Math.abs(velocity) * 0.05 + 2,
          name + ": encoder velocity");
    } finally {
      close(smc);
    }
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("vendorSimulated")
  void mechanismIsGivenTheVendorOutput(
      String name, Function<SmartMotorControllerConfig, SmartMotorController> create) throws InterruptedException {
    SmartMotorController smc = create.apply(config("Output(" + name + ")", ControlMode.OPEN_LOOP, 40));
    s_realTime = smc instanceof TalonFXWrapper || smc instanceof TalonFXSWrapper;
    try {
      pivot(smc);
      double[] largestError = {0};
      double[] largestDutyCycle = {0};
      int[] loops = {0};
      runAtVoltage(smc, 3, 0.5, () -> {
        // After the vendor simulation has applied the command: a Talon's motor voltage reaches the
        // pivot in its next simIterate(), so it is a loop behind while its output rises.
        if (++loops[0] > 5) {
          double vendorVolts = smc.getDutyCycle() * RoboRioSim.getVInVoltage();
          double mechanismVolts = mechanism(smc).getMechanismStatorVoltage().in(Volts);
          largestError[0] = Math.max(largestError[0], Math.abs(vendorVolts - mechanismVolts));
          largestDutyCycle[0] = Math.max(largestDutyCycle[0], Math.abs(smc.getDutyCycle()));
        }
      });
      assertTrue(largestDutyCycle[0] > 0.1, name + ": the vendor simulation applied no output");
      assertTrue(largestError[0] < 0.3, name + ": the pivot was given up to " + largestError[0]
          + " V more or less than the vendor simulation applied");
    } finally {
      close(smc);
    }
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("vendorSimulated")
  void vendorSimulationLimitsTheCurrent(
      String name, Function<SmartMotorControllerConfig, SmartMotorController> create) throws InterruptedException {
    double[] degrees = new double[2];
    double[] dutyCycle = new double[2];
    double[] limitsAmps = {120, 10};
    for (int i = 0; i < 2; i++) {
      SmartMotorController smc =
          // A heavy pivot, so the motor is still slow, and drawing a high current, at 100 ms.
          create.apply(config("Limit" + (int) limitsAmps[i] + "(" + name + ")", ControlMode.OPEN_LOOP, limitsAmps[i], 36, 30));
      s_realTime = smc instanceof TalonFXWrapper || smc instanceof TalonFXSWrapper;
      try {
        pivot(smc);
        // From a standstill at 12 V, a motor draws far more than 10 A, until it is near its free speed.
        int[] loops = {0};
        int sample = i;
        degrees[i] = runAtVoltage(smc, 12, 0.2, () -> {
          if (++loops[0] == 5) {
            dutyCycle[sample] = Math.abs(smc.getDutyCycle());
          }
        });
      } finally {
        close(smc);
      }
      BatterySim.reset();
      RoboRioSim.setVInVoltage(12.0);
    }
    assertTrue(dutyCycle[1] < dutyCycle[0] * 0.8, name + ": a 10 A limit applied a duty cycle of " + dutyCycle[1]
        + ", and a 120 A limit " + dutyCycle[0]);
    assertTrue(Math.abs(degrees[1]) < Math.abs(degrees[0]) * 0.8, name + ": a 10 A limit moved the pivot "
        + degrees[1] + "°, and a 120 A limit " + degrees[0] + "°");
  }

  /** A vendor simulated motor controller, and a motor controller device of its vendor to follow it. */
  private record Vendor(
      String name,
      BiFunction<SmartMotorControllerConfig, Integer, SmartMotorController> create,
      Supplier<Object> device,
      DCMotor motor) {}

  private static final List<Vendor> VENDORS = List.of(
      new Vendor("SPARK MAX", (cfg, motors) -> new SparkWrapper(DeviceCreator.createSparkMax(), DCMotor.getNEO(motors), cfg),
          DeviceCreator::createSparkMax, DCMotor.getNEO(1)),
      new Vendor("SPARK Flex", (cfg, motors) -> new SparkWrapper(DeviceCreator.createSparkFlex(), DCMotor.getNeoVortex(motors), cfg),
          DeviceCreator::createSparkFlex, DCMotor.getNeoVortex(1)),
      new Vendor("TalonFX", (cfg, motors) -> new TalonFXWrapper(DeviceCreator.createTalonFX(), DCMotor.getKrakenX60(motors), cfg),
          DeviceCreator::createTalonFX, DCMotor.getKrakenX60(1)),
      new Vendor("TalonFXS", (cfg, motors) -> new TalonFXSWrapper(DeviceCreator.createTalonFXS(), DCMotor.getNEO(motors), cfg),
          DeviceCreator::createTalonFXS, DCMotor.getNEO(1)));

  /** How the closed loop gets to the setpoint. */
  private enum Control { PID, FEEDFORWARD, EXPONENTIAL_PROFILE }

  /** The motors following the vendor simulated motor controller. */
  private enum Followers { NONE, TIGHTLY_COUPLED, LOOSELY_COUPLED }

  private static Stream<Arguments> controlsAndFollowers() {
    List<Arguments> cases = new ArrayList<>();
    for (Vendor vendor : VENDORS) {
      for (Control control : Control.values()) {
        for (Followers followers : Followers.values()) {
          cases.add(Arguments.of(vendor.name() + ", " + control + ", " + followers + " followers", vendor, control, followers));
        }
      }
    }
    return cases.stream();
  }

  /** A closed loop config for the control, with no current limit worth reaching. */
  private static SmartMotorControllerConfig controlConfig(String label, Control control, DCMotor motor) {
    SmartMotorControllerConfig cfg = config(label, ControlMode.OPEN_LOOP, 80);
    cfg.withControlMode(ControlMode.CLOSED_LOOP);
    switch (control) {
      case PID -> cfg.withClosedLoopController(40, 0, 0);
      case FEEDFORWARD -> {
        // Volts per pivot rotation per second, from the motor's free speed through the gearing.
        double freeRotationsPerSecond = motor.freeSpeed / (2 * Math.PI) / GEARING.getMechanismToRotorRatio();
        cfg.withClosedLoopController(0, 0, 0)
            .withFeedforward(new SimpleMotorFeedforward(0, motor.nominalVoltage / freeRotationsPerSecond, 0))
            .withTrapezoidalProfile(DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(180));
      }
      case EXPONENTIAL_PROFILE -> cfg.withClosedLoopController(40, 0, 0)
          .withExponentialProfile(Volts.of(12), DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90));
    }
    return cfg;
  }

  /** The largest magnitudes of a motor controller's vendor simulated values during a run. */
  private static final class Reacted {
    double dutyCycle;
    double volts;
    double statorAmps;
    double degreesPerSecond;

    void sample(SmartMotorController smc) {
      dutyCycle = Math.max(dutyCycle, Math.abs(smc.getDutyCycle()));
      volts = Math.max(volts, Math.abs(smc.getVoltage().in(Volts)));
      statorAmps = Math.max(statorAmps, Math.abs(smc.getStatorCurrent().in(Amps)));
      degreesPerSecond = Math.max(degreesPerSecond, Math.abs(smc.getMechanismVelocity().in(DegreesPerSecond)));
    }

    void assertReacted(String name) {
      assertTrue(dutyCycle > 0.02, name + ": duty cycle never above " + dutyCycle);
      assertTrue(volts > 0.2, name + ": voltage never above " + volts + " V");
      assertTrue(statorAmps > 0.1, name + ": stator current never above " + statorAmps + " A");
      assertTrue(degreesPerSecond > 5, name + ": velocity never above " + degreesPerSecond + "°/s");
    }
  }

  private static void closeDevice(Object device) {
    if (device instanceof SparkMax spark) {
      spark.close();
    } else if (device instanceof SparkFlex spark) {
      spark.close();
    } else if (device instanceof TalonFX talon) {
      talon.close();
    } else if (device instanceof TalonFXS talon) {
      talon.close();
    }
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("controlsAndFollowers")
  void vendorControlReachesTheSetpoint(String name, Vendor vendor, Control control, Followers followers)
      throws InterruptedException {
    Object tightFollower = null;
    SmartMotorController looseFollower = null;
    SmartMotorControllerConfig cfg = controlConfig("Control(" + name + ")", control, vendor.motor());
    int motors = 1;
    if (followers == Followers.TIGHTLY_COUPLED) {
      tightFollower = vendor.device().get();
      cfg.withFollowers(Pair.of(tightFollower, false));
      motors = 2;
    } else if (followers == Followers.LOOSELY_COUPLED) {
      looseFollower = vendor.create().apply(controlConfig("Follower(" + name + ")", control, vendor.motor()), 1);
      cfg.withLooselyCoupledFollowers(looseFollower);
    }
    SmartMotorController smc = vendor.create().apply(cfg, motors);
    s_realTime = smc instanceof TalonFXWrapper || smc instanceof TalonFXSWrapper;
    try {
      SmartMotorControllerTestSubsystem subsys = pivot(smc);
      Reacted leader = new Reacted();
      Reacted follower = new Reacted();
      SmartMotorController loose = looseFollower;
      TestWithScheduler.schedule(subsys.setPositionSetpoint(Degrees.of(SETPOINT_DEGREES)));
      TestWithScheduler.cycle(Seconds.of(3), paced(() -> {
        leader.sample(smc);
        if (loose != null) {
          follower.sample(loose);
        }
      }));

      double tolerance = control == Control.FEEDFORWARD ? 10 : 3;
      assertEquals(SETPOINT_DEGREES, mechanism(smc).getMechanismPosition().in(Degrees), tolerance, name + ": pivot");
      assertEquals(SETPOINT_DEGREES, smc.getMechanismPosition().in(Degrees), tolerance, name + ": encoder");
      leader.assertReacted(name + " leader");
      if (loose != null) {
        follower.assertReacted(name + " loosely coupled follower");
        assertEquals(SETPOINT_DEGREES, loose.getMechanismPosition().in(Degrees), tolerance,
            name + ": loosely coupled follower's encoder");
      }
    } finally {
      close(smc);
      if (looseFollower != null) {
        close(looseFollower);
      }
      closeDevice(tightFollower);
    }
  }

  @BeforeEach
  void beforeTest() {
    offset++;
    MockHardwareExtension.beforeAll();
    RoboRioSim.setVInVoltage(12.0);
    BatterySim.reset();
    TestWithScheduler.schedulerStart();
    TestWithScheduler.schedulerClear();
  }

  @AfterEach
  void afterTest() {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
    TestWithScheduler.schedulerClear();
    BatterySim.reset();
  }
}
