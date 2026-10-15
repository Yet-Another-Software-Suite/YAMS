// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;
import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.thrifty.nova.Nova;
import java.util.function.BiFunction;
import java.util.function.DoubleSupplier;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.controller.ElevatorFeedforward;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.preferences.Preferences;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.system.Timer;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.telemetry.SmartMotorControllerCommandRegistry;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.SmartMotorController.ClosedLoopControllerSlot;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.NovaWrapper;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.motorcontrollers.remote.TalonFXSWrapper;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.motorcontrollers.simulation.ArmSimSupplier;
import yams.core.motorcontrollers.simulation.ElevatorSimSupplier;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.PeriodicScheduler;
import yams.helpers.SmartMotorControllerTestSubsystem;

/**
 * Tests that every {@link SmartMotorController} follows a trapezoidal profile from each closed loop
 * slot, given the same gains in YAMS units.
 *
 * <p>Each mechanism's feedforward is worked out from its simulated physics, so a feedforward in the
 * right units moves the mechanism along the profile with little left for kP to correct. kP, kV and
 * kA are per mechanism rotation (per meter for an elevator), and kS and kG are in volts, on every
 * motor controller. The other slots are given four times the gains, so a controller that runs another
 * slot's gains, or converts a gain into the wrong units, falls away from the profile.
 */
public class FeedforwardProfileTest {
  /** 12 motor rotations per mechanism rotation. */
  private static final MechanismGearing kGearing =
      new MechanismGearing(GearBox.fromReductionStages(3, 4));

  private static final double kRatio = 12;
  private static final double kGravity = 9.81;

  /** Gains of the slots that are not selected, as a multiple of the selected slot's gains. */
  private static final double kOtherSlotScale = 4;

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

  /**
   * A motor controller to test.
   *
   * @param name Name of the motor controller.
   * @param motor Motor it drives.
   * @param slots Number of closed loop slots it has.
   * @param realTime Whether its simulation runs in real time. Phoenix simulates a Talon's Motion
   *     Magic on its own clock, which does not stop with WPILib's simulated time.
   * @param create Creates the motor controller.
   */
  private record Controller(
      String name,
      DCMotor motor,
      int slots,
      boolean realTime,
      BiFunction<DCMotor, SmartMotorControllerConfig, SmartMotorController> create) {
    @Override
    public String toString() {
      return name;
    }
  }

  private static final Controller[] kControllers = {
    new Controller(
        "SparkMax",
        DCMotor.getNEO(1),
        4,
        false,
        (motor, cfg) -> new SparkWrapper(DeviceCreator.createSparkMax(), motor, cfg)),
    new Controller(
        "SparkFlex",
        DCMotor.getNeoVortex(1),
        4,
        false,
        (motor, cfg) -> new SparkWrapper(DeviceCreator.createSparkFlex(), motor, cfg)),
    new Controller(
        "TalonFX",
        DCMotor.getKrakenX60(1),
        3,
        true,
        (motor, cfg) -> new TalonFXWrapper(DeviceCreator.createTalonFX(), motor, cfg)),
    new Controller(
        "TalonFXS",
        DCMotor.getNEO(1),
        3,
        true,
        (motor, cfg) -> new TalonFXSWrapper(DeviceCreator.createTalonFXS(), motor, cfg)),
    new Controller(
        "Nova",
        DCMotor.getNEO(1),
        4,
        false,
        (motor, cfg) -> new NovaWrapper(DeviceCreator.createNova(), motor, cfg)),
  };

  /** Every motor controller with each of its closed loop slots. */
  private static Stream<Arguments> controllersAndSlots() {
    return Stream.of(kControllers)
        .flatMap(
            controller ->
                Stream.of(ClosedLoopControllerSlot.values())
                    .limit(controller.slots())
                    .map(slot -> Arguments.of(controller, slot)));
  }

  /** The closed loop slots a motor controller has. */
  private static Stream<ClosedLoopControllerSlot> slots(Controller controller) {
    return Stream.of(ClosedLoopControllerSlot.values()).limit(controller.slots());
  }

  /** Volts per mechanism radian per second squared: J R / (G Kt). */
  private static double voltsPerRadianPerSecondSquared(DCMotor motor, double moi) {
    return moi * motor.R / (kRatio * motor.Kt);
  }

  /** Volts per mechanism radian per second: G / Kv. */
  private static double voltsPerRadianPerSecond(DCMotor motor) {
    return kRatio / motor.Kv;
  }

  private static SmartMotorControllerConfig baseConfig(String name) {
    return new yams.commands2.config.SmartMotorControllerConfig()
        .withSubsystem(new SmartMotorControllerTestSubsystem())
        .withGearing(kGearing)
        .withStatorCurrentLimit(Amps.of(80))
        .withZeroPower(MotorMode.BRAKE)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withTelemetry("FeedforwardProfileTest " + name, TelemetryVerbosity.LOW);
  }

  /** The gains of a slot: the selected slot's own, or {@link #kOtherSlotScale} times them. */
  private static double scale(ClosedLoopControllerSlot slot, ClosedLoopControllerSlot selected) {
    return slot == selected ? 1 : kOtherSlotScale;
  }

  private static SmartMotorController create(Controller controller, SmartMotorControllerConfig cfg) {
    final SmartMotorController smc = controller.create().apply(controller.motor(), cfg);
    ((SmartMotorControllerTestSubsystem)
            ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem())
        .setSMC(smc);
    return smc;
  }

  private static void close(SmartMotorController smc) {
    SmartMotorControllerTestSubsystem subsys =
        (SmartMotorControllerTestSubsystem)
            ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem();
    SmartMotorControllerCommandRegistry.removeCommands(subsys);
    CommandScheduler.getInstance().unregisterSubsystem(subsys);
    subsys.close();
    smc.close();
    DeviceCreator.silence(smc);
    Object motorController = smc.getMotorController();
    if (motorController instanceof SparkMax sparkMax) {
      sparkMax.close();
    } else if (motorController instanceof SparkFlex sparkFlex) {
      sparkFlex.close();
    } else if (motorController instanceof TalonFXS talonFXS) {
      talonFXS.close();
    } else if (motorController instanceof TalonFX talonFX) {
      talonFX.close();
    } else if (motorController instanceof Nova nova) {
      nova.close();
    }
  }

  /** Period of the robot loop and of the simulation, in seconds. */
  private static final double kPeriod = 0.020;

  /**
   * Command a profiled move with the slot selected, and check the mechanism follows the profile and
   * ends at the goal.
   *
   * <p>The mechanism may trail the profile by up to four loops of travel at the profile's cruise
   * velocity: a closed loop that runs on the SystemCore, as the Nova's does, applies each step of the
   * profile one loop after it is computed, and a simulated Talon starts its Motion Magic two to three
   * loops after it is commanded. A feedforward in the wrong units, or another slot's gains, leaves it
   * much further behind.
   *
   * @param controller Motor controller under test.
   * @param smc The motor controller, with its simulation set up at {@code start}.
   * @param slot Slot to select.
   * @param command Commands the goal.
   * @param position Mechanism position, in the units of the profile.
   * @param constraints Constraints of the profile the motor controller is configured with.
   * @param start Starting position.
   * @param goal Goal position.
   */
  private static void assertFollowsProfile(
      Controller controller,
      SmartMotorController smc,
      ClosedLoopControllerSlot slot,
      Runnable command,
      DoubleSupplier position,
      TrapezoidProfile.Constraints constraints,
      double start,
      double goal) {
    final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    final TrapezoidProfile.State startState = new TrapezoidProfile.State(start, 0);
    final TrapezoidProfile.State goalState = new TrapezoidProfile.State(goal, 0);
    final double tolerance = 4 * constraints.maxVelocity * kPeriod;
    final double settle = 1.0;
    final double[] time = {0};
    // The largest distance from the profile, and when it was.
    final double[] worst = {0, 0};
    try (PeriodicScheduler scheduler = new PeriodicScheduler()) {
      smc.setClosedLoopSlot(slot);
      command.run();
      final long startNanos = System.nanoTime();
      scheduler.addPeriodic(
          () -> {
            smc.simIterate();
            time[0] += kPeriod;
            command.run();
            final double error =
                Math.abs(
                    position.getAsDouble()
                        - profile.calculate(time[0], startState, goalState).position);
            if (error > worst[0]) {
              worst[0] = error;
              worst[1] = time[0];
            }
            if (controller.realTime()) {
              // Keep simulated time with the wall clock, rather than wait a whole period after each
              // step, so Phoenix's clock does not run ahead by the time each step takes.
              final double behind = time[0] - (System.nanoTime() - startNanos) / 1e9;
              if (behind > 0) {
                Timer.delay(Seconds.of(behind));
              }
            }
          },
          Seconds.of(kPeriod));
      scheduler.runFor(Seconds.of(profile.timeLeftUntil(startState, goalState) + settle));
    }
    final double end = position.getAsDouble();
    final String what = controller + " " + slot;
    System.out.println(
        what + ": at most " + worst[0] + " from the profile, at " + worst[1] + " s; ended at " + end
            + " (goal " + goal + ")");
    assertTrue(
        worst[0] <= tolerance,
        what + ": should follow the profile within " + tolerance + ", but was " + worst[0]
            + " from it at " + worst[1] + " s");
    assertTrue(
        Math.abs(end - goal) <= tolerance / 2,
        what + ": should end at " + goal + ", but ended at " + end);
  }

  /** A rotating mechanism with no gravity, such as a turret, with a {@link SimpleMotorFeedforward}. */
  @ParameterizedTest(name = "{0} {1}")
  @MethodSource("controllersAndSlots")
  void rotaryFollowsProfile(Controller controller, ClosedLoopControllerSlot selected) {
    final double moi = 0.1;
    final double kV = 2 * Math.PI * voltsPerRadianPerSecond(controller.motor());
    final double kA = 2 * Math.PI * voltsPerRadianPerSecondSquared(controller.motor(), moi);
    final double kP = 10;
    final double maxVelocity = 1;
    final double maxAcceleration = 4;
    final SmartMotorControllerConfig cfg =
        baseConfig("rotary " + controller + " " + selected)
            .withMomentOfInertia(KilogramSquareMeters.of(moi))
            .withStartingPosition(Rotations.of(0))
            .withTrapezoidalProfile(
                RotationsPerSecond.of(maxVelocity),
                RotationsPerSecondPerSecond.of(maxAcceleration));
    for (ClosedLoopControllerSlot slot : slots(controller).toList()) {
      final double scale = scale(slot, selected);
      cfg.withClosedLoopController(kP * scale, 0, 0, slot)
          .withFeedforward(new SimpleMotorFeedforward(0, kV * scale, kA * scale), slot);
    }
    final SmartMotorController smc = create(controller, cfg);
    try {
      final double goal = 1.5;
      assertFollowsProfile(
          controller,
          smc,
          selected,
          () -> smc.setPosition(Rotations.of(goal)),
          () -> smc.getMechanismPosition().in(Rotations),
          new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration),
          0,
          goal);
    } finally {
      close(smc);
    }
  }

  /** An arm under gravity, with an {@link ArmFeedforward}. */
  @ParameterizedTest(name = "{0} {1}")
  @MethodSource("controllersAndSlots")
  void armFollowsProfile(Controller controller, ClosedLoopControllerSlot selected) {
    final double moi = 0.1;
    final double length = 0.5;
    final double perRadianPerSecondSquared =
        voltsPerRadianPerSecondSquared(controller.motor(), moi);
    // SingleJointedArmSim's gravity is 3 g cos(angle) / (2 length) radians per second squared.
    final double kG = perRadianPerSecondSquared * 3 * kGravity / (2 * length);
    final double kV = 2 * Math.PI * voltsPerRadianPerSecond(controller.motor());
    final double kA = 2 * Math.PI * perRadianPerSecondSquared;
    final double kP = 10;
    final double maxVelocity = 0.5;
    final double maxAcceleration = 2;
    final double start = Degrees.of(-30).in(Rotations);
    final double goal = Degrees.of(90).in(Rotations);
    final SmartMotorControllerConfig cfg =
        baseConfig("arm " + controller + " " + selected)
            .withMomentOfInertia(KilogramSquareMeters.of(moi))
            .withStartingPosition(Rotations.of(start))
            .withTrapezoidalProfile(
                RotationsPerSecond.of(maxVelocity),
                RotationsPerSecondPerSecond.of(maxAcceleration));
    for (ClosedLoopControllerSlot slot : slots(controller).toList()) {
      final double scale = scale(slot, selected);
      cfg.withClosedLoopController(kP * scale, 0, 0, slot)
          .withFeedforward(new ArmFeedforward(0, kG * scale, kV * scale, kA * scale), slot);
    }
    final SmartMotorController smc = create(controller, cfg);
    try {
      final SingleJointedArmSim sim =
          new SingleJointedArmSim(
              controller.motor(),
              kRatio,
              moi,
              length,
              Degrees.of(-180).in(Radians),
              Degrees.of(180).in(Radians),
              true,
              Rotations.of(start).in(Radians));
      smc.setSimSupplier(new ArmSimSupplier(sim, smc));
      smc.setupSimulation();
      assertFollowsProfile(
          controller,
          smc,
          selected,
          () -> smc.setPosition(Rotations.of(goal)),
          () -> smc.getMechanismPosition().in(Rotations),
          new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration),
          start,
          goal);
    } finally {
      close(smc);
    }
  }

  /** An elevator under gravity, with an {@link ElevatorFeedforward}, whose gains are per meter. */
  @ParameterizedTest(name = "{0} {1}")
  @MethodSource("controllersAndSlots")
  void elevatorFollowsProfile(Controller controller, ClosedLoopControllerSlot selected) {
    final double mass = 10;
    final double drumRadius = 0.02;
    final DCMotor motor = controller.motor();
    // ElevatorSim: a = G Kt V / (R r m) - G^2 Kt v / (R r^2 m Kv) - g.
    final double kA = motor.R * drumRadius * mass / (kRatio * motor.Kt);
    final double kV = kRatio / (drumRadius * motor.Kv);
    final double kG = kA * kGravity;
    final double kP = 40;
    final double maxVelocity = 0.4;
    final double maxAcceleration = 1;
    final double start = 0.1;
    final double goal = 0.6;
    final SmartMotorControllerConfig cfg =
        baseConfig("elevator " + controller + " " + selected)
            .withMechanismCircumference(Meters.of(2 * Math.PI * drumRadius))
            .withMomentOfInertia(KilogramSquareMeters.of(mass * drumRadius * drumRadius))
            .withTrapezoidalProfile(
                MetersPerSecond.of(maxVelocity), MetersPerSecondPerSecond.of(maxAcceleration));
    cfg.withStartingPosition(cfg.convertToMechanism(Meters.of(start)));
    for (ClosedLoopControllerSlot slot : slots(controller).toList()) {
      final double scale = scale(slot, selected);
      cfg.withClosedLoopController(kP * scale, 0, 0, slot)
          .withFeedforward(new ElevatorFeedforward(0, kG * scale, kV * scale, kA * scale), slot);
    }
    final SmartMotorController smc = create(controller, cfg);
    try {
      final ElevatorSim sim =
          new ElevatorSim(motor, kRatio, mass, drumRadius, 0, 2, true, start, 0.0, 0.0);
      smc.setSimSupplier(new ElevatorSimSupplier(sim, smc));
      smc.setupSimulation();
      assertFollowsProfile(
          controller,
          smc,
          selected,
          () -> smc.setPosition(Meters.of(goal)),
          () -> smc.getMeasurementPosition().in(Meters),
          new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration),
          start,
          goal);
    } finally {
      close(smc);
    }
  }
}
