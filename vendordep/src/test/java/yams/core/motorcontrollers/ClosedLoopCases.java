// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.thrifty.nova.Nova;
import java.util.function.BiFunction;
import java.util.function.DoubleConsumer;
import java.util.stream.Stream;
import org.junit.jupiter.params.provider.Arguments;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.system.DCMotor;
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
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.PeriodicScheduler;
import yams.helpers.SmartMotorControllerTestSubsystem;

/**
 * Every motor controller, for tests that give each one the same closed loop gains in YAMS units and
 * expect the same reaction. The SPARK and Talons close their loops onboard; the Nova's loop runs on
 * the SystemCore.
 */
final class ClosedLoopCases {
  private ClosedLoopCases() {}

  /** 12 motor rotations per mechanism rotation. */
  static final double kRatio = 12;

  static final MechanismGearing kGearing = new MechanismGearing(GearBox.fromReductionStages(3, 4));

  /** Period of the robot loop and of the simulation, in seconds. */
  static final double kPeriod = 0.020;

  /**
   * A motor controller to test.
   *
   * @param name Name of the motor controller.
   * @param motor Motor it drives.
   * @param slots Number of closed loop slots it has.
   * @param realTime Whether its simulation runs in real time. Phoenix simulates a Talon on its own
   *     clock, which does not stop with WPILib's simulated time.
   * @param create Creates the motor controller.
   */
  record Controller(
      String name,
      DCMotor motor,
      int slots,
      boolean realTime,
      BiFunction<DCMotor, SmartMotorControllerConfig, SmartMotorController> create) {
    @Override
    public String toString() {
      return name;
    }

    /** Whether this is a SPARK. */
    boolean isSpark() {
      return name.startsWith("Spark");
    }

    /** The closed loop slots it has. */
    Stream<ClosedLoopControllerSlot> slotStream() {
      return Stream.of(ClosedLoopControllerSlot.values()).limit(slots);
    }
  }

  static final Controller[] kControllers = {
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

  /** Every motor controller. */
  static Stream<Controller> controllers() {
    return Stream.of(kControllers);
  }

  /** Every motor controller with each of its closed loop slots. */
  static Stream<Arguments> controllersAndSlots() {
    return controllers()
        .flatMap(controller -> controller.slotStream().map(slot -> Arguments.of(controller, slot)));
  }

  /** A closed loop config with no gains, for the named test. */
  static SmartMotorControllerConfig baseConfig(String name) {
    return new yams.commands2.config.SmartMotorControllerConfig()
        .withSubsystem(new SmartMotorControllerTestSubsystem())
        .withGearing(kGearing)
        .withStatorCurrentLimit(Amps.of(80))
        .withZeroPower(MotorMode.BRAKE)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withTelemetry(name, TelemetryVerbosity.LOW);
  }

  static SmartMotorController create(Controller controller, SmartMotorControllerConfig cfg) {
    final SmartMotorController smc = controller.create().apply(controller.motor(), cfg);
    ((SmartMotorControllerTestSubsystem)
            ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem())
        .setSMC(smc);
    return smc;
  }

  static void close(SmartMotorController smc) {
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

  /**
   * Run the simulation, commanding the setpoint each loop as a robot does.
   *
   * @param controller Motor controller under test.
   * @param smc The motor controller.
   * @param seconds How long to run.
   * @param command Commands the setpoint; run before the first loop and in every loop.
   * @param afterLoop Given the time since the first command, in seconds, after each loop.
   */
  static void run(
      Controller controller,
      SmartMotorController smc,
      double seconds,
      Runnable command,
      DoubleConsumer afterLoop) {
    final double[] time = {0};
    try (PeriodicScheduler scheduler = new PeriodicScheduler()) {
      command.run();
      final long startNanos = System.nanoTime();
      scheduler.addPeriodic(
          () -> {
            smc.simIterate();
            time[0] += kPeriod;
            command.run();
            afterLoop.accept(time[0]);
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
      scheduler.runFor(Seconds.of(seconds));
    }
  }

  /**
   * The average voltage the motor controller applies from {@code from} to {@code to} seconds after
   * the first command. Start half a second in: with many simulated Talons on the CAN bus, a Talon can
   * take a few hundred milliseconds to apply its first output.
   */
  static double averageVoltage(
      Controller controller,
      SmartMotorController smc,
      double from,
      double to,
      Runnable command) {
    final double[] sum = {0, 0};
    run(
        controller,
        smc,
        to,
        command,
        time -> {
          if (time >= from - 1e-9) {
            sum[0] += smc.getVoltage().in(Volts);
            sum[1]++;
          }
        });
    return sum[0] / sum[1];
  }
}
