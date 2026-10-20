// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Pounds;

import com.revrobotics.spark.SparkMax;
import java.util.function.BiConsumer;
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
import org.wpilib.preferences.Preferences;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.Arm;
import yams.commands2.mechanisms.Elevator;
import yams.commands2.mechanisms.FlyWheel;
import yams.commands2.mechanisms.Pivot;
import yams.commands2.telemetry.SmartMotorControllerCommandRegistry;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.mechanisms.config.ElevatorConfig;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.mechanisms.config.PivotConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;
import yams.helpers.TestWithScheduler;

/**
 * A mechanism with a telemetry name sets up its motor's telemetry itself, so it must also register
 * the motor's "Live Tuning" command when the motor's telemetry verbosity is HIGH, and not when it
 * is lower.
 */
public class MechanismLiveTuningTest {
  private static int offset = 0;

  private static SmartMotorControllerConfig baseConfig() {
    return new SmartMotorControllerConfig()
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withZeroPower(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(false)
        .withControlMode(ControlMode.CLOSED_LOOP);
  }

  private static Stream<Arguments> mechanisms() {
    return Stream.of(
        Arguments.of(
            "Arm",
            baseConfig()
                .withClosedLoopController(5, 0, 0)
                .withSoftLimits(Degrees.of(-100), Degrees.of(100))
                .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
                .withStartingPosition(Degrees.of(0)),
            (BiConsumer<String, SmartMotorController>)
                (name, smc) ->
                    new Arm(
                        new ArmConfig()
                            .withLength(Inches.of(4))
                            .withHardLimits(Degrees.of(-100), Degrees.of(200))
                            .withTelemetry(name, TelemetryVerbosity.HIGH),
                        smc)),
        Arguments.of(
            "Elevator",
            baseConfig()
                .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
                .withClosedLoopController(30, 0, 0)
                .withSoftLimits(Meters.of(0), Meters.of(2))
                .withFeedforward(new ElevatorFeedforward(0, 0, 0, 0))
                .withStartingPosition(Meters.of(0)),
            (BiConsumer<String, SmartMotorController>)
                (name, smc) ->
                    new Elevator(
                        new ElevatorConfig()
                            .withHardLimits(Meters.of(0), Meters.of(3))
                            .withCarriageWeight(Pounds.of(16))
                            .withTelemetry(name, TelemetryVerbosity.HIGH),
                        smc)),
        Arguments.of(
            "Pivot",
            baseConfig()
                .withClosedLoopController(4, 0, 0)
                .withSoftLimits(Degrees.of(-100), Degrees.of(100))
                .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
                .withStartingPosition(Degrees.of(0))
                .withMomentOfInertia(Inches.of(4), Pounds.of(1)),
            (BiConsumer<String, SmartMotorController>)
                (name, smc) ->
                    new Pivot(
                        new PivotConfig()
                            .withHardLimits(Degrees.of(-100), Degrees.of(150))
                            .withTelemetry(name, TelemetryVerbosity.HIGH),
                        smc)),
        Arguments.of(
            "FlyWheel",
            baseConfig()
                .withZeroPower(MotorMode.COAST)
                .withClosedLoopController(1, 0, 0)
                .withFeedforward(new SimpleMotorFeedforward(0, 0, 0)),
            (BiConsumer<String, SmartMotorController>)
                (name, smc) ->
                    new FlyWheel(
                        new FlyWheelConfig()
                            .withDiameter(Inches.of(4))
                            .withTelemetry(name, TelemetryVerbosity.HIGH),
                        smc)));
  }

  private static Stream<Arguments> mechanismsAndVerbosities() {
    // Each verbosity gets its own configs, since a config can only be given one subsystem.
    return Stream.of(TelemetryVerbosity.HIGH, TelemetryVerbosity.LOW)
        .flatMap(
            verbosity ->
                mechanisms()
                    .map(
                        args ->
                            Arguments.of(args.get()[0], args.get()[1], args.get()[2], verbosity)));
  }

  @ParameterizedTest(name = "{0} {3}")
  @MethodSource("mechanismsAndVerbosities")
  void mechanismTelemetryRegistersLiveTuning(
      String name,
      SmartMotorControllerConfig config,
      BiConsumer<String, SmartMotorController> createMechanism,
      TelemetryVerbosity verbosity) {
    offset += 1;
    SmartMotorControllerTestSubsystem subsystem = new SmartMotorControllerTestSubsystem();
    SparkMax spark = DeviceCreator.createSparkMax();
    SmartMotorController smc =
        new SparkWrapper(
            spark,
            DCMotor.getNEO(1),
            config
                .withSubsystem(subsystem)
                .withTelemetry("LiveTuningMotor" + name + offset, verbosity));
    subsystem.smc = smc;
    try {
      createMechanism.accept("LiveTuning" + name + offset, smc);
      assertEquals(
          verbosity == TelemetryVerbosity.HIGH,
          SmartMotorControllerCommandRegistry.commandExists("Live Tuning", subsystem),
          name + " with " + verbosity + " motor telemetry registered the wrong Live Tuning state.");
    } finally {
      SmartMotorControllerCommandRegistry.removeCommands(subsystem);
      CommandScheduler.getInstance().unregisterSubsystem(subsystem);
      subsystem.close();
      DeviceCreator.silence(smc);
      spark.close();
    }
  }

  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
    TestWithScheduler.schedulerStart();
    TestWithScheduler.schedulerClear();
  }

  @AfterEach
  void endTest() {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
    TestWithScheduler.schedulerClear();
  }
}
