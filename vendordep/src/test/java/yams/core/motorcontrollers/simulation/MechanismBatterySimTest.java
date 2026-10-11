// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechs;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MilliOhms;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import java.util.UUID;
import java.util.function.Function;
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
import org.wpilib.simulation.RoboRioSim;
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
import yams.core.motorcontrollers.simulation.BatterySim;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;
import yams.helpers.TestWithScheduler;

/**
 * A mechanism's {@code simIterate()} must leave the shared {@link BatterySim} voltage in place. The
 * battery sees every mechanism on the robot, so an idle mechanism must not reset the simulated
 * battery to the voltage its own current draw alone would give.
 */
public class MechanismBatterySimTest {
  /** Current drawn by "the rest of the robot" while the mechanism under test sits idle. */
  private static final double OTHER_LOAD_AMPS = 100.0;

  private static int offset = 0;

  private static SmartMotorControllerConfig baseConfig() {
    return new SmartMotorControllerConfig()
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withZeroPower(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(false)
        .withControlMode(ControlMode.CLOSED_LOOP);
  }

  private static Runnable createArm(SmartMotorController smc) {
    return new Arm(
            new ArmConfig()
                .withLength(Inches.of(4))
                .withHardLimits(Degrees.of(-100), Degrees.of(200)),
            smc)
        ::simIterate;
  }

  private static Runnable createElevator(SmartMotorController smc) {
    return new Elevator(
            new ElevatorConfig()
                .withHardLimits(Meters.of(0), Meters.of(3))
                .withCarriageWeight(Pounds.of(16)),
            smc)
        ::simIterate;
  }

  private static Runnable createPivot(SmartMotorController smc) {
    return new Pivot(new PivotConfig().withHardLimits(Degrees.of(-100), Degrees.of(150)), smc)
        ::simIterate;
  }

  private static Runnable createFlyWheel(SmartMotorController smc) {
    return new FlyWheel(new FlyWheelConfig().withDiameter(Inches.of(4)), smc)::simIterate;
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
            (Function<SmartMotorController, Runnable>) MechanismBatterySimTest::createArm),
        Arguments.of(
            "Elevator",
            baseConfig()
                .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
                .withClosedLoopController(30, 0, 0)
                .withSoftLimits(Meters.of(0), Meters.of(5))
                .withFeedforward(new ElevatorFeedforward(0, 0, 0, 0))
                .withStartingPosition(Meters.of(1)),
            (Function<SmartMotorController, Runnable>) MechanismBatterySimTest::createElevator),
        Arguments.of(
            "Pivot",
            baseConfig()
                .withClosedLoopController(4, 0, 0)
                .withSoftLimits(Degrees.of(-100), Degrees.of(100))
                .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
                .withStartingPosition(Degrees.of(0))
                .withMomentOfInertia(Inches.of(4), Pounds.of(1)),
            (Function<SmartMotorController, Runnable>) MechanismBatterySimTest::createPivot),
        Arguments.of(
            "FlyWheel",
            baseConfig()
                .withZeroPower(MotorMode.COAST)
                .withClosedLoopController(1, 0, 0)
                .withFeedforward(new SimpleMotorFeedforward(0, 0, 0)),
            (Function<SmartMotorController, Runnable>) MechanismBatterySimTest::createFlyWheel));
  }

  private static void close(SmartMotorController smc, SparkMax spark) {
    SmartMotorControllerTestSubsystem subsystem =
        (SmartMotorControllerTestSubsystem)
            ((SmartMotorControllerConfig) smc.getConfig()).getSubsystem();
    SmartMotorControllerCommandRegistry.removeCommands(subsystem);
    CommandScheduler.getInstance().unregisterSubsystem(subsystem);
    subsystem.close();
    DeviceCreator.silence(smc);
    spark.close();
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("mechanisms")
  void simIterateKeepsTheSharedBatteryVoltage(
      String name,
      SmartMotorControllerConfig config,
      Function<SmartMotorController, Runnable> createMechanism) {
    offset += 1;
    SparkMax spark = DeviceCreator.createSparkMax();
    SmartMotorController smc =
        new SparkWrapper(
            spark,
            DCMotor.getNEO(1),
            config
                .withSubsystem(new SmartMotorControllerTestSubsystem())
                .withTelemetry("BatteryShared" + name + offset, TelemetryVerbosity.LOW));
    ((SmartMotorControllerTestSubsystem) config.getSubsystem()).smc = smc;
    Runnable simIterate = createMechanism.apply(smc);
    UUID restOfRobot = UUID.randomUUID();
    try {
      // The mechanism is idle, so the voltage without any other load is the resting voltage.
      simIterate.run();
      double restingVoltage = RoboRioSim.getVInVoltage();

      // Something else on the robot starts drawing a lot of current from the same battery.
      double sharedVoltage = BatterySim.calculateVoltage(restOfRobot, OTHER_LOAD_AMPS);
      assertTrue(
          sharedVoltage < restingVoltage - 1.0,
          "A 100 A load should sag the shared battery by well over a volt.");

      // The idle mechanism iterating must not wipe out the sag caused by the rest of the robot.
      simIterate.run();
      assertEquals(
          sharedVoltage,
          RoboRioSim.getVInVoltage(),
          0.5,
          name + ".simIterate() replaced the shared battery voltage with its own.");
    } finally {
      BatterySim.removeCurrent(restOfRobot);
      close(smc, spark);
    }
  }

  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
    TestWithScheduler.schedulerStart();
    TestWithScheduler.schedulerClear();
    BatterySim.reset();
    BatterySim.enableDischarge(18.0, Volts.of(12.0), MilliOhms.of(20));
    BatterySim.disableDischarge();
  }

  @AfterEach
  void endTest() {
    BatterySim.reset();
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
    TestWithScheduler.schedulerClear();
  }
}
