// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechs;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.thrifty.nova.Nova;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.preferences.Preferences;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.Pivot;
import yams.commands2.telemetry.SmartMotorControllerCommandRegistry;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.PivotConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.NovaWrapper;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.motorcontrollers.remote.TalonFXSWrapper;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;
import yams.helpers.TestWithScheduler;

/**
 * Verifies that a simulated mechanism stops at its hard stops, as a real one does:
 *
 * <ol>
 *   <li>A pivot driven into a hard stop stays on it. Its DCMotorSim has no hard stops, so YAMS puts
 *       it back on the stop, and it must stop it there too, or the pivot flies past again.
 *   <li>A Talon keeps an encoder position it is set to. The Talon sets its position from the rotor
 *       it has, so the simulated rotor must move first.
 * </ol>
 */
public class HardStopTest {
  /** The pivot's hard stops. */
  private static final double LOWER_HARD_STOP_DEGREES = -30;
  private static final double UPPER_HARD_STOP_DEGREES = 60;

  private static int offset = 0;

  private static SmartMotorControllerConfig config(String label) {
    return (SmartMotorControllerConfig) new SmartMotorControllerConfig()
        .withClosedLoopController(4, 0, 0)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4, 5)))
        .withZeroPower(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(false)
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0, 0.02))
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withMomentOfInertia(Inches.of(4), Pounds.of(1))
        .withStartingPosition(Degrees.of(0))
        .withSubsystem(new SmartMotorControllerTestSubsystem())
        .withTelemetry(label, TelemetryVerbosity.HIGH);
  }

  private static SmartMotorController sparkMax(String label) {
    return new SparkWrapper(DeviceCreator.createSparkMax(), DCMotor.getNEO(1), config(label));
  }

  private static SmartMotorController sparkFlex(String label) {
    return new SparkWrapper(DeviceCreator.createSparkFlex(), DCMotor.getNeoVortex(1), config(label));
  }

  private static SmartMotorController talonFXS(String label) {
    return new TalonFXSWrapper(DeviceCreator.createTalonFXS(), DCMotor.getNEO(1), config(label));
  }

  private static SmartMotorController talonFX(String label) {
    return new TalonFXWrapper(DeviceCreator.createTalonFX(), DCMotor.getKrakenX60(1), config(label));
  }

  private static SmartMotorController nova(String label) {
    return new NovaWrapper(DeviceCreator.createNova(), DCMotor.getNEO(1), config(label));
  }

  private static Stream<Arguments> everyMotorController() {
    offset++;
    return Stream.of(
        Arguments.of("SPARK MAX", (Supplier<SmartMotorController>) () -> sparkMax("SparkMax(hardStop)[" + offset + "]")),
        Arguments.of("SPARK Flex", (Supplier<SmartMotorController>) () -> sparkFlex("SparkFlex(hardStop)[" + offset + "]")),
        Arguments.of("TalonFXS", (Supplier<SmartMotorController>) () -> talonFXS("TalonFXS(hardStop)[" + offset + "]")),
        Arguments.of("TalonFX", (Supplier<SmartMotorController>) () -> talonFX("TalonFX(hardStop)[" + offset + "]")),
        Arguments.of("Nova", (Supplier<SmartMotorController>) () -> nova("Nova(hardStop)[" + offset + "]")));
  }

  private static Stream<Arguments> talons() {
    offset++;
    return Stream.of(
        Arguments.of("TalonFXS", (Supplier<SmartMotorController>) () -> talonFXS("TalonFXS(setEncoder)[" + offset + "]")),
        Arguments.of("TalonFX", (Supplier<SmartMotorController>) () -> talonFX("TalonFX(setEncoder)[" + offset + "]")));
  }

  /** A pivot on the motor controller, simulated and pumped by its test subsystem. */
  private static SmartMotorControllerTestSubsystem pivot(SmartMotorController smc) {
    SmartMotorControllerTestSubsystem subsys = subsystem(smc);
    subsys.setSMC(smc);
    Pivot pivot = new Pivot(
        new PivotConfig().withHardLimits(Degrees.of(LOWER_HARD_STOP_DEGREES), Degrees.of(UPPER_HARD_STOP_DEGREES)), smc);
    // Pivot's constructor does not set up the simulation.
    smc.setupSimulation();
    subsys.mechSimPeriodic = pivot::simIterate;
    subsys.mechUpdateTelemetry = pivot::updateTelemetry;
    subsys.testRunning = true;
    return subsys;
  }

  private static SmartMotorControllerTestSubsystem subsystem(SmartMotorController smc) {
    return (SmartMotorControllerTestSubsystem) ((SmartMotorControllerConfig) smc.getConfig()).getSubsystem();
  }

  /** Where the simulated pivot really is, in degrees. */
  private static double simulatedDegrees(SmartMotorController smc) {
    return smc.getSimSupplier().orElseThrow().getMechanismPosition().in(Degrees);
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("everyMotorController")
  void pivotStaysOnItsHardStops(String name, Supplier<SmartMotorController> create)
      throws InterruptedException {
    SmartMotorController smc = create.get();
    try {
      SmartMotorControllerTestSubsystem subsys = pivot(smc);
      for (double dutyCycle : new double[] {0.5, -0.5}) {
        double stop = dutyCycle > 0 ? UPPER_HARD_STOP_DEGREES : LOWER_HARD_STOP_DEGREES;
        TestWithScheduler.schedule(subsys.setDutyCycle(dutyCycle));
        // Long enough to reach the stop from the other one.
        TestWithScheduler.cycle(Seconds.of(2));
        // Pushed against the stop, the pivot must stay on it, loop after loop.
        double[] furthestPast = {0};
        TestWithScheduler.cycle(Milliseconds.of(500), () -> {
          double past = dutyCycle > 0 ? simulatedDegrees(smc) - stop : stop - simulatedDegrees(smc);
          furthestPast[0] = Math.max(furthestPast[0], past);
        });
        assertTrue(furthestPast[0] < 1,
            name + " pivot went " + furthestPast[0] + "° past its hard stop at " + stop + "°");
        assertEquals(stop, smc.getMechanismPosition().in(Degrees), 1, name + " encoder at the hard stop at " + stop + "°");
      }
    } finally {
      close(smc);
    }
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("talons")
  void talonKeepsAnEncoderPositionItIsSet(String name, Supplier<SmartMotorController> create)
      throws InterruptedException {
    SmartMotorController smc = create.get();
    try {
      SmartMotorControllerTestSubsystem subsys = pivot(smc);
      TestWithScheduler.schedule(subsys.setDutyCycle(0.3));
      TestWithScheduler.cycle(Milliseconds.of(300));
      TestWithScheduler.schedule(subsys.setDutyCycle(0));
      TestWithScheduler.cycle(Milliseconds.of(500));
      double moved = simulatedDegrees(smc);
      assertTrue(moved > 5, name + " pivot did not move: " + moved + "°");
      smc.setEncoderPosition(Degrees.of(0));
      TestWithScheduler.cycle(Milliseconds.of(200));
      // The pivot is at 0°, and the Talon says so: it did not count the move to 0° twice.
      assertEquals(0, simulatedDegrees(smc), 1, name + " simulated pivot after setting the encoder to 0° at " + moved + "°");
      assertEquals(0, smc.getMechanismPosition().in(Degrees), 1, name + " encoder after setting it to 0° at " + moved + "°");
    } finally {
      close(smc);
    }
  }

  @Test
  void pivotSimulationStartsAtItsStartingPosition() {
    // Without setupSimulation(), which seeds the motor controller's own simulation: the pivot's
    // simulation must start where the motor controller does, or its hard stops are off by as much.
    SmartMotorController smc = new SparkWrapper(DeviceCreator.createSparkMax(), DCMotor.getNEO(1),
        (SmartMotorControllerConfig) config("SparkMax(start)[" + (++offset) + "]").withStartingPosition(Degrees.of(45)));
    try {
      subsystem(smc).setSMC(smc);
      new Pivot(new PivotConfig().withHardLimits(Degrees.of(LOWER_HARD_STOP_DEGREES), Degrees.of(UPPER_HARD_STOP_DEGREES)), smc);
      assertEquals(45, simulatedDegrees(smc), 1e-6, "simulated pivot at its starting position");
    } finally {
      close(smc);
    }
  }

  @Test
  void continuousPivotTurnsPastWhereItWraps() throws InterruptedException {
    // A pivot that wraps continuously, such as a turret on a slip ring, has no hard stops: its hard
    // limits are the range it wraps over, and it turns on past them.
    SmartMotorController smc = new SparkWrapper(DeviceCreator.createSparkMax(), DCMotor.getNEO(1),
        (SmartMotorControllerConfig) config("SparkMax(wrap)[" + (++offset) + "]")
            .withContinuousWrapping(Rotations.of(0), Rotations.of(1))
            .withStartingPosition(Rotations.of(0.9)));
    try {
      SmartMotorControllerTestSubsystem subsys = subsystem(smc);
      subsys.setSMC(smc);
      Pivot pivot = new Pivot(new PivotConfig().withHardLimits(Rotations.of(0), Rotations.of(1)), smc);
      subsys.mechSimPeriodic = pivot::simIterate;
      subsys.mechUpdateTelemetry = pivot::updateTelemetry;
      subsys.testRunning = true;
      TestWithScheduler.schedule(subsys.setDutyCycle(0.5));
      TestWithScheduler.cycle(Seconds.of(1));
      double rotations = smc.getSimSupplier().orElseThrow().getMechanismPosition().in(Rotations);
      assertTrue(rotations > 1.05, "continuous pivot stopped at " + rotations + " rotations, where it wraps");
    } finally {
      close(smc);
    }
  }

  private static void close(SmartMotorController smc) {
    SmartMotorControllerTestSubsystem subsys = subsystem(smc);
    SmartMotorControllerCommandRegistry.removeCommands(subsys);
    CommandScheduler.getInstance().unregisterSubsystem(subsys);
    subsys.close();
    smc.close();
    DeviceCreator.silence(smc);
    Object motor = smc.getMotorController();
    if (motor instanceof SparkMax spark) {
      spark.close();
    } else if (motor instanceof SparkFlex spark) {
      spark.close();
    } else if (motor instanceof TalonFXS talon) {
      talon.close();
    } else if (motor instanceof TalonFX talon) {
      talon.close();
    } else if (motor instanceof Nova nova) {
      nova.close();
    }
  }

  @BeforeEach
  void beforeTest() {
    MockHardwareExtension.beforeAll();
    TestWithScheduler.schedulerStart();
    TestWithScheduler.schedulerClear();
  }

  @AfterEach
  void afterTest() {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
    TestWithScheduler.schedulerClear();
  }
}
