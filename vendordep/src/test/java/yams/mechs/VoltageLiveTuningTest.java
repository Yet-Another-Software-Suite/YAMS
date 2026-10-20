// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechs;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
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
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.thrifty.nova.Nova;
import java.util.Arrays;
import java.util.function.Function;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
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
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;
import yams.helpers.TestWithScheduler;

/**
 * Verifies the tuning voltage of a {@link SmartMotorController}'s "Live Tuning" command, whether the
 * closed loop runs on the motor controller or on the SystemCore.
 *
 * <ol>
 *   <li>Turning the voltage on stops the closed loop controller and starts at 0 V, with voltage/setpoint
 *       set back to 0. A voltage typed then drives the leader and its loosely coupled follower.
 *   <li>Turning it off starts the closed loop controller again, which takes both back to the setpoint.
 *   <li>Ending the Live Tuning command with the voltage on stops the voltage, turns it off in the
 *       Tuning table, and the closed loop controller holds the setpoint again.
 * </ol>
 */
public class VoltageLiveTuningTest {
  private static final MechanismGearing GEARING = new MechanismGearing(GearBox.fromReductionStages(3, 4, 5));
  /** Tuning voltage, low enough that the pivots stay well inside their hard limits. */
  private static final double TUNING_VOLTS = 2;
  /** How far the pivots must move under the tuning voltage, in degrees. */
  private static final double MOVED_DEGREES = 10;
  /** How close to the setpoint the pivots must come back, in degrees. */
  private static final double HELD_DEGREES = 3;

  private static int offset = 0;

  private static SmartMotorControllerConfig config(String label) {
    // The leader and follower each have a subsystem, which the command registry needs named apart.
    SmartMotorControllerTestSubsystem subsys = new SmartMotorControllerTestSubsystem();
    subsys.setName(label);
    return (SmartMotorControllerConfig) new SmartMotorControllerConfig()
        .withClosedLoopController(40, 0, 0)
        .withGearing(GEARING)
        .withZeroPower(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(false)
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0, 0.02))
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withMomentOfInertia(Inches.of(4), Pounds.of(1))
        .withStartingPosition(Degrees.of(0))
        .withSubsystem(subsys)
        .withTelemetry(label, TelemetryVerbosity.HIGH);
  }

  private static SmartMotorControllerConfig exponentialProfile(SmartMotorControllerConfig cfg) {
    return cfg.withExponentialProfile(Volts.of(12), DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90));
  }

  private static SmartMotorController sparkMax(SmartMotorControllerConfig cfg) {
    return new SparkWrapper(DeviceCreator.createSparkMax(), DCMotor.getNEO(1), cfg);
  }

  private static SmartMotorController sparkFlex(SmartMotorControllerConfig cfg) {
    return new SparkWrapper(DeviceCreator.createSparkFlex(), DCMotor.getNeoVortex(1), cfg);
  }

  private static SmartMotorController talonFX(SmartMotorControllerConfig cfg) {
    return new TalonFXWrapper(DeviceCreator.createTalonFX(), DCMotor.getKrakenX60(1), cfg);
  }

  private static SmartMotorController nova(SmartMotorControllerConfig cfg) {
    return new NovaWrapper(DeviceCreator.createNova(), DCMotor.getNEO(1), cfg);
  }

  private static Arguments pair(
      String name,
      Function<SmartMotorControllerConfig, SmartMotorController> leader,
      Function<SmartMotorControllerConfig, SmartMotorControllerConfig> leaderConfig,
      Function<SmartMotorControllerConfig, SmartMotorController> follower) {
    return Arguments.of(name, leader, leaderConfig, follower);
  }

  private static Stream<Arguments> leadersAndFollowers() {
    offset++;
    return Stream.of(
        // The closed loops run on the motor controllers.
        pair("SPARK Flex leading a SPARK MAX", VoltageLiveTuningTest::sparkFlex, cfg -> cfg, VoltageLiveTuningTest::sparkMax),
        pair("TalonFX leading a SPARK MAX", VoltageLiveTuningTest::talonFX, cfg -> cfg, VoltageLiveTuningTest::sparkMax),
        // The leader's closed loop runs on the SystemCore, and so does the Nova follower's.
        pair("SPARK MAX with an exponential profile leading a Nova", VoltageLiveTuningTest::sparkMax, VoltageLiveTuningTest::exponentialProfile,
            VoltageLiveTuningTest::nova));
  }

  /** A pivot on the motor controller, simulated and pumped by its test subsystem. */
  private static SmartMotorControllerTestSubsystem pivot(SmartMotorController smc) {
    SmartMotorControllerTestSubsystem subsys = subsystem(smc);
    subsys.setSMC(smc);
    Pivot pivot = new Pivot(new PivotConfig().withHardLimits(Degrees.of(-180), Degrees.of(180)), smc);
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
  @MethodSource("leadersAndFollowers")
  void voltageDrivesTheLeaderAndFollowerAndHandsBackToTheClosedLoop(
      String name,
      Function<SmartMotorControllerConfig, SmartMotorController> createLeader,
      Function<SmartMotorControllerConfig, SmartMotorControllerConfig> leaderConfig,
      Function<SmartMotorControllerConfig, SmartMotorController> createFollower)
      throws InterruptedException {
    String leaderName = "VoltageLeader(" + name + ")[" + offset + "]";
    // Each motor drives its own simulated pivot, so the follower only moves as it is driven itself.
    SmartMotorController follower = createFollower.apply(config("VoltageFollower(" + name + ")[" + offset + "]"));
    SmartMotorController leader = createLeader.apply(
        (SmartMotorControllerConfig) leaderConfig.apply(config(leaderName)).withLooselyCoupledFollowers(follower));
    try {
      SmartMotorControllerTestSubsystem leaderSubsys = pivot(leader);
      pivot(follower);

      // Hold both pivots at 0°, which also sets up the telemetry and registers Live Tuning.
      TestWithScheduler.schedule(leaderSubsys.setPositionSetpoint(Degrees.of(0)));
      TestWithScheduler.cycle(Seconds.of(0.5));
      Command liveTuning = SmartMotorControllerCommandRegistry.getCommand("Live Tuning", leaderSubsys).orElseThrow();
      NetworkTable tuning = NetworkTableInstance.getDefault().getTable("Tuning").getSubTable(leaderName);
      TestWithScheduler.schedule(liveTuning);
      TestWithScheduler.cycle(Seconds.of(0.1));

      // 1. Turning the voltage on starts at 0 V, with voltage/setpoint set back to 0 even if it was not.
      tuning.getEntry("voltage/setpoint").setDouble(TUNING_VOLTS);
      tuning.getEntry("voltage/enabled").setBoolean(true);
      TestWithScheduler.cycle(Seconds.of(0.5));
      assertEquals(0, tuning.getEntry("voltage/setpoint").getDouble(Double.NaN), name + ": voltage/setpoint set back to 0");
      assertEquals(0, simulatedDegrees(leader), HELD_DEGREES, name + ": leader not driven before a voltage is typed");
      assertEquals(0, simulatedDegrees(follower), HELD_DEGREES, name + ": follower not driven before a voltage is typed");

      // The voltage typed then drives both pivots forward, instead of the closed loop holding them at 0°.
      tuning.getEntry("voltage/setpoint").setDouble(TUNING_VOLTS);
      TestWithScheduler.cycle(Seconds.of(0.5));
      assertTrue(simulatedDegrees(leader) > MOVED_DEGREES, name + ": leader driven by the voltage, at " + simulatedDegrees(leader) + "°");
      assertTrue(simulatedDegrees(follower) > MOVED_DEGREES, name + ": follower driven by the voltage, at " + simulatedDegrees(follower) + "°");

      // 2. Turning the voltage off hands both back to the closed loop, which holds 0° again.
      tuning.getEntry("voltage/enabled").setBoolean(false);
      TestWithScheduler.cycle(Seconds.of(3));
      assertEquals(0, simulatedDegrees(leader), HELD_DEGREES, name + ": leader back at the setpoint");
      assertEquals(0, simulatedDegrees(follower), HELD_DEGREES, name + ": follower back at the setpoint");

      // 3. Ending Live Tuning with the voltage on stops it, and turns it off in the Tuning table.
      tuning.getEntry("voltage/enabled").setBoolean(true);
      TestWithScheduler.cycle(Seconds.of(0.1));
      tuning.getEntry("voltage/setpoint").setDouble(TUNING_VOLTS);
      TestWithScheduler.cycle(Seconds.of(0.5));
      assertTrue(simulatedDegrees(leader) > MOVED_DEGREES, name + ": leader driven by the voltage again, at " + simulatedDegrees(leader) + "°");
      liveTuning.cancel();
      TestWithScheduler.cycle(Seconds.of(3));
      assertFalse(tuning.getEntry("voltage/enabled").getBoolean(true), name + ": voltage still on in the Tuning table");
      assertEquals(0, simulatedDegrees(leader), HELD_DEGREES, name + ": leader held at the setpoint after Live Tuning ended");
      assertEquals(0, simulatedDegrees(follower), HELD_DEGREES, name + ": follower held at the setpoint after Live Tuning ended");
    } finally {
      close(leader);
      close(follower);
    }
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("leadersAndFollowers")
  void voltageSetpointIsHeldWithoutChanging(
      String name,
      Function<SmartMotorControllerConfig, SmartMotorController> createLeader,
      Function<SmartMotorControllerConfig, SmartMotorControllerConfig> leaderConfig,
      Function<SmartMotorControllerConfig, SmartMotorController> createFollower)
      throws InterruptedException {
    String leaderName = "HeldVoltageLeader(" + name + ")[" + offset + "]";
    SmartMotorController follower = createFollower.apply(config("HeldVoltageFollower(" + name + ")[" + offset + "]"));
    SmartMotorController leader = createLeader.apply(
        (SmartMotorControllerConfig) leaderConfig.apply(config(leaderName)).withLooselyCoupledFollowers(follower));
    try {
      SmartMotorControllerTestSubsystem leaderSubsys = pivot(leader);
      pivot(follower);

      TestWithScheduler.schedule(leaderSubsys.setPositionSetpoint(Degrees.of(0)));
      TestWithScheduler.cycle(Seconds.of(0.5));
      Command liveTuning = SmartMotorControllerCommandRegistry.getCommand("Live Tuning", leaderSubsys).orElseThrow();
      NetworkTable tuning = NetworkTableInstance.getDefault().getTable("Tuning").getSubTable(leaderName);
      TestWithScheduler.schedule(liveTuning);
      TestWithScheduler.cycle(Seconds.of(0.1));
      tuning.getEntry("voltage/enabled").setBoolean(true);
      TestWithScheduler.cycle(Seconds.of(0.1));

      // Type 1 V once, then leave it alone. The leader and follower must keep outputting 1 V.
      tuning.getEntry("voltage/setpoint").setDouble(1);
      // A TalonFX in simulation reports its output voltage a loop or two late.
      TestWithScheduler.cycle(Seconds.of(0.2));
      double[] leaderVolts = new double[20];
      double[] followerVolts = new double[20];
      for (int i = 0; i < 20; i++) {
        TestWithScheduler.cycle(Seconds.of(0.1));
        leaderVolts[i] = leader.getVoltage().in(Volts);
        followerVolts[i] = follower.getVoltage().in(Volts);
      }
      String samples = " (leader " + Arrays.toString(leaderVolts) + ", follower " + Arrays.toString(followerVolts) + ")";
      for (int i = 0; i < 20; i++) {
        double elapsed = (i + 1) * 0.1;
        assertEquals(1, leaderVolts[i], 0.1, name + ": leader output after " + elapsed + "s" + samples);
        assertEquals(1, followerVolts[i], 0.1, name + ": follower output after " + elapsed + "s" + samples);
      }
    } finally {
      close(leader);
      close(follower);
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
