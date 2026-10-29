// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechs;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.thrifty.nova.Nova;
import java.util.function.Function;
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
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.Pivot;
import yams.commands2.telemetry.SmartMotorControllerCommandRegistry;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.math.LQRConfig;
import yams.core.math.LQRController;
import yams.core.mechanisms.config.PivotConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.SmartMotorController.ClosedLoopControllerSlot;
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
 * Verifies that a loosely coupled follower follows its leader, whether its closed loop runs on the
 * motor controller or on the SystemCore: a Nova's always does, as does a SPARK's with an exponential
 * profile and any motor controller's with an LQR.
 *
 * <ol>
 *   <li>The follower goes to the leader's setpoint.
 *   <li>When the leader's closed loop is stopped, such as to drive it at a duty cycle, the follower's
 *       is stopped too, and the follower is not driven, rather than driving on to the last setpoint.
 *   <li>When the leader's closed loop is started again, the follower's is too.
 * </ol>
 */
public class LooselyCoupledFollowerTest {
  private static final MechanismGearing GEARING = new MechanismGearing(GearBox.fromReductionStages(3, 4, 5));
  private static final double SETPOINT_DEGREES = 45;

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

  private static SmartMotorControllerConfig lqr(SmartMotorControllerConfig cfg) {
    return cfg.withClosedLoopController(new LQRController(
        new LQRConfig(DCMotor.getNEO(1), GEARING, KilogramSquareMeters.of(0.02))
            .withArm(Radians.of(0.01), RadiansPerSecond.of(0.5), Radians.of(0.05), RadiansPerSecond.of(0.5), Radians.of(0.01))
            .withControlEffort(Volts.of(12))
            .withMaxVoltage(Volts.of(12))));
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

  private static SmartMotorController talonFXS(SmartMotorControllerConfig cfg) {
    return new TalonFXSWrapper(DeviceCreator.createTalonFXS(), DCMotor.getNEO(1), cfg);
  }

  private static SmartMotorController nova(SmartMotorControllerConfig cfg) {
    return new NovaWrapper(DeviceCreator.createNova(), DCMotor.getNEO(1), cfg);
  }

  private static Arguments pair(
      String name,
      Function<SmartMotorControllerConfig, SmartMotorController> leader,
      Function<SmartMotorControllerConfig, SmartMotorController> follower,
      Function<SmartMotorControllerConfig, SmartMotorControllerConfig> followerConfig) {
    return Arguments.of(name, leader, follower, followerConfig);
  }

  private static Stream<Arguments> leadersAndFollowers() {
    offset++;
    return Stream.of(
        // The follower's closed loop runs on the motor controller.
        pair("SPARK MAX following a SPARK Flex", LooselyCoupledFollowerTest::sparkFlex, LooselyCoupledFollowerTest::sparkMax, cfg -> cfg),
        // The follower's closed loop runs on the SystemCore.
        pair("Nova following a SPARK MAX", LooselyCoupledFollowerTest::sparkMax, LooselyCoupledFollowerTest::nova, cfg -> cfg),
        pair("SPARK MAX with an exponential profile following a TalonFX", LooselyCoupledFollowerTest::talonFX, LooselyCoupledFollowerTest::sparkMax,
            LooselyCoupledFollowerTest::exponentialProfile),
        pair("SPARK MAX with an LQR following a Nova", LooselyCoupledFollowerTest::nova, LooselyCoupledFollowerTest::sparkMax, LooselyCoupledFollowerTest::lqr));
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
  void followerFollowsAndStopsWithItsLeader(
      String name,
      Function<SmartMotorControllerConfig, SmartMotorController> createLeader,
      Function<SmartMotorControllerConfig, SmartMotorController> createFollower,
      Function<SmartMotorControllerConfig, SmartMotorControllerConfig> followerConfig)
      throws InterruptedException {
    // Each motor drives its own simulated pivot, so the follower only moves as its own closed loop drives it.
    SmartMotorController follower = createFollower.apply(followerConfig.apply(config("Follower(" + name + ")[" + offset + "]")));
    SmartMotorController leader = createLeader.apply(
        (SmartMotorControllerConfig) config("Leader(" + name + ")[" + offset + "]").withLooselyCoupledFollowers(follower));
    try {
      SmartMotorControllerTestSubsystem leaderSubsys = pivot(leader);
      pivot(follower);

      // 1. The follower goes to the leader's setpoint.
      TestWithScheduler.schedule(leaderSubsys.setPositionSetpoint(Degrees.of(SETPOINT_DEGREES)));
      TestWithScheduler.cycle(Seconds.of(3));
      assertEquals(SETPOINT_DEGREES, simulatedDegrees(follower), 3, name + ": follower at the leader's setpoint");

      // 2. Back to 0°, then the leader is stopped partway, as a duty cycle command does.
      TestWithScheduler.schedule(leaderSubsys.setPositionSetpoint(Degrees.of(0)));
      for (int loop = 0; loop < 150 && simulatedDegrees(follower) > SETPOINT_DEGREES / 2; loop++) {
        TestWithScheduler.cycle(Milliseconds.of(20));
      }
      double stoppedAt = simulatedDegrees(follower);
      assertTrue(stoppedAt > 5, name + ": follower was not still on its way to 0°, at " + stoppedAt + "°");
      TestWithScheduler.schedule(leaderSubsys.setDutyCycle(0));
      // The follower is stopped as well, loop after loop, rather than its closed loop driving it on to
      // 0°. Its position cannot show this, as a fast follower coasts most of the way there.
      TestWithScheduler.cycle(Milliseconds.of(40));
      double[] largestDutyCycle = {0};
      TestWithScheduler.cycle(Seconds.of(1),
          () -> largestDutyCycle[0] = Math.max(largestDutyCycle[0], Math.abs(follower.getDutyCycle())));
      assertTrue(largestDutyCycle[0] < 0.01, name + ": follower driven at a duty cycle of up to "
          + largestDutyCycle[0] + " after its leader stopped, at " + stoppedAt + "°");

      // 3. The leader's closed loop is started again, and the follower follows again.
      TestWithScheduler.schedule(leaderSubsys.setPositionSetpoint(Degrees.of(SETPOINT_DEGREES)));
      TestWithScheduler.cycle(Seconds.of(3));
      assertEquals(SETPOINT_DEGREES, simulatedDegrees(follower), 3, name + ": follower back at the leader's setpoint");
    } finally {
      close(leader);
      close(follower);
    }
  }

  private static Stream<Arguments> leaders() {
    offset++;
    return Stream.of(
        Arguments.of("SPARK MAX", (Function<SmartMotorControllerConfig, SmartMotorController>) LooselyCoupledFollowerTest::sparkMax),
        Arguments.of("SPARK Flex", (Function<SmartMotorControllerConfig, SmartMotorController>) LooselyCoupledFollowerTest::sparkFlex),
        Arguments.of("TalonFX", (Function<SmartMotorControllerConfig, SmartMotorController>) LooselyCoupledFollowerTest::talonFX),
        Arguments.of("TalonFXS", (Function<SmartMotorControllerConfig, SmartMotorController>) LooselyCoupledFollowerTest::talonFXS),
        Arguments.of("Nova", (Function<SmartMotorControllerConfig, SmartMotorController>) LooselyCoupledFollowerTest::nova));
  }

  /** A follower switches closed loop slot when its leader does, whatever the leader's motor controller. */
  @ParameterizedTest(name = "{0}")
  @MethodSource("leaders")
  void followerSwitchesSlotWithItsLeader(String name, Function<SmartMotorControllerConfig, SmartMotorController> createLeader) {
    SmartMotorController follower = sparkMax(config("SlotFollower(" + name + ")[" + offset + "]"));
    SmartMotorController leader = createLeader.apply(
        (SmartMotorControllerConfig) config("SlotLeader(" + name + ")[" + offset + "]").withLooselyCoupledFollowers(follower));
    try {
      for (ClosedLoopControllerSlot slot : new ClosedLoopControllerSlot[] {ClosedLoopControllerSlot.SLOT_1, ClosedLoopControllerSlot.SLOT_2, ClosedLoopControllerSlot.SLOT_0}) {
        leader.setClosedLoopSlot(slot);
        assertSame(slot, leader.getClosedLoopControllerSlot(), name + ": leader on " + slot);
        assertSame(slot, follower.getClosedLoopControllerSlot(), name + ": follower on " + slot);
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
    } else if (motor instanceof TalonFXS talon) {
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
