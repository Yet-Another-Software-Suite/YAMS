// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechs;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import com.revrobotics.spark.A301;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.thrifty.nova.Nova;
import java.util.UUID;
import java.util.function.Function;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.system.DCMotor;
import org.wpilib.preferences.Preferences;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.util.Pair;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.Pivot;
import yams.commands2.telemetry.SmartMotorControllerCommandRegistry;
import yams.core.exceptions.SmartMotorControllerConfigurationException;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.PivotConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.YMotors;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.A301Wrapper;
import yams.core.motorcontrollers.local.NovaWrapper;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.motorcontrollers.simulation.BatterySim;
import yams.core.motorcontrollers.simulation.FollowerSimSupplier;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;
import yams.helpers.TestWithScheduler;

/**
 * How followers are simulated, so a mechanism has the torque of every motor on it and the battery
 * counts each motor's current once.
 *
 * <ul>
 *   <li>An A301 is followed by other {@link A301Wrapper}s: its motor model has every A301, and a
 *       follower reads its simulation.
 *   <li>A mechanism simulates its motor controller's loosely coupled followers with {@link
 *       SmartMotorController#getMechanismDCMotor()}, and each follower reads its simulation.
 *   <li>Without a mechanism, or with a mechanism of its own, a loosely coupled follower simulates its
 *       own load and current.
 * </ul>
 */
public class FollowerSimulationTest {
  private static final MechanismGearing GEARING =
      new MechanismGearing(GearBox.fromReductionStages(3, 4, 5));
  private static final double SETPOINT_DEGREES = 45;

  private static int offset = 0;

  /** A closed loop config, with a subsystem named apart for the command registry. */
  private static SmartMotorControllerConfig config(String label) {
    SmartMotorControllerTestSubsystem subsys = new SmartMotorControllerTestSubsystem();
    subsys.setName(label + "[" + offset + "]");
    return (SmartMotorControllerConfig) new SmartMotorControllerConfig()
        .withClosedLoopController(40, 0, 0)
        .withGearing(GEARING)
        .withZeroPower(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(false)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withMomentOfInertia(Inches.of(4), Pounds.of(1))
        .withStartingPosition(Degrees.of(0))
        .withSubsystem(subsys)
        .withTelemetry(label + "[" + offset + "]", TelemetryVerbosity.HIGH);
  }

  /** An open loop A301 config, so a test commands the A301 itself. */
  private static SmartMotorControllerConfig a301Config(String label) {
    SmartMotorControllerTestSubsystem subsys = new SmartMotorControllerTestSubsystem();
    subsys.setName(label + "[" + offset + "]");
    return (SmartMotorControllerConfig) new SmartMotorControllerConfig()
        .withSubsystem(subsys)
        // The A301's 64:1 gearbox.
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(64)))
        .withControlMode(ControlMode.OPEN_LOOP)
        .withMomentOfInertia(Inches.of(4), Pounds.of(1))
        .withStartingPosition(Degrees.of(0))
        .withTelemetry(label + "[" + offset + "]", TelemetryVerbosity.LOW);
  }

  private static SmartMotorController sparkMax(SmartMotorControllerConfig cfg) {
    return new SparkWrapper(DeviceCreator.createSparkMax(), DCMotor.getNEO(1), cfg);
  }

  private static SmartMotorController talonFX(SmartMotorControllerConfig cfg) {
    return new TalonFXWrapper(DeviceCreator.createTalonFX(), DCMotor.getKrakenX60(1), cfg);
  }

  private static SmartMotorController nova(SmartMotorControllerConfig cfg) {
    return new NovaWrapper(DeviceCreator.createNova(), DCMotor.getNEO(1), DeviceCreator.withNovaProfile(cfg, false));
  }

  private static SmartMotorController looseA301(SmartMotorControllerConfig cfg) {
    return new A301Wrapper(DeviceCreator.createA301(), DeviceCreator.forA301(cfg));
  }

  private static A301Wrapper a301(SmartMotorControllerConfig cfg) {
    return new A301Wrapper(DeviceCreator.createA301(), cfg);
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

  /** Where the simulated mechanism really is, in degrees. */
  private static double simulatedDegrees(SmartMotorController smc) {
    return smc.getSimSupplier().orElseThrow().getMechanismPosition().in(Degrees);
  }

  /** Whether the battery counts the motor controller's current. */
  private static boolean batteryCounts(SmartMotorController smc) {
    UUID probe = UUID.randomUUID();
    double withoutIt = BatterySim.calculateVoltage(probe, 0);
    double withIt = BatterySim.calculateVoltage(smc.m_batterySimUUID, 100);
    BatterySim.removeCurrent(probe);
    BatterySim.removeCurrent(smc.m_batterySimUUID);
    return withIt < withoutIt - 1e-6;
  }

  private static void close(SmartMotorController smc) {
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
    } else if (motor instanceof TalonFX talon) {
      talon.close();
    } else if (motor instanceof Nova nova) {
      nova.close();
    } else if (motor instanceof A301 a301) {
      a301.close();
    }
  }

  @Test
  void a301FollowersAreInTheLeadersMotorAndSimulation() {
    // Two A301s on one mechanism, the follower facing the other way.
    A301Wrapper follower = a301(a301Config("A301Follower"));
    A301Wrapper leader = a301((SmartMotorControllerConfig) a301Config("A301Leader")
        .withFollowers(Pair.of(follower, true)));
    try {
      DCMotor one = YMotors.getA301(1);
      assertEquals(2 * one.stallTorque, leader.getDCMotor().stallTorque, 1e-9);
      assertEquals(one.freeSpeed, leader.getDCMotor().freeSpeed, 1e-9);
      assertTrue(follower.getSimSupplier().orElseThrow() instanceof FollowerSimSupplier);

      // The follower is sent the leader's voltage, inverted.
      leader.setVoltage(Volts.of(6));
      assertEquals(-6, follower.getVoltage().in(Volts), 1e-6);

      // The follower reads the leader's simulation, which the leader iterates.
      for (int i = 0; i < 25; i++) {
        leader.simIterate();
      }
      assertTrue(leader.getMechanismPosition().in(Rotations) > 0.001);
      assertEquals(leader.getMechanismPosition().in(Rotations),
          follower.getMechanismPosition().in(Rotations), 1e-9);
      // Each A301 draws half of the current, and the battery counts it once, in the leader's.
      assertEquals(leader.getStatorCurrent().in(Amps) / 2, follower.getStatorCurrent().in(Amps), 1e-9);
      assertFalse(batteryCounts(follower));
    } finally {
      close(leader);
    }
    // Without its leader, the follower simulates itself again.
    assertFalse(follower.getSimSupplier().orElseThrow() instanceof FollowerSimSupplier);
    assertTrue(batteryCounts(follower));
    close(follower);
  }

  @Test
  void onlyAnA301WrapperCanFollowAnA301() {
    A301Wrapper other = a301(a301Config("Other"));
    A301Wrapper leader = a301((SmartMotorControllerConfig) a301Config("First")
        .withFollowers(Pair.of(other, false)));
    try {
      // Not an A301Wrapper.
      assertThrows(SmartMotorControllerConfigurationException.class, () -> a301(
          (SmartMotorControllerConfig) a301Config("Bare").withFollowers(Pair.of(DeviceCreator.createA301(), false))));
      // An A301 cannot follow two A301s.
      assertThrows(SmartMotorControllerConfigurationException.class, () -> a301(
          (SmartMotorControllerConfig) a301Config("Second").withFollowers(Pair.of(other, false))));
    } finally {
      close(leader);
      close(other);
    }
  }

  @Test
  void mechanismSimulatesItsLooseFollowers() throws InterruptedException {
    SmartMotorController follower = sparkMax(config("MechanismFollower"));
    SmartMotorController leader = sparkMax(
        (SmartMotorControllerConfig) config("MechanismLeader").withLooselyCoupledFollowers(follower));
    try {
      // Two NEOs on the pivot.
      DCMotor two = DCMotor.getNEO(2);
      DCMotor mechanism = leader.getMechanismDCMotor();
      assertEquals(two.stallTorque, mechanism.stallTorque, 1e-9);
      assertEquals(two.stallCurrent, mechanism.stallCurrent, 1e-9);
      assertEquals(two.freeCurrent, mechanism.freeCurrent, 1e-9);
      assertEquals(two.freeSpeed, mechanism.freeSpeed, 1e-9);
      // The leader's own motor model is still its own.
      assertEquals(DCMotor.getNEO(1).stallTorque, leader.getDCMotor().stallTorque, 1e-9);

      SmartMotorControllerTestSubsystem leaderSubsys = pivot(leader);
      assertTrue(follower.getSimSupplier().orElseThrow() instanceof FollowerSimSupplier);
      // The follower is iterated on its own, as robot code does.
      SmartMotorControllerTestSubsystem followerSubsys = subsystem(follower);
      followerSubsys.setSMC(follower);
      followerSubsys.testRunning = true;

      TestWithScheduler.schedule(leaderSubsys.setPositionSetpoint(Degrees.of(SETPOINT_DEGREES)));
      TestWithScheduler.cycle(Seconds.of(3));
      assertEquals(SETPOINT_DEGREES, simulatedDegrees(leader), 3, "pivot at the setpoint");
      // The follower's encoder follows the pivot.
      assertEquals(simulatedDegrees(leader), follower.getMechanismPosition().in(Degrees), 1,
          "follower's encoder on the pivot");
      // The follower's current is in the pivot's, so the battery counts it once.
      assertFalse(batteryCounts(follower));
    } finally {
      close(leader);
      close(follower);
    }
  }

  private static Stream<Arguments> followers() {
    return Stream.of(
        Arguments.of("SPARK MAX", (Function<SmartMotorControllerConfig, SmartMotorController>) FollowerSimulationTest::sparkMax),
        // A Talon's simIterate gives its sim supplier its voltage and steps it every call.
        Arguments.of("TalonFX", (Function<SmartMotorControllerConfig, SmartMotorController>) FollowerSimulationTest::talonFX),
        Arguments.of("Nova", (Function<SmartMotorControllerConfig, SmartMotorController>) FollowerSimulationTest::nova),
        Arguments.of("A301", (Function<SmartMotorControllerConfig, SmartMotorController>) FollowerSimulationTest::looseA301));
  }

  /**
   * A loosely coupled follower's simIterate does not step the mechanism's simulation, give it its
   * own output, or draw current, however often it is called: only the mechanism steps it, once a
   * loop, with the leader's output.
   */
  @ParameterizedTest(name = "{0}")
  @MethodSource("followers")
  void followerSimIterateDoesNotStepTheMechanism(
      String name, Function<SmartMotorControllerConfig, SmartMotorController> createFollower)
      throws InterruptedException {
    SmartMotorController follower = createFollower.apply(config("StepFollower(" + name + ")"));
    SmartMotorController leader = sparkMax(
        (SmartMotorControllerConfig) config("StepLeader(" + name + ")").withLooselyCoupledFollowers(follower));
    try {
      SmartMotorControllerTestSubsystem leaderSubsys = pivot(leader);
      // The pivot moving, its follower commanded to the same setpoint.
      TestWithScheduler.schedule(leaderSubsys.setPositionSetpoint(Degrees.of(SETPOINT_DEGREES)));
      TestWithScheduler.cycle(Seconds.of(0.2));
      var mechanism = leader.getSimSupplier().orElseThrow();
      double position = mechanism.getMechanismPosition().in(Degrees);
      double velocity = mechanism.getMechanismVelocity().in(Degrees.per(Seconds));
      assertTrue(Math.abs(velocity) > 1, name + ": pivot moving");

      for (int i = 0; i < 50; i++) {
        follower.simIterate();
      }
      assertEquals(position, mechanism.getMechanismPosition().in(Degrees), 1e-9,
          name + ": the follower stepped the pivot");
      assertEquals(velocity, mechanism.getMechanismVelocity().in(Degrees.per(Seconds)), 1e-9,
          name + ": the follower stepped the pivot");
      assertFalse(batteryCounts(follower), name + ": battery counted the follower's current");
    } finally {
      close(leader);
      close(follower);
    }
  }

  @Test
  void looseFollowerWithoutAMechanismSimulatesItself() {
    // Two motors on their own shafts, such as two flywheels: no mechanism simulates them together.
    SmartMotorController follower = sparkMax(config("ShaftFollower"));
    SmartMotorController leader = sparkMax(
        (SmartMotorControllerConfig) config("ShaftLeader").withLooselyCoupledFollowers(follower));
    try {
      assertFalse(follower.getSimSupplier().orElseThrow() instanceof FollowerSimSupplier);
      assertTrue(batteryCounts(follower));
    } finally {
      close(leader);
      close(follower);
    }
  }

  @Test
  void looseFollowerWithItsOwnMechanismSimulatesItself() {
    SmartMotorController follower = sparkMax(config("OwnFollower"));
    SmartMotorController leader = sparkMax(
        (SmartMotorControllerConfig) config("OwnLeader").withLooselyCoupledFollowers(follower));
    try {
      pivot(leader);
      // The follower's own pivot replaces the leader's simulation, and it draws its own current.
      pivot(follower);
      assertFalse(follower.getSimSupplier().orElseThrow() instanceof FollowerSimSupplier);
      assertTrue(batteryCounts(follower));
    } finally {
      close(leader);
      close(follower);
    }
  }

  @Test
  void mechanismMotorGivesAFollowersTorqueAtTheLeadersRotor() {
    // The follower is geared 30:1 and the leader 60:1, so the follower adds half a NEO at the leader's rotor.
    SmartMotorController follower = sparkMax((SmartMotorControllerConfig) config("GearedFollower")
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(30))));
    SmartMotorController leader = sparkMax((SmartMotorControllerConfig) config("GearedLeader")
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(60)))
        .withLooselyCoupledFollowers(follower));
    try {
      DCMotor neo = DCMotor.getNEO(1);
      assertEquals(1.5 * neo.stallTorque, leader.getMechanismDCMotor().stallTorque, 1e-9);
      assertEquals(2 * neo.stallCurrent, leader.getMechanismDCMotor().stallCurrent, 1e-9);
      assertEquals(neo.freeSpeed, leader.getMechanismDCMotor().freeSpeed, 1e-9);
    } finally {
      close(leader);
      close(follower);
    }
  }

  @Test
  void a301WithFollowersInAMechanism() {
    A301Wrapper follower = a301(a301Config("PivotA301Follower"));
    A301Wrapper leader = a301((SmartMotorControllerConfig) a301Config("PivotA301Leader")
        .withFollowers(Pair.of(follower, false)));
    try {
      pivot(leader);
      // The pivot has both A301s, and the follower reads the pivot's simulation.
      assertEquals(YMotors.getA301(2).stallTorque, leader.getMechanismDCMotor().stallTorque, 1e-9);
      assertTrue(follower.getSimSupplier().orElseThrow() instanceof FollowerSimSupplier);
      leader.setVoltage(Volts.of(6));
      for (int i = 0; i < 25; i++) {
        leader.getSimSupplier().orElseThrow().updateSimState();
        leader.getSimSupplier().orElseThrow().starveUpdateSim();
      }
      assertTrue(Math.abs(simulatedDegrees(leader)) > 0.1);
      assertEquals(simulatedDegrees(leader), follower.getMechanismPosition().in(Degrees), 1e-6);
      assertFalse(batteryCounts(follower));
    } finally {
      close(leader);
      close(follower);
    }
  }

  @BeforeEach
  void beforeTest() {
    offset++;
    MockHardwareExtension.beforeAll();
    RoboRioSim.setVInVoltage(12.0);
    TestWithScheduler.schedulerStart();
    TestWithScheduler.schedulerClear();
    BatterySim.reset();
  }

  @AfterEach
  void afterTest() {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
    TestWithScheduler.schedulerClear();
    BatterySim.reset();
  }
}
