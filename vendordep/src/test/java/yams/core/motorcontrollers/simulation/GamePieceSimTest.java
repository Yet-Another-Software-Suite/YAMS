// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers.simulation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.system.DCMotor;
import org.wpilib.preferences.Preferences;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.simulation.SingleJointedArmSim;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.Arm;
import yams.commands2.mechanisms.Elevator;
import yams.commands2.mechanisms.Pivot;
import yams.commands2.telemetry.SmartMotorControllerCommandRegistry;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.mechanisms.config.ElevatorConfig;
import yams.core.mechanisms.config.PivotConfig;
import yams.core.motorcontrollers.SimSupplier;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;

/**
 * Verifies that a simulated mechanism carrying game pieces moves as the heavier mechanism would: each
 * test drives the simulation at a voltage worked out by hand from the physics, and checks the mechanism
 * holds still, falls or speeds up as it should.
 */
public class GamePieceSimTest {
  private static final double GRAVITY = 9.8;
  /**
   * The reduction from the NEO to the mechanism. Low enough that the mechanism takes a few tenths of a
   * second to reach its speed, so its inertia shows.
   */
  private static final double GEARING = 10;
  private static final DCMotor MOTOR = DCMotor.getNEO(1);
  private static final double ARM_LENGTH = 0.5;
  private static final double ARM_MASS = 2;
  private static final double PIECE_MASS = 1;
  private static final double CARRIAGE_MASS = 5;
  private static final double DRUM_RADIUS = 0.02;
  private static final double SIM_PERIOD = 0.02;

  private static int offset = 0;
  private SmartMotorController smc;

  private static SmartMotorControllerConfig config(String name) {
    offset++;
    SmartMotorControllerTestSubsystem subsys = new SmartMotorControllerTestSubsystem();
    subsys.setName(name + offset);
    return (SmartMotorControllerConfig) new SmartMotorControllerConfig()
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(GEARING)))
        .withZeroPower(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(80))
        .withMotorInverted(false)
        .withControlMode(ControlMode.OPEN_LOOP)
        .withSubsystem(subsys)
        .withTelemetry(name + offset, TelemetryVerbosity.LOW);
  }

  private SmartMotorController spark(SmartMotorControllerConfig cfg) {
    smc = new SparkWrapper(DeviceCreator.createSparkMax(), MOTOR, cfg);
    return smc;
  }

  /** An arm, level, whose moment of inertia is that of a uniform rod, as WPILib's arm assumes. */
  private SimSupplier arm() {
    SmartMotorControllerConfig cfg = config("GamePieceArm");
    cfg.withMomentOfInertia(Meters.of(ARM_LENGTH), Kilograms.of(ARM_MASS)).withStartingPosition(Degrees.of(0));
    SmartMotorController motor = spark(cfg);
    new Arm(new ArmConfig().withLength(Meters.of(ARM_LENGTH)).withHardLimits(Degrees.of(-100), Degrees.of(100)), motor);
    return motor.getSimSupplier().orElseThrow();
  }

  /** An elevator, halfway up. */
  private SimSupplier elevator() {
    SmartMotorControllerConfig cfg = config("GamePieceElevator");
    cfg.withMechanismCircumference(Meters.of(2 * Math.PI * DRUM_RADIUS)).withStartingPosition(Meters.of(1));
    SmartMotorController motor = spark(cfg);
    new Elevator(new ElevatorConfig().withHardLimits(Meters.of(0), Meters.of(2)).withCarriageWeight(Kilograms.of(CARRIAGE_MASS)), motor);
    return motor.getSimSupplier().orElseThrow();
  }

  /** A pivot, which gravity does not turn. */
  private SimSupplier pivot() {
    SmartMotorControllerConfig cfg = config("GamePiecePivot");
    cfg.withMomentOfInertia(Meters.of(ARM_LENGTH), Kilograms.of(ARM_MASS)).withStartingPosition(Degrees.of(0));
    SmartMotorController motor = spark(cfg);
    new Pivot(new PivotConfig().withHardLimits(Degrees.of(-180), Degrees.of(180)), motor);
    motor.setupSimulation();
    return motor.getSimSupplier().orElseThrow();
  }

  /** Drive the simulation at a voltage for a time, as the motor controller would each loop. */
  private static void drive(SimSupplier sim, double volts, double seconds) {
    for (int loop = 0; loop < Math.round(seconds / SIM_PERIOD); loop++) {
      sim.setMechanismStatorVoltage(Volts.of(volts));
      sim.starveUpdateSim();
      sim.updateSimState();
    }
  }

  /** The voltage that holds a torque at the mechanism with the motor stalled. */
  private static double stallVolts(double torque) {
    return torque * MOTOR.R / (GEARING * MOTOR.Kt);
  }

  private static double armDegrees(SimSupplier sim) {
    return sim.getMechanismPosition().in(Degrees);
  }

  @Test
  void armWithAGamePieceIsHeldLevelByTheVoltageThatHoldsBoth() {
    SimSupplier sim = arm();
    double armTorque = ARM_MASS * GRAVITY * ARM_LENGTH / 2;
    double pieceTorque = PIECE_MASS * GRAVITY * ARM_LENGTH;

    // Without the game piece, the voltage that holds the arm level holds it.
    drive(sim, stallVolts(armTorque), 0.5);
    assertEquals(0, armDegrees(sim), 0.5, "empty arm held level");

    // With the game piece at its end, the same voltage no longer holds it.
    sim.addGamePiece(Kilograms.of(PIECE_MASS));
    drive(sim, stallVolts(armTorque), 0.5);
    assertTrue(armDegrees(sim) < -5, "arm with a game piece should fall, but is at " + armDegrees(sim) + "°");

    // Back at level, the voltage that holds the arm and the game piece holds it.
    sim.setMechanismPosition(Degrees.of(0));
    sim.setMechanismVelocity(RadiansPerSecond.of(0));
    double holdVolts = stallVolts(armTorque + pieceTorque);
    drive(sim, holdVolts, 0.5);
    assertEquals(0, armDegrees(sim), 0.5, "arm with a game piece held level");
    // The current is the motor's at that voltage, not at the voltage the simulation is given.
    assertEquals(holdVolts / MOTOR.R, sim.getStatorCurrent().in(Amps), holdVolts / MOTOR.R * 0.03, "current holding the game piece");

    // Without the game piece again, that voltage lifts the arm.
    sim.removeGamePiece();
    drive(sim, holdVolts, 0.5);
    assertTrue(armDegrees(sim) > 5, "arm without its game piece should rise, but is at " + armDegrees(sim) + "°");
  }

  @Test
  void armWithAGamePieceAcceleratesAsItsMomentOfInertiaSays() {
    SimSupplier sim = arm();
    double armMoi = SingleJointedArmSim.estimateMOI(ARM_LENGTH, ARM_MASS);
    double pieceDistance = 0.3;
    double withPiece = armMoi + PIECE_MASS * pieceDistance * pieceDistance;
    double holdVolts = stallVolts(ARM_MASS * GRAVITY * ARM_LENGTH / 2 + PIECE_MASS * GRAVITY * pieceDistance);
    sim.addGamePiece(Kilograms.of(PIECE_MASS), Meters.of(pieceDistance));

    // A volt above the voltage that holds it speeds the arm up toward a volt's free speed, with a time
    // constant in proportion to the moment of inertia with the game piece. It stays close to level, where
    // gravity barely changes.
    double seconds = 0.1;
    drive(sim, holdVolts + 1, seconds);
    double expected = speedAfter(1, seconds, withPiece);
    assertEquals(expected, sim.getMechanismVelocity().in(RadiansPerSecond), expected * 0.03, "arm's speed after " + seconds + " s");
    // Without the game piece's inertia, the arm would be noticeably faster.
    assertTrue(speedAfter(1, seconds, armMoi) > expected * 1.2);
  }

  /**
   * A motor's speed at the mechanism, from rest, after a time at a voltage above what holds it: it rises
   * to the voltage's free speed with a time constant in proportion to the inertia.
   */
  private static double speedAfter(double volts, double seconds, double moi) {
    double freeSpeed = volts * MOTOR.Kv / GEARING;
    double timeConstant = moi * MOTOR.R * MOTOR.Kv / (GEARING * GEARING * MOTOR.Kt);
    return freeSpeed * (1 - Math.exp(-seconds / timeConstant));
  }

  @Test
  void elevatorWithAGamePieceIsHeldByTheVoltageThatHoldsBoth() {
    SimSupplier sim = elevator();
    double carriageVolts = stallVolts(CARRIAGE_MASS * GRAVITY * DRUM_RADIUS);
    double loadedVolts = stallVolts((CARRIAGE_MASS + PIECE_MASS) * GRAVITY * DRUM_RADIUS);
    double start = sim.getMechanismPosition().in(Radians);

    drive(sim, carriageVolts, 0.5);
    assertEquals(start, sim.getMechanismPosition().in(Radians), 0.05, "empty carriage held");

    // With the game piece, the same voltage lets the carriage fall, at the speed where the motor's back
    // EMF holds the game piece's extra weight.
    sim.addGamePiece(Kilograms.of(PIECE_MASS));
    drive(sim, carriageVolts, 0.5);
    double fallSpeed = PIECE_MASS * GRAVITY * DRUM_RADIUS * MOTOR.R * MOTOR.Kv / (GEARING * GEARING * MOTOR.Kt) * DRUM_RADIUS;
    assertEquals(-fallSpeed, sim.getMechanismVelocity().in(RadiansPerSecond) * DRUM_RADIUS, fallSpeed * 0.03,
                 "carriage with a game piece falling");

    double heldAt = sim.getMechanismPosition().in(Radians);
    sim.setMechanismVelocity(RadiansPerSecond.of(0));
    drive(sim, loadedVolts, 0.5);
    assertEquals(heldAt, sim.getMechanismPosition().in(Radians), 0.05, "carriage with a game piece held");
  }

  @Test
  void pivotWithAGamePieceSpeedsUpAsItsMomentOfInertiaSays() {
    SimSupplier sim = pivot();
    double moi = SingleJointedArmSim.estimateMOI(ARM_LENGTH, ARM_MASS);
    double pieceDistance = 0.4;
    double withPiece = moi + PIECE_MASS * pieceDistance * pieceDistance;
    sim.addGamePiece(Kilograms.of(PIECE_MASS), Meters.of(pieceDistance));

    // A motor's speed rises to its free speed with a time constant in proportion to the inertia.
    double seconds = 0.3;
    drive(sim, 2, seconds);
    double expected = speedAfter(2, seconds, withPiece);
    assertEquals(expected, sim.getMechanismVelocity().in(RadiansPerSecond), expected * 0.02, "pivot's speed after " + seconds + " s");
    // Without the game piece's inertia, the pivot would be noticeably faster.
    assertTrue(speedAfter(2, seconds, moi) > expected * 1.2);
  }

  @Test
  void aPivotGamePieceNeedsItsDistanceFromTheAxis() {
    SimSupplier sim = pivot();
    assertThrows(UnsupportedOperationException.class, () -> sim.addGamePiece(Kilograms.of(PIECE_MASS)));
    // Removing a game piece that is not there does nothing.
    sim.removeGamePiece();
  }

  @BeforeEach
  void beforeTest() {
    MockHardwareExtension.beforeAll();
    RoboRioSim.setVInVoltage(12);
  }

  @AfterEach
  void afterTest() {
    if (smc != null) {
      SmartMotorControllerTestSubsystem subsys =
          (SmartMotorControllerTestSubsystem) ((SmartMotorControllerConfig) smc.getConfig()).getSubsystem();
      SmartMotorControllerCommandRegistry.removeCommands(subsys);
      CommandScheduler.getInstance().unregisterSubsystem(subsys);
      smc.close();
      DeviceCreator.silence(smc);
      ((SparkMax) smc.getMotorController()).close();
      smc = null;
    }
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }
}
