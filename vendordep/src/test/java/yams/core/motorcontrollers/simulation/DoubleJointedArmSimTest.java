// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers.simulation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.SingleJointedArmSim;
import yams.helpers.MockHardwareExtension;

class DoubleJointedArmSimTest {
  private static final double kLowerLength = 0.6;
  private static final double kUpperLength = 0.75;
  private static final double kLowerMass = 3;
  private static final double kUpperMass = 2;
  private static final double kGearing = 60;
  private static final DCMotor kMotor = DCMotor.getNEO(1);
  private static final double kLowerMOI = SingleJointedArmSim.estimateMOI(kLowerLength, kLowerMass);
  private static final double kUpperMOI = SingleJointedArmSim.estimateMOI(kUpperLength, kUpperMass);

  @BeforeAll
  static void setUp() {
    // SingleJointedArmSim reads the battery voltage from the HAL.
    MockHardwareExtension.beforeAll();
  }

  @AfterAll
  static void tearDown() {
    MockHardwareExtension.afterAll();
  }

  private static DoubleJointedArmSim sim(double lowerStart, double upperStart, boolean gravity) {
    return new DoubleJointedArmSim(
        kMotor,
        kGearing,
        kLowerMOI,
        Meters.of(kLowerLength),
        Degrees.of(-360),
        Degrees.of(360),
        Degrees.of(lowerStart),
        kMotor,
        kGearing,
        kUpperMOI,
        Meters.of(kUpperLength),
        Degrees.of(-360),
        Degrees.of(360),
        Degrees.of(upperStart),
        gravity);
  }

  /** Voltage that holds a still arm against a torque, from the motor's equation at 0 speed. */
  private static double holdingVoltage(double torque) {
    return -torque / kGearing / kMotor.Kt * kMotor.R;
  }

  @Test
  void theShoulderCarriesTheUpperArmsWeight() {
    DoubleJointedArmSim sim = sim(0, 0, true);
    double[] gravity = sim.getGravityTorques(0, 0);

    // Both arms horizontal: the lower arm's own weight at half its length, and the upper arm's
    // whole weight at the elbow.
    assertEquals(
        -(kLowerMass * kLowerLength / 2 + kUpperMass * kLowerLength) * 9.8, gravity[0], 1e-9);
    assertEquals(-kUpperMass * kUpperLength / 2 * 9.8, gravity[1], 1e-9);
  }

  @Test
  void theUpperArmsWeightDependsOnItsOwnAngle() {
    DoubleJointedArmSim sim = sim(0, 0, true);

    // Pointing straight up, the upper arm puts no torque on itself, but its weight still hangs off
    // the elbow.
    double[] gravity = sim.getGravityTorques(0, Math.PI / 2);
    assertEquals(0, gravity[1], 1e-9);
    assertEquals(
        -(kLowerMass * kLowerLength / 2 + kUpperMass * kLowerLength) * 9.8, gravity[0], 1e-9);
  }

  @Test
  void holdingVoltagesKeepTheArmStill() {
    double lower = Math.toRadians(30);
    double upper = Math.toRadians(-20);
    DoubleJointedArmSim sim = sim(30, -20, true);
    double[] gravity = sim.getGravityTorques(lower, upper);
    sim.setLowerInputVoltage(holdingVoltage(gravity[0]));
    sim.setUpperInputVoltage(holdingVoltage(gravity[1]));

    for (int loop = 0; loop < 50; loop++) {
      sim.update(0.02);
    }

    assertEquals(lower, sim.getLowerAngle(), 1e-6);
    assertEquals(upper, sim.getUpperAngle(), 1e-6);
  }

  @Test
  void holdingOnlyTheLowerArmsWeightLetsTheShoulderDrop() {
    // Two separate single jointed arm simulations would hold the shoulder with this voltage.
    DoubleJointedArmSim sim = sim(0, 0, true);
    sim.setLowerInputVoltage(holdingVoltage(-kLowerMass * kLowerLength / 2 * 9.8));
    sim.setUpperInputVoltage(holdingVoltage(sim.getGravityTorques(0, 0)[1]));

    for (int loop = 0; loop < 10; loop++) {
      sim.update(0.02);
    }

    assertTrue(
        sim.getLowerAngle() < Math.toRadians(-1),
        "the shoulder drops under the upper arm's weight: " + Math.toDegrees(sim.getLowerAngle()));
  }

  @Test
  void swingingTheUpperArmPushesTheLowerArm() {
    // No gravity and the lower motor off: only the upper arm's swing can move the lower arm.
    DoubleJointedArmSim sim = sim(0, 0, false);
    sim.setUpperInputVoltage(12);

    for (int loop = 0; loop < 5; loop++) {
      sim.update(0.02);
    }

    assertTrue(sim.getUpperVelocity() > 0);
    assertTrue(
        sim.getLowerVelocity() < 0, "the lower arm turns the other way: " + sim.getLowerVelocity());
  }

  @Test
  void aLoneArmMatchesASingleJointedArm() {
    // Without an upper arm worth mentioning, the lower arm is a single jointed arm. The upper arm's
    // motor is weak too, or
    // it would hold the tiny arm still at any speed.
    DCMotor weak = new DCMotor(12, 1e-9, 1e-3, 0, 1000, 1);
    DoubleJointedArmSim sim =
        new DoubleJointedArmSim(
            kMotor,
            kGearing,
            kLowerMOI,
            Meters.of(kLowerLength),
            Degrees.of(-360),
            Degrees.of(360),
            Degrees.of(0),
            weak,
            1,
            1e-6,
            Meters.of(kUpperLength),
            Degrees.of(-360),
            Degrees.of(360),
            Degrees.of(0),
            true);
    SingleJointedArmSim single =
        new SingleJointedArmSim(
            kMotor,
            kGearing,
            kLowerMOI,
            kLowerLength,
            Math.toRadians(-360),
            Math.toRadians(360),
            true,
            0);
    sim.setLowerInputVoltage(6);
    single.setInputVoltage(6);

    for (int loop = 0; loop < 25; loop++) {
      sim.update(0.02);
      single.update(0.02);
    }

    assertEquals(single.getAngle(), sim.getLowerAngle(), 1e-3);
    assertEquals(single.getVelocity(), sim.getLowerVelocity(), 1e-2);
  }

  @Test
  void aLightArmOnABigReductionStaysStable() {
    // 0.5 kg and 0.3 m on 100:1: the motor can stop the arm in a third of a millisecond.
    double moi = SingleJointedArmSim.estimateMOI(0.3, 0.5);
    DoubleJointedArmSim sim =
        new DoubleJointedArmSim(
            kMotor,
            100,
            moi,
            Meters.of(0.3),
            Degrees.of(-360),
            Degrees.of(360),
            Degrees.of(0),
            kMotor,
            100,
            moi,
            Meters.of(0.3),
            Degrees.of(-360),
            Degrees.of(360),
            Degrees.of(0),
            true);
    sim.setLowerInputVoltage(12);

    for (int loop = 0; loop < 50; loop++) {
      sim.update(0.02);
    }

    assertTrue(Double.isFinite(sim.getLowerAngle()) && Double.isFinite(sim.getUpperAngle()));
    // Near the free speed of the motor through the reduction, less what gravity takes.
    assertTrue(
        Math.abs(sim.getLowerVelocity()) < kMotor.freeSpeed / 100 * 1.01,
        "no faster than the motor goes: " + sim.getLowerVelocity());
  }

  @Test
  void theArmStopsAtItsHardLimits() {
    DoubleJointedArmSim sim =
        new DoubleJointedArmSim(
            kMotor,
            kGearing,
            kLowerMOI,
            Meters.of(kLowerLength),
            Degrees.of(-10),
            Degrees.of(10),
            Degrees.of(0),
            kMotor,
            kGearing,
            kUpperMOI,
            Meters.of(kUpperLength),
            Degrees.of(-10),
            Degrees.of(10),
            Degrees.of(0),
            true);

    for (int loop = 0; loop < 50; loop++) {
      sim.update(0.02);
    }

    assertEquals(Math.toRadians(-10), sim.getLowerAngle(), 1e-9);
    assertEquals(Math.toRadians(-10), sim.getUpperAngle(), 1e-9);
    assertEquals(0, sim.getLowerVelocity(), 1e-9);
  }
}
