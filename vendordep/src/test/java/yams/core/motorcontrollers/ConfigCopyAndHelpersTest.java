// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Volts;

import org.junit.jupiter.api.Test;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.trajectory.ExponentialProfile;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.math.ExponentialProfilePIDController;
import yams.core.motorcontrollers.SmartMotorController.ClosedLoopControllerSlot;

/** Fixes found while reviewing the docs: config copies, wheel inertia and elevator constraints. */
public class ConfigCopyAndHelpersTest {
  private static final ClosedLoopControllerSlot SLOT = ClosedLoopControllerSlot.values()[0];

  @Test
  void cloneDoesNotShareControllers() {
    SmartMotorControllerConfig original = new SmartMotorControllerConfig()
        .withClosedLoopController(4, 0, 0)
        .withFeedforward(new ArmFeedforward(0, 0.5, 0));
    SmartMotorControllerConfig copy = original.clone();

    assertNotSame(original.getPID(SLOT).orElseThrow(), copy.getPID(SLOT).orElseThrow());
    assertNotSame(original.getArmFeedforward(SLOT).orElseThrow(), copy.getArmFeedforward(SLOT).orElseThrow());

    // Live tuning one motor must not change the other.
    original.getPID(SLOT).orElseThrow().setP(10);
    original.getArmFeedforward(SLOT).orElseThrow().setKg(2);
    assertEquals(4, copy.getPID(SLOT).orElseThrow().getP());
    assertEquals(0.5, copy.getArmFeedforward(SLOT).orElseThrow().getKg());
  }

  @Test
  void wheelMomentOfInertiaIsASolidDisk() {
    double radius = Inches.of(2).in(Meters);
    double mass = Pounds.of(1).in(Kilograms);
    SmartMotorControllerConfig config = new SmartMotorControllerConfig()
        .withWheelMomentOfInertia(Inches.of(4), Pounds.of(1));
    assertEquals(0.5 * mass * radius * radius, config.getMOI(), 1e-12);
  }

  @Test
  void elevatorConstraintsAreFinite() {
    ExponentialProfile.Constraints constraints = ExponentialProfilePIDController.createElevatorConstraints(
        Volts.of(12), DCMotor.getNEO(2), Pounds.of(16), Inches.of(1),
        new MechanismGearing(GearBox.fromReductionStages(3, 4)));
    assertTrue(Double.isFinite(constraints.A) && Double.isFinite(constraints.B),
               "The elevator's exponential profile constraints must not be NaN or infinite.");
    assertTrue(constraints.maxVelocity() > 0, "The elevator must be able to move.");
  }
}
