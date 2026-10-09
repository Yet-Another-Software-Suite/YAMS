// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Volts;

import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.trajectory.ExponentialProfile;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;

/**
 * Tests the exponential profile constraints derived from physical constants: kV and kA must be
 * finite and match the velocity dynamics of the mechanism (the same formula as the C++ port).
 */
public class ExponentialProfileConfigTest {
  private static final double kGearRatio = 12;

  private static yams.commands2.config.SmartMotorControllerConfig baseConfig() {
    return new yams.commands2.config.SmartMotorControllerConfig()
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)));
  }

  private static void assertFiniteAndClose(double expected, double actual, String what) {
    assertTrue(Double.isFinite(actual), what + " should be finite but was " + actual);
    assertEquals(expected, actual, Math.abs(expected) * 1e-9, what);
  }

  @Test
  void armProfileFromPhysicalConstantsHasFiniteGains() {
    final DCMotor motor = DCMotor.getNEO(1);
    final double moi = 0.1;
    final ExponentialProfile.Constraints constraints =
        baseConfig()
            .withExponentialProfile(Volts.of(12), motor, KilogramSquareMeters.of(moi))
            .getExponentialProfile()
            .orElseThrow();

    // Velocity dynamics of the arm, in radians: dω/dt = A ω + B u.
    final double a = -kGearRatio * kGearRatio * motor.Kt / (motor.Kv * motor.R * moi);
    final double b = kGearRatio * motor.Kt / (motor.R * moi);
    // The profile is in rotations; a rotation is 2π radians.
    final double expectedKv = -a / b * 2 * Math.PI;
    final double expectedKa = 2 * Math.PI / b;

    assertFiniteAndClose(expectedKv, -constraints.A / constraints.B, "arm kV");
    assertFiniteAndClose(expectedKa, 1.0 / constraints.B, "arm kA");
    assertEquals(12, constraints.maxInput, 1e-9);
  }

  @Test
  void elevatorProfileFromPhysicalConstantsHasFiniteGains() {
    final DCMotor motor = DCMotor.getKrakenX60(2);
    final double mass = 8;
    final double drumRadius = 0.025;
    final ExponentialProfile.Constraints constraints =
        baseConfig()
            .withExponentialProfile(Volts.of(10), motor, Kilograms.of(mass), Meters.of(drumRadius))
            .getExponentialProfile()
            .orElseThrow();

    // Velocity dynamics of the elevator, in meters: dv/dt = A v + B u.
    final double a =
        -kGearRatio * kGearRatio * motor.Kt / (motor.R * drumRadius * drumRadius * mass * motor.Kv);
    final double b = kGearRatio * motor.Kt / (motor.R * drumRadius * mass);
    final double expectedKv = -a / b;
    final double expectedKa = 1.0 / b;

    assertFiniteAndClose(expectedKv, -constraints.A / constraints.B, "elevator kV");
    assertFiniteAndClose(expectedKa, 1.0 / constraints.B, "elevator kA");
    assertEquals(10, constraints.maxInput, 1e-9);
  }
}
