// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Volts;

import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.core.exceptions.SmartMotorControllerConfigurationException;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;

/**
 * A config's moment of inertia may only be set once, whichever setter sets it, but a copy of the
 * config may set its own.
 */
public class MomentOfInertiaTest {
  private static SmartMotorControllerConfig config() {
    return new SmartMotorControllerConfig()
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)));
  }

  @Test
  void settingItTwiceThrows() {
    assertThrows(SmartMotorControllerConfigurationException.class,
        () -> config().withMomentOfInertia(KilogramSquareMeters.of(0.1))
            .withMomentOfInertia(KilogramSquareMeters.of(0.2)));
    assertThrows(SmartMotorControllerConfigurationException.class,
        () -> config().withMomentOfInertia(Inches.of(18), Pounds.of(8))
            .withWheelMomentOfInertia(Inches.of(4), Pounds.of(1)));
    assertThrows(SmartMotorControllerConfigurationException.class,
        () -> config().withWheelMomentOfInertia(Inches.of(4), Pounds.of(1))
            .withExponentialProfile(Volts.of(12), DCMotor.getNEO(1), KilogramSquareMeters.of(0.1)));
  }

  @Test
  void aCopyCanSetItAgain() {
    SmartMotorControllerConfig original = config().withMomentOfInertia(KilogramSquareMeters.of(0.1));
    SmartMotorControllerConfig copy = original.clone();
    assertEquals(0.1, copy.getMOI(), 1e-12);
    assertTrue(copy.isMomentOfInertiaConfigured());

    copy.withMomentOfInertia(KilogramSquareMeters.of(0.2));
    assertEquals(0.2, copy.getMOI(), 1e-12);
    assertEquals(0.1, original.getMOI(), 1e-12);
    assertThrows(SmartMotorControllerConfigurationException.class,
        () -> copy.withMomentOfInertia(KilogramSquareMeters.of(0.3)));
    assertThrows(SmartMotorControllerConfigurationException.class,
        () -> original.withMomentOfInertia(KilogramSquareMeters.of(0.3)));
  }
}
