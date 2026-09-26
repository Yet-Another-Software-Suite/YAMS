// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.wpilib.units.Units.Rotations;

import yams.commands2.config.SmartMotorControllerConfig;
import com.revrobotics.spark.SparkMax;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;
import org.wpilib.preferences.Preferences;
import yams.core.exceptions.SmartMotorControllerConfigurationException;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;

public class SparkAbsoluteEncoderTest {
  private static SmartMotorControllerConfig baseConfig() {
    return new yams.commands2.config.SmartMotorControllerConfig()
        .withSubsystem(new SmartMotorControllerTestSubsystem())
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)));
  }

  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
  }

  @AfterEach
  void endTest() {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  @Test
  void testZeroCenteredTrueWith0_5DiscontinuityPoint() {
    SparkMax sparkMax = DeviceCreator.createSparkMax();
    SmartMotorControllerConfig config =
        baseConfig()
            .withExternalEncoder(sparkMax.getAbsoluteEncoder())
            .withExternalEncoderDiscontinuityPoint(Rotations.of(0.5));
    assertDoesNotThrow(() -> new SparkWrapper(sparkMax, DCMotor.getNEO(1), config));
    sparkMax.close();
  }

  @Test
  void testZeroCenteredFalseWith1_0DiscontinuityPoint() {
    SparkMax sparkMax = DeviceCreator.createSparkMax();
    SmartMotorControllerConfig config =
        baseConfig()
            .withExternalEncoder(sparkMax.getAbsoluteEncoder())
            .withExternalEncoderDiscontinuityPoint(Rotations.of(1));
    assertDoesNotThrow(() -> new SparkWrapper(sparkMax, DCMotor.getNEO(1), config));
    sparkMax.close();
  }

  @Test
  void testExceptionWhenDiscontinuityPointWithoutEncoder() {
    SparkMax sparkMax = DeviceCreator.createSparkMax();
    SmartMotorControllerConfig config =
        baseConfig().withExternalEncoderDiscontinuityPoint(Rotations.of(0.5));
    assertThrows(
        SmartMotorControllerConfigurationException.class,
        () -> new SparkWrapper(sparkMax, DCMotor.getNEO(1), config));
    sparkMax.close();
  }
}
