// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.motorcontrollers;

import static org.wpilib.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertThrows;

import com.revrobotics.spark.SparkMax;
import yams.helpers.DeviceCreator;
import org.wpilib.math.system.DCMotor;
import org.wpilib.util.Preferences;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import yams.exceptions.SmartMotorControllerConfigurationException;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;
import yams.motorcontrollers.local.SparkWrapper;

public class SparkAbsoluteEncoderTest
{
    private static SmartMotorControllerConfig baseConfig()
    {
        return new SmartMotorControllerConfig()
                .withSubsystem(new SmartMotorControllerTestSubsystem())
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)));
    }

    @BeforeEach
    void startTest()
    {
        MockHardwareExtension.beforeAll();
    }

    @AfterEach
    void endTest()
    {
        MockHardwareExtension.afterAll();
        Preferences.removeAll();
    }

    @Test
    void testZeroCenteredTrueWith0_5DiscontinuityPoint()
    {
        SparkMax sparkMax = DeviceCreator.createSparkMax();
        SmartMotorControllerConfig config = baseConfig()
                .withExternalEncoder(sparkMax.getAbsoluteEncoder())
                .withExternalEncoderDiscontinuityPoint(Rotations.of(0.5));
        assertDoesNotThrow(() -> new SparkWrapper(sparkMax, DCMotor.getNEO(1), config));
        sparkMax.close();
    }

    @Test
    void testZeroCenteredFalseWith1_0DiscontinuityPoint()
    {
        SparkMax sparkMax = DeviceCreator.createSparkMax();
        SmartMotorControllerConfig config = baseConfig()
                .withExternalEncoder(sparkMax.getAbsoluteEncoder())
                .withExternalEncoderDiscontinuityPoint(Rotations.of(1));
        assertDoesNotThrow(() -> new SparkWrapper(sparkMax, DCMotor.getNEO(1), config));
        sparkMax.close();
    }

    @Test
    void testExceptionWhenDiscontinuityPointWithoutEncoder()
    {
        SparkMax sparkMax = DeviceCreator.createSparkMax();
        SmartMotorControllerConfig config = baseConfig()
                .withExternalEncoderDiscontinuityPoint(Rotations.of(0.5));
        assertThrows(SmartMotorControllerConfigurationException.class,
                () -> new SparkWrapper(sparkMax, DCMotor.getNEO(1), config));
        sparkMax.close();
    }
}
