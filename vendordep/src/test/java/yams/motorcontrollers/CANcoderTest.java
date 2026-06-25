// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.motorcontrollers;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertThrows;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import yams.exceptions.SmartMotorControllerConfigurationException;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * Tests for CANcoder integration with TalonFXWrapper via SmartMotorControllerConfig.
 * @implNote Disabled until 2027, bc CTRE bug
 */
public class CANcoderTest
{
    @BeforeEach
    void setUp()
    {
        MockHardwareExtension.beforeAll();
    }

    @AfterEach
    void tearDown()
    {
        MockHardwareExtension.afterAll();
        Preferences.removeAll();
    }

    /**
     * Close a wrapper and unregister its subsystem to free HAL handles and scheduler registrations.
     * Does not call subsys.close() because smc is not set via setSMC() in these tests.
     */
    private static void closeWrapper(TalonFXWrapper wrapper, TalonFX talon)
    {
        SmartMotorControllerTestSubsystem subsys =
                (SmartMotorControllerTestSubsystem) wrapper.getConfig().getSubsystem();
        CommandScheduler.getInstance().unregisterSubsystem(subsys);
        wrapper.close();
        talon.close();
    }

    /**
     * Test 1: Verify that a valid discontinuity point of 0.5 rotations is accepted without exception.
     */
    @Test
    void testDiscontinuityPointApplied()
    {
        TalonFX   talon    = DeviceCreator.createTalonFX();
        CANcoder  cancoder = DeviceCreator.createCANcoderFor(talon);
        SmartMotorControllerConfig config = new SmartMotorControllerConfig()
                .withSubsystem(new SmartMotorControllerTestSubsystem())
                .withTelemetry("TalonFX(80) CANcoder DP0.5", TelemetryVerbosity.LOW)
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
                .withExternalEncoder(cancoder)
                .withExternalEncoderDiscontinuityPoint(Rotations.of(0.5));

        TalonFXWrapper[] result = new TalonFXWrapper[1];
        assertDoesNotThrow(() -> result[0] = new TalonFXWrapper(talon, DCMotor.getKrakenX60(1), config));
        if (result[0] != null)
        {
            closeWrapper(result[0], talon);
        }
    }

    /**
     * Test 2: Verify that a valid discontinuity point of 1.0 rotations is accepted without exception.
     */
    @Test
    void testDiscontinuityPoint1_0Accepted()
    {
        TalonFX   talon    = DeviceCreator.createTalonFX();
        CANcoder  cancoder = DeviceCreator.createCANcoderFor(talon);
        SmartMotorControllerConfig config = new SmartMotorControllerConfig()
                .withSubsystem(new SmartMotorControllerTestSubsystem())
                .withTelemetry("TalonFX(81) CANcoder DP1.0", TelemetryVerbosity.LOW)
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
                .withExternalEncoder(cancoder)
                .withExternalEncoderDiscontinuityPoint(Rotations.of(1));

        TalonFXWrapper[] result = new TalonFXWrapper[1];
        assertDoesNotThrow(() -> result[0] = new TalonFXWrapper(talon, DCMotor.getKrakenX60(1), config));
        if (result[0] != null)
        {
            closeWrapper(result[0], talon);
        }
    }

    /**
     * Test 3: Verify that an invalid discontinuity point (not 0.5 or 1.0) throws at config-build time.
     * No TalonFX/subsystem is created because the exception fires on withExternalEncoderDiscontinuityPoint.
     */
    @Test
    void testInvalidDiscontinuityPointThrows()
    {
        assertThrows(SmartMotorControllerConfigurationException.class, () ->
                new SmartMotorControllerConfig()
                        .withExternalEncoderDiscontinuityPoint(Rotations.of(0.3))
        );
    }

    /**
     * Test 4: Verify that a zero offset of 45 degrees is accepted without exception.
     */
    @Test
    void testZeroOffsetApplied()
    {
        TalonFX   talon    = DeviceCreator.createTalonFX();
        CANcoder  cancoder = DeviceCreator.createCANcoderFor(talon);
        SmartMotorControllerConfig config = new SmartMotorControllerConfig()
                .withSubsystem(new SmartMotorControllerTestSubsystem())
                .withTelemetry("TalonFX(83) CANcoder ZeroOffset", TelemetryVerbosity.LOW)
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
                .withExternalEncoder(cancoder)
                .withExternalEncoderZeroOffset(Degrees.of(45));

        TalonFXWrapper[] result = new TalonFXWrapper[1];
        assertDoesNotThrow(() -> result[0] = new TalonFXWrapper(talon, DCMotor.getKrakenX60(1), config));
        if (result[0] != null)
        {
            closeWrapper(result[0], talon);
        }
    }

    /**
     * Test 5: Verify that constructing a TalonFXWrapper with a CANcoder but no discontinuity point
     * does NOT throw — TalonFX handles this differently from SparkAbsoluteEncoder.
     */
    @Test
    void testCANcoderConfiguredWithoutDiscontinuityPoint()
    {
        TalonFX   talon    = DeviceCreator.createTalonFX();
        CANcoder  cancoder = DeviceCreator.createCANcoderFor(talon);
        SmartMotorControllerConfig config = new SmartMotorControllerConfig()
                .withSubsystem(new SmartMotorControllerTestSubsystem())
                .withTelemetry("TalonFX(84) CANcoder NoDP", TelemetryVerbosity.LOW)
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
                .withExternalEncoder(cancoder);

        TalonFXWrapper[] result = new TalonFXWrapper[1];
        assertDoesNotThrow(() -> result[0] = new TalonFXWrapper(talon, DCMotor.getKrakenX60(1), config));
        if (result[0] != null)
        {
            closeWrapper(result[0], talon);
        }
    }
}
