// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.system.DCMotor;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.core.exceptions.SmartMotorControllerConfigurationException;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;

/**
 * Tests for CANcoder integration with TalonFXWrapper via SmartMotorControllerConfig.
 *
 * @implNote Disabled until 2027, bc CTRE bug
 */
public class CANcoderTest {
  @BeforeEach
  void setUp() {
    MockHardwareExtension.beforeAll();
  }

  @AfterEach
  void tearDown() {
    MockHardwareExtension.afterAll();
  }

  /**
   * Close a wrapper and unregister its subsystem to free HAL handles and scheduler registrations.
   * Does not call subsys.close() because smc is not set via setSMC() in these tests.
   */
  private static void closeWrapper(TalonFXWrapper wrapper, TalonFX talon) {
    SmartMotorControllerTestSubsystem subsys =
        (SmartMotorControllerTestSubsystem)
            ((yams.commands2.config.SmartMotorControllerConfig) wrapper.getConfig()).getSubsystem();
    CommandScheduler.getInstance().unregisterSubsystem(subsys);
    wrapper.close();
    DeviceCreator.silence(wrapper);
    talon.close();
  }

  /** The AbsoluteSensorDiscontinuityPoint the CANcoder holds, in rotations. */
  private static double cancoderAbsoluteSensorDiscontinuityPoint(CANcoder cancoder) {
    final CANcoderConfiguration config = new CANcoderConfiguration();
    DeviceCreator.refreshConfig(() -> cancoder.getConfigurator().refresh(config, 1.0));
    return config.MagnetSensor.AbsoluteSensorDiscontinuityPoint;
  }

  /**
   * Test 1: Continuous wrapping over [-0.5, 0.5) rotations gives the CANcoder an
   * AbsoluteSensorDiscontinuityPoint of 0.5 rotations.
   */
  @Test
  void testWrappingCenteredOnZeroApplied() {
    TalonFX talon = DeviceCreator.createTalonFX();
    CANcoder cancoder = DeviceCreator.createCANcoderFor(talon);
    SmartMotorControllerConfig config =
        new yams.commands2.config.SmartMotorControllerConfig()
            .withSubsystem(new SmartMotorControllerTestSubsystem())
            .withTelemetry("TalonFX(80) CANcoder DP0.5", TelemetryVerbosity.LOW)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            .withExternalEncoder(cancoder)
            .withClosedLoopController(4, 0, 0)
            .withContinuousWrapping(Rotations.of(-0.5), Rotations.of(0.5));

    TalonFXWrapper[] result = new TalonFXWrapper[1];
    assertDoesNotThrow(
        () -> result[0] = new TalonFXWrapper(talon, DCMotor.getKrakenX60(1), config));
    if (result[0] != null) {
      assertEquals(0.5, cancoderAbsoluteSensorDiscontinuityPoint(cancoder), 1e-9);
      closeWrapper(result[0], talon);
    }
  }

  /**
   * Wrapping over [-0.5, 0.5) rotations gives the CANcoder an AbsoluteSensorDiscontinuityPoint of
   * 0.5 rotations, and the Talon's closed loop does not wrap.
   */
  @Test
  void testWrappingAppliedWithoutContinuousWrap() {
    TalonFX talon = DeviceCreator.createTalonFX();
    CANcoder cancoder = DeviceCreator.createCANcoderFor(talon);
    SmartMotorControllerConfig config =
        new yams.commands2.config.SmartMotorControllerConfig()
            .withSubsystem(new SmartMotorControllerTestSubsystem())
            .withTelemetry("TalonFX(84) CANcoder wrapping", TelemetryVerbosity.LOW)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            .withExternalEncoder(cancoder)
            .withClosedLoopController(4, 0, 0)
            .withWrapping(Rotations.of(-0.5), Rotations.of(0.5));
    final TalonFXWrapper wrapper = new TalonFXWrapper(talon, DCMotor.getKrakenX60(1), config);
    assertEquals(0.5, cancoderAbsoluteSensorDiscontinuityPoint(cancoder), 1e-9);
    final TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    DeviceCreator.refreshConfig(() -> talon.getConfigurator().refresh(talonConfig, 1.0));
    assertFalse(talonConfig.ClosedLoopGeneral.ContinuousWrap, "the Talon does not wrap");
    closeWrapper(wrapper, talon);
  }

  /**
   * Test 2: Continuous wrapping over [0, 1) rotations gives the CANcoder an
   * AbsoluteSensorDiscontinuityPoint of 1 rotation.
   */
  @Test
  void testWrappingFromZeroApplied() {
    TalonFX talon = DeviceCreator.createTalonFX();
    CANcoder cancoder = DeviceCreator.createCANcoderFor(talon);
    SmartMotorControllerConfig config =
        new yams.commands2.config.SmartMotorControllerConfig()
            .withSubsystem(new SmartMotorControllerTestSubsystem())
            .withTelemetry("TalonFX(81) CANcoder DP1.0", TelemetryVerbosity.LOW)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            .withExternalEncoder(cancoder)
            .withClosedLoopController(4, 0, 0)
            .withContinuousWrapping(Rotations.of(0), Rotations.of(1));

    TalonFXWrapper[] result = new TalonFXWrapper[1];
    assertDoesNotThrow(
        () -> result[0] = new TalonFXWrapper(talon, DCMotor.getKrakenX60(1), config));
    if (result[0] != null) {
      assertEquals(1, cancoderAbsoluteSensorDiscontinuityPoint(cancoder), 1e-9);
      closeWrapper(result[0], talon);
    }
  }

  /**
   * Test 3: A CANcoder reads a range from [-1, 0) to [0, 1) rotations, so wrapping past 1 rotation
   * is rejected.
   */
  @Test
  void testWrappingTheCANcoderCannotReadThrows() {
    TalonFX talon = DeviceCreator.createTalonFX();
    CANcoder cancoder = DeviceCreator.createCANcoderFor(talon);
    SmartMotorControllerConfig config =
        new yams.commands2.config.SmartMotorControllerConfig()
            .withSubsystem(new SmartMotorControllerTestSubsystem())
            .withTelemetry("TalonFX(82) CANcoder wrapping", TelemetryVerbosity.LOW)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            .withExternalEncoder(cancoder)
            .withClosedLoopController(4, 0, 0)
            .withContinuousWrapping(Rotations.of(0.5), Rotations.of(1.5));
    assertThrows(
        SmartMotorControllerConfigurationException.class,
        () -> new TalonFXWrapper(talon, DCMotor.getKrakenX60(1), config));
    talon.close();
    cancoder.close();
  }

  /** Test 4: Verify that a zero offset of 45 degrees is accepted without exception. */
  @Test
  void testZeroOffsetApplied() {
    TalonFX talon = DeviceCreator.createTalonFX();
    CANcoder cancoder = DeviceCreator.createCANcoderFor(talon);
    SmartMotorControllerConfig config =
        new yams.commands2.config.SmartMotorControllerConfig()
            .withSubsystem(new SmartMotorControllerTestSubsystem())
            .withTelemetry("TalonFX(83) CANcoder ZeroOffset", TelemetryVerbosity.LOW)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            .withExternalEncoder(cancoder)
            .withExternalEncoderZeroOffset(Degrees.of(45));

    TalonFXWrapper[] result = new TalonFXWrapper[1];
    assertDoesNotThrow(
        () -> result[0] = new TalonFXWrapper(talon, DCMotor.getKrakenX60(1), config));
    if (result[0] != null) {
      closeWrapper(result[0], talon);
    }
  }

  /**
   * Test 5: Verify that constructing a TalonFXWrapper with a CANcoder but no continuous wrapping
   * does NOT throw TalonFX handles this differently from SparkAbsoluteEncoder.
   */
  @Test
  void testCANcoderConfiguredWithoutWrapping() {
    TalonFX talon = DeviceCreator.createTalonFX();
    CANcoder cancoder = DeviceCreator.createCANcoderFor(talon);
    SmartMotorControllerConfig config =
        new yams.commands2.config.SmartMotorControllerConfig()
            .withSubsystem(new SmartMotorControllerTestSubsystem())
            .withTelemetry("TalonFX(84) CANcoder NoDP", TelemetryVerbosity.LOW)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            .withExternalEncoder(cancoder);

    TalonFXWrapper[] result = new TalonFXWrapper[1];
    assertDoesNotThrow(
        () -> result[0] = new TalonFXWrapper(talon, DCMotor.getKrakenX60(1), config));
    if (result[0] != null) {
      closeWrapper(result[0], talon);
    }
  }
}
