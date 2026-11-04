// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Celsius;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;
import static org.wpilib.units.Units.Volts;

import com.thrifty.canEncoder.CanEncoder;
import com.thrifty.core.Motor.FeedbackSensorType;
import com.thrifty.nova.Nova;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.preferences.Preferences;
import yams.core.exceptions.SmartMotorControllerConfigurationException;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.SmartMotorController.ClosedLoopControllerSlot;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.local.NovaWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;

/**
 * Tests what a {@link NovaWrapper} sends to the Nova, read back from it, and the configs it rejects:
 * the Nova runs every closed loop but an LQR, an exponential profile and a velocity profile, so
 * options only the SystemCore's closed loop applies are rejected for the others.
 */
public class NovaConfigTest {
  /** Motor rotations per mechanism rotation. */
  private static final double kGearRatio = 12;

  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
  }

  @AfterEach
  void endTest() {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  private static SmartMotorControllerConfig config(String name) {
    return new yams.commands2.config.SmartMotorControllerConfig()
        .withSubsystem(new SmartMotorControllerTestSubsystem())
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withStatorCurrentLimit(Amps.of(40))
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(4, 0, 0)
        .withTelemetry("NovaConfigTest " + name, TelemetryVerbosity.LOW);
  }

  private static void assertEventually(BooleanSupplier condition, String what) throws InterruptedException {
    assertTrue(AbsoluteEncoderCases.eventually(condition), what);
  }

  @Test
  void feedforwardNeedsAMotionProfile() {
    final var exception = assertThrows(SmartMotorControllerConfigurationException.class,
        () -> new NovaWrapper(DeviceCreator.createNova(), DCMotor.getNEO(1), config("feedforward").withFeedforward(new ArmFeedforward(0, 0.2, 0, 0))));
    assertTrue(exception.getMessage().contains("motion profile"), exception.getMessage());
    assertDoesNotThrow(() -> new NovaWrapper(DeviceCreator.createNova(), DCMotor.getNEO(1),
        config("feedforward with profile").withFeedforward(new ArmFeedforward(0, 0.2, 0, 0))
            .withTrapezoidalProfile(RotationsPerSecond.of(1), RotationsPerSecondPerSecond.of(2))));
  }

  @Test
  void systemCoreOptionsNeedTheSystemCoresClosedLoop() {
    assertThrows(SmartMotorControllerConfigurationException.class,
        () -> new NovaWrapper(DeviceCreator.createNova(), DCMotor.getNEO(1), config("temperature cutoff").withTemperatureCutoff(Celsius.of(70))));
    assertThrows(SmartMotorControllerConfigurationException.class,
        () -> new NovaWrapper(DeviceCreator.createNova(), DCMotor.getNEO(1), config("maximum voltage").withClosedLoopControllerMaximumVoltage(Volts.of(6))));
    // An exponential profile runs on the SystemCore, which applies them.
    assertDoesNotThrow(() -> new NovaWrapper(DeviceCreator.createNova(), DCMotor.getNEO(1),
        config("exponential profile temperature cutoff").withTemperatureCutoff(Celsius.of(70))
            .withExponentialProfile(Volts.of(12), RotationsPerSecond.of(10), RotationsPerSecondPerSecond.of(20))));
  }

  /** The closed loop tolerance is the allowable error of the Nova's slots with a PID, in mechanism rotations. */
  @Test
  void toleranceIsTheSlotsAllowableError() throws InterruptedException {
    final Nova nova = DeviceCreator.createNova();
    new NovaWrapper(nova, DCMotor.getNEO(1), config("tolerance").withClosedLoopController(4, 0, 0, ClosedLoopControllerSlot.SLOT_1).withClosedLoopTolerance(Degrees.of(9)));
    assertEventually(() -> Math.abs(nova.status().Pid0.getAllowableError() - 0.025) < 1e-6, "slot 0 allowable error");
    assertEventually(() -> Math.abs(nova.status().Pid1.getAllowableError() - 0.025) < 1e-6, "slot 1 allowable error");
  }

  @Test
  void novaHasTwoSlots() {
    assertThrows(SmartMotorControllerConfigurationException.class,
        () -> new NovaWrapper(DeviceCreator.createNova(), DCMotor.getNEO(1), config("slot 2").withClosedLoopController(4, 0, 0, ClosedLoopControllerSlot.SLOT_2)));
  }

  @Test
  void slotOneGainsGoToTheNovasSecondSlot() throws InterruptedException {
    final Nova nova = DeviceCreator.createNova();
    new NovaWrapper(nova, DCMotor.getNEO(1), config("slot 1").withClosedLoopController(6, 0, 0, ClosedLoopControllerSlot.SLOT_1));
    assertEventually(() -> nova.status().Pid1.getP() == 6, "slot 1 kP on the Nova's slot 1");
    assertEventually(() -> nova.status().Pid0.getP() == 4, "slot 0 kP on the Nova's slot 0");
  }

  @Test
  void mechanismLimitsAndWrappingAreTheNovas() throws InterruptedException {
    final Nova limited = DeviceCreator.createNova();
    new NovaWrapper(limited, DCMotor.getNEO(1), config("limits").withSoftLimits(Degrees.of(-90), Degrees.of(45)));
    assertEventually(() -> limited.status().getSoftLimitEnable(), "soft limits enabled");
    assertEventually(() -> Math.abs(limited.status().getSoftLimitForward() - 0.125) < 1e-6, "forward soft limit in mechanism rotations");
    assertEventually(() -> Math.abs(limited.status().getSoftLimitReverse() + 0.25) < 1e-6, "reverse soft limit in mechanism rotations");
    assertEventually(() -> !limited.status().getAbsWrapping(), "no wrapping");

    final Nova wrapping = DeviceCreator.createNova();
    new NovaWrapper(wrapping, DCMotor.getNEO(1),
        config("wrapping").withExternalEncoder(DeviceCreator.createCanEncoder()).withUseExternalFeedbackEncoder(true)
            .withContinuousWrapping(Rotations.of(0), Rotations.of(1)));
    assertEventually(() -> wrapping.status().getAbsWrapping(), "wrapping enabled");
    assertEventually(() -> !wrapping.status().getSoftLimitEnable(), "no soft limits");
  }

  /** The Nova wraps an absolute encoder's reading over [0, 1) rotations, and nothing else. */
  @Test
  void wrappingNeedsZeroToOneOfAnAbsoluteEncoder() {
    assertThrows(SmartMotorControllerConfigurationException.class,
        () -> new NovaWrapper(DeviceCreator.createNova(), DCMotor.getNEO(1),
            config("wrapping -0.5 to 0.5").withExternalEncoder(DeviceCreator.createCanEncoder()).withUseExternalFeedbackEncoder(true)
                .withContinuousWrapping(Rotations.of(-0.5), Rotations.of(0.5))));
    assertThrows(SmartMotorControllerConfigurationException.class,
        () -> new NovaWrapper(DeviceCreator.createNova(), DCMotor.getNEO(1),
            config("wrapping on the motor's encoder").withContinuousWrapping(Rotations.of(0), Rotations.of(1))));
    // An exponential profile runs on the SystemCore, which wraps any range.
    assertDoesNotThrow(() -> new NovaWrapper(DeviceCreator.createNova(), DCMotor.getNEO(1),
        config("wrapping on the SystemCore").withContinuousWrapping(Rotations.of(-0.5), Rotations.of(0.5))
            .withExponentialProfile(Volts.of(12), RotationsPerSecond.of(10), RotationsPerSecondPerSecond.of(20))));
  }

  @Test
  void feedbackSensorAndItsRatio() throws InterruptedException {
    final Nova motorEncoder = DeviceCreator.createNova();
    new NovaWrapper(motorEncoder, DCMotor.getNEO(1), config("motor encoder"));
    assertEventually(() -> motorEncoder.status().getFeedbackSensor() == FeedbackSensorType.INTERNAL, "the motor's encoder closes the loop");
    assertEventually(() -> Math.abs(motorEncoder.status().getSensorToMechanismRatio() - kGearRatio) < 1e-6, "motor rotations per mechanism rotation");

    final Nova canEncoderNova = DeviceCreator.createNova();
    final CanEncoder canEncoder = DeviceCreator.createCanEncoder();
    new NovaWrapper(canEncoderNova, DCMotor.getNEO(1),
        config("CAN encoder").withExternalEncoder(canEncoder).withUseExternalFeedbackEncoder(true)
            .withExternalEncoderGearing(new MechanismGearing(GearBox.fromReductionStages(2))));
    assertEventually(() -> canEncoderNova.status().getFeedbackSensor() == FeedbackSensorType.CAN_ENCODER, "the CAN encoder closes the loop");
    assertEventually(() -> canEncoderNova.status().getCanEncoderId() == canEncoder.status().getCanId(), "paired with the CAN encoder");
    assertEventually(() -> Math.abs(canEncoderNova.status().getSensorToMechanismRatio() - 2) < 1e-6, "CAN encoder rotations per mechanism rotation");
    assertEquals(FeedbackSensorType.CAN_ENCODER, canEncoderNova.status().getFeedbackSensor());
  }

  /** In simulation, a CAN encoder reads the mechanism's sim supplier through ThriftyLib's MotorSim. */
  @Test
  void canEncoderReadsTheSimulatedMechanism() {
    final CanEncoder canEncoder = DeviceCreator.createCanEncoder();
    final NovaWrapper smc = new NovaWrapper(DeviceCreator.createNova(), DCMotor.getNEO(1),
        config("CAN encoder sim").withExternalEncoder(canEncoder)
            .withExternalEncoderGearing(new MechanismGearing(GearBox.fromReductionStages(2))));
    smc.setEncoderPosition(Rotations.of(0.3));
    smc.setEncoderVelocity(RotationsPerSecond.of(1.5));
    assertEquals(0.6, canEncoder.status().getPositionRel(), 1e-6, "CAN encoder rotations");
    assertEquals(3, canEncoder.status().getVelocity(), 1e-6, "CAN encoder rotations per second");
    assertEquals(0.3, smc.getExternalEncoderMechanismPosition().orElseThrow().in(Rotations), 1e-6);
    assertEquals(1.5, smc.getExternalEncoderMechanismVelocity().orElseThrow().in(RotationsPerSecond), 1e-6);
  }
}
