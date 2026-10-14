// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Value;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.controller.ElevatorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.preferences.Preferences;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Voltage;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.NovaWrapper;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;

/**
 * Tests the units of the closed loop gains: an {@link ArmFeedforward}'s kV and kA are per rotation,
 * as YAMS documents, both on the motor controller and on the SystemCore, where WPILib's
 * {@link ArmFeedforward} works in radians; and a distance based controller's gains are per meter.
 */
public class ClosedLoopGainUnitsTest {
  /** Phoenix stores gains in fixed point, so 0.1 reads back as 0.0996. A 2π error is far larger. */
  private static final double kTolerance = 0.01;
  /** Motor rotations per mechanism rotation. */
  private static final double kGearRatio = 12;
  private static final double kVVoltsPerRotationPerSecond = 1.0;
  private static final double kAVoltsPerRotationPerSecondSquared = 0.5;

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
        .withZeroPower(MotorMode.BRAKE)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(0, 0, 0)
        .withTelemetry("ClosedLoopGainUnitsTest " + name, TelemetryVerbosity.LOW);
  }

  @Test
  void talonGetsArmGainsAsTheyAre() {
    final var smc = new TalonFXWrapper(DeviceCreator.createTalonFX(), DCMotor.getKrakenX60(1),
        config("TalonFX").withFeedforward(new ArmFeedforward(0.1, 0.2, kVVoltsPerRotationPerSecond, kAVoltsPerRotationPerSecondSquared)));
    final Slot0Configs slot0 = new Slot0Configs();
    DeviceCreator.refreshConfig(() -> ((TalonFX) smc.getMotorController()).getConfigurator().refresh(slot0, 1.0));
    // The Talon works per mechanism rotation, as YAMS does, so nothing is converted.
    assertEquals(kVVoltsPerRotationPerSecond, slot0.kV, kTolerance, "kV per rotation per second");
    assertEquals(kAVoltsPerRotationPerSecondSquared, slot0.kA, kTolerance, "kA per rotation per second squared");
  }

  /** Wait for a SPARK's configuration, which is applied asynchronously and stored as floats. */
  private static void assertSpark(double expected, DoubleSupplier actual, String what) throws InterruptedException {
    assertTrue(
        AbsoluteEncoderCases.eventually(() -> Math.abs(actual.getAsDouble() - expected) <= 1e-4 * Math.max(1, Math.abs(expected))),
        what + ": expected " + expected + " on the SPARK but was " + actual.getAsDouble());
  }

  @Test
  void sparkGetsArmGainsPerRpmOfItsEncoder() throws InterruptedException {
    final SparkMax spark = DeviceCreator.createSparkMax();
    new SparkWrapper(spark, DCMotor.getNEO(1),
        config("SPARK").withFeedforward(new ArmFeedforward(0.1, 0.2, kVVoltsPerRotationPerSecond, kAVoltsPerRotationPerSecondSquared)));
    // The SPARK's encoder turns kGearRatio times per mechanism rotation, and counts in RPM.
    final double encoderRpmPerMechanismRotationPerSecond = RotationsPerSecond.of(kGearRatio).in(RPM);
    final var feedForward = spark.configAccessor.closedLoop.feedForward;
    assertSpark(kVVoltsPerRotationPerSecond / encoderRpmPerMechanismRotationPerSecond, feedForward::getkV, "kV per RPM");
    assertSpark(kAVoltsPerRotationPerSecondSquared / encoderRpmPerMechanismRotationPerSecond, feedForward::getkA, "kA per RPM per second");
  }

  /**
   * The Nova's loop runs on the SystemCore. From rest to 1 rotation per second in one 20 ms loop is
   * 50 rotations per second squared, so a kA of 0.05 volts per rotation per second squared asks for
   * 2.5 V. Read per radian, the same kA would ask for 2π times as much.
   */
  @Test
  void systemCoreUsesArmGainsPerRotation() {
    final double kA = 0.05;
    final AtomicReference<Double> commandedVolts = new AtomicReference<>(Double.NaN);
    final var smc = new NovaWrapper(DeviceCreator.createNova(), DCMotor.getNEO(1), config("Nova").withFeedforward(new ArmFeedforward(0, 0, 0, kA))) {
      @Override
      public void setVoltage(Voltage voltage) {
        commandedVolts.set(voltage.in(Volts));
        super.setVoltage(voltage);
      }
    };
    ((SmartMotorControllerTestSubsystem) ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem()).setSMC(smc);
    smc.setVelocity(RotationsPerSecond.of(1));
    commandedVolts.set(Double.NaN);
    smc.iterateClosedLoopController();
    final double accelerationRotationsPerSecondSquared = RotationsPerSecond.of(1).in(RotationsPerSecond) / 0.020;
    assertEquals(kA * accelerationRotationsPerSecondSquared, commandedVolts.get(), 0.1, "volts from kA per rotation per second squared");
  }

  /** An elevator whose drum moves 0.2 m per mechanism rotation. */
  private static final Distance kTravelPerMechanismRotation = Meters.of(0.2);
  private static final double kPVoltsPerMeter = 40;
  private static final double kVVoltsPerMeterPerSecond = 8;
  private static final double kAVoltsPerMeterPerSecondSquared = 0.5;

  /** With an {@link ElevatorFeedforward} the controller is distance based, so its gains are per meter. */
  private static SmartMotorControllerConfig elevatorConfig(String name) {
    return config(name)
        .withMechanismCircumference(kTravelPerMechanismRotation)
        .withClosedLoopController(kPVoltsPerMeter, 0, 0)
        .withFeedforward(new ElevatorFeedforward(0.1, 0.2, kVVoltsPerMeterPerSecond, kAVoltsPerMeterPerSecondSquared));
  }

  private static double mechanismRotationsPerMeter() {
    return Meters.of(1).div(kTravelPerMechanismRotation).in(Value);
  }

  @Test
  void talonGetsElevatorGainsPerMechanismRotation() {
    final var smc = new TalonFXWrapper(DeviceCreator.createTalonFX(), DCMotor.getKrakenX60(1), elevatorConfig("TalonFX elevator"));
    final Slot0Configs slot0 = new Slot0Configs();
    DeviceCreator.refreshConfig(() -> ((TalonFX) smc.getMotorController()).getConfigurator().refresh(slot0, 1.0));
    assertEquals(kPVoltsPerMeter / mechanismRotationsPerMeter(), slot0.kP, kTolerance, "kP per mechanism rotation");
    assertEquals(kVVoltsPerMeterPerSecond / mechanismRotationsPerMeter(), slot0.kV, kTolerance, "kV per mechanism rotation per second");
    assertEquals(kAVoltsPerMeterPerSecondSquared / mechanismRotationsPerMeter(), slot0.kA, kTolerance, "kA per mechanism rotation per second squared");
    assertEquals(0.1, slot0.kS, kTolerance, "kS is in volts");
    assertEquals(0.2, slot0.kG, kTolerance, "kG is in volts");
  }

  @Test
  void sparkGetsElevatorGainsPerEncoderRotation() throws InterruptedException {
    final SparkMax spark = DeviceCreator.createSparkMax();
    new SparkWrapper(spark, DCMotor.getNEO(1), elevatorConfig("SPARK elevator"));
    final double encoderRotationsPerMeter = mechanismRotationsPerMeter() * kGearRatio;
    final double encoderRpmPerMeterPerSecond = RotationsPerSecond.of(encoderRotationsPerMeter).in(RPM);
    // The SPARK's PID output is a fraction of the 12 V of full output; its feedforward is in volts.
    assertSpark(kPVoltsPerMeter / 12 / encoderRotationsPerMeter, spark.configAccessor.closedLoop::getP, "kP per encoder rotation");
    final var feedForward = spark.configAccessor.closedLoop.feedForward;
    assertSpark(kVVoltsPerMeterPerSecond / encoderRpmPerMeterPerSecond, feedForward::getkV, "kV per encoder RPM");
    assertSpark(kAVoltsPerMeterPerSecondSquared / encoderRpmPerMeterPerSecond, feedForward::getkA, "kA per encoder RPM per second");
    assertSpark(0.1, feedForward::getkS, "kS is in volts");
    assertSpark(0.2, feedForward::getkG, "kG is in volts");
  }
}
