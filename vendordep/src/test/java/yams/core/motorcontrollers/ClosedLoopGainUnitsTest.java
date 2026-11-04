// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Value;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.thrifty.nova.Nova;
import com.thrifty.core.MotorConfig.MPGravityType;
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
import yams.core.motorcontrollers.local.A301Wrapper;
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

  /** kA of {@link #systemCoreArmConfig(String)}, in volts per rotation per second squared. */
  private static final double kSystemCoreKa = 0.05;

  /**
   * An arm with only a kA, and a velocity profile fast enough to reach 1 rotation per second in one
   * loop.
   */
  private static SmartMotorControllerConfig systemCoreArmConfig(String name) {
    return config(name)
        .withFeedforward(new ArmFeedforward(0, 0, 0, kSystemCoreKa))
        .withTrapezoidalProfile(RotationsPerSecondPerSecond.of(1000), RotationsPerSecondPerSecond.per(Second).of(1e6));
  }

  /**
   * A closed loop on the SystemCore uses an arm's kA per rotation. From rest to 1 rotation per second
   * in one 20 ms loop is 50 rotations per second squared, so a kA of 0.05 volts per rotation per
   * second squared asks for 2.5 V. Read per radian, the same kA would ask for 2π times as much.
   *
   * @param smc Motor controller made from {@link #systemCoreArmConfig(String)}.
   * @param commandedVolts The voltage the motor controller was last commanded.
   */
  private static void assertSystemCoreUsesArmGainsPerRotation(SmartMotorController smc, AtomicReference<Double> commandedVolts) {
    ((SmartMotorControllerTestSubsystem) ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem()).setSMC(smc);
    smc.setVelocity(RotationsPerSecond.of(1));
    commandedVolts.set(Double.NaN);
    smc.iterateClosedLoopController();
    final double accelerationRotationsPerSecondSquared = RotationsPerSecond.of(1).in(RotationsPerSecond) / 0.020;
    assertEquals(kSystemCoreKa * accelerationRotationsPerSecondSquared, commandedVolts.get(), 0.1, "volts from kA per rotation per second squared");
  }

  /** A Nova's velocity profile runs on the SystemCore. */
  @Test
  void systemCoreUsesArmGainsPerRotation() {
    final AtomicReference<Double> commandedVolts = new AtomicReference<>(Double.NaN);
    final var smc = new NovaWrapper(DeviceCreator.createNova(), DCMotor.getNEO(1), systemCoreArmConfig("Nova")) {
      @Override
      public void setVoltage(Voltage voltage) {
        commandedVolts.set(voltage.in(Volts));
        super.setVoltage(voltage);
      }
    };
    assertSystemCoreUsesArmGainsPerRotation(smc, commandedVolts);
  }

  /** An A301's closed loop always runs on the SystemCore. */
  @Test
  void a301UsesArmGainsPerRotation() {
    final AtomicReference<Double> commandedVolts = new AtomicReference<>(Double.NaN);
    final var smc = new A301Wrapper(DeviceCreator.createA301(), DeviceCreator.forA301(systemCoreArmConfig("A301"))) {
      @Override
      public void setVoltage(Voltage voltage) {
        commandedVolts.set(voltage.in(Volts));
        super.setVoltage(voltage);
      }
    };
    assertSystemCoreUsesArmGainsPerRotation(smc, commandedVolts);
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

  /** Wait for the Nova to report a value it was configured with. */
  private static void assertNova(double expected, DoubleSupplier actual, String what) throws InterruptedException {
    assertTrue(
        AbsoluteEncoderCases.eventually(() -> Math.abs(actual.getAsDouble() - expected) <= 1e-4 * Math.max(1, Math.abs(expected))),
        what + ": expected " + expected + " on the Nova but was " + actual.getAsDouble());
  }

  /**
   * The Nova runs the closed loop in mechanism rotations, with its sensor-to-mechanism ratio, so an
   * arm's gains, integral and derivative too, are sent per rotation as they are. Its feedforward is its motion profile's, with cosine gravity.
   */
  @Test
  void novaGetsArmGainsPerMechanismRotation() throws InterruptedException {
    final Nova nova = DeviceCreator.createNova();
    new NovaWrapper(nova, DCMotor.getNEO(1),
        config("Nova arm")
            .withClosedLoopController(8, 2, 0.5)
            .withFeedforward(new ArmFeedforward(0.1, 0.2, kVVoltsPerRotationPerSecond, kAVoltsPerRotationPerSecondSquared))
            .withTrapezoidalProfile(RotationsPerSecond.of(2), RotationsPerSecondPerSecond.of(4)));
    final var status = nova.status();
    assertNova(kGearRatio, status::getSensorToMechanismRatio, "motor rotations per mechanism rotation");
    assertNova(8, status.Pid0::getP, "kP per mechanism rotation");
    assertNova(2, status.Pid0::getI, "kI per second");
    assertNova(0.5, status.Pid0::getD, "kD per second");
    assertNova(2, status.Mp0::getMaxVel, "maximum velocity in rotations per second");
    assertNova(4, status.Mp0::getMaxAccel, "maximum acceleration in rotations per second squared");
    assertNova(kVVoltsPerRotationPerSecond, status.Mp0::getKV, "kV per rotation per second");
    assertNova(kAVoltsPerRotationPerSecondSquared, status.Mp0::getKA, "kA per rotation per second squared");
    assertNova(0.1, status.Mp0::getKS, "kS is in volts");
    assertNova(0.2, status.Mp0::getKG, "kG is in volts");
    assertEquals(MPGravityType.COSINE, status.Mp0.getGravType(), "an arm's gravity is cosine");
  }

  /** An elevator's gains and feedforward are per meter; the Nova's per mechanism rotation. */
  @Test
  void novaGetsElevatorGainsPerMechanismRotation() throws InterruptedException {
    final Nova nova = DeviceCreator.createNova();
    new NovaWrapper(nova, DCMotor.getNEO(1),
        elevatorConfig("Nova elevator").withTrapezoidalProfile(MetersPerSecond.of(1), MetersPerSecondPerSecond.of(2)));
    final var status = nova.status();
    assertNova(kPVoltsPerMeter / mechanismRotationsPerMeter(), status.Pid0::getP, "kP per mechanism rotation");
    assertNova(1 * mechanismRotationsPerMeter(), status.Mp0::getMaxVel, "maximum velocity in mechanism rotations per second");
    assertNova(kVVoltsPerMeterPerSecond / mechanismRotationsPerMeter(), status.Mp0::getKV, "kV per mechanism rotation per second");
    assertNova(kAVoltsPerMeterPerSecondSquared / mechanismRotationsPerMeter(), status.Mp0::getKA, "kA per mechanism rotation per second squared");
    assertNova(0.1, status.Mp0::getKS, "kS is in volts");
    assertNova(0.2, status.Mp0::getKG, "kG is in volts");
    assertEquals(MPGravityType.CONSTANT, status.Mp0.getGravType(), "an elevator's gravity is constant");
  }
}
