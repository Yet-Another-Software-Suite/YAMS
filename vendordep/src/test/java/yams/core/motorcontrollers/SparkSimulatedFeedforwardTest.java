// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.preferences.Preferences;
import org.wpilib.simulation.SimDeviceSim;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.telemetry.SmartMotorControllerCommandRegistry;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;

/**
 * Tests the feedforward the {@link SparkWrapper} gives a simulated SPARK as an arbitrary feedforward.
 * REVLib's MAXMotion does not follow its profile in simulation, so YAMS runs the profile and holds
 * each point with position control. In position control the SPARK applies kS and kG itself, but not
 * kV or kA; and its simulated kCos works from the setpoint, read as degrees, rather than the arm's
 * angle. The arbitrary feedforward has to make up exactly the difference.
 */
public class SparkSimulatedFeedforwardTest {
  /** 12 motor rotations per mechanism rotation. */
  private static final MechanismGearing kGearing =
      new MechanismGearing(GearBox.fromReductionStages(3, 4));

  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
  }

  @AfterEach
  void endTest() {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  private static SmartMotorControllerConfig baseConfig(String name) {
    return new yams.commands2.config.SmartMotorControllerConfig()
        .withSubsystem(new SmartMotorControllerTestSubsystem())
        .withGearing(kGearing)
        .withStatorCurrentLimit(Amps.of(40))
        .withZeroPower(MotorMode.BRAKE)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(0, 0, 0)
        .withMomentOfInertia(KilogramSquareMeters.of(0.02))
        .withStartingPosition(Rotations.of(0))
        .withTelemetry(
            "SparkSimulatedFeedforwardTest " + name,
            yams.core.telemetry.enums.TelemetryVerbosity.LOW);
  }

  private static SmartMotorController create(SparkMax sparkMax, SmartMotorControllerConfig config) {
    final SmartMotorController smc = new SparkWrapper(sparkMax, DCMotor.getNEO(1), config);
    ((SmartMotorControllerTestSubsystem)
            ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem())
        .setSMC(smc);
    smc.setupSimulation();
    return smc;
  }

  private static void close(SmartMotorController smc, SparkMax sparkMax) {
    SmartMotorControllerTestSubsystem subsys =
        (SmartMotorControllerTestSubsystem)
            ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem();
    SmartMotorControllerCommandRegistry.removeCommands(subsys);
    CommandScheduler.getInstance().unregisterSubsystem(subsys);
    subsys.close();
    smc.close();
    DeviceCreator.silence(smc);
    sparkMax.configure(
        new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    sparkMax.close();
  }

  /** The arbitrary feedforward, in volts, last sent to the simulated SPARK. */
  private static double arbitraryFeedforward(SparkMax sparkMax) {
    return new SimDeviceSim(
            "SPARK MAX [" + sparkMax.getCanPort().value + "," + sparkMax.getDeviceId() + "]")
        .getDouble("Arbitrary Feedforward")
        .get();
  }

  /** While the profile moves the mechanism, kS is left to the SPARK, not sent again. */
  @Test
  void staticGainIsNotSentTwice() {
    final SparkMax sparkMax = DeviceCreator.createSparkMax();
    final SmartMotorController smc =
        create(
            sparkMax,
            baseConfig("kS")
                .withFeedforward(new SimpleMotorFeedforward(0.5, 0, 0))
                .withTrapezoidalProfile(RotationsPerSecond.of(2), RotationsPerSecondPerSecond.of(4)));
    try {
      smc.setPosition(Rotations.of(1));
      for (int i = 0; i < 5; i++) {
        smc.simIterate();
      }
      assertEquals(
          0,
          arbitraryFeedforward(sparkMax),
          1e-6,
          "kS is applied by the SPARK in position control, so it must not be in the arbitrary feedforward");
    } finally {
      close(smc, sparkMax);
    }
  }

  /** While the profile accelerates, kA times the profile's acceleration is sent. */
  @Test
  void accelerationGainIsSent() {
    final SparkMax sparkMax = DeviceCreator.createSparkMax();
    final double kA = 0.1;
    final double maxAcceleration = 4;
    final SmartMotorController smc =
        create(
            sparkMax,
            baseConfig("kA")
                .withFeedforward(new SimpleMotorFeedforward(0, 0, kA))
                .withTrapezoidalProfile(
                    RotationsPerSecond.of(2), RotationsPerSecondPerSecond.of(maxAcceleration)));
    try {
      // The profile starts from rest at the full acceleration, toward a goal far enough away that it
      // is still accelerating after one step.
      smc.setPosition(Rotations.of(5));
      smc.simIterate();
      assertEquals(
          kA * maxAcceleration,
          arbitraryFeedforward(sparkMax),
          1e-3,
          "kA times the profile's acceleration should be in the arbitrary feedforward");
    } finally {
      close(smc, sparkMax);
    }
  }

  /**
   * Holding an arm, the arbitrary feedforward carries the gravity, kG times the cosine of its angle,
   * and the SPARK's kCos is 0 so the gravity is not added twice.
   */
  @Test
  void armGravityIsSentInPlaceOfKCos() throws InterruptedException {
    final SparkMax sparkMax = DeviceCreator.createSparkMax();
    final double kG = 1.2;
    final SmartMotorController smc =
        create(
            sparkMax,
            baseConfig("arm")
                .withFeedforward(new ArmFeedforward(0, kG, 0))
                .withStartingPosition(Degrees.of(60)));
    try {
      assertTrue(
          AbsoluteEncoderCases.eventually(
              () -> sparkMax.configAccessor.closedLoop.feedForward.getkCos() == 0),
          "kCos should be 0 on a simulated SPARK, but was "
              + sparkMax.configAccessor.closedLoop.feedForward.getkCos());
      smc.setPosition(Degrees.of(60));
      smc.simIterate();
      // The gravity is worked out before the simulation step moves the arm, so allow for that step.
      final double expected = kG * Math.cos(smc.getMechanismPosition().in(org.wpilib.units.Units.Radians));
      assertEquals(
          expected,
          arbitraryFeedforward(sparkMax),
          0.02,
          "The arm's gravity, kG times the cosine of its angle, should be in the arbitrary feedforward");
      assertEquals(kG * 0.5, expected, 0.05, "The arm should still be near 60 degrees");
    } finally {
      close(smc, sparkMax);
    }
  }
}
