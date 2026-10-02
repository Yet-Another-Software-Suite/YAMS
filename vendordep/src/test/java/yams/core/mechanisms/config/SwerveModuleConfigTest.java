// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.config;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.MetersPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.system.DCMotor;
import org.wpilib.preferences.Preferences;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;

/**
 * Tests {@link SwerveModuleConfig#getOptimizedState}: optimization turns a wheel at most 90 degrees
 * from where it was last sent and reverses the drive instead, a wheel that is still turning is not
 * reversed back and forth by a target that keeps moving, and a wheel commanded too slowly to steer
 * by holds where it is.
 */
public class SwerveModuleConfigTest {
  private SmartMotorController drive;
  private SmartMotorController azimuth;
  private SwerveModuleConfig config;

  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
    drive = spark("SwerveModuleConfigTest drive");
    azimuth = spark("SwerveModuleConfigTest azimuth");
    config = new SwerveModuleConfig(drive, azimuth).withOptimization(true).withMinimumVelocity(MetersPerSecond.of(0.1));
  }

  @AfterEach
  void endTest() {
    close(drive);
    close(azimuth);
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  private static SmartMotorController spark(String name) {
    return new SparkWrapper(DeviceCreator.createSparkMax(), DCMotor.getNEO(1),
        new yams.commands2.config.SmartMotorControllerConfig()
            .withSubsystem(new SmartMotorControllerTestSubsystem())
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            .withClosedLoopController(1, 0, 0)
            .withTelemetry(name, yams.core.telemetry.enums.TelemetryVerbosity.LOW));
  }

  private static void close(SmartMotorController smc) {
    CommandScheduler.getInstance().unregisterSubsystem(((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem());
    smc.close();
    final SparkMax spark = (SparkMax) smc.getMotorController();
    spark.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    spark.close();
  }

  /** Put the wheel at an angle. */
  private void wheelAt(double degrees) {
    azimuth.setEncoderPosition(Degrees.of(degrees));
  }

  private SwerveModuleVelocity optimize(double velocity, double degrees) {
    return config.getOptimizedState(new SwerveModuleVelocity(velocity, Rotation2d.fromDegrees(degrees)));
  }

  private static double degreesBetween(Rotation2d a, Rotation2d b) {
    return Math.abs(a.minus(b).getDegrees());
  }

  @Test
  void turnsAtMostAQuarterTurnAndReversesTheDrive() {
    wheelAt(0);
    final SwerveModuleVelocity state = optimize(1, 150);
    assertEquals(-30, state.angle.getDegrees(), 1e-6, "turns 30 degrees the other way");
    assertEquals(-1, state.velocity, 1e-9, "and drives backwards");
  }

  @Test
  void targetThatTurnsAroundReversesTheDrive() {
    wheelAt(0);
    optimize(1, 0);
    final SwerveModuleVelocity state = optimize(1, 180);
    assertEquals(0, state.angle.getDegrees(), 1e-6, "the wheel stays where it is");
    assertEquals(-1, state.velocity, 1e-9, "and drives the other way");
  }

  @Test
  void turningWheelIsNotReversedBackAndForth() {
    // The wheel is still at 0 degrees, turning toward a target near 90 degrees from it that moves
    // from loop to loop, as when a path starts off in a new direction. Chosen by the wheel's
    // measured angle, the target's two orientations would swap every loop.
    wheelAt(0);
    Rotation2d last = optimize(1, 110).angle;
    for (double target : new double[] {70, 112, 68, 110, 72, 88}) {
      final Rotation2d commanded = optimize(1, target).angle;
      assertTrue(degreesBetween(commanded, last) < 90,
          "commanded " + commanded.getDegrees() + " degrees right after " + last.getDegrees() + " for a target of " + target + " degrees");
      last = commanded;
    }
  }

  @Test
  void tooSlowToSteerByHoldsTheWheel() {
    wheelAt(30);
    final SwerveModuleVelocity state = optimize(0.05, 120);
    assertEquals(30, state.angle.getDegrees(), 1e-6, "the wheel holds where it is");
    assertEquals(0, state.velocity, 1e-9, "and does not drive");
    // The next state is optimized from where the wheel was held: 140 degrees is 110 degrees from it,
    // so the wheel turns 70 degrees the other way and drives backwards.
    final SwerveModuleVelocity next = optimize(1, 140);
    assertEquals(-40, next.angle.getDegrees(), 1e-6, "turns to -40 degrees");
    assertEquals(-1, next.velocity, 1e-9, "and drives backwards");
  }
}
