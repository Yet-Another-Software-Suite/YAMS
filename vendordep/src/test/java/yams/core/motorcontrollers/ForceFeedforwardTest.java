// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Newtons;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.system.DCMotor;
import org.wpilib.preferences.Preferences;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Force;
import yams.commands2.telemetry.SmartMotorControllerCommandRegistry;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.PeriodicScheduler;
import yams.helpers.SmartMotorControllerTestSubsystem;

/* CTRE has not published a Phoenix6 build compatible with wpilib 2027-alpha-7; re-enable once
available.
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import yams.core.motorcontrollers.remote.TalonFXSWrapper;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
*/

/**
 * Tests for the drive-wheel feedforward {@link Force} added to {@link
 * SmartMotorController#setVelocity( org.wpilib.units.measure.AngularVelocity, Force)} (and its
 * {@code LinearVelocity} counterpart), e.g. from a PathPlanner set-point generator.
 *
 * <p>Covers three layers: the pure physics conversion in {@link
 * SmartMotorControllerConfig#convertToVoltage} / {@link
 * SmartMotorControllerConfig#convertToCurrent} against WPILib's own {@link DCMotor} model, the
 * {@code setpointFeedforwardForce} bookkeeping used for telemetry, and (across every vendor
 * wrapper) that a feedforward Force alone with the closed-loop gains zeroed out actually
 * commands real motor output in simulation.
 */
public class ForceFeedforwardTest {
  private static final double WHEEL_RADIUS_METERS = 0.05;
  private static final double GEAR_RATIO = 4.0;

  private static SmartMotorControllerConfig configWithGearingAndWheel() {
    return new yams.commands2.config.SmartMotorControllerConfig()
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(GEAR_RATIO)))
        .withMechanismCircumference(Meters.of(2 * Math.PI * WHEEL_RADIUS_METERS));
  }

  @Test
  void testConvertToVoltageMatchesDCMotorPhysics() {
    SmartMotorControllerConfig config = configWithGearingAndWheel();
    DCMotor motor = DCMotor.getNEO(1);
    Force feedforwardForce = Newtons.of(10);
    AngularVelocity mechanismVelocity = RotationsPerSecond.of(2);

    double expectedRotorTorqueNm = feedforwardForce.in(Newtons) *WHEEL_RADIUS_METERS / GEAR_RATIO;
    double expectedRotorVelocityRadS = mechanismVelocity.in(RadiansPerSecond) *GEAR_RATIO;
    double expectedVoltage = motor.getVoltage(expectedRotorTorqueNm, expectedRotorVelocityRadS);

    assertEquals(expectedVoltage,
        config.convertToVoltage(motor, mechanismVelocity, feedforwardForce).in(Volts), 1e-9,
        "convertToVoltage should match DCMotor's own torque/speed voltage model.");
  }

  @Test
  void testConvertToCurrentMatchesDCMotorPhysics() {
    SmartMotorControllerConfig config = configWithGearingAndWheel();
    DCMotor motor = DCMotor.getKrakenX60(1);
    Force feedforwardForce = Newtons.of(25);

    double expectedRotorTorqueNm = feedforwardForce.in(Newtons) *WHEEL_RADIUS_METERS / GEAR_RATIO;
    double expectedCurrent = motor.getCurrent(expectedRotorTorqueNm);

    assertEquals(expectedCurrent, config.convertToCurrent(motor, feedforwardForce).in(Amps), 1e-9,
        "convertToCurrent should match DCMotor's own torque/current model.");
  }

  private static SmartMotorControllerConfig baseSmcConfig() {
    return configWithGearingAndWheel()
        .withClosedLoopController(0, 0, 0)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withControlMode(ControlMode.CLOSED_LOOP);
  }

  private static int offset = 0;

  private static Stream<Arguments> createConfigs() {
    offset += 1;
    SmartMotorControllerConfig cfg = baseSmcConfig();
    return Stream.of(
        Arguments.of(new SparkWrapper(DeviceCreator.createSparkMax(), DCMotor.getNEO(1),
            ((yams.commands2.config.SmartMotorControllerConfig) cfg.clone())
                .withSubsystem(new SmartMotorControllerTestSubsystem())
                .withTelemetry(
                    "ForceFeedforwardTest SparkMax(" + offset + ")", TelemetryVerbosity.LOW)))
        /* CTRE has not published a Phoenix6 build compatible with wpilib 2027-alpha-7.
        ,
        Arguments.of(
            new TalonFXWrapper(
                DeviceCreator.createTalonFX(),
                DCMotor.getKrakenX60(1),
                cfg.clone()
                    .withSubsystem(new SmartMotorControllerTestSubsystem())
                    .withTelemetry(
                        "ForceFeedforwardTest TalonFX(" + offset + ")", TelemetryVerbosity.LOW))),
        Arguments.of(
            new TalonFXSWrapper(
                DeviceCreator.createTalonFXS(),
                DCMotor.getNEO(1),
                cfg.clone()
                    .withSubsystem(new SmartMotorControllerTestSubsystem())
                    .withTelemetry(
                        "ForceFeedforwardTest TalonFXS(" + offset + ")", TelemetryVerbosity.LOW)))
        */
    );
  }

  private static void closeSmc(SmartMotorController smc) {
    SmartMotorControllerTestSubsystem subsys =
        (SmartMotorControllerTestSubsystem) ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem();
    SmartMotorControllerCommandRegistry.removeCommands(subsys);
    CommandScheduler.getInstance().unregisterSubsystem(subsys);
    subsys.close();
    smc.close();

    Object motorController = smc.getMotorController();
    if (motorController instanceof SparkMax) {
      ((SparkMax) motorController).close();
    } else if (motorController instanceof SparkFlex) {
      ((SparkFlex) motorController).close();
    }
    /* CTRE has not published a Phoenix6 build compatible with wpilib 2027-alpha-7.
    else if (motorController instanceof TalonFXS)
    {
      ((TalonFXS) motorController).close();
    } else if (motorController instanceof TalonFX)
    {
      ((TalonFX) motorController).close();
    }
    */
  }

  @ParameterizedTest
  @MethodSource("createConfigs")
  void testSetpointFeedforwardForceOptionalTracking(SmartMotorController smc) {
    try {
      ((SmartMotorControllerTestSubsystem) ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem()).setSMC(smc);

      smc.setVelocity(RPM.of(500), Newtons.of(5));
      assertTrue(smc.getSetpointFeedforwardForce().isPresent(),
          smc.getName() + ": setVelocity(velocity, force) should record the feedforward force.");
      assertEquals(5.0, smc.getSetpointFeedforwardForce().get().in(Newtons), 1e-9,
          smc.getName() + ": the recorded feedforward force should match what was supplied.");

      smc.setVelocity(RPM.of(500));
      assertTrue(smc.getSetpointFeedforwardForce().isEmpty(),
          smc.getName()
              + ": setVelocity(velocity) with no force should clear the feedforward force, "
              + "primarily so telemetry reports it as unset.");

      // A null Force falls back to plain velocity control (as if the no-force overload was called)
      // rather
      // than attempting to convert a fabricated zero Force, so it must not record a feedforward
      // force either.
      smc.setVelocity(RPM.of(500), Newtons.of(5));
      assertTrue(smc.getSetpointFeedforwardForce().isPresent());
      smc.setVelocity(RPM.of(500), null);
      assertTrue(smc.getSetpointFeedforwardForce().isEmpty(),
          smc.getName()
              + ": setVelocity(velocity, null) should fall back to plain velocity control and "
              + "clear the feedforward force.");
    } finally {
      closeSmc(smc);
    }
  }

  @ParameterizedTest
  @MethodSource("createConfigs")
  void testFeedforwardForceProducesMotorOutput(SmartMotorController smc) {
    try {
      SmartMotorControllerTestSubsystem subsys =
          (SmartMotorControllerTestSubsystem) ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem();
      subsys.setSMC(smc);
      subsys.testRunning = true;
      smc.setupSimulation();

      // Zero velocity setpoint with zero PID gains configured: any commanded output can only come
      // from
      // whatever feedforward Force is supplied below, isolating its effect from closed-loop
      // correction.
      try (PeriodicScheduler scheduler = new PeriodicScheduler()) {
        scheduler.addPeriodic(() -> {
          smc.setVelocity(RPM.of(0), Newtons.of(0));
          smc.simIterate();
        }, Milliseconds.of(20));
        scheduler.runFor(Seconds.of(1));
      }
      double zeroForceDutyCycle = Math.abs(smc.getDutyCycle());

      try (PeriodicScheduler scheduler = new PeriodicScheduler()) {
        scheduler.addPeriodic(() -> {
          smc.setVelocity(RPM.of(0), Newtons.of(8));
          smc.simIterate();
        }, Milliseconds.of(20));
        scheduler.runFor(Seconds.of(1));
      }
      double forceDutyCycle = Math.abs(smc.getDutyCycle());

      System.out.println(smc.getName() + ": zero-force duty cycle=" + zeroForceDutyCycle
          + ", with-force duty cycle=" + forceDutyCycle);

      assertEquals(0.0, zeroForceDutyCycle, 1e-6,
          smc.getName() + ": expected no commanded output with zero PID gains and zero feedforward "
              + "force.");
      assertTrue(forceDutyCycle > 0.01,
          smc.getName() + ": expected the feedforward Force to produce commanded output even with "
              + "zero PID gains.");
    } finally {
      closeSmc(smc);
    }
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
}
