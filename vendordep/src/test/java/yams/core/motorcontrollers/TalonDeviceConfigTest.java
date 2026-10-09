// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import java.util.function.Function;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.system.DCMotor;
import org.wpilib.preferences.Preferences;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.telemetry.SmartMotorControllerCommandRegistry;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.remote.TalonFXSWrapper;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;

/**
 * Tests that the live setters of the Talon wrappers write the same Talon configuration as
 * applyConfig: ramp rates on every ramp, and the motion profile's max acceleration on the jerk when
 * the trapezoidal profile is a velocity profile.
 */
public class TalonDeviceConfigTest {
  private static final double kTolerance = 1e-6;

  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
  }

  @AfterEach
  void endTest() {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  private record Case(String name, Function<SmartMotorControllerConfig, SmartMotorController> create) {
    @Override
    public String toString() {
      return name;
    }
  }

  private static Stream<Case> createControllers() {
    return Stream.of(
        new Case(
            "TalonFX",
            cfg ->
                new TalonFXWrapper(DeviceCreator.createTalonFX(), DCMotor.getKrakenX60(1), cfg)),
        new Case(
            "TalonFXS",
            cfg -> new TalonFXSWrapper(DeviceCreator.createTalonFXS(), DCMotor.getNEO(1), cfg)));
  }

  private static SmartMotorControllerConfig config(String name) {
    return new yams.commands2.config.SmartMotorControllerConfig()
        .withSubsystem(new SmartMotorControllerTestSubsystem())
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withStatorCurrentLimit(Amps.of(40))
        .withZeroPower(MotorMode.BRAKE)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(1, 0, 0)
        .withTelemetry("TalonDeviceConfigTest " + name, TelemetryVerbosity.LOW);
  }

  /** Read a configuration group from the Talon. */
  private static void refresh(SmartMotorController smc, ParentConfiguration configs) {
    final Object motorController = smc.getMotorController();
    if (motorController instanceof TalonFXS talonFXS) {
      DeviceCreator.refreshConfig(() -> refresh(talonFXS, configs));
    } else {
      DeviceCreator.refreshConfig(() -> refresh((TalonFX) motorController, configs));
    }
  }

  private static StatusCode refresh(TalonFXS talon, ParentConfiguration configs) {
    if (configs instanceof OpenLoopRampsConfigs c) {
      return talon.getConfigurator().refresh(c, 1.0);
    } else if (configs instanceof ClosedLoopRampsConfigs c) {
      return talon.getConfigurator().refresh(c, 1.0);
    }
    return talon.getConfigurator().refresh((MotionMagicConfigs) configs, 1.0);
  }

  private static StatusCode refresh(TalonFX talon, ParentConfiguration configs) {
    if (configs instanceof OpenLoopRampsConfigs c) {
      return talon.getConfigurator().refresh(c, 1.0);
    } else if (configs instanceof ClosedLoopRampsConfigs c) {
      return talon.getConfigurator().refresh(c, 1.0);
    }
    return talon.getConfigurator().refresh((MotionMagicConfigs) configs, 1.0);
  }

  private static void close(SmartMotorController smc) {
    SmartMotorControllerTestSubsystem subsys =
        (SmartMotorControllerTestSubsystem)
            ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem();
    SmartMotorControllerCommandRegistry.removeCommands(subsys);
    CommandScheduler.getInstance().unregisterSubsystem(subsys);
    subsys.close();
    smc.close();
    DeviceCreator.silence(smc);
    final Object motorController = smc.getMotorController();
    if (motorController instanceof TalonFXS talonFXS) {
      talonFXS.getConfigurator().apply(new TalonFXSConfiguration());
      talonFXS.close();
    } else if (motorController instanceof TalonFX talonFX) {
      talonFX.getConfigurator().apply(new TalonFXConfiguration());
      talonFX.close();
    }
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("createControllers")
  void rampRateSettersSetEveryRamp(Case testCase) {
    final String name = testCase.name();
    final SmartMotorController smc = testCase.create().apply(config(name));
    ((SmartMotorControllerTestSubsystem)
            ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem())
        .setSMC(smc);
    try {
      smc.setOpenLoopRampRate(Seconds.of(0.3));
      smc.setClosedLoopRampRate(Seconds.of(0.4));

      final OpenLoopRampsConfigs openLoop = new OpenLoopRampsConfigs();
      refresh(smc, openLoop);
      assertEquals(0.3, openLoop.DutyCycleOpenLoopRampPeriod, kTolerance, name + ": duty cycle");
      assertEquals(0.3, openLoop.VoltageOpenLoopRampPeriod, kTolerance, name + ": voltage");
      assertEquals(0.3, openLoop.TorqueOpenLoopRampPeriod, kTolerance, name + ": torque");

      final ClosedLoopRampsConfigs closedLoop = new ClosedLoopRampsConfigs();
      refresh(smc, closedLoop);
      assertEquals(0.4, closedLoop.DutyCycleClosedLoopRampPeriod, kTolerance, name + ": duty cycle");
      assertEquals(0.4, closedLoop.VoltageClosedLoopRampPeriod, kTolerance, name + ": voltage");
      assertEquals(0.4, closedLoop.TorqueClosedLoopRampPeriod, kTolerance, name + ": torque");
    } finally {
      close(smc);
    }
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("createControllers")
  void maxAccelerationWithVelocityProfileSetsJerk(Case testCase) {
    final String name = testCase.name();
    // A velocity trapezoidal profile: its max velocity is the max acceleration, and its max
    // acceleration is the max jerk.
    final SmartMotorController smc =
        testCase
            .create()
            .apply(
                config(name)
                    .withTrapezoidalProfile(
                        RotationsPerSecondPerSecond.of(10),
                        RotationsPerSecondPerSecond.per(Second).of(100)));
    ((SmartMotorControllerTestSubsystem)
            ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem())
        .setSMC(smc);
    try {
      final MotionMagicConfigs motionMagic = new MotionMagicConfigs();
      refresh(smc, motionMagic);
      assertEquals(10, motionMagic.MotionMagicAcceleration, kTolerance, name + ": applied accel");
      assertEquals(100, motionMagic.MotionMagicJerk, kTolerance, name + ": applied jerk");

      smc.setMotionProfileMaxAcceleration(RotationsPerSecondPerSecond.of(7));
      refresh(smc, motionMagic);
      assertEquals(
          10, motionMagic.MotionMagicAcceleration, kTolerance, name + ": acceleration unchanged");
      assertEquals(7, motionMagic.MotionMagicJerk, kTolerance, name + ": jerk set");
    } finally {
      close(smc);
    }
  }
}
