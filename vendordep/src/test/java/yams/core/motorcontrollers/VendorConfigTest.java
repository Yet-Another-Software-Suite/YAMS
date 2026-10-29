// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Milliseconds;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.controller.SimpleMotorFeedforward;
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
 * Tests which settings a TalonFX or TalonFXS ends up with, with
 * {@code withResetPreviousConfig(false)}, for a setting YAMS does not manage (the duty cycle neutral
 * deadband):
 *
 * <ul>
 *   <li>Without a vendor config, the setting saved on the device is kept.
 *   <li>With a vendor config, the vendor config's setting replaces the one saved on the device.
 * </ul>
 */
public class VendorConfigTest {
  private static final double kSavedDeadband = 0.05;
  private static final double kVendorDeadband = 0.1;

  private static int offset = 0;

  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
    offset++;
  }

  @AfterEach
  void endTest() {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  private static SmartMotorControllerConfig config(String name) {
    return new SmartMotorControllerConfig()
        .withSubsystem(new SmartMotorControllerTestSubsystem())
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withStatorCurrentLimit(Amps.of(40))
        .withZeroPower(MotorMode.BRAKE)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withSimulationPeriod(Milliseconds.of(10))
        .withClosedLoopController(8, 0, 0)
        .withFeedforward(new SimpleMotorFeedforward(0, 1.5))
        .withResetPreviousConfig(false)
        .withTelemetry(name + "[" + offset + "]", TelemetryVerbosity.LOW);
  }

  /** Save a deadband on the device, as if set earlier in Tuner X. */
  private static void saveDeadband(Object talon) {
    if (talon instanceof TalonFXS talonFXS) {
      final TalonFXSConfiguration saved = new TalonFXSConfiguration();
      saved.MotorOutput.DutyCycleNeutralDeadband = kSavedDeadband;
      talonFXS.getConfigurator().apply(saved);
    } else {
      final TalonFXConfiguration saved = new TalonFXConfiguration();
      saved.MotorOutput.DutyCycleNeutralDeadband = kSavedDeadband;
      ((TalonFX) talon).getConfigurator().apply(saved);
    }
  }

  private static double deviceDeadband(Object talon) {
    final MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    if (talon instanceof TalonFXS talonFXS) {
      DeviceCreator.refreshConfig(() -> talonFXS.getConfigurator().refresh(motorOutput, 1.0));
    } else {
      DeviceCreator.refreshConfig(() -> ((TalonFX) talon).getConfigurator().refresh(motorOutput, 1.0));
    }
    return motorOutput.DutyCycleNeutralDeadband;
  }

  private static SmartMotorController create(boolean talonFXS, boolean vendorConfig) {
    final String name = (talonFXS ? "TalonFXS" : "TalonFX") + (vendorConfig ? " vendor config" : " saved config");
    final SmartMotorControllerConfig cfg = config("VendorConfigTest " + name);
    if (talonFXS) {
      final TalonFXS talon = DeviceCreator.createTalonFXS();
      saveDeadband(talon);
      if (vendorConfig) {
        final TalonFXSConfiguration vendor = new TalonFXSConfiguration();
        vendor.MotorOutput.DutyCycleNeutralDeadband = kVendorDeadband;
        cfg.withVendorConfig(vendor);
      }
      return new TalonFXSWrapper(talon, DCMotor.getNEO(1), cfg);
    }
    final TalonFX talon = DeviceCreator.createTalonFX();
    saveDeadband(talon);
    if (vendorConfig) {
      final TalonFXConfiguration vendor = new TalonFXConfiguration();
      vendor.MotorOutput.DutyCycleNeutralDeadband = kVendorDeadband;
      cfg.withVendorConfig(vendor);
    }
    return new TalonFXWrapper(talon, DCMotor.getKrakenX60(1), cfg);
  }

  private static void close(SmartMotorController smc) {
    final SmartMotorControllerTestSubsystem subsys =
        (SmartMotorControllerTestSubsystem) ((SmartMotorControllerConfig) smc.getConfig()).getSubsystem();
    SmartMotorControllerCommandRegistry.removeCommands(subsys);
    CommandScheduler.getInstance().unregisterSubsystem(subsys);
    subsys.close();
    smc.close();
    DeviceCreator.silence(smc);
    if (smc.getMotorController() instanceof TalonFXS talonFXS) {
      talonFXS.getConfigurator().apply(new TalonFXSConfiguration());
      talonFXS.close();
    } else if (smc.getMotorController() instanceof TalonFX talonFX) {
      talonFX.getConfigurator().apply(new TalonFXConfiguration());
      talonFX.close();
    }
  }

  @ParameterizedTest(name = "TalonFXS: {0}")
  @ValueSource(booleans = {false, true})
  void keepsTheSavedSettingsWithoutAVendorConfig(boolean talonFXS) {
    final SmartMotorController smc = create(talonFXS, false);
    try {
      assertEquals(kSavedDeadband, deviceDeadband(smc.getMotorController()), 1e-9, "after construction");
      smc.applyConfig(smc.getConfig());
      assertEquals(kSavedDeadband, deviceDeadband(smc.getMotorController()), 1e-9, "after applying the config again");
    } finally {
      close(smc);
    }
  }

  @ParameterizedTest(name = "TalonFXS: {0}")
  @ValueSource(booleans = {false, true})
  void vendorConfigWinsOverTheSavedSettings(boolean talonFXS) {
    final SmartMotorController smc = create(talonFXS, true);
    try {
      assertEquals(kVendorDeadband, deviceDeadband(smc.getMotorController()), 1e-9, "after construction");
      smc.applyConfig(smc.getConfig());
      assertEquals(kVendorDeadband, deviceDeadband(smc.getMotorController()), 1e-9, "after applying the config again");
    } finally {
      close(smc);
    }
  }
}
