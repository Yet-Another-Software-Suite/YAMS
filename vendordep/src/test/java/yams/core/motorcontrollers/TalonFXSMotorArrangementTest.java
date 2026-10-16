// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
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
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.telemetry.SmartMotorControllerCommandRegistry;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.remote.TalonFXSWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;

/** Tests that the TalonFXSWrapper commutates each motor it supports with the right arrangement. */
public class TalonFXSMotorArrangementTest {
  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
  }

  @AfterEach
  void endTest() {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  private static Stream<Arguments> motors() {
    return Stream.of(
        Arguments.of("Minion", DCMotor.getMinion(1), MotorArrangementValue.Minion_JST, AdvancedHallSupportValue.Enabled),
        Arguments.of("NEO", DCMotor.getNEO(1), MotorArrangementValue.NEO_JST, AdvancedHallSupportValue.Disabled),
        Arguments.of("NEO 550", DCMotor.getNeo550(1), MotorArrangementValue.NEO550_JST, AdvancedHallSupportValue.Disabled),
        Arguments.of("NEO Vortex", DCMotor.getNeoVortex(1), MotorArrangementValue.VORTEX_JST, AdvancedHallSupportValue.Disabled),
        Arguments.of("Thrifty Pulsar", Motors.getThriftyPulsar(1), MotorArrangementValue.NEO_JST, AdvancedHallSupportValue.Disabled),
        Arguments.of("2 Thrifty Pulsars", Motors.getThriftyPulsar(2), MotorArrangementValue.NEO_JST, AdvancedHallSupportValue.Disabled));
  }

  private static SmartMotorControllerConfig config(String name) {
    return new SmartMotorControllerConfig()
        .withSubsystem(new SmartMotorControllerTestSubsystem())
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        // A NEO 550 needs a stator current limit of at most 40 A
        .withStatorCurrentLimit(Amps.of(40))
        .withZeroPower(MotorMode.BRAKE)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(1, 0, 0)
        .withTelemetry("TalonFXSMotorArrangementTest " + name, TelemetryVerbosity.LOW);
  }

  private static void close(SmartMotorController smc) {
    SmartMotorControllerTestSubsystem subsys =
        (SmartMotorControllerTestSubsystem) ((SmartMotorControllerConfig) smc.getConfig()).getSubsystem();
    SmartMotorControllerCommandRegistry.removeCommands(subsys);
    CommandScheduler.getInstance().unregisterSubsystem(subsys);
    subsys.close();
    smc.close();
    DeviceCreator.silence(smc);
    TalonFXS talon = (TalonFXS) smc.getMotorController();
    talon.getConfigurator().apply(new TalonFXSConfiguration());
    talon.close();
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("motors")
  void commutatesEachMotor(String name, DCMotor motor, MotorArrangementValue arrangement, AdvancedHallSupportValue advancedHall) {
    final TalonFXS talon = DeviceCreator.createTalonFXS();
    final SmartMotorController smc = new TalonFXSWrapper(talon, motor, config(name));
    ((SmartMotorControllerTestSubsystem) ((SmartMotorControllerConfig) smc.getConfig()).getSubsystem()).setSMC(smc);
    try {
      final CommutationConfigs commutation = new CommutationConfigs();
      DeviceCreator.refreshConfig(() -> talon.getConfigurator().refresh(commutation, 1.0));
      assertEquals(arrangement, commutation.MotorArrangement, name);
      assertEquals(advancedHall, commutation.AdvancedHallSupport, name);
    } finally {
      close(smc);
    }
  }

  /** YAMG builds the Pulsar's DCMotor itself, and the wrapper only recognizes it if it matches exactly. */
  @Test
  void recognizesThePulsarYamgGenerates() {
    for (int n = 1; n <= 4; n++) {
      final DCMotor yamg = new DCMotor(12.0, 3.1, 189.0, 2.7, 7500 * 2 * Math.PI / 60, n);
      final DCMotor yams = Motors.getThriftyPulsar(n);
      assertTrue(
          yamg.stallTorque == yams.stallTorque && yamg.stallCurrent == yams.stallCurrent && yamg.freeCurrent == yams.freeCurrent
              && yamg.freeSpeed == yams.freeSpeed && yamg.Kt == yams.Kt && yamg.Kv == yams.Kv && yamg.nominalVoltage == yams.nominalVoltage,
          n + " Pulsars");
    }
  }
}
