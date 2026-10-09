// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.thrifty.nova.Nova;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.preferences.Preferences;
import org.wpilib.units.measure.Voltage;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.telemetry.SmartMotorControllerCommandRegistry;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.math.LQRConfig;
import yams.core.math.LQRController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.NovaWrapper;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.motorcontrollers.remote.TalonFXSWrapper;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;

/**
 * Tests that an {@link ArmFeedforward} contributes to the output of the YAMS closed loop controller
 * when no motion profile is configured (here with an LQR, which always runs in YAMS).
 */
public class ArmFeedforwardTest {
  private static final MechanismGearing kGearing =
      new MechanismGearing(GearBox.fromReductionStages(3, 4));
  private static final double kG = 3.0;

  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
  }

  @AfterEach
  void endTest() {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  /** The last voltage the closed loop controller commanded, captured from setVoltage. */
  private static final AtomicReference<Double> commandedVolts = new AtomicReference<>(Double.NaN);

  private record Case(
      String name, Function<SmartMotorControllerConfig, SmartMotorController> create) {
    @Override
    public String toString() {
      return name;
    }
  }

  private static Stream<Case> createControllers() {
    return Stream.of(
        new Case(
            "SparkMax",
            cfg ->
                new SparkWrapper(DeviceCreator.createSparkMax(), DCMotor.getNEO(1), cfg) {
                  @Override
                  public void setVoltage(Voltage voltage) {
                    commandedVolts.set(voltage.in(Volts));
                    super.setVoltage(voltage);
                  }
                }),
        new Case(
            "TalonFX",
            cfg ->
                new TalonFXWrapper(DeviceCreator.createTalonFX(), DCMotor.getKrakenX60(1), cfg) {
                  @Override
                  public void setVoltage(Voltage voltage) {
                    commandedVolts.set(voltage.in(Volts));
                    super.setVoltage(voltage);
                  }
                }),
        new Case(
            "TalonFXS",
            cfg ->
                new TalonFXSWrapper(DeviceCreator.createTalonFXS(), DCMotor.getNEO(1), cfg) {
                  @Override
                  public void setVoltage(Voltage voltage) {
                    commandedVolts.set(voltage.in(Volts));
                    super.setVoltage(voltage);
                  }
                }),
        new Case(
            "Nova",
            cfg ->
                new NovaWrapper(DeviceCreator.createNova(), DCMotor.getNEO(1), cfg) {
                  @Override
                  public void setVoltage(Voltage voltage) {
                    commandedVolts.set(voltage.in(Volts));
                    super.setVoltage(voltage);
                  }
                }));
  }

  private static LQRController lqr() {
    return new LQRController(
        new LQRConfig(DCMotor.getNEO(1), kGearing, KilogramSquareMeters.of(0.02))
            .withArm(
                Radians.of(0.01),
                RadiansPerSecond.of(0.5),
                Radians.of(0.05),
                RadiansPerSecond.of(0.5),
                Radians.of(0.01))
            .withControlEffort(Volts.of(12))
            .withMaxVoltage(Volts.of(12)));
  }

  private static SmartMotorControllerConfig config(String name) {
    return new yams.commands2.config.SmartMotorControllerConfig()
        .withSubsystem(new SmartMotorControllerTestSubsystem())
        .withGearing(kGearing)
        .withStatorCurrentLimit(Amps.of(40))
        .withZeroPower(MotorMode.BRAKE)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withStartingPosition(Degrees.of(0))
        .withClosedLoopController(lqr())
        .withFeedforward(new ArmFeedforward(0, kG, 0))
        .withTelemetry("ArmFeedforwardTest " + name, TelemetryVerbosity.LOW);
  }

  private static void closeSmc(SmartMotorController smc) {
    SmartMotorControllerTestSubsystem subsys =
        (SmartMotorControllerTestSubsystem)
            ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem();
    SmartMotorControllerCommandRegistry.removeCommands(subsys);
    CommandScheduler.getInstance().unregisterSubsystem(subsys);
    subsys.close();
    smc.close();
    DeviceCreator.silence(smc);
    Object motorController = smc.getMotorController();
    if (motorController instanceof SparkMax sparkMax) {
      sparkMax.close();
    } else if (motorController instanceof SparkFlex sparkFlex) {
      sparkFlex.close();
    } else if (motorController instanceof TalonFXS talonFXS) {
      talonFXS.close();
    } else if (motorController instanceof TalonFX talonFX) {
      talonFX.close();
    } else if (motorController instanceof Nova nova) {
      nova.close();
    }
  }

  /**
   * Holding a horizontal arm at its setpoint, the closed loop output is the arm feedforward's kG:
   * the LQR has no error to correct, so without the feedforward there would be no output.
   */
  @ParameterizedTest(name = "{0}")
  @MethodSource("createControllers")
  void armFeedforwardContributesWithoutAProfile(Case testCase) {
    final SmartMotorController smc = testCase.create().apply(config(testCase.name()));
    try {
      ((SmartMotorControllerTestSubsystem)
              ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem())
          .setSMC(smc);
      smc.setPosition(Degrees.of(0));
      commandedVolts.set(Double.NaN);
      smc.iterateClosedLoopController();
      final double volts = commandedVolts.get();
      assertTrue(
          volts > kG * 0.75 && volts < kG * 1.25,
          testCase.name()
              + ": expected about "
              + kG
              + " V from the arm feedforward with no motion profile, but the output was "
              + volts
              + " V");
    } finally {
      closeSmc(smc);
    }
  }
}
