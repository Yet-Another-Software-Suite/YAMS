// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.ValueSource;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.MathUtil;
import org.wpilib.preferences.Preferences;
import org.wpilib.units.measure.Angle;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.telemetry.SmartMotorControllerCommandRegistry;
import yams.core.exceptions.SmartMotorControllerConfigurationException;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.remote.TalonFXSWrapper;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.PeriodicScheduler;
import yams.helpers.SmartMotorControllerTestSubsystem;

/**
 * Tests a {@link CANdi} as the external feedback encoder of a {@link TalonFXWrapper} and a
 * {@link TalonFXSWrapper}, on either PWM input: the Talon and the CANdi are configured from the
 * {@link SmartMotorControllerConfig}, they stay configured when the config is applied again, and in
 * simulation the Talon closes its loop on the CANdi, with or without a zero offset. Continuous
 * wrapping with a CANdi is tested by {@link ContinuousWrappingTest}.
 *
 * <p>A CANdi has two PWM inputs and YAMS cannot tell which the encoder is wired to, so the input is
 * selected with a vendor config whose feedback sensor source is {@code SyncCANdiPWM1} or
 * {@code SyncCANdiPWM2}.
 */
public class CANdiTest {
  private static final Angle kTolerance = Degrees.of(5);
  private static final MechanismGearing kGearing = new MechanismGearing(GearBox.fromReductionStages(3, 4));

  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
  }

  @AfterEach
  void endTest() {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  /** A Talon, the PWM input of its CANdi, and the CANdi's zero offset. */
  private record Case(boolean talonFXS, int pwm, Angle zeroOffset) {
    String name() {
      return (talonFXS ? "TalonFXS" : "TalonFX") + " CANdi PWM" + pwm
          + (zeroOffset.isEquivalent(Rotations.zero()) ? "" : " offset " + zeroOffset.in(Degrees) + "°");
    }

    @Override
    public String toString() {
      return name();
    }
  }

  private static Stream<Case> createCases() {
    final List<Case> cases = new ArrayList<>();
    for (boolean talonFXS : new boolean[] {false, true}) {
      for (int pwm : new int[] {1, 2}) {
        for (Angle zeroOffset : new Angle[] {Rotations.zero(), Degrees.of(40)}) {
          cases.add(new Case(talonFXS, pwm, zeroOffset));
        }
      }
    }
    return cases.stream();
  }

  /**
   * A vendor config selecting the CANdi PWM input, as YAMS's documentation describes.
   *
   * @param talonFXS Whether the Talon is a TalonFXS.
   * @param pwm      CANdi PWM input the encoder is wired to, 1 or 2.
   * @param candi    CANdi.
   * @return {@link TalonFXConfiguration} or {@link TalonFXSConfiguration}.
   */
  static Object vendorConfig(boolean talonFXS, int pwm, CANdi candi) {
    if (talonFXS) {
      final TalonFXSConfiguration config = new TalonFXSConfiguration();
      config.ExternalFeedback.ExternalFeedbackSensorSource =
          pwm == 1 ? ExternalFeedbackSensorSourceValue.SyncCANdiPWM1 : ExternalFeedbackSensorSourceValue.SyncCANdiPWM2;
      config.ExternalFeedback.FeedbackRemoteSensorID = candi.getDeviceID();
      return config;
    }
    final TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.FeedbackSensorSource =
        pwm == 1 ? FeedbackSensorSourceValue.SyncCANdiPWM1 : FeedbackSensorSourceValue.SyncCANdiPWM2;
    config.Feedback.FeedbackRemoteSensorID = candi.getDeviceID();
    return config;
  }

  private static SmartMotorControllerConfig config(String name) {
    return new yams.commands2.config.SmartMotorControllerConfig()
        .withSubsystem(new SmartMotorControllerTestSubsystem())
        .withGearing(kGearing)
        .withStatorCurrentLimit(Amps.of(40))
        .withZeroPower(MotorMode.BRAKE)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withSimulationPeriod(Milliseconds.of(10))
        .withClosedLoopController(8, 0, 0)
        .withFeedforward(new SimpleMotorFeedforward(0, 1.5))
        .withTelemetry(name, TelemetryVerbosity.LOW);
  }

  /** Close the loop on a CANdi mounted on the mechanism, 1:1. */
  private static SmartMotorControllerConfig withCANdi(SmartMotorControllerConfig cfg, Case testCase, CANdi candi) {
    cfg = cfg.withVendorConfig(vendorConfig(testCase.talonFXS(), testCase.pwm(), candi))
        .withExternalEncoder(candi)
        .withUseExternalFeedbackEncoder(true)
        .withExternalEncoderDiscontinuityPoint(Rotations.of(0.5));
    return testCase.zeroOffset().isEquivalent(Rotations.zero()) ? cfg : cfg.withExternalEncoderZeroOffset(testCase.zeroOffset());
  }

  private static SmartMotorController create(Case testCase, String name) {
    if (testCase.talonFXS()) {
      final TalonFXS talon = DeviceCreator.createTalonFXS();
      return new TalonFXSWrapper(talon, DCMotor.getNEO(1), withCANdi(config(name), testCase, DeviceCreator.createCANdiFor(talon)));
    }
    final TalonFX talon = DeviceCreator.createTalonFX();
    return new TalonFXWrapper(talon, DCMotor.getKrakenX60(1), withCANdi(config(name), testCase, DeviceCreator.createCANdiFor(talon)));
  }

  /** Put the Talon and the CANdi back to factory defaults and close them. */
  static void closeDevices(Object motorController, CANdi candi) {
    candi.getConfigurator().apply(new CANdiConfiguration());
    candi.close();
    if (motorController instanceof TalonFXS talonFXS) {
      talonFXS.getConfigurator().apply(new TalonFXSConfiguration());
      talonFXS.close();
    } else if (motorController instanceof TalonFX talonFX) {
      talonFX.getConfigurator().apply(new TalonFXConfiguration());
      talonFX.close();
    }
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
    closeDevices(smc.getMotorController(), (CANdi) smc.getConfig().getExternalEncoder().orElseThrow());
  }

  /** Check that the Talon closes its loop on the CANdi, configured as the SMC config says. */
  private static void assertConfigured(SmartMotorController smc, Case testCase, String when) {
    assertConfigured(smc, testCase, when, 0.5);
  }

  /** Check that the Talon closes its loop on the CANdi, with the discontinuity point in rotations. */
  private static void assertConfigured(SmartMotorController smc, Case testCase, String when, double expectedDiscontinuityPoint) {
    final String name = testCase.name() + " " + when;
    final CANdi candi = (CANdi) smc.getConfig().getExternalEncoder().orElseThrow();
    if (testCase.talonFXS()) {
      final ExternalFeedbackConfigs feedback = new ExternalFeedbackConfigs();
      DeviceCreator.refreshConfig(() -> ((TalonFXS) smc.getMotorController()).getConfigurator().refresh(feedback, 1.0));
      assertEquals(testCase.pwm() == 1 ? ExternalFeedbackSensorSourceValue.FusedCANdiPWM1 : ExternalFeedbackSensorSourceValue.FusedCANdiPWM2,
          feedback.ExternalFeedbackSensorSource, name + ": feedback sensor source");
      assertEquals(candi.getDeviceID(), feedback.FeedbackRemoteSensorID, name + ": remote sensor ID");
      assertEquals(kGearing.getMechanismToRotorRatio(), feedback.RotorToSensorRatio, 1e-9, name + ": rotor to sensor ratio");
      assertEquals(1, feedback.SensorToMechanismRatio, 1e-9, name + ": sensor to mechanism ratio");
    } else {
      final FeedbackConfigs feedback = new FeedbackConfigs();
      DeviceCreator.refreshConfig(() -> ((TalonFX) smc.getMotorController()).getConfigurator().refresh(feedback, 1.0));
      assertEquals(testCase.pwm() == 1 ? FeedbackSensorSourceValue.FusedCANdiPWM1 : FeedbackSensorSourceValue.FusedCANdiPWM2,
          feedback.FeedbackSensorSource, name + ": feedback sensor source");
      assertEquals(candi.getDeviceID(), feedback.FeedbackRemoteSensorID, name + ": remote sensor ID");
      assertEquals(kGearing.getMechanismToRotorRatio(), feedback.RotorToSensorRatio, 1e-9, name + ": rotor to sensor ratio");
      assertEquals(1, feedback.SensorToMechanismRatio, 1e-9, name + ": sensor to mechanism ratio");
    }

    final CANdiConfiguration candiConfig = new CANdiConfiguration();
    DeviceCreator.refreshConfig(() -> candi.getConfigurator().refresh(candiConfig, 1.0));
    final double discontinuityPoint = testCase.pwm() == 1 ? candiConfig.PWM1.AbsoluteSensorDiscontinuityPoint : candiConfig.PWM2.AbsoluteSensorDiscontinuityPoint;
    final double zeroOffset = testCase.pwm() == 1 ? candiConfig.PWM1.AbsoluteSensorOffset : candiConfig.PWM2.AbsoluteSensorOffset;
    assertEquals(expectedDiscontinuityPoint, discontinuityPoint, 1e-9, name + ": CANdi discontinuity point");
    assertEquals(testCase.zeroOffset().in(Rotations), zeroOffset, 1e-3, name + ": CANdi zero offset");
  }

  /** Let 10ms of real time pass. */
  private static void sleepOneStep() {
    try {
      Thread.sleep(10);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
  }

  /** Difference between two angles, wrapped into [-180°, 180°). */
  private static double wrappedErrorDegrees(Angle actual, Angle expected) {
    return MathUtil.inputModulus(actual.in(Degrees) - expected.in(Degrees), -180, 180);
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("createCases")
  void configuresTheTalonAndTheCANdi(Case testCase) {
    final SmartMotorController smc = create(testCase, "CANdiTest config " + testCase.name());
    try {
      ((SmartMotorControllerTestSubsystem) ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem()).setSMC(smc);
      assertConfigured(smc, testCase, "after construction");
      // Applying the config again, as live tuning and the config commands do, keeps the CANdi.
      assertTrue(smc.applyConfig(smc.getConfig()), testCase.name() + ": applying the config again");
      assertConfigured(smc, testCase, "after applying the config again");
      assertTrue(smc.applyConfig(smc.getConfig().withExternalEncoderDiscontinuityPoint(Rotations.of(1))), testCase.name() + ": applying a 1 rotation discontinuity point");
      assertConfigured(smc, testCase, "with a 1 rotation discontinuity point", 1);
    } finally {
      closeSmc(smc);
    }
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("createCases")
  void closesTheLoopOnTheCANdi(Case testCase) {
    final String name = testCase.name();
    final SmartMotorController smc = create(testCase, "CANdiTest closed loop " + name);
    try {
      ((SmartMotorControllerTestSubsystem) ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem()).setSMC(smc);
      smc.setupSimulation();

      try (PeriodicScheduler scheduler = new PeriodicScheduler()) {
        final Angle setpoint = Degrees.of(100);
        smc.setPosition(setpoint);
        scheduler.addPeriodic(smc::simIterate, Milliseconds.of(10));
        // Phoenix simulates a Talon on its own thread in real time; keep the simulation in step.
        scheduler.addPeriodic(CANdiTest::sleepOneStep, Milliseconds.of(10));
        scheduler.addPeriodic(
            () -> {
              smc.setPosition(setpoint);
              smc.updateTelemetry();
            },
            Milliseconds.of(20));
        scheduler.runFor(Seconds.of(2.0));

        final Angle mechanism = smc.getMechanismPosition();
        final Angle candi = smc.getExternalEncoderPosition().orElseThrow();
        assertTrue(Math.abs(wrappedErrorDegrees(mechanism, setpoint)) < kTolerance.in(Degrees),
            name + ": expected the mechanism at " + setpoint.in(Degrees) + "° but was at " + mechanism.in(Degrees) + "°");
        assertTrue(Math.abs(wrappedErrorDegrees(candi, setpoint)) < kTolerance.in(Degrees),
            name + ": expected the CANdi to read " + setpoint.in(Degrees) + "° but it read " + candi.in(Degrees) + "°");
      }
    } finally {
      closeSmc(smc);
    }
  }

  @ParameterizedTest(name = "{0}")
  @ValueSource(strings = {"TalonFX", "TalonFXS"})
  void requiresAPwmInput(String name) {
    final Object talon = name.equals("TalonFXS") ? DeviceCreator.createTalonFXS() : DeviceCreator.createTalonFX();
    final CANdi candi = talon instanceof TalonFXS fxs ? DeviceCreator.createCANdiFor(fxs) : DeviceCreator.createCANdiFor((TalonFX) talon);
    final SmartMotorControllerConfig config = config("CANdiTest no PWM input " + name)
        .withExternalEncoder(candi)
        .withUseExternalFeedbackEncoder(true);
    try {
      assertThrows(SmartMotorControllerConfigurationException.class, () -> {
        if (talon instanceof TalonFXS fxs) {
          new TalonFXSWrapper(fxs, DCMotor.getNEO(1), config);
        } else {
          new TalonFXWrapper((TalonFX) talon, DCMotor.getKrakenX60(1), config);
        }
      }, name + ": a CANdi without a PWM input selected");
    } finally {
      CommandScheduler.getInstance().unregisterSubsystem(config.getSubsystem());
      DeviceCreator.silence(talon);
      DeviceCreator.silence(candi);
      closeDevices(talon, candi);
    }
  }
}
