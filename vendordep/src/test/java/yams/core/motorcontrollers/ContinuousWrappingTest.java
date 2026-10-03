// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.encoder.DetachedEncoder;
import com.revrobotics.encoder.config.DetachedEncoderConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.MathUtil;
import org.wpilib.preferences.Preferences;
import org.wpilib.units.measure.Angle;
import yams.core.motorcontrollers.SmartMotorController.ClosedLoopControllerSlot;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.telemetry.SmartMotorControllerCommandRegistry;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.math.LQRConfig;
import yams.core.math.LQRController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.motorcontrollers.remote.TalonFXSWrapper;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.PeriodicScheduler;
import yams.helpers.SmartMotorControllerTestSubsystem;

/**
 * Tests continuous wrapping: {@link SmartMotorControllerConfig#getContinuousWrappingSetpoint}, and,
 * in simulation, every motor controller wrapper under every closed loop controller, with and without
 * an attached encoder, crossing the wrapping point the short way. The attached encoders are absolute
 * (a CANcoder, a CANdi, a SPARK's absolute encoder, or an absolute encoder a SPARK reads over CAN)
 * or not (a Through Bore Encoder's quadrature output on a SPARK).
 *
 * <p>The mechanisms are geared 12:1, so the motor's own encoder turns twelve times per mechanism
 * rotation. A SPARK's onboard position wrapping wraps every rotation of its feedback sensor, so
 * before {@link SparkWrapper} sent the nearest equivalent setpoint itself, a SPARK without an
 * attached encoder could only stop within 15° of a multiple of 30° from its target.
 */
public class ContinuousWrappingTest {
  private static final Angle kTolerance = Degrees.of(5);
  static final MechanismGearing kGearing = new MechanismGearing(GearBox.fromReductionStages(3, 4));

  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
  }

  @AfterEach
  void endTest() {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  private static SmartMotorControllerConfig wrappingConfig() {
    return new yams.commands2.config.SmartMotorControllerConfig()
        .withClosedLoopController(8, 0, 0)
        .withContinuousWrapping(Rotations.of(-0.5), Rotations.of(0.5));
  }

  @Test
  void setpointIsUnchangedWithoutContinuousWrapping() {
    final SmartMotorControllerConfig config =
        new yams.commands2.config.SmartMotorControllerConfig().withClosedLoopController(8, 0, 0);
    assertEquals(
        0.45,
        config.getContinuousWrappingSetpoint(Rotations.of(0.45), Rotations.of(-0.45)).in(Rotations),
        1e-9);
  }

  @Test
  void setpointIsTheEquivalentNearestTheCurrentPosition() {
    final SmartMotorControllerConfig config = wrappingConfig();
    // Across the wrapping point: from -0.45 to 0.45 is 0.1 rotations the other way, at -0.55.
    assertEquals(
        -0.55,
        config.getContinuousWrappingSetpoint(Rotations.of(0.45), Rotations.of(-0.45)).in(Rotations),
        1e-9);
    assertEquals(
        0.6,
        config.getContinuousWrappingSetpoint(Rotations.of(-0.4), Rotations.of(0.4)).in(Rotations),
        1e-9);
    // Several rotations away from the wrapping range, as a swerve module's encoder can be.
    assertEquals(
        2.1,
        config.getContinuousWrappingSetpoint(Rotations.of(0.1), Rotations.of(2.0)).in(Rotations),
        1e-9);
    // Already the nearest equivalent.
    assertEquals(
        0.25,
        config.getContinuousWrappingSetpoint(Rotations.of(0.25), Rotations.of(0.0)).in(Rotations),
        1e-9);
  }

  @Test
  void pidControllerSetAfterContinuousWrappingWraps() {
    final SmartMotorControllerConfig config = wrappingConfig().withClosedLoopController(4, 0, 0);
    assertTrue(config.getPID(ClosedLoopControllerSlot.SLOT_0).orElseThrow().isContinuousInputEnabled());
  }

  @Test
  void simulationPidControllerWraps() {
    // Unit tests run in simulation, so getPID returns the simulation controller.
    final SmartMotorControllerConfig before = new yams.commands2.config.SmartMotorControllerConfig()
        .withClosedLoopController(8, 0, 0)
        .withSimClosedLoopController(4, 0, 0)
        .withContinuousWrapping(Rotations.of(-0.5), Rotations.of(0.5));
    assertTrue(before.getPID(ClosedLoopControllerSlot.SLOT_0).orElseThrow().isContinuousInputEnabled());
    final SmartMotorControllerConfig after = wrappingConfig().withSimClosedLoopController(4, 0, 0);
    assertTrue(after.getPID(ClosedLoopControllerSlot.SLOT_0).orElseThrow().isContinuousInputEnabled());
  }

  @Test
  void continuousWrappingWorksWithAnLqrController() {
    assertDoesNotThrow(
        () -> new yams.commands2.config.SmartMotorControllerConfig()
            .withClosedLoopController(lqr())
            .withContinuousWrapping(Rotations.of(-0.5), Rotations.of(0.5)));
  }

  /** The closed loop controllers a mechanism can be configured with. */
  private enum Controller {
    /** A plain PID controller, run onboard the motor controller. */
    PID,
    /** A trapezoidal profile, run onboard (SPARK MAXMotion, Talon Motion Magic). */
    TRAPEZOIDAL_PROFILE,
    /** An exponential profile, run onboard on Talons and by YAMS on SPARKs. */
    EXPONENTIAL_PROFILE,
    /** An LQR, always run by YAMS. */
    LQR
  }

  /** A motor controller to test, created from its config when the test runs. */
  private record Case(String name, Controller controller,
                      Function<SmartMotorControllerConfig, SmartMotorController> create) {
    @Override
    public String toString() {
      return name;
    }
  }

  static LQRController lqr() {
    return new LQRController(new LQRConfig(DCMotor.getNEO(1), kGearing, KilogramSquareMeters.of(0.02))
        .withArm(Radians.of(0.01), RadiansPerSecond.of(0.5), Radians.of(0.05), RadiansPerSecond.of(0.5), Radians.of(0.01))
        .withControlEffort(Volts.of(12))
        .withMaxVoltage(Volts.of(12)));
  }

  private static SmartMotorControllerConfig config(String name, Controller controller) {
    SmartMotorControllerConfig config = new yams.commands2.config.SmartMotorControllerConfig()
        .withSubsystem(new SmartMotorControllerTestSubsystem())
        .withGearing(kGearing)
        .withStatorCurrentLimit(Amps.of(40))
        .withZeroPower(MotorMode.BRAKE)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withSimulationPeriod(Milliseconds.of(10))
        .withStartingPosition(Degrees.of(0))
        // Roughly 12 V over the free speed of a NEO or Kraken behind the 12:1 gearing, in volts per
        // mechanism rotation per second, so the motion profiles can be followed.
        .withFeedforward(new SimpleMotorFeedforward(0, 1.5))
        .withTelemetry(name, TelemetryVerbosity.LOW);
    config = switch (controller) {
      case PID -> config.withClosedLoopController(8, 0, 0);
      case TRAPEZOIDAL_PROFILE -> config.withClosedLoopController(8, 0, 0)
          .withTrapezoidalProfile(DegreesPerSecond.of(720), DegreesPerSecondPerSecond.of(1440));
      case EXPONENTIAL_PROFILE -> config.withClosedLoopController(8, 0, 0)
          .withExponentialProfile(Volts.of(12), DegreesPerSecond.of(720), DegreesPerSecondPerSecond.of(1440));
      case LQR -> config.withClosedLoopController(lqr());
    };
    return config.withContinuousWrapping(Rotations.of(-0.5), Rotations.of(0.5));
  }

  /** Every wrapper, closed loop controller, and attached encoder combination. */
  private static Stream<Case> createControllers() {
    final List<Case> cases = new ArrayList<>();
    for (Controller controller : Controller.values()) {
      for (boolean attachedEncoder : new boolean[] {false, true}) {
        final String suffix = " " + controller + (attachedEncoder ? " with attached encoder" : "");
        cases.add(new Case("SparkMax" + suffix, controller, cfg -> spark(DeviceCreator.createSparkMax(), DCMotor.getNEO(1), cfg, attachedEncoder)));
        cases.add(new Case("SparkFlex" + suffix, controller, cfg -> spark(DeviceCreator.createSparkFlex(), DCMotor.getNeoVortex(1), cfg, attachedEncoder)));
        cases.add(new Case("TalonFX" + suffix, controller, cfg -> {
          final TalonFX talon = DeviceCreator.createTalonFX();
          return new TalonFXWrapper(talon, DCMotor.getKrakenX60(1),
              attachedEncoder ? withEncoder(cfg, DeviceCreator.createCANcoderFor(talon)) : cfg);
        }));
        cases.add(new Case("TalonFXS" + suffix, controller, cfg -> {
          final TalonFXS talon = DeviceCreator.createTalonFXS();
          return new TalonFXSWrapper(talon, DCMotor.getNEO(1),
              attachedEncoder ? withEncoder(cfg, DeviceCreator.createCANcoderFor(talon)) : cfg);
        }));
      }
      // A Through Bore Encoder on either SPARK, on the mechanism: its quadrature output, which is not
      // absolute, or an absolute encoder on the CAN bus.
      for (ThroughBoreEncoderTest.Connection connection : new ThroughBoreEncoderTest.Connection[] {
          ThroughBoreEncoderTest.Connection.QUADRATURE, ThroughBoreEncoderTest.Connection.CAN}) {
        final String throughBoreSuffix = " " + controller + " with " + connection + " Through Bore";
        cases.add(new Case("SparkMax" + throughBoreSuffix, controller, cfg -> {
          final SparkMax spark = DeviceCreator.createSparkMax();
          return new SparkWrapper(spark, DCMotor.getNEO(1),
              ThroughBoreEncoderTest.withThroughBore(spark, connection, cfg).withUseExternalFeedbackEncoder(true));
        }));
        cases.add(new Case("SparkFlex" + throughBoreSuffix, controller, cfg -> {
          final SparkFlex spark = DeviceCreator.createSparkFlex();
          return new SparkWrapper(spark, DCMotor.getNeoVortex(1),
              ThroughBoreEncoderTest.withThroughBore(spark, connection, cfg).withUseExternalFeedbackEncoder(true));
        }));
      }
      // A CANdi on either Talon, its encoder wired to PWM1.
      final String suffix = " " + controller + " with attached CANdi";
      cases.add(new Case("TalonFX" + suffix, controller, cfg -> {
        final TalonFX talon = DeviceCreator.createTalonFX();
        final CANdi candi = DeviceCreator.createCANdiFor(talon);
        return new TalonFXWrapper(talon, DCMotor.getKrakenX60(1),
            withEncoder(cfg.withVendorConfig(CANdiTest.vendorConfig(false, 1, candi)), candi));
      }));
      cases.add(new Case("TalonFXS" + suffix, controller, cfg -> {
        final TalonFXS talon = DeviceCreator.createTalonFXS();
        final CANdi candi = DeviceCreator.createCANdiFor(talon);
        return new TalonFXSWrapper(talon, DCMotor.getNEO(1),
            withEncoder(cfg.withVendorConfig(CANdiTest.vendorConfig(true, 1, candi)), candi));
      }));
    }
    return cases.stream();
  }

  /** A SPARK, optionally closing the loop on its absolute encoder, mounted on the mechanism. */
  private static SmartMotorController spark(SparkBase spark, DCMotor motor, SmartMotorControllerConfig cfg,
                                            boolean attachedEncoder) {
    return new SparkWrapper(spark, motor, attachedEncoder ? withEncoder(cfg, spark.getAbsoluteEncoder()) : cfg);
  }

  /** Close the loop on an encoder mounted on the mechanism, 1:1. */
  private static SmartMotorControllerConfig withEncoder(SmartMotorControllerConfig cfg, Object encoder) {
    return cfg.withExternalEncoder(encoder)
        .withUseExternalFeedbackEncoder(true)
        .withExternalEncoderDiscontinuityPoint(Rotations.of(0.5));
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

    // Simulated devices keep their configuration for the rest of the test run, and YAMS starts from a
    // device's previous configuration, so put every device back to factory defaults for the tests
    // that reuse its ID.
    smc.getConfig().getExternalEncoder().ifPresent(encoder -> {
      if (encoder instanceof CANcoder cancoder) {
        cancoder.getConfigurator().apply(new CANcoderConfiguration());
        cancoder.close();
      } else if (encoder instanceof CANdi candi) {
        candi.getConfigurator().apply(new CANdiConfiguration());
        candi.close();
      } else if (encoder instanceof DetachedEncoder detachedEncoder) {
        detachedEncoder.configure(new DetachedEncoderConfig(), ResetMode.kResetSafeParameters);
        detachedEncoder.close();
      }
    });
    Object motorController = smc.getMotorController();
    if (motorController instanceof SparkMax sparkMax) {
      try {
        sparkMax.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      } catch (IllegalStateException e) {
        // Resetting puts the data port back to its default, and REVLib then reports that a SPARK MAX
        // whose alternate encoder is in use is not configured for it. The reset is applied.
      }
      sparkMax.close();
    } else if (motorController instanceof SparkFlex sparkFlex) {
      sparkFlex.configure(new SparkFlexConfig(), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      sparkFlex.close();
    } else if (motorController instanceof TalonFXS talonFXS) {
      talonFXS.getConfigurator().apply(new TalonFXSConfiguration());
      talonFXS.close();
    } else if (motorController instanceof TalonFX talonFX) {
      talonFX.getConfigurator().apply(new TalonFXConfiguration());
      talonFX.close();
    }
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
  @MethodSource("createControllers")
  void crossesTheWrappingPointTheShortWay(Case testCase) {
    final String name = testCase.name();
    final SmartMotorController smc = testCase.create().apply(config("ContinuousWrappingTest " + name, testCase.controller()));
    try {
      SmartMotorControllerTestSubsystem subsys =
          (SmartMotorControllerTestSubsystem)
              ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem();
      subsys.setSMC(smc);
      smc.setupSimulation();

      try (PeriodicScheduler scheduler = new PeriodicScheduler()) {
        // Not a multiple of 30°, so a SPARK wrapping every motor rotation cannot reach it.
        final Angle[] setpoint = {Degrees.of(170)};
        smc.setPosition(setpoint[0]);
        scheduler.addPeriodic(smc::simIterate, Milliseconds.of(10));
        if (!(smc instanceof SparkWrapper)) {
          // Phoenix simulates a Talon on its own thread in real time: its onboard motion profiles
          // advance with the wall clock, and its status signals refresh on it. Keep the simulation in
          // step with it.
          scheduler.addPeriodic(ContinuousWrappingTest::sleepOneStep, Milliseconds.of(10));
        }
        scheduler.addPeriodic(
            () -> {
              smc.setPosition(setpoint[0]);
              smc.updateTelemetry();
            },
            Milliseconds.of(20));

        scheduler.runFor(Seconds.of(2.0));
        final Angle afterFirstMove = smc.getMechanismPosition();
        assertTrue(
            Math.abs(wrappedErrorDegrees(afterFirstMove, setpoint[0])) < kTolerance.in(Degrees),
            name + ": expected to reach 170° but was at " + afterFirstMove.in(Degrees) + "°");

        // -170° is 20° further on through the wrapping point, or 340° back the long way, which
        // passes 0°. Watch how close to 0° the mechanism gets on the way.
        setpoint[0] = Degrees.of(-170);
        smc.setPosition(setpoint[0]);
        final double[] closestToZeroDegrees = {180};
        scheduler.addPeriodic(
            () -> closestToZeroDegrees[0] = Math.min(closestToZeroDegrees[0],
                Math.abs(wrappedErrorDegrees(smc.getMechanismPosition(), Degrees.of(0)))),
            Milliseconds.of(10));
        scheduler.runFor(Seconds.of(2.0));
        final Angle afterSecondMove = smc.getMechanismPosition();
        assertTrue(
            Math.abs(wrappedErrorDegrees(afterSecondMove, setpoint[0])) < kTolerance.in(Degrees),
            name + ": expected to reach -170° but was at " + afterSecondMove.in(Degrees) + "°");
        assertTrue(
            closestToZeroDegrees[0] > 90,
            name + ": expected to turn through the wrapping point, but came within "
                + closestToZeroDegrees[0] + "° of 0°, the long way round");
      }
    } finally {
      closeSmc(smc);
    }
  }
}
