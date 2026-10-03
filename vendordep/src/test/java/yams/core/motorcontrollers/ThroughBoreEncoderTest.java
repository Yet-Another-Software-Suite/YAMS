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

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.encoder.DetachedEncoder;
import com.revrobotics.encoder.config.DetachedEncoderConfig;
import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.ExternalEncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkParameters;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
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
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.PeriodicScheduler;
import yams.helpers.SmartMotorControllerTestSubsystem;

/**
 * Tests a REV Through Bore Encoder on a {@link SparkWrapper} around a SPARK MAX and a SPARK Flex,
 * in each way it can be connected:
 *
 * <ul>
 *   <li>Wired, absolute: its duty cycle output on the SPARK's absolute encoder input.
 *   <li>Wired, quadrature: its quadrature output on the SPARK MAX's alternate encoder or the SPARK
 *       Flex's external encoder input, with REVLib's Through Bore preset for its counts per
 *       revolution.
 *   <li>CAN: an absolute encoder the SPARK reads over CAN, a REVLib {@link DetachedEncoder}. REVLib's
 *       only detached encoder model is the MAXSpline Encoder, so the test uses one.
 * </ul>
 *
 * <p>For each, the SPARK and the encoder are configured from the {@link SmartMotorControllerConfig},
 * and in simulation the encoder reads the mechanism, both when the SPARK closes its loop on it and
 * when it only reports. Continuous wrapping with each is tested by {@link ContinuousWrappingTest}.
 */
public class ThroughBoreEncoderTest {
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

  /** How the Through Bore Encoder is connected. */
  enum Connection {
    /** Duty cycle output on the SPARK's absolute encoder input. */
    ABSOLUTE,
    /** Quadrature output on the SPARK MAX's alternate or the SPARK Flex's external encoder input. */
    QUADRATURE,
    /** On the CAN bus, read by the SPARK over CAN. */
    CAN
  }

  /** A SPARK, how its Through Bore Encoder is connected, and whether it closes the loop on it. */
  private record Case(boolean sparkFlex, Connection connection, boolean feedback) {
    String name() {
      return (sparkFlex ? "SparkFlex" : "SparkMax") + " " + connection + (feedback ? " feedback" : " reporting");
    }

    @Override
    public String toString() {
      return name();
    }
  }

  private static Stream<Case> createCases() {
    final List<Case> cases = new ArrayList<>();
    for (boolean sparkFlex : new boolean[] {false, true}) {
      for (Connection connection : Connection.values()) {
        for (boolean feedback : new boolean[] {true, false}) {
          cases.add(new Case(sparkFlex, connection, feedback));
        }
      }
    }
    return cases.stream();
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

  /**
   * Attach a Through Bore Encoder, mounted on the mechanism 1:1, to the SPARK.
   *
   * @param spark      SPARK.
   * @param connection How the encoder is connected.
   * @param cfg        Config to attach it to.
   * @return The config.
   */
  static SmartMotorControllerConfig withThroughBore(SparkBase spark, Connection connection, SmartMotorControllerConfig cfg) {
    return switch (connection) {
      case ABSOLUTE -> cfg.withExternalEncoder(spark.getAbsoluteEncoder())
          .withExternalEncoderDiscontinuityPoint(Rotations.of(0.5));
      case QUADRATURE -> {
        // The vendor config gives the Through Bore Encoder's counts per revolution.
        if (spark instanceof SparkMax sparkMax) {
          final SparkMaxConfig vendorConfig = new SparkMaxConfig();
          vendorConfig.alternateEncoder.apply(AlternateEncoderConfig.Presets.REV_ThroughBoreEncoder);
          sparkMax.configure(vendorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
          yield cfg.withVendorConfig(vendorConfig).withExternalEncoder(sparkMax.getAlternateEncoder());
        }
        final SparkFlexConfig vendorConfig = new SparkFlexConfig();
        vendorConfig.externalEncoder.apply(ExternalEncoderConfig.Presets.REV_ThroughBoreEncoder);
        yield cfg.withVendorConfig(vendorConfig).withExternalEncoder(((SparkFlex) spark).getExternalEncoder());
      }
      case CAN -> cfg.withExternalEncoder(DeviceCreator.createDetachedEncoder())
          .withExternalEncoderDiscontinuityPoint(Rotations.of(0.5));
    };
  }

  private static SmartMotorController create(Case testCase, String name) {
    final SparkBase spark = testCase.sparkFlex() ? DeviceCreator.createSparkFlex() : DeviceCreator.createSparkMax();
    final SmartMotorControllerConfig cfg = withThroughBore(spark, testCase.connection(), config(name))
        .withUseExternalFeedbackEncoder(testCase.feedback());
    return new SparkWrapper(spark, testCase.sparkFlex() ? DCMotor.getNeoVortex(1) : DCMotor.getNEO(1), cfg);
  }

  /** Put the SPARK and the detached encoder, if any, back to factory defaults and close them. */
  static void closeDevices(Object motorController, Object externalEncoder) {
    if (externalEncoder instanceof DetachedEncoder detachedEncoder) {
      detachedEncoder.configure(new DetachedEncoderConfig(), ResetMode.kResetSafeParameters);
      detachedEncoder.close();
    }
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
    closeDevices(smc.getMotorController(), smc.getConfig().getExternalEncoder().orElse(null));
  }

  /** Wait for a SPARK configuration applied asynchronously. */
  private static boolean eventually(BooleanSupplier condition) throws InterruptedException {
    for (int i = 0; i < 50; i++) {
      if (condition.getAsBoolean()) {
        return true;
      }
      Thread.sleep(10);
    }
    return condition.getAsBoolean();
  }

  /**
   * Read a SPARK parameter. REVLib's config accessor reports a detached encoder feedback sensor as
   * {@link FeedbackSensor#kNoSensor}, since {@link FeedbackSensor#fromId} has no case for it.
   */
  private static int sparkParameter(SparkBase spark, SparkParameters parameter) {
    try {
      final Field handle = SparkLowLevel.class.getDeclaredField("sparkHandle");
      handle.setAccessible(true);
      return CANSparkJNI.c_Spark_GetParameterUint32(handle.getLong(spark), parameter.value);
    } catch (ReflectiveOperationException e) {
      throw new AssertionError(e);
    }
  }

  /** The SPARK's absolute encoder range offset. */
  private static double absoluteEncoderRangeOffset(SparkBase spark) {
    return spark instanceof SparkMax sparkMax
           ? sparkMax.configAccessor.absoluteEncoder.getRangeOffset()
           : ((SparkFlex) spark).configAccessor.absoluteEncoder.getRangeOffset();
  }

  /** Difference between two angles, wrapped into [-180°, 180°). */
  private static double wrappedErrorDegrees(Angle actual, Angle expected) {
    return MathUtil.inputModulus(actual.in(Degrees) - expected.in(Degrees), -180, 180);
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("createCases")
  void configuresTheSparkAndTheEncoder(Case testCase) throws InterruptedException {
    final String name = testCase.name();
    final SmartMotorController smc = create(testCase, "ThroughBoreEncoderTest config " + name);
    try {
      ((SmartMotorControllerTestSubsystem) ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem()).setSMC(smc);
      final SparkBase spark = (SparkBase) smc.getMotorController();
      final FeedbackSensor expectedSensor = !testCase.feedback()
                                            ? FeedbackSensor.kPrimaryEncoder
                                            : switch (testCase.connection()) {
                                              case ABSOLUTE -> FeedbackSensor.kAbsoluteEncoder;
                                              case QUADRATURE -> FeedbackSensor.kAlternateOrExternalEncoder;
                                              case CAN -> FeedbackSensor.kDetachedAbsoluteEncoder;
                                            };
      assertTrue(eventually(() -> sparkParameter(spark, SparkParameters.kClosedLoopControlSensor) == expectedSensor.value),
          name + ": expected the SPARK's feedback sensor to be " + expectedSensor + " but it was " + sparkParameter(spark, SparkParameters.kClosedLoopControlSensor));

      switch (testCase.connection()) {
        case ABSOLUTE -> {
          // REVLib's range offset is the middle of the range the encoder reports: 0 for [-0.5, 0.5)
          // rotations, 0.5 for [0, 1).
          assertTrue(eventually(() -> Math.abs(absoluteEncoderRangeOffset(spark)) < 1e-6),
              name + ": a 0.5 rotation discontinuity point is a range offset of 0, but it was " + absoluteEncoderRangeOffset(spark));
          smc.applyConfig(smc.getConfig().withExternalEncoderDiscontinuityPoint(Rotations.of(1)));
          assertTrue(eventually(() -> Math.abs(absoluteEncoderRangeOffset(spark) - 0.5) < 1e-6),
              name + ": a 1 rotation discontinuity point is a range offset of 0.5, but it was " + absoluteEncoderRangeOffset(spark));
        }
        case QUADRATURE -> {
          final int countsPerRevolution = spark instanceof SparkMax sparkMax
                                          ? sparkMax.configAccessor.alternateEncoder.getCountsPerRevolution()
                                          : ((SparkFlex) spark).configAccessor.externalEncoder.getCountsPerRevolution();
          assertEquals(8192, countsPerRevolution, name + ": Through Bore Encoder counts per revolution");
        }
        case CAN -> {
          final DetachedEncoder encoder = (DetachedEncoder) smc.getConfig().getExternalEncoder().orElseThrow();
          if (testCase.feedback()) {
            assertEquals(encoder.getDeviceId(), sparkParameter(spark, SparkParameters.kDetachedEncoderDeviceID), name + ": the SPARK reads the detached encoder's CAN ID");
          }
          assertTrue(encoder.detachedEncoderAccessor.isDutyCycleZeroCentered(), name + ": a 0.5 rotation discontinuity point is zero-centered");
          smc.applyConfig(smc.getConfig().withExternalEncoderDiscontinuityPoint(Rotations.of(1)));
          assertTrue(!encoder.detachedEncoderAccessor.isDutyCycleZeroCentered(), name + ": a 1 rotation discontinuity point is not zero-centered");
        }
      }
    } finally {
      closeSmc(smc);
    }
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("createCases")
  void readsTheMechanism(Case testCase) {
    final String name = testCase.name();
    final SmartMotorController smc = create(testCase, "ThroughBoreEncoderTest closed loop " + name);
    try {
      ((SmartMotorControllerTestSubsystem) ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem()).setSMC(smc);
      smc.setupSimulation();

      try (PeriodicScheduler scheduler = new PeriodicScheduler()) {
        final Angle setpoint = Degrees.of(100);
        smc.setPosition(setpoint);
        scheduler.addPeriodic(smc::simIterate, Milliseconds.of(10));
        scheduler.addPeriodic(
            () -> {
              smc.setPosition(setpoint);
              smc.updateTelemetry();
            },
            Milliseconds.of(20));
        scheduler.runFor(Seconds.of(2.0));

        final Angle simulated = smc.getSimSupplier().orElseThrow().getMechanismPosition();
        final Angle mechanism = smc.getMechanismPosition();
        final Angle encoder = smc.getExternalEncoderPosition().orElseThrow();
        assertTrue(Math.abs(wrappedErrorDegrees(simulated, setpoint)) < kTolerance.in(Degrees),
            name + ": expected the simulated mechanism at " + setpoint.in(Degrees) + "° but it was at " + simulated.in(Degrees) + "°");
        assertTrue(Math.abs(wrappedErrorDegrees(mechanism, setpoint)) < kTolerance.in(Degrees),
            name + ": expected the mechanism to read " + setpoint.in(Degrees) + "° but it read " + mechanism.in(Degrees) + "°");
        assertTrue(Math.abs(wrappedErrorDegrees(encoder, setpoint)) < kTolerance.in(Degrees),
            name + ": expected the Through Bore Encoder to read " + setpoint.in(Degrees) + "° but it read " + encoder.in(Degrees) + "°");
      }
    } finally {
      closeSmc(smc);
    }
  }

  @ParameterizedTest(name = "{0}")
  @ValueSource(strings = {"SparkMax", "SparkFlex"})
  void quadratureEncoderHasNoZeroOffset(String name) {
    final SparkBase spark = name.equals("SparkFlex") ? DeviceCreator.createSparkFlex() : DeviceCreator.createSparkMax();
    final SmartMotorControllerConfig config = withThroughBore(spark, Connection.QUADRATURE, config("ThroughBoreEncoderTest zero offset " + name))
        .withUseExternalFeedbackEncoder(true)
        .withExternalEncoderZeroOffset(Degrees.of(30));
    try {
      assertThrows(SmartMotorControllerConfigurationException.class,
          () -> new SparkWrapper(spark, name.equals("SparkFlex") ? DCMotor.getNeoVortex(1) : DCMotor.getNEO(1), config),
          name + ": a quadrature encoder has no zero offset");
    } finally {
      CommandScheduler.getInstance().unregisterSubsystem(config.getSubsystem());
      closeDevices(spark, null);
    }
  }
}
