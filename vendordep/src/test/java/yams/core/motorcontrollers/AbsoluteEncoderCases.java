// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.encoder.DetachedEncoder;
import com.revrobotics.spark.SparkBase;
import com.thrifty.canEncoder.CanEncoder;
import com.thrifty.core.Motor.FeedbackSensorType;
import com.thrifty.nova.Nova;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.UnaryOperator;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Time;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.telemetry.SmartMotorControllerCommandRegistry;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.NovaWrapper;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.motorcontrollers.remote.TalonFXSWrapper;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.PeriodicScheduler;
import yams.helpers.SmartMotorControllerTestSubsystem;

/**
 * Every absolute external encoder on every motor controller, and every closed loop controller, for
 * the tests of options that apply to absolute encoders: {@link DiscontinuityPointTest},
 * {@link ZeroOffsetTest}, and {@link StartingPositionTest}. Each encoder is mounted on the
 * mechanism 1:1, behind 12:1 gearing, and closes the loop.
 */
final class AbsoluteEncoderCases {
  private AbsoluteEncoderCases() {}

  /** An absolute external encoder on a motor controller. */
  enum Encoder {
    /** A SPARK MAX's absolute encoder, such as a wired Through Bore Encoder's duty cycle output. */
    SPARK_MAX_ABSOLUTE("SparkMax absolute encoder"),
    /** A SPARK Flex's absolute encoder. */
    SPARK_FLEX_ABSOLUTE("SparkFlex absolute encoder"),
    /** An absolute encoder a SPARK MAX reads over CAN. */
    SPARK_MAX_CAN("SparkMax CAN encoder"),
    /** An absolute encoder a SPARK Flex reads over CAN. */
    SPARK_FLEX_CAN("SparkFlex CAN encoder"),
    /** A CANcoder on a TalonFX. */
    TALONFX_CANCODER("TalonFX CANcoder"),
    /** A CANcoder on a TalonFXS. */
    TALONFXS_CANCODER("TalonFXS CANcoder"),
    /** A CANdi on a TalonFX, its encoder wired to PWM1. */
    TALONFX_CANDI("TalonFX CANdi"),
    /** A CANdi on a TalonFXS, its encoder wired to PWM1. */
    TALONFXS_CANDI("TalonFXS CANdi"),
    /** An absolute encoder wired to a Nova's data port. */
    NOVA_ABSOLUTE("Nova absolute encoder"),
    /** A Thrifty CAN Encoder a Nova reads over CAN. */
    NOVA_CAN("Nova CAN encoder");

    private final String name;

    Encoder(String name) {
      this.name = name;
    }

    /** Whether the motor controller is a Talon, which Phoenix simulates in real time. */
    boolean talon() {
      return name.startsWith("Talon");
    }

    @Override
    public String toString() {
      return name;
    }
  }

  /** A motor controller without an absolute encoder. */
  enum RelativeFeedback {
    /** A SPARK MAX on its motor's encoder. */
    SPARK_MAX("SparkMax motor encoder"),
    /** A SPARK Flex on its motor's encoder. */
    SPARK_FLEX("SparkFlex motor encoder"),
    /** A TalonFX on its motor's encoder. */
    TALONFX("TalonFX motor encoder"),
    /** A TalonFXS on its motor's encoder. */
    TALONFXS("TalonFXS motor encoder"),
    /** A Through Bore Encoder's quadrature output on a SPARK MAX's alternate encoder port. */
    SPARK_MAX_QUADRATURE("SparkMax quadrature Through Bore"),
    /** A Through Bore Encoder's quadrature output on a SPARK Flex's external encoder port. */
    SPARK_FLEX_QUADRATURE("SparkFlex quadrature Through Bore"),
    /** A Nova on its motor's encoder. */
    NOVA("Nova motor encoder"),
    /** A quadrature encoder on a Nova's data port. */
    NOVA_QUADRATURE("Nova quadrature encoder");

    private final String name;

    RelativeFeedback(String name) {
      this.name = name;
    }

    /** Whether the motor controller is a Talon, which Phoenix simulates in real time. */
    boolean talon() {
      return this == TALONFX || this == TALONFXS;
    }

    @Override
    public String toString() {
      return name;
    }
  }

  /** The closed loop controllers a mechanism can be configured with. */
  enum Controller {
    /** A plain PID controller, run onboard the motor controller. */
    PID,
    /** A trapezoidal profile, run onboard (SPARK MAXMotion, Talon Motion Magic). */
    TRAPEZOIDAL_PROFILE,
    /** An exponential profile, run onboard on Talons and by YAMS on SPARKs. */
    EXPONENTIAL_PROFILE,
    /** An LQR, always run by YAMS. */
    LQR
  }

  /**
   * A mechanism config with the closed loop controller.
   *
   * @param name       Telemetry name.
   * @param controller Closed loop controller.
   * @return The config.
   */
  static SmartMotorControllerConfig config(String name, Controller controller) {
    SmartMotorControllerConfig config =
        new yams.commands2.config.SmartMotorControllerConfig()
            .withSubsystem(new SmartMotorControllerTestSubsystem())
            .withGearing(ContinuousWrappingTest.kGearing)
            .withStatorCurrentLimit(Amps.of(40))
            .withZeroPower(MotorMode.BRAKE)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withSimulationPeriod(Milliseconds.of(10))
            // Roughly 12 V over the free speed of a NEO or Kraken behind the 12:1 gearing, in volts
            // per
            // mechanism rotation per second, so the motion profiles can be followed.
            .withFeedforward(new SimpleMotorFeedforward(0, 1.5))
            .withTelemetry(name, TelemetryVerbosity.LOW);
    return switch (controller) {
      case PID -> config.withClosedLoopController(8, 0, 0);
      case TRAPEZOIDAL_PROFILE ->
          config
              .withClosedLoopController(8, 0, 0)
              .withTrapezoidalProfile(DegreesPerSecond.of(720), DegreesPerSecondPerSecond.of(1440));
      case EXPONENTIAL_PROFILE ->
          config
              .withClosedLoopController(8, 0, 0)
              .withExponentialProfile(
                  Volts.of(12), DegreesPerSecond.of(720), DegreesPerSecondPerSecond.of(1440));
      case LQR -> config.withClosedLoopController(ContinuousWrappingTest.lqr());
    };
  }

  /**
   * Create the motor controller and its encoder, closing the loop on the encoder.
   *
   * @param encoder        Encoder.
   * @param config         Mechanism config.
   * @param encoderOptions Options for the encoder, such as its discontinuity point or zero offset.
   * @return The motor controller.
   */
  static SmartMotorController create(
      Encoder encoder,
      SmartMotorControllerConfig config,
      UnaryOperator<SmartMotorControllerConfig> encoderOptions) {
    return switch (encoder) {
      case SPARK_MAX_ABSOLUTE, SPARK_FLEX_ABSOLUTE, SPARK_MAX_CAN, SPARK_FLEX_CAN -> {
        final boolean flex =
            encoder == Encoder.SPARK_FLEX_ABSOLUTE || encoder == Encoder.SPARK_FLEX_CAN;
        final SparkBase spark =
            flex ? DeviceCreator.createSparkFlex() : DeviceCreator.createSparkMax();
        final Object absoluteEncoder =
            encoder == Encoder.SPARK_MAX_CAN || encoder == Encoder.SPARK_FLEX_CAN
                ? DeviceCreator.createDetachedEncoder()
                : spark.getAbsoluteEncoder();
        yield new SparkWrapper(
            spark,
            flex ? DCMotor.getNeoVortex(1) : DCMotor.getNEO(1),
            encoderOptions.apply(
                config.withExternalEncoder(absoluteEncoder).withUseExternalFeedbackEncoder(true)));
      }
      case TALONFX_CANCODER, TALONFX_CANDI -> {
        final TalonFX talon = DeviceCreator.createTalonFX();
        if (encoder == Encoder.TALONFX_CANDI) {
          final CANdi candi = DeviceCreator.createCANdiFor(talon);
          config =
              config
                  .withVendorConfig(CANdiTest.vendorConfig(false, 1, candi))
                  .withExternalEncoder(candi);
        } else {
          config = config.withExternalEncoder(DeviceCreator.createCANcoderFor(talon));
        }
        yield new TalonFXWrapper(
            talon,
            DCMotor.getKrakenX60(1),
            encoderOptions.apply(config.withUseExternalFeedbackEncoder(true)));
      }
      case TALONFXS_CANCODER, TALONFXS_CANDI -> {
        final TalonFXS talon = DeviceCreator.createTalonFXS();
        if (encoder == Encoder.TALONFXS_CANDI) {
          final CANdi candi = DeviceCreator.createCANdiFor(talon);
          config =
              config
                  .withVendorConfig(CANdiTest.vendorConfig(true, 1, candi))
                  .withExternalEncoder(candi);
        } else {
          config = config.withExternalEncoder(DeviceCreator.createCANcoderFor(talon));
        }
        yield new TalonFXSWrapper(
            talon,
            DCMotor.getNEO(1),
            encoderOptions.apply(config.withUseExternalFeedbackEncoder(true)));
      }
      case NOVA_ABSOLUTE, NOVA_CAN -> {
        final Nova nova = DeviceCreator.createNova();
        final Object absoluteEncoder =
            encoder == Encoder.NOVA_CAN ? DeviceCreator.createCanEncoder() : FeedbackSensorType.ABS;
        yield new NovaWrapper(
            nova,
            DCMotor.getNEO(1),
            encoderOptions.apply(
                config.withExternalEncoder(absoluteEncoder).withUseExternalFeedbackEncoder(true)));
      }
    };
  }

  /**
   * Create a motor controller without an absolute encoder. A quadrature encoder closes the loop.
   *
   * @param feedback Its feedback sensor.
   * @param config   Mechanism config.
   * @return The motor controller.
   */
  static SmartMotorController create(RelativeFeedback feedback, SmartMotorControllerConfig config) {
    if (feedback.talon()) {
      return feedback == RelativeFeedback.TALONFX
          ? new TalonFXWrapper(DeviceCreator.createTalonFX(), DCMotor.getKrakenX60(1), config)
          : new TalonFXSWrapper(DeviceCreator.createTalonFXS(), DCMotor.getNEO(1), config);
    }
    if (feedback == RelativeFeedback.NOVA || feedback == RelativeFeedback.NOVA_QUADRATURE) {
      final Nova nova = DeviceCreator.createNova();
      try {
        return new NovaWrapper(
            nova,
            DCMotor.getNEO(1),
            feedback == RelativeFeedback.NOVA_QUADRATURE
                ? config
                    .withExternalEncoder(FeedbackSensorType.QUAD)
                    .withUseExternalFeedbackEncoder(true)
                : config);
      } catch (RuntimeException e) {
        // The Nova rejected the config; release it for the tests after this one.
        CommandScheduler.getInstance().unregisterSubsystem(config.getSubsystem());
        nova.close();
        throw e;
      }
    }
    final boolean flex =
        feedback == RelativeFeedback.SPARK_FLEX
            || feedback == RelativeFeedback.SPARK_FLEX_QUADRATURE;
    final SparkBase spark = flex ? DeviceCreator.createSparkFlex() : DeviceCreator.createSparkMax();
    try {
      final boolean quadrature =
          feedback == RelativeFeedback.SPARK_MAX_QUADRATURE
              || feedback == RelativeFeedback.SPARK_FLEX_QUADRATURE;
      return new SparkWrapper(
          spark,
          flex ? DCMotor.getNeoVortex(1) : DCMotor.getNEO(1),
          quadrature
              ? ThroughBoreEncoderTest.withThroughBore(
                      spark, ThroughBoreEncoderTest.Connection.QUADRATURE, config)
                  .withUseExternalFeedbackEncoder(true)
              : config);
    } catch (RuntimeException e) {
      // The SPARK rejected the config; put it back to factory defaults for the tests after this
      // one.
      CommandScheduler.getInstance().unregisterSubsystem(config.getSubsystem());
      ThroughBoreEncoderTest.closeDevices(spark, null);
      throw e;
    }
  }

  /**
   * The encoder's own absolute angle: what it reports, in the range set by its discontinuity point.
   * A Talon fuses its encoder with the motor's, so its mechanism position does not wrap; the
   * encoder's absolute position still does.
   *
   * @param smc Motor controller.
   * @return Absolute angle of the encoder, which is mounted on the mechanism 1:1.
   */
  static Angle absoluteAngle(SmartMotorController smc) {
    final Object encoder = smc.getConfig().getExternalEncoder().orElseThrow();
    if (encoder instanceof CANcoder cancoder) {
      return cancoder.getAbsolutePosition().refresh().getValue();
    }
    if (encoder instanceof CANdi candi) {
      return candi.getPWM1Position().refresh().getValue();
    }
    return smc.getExternalEncoderMechanismPosition().orElseThrow();
  }

  /** Let 10ms of real time pass. */
  private static void sleepOneStep() {
    try {
      Thread.sleep(10);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
  }

  /**
   * Hold the position setpoint in simulation.
   *
   * @param smc      Motor controller.
   * @param encoder  Its encoder.
   * @param setpoint Mechanism position setpoint.
   * @param duration How long to hold it.
   */
  static void runTo(SmartMotorController smc, Encoder encoder, Angle setpoint, Time duration) {
    run(smc, encoder.talon(), Optional.of(setpoint), duration);
  }

  /**
   * Run the simulation, holding a position setpoint if there is one.
   *
   * @param smc      Motor controller.
   * @param talon    Whether it is a Talon, which Phoenix simulates in real time.
   * @param setpoint Mechanism position setpoint, or empty to leave the mechanism where it is.
   * @param duration How long to run.
   */
  static void run(
      SmartMotorController smc, boolean talon, Optional<Angle> setpoint, Time duration) {
    ((SmartMotorControllerTestSubsystem)
            ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem())
        .setSMC(smc);
    smc.setupSimulation();
    try (PeriodicScheduler scheduler = new PeriodicScheduler()) {
      setpoint.ifPresent(smc::setPosition);
      scheduler.addPeriodic(smc::simIterate, Milliseconds.of(10));
      if (talon) {
        // Phoenix simulates a Talon on its own thread in real time; keep the simulation in step.
        scheduler.addPeriodic(AbsoluteEncoderCases::sleepOneStep, Milliseconds.of(10));
      }
      scheduler.addPeriodic(
          () -> {
            setpoint.ifPresent(smc::setPosition);
            smc.updateTelemetry();
          },
          Milliseconds.of(20));
      scheduler.runFor(duration);
    }
  }

  /**
   * Wait for a configuration applied asynchronously, as a SPARK's is.
   *
   * @param condition Whether it has been applied.
   * @return Whether it was applied within half a second.
   */
  static boolean eventually(BooleanSupplier condition) throws InterruptedException {
    for (int i = 0; i < 50; i++) {
      if (condition.getAsBoolean()) {
        return true;
      }
      Thread.sleep(10);
    }
    return condition.getAsBoolean();
  }

  /**
   * Close the motor controller, and put it and its encoder, if any, back to factory defaults.
   *
   * @param smc Motor controller.
   */
  static void close(SmartMotorController smc) {
    SmartMotorControllerTestSubsystem subsys =
        (SmartMotorControllerTestSubsystem)
            ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem();
    SmartMotorControllerCommandRegistry.removeCommands(subsys);
    CommandScheduler.getInstance().unregisterSubsystem(subsys);
    if (subsys.smc != null) {
      subsys.close();
    }
    smc.close();
    DeviceCreator.silence(smc);
    final Object encoder = smc.getConfig().getExternalEncoder().orElse(null);
    if (encoder instanceof CANcoder cancoder) {
      cancoder.getConfigurator().apply(new CANcoderConfiguration());
      cancoder.close();
    }
    if (encoder instanceof CanEncoder canEncoder) {
      canEncoder.close();
    }
    if (encoder instanceof CANdi candi) {
      CANdiTest.closeDevices(smc.getMotorController(), candi);
    } else if (encoder instanceof DetachedEncoder || smc instanceof SparkWrapper) {
      ThroughBoreEncoderTest.closeDevices(smc.getMotorController(), encoder);
    } else if (smc.getMotorController() instanceof TalonFXS talonFXS) {
      talonFXS.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXSConfiguration());
      talonFXS.close();
    } else if (smc.getMotorController() instanceof TalonFX talonFX) {
      talonFX.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());
      talonFX.close();
    } else if (smc.getMotorController() instanceof Nova nova) {
      // A Nova is put back to factory defaults when YAMS configures it, so only close it.
      nova.close();
    }
  }
}
