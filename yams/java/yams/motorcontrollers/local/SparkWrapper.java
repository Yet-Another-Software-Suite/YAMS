package yams.motorcontrollers.local;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.util.StatusLogger;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import yams.exceptions.SmartMotorControllerConfigurationException;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.simulation.DCMotorSimSupplier;
import yams.telemetry.SmartMotorControllerTelemetry.BooleanTelemetryField;
import yams.telemetry.SmartMotorControllerTelemetry.DoubleTelemetryField;

/**
 * Spark wrapper for REV Spark Motor controllers.
 */
public class SparkWrapper extends SmartMotorController
{

  /**
   * Spark configuration retry count.
   */
  private final int                               CONFIG_RETRIES            = 4;
  /**
   * Spark configuration retry delay.
   */
  private final double                            CONFIG_RETRY_DELAY        = Milliseconds.of(5).in(Seconds);
  /**
   * Spark motor controller
   */
  private final SparkBase                         m_spark;
  /**
   * Spark Closed loop controller.
   */
  private final SparkClosedLoopController         m_sparkPidController;
  /**
   * Motor type.
   */
  private final DCMotor                           m_motor;
  /**
   * Spark base configuration.
   */
  private final SparkBaseConfig                   m_sparkBaseConfig;
  /**
   * Spark relative encoder.
   */
  private final RelativeEncoder                   m_sparkRelativeEncoder;
  /**
   * Spark relative encoder sim object.
   */
  private       Optional<SparkRelativeEncoderSim> sparkRelativeEncoderSim   = Optional.empty();
  /**
   * Spark simulation.
   */
  private       Optional<SparkSim>                sparkSim                  = Optional.empty();
  /**
   * Spark absolute encoder.
   */
  private       Optional<AbsoluteEncoder>         m_sparkAbsoluteEncoder    = Optional.empty();
  /**
   * Spark absolute encoder sim object
   */
  private       Optional<SparkAbsoluteEncoderSim> m_sparkAbsoluteEncoderSim = Optional.empty();
  /**
   * DC Motor Sim.
   */
  private       Optional<DCMotorSim>              m_dcMotorSim              = Optional.empty();

  /**
   * Create a {@link SmartMotorController} from {@link SparkMax} or {@link SparkFlex}
   *
   * @param controller {@link SparkMax} or {@link SparkFlex}
   * @param motor      {@link DCMotor} controller by the {@link SparkFlex} or {@link SparkMax}. Must be a brushless
   *                   motor.
   * @param config     {@link SmartMotorControllerConfig} to apply.
   */
  public SparkWrapper(SparkBase controller, DCMotor motor, SmartMotorControllerConfig config)
  {
    if (controller instanceof SparkMax)
    {
      m_sparkBaseConfig = new SparkMaxConfig();
    } else if (controller instanceof SparkFlex)
    {
      m_sparkBaseConfig = new SparkFlexConfig();
    } else
    {
      throw new IllegalArgumentException(
          "[ERROR] Unsupported controller type: " + controller.getClass().getSimpleName());
    }

    this.m_motor = motor;
    m_spark = controller;
    m_sparkPidController = m_spark.getClosedLoopController();
    this.m_config = config;
    m_sparkRelativeEncoder = controller.getEncoder();
    setupSimulation();
    applyConfig(config);
    checkConfigSafety();

  }

  /**
   * Run the configuration until it succeeds or times out.
   *
   * @param config Lambda supplier returning the error state.
   * @return Successful configuration
   */
  private boolean configureSpark(Supplier<REVLibError> config)
  {
    for (int i = 0; i < CONFIG_RETRIES; i++)
    {
      if (config.get() == REVLibError.kOk)
      {
        return true;
      }
      Timer.delay(CONFIG_RETRY_DELAY);
    }
    return false;
  }

  @Override
  public void setupSimulation()
  {
    if (RobotBase.isSimulation())
    {
      var setupRan = sparkSim.isPresent();
      if (!setupRan)
      {
        sparkSim = Optional.of(new SparkSim(m_spark, m_motor));
        sparkRelativeEncoderSim = Optional.of(sparkSim.get().getRelativeEncoderSim());
        m_dcMotorSim = Optional.of(new DCMotorSim(LinearSystemId.createDCMotorSystem(m_motor,
                                                                                     0.001,
                                                                                     m_config.getGearing()
                                                                                             .getRotorToMechanismRatio()),
                                                  m_motor,
                                                  1.0 / 1024.0, 0));
        setSimSupplier(new DCMotorSimSupplier(m_dcMotorSim.get(), this));
      }
      m_config.getStartingPosition().ifPresent(startingPos -> {
        sparkSim.get().setPosition(startingPos.in(Rotations));
        sparkRelativeEncoderSim.get().setPosition(startingPos.in(Rotations));
      });
    }
  }


  @Override
  public void seedRelativeEncoder()
  {
    if (m_sparkAbsoluteEncoder.isPresent())
    {
      m_sparkRelativeEncoder.setPosition(m_sparkAbsoluteEncoder.get().getPosition());
      sparkRelativeEncoderSim.ifPresent(sparkRelativeEncoderSim -> sparkRelativeEncoderSim.setPosition(
          m_sparkAbsoluteEncoder.get().getPosition()));
    }
  }

  @Override
  public void synchronizeRelativeEncoder()
  {
    if (m_config.getFeedbackSynchronizationThreshold().isPresent())
    {
      if (m_sparkAbsoluteEncoder.isPresent())
      {
        if (!Rotations.of(m_sparkRelativeEncoder.getPosition()).isNear(Rotations.of(m_sparkAbsoluteEncoder.get()
                                                                                                          .getPosition()),
                                                                       m_config.getFeedbackSynchronizationThreshold()
                                                                               .get()))
        {
          seedRelativeEncoder();
        }
      }
    }
  }

  @Override
  public void simIterate()
  {
    if (RobotBase.isSimulation() && m_simSupplier.isPresent())
    {
      if (!m_simSupplier.get().getUpdatedSim())
      {
        m_simSupplier.get().updateSimState();
        m_simSupplier.get().starveUpdateSim();
      }
      Time controlLoop = m_config.getClosedLoopControlPeriod().orElse(Milliseconds.of(20));
      m_simSupplier.ifPresent(mSimSupplier -> {
        sparkSim.ifPresent(sim -> sim.iterate(mSimSupplier.getMechanismVelocity().in(RotationsPerSecond),
                                              mSimSupplier.getMechanismSupplyVoltage().in(Volts),
                                              controlLoop.in(Second)));
        sparkRelativeEncoderSim.ifPresent(sim -> sim.iterate(mSimSupplier.getMechanismVelocity()
                                                                         .in(RotationsPerSecond),
                                                             controlLoop.in(Seconds)));
        m_sparkAbsoluteEncoderSim.ifPresent(absoluteEncoderSim ->
                                                absoluteEncoderSim.iterate(mSimSupplier.getMechanismVelocity()
                                                                                       .in(RotationsPerSecond),
                                                                           controlLoop.in(Seconds)));
      });
    }
  }

  @Override
  public void setIdleMode(MotorMode mode)
  {
    m_sparkBaseConfig.idleMode(mode == MotorMode.BRAKE ? IdleMode.kBrake : IdleMode.kCoast);
    configureSpark(() -> m_spark.configure(m_sparkBaseConfig,
                                           ResetMode.kNoResetSafeParameters,
                                           PersistMode.kPersistParameters));
  }

  @Override
  public void setEncoderVelocity(LinearVelocity velocity)
  {
    setEncoderVelocity(m_config.convertToMechanism(velocity));
  }

  @Override
  public void setEncoderPosition(Angle angle)
  {
    if (m_sparkAbsoluteEncoder.isPresent())
    {
      m_sparkBaseConfig.absoluteEncoder.zeroOffset(getMechanismPosition().minus(angle).in(Rotations));
      m_sparkAbsoluteEncoderSim.ifPresent(absoluteEncoderSim -> absoluteEncoderSim.setPosition(angle.in(Rotations)));
    }
    m_sparkRelativeEncoder.setPosition(angle.in(Rotations));
    sparkRelativeEncoderSim.ifPresent(relativeEncoderSim -> relativeEncoderSim.setPosition(angle.in(Rotations)));
    m_simSupplier.ifPresent(simSupplier -> simSupplier.setMechanismPosition(angle));
  }

  @Override
  public void setEncoderVelocity(AngularVelocity velocity)
  {
    sparkRelativeEncoderSim.ifPresent(relativeEncoderSim -> relativeEncoderSim.setVelocity(velocity.in(
        RotationsPerSecond)));
    m_sparkAbsoluteEncoderSim.ifPresent(absoluteEncoderSim -> absoluteEncoderSim.setVelocity(velocity.in(
        RotationsPerSecond)));
  }

  @Override
  public void setEncoderPosition(Distance distance)
  {
    setEncoderPosition(m_config.convertToMechanism(distance));
  }

  @Override
  public void setPosition(Angle angle)
  {
    setpointPosition = Optional.ofNullable(angle);
    if (m_expoPidController.isEmpty() && angle != null)
    {
      m_sparkPidController.setSetpoint(angle.in(Rotations),
                                       ControlType.kMAXMotionPositionControl,
                                       ClosedLoopSlot.kSlot0);
    }
  }

  @Override
  public void setPosition(Distance distance)
  {
    setPosition(m_config.convertToMechanism(distance));
  }

  @Override
  public void setVelocity(LinearVelocity velocity)
  {
    setVelocity(m_config.convertToMechanism(velocity));
  }

  @Override
  public void setVelocity(AngularVelocity angle)
  {
    setpointVelocity = Optional.ofNullable(angle);
    if (m_expoPidController.isEmpty() && angle != null)
    {
      m_sparkPidController.setSetpoint(angle.in(RotationsPerSecond),
                                       ControlType.kVelocity,
                                       ClosedLoopSlot.kSlot0);
    }
  }

  @Override
  public boolean applyConfig(SmartMotorControllerConfig config)
  {
    config.resetValidationCheck();

    if (m_spark.isFollower())
    {
      m_spark.pauseFollowerMode();
      m_sparkBaseConfig.disableFollowerMode();
    }
    m_expoPidController = config.getExponentiallyProfiledClosedLoopController();
    m_pidController = config.getClosedLoopController();
    m_simplePidController = config.getSimpleClosedLoopController();

    // Handle simple pid vs profile pid controller.
    if (m_expoPidController.isEmpty())
    {
      if (m_pidController.isEmpty())
      {
        if (m_simplePidController.isEmpty())
        {
          if (config.getMotorControllerMode() == ControlMode.CLOSED_LOOP)
          {throw new IllegalArgumentException("[ERROR] closed loop controller must not be empty");}
        }
      } else if (config.getSimpleClosedLoopController().isPresent())
      {
        throw new SmartMotorControllerConfigurationException("ProfiledPIDController and PIDController defined",
                                                             "Cannot have both PID Controllers.",
                                                             ".withClosedLoopController");
      }
    }

    // Handle closed loop controller thread
    if (m_expoPidController.isPresent())
    {

      config.getClosedLoopTolerance().ifPresent(tolerance -> {
        // Set closed loop tolerance and profile tolerance to the same thing.
        if (config.getMechanismCircumference().isPresent())
        {
//          m_pidController.ifPresent(pidController -> pidController.setTolerance(config.convertFromMechanism(tolerance)
//                                                                                      .in(Meters)));
//          m_simplePidController.ifPresent(pidController -> pidController.setTolerance(config.convertFromMechanism(
//              tolerance).in(Meters)));
          m_expoPidController.ifPresent(pidController -> pidController.setTolerance(config.convertFromMechanism(
                                                                                              tolerance)
                                                                                          .in(Meters)));
        } else
        {
//          m_pidController.ifPresent(pidController -> pidController.setTolerance(tolerance.in(Rotations)));
//          m_simplePidController.ifPresent(pidController -> pidController.setTolerance(tolerance.in(Rotations)));
          m_expoPidController.ifPresent(pidController -> pidController.setTolerance(tolerance.in(Rotations)));
        }
      });

      iterateClosedLoopController();

      if (m_closedLoopControllerThread == null)
      {
        m_closedLoopControllerThread = new Notifier(this::iterateClosedLoopController);
      } else
      {
        stopClosedLoopController();
        m_closedLoopControllerThread.stop();
        m_closedLoopControllerThread.close();
        m_closedLoopControllerThread = new Notifier(this::iterateClosedLoopController);
      }

      if (config.getTelemetryName().isPresent())
      {
        m_closedLoopControllerThread.setName(config.getTelemetryName().get());
      }
      if (config.getMotorControllerMode() == ControlMode.CLOSED_LOOP)
      {
        startClosedLoopController();
      } else
      {
        m_closedLoopControllerThread.stop();
        if (config.getClosedLoopControlPeriod().isPresent())
        {
          throw new IllegalArgumentException("[Error] Closed loop control period is only supported in closed loop mode.");
        }
      }
    }

    // Calculate Spark conversion factors
    double positionConversionFactor = config.getGearing().getRotorToMechanismRatio();
    double velocityConversionFactor = config.getGearing().getRotorToMechanismRatio() / 60.0;

    // Set base config options
    m_sparkBaseConfig.openLoopRampRate(config.getOpenLoopRampRate().in(Seconds))
                     .closedLoopRampRate(config.getClosedLoopRampRate().in(Seconds))
                     .inverted(config.getMotorInverted());
    m_sparkBaseConfig.encoder.positionConversionFactor(positionConversionFactor)
                             .velocityConversionFactor(velocityConversionFactor);

    // Control mode is ignored
    config.getMotorControllerMode();

    // Set motion profile and PID values.
    config.getClosedLoopController().ifPresent(pidController -> {
      var maxAccel = pidController.getConstraints().maxAcceleration;
      var maxVel   = pidController.getConstraints().maxVelocity;
      // Distance based mechanism, converting from meters to rotations.
      if (config.getMechanismCircumference().isPresent())
      {
        maxVel = config.convertToMechanism(MetersPerSecond.of(maxVel)).in(RotationsPerSecond);
        maxAccel = config.convertToMechanism(MetersPerSecondPerSecond.of(maxAccel)).in(RotationsPerSecondPerSecond);
      }

      m_sparkBaseConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(pidController.getP(),
               pidController.getI(),
               pidController.getD(),
               ClosedLoopSlot.kSlot0)
          .maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                    .cruiseVelocity(maxVel)
                    .maxAcceleration(maxAccel);
    });
    // Set simple closed loop controller values
    config.getSimpleClosedLoopController().ifPresent(pidController -> {
      m_sparkBaseConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(pidController.getP(),
               pidController.getI(),
               pidController.getD(),
               ClosedLoopSlot.kSlot0);
    });

    // Set feedforward values
    config.getArmFeedforward().ifPresent(ff -> {
      m_sparkBaseConfig.closedLoop.feedForward
          .kS(ff.getKs())
          .kV(ff.getKv())
          .kA(ff.getKa())
          .kCos(ff.getKg());
    });
    config.getElevatorFeedforward().ifPresent(ff -> {
      m_sparkBaseConfig.closedLoop.feedForward
          .kS(ff.getKs())
          .kV(ff.getKv())
          .kA(ff.getKa())
          .kG(ff.getKg());
    });
    config.getSimpleFeedforward().ifPresent(ff -> {
      m_sparkBaseConfig.closedLoop.feedForward
          .kS(ff.getKs())
          .kV(ff.getKv())
          .kA(ff.getKa());
    });

    // Set closed loop tolerance and profile tolerance to the same thing.
    config.getClosedLoopTolerance().ifPresent(tolerance -> {
      m_sparkBaseConfig.closedLoop.allowedClosedLoopError(tolerance.in(Rotations), ClosedLoopSlot.kSlot0);
      if (m_pidController.isPresent())
      {
        m_sparkBaseConfig.closedLoop.maxMotion.allowedProfileError(tolerance.in(Rotations), ClosedLoopSlot.kSlot0);
      }
    });

    // Set Mechanism Limits
    config.getMechanismLowerLimit().ifPresent(lowerLimit -> {
      m_sparkBaseConfig.softLimit.reverseSoftLimit(lowerLimit.in(Rotations)).reverseSoftLimitEnabled(true);
    });
    config.getMechanismUpperLimit().ifPresent(upperLimit -> {
      m_sparkBaseConfig.softLimit.forwardSoftLimit(upperLimit.in(Rotations)).forwardSoftLimitEnabled(true);
    });

    // Throw warning about supply stator limits on Spark's
    if (config.getSupplyStallCurrentLimit().isPresent())
    {
      DriverStation.reportError("[WARNING] Supply stall currently not supported on Spark", true);
    }
    // Handle stator current limit.
    if (config.getStatorStallCurrentLimit().isPresent())
    {
      m_sparkBaseConfig.smartCurrentLimit(config.getStatorStallCurrentLimit().getAsInt());
    }
    // Handle voltage compensation.
    if (config.getVoltageCompensation().isPresent())
    {
      m_sparkBaseConfig.voltageCompensation(config.getVoltageCompensation().get().in(Volts));
    }
    // Setup idle mode.
    if (config.getIdleMode().isPresent())
    {
      m_sparkBaseConfig.idleMode(config.getIdleMode().get() == MotorMode.BRAKE ? IdleMode.kBrake : IdleMode.kCoast);
    }
    // Setup starting position
    if (config.getStartingPosition().isPresent())
    {
      m_sparkRelativeEncoder.setPosition(config.getStartingPosition().get().in(Rotations));
    }
    // PID Wrapping
    if (config.getMaxDiscontinuityPoint().isPresent())
    {
      m_sparkBaseConfig.closedLoop
          .positionWrappingMaxInput(config.getMaxDiscontinuityPoint().get().in(Rotations))
          .positionWrappingEnabled(true);
    }

    // Setup external encoder.
    boolean useExternalEncoder = config.getUseExternalFeedback();
    if (config.getExternalEncoder().isPresent() && useExternalEncoder)
    {
      Object externalEncoder = config.getExternalEncoder().get();
      if (externalEncoder instanceof SparkAbsoluteEncoder)
      {
        double absoluteEncoderConversionFactor = config.getExternalEncoderGearing().getRotorToMechanismRatio();
        m_sparkAbsoluteEncoder = Optional.of((SparkAbsoluteEncoder) externalEncoder);
        m_sparkBaseConfig.absoluteEncoder.positionConversionFactor(absoluteEncoderConversionFactor)
                                         .velocityConversionFactor(absoluteEncoderConversionFactor / 60)
                                         .inverted(config.getExternalEncoderInverted());
        // Set the absolute encoder as the primary feedback sensor for closed loop control.
        m_sparkBaseConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        if (config.getZeroOffset().isPresent())
        {
          m_sparkBaseConfig.absoluteEncoder.zeroOffset(config.getZeroOffset().get().in(Rotations));
        }

        if (RobotBase.isSimulation())
        {
          if (m_spark instanceof SparkMax)
          {
            m_sparkAbsoluteEncoderSim = Optional.of(new SparkAbsoluteEncoderSim((SparkMax) m_spark));
          } else if (m_spark instanceof SparkFlex)
          {
            m_sparkAbsoluteEncoderSim = Optional.of(new SparkAbsoluteEncoderSim((SparkFlex) m_spark));
          }
          if (config.getStartingPosition().isPresent())
          {
            m_sparkAbsoluteEncoderSim.ifPresent(enc -> enc.setPosition(config.getStartingPosition().get()
                                                                             .in(Rotations)));
          }
          if (config.getZeroOffset().isPresent())
          {
            m_sparkAbsoluteEncoderSim.ifPresent(enc -> enc.setZeroOffset(config.getZeroOffset().get().in(Rotations)));
          }
        }
      } else
      {
        throw new IllegalArgumentException(
            "[ERROR] Unsupported external encoder: " + externalEncoder.getClass().getSimpleName());
      }

      // Set starting position if external encoder is empty.
      if (config.getStartingPosition().isEmpty())
      {
        m_sparkRelativeEncoder.setPosition(m_sparkAbsoluteEncoder.get().getPosition());
      }

    } else
    {

      if (config.getZeroOffset().isPresent())
      {
        throw new SmartMotorControllerConfigurationException("Zero offset is only available for external encoders",
                                                             "Zero offset could not be applied",
                                                             ".withExternalEncoderZeroOffset");
      }

      if (config.getExternalEncoderInverted())
      {
        throw new SmartMotorControllerConfigurationException(
            "External encoder cannot be inverted because no external encoder exists",
            "External encoder could not be inverted",
            "withExternalEncoderInverted");
      }

      if (config.getExternalEncoderGearing().getRotorToMechanismRatio() != 1.0)
      {
        throw new SmartMotorControllerConfigurationException(
            "External encoder gearing is not supported when there is no external encoder",
            "External encoder gearing could not be set",
            "withExternalEncoderGearing");
      }
    }

    // Configure follower motors
    if (config.getFollowers().isPresent())
    {
      for (Pair<Object, Boolean> follower : config.getFollowers().get())
      {
        if (follower.getFirst() instanceof SparkMax)
        {
          ((SparkMax) follower.getFirst()).configure(new SparkMaxConfig().follow(m_spark, follower.getSecond()),
                                                     ResetMode.kNoResetSafeParameters,
                                                     PersistMode.kPersistParameters);

        } else if (follower.getFirst() instanceof SparkFlex)
        {
          ((SparkFlex) follower.getFirst()).configure(new SparkFlexConfig().follow(m_spark, follower.getSecond()),
                                                      ResetMode.kNoResetSafeParameters,
                                                      PersistMode.kPersistParameters);

        } else
        {
          throw new IllegalArgumentException(
              "[ERROR] Unknown follower type: " + follower.getFirst().getClass().getSimpleName());
        }
      }
      config.clearFollowers();
    }

    if (config.getZeroOffset().isPresent() && config.getExternalEncoder().isEmpty() && !useExternalEncoder)
    {
      throw new SmartMotorControllerConfigurationException("Zero offset is only available for external encoders",
                                                           "Zero offset could not be applied",
                                                           ".withZeroOffset");
    }

    if (config.getExternalEncoderInverted() && config.getExternalEncoder().isEmpty() && !useExternalEncoder)
    {
      throw new SmartMotorControllerConfigurationException(
          "External encoder cannot be inverted because no external encoder exists",
          "External encoder could not be inverted",
          "withExternalEncoderInverted");
    }

    if (config.getExternalEncoderGearing().getRotorToMechanismRatio() != 1.0 && config.getExternalEncoder().isEmpty() &&
        !useExternalEncoder)
    {
      throw new SmartMotorControllerConfigurationException(
          "External encoder gearing is not supported when there is no external encoder",
          "External encoder gearing could not be set",
          "withExternalEncoderGearing");
    }

    if (config.getClosedLoopControlPeriod().isPresent() && m_expoPidController.isEmpty())
    {
      throw new SmartMotorControllerConfigurationException(
          "Closed loop control period is unsupported without Exponential Profiles",
          "Closed loop control period does not take affect",
          ".withClosedLoopControlPeriod");
    }

    if (config.getClosedLoopControllerMaximumVoltage().isPresent() &&
        config.getExponentiallyProfiledClosedLoopController().isEmpty())
    {
      throw new SmartMotorControllerConfigurationException(
          "Closed loop controller maximum voltage is only available for Exponential Profiled closed loop controllers",
          "Closed loop controller maximum voltage could not be applied",
          "withClosedLoopControllerMaximumVoltage");
    }

    if (config.getTemperatureCutoff().isPresent() && config.getExponentiallyProfiledClosedLoopController().isEmpty())
    {
      throw new SmartMotorControllerConfigurationException(
          "Temperature cutoff is only available for exponentially profiled closed loop controllers",
          "Temperature cutoff could not be applied",
          "withTemperatureCutoff");
    }

    if (config.getFeedbackSynchronizationThreshold().isPresent() &&
        config.getExponentiallyProfiledClosedLoopController().isEmpty())
    {
      throw new SmartMotorControllerConfigurationException(
          "Feedback synchronization threshold is only available for exponentially profiled closed loop controllers",
          "Feedback synchronization threshold could not be applied",
          "withFeedbackSynchronizationThreshold");
    }

    if (config.getEncoderInverted())
    {
      throw new IllegalArgumentException("[ERROR] Spark relative encoder cannot be inverted!");
    }

    config.validateBasicOptions();
    config.validateExternalEncoderOptions();
    return configureSpark(() -> m_spark.configure(m_sparkBaseConfig,
                                                  ResetMode.kNoResetSafeParameters,
                                                  PersistMode.kPersistParameters));
  }

  @Override
  public double getDutyCycle()
  {
    return m_spark.getAppliedOutput();/* m_simSupplier.map(simSupplier -> simSupplier.getMechanismStatorVoltage().in(Volts) /
                                            simSupplier.getMechanismSupplyVoltage().in(Volts))
                        .orElseGet(spark::getAppliedOutput);*/
  }

  @Override
  public void setDutyCycle(double dutyCycle)
  {
    m_spark.set(dutyCycle);
//    m_simSupplier.ifPresent(simSupplier -> simSupplier.setMechanismStatorDutyCycle(dutyCycle));
  }

  @Override
  @Deprecated
  public Optional<Current> getSupplyCurrent()
  {
    return Optional.empty();
//    DriverStation.reportError("[WARNING] Supply currently not supported on Spark", true);
//    return null;
  }

  @Override
  public Current getStatorCurrent()
  {
    return m_simSupplier.isPresent() ? m_simSupplier.get().getCurrentDraw() : Amps.of(m_spark.getOutputCurrent());
  }

  @Override
  public Voltage getVoltage()
  {
    return m_simSupplier.isPresent() ? m_simSupplier.get().getMechanismStatorVoltage() : Volts.of(
        m_spark.getAppliedOutput() * m_spark.getBusVoltage());
  }

  @Override
  public void setVoltage(Voltage voltage)
  {
    m_spark.setVoltage(voltage);
    m_simSupplier.ifPresent(simSupplier -> simSupplier.setMechanismStatorVoltage(voltage));
  }

  @Override
  public DCMotor getDCMotor()
  {
    return m_motor;
  }

  @Override
  public LinearVelocity getMeasurementVelocity()
  {
    return m_config.convertFromMechanism(getMechanismVelocity());
  }

  @Override
  public Distance getMeasurementPosition()
  {
    return m_config.convertFromMechanism(getMechanismPosition());
  }

  @Override
  public AngularVelocity getMechanismVelocity()
  {
    if (m_sparkAbsoluteEncoder.isPresent() && m_config.getUseExternalFeedback())
    {
      return RotationsPerSecond.of(m_sparkAbsoluteEncoder.get().getVelocity());
    }
    return RotationsPerSecond.of(sparkSim.map(SparkSim::getVelocity)
                                         .orElseGet(m_sparkRelativeEncoder::getVelocity));
  }

  @Override
  public Angle getMechanismPosition()
  {
    Angle pos = Rotations.of(m_sparkRelativeEncoder.getPosition());
    if (m_sparkAbsoluteEncoder.isPresent() && m_config.getUseExternalFeedback())
    {
      pos = Rotations.of(m_sparkAbsoluteEncoder.get().getPosition());
    }
    return pos;
  }

  @Override
  public AngularVelocity getRotorVelocity()
  {
    return RotationsPerSecond.of(
        getMechanismPosition().in(Rotations) * m_config.getGearing().getMechanismToRotorRatio());
  }

  @Override
  public Angle getRotorPosition()
  {
    return Rotations.of(getMechanismPosition().in(Rotations) * m_config.getGearing().getMechanismToRotorRatio());
  }

  @Override
  public void setMotorInverted(boolean inverted)
  {
    m_config.withMotorInverted(inverted);
    m_sparkBaseConfig.inverted(inverted);
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setEncoderInverted(boolean inverted)
  {
    m_config.withEncoderInverted(inverted);
//    if (sparkAbsoluteEncoder.isPresent())
//    {
//      sparkBaseConfig.absoluteEncoder.inverted(inverted);
//    }
//    sparkBaseConfig.analogSensor.inverted(inverted);
    m_sparkBaseConfig.encoder.inverted(inverted);
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setMotionProfileMaxVelocity(LinearVelocity maxVelocity)
  {
    if (m_pidController.isPresent())
    {
      ProfiledPIDController ctr = m_pidController.get();
      ctr.setConstraints(new Constraints(maxVelocity.in(MetersPerSecond), ctr.getConstraints().maxAcceleration));
      m_config.withClosedLoopController(ctr);
      m_pidController = Optional.of(ctr);
    }
    m_sparkBaseConfig.closedLoop.maxMotion.cruiseVelocity(m_config.convertToMechanism(maxVelocity)
                                                                  .in(RotationsPerSecond));
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setMotionProfileMaxAcceleration(LinearAcceleration maxAcceleration)
  {
    if (m_pidController.isPresent())
    {
      ProfiledPIDController ctr = m_pidController.get();
      ctr.setConstraints(new Constraints(ctr.getConstraints().maxVelocity,
                                         maxAcceleration.in(MetersPerSecondPerSecond)));
      m_config.withClosedLoopController(ctr);
      m_pidController = Optional.of(ctr);
    }
    m_sparkBaseConfig.closedLoop.maxMotion.maxAcceleration(m_config.convertToMechanism(maxAcceleration)
                                                                   .in(RotationsPerSecondPerSecond));
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setMotionProfileMaxVelocity(AngularVelocity maxVelocity)
  {
    if (m_pidController.isPresent())
    {
      ProfiledPIDController ctr = m_pidController.get();
      ctr.setConstraints(new Constraints(maxVelocity.in(RotationsPerSecond), ctr.getConstraints().maxAcceleration));
      m_config.withClosedLoopController(ctr);
      m_pidController = Optional.of(ctr);
    }
    m_sparkBaseConfig.closedLoop.maxMotion.cruiseVelocity(maxVelocity.in(RotationsPerSecond));
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setMotionProfileMaxAcceleration(AngularAcceleration maxAcceleration)
  {
    if (m_pidController.isPresent())
    {
      ProfiledPIDController ctr = m_pidController.get();
      ctr.setConstraints(new Constraints(ctr.getConstraints().maxVelocity,
                                         maxAcceleration.in(RotationsPerSecondPerSecond)));
      m_config.withClosedLoopController(ctr);
      m_pidController = Optional.of(ctr);
    }
    m_sparkBaseConfig.closedLoop.maxMotion.maxAcceleration(maxAcceleration.in(RotationsPerSecondPerSecond));
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setKp(double kP)
  {
    m_simplePidController.ifPresent(simplePidController -> {
      simplePidController.setP(kP);
    });
    m_pidController.ifPresent(pidController -> {
      pidController.setP(kP);
    });
    m_expoPidController.ifPresent(expoPidController -> {expoPidController.setP(kP);});
    m_sparkBaseConfig.closedLoop.p(kP);
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setKi(double kI)
  {
    m_simplePidController.ifPresent(simplePidController -> {
      simplePidController.setI(kI);
    });
    m_pidController.ifPresent(pidController -> {
      pidController.setI(kI);
    });
    m_expoPidController.ifPresent(expoPidController -> {expoPidController.setI(kI);});
    m_sparkBaseConfig.closedLoop.i(kI);
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void setKd(double kD)
  {
    m_simplePidController.ifPresent(simplePidController -> {
      simplePidController.setD(kD);
    });
    m_pidController.ifPresent(pidController -> {
      pidController.setD(kD);
    });
    m_expoPidController.ifPresent(expoPidController -> {expoPidController.setD(kD);});
    m_sparkBaseConfig.closedLoop.d(kD);
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setFeedback(double kP, double kI, double kD)
  {
    m_simplePidController.ifPresent(simplePidController -> {
      simplePidController.setP(kP);
      simplePidController.setI(kI);
      simplePidController.setD(kD);
    });
    m_pidController.ifPresent(pidController -> {
      pidController.setP(kP);
      pidController.setI(kI);
      pidController.setD(kD);
    });
    m_expoPidController.ifPresent(expoPidController -> {
      expoPidController.setP(kP);
      expoPidController.setI(kI);
      expoPidController.setD(kD);
    });
    m_sparkBaseConfig.closedLoop.pid(kP, kI, kD);
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setKs(double kS)
  {
    m_config.getSimpleFeedforward().ifPresent(simpleMotorFeedforward -> {
      simpleMotorFeedforward.setKs(kS);
    });
    m_config.getArmFeedforward().ifPresent(armFeedforward -> {
      armFeedforward.setKs(kS);
    });
    m_config.getElevatorFeedforward().ifPresent(elevatorFeedforward -> {
      elevatorFeedforward.setKs(kS);
    });
    m_sparkBaseConfig.closedLoop.feedForward.kS(kS);
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setKv(double kV)
  {
    m_config.getSimpleFeedforward().ifPresent(simpleMotorFeedforward -> {
      simpleMotorFeedforward.setKv(kV);
    });
    m_config.getArmFeedforward().ifPresent(armFeedforward -> {
      armFeedforward.setKv(kV);
    });
    m_config.getElevatorFeedforward().ifPresent(elevatorFeedforward -> {
      elevatorFeedforward.setKv(kV);
    });
    m_sparkBaseConfig.closedLoop.feedForward.kV(kV);
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setKa(double kA)
  {
    m_config.getSimpleFeedforward().ifPresent(simpleMotorFeedforward -> {
      simpleMotorFeedforward.setKa(kA);
    });
    m_config.getArmFeedforward().ifPresent(armFeedforward -> {
      armFeedforward.setKa(kA);
    });
    m_config.getElevatorFeedforward().ifPresent(elevatorFeedforward -> {
      elevatorFeedforward.setKa(kA);
    });
    m_sparkBaseConfig.closedLoop.feedForward.kA(kA);
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setKg(double kG)
  {
    m_config.getArmFeedforward().ifPresent(armFeedforward -> {
      armFeedforward.setKg(kG);
    });
    m_config.getElevatorFeedforward().ifPresent(elevatorFeedforward -> {
      elevatorFeedforward.setKg(kG);
    });
    if (m_config.getArmFeedforward().isEmpty())
    {
      m_sparkBaseConfig.closedLoop.feedForward.kG(kG);
    } else
    {
      m_sparkBaseConfig.closedLoop.feedForward.kCos(kG);
    }
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setFeedforward(double kS, double kV, double kA, double kG)
  {
    m_config.getSimpleFeedforward().ifPresent(simpleMotorFeedforward -> {
      simpleMotorFeedforward.setKs(kS);
      simpleMotorFeedforward.setKv(kV);
      simpleMotorFeedforward.setKa(kA);
    });
    m_config.getArmFeedforward().ifPresent(armFeedforward -> {
      armFeedforward.setKs(kS);
      armFeedforward.setKv(kV);
      armFeedforward.setKa(kA);
      armFeedforward.setKg(kG);
      m_sparkBaseConfig.closedLoop.feedForward.kCos(kG);
    });
    m_config.getElevatorFeedforward().ifPresent(elevatorFeedforward -> {
      elevatorFeedforward.setKs(kS);
      elevatorFeedforward.setKv(kV);
      elevatorFeedforward.setKa(kA);
      elevatorFeedforward.setKg(kG);
      m_sparkBaseConfig.closedLoop.feedForward.kG(kG);
    });
    m_sparkBaseConfig.closedLoop.feedForward.kS(kS).kV(kV).kA(kA);
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setStatorCurrentLimit(Current currentLimit)
  {
    m_config.withStatorCurrentLimit(currentLimit);
    m_sparkBaseConfig.smartCurrentLimit((int) currentLimit.in(Amps));
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Deprecated
  /// Unsupported.
  public void setSupplyCurrentLimit(Current currentLimit)
  {
  }

  @Override
  public void setClosedLoopRampRate(Time rampRate)
  {
    m_config.withClosedLoopRampRate(rampRate);
    m_sparkBaseConfig.closedLoopRampRate(rampRate.in(Seconds));
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setOpenLoopRampRate(Time rampRate)
  {
    m_config.withOpenLoopRampRate(rampRate);
    m_sparkBaseConfig.openLoopRampRate(rampRate.in(Seconds));
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setMeasurementUpperLimit(Distance upperLimit)
  {
    if (m_config.getMechanismCircumference().isPresent() && m_config.getMechanismLowerLimit().isPresent())
    {
      m_config.withSoftLimit(m_config.convertFromMechanism(m_config.getMechanismLowerLimit().get()), upperLimit);
      m_sparkBaseConfig.softLimit.forwardSoftLimit(m_config.convertToMechanism(upperLimit).in(Rotations));
      m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  @Override
  public void setMeasurementLowerLimit(Distance lowerLimit)
  {
    if (m_config.getMechanismCircumference().isPresent() && m_config.getMechanismUpperLimit().isPresent())
    {
      m_config.withSoftLimit(lowerLimit, m_config.convertFromMechanism(m_config.getMechanismUpperLimit().get()));
      m_sparkBaseConfig.softLimit.reverseSoftLimit(m_config.convertToMechanism(lowerLimit).in(Rotations));
      m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  @Override
  public void setMechanismUpperLimit(Angle upperLimit)
  {
    m_config.getMechanismLowerLimit().ifPresent(lowerLimit -> {
      m_config.withSoftLimit(lowerLimit, upperLimit);
    });
    m_sparkBaseConfig.softLimit.forwardSoftLimit(upperLimit.in(Rotations));
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setMechanismLowerLimit(Angle lowerLimit)
  {
    m_config.getMechanismUpperLimit().ifPresent(upperLimit -> {
      m_config.withSoftLimit(lowerLimit, upperLimit);
    });
    m_sparkBaseConfig.softLimit.reverseSoftLimit(lowerLimit.in(Rotations));
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public Temperature getTemperature()
  {
    return Celsius.of(m_spark.getMotorTemperature());
  }

  @Override
  public SmartMotorControllerConfig getConfig()
  {
    return m_config;
  }

  @Override
  public Object getMotorController()
  {
    return m_spark;
  }

  @Override
  public Object getMotorControllerConfig()
  {
    return m_sparkBaseConfig;
  }

  @Override
  public Pair<Optional<List<BooleanTelemetryField>>, Optional<List<DoubleTelemetryField>>> getUnsupportedTelemetryFields()
  {
    return Pair.of(Optional.empty(),
                   Optional.of(List.of(DoubleTelemetryField.SupplyCurrent,
                                       DoubleTelemetryField.SupplyCurrentLimit)));
  }

  @Override
  protected Config getSysIdConfig(Voltage maxVoltage, Velocity<VoltageUnit> stepVoltage, Time testDuration)
  {
    StatusLogger.start();
    return new Config(stepVoltage, maxVoltage, testDuration);
  }
}
