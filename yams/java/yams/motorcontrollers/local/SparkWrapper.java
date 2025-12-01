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
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
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
    sparkSim.ifPresent(sim -> sim.setPosition(angle.in(Rotations)));
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

    config.getClosedLoopTolerance().ifPresent(tolerance -> {
      if (config.getMechanismCircumference().isPresent())
      {
        m_pidController.ifPresent(pidController -> pidController.setTolerance(config.convertFromMechanism(tolerance)
                                                                                    .in(Meters)));
        m_simplePidController.ifPresent(pidController -> pidController.setTolerance(config.convertFromMechanism(
            tolerance).in(Meters)));
        m_expoPidController.ifPresent(pidController -> pidController.setTolerance(config.convertFromMechanism(tolerance)
                                                                                        .in(Meters)));
      } else
      {
        m_pidController.ifPresent(pidController -> pidController.setTolerance(tolerance.in(Rotations)));
        m_simplePidController.ifPresent(pidController -> pidController.setTolerance(tolerance.in(Rotations)));
        m_expoPidController.ifPresent(pidController -> pidController.setTolerance(tolerance.in(Rotations)));
      }
    });

    iterateClosedLoopController();

    // Handle closed loop controller thread
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
    // Calculate Spark conversion factors
    double positionConversionFactor = config.getGearing().getRotorToMechanismRatio();
    double velocityConversionFactor = config.getGearing().getRotorToMechanismRatio() / 60.0;

    // Set base config options
    m_sparkBaseConfig.openLoopRampRate(config.getOpenLoopRampRate().in(Seconds));
    m_sparkBaseConfig.closedLoopRampRate(config.getClosedLoopRampRate().in(Seconds));
    m_sparkBaseConfig.inverted(config.getMotorInverted());
    m_sparkBaseConfig.encoder.positionConversionFactor(positionConversionFactor);
    m_sparkBaseConfig.encoder.velocityConversionFactor(velocityConversionFactor);
    if (config.getEncoderInverted())
    {
      throw new IllegalArgumentException("[ERROR] Spark relative encoder cannot be inverted!");
    }
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
    // Setup external encoder.
    boolean useExternalEncoder = config.getUseExternalFeedback();
    if (config.getExternalEncoder().isPresent() && useExternalEncoder)
    {
      Object externalEncoder = config.getExternalEncoder().get();
      if (externalEncoder instanceof SparkAbsoluteEncoder)
      {
        double absoluteEncoderConversionFactor = config.getExternalEncoderGearing().getRotorToMechanismRatio();
        m_sparkAbsoluteEncoder = Optional.of((SparkAbsoluteEncoder) externalEncoder);
        m_sparkBaseConfig.absoluteEncoder.positionConversionFactor(absoluteEncoderConversionFactor);
        m_sparkBaseConfig.absoluteEncoder.velocityConversionFactor(absoluteEncoderConversionFactor / 60);
        m_sparkBaseConfig.absoluteEncoder.inverted(config.getExternalEncoderInverted());

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

//        seedRelativeEncoder();
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

    if (config.getMaxDiscontinuityPoint().isPresent() &&
        !(m_pidController.isPresent() || m_expoPidController.isPresent() || m_simplePidController.isPresent()))
    {
      throw new IllegalArgumentException(
          "[ERROR] Discontinuity point is not supported on Sparks, or we have not implemented this!");
    } else if (config.getMaxDiscontinuityPoint().isPresent() && config.getMinDiscontinuityPoint().isPresent())
    {
      var max = config.getMaxDiscontinuityPoint().get().in(Rotations);
      var min = config.getMinDiscontinuityPoint().get().in(Rotations);

      m_pidController.ifPresent(pidController -> {pidController.enableContinuousInput(min, max);});
      m_expoPidController.ifPresent(pidController -> {pidController.enableContinuousInput(min, max);});
      m_simplePidController.ifPresent(pidController -> {pidController.enableContinuousInput(min, max);});
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

  }

  @Override
  public void setFeedback(double kP, double kI, double kD)
  {
    setKp(kP);
    setKi(kI);
    setKd(kD);
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
  }

  @Override
  public void setFeedforward(double kS, double kV, double kA, double kG)
  {
    setKs(kS);
    setKv(kV);
    setKa(kA);
    setKg(kG);
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
    }
  }

  @Override
  public void setMeasurementLowerLimit(Distance lowerLimit)
  {
    if (m_config.getMechanismCircumference().isPresent() && m_config.getMechanismUpperLimit().isPresent())
    {
      m_config.withSoftLimit(lowerLimit, m_config.convertFromMechanism(m_config.getMechanismUpperLimit().get()));
    }
  }

  @Override
  public void setMechanismUpperLimit(Angle upperLimit)
  {
    m_config.getMechanismLowerLimit().ifPresent(lowerLimit -> {
      m_config.withSoftLimit(lowerLimit, upperLimit);
    });
  }

  @Override
  public void setMechanismLowerLimit(Angle lowerLimit)
  {
    m_config.getMechanismUpperLimit().ifPresent(upperLimit -> {
      m_config.withSoftLimit(lowerLimit, upperLimit);
    });
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
}
