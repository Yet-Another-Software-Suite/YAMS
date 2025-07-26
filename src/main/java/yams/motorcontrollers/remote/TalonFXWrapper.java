package yams.motorcontrollers.remote;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.Optional;
import yams.exceptions.SmartMotorControllerConfigurationException;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;

public class TalonFXWrapper extends SmartMotorController
{

  /**
   * {@link TalonFX} motor controller
   */
  private final TalonFX                   m_talonfx;
  /**
   * {@link DCMotor} controlled by {@link TalonFX}
   */
  private final DCMotor                   m_dcmotor;
  /**
   * Configurator
   */
  private final TalonFXConfigurator       m_configurator;
  /**
   * Velocity control request
   */
  private final VelocityVoltage           m_velocityReq     = new VelocityVoltage(0).withSlot(0);
  /**
   * Position with trapazoidal profiling request.
   */
  private final MotionMagicVoltage        m_trapPositionReq = new MotionMagicVoltage(0).withSlot(0);
  /**
   * Position with exponential profiling request.
   */
  private final MotionMagicExpoVoltage    m_expoPositionReq = new MotionMagicExpoVoltage(0).withSlot(0);
  /**
   * Configuration of the motor
   */
  private final TalonFXConfiguration      m_talonConfig;
  /**
   * Mechanism position in rotations.
   */
  private final StatusSignal<Angle>       m_mechanismPosition;
  /**
   * Mechanism velocity in rotations per second.
   */
  private final StatusSignal<AngularVelocity> m_mechanismVelocity;
  /**
   * Supply current of the motor controller.
   */
  private final StatusSignal<Current>     m_supplyCurrent;
  /**
   * Stator current of the motor controller.
   */
  private final StatusSignal<Current>     m_statorCurrent;
  /**
   * DutyCycle of the motor controller.
   */
  private final StatusSignal<Double>      m_dutyCycle;
  /**
   * The motor voltage.
   */
  private final StatusSignal<Voltage>     m_outputVoltage;
  /**
   * Rotor position.
   */
  private final StatusSignal<Angle>       m_rotorPosition;
  /**
   * Rotor velocity.
   */
  private final StatusSignal<AngularVelocity> m_rotorVelocity;
  /**
   * Temperature status
   */
  private final StatusSignal<Temperature> m_deviceTemperature;
  /**
   * {@link CANcoder} to use as external feedback sensor.
   */
  private final Optional<CANcoder>        m_cancoder        = Optional.empty();
  /**
   * {@link CANdi} to use as external feedback sensor.
   */
  private final Optional<CANdi>           m_candi           = Optional.empty();
  /**
   * {@link DCMotorSim} for the {@link TalonFX}.
   */
  private       Optional<DCMotorSim>      m_dcmotorSim      = Optional.empty();

  /**
   * Create the {@link TalonFX} wrapper
   *
   * @param controller  {@link TalonFX}
   * @param motor       {@link DCMotor}
   * @param smartConfig {@link SmartMotorControllerConfig}
   */
  public TalonFXWrapper(TalonFX controller, DCMotor motor, SmartMotorControllerConfig smartConfig)
  {
    this.m_talonfx = controller;
    this.m_dcmotor = motor;
    this.config = smartConfig;
    m_configurator = m_talonfx.getConfigurator();
    m_talonConfig = new TalonFXConfiguration();
    m_configurator.refresh(m_talonConfig);
    m_mechanismPosition = m_talonfx.getPosition();
    m_mechanismVelocity = m_talonfx.getVelocity();
    m_dutyCycle = m_talonfx.getDutyCycle();
    m_statorCurrent = m_talonfx.getStatorCurrent();
    m_supplyCurrent = m_talonfx.getSupplyCurrent();
    m_outputVoltage = m_talonfx.getMotorVoltage();
    m_rotorPosition = m_talonfx.getRotorPosition();
    m_rotorVelocity = m_talonfx.getRotorVelocity();
    m_deviceTemperature = m_talonfx.getDeviceTemp();

    setupSimulation();
    applyConfig(smartConfig);
    checkConfigSafety();
  }

  @Override
  public void setupSimulation()
  {
    if (RobotBase.isSimulation())
    {
      m_dcmotorSim = Optional.of(new DCMotorSim(LinearSystemId.createDCMotorSystem(m_dcmotor,
                                                                                   0.001,
                                                                                   config.getGearing()
                                                                                         .getRotorToMechanismRatio()),
                                                m_dcmotor));
    }
  }

  @Override
  public void seedRelativeEncoder()
  {

  }

  @Override
  public void synchronizeRelativeEncoder()
  {
    if (config.getFeedbackSynchronizationThreshold().isPresent())
    {
      // if (sparkAbsoluteEncoder.isPresent())
      // {
      // if
      // (!Rotations.of(sparkRelativeEncoder.getPosition()).isNear(Rotations.of(sparkAbsoluteEncoder.get()
      // .getPosition()),
      // config.getFeedbackSynchronizationThreshold()
      // .get()))
      // {
      // seedRelativeEncoder();
      // }
      // }
    }
  }

  @Override
  public void simIterate(AngularVelocity mechanismVelocity)
  {
    if (RobotBase.isSimulation() && m_dcmotorSim.isPresent())
    {
      var talonFXSim = m_talonfx.getSimState();

      // set the supply voltage of the TalonFX
      talonFXSim.setSupplyVoltage(RoboRioSim.getVInVoltage());

      // get the motor voltage of the TalonFX
      var motorVoltage = talonFXSim.getMotorVoltageMeasure();

      // use the motor voltage to calculate new position and velocity
      // using WPILib's DCMotorSim class for physics simulation
      // m_dcmotorSim.get().setInputVoltage(motorVoltage.in(Volts));
      m_dcmotorSim.ifPresent(sim -> {
        sim.setAngularVelocity(mechanismVelocity.in(RadiansPerSecond));
        sim.update(config.getClosedLoopControlPeriod().in(Seconds));
      });

      // apply the new rotor position and velocity to the TalonFX;
      // note that this is rotor position/velocity (before gear ratio), but
      // DCMotorSim returns mechanism position/velocity (after gear ratio)
      talonFXSim.setRawRotorPosition(m_dcmotorSim.get().getAngularPosition()
                                                 .times(config.getGearing().getMechanismToRotorRatio()));
      talonFXSim.setRotorVelocity(m_dcmotorSim.get().getAngularVelocity()
                                              .times(config.getGearing().getMechanismToRotorRatio()));

      if (m_cancoder.isPresent())
      {
        var cancoderSim = m_cancoder.get().getSimState();
        cancoderSim.setSupplyVoltage(RoboRioSim.getVInVoltage());
        cancoderSim.setVelocity(m_dcmotorSim.get().getAngularVelocity()
                                            .times(config.getExternalEncoderGearing().getMechanismToRotorRatio()));
        cancoderSim.setRawPosition(m_dcmotorSim.get().getAngularPosition()
                                               .times(config.getExternalEncoderGearing().getMechanismToRotorRatio()));
        cancoderSim.setMagnetHealth(MagnetHealthValue.Magnet_Green);
      }
      if (m_candi.isPresent())
      {
        var candiSim = m_candi.get().getSimState();
        candiSim.setSupplyVoltage(RoboRioSim.getVInVoltage());
        if (useCANdiPWM1())
        {
          candiSim.setPwm1Connected(true);
          candiSim.setPwm1Position(m_dcmotorSim.get().getAngularPosition()
                                               .times(config.getExternalEncoderGearing().getMechanismToRotorRatio()));
          candiSim.setPwm1Velocity(m_dcmotorSim.get().getAngularVelocity()
                                               .times(config.getExternalEncoderGearing().getMechanismToRotorRatio()));
        } else if (useCANdiPWM2())
        {
          candiSim.setPwm2Connected(true);
          candiSim.setPwm2Position(m_dcmotorSim.get().getAngularPosition()
                                               .times(config.getExternalEncoderGearing().getMechanismToRotorRatio()));
          candiSim.setPwm2Velocity(m_dcmotorSim.get().getAngularVelocity()
                                               .times(config.getExternalEncoderGearing().getMechanismToRotorRatio()));
        }
      }
    }
  }

  /**
   * Check if {@link CANdi} PWM1 is used as the
   * {@link com.ctre.phoenix6.configs.ExternalFeedbackConfigs#ExternalFeedbackSensorSource} in
   * {@link TalonFXConfiguration#Feedback}.
   *
   * @return True if CANdi PWM1 is used and configured.
   */
  public boolean useCANdiPWM1()
  {
    m_configurator.refresh(m_talonConfig.Feedback);
    boolean configured = (m_talonConfig.Feedback.FeedbackSensorSource == FeedbackSensorSourceValue.SyncCANdiPWM1 ||
                          m_talonConfig.Feedback.FeedbackSensorSource == FeedbackSensorSourceValue.RemoteCANdiPWM1);
    if (configured && m_candi.isEmpty())
    {
      throw new IllegalArgumentException(
          "[ERROR] CANdi PWM1 has been configured but is not present in SmartMotorControllerConfig!");
    }
    return configured;
  }

  /**
   * Check if {@link CANdi} PWM1 is used as the
   * {@link com.ctre.phoenix6.configs.ExternalFeedbackConfigs#ExternalFeedbackSensorSource} in
   * {@link TalonFXConfiguration#Feedback}.
   *
   * @return True if CANdi is used.
   */
  public boolean useCANdiPWM2()
  {
    m_configurator.refresh(m_talonConfig.Feedback);
    boolean configured = (m_talonConfig.Feedback.FeedbackSensorSource == FeedbackSensorSourceValue.SyncCANdiPWM2 ||
                          m_talonConfig.Feedback.FeedbackSensorSource == FeedbackSensorSourceValue.RemoteCANdiPWM2);
    if (configured && m_candi.isEmpty())
    {
      throw new IllegalArgumentException(
          "[ERROR] CANdi PWM2 has been configured but is not present in SmartMotorControllerConfig!");
    }
    return configured;
  }

  @Override
  public void setEncoderVelocity(AngularVelocity velocity)
  {
    m_dcmotorSim.ifPresent(sim -> sim.setAngularVelocity(velocity.in(RadiansPerSecond)));
    // Cannot set velocity of CANdi or CANCoder.
  }

  @Override
  public void setEncoderVelocity(LinearVelocity velocity)
  {
    setEncoderVelocity(config.convertToMechanism(velocity));
  }

  @Override
  public void setEncoderPosition(Angle angle)
  {
    m_talonfx.setPosition(angle);
    m_dcmotorSim.ifPresent(dcMotorSim -> dcMotorSim.setAngle(angle.in(Radians)));

    // Might want to set absolute encoder position in the future
    /*
     * if (m_candi.isPresent())
     * {
     * CANdiConfigurator configurator = m_candi.get().getConfigurator();
     * CANdiConfiguration cfg = new CANdiConfiguration();
     * configurator.refresh(cfg);
     *
     * if (useCANdiPWM1())
     * {
     * Angle newOffset = m_candi.get().getPWM1Position().getValue()
     * .plus(Rotations.of(cfg.PWM1.AbsoluteSensorOffset))
     * .minus(angle);
     * cfg.PWM1.withAbsoluteSensorOffset(newOffset);
     * }
     * if (useCANdiPWM2())
     * {
     * Angle newOffset = m_candi.get().getPWM2Position().getValue()
     * .plus(Rotations.of(cfg.PWM2.AbsoluteSensorOffset))
     * .minus(angle);
     * cfg.PWM2.withAbsoluteSensorOffset(newOffset);
     * }
     * configurator.apply(cfg);
     * }
     * if (m_cancoder.isPresent())
     * {
     * var configurator = m_cancoder.get().getConfigurator();
     * var cfg = new CANcoderConfiguration();
     * configurator.refresh(cfg);
     * Angle newOffset = m_cancoder.get().getPosition().getValue()
     * .plus(Rotations.of(cfg.MagnetSensor.MagnetOffset))
     * .minus(angle);
     * cfg.MagnetSensor.withMagnetOffset(newOffset);
     * configurator.apply(cfg);
     * }
     */
  }

  @Override
  public void setEncoderPosition(Distance distance)
  {
    setEncoderPosition(config.convertToMechanism(distance));
  }

  @Override
  public void setPosition(Angle angle)
  {
    setpointPosition = Optional.ofNullable(angle);
    m_talonfx.setControl(m_trapPositionReq.withPosition(angle));
  }

  @Override
  public void setPosition(Distance distance)
  {
    setPosition(config.convertToMechanism(distance));
  }

  @Override
  public void setVelocity(LinearVelocity velocity)
  {
    setVelocity(config.convertToMechanism(velocity));
  }

  @Override
  public void setVelocity(AngularVelocity angle)
  {
    setpointVelocity = Optional.ofNullable(angle);
    if(angle != null)
        m_talonfx.setControl(m_velocityReq.withVelocity(angle));
  }

  @Override
  public double getDutyCycle()
  {
    return m_dutyCycle.refresh().getValue();
  }

  @Override
  public void setDutyCycle(double dutyCycle)
  {
    m_talonfx.set(dutyCycle);
  }

  @Override
  public boolean applyConfig(SmartMotorControllerConfig config)
  {
    m_configurator.refresh(m_talonConfig);
    this.config = config;
    // Closed loop controllers.
    if (config.getClosedLoopController().isPresent())
    {
      ProfiledPIDController controller = config.getClosedLoopController().get();
      if (controller.getPositionTolerance() != new ProfiledPIDController(0,
                                                                         0,
                                                                         0,
                                                                         new Constraints(0, 0)).getPositionTolerance())
      {
        throw new IllegalArgumentException("[ERROR] Cannot set closed-loop controller error tolerance on " +
                                           (config.getTelemetryName().isPresent() ? config.getTelemetryName().get()
                                                                                  : "TalonFX(" +
                                                                                    m_talonfx.getDeviceID() + ")"));
      }
      m_talonConfig.Slot0.kP = controller.getP();
      m_talonConfig.Slot0.kI = controller.getI();
      m_talonConfig.Slot0.kD = controller.getD();
      if (config.getMechanismCircumference().isPresent())
      {
        m_talonConfig.MotionMagic
            .withMotionMagicCruiseVelocity(
                config.convertToMechanism(MetersPerSecond.of(controller.getConstraints().maxVelocity)));
      } else
      {
        m_talonConfig.MotionMagic
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(controller.getConstraints().maxAcceleration));
      }
      if (config.getMechanismCircumference().isPresent())
      {
        m_talonConfig.MotionMagic
            .withMotionMagicAcceleration(
                config.convertToMechanism(MetersPerSecondPerSecond.of(controller.getConstraints().maxAcceleration)));
      } else
      {
        m_talonConfig.MotionMagic
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(controller.getConstraints().maxAcceleration));
      }
    } else if (config.getSimpleClosedLoopController().isPresent())
    {
      PIDController controller = config.getSimpleClosedLoopController().get();
      m_talonConfig.Slot0.kP = controller.getP();
      m_talonConfig.Slot0.kI = controller.getI();
      m_talonConfig.Slot0.kD = controller.getD();
      if (controller.getErrorTolerance() != new PIDController(0, 0, 0).getErrorTolerance())
      {
        throw new IllegalArgumentException("[ERROR] Cannot set closed-loop controller error tolerance on " +
                                           (config.getTelemetryName().isPresent() ? config.getTelemetryName().get()
                                                                                  : "TalonFX(" +
                                                                                    m_talonfx.getDeviceID() + ")"));
      }
    } else
    {
      throw new IllegalArgumentException("[ERROR] No closed loop configuration available!");
    }
    // Feedforwards
    if (config.getArmFeedforward().isPresent() || config.getElevatorFeedforward().isPresent() ||
        config.getSimpleFeedforward().isPresent())
    {
      double kS = 0, kV = 0, kA = 0, kG = 0;
      if (config.getArmFeedforward().isPresent())
      {
        var ff = config.getArmFeedforward().get();
        kS = ff.getKs();
        kV = ff.getKv();
        kA = ff.getKa();
        kG = ff.getKg();
        m_talonConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

      } else if (config.getElevatorFeedforward().isPresent())
      {
        var ff = config.getElevatorFeedforward().get();
        kS = ff.getKs();
        kV = ff.getKv();
        kA = ff.getKa();
        kG = ff.getKg();
        m_talonConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
      } else if (config.getSimpleFeedforward().isPresent())
      {
        var ff = config.getSimpleFeedforward().get();
        kS = ff.getKs();
        kV = ff.getKv();
        kA = ff.getKa();
      }
      m_talonConfig.MotionMagic.MotionMagicExpo_kA = kA;
      m_talonConfig.MotionMagic.MotionMagicExpo_kV = kV;
      m_talonConfig.Slot0.kS = kS;
      m_talonConfig.Slot0.kV = kV;
      m_talonConfig.Slot0.kA = kA;
      m_talonConfig.Slot0.kG = kG;
    }

    // Motor inversion
    m_talonConfig.MotorOutput.Inverted = config.getMotorInverted() ? InvertedValue.Clockwise_Positive
                                                                   : InvertedValue.CounterClockwise_Positive;
    // Idle mode
    if (config.getIdleMode().isPresent())
    {
      m_talonConfig.MotorOutput.NeutralMode = config.getIdleMode().get() == MotorMode.BRAKE ? NeutralModeValue.Brake
                                                                                            : NeutralModeValue.Coast;
    }
    // Maximum and minimum voltage
    if (config.getClosedLoopControllerMaximumVoltage().isPresent())
    {
      m_talonConfig.Voltage.withPeakForwardVoltage(config.getClosedLoopControllerMaximumVoltage().get());
      m_talonConfig.Voltage.withPeakReverseVoltage(config.getClosedLoopControllerMaximumVoltage().get().times(-1));
    }
    // Ramp rates
    m_talonConfig.ClosedLoopRamps.withDutyCycleClosedLoopRampPeriod(config.getClosedLoopRampRate());
    m_talonConfig.OpenLoopRamps.withDutyCycleOpenLoopRampPeriod(config.getOpenLoopRampRate());
    m_talonConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(config.getClosedLoopRampRate());
    m_talonConfig.OpenLoopRamps.withVoltageOpenLoopRampPeriod(config.getOpenLoopRampRate());
    m_talonConfig.ClosedLoopRamps.withTorqueClosedLoopRampPeriod(config.getClosedLoopRampRate());
    m_talonConfig.OpenLoopRamps.withTorqueOpenLoopRampPeriod(config.getOpenLoopRampRate());
    // Current limits
    if (config.getStatorStallCurrentLimit().isPresent())
    {
      m_talonConfig.CurrentLimits.withStatorCurrentLimit(config.getStatorStallCurrentLimit().getAsInt());
    }
    if (config.getSupplyStallCurrentLimit().isPresent())
    {
      m_talonConfig.CurrentLimits.withSupplyCurrentLimit(config.getSupplyStallCurrentLimit().getAsInt());
    }
    // Soft limit
    if (config.getMechanismUpperLimit().isPresent())
    {
      m_talonConfig.SoftwareLimitSwitch.withForwardSoftLimitThreshold(config.getMechanismUpperLimit().get());
    }
    if (config.getMechanismLowerLimit().isPresent())
    {
      m_talonConfig.SoftwareLimitSwitch.withReverseSoftLimitThreshold(config.getMechanismLowerLimit().get());
    }

    // Configure external encoders
    if (config.getExternalEncoder().isPresent())
    {
      // Starting position
      if (config.getStartingPosition().isPresent())
      {
        DriverStation.reportWarning("[WARNING] Starting position is not applied to " +
                                    (config.getTelemetryName().isPresent() ? config.getTelemetryName().get()
                                                                           : ("TalonFX(" + m_talonfx.getDeviceID() +
                                                                              ")"))
                                    +
                                    " because an external encoder is used!", false);
      }
      // Set the gear ratio for external encoders.
      m_talonConfig.Feedback.RotorToSensorRatio = 1.0;
      // config.getExternalEncoderGearing().getMechanismToRotorRatio() *
      // config.getGearing().getMechanismToRotorRatio();
      m_talonConfig.Feedback.SensorToMechanismRatio = config.getExternalEncoderGearing()
                                                            .getMechanismToRotorRatio();
      if (config.getExternalEncoder().get() instanceof CANcoder encoder)
      {
        var configurator = encoder.getConfigurator();
        var cfg = new CANcoderConfiguration();
        configurator.refresh(cfg);
        m_talonConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        cfg.MagnetSensor.withSensorDirection(
            config.getExternalEncoderInverted() ? SensorDirectionValue.Clockwise_Positive
                                                : SensorDirectionValue.CounterClockwise_Positive);

        // Configure feedback source for CANCoder
        if (encoder.getIsProLicensed().getValue())
        {
          m_talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        } else
        {
          m_talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        }
        // Zero offset
        if (config.getZeroOffset().isPresent())
        {
          cfg.MagnetSensor.withMagnetOffset(encoder.getPosition().getValue()
                                                   .plus(Rotations.of(cfg.MagnetSensor.MagnetOffset))
                                                   .minus(config.getZeroOffset().get()));
          m_talonConfig.Feedback.FeedbackRotorOffset = 0;
        }
        // Discontinuity Point
        if (config.getDiscontinuityPoint().isPresent())
        {
          cfg.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(config.getDiscontinuityPoint().get());
        }
        configurator.apply(cfg);
      } else if (config.getExternalEncoder().get() instanceof CANdi encoder)
      {
        var configurator = encoder.getConfigurator();
        var cfg = new CANdiConfiguration();
        configurator.refresh(cfg);
        m_talonConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        // Ensure pro uses best option.
        if (encoder.getIsProLicensed().getValue())
        {
          if (useCANdiPWM2())
          {
            m_talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANdiPWM2;
          }
          if (useCANdiPWM1())
          {
            m_talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANdiPWM1;
          }
        } else
        {
          if (useCANdiPWM2())
          {
            m_talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANdiPWM2;
          }
          if (useCANdiPWM1())
          {
            m_talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANdiPWM1;
          }
        }
        if (useCANdiPWM1())
        {
          cfg.PWM1.SensorDirection = config.getExternalEncoderInverted();

          // Zero offset
          if (config.getZeroOffset().isPresent())
          {
            cfg.PWM1.withAbsoluteSensorOffset(encoder.getPWM1Position().getValue()
                                                     .plus(Rotations.of(cfg.PWM1.AbsoluteSensorOffset))
                                                     .minus(config.getZeroOffset().get()));
            m_talonConfig.Feedback.FeedbackRotorOffset = 0;

          }
          // Discontinuity point
          if (config.getDiscontinuityPoint().isPresent())
          {
            cfg.PWM1.withAbsoluteSensorDiscontinuityPoint(config.getDiscontinuityPoint().get());
          }
        } else if (useCANdiPWM2())
        {
          cfg.PWM2.SensorDirection = config.getExternalEncoderInverted();
          // Zero offset
          if (config.getZeroOffset().isPresent())
          {
            cfg.PWM2.withAbsoluteSensorOffset(encoder.getPWM2Position().getValue()
                                                     .plus(Rotations.of(cfg.PWM2.AbsoluteSensorOffset))
                                                     .minus(config.getZeroOffset().get()));
            m_talonConfig.Feedback.FeedbackRotorOffset = 0;
          }
          // Discontinuity point
          if (config.getDiscontinuityPoint().isPresent())
          {
            cfg.PWM2.withAbsoluteSensorDiscontinuityPoint(config.getDiscontinuityPoint().get());
          }
        }
        configurator.apply(cfg);
      }
    } else
    {
      m_talonConfig.Feedback.RotorToSensorRatio = 1.0;
      m_talonConfig.Feedback.SensorToMechanismRatio = config.getGearing().getMechanismToRotorRatio();

      // Starting position
      if (config.getStartingPosition().isPresent())
      {
        m_configurator.apply(m_talonConfig);
        m_talonfx.setPosition(config.getStartingPosition().get());
      }
      // Discontinuity point
      if (config.getDiscontinuityPoint().isPresent())
      {
        if (!config.getDiscontinuityPoint().get().equals(Rotations.of(0.5)))
        {
          throw new SmartMotorControllerConfigurationException("Invalid wrapping given to TalonFX",
                                                               "Cannot configure TalonFX!",
                                                               "withContinuousWrapping(Rotations.of(-0.5),Rotations.of(0.5))");
        }
        m_talonConfig.ClosedLoopGeneral.ContinuousWrap = true;
      }
    }
    // Zero offset.
    if (config.getZeroOffset().isPresent())
    {
      m_talonConfig.Feedback.withFeedbackRotorOffset(config.getZeroOffset().get());
    }

    // Invert the encoder.
    if (config.getEncoderInverted())
    {
      throw new SmartMotorControllerConfigurationException("Integrated encoder phase cannot be set",
                                                           "Cannot configure TalonFX!",
                                                           "withEncoderInverted(false)");
    }

    // Control loop frequency.
    m_velocityReq.withUpdateFreqHz(config.getClosedLoopControlPeriod().in(Second));
    m_trapPositionReq.withUpdateFreqHz(config.getClosedLoopControlPeriod().in(Second));
    m_expoPositionReq.withUpdateFreqHz(config.getClosedLoopControlPeriod().in(Second));

    // Configure follower motors
    if (config.getFollowers().isPresent())
    {
      for (Pair<Object, Boolean> follower : config.getFollowers().get())
      {
        if (follower.getFirst() instanceof TalonFXS)
        {
          ((TalonFXS) follower.getFirst()).setControl(new Follower(m_talonfx.getDeviceID(),
                                                                   follower.getSecond()).withUpdateFreqHz(
              config.getClosedLoopControlPeriod()
                    .in(Second)));

        } else if (follower.getFirst() instanceof TalonFX)
        {
          ((TalonFX) follower.getFirst()).setControl(new Follower(m_talonfx.getDeviceID(),
                                                                  follower.getSecond()).withUpdateFreqHz(
              config.getClosedLoopControlPeriod()
                    .in(Second)));
        } else
        {
          throw new IllegalArgumentException(
              "[ERROR] Unknown follower type: " + follower.getFirst().getClass().getSimpleName());
        }
      }
      config.clearFollowers();
    }

    // Unsupported options.
    if (config.getTemperatureCutoff().isPresent())
    {
      throw new IllegalArgumentException("[ERROR] TemperatureCutoff is not supported");
    }
    if (config.getFeedbackSynchronizationThreshold().isPresent())
    {
      throw new IllegalArgumentException("[ERROR] FeedbackSynchronizationThreshold is not supported");
    }
    if (config.getVoltageCompensation().isPresent())
    {
      throw new IllegalArgumentException("[ERROR] VoltageCompensation is not supported");
    }

    return m_configurator.apply(m_talonConfig).isOK();
  }

  @Override
  public Optional<Current> getSupplyCurrent()
  {
    return Optional.of(m_supplyCurrent.refresh().getValue());
  }

  @Override
  public Current getStatorCurrent()
  {
    return m_statorCurrent.refresh().getValue();
  }

  @Override
  public Voltage getVoltage()
  {
    return m_outputVoltage.refresh().getValue();
  }

  @Override
  public void setVoltage(Voltage voltage)
  {
    m_talonfx.setVoltage(voltage.in(Volts));
  }

  @Override
  public DCMotor getDCMotor()
  {
    return m_dcmotor;
  }

  @Override
  public LinearVelocity getMeasurementVelocity()
  {
    return config.convertFromMechanism(getMechanismVelocity());
  }

  @Override
  public Distance getMeasurementPosition()
  {
    return config.convertFromMechanism(getMechanismPosition());
  }

  @Override
  public AngularVelocity getMechanismVelocity()
  {
    if (m_cancoder.isPresent())
    {
      return m_cancoder.get().getVelocity().getValue();
    }
    if (m_candi.isPresent())
    {
      if (useCANdiPWM1())
      {
        return m_candi.get().getPWM1Velocity().getValue();
      }
      if (useCANdiPWM2())
      {
        return m_candi.get().getPWM2Velocity().getValue();
      }
    }
    return m_mechanismVelocity.refresh().getValue();
  }

  @Override
  public Angle getMechanismPosition()
  {
    if (m_cancoder.isPresent())
    {
      return m_cancoder.get().getPosition().getValue();
    }
    if (m_candi.isPresent())
    {
      if (useCANdiPWM1())
      {
        return m_candi.get().getPWM1Position().getValue();
      }
      if (useCANdiPWM2())
      {
        return m_candi.get().getPWM2Position().getValue();
      }
    }
    return m_mechanismPosition.refresh().getValue();
  }

  @Override
  public AngularVelocity getRotorVelocity()
  {
    return m_rotorVelocity.refresh().getValue();
  }

  @Override
  public Angle getRotorPosition()
  {
    return m_rotorPosition.refresh().getValue();
  }

  @Override
  public Temperature getTemperature()
  {
    return m_deviceTemperature.refresh().getValue();
  }

  @Override
  public SmartMotorControllerConfig getConfig()
  {
    return config;
  }

  @Override
  public Object getMotorController()
  {
    return m_talonfx;
  }

  @Override
  public Object getMotorControllerConfig()
  {
    return m_talonConfig;
  }
}
