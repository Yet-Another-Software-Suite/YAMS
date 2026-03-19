package yams.motorcontrollers.remote;

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

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.AngularAccelerationUnit;
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
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Supplier;
import yams.exceptions.SmartMotorControllerConfigurationException;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.simulation.DCMotorSimSupplier;
import yams.telemetry.SmartMotorControllerTelemetry.BooleanTelemetryField;
import yams.telemetry.SmartMotorControllerTelemetry.DoubleTelemetryField;

/**
 * TalonFX wrapper for a CTRE TalonFX motor controller.
 */
public class TalonFXWrapper extends SmartMotorController
{

  /**
   * {@link TalonFX} motor controller
   */
  private final TalonFX                       m_talonfx;
  /**
   * {@link DCMotor} controlled by {@link TalonFX}
   */
  private final DCMotor                       m_dcmotor;
  /**
   * Configurator
   */
  private final TalonFXConfigurator           m_configurator;
  /**
   * Control request slot.
   */
  private final int                           m_controlReqSlot    = 0;
  /**
   * Velocity control request
   */
  private final VelocityVoltage               m_simpleVelocityReq = new VelocityVoltage(0)
      .withSlot(m_controlReqSlot)
      .withEnableFOC(false);
  /**
   * Position control request.
   */
  private final PositionVoltage               m_simplePositionReq = new PositionVoltage(0)
      .withSlot(m_controlReqSlot)
      .withEnableFOC(false);
  /**
   * Position with trapezoidal profiling request.
   */
  private final MotionMagicVoltage            m_trapPositionReq   = new MotionMagicVoltage(0)
      .withSlot(m_controlReqSlot)
      .withEnableFOC(false);
  /**
   * Velocity with trapezoidal profiling request.
   */
  private final MotionMagicVelocityVoltage    m_trapVelocityReq   = new MotionMagicVelocityVoltage(0)
      .withSlot(m_controlReqSlot)
      .withEnableFOC(false);
  /**
   * Position with exponential profiling request.
   */
  private final MotionMagicExpoVoltage        m_expoPositionReq   = new MotionMagicExpoVoltage(0)
      .withSlot(m_controlReqSlot)
      .withEnableFOC(false);
  /**
   * Position control request to use.
   */
  private       ControlRequest                m_positionReq       = m_simplePositionReq;
  /**
   * Velocity control request to use.
   */
  private       ControlRequest                m_velocityReq       = m_simpleVelocityReq;
  /**
   * Configuration of the motor
   */
  private final TalonFXConfiguration          m_talonConfig;
  /**
   * Mechanism position in rotations.
   */
  private final StatusSignal<Angle>           m_mechanismPosition;
  /**
   * Mechanism velocity in rotations per second.
   */
  private final StatusSignal<AngularVelocity> m_mechanismVelocity;
  /**
   * Supply current of the motor controller.
   */
  private final StatusSignal<Current>         m_supplyCurrent;
  /**
   * Stator current of the motor controller.
   */
  private final StatusSignal<Current>         m_statorCurrent;
  /**
   * DutyCycle of the motor controller.
   */
  private final StatusSignal<Double>          m_dutyCycle;
  /**
   * The motor voltage.
   */
  private final StatusSignal<Voltage>         m_outputVoltage;
  /**
   * Rotor position.
   */
  private final StatusSignal<Angle>           m_rotorPosition;
  /**
   * Rotor velocity.
   */
  private final StatusSignal<AngularVelocity> m_rotorVelocity;
  /**
   * Temperature status
   */
  private final StatusSignal<Temperature>     m_deviceTemperature;
  /**
   * {@link CANcoder} to use as external feedback sensor.
   */
  private       Optional<CANcoder>            m_cancoder          = Optional.empty();
  /**
   * {@link CANdi} to use as external feedback sensor.
   */
  private       Optional<CANdi>               m_candi             = Optional.empty();
  /**
   * Exponential profiled velocity control request enabled.
   */
  private       boolean                       expEnabled          = false;
  /**
   * {@link DCMotorSim} for the {@link TalonFX}.
   */
  private       Optional<DCMotorSim>          m_dcmotorSim        = Optional.empty();

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
    this.m_config = smartConfig;
    m_configurator = m_talonfx.getConfigurator();
    if (smartConfig.getVendorConfig().isPresent())
    {
      var genCfg = smartConfig.getVendorConfig().get();
      if (genCfg instanceof TalonFXConfiguration)
      {
        m_talonConfig = (TalonFXConfiguration) genCfg;
      } else
      {
        throw new SmartMotorControllerConfigurationException(
            "TalonFXConfiguration is the only acceptable vendor config type for TalonFXWrapper",
            "Vendor config is unable to be applied",
            ".withVendorConfig(new TalonFXConfiguration())");
      }
    } else
    {
      m_talonConfig = new TalonFXConfiguration();
      if (!m_config.getResetPreviousConfig())
      {m_configurator.refresh(m_talonConfig);}
    }
    m_mechanismPosition = m_talonfx.getPosition();
    m_mechanismVelocity = m_talonfx.getVelocity();
    m_dutyCycle = m_talonfx.getDutyCycle();
    m_statorCurrent = m_talonfx.getStatorCurrent();
    m_supplyCurrent = m_talonfx.getSupplyCurrent();
    m_outputVoltage = m_talonfx.getMotorVoltage();
    m_rotorPosition = m_talonfx.getRotorPosition();
    m_rotorVelocity = m_talonfx.getRotorVelocity();
    m_deviceTemperature = m_talonfx.getDeviceTemp();
    m_closedLoopControllerThread = null;

    for (int i = 0; i < 6; i++)
    {
      if (isMotor(motor, DCMotor.getFalcon500Foc(i)) ||
          isMotor(motor, DCMotor.getKrakenX44Foc(i)) ||
          isMotor(motor, DCMotor.getKrakenX60Foc(i)))
      {enableFOC();}
    }
    setupSimulation();
    applyConfig(smartConfig);
    checkConfigSafety();
  }

  /**
   * Enable FOC control, ignored if the device isn't PRO licensed.
   *
   * @return {@link TalonFXWrapper} for ease of use.
   */
  public TalonFXWrapper enableFOC()
  {
    m_simpleVelocityReq.withEnableFOC(true);
    m_simplePositionReq.withEnableFOC(true);
    m_trapPositionReq.withEnableFOC(true);
    m_trapVelocityReq.withEnableFOC(true);
    m_expoPositionReq.withEnableFOC(true);
    return this;
  }

  /**
   * Disable FOC control, ignored if the device isn't PRO licensed.
   *
   * @return {@link TalonFXWrapper} for ease of use.
   */
  public TalonFXWrapper disableFOC()
  {
    m_simpleVelocityReq.withEnableFOC(false);
    m_simplePositionReq.withEnableFOC(false);
    m_trapPositionReq.withEnableFOC(false);
    m_trapVelocityReq.withEnableFOC(false);
    m_expoPositionReq.withEnableFOC(false);
    return this;
  }

  @Override
  public void setupSimulation()
  {
    if (RobotBase.isSimulation())
    {
      var setupRan = m_dcmotorSim.isPresent();
      if (!setupRan)
      {
        m_dcmotorSim = Optional.of(new DCMotorSim(LinearSystemId.createDCMotorSystem(m_dcmotor,
                                                                                     m_config.getMOI(),
                                                                                     m_config.getGearing()
                                                                                             .getMechanismToRotorRatio()),
                                                  m_dcmotor));
        setSimSupplier(new DCMotorSimSupplier(m_dcmotorSim.get(), this));
      }
      m_config.getStartingPosition().ifPresent(mechPos -> {
        m_simSupplier.get().setMechanismPosition(mechPos);
      });
    }
  }

  @Override
  public void seedRelativeEncoder()
  {

  }

  @Override
  @Deprecated
  public void synchronizeRelativeEncoder()
  {
    // Unused
  }

  @Override
  public void simIterate()
  {
    if (RobotBase.isSimulation() && m_simSupplier.isPresent())
    {
      var talonFXSim = m_talonfx.getSimState();

      // set the supply voltage of the TalonFX
      talonFXSim.setSupplyVoltage(m_simSupplier.get().getMechanismSupplyVoltage());

      // get the motor voltage of the TalonFX
      var motorVoltage = talonFXSim.getMotorVoltageMeasure();

      m_simSupplier.ifPresent(simSupplier -> {
        simSupplier.setMechanismStatorVoltage(motorVoltage); // dcmotorSim.setInputVoltage(motorVoltage)
        simSupplier.updateSimState(); // dcmotorSim.update(0.020)
        simSupplier.starveUpdateSim(); // clear update once atomic
      });

      // apply the new rotor position and velocity to the TalonFX;
      // note that this is rotor position/velocity (before gear ratio), but
      // DCMotorSim returns mechanism position/velocity (after gear ratio)
      talonFXSim.setRawRotorPosition(m_simSupplier.get().getRotorPosition());
      talonFXSim.setRotorVelocity(m_simSupplier.get().getRotorVelocity());

      if (m_cancoder.isPresent())
      {
        var cancoderSim = m_cancoder.get().getSimState();
        cancoderSim.setSupplyVoltage(m_simSupplier.get().getMechanismSupplyVoltage());
        cancoderSim.setVelocity(m_simSupplier.get().getMechanismVelocity()
                                             .times(m_config.getExternalEncoderGearing().orElse(MechanismGearing.kOne)
                                                            .getMechanismToRotorRatio()));
        cancoderSim.setRawPosition(m_simSupplier.get().getMechanismPosition()
                                                .times(m_config.getExternalEncoderGearing()
                                                               .orElse(MechanismGearing.kOne)
                                                               .getMechanismToRotorRatio()));
        cancoderSim.setMagnetHealth(MagnetHealthValue.Magnet_Green);
      }
      if (m_candi.isPresent())
      {
        var candiSim = m_candi.get().getSimState();
        candiSim.setSupplyVoltage(RoboRioSim.getVInVoltage());
        if (useCANdiPWM1())
        {
          candiSim.setPwm1Connected(true);
          candiSim.setPwm1Position(m_simSupplier.get().getMechanismPosition()
                                                .times(m_config.getExternalEncoderGearing()
                                                               .orElse(MechanismGearing.kOne)
                                                               .getMechanismToRotorRatio()));
          candiSim.setPwm1Velocity(m_simSupplier.get().getMechanismVelocity()
                                                .times(m_config.getExternalEncoderGearing()
                                                               .orElse(MechanismGearing.kOne)
                                                               .getMechanismToRotorRatio()));
        } else if (useCANdiPWM2())
        {
          candiSim.setPwm2Connected(true);
          candiSim.setPwm2Position(m_simSupplier.get().getMechanismPosition()
                                                .times(m_config.getExternalEncoderGearing()
                                                               .orElse(MechanismGearing.kOne)
                                                               .getMechanismToRotorRatio()));
          candiSim.setPwm2Velocity(m_simSupplier.get().getMechanismVelocity()
                                                .times(m_config.getExternalEncoderGearing()
                                                               .orElse(MechanismGearing.kOne)
                                                               .getMechanismToRotorRatio()));
        }
      }
      // TODO: Uncomment after the 2026 season
//      m_looseFollowers.ifPresent(smcs -> {for(var f : smcs){f.simIterate();}});
    }
  }

  @Override
  public void setIdleMode(MotorMode mode)
  {
    m_talonConfig.MotorOutput.withNeutralMode(
        mode == MotorMode.BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    forceConfigApply();
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
  @Deprecated
  public void setEncoderVelocity(AngularVelocity velocity)
  {
    //m_simSupplier.ifPresent(mSim -> mSim.setMechanismVelocity(velocity));
//    m_dcmotorSim.ifPresent(sim -> sim.setAngularVelocity(velocity.in(RadiansPerSecond)));
    // Cannot set velocity of CANdi or CANCoder.
  }

  @Override
  public void setEncoderVelocity(LinearVelocity velocity)
  {
    setEncoderVelocity(m_config.convertToMechanism(velocity));
  }

  @Override
  public void setEncoderPosition(Angle angle)
  {
    m_talonfx.setPosition(angle);
    m_cancoder.ifPresent(caNcoder -> caNcoder.setPosition(angle.in(Rotations)));
    m_simSupplier.ifPresent(mSim -> {
      m_talonfx.getSimState().setRawRotorPosition(angle.times(m_config.getGearing().getMechanismToRotorRatio()));
      mSim.setMechanismPosition(angle);
    });
    // TODO: Set external encoders other than CANCoders
//    m_dcmotorSim.ifPresent(dcMotorSim -> dcMotorSim.setAngle(angle.in(Radians)));

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
    setEncoderPosition(m_config.convertToMechanism(distance));
  }

  /**
   * Ensure the controls request is sent successfully.
   *
   * @param controlRequest Control request to send.
   */
  private void ensureRequest(Supplier<StatusCode> controlRequest)
  {
    for (int i = 0; i < 8; i++)
    {
      if (controlRequest.get() == StatusCode.OK)
      {return;}
      Timer.delay(Milliseconds.of(1));
    }
  }

  @Override
  public void setPosition(Angle angle)
  {
    setpointVelocity = Optional.empty();
    setpointPosition = Optional.ofNullable(angle);
    if (angle != null && m_lqr.isEmpty())
    {
      switch (m_positionReq.getName())
      {
        case "MotionMagicDutyCycle":
          ensureRequest(() -> m_talonfx.setControl(((MotionMagicDutyCycle) m_positionReq).withPosition(angle)));
          break;
        case "MotionMagicExpoDutyCycle":
          ensureRequest(() -> m_talonfx.setControl(((MotionMagicExpoDutyCycle) m_positionReq).withPosition(angle)));
          break;
        case "MotionMagicExpoVoltage":
          ensureRequest(() -> m_talonfx.setControl(((MotionMagicExpoVoltage) m_positionReq).withPosition(angle)));
          break;
        case "MotionMagicTorqueCurrentFOC":
          ensureRequest(() -> m_talonfx.setControl(((MotionMagicTorqueCurrentFOC) m_positionReq).withPosition(angle)));
          break;
        case "MotionMagicVoltage":
          ensureRequest(() -> m_talonfx.setControl(((MotionMagicVoltage) m_positionReq).withPosition(angle)));
          break;
        case "PositionDutyCycle":
          ensureRequest(() -> m_talonfx.setControl(((PositionDutyCycle) m_positionReq).withPosition(angle)));
          break;
        case "PositionTorqueCurrentFOC":
          ensureRequest(() -> m_talonfx.setControl(((PositionTorqueCurrentFOC) m_positionReq).withPosition(angle)));
          break;
        case "PositionVoltage":
          ensureRequest(() -> m_talonfx.setControl(((PositionVoltage) m_positionReq).withPosition(angle)));
          break;
        default:
          throw new SmartMotorControllerConfigurationException(
              "TalonFX(" + m_talonfx.getDeviceID() + ") does not support the '" + m_positionReq.getName() +
              "' control request!", "Cannot use given control request", "withVendorControlRequest()");
      }
      m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setPosition(angle);}});
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
  public void setVelocity(AngularVelocity angularVelocity)
  {
    setpointPosition = Optional.empty();
    setpointVelocity = Optional.ofNullable(angularVelocity);
    if (angularVelocity != null && m_lqr.isEmpty())
    {
      switch (m_velocityReq.getName())
      {
        case "MotionMagicVelocityDutyCycle":
          ensureRequest(() -> m_talonfx.setControl(((MotionMagicVelocityDutyCycle) m_velocityReq).withVelocity(
              angularVelocity)));
          break;
        case "MotionMagicVelocityTorqueCurrentFOC":
          ensureRequest(() -> m_talonfx.setControl(((MotionMagicVelocityTorqueCurrentFOC) m_velocityReq).withVelocity(
              angularVelocity)));
          break;
        case "MotionMagicVelocityVoltage":
          ensureRequest(() -> m_talonfx.setControl(((MotionMagicVelocityVoltage) m_velocityReq).withVelocity(
              angularVelocity)));
          break;
        case "VelocityDutyCycle":
          ensureRequest(() -> m_talonfx.setControl(((VelocityDutyCycle) m_velocityReq).withVelocity(angularVelocity)));
          break;
        case "VelocityTorqueCurrentFOC":
          ensureRequest(() -> m_talonfx.setControl(((VelocityTorqueCurrentFOC) m_velocityReq).withVelocity(
              angularVelocity)));
          break;
        case "VelocityVoltage":
          ensureRequest(() -> m_talonfx.setControl(((VelocityVoltage) m_velocityReq).withVelocity(angularVelocity)));
          break;
        default:
          throw new SmartMotorControllerConfigurationException(
              "TalonFX(" + m_talonfx.getDeviceID() + ") does not support the '" + m_velocityReq.getName() +
              "' control request!", "Cannot use given control request", "withVendorControlRequest()");
      }
      m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setVelocity(angularVelocity);}});
//      m_simSupplier.ifPresent(simSupplier -> simSupplier.setMechanismVelocity(angularVelocity));
    }
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
    if (dutyCycle == 0.0)
    {
      m_looseFollowers.ifPresent(looseFollower -> {
        for (var follower : looseFollower) {follower.setDutyCycle(dutyCycle);}
      });
    }
    //m_simSupplier.ifPresent(simSupplier -> simSupplier.setMechanismStatorDutyCycle(dutyCycle));
  }

  @Override
  public boolean applyConfig(SmartMotorControllerConfig config)
  {
    config.resetValidationCheck();
    if (!m_config.getResetPreviousConfig())
    {m_configurator.refresh(m_talonConfig);}
    this.m_config = config;
    this.m_looseFollowers = config.getLooselyCoupledFollowers();
    m_lqr = config.getLQRClosedLoopController();
    // Closed loop controllers.
    m_config.getPID().ifPresent(pid -> {
      m_talonConfig.Slot0.kP = pid.getP();
      m_talonConfig.Slot0.kI = pid.getI();
      m_talonConfig.Slot0.kD = pid.getD();
    });
    m_config.getExponentialProfile().ifPresent(exp -> {
      m_expoProfile = Optional.of(new ExponentialProfile(exp));
      m_talonConfig.MotionMagic.MotionMagicExpo_kV = m_config.getLinearClosedLoopControllerUse() ?
                                                     m_config.convertToMechanism(Meters.of(-exp.A / exp.B))
                                                             .in(Rotations) :
                                                     (-exp.A / exp.B);
      m_talonConfig.MotionMagic.MotionMagicExpo_kA = m_config.getLinearClosedLoopControllerUse() ?
                                                     m_config.convertToMechanism(Meters.of(1.0 / exp.B))
                                                             .in(Rotations) : (1.0 / exp.B);

      m_positionReq = m_expoPositionReq;
    });
    m_config.getTrapezoidProfile().ifPresent(trap -> {
      m_trapezoidProfile = Optional.of(new TrapezoidProfile(trap));
      if (m_config.getVelocityTrapezoidalProfileInUse())
      {
        m_talonConfig.MotionMagic.MotionMagicAcceleration = m_config.getLinearClosedLoopControllerUse() ?
                                                            m_config.convertToMechanism(Meters.of(trap.maxVelocity))
                                                                    .in(Rotations) : trap.maxVelocity;
        m_talonConfig.MotionMagic.MotionMagicJerk = m_config.getLinearClosedLoopControllerUse() ?
                                                    m_config.convertToMechanism(Meters.of(trap.maxAcceleration))
                                                            .in(Rotations) : trap.maxAcceleration;
      } else
      {
        m_talonConfig.MotionMagic.MotionMagicCruiseVelocity = m_config.getLinearClosedLoopControllerUse() ?
                                                              m_config.convertToMechanism(Meters.of(trap.maxVelocity))
                                                                      .in(Rotations) : trap.maxVelocity;
        m_talonConfig.MotionMagic.MotionMagicAcceleration = m_config.getLinearClosedLoopControllerUse() ?
                                                            m_config.convertToMechanism(Meters.of(trap.maxAcceleration))
                                                                    .in(Rotations) : trap.maxAcceleration;
      }
      m_positionReq = m_trapPositionReq;
      m_velocityReq = m_trapVelocityReq;
    });

    if (m_lqr.isPresent())
    {
      if (m_config.getClosedLoopTolerance().isPresent())
      {
        throw new IllegalArgumentException("[ERROR] Cannot set closed-loop controller error tolerance on " +
                                           (config.getTelemetryName().isPresent() ? getName()
                                                                                  : "TalonFX(" +
                                                                                    m_talonfx.getDeviceID() + ")"));
      }
      System.err.println("====== TalonFX(" + m_talonfx.getDeviceID() + ")Using RIO Closed Loop Controller ======");

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

    if (config.getClosedLoopTolerance().isPresent())
    {
      throw new SmartMotorControllerConfigurationException("Closed loop tolerance is not available on TalonFX",
                                                           "Cannot set closed loop tolerance on TalonFX",
                                                           ".withClosedLoopTolerance");
    }

    // Fetch the controller mode to satisfy the requirement of knowing the control mode.
    config.getMotorControllerMode();

    // Feedforwards
    Optional<ArmFeedforward>         armFeedforward         = m_config.getArmFeedforward();
    Optional<ElevatorFeedforward>    elevatorFeedforward    = m_config.getElevatorFeedforward();
    Optional<SimpleMotorFeedforward> simpleMotorFeedforward = m_config.getSimpleFeedforward();
    if (armFeedforward.isPresent() || elevatorFeedforward.isPresent() ||
        simpleMotorFeedforward.isPresent())
    {
      double kS = 0, kV = 0, kA = 0, kG = 0;
      if (armFeedforward.isPresent())
      {
        var ff = armFeedforward.get();
        kS = ff.getKs();
        kV = ff.getKv();
        kA = ff.getKa();
        kG = ff.getKg();
        m_talonConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

      } else if (elevatorFeedforward.isPresent())
      {
        var ff = elevatorFeedforward.get();
        kS = ff.getKs();
        kV = ff.getKv();
        kA = ff.getKa();
        kG = ff.getKg();
        m_talonConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
      } else
      {
        var ff = simpleMotorFeedforward.get();
        kS = ff.getKs();
        kV = ff.getKv();
        kA = ff.getKa();
      }
      m_talonConfig.Slot0.kS = kS;
      m_talonConfig.Slot0.kV = kV;
      m_talonConfig.Slot0.kA = kA;
      m_talonConfig.Slot0.kG = kG;
    }

    // Motor inversion
    config.getMotorInverted().ifPresent(inverted -> {
      m_talonConfig.MotorOutput.Inverted =
          inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    });
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
    config.getClosedLoopRampRate().ifPresent(rampRate -> {
      m_talonConfig.ClosedLoopRamps.withDutyCycleClosedLoopRampPeriod(rampRate)
                                   .withVoltageClosedLoopRampPeriod(rampRate)
                                   .withTorqueClosedLoopRampPeriod(rampRate);
    });
    config.getOpenLoopRampRate().ifPresent(rampRate -> {
      m_talonConfig.OpenLoopRamps.withDutyCycleOpenLoopRampPeriod(rampRate)
                                 .withVoltageOpenLoopRampPeriod(rampRate)
                                 .withTorqueOpenLoopRampPeriod(rampRate);
    });
    // Current limits
    if (config.getStatorStallCurrentLimit().isPresent())
    {
      m_talonConfig.CurrentLimits.withStatorCurrentLimitEnable(true)
                                 .withStatorCurrentLimit(config.getStatorStallCurrentLimit().getAsInt());
    }
    if (config.getSupplyStallCurrentLimit().isPresent())
    {
      m_talonConfig.CurrentLimits.withSupplyCurrentLimitEnable(true)
                                 .withSupplyCurrentLimit(config.getSupplyStallCurrentLimit().getAsInt());
    }
    // Soft limit
    if (config.getMechanismUpperLimit().isPresent())
    {
      m_talonConfig.SoftwareLimitSwitch.withForwardSoftLimitEnable(
                       config.getMotorControllerMode() == ControlMode.CLOSED_LOOP)
                                       .withForwardSoftLimitThreshold(config.getMechanismUpperLimit().get());
    }
    if (config.getMechanismLowerLimit().isPresent())
    {
      m_talonConfig.SoftwareLimitSwitch.withReverseSoftLimitEnable(
                       config.getMotorControllerMode() == ControlMode.CLOSED_LOOP)
                                       .withReverseSoftLimitThreshold(config.getMechanismLowerLimit().get());
    }

    // Configure external encoders
    boolean useExternalEncoder = config.getUseExternalFeedback();
    if (config.getExternalEncoder().isPresent() && useExternalEncoder)
    {
      // Starting position
      if (config.getStartingPosition().isPresent())
      {
        DriverStation.reportWarning("[WARNING] Starting position is not applied to " +
                                    (config.getTelemetryName().isPresent() ? getName()
                                                                           : ("TalonFX(" + m_talonfx.getDeviceID() +
                                                                              ")"))
                                    +
                                    " because an external encoder is used!", false);
      }
      // Set the gear ratio for external encoders.
      m_talonConfig.Feedback.RotorToSensorRatio = config.getGearing().getMechanismToRotorRatio() *
                                                  config.getExternalEncoderGearing().orElse(MechanismGearing.kOne)
                                                        .getRotorToMechanismRatio();
      // config.getExternalEncoderGearing().getMechanismToRotorRatio() *
      m_talonConfig.Feedback.SensorToMechanismRatio = config.getExternalEncoderGearing().orElse(MechanismGearing.kOne)
                                                            .getMechanismToRotorRatio();
      if (config.getExternalEncoder().get() instanceof CANcoder encoder)
      {
        m_cancoder = Optional.of((CANcoder) config.getExternalEncoder().get());
        var configurator = encoder.getConfigurator();
        var cfg          = new CANcoderConfiguration();
        configurator.refresh(cfg);
        m_talonConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        config.getExternalEncoderInverted().ifPresent(inversion -> {
          cfg.MagnetSensor.withSensorDirection(
              inversion ? SensorDirectionValue.Clockwise_Positive
                        : SensorDirectionValue.CounterClockwise_Positive);
        });

        // Configure feedback source for CANCoder
        m_talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        // Zero offset
        if (config.getZeroOffset().isPresent())
        {
          cfg.MagnetSensor.withMagnetOffset(config.getZeroOffset().get());
          m_talonConfig.Feedback.FeedbackRotorOffset = 0;
        }
        // Discontinuity Point
        if (config.getMaxDiscontinuityPoint().isPresent())
        {
          cfg.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(config.getMaxDiscontinuityPoint().get());
        }
        configurator.apply(cfg);
      } else if (config.getExternalEncoder().get() instanceof CANdi encoder)
      {
        m_candi = Optional.of((CANdi) config.getExternalEncoder().get());
        var configurator = encoder.getConfigurator();
        var cfg          = new CANdiConfiguration();
        configurator.refresh(cfg);
        m_talonConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        // Ensure pro uses best option.
        if (useCANdiPWM2())
        {
          m_talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANdiPWM2;
        }
        if (useCANdiPWM1())
        {
          m_talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANdiPWM1;
        }
        if (useCANdiPWM1())
        {
          config.getExternalEncoderInverted().ifPresent(cfg.PWM1::withSensorDirection);

          // Zero offset
          if (config.getZeroOffset().isPresent())
          {
            cfg.PWM1.withAbsoluteSensorOffset(config.getZeroOffset().get());
            m_talonConfig.Feedback.FeedbackRotorOffset = 0;

          }
          // Discontinuity point
          if (config.getMaxDiscontinuityPoint().isPresent())
          {
            cfg.PWM1.withAbsoluteSensorDiscontinuityPoint(config.getMaxDiscontinuityPoint().get());
          }
        } else if (useCANdiPWM2())
        {
          config.getExternalEncoderInverted().ifPresent(cfg.PWM2::withSensorDirection);
          // Zero offset
          if (config.getZeroOffset().isPresent())
          {
            cfg.PWM2.withAbsoluteSensorOffset(config.getZeroOffset().get());
            m_talonConfig.Feedback.FeedbackRotorOffset = 0;
          }
          // Discontinuity point
          if (config.getMaxDiscontinuityPoint().isPresent())
          {
            cfg.PWM2.withAbsoluteSensorDiscontinuityPoint(config.getMaxDiscontinuityPoint().get());
          }
        }
        configurator.apply(cfg);
      }

    } else
    {
      if (config.getExternalEncoderInverted().isPresent())
      {
        throw new SmartMotorControllerConfigurationException("External Encoder cannot be inverted if not present!",
                                                             "External encoder is not inverted!",
                                                             "withExternalEncoderInverted(false)");
      }

      if (config.getExternalEncoderGearing().isPresent())
      {
        throw new SmartMotorControllerConfigurationException("External Encoder cannot be set if not present!",
                                                             "External encoder gearing is not 1.0!",
                                                             "withExternalEncoderGearing(Rotations.of(1.0))");
      }

      m_talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
      m_talonConfig.Feedback.RotorToSensorRatio = 1.0;
      m_talonConfig.Feedback.SensorToMechanismRatio = config.getGearing().getMechanismToRotorRatio();

      // Starting position
      if (config.getStartingPosition().isPresent())
      {
        m_configurator.apply(m_talonConfig);
        if (RobotBase.isSimulation())
        {
          m_talonfx.getSimState().setRawRotorPosition(config.getStartingPosition().get()
                                                            .times(config.getGearing().getMechanismToRotorRatio()));
        }
        StatusCode applied;
        int        iterations = 0;
        do
        {
          applied = m_talonfx.setPosition(config.getStartingPosition().get());
          Timer.delay(Milliseconds.of(10).in(Seconds));
          iterations++;
          if (iterations > 100)
          {
            break;
          }
        } while (!applied.isOK());

      }
      // Discontinuity point
      if (config.getMaxDiscontinuityPoint().isPresent())
      {
        if (!config.getMaxDiscontinuityPoint().get().equals(Rotations.of(0.5)))
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
    if (config.getEncoderInverted().isPresent())
    {
      throw new SmartMotorControllerConfigurationException("Integrated encoder phase cannot be set",
                                                           "Cannot configure TalonFX!",
                                                           "withEncoderInverted(false)");
    }

    // Configure follower motors
    if (config.getFollowers().isPresent())
    {
      for (Pair<Object, Boolean> follower : config.getFollowers().get())
      {
        StatusCode applied;
        do
        {
          if (follower.getFirst() instanceof TalonFXS)
          {
            applied = ((TalonFXS) follower.getFirst()).setControl(new Follower(m_talonfx.getDeviceID(),
                                                                               follower.getSecond()
                                                                               ? MotorAlignmentValue.Opposed
                                                                               : MotorAlignmentValue.Aligned));


          } else if (follower.getFirst() instanceof TalonFX)
          {
            applied = ((TalonFX) follower.getFirst()).setControl(new Follower(m_talonfx.getDeviceID(),
                                                                              follower.getSecond()
                                                                              ? MotorAlignmentValue.Opposed
                                                                              : MotorAlignmentValue.Aligned));
          } else
          {
            throw new IllegalArgumentException(
                "[ERROR] Unknown follower type: " + follower.getFirst().getClass().getSimpleName());
          }
          Timer.delay(Milliseconds.of(10).in(Seconds));
        } while (!applied.equals(StatusCode.OK));
      }
      config.clearFollowers();
    }

    if (config.getVendorControlRequest().isPresent())
    {
      var req = config.getVendorControlRequest().get();
      if (req instanceof ControlRequest)
      {
        switch (((ControlRequest) req).getName())
        {
          case "MotionMagicDutyCycle":
          case "MotionMagicExpoDutyCycle":
          case "MotionMagicExpoVoltage":
          case "MotionMagicTorqueCurrentFOC":
          case "MotionMagicVoltage":
          case "PositionDutyCycle":
          case "PositionTorqueCurrentFOC":
          case "PositionVoltage":
            m_positionReq = ((ControlRequest) req);
            break;
          case "MotionMagicVelocityDutyCycle":
          case "MotionMagicVelocityTorqueCurrentFOC":
          case "MotionMagicVelocityVoltage":
          case "VelocityDutyCycle":
          case "VelocityTorqueCurrentFOC":
          case "VelocityVoltage":
            m_velocityReq = ((ControlRequest) req);
            break;
          default:
            throw new SmartMotorControllerConfigurationException(
                "TalonFX(" + m_talonfx.getDeviceID() + ") does not support the '" + ((ControlRequest) req).getName() +
                "' control request!", "Cannot use given control request", "withVendorControlRequest()");
        }
      } else
      {
        throw new SmartMotorControllerConfigurationException(
            "TalonFX(" + m_talonfx.getDeviceID() + ") does not support the '" + ((ControlRequest) req).getName() +
            "' control request!", "Cannot use given control request", "withVendorControlRequest()");
      }
    }

    // Unsupported options.
    // TODO: This isn't really unsupported but needs to be adjusted to 1microsecond since the control loop runs at that speed
    if (config.getClosedLoopControlPeriod().isPresent())
    {
      throw new IllegalArgumentException("[ERROR] ClosedLoopControlPeriod is not supported");
    }
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

    config.validateBasicOptions();
    config.validateExternalEncoderOptions();

    return forceConfigApply().isOK();
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
//    if (voltage.in(Volts) == 0.0)
//    {m_looseFollowers.ifPresent(looseFollower -> {for (var follower : looseFollower) {follower.setVoltage(voltage);}});}
  }

  @Override
  public DCMotor getDCMotor()
  {
    return m_dcmotor;
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
    /*if (m_cancoder.isPresent())
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
    }*/
    return m_mechanismVelocity.refresh().getValue();
  }

  @Override
  public Angle getMechanismPosition()
  {
    /*if (m_cancoder.isPresent())
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
    }*/
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
  public void setMotorInverted(boolean inverted)
  {
    m_config.withMotorInverted(inverted);
    m_talonConfig.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    forceConfigApply();
  }

  @Override
  public void setEncoderInverted(boolean inverted)
  {
//    config.withEncoderInverted(inverted);
    // TODO: Support other encoders.
//    m_talonConfig.ExternalFeedback.withSensorPhase(inverted ? SensorPhaseValue.Opposed : SensorPhaseValue.Aligned);
//    m_configurator.apply(m_talonConfig);
  }

  @Override
  public void setMotionProfileMaxVelocity(LinearVelocity maxVelocity)
  {
    if (m_trapezoidProfile.isPresent())
    {
      m_trapezoidProfile = Optional.of(new TrapezoidProfile(new Constraints(maxVelocity.in(MetersPerSecond),
                                                                            m_config.getTrapezoidProfile()
                                                                                    .orElseThrow().maxAcceleration)));
    }
    if (m_config.getVelocityTrapezoidalProfileInUse())
    {
      m_talonConfig.MotionMagic.MotionMagicAcceleration = m_config.convertToMechanism(maxVelocity)
                                                                  .in(RotationsPerSecond);
    } else
    {m_talonConfig.MotionMagic.withMotionMagicCruiseVelocity(m_config.convertToMechanism(maxVelocity));}
    forceConfigApply();
    m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setMotionProfileMaxVelocity(maxVelocity);}});
  }

  @Override
  public void setMotionProfileMaxAcceleration(LinearAcceleration maxAcceleration)
  {
    if (m_trapezoidProfile.isPresent())
    {
      m_trapezoidProfile = Optional.of(new TrapezoidProfile(new Constraints(m_config.getTrapezoidProfile()
                                                                                    .orElseThrow().maxVelocity,
                                                                            maxAcceleration.in(MetersPerSecondPerSecond))));
    }
    if (m_config.getVelocityTrapezoidalProfileInUse())
    {
      m_talonConfig.MotionMagic.MotionMagicJerk = m_config.convertToMechanism(maxAcceleration).in(
          RotationsPerSecondPerSecond);
    } else
    {m_talonConfig.MotionMagic.withMotionMagicAcceleration(m_config.convertToMechanism(maxAcceleration));}
    forceConfigApply();
    m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setMotionProfileMaxAcceleration(maxAcceleration);}});
  }

  @Override
  public void setMotionProfileMaxVelocity(AngularVelocity maxVelocity)
  {
    if (m_trapezoidProfile.isPresent())
    {
      m_trapezoidProfile = Optional.of(new TrapezoidProfile(new Constraints(maxVelocity.in(RotationsPerSecond),
                                                                            m_config.getTrapezoidProfile()
                                                                                    .orElseThrow().maxAcceleration)));
    }
    if (m_config.getVelocityTrapezoidalProfileInUse())
    {
      m_talonConfig.MotionMagic.MotionMagicAcceleration = maxVelocity.in(RotationsPerSecond);
    } else
    {m_talonConfig.MotionMagic.withMotionMagicCruiseVelocity(maxVelocity);}
    forceConfigApply();
    m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setMotionProfileMaxVelocity(maxVelocity);}});

  }

  @Override
  public void setMotionProfileMaxAcceleration(AngularAcceleration maxAcceleration)
  {
    if (m_trapezoidProfile.isPresent())
    {
      m_trapezoidProfile = Optional.of(new TrapezoidProfile(
          new Constraints(m_config.getTrapezoidProfile()
                                  .orElseThrow().maxVelocity,
                          maxAcceleration.in(RotationsPerSecondPerSecond))));
    }
    if (m_config.getVelocityTrapezoidalProfileInUse())
    {
      m_talonConfig.MotionMagic.MotionMagicJerk = maxAcceleration.in(RotationsPerSecondPerSecond);
    } else
    {m_talonConfig.MotionMagic.withMotionMagicAcceleration(maxAcceleration);}
    forceConfigApply();
    m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setMotionProfileMaxAcceleration(maxAcceleration);}});
  }

  @Override
  public void setMotionProfileMaxJerk(Velocity<AngularAccelerationUnit> maxJerk)
  {
    if (m_trapezoidProfile.isPresent())
    {
      m_trapezoidProfile = Optional.of(new TrapezoidProfile(
          new Constraints(m_config.getTrapezoidProfile()
                                  .orElseThrow().maxVelocity,
                          maxJerk.in(RotationsPerSecondPerSecond.per(Second)))));
    }
    m_talonConfig.MotionMagic.MotionMagicJerk = maxJerk.in(RotationsPerSecondPerSecond.per(Second));
    forceConfigApply();
    m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setMotionProfileMaxJerk(maxJerk);}});
  }

  @Override
  public void setExponentialProfile(OptionalDouble kV, OptionalDouble kA, Optional<Voltage> maxInput)
  {
    if (m_expoProfile.isPresent() && m_config.getExponentialProfile().isPresent())
    {
      var exp = m_config.getExponentialProfile().get();
      var defaultkV = m_config.getLinearClosedLoopControllerUse() ?
                      m_config.convertToMechanism(Meters.of(-exp.A / exp.B))
                              .in(Rotations) :
                      (-exp.A / exp.B);
      var defaultkA = m_config.getLinearClosedLoopControllerUse() ?
                      m_config.convertToMechanism(Meters.of(1.0 / exp.B))
                              .in(Rotations) : (1.0 / exp.B);
      var defaultMaxInput = exp.maxInput;
      m_expoProfile = Optional.of(new ExponentialProfile(ExponentialProfile.Constraints
                                                             .fromCharacteristics(kV.orElse(defaultkV),
                                                                                  kA.orElse(defaultkA),
                                                                                  maxInput.orElse(Volts.of(
                                                                                              defaultMaxInput))
                                                                                          .in(Volts))));

      m_talonConfig.MotionMagic.MotionMagicExpo_kV = kV.orElse(defaultkV);
      m_talonConfig.MotionMagic.MotionMagicExpo_kA = kA.orElse(defaultkA);
      forceConfigApply();
      m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setExponentialProfile(kV, kA, maxInput);}});
    }
  }

  /**
   * Ensure setting is applied, retries every 10ms.
   *
   * @return {@link StatusCode} from the device.
   */
  public StatusCode forceConfigApply()
  {
    StatusCode status = m_configurator.apply(m_talonConfig);

    for (int i = 0; i < 10 && !status.isOK(); i++)
    {
      Timer.delay(Milliseconds.of(10).in(Seconds));
      status = m_configurator.apply(m_talonConfig);
    }
    return status;
  }

  @Override
  public void setKp(double kP)
  {
    m_config.getPID().ifPresent(pidController -> {
      pidController.setP(kP);
    });
    m_talonConfig.Slot0.kP = kP;
    forceConfigApply();
    m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setKp(kP);}});
  }

  @Override
  public void setKi(double kI)
  {
    m_config.getPID().ifPresent(pidController -> {
      pidController.setI(kI);
    });
    m_talonConfig.Slot0.kI = kI;
    forceConfigApply();
    m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setKi(kI);}});
  }

  @Override
  public void setKd(double kD)
  {
    m_config.getPID().ifPresent(pidController -> {
      pidController.setD(kD);
    });
    m_talonConfig.Slot0.kD = kD;
    forceConfigApply();
    m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setKd(kD);}});
  }

  @Override
  public void setFeedback(double kP, double kI, double kD)
  {
    m_pid.ifPresent(simplePidController -> {
      simplePidController.setP(kP);
      simplePidController.setI(kI);
      simplePidController.setD(kD);
    });
    m_talonConfig.Slot0.kP = kP;
    m_talonConfig.Slot0.kI = kI;
    m_talonConfig.Slot0.kD = kD;
    forceConfigApply();
    m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setFeedback(kP, kI, kD);}});
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
    m_talonConfig.Slot0.kS = kS;
    forceConfigApply();
    m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setKs(kS);}});
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
    m_talonConfig.Slot0.kV = kV;
    forceConfigApply();
    m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setKv(kV);}});
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
      m_config.withFeedforward(elevatorFeedforward);
    });
    m_talonConfig.Slot0.kA = kA;
    forceConfigApply();
    m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setKa(kA);}});
  }

  @Override
  public void setKg(double kG)
  {
    m_config.getArmFeedforward().ifPresent(armFeedforward -> {
      armFeedforward.setKg(kG);
      //m_config.withFeedforward(armFeedforward);
    });
    m_config.getElevatorFeedforward().ifPresent(elevatorFeedforward -> {
      elevatorFeedforward.setKg(kG);
      //m_config.withFeedforward(elevatorFeedforward);
    });
    m_talonConfig.Slot0.kG = kG;
    forceConfigApply();
    m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setKg(kG);}});
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
      m_talonConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    });
    m_config.getElevatorFeedforward().ifPresent(elevatorFeedforward -> {
      elevatorFeedforward.setKs(kS);
      elevatorFeedforward.setKv(kV);
      elevatorFeedforward.setKa(kA);
      elevatorFeedforward.setKg(kG);
      m_talonConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    });
    m_talonConfig.Slot0.kS = kS;
    m_talonConfig.Slot0.kV = kV;
    m_talonConfig.Slot0.kA = kA;
    m_talonConfig.Slot0.kG = kG;
    forceConfigApply();
    m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setFeedforward(kS, kV, kA, kG);}});
  }

  @Override
  public void setStatorCurrentLimit(Current currentLimit)
  {
    m_config.withStatorCurrentLimit(currentLimit);
    m_talonConfig.CurrentLimits.withStatorCurrentLimit(currentLimit).withStatorCurrentLimitEnable(true);
    forceConfigApply();
    m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setStatorCurrentLimit(currentLimit);}});
  }

  @Deprecated
  public void setSupplyCurrentLimit(Current currentLimit)
  {
    m_talonConfig.CurrentLimits.withSupplyCurrentLimit(currentLimit).withSupplyCurrentLimitEnable(true);
    forceConfigApply();
    m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setSupplyCurrentLimit(currentLimit);}});
  }

  @Override
  public void setClosedLoopRampRate(Time rampRate)
  {
    m_config.withClosedLoopRampRate(rampRate);
    m_talonConfig.ClosedLoopRamps.withDutyCycleClosedLoopRampPeriod(rampRate);
    forceConfigApply();
    m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setClosedLoopRampRate(rampRate);}});
  }

  @Override
  public void setOpenLoopRampRate(Time rampRate)
  {
    m_config.withOpenLoopRampRate(rampRate);
    m_talonConfig.OpenLoopRamps.withDutyCycleOpenLoopRampPeriod(rampRate);
    forceConfigApply();
    m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setOpenLoopRampRate(rampRate);}});
  }

  @Override
  public void setMeasurementUpperLimit(Distance upperLimit)
  {
    if (m_config.getMechanismCircumference().isPresent() && m_config.getMechanismLowerLimit().isPresent())
    {
      m_config.withSoftLimit(m_config.convertFromMechanism(m_config.getMechanismLowerLimit().get()), upperLimit);
      m_talonConfig.SoftwareLimitSwitch.withForwardSoftLimitThreshold(m_config.convertToMechanism(upperLimit))
                                       .withForwardSoftLimitEnable(true);
      forceConfigApply();
      m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setMeasurementUpperLimit(upperLimit);}});
    }
  }

  @Override
  public void setMeasurementLowerLimit(Distance lowerLimit)
  {
    if (m_config.getMechanismCircumference().isPresent() && m_config.getMechanismUpperLimit().isPresent())
    {
      m_config.withSoftLimit(lowerLimit, m_config.convertFromMechanism(m_config.getMechanismUpperLimit().get()));
      m_talonConfig.SoftwareLimitSwitch.withReverseSoftLimitThreshold(m_config.convertToMechanism(lowerLimit))
                                       .withReverseSoftLimitEnable(true);
      forceConfigApply();
      m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setMeasurementLowerLimit(lowerLimit);}});
    }
  }

  @Override
  public void setMechanismUpperLimit(Angle upperLimit)
  {
    m_config.getMechanismLowerLimit().ifPresent(lowerLimit -> {
      m_config.withSoftLimit(lowerLimit, upperLimit);
    });
    m_talonConfig.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
                                     .withForwardSoftLimitThreshold(upperLimit);
    forceConfigApply();
    m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setMechanismUpperLimit(upperLimit);}});
  }

  @Override
  public void setMechanismLowerLimit(Angle lowerLimit)
  {
    m_config.getMechanismUpperLimit().ifPresent(upperLimit -> {
      m_config.withSoftLimit(lowerLimit, upperLimit);
    });
    m_talonConfig.SoftwareLimitSwitch.withReverseSoftLimitEnable(true)
                                     .withReverseSoftLimitThreshold(lowerLimit);
    forceConfigApply();
    m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setMechanismLowerLimit(lowerLimit);}});
  }

  @Override
  public void setMechanismLimits(Angle lower, Angle upper)
  {
    m_config.withSoftLimit(lower, upper);
    m_talonConfig.SoftwareLimitSwitch.withForwardSoftLimitThreshold(upper).withReverseSoftLimitThreshold(lower);
    forceConfigApply();
    m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setMechanismLimits(lower, upper);}});
  }

  @Override
  public void setMechanismLimitsEnabled(boolean enabled)
  {
    m_talonConfig.SoftwareLimitSwitch.withForwardSoftLimitEnable(enabled).withReverseSoftLimitEnable(enabled);
    forceConfigApply();
    m_looseFollowers.ifPresent(smcs -> {for (var f : smcs) {f.setMechanismLimitsEnabled(enabled);}});
  }

  @Override
  public Temperature getTemperature()
  {
    return m_deviceTemperature.refresh().getValue();
  }

  @Override
  public SmartMotorControllerConfig getConfig()
  {
    return m_config;
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

  @Override
  public Pair<Optional<List<BooleanTelemetryField>>, Optional<List<DoubleTelemetryField>>> getUnsupportedTelemetryFields()
  {
    return Pair.of(Optional.empty(), Optional.empty());
  }

  @Override
  public Config getSysIdConfig(Voltage maxVoltage, Velocity<VoltageUnit> stepVoltage, Time testDuration)
  {
    return new Config(stepVoltage,
                      maxVoltage,
                      testDuration,
                      state -> SignalLogger.writeString("state", state.toString()));
  }
}
