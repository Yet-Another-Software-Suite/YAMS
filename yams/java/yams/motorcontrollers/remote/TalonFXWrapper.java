package yams.motorcontrollers.remote;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import java.util.List;
import java.util.Optional;
import yams.exceptions.SmartMotorControllerConfigurationException;
import yams.math.ExponentialProfilePIDController;
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
   * Velocity control request
   */
  private final VelocityVoltage               m_velocityReq     = new VelocityVoltage(0).withSlot(0);
  /**
   * Position with trapezoidal profiling request.
   */
  private final MotionMagicVoltage            m_trapPositionReq = new MotionMagicVoltage(0).withSlot(0);
  /**
   * Position with exponential profiling request.
   */
  private final MotionMagicExpoVoltage        m_expoPositionReq = new MotionMagicExpoVoltage(0).withSlot(0);
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
  private final StatusSignal<Temperature> m_deviceTemperature;
  /**
   * {@link CANcoder} to use as external feedback sensor.
   */
  private       Optional<CANcoder>        m_cancoder = Optional.empty();
  /**
   * {@link CANdi} to use as external feedback sensor.
   */
  private       Optional<CANdi>           m_candi    = Optional.empty();
  /**
   * Exponential profiled velocity control request enabled.
   */
  private       boolean                   expEnabled = false;
  /**
   * {@link DCMotorSim} for the {@link TalonFX}.
   */
  private       Optional<DCMotorSim>          m_dcmotorSim      = Optional.empty();

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
    m_closedLoopControllerThread = null;

    setupSimulation();
    applyConfig(smartConfig);
    checkConfigSafety();
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
                                                                                             .getRotorToMechanismRatio()),
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
      if (!m_simSupplier.get().getUpdatedSim())
      {
        m_simSupplier.get().updateSimState();
        m_simSupplier.get().starveUpdateSim();
      }
      var talonFXSim = m_talonfx.getSimState();

      // set the supply voltage of the TalonFX
      talonFXSim.setSupplyVoltage(m_simSupplier.get().getMechanismSupplyVoltage());

      // get the motor voltage of the TalonFX
      var motorVoltage = talonFXSim.getMotorVoltageMeasure();

      // use the motor voltage to calculate new position and velocity
      // using WPILib's DCMotorSim class for physics simulation
      // m_dcmotorSim.get().setInputVoltage(motorVoltage.in(Volts));
      /*m_dcmotorSim.ifPresent(sim -> {
        sim.setAngularVelocity(m_simSupplier.get().getMechanismVelocity().in(RadiansPerSecond));
        sim.update(config.getClosedLoopControlPeriod().in(Seconds));
      });*/

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
                                             .times(m_config.getExternalEncoderGearing().getMechanismToRotorRatio()));
        cancoderSim.setRawPosition(m_simSupplier.get().getMechanismPosition()
                                                .times(m_config.getExternalEncoderGearing()
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
                                                               .getMechanismToRotorRatio()));
          candiSim.setPwm1Velocity(m_simSupplier.get().getMechanismVelocity()
                                                .times(m_config.getExternalEncoderGearing()
                                                               .getMechanismToRotorRatio()));
        } else if (useCANdiPWM2())
        {
          candiSim.setPwm2Connected(true);
          candiSim.setPwm2Position(m_simSupplier.get().getMechanismPosition()
                                                .times(m_config.getExternalEncoderGearing()
                                                               .getMechanismToRotorRatio()));
          candiSim.setPwm2Velocity(m_simSupplier.get().getMechanismVelocity()
                                                .times(m_config.getExternalEncoderGearing()
                                                               .getMechanismToRotorRatio()));
        }
      }
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

  @Override
  public void setPosition(Angle angle)
  {
    setpointPosition = Optional.ofNullable(angle);
    if (angle != null)
    {
      m_talonfx.setControl(expEnabled ? m_expoPositionReq.withPosition(angle) : m_trapPositionReq.withPosition(angle));
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
    if (angle != null)
    {
      m_talonfx.setControl(m_velocityReq.withVelocity(angle));
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
    //m_simSupplier.ifPresent(simSupplier -> simSupplier.setMechanismStatorDutyCycle(dutyCycle));
  }

  @Override
  public boolean applyConfig(SmartMotorControllerConfig config)
  {
    config.resetValidationCheck();
    m_configurator.refresh(m_talonConfig);
    this.m_config = config;
    // Closed loop controllers.
    if (config.getClosedLoopController().isPresent() && config.getSimpleClosedLoopController().isPresent())
    {
      throw new SmartMotorControllerConfigurationException("ProfiledPIDController and PIDController defined",
                                                           "Cannot have both PID Controllers.",
                                                           ".withClosedLoopController");
    }

    if (config.getExponentiallyProfiledClosedLoopController().isPresent())
    {
      ExponentialProfilePIDController controller = config.getExponentiallyProfiledClosedLoopController().get();
      if (controller.getPositionTolerance() != 0.05)
      {
        throw new IllegalArgumentException("[ERROR] Cannot set closed-loop controller error tolerance on " +
                                           (config.getTelemetryName().isPresent() ? getName()
                                                                                  : "TalonFX(" +
                                                                                    m_talonfx.getDeviceID() + ")"));
      }

      m_talonConfig.Slot0.kP = controller.getP();
      m_talonConfig.Slot0.kI = controller.getI();
      m_talonConfig.Slot0.kD = controller.getD();
      m_talonConfig.MotionMagic
          .withMotionMagicExpo_kV(controller.getKv().in(RotationsPerSecond))
          .withMotionMagicExpo_kA(controller.getKa().in(RotationsPerSecondPerSecond));
      expEnabled = true;
//      if (config.getMechanismCircumference().isPresent())
//      {
//        //m_talonConfig.Slot0.kP = config.convertToMechanism(Meters.of(controller.getP())).in(Rotations);
//        //m_talonConfig.Slot0.kI = config.convertToMechanism(Meters.of(controller.getI())).in(Rotations);
//        //m_talonConfig.Slot0.kD = config.convertToMechanism(Meters.of(controller.getD())).in(Rotations);
//
//      } else
//      {
//        m_talonConfig.MotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(controller.getConstraints().maxAcceleration));
//        m_talonConfig.MotionMagic.withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(controller.getConstraints().maxAcceleration));
//
//      }
    } else if (config.getClosedLoopController().isPresent())
    {
      ProfiledPIDController controller = config.getClosedLoopController().get();
      if (controller.getPositionTolerance() != 0.05)
      {
        throw new IllegalArgumentException("[ERROR] Cannot set closed-loop controller error tolerance on " +
                                           (config.getTelemetryName().isPresent() ? getName()
                                                                                  : "TalonFX(" +
                                                                                    m_talonfx.getDeviceID() + ")"));
      }
      m_talonConfig.Slot0.kP = controller.getP();
      m_talonConfig.Slot0.kI = controller.getI();
      m_talonConfig.Slot0.kD = controller.getD();
      if (config.getMechanismCircumference().isPresent())
      {
//        m_talonConfig.Slot0.kP = config.convertToMechanism(Meters.of(controller.getP())).in(Rotations);
//        m_talonConfig.Slot0.kI = config.convertToMechanism(Meters.of(controller.getI())).in(Rotations);
//        m_talonConfig.Slot0.kD = config.convertToMechanism(Meters.of(controller.getD())).in(Rotations);
        m_talonConfig.MotionMagic
            .withMotionMagicCruiseVelocity(
                config.convertToMechanism(MetersPerSecond.of(controller.getConstraints().maxVelocity)));
        m_talonConfig.MotionMagic
            .withMotionMagicAcceleration(
                config.convertToMechanism(MetersPerSecondPerSecond.of(controller.getConstraints().maxAcceleration)));
      } else
      {
        m_talonConfig.MotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(controller.getConstraints().maxAcceleration));
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
                                           (config.getTelemetryName().isPresent() ? getName()
                                                                                  : "TalonFX(" +
                                                                                    m_talonfx.getDeviceID() + ")"));
      }
    } else if (config.getMotorControllerMode() == ControlMode.CLOSED_LOOP)
    {
      throw new IllegalArgumentException("[ERROR] No closed loop configuration available!");
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
                                                  config.getExternalEncoderGearing()
                                                        .getRotorToMechanismRatio();
      // config.getExternalEncoderGearing().getMechanismToRotorRatio() *
      m_talonConfig.Feedback.SensorToMechanismRatio = config.getExternalEncoderGearing()
                                                            .getMechanismToRotorRatio();
      if (config.getExternalEncoder().get() instanceof CANcoder encoder)
      {
        m_cancoder = Optional.of((CANcoder) config.getExternalEncoder().get());
        var configurator = encoder.getConfigurator();
        var cfg          = new CANcoderConfiguration();
        configurator.refresh(cfg);
        m_talonConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        cfg.MagnetSensor.withSensorDirection(
            config.getExternalEncoderInverted() ? SensorDirectionValue.Clockwise_Positive
                                                : SensorDirectionValue.CounterClockwise_Positive);

        // Configure feedback source for CANCoder
        m_talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        // Zero offset
        if (config.getZeroOffset().isPresent())
        {
          cfg.MagnetSensor.withMagnetOffset(encoder.getPosition().getValue()
                                                   .plus(Rotations.of(cfg.MagnetSensor.MagnetOffset))
                                                   .minus(config.getZeroOffset().get()));
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
          if (config.getMaxDiscontinuityPoint().isPresent())
          {
            cfg.PWM1.withAbsoluteSensorDiscontinuityPoint(config.getMaxDiscontinuityPoint().get());
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
          if (config.getMaxDiscontinuityPoint().isPresent())
          {
            cfg.PWM2.withAbsoluteSensorDiscontinuityPoint(config.getMaxDiscontinuityPoint().get());
          }
        }
        configurator.apply(cfg);
      }

    } else
    {
      if (config.getExternalEncoderInverted())
      {
        throw new SmartMotorControllerConfigurationException("External Encoder cannot be inverted if not present!",
                                                             "External encoder is not inverted!",
                                                             "withExternalEncoderInverted(false)");
      }

      if (config.getExternalEncoderGearing().getMechanismToRotorRatio() != 1.0)
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
        do
        {
          applied = m_talonfx.setPosition(config.getStartingPosition().get());
          Timer.delay(Milliseconds.of(10).in(Seconds));
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
    if (config.getEncoderInverted())
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
                                                                               follower.getSecond()));


          } else if (follower.getFirst() instanceof TalonFX)
          {
            applied = ((TalonFX) follower.getFirst()).setControl(new Follower(m_talonfx.getDeviceID(),
                                                                              follower.getSecond()));
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
    if (m_config.getClosedLoopController().isPresent())
    {
      ProfiledPIDController ctr = m_config.getClosedLoopController().get();
      ctr.setConstraints(new Constraints(maxVelocity.in(MetersPerSecond), ctr.getConstraints().maxAcceleration));
      m_config.withClosedLoopController(ctr);
    }
    m_talonConfig.MotionMagic.withMotionMagicCruiseVelocity(m_config.convertToMechanism(maxVelocity));
    forceConfigApply();
  }

  @Override
  public void setMotionProfileMaxAcceleration(LinearAcceleration maxAcceleration)
  {
    if (m_config.getClosedLoopController().isPresent())
    {
      ProfiledPIDController ctr = m_config.getClosedLoopController().get();
      ctr.setConstraints(new Constraints(ctr.getConstraints().maxVelocity,
                                         maxAcceleration.in(MetersPerSecondPerSecond)));
      m_config.withClosedLoopController(ctr);
    }
    m_talonConfig.MotionMagic.withMotionMagicAcceleration(m_config.convertToMechanism(maxAcceleration));
    forceConfigApply();
  }

  @Override
  public void setMotionProfileMaxVelocity(AngularVelocity maxVelocity)
  {
    if (m_config.getClosedLoopController().isPresent())
    {
      ProfiledPIDController ctr = m_config.getClosedLoopController().get();
      ctr.setConstraints(new Constraints(maxVelocity.in(RotationsPerSecond), ctr.getConstraints().maxAcceleration));
      m_config.withClosedLoopController(ctr);
    }
    m_talonConfig.MotionMagic.withMotionMagicCruiseVelocity(maxVelocity);
    forceConfigApply();
  }

  @Override
  public void setMotionProfileMaxAcceleration(AngularAcceleration maxAcceleration)
  {
    if (m_config.getClosedLoopController().isPresent())
    {
      ProfiledPIDController ctr = m_config.getClosedLoopController().get();
      ctr.setConstraints(new Constraints(ctr.getConstraints().maxVelocity,
                                         maxAcceleration.in(RotationsPerSecondPerSecond)));
      m_config.withClosedLoopController(ctr);
    }
    m_talonConfig.MotionMagic.withMotionMagicAcceleration(maxAcceleration);
    forceConfigApply();
  }

  /**
   * Ensure setting is applied.
   */
  public StatusCode forceConfigApply()
  {
    StatusCode status = m_configurator.apply(m_talonConfig);
    while (!status.isOK())
    {
      Timer.delay(Milliseconds.of(10).in(Seconds));
      status = m_configurator.apply(m_talonConfig);
    }
    return status;
  }

  @Override
  public void setKp(double kP)
  {
    m_config.getClosedLoopController().ifPresent(simplePidController -> {
      simplePidController.setP(kP);
    });
    m_config.getSimpleClosedLoopController().ifPresent(pidController -> {
      pidController.setP(kP);
    });
    m_config.getExponentiallyProfiledClosedLoopController().ifPresent(pidController -> {
      pidController.setP(kP);
    });
    m_talonConfig.Slot0.kP = kP;
    System.out.println("setKp: " + kP);
    forceConfigApply();
  }

  @Override
  public void setKi(double kI)
  {
    m_config.getClosedLoopController().ifPresent(simplePidController -> {
      simplePidController.setI(kI);
    });
    m_config.getSimpleClosedLoopController().ifPresent(pidController -> {
      pidController.setI(kI);
    });
    m_config.getExponentiallyProfiledClosedLoopController().ifPresent(pidController -> {
      pidController.setI(kI);
    });
    m_talonConfig.Slot0.kI = kI;
    forceConfigApply();
  }

  @Override
  public void setKd(double kD)
  {
    m_config.getClosedLoopController().ifPresent(simplePidController -> {
      simplePidController.setD(kD);
    });
    m_config.getSimpleClosedLoopController().ifPresent(pidController -> {
      pidController.setD(kD);
    });
    m_config.getExponentiallyProfiledClosedLoopController().ifPresent(pidController -> {
      pidController.setD(kD);
    });
    m_talonConfig.Slot0.kD = kD;
    forceConfigApply();
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
    m_talonConfig.Slot0.kP = kP;
    m_talonConfig.Slot0.kI = kI;
    m_talonConfig.Slot0.kD = kD;
    forceConfigApply();
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
  }

  @Override
  public void setStatorCurrentLimit(Current currentLimit)
  {
    m_config.withStatorCurrentLimit(currentLimit);
    m_talonConfig.CurrentLimits.withStatorCurrentLimit(currentLimit);
    m_talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    forceConfigApply();
  }

  @Deprecated
  public void setSupplyCurrentLimit(Current currentLimit)
  {
    m_talonConfig.CurrentLimits.withSupplyCurrentLimit(currentLimit);
    m_talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    forceConfigApply();
  }

  @Override
  public void setClosedLoopRampRate(Time rampRate)
  {
    m_config.withClosedLoopRampRate(rampRate);
    m_talonConfig.ClosedLoopRamps.withDutyCycleClosedLoopRampPeriod(rampRate);
    forceConfigApply();
  }

  @Override
  public void setOpenLoopRampRate(Time rampRate)
  {
    m_config.withOpenLoopRampRate(rampRate);
    m_talonConfig.OpenLoopRamps.withDutyCycleOpenLoopRampPeriod(rampRate);
    forceConfigApply();
  }

  @Override
  public void setMeasurementUpperLimit(Distance upperLimit)
  {
    if (m_config.getMechanismCircumference().isPresent() && m_config.getMechanismLowerLimit().isPresent())
    {
      m_config.withSoftLimit(m_config.convertFromMechanism(m_config.getMechanismLowerLimit().get()), upperLimit);
      m_talonConfig.SoftwareLimitSwitch.withForwardSoftLimitThreshold(m_config.convertToMechanism(upperLimit));
      forceConfigApply();
    }
  }

  @Override
  public void setMeasurementLowerLimit(Distance lowerLimit)
  {
    if (m_config.getMechanismCircumference().isPresent() && m_config.getMechanismUpperLimit().isPresent())
    {
      m_config.withSoftLimit(lowerLimit, m_config.convertFromMechanism(m_config.getMechanismUpperLimit().get()));
      m_talonConfig.SoftwareLimitSwitch.withReverseSoftLimitThreshold(m_config.convertToMechanism(lowerLimit));
      forceConfigApply();
    }
  }

  @Override
  public void setMechanismUpperLimit(Angle upperLimit)
  {
    m_config.getMechanismLowerLimit().ifPresent(lowerLimit -> {
      m_config.withSoftLimit(lowerLimit, upperLimit);
    });
    m_talonConfig.SoftwareLimitSwitch.withForwardSoftLimitThreshold(upperLimit);
    forceConfigApply();
  }

  @Override
  public void setMechanismLowerLimit(Angle lowerLimit)
  {
    m_config.getMechanismUpperLimit().ifPresent(upperLimit -> {
      m_config.withSoftLimit(lowerLimit, upperLimit);
    });
    m_talonConfig.SoftwareLimitSwitch.withReverseSoftLimitThreshold(lowerLimit);
    forceConfigApply();
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
  protected Config getSysIdConfig(Voltage maxVoltage, Velocity<VoltageUnit> stepVoltage, Time testDuration)
  {
    return new Config(stepVoltage,
                      maxVoltage,
                      testDuration,
                      state -> SignalLogger.writeString("state", state.toString()));
  }
}
