package yams.motorcontrollers;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import yams.exceptions.SmartMotorControllerConfigurationException;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.telemetry.SmartMotorControllerTelemetry;
import yams.telemetry.SmartMotorControllerTelemetry.BooleanTelemetryField;
import yams.telemetry.SmartMotorControllerTelemetry.DoubleTelemetryField;
import yams.telemetry.SmartMotorControllerTelemetryConfig;

/**
 * Smart motor controller wrapper for motor controllers.
 */
public abstract class SmartMotorController
{

  /**
   * Telemetry.
   */
  protected SmartMotorControllerTelemetry                 telemetry                     = new SmartMotorControllerTelemetry();
  /**
   * {@link SmartMotorControllerConfig} for the motor.
   */
  protected SmartMotorControllerConfig                    m_config;
  /**
   * Profiled PID controller for the motor controller.
   */
  protected Optional<ProfiledPIDController>               m_pidController               = Optional.empty();
  /**
   * Simple PID controller for the motor controller.
   */
  protected Optional<PIDController>                       m_simplePidController         = Optional.empty();
  /**
   * Setpoint position
   */
  protected Optional<Angle>                               setpointPosition              = Optional.empty();
  /**
   * Setpoint velocity.
   */
  protected Optional<AngularVelocity>                     setpointVelocity              = Optional.empty();
  /**
   * Thread of the closed loop controller.
   */
  protected Notifier                                      m_closedLoopControllerThread  = null;
  /**
   * Running status of the closed loop controller.
   */
  private   boolean                                       m_closedLoopControllerRunning = false;
  /**
   * Parent table for telemetry.
   */
  protected Optional<NetworkTable>                        parentTable                   = Optional.empty();
  /**
   * {@link SmartMotorController} telemetry table.
   */
  protected Optional<NetworkTable>                        telemetryTable                = Optional.empty();
  /**
   * {@link SmartMotorController} tuning table.
   */
  protected Optional<NetworkTable>                        tuningTable                   = Optional.empty();
  /**
   * Config for publishing specific telemetry.
   */
  protected Optional<SmartMotorControllerTelemetryConfig> telemetryConfig               = Optional.empty();
  /**
   * {@link SimSupplier} for the mechanism.
   */
  protected Optional<SimSupplier>                         m_simSupplier                 = Optional.empty();


  /**
   * Create a {@link SmartMotorController} wrapper from the provided motor controller object.
   *
   * @param motorController Motor controller object.
   * @param motorSim        {@link DCMotor} which the motor controller is connected too.
   * @param cfg             {@link SmartMotorControllerConfig} for the {@link SmartMotorController}
   * @return {@link SmartMotorController}.
   */
  public static SmartMotorController create(Object motorController, DCMotor motorSim, SmartMotorControllerConfig cfg)
  {
    return null;
  }

  /**
   * Compare {@link DCMotor}s to identify the given motor.
   *
   * @param a {@link DCMotor} a
   * @param b {@link DCMotor} b
   * @return True if same DC motor.
   */
  public boolean isMotor(DCMotor a, DCMotor b)
  {
    return a.stallTorqueNewtonMeters == b.stallTorqueNewtonMeters &&
           a.stallCurrentAmps == b.stallCurrentAmps &&
           a.freeCurrentAmps == b.freeCurrentAmps &&
           a.freeSpeedRadPerSec == b.freeSpeedRadPerSec &&
           a.KtNMPerAmp == b.KtNMPerAmp &&
           a.KvRadPerSecPerVolt == b.KvRadPerSecPerVolt &&
           a.nominalVoltageVolts == b.nominalVoltageVolts;
  }

  /**
   * Check config for safe values.
   */
  public void checkConfigSafety()
  {
    if (isMotor(getDCMotor(), DCMotor.getNeo550(1)))
    {
      if (m_config.getStatorStallCurrentLimit().isEmpty())
      {
        throw new SmartMotorControllerConfigurationException("Stator current limit is not defined for NEO550!",
                                                             "Safety check failed.",
                                                             "withStatorCurrentLimit(Current)");
      } else if (m_config.getStatorStallCurrentLimit().getAsInt() > 40)
      {
        throw new SmartMotorControllerConfigurationException("Stator current limit is too high for NEO550!",
                                                             "Safety check failed.",
                                                             "withStatorCurrentLimit(Current) where the Current is under 40A");

      }

    }
    if (isMotor(getDCMotor(), DCMotor.getNEO(1)))
    {
      if (m_config.getStatorStallCurrentLimit().isEmpty())
      {
        throw new SmartMotorControllerConfigurationException("Stator current limit is not defined for NEO!",
                                                             "Safety check failed.",
                                                             "withStatorCurrentLimit(Current)");
      } else if (m_config.getStatorStallCurrentLimit().getAsInt() > 60)
      {
        throw new SmartMotorControllerConfigurationException("Stator current limit is too high for NEO!",
                                                             "Safety check failed.",
                                                             "withStatorCurrentLimit(Current) where the Current is under 60A");

      }

    }
  }

  /**
   * Get the sim supplier.
   *
   * @return Sim supplier.
   */
  public Optional<SimSupplier> getSimSupplier()
  {
    return m_simSupplier;
  }

  /**
   * Set the {@link SimSupplier} Mechanism.
   *
   * @param mechanismSupplier Mechanism sim supplier.
   */
  public void setSimSupplier(SimSupplier mechanismSupplier)
  {
    m_simSupplier = Optional.of(mechanismSupplier);
  }

  /**
   * Stop the closed loop controller.
   */
  public void stopClosedLoopController()
  {
    if (m_closedLoopControllerThread != null)
    {
      m_closedLoopControllerThread.stop();
      m_closedLoopControllerRunning = false;
    }
  }

  /**
   * Start the closed loop controller with the period.
   */
  public void startClosedLoopController()
  {
    if (m_closedLoopControllerThread != null && m_config.getMotorControllerMode() == ControlMode.CLOSED_LOOP)
    {
      m_simplePidController.ifPresent(PIDController::reset);
      m_pidController.ifPresent(pid -> pid.reset(getMechanismPosition().in(Rotations),
                                                 getMechanismVelocity().in(RotationsPerSecond)));
      m_config.getMechanismCircumference().ifPresent(circumference -> {
        m_pidController.ifPresent(pid -> pid.reset(getMeasurementPosition().in(Meters),
                                                   getMeasurementVelocity().in(MetersPerSecond)));
      });
      m_closedLoopControllerThread.stop();
      m_closedLoopControllerThread.startPeriodic(m_config.getClosedLoopControlPeriod().orElse(Milliseconds.of(20))
                                                         .in(Seconds));
      m_closedLoopControllerRunning = true;
    }/* else if (config.getMotorControllerMode() == ControlMode.CLOSED_LOOP)
    {
      closedLoopControllerThread = new Notifier(this::iterateClosedLoopController);
      closedLoopControllerThread.startPeriodic(config.getClosedLoopControlPeriod().in(Seconds));
    }*/
  }

  /**
   * Iterate the closed loop controller. Feedforward are only applied with profiled pid controllers.
   */
  public void iterateClosedLoopController()
  {
    AtomicReference<Double>          pidOutputVoltage       = new AtomicReference<>((double) 0);
    double                           feedforward            = 0.0;
    Optional<Angle>                  mechLowerLimit         = m_config.getMechanismLowerLimit();
    Optional<Angle>                  mechUpperLimit         = m_config.getMechanismUpperLimit();
    Optional<ArmFeedforward>         armFeedforward         = m_config.getArmFeedforward();
    Optional<ElevatorFeedforward>    elevatorFeedforward    = m_config.getElevatorFeedforward();
    Optional<SimpleMotorFeedforward> simpleMotorFeedforward = m_config.getSimpleFeedforward();
    Optional<Temperature>            temperatureCutoff      = m_config.getTemperatureCutoff();
    Optional<Voltage>                maximumVoltage         = m_config.getClosedLoopControllerMaximumVoltage();

    synchronizeRelativeEncoder();

    if (!m_closedLoopControllerRunning)
    {return;}

    if (setpointPosition.isPresent())
    {
      if (mechLowerLimit.isPresent())
      {
        if (setpointPosition.get().lt(mechLowerLimit.get()))
        {
          DriverStation.reportWarning("[WARNING] Setpoint is lower than Mechanism " +
                                      (m_config.getTelemetryName().isPresent() ? m_config.getTelemetryName().get()
                                                                               : "Unnamed smart motor") +
                                      " lower limit, changing setpoint to lower limit.", false);
          setpointPosition = mechLowerLimit;
        }
      }
      if (mechUpperLimit.isPresent())
      {
        if (setpointPosition.get().gt(mechUpperLimit.get()))
        {
          DriverStation.reportWarning("[WARNING] Setpoint is higher than Mechanism " +
                                      (m_config.getTelemetryName().isPresent() ? getName()
                                                                               : "Unnamed smart motor") +
                                      " upper limit, changing setpoint to upper limit.", false);
          setpointPosition = mechUpperLimit;
        }
      }
    }

    if (m_pidController.isPresent() && setpointPosition.isPresent())
    {
      if (armFeedforward.isPresent())
      {
        pidOutputVoltage.set(m_pidController.get().calculate(getMechanismPosition().in(Rotations),
                                                             setpointPosition.get().in(Rotations)));
        feedforward = armFeedforward.get().calculateWithVelocities(getMechanismPosition().in(Rotations),
                                                                   getMechanismVelocity().in(
                                                                       RotationsPerSecond),
                                                                   m_pidController.get()
                                                                                  .getSetpoint().velocity);
      } else if (elevatorFeedforward.isPresent())
      {
        pidOutputVoltage.set(m_pidController.get().calculate(getMeasurementPosition().in(Meters),
                                                             m_config.convertFromMechanism(setpointPosition.get())
                                                                     .in(Meters)));
        feedforward = elevatorFeedforward.get().calculateWithVelocities(getMeasurementVelocity().in(
            MetersPerSecond), m_pidController.get().getSetpoint().velocity);

      } else if (simpleMotorFeedforward.isPresent())
      {
        pidOutputVoltage.set(m_pidController.get().calculate(getMechanismPosition().in(Rotations),
                                                             setpointPosition.get().in(Rotations)));
        feedforward = simpleMotorFeedforward.get().calculateWithVelocities(getMechanismVelocity().in(
            RotationsPerSecond), m_pidController.get().getSetpoint().velocity);

      }
    } else
    {
      if (setpointPosition.isPresent())
      {
        m_simplePidController.ifPresent(pid -> {
          pidOutputVoltage.set(pid.calculate(getMechanismPosition().in(Rotations),
                                             setpointPosition.get().in(Rotations)));
        });
        m_pidController.ifPresent(pid -> {
          pidOutputVoltage.set(pid.calculate(getMechanismPosition().in(Rotations),
                                             setpointPosition.get().in(Rotations)));
        });
      } else if (setpointVelocity.isPresent())
      {
        m_simplePidController.ifPresent(pid -> {
          pidOutputVoltage.set(pid.calculate(getMechanismVelocity().in(RotationsPerSecond),
                                             setpointVelocity.get().in(RotationsPerSecond)));
        });
        m_pidController.ifPresent(pid -> {
          pidOutputVoltage.set(pid.calculate(getMechanismVelocity().in(RotationsPerSecond),
                                             setpointVelocity.get().in(RotationsPerSecond)));
        });
      }
    }
    if (mechUpperLimit.isPresent())
    {
      if (getMechanismPosition().gt(mechUpperLimit.get()) &&
          (pidOutputVoltage.get() + feedforward) > 0)
      {
        pidOutputVoltage.set(feedforward = 0);
      }
    }
    if (mechLowerLimit.isPresent())
    {
      if (getMechanismPosition().lt(mechLowerLimit.get()) &&
          (pidOutputVoltage.get() + feedforward) < 0)
      {
        pidOutputVoltage.set(feedforward = 0);
      }
    }
    if (temperatureCutoff.isPresent())
    {
      if (getTemperature().gte(temperatureCutoff.get()))
      {
        pidOutputVoltage.set(feedforward = 0);
      }
    }
//    telemetry.pidOutputVoltage = pidOutputVoltage;
//    telemetry.feedforwardVoltage = feedforward;
//    telemetry.outputVoltage = pidOutputVoltage + feedforward;
    double outputVoltage = pidOutputVoltage.get() + feedforward;
    if (maximumVoltage.isPresent())
    {
      double maxVolts = maximumVoltage.get().in(Volts);
      outputVoltage = MathUtil.clamp(outputVoltage, -maxVolts, maxVolts);
//      telemetry.outputVoltage = MathUtil.clamp(telemetry.outputVoltage, -maximumVoltage, maximumVoltage);
    }
    setVoltage(Volts.of(outputVoltage));
  }

  /**
   * Setup the simulation for the wrapper.
   */
  public abstract void setupSimulation();

  /**
   * Seed the relative encoder with the position from the absolute encoder.
   */
  public abstract void seedRelativeEncoder();

  /**
   * Check if the relative encoder is out of sync with absolute encoder within defined tolerances.
   */
  public abstract void synchronizeRelativeEncoder();

  /**
   * Simulation iteration.
   */
  public abstract void simIterate();

  /**
   * Set the encoder velocity
   *
   * @param velocity {@link AngularVelocity} of the Mechanism.
   */
  public abstract void setEncoderVelocity(AngularVelocity velocity);

  /**
   * Set the encoder velocity.
   *
   * @param velocity Measurement {@link LinearVelocity}
   */
  public abstract void setEncoderVelocity(LinearVelocity velocity);

  /**
   * Set the encoder position
   *
   * @param angle Mechanism {@link Angle} to reach.
   */
  public abstract void setEncoderPosition(Angle angle);

  /**
   * Set the encoder position.
   *
   * @param distance Measurement {@link Distance} to reach.
   */
  public abstract void setEncoderPosition(Distance distance);

  /**
   * Set the Mechanism {@link Angle} using the PID and feedforward from {@link SmartMotorControllerConfig}.
   *
   * @param angle Mechanism angle to set.
   */
  public abstract void setPosition(Angle angle);

  /**
   * Set the Mechanism {@link Distance} using the PID and feedforward from {@link SmartMotorControllerConfig}.
   *
   * @param distance Mechanism {@link Distance} to set.
   */
  public abstract void setPosition(Distance distance);

  /**
   * Set the Mechanism {@link LinearVelocity} using the PID and feedforward from {@link SmartMotorControllerConfig}.
   *
   * @param velocity Mechanism {@link LinearVelocity} to target.
   */
  public abstract void setVelocity(LinearVelocity velocity);

  /**
   * Set the Mechanism {@link AngularVelocity} using the PID and feedforward from {@link SmartMotorControllerConfig}.
   *
   * @param angle Mechanism {@link AngularVelocity} to target.
   */
  public abstract void setVelocity(AngularVelocity angle);

  /**
   * Get the SysIdConfig which may need to have modifications based on the SmartMotorController, like TalonFX and
   * TalonFXS to record states correctly.
   *
   * @param maxVoltage   Maximum voltage of the {@link SysIdRoutine}.
   * @param stepVoltage  Step voltage for the dynamic test in {@link SysIdRoutine}.
   * @param testDuration Duration of each {@link SysIdRoutine} run.
   * @return {@link Config} of the {@link SysIdRoutine} to run.
   */
  protected Config getSysIdConfig(Voltage maxVoltage, Velocity<VoltageUnit> stepVoltage, Time testDuration)
  {
    return new Config(stepVoltage, maxVoltage, testDuration);
  }

  /**
   * Run the  {@link SysIdRoutine} which runs to the maximum MEASUREMENT at the step voltage then down to the minimum
   * MEASUREMENT with the step voltage then up to the maximum MEASUREMENT increasing each second by the step voltage
   * generated via the {@link SmartMotorControllerConfig}.
   *
   * @param maxVoltage   Maximum voltage of the {@link SysIdRoutine}.
   * @param stepVoltage  Step voltage for the dynamic test in {@link SysIdRoutine}.
   * @param testDuration Duration of each {@link SysIdRoutine} run.
   * @return Sequential command group of {@link SysIdRoutine} running all required tests to the configured MINIMUM and
   * MAXIMUM MEASUREMENTS.
   */
  public SysIdRoutine sysId(Voltage maxVoltage, Velocity<VoltageUnit> stepVoltage, Time testDuration)
  {
    SysIdRoutine sysIdRoutine = null;
    if (m_config.getTelemetryName().isEmpty())
    {
      throw new SmartMotorControllerConfigurationException("Telemetry is undefined",
                                                           "Cannot create SysIdRoutine",
                                                           "withTelemetry(String,TelemetryVerbosity)");
    }
    Config sysIdConfig = getSysIdConfig(maxVoltage, stepVoltage, testDuration);
    if (m_config.getMechanismCircumference().isPresent())
    {
      sysIdRoutine = new SysIdRoutine(sysIdConfig,
                                      new SysIdRoutine.Mechanism(
                                          this::setVoltage,
                                          log -> {
                                            log.motor(getName())
                                               .voltage(
                                                   getVoltage())
                                               .linearVelocity(getMeasurementVelocity())
                                               .linearPosition(getMeasurementPosition());
                                          },
                                          m_config.getSubsystem()));
    } else
    {
      sysIdRoutine = new SysIdRoutine(sysIdConfig,
                                      new SysIdRoutine.Mechanism(
                                          this::setVoltage,
                                          log -> {
                                            log.motor(getName())
                                               .voltage(
                                                   getVoltage())
                                               .angularPosition(getMechanismPosition())
                                               .angularVelocity(getMechanismVelocity());
                                          },
                                          m_config.getSubsystem()));
    }
    return sysIdRoutine;
  }

  /**
   * Apply the {@link SmartMotorControllerConfig} to the {@link SmartMotorController}.
   *
   * @param config {@link SmartMotorControllerConfig} to use.
   * @return Successful Application of the configuration.
   */
  public abstract boolean applyConfig(SmartMotorControllerConfig config);

  /**
   * Get the duty cycle output of the motor controller.
   *
   * @return DutyCyle of the motor controller.
   */
  public abstract double getDutyCycle();

  /**
   * Set the dutycycle output of the motor controller.
   *
   * @param dutyCycle Value between [-1,1]
   */
  public abstract void setDutyCycle(double dutyCycle);

  /**
   * Get the supply current of the motor controller.
   *
   * @return The supply current of the motor controller.
   */
  public abstract Optional<Current> getSupplyCurrent();

  /**
   * Get the stator current of the motor controller.
   *
   * @return Stator current
   */
  public abstract Current getStatorCurrent();

  /**
   * Get the voltage output of the motor.
   *
   * @return Voltage output of the motor.
   */
  public abstract Voltage getVoltage();

  /**
   * Set the voltage output of the motor controller. Useful for Sysid.
   *
   * @param voltage Voltage to set the motor controller output to.
   */
  public abstract void setVoltage(Voltage voltage);

  /**
   * Get the {@link DCMotor} modeling the motor controlled by the motor controller.
   *
   * @return {@link DCMotor} of the controlled motor.
   */
  public abstract DCMotor getDCMotor();


  /**
   * Get the usable measurement of the motor for mechanisms operating under distance units converted with the
   * {@link SmartMotorControllerConfig}.
   *
   * @return Measurement velocity of the mechanism post-gearing.
   */
  public abstract LinearVelocity getMeasurementVelocity();

  /**
   * Get the usable measurement of the motor for mechanisms operating under distance units converted with the
   * {@link SmartMotorControllerConfig}.
   *
   * @return Measurement velocity of the mechanism post-gearing.
   */
  public abstract Distance getMeasurementPosition();

  /**
   * Get the Mechanism {@link AngularVelocity} taking the configured {@link MechanismGearing} into the measurement
   * applied via the {@link SmartMotorControllerConfig}.
   *
   * @return Mechanism {@link AngularVelocity}
   */
  public abstract AngularVelocity getMechanismVelocity();

  /**
   * Get the mechanism {@link Angle} taking the configured {@link MechanismGearing} from
   * {@link SmartMotorControllerConfig}.
   *
   * @return Mechanism {@link Angle}
   */
  public abstract Angle getMechanismPosition();

  /**
   * Gets the angular velocity of the motor.
   *
   * @return {@link AngularVelocity} of the relative motor encoder.
   */
  public abstract AngularVelocity getRotorVelocity();

  /**
   * Get the rotations of the motor with the relative encoder since the motor controller powered on scaled to the
   * mechanism rotations.
   *
   * @return {@link Angle} of the relative encoder in the motor.
   */
  public abstract Angle getRotorPosition();

  /**
   * Update the telemetry under the motor name under the given {@link NetworkTable}
   *
   * @param telemetry {@link NetworkTable} to create the {@link SmartMotorControllerTelemetry} subtable under based off
   *                  of {@link SmartMotorControllerConfig#getTelemetryName()}.
   * @param tuning    {@link NetworkTable} to create the tunable telemetry from {@link SmartMotorControllerTelemetry}
   *                  subtable under. Based off of {@link SmartMotorControllerConfig#getTelemetryName()}.
   */
  public void setupTelemetry(NetworkTable telemetry, NetworkTable tuning)
  {
    System.out.println(
        "=====================================================\nSETUP TELEMETRY\n=====================================================");
    if (parentTable.isEmpty())
    {
      parentTable = Optional.of(telemetry);
      if (m_config.getTelemetryName().isPresent())
      {
        telemetryTable = Optional.of(telemetry.getSubTable(getName()));
        tuningTable = Optional.of(tuning.getSubTable(getName()));
        if (m_config.getSmartControllerTelemetryConfig().isPresent())
        {
          this.telemetry.setupTelemetry(this,
                                        telemetryTable.get(),
                                        tuningTable.get(),
                                        m_config.getSmartControllerTelemetryConfig().get());
        } else
        {
          this.telemetry.setupTelemetry(this, telemetryTable.get(), tuningTable.get(),
                                        new SmartMotorControllerTelemetryConfig().withTelemetryVerbosity(m_config.getVerbosity()
                                                                                                                 .orElse(
                                                                                                                     TelemetryVerbosity.HIGH)));
        }
        updateTelemetry();
        Command liveTuningCommand = Commands.run(() -> this.telemetry.applyTuningValues(this),
                                                 m_config.getSubsystem())
                                            .finallyDo(() -> System.err.println(
                                                "=====================================================\nLIVE TUNING MODE STOP\n====================================================="));
        liveTuningCommand.setName("LiveTuning");
        liveTuningCommand.setSubsystem(m_config.getSubsystem().getName());
        System.out.println("SmartDashboard/" + telemetryTable.get().getPath().substring(1));
        SmartDashboard.putData(telemetryTable.get().getPath().substring(1) + "/LiveTuning", liveTuningCommand);
      }
    }
  }

  /**
   * Update the telemetry under the motor name under the given {@link NetworkTable}
   */
  public void updateTelemetry()
  {
    if (telemetryTable.isPresent() && m_config.getVerbosity().isPresent())
    {
//      telemetry.refresh(this);
      telemetry.publish(this);
      // if(tuningTable.isPresent())
      //   telemetry.applyChanges(this);

    }
    // TODO: Update PID, Feedforward, current limits, soft limits, ramp rate, motor inversion, encoder inversion
  }

  /**
   * Set the inversion state of the motor.
   *
   * @param inverted Inverted motor.
   */
  public abstract void setMotorInverted(boolean inverted);

  /**
   * Set the phase of the encoder attached to the brushless motor.
   *
   * @param inverted Phase of the encoder.
   */
  public abstract void setEncoderInverted(boolean inverted);

  /**
   * Set the maximum velocity of the trapazoidal profile for the feedback controller.
   *
   * @param maxVelocity Maximum velocity, will be translated to MetersPerSecond.
   */
  public abstract void setMotionProfileMaxVelocity(LinearVelocity maxVelocity);

  /**
   * Set the maximum acceleration of the trapazoidal profile for the feedback controller.
   *
   * @param maxAcceleration Maximum acceleration, will be translated to MetersPerSecondPerSecond.
   */
  public abstract void setMotionProfileMaxAcceleration(LinearAcceleration maxAcceleration);

  /**
   * Set the maximum velocity for the trapazoidal profile for the feedback controller.
   *
   * @param maxVelocity Maximum velocity, will be translated to RotationsPerSecond.
   */
  public abstract void setMotionProfileMaxVelocity(AngularVelocity maxVelocity);

  /**
   * Set the maximum acceleration for the trapazoidal profile for the feedback controller.
   *
   * @param maxAcceleration Maximum acceleration, will be translated to RotationsPerSecondPerSecond.
   */
  public abstract void setMotionProfileMaxAcceleration(AngularAcceleration maxAcceleration);

  /**
   * Set kP for the feedback controller PID.
   *
   * @param kP kP
   */
  public abstract void setKp(double kP);

  /**
   * Set kI for the feedback controller PID.
   *
   * @param kI kI.
   */
  public abstract void setKi(double kI);

  /**
   * Set kD for the feedback controller PID.
   *
   * @param kD kD for the feedback controller PID.
   */
  public abstract void setKd(double kD);

  /**
   * Set the closed loop feedback controller PID.
   *
   * @param kP kP; Proportional scalar.
   * @param kI kI; Integral scalar.
   * @param kD kD; derivative scalar.
   */
  public abstract void setFeedback(double kP, double kI, double kD);

  /**
   * Static feedforward element.
   *
   * @param kS kS; Static feedforward.
   */
  public abstract void setKs(double kS);

  /**
   * Velocity feedforward element.
   *
   * @param kV kV; Velocity feedforward.
   */
  public abstract void setKv(double kV);

  /**
   * Acceleration feedforward element.
   *
   * @param kA kA; Acceleration feedforward.
   */
  public abstract void setKa(double kA);

  /**
   * kSin feedforward element.
   *
   * @param kG kG; Gravity feedforward.
   */
  public abstract void setKg(double kG);

  /**
   * Set the feedforward controller.
   *
   * @param kS kS; Static feedforward.
   * @param kV kV; Velocity feedforward.
   * @param kA kA; Acceleration feedforward.
   * @param kG kG; Gravity feedforward.
   */
  public abstract void setFeedforward(double kS, double kV, double kA, double kG);

  /**
   * Set the stator current limit for the device.
   *
   * @param currentLimit Stator current limit.
   */
  public abstract void setStatorCurrentLimit(Current currentLimit);

  /**
   * Set the supply current limit.
   *
   * @param currentLimit Supply current limit.
   */
  public abstract void setSupplyCurrentLimit(Current currentLimit);

  /**
   * Set the closed loop ramp rate. The ramp rate is how fast the motor can go from 0-100, measured in seconds.
   *
   * @param rampRate Time from 0 to 100.
   */
  public abstract void setClosedLoopRampRate(Time rampRate);

  /**
   * Set the open loop ramp rate. The ramp rate is how fast the motor can go from 0 to 100, measured in Seconds.
   *
   * @param rampRate Time it takes to go from 0 to 100.
   */
  public abstract void setOpenLoopRampRate(Time rampRate);

  /**
   * Set the measurement upper limit, only works if mechanism circumference is defined.
   *
   * @param upperLimit Upper limit, will be translated to meters.
   */
  public abstract void setMeasurementUpperLimit(Distance upperLimit);

  /**
   * Set the measurement lower limit, only works if mechanism circumference is defined.
   *
   * @param lowerLimit Lower limit, will be translated to meters.
   */
  public abstract void setMeasurementLowerLimit(Distance lowerLimit);

  /**
   * Set the mechanism upper limit.
   *
   * @param upperLimit Upper limit, will be translated to rotations.
   */
  public abstract void setMechanismUpperLimit(Angle upperLimit);

  /**
   * Set the mechanism lower limit.
   *
   * @param lowerLimit Lower limit, will be translated to rotations.
   */
  public abstract void setMechanismLowerLimit(Angle lowerLimit);


  /**
   * Get the {@link SmartMotorController} temperature.
   *
   * @return {@link Temperature}
   */
  public abstract Temperature getTemperature();

  /**
   * Get the {@link SmartMotorControllerConfig} for the {@link SmartMotorController}
   *
   * @return {@link SmartMotorControllerConfig} used.
   */
  public abstract SmartMotorControllerConfig getConfig();

  /**
   * Get the Motor Controller Object passed into the {@link SmartMotorController}.
   *
   * @return Motor Controller object.
   */
  public abstract Object getMotorController();

  /**
   * Get the motor controller object config generated by {@link SmartMotorController} based off the
   * {@link SmartMotorControllerConfig}
   *
   * @return Motor controller config.
   */
  public abstract Object getMotorControllerConfig();

  /**
   * Get the Mechanism setpoint position.
   *
   * @return Mechanism Setpoint position.
   */
  public Optional<Angle> getMechanismPositionSetpoint()
  {
    return setpointPosition;
  }

  /**
   * Get the Mechanism velocity setpoint.
   *
   * @return Mechanism velocity setpoint.
   */
  public Optional<AngularVelocity> getMechanismSetpointVelocity()
  {
    return setpointVelocity;
  }

  /**
   * Get a list of unsupported telemetry fields if any exist.
   *
   * @return Optional list of unsupported telemetry fields.
   */
  public abstract Pair<Optional<List<BooleanTelemetryField>>, Optional<List<DoubleTelemetryField>>> getUnsupportedTelemetryFields();

  /**
   * Get the name of the {@link SmartMotorController}
   *
   * @return {@link String} name if present, else "SmartMotorController"
   */
  public String getName()
  {
    return m_config.getTelemetryName().orElse("SmartMotorController");
  }

  /**
   * Close the SMC for unit testing.
   */
  public void close()
  {
    if (m_closedLoopControllerThread != null)
    {
      m_closedLoopControllerThread.stop();
      m_closedLoopControllerThread.close();
      m_closedLoopControllerThread = null;
    }
    telemetry.close();
  }
}
