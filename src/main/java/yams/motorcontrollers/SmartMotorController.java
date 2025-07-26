package yams.motorcontrollers;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import java.util.Optional;
import yams.exceptions.SmartMotorControllerConfigurationException;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.telemetry.SmartMotorControllerTelemetry;
import yams.telemetry.SmartMotorControllerTelemetryConfig;

public abstract class SmartMotorController
{

  /**
   * Telemetry.
   */
  protected SmartMotorControllerTelemetry                 telemetry                  = new SmartMotorControllerTelemetry();
  /**
   * {@link SmartMotorControllerConfig} for the motor.
   */
  protected SmartMotorControllerConfig                    config;
  /**
   * Profiled PID controller for the motor controller.
   */
  protected Optional<ProfiledPIDController>               pidController              = Optional.empty();
  /**
   * Simple PID controller for the motor controller.
   */
  protected Optional<PIDController>                       simplePidController        = Optional.empty();
  /**
   * Setpoint position
   */
  protected Optional<Angle>                               setpointPosition           = Optional.empty();
  /**
   * Setpoint velocity.
   */
  protected Optional<AngularVelocity>                     setpointVelocity           = Optional.empty();
  /**
   * Thread of the closed loop controller.
   */
  protected Notifier                                      closedLoopControllerThread = null;
  /**
   * Parent table for telemetry.
   */
  protected Optional<NetworkTable>                        parentTable                = Optional.empty();
  /**
   * {@link SmartMotorController} telemetry table.
   */
  protected Optional<NetworkTable>                        telemetryTable             = Optional.empty();
  /**
   * {@link SmartMotorController} tuning table.
   */
  protected Optional<NetworkTable> tuningTable = Optional.empty();
  /**
   * Config for publishing specific telemetry.
   */
  protected Optional<SmartMotorControllerTelemetryConfig> telemetryConfig            = Optional.empty();


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
      if (config.getStatorStallCurrentLimit().isEmpty())
      {
        throw new SmartMotorControllerConfigurationException("Stator current limit is not defined for NEO550!",
                                                             "Safety check failed.",
                                                             "withStatorCurrentLimit(Current)");
      } else if (config.getStatorStallCurrentLimit().getAsInt() > 40)
      {
        throw new SmartMotorControllerConfigurationException("Stator current limit is too high for NEO550!",
                                                             "Safety check failed.",
                                                             "withStatorCurrentLimit(Current) where the Current is under 40A");

      }

    }
    if (isMotor(getDCMotor(), DCMotor.getNEO(1)))
    {
      if (config.getStatorStallCurrentLimit().isEmpty())
      {
        throw new SmartMotorControllerConfigurationException("Stator current limit is not defined for NEO!",
                                                             "Safety check failed.",
                                                             "withStatorCurrentLimit(Current)");
      } else if (config.getStatorStallCurrentLimit().getAsInt() > 60)
      {
        throw new SmartMotorControllerConfigurationException("Stator current limit is too high for NEO!",
                                                             "Safety check failed.",
                                                             "withStatorCurrentLimit(Current) where the Current is under 60A");

      }

    }
  }

  /**
   * Iterate the closed loop controller. Feedforward are only applied with profiled pid controllers.
   */
  public void iterateClosedLoopController()
  {
    double pidOutputVoltage = 0;
    double feedforward      = 0.0;
    synchronizeRelativeEncoder();

    if (setpointPosition.isPresent())
    {
      if (config.getMechanismLowerLimit().isPresent())
      {
        if (setpointPosition.get().lt(config.getMechanismLowerLimit().get()))
        {
          DriverStation.reportWarning("[WARNING] Setpoint is lower than Mechanism " +
                                      (config.getTelemetryName().isPresent() ? config.getTelemetryName().get()
                                                                             : "Unnamed smart motor") +
                                      " lower limit, changing setpoint to lower limit.", false);
          setpointPosition = config.getMechanismLowerLimit();
        }
      }
      if (config.getMechanismUpperLimit().isPresent())
      {
        if (setpointPosition.get().gt(config.getMechanismUpperLimit().get()))
        {
          DriverStation.reportWarning("[WARNING] Setpoint is higher than Mechanism " +
                                      (config.getTelemetryName().isPresent() ? config.getTelemetryName().get()
                                                                             : "Unnamed smart motor") +
                                      " upper limit, changing setpoint to upper limit.", false);
          setpointPosition = config.getMechanismUpperLimit();
        }
      }
    }

    if (pidController.isPresent() && setpointPosition.isPresent())
    {
      if (config.getArmFeedforward().isPresent())
      {
        pidOutputVoltage = pidController.get().calculate(getMechanismPosition().in(Rotations),
                                                         setpointPosition.get().in(Rotations));
        feedforward = config.getArmFeedforward().get().calculateWithVelocities(getMechanismPosition().in(Rotations),
                                                                               getMechanismVelocity().in(
                                                                                   RotationsPerSecond),
                                                                               pidController.get()
                                                                                            .getSetpoint().velocity);
      } else if (config.getElevatorFeedforward().isPresent())
      {
        pidOutputVoltage = pidController.get().calculate(getMeasurementPosition().in(Meters),
                                                         config.convertFromMechanism(setpointPosition.get())
                                                               .in(Meters));
        feedforward = config.getElevatorFeedforward().get().calculateWithVelocities(getMeasurementVelocity().in(
            MetersPerSecond), pidController.get().getSetpoint().velocity);

      } else if (config.getSimpleFeedforward().isPresent())
      {
        feedforward = config.getSimpleFeedforward().get().calculateWithVelocities(getMechanismVelocity().in(
            RotationsPerSecond), pidController.get().getSetpoint().velocity);

      }

    } else if (simplePidController.isPresent())
    {
      if (setpointPosition.isPresent())
      {
        pidOutputVoltage = simplePidController.get().calculate(setpointPosition.get().in(Rotations),
                                                               setpointPosition.get().in(Rotations));
      } else if (setpointVelocity.isPresent())
      {
        pidOutputVoltage = simplePidController.get().calculate(setpointVelocity.get().in(RotationsPerSecond));
      }
    }
    if (config.getMechanismUpperLimit().isPresent())
    {
      if (getMechanismPosition().gt(config.getMechanismUpperLimit().get()) && (pidOutputVoltage + feedforward) > 0)
      {
        pidOutputVoltage = feedforward = 0;
      }
    }
    if (config.getMechanismLowerLimit().isPresent())
    {
      if (getMechanismPosition().lt(config.getMechanismLowerLimit().get()) && (pidOutputVoltage + feedforward) < 0)
      {
        pidOutputVoltage = feedforward = 0;
      }
    }
    if (config.getTemperatureCutoff().isPresent())
    {
      if (getTemperature().gte(config.getTemperatureCutoff().get()))
      {
        pidOutputVoltage = feedforward = 0;
      }
    }
//    telemetry.pidOutputVoltage = pidOutputVoltage;
//    telemetry.feedforwardVoltage = feedforward;
//    telemetry.outputVoltage = pidOutputVoltage + feedforward;
    double outputVoltage = pidOutputVoltage + feedforward;
    if (config.getClosedLoopControllerMaximumVoltage().isPresent())
    {
      double maximumVoltage = config.getClosedLoopControllerMaximumVoltage().get().in(Volts);
      outputVoltage = MathUtil.clamp(outputVoltage, -maximumVoltage, maximumVoltage);
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
   *
   * @param mechanismVelocity Mechanism velocity to apply to the simulated motor controller.
   */
  public abstract void simIterate(AngularVelocity mechanismVelocity);

  /**
   * Simulation iteration using existing mechanism velocity.
   */
  public void simIterate()
  {
    if (RobotBase.isSimulation() && setpointVelocity.isPresent())
    {
      simIterate(setpointVelocity.get());
    }
  }

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
    if (config.getTelemetryName().isEmpty())
    {
      throw new SmartMotorControllerConfigurationException("Telemetry is undefined",
                                                           "Cannot create SysIdRoutine",
                                                           "withTelemetry(String,TelemetryVerbosity)");
    }
    if (config.getMechanismCircumference().isPresent())
    {
      sysIdRoutine = new SysIdRoutine(new Config(stepVoltage, maxVoltage, testDuration),
                                      new SysIdRoutine.Mechanism(
                                          this::setVoltage,
                                          log -> {
                                            log.motor(config.getTelemetryName().get())
                                               .voltage(
                                                   getVoltage())
                                               .linearVelocity(getMeasurementVelocity())
                                               .linearPosition(getMeasurementPosition());
                                          },
                                          config.getSubsystem()));
    } else
    {
      sysIdRoutine = new SysIdRoutine(new Config(stepVoltage, maxVoltage, testDuration),
                                      new SysIdRoutine.Mechanism(
                                          this::setVoltage,
                                          log -> {
                                            log.motor(config.getTelemetryName().get())
                                               .voltage(
                                                   getVoltage())
                                               .angularPosition(getMechanismPosition())
                                               .angularVelocity(getMechanismVelocity());
                                          },
                                          config.getSubsystem()));
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
    if (parentTable.isEmpty())
    {
      parentTable = Optional.of(telemetry);
      if (config.getTelemetryName().isPresent())
      {
        telemetryTable = Optional.of(telemetry.getSubTable(config.getTelemetryName().get()));
        tuningTable = Optional.of(tuning.getSubTable(config.getTelemetryName().get()));
        if (config.getSmartControllerTelemetryConfig().isPresent())
        {
          this.telemetry.setupTelemetry(this, telemetry, tuning, config.getSmartControllerTelemetryConfig().get());
        } else
        {
          this.telemetry.setupTelemetry(this, telemetry,
                                        tuning,
                                        new SmartMotorControllerTelemetryConfig().withTelemetryVerbosity(config.getVerbosity()
                                                                                                               .orElse(
                                                                                                                   TelemetryVerbosity.HIGH)));
        }
        updateTelemetry();
      }
    }
  }

  /**
   * Update the telemetry under the motor name under the given {@link NetworkTable}
   */
  public void updateTelemetry()
  {
    if (telemetryTable.isPresent() && config.getVerbosity().isPresent())
    {
//      telemetry.refresh(this);
      telemetry.publish(this);
      if (DriverStation.isTestEnabled())
      {
        telemetry.applyTuningValues(this);
      }
      // if(tuningTable.isPresent())
      //   telemetry.applyChanges(this);

    }
    // TODO: Update PID, Feedforward, current limits, soft limits, ramp rate, motor inversion, encoder inversion
  }

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
}
