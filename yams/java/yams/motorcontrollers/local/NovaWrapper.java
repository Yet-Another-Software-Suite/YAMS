package yams.motorcontrollers.local;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import com.thethriftybot.ThriftyNova.EncoderType;
import com.thethriftybot.ThriftyNova.ExternalEncoder;
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
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.List;
import java.util.Optional;
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
 * Nova wrapper for {@link SmartMotorController}
 */
public class NovaWrapper extends SmartMotorController
{

  /**
   * Thrifty Nova controller.
   */
  private final ThriftyNova          m_nova;
  /**
   * Motor characteristics controlled by the {@link ThriftyNova}.
   */
  private final DCMotor              m_motor;
  /**
   * Sim for ThriftyNova's.
   */
  private       Optional<DCMotorSim> m_dcMotorSim = Optional.empty();
  /**
   * Gearing for the {@link ThriftyNova}.
   */
  private       MechanismGearing     m_gearing;

  /**
   * Construct the Nova Wrapper for the generic {@link SmartMotorController}.
   *
   * @param controller {@link ThriftyNova} to use.
   * @param motor      {@link DCMotor} connected to the {@link ThriftyNova}.
   * @param config     {@link SmartMotorControllerConfig} to apply to the {@link ThriftyNova}.
   */
  public NovaWrapper(ThriftyNova controller, DCMotor motor, SmartMotorControllerConfig config)
  {
    m_nova = controller;
    this.m_motor = motor;
    this.m_config = config;
    setupSimulation();
    applyConfig(config);
  }

  @Override
  public void setupSimulation()
  {
    if (RobotBase.isSimulation())
    {
      var setupRan = m_dcMotorSim.isPresent();
      if (!setupRan)
      {
        m_dcMotorSim = Optional.of(new DCMotorSim(LinearSystemId.createDCMotorSystem(m_motor,
                                                                                     m_config.getMOI(),
                                                                                     m_config.getGearing()
                                                                                             .getRotorToMechanismRatio()),
                                                  m_motor,
                                                  1.0 / 1024.0, 0));

        setSimSupplier(new DCMotorSimSupplier(m_dcMotorSim.get(), this));
      }

      m_config.getStartingPosition().ifPresent(mechPos -> {
        m_dcMotorSim.get().setAngle(mechPos.in(Radians));
      });
    }
  }

  @Override
  public void seedRelativeEncoder()
  {
    DriverStation.reportWarning("[WARNING] NovaWrapper.seedRelativeEncoder() is not supported", false);
  }

  @Override
  public void synchronizeRelativeEncoder()
  {
//    if (!RobotBase.isSimulation())
//    {
//      DriverStation.reportWarning(
//          "[WARNING] NovaWrapper.synchronizeRelativeEncoder() is not supported on ThriftyNova's.",
//          false);
//    }
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
//      m_dcMotorSim.ifPresent(sim -> {
//        sim.setAngularVelocity(m_simSupplier.get().getMechanismVelocity().in(RadiansPerSecond));
//        sim.update(config.getClosedLoopControlPeriod().in(Seconds));
//      });
    }
  }

  @Override
  public void setIdleMode(MotorMode mode)
  {
    m_nova.setBrakeMode(mode == MotorMode.BRAKE);
  }

  @Override
  public void setEncoderVelocity(AngularVelocity velocity)
  {
//    m_sim.ifPresent(dcMotorSim -> dcMotorSim.setAngularVelocity(velocity.in(RadiansPerSecond)));
    DriverStation.reportWarning("[WARNING] NovaWrapper.setEncoderVelocity() is not supported on ThriftyNova's.", false);
  }

  @Override
  public void setEncoderVelocity(LinearVelocity velocity)
  {
    DriverStation.reportWarning("[WARNING] NovaWrapper.setEncoderVelocity() is not supported on ThriftyNova's.", false);
  }

  @Override
  public void setEncoderPosition(Angle angle)
  {
    m_nova.setEncoderPosition(angle.in(Rotations));
    m_simSupplier.ifPresent(simSupplier -> simSupplier.setMechanismPosition(angle));
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

    this.m_config = config;
    m_config.resetValidationCheck();
    m_gearing = config.getGearing();
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
          throw new IllegalArgumentException("[ERROR] closed loop controller must not be empty");
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

    if (config.getFeedbackSynchronizationThreshold().isPresent())
    {
      throw new SmartMotorControllerConfigurationException(
          "Feedback synchronization threshold is not supported on ThriftyNovas",
          "Cannot configure ThriftyNova with a feedback synchronization threshold.",
          ".withFeedbackSynchronizationThreshold");
    }

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
      m_closedLoopControllerThread.setName(getName());
    }
    if (config.getMotorControllerMode() == ControlMode.CLOSED_LOOP)
    {
      startClosedLoopController();
    } else
    {
      m_closedLoopControllerThread.stop();
    }

    // Ramp rates
    m_nova.setRampUp(config.getClosedLoopRampRate().in(Seconds));
    m_nova.setRampDown(config.getClosedLoopRampRate().in(Seconds));
    if (config.getOpenLoopRampRate().gt(Seconds.of(0)))
    {
      throw new IllegalArgumentException(
          "[Error] ThriftyNova does not support separate closed loop and open loop ramp rates, using the SmartMotorControllerConfig.withClosedLoopRampRate() as both.");
    }

    // Inversions
    m_nova.setInverted(config.getMotorInverted());
    if (config.getEncoderInverted())
    {
      throw new IllegalArgumentException("[ERROR] ThriftyNova does not support encoder inversions.");
    }

    // Current limits
    if (config.getSupplyStallCurrentLimit().isPresent())
    {
      m_nova.setMaxCurrent(CurrentType.SUPPLY, config.getSupplyStallCurrentLimit().getAsInt());
    }
    if (config.getStatorStallCurrentLimit().isPresent())
    {
      m_nova.setMaxCurrent(CurrentType.STATOR, config.getStatorStallCurrentLimit().getAsInt());
    }

    // Voltage Compensation
    if (config.getVoltageCompensation().isPresent())
    {
      m_nova.setVoltageCompensation(config.getVoltageCompensation().get().in(Volts));
    }

    // Setup idle mode.
    if (config.getIdleMode().isPresent())
    {
      m_nova.setBrakeMode(config.getIdleMode().get() == MotorMode.BRAKE);
    }

    // Starting Position
    if (config.getStartingPosition().isPresent())
    {
      setEncoderPosition(config.getStartingPosition().get());
    }

    // External Encoder
    boolean useExt = config.getUseExternalFeedback();
    if (config.getExternalEncoder().isPresent() && useExt)
    {
      Object externalEncoder = config.getExternalEncoder().get();
      if (externalEncoder instanceof EncoderType)
      {
        if (externalEncoder == EncoderType.QUAD)
        {
          m_nova.useEncoderType(EncoderType.QUAD);
        } else if (externalEncoder == EncoderType.ABS)
        {
          m_nova.useEncoderType(EncoderType.ABS);
        }
        if (config.getStartingPosition().isEmpty())
        {
          if (externalEncoder == EncoderType.ABS)
          {
            m_nova.setEncoderPosition(m_nova.getPositionAbs());
          }
        }
      } else if (externalEncoder instanceof ExternalEncoder)
      {
        m_nova.setExternalEncoder((ExternalEncoder) externalEncoder);
      } else
      {
        throw new IllegalArgumentException(
            "[ERROR] Unsupported external encoder: " + externalEncoder.getClass().getSimpleName() +
            ".\n\tPlease use an `EncoderType` instead.");
      }

      if (config.getZeroOffset().isPresent())
      {
        throw new SmartMotorControllerConfigurationException("Zero offset is unavailable for ThriftyNova",
                                                             "Zero offset could not be applied",
                                                             ".withZeroOffset");
//        m_nova.setAbsOffset(config.getZeroOffset().get().in(Rotations));
      }
      if (config.getExternalEncoderGearing().getRotorToMechanismRatio() != 1.0)
      {
        // Do nothing, applied later.
      }

    } else
    {
      if (config.getZeroOffset().isPresent())
      {
        throw new SmartMotorControllerConfigurationException("Zero offset is only available for external encoders",
                                                             "Zero offset could not be applied",
                                                             ".withZeroOffset");
      }

      if (config.getExternalEncoderGearing().getRotorToMechanismRatio() != 1.0)
      {
        throw new SmartMotorControllerConfigurationException(
            "External encoder gearing is not supported when there is no external encoder",
            "External encoder gearing could not be set",
            ".withExternalEncoderGearing");
      }
    }

    if (config.getExternalEncoderInverted())
    {
      throw new SmartMotorControllerConfigurationException(
          "External encoder cannot be inverted because no external encoder exists",
          "External encoder could not be inverted",
          ".withExternalEncoderInverted");
    }

    if (config.getFollowers().isPresent())
    {
      for (Pair<Object, Boolean> follower : config.getFollowers().get())
      {
        if (follower.getFirst() instanceof ThriftyNova)
        {
          ((ThriftyNova) follower.getFirst()).follow(m_nova.getID());
          ((ThriftyNova) follower.getFirst()).setInverted(follower.getSecond());
        } else
        {
          throw new IllegalArgumentException(
              "[ERROR] Unknown follower type: " + follower.getFirst().getClass().getSimpleName());
        }
      }
    }

    if (config.getMaxDiscontinuityPoint().isPresent() &&
        !(m_expoPidController.isPresent() || m_pidController.isPresent() || m_simplePidController.isPresent()))
    {
      throw new IllegalArgumentException(
          "[ERROR] ThriftyNova does not support discontinuity points, or we have not implemented this.");
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
    return true;
  }

  @Override
  public double getDutyCycle()
  {
    return m_simSupplier.isPresent() ? m_simSupplier.get().getMechanismStatorVoltage().in(Volts) /
                                       m_simSupplier.get().getMechanismSupplyVoltage().in(Volts) : m_nova.get();
  }

  @Override
  public void setDutyCycle(double dutyCycle)
  {
    m_simSupplier.ifPresent(simSupplier -> simSupplier.setMechanismStatorDutyCycle(dutyCycle));
    m_nova.set(dutyCycle);
  }

  @Override
  public Optional<Current> getSupplyCurrent()
  {
    if (m_simSupplier.isPresent())
    {
      return Optional.of(Amps.of(RoboRioSim.getVInCurrent()));
    }
    return Optional.of(Amps.of(m_nova.getSupplyCurrent()));
  }

  @Override
  public Current getStatorCurrent()
  {
    return m_simSupplier.isPresent() ? m_simSupplier.get().getCurrentDraw() : Amps.of(m_nova.getStatorCurrent());
  }

  @Override
  public Voltage getVoltage()
  {
    return m_simSupplier.isPresent() ? m_simSupplier.get().getMechanismStatorVoltage() : Volts.of(m_nova.getVoltage());
  }

  @Override
  public void setVoltage(Voltage voltage)
  {
    m_simSupplier.ifPresent(simSupplier -> simSupplier.setMechanismStatorVoltage(voltage));
    m_nova.setVoltage(voltage);
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
    if (m_simSupplier.isPresent())
    {
      return m_simSupplier.get().getMechanismVelocity();
    }
    if (m_config.getUseExternalFeedback() && m_config.getExternalEncoder().isPresent())
    {
      Object externalEncoder = m_config.getExternalEncoder().get();
      if (externalEncoder == EncoderType.ABS)
      {
        // Do nothing since attached absolute encoders do not give their velocity.
        // There should be an alert thrown here; but Alerts are not thread-safe.
      } else if (externalEncoder == EncoderType.QUAD)
      {
        // There should be an alert thrown here; but Alerts are not thread-safe.
        return RotationsPerSecond.of(
            m_nova.getVelocityQuad() * m_config.getExternalEncoderGearing().getRotorToMechanismRatio());
      }
    }
    return getRotorVelocity().times(m_gearing.getRotorToMechanismRatio());
  }

  @Override
  public Angle getMechanismPosition()
  {
    if (m_simSupplier.isPresent())
    {
      return m_simSupplier.get().getMechanismPosition();
    }
    if (m_config.getUseExternalFeedback() && m_config.getExternalEncoder().isPresent())
    {
      Object externalEncoder = m_config.getExternalEncoder().get();
      if (externalEncoder == EncoderType.ABS)
      {
        return Rotations.of(m_nova.getPositionAbs() * m_config.getExternalEncoderGearing().getRotorToMechanismRatio());
      } else if (externalEncoder == EncoderType.QUAD)
      {
        return Rotations.of(m_nova.getPositionQuad() * m_config.getExternalEncoderGearing().getRotorToMechanismRatio());
      }
    }
    return getRotorPosition().times(m_gearing.getRotorToMechanismRatio());
  }

  @Override
  public AngularVelocity getRotorVelocity()
  {
    if (RobotBase.isSimulation() && m_simSupplier.isPresent())
    {
      return m_simSupplier.get().getRotorVelocity();
    }
    return RotationsPerSecond.of(m_nova.getVelocity());
  }

  @Override
  public Angle getRotorPosition()
  {
    if (RobotBase.isSimulation() && m_simSupplier.isPresent())
    {
      return m_simSupplier.get().getRotorPosition();
    }
    return Rotations.of(m_nova.getPosition());
  }

  @Override
  public void setMotorInverted(boolean inverted)
  {
    m_config.withMotorInverted(inverted);
    m_nova.setInverted(inverted);
  }

  @Override
  public void setEncoderInverted(boolean inverted)
  {
    m_config.withEncoderInverted(inverted);
    m_nova.setInverted(inverted);
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
    m_expoPidController.ifPresent(expoPidController -> {
      expoPidController.setP(kP);
    });
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
    m_expoPidController.ifPresent(expoPidController -> {
      expoPidController.setI(kI);
    });

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
    m_expoPidController.ifPresent(expoPidController -> {
      expoPidController.setD(kD);
    });
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
    m_nova.setMaxCurrent(CurrentType.STATOR, currentLimit.in(Amps));
  }

  @Override
  public void setSupplyCurrentLimit(Current currentLimit)
  {
    m_config.withSupplyCurrentLimit(currentLimit);
    m_nova.setMaxCurrent(CurrentType.SUPPLY, currentLimit.in(Amps));
  }

  @Override
  public void setClosedLoopRampRate(Time rampRate)
  {
    m_config.withClosedLoopRampRate(rampRate);
    m_nova.setRampUp(rampRate.in(Seconds));
    m_nova.setRampDown(rampRate.in(Seconds));
  }

  @Deprecated
  /// Unsupported
  public void setOpenLoopRampRate(Time rampRate)
  {
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
    return Celsius.of(m_nova.getTemperature());
  }

  @Override
  public SmartMotorControllerConfig getConfig()
  {
    return m_config;
  }

  @Override
  public Object getMotorController()
  {
    return m_nova;
  }

  @Override
  public Object getMotorControllerConfig()
  {
    DriverStation.reportWarning(
        "[WARNING] Thrifty Nova's have no configuration class, returning the ThriftyNova Object.",
        true);
    return m_nova;
  }

  @Override
  public Pair<Optional<List<BooleanTelemetryField>>, Optional<List<DoubleTelemetryField>>> getUnsupportedTelemetryFields()
  {
    return Pair.of(Optional.empty(), Optional.empty());
  }
}
