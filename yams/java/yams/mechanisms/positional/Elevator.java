package yams.mechanisms.positional;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.Optional;
import java.util.function.Supplier;
import yams.exceptions.ElevatorConfigurationException;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.MechanismPositionConfig.Plane;
import yams.motorcontrollers.SimSupplier;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;

/**
 * Elevator mechanism.
 */
public class Elevator extends SmartPositionalMechanism
{

  /**
   * Config class for the elevator.
   */
  private final ElevatorConfig        m_config;
  /**
   * Simulation for the elevator
   */
  private       Optional<ElevatorSim> m_sim = Optional.empty();

  /**
   * Construct the {@link Elevator} class for easy manipulation of an elevator.
   *
   * @param config {@link ElevatorConfig} to set.
   */
  public Elevator(ElevatorConfig config)
  {
    m_config = config;
    m_smc = config.getMotor();
    DCMotor                    dcMotor   = m_smc.getDCMotor();
    MechanismGearing           gearing   = m_smc.getConfig().getGearing();
    SmartMotorControllerConfig smcConfig = m_smc.getConfig();
    m_subsystem = config.getMotor().getConfig().getSubsystem();
    if (config.getTelemetryName().isPresent())
    {
      // TODO: Add telemetry units to config.
      m_telemetry.setupTelemetry(getName(),
                                 m_smc);
    }
    config.applyConfig();

    if (RobotBase.isSimulation())
    {
      SmartMotorController motor = config.getMotor();
      motor.setupSimulation();
      if (config.getCarriageMass().isEmpty())
      {
        throw new ElevatorConfigurationException("Mass is not configured!",
                                                 "Cannot create simulator",
                                                 "withMass(Mass)");
      }
      if (config.getMinimumHeight().isEmpty())
      {
        throw new ElevatorConfigurationException("Minimum height is not configured!",
                                                 "Cannot create simulator",
                                                 "withHardLimits(Distance,Distance)");
      }
      if (config.getMaximumHeight().isEmpty())
      {
        throw new ElevatorConfigurationException("Maximum height is not configured!",
                                                 "Cannot create simulator",
                                                 "withHardLimits(Distance,Distance)");
      }
      if (config.getStartingHeight().isEmpty())
      {
        throw new ElevatorConfigurationException("Starting height is not configured!",
                                                 "Cannot create simulator",
                                                 "withStartingHeight(Distance)");
      }
      m_sim = Optional.of(new ElevatorSim(motor.getDCMotor(),
                                          motor.getConfig().getGearing().getMechanismToRotorRatio(),
                                          config.getCarriageMass().get().in(Kilograms),
                                          config.getDrumRadius().in(Meters),
                                          config.getMinimumHeight().get().in(Meters),
                                          config.getMaximumHeight().get().in(Meters),
                                          true,
                                          config.getStartingHeight().get().in(Meters),
                                          0.01 / 4096, 0.01 / 4096));
      m_smc.setSimSupplier(new SimSupplier()
      {
        final Supplier<Double> pos = m_sim.get()::getPositionMeters;
        final Supplier<Double> mps = m_sim.get()::getVelocityMetersPerSecond;
        boolean inputFed   = false;
        boolean updatedSim = false;

        @Override
        public void updateSimState()
        {
          if (!isInputFed())
          {
            m_sim.get().setInput(m_smc.getDutyCycle() * RoboRioSim.getVInVoltage());
          }
          if (!updatedSim)
          {
            starveInput();
            m_sim.get().update(smcConfig.getClosedLoopControlPeriod().orElse(Milliseconds.of(20)).in(Seconds));
            feedUpdateSim();
          }
        }

        @Override
        public boolean getUpdatedSim()
        {
          return updatedSim;
        }

        @Override
        public void feedUpdateSim()
        {
          updatedSim = true;
        }

        @Override
        public void starveUpdateSim()
        {
          updatedSim = false;
        }

        @Override
        public boolean isInputFed()
        {
          return inputFed;
        }

        @Override
        public void feedInput()
        {
          inputFed = true;
        }

        @Override
        public void starveInput()
        {
          inputFed = false;
        }

        @Override
        public void setMechanismStatorDutyCycle(double dutyCycle)
        {
          feedInput();
          m_sim.get().setInputVoltage(dutyCycle * getMechanismSupplyVoltage().in(Volts));
        }

        @Override
        public Voltage getMechanismSupplyVoltage()
        {
          return Volts.of(RoboRioSim.getVInVoltage());
        }

        @Override
        public Voltage getMechanismStatorVoltage()
        {
          return Volts.of(dcMotor.getVoltage(dcMotor.getTorque(m_sim.get().getCurrentDrawAmps()),
                                             getMechanismVelocity().in(RadiansPerSecond)));
        }

        @Override
        public void setMechanismStatorVoltage(Voltage volts)
        {
          feedInput();
          m_sim.get().setInputVoltage(volts.in(Volts));
        }

        @Override
        public Angle getMechanismPosition()
        {
          return smcConfig.convertToMechanism(Meters.of(pos.get()));
        }

        @Override
        public void setMechanismPosition(Angle position)
        {
          m_sim.get().setState(smcConfig.convertFromMechanism(position).in(Meters), mps.get());

        }

        @Override
        public Angle getRotorPosition()
        {
          return getMechanismPosition().times(gearing.getMechanismToRotorRatio());
        }

        @Override
        public AngularVelocity getMechanismVelocity()
        {
          return smcConfig.convertToMechanism(MetersPerSecond.of(mps.get()));

        }

        @Override
        public void setMechanismVelocity(AngularVelocity velocity)
        {
          m_sim.get().setState(pos.get(), smcConfig.convertFromMechanism(velocity).in(MetersPerSecond));
        }

        @Override
        public AngularVelocity getRotorVelocity()
        {
          return getMechanismVelocity().times(gearing.getMechanismToRotorRatio());
        }

        @Override
        public Current getCurrentDraw()
        {
          return Amps.of(m_sim.get().getCurrentDrawAmps());
        }
      });
      m_mechanismWindow = new Mechanism2d(config.getMaximumHeight().get().in(Meters) * 2,
                                          config.getMaximumHeight().get().in(Meters) * 2);

      m_mechanismRoot = m_mechanismWindow.getRoot(getName() + "Root",
                                                  config.getMaximumHeight().get().in(Meters) -
                                                  config.getMechanismPositionConfig().getRelativePosition()
                                                        .orElse(new Translation3d()).getX(),
                                                  config.getMechanismPositionConfig().getRelativePosition()
                                                        .orElse(new Translation3d()).getZ());

      if (m_smc.getConfig().getMechanismLowerLimit().isPresent())
      {
        m_mechanismWindow.getRoot(
                             "MinSoft",
                             config.getMaximumHeight().get().in(Meters) +
                             config.getMechanismPositionConfig().getRelativePosition().orElse(new Translation3d()).getX() -
                             Inches.of(6).in(Meters),
                             config.getMechanismPositionConfig().getRelativePosition().orElse(new Translation3d()).getZ())
                         .append(new MechanismLigament2d(
                             "Limit",
                             m_smc.getConfig().convertFromMechanism(m_smc.getConfig().getMechanismLowerLimit().get())
                                  .in(Meters),
                             config.getAngle().in(Degrees),
                             3,
                             new Color8Bit(Color.kYellow)
                         ));
      }
      if (m_smc.getConfig().getMechanismUpperLimit().isPresent())
      {
        m_mechanismWindow.getRoot(
                             "MaxSoft",
                             config.getMaximumHeight().get().in(Meters) +
                             config.getMechanismPositionConfig().getRelativePosition().orElse(new Translation3d()).getX() -
                             Inches.of(6).in(Meters),
                             config.getMechanismPositionConfig().getRelativePosition().orElse(new Translation3d()).getZ())
                         .append(new MechanismLigament2d(
                             "Limit",
                             m_smc.getConfig().convertFromMechanism(m_smc.getConfig().getMechanismUpperLimit().get())
                                  .in(Meters),
                             config.getAngle().in(Degrees),
                             3,
                             new Color8Bit(Color.kHotPink)
                         ));
      }
      m_mechanismWindow.getRoot(
                           "MinHard",
                           config.getMaximumHeight().get().in(Meters) + config.getMechanismPositionConfig().getRelativePosition().orElse(
                               new Translation3d()).getX() - Inches.of(8).in(Meters),
                           config.getMechanismPositionConfig().getRelativePosition().orElse(new Translation3d()).getZ())
                       .append(new MechanismLigament2d(
                           "Limit",
                           config.getMinimumHeight().get().in(Meters),
                           config.getAngle().in(Degrees),
                           3,
                           new Color8Bit(Color.kRed)
                       ));
      m_mechanismWindow.getRoot(
                           "MaxHard",
                           config.getMaximumHeight().get().in(Meters) + config.getMechanismPositionConfig().getRelativePosition().orElse(
                               new Translation3d()).getX() - Inches.of(8).in(Meters),
                           config.getMechanismPositionConfig().getRelativePosition().orElse(new Translation3d()).getZ())
                       .append(new MechanismLigament2d(
                           "Limit",
                           config.getMaximumHeight().get().in(Meters),
                           config.getAngle().in(Degrees),
                           3,
                           new Color8Bit(Color.kLimeGreen)
                       ));

      m_mechanismLigament = m_mechanismRoot.append(new MechanismLigament2d(getName(),
                                                                           config.getStartingHeight().get().in(Meters),
                                                                           config.getAngle().in(Degrees),
                                                                           6,
                                                                           config.getSimColor()));
      SmartDashboard.putData(getName() + "/mechanism", m_mechanismWindow);
    }
  }


  @Override
  public void updateTelemetry()
  {
//    m_telemetry.updatePosition(getHeight());
//    m_motor.getMechanismPositionSetpoint().ifPresent(m_setpoint -> m_telemetry.updateSetpoint(m_setpoint));
    m_smc.updateTelemetry();
  }

  @Override
  public void simIterate()
  {
    if (m_sim.isPresent() && m_smc.getSimSupplier().isPresent())
    {
      m_smc.getSimSupplier().get().updateSimState();
      m_smc.simIterate();
      m_smc.getSimSupplier().get().starveUpdateSim();
      // It is impossible for an elevator to go bellow the minimum height, it would break...
      if (m_config.getMinimumHeight().isPresent() && getHeight().lt(m_config.getMinimumHeight().get()))
      {
//        m_motor.simIterate(RotationsPerSecond.of(0));
//        m_motor.setEncoderPosition(m_config.getMinimumHeight().get());
      } else
      {
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_sim.get().getCurrentDrawAmps()));
      }
      visualizationUpdate();
    }
  }

  /**
   * Updates the length of the mechanism ligament to match the current height of the elevator in meters.
   */
  @Override
  public void visualizationUpdate()
  {
    m_mechanismLigament.setLength(getHeight().in(Meters));
  }

  /**
   * Get the relative position of the mechanism, taking into account the relative position defined in the
   * {@link MechanismPositionConfig}.
   *
   * @return The relative position of the mechanism as a {@link Translation3d}.
   */
  @Override
  public Translation3d getRelativeMechanismPosition()
  {
    Plane movementPlane = m_config.getMechanismPositionConfig().getMovementPlane();
    Translation3d mechanismTranslation = new Translation3d(m_mechanismLigament.getLength(),
                                                           new Rotation3d(
                                                               Plane.YZ == movementPlane
                                                               ? m_mechanismLigament.getAngle()
                                                               : 0,
                                                               Plane.XZ == movementPlane
                                                               ? m_mechanismLigament.getAngle()
                                                               : 0, 0));
    if (m_config.getMechanismPositionConfig().getRelativePosition().isPresent())
    {
      return m_config.getMechanismPositionConfig().getRelativePosition().get()
                     .plus(mechanismTranslation);
    }
    return mechanismTranslation;
  }

  @Override
  public String getName()
  {
    return m_config.getTelemetryName().orElse("Elevator");
  }

  /**
   * Set the height of the elevator.
   *
   * @param height Height of the elevator to reach.
   * @return {@link Command} that  sets the elevator height, stops immediately.
   */
  public Command setHeight(Distance height)
  {
    return Commands.run(() -> m_smc.setPosition(height), m_subsystem).withName(m_subsystem.getName() + " SetHeight");
  }

  /**
   * Set the height of the elevator.
   *
   * @param height Height of the elevator to reach.
   * @return {@link Command} that  sets the elevator height, stops immediately.
   */
  public Command setHeight(Supplier<Distance> height)
  {
    return Commands.run(() -> m_smc.setPosition(height.get()), m_subsystem).withName(
        m_subsystem.getName() + " SetHeight Supplier");
  }


  /**
   * Get the Height of the Elevator.
   *
   * @return {@link Distance} of the Elevator.
   */
  public Distance getHeight()
  {
    return m_smc.getMeasurementPosition();
  }

  /**
   * Get the linear velocity of the Elevator.
   *
   * @return {@link LinearVelocity} of the elevator.
   */
  public LinearVelocity getVelocity()
  {
    return m_smc.getMeasurementVelocity();
  }


  /**
   * Elevator is near a height.
   *
   * @param height {@link Distance} to be near.
   * @param within {@link Distance} within.
   * @return Trigger on when the elevator is near another height.
   */
  public Trigger isNear(Distance height, Distance within)
  {
    return new Trigger(() -> getHeight().isNear(height, within));
  }

  @Override
  public Trigger max()
  {
    if (m_smc.getConfig().getMechanismUpperLimit().isPresent())
    {
      return new Trigger(gte(m_smc.getConfig()
                                  .convertFromMechanism(m_smc.getConfig().getMechanismUpperLimit().get())));
    }
    if (m_config.getMaximumHeight().isPresent())
    {
      return gte(m_config.getMaximumHeight().get());
    }
    throw new ElevatorConfigurationException("Maximum height is not configured!",
                                             "Cannot create max trigger.",
                                             "withHardLimits(Distance,Distance)");
  }

  @Override
  public Trigger min()
  {
    if (m_smc.getConfig().getMechanismLowerLimit().isPresent())
    {
      return new Trigger(gte(m_smc.getConfig()
                                  .convertFromMechanism(m_smc.getConfig().getMechanismLowerLimit().get())));
    }
    if (m_config.getMinimumHeight().isPresent())
    {
      return gte(m_config.getMinimumHeight().get());
    }
    throw new ElevatorConfigurationException("Minimum height is not configured!",
                                             "Cannot create min trigger.",
                                             "withHardLimits(Distance,Distance)");
  }


  /**
   * Between two heights.
   *
   * @param start Start height.
   * @param end   End height.
   * @return {@link Trigger}
   */
  public Trigger between(Distance start, Distance end)
  {
    return gte(start).and(lte(end));
  }

  /**
   * Less than or equal to height
   *
   * @param height {@link Distance} to check against
   * @return {@link Trigger}
   */
  public Trigger lte(Distance height)
  {
    return new Trigger(() -> getHeight().lte(height));
  }

  /**
   * Greater than or equal to height.
   *
   * @param height Height to check against.
   * @return {@link Trigger} for elevator.
   */
  public Trigger gte(Distance height)
  {
    return new Trigger(() -> getHeight().gte(height));
  }

  @Override
  public Command sysId(Voltage maximumVoltage, Velocity<VoltageUnit> step, Time duration)
  {
    SysIdRoutine               routine     = m_smc.sysId(maximumVoltage, step, duration);
    SmartMotorControllerConfig motorConfig = m_smc.getConfig();
    Distance                   max;
    Distance                   min;
    if (m_smc.getConfig().getMechanismUpperLimit().isPresent())
    {
      max = motorConfig.convertFromMechanism(m_smc.getConfig().getMechanismUpperLimit().get())
                       .minus(Centimeters.of(1));
    } else if (m_config.getMaximumHeight().isPresent())
    {
      max = m_config.getMaximumHeight().get().minus(Centimeters.of(1));
    } else
    {
      throw new ElevatorConfigurationException("Maximum height is not configured!",
                                               "Cannot create SysIdRoutine!",
                                               "withHardLimits(Distance,Distance)");

    }
    if (m_smc.getConfig().getMechanismLowerLimit().isPresent())
    {
      min = motorConfig.convertFromMechanism(m_smc.getConfig().getMechanismLowerLimit().get())
                       .plus(Centimeters.of(10));
    } else if (m_config.getMinimumHeight().isPresent())
    {
      min = m_config.getMinimumHeight().get().plus(Centimeters.of(1));
    } else
    {
      throw new ElevatorConfigurationException("Minimum height is not configured!",
                                               "Cannot create SysIdRoutine!",
                                               "withHardLimits(Distance,Distance)");
    }
    Trigger maxTrigger = gte(max);
    Trigger minTrigger = lte(min);

    Command group = Commands.print("Starting SysId!")
                            .beforeStarting(Commands.runOnce(m_smc::stopClosedLoopController))
                            .andThen(routine.dynamic(Direction.kForward).until(maxTrigger).withTimeout(3))
                            .andThen(routine.dynamic(Direction.kReverse).until(minTrigger))
                            .andThen(routine.quasistatic(Direction.kForward).until(maxTrigger))
                            .andThen(routine.quasistatic(Direction.kReverse).until(minTrigger).withTimeout(3))
                            .finallyDo(m_smc::startClosedLoopController);
    if (m_config.getTelemetryName().isPresent())
    {
      group = group.andThen(Commands.print(getName() + " SysId test done."));
    }
    return group.withName(m_subsystem.getName() + " SysId");
  }

  /**
   * Get the {@link ElevatorConfig} for this {@link Elevator}
   *
   * @return {@link ElevatorConfig} for this {@link Elevator}
   */
  public ElevatorConfig getConfig()
  {
    return m_config;
  }
}
