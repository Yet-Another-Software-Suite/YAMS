package yams.mechanisms.positional;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
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
import yams.exceptions.ArmConfigurationException;
import yams.exceptions.PivotConfigurationException;
import yams.mechanisms.config.PivotConfig;
import yams.motorcontrollers.SmartMotorController;

public class Pivot extends SmartPositionalMechanism
{

  /**
   * Pivot config.
   */
  private final PivotConfig           m_config;
  /**
   * Simulation for the Pivot.
   */
  private       Optional<FlywheelSim> m_sim = Optional.empty();

  /**
   * Construct the Pivot class
   *
   * @param config Pivot configuration.
   */
  public Pivot(PivotConfig config)
  {
    m_config = config;
    m_motor = config.getMotor();
    m_subsystem = m_motor.getConfig().getSubsystem();
    // Seed the relative encoder
    if (m_motor.getConfig().getExternalEncoder().isPresent())
    {
      m_motor.seedRelativeEncoder();
    }
    if (config.getTelemetryName().isPresent())
    {
      NetworkTable table = NetworkTableInstance.getDefault().getTable(config.getNetworkRoot().orElse("Tuning"))
                                               .getSubTable(config.getTelemetryName().get());
      m_telemetry.setupTelemetry(table);
      m_telemetry.units.set("Degrees");
      m_motor.updateTelemetry(table);
    }
    config.applyConfig();

    if (RobotBase.isSimulation())
    {
      SmartMotorController motor = config.getMotor();
      if (config.getLowerHardLimit().isEmpty())
      {
        throw new PivotConfigurationException("Pivot lower hard limit is empty",
                                              "Cannot create simulation.",
                                              "withHardLimit(Angle,Angle)");
      }
      if (config.getUpperHardLimit().isEmpty())
      {
        throw new PivotConfigurationException("Pivot upper hard limit is empty",
                                              "Cannot create simulation.",
                                              "withHardLimit(Angle,Angle)");
      }
      if (config.getStartingAngle().isEmpty())
      {
        throw new PivotConfigurationException("Pivot starting angle is empty",
                                              "Cannot create simulation.",
                                              "withStartingPosition(Angle)");
      }
      m_sim = Optional.of(new FlywheelSim(LinearSystemId.createFlywheelSystem(motor.getDCMotor(),
                                                                              config.getMOI(),
                                                                              motor.getConfig().getGearing()
                                                                                   .getMechanismToRotorRatio()),
                                          motor.getDCMotor(),
                                          1.0 / 4096.0));// Add noise with a std-dev of 1 tick

      Distance pivotLength = Inches.of(36);
      mechanismWindow = new Mechanism2d(pivotLength.in(Meters) * 2,
                                        pivotLength.in(Meters) * 2);
      mechanismRoot = mechanismWindow.getRoot(
          config.getTelemetryName().isPresent() ? config.getTelemetryName().get() + "Root" : "PivotRoot",
          pivotLength.in(Meters), pivotLength.in(Meters));
      mechanismLigament = mechanismRoot.append(new MechanismLigament2d(
          config.getTelemetryName().isPresent() ? config.getTelemetryName().get() : "Pivot",
          pivotLength.in(Meters),
          config.getStartingAngle().get().in(Degrees), 6, config.getSimColor()));
      mechanismRoot.append(new MechanismLigament2d("MaxHard",
                                                   Inch.of(3).in(Meters),
                                                   config.getUpperHardLimit().get()
                                                         .in(Degrees),
                                                   4,
                                                   new Color8Bit(Color.kLimeGreen)));
      mechanismRoot.append(new MechanismLigament2d("MinHard", Inch.of(3).in(Meters),
                                                   config.getLowerHardLimit().get()
                                                         .in(Degrees),
                                                   4, new Color8Bit(Color.kRed)));
      if (motor.getConfig().getMechanismLowerLimit().isPresent() &&
          motor.getConfig().getMechanismUpperLimit().isPresent())
      {
        mechanismRoot.append(new MechanismLigament2d("MaxSoft",
                                                     Inch.of(3).in(Meters),
                                                     motor.getConfig().getMechanismUpperLimit().get()
                                                          .in(Degrees),
                                                     4,
                                                     new Color8Bit(Color.kHotPink)));
        mechanismRoot.append(new MechanismLigament2d("MinSoft", Inch.of(3).in(Meters),
                                                     motor.getConfig().getMechanismLowerLimit().get()
                                                          .in(Degrees),
                                                     4, new Color8Bit(Color.kYellow)));
      }
      SmartDashboard.putData(
          config.getTelemetryName().isPresent() ? config.getTelemetryName().get() + "/mechanism" : "Pivot/mechanism",
          mechanismWindow);
    }
  }

  /**
   * Get the {@link SmartMotorController} Mechanism Position representing the pivot.
   *
   * @return Pivot {@link Angle}
   */
  public Angle getAngle()
  {
    return m_motor.getMechanismPosition();
  }


  /**
   * Between two angles.
   *
   * @param start Start angle.
   * @param end   End angle
   * @return {@link Trigger}
   */
  public Trigger between(Angle start, Angle end)
  {
    return gte(start).and(lte(end));
  }

  /**
   * Less than or equal to angle
   *
   * @param angle {@link Angle} to check against
   * @return {@link Trigger}
   */
  public Trigger lte(Angle angle)
  {
    return new Trigger(() -> getAngle().lte(angle));
  }

  /**
   * Greater than or equal to angle.
   *
   * @param angle Angle to check against.
   * @return {@link Trigger} for Pivot.
   */
  public Trigger gte(Angle angle)
  {
    return new Trigger(() -> getAngle().gte(angle));
  }

  /**
   * Pivot is near an angle.
   *
   * @param angle  {@link Angle} to be near.
   * @param within {@link Angle} within.
   * @return Trigger on when the pivot is near another angle.
   */
  public Trigger isNear(Angle angle, Angle within)
  {
    return new Trigger(() -> getAngle().isNear(angle, within));
  }

  /**
   * Set the pivot to the given angle.
   *
   * @param angle Pivot angle to go to.
   * @return {@link Command} that sets the pivot to the desired angle.
   */
  public Command setAngle(Angle angle)
  {
    return Commands.run(() -> m_motor.setPosition(angle), m_subsystem);
  }

  @Override
  public Trigger max()
  {
    if (m_motor.getConfig().getMechanismUpperLimit().isPresent())
    {
      return new Trigger(gte(m_motor.getConfig().getMechanismUpperLimit().get()));
    }
    if (m_config.getUpperHardLimit().isEmpty())
    {
      return gte(m_config.getUpperHardLimit().get());
    }
    throw new ArmConfigurationException("Pivot upper hard and motor controller soft limit is empty",
                                        "Cannot create max trigger.",
                                        "withHardLimit(Angle,Angle)");
  }

  @Override
  public Trigger min()
  {
    if (m_motor.getConfig().getMechanismLowerLimit().isPresent())
    {
      return new Trigger(gte(m_motor.getConfig().getMechanismLowerLimit().get()));
    }
    if (m_config.getLowerHardLimit().isEmpty())
    {
      return gte(m_config.getLowerHardLimit().get());
    }
    throw new ArmConfigurationException("Pivot lower hard and motor controller soft limit is empty",
                                        "Cannot create min trigger.",
                                        "withHardLimit(Angle,Angle)");
  }

  @Override
  public Command sysId(Voltage maximumVoltage, Velocity<VoltageUnit> step, Time duration)
  {
    SysIdRoutine routine = m_motor.sysId(maximumVoltage, step, duration);
    Angle        max;
    Angle        min;
    if (m_motor.getConfig().getMechanismUpperLimit().isPresent())
    {
      max = m_motor.getConfig().getMechanismUpperLimit().get().minus(Degrees.of(1));
    } else if (m_config.getUpperHardLimit().isPresent())
    {
      max = m_config.getUpperHardLimit().get().minus(Degrees.of(1));
    } else
    {
      throw new PivotConfigurationException("Pivot upper hard and motor controller soft limit is empty",
                                            "Cannot create SysIdRoutine.",
                                            "withHardLimit(Angle,Angle)");
    }
    if (m_motor.getConfig().getMechanismLowerLimit().isPresent())
    {
      min = m_motor.getConfig().getMechanismLowerLimit().get().plus(Degrees.of(1));
    } else if (m_config.getLowerHardLimit().isPresent())
    {
      min = m_config.getLowerHardLimit().get().plus(Degrees.of(1));
    } else
    {
      throw new PivotConfigurationException("Pivot lower hard and motor controller soft limit is empty",
                                            "Cannot create SysIdRoutine.",
                                            "withHardLimit(Angle,Angle)");
    }
    Trigger maxTrigger = gte(max);
    Trigger minTrigger = lte(min);

    Command group = routine.dynamic(Direction.kForward).until(maxTrigger)
                           .andThen(routine.dynamic(Direction.kReverse).until(minTrigger))
                           .andThen(routine.quasistatic(Direction.kForward).until(maxTrigger))
                           .andThen(routine.quasistatic(Direction.kReverse).until(minTrigger));
    if (m_config.getTelemetryName().isPresent())
    {
      group = group.andThen(Commands.print(m_config.getTelemetryName().get() + " SysId test done."));
    }
    return group;
  }

  @Override
  public void simIterate()
  {
    if (m_sim.isPresent())
    {
      m_sim.get().setInput(m_motor.getDutyCycle() * RoboRioSim.getVInVoltage());
      m_sim.get().update(m_motor.getConfig().getClosedLoopControlPeriod().in(Seconds));

      m_motor.simIterate(m_sim.get().getAngularVelocity());

      RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_sim.get().getCurrentDrawAmps()));
      visualizationUpdate();
    }
  }

  @Override
  public void updateTelemetry()
  {
    m_telemetry.positionPublisher.set(m_motor.getMechanismPosition().in(Degrees));
    m_motor.getMechanismPositionSetpoint().ifPresent(m_setpoint -> m_telemetry.setpointPublisher.set(m_setpoint.in(
        Degrees)));
    m_motor.updateTelemetry();
  }

  /**
   * Updates the angle of the mechanism ligament to match the current
   * angle of the pivot.
   */
  @Override
  public void visualizationUpdate()
  {
    mechanismLigament.setAngle(getAngle().in(Degrees));
  }

  /**
   * Get the relative position of the mechanism, taking into account the relative position defined
   * in the {@link MechanismPositionConfig}.
   *
   * @return The relative position of the mechanism as a {@link Translation3d}.
   */
  @Override
  public Translation3d getRelativeMechanismPosition()
  {
    Translation3d mechanismTranslation = new Translation3d(mechanismLigament.getLength(), new Rotation3d(0, 0, mechanismLigament.getAngle()));
    if (m_config.getMechanismPositionConfig().getRelativePosition().isPresent())
    {
      return m_config.getMechanismPositionConfig().getRelativePosition().get()
                     .plus(mechanismTranslation);
    }
    return mechanismTranslation;
  }

  /**
   * Get the {@link PivotConfig} object for this {@link Pivot}
   *
   * @return The {@link PivotConfig} object for this {@link Pivot}
   */
  public PivotConfig getPivotConfig()
  {
    return m_config;
  }
}
