package yams.mechanisms.velocity;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.function.Supplier;
import yams.exceptions.ShooterConfigurationException;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.ShooterConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.simulation.DCMotorSimSupplier;

/**
 * Shooter mechanism.
 */
public class Shooter extends SmartVelocityMechanism
{

  /**
   * Shooter config.
   */
  private final ShooterConfig        m_config;
  /**
   * Simulation for the Shooter.
   */
  private       Optional<DCMotorSim> m_dcmotorSim = Optional.empty();

  /**
   * Construct the Shooter class
   *
   * @param config Shooter configuration.
   */
  public Shooter(ShooterConfig config)
  {
    m_config = config;
    m_smc = config.getMotor();
    SmartMotorControllerConfig motorConfig = m_smc.getConfig();
    DCMotor                    dcMotor     = m_smc.getDCMotor();
    MechanismGearing           gearing     = m_smc.getConfig().getGearing();
    m_subsystem = m_smc.getConfig().getSubsystem();
    // Seed the relative encoder
    if (m_smc.getConfig().getExternalEncoder().isPresent())
    {
      m_smc.seedRelativeEncoder();
    }
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
      m_dcmotorSim = Optional.of(new DCMotorSim(LinearSystemId.createDCMotorSystem(dcMotor,
                                                                                   config.getMOI(),
                                                                                   motor.getConfig().getGearing()
                                                                                        .getMechanismToRotorRatio()),
                                                dcMotor));

      m_smc.setSimSupplier(new DCMotorSimSupplier(m_dcmotorSim.get(), m_smc));
      Distance ShooterLength = config.getLength().orElse(Inches.of(36));
      m_mechanismWindow = new Mechanism2d(ShooterLength.in(Meters) * 2,
                                        ShooterLength.in(Meters) * 2);
      mechanismRoot = m_mechanismWindow.getRoot(getName() + "Root",
                                                ShooterLength.in(Meters), ShooterLength.in(Meters));
      mechanismLigament = mechanismRoot.append(new MechanismLigament2d(getName(),
                                                                       ShooterLength.in(Meters),
                                                                       0, 6, config.getSimColor()));

      SmartDashboard.putData(getName() + "/mechanism", m_mechanismWindow);
    }
  }

  /**
   * Between two velocities.
   *
   * @param start Start Velocity.
   * @param end   End velocity
   * @return {@link Trigger}
   */
  public Trigger between(AngularVelocity start, AngularVelocity end)
  {
    return gte(start).and(lte(end));
  }

  /**
   * Greater than or equal to angular velocity.
   *
   * @param speed {@link AngularVelocity} to check against.
   * @return {@link Trigger} for Shooter.
   */
  public Trigger gte(AngularVelocity speed)
  {
    return new Trigger(() -> getSpeed().gte(speed));
  }

  /**
   * Less than or equal to angular velocity
   *
   * @param speed {@link AngularVelocity} to check against
   * @return {@link Trigger}
   */
  public Trigger lte(AngularVelocity speed)
  {
    return new Trigger(() -> getSpeed().lte(speed));
  }

  /**
   * Get the {@link SmartMotorController} Mechanism velocity representing the Shooter.
   *
   * @return Shooter {@link AngularVelocity}
   */
  public AngularVelocity getSpeed()
  {
    return m_smc.getMechanismVelocity();
  }

  /**
   * Shooter is near a speed.
   *
   * @param speed  {@link AngularVelocity} to be near.
   * @param within {@link AngularVelocity} within.
   * @return Trigger on when the Shooter is near another speed.
   */
  public Trigger isNear(AngularVelocity speed, AngularVelocity within)
  {
    return new Trigger(() -> getSpeed().isNear(speed, within));
  }

  /**
   * Set the Shooter to the given speed.
   *
   * @param speed Shooter speed to go to.
   * @return {@link Command} that sets the Shooter to the desired speed.
   */
  public Command setSpeed(AngularVelocity speed)
  {
    m_config.getLowerSoftLimit().ifPresent(low -> {
      if (low.gt(speed))
      {
        DriverStation.reportWarning("[WARNING] You have requested to set " + getName() + " to " + speed +
                                    " which is lower than minimum velocity " + low + "!", false);
      }
    });
    m_config.getUpperSoftLimit().ifPresent(high -> {
      if (high.lt(speed))
      {
        DriverStation.reportWarning("[WARNING] You have requested to set " + getName() + " to " + speed +
                                    " which is greater than maximum velocity " + high + "!", false);
      }
    });

    return Commands.run(() -> m_smc.setVelocity(speed), m_subsystem).withName(
        m_subsystem.getName() + " " + getName() + " SetSpeed");
  }

  /**
   * Set the Shooter to the given speed.
   *
   * @param speed Shooter speed to go to.
   * @return {@link Command} that sets the Shooter to the desired speed.
   */
  public Command setSpeed(Supplier<AngularVelocity> speed)
  {
    return Commands.run(() -> m_smc.setVelocity(speed.get()), m_subsystem).withName(
        m_subsystem.getName() + " SetSpeed Supplier");
  }

  @Override
  public Trigger max()
  {
    if (m_config.getUpperSoftLimit().isPresent())
    {
      return new Trigger(gte(m_config.getUpperSoftLimit().get()));
    }
    throw new ShooterConfigurationException("Shooter upper soft limit is empty",
                                            "Cannot create max trigger.",
                                            "withUpperSoftLimit(AngularVelocity) OR withSoftLimit(AngularVelocity,AngularVelocity)");
  }

  @Override
  public Trigger min()
  {
    if (m_config.getLowerSoftLimit().isPresent())
    {
      return new Trigger(gte(m_config.getLowerSoftLimit().get()));
    }
    throw new ShooterConfigurationException("Shooter upper soft limit is empty",
                                            "Cannot create max trigger.",
                                            "withLowerSoftLimit(AngularVelocity) OR withSoftLimit(AngularVelocity,AngularVelocity)");
  }

  @Override
  public Command sysId(Voltage maximumVoltage, Velocity<VoltageUnit> step, Time duration)
  {
    return Commands.print("You cannot SysId a velocity based mechanism!");
//        SysIdRoutine routine = m_motor.sysId(maximumVoltage, step, duration);
//        Angle max;
//        Angle min;
//        if (m_motor.getConfig().getMechanismUpperLimit().isPresent()) {
//            max = m_motor.getConfig().getMechanismUpperLimit().get().minus(Degrees.of(1));
//        } else if (m_config.getUpperHardLimit().isPresent()) {
//            max = m_config.getUpperHardLimit().get().minus(Degrees.of(1));
//        } else {
//            throw new ShooterConfigurationException("Shooter upper hard and motor controller soft limit is empty",
//                    "Cannot create SysIdRoutine.",
//                    "withHardLimit(Angle,Angle)");
//        }
//        if (m_motor.getConfig().getMechanismLowerLimit().isPresent()) {
//            min = m_motor.getConfig().getMechanismLowerLimit().get().plus(Degrees.of(1));
//        } else if (m_config.getLowerHardLimit().isPresent()) {
//            min = m_config.getLowerHardLimit().get().plus(Degrees.of(1));
//        } else {
//            throw new ShooterConfigurationException("Shooter lower hard and motor controller soft limit is empty",
//                    "Cannot create SysIdRoutine.",
//                    "withHardLimit(Angle,Angle)");
//        }
//        Trigger maxTrigger = gte(max);
//        Trigger minTrigger = lte(min);
//
//        Command group = Commands.print("Starting SysId")
//                .andThen(Commands.runOnce(m_motor::stopClosedLoopController))
//                .andThen(routine.dynamic(Direction.kForward).until(maxTrigger))
//                .andThen(routine.dynamic(Direction.kReverse).until(minTrigger))
//                .andThen(routine.quasistatic(Direction.kForward).until(maxTrigger))
//                .andThen(routine.quasistatic(Direction.kReverse).until(minTrigger))
//                .finallyDo(m_motor::startClosedLoopController);
//        if (m_config.getTelemetryName().isPresent()) {
//            group = group.andThen(Commands.print(m_getName() + " SysId test done."));
//        }
//        return group.withName(m_subsystem.getName() + " SysId");
  }

  @Override
  public void simIterate()
  {
    if (m_dcmotorSim.isPresent() && m_smc.getSimSupplier().isPresent())
    {
      m_smc.getSimSupplier().get().updateSimState();
      m_smc.simIterate();
      m_smc.getSimSupplier().get().starveUpdateSim();

      RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_dcmotorSim.get()
                                                                                           .getCurrentDrawAmps()));
      visualizationUpdate();
    }
  }

  @Override
  public void updateTelemetry()
  {
//    m_telemetry.updatePosition(getAngle());
//    m_motor.getMechanismPositionSetpoint().ifPresent(m_setpoint -> m_telemetry.updateSetpoint(m_setpoint));
    m_smc.updateTelemetry();
  }

  /**
   * Updates the angle of the mechanism ligament to match the current angle of the Shooter.
   */
  @Override
  public void visualizationUpdate()
  {
    mechanismLigament.setAngle(m_smc.getMechanismPosition().in(Degrees));
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
    Translation3d mechanismTranslation = new Translation3d(mechanismLigament.getLength(),
                                                           new Rotation3d(0, 0, mechanismLigament.getAngle()));
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
    return m_config.getTelemetryName().orElse("Shooter");
  }

  /**
   * Get the {@link ShooterConfig} object for this {@link Shooter}
   *
   * @return The {@link ShooterConfig} object for this {@link Shooter}
   */
  public ShooterConfig getShooterConfig()
  {
    return m_config;
  }
}
