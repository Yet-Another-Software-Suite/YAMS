// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechanisms.velocity;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
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
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.simulation.DCMotorSimSupplier;

/**
 * FlyWheel mechanism.
 *
 * <h2>Usage Example</h2>
 * <pre>{@code
 * // Construct using a fully configured FlyWheelConfig
 * FlyWheel shooter = new FlyWheel(config);
 *
 * // Spin at a fixed target RPM (runs continuously as a RunCommand)
 * Command spinUp = shooter.run(RPM.of(3000));
 *
 * // Block until the wheel reaches 3000 RPM within ±50 RPM, then finish
 * Command spinToSpeed = shooter.runTo(RPM.of(3000), RPM.of(50));
 *
 * // Trigger that is true whenever the wheel is within ±50 RPM of target
 * Trigger atSpeed = shooter.isNear(RPM.of(3000), RPM.of(50));
 * atSpeed.onTrue(Commands.print("Shooter at speed!"));
 * }</pre>
 *
 * <p><b>Unsupported operations:</b> {@link #max()} and {@link #min()} are not supported for
 * velocity mechanisms and will throw {@link java.lang.UnsupportedOperationException} if called.
 * Use {@link #isNear(edu.wpi.first.units.measure.AngularVelocity,
 * edu.wpi.first.units.measure.AngularVelocity)} or {@link #gte}/{@link #lte} triggers instead.</p>
 *
 * <p>Call {@link #simIterate()} and {@link #updateTelemetry()} inside your subsystem's
 * {@code periodic()} method to keep simulation state and NetworkTables up to date:</p>
 * <pre>{@code
 * @Override
 * public void periodic() {
 *     shooter.simIterate();       // advances the DCMotorSim each loop
 *     shooter.updateTelemetry();  // pushes data to SmartDashboard / AdvantageScope
 * }
 * }</pre>
 */
public class FlyWheel extends SmartVelocityMechanism
{
  /**
   * FlyWheel config.
   */
  private final FlyWheelConfig       m_config;
  /**
   * Simulation for the FlyWheel.
   */
  private       Optional<DCMotorSim> m_dcmotorSim = Optional.empty();

  /**
   * Construct the FlyWheel class
   *
   * @param config FlyWheel configuration.
   * @param smc {@link SmartMotorController} for the Mechanism
   */
  public FlyWheel(FlyWheelConfig config, SmartMotorController smc)
  {
    m_config = config;
    m_smc = smc;
    SmartMotorControllerConfig smcCfg = smc.getConfig();
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

    if (RobotBase.isSimulation())
    {
      m_dcmotorSim = Optional.of(new DCMotorSim(LinearSystemId.createDCMotorSystem(dcMotor,
                                                                                   smcCfg.getMOI(),
                                                                                   smcCfg.getGearing()
                                                                                        .getMechanismToRotorRatio()),
                                                dcMotor));

      m_smc.setSimSupplier(new DCMotorSimSupplier(m_dcmotorSim.get(), m_smc));
      Distance ShooterLength = config.getDiameter().orElse(Inches.of(36));
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
   * @return {@link Trigger} for FlyWheel.
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
   * Get the {@link SmartMotorController} Mechanism velocity representing the FlyWheel.
   *
   * @return FlyWheel {@link AngularVelocity}
   */
  public AngularVelocity getSpeed()
  {
    return m_smc.getMechanismVelocity();
  }

  /**
   * Get the {@link LinearVelocity} of the FlyWheel.
   *
   * @return FlyWheel {@link LinearVelocity}
   */
  public LinearVelocity getLinearVelocity() {return m_config.getLinearVelocity(m_smc.getMechanismVelocity());}

  /**
   * FlyWheel is near a speed.
   *
   * @param speed  {@link AngularVelocity} to be near.
   * @param within {@link AngularVelocity} within.
   * @return Trigger on when the FlyWheel is near another speed.
   */
  public Trigger isNear(AngularVelocity speed, AngularVelocity within)
  {
    return new Trigger(() -> getSpeed().isNear(speed, within));
  }

  /**
   * Run the FlyWheel to a given velocity
   *
   * @param velocity {@link Supplier} of {@link LinearVelocity} or {@link AngularVelocity}
   * @param <T>      Must be a {@link LinearVelocity} or {@link AngularVelocity}
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand} which runs the FlyWheel to the desired velocity with the
   * closed loop controller.
   */
  public <T> Command run(Supplier<T> velocity)
  {
    var cmdName = m_subsystem.getName() + " RunSpeed Supplier";
    if (velocity.get() instanceof AngularVelocity)
    {
      return Commands.startRun(m_smc::startClosedLoopController,
                               () -> m_smc.setVelocity((AngularVelocity) velocity.get()),
                               m_subsystem)
                     .withName(cmdName);
    } else if (velocity.get() instanceof LinearVelocity)
    {
      m_config.getCircumference(); // Circumference check
      return Commands.startRun(m_smc::startClosedLoopController,
                               () -> m_smc.setVelocity(m_config.getAngularVelocity((LinearVelocity) velocity.get())),
                               m_subsystem)
                     .withName(cmdName);
    }
    throw new IllegalArgumentException("Velocity must be an AngularVelocity or LinearVelocity");
  }

  /**
   * Set the FlyWheel to the given speed.
   *
   * @param velocity FlyWheel speed to go to.
   * @return {@link Command} that sets the FlyWheel to the desired speed.
   */
  public Command run(AngularVelocity velocity)
  {
    return Commands.run(()->m_smc.setVelocity(velocity), m_subsystem).withName(m_subsystem.getName() + " " + getName() + " SetSpeed");
  }

  /**
   * Run the FlyWheel to a velocity within a tolerance, then end the command.
   *
   * @param velocity  {@link Supplier} of {@link AngularVelocity}
   * @param tolerance {@link AngularVelocity} tolerance
   * @return {@link Command} that runs the FlyWheel to the desired velocity then moves on.
   * @implNote If you are using this function, try not to have a default command or else the default command will
   * override the setting after this command ends.
   */
  public Command runTo(Supplier<AngularVelocity> velocity, AngularVelocity tolerance)
  {
    return Commands.runOnce(m_smc::startClosedLoopController, m_subsystem)
                   .andThen(Commands.runOnce(() -> m_smc.setVelocity(velocity.get()), m_subsystem))
                   .andThen(Commands.waitUntil(isNear(velocity.get(), tolerance).debounce(0.1, DebounceType.kRising)))
                   .withName(m_subsystem.getName() + " RunToVelocity Supplier");
  }

  /**
   * Run the FlyWheel to a velocity within a tolerance, then end the command.
   *
   * @param velocity  {@link AngularVelocity} to go to.
   * @param tolerance {@link AngularVelocity} tolerance
   * @return {@link Command} that runs the FlyWheel to the desired velocity then moves on.
   * @implNote If you are using this function, try not to have a default command or else the default command will
   * override the setting after this command ends.
   */
  public Command runTo(AngularVelocity velocity, AngularVelocity tolerance)
  {
    return Commands.runOnce(m_smc::startClosedLoopController, m_subsystem)
                   .andThen(Commands.runOnce(() -> m_smc.setVelocity(velocity), m_subsystem))
                   .andThen(Commands.waitUntil(isNear(velocity, tolerance).debounce(0.1, DebounceType.kRising)))
                   .withName(m_subsystem.getName() + " RunToVelocity");
  }

  /**
   * Run the FlyWheel to a velocity within a tolerance then end the command.
   *
   * @param velocity  {@link LinearVelocity} to go to.
   * @param tolerance {@link LinearVelocity} tolerance
   * @return {@link Command} that runs the FlyWheel to the desired velocity then moves on.
   * @implNote If you are using this function, try not to have a default command or else the default command will
   * override the setting after this command ends.
   */
  public Command runTo(LinearVelocity velocity, LinearVelocity tolerance)
  {
    m_config.getCircumference(); // Circumference check
    return runTo(m_config.getAngularVelocity(velocity), m_config.getAngularVelocity(tolerance));
  }

  /**
   * Run the FlyWheel to a velocity within a tolerance then end the command.
   *
   * @param velocity  {@link LinearVelocity} to go to.
   * @param tolerance {@link LinearVelocity} tolerance
   * @return {@link Command} that runs the FlyWheel to the desired velocity then moves on.
   * @implNote If you are using this function, try not to have a default command or else the default command will
   * override the setting after this command ends.
   */
  public Command runTo(Supplier<LinearVelocity> velocity, LinearVelocity tolerance)
  {
    m_config.getCircumference(); // Circumference check
    return runTo(() -> m_config.getAngularVelocity(velocity.get()), m_config.getAngularVelocity(tolerance));
  }


  /**
   * Set the FlyWheel to the given speed.
   *
   * @param speed FlyWheel speed to go to.
   * @return {@link Command} that sets the FlyWheel to the desired speed.
   */
  public Command run(LinearVelocity speed)
  {
    return run(m_config.getAngularVelocity(speed)).withName(m_subsystem.getName() + " RunSpeed");
  }

  /**
   * Mechanism velocity setpoint.
   *
   * @return {@link AngularVelocity} setpoint of the FlyWheel.
   */
  public Optional<AngularVelocity> getMechanismSetpointVelocity() {return m_smc.getMechanismSetpointVelocity();}

  /**
   * Set the FlyWheel to the given speed.
   *
   * @param speed {@link LinearVelocity} to go to.
   */
  @Override
  public void setMeasurementVelocitySetpoint(LinearVelocity speed)
  {
    m_smc.startClosedLoopController();
    m_smc.setVelocity(m_config.getAngularVelocity(speed));
  }

  @Override
  public Trigger max()
  {
    throw new UnsupportedOperationException("Velocity soft limits have been removed from FlyWheel.");
  }

  @Override
  public Trigger min()
  {
    throw new UnsupportedOperationException("Velocity soft limits have been removed from FlyWheel.");
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
    m_telemetry.updateLoopTime();
  }

  /**
   * Updates the angle of the mechanism ligament to match the current angle of the FlyWheel.
   */
  @Override
  public void visualizationUpdate()
  {
    if (m_config.isUsingSpeedometerSimulation() && m_config.getSpeedometerMaxVelocity().isPresent())
    {
      mechanismLigament.setAngle(
          270 - m_smc.getMechanismVelocity().in(RPM) / m_config.getSpeedometerMaxVelocity().get().in(RPM) * 180);
    } else
    {
      mechanismLigament.setAngle(m_smc.getMechanismPosition().in(Degrees));
    }
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
    return m_config.getTelemetryName().orElse("FlyWheel");
  }

  /**
   * Get the {@link FlyWheelConfig} object for this {@link FlyWheel}
   *
   * @return The {@link FlyWheelConfig} object for this {@link FlyWheel}
   */
  public FlyWheelConfig getShooterConfig()
  {
    return m_config;
  }
}
