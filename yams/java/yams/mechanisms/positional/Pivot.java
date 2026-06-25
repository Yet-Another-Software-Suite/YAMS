// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechanisms.positional;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.function.Supplier;
import yams.exceptions.PivotConfigurationException;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.simulation.DCMotorSimSupplier;

/**
 * Pivot mechanism.
 *
 * <p>A Pivot is a single-jointed rotation mechanism that rotates around the vertical axis,
 * such as a shooter hood or turret. It is controlled by a {@link SmartMotorController} and
 * supports position control, trigger bindings, and simulation.</p>
 *
 * <h2>Usage Example</h2>
 * <pre>{@code
 * // --- Configuration ---
 * SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig()
 *     .withClosedLoopController(0.15,0,0.004)
 *     .withFeedforward(new SimpleMotorFeedforward(0.05,0,0)
 *     .withStatorCurrentLimit(Amps.of(40))
 *     .withMechanismUpperLimit(Degrees.of(60))
 *     .withMechanismLowerLimit(Degrees.of(0))
 *     .withStartingPosition(Degrees.of(0));
 *
 * SmartMotorController motor = new TalonFXWrapper(
 *     new TalonFX(3),
 *     DCMotor.getKrakenX60(1),
 *     motorConfig);
 *
 * PivotConfig pivotConfig = new PivotConfig()
 *     .withHardLimits(Degrees.of(0), Degrees.of(60))
 *     .withTelemetry("ShooterHood", TelemetryVerbosity.HIGH);
 *
 * // --- Instantiation ---
 * Pivot pivot = new Pivot(pivotConfig, motor);
 *
 * // --- Commands ---
 * // setAngle() returns a Command that continuously drives the pivot to the target angle
 * Command aimHigh = pivot.setAngle(Degrees.of(45));
 *
 * // runTo() drives to the angle and ends once the pivot is within tolerance
 * Command stowPivot = pivot.runTo(Degrees.of(0), Degrees.of(1));
 *
 * // --- Trigger bindings ---
 * // Fire when the pivot is within 2 degrees of the shooting angle
 * pivot.isNear(Degrees.of(45), Degrees.of(2)).onTrue(shooter.runShooter());
 *
 * // Bind on boundary conditions
 * pivot.gte(Degrees.of(55)).onTrue(Commands.print("Approaching upper limit!"));
 * pivot.lte(Degrees.of(5)).onTrue(Commands.print("Pivot near stow position."));
 *
 * // --- Periodic callbacks (robotPeriodic or subsystem periodic) ---
 * pivot.simIterate();      // advances simulation state each loop
 * pivot.updateTelemetry(); // publishes data to NetworkTables/SmartDashboard
 * }</pre>
 */
public class Pivot extends SmartPositionalMechanism
{
  /**
   * Pivot config.
   */
  private final PivotConfig          m_config;
  /**
   * Simulation for the Pivot.
   */
  private       Optional<DCMotorSim> m_dcmotorSim = Optional.empty();
  /**
   * Mechanism ligament for the setpoint.
   */
  private MechanismLigament2d m_setpointLigament = null;

  /**
   * Construct the Pivot class
   *
   * @param config Pivot configuration.
   * @param smc    {@link SmartMotorController} driving the pivot.
   */
  public Pivot(PivotConfig config, SmartMotorController smc)
  {
    m_config = config;
    m_smc = smc;
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
      m_telemetry.setupTelemetry(getName(), m_smc);
    }

    if (RobotBase.isSimulation())
    {
      if (config.getLowerHardLimit().isEmpty())
      {
        throw new PivotConfigurationException("Pivot lower hard limit is empty",
                                              "Cannot create simulation.",
                                              "withHardLimits(Angle,Angle)");
      }
      if (config.getUpperHardLimit().isEmpty())
      {
        throw new PivotConfigurationException("Pivot upper hard limit is empty",
                                              "Cannot create simulation.",
                                              "withHardLimits(Angle,Angle)");
      }
      if (smc.getConfig().getStartingPosition().isEmpty())
      {
        throw new PivotConfigurationException("Pivot starting angle is empty",
                                              "Cannot create simulation.",
                                              "SmartMotorControllerConfig.withStartingPosition(Angle)");
      }
      if (smc.getConfig().getStartingPosition().get().lt(config.getLowerHardLimit().get()) ||
          smc.getConfig().getStartingPosition().get().gt(config.getUpperHardLimit().get()))
      {
        throw new PivotConfigurationException("Pivot starting angle is outside hard limits",
                                              "Cannot create simulation.",
                                              "SmartMotorControllerConfig.withStartingPosition(Angle)");
      }
      m_dcmotorSim = Optional.of(new DCMotorSim(LinearSystemId.createDCMotorSystem(dcMotor,
                                                                                   smc.getConfig().getMOI(),
                                                                                   smc.getConfig().getGearing()
                                                                                      .getMechanismToRotorRatio()),
                                                dcMotor));

      m_smc.setSimSupplier(new DCMotorSimSupplier(m_dcmotorSim.get(), smc));
      Distance pivotLength = Inches.of(36);
      m_mechanismWindow = new Mechanism2d(pivotLength.in(Meters) * 2,
                                          pivotLength.in(Meters) * 2);
      m_mechanismRoot = m_mechanismWindow.getRoot(getName() + "Root",
                                                  pivotLength.in(Meters), pivotLength.in(Meters));
      m_mechanismLigament = m_mechanismRoot.append(new MechanismLigament2d(getName(),
                                                                           pivotLength.in(Meters),
                                                                           smc.getConfig().getStartingPosition().get().in(Degrees),
                                                                           6,
                                                                           config.getSimColor()));
      m_setpointLigament = m_mechanismRoot.append(new MechanismLigament2d("Setpoint",
                                                                          pivotLength.in(Meters),
                                                                          smc.getConfig().getStartingPosition().get()
                                                                                .in(Degrees),
                                                                          3,
                                                                          new Color8Bit(Color.kWhite)));
      m_mechanismRoot.append(new MechanismLigament2d("MaxHard",
                                                     Inch.of(3).in(Meters),
                                                     config.getUpperHardLimit().get()
                                                           .in(Degrees),
                                                     4,
                                                     new Color8Bit(Color.kLimeGreen)));
      m_mechanismRoot.append(new MechanismLigament2d("MinHard", Inch.of(3).in(Meters),
                                                     config.getLowerHardLimit().get()
                                                           .in(Degrees),
                                                     4, new Color8Bit(Color.kRed)));
      if (smc.getConfig().getMechanismLowerLimit().isPresent() &&
          smc.getConfig().getMechanismUpperLimit().isPresent())
      {
        m_mechanismRoot.append(new MechanismLigament2d("MaxSoft",
                                                       Inch.of(3).in(Meters),
                                                       smc.getConfig().getMechanismUpperLimit().get()
                                                          .in(Degrees),
                                                       4,
                                                       new Color8Bit(Color.kHotPink)));
        m_mechanismRoot.append(new MechanismLigament2d("MinSoft", Inch.of(3).in(Meters),
                                                       smc.getConfig().getMechanismLowerLimit().get()
                                                          .in(Degrees),
                                                       4, new Color8Bit(Color.kYellow)));
      }
      SmartDashboard.putData(getName() + "/mechanism",
                             m_mechanismWindow);
    }
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
   * Get the {@link SmartMotorController} Mechanism Position representing the pivot.
   *
   * @return Pivot {@link Angle}
   */
  public Angle getAngle()
  {
    return m_smc.getMechanismPosition();
  }

  /**
   * Set the pivot to the given angle.
   *
   * @param angle Pivot angle to go to.
   * @return {@link Command} that sets the pivot to the desired angle.
   */
  public Command setAngle(Angle angle)
  {
    return run(angle).withName(m_subsystem.getName() + " SetAngle");
  }

  /**
   * Set the pivot to the given angle.
   *
   * @param angle Pivot angle to go to.
   * @return {@link Command} that sets the pivot to the desired angle.
   */
  public Command setAngle(Supplier<Angle> angle)
  {
    return run(angle).withName(m_subsystem.getName() + " SetAngle Supplier");
  }

  /**
   * Set the pivot to the given angle.
   *
   * @param angle Pivot angle to go to.
   * @return {@link Command} that sets the pivot to the desired angle.
   */
  public Command run(Angle angle)
  {
    return Commands.run(() -> m_smc.setPosition(angle), m_subsystem).withName(m_subsystem.getName() + " SetAngle");
  }

  /**
   * Set the pivot to the given angle via a supplier.
   *
   * @param angle Supplier for the pivot angle to go to.
   * @return {@link Command} that sets the pivot to the desired angle.
   */
  public Command run(Supplier<Angle> angle)
  {
    return Commands.run(() -> m_smc.setPosition(angle.get()), m_subsystem).withName(
        m_subsystem.getName() + " RunAngle Supplier");
  }

  /**
   * Set the pivot to the given {@link Angle} then end the command.
   *
   * @param angle     {@link Angle} to go to.
   * @param tolerance Tolerance {@link Angle}
   * @return {@link Command} that sets the pivot to the desired angle.
   * @implNote This command will not stop. It should NOT be used when there is a default command on the Subsystem.
   */
  public Command runTo(Angle angle, Angle tolerance)
  {
    return Commands.runOnce(() -> m_smc.setPosition(angle), m_subsystem)
                   .andThen(Commands.waitUntil(isNear(angle, tolerance).debounce(0.1, DebounceType.kRising)))
                   .withName(m_subsystem.getName() + " RunTo Angle");
  }

  /**
   * Set the pivot to the given angle then end the command.
   *
   * @param angle     {@link Angle} to go to.
   * @param tolerance Tolerance {@link Angle}
   * @return {@link Command} that sets the pivot to the desired angle.
   * @implNote This command will not stop. It should NOT be used when there is a default command on the Subsystem.
   */
  public Command runTo(Supplier<Angle> angle, Angle tolerance)
  {
    return Commands.runOnce(() -> m_smc.setPosition(angle.get()), m_subsystem)
                   .andThen(Commands.waitUntil(isNear(angle.get(), tolerance).debounce(0.1, DebounceType.kRising)))
                   .withName(m_subsystem.getName() + " RunTo Angle Supplier");
  }

  /**
   * Pivot is near an angle.
   *
   * @param angle  {@link Angle} to be near.
   * @param within {@link Angle} within.
   * @return {@link Trigger} on when the pivot is near another angle.
   */
  public Trigger isNear(Angle angle, Angle within)
  {
    return new Trigger(() -> getAngle().isNear(angle, within));
  }

  @Override
  public Trigger max()
  {
    if (m_smc.getConfig().getMechanismUpperLimit().isPresent())
    {
      return new Trigger(gte(m_smc.getConfig().getMechanismUpperLimit().get()));
    }
    if (m_config.getUpperHardLimit().isPresent())
    {
      return gte(m_config.getUpperHardLimit().get());
    }
    throw new PivotConfigurationException("Pivot upper hard and motor controller soft limit is empty",
                                        "Cannot create max trigger.",
                                        "withHardLimits(Angle,Angle)");
  }

  @Override
  public Trigger min()
  {
    if (m_smc.getConfig().getMechanismLowerLimit().isPresent())
    {
      return new Trigger(lte(m_smc.getConfig().getMechanismLowerLimit().get()));
    }
    if (m_config.getLowerHardLimit().isPresent())
    {
      return lte(m_config.getLowerHardLimit().get());
    }
    throw new PivotConfigurationException("Pivot lower hard and motor controller soft limit is empty",
                                        "Cannot create min trigger.",
                                        "withHardLimits(Angle,Angle)");
  }

  @Override
  public void simIterate()
  {
    if (m_dcmotorSim.isPresent() && m_smc.getSimSupplier().isPresent())
    {
      m_smc.getSimSupplier().get().updateSimState();
      m_smc.simIterate();
      m_smc.getSimSupplier().get().starveUpdateSim();
      if (m_config.getLowerHardLimit().isPresent() && m_dcmotorSim.get().getAngularVelocityRadPerSec() < 0 &&
          m_smc.getMechanismPosition().lt(m_config.getLowerHardLimit().get()))
      {
        m_smc.setEncoderPosition(m_config.getLowerHardLimit().get());
      }
      if (m_config.getUpperHardLimit().isPresent() && m_dcmotorSim.get().getAngularVelocityRadPerSec() > 0 &&
          m_smc.getMechanismPosition().gt(m_config.getUpperHardLimit().get()))
      {
        m_smc.setEncoderPosition(m_config.getUpperHardLimit().get());
      }
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
   * Updates the angle of the mechanism ligament to match the current angle of the pivot.
   */
  @Override
  public void visualizationUpdate()
  {
    m_mechanismLigament.setAngle(getAngle().in(Degrees));
    m_setpointLigament.setAngle(m_smc.getMechanismPositionSetpoint().orElse(getAngle()).in(Degrees));
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
    Translation3d mechanismTranslation = new Translation3d(m_mechanismLigament.getLength(),
                                                           new Rotation3d(0, 0, m_mechanismLigament.getAngle()));
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
    return m_config.getTelemetryName().orElse("Pivot");
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
