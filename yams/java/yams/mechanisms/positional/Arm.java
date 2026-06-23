// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechanisms.positional;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
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
import yams.exceptions.ArmConfigurationException;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.MechanismPositionConfig.Plane;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.simulation.ArmSimSupplier;

/**
 * Arm mechanism.
 *
 * <h2>Usage Example</h2>
 * <pre>{@code
 * // Build and construct
 * SmartMotorController motor = new SparkWrapper(
 *     new SparkMax(1, MotorType.kBrushless), DCMotor.getNEO(1),
 *     new SmartMotorControllerConfig().withClosedLoopController(0.2,0,0).withStatorCurrentLimit(Amps.of(40)));
 * ArmConfig armConfig = new ArmConfig().withLength(Meters.of(0.5));
 * Arm arm = new Arm(armConfig, motor);
 *
 * // Schedule a setpoint command
 * Command moveToScore = arm.setAngle(Degrees.of(80));
 * Command holdAtZero = arm.runTo(Degrees.of(0), Degrees.of(2));
 *
 * // Bind triggers
 * arm.isNear(Degrees.of(80), Degrees.of(2)).onTrue(indexer.run());
 * arm.max().onTrue(Commands.print("Arm at max!"));
 *
 * // Call in robotPeriodic() or a subsystem's periodic():
 * arm.simIterate();
 * arm.updateTelemetry();
 * }</pre>
 */
public class Arm extends SmartPositionalMechanism
{
  /**
   * Arm config.
   */
  private final ArmConfig                     m_config;
  /**
   * Simulation for the arm.
   */
  private Optional<SingleJointedArmSim> m_sim              = Optional.empty();
  /**
   * Mechanism ligament for the setpoint.
   */
  private MechanismLigament2d           m_setpointLigament = null;

  /**
   * Constructor for the Arm mechanism.
   *
   * @param config {@link ArmConfig} to use.
   * @param smc {@link SmartMotorController} for the Arm.
   */
  public Arm(ArmConfig config, SmartMotorController smc)
  {
    this.m_config = config;
    m_smc = smc;
    SmartMotorControllerConfig smccfg = smc.getConfig();
    DCMotor          dcmotor = m_smc.getDCMotor();
    MechanismGearing gearing = m_smc.getConfig().getGearing();
    m_subsystem = smc.getConfig().getSubsystem();
    // Seed the relative encoder
    if (m_smc.getConfig().getExternalEncoder().isPresent())
    {
      m_smc.seedRelativeEncoder();
    }
    if (config.getTelemetryName().isPresent())
    {
      // TODO: Add telemetry units to config.
      m_telemetry.setupTelemetry(getName(), m_smc);
    }

    if (RobotBase.isSimulation())
    {
      if (config.getLength().isEmpty())
      {
        throw new ArmConfigurationException("Arm Length is empty", "Cannot create simulation.", "withLength(Distance)");
      }
      if (config.getLowerHardLimit().isEmpty())
      {
        throw new ArmConfigurationException("Arm lower hard limit is empty",
                                            "Cannot create simulation.",
                                            "withHardLimits(Angle,Angle)");
      }
      if (config.getUpperHardLimit().isEmpty())
      {
        throw new ArmConfigurationException("Arm upper hard limit is empty",
                                            "Cannot create simulation.",
                                            "withHardLimits(Angle,Angle)");
      }
      if (smccfg.getStartingPosition().isEmpty() && smccfg.getZeroOffset().isEmpty())
      {
        throw new ArmConfigurationException("Arm starting angle is empty",
                                            "Cannot create simulation.",
                                            "SmartMotorControllerConfig.withStartingPosition(Angle)");
      }
      if (smccfg.getStartingPosition().isPresent() &&
          (smccfg.getStartingPosition().get().lt(config.getLowerHardLimit().get()) ||
                  smccfg.getStartingPosition().get().gt(config.getUpperHardLimit().get())))
      {
        throw new ArmConfigurationException("Arm starting angle is outside hard limits",
                                            "Cannot create simulation.",
                                            "SmartMotorControllerConfig.withStartingPosition(Angle)");
      }
      m_sim = Optional.of(new SingleJointedArmSim(smc.getDCMotor(),
                                                  smccfg.getGearing().getMechanismToRotorRatio(),
                                                  smccfg.getMOI(),
                                                  config.getLength().get().in(Meters),
                                                  config.getLowerHardLimit().get().in(Radians),
                                                  config.getUpperHardLimit().get().in(Radians),
                                                  true,
                                                  smccfg.getStartingPosition().orElse(Rotations.zero()).in(Radians),
                                                  0.002 / 4096.0,
                                                  0.0));// Add noise with a std-dev of 1 tick
      m_smc.setSimSupplier(new ArmSimSupplier(m_sim.get(), m_smc));

      m_mechanismWindow = new Mechanism2d(config.getMechanismPositionConfig()
                                                .getWindowXDimension(config.getLength().get()).in(Meters),
                                          config.getMechanismPositionConfig()
                                                .getWindowYDimension(config.getLength().get()).in(Meters));
      m_mechanismRoot = m_mechanismWindow.getRoot(getName() + "Root",
                                                  config.getMechanismPositionConfig()
                                                        .getMechanismX(config.getLength().get()).in(Meters)
                                                  + config.getMechanismPositionConfig().getRelativePosition()
                                                          .orElse(new Translation3d()).getX(),
                                                  config.getMechanismPositionConfig()
                                                        .getMechanismY(config.getLength().get()).in(Meters)
                                                  + config.getMechanismPositionConfig().getRelativePosition()
                                                          .orElse(new Translation3d()).getZ()
                                                 );

      m_mechanismLigament = m_mechanismRoot.append(new MechanismLigament2d(getName(),
                                                                           config.getLength().get().in(Meters),
                                                                           smccfg.getStartingPosition()
                                                                                 .orElse(Rotations.zero()).in(Degrees),
                                                                           6,
                                                                           config.getSimColor()));
      m_setpointLigament = m_mechanismRoot.append(new MechanismLigament2d("Setpoint",
                                                                          config.getLength().get()
                                                                                .in(Meters),
              smccfg.getStartingPosition()
                                                                                .orElse(Rotations.zero())
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
      if (smccfg.getMechanismLowerLimit().isPresent() &&
          smccfg.getMechanismUpperLimit().isPresent())
      {
        m_mechanismRoot.append(new MechanismLigament2d("MaxSoft",
                                                       Inch.of(3).in(Meters),
                                                       smccfg.getMechanismUpperLimit().get()
                                                            .in(Degrees),
                                                       4,
                                                       new Color8Bit(Color.kHotPink)));
        m_mechanismRoot.append(new MechanismLigament2d("MinSoft", Inch.of(3).in(Meters),
                                                       smccfg.getMechanismLowerLimit().get()
                                                            .in(Degrees),
                                                       4, new Color8Bit(Color.kYellow)));
      }
      SmartDashboard.putData(getName() + "/mechanism", m_mechanismWindow);
      m_smc.setupSimulation();
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

  @Override
  public void simIterate()
  {
    if (m_sim.isPresent() && m_smc.getSimSupplier().isPresent())
    {
      m_smc.getSimSupplier().get().updateSimState();
      m_smc.simIterate();
      m_smc.getSimSupplier().get().starveUpdateSim();
      if (m_config.getLowerHardLimit().isPresent() && m_sim.get().getVelocityRadPerSec() < 0 &&
          m_smc.getMechanismPosition().lt(m_config.getLowerHardLimit().get()))
      {
        m_smc.setEncoderPosition(m_config.getLowerHardLimit().get());
      }
      if (m_config.getUpperHardLimit().isPresent() && m_sim.get().getVelocityRadPerSec() > 0 &&
          m_smc.getMechanismPosition().gt(m_config.getUpperHardLimit().get()))
      {
        m_smc.setEncoderPosition(m_config.getUpperHardLimit().get());
      }
      RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_sim.get().getCurrentDrawAmps()));
      visualizationUpdate();
    }
  }

  /**
   * Updates the mechanism ligament with the current angle of the arm.
   *
   * @see SmartPositionalMechanism#visualizationUpdate()
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
    return m_config.getTelemetryName().orElse("Arm");
  }

  /**
   * Get the {@link SmartMotorController} Mechanism Position representing the arm.
   *
   * @return Arm {@link Angle}
   */
  public Angle getAngle()
  {
    return m_smc.getMechanismPosition();
  }

  /**
   * Set the arm to the given angle.
   *
   * @param angle Arm angle to go to.
   * @return {@link Command} that sets the arm to the desired angle.
   */
  public Command setAngle(Angle angle)
  {
    return run(angle).withName(m_subsystem.getName() + " SetAngle");
  }

  /**
   * Set the arm to the given angle via a supplier.
   *
   * @param angle Supplier for the arm angle to go to.
   * @return {@link Command} that sets the arm to the desired angle.
   */
  public Command setAngle(Supplier<Angle> angle)
  {
    return run(angle).withName(m_subsystem.getName() + " SetAngle Supplier");
  }

  /**
   * Set the arm to the given angle.
   *
   * @param angle Arm angle to go to.
   * @return {@link Command} that sets the arm to the desired angle.
   */
  public Command run(Angle angle)
  {
    return Commands.run(() -> m_smc.setPosition(angle), m_subsystem).withName(m_subsystem.getName() + " SetAngle");
  }

  /**
   * Set the arm to the given angle via a supplier.
   *
   * @param angle Supplier for the arm angle to go to.
   * @return {@link Command} that sets the arm to the desired angle.
   */
  public Command run(Supplier<Angle> angle)
  {
    return Commands.run(() -> m_smc.setPosition(angle.get()), m_subsystem).withName(
        m_subsystem.getName() + " RunAngle Supplier");
  }

  /**
   * Set the arm to the given angle then end the command.
   *
   * @param angle     {@link Angle} to go to.
   * @param tolerance Tolerance {@link Angle}
   * @return {@link Command} that sets the arm to the desired angle.
   * @implNote This command will not stop. It should NOT be used when there is a default command on the Subsystem.
   */
  public Command runTo(Angle angle, Angle tolerance)
  {
    return Commands.runOnce(() -> m_smc.setPosition(angle), m_subsystem)
                   .andThen(Commands.waitUntil(isNear(angle, tolerance).debounce(0.1, DebounceType.kRising)))
                   .withName(m_subsystem.getName() + " RunTo Angle");
  }

  /**
   * Set the arm to the given angle then end the command.
   *
   * @param angle     {@link Angle} to go to.
   * @param tolerance Tolerance {@link Angle}
   * @return {@link Command} that sets the arm to the desired angle.
   * @implNote This command will stop, but the last control request to the motor controller will continue. It should NOT be used when there is a default command on the Subsystem.
   */
  public Command runTo(Supplier<Angle> angle, Angle tolerance)
  {
    return Commands.runOnce(() -> m_smc.setPosition(angle.get()), m_subsystem)
                   .andThen(Commands.waitUntil(isNear(angle.get(), tolerance).debounce(0.1, DebounceType.kRising)))
                   .withName(m_subsystem.getName() + " RunTo Angle Supplier");
  }

  /**
   * Arm is near an angle.
   *
   * @param angle  {@link Angle} to be near.
   * @param within {@link Angle} within.
   * @return {@link Trigger} on when the arm is near another angle.
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
    throw new ArmConfigurationException("Arm upper hard and motor controller soft limit is empty",
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
    throw new ArmConfigurationException("Arm lower hard and motor controller soft limit is empty",
                                        "Cannot create min trigger.",
                                        "withHardLimits(Angle,Angle)");
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
   * @return {@link Trigger} for Arm.
   */
  public Trigger gte(Angle angle)
  {
    return new Trigger(() -> getAngle().gte(angle));
  }

  /**
   * Get the {@link ArmConfig} for this {@link Arm}.
   *
   * @return The {@link ArmConfig} used to configure this {@link Arm}.
   */
  public ArmConfig getArmConfig()
  {
    return m_config;
  }
}
