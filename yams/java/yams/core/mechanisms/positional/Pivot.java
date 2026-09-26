// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.positional;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inch;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;

import java.util.Optional;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.BatterySim;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;
import yams.core.exceptions.PivotConfigurationException;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.MechanismPositionConfig;
import yams.core.mechanisms.config.PivotConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.SmartMotorControllerConfig;
import yams.core.motorcontrollers.simulation.DCMotorSimSupplier;

/**
 * Pivot mechanism.
 *
 * <p>A Pivot is a single-jointed rotation mechanism that rotates around the vertical axis, such
 * as a shooter hood or turret. This core class holds pivot state, physics simulation, and
 * measurement/predicate logic only. Command and Trigger factories live on
 * {@link yams.commands2.mechanisms.Pivot}, which extends this class and has a full usage example
 * in its Javadoc.
 */
public class Pivot extends SmartPositionalMechanism {
  /**
   * Pivot config.
   */
  private final PivotConfig    m_config;
  /**
   * Simulation for the Pivot.
   */
  private Optional<DCMotorSim> m_dcmotorSim       = Optional.empty();
  /**
   * Mechanism ligament for the setpoint.
   */
  private MechanismLigament2d  m_setpointLigament = null;

  /**
   * Construct the Pivot class
   *
   * @param config Pivot configuration.
   * @param smc    {@link SmartMotorController} driving the pivot.
   * @implNote Protected so only {@link yams.commands2.mechanisms.Pivot} can construct this.
   * @throws PivotConfigurationException if running in simulation and the lower or upper hard limit
   *                                     is not set, the starting position is not set on the
   *                                     {@link SmartMotorControllerConfig}, or the starting
   *                                     position is outside the hard limits.
   */
  protected Pivot(PivotConfig config, SmartMotorController smc) {
    m_config = config;
    m_smc = smc;
    SmartMotorControllerConfig<?> motorConfig = m_smc.getConfig();
    DCMotor dcMotor = m_smc.getDCMotor();
    MechanismGearing gearing = m_smc.getConfig().getGearing();
    // Seed the relative encoder
    if (m_smc.getConfig().getExternalEncoder().isPresent()) {
      m_smc.seedRelativeEncoder();
    }
    if (config.getTelemetryName().isPresent()) {
      m_telemetry.setupTelemetry(getName(), m_smc);
    }

    if (RobotBase.isSimulation()) {
      if (config.getLowerHardLimit().isEmpty()) {
        throw new PivotConfigurationException("Pivot lower hard limit is empty", "Cannot create simulation.", "withHardLimits(Angle,Angle)");
      }
      if (config.getUpperHardLimit().isEmpty()) {
        throw new PivotConfigurationException("Pivot upper hard limit is empty", "Cannot create simulation.", "withHardLimits(Angle,Angle)");
      }
      if (smc.getConfig().getStartingPosition().isEmpty()) {
        throw new PivotConfigurationException("Pivot starting angle is empty", "Cannot create simulation.", "SmartMotorControllerConfig.withStartingPosition(Angle)");
      }
      if (smc.getConfig().getStartingPosition().get().lt(config.getLowerHardLimit().get()) || smc.getConfig().getStartingPosition().get().gt(config.getUpperHardLimit().get())) {
        throw new PivotConfigurationException("Pivot starting angle is outside hard limits", "Cannot create simulation.", "SmartMotorControllerConfig.withStartingPosition(Angle)");
      }
      m_dcmotorSim = Optional.of(new DCMotorSim(Models.singleJointedArmFromPhysicalConstants(dcMotor, smc.getConfig().getMOI(), smc.getConfig().getGearing().getMechanismToRotorRatio()), dcMotor));

      m_smc.setSimSupplier(new DCMotorSimSupplier(m_dcmotorSim.get(), smc));
      Distance pivotLength = Inches.of(36);
      m_mechanismWindow = new Mechanism2d(pivotLength.in(Meters) * 2, pivotLength.in(Meters) * 2);
      m_mechanismRoot = m_mechanismWindow.getRoot(getName() + "Root", pivotLength.in(Meters), pivotLength.in(Meters));
      m_mechanismLigament = m_mechanismRoot.append(new MechanismLigament2d(getName(), pivotLength.in(Meters), smc.getConfig().getStartingPosition().get().in(Degrees), 6, config.getSimColor()));
      m_setpointLigament = m_mechanismRoot.append(new MechanismLigament2d("Setpoint", pivotLength.in(Meters), smc.getConfig().getStartingPosition().get().in(Degrees), 3, new Color8Bit(Color.WHITE)));
      m_mechanismRoot.append(new MechanismLigament2d("MaxHard", Inch.of(3).in(Meters), config.getUpperHardLimit().get().in(Degrees), 4, new Color8Bit(Color.LIME_GREEN)));
      m_mechanismRoot.append(new MechanismLigament2d("MinHard", Inch.of(3).in(Meters), config.getLowerHardLimit().get().in(Degrees), 4, new Color8Bit(Color.RED)));
      if (smc.getConfig().getMechanismLowerLimit().isPresent() && smc.getConfig().getMechanismUpperLimit().isPresent()) {
        m_mechanismRoot.append(new MechanismLigament2d("MaxSoft", Inch.of(3).in(Meters), smc.getConfig().getMechanismUpperLimit().get().in(Degrees), 4, new Color8Bit(Color.HOT_PINK)));
        m_mechanismRoot.append(new MechanismLigament2d("MinSoft", Inch.of(3).in(Meters), smc.getConfig().getMechanismLowerLimit().get().in(Degrees), 4, new Color8Bit(Color.YELLOW)));
      }
      publishMechanismWindow();
    }
  }

  /**
   * Between two angles.
   *
   * @param start Start angle.
   * @param end   End angle
   * @return True if the pivot is between the given angles.
   */
  public boolean isBetween(Angle start, Angle end) {
    return isGte(start) && isLte(end);
  }

  /**
   * Greater than or equal to angle.
   *
   * @param angle Angle to check against.
   * @return True if the pivot's angle is greater than or equal to the given angle.
   */
  public boolean isGte(Angle angle) {
    return getAngle().gte(angle);
  }

  /**
   * Less than or equal to angle
   *
   * @param angle {@link Angle} to check against
   * @return True if the pivot's angle is less than or equal to the given angle.
   */
  public boolean isLte(Angle angle) {
    return getAngle().lte(angle);
  }

  /**
   * Get the {@link SmartMotorController} Mechanism Position representing the pivot.
   *
   * @return Pivot {@link Angle}
   */
  public Angle getAngle() {
    return m_smc.getMechanismPosition();
  }

  /**
   * Pivot is near an angle.
   *
   * @param angle  {@link Angle} to be near.
   * @param within {@link Angle} within.
   * @return True if the pivot is near another angle.
   */
  public boolean isNear(Angle angle, Angle within) {
    return getAngle().isNear(angle, within);
  }

  /**
   * Whether the pivot is at or above its maximum angle, defined by the motor controller upper soft
   * limit if present, otherwise the pivot upper hard limit.
   *
   * @return True if the pivot is at its configured maximum.
   * @throws PivotConfigurationException if neither a motor controller upper soft limit nor a pivot
   *                                     upper hard limit is configured.
   */
  @Override
  public boolean isAtMax() {
    if (m_smc.getConfig().getMechanismUpperLimit().isPresent()) {
      return isGte(m_smc.getConfig().getMechanismUpperLimit().get());
    }
    if (m_config.getUpperHardLimit().isPresent()) {
      return isGte(m_config.getUpperHardLimit().get());
    }
    throw new PivotConfigurationException("Pivot upper hard and motor controller soft limit is empty", "Cannot create max trigger.", "withHardLimits(Angle,Angle)");
  }

  /**
   * Whether the pivot is at or below its minimum angle, defined by the motor controller lower soft
   * limit if present, otherwise the pivot lower hard limit.
   *
   * @return True if the pivot is at its configured minimum.
   * @throws PivotConfigurationException if neither a motor controller lower soft limit nor a pivot
   *                                     lower hard limit is configured.
   */
  @Override
  public boolean isAtMin() {
    if (m_smc.getConfig().getMechanismLowerLimit().isPresent()) {
      return isLte(m_smc.getConfig().getMechanismLowerLimit().get());
    }
    if (m_config.getLowerHardLimit().isPresent()) {
      return isLte(m_config.getLowerHardLimit().get());
    }
    throw new PivotConfigurationException("Pivot lower hard and motor controller soft limit is empty", "Cannot create min trigger.", "withHardLimits(Angle,Angle)");
  }

  @Override
  public void simIterate() {
    if (m_dcmotorSim.isPresent() && m_smc.getSimSupplier().isPresent()) {
      m_smc.getSimSupplier().get().updateSimState();
      m_smc.simIterate();
      m_smc.getSimSupplier().get().starveUpdateSim();
      if (m_config.getLowerHardLimit().isPresent() && m_dcmotorSim.get().getAngularVelocity() < 0 && m_smc.getMechanismPosition().lt(m_config.getLowerHardLimit().get())) {
        m_smc.setEncoderPosition(m_config.getLowerHardLimit().get());
      }
      if (m_config.getUpperHardLimit().isPresent() && m_dcmotorSim.get().getAngularVelocity() > 0 && m_smc.getMechanismPosition().gt(m_config.getUpperHardLimit().get())) {
        m_smc.setEncoderPosition(m_config.getUpperHardLimit().get());
      }
      RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_dcmotorSim.get().getCurrentDraw()));
      visualizationUpdate();
    }
  }

  @Override
  public void updateTelemetry() {
    //    m_telemetry.updatePosition(getAngle());
    //    m_motor.getMechanismPositionSetpoint().ifPresent(m_setpoint ->
    //    m_telemetry.updateSetpoint(m_setpoint));
    m_smc.updateTelemetry();
    m_telemetry.updateLoopTime();
  }

  /**
   * Updates the angle of the mechanism ligament to match the current angle of the pivot.
   */
  @Override
  public void visualizationUpdate() {
    m_mechanismLigament.setAngle(getAngle().in(Degrees));
    m_setpointLigament.setAngle(m_smc.getMechanismPositionSetpoint().orElse(getAngle()).in(Degrees));
  }

  /**
   * Get the relative position of the mechanism, taking into account the relative position defined
   * in the
   * {@link MechanismPositionConfig}.
   *
   * @return The relative position of the mechanism as a {@link Translation3d}.
   */
  @Override
  public Translation3d getRelativeMechanismPosition() {
    Translation3d mechanismTranslation = new Translation3d(m_mechanismLigament.getLength(), new Rotation3d(0, 0, m_mechanismLigament.getAngle()));
    if (m_config.getMechanismPositionConfig().getRelativePosition().isPresent()) {
      return m_config.getMechanismPositionConfig().getRelativePosition().get().plus(mechanismTranslation);
    }
    return mechanismTranslation;
  }

  @Override
  public String getName() {
    return m_config.getTelemetryName().orElse("Pivot");
  }

  /**
   * Get the {@link PivotConfig} object for this {@link Pivot}
   *
   * @return The {@link PivotConfig} object for this {@link Pivot}
   */
  public PivotConfig getPivotConfig() {
    return m_config;
  }
}
