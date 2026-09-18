// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.positional;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inch;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Rotations;

import java.util.Optional;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.BatterySim;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;
import yams.core.exceptions.ArmConfigurationException;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.mechanisms.config.MechanismPositionConfig;
import yams.core.mechanisms.config.MechanismPositionConfig.Plane;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.SmartMotorControllerConfig;
import yams.core.motorcontrollers.simulation.ArmSimSupplier;

/**
 * Arm mechanism.
 *
 * <p>This core class holds arm state, physics simulation, and measurement/predicate logic only.
 * Command and Trigger factories ({@code setAngle}, {@code run}, {@code runTo}, {@code near},
 * {@code max}, {@code min}, ...) live on {@link yams.commands2.mechanisms.Arm}, which extends this
 * class and has a full usage example in its Javadoc.
 */
public class Arm extends SmartPositionalMechanism {
  /**
   * Arm config.
   */
  private final ArmConfig m_config;
  /**
   * Simulation for the arm.
   */
  private Optional<SingleJointedArmSim> m_sim = Optional.empty();
  /**
   * Mechanism ligament for the setpoint.
   */
  private MechanismLigament2d m_setpointLigament = null;

  /**
   * Constructor for the Arm mechanism.
   *
   * @param config {@link ArmConfig} to use.
   * @param smc    {@link SmartMotorController} for the Arm.
   * @implNote Protected so only {@link yams.commands2.mechanisms.Arm} can construct this.
   */
  protected Arm(ArmConfig config, SmartMotorController smc) {
    this.m_config = config;
    m_smc = smc;
    SmartMotorControllerConfig smccfg = smc.getConfig();
    DCMotor dcmotor = m_smc.getDCMotor();
    MechanismGearing gearing = m_smc.getConfig().getGearing();
    // Seed the relative encoder
    if (m_smc.getConfig().getExternalEncoder().isPresent()) {
      m_smc.seedRelativeEncoder();
    }
    if (config.getTelemetryName().isPresent()) {
      // TODO: Add telemetry units to config.
      m_telemetry.setupTelemetry(getName(), m_smc);
    }

    if (RobotBase.isSimulation()) {
      if (config.getLength().isEmpty()) {
        throw new ArmConfigurationException("Arm Length is empty", "Cannot create simulation.", "withLength(Distance)");
      }
      if (config.getLowerHardLimit().isEmpty()) {
        throw new ArmConfigurationException("Arm lower hard limit is empty", "Cannot create simulation.", "withHardLimits(Angle,Angle)");
      }
      if (config.getUpperHardLimit().isEmpty()) {
        throw new ArmConfigurationException("Arm upper hard limit is empty", "Cannot create simulation.", "withHardLimits(Angle,Angle)");
      }
      if (smccfg.getStartingPosition().isEmpty() && smccfg.getExternalEncoderZeroOffset().isEmpty()) {
        throw new ArmConfigurationException("Arm starting angle is empty", "Cannot create simulation.", "SmartMotorControllerConfig.withStartingPosition(Angle)");
      }
      if (smccfg.getStartingPosition().isPresent() && (smccfg.getStartingPosition().get().lt(config.getLowerHardLimit().get()) || smccfg.getStartingPosition().get().gt(config.getUpperHardLimit().get()))) {
        throw new ArmConfigurationException("Arm starting angle is outside hard limits", "Cannot create simulation.", "SmartMotorControllerConfig.withStartingPosition(Angle)");
      }
      m_sim = Optional.of(new SingleJointedArmSim(smc.getDCMotor(), smccfg.getGearing().getMechanismToRotorRatio(), smccfg.getMOI(), config.getLength().get().in(Meters), config.getLowerHardLimit().get().in(Radians), config.getUpperHardLimit().get()
          .in(Radians), true, smccfg.getStartingPosition().orElse(Rotations.zero()).in(Radians), 0.002 / 4096.0, 0.0)); // Add noise with a std-dev of 1 tick
      m_smc.setSimSupplier(new ArmSimSupplier(m_sim.get(), m_smc));

      m_mechanismWindow = new Mechanism2d(config.getMechanismPositionConfig().getWindowXDimension(config.getLength().get()).in(Meters), config.getMechanismPositionConfig().getWindowYDimension(config.getLength().get()).in(Meters));
      m_mechanismRoot = m_mechanismWindow.getRoot(getName() + "Root", config.getMechanismPositionConfig().getMechanismX(config.getLength().get()).in(Meters) + config.getMechanismPositionConfig().getRelativePosition().orElse(new Translation3d())
          .getX(), config.getMechanismPositionConfig().getMechanismY(config.getLength().get()).in(Meters) + config.getMechanismPositionConfig().getRelativePosition().orElse(new Translation3d()).getZ());

      m_mechanismLigament = m_mechanismRoot.append(new MechanismLigament2d(getName(), config.getLength().get().in(Meters), smccfg.getStartingPosition().orElse(Rotations.zero()).in(Degrees), 6, config.getSimColor()));
      m_setpointLigament = m_mechanismRoot.append(new MechanismLigament2d("Setpoint", config.getLength().get().in(Meters), smccfg.getStartingPosition().orElse(Rotations.zero()).in(Degrees), 3, new Color8Bit(Color.WHITE)));
      m_mechanismRoot.append(new MechanismLigament2d("MaxHard", Inch.of(3).in(Meters), config.getUpperHardLimit().get().in(Degrees), 4, new Color8Bit(Color.LIME_GREEN)));
      m_mechanismRoot.append(new MechanismLigament2d("MinHard", Inch.of(3).in(Meters), config.getLowerHardLimit().get().in(Degrees), 4, new Color8Bit(Color.RED)));
      if (smccfg.getMechanismLowerLimit().isPresent() && smccfg.getMechanismUpperLimit().isPresent()) {
        m_mechanismRoot.append(new MechanismLigament2d("MaxSoft", Inch.of(3).in(Meters), smccfg.getMechanismUpperLimit().get().in(Degrees), 4, new Color8Bit(Color.HOT_PINK)));
        m_mechanismRoot.append(new MechanismLigament2d("MinSoft", Inch.of(3).in(Meters), smccfg.getMechanismLowerLimit().get().in(Degrees), 4, new Color8Bit(Color.YELLOW)));
      }
      publishMechanismWindow();
      m_smc.setupSimulation();
    }
  }

  @Override
  public void updateTelemetry() {
    //    m_telemetry.updatePosition(getAngle());
    //    m_motor.getMechanismPositionSetpoint().ifPresent(m_setpoint ->
    // m_telemetry.updateSetpoint(m_setpoint));
    m_smc.updateTelemetry();
    m_telemetry.updateLoopTime();
  }

  @Override
  public void simIterate() {
    if (m_sim.isPresent() && m_smc.getSimSupplier().isPresent()) {
      m_smc.getSimSupplier().get().updateSimState();
      m_smc.simIterate();
      m_smc.getSimSupplier().get().starveUpdateSim();
      if (m_config.getLowerHardLimit().isPresent() && m_sim.get().getVelocity() < 0 && m_smc.getMechanismPosition().lt(m_config.getLowerHardLimit().get())) {
        m_smc.setEncoderPosition(m_config.getLowerHardLimit().get());
      }
      if (m_config.getUpperHardLimit().isPresent() && m_sim.get().getVelocity() > 0 && m_smc.getMechanismPosition().gt(m_config.getUpperHardLimit().get())) {
        m_smc.setEncoderPosition(m_config.getUpperHardLimit().get());
      }
      RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_sim.get().getCurrentDraw()));
      visualizationUpdate();
    }
  }

  /**
   * Updates the mechanism ligament with the current angle of the arm.
   *
   * @see SmartPositionalMechanism#visualizationUpdate()
   */
  @Override
  public void visualizationUpdate() {
    m_mechanismLigament.setAngle(getAngle().in(Degrees));
    m_setpointLigament.setAngle(m_smc.getMechanismPositionSetpoint().orElse(getAngle()).in(Degrees));
  }

  /**
   * Get the relative position of the mechanism, taking into account the relative position defined
   * in the {@link MechanismPositionConfig}.
   *
   * @return The relative position of the mechanism as a {@link Translation3d}.
   */
  @Override
  public Translation3d getRelativeMechanismPosition() {
    Plane movementPlane = m_config.getMechanismPositionConfig().getMovementPlane();
    Translation3d mechanismTranslation = new Translation3d(m_mechanismLigament.getLength(), new Rotation3d(Plane.YZ == movementPlane ? m_mechanismLigament.getAngle() : 0, Plane.XZ == movementPlane ? m_mechanismLigament.getAngle() : 0, 0));
    if (m_config.getMechanismPositionConfig().getRelativePosition().isPresent()) {
      return m_config.getMechanismPositionConfig().getRelativePosition().get().plus(mechanismTranslation);
    }
    return mechanismTranslation;
  }

  @Override
  public String getName() {
    return m_config.getTelemetryName().orElse("Arm");
  }

  /**
   * Get the {@link SmartMotorController} Mechanism Position representing the arm.
   *
   * @return Arm {@link Angle}
   */
  public Angle getAngle() {
    return m_smc.getMechanismPosition();
  }

  /**
   * Arm is near an angle.
   *
   * @param angle  {@link Angle} to be near.
   * @param within {@link Angle} within.
   * @return True if the arm is near the given angle.
   */
  public boolean isNear(Angle angle, Angle within) {
    return getAngle().isNear(angle, within);
  }

  @Override
  public boolean isAtMax() {
    if (m_smc.getConfig().getMechanismUpperLimit().isPresent()) {
      return isGte(m_smc.getConfig().getMechanismUpperLimit().get());
    }
    if (m_config.getUpperHardLimit().isPresent()) {
      return isGte(m_config.getUpperHardLimit().get());
    }
    throw new ArmConfigurationException("Arm upper hard and motor controller soft limit is empty", "Cannot create max trigger.", "withHardLimits(Angle,Angle)");
  }

  @Override
  public boolean isAtMin() {
    if (m_smc.getConfig().getMechanismLowerLimit().isPresent()) {
      return isLte(m_smc.getConfig().getMechanismLowerLimit().get());
    }
    if (m_config.getLowerHardLimit().isPresent()) {
      return isLte(m_config.getLowerHardLimit().get());
    }
    throw new ArmConfigurationException("Arm lower hard and motor controller soft limit is empty", "Cannot create min trigger.", "withHardLimits(Angle,Angle)");
  }

  /**
   * Between two angles.
   *
   * @param start Start angle.
   * @param end   End angle
   * @return True if the arm is between the given angles.
   */
  public boolean isBetween(Angle start, Angle end) {
    return isGte(start) && isLte(end);
  }

  /**
   * Less than or equal to angle
   *
   * @param angle {@link Angle} to check against
   * @return True if the arm's angle is less than or equal to the given angle.
   */
  public boolean isLte(Angle angle) {
    return getAngle().lte(angle);
  }

  /**
   * Greater than or equal to angle.
   *
   * @param angle Angle to check against.
   * @return True if the arm's angle is greater than or equal to the given angle.
   */
  public boolean isGte(Angle angle) {
    return getAngle().gte(angle);
  }

  /**
   * Get the {@link ArmConfig} for this {@link Arm}.
   *
   * @return The {@link ArmConfig} used to configure this {@link Arm}.
   */
  public ArmConfig getArmConfig() {
    return m_config;
  }
}
