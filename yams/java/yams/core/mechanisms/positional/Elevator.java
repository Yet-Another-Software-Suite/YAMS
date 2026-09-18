// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.positional;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

import java.util.Optional;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.simulation.BatterySim;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;
import yams.core.exceptions.ElevatorConfigurationException;
import yams.core.exceptions.SmartMotorControllerConfigurationException;
import yams.core.mechanisms.config.ElevatorConfig;
import yams.core.mechanisms.config.MechanismPositionConfig;
import yams.core.mechanisms.config.MechanismPositionConfig.Plane;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.SmartMotorControllerConfig;
import yams.core.motorcontrollers.simulation.ElevatorSimSupplier;

/**
 * Elevator mechanism.
 *
 * <p>This core class holds elevator state, physics simulation, and measurement/predicate logic
 * only. Command and Trigger factories live on {@link yams.commands2.mechanisms.Elevator}, which
 * extends this class and has a full usage example in its Javadoc.
 */
public class Elevator extends SmartPositionalMechanism {
  /**
   * Config class for the elevator.
   */
  private final ElevatorConfig m_config;
  /**
   * Simulation for the elevator
   */
  private Optional<ElevatorSim> m_sim = Optional.empty();
  /**
   * Mechanism ligament for the setpoint.
   */
  private MechanismLigament2d m_setpointLigament = null;

  /**
   * Construct the {@link Elevator} class for easy manipulation of an elevator.
   *
   * @param config {@link ElevatorConfig} to set.
   * @param smc    {@link SmartMotorController} to use for the Elevator
   * @implNote Protected so only {@link yams.commands2.mechanisms.Elevator} can construct this.
   */
  protected Elevator(ElevatorConfig config, SmartMotorController smc) {
    m_config = config;
    m_smc = smc;
    SmartMotorControllerConfig smcCfg = smc.getConfig();
    SmartMotorControllerConfig smcConfig = m_smc.getConfig();
    if (config.getTelemetryName().isPresent()) {
      // TODO: Add telemetry units to config.
      m_telemetry.setupTelemetry(getName(), m_smc);
    }

    if (RobotBase.isSimulation()) {
      smc.setupSimulation();
      if (config.getCarriageMass().isEmpty()) {
        throw new ElevatorConfigurationException(
            "Mass is not configured!", "Cannot create simulator", "withCarriageWeight(Mass)");
      }
      if (config.getMinimumHeight().isEmpty()) {
        throw new ElevatorConfigurationException("Minimum height is not configured!",
            "Cannot create simulator", "withHardLimits(Distance,Distance)");
      }
      if (config.getMaximumHeight().isEmpty()) {
        throw new ElevatorConfigurationException("Maximum height is not configured!",
            "Cannot create simulator", "withHardLimits(Distance,Distance)");
      }
      if (smcConfig.getStartingPosition().isEmpty()) {
        throw new SmartMotorControllerConfigurationException("Starting height is not configured!",
            "Cannot create simulator", "withStartingPosition(Distance)");
      }
      if (smcConfig.convertFromMechanism(smcConfig.getStartingPosition().orElseThrow())
              .lt(config.getMinimumHeight().get())
          || smcConfig.convertFromMechanism(smcConfig.getStartingPosition().orElseThrow())
              .gt(config.getMaximumHeight().get())) {
        throw new SmartMotorControllerConfigurationException(
            "Elevator starting height is outside hard limits", "Cannot create simulator",
            "withStartingPosition(Distance)");
      }

      boolean simulateGravity = !config.getIsElevatorHorizontal();

      m_sim = Optional.of(new ElevatorSim(smc.getDCMotor(),
          smcConfig.getGearing().getMechanismToRotorRatio(),
          config.getCarriageMass().get().in(Kilograms),
          smcCfg.getMechanismCircumference().orElseThrow().div(2).div(Math.PI).in(Meters),
          config.getMinimumHeight().get().in(Meters), config.getMaximumHeight().get().in(Meters),
          simulateGravity,
          smcConfig.convertFromMechanism(smcConfig.getStartingPosition().orElseThrow()).in(Meters),
          0.01 / 4096, 0.01 / 4096));
      m_smc.setSimSupplier(new ElevatorSimSupplier(m_sim.get(), m_smc));
      m_mechanismWindow = new Mechanism2d(config.getMaximumHeight().get().in(Meters) * 2,
          config.getMaximumHeight().get().in(Meters) * 2);

      m_mechanismRoot = m_mechanismWindow.getRoot(getName() + "Root",
          config.getMaximumHeight()
              .get()
              .in(Meters) -config.getMechanismPositionConfig()
              .getRelativePosition()
              .orElse(new Translation3d())
              .getX(),
          config.getMechanismPositionConfig()
              .getRelativePosition()
              .orElse(new Translation3d())
              .getZ());

      if (m_smc.getConfig().getMechanismLowerLimit().isPresent()) {
        m_mechanismWindow
            .getRoot("MinSoft",
                config.getMaximumHeight().get().in(Meters)
                    + config.getMechanismPositionConfig()
                        .getRelativePosition()
                        .orElse(new Translation3d())
                        .getX()
                    - Inches.of(6).in(Meters),
                config.getMechanismPositionConfig()
                    .getRelativePosition()
                    .orElse(new Translation3d())
                    .getZ())
            .append(new MechanismLigament2d("Limit",
                m_smc.getConfig()
                    .convertFromMechanism(m_smc.getConfig().getMechanismLowerLimit().get())
                    .in(Meters),
                config.getAngle().in(Degrees), 3, new Color8Bit(Color.YELLOW)));
      }
      if (m_smc.getConfig().getMechanismUpperLimit().isPresent()) {
        m_mechanismWindow
            .getRoot("MaxSoft",
                config.getMaximumHeight().get().in(Meters)
                    + config.getMechanismPositionConfig()
                        .getRelativePosition()
                        .orElse(new Translation3d())
                        .getX()
                    - Inches.of(6).in(Meters),
                config.getMechanismPositionConfig()
                    .getRelativePosition()
                    .orElse(new Translation3d())
                    .getZ())
            .append(new MechanismLigament2d("Limit",
                m_smc.getConfig()
                    .convertFromMechanism(m_smc.getConfig().getMechanismUpperLimit().get())
                    .in(Meters),
                config.getAngle().in(Degrees), 3, new Color8Bit(Color.HOT_PINK)));
      }
      m_mechanismWindow
          .getRoot("MinHard",
              config.getMaximumHeight().get().in(Meters)
                  + config.getMechanismPositionConfig()
                      .getRelativePosition()
                      .orElse(new Translation3d())
                      .getX()
                  - Inches.of(8).in(Meters),
              config.getMechanismPositionConfig()
                  .getRelativePosition()
                  .orElse(new Translation3d())
                  .getZ())
          .append(new MechanismLigament2d("Limit", config.getMinimumHeight().get().in(Meters),
              config.getAngle().in(Degrees), 3, new Color8Bit(Color.RED)));
      m_mechanismWindow
          .getRoot("MaxHard",
              config.getMaximumHeight().get().in(Meters)
                  + config.getMechanismPositionConfig()
                      .getRelativePosition()
                      .orElse(new Translation3d())
                      .getX()
                  - Inches.of(8).in(Meters),
              config.getMechanismPositionConfig()
                  .getRelativePosition()
                  .orElse(new Translation3d())
                  .getZ())
          .append(new MechanismLigament2d("Limit", config.getMaximumHeight().get().in(Meters),
              config.getAngle().in(Degrees), 3, new Color8Bit(Color.LIME_GREEN)));

      m_mechanismLigament = m_mechanismRoot.append(new MechanismLigament2d(getName(),
          smcConfig.convertFromMechanism(smcConfig.getStartingPosition().orElseThrow()).in(Meters),
          config.getAngle().in(Degrees), 6, config.getSimColor()));
      m_setpointLigament = m_mechanismRoot.append(new MechanismLigament2d("Setpoint",
          smcConfig.convertFromMechanism(smcConfig.getStartingPosition().orElseThrow()).in(Meters),
          config.getAngle().in(Degrees), 3, new Color8Bit(Color.WHITE)));
      publishMechanismWindow();
    }
  }

  @Override
  public void updateTelemetry() {
    //    m_telemetry.updatePosition(getHeight());
    //    m_motor.getMechanismPositionSetpoint().ifPresent(m_setpoint ->
    //    m_telemetry.updateSetpoint(m_setpoint));
    m_smc.updateTelemetry();
    m_telemetry.updateLoopTime();
  }

  @Override
  public void simIterate() {
    if (m_sim.isPresent() && m_smc.getSimSupplier().isPresent()) {
      m_smc.getSimSupplier().get().updateSimState();
      m_smc.simIterate();
      m_smc.getSimSupplier().get().starveUpdateSim();
      // It is impossible for an elevator to go below the minimum height, it would break...
      if (m_config.getMinimumHeight().isPresent()
          && getHeight().lt(m_config.getMinimumHeight().get())) {
        //        m_motor.simIterate(RotationsPerSecond.of(0));
        //        m_motor.setEncoderPosition(m_config.getMinimumHeight().get());
      } else {
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(m_sim.get().getCurrentDraw()));
      }
      visualizationUpdate();
    }
  }

  /**
   * Updates the length of the mechanism ligament to match the current height of the elevator in
   * meters.
   */
  @Override
  public void visualizationUpdate() {
    m_mechanismLigament.setLength(getHeight().in(Meters));
    if (getMotor().getMechanismPositionSetpoint().isPresent()) {
      m_setpointLigament.setLength(m_smc.getConfig()
              .convertFromMechanism(getMotor().getMechanismPositionSetpoint().get())
              .in(Meters));
    }
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
    Plane movementPlane = m_config.getMechanismPositionConfig().getMovementPlane();
    Translation3d mechanismTranslation = new Translation3d(m_mechanismLigament.getLength(),
        new Rotation3d(Plane.YZ == movementPlane ? m_mechanismLigament.getAngle() : 0,
            Plane.XZ == movementPlane ? m_mechanismLigament.getAngle() : 0, 0));
    if (m_config.getMechanismPositionConfig().getRelativePosition().isPresent()) {
      return m_config.getMechanismPositionConfig().getRelativePosition().get().plus(
          mechanismTranslation);
    }
    return mechanismTranslation;
  }

  @Override
  public String getName() {
    return m_config.getTelemetryName().orElse("Elevator");
  }

  /**
   * Get the Height of the Elevator.
   *
   * @return {@link Distance} of the Elevator.
   */
  public Distance getHeight() {
    return m_smc.getMeasurementPosition();
  }

  /**
   * Get the linear velocity of the Elevator.
   *
   * @return {@link LinearVelocity} of the elevator.
   */
  public LinearVelocity getVelocity() {
    return m_smc.getMeasurementVelocity();
  }

  /**
   * Elevator is near a height.
   *
   * @param height {@link Distance} to be near.
   * @param within {@link Distance} within.
   * @return True if the elevator is near another height.
   */
  public boolean isNear(Distance height, Distance within) {
    return getHeight().isNear(height, within);
  }

  @Override
  public boolean isAtMax() {
    if (m_smc.getConfig().getMechanismUpperLimit().isPresent()) {
      return isGte(m_smc.getConfig().convertFromMechanism(
          m_smc.getConfig().getMechanismUpperLimit().get()));
    }
    if (m_config.getMaximumHeight().isPresent()) {
      return isGte(m_config.getMaximumHeight().get());
    }
    throw new ElevatorConfigurationException("Maximum height is not configured!",
        "Cannot create max trigger.", "withHardLimits(Distance,Distance)");
  }

  @Override
  public boolean isAtMin() {
    if (m_smc.getConfig().getMechanismLowerLimit().isPresent()) {
      return isLte(m_smc.getConfig().convertFromMechanism(
          m_smc.getConfig().getMechanismLowerLimit().get()));
    }
    if (m_config.getMinimumHeight().isPresent()) {
      return isLte(m_config.getMinimumHeight().get());
    }
    throw new ElevatorConfigurationException("Minimum height is not configured!",
        "Cannot create min trigger.", "withHardLimits(Distance,Distance)");
  }

  /**
   * Between two heights.
   *
   * @param start Start height.
   * @param end   End height.
   * @return True if the elevator is between the given heights.
   */
  public boolean isBetween(Distance start, Distance end) {
    return isGte(start) && isLte(end);
  }

  /**
   * Less than or equal to height
   *
   * @param height {@link Distance} to check against
   * @return True if the elevator's height is less than or equal to the given height.
   */
  public boolean isLte(Distance height) {
    return getHeight().lte(height);
  }

  /**
   * Greater than or equal to height.
   *
   * @param height Height to check against.
   * @return True if the elevator's height is greater than or equal to the given height.
   */
  public boolean isGte(Distance height) {
    return getHeight().gte(height);
  }

  /**
   * Get the {@link ElevatorConfig} for this {@link Elevator}
   *
   * @return {@link ElevatorConfig} for this {@link Elevator}
   */
  public ElevatorConfig getConfig() {
    return m_config;
  }
}
