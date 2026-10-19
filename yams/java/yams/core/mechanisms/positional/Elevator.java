// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.positional;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.simulation.ElevatorSim;
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
  private final ElevatorConfig  m_config;
  /**
   * Simulation for the elevator
   */
  private Optional<ElevatorSim> m_sim              = Optional.empty();
  /**
   * Mechanism ligament for the setpoint.
   */
  private MechanismLigament2d   m_setpointLigament = null;

  /**
   * Construct the {@link Elevator} class for easy manipulation of an elevator.
   *
   * @param config {@link ElevatorConfig} to set.
   * @param smc    {@link SmartMotorController} to use for the Elevator
   * @implNote Protected so only {@link yams.commands2.mechanisms.Elevator} can construct this.
   * @throws ElevatorConfigurationException             if running in simulation and the carriage
   *                                                    mass is not configured.
   * @throws ElevatorConfigurationException             if running in simulation and the minimum or
   *                                                    maximum height is not configured.
   * @throws ElevatorConfigurationException             if the stage heights do not add up to the
   *                                                    maximum height.
   * @throws SmartMotorControllerConfigurationException if running in simulation and the starting
   *                                                    position is not set on the
   *                                                    {@link SmartMotorControllerConfig}.
   * @throws SmartMotorControllerConfigurationException if running in simulation and the mechanism
   *                                                    circumference is not configured.
   * @throws SmartMotorControllerConfigurationException if running in simulation and the starting
   *                                                    height is outside the minimum and maximum
   *                                                    heights.
   */
  protected Elevator(ElevatorConfig config, SmartMotorController smc) {
    m_config = config;
    m_smc = smc;
    SmartMotorControllerConfig<?> smcCfg = smc.getConfig();
    SmartMotorControllerConfig<?> smcConfig = m_smc.getConfig();
    if (config.getTelemetryName().isPresent()) {
      // TODO: Add telemetry units to config.
      m_telemetry.setupTelemetry(getName(), m_smc);
    }

    // The stages travel the elevator's whole height together, so their heights must add up to it.
    // Without hard limits, the maximum height is the stage heights' total.
    if (config.getStageHeightTotal().isPresent() && config.getMaximumHeight().isPresent()) {
      Distance total = config.getStageHeightTotal().get();
      Distance maximum = config.getMaximumHeight().get();
      if (Math.abs(total.in(Meters) - maximum.in(Meters)) > 1e-6) {
        throw new ElevatorConfigurationException("Elevator stage heights add up to " + total.in(Meters) + " m, not the maximum height of " + maximum.in(Meters) + " m",
                                                 "Cannot create elevator", "withStages(Distance...) with heights that add up to the maximum height");
      }
    }

    if (RobotBase.isSimulation()) {
      smc.setupSimulation();
      if (config.getCarriageMass().isEmpty()) {
        throw new ElevatorConfigurationException("Mass is not configured!", "Cannot create simulator", "withCarriageWeight(Mass)");
      }
      if (config.getMinimumHeight().isEmpty()) {
        throw new ElevatorConfigurationException("Minimum height is not configured!", "Cannot create simulator", "withHardLimits(Distance,Distance)");
      }
      if (config.getMaximumHeight().isEmpty()) {
        throw new ElevatorConfigurationException("Maximum height is not configured!", "Cannot create simulator", "withHardLimits(Distance,Distance)");
      }
      if (smcConfig.getStartingPosition().isEmpty()) {
        throw new SmartMotorControllerConfigurationException("Starting height is not configured!", "Cannot create simulator", "withStartingPosition(Distance)");
      }
      if (smcConfig.convertFromMechanism(smcConfig.getStartingPosition().orElseThrow()).lt(config.getMinimumHeight().get()) || smcConfig.convertFromMechanism(smcConfig.getStartingPosition().orElseThrow()).gt(config.getMaximumHeight().get())) {
        throw new SmartMotorControllerConfigurationException("Elevator starting height is outside hard limits", "Cannot create simulator", "withStartingPosition(Distance)");
      }

      boolean simulateGravity = !config.getIsElevatorHorizontal();

      m_sim = Optional.of(new ElevatorSim(smc.getDCMotor(), smcConfig.getGearing().getMechanismToRotorRatio(), config.getCarriageMass().get().in(Kilograms), smcCfg.getMechanismCircumference().orElseThrow().div(2).div(Math.PI).in(Meters), config
          .getMinimumHeight().get().in(Meters), config.getMaximumHeight().get().in(Meters), simulateGravity, smcConfig.convertFromMechanism(smcConfig.getStartingPosition().orElseThrow()).in(Meters), 0.01 / 4096, 0.01 / 4096));
      m_smc.setSimSupplier(new ElevatorSimSupplier(m_sim.get(), m_smc));
      m_mechanismWindow = new Mechanism2d(config.getMaximumHeight().get().in(Meters) * 2, config.getMaximumHeight().get().in(Meters) * 2);

      m_mechanismRoot = m_mechanismWindow.getRoot(getName() + "Root", config.getMaximumHeight().get().in(Meters) - config.getMechanismPositionConfig().getRelativePosition().orElse(new Translation3d()).getX(), config.getMechanismPositionConfig()
          .getRelativePosition().orElse(new Translation3d()).getZ());

      if (m_smc.getConfig().getMechanismLowerLimit().isPresent()) {
        m_mechanismWindow.getRoot("MinSoft", config.getMaximumHeight().get().in(Meters) + config.getMechanismPositionConfig().getRelativePosition().orElse(new Translation3d()).getX() - Inches.of(6).in(Meters), config.getMechanismPositionConfig()
            .getRelativePosition().orElse(new Translation3d()).getZ()).append(new MechanismLigament2d("Limit", m_smc.getConfig().convertFromMechanism(m_smc.getConfig().getMechanismLowerLimit().get()).in(Meters), config.getAngle().in(Degrees), 3,
                new Color8Bit(Color.YELLOW)));
      }
      if (m_smc.getConfig().getMechanismUpperLimit().isPresent()) {
        m_mechanismWindow.getRoot("MaxSoft", config.getMaximumHeight().get().in(Meters) + config.getMechanismPositionConfig().getRelativePosition().orElse(new Translation3d()).getX() - Inches.of(6).in(Meters), config.getMechanismPositionConfig()
            .getRelativePosition().orElse(new Translation3d()).getZ()).append(new MechanismLigament2d("Limit", m_smc.getConfig().convertFromMechanism(m_smc.getConfig().getMechanismUpperLimit().get()).in(Meters), config.getAngle().in(Degrees), 3,
                new Color8Bit(Color.HOT_PINK)));
      }
      m_mechanismWindow.getRoot("MinHard", config.getMaximumHeight().get().in(Meters) + config.getMechanismPositionConfig().getRelativePosition().orElse(new Translation3d()).getX() - Inches.of(8).in(Meters), config.getMechanismPositionConfig()
          .getRelativePosition().orElse(new Translation3d()).getZ()).append(new MechanismLigament2d("Limit", config.getMinimumHeight().get().in(Meters), config.getAngle().in(Degrees), 3, new Color8Bit(Color.RED)));
      m_mechanismWindow.getRoot("MaxHard", config.getMaximumHeight().get().in(Meters) + config.getMechanismPositionConfig().getRelativePosition().orElse(new Translation3d()).getX() - Inches.of(8).in(Meters), config.getMechanismPositionConfig()
          .getRelativePosition().orElse(new Translation3d()).getZ()).append(new MechanismLigament2d("Limit", config.getMaximumHeight().get().in(Meters), config.getAngle().in(Degrees), 3, new Color8Bit(Color.LIME_GREEN)));

      m_mechanismLigament = m_mechanismRoot.append(new MechanismLigament2d(getName(), smcConfig.convertFromMechanism(smcConfig.getStartingPosition().orElseThrow()).in(Meters), config.getAngle().in(Degrees), 6, config.getSimColor()));
      m_setpointLigament = m_mechanismRoot.append(new MechanismLigament2d("Setpoint", smcConfig.convertFromMechanism(smcConfig.getStartingPosition().orElseThrow()).in(Meters), config.getAngle().in(Degrees), 3, new Color8Bit(Color.WHITE)));
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
      visualizationUpdate();
    }
  }

  /**
   * Updates the length of the mechanism ligament to match the current height of the elevator in
   * meters.
   *
   * @throws SmartMotorControllerConfigurationException if a mechanism position setpoint is present
   *                                                    and the mechanism circumference is not
   *                                                    configured.
   */
  @Override
  public void visualizationUpdate() {
    m_mechanismLigament.setLength(getHeight().in(Meters));
    if (getMotor().getMechanismPositionSetpoint().isPresent()) {
      m_setpointLigament.setLength(m_smc.getConfig().convertFromMechanism(getMotor().getMechanismPositionSetpoint().get()).in(Meters));
    }
  }

  /**
   * Get the carriage's pose, for publishing to AdvantageScope as a component of a robot model: where the
   * elevator is mounted, as set in its {@link MechanismPositionConfig}, moved up the elevator by its
   * height, at the elevator's angle in its movement plane.
   *
   * @return The carriage's {@link Pose3d}, relative to the robot.
   */
  @Override
  public Pose3d getPose3d() {
    return m_config.getMechanismPositionConfig().getTranslatedPose(getHeight(), m_config.getAngle());
  }

  /**
   * Get the pose of each moving stage of a cascade elevator, for publishing to AdvantageScope as
   * components of a robot model. A cascade elevator's stages all move at once, each by its share of the
   * carriage's height: a stage that travels a third of the elevator's total travel moves a third of the
   * carriage's height relative to the stage below it.
   *
   * <p>The stages are set with {@link ElevatorConfig#withStages(Distance...)}.
   *
   * @return Each moving stage's {@link Pose3d}, relative to the robot, from the stage nearest the fixed
   *         stage to the carriage. The last is the carriage, {@link #getPose3d()}.
   * @throws ElevatorConfigurationException if the stages are not set, or there are fewer than 2.
   */
  public List<Pose3d> getCascadePose3d() {
    List<Distance> stageHeights = m_config.getStageHeights();
    if (stageHeights.size() < 2) {
      // An elevator with only a carriage has no stages to cascade or move in turn: use getPose3d().
      throw new ElevatorConfigurationException(stageHeights.isEmpty() ? "Elevator stages are not configured" : "Elevator has only 1 moving stage, its carriage",
                                               "Cannot get the pose of each stage. Use getPose3d() for the carriage.", "withStages(Distance...) with 2 or more stages");
    }
    double total = m_config.getStageHeightTotal().orElseThrow().in(Meters);
    double height = getHeight().in(Meters);
    List<Pose3d> poses = new ArrayList<>(stageHeights.size());
    double stageHeight = 0;
    for (Distance travel : stageHeights) {
      // Each stage moves its share of the carriage's height, on top of the stage below it.
      stageHeight += height * travel.in(Meters) / total;
      poses.add(m_config.getMechanismPositionConfig().getTranslatedPose(Meters.of(stageHeight), m_config.getAngle()));
    }
    return poses;
  }

  /**
   * Get the pose of each moving stage of a continuous elevator, for publishing to AdvantageScope as
   * components of a robot model. A continuous elevator's stages move one at a time: the carriage rises
   * the whole of its travel first, then the stage below it, and so on out to the stage nearest the fixed
   * stage.
   *
   * <p>The stages are set with {@link ElevatorConfig#withStages(Distance...)}.
   *
   * @return Each moving stage's {@link Pose3d}, relative to the robot, from the stage nearest the fixed
   *         stage to the carriage. The last is the carriage, {@link #getPose3d()}.
   * @throws ElevatorConfigurationException if the stages are not set, or there are fewer than 2.
   */
  public List<Pose3d> getContinuousPose3d() {
    List<Distance> stageHeights = m_config.getStageHeights();
    if (stageHeights.size() < 2) {
      // An elevator with only a carriage has no stages to cascade or move in turn: use getPose3d().
      throw new ElevatorConfigurationException(stageHeights.isEmpty() ? "Elevator stages are not configured" : "Elevator has only 1 moving stage, its carriage",
                                               "Cannot get the pose of each stage. Use getPose3d() for the carriage.", "withStages(Distance...) with 2 or more stages");
    }
    int stages = stageHeights.size();
    double[] extension = new double[stages];
    double remaining = getHeight().in(Meters);
    // From the carriage down, each stage takes as much of the height as it can travel. The stage
    // nearest the fixed stage takes the rest, and the carriage any height below zero, so the carriage
    // is always at the elevator's height.
    for (int stage = stages - 1; stage > 0; stage--) {
      double travel = stageHeights.get(stage).in(Meters);
      extension[stage] = stage == stages - 1 ? Math.min(remaining, travel) : Math.min(Math.max(remaining, 0), travel);
      remaining -= extension[stage];
    }
    extension[0] += remaining;
    List<Pose3d> poses = new ArrayList<>(stages);
    double stageHeight = 0;
    for (double stageExtension : extension) {
      stageHeight += stageExtension;
      poses.add(m_config.getMechanismPositionConfig().getTranslatedPose(Meters.of(stageHeight), m_config.getAngle()));
    }
    return poses;
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

  /**
   * Whether the elevator is at or above its maximum height, defined by the motor controller upper
   * soft limit if present, otherwise the elevator maximum height.
   *
   * @return True if the elevator is at its configured maximum.
   * @throws ElevatorConfigurationException             if neither a motor controller upper soft
   *                                                    limit nor an elevator maximum height is
   *                                                    configured.
   * @throws SmartMotorControllerConfigurationException if a motor controller upper soft limit is
   *                                                    configured but the mechanism circumference
   *                                                    is not.
   */
  @Override
  public boolean isAtMax() {
    if (m_smc.getConfig().getMechanismUpperLimit().isPresent()) {
      return isGte(m_smc.getConfig().convertFromMechanism(m_smc.getConfig().getMechanismUpperLimit().get()));
    }
    if (m_config.getMaximumHeight().isPresent()) {
      return isGte(m_config.getMaximumHeight().get());
    }
    throw new ElevatorConfigurationException("Maximum height is not configured!", "Cannot create max trigger.", "withHardLimits(Distance,Distance)");
  }

  /**
   * Whether the elevator is at or below its minimum height, defined by the motor controller lower
   * soft limit if present, otherwise the elevator minimum height.
   *
   * @return True if the elevator is at its configured minimum.
   * @throws ElevatorConfigurationException             if neither a motor controller lower soft
   *                                                    limit nor an elevator minimum height is
   *                                                    configured.
   * @throws SmartMotorControllerConfigurationException if a motor controller lower soft limit is
   *                                                    configured but the mechanism circumference
   *                                                    is not.
   */
  @Override
  public boolean isAtMin() {
    if (m_smc.getConfig().getMechanismLowerLimit().isPresent()) {
      return isLte(m_smc.getConfig().convertFromMechanism(m_smc.getConfig().getMechanismLowerLimit().get()));
    }
    if (m_config.getMinimumHeight().isPresent()) {
      return isLte(m_config.getMinimumHeight().get());
    }
    throw new ElevatorConfigurationException("Minimum height is not configured!", "Cannot create min trigger.", "withHardLimits(Distance,Distance)");
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
