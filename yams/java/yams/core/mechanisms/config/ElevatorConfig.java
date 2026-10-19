// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.config;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import java.util.List;
import java.util.Optional;
import java.util.OptionalInt;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Mass;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;
import yams.core.mechanisms.positional.Elevator;
import yams.core.mechanisms.positional.Pivot;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Elevator configuration class.
 *
 * <h2>Configuration Example</h2>
 * <pre>{@code
 * SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig()
 *     .withClosedLoopController(0.5,0,0)
 *     .withFeedforward(new ElevatorFeedforward(0.1,0.4,0.12)
 *     .withStatorCurrentLimit(Amps.of(40));
 * SmartMotorController motor = new TalonFXWrapper(
 *     new TalonFX(4), DCMotor.getKrakenX60(1), motorConfig);
 *
 * ElevatorConfig config = new ElevatorConfig()
 *     .withCarriageWeight(Kilograms.of(4.0))
 *     .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
 *     .withHardLimits(Meters.of(0), Meters.of(2));
 *
 * Elevator elevator = new Elevator(config, motor);
 * }</pre>
 */
public class ElevatorConfig {
  /**
   * Telemetry name.
   */
  private Optional<String>             telemetryName           = Optional.empty();
  /**
   * Telemetry verbosity
   */
  private Optional<TelemetryVerbosity> telemetryVerbosity      = Optional.empty();
  /**
   * Lower Hard Limit for the {@link Elevator} to be representing in simulation.
   */
  private Optional<Distance>           lowerHardLimit          = Optional.empty();
  /**
   * Upper hard limit for the {@link Elevator} representing in simulation.
   */
  private Optional<Distance>           upperHardLimit          = Optional.empty();
  /**
   * {@link Elevator} angle for simulation.
   */
  private Angle                        angle                   = Degrees.of(90);
  /**
   * {@link Elevator} carriage mass for simulation.
   */
  private Optional<Mass>               carriageWeight          = Optional.empty();
  /**
   * Sim color value
   */
  private Color8Bit                    simColor                = new Color8Bit(Color.ORANGE);
  /**
   * Mechanism position configuration for the {@link Pivot} (Optional).
   */
  private MechanismPositionConfig      mechanismPositionConfig = new MechanismPositionConfig();
  /**
   * Drum radius of the elevator spool, or the sprocket pitch * teeth.
   */
  private Optional<Distance>           drumCircumference       = Optional.empty();
  /**
   * The number of stages that move, counting the carriage as the last.
   */
  private OptionalInt                  stages                  = OptionalInt.empty();
  /**
   * How far each moving stage travels relative to the stage below it, from the stage nearest the fixed
   * stage to the carriage.
   */
  private List<Distance>               stageHeights            = List.of();
  /**
   * How far the stages travel together: the sum of the stage heights.
   */
  private Optional<Distance>           stageHeightTotal        = Optional.empty();
  /**
   * Disable gravity on the elevator simulation.
   */
  private boolean                      isElevatorHorizontal    = false;

  /**
   * Elevator Configuration class
   *
   */
  public ElevatorConfig() {
  }

  /**
   * Copy constructor.
   *
   * @param cfg Configuration to copy from.
   */
  private ElevatorConfig(ElevatorConfig cfg) {
    this.isElevatorHorizontal = cfg.isElevatorHorizontal;
    this.drumCircumference = cfg.drumCircumference;
    this.stages = cfg.stages;
    this.stageHeights = cfg.stageHeights;
    this.stageHeightTotal = cfg.stageHeightTotal;
    this.simColor = cfg.simColor;
    this.angle = cfg.angle;
    this.carriageWeight = cfg.carriageWeight;
    this.telemetryName = cfg.telemetryName;
    this.telemetryVerbosity = cfg.telemetryVerbosity;
    this.mechanismPositionConfig = cfg.mechanismPositionConfig;
    this.lowerHardLimit = cfg.lowerHardLimit;
    this.upperHardLimit = cfg.upperHardLimit;
  }

  @Override
  public ElevatorConfig clone() {
    return new ElevatorConfig(this);
  }

  /**
   * Publish the color in sim as this.
   *
   * @param simColor {@link Color8Bit} to show.
   * @return {@link ElevatorConfig} for chaining.
   */
  public ElevatorConfig withSimColor(final Color8Bit simColor) {
    this.simColor = simColor;
    return this;
  }

  /**
   * Configure the {@link Elevator}s angle for simulation.
   *
   * @param angle Angle of the {@link Elevator}.
   * @return {@link ElevatorConfig} for chaining.
   */
  public ElevatorConfig withAngle(Angle angle) {
    this.angle = angle;
    return this;
  }

  /**
   * Configure the {@link Elevator}s {@link Mass} for simulation.
   *
   * @param mass {@link Mass} of the {@link Elevator}
   * @return {@link ElevatorConfig} for chaining.
   */
  public ElevatorConfig withCarriageWeight(Mass mass) {
    this.carriageWeight = Optional.ofNullable(mass);
    return this;
  }

  /**
   * Configure telemetry for the {@link Elevator} mechanism.
   *
   * @param telemetryName      Telemetry NetworkTable name to appear under "SmartDashboard/"
   * @param telemetryVerbosity Telemetry verbosity to apply.
   * @return {@link ElevatorConfig} for chaining.
   */
  public ElevatorConfig withTelemetry(String telemetryName, TelemetryVerbosity telemetryVerbosity) {
    this.telemetryName = Optional.ofNullable(telemetryName);
    this.telemetryVerbosity = Optional.ofNullable(telemetryVerbosity);
    return this;
  }

  /**
   * Set the elevator mechanism position configuration.
   *
   * @param mechanismPositionConfig {@link MechanismPositionConfig} for the {@link Elevator}
   * @return {@link ElevatorConfig} for chaining
   */
  public ElevatorConfig withMechanismPositionConfig(MechanismPositionConfig mechanismPositionConfig) {
    this.mechanismPositionConfig = mechanismPositionConfig;
    return this;
  }

  /**
   * Set the Hard Limits for simulation
   *
   * @param min Height where the physical stop appears.
   * @param max Height where the physical stop appears
   * @return {@link ElevatorConfig} for chaining.
   */
  public ElevatorConfig withHardLimits(Distance min, Distance max) {
    lowerHardLimit = Optional.ofNullable(min);
    upperHardLimit = Optional.ofNullable(max);
    return this;
  }

  /**
   * Set elevator as horizontal to avoid gravity simulation
   *
   * @return {@link ElevatorConfig} for chaining.
   */
  public ElevatorConfig withHorizontalElevator() {
    isElevatorHorizontal = true;
    return this;
  }

  /**
   * Get the Angle of the {@link Elevator}
   *
   * @return {@link Angle} of the Elevator.
   */
  public Angle getAngle() {
    return angle;
  }

  /**
   * Set the elevator's moving stages, for the pose of each stage from
   * {@code Elevator.getCascadePose3d()} or {@code Elevator.getContinuousPose3d(boolean)}.
   *
   * <p>The stage heights must add up to the elevator's maximum height, from
   * {@link #withHardLimits(Distance, Distance)}: the {@code Elevator} throws if they do not. Without hard
   * limits, the elevator's height is from 0 to the stage heights' total.
   *
   * @param stageHeights How far each moving stage travels relative to the stage below it, from the
   *                     stage nearest the fixed stage to the carriage, which is the last. One for each
   *                     stage that moves.
   * @return {@link ElevatorConfig} for chaining.
   * @throws IllegalArgumentException if no height is given, or a height is not positive.
   */
  public ElevatorConfig withStages(Distance... stageHeights) {
    if (stageHeights == null || stageHeights.length == 0) {
      throw new IllegalArgumentException("An elevator has at least 1 moving stage, its carriage, but no stage heights were given.");
    }
    for (int stage = 0; stage < stageHeights.length; stage++) {
      if (!(stageHeights[stage].in(Meters) > 0)) {
        throw new IllegalArgumentException("Stage " + (stage + 1) + "'s height must be positive, but is " + stageHeights[stage] + ".");
      }
    }
    this.stages = OptionalInt.of(stageHeights.length);
    this.stageHeights = List.of(stageHeights);
    Distance total = Meters.zero();
    for (Distance stageHeight : stageHeights) {
      total = total.plus(stageHeight);
    }
    this.stageHeightTotal = Optional.of(total);
    return this;
  }

  /**
   * Get the number of stages that move, counting the carriage as the last.
   *
   * @return The number of moving stages, or empty if {@link #withStages(Distance...)} was not called.
   */
  public OptionalInt getStages() {
    return stages;
  }

  /**
   * Get how far each moving stage travels relative to the stage below it.
   *
   * @return Each moving stage's height, from the stage nearest the fixed stage to the carriage, or an
   *         empty list if {@link #withStages(Distance...)} was not called.
   */
  public List<Distance> getStageHeights() {
    return stageHeights;
  }

  /**
   * Get how far the stages travel together, the sum of the stage heights.
   *
   * @return The total of the stage heights, or empty if {@link #withStages(Distance...)} was not called.
   */
  public Optional<Distance> getStageHeightTotal() {
    return stageHeightTotal;
  }

  /**
   * Get the Upper hard limit of the {@link Elevator}: the maximum height from
   * {@link #withHardLimits(Distance, Distance)}, or the stage heights' total without hard limits, as the
   * stages carry the carriage to the top.
   *
   * @return {@link Distance} hard limit.
   */
  public Optional<Distance> getMaximumHeight() {
    return upperHardLimit.or(() -> stageHeightTotal);
  }


  /**
   * Get the lower hard limit of the {@link Elevator}: the minimum height from
   * {@link #withHardLimits(Distance, Distance)}, or 0 without hard limits when the stages are set, as the
   * stages start at the bottom.
   *
   * @return {@link Distance} hard limit.
   */
  public Optional<Distance> getMinimumHeight() {
    return lowerHardLimit.or(() -> stageHeightTotal.map(total -> Meters.zero()));
  }


  /**
   * Get the telemetry verbosity of the {@link Elevator}
   *
   * @return {@link TelemetryVerbosity} of the {@link Elevator}
   */
  public Optional<TelemetryVerbosity> getTelemetryVerbosity() {
    return telemetryVerbosity;
  }

  /**
   * Network Tables name for the {@link Elevator}
   *
   * @return Network Tables name.
   */
  public Optional<String> getTelemetryName() {
    return telemetryName;
  }

  /**
   * Get sim color
   *
   * @return sim color
   */
  public Color8Bit getSimColor() {
    return simColor;
  }

  /**
   * Get the {@link Mass} of the {@link Elevator} carriage.
   *
   * @return Carriage mass.
   */
  public Optional<Mass> getCarriageMass() {
    return carriageWeight;
  }

  /**
   * Get if the elevator is horizontal
   *
   * @return if elevator is horizontal.
   */
  public boolean getIsElevatorHorizontal() {
    return isElevatorHorizontal;
  }

  /**
   * Get the mechanism position configuration of the elevator.
   *
   * @return Optional containing the mechanism position configuration if set, otherwise an empty
   *         Optional.
   */
  public MechanismPositionConfig getMechanismPositionConfig() {
    return mechanismPositionConfig;
  }
}
