// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechanisms.config;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.Optional;
import yams.exceptions.PivotConfigurationException;
import yams.mechanisms.config.MechanismPositionConfig.Plane;
import yams.mechanisms.positional.Elevator;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * Pivot configuration class.
 *
 * <h2>Configuration Example</h2>
 * <pre>{@code
 * // Create the motor controller configuration with PID and feedforward gains
 * SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig()
 *     .withClosedLoopController(0.15,0,0)
 *     .withFeedforward(new SimpleMotorFeedforward(0.004, 0.05, 0.2)
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
 * // Build the pivot config and mechanism
 * PivotConfig config = new PivotConfig()
 *     .withHardLimits(Degrees.of(0), Degrees.of(60))
 *     .withTelemetry("ShooterHood", TelemetryVerbosity.HIGH);
 *
 * Pivot pivot = new Pivot(config, motor);
 * }</pre>
 */
public class PivotConfig
{
  /**
   * Telemetry name.
   */
  private   Optional<String>               telemetryName           = Optional.empty();
  /**
   * Telemetry verbosity
   */
  private   Optional<TelemetryVerbosity>   telemetryVerbosity      = Optional.empty();
  /**
   * Lower Hard Limit for the {@link Pivot} to be representing in simulation.
   */
  private   Optional<Angle>                lowerHardLimit          = Optional.empty();
  /**
   * Upper hard limit for the {@link Pivot} representing in simulation.
   */
  private   Optional<Angle>                upperHardLimit          = Optional.empty();
  /**
   * Sim color value
   */
  private   Color8Bit                      simColor                = new Color8Bit(Color.kOrange);
  /**
   * Mechanism position configuration for the {@link Pivot}
   */
  private   MechanismPositionConfig        mechanismPositionConfig = new MechanismPositionConfig();

  /**
   * Pivot Configuration class
   * @implNote Defaults to XY plane.
   */
  public PivotConfig()
  {
    mechanismPositionConfig.withMovementPlane(Plane.XY);
  }

  /**
   * Copy constructor.
   *
   * @param cfg Config to copy.
   */
  public PivotConfig(PivotConfig cfg)
  {
    this.telemetryName = cfg.telemetryName;
    this.telemetryVerbosity = cfg.telemetryVerbosity;
    this.lowerHardLimit = cfg.lowerHardLimit;
    this.upperHardLimit = cfg.upperHardLimit;
    this.simColor = cfg.simColor;
    this.mechanismPositionConfig = cfg.mechanismPositionConfig;
  }

  @Override
  public PivotConfig clone()
  {
    return new PivotConfig(this);
  }

  /**
   * Publish the color in sim as this.
   *
   * @param simColor {@link Color8Bit} to show.
   * @return {@link PivotConfig} for chaining.
   */
  public PivotConfig withSimColor(final Color8Bit simColor)
  {
    this.simColor = simColor;
    return this;
  }

  /**
   * Configure telemetry for the {@link Pivot} mechanism.
   *
   * @param telemetryName      Telemetry NetworkTable name to appear under "SmartDashboard/"
   * @param telemetryVerbosity Telemetry verbosity to apply.
   * @return {@link PivotConfig} for chaining.
   */
  public PivotConfig withTelemetry(String telemetryName, TelemetryVerbosity telemetryVerbosity)
  {
    this.telemetryName = Optional.ofNullable(telemetryName);
    this.telemetryVerbosity = Optional.ofNullable(telemetryVerbosity);
    return this;
  }

  /**
   * Set the elevator mechanism position configuration.
   *
   * @param mechanismPositionConfig {@link MechanismPositionConfig} for the {@link Elevator}
   * @return {@link PivotConfig} for chaining
   */
  public PivotConfig withMechanismPositionConfig(MechanismPositionConfig mechanismPositionConfig)
  {
    this.mechanismPositionConfig = mechanismPositionConfig;
    return this;
  }

  /**
   * Set the Hard Limits for simulation
   *
   * @param min Angle where the physical stop appears.
   * @param max Angle where the physical stop appears
   * @return {@link PivotConfig} for chaining.
   */
  public PivotConfig withHardLimits(Angle min, Angle max)
  {
    lowerHardLimit = Optional.ofNullable(min);
    upperHardLimit = Optional.ofNullable(max);
    return this;
  }

  /**
   * Get the Upper hard limit of the {@link Pivot}.
   *
   * @return {@link Angle} hard limit.
   */
  public Optional<Angle> getUpperHardLimit()
  {
    return upperHardLimit;
  }

  /**
   * Get the lower hard limit of the {@link Pivot}
   *
   * @return {@link Angle} hard limit.
   */
  public Optional<Angle> getLowerHardLimit()
  {
    return lowerHardLimit;
  }

  /**
   * Get the telemetry verbosity of the {@link Pivot}
   *
   * @return {@link TelemetryVerbosity} of the {@link Pivot}
   */
  public Optional<TelemetryVerbosity> getTelemetryVerbosity()
  {
    return telemetryVerbosity;
  }

  /**
   * Network Tables name for the {@link Pivot}
   *
   * @return Network Tables name.
   */
  public Optional<String> getTelemetryName()
  {
    return telemetryName;
  }

  /**
   * Sim color value
   *
   * @return Sim color value
   */
  public Color8Bit getSimColor()
  {
    return simColor;
  }


  /**
   * Get the {@link MechanismPositionConfig} associated with this {@link PivotConfig}.
   *
   * @return An {@link Optional} containing the {@link MechanismPositionConfig} if present, otherwise an empty
   * {@link Optional}.
   */
  public MechanismPositionConfig getMechanismPositionConfig()
  {
    return mechanismPositionConfig;
  }

}
