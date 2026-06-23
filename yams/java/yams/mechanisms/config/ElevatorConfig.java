// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechanisms.config;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.Optional;
import java.util.OptionalInt;
import yams.exceptions.SmartMotorControllerConfigurationException;
import yams.mechanisms.positional.Elevator;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

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
public class ElevatorConfig
{
  /**
   * Telemetry name.
   */
  private   Optional<String>                   telemetryName           = Optional.empty();
  /**
   * Telemetry verbosity
   */
  private   Optional<TelemetryVerbosity>       telemetryVerbosity      = Optional.empty();
  /**
   * Lower Hard Limit for the {@link Elevator} to be representing in simulation.
   */
  private   Optional<Distance>                 lowerHardLimit          = Optional.empty();
  /**
   * Upper hard limit for the {@link Elevator} representing in simulation.
   */
  private   Optional<Distance>                 upperHardLimit          = Optional.empty();
  /**
   * {@link Elevator} angle for simulation.
   */
  private   Angle                              angle                   = Degrees.of(90);
  /**
   * {@link Elevator} carriage mass for simulation.
   */
  private   Optional<Mass>                     carriageWeight          = Optional.empty();
  /**
   * Sim color value
   */
  private   Color8Bit                          simColor                = new Color8Bit(Color.kOrange);
  /**
   * Mechanism position configuration for the {@link Pivot} (Optional).
   */
  private   MechanismPositionConfig            mechanismPositionConfig = new MechanismPositionConfig();
  /**
   * Drum radius of the elevator spool, or the sprocket pitch * teeth.
   */
  private   Optional<Distance>                 drumCircumference       = Optional.empty();
  /**
   * Elevator stages, applied to the motor controller config gearing by dividing it by the number of stages given.
   */
  private   OptionalInt                        stages                  = OptionalInt.empty();
  /**
   * Disable gravity on the elevator simulation.
   */
  private   boolean                            isElevatorHorizontal    = false;

  /**
   * Elevator Configuration class
   *
   */
  public ElevatorConfig() {}

  /**
   * Copy constructor.
   *
   * @param cfg Configuration to copy from.
   */
  private ElevatorConfig(ElevatorConfig cfg)
  {
    this.isElevatorHorizontal = cfg.isElevatorHorizontal;
    this.drumCircumference = cfg.drumCircumference;
    this.stages = cfg.stages;
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
  public ElevatorConfig clone()
  {
    return new ElevatorConfig(this);
  }

  /**
   * Publish the color in sim as this.
   *
   * @param simColor {@link Color8Bit} to show.
   * @return {@link ElevatorConfig} for chaining.
   */
  public ElevatorConfig withSimColor(final Color8Bit simColor)
  {
    this.simColor = simColor;
    return this;
  }

  /**
   * Configure the {@link Elevator}s angle for simulation.
   *
   * @param angle Angle of the {@link Elevator}.
   * @return {@link ElevatorConfig} for chaining.
   */
  public ElevatorConfig withAngle(Angle angle)
  {
    this.angle = angle;
    return this;
  }

  /**
   * Configure the {@link Elevator}s {@link Mass} for simulation.
   *
   * @param mass {@link Mass} of the {@link Elevator}
   * @return {@link ElevatorConfig} for chaining.
   */
  public ElevatorConfig withCarriageWeight(Mass mass)
  {
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
  public ElevatorConfig withTelemetry(String telemetryName, TelemetryVerbosity telemetryVerbosity)
  {
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
  public ElevatorConfig withMechanismPositionConfig(MechanismPositionConfig mechanismPositionConfig)
  {
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
  public ElevatorConfig withHardLimits(Distance min, Distance max)
  {
    lowerHardLimit = Optional.ofNullable(min);
    upperHardLimit = Optional.ofNullable(max);
    return this;
  }

  /**
   * Set elevator as horizontal to avoid gravity simulation
   *
   * @return {@link ElevatorConfig} for chaining.
   */
  public ElevatorConfig withHorizontalElevator()
  {
    isElevatorHorizontal = true;
    return this;
  }

  /**
   * Get the Angle of the {@link Elevator}
   *
   * @return {@link Angle} of the Elevator.
   */
  public Angle getAngle()
  {
    return angle;
  }

  /**
   * Get the Upper hard limit of the {@link Elevator}.
   *
   * @return {@link Distance} hard limit.
   */
  public Optional<Distance> getMaximumHeight()
  {
    return upperHardLimit;
  }

  /**
   * Get the lower hard limit of the {@link Elevator}
   *
   * @return {@link Distance} hard limit.
   */
  public Optional<Distance> getMinimumHeight()
  {
    return lowerHardLimit;
  }

  /**
   * Get the telemetry verbosity of the {@link Elevator}
   *
   * @return {@link TelemetryVerbosity} of the {@link Elevator}
   */
  public Optional<TelemetryVerbosity> getTelemetryVerbosity()
  {
    return telemetryVerbosity;
  }

  /**
   * Network Tables name for the {@link Elevator}
   *
   * @return Network Tables name.
   */
  public Optional<String> getTelemetryName()
  {
    return telemetryName;
  }

  /**
   * Get sim color
   *
   * @return sim color
   */
  public Color8Bit getSimColor()
  {
    return simColor;
  }

  /**
   * Get the {@link Mass} of the {@link Elevator} carriage.
   *
   * @return Carriage mass.
   */
  public Optional<Mass> getCarriageMass()
  {
    return carriageWeight;
  }

  /**
   * Get if the elevator is horizontal
   *
   * @return if elevator is horizontal.
   */
  public boolean getIsElevatorHorizontal()
  {
    return isElevatorHorizontal;
  }

  /**
   * Get the mechanism position configuration of the elevator.
   *
   * @return Optional containing the mechanism position configuration if set, otherwise an empty Optional.
   */
  public MechanismPositionConfig getMechanismPositionConfig()
  {
    return mechanismPositionConfig;
  }

}
