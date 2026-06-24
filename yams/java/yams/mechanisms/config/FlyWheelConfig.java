// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechanisms.config;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.Optional;
import java.util.OptionalDouble;
import yams.exceptions.FlyWheelConfigurationException;
import yams.mechanisms.positional.Pivot;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * FlyWheel configuration class.
 *
 * <h2>Configuration Example</h2>
 * <pre>{@code
 * // 1. Build the motor config with kV and kS feedforward (critical for velocity mechanisms)
 * SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig()
 *     .withFeedforward(new SimpleMotorFeedforward(0.15, 0.12))   // kS=0.15, kV=0.12 — kV dominates at steady-state RPM
 *     .withClosedLoopController(0.0005, 0.0, 0.0) // kP
 *     .withTelemetry("Shooter", TelemetryVerbosity.HIGH);
 *
 * // 2. Create a TalonFXWrapper for a Kraken X60 on CAN ID 1
 * SmartMotorController motor = new TalonFXWrapper(new TalonFX(1), DCMotor.getKrakenX60(1), motorConfig);
 *
 * // 3. Assemble the FlyWheelConfig and mechanism
 * FlyWheelConfig config = new FlyWheelConfig()
 *     .withDiameter(Inches.of(4))
 *     .withSpeedometerSimulation(RPM.of(6000))  // optional: sim speedometer up to 6000 RPM
 *     .withTelemetry("Shooter", TelemetryVerbosity.HIGH);
 *
 * FlyWheel flywheel = new FlyWheel(config, motor);
 * }</pre>
 */
public class FlyWheelConfig
{
  /**
   * Telemetry name.
   */
  private   Optional<String>               telemetryName           = Optional.empty();
  /**
   * Telemetry verbosity
   */
  private Optional<TelemetryVerbosity> telemetryVerbosity = Optional.empty();
  /**
   * {@link FlyWheel} length for simulation.
   */
  private Optional<Distance>           diameter           = Optional.empty();
  /**
   * Sim color value
   */
  private   Color8Bit                      simColor                = new Color8Bit(Color.kOrange);
  /**
   * Mechanism position configuration for the {@link Pivot} (Optional).
   */
  private   MechanismPositionConfig        mechanismPositionConfig = new MechanismPositionConfig();
  /**
   * Use speedometer simulation for the shooter.
   */
  private   boolean                        useSpeedometer          = false;
  /**
   * Max velocity of the speedometer simulation (Optional).
   */
  private   Optional<AngularVelocity>      speedometerMaxVelocity  = Optional.empty();

  /**
   * FlyWheel Configuration class
   *
   */
  public FlyWheelConfig()
  {}

  private FlyWheelConfig(FlyWheelConfig cfg)
  {
    this.telemetryName = cfg.telemetryName;
    this.telemetryVerbosity = cfg.telemetryVerbosity;
    this.diameter = cfg.diameter;
    this.simColor = cfg.simColor;
    this.mechanismPositionConfig = cfg.mechanismPositionConfig;
    this.useSpeedometer = cfg.useSpeedometer;
    this.speedometerMaxVelocity = cfg.speedometerMaxVelocity;
  }

  @Override
  public FlyWheelConfig clone()
  {
    return new FlyWheelConfig(this);
  }

  /**
   * Enables the use of the speedometer simulation for the shooter.
   * <p>
   * The speedometer simulation is a simulation of a speedometer that is used to simulate the behavior of the shooter.
   * This is useful for testing and debugging the shooter without having to physically move it.
   *
   * @param maxVelocity The maximum velocity of the shooter.
   * @return {@link FlyWheelConfig} for chaining.
   */
  public FlyWheelConfig withSpeedometerSimulation(AngularVelocity maxVelocity)
  {
    this.useSpeedometer = true;
    this.speedometerMaxVelocity = Optional.ofNullable(maxVelocity);
    return this;
  }

  /**
   * Enables the use of the speedometer simulation for the shooter.
   * <p>
   * The speedometer simulation is a simulation of a speedometer that is used to simulate the behavior of the shooter.
   * This is useful for testing and debugging the shooter without having to physically move it.
   *
   * @return {@link FlyWheelConfig} for chaining.
   */
  public FlyWheelConfig withSpeedometerSimulation()
  {
    if (!speedometerMaxVelocity.isPresent())
    {
      throw new FlyWheelConfigurationException("Speedometer max velocity is not set.",
                                               "Cannot use speedometer simulation!",
                                               "Set it with withSpeedometerSimulation(AngularVelocity)");
    }
    this.useSpeedometer = true;
    return this;
  }

  /**
   * Disable the use of the speedometer simulation for the shooter.
   * <p>
   * The speedometer simulation is a simulation of a speedometer that is used to simulate the behavior of the shooter.
   * This is useful for testing and debugging the shooter without having to physically move it.
   *
   * @return {@link FlyWheelConfig} for chaining.
   */
  public FlyWheelConfig disableSpeedometerSimulation()
  {
    this.useSpeedometer = false;
    return this;
  }

  /**
   * Check if the shooter is using the speedometer simulation.
   * <p>
   * The speedometer simulation is a simulation of the speedometer that is used to simulate the behavior of the shooter.
   * This is useful for testing and debugging the shooter without having to physically move it.
   *
   * @return True if the shooter is using the speedometer simulation.
   */
  public boolean isUsingSpeedometerSimulation()
  {
    return useSpeedometer;
  }


  /**
   * Get the maximum velocity of the speedometer simulation.
   * <p>
   * If the speedometer simulation is not enabled, this will return an empty Optional.
   *
   * @return The maximum velocity of the speedometer simulation, or an empty Optional if the speedometer simulation is
   * not enabled.
   */
  public Optional<AngularVelocity> getSpeedometerMaxVelocity()
  {
    return speedometerMaxVelocity;
  }

  /**
   * Publish the color in sim as this.
   *
   * @param simColor {@link Color8Bit} to show.
   * @return {@link FlyWheelConfig} for chaining.
   */
  public FlyWheelConfig withSimColor(final Color8Bit simColor)
  {
    this.simColor = simColor;
    return this;
  }

  /**
   * Configure the {@link FlyWheel}s diameter for simulation.
   *
   * @param distance Length of the {@link FlyWheel}.
   * @return {@link FlyWheelConfig} for chaining.
   */
  public FlyWheelConfig withDiameter(Distance distance)
  {
    this.diameter = Optional.ofNullable(distance);
    return this;
  }

  /**
   * Set the shooter mechanism position configuration.
   *
   * @param mechanismPositionConfig {@link MechanismPositionConfig} for the {@link FlyWheel}
   * @return {@link FlyWheelConfig} for chaining
   */
  public FlyWheelConfig withMechanismPositionConfig(MechanismPositionConfig mechanismPositionConfig)
  {
    this.mechanismPositionConfig = mechanismPositionConfig;
    return this;
  }

  /**
   * Configure telemetry for the {@link FlyWheel} mechanism.
   *
   * @param telemetryName      Telemetry NetworkTable name to appear under "SmartDashboard/"
   * @param telemetryVerbosity Telemetry verbosity to apply.
   * @return {@link FlyWheelConfig} for chaining.
   */
  public FlyWheelConfig withTelemetry(String telemetryName, TelemetryVerbosity telemetryVerbosity)
  {
    this.telemetryName = Optional.ofNullable(telemetryName);
    this.telemetryVerbosity = Optional.ofNullable(telemetryVerbosity);
    return this;
  }
  /**
   * Get the Length of the {@link FlyWheel}
   *
   * @return {@link Distance} of the {@link FlyWheel} or an empty {@link Optional} if not set.
   */
  public Optional<Distance> getDiameter()
  {
    return diameter;
  }

  /**
   * Get the telemetry verbosity of the {@link FlyWheel}
   *
   * @return {@link TelemetryVerbosity} of the {@link FlyWheel}
   */
  public Optional<TelemetryVerbosity> getTelemetryVerbosity()
  {
    return telemetryVerbosity;
  }

  /**
   * Network Tables name for the {@link FlyWheel}
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
   * @return sim color.
   */
  public Color8Bit getSimColor()
  {
    return simColor;
  }

  /**
   * Get the {@link MechanismPositionConfig} associated with this {@link FlyWheelConfig}.
   *
   * @return An {@link Optional} containing the {@link MechanismPositionConfig} if present, otherwise an empty
   * {@link Optional}.
   */
  public MechanismPositionConfig getMechanismPositionConfig()
  {
    return mechanismPositionConfig;
  }

  /**
   * Get the circumference of the {@link FlyWheel}.
   *
   * @return {@link Distance} representing the circumference of the FlyWheel.
   */
  public Distance getCircumference()
  {
    if (diameter.isEmpty())
    {
      throw new FlyWheelConfigurationException("FlyWheel diameter is empty",
                                               "Cannot run speed without diameter.",
                                               "withDiameter(Distance)");
    }
    return diameter.orElseThrow().times(Math.PI);
  }

  /**
   * Get the {@link LinearVelocity} of the {@link FlyWheel} given an {@link AngularVelocity}
   *
   * @param velocity {@link AngularVelocity} to convert to {@link LinearVelocity}
   * @return {@link LinearVelocity} of the {@link FlyWheel}
   */
  public LinearVelocity getLinearVelocity(AngularVelocity velocity)
  {
    return getCircumference().per(Second).times(velocity.in(RotationsPerSecond));
  }

  /**
   * Get the {@link AngularVelocity} of the {@link FlyWheel} given a {@link LinearVelocity}
   *
   * @param velocity {@link LinearVelocity} to convert to {@link AngularVelocity}
   * @return {@link AngularVelocity} of the {@link FlyWheel}
   */
  public AngularVelocity getAngularVelocity(LinearVelocity velocity)
  {
    return RotationsPerSecond.of(velocity.in(MetersPerSecond) / getCircumference().in(Meters));
  }
}
