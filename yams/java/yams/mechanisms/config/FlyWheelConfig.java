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
 * <h3>Configuration Example</h3>
 * <pre>{@code
 * // 1. Build the motor config with kV and kS feedforward (critical for velocity mechanisms)
 * SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig()
 *     .withFeedforward(0.15, 0.12)   // kS=0.15, kV=0.12 — kV dominates at steady-state RPM
 *     .withVelocityPID(0.0005, 0.0, 0.0) // kP
 *     .withTelemetry("Shooter", TelemetryVerbosity.HIGH);
 *
 * // 2. Create a TalonFXWrapper for a Kraken X60 on CAN ID 1
 * SmartMotorController motor = new TalonFXWrapper(new TalonFX(1), DCMotor.getKrakenX60(1), motorConfig);
 *
 * // 3. Assemble the FlyWheelConfig
 * FlyWheelConfig config = new FlyWheelConfig()
 *     .withSmartMotorController(motor)
 *     .withDiameter(Inches.of(4))
 *     .withMass(Pounds.of(0.5))
 *     .withSpeedometerSimulation(RPM.of(6000))  // optional: sim speedometer up to 6000 RPM
 *     .withTelemetry("Shooter", TelemetryVerbosity.HIGH);
 * }</pre>
 */
public class FlyWheelConfig
{
  /**
   * {@link SmartMotorController} for the {@link FlyWheel}
   */
  private Optional<SmartMotorController> motor = Optional.empty();
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
   * {@link FlyWheel} mass for simulation.
   */
  private Optional<Mass>               weight             = Optional.empty();
  /**
   * {@link FlyWheel} MOI from CAD software. If not given estimated with length and weight.
   */
  private   OptionalDouble                 moi                     = OptionalDouble.empty();
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
   * Arm Configuration class
   *
   * @param motorController Primary {@link SmartMotorController} for the {@link FlyWheel}
   */
  public FlyWheelConfig(SmartMotorController motorController)
  {
    motor = Optional.ofNullable(motorController);
  }

  /**
   * FlyWheel configuration class.
   *
   * @implNote Required to call {@link #withSmartMotorController(SmartMotorController)} before this is used with an
   * {@link FlyWheel}
   */
  public FlyWheelConfig() {}

  private FlyWheelConfig(FlyWheelConfig cfg)
  {
    this.motor = cfg.motor;
    this.telemetryName = cfg.telemetryName;
    this.telemetryVerbosity = cfg.telemetryVerbosity;
    this.diameter = cfg.diameter;
    this.weight = cfg.weight;
    this.moi = cfg.moi;
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
   * Set the {@link SmartMotorController} for the {@link FlyWheel}
   *
   * @param motorController Primary {@link SmartMotorController} for the {@link FlyWheel}
   * @return {@link FlyWheelConfig} for chaining.
   */
  public FlyWheelConfig withSmartMotorController(SmartMotorController motorController)
  {
    if (motor.isPresent())
    {
      throw new FlyWheelConfigurationException("FlyWheel SmartMotorController already set!",
                                               "Flywheel cannot be set",
                                               "withSmartMotorController(SmartMotorController)");
    }
    motor = Optional.of(motorController);
    return this;
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
   * Configure the MOI directly instead of estimating it with the length and mass of the {@link FlyWheel} for
   * simulation.
   *
   * @param MOI Moment of Inertia of the {@link FlyWheel}
   * @return {@link FlyWheelConfig} for chaining.
   */
  public FlyWheelConfig withMOI(MomentOfInertia MOI)
  {
    motor.ifPresent(motor -> motor.getConfig().withMomentOfInertia(MOI));
    this.moi = OptionalDouble.of(MOI.in(KilogramSquareMeters));
    return this;
  }

  /**
   * Configure the MOI directly instead of estimating it with the length and mass of the {@link FlyWheel} for
   * simulation.
   *
   * @param length Length of the {@link FlyWheel}.
   * @param weight Weight of the {@link FlyWheel}
   * @return {@link FlyWheelConfig} for chaining.
   */
  public FlyWheelConfig withMOI(Distance length, Mass weight)
  {
    motor.ifPresent(motor -> motor.getConfig().withMomentOfInertia(length, weight));
    this.moi = OptionalDouble.of(SingleJointedArmSim.estimateMOI(length.in(Meters), weight.in(Kilograms)));
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
   * Configure the {@link FlyWheel}s {@link Mass} for simulation.
   *
   * @param mass {@link Mass} of the {@link FlyWheel}
   * @return {@link FlyWheelConfig} for chaining.
   */
  public FlyWheelConfig withMass(Mass mass)
  {
    this.weight = Optional.ofNullable(mass);
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
   * Apply config changes from this class to the {@link SmartMotorController}
   *
   * @return {@link SmartMotorController#applyConfig(SmartMotorControllerConfig)} result.
   */
  public boolean applyConfig()
  {
    return motor.orElseThrow().applyConfig(motor.orElseThrow().getConfig());
  }

  /**
   * Get the Length of the {@link FlyWheel}
   *
   * @return {@link Distance} of the {@link FlyWheel} or an empty {@link Optional} if not set..
   */
  public Optional<Distance> getDiameter()
  {
    return diameter;
  }

  /**
   * Get the moment of inertia for the {@link FlyWheel} simulation.
   *
   * @return Moment of Inertia.
   */
  public double getMOI()
  {
    if (moi.isPresent())
    {
      return moi.getAsDouble();
    }
    if (diameter.isPresent() && weight.isPresent())
    {
      return SingleJointedArmSim.estimateMOI(diameter.get().in(Units.Meters), weight.get().in(Units.Kilograms));
    }
    throw new FlyWheelConfigurationException("FlyWheel diameter and weight or MOI must be set!",
                                             "Cannot get the MOI!",
                                             "withDiameter(Distance).withMass(Mass) OR FlyWheelConfig.withMOI()");
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
   * Get the {@link SmartMotorController} of the {@link FlyWheel}
   *
   * @return {@link SmartMotorController} for the {@link FlyWheel}
   */
  public SmartMotorController getMotor()
  {
    return motor.orElseThrow();
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
