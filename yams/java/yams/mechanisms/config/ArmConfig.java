// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechanisms.config;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.Optional;
import yams.exceptions.ArmConfigurationException;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Elevator;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * Arm configuration class.
 *
 * <h2>Configuration Example</h2>
 * <pre>{@code
 * // Create a motor first
 * SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig()
 *     .withClosedLoopController(0.12,0,0)
 *     .withFeedforward(new ArmFeedforward(0.005,0.05,0.3))
 *     .withStatorCurrentLimit(Amps.of(40))
 *     .withMechanismUpperLimit(Degrees.of(90))
 *     .withMechanismLowerLimit(Degrees.of(-10));
 * SmartMotorController motor = new SparkWrapper(
 *     new SparkMax(1, MotorType.kBrushless), DCMotor.getNEO(1), motorConfig);
 *
 * // Build the arm config
 * ArmConfig config = new ArmConfig(motor)
 *     .withLength(Meters.of(0.5))
 *     .withMass(Kilograms.of(2.0))
 *     .withTelemetry("Arm", TelemetryVerbosity.HIGH)
 *     .withHardLimits(Degrees.of(-10), Degrees.of(90))
 *     .withTelemetry("Arm", TelemetryVerbosity.HIGH);
 * }</pre>
 */
public class ArmConfig
{
  /**
   * {@link SmartMotorController} for the {@link Arm}
   */
  private Optional<SmartMotorController> motor               = Optional.empty();
  /**
   * Telemetry name.
   */
  private   Optional<String>               telemetryName           = Optional.empty();
  /**
   * Telemetry verbosity
   */
  private   Optional<TelemetryVerbosity>   telemetryVerbosity      = Optional.empty();
  /**
   * Lower Hard Limit for the {@link Arm} to be representing in simulation.
   */
  private   Optional<Angle>                lowerHardLimit          = Optional.empty();
  /**
   * Upper hard limit for the {@link Arm} representing in simulation.
   */
  private   Optional<Angle>                upperHardLimit          = Optional.empty();
  /**
   * {@link Arm} length for simulation.
   */
  private   Optional<Distance>             length                  = Optional.empty();
  /**
   * {@link Arm} mass for simulation.
   */
  private   Optional<Mass>                 weight                  = Optional.empty();
  /**
   * Sim color value
   */
  private   Color8Bit                      simColor                = new Color8Bit(Color.kOrange);
  /**
   * Mechanism position configuration for the {@link Pivot} (Optional).
   */
  private   MechanismPositionConfig        mechanismPositionConfig = new MechanismPositionConfig();
  /**
   * Arm Configuration class
   *
   * @param motorController Primary {@link SmartMotorController} for the {@link Arm}
   */
  public ArmConfig(SmartMotorController motorController)
  {
    motor = Optional.of(motorController);
  }

  /**
   * Arm configuration class. Required
   */
  public ArmConfig()
  {}

  /**
   * Copy constructor.
   *
   * @param cfg Configuration to copy from.
   */
  private ArmConfig(ArmConfig cfg)
  {
    motor = cfg.motor;
    telemetryName = cfg.telemetryName;
    telemetryVerbosity = cfg.telemetryVerbosity;
    lowerHardLimit = cfg.lowerHardLimit;
    upperHardLimit = cfg.upperHardLimit;
    length = cfg.length;
    weight = cfg.weight;
    simColor = cfg.simColor;
    mechanismPositionConfig = cfg.mechanismPositionConfig;
  }

  @Override
  public ArmConfig clone()
  {
    return new ArmConfig(this);
  }

  /**
   * Configure the {@link SmartMotorController} for the {@link Arm}
   *
   * @param motorController Primary {@link SmartMotorController} for the {@link Arm}
   * @return {@link ArmConfig} for chaining.
   */
  public ArmConfig withSmartMotorController(SmartMotorController motorController)
  {
    if (motor.isPresent())
    {
      throw new ArmConfigurationException("Arm already has a SmartMotorController!",
                                          "Cannot set a new one!",
                                          ".withSmartMotorController(SmartMotorController)");
    }
    motor = Optional.of(motorController);
    if (length.isPresent() && weight.isPresent())
    {
      motorController.getConfig().withMomentOfInertia(length.get(), weight.get());
    }
    return this;
  }


  /**
   * Publish the color in sim as this.
   *
   * @param simColor {@link Color8Bit} to show.
   * @return {@link ArmConfig} for chaining.
   */
  public ArmConfig withSimColor(final Color8Bit simColor)
  {
    this.simColor = simColor;
    return this;
  }

  /**
   * Configure the {@link Arm}s length for simulation.
   *
   * @param distance Length of the {@link Arm}.
   * @return {@link ArmConfig} for chaining.
   */
  public ArmConfig withLength(Distance distance)
  {
    if (weight.isPresent() && distance != null)
    {
      motor.ifPresent(motor -> motor.getConfig().withMomentOfInertia(distance, weight.get()));
    }
    this.length = Optional.ofNullable(distance);
    return this;
  }

  /**
   * Configure the {@link Arm}s {@link Mass} for simulation.
   *
   * @param mass {@link Mass} of the {@link Arm}
   * @return {@link ArmConfig} for chaining.
   */
  public ArmConfig withMass(Mass mass)
  {
    if (length.isPresent() && mass != null)
    {
      motor.ifPresent(motor -> motor.getConfig().withMomentOfInertia(length.get(), mass));
    }
    this.weight = Optional.ofNullable(mass);
    return this;
  }

  /**
   * Configure telemetry for the {@link Arm} mechanism.
   *
   * @param telemetryName      Telemetry NetworkTable name to appear under "SmartDashboard/"
   * @param telemetryVerbosity Telemetry verbosity to apply.
   * @return {@link ArmConfig} for chaining.
   */
  public ArmConfig withTelemetry(String telemetryName, TelemetryVerbosity telemetryVerbosity)
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
  public ArmConfig withMechanismPositionConfig(MechanismPositionConfig mechanismPositionConfig)
  {
    this.mechanismPositionConfig = mechanismPositionConfig;
    return this;
  }

  /**
   * Set the Hard Limits for simulation
   *
   * @param min Angle where the physical stop appears.
   * @param max Angle where the physical stop appears
   * @return {@link ArmConfig} for chaining.
   */
  public ArmConfig withHardLimits(Angle min, Angle max)
  {
    lowerHardLimit = Optional.ofNullable(min);
    upperHardLimit = Optional.ofNullable(max);
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
   * Get the Length of the {@link Arm}
   *
   * @return {@link Distance} of the Arm.
   */
  public Optional<Distance> getLength()
  {
    return length;
  }

  /**
   * Get the moment of inertia for the {@link Arm} simulation.
   * MOI must be configured via {@link SmartMotorControllerConfig#withMomentOfInertia(Distance, Mass)}
   * or {@link SmartMotorControllerConfig#withMomentOfInertia(edu.wpi.first.units.measure.MomentOfInertia)}.
   * Providing {@link #withLength(Distance)} and {@link #withMass(Mass)} will set it automatically.
   *
   * @return Moment of Inertia in KgMetersSquared.
   */
  public double getMOI()
  {
    if (motor.isPresent())
    {
      return motor.get().getConfig().getMOI();
    }
    if (length.isPresent() && weight.isPresent())
    {
      return SingleJointedArmSim.estimateMOI(length.get().in(Units.Meters), weight.get().in(Units.Kilograms));
    }
    throw new ArmConfigurationException("Arm MOI must be configured!",
                                        "Cannot get the MOI!",
                                        "SmartMotorControllerConfig.withMomentOfInertia(Distance, Mass) or withMomentOfInertia(MomentOfInertia)");
  }

  /**
   * Get the Upper hard limit of the {@link Arm}.
   *
   * @return {@link Angle} hard limit.
   */
  public Optional<Angle> getUpperHardLimit()
  {
    return upperHardLimit;
  }

  /**
   * Get the lower hard limit of the {@link Arm}
   *
   * @return {@link Angle} hard limit.
   */
  public Optional<Angle> getLowerHardLimit()
  {
    return lowerHardLimit;
  }

  /**
   * Get the telemetry verbosity of the {@link Arm}
   *
   * @return {@link TelemetryVerbosity} of the {@link Arm}
   */
  public Optional<TelemetryVerbosity> getTelemetryVerbosity()
  {
    return telemetryVerbosity;
  }

  /**
   * Network Tables name for the {@link Arm}
   *
   * @return Network Tables name.
   */
  public Optional<String> getTelemetryName()
  {
    return telemetryName;
  }

  /**
   * Get the starting angle of the {@link Arm}. Reads from {@link SmartMotorControllerConfig#getStartingPosition()}.
   * Configure via {@link SmartMotorControllerConfig#withStartingPosition(Angle)}.
   *
   * @return {@link Angle} of the {@link Arm}
   */
  public Optional<Angle> getStartingAngle()
  {
    return motor.flatMap(m -> m.getConfig().getStartingPosition());
  }

  /**
   * Get the {@link SmartMotorController} of the {@link Arm}
   *
   * @return {@link SmartMotorController} for the {@link Arm}
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
   * Get the {@link MechanismPositionConfig} associated with this {@link ArmConfig}.
   *
   * @return An {@link Optional} containing the {@link MechanismPositionConfig} if present, otherwise an empty
   * {@link Optional}.
   */
  public MechanismPositionConfig getMechanismPositionConfig()
  {
    return mechanismPositionConfig;
  }

}
