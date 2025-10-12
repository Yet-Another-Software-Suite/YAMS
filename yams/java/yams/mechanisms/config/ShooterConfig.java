package yams.mechanisms.config;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.Optional;
import java.util.OptionalDouble;
import yams.exceptions.ShooterConfigurationException;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * Shooter configuration class.
 */
public class ShooterConfig
{

  /**
   * {@link SmartMotorController} for the {@link yams.mechanisms.velocity.Shooter}
   */
  private final SmartMotorController         motor;
  /**
   * The network root of the mechanism (Optional).
   */
  protected     Optional<String>             networkTableName        = Optional.empty();
  /**
   * Minimum velocity of the shooter.
   */
  private       Optional<AngularVelocity>    minVelocity             = Optional.empty();
  /**
   * Maximum velocity of the shooter.
   */
  private       Optional<AngularVelocity>    maxVelocity             = Optional.empty();
  /**
   * Telemetry name.
   */
  private       Optional<String>             telemetryName           = Optional.empty();
  /**
   * Telemetry verbosity
   */
  private       Optional<TelemetryVerbosity> telemetryVerbosity      = Optional.empty();
  /**
   * {@link yams.mechanisms.velocity.Shooter} length for simulation.
   */
  private       Optional<Distance>           length                  = Optional.empty();
  /**
   * {@link yams.mechanisms.velocity.Shooter} mass for simulation.
   */
  private       Optional<Mass>               weight                  = Optional.empty();
  /**
   * {@link yams.mechanisms.velocity.Shooter} MOI from CAD software. If not given estimated with length and weight.
   */
  private       OptionalDouble               moi                     = OptionalDouble.empty();
  /**
   * Sim color value
   */
  private       Color8Bit                    simColor                = new Color8Bit(Color.kOrange);
  /**
   * Mechanism position configuration for the {@link yams.mechanisms.positional.Pivot} (Optional).
   */
  private       MechanismPositionConfig      mechanismPositionConfig = new MechanismPositionConfig();
  /*
   * Use speedometer simulation for the shooter.
   */
  private       boolean                      useSpeedometer           = false;
  /*
   * Max velocity of the speedometer simulation (Optional).
   */
  private       Optional<AngularVelocity>    speedometerMaxVelocity   = Optional.empty();

  /**
   * Arm Configuration class
   *
   * @param motorController Primary {@link SmartMotorController} for the {@link yams.mechanisms.velocity.Shooter}
   */
  public ShooterConfig(SmartMotorController motorController)
  {
    motor = motorController;
  }

  /**
   * Set the minimum velocity of the shooter.
   * Also updates the speedometer simulation max velocity
   *
   * @param speed Minimum velocity of the shooter.
   * @return {@link ShooterConfig} for chaining
   */
  public ShooterConfig withLowerSoftLimit(AngularVelocity speed)
  {
    minVelocity = Optional.ofNullable(speed);
    // Set the speedometer max to the highest absolute value of the max and min Velocity
    // If the max is less than the min, set the speedometer max to the min
    speedometerMaxVelocity = Optional.ofNullable(maxVelocity.orElse(RPM.of(0)).abs(RPM) > (minVelocity.orElse(RPM.of(0))).abs(RPM) 
      ? RPM.of(maxVelocity.orElse(RPM.of(0)).abs(RPM)) // The orElse here is just to deal with the Optional, it will likely never be 0
      : RPM.of(minVelocity.orElse(RPM.of(0)).abs(RPM))); // The orElse here is just to deal with the Optional, it will likely never be 0
    return this;
  }

  /**
   * Set the maximum velocity of the shooter.
   * Also updates the speedometer simulation max velocity
   *
   * @param speed Maximum velocity of the shooter.
   * @return {@link ShooterConfig} for chaining.
   */
  public ShooterConfig withUpperSoftLimit(AngularVelocity speed)
  {
    maxVelocity = Optional.ofNullable(speed);
    // Set the speedometer max to the highest absolute value of the max and min Velocity
    // If the max is less than the min, set the speedometer max to the min
    speedometerMaxVelocity = Optional.ofNullable(maxVelocity.orElse(RPM.of(0)).abs(RPM) > (minVelocity.orElse(RPM.of(0))).abs(RPM) 
      ? RPM.of(maxVelocity.orElse(RPM.of(0)).abs(RPM)) // The orElse here is just to deal with the Optional, it will likely never be 0
      : RPM.of(minVelocity.orElse(RPM.of(0)).abs(RPM))); // The orElse here is just to deal with the Optional, it will likely never be 0
    return this;
  }

  /**
   * Set the shooter soft limits.
   *
   * @param low  Minimum velocity of the shooter.
   * @param high Maximum velocity of the shooter.
   * @return {@link ShooterConfig} for chaining.
   */
  public ShooterConfig withSoftLimit(AngularVelocity low, AngularVelocity high)
  {
    minVelocity = Optional.ofNullable(low);
    maxVelocity = Optional.ofNullable(high);
    // Set the speedometer max to the highest absolute value of the two
    speedometerMaxVelocity = Optional.ofNullable(high.abs(RPM) > low.abs(RPM) ? RPM.of(high.abs(RPM)) : RPM.of(low.abs(RPM)));
    return this;
  }

  /**
   * Enables the use of the speedometer simulation for the shooter.
   *
   * The speedometer simulation is a simulation of a speedometer that is used
   * to simulate the behavior of the shooter. This is useful for testing and
   * debugging the shooter without having to physically move it.
   *
   * @param maxVelocity The maximum velocity of the shooter.
   * @return {@link ShooterConfig} for chaining.
   */
  public ShooterConfig withSpeedometerSimulation(AngularVelocity maxVelocity)
  {
    this.useSpeedometer = true;
    this.speedometerMaxVelocity = Optional.ofNullable(maxVelocity);
    return this;
  }

  /**
   * Enables the use of the speedometer simulation for the shooter.
   *
   * The speedometer simulation is a simulation of a speedometer that is used
   * to simulate the behavior of the shooter. This is useful for testing and
   * debugging the shooter without having to physically move it.
   *
   * @return {@link ShooterConfig} for chaining.
   */
  public ShooterConfig withSpeedometerSimulation()
  {
    if (!speedometerMaxVelocity.isPresent()) {
      throw new ShooterConfigurationException("Speedometer max velocity is not set." ,
        "Cannot use speedometer simulation!",
        "Set it with useSpeedometerSimulation(AngularVelocity) or withSoftLimit(AngularVelocity, AngularVelocity)");
    }
    this.useSpeedometer = true;
    return this;
  }
  
  /**
   * Disable the use of the speedometer simulation for the shooter.
   *
   * The speedometer simulation is a simulation of a speedometer that is used
   * to simulate the behavior of the shooter. This is useful for testing and
   * debugging the shooter without having to physically move it.
   *
   * @return {@link ShooterConfig} for chaining.
   */
  public ShooterConfig disableSpeedometerSimulation()
  {
    this.useSpeedometer = false;
    return this;
  }

  /**
   * Check if the shooter is using the speedometer simulation.
   *
   * The speedometer simulation is a simulation of the speedometer that is used
   * to simulate the behavior of the shooter. This is useful for testing and
   * debugging the shooter without having to physically move it.
   *
   * @return True if the shooter is using the speedometer simulation.
   */
  public boolean isUsingSpeedometerSimulation()
  {
    return useSpeedometer;
  }


  /**
   * Get the maximum velocity of the speedometer simulation.
   *
   * If the speedometer simulation is not enabled, this will return an empty Optional.
   *
   * @return The maximum velocity of the speedometer simulation, or an empty Optional if the speedometer simulation is not enabled.
   */
  public Optional<AngularVelocity> getSpeedometerMaxVelocity()
  {
    return speedometerMaxVelocity;
  }
  
  /**
   * Publish the color in sim as this.
   *
   * @param simColor {@link Color8Bit} to show.
   * @return {@link ShooterConfig} for chaining.
   */
  public ShooterConfig withSimColor(final Color8Bit simColor)
  {
    this.simColor = simColor;
    return this;
  }

  /**
   * Configure the MOI directly instead of estimating it with the length and mass of the
   * {@link yams.mechanisms.velocity.Shooter} for simulation.
   *
   * @param MOI Moment of Inertia of the {@link yams.mechanisms.velocity.Shooter}
   * @return {@link ShooterConfig} for chaining.
   */
  public ShooterConfig withMOI(double MOI)
  {
    motor.getConfig().withMomentOfInertia(MOI);
    this.moi = OptionalDouble.of(MOI);
    return this;
  }

  /**
   * Configure the MOI directly instead of estimating it with the length and mass of the
   * {@link yams.mechanisms.velocity.Shooter} for simulation.
   *
   * @param length Length of the {@link yams.mechanisms.velocity.Shooter}.
   * @param weight Weight of the {@link yams.mechanisms.velocity.Shooter}
   * @return {@link ShooterConfig} for chaining.
   */
  public ShooterConfig withMOI(Distance length, Mass weight)
  {
    motor.getConfig().withMomentOfInertia(length, weight);
    this.moi = OptionalDouble.of(SingleJointedArmSim.estimateMOI(length.in(Meters), weight.in(Kilograms)));
    return this;
  }

  /**
   * Configure the {@link yams.mechanisms.velocity.Shooter}s diameter for simulation.
   *
   * @param distance Length of the {@link yams.mechanisms.velocity.Shooter}.
   * @return {@link ShooterConfig} for chaining.
   */
  public ShooterConfig withDiameter(Distance distance)
  {
    this.length = Optional.ofNullable(distance);
    return this;
  }

  /**
   * Configure the {@link yams.mechanisms.velocity.Shooter}s {@link Mass} for simulation.
   *
   * @param mass {@link Mass} of the {@link yams.mechanisms.velocity.Shooter}
   * @return {@link ShooterConfig} for chaining.
   */
  public ShooterConfig withMass(Mass mass)
  {
    this.weight = Optional.ofNullable(mass);
    return this;
  }

  /**
   * Set the shooter mechanism position configuration.
   *
   * @param mechanismPositionConfig {@link MechanismPositionConfig} for the {@link yams.mechanisms.velocity.Shooter}
   * @return {@link ShooterConfig} for chaining
   */
  public ShooterConfig withMechanismPositionConfig(MechanismPositionConfig mechanismPositionConfig)
  {
    this.mechanismPositionConfig = mechanismPositionConfig;
    return this;
  }
  
  /**
   * Configure telemetry for the {@link yams.mechanisms.velocity.Shooter} mechanism.
   *
   * @param telemetryName      Telemetry NetworkTable name to appear under "SmartDashboard/"
   * @param telemetryVerbosity Telemetry verbosity to apply.
   * @return {@link ShooterConfig} for chaining.
   */
  public ShooterConfig withTelemetry(String telemetryName, TelemetryVerbosity telemetryVerbosity)
  {
    this.telemetryName = Optional.ofNullable(telemetryName);
    this.telemetryVerbosity = Optional.ofNullable(telemetryVerbosity);
    return this;
  }

  /**
   * Configure telemetry for the {@link yams.mechanisms.velocity.Shooter} mechanism.
   *
   * @param networkRoot        Telemetry NetworkTable
   * @param telemetryName      Telemetry NetworkTable name to appear under _networkTableName_
   * @param telemetryVerbosity Telemetry verbosity to apply.
   * @return {@link ShooterConfig} for chaining.
   */
  @Deprecated
  public ShooterConfig withTelemetry(String networkRoot, String telemetryName, TelemetryVerbosity telemetryVerbosity)
  {
    this.networkTableName = Optional.ofNullable(networkRoot);
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
    return motor.applyConfig(motor.getConfig());
  }

  /**
   * Get the Length of the {@link yams.mechanisms.velocity.Shooter}
   *
   * @return {@link Distance} of the Arm.
   */
  public Optional<Distance> getLength()
  {
    return length;
  }

  /**
   * Get the moment of inertia for the {@link yams.mechanisms.velocity.Shooter} simulation.
   *
   * @return Moment of Inertia.
   */
  public double getMOI()
  {
    if (moi.isPresent())
    {
      return moi.getAsDouble();
    }
    if (length.isPresent() && weight.isPresent())
    {
      return SingleJointedArmSim.estimateMOI(length.get().in(Units.Meters), weight.get().in(Units.Kilograms));
    }
    throw new ShooterConfigurationException("Shooter diameter and weight or MOI must be set!",
                                            "Cannot get the MOI!",
                                            "withDiameter(Distance).withMass(Mass) OR ShooterConfig.withMOI()");
  }

  /**
   * Get the telemetry verbosity of the {@link yams.mechanisms.velocity.Shooter}
   *
   * @return {@link TelemetryVerbosity} of the {@link yams.mechanisms.velocity.Shooter}
   */
  public Optional<TelemetryVerbosity> getTelemetryVerbosity()
  {
    return telemetryVerbosity;
  }

  /**
   * Get the upper soft limit for the shooter.
   *
   * @return Maximum velocity of the shooter.
   */
  public Optional<AngularVelocity> getUpperSoftLimit()
  {
    return maxVelocity;
  }

  /**
   * Get the lower soft limit of the shooter.
   *
   * @return Minimum velocity of the shooter.
   */
  public Optional<AngularVelocity> getLowerSoftLimit()
  {
    return minVelocity;
  }

  /**
   * Network Tables name for the {@link yams.mechanisms.velocity.Shooter}
   *
   * @return Network Tables name.
   */
  public Optional<String> getTelemetryName()
  {
    return telemetryName;
  }


  /**
   * Get the {@link SmartMotorController} of the {@link yams.mechanisms.velocity.Shooter}
   *
   * @return {@link SmartMotorController} for the {@link yams.mechanisms.velocity.Shooter}
   */
  public SmartMotorController getMotor()
  {
    return motor;
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
   * Get the {@link MechanismPositionConfig} associated with this {@link ShooterConfig}.
   *
   * @return An {@link Optional} containing the {@link MechanismPositionConfig} if present, otherwise an empty
   * {@link Optional}.
   */
  public MechanismPositionConfig getMechanismPositionConfig()
  {
    return mechanismPositionConfig;
  }

  /**
   * Get the telemetry network subtable of the mechanism.
   *
   * @return Optional containing the telemetry network subtable if set, otherwise an empty Optional.
   */
  @Deprecated
  public Optional<String> getTelemetryNetworkTableName()
  {
    return networkTableName;
  }
}
