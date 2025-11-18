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
import yams.exceptions.FlyWheelConfigurationException;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * FlyWheel configuration class.
 */
public class FlyWheelConfig
{

  /**
   * {@link SmartMotorController} for the {@link FlyWheel}
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
   * {@link FlyWheel} length for simulation.
   */
  private       Optional<Distance>           length                  = Optional.empty();
  /**
   * {@link FlyWheel} mass for simulation.
   */
  private       Optional<Mass>               weight                  = Optional.empty();
  /**
   * {@link FlyWheel} MOI from CAD software. If not given estimated with length and weight.
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
  private boolean                   useSpeedometer         = false;
  /*
   * Max velocity of the speedometer simulation (Optional).
   */
  private Optional<AngularVelocity> speedometerMaxVelocity = Optional.empty();

  /**
   * Arm Configuration class
   *
   * @param motorController Primary {@link SmartMotorController} for the {@link FlyWheel}
   */
  public FlyWheelConfig(SmartMotorController motorController)
  {
    motor = motorController;
  }

  /**
   * Set the minimum velocity of the shooter. Also updates the speedometer simulation max velocity
   *
   * @param speed Minimum velocity of the shooter.
   * @return {@link FlyWheelConfig} for chaining
   */
  public FlyWheelConfig withLowerSoftLimit(AngularVelocity speed)
  {
    minVelocity = Optional.ofNullable(speed);
    // Set the speedometer max to the highest absolute value of the max and min Velocity
    // If the max is less than the min, set the speedometer max to the min
    speedometerMaxVelocity = Optional.ofNullable(
        maxVelocity.orElse(RPM.of(0)).abs(RPM) > (minVelocity.orElse(RPM.of(0))).abs(RPM)
        ? RPM.of(maxVelocity.orElse(RPM.of(0)).abs(RPM))
        // The orElse here is just to deal with the Optional, it will likely never be 0
        : RPM.of(minVelocity.orElse(RPM.of(0))
                            .abs(RPM))); // The orElse here is just to deal with the Optional, it will likely never be 0
    return this;
  }

  /**
   * Set the maximum velocity of the shooter. Also updates the speedometer simulation max velocity
   *
   * @param speed Maximum velocity of the shooter.
   * @return {@link FlyWheelConfig} for chaining.
   */
  public FlyWheelConfig withUpperSoftLimit(AngularVelocity speed)
  {
    maxVelocity = Optional.ofNullable(speed);
    // Set the speedometer max to the highest absolute value of the max and min Velocity
    // If the max is less than the min, set the speedometer max to the min
    speedometerMaxVelocity = Optional.ofNullable(
        maxVelocity.orElse(RPM.of(0)).abs(RPM) > (minVelocity.orElse(RPM.of(0))).abs(RPM)
        ? RPM.of(maxVelocity.orElse(RPM.of(0)).abs(RPM))
        // The orElse here is just to deal with the Optional, it will likely never be 0
        : RPM.of(minVelocity.orElse(RPM.of(0))
                            .abs(RPM))); // The orElse here is just to deal with the Optional, it will likely never be 0
    return this;
  }

  /**
   * Set the shooter soft limits.
   *
   * @param low  Minimum velocity of the shooter.
   * @param high Maximum velocity of the shooter.
   * @return {@link FlyWheelConfig} for chaining.
   */
  public FlyWheelConfig withSoftLimit(AngularVelocity low, AngularVelocity high)
  {
    minVelocity = Optional.ofNullable(low);
    maxVelocity = Optional.ofNullable(high);
    // Set the speedometer max to the highest absolute value of the two
    speedometerMaxVelocity = Optional.ofNullable(
        high.abs(RPM) > low.abs(RPM) ? RPM.of(high.abs(RPM)) : RPM.of(low.abs(RPM)));
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
                                               "Set it with useSpeedometerSimulation(AngularVelocity) or withSoftLimit(AngularVelocity, AngularVelocity)");
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
   * Configure the MOI directly instead of estimating it with the length and mass of the
   * {@link FlyWheel} for simulation.
   *
   * @param MOI Moment of Inertia of the {@link FlyWheel}
   * @return {@link FlyWheelConfig} for chaining.
   */
  public FlyWheelConfig withMOI(double MOI)
  {
    motor.getConfig().withMomentOfInertia(MOI);
    this.moi = OptionalDouble.of(MOI);
    return this;
  }

  /**
   * Configure the MOI directly instead of estimating it with the length and mass of the
   * {@link FlyWheel} for simulation.
   *
   * @param length Length of the {@link FlyWheel}.
   * @param weight Weight of the {@link FlyWheel}
   * @return {@link FlyWheelConfig} for chaining.
   */
  public FlyWheelConfig withMOI(Distance length, Mass weight)
  {
    motor.getConfig().withMomentOfInertia(length, weight);
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
    this.length = Optional.ofNullable(distance);
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
   * Configure telemetry for the {@link FlyWheel} mechanism.
   *
   * @param networkRoot        Telemetry NetworkTable
   * @param telemetryName      Telemetry NetworkTable name to appear under _networkTableName_
   * @param telemetryVerbosity Telemetry verbosity to apply.
   * @return {@link FlyWheelConfig} for chaining.
   */
  @Deprecated
  public FlyWheelConfig withTelemetry(String networkRoot, String telemetryName, TelemetryVerbosity telemetryVerbosity)
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
   * Get the Length of the {@link FlyWheel}
   *
   * @return {@link Distance} of the Arm.
   */
  public Optional<Distance> getLength()
  {
    return length;
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
    if (length.isPresent() && weight.isPresent())
    {
      return SingleJointedArmSim.estimateMOI(length.get().in(Units.Meters), weight.get().in(Units.Kilograms));
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
