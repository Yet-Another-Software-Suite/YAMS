package yams.mechanisms.config;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

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
   *
   * @param speed Minimum velocity of the shooter.
   * @return {@link ShooterConfig} for chaining
   */
  public ShooterConfig withLowerSoftLimit(AngularVelocity speed)
  {
    minVelocity = Optional.ofNullable(speed);
    return this;
  }

  /**
   * Set the maximum velocity of the shooter.
   *
   * @param speed Maximum velocity of the shooter.
   * @return {@link ShooterConfig} for chaining.
   */
  public ShooterConfig withUpperSoftLimit(AngularVelocity speed)
  {
    maxVelocity = Optional.ofNullable(speed);
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
    return this;
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
