package yams.mechanisms.config;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.Optional;
import java.util.OptionalDouble;
import yams.exceptions.PivotConfigurationException;
import yams.mechanisms.config.MechanismPositionConfig.Plane;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * Pivot config.
 */
public class PivotConfig
{

  /**
   * {@link SmartMotorController} for the {@link yams.mechanisms.positional.Pivot}
   */
  private final SmartMotorController         motor;
  /**
   * The network root of the mechanism (Optional).
   */
  @Deprecated
  protected     Optional<String>             networkRoot             = Optional.empty();
  /**
   * Telemetry name.
   */
  private       Optional<String>             telemetryName           = Optional.empty();
  /**
   * Telemetry verbosity
   */
  private       Optional<TelemetryVerbosity> telemetryVerbosity      = Optional.empty();
  /**
   * Lower Hard Limit for the {@link yams.mechanisms.positional.Pivot} to be representing in simulation.
   */
  private       Optional<Angle>              lowerHardLimit          = Optional.empty();
  /**
   * Upper hard limit for the {@link yams.mechanisms.positional.Pivot} representing in simulation.
   */
  private       Optional<Angle>              upperHardLimit          = Optional.empty();
  /**
   * {@link yams.mechanisms.positional.Pivot} MOI from CAD software. If not given estimated with length and weight.
   */
  private       OptionalDouble               moi                     = OptionalDouble.empty();
  /**
   * Sim color value
   */
  private       Color8Bit                    simColor                = new Color8Bit(Color.kOrange);
  /**
   * Mechanism position configuration for the {@link yams.mechanisms.positional.Pivot}
   */
  private       MechanismPositionConfig      mechanismPositionConfig = new MechanismPositionConfig();

  /**
   * Pivot Configuration class
   *
   * @param motorController Primary {@link SmartMotorController} for the {@link yams.mechanisms.positional.Pivot}
   */
  public PivotConfig(SmartMotorController motorController)
  {
    motor = motorController;
    mechanismPositionConfig.withMovementPlane(Plane.XY);
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
   * Configure the MOI directly instead of estimating it with the length and mass of the
   * {@link yams.mechanisms.positional.Pivot} for simulation.
   *
   * @param MOI Moment of Inertia of the {@link yams.mechanisms.positional.Pivot}
   * @return {@link PivotConfig} for chaining.
   */
  public PivotConfig withMOI(double MOI)
  {
    motor.getConfig().withMomentOfInertia(MOI);
    this.moi = OptionalDouble.of(MOI);
    return this;
  }

  /**
   * Configure the MOI directly instead of estimating it with the length and mass of the
   * {@link yams.mechanisms.positional.Pivot} for simulation.
   *
   * @param length Length of the {@link yams.mechanisms.positional.Pivot}.
   * @param weight Weight of the {@link yams.mechanisms.positional.Pivot}
   * @return {@link PivotConfig} for chaining.
   */
  public PivotConfig withMOI(Distance length, Mass weight)
  {
    motor.getConfig().withMomentOfInertia(length, weight);
    this.moi = OptionalDouble.of(SingleJointedArmSim.estimateMOI(length.in(Meters), weight.in(Kilograms)));
    return this;
  }

  /**
   * Configure telemetry for the {@link yams.mechanisms.positional.Pivot} mechanism.
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
   * Configure telemetry for the {@link yams.mechanisms.positional.Pivot} mechanism.
   *
   * @param telemetryName      Telemetry NetworkTable name to appear under "SmartDashboard/"
   * @param telemetryVerbosity Telemetry verbosity to apply.
   * @param networkRoot        Network root to publish the telemetry under.
   * @return {@link ArmConfig} for chaining.
   */
  @Deprecated
  public PivotConfig withTelemetry(String networkRoot, String telemetryName, TelemetryVerbosity telemetryVerbosity)
  {
    this.networkRoot = Optional.ofNullable(networkRoot);
    this.telemetryName = Optional.ofNullable(telemetryName);
    this.telemetryVerbosity = Optional.ofNullable(telemetryVerbosity);
    return this;
  }

  /**
   * Set the elevator mechnism position configuration.
   *
   * @param mechanismPositionConfig {@link MechanismPositionConfig} for the {@link yams.mechanisms.positional.Elevator}
   * @return {@link PivotConfig} for chaining
   */
  public PivotConfig withMechanismPositionConfig(MechanismPositionConfig mechanismPositionConfig)
  {
    this.mechanismPositionConfig = mechanismPositionConfig;
    return this;
  }

  /**
   * Set the pivot starting position.
   *
   * @param startingPosition Starting position of the pivot.
   * @return {@link PivotConfig} for chaining
   */
  public PivotConfig withStartingPosition(Angle startingPosition)
  {
    motor.getConfig().withStartingPosition(startingPosition);
    return this;
  }

  /**
   * Set the pivot soft limits. When exceeded the power will be set to 0.
   *
   * @param lowerLimit Minimum rotation of the pivot.
   * @param upperLimit Maximum rotation of the pivot.
   * @return {@link PivotConfig} for chaining.
   */
  public PivotConfig withSoftLimits(Angle lowerLimit, Angle upperLimit)
  {
    motor.getConfig().withSoftLimit(lowerLimit, upperLimit);
    return this;
  }

  /**
   * Wrap the pivot around these angles. When the pivot exceeds the maximum angle it will read as if it is the minimum
   * angle.
   *
   * @param min Minimum angle for wrapping
   * @param max Maximum angle for wrapping.
   * @return {@link PivotConfig} for chaining.
   */
  public PivotConfig withWrapping(Angle min, Angle max)
  {
    motor.getConfig().withContinuousWrapping(min, max);
    return this;
  }

  /**
   * Set the Hard Limits for simulation
   *
   * @param min Angle where the physical stop appears.
   * @param max Angle where the physical stop appears
   * @return {@link PivotConfig} for chaining.
   */
  public PivotConfig withHardLimit(Angle min, Angle max)
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
    return motor.applyConfig(motor.getConfig());
  }


  /**
   * Get the moment of inertia for the {@link yams.mechanisms.positional.Pivot} simulation.
   *
   * @return Moment of Inertia.
   */
  public double getMOI()
  {
    if (moi.isPresent())
    {
      return moi.getAsDouble();
    }
    throw new PivotConfigurationException("Pivot MOI must be set!",
                                          "Cannot get the MOI!",
                                          "withLength(Distance).withMass(Mass) OR PivotConfig.withMOI()");
  }

  /**
   * Get the Upper hard limit of the {@link yams.mechanisms.positional.Pivot}.
   *
   * @return {@link Angle} hard limit.
   */
  public Optional<Angle> getUpperHardLimit()
  {
    return upperHardLimit;
  }

  /**
   * Get the lower hard limit of the {@link yams.mechanisms.positional.Pivot}
   *
   * @return {@link Angle} hard limit.
   */
  public Optional<Angle> getLowerHardLimit()
  {
    return lowerHardLimit;
  }

  /**
   * Get the telemetry verbosity of the {@link yams.mechanisms.positional.Pivot}
   *
   * @return {@link TelemetryVerbosity} of the {@link yams.mechanisms.positional.Pivot}
   */
  public Optional<TelemetryVerbosity> getTelemetryVerbosity()
  {
    return telemetryVerbosity;
  }

  /**
   * Network Tables name for the {@link yams.mechanisms.positional.Pivot}
   *
   * @return Network Tables name.
   */
  public Optional<String> getTelemetryName()
  {
    return telemetryName;
  }

  /**
   * Get the starting angle of the {@link yams.mechanisms.positional.Pivot}
   *
   * @return {@link Angle} of the {@link yams.mechanisms.positional.Pivot}
   */
  public Optional<Angle> getStartingAngle()
  {
    return motor.getConfig().getStartingPosition();
  }

  /**
   * Get the {@link SmartMotorController} of the {@link yams.mechanisms.positional.Pivot}
   *
   * @return {@link SmartMotorController} for the {@link yams.mechanisms.positional.Pivot}
   */
  public SmartMotorController getMotor()
  {
    return motor;
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

  /**
   * Get the network root of the mechanism.
   *
   * @return Optional containing the network root if set, otherwise an empty Optional.
   */
  @Deprecated
  public Optional<String> getNetworkRoot()
  {
    return networkRoot;
  }
}
