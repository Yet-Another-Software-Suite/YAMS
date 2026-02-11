package yams.mechanisms.config;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
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
  private   Optional<SmartMotorController> motor;
  /**
   * The network root of the mechanism (Optional).
   */
  @Deprecated
  protected Optional<String>               networkRoot             = Optional.empty();
  /**
   * Telemetry name.
   */
  private   Optional<String>               telemetryName           = Optional.empty();
  /**
   * Telemetry verbosity
   */
  private   Optional<TelemetryVerbosity>   telemetryVerbosity      = Optional.empty();
  /**
   * Lower Hard Limit for the {@link yams.mechanisms.positional.Pivot} to be representing in simulation.
   */
  private   Optional<Angle>                lowerHardLimit          = Optional.empty();
  /**
   * Upper hard limit for the {@link yams.mechanisms.positional.Pivot} representing in simulation.
   */
  private   Optional<Angle>                upperHardLimit          = Optional.empty();
  /**
   * {@link yams.mechanisms.positional.Pivot} MOI from CAD software. If not given estimated with length and weight.
   */
  private   OptionalDouble                 moi                     = OptionalDouble.empty();
  /**
   * Sim color value
   */
  private   Color8Bit                      simColor                = new Color8Bit(Color.kOrange);
  /**
   * Mechanism position configuration for the {@link yams.mechanisms.positional.Pivot}
   */
  private   MechanismPositionConfig        mechanismPositionConfig = new MechanismPositionConfig();
  /**
   * Starting position of the pivot {@link SmartMotorController} motor encoder.
   */
  private   Optional<Angle>                startingPosition        = Optional.empty();
  /**
   * Soft limits of the pivot motor {@link SmartMotorController} closed loop controller. (LowerLimit, UpperLimit)
   */
  private   Optional<Pair<Angle, Angle>>   softLimits              = Optional.empty();
  /**
   * Continuous wrapping of the pivot {@link SmartMotorController} closed loop controller. (Min, Max)
   */
  private   Optional<Pair<Angle, Angle>>   continuousWrapping      = Optional.empty();

  /**
   * Pivot Configuration class
   *
   * @param motorController Primary {@link SmartMotorController} for the {@link yams.mechanisms.positional.Pivot}
   */
  public PivotConfig(SmartMotorController motorController)
  {
    motor = Optional.ofNullable(motorController);
    mechanismPositionConfig.withMovementPlane(Plane.XY);
  }

  /**
   * Pivot Configuration class
   *
   * @implNote Required call to {@link #withSmartMotorController(SmartMotorController)} before usage.
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
    this.motor = cfg.motor;
    this.networkRoot = cfg.networkRoot;
    this.telemetryName = cfg.telemetryName;
    this.telemetryVerbosity = cfg.telemetryVerbosity;
    this.lowerHardLimit = cfg.lowerHardLimit;
    this.upperHardLimit = cfg.upperHardLimit;
    this.moi = cfg.moi;
    this.simColor = cfg.simColor;
    this.mechanismPositionConfig = cfg.mechanismPositionConfig;
    this.startingPosition = cfg.startingPosition;
    this.softLimits = cfg.softLimits;
    this.continuousWrapping = cfg.continuousWrapping;
  }

  @Override
  public PivotConfig clone()
  {
    return new PivotConfig(this);
  }

  /**
   * Configure the {@link SmartMotorController} for the {@link yams.mechanisms.positional.Pivot}
   *
   * @param motorController {@link SmartMotorController} for the {@link yams.mechanisms.positional.Pivot}.
   * @return {@link PivotConfig} for chaining.
   */
  public PivotConfig withSmartMotorController(SmartMotorController motorController)
  {
    if (motor.isPresent())
    {
      throw new PivotConfigurationException("Pivot SmartMotorController already set!",
                                            "Pivot motor cannot be set",
                                            "withSmartMotorController(SmartMotorController)");
    }
    motor = Optional.of(motorController);
    startingPosition.ifPresent(this::withStartingPosition);
    softLimits.ifPresent(softLimits -> withSoftLimits(softLimits.getFirst(), softLimits.getSecond()));
    continuousWrapping.ifPresent(continuousWrapping -> withWrapping(continuousWrapping.getFirst(),
                                                                    continuousWrapping.getSecond()));
    return this;
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
   * @param MOI Moment of Inertia of the {@link yams.mechanisms.positional.Pivot}. in {@link edu.wpi.first.units.Units#KilogramSquareMeters}
   * @return {@link PivotConfig} for chaining.
   * @implNote Please use {@link #withMOI(MomentOfInertia)} instead. Default unit is KilogramSquareMeters
   */
  @Deprecated(since = "2026", forRemoval = true)
  public PivotConfig withMOI(double MOI)
  {
    motor.ifPresent(motor -> motor.getConfig().withMomentOfInertia(KilogramSquareMeters.of(MOI)));
    this.moi = OptionalDouble.of(MOI);
    return this;
  }

  /**
   * Configure the MOI directly instead of estimating it with the length and mass of the
   * {@link yams.mechanisms.positional.Pivot} for simulation.
   *
   * @param MOI Moment of Inertia of the {@link yams.mechanisms.positional.Pivot}
   * @return {@link PivotConfig} for chaining.
   */
  public PivotConfig withMOI(MomentOfInertia MOI)
  {
    motor.ifPresent(motor -> motor.getConfig().withMomentOfInertia(MOI));
    this.moi = OptionalDouble.of(MOI.in(KilogramSquareMeters));
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
    motor.ifPresent(motor -> motor.getConfig().withMomentOfInertia(length, weight));
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
   * Set the elevator mechanism position configuration.
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
    this.startingPosition = Optional.ofNullable(startingPosition);
    motor.ifPresent(motor -> motor.getConfig().withStartingPosition(startingPosition));
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
    softLimits = Optional.of(Pair.of(lowerLimit, upperLimit));
    motor.ifPresent(motor -> motor.getConfig().withSoftLimit(lowerLimit, upperLimit));
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
    continuousWrapping = Optional.of(Pair.of(min, max));
    motor.ifPresent(motor -> motor.getConfig().withContinuousWrapping(min, max));
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
    return motor.orElseThrow().applyConfig(motor.orElseThrow().getConfig());
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
    return motor.orElseThrow().getConfig().getStartingPosition();
  }

  /**
   * Get the {@link SmartMotorController} of the {@link yams.mechanisms.positional.Pivot}
   *
   * @return {@link SmartMotorController} for the {@link yams.mechanisms.positional.Pivot}
   */
  public SmartMotorController getMotor()
  {
    return motor.orElseThrow();
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
