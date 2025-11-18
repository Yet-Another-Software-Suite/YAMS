package yams.mechanisms.config;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.Optional;
import java.util.OptionalDouble;
import yams.exceptions.ArmConfigurationException;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * Arm configuration class.
 */
public class ArmConfig
{

  /**
   * {@link SmartMotorController} for the {@link yams.mechanisms.positional.Arm}
   */
  private final SmartMotorController         motor;
  /**
   * The network root of the mechanism (Optional).
   */
  protected     Optional<String>             networkTableName        = Optional.empty();
  /**
   * Telemetry name.
   */
  private       Optional<String>             telemetryName           = Optional.empty();
  /**
   * Telemetry verbosity
   */
  private       Optional<TelemetryVerbosity> telemetryVerbosity      = Optional.empty();
  /**
   * Lower Hard Limit for the {@link yams.mechanisms.positional.Arm} to be representing in simulation.
   */
  private       Optional<Angle>              lowerHardLimit          = Optional.empty();
  /**
   * Upper hard limit for the {@link yams.mechanisms.positional.Arm} representing in simulation.
   */
  private       Optional<Angle>              upperHardLimit          = Optional.empty();
  /**
   * {@link yams.mechanisms.positional.Arm} length for simulation.
   */
  private       Optional<Distance>           length                  = Optional.empty();
  /**
   * {@link yams.mechanisms.positional.Arm} mass for simulation.
   */
  private       Optional<Mass>               weight                  = Optional.empty();
  /**
   * {@link yams.mechanisms.positional.Arm} MOI from CAD software. If not given estimated with length and weight.
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
   * @param motorController Primary {@link SmartMotorController} for the {@link yams.mechanisms.positional.Arm}
   */
  public ArmConfig(SmartMotorController motorController)
  {
    motor = motorController;
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
   * Configure the MOI directly instead of estimating it with the length and mass of the
   * {@link yams.mechanisms.positional.Arm} for simulation.
   *
   * @param MOI Moment of Inertia of the {@link yams.mechanisms.positional.Arm}
   * @return {@link ArmConfig} for chaining.
   */
  public ArmConfig withMOI(double MOI)
  {
    motor.getConfig().withMomentOfInertia(MOI);
    this.moi = OptionalDouble.of(MOI);
    return this;
  }

  /**
   * Configure the {@link yams.mechanisms.positional.Arm}s length for simulation.
   *
   * @param distance Length of the {@link yams.mechanisms.positional.Arm}.
   * @return {@link ArmConfig} for chaining.
   */
  public ArmConfig withLength(Distance distance)
  {
    if (weight.isPresent() && distance != null)
    {
      motor.getConfig().withMomentOfInertia(distance, weight.get());
    }
    this.length = Optional.ofNullable(distance);
    return this;
  }

  /**
   * Configure the {@link yams.mechanisms.positional.Arm}s {@link Mass} for simulation.
   *
   * @param mass {@link Mass} of the {@link yams.mechanisms.positional.Arm}
   * @return {@link ArmConfig} for chaining.
   */
  public ArmConfig withMass(Mass mass)
  {
    if (length.isPresent() && mass != null)
    {
      motor.getConfig().withMomentOfInertia(length.get(), mass);
    }
    this.weight = Optional.ofNullable(mass);
    return this;
  }

  /**
   * Configure telemetry for the {@link yams.mechanisms.positional.Arm} mechanism.
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
   * Configure telemetry for the {@link yams.mechanisms.positional.Arm} mechanism.
   *
   * @param networkRoot        Telemetry NetworkTable
   * @param telemetryName      Telemetry NetworkTable name to appear under _networkTableName_
   * @param telemetryVerbosity Telemetry verbosity to apply.
   * @return {@link ArmConfig} for chaining.
   */
  @Deprecated
  public ArmConfig withTelemetry(String networkRoot, String telemetryName, TelemetryVerbosity telemetryVerbosity)
  {
    this.networkTableName = Optional.ofNullable(networkRoot);
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
  public ArmConfig withMechanismPositionConfig(MechanismPositionConfig mechanismPositionConfig)
  {
    this.mechanismPositionConfig = mechanismPositionConfig;
    return this;
  }

  /**
   * Set the horizontal zero of the arm.
   *
   * @param horizontalZero Offset of the arm that will make the arm read 0 when horizontal.
   * @return {@link ArmConfig} for chaining.
   */
  public ArmConfig withHorizontalZero(Angle horizontalZero)
  {
    motor.getConfig().withExternalEncoderZeroOffset(horizontalZero);
    return this;
  }

  /**
   * Set the arm starting position.
   *
   * @param startingPosition Starting position of the arm.
   * @return {@link ArmConfig} for chaining
   */
  public ArmConfig withStartingPosition(Angle startingPosition)
  {
    motor.getConfig().withStartingPosition(startingPosition);
    return this;
  }

  /**
   * Set the arm soft limits. When exceeded the power will be set to 0.
   *
   * @param lowerLimit Minimum rotation of the arm.
   * @param upperLimit Maximum rotation of the arm.
   * @return {@link ArmConfig} for chaining.
   */
  public ArmConfig withSoftLimits(Angle lowerLimit, Angle upperLimit)
  {
    motor.getConfig().withSoftLimit(lowerLimit, upperLimit);
    return this;
  }

  /**
   * Wrap the arm around these angles. When the arm exceeds the maximum angle it will read as if it is the minimum
   * angle.
   *
   * @param min Minimum angle for wrapping
   * @param max Maximum angle for wrapping.
   * @return {@link ArmConfig} for chaining.
   */
  public ArmConfig withWrapping(Angle min, Angle max)
  {
    motor.getConfig().withContinuousWrapping(min, max);
    return this;
  }

  /**
   * Set the Hard Limits for simulation
   *
   * @param min Angle where the physical stop appears.
   * @param max Angle where the physical stop appears
   * @return {@link ArmConfig} for chaining.
   */
  public ArmConfig withHardLimit(Angle min, Angle max)
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
   * Get the Length of the {@link yams.mechanisms.positional.Arm}
   *
   * @return {@link Distance} of the Arm.
   */
  public Optional<Distance> getLength()
  {
    return length;
  }

  /**
   * Get the moment of inertia for the {@link yams.mechanisms.positional.Arm} simulation.
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
    throw new ArmConfigurationException("Arm length and weight or MOI must be set!",
                                        "Cannot get the MOI!",
                                        "withLength(Distance).withMass(Mass) OR ArmConfig.withMOI()");
  }

  /**
   * Get the Upper hard limit of the {@link yams.mechanisms.positional.Arm}.
   *
   * @return {@link Angle} hard limit.
   */
  public Optional<Angle> getUpperHardLimit()
  {
    return upperHardLimit;
  }

  /**
   * Get the lower hard limit of the {@link yams.mechanisms.positional.Arm}
   *
   * @return {@link Angle} hard limit.
   */
  public Optional<Angle> getLowerHardLimit()
  {
    return lowerHardLimit;
  }

  /**
   * Get the telemetry verbosity of the {@link yams.mechanisms.positional.Arm}
   *
   * @return {@link TelemetryVerbosity} of the {@link yams.mechanisms.positional.Arm}
   */
  public Optional<TelemetryVerbosity> getTelemetryVerbosity()
  {
    return telemetryVerbosity;
  }

  /**
   * Network Tables name for the {@link yams.mechanisms.positional.Arm}
   *
   * @return Network Tables name.
   */
  public Optional<String> getTelemetryName()
  {
    return telemetryName;
  }

  /**
   * Get the starting angle of the {@link yams.mechanisms.positional.Arm}
   *
   * @return {@link Angle} of the {@link yams.mechanisms.positional.Arm}
   */
  public Optional<Angle> getStartingAngle()
  {
    return motor.getConfig().getStartingPosition();
  }

  /**
   * Get the {@link SmartMotorController} of the {@link yams.mechanisms.positional.Arm}
   *
   * @return {@link SmartMotorController} for the {@link yams.mechanisms.positional.Arm}
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
   * Get the {@link MechanismPositionConfig} associated with this {@link ArmConfig}.
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
