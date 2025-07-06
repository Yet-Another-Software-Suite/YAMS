package yams.mechanisms.config;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import yams.exceptions.SmartMotorControllerConfigurationException;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class ElevatorConfig
{

  /**
   * {@link SmartMotorController} for the {@link yams.mechanisms.positional.Elevator}
   */
  private final SmartMotorController motor;
  /**
   * Telemetry name.
   */
  private       Optional<String>     telemetryName = Optional.empty();
  /**
   * The network root of the mechanism (Optional).
   */
  protected Optional<String> networkRoot = Optional.empty();
  /**
   * Telemetry verbosity
   */
  private Optional<TelemetryVerbosity> telemetryVerbosity = Optional.empty();
  /**
   * Lower Hard Limit for the {@link yams.mechanisms.positional.Elevator} to be representing in simulation.
   */
  private Optional<Distance>           lowerHardLimit     = Optional.empty();
  /**
   * Upper hard limit for the {@link yams.mechanisms.positional.Elevator} representing in simulation.
   */
  private Optional<Distance>           upperHardLimit     = Optional.empty();
  /**
   * {@link yams.mechanisms.positional.Elevator} length for simulation.
   */
  private Angle                        angle              = Degrees.of(90);
  /**
   * {@link yams.mechanisms.positional.Elevator} carriage mass for simulation.
   */
  private Optional<Mass>               carriageWeight     = Optional.empty();
  /**
   * Sim color value
   */
  private Color8Bit                    simColor           = new Color8Bit(Color.kOrange);
  /**
   * Mechanism position configuration for the {@link yams.mechanisms.positional.Pivot} (Optional).
   */
  private MechanismPositionConfig mechanismPositionConfig = new MechanismPositionConfig();


  /**
   * Elevator Configuration class
   *
   * @param motorController Primary {@link SmartMotorController} for the {@link yams.mechanisms.positional.Elevator}
   */
  public ElevatorConfig(SmartMotorController motorController)
  {
    motor = motorController;
  }

  /**
   * Set the {@link yams.mechanisms.positional.Elevator} drum radius.
   *
   * @param drumRadius Elevator drum radius
   * @return {@link ElevatorConfig} for chaining.
   */
  public ElevatorConfig withDrumRadius(Distance drumRadius)
  {
    motor.getConfig().withMechanismCircumference(drumRadius.times(2 * Math.PI));
    return this;
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
   * Configure the {@link yams.mechanisms.positional.Elevator}s length for simulation.
   *
   * @param angle Length of the {@link yams.mechanisms.positional.Elevator}.
   * @return {@link ElevatorConfig} for chaining.
   */
  public ElevatorConfig withAngle(Angle angle)
  {
    this.angle = angle;
    return this;
  }

  /**
   * Configure the {@link yams.mechanisms.positional.Elevator}s {@link Mass} for simulation.
   *
   * @param mass {@link Mass} of the {@link yams.mechanisms.positional.Elevator}
   * @return {@link ElevatorConfig} for chaining.
   */
  public ElevatorConfig withMass(Mass mass)
  {
    this.carriageWeight = mass == null ? Optional.empty() : Optional.of(mass);
    return this;
  }

  /**
   * Configure telemetry for the {@link yams.mechanisms.positional.Elevator} mechanism.
   *
   * @param telemetryName      Telemetry NetworkTable name to appear under "SmartDashboard/"
   * @param telemetryVerbosity Telemetry verbosity to apply.
   * @return {@link ElevatorConfig} for chaining.
   */
  public ElevatorConfig withTelemetry(String telemetryName, TelemetryVerbosity telemetryVerbosity)
  {
    this.telemetryName = telemetryName == null ? Optional.empty() : Optional.of(telemetryName);
    this.telemetryVerbosity = telemetryVerbosity == null ? Optional.empty() : Optional.of(telemetryVerbosity);
    return this;
  }

  /**
   * Configure telemetry for the {@link yams.mechanisms.positional.Arm} mechanism.
   *
   * @param telemetryName      Telemetry NetworkTable name to appear under "SmartDashboard/"
   * @param telemetryVerbosity Telemetry verbosity to apply.
   * @return {@link ArmConfig} for chaining.
   */
  public ElevatorConfig withTelemetry(String networkRoot, String telemetryName, TelemetryVerbosity telemetryVerbosity)
  {
    this.networkRoot = Optional.ofNullable(networkRoot);
    this.telemetryName = telemetryName == null ? Optional.empty() : Optional.of(telemetryName);
    this.telemetryVerbosity = telemetryVerbosity == null ? Optional.empty() : Optional.of(telemetryVerbosity);
    return this;
  }

  /**
   * Set the elevator mechnism position configuration.
   * 
   * @param mechanismPositionConfig {@link MechanismPositionConfig} for the {@link yams.mechanisms.positional.Elevator}
   * @return {@link ElevatorConfig} for chaining
   */
  public ElevatorConfig withMechanismPositionConfig(MechanismPositionConfig mechanismPositionConfig)
  {
    this.mechanismPositionConfig = mechanismPositionConfig;
    return this;
  }

  /**
   * Set the elevator starting position.
   *
   * @param startingPosition Starting position of the elevator.
   * @return {@link ElevatorConfig} for chaining
   */
  public ElevatorConfig withStartingHeight(Distance startingPosition)
  {
    motor.getConfig().withStartingPosition(startingPosition);
    return this;
  }

  /**
   * Set the elevator soft limits. When exceeded the power will be set to 0.
   *
   * @param lowerLimit Minimum distance of the elevator.
   * @param upperLimit Maximum distance of the elevator.
   * @return {@link ElevatorConfig} for chaining.
   */
  public ElevatorConfig withSoftLimits(Distance lowerLimit, Distance upperLimit)
  {
    motor.getConfig().withSoftLimit(lowerLimit, upperLimit);
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
    lowerHardLimit = min == null ? Optional.empty() : Optional.of(min);
    upperHardLimit = max == null ? Optional.empty() : Optional.of(max);
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
   * Get the Length of the {@link yams.mechanisms.positional.Elevator}
   *
   * @return {@link Angle} of the Elevator.
   */
  public Angle getAngle()
  {
    return angle;
  }

  /**
   * Get the Upper hard limit of the {@link yams.mechanisms.positional.Elevator}.
   *
   * @return {@link Distance} hard limit.
   */
  public Optional<Distance> getMaximumHeight()
  {
    return upperHardLimit;
  }

  /**
   * Get the lower hard limit of the {@link yams.mechanisms.positional.Elevator}
   *
   * @return {@link Distance} hard limit.
   */
  public Optional<Distance> getMinimumHeight()
  {
    return lowerHardLimit;
  }

  /**
   * Get the telemetry verbosity of the {@link yams.mechanisms.positional.Elevator}
   *
   * @return {@link TelemetryVerbosity} of the {@link yams.mechanisms.positional.Elevator}
   */
  public Optional<TelemetryVerbosity> getTelemetryVerbosity()
  {
    return telemetryVerbosity;
  }

  /**
   * Network Tables name for the {@link yams.mechanisms.positional.Elevator}
   *
   * @return Network Tables name.
   */
  public Optional<String> getTelemetryName()
  {
    return telemetryName;
  }

  /**
   * Get the starting height of the {@link yams.mechanisms.positional.Elevator}
   *
   * @return {@link Distance} of the {@link yams.mechanisms.positional.Elevator}
   */
  public Optional<Distance> getStartingHeight()
  {
    return motor.getConfig().getStartingPosition().isPresent() ? Optional.of(motor.getConfig()
                                                                                  .convertFromMechanism(motor.getConfig()
                                                                                                             .getStartingPosition()
                                                                                                             .get()))
                                                               : Optional.empty();
  }

  /**
   * Get the {@link SmartMotorController} of the {@link yams.mechanisms.positional.Elevator}
   *
   * @return {@link SmartMotorController} for the {@link yams.mechanisms.positional.Elevator}
   */
  public SmartMotorController getMotor()
  {
    return motor;
  }

  public Color8Bit getSimColor()
  {
    return simColor;
  }

  /**
   * Get the {@link yams.mechanisms.positional.Elevator} drum radius.
   *
   * @return Drum radius of the elevator.
   */
  public Distance getDrumRadius()
  {
    if (motor.getConfig().getMechanismCircumference().isEmpty())
    {
      throw new SmartMotorControllerConfigurationException("Mechanism circumference is undefined",
                                                           "Drum radius cannot be fetched.",
                                                           "withMechanismCircumference(Distance)");
    }
    return motor.getConfig().getMechanismCircumference().get().div(2 * Math.PI);
  }

  /**
   * Get the {@link Mass} of the {@link yams.mechanisms.positional.Elevator} carriage.
   *
   * @return Carriage mass.
   */
  public Optional<Mass> getCarriageMass()
  {
    return carriageWeight;
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

  /**
   * Get the network root of the mechanism.
   * @return Optional containing the network root if set, otherwise an empty Optional.
   */
  public Optional<String> getNetworkRoot()
  {
    return networkRoot;
  }
}
