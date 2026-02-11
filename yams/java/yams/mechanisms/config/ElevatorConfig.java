package yams.mechanisms.config;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.Optional;
import java.util.OptionalInt;
import yams.exceptions.SmartMotorControllerConfigurationException;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * Elevator configuration class.
 */
public class ElevatorConfig
{

  /**
   * {@link SmartMotorController} for the {@link yams.mechanisms.positional.Elevator}
   */
  private Optional<SmartMotorController>         motor = Optional.empty();
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
   * Lower Hard Limit for the {@link yams.mechanisms.positional.Elevator} to be representing in simulation.
   */
  private       Optional<Distance>           lowerHardLimit          = Optional.empty();
  /**
   * Upper hard limit for the {@link yams.mechanisms.positional.Elevator} representing in simulation.
   */
  private       Optional<Distance>           upperHardLimit          = Optional.empty();
  /**
   * {@link yams.mechanisms.positional.Elevator} angle for simulation.
   */
  private       Angle                        angle                   = Degrees.of(90);
  /**
   * {@link yams.mechanisms.positional.Elevator} carriage mass for simulation.
   */
  private       Optional<Mass>               carriageWeight          = Optional.empty();
  /**
   * Sim color value
   */
  private       Color8Bit                    simColor                = new Color8Bit(Color.kOrange);
  /**
   * Mechanism position configuration for the {@link yams.mechanisms.positional.Pivot} (Optional).
   */
  private       MechanismPositionConfig mechanismPositionConfig = new MechanismPositionConfig();
  /**
   * Drum radius of the elevator spool, or the sprocket pitch * teeth.
   */
  private Optional<Distance>            drumCircumference       = Optional.empty();
  /**
   * Elevator stages, applied to the motor controller config gearing by dividing it by the number of stages given.
   */
  private OptionalInt                   stages                  = OptionalInt.empty();
  /**
   * Starting height to set the motor's encoder to.
   */
  private Optional<Distance> startingHeight = Optional.empty();
  /**
   * Soft limits of the {@link SmartMotorController} closed loop controller. Can be exceeded. (LowerLimit, UpperLimit)
   */
  private Optional<Pair<Distance, Distance>> softLimits = Optional.empty();
  /**
   * Disable gravity on the elevator simulation.
   */
  private boolean       isElevatorHorizontal          = false;

  /**
   * Elevator Configuration class
   *
   * @param motorController Primary {@link SmartMotorController} for the {@link yams.mechanisms.positional.Elevator}
   */
  public ElevatorConfig(SmartMotorController motorController)
  {
    motor = Optional.ofNullable(motorController);
  }

  /**
   * Elevator Configuration class
   *
   * @implNote You are REQUIRED to call {@link #withSmartMotorController(SmartMotorController)} before this is used with
   * an {@link yams.mechanisms.positional.Elevator}
   */
  public ElevatorConfig() {}

  /**
   * Copy constructor.
   *
   * @param cfg Configuration to copy from.
   */
  private ElevatorConfig(ElevatorConfig cfg)
  {
    this.motor = cfg.motor;
    this.drumCircumference = cfg.drumCircumference;
    this.stages = cfg.stages;
    this.startingHeight = cfg.startingHeight;
    this.softLimits = cfg.softLimits;
    this.simColor = cfg.simColor;
    this.angle = cfg.angle;
    this.carriageWeight = cfg.carriageWeight;
    this.telemetryName = cfg.telemetryName;
    this.telemetryVerbosity = cfg.telemetryVerbosity;
    this.networkRoot = cfg.networkRoot;
    this.mechanismPositionConfig = cfg.mechanismPositionConfig;
    this.lowerHardLimit = cfg.lowerHardLimit;
    this.upperHardLimit = cfg.upperHardLimit;
  }

  @Override
  public ElevatorConfig clone()
  {
    return new ElevatorConfig(this);
  }

  /**
   * Set the {@link SmartMotorController} for the {@link yams.mechanisms.positional.Elevator}
   *
   * @param motorController Primary {@link SmartMotorController} for the {@link yams.mechanisms.positional.Elevator}
   * @return {@link ElevatorConfig} for chaining.
   */
  public ElevatorConfig withSmartMotorController(SmartMotorController motorController)
  {
    motor = Optional.of(motorController);
    drumCircumference.ifPresent(drumRadius->motor.get().getConfig().withMechanismCircumference(drumRadius));
    stages.ifPresent(this::withCascadingElevatorStages);
    startingHeight.ifPresent(this::withStartingHeight);
    softLimits.ifPresent(softLimits->withSoftLimits(softLimits.getFirst(), softLimits.getSecond()));
    return this;
  }

  /**
   * Set the {@link yams.mechanisms.positional.Elevator} drum radius.
   *
   * @param drumRadius Elevator drum radius
   * @return {@link ElevatorConfig} for chaining.
   */
  public ElevatorConfig withDrumRadius(Distance drumRadius)
  {
    this.drumCircumference = Optional.ofNullable(drumRadius.times(2 * Math.PI));
    motor.ifPresent(motor -> motor.getConfig().withMechanismCircumference(drumRadius.times(2 * Math.PI)));
    return this;
  }

  /**
   * Set the {@link yams.mechanisms.positional.Elevator} drum radius via the chain pitch (.25in or .35in) and teeth
   * count.
   *
   * @param chainPitch Chain pitch.
   * @param teeth      Sprocket teeth count.
   * @return {@link ElevatorConfig} for chaining.
   */
  public ElevatorConfig withDrumRadius(Distance chainPitch, int teeth)
  {
    this.drumCircumference = Optional.ofNullable(chainPitch.times(teeth));
    motor.ifPresent(motor -> motor.getConfig().withMechanismCircumference(chainPitch.times(teeth)));
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
   * Configure the {@link yams.mechanisms.positional.Elevator}s angle for simulation.
   *
   * @param angle Angle of the {@link yams.mechanisms.positional.Elevator}.
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
    this.carriageWeight = Optional.ofNullable(mass);
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
    this.telemetryName = Optional.ofNullable(telemetryName);
    this.telemetryVerbosity = Optional.ofNullable(telemetryVerbosity);
    return this;
  }

  /**
   * Configure telemetry for the {@link yams.mechanisms.positional.Arm} mechanism.
   *
   * @param telemetryName      Telemetry NetworkTable name to appear under "SmartDashboard/"
   * @param telemetryVerbosity Telemetry verbosity to apply.
   * @param networkRoot        Network root to publish the telemetry under.
   * @return {@link ElevatorConfig} for chaining.
   */
  @Deprecated
  public ElevatorConfig withTelemetry(String networkRoot, String telemetryName, TelemetryVerbosity telemetryVerbosity)
  {
    this.networkRoot = Optional.ofNullable(networkRoot);
    this.telemetryName = Optional.ofNullable(telemetryName);
    this.telemetryVerbosity = Optional.ofNullable(telemetryVerbosity);
    return this;
  }

  /**
   * Change the {@link SmartMotorControllerConfig} gear ratio to be divided by the number of stages given, will reapply
   * it if already done manually.
   *
   * @param stages Stages given
   * @return {@link ElevatorConfig} for chaining.
   */
  public ElevatorConfig withCascadingElevatorStages(int stages)
  {
    this.stages = OptionalInt.of(stages);
    motor.ifPresent(motor -> motor.getConfig().withGearing(motor.getConfig().getGearing().div(stages)));
    return this;
  }

  /**
   * Set the elevator mechanism position configuration.
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
    startingHeight = Optional.ofNullable(startingPosition);
    motor.ifPresent(motor->motor.getConfig().withStartingPosition(startingPosition));
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
    softLimits = Optional.of(Pair.of(lowerLimit,upperLimit));
    motor.ifPresent(motor->motor.getConfig().withSoftLimit(lowerLimit, upperLimit));
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
    lowerHardLimit = Optional.ofNullable(min);
    upperHardLimit = Optional.ofNullable(max);
    return this;
  }

  /**
   * Set elevator as horizontal to avoid gravity simulation
   * 
   * 
   * @return {@link ElevatorConfig} for chaining.
   */
  public ElevatorConfig withHorizontalElevator()
  {
    isElevatorHorizontal = true;
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
   * Get the Angle of the {@link yams.mechanisms.positional.Elevator}
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
    return motor.orElseThrow().getConfig().getStartingPosition().isPresent() ? Optional.of(motor.orElseThrow().getConfig()
                                                                                  .convertFromMechanism(motor.orElseThrow().getConfig()
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
    return motor.orElseThrow();
  }

  /**
   * Get sim color
   *
   * @return sim color
   */
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
    if (motor.orElseThrow().getConfig().getMechanismCircumference().isEmpty())
    {
      throw new SmartMotorControllerConfigurationException("Mechanism circumference is undefined",
                                                           "Drum radius cannot be fetched.",
                                                           "withMechanismCircumference(Distance)");
    }
    return motor.orElseThrow().getConfig().getMechanismCircumference().get().div(2 * Math.PI);
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
   * Get if the elevator is horizontal
   *
   * @return if elevator is horizontal.
   */
  public boolean getIsElevatorHorizontal()
  {
    return isElevatorHorizontal;
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
   *
   * @return Optional containing the network root if set, otherwise an empty Optional.
   */
  @Deprecated
  public Optional<String> getNetworkRoot()
  {
    return networkRoot;
  }
}
