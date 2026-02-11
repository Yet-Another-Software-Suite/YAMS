package yams.mechanisms.config;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Supplier;
import yams.exceptions.DifferentialMechanismConfigurationException;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig.Plane;
import yams.mechanisms.positional.DifferentialMechanism;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * Differential Mechanism config.
 */
public class DifferentialMechanismConfig
{

  /**
   * {@link SmartMotorController} for the {@link DifferentialMechanism}
   */
  private Optional<SmartMotorController>         leftMotorController = Optional.empty();
  /**
   * {@link SmartMotorController} for the {@link DifferentialMechanism}
   */
  private Optional<SmartMotorController>         rightMotorController = Optional.empty();
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
   * Twist gearing between the twist bevel gear and the motor sprocket. Separate from Tilt gearing implemented in the
   * {@link SmartMotorControllerConfig}.
   */
  private       MechanismGearing             twistGearing            = new MechanismGearing(new GearBox(new double[]{
      1}));
  /**
   * Supplier of the twist angle using an absolute encoder for the {@link DifferentialMechanism}.
   */
  private       Optional<Supplier<Angle>>    twistAngle              = Optional.empty();
  /**
   * Supplier of the tilt angle using an absolute encoder for the {@link DifferentialMechanism}.
   */
  private       Optional<Supplier<Angle>>    tiltAngle               = Optional.empty();
  /**
   * Starting twist angle for the {@link DifferentialMechanism}.
   */
  private       Optional<Angle>              startingTwistAngle      = Optional.empty();
  /**
   * Starting tilt angle for the {@link DifferentialMechanism}.
   */
  private       Optional<Angle>              startingTiltAngle       = Optional.empty();
  /**
   * {@link DifferentialMechanism} MOI from CAD software. If not given estimated with length and weight.
   */
  private       OptionalDouble               MOI                     = OptionalDouble.empty();
  /**
   * {@link DifferentialMechanism} length.
   */
  private       Optional<Distance>           length                  = Optional.empty();
  /**
   * Sim color value
   */
  private       Color8Bit                    simColor                = new Color8Bit(Color.kOrange);
  /**
   * Mechanism position configuration for the {@link DifferentialMechanism}
   */
  private       MechanismPositionConfig      mechanismPositionConfig = new MechanismPositionConfig();

  /**
   * Differential Mechanism configuration class.
   *
   * @param left  Left {@link SmartMotorController} configured with the gearing up to the tilt movement if
   *              synchronized.
   * @param right Right {@link SmartMotorController} configured with the gearing up to the tilt movement if
   *              syncrhonized.
   */
  public DifferentialMechanismConfig(SmartMotorController left, SmartMotorController right)
  {
    leftMotorController = Optional.ofNullable(left);
    rightMotorController = Optional.ofNullable(right);
    mechanismPositionConfig.withMovementPlane(Plane.XY);
  }

  /**
   * Differential Mechanism configuration class. Required call to
   * {@link #withSmartMotorControllers(SmartMotorController, SmartMotorController)} before usage.
   */
  public DifferentialMechanismConfig()
  {
    mechanismPositionConfig.withMovementPlane(Plane.XY);
  }

  /**
   * Copy constructor.
   *
   * @param cfg Configuration to copy.
   */
  private DifferentialMechanismConfig(DifferentialMechanismConfig cfg)
  {
    this.leftMotorController = cfg.leftMotorController;
    this.rightMotorController = cfg.rightMotorController;
    this.telemetryName = cfg.telemetryName;
    this.telemetryVerbosity = cfg.telemetryVerbosity;
    this.twistGearing = cfg.twistGearing;
    this.twistAngle = cfg.twistAngle;
    this.tiltAngle = cfg.tiltAngle;
    this.startingTwistAngle = cfg.startingTwistAngle;
    this.startingTiltAngle = cfg.startingTiltAngle;
    this.MOI = cfg.MOI;
    this.length = cfg.length;
    this.simColor = cfg.simColor;
    this.mechanismPositionConfig = cfg.mechanismPositionConfig;
    this.networkRoot = cfg.networkRoot;
  }

  @Override
  public DifferentialMechanismConfig clone()
  {
    return new DifferentialMechanismConfig(this);
  }

  /**
   * Add the smart motor controllers to the configuration.
   *
   * @param left  Left {@link SmartMotorController}
   * @param right Right {@link SmartMotorController}
   * @return {@link DifferentialMechanismConfig} for chaining.
   */
  public DifferentialMechanismConfig withSmartMotorControllers(SmartMotorController left, SmartMotorController right)
  {
    if(leftMotorController.isPresent())
      throw new DifferentialMechanismConfigurationException("Left motor controller already defined.","Cannot redefine left motor controller!", ".withSmartMotorControllers");
    if(rightMotorController.isPresent())
      throw new DifferentialMechanismConfigurationException("Right motor controller already defined.","Cannot redefine right motor controller!", ".withSmartMotorControllers");
    leftMotorController = Optional.ofNullable(left);
    rightMotorController = Optional.ofNullable(right);
    setStartingEncoderPositions();
    return this;
  }

  /**
   * Get the twist angle of the {@link DifferentialMechanism} with any additional twist gearing.
   *
   * @param leftMechPos  Left {@link SmartMotorController} Mechanism Position
   * @param rightMechPos Right {@link SmartMotorController} Mechanism Position
   * @return Twist {@link Angle}
   */
  public Angle getTwistAngle(Angle leftMechPos, Angle rightMechPos)
  {
    return leftMechPos.minus(rightMechPos).times(twistGearing.getRotorToMechanismRatio()).div(2);
  }

  /**
   * Get the tilt angle of the {@link DifferentialMechanism}.
   *
   * @param leftMechPos  Left {@link SmartMotorController} Mechanism Position
   * @param rightMechPos Right {@link SmartMotorController} Mechanism Position
   * @return Tilt {@link Angle}
   */
  public Angle getTiltAngle(Angle leftMechPos, Angle rightMechPos)
  {
    return leftMechPos.plus(rightMechPos).div(2);
  }

  /**
   * The left {@link SmartMotorController} mechanism position.
   *
   * @param tilt  {@link Angle} of the Tilt.
   * @param twist {@link Angle} of the twist.
   * @return Left mechanism position {@link Angle}
   */
  public Angle getLeftMechanismPosition(Angle tilt, Angle twist)
  {
    return tilt.plus(twist.times(twistGearing.getMechanismToRotorRatio()));
  }

  /**
   * The right {@link SmartMotorController} mechanism position.
   *
   * @param tilt  {@link Angle} of the Tilt.
   * @param twist {@link Angle} of the Twist.
   * @return Right mechanism position {@link Angle}
   */
  public Angle getRightMechanismPosition(Angle tilt, Angle twist)
  {
    return tilt.minus(twist.times(twistGearing.getMechanismToRotorRatio()));
  }

  /**
   * Set the starting encoder positions for the {@link DifferentialMechanism}.
   */
  private void setStartingEncoderPositions()
  {
    if (startingTiltAngle.isPresent() && startingTwistAngle.isPresent())
    {
      var twist = startingTwistAngle.get();
      var tilt  = startingTiltAngle.get();
      var left  = getLeftMechanismPosition(tilt, twist);
      var right = getRightMechanismPosition(tilt, twist);
      leftMotorController.ifPresent(leftMotorController ->leftMotorController.getConfig().withStartingPosition(left));
      rightMotorController.ifPresent(rightMotorController -> rightMotorController.getConfig().withStartingPosition(right));
    }
  }

  /**
   * Publish the color in sim as this.
   *
   * @param simColor {@link Color8Bit} to show.
   * @return {@link DifferentialMechanismConfig} for chaining.
   */
  public DifferentialMechanismConfig withSimColor(final Color8Bit simColor)
  {
    this.simColor = simColor;
    return this;
  }

  /**
   * Configure the MOI directly instead of estimating it with the length and mass of the {@link DifferentialMechanism}
   * for simulation.
   *
   * @param MOI Moment of Inertia of the {@link DifferentialMechanism}. In {@link edu.wpi.first.units.Units#KilogramSquareMeters}
   * @return {@link DifferentialMechanismConfig} for chaining.
   * @implNote Please use {@link #withMOI(MomentOfInertia)} instead. Default unit is KilogramSquareMeters
   */
  @Deprecated(since = "2026", forRemoval = true)
  public DifferentialMechanismConfig withMOI(double MOI)
  {
    this.MOI = OptionalDouble.of(MOI);
    return this;
  }

  /**
   * Configure the MOI directly instead of estimating it with the length and mass of the {@link DifferentialMechanism}
   * for simulation.
   *
   * @param MOI Moment of Inertia of the {@link DifferentialMechanism}
   * @return {@link DifferentialMechanismConfig} for chaining.
   */
  public DifferentialMechanismConfig withMOI(MomentOfInertia MOI)
  {
    this.MOI = OptionalDouble.of(MOI.in(KilogramSquareMeters));
    return this;
  }

  /**
   * Set the length of the {@link DifferentialMechanism}.
   *
   * @param length Length of the {@link DifferentialMechanism}
   * @return {@link DifferentialMechanismConfig} for chaining.
   */
  public DifferentialMechanismConfig withLength(Distance length)
  {
    this.length = Optional.ofNullable(length);
    return this;
  }

  /**
   * Configure the MOI directly instead of estimating it with the length and mass of the {@link DifferentialMechanism}
   * for simulation.
   *
   * @param length Length of the {@link DifferentialMechanism}.
   * @param weight Weight of the {@link DifferentialMechanism}
   * @return {@link DifferentialMechanismConfig} for chaining.
   */
  public DifferentialMechanismConfig withMOI(Distance length, Mass weight)
  {
    this.length = Optional.ofNullable(length);
    this.MOI = OptionalDouble.of(SingleJointedArmSim.estimateMOI(length.in(Meters), weight.in(Kilograms)));
    return this;
  }

  /**
   * Configure telemetry for the {@link yams.mechanisms.positional.Pivot} mechanism.
   *
   * @param telemetryName      Telemetry NetworkTable name to appear under "SmartDashboard/"
   * @param telemetryVerbosity Telemetry verbosity to apply.
   * @return {@link DifferentialMechanismConfig} for chaining.
   */
  public DifferentialMechanismConfig withTelemetry(String telemetryName, TelemetryVerbosity telemetryVerbosity)
  {
    this.telemetryName = Optional.ofNullable(telemetryName);
    this.telemetryVerbosity = Optional.ofNullable(telemetryVerbosity);
    return this;
  }

  /**
   * Set the differential mechanism position configuration.
   *
   * @param mechanismPositionConfig {@link MechanismPositionConfig} for the {@link DifferentialMechanism}
   * @return {@link DifferentialMechanismConfig} for chaining
   */
  public DifferentialMechanismConfig withMechanismPositionConfig(MechanismPositionConfig mechanismPositionConfig)
  {
    this.mechanismPositionConfig = mechanismPositionConfig;
    return this;
  }

  /**
   * Aet the gearing for the differential mechanism if it isnt 1:1. Separate from gearing to the bevel gears.
   *
   * @param gearing Gearing for the bevel gears.
   * @return {@link DifferentialMechanismConfig} for chaining
   */
  public DifferentialMechanismConfig withGearing(MechanismGearing gearing)
  {
    twistGearing = gearing;
    return this;
  }


  /**
   * Set the {@link DifferentialMechanism} twist starting position.
   *
   * @param startingPosition Starting position of the {@link DifferentialMechanism} twist.
   * @return {@link DifferentialMechanismConfig} for chaining
   */
  public DifferentialMechanismConfig withTwistStartingPosition(Angle startingPosition)
  {
    startingTwistAngle = Optional.ofNullable(startingPosition);
    setStartingEncoderPositions();
    return this;
  }

  /**
   * Setup suppliers for absolute encoders on the {@link DifferentialMechanism}
   *
   * @param twistSupplier {@link Supplier<Angle>} for the twist.
   * @param tiltSupplier  {@link Supplier<Angle>} for the tilt.
   * @return {@link DifferentialMechanismConfig} for chaining.
   */
  public DifferentialMechanismConfig withAngleSuppliers(Supplier<Angle> twistSupplier, Supplier<Angle> tiltSupplier)
  {
    this.twistAngle = Optional.ofNullable(twistSupplier);
    this.tiltAngle = Optional.ofNullable(tiltSupplier);
    return this;
  }

  /**
   * Set the {@link DifferentialMechanism} starting positions.
   *
   * @param tilt  {@link Angle} of the tilt.
   * @param twist {@link Angle} of the twist.
   * @return {@link DifferentialMechanismConfig} for chaining.
   */
  public DifferentialMechanismConfig withStartingPosition(Angle tilt, Angle twist)
  {
    startingTiltAngle = Optional.ofNullable(tilt);
    startingTwistAngle = Optional.ofNullable(twist);
    setStartingEncoderPositions();
    return this;
  }

  /**
   * Set the {@link DifferentialMechanism} tilt starting position.
   *
   * @param startingPosition Starting position of the {@link DifferentialMechanism} tilt.
   * @return {@link DifferentialMechanismConfig} for chaining
   */
  public DifferentialMechanismConfig withTiltStartingPosition(Angle startingPosition)
  {
    startingTiltAngle = Optional.ofNullable(startingPosition);
    setStartingEncoderPositions();
    return this;
  }

  /**
   * Apply config changes from this class to the {@link SmartMotorController}
   *
   * @return {@link SmartMotorController#applyConfig(SmartMotorControllerConfig)} result.
   */
  public boolean applyConfig()
  {
    return leftMotorController.orElseThrow().applyConfig(leftMotorController.orElseThrow().getConfig()) &&
           rightMotorController.orElseThrow().applyConfig(rightMotorController.orElseThrow().getConfig());
  }


  /**
   * Get the moment of inertia for the {@link DifferentialMechanism} simulation.
   *
   * @return Moment of Inertia.
   */
  public double getMOI()
  {
    if (MOI.isPresent())
    {
      return MOI.getAsDouble();
    }
    throw new DifferentialMechanismConfigurationException("Differential Mechanism Twist MOI must be set!",
                                                          "Cannot get the MOI!",
                                                          ".withMOI()");
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
   * Get the starting angle of the {@link DifferentialMechanism} tilt.
   *
   * @return {@link Angle} of the {@link DifferentialMechanism} tilt.
   */
  public Optional<Angle> getTiltStartingAngle()
  {
    return startingTiltAngle;
  }

  /**
   * Get starting tilt {@link Angle}.
   *
   * @return Tilt {@link Angle}.
   */
  public Optional<Angle> getStartingTiltAngle()
  {
    return startingTiltAngle;
  }

  /**
   * Get starting twist {@link Angle}.
   *
   * @return Twist {@link Angle}.
   */
  public Optional<Angle> getStartingTwistAngle()
  {
    return startingTwistAngle;
  }

  /**
   * Get the starting angle of the {@link DifferentialMechanism} twist.
   *
   * @return {@link Angle} of the {@link DifferentialMechanism} twist.
   */
  public Optional<Angle> getTwistStartingAngle()
  {
    return startingTwistAngle;
  }

  /**
   * Get the angle supplier for the attached absolute encoder, if it exists.
   *
   * @return {@link Angle} supplier for the given absolute encoder attached to the twist.
   */
  public Optional<Supplier<Angle>> getTwistAngleSupplier()
  {
    return twistAngle;
  }

  /**
   * Get the length of the {@link DifferentialMechanism}
   *
   * @return {@link Distance} of {@link DifferentialMechanism}
   */
  public Optional<Distance> getLength()
  {
    return length;
  }

  /**
   * The gearing for the twist in the {@link DifferentialMechanism}
   *
   * @return {@link MechanismGearing} of the twist.
   */
  public MechanismGearing getTwistGearing()
  {
    return twistGearing;
  }

  /**
   * Get the angle supplier for the attached absolute encoder, if it exists.
   *
   * @return {@link Angle} supplier for the given absolute encoder attached to the tilt.
   */
  public Optional<Supplier<Angle>> getTiltAngleSupplier()
  {
    return tiltAngle;
  }

  /**
   * Get the left {@link SmartMotorController} of the {@link DifferentialMechanism}
   *
   * @return left {@link SmartMotorController}
   */
  public SmartMotorController getLeftMotorController()
  {
    return leftMotorController.orElseThrow();
  }

  /**
   * Get the right {@link SmartMotorController} of the {@link DifferentialMechanism}
   *
   * @return right {@link SmartMotorController}
   */
  public SmartMotorController getRightMotorController()
  {
    return rightMotorController.orElseThrow();
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
   * Get the {@link MechanismPositionConfig} associated with this {@link DifferentialMechanismConfig}.
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
