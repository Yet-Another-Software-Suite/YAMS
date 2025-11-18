package yams.mechanisms.config;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * Swerve Module
 **/
public class SwerveModuleConfig
{

  /**
   * {@link SmartMotorController} for the {@link yams.mechanisms.swerve.SwerveModule}
   */
  private final SmartMotorController         driveMotor;
  /**
   * {@link SmartMotorController} for the {@link yams.mechanisms.swerve.SwerveModule}
   */
  private final SmartMotorController         azimuthMotor;
  /**
   * Telemetry name.
   */
  private       Optional<String>             telemetryName                 = Optional.empty();
  /**
   * Telemetry verbosity
   */
  private       Optional<TelemetryVerbosity> telemetryVerbosity            = Optional.empty();
  /**
   * Absolute encoder supplier for the azimuth {@link SmartMotorController}.
   */
  private       Optional<Supplier<Angle>>    absoluteEncoderSupplier       = Optional.empty();
  /**
   * Absolute encoder offset for the azimuth {@link SmartMotorController} to 0 with the bevel facing left.
   */
  private       Optional<Angle>              absoluteEncoderOffset         = Optional.empty();
  /**
   * Gearbox for the absolute encoder.
   */
  private       GearBox                      absoluteEncoderGearbox        = new GearBox(new double[]{1});
  /**
   * Swerve module state optimization using
   * {@link edu.wpi.first.math.kinematics.SwerveModuleState#optimize(Rotation2d)}.
   */
  private boolean                  swerveModuleStateOptimization = true;
  /**
   * Swerve module cosine compensation.
   */
  private boolean                  cosineCompensation            = false;
  /**
   * Coupling ratio for the {@link yams.mechanisms.swerve.SwerveModule}.
   */
  private GearBox                  couplingRatio;
  /**
   * Swerve module minimum velocity.
   */
  private Optional<LinearVelocity> minimumVelocity               = Optional.empty();
  /**
   * Distance from the center of rotation for the {@link yams.mechanisms.swerve.SwerveModule}.
   */
  private       Optional<Translation2d>      distanceFromCenterOfRotation  = Optional.empty();

  /**
   * Create the {@link SwerveModuleConfig} for the {@link yams.mechanisms.swerve.SwerveModule}
   *
   * @param drive   Drive motor controller.
   * @param azimuth Azimuth motor controller.
   */
  public SwerveModuleConfig(SmartMotorController drive, SmartMotorController azimuth)
  {
    this.driveMotor = drive;
    this.azimuthMotor = azimuth;
  }

  /**
   * Cosine compensation for the {@link yams.mechanisms.swerve.SwerveModule}, adjusting the velocity by the cosine of
   * the (current_angle-desired_angle).
   *
   * @param compensate Enable or disable cosine compensation.
   * @return {@link SwerveModuleConfig} for chaining.
   */
  public SwerveModuleConfig withCosineCompensation(boolean compensate)
  {
    this.cosineCompensation = compensate;
    return this;
  }
  // TODO: Add coupling ratio

  /**
   * Set the absolute encoder for the azimuth {@link SmartMotorController} if it's the SAME VENDOR, only.
   *
   * @param absoluteEncoder Same vendor, absolute encoder for the {@link SmartMotorController}
   * @return {@link SwerveModuleConfig} for chaining.
   */
  public SwerveModuleConfig withAbsoluteEncoder(Object absoluteEncoder)
  {
    azimuthMotor.getConfig().withExternalEncoder(absoluteEncoder);
    return this;
  }

  /**
   * Set the distance from the center of rotation for the {@link yams.mechanisms.swerve.SwerveModule}.
   *
   * @param front Distance from the front of the robot, will be converted to Meters. (The X location)
   * @param left  Distance from the left of the robot, will be converted to Meters. (The Y location)
   * @return {@link SwerveModuleConfig} for chaining.
   */
  public SwerveModuleConfig withDistanceFromCenterOfRotation(Distance front, Distance left)
  {
    distanceFromCenterOfRotation = Optional.of(new Translation2d(front, left));
    return this;
  }


  /**
   * Set the location for the {@link yams.mechanisms.swerve.SwerveModule} in Meters from the center of rotation.
   *
   * @param location Location of the {@link yams.mechanisms.swerve.SwerveModule} in meters from the center of rotation.
   * @return {@link SwerveModuleConfig} for chaining.
   */
  public SwerveModuleConfig withLocation(Translation2d location)
  {
    distanceFromCenterOfRotation = Optional.of(location);
    return this;
  }

  /**
   * Set the distance from the center of rotation for the {@link yams.mechanisms.swerve.SwerveModule}.
   *
   * @param front Distance from the front of the robot, will be converted to meters. (The X location)
   * @param left  Distance from the left of the robot, will be converted to meters. (The Y location)
   * @return {@link SwerveModuleConfig} for chaining.
   */
  public SwerveModuleConfig withLocation(Distance front, Distance left)
  {
    return withDistanceFromCenterOfRotation(front, left);
  }

  /**
   * Set the location for the {@link yams.mechanisms.swerve.SwerveModule} using polar coordinates.
   *
   * @param distance Distance from the center of rotation will be converted to meters.
   * @param angle    Angle from the center of rotation.
   * @return {@link SwerveModuleConfig} for chaining.
   */
  public SwerveModuleConfig withLocation(Distance distance, Angle angle)
  {
    distanceFromCenterOfRotation = Optional.of(new Translation2d(distance.in(Meters), new Rotation2d(angle)));
    return this;
  }

  /**
   * Supply a non-alike vendor absolute encoder for the azimuth {@link SmartMotorController}.
   *
   * @param locationProvider Provides the Rotation for the azimuth {@link SmartMotorController}.
   * @return {@link SwerveModuleConfig} for chaining.
   */
  public SwerveModuleConfig withAbsoluteEncoder(Supplier<Angle> locationProvider)
  {
    absoluteEncoderSupplier = Optional.ofNullable(locationProvider);
    return this;
  }

  /**
   * Supply a non-alike vendor absolute encoder for the azimuth {@link SmartMotorController}.
   *
   * @param rotationSupplier Provides the Rotation for the azimuth {@link SmartMotorController}.
   * @return {@link SwerveModuleConfig} for chaining.
   */
  public SwerveModuleConfig withAbsoluteEncoder(DoubleSupplier rotationSupplier)
  {
    if (rotationSupplier != null)
    {return withAbsoluteEncoder(() -> Rotations.of(rotationSupplier.getAsDouble()));}
    return this;
  }

  /**
   * Set the gearing of the absolute encoder if it's not 1:1.
   *
   * @param gearing {@link GearBox} for the absolute encoder to get the azimuth angle.
   * @return {@link SwerveModuleConfig} for chaining.
   */
  public SwerveModuleConfig withAbsoluteEncoderGearing(GearBox gearing)
  {
    absoluteEncoderGearbox = gearing;
    SmartMotorControllerConfig azimuthConfig = azimuthMotor.getConfig();
    if (azimuthConfig.getExternalEncoder().isPresent())
    {azimuthConfig.withExternalEncoderGearing(new MechanismGearing(gearing));}
    return this;
  }

  /**
   * Set the absolute encoder offset for the azimuth {@link SmartMotorController} so that the absolute encoder reads 0
   * while the wheel is facing forwards and bevel to the left.
   *
   * @param offset Offset for the absolute encoder.
   * @return {@link SwerveModuleConfig} for chaining.
   */
  public SwerveModuleConfig withAbsoluteEncoderOffset(Angle offset)
  {
    absoluteEncoderOffset = Optional.ofNullable(offset);

    SmartMotorControllerConfig azimuthConfig = azimuthMotor.getConfig();
    if (azimuthConfig.getExternalEncoder().isPresent())
    {
      azimuthConfig.withExternalEncoderZeroOffset(offset);
    }
    return this;
  }

  /**
   * Set the wheel radius for the {@link SmartMotorController}, ideally should be set inside of the drive motor
   * {@link SmartMotorControllerConfig}
   *
   * @param radius Radius of the wheel.
   * @return {@link SwerveModuleConfig} for chaining.
   */
  public SwerveModuleConfig withWheelRadius(Distance radius)
  {
    SmartMotorControllerConfig driveConfig = driveMotor.getConfig();
    driveConfig.withMechanismCircumference(radius.times(2));
    return this;
  }

  /**
   * Set the wheel diameter for the {@link SmartMotorController}, ideally should be set inside of the drive motor
   * {@link SmartMotorControllerConfig}.
   *
   * @param diameter Diameter of the wheel.
   * @return {@link SwerveModuleConfig} for chaining.
   */
  public SwerveModuleConfig withWheelDiameter(Distance diameter)
  {
    SmartMotorControllerConfig driveConfig = driveMotor.getConfig();
    driveConfig.withMechanismCircumference(diameter);
    return this;
  }

  /**
   * Set the minimum velocity for the {@link SmartMotorController}.
   *
   * @param speed Minimum velocity for the {@link SmartMotorController}.
   * @return {@link SwerveModuleConfig} for chaining.
   */
  public SwerveModuleConfig withMinimumVelocity(LinearVelocity speed)
  {
    minimumVelocity = Optional.ofNullable(speed);
    return this;
  }

  /**
   * Use {@link edu.wpi.first.math.kinematics.SwerveModuleState#optimize(Rotation2d)} to optimize each state.
   *
   * @param swerveModuleStateOptimization True to enable optimization, false otherwise.
   * @return {@link SwerveModuleConfig} for chaining.
   */
  public SwerveModuleConfig withOptimization(boolean swerveModuleStateOptimization)
  {
    this.swerveModuleStateOptimization = swerveModuleStateOptimization;
    return this;
  }

  /**
   * Configure telemetry for the {@link yams.mechanisms.swerve.SwerveModule} mechanism.
   *
   * @param telemetryName      Telemetry NetworkTable name to appear under "SmartDashboard/"
   * @param telemetryVerbosity Telemetry verbosity to apply.
   * @return {@link ArmConfig} for chaining.
   */
  public SwerveModuleConfig withTelemetry(String telemetryName, TelemetryVerbosity telemetryVerbosity)
  {
    this.telemetryName = Optional.ofNullable(telemetryName);
    this.telemetryVerbosity = Optional.ofNullable(telemetryVerbosity);
    return this;
  }

  /**
   * Get the absolute encoder angle for the azimuth {@link SmartMotorController}.
   *
   * @return Absolute encoder {@link Angle}.
   */
  public Angle getAbsoluteEncoderAngle()
  {
    return absoluteEncoderSupplier.map(angleSupplier -> angleSupplier.get()
                                                                     .times(absoluteEncoderGearbox.getInputToOutputConversionFactor())
                                                                     .minus(absoluteEncoderOffset.orElse(Rotations.of(0))))
                                  .orElse(azimuthMotor.getMechanismPosition());
  }

  /**
   * Enable or disable state optimization.
   *
   * @return True if optimization is enabled, false otherwise.
   */
  public boolean getStateOptimization()
  {
    return swerveModuleStateOptimization;
  }

  /**
   * Get the drive {@link SmartMotorController} for the {@link yams.mechanisms.swerve.SwerveModule}.
   *
   * @return {@link SmartMotorController} for the drive motor.
   */
  public SmartMotorController getDriveMotor()
  {
    return driveMotor;
  }

  /**
   * Get the azimuth {@link SmartMotorController} for the {@link yams.mechanisms.swerve.SwerveModule}.
   *
   * @return {@link SmartMotorController} for the azimuth motor.
   */
  public SmartMotorController getAzimuthMotor()
  {
    return azimuthMotor;
  }

  /**
   * Get the cosine-compensated velocity to set the swerve module to.
   *
   * @param desiredState Desired {@link SwerveModuleState} to use.
   * @return Cosine compensated velocity in meters/second.
   */
  private double getCosineCompensatedVelocity(SwerveModuleState desiredState)
  {
    double cosineScalar = 1.0;
    // Taken from the CTRE SwerveModule class.
    // https://api.ctr-electronics.com/phoenix6/release/java/src-html/com/ctre/phoenix6/mechanisms/swerve/SwerveModule.html#line.46
    /* From FRC 900's whitepaper, we add a cosine compensator to the applied drive velocity */
    /* To reduce the "skew" that occurs when changing direction */
    /* If error is close to 0 rotations, we're already there, so apply full power */
    /* If the error is close to 0.25 rotations, then we're 90 degrees, so movement doesn't help us at all */
    cosineScalar = Rotation2d.fromDegrees(desiredState.angle.getDegrees())
                             .minus(Rotation2d.fromRotations(getAbsoluteEncoderAngle().in(Rotations)))
                             .getCos(); // TODO: Investigate angle modulus by 180.
    /* Make sure we don't invert our drive, even though we shouldn't ever target over 90 degrees anyway */
    if (cosineScalar < 0.0)
    {
      cosineScalar = 1;
    }

    return desiredState.speedMetersPerSecond * cosineScalar;
  }


  /**
   * Get the optimized {@link SwerveModuleState} applying all optional optimizations.
   *
   * @param state {@link SwerveModuleState} to optimize.
   * @return {@link SwerveModuleState} optimized.
   */
  public SwerveModuleState getOptimizedState(SwerveModuleState state)
  {
    if (minimumVelocity.isPresent())
    {
      if (MetersPerSecond.of(Math.abs(state.speedMetersPerSecond)).lte(minimumVelocity.get()))
      {
//        state = new SwerveModuleState(0, state.angle);
        state = new SwerveModuleState(0, new Rotation2d(getAbsoluteEncoderAngle()));

      }
    }
    if (swerveModuleStateOptimization)
    {
      state.optimize(new Rotation2d(getAbsoluteEncoderAngle()));
    }
    if (cosineCompensation)
    {
      state.speedMetersPerSecond *= getCosineCompensatedVelocity(state);
    }
    return state;
  }

  /**
   * Get the telemetry name for the {@link yams.mechanisms.swerve.SwerveModule}.
   *
   * @return Telemetry name for the {@link yams.mechanisms.swerve.SwerveModule}.
   */
  public Optional<String> getTelemetryName()
  {
    return telemetryName;
  }

  /**
   * Get the telemetry verbosity for the {@link yams.mechanisms.swerve.SwerveModule}.
   *
   * @return {@link TelemetryVerbosity} for the {@link yams.mechanisms.swerve.SwerveModule}.
   */
  public Optional<TelemetryVerbosity> getTelemetryVerbosity()
  {
    return telemetryVerbosity;
  }

  /**
   * Get the location of the {@link yams.mechanisms.swerve.SwerveModule} in meters from the center of rotation.
   *
   * @return {@link Translation2d} of the {@link yams.mechanisms.swerve.SwerveModule}
   */
  public Optional<Translation2d> getLocation()
  {
    return distanceFromCenterOfRotation;
  }

}
