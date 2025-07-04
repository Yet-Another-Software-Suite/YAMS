package yams.motorcontrollers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Optional;
import java.util.OptionalInt;
import yams.exceptions.SmartMotorControllerConfigurationException;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.telemetry.SmartMotorControllerTelemetryConfig;

/**
 * Smart motor controller config.
 */
public class SmartMotorControllerConfig
{

  /**
   * Subsystem that the {@link SmartMotorController} controls.
   */
  private final Subsystem         subsystem;
  /**
   * External encoder.
   */
  private       Optional<Object>  externalEncoder                  = Optional.empty();
  /**
   * External encoder inversion state.
   */
  private       boolean           externalEncoderInverted          = false;
  /**
   * Follower motors and inversion.
   */
  private Optional<Pair<Object, Boolean>[]> followers                          = Optional.empty();
  /**
   * Simple feedforward for the motor controller.
   */
  private Optional<SimpleMotorFeedforward>  simpleFeedforward                  = Optional.empty();
  /**
   * Elevator feedforward for the motor controller.
   */
  private Optional<ElevatorFeedforward>     elevatorFeedforward                = Optional.empty();
  /**
   * Arm feedforward for the motor controller.
   */
  private Optional<ArmFeedforward>          armFeedforward                     = Optional.empty();
  /**
   * Controller for the {@link SmartMotorController}.
   */
  private Optional<ProfiledPIDController>   controller                         = Optional.empty();
  /**
   * Controller for the {@link SmartMotorController}.
   */
  private Optional<PIDController>           simpleController                   = Optional.empty();
  /**
   * Gearing for the {@link SmartMotorController}.
   */
  private MechanismGearing                  gearing;
  /**
   * External encoder gearing, defaults to 1:1.
   */
  private MechanismGearing externalEncoderGearing = new MechanismGearing(new GearBox(
      new double[]{1.0}));
  /**
   * Mechanism Circumference for distance calculations.
   */
  private Optional<Distance>                mechanismCircumference             = Optional.empty();
  /**
   * PID Controller period for robot controller based PIDs
   */
  private Time                              controlPeriod                      = Milliseconds.of(20);
  /**
   * Open loop ramp rate, amount of time to go from 0 to 100 speed..
   */
  private       Time              openLoopRampRate                 = Seconds.of(0);
  /**
   * Closed loop ramp rate, amount of time to go from 0 to 100 speed.
   */
  private       Time              closeLoopRampRate                = Seconds.of(0);
  /**
   * Set the stator current limit in Amps for the {@link SmartMotorController}
   */
  private OptionalInt                       statorStallCurrentLimit            = OptionalInt.empty();
  /**
   * The supply current limit in Amps for the {@link SmartMotorController}
   */
  private OptionalInt                       supplyStallCurrentLimit            = OptionalInt.empty();
  /**
   * The voltage compensation.
   */
  private       Optional<Voltage> voltageCompensation              = Optional.empty();
  /**
   * Set the {@link MotorMode} for the {@link SmartMotorController}.
   */
  private Optional<MotorMode>               idleMode                           = Optional.empty();
  /**
   * Mechanism lower limit to prevent movement below.
   */
  private Optional<Angle>                   mechanismLowerLimit                = Optional.empty();
  /**
   * High distance soft limit to prevent movement above.
   */
  private Optional<Angle>                   mechanismUpperLimit                = Optional.empty();
  /**
   * Name for the {@link SmartMotorController} telemetry.
   */
  private Optional<String>                  telemetryName                      = Optional.empty();
  /**
   * Telemetry verbosity setting.
   */
  private Optional<TelemetryVerbosity>      verbosity                          = Optional.empty();
  /**
   * Optional config for custom telemetry setup.
   */
  private Optional<Object>                  specifiedTelemetryConfig           = Optional.empty();
  /**
   * Zero offset of the {@link SmartMotorController}
   */
  private Optional<Angle>                   zeroOffset                         = Optional.empty();
  /**
   * Temperature cutoff for the {@link SmartMotorController} to prevent running if above.
   */
  private Optional<Temperature>             temperatureCutoff                  = Optional.empty();
  /**
   * The encoder readings are inverted.
   */
  private boolean                           encoderInverted                    = false;
  /**
   * The motor is inverted.
   */
  private boolean                           motorInverted                      = false;
  /**
   * Use the provided external encoder if set.
   */
  private boolean                           useExternalEncoder                 = true;
  /**
   * {@link SmartMotorController} starting angle.
   */
  private Optional<Angle>                   startingPosition                   = Optional.empty();
  /**
   * Maximum voltage output for the motor controller while using the closed loop controller.
   */
  private Optional<Voltage>                 closedLoopControllerMaximumVoltage = Optional.empty();
  /**
   * Feedback synchronization threshhold.
   */
  private       Optional<Angle>   feedbackSynchronizationThreshold = Optional.empty();
  /**
   * The motor controller mode.
   */
  private       ControlMode       motorControllerMode              = ControlMode.CLOSED_LOOP;
  /**
   * Encoder discontinuity point.
   */
  private       Optional<Angle>   maxDiscontinuityPoint            = Optional.empty();
  /**
   * Encoder discontinuity point.
   */
  private       Optional<Angle>   minDiscontinuityPoint            = Optional.empty();

  /**
   * Construct the {@link SmartMotorControllerConfig} for the {@link Subsystem}
   *
   * @param subsystem {@link Subsystem} to use.
   */
  public SmartMotorControllerConfig(Subsystem subsystem)
  {
    this.subsystem = subsystem;
  }

  /**
   * Set the external encoder inversion state.
   *
   * @param externalEncoderInverted External encoder inversion state.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withExternalEncoderInverted(boolean externalEncoderInverted)
  {
    this.externalEncoderInverted = externalEncoderInverted;
    return this;
  }

  /**
   * Set the control mode for the {@link SmartMotorController}
   *
   * @param controlMode {@link ControlMode} to apply.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withControlMode(ControlMode controlMode)
  {
    this.motorControllerMode = controlMode;
    return this;
  }

  /**
   * Set the feedback synchronization threshhold so the relative encoder synchronizes with the absolute encoder at this
   * point.
   *
   * @param angle {@link Angle} to exceed.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withFeedbackSynchronizationThreshold(Angle angle)
  {
    if (mechanismCircumference.isPresent())
    {
      throw new SmartMotorControllerConfigurationException(
          "Auto-synchronization is unavailable when using distance based mechanisms",
          "Cannot set synchronization threshold.",
          "withMechanismCircumference(Distance) should be removed.");
    }
    feedbackSynchronizationThreshold = angle == null ? Optional.empty() : Optional.of(angle);
    return this;
  }

  /**
   * Set the closed loop maximum voltage output.
   *
   * @param volts Maximum voltage.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withClosedLoopControllerMaximumVoltage(Voltage volts)
  {
    closedLoopControllerMaximumVoltage = volts == null ? Optional.empty() : Optional.of(volts);
    return this;
  }

  /**
   * Set the starting angle of the {@link SmartMotorController}
   *
   * @param startingAngle Starting Mechanism Angle.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withStartingPosition(Angle startingAngle)
  {
    this.startingPosition = startingAngle == null ? Optional.empty() : Optional.of(startingAngle);
    return this;
  }

  /**
   * Set the starting angle of the {@link SmartMotorController}
   *
   * @param startingAngle Starting Mechanism Distance.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withStartingPosition(Distance startingAngle)
  {
    return withStartingPosition(convertToMechanism(startingAngle));
  }

  /**
   * Set the external encoder to be the primary feedback device for the PID controller.
   *
   * @param useExternalEncoder External encoder as primary feedback device for the PID controller.
   * @return {@link SmartMotorControllerConfig} for chaining
   */
  public SmartMotorControllerConfig withUseExternalFeedbackEncoder(boolean useExternalEncoder)
  {
    this.useExternalEncoder = useExternalEncoder;
    return this;
  }

  /**
   * Set the encoder inversion state.
   *
   * @param inverted Encoder inversion state.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withEncoderInverted(boolean inverted)
  {
    this.encoderInverted = inverted;
    return this;
  }

  /**
   * Set the motor inversion state.
   *
   * @param motorInverted Motor inversion state.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withMotorInverted(boolean motorInverted)
  {
    this.motorInverted = motorInverted;
    return this;
  }

  /**
   * Set the {@link Temperature} cut off for the {@link SmartMotorController}/
   *
   * @param cutoff maximum {@link Temperature}
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withTemperatureCutoff(Temperature cutoff)
  {
    temperatureCutoff = cutoff == null ? Optional.empty() : Optional.of(cutoff);
    return this;
  }

  /**
   * Set the zero offset of the {@link SmartMotorController}
   *
   * @param distance Zero offset in distance.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withZeroOffset(Distance distance)
  {
    if (mechanismCircumference.isEmpty())
    {
      throw new SmartMotorControllerConfigurationException("Mechanism circumference is undefined",
                                                           "Cannot set zero offset.",
                                                           "withMechanismCircumference(Distance)");
    }
    zeroOffset = distance == null ? Optional.empty() : Optional.of(convertToMechanism(distance));
    return this;
  }

  /**
   * Set the zero offset of the {@link SmartMotorController}
   *
   * @param angle {@link Angle} to 0.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withZeroOffset(Angle angle)
  {
    zeroOffset = angle == null ? Optional.empty() : Optional.of(angle);
    return this;
  }

  /**
   * Set continuous wrapping for the {@link SmartMotorController}
   *
   * @param bottom Bottom value to wrap to.
   * @param top    Top value to wrap to.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withContinuousWrapping(Angle bottom, Angle top)
  {
    if (mechanismUpperLimit.isPresent() || mechanismLowerLimit.isPresent())
    {
      throw new SmartMotorControllerConfigurationException("Soft limits set while configuring continuous wrapping",
                                                           "Cannot set continuous wrapping",
                                                           "withSoftLimit(Angle,Angle) should be removed");
    }
    if (mechanismCircumference.isPresent())
    {
      throw new SmartMotorControllerConfigurationException("Distance based mechanism used with continuous wrapping",
                                                           "Cannot set continuous wrapping",
                                                           "withMechanismCircumference(Distance) should be removed");
    }
    simpleController.ifPresent(pidController -> pidController.enableContinuousInput(bottom.in(Rotations),
                                                                                    top.in(Rotations)));
    controller.ifPresent(profiledPIDController -> profiledPIDController.enableContinuousInput(bottom.in(Rotations),
                                                                                              top.in(Rotations)));
    if (simpleController.isEmpty() && controller.isEmpty())
    {
      throw new SmartMotorControllerConfigurationException("No PID controller used",
                                                           "Cannot set continuous wrapping!",
                                                           "withClosedLoopController()");
    }

    maxDiscontinuityPoint = Optional.of(top);
    minDiscontinuityPoint = Optional.of(bottom);
    return this;
  }

  /**
   * Set the closed loop tolerance of the mechanism controller.
   *
   * @param tolerance Closed loop controller tolerance
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withClosedLoopTolerance(Angle tolerance)
  {
    if (tolerance != null)
    {
      controller.ifPresent(profiledPIDController -> profiledPIDController.setTolerance(tolerance.in(Rotations)));
      simpleController.ifPresent(pidController -> pidController.setTolerance(tolerance.in(Rotations)));
      if (controller.isEmpty() && simpleController.isEmpty())
      {
        throw new SmartMotorControllerConfigurationException("No PID controller used",
                                                             "Cannot set tolerance!",
                                                             "withClosedLoopController()");
      }
    }
    return this;
  }

  /**
   * Set the {@link SmartMotorController} closed loop controller tolerance via distance.
   *
   * @param tolerance {@link Distance} tolerance.
   * @return {@link SmartMotorControllerConfig} fpr chaining.
   */
  public SmartMotorControllerConfig withClosedLoopTolerance(Distance tolerance)
  {
    if (mechanismCircumference.isEmpty())
    {
      throw new SmartMotorControllerConfigurationException("Mechanism circumference is undefined",
                                                           "Closed loop tolerance cannot be set.",
                                                           "withMechanismCircumference(Distance)");
    }
    if (tolerance != null)
    {
      controller.ifPresent(profiledPIDController -> profiledPIDController.setTolerance(tolerance.in(Meters)));
      simpleController.ifPresent(pidController -> pidController.setTolerance(tolerance.in(Meters)));
      if (controller.isEmpty() && simpleController.isEmpty())
      {
        throw new SmartMotorControllerConfigurationException("No PID controller used",
                                                             "Cannot set tolerance!",
                                                             "withClosedLoopController()");
      }
    }
    return this;
  }

  /**
   * Set the telemetry for the {@link SmartMotorController}
   *
   * @param telemetryName Name for the {@link SmartMotorController}
   * @param verbosity     Verbosity of the Telemetry for the {@link SmartMotorController}
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withTelemetry(String telemetryName, TelemetryVerbosity verbosity)
  {
    this.telemetryName = telemetryName == null ? Optional.empty() : Optional.of(telemetryName);
    this.verbosity = verbosity == null ? Optional.empty() : Optional.of(verbosity);
    return this;
  }

  /**
   * Set the telemetry for the {@link SmartMotorController}
   *
   * @param verbosity Verbosity of the Telemetry for the {@link SmartMotorController}
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withTelemetry(TelemetryVerbosity verbosity)
  {
    this.telemetryName = Optional.of("motor");
    this.verbosity = Optional.of(verbosity);
    return this;
  }

  /**
   * Set the telemetry for the {@link SmartMotorController} with a {@link SmartMotorControllerTelemetryConfig}
   *
   * @param telemetryName Name for the {@link SmartMotorController}
   * @param telemetryConfig Config that specifies what to log.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withSpecificTelemetry(String telemetryName, SmartMotorControllerTelemetryConfig telemetryConfig) {
    this.telemetryName = telemetryName == null ? Optional.empty() : Optional.of(telemetryName);
    this.verbosity = Optional.of(TelemetryVerbosity.HIGH);
    this.specifiedTelemetryConfig = telemetryConfig == null ? Optional.empty() : Optional.of(telemetryConfig);
    return this;
  }

  public Optional<Object> getSmartControllerTelemetryConfig() {
    return specifiedTelemetryConfig;
  }

  /**
   * Get the stator stall current limit.
   *
   * @return Stator current limit.
   */
  public OptionalInt getStatorStallCurrentLimit()
  {
    return statorStallCurrentLimit;
  }

  /**
   * Set the distance soft limits.
   *
   * @param low  Low distance soft limit.
   * @param high High distance soft limit.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withSoftLimit(Distance low, Distance high)
  {
    if (mechanismCircumference.isEmpty())
    {
      throw new SmartMotorControllerConfigurationException("Mechanism circumference is undefined",
                                                           "Cannot set soft limits.",
                                                           "withMechanismCircumference(Distance)");
    }
    mechanismLowerLimit = low == null ? Optional.empty() : Optional.of(Rotations.of(
        low.in(Meters) / mechanismCircumference.get().in(Meters)));
    mechanismUpperLimit = high == null ? Optional.empty() : Optional.of(Rotations.of(
        high.in(Meters) / mechanismCircumference.get().in(Meters)));

    return this;
  }

  /**
   * Set the angle soft limits.
   *
   * @param low  Low angle soft limit.
   * @param high High angle soft limit.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withSoftLimit(Angle low, Angle high)
  {
    mechanismLowerLimit = low == null ? Optional.empty() : Optional.of(low);
    mechanismUpperLimit = high == null ? Optional.empty() : Optional.of(high);
    return this;
  }

  /**
   * Get the supply stall current limit.
   *
   * @return Supply stall current limit.
   */
  public OptionalInt getSupplyStallCurrentLimit()
  {
    return supplyStallCurrentLimit;
  }

  /**
   * Get the voltage compensation for the {@link SmartMotorController}
   *
   * @return Ideal voltage
   */
  public Optional<Voltage> getVoltageCompensation()
  {
    return voltageCompensation;
  }

  /**
   * Get the idle mode for the {@link SmartMotorController}
   *
   * @return {@link MotorMode}
   */
  public Optional<MotorMode> getIdleMode()
  {
    return idleMode;
  }


  /**
   * Lower limit of the mechanism.
   *
   * @return Lower angle soft limit.
   */
  public Optional<Angle> getMechanismLowerLimit()
  {
    return mechanismLowerLimit;
  }

  /**
   * Upper limit of the mechanism.
   *
   * @return Higher angle soft limit.
   */
  public Optional<Angle> getMechanismUpperLimit()
  {
    return mechanismUpperLimit;
  }

  /**
   * Set the {@link SmartMotorController} to brake or coast mode.
   *
   * @param idleMode {@link MotorMode} idle mode
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withIdleMode(MotorMode idleMode)
  {
    this.idleMode = idleMode == null ? Optional.empty() : Optional.of(idleMode);
    return this;
  }

  /**
   * Set the voltage compensation for the {@link SmartMotorController}
   *
   * @param voltageCompensation Ideal voltage value.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withVoltageCompensation(Voltage voltageCompensation)
  {
    this.voltageCompensation =
        voltageCompensation == null ? Optional.empty() : Optional.of(voltageCompensation);
    return this;
  }

  /**
   * Set the follower motors of the {@link SmartMotorController}
   *
   * @param followers Base motor types (NOT {@link SmartMotorController}!) to configure as followers, must be same brand
   *                  as the {@link SmartMotorController} with inversion..
   * @return {@link SmartMotorControllerConfig} for chaining
   */
  @SafeVarargs
  public final SmartMotorControllerConfig withFollowers(Pair<Object, Boolean>... followers)
  {
    this.followers = followers == null ? Optional.empty() : Optional.of(followers);
    return this;
  }

  /**
   * Clear the follower motors so they are not reapplied
   */
  public void clearFollowers()
  {
    this.followers = Optional.empty();
  }

  /**
   * Set the stall stator current limit for the {@link SmartMotorController}
   *
   * @param stallCurrent Stall stator current limit for the {@link SmartMotorController}.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withStatorCurrentLimit(Current stallCurrent)
  {
    this.statorStallCurrentLimit = stallCurrent == null ? OptionalInt.empty() : OptionalInt.of((int) stallCurrent.in(
        Amps));
    return this;
  }

  /**
   * Set the stall supply current limit for the {@link SmartMotorController}
   *
   * @param supplyCurrent Supply current limit.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withSupplyCurrentLimit(Current supplyCurrent)
  {
    this.supplyStallCurrentLimit = supplyCurrent == null ? OptionalInt.empty() : OptionalInt.of((int) supplyCurrent.in(
        Amps));
    return this;
  }

  /**
   * Set the closed loop ramp rate. The ramp rate is the minimum time it should take to go from 0 power to full power in
   * the motor controller while using PID.
   *
   * @param rate time to go from 0 to full throttle.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withClosedLoopRampRate(Time rate)
  {
    this.closeLoopRampRate = rate;
    return this;
  }

  /**
   * Set the open loop ramp rate. The ramp rate is the minimum time it should take to go from 0 power to full power in
   * the motor controller while not using PID.
   *
   * @param rate time to go from 0 to full throttle.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withOpenLoopRampRate(Time rate)
  {
    this.openLoopRampRate = rate;
    return this;
  }

  /**
   * Set the external encoder which is attached to the motor type sent used by {@link SmartMotorController}
   *
   * @param externalEncoder External encoder attached to the {@link SmartMotorController}
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withExternalEncoder(Object externalEncoder)
  {
    this.externalEncoder = externalEncoder == null ? Optional.empty() : Optional.of(externalEncoder);
    return this;
  }

  /**
   * Get the follower motors to the {@link SmartMotorControllerConfig}
   *
   * @return Follower motor list.
   */
  public Optional<Pair<Object, Boolean>[]> getFollowers()
  {
    return followers;
  }

  /**
   * Set the {@link MechanismGearing} for the {@link SmartMotorController}.
   *
   * @param gear {@link MechanismGearing} representing the gearbox and sprockets to the final axis.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withGearing(MechanismGearing gear)
  {
    gearing = gear;
    return this;
  }

  /**
   * Set the mechanism circumference to allow distance calculations on the {@link SmartMotorController}.
   *
   * @param circumference Circumference of the actuating spool or sprocket+chain attached the mechanism actuator.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withMechanismCircumference(Distance circumference)
  {
    if (circumference == null)
    {
      mechanismCircumference = Optional.empty();
    } else
    {
      mechanismCircumference = Optional.of(circumference);
    }
    return this;
  }

  /**
   * Get the mechanism to distance ratio for the {@link SmartMotorController}
   *
   * @return Rotations/Distance ratio to convert mechanism rotations to distance.
   */
  public Optional<Distance> getMechanismCircumference()
  {
    return mechanismCircumference;
  }

  /**
   * Modify the period of the PID controller for the motor controller.
   *
   * @param time Period of the motor controller PID.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withPeriod(Time time)
  {
    controlPeriod = time;
    return this;
  }

  /**
   * Get the {@link ArmFeedforward} if it is set.
   *
   * @return {@link Optional} of the {@link ArmFeedforward}.
   */
  public Optional<ArmFeedforward> getArmFeedforward()
  {
    return armFeedforward;
  }

  /**
   * Configure the {@link ArmFeedforward} for the
   *
   * @param armFeedforward Arm feedforward for the {@link SmartMotorController}
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withFeedforward(ArmFeedforward armFeedforward)
  {
    if (armFeedforward == null)
    {
      this.armFeedforward = Optional.empty();
    } else
    {
      this.elevatorFeedforward = Optional.empty();
      this.simpleFeedforward = Optional.empty();
      this.armFeedforward = Optional.of(armFeedforward);
    }
    return this;
  }

  /**
   * Get the {@link ElevatorFeedforward} {@link Optional}
   *
   * @return {@link ElevatorFeedforward} {@link Optional}
   */
  public Optional<ElevatorFeedforward> getElevatorFeedforward()
  {
    return elevatorFeedforward;
  }

  /**
   * Configure {@link ElevatorFeedforward} for the {@link SmartMotorController}
   *
   * @param elevatorFeedforward {@link ElevatorFeedforward} to set.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withFeedforward(ElevatorFeedforward elevatorFeedforward)
  {
    if (elevatorFeedforward == null)
    {
      this.elevatorFeedforward = Optional.empty();
    } else
    {
      this.armFeedforward = Optional.empty();
      this.simpleFeedforward = Optional.empty();
      this.elevatorFeedforward = Optional.of(elevatorFeedforward);
    }
    return this;
  }

  /**
   * Get the {@link SimpleMotorFeedforward} {@link Optional}.
   *
   * @return {@link SimpleMotorFeedforward} {@link Optional}
   */
  public Optional<SimpleMotorFeedforward> getSimpleFeedforward()
  {
    return simpleFeedforward;
  }


  /**
   * Set the closed loop controller for the {@link SmartMotorController}. The units passed in are in Rotations and
   * outputs are in Rotations.
   *
   * @param controller {@link ProfiledPIDController} to use, the units passed in are in Rotations and output is
   *                   Voltage.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withClosedLoopController(ProfiledPIDController controller)
  {
    this.controller = controller == null ? Optional.empty() : Optional.of(controller);
    this.simpleController = Optional.empty();
    return this;
  }

  /**
   * Set the closed loop controller for the {@link SmartMotorController}. Units are Meters.
   *
   * @param kP KP scalar for the PID Controller.
   * @param kI KI scalar for the PID Controller.
   * @param kD KD scalar for the PID Controller.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withClosedLoopController(double kP, double kI, double kD)
  {
    if (mechanismCircumference.isEmpty())
    {
      throw new SmartMotorControllerConfigurationException("Mechanism circumference is undefined",
                                                           "Closed loop controller cannot be created.",
                                                           "withMechanismCircumference(Distance)");
    }
    this.controller = Optional.empty();
    this.simpleController = Optional.of(new PIDController(kP, kI, kD));
    return this;
  }

  /**
   * Set the closed loop controller for the {@link SmartMotorController}
   *
   * @param controller {@link PIDController} to use.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withClosedLoopController(PIDController controller)
  {
    this.controller = Optional.empty();
    this.simpleController = controller == null ? Optional.empty() : Optional.of(controller);
    return this;
  }

  /**
   * Set the closed loop controller for the {@link SmartMotorController}. Units are Meters.
   *
   * @param kP              KP scalar for the PID Controller.
   * @param kI              KI scalar for the PID Controller.
   * @param kD              KD scalar for the PID Controller.
   * @param maxVelocity     Maximum linear velocity for the Trapazoidal profile.
   * @param maxAcceleration Maximum linear acceleration for the Trapazoidal profile.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withClosedLoopController(double kP, double kI, double kD,
                                                             LinearVelocity maxVelocity,
                                                             LinearAcceleration maxAcceleration)
  {
    if (mechanismCircumference.isEmpty())
    {
      throw new SmartMotorControllerConfigurationException("Mechanism circumference is undefined",
                                                           "Closed loop controller cannot be created.",
                                                           "withMechanismCircumference(Distance)");
    }
    this.simpleController = Optional.empty();
    this.controller = Optional.of(new ProfiledPIDController(kP,
                                                            kI,
                                                            kD,
                                                            new Constraints(maxVelocity.in(MetersPerSecond),
                                                                            maxAcceleration.in(MetersPerSecondPerSecond))));
    return this;
  }

  /**
   * Set the closed loop controller for the {@link SmartMotorController}. Units are Meters.
   *
   * @param kP              KP scalar for the PID Controller.
   * @param kI              KI scalar for the PID Controller.
   * @param kD              KD scalar for the PID Controller.
   * @param maxVelocity     Maximum angular velocity for the Trapazoidal profile.
   * @param maxAcceleration Maximum angular acceleration for the Trapazoidal profile.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withClosedLoopController(double kP, double kI, double kD,
                                                             AngularVelocity maxVelocity,
                                                             AngularAcceleration maxAcceleration)
  {
    this.simpleController = Optional.empty();
    this.controller = Optional.of(new ProfiledPIDController(kP,
                                                            kI,
                                                            kD,
                                                            new Constraints(maxVelocity.in(RotationsPerSecond),
                                                                            maxAcceleration.in(
                                                                                RotationsPerSecondPerSecond))));
    return this;
  }

  /**
   * Get the controller for the {@link SmartMotorController}
   *
   * @return {@link ProfiledPIDController}
   */
  public Optional<ProfiledPIDController> getClosedLoopController()
  {
    return controller;
  }

  /**
   * Get the simple closed loop controller without motion profiling.
   *
   * @return {@link PIDController} if it exists.
   */
  public Optional<PIDController> getSimpleClosedLoopController()
  {
    return simpleController;
  }

  /**
   * Set the {@link SimpleMotorFeedforward} for {@link SmartMotorController}
   *
   * @param simpleFeedforward {@link SimpleMotorFeedforward}
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withFeedforward(SimpleMotorFeedforward simpleFeedforward)
  {
    if (simpleFeedforward == null)
    {
      this.simpleFeedforward = Optional.empty();
    } else
    {
      this.armFeedforward = Optional.empty();
      this.elevatorFeedforward = Optional.empty();
      this.simpleFeedforward = Optional.of(simpleFeedforward);
    }
    return this;
  }

  /**
   * Get the period of the {@link SmartMotorController} closed loop period.
   *
   * @return {@link SmartMotorController} closed loop controller period.
   */
  public Time getClosedLoopControlPeriod()
  {
    return controlPeriod;
  }

  /**
   * Get the gearing to convert rotor rotations to mechanisms rotations connected to the {@link SmartMotorController}
   *
   * @return {@link MechanismGearing} representing the gearbox and sprockets attached to the
   * {@link SmartMotorController}.
   */
  public MechanismGearing getGearing()
  {
    return gearing;
  }

  /**
   * Get the external encoder.
   *
   * @return Attached external encoder.
   */
  public Optional<Object> getExternalEncoder()
  {
    return externalEncoder;
  }

  /**
   * Get the open loop ramp rate.
   *
   * @return Open loop ramp rate.
   */
  public Time getOpenLoopRampRate()
  {
    return openLoopRampRate;
  }

  /**
   * Get the closed loop ramp rate.
   *
   * @return Closed loop ramp.
   */
  public Time getClosedLoopRampRate()
  {
    return closeLoopRampRate;
  }

  /**
   * Get Telemetry verbosity.
   *
   * @return Verbosity for telemetry.
   */
  public Optional<TelemetryVerbosity> getVerbosity()
  {
    return verbosity;
  }

  /**
   * Telemetry name.
   *
   * @return Telemetry name for NetworkTables.
   */
  public Optional<String> getTelemetryName()
  {
    return telemetryName;
  }

  /**
   * Get the subsystem controlled by the {@link SmartMotorController}
   *
   * @return {@link Subsystem} controlled.
   */
  public Subsystem getSubsystem()
  {
    return subsystem;
  }


  /**
   * Convert {@link LinearVelocity} to {@link AngularVelocity} using the
   * {@link SmartMotorControllerConfig#mechanismCircumference}
   *
   * @param velocity Linear velocity to convert.
   * @return Equivalent angular velocity.
   */
  public AngularVelocity convertToMechanism(LinearVelocity velocity)
  {
    if (mechanismCircumference.isEmpty())
    {
      throw new SmartMotorControllerConfigurationException("Mechanism circumference is undefined",
                                                           "Cannot convert LinearVelocity to AngularVelocity.",
                                                           "withMechanismCircumference(Distance)");

    }

    return RotationsPerSecond.of(velocity.in(MetersPerSecond) / mechanismCircumference.get().in(Meters));
  }

  /**
   * Convert {@link LinearAcceleration} to {@link AngularAcceleration} using the
   * {@link SmartMotorControllerConfig#mechanismCircumference}
   *
   * @param acceleration Linear acceleration to convert.
   * @return Equivalent angular acceleration.
   */
  public AngularAcceleration convertToMechanism(LinearAcceleration acceleration)
  {
    if (mechanismCircumference.isEmpty())
    {
      throw new SmartMotorControllerConfigurationException("Mechanism circumference is undefined",
                                                           "Cannot convert LinearAcceleration to AngularAcceleration.",
                                                           "withMechanismCircumference(Distance)");
    }

    return RotationsPerSecondPerSecond.of(
        acceleration.in(MetersPerSecondPerSecond) / mechanismCircumference.get().in(Meters));
  }

  /**
   * Convert {@link Distance} to {@link Angle} using {@link SmartMotorControllerConfig#mechanismCircumference}
   *
   * @param distance {@link Distance} to convert to {@link Angle}
   * @return {@link Angle} of distance.
   */
  public Angle convertToMechanism(Distance distance)
  {
    if (mechanismCircumference.isEmpty())
    {
      throw new SmartMotorControllerConfigurationException("Mechanism circumference is undefined",
                                                           "Cannot convert Distance to Angle.",
                                                           "withMechanismCircumference(Distance)");

    }
    return Rotations.of(distance.in(Meters) / (mechanismCircumference.get().in(Meters)));
  }

  /**
   * Convert {@link Angle} to {@link Distance} using {@link SmartMotorControllerConfig#mechanismCircumference}
   *
   * @param rotations Rotations to convert.
   * @return Distance of the mechanism.
   */
  public Distance convertFromMechanism(Angle rotations)
  {
    if (mechanismCircumference.isEmpty())
    {
      throw new SmartMotorControllerConfigurationException("Mechanism circumference is undefined",
                                                           "Cannot convert Angle to Distance.",
                                                           "withMechanismCircumference(Distance)");
    }
    return Meters.of(rotations.in(Rotations) * mechanismCircumference.get().in(Meters));
  }

  /**
   * Convert {@link Angle} to {@link LinearVelocity} using {@link SmartMotorControllerConfig#mechanismCircumference}
   *
   * @param velocity Velocity to convert.
   * @return Velocity of the mechanism.
   */
  public LinearVelocity convertFromMechanism(AngularVelocity velocity)
  {
    if (mechanismCircumference.isEmpty())
    {
      throw new SmartMotorControllerConfigurationException("Mechanism circumference is undefined",
                                                           "Cannot convert AngularVelocity to LinearVelocity.",
                                                           "withMechanismCircumference(Distance)");
    }
    return MetersPerSecond.of(velocity.in(RotationsPerSecond) * mechanismCircumference.get().in(Meters));
  }

  /**
   * Convert {@link Angle} to {@link LinearAcceleration} using
   * {@link SmartMotorControllerConfig#mechanismCircumference}
   *
   * @param acceleration Rotations to convert.
   * @return Acceleration of the mechanism.
   */
  public LinearAcceleration convertFromMechanism(AngularAcceleration acceleration)
  {
    if (mechanismCircumference.isEmpty())
    {
      throw new SmartMotorControllerConfigurationException("Mechanism circumference is undefined",
                                                           "Cannot convert AngularAcceleration to LinearAcceleration.",
                                                           "withMechanismCircumference(Distance)");
    }
    return MetersPerSecondPerSecond.of(
        acceleration.in(RotationsPerSecondPerSecond) * mechanismCircumference.get().in(Meters));
  }

  /**
   * Get the zero offset for the {@link SmartMotorController}
   *
   * @return {@link Angle} offset.
   */
  public Optional<Angle> getZeroOffset()
  {
    return zeroOffset;
  }

  /**
   * Get the temperature cut off for the {@link SmartMotorController}
   *
   * @return Maximum {@link Temperature}
   */
  public Optional<Temperature> getTemperatureCutoff()
  {
    return temperatureCutoff;
  }

  /**
   * Get the encoder inversion state
   *
   * @return encoder inversion state
   */
  public boolean getEncoderInverted()
  {
    return encoderInverted;
  }

  /**
   * Get the motor inversion state
   *
   * @return moto inversion state.
   */
  public boolean getMotorInverted()
  {
    return motorInverted;
  }

  /**
   * Use the external feedback sensor.
   *
   * @return Use the attached absolute encoder.
   */
  public boolean getUseExternalFeedback()
  {
    return useExternalEncoder;
  }

  /**
   * Get the starting mechanism position of the {@link SmartMotorController}
   *
   * @return Starting Mechanism position.
   */
  public Optional<Angle> getStartingPosition()
  {
    return startingPosition;
  }

  /**
   * Get the closed loop maximum voltage.
   *
   * @return Maximum voltage in Volts.
   */
  public Optional<Voltage> getClosedLoopControllerMaximumVoltage()
  {
    return closedLoopControllerMaximumVoltage;
  }

  public Optional<Angle> getFeedbackSynchronizationThreshold()
  {
    return feedbackSynchronizationThreshold;
  }

  /**
   * Get the motor controller mdoe to use.
   *
   * @return {@link ControlMode} to use.
   */
  public ControlMode getMotorControllerMode()
  {
    return motorControllerMode;
  }

  /**
   * Get the external encoder gearing, default is 1:1 on a MAXPlanetary.
   *
   * @return External encoder gearing.
   */
  public MechanismGearing getExternalEncoderGearing()
  {
    return externalEncoderGearing;
  }

  /**
   * Set the external encoder gearing. Default is a 1:1 with MAX Planetary.
   *
   * @param externalEncoderGearing External encoder gearing.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public SmartMotorControllerConfig withExternalGearing(MechanismGearing externalEncoderGearing)
  {
    this.externalEncoderGearing = externalEncoderGearing;
    return this;
  }

  /**
   * Get the discontinuity point for the {@link SmartMotorController} encoder.
   *
   * @return {@link Angle} where the encoder wraps around.
   */
  public Optional<Angle> getDiscontinuityPoint()
  {
    if (maxDiscontinuityPoint.isPresent() && minDiscontinuityPoint.isPresent() && !minDiscontinuityPoint.get().equals(
        Rotations.of(maxDiscontinuityPoint.get().in(Rotations) - 1)))
    {
      throw new SmartMotorControllerConfigurationException("Bounds are not correct!",
                                                           "Cannot get the discontinuity point.",
                                                           "withContinuousWrapping(Rotations.of(" +
                                                           Rotations.of(maxDiscontinuityPoint.get().in(Rotations) - 1)
                                                                    .in(Rotations) + "),Rotations.of(" +
                                                           maxDiscontinuityPoint.get().in(Rotations) + ")) instead ");
    }
    return maxDiscontinuityPoint;

  }

  /**
   * Get whether or not the external encoder is inverted.
   *
   * @return External encoder inversion state
   */
  public boolean getExternalEncoderInverted()
  {
    return externalEncoderInverted;
  }


  /**
   * Telemetry verbosity for the {@link SmartMotorController}
   */
  public enum TelemetryVerbosity
  {
    LOW, MID, HIGH
  }

  /**
   * Idle mode for the {@link SmartMotorController}
   */
  public enum MotorMode
  {
    /**
     * Brake mode.
     */
    BRAKE,
    /**
     * Coast mode.
     */
    COAST
  }

  /**
   * Control mode for a motor controller.
   */
  public enum ControlMode
  {
    /**
     * Open loop control mode. Does not use the PID controller.
     */
    OPEN_LOOP,
    /**
     * Use the PID controller.
     */
    CLOSED_LOOP,
  }
}
