// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Newtons;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Volts;

import java.util.Arrays;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.Set;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.controller.ElevatorFeedforward;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.trajectory.ExponentialProfile;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.units.AngularAccelerationUnit;
import org.wpilib.units.LinearAccelerationUnit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Force;
import org.wpilib.units.measure.Frequency;
import org.wpilib.units.measure.LinearAcceleration;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Mass;
import org.wpilib.units.measure.MomentOfInertia;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Time;
import org.wpilib.units.measure.Velocity;
import org.wpilib.units.measure.Voltage;
import org.wpilib.util.Pair;
import yams.core.exceptions.SmartMotorControllerConfigurationException;
import yams.core.gearing.MechanismGearing;
import yams.core.math.LQRController;
import yams.core.mechanisms.positional.Elevator;
import yams.core.motorcontrollers.SmartMotorController.ClosedLoopControllerSlot;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.telemetry.SmartMotorControllerTelemetryConfig;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Smart motor controller config.
 *
 * <h2>Example</h2>
 *
 * <pre>{@code
 * SmartMotorControllerConfig config = new SmartMotorControllerConfig()
 *     .withMotorInverted(false)
 *     .withStatorCurrentLimit(Amps.of(40))
 *     .withSupplyCurrentLimit(Amps.of(60))
 *     .withFeedforward(new SimpleMotorFeedforward(0.001, 0.002))
 *     .withMechanismUpperLimit(Degrees.of(90))
 *     .withMechanismLowerLimit(Degrees.of(-90))
 *     .withGearing(new GearBox(12.0, 60.0));
 * }</pre>
 */
public abstract class SmartMotorControllerConfig<T extends SmartMotorControllerConfig<T>> {
  /**
   * Reset old configurations, so they are no longer persistent.
   */
  private boolean resetPreviousConfig = true;
  /**
   * Vendor specific configuration for the {@link SmartMotorController}.
   */
  private Optional<Object> vendorConfig = Optional.empty();
  /**
   * Vendor specific control request for the {@link SmartMotorController}
   */
  private Optional<Object> vendorControlRequest = Optional.empty();
  /**
   * {@link SmartMotorController} this config has been attached to, if any.
   */
  private SmartMotorController attachedController;
  /**
   * Missing options that would be decremented for each motor application.
   */
  private final List<SmartMotorControllerOptions> missingOptions = Arrays.asList(SmartMotorControllerOptions.values());
  /**
   * Validation set to confirm all options have been applied to the Smart Motor Controller.
   */
  private Set<BasicOptions> basicOptions = EnumSet.allOf(BasicOptions.class);
  /**
   * Validation set to confirm all options have been applied to the Smart Motor Controller's
   * external encoder.
   */
  private Set<ExternalEncoderOptions> externalEncoderOptions = EnumSet.allOf(ExternalEncoderOptions.class);
  /**
   * External encoder.
   */
  private Optional<Object> externalEncoder = Optional.empty();
  /**
   * External encoder inversion state.
   */
  private Optional<Boolean> externalEncoderInverted = Optional.empty();
  /**
   * Follower motors and inversion.
   */
  private Optional<Pair<Object, Boolean>[]> followers = Optional.empty();
  /**
   * Simple feedforward for the motor controller.
   */
  private EnumMap<ClosedLoopControllerSlot, SimpleMotorFeedforward> simpleFeedforward = new EnumMap<>(ClosedLoopControllerSlot.class);
  /**
   * Elevator feedforward for the motor controller.
   */
  private EnumMap<ClosedLoopControllerSlot, ElevatorFeedforward> elevatorFeedforward = new EnumMap<>(ClosedLoopControllerSlot.class);
  /**
   * Arm feedforward for the motor controller.
   */
  private EnumMap<ClosedLoopControllerSlot, ArmFeedforward> armFeedforward = new EnumMap<>(ClosedLoopControllerSlot.class);
  /**
   * Simple feedforward for the motor controller.
   */
  private EnumMap<ClosedLoopControllerSlot, SimpleMotorFeedforward> sim_simpleFeedforward = new EnumMap<>(ClosedLoopControllerSlot.class);
  /**
   * Elevator feedforward for the motor controller.
   */
  private EnumMap<ClosedLoopControllerSlot, ElevatorFeedforward> sim_elevatorFeedforward = new EnumMap<>(ClosedLoopControllerSlot.class);
  /**
   * Arm feedforward for the motor controller.
   */
  private EnumMap<ClosedLoopControllerSlot, ArmFeedforward> sim_armFeedforward = new EnumMap<>(ClosedLoopControllerSlot.class);
  /**
   * Exponential Profile
   */
  private Optional<ExponentialProfile.Constraints> exponentialProfile = Optional.empty();
  /**
   * Trapezoidal Profile
   */
  private Optional<TrapezoidProfile.Constraints> trapezoidProfile = Optional.empty();
  /**
   * Exponential Profile
   */
  private Optional<ExponentialProfile.Constraints> sim_exponentialProfile = Optional.empty();
  /**
   * Trapezoidal Profile
   */
  private Optional<TrapezoidProfile.Constraints> sim_trapezoidProfile = Optional.empty();
  /**
   * Controller for the {@link SmartMotorController}.
   */
  private EnumMap<ClosedLoopControllerSlot, PIDController> pid = new EnumMap<>(ClosedLoopControllerSlot.class);
  /**
   * Controller for the {@link SmartMotorController}.
   */
  private Optional<LQRController> lqr = Optional.empty();
  /**
   * Controller for the {@link SmartMotorController}.
   */
  private Optional<LQRController> sim_lqr = Optional.empty();
  /**
   * Controller for the {@link SmartMotorController}.
   */
  private EnumMap<ClosedLoopControllerSlot, PIDController> sim_pid = new EnumMap<>(ClosedLoopControllerSlot.class);
  /**
   * Gearing for the {@link SmartMotorController}.
   */
  private MechanismGearing gearing;
  /**
   * External encoder gearing,
   defaults to 1:1. */
  private Optional<MechanismGearing> externalEncoderGearing = Optional.empty();
  /**
   * Mechanism Circumference for distance calculations.
   */
  private Optional<Distance> mechanismCircumference = Optional.empty();
  /**
   * PID Controller period for robot controller based PIDs
   */
  private Optional<Time> controlPeriod = Optional.empty();
  /**
   * Simulation loop period, the rate at which {@link SmartMotorController#simIterate()} steps the
   * simulated physics forward. This is independent of {@link #controlPeriod} a robot can run its
   * periodic loop at 20ms while stepping simulation physics at 5ms, for example. Defaults to 20ms
   * if not configured.
   */
  private Optional<Time> simulationPeriod = Optional.empty();
  /**
   * Open loop ramp rate, amount of time to go from 0 to 100 speed.
   */
  private Optional<Time> openLoopRampRate = Optional.empty();
  /**
   * Closed loop ramp rate, amount of time to go from 0 to
   100 speed. */
  private Optional<Time> closeLoopRampRate = Optional.empty();
  /**
   * Set the stator current limit in Amps for the {@link
   SmartMotorController} */
  private OptionalInt statorStallCurrentLimit = OptionalInt.empty();
  /**
   * The supply current limit in Amps for the {@link
   SmartMotorController} */
  private OptionalInt supplyStallCurrentLimit = OptionalInt.empty();
  /**
   * The voltage compensation.
   */
  private Optional<Voltage> voltageCompensation = Optional.empty();
  /**
   * Set the {@link MotorMode} for the {@link
   SmartMotorController}. */
  private Optional<MotorMode> idleMode = Optional.empty();
  /**
   * Mechanism lower limit to prevent movement below.
   */
  private Optional<Angle> mechanismLowerLimit = Optional.empty();
  /**
   * High distance soft limit to prevent movement above.
   */
  private Optional<Angle> mechanismUpperLimit = Optional.empty();
  /**
   * Name for the {@link SmartMotorController} telemetry.
   */
  private Optional<String> telemetryName = Optional.empty();
  /**
   * Telemetry verbosity setting.
   */
  private Optional<TelemetryVerbosity> verbosity = Optional.empty();
  /**
   * Optional config for custom telemetry setup.
   */
  private Optional<SmartMotorControllerTelemetryConfig> specifiedTelemetryConfig = Optional.empty();
  /**
   * Zero offset of the {@link SmartMotorController}
   */
  private Optional<Angle> zeroOffset = Optional.empty();
  /**
   * External absolute encoder discontinuity point.
   */
  private Optional<Angle> externalEncoderDiscontinuityPoint = Optional.empty();
  /**
   * Temperature cutoff for the {@link SmartMotorController} to prevent
   running if above. */
  private Optional<Temperature> temperatureCutoff = Optional.empty();
  /**
   * The encoder readings are inverted.
   */
  private Optional<Boolean> encoderInverted = Optional.empty();
  /**
   * The motor is inverted.
   */
  private Optional<Boolean> motorInverted = Optional.empty();
  /**
   * Use the provided external encoder if set.
   */
  private boolean useExternalEncoder = true;
  /**
   * {@link SmartMotorController}
   starting angle. */
  private Optional<Angle> startingPosition = Optional.empty();
  /**
   * {@link SmartMotorController} starting angle to be used
   during simulation. */
  private Optional<Angle> sim_startingPosition = Optional.empty();
  /**
   * Maximum voltage output for the motor controller while
   using the closed loop controller. */
  private Optional<Voltage> closedLoopControllerMaximumVoltage = Optional.empty();
  /**
   * Feedback synchronization threshold.
   */
  private Optional<Angle> feedbackSynchronizationThreshold = Optional.empty();
  /**
   * The motor controller mode.
   */
  private ControlMode motorControllerMode = ControlMode.CLOSED_LOOP;
  /**
   * Closed loop controller continuous wrapping point.
   */
  private Optional<Angle> maxContinuousWrappingAngle = Optional.empty();
  /**
   * Closed loop controller continuous wrapping point.
   */
  private Optional<Angle> minContinuousWrappingAngle = Optional.empty();
  /**
   * Closed loop controller tolerance.
   */
  private Optional<Angle> closedLoopTolerance = Optional.empty();
  /**
   * Moment of inertia for DCSim
   */
  private MomentOfInertia moi = KilogramSquareMeters.of(0.02);
  /**
   * Loosely coupled followers.
   */
  private Optional<SmartMotorController[]> looselyCoupledFollowers = Optional.empty();
  /**
   * Linear or {@link Distance} based closed loop controller.
   */
  private boolean linearClosedLoopController = false;
  /**
   * Velocity trapezoidal profile.
   */
  private boolean velocityTrapezoidalProfile = false;

  /**
   * Construct the {@link SmartMotorControllerConfig}.
   *
   * @implNote Protected so only {@link yams.commands2.config.SmartMotorControllerConfig} can
   *           construct this.
   */
  protected SmartMotorControllerConfig() {
    HAL.reportUsage(null, null);
  }

  /**
   * Duplicate the SmartMotorControllerConfig.
   *
   * @param cfg Config to duplicate.
   */
  protected SmartMotorControllerConfig(SmartMotorControllerConfig<?> cfg) {
    this.resetPreviousConfig = cfg.resetPreviousConfig;
    this.vendorConfig = cfg.vendorConfig;
    this.vendorControlRequest = cfg.vendorControlRequest;
    this.basicOptions = EnumSet.copyOf(cfg.basicOptions);
    this.externalEncoderOptions = EnumSet.copyOf(cfg.externalEncoderOptions);
    this.externalEncoder = cfg.externalEncoder;
    this.externalEncoderInverted = cfg.externalEncoderInverted;
    this.followers = cfg.followers;
    this.simpleFeedforward = cfg.simpleFeedforward;
    this.elevatorFeedforward = cfg.elevatorFeedforward;
    this.armFeedforward = cfg.armFeedforward;
    this.sim_simpleFeedforward = cfg.sim_simpleFeedforward;
    this.sim_elevatorFeedforward = cfg.sim_elevatorFeedforward;
    this.sim_armFeedforward = cfg.sim_armFeedforward;
    this.sim_lqr = cfg.sim_lqr;
    this.lqr = cfg.lqr;
    this.pid = cfg.pid;
    this.sim_pid = cfg.sim_pid;
    this.exponentialProfile = cfg.exponentialProfile;
    this.trapezoidProfile = cfg.trapezoidProfile;
    this.sim_exponentialProfile = cfg.sim_exponentialProfile;
    this.sim_trapezoidProfile = cfg.sim_trapezoidProfile;
    this.gearing = cfg.gearing;
    this.externalEncoderGearing = cfg.externalEncoderGearing;
    this.mechanismCircumference = cfg.mechanismCircumference;
    this.controlPeriod = cfg.controlPeriod;
    this.simulationPeriod = cfg.simulationPeriod;
    this.openLoopRampRate = cfg.openLoopRampRate;
    this.closeLoopRampRate = cfg.closeLoopRampRate;
    this.statorStallCurrentLimit = cfg.statorStallCurrentLimit;
    this.supplyStallCurrentLimit = cfg.supplyStallCurrentLimit;
    this.voltageCompensation = cfg.voltageCompensation;
    this.idleMode = cfg.idleMode;
    this.mechanismLowerLimit = cfg.mechanismLowerLimit;
    this.mechanismUpperLimit = cfg.mechanismUpperLimit;
    this.telemetryName = cfg.telemetryName;
    this.verbosity = cfg.verbosity;
    this.specifiedTelemetryConfig = cfg.specifiedTelemetryConfig;
    this.zeroOffset = cfg.zeroOffset;
    this.externalEncoderDiscontinuityPoint = cfg.externalEncoderDiscontinuityPoint;
    this.temperatureCutoff = cfg.temperatureCutoff;
    this.encoderInverted = cfg.encoderInverted;
    this.motorInverted = cfg.motorInverted;
    this.useExternalEncoder = cfg.useExternalEncoder;
    this.startingPosition = cfg.startingPosition;
    this.sim_startingPosition = cfg.sim_startingPosition;
    this.closedLoopControllerMaximumVoltage = cfg.closedLoopControllerMaximumVoltage;
    this.feedbackSynchronizationThreshold = cfg.feedbackSynchronizationThreshold;
    this.motorControllerMode = cfg.motorControllerMode;
    this.maxContinuousWrappingAngle = cfg.maxContinuousWrappingAngle;
    this.minContinuousWrappingAngle = cfg.minContinuousWrappingAngle;
    this.closedLoopTolerance = cfg.closedLoopTolerance;
    this.moi = cfg.moi;
    this.looselyCoupledFollowers = cfg.looselyCoupledFollowers;
    this.linearClosedLoopController = cfg.linearClosedLoopController;
    this.velocityTrapezoidalProfile = cfg.velocityTrapezoidalProfile;
  }

  /**
   * Get this config as its concrete type, so chained {@code with} calls keep the subclass type.
   *
   * @return this {@link SmartMotorControllerConfig}.
   */
  protected abstract T self();

  @Override
  public abstract T clone();

  /**
   * Set the vendor specific config for the {@link SmartMotorController} which will be used as a
   * base. Vendor configs will be overridden by the {@link SmartMotorControllerConfig} options.
   *
   * @param vendorConfig Vendor specific config object. Must be of the correct type for the {@link
   *                     SmartMotorController}. Only the root configuration class is accepted.
   * @return {@link SmartMotorControllerConfig} for chaining.
   * @implSpec {@link SmartMotorControllerConfig} options will always take precedence and overwrite
   *           the vendor config. Apply any changes after the {@link SmartMotorController} is
   *           created to ensure accuracy.
   */
  public T withVendorConfig(Object vendorConfig) {
    this.vendorConfig = Optional.ofNullable(vendorConfig);
    return self();
  }

  /**
   * Attach the {@link SmartMotorController} this config was applied to, so command-layer helpers
   * (e.g. live tuning) can reach back into it without core depending on any command framework
   * types.
   *
   * @param controller {@link SmartMotorController} this config configured.
   */
  protected void attachController(SmartMotorController controller) {
    this.attachedController = controller;
  }

  /**
   * Get the {@link SmartMotorController} this config was attached to via {@link
   * #attachController(SmartMotorController)}, if any.
   *
   * @return {@link Optional} of the attached {@link SmartMotorController}.
   */
  protected Optional<SmartMotorController> getAttachedController() {
    return Optional.ofNullable(attachedController);
  }

  /**
   * Set the external encoder inversion state.
   *
   * @param externalEncoderInverted External encoder inversion state.
   * @return {@link SmartMotorControllerConfig} for chaining.
   * @implNote This may mean the zero offset you gather should be negated.
   */
  public T withExternalEncoderInverted(boolean externalEncoderInverted) {
    this.externalEncoderInverted = Optional.of(externalEncoderInverted);
    return self();
  }

  /**
   * Set the control mode for the {@link SmartMotorController}
   *
   * @param controlMode {@link ControlMode} to apply.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withControlMode(ControlMode controlMode) {
    this.motorControllerMode = controlMode;
    return self();
  }

  /**
   * Set the feedback synchronization threshold so the relative encoder synchronizes with the
   * absolute encoder at this point.
   *
   * @param angle {@link Angle} to exceed.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withFeedbackSynchronizationThreshold(Angle angle) {
    if (mechanismCircumference.isPresent()) {
      throw new SmartMotorControllerConfigurationException("Auto-synchronization is unavailable when using distance based mechanisms", "Cannot set synchronization threshold.", "withMechanismCircumference(Distance) should be removed.");
    }
    feedbackSynchronizationThreshold = Optional.ofNullable(angle);
    return self();
  }

  /**
   * Set the closed loop maximum voltage output.
   *
   * @param volts Maximum voltage.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withClosedLoopControllerMaximumVoltage(Voltage volts) {
    closedLoopControllerMaximumVoltage = Optional.ofNullable(volts);
    return self();
  }

  /**
   * Set the starting angle of the {@link SmartMotorController}
   *
   * @param startingAngle Starting Mechanism Angle.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withStartingPosition(Angle startingAngle) {
    this.startingPosition = Optional.ofNullable(startingAngle);
    return self();
  }

  /**
   * Set the starting angle of the {@link SmartMotorController}
   *
   * @param startingAngle Starting Mechanism Distance.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withStartingPosition(Distance startingAngle) {
    return withStartingPosition(convertToMechanism(startingAngle));
  }

  /**
   * Set the starting angle of the {@link SmartMotorController} to be used only in simulation
   *
   * @param simStartingAngle Starting Mechanism Angle.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withSimStartingPosition(Angle simStartingAngle) {
    this.sim_startingPosition = Optional.ofNullable(simStartingAngle);
    return self();
  }

  /**
   * Set the starting angle of the {@link SmartMotorController} to be used only in simulation
   *
   * @param simStartingAngle Starting Mechanism Distance.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withSimStartingPosition(Distance simStartingAngle) {
    return withSimStartingPosition(convertToMechanism(simStartingAngle));
  }

  /**
   * Set the external encoder to be the primary feedback device for the PID controller.
   *
   * @param useExternalEncoder External encoder as primary feedback device for the PID controller.
   * @return {@link SmartMotorControllerConfig} for chaining
   */
  public T withUseExternalFeedbackEncoder(boolean useExternalEncoder) {
    this.useExternalEncoder = useExternalEncoder;
    return self();
  }

  /**
   * Set the encoder inversion state.
   *
   * @param inverted Encoder inversion state.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withEncoderInverted(boolean inverted) {
    this.encoderInverted = Optional.of(inverted);
    return self();
  }

  /**
   * Set the motor inversion state.
   *
   * @param motorInverted Motor inversion state.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withMotorInverted(boolean motorInverted) {
    this.motorInverted = Optional.of(motorInverted);
    return self();
  }

  /**
   * Set the {@link SmartMotorController} to reset the old configurations and only apply what is
   * given to {@link SmartMotorControllerConfig}
   *
   * @param resetPreviousConfig Reset the old config?
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withResetPreviousConfig(boolean resetPreviousConfig) {
    this.resetPreviousConfig = resetPreviousConfig;
    return self();
  }

  /**
   * Set the {@link Temperature} cut off for the {@link SmartMotorController}/
   *
   * @param cutoff maximum {@link Temperature}
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withTemperatureCutoff(Temperature cutoff) {
    temperatureCutoff = Optional.ofNullable(cutoff);
    return self();
  }

  /**
   * Set the zero offset of the {@link SmartMotorController}'s external Encoder.
   *
   * @param distance Zero offset in distance.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withExternalEncoderZeroOffset(Distance distance) {
    if (mechanismCircumference.isEmpty()) {
      throw new SmartMotorControllerConfigurationException("Mechanism circumference is undefined", "Cannot set zero offset.", "withMechanismCircumference(Distance)");
    }
    return withExternalEncoderZeroOffset(convertToMechanism(distance));
  }

  /**
   * Set the external encoder absolute position to wrap around a defined discontinuity point.
   *
   * @param discontinuityPoint Discontinuity point of the external encoder, when provided 0.5rot the
   *                           external encoder will read between [-0.5, 0.5]; when provided 1rot
   *                           the external encoder will read between [0, 1].
   * @return {@link SmartMotorControllerConfig} for chaining.
   * @implNote Only works for External Absolute Encoders.
   */
  public T withExternalEncoderDiscontinuityPoint(Angle discontinuityPoint) {
    if (!discontinuityPoint.isEquivalent(Rotations.of(1)) && !discontinuityPoint.isEquivalent(Rotations.of(0.5))) {
      throw new SmartMotorControllerConfigurationException("Cannot set external encoder discontinuity point", "Discontinuity point must be 0.5 or 1 rotations", "withExternalEncoderDiscontinuityPoint(Rotations.of(0.5)");
    }
    externalEncoderDiscontinuityPoint = Optional.of(discontinuityPoint);
    return self();
  }

  /**
   * Set the zero offset of the {@link SmartMotorController}'s external Encoder.
   *
   * @param angle {@link Angle} to 0.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withExternalEncoderZeroOffset(Angle angle) {
    if (angle != null && angle.lt(Rotations.of(0))) {
      // Zero offsets cannot be negative.
      angle = angle.plus(Rotations.of(1));
    }
    zeroOffset = Optional.ofNullable(angle);
    return self();
  }

  /**
   * Set continuous wrapping for the {@link SmartMotorController}
   *
   * @param bottom Bottom value to wrap to.
   * @param top    Top value to wrap to.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withContinuousWrapping(Angle bottom, Angle top) {
    if (mechanismUpperLimit.isPresent() || mechanismLowerLimit.isPresent()) {
      throw new SmartMotorControllerConfigurationException("Soft limits set while configuring continuous wrapping", "Cannot set continuous wrapping", "withSoftLimits(Angle,Angle) should be removed");
    }
    if (linearClosedLoopController) {
      throw new SmartMotorControllerConfigurationException("Distance based mechanism used with continuous wrapping", "Cannot set continuous wrapping", "withMechanismCircumference(Distance) should be removed");
    }
    for (var pidController : pid.values()) {
      pidController.enableContinuousInput(bottom.in(Rotations), top.in(Rotations));
    }
    if (pid.isEmpty()) {
      throw new SmartMotorControllerConfigurationException("No PID controller used", "Cannot set continuous wrapping!", "withClosedLoopController()");
    }

    maxContinuousWrappingAngle = Optional.of(top);
    minContinuousWrappingAngle = Optional.of(bottom);
    return self();
  }

  /**
   * Set the closed loop tolerance of the mechanism controller.
   *
   * @param tolerance Closed loop controller tolerance
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withClosedLoopTolerance(Angle tolerance) {
    closedLoopTolerance = Optional.ofNullable(tolerance);
    if (tolerance != null) {
      for (var pidController : pid.values()) {
        pidController.setTolerance(getClosedLoopTolerance().orElse(tolerance).in(Rotations));
      }
      if (pid.isEmpty()) {
        throw new SmartMotorControllerConfigurationException("No PID controller used", "Cannot set tolerance!", "withClosedLoopController()");
      }
    }
    return self();
  }

  /**
   * Set the {@link SmartMotorController} closed loop controller tolerance via distance.
   *
   * @param tolerance {@link Distance} tolerance.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withClosedLoopTolerance(Distance tolerance) {
    if (!linearClosedLoopController) {
      throw new SmartMotorControllerConfigurationException("Linear closed loop controller used with distance tolerance.", "Closed loop tolerance cannot be set.", "withLinearClosedLoopController(true)");
    }
    if (tolerance != null) {
      Angle toleranceAngle = convertToMechanism(tolerance);
      closedLoopTolerance = Optional.ofNullable(toleranceAngle);
      for (var pidController : pid.values()) {
        pidController.setTolerance(convertFromMechanism(getClosedLoopTolerance().orElse(toleranceAngle)).in(Meters));
      }
      if (pid.isEmpty()) {
        throw new SmartMotorControllerConfigurationException("No PID controller used", "Cannot set tolerance!", "withClosedLoopController()");
      }
    }
    return self();
  }

  /**
   * Get the {@link SmartMotorController} closed loop controller tolerance.
   *
   * @return {@link Angle} tolerance.
   */
  public Optional<Angle> getClosedLoopTolerance() {
    basicOptions.remove(BasicOptions.ClosedLoopTolerance);
    return closedLoopTolerance;
  }

  /**
   * Set the telemetry for the {@link SmartMotorController}
   *
   * @param telemetryName Name for the {@link SmartMotorController}
   * @param verbosity     Verbosity of the Telemetry for the {@link SmartMotorController}
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withTelemetry(String telemetryName, TelemetryVerbosity verbosity) {
    this.telemetryName = Optional.ofNullable(telemetryName);
    this.verbosity = Optional.ofNullable(verbosity);
    return self();
  }

  /**
   * Set the telemetry for the {@link SmartMotorController}
   *
   * @param verbosity Verbosity of the Telemetry for the {@link SmartMotorController}
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withTelemetry(TelemetryVerbosity verbosity) {
    this.telemetryName = Optional.of("motor");
    this.verbosity = Optional.of(verbosity);
    return self();
  }

  /**
   * Set the telemetry for the {@link SmartMotorController} with a {@link
   * SmartMotorControllerTelemetryConfig}
   *
   * @param telemetryName   Name for the {@link SmartMotorController}
   * @param telemetryConfig Config that specifies what to log.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withTelemetry(String telemetryName, SmartMotorControllerTelemetryConfig telemetryConfig) {
    this.telemetryName = Optional.ofNullable(telemetryName);
    this.verbosity = Optional.of(TelemetryVerbosity.HIGH);
    this.specifiedTelemetryConfig = Optional.ofNullable(telemetryConfig);
    return self();
  }

  /**
   * Get the telemetry configuration
   *
   * @return Telemetry configuration.
   */
  public Optional<SmartMotorControllerTelemetryConfig> getSmartControllerTelemetryConfig() {
    return specifiedTelemetryConfig;
  }

  /**
   * Get the stator stall current limit.
   *
   * @return Stator current limit.
   */
  public OptionalInt getStatorStallCurrentLimit() {
    basicOptions.remove(BasicOptions.StatorCurrentLimit);
    return statorStallCurrentLimit;
  }

  /**
   * Set the distance soft limits.
   *
   * @param low  Low distance soft limit.
   * @param high High distance soft limit.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withSoftLimits(Distance low, Distance high) {
    if (mechanismCircumference.isEmpty()) {
      throw new SmartMotorControllerConfigurationException("Mechanism circumference is undefined", "Cannot set soft limits.", "withMechanismCircumference(Distance)");
    }

    return withSoftLimits(Rotations.of(low.in(Meters) / mechanismCircumference.get().in(Meters)), Rotations.of(high.in(Meters) / mechanismCircumference.get().in(Meters)));
  }

  /**
   * Add the mechanism moment of inertia to the {@link SmartMotorController}s simulation when not
   * run under a formal mechanism.
   *
   * @param length Length of the mechanism for MOI.
   * @param weight Weight of the mechanism for MOI.
   * @return {@link SmartMotorControllerConfig} for chaining
   */
  public T withMomentOfInertia(Distance length, Mass weight) {
    if (length == null || weight == null) {
      throw new SmartMotorControllerConfigurationException("Length or Weight cannot be null!", "MOI is necessary for standalone SmartMotorController simulation!", "withMOI(Inches.of(4),Pounds.of(1))");
    } else {
      moi = KilogramSquareMeters.of(SingleJointedArmSim.estimateMOI(length.in(Meters), weight.in(Kilograms)));
    }
    return self();
  }

  /**
   * Add the mechanism moment of inertia to the {@link SmartMotorController}s simulation when not
   * run under a formal mechanism.
   *
   * @param MOI Known moment of inertia.
   * @return {@link SmartMotorControllerConfig} for chaining
   */
  public T withMomentOfInertia(MomentOfInertia MOI) {
    moi = MOI;
    return self();
  }

  /**
   * Set the angle soft limits.
   *
   * @param low  Low angle soft limit.
   * @param high High angle soft limit.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withSoftLimits(Angle low, Angle high) {
    if (low != null && high != null && low.gte(high)) {
      throw new SmartMotorControllerConfigurationException("Lower limit is higher than upper limit", "Cannot configure SmartMotorController", "withSoftLimit(LOW, HIGH)");
    }
    mechanismLowerLimit = Optional.ofNullable(low);
    mechanismUpperLimit = Optional.ofNullable(high);

    return self();
  }

  /**
   * Get the supply stall current limit.
   *
   * @return Supply stall current limit.
   */
  public OptionalInt getSupplyStallCurrentLimit() {
    basicOptions.remove(BasicOptions.SupplyCurrentLimit);
    return supplyStallCurrentLimit;
  }

  /**
   * Get the voltage compensation for the {@link SmartMotorController}
   *
   * @return Ideal voltage
   */
  public Optional<Voltage> getVoltageCompensation() {
    basicOptions.remove(BasicOptions.VoltageCompensation);
    return voltageCompensation;
  }

  /**
   * Get the idle mode for the {@link SmartMotorController}
   *
   * @return {@link MotorMode}
   */
  public Optional<MotorMode> getIdleMode() {
    basicOptions.remove(BasicOptions.IdleMode);
    return idleMode;
  }

  /**
   * Get the Moment of Inertia of the {@link SmartMotorController}'s mechanism for the
   * {@link org.wpilib.simulation.DCMotorSim}.
   *
   * @return Moment of Inertia in JKgMetersSquared.
   */
  public double getMOI() {
    //    basicOptions.remove(BasicOptions.MomentOfInertia);
    return moi.in(KilogramSquareMeters);
  }

  /**
   * Lower limit of the mechanism.
   *
   * @return Lower angle soft limit.
   */
  public Optional<Angle> getMechanismLowerLimit() {
    basicOptions.remove(BasicOptions.LowerLimit);
    return mechanismLowerLimit;
  }

  /**
   * Upper limit of the mechanism.
   *
   * @return Higher angle soft limit.
   */
  public Optional<Angle> getMechanismUpperLimit() {
    basicOptions.remove(BasicOptions.UpperLimit);
    return mechanismUpperLimit;
  }

  /**
   * Set the {@link SmartMotorController} to brake or coast mode.
   *
   * @param idleMode {@link MotorMode} idle mode
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withIdleMode(MotorMode idleMode) {
    this.idleMode = Optional.ofNullable(idleMode);
    return self();
  }

  /**
   * Set the voltage compensation for the {@link SmartMotorController}
   *
   * @param voltageCompensation Ideal voltage value.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withVoltageCompensation(Voltage voltageCompensation) {
    this.voltageCompensation = voltageCompensation == null ? Optional.empty() : Optional.of(voltageCompensation);
    return self();
  }

  /**
   * Set the follower motors of the {@link SmartMotorController}
   *
   * @param followers Base motor types (NOT {@link SmartMotorController}!) to configure as
   *                  followers, must be same brand as the {@link SmartMotorController} with
   *                  inversion from the base motor.
   * @return {@link SmartMotorControllerConfig} for chaining
   */
  @SafeVarargs
  public final T withFollowers(Pair<Object, Boolean>... followers) {
    this.followers = Optional.ofNullable(followers);
    return self();
  }

  /**
   * Applies loosely coupled follower motors to the {@link SmartMotorController}.
   *
   * @param followers {@link SmartMotorController}s to configure as followers.
   * @return {@link SmartMotorControllerConfig} for chaining
   * @implNote ONLY the position and velocity requests will be forwarded.
   * @implSpec Configurations are not transferred!
   */
  public final T withLooselyCoupledFollowers(SmartMotorController... followers) {
    this.looselyCoupledFollowers = Optional.ofNullable(followers);
    return self();
  }

  /** Clear the follower motors so they are not reapplied */
  public void clearFollowers() {
    this.followers = Optional.empty();
  }

  /**
   * Set the stall stator current limit for the {@link SmartMotorController}
   *
   * @param stallCurrent Stall stator current limit for the {@link SmartMotorController}.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withStatorCurrentLimit(Current stallCurrent) {
    this.statorStallCurrentLimit = stallCurrent == null ? OptionalInt.empty() : OptionalInt.of((int) stallCurrent.in(Amps));
    return self();
  }

  /**
   * Set the stall stator current limit for the {@link SmartMotorController}
   *
   * @param stallCurrent Stall stator current limit for the {@link SmartMotorController}.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withOutputCurrentLimit(Current stallCurrent) {
    return withStatorCurrentLimit(stallCurrent);
  }

  /**
   * Set the stall supply current limit for the {@link SmartMotorController}
   *
   * @param supplyCurrent Supply current limit.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withSupplyCurrentLimit(Current supplyCurrent) {
    this.supplyStallCurrentLimit = supplyCurrent == null ? OptionalInt.empty() : OptionalInt.of((int) supplyCurrent.in(Amps));
    return self();
  }

  /**
   * Set the stall supply current limit for the {@link SmartMotorController}
   *
   * @param supplyCurrent Supply current limit.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withInputCurrentLimit(Current supplyCurrent) {
    return withSupplyCurrentLimit(supplyCurrent);
  }

  /**
   * Set the closed loop ramp rate. The ramp rate is the minimum time it should take to go from 0
   * power to full power in the motor controller while using PID.
   *
   * @param rate time to go from 0 to full throttle.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withClosedLoopRampRate(Time rate) {
    this.closeLoopRampRate = Optional.of(rate);
    return self();
  }

  /**
   * Set the open loop ramp rate. The ramp rate is the minimum time it should take to go from 0
   * power to full power in the motor controller while not using PID.
   *
   * @param rate time to go from 0 to full throttle.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withOpenLoopRampRate(Time rate) {
    this.openLoopRampRate = Optional.of(rate);
    return self();
  }

  /**
   * Set the external encoder which is attached to the motor type sent used by {@link
   * SmartMotorController}
   *
   * @param externalEncoder External encoder attached to the {@link SmartMotorController}
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withExternalEncoder(Object externalEncoder) {
    this.externalEncoder = Optional.ofNullable(externalEncoder);
    return self();
  }

  /**
   * Get the follower motors to the {@link SmartMotorControllerConfig}
   *
   * @return Follower motor list.
   */
  public Optional<Pair<Object, Boolean>[]> getFollowers() {
    basicOptions.remove(BasicOptions.Followers);
    return followers;
  }

  /**
   * Set the {@link MechanismGearing} for the {@link SmartMotorController}.
   *
   * @param gear {@link MechanismGearing} representing the gearbox and sprockets to the final axis.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withGearing(MechanismGearing gear) {
    gearing = gear;
    return self();
  }

  /**
   * Set the {@link MechanismGearing} for the {@link SmartMotorController}.
   *
   * @param reductionRatio Reduction ratio, for example, a ratio of "3:1" is 3; a ratio of "1:2" is
   *                       0.5.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withGearing(double reductionRatio) {
    gearing = new MechanismGearing(reductionRatio);
    return self();
  }

  /**
   * Set the mechanism circumference to allow distance calculations on the {@link
   * SmartMotorController}.
   *
   * @param circumference Circumference of the actuating spool or sprocket+chain attached the
   *                      mechanism actuator.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withMechanismCircumference(Distance circumference) {
    mechanismCircumference = Optional.ofNullable(circumference);
    return self();
  }

  /**
   * Set the wheel radius for the mechanism.
   *
   * @param radius Radius of the wheels as {@link Distance}
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withWheelRadius(Distance radius) {
    mechanismCircumference = Optional.ofNullable(radius.times(2).times(Math.PI));
    return self();
  }

  /**
   * Set the {@link Elevator} drum radius
   *
   * @implNote Overload for {@link #withWheelRadius(Distance)}
   * @param radius Drum radius of elevator spool.
   * @return {@link SmartMotorControllerConfig} for chaining
   */
  public T withDrumRadius(Distance radius) {
    return withWheelRadius(radius);
  }

  /**
   * Set the {@link Elevator} drum radius via the chain pitch (.25in or .35in) and teeth count.
   *
   * @param chainPitch Chain pitch.
   * @param teeth      Sprocket teeth count.
   * @return {@link SmartMotorControllerConfig} for chaining.
   * @implNote Overload for {@link #withWheelRadius(Distance)}
   */
  public T withDrumRadius(Distance chainPitch, int teeth) {
    return withWheelRadius(chainPitch.times(teeth));
  }

  /**
   * Change the {@link SmartMotorControllerConfig} gear ratio to be divided by the number of stages
   * given, will reapply it if already done manually.
   *
   * @param stages Stages given
   * @implNote Alias for {@link #withGearing(MechanismGearing)} where {@link MechanismGearing#div}
   *           is applied via the stages.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withCascadingElevatorStages(int stages) {
    return withGearing(gearing.div(stages));
  }

  /**
   * Set the wheel diameter for the mechanism.
   *
   * @param diameter Diameter of the wheels as {@link Distance}
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withWheelDiameter(Distance diameter) {
    mechanismCircumference = Optional.ofNullable(diameter.times(Math.PI));
    return self();
  }

  /**
   * Get the mechanism to distance ratio for the {@link SmartMotorController}
   *
   * @return Rotations/Distance ratio to convert mechanism rotations to distance.
   */
  public Optional<Distance> getMechanismCircumference() {
    return mechanismCircumference;
  }

  /**
   * If the closed loop controller is linear or {@link Distance} based.
   *
   * @return Linear closed loop controller.
   */
  public boolean getLinearClosedLoopControllerUse() {
    return linearClosedLoopController && mechanismCircumference.isPresent();
  }

  /**
   * Modify the period of the PID controller for the motor controller.
   *
   * @param time Period of the motor controller PID.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withClosedLoopControlPeriod(Time time) {
    controlPeriod = Optional.of(time);
    return self();
  }

  /**
   * Modify the period of the PID controller for the motor controller.
   *
   * @param time Period of the motor controller PID.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withClosedLoopControlPeriod(Frequency time) {
    controlPeriod = Optional.of(time.asPeriod());
    return self();
  }

  /**
   * Set the simulation loop period the rate at which {@link SmartMotorController#simIterate()}
   * steps the simulated physics forward. Independent of {@link #withClosedLoopControlPeriod(Time)};
   * a robot can run its periodic loop at 20ms while stepping simulation physics at 5ms, for
   * example. Defaults to 20ms if not set.
   *
   * @param time Simulation loop period.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withSimulationPeriod(Time time) {
    simulationPeriod = Optional.of(time);
    return self();
  }

  /**
   * Get the simulation loop period the rate at which {@link SmartMotorController#simIterate()}
   * steps the simulated physics forward. Defaults to 20ms if not set via {@link
   * #withSimulationPeriod(Time)}.
   *
   * @return Simulation loop period.
   */
  public Time getSimulationPeriod() {
    return simulationPeriod.orElse(Milliseconds.of(20));
  }

  /**
   * Get the {@link ArmFeedforward} if it is set.
   *
   * @param slot {@link ClosedLoopControllerSlot} for the {@link ArmFeedforward}.
   * @return {@link Optional} of the {@link ArmFeedforward}.
   */
  public Optional<ArmFeedforward> getArmFeedforward(ClosedLoopControllerSlot slot) {
    basicOptions.remove(BasicOptions.ArmFeedforward);
    if (RobotBase.isSimulation() && sim_armFeedforward.containsKey(slot)) {
      return Optional.ofNullable(sim_armFeedforward.get(slot));
    }
    return Optional.ofNullable(armFeedforward.getOrDefault(slot, null));
  }

  /**
   * Configure the {@link ArmFeedforward} for the {@link SmartMotorController}.
   *
   * @param slot           {@link ClosedLoopControllerSlot} for the {@link ArmFeedforward}.
   * @param armFeedforward Arm feedforward for the {@link SmartMotorController}
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withSimFeedforward(ArmFeedforward armFeedforward, ClosedLoopControllerSlot slot) {
    if (armFeedforward == null) {
      this.sim_armFeedforward.remove(slot);
    } else {
      this.sim_elevatorFeedforward.remove(slot);
      this.sim_simpleFeedforward.remove(slot);
      this.sim_armFeedforward.put(slot, armFeedforward);
    }
    return self();
  }

  /**
   * Configure the {@link ArmFeedforward} for the
   *
   * @param armFeedforward Arm feedforward for the {@link SmartMotorController}
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withSimFeedforward(ArmFeedforward armFeedforward) {
    return withSimFeedforward(armFeedforward, ClosedLoopControllerSlot.SLOT_0);
  }

  /**
   * Configure the {@link ArmFeedforward} for the
   *
   * @param slot           {@link ClosedLoopControllerSlot} for the {@link ArmFeedforward}.
   * @param armFeedforward Arm feedforward for the {@link SmartMotorController}
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withFeedforward(ArmFeedforward armFeedforward, ClosedLoopControllerSlot slot) {
    if (armFeedforward == null) {
      this.armFeedforward.remove(slot);
    } else {
      this.elevatorFeedforward.remove(slot);
      this.simpleFeedforward.remove(slot);
      this.armFeedforward.put(slot, armFeedforward);
    }
    return self();
  }

  /**
   * Configure the {@link ArmFeedforward} for the
   *
   * @param armFeedforward Arm feedforward for the {@link SmartMotorController}
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withFeedforward(ArmFeedforward armFeedforward) {
    return withFeedforward(armFeedforward, ClosedLoopControllerSlot.SLOT_0);
  }

  /**
   * Get the {@link ElevatorFeedforward} {@link Optional}
   *
   * @param slot {@link ClosedLoopControllerSlot} for the {@link ElevatorFeedforward}.
   * @return {@link ElevatorFeedforward} {@link Optional}
   */
  public Optional<ElevatorFeedforward> getElevatorFeedforward(ClosedLoopControllerSlot slot) {
    basicOptions.remove(BasicOptions.ElevatorFeedforward);
    if (RobotBase.isSimulation() && sim_elevatorFeedforward.containsKey(slot)) {
      return Optional.ofNullable(sim_elevatorFeedforward.get(slot));
    }
    return Optional.ofNullable(elevatorFeedforward.getOrDefault(slot, null));
  }

  /**
   * Configure {@link ElevatorFeedforward} for the {@link SmartMotorController}
   *
   * @param slot                {@link ClosedLoopControllerSlot} for the {@link
   *                            ElevatorFeedforward}.
   * @param elevatorFeedforward {@link ElevatorFeedforward} to set.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withSimFeedforward(ElevatorFeedforward elevatorFeedforward, ClosedLoopControllerSlot slot) {
    if (elevatorFeedforward == null) {
      this.sim_elevatorFeedforward.remove(slot);
    } else {
      linearClosedLoopController = true;
      this.sim_armFeedforward.remove(slot);
      this.sim_simpleFeedforward.remove(slot);
      this.sim_elevatorFeedforward.put(slot, elevatorFeedforward);
    }
    return self();
  }

  /**
   * Configure {@link ElevatorFeedforward} for the {@link SmartMotorController}
   *
   * @param elevatorFeedforward {@link ElevatorFeedforward} to set.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withSimFeedforward(ElevatorFeedforward elevatorFeedforward) {
    return withSimFeedforward(elevatorFeedforward, ClosedLoopControllerSlot.SLOT_0);
  }

  /**
   * Configure {@link ElevatorFeedforward} for the {@link SmartMotorController}
   *
   * @param slot                {@link ClosedLoopControllerSlot} for the {@link
   *                            ElevatorFeedforward}.
   * @param elevatorFeedforward {@link ElevatorFeedforward} to set.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withFeedforward(ElevatorFeedforward elevatorFeedforward, ClosedLoopControllerSlot slot) {
    if (elevatorFeedforward == null) {
      this.elevatorFeedforward.remove(slot);
    } else {
      linearClosedLoopController = true;
      this.armFeedforward.remove(slot);
      this.simpleFeedforward.remove(slot);
      this.elevatorFeedforward.put(slot, elevatorFeedforward);
    }
    return self();
  }

  /**
   * Configure {@link ElevatorFeedforward} for the {@link SmartMotorController}
   *
   * @param elevatorFeedforward {@link ElevatorFeedforward} to set.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withFeedforward(ElevatorFeedforward elevatorFeedforward) {
    return withFeedforward(elevatorFeedforward, ClosedLoopControllerSlot.SLOT_0);
  }

  /**
   * Get the {@link SimpleMotorFeedforward} {@link Optional}.
   *
   * @param slot {@link ClosedLoopControllerSlot} for the {@link SimpleMotorFeedforward}.
   * @return {@link SimpleMotorFeedforward} {@link Optional}
   */
  public Optional<SimpleMotorFeedforward> getSimpleFeedforward(ClosedLoopControllerSlot slot) {
    basicOptions.remove(BasicOptions.SimpleFeedforward);
    if (RobotBase.isSimulation() && sim_simpleFeedforward.containsKey(slot)) {
      return Optional.of(sim_simpleFeedforward.get(slot));
    }
    return Optional.ofNullable(simpleFeedforward.getOrDefault(slot, null));
  }

  /**
   * Set the closed loop controller for the {@link SmartMotorController}. The units passed in are in
   * Rotations (or Meters if Mechanism Circumference is configured), and outputs are in Volts.
   *
   * @param controller {@link LQRController} to use, the units passed in are in Rotations (or Meters
   *                   if Mechanism Circumference is configured), and output is Voltage.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withSimClosedLoopController(LQRController controller) {
    this.sim_pid.clear();
    this.sim_lqr = Optional.ofNullable(controller);
    return self();
  }

  /**
   * Set the closed loop controller for the {@link SmartMotorController}, the units passed in are in
   * Rotations (or Meters if Mechanism Circumference is configured).
   *
   * @param slot Closed loop controller slot.
   * @param kP   KP scalar for the PID Controller.
   * @param kI   KI scalar for the PID Controller.
   * @param kD   KD scalar for the PID Controller.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withSimClosedLoopController(double kP, double kI, double kD, ClosedLoopControllerSlot slot) {
    this.sim_pid.put(slot, new PIDController(kP, kI, kD));
    this.sim_lqr = Optional.empty();
    return self();
  }

  /**
   * Set the closed loop controller for the {@link SmartMotorController}, the units passed in are in
   * Rotations (or Meters if Mechanism Circumference is configured).
   *
   * @param kP KP scalar for the PID Controller.
   * @param kI KI scalar for the PID Controller.
   * @param kD KD scalar for the PID Controller.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withSimClosedLoopController(double kP, double kI, double kD) {
    return withSimClosedLoopController(kP, kI, kD, ClosedLoopControllerSlot.SLOT_0);
  }

  /**
   * Set the closed loop controller for the {@link SmartMotorController}, the units passed in are in
   * Rotations (or Meters if Mechanism Circumference is configured).
   *
   * @param slot       Closed loop controller slot.
   * @param controller {@link PIDController} to use.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withSimClosedLoopController(PIDController controller, ClosedLoopControllerSlot slot) {
    this.sim_pid.put(slot, controller);
    this.sim_lqr = Optional.empty();
    return self();
  }

  /**
   * Set the closed loop controller for the {@link SmartMotorController}, the units passed in are in
   * Rotations (or Meters if Mechanism Circumference is configured).
   *
   * @param controller {@link PIDController} to use.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withSimClosedLoopController(PIDController controller) {
    return withSimClosedLoopController(controller, ClosedLoopControllerSlot.SLOT_0);
  }

  /**
   * Set the trapezoidal motion profile for the {@link SmartMotorController}.
   *
   * @param profile {@link TrapezoidProfile}; if linear use meters/s, meters/s^2; if rotational use
   *                rotations/s, rotations/s^2.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withProfile(TrapezoidProfile.Constraints profile) {
    DriverStationBackend.reportWarning("Trapezoidal profile will be given rotations/s and rotations/s^2 " + "for rotational closed loop controllers.", true);
    DriverStationBackend.reportWarning("Trapezoidal profile will be given meters/s and meters/s^2 for " + "linear closed loop controllers.", true);
    this.exponentialProfile = Optional.empty();
    this.trapezoidProfile = Optional.ofNullable(profile);
    return self();
  }

  /**
   * Set the linear trapezoidal motion profile for the {@link SmartMotorController}.
   *
   * @param maxVel   Max velocity for the profile.
   * @param maxAccel Max acceleration for the profile.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withTrapezoidalProfile(LinearVelocity maxVel, LinearAcceleration maxAccel) {
    linearClosedLoopController = true;
    this.exponentialProfile = Optional.empty();
    this.trapezoidProfile = Optional.of(new TrapezoidProfile.Constraints(maxVel.in(MetersPerSecond), maxAccel.in(MetersPerSecondPerSecond)));
    return self();
  }

  /**
   * Set the angular trapezoidal motion profile for the {@link SmartMotorController}.
   *
   * @param maxVel   Max velocity for the profile.
   * @param maxAccel Max acceleration for the profile.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withTrapezoidalProfile(AngularVelocity maxVel, AngularAcceleration maxAccel) {
    this.exponentialProfile = Optional.empty();
    this.trapezoidProfile = Optional.of(new TrapezoidProfile.Constraints(maxVel.in(RotationsPerSecond), maxAccel.in(RotationsPerSecondPerSecond)));
    return self();
  }

  /**
   * Set the angular velocity trapezoidal profile for the {@link SmartMotorController}.
   *
   * @param maxAccel Max acceleration for the profile.
   * @param maxJerk  Max velocity for the profile.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withTrapezoidalProfile(AngularAcceleration maxAccel, Velocity<AngularAccelerationUnit> maxJerk) {
    this.exponentialProfile = Optional.empty();
    this.trapezoidProfile = Optional.of(new TrapezoidProfile.Constraints(maxAccel.in(RotationsPerSecondPerSecond), maxJerk.in(RotationsPerSecondPerSecond.per(Second))));
    velocityTrapezoidalProfile = true;
    return self();
  }

  /**
   * Set the linear velocity trapezoidal profile for the {@link SmartMotorController}.
   *
   * @param maxAccel Max acceleration for the profile.
   * @param maxJerk  Max velocity for the profile.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withTrapezoidalProfile(LinearAcceleration maxAccel, Velocity<LinearAccelerationUnit> maxJerk) {
    this.exponentialProfile = Optional.empty();
    this.trapezoidProfile = Optional.of(new TrapezoidProfile.Constraints(maxAccel.in(MetersPerSecondPerSecond), maxJerk.in(MetersPerSecondPerSecond.per(Second))));
    velocityTrapezoidalProfile = true;
    linearClosedLoopController = true;
    return self();
  }

  /**
   * Set the exponential profile
   *
   * @param profile Exponential profile; meters/s, meters/s^2 for linear controllers; rotations/s,
   *                and rotations/s^2 for rotational controllers..
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withProfile(ExponentialProfile.Constraints profile) {
    DriverStationBackend.reportWarning("Exponential profile will be given rotations/s and rotations/s^2 " + "for rotational closed loop controllers.", true);
    DriverStationBackend.reportWarning("Exponential profile will be given meters/s and meters/s^2 for " + "linear closed loop controllers.", true);
    this.exponentialProfile = Optional.ofNullable(profile);
    this.trapezoidProfile = Optional.empty();
    return self();
  }

  /**
   * Get the {@link ExponentialProfile.Constraints} for an arm or flywheel.
   *
   * @param maxVolts Maximum input voltage for profile generation.
   * @param motor    {@link DCMotor} of the arm.
   * @param moi      {@link MomentOfInertia} of the arm.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withExponentialProfile(Voltage maxVolts, DCMotor motor, MomentOfInertia moi) {
    this.moi = moi;
    var sysid = Models.singleJointedArmFromPhysicalConstants(motor, moi.in(KilogramSquareMeters), gearing.getMechanismToRotorRatio());
    var A = sysid.getA(0, 0); // radians
    var B = sysid.getB(0, 0); // radians
    var kV = RadiansPerSecond.of(-A / B);
    var kA = RadiansPerSecondPerSecond.of(1.0 / B);
    this.trapezoidProfile = Optional.empty();
    this.exponentialProfile = Optional.of(ExponentialProfile.Constraints.fromCharacteristics(maxVolts.in(Volts), kV.in(RotationsPerSecond), kA.in(RotationsPerSecondPerSecond)));
    return self();
  }

  /**
   * Get the {@link ExponentialProfile.Constraints} for an elevator.
   *
   * @param maxVolts   Maximum input voltage for profile generation.
   * @param motor      {@link DCMotor} of the elevator.
   * @param mass       {@link Mass} of the elevator carriage.
   * @param drumRadius {@link Distance} of the elevator drum radius.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withExponentialProfile(Voltage maxVolts, DCMotor motor, Mass mass, Distance drumRadius) {
    var sysid = Models.elevatorFromPhysicalConstants(motor, mass.in(Kilograms), drumRadius.in(Meters), gearing.getMechanismToRotorRatio());
    var circumference = (2.0 * Math.PI * drumRadius.in(Meters));
    var A = sysid.getA(0, 0);
    var B = sysid.getB(0, 0);
    var kV = MetersPerSecond.of(-A / B);
    var kA = MetersPerSecondPerSecond.of(1.0 / B);
    this.trapezoidProfile = Optional.empty();
    this.exponentialProfile = Optional.of(ExponentialProfile.Constraints.fromCharacteristics(maxVolts.in(Volts), kV.in(MetersPerSecond), kA.in(MetersPerSecondPerSecond)));
    this.linearClosedLoopController = true;
    return self();
  }

  /**
   * Create a generic constraints object.
   *
   * @param maxVolts        Maximum input voltage for profile generation.
   * @param maxVelocity     Maximum velocity.
   * @param maxAcceleration Maximum acceleration.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withExponentialProfile(Voltage maxVolts, AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
    var maxV = maxVolts.in(Volts);
    this.trapezoidProfile = Optional.empty();
    this.exponentialProfile = Optional.of(ExponentialProfile.Constraints.fromStateSpace(maxVolts.in(Volts), maxV / maxVelocity.in(RotationsPerSecond), maxV / maxAcceleration.in(RotationsPerSecondPerSecond)));
    return self();
  }

  /**
   * Set the exponential profile for the {@link SmartMotorController}.
   *
   * @param constraints {@link ExponentialProfile.Constraints} for the profile.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withExponentialProfile(ExponentialProfile.Constraints constraints) {
    this.trapezoidProfile = Optional.empty();
    this.exponentialProfile = Optional.ofNullable(constraints);
    return self();
  }

  /**
   * Set the closed loop controller for the {@link SmartMotorController} with an exponential
   * profile, the units passed in are in Rotations (or Meters if linear closed loop controller
   * configured), outputs are in Volts.
   *
   * @param controller {@link LQRController} to use, the units passed in are in Rotations and output
   *                   is Voltage.
   * @return {@link SmartMotorControllerConfig} for chaining.
   * @implNote This clears all PIDs.
   */
  public T withClosedLoopController(LQRController controller) {
    this.pid.clear();
    this.lqr = Optional.ofNullable(controller);
    return self();
  }

  /**
   * Set the closed loop controller for the {@link SmartMotorController}, the units passed in are in
   * Rotations (or Meters if linear closed loop controller is configured), the outputs are in Volts.
   *
   * @param slot Closed loop controller slot.
   * @param kP   KP scalar for the PID Controller, the units passed in are in Rotations (or Meters
   *             if
   *             Mechanism Circumference is configured), the outputs are in Volts.
   * @param kI   KI scalar for the PID Controller, the units passed in are in Rotations (or Meters
   *             if
   *             Mechanism Circumference is configured), the outputs are in Volts.
   * @param kD   KD scalar for the PID Controller, the units passed in are in Rotations (or Meters
   *             if
   *             Mechanism Circumference is configured), the outputs are in Volts.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withClosedLoopController(double kP, double kI, double kD, ClosedLoopControllerSlot slot) {
    this.pid.put(slot, new PIDController(kP, kI, kD));
    this.lqr = Optional.empty();
    return self();
  }

  /**
   * Set the closed loop controller for the {@link SmartMotorController}, the units passed in are in
   * Rotations (or Meters if linear closed loop controller is configured), the outputs are in Volts.
   *
   * @param kP KP scalar for the PID Controller, the units passed in are in Rotations (or Meters if
   *           Mechanism Circumference is configured), the outputs are in Volts.
   * @param kI KI scalar for the PID Controller, the units passed in are in Rotations (or Meters if
   *           Mechanism Circumference is configured), the outputs are in Volts.
   * @param kD KD scalar for the PID Controller, the units passed in are in Rotations (or Meters if
   *           Mechanism Circumference is configured), the outputs are in Volts.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withClosedLoopController(double kP, double kI, double kD) {
    return withClosedLoopController(kP, kI, kD, ClosedLoopControllerSlot.SLOT_0);
  }

  /**
   * Set the closed loop controller for the {@link SmartMotorController}, the units passed in are in
   * Rotations (or Meters if linear closed loop controller is configured), the outputs are in Volts.
   *
   * @param slot       Closed loop controller slot.
   * @param controller {@link PIDController} to use, the units passed in are in Rotations (or Meters
   *                   if Mechanism Circumference is configured), the outputs are in Volts.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withClosedLoopController(PIDController controller, ClosedLoopControllerSlot slot) {
    this.pid.put(slot, controller);
    this.lqr = Optional.empty();
    return self();
  }

  /**
   * Set the closed loop controller for the {@link SmartMotorController}, the units passed in are in
   * Rotations (or Meters if linear closed loop controller is configured), the outputs are in Volts.
   *
   * @param controller {@link PIDController} to use, the units passed in are in Rotations (or Meters
   *                   if Mechanism Circumference is configured), the outputs are in Volts.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withClosedLoopController(PIDController controller) {
    return withClosedLoopController(controller, ClosedLoopControllerSlot.SLOT_0);
  }

  /**
   * Get the controller for the {@link SmartMotorController}, the units passed in are in Rotations
   * (or Meters if Mechanism Circumference is configured), the outputs are in Volts.
   *
   * @return {@link LQRController}
   */
  public Optional<LQRController> getLQRClosedLoopController() {
    // TODO: Fix to ensure its always attempted
    basicOptions.remove(BasicOptions.PID);
    if (RobotBase.isSimulation() && sim_lqr.isPresent()) {
      return sim_lqr;
    }
    return lqr;
  }

  /**
   * Get the simple closed loop controller without motion profiling.
   *
   * @param slot Closed loop controller slot.
   * @return {@link PIDController} if it exists.
   */
  public Optional<PIDController> getPID(ClosedLoopControllerSlot slot) {
    basicOptions.remove(BasicOptions.PID);
    if (RobotBase.isSimulation() && sim_pid.containsKey(slot)) {
      return Optional.of(sim_pid.get(slot));
    }
    return Optional.ofNullable(pid.getOrDefault(slot, null));
  }

  /**
   * Get the exponential profile for the closed loop controller on {@link SmartMotorController}.
   *
   * @return {@link ExponentialProfile} if it exists.
   */
  public Optional<ExponentialProfile.Constraints> getExponentialProfile() {
    basicOptions.remove(BasicOptions.ExponentialProfile);
    if (RobotBase.isSimulation() && sim_exponentialProfile.isPresent()) {
      return sim_exponentialProfile;
    }
    return exponentialProfile;
  }

  /**
   * Get the trapezoidal motion profile for the {@link SmartMotorController}.
   *
   * @return {@link TrapezoidProfile} if it exists.
   */
  public Optional<TrapezoidProfile.Constraints> getTrapezoidProfile() {
    basicOptions.remove(BasicOptions.TrapezoidalProfile);
    if (RobotBase.isSimulation() && sim_trapezoidProfile.isPresent()) {
      return sim_trapezoidProfile;
    }
    return trapezoidProfile;
  }

  /**
   * Set the {@link SimpleMotorFeedforward} for {@link SmartMotorController}, units are in
   * Rotations.
   *
   * @param slot              Closed loop controller slot.
   * @param simpleFeedforward {@link SimpleMotorFeedforward}
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withSimFeedforward(SimpleMotorFeedforward simpleFeedforward, ClosedLoopControllerSlot slot) {
    if (simpleFeedforward == null) {
      this.sim_simpleFeedforward.remove(slot);
    } else {
      this.sim_armFeedforward.remove(slot);
      this.sim_elevatorFeedforward.remove(slot);
      this.sim_simpleFeedforward.put(slot, simpleFeedforward);
    }
    return self();
  }

  /**
   * Set the {@link SimpleMotorFeedforward} for {@link SmartMotorController}, units are in
   * Rotations.
   *
   * @param simpleFeedforward {@link SimpleMotorFeedforward}
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withSimFeedforward(SimpleMotorFeedforward simpleFeedforward) {
    return withSimFeedforward(simpleFeedforward, ClosedLoopControllerSlot.SLOT_0);
  }

  /**
   * Set the closed loop controller to be Linear or {@link Distance} based.
   *
   * @param linearClosedLoopController Closed loop controller is distance based when true, angle
   *                                   based when false.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withLinearClosedLoopController(boolean linearClosedLoopController) {
    this.linearClosedLoopController = linearClosedLoopController;
    return self();
  }

  /**
   * Set the trapezoidal profile to be read explicitly as a velocity profile.
   *
   * @param velocityTrapezoidalProfile The trapezoidal profile is read explicitly as a velocity
   *                                   profile.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withVelocityTrapezoidalProfile(boolean velocityTrapezoidalProfile) {
    this.velocityTrapezoidalProfile = velocityTrapezoidalProfile;
    return self();
  }

  /**
   * Set the {@link SimpleMotorFeedforward} for {@link SmartMotorController}, units are in Rotations
   *
   * @param slot              Closed loop controller slot.
   * @param simpleFeedforward {@link SimpleMotorFeedforward}
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withFeedforward(SimpleMotorFeedforward simpleFeedforward, ClosedLoopControllerSlot slot) {
    if (simpleFeedforward == null) {
      this.simpleFeedforward.remove(slot);
    } else {
      this.armFeedforward.remove(slot);
      this.elevatorFeedforward.remove(slot);
      this.simpleFeedforward.put(slot, simpleFeedforward);
    }
    return self();
  }

  /**
   * Set the {@link SimpleMotorFeedforward} for {@link SmartMotorController}, units are in Rotations
   *
   * @param simpleFeedforward {@link SimpleMotorFeedforward}
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withFeedforward(SimpleMotorFeedforward simpleFeedforward) {
    return withFeedforward(simpleFeedforward, ClosedLoopControllerSlot.SLOT_0);
  }

  /**
   * Get the period of the {@link SmartMotorController} closed loop period.
   *
   * @return {@link SmartMotorController} closed loop controller period.
   */
  public Optional<Time> getClosedLoopControlPeriod() {
    basicOptions.remove(BasicOptions.ClosedLoopControlPeriod);
    return controlPeriod;
  }

  /**
   * Get the gearing to convert rotor rotations to mechanisms rotations connected to the {@link
   * SmartMotorController}
   *
   * @return {@link MechanismGearing} representing the gearbox and sprockets attached to the {@link
   *         SmartMotorController}.
   */
  public MechanismGearing getGearing() {
    basicOptions.remove(BasicOptions.Gearing);
    return gearing;
  }

  /**
   * Get the external encoder.
   *
   * @return Attached external encoder.
   */
  public Optional<Object> getExternalEncoder() {
    basicOptions.remove(BasicOptions.ExternalEncoder);
    return externalEncoder;
  }

  /**
   * Get the open loop ramp rate.
   *
   * @return Open loop ramp rate.
   */
  public Optional<Time> getOpenLoopRampRate() {
    basicOptions.remove(BasicOptions.OpenLoopRampRate);
    return openLoopRampRate;
  }

  /**
   * Get the closed loop ramp rate.
   *
   * @return Closed loop ramp.
   */
  public Optional<Time> getClosedLoopRampRate() {
    basicOptions.remove(BasicOptions.ClosedLoopRampRate);
    return closeLoopRampRate;
  }

  /**
   * Get Telemetry verbosity.
   *
   * @return Verbosity for telemetry.
   */
  public Optional<TelemetryVerbosity> getVerbosity() {
    return verbosity;
  }

  /**
   * Telemetry name.
   *
   * @return Telemetry name for NetworkTables.
   */
  public Optional<String> getTelemetryName() {
    return telemetryName;
  }

  /**
   * Convert {@link Velocity<LinearAccelerationUnit>} to {@link Velocity<AngularAccelerationUnit>}
   * using the {@link SmartMotorControllerConfig#mechanismCircumference}
   *
   * @param jerk Linear jerk to convert.
   * @return Equivalent angular jerk.
   */
  public Velocity<AngularAccelerationUnit> convertToMechanism(Velocity<LinearAccelerationUnit> jerk) {
    if (mechanismCircumference.isEmpty()) {
      throw new SmartMotorControllerConfigurationException("Mechanism circumference is undefined", "Cannot convert LinearVelocity to AngularVelocity.", "withMechanismCircumference(Distance)");
    }

    return RotationsPerSecondPerSecond.per(Second).of(jerk.in(MetersPerSecondPerSecond.per(Second)) / mechanismCircumference.get().in(Meters));
  }

  /**
   * Convert {@link Velocity<AngularAccelerationUnit>} to {@link Velocity<LinearAccelerationUnit>}
   * using the {@link SmartMotorControllerConfig#mechanismCircumference}
   *
   * @param jerk Angular jerk to convert.
   * @return Equivalent angular jerk.
   */
  public Velocity<LinearAccelerationUnit> convertFromMechanism(Velocity<AngularAccelerationUnit> jerk) {
    if (mechanismCircumference.isEmpty()) {
      throw new SmartMotorControllerConfigurationException("Mechanism circumference is undefined", "Cannot convert LinearVelocity to AngularVelocity.", "withMechanismCircumference(Distance)");
    }

    return MetersPerSecondPerSecond.per(Second).of(jerk.in(RotationsPerSecondPerSecond.per(Second)) * mechanismCircumference.get().in(Meters));
  }

  /**
   * Convert {@link LinearVelocity} to {@link AngularVelocity} using the {@link
   * SmartMotorControllerConfig#mechanismCircumference}
   *
   * @param velocity Linear velocity to convert.
   * @return Equivalent angular velocity.
   */
  public AngularVelocity convertToMechanism(LinearVelocity velocity) {
    if (mechanismCircumference.isEmpty()) {
      throw new SmartMotorControllerConfigurationException("Mechanism circumference is undefined", "Cannot convert LinearVelocity to AngularVelocity.", "withMechanismCircumference(Distance)");
    }

    return RotationsPerSecond.of(velocity.in(MetersPerSecond) / mechanismCircumference.get().in(Meters));
  }

  /**
   * Convert {@link LinearAcceleration} to {@link AngularAcceleration} using the {@link
   * SmartMotorControllerConfig#mechanismCircumference}
   *
   * @param acceleration Linear acceleration to convert.
   * @return Equivalent angular acceleration.
   */
  public AngularAcceleration convertToMechanism(LinearAcceleration acceleration) {
    if (mechanismCircumference.isEmpty()) {
      throw new SmartMotorControllerConfigurationException("Mechanism circumference is undefined", "Cannot convert LinearAcceleration to AngularAcceleration.", "withMechanismCircumference(Distance)");
    }

    return RotationsPerSecondPerSecond.of(acceleration.in(MetersPerSecondPerSecond) / mechanismCircumference.get().in(Meters));
  }

  /**
   * Convert {@link Distance} to {@link Angle} using {@link
   * SmartMotorControllerConfig#mechanismCircumference}
   *
   * @param distance {@link Distance} to convert to {@link Angle}
   * @return {@link Angle} of distance.
   */
  public Angle convertToMechanism(Distance distance) {
    if (mechanismCircumference.isEmpty()) {
      throw new SmartMotorControllerConfigurationException("Mechanism circumference is undefined", "Cannot convert Distance to Angle.", "withMechanismCircumference(Distance)");
    }
    return Rotations.of(distance.in(Meters) / (mechanismCircumference.get().in(Meters)));
  }

  /**
   * Convert {@link Angle} to {@link Distance} using {@link
   * SmartMotorControllerConfig#mechanismCircumference}
   *
   * @param rotations Rotations to convert.
   * @return Distance of the mechanism.
   */
  public Distance convertFromMechanism(Angle rotations) {
    if (mechanismCircumference.isEmpty()) {
      throw new SmartMotorControllerConfigurationException("Mechanism circumference is undefined", "Cannot convert Angle to Distance.", "withMechanismCircumference(Distance)");
    }
    return Meters.of(rotations.in(Rotations) * mechanismCircumference.get().in(Meters));
  }

  /**
   * Convert {@link Angle} to {@link LinearVelocity} using {@link
   * SmartMotorControllerConfig#mechanismCircumference}
   *
   * @param velocity Velocity to convert.
   * @return Velocity of the mechanism.
   */
  public LinearVelocity convertFromMechanism(AngularVelocity velocity) {
    if (mechanismCircumference.isEmpty()) {
      throw new SmartMotorControllerConfigurationException("Mechanism circumference is undefined", "Cannot convert AngularVelocity to LinearVelocity.", "withMechanismCircumference(Distance)");
    }
    return MetersPerSecond.of(velocity.in(RotationsPerSecond) * mechanismCircumference.get().in(Meters));
  }

  /**
   * Convert {@link Angle} to {@link LinearAcceleration} using {@link
   * SmartMotorControllerConfig#mechanismCircumference}
   *
   * @param acceleration Rotations to convert.
   * @return Acceleration of the mechanism.
   */
  public LinearAcceleration convertFromMechanism(AngularAcceleration acceleration) {
    if (mechanismCircumference.isEmpty()) {
      throw new SmartMotorControllerConfigurationException("Mechanism circumference is undefined", "Cannot convert AngularAcceleration to LinearAcceleration.", "withMechanismCircumference(Distance)");
    }
    return MetersPerSecondPerSecond.of(acceleration.in(RotationsPerSecondPerSecond) * mechanismCircumference.get().in(Meters));
  }

  /**
   * Convert a feedforward {@link Force} applied at the mechanism into the equivalent motor
   * feedforward {@link Voltage}, using {@link SmartMotorControllerConfig#gearing} and {@link
   * SmartMotorControllerConfig#mechanismCircumference}.
   *
   * @param motor             {@link DCMotor} of the mechanism.
   * @param mechanismVelocity Commanded mechanism (post-gearbox) {@link AngularVelocity}.
   * @param feedforwardForce  Feedforward {@link Force} applied to the mechanism.
   * @return Equivalent feedforward {@link Voltage} at the motor.
   */
  public Voltage convertToVoltage(DCMotor motor, AngularVelocity mechanismVelocity, Force feedforwardForce) {
    double rotorAngularVelocityRadPerSec = mechanismVelocity.in(RadiansPerSecond) * gearing.getMechanismToRotorRatio();
    return Volts.of(motor.getVoltage(forceToRotorTorque(feedforwardForce), rotorAngularVelocityRadPerSec));
  }

  /**
   * Convert a feedforward {@link Force} applied at the mechanism into the equivalent motor
   * feedforward {@link Current}, using {@link SmartMotorControllerConfig#gearing} and {@link
   * SmartMotorControllerConfig#mechanismCircumference}. Unlike {@link #convertToVoltage(DCMotor,
   * AngularVelocity, Force)}, this does not depend on the commanded speed and is the correct
   * feedforward to use for torque-current based closed-loop control (e.g. TorqueCurrentFOC).
   *
   * @param motor            {@link DCMotor} of the mechanism.
   * @param feedforwardForce Feedforward {@link Force} applied to the mechanism.
   * @return Equivalent feedforward {@link Current} at the motor.
   */
  public Current convertToCurrent(DCMotor motor, Force feedforwardForce) {
    return Amps.of(motor.getCurrent(forceToRotorTorque(feedforwardForce)));
  }

  /**
   * Convert a feedforward {@link Force} applied at the mechanism into the equivalent rotor torque,
   * using {@link SmartMotorControllerConfig#gearing} and {@link
   * SmartMotorControllerConfig#mechanismCircumference}.
   *
   * @param feedforwardForce Feedforward {@link Force} applied to the mechanism.
   * @return Equivalent rotor torque in Newton-meters.
   */
  private double forceToRotorTorque(Force feedforwardForce) {
    if (mechanismCircumference.isEmpty()) {
      throw new SmartMotorControllerConfigurationException("Mechanism circumference is undefined", "Cannot convert feedforward Force to Voltage/Current.", "withMechanismCircumference(Distance)");
    }
    double gearRatio = gearing.getMechanismToRotorRatio();
    double radiusMeters = mechanismCircumference.get().in(Meters) / (2 * Math.PI);
    return feedforwardForce.in(Newtons) * radiusMeters / gearRatio;
  }

  /**
   * Get the zero offset for the {@link SmartMotorController}
   *
   * @return {@link Angle} offset.
   */
  public Optional<Angle> getExternalEncoderZeroOffset() {
    externalEncoderOptions.remove(ExternalEncoderOptions.ZeroOffset);
    return zeroOffset;
  }

  /**
   * Get the temperature cut off for the {@link SmartMotorController}
   *
   * @return Maximum {@link Temperature}
   */
  public Optional<Temperature> getTemperatureCutoff() {
    basicOptions.remove(BasicOptions.TemperatureCutoff);
    return temperatureCutoff;
  }

  /**
   * Get the encoder inversion state
   *
   * @return encoder inversion state
   */
  public Optional<Boolean> getEncoderInverted() {
    basicOptions.remove(BasicOptions.EncoderInverted);
    return encoderInverted;
  }

  /**
   * Get the motor inversion state
   *
   * @return moto inversion state.
   */
  public Optional<Boolean> getMotorInverted() {
    basicOptions.remove(BasicOptions.MotorInverted);
    if (RobotBase.isSimulation() && motorInverted.isPresent()) {
      return Optional.of(false);
    }
    return motorInverted;
  }

  /**
   * Use the external feedback sensor.
   *
   * @return Use the attached absolute encoder.
   */
  public boolean getUseExternalFeedback() {
    externalEncoderOptions.remove(ExternalEncoderOptions.UseExternalFeedbackEncoder);
    return useExternalEncoder;
  }

  /**
   * Get the starting mechanism position of the {@link SmartMotorController}
   *
   * @return Starting Mechanism position.
   */
  public Optional<Angle> getStartingPosition() {
    basicOptions.remove(BasicOptions.StartingPosition);
    if (RobotBase.isSimulation() && sim_startingPosition.isPresent()) {
      return sim_startingPosition;
    }
    return startingPosition;
  }

  /**
   * Get the closed loop maximum voltage.
   *
   * @return Maximum voltage in Volts.
   */
  public Optional<Voltage> getClosedLoopControllerMaximumVoltage() {
    basicOptions.remove(BasicOptions.ClosedLoopControllerMaximumVoltage);
    return closedLoopControllerMaximumVoltage;
  }

  /**
   * Get the feedback synchronization threshold.
   *
   * @return Feedback synchronization threshold.
   */
  public Optional<Angle> getFeedbackSynchronizationThreshold() {
    basicOptions.remove(BasicOptions.FeedbackSynchronizationThreshold);
    return feedbackSynchronizationThreshold;
  }

  /**
   * Get the motor controller mode to use.
   *
   * @return {@link ControlMode} to use.
   */
  public ControlMode getMotorControllerMode() {
    basicOptions.remove(BasicOptions.ControlMode);
    return motorControllerMode;
  }

  /**
   * Get the external encoder gearing, default is 1:1 on a MAXPlanetary.
   *
   * @return External encoder gearing.
   */
  public Optional<MechanismGearing> getExternalEncoderGearing() {
    externalEncoderOptions.remove(ExternalEncoderOptions.ExternalGearing);
    return externalEncoderGearing;
  }

  /**
   * Set the external encoder gearing.
   *
   * @param externalEncoderGearing External encoder gearing.
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withExternalEncoderGearing(MechanismGearing externalEncoderGearing) {
    if (externalEncoderGearing.getRotorToMechanismRatio() > 1) {
      DriverStationBackend.reportWarning(
          "[IMPORTANT] Your gearing is set in a way that the external " + "encoder will exceed the maximum reading, " + "this WILL result in multiple angle's being read as the same 'angle.\n\t" + "Ignore this warning IF your mechanism will never travel outside of the slice you " + "are reading, adjust the offset accordingly.\n\t" + "You have been warned! (^.^) - Rivet",
          true);
    }
    this.externalEncoderGearing = Optional.of(externalEncoderGearing);
    return self();
  }

  /**
   * Set the external encoder gearing.
   *
   * @param reductionRatio External encoder gearing. For example, a ratio of "3:1" is 3; "1:2" is
   *                       0.5
   * @return {@link SmartMotorControllerConfig} for chaining.
   */
  public T withExternalEncoderGearing(double reductionRatio) {
    if (reductionRatio > 1) {
      DriverStationBackend.reportWarning(
          "[IMPORTANT] Your gearing is set in a way that the external " + "encoder will exceed the maximum reading, " + "this WILL result in multiple angle's being read as the same 'angle.\n\t" + "Ignore this warning IF your mechanism will never travel outside of the slice you " + "are reading, adjust the offset accordingly.\n\t" + "You have been warned! (^.^) - Rivet",
          true);
    }
    this.externalEncoderGearing = Optional.of(new MechanismGearing(reductionRatio));
    return self();
  }

  /**
   * Get the continuous wrapping point for the {@link SmartMotorController} encoder.
   *
   * @return {@link Angle} where the encoder wraps around.
   */
  public Optional<Angle> getContinuousWrapping() {
    if (maxContinuousWrappingAngle.isPresent() && minContinuousWrappingAngle.isPresent() && !minContinuousWrappingAngle.get().equals(Rotations.of(maxContinuousWrappingAngle.get().in(Rotations) - 1))) {
      throw new SmartMotorControllerConfigurationException("Bounds are not correct!", "Cannot get the discontinuity point.", "withContinuousWrapping(Rotations.of(" + Rotations.of(maxContinuousWrappingAngle.get().in(Rotations) - 1).in(
          Rotations) + "),Rotations.of(" + maxContinuousWrappingAngle.get().in(Rotations) + ")) instead ");
    }
    basicOptions.remove(BasicOptions.ContinuousWrapping);
    return maxContinuousWrappingAngle;
  }

  /**
   * Get the continuous wrapping point for the {@link SmartMotorController} encoder.
   *
   * @return {@link Angle} where the encoder wraps around.
   */
  public Optional<Angle> getContinuousWrappingMin() {
    if (maxContinuousWrappingAngle.isPresent() && minContinuousWrappingAngle.isPresent() && !minContinuousWrappingAngle.get().equals(Rotations.of(maxContinuousWrappingAngle.get().in(Rotations) - 1))) {
      throw new SmartMotorControllerConfigurationException("Bounds are not correct!", "Cannot get the discontinuity point.", "withContinuousWrapping(Rotations.of(" + Rotations.of(maxContinuousWrappingAngle.get().in(Rotations) - 1).in(
          Rotations) + "),Rotations.of(" + maxContinuousWrappingAngle.get().in(Rotations) + ")) instead ");
    }
    return minContinuousWrappingAngle;
  }

  /**
   * Get the loosely coupled follower motors.
   *
   * @return {@link SmartMotorController} list of loosely coupled followers.
   */
  public Optional<SmartMotorController[]> getLooselyCoupledFollowers() {
    basicOptions.remove(BasicOptions.LooselyCoupledFollowers);
    return looselyCoupledFollowers;
  }

  /**
   * Velocity trapezoidal profile configured.
   *
   * @return true, if trapezoidal profile is configured as a velocity profile. false otherwise.
   */
  public boolean getVelocityTrapezoidalProfileInUse() {
    return velocityTrapezoidalProfile;
  }

  /**
   * Get the vendor specific configuration object to mutate with {@link SmartMotorControllerConfig}
   * options.
   *
   * @return {@link SmartMotorController} vendor-specific configuration object.
   */
  public Optional<Object> getVendorConfig() {
    return vendorConfig;
  }

  /**
   * Set the vendor specific control request for the {@link SmartMotorController} which will be used
   * in place of default or calculated ones.
   *
   * @param vendorControlRequest Vendor specific control request for velocity or position.
   * @return {@link SmartMotorControllerConfig} for chaining
   */
  public T withVendorControlRequest(Object vendorControlRequest) {
    this.vendorControlRequest = Optional.ofNullable(vendorControlRequest);
    return self();
  }

  /**
   * Reset the old config?
   *
   * @return Should reset old config
   */
  public boolean getResetPreviousConfig() {
    basicOptions.remove(BasicOptions.resetPreviousConfig);
    return resetPreviousConfig;
  }

  /**
   * Reset the validation checks for all required options to be applied to {@link
   * SmartMotorController} from {@link
   * SmartMotorController#applyConfig(SmartMotorControllerConfig)}.
   */
  public void resetValidationCheck() {
    basicOptions = EnumSet.allOf(BasicOptions.class);
    externalEncoderOptions = EnumSet.allOf(ExternalEncoderOptions.class);
  }

  /**
   * Validate all required options are at least fetched and handled in each {@link
   * SmartMotorController} wrapper.
   */
  public void validateBasicOptions() {
    if (!basicOptions.isEmpty()) {
      System.err.println("========= Basic Option Validation FAILED ==========");
      for (BasicOptions option : basicOptions) {
        System.err.println("Missing required option: " + option);
      }
      throw new SmartMotorControllerConfigurationException("Basic options are not applied", "Cannot validate basic options.", "get");
    }
  }

  /** Validate external encoder config options for the config. */
  public void validateExternalEncoderOptions() {
    if (!externalEncoderOptions.isEmpty()) {
      System.err.println("========= External Encoder Option Validation FAILED ==========");
      for (ExternalEncoderOptions option : externalEncoderOptions) {
        System.err.println("Missing required option: " + option);
      }
      throw new SmartMotorControllerConfigurationException("External encoder options are not applied", "Cannot validate external encoder options.", "get");
    }
  }

  /**
   * Get whether or not the external encoder is inverted.
   *
   * @return External encoder inversion state
   */
  public Optional<Boolean> getExternalEncoderInverted() {
    externalEncoderOptions.remove(ExternalEncoderOptions.ExternalEncoderInverted);
    if (RobotBase.isSimulation() && externalEncoderInverted.isPresent()) {
      return Optional.of(false);
    }
    return externalEncoderInverted;
  }

  /**
   * Get the vendor specific control request.
   *
   * @return Vendor specific control request for velocity or position.
   */
  public Optional<Object> getVendorControlRequest() {
    basicOptions.remove(BasicOptions.VendorControlRequest);
    return vendorControlRequest;
  }

  /**
   * Get the external encoder discontinuity point.
   *
   * @return External encoder discontinuity point.
   */
  public Optional<Angle> getExternalEncoderDiscontinuityPoint() {
    externalEncoderOptions.remove(ExternalEncoderOptions.DiscontinuityPoint);
    return externalEncoderDiscontinuityPoint;
  }

  /**
   * Setup live tuning for the {@link SmartMotorController}
   */
  public void setupLiveTuning()
  {
    // Intentionally empty
  }

  /** Basic Options that should be applied to every {@link SmartMotorController} */
  private enum BasicOptions {
    /** Vendor specific control request for velocity or position. */
    VendorControlRequest,
    /** Persist the old config. */
    resetPreviousConfig,
    /** Control Mode */
    ControlMode,
    /** Feedback Synchronization for encoder seeding. */
    FeedbackSynchronizationThreshold,
    /** Closed loop controller maximum voltage */
    ClosedLoopControllerMaximumVoltage,
    /** Starting mechanism position of the {@link SmartMotorController} */
    StartingPosition,
    /** Integrated Encoder Inverted */
    EncoderInverted,
    /** Motor inversion state */
    MotorInverted,
    /** Temperature Cutoff */
    TemperatureCutoff,
    /** Continuous Wrapping */
    ContinuousWrapping,
    /** Closed Loop Tolerance */
    ClosedLoopTolerance,
    //    Telemetry,
    //    TelemetryVerbosity,
    //    SpecifiedTelemetryConfig,
    /** Closed loop controller upper limit. */
    UpperLimit,
    /** Closed loop controller lower limit. */
    LowerLimit,
    //    MomentOfInertia,
    /** Motor idle mode. */
    IdleMode,
    /** Voltage compensation. */
    VoltageCompensation,
    /** Follower motors */
    Followers,
    /** Loosely Coupled Follower Motors */
    LooselyCoupledFollowers,
    /** Stator current limits. */
    StatorCurrentLimit,
    /** Supply current limits. */
    SupplyCurrentLimit,
    /** Closed loop ramp rate. */
    ClosedLoopRampRate,
    /** Open loop ramp rate. */
    OpenLoopRampRate,
    /** External encoder used. */
    ExternalEncoder,
    /** Mechanism gearing from rotor to mechanisms. */
    Gearing,
    //    MechanismCircumference,
    /** Closed loop control period */
    ClosedLoopControlPeriod,
    /** Simple motor feedforward */
    SimpleFeedforward,
    /** Arm feedforward. */
    ArmFeedforward,
    /** Elevator feedforward */
    ElevatorFeedforward,
    /** PID controller. */
    PID,
    /** Trapezoidal profile. */
    TrapezoidalProfile,
    /** Exponentially profiled closed loop controller. */
    ExponentialProfile,
  }


  /** External encoder options */
  private enum ExternalEncoderOptions {
    /** External encoder offset. */
    ZeroOffset,
    /** External encoder zero centered. */
    DiscontinuityPoint,
    /** Use the encoder as a feedback device. */
    UseExternalFeedbackEncoder,
    /** External gearing. */
    ExternalGearing,
    /** External encoder inversion. */
    ExternalEncoderInverted
  }


  /** All possible options that must be checked and applied during motor config application. */
  public enum SmartMotorControllerOptions {
    /** Inversion state of the motor */
    MOTOR_INVERTED,
    /** Supply current limit */
    SUPPLY_CURRENT_LIMIT,
    /** Stator current limit. */
    STATOR_CURRENT_LIMIT,
    // TODO: Add more
  }
}
