// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers.config;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import java.util.Arrays;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalDouble;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Time;
import org.wpilib.units.measure.Velocity;
import org.wpilib.units.measure.Voltage;
import yams.core.exceptions.SmartMotorControllerConfigurationException;
import yams.core.gearing.MechanismGearing;
import yams.core.math.ADRCController;
import yams.core.math.TrackingDifferentiator;
import yams.core.motorcontrollers.SmartMotorControllerConfig;

/**
 * Configuration for an active disturbance rejection controller.
 *
 * <p><b>Written by FRC Team 8331</b>, who contributed active disturbance rejection control to YAMS.
 *
 * <p>An active disturbance rejection controller lumps everything the mechanism model does not
 * capture — friction, back-EMF, game-piece loads, dropped battery voltage, unmodeled dynamics —
 * into a single "total disturbance" and estimates it live with an extended state observer. The
 * estimate is then cancelled out of the control law, so the closed loop behaves like a clean chain
 * of integrators that settles in the time you ask for, without hand-tuning gains. There is no
 * proportional, integral, or derivative gain to tune: you describe the plant (order and input
 * gain) and the response you want (settling time), and the controller derives its own gains.
 *
 * <p>This is the drop-in equivalent of {@code withClosedLoopController(...)}: pass it to
 * {@link SmartMotorControllerConfig#withADRCController(ADRCConfig)} and the motor controller runs
 * the active disturbance rejection controller on its own closed-loop thread. Mechanisms need no
 * extra code; they keep calling {@code setVelocity(...)} and {@code setPosition(...)} exactly as
 * they would with a {@code PIDController}.
 *
 * <h2>Getting started</h2>
 *
 * <p>Like every other YAMS config class, this is a plain builder: teams set the values they tune
 * with the {@code withX(...)} methods, usually from their {@code Constants} class. A complete
 * velocity-loop configuration for a shooter wheel looks like this:
 *
 * <pre>{@code
 * new ADRCConfig("Shooter")
 *     .withSystemOrder(SystemOrder.FIRST)
 *     .withSettlingTime(Seconds.of(0.2))
 *     .withVoltageLimits(Volts.of(-12), Volts.of(12))
 * }</pre>
 *
 * <p>Anything left unset falls back to the defaults documented on the {@code withX(...)} method,
 * except for {@code systemOrder}, which has no default: {@link #buildController} throws until it
 * is set.
 *
 * <h2>Choosing the system order</h2>
 *
 * <p>The system order is the number of integrators between the motor voltage and the quantity the
 * closed loop commands and measures. Pick the lowest order that describes your loop; the observer
 * adds one more state on top of the plant order, so higher orders are more sensitive to sensor
 * noise.
 *
 * <ul>
 * <li>{@link SystemOrder#FIRST} — you command and measure velocity: flywheels, shooter wheels,
 * intake and conveyor rollers. A brushed or brushless DC motor's velocity responds to voltage
 * directly, so most FRC speed loops are first order.
 * <li>{@link SystemOrder#SECOND} — you command and measure position: arms, elevators, slides.
 * Position is the integral of velocity, which adds one order.
 * <li>{@link SystemOrder#THIRD} and {@link SystemOrder#FOURTH} — rare. Use when the measured
 * quantity sits two or more integrations behind the voltage, for example a mechanism whose
 * actuator itself has significant first-order lag. Most mechanisms never need these.
 * </ul>
 *
 * <p>Velocity loops and position loops differ only in the order; every other option in this class
 * works the same for both.
 *
 * <h2>Tuning the response</h2>
 *
 * <ul>
 * <li>{@link #withSettlingTime(Time)} — how quickly the closed loop settles after a setpoint
 * change. The default, 0.2 seconds, is a good starting point. Lower is faster and more aggressive
 * but demands more voltage and stresses the mechanism; higher is gentler.
 * <li>{@link #withBandwidthScale(double)} — scales both the closed-loop bandwidth and the
 * observer bandwidth together. Above 1.0 is more aggressive, below 1.0 is more conservative.
 * Leave it at the default of 1.0 until the settling time is dialed in.
 * <li>{@link #withB0(double)} — the approximate input gain: how strongly one volt accelerates the
 * measured quantity. It only sets the controller's operating point, so being off by 20 to 50
 * percent usually still works. Leave it unset to derive it from the motor, gearing, and moment of
 * inertia (see "What is never assumed" below); set it explicitly when the mechanism has no
 * meaningful inertia, such as a mechanism driven against a hard stop.
 * <li>{@link #withVoltageLimits(Voltage, Voltage)} — output saturation. Without it, the maximum
 * output falls back to the motor config's closed-loop voltage limit, or the motor's nominal
 * voltage with a warning.
 * <li>{@link #withSlewRateLimit(Velocity)} — caps how fast the controller output can change, in
 * volts per second. Useful to stop a violent step when the loop first enables. Zero (the default)
 * disables the cap.
 * <li>{@link #withInputDelay(Time)} — compensates a known delay between commanding the output and
 * seeing its effect, such as a deliberately slowed sensor. Zero (the default) disables it.
 * <li>{@link #withCascaded(boolean)} — runs the observer in cascaded form, which some higher-order
 * plants track more accurately. Leave it disabled unless you have a reason.
 * </ul>
 *
 * <h2>Smoothing the reference with a tracking differentiator</h2>
 *
 * <p>A step setpoint asks the controller for an instantaneous change, which spikes the output and
 * excites unmodeled dynamics. A tracking differentiator replaces the raw reference with a
 * smoothed reference plus its derivatives, so the controller follows a feasible trajectory
 * instead. Enable one with {@link #withTrackingDifferentiator(TrackingDifferentiator.Method,
 * double...)}, for example:
 *
 * <pre>{@code
 * new ADRCConfig("Shooter")
 *     .withSystemOrder(SystemOrder.FIRST)
 *     .withTrackingDifferentiator(TrackingDifferentiator.Method.EULER, 0.9)
 * }</pre>
 *
 * <p>Five methods are available, from gentle to aggressive; see {@link TrackingDifferentiator}
 * for the full parameter reference. Each entry below lists its parameters in order:
 *
 * <ul>
 * <li>{@link TrackingDifferentiator.Method#EULER} — first-order exponential smoothing. One
 * parameter, {@code alpha} in [0, 1] (default 0.9): higher is smoother and slower. The mildest
 * option; a good first choice for velocity loops.
 * <li>{@link TrackingDifferentiator.Method#TOD} — time-optimal tracking. One parameter,
 * {@code r} greater than 0 (default 1.0): higher tracks faster. Follows the reference as fast as
 * the parameter allows, so expect abrupt output changes.
 * <li>{@link TrackingDifferentiator.Method#LD} — linear differentiator. One parameter,
 * {@code lambda} greater than 0 (default 1.0); the sample period must be below
 * {@code 2 * lambda}. Smooth and predictable; a good default for position loops.
 * <li>{@link TrackingDifferentiator.Method#RED} — nonlinear differentiator. Two parameters,
 * {@code lambda1} and {@code lambda2}, both greater than 0 (default 1.0 each). Converges in
 * finite time and rejects measurement noise between samples.
 * <li>{@link TrackingDifferentiator.Method#INTD} — integral differentiator. Four parameters:
 * {@code alpha} in [0, 1) (default 0.5), and {@code beta}, {@code gamma}, {@code r} greater than
 * 0 (default 1.0 each). A smooth nonlinear option that integrates the error over a sliding
 * window.
 * </ul>
 *
 * <p>Parameters may be omitted entirely to use the method defaults, and the differentiator is off
 * by default. If the reference only ever ramps (a joystick-driven setpoint, for example), a
 * differentiator buys little; it earns its keep on step changes.
 *
 * <h2>What is never assumed</h2>
 *
 * <ul>
 * <li>{@code systemOrder} must be set with {@link #withSystemOrder(SystemOrder)}. There is no safe
 * default, so {@link #buildController} throws while it is unset.
 * <li>{@code b0} may be set with {@link #withB0(double)}. Otherwise it is derived from the motor,
 * gearing, and the mechanism's moment of inertia, but only when the mechanism explicitly
 * configured that inertia with {@code withMomentOfInertia(...)} or
 * {@code withWheelMomentOfInertia(...)}. The library's placeholder inertia is never used.
 * <li>Output saturation comes from {@code withClosedLoopControllerMaximumVoltage(...)} or
 * {@code withVoltageLimits(...)}. Falling back to the motor's nominal voltage is logged.
 * <li>The control period comes from {@code withClosedLoopControlPeriod(...)}, so the controller
 * and the loop cannot disagree.
 * </ul>
 *
 * @see SmartMotorControllerConfig#withADRCController(ADRCConfig)
 * @see ADRCController
 * @see TrackingDifferentiator
 */
public class ADRCConfig implements Cloneable {
  /** Default closed-loop settling time, seconds. */
  public static final double DEFAULT_SETTLING_TIME = 0.2;
  /** Default bandwidth scale. */
  public static final double DEFAULT_BANDWIDTH_SCALE = 1.0;
  /** Default slew-rate limit; zero disables it. */
  public static final double DEFAULT_SLEW_RATE_LIMIT = 0.0;
  /** Default input delay; zero disables it. */
  public static final double DEFAULT_INPUT_DELAY = 0.0;

  /**
   * Plant order for the active disturbance rejection controller.
   *
   * <p>The order is how many integrators sit between the motor voltage and the measured output:
   * velocity loops on a DC motor are {@link #FIRST}, position loops are {@link #SECOND}. The class
   * documentation of {@link ADRCConfig} walks through choosing a value.
   */
  public enum SystemOrder {
    /**
     * First-order plant: the loop commands and measures velocity, e.g. a flywheel or shooter
     * wheel. The common case for FRC mechanisms.
     */
    FIRST(1),
    /**
     * Second-order plant: the loop commands and measures position, e.g. an arm or elevator.
     * Position is the integral of velocity, so the plant is one order higher.
     */
    SECOND(2),
    /**
     * Third-order plant: the measured quantity sits two integrations behind the voltage. Rare;
     * see the class documentation for when this applies.
     */
    THIRD(3),
    /**
     * Fourth-order plant: three integrations between the voltage and the measurement. Rare; see
     * the class documentation for when this applies.
     */
    FOURTH(4);

    /** Plant order as an integer. */
    private final int order;

    SystemOrder(int order) {
      this.order = order;
    }

    /**
     * Get the plant order as an integer.
     *
     * @return Plant order, 1 through 4.
     */
    public int getOrder() {
      return order;
    }
  }

  /** Display name used in error and warning messages. */
  private final String                        name;
  /** Plant order, or {@code null} when it has not been configured. */
  private SystemOrder                         systemOrder;
  /** Desired settling time, or {@code null} for the default. */
  private Time                                settlingTime;
  /** Bandwidth scale, or {@code null} for the default. */
  private Double                              bandwidthScale;
  /** Explicit input gain, or {@code null} to derive it. */
  private Double                              b0;
  /** Explicit minimum output, or {@code null} for {@code -maxVoltage}. */
  private Voltage                             minVoltage;
  /** Explicit maximum output, or {@code null} to use the motor config. */
  private Voltage                             maxVoltage;
  /** Slew-rate limit, or {@code null} for the default. */
  private Velocity<VoltageUnit>               slewRateLimit;
  /** Input delay, or {@code null} for the default. */
  private Time                                inputDelay;
  /** Cascaded extended-state-observer flag, or {@code null} for the default. */
  private Boolean                             cascaded;
  /** Tracking differentiator method, or {@code null} when disabled. */
  private TrackingDifferentiator.Method       tdMethod;
  /** Tracking differentiator parameters. */
  private double[]                            tdParams;

  /**
   * Create a named configuration.
   *
   * <p>The name identifies this controller in error and warning messages, such as
   * {@code ADRC "Shooter" has no system order}. It is not used to read any file.
   *
   * @param name Display name for this controller.
   * @throws IllegalArgumentException if the name is blank or contains anything but letters, digits,
   *                                  {@code '_'} and {@code '-'}.
   */
  public ADRCConfig(String name) {
    if (name == null || name.isBlank()) {
      throw new IllegalArgumentException("ADRC config name must not be blank");
    }
    if (!name.matches("[A-Za-z0-9_-]+")) {
      throw new IllegalArgumentException("ADRC config name may only contain letters, digits, '_' and '-': " + name);
    }
    this.name = name;
  }

  /**
   * Copy an existing configuration.
   *
   * @param cfg Configuration to copy.
   */
  private ADRCConfig(ADRCConfig cfg) {
    this.name = cfg.name;
    this.systemOrder = cfg.systemOrder;
    this.settlingTime = cfg.settlingTime;
    this.bandwidthScale = cfg.bandwidthScale;
    this.b0 = cfg.b0;
    this.minVoltage = cfg.minVoltage;
    this.maxVoltage = cfg.maxVoltage;
    this.slewRateLimit = cfg.slewRateLimit;
    this.inputDelay = cfg.inputDelay;
    this.cascaded = cfg.cascaded;
    this.tdMethod = cfg.tdMethod;
    this.tdParams = cfg.tdParams == null ? null : cfg.tdParams.clone();
  }

  @Override
  public ADRCConfig clone() {
    return new ADRCConfig(this);
  }

  /**
   * Get the display name.
   *
   * @return Config name.
   */
  public String getName() {
    return name;
  }

  /**
   * Set the plant order.
   *
   * @param systemOrder Plant order.
   * @return This config for chaining.
   * @throws NullPointerException if the order is {@code null}.
   */
  public ADRCConfig withSystemOrder(SystemOrder systemOrder) {
    this.systemOrder = Objects.requireNonNull(systemOrder, "systemOrder must not be null");
    return this;
  }

  /**
   * Set the desired closed-loop settling time.
   *
   * @param settlingTime Settling time.
   * @return This config for chaining.
   * @throws NullPointerException     if the value is {@code null}.
   * @throws IllegalArgumentException if the value is not finite and positive.
   */
  public ADRCConfig withSettlingTime(Time settlingTime) {
    requirePositiveFinite("settlingTime", Objects.requireNonNull(settlingTime, "settlingTime must not be null").in(Seconds));
    this.settlingTime = settlingTime;
    return this;
  }

  /**
   * Set the closed-loop bandwidth scale.
   *
   * @param bandwidthScale Bandwidth scale; larger is more aggressive.
   * @return This config for chaining.
   * @throws IllegalArgumentException if the value is not finite and positive.
   */
  public ADRCConfig withBandwidthScale(double bandwidthScale) {
    requirePositiveFinite("bandwidthScale", bandwidthScale);
    this.bandwidthScale = bandwidthScale;
    return this;
  }

  /**
   * Set the input gain explicitly instead of deriving it.
   *
   * @param b0 Input gain in mechanism units per volt per second squared.
   * @return This config for chaining.
   * @throws IllegalArgumentException if the value is not finite or is zero.
   */
  public ADRCConfig withB0(double b0) {
    requireNonZeroFinite("b0", b0);
    this.b0 = b0;
    return this;
  }

  /**
   * Set symmetric output saturation.
   *
   * @param maxVoltage Maximum output magnitude.
   * @return This config for chaining.
   * @throws NullPointerException     if the value is {@code null}.
   * @throws IllegalArgumentException if the value is not finite and positive.
   */
  public ADRCConfig withMaxVoltage(Voltage maxVoltage) {
    requirePositiveFinite("maxVoltage", Objects.requireNonNull(maxVoltage, "maxVoltage must not be null").in(Volts));
    this.maxVoltage = maxVoltage;
    return this;
  }

  /**
   * Set asymmetric output saturation.
   *
   * @param minVoltage Minimum output.
   * @param maxVoltage Maximum output.
   * @return This config for chaining.
   * @throws NullPointerException     if a value is {@code null}.
   * @throws IllegalArgumentException if a value is not finite or {@code minVoltage >= maxVoltage}.
   */
  public ADRCConfig withVoltageLimits(Voltage minVoltage, Voltage maxVoltage) {
    double min = Objects.requireNonNull(minVoltage, "minVoltage must not be null").in(Volts);
    double max = Objects.requireNonNull(maxVoltage, "maxVoltage must not be null").in(Volts);
    requireFinite("minVoltage", min);
    requirePositiveFinite("maxVoltage", max);
    if (min >= max) {
      throw new IllegalArgumentException("minVoltage must be below maxVoltage, got: " + min + " and " + max);
    }
    this.minVoltage = minVoltage;
    this.maxVoltage = maxVoltage;
    return this;
  }

  /**
   * Set the output slew-rate limit.
   *
   * @param slewRateLimit Maximum output change per second; zero disables it.
   * @return This config for chaining.
   * @throws NullPointerException     if the value is {@code null}.
   * @throws IllegalArgumentException if the value is not finite or is negative.
   */
  public ADRCConfig withSlewRateLimit(Velocity<VoltageUnit> slewRateLimit) {
    requireNonNegativeFinite("slewRateLimit", Objects.requireNonNull(slewRateLimit, "slewRateLimit must not be null").in(Volts.per(Second)));
    this.slewRateLimit = slewRateLimit;
    return this;
  }

  /**
   * Set the input delay.
   *
   * @param inputDelay Input delay; zero disables it.
   * @return This config for chaining.
   * @throws NullPointerException     if the value is {@code null}.
   * @throws IllegalArgumentException if the value is not finite or is negative.
   */
  public ADRCConfig withInputDelay(Time inputDelay) {
    requireNonNegativeFinite("inputDelay", Objects.requireNonNull(inputDelay, "inputDelay must not be null").in(Seconds));
    this.inputDelay = inputDelay;
    return this;
  }

  /**
   * Select the cascaded extended-state-observer form.
   *
   * @param cascaded Whether to run the cascaded observer.
   * @return This config for chaining.
   */
  public ADRCConfig withCascaded(boolean cascaded) {
    this.cascaded = cascaded;
    return this;
  }

  /**
   * Enable a tracking differentiator on the reference.
   *
   * <p>The differentiator smooths the reference and supplies its derivatives, so a step setpoint
   * no longer spikes the controller output. See the class documentation for a guide to the five
   * methods and their parameters, and {@link TrackingDifferentiator} for the full reference.
   *
   * @param method Differentiation method.
   * @param params Method parameters in order; may be empty to use the method defaults.
   * @return This config for chaining.
   */
  public ADRCConfig withTrackingDifferentiator(TrackingDifferentiator.Method method, double... params) {
    this.tdMethod = method;
    this.tdParams = params == null || params.length == 0 ? null : params.clone();
    return this;
  }

  /**
   * Disable the tracking differentiator on the reference.
   *
   * @return This config for chaining.
   */
  public ADRCConfig withoutTrackingDifferentiator() {
    this.tdMethod = null;
    this.tdParams = null;
    return this;
  }

  /**
   * Get the configured plant order, if any.
   *
   * @return Plant order, or an empty optional when it has not been configured.
   */
  public Optional<SystemOrder> getSystemOrder() {
    return Optional.ofNullable(systemOrder);
  }

  /**
   * Get the effective settling time.
   *
   * @return Settling time.
   */
  public Time getSettlingTime() {
    return settlingTime == null ? Seconds.of(DEFAULT_SETTLING_TIME) : settlingTime;
  }

  /**
   * Get the effective bandwidth scale.
   *
   * @return Bandwidth scale.
   */
  public double getBandwidthScale() {
    return bandwidthScale == null ? DEFAULT_BANDWIDTH_SCALE : bandwidthScale;
  }

  /**
   * Get the explicitly configured input gain, if any.
   *
   * @return Input gain, or an empty optional when it should be derived.
   */
  public OptionalDouble getB0() {
    return b0 == null ? OptionalDouble.empty() : OptionalDouble.of(b0);
  }

  /**
   * Get the explicitly configured minimum output, if any.
   *
   * @return Minimum output, or an empty optional for {@code -maxVoltage}.
   */
  public Optional<Voltage> getMinVoltage() {
    return Optional.ofNullable(minVoltage);
  }

  /**
   * Get the explicitly configured maximum output, if any.
   *
   * @return Maximum output, or an empty optional to use the motor config.
   */
  public Optional<Voltage> getMaxVoltage() {
    return Optional.ofNullable(maxVoltage);
  }

  /**
   * Get the effective slew-rate limit.
   *
   * @return Slew-rate limit, or zero when disabled.
   */
  public Velocity<VoltageUnit> getSlewRateLimit() {
    return Volts.per(Second).of(slewRateLimit == null ? DEFAULT_SLEW_RATE_LIMIT : slewRateLimit.in(Volts.per(Second)));
  }

  /**
   * Get the effective input delay.
   *
   * @return Input delay, or zero when disabled.
   */
  public Time getInputDelay() {
    return inputDelay == null ? Seconds.of(DEFAULT_INPUT_DELAY) : inputDelay;
  }

  /**
   * Whether the cascaded extended-state-observer form is enabled.
   *
   * @return {@code true} when the cascaded observer is enabled.
   */
  public boolean isCascaded() {
    return cascaded != null && cascaded;
  }

  /**
   * Get the configured tracking differentiator method.
   *
   * @return Differentiation method, or {@code null} when the differentiator is disabled.
   */
  public TrackingDifferentiator.Method getTdMethod() {
    return tdMethod;
  }

  /**
   * Get the tracking differentiator parameters.
   *
   * @return Copy of the parameters, or {@code null} when none were set.
   */
  public double[] getTdParams() {
    return tdParams == null ? null : tdParams.clone();
  }

  /**
   * Build a controller from this config.
   *
   * @param motor  Motor model used for {@code b0} derivation and the nominal-voltage fallback.
   * @param config Motor config that carries gearing, inertia, circumference, and the loop period.
   * @param period Closed-loop control period.
   * @return Initialized {@link ADRCController}.
   * @throws SmartMotorControllerConfigurationException if the order is missing, {@code b0} cannot be
   *                                                    derived, or a limit is inconsistent.
   */
  public ADRCController buildController(DCMotor motor, SmartMotorControllerConfig<?> config, Time period) {
    int order = requireSystemOrder().getOrder();
    double resolvedMaxVoltage = resolveMaxVoltage(motor, config);
    double resolvedMinVoltage = minVoltage == null ? -resolvedMaxVoltage : minVoltage.in(Volts);
    if (resolvedMinVoltage >= resolvedMaxVoltage) {
      throw new SmartMotorControllerConfigurationException("ADRC \"" + name + "\" has minVoltage >= maxVoltage", "Cannot initialize the controller.", "withADRCController(new ADRCConfig(\"" + name + "\").withVoltageLimits(Volts.of(-12), Volts.of(12)))");
    }

    ADRCController controller = new ADRCController(order);
    controller.initialize(getSettlingTime().in(Seconds), getBandwidthScale(), resolveB0(motor, config), resolvedMinVoltage, resolvedMaxVoltage, period.in(Seconds), null, 0.0, 1.0,
                          getSlewRateLimit().in(Volts.per(Second)), getInputDelay().in(Seconds), getTdMethod(), getTdParams(), isCascaded());
    return controller;
  }

  /**
   * Resolve the input gain, deriving it only when the mechanism explicitly configured its inertia.
   *
   * @param motor  Motor model.
   * @param config Motor config that carries gearing, inertia, and circumference.
   * @return Input gain in mechanism units per volt per second squared.
   * @throws SmartMotorControllerConfigurationException if {@code b0} is not set and cannot be
   *                                                    derived safely.
   */
  public double resolveB0(DCMotor motor, SmartMotorControllerConfig<?> config) {
    if (b0 != null) {
      return b0;
    }
    if (!config.isMomentOfInertiaConfigured()) {
      throw new SmartMotorControllerConfigurationException("ADRC \"" + name + "\" has no b0 and cannot derive one", "withB0(...) was never called and the mechanism never configured its moment of inertia, so the library's placeholder inertia would be used. Inertia is different for every mechanism.", "withADRCController(new ADRCConfig(\"" + name + "\").withB0(...)) or call withMomentOfInertia(...) on the motor config");
    }
    MechanismGearing gearing = config.getGearing();
    if (gearing == null) {
      throw new SmartMotorControllerConfigurationException("ADRC \"" + name + "\" cannot derive b0 without gearing", "No gearing was configured, so the rotor-to-mechanism ratio is unknown.", "withADRCController(new ADRCConfig(\"" + name + "\").withB0(...)) or call withGearing(...) on the motor config");
    }

    // YAMS's gearing getters are named for the direction the factor converts, and the two names are
    // easy to read backwards: getMechanismToRotorRatio() returns rotor rotations per mechanism
    // rotation, which is the ratio b0 needs. This is the pre-existing naming the rest of YAMS uses.
    double rotorToMechanism = gearing.getMechanismToRotorRatio();
    if (!Double.isFinite(rotorToMechanism) || rotorToMechanism <= 0.0) {
      throw new SmartMotorControllerConfigurationException("ADRC \"" + name + "\" cannot derive b0 from gear ratio " + rotorToMechanism, "The rotor-to-mechanism ratio must be finite and positive.", "withGearing(...) with a positive reduction");
    }
    double inertia = config.getMOI();
    if (!Double.isFinite(inertia) || inertia <= 0.0) {
      throw new SmartMotorControllerConfigurationException("ADRC \"" + name + "\" cannot derive b0 from inertia " + inertia + " kg*m^2", "The mechanism moment of inertia must be finite and positive.", "withMomentOfInertia(...) with a real value");
    }
    double b0RadiansPerSecondSquaredPerVolt = rotorToMechanism * motor.Kt / (motor.R * inertia);
    double b0RotationsPerSecondSquaredPerVolt = b0RadiansPerSecondSquaredPerVolt / (2.0 * Math.PI);

    if (config.getLinearClosedLoopControllerUse()) {
      if (config.getMechanismCircumference().isEmpty()) {
        throw new SmartMotorControllerConfigurationException("ADRC \"" + name + "\" cannot derive linear b0 without a mechanism circumference", "The linear closed loop needs a circumference to convert rotations to meters.", "withMechanismCircumference(...) or withWheelDiameter(...)");
      }
      double circumferenceMeters = config.getMechanismCircumference().orElseThrow().in(Meters);
      return b0RotationsPerSecondSquaredPerVolt * circumferenceMeters;
    }
    return b0RotationsPerSecondSquaredPerVolt;
  }

  /**
   * Resolve the maximum output voltage.
   *
   * @param motor  Motor model, used for the nominal-voltage fallback.
   * @param config Motor config, which may carry a closed-loop voltage limit.
   * @return Maximum output in volts.
   */
  private double resolveMaxVoltage(DCMotor motor, SmartMotorControllerConfig<?> config) {
    if (maxVoltage != null) {
      return maxVoltage.in(Volts);
    }
    var configured = config.getClosedLoopControllerMaximumVoltage();
    if (configured.isPresent()) {
      return configured.get().in(Volts);
    }
    warn("ADRC \"" + name + "\" has no maxVoltage and the motor config has no closed-loop voltage limit; using the motor's nominal voltage (" + motor.nominalVoltage + " V).");
    return motor.nominalVoltage;
  }

  /**
   * Report a config warning to the Driver Station and stderr.
   *
   * @param message Warning text.
   */
  private static void warn(String message) {
    DriverStationBackend.reportWarning("[ADRC] " + message, false);
  }

  /**
   * Get the plant order, throwing a config exception when it was never set.
   *
   * @return Plant order.
   * @throws SmartMotorControllerConfigurationException if the order is missing.
   */
  private SystemOrder requireSystemOrder() {
    if (systemOrder == null) {
      throw new SmartMotorControllerConfigurationException("ADRC \"" + name + "\" has no system order", "withSystemOrder(...) was never called, and there is no safe default plant order.", "withADRCController(new ADRCConfig(\"" + name + "\").withSystemOrder(SystemOrder.FIRST))");
    }
    return systemOrder;
  }

  /**
   * Require a finite, positive value.
   *
   * @param field Field name.
   * @param value Value to check.
   */
  private static void requirePositiveFinite(String field, double value) {
    if (!Double.isFinite(value) || value <= 0) {
      throw new IllegalArgumentException(field + " must be finite and positive, got: " + value);
    }
  }

  /**
   * Require a finite, non-zero value.
   *
   * @param field Field name.
   * @param value Value to check.
   */
  private static void requireNonZeroFinite(String field, double value) {
    if (!Double.isFinite(value) || value == 0) {
      throw new IllegalArgumentException(field + " must be finite and non-zero, got: " + value);
    }
  }

  /**
   * Require a finite, non-negative value.
   *
   * @param field Field name.
   * @param value Value to check.
   */
  private static void requireNonNegativeFinite(String field, double value) {
    if (!Double.isFinite(value) || value < 0) {
      throw new IllegalArgumentException(field + " must be finite and non-negative, got: " + value);
    }
  }

  /**
   * Require a finite value.
   *
   * @param field Field name.
   * @param value Value to check.
   */
  private static void requireFinite(String field, double value) {
    if (!Double.isFinite(value)) {
      throw new IllegalArgumentException(field + " must be finite, got: " + value);
    }
  }

  @Override
  public String toString() {
    return "ADRCConfig{"
        + "name='" + name + '\''
        + ", systemOrder=" + systemOrder
        + ", settlingTime=" + getSettlingTime()
        + ", bandwidthScale=" + getBandwidthScale()
        + ", b0=" + b0
        + ", minVoltage=" + minVoltage
        + ", maxVoltage=" + maxVoltage
        + ", slewRateLimit=" + getSlewRateLimit()
        + ", inputDelay=" + getInputDelay()
        + ", cascaded=" + isCascaded()
        + ", tdMethod=" + getTdMethod()
        + ", tdParams=" + Arrays.toString(tdParams)
        + '}';
  }
}
