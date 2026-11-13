// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.math;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import java.util.Arrays;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.concurrent.atomic.AtomicInteger;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Time;
import org.wpilib.util.Alert;
import yams.core.exceptions.SmartMotorControllerConfigurationException;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.SmartMotorControllerConfig;

/**
 * Configuration for an active disturbance rejection controller.
 *
 * <p><b>Written by FRC Team 8331</b>, who contributed active disturbance rejection control to YAMS.
 *
 * <p>An active disturbance rejection controller lumps everything the mechanism model does not
 * capture (friction, back-EMF, game-piece loads, dropped battery voltage, unmodeled dynamics)
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
 * new ADRCConfig()
 *     .withSystemOrder(SystemOrder.FIRST)
 *     .withSettlingTime(Seconds.of(0.2))
 * }</pre>
 *
 * <p>Every value starts empty. The getters return that empty {@link Optional}, and
 * {@link #buildController} fills it with the default documented on the {@code withX(...)} method.
 * The one exception is {@code systemOrder}, which has no default: {@link #buildController} throws
 * until it is set.
 *
 * <h2>Choosing the system order</h2>
 *
 * <p>The system order is the number of integrators between the motor voltage and the quantity the
 * closed loop commands and measures. Pick the lowest order that describes your loop; the observer
 * adds one more state on top of the plant order, so higher orders are more sensitive to sensor
 * noise.
 *
 * <ul>
 * <li>{@link SystemOrder#FIRST}: you command and measure velocity (flywheels, shooter wheels,
 * intake and conveyor rollers). A brushed or brushless DC motor's velocity responds to voltage
 * directly, so most FRC speed loops are first order.
 * <li>{@link SystemOrder#SECOND}: you command and measure position (arms, pivots, turrets).
 * Position is the integral of velocity, which adds one order.
 * <li>{@link SystemOrder#ELEVATOR}: an elevator's position, in a linear closed loop. An elevator's
 * speed is set by its motor's back-EMF more than by its inertia, so its position is first order,
 * with its speed per volt as {@code b0}, which is derived from the motor and gearing.
 * <li>{@link SystemOrder#THIRD} and {@link SystemOrder#FOURTH}: rare. Use when the measured
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
 * <li>{@link #withSettlingTime(Time)} sets how quickly the closed loop settles after a setpoint
 * change. The default, 0.2 seconds, is a good starting point. Lower is faster and more aggressive
 * but demands more voltage and stresses the mechanism; higher is gentler.
 * <li>{@link #withBandwidthScale(double)} scales both the closed-loop bandwidth and the
 * observer bandwidth together. Above 1.0 is more aggressive, below 1.0 is more conservative.
 * Leave it at the default of 1.0 until the settling time is dialed in.
 * <li>{@link #withB0(double)} sets the approximate input gain, meaning how strongly one volt
 * accelerates the measured quantity. It only sets the controller's operating point, so being off
 * by 20 to 50 percent usually still works. Leave it unset to derive it from the motor, gearing,
 * and moment of inertia (see "What is never assumed" below); set it explicitly when the mechanism
 * has no meaningful inertia, such as a mechanism driven against a hard stop.
 * <li>{@link #withInputDelay(Time)} compensates a known delay between commanding the output and
 * seeing its effect, such as a deliberately slowed sensor. Zero (the default) disables it.
 * <li>{@link #withCascaded(boolean)} runs the observer in cascaded form, which some higher-order
 * plants track more accurately. Leave it disabled unless you have a reason.
 * </ul>
 *
 * <p>The output limits and ramp come from the motor config, as for every closed loop controller:
 * {@code withClosedLoopControllerMaximumVoltage(...)} limits the output, and
 * {@code withClosedLoopRampRate(...)} limits how quickly it changes.
 *
 * <p>The {@code withX(...)} methods only accept values the controller can use: an invalid one,
 * such as a settling time that is not positive or a zero {@code b0}, throws
 * {@link IllegalArgumentException} and leaves the config as it was. The controller reads its
 * settings from this config instead of keeping copies, so the config never holds a value it cannot
 * run.
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
 * new ADRCConfig()
 *     .withSystemOrder(SystemOrder.FIRST)
 *     .withTrackingDifferentiator(TrackingDifferentiator.Method.EULER, 0.9)
 * }</pre>
 *
 * <p>Five methods are available, from gentle to aggressive; see {@link TrackingDifferentiator}
 * for the full parameter reference. Each entry below lists its parameters in order:
 *
 * <ul>
 * <li>{@link TrackingDifferentiator.Method#EULER}, first-order exponential smoothing. One
 * parameter, {@code alpha} in [0, 1] (default 0.9): higher is smoother and slower. The mildest
 * option; a good first choice for velocity loops.
 * <li>{@link TrackingDifferentiator.Method#TOD}, time-optimal tracking. One parameter,
 * {@code r} greater than 0 (default 1.0): higher tracks faster. Follows the reference as fast as
 * the parameter allows, so expect abrupt output changes.
 * <li>{@link TrackingDifferentiator.Method#LD}, the linear differentiator. One parameter,
 * {@code lambda} greater than 0 (default 1.0); the sample period must be below
 * {@code 2 * lambda}. Smooth and predictable; a good default for position loops.
 * <li>{@link TrackingDifferentiator.Method#RED}, the nonlinear differentiator. Two parameters,
 * {@code lambda1} and {@code lambda2}, both greater than 0 (default 1.0 each). Converges in
 * finite time and rejects measurement noise between samples.
 * <li>{@link TrackingDifferentiator.Method#INTD}, the integral differentiator. Four parameters:
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
 * <li>Output saturation comes from the motor config's
 * {@code withClosedLoopControllerMaximumVoltage(...)}. Without it, the controller saturates at the
 * motor's nominal voltage and raises an alert.
 * <li>The control period comes from {@code withClosedLoopControlPeriod(...)}, so the controller
 * and the loop cannot disagree.
 * </ul>
 *
 * @see SmartMotorControllerConfig#withADRCController(ADRCConfig)
 * @see ADRCController
 * @see TrackingDifferentiator
 */
public class ADRCConfig implements Cloneable {
  /** Closed-loop settling time used when none is set. */
  public static final Time DEFAULT_SETTLING_TIME = Seconds.of(0.2);
  /** Bandwidth scale used when none is set. */
  public static final double DEFAULT_BANDWIDTH_SCALE = 1.0;
  /** Input delay used when none is set; zero disables it. */
  public static final Time DEFAULT_INPUT_DELAY = Seconds.of(0);

  /**
   * Plant order for the active disturbance rejection controller.
   *
   * <p>The order is how many integrators sit between the motor voltage and the measured output:
   * velocity loops on a DC motor are {@link #FIRST}, position loops are {@link #SECOND}, and an
   * elevator's position is {@link #ELEVATOR}, first order with a speed per volt as {@code b0}. The
   * class documentation of {@link ADRCConfig} walks through choosing a value.
   */
  public enum SystemOrder {
    /**
     * First-order plant: the loop commands and measures velocity, e.g. a flywheel or shooter
     * wheel. The common case for FRC mechanisms.
     */
    FIRST(1, false),
    /** A flywheel's velocity loop; the same as {@link #FIRST}. */
    FLYWHEEL(1, false),
    /**
     * Second-order plant: the loop commands and measures position, e.g. an arm or turret. Position
     * is the integral of velocity, so the plant is one order higher.
     */
    SECOND(2, false),
    /** An arm's position loop; the same as {@link #SECOND}. */
    ARM(2, false),
    /**
     * An elevator's position loop, in a linear closed loop. An elevator's speed is set by its
     * motor's back-EMF more than by its inertia: each volt moves it at about the same speed. So its
     * position is a first order plant, with its speed per volt as {@code b0}, in meters per second
     * per volt, which {@link ADRCConfig#resolveB0} derives from the motor, gearing and mechanism
     * circumference. The motor config must use a linear closed loop.
     */
    ELEVATOR(1, true),
    /**
     * Third-order plant: the measured quantity sits two integrations behind the voltage. Rare;
     * see the class documentation for when this applies.
     */
    THIRD(3, false),
    /**
     * Fourth-order plant: three integrations between the voltage and the measurement. Rare; see
     * the class documentation for when this applies.
     */
    FOURTH(4, false);

    /** Plant order as an integer. */
    private final int order;
    /** Whether {@code b0} is the mechanism's speed per volt, as for an elevator. */
    private final boolean speedPerVolt;

    SystemOrder(int order, boolean speedPerVolt) {
      this.order = order;
      this.speedPerVolt = speedPerVolt;
    }

    /**
     * Get the plant order as an integer.
     *
     * @return Plant order, 1 through 4.
     */
    public int getOrder() {
      return order;
    }

    /**
     * Whether {@code b0} is the mechanism's speed per volt, derived from the motor and gearing,
     * rather than its acceleration per volt, derived from its inertia.
     *
     * @return {@code true} for {@link #ELEVATOR}.
     */
    public boolean isSpeedPerVolt() {
      return speedPerVolt;
    }
  }

  /** The motor controller this config belongs to, named in errors and alerts once it is built. */
  private String owner = "ADRC";
  /** Plant order; empty until it is configured. */
  private Optional<SystemOrder> systemOrder = Optional.empty();
  /** Desired settling time; empty for {@link #DEFAULT_SETTLING_TIME}. */
  private Optional<Time> settlingTime = Optional.empty();
  /** Bandwidth scale; empty for {@link #DEFAULT_BANDWIDTH_SCALE}. */
  private OptionalDouble bandwidthScale = OptionalDouble.empty();
  /** Explicit input gain; empty to derive it. */
  private OptionalDouble b0 = OptionalDouble.empty();
  /** Input delay; empty for {@link #DEFAULT_INPUT_DELAY}. */
  private Optional<Time> inputDelay = Optional.empty();
  /** Whether the cascaded extended-state-observer form is used. */
  private boolean cascaded = false;
  /** Tracking differentiator method; empty when it is disabled. */
  private Optional<TrackingDifferentiator.Method> tdMethod = Optional.empty();
  /** Tracking differentiator parameters; empty for the method defaults. */
  private Optional<double[]> tdParams = Optional.empty();
  /** Alert raised when the maximum output falls back to the motor's nominal voltage. */
  private Optional<Alert> nominalVoltageAlert = Optional.empty();
  /** Alert raised while this config holds a value its controller rejected. */
  private Optional<Alert> rejectedAlert = Optional.empty();
  /** Counts every config made, so each one's alerts get their own ids. */
  private static final AtomicInteger INSTANCES = new AtomicInteger();
  /** This config's number, for its alert ids. */
  private final int instance = INSTANCES.getAndIncrement();

  /** Control period of the controller built from this config; empty until one is built. */
  private Optional<Time> builtPeriod = Optional.empty();
  /** {@code b0} used while {@link #b0} is unset, such as the derived one; set when a controller is built. */
  private OptionalDouble builtB0 = OptionalDouble.empty();
  /**
   * Observer and control gains worked out from this config once a controller is built, and again
   * whenever the settling time, bandwidth scale or {@code b0} changes. Replaced whole, so the
   * controller's thread never uses a mix of old and new gains.
   */
  private volatile ADRCController.Gains gains;

  /**
   * Create a configuration. It belongs to the motor controller it is given to, whose telemetry name
   * its errors and alerts use.
   */
  public ADRCConfig() {}

  /**
   * Copy an existing configuration deeply: its settings, the values given when a controller was
   * built from it, and its own copy of the gains. The units, {@link Optional}s and the method are
   * immutable, so they are shared; the arrays and matrices are copied. The copy raises its own
   * alerts, so none are copied.
   *
   * @param cfg Configuration to copy.
   */
  private ADRCConfig(ADRCConfig cfg) {
    this.owner = cfg.owner;
    this.systemOrder = cfg.systemOrder;
    this.settlingTime = cfg.settlingTime;
    this.bandwidthScale = cfg.bandwidthScale;
    this.b0 = cfg.b0;
    this.inputDelay = cfg.inputDelay;
    this.cascaded = cfg.cascaded;
    this.tdMethod = cfg.tdMethod;
    this.tdParams = cfg.tdParams.map(double[]::clone);
    this.builtPeriod = cfg.builtPeriod;
    this.builtB0 = cfg.builtB0;
    final ADRCController.Gains cfgGains = cfg.gains;
    this.gains = cfgGains == null ? null : cfgGains.copy();
  }

  @Override
  public ADRCConfig clone() {
    return new ADRCConfig(this);
  }

  /**
   * Set the plant order. There is no default; the controller cannot be built without it.
   *
   * @param systemOrder Plant order.
   * @return This config for chaining.
   * @throws IllegalStateException if a controller was built from this config with a different
   *                               order, or a different kind of {@code b0}.
   */
  public ADRCConfig withSystemOrder(SystemOrder systemOrder) {
    if (builtPeriod.isPresent()
        && (systemOrder == null || systemOrder.getOrder() != this.systemOrder.orElseThrow().getOrder()
            || systemOrder.isSpeedPerVolt() != this.systemOrder.orElseThrow().isSpeedPerVolt())) {
      throw new IllegalStateException(owner + " already runs a " + this.systemOrder.orElseThrow()
          + " controller; a new order needs a new ADRCConfig");
    }
    this.systemOrder = Optional.ofNullable(systemOrder);
    return this;
  }

  /**
   * Set the desired closed-loop settling time. Defaults to {@link #DEFAULT_SETTLING_TIME}.
   *
   * @param settlingTime Settling time; must be finite and positive.
   * @return This config for chaining.
   * @throws IllegalArgumentException if the settling time is not finite and positive.
   */
  public ADRCConfig withSettlingTime(Time settlingTime) {
    if (settlingTime != null) {
      final double seconds = settlingTime.in(Seconds);
      if (!Double.isFinite(seconds) || seconds <= 0) {
        throw new IllegalArgumentException(owner + " settling time must be finite and positive, got " + seconds + " s");
      }
    }
    this.settlingTime = Optional.ofNullable(settlingTime);
    rebuildGains();
    return this;
  }

  /**
   * Set the closed-loop bandwidth scale. Defaults to {@link #DEFAULT_BANDWIDTH_SCALE}.
   *
   * @param bandwidthScale Bandwidth scale; must be finite and positive. Larger is more aggressive.
   * @return This config for chaining.
   * @throws IllegalArgumentException if the bandwidth scale is not finite and positive.
   */
  public ADRCConfig withBandwidthScale(double bandwidthScale) {
    if (!Double.isFinite(bandwidthScale) || bandwidthScale <= 0) {
      throw new IllegalArgumentException(owner + " bandwidth scale must be finite and positive, got " + bandwidthScale);
    }
    this.bandwidthScale = OptionalDouble.of(bandwidthScale);
    rebuildGains();
    return this;
  }

  /**
   * Set the input gain explicitly instead of deriving it.
   *
   * @param b0 Input gain in mechanism units per second squared per volt; must be finite and
   *           non-zero.
   * @return This config for chaining.
   * @throws IllegalArgumentException if {@code b0} is not finite or is zero.
   */
  public ADRCConfig withB0(double b0) {
    if (!Double.isFinite(b0) || b0 == 0.0) {
      throw new IllegalArgumentException(owner + " b0 must be finite and non-zero, got " + b0);
    }
    this.b0 = OptionalDouble.of(b0);
    rebuildGains();
    return this;
  }

  /**
   * Set the input delay. Defaults to {@link #DEFAULT_INPUT_DELAY}, which disables it.
   *
   * @param inputDelay Input delay; zero disables it. Must be finite and not negative.
   * @return This config for chaining.
   * @throws IllegalArgumentException if the delay is not finite or is negative.
   */
  public ADRCConfig withInputDelay(Time inputDelay) {
    if (inputDelay != null) {
      final double seconds = inputDelay.in(Seconds);
      if (!Double.isFinite(seconds) || seconds < 0) {
        throw new IllegalArgumentException(owner + " input delay must be finite and not negative, got " + seconds + " s");
      }
    }
    this.inputDelay = Optional.ofNullable(inputDelay);
    return this;
  }

  /**
   * Select the cascaded extended-state-observer form. Disabled by default.
   *
   * @param cascaded Whether to run the cascaded observer.
   * @return This config for chaining.
   */
  public ADRCConfig withCascaded(boolean cascaded) {
    this.cascaded = cascaded;
    return this;
  }

  /**
   * Enable a tracking differentiator on the reference. Disabled by default.
   *
   * <p>The differentiator smooths the reference and supplies its derivatives, so a step setpoint
   * no longer spikes the controller output. See the class documentation for a guide to the five
   * methods and their parameters, and {@link TrackingDifferentiator} for the full reference.
   *
   * @param method Differentiation method.
   * @param params Method parameters in order; leave them out to use the method defaults.
   * @return This config for chaining.
   */
  public ADRCConfig withTrackingDifferentiator(
      TrackingDifferentiator.Method method, double... params) {
    this.tdMethod = Optional.ofNullable(method);
    this.tdParams = params == null || params.length == 0 ? Optional.empty() : Optional.of(params.clone());
    return this;
  }

  /**
   * Disable the tracking differentiator on the reference.
   *
   * @return This config for chaining.
   */
  public ADRCConfig withoutTrackingDifferentiator() {
    this.tdMethod = Optional.empty();
    this.tdParams = Optional.empty();
    return this;
  }

  /**
   * Get the configured plant order.
   *
   * @return Plant order, or empty when it has not been configured.
   */
  public Optional<SystemOrder> getSystemOrder() {
    return systemOrder;
  }

  /**
   * Get the configured settling time.
   *
   * @return Settling time, or empty for {@link #DEFAULT_SETTLING_TIME}.
   */
  public Optional<Time> getSettlingTime() {
    return settlingTime;
  }

  /**
   * Get the configured bandwidth scale.
   *
   * @return Bandwidth scale, or empty for {@link #DEFAULT_BANDWIDTH_SCALE}.
   */
  public OptionalDouble getBandwidthScale() {
    return bandwidthScale;
  }

  /**
   * Get the configured input gain.
   *
   * @return Input gain, or empty when it should be derived.
   */
  public OptionalDouble getB0() {
    return b0;
  }

  /**
   * Get the configured input delay.
   *
   * @return Input delay, or empty for {@link #DEFAULT_INPUT_DELAY}.
   */
  public Optional<Time> getInputDelay() {
    return inputDelay;
  }

  /**
   * Whether the cascaded extended-state-observer form is enabled.
   *
   * @return {@code true} when the cascaded observer is enabled.
   */
  public boolean isCascaded() {
    return cascaded;
  }

  /**
   * Get the configured tracking differentiator method.
   *
   * @return Differentiation method, or empty when the differentiator is disabled.
   */
  public Optional<TrackingDifferentiator.Method> getTdMethod() {
    return tdMethod;
  }

  /**
   * Get the tracking differentiator parameters.
   *
   * @return Copy of the parameters, or empty for the method defaults.
   */
  public Optional<double[]> getTdParams() {
    return tdParams.map(double[]::clone);
  }

  /**
   * Build a controller from this config, filling every unset value with its default.
   *
   * <p>The controller reads its settings from this config every step, and its gains too: the config
   * keeps the control period, the {@code b0} used while it is unset, and the gains worked out from
   * them, and works the gains out again whenever its settling time, bandwidth
   * scale or {@code b0} changes. So a change, as live tuning makes, takes effect on the next step
   * without resetting the state estimate. The system order, input delay and tracking differentiator
   * only take effect when a controller is built, and a built config refuses a different order.
   *
   * <p>The controller saturates at the motor config's closed loop maximum voltage, which the motor
   * controller limits its output to as well, and has no slew-rate limit of its own: the motor
   * controller ramps its output with {@code withClosedLoopRampRate(...)}.
   *
   * @param motor  Motor model used for {@code b0} derivation and the nominal-voltage fallback.
   * @param config Motor config that carries gearing, inertia, circumference, and the loop period.
   * @param period Closed-loop control period.
   * @return Initialized {@link ADRCController}.
   * @throws SmartMotorControllerConfigurationException if the order is missing or {@code b0} cannot
   *                                                    be derived.
   * @throws IllegalArgumentException                   if {@link ADRCController#initialize} rejects
   *                                                    the derived {@code b0}, the motor config's
   *                                                    limit or the tracking differentiator.
   */
  public ADRCController buildController(
      DCMotor motor, SmartMotorControllerConfig<?> config, Time period) {
    owner = config.getTelemetryName().map(name -> name + " ADRC").orElse("ADRC");
    systemOrder.orElseThrow(() -> new SmartMotorControllerConfigurationException(
        owner + " has no system order",
        "withSystemOrder(...) was never called, and there is no safe default plant order.",
        "withADRCController(new ADRCConfig().withSystemOrder(SystemOrder.FIRST))"));
    ADRCController controller = new ADRCController(this);
    controller.initialize(resolveB0(motor, config), Volts.of(resolveMaxVoltage(motor, config)), period);
    return controller;
  }

  /**
   * Resolve the input gain: the one set, or one derived in the loop's units. For an
   * {@link SystemOrder#ELEVATOR} it is the speed per volt, from the motor, gearing and mechanism
   * circumference. Otherwise it is the acceleration per volt, derived only when the mechanism
   * explicitly configured its inertia.
   *
   * @param motor  Motor model.
   * @param config Motor config that carries gearing, inertia, and circumference.
   * @return Input gain in the loop's units per volt per second to the plant's order.
   * @throws SmartMotorControllerConfigurationException if the order is an elevator's and the closed
   *                                                    loop is not linear, or {@code b0} is not
   *                                                    set and cannot be derived safely.
   */
  public double resolveB0(DCMotor motor, SmartMotorControllerConfig<?> config) {
    final boolean elevator = systemOrder.map(SystemOrder::isSpeedPerVolt).orElse(false);
    if (elevator && !config.getLinearClosedLoopControllerUse()) {
      throw new SmartMotorControllerConfigurationException(
          owner + " is an elevator's ADRC, but the closed loop is not linear",
          "SystemOrder.ELEVATOR runs in meters, with b0 in meters per second per volt, so the motor "
              + "config needs a linear closed loop and a mechanism circumference.",
          "withLinearClosedLoopController(true) and withMechanismCircumference(...) on the motor config");
    }
    if (b0.isPresent()) {
      return b0.getAsDouble();
    }
    if (elevator) {
      return speedPerVolt(motor, config);
    }
    if (!config.isMomentOfInertiaConfigured()) {
      throw new SmartMotorControllerConfigurationException(
          owner + " has no b0 and cannot derive one",
          "withB0(...) was never called and the mechanism never configured its moment of inertia, "
              + "so the library's placeholder inertia would be used. Inertia is different for "
              + "every mechanism.",
          "withADRCController(new ADRCConfig().withB0(...)) or call withMomentOfInertia(...) on the motor config");
    }
    MechanismGearing gearing = config.getGearing();
    if (gearing == null) {
      throw new SmartMotorControllerConfigurationException(
          owner + " cannot derive b0 without gearing",
          "No gearing was configured, so the rotor-to-mechanism ratio is unknown.",
          "withADRCController(new ADRCConfig().withB0(...)) or call withGearing(...) on the motor config");
    }

    // YAMS's gearing getters are named for the direction the factor converts, and the two names are
    // easy to read backwards: getMechanismToRotorRatio() returns rotor rotations per mechanism
    // rotation, which is the ratio b0 needs. This is the pre-existing naming the rest of YAMS uses.
    double rotorToMechanism = gearing.getMechanismToRotorRatio();
    if (!Double.isFinite(rotorToMechanism) || rotorToMechanism <= 0.0) {
      throw new SmartMotorControllerConfigurationException(
          owner + " cannot derive b0 from gear ratio " + rotorToMechanism,
          "The rotor-to-mechanism ratio must be finite and positive.",
          "withGearing(...) with a positive reduction");
    }
    double inertia = config.getMOI();
    if (!Double.isFinite(inertia) || inertia <= 0.0) {
      throw new SmartMotorControllerConfigurationException(
          owner + " cannot derive b0 from inertia " + inertia + " kg*m^2",
          "The mechanism moment of inertia must be finite and positive.",
          "withMomentOfInertia(...) with a real value");
    }
    double b0RadiansPerSecondSquaredPerVolt = rotorToMechanism * motor.Kt / (motor.R * inertia);
    double b0RotationsPerSecondSquaredPerVolt = b0RadiansPerSecondSquaredPerVolt / (2.0 * Math.PI);

    if (config.getLinearClosedLoopControllerUse()) {
      if (config.getMechanismCircumference().isEmpty()) {
        throw new SmartMotorControllerConfigurationException(
            owner + " cannot derive linear b0 without a mechanism circumference",
            "The linear closed loop needs a circumference to convert rotations to meters.",
            "withMechanismCircumference(...) or withWheelDiameter(...)");
      }
      double circumferenceMeters = config.getMechanismCircumference().orElseThrow().in(Meters);
      return b0RotationsPerSecondSquaredPerVolt * circumferenceMeters;
    }
    return b0RotationsPerSecondSquaredPerVolt;
  }

  /**
   * The elevator's speed per volt: the motor's free speed per volt, through the gearing, times the
   * mechanism circumference.
   *
   * @param motor  Motor model.
   * @param config Motor config, with a linear closed loop, the gearing and the circumference.
   * @return Meters per second per volt.
   * @throws SmartMotorControllerConfigurationException if the gearing is missing or not positive.
   */
  private double speedPerVolt(DCMotor motor, SmartMotorControllerConfig<?> config) {
    final MechanismGearing gearing = config.getGearing();
    final double rotorToMechanism = gearing == null ? Double.NaN : gearing.getMechanismToRotorRatio();
    if (!Double.isFinite(rotorToMechanism) || rotorToMechanism <= 0.0) {
      throw new SmartMotorControllerConfigurationException(
          owner + " cannot derive the elevator's b0 from gear ratio " + rotorToMechanism,
          "The rotor-to-mechanism ratio must be finite and positive.",
          "withGearing(...) with a positive reduction");
    }
    // Kv is in radians per second per volt at the motor; the reduction slows it at the mechanism,
    // and each mechanism rotation moves the carriage one circumference.
    return motor.Kv / (2.0 * Math.PI * rotorToMechanism)
           * config.getMechanismCircumference().orElseThrow().in(Meters);
  }

  /**
   * Resolve the maximum output voltage: the motor config's closed loop maximum voltage, or the
   * motor's nominal voltage with an alert.
   *
   * @param motor  Motor model, used for the nominal-voltage fallback.
   * @param config Motor config, which may carry a closed-loop voltage limit.
   * @return Maximum output in volts.
   */
  private double resolveMaxVoltage(DCMotor motor, SmartMotorControllerConfig<?> config) {
    var configured = config.getClosedLoopControllerMaximumVoltage();
    if (configured.isPresent()) {
      return configured.get().in(Volts);
    }
    if (nominalVoltageAlert.isEmpty()) {
      nominalVoltageAlert = Optional.of(new Alert("YAMS", alertId("NominalVoltage"),
          owner + ": the motor config has no closed loop voltage limit "
              + "(withClosedLoopControllerMaximumVoltage); using the motor's nominal voltage ("
              + motor.nominalVoltage + " V).",
          Alert.Level.MEDIUM));
    }
    nominalVoltageAlert.get().set(true);
    return motor.nominalVoltage;
  }

  /**
   * Build an alert id of this config's own, so two configs never share an alert.
   *
   * @param alert What the alert is about.
   * @return Alert id.
   */
  private String alertId(String alert) {
    return "ADRC_" + instance + "_" + alert;
  }

  /**
   * Give this config the values a controller needs that the config does not hold, and work out the
   * gains. {@link ADRCController#initialize} calls this; a later call, as when the motor config is
   * applied again, replaces them.
   *
   * @param b0     Input gain used while {@code b0} is unset.
   * @param period Control period.
   * @throws IllegalArgumentException if a value is invalid, leaving the config as it was.
   */
  void setBuiltValues(double b0, Time period) {
    final double seconds = period.in(Seconds);
    if (!Double.isFinite(seconds) || seconds <= 0) {
      throw new IllegalArgumentException(owner + " control period must be finite and positive");
    }
    final double usedB0 = this.b0.orElse(b0);
    if (!Double.isFinite(usedB0) || usedB0 == 0.0) {
      throw new IllegalArgumentException(owner + " b0 must be finite and non-zero, got " + usedB0);
    }
    this.builtPeriod = Optional.of(period);
    this.builtB0 = OptionalDouble.of(b0);
    rebuildGains();
  }

  /** Work the gains out again from the settling time, bandwidth scale and {@code b0}, once built. */
  private void rebuildGains() {
    if (builtPeriod.isEmpty()) {
      return;
    }
    final int order = systemOrder.orElseThrow().getOrder();
    gains = ADRCController.computeGains(order,
        ADRCController.bandwidth(order, bandwidthScale.orElse(DEFAULT_BANDWIDTH_SCALE),
            settlingTime.orElse(DEFAULT_SETTLING_TIME)),
        b0.orElse(builtB0.getAsDouble()),
        builtPeriod.get().in(Seconds));
  }

  /**
   * Get the gains worked out from this config.
   *
   * @return The gains, or {@code null} until a controller is built.
   */
  ADRCController.Gains getGains() {
    return gains;
  }

  /**
   * Get the control period of the controller built from this config.
   *
   * @return Control period.
   * @throws java.util.NoSuchElementException if no controller has been built.
   */
  Time getBuiltPeriod() {
    return builtPeriod.orElseThrow();
  }

  /**
   * Report a value this config refused, as live tuning does when a tuned value throws. The alert
   * lives on the config, so it outlasts a controller rebuilt from it.
   *
   * @param reason Why the value was refused, or empty once a tuned value is accepted.
   */
  public void reportRejected(Optional<String> reason) {
    if (reason.isEmpty()) {
      rejectedAlert.ifPresent(alert -> alert.set(false));
      return;
    }
    if (rejectedAlert.isEmpty()) {
      rejectedAlert = Optional.of(new Alert("YAMS", alertId("Rejected"), "", Alert.Level.HIGH));
    }
    rejectedAlert.get().setText("Not retuned: " + reason.get());
    rejectedAlert.get().set(true);
  }

  @Override
  public String toString() {
    return "ADRCConfig{"
        + "systemOrder=" + systemOrder + ", settlingTime="
        + settlingTime + ", bandwidthScale=" + bandwidthScale + ", b0=" + b0
        + ", inputDelay=" + inputDelay + ", cascaded=" + cascaded
        + ", tdMethod=" + tdMethod + ", tdParams=" + tdParams.map(Arrays::toString) + '}';
  }
}
