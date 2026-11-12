// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers.config;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import java.nio.file.Path;
import java.util.Arrays;
import java.util.Locale;
import java.util.OptionalDouble;
import java.util.OptionalInt;
import org.wpilib.math.system.DCMotor;
import org.wpilib.system.Filesystem;
import org.wpilib.units.measure.Time;
import yams.core.exceptions.SmartMotorControllerConfigurationException;
import yams.core.gearing.MechanismGearing;
import yams.core.math.ADRCController;
import yams.core.motorcontrollers.SmartMotorControllerConfig;

/**
 * Active Disturbance Rejection Controller configuration.
 *
 * <p><b>Written by FRC Team 8331</b>, who contributed ADRC to YAMS.
 *
 * <p>This is the ADRC equivalent of {@code withClosedLoopController(...)}: pass it to
 * {@link SmartMotorControllerConfig#withADRCController(ADRCConfig)} and the motor controller runs
 * the ADRC on its own closed-loop thread. Mechanisms need no extra code; they keep calling
 * {@code setVelocity(...)} and {@code setPosition(...)} same as with a PID.
 *
 * <h2>Java defaults and team-editable JSON</h2>
 *
 * <p>The Java object holds the defaults, validation, and {@code b0} derivation. Anything a team
 * would tune lives in {@code src/main/deploy/adrc/<name>.json}:
 *
 * <pre>{@code
 * {
 *   "systemOrder": 1,
 *   "settlingTime": 0.2,
 *   "bandwidthScale": 1.0,
 *   "b0": 58.91814489109694,
 *   "minVoltage": -12.0,
 *   "maxVoltage": 12.0,
 *   "slewRateLimit": 0.0,
 *   "inputDelay": 0.0,
 *   "cascaded": false,
 *   "tdMethod": "none",
 *   "tdParams": []
 * }
 * }</pre>
 *
 * <p>Values set in Java with the {@code withX(...)} methods win over the file; the file wins over
 * the defaults. A missing or malformed file is reported to the Driver Station and the defaults are
 * used, except for {@code systemOrder}, which has no default: {@link #buildController} throws until
 * it is set in the file or in Java.
 *
 * <h2>What is never assumed</h2>
 *
 * <ul>
 * <li>{@code systemOrder} must come from the file or a Java call. There is no safe default, so
 * {@link #buildController} throws while it is unset.
 * <li>{@code b0} may be set in the file. Otherwise it is derived from the motor, gearing, and the
 * mechanism's moment of inertia, but only when the mechanism explicitly configured that inertia
 * with {@code withMomentOfInertia(...)} or {@code withWheelMomentOfInertia(...)}. The library's
 * placeholder inertia is never used.
 * <li>Output saturation comes from {@code withClosedLoopControllerMaximumVoltage(...)} or the
 * file. Falling back to the motor's nominal voltage is logged.
 * <li>The control period comes from {@code withClosedLoopControlPeriod(...)}, so ADRC and the loop
 * cannot disagree.
 * </ul>
 *
 * @see SmartMotorControllerConfig#withADRCController(ADRCConfig)
 * @see ADRCController
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
  /** Default tracking differentiator method. */
  public static final String DEFAULT_TD_METHOD = "none";

  /** Name of the JSON file, without the extension. */
  private final String  name;
  /** Plant order, or {@code null} when it has not been configured. */
  private Integer       systemOrder;
  /** Desired settling time, or {@code null} for the default. */
  private Double        settlingTime;
  /** Bandwidth scale, or {@code null} for the default. */
  private Double        bandwidthScale;
  /** Explicit input gain, or {@code null} to derive it. */
  private Double        b0;
  /** Explicit minimum output, or {@code null} for {@code -maxVoltage}. */
  private Double        minVoltage;
  /** Explicit maximum output, or {@code null} to use the motor config. */
  private Double        maxVoltage;
  /** Slew-rate limit, or {@code null} for the default. */
  private Double        slewRateLimit;
  /** Input delay, or {@code null} for the default. */
  private Double        inputDelay;
  /** Cascaded ESO flag, or {@code null} for the default. */
  private Boolean       cascaded;
  /** Tracking differentiator method, or {@code null} for the default. */
  private String        tdMethod;
  /** Tracking differentiator parameters. */
  private double[]      tdParams;

  /**
   * Create a configuration that reads {@code deploy/adrc/<name>.json}.
   *
   * @param name File name without the extension.
   * @throws IllegalArgumentException if the name is blank or contains path separators.
   */
  public ADRCConfig(String name) {
    this(name, Filesystem.getDeployDirectory().toPath());
  }

  /**
   * Create a configuration that reads {@code <deployDirectory>/adrc/<name>.json}.
   *
   * <p>This overload exists for tests and tools that do not run under the robot deploy directory.
   *
   * @param name            File name without the extension.
   * @param deployDirectory Directory that contains the {@code adrc} folder.
   * @throws IllegalArgumentException if the name is blank or contains path separators.
   */
  public ADRCConfig(String name, Path deployDirectory) {
    if (name == null || name.isBlank()) {
      throw new IllegalArgumentException("ADRC config name must not be blank");
    }
    if (!name.matches("[A-Za-z0-9_-]+")) {
      throw new IllegalArgumentException("ADRC config name may only contain letters, digits, '_' and '-': " + name);
    }
    this.name = name;
    ADRCConfigLoader.load(this, deployDirectory);
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
   * Get the JSON file name, without the extension.
   *
   * @return Config name.
   */
  public String getName() {
    return name;
  }

  /**
   * Set the plant order.
   *
   * @param systemOrder Plant order, 1 through 4.
   * @return This config for chaining.
   * @throws IllegalArgumentException if the order is outside 1 through 4.
   */
  public ADRCConfig withSystemOrder(int systemOrder) {
    if (systemOrder < 1 || systemOrder > 4) {
      throw new IllegalArgumentException("systemOrder must be between 1 and 4, got: " + systemOrder);
    }
    this.systemOrder = systemOrder;
    return this;
  }

  /**
   * Set the desired closed-loop settling time.
   *
   * @param settlingTime Settling time in seconds.
   * @return This config for chaining.
   * @throws IllegalArgumentException if the value is not finite and positive.
   */
  public ADRCConfig withSettlingTime(double settlingTime) {
    requirePositiveFinite("settlingTime", settlingTime);
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
   * @param maxVoltage Maximum output magnitude in volts.
   * @return This config for chaining.
   * @throws IllegalArgumentException if the value is not finite and positive.
   */
  public ADRCConfig withMaxVoltage(double maxVoltage) {
    requirePositiveFinite("maxVoltage", maxVoltage);
    this.maxVoltage = maxVoltage;
    return this;
  }

  /**
   * Set asymmetric output saturation.
   *
   * @param minVoltage Minimum output in volts.
   * @param maxVoltage Maximum output in volts.
   * @return This config for chaining.
   * @throws IllegalArgumentException if a value is not finite or {@code minVoltage >= maxVoltage}.
   */
  public ADRCConfig withVoltageLimits(double minVoltage, double maxVoltage) {
    requireFinite("minVoltage", minVoltage);
    requirePositiveFinite("maxVoltage", maxVoltage);
    if (minVoltage >= maxVoltage) {
      throw new IllegalArgumentException("minVoltage must be below maxVoltage, got: " + minVoltage + " and " + maxVoltage);
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
   * @throws IllegalArgumentException if the value is not finite or is negative.
   */
  public ADRCConfig withSlewRateLimit(double slewRateLimit) {
    requireNonNegativeFinite("slewRateLimit", slewRateLimit);
    this.slewRateLimit = slewRateLimit;
    return this;
  }

  /**
   * Set the input delay.
   *
   * @param inputDelay Input delay in seconds; zero disables it.
   * @return This config for chaining.
   * @throws IllegalArgumentException if the value is not finite or is negative.
   */
  public ADRCConfig withInputDelay(double inputDelay) {
    requireNonNegativeFinite("inputDelay", inputDelay);
    this.inputDelay = inputDelay;
    return this;
  }

  /**
   * Select the cascaded ESO form.
   *
   * @param cascaded Whether to run the cascaded ESO.
   * @return This config for chaining.
   */
  public ADRCConfig withCascaded(boolean cascaded) {
    this.cascaded = cascaded;
    return this;
  }

  /**
   * Enable a tracking differentiator on the reference.
   *
   * @param method TD method: {@code euler}, {@code tod}, {@code ld}, {@code red}, {@code intd}, or
   *               {@code none}.
   * @param params Method parameters; may be empty to use the method defaults.
   * @return This config for chaining.
   * @throws IllegalArgumentException if the method is not one of the supported names.
   */
  public ADRCConfig withTrackingDifferentiator(String method, double... params) {
    String normalized = method == null ? "" : method.toLowerCase(Locale.ROOT);
    if (!normalized.equals("none") && !normalized.equals("euler") && !normalized.equals("tod") && !normalized.equals("ld") && !normalized.equals("red") && !normalized.equals("intd")) {
      throw new IllegalArgumentException("Unknown TD method: " + method);
    }
    this.tdMethod = normalized;
    this.tdParams = params == null || params.length == 0 ? null : params.clone();
    return this;
  }

  /**
   * Get the configured plant order, if any.
   *
   * @return Plant order, or an empty optional when it has not been configured.
   */
  public OptionalInt getSystemOrder() {
    return systemOrder == null ? OptionalInt.empty() : OptionalInt.of(systemOrder);
  }

  /**
   * Get the effective settling time.
   *
   * @return Settling time in seconds.
   */
  public double getSettlingTime() {
    return settlingTime == null ? DEFAULT_SETTLING_TIME : settlingTime;
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
   * @return Minimum output in volts, or an empty optional for {@code -maxVoltage}.
   */
  public OptionalDouble getMinVoltage() {
    return minVoltage == null ? OptionalDouble.empty() : OptionalDouble.of(minVoltage);
  }

  /**
   * Get the explicitly configured maximum output, if any.
   *
   * @return Maximum output in volts, or an empty optional to use the motor config.
   */
  public OptionalDouble getMaxVoltage() {
    return maxVoltage == null ? OptionalDouble.empty() : OptionalDouble.of(maxVoltage);
  }

  /**
   * Get the effective slew-rate limit.
   *
   * @return Slew-rate limit, or zero when disabled.
   */
  public double getSlewRateLimit() {
    return slewRateLimit == null ? DEFAULT_SLEW_RATE_LIMIT : slewRateLimit;
  }

  /**
   * Get the effective input delay.
   *
   * @return Input delay in seconds, or zero when disabled.
   */
  public double getInputDelay() {
    return inputDelay == null ? DEFAULT_INPUT_DELAY : inputDelay;
  }

  /**
   * Whether the cascaded ESO form is enabled.
   *
   * @return {@code true} when the cascaded ESO is enabled.
   */
  public boolean isCascaded() {
    return cascaded != null && cascaded;
  }

  /**
   * Get the effective tracking differentiator method.
   *
   * @return TD method name, or {@code "none"}.
   */
  public String getTdMethod() {
    return tdMethod == null ? DEFAULT_TD_METHOD : tdMethod;
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
    int order = requireSystemOrder();
    double resolvedMaxVoltage = resolveMaxVoltage(motor, config);
    double resolvedMinVoltage = minVoltage == null ? -resolvedMaxVoltage : minVoltage;
    if (resolvedMinVoltage >= resolvedMaxVoltage) {
      throw new SmartMotorControllerConfigurationException("ADRC \"" + name + "\" has minVoltage >= maxVoltage", "Cannot initialize the controller.", "withADRCController(new ADRCConfig(\"" + name + "\").withVoltageLimits(-12.0, 12.0))");
    }

    ADRCController controller = new ADRCController(order);
    controller.initialize(getSettlingTime(), getBandwidthScale(), resolveB0(motor, config), resolvedMinVoltage, resolvedMaxVoltage, period.in(Seconds), null, 0.0, 1.0, getSlewRateLimit(), getInputDelay(), getTdMethod(), getTdParams(), isCascaded());
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
      throw new SmartMotorControllerConfigurationException("ADRC \"" + name + "\" has no b0 and cannot derive one", "No \"b0\" was set and the mechanism never configured its moment of inertia, so the library's placeholder inertia would be used. Inertia is different for every mechanism.", "withADRCController(new ADRCConfig(\"" + name + "\").withB0(...)) or call withMomentOfInertia(...) on the motor config");
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
      return maxVoltage;
    }
    var configured = config.getClosedLoopControllerMaximumVoltage();
    if (configured.isPresent()) {
      return configured.get().in(Volts);
    }
    ADRCConfigLoader.warn("ADRC \"" + name + "\" has no maxVoltage and the motor config has no closed-loop voltage limit; using the motor's nominal voltage (" + motor.nominalVoltage + " V).");
    return motor.nominalVoltage;
  }

  /**
   * Get the plant order, throwing a config exception when it was never set.
   *
   * @return Plant order.
   * @throws SmartMotorControllerConfigurationException if the order is missing.
   */
  private int requireSystemOrder() {
    if (systemOrder == null) {
      throw new SmartMotorControllerConfigurationException("ADRC \"" + name + "\" has no system order", "No \"systemOrder\" was set in deploy/adrc/" + name + ".json and none was set in Java.", "withADRCController(new ADRCConfig(\"" + name + "\").withSystemOrder(1))");
    }
    return systemOrder;
  }

  /**
   * Merge the non-null JSON values onto this config.
   *
   * @param json Parsed file.
   */
  void applyJson(ADRCJsonConfig json) {
    if (json.systemOrder() != null) {
      if (json.systemOrder() >= 1 && json.systemOrder() <= 4) {
        this.systemOrder = json.systemOrder();
      } else {
        ADRCConfigLoader.warn("ADRC \"" + name + "\": systemOrder must be between 1 and 4, got " + json.systemOrder() + ". Ignoring.");
      }
    }

    applyPositive(json.settlingTime(), "settlingTime", value -> this.settlingTime = value);
    applyPositive(json.bandwidthScale(), "bandwidthScale", value -> this.bandwidthScale = value);
    applyNonZero(json.b0(), "b0", value -> this.b0 = value);
    applyFinite(json.minVoltage(), "minVoltage", value -> this.minVoltage = value);
    applyPositive(json.maxVoltage(), "maxVoltage", value -> this.maxVoltage = value);
    applyNonNegative(json.slewRateLimit(), "slewRateLimit", value -> this.slewRateLimit = value);
    applyNonNegative(json.inputDelay(), "inputDelay", value -> this.inputDelay = value);

    if (json.cascaded() != null) {
      this.cascaded = json.cascaded();
    }
    if (json.tdMethod() != null) {
      String normalized = json.tdMethod().toLowerCase(Locale.ROOT);
      if (normalized.equals("none") || normalized.equals("euler") || normalized.equals("tod") || normalized.equals("ld") || normalized.equals("red") || normalized.equals("intd")) {
        this.tdMethod = normalized;
      } else {
        ADRCConfigLoader.warn("ADRC \"" + name + "\": unknown tdMethod \"" + json.tdMethod() + "\". Ignoring.");
      }
    }
    if (json.tdParams() != null) {
      this.tdParams = json.tdParams().clone();
    }

    if (minVoltage != null && maxVoltage != null && minVoltage >= maxVoltage) {
      ADRCConfigLoader.warn("ADRC \"" + name + "\": minVoltage must be below maxVoltage, got " + minVoltage + " and " + maxVoltage + ". Ignoring both.");
      this.minVoltage = null;
      this.maxVoltage = null;
    }
  }

  /**
   * Apply a positive finite JSON value.
   *
   * @param value   JSON value.
   * @param field   Field name for warnings.
   * @param applier Setter.
   */
  private void applyPositive(Double value, String field, java.util.function.DoubleConsumer applier) {
    if (value == null) {
      return;
    }
    if (Double.isFinite(value) && value > 0) {
      applier.accept(value);
    } else {
      ADRCConfigLoader.warn("ADRC \"" + name + "\": " + field + " must be finite and positive, got " + value + ". Ignoring.");
    }
  }

  /**
   * Apply a non-zero finite JSON value.
   *
   * @param value   JSON value.
   * @param field   Field name for warnings.
   * @param applier Setter.
   */
  private void applyNonZero(Double value, String field, java.util.function.DoubleConsumer applier) {
    if (value == null) {
      return;
    }
    if (Double.isFinite(value) && value != 0.0) {
      applier.accept(value);
    } else {
      ADRCConfigLoader.warn("ADRC \"" + name + "\": " + field + " must be finite and non-zero, got " + value + ". Ignoring.");
    }
  }

  /**
   * Apply a finite JSON value.
   *
   * @param value   JSON value.
   * @param field   Field name for warnings.
   * @param applier Setter.
   */
  private void applyFinite(Double value, String field, java.util.function.DoubleConsumer applier) {
    if (value == null) {
      return;
    }
    if (Double.isFinite(value)) {
      applier.accept(value);
    } else {
      ADRCConfigLoader.warn("ADRC \"" + name + "\": " + field + " must be finite, got " + value + ". Ignoring.");
    }
  }

  /**
   * Apply a non-negative finite JSON value.
   *
   * @param value   JSON value.
   * @param field   Field name for warnings.
   * @param applier Setter.
   */
  private void applyNonNegative(Double value, String field, java.util.function.DoubleConsumer applier) {
    if (value == null) {
      return;
    }
    if (Double.isFinite(value) && value >= 0) {
      applier.accept(value);
    } else {
      ADRCConfigLoader.warn("ADRC \"" + name + "\": " + field + " must be finite and non-negative, got " + value + ". Ignoring.");
    }
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
        + ", tdMethod='" + getTdMethod() + '\''
        + ", tdParams=" + Arrays.toString(tdParams)
        + '}';
  }
}
