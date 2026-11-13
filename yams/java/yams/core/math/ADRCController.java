// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.math;

import static org.wpilib.units.Units.Seconds;

import java.util.Arrays;
import java.util.Optional;
import org.ejml.simple.SimpleMatrix;
import org.wpilib.units.measure.Time;

/**
 * Active disturbance rejection controller.
 *
 * <p><b>Written by FRC Team 8331</b>, who contributed active disturbance rejection control to YAMS.
 *
 * <p>An active disturbance rejection controller treats every unmodeled effect (friction,
 * gravity, back-EMF, coupling, dropped battery voltage, external loads) as a single "total
 * disturbance" and estimates it online with an extended state observer. The controller then
 * cancels the estimate while driving the modeled part of the plant with a bandwidth-parameterized
 * control law. Because the disturbance estimate absorbs plant mismatch, the controller needs very
 * little model knowledge: just the system order and an approximate input gain {@code b0}.
 *
 * <p>Most teams do not construct this class directly; they describe the controller with
 * {@link ADRCConfig} and the motor controller builds and runs
 * it. This class is the engine underneath, useful for custom loops and for understanding what the
 * configuration produces.
 *
 * <p>The controller supports plant orders 1 through 4: order 1 for velocity loops (a flywheel or
 * shooter wheel), order 2 for position loops (an arm or elevator), and orders 3 and 4 for
 * mechanisms whose measured output sits further integrations behind the input. The extended state
 * observer is one order higher than the plant, with its final state holding the disturbance
 * estimate. Gains are generated from a single closed-loop bandwidth {@code wc}; the observer
 * bandwidth is {@code 4 * wc}.
 *
 * <p>The controller keeps only its running state: the observer's estimate, its last outputs and the
 * tracking differentiator. Everything else it reads from its {@link ADRCConfig}, including the observer and control gains,
 * which the config works out again whenever its settling time, bandwidth scale or {@code b0}
 * changes. So every live-tunable value takes effect on the next step. The system order, input delay
 * and tracking differentiator are fixed when the controller is initialized.
 *
 * <p>The controller does not limit or ramp its output. A motor controller running it limits and
 * ramps the output itself, with {@code withClosedLoopControllerMaximumVoltage(...)} and
 * {@code withClosedLoopRampRate(...)}, and tells the observer the voltage it applied. Run on its
 * own, clamp the output to what the actuator can apply, ramp it with WPILib's
 * {@code SlewRateLimiter} if it needs one, and tell the observer what was applied with
 * {@link #calculate(double, double, double)}, so a clamped output is not mistaken for a
 * disturbance.
 *
 * <h2>Example velocity control</h2>
 * <pre>{@code
 * // A first-order velocity plant driven by volts, with b0 in (rotations per second) per volt
 * // per second.
 * ADRCController adrc = new ADRCController(new ADRCConfig()
 *     .withSystemOrder(SystemOrder.FIRST)
 *     .withSettlingTime(Seconds.of(0.2)));
 * // b0, which the config leaves unset, and the control period.
 * adrc.initialize(b0, Milliseconds.of(20));
 *
 * // In periodic(): clamp the output, and tell the observer what was applied last loop.
 * double volts = MathUtil.clamp(adrc.calculate(
 *     flywheel.getSpeed().in(RotationsPerSecond), // measurement
 *     setpoint.in(RotationsPerSecond),            // reference
 *     motor.getAppliedVoltage()), -12, 12);       // applied last loop
 * }</pre>
 *
 * <p>Keep {@code b0} in the units of the loop: for a velocity loop it is the velocity change per
 * second per volt.
 */
public class ADRCController {
  /** Observer bandwidth as a multiple of the controller bandwidth. */
  private static final double OBSERVER_BANDWIDTH_RATIO = 4.0;

  /**
   * Observer and control gains worked out from a config. Never changed once made: the config
   * replaces the whole object, so a step never uses a mix of old and new gains.
   */
  static final class Gains {
    /** ESO observer gains. */
    final SimpleMatrix ld;
    /** Discrete ESO state matrix. */
    final SimpleMatrix ad;
    /** Discrete ESO input matrix. */
    final SimpleMatrix bd;
    /** Controller gains. */
    final SimpleMatrix k;
    /** Input gain the matrices were worked out with, which the control law divides by. */
    final double b0;

    /**
     * Hold a set of gains.
     *
     * @param ld Observer gains.
     * @param ad Discrete state matrix.
     * @param bd Discrete input matrix.
     * @param k  Controller gains.
     * @param b0 Input gain.
     */
    Gains(SimpleMatrix ld, SimpleMatrix ad, SimpleMatrix bd, SimpleMatrix k, double b0) {
      this.ld = ld;
      this.ad = ad;
      this.bd = bd;
      this.k = k;
      this.b0 = b0;
    }

    /**
     * Copy these gains, with matrices of their own.
     *
     * @return The copy.
     */
    Gains copy() {
      return new Gains(ld.copy(), ad.copy(), bd.copy(), k.copy(), b0);
    }
  }

  /** Config this controller reads its settings and gains from. */
  private final ADRCConfig config;
  /**
   * Plant order, which sizes the observer. The config refuses another order once a controller is
   * built from it.
   */
  private final int systemOrder;
  /** ESO order, one greater than the plant order. */
  private final int controllerOrder;

  /** ESO state estimate. */
  private SimpleMatrix xhat;
  /** Second ESO state estimate, which the cascaded form takes its disturbance from. */
  private SimpleMatrix secondaryXhat;
  /** Previous output. */
  private double uPrev;
  /** Outputs of the last steps, newest first, for the input delay: its length is the delay plus one. */
  private double[] uHistory;
  /** Reference tracking differentiator, or {@code null} when it is off. */
  private TrackingDifferentiator td;
  /** Whether {@link #initialize} has completed. */
  private boolean initialized;

  /**
   * Create an active disturbance rejection controller for a config. The plant order comes from the
   * config, and the controller reads every other setting from it.
   *
   * @param config Config this controller runs.
   * @throws IllegalArgumentException if the config has no system order.
   */
  public ADRCController(ADRCConfig config) {
    this.config = config;
    this.systemOrder = config.getSystemOrder()
        .orElseThrow(() -> new IllegalArgumentException("The ADRC config has no system order"))
        .getOrder();
    this.controllerOrder = systemOrder + 1;
  }

  /**
   * Initialize the controller: give its config the values it cannot hold itself, which also works
   * out its gains, and start the observer and the output at zero.
   *
   * @param b0     Input gain used while the config leaves {@code b0} unset, such as the one
   *               {@link ADRCConfig#resolveB0} derives. Must be finite and non-zero when used.
   * @param period Control period. Must be finite and positive.
   * @throws IllegalArgumentException if a value is invalid, or the tracking differentiator cannot
   *                                  run at this period.
   */
  public void initialize(double b0, Time period) {
    this.initialized = false;
    final double seconds = period.in(Seconds);
    if (!Double.isFinite(seconds) || seconds <= 0) {
      throw new IllegalArgumentException("The control period must be finite and positive");
    }
    final double delaySteps =
        config.getInputDelay().orElse(ADRCConfig.DEFAULT_INPUT_DELAY).in(Seconds) / seconds;
    if (delaySteps > Integer.MAX_VALUE - 1) {
      throw new IllegalArgumentException("The input delay is too many control periods");
    }
    final TrackingDifferentiator newTd = config.getTdMethod()
        .map(method -> new TrackingDifferentiator(method, seconds, config.getTdParams()))
        .orElse(null);
    config.setBuiltValues(b0, period);

    this.xhat = new SimpleMatrix(controllerOrder, 1);
    this.secondaryXhat = new SimpleMatrix(controllerOrder, 1);
    this.uPrev = 0.0;
    this.uHistory = new double[(int) Math.round(delaySteps) + 1];
    this.td = newTd;
    this.initialized = true;
  }

  /**
   * Run one control step.
   *
   * @param reference       Reference.
   * @param output          Measured output.
   * @param refDerivatives  Reference derivatives, or {@code null}. Only the first
   *                        {@code systemOrder} entries are used.
   * @return Controller output.
   * @throws IllegalStateException if the controller has not been initialized.
   */
  public double step(double reference, double output, double[] refDerivatives) {
    requireInitialized();
    // One read, so the whole step uses the same gains while live tuning replaces them.
    final Gains gains = config.getGains();

    // The reference and its derivatives, up to the plant order.
    double[] ref = new double[systemOrder + 1];
    ref[0] = reference;
    if (refDerivatives != null) {
      System.arraycopy(
          refDerivatives, 0, ref, 1, Math.min(systemOrder, refDerivatives.length));
    }
    if (td != null) {
      td.update(reference);
      ref[0] = td.getY();
      ref[1] = td.getYd();
    }

    // The observers are told the output that reached the plant, one input delay ago.
    double uForEso = uHistory[uHistory.length - 1];
    this.xhat = gains.ad.mult(xhat).plus(gains.bd.scale(uForEso)).plus(gains.ld.scale(output));
    this.secondaryXhat = gains.ad.mult(secondaryXhat).plus(gains.bd.scale(uForEso))
        .plus(gains.ld.scale(xhat.get(0)));

    // The cascaded form cancels the second observer's disturbance estimate.
    SimpleMatrix disturbance = config.isCascaded() ? secondaryXhat : xhat;
    double v = ref[systemOrder] - disturbance.get(systemOrder);
    for (int i = 0; i < systemOrder; i++) {
      v += gains.k.get(i) * (ref[i] - xhat.get(i));
    }
    final double u = v / gains.b0;

    System.arraycopy(uHistory, 0, uHistory, 1, uHistory.length - 1);
    uHistory[0] = u;
    uPrev = u;
    return u;
  }

  /**
   * Run one control step using the YAMS controller convention.
   *
   * <p>This is equivalent to {@code step(setpoint, measurement, null)}.
   *
   * @param measurement Measured output.
   * @param setpoint    Reference.
   * @return Controller output.
   * @throws IllegalStateException if the controller has not been initialized.
   */
  public double calculate(double measurement, double setpoint) {
    return step(setpoint, measurement, null);
  }

  /**
   * Run one control step, telling the observer what the actuator really applied last step.
   *
   * <p>A motor controller does not always apply what it is asked for: a current limit, a low
   * battery or its own saturation can apply less. The observer works out the disturbance from the
   * output it believes moved the plant, so if it is told the output it asked for while the motor
   * got less, it mistakes the shortfall for a disturbance, winds up, and can settle into a limit
   * cycle. Giving it the applied output keeps its estimate true to what moved the mechanism.
   *
   * @param measurement   Measured output.
   * @param setpoint      Reference.
   * @param appliedOutput Output the actuator applied since the last step, such as the motor
   *                      controller's measured voltage. A non-finite value is ignored.
   * @return Controller output.
   * @throws IllegalStateException if the controller has not been initialized.
   */
  public double calculate(double measurement, double setpoint, double appliedOutput) {
    requireInitialized();
    if (Double.isFinite(appliedOutput)) {
      uHistory[0] = appliedOutput;
      uPrev = appliedOutput;
    }
    return step(setpoint, measurement, null);
  }

  /**
   * The closed-loop bandwidth a settling time and bandwidth scale give.
   *
   * @param systemOrder    Plant order.
   * @param bandwidthScale Bandwidth scale.
   * @param settlingTime   Settling time.
   * @return Closed-loop bandwidth, radians per second.
   */
  static double bandwidth(int systemOrder, double bandwidthScale, Time settlingTime) {
    return 2.0 * (systemOrder + 1) * bandwidthScale / settlingTime.in(Seconds);
  }

  /**
   * Work out the observer and controller gains.
   *
   * @param systemOrder Plant order, 1 through 4.
   * @param wc          Closed-loop bandwidth, radians per second.
   * @param b0          Input gain.
   * @param dt          Control period, seconds.
   * @return The gains.
   */
  static Gains computeGains(int systemOrder, double wc, double b0, double dt) {
    final int m = systemOrder + 1;
    final double wo = OBSERVER_BANDWIDTH_RATIO * wc;

    SimpleMatrix a = new SimpleMatrix(m, m);
    for (int i = 0; i < systemOrder; i++) {
      a.set(i, i + 1, 1.0);
    }
    SimpleMatrix b = new SimpleMatrix(m, 1);
    b.set(systemOrder - 1, 0, b0);

    SimpleMatrix ld = new SimpleMatrix(m, 1, true, computeLdData(systemOrder, Math.exp(-wo * dt), dt));
    SimpleMatrix c = new SimpleMatrix(1, m);
    c.set(0, 0, 1.0);
    SimpleMatrix iMinusLc = SimpleMatrix.identity(m).minus(ld.mult(c));

    SimpleMatrix k = new SimpleMatrix(systemOrder, 1);
    for (int i = 1; i <= systemOrder; i++) {
      k.set(systemOrder - i, 0, binomial(systemOrder, i) * Math.pow(wc, i));
    }
    return new Gains(ld, iMinusLc.mult(computeExpm(a, dt)), iMinusLc.mult(computeBd(a, b, dt)), k, b0);
  }

  /**
   * Matrix exponential by Taylor series.
   *
   * @param a  Matrix.
   * @param dt Time step.
   * @return {@code exp(a * dt)}.
   */
  private static SimpleMatrix computeExpm(SimpleMatrix a, double dt) {
    int m = a.getNumRows();
    SimpleMatrix sum = SimpleMatrix.identity(m);
    SimpleMatrix term = SimpleMatrix.identity(m);

    for (int i = 1; i < 50; i++) {
      term = term.mult(a).scale(dt / i);
      sum = sum.plus(term);
      if (term.elementMaxAbs() < 1e-15) {
        break;
      }
    }
    return sum;
  }

  /**
   * Discretize the input matrix by a Taylor series.
   *
   * @param a  State matrix.
   * @param b  Input matrix.
   * @param dt Time step.
   * @return Discrete input matrix.
   */
  private static SimpleMatrix computeBd(SimpleMatrix a, SimpleMatrix b, double dt) {
    int m = a.getNumRows();

    SimpleMatrix sum = new SimpleMatrix(m, m);
    SimpleMatrix aPow = SimpleMatrix.identity(m);
    for (int i = 1; i <= m + 1; i++) {
      sum = sum.plus(aPow.scale(Math.pow(dt, i) / factorial(i)));
      aPow = aPow.mult(a);
    }
    return sum.mult(b);
  }

  /**
   * Compute the observer gains for a plant order.
   *
   * @param systemOrder Plant order, 1 through 4.
   * @param zEso        Discrete observer pole, {@code exp(-wo * dt)}.
   * @param dt          Control period.
   * @return Observer gain vector data.
   */
  private static double[] computeLdData(int systemOrder, double zEso, double dt) {
    double[] res = new double[systemOrder + 1];
    switch (systemOrder) {
      case 1:
        res[0] = 1.0 - zEso * zEso;
        res[1] = (1.0 - zEso) * (1.0 - zEso) / dt;
        break;
      case 2:
        res[0] = 1.0 - zEso * zEso * zEso;
        res[1] = (1.0 + zEso) * (1.0 - zEso) * (1.0 - zEso) * 3.0 / (2.0 * dt);
        res[2] = (1.0 - zEso) * (1.0 - zEso) * (1.0 - zEso) / (dt * dt);
        break;
      case 3:
        res[0] = 1.0 - zEso * zEso * zEso * zEso;
        res[1] = (1.0 - zEso) * (1.0 - zEso) * (11.0 + zEso * (14.0 + 11.0 * zEso)) / (6.0 * dt);
        res[2] = (1.0 - zEso) * (1.0 - zEso) * (1.0 - zEso) * (1.0 + zEso) * 2.0 / (dt * dt);
        res[3] = (1.0 - zEso) * (1.0 - zEso) * (1.0 - zEso) * (1.0 - zEso) / (dt * dt * dt);
        break;
      case 4:
        res[0] = 1.0 - zEso * zEso * zEso * zEso * zEso;

        res[1] = 5.0 * (1.0 - zEso) * (1.0 - zEso) * (1.0 + zEso)
            * (5.0 + zEso * (2.0 + 5.0 * zEso)) / (12.0 * dt);

        res[2] = 5.0 * (1.0 - zEso) * (1.0 - zEso) * (1.0 - zEso)
            * (7.0 + zEso * (10.0 + 7.0 * zEso)) / (12.0 * dt * dt);

        res[3] = (1.0 - zEso) * (1.0 - zEso) * (1.0 - zEso) * (1.0 - zEso) * (1.0 + zEso) * 5.0
            / (2.0 * dt * dt * dt);
        res[4] = (1.0 - zEso) * (1.0 - zEso) * (1.0 - zEso) * (1.0 - zEso) * (1.0 - zEso)
            / (dt * dt * dt * dt);
        break;
      default:
        throw new IllegalStateException("Unsupported system order: " + systemOrder);
    }
    return res;
  }

  /**
   * Binomial coefficient.
   *
   * @param total  Total number of items.
   * @param choose Number of items chosen.
   * @return {@code total choose choose}.
   */
  private static double binomial(int total, int choose) {
    if (choose < 0 || choose > total) {
      return 0;
    }
    if (choose == 0 || choose == total) {
      return 1;
    }
    if (choose > total / 2) {
      choose = total - choose;
    }

    double result = 1;
    for (int i = 1; i <= choose; i++) {
      result = result * (total - i + 1) / i;
    }
    return result;
  }

  /**
   * Factorial.
   *
   * @param value Non-negative integer.
   * @return {@code value!}.
   */
  private static double factorial(int value) {
    double result = 1;
    for (int i = 2; i <= value; i++) {
      result *= i;
    }
    return result;
  }

  /**
   * Reset the observer state from a measured output.
   *
   * <p>The measured state starts where the mechanism is, so enabling the controller does not look
   * like a step from zero. The remaining states, including the disturbance estimate, start at zero,
   * and the output history starts at zero because the motors were unpowered. The tracking
   * differentiator, when enabled, also starts at the measurement.
   *
   * @param measurement Initial measured output, in the units the controller is stepped with.
   * @throws IllegalStateException    if the controller has not been initialized.
   * @throws IllegalArgumentException if {@code measurement} is not finite.
   */
  public void reset(double measurement) {
    requireInitialized();
    if (!Double.isFinite(measurement)) {
      throw new IllegalArgumentException("measurement must be finite");
    }
    double[] xhatInit = new double[controllerOrder];
    xhatInit[0] = measurement;
    reset(xhatInit, 0.0);
  }

  /**
   * Reset the observer state and output history.
   *
   * <p>Both observers start from the supplied state. The tracking differentiator, when enabled,
   * starts at {@code xhatInit[0]} (or zero when {@code xhatInit} is {@code null}).
   *
   * @param xhatInit Initial observer state, length {@code systemOrder + 1}. {@code null} starts at
   *                 zero.
   * @param uInit    Initial output.
   * @throws IllegalStateException    if the controller has not been initialized.
   * @throws IllegalArgumentException if {@code xhatInit} has the wrong length or {@code uInit} is
   *                                  not finite.
   */
  public void reset(double[] xhatInit, double uInit) {
    requireInitialized();
    if (xhatInit != null && xhatInit.length != controllerOrder) {
      throw new IllegalArgumentException("xhatInit length must equal controller order");
    }
    if (!Double.isFinite(uInit)) {
      throw new IllegalArgumentException("uInit must be finite");
    }
    double[] xInit = xhatInit == null ? new double[controllerOrder] : xhatInit.clone();
    this.xhat = new SimpleMatrix(controllerOrder, 1, true, xInit);
    this.secondaryXhat = new SimpleMatrix(controllerOrder, 1, true, xInit);
    this.uPrev = uInit;
    Arrays.fill(this.uHistory, this.uPrev);
    if (td != null) {
      td.reset(xInit[0]);
    }
  }

  /**
   * Throw unless {@link #initialize} has completed.
   *
   * @throws IllegalStateException if the controller has not been initialized.
   */
  private void requireInitialized() {
    if (!initialized) {
      throw new IllegalStateException("Controller must be initialized before use");
    }
  }

  /**
   * Get the input gain the gains were worked out with: the config's, or the one given to
   * {@link #initialize} while the config leaves it unset.
   *
   * @return Input gain.
   * @throws IllegalStateException if the controller has not been initialized.
   */
  public double getB0() {
    requireInitialized();
    return config.getGains().b0;
  }

  /**
   * Get the closed-loop bandwidth the config's settling time and bandwidth scale give.
   *
   * @return Closed-loop bandwidth, radians per second.
   */
  public double getBandwidth() {
    return bandwidth(systemOrder, getBandwidthScale(), getSettlingTime());
  }

  /**
   * Get the observer bandwidth, four times the closed-loop bandwidth.
   *
   * @return Observer bandwidth, radians per second.
   */
  public double getObserverBandwidth() {
    return OBSERVER_BANDWIDTH_RATIO * getBandwidth();
  }

  /**
   * Get the desired closed-loop settling time from the config.
   *
   * @return Settling time.
   */
  public Time getSettlingTime() {
    return config.getSettlingTime().orElse(ADRCConfig.DEFAULT_SETTLING_TIME);
  }

  /**
   * Get the closed-loop bandwidth scale from the config.
   *
   * @return Bandwidth scale.
   */
  public double getBandwidthScale() {
    return config.getBandwidthScale().orElse(ADRCConfig.DEFAULT_BANDWIDTH_SCALE);
  }

  /**
   * Get the tracking differentiator the controller smooths its reference with.
   *
   * @return The tracking differentiator, or empty when it is off.
   */
  public Optional<TrackingDifferentiator> getTrackingDifferentiator() {
    return Optional.ofNullable(td);
  }

  /**
   * Get the extended state observer's state estimate.
   *
   * <p>Entry {@code 0} is the estimated measured output, entries {@code 1..systemOrder-1} are the
   * remaining plant states, and the final entry is the total disturbance estimate.
   *
   * @return Copy of the observer state, length {@code systemOrder + 1}.
   * @throws IllegalStateException if the controller has not been initialized.
   */
  public double[] getEstimatedStates() {
    requireInitialized();
    double[] out = new double[controllerOrder];
    System.arraycopy(xhat.getDDRM().getData(), 0, out, 0, controllerOrder);
    return out;
  }

  /**
   * Get the estimated total disturbance.
   *
   * <p>This is the final observer state, in the units of the plant's highest derivative per second.
   * Divide by {@code b0} to view it as an equivalent control input.
   *
   * @return Estimated total disturbance.
   * @throws IllegalStateException if the controller has not been initialized.
   */
  public double getEstimatedDisturbance() {
    requireInitialized();
    return (config.isCascaded() ? secondaryXhat : xhat).get(systemOrder);
  }

  /**
   * Get the most recent controller output.
   *
   * @return Last output returned by {@link #step} or {@link #calculate}.
   */
  public double getLastOutput() {
    return uPrev;
  }

  /**
   * Get the plant order.
   *
   * @return Plant order.
   */
  public int getSystemOrder() {
    return systemOrder;
  }

  /**
   * Get the observer order.
   *
   * @return {@code systemOrder + 1}: one state per plant order plus the disturbance estimate.
   */
  public int getControllerOrder() {
    return controllerOrder;
  }

  /**
   * Get the config this controller reads its settings and gains from.
   *
   * @return The {@link ADRCConfig}.
   */
  public ADRCConfig getConfig() {
    return config;
  }

  /**
   * Whether {@link #initialize} has completed successfully.
   *
   * @return {@code true} if the controller is ready to use.
   */
  public boolean isInitialized() {
    return initialized;
  }
}
