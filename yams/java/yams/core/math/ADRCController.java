// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.math;

import java.util.Arrays;
import org.ejml.simple.SimpleMatrix;

/**
 * Active Disturbance Rejection Controller (ADRC).
 *
 * <p><b>Written by FRC Team 8331</b>, who contributed ADRC to YAMS.
 *
 * <p>An ADRC treats every unmodeled effect (friction, gravity, back-EMF, coupling, and external
 * loads) as a single "total disturbance" and estimates it online with an extended state observer
 * (ESO). The controller then cancels the estimate while driving the modeled part of the plant with
 * a bandwidth-parameterized control law. Because the disturbance estimate absorbs plant mismatch,
 * ADRC needs very little model knowledge: just the system order and an approximate input gain
 * {@code b0}.
 *
 * <p>The controller supports system orders 1 through 4. The ESO is one order higher than the plant,
 * with its final state holding the disturbance estimate. Gains are generated from a single
 * closed-loop bandwidth {@code wc}; the observer bandwidth is {@code 4 * wc}. The design settling
 * time and bandwidth scale are supplied through {@link #initialize}.
 *
 * <p>An optional {@link TrackingDifferentiator} can smooth the reference and provide its
 * derivatives. A first-order input delay and an output slew-rate limit are also supported. Both are
 * disabled by passing {@code 0}.
 *
 * <h2>Example velocity control</h2>
 * <pre>{@code
 * // A first-order velocity plant driven by volts, with b0 in (rotations per second) per volt
 * // per second.
 * ADRCController adrc = new ADRCController(1);
 * adrc.initialize(
 *     0.2,                                  // desired settling time, seconds
 *     1.0,                                  // closed-loop bandwidth scale
 *     b0,                                   // approximate input gain
 *     -12.0, 12.0,                          // output saturation, volts
 *     0.02,                                 // control period, seconds
 *     new double[] {0.0, 0.0},              // initial ESO state
 *     0.0,                                  // initial output
 *     1.0,                                  // b0 scale
 *     0.0,                                  // slew-rate limit, output units per second
 *     0.0,                                  // input delay, seconds
 *     "none", null,                         // tracking differentiator
 *     false);                               // cascaded ESO
 *
 * // In periodic():
 * double volts = adrc.calculate(
 *     flywheel.getSpeed().in(RotationsPerSecond), // measurement
 *     setpoint.in(RotationsPerSecond));           // reference
 * }</pre>
 *
 * <p>The output units are whatever the actuator accepts (volts, duty cycle, ...). Keep {@code b0}
 * in the matching units: for a velocity loop {@code b0} is the steady-state velocity change per
 * unit of input per second, i.e. {@code (output units / second) / input unit}.
 */
public class ADRCController {
  /** Observer bandwidth as a multiple of the controller bandwidth. */
  private static final double OBSERVER_BANDWIDTH_RATIO = 4.0;

  /** Plant order. */
  private final int systemOrder;
  /** ESO order, one greater than the plant order. */
  private final int controllerOrder;

  /** Output slew-rate limit. */
  private double slewRateLimit;
  /** ESO state estimate. */
  private SimpleMatrix xhat;
  /** Second ESO state estimate used by the cascaded form. */
  private SimpleMatrix secondaryXhat;
  /** ESO observer gains. */
  private SimpleMatrix ld;
  /** Discrete ESO state matrix. */
  private SimpleMatrix ad;
  /** Discrete ESO input matrix. */
  private SimpleMatrix bd;
  /** Controller gains. */
  private SimpleMatrix k;
  /** Continuous ESO state matrix. */
  private SimpleMatrix phi;
  /** Continuous ESO input matrix. */
  private SimpleMatrix gam;
  /** Previous output. */
  private double uPrev;
  /** Controller bandwidth. */
  private double wc;
  /** Observer bandwidth. */
  private double wo;
  /** Input gain. */
  private double b0;
  /** Input-gain scale the controller's {@code b0} is divided by. */
  private double ke;
  /** Desired closed-loop settling time, seconds. */
  private double tSettle;
  /** Closed-loop bandwidth scale. */
  private double kob;
  /** Control period. */
  private double dt;
  /** Minimum output. */
  private double uMin;
  /** Maximum output. */
  private double uMax;

  /** Input delay in whole control steps. */
  private int         inputDelaySteps;
  /** Output history used to compensate the input delay. */
  private double[]    uHistory;
  /** Whether a tracking differentiator is active. */
  private boolean     useTd;
  /** Reference tracking differentiator. */
  private TrackingDifferentiator td;
  /** Whether {@link #initialize} has completed. */
  private boolean     initialized;
  /** Whether the cascaded ESO form is used. */
  private boolean     useCascaded;

  /**
   * Create an ADRC.
   *
   * @param systemOrder Plant order, from 1 through 4.
   * @throws IllegalArgumentException if the order is outside that range.
   */
  public ADRCController(int systemOrder) {
    if (systemOrder < 1 || systemOrder > 4) {
      throw new IllegalArgumentException("System order must be between 1 and 4, got: " + systemOrder);
    }
    this.systemOrder = systemOrder;
    this.controllerOrder = systemOrder + 1;
    this.uMin = Double.NEGATIVE_INFINITY;
    this.uMax = Double.POSITIVE_INFINITY;
    this.slewRateLimit = Double.POSITIVE_INFINITY;
  }

  /**
   * Initialize the controller and compute its gains.
   *
   * @param tSettle       Desired closed-loop settling time, seconds. Must be finite and positive.
   * @param kob           Closed-loop bandwidth scale. Must be finite and positive. Larger values
   *                      are more aggressive.
   * @param b0            Approximate input gain. Must be finite and non-zero.
   * @param uMin          Minimum output.
   * @param uMax          Maximum output.
   * @param dt            Control period, seconds. Must be finite and positive.
   * @param xhatInit      Initial ESO state, length {@code systemOrder + 1}. {@code null} starts at
   *                      zero.
   * @param uInit         Initial output.
   * @param ke            Input-gain scale. The controller uses {@code b0 / ke}. Must be finite and
   *                      positive.
   * @param slewRateLimit Maximum output change per second. A non-positive or NaN value disables the
   *                      limit.
   * @param inputDelay    Input delay, seconds. Must be finite and non-negative.
   * @param tdMethod      Tracking differentiator method, or {@code "none"} to disable it.
   * @param tdParams      Tracking differentiator parameters. May be {@code null}.
   * @param useCascaded   Whether to run the cascaded ESO form.
   * @throws IllegalArgumentException if any argument is invalid.
   */
  public void initialize(double tSettle, double kob, double b0, double uMin, double uMax, double dt, double[] xhatInit, double uInit, double ke, double slewRateLimit, double inputDelay, String tdMethod, double[] tdParams, boolean useCascaded) {
    this.initialized = false;
    if (!Double.isFinite(tSettle) || tSettle <= 0 || !Double.isFinite(kob) || kob <= 0 || !Double.isFinite(dt) || dt <= 0 || !Double.isFinite(ke) || ke <= 0) {
      throw new IllegalArgumentException("tSettle, kob, dt, and ke must be finite and positive");
    }
    if (xhatInit != null && xhatInit.length != controllerOrder) {
      throw new IllegalArgumentException("xhatInit length must equal controller order");
    }
    if (!Double.isFinite(b0) || b0 == 0.0) {
      throw new IllegalArgumentException("b0 must be finite and non-zero");
    }
    if (Double.isNaN(uMin) || Double.isNaN(uMax) || uMin > uMax) {
      throw new IllegalArgumentException("uMin and uMax must not be NaN and uMin must be <= uMax");
    }
    if (slewRateLimit <= 0.0 || Double.isNaN(slewRateLimit)) {
      this.slewRateLimit = Double.POSITIVE_INFINITY;
    } else {
      this.slewRateLimit = slewRateLimit;
    }
    if (!Double.isFinite(inputDelay) || inputDelay < 0.0) {
      throw new IllegalArgumentException("inputDelay must be finite and >= 0");
    }
    if (!Double.isFinite(uInit)) {
      throw new IllegalArgumentException("uInit must be finite");
    }
    this.b0 = b0 / ke;
    if (!Double.isFinite(this.b0)) {
      throw new IllegalArgumentException("b0 / ke must be finite");
    }
    this.ke = ke;
    this.tSettle = tSettle;
    this.kob = kob;
    this.dt = dt;
    this.uMin = uMin;
    this.uMax = uMax;
    this.useCascaded = useCascaded;
    double newWc = 2.0 * (systemOrder + 1) * kob / tSettle;
    double newWo = OBSERVER_BANDWIDTH_RATIO * newWc;
    if (!Double.isFinite(newWc) || !Double.isFinite(newWo)) {
      throw new IllegalArgumentException("tuning produced non-finite bandwidths");
    }
    this.wc = newWc;
    this.wo = newWo;

    double[] xInit = xhatInit == null ? new double[controllerOrder] : xhatInit.clone();
    this.xhat = new SimpleMatrix(controllerOrder, 1, true, xInit);
    this.secondaryXhat = new SimpleMatrix(controllerOrder, 1, true, useCascaded ? xInit : new double[controllerOrder]);
    this.uPrev = uInit;

    double delaySteps = inputDelay / dt;
    if (!Double.isFinite(delaySteps) || delaySteps > Integer.MAX_VALUE - 1) {
      throw new IllegalArgumentException("inputDelay / dt is too large");
    }
    this.inputDelaySteps = (int) Math.round(delaySteps);
    this.uHistory = new double[this.inputDelaySteps + 1];
    Arrays.fill(this.uHistory, this.uPrev);

    computeEsoGains();
    computeControllerGains();

    setTd(tdMethod, tdParams);
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
    if (!initialized) {
      throw new IllegalStateException("Controller must be initialized before use");
    }

    double refFiltered = reference;
    double[] refDerivs = new double[systemOrder];
    if (refDerivatives != null) {
      System.arraycopy(refDerivatives, 0, refDerivs, 0, Math.min(systemOrder, refDerivatives.length));
    }
    if (useTd && td != null) {
      td.update(reference);
      refFiltered = td.getY();
      refDerivs[0] = td.getYd();
    }

    double uForEso = uHistory[inputDelaySteps];

    this.xhat = ad.mult(xhat).plus(bd.scale(uForEso)).plus(ld.scale(output));
    if (useCascaded) {
      updateLeso2(uForEso);
    }

    SimpleMatrix refVec = new SimpleMatrix(systemOrder + 1, 1);
    refVec.set(0, 0, refFiltered);
    for (int i = 0; i < systemOrder; i++) {
      refVec.set(i + 1, 0, refDerivs[i]);
    }

    double u = useCascaded ? computeControlCascaded(refVec) : computeControlLinear(refVec);
    double maxChange = slewRateLimit * dt;

    double deltaU = u - uPrev;
    if (Math.abs(deltaU) > maxChange) {
      deltaU = Math.signum(deltaU) * maxChange;
    }

    u = uPrev + deltaU;
    u = saturate(u);

    if (uHistory.length > 1) {
      System.arraycopy(uHistory, 0, uHistory, 1, uHistory.length - 1);
    }
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
   * <p>A motor controller does not always apply what it is asked for: a current limit, a low battery
   * or its own saturation can apply less. The observer works out the disturbance from the output it
   * believes moved the plant, so if it is told the output it asked for while the motor got less, it
   * mistakes the shortfall for a disturbance, winds up, and can settle into a limit cycle. Giving it
   * the applied output keeps its estimate true to what moved the mechanism.
   *
   * @param measurement   Measured output.
   * @param setpoint      Reference.
   * @param appliedOutput Output the actuator applied since the last step, such as the motor
   *                      controller's measured voltage. A non-finite value is ignored.
   * @return Controller output.
   * @throws IllegalStateException if the controller has not been initialized.
   */
  public double calculate(double measurement, double setpoint, double appliedOutput) {
    if (!initialized) {
      throw new IllegalStateException("Controller must be initialized before use");
    }
    if (Double.isFinite(appliedOutput)) {
      uHistory[0] = appliedOutput;
      uPrev = appliedOutput;
    }
    return step(setpoint, measurement, null);
  }

  /**
   * Update the second ESO used by the cascaded form.
   *
   * @param u Delayed output.
   */
  private void updateLeso2(double u) {
    this.secondaryXhat = ad.mult(secondaryXhat).plus(bd.scale(u)).plus(ld.scale(xhat.get(0)));
  }

  /**
   * Linear controller law.
   *
   * @param refVec Reference vector.
   * @return Controller output.
   */
  private double computeControlLinear(SimpleMatrix refVec) {
    double v = refVec.get(systemOrder) - xhat.get(systemOrder);
    for (int i = 0; i < systemOrder; i++) {
      v += k.get(i) * (refVec.get(i) - xhat.get(i));
    }
    return v / b0;
  }

  /**
   * Cascaded controller law.
   *
   * @param refVec Reference vector.
   * @return Controller output.
   */
  private double computeControlCascaded(SimpleMatrix refVec) {
    double v = refVec.get(systemOrder) - secondaryXhat.get(systemOrder);
    for (int i = 0; i < systemOrder; i++) {
      v += k.get(i) * (refVec.get(i) - xhat.get(i));
    }
    return v / b0;
  }

  /** Compute the ESO and controller gains from the current bandwidths. */
  private void computeEsoGains() {
    int m = controllerOrder;

    SimpleMatrix a = new SimpleMatrix(m, m);
    for (int i = 0; i < systemOrder; i++) {
      a.set(i, i + 1, 1.0);
    }

    SimpleMatrix b = new SimpleMatrix(m, 1);
    b.set(systemOrder - 1, 0, b0);

    double z = Math.exp(-wo * dt);
    this.ld = new SimpleMatrix(m, 1, true, computeLdData(z, dt));

    this.phi = computeExpm(a, dt);
    this.gam = computeBd(a, b, dt);

    SimpleMatrix c = new SimpleMatrix(1, m);
    c.set(0, 0, 1.0);
    SimpleMatrix iMinusLc = SimpleMatrix.identity(m).minus(ld.mult(c));

    this.ad = iMinusLc.mult(phi);
    this.bd = iMinusLc.mult(gam);
  }

  /**
   * Matrix exponential by Taylor series.
   *
   * @param a  Matrix.
   * @param dt Time step.
   * @return {@code exp(a * dt)}.
   */
  private SimpleMatrix computeExpm(SimpleMatrix a, double dt) {
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
  private SimpleMatrix computeBd(SimpleMatrix a, SimpleMatrix b, double dt) {
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
   * Compute the observer gains for the current order.
   *
   * @param zEso Discrete observer pole, {@code exp(-wo * dt)}.
   * @param dt   Control period.
   * @return Observer gain vector data.
   */
  private double[] computeLdData(double zEso, double dt) {
    int m = controllerOrder;
    double[] res = new double[m];
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

        res[1] = 5.0 * (1.0 - zEso) * (1.0 - zEso) * (1.0 + zEso) * (5.0 + zEso * (2.0 + 5.0 * zEso)) / (12.0 * dt);

        res[2] = 5.0 * (1.0 - zEso) * (1.0 - zEso) * (1.0 - zEso) * (7.0 + zEso * (10.0 + 7.0 * zEso)) / (12.0 * dt * dt);

        res[3] = (1.0 - zEso) * (1.0 - zEso) * (1.0 - zEso) * (1.0 - zEso) * (1.0 + zEso) * 5.0 / (2.0 * dt * dt * dt);
        res[4] = (1.0 - zEso) * (1.0 - zEso) * (1.0 - zEso) * (1.0 - zEso) * (1.0 - zEso) / (dt * dt * dt * dt);
        break;
      default:
        throw new IllegalStateException("Unsupported system order: " + systemOrder);
    }
    return res;
  }

  /** Compute the controller gains from the current bandwidth. */
  private void computeControllerGains() {
    this.k = new SimpleMatrix(systemOrder, 1);
    double w = wc;
    for (int i = 1; i <= systemOrder; i++) {
      this.k.set(systemOrder - i, 0, binomial(systemOrder, i) * Math.pow(w, i));
    }
  }

  /**
   * Binomial coefficient.
   *
   * @param total  Total number of items.
   * @param choose Number of items chosen.
   * @return {@code total choose choose}.
   */
  private double binomial(int total, int choose) {
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
  private double factorial(int value) {
    double result = 1;
    for (int i = 2; i <= value; i++) {
      result *= i;
    }
    return result;
  }

  /**
   * Set or replace the tracking differentiator.
   *
   * @param method Differentiation method, or {@code "none"} to disable it.
   * @param params Method parameters. May be {@code null}.
   * @throws IllegalStateException    if the controller has not been initialized.
   * @throws IllegalArgumentException if the method or parameters are invalid.
   */
  public void setTd(String method, double[] params) {
    if (dt <= 0.0) {
      throw new IllegalStateException("Controller must be initialized before setting TD");
    }

    TrackingDifferentiator newTd = null;
    if (method != null && !"none".equalsIgnoreCase(method)) {
      newTd = new TrackingDifferentiator(method, this.dt, params);
      newTd.reset(tdSeed());
    }

    this.td = newTd;
    this.useTd = (newTd != null);
  }

  /**
   * Pick the value a new or reset tracking differentiator starts from.
   *
   * <p>During {@link #initialize} the old differentiator is stale, so the freshly supplied initial
   * state wins. At runtime an existing differentiator carries its output forward, which keeps a
   * live method change from looking like a step from zero.
   *
   * @return Old TD output when running, else the measured-state estimate, else zero.
   */
  private double tdSeed() {
    if (initialized && td != null) {
      return td.getY();
    }
    return xhat == null ? 0.0 : xhat.get(0);
  }

  /**
   * Reset the ESO state from a measured output.
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
    if (!initialized) {
      throw new IllegalStateException("Controller must be initialized before reset");
    }
    if (!Double.isFinite(measurement)) {
      throw new IllegalArgumentException("measurement must be finite");
    }
    double[] xhatInit = new double[controllerOrder];
    xhatInit[0] = measurement;
    reset(xhatInit, 0.0);
  }

  /**
   * Reset the ESO state and output history.
   *
   * <p>When the cascaded ESO is enabled, both observers start from the supplied state. The tracking
   * differentiator, when enabled, starts at {@code xhatInit[0]} (or zero when {@code xhatInit} is
   * {@code null}).
   *
   * @param xhatInit Initial ESO state, length {@code systemOrder + 1}. {@code null} starts at zero.
   * @param uInit    Initial output.
   * @throws IllegalStateException    if the controller has not been initialized.
   * @throws IllegalArgumentException if {@code xhatInit} has the wrong length or {@code uInit} is
   *                                  not finite.
   */
  public void reset(double[] xhatInit, double uInit) {
    if (!initialized) {
      throw new IllegalStateException("Controller must be initialized before reset");
    }
    if (xhatInit != null && xhatInit.length != controllerOrder) {
      throw new IllegalArgumentException("xhatInit length must equal controller order");
    }
    if (!Double.isFinite(uInit)) {
      throw new IllegalArgumentException("uInit must be finite");
    }
    double[] xInit = xhatInit == null ? new double[controllerOrder] : xhatInit.clone();
    this.xhat = new SimpleMatrix(controllerOrder, 1, true, xInit);
    this.secondaryXhat = new SimpleMatrix(controllerOrder, 1, true, useCascaded ? xInit : new double[controllerOrder]);
    this.uPrev = uInit;
    Arrays.fill(this.uHistory, this.uPrev);
    if (useTd && td != null) {
      td.reset(xInit[0]);
    }
  }

  /**
   * Change the output saturation limits.
   *
   * @param uMin Minimum output.
   * @param uMax Maximum output.
   * @throws IllegalArgumentException if a limit is NaN or {@code uMin > uMax}.
   */
  public void setSaturation(double uMin, double uMax) {
    if (Double.isNaN(uMin) || Double.isNaN(uMax) || uMin > uMax) {
      throw new IllegalArgumentException("uMin and uMax must not be NaN and uMin must be <= uMax");
    }
    this.uMin = uMin;
    this.uMax = uMax;
  }

  /**
   * Retune the controller bandwidths and recompute the gains.
   *
   * @param tSettle Desired closed-loop settling time, seconds. Must be finite and positive.
   * @param kob     Closed-loop bandwidth scale. Must be finite and positive.
   * @throws IllegalStateException    if the controller has not been initialized.
   * @throws IllegalArgumentException if a value is invalid.
   */
  public void updateTuning(double tSettle, double kob) {
    if (!initialized) {
      throw new IllegalStateException("Controller must be initialized before updating tuning");
    }
    if (!Double.isFinite(tSettle) || tSettle <= 0 || !Double.isFinite(kob) || kob <= 0) {
      throw new IllegalArgumentException("tSettle and kob must be finite and positive");
    }
    double newWc = 2.0 * (systemOrder + 1) * kob / tSettle;
    double newWo = OBSERVER_BANDWIDTH_RATIO * newWc;
    if (!Double.isFinite(newWc) || !Double.isFinite(newWo)) {
      throw new IllegalArgumentException("tuning produced non-finite bandwidths");
    }
    this.wc = newWc;
    this.wo = newWo;
    this.tSettle = tSettle;
    this.kob = kob;
    computeEsoGains();
    computeControllerGains();
  }

  /**
   * Replace the input gain, keeping the current state estimate and output history.
   *
   * <p>The ESO and the control law both use {@code b0}, so the gains are recomputed. The estimated
   * disturbance carries over, so a live-tuned input gain does not make the output jump.
   *
   * @param b0 New input gain, in the same units as the one the controller was initialized with.
   * @throws IllegalStateException    if the controller has not been initialized.
   * @throws IllegalArgumentException if the value is not finite or is zero.
   */
  public void setB0(double b0) {
    if (!initialized) {
      throw new IllegalStateException("Controller must be initialized before changing b0");
    }
    if (!Double.isFinite(b0) || b0 == 0.0) {
      throw new IllegalArgumentException("b0 must be finite and non-zero");
    }
    this.b0 = b0 / ke;
    computeEsoGains();
  }

  /**
   * Get the input gain the controller is using.
   *
   * @return Input gain, in the same units the controller was initialized with.
   * @throws IllegalStateException if the controller has not been initialized.
   */
  public double getB0() {
    if (!initialized) {
      throw new IllegalStateException("Controller must be initialized before reading b0");
    }
    return b0;
  }

  /**
   * Get the closed-loop bandwidth.
   *
   * @return Closed-loop bandwidth, radians per second.
   * @throws IllegalStateException if the controller has not been initialized.
   */
  public double getBandwidth() {
    if (!initialized) {
      throw new IllegalStateException("Controller must be initialized before reading the bandwidth");
    }
    return wc;
  }

  /**
   * Get the ESO observer bandwidth.
   *
   * @return Observer bandwidth, radians per second.
   * @throws IllegalStateException if the controller has not been initialized.
   */
  public double getObserverBandwidth() {
    if (!initialized) {
      throw new IllegalStateException("Controller must be initialized before reading the observer bandwidth");
    }
    return wo;
  }

  /**
   * Get the desired closed-loop settling time.
   *
   * @return Settling time, seconds.
   * @throws IllegalStateException if the controller has not been initialized.
   */
  public double getSettlingTime() {
    if (!initialized) {
      throw new IllegalStateException("Controller must be initialized before reading the settling time");
    }
    return tSettle;
  }

  /**
   * Get the closed-loop bandwidth scale.
   *
   * @return Bandwidth scale.
   * @throws IllegalStateException if the controller has not been initialized.
   */
  public double getBandwidthScale() {
    if (!initialized) {
      throw new IllegalStateException("Controller must be initialized before reading the bandwidth scale");
    }
    return kob;
  }

  /**
   * Get the minimum output.
   *
   * @return Minimum output.
   * @throws IllegalStateException if the controller has not been initialized.
   */
  public double getMinOutput() {
    if (!initialized) {
      throw new IllegalStateException("Controller must be initialized before reading the output limits");
    }
    return uMin;
  }

  /**
   * Get the maximum output.
   *
   * @return Maximum output.
   * @throws IllegalStateException if the controller has not been initialized.
   */
  public double getMaxOutput() {
    if (!initialized) {
      throw new IllegalStateException("Controller must be initialized before reading the output limits");
    }
    return uMax;
  }

  /**
   * Get the ESO state estimate.
   *
   * <p>Entry {@code 0} is the estimated measured output, entries {@code 1..systemOrder-1} are the
   * remaining plant states, and the final entry is the total disturbance estimate.
   *
   * @return Copy of the ESO state, length {@code systemOrder + 1}.
   * @throws IllegalStateException if the controller has not been initialized.
   */
  public double[] getEstimatedStates() {
    if (!initialized) {
      throw new IllegalStateException("Controller must be initialized before reading state estimates");
    }
    double[] out = new double[controllerOrder];
    System.arraycopy(xhat.getDDRM().getData(), 0, out, 0, controllerOrder);
    return out;
  }

  /**
   * Get the estimated total disturbance.
   *
   * <p>This is the final ESO state, in the units of the plant's highest derivative per second.
   * Divide by {@code b0} to view it as an equivalent control input.
   *
   * @return Estimated total disturbance.
   * @throws IllegalStateException if the controller has not been initialized.
   */
  public double getEstimatedDisturbance() {
    if (!initialized) {
      throw new IllegalStateException("Controller must be initialized before reading disturbance estimates");
    }
    return useCascaded ? secondaryXhat.get(systemOrder) : xhat.get(systemOrder);
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
   * Get the ESO order.
   *
   * @return {@code systemOrder + 1}.
   */
  public int getControllerOrder() {
    return controllerOrder;
  }

  /**
   * Whether {@link #initialize} has completed successfully.
   *
   * @return {@code true} if the controller is ready to use.
   */
  public boolean isInitialized() {
    return initialized;
  }

  /**
   * Saturate an output.
   *
   * @param u Output.
   * @return Output clamped to {@code [uMin, uMax]}.
   */
  private double saturate(double u) {
    return Math.min(Math.max(u, uMin), uMax);
  }
}
