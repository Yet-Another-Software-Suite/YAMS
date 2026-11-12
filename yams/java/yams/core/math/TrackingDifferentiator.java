// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.math;

import java.util.Arrays;
import java.util.Locale;

/**
 * Tracking differentiator (TD).
 *
 * <p><b>Written by FRC Team 8331</b>, who contributed ADRC to YAMS.
 *
 * <p>A tracking differentiator turns a raw reference into a smooth reference {@code y} together
 * with its first and second derivatives {@code yd} and {@code ydd}. Feeding the smoothed reference
 * to a controller avoids the large derivative spikes that a step input would otherwise produce.
 *
 * <p>Several discrete implementations are supported:
 *
 * <ul>
 * <li>{@code euler}: first-order exponential smoothing.
 * <li>{@code tod}: time-optimal discrete differentiator.
 * <li>{@code ld}: linear differentiator.
 * <li>{@code red}: nonlinear (reduced) differentiator.
 * <li>{@code intd}: integral differentiator.
 * </ul>
 *
 * <p>The method and its parameters are passed by name so existing ADRC tuning strings can be used
 * directly. Unknown methods and invalid parameters throw {@link IllegalArgumentException}.
 *
 * <h2>Example</h2>
 *
 * <pre>{@code
 * TrackingDifferentiator td = new TrackingDifferentiator("euler", 0.02, new double[] {0.9});
 * td.update(setpoint);
 * double smoothSetpoint = td.getY();
 * double setpointVelocity = td.getYd();
 * }</pre>
 */
public class TrackingDifferentiator {
  /** Identifier of the differentiation method. */
  private final String method;
  /** Sample time in seconds. */
  private double       dt;
  /** Method parameters. */
  private double[]     params;

  /** Previous filtered value, used by the Euler method. */
  private double       prevY;
  /** Previous filtered derivative, used by the Euler method. */
  private double       prevYd;

  /** First state of the continuous methods. */
  private double       v1;
  /** Second state of the continuous methods. */
  private double       v2;

  /** Filtered reference. */
  private double       y   = 0.0;
  /** Filtered reference derivative. */
  private double       yd  = 0.0;
  /** Filtered reference second derivative. */
  private double       ydd = 0.0;

  /**
   * Create a tracking differentiator.
   *
   * @param method Differentiation method: {@code euler}, {@code tod}, {@code ld}, {@code red}, or
   *               {@code intd}.
   * @param dt     Sample time in seconds.
   * @param params Method parameters. May be {@code null} to use the method defaults.
   * @throws IllegalArgumentException if the method or sample time is invalid, or if the parameters
   *                                  do not validate.
   */
  public TrackingDifferentiator(String method, double dt, double[] params) {
    if (method == null || method.isBlank()) {
      throw new IllegalArgumentException("TD method must not be null or blank");
    }
    if (!Double.isFinite(dt) || dt <= 0.0) {
      throw new IllegalArgumentException("Sample time dt must be finite and positive");
    }

    this.method = method.toLowerCase(Locale.ROOT);
    this.dt = dt;
    this.params = params != null ? params.clone() : null;

    validateParams();
    initializeState(0.0);
  }

  /**
   * Replace the method parameters.
   *
   * @param newParams New parameters. {@code null} or an empty array leaves the current parameters
   *                  unchanged.
   * @throws IllegalArgumentException if the new parameters do not validate. The previous parameters
   *                                  are restored before the exception is rethrown.
   */
  public void setParameters(double[] newParams) {
    if (newParams == null || newParams.length == 0) {
      return;
    }
    double[] oldParams = this.params;
    this.params = Arrays.copyOf(newParams, newParams.length);
    try {
      validateParams();
    } catch (RuntimeException e) {
      this.params = oldParams;
      throw e;
    }
  }

  /**
   * Read a parameter, falling back to the method default.
   *
   * @param index        Parameter index.
   * @param defaultValue Value used when the parameter was not supplied.
   * @return Parameter value.
   */
  private double param(int index, double defaultValue) {
    return params != null && params.length > index ? params[index] : defaultValue;
  }

  /** Validate the parameters at the current sample time. */
  private void validateParams() {
    validateParams(this.dt);
  }

  /**
   * Validate the parameters for a sample time.
   *
   * @param dt Sample time in seconds.
   * @throws IllegalArgumentException if a parameter is invalid for the configured method.
   */
  private void validateParams(double dt) {
    switch (method) {
      case "euler": {
        double alpha = param(0, 0.9);
        if (!Double.isFinite(alpha) || alpha < 0.0 || alpha > 1.0) {
          throw new IllegalArgumentException("alpha must be in [0, 1] for EULER");
        }
        break;
      }
      case "tod": {
        double r = param(0, 1.0);
        if (!Double.isFinite(r) || r <= 0.0) {
          throw new IllegalArgumentException("r must be positive for TOD");
        }
        break;
      }
      case "ld": {
        double lambda = param(0, 1.0);
        if (!Double.isFinite(lambda) || lambda <= 0.0) {
          throw new IllegalArgumentException("lambda must be positive for LD");
        }
        if (dt >= 2.0 * lambda) {
          throw new IllegalArgumentException("ld unstable at this dt: need dt < 2 * lambda");
        }
        break;
      }
      case "red": {
        double lambda1 = param(0, 1.0);
        double lambda2 = param(1, 1.0);
        if (!Double.isFinite(lambda1) || !Double.isFinite(lambda2) || lambda1 <= 0.0 || lambda2 <= 0.0) {
          throw new IllegalArgumentException("lambda1 and lambda2 must be positive for RED");
        }
        break;
      }
      case "intd": {
        double alpha = param(0, 0.5);
        double beta = param(1, 1.0);
        double gamma = param(2, 1.0);
        double r = param(3, 1.0);
        if (!Double.isFinite(alpha) || alpha < 0.0 || alpha >= 1.0) {
          throw new IllegalArgumentException("alpha must be in [0, 1) for INTD");
        }
        if (!Double.isFinite(beta) || !Double.isFinite(gamma) || !Double.isFinite(r) || beta <= 0.0 || gamma <= 0.0 || r <= 0.0) {
          throw new IllegalArgumentException("beta, gamma, r must be positive for INTD");
        }
        break;
      }
      default:
        throw new IllegalArgumentException("Unknown TD method: " + method);
    }
  }

  /**
   * Reset the internal states.
   *
   * @param initialValue Initial value of the filtered reference.
   */
  private void initializeState(double initialValue) {
    switch (method) {
      case "euler":
      case "tod":
      case "ld":
      case "red":
      case "intd":
        break;
      default:
        throw new IllegalArgumentException("Unknown TD method: " + method);
    }

    prevY = initialValue;
    prevYd = 0.0;
    v1 = initialValue;
    v2 = 0.0;
    y = initialValue;
    yd = 0.0;
    ydd = 0.0;
  }

  /**
   * Advance the differentiator by one sample.
   *
   * @param ref Raw reference.
   */
  public void update(double ref) {
    switch (method) {
      case "euler":
        updateEuler(ref);
        break;
      case "tod":
        updateTod(ref);
        break;
      case "ld":
        updateLd(ref);
        break;
      case "red":
        updateRed(ref);
        break;
      case "intd":
        updateIntd(ref);
        break;
      default:
        throw new IllegalArgumentException("Unknown TD method: " + method);
    }
  }

  /**
   * Reset the internal states to an initial value.
   *
   * @param initialValue Initial value of the filtered reference.
   */
  public void reset(double initialValue) {
    initializeState(initialValue);
  }

  /**
   * Change the sample time.
   *
   * @param dt Sample time in seconds.
   * @throws IllegalArgumentException if the sample time or the current parameters are invalid at
   *                                  the new sample time.
   */
  public void setSampleTime(double dt) {
    if (!Double.isFinite(dt) || dt <= 0.0) {
      throw new IllegalArgumentException("Sample time dt must be finite and positive");
    }
    validateParams(dt);
    this.dt = dt;
  }

  /**
   * Euler first-order smoothing.
   *
   * @param ref Raw reference.
   */
  private void updateEuler(double ref) {
    double alpha = param(0, 0.9);

    y = alpha * prevY + (1.0 - alpha) * ref;
    yd = (y - prevY) / dt;
    ydd = (yd - prevYd) / dt;

    prevY = y;
    prevYd = yd;
  }

  /**
   * Time-optimal discrete differentiator.
   *
   * @param ref Raw reference.
   */
  private void updateTod(double ref) {
    double r = param(0, 1.0);

    double diff = v1 - ref + (v2 * Math.abs(v2)) / (2.0 * r);
    double dotV1 = v2;
    double dotV2 = -r * Math.signum(diff);

    v1 += dotV1 * dt;
    v2 += dotV2 * dt;

    y = v1;
    yd = v2;
    ydd = dotV2;
  }

  /**
   * Linear differentiator.
   *
   * @param ref Raw reference.
   */
  private void updateLd(double ref) {
    double lambda = param(0, 1.0);
    double r = 1.0 / lambda;

    double dotV1 = v2;
    double dotV2 = -r * r * (v1 - ref) - 2.0 * r * v2;

    v1 += dotV1 * dt;
    v2 += dotV2 * dt;

    y = v1;
    yd = v2;
    ydd = dotV2;
  }

  /**
   * Nonlinear reduced differentiator.
   *
   * @param ref Raw reference.
   */
  private void updateRed(double ref) {
    double lambda1 = param(0, 1.0);
    double lambda2 = param(1, 1.0);

    double err = v1 - ref;
    double absErr = Math.abs(err);

    double dotV1 = v2 - lambda1 * Math.sqrt(absErr) * ((err > 0) ? 1.0 : -1.0);
    double dotV2 = -lambda2 * Math.signum(err);

    v1 += dotV1 * dt;
    v2 += dotV2 * dt;

    y = v1;
    yd = v2;
    ydd = dotV2;
  }

  /**
   * Integral differentiator.
   *
   * @param ref Raw reference.
   */
  private void updateIntd(double ref) {
    double alpha = param(0, 0.5);
    double beta = param(1, 1.0);
    double gamma = param(2, 1.0);
    double r = param(3, 1.0);

    double scale = beta / (1.0 - alpha);
    double dotV1 = v2;
    double dotV2 = -r * r * scale * Math.tanh((1.0 - alpha) * (v1 - ref) / gamma) - r * v2;

    v1 += dotV1 * dt;
    v2 += dotV2 * dt;

    y = v1;
    yd = v2;
    ydd = dotV2;
  }

  /**
   * Get the filtered reference.
   *
   * @return Filtered reference.
   */
  public double getY() {
    return y;
  }

  /**
   * Get the filtered reference derivative.
   *
   * @return Filtered reference derivative.
   */
  public double getYd() {
    return yd;
  }

  /**
   * Get the filtered reference second derivative.
   *
   * @return Filtered reference second derivative.
   */
  public double getYdd() {
    return ydd;
  }

  /**
   * Get the differentiation method name.
   *
   * @return Lower-case method name.
   */
  public String getMethod() {
    return method;
  }
}
