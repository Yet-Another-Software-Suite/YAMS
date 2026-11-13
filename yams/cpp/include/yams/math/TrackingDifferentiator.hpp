// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <vector>

#include <wpi/units/time.hpp>

namespace yams::math {

/**
 * Tracking differentiator.
 *
 * <b>Written by FRC Team 8331</b>, who contributed active disturbance rejection control to YAMS.
 *
 * A tracking differentiator turns a raw reference into a smooth reference y together with its first and second
 * derivatives yd and ydd. Feeding the smoothed reference to a controller avoids the large output spikes a step setpoint
 * would otherwise produce: the controller follows a trajectory the mechanism can track instead of demanding an
 * instantaneous change.
 *
 * Mechanisms rarely construct this class directly; they configure one with
 * ADRCConfig::WithTrackingDifferentiator and the controller applies it to every setpoint.
 *
 * Five methods are supported, selected with Method. Each takes a short list of parameters; a parameter left out uses
 * its default:
 *
 * - EULER, first order smoothing: alpha in [0, 1], default 0.9. Higher is smoother and slower; the time constant is
 *   about dt / (1 - alpha), and yd jumps on a step.
 * - TOD, bang-bang: r greater than 0, default 1, the acceleration bound in the loop's units per second squared. yd
 *   chatters at rest at a 20 ms period.
 * - LD, linear: lambda greater than 0, default 1, a time constant in seconds; the sample time must stay below
 *   2 * lambda. No overshoot; a step is followed to 98% in about 5.8 * lambda. The best default.
 * - RED, super-twisting: lambda1 (how hard the error pulls) and lambda2 (the bound on how quickly yd changes), greater
 *   than 0, default 1 each. Overshoots, and yd chatters at rest at a 20 ms period.
 * - INTD, saturating: alpha in [0, 1) default 0.5, then beta and gamma in the loop's units and r per second, greater
 *   than 0, default 1 each. Cruises at r * beta / (1 - alpha), then lands like LD with natural frequency
 *   r * sqrt(beta / gamma) and damping 0.5 * sqrt(gamma / beta); gamma = 4 * beta is critically damped.
 *
 * ADRCGuide.md explains how to choose and tune each method.
 *
 * Invalid parameters throw std::invalid_argument.
 *
 * @code
 * yams::math::TrackingDifferentiator td{yams::math::TrackingDifferentiator::Method::LD, 20_ms, {0.08}};
 * td.Update(setpoint);
 * double smoothSetpoint = td.GetY();
 * double setpointVelocity = td.GetYd();
 * @endcode
 */
class TrackingDifferentiator {
 public:
  /** Differentiation method. */
  enum class Method {
    EULER,  ///< First order exponential smoothing; alpha in [0, 1], default 0.9.
    TOD,    ///< Bang-bang time optimal differentiator; r greater than 0, default 1.
    LD,     ///< Linear differentiator; lambda greater than 0, default 1.
    RED,    ///< Super-twisting differentiator; lambda1, lambda2, default 1 each.
    INTD,   ///< Saturating differentiator; alpha, beta, gamma, r, defaults 0.5, 1, 1, 1.
  };

  /**
   * Create a tracking differentiator.
   *
   * @param method Differentiation method.
   * @param dt     Sample time.
   * @param params Method parameters in order; empty, or shorter than the method takes, for the defaults.
   * @throws std::invalid_argument if the sample time or a parameter is invalid.
   */
  TrackingDifferentiator(Method method, wpi::units::second_t dt, std::vector<double> params = {});

  /**
   * Replace the method parameters. An empty list leaves them unchanged.
   *
   * @param params New parameters.
   * @throws std::invalid_argument if a parameter is invalid; the previous parameters are kept.
   */
  void SetParameters(std::vector<double> params);

  /**
   * Change the sample time.
   *
   * @param dt Sample time.
   * @throws std::invalid_argument if the sample time is invalid, or the parameters are at it.
   */
  void SetSampleTime(wpi::units::second_t dt);

  /**
   * Advance the differentiator by one sample.
   *
   * @param reference Raw reference.
   */
  void Update(double reference);

  /**
   * Reset the internal states to a value, at rest.
   *
   * @param initialValue Initial value of the filtered reference.
   */
  void Reset(double initialValue);

  /** @return The filtered reference. */
  double GetY() const { return m_y; }
  /** @return The filtered reference's derivative. */
  double GetYd() const { return m_yd; }
  /** @return The filtered reference's second derivative. */
  double GetYdd() const { return m_ydd; }
  /** @return The differentiation method. */
  Method GetMethod() const { return m_method; }

 private:
  /** A parameter, or its default when it was not given. */
  double Param(size_t index, double defaultValue) const;
  /** Check the parameters at a sample time; throws std::invalid_argument. */
  void ValidateParams(double dt) const;

  void UpdateEuler(double reference);
  void UpdateTod(double reference);
  void UpdateLd(double reference);
  void UpdateRed(double reference);
  void UpdateIntd(double reference);

  Method m_method;
  double m_dt;
  std::vector<double> m_params;

  double m_prevY{0.0};
  double m_prevYd{0.0};
  double m_v1{0.0};
  double m_v2{0.0};
  double m_y{0.0};
  double m_yd{0.0};
  double m_ydd{0.0};
};

}  // namespace yams::math
