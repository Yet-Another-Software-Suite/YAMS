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
 * - EULER, first order smoothing: alpha in [0, 1], default 0.9. Higher is smoother and slower.
 * - TOD, time optimal: r greater than 0, default 1. The acceleration bound; higher tracks faster.
 * - LD, linear: lambda greater than 0, default 1. The sample time must stay below 2 * lambda.
 * - RED, nonlinear reduced: lambda1 and lambda2 greater than 0, default 1 each.
 * - INTD, integral: alpha in [0, 1) default 0.5, then beta, gamma and r greater than 0, default 1 each.
 *
 * Invalid parameters throw std::invalid_argument.
 *
 * @code
 * yams::math::TrackingDifferentiator td{yams::math::TrackingDifferentiator::Method::EULER, 20_ms, {0.9}};
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
    TOD,    ///< Time optimal discrete differentiator; r greater than 0, default 1.
    LD,     ///< Linear differentiator; lambda greater than 0, default 1.
    RED,    ///< Nonlinear reduced differentiator; lambda1, lambda2, default 1 each.
    INTD,   ///< Integral differentiator; alpha, beta, gamma, r, defaults 0.5, 1, 1, 1.
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
