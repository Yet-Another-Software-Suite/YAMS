// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <units/angle.h>

#include <optional>

#include "EasyCRTConfig.hpp"

namespace yams::units {

/**
 * CRT-inspired absolute mechanism angle estimator using two absolute encoders.
 *
 * Generates mechanism angle candidates consistent with encoder 1, predicts what encoder 2
 * should read for each candidate, and selects the unique best match within a configurable
 * tolerance.  Created by team 6911.
 *
 * This is NOT a textbook Chinese Remainder Theorem solve; it is a CRT-inspired unwrapping
 * method designed to remain stable under backlash and sensor noise.
 */
class EasyCRT {
 public:
  /** Result status of the most recent solve attempt. */
  enum class CRTStatus {
    OK,              ///< Solve succeeded with a unique solution.
    NO_SOLUTION,     ///< No candidate fell within the match tolerance.
    AMBIGUOUS,       ///< Two nearly-equal candidates both fell within tolerance.
    NOT_ATTEMPTED,   ///< No solve has been attempted yet.
    INVALID_CONFIG,  ///< Solve was skipped due to an invalid configuration.
  };

  /**
   * Create an EasyCRT solver.
   *
   * @param config Configuration describing encoder ratios, offsets, and mechanism range.
   */
  explicit EasyCRT(EasyCRTConfig config);

  /**
   * Return the mechanism angle if a unique solution is found.
   *
   * If no unique solution is found (outside tolerance or ambiguous), returns empty.
   *
   * @return Optional mechanism angle in turns when uniquely resolved.
   */
  std::optional<::units::angle::turn_t> GetAngleOptional();

  /**
   * Return the status of the most recent solve attempt.
   *
   * @return CRTStatus from the previous call to GetAngleOptional().
   */
  CRTStatus GetLastStatus() const;

  /**
   * Return the best-match modular error (in rotations) from the last solve.
   *
   * @return Modular error, or NaN if not yet solved.
   */
  double GetLastErrorRotations() const;

  /**
   * Return the number of candidates evaluated in the last solve attempt.
   *
   * @return Iteration count from the previous solve.
   */
  int GetLastIterations() const;

 private:
  struct CrtSolution {
    double mechanismRotations;
    double errorRotations;
  };

  EasyCRTConfig m_config;
  CRTStatus m_lastStatus{CRTStatus::NOT_ATTEMPTED};
  double m_lastErrorRot{std::numeric_limits<double>::quiet_NaN()};
  int m_lastIterations{0};

  std::optional<CrtSolution> ResolveFromSensors(double abs1, double abs2, double ratio1,
                                                double ratio2, double minMechRot, double maxMechRot,
                                                double matchTolerance);

  static double ModularError(double a, double b);
};

}  // namespace yams::units
