// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <stdexcept>
#include <vector>
#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/math/kinematics/SwerveModuleVelocity.hpp>
#include <wpi/units/velocity.hpp>

namespace yams::mechanisms::swerve {

/**
 * Rigid-body swerve wheel-skid detector based on module translational-velocity consistency.
 *
 * For a rigid swerve chassis rotating at gyro-measured angular velocity omega, each module i at
 * robot-frame location (x_i, y_i) has rotational velocity v_rot_i = (-omega * y_i, omega * x_i).
 * Subtracting v_rot_i from the module's measured velocity isolates its translational velocity
 * v_trans_i. Without wheel slip, every module measures the same translational speed, so
 * skidRatio = max(|v_trans_i|) / min(|v_trans_i|) equals 1.0.
 */
class SkidDetector {
 public:
  /** Default skid ratio above which SkidState::skidding becomes true. */
  static constexpr double kDefaultSkidRatioThreshold = 1.35;

  /** Default minimum translational speed and speed-spread deadband. */
  static constexpr wpi::units::meters_per_second_t kDefaultMinTranslationalSpeed{0.10};

  /** Bitmask indicating that no modules are flagged as slipping. */
  static constexpr uint32_t kNoSlippingModules = 0U;

  /** Immutable snapshot of one skid evaluation. */
  struct SkidState {
    double skidRatio{kNominalRatio};
    bool skidding{false};
    double minTranslationalSpeedMps{0.0};
    double maxTranslationalSpeedMps{0.0};
    uint32_t slippingModuleMask{kNoSlippingModules};

    /** Nominal non-skidding state returned before first update or while stationary. */
    static constexpr SkidState Nominal() { return SkidState{}; }

    /** Check whether the given zero-based module index was flagged as slipping on this tick. */
    constexpr bool IsModuleSlipping(size_t moduleIndex) const {
      return moduleIndex < kMaxMaskBits && ((slippingModuleMask & (1U << moduleIndex)) != 0U);
    }
  };

  /**
   * Create a SkidDetector.
   *
   * @param skidRatioThreshold Ratio threshold strictly greater than 1.0 (e.g. 1.35).
   * @param minTranslationalSpeed Minimum translational speed and speed-spread deadband (>= 0)
   *     below which skidRatio stays at 1.0.
   */
  explicit SkidDetector(
      double skidRatioThreshold = kDefaultSkidRatioThreshold,
      wpi::units::meters_per_second_t minTranslationalSpeed = kDefaultMinTranslationalSpeed) {
    if (!std::isfinite(skidRatioThreshold) || skidRatioThreshold <= kNominalRatio) {
      throw std::invalid_argument("skidRatioThreshold must be finite and greater than 1.0.");
    }
    double minSpeedMps = minTranslationalSpeed.value();
    if (!std::isfinite(minSpeedMps) || minSpeedMps < 0.0) {
      throw std::invalid_argument("minTranslationalSpeed must be finite and non-negative.");
    }
    m_skidRatioThreshold = skidRatioThreshold;
    m_minTranslationalSpeedMps = minSpeedMps;
  }

  /** Get the configured skid ratio threshold (> 1.0). */
  double GetSkidRatioThreshold() const { return m_skidRatioThreshold; }

  /** Get the configured minimum translational speed deadband in m/s. */
  double GetMinTranslationalSpeedMps() const { return m_minTranslationalSpeedMps; }

  /**
   * Evaluate wheel skid from measured module states, module robot-frame locations, and
   * gyro-measured chassis angular velocity.
   *
   * @param moduleStates Measured SwerveModuleVelocity for each module.
   * @param moduleLocations Robot-frame Translation2d location for each module (same length).
   * @param omegaRadPerSec Gyro-measured chassis angular velocity about +Z in rad/s (CCW positive).
   * @return SkidState for this tick.
   */
  SkidState Calculate(std::span<const wpi::math::SwerveModuleVelocity> moduleStates,
                      std::span<const wpi::math::Translation2d> moduleLocations,
                      double omegaRadPerSec) const {
    if (moduleStates.size() != moduleLocations.size()) {
      throw std::invalid_argument("moduleStates and moduleLocations must have the same length.");
    }
    const size_t n = moduleStates.size();
    if (n < kMinModulesForSkid || !std::isfinite(omegaRadPerSec)) {
      return SkidState::Nominal();
    }

    std::vector<double> transSpeeds(n);
    double minTrans = std::numeric_limits<double>::infinity();
    double maxTrans = 0.0;

    for (size_t i = 0; i < n; ++i) {
      const auto& state = moduleStates[i];
      const auto& loc = moduleLocations[i];
      const double angleRad = state.angle.Radians().value();
      const double speedMps = state.velocity.value();
      const double measVx = speedMps * std::cos(angleRad);
      const double measVy = speedMps * std::sin(angleRad);

      // Subtract pure rigid-body rotational velocity: omega x r = (-omega * y, omega * x).
      const double transVx = measVx - (-omegaRadPerSec * loc.Y().value());
      const double transVy = measVy - (omegaRadPerSec * loc.X().value());
      const double transSpeed = std::hypot(transVx, transVy);
      transSpeeds[i] = transSpeed;

      if (transSpeed < minTrans) minTrans = transSpeed;
      if (transSpeed > maxTrans) maxTrans = transSpeed;
    }

    // Guard against division by near-zero translational speeds and low-speed encoder noise.
    if (maxTrans < m_minTranslationalSpeedMps ||
        (maxTrans - minTrans) < m_minTranslationalSpeedMps) {
      return SkidState{kNominalRatio, false, minTrans, maxTrans, kNoSlippingModules};
    }

    const double denom = std::max(minTrans, kMinDenominatorMps);
    const double ratio = maxTrans / denom;
    const bool skidding = ratio > m_skidRatioThreshold;
    const uint32_t slippingMask = skidding ? ComputeSlippingMask(transSpeeds) : kNoSlippingModules;
    return SkidState{ratio, skidding, minTrans, maxTrans, slippingMask};
  }

 private:
  static constexpr double kNominalRatio = 1.0;
  static constexpr double kMinDenominatorMps = 1e-4;
  static constexpr size_t kMinModulesForSkid = 2;
  static constexpr size_t kMaxMaskBits = 32;

  uint32_t ComputeSlippingMask(const std::vector<double>& transSpeeds) const {
    const size_t n = transSpeeds.size();
    std::vector<double> sorted = transSpeeds;
    std::sort(sorted.begin(), sorted.end());
    const double median =
        (n % 2 == 1) ? sorted[n / 2] : 0.5 * (sorted[(n / 2) - 1] + sorted[n / 2]);

    uint32_t mask = kNoSlippingModules;
    const size_t bitLimit = std::min(n, kMaxMaskBits);
    for (size_t i = 0; i < bitLimit; ++i) {
      const double speed = transSpeeds[i];
      const double hi = std::max(speed, median);
      const double lo = std::max(std::min(speed, median), kMinDenominatorMps);
      if ((hi / lo) > m_skidRatioThreshold &&
          std::abs(speed - median) >= m_minTranslationalSpeedMps) {
        mask |= (1U << i);
      }
    }
    if (mask == kNoSlippingModules) {
      for (size_t i = 0; i < bitLimit; ++i) {
        mask |= (1U << i);
      }
    }
    return mask;
  }

  double m_skidRatioThreshold{kDefaultSkidRatioThreshold};
  double m_minTranslationalSpeedMps{kDefaultMinTranslationalSpeed.value()};
};

}  // namespace yams::mechanisms::swerve
