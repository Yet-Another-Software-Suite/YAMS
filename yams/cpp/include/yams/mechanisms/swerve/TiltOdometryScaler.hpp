// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <span>
#include <stdexcept>
#include <vector>
#include <wpi/math/geometry/Rotation3d.hpp>
#include <wpi/math/kinematics/SwerveModulePosition.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/length.hpp>
#include <wpi/util/array.hpp>

#include "yams/mechanisms/swerve/SkidDetector.hpp"

namespace yams::mechanisms::swerve {

/**
 * Scales incremental swerve wheel-odometry distance deltas by the robot's 3D tilt angle (and
 * optionally drops individual slipping modules) so driving over field bumps, ramps, or low-traction
 * patches does not corrupt 2D pose estimation with inflated wheel travel.
 *
 * Uses the exact yaw-invariant 3D tilt angle from vertical: tilt = acos(cos(pitch) * cos(roll)).
 */
class TiltOdometryScaler {
 public:
  /** Default tilt angle below which wheel odometry is trusted at 100% (1.0). */
  static constexpr wpi::units::degree_t kDefaultFullTrustTilt{2.0};

  /** Default tilt angle at or above which wheel odometry deltas are attenuated to 0.0. */
  static constexpr wpi::units::degree_t kDefaultZeroTrustTilt{25.0};

  /** How wheel-odometry distance deltas are attenuated as the robot tilts away from horizontal. */
  enum class ScalingMode {
    /**
     * Heuristic linear trust ramp from 1.0 at fullTrustTilt down to 0.0 at zeroTrustTilt.
     * Aggressively suppresses noisy wheel deltas on steep bumps or ramps where wheel contact is
     * unreliable.
     */
    LINEAR_RAMP,
    /**
     * Geometric horizontal projection cos(tilt) above fullTrustTilt, dropping to 0.0 at or above
     * zeroTrustTilt. Matches horizontal ground travel on a rigid slope without wheel slip.
     */
    COSINE_PROJECTION
  };

  /**
   * Immutable configuration for constructing a stateful TiltOdometryScaler per SwerveDrive
   * instance without sharing mutable wheel-distance history through SwerveDriveConfig.
   */
  struct Config {
    wpi::units::radian_t fullTrustTilt{kDefaultFullTrustTilt};
    wpi::units::radian_t zeroTrustTilt{kDefaultZeroTrustTilt};
    ScalingMode scalingMode{ScalingMode::LINEAR_RAMP};

    Config(wpi::units::radian_t fullTrust = kDefaultFullTrustTilt,
           wpi::units::radian_t zeroTrust = kDefaultZeroTrustTilt,
           ScalingMode mode = ScalingMode::LINEAR_RAMP)
        : fullTrustTilt{fullTrust}, zeroTrustTilt{zeroTrust}, scalingMode{mode} {
      const double fullRad = fullTrustTilt.value();
      const double zeroRad = zeroTrustTilt.value();
      if (!std::isfinite(fullRad) || !std::isfinite(zeroRad) || fullRad < 0.0 ||
          zeroRad <= fullRad || zeroRad > kMaxTiltRad) {
        throw std::invalid_argument(
            "Require 0 <= fullTrustTilt < zeroTrustTilt <= 90 deg.");
      }
    }
  };

  /** Create a TiltOdometryScaler with default thresholds (2 deg to 25 deg, LINEAR_RAMP). */
  TiltOdometryScaler() : TiltOdometryScaler(Config{}) {}

  /** Create a TiltOdometryScaler with explicit thresholds and optional ScalingMode. */
  TiltOdometryScaler(wpi::units::radian_t fullTrustTilt, wpi::units::radian_t zeroTrustTilt,
                     ScalingMode scalingMode = ScalingMode::LINEAR_RAMP)
      : TiltOdometryScaler(Config{fullTrustTilt, zeroTrustTilt, scalingMode}) {}

  /** Create a TiltOdometryScaler from an immutable Config. */
  explicit TiltOdometryScaler(const Config& config)
      : m_fullTrustTiltRad{config.fullTrustTilt.value()},
        m_zeroTrustTiltRad{config.zeroTrustTilt.value()},
        m_scalingMode{config.scalingMode},
        m_tiltScalingEnabled{true} {}

  /**
   * Create a passthrough TiltOdometryScaler that leaves wheel deltas unscaled by tilt (1.0) while
   * still tracking raw wheel-distance deltas for per-module or full-chassis skid suppression.
   */
  static TiltOdometryScaler Passthrough() { return TiltOdometryScaler(false); }

  /** Compute the exact yaw-invariant 3D tilt angle from vertical for the given robot attitude. */
  static wpi::units::radian_t ComputeTiltAngle(const wpi::math::Rotation3d& attitude) {
    double cosTilt = std::cos(attitude.Y().value()) * std::cos(attitude.X().value());
    cosTilt = std::clamp(cosTilt, -kFullScale, kFullScale);
    return wpi::units::radian_t{std::acos(cosTilt)};
  }

  /** Check whether the robot's 3D tilt angle strictly exceeds maxTilt. */
  static bool IsTiltExceeding(const wpi::math::Rotation3d& attitude,
                              wpi::units::radian_t maxTilt) {
    return ComputeTiltAngle(attitude).value() > maxTilt.value();
  }

  /** Get the configured ScalingMode. */
  ScalingMode GetScalingMode() const { return m_scalingMode; }

  /** Compute the tilt odometry scale factor in [0.0, 1.0] without mutating state. */
  double ComputeScale(const wpi::math::Rotation3d& attitude) const {
    if (!m_tiltScalingEnabled) {
      return kFullScale;
    }
    const double tiltRad = ComputeTiltAngle(attitude).value();
    if (tiltRad <= m_fullTrustTiltRad) {
      return kFullScale;
    }
    if (tiltRad >= m_zeroTrustTiltRad) {
      return kZeroScale;
    }
    switch (m_scalingMode) {
      case ScalingMode::LINEAR_RAMP:
        return kFullScale -
               (tiltRad - m_fullTrustTiltRad) / (m_zeroTrustTiltRad - m_fullTrustTiltRad);
      case ScalingMode::COSINE_PROJECTION:
        return std::max(kZeroScale, std::cos(tiltRad));
    }
    return kFullScale;
  }

  /** Reset internal wheel-distance history to match currentRawPositions. */
  void Reset(std::span<const wpi::math::SwerveModulePosition> currentRawPositions) {
    const size_t n = currentRawPositions.size();
    m_lastRawDistancesMeters.resize(n);
    m_scaledDistancesMeters.resize(n);
    for (size_t i = 0; i < n; ++i) {
      const double d = currentRawPositions[i].distance.value();
      m_lastRawDistancesMeters[i] = d;
      m_scaledDistancesMeters[i] = d;
    }
    m_lastScale = kFullScale;
    m_lastTiltRad = 0.0;
  }

  /**
   * Apply tilt scaling to incremental wheel-distance deltas while freezing only modules flagged in
   * slippingModuleMask.
   */
  std::vector<wpi::math::SwerveModulePosition> ScalePositions(
      const wpi::math::Rotation3d& attitude,
      std::span<const wpi::math::SwerveModulePosition> rawPositions,
      uint32_t slippingModuleMask) {
    const size_t n = rawPositions.size();
    if (m_lastRawDistancesMeters.size() != n) {
      Reset(rawPositions);
    }

    m_lastTiltRad = ComputeTiltAngle(attitude).value();
    const double tiltScale = ComputeScale(attitude);

    std::vector<wpi::math::SwerveModulePosition> out;
    out.reserve(n);
    double scaleSum = 0.0;
    for (size_t i = 0; i < n; ++i) {
      const bool moduleSlipping = i < kMaxMaskBits && ((slippingModuleMask & (1U << i)) != 0U);
      const double moduleScale = moduleSlipping ? kZeroScale : tiltScale;
      scaleSum += moduleScale;

      const double rawDist = rawPositions[i].distance.value();
      const double delta = rawDist - m_lastRawDistancesMeters[i];
      m_lastRawDistancesMeters[i] = rawDist;
      m_scaledDistancesMeters[i] += delta * moduleScale;
      out.push_back(wpi::math::SwerveModulePosition{
          wpi::units::meter_t{m_scaledDistancesMeters[i]}, rawPositions[i].angle});
    }
    m_lastScale = (n > 0) ? (scaleSum / static_cast<double>(n)) : tiltScale;
    return out;
  }

  /** Overload accepting a boolean rejectUpdate flag (freezes all modules when true). */
  std::vector<wpi::math::SwerveModulePosition> ScalePositions(
      const wpi::math::Rotation3d& attitude,
      std::span<const wpi::math::SwerveModulePosition> rawPositions,
      bool rejectUpdate) {
    uint32_t mask = SkidDetector::kNoSlippingModules;
    if (rejectUpdate) {
      const size_t bitLimit = std::min(rawPositions.size(), kMaxMaskBits);
      for (size_t i = 0; i < bitLimit; ++i) {
        mask |= (1U << i);
      }
    }
    return ScalePositions(attitude, rawPositions, mask);
  }

  /** Fixed-size array overload used directly by SwerveDrive<NumModules>. */
  template <size_t NumModules>
  wpi::util::array<wpi::math::SwerveModulePosition, NumModules> ScalePositions(
      const wpi::math::Rotation3d& attitude,
      const wpi::util::array<wpi::math::SwerveModulePosition, NumModules>& rawPositions,
      uint32_t slippingModuleMask) {
    auto scaledVec = ScalePositions(
        attitude, std::span<const wpi::math::SwerveModulePosition>{rawPositions},
        slippingModuleMask);
    wpi::util::array<wpi::math::SwerveModulePosition, NumModules> out{wpi::util::empty_array};
    for (size_t i = 0; i < NumModules; ++i) {
      out[i] = scaledVec[i];
    }
    return out;
  }

  /** Get the most recently applied average effective scale factor across all modules in [0, 1]. */
  double GetLastScale() const { return m_lastScale; }

  /** Get the most recently computed 3D tilt angle. */
  wpi::units::radian_t GetLastTiltAngle() const { return wpi::units::radian_t{m_lastTiltRad}; }

 private:
  static constexpr double kFullScale = 1.0;
  static constexpr double kZeroScale = 0.0;
  static constexpr double kMaxTiltRad = std::numbers::pi * 0.5;
  static constexpr size_t kMaxMaskBits = 32;

  explicit TiltOdometryScaler(bool tiltScalingEnabled)
      : m_fullTrustTiltRad{0.0},
        m_zeroTrustTiltRad{kMaxTiltRad},
        m_scalingMode{ScalingMode::LINEAR_RAMP},
        m_tiltScalingEnabled{tiltScalingEnabled} {}

  double m_fullTrustTiltRad{0.0};
  double m_zeroTrustTiltRad{kMaxTiltRad};
  ScalingMode m_scalingMode{ScalingMode::LINEAR_RAMP};
  bool m_tiltScalingEnabled{true};
  std::vector<double> m_lastRawDistancesMeters;
  std::vector<double> m_scaledDistancesMeters;
  double m_lastScale{kFullScale};
  double m_lastTiltRad{0.0};
};

}  // namespace yams::mechanisms::swerve
