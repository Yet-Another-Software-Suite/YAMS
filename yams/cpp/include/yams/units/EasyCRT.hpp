// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <units/angle.h>

#include <cmath>
#include <functional>
#include <limits>
#include <optional>

/**
 * @file EasyCRT.hpp
 * @brief CRT-based absolute angle solver for two-encoder FRC mechanisms.
 *
 * Provides the EasyCRT solver and its configuration, plus free inline utilities
 * for gear-ratio arithmetic and coprimality checks. Created by team 6911.
 *
 * @par Full example — turret with 200T gear, 19T and 21T encoder pinions
 * @code{.cpp}
 * #include "yams/units/EasyCRT.hpp"
 * #include <ctre/phoenix6/CANcoder.hpp>
 * #include <frc2/command/SubsystemBase.h>
 *
 * using namespace yams::units;
 *
 * class Turret : public frc2::SubsystemBase {
 *  public:
 *   Turret() {
 *     // enc1Teeth=19, enc2Teeth=21 are coprime.
 *     // commonK = 1.0 (no intermediate gearbox) * 200T (turret gear) = 200.
 *     // CRT period = lcm(19,21)/200 = 399/200 = 1.995 rot — covers full travel.
 *     m_crt = std::make_unique<EasyCRT>(
 *         EasyCRTConfig{}
 *             .WithTeeth(19, 21, CrtCommonK(1.0, 200))
 *             .WithRange(0.0, 1.99)
 *             .WithTolerance(0.01));
 *
 *     // Wire suppliers after construction.
 *     m_crt->GetConfig().enc1 = [this] {
 *       return m_enc19T.GetAbsolutePosition().GetValue();
 *     };
 *     m_crt->GetConfig().enc2 = [this] {
 *       return m_enc21T.GetAbsolutePosition().GetValue();
 *     };
 *   }
 *
 *   void Periodic() override {
 *     auto angle = m_crt->GetAngle();
 *
 *     if (angle) {
 *       // Resolved: seed or correct the motor controller position.
 *       m_motor.SetEncoderPosition(*angle);
 *     } else if (m_crt->GetStatus() == EasyCRT::Status::InvalidConfig) {
 *       // Sensor read failed (NaN) — flag for diagnostics.
 *     }
 *
 *     // Log for tolerance tuning.
 *     frc::SmartDashboard::PutNumber("CRT/error", m_crt->GetLastError());
 *   }
 *
 *  private:
 *   ctre::phoenix6::hardware::CANcoder m_enc19T{33};
 *   ctre::phoenix6::hardware::CANcoder m_enc21T{34};
 *   std::unique_ptr<EasyCRT> m_crt;
 * };
 * @endcode
 */

namespace yams::units {

// ── Math utilities ────────────────────────────────────────────────────────────

/**
 * @brief Greatest common divisor (iterative Euclidean algorithm).
 * @param a First integer.
 * @param b Second integer.
 * @return GCD of @p a and @p b.
 */
inline int CrtGcd(int a, int b) {
  while (b) {
    int t = b;
    b = a % b;
    a = t;
  }
  return a;
}

/**
 * @brief Least common multiple.
 * @param a First integer.
 * @param b Second integer.
 * @return LCM of @p a and @p b.
 */
inline int CrtLcm(int a, int b) { return (a / CrtGcd(a, b)) * b; }

/**
 * @brief Returns true when @p a and @p b share no common factor beyond 1.
 * @param a First integer.
 * @param b Second integer.
 * @return @c true if coprime.
 */
inline bool CrtIsCoprime(int a, int b) { return CrtGcd(a, b) == 1; }

// ── Ratio discovery ───────────────────────────────────────────────────────────

/**
 * @brief Encoder rotations per mechanism rotation from a meshed gear chain.
 *
 * Supply tooth counts in order from the mechanism-side gear to the encoder
 * pinion, e.g. @c {50, 20, 40} for 50T→20T→40T encoder.  Intermediate
 * idler teeth cancel algebraically, so the result equals
 * @c teeth[0] / teeth[last].
 *
 * @param teeth Ordered tooth counts, at least two elements.
 * @return Encoder rotations per mechanism rotation.
 */
inline double CrtRatioFromChain(std::initializer_list<int> teeth) {
  if (teeth.size() < 2) return 1.0;
  return static_cast<double>(*teeth.begin()) / static_cast<double>(*(teeth.end() - 1));
}

/**
 * @brief Encoder rotations per mechanism rotation from explicit stage pairs.
 *
 * Supply alternating (driver, driven) tooth counts for each mesh stage,
 * e.g. @c {12, 36, 18, 60} for a two-stage compound train.
 * Each stage contributes @c driver/driven to the product.
 *
 * @param pairs Even-length list of alternating driver and driven tooth counts.
 * @return Encoder rotations per mechanism rotation.
 */
inline double CrtRatioFromStages(std::initializer_list<int> pairs) {
  double r = 1.0;
  const int* p = pairs.begin();
  while (p + 1 < pairs.end()) {
    r *= static_cast<double>(p[0]) / static_cast<double>(p[1]);
    p += 2;
  }
  return r;
}

/**
 * @brief Computes the common-scale factor K used by EasyCRTConfig::commonK.
 *
 * When both encoders are driven off the same drive gear after a shared
 * reduction, K = commonRatio × driveTeeth.  The per-encoder ratio is then
 * K / encTeeth, and the CRT period (unique coverage) is
 * lcm(enc1Teeth, enc2Teeth) / K mechanism rotations.
 *
 * @param commonRatio Ratio between mechanism and the shared drive gear.
 * @param driveTeeth  Tooth count on the gear that drives both encoder pinions.
 * @return K value to pass to EasyCRTConfig::WithTeeth.
 */
inline double CrtCommonK(double commonRatio, int driveTeeth) { return commonRatio * driveTeeth; }

// ── Config ────────────────────────────────────────────────────────────────────

/**
 * @brief Configuration for the EasyCRT solver.
 *
 * @par Required setup
 * Call WithTeeth() to supply the two encoder gear tooth counts and the
 * common scale factor K.  enc1Teeth and enc2Teeth **must be coprime**; the
 * mechanism range must not exceed one CRT period
 * (= lcm(enc1Teeth, enc2Teeth) / commonK rotations).
 *
 * @par Offsets and inversion
 * If the encoder hardware supports on-device zeroing, leave the software
 * offsets at zero to avoid double-offsetting.  Use inv1 / inv2 only when
 * the encoder physically reads backwards and cannot be inverted in firmware.
 */
struct EasyCRTConfig {
  std::function<::units::turn_t()> enc1; /**< Supplier for absolute encoder 1. */
  std::function<::units::turn_t()> enc2; /**< Supplier for absolute encoder 2. */

  int enc1Teeth = 19; /**< Tooth count of encoder 1 gear — must be coprime with enc2Teeth. */
  int enc2Teeth = 21; /**< Tooth count of encoder 2 gear — must be coprime with enc1Teeth. */
  double commonK =
      1.0; /**< Mechanism-to-encoder scale: ratio = commonK / encTeeth. See CrtCommonK(). */
  double offset1 = 0.0; /**< Offset (turns) added to enc1 reading before wrap. */
  double offset2 = 0.0; /**< Offset (turns) added to enc2 reading before wrap. */
  double minRot = 0.0;  /**< Minimum allowed mechanism position (turns). */
  double maxRot = 1.0;  /**< Maximum allowed mechanism position (turns). */
  double tolerance =
      0.005; /**< Max combined encoder modular error (normalised turns) to accept a solve. */
  bool inv1 =
      false; /**< Flip enc1 reading; use only when the sensor cannot be inverted in firmware. */
  bool inv2 =
      false; /**< Flip enc2 reading; use only when the sensor cannot be inverted in firmware. */

  /**
   * @brief Set encoder teeth counts and the common scale factor.
   * @param t1 Tooth count for encoder 1 gear (coprime with @p t2).
   * @param t2 Tooth count for encoder 2 gear (coprime with @p t1).
   * @param k  Common scale factor K = commonRatio × driveTeeth.
   * @return *this for chaining.
   */
  EasyCRTConfig& WithTeeth(int t1, int t2, double k) {
    enc1Teeth = t1;
    enc2Teeth = t2;
    commonK = k;
    return *this;
  }

  /**
   * @brief Set software offsets applied to both encoders before wrap.
   * @param o1 Offset in turns for encoder 1.
   * @param o2 Offset in turns for encoder 2.
   * @return *this for chaining.
   */
  EasyCRTConfig& WithOffsets(double o1, double o2) {
    offset1 = o1;
    offset2 = o2;
    return *this;
  }

  /**
   * @brief Set the allowed mechanism position range.
   * @param minR Minimum position in turns.
   * @param maxR Maximum position in turns (must satisfy maxR - minR ≤ CRT period).
   * @return *this for chaining.
   */
  EasyCRTConfig& WithRange(double minR, double maxR) {
    minRot = minR;
    maxRot = maxR;
    return *this;
  }

  /**
   * @brief Set the maximum acceptable combined encoder error.
   *
   * Each encoder's modular error is measured as a normalised fraction of one
   * encoder rotation [0, 0.5).  A combined error above @p tol causes the
   * solver to report NoSolution.  Tune against GetLastError() on your robot.
   *
   * @param tol Error threshold (normalised encoder turns, typically 0.005–0.05).
   * @return *this for chaining.
   */
  EasyCRTConfig& WithTolerance(double tol) {
    tolerance = tol;
    return *this;
  }

  /**
   * @brief Flip the sign of encoder readings for reversed sensors.
   * @param i1 True to invert encoder 1.
   * @param i2 True to invert encoder 2.
   * @return *this for chaining.
   */
  EasyCRTConfig& WithInversions(bool i1, bool i2) {
    inv1 = i1;
    inv2 = i2;
    return *this;
  }
};

// ── Solver ────────────────────────────────────────────────────────────────────

/**
 * @brief CRT-based absolute angle solver using two absolute encoders.
 *
 * @par Algorithm
 * Given two absolute encoders with coprime gear tooth counts (enc1Teeth,
 * enc2Teeth) driven from a common mechanism, the Chinese Remainder Theorem
 * guarantees a unique mechanism position within one CRT period
 * (= lcm(enc1Teeth, enc2Teeth) / commonK rotations).
 *
 * GetAngle() tries all four floor/ceil rounding combinations for the two
 * encoder readings (2 × 2 = 4 candidates), applies the precomputed CRT
 * weights to recover the mechanism tooth count for each candidate in O(1),
 * then selects the candidate with the smallest combined encoder error.
 * Sub-tooth interpolation from enc2 provides fractional-tooth precision
 * beyond the integer CRT result.
 *
 * @par Configuration
 * Construct with a fully populated EasyCRTConfig. enc1Teeth and enc2Teeth
 * must be coprime and the mechanism range must not exceed one CRT period.
 *
 * @par Typical usage
 * @code
 * EasyCRTConfig cfg;
 * cfg.enc1 = [] { return encoder1.GetAbsolutePosition(); };
 * cfg.enc2 = [] { return encoder2.GetAbsolutePosition(); };
 * cfg.WithTeeth(19, 21, CrtCommonK(1.0, 200))
 *    .WithRange(0.0, 1.99)
 *    .WithTolerance(0.01);
 * EasyCRT solver{cfg};
 *
 * // In periodic:
 * auto angle = solver.GetAngle();  // std::optional<units::turn_t>
 * @endcode
 */
class EasyCRT {
 public:
  /**
   * @brief Result status from the most recent GetAngle() call.
   */
  enum class Status {
    NotAttempted, /**< No solve has been attempted since construction. */
    Ok,           /**< Solve succeeded; GetAngle() returned a value. */
    NoSolution,   /**< No candidate fell within the range and tolerance. */
    Ambiguous,    /**< Two candidates tied within tolerance; result suppressed. */
    InvalidConfig /**< A config parameter was non-finite, zero, or inconsistent. */
  };

  /**
   * @brief Construct a solver and precompute the CRT weights.
   *
   * The constructor derives modular-inverse weights from enc1Teeth and
   * enc2Teeth.  If those values are invalid (≤ 0) the first GetAngle() call
   * will return InvalidConfig.
   *
   * @param cfg Fully populated EasyCRTConfig.
   */
  explicit EasyCRT(EasyCRTConfig cfg);

  /**
   * @brief Read both encoders and return the mechanism angle if uniquely resolved.
   *
   * Calls the enc1 and enc2 suppliers, applies offsets and optional inversion,
   * then runs the four-candidate CRT solve.  Updates status, last error, and
   * iteration count as side-effects.
   *
   * @return Mechanism angle in turns, or @c std::nullopt on failure.
   */
  std::optional<::units::turn_t> GetAngle();

  /**
   * @brief Returns the status from the most recent GetAngle() call.
   * @return Last Status value.
   */
  Status GetStatus() const { return m_status; }

  /**
   * @brief Returns the combined encoder modular error from the last solve.
   *
   * This is the sum of the per-encoder ModularError values for the winning
   * candidate, in normalised encoder turns.  Returns NaN before the first call.
   *
   * @return Last error, or NaN if not yet attempted.
   */
  double GetLastError() const { return m_lastErr; }

  /**
   * @brief Returns the number of candidates evaluated in the last solve.
   *
   * Always 4 (the 2 × 2 floor/ceil combinations), or 0 before the first call.
   *
   * @return Candidate count from the previous GetAngle() call.
   */
  int GetLastIterations() const { return m_lastIter; }

 private:
  static double ModularError(double a, double b);
  static double CenteredMod(double v, double halfRange);

  EasyCRTConfig m_cfg;
  int m_crtW1;   ///< CRT weight for enc1: ModInverse(enc2Teeth, enc1Teeth) * enc2Teeth.
  int m_crtW2;   ///< CRT weight for enc2: ModInverse(enc1Teeth, enc2Teeth) * enc1Teeth.
  int m_crtMod;  ///< enc1Teeth * enc2Teeth (valid when the teeth are coprime).
  Status m_status = Status::NotAttempted;
  double m_lastErr = std::numeric_limits<double>::quiet_NaN();
  int m_lastIter = 0;
};

}  // namespace yams::units
