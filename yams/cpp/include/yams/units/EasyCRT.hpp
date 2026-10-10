// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <optional>
#include <wpi/units/angle.hpp>

/**
 * @file EasyCRT.hpp
 * @brief Absolute angle solver for a gear that turns more than once, read by two absolute encoders.
 *
 * @code{.cpp}
 * #include "yams/units/EasyCRT.hpp"
 *
 * // 100 tooth turret gear, 19 tooth pinion on encoder A, 21 tooth pinion on encoder B.
 * yams::units::EasyCRT crt{100, 19, 21};
 *
 * // Read once at startup, while the mechanism is not moving.
 * std::optional<wpi::units::turn_t> turretAngle =
 *     crt.Solve(encoderA.GetAbsolutePosition().GetValue(),
 *               encoderB.GetAbsolutePosition().GetValue());
 * @endcode
 */

namespace yams::units {

/**
 * Finds the absolute angle of a gear that turns more than once, from two absolute encoders on
 * pinions with different tooth counts that both mesh with it.
 *
 * Each encoder only knows its angle within one of its own rotations, but the two readings together
 * are unique over GetUniqueRange(), which is lcm(encoderATeeth, encoderBTeeth) / sharedGearTeeth
 * rotations of the shared gear.
 *
 * Both encoders must be zeroed with the mechanism at zero, and both must read increasing angles
 * when the mechanism moves in its positive direction. Set the offset and inversion on the encoders
 * themselves, or adjust the readings before passing them in.
 *
 * This is not a textbook Chinese Remainder Theorem solve. Every angle consistent with encoder A is
 * tried, and the one whose predicted encoder B reading is closest to the real one wins, which
 * stays stable with backlash and sensor noise. Created by team 6911.
 */
class EasyCRT {
 public:
  /**
   * Create a solver from the tooth counts of the three gears.
   *
   * @param sharedGearTeeth Teeth on the gear both encoder pinions mesh with. The solved angle is
   *                        the angle of this gear.
   * @param encoderATeeth   Teeth on the pinion of encoder A.
   * @param encoderBTeeth   Teeth on the pinion of encoder B.
   * @throws std::invalid_argument if any tooth count is zero or negative.
   */
  EasyCRT(int sharedGearTeeth, int encoderATeeth, int encoderBTeeth);

  /**
   * Solve for the angle of the shared gear, assuming its travel is centered on zero. The result is
   * within half of GetUniqueRange() either side of zero.
   *
   * @param encoderA Encoder A reading.
   * @param encoderB Encoder B reading.
   * @return The angle of the shared gear, or std::nullopt if the two readings do not agree on one.
   */
  std::optional<wpi::units::turn_t> Solve(wpi::units::turn_t encoderA,
                                          wpi::units::turn_t encoderB) const;

  /**
   * Solve for the angle of the shared gear when its travel is not centered on zero.
   *
   * @param encoderA Encoder A reading.
   * @param encoderB Encoder B reading.
   * @param minAngle Lowest angle the shared gear can be at.
   * @param maxAngle Highest angle the shared gear can be at.
   * @return The angle of the shared gear, or std::nullopt if the two readings do not agree on one.
   * @throws std::invalid_argument if the travel is longer than GetUniqueRange(), or maxAngle is
   *                               below minAngle.
   */
  std::optional<wpi::units::turn_t> Solve(wpi::units::turn_t encoderA, wpi::units::turn_t encoderB,
                                          wpi::units::turn_t minAngle,
                                          wpi::units::turn_t maxAngle) const;

  /**
   * Get how far the shared gear can turn before the pair of encoder readings repeats.
   *
   * @return The unique range of the shared gear.
   */
  wpi::units::turn_t GetUniqueRange() const;

 private:
  /**
   * Try every shared gear angle consistent with encoder A and keep the one that best predicts
   * encoder B.
   *
   * @param encoderA     Encoder A reading in rotations.
   * @param encoderB     Encoder B reading in rotations.
   * @param minRotations Lowest shared gear angle in rotations.
   * @param maxRotations Highest shared gear angle in rotations.
   * @return The angle of the shared gear, or std::nullopt if no single candidate matches.
   */
  std::optional<wpi::units::turn_t> SolveRotations(double encoderA, double encoderB,
                                                   double minRotations,
                                                   double maxRotations) const;

  /** Encoder A rotations per rotation of the shared gear. */
  double m_ratioA;
  /** Encoder B rotations per rotation of the shared gear. */
  double m_ratioB;
  /** Rotations of the shared gear over which the pair of encoder readings is unique. */
  double m_uniqueRangeRotations;
  /** Largest difference, in encoder B rotations, between its predicted and real reading. */
  double m_toleranceRotations;
};

}  // namespace yams::units
