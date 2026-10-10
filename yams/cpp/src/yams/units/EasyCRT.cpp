// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/units/EasyCRT.hpp"

#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace yams::units {

namespace {

/** Fraction of the gap between neighboring candidates that a match may be off by. */
constexpr double kToleranceFraction = 0.25;

/** Wrap a value in rotations into [0, 1). */
double WrapRotation(double rotations) { return rotations - std::floor(rotations); }

/** Distance in rotations between two readings, at most half a rotation. */
double RotationError(double a, double b) {
  const double difference = WrapRotation(a - b);
  return difference > 0.5 ? 1.0 - difference : difference;
}

}  // namespace

EasyCRT::EasyCRT(int sharedGearTeeth, int encoderATeeth, int encoderBTeeth) {
  if (sharedGearTeeth <= 0 || encoderATeeth <= 0 || encoderBTeeth <= 0) {
    throw std::invalid_argument("Gear tooth counts must be positive");
  }
  const int commonTeeth = std::gcd(encoderATeeth, encoderBTeeth);
  m_ratioA = static_cast<double>(sharedGearTeeth) / encoderATeeth;
  m_ratioB = static_cast<double>(sharedGearTeeth) / encoderBTeeth;
  // The readings repeat together every lcm(A, B) teeth of the shared gear.
  m_uniqueRangeRotations =
      static_cast<double>(encoderATeeth) / commonTeeth * encoderBTeeth / sharedGearTeeth;
  // Neighboring candidates predict encoder B readings this many rotations apart.
  m_toleranceRotations = kToleranceFraction * commonTeeth / encoderBTeeth;
}

std::optional<wpi::units::turn_t> EasyCRT::Solve(wpi::units::turn_t encoderA,
                                                 wpi::units::turn_t encoderB) const {
  return SolveRotations(encoderA.value(), encoderB.value(), -m_uniqueRangeRotations / 2.0,
                        m_uniqueRangeRotations / 2.0);
}

std::optional<wpi::units::turn_t> EasyCRT::Solve(wpi::units::turn_t encoderA,
                                                 wpi::units::turn_t encoderB,
                                                 wpi::units::turn_t minAngle,
                                                 wpi::units::turn_t maxAngle) const {
  const double minRotations = minAngle.value();
  const double maxRotations = maxAngle.value();
  if (maxRotations < minRotations) {
    throw std::invalid_argument("maxAngle must not be below minAngle");
  }
  if (maxRotations - minRotations > m_uniqueRangeRotations + 1e-9) {
    throw std::invalid_argument("Travel is longer than the range these gears can tell apart");
  }
  return SolveRotations(encoderA.value(), encoderB.value(), minRotations, maxRotations);
}

wpi::units::turn_t EasyCRT::GetUniqueRange() const {
  return wpi::units::turn_t{m_uniqueRangeRotations};
}

std::optional<wpi::units::turn_t> EasyCRT::SolveRotations(double encoderA, double encoderB,
                                                          double minRotations,
                                                          double maxRotations) const {
  if (!std::isfinite(encoderA) || !std::isfinite(encoderB)) {
    return std::nullopt;
  }
  const double a = WrapRotation(encoderA);
  const double b = WrapRotation(encoderB);

  // a = (ratioA * gear) mod 1, so gear = (a + turns) / ratioA for a whole number of turns.
  const int minTurns = static_cast<int>(std::floor(m_ratioA * minRotations - a)) - 1;
  const int maxTurns = static_cast<int>(std::ceil(m_ratioA * maxRotations - a)) + 1;

  double bestError = std::numeric_limits<double>::max();
  double secondError = std::numeric_limits<double>::max();
  double bestRotations = std::numeric_limits<double>::quiet_NaN();
  for (int turns = minTurns; turns <= maxTurns; ++turns) {
    const double gearRotations = (a + turns) / m_ratioA;
    if (gearRotations < minRotations - 1e-6 || gearRotations > maxRotations + 1e-6) {
      continue;
    }
    const double error = RotationError(m_ratioB * gearRotations, b);
    if (error < bestError) {
      secondError = bestError;
      bestError = error;
      bestRotations = gearRotations;
    } else if (error < secondError) {
      secondError = error;
    }
  }

  // No candidate is close enough, or two are, which happens at the very ends of the range.
  if (bestError > m_toleranceRotations || secondError <= m_toleranceRotations) {
    return std::nullopt;
  }
  return wpi::units::turn_t{bestRotations};
}

}  // namespace yams::units
