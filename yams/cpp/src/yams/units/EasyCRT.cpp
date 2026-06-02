// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/units/EasyCRT.h"

#include <frc/MathUtil.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace yams::units {

EasyCRT::EasyCRT(EasyCRTConfig config) : m_config(std::move(config)) {}

std::optional<::units::angle::turn_t> EasyCRT::GetAngleOptional() {
  double ratio1 = m_config.GetEncoder1RotationsPerMechanismRotation();
  double ratio2 = m_config.GetEncoder2RotationsPerMechanismRotation();
  double minMechRot = m_config.GetMinMechanismAngle().value();
  double maxMechRot = m_config.GetMaxMechanismAngle().value();
  double tolRot = m_config.GetMatchTolerance().value();

  double raw1 =
      (m_config.GetAbsoluteEncoder1Angle() + m_config.GetAbsoluteEncoder1Offset()).value();
  double raw2 =
      (m_config.GetAbsoluteEncoder2Angle() + m_config.GetAbsoluteEncoder2Offset()).value();

  double abs1 = frc::InputModulus(raw1, 0.0, 1.0);
  double abs2 = frc::InputModulus(raw2, 0.0, 1.0);

  auto sol = ResolveFromSensors(abs1, abs2, ratio1, ratio2, minMechRot, maxMechRot, tolRot);
  if (!sol) return std::nullopt;
  return ::units::angle::turn_t{sol->mechanismRotations};
}

EasyCRT::CRTStatus EasyCRT::GetLastStatus() const { return m_lastStatus; }
double EasyCRT::GetLastErrorRotations() const { return m_lastErrorRot; }
int EasyCRT::GetLastIterations() const { return m_lastIterations; }

std::optional<EasyCRT::CrtSolution> EasyCRT::ResolveFromSensors(double abs1, double abs2,
                                                                double ratio1, double ratio2,
                                                                double minMechRot,
                                                                double maxMechRot,
                                                                double matchTolerance) {
  m_lastIterations = 0;
  m_lastErrorRot = std::numeric_limits<double>::quiet_NaN();

  if (!std::isfinite(abs1) || !std::isfinite(abs2) || !std::isfinite(ratio1) ||
      !std::isfinite(ratio2) || std::abs(ratio1) < 1e-12 || !std::isfinite(minMechRot) ||
      !std::isfinite(maxMechRot) || minMechRot > maxMechRot || !std::isfinite(matchTolerance) ||
      matchTolerance < 0.0) {
    m_lastStatus = CRTStatus::INVALID_CONFIG;
    return std::nullopt;
  }

  double bestErr = std::numeric_limits<double>::max();
  double secondErr = std::numeric_limits<double>::max();
  double bestRot = std::numeric_limits<double>::quiet_NaN();

  double nMinD = std::min(ratio1 * minMechRot, ratio1 * maxMechRot) - abs1;
  double nMaxD = std::max(ratio1 * minMechRot, ratio1 * maxMechRot) - abs1;
  int minN = static_cast<int>(std::floor(nMinD)) - 1;
  int maxN = static_cast<int>(std::ceil(nMaxD)) + 1;

  for (int n = minN; n <= maxN; ++n) {
    ++m_lastIterations;
    double mechRot = (abs1 + n) / ratio1;
    if (mechRot < minMechRot - 1e-6 || mechRot > maxMechRot + 1e-6) continue;

    double predicted2 = frc::InputModulus(ratio2 * mechRot, 0.0, 1.0);
    double err = ModularError(predicted2, abs2);

    if (err < bestErr) {
      secondErr = bestErr;
      bestErr = err;
      bestRot = mechRot;
    } else if (err < secondErr) {
      secondErr = err;
    }
  }

  if (!std::isfinite(bestRot) || bestErr > matchTolerance) {
    m_lastStatus = CRTStatus::NO_SOLUTION;
    m_lastErrorRot = bestErr;
    return std::nullopt;
  }

  if (secondErr <= matchTolerance && std::abs(secondErr - bestErr) < 1e-3) {
    m_lastStatus = CRTStatus::AMBIGUOUS;
    m_lastErrorRot = bestErr;
    return std::nullopt;
  }

  m_lastStatus = CRTStatus::OK;
  m_lastErrorRot = bestErr;
  return CrtSolution{bestRot, bestErr};
}

double EasyCRT::ModularError(double a, double b) {
  double diff = std::abs(a - b);
  return diff > 0.5 ? 1.0 - diff : diff;
}

}  // namespace yams::units
