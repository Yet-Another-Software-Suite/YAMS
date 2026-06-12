// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/units/EasyCRT.hpp"

#include <frc/MathUtil.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

namespace yams::units {

namespace {

int ModInverse(int a, int m) {
  int m0 = m, x0 = 0, x1 = 1;
  while (a > 1) {
    int q = a / m, t = m;
    m = a % m;
    a = t;
    t = x0;
    x0 = x1 - q * x0;
    x1 = t;
  }
  return x1 < 0 ? x1 + m0 : x1;
}

}  // namespace

EasyCRT::EasyCRT(EasyCRTConfig cfg) : m_cfg{std::move(cfg)} {
  const int m1 = m_cfg.enc1Teeth, m2 = m_cfg.enc2Teeth;
  m_crtMod = m1 * m2;
  m_crtW1 = (ModInverse(m2, m1) * m2) % m_crtMod;
  m_crtW2 = (ModInverse(m1, m2) * m1) % m_crtMod;
}

std::optional<::units::turn_t> EasyCRT::GetAngle() {
  m_lastIter = 4;
  m_lastErr = std::numeric_limits<double>::quiet_NaN();

  // Read, offset, wrap, then optionally invert into the CRT coordinate frame.
  const double r1 = frc::InputModulus(m_cfg.enc1().value() + m_cfg.offset1, 0.0, 1.0);
  const double r2 = frc::InputModulus(m_cfg.enc2().value() + m_cfg.offset2, 0.0, 1.0);
  const double abs1 = m_cfg.inv1 ? frc::InputModulus(1.0 - r1, 0.0, 1.0) : r1;
  const double abs2 = m_cfg.inv2 ? frc::InputModulus(1.0 - r2, 0.0, 1.0) : r2;

  // Single flat validity check.
  if (!std::isfinite(abs1) || !std::isfinite(abs2) || !std::isfinite(m_cfg.commonK) ||
      m_cfg.commonK < 1e-12 || m_cfg.minRot > m_cfg.maxRot || m_cfg.tolerance < 0.0 ||
      m_cfg.enc1Teeth < 1 || m_cfg.enc2Teeth < 1) {
    m_status = Status::InvalidConfig;
    return std::nullopt;
  }

  const double k = m_cfg.commonK;
  const double period = m_crtMod / k;
  const double raw1 = abs1 * m_cfg.enc1Teeth;  // [0, enc1Teeth)
  const double raw2 = abs2 * m_cfg.enc2Teeth;  // [0, enc2Teeth)
  const int64_t f1 = static_cast<int64_t>(std::floor(raw1));
  const int64_t f2 = static_cast<int64_t>(std::floor(raw2));

  double bestErr = std::numeric_limits<double>::max();
  double secondErr = std::numeric_limits<double>::max();
  double bestRot = std::numeric_limits<double>::quiet_NaN();

  for (int i = 0; i < 2; ++i) {
    const int a1 = static_cast<int>((f1 + i) % m_cfg.enc1Teeth);
    for (int j = 0; j < 2; ++j) {
      const int a2 = static_cast<int>((f2 + j) % m_cfg.enc2Teeth);

      // Closed-form CRT: directly recovers the mechanism position for this candidate.
      const int64_t residue =
          (static_cast<int64_t>(a1) * m_crtW1 + static_cast<int64_t>(a2) * m_crtW2) % m_crtMod;
      double mechRot = residue / k;

      // Shift into [minRot, maxRot] — valid when range ≤ period.
      const double shifts = std::ceil((m_cfg.minRot - mechRot) / period);
      if (shifts > 0.0) mechRot += shifts * period;
      if (mechRot < m_cfg.minRot - 1e-6 || mechRot > m_cfg.maxRot + 1e-6) continue;

      const double pred1 =
          std::fmod(mechRot * k, static_cast<double>(m_cfg.enc1Teeth)) / m_cfg.enc1Teeth;
      const double pred2 =
          std::fmod(mechRot * k, static_cast<double>(m_cfg.enc2Teeth)) / m_cfg.enc2Teeth;
      const double err = ModularError(pred1, abs1) + ModularError(pred2, abs2);

      if (err < bestErr) {
        secondErr = bestErr;
        bestErr = err;
        bestRot = mechRot;
      } else if (err < secondErr) {
        secondErr = err;
      }
    }
  }

  m_lastErr = bestErr;

  if (!std::isfinite(bestRot) || bestErr > m_cfg.tolerance) {
    m_status = Status::NoSolution;
    return std::nullopt;
  }
  if (secondErr <= m_cfg.tolerance && std::abs(secondErr - bestErr) < 1e-6) {
    m_status = Status::Ambiguous;
    return std::nullopt;
  }

  // Sub-tooth interpolation from enc2 adds fractional-tooth precision.
  const double pred2_teeth = std::fmod(bestRot * k, static_cast<double>(m_cfg.enc2Teeth));
  bestRot += CenteredMod(raw2 - pred2_teeth, 0.5) / k;

  m_status = Status::Ok;
  return ::units::turn_t{bestRot};
}

double EasyCRT::ModularError(double a, double b) {
  const double d = std::abs(a - b);
  return d > 0.5 ? 1.0 - d : d;
}

double EasyCRT::CenteredMod(double v, double halfRange) {
  const double mod = 2.0 * halfRange;
  v = std::fmod(v, mod);
  if (v > halfRange) v -= mod;
  if (v < -halfRange) v += mod;
  return v;
}

}  // namespace yams::units
