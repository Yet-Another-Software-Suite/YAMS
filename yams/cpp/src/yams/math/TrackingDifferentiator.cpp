// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// The tracking differentiator, part of the ADRC, was written by FRC Team 8331, who contributed it to YAMS.

#include "yams/math/TrackingDifferentiator.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace yams::math {

namespace {
/** -1, 0 or 1, as Java's Math.signum. */
double Signum(double value) { return value > 0 ? 1.0 : (value < 0 ? -1.0 : 0.0); }
}  // namespace

TrackingDifferentiator::TrackingDifferentiator(Method method, wpi::units::second_t dt, std::vector<double> params)
    : m_method{method}, m_dt{dt.value()}, m_params{std::move(params)} {
  if (!std::isfinite(m_dt) || m_dt <= 0.0) {
    throw std::invalid_argument("Sample time dt must be finite and positive");
  }
  ValidateParams(m_dt);
  Reset(0.0);
}

void TrackingDifferentiator::SetParameters(std::vector<double> params) {
  if (params.empty()) return;
  std::vector<double> old = std::exchange(m_params, std::move(params));
  try {
    ValidateParams(m_dt);
  } catch (...) {
    m_params = std::move(old);
    throw;
  }
}

void TrackingDifferentiator::SetSampleTime(wpi::units::second_t dt) {
  const double seconds = dt.value();
  if (!std::isfinite(seconds) || seconds <= 0.0) {
    throw std::invalid_argument("Sample time dt must be finite and positive");
  }
  ValidateParams(seconds);
  m_dt = seconds;
}

double TrackingDifferentiator::Param(size_t index, double defaultValue) const {
  return m_params.size() > index ? m_params[index] : defaultValue;
}

void TrackingDifferentiator::ValidateParams(double dt) const {
  switch (m_method) {
    case Method::EULER: {
      const double alpha = Param(0, 0.9);
      if (!std::isfinite(alpha) || alpha < 0.0 || alpha > 1.0) {
        throw std::invalid_argument("alpha must be in [0, 1] for EULER");
      }
      break;
    }
    case Method::TOD: {
      const double r = Param(0, 1.0);
      if (!std::isfinite(r) || r <= 0.0) throw std::invalid_argument("r must be positive for TOD");
      break;
    }
    case Method::LD: {
      const double lambda = Param(0, 1.0);
      if (!std::isfinite(lambda) || lambda <= 0.0) throw std::invalid_argument("lambda must be positive for LD");
      if (dt >= 2.0 * lambda) throw std::invalid_argument("ld unstable at this dt: need dt < 2 * lambda");
      break;
    }
    case Method::RED: {
      const double lambda1 = Param(0, 1.0);
      const double lambda2 = Param(1, 1.0);
      if (!std::isfinite(lambda1) || !std::isfinite(lambda2) || lambda1 <= 0.0 || lambda2 <= 0.0) {
        throw std::invalid_argument("lambda1 and lambda2 must be positive for RED");
      }
      break;
    }
    case Method::INTD: {
      const double alpha = Param(0, 0.5);
      const double beta = Param(1, 1.0);
      const double gamma = Param(2, 1.0);
      const double r = Param(3, 1.0);
      if (!std::isfinite(alpha) || alpha < 0.0 || alpha >= 1.0) {
        throw std::invalid_argument("alpha must be in [0, 1) for INTD");
      }
      if (!std::isfinite(beta) || !std::isfinite(gamma) || !std::isfinite(r) || beta <= 0.0 || gamma <= 0.0 ||
          r <= 0.0) {
        throw std::invalid_argument("beta, gamma, r must be positive for INTD");
      }
      break;
    }
  }
}

void TrackingDifferentiator::Reset(double initialValue) {
  m_prevY = initialValue;
  m_prevYd = 0.0;
  m_v1 = initialValue;
  m_v2 = 0.0;
  m_y = initialValue;
  m_yd = 0.0;
  m_ydd = 0.0;
}

void TrackingDifferentiator::Update(double reference) {
  switch (m_method) {
    case Method::EULER:
      UpdateEuler(reference);
      break;
    case Method::TOD:
      UpdateTod(reference);
      break;
    case Method::LD:
      UpdateLd(reference);
      break;
    case Method::RED:
      UpdateRed(reference);
      break;
    case Method::INTD:
      UpdateIntd(reference);
      break;
  }
}

void TrackingDifferentiator::UpdateEuler(double reference) {
  const double alpha = Param(0, 0.9);
  m_y = alpha * m_prevY + (1.0 - alpha) * reference;
  m_yd = (m_y - m_prevY) / m_dt;
  m_ydd = (m_yd - m_prevYd) / m_dt;
  m_prevY = m_y;
  m_prevYd = m_yd;
}

void TrackingDifferentiator::UpdateTod(double reference) {
  const double r = Param(0, 1.0);
  const double diff = m_v1 - reference + (m_v2 * std::abs(m_v2)) / (2.0 * r);
  const double dotV1 = m_v2;
  const double dotV2 = -r * Signum(diff);
  m_v1 += dotV1 * m_dt;
  m_v2 += dotV2 * m_dt;
  m_y = m_v1;
  m_yd = m_v2;
  m_ydd = dotV2;
}

void TrackingDifferentiator::UpdateLd(double reference) {
  const double r = 1.0 / Param(0, 1.0);
  const double dotV1 = m_v2;
  const double dotV2 = -r * r * (m_v1 - reference) - 2.0 * r * m_v2;
  m_v1 += dotV1 * m_dt;
  m_v2 += dotV2 * m_dt;
  m_y = m_v1;
  m_yd = m_v2;
  m_ydd = dotV2;
}

void TrackingDifferentiator::UpdateRed(double reference) {
  const double lambda1 = Param(0, 1.0);
  const double lambda2 = Param(1, 1.0);
  const double err = m_v1 - reference;
  const double dotV1 = m_v2 - lambda1 * std::sqrt(std::abs(err)) * (err > 0 ? 1.0 : -1.0);
  const double dotV2 = -lambda2 * Signum(err);
  m_v1 += dotV1 * m_dt;
  m_v2 += dotV2 * m_dt;
  m_y = m_v1;
  m_yd = m_v2;
  m_ydd = dotV2;
}

void TrackingDifferentiator::UpdateIntd(double reference) {
  const double alpha = Param(0, 0.5);
  const double beta = Param(1, 1.0);
  const double gamma = Param(2, 1.0);
  const double r = Param(3, 1.0);
  const double scale = beta / (1.0 - alpha);
  const double dotV1 = m_v2;
  const double dotV2 = -r * r * scale * std::tanh((1.0 - alpha) * (m_v1 - reference) / gamma) - r * m_v2;
  m_v1 += dotV1 * m_dt;
  m_v2 += dotV2 * m_dt;
  m_y = m_v1;
  m_yd = m_v2;
  m_ydd = dotV2;
}

}  // namespace yams::math
