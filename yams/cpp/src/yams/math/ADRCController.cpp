// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// The ADRC (active disturbance rejection controller) was written by FRC Team 8331, who contributed it to YAMS.

#include "yams/math/ADRCController.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <stdexcept>
#include <utility>

namespace yams::math {

namespace {
/** Observer bandwidth as a multiple of the controller bandwidth. */
constexpr double kObserverBandwidthRatio = 4.0;

/** Binomial coefficient, total choose choose. */
double Binomial(int total, int choose) {
  if (choose < 0 || choose > total) return 0;
  if (choose == 0 || choose == total) return 1;
  if (choose > total / 2) choose = total - choose;
  double result = 1;
  for (int i = 1; i <= choose; ++i) result = result * (total - i + 1) / i;
  return result;
}

/** value! */
double Factorial(int value) {
  double result = 1;
  for (int i = 2; i <= value; ++i) result *= i;
  return result;
}

/** Matrix exponential exp(a * dt) by Taylor series. */
Eigen::MatrixXd Expm(const Eigen::MatrixXd& a, double dt) {
  const auto m = a.rows();
  Eigen::MatrixXd sum = Eigen::MatrixXd::Identity(m, m);
  Eigen::MatrixXd term = Eigen::MatrixXd::Identity(m, m);
  for (int i = 1; i < 50; ++i) {
    term = (term * a) * (dt / i);
    sum += term;
    if (term.cwiseAbs().maxCoeff() < 1e-15) break;
  }
  return sum;
}

/** The discrete input matrix by Taylor series. */
Eigen::MatrixXd DiscreteInput(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b, double dt) {
  const auto m = a.rows();
  Eigen::MatrixXd sum = Eigen::MatrixXd::Zero(m, m);
  Eigen::MatrixXd aPow = Eigen::MatrixXd::Identity(m, m);
  for (int i = 1; i <= m + 1; ++i) {
    sum += aPow * (std::pow(dt, i) / Factorial(i));
    aPow = aPow * a;
  }
  return sum * b;
}

/** The observer gains for a plant order, from the discrete observer pole z = exp(-wo * dt). */
Eigen::VectorXd ObserverGains(int systemOrder, double z, double dt) {
  Eigen::VectorXd res = Eigen::VectorXd::Zero(systemOrder + 1);
  const double q = 1.0 - z;
  switch (systemOrder) {
    case 1:
      res(0) = 1.0 - z * z;
      res(1) = q * q / dt;
      break;
    case 2:
      res(0) = 1.0 - z * z * z;
      res(1) = (1.0 + z) * q * q * 3.0 / (2.0 * dt);
      res(2) = q * q * q / (dt * dt);
      break;
    case 3:
      res(0) = 1.0 - z * z * z * z;
      res(1) = q * q * (11.0 + z * (14.0 + 11.0 * z)) / (6.0 * dt);
      res(2) = q * q * q * (1.0 + z) * 2.0 / (dt * dt);
      res(3) = q * q * q * q / (dt * dt * dt);
      break;
    case 4:
      res(0) = 1.0 - z * z * z * z * z;
      res(1) = 5.0 * q * q * (1.0 + z) * (5.0 + z * (2.0 + 5.0 * z)) / (12.0 * dt);
      res(2) = 5.0 * q * q * q * (7.0 + z * (10.0 + 7.0 * z)) / (12.0 * dt * dt);
      res(3) = q * q * q * q * (1.0 + z) * 5.0 / (2.0 * dt * dt * dt);
      res(4) = q * q * q * q * q / (dt * dt * dt * dt);
      break;
    default:
      throw std::logic_error("Unsupported system order: " + std::to_string(systemOrder));
  }
  return res;
}
}  // namespace

ADRCController::ADRCController(std::shared_ptr<ADRCConfig> config) : m_config{std::move(config)} {
  if (!m_config) throw std::invalid_argument("The ADRC config must not be null");
  const auto order = m_config->GetSystemOrder();
  if (!order) throw std::invalid_argument("The ADRC config has no system order");
  m_systemOrder = ADRCConfig::GetOrder(*order);
}

void ADRCController::Initialize(double b0, wpi::units::second_t period) {
  m_initialized = false;
  const double seconds = period.value();
  if (!std::isfinite(seconds) || seconds <= 0) {
    throw std::invalid_argument("The control period must be finite and positive");
  }
  const double delaySteps = m_config->GetInputDelay().value_or(ADRCConfig::kDefaultInputDelay).value() / seconds;
  if (delaySteps > std::numeric_limits<int>::max() - 1) {
    throw std::invalid_argument("The input delay is too many control periods");
  }
  std::optional<TrackingDifferentiator> td;
  if (const auto method = m_config->GetTdMethod()) {
    td.emplace(*method, period, m_config->GetTdParams());
  }
  m_config->SetBuiltValues(b0, period);

  m_xhat = Eigen::VectorXd::Zero(m_systemOrder + 1);
  m_secondaryXhat = Eigen::VectorXd::Zero(m_systemOrder + 1);
  m_uPrev = 0.0;
  m_uHistory.assign(static_cast<size_t>(std::lround(delaySteps)) + 1, 0.0);
  m_td = std::move(td);
  m_initialized = true;
}

wpi::units::volt_t ADRCController::Step(double reference, double output, const std::vector<double>& refDerivatives) {
  RequireInitialized();
  // One read, so the whole step uses the same gains and limits while live tuning changes them.
  const ADRCConfig::StepValues values = m_config->GetStepValues();
  const ADRCGains& gains = *values.gains;

  // The reference and its derivatives, up to the plant order.
  std::vector<double> ref(m_systemOrder + 1, 0.0);
  ref[0] = reference;
  for (size_t i = 0; i < refDerivatives.size() && i < static_cast<size_t>(m_systemOrder); ++i) {
    ref[i + 1] = refDerivatives[i];
  }
  if (m_td) {
    m_td->Update(reference);
    ref[0] = m_td->GetY();
    ref[1] = m_td->GetYd();
  }

  // The observers are told the output that reached the plant, one input delay ago.
  const double uForEso = m_uHistory.back();
  m_xhat = gains.ad * m_xhat + gains.bd * uForEso + gains.ld * output;
  m_secondaryXhat = gains.ad * m_secondaryXhat + gains.bd * uForEso + gains.ld * m_xhat(0);

  // The cascaded form cancels the second observer's disturbance estimate.
  const Eigen::VectorXd& disturbance = values.cascaded ? m_secondaryXhat : m_xhat;
  double v = ref[m_systemOrder] - disturbance(m_systemOrder);
  for (int i = 0; i < m_systemOrder; ++i) {
    v += gains.k(i) * (ref[i] - m_xhat(i));
  }
  const double u = v / gains.b0;

  std::rotate(m_uHistory.rbegin(), m_uHistory.rbegin() + 1, m_uHistory.rend());
  m_uHistory[0] = u;
  m_uPrev = u;
  return wpi::units::volt_t{u};
}

wpi::units::volt_t ADRCController::Calculate(double measurement, double setpoint) {
  return Step(setpoint, measurement);
}

wpi::units::volt_t ADRCController::Calculate(double measurement, double setpoint, wpi::units::volt_t appliedOutput) {
  RequireInitialized();
  if (std::isfinite(appliedOutput.value())) {
    m_uHistory[0] = appliedOutput.value();
    m_uPrev = appliedOutput.value();
  }
  return Step(setpoint, measurement);
}

double ADRCController::Bandwidth(int systemOrder, double bandwidthScale, wpi::units::second_t settlingTime) {
  return 2.0 * (systemOrder + 1) * bandwidthScale / settlingTime.value();
}

ADRCGains ADRCController::ComputeGains(int systemOrder, double wc, double b0, double dt) {
  const int m = systemOrder + 1;
  const double wo = kObserverBandwidthRatio * wc;

  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(m, m);
  for (int i = 0; i < systemOrder; ++i) a(i, i + 1) = 1.0;
  Eigen::MatrixXd b = Eigen::MatrixXd::Zero(m, 1);
  b(systemOrder - 1, 0) = b0;

  ADRCGains gains;
  gains.ld = ObserverGains(systemOrder, std::exp(-wo * dt), dt);
  Eigen::MatrixXd c = Eigen::MatrixXd::Zero(1, m);
  c(0, 0) = 1.0;
  const Eigen::MatrixXd iMinusLc = Eigen::MatrixXd::Identity(m, m) - gains.ld * c;
  gains.ad = iMinusLc * Expm(a, dt);
  gains.bd = iMinusLc * DiscreteInput(a, b, dt);
  gains.k = Eigen::VectorXd::Zero(systemOrder);
  for (int i = 1; i <= systemOrder; ++i) {
    gains.k(systemOrder - i) = Binomial(systemOrder, i) * std::pow(wc, i);
  }
  gains.b0 = b0;
  return gains;
}

void ADRCController::Reset(double measurement) {
  RequireInitialized();
  if (!std::isfinite(measurement)) throw std::invalid_argument("measurement must be finite");
  std::vector<double> xhatInit(m_systemOrder + 1, 0.0);
  xhatInit[0] = measurement;
  Reset(xhatInit, wpi::units::volt_t{0.0});
}

void ADRCController::Reset(const std::vector<double>& xhatInit, wpi::units::volt_t uInit) {
  RequireInitialized();
  if (!xhatInit.empty() && xhatInit.size() != static_cast<size_t>(m_systemOrder + 1)) {
    throw std::invalid_argument("xhatInit length must equal controller order");
  }
  if (!std::isfinite(uInit.value())) throw std::invalid_argument("uInit must be finite");
  Eigen::VectorXd x = Eigen::VectorXd::Zero(m_systemOrder + 1);
  for (size_t i = 0; i < xhatInit.size(); ++i) x(static_cast<Eigen::Index>(i)) = xhatInit[i];
  m_xhat = x;
  m_secondaryXhat = x;
  m_uPrev = uInit.value();
  std::fill(m_uHistory.begin(), m_uHistory.end(), m_uPrev);
  if (m_td) m_td->Reset(x(0));
}

void ADRCController::RequireInitialized() const {
  if (!m_initialized) throw std::logic_error("Controller must be initialized before use");
}

double ADRCController::GetB0() const {
  RequireInitialized();
  return m_config->GetGains()->b0;
}

wpi::units::radians_per_second_t ADRCController::GetBandwidth() const {
  return wpi::units::radians_per_second_t{Bandwidth(m_systemOrder, GetBandwidthScale(), GetSettlingTime())};
}

wpi::units::radians_per_second_t ADRCController::GetObserverBandwidth() const {
  return kObserverBandwidthRatio * GetBandwidth();
}

wpi::units::second_t ADRCController::GetSettlingTime() const {
  return m_config->GetSettlingTime().value_or(ADRCConfig::kDefaultSettlingTime);
}

double ADRCController::GetBandwidthScale() const {
  return m_config->GetBandwidthScale().value_or(ADRCConfig::kDefaultBandwidthScale);
}

const TrackingDifferentiator* ADRCController::GetTrackingDifferentiator() const { return m_td ? &*m_td : nullptr; }

std::vector<double> ADRCController::GetEstimatedStates() const {
  RequireInitialized();
  return std::vector<double>(m_xhat.data(), m_xhat.data() + m_xhat.size());
}

double ADRCController::GetEstimatedDisturbance() const {
  RequireInitialized();
  return (m_config->IsCascaded() ? m_secondaryXhat : m_xhat)(m_systemOrder);
}

wpi::units::volt_t ADRCController::GetLastOutput() const { return wpi::units::volt_t{m_uPrev}; }

}  // namespace yams::math
