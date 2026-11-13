// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// The ADRC (active disturbance rejection controller) was written by FRC Team 8331, who contributed it to YAMS.

#include "yams/math/ADRCConfig.hpp"

#include <atomic>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <utility>

#include <format>

#include "yams/exceptions.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/math/ADRCController.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"

namespace yams::math {

namespace {
/** Counts every config made, so each one's alerts get their own ids. */
std::atomic<int> gInstances{0};
}  // namespace

ADRCConfig::ADRCConfig() : m_instance{gInstances++} {}

ADRCConfig::ADRCConfig(const ADRCConfig& other) : m_instance{gInstances++} {
  std::scoped_lock lock{other.m_mutex};
  m_owner = other.m_owner;
  m_systemOrder = other.m_systemOrder;
  m_settlingTime = other.m_settlingTime;
  m_bandwidthScale = other.m_bandwidthScale;
  m_b0 = other.m_b0;
  m_inputDelay = other.m_inputDelay;
  m_cascaded = other.m_cascaded;
  m_tdMethod = other.m_tdMethod;
  m_tdParams = other.m_tdParams;
  m_builtPeriod = other.m_builtPeriod;
  m_builtB0 = other.m_builtB0;
  // Gains of its own, so retuning the copy leaves the original's alone.
  m_gains = other.m_gains ? std::make_shared<const ADRCGains>(*other.m_gains) : nullptr;
}

ADRCConfig& ADRCConfig::operator=(const ADRCConfig& other) {
  if (this == &other) return *this;
  ADRCConfig copy{other};
  std::scoped_lock lock{m_mutex};
  m_owner = std::move(copy.m_owner);
  m_systemOrder = copy.m_systemOrder;
  m_settlingTime = copy.m_settlingTime;
  m_bandwidthScale = copy.m_bandwidthScale;
  m_b0 = copy.m_b0;
  m_inputDelay = copy.m_inputDelay;
  m_cascaded = copy.m_cascaded;
  m_tdMethod = copy.m_tdMethod;
  m_tdParams = std::move(copy.m_tdParams);
  m_builtPeriod = copy.m_builtPeriod;
  m_builtB0 = copy.m_builtB0;
  m_gains = std::move(copy.m_gains);
  return *this;
}

ADRCConfig::~ADRCConfig() = default;

ADRCConfig& ADRCConfig::WithSystemOrder(SystemOrder order) {
  std::scoped_lock lock{m_mutex};
  if (m_builtPeriod &&
      (GetOrder(order) != GetOrder(*m_systemOrder) || IsSpeedPerVolt(order) != IsSpeedPerVolt(*m_systemOrder))) {
    throw std::logic_error(std::format(
        "{} already runs a controller of order {}{}; a new order needs a new ADRCConfig", m_owner,
        GetOrder(*m_systemOrder), IsSpeedPerVolt(*m_systemOrder) ? " for an elevator" : ""));
  }
  m_systemOrder = order;
  return *this;
}

ADRCConfig& ADRCConfig::WithSettlingTime(wpi::units::second_t settlingTime) {
  const double seconds = settlingTime.value();
  std::scoped_lock lock{m_mutex};
  if (!std::isfinite(seconds) || seconds <= 0) {
    throw std::invalid_argument(
        std::format("{} settling time must be finite and positive, got {} s", m_owner, seconds));
  }
  m_settlingTime = settlingTime;
  RebuildGains();
  return *this;
}

ADRCConfig& ADRCConfig::WithBandwidthScale(double bandwidthScale) {
  std::scoped_lock lock{m_mutex};
  if (!std::isfinite(bandwidthScale) || bandwidthScale <= 0) {
    throw std::invalid_argument(
        std::format("{} bandwidth scale must be finite and positive, got {}", m_owner, bandwidthScale));
  }
  m_bandwidthScale = bandwidthScale;
  RebuildGains();
  return *this;
}

ADRCConfig& ADRCConfig::WithB0(double b0) {
  std::scoped_lock lock{m_mutex};
  if (!std::isfinite(b0) || b0 == 0.0) {
    throw std::invalid_argument(std::format("{} b0 must be finite and non-zero, got {}", m_owner, b0));
  }
  m_b0 = b0;
  RebuildGains();
  return *this;
}

ADRCConfig& ADRCConfig::WithInputDelay(wpi::units::second_t inputDelay) {
  const double seconds = inputDelay.value();
  std::scoped_lock lock{m_mutex};
  if (!std::isfinite(seconds) || seconds < 0) {
    throw std::invalid_argument(
        std::format("{} input delay must be finite and not negative, got {} s", m_owner, seconds));
  }
  m_inputDelay = inputDelay;
  return *this;
}

ADRCConfig& ADRCConfig::WithCascaded(bool cascaded) {
  std::scoped_lock lock{m_mutex};
  m_cascaded = cascaded;
  return *this;
}

ADRCConfig& ADRCConfig::WithTrackingDifferentiator(TrackingDifferentiator::Method method,
                                                   std::vector<double> params) {
  std::scoped_lock lock{m_mutex};
  m_tdMethod = method;
  m_tdParams = std::move(params);
  return *this;
}

ADRCConfig& ADRCConfig::WithoutTrackingDifferentiator() {
  std::scoped_lock lock{m_mutex};
  m_tdMethod.reset();
  m_tdParams.clear();
  return *this;
}

std::optional<ADRCConfig::SystemOrder> ADRCConfig::GetSystemOrder() const {
  std::scoped_lock lock{m_mutex};
  return m_systemOrder;
}
std::optional<wpi::units::second_t> ADRCConfig::GetSettlingTime() const {
  std::scoped_lock lock{m_mutex};
  return m_settlingTime;
}
std::optional<double> ADRCConfig::GetBandwidthScale() const {
  std::scoped_lock lock{m_mutex};
  return m_bandwidthScale;
}
std::optional<double> ADRCConfig::GetB0() const {
  std::scoped_lock lock{m_mutex};
  return m_b0;
}
std::optional<wpi::units::second_t> ADRCConfig::GetInputDelay() const {
  std::scoped_lock lock{m_mutex};
  return m_inputDelay;
}
bool ADRCConfig::IsCascaded() const {
  std::scoped_lock lock{m_mutex};
  return m_cascaded;
}
std::optional<TrackingDifferentiator::Method> ADRCConfig::GetTdMethod() const {
  std::scoped_lock lock{m_mutex};
  return m_tdMethod;
}
std::vector<double> ADRCConfig::GetTdParams() const {
  std::scoped_lock lock{m_mutex};
  return m_tdParams;
}

double ADRCConfig::ResolveB0(const wpi::math::DCMotor& motor,
                             const motorcontrollers::SmartMotorControllerConfig& config) const {
  std::string owner;
  std::optional<double> b0;
  bool elevator = false;
  {
    std::scoped_lock lock{m_mutex};
    b0 = m_b0;
    owner = m_owner;
    elevator = m_systemOrder && IsSpeedPerVolt(*m_systemOrder);
  }
  if (elevator && !config.GetLinearClosedLoopControllerUse()) {
    throw exceptions::SmartMotorControllerConfigurationException(
        owner + " is an elevator's ADRC, but the closed loop is not linear",
        "SystemOrder::ELEVATOR runs in meters, with b0 in meters per second per volt, so the motor config needs a "
        "linear closed loop and a mechanism circumference.",
        "WithLinearClosedLoopController(true) and WithMechanismCircumference(...) on the motor config");
  }
  if (b0) return *b0;
  if (elevator) {
    const auto& gearing = config.GetMotorGearing();
    const double rotorToMechanism = gearing ? gearing->GetMechanismToRotorRatio() : std::nan("");
    if (!std::isfinite(rotorToMechanism) || rotorToMechanism <= 0.0) {
      throw exceptions::SmartMotorControllerConfigurationException(
          std::format("{} cannot derive the elevator's b0 from gear ratio {}", owner, rotorToMechanism),
          "The rotor-to-mechanism ratio must be finite and positive.", "WithMotorGearing(...) with a positive reduction");
    }
    // Kv is in radians per second per volt at the motor; the reduction slows it at the mechanism, and each mechanism
    // rotation moves the carriage one circumference.
    return motor.Kv.value() / (2.0 * std::numbers::pi * rotorToMechanism) *
           config.GetMechanismCircumference()->value();
  }
  if (!config.IsMomentOfInertiaConfigured()) {
    throw exceptions::SmartMotorControllerConfigurationException(
        owner + " has no b0 and cannot derive one",
        "WithB0(...) was never called and the mechanism never configured its moment of inertia, so the library's "
        "placeholder inertia would be used. Inertia is different for every mechanism.",
        "WithADRCController(ADRCConfig{}.WithB0(...)) or call WithMOI(...) on the motor config");
  }
  const auto& gearing = config.GetMotorGearing();
  if (!gearing) {
    throw exceptions::SmartMotorControllerConfigurationException(
        owner + " cannot derive b0 without gearing",
        "No gearing was configured, so the rotor-to-mechanism ratio is unknown.",
        "WithADRCController(ADRCConfig{}.WithB0(...)) or call WithGearing(...) on the motor config");
  }
  // Rotor rotations per mechanism rotation, the ratio b0 needs.
  const double rotorToMechanism = gearing->GetMechanismToRotorRatio();
  if (!std::isfinite(rotorToMechanism) || rotorToMechanism <= 0.0) {
    throw exceptions::SmartMotorControllerConfigurationException(
        std::format("{} cannot derive b0 from gear ratio {}", owner, rotorToMechanism),
        "The rotor-to-mechanism ratio must be finite and positive.", "WithGearing(...) with a positive reduction");
  }
  const double inertia = config.GetMOI().value();
  if (!std::isfinite(inertia) || inertia <= 0.0) {
    throw exceptions::SmartMotorControllerConfigurationException(
        std::format("{} cannot derive b0 from inertia {} kg*m^2", owner, inertia),
        "The mechanism moment of inertia must be finite and positive.", "WithMOI(...) with a real value");
  }
  const double radiansPerSecondSquaredPerVolt = rotorToMechanism * motor.Kt.value() / (motor.R.value() * inertia);
  const double rotationsPerSecondSquaredPerVolt = radiansPerSecondSquaredPerVolt / (2.0 * std::numbers::pi);
  if (config.GetLinearClosedLoopControllerUse()) {
    const auto circumference = config.GetMechanismCircumference();
    if (!circumference) {
      throw exceptions::SmartMotorControllerConfigurationException(
          owner + " cannot derive linear b0 without a mechanism circumference",
          "The linear closed loop needs a circumference to convert rotations to meters.",
          "WithMechanismCircumference(...)");
    }
    return rotationsPerSecondSquaredPerVolt * circumference->value();
  }
  return rotationsPerSecondSquaredPerVolt;
}

wpi::units::volt_t ADRCConfig::ResolveMaxVoltage(const wpi::math::DCMotor& motor,
                                                 const motorcontrollers::SmartMotorControllerConfig& config) {
  if (const auto configured = config.GetClosedLoopControllerMaximumVoltage()) return *configured;
  std::scoped_lock lock{m_mutex};
  if (!m_nominalVoltageAlert) {
    m_nominalVoltageAlert = std::make_unique<wpi::util::Alert>(
        "YAMS", AlertId("NominalVoltage"),
        std::format("{}: the motor config has no closed loop voltage limit (WithClosedLoopMaxVoltage); "
                    "using the motor's nominal voltage ({} V).",
                    m_owner, motor.nominalVoltage.value()),
        wpi::util::Alert::Level::MEDIUM);
  }
  m_nominalVoltageAlert->Set(true);
  return motor.nominalVoltage;
}

ADRCController ADRCConfig::BuildController(const std::shared_ptr<ADRCConfig>& self, const wpi::math::DCMotor& motor,
                                           const motorcontrollers::SmartMotorControllerConfig& config,
                                           wpi::units::second_t period) {
  if (!self) throw std::invalid_argument("The ADRC config must not be null");
  {
    std::scoped_lock lock{self->m_mutex};
    self->m_owner = config.GetTelemetryName() ? *config.GetTelemetryName() + " ADRC" : "ADRC";
    if (!self->m_systemOrder) {
      throw exceptions::SmartMotorControllerConfigurationException(
          self->m_owner + " has no system order",
          "WithSystemOrder(...) was never called, and there is no safe default plant order.",
          "WithADRCController(ADRCConfig{}.WithSystemOrder(ADRCConfig::SystemOrder::FIRST))");
    }
  }
  ADRCController controller{self};
  controller.Initialize(self->ResolveB0(motor, config), self->ResolveMaxVoltage(motor, config), period);
  return controller;
}

void ADRCConfig::SetBuiltValues(double b0, wpi::units::second_t period) {
  std::scoped_lock lock{m_mutex};
  const double seconds = period.value();
  if (!std::isfinite(seconds) || seconds <= 0) {
    throw std::invalid_argument(m_owner + " control period must be finite and positive");
  }
  const double usedB0 = m_b0.value_or(b0);
  if (!std::isfinite(usedB0) || usedB0 == 0.0) {
    throw std::invalid_argument(std::format("{} b0 must be finite and non-zero, got {}", m_owner, usedB0));
  }
  m_builtPeriod = period;
  m_builtB0 = b0;
  RebuildGains();
}

void ADRCConfig::RebuildGains() {
  if (!m_builtPeriod) return;
  const int order = GetOrder(*m_systemOrder);
  const double wc = ADRCController::Bandwidth(order, m_bandwidthScale.value_or(kDefaultBandwidthScale),
                                              m_settlingTime.value_or(kDefaultSettlingTime));
  m_gains = std::make_shared<const ADRCGains>(
      ADRCController::ComputeGains(order, wc, m_b0.value_or(*m_builtB0), m_builtPeriod->value()));
}

ADRCConfig::StepValues ADRCConfig::GetStepValues() const {
  std::scoped_lock lock{m_mutex};
  return StepValues{m_gains, m_builtPeriod->value(), m_cascaded};
}

std::shared_ptr<const ADRCGains> ADRCConfig::GetGains() const {
  std::scoped_lock lock{m_mutex};
  return m_gains;
}

void ADRCConfig::ReportRejected(std::optional<std::string> reason) {
  std::scoped_lock lock{m_mutex};
  if (!reason) {
    if (m_rejectedAlert) m_rejectedAlert->Set(false);
    return;
  }
  if (!m_rejectedAlert) {
    m_rejectedAlert =
        std::make_unique<wpi::util::Alert>("YAMS", AlertId("Rejected"), "", wpi::util::Alert::Level::HIGH);
  }
  m_rejectedAlert->SetText("Not retuned: " + *reason);
  m_rejectedAlert->Set(true);
}

std::string ADRCConfig::AlertId(const std::string& alert) const {
  return std::format("ADRC_{}_{}", m_instance, alert);
}

}  // namespace yams::math
