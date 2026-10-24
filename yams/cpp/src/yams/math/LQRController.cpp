// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/math/LQRController.hpp"

#include <memory>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <wpi/math/linalg/EigenCore.hpp>

namespace yams::math {

// ── Private factories ────────────────────────────────────────────────────────

std::unique_ptr<LQRController::FlywheelBundle> LQRController::BuildFlywheel(const LQRConfig& cfg) {
  return std::get<std::unique_ptr<FlywheelBundle>>(cfg.GetLoop());
}

std::unique_ptr<LQRController::ArmElevatorBundle> LQRController::BuildArmElevator(
    const LQRConfig& cfg) {
  return std::get<std::unique_ptr<ArmElevatorBundle>>(cfg.GetLoop());
}

// ── Constructors ─────────────────────────────────────────────────────────────

LQRController::LQRController(LQRConfig config)
    : m_config(config),
      m_type(config.GetType()),
      m_bundle(config.GetType() == LQRConfig::LQRType::FLYWHEEL
                   ? std::variant<std::unique_ptr<FlywheelBundle>,
                                  std::unique_ptr<ArmElevatorBundle>>{BuildFlywheel(config)}
                   : std::variant<std::unique_ptr<FlywheelBundle>,
                                  std::unique_ptr<ArmElevatorBundle>>{BuildArmElevator(config)}),
      m_period(config.GetPeriod()) {}

void LQRController::UpdateConfig(LQRConfig config) {
  std::lock_guard lock{*m_mutex};
  m_config = config;
  m_type = config.GetType();
  if (m_type == LQRConfig::LQRType::FLYWHEEL) {
    m_bundle = BuildFlywheel(config);
  } else {
    m_bundle = BuildArmElevator(config);
  }
  m_period = config.GetPeriod();
}

void LQRController::Retune(LQRConfig config) {
  if (config.GetType() != m_type) {
    throw std::invalid_argument("Cannot retune an LQR with a config for another mechanism type");
  }
  // Built outside the lock: solving the LQR takes a while, and the closed loop keeps running meanwhile.
  if (m_type == LQRConfig::LQRType::FLYWHEEL) {
    auto next = BuildFlywheel(config);
    std::lock_guard lock{*m_mutex};
    next->loop.Reset(std::get<std::unique_ptr<FlywheelBundle>>(m_bundle)->loop.Xhat());
    m_bundle = std::move(next);
  } else {
    auto next = BuildArmElevator(config);
    std::lock_guard lock{*m_mutex};
    next->loop.Reset(std::get<std::unique_ptr<ArmElevatorBundle>>(m_bundle)->loop.Xhat());
    m_bundle = std::move(next);
  }
  std::lock_guard lock{*m_mutex};
  m_config = config;
  m_period = config.GetPeriod();
}

// ── Reset ────────────────────────────────────────────────────────────────────

void LQRController::Reset(wpi::units::radian_t angle, wpi::units::radians_per_second_t velocity) {
  std::lock_guard lock{*m_mutex};
  if (m_type == LQRConfig::LQRType::FLYWHEEL) {
    wpi::math::Vectord<1> v;
    v << velocity.value();
    std::get<std::unique_ptr<FlywheelBundle>>(m_bundle)->loop.Reset(v);
  } else {
    wpi::math::Vectord<2> v;
    v << angle.value(), velocity.value();
    std::get<std::unique_ptr<ArmElevatorBundle>>(m_bundle)->loop.Reset(v);
  }
}

void LQRController::Reset(wpi::units::meter_t distance, wpi::units::meters_per_second_t velocity) {
  std::lock_guard lock{*m_mutex};
  if (m_type == LQRConfig::LQRType::ELEVATOR) {
    wpi::math::Vectord<2> v;
    v << distance.value(), velocity.value();
    std::get<std::unique_ptr<ArmElevatorBundle>>(m_bundle)->loop.Reset(v);
  }
}

// ── Calculate ────────────────────────────────────────────────────────────────

wpi::units::volt_t LQRController::Calculate(wpi::units::radian_t measured,
                                            wpi::units::radian_t position,
                                            wpi::units::radians_per_second_t velocity) {
  std::lock_guard lock{*m_mutex};
  auto& loop = std::get<std::unique_ptr<ArmElevatorBundle>>(m_bundle)->loop;
  wpi::math::Vectord<2> nextR;
  nextR << position.value(), velocity.value();
  loop.SetNextR(nextR);
  wpi::math::Vectord<1> meas;
  meas << measured.value();
  loop.Correct(meas);
  loop.Predict(m_period);
  return wpi::units::volt_t{loop.U(0)};
}

wpi::units::volt_t LQRController::Calculate(wpi::units::meter_t measured,
                                            wpi::units::meter_t position,
                                            wpi::units::meters_per_second_t velocity) {
  std::lock_guard lock{*m_mutex};
  auto& loop = std::get<std::unique_ptr<ArmElevatorBundle>>(m_bundle)->loop;
  wpi::math::Vectord<2> nextR;
  nextR << position.value(), velocity.value();
  loop.SetNextR(nextR);
  wpi::math::Vectord<1> meas;
  meas << measured.value();
  loop.Correct(meas);
  loop.Predict(m_period);
  return wpi::units::volt_t{loop.U(0)};
}

wpi::units::volt_t LQRController::Calculate(wpi::units::radians_per_second_t measured,
                                            wpi::units::radians_per_second_t velocity) {
  std::lock_guard lock{*m_mutex};
  auto& loop = std::get<std::unique_ptr<FlywheelBundle>>(m_bundle)->loop;
  wpi::math::Vectord<1> nextR;
  nextR << velocity.value();
  loop.SetNextR(nextR);
  wpi::math::Vectord<1> meas;
  meas << measured.value();
  loop.Correct(meas);
  loop.Predict(m_period);
  return wpi::units::volt_t{loop.U(0)};
}

wpi::units::volt_t LQRController::Calculate(wpi::units::meters_per_second_t measured,
                                            wpi::units::meters_per_second_t velocity) {
  std::lock_guard lock{*m_mutex};
  auto& loop = std::get<std::unique_ptr<FlywheelBundle>>(m_bundle)->loop;
  wpi::math::Vectord<1> nextR;
  nextR << velocity.value();
  loop.SetNextR(nextR);
  wpi::math::Vectord<1> meas;
  meas << measured.value();
  loop.Correct(meas);
  loop.Predict(m_period);
  return wpi::units::volt_t{loop.U(0)};
}

// ── Accessors ────────────────────────────────────────────────────────────────

LQRConfig::LQRType LQRController::GetType() const { return m_type; }
const std::optional<LQRConfig>& LQRController::GetConfig() const { return m_config; }

}  // namespace yams::math
