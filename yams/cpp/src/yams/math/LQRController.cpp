// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/math/LQRController.hpp"

#include <frc/EigenCore.h>

#include <utility>

namespace yams::math {

LQRController::LQRController(LQRConfig::LQRType type,
                             std::variant<LQRConfig::Loop1, LQRConfig::Loop2> loop,
                             units::second_t period)
    : m_type(type), m_loop(std::move(loop)), m_period(period) {}

LQRController::LQRController(LQRConfig config)
    : m_config(config),
      m_type(config.GetType()),
      m_loop(config.GetLoop()),
      m_period(config.GetPeriod()) {}

void LQRController::UpdateConfig(LQRConfig config) {
  m_config = config;
  m_type = config.GetType();
  m_loop = config.GetLoop();
  m_period = config.GetPeriod();
}

void LQRController::Reset(units::radian_t angle, units::radians_per_second_t velocity) {
  if (m_type == LQRConfig::LQRType::FLYWHEEL) {
    auto& loop = std::get<LQRConfig::Loop1>(m_loop);
    frc::Vectord<1> v;
    v << velocity.value();
    loop.Reset(v);
  } else {
    auto& loop = std::get<LQRConfig::Loop2>(m_loop);
    frc::Vectord<2> v;
    v << angle.value(), velocity.value();
    loop.Reset(v);
  }
}

void LQRController::Reset(units::meter_t distance, units::meters_per_second_t velocity) {
  if (m_type == LQRConfig::LQRType::ELEVATOR) {
    auto& loop = std::get<LQRConfig::Loop2>(m_loop);
    frc::Vectord<2> v;
    v << distance.value(), velocity.value();
    loop.Reset(v);
  }
}

units::volt_t LQRController::Calculate(units::radian_t measured, units::radian_t position,
                                       units::radians_per_second_t velocity) {
  auto& loop = std::get<LQRConfig::Loop2>(m_loop);
  frc::Vectord<2> nextR;
  nextR << position.value(), velocity.value();
  loop.SetNextR(nextR);
  frc::Vectord<1> meas;
  meas << measured.value();
  loop.Correct(meas);
  loop.Predict(m_period);
  return units::volt_t{loop.U(0)};
}

units::volt_t LQRController::Calculate(units::meter_t measured, units::meter_t position,
                                       units::meters_per_second_t velocity) {
  auto& loop = std::get<LQRConfig::Loop2>(m_loop);
  frc::Vectord<2> nextR;
  nextR << position.value(), velocity.value();
  loop.SetNextR(nextR);
  frc::Vectord<1> meas;
  meas << measured.value();
  loop.Correct(meas);
  loop.Predict(m_period);
  return units::volt_t{loop.U(0)};
}

units::volt_t LQRController::Calculate(units::radians_per_second_t measured,
                                       units::radians_per_second_t velocity) {
  auto& loop = std::get<LQRConfig::Loop1>(m_loop);
  frc::Vectord<1> nextR;
  nextR << velocity.value();
  loop.SetNextR(nextR);
  frc::Vectord<1> meas;
  meas << measured.value();
  loop.Correct(meas);
  loop.Predict(m_period);
  return units::volt_t{loop.U(0)};
}

units::volt_t LQRController::Calculate(units::meters_per_second_t measured,
                                       units::meters_per_second_t velocity) {
  auto& loop = std::get<LQRConfig::Loop1>(m_loop);
  frc::Vectord<1> nextR;
  nextR << velocity.value();
  loop.SetNextR(nextR);
  frc::Vectord<1> meas;
  meas << measured.value();
  loop.Correct(meas);
  loop.Predict(m_period);
  return units::volt_t{loop.U(0)};
}

LQRConfig::LQRType LQRController::GetType() const { return m_type; }
const std::optional<LQRConfig>& LQRController::GetConfig() const { return m_config; }

}  // namespace yams::math
