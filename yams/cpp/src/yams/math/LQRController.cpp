// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/math/LQRController.hpp"

#include <frc/EigenCore.h>

#include <memory>
#include <utility>

namespace yams::math {

// ── Private factories ────────────────────────────────────────────────────────

std::unique_ptr<LQRController::FlywheelBundle> LQRController::BuildFlywheel(const LQRConfig& cfg) {
  auto plant = cfg.m_flywheelPlant.value();
  frc::LinearQuadraticRegulator<1, 1> ctrl{plant,
                                           {cfg.m_qElems.empty() ? 3.0 : cfg.m_qElems[0]},
                                           {cfg.m_rElems.empty() ? 12.0 : cfg.m_rElems[0]},
                                           cfg.m_period};
  frc::KalmanFilter<1, 1, 1> obs{plant,
                                 {cfg.m_stateStdDevs.empty() ? 3.0 : cfg.m_stateStdDevs[0]},
                                 {cfg.m_measStdDevs.empty() ? 0.01 : cfg.m_measStdDevs[0]},
                                 cfg.m_period};
  return std::make_unique<FlywheelBundle>(plant, std::move(ctrl), std::move(obs), cfg.m_maxVoltage,
                                          cfg.m_period);
}

std::unique_ptr<LQRController::ArmElevatorBundle> LQRController::BuildArmElevator(
    const LQRConfig& cfg) {
  auto plant = cfg.m_armElevatorPlant.value();
  frc::LinearQuadraticRegulator<2, 1> ctrl{plant,
                                           {cfg.m_qElems.size() > 0 ? cfg.m_qElems[0] : 0.01,
                                            cfg.m_qElems.size() > 1 ? cfg.m_qElems[1] : 0.01},
                                           {cfg.m_rElems.empty() ? 12.0 : cfg.m_rElems[0]},
                                           cfg.m_period};
  frc::KalmanFilter<2, 1, 1> obs{plant,
                                 {cfg.m_stateStdDevs.size() > 0 ? cfg.m_stateStdDevs[0] : 0.01,
                                  cfg.m_stateStdDevs.size() > 1 ? cfg.m_stateStdDevs[1] : 0.01},
                                 {cfg.m_measStdDevs.empty() ? 0.0001 : cfg.m_measStdDevs[0]},
                                 cfg.m_period};
  return std::make_unique<ArmElevatorBundle>(plant, std::move(ctrl), std::move(obs),
                                             cfg.m_maxVoltage, cfg.m_period);
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
  m_config = config;
  m_type = config.GetType();
  if (m_type == LQRConfig::LQRType::FLYWHEEL) {
    m_bundle = BuildFlywheel(config);
  } else {
    m_bundle = BuildArmElevator(config);
  }
  m_period = config.GetPeriod();
}

// ── Reset ────────────────────────────────────────────────────────────────────

void LQRController::Reset(units::radian_t angle, units::radians_per_second_t velocity) {
  if (m_type == LQRConfig::LQRType::FLYWHEEL) {
    frc::Vectord<1> v;
    v << velocity.value();
    std::get<std::unique_ptr<FlywheelBundle>>(m_bundle)->loop.Reset(v);
  } else {
    frc::Vectord<2> v;
    v << angle.value(), velocity.value();
    std::get<std::unique_ptr<ArmElevatorBundle>>(m_bundle)->loop.Reset(v);
  }
}

void LQRController::Reset(units::meter_t distance, units::meters_per_second_t velocity) {
  if (m_type == LQRConfig::LQRType::ELEVATOR) {
    frc::Vectord<2> v;
    v << distance.value(), velocity.value();
    std::get<std::unique_ptr<ArmElevatorBundle>>(m_bundle)->loop.Reset(v);
  }
}

// ── Calculate ────────────────────────────────────────────────────────────────

units::volt_t LQRController::Calculate(units::radian_t measured, units::radian_t position,
                                       units::radians_per_second_t velocity) {
  auto& loop = std::get<std::unique_ptr<ArmElevatorBundle>>(m_bundle)->loop;
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
  auto& loop = std::get<std::unique_ptr<ArmElevatorBundle>>(m_bundle)->loop;
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
  auto& loop = std::get<std::unique_ptr<FlywheelBundle>>(m_bundle)->loop;
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
  auto& loop = std::get<std::unique_ptr<FlywheelBundle>>(m_bundle)->loop;
  frc::Vectord<1> nextR;
  nextR << velocity.value();
  loop.SetNextR(nextR);
  frc::Vectord<1> meas;
  meas << measured.value();
  loop.Correct(meas);
  loop.Predict(m_period);
  return units::volt_t{loop.U(0)};
}

// ── Accessors ────────────────────────────────────────────────────────────────

LQRConfig::LQRType LQRController::GetType() const { return m_type; }
const std::optional<LQRConfig>& LQRController::GetConfig() const { return m_config; }

}  // namespace yams::math
