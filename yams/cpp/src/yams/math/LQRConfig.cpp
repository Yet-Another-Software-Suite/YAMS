// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/math/LQRConfig.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>
#include <wpi/math/linalg/EigenCore.hpp>
#include <wpi/math/system/Models.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/mass.hpp>
#include <wpi/units/moment_of_inertia.hpp>

namespace yams::math {

LQRConfig& LQRConfig::WithType(LQRType type) {
  m_type = type;
  return *this;
}

LQRConfig& LQRConfig::WithFlywheelSystem(const wpi::math::DCMotor& motor, double momentOfInertia,
                                         double gearing) {
  m_flywheelPlant = wpi::math::Models::FlywheelFromPhysicalConstants(
      motor, wpi::units::kilogram_square_meter_t{momentOfInertia}, gearing);
  m_type = LQRType::FLYWHEEL;
  return *this;
}

LQRConfig& LQRConfig::WithArmSystem(const wpi::math::DCMotor& motor, double momentOfInertia,
                                    double gearing) {
  // Build full 2-output plant, then reduce to position-only output to match
  // the Java LinearSystem<N2,N1,N1> behaviour.
  auto full = wpi::math::Models::SingleJointedArmFromPhysicalConstants(
      motor, wpi::units::kilogram_square_meter_t{momentOfInertia}, gearing);
  m_armElevatorPlant = wpi::math::LinearSystem<2, 1, 1>{
      full.A(), full.B(), (wpi::math::Matrixd<1, 2>() << 1.0, 0.0).finished(),
      wpi::math::Matrixd<1, 1>::Zero()};
  m_type = LQRType::ARM;
  return *this;
}

LQRConfig& LQRConfig::WithElevatorSystem(const wpi::math::DCMotor& motor, double mass,
                                         double drumRadius, double gearing) {
  // Same reduction to position-only output.
  auto full = wpi::math::Models::ElevatorFromPhysicalConstants(
      motor, wpi::units::kilogram_t{mass}, wpi::units::meter_t{drumRadius}, gearing);
  m_armElevatorPlant = wpi::math::LinearSystem<2, 1, 1>{
      full.A(), full.B(), (wpi::math::Matrixd<1, 2>() << 1.0, 0.0).finished(),
      wpi::math::Matrixd<1, 1>::Zero()};
  m_type = LQRType::ELEVATOR;
  return *this;
}

LQRConfig& LQRConfig::WithQElems(std::initializer_list<double> qElems) {
  m_qElems = std::vector<double>(qElems);
  return *this;
}
LQRConfig& LQRConfig::WithRElems(std::initializer_list<double> rElems) {
  m_rElems = std::vector<double>(rElems);
  return *this;
}
LQRConfig& LQRConfig::WithStateStdDevs(std::initializer_list<double> stateStdDevs) {
  m_stateStdDevs = std::vector<double>(stateStdDevs);
  return *this;
}
LQRConfig& LQRConfig::WithMeasurementStdDevs(std::initializer_list<double> measStdDevs) {
  m_measStdDevs = std::vector<double>(measStdDevs);
  return *this;
}
LQRConfig& LQRConfig::WithMaxVoltage(wpi::units::volt_t maxVoltage) {
  m_maxVoltage = maxVoltage;
  return *this;
}
LQRConfig& LQRConfig::WithPeriod(wpi::units::second_t period) {
  m_period = period;
  return *this;
}

std::vector<double> LQRConfig::GetQElems() const {
  if (m_type.value() == LQRType::FLYWHEEL) return {m_qElems.empty() ? 3.0 : m_qElems[0]};
  return {m_qElems.size() > 0 ? m_qElems[0] : 0.01, m_qElems.size() > 1 ? m_qElems[1] : 0.01};
}
std::vector<double> LQRConfig::GetRElems() const { return {m_rElems.empty() ? 12.0 : m_rElems[0]}; }
std::vector<double> LQRConfig::GetStateStdDevs() const {
  if (m_type.value() == LQRType::FLYWHEEL) return {m_stateStdDevs.empty() ? 3.0 : m_stateStdDevs[0]};
  return {m_stateStdDevs.size() > 0 ? m_stateStdDevs[0] : 0.01,
          m_stateStdDevs.size() > 1 ? m_stateStdDevs[1] : 0.01};
}
std::vector<double> LQRConfig::GetMeasurementStdDevs() const {
  if (m_type.value() == LQRType::FLYWHEEL) return {m_measStdDevs.empty() ? 0.01 : m_measStdDevs[0]};
  return {m_measStdDevs.empty() ? 0.0001 : m_measStdDevs[0]};
}

std::vector<double> LQRConfig::Checked(const std::vector<double>& values, size_t size, const char* name) {
  if (values.size() != size) {
    throw std::invalid_argument("Expected " + std::to_string(size) + " " + name + " for this LQR, got " +
                                std::to_string(values.size()));
  }
  for (double v : values) {
    if (!(v > 0) || !std::isfinite(v)) {
      throw std::invalid_argument(std::string{"The "} + name + " must be above 0, got " + std::to_string(v));
    }
  }
  return values;
}

LQRConfig& LQRConfig::WithQElems(const std::vector<double>& qElems) {
  m_qElems = Checked(qElems, GetQElems().size(), "state error tolerances");
  return *this;
}
LQRConfig& LQRConfig::WithRElems(const std::vector<double>& rElems) {
  m_rElems = Checked(rElems, 1, "control effort tolerances");
  return *this;
}
LQRConfig& LQRConfig::WithStateStdDevs(const std::vector<double>& stateStdDevs) {
  m_stateStdDevs = Checked(stateStdDevs, GetStateStdDevs().size(), "model standard deviations");
  return *this;
}
LQRConfig& LQRConfig::WithMeasurementStdDevs(const std::vector<double>& measStdDevs) {
  m_measStdDevs = Checked(measStdDevs, 1, "encoder standard deviations");
  return *this;
}

LQRConfig::LQRType LQRConfig::GetType() const { return m_type.value(); }
wpi::units::second_t LQRConfig::GetPeriod() const { return m_period; }
wpi::units::volt_t LQRConfig::GetMaxVoltage() const { return m_maxVoltage; }

std::variant<wpi::math::LinearSystem<1, 1, 1>, wpi::math::LinearSystem<2, 1, 1>>
LQRConfig::GetSystem() const {
  if (m_type.value() == LQRType::FLYWHEEL) return m_flywheelPlant.value();
  return m_armElevatorPlant.value();
}

std::variant<std::unique_ptr<LQRConfig::LoopBundle<1>>, std::unique_ptr<LQRConfig::LoopBundle<2>>>
LQRConfig::GetLoop() const {
  const auto q = GetQElems();
  const auto model = GetStateStdDevs();
  const wpi::util::array<double, 1> r{GetRElems()[0]};
  const wpi::util::array<double, 1> encoder{GetMeasurementStdDevs()[0]};
  if (m_type.value() == LQRType::FLYWHEEL) {
    return std::make_unique<LoopBundle<1>>(m_flywheelPlant.value(), wpi::util::array<double, 1>{q[0]}, r,
                                           wpi::util::array<double, 1>{model[0]}, encoder, m_maxVoltage, m_period);
  }
  return std::make_unique<LoopBundle<2>>(m_armElevatorPlant.value(), wpi::util::array<double, 2>{q[0], q[1]}, r,
                                         wpi::util::array<double, 2>{model[0], model[1]}, encoder, m_maxVoltage,
                                         m_period);
}

}  // namespace yams::math
