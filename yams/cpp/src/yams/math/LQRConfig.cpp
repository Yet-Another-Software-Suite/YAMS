// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/math/LQRConfig.hpp"

#include <frc/EigenCore.h>
#include <frc/system/plant/LinearSystemId.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>

#include <stdexcept>
#include <vector>

namespace yams::math {

LQRConfig& LQRConfig::WithType(LQRType type) {
  m_type = type;
  return *this;
}

LQRConfig& LQRConfig::WithFlywheelSystem(const frc::DCMotor& motor, double momentOfInertia,
                                         double gearing) {
  m_flywheelPlant = frc::LinearSystemId::FlywheelSystem(
      motor, units::kilogram_square_meter_t{momentOfInertia}, gearing);
  m_type = LQRType::FLYWHEEL;
  return *this;
}

LQRConfig& LQRConfig::WithArmSystem(const frc::DCMotor& motor, double momentOfInertia,
                                    double gearing) {
  // Build full 2-output plant, then reduce to position-only output to match
  // the Java LinearSystem<N2,N1,N1> behaviour.
  auto full = frc::LinearSystemId::SingleJointedArmSystem(
      motor, units::kilogram_square_meter_t{momentOfInertia}, gearing);
  m_armElevatorPlant =
      frc::LinearSystem<2, 1, 1>{full.A(), full.B(), (frc::Matrixd<1, 2>() << 1.0, 0.0).finished(),
                                 frc::Matrixd<1, 1>::Zero()};
  m_type = LQRType::ARM;
  return *this;
}

LQRConfig& LQRConfig::WithElevatorSystem(const frc::DCMotor& motor, double mass, double drumRadius,
                                         double gearing) {
  // Same reduction to position-only output.
  auto full = frc::LinearSystemId::ElevatorSystem(motor, units::kilogram_t{mass},
                                                  units::meter_t{drumRadius}, gearing);
  m_armElevatorPlant =
      frc::LinearSystem<2, 1, 1>{full.A(), full.B(), (frc::Matrixd<1, 2>() << 1.0, 0.0).finished(),
                                 frc::Matrixd<1, 1>::Zero()};
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
LQRConfig& LQRConfig::WithMaxVoltage(units::volt_t maxVoltage) {
  m_maxVoltage = maxVoltage;
  return *this;
}
LQRConfig& LQRConfig::WithPeriod(units::second_t period) {
  m_period = period;
  return *this;
}

LQRConfig::LQRType LQRConfig::GetType() const { return m_type.value(); }
units::second_t LQRConfig::GetPeriod() const { return m_period; }
units::volt_t LQRConfig::GetMaxVoltage() const { return m_maxVoltage; }

std::variant<frc::LinearSystem<1, 1, 1>, frc::LinearSystem<2, 1, 1>> LQRConfig::GetSystem() const {
  if (m_type.value() == LQRType::FLYWHEEL) return m_flywheelPlant.value();
  return m_armElevatorPlant.value();
}

std::variant<LQRConfig::Loop1, LQRConfig::Loop2> LQRConfig::GetLoop() const {
  auto type = m_type.value();
  if (type == LQRType::FLYWHEEL) {
    auto plant = m_flywheelPlant.value();
    frc::LinearQuadraticRegulator<1, 1> controller{plant,
                                                   {m_qElems.empty() ? 3.0 : m_qElems[0]},
                                                   {m_rElems.empty() ? 12.0 : m_rElems[0]},
                                                   m_period};
    frc::KalmanFilter<1, 1, 1> observer{plant,
                                        {m_stateStdDevs.empty() ? 3.0 : m_stateStdDevs[0]},
                                        {m_measStdDevs.empty() ? 0.01 : m_measStdDevs[0]},
                                        m_period};
    return Loop1{plant, controller, observer, m_maxVoltage, m_period};
  } else {
    auto plant = m_armElevatorPlant.value();
    frc::LinearQuadraticRegulator<2, 1> controller{
        plant,
        {m_qElems.size() > 0 ? m_qElems[0] : 0.01, m_qElems.size() > 1 ? m_qElems[1] : 0.01},
        {m_rElems.empty() ? 12.0 : m_rElems[0]},
        m_period};
    frc::KalmanFilter<2, 1, 1> observer{plant,
                                        {m_stateStdDevs.size() > 0 ? m_stateStdDevs[0] : 0.01,
                                         m_stateStdDevs.size() > 1 ? m_stateStdDevs[1] : 0.01},
                                        {m_measStdDevs.empty() ? 0.0001 : m_measStdDevs[0]},
                                        m_period};
    return Loop2{plant, controller, observer, m_maxVoltage, m_period};
  }
}

}  // namespace yams::math
