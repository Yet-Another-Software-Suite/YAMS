// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/LinearSystemLoop.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <units/time.h>
#include <units/voltage.h>

#include <optional>
#include <variant>
#include <vector>

namespace yams::math {

class LQRConfig {
 public:
  enum class LQRType {
    FLYWHEEL,
    ARM,
    ELEVATOR,
  };

  using Loop1 = frc::LinearSystemLoop<1, 1, 1>;
  using Loop2 = frc::LinearSystemLoop<2, 1, 1>;

  LQRConfig& WithType(LQRType type);
  LQRConfig& WithFlywheelSystem(const frc::DCMotor& motor, double momentOfInertia, double gearing);
  LQRConfig& WithArmSystem(const frc::DCMotor& motor, double momentOfInertia, double gearing);
  LQRConfig& WithElevatorSystem(const frc::DCMotor& motor, double mass, double drumRadius,
                                double gearing);

  LQRConfig& WithQElems(std::initializer_list<double> qElems);
  LQRConfig& WithRElems(std::initializer_list<double> rElems);
  LQRConfig& WithStateStdDevs(std::initializer_list<double> stateStdDevs);
  LQRConfig& WithMeasurementStdDevs(std::initializer_list<double> measStdDevs);

  LQRConfig& WithMaxVoltage(units::volt_t maxVoltage);
  LQRConfig& WithPeriod(units::second_t period);

  LQRType GetType() const;
  units::second_t GetPeriod() const;
  units::volt_t GetMaxVoltage() const;

  std::variant<frc::LinearSystem<1, 1, 1>, frc::LinearSystem<2, 1, 1>> GetSystem() const;

  std::variant<Loop1, Loop2> GetLoop() const;

 private:
  std::optional<LQRType> m_type;
  std::optional<frc::LinearSystem<1, 1, 1>> m_flywheelPlant;
  std::optional<frc::LinearSystem<2, 1, 1>> m_armElevatorPlant;

  std::vector<double> m_qElems;
  std::vector<double> m_rElems;
  std::vector<double> m_stateStdDevs;
  std::vector<double> m_measStdDevs;

  units::volt_t m_maxVoltage{12.0_V};
  units::second_t m_period{0.020_s};
};

}  // namespace yams::math
