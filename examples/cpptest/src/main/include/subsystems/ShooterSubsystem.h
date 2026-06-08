// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/system/plant/DCMotor.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <rev/SparkMax.h>
#include <functional>
#include <optional>

#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/mechanisms/config/FlyWheelConfig.hpp"
#include "yams/mechanisms/velocity/FlyWheel.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/local/SparkWrapper.hpp"

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  units::degrees_per_second_t GetVelocity() const;
  bool ReadyToShoot(units::degrees_per_second_t tolerance) const;

  frc2::CommandPtr SetVelocity(units::degrees_per_second_t speed);
  frc2::CommandPtr SetVelocity(std::function<units::degrees_per_second_t()> speed);
  frc2::CommandPtr Set(double dutyCycle);
  frc2::CommandPtr Set(std::function<double()> dutyCycle);

  void SetVelocitySetpoint(units::degrees_per_second_t speed);
  void SetDutyCycleSetpoint(double dutyCycle);
  void SetSurfaceSpeedSetpoint(units::meters_per_second_t speed);

  void Periodic() override;
  void SimulationPeriodic() override;

 private:
 rev::spark::SparkMax m_flywheelMotor1{3, rev::spark::SparkMax::MotorType::kBrushless};
 rev::spark::SparkMax m_flywheelMotor2{4, rev::spark::SparkMax::MotorType::kBrushless};

  yams::motorcontrollers::SmartMotorControllerConfig m_motorConfig;
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_motor;

  yams::mechanisms::config::FlyWheelConfig m_shooterConfig;
  std::optional<yams::mechanisms::velocity::FlyWheel> m_shooter;
};
