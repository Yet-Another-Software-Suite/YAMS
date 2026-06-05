// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/system/plant/DCMotor.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/time.h>
#include <units/voltage.h>

#include <functional>
#include <optional>

#include <ctre/phoenix6/TalonFX.hpp>

#include "yams/gearing/GearBox.h"
#include "yams/gearing/MechanismGearing.h"
#include "yams/mechanisms/config/FlyWheelConfig.h"
#include "yams/mechanisms/velocity/FlyWheel.h"
#include "yams/motorcontrollers/SmartMotorControllerConfig.h"
#include "yams/motorcontrollers/remote/TalonFXWrapper.h"

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
  ctre::phoenix6::hardware::TalonFX m_flywheelMotor1{1};
  ctre::phoenix6::hardware::TalonFX m_flywheelMotor2{2};

  yams::motorcontrollers::SmartMotorControllerConfig m_motorConfig;
  std::optional<yams::motorcontrollers::remote::TalonFXWrapper> m_motor;

  yams::mechanisms::config::FlyWheelConfig m_shooterConfig;
  std::optional<yams::mechanisms::velocity::FlyWheel> m_shooter;
};
