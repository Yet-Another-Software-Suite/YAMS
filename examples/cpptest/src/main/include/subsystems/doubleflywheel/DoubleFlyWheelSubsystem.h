// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/system/plant/DCMotor.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <functional>
#include <optional>

#include <ctre/phoenix6/TalonFX.hpp>

#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/remote/TalonFXWrapper.hpp"

class DoubleFlyWheelSubsystem : public frc2::SubsystemBase {
 public:
  DoubleFlyWheelSubsystem();

  frc2::CommandPtr SetDutyCycle(double lower, double upper);
  frc2::CommandPtr SetVoltage(units::volt_t lower, units::volt_t upper);
  frc2::CommandPtr SetVelocity(units::degrees_per_second_t lower,
                               units::degrees_per_second_t upper);
  frc2::CommandPtr SetVelocity(std::function<units::degrees_per_second_t()> lower,
                               std::function<units::degrees_per_second_t()> upper);

  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  ctre::phoenix6::hardware::TalonFX m_talonLower{4};
  ctre::phoenix6::hardware::TalonFX m_talonUpper{6};

  yams::motorcontrollers::SmartMotorControllerConfig m_lowerConfig;
  yams::motorcontrollers::SmartMotorControllerConfig m_upperConfig;

  std::optional<yams::motorcontrollers::remote::TalonFXWrapper> m_lowerFlyWheel;
  std::optional<yams::motorcontrollers::remote::TalonFXWrapper> m_upperFlyWheel;
};
