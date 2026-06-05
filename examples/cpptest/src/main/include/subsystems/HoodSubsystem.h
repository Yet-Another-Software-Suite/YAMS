// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/system/plant/DCMotor.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/voltage.h>

#include <optional>

#include <ctre/phoenix6/TalonFXS.hpp>

#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/mechanisms/config/PivotConfig.hpp"
#include "yams/mechanisms/positional/Pivot.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/remote/TalonFXSWrapper.hpp"

class HoodSubsystem : public frc2::SubsystemBase {
 public:
  HoodSubsystem();

  void SetAngleSetpoint(units::degree_t angle);

  frc2::CommandPtr HoodCmd(double dutycycle);
  frc2::CommandPtr SysId();
  frc2::CommandPtr SetAngle(units::degree_t angle);

  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  ctre::phoenix6::hardware::TalonFXS m_hoodMotor{9};

  yams::motorcontrollers::SmartMotorControllerConfig m_motorConfig;
  std::optional<yams::motorcontrollers::remote::TalonFXSWrapper> m_motor;

  yams::mechanisms::config::PivotConfig m_pivotConfig;
  std::optional<yams::mechanisms::positional::Pivot> m_hood;
};
