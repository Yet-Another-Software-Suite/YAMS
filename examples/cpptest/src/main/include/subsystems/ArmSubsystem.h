// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DigitalInput.h>
#include <frc/system/plant/DCMotor.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/voltage.h>

#include <optional>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include "yams/gearing/GearBox.h"
#include "yams/gearing/MechanismGearing.h"
#include "yams/mechanisms/config/ArmConfig.h"
#include "yams/mechanisms/positional/Arm.h"
#include "yams/motorcontrollers/SmartMotorControllerConfig.h"
#include "yams/motorcontrollers/remote/TalonFXWrapper.h"

class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem();

  bool GetBeamBreak();

  frc2::CommandPtr ArmCmd(double dutycycle);
  frc2::CommandPtr SysId();
  frc2::CommandPtr SetAngle(units::degree_t angle);

  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  ctre::phoenix6::hardware::CANcoder m_cancoder{2};
  ctre::phoenix6::hardware::TalonFX m_armMotor{1};

  yams::motorcontrollers::SmartMotorControllerConfig m_motorConfig;
  std::optional<yams::motorcontrollers::remote::TalonFXWrapper> m_motor;

  yams::mechanisms::config::ArmConfig m_armConfig;
  std::optional<yams::mechanisms::positional::Arm> m_arm;

  frc::DigitalInput m_dio{0};
};
