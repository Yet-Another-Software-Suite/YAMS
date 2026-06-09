// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/DigitalInput.h>
#include <frc/system/plant/DCMotor.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <optional>

#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/mechanisms/config/ArmConfig.hpp"
#include "yams/mechanisms/positional/Arm.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/remote/TalonFXWrapper.hpp"

class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem();

  bool GetBeamBreak();

  frc2::CommandPtr ArmCmd(double dutycycle);
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
