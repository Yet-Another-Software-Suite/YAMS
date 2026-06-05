// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/system/plant/DCMotor.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/length.h>
#include <units/time.h>
#include <units/voltage.h>

#include <optional>

#include <rev/SparkMax.h>

#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/mechanisms/config/ElevatorConfig.hpp"
#include "yams/mechanisms/positional/Elevator.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/local/SparkWrapper.hpp"

class ElevatorSubsystem : public frc2::SubsystemBase {
 public:
  ElevatorSubsystem();

  frc2::CommandPtr ElevCmd(double dutycycle);
  frc2::CommandPtr SetHeight(units::meter_t height);
  frc2::CommandPtr SysId();

  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  rev::spark::SparkMax m_elevatorMotor{2, rev::spark::SparkMax::MotorType::kBrushless};

  yams::motorcontrollers::SmartMotorControllerConfig m_motorConfig;
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_motor;

  yams::mechanisms::config::ElevatorConfig m_elevatorConfig;
  std::optional<yams::mechanisms::positional::Elevator> m_elevator;
};
