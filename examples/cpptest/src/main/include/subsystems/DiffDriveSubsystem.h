// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/drive/DifferentialDrive.h>
#include <frc/system/plant/DCMotor.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <functional>
#include <optional>

#include <rev/SparkMax.h>

#include "yams/gearing/GearBox.h"
#include "yams/gearing/MechanismGearing.h"
#include "yams/motorcontrollers/SmartMotorControllerConfig.h"
#include "yams/motorcontrollers/local/SparkWrapper.h"

class DiffDriveSubsystem : public frc2::SubsystemBase {
 public:
  DiffDriveSubsystem();

  frc2::CommandPtr Stop();
  frc2::CommandPtr TankDrive(std::function<double()> left, std::function<double()> right);
  frc2::CommandPtr ArcadeDrive(std::function<double()> xSpeed,
                               std::function<double()> zRotation);

  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  rev::spark::SparkMax m_leftMotor{21, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_rightMotor{24, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_leftFollowerMotor{22, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_rightFollowerMotor{23, rev::spark::SparkMax::MotorType::kBrushless};

  yams::motorcontrollers::SmartMotorControllerConfig m_leftConfig;
  yams::motorcontrollers::SmartMotorControllerConfig m_rightConfig;

  std::optional<yams::motorcontrollers::local::SparkWrapper> m_leftSMC;
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_rightSMC;

  std::optional<frc::DifferentialDrive> m_drive;
};
