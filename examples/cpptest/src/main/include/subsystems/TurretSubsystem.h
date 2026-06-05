// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/system/plant/DCMotor.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/voltage.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <optional>

#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/mechanisms/config/PivotConfig.hpp"
#include "yams/mechanisms/positional/Pivot.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/remote/TalonFXWrapper.hpp"

class TurretSubsystem : public frc2::SubsystemBase {
 public:
  TurretSubsystem();

  frc::Pose2d GetPose(frc::Pose2d robotPose) const;
  frc::ChassisSpeeds GetVelocity(frc::ChassisSpeeds robotVelocity, units::degree_t robotAngle);

  void SetAngleSetpoint(units::degree_t angle);

  frc2::CommandPtr TurretCmd(double dutycycle);
  frc2::CommandPtr SysId();
  frc2::CommandPtr SetAngle(units::degree_t angle);

  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  ctre::phoenix6::hardware::TalonFX m_talonFX{12};

  yams::motorcontrollers::SmartMotorControllerConfig m_motorConfig;
  std::optional<yams::motorcontrollers::remote::TalonFXWrapper> m_motor;

  yams::mechanisms::config::PivotConfig m_pivotConfig;
  std::optional<yams::mechanisms::positional::Pivot> m_turret;

  // Transform from robot center to turret pivot (1.5 ft back, 0.5 ft up)
  frc::Transform3d m_roboToTurret;
};
