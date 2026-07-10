// Copyright (c) 2026 Yet Another Software Suite
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

#include <ctre/phoenix6/TalonFX.hpp>
#include <optional>

#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/mechanisms/config/PivotConfig.hpp"
#include "yams/mechanisms/positional/Pivot.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/remote/TalonFXWrapper.hpp"

// Turret driven by a single Kraken X60 (CAN 12) through a compound gearbox:
//   stage 1: 144/15 (= 9.6:1), stage 2: 5:1, stage 3: 1.08:1  -> ~51.84:1 total.
// Angle is in degrees (mechanism space); soft limits are [-360, +360] deg so the
// turret can spin a full revolution in either direction without a hard stop.
// WithFeedforward(ArmFeedforward) is used here for the pivot-style kS/kV gravity model (kG = 0).
//
// The turret pivot is mounted 1.5 ft behind and 0.5 ft above the robot center.
// GetPose / GetVelocity project that offset into field-relative coordinates so
// vision or shooter code can aim relative to the turret barrel, not the robot center.
//
// Commands:
//   TurretCmd(dutycycle) -- open-loop, for manual adjustment
//   SetAngle(degree_t)   -- closed-loop continuous move; non-finishing
//
// Non-command entry point:
//   SetAngleSetpoint(degree_t) -- call from Periodic if you want to drive setpoint
//                                 from outside a command (e.g. field-relative auto-aim)
class TurretSubsystem : public frc2::SubsystemBase {
 public:
  TurretSubsystem();

  frc::Pose2d GetPose(frc::Pose2d robotPose) const;
  frc::ChassisSpeeds GetVelocity(frc::ChassisSpeeds robotVelocity, units::degree_t robotAngle);

  void SetAngleSetpoint(units::degree_t angle);

  frc2::CommandPtr TurretCmd(double dutycycle);
  frc2::CommandPtr SetAngle(units::degree_t angle);

  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  ctre::phoenix6::hardware::TalonFX m_talonFX{12};  // Kraken X60, CAN 12

  yams::motorcontrollers::SmartMotorControllerConfig m_motorConfig;
  std::optional<yams::motorcontrollers::remote::TalonFXWrapper> m_motor;

  yams::mechanisms::config::PivotConfig m_pivotConfig;
  std::optional<yams::mechanisms::positional::Pivot> m_turret;

  // Transform from robot center to turret pivot (1.5 ft back, 0.5 ft up)
  frc::Transform3d m_roboToTurret;
};
