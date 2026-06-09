// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "subsystems/TurretSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/system/plant/DCMotor.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>

#include <cmath>
#include <iostream>
#include <numbers>

using namespace yams::motorcontrollers;
using namespace yams::gearing;
using namespace yams::mechanisms;
using Cfg = SmartMotorControllerConfig;

TurretSubsystem::TurretSubsystem()
    : m_roboToTurret{frc::Translation3d{units::foot_t{-1.5}, units::foot_t{0}, units::foot_t{0.5}},
                     frc::Rotation3d{}} {
  m_motorConfig.WithSubsystem(this)
      .WithClosedLoopMode()
      .WithFeedback(0.0, 0.0, 0)
      .WithMotorGearing(MechanismGearing{GearBox({144.0 / 15.0, 5.0, 1.08})})
      .WithIdleMode(Cfg::MotorMode::BRAKE)
      .WithMotorInverted(false)
      .WithArmFeedforward(0.5, 5.0, 0, 0.0)
      .WithTelemetry("TurretMotor", Cfg::TelemetryVerbosity::HIGH)
      .WithStatorCurrentLimit(units::ampere_t{60});

  m_motor.emplace(m_talonFX, frc::DCMotor::KrakenX60(1), m_motorConfig);

  m_pivotConfig.WithMotorController(&m_motor.value())
      .WithSubsystem(this)
      .WithStartingAngle(units::degree_t{0})
      .WithMinAngle(units::degree_t{-360})
      .WithMaxAngle(units::degree_t{360})
      .WithTelemetryName("Turret");

  m_turret.emplace(m_pivotConfig);
}

frc::Pose2d TurretSubsystem::GetPose(frc::Pose2d robotPose) const {
  return robotPose.TransformBy(frc::Transform2d{m_roboToTurret.Translation().ToTranslation2d(),
                                                m_roboToTurret.Rotation().ToRotation2d()});
}

frc::ChassisSpeeds TurretSubsystem::GetVelocity(frc::ChassisSpeeds robotVelocity,
                                                units::degree_t robotAngle) {
  auto rRobot = m_roboToTurret.Translation().ToTranslation2d();
  auto rWorld = rRobot.RotateBy(frc::Rotation2d{robotAngle});

  double omega = robotVelocity.omega.value();  // rad/s

  double vRotX = -omega * rWorld.Y().value();
  double vRotY = omega * rWorld.X().value();

  units::meters_per_second_t turretVx{robotVelocity.vx.value() + vRotX};
  units::meters_per_second_t turretVy{robotVelocity.vy.value() + vRotY};

  double mecVelDegPerS = m_motor->GetMechanismVelocity().value();
  double mecVelRadPerS = mecVelDegPerS * std::numbers::pi / 180.0;
  units::radians_per_second_t turretOmega{omega + mecVelRadPerS};

  return frc::ChassisSpeeds{turretVx, turretVy, turretOmega};
}

void TurretSubsystem::SetAngleSetpoint(units::degree_t angle) {
  m_turret->SetMechanismPositionSetpoint(angle);
}

void TurretSubsystem::Periodic() { m_turret->UpdateTelemetry(); }

void TurretSubsystem::SimulationPeriodic() { m_turret->SimIterate(); }

frc2::CommandPtr TurretSubsystem::TurretCmd(double dutycycle) {
  return Run([this, dutycycle] {
    m_motor->SetDutyCycle(dutycycle);
    std::cout << "HIIII";
  });
}

frc2::CommandPtr TurretSubsystem::SetAngle(units::degree_t angle) { return m_turret->Run(angle); }
