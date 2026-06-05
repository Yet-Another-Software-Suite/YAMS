// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "subsystems/SwerveSubsystem.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/system/plant/DCMotor.h>
#include <frc2/command/Commands.h>
#include <units/angle.h>
#include <units/current.h>
#include <units/length.h>

#include <numbers>
#include <string>
#include <utility>

using namespace yams::motorcontrollers;
using namespace yams::gearing;
using namespace yams::mechanisms::swerve;
using namespace yams::mechanisms::config;
using Cfg = SmartMotorControllerConfig;

yams::mechanisms::swerve::SwerveModule SwerveSubsystem::CreateModule(
    rev::spark::SparkMax& drive, rev::spark::SparkMax& azimuth,
    ctre::phoenix6::hardware::CANcoder& absoluteEncoder, const std::string& moduleName,
    frc::Translation2d location,
    std::optional<yams::motorcontrollers::local::SparkWrapper>& driveSMC,
    std::optional<yams::motorcontrollers::local::SparkWrapper>& azimuthSMC) {
  MechanismGearing driveGearing{GearBox::FromStages({"12:1", "2:1"})};
  MechanismGearing azimuthGearing{GearBox::FromStages({"21:1"})};

  SmartMotorControllerConfig driveCfg;
  driveCfg.WithSubsystem(this)
      .WithMechanismCircumference(units::meter_t{4.0 * 0.0254 * std::numbers::pi})
      .WithFeedback(50, 0, 4)
      .WithMotorGearing(driveGearing)
      .WithStatorCurrentLimit(units::ampere_t{40})
      .WithTelemetry("driveMotor", Cfg::TelemetryVerbosity::HIGH);

  SmartMotorControllerConfig azimuthCfg;
  azimuthCfg.WithSubsystem(this)
      .WithFeedback(50, 0, 4)
      .WithMotorGearing(azimuthGearing)
      .WithStatorCurrentLimit(units::ampere_t{20})
      .WithTelemetry("angleMotor", Cfg::TelemetryVerbosity::HIGH);

  driveSMC.emplace(drive, frc::DCMotor::NEO(1), driveCfg);
  azimuthSMC.emplace(azimuth, frc::DCMotor::NEO(1), azimuthCfg);

  auto* encoderPtr = &absoluteEncoder;
  SwerveModuleConfig moduleConfig{&driveSMC.value(), &azimuthSMC.value()};
  moduleConfig
      .WithAbsoluteEncoder([encoderPtr]() -> units::degree_t {
        return units::degree_t{units::turn_t{encoderPtr->GetAbsolutePosition().GetValue()}};
      })
      .WithTelemetry(moduleName, Cfg::TelemetryVerbosity::HIGH)
      .WithLocation(location)
      .WithOptimization(true);

  return SwerveModule{moduleConfig};
}

SwerveSubsystem::SwerveSubsystem() {
  m_fl.emplace(CreateModule(m_flDrive, m_flAzimuth, m_flEncoder, "frontleft",
                            frc::Translation2d{units::inch_t{24}, units::inch_t{24}}, m_flDriveSMC,
                            m_flAzimuthSMC));
  m_fr.emplace(CreateModule(m_frDrive, m_frAzimuth, m_frEncoder, "frontright",
                            frc::Translation2d{units::inch_t{24}, units::inch_t{-24}}, m_frDriveSMC,
                            m_frAzimuthSMC));
  m_bl.emplace(CreateModule(m_blDrive, m_blAzimuth, m_blEncoder, "backleft",
                            frc::Translation2d{units::inch_t{-24}, units::inch_t{24}}, m_blDriveSMC,
                            m_blAzimuthSMC));
  m_br.emplace(CreateModule(m_brDrive, m_brAzimuth, m_brEncoder, "backright",
                            frc::Translation2d{units::inch_t{-24}, units::inch_t{-24}},
                            m_brDriveSMC, m_brAzimuthSMC));

  SwerveDriveConfig config;
  config.WithSubsystem(this)
      .WithModules({&m_fl.value(), &m_fr.value(), &m_bl.value(), &m_br.value()})
      .WithGyro([gyroPtr = &m_gyro]() -> units::degree_t {
        return units::degree_t{units::turn_t{gyroPtr->GetYaw().GetValue()}};
      })
      .WithStartingPose(frc::Pose2d{units::meter_t{0}, units::meter_t{0}, frc::Rotation2d{}})
      .WithTranslationController(frc::PIDController{1, 0, 0})
      .WithRotationController(frc::PIDController{1, 0, 0});

  m_drive.emplace(std::move(config));
}

frc2::CommandPtr SwerveSubsystem::SetRobotRelativeChassisSpeeds(frc::ChassisSpeeds speeds) {
  return Run([this, speeds] { m_drive->SetRobotRelativeChassisSpeeds(speeds); });
}

frc2::CommandPtr SwerveSubsystem::DriveToPose(frc::Pose2d pose) {
  return m_drive->DriveToPose(pose);
}

frc2::CommandPtr SwerveSubsystem::DriveRobotRelative(
    std::function<frc::ChassisSpeeds()> speedsSupplier) {
  return m_drive->Drive(speedsSupplier);
}

frc2::CommandPtr SwerveSubsystem::Lock() {
  return Run([this] { m_drive->LockPose(); });
}

frc::Pose2d SwerveSubsystem::GetPose() { return m_drive->GetPose(); }

frc::ChassisSpeeds SwerveSubsystem::GetFieldOrientedChassisSpeed() {
  return m_drive->GetFieldRelativeSpeed();
}

units::degree_t SwerveSubsystem::GetGyroAngle() { return m_drive->GetGyroAngle(); }

void SwerveSubsystem::Periodic() { m_drive->UpdateTelemetry(); }

void SwerveSubsystem::SimulationPeriodic() { m_drive->SimIterate(); }
