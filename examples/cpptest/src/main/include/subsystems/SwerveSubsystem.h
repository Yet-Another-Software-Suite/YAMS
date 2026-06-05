// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/PIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/system/plant/DCMotor.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>
#include <units/length.h>

#include <functional>
#include <memory>
#include <optional>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <rev/SparkMax.h>

#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/mechanisms/config/SwerveModuleConfig.hpp"
#include "yams/mechanisms/swerve/SwerveDrive.hpp"
#include "yams/mechanisms/swerve/SwerveDriveConfig.hpp"
#include "yams/mechanisms/swerve/SwerveModule.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/local/SparkWrapper.hpp"

class SwerveSubsystem : public frc2::SubsystemBase {
 public:
  SwerveSubsystem();

  frc2::CommandPtr SetRobotRelativeChassisSpeeds(frc::ChassisSpeeds speeds);
  frc2::CommandPtr DriveToPose(frc::Pose2d pose);
  frc2::CommandPtr DriveRobotRelative(std::function<frc::ChassisSpeeds()> speedsSupplier);
  frc2::CommandPtr Lock();

  frc::Pose2d GetPose();
  frc::ChassisSpeeds GetFieldOrientedChassisSpeed();
  units::degree_t GetGyroAngle();

  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  ctre::phoenix6::hardware::Pigeon2 m_gyro{14};

  // Front-left
  rev::spark::SparkMax m_flDrive{1, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_flAzimuth{2, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder m_flEncoder{3};

  // Front-right
  rev::spark::SparkMax m_frDrive{4, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_frAzimuth{5, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder m_frEncoder{6};

  // Back-left
  rev::spark::SparkMax m_blDrive{7, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_blAzimuth{8, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder m_blEncoder{9};

  // Back-right
  rev::spark::SparkMax m_brDrive{10, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_brAzimuth{11, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder m_brEncoder{12};

  // Motor controllers (optional to allow deferred construction)
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_flDriveSMC;
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_flAzimuthSMC;
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_frDriveSMC;
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_frAzimuthSMC;
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_blDriveSMC;
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_blAzimuthSMC;
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_brDriveSMC;
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_brAzimuthSMC;

  // Swerve modules
  std::optional<yams::mechanisms::swerve::SwerveModule> m_fl;
  std::optional<yams::mechanisms::swerve::SwerveModule> m_fr;
  std::optional<yams::mechanisms::swerve::SwerveModule> m_bl;
  std::optional<yams::mechanisms::swerve::SwerveModule> m_br;

  // Drive
  std::optional<yams::mechanisms::swerve::SwerveDrive<4>> m_drive;

  yams::mechanisms::swerve::SwerveModule CreateModule(
      rev::spark::SparkMax& drive, rev::spark::SparkMax& azimuth,
      ctre::phoenix6::hardware::CANcoder& absoluteEncoder, const std::string& moduleName,
      frc::Translation2d location,
      std::optional<yams::motorcontrollers::local::SparkWrapper>& driveSMC,
      std::optional<yams::motorcontrollers::local::SparkWrapper>& azimuthSMC);
};
