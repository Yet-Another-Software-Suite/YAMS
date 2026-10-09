// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <wpi/math/controller/PIDController.hpp>
#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/math/kinematics/ChassisVelocities.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/commands2/SubsystemBase.hpp>
#include <wpi/commands2/button/CommandXboxController.hpp>
#include <rev/SparkMax.h>
#include <wpi/hardware/bus/CANPort.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/length.hpp>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/mechanisms/config/SwerveModuleConfig.hpp"
#include "yams/mechanisms/swerve/SwerveDrive.hpp"
#include "yams/mechanisms/swerve/SwerveDriveConfig.hpp"
#include "yams/mechanisms/swerve/SwerveModule.hpp"
#include "yams/mechanisms/swerve/utility/SwerveInputStream.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/local/SparkWrapper.hpp"

// Four-module swerve drive with REV SPARK Max (NEO) drive + azimuth motors and
// CTRE CANcoder absolute encoders.  Heading comes from a Pigeon2 (CAN 14).
//
// Module layout (robot-centric, x forward, y left):
//   Front-left  (FL): drive CAN 1, azimuth CAN 2, CANcoder CAN 3  -- +24 in, +24 in
//   Front-right (FR): drive CAN 4, azimuth CAN 5, CANcoder CAN 6  -- +24 in, -24 in
//   Back-left   (BL): drive CAN 7, azimuth CAN 8, CANcoder CAN 9  -- -24 in, +24 in
//   Back-right  (BR): drive CAN 10, azimuth CAN 11, CANcoder CAN 12 -- -24 in, -24 in
//
// Drive gearing: 12:1 (12:1 x 2:1) on 4-inch wheels.  Azimuth gearing: 21:1.
// Drive PID: kP=50, kD=4.  Azimuth PID: kP=50, kD=4.
// Drive stator limit: 40 A.  Azimuth stator limit: 20 A.
//
// SwerveDrive<4> is built via optional + emplace() because it owns non-copyable state
// (pose estimator, NT4 publishers) and must be constructed after all modules are ready.
// Modules are also held as optional<SwerveModule> for the same reason.  Hardware objects
// (SparkMax, CANcoder, Pigeon2) are plain members and are constructed first.
//
// Commands exposed:
//   SetRobotRelativeChassisSpeeds(speeds) -- one-shot robot-relative drive
//   DriveToPose(pose)                     -- path-following to a field pose
//   DriveRobotRelative(supplier)          -- continuous robot-relative drive from a supplier
//   Lock()                                -- X-lock all modules in place
//   DriveCommand(controller)              -- field-oriented teleop drive from an XboxController
class SwerveSubsystem : public wpi::cmd::SubsystemBase {
 public:
  SwerveSubsystem();

  wpi::cmd::CommandPtr SetRobotRelativeChassisSpeeds(wpi::math::ChassisVelocities speeds);
  wpi::cmd::CommandPtr DriveToPose(wpi::math::Pose2d pose);
  wpi::cmd::CommandPtr DriveRobotRelative(std::function<wpi::math::ChassisVelocities()> speedsSupplier);
  wpi::cmd::CommandPtr Lock();

  /**
   * Build a SwerveInputStream from an XboxController with typical competition settings:
   * left stick → translation, right stick X → angular velocity, 0.1 deadband,
   * 0.8 translation scale, cubed rotation, and alliance-relative field orientation.
   *
   * Store the returned stream and pass it to DriveCommand(), or capture it in a
   * default-command lambda: m_drive->Drive([&stream]{ return stream.Get(); }).
   */
  yams::mechanisms::swerve::utility::SwerveInputStream<4> MakeDriveInputStream(
      wpi::cmd::CommandXboxController& controller);

  /**
   * Return a command that drives the robot using MakeDriveInputStream().
   * The stream is stored internally for the lifetime of the subsystem.
   *
   * Typical usage in RobotContainer:
   *   m_drive.SetDefaultCommand(m_drive.DriveCommand(m_xboxController));
   */
  wpi::cmd::CommandPtr DriveCommand(wpi::cmd::CommandXboxController& controller);

  wpi::math::Pose2d GetPose();
  wpi::math::ChassisVelocities GetFieldOrientedChassisSpeed();
  wpi::units::degree_t GetGyroAngle();

  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  ctre::phoenix6::hardware::Pigeon2 m_gyro{14, ctre::phoenix6::CANBus{}};  // CAN ID 14; yaw is fed to SwerveDriveConfig

  // Front-left module  (drive CAN 1, azimuth CAN 2, CANcoder CAN 3)
  rev::spark::SparkMax m_flDrive{wpi::CANPort::CAN_S0, 1, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_flAzimuth{wpi::CANPort::CAN_S0, 2, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder m_flEncoder{3, ctre::phoenix6::CANBus{}};

  // Front-right module  (drive CAN 4, azimuth CAN 5, CANcoder CAN 6)
  rev::spark::SparkMax m_frDrive{wpi::CANPort::CAN_S0, 4, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_frAzimuth{wpi::CANPort::CAN_S0, 5, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder m_frEncoder{6, ctre::phoenix6::CANBus{}};

  // Back-left module  (drive CAN 7, azimuth CAN 8, CANcoder CAN 9)
  rev::spark::SparkMax m_blDrive{wpi::CANPort::CAN_S0, 7, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_blAzimuth{wpi::CANPort::CAN_S0, 8, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder m_blEncoder{9, ctre::phoenix6::CANBus{}};

  // Back-right module  (drive CAN 10, azimuth CAN 11, CANcoder CAN 12)
  rev::spark::SparkMax m_brDrive{wpi::CANPort::CAN_S0, 10, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_brAzimuth{wpi::CANPort::CAN_S0, 11, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder m_brEncoder{12, ctre::phoenix6::CANBus{}};

  // Config objects must be declared before the wrappers and modules that store pointers
  // into them, so they are fully constructed (and have stable addresses) before those
  // dependent members are emplace()'d in the constructor.

  // SmartMotorControllerConfigs for each module (must outlive the SparkWrappers)
  yams::motorcontrollers::SmartMotorControllerConfig m_flDriveCfg;
  yams::motorcontrollers::SmartMotorControllerConfig m_flAzimuthCfg;
  yams::motorcontrollers::SmartMotorControllerConfig m_frDriveCfg;
  yams::motorcontrollers::SmartMotorControllerConfig m_frAzimuthCfg;
  yams::motorcontrollers::SmartMotorControllerConfig m_blDriveCfg;
  yams::motorcontrollers::SmartMotorControllerConfig m_blAzimuthCfg;
  yams::motorcontrollers::SmartMotorControllerConfig m_brDriveCfg;
  yams::motorcontrollers::SmartMotorControllerConfig m_brAzimuthCfg;

  // SwerveModuleConfigs (must outlive the SwerveModule instances)
  yams::mechanisms::config::SwerveModuleConfig m_flModuleCfg;
  yams::mechanisms::config::SwerveModuleConfig m_frModuleCfg;
  yams::mechanisms::config::SwerveModuleConfig m_blModuleCfg;
  yams::mechanisms::config::SwerveModuleConfig m_brModuleCfg;

  // SwerveDriveConfig (must outlive SwerveDrive<4>)
  yams::mechanisms::swerve::SwerveDriveConfig m_driveConfig;

  // SparkWrapper takes its SparkMax by pointer, so the SparkMax members above must
  // outlive these wrappers.  optional allows deferred construction inside SetupModule().
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_flDriveSMC;
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_flAzimuthSMC;
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_frDriveSMC;
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_frAzimuthSMC;
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_blDriveSMC;
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_blAzimuthSMC;
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_brDriveSMC;
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_brAzimuthSMC;

  // SwerveModule owns pointers into the SMC optionals above; must be constructed after them.
  std::optional<yams::mechanisms::swerve::SwerveModule> m_fl;
  std::optional<yams::mechanisms::swerve::SwerveModule> m_fr;
  std::optional<yams::mechanisms::swerve::SwerveModule> m_bl;
  std::optional<yams::mechanisms::swerve::SwerveModule> m_br;

  // SwerveDrive<4> holds NT4 publishers and a pose estimator -- non-copyable, non-movable
  // after construction.  emplace()'d in the constructor after all four modules are ready.
  std::optional<yams::mechanisms::swerve::SwerveDrive<4>> m_drive;
  // Stored so its internal controller lambda stays alive for the lifetime of DriveCommand().
  std::optional<yams::mechanisms::swerve::utility::SwerveInputStream<4>> m_driveStream;

  // Populates one swerve module: fills the caller-supplied member config objects, emplace()'s
  // the two SparkWrappers, wires up the CANcoder absolute-position lambda, and emplace()'s
  // the finished SwerveModule into moduleOut.  Called once per corner in the constructor.
  void SetupModule(rev::spark::SparkMax* drive, rev::spark::SparkMax* azimuth,
                   ctre::phoenix6::hardware::CANcoder& absoluteEncoder,
                   const std::string& moduleName, wpi::math::Translation2d location,
                   std::optional<yams::motorcontrollers::local::SparkWrapper>& driveSMC,
                   std::optional<yams::motorcontrollers::local::SparkWrapper>& azimuthSMC,
                   yams::motorcontrollers::SmartMotorControllerConfig& driveCfgMember,
                   yams::motorcontrollers::SmartMotorControllerConfig& azimuthCfgMember,
                   yams::mechanisms::config::SwerveModuleConfig& moduleCfgMember,
                   std::optional<yams::mechanisms::swerve::SwerveModule>& moduleOut);
};
