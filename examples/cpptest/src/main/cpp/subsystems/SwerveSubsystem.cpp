// Copyright (c) 2026 Yet Another Software Suite
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
using namespace yams::mechanisms::swerve::utility;
using Cfg = SmartMotorControllerConfig;

// SetupModule is called once per corner in the constructor.  It populates the caller-supplied
// member config objects (so their addresses are stable for the lifetime of the subsystem),
// emplace()'s the SparkWrappers, and emplace()'s the finished SwerveModule into moduleOut.
//
// CAN ID layout (from the header):
//   FL: drive=1, azimuth=2, CANcoder=3
//   FR: drive=4, azimuth=5, CANcoder=6
//   BL: drive=7, azimuth=8, CANcoder=9
//   BR: drive=10, azimuth=11, CANcoder=12
//   Pigeon2 gyro: 14
void SwerveSubsystem::SetupModule(
    rev::spark::SparkMax* drive, rev::spark::SparkMax* azimuth,
    ctre::phoenix6::hardware::CANcoder& absoluteEncoder, const std::string& moduleName,
    frc::Translation2d location,
    std::optional<yams::motorcontrollers::local::SparkWrapper>& driveSMC,
    std::optional<yams::motorcontrollers::local::SparkWrapper>& azimuthSMC,
    SmartMotorControllerConfig& driveCfgMember, SmartMotorControllerConfig& azimuthCfgMember,
    SwerveModuleConfig& moduleCfgMember,
    std::optional<yams::mechanisms::swerve::SwerveModule>& moduleOut) {
  // Drive gearing: GearBox::FromStages parses ratio strings and multiplies them.
  // "12:1" then "2:1" gives a 24:1 total rotor-to-wheel reduction.
  // Circumference = pi * 4 in (4-inch wheel) -- tells the SMC how to convert rotor
  // turns into meters traveled, which the pose estimator needs for odometry.
  //
  // Azimuth gearing: 21:1 reduction from motor to steering axle.
  // Lower current limit (20 A vs 40 A) because the steering motor is smaller-duty.
  MechanismGearing driveGearing{GearBox::FromStages({"12:1", "2:1"})};
  MechanismGearing azimuthGearing{GearBox::FromStages({"21:1"})};

  // kP=50 with kD=4 gives a stiff velocity loop on the drive.  kD suppresses overshoot
  // when the module snaps to a new wheel speed; tune kP down if wheels chatter at low speed.
  driveCfgMember.WithSubsystem(this)
      .WithMechanismCircumference(units::meter_t{4.0 * 0.0254 * std::numbers::pi})
      .WithFeedback(50, 0, 4)
      .WithMotorGearing(driveGearing)
      .WithStatorCurrentLimit(units::ampere_t{40})
      .WithTelemetry("driveMotor", Cfg::TelemetryVerbosity::HIGH);

  // Same PID shape for the azimuth.  No feedforward needed because the NEO holds angle
  // with PD alone at these gains.  Add kS if you observe steady-state azimuth error.
  azimuthCfgMember.WithSubsystem(this)
      .WithFeedback(50, 0, 4)
      .WithMotorGearing(azimuthGearing)
      .WithStatorCurrentLimit(units::ampere_t{20})
      .WithTelemetry("angleMotor", Cfg::TelemetryVerbosity::HIGH);

  // SparkWrapper takes pointers; pass the address of the member configs and motors,
  // which have stable lifetime as subsystem members.
  driveSMC.emplace(drive, frc::DCMotor::NEO(1), &driveCfgMember);
  azimuthSMC.emplace(azimuth, frc::DCMotor::NEO(1), &azimuthCfgMember);

  // CANcoder: absolute magnetic encoder that gives the true wheel angle in turns on every
  // power cycle.  The lambda converts CANcoder turns to degrees and is called by
  // SwerveModule each loop to seed/synchronize the SparkMax relative encoder.
  // Without this, the azimuth SparkMax would reset to 0 deg after every deploy or brownout.
  //
  // WithOptimization(true): allows the module to flip drive direction and rotate at most 90 deg
  // instead of 270 deg to reach the commanded angle, cutting steering travel in half.
  auto* encoderPtr = &absoluteEncoder;
  moduleCfgMember = SwerveModuleConfig{&driveSMC.value(), &azimuthSMC.value()};
  moduleCfgMember
      .WithAbsoluteEncoder([encoderPtr]() -> units::degree_t {
        return units::degree_t{units::turn_t{encoderPtr->GetAbsolutePosition().GetValue()}};
      })
      .WithTelemetry(moduleName, Cfg::TelemetryVerbosity::HIGH)
      .WithLocation(location)
      .WithOptimization(true);

  moduleOut.emplace(&moduleCfgMember);
}

SwerveSubsystem::SwerveSubsystem() {
  // Module locations are Translation2d{X_forward, Y_left} from the robot centre.
  // Sign convention: +X is toward the intake/front, +Y is toward the left side.
  // All four corners are 24 in from centre in each axis, so the wheelbase and track
  // width are both 48 in.  Adjust these if the frame dimensions change.
  SetupModule(&m_flDrive, &m_flAzimuth, m_flEncoder, "frontleft",
              frc::Translation2d{units::inch_t{24}, units::inch_t{24}}, m_flDriveSMC,
              m_flAzimuthSMC, m_flDriveCfg, m_flAzimuthCfg, m_flModuleCfg, m_fl);
  SetupModule(&m_frDrive, &m_frAzimuth, m_frEncoder, "frontright",
              frc::Translation2d{units::inch_t{24}, units::inch_t{-24}}, m_frDriveSMC,
              m_frAzimuthSMC, m_frDriveCfg, m_frAzimuthCfg, m_frModuleCfg, m_fr);
  SetupModule(&m_blDrive, &m_blAzimuth, m_blEncoder, "backleft",
              frc::Translation2d{units::inch_t{-24}, units::inch_t{24}}, m_blDriveSMC,
              m_blAzimuthSMC, m_blDriveCfg, m_blAzimuthCfg, m_blModuleCfg, m_bl);
  SetupModule(&m_brDrive, &m_brAzimuth, m_brEncoder, "backright",
              frc::Translation2d{units::inch_t{-24}, units::inch_t{-24}}, m_brDriveSMC,
              m_brAzimuthSMC, m_brDriveCfg, m_brAzimuthCfg, m_brModuleCfg, m_br);

  // m_driveConfig is a subsystem member so its address is stable; SwerveDrive<4> takes a
  // pointer to it.  The gyro lambda converts Pigeon2 yaw (in turns) to degrees; SwerveDrive
  // applies offset and inversion internally.
  //
  // WithStartingPose at the origin: odometry starts at (0, 0) facing 0 degrees.  Reset
  // this before auto if the robot starts at a known field position.
  //
  // Translation and rotation PID controllers are used by DriveToPose only.  kP=1 is a
  // placeholder -- tune these in auto by checking overshoot and settling time.
  m_driveConfig.WithSubsystem(this)
      .WithModules({&m_fl.value(), &m_fr.value(), &m_bl.value(), &m_br.value()})
      .WithGyro([gyroPtr = &m_gyro]() -> units::degree_t {
        return units::degree_t{units::turn_t{gyroPtr->GetYaw().GetValue()}};
      })
      .WithStartingPose(frc::Pose2d{units::meter_t{0}, units::meter_t{0}, frc::Rotation2d{}})
      .WithTranslationController(frc::PIDController{1, 0, 0})
      .WithRotationController(frc::PIDController{1, 0, 0});

  m_drive.emplace(&m_driveConfig);
}

// Constant-speed command: captures the ChassisSpeeds by value so the command keeps
// driving at the speed it was given even if the caller changes its local variable.
frc2::CommandPtr SwerveSubsystem::SetRobotRelativeChassisSpeeds(frc::ChassisSpeeds speeds) {
  return Run([this, speeds] { m_drive->SetRobotRelativeChassisSpeeds(speeds); });
}

// DriveToPose uses the translation and rotation PID controllers set in the constructor.
// It drives field-relative using pose-estimator feedback; vision measurements improve accuracy.
frc2::CommandPtr SwerveSubsystem::DriveToPose(frc::Pose2d pose) {
  return m_drive->DriveToPose(pose);
}

// Accepts a supplier so the caller can update speeds each loop (e.g. from a joystick).
frc2::CommandPtr SwerveSubsystem::DriveRobotRelative(
    std::function<frc::ChassisSpeeds()> speedsSupplier) {
  return m_drive->Drive(speedsSupplier);
}

// X-lock: points all modules toward the centre of the robot to resist pushing.
// Useful as a defensive command bound to a button.
frc2::CommandPtr SwerveSubsystem::Lock() {
  return Run([this] { m_drive->LockPose(); });
}

frc::Pose2d SwerveSubsystem::GetPose() { return m_drive->GetPose(); }

frc::ChassisSpeeds SwerveSubsystem::GetFieldOrientedChassisSpeed() {
  return m_drive->GetFieldRelativeSpeed();
}

units::degree_t SwerveSubsystem::GetGyroAngle() { return m_drive->GetGyroAngle(); }

// Builds a SwerveInputStream from an Xbox controller with competition-ready defaults:
//   - Left stick Y -> forward/back (negated: push stick forward = positive X)
//   - Left stick X -> strafe left/right (negated: push stick left = positive Y)
//   - Right stick X -> rotation (not negated; CCW-positive by convention)
//   - 0.1 deadband removes controller drift
//   - 0.8 translation scale limits max speed to 80% to protect mechanisms
//   - Cubed rotation: fine control near centre, faster at the extremes
//   - WithAllianceRelativeControl: rotates the field frame so forward is always away from
//     your alliance wall regardless of which side of the field you start on
SwerveInputStream<4> SwerveSubsystem::MakeDriveInputStream(
    frc2::CommandXboxController& controller) {
  return SwerveInputStream<4>::Of(
             m_drive.value(), [&controller] { return -controller.GetLeftY(); },
             [&controller] { return -controller.GetLeftX(); })
      .WithControllerRotationAxis([&controller] { return controller.GetRightX(); })
      .WithDeadband(0.1)
      .WithScaleTranslation(0.8)
      .WithCubeRotationControllerAxis()
      .WithAllianceRelativeControl();
}

// m_driveStream must outlive the command returned here; it is stored as a subsystem member
// so the lambdas inside SwerveDrive::Drive remain valid for the command's lifetime.
frc2::CommandPtr SwerveSubsystem::DriveCommand(frc2::CommandXboxController& controller) {
  m_driveStream.emplace(MakeDriveInputStream(controller));
  return m_drive->Drive([this] { return (*m_driveStream)(); });
}

// UpdateTelemetry also updates the pose estimator from module positions + gyro.
void SwerveSubsystem::Periodic() { m_drive->UpdateTelemetry(); }

// SimIterate steps each module's DCMotorSim and integrates the simulated gyro heading.
void SwerveSubsystem::SimulationPeriodic() { m_drive->SimIterate(); }
