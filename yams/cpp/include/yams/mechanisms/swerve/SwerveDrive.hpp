// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/RobotBase.h>
#include <frc/Timer.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/struct/ChassisSpeedsStruct.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>
#include <units/time.h>
#include <units/velocity.h>
#include <wpi/array.h>
#include <wpi/json.h>

#include <cassert>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "yams/mechanisms/swerve/SwerveDriveConfig.hpp"
#include "yams/mechanisms/swerve/SwerveModule.hpp"

namespace yams::mechanisms::swerve {

/**
 * Swerve drive mechanism with pose estimation, telemetry, and simulation support.
 *
 * Owns the kinematics, pose estimator, and all NT4 struct publishers.  Modules are
 * created separately and handed in by pointer via SwerveDriveConfig.  Call
 * UpdateTelemetry() every loop and SimIterate() in SimulationPeriodic().
 *
 * @tparam NumModules Number of swerve modules (default 4).
 *
 * ### Example usage (inside a four-module subsystem)
 * @code{.cpp}
 * using namespace yams::motorcontrollers;
 * using namespace yams::motorcontrollers::remote;
 * using namespace yams::gearing;
 * using namespace yams::mechanisms::swerve;
 * using namespace yams::mechanisms::config;
 * using Cfg = SmartMotorControllerConfig;
 *
 * // Declare as subsystem members:
 * //   ctre::phoenix6::hardware::Pigeon2     m_pigeon{0};
 * //   // Four sets of: TalonFX, TalonFX, CANcoder, optional<TalonFXWrapper> ×2,
 * //   //               SwerveModuleConfig, optional<SwerveModule>
 * //   std::optional<SwerveDrive<4>>         m_drive;
 *
 * // --- build modules (front-left shown; repeat for FR, BL, BR) ---
 * SmartMotorControllerConfig driveCfg;
 * driveCfg.WithSubsystem(this)
 *         .WithMechanismCircumference(units::meter_t{4.0 * 0.0254 * std::numbers::pi})
 *         .WithFeedback(0.1, 0.0, 0.0)
 *         .WithMotorGearing(MechanismGearing{GearBox::FromStages({"6.75:1"})})
 *         .WithStatorCurrentLimit(40.0_A)
 *         .WithIdleMode(Cfg::MotorMode::BRAKE)
 *         .WithTelemetry("FL_Drive", Cfg::TelemetryVerbosity::HIGH);
 *
 * SmartMotorControllerConfig azimuthCfg;
 * azimuthCfg.WithSubsystem(this)
 *           .WithFeedback(50.0, 0.0, 0.5)
 *           .WithMotorGearing(MechanismGearing{GearBox::FromStages({"21.43:1"})})
 *           .WithStatorCurrentLimit(20.0_A)
 *           .WithClosedLoopMode()
 *           .WithTelemetry("FL_Azimuth", Cfg::TelemetryVerbosity::HIGH);
 *
 * m_flDriveSMC.emplace(m_flDrive, frc::DCMotor::KrakenX60(1), driveCfg);
 * m_flAzimuthSMC.emplace(m_flAzimuth, frc::DCMotor::KrakenX60(1), azimuthCfg);
 *
 * auto* enc = &m_flEncoder;
 * SwerveModuleConfig flCfg{&m_flDriveSMC.value(), &m_flAzimuthSMC.value()};
 * flCfg.WithAbsoluteEncoder([enc]() -> units::degree_t {
 *         return units::degree_t{units::turn_t{enc->GetAbsolutePosition().GetValue()}};
 *       })
 *      .WithAbsoluteEncoderOffset(15.0_deg)
 *      .WithWheelDiameter(4.0 * 0.0254_m)
 *      .WithLocation(units::inch_t{12}, units::inch_t{12})
 *      .WithOptimization(true)
 *      .WithTelemetry("FrontLeft", Cfg::TelemetryVerbosity::HIGH);
 * m_fl.emplace(flCfg);
 * // ... repeat for m_fr, m_bl, m_br ...
 *
 * // --- build the drive ---
 * SwerveDriveConfig driveCfg;
 * driveCfg.WithSubsystem(this)
 *         .WithModules({&m_fl.value(), &m_fr.value(), &m_bl.value(), &m_br.value()})
 *         .WithGyro([this]() -> units::degree_t {
 *           return units::degree_t{units::turn_t{m_pigeon.GetYaw().GetValue()}};
 *         })
 *         .WithMaximumChassisSpeed(4.5_mps, units::degrees_per_second_t{540})
 *         .WithDiscretizationTime(0.02_s)
 *         .WithStartingPose(frc::Pose2d{})
 *         .WithTranslationController(frc::PIDController{2.0, 0.0, 0.0})
 *         .WithRotationController(frc::PIDController{4.0, 0.0, 0.0});
 * m_drive.emplace(&m_driveConfig);
 *
 * // --- in Periodic() ---
 * //   m_drive->UpdateTelemetry();
 *
 * // --- in SimulationPeriodic() ---
 * //   m_drive->SimIterate();
 *
 * // --- drive with a joystick ---
 * //   frc2::CommandPtr driveCmd = m_drive->Drive([this]() -> frc::ChassisSpeeds {
 * //     return frc::ChassisSpeeds{xSpeed, ySpeed, rotSpeed};
 * //   });
 * @endcode
 */
template <size_t NumModules = 4>
class SwerveDrive {
 public:
  /**
   * Construct a SwerveDrive.
   *
   * @param config Drive configuration.  config.GetModules().size() must equal NumModules.
   */
  explicit SwerveDrive(SwerveDriveConfig* config)
      : m_config{config},
        m_kinematics{BuildKinematics(*m_config)},
        m_poseEstimator{m_kinematics, ComputeInitialRotation(*m_config),
                        ComputeInitialPositions(*m_config), m_config->GetInitialPose()} {
    assert(m_config->GetModules().size() == NumModules &&
           "Module count in SwerveDriveConfig must equal NumModules template parameter.");

    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("SmartDashboard/" + GetName());

    m_desiredModuleStatesPublisher =
        table->template GetStructArrayTopic<frc::SwerveModuleState>("states/desired").Publish();
    m_currentModuleStatesPublisher =
        table->template GetStructArrayTopic<frc::SwerveModuleState>("states/current").Publish();
    m_posePublisher = table->template GetStructTopic<frc::Pose2d>("pose").Publish();
    m_desiredRobotRelChassisSpeedsPublisher =
        table->template GetStructTopic<frc::ChassisSpeeds>("chassis/desired").Publish();
    m_currentRobotRelChassisSpeedsPublisher =
        table->template GetStructTopic<frc::ChassisSpeeds>("chassis/current").Publish();
    m_fieldRelChassisSpeedsPublisher =
        table->template GetStructTopic<frc::ChassisSpeeds>("chassis/field").Publish();
    auto gyroTopic = table->GetDoubleTopic("gyro");
    gyroTopic.SetProperties(wpi::json{{"units", "degrees"}});
    m_gyroPublisher = gyroTopic.Publish();

    m_field2d.SetRobotPose(GetPose());
    frc::SmartDashboard::PutData("Mechanisms/" + GetName() + "/field", &m_field2d);
  }

  // ---- Drive commands --------------------------------------------------------

  /**
   * Create a run command that continuously drives with robot-relative chassis speeds.
   *
   * @param robotRelativeSpeeds Callable returning the desired ChassisSpeeds.
   * @return RunCommand requiring the configured subsystem.
   */
  frc2::CommandPtr Drive(std::function<frc::ChassisSpeeds()> robotRelativeSpeeds) {
    return frc2::cmd::Run(
               [this, robotRelativeSpeeds] {
                 SetRobotRelativeChassisSpeeds(robotRelativeSpeeds());
               },
               {m_config->GetSubsystem()})
        .WithName("Drive");
  }

  /**
   * Create a command that drives the robot to a target field-relative pose using PID.
   *
   * Requires translation and rotation PID controllers to be configured.
   *
   * @param pose Target field-relative pose.
   * @return Command sequence.
   */
  frc2::CommandPtr DriveToPose(frc::Pose2d pose) {
    return frc2::cmd::RunOnce([this] {
             ResetTranslationPID();
             ResetAzimuthPID();
           })
        .AndThen(Drive([this, pose]() -> frc::ChassisSpeeds {
          auto& azimuthPID = m_config->GetRotationPID();
          auto& translationPID = m_config->GetTranslationPID();
          auto currentPose = GetPose();
          auto distance = GetDistanceFromPose(pose);
          auto translationScalar = translationPID.Calculate(distance.value(), 0.0);
          auto poseDiff = currentPose.RelativeTo(pose);
          return frc::ChassisSpeeds::FromFieldRelativeSpeeds(
              frc::ChassisSpeeds{
                  units::meters_per_second_t{poseDiff.X().value() * translationScalar},
                  units::meters_per_second_t{poseDiff.Y().value() * translationScalar},
                  units::radians_per_second_t{
                      azimuthPID.Calculate(currentPose.Rotation().Radians().value(),
                                           pose.Rotation().Radians().value())}},
              frc::Rotation2d{units::radian_t{GetGyroAngle()}});
        }))
        .WithName("Drive to Pose");
  }

  // ---- Core drive methods ---------------------------------------------------

  /**
   * Directly command the swerve modules to the given states.
   *
   * @param states Array of states (one per module, clockwise from front-left).
   */
  void SetSwerveModuleStates(wpi::array<frc::SwerveModuleState, NumModules> states) {
    for (size_t i = 0; i < NumModules; ++i) {
      m_config->GetModules()[i]->SetSwerveModuleState(states[i]);
    }
    m_desiredModuleStates = states;
  }

  /**
   * Convert robot-relative chassis speeds to module states (with optional optimisation).
   *
   * @param robotRelativeSpeeds Input chassis speeds.
   * @return Corresponding module states.
   */
  wpi::array<frc::SwerveModuleState, NumModules> GetStateFromRobotRelativeChassisSpeeds(
      frc::ChassisSpeeds robotRelativeSpeeds) {
    robotRelativeSpeeds = m_config->OptimizeRobotRelativeChassisSpeeds(robotRelativeSpeeds);
    if (auto cor = m_config->GetCenterOfRotation()) {
      return m_kinematics.ToSwerveModuleStates(robotRelativeSpeeds, *cor);
    }
    return m_kinematics.ToSwerveModuleStates(robotRelativeSpeeds);
  }

  /**
   * Set robot-relative chassis speeds.
   *
   * @param robotRelativeSpeeds Desired robot-relative chassis speeds.
   */
  void SetRobotRelativeChassisSpeeds(frc::ChassisSpeeds robotRelativeSpeeds) {
    m_desiredChassisSpeeds = robotRelativeSpeeds;
    SetSwerveModuleStates(GetStateFromRobotRelativeChassisSpeeds(robotRelativeSpeeds));
  }

  /**
   * Set field-relative chassis speeds.
   *
   * @param fieldRelativeSpeeds Desired field-relative chassis speeds.
   */
  void SetFieldRelativeChassisSpeeds(frc::ChassisSpeeds fieldRelativeSpeeds) {
    SetRobotRelativeChassisSpeeds(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        fieldRelativeSpeeds, frc::Rotation2d{units::radian_t{GetGyroAngle()}}));
  }

  /**
   * Lock all modules to an X-pattern to resist being pushed.
   */
  void LockPose() {
    wpi::array<frc::SwerveModuleState, NumModules> states{wpi::empty_array};
    for (size_t i = 0; i < NumModules; ++i) {
      states[i] =
          frc::SwerveModuleState{units::meters_per_second_t{0},
                                 m_config->GetModules()[i]->GetConfig().GetLocation()->Angle()};
    }
    SetSwerveModuleStates(states);
    m_desiredChassisSpeeds = frc::ChassisSpeeds{};
  }

  // ---- Odometry & pose -------------------------------------------------------

  /**
   * Get the current estimated field-relative pose.
   */
  frc::Pose2d GetPose() { return m_poseEstimator.GetEstimatedPosition(); }

  /**
   * Reset odometry to the given pose.
   *
   * @param pose New field-relative pose (blue-origin, 0° facing red alliance wall).
   */
  void ResetOdometry(frc::Pose2d pose) {
    m_poseEstimator.ResetPosition(frc::Rotation2d{units::radian_t{GetGyroAngle()}},
                                  GetModulePositions(), pose);
    m_desiredChassisSpeeds = frc::ChassisSpeeds{};
    m_desiredModuleStates = m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds{});
  }

  /**
   * Zero the gyro and reset odometry to the current translation with 0° heading.
   */
  void ZeroGyro() {
    m_config->WithGyroOffset(GetGyroAngle() + m_config->GetGyroOffset());
    ResetOdometry(frc::Pose2d{GetPose().Translation(), frc::Rotation2d{}});
  }

  /**
   * Add a vision measurement to the pose estimator.
   *
   * @param robotPose Measured field-relative pose from vision.
   * @param timestamp FPGA timestamp of the measurement (seconds).
   */
  void AddVisionMeasurement(frc::Pose2d robotPose, units::second_t timestamp) {
    m_poseEstimator.AddVisionMeasurement(robotPose, timestamp);
  }

  /**
   * Add a vision measurement with custom standard deviations.
   *
   * @param robotPose Measured field-relative pose from vision.
   * @param timestamp FPGA timestamp of the measurement (seconds).
   * @param stdDevs   Standard deviations {x_m, y_m, theta_rad}.
   */
  void AddVisionMeasurement(frc::Pose2d robotPose, units::second_t timestamp,
                            const wpi::array<double, 3>& stdDevs) {
    m_poseEstimator.AddVisionMeasurement(robotPose, timestamp, stdDevs);
  }

  // ---- Telemetry & simulation -----------------------------------------------

  /**
   * Update the pose estimator and publish all telemetry to NetworkTables.
   *
   * Call this once per robot loop.
   */
  void UpdateTelemetry() {
    UpdatePoseEstimator();
    auto pose = GetPose();
    auto currentStates = GetModuleStates();

    m_gyroPublisher.Set(units::degree_t{GetGyroAngle()}.value());
    m_desiredModuleStatesPublisher.Set(m_desiredModuleStates);
    m_currentModuleStatesPublisher.Set(currentStates);
    m_posePublisher.Set(pose);
    m_desiredRobotRelChassisSpeedsPublisher.Set(m_desiredChassisSpeeds);
    m_currentRobotRelChassisSpeedsPublisher.Set(GetRobotRelativeSpeed());
    m_fieldRelChassisSpeedsPublisher.Set(GetFieldRelativeSpeed());

    for (auto* mod : m_config->GetModules()) {
      mod->UpdateTelemetry();
    }

    m_field2d.SetRobotPose(pose);
    std::vector<frc::Pose2d> modulePoses;
    modulePoses.reserve(NumModules);
    for (size_t i = 0; i < NumModules; ++i) {
      auto* mod = m_config->GetModules()[i];
      auto location = *mod->GetConfig().GetLocation();
      auto moduleTranslation = pose.Translation() + location.RotateBy(pose.Rotation());
      auto moduleHeading = pose.Rotation() + currentStates[i].angle;
      modulePoses.push_back(frc::Pose2d{moduleTranslation, moduleHeading});
    }
    m_field2d.GetObject("modules")->SetPoses(modulePoses);
  }

  /**
   * Advance the simulation model.
   *
   * Iterates each module's sim model and integrates the simulated gyro angle
   * from the chassis angular velocity.
   */
  void SimIterate() {
    if (!m_simTimer.IsRunning()) m_simTimer.Start();
    for (auto* mod : m_config->GetModules()) {
      mod->SimIterate();
    }
    auto speeds = m_kinematics.ToChassisSpeeds(GetModuleStates());
    m_simGyroAngle += units::degree_t{units::degrees_per_second_t{speeds.omega}.value() *
                                      m_simTimer.Get().value()};
    m_simTimer.Reset();
  }

  // ---- Queries ---------------------------------------------------------------

  /**
   * Get the current gyro angle.
   *
   * Returns the simulated angle in simulation, or the real gyro angle on hardware.
   */
  units::degree_t GetGyroAngle() {
    if (frc::RobotBase::IsSimulation()) return m_simGyroAngle;
    return m_config->GetGyroAngle();
  }

  /** Get the robot-relative chassis speeds derived from the module states. */
  frc::ChassisSpeeds GetRobotRelativeSpeed() {
    return m_kinematics.ToChassisSpeeds(GetModuleStates());
  }

  /** Get the field-relative chassis speeds derived from the module states and gyro. */
  frc::ChassisSpeeds GetFieldRelativeSpeed() {
    return frc::ChassisSpeeds::FromRobotRelativeSpeeds(
        GetRobotRelativeSpeed(), frc::Rotation2d{units::radian_t{GetGyroAngle()}});
  }

  /** Get the current module positions (distance + angle). */
  wpi::array<frc::SwerveModulePosition, NumModules> GetModulePositions() {
    wpi::array<frc::SwerveModulePosition, NumModules> positions{wpi::empty_array};
    for (size_t i = 0; i < NumModules; ++i) {
      positions[i] = m_config->GetModules()[i]->GetPosition();
    }
    return positions;
  }

  /** Get the current module states (speed + angle). */
  wpi::array<frc::SwerveModuleState, NumModules> GetModuleStates() {
    wpi::array<frc::SwerveModuleState, NumModules> states{wpi::empty_array};
    for (size_t i = 0; i < NumModules; ++i) {
      states[i] = m_config->GetModules()[i]->GetState();
    }
    return states;
  }

  /**
   * Get the Euclidean distance from the robot's current pose to the given pose.
   *
   * @param pose Target pose.
   * @return Distance in metres.
   */
  units::meter_t GetDistanceFromPose(frc::Pose2d pose) {
    return units::meter_t{GetPose().Translation().Distance(pose.Translation())};
  }

  /**
   * Get the heading difference from the robot's current rotation to the given pose.
   *
   * @param pose Target pose.
   * @return Heading difference in degrees.
   */
  units::degree_t GetAngleDifferenceFromPose(frc::Pose2d pose) {
    return (GetPose() - pose).Rotation().Degrees();
  }

  /**
   * Find a module by name.
   *
   * @param name Telemetry name of the module.
   * @return Pointer to the module if found, or empty optional.
   */
  std::optional<SwerveModule*> GetModule(const std::string& name) {
    for (auto* mod : m_config->GetModules()) {
      if (mod->GetName() == name) return mod;
    }
    return std::nullopt;
  }

  void ResetAzimuthPID() { m_config->GetRotationPID().Reset(); }
  void ResetTranslationPID() { m_config->GetTranslationPID().Reset(); }

  /**
   * Set the standard deviations used for the vision pose estimator noise model.
   *
   * @param stdDevs Standard deviations {x_m, y_m, theta_rad}.
   */
  void SetVisionMeasurementStdDevs(const wpi::array<double, 3>& stdDevs) {
    m_poseEstimator.SetVisionMeasurementStdDevs(stdDevs);
  }

  /** Get the drive's unique name (always "swerve"). */
  std::string GetName() const { return "swerve"; }

  /** Get a mutable reference to the drive configuration. */
  SwerveDriveConfig& GetConfig() { return *m_config; }

  /** Get the SwerveDriveKinematics object. */
  frc::SwerveDriveKinematics<NumModules>& GetKinematics() { return m_kinematics; }

 private:
  SwerveDriveConfig* m_config{nullptr};
  frc::SwerveDriveKinematics<NumModules> m_kinematics;
  frc::SwerveDrivePoseEstimator<NumModules> m_poseEstimator;

  nt::StructArrayPublisher<frc::SwerveModuleState> m_desiredModuleStatesPublisher;
  nt::StructArrayPublisher<frc::SwerveModuleState> m_currentModuleStatesPublisher;
  nt::StructPublisher<frc::ChassisSpeeds> m_desiredRobotRelChassisSpeedsPublisher;
  nt::StructPublisher<frc::ChassisSpeeds> m_currentRobotRelChassisSpeedsPublisher;
  nt::StructPublisher<frc::ChassisSpeeds> m_fieldRelChassisSpeedsPublisher;
  nt::StructPublisher<frc::Pose2d> m_posePublisher;
  nt::DoublePublisher m_gyroPublisher;

  frc::Field2d m_field2d;
  frc::Timer m_simTimer;
  units::degree_t m_simGyroAngle{0};

  wpi::array<frc::SwerveModuleState, NumModules> m_desiredModuleStates{wpi::empty_array};
  frc::ChassisSpeeds m_desiredChassisSpeeds{};

  void UpdatePoseEstimator() {
    m_poseEstimator.Update(frc::Rotation2d{units::radian_t{GetGyroAngle()}}, GetModulePositions());
  }

  static frc::SwerveDriveKinematics<NumModules> BuildKinematics(const SwerveDriveConfig& config) {
    assert(config.GetModules().size() == NumModules);
    wpi::array<frc::Translation2d, NumModules> locations{wpi::empty_array};
    for (size_t i = 0; i < NumModules; ++i) {
      locations[i] = *config.GetModules()[i]->GetConfig().GetLocation();
    }
    return frc::SwerveDriveKinematics<NumModules>{locations};
  }

  static frc::Rotation2d ComputeInitialRotation(const SwerveDriveConfig& config) {
    if (frc::RobotBase::IsSimulation()) return frc::Rotation2d{};
    return frc::Rotation2d{units::radian_t{config.GetGyroAngle()}};
  }

  static wpi::array<frc::SwerveModulePosition, NumModules> ComputeInitialPositions(
      const SwerveDriveConfig& config) {
    wpi::array<frc::SwerveModulePosition, NumModules> positions{wpi::empty_array};
    for (size_t i = 0; i < NumModules; ++i) {
      positions[i] = config.GetModules()[i]->GetPosition();
    }
    return positions;
  }
};

}  // namespace yams::mechanisms::swerve
