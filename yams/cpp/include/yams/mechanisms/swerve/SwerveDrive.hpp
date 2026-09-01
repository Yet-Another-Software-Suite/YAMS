// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <wpi/framework/RobotBase.hpp>
#include <wpi/system/Timer.hpp>
#include <wpi/math/estimator/SwerveDrivePoseEstimator.hpp>
#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/math/geometry/Twist2d.hpp>
#include <wpi/math/kinematics/ChassisVelocities.hpp>
#include <wpi/math/kinematics/SwerveDriveKinematics.hpp>
#include <wpi/math/kinematics/SwerveModulePosition.hpp>
#include <wpi/math/kinematics/SwerveModuleVelocity.hpp>
#include <wpi/math/kinematics/struct/ChassisVelocitiesStruct.hpp>
#include <wpi/smartdashboard/Field2d.hpp>
#include <wpi/smartdashboard/SmartDashboard.hpp>
#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/commands2/Commands.hpp>
#include <wpi/nt/NetworkTableInstance.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/math.hpp>
#include <wpi/units/time.hpp>
#include <wpi/units/velocity.hpp>
#include <wpi/util/array.hpp>

#include <cassert>
#include <cstdio>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "yams/mechanisms/swerve/SwerveDriveConfig.hpp"
#include "yams/mechanisms/swerve/SwerveModule.hpp"
#include "yams/telemetry/MechanismTelemetry.hpp"
#include "yams/telemetry/SwerveDriveTelemetry.hpp"
#include "yams/telemetry/SwerveDriveTelemetryConfig.hpp"

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
 *         .WithMechanismCircumference(wpi::units::meter_t{4.0 * 0.0254 * std::numbers::pi})
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
 * m_flDriveSMC.emplace(m_flDrive, wpi::math::DCMotor::KrakenX60(1), driveCfg);
 * m_flAzimuthSMC.emplace(m_flAzimuth, wpi::math::DCMotor::KrakenX60(1), azimuthCfg);
 *
 * auto* enc = &m_flEncoder;
 * SwerveModuleConfig flCfg{&m_flDriveSMC.value(), &m_flAzimuthSMC.value()};
 * flCfg.WithAbsoluteEncoder([enc]() -> wpi::units::degree_t {
 *         return wpi::units::degree_t{wpi::units::turn_t{enc->GetAbsolutePosition().GetValue()}};
 *       })
 *      .WithAbsoluteEncoderOffset(15.0_deg)
 *      .WithWheelDiameter(4.0 * 0.0254_m)
 *      .WithLocation(wpi::units::inch_t{12}, wpi::units::inch_t{12})
 *      .WithOptimization(true)
 *      .WithTelemetry("FrontLeft", Cfg::TelemetryVerbosity::HIGH);
 * m_fl.emplace(flCfg);
 * // ... repeat for m_fr, m_bl, m_br ...
 *
 * // --- build the drive ---
 * SwerveDriveConfig driveCfg;
 * driveCfg.WithSubsystem(this)
 *         .WithModules({&m_fl.value(), &m_fr.value(), &m_bl.value(), &m_br.value()})
 *         .WithGyro([this]() -> wpi::units::degree_t {
 *           return wpi::units::degree_t{wpi::units::turn_t{m_pigeon.GetYaw().GetValue()}};
 *         })
 *         .WithMaximumChassisSpeed(4.5_mps, wpi::units::degrees_per_second_t{540})
 *         .WithDiscretizationTime(0.02_s)
 *         .WithStartingPose(wpi::math::Pose2d{})
 *         .WithTranslationController(wpi::math::PIDController{2.0, 0.0, 0.0})
 *         .WithRotationController(wpi::math::PIDController{4.0, 0.0, 0.0});
 * m_drive.emplace(&m_driveConfig);
 *
 * // --- in Periodic() ---
 * //   m_drive->UpdateTelemetry();
 *
 * // --- in SimulationPeriodic() ---
 * //   m_drive->SimIterate();
 *
 * // --- drive with a joystick ---
 * //   wpi::cmd::CommandPtr driveCmd = m_drive->Drive([this]() -> wpi::math::ChassisVelocities {
 * //     return wpi::math::ChassisVelocities{xSpeed, ySpeed, rotSpeed};
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

    m_simPose = m_config->GetInitialPose();

    SetupTelemetry();

    m_field2d.SetRobotPose(GetPose());
    wpi::SmartDashboard::PutData("Mechanisms/" + GetName() + "/field", &m_field2d);
  }

  // ---- Drive commands --------------------------------------------------------

  /**
   * Create a run command that continuously drives with robot-relative chassis speeds.
   *
   * @param robotRelativeSpeeds Callable returning the desired ChassisSpeeds.
   * @return RunCommand requiring the configured subsystem.
   */
  wpi::cmd::CommandPtr Drive(std::function<wpi::math::ChassisVelocities()> robotRelativeSpeeds) {
    return wpi::cmd::Run(
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
  wpi::cmd::CommandPtr DriveToPose(wpi::math::Pose2d pose) {
    return wpi::cmd::RunOnce([this] {
             ResetTranslationPID();
             ResetRotationPID();
           })
        .AndThen(Drive([this, pose] { return DriveToPoseSetpoint(pose); }))
        .WithName("Drive to Pose");
  }

  /**
   * Drive to the target pose, primarily for use in Live Tuning; could also be used for setpoint
   * commands.
   *
   * @param targetPose Pose to drive towards.
   * @return robot-relative ChassisSpeeds to drive the robot to the given pose.
   * @implNote Remember to call ResetTranslationPID() and ResetRotationPID() before calling this
   *     method in a loop.
   */
  wpi::math::ChassisVelocities DriveToPoseSetpoint(wpi::math::Pose2d targetPose) {
    auto& rotationPID = m_config->GetRotationPID();
    auto& translationPID = m_config->GetTranslationPID();
    auto distance = GetDistanceFromPose(targetPose);
    auto translationScalar = translationPID.Calculate(distance.value(), 0.0);
    auto currentPose = GetPose();
    auto poseDiff = currentPose.RelativeTo(targetPose);
    return (wpi::math::ChassisVelocities{
                wpi::units::meters_per_second_t{poseDiff.X().value() * translationScalar},
                wpi::units::meters_per_second_t{poseDiff.Y().value() * translationScalar},
                wpi::units::radians_per_second_t{
                    rotationPID.Calculate(currentPose.Rotation().Radians().value(),
                                         targetPose.Rotation().Radians().value())}})
        .ToRobotRelative(wpi::math::Rotation2d{wpi::units::radian_t{GetGyroAngle()}});
  }

  // ---- Core drive methods ---------------------------------------------------

  /**
   * Directly command the swerve modules to the given states.
   *
   * @param states Array of states (one per module, clockwise from front-left).
   */
  void SetSwerveModuleStates(wpi::util::array<wpi::math::SwerveModuleVelocity, NumModules> states) {
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
  wpi::util::array<wpi::math::SwerveModuleVelocity, NumModules> GetStateFromRobotRelativeChassisSpeeds(
      wpi::math::ChassisVelocities robotRelativeSpeeds) {
    robotRelativeSpeeds = m_config->OptimizeRobotRelativeChassisSpeeds(robotRelativeSpeeds);
    if (auto cor = m_config->GetCenterOfRotation()) {
      return m_kinematics.ToSwerveModuleVelocities(robotRelativeSpeeds, *cor);
    }
    return m_kinematics.ToSwerveModuleVelocities(robotRelativeSpeeds);
  }

  /**
   * Convert module states back to robot-relative chassis speeds.
   *
   * @param states Module states (one per module, clockwise from front-left).
   * @return Robot-relative chassis speeds corresponding to the given states.
   */
  wpi::math::ChassisVelocities GetRobotRelativeChassisSpeedsFromState(
      const wpi::util::array<wpi::math::SwerveModuleVelocity, NumModules>& states) {
    return m_kinematics.ToChassisVelocities(states);
  }

  /**
   * Set robot-relative chassis speeds.
   *
   * @param robotRelativeSpeeds Desired robot-relative chassis speeds.
   */
  void SetRobotRelativeChassisSpeeds(wpi::math::ChassisVelocities robotRelativeSpeeds) {
    m_desiredChassisSpeeds = robotRelativeSpeeds;
    SetSwerveModuleStates(GetStateFromRobotRelativeChassisSpeeds(robotRelativeSpeeds));
  }

  /**
   * Set field-relative chassis speeds.
   *
   * @param fieldRelativeSpeeds Desired field-relative chassis speeds.
   */
  void SetFieldRelativeChassisSpeeds(wpi::math::ChassisVelocities fieldRelativeSpeeds) {
    SetRobotRelativeChassisSpeeds((fieldRelativeSpeeds).ToRobotRelative(wpi::math::Rotation2d{wpi::units::radian_t{GetGyroAngle()}}));
  }

  /**
   * Lock all modules to an X-pattern to resist being pushed.
   */
  void LockPose() {
    wpi::util::array<wpi::math::SwerveModuleVelocity, NumModules> states{wpi::util::empty_array};
    for (size_t i = 0; i < NumModules; ++i) {
      states[i] =
          wpi::math::SwerveModuleVelocity{wpi::units::meters_per_second_t{0},
                                 m_config->GetModules()[i]->GetConfig().GetLocation()->Angle()};
    }
    SetSwerveModuleStates(states);
    m_desiredChassisSpeeds = wpi::math::ChassisVelocities{};
  }

  // ---- Odometry & pose -------------------------------------------------------

  /**
   * Get the current estimated field-relative pose.
   */
  wpi::math::Pose2d GetPose() { return m_poseEstimator.GetEstimatedPosition(); }

  /**
   * Reset odometry to the given pose.
   *
   * @param pose New field-relative pose (blue-origin, 0° facing red alliance wall).
   */
  void ResetOdometry(wpi::math::Pose2d pose) {
    m_poseEstimator.ResetPosition(wpi::math::Rotation2d{wpi::units::radian_t{GetGyroAngle()}},
                                  GetModulePositions(), pose);
    m_desiredChassisSpeeds = wpi::math::ChassisVelocities{};
    m_desiredModuleStates = m_kinematics.ToSwerveModuleVelocities(wpi::math::ChassisVelocities{});
    m_simPose = pose;
  }

  /**
   * Zero the gyro and reset odometry to the current translation with 0° heading.
   */
  void ZeroGyro() {
    m_config->WithGyroOffset(GetGyroAngle() + m_config->GetGyroOffset());
    ResetOdometry(wpi::math::Pose2d{GetPose().Translation(), wpi::math::Rotation2d{}});
  }

  /**
   * Add a vision measurement to the pose estimator.
   *
   * @param robotPose Measured field-relative pose from vision.
   * @param timestamp FPGA timestamp of the measurement (seconds).
   */
  void AddVisionMeasurement(wpi::math::Pose2d robotPose, wpi::units::second_t timestamp) {
    m_poseEstimator.AddVisionMeasurement(robotPose, timestamp);
  }

  /**
   * Add a vision measurement with custom standard deviations.
   *
   * @param robotPose Measured field-relative pose from vision.
   * @param timestamp FPGA timestamp of the measurement (seconds).
   * @param stdDevs   Standard deviations {x_m, y_m, theta_rad}.
   */
  void AddVisionMeasurement(wpi::math::Pose2d robotPose, wpi::units::second_t timestamp,
                            const wpi::util::array<double, 3>& stdDevs) {
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

    m_swerveTelemetry->Publish(this);

    for (auto* mod : m_config->GetModules()) {
      mod->UpdateTelemetry();
    }
    m_telemetry.UpdateLoopTime();

    m_field2d.SetRobotPose(pose);
    std::vector<wpi::math::Pose2d> modulePoses;
    modulePoses.reserve(NumModules);
    for (size_t i = 0; i < NumModules; ++i) {
      auto* mod = m_config->GetModules()[i];
      auto location = *mod->GetConfig().GetLocation();
      auto moduleTranslation = pose.Translation() + location.RotateBy(pose.Rotation());
      auto moduleHeading = pose.Rotation() + currentStates[i].angle;
      modulePoses.push_back(wpi::math::Pose2d{moduleTranslation, moduleHeading});
    }
    m_field2d.GetObject("modules")->SetPoses(modulePoses);
  }

  /**
   * Advance the simulation model.
   *
   * Iterates each module's sim model, integrates the simulated ground-truth pose from the
   * commanded (desired) module states assuming they are met perfectly, and integrates the
   * simulated gyro angle from the measured chassis angular velocity.
   */
  void SimIterate() {
    if (!m_simTimer.IsRunning()) m_simTimer.Start();
    for (auto* mod : m_config->GetModules()) {
      mod->SimIterate();
    }
    auto desired = m_kinematics.ToChassisVelocities(m_desiredModuleStates);

    auto dt = m_simTimer.Get();
    wpi::math::Twist2d twist{desired.vx * dt, desired.vy * dt, desired.omega * dt};
    m_simPose = m_simPose + twist.Exp();

    auto speeds = m_kinematics.ToChassisVelocities(GetModuleStates());
    m_simGyroAngle += wpi::units::degree_t{
        wpi::units::degrees_per_second_t{speeds.omega}.value() * dt.value()};
    m_simTimer.Reset();
  }

  // ---- Queries ---------------------------------------------------------------

  /**
   * Get the current gyro angle.
   *
   * Returns the simulated angle in simulation, or the real gyro angle on hardware.
   */
  wpi::units::degree_t GetGyroAngle() {
    if (wpi::RobotBase::IsSimulation()) return m_simGyroAngle;
    return m_config->GetGyroAngle();
  }

  /**
   * Get the simulated pose of the robot, assuming the commanded module states are met perfectly.
   * Useful for feeding a simulated vision system with ground-truth poses under perfect-world
   * conditions.
   *
   * Only updated in simulation by SimIterate(); on a real robot this remains the configured
   * starting pose.
   */
  wpi::math::Pose2d GetSimPose() { return m_simPose; }

  /** Get the robot-relative chassis speeds derived from the module states. */
  wpi::math::ChassisVelocities GetRobotRelativeSpeed() {
    return m_kinematics.ToChassisVelocities(GetModuleStates());
  }

  /** Get the field-relative chassis speeds derived from the module states and gyro. */
  wpi::math::ChassisVelocities GetFieldRelativeSpeed() {
    return GetRobotRelativeSpeed().ToFieldRelative(
        wpi::math::Rotation2d{wpi::units::radian_t{GetGyroAngle()}});
  }

  /** Get the current module positions (distance + angle). */
  wpi::util::array<wpi::math::SwerveModulePosition, NumModules> GetModulePositions() {
    wpi::util::array<wpi::math::SwerveModulePosition, NumModules> positions{wpi::util::empty_array};
    for (size_t i = 0; i < NumModules; ++i) {
      positions[i] = m_config->GetModules()[i]->GetPosition();
    }
    return positions;
  }

  /** Get the current module states (speed + angle). */
  wpi::util::array<wpi::math::SwerveModuleVelocity, NumModules> GetModuleStates() {
    wpi::util::array<wpi::math::SwerveModuleVelocity, NumModules> states{wpi::util::empty_array};
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
  wpi::units::meter_t GetDistanceFromPose(wpi::math::Pose2d pose) {
    return wpi::units::meter_t{GetPose().Translation().Distance(pose.Translation())};
  }

  /**
   * Get the heading difference from the robot's current rotation to the given pose.
   *
   * @param pose Target pose.
   * @return Heading difference in degrees.
   */
  wpi::units::degree_t GetAngleDifferenceFromPose(wpi::math::Pose2d pose) {
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

  void ResetRotationPID() { m_config->GetRotationPID().Reset(); }
  void ResetTranslationPID() { m_config->GetTranslationPID().Reset(); }

  /**
   * Set the auto-align rotational PID controller.
   *
   * @param controller PIDController to use, input units are radians.
   */
  void SetRotationPID(wpi::math::PIDController controller) {
    auto& currentRotationPID = m_config->GetRotationPID();
    if (currentRotationPID.GetP() != controller.GetP() ||
        currentRotationPID.GetI() != controller.GetI() ||
        currentRotationPID.GetD() != controller.GetD()) {
      controller.Reset();
      m_config->WithRotationController(std::move(controller));
    }
  }

  /**
   * Set the auto-align translation PID controller.
   *
   * @param controller PIDController to use, input units are metres.
   */
  void SetTranslationPID(wpi::math::PIDController controller) {
    auto& currentTranslationPID = m_config->GetTranslationPID();
    if (currentTranslationPID.GetP() != controller.GetP() ||
        currentTranslationPID.GetI() != controller.GetI() ||
        currentTranslationPID.GetD() != controller.GetD()) {
      controller.Reset();
      m_config->WithTranslationController(std::move(controller));
    }
  }

  /**
   * Set the standard deviations used for the vision pose estimator noise model.
   *
   * @param stdDevs Standard deviations {x_m, y_m, theta_rad}.
   */
  void SetVisionMeasurementStdDevs(const wpi::util::array<double, 3>& stdDevs) {
    m_poseEstimator.SetVisionMeasurementStdDevs(stdDevs);
  }

  /** Get the drive's telemetry name (from SwerveDriveConfig, defaults to "swerve"). */
  std::string GetName() const { return m_config->GetTelemetryName(); }

  /** Get a mutable reference to the drive configuration. */
  SwerveDriveConfig& GetConfig() { return *m_config; }

  /** Get the SwerveDriveKinematics object. */
  wpi::math::SwerveDriveKinematics<NumModules>& GetKinematics() { return m_kinematics; }

  /**
   * Get the Field2d used to display the robot's pose, so callers (e.g. vision subsystems)
   * can publish additional objects onto the same field widget instead of creating their own.
   */
  wpi::Field2d& GetField2d() { return m_field2d; }

  /**
   * Get the last-commanded desired robot-relative chassis speeds. This is the value cached by
   * SetRobotRelativeChassisSpeeds() (which SetFieldRelativeChassisSpeeds() and Drive() funnel
   * through) and published on every UpdateTelemetry() call; it is a setpoint, not a measurement
   * of actual robot motion. Use GetRobotRelativeSpeed() or GetFieldRelativeSpeed() instead if you
   * need the drive's actual measured speed.
   *
   * @return Robot-relative chassis speeds last commanded to the drive. Defaults to a zeroed
   *         wpi::math::ChassisVelocities if the drive has never been commanded.
   */
  wpi::math::ChassisVelocities GetDesiredChassisSpeeds() { return m_desiredChassisSpeeds; }

  /**
   * Get the last-commanded desired module states of the drive.
   *
   * @return Module states last commanded to the drive. Defaults to a zeroed set of module states
   *         if the drive has never been commanded.
   */
  wpi::util::array<wpi::math::SwerveModuleVelocity, NumModules> GetDesiredModuleStates() {
    return m_desiredModuleStates;
  }

 private:
  SwerveDriveConfig* m_config{nullptr};
  wpi::math::SwerveDriveKinematics<NumModules> m_kinematics;
  wpi::math::SwerveDrivePoseEstimator<NumModules> m_poseEstimator;

  telemetry::MechanismTelemetry m_telemetry;
  std::optional<telemetry::SwerveDriveTelemetry> m_swerveTelemetry;
  std::optional<wpi::cmd::CommandPtr> m_driveToPoseTuningCommand;

  wpi::Field2d m_field2d;
  wpi::Timer m_simTimer;
  wpi::units::degree_t m_simGyroAngle{0};
  wpi::math::Pose2d m_simPose;

  wpi::util::array<wpi::math::SwerveModuleVelocity, NumModules> m_desiredModuleStates{wpi::util::empty_array};
  wpi::math::ChassisVelocities m_desiredChassisSpeeds{};

  /**
   * Setup telemetry for the drive; the SwerveDriveTelemetry config used is either the one
   * supplied via SwerveDriveConfig::WithTelemetry(SwerveDriveTelemetryConfig) or a default built
   * from SwerveDriveConfig::GetTelemetryVerbosity() (defaulting to TelemetryVerbosity::HIGH).
   */
  void SetupTelemetry() {
    using TelemetryVerbosity = motorcontrollers::SmartMotorControllerConfig::TelemetryVerbosity;
    telemetry::SwerveDriveTelemetryConfig telemetryCfg;
    if (auto specified = m_config->GetSwerveDriveTelemetryConfig()) {
      telemetryCfg = std::move(*specified);
    } else {
      telemetryCfg.WithTelemetryVerbosity(
          m_config->GetTelemetryVerbosity().value_or(TelemetryVerbosity::HIGH));
    }

    if (auto dataLogName = telemetryCfg.GetDataLogName()) {
      m_telemetry.SetupTelemetry(GetName(), *dataLogName);
    } else {
      m_telemetry.SetupTelemetry(GetName());
    }

    m_swerveTelemetry.emplace(std::move(telemetryCfg));
    m_swerveTelemetry->SetupTelemetry(this);

    m_driveToPoseTuningCommand.emplace(wpi::cmd::StartRun(
        [this] {
          std::puts(
              "================= Starting SwerveDrive.driveToPoseTuning() =================\n");
          ResetTranslationPID();
          ResetRotationPID();
        },
        [this] { m_swerveTelemetry->ApplyTuningValues(this); }));
    wpi::SmartDashboard::PutData("Mechanisms/" + GetName() + "/tuning/driveToPose",
                                 m_driveToPoseTuningCommand->get());
  }

  void UpdatePoseEstimator() {
    m_poseEstimator.Update(wpi::math::Rotation2d{wpi::units::radian_t{GetGyroAngle()}}, GetModulePositions());
  }

  static wpi::math::SwerveDriveKinematics<NumModules> BuildKinematics(const SwerveDriveConfig& config) {
    assert(config.GetModules().size() == NumModules);
    wpi::util::array<wpi::math::Translation2d, NumModules> locations{wpi::util::empty_array};
    for (size_t i = 0; i < NumModules; ++i) {
      locations[i] = *config.GetModules()[i]->GetConfig().GetLocation();
    }
    return wpi::math::SwerveDriveKinematics<NumModules>{locations};
  }

  static wpi::math::Rotation2d ComputeInitialRotation(const SwerveDriveConfig& config) {
    if (wpi::RobotBase::IsSimulation()) return wpi::math::Rotation2d{};
    return wpi::math::Rotation2d{wpi::units::radian_t{config.GetGyroAngle()}};
  }

  static wpi::util::array<wpi::math::SwerveModulePosition, NumModules> ComputeInitialPositions(
      const SwerveDriveConfig& config) {
    wpi::util::array<wpi::math::SwerveModulePosition, NumModules> positions{wpi::util::empty_array};
    for (size_t i = 0; i < NumModules; ++i) {
      positions[i] = config.GetModules()[i]->GetPosition();
    }
    return positions;
  }
};

}  // namespace yams::mechanisms::swerve

namespace yams::telemetry {

// SwerveDriveTelemetry::SetupTelemetry()/Publish()/ApplyTuningValues() are templated on the
// SwerveDrive's module count so that SwerveDriveTelemetry itself need not be a template. Their
// bodies live here — after SwerveDrive<NumModules> is fully defined above — because this is the
// only place both SwerveDriveTelemetry's private members and SwerveDrive<NumModules>'s interface
// are simultaneously visible.

template <std::size_t NumModules>
void SwerveDriveTelemetry::SetupTelemetry(mechanisms::swerve::SwerveDrive<NumModules>* drive) {
  auto driveName = drive->GetName();
  for (auto* module : drive->GetConfig().GetModules()) {
    module->SetupTelemetry(driveName);
  }

  auto inst = wpi::nt::NetworkTableInstance::GetDefault();
  m_dataTable = inst.GetTable("Mechanisms")->GetSubTable(driveName);
  m_tuningTable = inst.GetTable("Tuning")->GetSubTable(driveName);

  bool nt4Enabled = m_config.GetNT4Enabled();
  auto dataLogName = m_config.GetDataLogName();

  for (auto& [field, dt] : m_config.GetDoubleFields()) {
    if (!dt.IsEnabled()) continue;
    if (nt4Enabled) dt.SetupNetworkTables(m_dataTable, m_tuningTable);
    if (dataLogName) dt.SetupDataLog(*dataLogName);
  }
  for (auto& [field, stt] : m_config.GetPoseFields()) {
    if (!stt.IsEnabled()) continue;
    if (nt4Enabled) stt.SetupNetworkTables(m_dataTable, m_tuningTable);
    if (dataLogName) stt.SetupDataLog(*dataLogName);
  }
  for (auto& [field, stt] : m_config.GetChassisSpeedsFields()) {
    if (!stt.IsEnabled()) continue;
    if (nt4Enabled) stt.SetupNetworkTables(m_dataTable, m_tuningTable);
    if (dataLogName) stt.SetupDataLog(*dataLogName);
  }
  for (auto& [field, stat] : m_config.GetModuleStatesFields()) {
    if (!stat.IsEnabled()) continue;
    if (nt4Enabled) stat.SetupNetworkTables(m_dataTable, m_tuningTable);
    if (dataLogName) stat.SetupDataLog(*dataLogName);
  }
}

template <std::size_t NumModules>
void SwerveDriveTelemetry::Publish(mechanisms::swerve::SwerveDrive<NumModules>* drive) {
  for (auto& [field, stt] : m_config.GetPoseFields()) {
    if (!stt.IsEnabled()) continue;
    switch (field) {
      case StructTelemetryField::Pose:
        stt.Set(drive->GetPose());
        break;
      default:
        break;
    }
  }
  for (auto& [field, stt] : m_config.GetChassisSpeedsFields()) {
    if (!stt.IsEnabled()) continue;
    switch (field) {
      case StructTelemetryField::DesiredRobotRelativeChassisSpeeds:
        stt.Set(drive->GetDesiredChassisSpeeds());
        break;
      case StructTelemetryField::CurrentRobotRelativeChassisSpeeds:
        stt.Set(drive->GetRobotRelativeSpeed());
        break;
      case StructTelemetryField::FieldRelativeChassisSpeeds:
        stt.Set(drive->GetFieldRelativeSpeed());
        break;
      default:
        break;
    }
  }
  for (auto& [field, dt] : m_config.GetDoubleFields()) {
    if (!dt.IsEnabled()) continue;
    switch (field) {
      case DoubleTelemetryField::Gyro:
        dt.Set(drive->GetGyroAngle().value());
        break;
      default:
        break;
    }
  }
  for (auto& [field, stat] : m_config.GetModuleStatesFields()) {
    if (!stat.IsEnabled()) continue;
    switch (field) {
      case StructArrayTelemetryField::DesiredModuleStates: {
        auto states = drive->GetDesiredModuleStates();
        stat.Set(std::vector<wpi::math::SwerveModuleVelocity>(states.begin(), states.end()));
        break;
      }
      case StructArrayTelemetryField::CurrentModuleStates: {
        auto states = drive->GetModuleStates();
        stat.Set(std::vector<wpi::math::SwerveModuleVelocity>(states.begin(), states.end()));
        break;
      }
      default:
        break;
    }
  }
}

template <std::size_t NumModules>
void SwerveDriveTelemetry::ApplyTuningValues(mechanisms::swerve::SwerveDrive<NumModules>* drive) {
  for (auto& [field, stt] : m_config.GetPoseFields()) {
    if (!stt.IsTunable()) continue;
    switch (field) {
      case StructTelemetryField::TargetPose:
        drive->SetRobotRelativeChassisSpeeds(drive->DriveToPoseSetpoint(stt.Get()));
        break;
      default:
        break;
    }
  }

  auto translationPID = drive->GetConfig().GetTranslationPID();
  auto rotationPID = drive->GetConfig().GetRotationPID();
  for (auto& [field, dt] : m_config.GetDoubleFields()) {
    if (!dt.IsTunable()) continue;
    switch (field) {
      case DoubleTelemetryField::TranslationP:
        translationPID.SetP(dt.Get());
        drive->SetTranslationPID(translationPID);
        break;
      case DoubleTelemetryField::TranslationI:
        translationPID.SetI(dt.Get());
        drive->SetTranslationPID(translationPID);
        break;
      case DoubleTelemetryField::TranslationD:
        translationPID.SetD(dt.Get());
        drive->SetTranslationPID(translationPID);
        break;
      case DoubleTelemetryField::RotationP:
        rotationPID.SetP(dt.Get());
        drive->SetRotationPID(rotationPID);
        break;
      case DoubleTelemetryField::RotationI:
        rotationPID.SetI(dt.Get());
        drive->SetRotationPID(rotationPID);
        break;
      case DoubleTelemetryField::RotationD:
        rotationPID.SetD(dt.Get());
        drive->SetRotationPID(rotationPID);
        break;
      default:
        break;
    }
  }
}

}  // namespace yams::telemetry
