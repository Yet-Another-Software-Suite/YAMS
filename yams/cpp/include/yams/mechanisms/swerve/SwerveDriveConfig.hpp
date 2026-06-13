// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/controller/PIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include <functional>
#include <optional>
#include <vector>

#include "yams/mechanisms/swerve/SwerveModule.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"

namespace yams::mechanisms::swerve {

/**
 * Configuration for a SwerveDrive.
 *
 * Uses a fluent builder pattern; all With* methods return *this for chaining.
 */
class SwerveDriveConfig {
 public:
  using TelemetryVerbosity = motorcontrollers::SmartMotorControllerConfig::TelemetryVerbosity;

  SwerveDriveConfig() = default;

  // ---- Builder methods -------------------------------------------------------

  /**
   * Associate a command subsystem with this drive (used for command requirements).
   *
   * @param subsystem Pointer to the subsystem (must outlive this config).
   * @return *this for chaining.
   */
  SwerveDriveConfig& WithSubsystem(frc2::SubsystemBase* subsystem);

  /**
   * Set the swerve modules (must match the NumModules template argument of SwerveDrive).
   *
   * @param modules Vector of module pointers (must outlive this config).
   * @return *this for chaining.
   */
  SwerveDriveConfig& WithModules(std::vector<SwerveModule*> modules);

  /**
   * Supply the gyro angle in degrees.
   *
   * @param gyroSupplier Callable returning the current gyro angle.
   * @return *this for chaining.
   */
  SwerveDriveConfig& WithGyro(std::function<units::degree_t()> gyroSupplier);

  /**
   * Supply the gyro angular velocity in degrees per second.
   *
   * Required for angular-velocity skew correction.
   *
   * @param angularVelocitySupplier Callable returning the current gyro angular velocity.
   * @return *this for chaining.
   */
  SwerveDriveConfig& WithGyroVelocity(
      std::function<units::degrees_per_second_t()> angularVelocitySupplier);

  /**
   * Set the gyro offset (subtracted from the raw gyro reading in GetGyroAngle).
   *
   * @param offset Gyro angle offset.
   * @return *this for chaining.
   */
  SwerveDriveConfig& WithGyroOffset(units::degree_t offset);

  /**
   * Invert the gyro direction.
   *
   * @param inverted true to negate the raw gyro angle.
   * @return *this for chaining.
   */
  SwerveDriveConfig& WithGyroInverted(bool inverted);

  /**
   * Set the starting field pose of the robot.
   *
   * @param pose Starting Pose2d (field-relative, blue-origin).
   * @return *this for chaining.
   */
  SwerveDriveConfig& WithStartingPose(frc::Pose2d pose);

  /**
   * Set the maximum chassis linear and angular speeds for wheel speed desaturation.
   *
   * @param speed           Maximum chassis linear speed.
   * @param angularVelocity Maximum chassis angular speed.
   * @return *this for chaining.
   */
  SwerveDriveConfig& WithMaximumChassisSpeed(units::meters_per_second_t speed,
                                             units::degrees_per_second_t angularVelocity);

  /**
   * Set the maximum module linear speed for wheel speed desaturation.
   *
   * @param speed Maximum module speed.
   * @return *this for chaining.
   */
  SwerveDriveConfig& WithMaximumModuleSpeed(units::meters_per_second_t speed);

  /**
   * Override the chassis centre of rotation (default is the robot centre).
   *
   * @param center Translation2d in metres (X forward, Y left).
   * @return *this for chaining.
   */
  SwerveDriveConfig& WithCenterOfRotation(frc::Translation2d center);

  /**
   * Override the chassis centre of rotation via forward/left distances.
   *
   * @param forward Distance forward from the robot centre.
   * @param left    Distance left from the robot centre.
   * @return *this for chaining.
   */
  SwerveDriveConfig& WithCenterOfRotation(units::meter_t forward, units::meter_t left);

  /**
   * Set the discretization timestep used to compensate for ChassisSpeeds skew.
   *
   * @param dt Discretization period (typically one robot loop period).
   * @return *this for chaining.
   */
  SwerveDriveConfig& WithDiscretizationTime(units::second_t dt);

  /**
   * Set the discretization timestep used during simulation.
   *
   * Falls back to WithDiscretizationTime if not set.
   *
   * @param dt Discretization period for simulation.
   * @return *this for chaining.
   */
  SwerveDriveConfig& WithSimDiscretizationTime(units::second_t dt);

  /**
   * Set the angular velocity scale factor for skew correction on the real robot.
   *
   * @param scaleFactor Scale applied to the gyro angular velocity [0, 1].
   * @return *this for chaining.
   */
  SwerveDriveConfig& WithGyroAngularVelocityScaleFactor(double scaleFactor);

  /**
   * Set the angular velocity scale factor for skew correction in simulation.
   *
   * Falls back to WithGyroAngularVelocityScaleFactor if not set.
   *
   * @param scaleFactor Scale applied to the gyro angular velocity [0, 1].
   * @return *this for chaining.
   */
  SwerveDriveConfig& WithSimGyroAngularVelocityScaleFactor(double scaleFactor);

  /**
   * Set the translation PID controller for DriveToPose (input units: metres).
   *
   * @param controller PIDController for X/Y translation.
   * @return *this for chaining.
   */
  SwerveDriveConfig& WithTranslationController(frc::PIDController controller);

  /**
   * Set the rotation PID controller for DriveToPose (input units: radians).
   *
   * Continuous input over [-π, π] is enabled automatically.
   *
   * @param controller PIDController for heading.
   * @return *this for chaining.
   */
  SwerveDriveConfig& WithRotationController(frc::PIDController controller);

  /**
   * Set the simulated translation PID controller for DriveToPose (input units: metres).
   *
   * Falls back to WithTranslationController if not set.
   *
   * @param controller PIDController for X/Y translation in simulation.
   * @return *this for chaining.
   */
  SwerveDriveConfig& WithSimTranslationController(frc::PIDController controller);

  /**
   * Set the simulated rotation PID controller for DriveToPose (input units: radians).
   *
   * Falls back to WithRotationController if not set.  Continuous input is enabled automatically.
   *
   * @param controller PIDController for heading in simulation.
   * @return *this for chaining.
   */
  SwerveDriveConfig& WithSimRotationController(frc::PIDController controller);

  /**
   * Set the telemetry verbosity for the drive.
   *
   * @param verbosity Amount of data to publish.
   * @return *this for chaining.
   */
  SwerveDriveConfig& WithTelemetry(TelemetryVerbosity verbosity);

  // ---- Getters ---------------------------------------------------------------

  /** Get the vector of module pointers. */
  const std::vector<SwerveModule*>& GetModules() const;

  /** Get the subsystem pointer. */
  frc2::SubsystemBase* GetSubsystem() const;

  /** Get the starting pose. */
  frc::Pose2d GetInitialPose() const;

  /** @return Optional maximum chassis linear speed for wheel-speed desaturation. */
  std::optional<units::meters_per_second_t> GetMaximumChassisLinearVelocity() const;

  /** @return Optional maximum chassis angular speed for wheel-speed desaturation. */
  std::optional<units::degrees_per_second_t> GetMaximumChassisAngularVelocity() const;
  /** @return Optional maximum module linear speed for wheel-speed desaturation. */
  std::optional<units::meters_per_second_t> GetMaximumModuleLinearVelocity() const;
  /** @return Optional override centre of rotation (empty = robot centre). */
  std::optional<frc::Translation2d> GetCenterOfRotation() const;
  /** @return Optional telemetry verbosity level. */
  std::optional<TelemetryVerbosity> GetTelemetryVerbosity() const;

  /**
   * Get the stored gyro offset (zero if never set).
   */
  units::degree_t GetGyroOffset() const;

  /**
   * Get the gyro angle with inversion and offset applied.
   *
   * @throws std::runtime_error if no gyro supplier has been configured.
   */
  units::degree_t GetGyroAngle() const;

  /**
   * Apply angular-velocity skew correction and ChassisSpeeds discretization.
   *
   * @param speeds Raw robot-relative chassis speeds.
   * @return Optimized chassis speeds.
   */
  frc::ChassisSpeeds OptimizeRobotRelativeChassisSpeeds(frc::ChassisSpeeds speeds) const;

  /**
   * Get the active translation PID controller (sim variant if in simulation and configured).
   *
   * @throws std::logic_error if no translation controller has been set.
   */
  frc::PIDController& GetTranslationPID();

  /**
   * Get the active rotation PID controller (sim variant if in simulation and configured).
   *
   * @throws std::logic_error if no rotation controller has been set.
   */
  frc::PIDController& GetRotationPID();

  /** Cube the polar-coordinate magnitude of a Translation2d. */
  static frc::Translation2d CubeTranslation(frc::Translation2d translation);

  /** Scale the polar-coordinate magnitude of a Translation2d by a scalar. */
  static frc::Translation2d ScaleTranslation(frc::Translation2d translation, double scalar);

 private:
  std::vector<SwerveModule*> m_modules;
  frc2::SubsystemBase* m_subsystem{nullptr};
  std::optional<TelemetryVerbosity> m_telemetryVerbosity;
  std::optional<std::function<units::degree_t()>> m_gyroSupplier;
  std::optional<std::function<units::degrees_per_second_t()>> m_gyroAngularVelocitySupplier;
  std::optional<units::degree_t> m_gyroOffset;
  bool m_gyroInverted{false};
  frc::Pose2d m_initialPose;
  std::optional<units::meters_per_second_t> m_maximumChassisLinearVelocity;
  std::optional<units::degrees_per_second_t> m_maximumChassisAngularVelocity;
  std::optional<units::meters_per_second_t> m_maximumModuleLinearVelocity;
  std::optional<units::second_t> m_discretizationSeconds;
  std::optional<units::second_t> m_simDiscretizationSeconds;
  std::optional<double> m_angularVelocityScaleFactor;
  std::optional<double> m_simAngularVelocityScaleFactor;
  std::optional<frc::Translation2d> m_centerOfRotation;
  std::optional<frc::PIDController> m_translationController;
  std::optional<frc::PIDController> m_rotationController;
  std::optional<frc::PIDController> m_simTranslationController;
  std::optional<frc::PIDController> m_simRotationController;

  frc::ChassisSpeeds AngularVelocitySkewCorrection(frc::ChassisSpeeds robotRelativeSpeeds) const;
};

}  // namespace yams::mechanisms::swerve
