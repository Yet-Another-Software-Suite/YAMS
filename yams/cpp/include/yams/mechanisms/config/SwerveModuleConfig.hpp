// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>

#include <functional>
#include <optional>
#include <string>

#include "yams/gearing/GearBox.hpp"
#include "yams/motorcontrollers/SmartMotorController.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"

namespace yams::mechanisms::config {

/**
 * Configuration for a single SwerveModule.
 *
 * Uses a fluent builder pattern; all With* methods return *this for chaining.
 */
class SwerveModuleConfig {
 public:
  using TelemetryVerbosity = motorcontrollers::SmartMotorControllerConfig::TelemetryVerbosity;

  SwerveModuleConfig() = default;

  /**
   * Construct with drive and azimuth motor controllers.
   *
   * @param drive   Drive motor controller (must outlive this config).
   * @param azimuth Azimuth motor controller (must outlive this config).
   */
  SwerveModuleConfig(motorcontrollers::SmartMotorController* drive,
                     motorcontrollers::SmartMotorController* azimuth);

  // ---- Builder methods -------------------------------------------------------

  /**
   * Set both motor controllers (use when constructed with the default constructor).
   *
   * @param driveMotor   Drive motor controller.
   * @param azimuthMotor Azimuth motor controller.
   * @return *this for chaining.
   */
  SwerveModuleConfig& WithSmartMotorController(
      motorcontrollers::SmartMotorController* driveMotor,
      motorcontrollers::SmartMotorController* azimuthMotor);

  /**
   * Enable or disable cosine compensation of the drive velocity.
   *
   * Scales drive speed by cos(angle_error) to reduce skew during steering.
   *
   * @param compensate true to enable.
   * @return *this for chaining.
   */
  SwerveModuleConfig& WithCosineCompensation(bool compensate);

  /**
   * Supply an external (cross-vendor) absolute encoder angle in degrees.
   *
   * @param supplier Callable that returns the current encoder angle.
   * @return *this for chaining.
   */
  SwerveModuleConfig& WithAbsoluteEncoder(std::function<units::degree_t()> supplier);

  /**
   * Set the gearing between the absolute encoder shaft and the azimuth mechanism.
   *
   * @param gearing GearBox representing the encoder-to-mechanism ratio.
   * @return *this for chaining.
   */
  SwerveModuleConfig& WithAbsoluteEncoderGearing(const gearing::GearBox& gearing);

  /**
   * Set the absolute encoder offset so that zero corresponds to the wheel pointing
   * forward with the bevel gear facing left.
   *
   * @param offset Offset angle to subtract from the raw encoder reading.
   * @return *this for chaining.
   */
  SwerveModuleConfig& WithAbsoluteEncoderOffset(units::degree_t offset);

  /**
   * Set the module location from the robot center as a Cartesian Translation2d
   * (X = forward, Y = left) in metres.
   *
   * @param location Location of the module.
   * @return *this for chaining.
   */
  SwerveModuleConfig& WithLocation(frc::Translation2d location);

  /**
   * Set the module location via separate forward (X) and left (Y) distances.
   *
   * @param front Distance forward from the centre of rotation.
   * @param left  Distance left from the centre of rotation.
   * @return *this for chaining.
   */
  SwerveModuleConfig& WithLocation(units::meter_t front, units::meter_t left);

  /**
   * Set the module location via polar coordinates.
   *
   * @param distance Radial distance from the centre of rotation.
   * @param angle    Angular position around the centre of rotation.
   * @return *this for chaining.
   */
  SwerveModuleConfig& WithLocation(units::meter_t distance, units::degree_t angle);

  /**
   * Set the wheel radius (derives the circumference used for linear-distance conversion).
   *
   * @param radius Wheel radius.
   * @return *this for chaining.
   */
  SwerveModuleConfig& WithWheelRadius(units::meter_t radius);

  /**
   * Set the wheel diameter (derives the circumference used for linear-distance conversion).
   *
   * @param diameter Wheel diameter.
   * @return *this for chaining.
   */
  SwerveModuleConfig& WithWheelDiameter(units::meter_t diameter);

  /**
   * Minimum speed below which the module holds its current azimuth instead of tracking.
   *
   * @param speed Minimum speed threshold.
   * @return *this for chaining.
   */
  SwerveModuleConfig& WithMinimumVelocity(units::meters_per_second_t speed);

  /**
   * Enable or disable WPILib SwerveModuleState optimization (flip by 180° to minimise rotation).
   *
   * @param enable true to enable (default is true).
   * @return *this for chaining.
   */
  SwerveModuleConfig& WithOptimization(bool enable);

  /**
   * Set the telemetry name and verbosity for this module.
   *
   * @param name      NetworkTable key for this module's data.
   * @param verbosity Amount of data to publish.
   * @return *this for chaining.
   */
  SwerveModuleConfig& WithTelemetry(const std::string& name, TelemetryVerbosity verbosity);

  // ---- Getters ---------------------------------------------------------------

  /** Get the drive motor controller pointer. */
  motorcontrollers::SmartMotorController* GetDriveMotor() const;

  /** Get the azimuth motor controller pointer. */
  motorcontrollers::SmartMotorController* GetAzimuthMotor() const;

  /** Get the optional telemetry name. */
  std::optional<std::string> GetTelemetryName() const;

  /** Get the optional telemetry verbosity. */
  std::optional<TelemetryVerbosity> GetTelemetryVerbosity() const;

  /** Get the optional module location (Translation2d from centre of rotation). */
  std::optional<frc::Translation2d> GetLocation() const;

  /** Returns true if state optimization is enabled. */
  bool GetStateOptimization() const;

  /**
   * Get the current absolute encoder angle with gearing and offset applied.
   *
   * Falls back to the azimuth motor's mechanism position if no encoder supplier is set.
   *
   * @return Absolute angle in degrees.
   */
  units::degree_t GetAbsoluteEncoderAngle() const;

  /**
   * Apply all enabled optimizations (min-velocity clamp, state optimization, cosine compensation)
   * and return the resulting state.
   *
   * @param state Input state.
   * @return Optimized state.
   */
  frc::SwerveModuleState GetOptimizedState(frc::SwerveModuleState state) const;

 private:
  motorcontrollers::SmartMotorController* m_driveMotor{nullptr};
  motorcontrollers::SmartMotorController* m_azimuthMotor{nullptr};
  std::optional<std::string> m_telemetryName;
  std::optional<TelemetryVerbosity> m_telemetryVerbosity;
  std::optional<std::function<units::degree_t()>> m_absoluteEncoderSupplier;
  std::optional<units::degree_t> m_absoluteEncoderOffset;
  gearing::GearBox m_absoluteEncoderGearbox{1.0};
  bool m_stateOptimization{true};
  bool m_cosineCompensation{false};
  std::optional<units::meters_per_second_t> m_minimumVelocity;
  std::optional<frc::Translation2d> m_location;
  std::optional<units::meter_t> m_wheelCircumference;

  /**
   * Compute the cosine-compensated drive velocity for the given desired state.
   */
  double GetCosineCompensatedVelocity(const frc::SwerveModuleState& desiredState) const;
};

}  // namespace yams::mechanisms::config
