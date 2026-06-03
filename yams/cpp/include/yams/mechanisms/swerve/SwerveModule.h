// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>

#include <string>

#include "yams/mechanisms/config/SwerveModuleConfig.h"
#include "yams/motorcontrollers/SmartMotorController.h"

namespace yams::mechanisms::swerve {

/**
 * A single swerve module comprising a drive motor and an azimuth (steering) motor.
 */
class SwerveModule {
 public:
  /**
   * Construct a SwerveModule from a SwerveModuleConfig.
   *
   * @param config Module configuration.  Must contain a telemetry name and a location.
   * @throws std::invalid_argument if telemetry name or location is not set.
   */
  explicit SwerveModule(const config::SwerveModuleConfig& config);

  /**
   * Seed the azimuth encoder from the absolute encoder reading (real robot only).
   */
  void SeedAzimuthEncoder();

  /** Get the module name (from telemetry config). */
  std::string GetName() const;

  /** Get the module configuration. */
  const config::SwerveModuleConfig& GetConfig() const;

  /**
   * Command the module to a target state.
   *
   * Applies optimization (direction flip, min-velocity clamp, cosine compensation)
   * before setting drive velocity and azimuth position.
   *
   * @param state Target module state.
   */
  void SetSwerveModuleState(frc::SwerveModuleState state);

  /** Get the current measured module state (speed and heading). */
  frc::SwerveModuleState GetState() const;

  /** Get the current measured module position (distance and heading). */
  frc::SwerveModulePosition GetPosition() const;

  /** Publish drive and azimuth motor telemetry to NetworkTables. */
  void UpdateTelemetry();

  /** Advance the simulation model for both drive and azimuth motors. */
  void SimIterate();

  /** Get the drive motor controller. */
  motorcontrollers::SmartMotorController* GetDriveMotorController() const;

  /** Get the azimuth motor controller. */
  motorcontrollers::SmartMotorController* GetAzimuthMotorController() const;

 private:
  config::SwerveModuleConfig m_config;

 public:
  // Public so SwerveDrive can access them for SysId routines (mirrors Java protected fields).
  motorcontrollers::SmartMotorController* m_driveMotorController;
  motorcontrollers::SmartMotorController* m_azimuthMotorController;
};

}  // namespace yams::mechanisms::swerve
