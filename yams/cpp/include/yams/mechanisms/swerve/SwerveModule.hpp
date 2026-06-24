// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>

#include <string>

#include "yams/mechanisms/config/SwerveModuleConfig.hpp"
#include "yams/motorcontrollers/SmartMotorController.hpp"

namespace yams::mechanisms::swerve {

/**
 * A single swerve module comprising a drive motor and an azimuth (steering) motor.
 *
 * Each module holds references to two SmartMotorControllers (drive and azimuth) and
 * a SwerveModuleConfig that describes its physical properties and absolute encoder.
 * Modules are normally owned by the subsystem and handed to SwerveDrive by pointer.
 *
 * ### Example usage (inside a subsystem constructor)
 * @code{.cpp}
 * using namespace yams::motorcontrollers;
 * using namespace yams::motorcontrollers::remote;
 * using namespace yams::gearing;
 * using namespace yams::mechanisms::config;
 * using Cfg = SmartMotorControllerConfig;
 *
 * // Declare as subsystem members:
 * //   ctre::phoenix6::hardware::TalonFX     m_driveMotor{1};
 * //   ctre::phoenix6::hardware::TalonFX     m_azimuthMotor{2};
 * //   ctre::phoenix6::hardware::CANcoder    m_encoder{3};
 * //   std::optional<TalonFXWrapper>         m_driveSMC;
 * //   std::optional<TalonFXWrapper>         m_azimuthSMC;
 * //   std::optional<SwerveModule>           m_module;
 *
 * SmartMotorControllerConfig driveCfg;
 * driveCfg.WithSubsystem(this)
 *         .WithMechanismCircumference(units::meter_t{4.0_in * std::numbers::pi})
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
 * m_driveSMC.emplace(m_driveMotor, frc::DCMotor::KrakenX60(1), driveCfg);
 * m_azimuthSMC.emplace(m_azimuthMotor, frc::DCMotor::KrakenX60(1), azimuthCfg);
 *
 * auto* enc = &m_encoder;
 * SwerveModuleConfig moduleConfig{&m_driveSMC.value(), &m_azimuthSMC.value()};
 * moduleConfig
 *     .WithAbsoluteEncoder([enc]() -> units::degree_t {
 *       return units::degree_t{units::turn_t{enc->GetAbsolutePosition().GetValue()}};
 *     })
 *     .WithAbsoluteEncoderOffset(15.0_deg)
 *     .WithWheelDiameter(4.0_in)
 *     .WithLocation(units::inch_t{12}, units::inch_t{12})
 *     .WithOptimization(true)
 *     .WithCosineCompensation(true)
 *     .WithTelemetry("FrontLeft", Cfg::TelemetryVerbosity::HIGH);
 *
 * m_module.emplace(&m_moduleConfig);
 * @endcode
 */
class SwerveModule {
 public:
  /**
   * Construct a SwerveModule from a SwerveModuleConfig.
   *
   * @param config Module configuration.  Must contain a telemetry name and a location.
   * @throws std::invalid_argument if telemetry name or location is not set.
   */
  explicit SwerveModule(config::SwerveModuleConfig* config);

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
  config::SwerveModuleConfig* m_config{nullptr};

 public:
  // Public so SwerveDrive can access them directly (mirrors Java protected fields).
  motorcontrollers::SmartMotorController* m_driveMotorController;
  motorcontrollers::SmartMotorController* m_azimuthMotorController;
};

}  // namespace yams::mechanisms::swerve
