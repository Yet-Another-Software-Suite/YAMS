// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <functional>
#include <optional>
#include <string>
#include <wpi/math/kinematics/SwerveModulePosition.hpp>
#include <wpi/math/kinematics/SwerveModuleVelocity.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/force.hpp>
#include <wpi/units/voltage.hpp>

#include "yams/mechanisms/config/SwerveModuleConfig.hpp"
#include "yams/mechanisms/swerve/SwerveModuleVelocityWithAzimuth.hpp"
#include "yams/motorcontrollers/SmartMotorController.hpp"
#include "yams/telemetry/MechanismTelemetry.hpp"
#include "yams/telemetry/SwerveDriveTelemetryConfig.hpp"
#include "yams/telemetry/SwerveModuleTelemetry.hpp"

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
 *         .WithMechanismCircumference(wpi::units::meter_t{4.0_in * std::numbers::pi})
 *         .WithFeedback(0.1, 0.0, 0.0)
 *         .WithMotorGearing(MechanismGearing{GearBox::FromStages({"6.75:1"})})
 *         .WithStatorCurrentLimit(40.0_A)
 *         .WithZeroPower(Cfg::MotorMode::BRAKE)
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
 * m_driveSMC.emplace(m_driveMotor, wpi::math::DCMotor::KrakenX60(1), driveCfg);
 * m_azimuthSMC.emplace(m_azimuthMotor, wpi::math::DCMotor::KrakenX60(1), azimuthCfg);
 *
 * auto* enc = &m_encoder;
 * SwerveModuleConfig moduleConfig{&m_driveSMC.value(), &m_azimuthSMC.value()};
 * moduleConfig
 *     .WithAbsoluteEncoder([enc]() -> wpi::units::degree_t {
 *       return wpi::units::degree_t{wpi::units::turn_t{enc->GetAbsolutePosition().GetValue()}};
 *     })
 *     .WithAbsoluteEncoderOffset(15.0_deg)
 *     .WithWheelDiameter(4.0_in)
 *     .WithLocation(wpi::units::inch_t{12}, wpi::units::inch_t{12})
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
   * Setup telemetry for the module; the SwerveModuleTelemetry config used is either the one
   * supplied via SwerveModuleConfig::WithTelemetry(name, SwerveModuleTelemetryConfig) or a
   * default built from SwerveModuleConfig::GetTelemetryVerbosity() (defaulting to
   * TelemetryVerbosity::HIGH).
   *
   * @param mechName Telemetry name of the parent SwerveDrive.
   */
  void SetupTelemetry(const std::string& mechName);

  /**
   * Seed the azimuth encoder from the absolute encoder reading (real robot only).
   */
  void SeedAzimuthEncoder();

  /** Get the module name (from telemetry config). */
  std::string GetName() const;

  /** Get the absolute encoder angle without any offsets applied. */
  wpi::units::degree_t GetRawAbsoluteEncoderAngle() const;

  /** Get the module configuration. */
  const config::SwerveModuleConfig& GetConfig() const;

  /**
   * Command the module to a target state.
   *
   * Applies optimization (direction flip, velocity deadband, cosine compensation)
   * before setting drive velocity and azimuth position.
   *
   * @param state Target module state.
   */
  wpi::math::SwerveModuleVelocity SetSwerveModuleState(wpi::math::SwerveModuleVelocity state);

  /**
   * Set the swerve module state with a feedforward force on the drive wheel, for following a
   * trajectory that supplies module forces. The force follows the state: it is reversed when
   * optimization reverses the wheel, and scaled by cosine compensation.
   *
   * @param state Desired module state.
   * @param feedforwardForce Force the wheel should apply along the requested direction.
   * @return The state the module was sent, after optimization.
   */
  wpi::math::SwerveModuleVelocity SetSwerveModuleState(wpi::math::SwerveModuleVelocity state,
                                                       wpi::units::newton_t feedforwardForce);

  /** Get the module config for changing it, as WithModuleStateOptimization does. */
  config::SwerveModuleConfig& GetMutableConfig();

  /**
   * Turn the wheel to the given angle without driving it, as for locking the drive in place or
   * tuning the azimuth motor. The velocity deadband is skipped, since it would hold the wheel where
   * it is.
   *
   * @param angle Angle to turn the wheel to.
   * @return The state the module was commanded to.
   */
  wpi::math::SwerveModuleVelocity SetAzimuthAngle(wpi::math::Rotation2d angle);

  /**
   * Drive the wheel at a voltage, without the drive motor's closed-loop controller, with the wheel
   * turned to an angle, as for characterizing the drive motor. The angle is not optimized, so the
   * wheel is always driven the same way. Stop the drive motor's closed-loop controller first, or
   * one run on the SystemCore replaces the voltage on its next loop.
   *
   * @param voltage Voltage to drive the wheel at.
   * @param angle Angle to turn the wheel to.
   */
  void SetDriveVoltage(wpi::units::volt_t voltage, wpi::math::Rotation2d angle);

  /**
   * Turn the wheel at a voltage, without the azimuth motor's closed-loop controller, and without
   * driving it, as for characterizing the azimuth motor. Stop the azimuth motor's closed-loop
   * controller first, or one run on the SystemCore replaces the voltage on its next loop.
   *
   * @param voltage Voltage to turn the wheel at.
   */
  void SetAzimuthVoltage(wpi::units::volt_t voltage);

  /**
   * Apply a live tuning gain from the SwerveDrive's telemetry to this module's drive or azimuth
   * motor controller. Fields that are not a module gain are ignored.
   *
   * @param field The tuning field.
   * @param value Its value.
   */
  void ApplyTuningValue(telemetry::SwerveDriveTelemetryConfig::DoubleTelemetryField field,
                        double value);

  /** Get the current measured module state (speed and heading). */
  SwerveModuleVelocityWithAzimuth GetState() const;

  /** Get the current measured module position (distance and heading). */
  wpi::math::SwerveModulePosition GetPosition() const;

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
  /** Used for the loop time only; module state/encoder telemetry lives in m_swerveModuleTelemetry.
   */
  telemetry::MechanismTelemetry m_telemetry;
  std::optional<telemetry::SwerveModuleTelemetry> m_swerveModuleTelemetry;
  std::function<wpi::units::degree_t()> m_azimuthEncoderWithoutOffsets;
  // Azimuth travel, unwrapped, for taking the coupled wheel travel out of the drive position.
  mutable std::optional<wpi::math::Rotation2d> m_lastAzimuthAngle;
  mutable wpi::units::turn_t m_azimuthTravel{0};

  /** Wheel travel the drive encoder reads because the azimuth turned by the given angle. */
  wpi::units::meter_t GetCouplingDistance(wpi::units::turn_t azimuthAngle) const;
  /** The drive motor's rotor to mechanism ratio, or 1 when no gearing is configured. */
  double GetDriveRotorToMechanismRatio() const;
  /** Wheel velocity the drive encoder reads because the azimuth is turning. */
  wpi::units::meters_per_second_t GetCouplingVelocity() const;

 public:
  // Public so SwerveDrive can access them directly (mirrors Java protected fields).
  motorcontrollers::SmartMotorController* m_driveMotorController;
  motorcontrollers::SmartMotorController* m_azimuthMotorController;
};

}  // namespace yams::mechanisms::swerve
