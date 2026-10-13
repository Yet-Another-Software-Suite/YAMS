// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <any>
#include <functional>
#include <optional>
#include <string>
#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/math/kinematics/SwerveModuleVelocity.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/velocity.hpp>

#include "yams/gearing/GearBox.hpp"
#include "yams/mechanisms/swerve/SwerveModuleVelocityWithAzimuth.hpp"
#include "yams/motorcontrollers/SmartMotorController.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/telemetry/SwerveModuleTelemetryConfig.hpp"

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
  SwerveModuleConfig& WithAbsoluteEncoder(std::function<wpi::units::degree_t()> supplier);

  /**
   * Use an absolute encoder object from the azimuth motor controller's own vendor as the azimuth's
   * external encoder, as SmartMotorControllerConfig::WithExternalEncoder does. It replaces a
   * supplier given with WithAbsoluteEncoder, and is applied when the motor controllers are given
   * if they have not been yet.
   *
   * @param absoluteEncoder The encoder, such as a pointer to a CANcoder.
   * @return *this for chaining.
   */
  SwerveModuleConfig& WithExternalAbsoluteEncoder(std::any absoluteEncoder);

  /**
   * Set the coupling ratio: how far the drive motor turns when the azimuth turns once with the
   * wheel held still. On most modules turning the azimuth also turns the wheel a little, which
   * shows up as wheel travel that did not happen. With this set, the module takes it out of the
   * wheel's position and velocity, and adds it to the velocity it commands.
   *
   * @param couplingRatio Drive motor rotations per azimuth rotation, as a gearbox.
   * @return *this for chaining.
   */
  SwerveModuleConfig& WithCouplingRatio(std::optional<gearing::GearBox> couplingRatio);

  /**
   * Set the coupling ratio as a number. Zero turns it off.
   *
   * @param driveMotorRotationsPerAzimuthRotation Drive motor rotations per azimuth rotation.
   * @return *this for chaining.
   */
  SwerveModuleConfig& WithCouplingRatio(double driveMotorRotationsPerAzimuthRotation);

  /**
   * Set where the module is, measured from the center of rotation. The same as WithLocation.
   *
   * @param front Distance toward the front of the robot.
   * @param left Distance toward the left of the robot.
   * @return *this for chaining.
   */
  SwerveModuleConfig& WithDistanceFromCenterOfRotation(wpi::units::meter_t front,
                                                       wpi::units::meter_t left);

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
  SwerveModuleConfig& WithAbsoluteEncoderOffset(wpi::units::degree_t offset);

  /**
   * Set the module location from the robot center as a Cartesian Translation2d
   * (X = forward, Y = left) in metres.
   *
   * @param location Location of the module.
   * @return *this for chaining.
   */
  SwerveModuleConfig& WithLocation(wpi::math::Translation2d location);

  /**
   * Set the module location via separate forward (X) and left (Y) distances.
   *
   * @param front Distance forward from the centre of rotation.
   * @param left  Distance left from the centre of rotation.
   * @return *this for chaining.
   */
  SwerveModuleConfig& WithLocation(wpi::units::meter_t front, wpi::units::meter_t left);

  /**
   * Set the module location via polar coordinates.
   *
   * @param distance Radial distance from the centre of rotation.
   * @param angle    Angular position around the centre of rotation.
   * @return *this for chaining.
   */
  SwerveModuleConfig& WithLocation(wpi::units::meter_t distance, wpi::units::degree_t angle);

  /**
   * Set the wheel radius (derives the circumference used for linear-distance conversion).
   *
   * @param radius Wheel radius.
   * @return *this for chaining.
   */
  SwerveModuleConfig& WithWheelRadius(wpi::units::meter_t radius);

  /**
   * Set the wheel diameter (derives the circumference used for linear-distance conversion).
   *
   * @param diameter Wheel diameter.
   * @return *this for chaining.
   */
  SwerveModuleConfig& WithWheelDiameter(wpi::units::meter_t diameter);

  /**
   * Velocity deadband: a requested wheel speed at or below it stops the module and holds its
   * current azimuth instead of tracking.
   *
   * The default is 0.1 m/s. Pass std::nullopt to turn it off.
   *
   * @param speed Velocity deadband.
   * @return *this for chaining.
   */
  SwerveModuleConfig& WithVelocityDeadband(std::optional<wpi::units::meters_per_second_t> speed);

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

  /**
   * Set the telemetry name for this module and configure telemetry with an explicit
   * SwerveModuleTelemetryConfig, taking precedence over WithTelemetry(name, TelemetryVerbosity).
   *
   * @param name            NetworkTable key for this module's data.
   * @param telemetryConfig Config that specifies what to log.
   * @return *this for chaining.
   */
  SwerveModuleConfig& WithTelemetry(const std::string& name,
                                    telemetry::SwerveModuleTelemetryConfig telemetryConfig);

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
  std::optional<wpi::math::Translation2d> GetLocation() const;

  /** Returns true if state optimization is enabled. */
  bool GetStateOptimization() const;

  /**
   * Get the current absolute encoder angle with gearing and offset applied.
   *
   * Falls back to the azimuth motor's mechanism position if no encoder supplier is set.
   *
   * @return Absolute angle in degrees.
   */
  wpi::units::degree_t GetAbsoluteEncoderAngle() const;

  /**
   * Get the absolute encoder angle as a supplier without offsets applied.
   *
   * Falls back to the azimuth motor's mechanism position (plus its external encoder zero
   * offset, on a real robot) if no encoder supplier is set.
   *
   * @return Callable returning the absolute encoder angle without offsets.
   */
  std::function<wpi::units::degree_t()> GetRawAbsoluteEncoderAngle() const;

  /** Get the absolute encoder supplier, if configured via WithAbsoluteEncoder(). */
  std::optional<std::function<wpi::units::degree_t()>> GetAbsoluteEncoderSupplier() const;

  /**
   * Get the user-specified SwerveModuleTelemetryConfig, if configured via
   * WithTelemetry(name, SwerveModuleTelemetryConfig). Moves the config out of this
   * SwerveModuleConfig; intended to be called exactly once, when the owning SwerveModule sets up
   * its telemetry.
   *
   * @return SwerveModuleTelemetryConfig if configured.
   */
  std::optional<telemetry::SwerveModuleTelemetryConfig> GetSwerveModuleTelemetryConfig();

  /**
   * Apply all enabled optimizations (velocity deadband, state optimization, cosine compensation)
   * and return the resulting state.
   *
   * @param state Input state.
   * @return Optimized state.
   */
  wpi::math::SwerveModuleVelocity GetOptimizedState(wpi::math::SwerveModuleVelocity state) const;

  /**
   * Apply all enabled optimizations against a measured state.
   *
   * @param state Input state.
   * @param measured The module's measured state.
   * @return Optimized state.
   */
  wpi::math::SwerveModuleVelocity GetOptimizedState(
      wpi::math::SwerveModuleVelocity state,
      const swerve::SwerveModuleVelocityWithAzimuth& measured) const;

  /**
   * The scale cosine compensation applies to a wheel aimed at the desired angle: the cosine of the
   * azimuth error, and never below zero. It is 1 when cosine compensation is off.
   *
   * @param desiredAngle Angle the wheel is being sent to.
   * @return Scale from 0 to 1.
   */
  double GetCosineCompensationScalar(wpi::math::Rotation2d desiredAngle) const;

  /** Get the coupling ratio, if configured via WithCouplingRatio(). */
  std::optional<gearing::GearBox> GetCouplingRatio() const;

  /**
   * Get the state that turns the wheel to the given angle without driving it, as for locking the
   * drive in place. Unlike GetOptimizedState(), the velocity deadband does not hold the wheel
   * where it is, since the wheel is meant to turn without driving. With optimization enabled the
   * wheel turns to whichever of the angle and its opposite is nearer, as either holds the drive
   * the same way.
   *
   * @param angle Angle to turn the wheel to.
   * @return Zero speed state at the angle to turn the wheel to.
   */
  wpi::math::SwerveModuleVelocity GetLockedState(wpi::math::Rotation2d angle) const;

  /**
   * Get the state that turns the wheel to the given angle without driving it, against a measured
   * state.
   *
   * @param angle Angle to turn the wheel to.
   * @param measured The module's measured state.
   * @return Zero speed state at the angle to turn the wheel to.
   */
  wpi::math::SwerveModuleVelocity GetLockedState(
      wpi::math::Rotation2d angle, const swerve::SwerveModuleVelocityWithAzimuth& measured) const;

 private:
  motorcontrollers::SmartMotorController* m_driveMotor{nullptr};
  motorcontrollers::SmartMotorController* m_azimuthMotor{nullptr};
  std::optional<std::string> m_telemetryName;
  std::optional<TelemetryVerbosity> m_telemetryVerbosity;
  std::optional<telemetry::SwerveModuleTelemetryConfig> m_specifiedTelemetryConfig;
  std::optional<std::function<wpi::units::degree_t()>> m_absoluteEncoderSupplier;
  std::optional<wpi::units::degree_t> m_absoluteEncoderOffset;
  gearing::GearBox m_absoluteEncoderGearbox{1.0};
  bool m_stateOptimization{true};
  bool m_cosineCompensation{false};
  std::optional<gearing::GearBox> m_couplingRatio;
  std::any m_absoluteEncoder;
  std::optional<wpi::units::meters_per_second_t> m_velocityDeadband{
      wpi::units::meters_per_second_t{0.1}};
  /** Angle the wheel was last sent to, so optimization does not reverse it back and forth. */
  mutable std::optional<wpi::math::Rotation2d> m_lastCommandedAngle;
  std::optional<wpi::math::Translation2d> m_location;
  std::optional<wpi::units::meter_t> m_wheelCircumference;

  /**
   * Compute the cosine-compensated drive velocity for the given desired state.
   *
   * @param desiredState Desired SwerveModuleState to use.
   * @param currentAngle Current azimuth angle to compare against, read once per control cycle so
   *                     it stays consistent with whatever value was used earlier in that cycle
   *                     (e.g. for optimization).
   */
  double GetCosineCompensatedVelocity(const wpi::math::SwerveModuleVelocity& desiredState,
                                      const wpi::math::Rotation2d& currentAngle) const;

  /**
   * Angle the azimuth controller is closing its loop on (seeded from the absolute encoder), so
   * optimization, cosine compensation and the azimuth setpoint agree on where the wheel points.
   */
  wpi::math::Rotation2d GetAzimuthAngle() const;

  /** The measured state used when none is given: the azimuth controller's position and velocity. */
  swerve::SwerveModuleVelocityWithAzimuth GetMeasuredAzimuth() const;

  /**
   * Optimize a module state like SwerveModuleVelocity::Optimize(), keeping the orientation the
   * wheel was last sent to unless the other is better by more than 30 degrees. A wheel still
   * turning can be near 90 degrees from both orientations of a target that keeps moving, such as
   * a path starting off in a new direction; choosing by its angle alone would reverse it back and
   * forth.
   *
   * @param desired       Module state to optimize.
   * @param currentAngle  Angle the wheel is at.
   * @param lastCommanded Angle the wheel was last sent to, if it has been sent anywhere.
   * @return The optimized module state.
   */
  static wpi::math::SwerveModuleVelocity Optimize(
      wpi::math::SwerveModuleVelocity desired, const wpi::math::Rotation2d& currentAngle,
      const std::optional<wpi::math::Rotation2d>& lastCommanded);
};

}  // namespace yams::mechanisms::config
