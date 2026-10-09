// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <thrifty/canEncoder/CanEncoder.h>
#include <thrifty/core/Motor.h>
#include <thrifty/nova/Nova.h>

#include <optional>
#include <string>
#include <wpi/simulation/DCMotorSim.hpp>
#include <wpi/util/Alert.hpp>

#include "yams/math/DerivativeTimeFilter.hpp"
#include "yams/motorcontrollers/SmartMotorController.hpp"

namespace yams::motorcontrollers::local {

/**
 * SmartMotorController implementation for The Thrifty Bot's Thrifty Nova, using ThriftyLib 2027.
 *
 * The closed loop controller runs on the SystemCore and commands the Nova with voltage, like the
 * profiled controllers of the other wrappers. ThriftyLib's simulation ignores the Nova's onboard
 * PID gains, so running the loop on the SystemCore keeps simulation and the robot the same.
 *
 * Supported external encoders (WithExternalEncoder):
 * - `thrifty::Motor::FeedbackSensorType::ABS`: an absolute encoder on the Nova's data port. Its
 *   type comes from the vendor config, such as
 *   `thrifty::NovaConfig::AbsoluteEncoderType(thrifty::Motor::AbsoluteEncoderType::REV_ENCODER)`.
 * - `thrifty::Motor::FeedbackSensorType::QUAD`: a quadrature encoder on the Nova's data port. Its
 *   type comes from the vendor config, such as
 *   `thrifty::NovaConfig::QuadratureEncoderType(thrifty::Motor::QuadratureEncoderType::Custom(2048))`.
 * - `thrifty::CanEncoder*`: a Thrifty CAN Encoder on the same CAN bus.
 *
 * External encoder discontinuity point (absolute and CAN encoders): 0.5 rotations reports
 * [-0.5, 0.5), 1 rotation (the default) reports [0, 1).
 *
 * The vendor config, if any, must be a `thrifty::NovaConfigBatch`; it is applied before the YAMS
 * config, which overrides it.
 *
 * ### Example usage (inside a subsystem constructor)
 * @code{.cpp}
 * using namespace yams::motorcontrollers;
 * using namespace yams::motorcontrollers::local;
 *
 * // Declare as subsystem members:
 * //   thrifty::Nova                 m_nova{0, 3, thrifty::Motor::NEO};
 * //   SmartMotorControllerConfig    m_cfg;
 * //   std::optional<NovaWrapper>    m_smc;
 *
 * m_cfg.WithSubsystem(this)
 *     .WithFeedback(1.0, 0.0, 0.0)
 *     .WithStatorCurrentLimit(40.0_A)
 *     .WithMotorInverted(false)
 *     .WithVendorConfig(thrifty::NovaConfigBatch{thrifty::NovaConfig::TempThrottleEnable(true)})
 *     .WithClosedLoopMode()
 *     .WithTelemetry("ArmMotor", SmartMotorControllerConfig::TelemetryVerbosity::HIGH);
 *
 * m_smc.emplace(&m_nova, wpi::math::DCMotor::NEO(1), &m_cfg);
 * @endcode
 */
class NovaWrapper : public SmartMotorController {
 public:
  // Expose SetVelocity(velocity, feedforwardForce) from the base class alongside the overrides.
  using SmartMotorController::SetVelocity;

  /**
   * Construct a NovaWrapper around a Thrifty Nova.
   *
   * @param nova   Pointer to the Nova hardware object (must outlive this wrapper).
   * @param motor  DC motor model used for simulation.
   * @param config Pointer to the SmartMotorControllerConfig (must outlive this wrapper).
   */
  NovaWrapper(thrifty::Nova* nova, wpi::math::DCMotor motor, SmartMotorControllerConfig* config);
  ~NovaWrapper();

  // ---- Configuration ------------------------------------------------------
  /** @copydoc SmartMotorController::ApplyConfig */
  bool ApplyConfig(const SmartMotorControllerConfig& config) override;

  // ---- Simulation ---------------------------------------------------------
  /** @copydoc SmartMotorController::SetupSimulation */
  void SetupSimulation() override;
  /** @copydoc SmartMotorController::SimIterate */
  void SimIterate() override;

  // ---- Encoder sync -------------------------------------------------------
  /** @copydoc SmartMotorController::SeedRelativeEncoder */
  void SeedRelativeEncoder() override;
  /** @copydoc SmartMotorController::SynchronizeRelativeEncoder */
  void SynchronizeRelativeEncoder() override;

  // ---- Open-loop outputs --------------------------------------------------
  /** @copydoc SmartMotorController::SetDutyCycle */
  void SetDutyCycle(double dutyCycle) override;
  /** @copydoc SmartMotorController::GetDutyCycle */
  double GetDutyCycle() override;
  /** @copydoc SmartMotorController::SetVoltage */
  void SetVoltage(wpi::units::volt_t voltage) override;
  /** @copydoc SmartMotorController::GetVoltage */
  wpi::units::volt_t GetVoltage() override;

  // ---- Closed-loop setpoints ----------------------------------------------
  /** @copydoc SmartMotorController::SetPosition(wpi::units::turn_t) */
  void SetPosition(wpi::units::turn_t angle) override;
  /** @copydoc SmartMotorController::SetPosition(wpi::units::meter_t) */
  void SetPosition(wpi::units::meter_t distance) override;
  /** @copydoc SmartMotorController::SetVelocity(wpi::units::turns_per_second_t) */
  void SetVelocity(wpi::units::turns_per_second_t velocity) override;
  /**
   * Command a velocity setpoint with an additional feedforward force, added by the SystemCore
   * closed loop.
   *
   * @param velocity         Target mechanism angular velocity.
   * @param feedforwardForce Additional feedforward force at the mechanism.
   */
  void SetVelocity(wpi::units::turns_per_second_t velocity,
                   wpi::units::newton_t feedforwardForce) override;
  /** @copydoc SmartMotorController::SetVelocity(wpi::units::meters_per_second_t) */
  void SetVelocity(wpi::units::meters_per_second_t velocity) override;

  // ---- Encoder writes -----------------------------------------------------
  /** @copydoc SmartMotorController::SetEncoderPosition(wpi::units::turn_t) */
  void SetEncoderPosition(wpi::units::turn_t angle) override;
  /** @copydoc SmartMotorController::SetEncoderPosition(wpi::units::meter_t) */
  void SetEncoderPosition(wpi::units::meter_t distance) override;
  /**
   * Set the simulated mechanism velocity. Throws std::runtime_error outside of simulation, since
   * Thrifty Novas do not support setting the encoder velocity.
   *
   * @param velocity Mechanism velocity.
   */
  void SetEncoderVelocity(wpi::units::turns_per_second_t velocity) override;
  /**
   * Linear version of SetEncoderVelocity(turns_per_second_t); throws without a mechanism
   * circumference.
   *
   * @param velocity Mechanism linear velocity.
   */
  void SetEncoderVelocity(wpi::units::meters_per_second_t velocity) override;

  // ---- Encoder reads ------------------------------------------------------
  /** @copydoc SmartMotorController::GetMechanismPosition */
  wpi::units::turn_t GetMechanismPosition() override;
  /** @copydoc SmartMotorController::GetMechanismVelocity */
  wpi::units::turns_per_second_t GetMechanismVelocity() override;
  /** @copydoc SmartMotorController::GetRelativeMechanismPosition */
  wpi::units::turn_t GetRelativeMechanismPosition() override;
  /** @copydoc SmartMotorController::GetRelativeMechanismVelocity */
  wpi::units::turns_per_second_t GetRelativeMechanismVelocity() override;
  /** @copydoc SmartMotorController::GetMechanismAcceleration */
  wpi::units::turns_per_second_squared_t GetMechanismAcceleration() override;
  /** @copydoc SmartMotorController::GetRotorPosition */
  wpi::units::turn_t GetRotorPosition() override;
  /** @copydoc SmartMotorController::GetRotorVelocity */
  wpi::units::turns_per_second_t GetRotorVelocity() override;
  /** @copydoc SmartMotorController::GetMeasurementPosition */
  wpi::units::meter_t GetMeasurementPosition() override;
  /** @copydoc SmartMotorController::GetMeasurementVelocity */
  wpi::units::meters_per_second_t GetMeasurementVelocity() override;
  /** @copydoc SmartMotorController::GetMeasurementAcceleration */
  wpi::units::meters_per_second_squared_t GetMeasurementAcceleration() override;
  /** @copydoc SmartMotorController::GetExternalEncoderMechanismPosition */
  std::optional<wpi::units::turn_t> GetExternalEncoderMechanismPosition() override;
  /** @copydoc SmartMotorController::GetExternalEncoderMechanismVelocity */
  std::optional<wpi::units::turns_per_second_t> GetExternalEncoderMechanismVelocity() override;

  // ---- Motor status -------------------------------------------------------
  /** @copydoc SmartMotorController::GetSupplyCurrent */
  std::optional<wpi::units::ampere_t> GetSupplyCurrent() override;
  /** @copydoc SmartMotorController::GetStatorCurrent */
  wpi::units::ampere_t GetStatorCurrent() override;
  /** @copydoc SmartMotorController::GetTemperature */
  wpi::units::celsius_t GetTemperature() override;
  /** @copydoc SmartMotorController::GetDCMotor */
  wpi::math::DCMotor GetDCMotor() override;

  // ---- Configuration setters (live tuning) --------------------------------
  /** @copydoc SmartMotorController::SetZeroPower */
  void SetZeroPower(MotorMode mode) override;
  /** @copydoc SmartMotorController::SetMotorInverted */
  void SetMotorInverted(bool inverted) override;
  /** The Nova's internal encoder cannot be inverted; always throws std::runtime_error. */
  void SetEncoderInverted(bool inverted) override;
  /** @copydoc SmartMotorController::SetKp */
  void SetKp(double kP) override;
  /** @copydoc SmartMotorController::SetKi */
  void SetKi(double kI) override;
  /** @copydoc SmartMotorController::SetKd */
  void SetKd(double kD) override;
  /** @copydoc SmartMotorController::SetFeedback */
  void SetFeedback(double kP, double kI, double kD) override;
  /** @copydoc SmartMotorController::SetKs */
  void SetKs(double kS) override;
  /** @copydoc SmartMotorController::SetKv */
  void SetKv(double kV) override;
  /** @copydoc SmartMotorController::SetKa */
  void SetKa(double kA) override;
  /** @copydoc SmartMotorController::SetKg */
  void SetKg(double kG) override;
  /** @copydoc SmartMotorController::SetFeedforward */
  void SetFeedforward(double kS, double kV, double kA, double kG) override;
  /** @copydoc SmartMotorController::SetStatorCurrentLimit */
  void SetStatorCurrentLimit(wpi::units::ampere_t currentLimit) override;
  /** @copydoc SmartMotorController::SetSupplyCurrentLimit */
  void SetSupplyCurrentLimit(wpi::units::ampere_t currentLimit) override;
  /** The Nova has one ramp rate, for both open and closed loop control. */
  void SetClosedLoopRampRate(wpi::units::second_t rampRate) override;
  /** The Nova has one ramp rate, for both open and closed loop control. */
  void SetOpenLoopRampRate(wpi::units::second_t rampRate) override;
  /** @copydoc SmartMotorController::SetMechanismUpperLimit */
  void SetMechanismUpperLimit(wpi::units::turn_t upperLimit) override;
  /** @copydoc SmartMotorController::SetMechanismLowerLimit */
  void SetMechanismLowerLimit(wpi::units::turn_t lowerLimit) override;
  /** @copydoc SmartMotorController::SetMechanismLimits */
  void SetMechanismLimits(wpi::units::turn_t lower, wpi::units::turn_t upper) override;
  /** The closed loop controller on the SystemCore enforces the mechanism limits. */
  void SetMechanismLimitsEnabled(bool enabled) override;
  /** @copydoc SmartMotorController::SetMeasurementUpperLimit */
  void SetMeasurementUpperLimit(wpi::units::meter_t upperLimit) override;
  /** @copydoc SmartMotorController::SetMeasurementLowerLimit */
  void SetMeasurementLowerLimit(wpi::units::meter_t lowerLimit) override;
  /** @copydoc SmartMotorController::SetMotionProfileMaxVelocity(wpi::units::turns_per_second_t) */
  void SetMotionProfileMaxVelocity(wpi::units::turns_per_second_t maxVelocity) override;
  /** @copydoc SmartMotorController::SetMotionProfileMaxVelocity(wpi::units::meters_per_second_t) */
  void SetMotionProfileMaxVelocity(wpi::units::meters_per_second_t maxVelocity) override;
  /** @copydoc SmartMotorController::SetMotionProfileMaxAcceleration(wpi::units::turns_per_second_squared_t) */
  void SetMotionProfileMaxAcceleration(wpi::units::turns_per_second_squared_t maxAcc) override;
  /** @copydoc SmartMotorController::SetMotionProfileMaxAcceleration(wpi::units::meters_per_second_squared_t) */
  void SetMotionProfileMaxAcceleration(wpi::units::meters_per_second_squared_t maxAcc) override;
  /**
   * Set the maximum jerk of a velocity trapezoidal profile (its acceleration constraint). Has no
   * effect on position profiles.
   *
   * @param maxJerk Maximum angular jerk.
   */
  void SetMotionProfileMaxJerk(wpi::units::angular_jerk::turns_per_second_cubed_t maxJerk) override;
  /** Update the exponential profile. Missing values keep the configured ones. */
  void SetExponentialProfile(std::optional<double> kV, std::optional<double> kA,
                             std::optional<wpi::units::volt_t> maxInput) override;
  /** @copydoc SmartMotorController::SetClosedLoopSlot */
  void SetClosedLoopSlot(ClosedLoopControllerSlot slot) override;
  /**
   * Update the mechanism gearing and rebuild the simulated motor, which depends on it.
   *
   * @param gearing New mechanism gearing.
   */
  void SetMechanismGearing(const gearing::MechanismGearing& gearing) override;
  /** @copydoc SmartMotorController::SetMechanismCircumference */
  void SetMechanismCircumference(wpi::units::meter_t circumference) override;

  /** @copydoc SmartMotorController::GetConfig */
  SmartMotorControllerConfig& GetConfig() override;
  /**
   * Get a raw pointer to the underlying Nova.
   *
   * @return Pointer to the thrifty::Nova instance.
   */
  void* GetMotorController() override;
  /**
   * Get a raw pointer to the vendor config given to WithVendorConfig().
   *
   * ThriftyLib configs are write-only actions; read the Nova's configuration back from
   * `thrifty::Nova::Status()`.
   *
   * @return Pointer to the thrifty::NovaConfigBatch, or nullptr without one.
   */
  void* GetMotorControllerConfig() override;

 private:
  thrifty::Nova* m_nova{nullptr};
  /** Vendor config applied to the Nova before the YAMS config. */
  std::optional<thrifty::NovaConfigBatch> m_vendorConfig;
  wpi::math::DCMotor m_motor;
  std::optional<wpi::sim::DCMotorSim> m_motorSim;
  /** Data port encoder used as the external encoder, ABS or QUAD. */
  std::optional<thrifty::Motor::FeedbackSensorType> m_dataPortEncoder;
  /** CAN encoder used as the external encoder. */
  thrifty::CanEncoder* m_canEncoder{nullptr};
  /**
   * Discontinuity point of the external absolute encoder: it reports angles from one rotation
   * below it up to it.
   */
  wpi::units::turn_t m_absoluteEncoderDiscontinuityPoint{1.0};
  /**
   * Duty cycle last applied to the simulated motor. The simulation holds it between commands, as
   * the Nova holds its output, so it must be what was commanded and not read back from the motor
   * model.
   */
  double m_simDutyCycle{0.0};
  math::DerivativeTimeFilter m_accelFilter{20_ms};

  std::optional<wpi::util::Alert> m_systemCoreClosedLoopAlert;

  /** Unique alert id, like the Java buildAlertId. */
  std::string AlertId(const std::string& alertType) const;
  /** Rotor rotations per mechanism rotation. */
  double MechanismToRotorRatio() const;
  /** External encoder rotations per mechanism rotation. */
  double MechanismToExternalEncoderRatio() const;
  /** Load the software PID and LQR from the active gain slot. */
  void LoadClosedLoopController(const SmartMotorControllerConfig& config);
  /** Update the slot's PID gains in the config and software PID (no forwarding). */
  void ApplyFeedback(double kP, double kI, double kD);
  /** Create the simulated motor and its sim supplier from the config. */
  void CreateMotorSim();
  /** Configure the external encoder from the config, adding Nova settings to @p novaConfig. */
  void ApplyExternalEncoder(const SmartMotorControllerConfig& config,
                            thrifty::NovaConfigBatch& novaConfig);
  /** Angle of the external encoder's shaft, if any. */
  std::optional<wpi::units::turn_t> GetExternalEncoderSensorPosition();
  /** Velocity of the external encoder's shaft, if any. */
  std::optional<wpi::units::turns_per_second_t> GetExternalEncoderSensorVelocity();
};

}  // namespace yams::motorcontrollers::local
