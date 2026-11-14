// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <first/A301.h>

#include <optional>
#include <string>
#include <wpi/simulation/DCMotorSim.hpp>
#include <wpi/util/Alert.hpp>

#include "yams/math/DerivativeTimeFilter.hpp"
#include "yams/motorcontrollers/SmartMotorController.hpp"
#include "yams/motorcontrollers/YMotors.hpp"

namespace yams::motorcontrollers::local {

/**
 * SmartMotorController implementation for the FIRST A301 smart motor, using REVLib 2027.
 *
 * The A301 has no closed loop gains, so the closed loop controller runs on the SystemCore and
 * commands the A301 with voltage. It also has no current limits, ramp rates or voltage
 * compensation, and configuring them throws a SmartMotorControllerConfigurationException.
 *
 * The gearing given to WithMotorGearing is from the A301's rotor to the mechanism: usually the
 * gearbox attached to the A301, and any gearing after it. Each A301 has one motor,
 * YMotors::A301().
 *
 * Followers: the A301 has no follower mode, so the wrapper follows in software. Wrap each other
 * A301 on the mechanism in its own A301Wrapper and give them to the main A301's WithFollowers, as
 * (A301Wrapper*, inverted) pairs. Only an A301Wrapper can follow an A301. The main A301 sends each
 * follower its voltage or duty cycle, and its GetDCMotor() and simulation have every A301, so a
 * follower does not simulate itself or draw current from BatterySim: it reads the main A301's
 * simulation, and the main A301's SimIterate() iterates it. Create each follower before the main
 * A301, and destroy it after. WithLooselyCoupledFollowers still works as on any motor controller,
 * for motors with their own load.
 *
 * REVLib does not simulate the A301, so in simulation the wrapper drives a DCMotorSim.
 *
 * External encoder: the A301's own absolute encoder, on its output shaft, used by giving the
 * `first::a301::A301*` itself to WithExternalEncoder. Its gearing, given to
 * WithExternalEncoderGearing, is from the output shaft to the mechanism. Its zero offset, inversion
 * and wrapping range are applied by the wrapper; without WithWrapping it reads [0, 1) rotations of
 * the encoder.
 *
 * ### Example usage (inside a subsystem constructor)
 * @code{.cpp}
 * using namespace yams::motorcontrollers;
 * using namespace yams::motorcontrollers::local;
 *
 * // Declare as subsystem members:
 * //   first::a301::A301             m_a301{wpi::CANPort::CAN_S0, 3};
 * //   SmartMotorControllerConfig    m_cfg;
 * //   std::optional<A301Wrapper>    m_smc;
 *
 * // The A301 has its 64:1 gearbox.
 * m_cfg.WithSubsystem(this)
 *     .WithMotorGearing(yams::gearing::MechanismGearing{64.0})
 *     .WithFeedback(1.0, 0.0, 0.0)
 *     .WithMotorInverted(false)
 *     .WithExternalEncoder(&m_a301)
 *     .WithUseExternalFeedbackEncoder(true)
 *     .WithClosedLoopMode()
 *     .WithTelemetry("ArmMotor", SmartMotorControllerConfig::TelemetryVerbosity::HIGH);
 *
 * m_smc.emplace(&m_a301, &m_cfg);
 * @endcode
 */
class A301Wrapper : public SmartMotorController {
 public:
  // Expose SetVelocity(velocity, feedforwardForce) from the base class alongside the overrides.
  using SmartMotorController::SetVelocity;

  /**
   * Construct an A301Wrapper around a FIRST A301.
   *
   * @param a301   Pointer to the A301 hardware object (must outlive this wrapper).
   * @param config Pointer to the SmartMotorControllerConfig (must outlive this wrapper).
   */
  A301Wrapper(first::a301::A301* a301, SmartMotorControllerConfig* config);
  ~A301Wrapper();

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
   * the A301 does not support setting the encoder velocity.
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
  /**
   * The A301 does not report its absolute encoder's velocity, so this is the A301's relative
   * encoder's velocity at the mechanism.
   */
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
  /**
   * The A301 does not report its absolute encoder's velocity, so this is the A301's relative
   * encoder's velocity at the mechanism, or empty without the absolute encoder.
   */
  std::optional<wpi::units::turns_per_second_t> GetExternalEncoderMechanismVelocity() override;

  // ---- Motor status -------------------------------------------------------
  /** The simulated supply current, or empty on the robot, since the A301 does not report it. */
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
  /** The A301's internal encoder cannot be inverted; always throws std::runtime_error. */
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
  /** The A301 has no current limits; always throws std::runtime_error. */
  void SetStatorCurrentLimit(wpi::units::ampere_t currentLimit) override;
  /** The A301 has no current limits; always throws std::runtime_error. */
  void SetSupplyCurrentLimit(wpi::units::ampere_t currentLimit) override;
  /** The closed loop runs on the SystemCore, which ramps its output. */
  void SetClosedLoopRampRate(wpi::units::second_t rampRate) override;
  /** The A301 has no ramp rates; always throws std::runtime_error. */
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
  /** @copydoc
   * SmartMotorController::SetMotionProfileMaxAcceleration(wpi::units::turns_per_second_squared_t)
   */
  void SetMotionProfileMaxAcceleration(wpi::units::turns_per_second_squared_t maxAcc) override;
  /** @copydoc
   * SmartMotorController::SetMotionProfileMaxAcceleration(wpi::units::meters_per_second_squared_t)
   */
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
   * Get a raw pointer to the underlying A301.
   *
   * @return Pointer to the first::a301::A301 instance.
   */
  void* GetMotorController() override;
  /**
   * The A301 has no vendor config.
   *
   * @return nullptr.
   */
  void* GetMotorControllerConfig() override;
  /** Supply current, current limits and ramp rates, which the A301 does not have. */
  telemetry::UnsupportedTelemetryFields GetUnsupportedTelemetryFields() override;

 private:
  first::a301::A301* m_a301{nullptr};
  /** Motors of this A301 and the A301s following it. */
  wpi::math::DCMotor m_motor{YMotors::A301()};
  /** How many A301s m_motor has: this one and its followers. */
  int m_motorCount{1};
  /** A301s following this one, with whether each is inverted from it. */
  std::vector<std::pair<A301Wrapper*, bool>> m_a301Followers;
  /** A301 this one follows, or nullptr. */
  A301Wrapper* m_leader{nullptr};
  std::optional<wpi::sim::DCMotorSim> m_motorSim;
  /** Whether the A301's absolute encoder is the external encoder. */
  bool m_useAbsoluteEncoder{false};
  /**
   * Duty cycle last applied to the simulated motor. The simulation holds it between commands, as
   * the A301 holds its output, so it must be what was commanded and not read back from the motor
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
  /** Zero offset of the absolute encoder, in rotations of the encoder. */
  double ZeroOffsetRotations() const;
  /** Load the software PID and LQR from the active gain slot. */
  void LoadClosedLoopController(const SmartMotorControllerConfig& config);
  /** Update the slot's PID gains in the config and software PID (no forwarding). */
  void ApplyFeedback(double kP, double kI, double kD);
  /** Create the simulated motor and its sim supplier from the config. */
  void CreateMotorSim();
  /** Stop following another A301, and simulate this one again. */
  void StopFollowing();
  /** Angle of the absolute encoder's shaft, if it is the external encoder. */
  std::optional<wpi::units::turn_t> GetExternalEncoderSensorPosition();
};

}  // namespace yams::motorcontrollers::local
