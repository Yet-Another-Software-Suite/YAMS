// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <any>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/CANdi.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <ctre/phoenix6/controls/MotionMagicDutyCycle.hpp>
#include <ctre/phoenix6/controls/MotionMagicExpoDutyCycle.hpp>
#include <ctre/phoenix6/controls/MotionMagicExpoVoltage.hpp>
#include <ctre/phoenix6/controls/MotionMagicTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/MotionMagicVelocityDutyCycle.hpp>
#include <ctre/phoenix6/controls/MotionMagicVelocityTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/MotionMagicVelocityVoltage.hpp>
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>
#include <ctre/phoenix6/controls/PositionDutyCycle.hpp>
#include <ctre/phoenix6/controls/PositionTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <ctre/phoenix6/controls/VelocityDutyCycle.hpp>
#include <ctre/phoenix6/controls/VelocityTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <wpi/simulation/DCMotorSim.hpp>
#include <wpi/units/frequency.hpp>
#include <wpi/util/Alert.hpp>

#include "yams/motorcontrollers/SmartMotorController.hpp"

namespace yams::motorcontrollers::remote {

/**
 * SmartMotorController implementation for the CTRE TalonFX motor controller (Phoenix 6).
 *
 * Wraps a TalonFX hardware object and exposes the full SmartMotorController interface
 * including MotionMagic profiles, CANcoder/CANdi synchronization, and simulation support.
 *
 * ### Example usage (inside a subsystem constructor)
 * @code{.cpp}
 * using namespace yams::motorcontrollers;
 * using namespace yams::motorcontrollers::remote;
 * using namespace yams::gearing;
 * using Cfg = SmartMotorControllerConfig;
 *
 * // Declare as subsystem members:
 * //   ctre::phoenix6::hardware::TalonFX m_talon{1};
 * //   std::optional<TalonFXWrapper>      m_smc;
 *
 * SmartMotorControllerConfig cfg;
 * cfg.WithSubsystem(this)
 *    .WithFeedback(4.0, 0.0, 0.0)
 *    .WithTrapezoidProfile(wpi::units::turns_per_second_t{0.5},
 *                          wpi::units::turns_per_second_squared_t{0.25})
 *    .WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({3.0, 4.0})})
 *    .WithZeroPower(Cfg::MotorMode::BRAKE)
 *    .WithStatorCurrentLimit(40.0_A)
 *    .WithMotorInverted(false)
 *    .WithFeedforward(wpi::math::ArmFeedforward{
 *        0.0_V, 0.0_V, wpi::units::unit_t<wpi::math::ArmFeedforward::kv_unit>{0.0},
 *        wpi::units::unit_t<wpi::math::ArmFeedforward::ka_unit>{0.0}})
 *    .WithClosedLoopMode()
 *    .WithTelemetry("ArmMotor", Cfg::TelemetryVerbosity::HIGH);
 *
 * m_smc.emplace(m_talon, wpi::math::DCMotor::KrakenX60(1), &cfg);
 * @endcode
 */
class TalonFXWrapper : public SmartMotorController {
 public:
  // Expose SetVelocity(velocity, feedforwardForce) from the base class alongside the overrides.
  using SmartMotorController::SetVelocity;

  /**
   * Construct a TalonFXWrapper.
   *
   * Throws SmartMotorControllerConfigurationException (or std::invalid_argument) if ApplyConfig()
   * rejects the config; anything already started is released first.
   *
   * @param talon   TalonFX hardware object (must outlive this wrapper).
   * @param dcMotor DC motor model used for simulation.
   * @param config  Initial SmartMotorControllerConfig to apply.
   */
  TalonFXWrapper(ctre::phoenix6::hardware::TalonFX* talon, wpi::math::DCMotor dcMotor,
                 SmartMotorControllerConfig* config);

  /** Calls Close() and releases the alerts. */
  ~TalonFXWrapper();

  // ---- TalonFX specific ---------------------------------------------------
  /**
   * Enable FOC for the position and velocity control requests; ignored by unlicensed devices.
   *
   * Throws SmartMotorControllerConfigurationException if a request is a TorqueCurrentFOC request
   * (set through WithVendorControlRequest), which cannot toggle FOC.
   *
   * @return *this for chaining.
   */
  TalonFXWrapper& EnableFOC();
  /**
   * Disable FOC for the position and velocity control requests.
   *
   * Throws SmartMotorControllerConfigurationException if a request is a TorqueCurrentFOC request.
   *
   * @return *this for chaining.
   */
  TalonFXWrapper& DisableFOC();
  /**
   * Whether CANdi PWM1 is the feedback sensor of the configuration being applied.
   *
   * Throws std::invalid_argument if it is but no CANdi is configured as the external encoder.
   *
   * @return true if the feedback sensor source is a CANdi PWM1 source.
   */
  bool UseCANdiPWM1() const;
  /**
   * Whether CANdi PWM2 is the feedback sensor of the configuration being applied.
   *
   * Throws std::invalid_argument if it is but no CANdi is configured as the external encoder.
   *
   * @return true if the feedback sensor source is a CANdi PWM2 source.
   */
  bool UseCANdiPWM2() const;
  /**
   * Set the update frequency of the status signals this controller reads.
   *
   * @param frequency Update frequency.
   */
  void SetUpdateFrequency(wpi::units::hertz_t frequency);
  /**
   * Apply the whole TalonFX configuration, retrying every 10 ms up to 10 times.
   *
   * @return Status of the last attempt.
   */
  ctre::phoenix::StatusCode ForceConfigApply();

  // ---- Telemetry ----------------------------------------------------------
  /** @copydoc SmartMotorController::GetUnsupportedTelemetryFields */
  telemetry::UnsupportedTelemetryFields GetUnsupportedTelemetryFields() override;

  // ---- Configuration ------------------------------------------------------
  /** @copydoc SmartMotorController::ApplyConfig */
  bool ApplyConfig(const SmartMotorControllerConfig& config) override;

  // ---- Simulation ---------------------------------------------------------
  /** @copydoc SmartMotorController::SetupSimulation */
  void SetupSimulation() override;
  /** @copydoc SmartMotorController::SimIterate */
  void SimIterate() override;

  // ---- Encoder sync -------------------------------------------------------
  /** TalonFX uses an absolute sensor internally; has no effect. */
  void SeedRelativeEncoder() override;
  /** The TalonFX fuses its feedback sources on the device; has no effect. */
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
  /**
   * Command a linear measurement position setpoint (closed-loop).
   * Converts distance to turns using the configured mechanism circumference.
   *
   * @param distance Target linear distance.
   */
  void SetPosition(wpi::units::meter_t distance) override;
  /** @copydoc SmartMotorController::SetVelocity(wpi::units::turns_per_second_t) */
  void SetVelocity(wpi::units::turns_per_second_t velocity) override;
  /**
   * Command a linear measurement velocity setpoint (closed-loop).
   * Converts linear velocity to turns per second using the configured mechanism circumference.
   *
   * @param velocity Target linear velocity.
   */
  void SetVelocity(wpi::units::meters_per_second_t velocity) override;
  /**
   * Command a velocity setpoint with an additional feedforward force, applied by the TalonFX as
   * an arbitrary feedforward (duty cycle, volts or TorqueCurrentFOC amps depending on the
   * request).  With an LQR the RoboRIO loop adds it instead.
   *
   * Throws SmartMotorControllerConfigurationException if the mechanism circumference is not set.
   *
   * @param velocity         Target mechanism angular velocity.
   * @param feedforwardForce Additional feedforward force at the mechanism.
   */
  void SetVelocity(wpi::units::turns_per_second_t velocity,
                   wpi::units::newton_t feedforwardForce) override;

  // ---- Encoder writes -----------------------------------------------------
  /** @copydoc SmartMotorController::SetEncoderPosition(wpi::units::turn_t) */
  void SetEncoderPosition(wpi::units::turn_t angle) override;
  /**
   * Write a linear distance into the encoder (seeds the position).
   * Converts distance to turns using the configured mechanism circumference.
   *
   * @param distance Linear distance to write.
   */
  void SetEncoderPosition(wpi::units::meter_t distance) override;
  /** Not applicable to TalonFX; has no effect. */
  void SetEncoderVelocity(wpi::units::turns_per_second_t velocity) override;
  /** Not applicable to TalonFX; has no effect. */
  void SetEncoderVelocity(wpi::units::meters_per_second_t velocity) override;

  // ---- Encoder reads ------------------------------------------------------
  /** @copydoc SmartMotorController::GetMechanismPosition */
  wpi::units::turn_t GetMechanismPosition() override;
  /** @copydoc SmartMotorController::GetMechanismVelocity */
  wpi::units::turns_per_second_t GetMechanismVelocity() override;
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
  /** @copydoc SmartMotorController::GetExternalEncoderPosition */
  std::optional<wpi::units::degree_t> GetExternalEncoderPosition() override;
  /** @copydoc SmartMotorController::GetExternalEncoderVelocity */
  std::optional<wpi::units::degrees_per_second_t> GetExternalEncoderVelocity() override;

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
  /** TalonFX encoder direction follows motor output direction; has no effect. */
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
  /** @copydoc SmartMotorController::SetClosedLoopRampRate */
  void SetClosedLoopRampRate(wpi::units::second_t rampRate) override;
  /** @copydoc SmartMotorController::SetOpenLoopRampRate */
  void SetOpenLoopRampRate(wpi::units::second_t rampRate) override;
  /** @copydoc SmartMotorController::SetMechanismUpperLimit(wpi::units::turn_t) */
  void SetMechanismUpperLimit(wpi::units::turn_t upperLimit) override;
  /** @copydoc SmartMotorController::SetMechanismLowerLimit(wpi::units::turn_t) */
  void SetMechanismLowerLimit(wpi::units::turn_t lowerLimit) override;
  /** @copydoc SmartMotorController::SetMechanismLimits */
  void SetMechanismLimits(wpi::units::turn_t lower, wpi::units::turn_t upper) override;
  /** @copydoc SmartMotorController::SetMechanismLimitsEnabled */
  void SetMechanismLimitsEnabled(bool enabled) override;
  /**
   * Set the upper linear soft limit for the measurement.
   * Converts the upper limit to turns using the configured mechanism circumference.
   *
   * @param upperLimit Upper distance limit.
   */
  void SetMeasurementUpperLimit(wpi::units::meter_t upperLimit) override;
  /**
   * Set the lower linear soft limit for the measurement.
   * Converts the lower limit to turns using the configured mechanism circumference.
   *
   * @param lowerLimit Lower distance limit.
   */
  void SetMeasurementLowerLimit(wpi::units::meter_t lowerLimit) override;
  /** @copydoc SmartMotorController::SetMotionProfileMaxVelocity(wpi::units::turns_per_second_t) */
  void SetMotionProfileMaxVelocity(wpi::units::turns_per_second_t maxVelocity) override;
  /**
   * Set the maximum linear velocity for the motion profile.
   * Converts linear velocity to turns per second using the configured mechanism circumference.
   *
   * @param maxVelocity Maximum linear velocity.
   */
  void SetMotionProfileMaxVelocity(wpi::units::meters_per_second_t maxVelocity) override;
  /** @copydoc
   * SmartMotorController::SetMotionProfileMaxAcceleration(wpi::units::turns_per_second_squared_t)
   */
  void SetMotionProfileMaxAcceleration(wpi::units::turns_per_second_squared_t maxAcc) override;
  /**
   * Set the maximum linear acceleration for the motion profile.
   * Converts linear acceleration to turns per second squared using the configured mechanism
   * circumference.
   *
   * @param maxAcc Maximum linear acceleration.
   */
  void SetMotionProfileMaxAcceleration(wpi::units::meters_per_second_squared_t maxAcc) override;
  /** @copydoc SmartMotorController::SetMotionProfileMaxJerk */
  void SetMotionProfileMaxJerk(wpi::units::angular_jerk::turns_per_second_cubed_t maxJerk) override;
  /** @copydoc SmartMotorController::SetExponentialProfile */
  void SetExponentialProfile(std::optional<double> kV, std::optional<double> kA,
                             std::optional<wpi::units::volt_t> maxInput) override;
  /**
   * Select the active closed-loop gain slot and forward it to loosely coupled followers.
   * TalonFX supports 3 slots (SLOT_0 through SLOT_2); throws std::invalid_argument for SLOT_3.
   *
   * @param slot Gain slot to activate.
   */
  void SetClosedLoopSlot(ClosedLoopControllerSlot slot) override;
  /**
   * Update the mechanism gearing and the TalonFX sensor ratios that depend on it.
   *
   * @param gearing New mechanism gearing.
   */
  void SetMechanismGearing(const gearing::MechanismGearing& gearing) override;
  /**
   * Update the mechanism circumference, rewrite the values converted with it (linear gains and
   * motion profile constraints) and forward it to loosely coupled followers.
   *
   * @param circumference New mechanism circumference.
   */
  void SetMechanismCircumference(wpi::units::meter_t circumference) override;

  /** @copydoc SmartMotorController::GetConfig */
  SmartMotorControllerConfig& GetConfig() override;
  /**
   * Get a raw pointer to the underlying TalonFX hardware object.
   *
   * @return Pointer to the ctre::phoenix6::hardware::TalonFX instance.
   */
  void* GetMotorController() override;
  /**
   * Get a raw pointer to the TalonFXConfiguration used by this wrapper.
   *
   * @return Pointer to the ctre::phoenix6::configs::TalonFXConfiguration instance.
   */
  void* GetMotorControllerConfig() override;

 private:
  ctre::phoenix6::hardware::TalonFX* m_talon;
  wpi::math::DCMotor m_dcMotor;
  // Whether StatusSignal refreshes should report errors; false in simulation, where status
  // signals are not always updated before they are read.
  const bool m_reportStatusSignalErrors;
  ctre::phoenix6::configs::TalonFXConfiguration m_talonConfig;

  // Active closed-loop control requests variant selects the active request type
  using PositionControlRequest = std::variant<
      ctre::phoenix6::controls::PositionVoltage, ctre::phoenix6::controls::PositionDutyCycle,
      ctre::phoenix6::controls::PositionTorqueCurrentFOC,
      ctre::phoenix6::controls::MotionMagicVoltage, ctre::phoenix6::controls::MotionMagicDutyCycle,
      ctre::phoenix6::controls::MotionMagicExpoVoltage,
      ctre::phoenix6::controls::MotionMagicExpoDutyCycle,
      ctre::phoenix6::controls::MotionMagicTorqueCurrentFOC>;
  using VelocityControlRequest =
      std::variant<ctre::phoenix6::controls::VelocityVoltage,
                   ctre::phoenix6::controls::VelocityDutyCycle,
                   ctre::phoenix6::controls::VelocityTorqueCurrentFOC,
                   ctre::phoenix6::controls::MotionMagicVelocityVoltage,
                   ctre::phoenix6::controls::MotionMagicVelocityDutyCycle,
                   ctre::phoenix6::controls::MotionMagicVelocityTorqueCurrentFOC>;

  PositionControlRequest m_positionReq{ctre::phoenix6::controls::PositionVoltage{0_tr}};
  VelocityControlRequest m_velocityReq{ctre::phoenix6::controls::VelocityVoltage{0_tps}};
  ctre::phoenix6::controls::VoltageOut m_voltageReq{0_V};
  ctre::phoenix6::controls::DutyCycleOut m_dutyCycleReq{0.0};

  // External sensors
  ctre::phoenix6::hardware::CANcoder* m_cancoder{nullptr};
  ctre::phoenix6::hardware::CANdi* m_candi{nullptr};

  // Simulation
  std::optional<wpi::sim::DCMotorSim> m_motorSim;

  /** Shown while the closed loop controller runs on the RoboRIO. */
  std::optional<wpi::util::Alert> m_rioControllerAlert;
  /** Shown when the starting position is not applied because an external encoder is used. */
  std::optional<wpi::util::Alert> m_startingPositionExternalEncoderAlert;
  /** Shown when a zero offset is set without an external encoder. */
  std::optional<wpi::util::Alert> m_zeroOffsetNoExternalEncoderAlert;
  /** Shown when a discontinuity point is set without an external encoder. */
  std::optional<wpi::util::Alert> m_discontinuityPointNoExternalEncoderAlert;

  /** Unique alert id, like the Java buildAlertId. */
  std::string AlertId(const std::string& alertType) const;
  /** Send a control request, retrying up to 8 times until it is accepted. */
  void EnsureRequest(const std::function<ctre::phoenix::StatusCode()>& request);
  /** Apply one configuration group, retrying every 10 ms up to 10 times. */
  template <typename Group>
  ctre::phoenix::StatusCode ApplyGroup(const Group& group);
  /** Set FOC on the position and velocity requests. */
  void SetFOC(bool foc);
  /** Mechanism rotations per linear gain unit (1 when the closed loop is not linear). */
  double PositionGainUnitsPerRotation() const;
  /** Mechanism rotations/s per linear velocity gain unit (1 when not linear). */
  double VelocityGainUnitsPerRotation() const;
  /** Mechanism rotations/s² per linear acceleration gain unit (1 when not linear). */
  double AccelerationGainUnitsPerRotation() const;
  /** Write PID gains (YAMS units) to a TalonFX slot; throws for SLOT_3. */
  void WriteSlotPID(ClosedLoopControllerSlot slot, double kP, double kI, double kD);
  /** Write the gains of every slot that has PID or feedforward configured. */
  void WriteSlotGains(const SmartMotorControllerConfig& config);
  /** Write the Motion Magic constraints from the config's profiles. */
  void WriteMotionMagic(const SmartMotorControllerConfig& config);
  /** Write the sensor ratios that depend on the mechanism and external encoder gearing. */
  void WriteSensorRatios(const SmartMotorControllerConfig& config);
  /** Configure the external encoder, or the rotor sensor when none is used. */
  void ApplyExternalEncoder(const SmartMotorControllerConfig& config);
  /** Configure the tightly coupled followers. */
  void ApplyFollowers(const SmartMotorControllerConfig& config);
  /** Use the vendor control request from the config, if any. */
  void ApplyVendorControlRequest(const SmartMotorControllerConfig& config);
};

}  // namespace yams::motorcontrollers::remote
