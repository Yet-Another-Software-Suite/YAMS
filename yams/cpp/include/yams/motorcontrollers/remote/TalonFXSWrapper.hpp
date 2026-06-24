// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/Alert.h>
#include <frc/simulation/DCMotorSim.h>

#include <any>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/CANdi.hpp>
#include <ctre/phoenix6/TalonFXS.hpp>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
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
#include <optional>
#include <variant>

#include "yams/math/DerivativeTimeFilter.hpp"
#include "yams/motorcontrollers/SmartMotorController.hpp"

namespace yams::motorcontrollers::remote {

/**
 * SmartMotorController implementation for the CTRE TalonFXS motor controller (Phoenix 6).
 *
 * The TalonFXS supports third-party brushed and brushless motors (e.g. REV NEO, Minion).
 * Wraps a TalonFXS hardware object and exposes the full SmartMotorController interface
 * including MotionMagic profiles, CANcoder/CANdi synchronization, and simulation support.
 * ### Example usage (inside a subsystem constructor)
 * @code{.cpp}
 * using namespace yams::motorcontrollers;
 * using namespace yams::motorcontrollers::remote;
 * using namespace yams::gearing;
 * using Cfg = SmartMotorControllerConfig;
 *
 * // Declare as subsystem members:
 * //   ctre::phoenix6::hardware::TalonFXS m_talonFXS{2};
 * //   std::optional<TalonFXSWrapper>      m_smc;
 *
 * SmartMotorControllerConfig cfg;
 * cfg.WithSubsystem(this)
 *    .WithFeedback(4.0, 0.0, 0.0)
 *    .WithTrapezoidProfile(units::turns_per_second_t{0.5},
 *                          units::turns_per_second_squared_t{0.25})
 *    .WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({3.0, 4.0})})
 *    .WithIdleMode(Cfg::MotorMode::BRAKE)
 *    .WithStatorCurrentLimit(40.0_A)
 *    .WithMotorInverted(false)
 *    .WithArmFeedforward(0.0, 0.0, 0.0, 0.0)
 *    .WithClosedLoopMode()
 *    .WithTelemetry("HoodMotor", Cfg::TelemetryVerbosity::HIGH);
 *
 * m_smc.emplace(&m_talonFXS, DCMotor::NEO(1), MotorArrangement::NEO, &cfg);
 * @endcode
 */
class TalonFXSWrapper : public SmartMotorController {
 public:
  /** Motor type attached to the TalonFXS external motor port. */
  enum class MotorArrangement { Minion, NEO, NEO550, NEOVortex, Brushed_2Wire, Brushed_3Wire };

  /**
   * Construct a TalonFXSWrapper.
   *
   * @param talon       TalonFXS hardware object (must outlive this wrapper).
   * @param dcMotor     DC motor model used for simulation.
   * @param arrangement Motor type connected to the TalonFXS.
   * @param config      Initial SmartMotorControllerConfig to apply.
   */
  TalonFXSWrapper(ctre::phoenix6::hardware::TalonFXS* talon, frc::DCMotor dcMotor,
                  MotorArrangement arrangement, SmartMotorControllerConfig* config);

  ~TalonFXSWrapper();

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
  /** TalonFXS uses an absolute sensor internally; has no effect. */
  void SeedRelativeEncoder() override;
  /** CANcoder fusion is handled automatically by Phoenix 6; has no effect. */
  void SynchronizeRelativeEncoder() override;

  // ---- Open-loop outputs --------------------------------------------------
  /** @copydoc SmartMotorController::SetDutyCycle */
  void SetDutyCycle(double dutyCycle) override;
  /** @copydoc SmartMotorController::GetDutyCycle */
  double GetDutyCycle() override;
  /** @copydoc SmartMotorController::SetVoltage */
  void SetVoltage(units::volt_t voltage) override;
  /** @copydoc SmartMotorController::GetVoltage */
  units::volt_t GetVoltage() override;

  // ---- Closed-loop setpoints ----------------------------------------------
  /** @copydoc SmartMotorController::SetPosition(units::turn_t) */
  void SetPosition(units::turn_t angle) override;
  /**
   * Command a linear measurement position setpoint (closed-loop).
   * Converts distance to turns using the configured mechanism circumference.
   *
   * @param distance Target linear distance.
   */
  void SetPosition(units::meter_t distance) override;
  /** @copydoc SmartMotorController::SetVelocity(units::turns_per_second_t) */
  void SetVelocity(units::turns_per_second_t velocity) override;
  /**
   * Command a linear measurement velocity setpoint (closed-loop).
   * Converts linear velocity to turns per second using the configured mechanism circumference.
   *
   * @param velocity Target linear velocity.
   */
  void SetVelocity(units::meters_per_second_t velocity) override;

  // ---- Encoder writes -----------------------------------------------------
  /** @copydoc SmartMotorController::SetEncoderPosition(units::turn_t) */
  void SetEncoderPosition(units::turn_t angle) override;
  /**
   * Write a linear distance into the encoder (seeds the position).
   * Converts distance to turns using the configured mechanism circumference.
   *
   * @param distance Linear distance to write.
   */
  void SetEncoderPosition(units::meter_t distance) override;
  /** Not applicable to TalonFXS; has no effect. */
  void SetEncoderVelocity(units::turns_per_second_t velocity) override;
  /** Not applicable to TalonFXS; has no effect. */
  void SetEncoderVelocity(units::meters_per_second_t velocity) override;

  // ---- Encoder reads ------------------------------------------------------
  /** @copydoc SmartMotorController::GetMechanismPosition */
  units::turn_t GetMechanismPosition() override;
  /** @copydoc SmartMotorController::GetMechanismVelocity */
  units::turns_per_second_t GetMechanismVelocity() override;
  /** @copydoc SmartMotorController::GetMechanismAcceleration */
  units::turns_per_second_squared_t GetMechanismAcceleration() override;
  /** @copydoc SmartMotorController::GetRotorPosition */
  units::turn_t GetRotorPosition() override;
  /** @copydoc SmartMotorController::GetRotorVelocity */
  units::turns_per_second_t GetRotorVelocity() override;
  /** @copydoc SmartMotorController::GetMeasurementPosition */
  units::meter_t GetMeasurementPosition() override;
  /** @copydoc SmartMotorController::GetMeasurementVelocity */
  units::meters_per_second_t GetMeasurementVelocity() override;
  /** @copydoc SmartMotorController::GetMeasurementAcceleration */
  units::meters_per_second_squared_t GetMeasurementAcceleration() override;
  /** @copydoc SmartMotorController::GetExternalEncoderPosition */
  std::optional<units::degree_t> GetExternalEncoderPosition() override;
  /** @copydoc SmartMotorController::GetExternalEncoderVelocity */
  std::optional<units::degrees_per_second_t> GetExternalEncoderVelocity() override;

  // ---- Motor status -------------------------------------------------------
  /** @copydoc SmartMotorController::GetSupplyCurrent */
  std::optional<units::ampere_t> GetSupplyCurrent() override;
  /** @copydoc SmartMotorController::GetStatorCurrent */
  units::ampere_t GetStatorCurrent() override;
  /** @copydoc SmartMotorController::GetTemperature */
  units::celsius_t GetTemperature() override;
  /** @copydoc SmartMotorController::GetDCMotor */
  frc::DCMotor GetDCMotor() override;

  // ---- Configuration setters (live tuning) --------------------------------
  /** @copydoc SmartMotorController::SetIdleMode */
  void SetIdleMode(MotorMode mode) override;
  /** @copydoc SmartMotorController::SetMotorInverted */
  void SetMotorInverted(bool inverted) override;
  /** TalonFXS encoder direction follows motor output direction; has no effect. */
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
  void SetStatorCurrentLimit(units::ampere_t currentLimit) override;
  /** @copydoc SmartMotorController::SetSupplyCurrentLimit */
  void SetSupplyCurrentLimit(units::ampere_t currentLimit) override;
  /** @copydoc SmartMotorController::SetClosedLoopRampRate */
  void SetClosedLoopRampRate(units::second_t rampRate) override;
  /** @copydoc SmartMotorController::SetOpenLoopRampRate */
  void SetOpenLoopRampRate(units::second_t rampRate) override;
  /** @copydoc SmartMotorController::SetMechanismUpperLimit(units::turn_t) */
  void SetMechanismUpperLimit(units::turn_t upperLimit) override;
  /** @copydoc SmartMotorController::SetMechanismLowerLimit(units::turn_t) */
  void SetMechanismLowerLimit(units::turn_t lowerLimit) override;
  /** @copydoc SmartMotorController::SetMechanismLimits */
  void SetMechanismLimits(units::turn_t lower, units::turn_t upper) override;
  /** @copydoc SmartMotorController::SetMechanismLimitsEnabled */
  void SetMechanismLimitsEnabled(bool enabled) override;
  /**
   * Set the upper linear soft limit for the measurement.
   * Converts the upper limit to turns using the configured mechanism circumference.
   *
   * @param upperLimit Upper distance limit.
   */
  void SetMeasurementUpperLimit(units::meter_t upperLimit) override;
  /**
   * Set the lower linear soft limit for the measurement.
   * Converts the lower limit to turns using the configured mechanism circumference.
   *
   * @param lowerLimit Lower distance limit.
   */
  void SetMeasurementLowerLimit(units::meter_t lowerLimit) override;
  /** @copydoc SmartMotorController::SetMotionProfileMaxVelocity(units::turns_per_second_t) */
  void SetMotionProfileMaxVelocity(units::turns_per_second_t maxVelocity) override;
  /**
   * Set the maximum linear velocity for the motion profile.
   * Converts linear velocity to turns per second using the configured mechanism circumference.
   *
   * @param maxVelocity Maximum linear velocity.
   */
  void SetMotionProfileMaxVelocity(units::meters_per_second_t maxVelocity) override;
  /** @copydoc
   * SmartMotorController::SetMotionProfileMaxAcceleration(units::turns_per_second_squared_t) */
  void SetMotionProfileMaxAcceleration(units::turns_per_second_squared_t maxAcc) override;
  /**
   * Set the maximum linear acceleration for the motion profile.
   * Converts linear acceleration to turns per second squared using the configured mechanism
   * circumference.
   *
   * @param maxAcc Maximum linear acceleration.
   */
  void SetMotionProfileMaxAcceleration(units::meters_per_second_squared_t maxAcc) override;
  /** @copydoc SmartMotorController::SetMotionProfileMaxJerk */
  void SetMotionProfileMaxJerk(units::angular_jerk::turns_per_second_cubed_t maxJerk) override;
  /** @copydoc SmartMotorController::SetExponentialProfile */
  void SetExponentialProfile(std::optional<double> kV, std::optional<double> kA,
                             std::optional<units::volt_t> maxInput) override;
  /**
   * Select the active closed-loop gain slot.
   * TalonFXS supports 3 slots (SLOT_0 through SLOT_2); SLOT_3 is silently ignored.
   *
   * @param slot Gain slot to activate.
   */
  void SetClosedLoopSlot(ClosedLoopControllerSlot slot) override;

  /** @copydoc SmartMotorController::GetConfig */
  SmartMotorControllerConfig& GetConfig() override;
  /**
   * Get a raw pointer to the underlying TalonFXS hardware object.
   *
   * @return Pointer to the ctre::phoenix6::hardware::TalonFXS instance.
   */
  void* GetMotorController() override;
  /**
   * Get a raw pointer to the TalonFXSConfiguration used by this wrapper.
   *
   * @return Pointer to the ctre::phoenix6::configs::TalonFXSConfiguration instance.
   */
  void* GetMotorControllerConfig() override;

 private:
  SmartMotorControllerConfig* m_config{nullptr};
  ctre::phoenix6::hardware::TalonFXS* m_talon;
  frc::DCMotor m_dcMotor;
  MotorArrangement m_arrangement;
  ctre::phoenix6::configs::TalonFXSConfiguration m_talonConfig;

  // Active closed-loop control requests — variant selects the active request type
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

  std::optional<std::reference_wrapper<ctre::phoenix6::hardware::CANcoder>> m_cancoder;
  std::optional<std::reference_wrapper<ctre::phoenix6::hardware::CANdi>> m_candi;

  std::optional<frc::sim::DCMotorSim> m_motorSim;
  math::DerivativeTimeFilter m_accelFilter{20_ms};

  std::optional<frc::Alert> m_rioControllerAlert;

  ctre::phoenix6::signals::ExternalFeedbackSensorSourceValue ArrangementToFeedbackSource() const;

  void ApplyPIDConfig();
  void ApplyFeedforwardConfig();
  void ApplyLimitsConfig();
  void ApplyMotionMagicConfig();
};

}  // namespace yams::motorcontrollers::remote
