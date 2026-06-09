// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/simulation/DCMotorSim.h>

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
#include <any>
#include <memory>
#include <optional>
#include <variant>

#include "yams/math/DerivativeTimeFilter.hpp"
#include "yams/motorcontrollers/SmartMotorController.hpp"

namespace yams::motorcontrollers::remote {

/**
 * SmartMotorController implementation for the CTRE TalonFX motor controller (Phoenix 6).
 *
 * Wraps a TalonFX hardware object and exposes the full SmartMotorController interface
 * including MotionMagic profiles, CANcoder/CANdi synchronization, and simulation support.
 */
class TalonFXWrapper : public SmartMotorController {
 public:
  /**
   * Construct a TalonFXWrapper.
   *
   * @param talon   TalonFX hardware object (must outlive this wrapper).
   * @param dcMotor DC motor model used for simulation.
   * @param config  Initial SmartMotorControllerConfig to apply.
   */
  TalonFXWrapper(ctre::phoenix6::hardware::TalonFX& talon, frc::DCMotor dcMotor,
                 const SmartMotorControllerConfig& config);

  // ---- Telemetry ----------------------------------------------------------
  telemetry::UnsupportedTelemetryFields GetUnsupportedTelemetryFields() override;

  // ---- Configuration ------------------------------------------------------
  bool ApplyConfig(const SmartMotorControllerConfig& config) override;

  // ---- Simulation ---------------------------------------------------------
  void SetupSimulation() override;
  void SimIterate() override;

  // ---- Encoder sync -------------------------------------------------------
  void SeedRelativeEncoder() override;
  void SynchronizeRelativeEncoder() override;

  // ---- Open-loop outputs --------------------------------------------------
  void SetDutyCycle(double dutyCycle) override;
  double GetDutyCycle() override;
  void SetVoltage(units::volt_t voltage) override;
  units::volt_t GetVoltage() override;

  // ---- Closed-loop setpoints ----------------------------------------------
  void SetPosition(units::turn_t angle) override;
  void SetPosition(units::meter_t distance) override;
  void SetVelocity(units::turns_per_second_t velocity) override;
  void SetVelocity(units::meters_per_second_t velocity) override;

  // ---- Encoder writes -----------------------------------------------------
  void SetEncoderPosition(units::turn_t angle) override;
  void SetEncoderPosition(units::meter_t distance) override;
  void SetEncoderVelocity(units::turns_per_second_t velocity) override;
  void SetEncoderVelocity(units::meters_per_second_t velocity) override;

  // ---- Encoder reads ------------------------------------------------------
  units::turn_t GetMechanismPosition() override;
  units::turns_per_second_t GetMechanismVelocity() override;
  units::turns_per_second_squared_t GetMechanismAcceleration() override;
  units::turn_t GetRotorPosition() override;
  units::turns_per_second_t GetRotorVelocity() override;
  units::meter_t GetMeasurementPosition() override;
  units::meters_per_second_t GetMeasurementVelocity() override;
  units::meters_per_second_squared_t GetMeasurementAcceleration() override;

  std::optional<units::degree_t> GetExternalEncoderPosition() override;
  std::optional<units::degrees_per_second_t> GetExternalEncoderVelocity() override;

  // ---- Motor status -------------------------------------------------------
  std::optional<units::ampere_t> GetSupplyCurrent() override;
  units::ampere_t GetStatorCurrent() override;
  units::celsius_t GetTemperature() override;
  frc::DCMotor GetDCMotor() override;

  // ---- Configuration setters (live tuning) --------------------------------
  void SetIdleMode(MotorMode mode) override;
  void SetMotorInverted(bool inverted) override;
  void SetEncoderInverted(bool inverted) override;
  void SetKp(double kP) override;
  void SetKi(double kI) override;
  void SetKd(double kD) override;
  void SetFeedback(double kP, double kI, double kD) override;
  void SetKs(double kS) override;
  void SetKv(double kV) override;
  void SetKa(double kA) override;
  void SetKg(double kG) override;
  void SetFeedforward(double kS, double kV, double kA, double kG) override;
  void SetStatorCurrentLimit(units::ampere_t currentLimit) override;
  void SetSupplyCurrentLimit(units::ampere_t currentLimit) override;
  void SetClosedLoopRampRate(units::second_t rampRate) override;
  void SetOpenLoopRampRate(units::second_t rampRate) override;
  void SetMechanismUpperLimit(units::turn_t upperLimit) override;
  void SetMechanismLowerLimit(units::turn_t lowerLimit) override;
  void SetMechanismLimits(units::turn_t lower, units::turn_t upper) override;
  void SetMechanismLimitsEnabled(bool enabled) override;
  void SetMeasurementUpperLimit(units::meter_t upperLimit) override;
  void SetMeasurementLowerLimit(units::meter_t lowerLimit) override;
  void SetMotionProfileMaxVelocity(units::turns_per_second_t maxVelocity) override;
  void SetMotionProfileMaxVelocity(units::meters_per_second_t maxVelocity) override;
  void SetMotionProfileMaxAcceleration(units::turns_per_second_squared_t maxAcc) override;
  void SetMotionProfileMaxAcceleration(units::meters_per_second_squared_t maxAcc) override;
  void SetMotionProfileMaxJerk(units::angular_jerk::turns_per_second_cubed_t maxJerk) override;
  void SetExponentialProfile(std::optional<double> kV, std::optional<double> kA,
                             std::optional<units::volt_t> maxInput) override;
  void SetClosedLoopSlot(ClosedLoopControllerSlot slot) override;

  SmartMotorControllerConfig& GetConfig() override;
  void* GetMotorController() override;
  void* GetMotorControllerConfig() override;

  // ---- CANcoder / CANdi support -------------------------------------------

  /**
   * Attach a CANcoder for absolute position feedback and encoder synchronization.
   *
   * @param cancoder CANcoder hardware object (must outlive this wrapper).
   */
  void WithCANcoder(ctre::phoenix6::hardware::CANcoder& cancoder);

  /**
   * Attach a CANdi for absolute position feedback and encoder synchronization.
   *
   * @param candi CANdi hardware object (must outlive this wrapper).
   */
  void WithCANdi(ctre::phoenix6::hardware::CANdi& candi);

 private:
  ctre::phoenix6::hardware::TalonFX& m_talon;
  frc::DCMotor m_dcMotor;
  ctre::phoenix6::configs::TalonFXConfiguration m_talonConfig;

  // Active closed-loop control requests — variant selects the active request type
  using PositionControlRequest = std::variant<
      ctre::phoenix6::controls::PositionVoltage,
      ctre::phoenix6::controls::PositionDutyCycle,
      ctre::phoenix6::controls::PositionTorqueCurrentFOC,
      ctre::phoenix6::controls::MotionMagicVoltage,
      ctre::phoenix6::controls::MotionMagicDutyCycle,
      ctre::phoenix6::controls::MotionMagicExpoVoltage,
      ctre::phoenix6::controls::MotionMagicExpoDutyCycle,
      ctre::phoenix6::controls::MotionMagicTorqueCurrentFOC>;
  using VelocityControlRequest = std::variant<
      ctre::phoenix6::controls::VelocityVoltage,
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
  std::optional<std::reference_wrapper<ctre::phoenix6::hardware::CANcoder>> m_cancoder;
  std::optional<std::reference_wrapper<ctre::phoenix6::hardware::CANdi>> m_candi;

  // Simulation
  std::optional<frc::sim::DCMotorSim> m_motorSim;

  math::DerivativeTimeFilter m_accelFilter{20_ms};

  void ApplyPIDConfig();
  void ApplyFeedforwardConfig();
  void ApplyLimitsConfig();
  void ApplyMotionMagicConfig();

};

}  // namespace yams::motorcontrollers::remote
