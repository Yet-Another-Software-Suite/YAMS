// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/simulation/DCMotorSim.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/CANdi.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <ctre/phoenix6/controls/MotionMagicExpoVoltage.hpp>
#include <ctre/phoenix6/controls/MotionMagicVelocityVoltage.hpp>
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>
#include <memory>
#include <optional>

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
  void SetPosition(units::degree_t angle) override;
  void SetPosition(units::meter_t distance) override;
  void SetVelocity(units::degrees_per_second_t velocity) override;
  void SetVelocity(units::meters_per_second_t velocity) override;

  // ---- Encoder writes -----------------------------------------------------
  void SetEncoderPosition(units::degree_t angle) override;
  void SetEncoderPosition(units::meter_t distance) override;
  void SetEncoderVelocity(units::degrees_per_second_t velocity) override;
  void SetEncoderVelocity(units::meters_per_second_t velocity) override;

  // ---- Encoder reads ------------------------------------------------------
  units::degree_t GetMechanismPosition() override;
  units::degrees_per_second_t GetMechanismVelocity() override;
  units::degrees_per_second_squared_t GetMechanismAcceleration() override;
  units::degree_t GetRotorPosition() override;
  units::degrees_per_second_t GetRotorVelocity() override;
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
  void SetMechanismUpperLimit(units::degree_t upperLimit) override;
  void SetMechanismLowerLimit(units::degree_t lowerLimit) override;
  void SetMechanismLimits(units::degree_t lower, units::degree_t upper) override;
  void SetMechanismLimitsEnabled(bool enabled) override;
  void SetMeasurementUpperLimit(units::meter_t upperLimit) override;
  void SetMeasurementLowerLimit(units::meter_t lowerLimit) override;
  void SetMotionProfileMaxVelocity(units::degrees_per_second_t maxVelocity) override;
  void SetMotionProfileMaxVelocity(units::meters_per_second_t maxVelocity) override;
  void SetMotionProfileMaxAcceleration(units::degrees_per_second_squared_t maxAcc) override;
  void SetMotionProfileMaxAcceleration(units::meters_per_second_squared_t maxAcc) override;
  void SetMotionProfileMaxJerk(
      units::unit_t<units::compound_unit<units::angular_acceleration::degrees_per_second_squared,
                                         units::inverse<units::seconds>>>
          maxJerk) override;
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

  // Control requests
  ctre::phoenix6::controls::VelocityVoltage m_simpleVelocityReq{0_tps};
  ctre::phoenix6::controls::PositionVoltage m_simplePositionReq{0_tr};
  ctre::phoenix6::controls::MotionMagicVoltage m_trapPositionReq{0_tr};
  ctre::phoenix6::controls::MotionMagicVelocityVoltage m_trapVelocityReq{0_tps};
  ctre::phoenix6::controls::MotionMagicExpoVoltage m_expoPositionReq{0_tr};
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

  units::turns_per_second_t ToTPS(units::degrees_per_second_t v) const;
  units::turn_t ToTurns(units::degree_t a) const;
};

}  // namespace yams::motorcontrollers::remote
