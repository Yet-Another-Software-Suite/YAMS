// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/simulation/DCMotorSim.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/CANdi.hpp>
#include <ctre/phoenix6/TalonFXS.hpp>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <ctre/phoenix6/controls/MotionMagicExpoVoltage.hpp>
#include <ctre/phoenix6/controls/MotionMagicVelocityVoltage.hpp>
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>
#include <optional>

#include "yams/math/DerivativeTimeFilter.hpp"
#include "yams/motorcontrollers/SmartMotorController.hpp"

namespace yams::motorcontrollers::remote {

/**
 * SmartMotorController implementation for the CTRE TalonFXS motor controller (Phoenix 6).
 *
 * The TalonFXS supports third-party brushed and brushless motors (e.g. REV NEO, Minion).
 * Wraps a TalonFXS hardware object and exposes the full SmartMotorController interface
 * including MotionMagic profiles, CANcoder/CANdi synchronization, and simulation support.
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
  TalonFXSWrapper(ctre::phoenix6::hardware::TalonFXS& talon, frc::DCMotor dcMotor,
                  MotorArrangement arrangement, const SmartMotorControllerConfig& config);

  telemetry::UnsupportedTelemetryFields GetUnsupportedTelemetryFields() override;

  bool ApplyConfig(const SmartMotorControllerConfig& config) override;
  void SetupSimulation() override;
  void SimIterate() override;
  void SeedRelativeEncoder() override;
  void SynchronizeRelativeEncoder() override;

  void SetDutyCycle(double dutyCycle) override;
  double GetDutyCycle() override;
  void SetVoltage(units::volt_t voltage) override;
  units::volt_t GetVoltage() override;

  void SetPosition(units::degree_t angle) override;
  void SetPosition(units::meter_t distance) override;
  void SetVelocity(units::degrees_per_second_t velocity) override;
  void SetVelocity(units::meters_per_second_t velocity) override;

  void SetEncoderPosition(units::degree_t angle) override;
  void SetEncoderPosition(units::meter_t distance) override;
  void SetEncoderVelocity(units::degrees_per_second_t velocity) override;
  void SetEncoderVelocity(units::meters_per_second_t velocity) override;

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

  std::optional<units::ampere_t> GetSupplyCurrent() override;
  units::ampere_t GetStatorCurrent() override;
  units::celsius_t GetTemperature() override;
  frc::DCMotor GetDCMotor() override;

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
  ctre::phoenix6::hardware::TalonFXS& m_talon;
  frc::DCMotor m_dcMotor;
  MotorArrangement m_arrangement;
  ctre::phoenix6::configs::TalonFXSConfiguration m_talonConfig;

  ctre::phoenix6::controls::VelocityVoltage m_simpleVelocityReq{0_tps};
  ctre::phoenix6::controls::PositionVoltage m_simplePositionReq{0_tr};
  ctre::phoenix6::controls::MotionMagicVoltage m_trapPositionReq{0_tr};
  ctre::phoenix6::controls::MotionMagicVelocityVoltage m_trapVelocityReq{0_tps};
  ctre::phoenix6::controls::MotionMagicExpoVoltage m_expoPositionReq{0_tr};
  ctre::phoenix6::controls::VoltageOut m_voltageReq{0_V};
  ctre::phoenix6::controls::DutyCycleOut m_dutyCycleReq{0.0};

  std::optional<std::reference_wrapper<ctre::phoenix6::hardware::CANcoder>> m_cancoder;
  std::optional<std::reference_wrapper<ctre::phoenix6::hardware::CANdi>> m_candi;

  std::optional<frc::sim::DCMotorSim> m_motorSim;
  math::DerivativeTimeFilter m_accelFilter{20_ms};

  ctre::phoenix6::signals::ExternalFeedbackSensorSourceValue ArrangementToFeedbackSource() const;
};

}  // namespace yams::motorcontrollers::remote
