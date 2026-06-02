// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/trajectory/ExponentialProfile.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/SubsystemBase.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/temperature.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <optional>
#include <string>
#include <vector>

#include "yams/gearing/MechanismGearing.h"
#include "yams/math/LQRConfig.h"

namespace yams::motorcontrollers {

class SmartMotorControllerConfig {
 public:
  enum class ControlMode { CLOSED_LOOP, OPEN_LOOP };
  enum class MotorMode { COAST, BRAKE };
  enum class TelemetryVerbosity { NONE, LOW, MEDIUM, HIGH };
  enum class ClosedLoopControllerSlot { SLOT_0, SLOT_1, SLOT_2, SLOT_3 };

  // ---- Feedback -------------------------------------------------------
  SmartMotorControllerConfig& WithFeedback(
      double kP, double kI, double kD,
      ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);
  SmartMotorControllerConfig& WithKp(
      double kP, ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);
  SmartMotorControllerConfig& WithKi(
      double kI, ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);
  SmartMotorControllerConfig& WithKd(
      double kD, ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);

  // ---- Feedforward -------------------------------------------------------
  SmartMotorControllerConfig& WithArmFeedforward(
      double kS, double kV, double kA, double kG,
      ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);
  SmartMotorControllerConfig& WithElevatorFeedforward(
      double kS, double kV, double kA,
      ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);
  SmartMotorControllerConfig& WithSimpleFeedforward(
      double kS, double kV, double kA = 0.0,
      ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);
  SmartMotorControllerConfig& WithKs(
      double kS, ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);
  SmartMotorControllerConfig& WithKv(
      double kV, ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);
  SmartMotorControllerConfig& WithKa(
      double kA, ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);
  SmartMotorControllerConfig& WithKg(
      double kG, ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);

  // ---- Motion profiles ---------------------------------------------------
  SmartMotorControllerConfig& WithTrapezoidProfile(
      units::turns_per_second_t maxVelocity, units::turns_per_second_squared_t maxAcceleration);
  SmartMotorControllerConfig& WithLinearTrapezoidProfile(
      units::meters_per_second_t maxVelocity, units::meters_per_second_squared_t maxAcceleration);
  SmartMotorControllerConfig& WithVelocityTrapezoidProfile(
      units::turns_per_second_t maxVelocity, units::turns_per_second_squared_t maxAcceleration);
  SmartMotorControllerConfig& WithExponentialProfile(double kV, double kA, units::volt_t maxInput);

  // ---- LQR ---------------------------------------------------------------
  SmartMotorControllerConfig& WithLQR(
      const math::LQRConfig& lqrConfig,
      ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);

  // ---- Gearing / linear --------------------------------------------------
  SmartMotorControllerConfig& WithMotorGearing(const gearing::MechanismGearing& gearing);
  SmartMotorControllerConfig& WithMechanismCircumference(units::meter_t circumference);

  // ---- Limits ------------------------------------------------------------
  SmartMotorControllerConfig& WithMechanismLimits(units::degree_t lower, units::degree_t upper);
  SmartMotorControllerConfig& WithMeasurementLimits(units::meter_t lower, units::meter_t upper);
  SmartMotorControllerConfig& WithStatorCurrentLimit(units::ampere_t limit);
  SmartMotorControllerConfig& WithSupplyCurrentLimit(units::ampere_t limit);
  SmartMotorControllerConfig& WithTemperatureCutoff(units::celsius_t temperature);
  SmartMotorControllerConfig& WithClosedLoopMaxVoltage(units::volt_t maxVoltage);

  // ---- Control behaviour -------------------------------------------------
  SmartMotorControllerConfig& WithIdleMode(MotorMode mode);
  SmartMotorControllerConfig& WithClosedLoopMode();
  SmartMotorControllerConfig& WithOpenLoopMode();
  SmartMotorControllerConfig& WithClosedLoopControlPeriod(units::second_t period);

  // ---- Ramp rates --------------------------------------------------------
  SmartMotorControllerConfig& WithOpenLoopRampRate(units::second_t rampRate);
  SmartMotorControllerConfig& WithClosedLoopRampRate(units::second_t rampRate);

  // ---- Inversion ---------------------------------------------------------
  SmartMotorControllerConfig& WithMotorInverted(bool inverted);
  SmartMotorControllerConfig& WithEncoderInverted(bool inverted);

  // ---- External encoder --------------------------------------------------
  SmartMotorControllerConfig& WithAbsoluteEncoderConversionFactor(double factor);
  SmartMotorControllerConfig& WithAbsoluteEncoderOffset(units::degree_t offset);
  SmartMotorControllerConfig& WithAbsoluteEncoderZeroOffset(units::degree_t zeroOffset);

  // ---- Telemetry ----------------------------------------------------------
  SmartMotorControllerConfig& WithTelemetry(
      const std::string& name, TelemetryVerbosity verbosity = TelemetryVerbosity::HIGH);

  // ---- Subsystem ---------------------------------------------------------
  SmartMotorControllerConfig& WithSubsystem(frc2::SubsystemBase* subsystem);

  // ---- Simulation --------------------------------------------------------
  SmartMotorControllerConfig& WithSimMotor(frc::DCMotor motor);

  // === Getters ============================================================
  struct PIDGains {
    double kP{0.0}, kI{0.0}, kD{0.0};
    double kS{0.0}, kV{0.0}, kA{0.0}, kG{0.0};
    std::optional<frc::ArmFeedforward> armFF;
    std::optional<frc::ElevatorFeedforward> elevatorFF;
    std::optional<frc::SimpleMotorFeedforward<units::turns>> simpleFF;
    std::optional<math::LQRConfig> lqr;
  };

  const PIDGains& GetSlotGains(ClosedLoopControllerSlot slot) const;

  std::optional<frc::ArmFeedforward> GetArmFeedforward(ClosedLoopControllerSlot slot) const;
  std::optional<frc::ElevatorFeedforward> GetElevatorFeedforward(
      ClosedLoopControllerSlot slot) const;
  std::optional<frc::SimpleMotorFeedforward<units::turns>> GetSimpleFeedforward(
      ClosedLoopControllerSlot slot) const;
  std::optional<math::LQRConfig> GetLQR(ClosedLoopControllerSlot slot) const;

  double GetKp(ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0) const;
  double GetKi(ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0) const;
  double GetKd(ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0) const;

  bool GetLinearClosedLoopControllerUse() const;

  std::optional<units::degree_t> GetMechanismLowerLimit() const;
  std::optional<units::degree_t> GetMechanismUpperLimit() const;
  std::optional<units::meter_t> GetMeasurementLowerLimit() const;
  std::optional<units::meter_t> GetMeasurementUpperLimit() const;

  std::optional<units::ampere_t> GetStatorCurrentLimit() const;
  std::optional<int> GetStatorStallCurrentLimit() const;
  std::optional<units::ampere_t> GetSupplyCurrentLimit() const;
  std::optional<units::celsius_t> GetTemperatureCutoff() const;
  std::optional<units::volt_t> GetClosedLoopControllerMaximumVoltage() const;

  ControlMode GetMotorControllerMode() const;
  MotorMode GetIdleMode() const;
  std::optional<units::second_t> GetClosedLoopControlPeriod() const;
  std::optional<units::second_t> GetOpenLoopRampRate() const;
  std::optional<units::second_t> GetClosedLoopRampRate() const;

  bool GetMotorInverted() const;
  bool GetEncoderInverted() const;
  bool GetVelocityTrapezoidalProfileInUse() const;

  std::optional<std::string> GetTelemetryName() const;
  std::optional<TelemetryVerbosity> GetVerbosity() const;
  frc2::SubsystemBase* GetSubsystem() const;
  std::optional<frc::DCMotor> GetSimMotor() const;

  const std::optional<gearing::MechanismGearing>& GetMotorGearing() const;
  std::optional<units::meter_t> GetMechanismCircumference() const;

  std::optional<double> GetAbsoluteEncoderConversionFactor() const;
  std::optional<units::degree_t> GetAbsoluteEncoderOffset() const;
  std::optional<units::degree_t> GetAbsoluteEncoderZeroOffset() const;

  bool HasTrapezoidProfile() const;
  bool HasExponentialProfile() const;

  std::optional<frc::TrapezoidProfile<units::turns>> GetTrapezoidProfile() const;
  std::optional<frc::TrapezoidProfile<units::meters>> GetLinearTrapezoidProfile() const;
  std::optional<frc::ExponentialProfile<units::turns, units::volts>> GetExponentialProfile() const;

  // Constraint values extracted for hardware motor controller configuration
  std::optional<units::turns_per_second_t> GetTrapMaxVelocityTurns() const;
  std::optional<units::turns_per_second_squared_t> GetTrapMaxAccelTurns() const;
  std::optional<units::meters_per_second_t> GetTrapMaxVelocityLinear() const;
  std::optional<units::meters_per_second_squared_t> GetTrapMaxAccelLinear() const;

  units::meter_t ConvertFromMechanism(units::degree_t mechanismAngle) const;
  units::meters_per_second_t ConvertFromMechanism(
      units::degrees_per_second_t mechanismVelocity) const;

 private:
  static constexpr int kNumSlots = 4;

  PIDGains m_slots[kNumSlots];
  int SlotIndex(ClosedLoopControllerSlot slot) const;

  // Profiles
  std::optional<frc::TrapezoidProfile<units::turns>> m_trapProfile;
  std::optional<frc::TrapezoidProfile<units::meters>> m_linearTrapProfile;
  std::optional<frc::ExponentialProfile<units::turns, units::volts>> m_expoProfile;
  bool m_velocityTrapProfile{false};
  // Stored constraint values for hardware motor controller configuration
  std::optional<units::turns_per_second_t> m_trapMaxVelTurns;
  std::optional<units::turns_per_second_squared_t> m_trapMaxAccTurns;
  std::optional<units::meters_per_second_t> m_trapMaxVelLinear;
  std::optional<units::meters_per_second_squared_t> m_trapMaxAccLinear;

  // Limits
  std::optional<units::degree_t> m_mechLowerLimit;
  std::optional<units::degree_t> m_mechUpperLimit;
  std::optional<units::meter_t> m_measLowerLimit;
  std::optional<units::meter_t> m_measUpperLimit;
  std::optional<units::ampere_t> m_statorCurrentLimit;
  std::optional<units::ampere_t> m_supplyCurrentLimit;
  std::optional<units::celsius_t> m_temperatureCutoff;
  std::optional<units::volt_t> m_closedLoopMaxVoltage;

  // Control behaviour
  ControlMode m_controlMode{ControlMode::CLOSED_LOOP};
  MotorMode m_idleMode{MotorMode::COAST};
  std::optional<units::second_t> m_closedLoopPeriod;
  std::optional<units::second_t> m_openLoopRampRate;
  std::optional<units::second_t> m_closedLoopRampRate;

  // Inversion
  bool m_motorInverted{false};
  bool m_encoderInverted{false};

  // Gearing / linear
  std::optional<gearing::MechanismGearing> m_motorGearing;
  std::optional<units::meter_t> m_mechanismCircumference;

  // External encoder
  std::optional<double> m_absEncoderConversionFactor;
  std::optional<units::degree_t> m_absEncoderOffset;
  std::optional<units::degree_t> m_absEncoderZeroOffset;

  // Telemetry
  std::optional<std::string> m_telemetryName;
  std::optional<TelemetryVerbosity> m_verbosity;

  frc2::SubsystemBase* m_subsystem{nullptr};
  std::optional<frc::DCMotor> m_simMotor;
};

}  // namespace yams::motorcontrollers
