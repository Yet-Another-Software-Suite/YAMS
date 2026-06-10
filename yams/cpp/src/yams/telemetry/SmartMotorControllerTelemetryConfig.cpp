// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/telemetry/SmartMotorControllerTelemetryConfig.hpp"

#include <units/angle.h>
#include <units/length.h>

#include <string>
#include <unordered_map>

#include "yams/motorcontrollers/SmartMotorController.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"

namespace yams::telemetry {

using motorcontrollers::SmartMotorController;
using motorcontrollers::SmartMotorControllerConfig;

// Helper: create a DoubleTelemetry for a given field
static DoubleTelemetry MakeDouble(DoubleTelemetryField field) {
  switch (field) {
    case DoubleTelemetryField::ExponentialProfileKV:
      return {"closedloop/motionprofile/kV", 0.0, field, true, "none"};
    case DoubleTelemetryField::ExponentialProfileKA:
      return {"closedloop/motionprofile/kA", 0.0, field, true, "none"};
    case DoubleTelemetryField::ExponentialProfileMaxInput:
      return {"closedloop/motionprofile/maxInput", 12.0, field, true, "volts"};
    case DoubleTelemetryField::TrapezoidalProfileMaxVelocity:
      return {"closedloop/motionprofile/maxVelocity", 0.0, field, true, "tunable_velocity"};
    case DoubleTelemetryField::TrapezoidalProfileMaxAcceleration:
      return {"closedloop/motionprofile/maxAcceleration", 0.0, field, true, "tunable_acceleration"};
    case DoubleTelemetryField::TrapezoidalProfileMaxJerk:
      return {"closedloop/motionprofile/maxJerk", 0.0, field, true,
              "rotations_per_minute_per_second_per_second"};
    case DoubleTelemetryField::TunableClosedLoopControllerSlot:
      return {"closedloop/slot", 0.0, field, true, "none"};
    case DoubleTelemetryField::ActiveClosedLoopControllerSlot:
      return {"closedloop/slot", 0.0, field, false, "none"};
    case DoubleTelemetryField::kS:
      return {"closedloop/feedforward/kS", 0.0, field, true, "none"};
    case DoubleTelemetryField::kV:
      return {"closedloop/feedforward/kV", 0.0, field, true, "none"};
    case DoubleTelemetryField::kA:
      return {"closedloop/feedforward/kA", 0.0, field, true, "none"};
    case DoubleTelemetryField::kG:
      return {"closedloop/feedforward/kG", 0.0, field, true, "none"};
    case DoubleTelemetryField::kP:
      return {"closedloop/feedback/kP", 0.0, field, true, "none"};
    case DoubleTelemetryField::kI:
      return {"closedloop/feedback/kI", 0.0, field, true, "none"};
    case DoubleTelemetryField::kD:
      return {"closedloop/feedback/kD", 0.0, field, true, "none"};
    case DoubleTelemetryField::TunableSetpointPosition:
      return {"closedloop/setpoint/position", 0.0, field, true, "tunable_position"};
    case DoubleTelemetryField::SetpointPosition:
      return {"closedloop/setpoint/position", 0.0, field, false, "position"};
    case DoubleTelemetryField::TunableSetpointVelocity:
      return {"closedloop/setpoint/velocity", 0.0, field, true, "tunable_velocity"};
    case DoubleTelemetryField::SetpointVelocity:
      return {"closedloop/setpoint/velocity", 0.0, field, false, "velocity"};
    case DoubleTelemetryField::OutputVoltage:
      return {"motor/outputVoltage", 0.0, field, false, "volts"};
    case DoubleTelemetryField::StatorCurrent:
      return {"current/stator", 0.0, field, false, "amps"};
    case DoubleTelemetryField::StatorCurrentLimit:
      return {"current/limit/stator", 0.0, field, true, "amps"};
    case DoubleTelemetryField::SupplyCurrent:
      return {"current/supply", 0.0, field, false, "amps"};
    case DoubleTelemetryField::SupplyCurrentLimit:
      return {"current/limit/supply", 0.0, field, true, "amps"};
    case DoubleTelemetryField::MotorTemperature:
      return {"motor/temperature", 0.0, field, false, "fahrenheit"};
    case DoubleTelemetryField::MeasurementPosition:
      return {"measurement/position", 0.0, field, false, "meters"};
    case DoubleTelemetryField::MeasurementVelocity:
      return {"measurement/velocity", 0.0, field, false, "meters_per_second"};
    case DoubleTelemetryField::MeasurementAcceleration:
      return {"measurement/acceleration", 0.0, field, false, "meters_per_second_per_second"};
    case DoubleTelemetryField::MeasurementLowerLimit:
      return {"measurement/limit/lower", 0.0, field, true, "meters"};
    case DoubleTelemetryField::MeasurementUpperLimit:
      return {"measurement/limit/upper", 0.0, field, true, "meters"};
    case DoubleTelemetryField::MechanismPosition:
      return {"mechanism/position", 0.0, field, false, "rotations"};
    case DoubleTelemetryField::MechanismVelocity:
      return {"mechanism/velocity", 0.0, field, false, "rotations_per_second"};
    case DoubleTelemetryField::MechanismAcceleration:
      return {"mechanism/acceleration", 0.0, field, false, "rotations_per_second_per_second"};
    case DoubleTelemetryField::MechanismLowerLimit:
      return {"mechanism/limit/lower", 0.0, field, true, "degrees"};
    case DoubleTelemetryField::MechanismUpperLimit:
      return {"mechanism/limit/upper", 0.0, field, true, "degrees"};
    case DoubleTelemetryField::RotorPosition:
      return {"rotor/position", 0.0, field, false, "rotations"};
    case DoubleTelemetryField::RotorVelocity:
      return {"rotor/velocity", 0.0, field, false, "rotations_per_second"};
    case DoubleTelemetryField::ExternalEncoderPosition:
      return {"externalencoder/position", 0.0, field, false, "rotations"};
    case DoubleTelemetryField::ExternalEncoderVelocity:
      return {"externalencoder/velocity", 0.0, field, false, "rotations_per_second"};
    case DoubleTelemetryField::ClosedloopRampRate:
      return {"ramprate/dutycycle/closedloop", 0.0, field, true, "seconds"};
    case DoubleTelemetryField::OpenloopRampRate:
      return {"ramprate/dutycycle/openloop", 0.0, field, true, "seconds"};
    default:
      return {"unknown", 0.0, field, false, "none"};
  }
}

static BooleanTelemetry MakeBool(BooleanTelemetryField field) {
  switch (field) {
    case BooleanTelemetryField::MechanismUpperLimit:
      return {"limits/mechanism/upper", false, field, false};
    case BooleanTelemetryField::MechanismLowerLimit:
      return {"limits/mechanism/lower", false, field, false};
    case BooleanTelemetryField::TemperatureLimit:
      return {"limits/temperature", false, field, false};
    case BooleanTelemetryField::VelocityControl:
      return {"control/closedloop/velocity", false, field, false};
    case BooleanTelemetryField::ElevatorFeedForward:
      return {"control/feedforward/elevator", false, field, false};
    case BooleanTelemetryField::ArmFeedForward:
      return {"control/feedforward/arm", false, field, false};
    case BooleanTelemetryField::SimpleMotorFeedForward:
      return {"control/Simple Motor Feedforward", false, field, false};
    case BooleanTelemetryField::MotionProfile:
      return {"control/closedloop/profiled", false, field, false};
    case BooleanTelemetryField::MotorInversion:
      return {"motor/inverted", false, field, true};
    case BooleanTelemetryField::EncoderInversion:
      return {"encoder/inverted", false, field, true};
    default:
      return {"unknown", false, field, false};
  }
}

SmartMotorControllerTelemetryConfig::SmartMotorControllerTelemetryConfig() {
  // Populate maps with all fields (all disabled by default)
  for (auto f : {
           DoubleTelemetryField::ExponentialProfileKV,
           DoubleTelemetryField::ExponentialProfileKA,
           DoubleTelemetryField::ExponentialProfileMaxInput,
           DoubleTelemetryField::TrapezoidalProfileMaxVelocity,
           DoubleTelemetryField::TrapezoidalProfileMaxAcceleration,
           DoubleTelemetryField::TrapezoidalProfileMaxJerk,
           DoubleTelemetryField::TunableClosedLoopControllerSlot,
           DoubleTelemetryField::ActiveClosedLoopControllerSlot,
           DoubleTelemetryField::kS,
           DoubleTelemetryField::kV,
           DoubleTelemetryField::kA,
           DoubleTelemetryField::kG,
           DoubleTelemetryField::kP,
           DoubleTelemetryField::kI,
           DoubleTelemetryField::kD,
           DoubleTelemetryField::TunableSetpointPosition,
           DoubleTelemetryField::SetpointPosition,
           DoubleTelemetryField::TunableSetpointVelocity,
           DoubleTelemetryField::SetpointVelocity,
           DoubleTelemetryField::OutputVoltage,
           DoubleTelemetryField::StatorCurrent,
           DoubleTelemetryField::StatorCurrentLimit,
           DoubleTelemetryField::SupplyCurrent,
           DoubleTelemetryField::SupplyCurrentLimit,
           DoubleTelemetryField::MotorTemperature,
           DoubleTelemetryField::MeasurementPosition,
           DoubleTelemetryField::MeasurementVelocity,
           DoubleTelemetryField::MeasurementAcceleration,
           DoubleTelemetryField::MeasurementLowerLimit,
           DoubleTelemetryField::MeasurementUpperLimit,
           DoubleTelemetryField::MechanismPosition,
           DoubleTelemetryField::MechanismVelocity,
           DoubleTelemetryField::MechanismAcceleration,
           DoubleTelemetryField::MechanismLowerLimit,
           DoubleTelemetryField::MechanismUpperLimit,
           DoubleTelemetryField::RotorPosition,
           DoubleTelemetryField::RotorVelocity,
           DoubleTelemetryField::ExternalEncoderPosition,
           DoubleTelemetryField::ExternalEncoderVelocity,
           DoubleTelemetryField::ClosedloopRampRate,
           DoubleTelemetryField::OpenloopRampRate,
       }) {
    m_doubleFields.emplace(f, MakeDouble(f));
  }
  for (auto f : {
           BooleanTelemetryField::MechanismUpperLimit,
           BooleanTelemetryField::MechanismLowerLimit,
           BooleanTelemetryField::TemperatureLimit,
           BooleanTelemetryField::VelocityControl,
           BooleanTelemetryField::ElevatorFeedForward,
           BooleanTelemetryField::ArmFeedForward,
           BooleanTelemetryField::SimpleMotorFeedForward,
           BooleanTelemetryField::MotionProfile,
           BooleanTelemetryField::MotorInversion,
           BooleanTelemetryField::EncoderInversion,
       }) {
    m_boolFields.emplace(f, MakeBool(f));
  }
}

SmartMotorControllerTelemetryConfig& SmartMotorControllerTelemetryConfig::WithDataLogName(
    const std::string& name) {
  m_dataLogName = name;
  return *this;
}

SmartMotorControllerTelemetryConfig& SmartMotorControllerTelemetryConfig::WithNetworkTables(
    bool enabled) {
  m_nt4Telemetry = enabled;
  return *this;
}

SmartMotorControllerTelemetryConfig& SmartMotorControllerTelemetryConfig::WithoutNetworkTables() {
  m_nt4Telemetry = false;
  return *this;
}

SmartMotorControllerTelemetryConfig& SmartMotorControllerTelemetryConfig::WithTelemetryVerbosity(
    TelemetryVerbosity verbosity) {
  using V = SmartMotorControllerConfig::TelemetryVerbosity;

  // Cascading fall-through: HIGH enables everything below
  if (verbosity == V::HIGH) {
    m_boolFields.at(BooleanTelemetryField::MechanismLowerLimit).Enable();
    m_boolFields.at(BooleanTelemetryField::MechanismUpperLimit).Enable();
    m_boolFields.at(BooleanTelemetryField::TemperatureLimit).Enable();
    m_boolFields.at(BooleanTelemetryField::VelocityControl).Enable();
    m_boolFields.at(BooleanTelemetryField::ElevatorFeedForward).Enable();
    m_boolFields.at(BooleanTelemetryField::ArmFeedForward).Enable();
    m_boolFields.at(BooleanTelemetryField::SimpleMotorFeedForward).Enable();
    m_boolFields.at(BooleanTelemetryField::MotionProfile).Enable();
    m_boolFields.at(BooleanTelemetryField::MotorInversion).Enable();
    m_boolFields.at(BooleanTelemetryField::EncoderInversion).Enable();
    m_doubleFields.at(DoubleTelemetryField::TunableSetpointPosition).Enable();
    m_doubleFields.at(DoubleTelemetryField::TunableSetpointVelocity).Enable();
    m_doubleFields.at(DoubleTelemetryField::TunableClosedLoopControllerSlot).Enable();
    m_doubleFields.at(DoubleTelemetryField::MotorTemperature).Enable();
    m_doubleFields.at(DoubleTelemetryField::MechanismLowerLimit).Enable();
    m_doubleFields.at(DoubleTelemetryField::MechanismUpperLimit).Enable();
    m_doubleFields.at(DoubleTelemetryField::StatorCurrentLimit).Enable();
    m_doubleFields.at(DoubleTelemetryField::SupplyCurrentLimit).Enable();
    m_doubleFields.at(DoubleTelemetryField::OpenloopRampRate).Enable();
    m_doubleFields.at(DoubleTelemetryField::ClosedloopRampRate).Enable();
    m_doubleFields.at(DoubleTelemetryField::MeasurementLowerLimit).Enable();
    m_doubleFields.at(DoubleTelemetryField::MeasurementUpperLimit).Enable();
    m_doubleFields.at(DoubleTelemetryField::kS).Enable();
    m_doubleFields.at(DoubleTelemetryField::kV).Enable();
    m_doubleFields.at(DoubleTelemetryField::kG).Enable();
    m_doubleFields.at(DoubleTelemetryField::kA).Enable();
    m_doubleFields.at(DoubleTelemetryField::kP).Enable();
    m_doubleFields.at(DoubleTelemetryField::kI).Enable();
    m_doubleFields.at(DoubleTelemetryField::kD).Enable();
  }
  if (verbosity == V::HIGH || verbosity == V::MEDIUM) {
    m_doubleFields.at(DoubleTelemetryField::OutputVoltage).Enable();
    m_doubleFields.at(DoubleTelemetryField::StatorCurrent).Enable();
    m_doubleFields.at(DoubleTelemetryField::SupplyCurrent).Enable();
  }
  if (verbosity == V::HIGH || verbosity == V::MEDIUM || verbosity == V::LOW) {
    m_doubleFields.at(DoubleTelemetryField::ActiveClosedLoopControllerSlot).Enable();
    m_doubleFields.at(DoubleTelemetryField::SetpointPosition).Enable();
    m_doubleFields.at(DoubleTelemetryField::SetpointVelocity).Enable();
    m_doubleFields.at(DoubleTelemetryField::MeasurementPosition).Enable();
    m_doubleFields.at(DoubleTelemetryField::MeasurementVelocity).Enable();
    m_doubleFields.at(DoubleTelemetryField::MeasurementAcceleration).Enable();
    m_doubleFields.at(DoubleTelemetryField::MechanismPosition).Enable();
    m_doubleFields.at(DoubleTelemetryField::MechanismVelocity).Enable();
    m_doubleFields.at(DoubleTelemetryField::MechanismAcceleration).Enable();
    m_doubleFields.at(DoubleTelemetryField::RotorPosition).Enable();
    m_doubleFields.at(DoubleTelemetryField::RotorVelocity).Enable();
    m_doubleFields.at(DoubleTelemetryField::ExternalEncoderPosition).Enable();
    m_doubleFields.at(DoubleTelemetryField::ExternalEncoderVelocity).Enable();
  }
  return *this;
}

// ---- Individual field enables -----------------------------------------------

SmartMotorControllerTelemetryConfig&
SmartMotorControllerTelemetryConfig::WithMechanismLowerLimit() {
  m_boolFields.at(BooleanTelemetryField::MechanismLowerLimit).Enable();
  return *this;
}
SmartMotorControllerTelemetryConfig&
SmartMotorControllerTelemetryConfig::WithMechanismUpperLimit() {
  m_boolFields.at(BooleanTelemetryField::MechanismUpperLimit).Enable();
  return *this;
}
SmartMotorControllerTelemetryConfig& SmartMotorControllerTelemetryConfig::WithTemperatureLimit() {
  m_boolFields.at(BooleanTelemetryField::TemperatureLimit).Enable();
  return *this;
}
SmartMotorControllerTelemetryConfig& SmartMotorControllerTelemetryConfig::WithVelocityControl() {
  m_boolFields.at(BooleanTelemetryField::VelocityControl).Enable();
  return *this;
}
SmartMotorControllerTelemetryConfig&
SmartMotorControllerTelemetryConfig::WithElevatorFeedforward() {
  m_boolFields.at(BooleanTelemetryField::ElevatorFeedForward).Enable();
  return *this;
}
SmartMotorControllerTelemetryConfig& SmartMotorControllerTelemetryConfig::WithArmFeedforward() {
  m_boolFields.at(BooleanTelemetryField::ArmFeedForward).Enable();
  return *this;
}
SmartMotorControllerTelemetryConfig& SmartMotorControllerTelemetryConfig::WithSimpleFeedforward() {
  m_boolFields.at(BooleanTelemetryField::SimpleMotorFeedForward).Enable();
  return *this;
}
SmartMotorControllerTelemetryConfig& SmartMotorControllerTelemetryConfig::WithMotionProfile() {
  m_boolFields.at(BooleanTelemetryField::MotionProfile).Enable();
  return *this;
}
SmartMotorControllerTelemetryConfig& SmartMotorControllerTelemetryConfig::WithSetpointPosition() {
  m_doubleFields.at(DoubleTelemetryField::SetpointPosition).Enable();
  return *this;
}
SmartMotorControllerTelemetryConfig& SmartMotorControllerTelemetryConfig::WithSetpointVelocity() {
  m_doubleFields.at(DoubleTelemetryField::SetpointVelocity).Enable();
  return *this;
}
SmartMotorControllerTelemetryConfig& SmartMotorControllerTelemetryConfig::WithOutputVoltage() {
  m_doubleFields.at(DoubleTelemetryField::OutputVoltage).Enable();
  return *this;
}
SmartMotorControllerTelemetryConfig& SmartMotorControllerTelemetryConfig::WithStatorCurrent() {
  m_doubleFields.at(DoubleTelemetryField::StatorCurrent).Enable();
  return *this;
}
SmartMotorControllerTelemetryConfig& SmartMotorControllerTelemetryConfig::WithTemperature() {
  m_doubleFields.at(DoubleTelemetryField::MotorTemperature).Enable();
  return *this;
}
SmartMotorControllerTelemetryConfig&
SmartMotorControllerTelemetryConfig::WithMeasurementPosition() {
  m_doubleFields.at(DoubleTelemetryField::MeasurementPosition).Enable();
  return *this;
}
SmartMotorControllerTelemetryConfig&
SmartMotorControllerTelemetryConfig::WithMeasurementVelocity() {
  m_doubleFields.at(DoubleTelemetryField::MeasurementVelocity).Enable();
  return *this;
}
SmartMotorControllerTelemetryConfig& SmartMotorControllerTelemetryConfig::WithMechanismPosition() {
  m_doubleFields.at(DoubleTelemetryField::MechanismPosition).Enable();
  return *this;
}
SmartMotorControllerTelemetryConfig& SmartMotorControllerTelemetryConfig::WithMechanismVelocity() {
  m_doubleFields.at(DoubleTelemetryField::MechanismVelocity).Enable();
  return *this;
}
SmartMotorControllerTelemetryConfig& SmartMotorControllerTelemetryConfig::WithRotorPosition() {
  m_doubleFields.at(DoubleTelemetryField::RotorPosition).Enable();
  return *this;
}
SmartMotorControllerTelemetryConfig& SmartMotorControllerTelemetryConfig::WithRotorVelocity() {
  m_doubleFields.at(DoubleTelemetryField::RotorVelocity).Enable();
  return *this;
}

// ---- Accessors --------------------------------------------------------------

std::optional<std::string> SmartMotorControllerTelemetryConfig::GetDataLogName() const {
  return m_dataLogName;
}
bool SmartMotorControllerTelemetryConfig::GetNT4Enabled() const { return m_nt4Telemetry; }

std::unordered_map<DoubleTelemetryField, DoubleTelemetry>&
SmartMotorControllerTelemetryConfig::GetDoubleFields(SmartMotorController& smc) {
  SmartMotorControllerConfig& cfg = smc.GetConfig();
  auto slot = smc.GetClosedLoopControllerSlot();

  // Apply unsupported field constraints from motor controller
  auto unsup = smc.GetUnsupportedTelemetryFields();
  if (unsup.boolFields) {
    for (auto f : *unsup.boolFields) {
      auto it = m_boolFields.find(f);
      if (it != m_boolFields.end()) it->second.Disable();
    }
  }
  if (unsup.doubleFields) {
    for (auto f : *unsup.doubleFields) {
      auto it = m_doubleFields.find(f);
      if (it != m_doubleFields.end()) it->second.Disable();
    }
  }

  // Supply current not available
  if (!smc.GetSupplyCurrent()) {
    m_doubleFields.at(DoubleTelemetryField::SupplyCurrent).Disable();
    m_doubleFields.at(DoubleTelemetryField::SupplyCurrentLimit).Disable();
  }

  // kG only makes sense with Arm/Elevator feedforward
  if (!cfg.GetArmFeedforward(slot) && !cfg.GetElevatorFeedforward(slot)) {
    m_doubleFields.at(DoubleTelemetryField::kG).Disable();
  }

  // Measurement (linear) fields
  if (!cfg.GetMechanismCircumference()) {
    m_doubleFields.at(DoubleTelemetryField::MeasurementLowerLimit).Disable();
    m_doubleFields.at(DoubleTelemetryField::MeasurementUpperLimit).Disable();
    m_doubleFields.at(DoubleTelemetryField::MeasurementPosition).Disable();
    m_doubleFields.at(DoubleTelemetryField::MeasurementVelocity).Disable();
    m_doubleFields.at(DoubleTelemetryField::MeasurementAcceleration).Disable();
  } else {
    // Set default values from config limits
    if (auto lim = cfg.GetMechanismUpperLimit())
      m_doubleFields.at(DoubleTelemetryField::MeasurementUpperLimit)
          .SetDefaultValue(cfg.ConvertFromMechanism(*lim).value());
    if (auto lim = cfg.GetMechanismLowerLimit())
      m_doubleFields.at(DoubleTelemetryField::MeasurementLowerLimit)
          .SetDefaultValue(cfg.ConvertFromMechanism(*lim).value());
  }

  // Set mechanism limit defaults (in degrees for human readability)
  if (auto lim = cfg.GetMechanismUpperLimit())
    m_doubleFields.at(DoubleTelemetryField::MechanismUpperLimit)
        .SetDefaultValue(units::degree_t{*lim}.value());
  if (auto lim = cfg.GetMechanismLowerLimit())
    m_doubleFields.at(DoubleTelemetryField::MechanismLowerLimit)
        .SetDefaultValue(units::degree_t{*lim}.value());

  // Current limit defaults
  if (auto lim = cfg.GetSupplyStallCurrentLimit())
    m_doubleFields.at(DoubleTelemetryField::SupplyCurrentLimit).SetDefaultValue(*lim);
  if (auto lim = cfg.GetStatorStallCurrentLimit())
    m_doubleFields.at(DoubleTelemetryField::StatorCurrentLimit).SetDefaultValue(*lim);

  // PID defaults
  const auto& gains = cfg.GetSlotGains(slot);
  m_doubleFields.at(DoubleTelemetryField::kP).SetDefaultValue(gains.kP);
  m_doubleFields.at(DoubleTelemetryField::kI).SetDefaultValue(gains.kI);
  m_doubleFields.at(DoubleTelemetryField::kD).SetDefaultValue(gains.kD);

  // Trapezoidal profile
  if (auto trap = cfg.GetTrapezoidProfile()) {
    m_doubleFields.at(DoubleTelemetryField::ExponentialProfileMaxInput).Disable();
    m_doubleFields.at(DoubleTelemetryField::ExponentialProfileKA).Disable();
    m_doubleFields.at(DoubleTelemetryField::ExponentialProfileKV).Disable();
    m_doubleFields.at(DoubleTelemetryField::TrapezoidalProfileMaxAcceleration).Enable();

    if (cfg.GetVelocityTrapezoidalProfileInUse()) {
      m_doubleFields.at(DoubleTelemetryField::TrapezoidalProfileMaxJerk).Enable();
      m_doubleFields.at(DoubleTelemetryField::TrapezoidalProfileMaxVelocity).Disable();
    } else if (cfg.GetLinearClosedLoopControllerUse()) {
      m_doubleFields.at(DoubleTelemetryField::TrapezoidalProfileMaxVelocity).Enable();
      m_doubleFields.at(DoubleTelemetryField::TrapezoidalProfileMaxJerk).Disable();
      if (auto lv = cfg.GetTrapMaxVelocityLinear())
        m_doubleFields.at(DoubleTelemetryField::TrapezoidalProfileMaxVelocity)
            .SetDefaultValue(lv->value());
      if (auto la = cfg.GetTrapMaxAccelLinear())
        m_doubleFields.at(DoubleTelemetryField::TrapezoidalProfileMaxAcceleration)
            .SetDefaultValue(la->value());
    } else {
      m_doubleFields.at(DoubleTelemetryField::TrapezoidalProfileMaxVelocity).Enable();
      m_doubleFields.at(DoubleTelemetryField::TrapezoidalProfileMaxJerk).Disable();
      // Convert turns/s and turns/s² to RPM and RPM/s
      if (auto mv = cfg.GetTrapMaxVelocityTurns())
        m_doubleFields.at(DoubleTelemetryField::TrapezoidalProfileMaxVelocity)
            .SetDefaultValue(mv->value() * 60.0);  // turns/s → RPM
      if (auto ma = cfg.GetTrapMaxAccelTurns())
        m_doubleFields.at(DoubleTelemetryField::TrapezoidalProfileMaxAcceleration)
            .SetDefaultValue(ma->value() * 60.0);  // turns/s² → RPM/s
    }
  }

  // Exponential profile
  if (cfg.HasExponentialProfile()) {
    m_doubleFields.at(DoubleTelemetryField::TrapezoidalProfileMaxAcceleration).Disable();
    m_doubleFields.at(DoubleTelemetryField::TrapezoidalProfileMaxVelocity).Disable();
    m_doubleFields.at(DoubleTelemetryField::TrapezoidalProfileMaxJerk).Disable();
    m_doubleFields.at(DoubleTelemetryField::ExponentialProfileKA).Enable();
    m_doubleFields.at(DoubleTelemetryField::ExponentialProfileKV).Enable();
    m_doubleFields.at(DoubleTelemetryField::ExponentialProfileMaxInput).Enable();
    // ExponentialProfile constraints are not publicly accessible in WPILib C++ 2026;
    // defaults remain at 0 and are tuned via NetworkTables.
  }

  // LQR: disable PID tuning (LQR computes gains internally)
  if (cfg.GetLQR(slot)) {
    m_doubleFields.at(DoubleTelemetryField::kP).Disable();
    m_doubleFields.at(DoubleTelemetryField::kI).Disable();
    m_doubleFields.at(DoubleTelemetryField::kD).Disable();
  }

  // Feedforward defaults — getters return units types, use .value() for raw doubles
  if (auto ff = cfg.GetArmFeedforward(slot)) {
    m_doubleFields.at(DoubleTelemetryField::kG).Enable();
    m_doubleFields.at(DoubleTelemetryField::kS).SetDefaultValue(ff->GetKs().value());
    m_doubleFields.at(DoubleTelemetryField::kV).SetDefaultValue(ff->GetKv().value());
    m_doubleFields.at(DoubleTelemetryField::kA).SetDefaultValue(ff->GetKa().value());
    m_doubleFields.at(DoubleTelemetryField::kG).SetDefaultValue(ff->GetKg().value());
  }
  if (auto ff = cfg.GetElevatorFeedforward(slot)) {
    m_doubleFields.at(DoubleTelemetryField::kG).Enable();
    m_doubleFields.at(DoubleTelemetryField::kS).SetDefaultValue(ff->GetKs().value());
    m_doubleFields.at(DoubleTelemetryField::kV).SetDefaultValue(ff->GetKv().value());
    m_doubleFields.at(DoubleTelemetryField::kA).SetDefaultValue(ff->GetKa().value());
    m_doubleFields.at(DoubleTelemetryField::kG).SetDefaultValue(ff->GetKg().value());
  }
  if (auto ff = cfg.GetSimpleFeedforward(slot)) {
    m_doubleFields.at(DoubleTelemetryField::kG).Disable();
    m_doubleFields.at(DoubleTelemetryField::kS).SetDefaultValue(ff->GetKs().value());
    m_doubleFields.at(DoubleTelemetryField::kV).SetDefaultValue(ff->GetKv().value());
    m_doubleFields.at(DoubleTelemetryField::kA).SetDefaultValue(ff->GetKa().value());
  }

  // External encoder
  if (!smc.GetExternalEncoderPosition()) {
    m_doubleFields.at(DoubleTelemetryField::ExternalEncoderPosition).Disable();
  }
  if (!smc.GetExternalEncoderVelocity()) {
    m_doubleFields.at(DoubleTelemetryField::ExternalEncoderVelocity).Disable();
  }

  return m_doubleFields;
}

std::unordered_map<BooleanTelemetryField, BooleanTelemetry>&
SmartMotorControllerTelemetryConfig::GetBoolFields(SmartMotorController& smc) {
  SmartMotorControllerConfig& cfg = smc.GetConfig();
  auto slot = smc.GetClosedLoopControllerSlot();

  if (!cfg.GetArmFeedforward(slot))
    m_boolFields.at(BooleanTelemetryField::ArmFeedForward).Disable();
  if (!cfg.GetElevatorFeedforward(slot))
    m_boolFields.at(BooleanTelemetryField::ElevatorFeedForward).Disable();
  if (!cfg.GetSimpleFeedforward(slot))
    m_boolFields.at(BooleanTelemetryField::SimpleMotorFeedForward).Disable();

  if (auto inv = cfg.GetMotorInverted())
    m_boolFields.at(BooleanTelemetryField::MotorInversion).SetDefaultValue(*inv);
  else
    m_boolFields.at(BooleanTelemetryField::MotorInversion).Disable();

  if (auto inv = cfg.GetEncoderInverted())
    m_boolFields.at(BooleanTelemetryField::EncoderInversion).SetDefaultValue(*inv);
  else
    m_boolFields.at(BooleanTelemetryField::EncoderInversion).Disable();

  return m_boolFields;
}

}  // namespace yams::telemetry
