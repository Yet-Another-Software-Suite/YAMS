// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/remote/TalonFXWrapper.hpp"

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include <units/angular_jerk.h>
#include <units/dimensionless.h>
#include <units/moment_of_inertia.h>

#include <cmath>

#include "yams/motorcontrollers/simulation/DCMotorSimSupplier.hpp"

using namespace ctre::phoenix6;

namespace yams::motorcontrollers::remote {

TalonFXWrapper::TalonFXWrapper(hardware::TalonFX& talon, frc::DCMotor dcMotor,
                               const SmartMotorControllerConfig& config)
    : SmartMotorController(), m_talon(talon), m_dcMotor(dcMotor) {
  m_config = config;
  SetupSimulation();
  ApplyConfig(config);
  CheckConfigSafety();
}

// ---- Configuration ----------------------------------------------------------

bool TalonFXWrapper::ApplyConfig(const SmartMotorControllerConfig& config) {
  m_config = config;
  auto& cfg = m_talonConfig;

  // Inversion
  if (auto inv = config.GetMotorInverted(); inv)
    cfg.MotorOutput.Inverted = *inv ? signals::InvertedValue::Clockwise_Positive
                                    : signals::InvertedValue::CounterClockwise_Positive;

  // Idle mode
  cfg.MotorOutput.NeutralMode = config.GetIdleMode() == SmartMotorControllerConfig::MotorMode::BRAKE
                                    ? signals::NeutralModeValue::Brake
                                    : signals::NeutralModeValue::Coast;

  // Gearing for sensor-to-mechanism
  if (auto& gearing = config.GetMotorGearing(); gearing) {
    cfg.Feedback.SensorToMechanismRatio =
        units::dimensionless::scalar_t{gearing->GetMechanismToRotorRatio()};
  }

  ApplyPIDConfig();
  ApplyFeedforwardConfig();
  ApplyLimitsConfig();
  ApplyMotionMagicConfig();

  // Ramp rates
  if (auto r = config.GetOpenLoopRampRate(); r) cfg.OpenLoopRamps.VoltageOpenLoopRampPeriod = *r;
  if (auto r = config.GetClosedLoopRampRate(); r)
    cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = *r;

  // Current limits
  if (auto stator = config.GetStatorCurrentLimit(); stator) {
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = *stator;
  }
  if (auto supply = config.GetSupplyCurrentLimit(); supply) {
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = *supply;
  }

  auto status = m_talon.GetConfigurator().Apply(cfg);
  return status.IsOK();
}

void TalonFXWrapper::ApplyPIDConfig() {
  auto gains =
      m_config.GetSlotGains(static_cast<SmartMotorControllerConfig::ClosedLoopControllerSlot>(0));
  m_talonConfig.Slot0.kP = gains.kP;
  m_talonConfig.Slot0.kI = gains.kI;
  m_talonConfig.Slot0.kD = gains.kD;
  m_talonConfig.Slot0.kS = gains.kS;
  m_talonConfig.Slot0.kV = gains.kV;
  m_talonConfig.Slot0.kA = gains.kA;

  auto gains1 = m_config.GetSlotGains(SmartMotorControllerConfig::ClosedLoopControllerSlot::SLOT_1);
  m_talonConfig.Slot1.kP = gains1.kP;
  m_talonConfig.Slot1.kI = gains1.kI;
  m_talonConfig.Slot1.kD = gains1.kD;
  m_talonConfig.Slot1.kS = gains1.kS;
  m_talonConfig.Slot1.kV = gains1.kV;
  m_talonConfig.Slot1.kA = gains1.kA;

  auto gains2 = m_config.GetSlotGains(SmartMotorControllerConfig::ClosedLoopControllerSlot::SLOT_2);
  m_talonConfig.Slot2.kP = gains2.kP;
  m_talonConfig.Slot2.kI = gains2.kI;
  m_talonConfig.Slot2.kD = gains2.kD;
  m_talonConfig.Slot2.kS = gains2.kS;
  m_talonConfig.Slot2.kV = gains2.kV;
  m_talonConfig.Slot2.kA = gains2.kA;
}

void TalonFXWrapper::ApplyFeedforwardConfig() {
  // Set gravity type for arm (kG on Slot0)
  auto gains = m_config.GetSlotGains(ClosedLoopControllerSlot::SLOT_0);
  if (gains.armFF) {
    m_talonConfig.Slot0.kG = gains.kG;
    m_talonConfig.Slot0.GravityType = signals::GravityTypeValue::Arm_Cosine;
  } else if (gains.elevatorFF) {
    m_talonConfig.Slot0.kG = gains.kG;
    m_talonConfig.Slot0.GravityType = signals::GravityTypeValue::Elevator_Static;
  }
}

void TalonFXWrapper::ApplyLimitsConfig() {
  bool limitEnabled = true;
  if (auto upper = m_config.GetMechanismUpperLimit(); upper) {
    m_talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = limitEnabled;
    m_talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = *upper;
  }
  if (auto lower = m_config.GetMechanismLowerLimit(); lower) {
    m_talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = limitEnabled;
    m_talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = *lower;
  }
}

void TalonFXWrapper::ApplyMotionMagicConfig() {
  bool hasTrap = m_config.HasTrapezoidProfile();
  bool hasExpo = m_config.HasExponentialProfile();
  bool velTrap = m_config.GetVelocityTrapezoidalProfileInUse();

  if (hasTrap && !velTrap) {
    if (auto v = m_config.GetTrapMaxVelocityTurns(); v)
      m_talonConfig.MotionMagic.MotionMagicCruiseVelocity = *v;
    if (auto a = m_config.GetTrapMaxAccelTurns(); a)
      m_talonConfig.MotionMagic.MotionMagicAcceleration = *a;
  } else if (hasTrap && velTrap) {
  } else if (hasExpo) {
  } else {
  }
}

// ---- Simulation -------------------------------------------------------------

void TalonFXWrapper::SetupSimulation() {
  if (!frc::RobotBase::IsSimulation() || m_simSupplier) return;

  auto simMotor = m_config.GetSimMotor();
  auto& gearing = m_config.GetMotorGearing();
  if (!simMotor || !gearing) return;

  auto plant = frc::LinearSystemId::DCMotorSystem(*simMotor, m_config.GetMOI(),
                                                  gearing->GetMechanismToRotorRatio());
  m_motorSim.emplace(plant, *simMotor);

  auto period = m_config.GetClosedLoopControlPeriod().value_or(20_ms);
  SetSimSupplier(std::make_shared<simulation::DCMotorSimSupplier>(
      *m_motorSim, [this]() { return GetDutyCycle(); }, *gearing, period));
}

void TalonFXWrapper::SimIterate() {
  if (!frc::RobotBase::IsSimulation() || !m_simSupplier) return;

  auto& sim = m_talon.GetSimState();
  sim.SetSupplyVoltage(m_simSupplier->GetMechanismSupplyVoltage());

  m_simSupplier->SetInputVoltage(sim.GetMotorVoltage());
  m_simSupplier->UpdateSim();

  sim.SetRawRotorPosition(units::turn_t{m_simSupplier->GetRotorPosition()});
  sim.SetRotorVelocity(units::turns_per_second_t{m_simSupplier->GetRotorVelocity()});
  sim.SetRotorAcceleration(
      units::turns_per_second_squared_t{m_simSupplier->GetRotorAcceleration()});

  if (m_cancoder) {
    auto& cancoderSim = m_cancoder->get().GetSimState();
    cancoderSim.SetSupplyVoltage(m_simSupplier->GetMechanismSupplyVoltage());
    cancoderSim.SetVelocity(units::turns_per_second_t{m_simSupplier->GetMechanismVelocity()});
    cancoderSim.SetRawPosition(units::turn_t{m_simSupplier->GetMechanismPosition()});
    cancoderSim.SetMagnetHealth(ctre::phoenix6::signals::MagnetHealthValue::Magnet_Green);
  }
}

// ---- Encoder sync -----------------------------------------------------------

void TalonFXWrapper::SeedRelativeEncoder() {
  // TalonFX uses an absolute sensor internally; no seeding needed unless external
}

void TalonFXWrapper::SynchronizeRelativeEncoder() {
  // TalonFX with CANcoder handles fusion automatically
}

// ---- Open-loop outputs ------------------------------------------------------

void TalonFXWrapper::SetDutyCycle(double dutyCycle) {
  m_talon.SetControl(m_dutyCycleReq.WithOutput(dutyCycle));
}

double TalonFXWrapper::GetDutyCycle() { return m_talon.GetDutyCycle().GetValue(); }

void TalonFXWrapper::SetVoltage(units::volt_t voltage) {
  m_talon.SetControl(m_voltageReq.WithOutput(voltage));
}

units::volt_t TalonFXWrapper::GetVoltage() { return m_talon.GetMotorVoltage().GetValue(); }

// ---- Closed-loop setpoints --------------------------------------------------

void TalonFXWrapper::SetPosition(units::turn_t angle) {
  m_setpointPosition = angle;
  if (m_config.GetMotorControllerMode() == ControlMode::CLOSED_LOOP &&
      !m_closedLoopControllerRunning) {
    bool hasTrap = m_config.HasTrapezoidProfile() && !m_config.GetVelocityTrapezoidalProfileInUse();
    bool hasExpo = m_config.HasExponentialProfile();
    if (hasTrap) {
      m_talon.SetControl(m_trapPositionReq.WithPosition(angle));
    } else if (hasExpo) {
      m_talon.SetControl(m_expoPositionReq.WithPosition(angle));
    } else {
      m_talon.SetControl(m_simplePositionReq.WithPosition(angle));
    }
  }
}

void TalonFXWrapper::SetPosition(units::meter_t distance) {
  if (auto circ = m_config.GetMechanismCircumference(); circ)
    SetPosition(units::turn_t{distance.value() / circ->value()});
}

void TalonFXWrapper::SetVelocity(units::turns_per_second_t velocity) {
  m_setpointVelocity = velocity;
  if (m_config.GetMotorControllerMode() == ControlMode::CLOSED_LOOP &&
      !m_closedLoopControllerRunning) {
    if (m_config.GetVelocityTrapezoidalProfileInUse())
      m_talon.SetControl(m_trapVelocityReq.WithVelocity(velocity));
    else
      m_talon.SetControl(m_simpleVelocityReq.WithVelocity(velocity));
  }
}

void TalonFXWrapper::SetVelocity(units::meters_per_second_t velocity) {
  if (auto circ = m_config.GetMechanismCircumference(); circ)
    SetVelocity(units::turns_per_second_t{velocity.value() / circ->value()});
}

// ---- Encoder writes ---------------------------------------------------------

void TalonFXWrapper::SetEncoderPosition(units::turn_t angle) { m_talon.SetPosition(angle); }

void TalonFXWrapper::SetEncoderPosition(units::meter_t distance) {
  if (auto circ = m_config.GetMechanismCircumference(); circ) {
    m_talon.SetPosition(units::turn_t{distance.value() / circ->value()});
  }
}

void TalonFXWrapper::SetEncoderVelocity(units::turns_per_second_t) {}

void TalonFXWrapper::SetEncoderVelocity(units::meters_per_second_t velocity) {}

// ---- Encoder reads ----------------------------------------------------------

units::turn_t TalonFXWrapper::GetMechanismPosition() { return m_talon.GetPosition().GetValue(); }

units::turns_per_second_t TalonFXWrapper::GetMechanismVelocity() {
  return m_talon.GetVelocity().GetValue();
}

units::turns_per_second_squared_t TalonFXWrapper::GetMechanismAcceleration() {
  return m_talon.GetAcceleration().GetValue();
}

units::turn_t TalonFXWrapper::GetRotorPosition() { return m_talon.GetRotorPosition().GetValue(); }

units::turns_per_second_t TalonFXWrapper::GetRotorVelocity() {
  return m_talon.GetRotorVelocity().GetValue();
}

units::meter_t TalonFXWrapper::GetMeasurementPosition() {
  auto circ = m_config.GetMechanismCircumference().value_or(1.0_m);
  return units::meter_t{GetMechanismPosition().value() * circ.value()};
}

units::meters_per_second_t TalonFXWrapper::GetMeasurementVelocity() {
  auto circ = m_config.GetMechanismCircumference().value_or(1.0_m);
  return units::meters_per_second_t{GetMechanismVelocity().value() * circ.value()};
}

units::meters_per_second_squared_t TalonFXWrapper::GetMeasurementAcceleration() {
  auto circ = m_config.GetMechanismCircumference().value_or(1.0_m);
  return units::meters_per_second_squared_t{GetMechanismAcceleration().value() * circ.value()};
}

std::optional<units::degree_t> TalonFXWrapper::GetExternalEncoderPosition() {
  if (m_cancoder) return units::degree_t{m_cancoder->get().GetAbsolutePosition().GetValue()};
  return std::nullopt;
}

std::optional<units::degrees_per_second_t> TalonFXWrapper::GetExternalEncoderVelocity() {
  if (m_cancoder) return units::degrees_per_second_t{m_cancoder->get().GetVelocity().GetValue()};
  return std::nullopt;
}

// ---- Motor status -----------------------------------------------------------

std::optional<units::ampere_t> TalonFXWrapper::GetSupplyCurrent() {
  return m_talon.GetSupplyCurrent().GetValue();
}

units::ampere_t TalonFXWrapper::GetStatorCurrent() { return m_talon.GetStatorCurrent().GetValue(); }

units::celsius_t TalonFXWrapper::GetTemperature() {
  return units::celsius_t{m_talon.GetDeviceTemp().GetValue().value()};
}

frc::DCMotor TalonFXWrapper::GetDCMotor() { return m_dcMotor; }

// ---- CANcoder / CANdi -------------------------------------------------------

void TalonFXWrapper::WithCANcoder(hardware::CANcoder& cancoder) {
  m_cancoder = cancoder;
  m_talonConfig.Feedback.FeedbackRemoteSensorID = cancoder.GetDeviceID();
  m_talonConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::FusedCANcoder;
  m_talon.GetConfigurator().Apply(m_talonConfig);
}

void TalonFXWrapper::WithCANdi(hardware::CANdi& candi) { m_candi = candi; }

// ---- Configuration setters --------------------------------------------------

void TalonFXWrapper::SetIdleMode(MotorMode mode) {
  m_talonConfig.MotorOutput.NeutralMode = mode == MotorMode::BRAKE
                                              ? signals::NeutralModeValue::Brake
                                              : signals::NeutralModeValue::Coast;
  m_talon.GetConfigurator().Apply(m_talonConfig.MotorOutput);
}

void TalonFXWrapper::SetMotorInverted(bool inv) {
  m_talonConfig.MotorOutput.Inverted = inv ? signals::InvertedValue::Clockwise_Positive
                                           : signals::InvertedValue::CounterClockwise_Positive;
  m_talon.GetConfigurator().Apply(m_talonConfig.MotorOutput);
}

void TalonFXWrapper::SetEncoderInverted(bool) {}

void TalonFXWrapper::SetKp(double kP) {
  m_talonConfig.Slot0.kP = kP;
  m_talon.GetConfigurator().Apply(m_talonConfig.Slot0);
}
void TalonFXWrapper::SetKi(double kI) {
  m_talonConfig.Slot0.kI = kI;
  m_talon.GetConfigurator().Apply(m_talonConfig.Slot0);
}
void TalonFXWrapper::SetKd(double kD) {
  m_talonConfig.Slot0.kD = kD;
  m_talon.GetConfigurator().Apply(m_talonConfig.Slot0);
}
void TalonFXWrapper::SetFeedback(double kP, double kI, double kD) {
  SetKp(kP);
  SetKi(kI);
  SetKd(kD);
}
void TalonFXWrapper::SetKs(double kS) {
  m_talonConfig.Slot0.kS = kS;
  m_talon.GetConfigurator().Apply(m_talonConfig.Slot0);
}
void TalonFXWrapper::SetKv(double kV) {
  m_talonConfig.Slot0.kV = kV;
  m_talon.GetConfigurator().Apply(m_talonConfig.Slot0);
}
void TalonFXWrapper::SetKa(double kA) {
  m_talonConfig.Slot0.kA = kA;
  m_talon.GetConfigurator().Apply(m_talonConfig.Slot0);
}
void TalonFXWrapper::SetKg(double kG) {
  m_talonConfig.Slot0.kG = kG;
  m_talon.GetConfigurator().Apply(m_talonConfig.Slot0);
}
void TalonFXWrapper::SetFeedforward(double kS, double kV, double kA, double kG) {
  SetKs(kS);
  SetKv(kV);
  SetKa(kA);
  SetKg(kG);
}

void TalonFXWrapper::SetStatorCurrentLimit(units::ampere_t limit) {
  m_talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  m_talonConfig.CurrentLimits.StatorCurrentLimit = limit;
  m_talon.GetConfigurator().Apply(m_talonConfig.CurrentLimits);
}

void TalonFXWrapper::SetSupplyCurrentLimit(units::ampere_t limit) {
  m_talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  m_talonConfig.CurrentLimits.SupplyCurrentLimit = limit;
  m_talon.GetConfigurator().Apply(m_talonConfig.CurrentLimits);
}

void TalonFXWrapper::SetClosedLoopRampRate(units::second_t r) {
  m_talonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = r;
  m_talon.GetConfigurator().Apply(m_talonConfig.ClosedLoopRamps);
}

void TalonFXWrapper::SetOpenLoopRampRate(units::second_t r) {
  m_talonConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = r;
  m_talon.GetConfigurator().Apply(m_talonConfig.OpenLoopRamps);
}

void TalonFXWrapper::SetMechanismUpperLimit(units::turn_t upper) {
  m_talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
  m_talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = upper;
  m_talon.GetConfigurator().Apply(m_talonConfig.SoftwareLimitSwitch);
}

void TalonFXWrapper::SetMechanismLowerLimit(units::turn_t lower) {
  m_talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
  m_talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = lower;
  m_talon.GetConfigurator().Apply(m_talonConfig.SoftwareLimitSwitch);
}

void TalonFXWrapper::SetMechanismLimits(units::turn_t lower, units::turn_t upper) {
  SetMechanismLowerLimit(lower);
  SetMechanismUpperLimit(upper);
}

void TalonFXWrapper::SetMechanismLimitsEnabled(bool en) {
  m_talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = en;
  m_talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = en;
  m_talon.GetConfigurator().Apply(m_talonConfig.SoftwareLimitSwitch);
}

void TalonFXWrapper::SetMeasurementUpperLimit(units::meter_t upper) {
  if (auto circ = m_config.GetMechanismCircumference(); circ)
    SetMechanismUpperLimit(units::turn_t{upper.value() / circ->value()});
}

void TalonFXWrapper::SetMeasurementLowerLimit(units::meter_t lower) {
  if (auto circ = m_config.GetMechanismCircumference(); circ)
    SetMechanismLowerLimit(units::turn_t{lower.value() / circ->value()});
}

void TalonFXWrapper::SetMotionProfileMaxVelocity(units::turns_per_second_t vel) {
  m_talonConfig.MotionMagic.MotionMagicCruiseVelocity = vel;
  m_talon.GetConfigurator().Apply(m_talonConfig.MotionMagic);
}

void TalonFXWrapper::SetMotionProfileMaxVelocity(units::meters_per_second_t vel) {
  if (auto circ = m_config.GetMechanismCircumference(); circ)
    SetMotionProfileMaxVelocity(units::turns_per_second_t{vel.value() / circ->value()});
}

void TalonFXWrapper::SetMotionProfileMaxAcceleration(units::turns_per_second_squared_t acc) {
  m_talonConfig.MotionMagic.MotionMagicAcceleration = acc;
  m_talon.GetConfigurator().Apply(m_talonConfig.MotionMagic);
}

void TalonFXWrapper::SetMotionProfileMaxAcceleration(units::meters_per_second_squared_t acc) {
  if (auto circ = m_config.GetMechanismCircumference(); circ)
    SetMotionProfileMaxAcceleration(units::turns_per_second_squared_t{acc.value() / circ->value()});
}

void TalonFXWrapper::SetMotionProfileMaxJerk(units::angular_jerk::turns_per_second_cubed_t jerk) {
  m_talonConfig.MotionMagic.MotionMagicJerk = jerk;
  m_talon.GetConfigurator().Apply(m_talonConfig.MotionMagic);
}

void TalonFXWrapper::SetExponentialProfile(std::optional<double> kV, std::optional<double> kA,
                                           std::optional<units::volt_t> maxInput) {
  if (kV) m_talonConfig.Slot0.kV = *kV;
  if (kA) m_talonConfig.Slot0.kA = *kA;
  m_talon.GetConfigurator().Apply(m_talonConfig);
}

void TalonFXWrapper::SetClosedLoopSlot(ClosedLoopControllerSlot slot) {
  m_slot = slot;
  int idx = static_cast<int>(slot);
  m_simplePositionReq.WithSlot(idx);
  m_simpleVelocityReq.WithSlot(idx);
  m_trapPositionReq.WithSlot(idx);
  m_trapVelocityReq.WithSlot(idx);
  m_expoPositionReq.WithSlot(idx);
}

SmartMotorControllerConfig& TalonFXWrapper::GetConfig() { return m_config; }
void* TalonFXWrapper::GetMotorController() { return &m_talon; }
void* TalonFXWrapper::GetMotorControllerConfig() { return &m_talonConfig; }

telemetry::UnsupportedTelemetryFields TalonFXWrapper::GetUnsupportedTelemetryFields() {
  return {};  // TalonFX supports all telemetry fields
}

}  // namespace yams::motorcontrollers::remote
