// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/remote/TalonFXSWrapper.hpp"

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

// ---- Motor arrangement → feedback source -----------------------------------

signals::ExternalFeedbackSensorSourceValue TalonFXSWrapper::ArrangementToFeedbackSource() const {
  switch (m_arrangement) {
    case MotorArrangement::Minion:
      return signals::ExternalFeedbackSensorSourceValue::PulseWidth;
    default:
      return signals::ExternalFeedbackSensorSourceValue::Quadrature;
  }
}

// ---- Constructor ------------------------------------------------------------

TalonFXSWrapper::TalonFXSWrapper(hardware::TalonFXS& talon, frc::DCMotor dcMotor,
                                 MotorArrangement arrangement,
                                 const SmartMotorControllerConfig& config)
    : SmartMotorController(), m_talon(talon), m_dcMotor(dcMotor), m_arrangement(arrangement) {
  m_config = config;
  SetupSimulation();
  ApplyConfig(config);
  CheckConfigSafety();
}

// ---- Configuration ----------------------------------------------------------

bool TalonFXSWrapper::ApplyConfig(const SmartMotorControllerConfig& config) {
  m_config = config;
  auto& cfg = m_talonConfig;

  if (auto inv = config.GetMotorInverted(); inv)
    cfg.MotorOutput.Inverted = *inv ? signals::InvertedValue::Clockwise_Positive
                                    : signals::InvertedValue::CounterClockwise_Positive;

  cfg.MotorOutput.NeutralMode = config.GetIdleMode() == SmartMotorControllerConfig::MotorMode::BRAKE
                                    ? signals::NeutralModeValue::Brake
                                    : signals::NeutralModeValue::Coast;

  if (auto& gearing = config.GetMotorGearing(); gearing)
    cfg.ExternalFeedback.SensorToMechanismRatio =
        units::dimensionless::scalar_t{gearing->GetMechanismToRotorRatio()};

  cfg.ExternalFeedback.ExternalFeedbackSensorSource = ArrangementToFeedbackSource();

  auto g0 = config.GetSlotGains(SmartMotorControllerConfig::ClosedLoopControllerSlot::SLOT_0);
  cfg.Slot0.kP = g0.kP;
  cfg.Slot0.kI = g0.kI;
  cfg.Slot0.kD = g0.kD;
  cfg.Slot0.kS = g0.kS;
  cfg.Slot0.kV = g0.kV;
  cfg.Slot0.kA = g0.kA;
  if (g0.armFF) {
    cfg.Slot0.kG = g0.kG;
    cfg.Slot0.GravityType = signals::GravityTypeValue::Arm_Cosine;
  } else if (g0.elevatorFF) {
    cfg.Slot0.kG = g0.kG;
    cfg.Slot0.GravityType = signals::GravityTypeValue::Elevator_Static;
  }

  auto g1 = config.GetSlotGains(SmartMotorControllerConfig::ClosedLoopControllerSlot::SLOT_1);
  cfg.Slot1.kP = g1.kP;
  cfg.Slot1.kI = g1.kI;
  cfg.Slot1.kD = g1.kD;
  cfg.Slot1.kS = g1.kS;
  cfg.Slot1.kV = g1.kV;
  cfg.Slot1.kA = g1.kA;

  auto g2 = config.GetSlotGains(SmartMotorControllerConfig::ClosedLoopControllerSlot::SLOT_2);
  cfg.Slot2.kP = g2.kP;
  cfg.Slot2.kI = g2.kI;
  cfg.Slot2.kD = g2.kD;
  cfg.Slot2.kS = g2.kS;
  cfg.Slot2.kV = g2.kV;
  cfg.Slot2.kA = g2.kA;

  if (auto upper = config.GetMechanismUpperLimit(); upper) {
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = units::turn_t{*upper};
  }
  if (auto lower = config.GetMechanismLowerLimit(); lower) {
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = units::turn_t{*lower};
  }

  if (config.HasTrapezoidProfile() && !config.GetVelocityTrapezoidalProfileInUse()) {
    if (auto v = config.GetTrapMaxVelocityTurns(); v)
      cfg.MotionMagic.MotionMagicCruiseVelocity = *v;
    if (auto a = config.GetTrapMaxAccelTurns(); a) cfg.MotionMagic.MotionMagicAcceleration = *a;
  } else if (config.GetVelocityTrapezoidalProfileInUse()) {
  } else if (config.HasExponentialProfile()) {
  }

  if (auto r = config.GetOpenLoopRampRate(); r) cfg.OpenLoopRamps.VoltageOpenLoopRampPeriod = *r;
  if (auto r = config.GetClosedLoopRampRate(); r)
    cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = *r;

  if (auto stator = config.GetStatorCurrentLimit(); stator) {
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = *stator;
  }
  if (auto supply = config.GetSupplyCurrentLimit(); supply) {
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = *supply;
  }

  return m_talon.GetConfigurator().Apply(cfg).IsOK();
}

// ---- Simulation -------------------------------------------------------------

void TalonFXSWrapper::SetupSimulation() {
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

void TalonFXSWrapper::SimIterate() {
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

void TalonFXSWrapper::SeedRelativeEncoder() {}
void TalonFXSWrapper::SynchronizeRelativeEncoder() {}

// ---- Open-loop outputs ------------------------------------------------------

void TalonFXSWrapper::SetDutyCycle(double dc) { m_talon.SetControl(m_dutyCycleReq.WithOutput(dc)); }

double TalonFXSWrapper::GetDutyCycle() { return m_talon.GetDutyCycle().GetValue(); }

void TalonFXSWrapper::SetVoltage(units::volt_t voltage) {
  m_talon.SetControl(m_voltageReq.WithOutput(voltage));
}

units::volt_t TalonFXSWrapper::GetVoltage() { return m_talon.GetMotorVoltage().GetValue(); }

// ---- Closed-loop setpoints --------------------------------------------------

void TalonFXSWrapper::SetPosition(units::turn_t angle) {
  m_setpointPosition = angle;
  if (m_config.GetMotorControllerMode() == ControlMode::CLOSED_LOOP &&
      !m_closedLoopControllerRunning) {
    if (m_config.HasTrapezoidProfile() && !m_config.GetVelocityTrapezoidalProfileInUse())
      m_talon.SetControl(m_trapPositionReq.WithPosition(angle));
    else if (m_config.HasExponentialProfile())
      m_talon.SetControl(m_expoPositionReq.WithPosition(angle));
    else
      m_talon.SetControl(m_simplePositionReq.WithPosition(angle));
  }
}

void TalonFXSWrapper::SetPosition(units::meter_t distance) {
  if (auto circ = m_config.GetMechanismCircumference(); circ)
    SetPosition(units::turn_t{distance.value() / circ->value()});
}

void TalonFXSWrapper::SetVelocity(units::turns_per_second_t velocity) {
  m_setpointVelocity = velocity;
  if (m_config.GetMotorControllerMode() == ControlMode::CLOSED_LOOP &&
      !m_closedLoopControllerRunning) {
    if (m_config.GetVelocityTrapezoidalProfileInUse())
      m_talon.SetControl(m_trapVelocityReq.WithVelocity(velocity));
    else
      m_talon.SetControl(m_simpleVelocityReq.WithVelocity(velocity));
  }
}

void TalonFXSWrapper::SetVelocity(units::meters_per_second_t velocity) {
  if (auto circ = m_config.GetMechanismCircumference(); circ)
    SetVelocity(units::turns_per_second_t{velocity.value() / circ->value()});
}

// ---- Encoder writes ---------------------------------------------------------

void TalonFXSWrapper::SetEncoderPosition(units::turn_t angle) { m_talon.SetPosition(angle); }

void TalonFXSWrapper::SetEncoderPosition(units::meter_t distance) {
  if (auto circ = m_config.GetMechanismCircumference(); circ)
    m_talon.SetPosition(units::turn_t{distance.value() / circ->value()});
}

void TalonFXSWrapper::SetEncoderVelocity(units::turns_per_second_t) {}
void TalonFXSWrapper::SetEncoderVelocity(units::meters_per_second_t) {}

// ---- Encoder reads ----------------------------------------------------------

units::turn_t TalonFXSWrapper::GetMechanismPosition() { return m_talon.GetPosition().GetValue(); }

units::turns_per_second_t TalonFXSWrapper::GetMechanismVelocity() {
  return m_talon.GetVelocity().GetValue();
}

units::turns_per_second_squared_t TalonFXSWrapper::GetMechanismAcceleration() {
  return m_talon.GetAcceleration().GetValue();
}

units::turn_t TalonFXSWrapper::GetRotorPosition() { return m_talon.GetRotorPosition().GetValue(); }

units::turns_per_second_t TalonFXSWrapper::GetRotorVelocity() {
  return m_talon.GetRotorVelocity().GetValue();
}

units::meter_t TalonFXSWrapper::GetMeasurementPosition() {
  auto circ = m_config.GetMechanismCircumference().value_or(1.0_m);
  return units::meter_t{GetMechanismPosition().value() * circ.value()};
}

units::meters_per_second_t TalonFXSWrapper::GetMeasurementVelocity() {
  auto circ = m_config.GetMechanismCircumference().value_or(1.0_m);
  return units::meters_per_second_t{GetMechanismVelocity().value() * circ.value()};
}

units::meters_per_second_squared_t TalonFXSWrapper::GetMeasurementAcceleration() {
  auto circ = m_config.GetMechanismCircumference().value_or(1.0_m);
  return units::meters_per_second_squared_t{GetMechanismAcceleration().value() * circ.value()};
}

std::optional<units::degree_t> TalonFXSWrapper::GetExternalEncoderPosition() {
  if (m_cancoder) return units::degree_t{m_cancoder->get().GetAbsolutePosition().GetValue()};
  return std::nullopt;
}

std::optional<units::degrees_per_second_t> TalonFXSWrapper::GetExternalEncoderVelocity() {
  if (m_cancoder) return units::degrees_per_second_t{m_cancoder->get().GetVelocity().GetValue()};
  return std::nullopt;
}

// ---- Motor status -----------------------------------------------------------

std::optional<units::ampere_t> TalonFXSWrapper::GetSupplyCurrent() {
  return m_talon.GetSupplyCurrent().GetValue();
}

units::ampere_t TalonFXSWrapper::GetStatorCurrent() {
  return m_talon.GetStatorCurrent().GetValue();
}

units::celsius_t TalonFXSWrapper::GetTemperature() {
  return units::celsius_t{m_talon.GetDeviceTemp().GetValue().value()};
}

frc::DCMotor TalonFXSWrapper::GetDCMotor() { return m_dcMotor; }

// ---- CANcoder / CANdi support -----------------------------------------------

void TalonFXSWrapper::WithCANcoder(hardware::CANcoder& cancoder) {
  m_cancoder = cancoder;
  m_talonConfig.ExternalFeedback.FeedbackRemoteSensorID = cancoder.GetDeviceID();
  m_talonConfig.ExternalFeedback.ExternalFeedbackSensorSource =
      signals::ExternalFeedbackSensorSourceValue::FusedCANcoder;
  m_talon.GetConfigurator().Apply(m_talonConfig);
}

void TalonFXSWrapper::WithCANdi(hardware::CANdi& candi) { m_candi = candi; }

// ---- Live-tuning setters ---------------------------------------------------

void TalonFXSWrapper::SetIdleMode(MotorMode mode) {
  m_talonConfig.MotorOutput.NeutralMode = mode == MotorMode::BRAKE
                                              ? signals::NeutralModeValue::Brake
                                              : signals::NeutralModeValue::Coast;
  m_talon.GetConfigurator().Apply(m_talonConfig.MotorOutput);
}

void TalonFXSWrapper::SetMotorInverted(bool inv) {
  m_talonConfig.MotorOutput.Inverted = inv ? signals::InvertedValue::Clockwise_Positive
                                           : signals::InvertedValue::CounterClockwise_Positive;
  m_talon.GetConfigurator().Apply(m_talonConfig.MotorOutput);
}

void TalonFXSWrapper::SetEncoderInverted(bool) {}

void TalonFXSWrapper::SetKp(double kP) {
  m_talonConfig.Slot0.kP = kP;
  m_talon.GetConfigurator().Apply(m_talonConfig.Slot0);
}
void TalonFXSWrapper::SetKi(double kI) {
  m_talonConfig.Slot0.kI = kI;
  m_talon.GetConfigurator().Apply(m_talonConfig.Slot0);
}
void TalonFXSWrapper::SetKd(double kD) {
  m_talonConfig.Slot0.kD = kD;
  m_talon.GetConfigurator().Apply(m_talonConfig.Slot0);
}
void TalonFXSWrapper::SetFeedback(double kP, double kI, double kD) {
  SetKp(kP);
  SetKi(kI);
  SetKd(kD);
}
void TalonFXSWrapper::SetKs(double kS) {
  m_talonConfig.Slot0.kS = kS;
  m_talon.GetConfigurator().Apply(m_talonConfig.Slot0);
}
void TalonFXSWrapper::SetKv(double kV) {
  m_talonConfig.Slot0.kV = kV;
  m_talon.GetConfigurator().Apply(m_talonConfig.Slot0);
}
void TalonFXSWrapper::SetKa(double kA) {
  m_talonConfig.Slot0.kA = kA;
  m_talon.GetConfigurator().Apply(m_talonConfig.Slot0);
}
void TalonFXSWrapper::SetKg(double kG) {
  m_talonConfig.Slot0.kG = kG;
  m_talon.GetConfigurator().Apply(m_talonConfig.Slot0);
}
void TalonFXSWrapper::SetFeedforward(double kS, double kV, double kA, double kG) {
  SetKs(kS);
  SetKv(kV);
  SetKa(kA);
  SetKg(kG);
}

void TalonFXSWrapper::SetStatorCurrentLimit(units::ampere_t limit) {
  m_talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  m_talonConfig.CurrentLimits.StatorCurrentLimit = limit;
  m_talon.GetConfigurator().Apply(m_talonConfig.CurrentLimits);
}

void TalonFXSWrapper::SetSupplyCurrentLimit(units::ampere_t limit) {
  m_talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  m_talonConfig.CurrentLimits.SupplyCurrentLimit = limit;
  m_talon.GetConfigurator().Apply(m_talonConfig.CurrentLimits);
}

void TalonFXSWrapper::SetClosedLoopRampRate(units::second_t r) {
  m_talonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = r;
  m_talon.GetConfigurator().Apply(m_talonConfig.ClosedLoopRamps);
}

void TalonFXSWrapper::SetOpenLoopRampRate(units::second_t r) {
  m_talonConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = r;
  m_talon.GetConfigurator().Apply(m_talonConfig.OpenLoopRamps);
}

void TalonFXSWrapper::SetMechanismUpperLimit(units::turn_t upper) {
  m_talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
  m_talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = upper;
  m_talon.GetConfigurator().Apply(m_talonConfig.SoftwareLimitSwitch);
}

void TalonFXSWrapper::SetMechanismLowerLimit(units::turn_t lower) {
  m_talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
  m_talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = lower;
  m_talon.GetConfigurator().Apply(m_talonConfig.SoftwareLimitSwitch);
}

void TalonFXSWrapper::SetMechanismLimits(units::turn_t lower, units::turn_t upper) {
  SetMechanismLowerLimit(lower);
  SetMechanismUpperLimit(upper);
}

void TalonFXSWrapper::SetMechanismLimitsEnabled(bool en) {
  m_talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = en;
  m_talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = en;
  m_talon.GetConfigurator().Apply(m_talonConfig.SoftwareLimitSwitch);
}

void TalonFXSWrapper::SetMeasurementUpperLimit(units::meter_t upper) {
  if (auto circ = m_config.GetMechanismCircumference(); circ)
    SetMechanismUpperLimit(units::turn_t{upper.value() / circ->value()});
}

void TalonFXSWrapper::SetMeasurementLowerLimit(units::meter_t lower) {
  if (auto circ = m_config.GetMechanismCircumference(); circ)
    SetMechanismLowerLimit(units::turn_t{lower.value() / circ->value()});
}

void TalonFXSWrapper::SetMotionProfileMaxVelocity(units::turns_per_second_t vel) {
  m_talonConfig.MotionMagic.MotionMagicCruiseVelocity = vel;
  m_talon.GetConfigurator().Apply(m_talonConfig.MotionMagic);
}

void TalonFXSWrapper::SetMotionProfileMaxVelocity(units::meters_per_second_t vel) {
  if (auto circ = m_config.GetMechanismCircumference(); circ)
    SetMotionProfileMaxVelocity(units::turns_per_second_t{vel.value() / circ->value()});
}

void TalonFXSWrapper::SetMotionProfileMaxAcceleration(units::turns_per_second_squared_t acc) {
  m_talonConfig.MotionMagic.MotionMagicAcceleration = acc;
  m_talon.GetConfigurator().Apply(m_talonConfig.MotionMagic);
}

void TalonFXSWrapper::SetMotionProfileMaxAcceleration(units::meters_per_second_squared_t acc) {
  if (auto circ = m_config.GetMechanismCircumference(); circ)
    SetMotionProfileMaxAcceleration(units::turns_per_second_squared_t{acc.value() / circ->value()});
}

void TalonFXSWrapper::SetMotionProfileMaxJerk(units::angular_jerk::turns_per_second_cubed_t jerk) {
  m_talonConfig.MotionMagic.MotionMagicJerk = jerk;
  m_talon.GetConfigurator().Apply(m_talonConfig.MotionMagic);
}

void TalonFXSWrapper::SetExponentialProfile(std::optional<double> kV, std::optional<double> kA,
                                            std::optional<units::volt_t> maxInput) {
  if (kV) m_talonConfig.Slot0.kV = *kV;
  if (kA) m_talonConfig.Slot0.kA = *kA;
  m_talon.GetConfigurator().Apply(m_talonConfig);
}

void TalonFXSWrapper::SetClosedLoopSlot(ClosedLoopControllerSlot slot) {
  m_slot = slot;
  int idx = static_cast<int>(slot);
  m_simplePositionReq.WithSlot(idx);
  m_simpleVelocityReq.WithSlot(idx);
  m_trapPositionReq.WithSlot(idx);
  m_trapVelocityReq.WithSlot(idx);
  m_expoPositionReq.WithSlot(idx);
}

SmartMotorControllerConfig& TalonFXSWrapper::GetConfig() { return m_config; }
void* TalonFXSWrapper::GetMotorController() { return &m_talon; }
void* TalonFXSWrapper::GetMotorControllerConfig() { return &m_talonConfig; }

telemetry::UnsupportedTelemetryFields TalonFXSWrapper::GetUnsupportedTelemetryFields() {
  return {};  // TalonFXS supports all telemetry fields
}

}  // namespace yams::motorcontrollers::remote
