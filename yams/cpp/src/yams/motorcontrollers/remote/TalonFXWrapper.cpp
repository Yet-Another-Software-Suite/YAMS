// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/remote/TalonFXWrapper.h"

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include <units/angular_jerk.h>
#include <units/dimensionless.h>
#include <units/moment_of_inertia.h>

#include <cmath>

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

// ---- Helpers ----------------------------------------------------------------

units::turns_per_second_t TalonFXWrapper::ToTPS(units::degrees_per_second_t v) const {
  return v;  // implicit deg/s → turns/s
}

units::turn_t TalonFXWrapper::ToTurns(units::degree_t a) const {
  return a;  // implicit deg → turns
}

// ---- Configuration ----------------------------------------------------------

bool TalonFXWrapper::ApplyConfig(const SmartMotorControllerConfig& config) {
  m_config = config;
  auto& cfg = m_talonConfig;

  // Inversion
  cfg.MotorOutput.Inverted = config.GetMotorInverted()
                                 ? signals::InvertedValue::Clockwise_Positive
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
    m_talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ToTurns(*upper);
  }
  if (auto lower = m_config.GetMechanismLowerLimit(); lower) {
    m_talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = limitEnabled;
    m_talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ToTurns(*lower);
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
  if (!frc::RobotBase::IsSimulation()) return;
  if (auto simMotor = m_config.GetSimMotor(); simMotor) {
    auto plant =
        frc::LinearSystemId::DCMotorSystem(*simMotor, units::kilogram_square_meter_t{0.001}, 1.0);
    m_motorSim.emplace(plant, *simMotor);
  }
}

void TalonFXWrapper::SimIterate() {
  if (!m_motorSim) return;
  auto& sim = m_talon.GetSimState();
  m_motorSim->SetInputVoltage(sim.GetMotorVoltage());
  m_motorSim->Update(20_ms);
  sim.SetRawRotorPosition(units::turn_t{m_motorSim->GetAngularPosition()});  // radian → turn
  sim.SetRotorVelocity(
      units::turns_per_second_t{m_motorSim->GetAngularVelocity()});  // rad/s → turn/s
  sim.SetSupplyVoltage(frc::sim::RoboRioSim::GetVInVoltage());
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

void TalonFXWrapper::SetPosition(units::degree_t angle) {
  m_setpointPosition = angle;
  if (m_config.GetMotorControllerMode() == ControlMode::CLOSED_LOOP &&
      !m_closedLoopControllerRunning) {
    bool hasTrap = m_config.HasTrapezoidProfile() && !m_config.GetVelocityTrapezoidalProfileInUse();
    bool hasExpo = m_config.HasExponentialProfile();
    if (hasTrap) {
      m_talon.SetControl(m_trapPositionReq.WithPosition(ToTurns(angle)));
    } else if (hasExpo) {
      m_talon.SetControl(m_expoPositionReq.WithPosition(ToTurns(angle)));
    } else {
      m_talon.SetControl(m_simplePositionReq.WithPosition(ToTurns(angle)));
    }
  }
}

void TalonFXWrapper::SetPosition(units::meter_t distance) {
  if (auto circ = m_config.GetMechanismCircumference(); circ) {
    units::turn_t turns{distance.value() / circ->value()};
    SetPosition(units::degree_t{turns});  // implicit turn → degree
  }
}

void TalonFXWrapper::SetVelocity(units::degrees_per_second_t velocity) {
  m_setpointVelocity = velocity;
  if (m_config.GetMotorControllerMode() == ControlMode::CLOSED_LOOP &&
      !m_closedLoopControllerRunning) {
    bool velTrap = m_config.GetVelocityTrapezoidalProfileInUse();
    if (velTrap) {
      m_talon.SetControl(m_trapVelocityReq.WithVelocity(ToTPS(velocity)));
    } else {
      m_talon.SetControl(m_simpleVelocityReq.WithVelocity(ToTPS(velocity)));
    }
  }
}

void TalonFXWrapper::SetVelocity(units::meters_per_second_t velocity) {
  if (auto circ = m_config.GetMechanismCircumference(); circ) {
    units::turns_per_second_t tps{velocity.value() / circ->value()};
    SetVelocity(units::degrees_per_second_t{tps});  // implicit turns/s → deg/s
  }
}

// ---- Encoder writes ---------------------------------------------------------

void TalonFXWrapper::SetEncoderPosition(units::degree_t angle) {
  m_talon.SetPosition(ToTurns(angle));
}

void TalonFXWrapper::SetEncoderPosition(units::meter_t distance) {
  if (auto circ = m_config.GetMechanismCircumference(); circ) {
    m_talon.SetPosition(units::turn_t{distance.value() / circ->value()});
  }
}

void TalonFXWrapper::SetEncoderVelocity(units::degrees_per_second_t velocity) {
  // Not directly settable on TalonFX outside simulation
}

void TalonFXWrapper::SetEncoderVelocity(units::meters_per_second_t velocity) {}

// ---- Encoder reads ----------------------------------------------------------

units::degree_t TalonFXWrapper::GetMechanismPosition() {
  return m_talon.GetPosition().GetValue();  // implicit turn_t → degree_t
}

units::degrees_per_second_t TalonFXWrapper::GetMechanismVelocity() {
  return m_talon.GetVelocity().GetValue();  // implicit turns/s → deg/s
}

units::degrees_per_second_squared_t TalonFXWrapper::GetMechanismAcceleration() {
  return m_talon.GetAcceleration().GetValue();  // implicit turns/s² → deg/s²
}

units::degree_t TalonFXWrapper::GetRotorPosition() { return m_talon.GetRotorPosition().GetValue(); }

units::degrees_per_second_t TalonFXWrapper::GetRotorVelocity() {
  return m_talon.GetRotorVelocity().GetValue();
}

units::meter_t TalonFXWrapper::GetMeasurementPosition() {
  auto circ = m_config.GetMechanismCircumference().value_or(1.0_m);
  return units::meter_t{GetMechanismPosition().value() / 360.0 * circ.value()};
}

units::meters_per_second_t TalonFXWrapper::GetMeasurementVelocity() {
  auto circ = m_config.GetMechanismCircumference().value_or(1.0_m);
  return units::meters_per_second_t{GetMechanismVelocity().value() / 360.0 * circ.value()};
}

units::meters_per_second_squared_t TalonFXWrapper::GetMeasurementAcceleration() {
  auto circ = m_config.GetMechanismCircumference().value_or(1.0_m);
  return units::meters_per_second_squared_t{GetMechanismAcceleration().value() / 360.0 *
                                            circ.value()};
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

void TalonFXWrapper::SetMechanismUpperLimit(units::degree_t upper) {
  m_talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
  m_talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ToTurns(upper);
  m_talon.GetConfigurator().Apply(m_talonConfig.SoftwareLimitSwitch);
}

void TalonFXWrapper::SetMechanismLowerLimit(units::degree_t lower) {
  m_talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
  m_talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ToTurns(lower);
  m_talon.GetConfigurator().Apply(m_talonConfig.SoftwareLimitSwitch);
}

void TalonFXWrapper::SetMechanismLimits(units::degree_t lower, units::degree_t upper) {
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
    SetMechanismUpperLimit(units::degree_t{upper.value() / circ->value() * 360.0});
}

void TalonFXWrapper::SetMeasurementLowerLimit(units::meter_t lower) {
  if (auto circ = m_config.GetMechanismCircumference(); circ)
    SetMechanismLowerLimit(units::degree_t{lower.value() / circ->value() * 360.0});
}

void TalonFXWrapper::SetMotionProfileMaxVelocity(units::degrees_per_second_t vel) {
  m_talonConfig.MotionMagic.MotionMagicCruiseVelocity = ToTPS(vel);
  m_talon.GetConfigurator().Apply(m_talonConfig.MotionMagic);
}

void TalonFXWrapper::SetMotionProfileMaxVelocity(units::meters_per_second_t vel) {
  if (auto circ = m_config.GetMechanismCircumference(); circ)
    SetMotionProfileMaxVelocity(units::degrees_per_second_t{vel.value() / circ->value() * 360.0});
}

void TalonFXWrapper::SetMotionProfileMaxAcceleration(units::degrees_per_second_squared_t acc) {
  units::turns_per_second_squared_t tps2 = acc;  // implicit deg/s² → turns/s²
  m_talonConfig.MotionMagic.MotionMagicAcceleration = tps2;
  m_talon.GetConfigurator().Apply(m_talonConfig.MotionMagic);
}

void TalonFXWrapper::SetMotionProfileMaxAcceleration(units::meters_per_second_squared_t acc) {
  if (auto circ = m_config.GetMechanismCircumference(); circ)
    SetMotionProfileMaxAcceleration(
        units::degrees_per_second_squared_t{acc.value() / circ->value() * 360.0});
}

void TalonFXWrapper::SetMotionProfileMaxJerk(
    units::unit_t<units::compound_unit<units::angular_acceleration::degrees_per_second_squared,
                                       units::inverse<units::seconds>>>
        jerk) {
  m_talonConfig.MotionMagic.MotionMagicJerk =
      units::angular_jerk::turns_per_second_cubed_t{jerk.value() / 360.0};
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

}  // namespace yams::motorcontrollers::remote
