// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/SmartMotorControllerConfig.h"

#include <cmath>
#include <stdexcept>
#include <string>

namespace yams::motorcontrollers {

int SmartMotorControllerConfig::SlotIndex(ClosedLoopControllerSlot slot) const {
  return static_cast<int>(slot);
}

// ---- Feedback -----------------------------------------------------------

SmartMotorControllerConfig& SmartMotorControllerConfig::WithFeedback(
    double kP, double kI, double kD, ClosedLoopControllerSlot slot) {
  auto& s = m_slots[SlotIndex(slot)];
  s.kP = kP;
  s.kI = kI;
  s.kD = kD;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithKp(double kP,
                                                               ClosedLoopControllerSlot slot) {
  m_slots[SlotIndex(slot)].kP = kP;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithKi(double kI,
                                                               ClosedLoopControllerSlot slot) {
  m_slots[SlotIndex(slot)].kI = kI;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithKd(double kD,
                                                               ClosedLoopControllerSlot slot) {
  m_slots[SlotIndex(slot)].kD = kD;
  return *this;
}

// ---- Feedforward ---------------------------------------------------------

SmartMotorControllerConfig& SmartMotorControllerConfig::WithArmFeedforward(
    double kS, double kV, double kA, double kG, ClosedLoopControllerSlot slot) {
  auto& s = m_slots[SlotIndex(slot)];
  s.kS = kS;
  s.kV = kV;
  s.kA = kA;
  s.kG = kG;
  s.armFF = frc::ArmFeedforward{units::volt_t{kS}, units::volt_t{kG},
                                units::unit_t<frc::ArmFeedforward::kv_unit>{kV},
                                units::unit_t<frc::ArmFeedforward::ka_unit>{kA}};
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithElevatorFeedforward(
    double kS, double kV, double kA, ClosedLoopControllerSlot slot) {
  auto& s = m_slots[SlotIndex(slot)];
  s.kS = kS;
  s.kV = kV;
  s.kA = kA;
  s.elevatorFF = frc::ElevatorFeedforward{units::volt_t{kS}, units::volt_t{0.0},
                                          units::unit_t<frc::ElevatorFeedforward::kv_unit>{kV},
                                          units::unit_t<frc::ElevatorFeedforward::ka_unit>{kA}};
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithSimpleFeedforward(
    double kS, double kV, double kA, ClosedLoopControllerSlot slot) {
  auto& s = m_slots[SlotIndex(slot)];
  s.kS = kS;
  s.kV = kV;
  s.kA = kA;
  s.simpleFF = frc::SimpleMotorFeedforward<units::turns>{
      units::volt_t{kS}, units::unit_t<frc::SimpleMotorFeedforward<units::turns>::kv_unit>{kV},
      units::unit_t<frc::SimpleMotorFeedforward<units::turns>::ka_unit>{kA}};
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithKs(double kS,
                                                               ClosedLoopControllerSlot slot) {
  m_slots[SlotIndex(slot)].kS = kS;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithKv(double kV,
                                                               ClosedLoopControllerSlot slot) {
  m_slots[SlotIndex(slot)].kV = kV;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithKa(double kA,
                                                               ClosedLoopControllerSlot slot) {
  m_slots[SlotIndex(slot)].kA = kA;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithKg(double kG,
                                                               ClosedLoopControllerSlot slot) {
  m_slots[SlotIndex(slot)].kG = kG;
  return *this;
}

// ---- Motion Profiles -----------------------------------------------------

SmartMotorControllerConfig& SmartMotorControllerConfig::WithTrapezoidProfile(
    units::turns_per_second_t maxVelocity, units::turns_per_second_squared_t maxAcceleration) {
  m_trapProfile = frc::TrapezoidProfile<units::turns>{{maxVelocity, maxAcceleration}};
  m_trapMaxVelTurns = maxVelocity;
  m_trapMaxAccTurns = maxAcceleration;
  m_velocityTrapProfile = false;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithLinearTrapezoidProfile(
    units::meters_per_second_t maxVelocity, units::meters_per_second_squared_t maxAcceleration) {
  m_linearTrapProfile = frc::TrapezoidProfile<units::meters>{{maxVelocity, maxAcceleration}};
  m_trapMaxVelLinear = maxVelocity;
  m_trapMaxAccLinear = maxAcceleration;
  m_velocityTrapProfile = false;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithVelocityTrapezoidProfile(
    units::turns_per_second_t maxVelocity, units::turns_per_second_squared_t maxAcceleration) {
  m_trapProfile = frc::TrapezoidProfile<units::turns>{{maxVelocity, maxAcceleration}};
  m_trapMaxVelTurns = maxVelocity;
  m_trapMaxAccTurns = maxAcceleration;
  m_velocityTrapProfile = true;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithExponentialProfile(
    double kV, double kA, units::volt_t maxInput) {
  using Profile = frc::ExponentialProfile<units::turns, units::volts>;
  m_expoProfile = Profile{Profile::Constraints{maxInput, Profile::kV_t{kV}, Profile::kA_t{kA}}};
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithLQR(const math::LQRConfig& lqrConfig,
                                                                ClosedLoopControllerSlot slot) {
  m_slots[SlotIndex(slot)].lqr = lqrConfig;
  return *this;
}

// ---- Gearing / linear ----------------------------------------------------

SmartMotorControllerConfig& SmartMotorControllerConfig::WithMotorGearing(
    const gearing::MechanismGearing& gearing) {
  m_motorGearing = gearing;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithMechanismCircumference(
    units::meter_t circumference) {
  m_mechanismCircumference = circumference;
  return *this;
}

// ---- Limits --------------------------------------------------------------

SmartMotorControllerConfig& SmartMotorControllerConfig::WithMechanismLimits(units::degree_t lower,
                                                                            units::degree_t upper) {
  m_mechLowerLimit = lower;
  m_mechUpperLimit = upper;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithMeasurementLimits(
    units::meter_t lower, units::meter_t upper) {
  m_measLowerLimit = lower;
  m_measUpperLimit = upper;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithStatorCurrentLimit(
    units::ampere_t limit) {
  m_statorCurrentLimit = limit;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithSupplyCurrentLimit(
    units::ampere_t limit) {
  m_supplyCurrentLimit = limit;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithTemperatureCutoff(
    units::celsius_t temp) {
  m_temperatureCutoff = temp;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithClosedLoopMaxVoltage(
    units::volt_t maxV) {
  m_closedLoopMaxVoltage = maxV;
  return *this;
}

// ---- Control behaviour ---------------------------------------------------

SmartMotorControllerConfig& SmartMotorControllerConfig::WithIdleMode(MotorMode mode) {
  m_idleMode = mode;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithClosedLoopMode() {
  m_controlMode = ControlMode::CLOSED_LOOP;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithOpenLoopMode() {
  m_controlMode = ControlMode::OPEN_LOOP;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithClosedLoopControlPeriod(
    units::second_t p) {
  m_closedLoopPeriod = p;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithOpenLoopRampRate(units::second_t r) {
  m_openLoopRampRate = r;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithClosedLoopRampRate(units::second_t r) {
  m_closedLoopRampRate = r;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithMotorInverted(bool inv) {
  m_motorInverted = inv;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithEncoderInverted(bool inv) {
  m_encoderInverted = inv;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithAbsoluteEncoderConversionFactor(
    double f) {
  m_absEncoderConversionFactor = f;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithAbsoluteEncoderOffset(
    units::degree_t o) {
  m_absEncoderOffset = o;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithAbsoluteEncoderZeroOffset(
    units::degree_t o) {
  m_absEncoderZeroOffset = o;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithTelemetry(
    const std::string& name, TelemetryVerbosity verbosity) {
  m_telemetryName = name;
  m_verbosity = verbosity;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithSubsystem(frc2::SubsystemBase* sys) {
  m_subsystem = sys;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithSimMotor(frc::DCMotor motor) {
  m_simMotor = motor;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithMOI(
    units::kilogram_square_meter_t moi) {
  m_moi = moi;
  return *this;
}

// ---- Getters -------------------------------------------------------------

const SmartMotorControllerConfig::PIDGains& SmartMotorControllerConfig::GetSlotGains(
    ClosedLoopControllerSlot slot) const {
  return m_slots[SlotIndex(slot)];
}

std::optional<frc::ArmFeedforward> SmartMotorControllerConfig::GetArmFeedforward(
    ClosedLoopControllerSlot slot) const {
  return m_slots[SlotIndex(slot)].armFF;
}
std::optional<frc::ElevatorFeedforward> SmartMotorControllerConfig::GetElevatorFeedforward(
    ClosedLoopControllerSlot slot) const {
  return m_slots[SlotIndex(slot)].elevatorFF;
}
std::optional<frc::SimpleMotorFeedforward<units::turns>>
SmartMotorControllerConfig::GetSimpleFeedforward(ClosedLoopControllerSlot slot) const {
  return m_slots[SlotIndex(slot)].simpleFF;
}
std::optional<math::LQRConfig> SmartMotorControllerConfig::GetLQR(
    ClosedLoopControllerSlot slot) const {
  return m_slots[SlotIndex(slot)].lqr;
}

double SmartMotorControllerConfig::GetKp(ClosedLoopControllerSlot slot) const {
  return m_slots[SlotIndex(slot)].kP;
}
double SmartMotorControllerConfig::GetKi(ClosedLoopControllerSlot slot) const {
  return m_slots[SlotIndex(slot)].kI;
}
double SmartMotorControllerConfig::GetKd(ClosedLoopControllerSlot slot) const {
  return m_slots[SlotIndex(slot)].kD;
}

bool SmartMotorControllerConfig::GetLinearClosedLoopControllerUse() const {
  return m_mechanismCircumference.has_value();
}

std::optional<units::degree_t> SmartMotorControllerConfig::GetMechanismLowerLimit() const {
  return m_mechLowerLimit;
}
std::optional<units::degree_t> SmartMotorControllerConfig::GetMechanismUpperLimit() const {
  return m_mechUpperLimit;
}
std::optional<units::meter_t> SmartMotorControllerConfig::GetMeasurementLowerLimit() const {
  return m_measLowerLimit;
}
std::optional<units::meter_t> SmartMotorControllerConfig::GetMeasurementUpperLimit() const {
  return m_measUpperLimit;
}

std::optional<units::ampere_t> SmartMotorControllerConfig::GetStatorCurrentLimit() const {
  return m_statorCurrentLimit;
}
std::optional<int> SmartMotorControllerConfig::GetStatorStallCurrentLimit() const {
  if (!m_statorCurrentLimit) return std::nullopt;
  return static_cast<int>(m_statorCurrentLimit->value());
}
std::optional<int> SmartMotorControllerConfig::GetSupplyStallCurrentLimit() const {
  if (!m_supplyCurrentLimit) return std::nullopt;
  return static_cast<int>(m_supplyCurrentLimit->value());
}
std::optional<units::ampere_t> SmartMotorControllerConfig::GetSupplyCurrentLimit() const {
  return m_supplyCurrentLimit;
}
std::optional<units::celsius_t> SmartMotorControllerConfig::GetTemperatureCutoff() const {
  return m_temperatureCutoff;
}
std::optional<units::volt_t> SmartMotorControllerConfig::GetClosedLoopControllerMaximumVoltage()
    const {
  return m_closedLoopMaxVoltage;
}

SmartMotorControllerConfig::ControlMode SmartMotorControllerConfig::GetMotorControllerMode() const {
  return m_controlMode;
}
SmartMotorControllerConfig::MotorMode SmartMotorControllerConfig::GetIdleMode() const {
  return m_idleMode;
}
std::optional<units::second_t> SmartMotorControllerConfig::GetClosedLoopControlPeriod() const {
  return m_closedLoopPeriod;
}
std::optional<units::second_t> SmartMotorControllerConfig::GetOpenLoopRampRate() const {
  return m_openLoopRampRate;
}
std::optional<units::second_t> SmartMotorControllerConfig::GetClosedLoopRampRate() const {
  return m_closedLoopRampRate;
}

std::optional<bool> SmartMotorControllerConfig::GetMotorInverted() const {
  return m_motorInverted;
}
std::optional<bool> SmartMotorControllerConfig::GetEncoderInverted() const {
  return m_encoderInverted;
}
bool SmartMotorControllerConfig::GetVelocityTrapezoidalProfileInUse() const {
  return m_velocityTrapProfile;
}

std::optional<std::string> SmartMotorControllerConfig::GetTelemetryName() const {
  return m_telemetryName;
}
std::optional<SmartMotorControllerConfig::TelemetryVerbosity>
SmartMotorControllerConfig::GetVerbosity() const {
  return m_verbosity;
}
frc2::SubsystemBase* SmartMotorControllerConfig::GetSubsystem() const { return m_subsystem; }
std::optional<frc::DCMotor> SmartMotorControllerConfig::GetSimMotor() const { return m_simMotor; }
units::kilogram_square_meter_t SmartMotorControllerConfig::GetMOI() const { return m_moi; }

const std::optional<gearing::MechanismGearing>& SmartMotorControllerConfig::GetMotorGearing()
    const {
  return m_motorGearing;
}
std::optional<units::meter_t> SmartMotorControllerConfig::GetMechanismCircumference() const {
  return m_mechanismCircumference;
}

std::optional<double> SmartMotorControllerConfig::GetAbsoluteEncoderConversionFactor() const {
  return m_absEncoderConversionFactor;
}
std::optional<units::degree_t> SmartMotorControllerConfig::GetAbsoluteEncoderOffset() const {
  return m_absEncoderOffset;
}
std::optional<units::degree_t> SmartMotorControllerConfig::GetAbsoluteEncoderZeroOffset() const {
  return m_absEncoderZeroOffset;
}

bool SmartMotorControllerConfig::HasTrapezoidProfile() const {
  return m_trapProfile.has_value() || m_linearTrapProfile.has_value();
}
bool SmartMotorControllerConfig::HasExponentialProfile() const { return m_expoProfile.has_value(); }

std::optional<frc::TrapezoidProfile<units::turns>> SmartMotorControllerConfig::GetTrapezoidProfile()
    const {
  return m_trapProfile;
}
std::optional<frc::TrapezoidProfile<units::meters>>
SmartMotorControllerConfig::GetLinearTrapezoidProfile() const {
  return m_linearTrapProfile;
}
std::optional<frc::ExponentialProfile<units::turns, units::volts>>
SmartMotorControllerConfig::GetExponentialProfile() const {
  return m_expoProfile;
}

std::optional<units::turns_per_second_t> SmartMotorControllerConfig::GetTrapMaxVelocityTurns()
    const {
  return m_trapMaxVelTurns;
}
std::optional<units::turns_per_second_squared_t> SmartMotorControllerConfig::GetTrapMaxAccelTurns()
    const {
  return m_trapMaxAccTurns;
}
std::optional<units::meters_per_second_t> SmartMotorControllerConfig::GetTrapMaxVelocityLinear()
    const {
  return m_trapMaxVelLinear;
}
std::optional<units::meters_per_second_squared_t>
SmartMotorControllerConfig::GetTrapMaxAccelLinear() const {
  return m_trapMaxAccLinear;
}

units::meter_t SmartMotorControllerConfig::ConvertFromMechanism(
    units::degree_t mechanismAngle) const {
  units::turn_t turns = mechanismAngle;  // implicit conversion deg → turns
  double circ = m_mechanismCircumference.value_or(units::meter_t{1.0}).value();
  return units::meter_t{turns.value() * circ};
}

units::meters_per_second_t SmartMotorControllerConfig::ConvertFromMechanism(
    units::degrees_per_second_t mechanismVelocity) const {
  units::turns_per_second_t rps = mechanismVelocity;  // implicit conversion
  double circ = m_mechanismCircumference.value_or(units::meter_t{1.0}).value();
  return units::meters_per_second_t{rps.value() * circ};
}

}  // namespace yams::motorcontrollers
