// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/local/SparkWrapper.h"

#include <frc/RobotBase.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include <rev/ClosedLoopTypes.h>
#include <rev/ConfigureTypes.h>
#include <units/moment_of_inertia.h>

using namespace rev::spark;

namespace yams::motorcontrollers::local {

SparkWrapper::SparkWrapper(SparkMax& spark, frc::DCMotor motor,
                           const SmartMotorControllerConfig& cfg)
    : m_motor(motor) {
  m_maxConfig.emplace();  // default-construct SparkMaxConfig in-place
  Init(&spark, motor, cfg);
}

SparkWrapper::SparkWrapper(SparkFlex& spark, frc::DCMotor motor,
                           const SmartMotorControllerConfig& cfg)
    : SmartMotorController(), m_motor(motor) {
  m_flexConfig.emplace();  // default-construct SparkFlexConfig in-place
  Init(&spark, motor, cfg);
}

void SparkWrapper::Init(SparkBase* spark, frc::DCMotor motor,
                        const SmartMotorControllerConfig& cfg) {
  m_spark = spark;
  m_sparkPid = &spark->GetClosedLoopController();
  m_relEncoder = &spark->GetEncoder();
  m_config = cfg;

  SetupSimulation();
  ApplyConfig(cfg);
  CheckConfigSafety();
}

// ---- Configuration ----------------------------------------------------------

bool SparkWrapper::ApplyConfig(const SmartMotorControllerConfig& cfg) {
  m_config = cfg;

  auto doConfig = [&](SparkBaseConfig& sparkCfg) {
    sparkCfg.Inverted(cfg.GetMotorInverted());
    sparkCfg.SetIdleMode(cfg.GetIdleMode() == SmartMotorControllerConfig::MotorMode::BRAKE
                             ? SparkBaseConfig::IdleMode::kBrake
                             : SparkBaseConfig::IdleMode::kCoast);

    if (auto r = cfg.GetOpenLoopRampRate(); r) sparkCfg.OpenLoopRampRate(r->value());
    if (auto r = cfg.GetClosedLoopRampRate(); r) sparkCfg.ClosedLoopRampRate(r->value());

    if (auto stator = cfg.GetStatorCurrentLimit(); stator)
      sparkCfg.SmartCurrentLimit(static_cast<int>(stator->value()));
    if (auto supply = cfg.GetSupplyCurrentLimit(); supply)
      sparkCfg.SecondaryCurrentLimit(supply->value());

    double convFactor = 1.0;
    if (auto& g = cfg.GetMotorGearing(); g) convFactor = g->GetRotorToMechanismRatio();
    if (auto cf = cfg.GetAbsoluteEncoderConversionFactor(); cf) convFactor = *cf;
    sparkCfg.encoder.PositionConversionFactor(convFactor * 360.0);
    sparkCfg.encoder.VelocityConversionFactor(convFactor * 360.0 / 60.0);

    auto gains = cfg.GetSlotGains(SmartMotorControllerConfig::ClosedLoopControllerSlot::SLOT_0);
    // Use non-deprecated separate P/I/D methods + feedForward for kV
    sparkCfg.closedLoop.Pid(gains.kP, gains.kI, gains.kD);
    sparkCfg.closedLoop.feedForward.kV(gains.kV);
    sparkCfg.closedLoop.IZone(0.0);

    if (cfg.HasTrapezoidProfile()) {
      if (auto v = cfg.GetTrapMaxVelocityTurns(); v)
        sparkCfg.closedLoop.maxMotion.CruiseVelocity(v->value() * 360.0);
      if (auto a = cfg.GetTrapMaxAccelTurns(); a)
        sparkCfg.closedLoop.maxMotion.MaxAcceleration(a->value() * 360.0);
      m_positionControlType = cfg.GetVelocityTrapezoidalProfileInUse()
                                  ? SparkLowLevel::ControlType::kVelocity
                                  : SparkLowLevel::ControlType::kMAXMotionPositionControl;
      m_velocityControlType = SparkLowLevel::ControlType::kMAXMotionVelocityControl;
    }

    sparkCfg.encoder.Inverted(cfg.GetEncoderInverted());
    if (auto offset = cfg.GetAbsoluteEncoderZeroOffset(); offset)
      sparkCfg.absoluteEncoder.ZeroOffset(offset->value() / 360.0);
  };

  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
  return true;
}

void SparkWrapper::CommitConfig() {
  if (m_maxConfig)
    m_spark->Configure(*m_maxConfig, rev::ResetMode::kResetSafeParameters,
                       rev::PersistMode::kPersistParameters);
  else if (m_flexConfig)
    m_spark->Configure(*m_flexConfig, rev::ResetMode::kResetSafeParameters,
                       rev::PersistMode::kPersistParameters);
}

// ---- Simulation -------------------------------------------------------------

void SparkWrapper::SetupSimulation() {
  if (!frc::RobotBase::IsSimulation()) return;
  if (auto simMotor = m_config.GetSimMotor(); simMotor) {
    auto plant =
        frc::LinearSystemId::DCMotorSystem(*simMotor, units::kilogram_square_meter_t{0.001}, 1.0);
    m_motorSim.emplace(plant, *simMotor);
  }
}

void SparkWrapper::SimIterate() {
  if (!m_motorSim) return;
  double inputVolts = m_spark->GetAppliedOutput() * frc::sim::RoboRioSim::GetVInVoltage().value();
  m_motorSim->SetInputVoltage(units::volt_t{inputVolts});
  m_motorSim->Update(20_ms);
}

// ---- Encoder sync -----------------------------------------------------------

void SparkWrapper::SeedRelativeEncoder() {
  if (m_absEncoder) m_relEncoder->SetPosition(m_absEncoder->GetPosition() * 360.0);
}
void SparkWrapper::SynchronizeRelativeEncoder() {}

// ---- Open-loop outputs ------------------------------------------------------

void SparkWrapper::SetDutyCycle(double dc) { m_spark->Set(dc); }
double SparkWrapper::GetDutyCycle() { return m_spark->GetAppliedOutput(); }

void SparkWrapper::SetVoltage(units::volt_t voltage) { m_spark->SetVoltage(voltage); }

units::volt_t SparkWrapper::GetVoltage() {
  return units::volt_t{m_spark->GetAppliedOutput() * frc::sim::RoboRioSim::GetVInVoltage().value()};
}

// ---- Helper -----------------------------------------------------------------

double SparkWrapper::GetRotorRotations() const {
  auto& g = m_config.GetMotorGearing();
  return m_relEncoder->GetPosition() / 360.0 * (g ? g->GetMechanismToRotorRatio() : 1.0);
}
double SparkWrapper::GetRotorRPS() const {
  auto& g = m_config.GetMotorGearing();
  return m_relEncoder->GetVelocity() / 360.0 * (g ? g->GetMechanismToRotorRatio() : 1.0);
}
double SparkWrapper::GetMechRotations() const { return m_relEncoder->GetPosition() / 360.0; }
double SparkWrapper::GetMechRPS() const { return m_relEncoder->GetVelocity() / 360.0; }

// ---- Closed-loop setpoints --------------------------------------------------

void SparkWrapper::SetPosition(units::degree_t angle) {
  m_setpointPosition = angle;
  if (m_config.GetMotorControllerMode() == ControlMode::CLOSED_LOOP &&
      !m_closedLoopControllerRunning)
    m_sparkPid->SetSetpoint(angle.value(), m_positionControlType,
                            static_cast<ClosedLoopSlot>(m_revSlot));
}

void SparkWrapper::SetPosition(units::meter_t distance) {
  if (auto circ = m_config.GetMechanismCircumference(); circ)
    SetPosition(units::degree_t{distance.value() / circ->value() * 360.0});
}

void SparkWrapper::SetVelocity(units::degrees_per_second_t velocity) {
  m_setpointVelocity = velocity;
  if (m_config.GetMotorControllerMode() == ControlMode::CLOSED_LOOP &&
      !m_closedLoopControllerRunning)
    m_sparkPid->SetSetpoint(velocity.value(), m_velocityControlType,
                            static_cast<ClosedLoopSlot>(m_revSlot));
}

void SparkWrapper::SetVelocity(units::meters_per_second_t velocity) {
  if (auto circ = m_config.GetMechanismCircumference(); circ)
    SetVelocity(units::degrees_per_second_t{velocity.value() / circ->value() * 360.0});
}

// ---- Encoder writes ---------------------------------------------------------

void SparkWrapper::SetEncoderPosition(units::degree_t angle) {
  m_relEncoder->SetPosition(angle.value());
}
void SparkWrapper::SetEncoderPosition(units::meter_t distance) {
  if (auto circ = m_config.GetMechanismCircumference(); circ)
    SetEncoderPosition(units::degree_t{distance.value() / circ->value() * 360.0});
}
void SparkWrapper::SetEncoderVelocity(units::degrees_per_second_t) {}
void SparkWrapper::SetEncoderVelocity(units::meters_per_second_t) {}

// ---- Encoder reads ----------------------------------------------------------

units::degree_t SparkWrapper::GetMechanismPosition() {
  return units::degree_t{m_relEncoder->GetPosition()};
}
units::degrees_per_second_t SparkWrapper::GetMechanismVelocity() {
  return units::degrees_per_second_t{m_relEncoder->GetVelocity()};
}
units::degrees_per_second_squared_t SparkWrapper::GetMechanismAcceleration() {
  return units::degrees_per_second_squared_t{
      m_accelFilter.Derivative(GetMechanismVelocity().value())};
}
units::degree_t SparkWrapper::GetRotorPosition() {
  return units::degree_t{GetRotorRotations() * 360.0};
}
units::degrees_per_second_t SparkWrapper::GetRotorVelocity() {
  return units::degrees_per_second_t{GetRotorRPS() * 360.0};
}

units::meter_t SparkWrapper::GetMeasurementPosition() {
  auto circ = m_config.GetMechanismCircumference().value_or(1.0_m);
  return units::meter_t{GetMechanismPosition().value() / 360.0 * circ.value()};
}
units::meters_per_second_t SparkWrapper::GetMeasurementVelocity() {
  auto circ = m_config.GetMechanismCircumference().value_or(1.0_m);
  return units::meters_per_second_t{GetMechanismVelocity().value() / 360.0 * circ.value()};
}
units::meters_per_second_squared_t SparkWrapper::GetMeasurementAcceleration() {
  auto circ = m_config.GetMechanismCircumference().value_or(1.0_m);
  return units::meters_per_second_squared_t{GetMechanismAcceleration().value() / 360.0 *
                                            circ.value()};
}

std::optional<units::degree_t> SparkWrapper::GetExternalEncoderPosition() {
  if (m_absEncoder) return units::degree_t{m_absEncoder->GetPosition() * 360.0};
  return std::nullopt;
}
std::optional<units::degrees_per_second_t> SparkWrapper::GetExternalEncoderVelocity() {
  if (m_absEncoder) return units::degrees_per_second_t{m_absEncoder->GetVelocity() * 360.0};
  return std::nullopt;
}

// ---- Motor status -----------------------------------------------------------

std::optional<units::ampere_t> SparkWrapper::GetSupplyCurrent() {
  return units::ampere_t{m_spark->GetOutputCurrent()};
}
units::ampere_t SparkWrapper::GetStatorCurrent() {
  return units::ampere_t{m_spark->GetOutputCurrent()};
}
units::celsius_t SparkWrapper::GetTemperature() {
  return units::celsius_t{m_spark->GetMotorTemperature()};
}
frc::DCMotor SparkWrapper::GetDCMotor() { return m_motor; }

// ---- Live-tuning setters ----------------------------------------------------

void SparkWrapper::SetIdleMode(MotorMode mode) {
  auto doConfig = [&](SparkBaseConfig& cfg) {
    cfg.SetIdleMode(mode == MotorMode::BRAKE ? SparkBaseConfig::IdleMode::kBrake
                                             : SparkBaseConfig::IdleMode::kCoast);
  };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetMotorInverted(bool inv) {
  auto doConfig = [&](SparkBaseConfig& cfg) { cfg.Inverted(inv); };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetEncoderInverted(bool inv) {
  auto doConfig = [&](SparkBaseConfig& cfg) { cfg.encoder.Inverted(inv); };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetKp(double kP) {
  auto doConfig = [&](SparkBaseConfig& cfg) { cfg.closedLoop.P(kP); };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetKi(double kI) {
  auto doConfig = [&](SparkBaseConfig& cfg) { cfg.closedLoop.I(kI); };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetKd(double kD) {
  auto doConfig = [&](SparkBaseConfig& cfg) { cfg.closedLoop.D(kD); };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetFeedback(double kP, double kI, double kD) {
  auto doConfig = [&](SparkBaseConfig& cfg) { cfg.closedLoop.Pid(kP, kI, kD); };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetKs(double) {}
void SparkWrapper::SetKv(double kV) {
  auto doConfig = [&](SparkBaseConfig& cfg) { cfg.closedLoop.feedForward.kV(kV); };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}
void SparkWrapper::SetKa(double) {}
void SparkWrapper::SetKg(double) {}
void SparkWrapper::SetFeedforward(double, double kV, double, double) { SetKv(kV); }

void SparkWrapper::SetStatorCurrentLimit(units::ampere_t limit) {
  auto doConfig = [&](SparkBaseConfig& cfg) {
    cfg.SmartCurrentLimit(static_cast<int>(limit.value()));
  };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetSupplyCurrentLimit(units::ampere_t limit) {
  auto doConfig = [&](SparkBaseConfig& cfg) { cfg.SecondaryCurrentLimit(limit.value()); };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetClosedLoopRampRate(units::second_t r) {
  auto doConfig = [&](SparkBaseConfig& cfg) { cfg.ClosedLoopRampRate(r.value()); };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetOpenLoopRampRate(units::second_t r) {
  auto doConfig = [&](SparkBaseConfig& cfg) { cfg.OpenLoopRampRate(r.value()); };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetMechanismUpperLimit(units::degree_t) {}
void SparkWrapper::SetMechanismLowerLimit(units::degree_t) {}
void SparkWrapper::SetMechanismLimits(units::degree_t, units::degree_t) {}
void SparkWrapper::SetMechanismLimitsEnabled(bool) {}
void SparkWrapper::SetMeasurementUpperLimit(units::meter_t) {}
void SparkWrapper::SetMeasurementLowerLimit(units::meter_t) {}

void SparkWrapper::SetMotionProfileMaxVelocity(units::degrees_per_second_t vel) {
  auto doConfig = [&](SparkBaseConfig& cfg) {
    cfg.closedLoop.maxMotion.CruiseVelocity(vel.value());
  };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetMotionProfileMaxVelocity(units::meters_per_second_t vel) {
  if (auto circ = m_config.GetMechanismCircumference(); circ)
    SetMotionProfileMaxVelocity(units::degrees_per_second_t{vel.value() / circ->value() * 360.0});
}

void SparkWrapper::SetMotionProfileMaxAcceleration(units::degrees_per_second_squared_t acc) {
  auto doConfig = [&](SparkBaseConfig& cfg) {
    cfg.closedLoop.maxMotion.MaxAcceleration(acc.value());
  };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetMotionProfileMaxAcceleration(units::meters_per_second_squared_t acc) {
  if (auto circ = m_config.GetMechanismCircumference(); circ)
    SetMotionProfileMaxAcceleration(
        units::degrees_per_second_squared_t{acc.value() / circ->value() * 360.0});
}

void SparkWrapper::SetMotionProfileMaxJerk(
    units::unit_t<units::compound_unit<units::angular_acceleration::degrees_per_second_squared,
                                       units::inverse<units::seconds>>>) {}

void SparkWrapper::SetExponentialProfile(std::optional<double>, std::optional<double>,
                                         std::optional<units::volt_t>) {}

void SparkWrapper::SetClosedLoopSlot(ClosedLoopControllerSlot slot) {
  m_slot = slot;
  m_revSlot = static_cast<int>(slot);
}

SmartMotorControllerConfig& SparkWrapper::GetConfig() { return m_config; }
void* SparkWrapper::GetMotorController() { return m_spark; }
void* SparkWrapper::GetMotorControllerConfig() { return nullptr; }

}  // namespace yams::motorcontrollers::local
