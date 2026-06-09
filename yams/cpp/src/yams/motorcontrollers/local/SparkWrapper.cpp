// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/local/SparkWrapper.hpp"

#include <frc/RobotBase.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include <rev/ClosedLoopTypes.h>
#include <rev/ConfigureTypes.h>
#include <units/moment_of_inertia.h>

#include <iostream>
#include <stdexcept>
#include <vector>

#include "yams/motorcontrollers/simulation/DCMotorSimSupplier.hpp"

using namespace rev::spark;

namespace yams::motorcontrollers::local {

SparkWrapper::SparkWrapper(SparkMax& spark, frc::DCMotor motor,
                           const SmartMotorControllerConfig& cfg)
    : SmartMotorController(), m_motor(motor) {
  if (auto& vc = cfg.GetVendorConfig(); vc.has_value()) {
    if (auto* p = std::any_cast<rev::spark::SparkMaxConfig>(&vc.value())) {
      m_maxConfig.emplace(*p);
    } else {
      throw std::invalid_argument(
          "SparkWrapper (SparkMax): WithVendorConfig requires a SparkMaxConfig");
    }
  } else {
    m_maxConfig.emplace();
  }
  Init(&spark, motor, cfg);
}

SparkWrapper::SparkWrapper(SparkFlex& spark, frc::DCMotor motor,
                           const SmartMotorControllerConfig& cfg)
    : SmartMotorController(), m_motor(motor) {
  if (auto& vc = cfg.GetVendorConfig(); vc.has_value()) {
    if (auto* p = std::any_cast<rev::spark::SparkFlexConfig>(&vc.value())) {
      m_flexConfig.emplace(*p);
    } else {
      throw std::invalid_argument(
          "SparkWrapper (SparkFlex): WithVendorConfig requires a SparkFlexConfig");
    }
  } else {
    m_flexConfig.emplace();
  }
  Init(&spark, motor, cfg);
}

void SparkWrapper::Init(SparkBase* spark, frc::DCMotor motor,
                        const SmartMotorControllerConfig& cfg) {
  m_spark = spark;
  m_sparkPid = &spark->GetClosedLoopController();
  m_relEncoder = &spark->GetEncoder();
  m_config = cfg;
  m_config.WithSimMotor(motor);

  SetupSimulation();
  ApplyConfig(cfg);
  CheckConfigSafety();
}

// ---- Configuration ----------------------------------------------------------

bool SparkWrapper::ApplyConfig(const SmartMotorControllerConfig& cfg) {
  m_config = cfg;

  auto doConfig = [&](SparkBaseConfig& sparkCfg) {
    if (auto inv = cfg.GetMotorInverted(); inv) sparkCfg.Inverted(*inv);
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
    sparkCfg.encoder.PositionConversionFactor(convFactor);
    sparkCfg.encoder.VelocityConversionFactor(convFactor / 60.0);

    using Slot = SmartMotorControllerConfig::ClosedLoopControllerSlot;
    for (auto slot : {Slot::SLOT_0, Slot::SLOT_1, Slot::SLOT_2, Slot::SLOT_3}) {
      auto gains = cfg.GetSlotGains(slot);
      auto revSlot = static_cast<ClosedLoopSlot>(static_cast<int>(slot));
      sparkCfg.closedLoop.Pid(gains.kP, gains.kI, gains.kD, revSlot);
      if (gains.armFF) {
        sparkCfg.closedLoop.feedForward.kS(gains.kS, revSlot)
            .kV(gains.kV, revSlot)
            .kA(gains.kA, revSlot)
            .kCos(gains.kG, revSlot);
      } else if (gains.elevatorFF) {
        sparkCfg.closedLoop.feedForward.kS(gains.kS, revSlot)
            .kV(gains.kV, revSlot)
            .kA(gains.kA, revSlot)
            .kG(gains.kG, revSlot);
      } else if (gains.simpleFF) {
        sparkCfg.closedLoop.feedForward.kS(gains.kS, revSlot)
            .kV(gains.kV, revSlot)
            .kA(gains.kA, revSlot);
      }
    }

    if (cfg.HasTrapezoidProfile()) {
      // Angular profile supplies velocity directly in TPS.  Linear profiles store
      // velocity in m/s; convert to TPS using the mechanism circumference.
      auto cruiseVelTps = cfg.GetTrapMaxVelocityTurns();
      auto cruiseAccTps = cfg.GetTrapMaxAccelTurns();
      if (!cruiseVelTps) {
        if (auto linVel = cfg.GetTrapMaxVelocityLinear(); linVel)
          if (auto circ = cfg.GetMechanismCircumference(); circ && circ->value() != 0.0)
            cruiseVelTps = units::turns_per_second_t{linVel->value() / circ->value()};
      }
      if (!cruiseAccTps) {
        if (auto linAcc = cfg.GetTrapMaxAccelLinear(); linAcc)
          if (auto circ = cfg.GetMechanismCircumference(); circ && circ->value() != 0.0)
            cruiseAccTps = units::turns_per_second_squared_t{linAcc->value() / circ->value()};
      }
      if (cruiseVelTps) sparkCfg.closedLoop.maxMotion.CruiseVelocity(cruiseVelTps->value());
      if (cruiseAccTps) sparkCfg.closedLoop.maxMotion.MaxAcceleration(cruiseAccTps->value());
      m_positionControlType = cfg.GetVelocityTrapezoidalProfileInUse()
                                  ? SparkLowLevel::ControlType::kVelocity
                                  : SparkLowLevel::ControlType::kMAXMotionPositionControl;
      m_velocityControlType = SparkLowLevel::ControlType::kMAXMotionVelocityControl;
    }

    if (auto inv = cfg.GetEncoderInverted(); inv) sparkCfg.encoder.Inverted(*inv);
    if (auto offset = cfg.GetAbsoluteEncoderZeroOffset(); offset)
      sparkCfg.absoluteEncoder.ZeroOffset(offset->value() / 360.0);

    bool isClosedLoop = cfg.GetMotorControllerMode() == ControlMode::CLOSED_LOOP;
    if (auto lower = cfg.GetMechanismLowerLimit()) {
      units::turn_t lowerTurns{*lower};
      sparkCfg.softLimit.ReverseSoftLimit(lowerTurns.value()).ReverseSoftLimitEnabled(isClosedLoop);
    }
    if (auto upper = cfg.GetMechanismUpperLimit()) {
      units::turn_t upperTurns{*upper};
      sparkCfg.softLimit.ForwardSoftLimit(upperTurns.value()).ForwardSoftLimitEnabled(isClosedLoop);
    }
  };

  if (cfg.GetVendorControlRequest().has_value())
    throw std::runtime_error(
        "SparkWrapper does not support custom vendor control requests (WithVendorControlRequest).");

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

  if (!m_sparkSim.has_value()) {
    auto simMotor = m_config.GetSimMotor();
    auto& gearing = m_config.GetMotorGearing();
    if (!simMotor || !gearing) return;

    auto plant = frc::LinearSystemId::DCMotorSystem(*simMotor, m_config.GetMOI(),
                                                    gearing->GetMechanismToRotorRatio());
    m_motorSim.emplace(plant, *simMotor);

    auto period = m_config.GetClosedLoopControlPeriod().value_or(20_ms);
    SetSimSupplier(std::make_shared<simulation::DCMotorSimSupplier>(
        *m_motorSim, [this]() { return GetDutyCycle(); }, *gearing, period));

    m_sparkSim.emplace(m_spark, &m_motor);

    if (m_maxConfig)
      m_relEncoderSim.emplace(static_cast<rev::spark::SparkMax*>(m_spark));
    else if (m_flexConfig)
      m_relEncoderSim.emplace(static_cast<rev::spark::SparkFlex*>(m_spark));

    if (m_absEncoder) {
      if (m_maxConfig)
        m_absEncoderSim.emplace(static_cast<rev::spark::SparkMax*>(m_spark));
      else if (m_flexConfig)
        m_absEncoderSim.emplace(static_cast<rev::spark::SparkFlex*>(m_spark));
    }
  }

  if (auto startPos = m_config.GetStartingPosition()) {
    m_sparkSim->SetPosition(units::turn_t{startPos->value()}.value());
    if (m_relEncoderSim) m_relEncoderSim->SetPosition(startPos->value());
  }
}

void SparkWrapper::SimIterate() {
  // if (!frc::RobotBase::IsSimulation() || !m_simSupplier || !m_sparkSim) return;
  if (m_simSupplier) {
    if (m_simSupplier->IsWatchdogExpired()) {
      m_simSupplier->UpdateSim();
    }
    units::second_t dt = m_config.GetClosedLoopControlPeriod().value_or(20_ms);
    units::turns_per_second_t mechVelRps = m_simSupplier->GetMechanismVelocity();
    double vbus = m_simSupplier->GetMechanismSupplyVoltage().value();
    m_sparkSim->iterate(mechVelRps.value(), vbus, dt.value());

    if (m_relEncoderSim) m_relEncoderSim->iterate(mechVelRps.value(), dt.value());

    if (m_absEncoderSim) m_absEncoderSim->iterate(mechVelRps.value(), dt.value());
  }
}

// ---- Encoder sync -----------------------------------------------------------

void SparkWrapper::SeedRelativeEncoder() {
  if (m_absEncoder) m_relEncoder->SetPosition(m_absEncoder->GetPosition());
}
void SparkWrapper::SynchronizeRelativeEncoder() {}

// ---- Open-loop outputs ------------------------------------------------------

void SparkWrapper::SetDutyCycle(double dc) { m_spark->Set(dc); }
double SparkWrapper::GetDutyCycle() {
  return m_sparkSim ? m_sparkSim.value().GetAppliedOutput() : m_spark->GetAppliedOutput();
}

void SparkWrapper::SetVoltage(units::volt_t voltage) { m_spark->SetVoltage(voltage); }

units::volt_t SparkWrapper::GetVoltage() {
  if (m_simSupplier) return m_simSupplier->GetMechanismStatorVoltage();
  return units::volt_t{m_spark->GetAppliedOutput() * m_spark->GetBusVoltage()};
}

// ---- Closed-loop setpoints --------------------------------------------------

void SparkWrapper::SetPosition(units::turn_t angle) {
  m_setpointPosition = angle;
  if (m_config.GetMotorControllerMode() == ControlMode::CLOSED_LOOP &&
      !m_closedLoopControllerRunning)
    m_sparkPid->SetSetpoint(angle.value(), m_positionControlType,
                            static_cast<ClosedLoopSlot>(m_revSlot));
}

void SparkWrapper::SetPosition(units::meter_t distance) {
  if (auto circ = m_config.GetMechanismCircumference(); circ)
    SetPosition(units::turn_t{distance.value() / circ->value()});
}

void SparkWrapper::SetVelocity(units::turns_per_second_t velocity) {
  m_setpointVelocity = velocity;
  if (m_config.GetMotorControllerMode() == ControlMode::CLOSED_LOOP &&
      !m_closedLoopControllerRunning)
    m_sparkPid->SetSetpoint(velocity.value(), m_velocityControlType,
                            static_cast<ClosedLoopSlot>(m_revSlot));
}

void SparkWrapper::SetVelocity(units::meters_per_second_t velocity) {
  if (auto circ = m_config.GetMechanismCircumference(); circ)
    SetVelocity(units::turns_per_second_t{velocity.value() / circ->value()});
}

// ---- Encoder writes ---------------------------------------------------------

void SparkWrapper::SetEncoderPosition(units::turn_t angle) {
  m_relEncoder->SetPosition(angle.value());
  if (m_relEncoderSim) m_relEncoderSim->SetPosition(angle.value());
  if (m_absEncoderSim) m_absEncoderSim->SetPosition(angle.value());
  if (m_simSupplier) m_simSupplier->SetMechanismPosition(angle);
}
void SparkWrapper::SetEncoderPosition(units::meter_t distance) {
  if (auto circ = m_config.GetMechanismCircumference(); circ)
    SetEncoderPosition(units::turn_t{distance.value() / circ->value()});
}
void SparkWrapper::SetEncoderVelocity(units::turns_per_second_t) {}
void SparkWrapper::SetEncoderVelocity(units::meters_per_second_t) {}

// ---- Encoder reads ----------------------------------------------------------

units::turn_t SparkWrapper::GetMechanismPosition() {
  return units::turn_t{m_relEncoder->GetPosition()};
}
units::turns_per_second_t SparkWrapper::GetMechanismVelocity() {
  return units::turns_per_second_t{m_relEncoder->GetVelocity()};
}
units::turns_per_second_squared_t SparkWrapper::GetMechanismAcceleration() {
  return units::turns_per_second_squared_t{
      m_accelFilter.Derivative(GetMechanismVelocity().value())};
}
units::turn_t SparkWrapper::GetRotorPosition() {
  auto& g = m_config.GetMotorGearing();
  return units::turn_t{m_relEncoder->GetPosition() * (g ? g->GetMechanismToRotorRatio() : 1.0)};
}
units::turns_per_second_t SparkWrapper::GetRotorVelocity() {
  auto& g = m_config.GetMotorGearing();
  return units::turns_per_second_t{m_relEncoder->GetVelocity() *
                                   (g ? g->GetMechanismToRotorRatio() : 1.0)};
}

units::meter_t SparkWrapper::GetMeasurementPosition() {
  auto circ = m_config.GetMechanismCircumference().value_or(1.0_m);
  return units::meter_t{GetMechanismPosition().value() * circ.value()};
}
units::meters_per_second_t SparkWrapper::GetMeasurementVelocity() {
  auto circ = m_config.GetMechanismCircumference().value_or(1.0_m);
  return units::meters_per_second_t{GetMechanismVelocity().value() * circ.value()};
}
units::meters_per_second_squared_t SparkWrapper::GetMeasurementAcceleration() {
  auto circ = m_config.GetMechanismCircumference().value_or(1.0_m);
  return units::meters_per_second_squared_t{GetMechanismAcceleration().value() * circ.value()};
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
  auto doConfig = [&](SparkBaseConfig& cfg) {
    cfg.closedLoop.P(kP, static_cast<ClosedLoopSlot>(m_revSlot));
  };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetKi(double kI) {
  auto doConfig = [&](SparkBaseConfig& cfg) {
    cfg.closedLoop.I(kI, static_cast<ClosedLoopSlot>(m_revSlot));
  };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetKd(double kD) {
  auto doConfig = [&](SparkBaseConfig& cfg) {
    cfg.closedLoop.D(kD, static_cast<ClosedLoopSlot>(m_revSlot));
  };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetFeedback(double kP, double kI, double kD) {
  auto doConfig = [&](SparkBaseConfig& cfg) {
    cfg.closedLoop.P(kP, static_cast<ClosedLoopSlot>(m_revSlot));
    cfg.closedLoop.I(kI, static_cast<ClosedLoopSlot>(m_revSlot));
    cfg.closedLoop.D(kD, static_cast<ClosedLoopSlot>(m_revSlot));
  };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetKs(double kS) {
  auto doConfig = [&](SparkBaseConfig& cfg) {
    cfg.closedLoop.feedForward.kS(kS, static_cast<ClosedLoopSlot>(m_revSlot));
  };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}
void SparkWrapper::SetKv(double kV) {
  auto doConfig = [&](SparkBaseConfig& cfg) {
    cfg.closedLoop.feedForward.kV(kV, static_cast<ClosedLoopSlot>(m_revSlot));
  };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}
void SparkWrapper::SetKa(double kA) {
  auto doConfig = [&](SparkBaseConfig& cfg) {
    cfg.closedLoop.feedForward.kA(kA, static_cast<ClosedLoopSlot>(m_revSlot));
  };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}
void SparkWrapper::SetKg(double kG) {
  auto doConfig = [&](SparkBaseConfig& cfg) {
    if (m_config.GetArmFeedforward(m_slot))
      cfg.closedLoop.feedForward.kCos(kG, static_cast<ClosedLoopSlot>(m_revSlot));
    else
      cfg.closedLoop.feedForward.kG(kG, static_cast<ClosedLoopSlot>(m_revSlot));
  };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}
void SparkWrapper::SetFeedforward(double kS, double kV, double kA, double kG) {
  auto doConfig = [&](SparkBaseConfig& cfg) {
    cfg.closedLoop.feedForward.kS(kS, static_cast<ClosedLoopSlot>(m_revSlot))
        .kV(kV, static_cast<ClosedLoopSlot>(m_revSlot))
        .kA(kA, static_cast<ClosedLoopSlot>(m_revSlot));
    if (m_config.GetArmFeedforward(m_slot))
      cfg.closedLoop.feedForward.kCos(kG, static_cast<ClosedLoopSlot>(m_revSlot));
    else if (m_config.GetElevatorFeedforward(m_slot))
      cfg.closedLoop.feedForward.kG(kG, static_cast<ClosedLoopSlot>(m_revSlot));
  };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

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

void SparkWrapper::SetMechanismUpperLimit(units::turn_t upper) {
  if (auto lower = m_config.GetMechanismLowerLimit())
    m_config.WithMechanismLimits(*lower, units::degree_t{upper});
  auto doConfig = [&](SparkBaseConfig& cfg) { cfg.softLimit.ForwardSoftLimit(upper.value()); };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}
void SparkWrapper::SetMechanismLowerLimit(units::turn_t lower) {
  if (auto upper = m_config.GetMechanismUpperLimit())
    m_config.WithMechanismLimits(units::degree_t{lower}, *upper);
  auto doConfig = [&](SparkBaseConfig& cfg) { cfg.softLimit.ReverseSoftLimit(lower.value()); };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}
void SparkWrapper::SetMechanismLimits(units::turn_t lower, units::turn_t upper) {
  m_config.WithMechanismLimits(units::degree_t{lower}, units::degree_t{upper});
  auto doConfig = [&](SparkBaseConfig& cfg) {
    cfg.softLimit.ReverseSoftLimit(lower.value()).ForwardSoftLimit(upper.value());
  };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}
void SparkWrapper::SetMechanismLimitsEnabled(bool enabled) {
  auto doConfig = [&](SparkBaseConfig& cfg) {
    cfg.softLimit.ForwardSoftLimitEnabled(enabled).ReverseSoftLimitEnabled(enabled);
  };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}
void SparkWrapper::SetMeasurementUpperLimit(units::meter_t upper) {
  auto circ = m_config.GetMechanismCircumference();
  auto lowerAngle = m_config.GetMechanismLowerLimit();
  if (!circ || !lowerAngle) return;
  units::turn_t lowerTurns{*lowerAngle};
  m_config.WithMeasurementLimits(units::meter_t{lowerTurns.value() * circ->value()}, upper);
  units::turn_t upperTurns{upper.value() / circ->value()};
  auto doConfig = [&](SparkBaseConfig& cfg) { cfg.softLimit.ForwardSoftLimit(upperTurns.value()); };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}
void SparkWrapper::SetMeasurementLowerLimit(units::meter_t lower) {
  auto circ = m_config.GetMechanismCircumference();
  auto upperAngle = m_config.GetMechanismUpperLimit();
  if (!circ || !upperAngle) return;
  units::turn_t upperTurns{*upperAngle};
  m_config.WithMeasurementLimits(lower, units::meter_t{upperTurns.value() * circ->value()});
  units::turn_t lowerTurns{lower.value() / circ->value()};
  auto doConfig = [&](SparkBaseConfig& cfg) { cfg.softLimit.ReverseSoftLimit(lowerTurns.value()); };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetMotionProfileMaxVelocity(units::turns_per_second_t vel) {
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
    SetMotionProfileMaxVelocity(units::turns_per_second_t{vel.value() / circ->value()});
}

void SparkWrapper::SetMotionProfileMaxAcceleration(units::turns_per_second_squared_t acc) {
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
    SetMotionProfileMaxAcceleration(units::turns_per_second_squared_t{acc.value() / circ->value()});
}

void SparkWrapper::SetMotionProfileMaxJerk(units::angular_jerk::turns_per_second_cubed_t) {}

void SparkWrapper::SetExponentialProfile(std::optional<double>, std::optional<double>,
                                         std::optional<units::volt_t>) {}

void SparkWrapper::SetClosedLoopSlot(ClosedLoopControllerSlot slot) {
  m_slot = slot;
  m_revSlot = static_cast<int>(slot);
}

SmartMotorControllerConfig& SparkWrapper::GetConfig() { return m_config; }
void* SparkWrapper::GetMotorController() { return m_spark; }
void* SparkWrapper::GetMotorControllerConfig() {
  if (m_maxConfig) return &m_maxConfig.value();
  if (m_flexConfig) return &m_flexConfig.value();
  return nullptr;
}

telemetry::UnsupportedTelemetryFields SparkWrapper::GetUnsupportedTelemetryFields() {
  return {std::nullopt, std::vector<telemetry::DoubleTelemetryField>{
                            telemetry::DoubleTelemetryField::SupplyCurrent,
                            telemetry::DoubleTelemetryField::SupplyCurrentLimit}};
}

}  // namespace yams::motorcontrollers::local
