// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/SmartMotorController.hpp"

#include <frc/DriverStation.h>
#include <frc/MathUtil.h>
#include <frc/filter/Debouncer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angular_velocity.h>
#include <units/time.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <memory>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

#include "yams/exceptions.hpp"
#include "yams/motorcontrollers/SmartMotorControllerCommandRegistry.hpp"

namespace yams::motorcontrollers {

// ---- Closed-loop controller thread ----------------------------------------

std::optional<frc::TrapezoidProfile<units::turns>::State>
SmartMotorController::GetTrapezoidalProfileState() {
  if (!m_config.GetTrapezoidProfile()) return std::nullopt;
  if (m_config.GetLinearClosedLoopControllerUse()) {
    return std::nullopt;  // linear profiles handled separately
  }
  return frc::TrapezoidProfile<units::turns>::State{
      units::turn_t{GetMechanismPosition()}, units::turns_per_second_t{GetMechanismVelocity()}};
}

std::optional<frc::ExponentialProfile<units::turns, units::volts>::State>
SmartMotorController::GetExponentialProfileState() {
  if (!m_config.GetExponentialProfile()) return std::nullopt;
  if (m_config.GetLinearClosedLoopControllerUse()) return std::nullopt;
  return frc::ExponentialProfile<units::turns, units::volts>::State{
      units::turn_t{GetMechanismPosition()}, units::turns_per_second_t{GetMechanismVelocity()}};
}

void SmartMotorController::StopClosedLoopController() {
  if (m_closedLoopControllerThread) {
    m_closedLoopControllerThread->Stop();
    m_closedLoopControllerRunning = false;
  }
}

void SmartMotorController::StartClosedLoopController() {
  if (m_closedLoopControllerThread &&
      m_config.GetMotorControllerMode() == ControlMode::CLOSED_LOOP) {
    if (m_pid) m_pid->Reset();
    m_trapState = GetTrapezoidalProfileState();
    m_expoState = GetExponentialProfileState();
    if (m_lqr) {
      if (m_config.GetLinearClosedLoopControllerUse()) {
        m_lqr->Reset(GetMeasurementPosition(), GetMeasurementVelocity());
      } else {
        m_lqr->Reset(units::radian_t{GetMechanismPosition()},
                     units::radians_per_second_t{GetMechanismVelocity()});
      }
    }
    m_closedLoopControllerThread->Stop();
    units::second_t period = m_config.GetClosedLoopControlPeriod().value_or(20_ms);
    m_closedLoopControllerThread->StartPeriodic(period);
    m_closedLoopControllerRunning = true;
  }
}

void SmartMotorController::IterateClosedLoopController() {
  if (!m_closedLoopControllerRunning) return;

  SynchronizeRelativeEncoder();

  bool linearMode = m_config.GetLinearClosedLoopControllerUse();
  units::second_t loopTime = m_config.GetClosedLoopControlPeriod().value_or(20_ms);

  // Clamp setpoint to limits
  if (m_setpointPosition.has_value()) {
    auto lower = m_config.GetMechanismLowerLimit();
    auto upper = m_config.GetMechanismUpperLimit();
    if (lower && *m_setpointPosition < *lower) {
      std::fprintf(stderr, "[YAMS] Setpoint below lower limit, clamping.\n");
      m_setpointPosition = lower;
    }
    if (upper && *m_setpointPosition > *upper) {
      std::fprintf(stderr, "[YAMS] Setpoint above upper limit, clamping.\n");
      m_setpointPosition = upper;
    }
  }

  // Motion profile advancement
  frc::TrapezoidProfile<units::turns>::State nextTrapState{};
  frc::ExponentialProfile<units::turns, units::volts>::State nextExpoState{};
  frc::TrapezoidProfile<units::meters>::State nextLinTrapState{};
  bool velocityTrapProfile = false;

  if (m_setpointPosition.has_value()) {
    if (!linearMode) {
      auto curPos = units::turn_t{GetMechanismPosition()};
      auto curVel = units::turns_per_second_t{GetMechanismVelocity()};
      auto setTurns = units::turn_t{*m_setpointPosition};

      if (auto expoP = m_config.GetExponentialProfile(); expoP) {
        auto curState = m_expoState.value_or(
            frc::ExponentialProfile<units::turns, units::volts>::State{curPos, curVel});
        nextExpoState = expoP->Calculate(loopTime, curState, {setTurns, {}});
        m_expoState = nextExpoState;
      } else if (auto trapP = m_config.GetTrapezoidProfile(); trapP) {
        auto curState =
            m_trapState.value_or(frc::TrapezoidProfile<units::turns>::State{curPos, curVel});
        nextTrapState = trapP->Calculate(loopTime, curState, {setTurns, {}});
        m_trapState = nextTrapState;
      }
    } else {
      auto curDist = GetMeasurementPosition();
      auto curLinV = GetMeasurementVelocity();
      auto setMeters = m_config.ConvertFromMechanism(*m_setpointPosition);

      if (auto trapP = m_config.GetLinearTrapezoidProfile(); trapP) {
        auto curState = m_linearTrapState.value_or(
            frc::TrapezoidProfile<units::meters>::State{curDist, curLinV});
        nextLinTrapState = trapP->Calculate(loopTime, curState, {setMeters, {}});
        m_linearTrapState = nextLinTrapState;
      }
    }
  } else if (m_setpointVelocity.has_value()) {
    if (auto trapP = m_config.GetTrapezoidProfile();
        trapP && m_config.GetVelocityTrapezoidalProfileInUse()) {
      auto setTurns = units::turns_per_second_t{*m_setpointVelocity};
      auto curTurns = units::turns_per_second_t{GetMechanismVelocity()};
      auto curState = m_trapState.value_or(
          frc::TrapezoidProfile<units::turns>::State{units::turn_t{curTurns.value()}, {}});
      nextTrapState = trapP->Calculate(loopTime, curState, {units::turn_t{setTurns.value()}, {}});
      m_trapState = nextTrapState;
      velocityTrapProfile = true;
    }
  }

  // PID + feedforward output
  double pidOutput = 0.0;
  double ffOutput = 0.0;

  if (m_setpointPosition.has_value()) {
    double measured{}, setpoint{}, velProfile{};
    if (!linearMode) {
      measured = units::turn_t{GetMechanismPosition()}.value();
      setpoint = units::turn_t{*m_setpointPosition}.value();
      if (m_config.GetExponentialProfile()) {
        setpoint = nextExpoState.position.value();
        velProfile = nextExpoState.velocity.value();
      } else if (m_config.GetTrapezoidProfile() && !m_config.GetVelocityTrapezoidalProfileInUse()) {
        setpoint = nextTrapState.position.value();
        velProfile = nextTrapState.velocity.value();
      }
    } else {
      measured = GetMeasurementPosition().value();
      setpoint = m_config.ConvertFromMechanism(*m_setpointPosition).value();
      if (m_config.GetLinearTrapezoidProfile()) {
        setpoint = nextLinTrapState.position.value();
        velProfile = nextLinTrapState.velocity.value();
      }
    }
    if (m_pid) pidOutput = m_pid->Calculate(measured, setpoint);
    if (m_lqr) {
      if (!linearMode) {
        pidOutput =
            m_lqr
                ->Calculate(units::radian_t{measured * (2.0 * std::numbers::pi)},
                            units::radian_t{setpoint * (2.0 * std::numbers::pi)},
                            units::radians_per_second_t{velProfile * (2.0 * std::numbers::pi)})
                .value();
      } else {
        pidOutput = m_lqr
                        ->Calculate(units::meter_t{measured}, units::meter_t{setpoint},
                                    units::meters_per_second_t{velProfile})
                        .value();
      }
    }

    // Arm feedforward
    if (auto armFF = m_config.GetArmFeedforward(m_slot); armFF) {
      auto profiled = m_config.HasTrapezoidProfile() || m_config.HasExponentialProfile();
      if (profiled) {
        double curVelRad =
            m_trapState
                ? m_trapState->velocity.value() * (2.0 * std::numbers::pi)
                : (m_expoState ? m_expoState->velocity.value() * (2.0 * std::numbers::pi) : 0.0);
        double nxtVelRad = m_config.GetTrapezoidProfile()
                               ? nextTrapState.velocity.value() * (2.0 * std::numbers::pi)
                               : nextExpoState.velocity.value() * (2.0 * std::numbers::pi);
        ffOutput = armFF
                       ->Calculate(units::radian_t{units::radian_t{GetMechanismPosition()}.value()},
                                   units::radians_per_second_t{curVelRad},
                                   units::radians_per_second_t{nxtVelRad})
                       .value();
      } else {
        auto setV = m_setpointVelocity.value_or(units::turns_per_second_t{0});
        ffOutput = armFF
                       ->Calculate(units::radian_t{units::radian_t{GetMechanismPosition()}.value()},
                                   units::radians_per_second_t{GetMechanismVelocity()},
                                   units::radians_per_second_t{setV})
                       .value();
      }
    }

    // Elevator feedforward
    if (auto elevFF = m_config.GetElevatorFeedforward(m_slot); elevFF) {
      auto profiled = m_config.GetLinearTrapezoidProfile().has_value();
      if (profiled) {
        double curLinV = m_linearTrapState ? m_linearTrapState->velocity.value() : 0.0;
        double nxtLinV = nextLinTrapState.velocity.value();
        ffOutput = elevFF
                       ->Calculate(units::meters_per_second_t{curLinV},
                                   units::meters_per_second_t{nxtLinV})
                       .value();
      } else {
        ffOutput =
            elevFF->Calculate(GetMeasurementVelocity(), units::meters_per_second_t{0}).value();
      }
    }

    // Simple motor feedforward
    if (auto simFF = m_config.GetSimpleFeedforward(m_slot); simFF) {
      auto profiled = m_config.HasTrapezoidProfile() || m_config.HasExponentialProfile();
      if (profiled) {
        double curV = m_trapState ? m_trapState->velocity.value()
                                  : (m_expoState ? m_expoState->velocity.value() : 0.0);
        double nxtV = m_config.GetTrapezoidProfile() ? nextTrapState.velocity.value()
                                                     : nextExpoState.velocity.value();
        ffOutput =
            simFF->Calculate(units::turns_per_second_t{curV}, units::turns_per_second_t{nxtV})
                .value();
      } else {
        auto nxtV = m_setpointVelocity.value_or(units::turns_per_second_t{0});
        ffOutput = simFF
                       ->Calculate(units::turns_per_second_t{GetMechanismVelocity()},
                                   units::turns_per_second_t{nxtV})
                       .value();
      }
    }

  } else if (m_setpointVelocity.has_value()) {
    double measured{}, setpoint{};
    if (!linearMode) {
      measured = units::turns_per_second_t{GetMechanismVelocity()}.value();
      setpoint = units::turns_per_second_t{*m_setpointVelocity}.value();
      if (velocityTrapProfile) setpoint = nextTrapState.position.value();
    } else {
      measured = GetMeasurementVelocity().value();
      setpoint = m_config.ConvertFromMechanism(*m_setpointVelocity).value();
    }
    if (m_pid) pidOutput = m_pid->Calculate(measured, setpoint);
    if (m_lqr) {
      if (!linearMode) {
        pidOutput =
            m_lqr
                ->Calculate(units::radians_per_second_t{measured * (2.0 * std::numbers::pi)},
                            units::radians_per_second_t{setpoint * (2.0 * std::numbers::pi)})
                .value();
      } else {
        pidOutput = m_lqr
                        ->Calculate(units::meters_per_second_t{measured},
                                    units::meters_per_second_t{setpoint})
                        .value();
      }
    }
    if (auto simFF = m_config.GetSimpleFeedforward(m_slot); simFF) {
      double nxtV = velocityTrapProfile ? nextTrapState.position.value()
                                        : units::turns_per_second_t{*m_setpointVelocity}.value();
      ffOutput = simFF
                     ->Calculate(units::turns_per_second_t{GetMechanismVelocity()},
                                 units::turns_per_second_t{nxtV})
                     .value();
    }
  }

  // Boundary safety
  if (auto upper = m_config.GetMechanismUpperLimit(); upper) {
    if (GetMechanismPosition() > *upper && (pidOutput + ffOutput) > 0.0) {
      pidOutput = ffOutput = 0.0;
    }
  }
  if (auto lower = m_config.GetMechanismLowerLimit(); lower) {
    if (GetMechanismPosition() < *lower && (pidOutput + ffOutput) < 0.0) {
      pidOutput = ffOutput = 0.0;
    }
  }
  if (auto tempCutoff = m_config.GetTemperatureCutoff(); tempCutoff) {
    if (GetTemperature() >= *tempCutoff) {
      pidOutput = ffOutput = 0.0;
    }
  }

  double output = pidOutput + ffOutput;
  if (auto maxV = m_config.GetClosedLoopControllerMaximumVoltage(); maxV) {
    output = std::clamp(output, -maxV->value(), maxV->value());
  }
  SetVoltage(units::volt_t{output});
}

// ---- Telemetry ------------------------------------------------------------

SmartMotorController& SmartMotorController::WithTelemetry(
    telemetry::SmartMotorControllerTelemetryConfig config) {
  m_telemetryConfig = std::move(config);
  m_telemetryConfigExplicit = true;
  return *this;
}

void SmartMotorController::SetupTelemetry(std::shared_ptr<nt::NetworkTable> dataTable,
                                          std::shared_ptr<nt::NetworkTable> tuningTable) {
  if (m_parentTable) return;  // already set up
  m_parentTable = dataTable;
  if (!m_config.GetTelemetryName()) return;

  m_telemetryTable = dataTable->GetSubTable(GetName());
  m_tuningTable = tuningTable->GetSubTable(GetName());

  auto verbosity =
      m_config.GetVerbosity().value_or(SmartMotorControllerConfig::TelemetryVerbosity::LOW);
  if (!m_telemetryConfigExplicit) m_telemetryConfig.WithTelemetryVerbosity(verbosity);

  auto& dfields = m_telemetryConfig.GetDoubleFields(*this);
  auto& bfields = m_telemetryConfig.GetBoolFields(*this);
  m_telemetry.SetupTelemetry(*this, m_telemetryTable, m_tuningTable, dfields, bfields,
                             m_telemetryConfig.GetNT4Enabled(), m_telemetryConfig.GetDataLogName());

  // Live tuning commands (requires subsystem and tuning enabled)
  if (m_telemetry.TuningEnabled() && m_config.GetSubsystem() != nullptr) {
    auto* subsystem = m_config.GetSubsystem();
    auto tablePath = m_telemetryTable->GetPath().substr(1);
    auto parts = [&]() {
      std::vector<std::string> p;
      std::string seg;
      for (char c : tablePath) {
        if (c == '/') {
          if (!seg.empty()) {
            p.push_back(seg);
            seg.clear();
          }
        } else {
          seg += c;
        }
      }
      if (!seg.empty()) p.push_back(seg);
      return p;
    }();
    auto motorName = parts.empty() ? GetName() : parts.back();
    auto cmdPath =
        "Mechanisms/Commands/" + (parts.empty() ? motorName : parts.front()) + "/" + motorName;

    // Live Tuning command (one per subsystem, shared across all SMCs)
    SmartMotorControllerCommandRegistry::AddCommand(
        "Live Tuning", subsystem, [this] { m_telemetry.ApplyTuningValues(*this); });
  }
}

void SmartMotorController::SetupTelemetry() {
  auto inst = nt::NetworkTableInstance::GetDefault();
  SetupTelemetry(inst.GetTable("Mechanisms"), inst.GetTable("Tuning"));
}

void SmartMotorController::UpdateTelemetry() {
  if (!m_telemetryTable) SetupTelemetry();
  m_telemetry.Publish(*this);
  if (m_telemetry.TuningEnabled()) {
    m_telemetry.ApplyTuningValues(*this);
  }
}

telemetry::UnsupportedTelemetryFields SmartMotorController::GetUnsupportedTelemetryFields() {
  return {};  // base: no unsupported fields
}

SmartMotorController::ClosedLoopControllerSlot SmartMotorController::GetClosedLoopControllerSlot()
    const {
  return m_slot;
}

// ---- Misc -----------------------------------------------------------------

std::optional<units::turn_t> SmartMotorController::GetMechanismPositionSetpoint() const {
  return m_setpointPosition;
}

std::optional<units::turns_per_second_t> SmartMotorController::GetMechanismSetpointVelocity()
    const {
  return m_setpointVelocity;
}

std::optional<units::meter_t> SmartMotorController::GetMeasurementPositionSetpoint() const {
  if (!m_setpointPosition) return std::nullopt;
  if (!m_config.GetMechanismCircumference()) return std::nullopt;
  return m_config.ConvertFromMechanism(*m_setpointPosition);
}

std::optional<units::meters_per_second_t> SmartMotorController::GetMeasurementSetpointVelocity()
    const {
  if (!m_setpointVelocity) return std::nullopt;
  if (!m_config.GetMechanismCircumference()) return std::nullopt;
  return m_config.ConvertFromMechanism(*m_setpointVelocity);
}

std::string SmartMotorController::GetName() const {
  return m_config.GetTelemetryName().value_or("SmartMotorController");
}

bool SmartMotorController::IsMotor(const frc::DCMotor& a, const frc::DCMotor& b) const {
  return a.stallTorque == b.stallTorque && a.stallCurrent == b.stallCurrent &&
         a.freeCurrent == b.freeCurrent && a.freeSpeed == b.freeSpeed && a.Kt == b.Kt &&
         a.Kv == b.Kv && a.nominalVoltage == b.nominalVoltage;
}

void SmartMotorController::CheckConfigSafety() {
  frc::DCMotor neo550 = frc::DCMotor::NEO550(1);
  if (IsMotor(GetDCMotor(), neo550)) {
    auto limit = m_config.GetStatorStallCurrentLimit();
    if (!limit) {
      throw exceptions::SmartMotorControllerConfigurationException(
          "Stator current limit is not defined for NEO550!", "Safety check failed.",
          "WithStatorCurrentLimit(Current)");
    }
    if (*limit > 40) {
      throw exceptions::SmartMotorControllerConfigurationException(
          "Stator current limit is too high for NEO550!", "Safety check failed.",
          "WithStatorCurrentLimit(Current) where the Current is under 40A");
    }
  }
}

void SmartMotorController::Close() {
  if (m_closedLoopControllerThread) {
    m_closedLoopControllerThread->Stop();
    m_closedLoopControllerThread.reset();
  }
}

// ---- SimSupplier ------------------------------------------------------------

void SmartMotorController::SetSimSupplier(std::shared_ptr<SimSupplier> supplier) {
  m_simSupplier = std::move(supplier);
}

SimSupplier* SmartMotorController::GetSimSupplier() const { return m_simSupplier.get(); }

// ---- Loosely coupled followers ----------------------------------------------

void SmartMotorController::LoadLooselyCoupledFollowers() {
  m_looseFollowers = m_config.GetLooselyCoupledFollowers();
}

void SmartMotorController::ForwardPositionToFollowers(units::turn_t pos) {
  for (auto* f : m_looseFollowers) f->SetPosition(pos);
}

void SmartMotorController::ForwardPositionToFollowers(units::meter_t dist) {
  for (auto* f : m_looseFollowers) f->SetPosition(dist);
}

void SmartMotorController::ForwardVelocityToFollowers(units::turns_per_second_t vel) {
  for (auto* f : m_looseFollowers) f->SetVelocity(vel);
}

void SmartMotorController::ForwardVelocityToFollowers(units::meters_per_second_t vel) {
  for (auto* f : m_looseFollowers) f->SetVelocity(vel);
}

}  // namespace yams::motorcontrollers
