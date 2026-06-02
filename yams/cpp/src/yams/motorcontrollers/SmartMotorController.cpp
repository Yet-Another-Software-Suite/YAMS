// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/SmartMotorController.h"

#include <frc/DriverStation.h>
#include <frc/MathUtil.h>
#include <networktables/NetworkTableInstance.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include "yams/exceptions/SmartMotorControllerConfigurationException.h"

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
        pidOutput = m_lqr
                        ->Calculate(units::radian_t{measured * (2.0 * M_PI)},
                                    units::radian_t{setpoint * (2.0 * M_PI)},
                                    units::radians_per_second_t{velProfile * (2.0 * M_PI)})
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
        double curVelRad = m_trapState
                               ? m_trapState->velocity.value() * (2.0 * M_PI)
                               : (m_expoState ? m_expoState->velocity.value() * (2.0 * M_PI) : 0.0);
        double nxtVelRad = m_config.GetTrapezoidProfile()
                               ? nextTrapState.velocity.value() * (2.0 * M_PI)
                               : nextExpoState.velocity.value() * (2.0 * M_PI);
        ffOutput = armFF
                       ->Calculate(units::radian_t{units::radian_t{GetMechanismPosition()}.value()},
                                   units::radians_per_second_t{curVelRad},
                                   units::radians_per_second_t{nxtVelRad})
                       .value();
      } else {
        auto setV = m_setpointVelocity.value_or(units::degrees_per_second_t{0});
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
        auto nxtV = m_setpointVelocity.value_or(units::degrees_per_second_t{0});
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
        pidOutput = m_lqr
                        ->Calculate(units::radians_per_second_t{measured * (2.0 * M_PI)},
                                    units::radians_per_second_t{setpoint * (2.0 * M_PI)})
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

void SmartMotorController::SetupTelemetry(std::shared_ptr<nt::NetworkTable> dataTable,
                                          std::shared_ptr<nt::NetworkTable> tuningTable) {
  if (m_parentTable) return;  // already set up
  m_parentTable = dataTable;
  if (m_config.GetTelemetryName()) {
    m_telemetryTable = dataTable->GetSubTable(GetName());
    m_tuningTable = tuningTable->GetSubTable(GetName());
  }
}

void SmartMotorController::SetupTelemetry() {
  auto inst = nt::NetworkTableInstance::GetDefault();
  SetupTelemetry(inst.GetTable("Mechanisms"), inst.GetTable("Tuning"));
}

void SmartMotorController::UpdateTelemetry() {
  if (m_telemetryTable && m_config.GetVerbosity()) {
    // Publish basic values to NT
    m_telemetryTable->GetEntry("position_deg").SetDouble(GetMechanismPosition().value());
    m_telemetryTable->GetEntry("velocity_dps").SetDouble(GetMechanismVelocity().value());
    m_telemetryTable->GetEntry("voltage_V").SetDouble(GetVoltage().value());
    m_telemetryTable->GetEntry("stator_A").SetDouble(GetStatorCurrent().value());
    m_telemetryTable->GetEntry("temp_C").SetDouble(GetTemperature().value());
  }
}

// ---- SysId ----------------------------------------------------------------

frc2::sysid::SysIdRoutine SmartMotorController::SysId(units::volt_t maxVoltage,
                                                      frc2::sysid::ramp_rate_t stepVoltage,
                                                      units::second_t testDuration) {
  if (!m_config.GetTelemetryName()) {
    throw SmartMotorControllerConfigurationException(
        "Telemetry is undefined", "Cannot create SysIdRoutine", "WithTelemetry(name, verbosity)");
  }
  frc2::sysid::Config cfg{stepVoltage, maxVoltage, testDuration, {}};
  if (m_config.GetLinearClosedLoopControllerUse()) {
    return frc2::sysid::SysIdRoutine(
        cfg, frc2::sysid::Mechanism{[this](units::volt_t v) { SetVoltage(v); },
                                    [this](frc::sysid::SysIdRoutineLog* log) {
                                      log->Motor(GetName())
                                          .voltage(GetVoltage())
                                          .velocity(GetMeasurementVelocity())
                                          .position(GetMeasurementPosition());
                                    },
                                    m_config.GetSubsystem()});
  } else {
    return frc2::sysid::SysIdRoutine(
        cfg,
        frc2::sysid::Mechanism{[this](units::volt_t v) { SetVoltage(v); },
                               [this](frc::sysid::SysIdRoutineLog* log) {
                                 log->Motor(GetName())
                                     .voltage(GetVoltage())
                                     .velocity(units::turns_per_second_t{GetMechanismVelocity()})
                                     .position(units::turn_t{GetMechanismPosition()});
                               },
                               m_config.GetSubsystem()});
  }
}

// ---- Misc -----------------------------------------------------------------

std::optional<units::degree_t> SmartMotorController::GetMechanismPositionSetpoint() const {
  return m_setpointPosition;
}

std::optional<units::degrees_per_second_t> SmartMotorController::GetMechanismSetpointVelocity()
    const {
  return m_setpointVelocity;
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
      throw SmartMotorControllerConfigurationException(
          "Stator current limit is not defined for NEO550!", "Safety check failed.",
          "WithStatorCurrentLimit(Current)");
    }
    if (*limit > 40) {
      throw SmartMotorControllerConfigurationException(
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

}  // namespace yams::motorcontrollers
