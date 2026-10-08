// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/SmartMotorController.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <memory>
#include <numbers>
#include <string>
#include <utility>
#include <vector>
#include <wpi/commands2/Commands.hpp>
#include <wpi/driverstation/DriverStation.hpp>
#include <wpi/math/filter/Debouncer.hpp>
#include <wpi/math/util/MathUtil.hpp>
#include <wpi/nt/NetworkTableInstance.hpp>
#include <wpi/system/Errors.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/time.hpp>

#include "yams/exceptions.hpp"
#include "yams/motorcontrollers/SmartMotorControllerCommandRegistry.hpp"
#include "yams/motorcontrollers/simulation/BatterySim.hpp"

namespace yams::motorcontrollers {

SmartMotorController::~SmartMotorController() { Close(); }

// ---- Closed-loop controller thread ----------------------------------------

void SmartMotorController::ResetProfileStates() {
  using TrapTurns = wpi::math::TrapezoidProfile<wpi::units::turns>;
  using TrapMeters = wpi::math::TrapezoidProfile<wpi::units::meters>;
  using ExpoTurns = wpi::math::ExponentialProfile<wpi::units::turns, wpi::units::volts>;
  using ExpoMeters = wpi::math::ExponentialProfile<wpi::units::meters, wpi::units::volts>;
  m_trapState.reset();
  m_expoState.reset();
  m_linearTrapState.reset();
  m_linearExpoState.reset();
  // A velocity profile starts from the measured velocity in the closed-loop iteration.
  if (m_config->GetVelocityTrapezoidalProfileInUse()) return;
  if (m_config->GetLinearClosedLoopControllerUse()) {
    auto pos = GetMeasurementPosition();
    auto vel = GetMeasurementVelocity();
    if (m_config->GetLinearTrapezoidProfile()) m_linearTrapState = TrapMeters::State{pos, vel};
    if (m_config->GetLinearExponentialProfile()) m_linearExpoState = ExpoMeters::State{pos, vel};
  } else {
    auto pos = GetMechanismPosition();
    auto vel = GetMechanismVelocity();
    if (m_config->GetTrapezoidProfile()) m_trapState = TrapTurns::State{pos, vel};
    if (m_config->GetExponentialProfile()) m_expoState = ExpoTurns::State{pos, vel};
  }
}

void SmartMotorController::StopClosedLoopController() {
  if (m_closedLoopControllerThread) {
    m_closedLoopControllerThread->Stop();
    m_closedLoopControllerRunning = false;
  }
}

void SmartMotorController::StartClosedLoopController() {
  if (m_closedLoopControllerThread &&
      m_config->GetMotorControllerMode() == ControlMode::CLOSED_LOOP) {
    if (m_pid) m_pid->Reset();
    m_lastClosedLoopMechanismPosition.reset();
    ResetProfileStates();
    if (m_lqr) {
      if (m_config->GetLinearClosedLoopControllerUse()) {
        m_lqr->Reset(GetMeasurementPosition(), GetMeasurementVelocity());
      } else {
        m_lqr->Reset(wpi::units::radian_t{GetMechanismPosition()},
                     wpi::units::radians_per_second_t{GetMechanismVelocity()});
      }
    }
    m_closedLoopControllerThread->Stop();
    wpi::units::second_t period = m_config->GetClosedLoopControlPeriod().value_or(20_ms);
    m_closedLoopControllerThread->StartPeriodic(period);
    m_closedLoopControllerRunning = true;
  }
}

void SmartMotorController::IterateClosedLoopController() {
  // Synchronize before the running check, like the Java implementation.
  SynchronizeRelativeEncoder();
  if (!m_closedLoopControllerRunning) return;

  const SmartMotorControllerConfig& cfg = *m_config;
  const bool linear = cfg.GetLinearClosedLoopControllerUse();
  const bool velocityProfileConfigured = cfg.GetVelocityTrapezoidalProfileInUse();
  const wpi::units::second_t loopTime = cfg.GetClosedLoopControlPeriod().value_or(20_ms);
  const auto mechLower = cfg.GetMechanismLowerLimit();
  const auto mechUpper = cfg.GetMechanismUpperLimit();
  const auto armFF = cfg.GetArmFeedforward(m_slot);
  const auto elevFF = cfg.GetElevatorFeedforward(m_slot);
  const auto simpleFF = cfg.GetSimpleFeedforward(m_slot);
  const auto tempCutoff = cfg.GetTemperatureCutoff();
  const auto maxV = cfg.GetClosedLoopControllerMaximumVoltage();

  // Profiles in the units of the closed-loop controller (rotations, or meters when linear).
  auto trapProfile = cfg.GetTrapezoidProfile();
  auto expoProfile = cfg.GetExponentialProfile();
  auto linTrapProfile = cfg.GetLinearTrapezoidProfile();
  auto linExpoProfile = cfg.GetLinearExponentialProfile();
  const bool hasExpo = linear ? linExpoProfile.has_value() : expoProfile.has_value();
  const bool hasTrap = linear ? linTrapProfile.has_value() : trapProfile.has_value();

  // With continuous wrapping an absolute encoder's angle jumps by a whole wrapping range at the
  // wrapping point; use the equivalent angle nearest the last one so the profile state and the
  // LQR estimate, which carry over from the last loop, do not see the jump.
  const bool wrapping = cfg.GetContinuousWrapping().has_value();
  const wpi::units::turn_t mechanismPosition =
      wrapping && m_lastClosedLoopMechanismPosition
          ? cfg.GetContinuousWrappingSetpoint(GetMechanismPosition(),
                                              *m_lastClosedLoopMechanismPosition)
          : wpi::units::turn_t{GetMechanismPosition()};
  m_lastClosedLoopMechanismPosition = mechanismPosition;

  // Clamp setpoint to limits
  if (m_setpointPosition.has_value()) {
    if (mechLower && *m_setpointPosition < *mechLower) {
      WPILIB_ReportWarning(
          "[WARNING] Setpoint is lower than Mechanism {} lower limit, changing setpoint to lower "
          "limit.",
          GetName());
      m_setpointPosition = mechLower;
    }
    if (mechUpper && *m_setpointPosition > *mechUpper) {
      WPILIB_ReportWarning(
          "[WARNING] Setpoint is higher than Mechanism {} upper limit, changing setpoint to upper "
          "limit.",
          GetName());
      m_setpointPosition = mechUpper;
    }
  }

  // With continuous wrapping, go to the equivalent setpoint nearest the current position so the
  // profile and controller take the short way around.
  std::optional<wpi::units::turn_t> wrappedSetpoint;
  if (m_setpointPosition) {
    wrappedSetpoint = wrapping
                          ? cfg.GetContinuousWrappingSetpoint(*m_setpointPosition, mechanismPosition)
                          : *m_setpointPosition;
  }

  // Current and next profile states, as {position, velocity} in controller units.  For a
  // velocity profile the "position" is the velocity setpoint.
  struct ProfileState {
    double position{0.0};
    double velocity{0.0};
  };
  ProfileState nextTrap{};
  ProfileState nextExpo{};
  bool velocityTrapezoidalProfile = false;

  auto calculateTrap = [&](ProfileState current, double goal) {
    if (linear) {
      using P = wpi::math::TrapezoidProfile<wpi::units::meters>;
      auto st = m_linearTrapState.value_or(P::State{wpi::units::meter_t{current.position},
                                                     wpi::units::meters_per_second_t{current.velocity}});
      auto n = linTrapProfile->Calculate(loopTime, st, P::State{wpi::units::meter_t{goal}, {}});
      return ProfileState{n.position.value(), n.velocity.value()};
    }
    using P = wpi::math::TrapezoidProfile<wpi::units::turns>;
    auto st = m_trapState.value_or(P::State{wpi::units::turn_t{current.position},
                                            wpi::units::turns_per_second_t{current.velocity}});
    auto n = trapProfile->Calculate(loopTime, st, P::State{wpi::units::turn_t{goal}, {}});
    return ProfileState{n.position.value(), n.velocity.value()};
  };

  if (m_setpointPosition) {
    double setpoint = wrappedSetpoint->value();
    double position = mechanismPosition.value();
    double velocity = wpi::units::turns_per_second_t{GetMechanismVelocity()}.value();
    if (linear) {
      position = GetMeasurementPosition().value();
      velocity = GetMeasurementVelocity().value();
      setpoint = cfg.ConvertFromMechanism(*m_setpointPosition).value();
    }
    if (hasExpo) {
      if (linear) {
        using P = wpi::math::ExponentialProfile<wpi::units::meters, wpi::units::volts>;
        auto st = m_linearExpoState.value_or(
            P::State{wpi::units::meter_t{position}, wpi::units::meters_per_second_t{velocity}});
        auto n = linExpoProfile->Calculate(loopTime, st, P::State{wpi::units::meter_t{setpoint}, {}});
        nextExpo = {n.position.value(), n.velocity.value()};
      } else {
        using P = wpi::math::ExponentialProfile<wpi::units::turns, wpi::units::volts>;
        auto st = m_expoState.value_or(
            P::State{wpi::units::turn_t{position}, wpi::units::turns_per_second_t{velocity}});
        auto n = expoProfile->Calculate(loopTime, st, P::State{wpi::units::turn_t{setpoint}, {}});
        nextExpo = {n.position.value(), n.velocity.value()};
      }
    } else if (hasTrap && !velocityProfileConfigured) {
      nextTrap = calculateTrap({position, velocity}, setpoint);
    }
  } else if (m_setpointVelocity) {
    double setpoint = m_setpointVelocity->value();
    double velocity = wpi::units::turns_per_second_t{GetMechanismVelocity()}.value();
    if (linear) {
      velocity = GetMeasurementVelocity().value();
      setpoint = cfg.ConvertFromMechanism(*m_setpointVelocity).value();
    }
    if (hasTrap && velocityProfileConfigured) {
      nextTrap = calculateTrap({velocity, 0.0}, setpoint);
      velocityTrapezoidalProfile = true;
    }
  }

  // PID / LQR output
  double pidOutput = 0.0;
  double ffOutput = 0.0;

  if (m_setpointPosition) {
    double measured = mechanismPosition.value();
    double setpoint = wrappedSetpoint->value();
    double velProfile = 0.0;
    if (linear) {
      measured = GetMeasurementPosition().value();
      setpoint = cfg.ConvertFromMechanism(*m_setpointPosition).value();
    }
    if (hasExpo) {
      setpoint = nextExpo.position;
      velProfile = nextExpo.velocity;
    } else if (hasTrap && !velocityProfileConfigured) {
      setpoint = nextTrap.position;
      velProfile = nextTrap.velocity;
    }
    if (m_pid) pidOutput = m_pid->Calculate(measured, setpoint);
    if (m_lqr) {
      if (!linear) {
        pidOutput =
            m_lqr
                ->Calculate(wpi::units::radian_t{wpi::units::turn_t{measured}},
                            wpi::units::radian_t{wpi::units::turn_t{setpoint}},
                            wpi::units::radians_per_second_t{
                                wpi::units::turns_per_second_t{velProfile}})
                .value();
      } else {
        pidOutput = m_lqr
                        ->Calculate(wpi::units::meter_t{measured}, wpi::units::meter_t{setpoint},
                                    wpi::units::meters_per_second_t{velProfile})
                        .value();
      }
    }
  } else if (m_setpointVelocity) {
    double measured = wpi::units::turns_per_second_t{GetMechanismVelocity()}.value();
    double setpoint = m_setpointVelocity->value();
    if (linear) {
      measured = GetMeasurementVelocity().value();
      setpoint = cfg.ConvertFromMechanism(*m_setpointVelocity).value();
    }
    if (velocityTrapezoidalProfile) setpoint = nextTrap.position;
    if (m_pid) pidOutput = m_pid->Calculate(measured, setpoint);
    if (m_lqr) {
      if (!linear) {
        pidOutput = m_lqr
                        ->Calculate(wpi::units::radians_per_second_t{
                                        wpi::units::turns_per_second_t{measured}},
                                    wpi::units::radians_per_second_t{
                                        wpi::units::turns_per_second_t{setpoint}})
                        .value();
      } else {
        pidOutput = m_lqr
                        ->Calculate(wpi::units::meters_per_second_t{measured},
                                    wpi::units::meters_per_second_t{setpoint})
                        .value();
      }
    }
  }

  // Feedforward.  Profile states are read before they are advanced below, so the current and
  // next setpoint velocities differ and the kA term is applied.
  if (m_setpointPosition || m_setpointVelocity) {
    const bool profiled = hasExpo || hasTrap;
    // Profile velocities in controller units (rotations/s, or meters/s when linear).
    double curVel = 0.0;
    if (hasTrap) {
      curVel = linear ? (m_linearTrapState ? m_linearTrapState->velocity.value() : 0.0)
                      : (m_trapState ? m_trapState->velocity.value() : 0.0);
    } else if (hasExpo) {
      curVel = linear ? (m_linearExpoState ? m_linearExpoState->velocity.value() : 0.0)
                      : (m_expoState ? m_expoState->velocity.value() : 0.0);
    }
    const double nextVel = hasTrap ? nextTrap.velocity : (hasExpo ? nextExpo.velocity : 0.0);
    // Next velocity setpoint when not position-profiled; for a velocity profile its "position".
    double nextVelocitySetpoint =
        velocityTrapezoidalProfile
            ? nextTrap.position
            : (linear && m_setpointVelocity
                   ? cfg.ConvertFromMechanism(*m_setpointVelocity).value()
                   : m_setpointVelocity.value_or(wpi::units::turns_per_second_t{0}).value());
    // Rotational feedforwards take rotations; convert linear controller units back.
    auto toTurns = [&](double v) {
      return linear ? cfg.ConvertToMechanism(wpi::units::meters_per_second_t{v}).value() : v;
    };

    if (armFF) {
      if (profiled && !velocityTrapezoidalProfile) {
        ffOutput = armFF
                       ->Calculate(wpi::units::radian_t{mechanismPosition},
                                   wpi::units::radians_per_second_t{
                                       wpi::units::turns_per_second_t{toTurns(curVel)}},
                                   wpi::units::radians_per_second_t{
                                       wpi::units::turns_per_second_t{toTurns(nextVel)}})
                       .value();
      } else {
        ffOutput = armFF
                       ->Calculate(wpi::units::radian_t{mechanismPosition},
                                   wpi::units::radians_per_second_t{GetMechanismVelocity()},
                                   wpi::units::radians_per_second_t{wpi::units::turns_per_second_t{
                                       toTurns(nextVelocitySetpoint)}})
                       .value();
      }
    }

    if (elevFF) {
      if (profiled && m_setpointPosition && !velocityTrapezoidalProfile) {
        ffOutput = elevFF
                       ->Calculate(wpi::units::meters_per_second_t{curVel},
                                   wpi::units::meters_per_second_t{nextVel})
                       .value();
      } else {
        ffOutput =
            elevFF->Calculate(GetMeasurementVelocity(), wpi::units::meters_per_second_t{0}).value();
      }
    }

    if (simpleFF) {
      if (profiled && !velocityTrapezoidalProfile) {
        ffOutput = simpleFF
                       ->Calculate(wpi::units::turns_per_second_t{toTurns(curVel)},
                                   wpi::units::turns_per_second_t{toTurns(nextVel)})
                       .value();
      } else {
        ffOutput = simpleFF
                       ->Calculate(wpi::units::turns_per_second_t{GetMechanismVelocity()},
                                   wpi::units::turns_per_second_t{toTurns(nextVelocitySetpoint)})
                       .value();
      }
    }
  }

  // Save the advanced profile states.
  if (hasExpo && m_setpointPosition) {
    if (linear) {
      m_linearExpoState = wpi::math::ExponentialProfile<wpi::units::meters, wpi::units::volts>::State{
          wpi::units::meter_t{nextExpo.position}, wpi::units::meters_per_second_t{nextExpo.velocity}};
    } else {
      m_expoState = wpi::math::ExponentialProfile<wpi::units::turns, wpi::units::volts>::State{
          wpi::units::turn_t{nextExpo.position}, wpi::units::turns_per_second_t{nextExpo.velocity}};
    }
  }
  if (hasTrap && ((m_setpointPosition && !velocityProfileConfigured) || velocityTrapezoidalProfile)) {
    if (linear) {
      m_linearTrapState = wpi::math::TrapezoidProfile<wpi::units::meters>::State{
          wpi::units::meter_t{nextTrap.position}, wpi::units::meters_per_second_t{nextTrap.velocity}};
    } else {
      m_trapState = wpi::math::TrapezoidProfile<wpi::units::turns>::State{
          wpi::units::turn_t{nextTrap.position}, wpi::units::turns_per_second_t{nextTrap.velocity}};
    }
  }

  // Feedforward force supplied to SetVelocity(velocity, force).
  if (m_setpointVelocity && m_setpointFeedforwardForce) {
    ffOutput += cfg.ConvertToVoltage(GetDCMotor(), *m_setpointFeedforwardForce).value();
  }

  // Boundary safety
  if (mechUpper && GetMechanismPosition() > *mechUpper && (pidOutput + ffOutput) > 0.0) {
    pidOutput = ffOutput = 0.0;
  }
  if (mechLower && GetMechanismPosition() < *mechLower && (pidOutput + ffOutput) < 0.0) {
    pidOutput = ffOutput = 0.0;
  }
  if (tempCutoff && GetTemperature() >= *tempCutoff) {
    pidOutput = ffOutput = 0.0;
  }

  double output = pidOutput + ffOutput;
  if (maxV) {
    output = std::clamp(output, -maxV->value(), maxV->value());
  }
  SetVoltage(wpi::units::volt_t{output});
}

// ---- Setpoints ------------------------------------------------------------

void SmartMotorController::SetVelocity(wpi::units::turns_per_second_t velocity,
                                       wpi::units::newton_t feedforwardForce) {
  SetVelocity(velocity);
  m_setpointFeedforwardForce = feedforwardForce;
}

void SmartMotorController::SetVelocity(wpi::units::meters_per_second_t velocity,
                                       wpi::units::newton_t feedforwardForce) {
  SetVelocity(m_config->ConvertToMechanism(velocity), feedforwardForce);
}

void SmartMotorController::SetMechanismGearing(const gearing::MechanismGearing& gearing) {
  m_config->WithMotorGearing(gearing);
  for (auto* f : m_looseFollowers) f->SetMechanismGearing(gearing);
}

void SmartMotorController::SetMechanismCircumference(wpi::units::meter_t circumference) {
  m_config->WithMechanismCircumference(circumference);
}

// ---- Telemetry ------------------------------------------------------------

SmartMotorController& SmartMotorController::WithTelemetry(
    telemetry::SmartMotorControllerTelemetryConfig config) {
  m_telemetryConfig = std::move(config);
  m_telemetryConfigExplicit = true;
  return *this;
}

void SmartMotorController::SetupTelemetry(std::shared_ptr<wpi::nt::NetworkTable> dataTable,
                                          std::shared_ptr<wpi::nt::NetworkTable> tuningTable) {
  if (m_parentTable) return;  // already set up
  m_parentTable = dataTable;
  if (!m_config->GetTelemetryName()) return;

  m_telemetryTable = dataTable->GetSubTable(GetName());
  m_tuningTable = tuningTable->GetSubTable(GetName());

  telemetry::SmartMotorControllerTelemetryConfig* telemetryConfig = &m_telemetryConfig;
  if (!m_telemetryConfigExplicit) {
    m_specifiedTelemetryConfig = m_config->GetSmartControllerTelemetryConfig();
    if (m_specifiedTelemetryConfig) {
      telemetryConfig = m_specifiedTelemetryConfig.get();
    } else {
      m_telemetryConfig.WithTelemetryVerbosity(
          m_config->GetVerbosity().value_or(SmartMotorControllerConfig::TelemetryVerbosity::HIGH));
    }
  }

  auto& dfields = telemetryConfig->GetDoubleFields(*this);
  auto& bfields = telemetryConfig->GetBoolFields(*this);
  m_telemetry.SetupTelemetry(*this, m_telemetryTable, m_tuningTable, dfields, bfields,
                             telemetryConfig->GetNT4Enabled(), telemetryConfig->GetDataLogName());
  UpdateTelemetry();
}

void SmartMotorController::SetupTelemetry() {
  auto inst = wpi::nt::NetworkTableInstance::GetDefault();
  SetupTelemetry(inst.GetTable("Mechanisms"), inst.GetTable("Tuning"));
  if (m_config->GetVerbosity().value_or(SmartMotorControllerConfig::TelemetryVerbosity::LOW) ==
          SmartMotorControllerConfig::TelemetryVerbosity::HIGH &&
      m_telemetryTable && !m_liveTuningRegistered && m_config->HasSubsystem()) {
    // Live Tuning command (one per subsystem, shared across all SMCs)
    auto* subsystem = m_config->GetSubsystem();
    SmartMotorControllerCommandRegistry::AddCommand("Live Tuning", subsystem,
                                                    [this] { ApplyTuningValues(); });
    // The registered callback captures this; drop the registration when this SMC is closed.
    AddCloseHook([subsystem] { SmartMotorControllerCommandRegistry::RemoveCommands(subsystem); });
    m_liveTuningRegistered = true;
  }
}

void SmartMotorController::UpdateTelemetry() {
  if (!m_config->GetVerbosity()) return;
  if (m_telemetryTable) {
    m_telemetry.Publish(*this);
  } else {
    SetupTelemetry();
  }
}

bool SmartMotorController::TuningEnabled() const { return m_telemetry.TuningEnabled(); }

void SmartMotorController::ApplyTuningValues() { m_telemetry.ApplyTuningValues(*this); }

void SmartMotorController::AddCloseHook(std::function<void()> hook) {
  m_closeHooks.push_back(std::move(hook));
}

telemetry::UnsupportedTelemetryFields SmartMotorController::GetUnsupportedTelemetryFields() {
  return {};  // base: no unsupported fields
}

SmartMotorController::ClosedLoopControllerSlot SmartMotorController::GetClosedLoopControllerSlot()
    const {
  return m_slot;
}

// ---- Misc -----------------------------------------------------------------

std::optional<wpi::units::turn_t> SmartMotorController::GetMechanismPositionSetpoint() const {
  return m_setpointPosition;
}

std::optional<wpi::units::turns_per_second_t> SmartMotorController::GetMechanismSetpointVelocity()
    const {
  return m_setpointVelocity;
}

std::optional<wpi::units::meter_t> SmartMotorController::GetMeasurementPositionSetpoint() const {
  if (!m_setpointPosition) return std::nullopt;
  if (!m_config->GetMechanismCircumference()) return std::nullopt;
  return m_config->ConvertFromMechanism(*m_setpointPosition);
}

std::optional<wpi::units::meters_per_second_t>
SmartMotorController::GetMeasurementSetpointVelocity() const {
  if (!m_setpointVelocity) return std::nullopt;
  if (!m_config->GetMechanismCircumference()) return std::nullopt;
  return m_config->ConvertFromMechanism(*m_setpointVelocity);
}

std::optional<wpi::units::newton_t> SmartMotorController::GetSetpointFeedforwardForce() const {
  return m_setpointFeedforwardForce;
}

std::string SmartMotorController::GetName() const {
  if (!m_config) return "SmartMotorController";
  return m_config->GetTelemetryName().value_or("SmartMotorController");
}

bool SmartMotorController::IsMotor(const wpi::math::DCMotor& a, const wpi::math::DCMotor& b) const {
  return a.stallTorque == b.stallTorque && a.stallCurrent == b.stallCurrent &&
         a.freeCurrent == b.freeCurrent && a.freeSpeed == b.freeSpeed && a.Kt == b.Kt &&
         a.Kv == b.Kv && a.nominalVoltage == b.nominalVoltage;
}

void SmartMotorController::CheckConfigSafety() {
  wpi::math::DCMotor neo550 = wpi::math::DCMotor::NEO550(1);
  if (IsMotor(GetDCMotor(), neo550)) {
    auto limit = m_config->GetStatorStallCurrentLimit();
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
  m_closedLoopControllerRunning = false;
  if (m_closed) return;
  m_closed = true;
  m_telemetry.Close();
  simulation::BatterySim::RemoveCurrent(BatterySimKey());
  auto hooks = std::move(m_closeHooks);
  m_closeHooks.clear();
  for (auto& hook : hooks) hook();
}

// ---- SimSupplier ------------------------------------------------------------

void SmartMotorController::SetSimSupplier(std::shared_ptr<SimSupplier> supplier) {
  m_simSupplier = std::move(supplier);
}

SimSupplier* SmartMotorController::GetSimSupplier() const { return m_simSupplier.get(); }

// ---- Loosely coupled followers ----------------------------------------------

void SmartMotorController::LoadLooselyCoupledFollowers() {
  m_looseFollowers = m_config->GetLooselyCoupledFollowers();
}

void SmartMotorController::ConfigureSoftwarePID(const SmartMotorControllerConfig& config) {
  auto tolerance = config.GetClosedLoopTolerance();
  auto wrapMax = config.GetContinuousWrapping();
  auto wrapMin = config.GetContinuousWrappingMin();
  if (!m_pid) return;
  if (wrapMax && wrapMin) m_pid->EnableContinuousInput(wrapMin->value(), wrapMax->value());
  if (tolerance) {
    m_pid->SetTolerance(config.GetLinearClosedLoopControllerUse()
                            ? config.ConvertFromMechanism(*tolerance).value()
                            : tolerance->value());
  }
}

void SmartMotorController::ForwardPositionToFollowers(wpi::units::turn_t pos) {
  for (auto* f : m_looseFollowers) f->SetPosition(pos);
}

void SmartMotorController::ForwardPositionToFollowers(wpi::units::meter_t dist) {
  for (auto* f : m_looseFollowers) f->SetPosition(dist);
}

void SmartMotorController::ForwardVelocityToFollowers(wpi::units::turns_per_second_t vel) {
  for (auto* f : m_looseFollowers) f->SetVelocity(vel);
}

void SmartMotorController::ForwardVelocityToFollowers(wpi::units::meters_per_second_t vel) {
  for (auto* f : m_looseFollowers) f->SetVelocity(vel);
}

}  // namespace yams::motorcontrollers
