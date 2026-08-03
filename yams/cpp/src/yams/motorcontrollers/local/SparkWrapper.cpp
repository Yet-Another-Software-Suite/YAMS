// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/local/SparkWrapper.hpp"

#include <wpi/driverstation/Alert.hpp>
#include <wpi/framework/RobotBase.hpp>
#include <wpi/simulation/RoboRioSim.hpp>
#include <wpi/math/system/Models.hpp>
#include <rev/ClosedLoopTypes.h>
#include <rev/ConfigureTypes.h>
#include <wpi/units/moment_of_inertia.hpp>

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "yams/exceptions.hpp"
#include "yams/math/LQRController.hpp"
#include "yams/motorcontrollers/simulation/DCMotorSimSupplier.hpp"

using namespace rev::spark;

namespace yams::motorcontrollers::local {

SparkWrapper::SparkWrapper(SparkMax* spark, wpi::math::DCMotor motor, SmartMotorControllerConfig* cfg)
    : SmartMotorController(), m_motor(motor) {
  m_maxConfig.emplace();
  if (auto vc = cfg->GetVendorConfig(); vc.has_value()) {
    if (auto* p = std::any_cast<rev::spark::SparkMaxConfig>(&vc.value())) {
      m_maxConfig->Apply(*p);
    } else {
      throw std::invalid_argument(
          "SparkWrapper (SparkMax): WithVendorConfig requires a SparkMaxConfig");
    }
  }
  Init(spark, motor, cfg);
}

SparkWrapper::SparkWrapper(SparkFlex* spark, wpi::math::DCMotor motor, SmartMotorControllerConfig* cfg)
    : SmartMotorController(), m_motor(motor) {
  m_flexConfig.emplace();
  if (auto vc = cfg->GetVendorConfig(); vc.has_value()) {
    if (auto* p = std::any_cast<rev::spark::SparkFlexConfig>(&vc.value())) {
      m_flexConfig->Apply(*p);
    } else {
      throw std::invalid_argument(
          "SparkWrapper (SparkFlex): WithVendorConfig requires a SparkFlexConfig");
    }
  }
  Init(spark, motor, cfg);
}

void SparkWrapper::Init(SparkBase* spark, wpi::math::DCMotor motor, SmartMotorControllerConfig* cfg) {
  m_spark = spark;
  m_sparkPid = &spark->GetClosedLoopController();
  m_relEncoder = &spark->GetEncoder();
  m_config = cfg;
  m_config->WithSimMotor(motor);

  SetupSimulation();
  ApplyConfig(*m_config);
  CheckConfigSafety();
}

// ---- Configuration ----------------------------------------------------------

bool SparkWrapper::ApplyConfig(const SmartMotorControllerConfig& config) {
  // If the device is currently a follower, applying a new config (without a follow directive)
  // automatically clears follower mode, allowing it to be used as an independent master.
  m_config->ResetValidationCheck();

  auto doConfig = [&](SparkBaseConfig& sparkCfg) {
    sparkCfg.DisableFollowerMode();  // Disable follower from the Spark.
    if (auto inv = config.GetMotorInverted(); inv) sparkCfg.Inverted(*inv);
    sparkCfg.SetIdleMode(config.GetIdleMode() == SmartMotorControllerConfig::MotorMode::BRAKE
                             ? SparkBaseConfig::IdleMode::kBrake
                             : SparkBaseConfig::IdleMode::kCoast);

    if (auto r = config.GetOpenLoopRampRate(); r) sparkCfg.OpenLoopRampRate(r->value());
    if (auto r = config.GetClosedLoopRampRate(); r) sparkCfg.ClosedLoopRampRate(r->value());

    if (auto stator = config.GetStatorCurrentLimit(); stator)
      sparkCfg.SmartCurrentLimit(static_cast<int>(stator->value()));
    if (auto supply = config.GetSupplyCurrentLimit(); supply)
      sparkCfg.SecondaryCurrentLimit(supply->value());

    // No software temperature cutoff on Spark; consume option for validation
    config.GetTemperatureCutoff();

    // Closed-loop max output percentage
    if (auto maxV = config.GetClosedLoopControllerMaximumVoltage(); maxV) {
      double pct = maxV->value() / 12.0;
      sparkCfg.closedLoop.MaxOutput(pct).MinOutput(-pct);
    }

    // Base (relative) encoder: conversion from motor rotations to mechanism turns.
    double convFactor = 1.0;
    if (auto& g = config.GetMotorGearing(); g) convFactor = g->GetRotorToMechanismRatio();
    sparkCfg.encoder.PositionConversionFactor(convFactor);
    sparkCfg.encoder.VelocityConversionFactor(convFactor / 60.0);

    using Slot = SmartMotorControllerConfig::ClosedLoopControllerSlot;
    for (auto slot : {Slot::SLOT_0, Slot::SLOT_1, Slot::SLOT_2, Slot::SLOT_3}) {
      auto gains = config.GetSlotGains(slot);
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

    if (config.HasTrapezoidProfile()) {
      // Angular profile supplies velocity directly in TPS.  Linear profiles store
      // velocity in m/s; convert to TPS using the mechanism circumference.
      auto cruiseVelTps = config.GetTrapMaxVelocityTurns();
      auto cruiseAccTps = config.GetTrapMaxAccelTurns();
      if (!cruiseVelTps) {
        if (auto linVel = config.GetTrapMaxVelocityLinear(); linVel)
          if (auto circ = config.GetMechanismCircumference(); circ && circ->value() != 0.0)
            cruiseVelTps = wpi::units::turns_per_second_t{linVel->value() / circ->value()};
      }
      if (!cruiseAccTps) {
        if (auto linAcc = config.GetTrapMaxAccelLinear(); linAcc)
          if (auto circ = config.GetMechanismCircumference(); circ && circ->value() != 0.0)
            cruiseAccTps = wpi::units::turns_per_second_squared_t{linAcc->value() / circ->value()};
      }
      if (cruiseVelTps) sparkCfg.closedLoop.maxMotion.CruiseVelocity(cruiseVelTps->value());
      if (cruiseAccTps) sparkCfg.closedLoop.maxMotion.MaxAcceleration(cruiseAccTps->value());
      m_positionControlType = config.GetVelocityTrapezoidalProfileInUse()
                                  ? SparkLowLevel::ControlType::kVelocity
                                  : SparkLowLevel::ControlType::kMAXMotionPositionControl;
      m_velocityControlType = SparkLowLevel::ControlType::kMAXMotionVelocityControl;
    }

    // Exponential profiles are not natively supported by SPARK hardware; the option is consumed
    // here to satisfy validation tracking — actual profile execution uses the software controller
    // started below.
    config.HasExponentialProfile();

    if (auto inv = config.GetEncoderInverted(); inv) sparkCfg.encoder.Inverted(*inv);

    // External (absolute) encoder configuration.
    if (auto enc = config.GetExternalEncoder(); enc.has_value()) {
      auto* absPtr = std::any_cast<rev::spark::SparkAbsoluteEncoder*>(&enc.value());
      if (!absPtr || !*absPtr)
        throw std::invalid_argument(
            "SparkWrapper: WithExternalEncoder requires a rev::spark::SparkAbsoluteEncoder*");
      m_absEncoder = *absPtr;

      // Conversion factor: gearing-derived, overridden by direct CF if set.
      double absCF = config.GetExternalEncoderGearing()
                         .value_or(gearing::MechanismGearing::kOne)
                         .GetRotorToMechanismRatio();
      if (auto cf = config.GetExternalEncoderConversionFactor(); cf) absCF = *cf;
      sparkCfg.absoluteEncoder.PositionConversionFactor(absCF).VelocityConversionFactor(absCF /
                                                                                        60.0);

      if (auto inv = config.GetExternalEncoderInverted(); inv)
        sparkCfg.absoluteEncoder.Inverted(*inv);

      if (config.GetUseExternalFeedback())
        sparkCfg.closedLoop.SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder);

      if (auto offset = config.GetExternalEncoderZeroOffset(); offset)
        sparkCfg.absoluteEncoder.ZeroOffset(offset->value());

      if (auto dp = config.GetExternalEncoderDiscontinuityPoint(); dp)
        sparkCfg.absoluteEncoder.ZeroCentered(*dp == wpi::units::turn_t{0.5});

    } else {
      // Validate: encoder-specific options require an encoder to be attached.
      if (config.GetExternalEncoderZeroOffset().has_value())
        throw exceptions::SmartMotorControllerConfigurationException(
            "Zero offset is only available for external encoders",
            "Zero offset could not be applied", "WithExternalEncoder(encoder)");
      if (config.GetExternalEncoderDiscontinuityPoint().has_value())
        throw exceptions::SmartMotorControllerConfigurationException(
            "External encoder discontinuity point is only available for external encoders",
            "Discontinuity point could not be applied", "WithExternalEncoder(encoder)");
      if (config.GetExternalEncoderInverted().has_value())
        throw exceptions::SmartMotorControllerConfigurationException(
            "External encoder cannot be inverted when no external encoder is attached",
            "External encoder inversion could not be applied",
            "WithExternalEncoder(encoder) + WithExternalEncoderInverted(bool)");
      if (config.GetExternalEncoderGearing().has_value())
        throw exceptions::SmartMotorControllerConfigurationException(
            "External encoder gearing requires an external encoder to be attached",
            "External encoder gearing could not be applied",
            "WithExternalEncoder(encoder) + WithExternalEncoderGearing(...)");
      // Consume remaining external encoder options for validation tracking
      config.GetUseExternalFeedback();
    }

    bool isClosedLoop = config.GetMotorControllerMode() == ControlMode::CLOSED_LOOP;
    if (auto lower = config.GetMechanismLowerLimit()) {
      sparkCfg.softLimit.ReverseSoftLimit(lower->value()).ReverseSoftLimitEnabled(isClosedLoop);
    }
    if (auto upper = config.GetMechanismUpperLimit()) {
      sparkCfg.softLimit.ForwardSoftLimit(upper->value()).ForwardSoftLimitEnabled(isClosedLoop);
    }
    if (auto wrapMax = config.GetContinuousWrapping(); wrapMax) {
      if (auto wrapMin = config.GetContinuousWrappingMin(); wrapMin)
        sparkCfg.closedLoop.PositionWrappingInputRange(wrapMin->value(), wrapMax->value())
            .PositionWrappingEnabled(true);
      else
        sparkCfg.closedLoop.PositionWrappingMaxInput(wrapMax->value())
            .PositionWrappingEnabled(true);
    }
  };

  if (config.GetVendorControlRequest().has_value())
    throw std::runtime_error(
        "SparkWrapper does not support custom vendor control requests (WithVendorControlRequest).");

  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();

  // Load LQR from the active gain slot so IterateClosedLoopController can use it.
  auto gains = config.GetSlotGains(m_slot);
  if (gains.lqr) {
    m_lqr = math::LQRController{*gains.lqr};
  } else {
    m_lqr.reset();
  }

  // Load software PID (used by IterateClosedLoopController when no LQR is present).
  if (gains.kP != 0.0 || gains.kI != 0.0 || gains.kD != 0.0) {
    m_pid = wpi::math::PIDController{gains.kP, gains.kI, gains.kD};
  } else {
    m_pid.reset();
  }

  // SPARK hardware has no native exponential profile or LQR support.  When either is configured,
  // the RoboRIO runs the closed-loop controller via a Notifier and sends voltage commands to the
  // Spark.  SetPosition/SetVelocity are gated by m_closedLoopControllerRunning so they do not
  // forward raw setpoints to the Spark hardware controller while the notifier is active.
  bool needsRioController = config.HasExponentialProfile() || m_lqr.has_value();
  if (needsRioController) {
    int deviceId = m_spark->GetDeviceId();
    std::string alertText =
        "[YAMS] Spark(" + std::to_string(deviceId) +
        ") is running closed-loop control on the SystemCore (exponential profile or LQR active). "
        "Gains are not consistent with Spark hardware PID and control runs at a lower frequency.";
    m_rioControllerAlert.emplace(alertText, wpi::Alert::Level::MEDIUM);
    m_rioControllerAlert->Set(true);

    std::fprintf(stderr, "====== Spark(%d) Using RIO Closed Loop Controller ======\n", deviceId);

    if (m_closedLoopControllerThread) {
      StopClosedLoopController();
      m_closedLoopControllerThread.reset();
    }
    m_closedLoopControllerThread =
        std::make_unique<wpi::Notifier>([this] { IterateClosedLoopController(); });
    if (auto name = config.GetTelemetryName(); name) {
      m_closedLoopControllerThread->SetName(*name);
    }
    if (config.GetMotorControllerMode() == ControlMode::CLOSED_LOOP) {
      StartClosedLoopController();
    }
  } else if (m_closedLoopControllerThread) {
    // Config no longer requires a software controller; tear it down.
    StopClosedLoopController();
    m_closedLoopControllerThread.reset();
    if (m_rioControllerAlert) m_rioControllerAlert->Set(false);
  }

  if (auto startPos = config.GetStartingPosition()) {
    m_relEncoder->SetPosition(startPos->value());
  } else if (m_absEncoder) {
    m_relEncoder->SetPosition(m_absEncoder->GetPosition().Get());
  }

  // Tightly coupled followers — accept SparkMax and SparkFlex only
  for (auto& [hw, inverted] : config.GetFollowers()) {
    if (auto* mx = std::any_cast<rev::spark::SparkMax*>(&hw)) {
      SparkMaxConfig followCfg;
      followCfg.Follow(*m_spark, inverted);
      (*mx)->Configure(followCfg, rev::ResetMode::kNoResetSafeParameters,
                       rev::PersistMode::kPersistParameters);
    } else if (auto* fx = std::any_cast<rev::spark::SparkFlex*>(&hw)) {
      SparkFlexConfig followCfg;
      followCfg.Follow(*m_spark, inverted);
      (*fx)->Configure(followCfg, rev::ResetMode::kNoResetSafeParameters,
                       rev::PersistMode::kPersistParameters);
    } else {
      std::cerr << "[YAMS] SparkWrapper: follower is not a SparkMax or SparkFlex and will be "
                   "ignored.\n";
    }
  }
  LoadLooselyCoupledFollowers();

  config.ValidateBasicOptions();
  config.ValidateExternalEncoderOptions();
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
  if (!wpi::RobotBase::IsSimulation()) return;

  if (!m_sparkSim.has_value()) {
    auto simMotor = m_config->GetSimMotor();
    auto& gearing = m_config->GetMotorGearing();
    if (!simMotor || !gearing) return;

    auto plant = wpi::math::Models::SingleJointedArmFromPhysicalConstants(*simMotor, m_config->GetMOI(),
                                                    gearing->GetMechanismToRotorRatio());
    m_motorSim.emplace(plant, *simMotor);

    auto period = m_config->GetClosedLoopControlPeriod().value_or(20_ms);
    SetSimSupplier(std::make_shared<simulation::DCMotorSimSupplier>(
        *m_motorSim, [this]() { return GetDutyCycle(); }, *gearing, period));

    m_sparkSim.emplace(m_spark, &m_motor);

    if (m_maxConfig)
      m_relEncoderSim.emplace(static_cast<rev::spark::SparkMax*>(m_spark));
    else if (m_flexConfig)
      m_relEncoderSim.emplace(static_cast<rev::spark::SparkFlex*>(m_spark));

    if (m_config->GetExternalEncoder().has_value()) {
      if (m_maxConfig)
        m_absEncoderSim.emplace(static_cast<rev::spark::SparkMax*>(m_spark));
      else if (m_flexConfig)
        m_absEncoderSim.emplace(static_cast<rev::spark::SparkFlex*>(m_spark));
    }
  }

  if (auto startPos = m_config->GetStartingPosition()) {
    m_sparkSim->SetPosition(startPos->value());
    if (m_relEncoderSim) m_relEncoderSim->SetPosition(startPos->value());
    if (m_absEncoderSim) m_absEncoderSim->SetPosition(startPos->value());
  }
  if (auto offset = m_config->GetExternalEncoderZeroOffset()) {
    if (m_absEncoderSim) m_absEncoderSim->SetZeroOffset(offset->value());
  }
}

void SparkWrapper::SimIterate() {
  // if (!wpi::RobotBase::IsSimulation() || !m_simSupplier || !m_sparkSim) return;
  if (m_simSupplier) {
    if (m_simSupplier->IsWatchdogExpired()) {
      m_simSupplier->UpdateSim();
    }
    wpi::units::second_t dt = m_config->GetClosedLoopControlPeriod().value_or(20_ms);
    wpi::units::turns_per_second_t mechVelRps = m_simSupplier->GetMechanismVelocity();
    double vbus = m_simSupplier->GetMechanismSupplyVoltage().value();
    m_sparkSim->iterate(mechVelRps.value(), vbus, dt.value());

    if (m_relEncoderSim) m_relEncoderSim->iterate(mechVelRps.value(), dt.value());

    if (m_absEncoderSim) m_absEncoderSim->iterate(mechVelRps.value(), dt.value());
  }
}

// ---- Encoder sync -----------------------------------------------------------

void SparkWrapper::SeedRelativeEncoder() {
  if (m_absEncoder) m_relEncoder->SetPosition(m_absEncoder->GetPosition().Get());
}
void SparkWrapper::SynchronizeRelativeEncoder() {}

// ---- Open-loop outputs ------------------------------------------------------

void SparkWrapper::SetDutyCycle(double dc) { m_spark->SetThrottle(dc); }
double SparkWrapper::GetDutyCycle() {
  return m_sparkSim ? m_sparkSim.value().GetAppliedOutput() : m_spark->GetAppliedOutput().Get();
}

void SparkWrapper::SetVoltage(wpi::units::volt_t voltage) { m_spark->SetVoltage(voltage); }

wpi::units::volt_t SparkWrapper::GetVoltage() {
  if (m_simSupplier) return m_simSupplier->GetMechanismStatorVoltage();
  return wpi::units::volt_t{m_spark->GetAppliedOutput().Get() * m_spark->GetBusVoltage().Get()};
}

// ---- Closed-loop setpoints --------------------------------------------------

void SparkWrapper::SetPosition(wpi::units::turn_t angle) {
  m_setpointPosition = angle;
  if (m_config->GetMotorControllerMode() == ControlMode::CLOSED_LOOP &&
      !m_closedLoopControllerRunning)
    m_sparkPid->SetSetpoint(angle.value(), m_positionControlType,
                            static_cast<ClosedLoopSlot>(m_revSlot));
  ForwardPositionToFollowers(angle);
}

void SparkWrapper::SetPosition(wpi::units::meter_t distance) {
  if (auto circ = m_config->GetMechanismCircumference(); circ) {
    SetPosition(wpi::units::turn_t{distance.value() / circ->value()});
  } else {
    ForwardPositionToFollowers(distance);
  }
}

void SparkWrapper::SetVelocity(wpi::units::turns_per_second_t velocity) {
  m_setpointVelocity = velocity;
  if (m_config->GetMotorControllerMode() == ControlMode::CLOSED_LOOP &&
      !m_closedLoopControllerRunning)
    m_sparkPid->SetSetpoint(velocity.value(), m_velocityControlType,
                            static_cast<ClosedLoopSlot>(m_revSlot));
  ForwardVelocityToFollowers(velocity);
}

void SparkWrapper::SetVelocity(wpi::units::meters_per_second_t velocity) {
  if (auto circ = m_config->GetMechanismCircumference(); circ) {
    SetVelocity(wpi::units::turns_per_second_t{velocity.value() / circ->value()});
  } else {
    ForwardVelocityToFollowers(velocity);
  }
}

// ---- Encoder writes ---------------------------------------------------------

void SparkWrapper::SetEncoderPosition(wpi::units::turn_t angle) {
  m_relEncoder->SetPosition(angle.value());
  if (m_relEncoderSim) m_relEncoderSim->SetPosition(angle.value());
  if (m_absEncoderSim) m_absEncoderSim->SetPosition(angle.value());
  if (m_simSupplier) m_simSupplier->SetMechanismPosition(angle);
}
void SparkWrapper::SetEncoderPosition(wpi::units::meter_t distance) {
  if (auto circ = m_config->GetMechanismCircumference(); circ)
    SetEncoderPosition(wpi::units::turn_t{distance.value() / circ->value()});
}
void SparkWrapper::SetEncoderVelocity(wpi::units::turns_per_second_t) {}
void SparkWrapper::SetEncoderVelocity(wpi::units::meters_per_second_t) {}

// ---- Encoder reads ----------------------------------------------------------

wpi::units::turn_t SparkWrapper::GetMechanismPosition() {
  return wpi::units::turn_t{m_relEncoder->GetPosition().Get()};
}
wpi::units::turns_per_second_t SparkWrapper::GetMechanismVelocity() {
  return wpi::units::turns_per_second_t{m_relEncoder->GetVelocity().Get()};
}
wpi::units::turns_per_second_squared_t SparkWrapper::GetMechanismAcceleration() {
  return wpi::units::turns_per_second_squared_t{
      m_accelFilter.Derivative(GetMechanismVelocity().value())};
}
wpi::units::turn_t SparkWrapper::GetRotorPosition() {
  auto& g = m_config->GetMotorGearing();
  return wpi::units::turn_t{m_relEncoder->GetPosition().Get() * (g ? g->GetMechanismToRotorRatio() : 1.0)};
}
wpi::units::turns_per_second_t SparkWrapper::GetRotorVelocity() {
  auto& g = m_config->GetMotorGearing();
  return wpi::units::turns_per_second_t{m_relEncoder->GetVelocity().Get() *
                                   (g ? g->GetMechanismToRotorRatio() : 1.0)};
}

wpi::units::meter_t SparkWrapper::GetMeasurementPosition() {
  auto circ = m_config->GetMechanismCircumference().value_or(1.0_m);
  return wpi::units::meter_t{GetMechanismPosition().value() * circ.value()};
}
wpi::units::meters_per_second_t SparkWrapper::GetMeasurementVelocity() {
  auto circ = m_config->GetMechanismCircumference().value_or(1.0_m);
  return wpi::units::meters_per_second_t{GetMechanismVelocity().value() * circ.value()};
}
wpi::units::meters_per_second_squared_t SparkWrapper::GetMeasurementAcceleration() {
  auto circ = m_config->GetMechanismCircumference().value_or(1.0_m);
  return wpi::units::meters_per_second_squared_t{GetMechanismAcceleration().value() * circ.value()};
}

std::optional<wpi::units::degree_t> SparkWrapper::GetExternalEncoderPosition() {
  if (m_absEncoder) return wpi::units::degree_t{m_absEncoder->GetPosition().Get() * 360.0};
  return std::nullopt;
}
std::optional<wpi::units::degrees_per_second_t> SparkWrapper::GetExternalEncoderVelocity() {
  if (m_absEncoder) return wpi::units::degrees_per_second_t{m_absEncoder->GetVelocity().Get() * 360.0};
  return std::nullopt;
}

// ---- Motor status -----------------------------------------------------------

std::optional<wpi::units::ampere_t> SparkWrapper::GetSupplyCurrent() {
  return wpi::units::ampere_t{m_spark->GetOutputCurrent().Get()};
}
wpi::units::ampere_t SparkWrapper::GetStatorCurrent() {
  return wpi::units::ampere_t{m_spark->GetOutputCurrent().Get()};
}
wpi::units::celsius_t SparkWrapper::GetTemperature() {
  return wpi::units::celsius_t{m_spark->GetMotorTemperature().Get()};
}
wpi::math::DCMotor SparkWrapper::GetDCMotor() { return m_motor; }

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
    if (m_config->GetArmFeedforward(m_slot))
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
    if (m_config->GetArmFeedforward(m_slot))
      cfg.closedLoop.feedForward.kCos(kG, static_cast<ClosedLoopSlot>(m_revSlot));
    else if (m_config->GetElevatorFeedforward(m_slot))
      cfg.closedLoop.feedForward.kG(kG, static_cast<ClosedLoopSlot>(m_revSlot));
  };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetStatorCurrentLimit(wpi::units::ampere_t limit) {
  auto doConfig = [&](SparkBaseConfig& cfg) {
    cfg.SmartCurrentLimit(static_cast<int>(limit.value()));
  };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetSupplyCurrentLimit(wpi::units::ampere_t limit) {
  auto doConfig = [&](SparkBaseConfig& cfg) { cfg.SecondaryCurrentLimit(limit.value()); };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetClosedLoopRampRate(wpi::units::second_t r) {
  auto doConfig = [&](SparkBaseConfig& cfg) { cfg.ClosedLoopRampRate(r.value()); };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetOpenLoopRampRate(wpi::units::second_t r) {
  auto doConfig = [&](SparkBaseConfig& cfg) { cfg.OpenLoopRampRate(r.value()); };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetMechanismUpperLimit(wpi::units::turn_t upper) {
  if (auto lower = m_config->GetMechanismLowerLimit()) m_config->WithMechanismLimits(*lower, upper);
  auto doConfig = [&](SparkBaseConfig& cfg) { cfg.softLimit.ForwardSoftLimit(upper.value()); };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}
void SparkWrapper::SetMechanismLowerLimit(wpi::units::turn_t lower) {
  if (auto upper = m_config->GetMechanismUpperLimit()) m_config->WithMechanismLimits(lower, *upper);
  auto doConfig = [&](SparkBaseConfig& cfg) { cfg.softLimit.ReverseSoftLimit(lower.value()); };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}
void SparkWrapper::SetMechanismLimits(wpi::units::turn_t lower, wpi::units::turn_t upper) {
  m_config->WithMechanismLimits(lower, upper);
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
void SparkWrapper::SetMeasurementUpperLimit(wpi::units::meter_t upper) {
  auto circ = m_config->GetMechanismCircumference();
  auto lowerAngle = m_config->GetMechanismLowerLimit();
  if (!circ || !lowerAngle) return;
  m_config->WithMeasurementLimits(wpi::units::meter_t{lowerAngle->value() * circ->value()}, upper);
  wpi::units::turn_t upperTurns{upper.value() / circ->value()};
  auto doConfig = [&](SparkBaseConfig& cfg) { cfg.softLimit.ForwardSoftLimit(upperTurns.value()); };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}
void SparkWrapper::SetMeasurementLowerLimit(wpi::units::meter_t lower) {
  auto circ = m_config->GetMechanismCircumference();
  auto upperAngle = m_config->GetMechanismUpperLimit();
  if (!circ || !upperAngle) return;
  m_config->WithMeasurementLimits(lower, wpi::units::meter_t{upperAngle->value() * circ->value()});
  wpi::units::turn_t lowerTurns{lower.value() / circ->value()};
  auto doConfig = [&](SparkBaseConfig& cfg) { cfg.softLimit.ReverseSoftLimit(lowerTurns.value()); };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetMotionProfileMaxVelocity(wpi::units::turns_per_second_t vel) {
  auto doConfig = [&](SparkBaseConfig& cfg) {
    cfg.closedLoop.maxMotion.CruiseVelocity(vel.value());
  };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetMotionProfileMaxVelocity(wpi::units::meters_per_second_t vel) {
  if (auto circ = m_config->GetMechanismCircumference(); circ)
    SetMotionProfileMaxVelocity(wpi::units::turns_per_second_t{vel.value() / circ->value()});
}

void SparkWrapper::SetMotionProfileMaxAcceleration(wpi::units::turns_per_second_squared_t acc) {
  auto doConfig = [&](SparkBaseConfig& cfg) {
    cfg.closedLoop.maxMotion.MaxAcceleration(acc.value());
  };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetMotionProfileMaxAcceleration(wpi::units::meters_per_second_squared_t acc) {
  if (auto circ = m_config->GetMechanismCircumference(); circ)
    SetMotionProfileMaxAcceleration(wpi::units::turns_per_second_squared_t{acc.value() / circ->value()});
}

void SparkWrapper::SetMotionProfileMaxJerk(wpi::units::angular_jerk::turns_per_second_cubed_t maxJerk) {
  auto doConfig = [&](SparkBaseConfig& cfg) {
    cfg.closedLoop.maxMotion.MaxAcceleration(maxJerk.value());
  };
  if (m_maxConfig)
    doConfig(*m_maxConfig);
  else if (m_flexConfig)
    doConfig(*m_flexConfig);
  CommitConfig();
}

void SparkWrapper::SetExponentialProfile(std::optional<double> kV, std::optional<double> kA,
                                         std::optional<wpi::units::volt_t> maxInput) {
  if (!m_config->GetExponentialProfile()) return;

  double newKV = kV.value_or(m_config->GetExponentialProfileKV().value_or(0.0));
  double newKA = kA.value_or(m_config->GetExponentialProfileKA().value_or(0.0));
  wpi::units::volt_t newMaxInput =
      maxInput.value_or(m_config->GetExponentialProfileMaxInput().value_or(12_V));

  m_config->WithExponentialProfile(newKV, newKA, newMaxInput);

  for (auto* f : m_looseFollowers) f->SetExponentialProfile(kV, kA, maxInput);
}

void SparkWrapper::SetClosedLoopSlot(ClosedLoopControllerSlot slot) {
  m_slot = slot;
  m_revSlot = static_cast<int>(slot);
}

SmartMotorControllerConfig& SparkWrapper::GetConfig() { return *m_config; }
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

SparkWrapper::~SparkWrapper() {
  if (m_closedLoopControllerThread) {
    m_closedLoopControllerThread->Stop();
    m_closedLoopControllerThread.reset();
  }
  // delete m_spark;
}

}  // namespace yams::motorcontrollers::local
