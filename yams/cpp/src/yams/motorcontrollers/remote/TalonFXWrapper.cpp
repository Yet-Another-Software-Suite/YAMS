// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/remote/TalonFXWrapper.hpp"

#include <ctre/unit/pid_ff.h>
#include <frc/DriverStation.h>
#include <frc/Errors.h>
#include <frc/RobotBase.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include <units/angular_jerk.h>
#include <units/dimensionless.h>
#include <units/moment_of_inertia.h>

#include <cmath>
#include <cstdio>
#include <ctre/phoenix6/TalonFXS.hpp>
#include <memory>
#include <stdexcept>
#include <string>

#include "yams/exceptions.hpp"
#include "yams/math/LQRController.hpp"
#include "yams/motorcontrollers/simulation/DCMotorSimSupplier.hpp"

using namespace ctre::phoenix6;

namespace yams::motorcontrollers::remote {

TalonFXWrapper::TalonFXWrapper(hardware::TalonFX* talon, frc::DCMotor dcMotor,
                               SmartMotorControllerConfig* config)
    : SmartMotorController(), m_talon(talon), m_dcMotor(dcMotor) {
  m_config = config;
  m_config->WithSimMotor(dcMotor);
  if (auto vc = config->GetVendorConfig(); vc.has_value()) {
    if (auto* p = std::any_cast<configs::TalonFXConfiguration>(&vc.value())) {
      m_talonConfig = *p;
    } else {
      throw std::invalid_argument(
          "TalonFXWrapper: WithVendorConfig requires a TalonFXConfiguration");
    }
  }
  SetupSimulation();
  ApplyConfig(*m_config);
  CheckConfigSafety();
}

// ---- Configuration ----------------------------------------------------------

bool TalonFXWrapper::ApplyConfig(const SmartMotorControllerConfig& config) {
  // Do not update m_config pointer; read from the passed config parameter.
  // Reset validation on the stored pointer's config so validation tracking is accurate.
  m_config->ResetValidationCheck();
  auto& cfg = m_talonConfig;

  // Inversion
  if (auto inv = config.GetMotorInverted(); inv)
    cfg.MotorOutput.Inverted = *inv ? signals::InvertedValue::Clockwise_Positive
                                    : signals::InvertedValue::CounterClockwise_Positive;

  // Idle mode
  cfg.MotorOutput.NeutralMode = config.GetIdleMode() == SmartMotorControllerConfig::MotorMode::BRAKE
                                    ? signals::NeutralModeValue::Brake
                                    : signals::NeutralModeValue::Coast;

  // Consume control-mode option for validation tracking
  config.GetMotorControllerMode();

  // Encoder inversion is not supported on TalonFX; warn if set
  if (config.GetEncoderInverted().has_value())
    FRC_ReportWarning("TalonFXWrapper: EncoderInverted is not supported and will be ignored.");

  // No software temperature cutoff in Phoenix 6; consume option for validation
  config.GetTemperatureCutoff();

  // Closed-loop peak voltage
  if (auto maxV = config.GetClosedLoopControllerMaximumVoltage(); maxV) {
    cfg.Voltage.PeakForwardVoltage = *maxV;
    cfg.Voltage.PeakReverseVoltage = -*maxV;
  }

  // Gearing for sensor-to-mechanism
  if (auto& gearing = config.GetMotorGearing(); gearing) {
    cfg.Feedback.SensorToMechanismRatio =
        units::dimensionless::scalar_t{gearing->GetMechanismToRotorRatio()};
  }

  // External encoder configuration (driven by SmartMotorControllerConfig)
  if (auto enc = config.GetExternalEncoder(); enc.has_value() && config.GetUseExternalFeedback()) {
    gearing::MechanismGearing motorGearing =
        config.GetMotorGearing().value_or(gearing::MechanismGearing::kOne);
    gearing::MechanismGearing extGearing =
        config.GetExternalEncoderGearing().value_or(gearing::MechanismGearing::kOne);
    cfg.Feedback.RotorToSensorRatio =
        motorGearing.GetMechanismToRotorRatio() * extGearing.GetRotorToMechanismRatio();
    cfg.Feedback.SensorToMechanismRatio = extGearing.GetMechanismToRotorRatio();

    if (auto* pp = std::any_cast<hardware::CANcoder*>(&enc.value()); pp && *pp) {
      m_cancoder = **pp;
      auto& cancoderConfigurator = (*pp)->GetConfigurator();
      configs::CANcoderConfiguration cancoderCfg;
      cancoderConfigurator.Refresh(cancoderCfg);
      cfg.Feedback.FeedbackRemoteSensorID = (*pp)->GetDeviceID();
      if (auto inv = config.GetExternalEncoderInverted(); inv) {
        cancoderCfg.MagnetSensor.SensorDirection =
            *inv ? signals::SensorDirectionValue::Clockwise_Positive
                 : signals::SensorDirectionValue::CounterClockwise_Positive;
      }
      cfg.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::FusedCANcoder;
      if (auto offset = config.GetExternalEncoderZeroOffset(); offset) {
        cancoderCfg.MagnetSensor.MagnetOffset = *offset;
        cfg.Feedback.FeedbackRotorOffset = 0.0_tr;
      }
      if (auto dp = config.GetExternalEncoderDiscontinuityPoint(); dp) {
        cancoderCfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = *dp;
      }
      cancoderConfigurator.Apply(cancoderCfg);
    } else if (auto* pp = std::any_cast<hardware::CANdi*>(&enc.value()); pp && *pp) {
      m_candi = **pp;
      auto& candiConfigurator = (*pp)->GetConfigurator();
      configs::CANdiConfiguration candiCfg;
      candiConfigurator.Refresh(candiCfg);
      cfg.Feedback.FeedbackRemoteSensorID = (*pp)->GetDeviceID();
      const bool isPWM2 =
          (cfg.Feedback.FeedbackSensorSource == signals::FeedbackSensorSourceValue::SyncCANdiPWM2 ||
           cfg.Feedback.FeedbackSensorSource ==
               signals::FeedbackSensorSourceValue::RemoteCANdiPWM2);
      const bool isPWM1 =
          (cfg.Feedback.FeedbackSensorSource == signals::FeedbackSensorSourceValue::SyncCANdiPWM1 ||
           cfg.Feedback.FeedbackSensorSource ==
               signals::FeedbackSensorSourceValue::RemoteCANdiPWM1);
      if (isPWM2)
        cfg.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::FusedCANdiPWM2;
      if (isPWM1)
        cfg.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::FusedCANdiPWM1;
      if (isPWM1) {
        if (auto inv = config.GetExternalEncoderInverted(); inv)
          candiCfg.PWM1.SensorDirection = *inv;
        if (auto offset = config.GetExternalEncoderZeroOffset(); offset) {
          candiCfg.PWM1.AbsoluteSensorOffset = *offset;
          cfg.Feedback.FeedbackRotorOffset = 0.0_tr;
        }
        if (auto dp = config.GetExternalEncoderDiscontinuityPoint(); dp)
          candiCfg.PWM1.AbsoluteSensorDiscontinuityPoint = *dp;
      } else if (isPWM2) {
        if (auto inv = config.GetExternalEncoderInverted(); inv)
          candiCfg.PWM2.SensorDirection = *inv;
        if (auto offset = config.GetExternalEncoderZeroOffset(); offset) {
          candiCfg.PWM2.AbsoluteSensorOffset = *offset;
          cfg.Feedback.FeedbackRotorOffset = 0.0_tr;
        }
        if (auto dp = config.GetExternalEncoderDiscontinuityPoint(); dp)
          candiCfg.PWM2.AbsoluteSensorDiscontinuityPoint = *dp;
      } else {
        // Consume external encoder options even when no PWM source is active
        config.GetExternalEncoderInverted();
        config.GetExternalEncoderZeroOffset();
        config.GetExternalEncoderDiscontinuityPoint();
      }
      candiConfigurator.Apply(candiCfg);
    }
  } else {
    if (config.GetExternalEncoderInverted().has_value())
      throw exceptions::SmartMotorControllerConfigurationException(
          "External Encoder cannot be inverted if not present!",
          "External encoder is not inverted!", "WithExternalEncoderInverted(false)");
    if (config.GetExternalEncoderGearing().has_value())
      throw exceptions::SmartMotorControllerConfigurationException(
          "External Encoder cannot be set if not present!", "External encoder gearing is not 1.0!",
          "WithExternalEncoderGearing(1.0)");
    // Consume remaining external encoder options for validation tracking
    config.GetUseExternalFeedback();
    config.GetExternalEncoderZeroOffset();
    config.GetExternalEncoderDiscontinuityPoint();
    cfg.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RotorSensor;
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

  // Vendor control request — overrides the default profile-driven request
  if (auto req = config.GetVendorControlRequest()) {
    auto& r = *req;
    if (auto* p = std::any_cast<controls::PositionVoltage>(&r))
      m_positionReq = *p;
    else if (auto* p = std::any_cast<controls::PositionDutyCycle>(&r))
      m_positionReq = *p;
    else if (auto* p = std::any_cast<controls::PositionTorqueCurrentFOC>(&r))
      m_positionReq = *p;
    else if (auto* p = std::any_cast<controls::MotionMagicVoltage>(&r))
      m_positionReq = *p;
    else if (auto* p = std::any_cast<controls::MotionMagicDutyCycle>(&r))
      m_positionReq = *p;
    else if (auto* p = std::any_cast<controls::MotionMagicExpoVoltage>(&r))
      m_positionReq = *p;
    else if (auto* p = std::any_cast<controls::MotionMagicExpoDutyCycle>(&r))
      m_positionReq = *p;
    else if (auto* p = std::any_cast<controls::MotionMagicTorqueCurrentFOC>(&r))
      m_positionReq = *p;
    else if (auto* p = std::any_cast<controls::VelocityVoltage>(&r))
      m_velocityReq = *p;
    else if (auto* p = std::any_cast<controls::VelocityDutyCycle>(&r))
      m_velocityReq = *p;
    else if (auto* p = std::any_cast<controls::VelocityTorqueCurrentFOC>(&r))
      m_velocityReq = *p;
    else if (auto* p = std::any_cast<controls::MotionMagicVelocityVoltage>(&r))
      m_velocityReq = *p;
    else if (auto* p = std::any_cast<controls::MotionMagicVelocityDutyCycle>(&r))
      m_velocityReq = *p;
    else if (auto* p = std::any_cast<controls::MotionMagicVelocityTorqueCurrentFOC>(&r))
      m_velocityReq = *p;
  }

  auto status = m_talon->GetConfigurator().Apply(cfg);

  // LQR is not supported natively by Phoenix 6; hand control to the RoboRIO when configured.
  auto gains = config.GetSlotGains(m_slot);
  if (gains.lqr) {
    m_lqr = math::LQRController{*gains.lqr};
  } else {
    m_lqr.reset();
  }
  if (gains.kP != 0.0 || gains.kI != 0.0 || gains.kD != 0.0) {
    m_pid = frc::PIDController{gains.kP, gains.kI, gains.kD};
  } else {
    m_pid.reset();
  }

  if (m_lqr.has_value()) {
    int deviceId = static_cast<int>(m_talon->GetDeviceID());
    std::string alertText =
        "[YAMS] TalonFX(" + std::to_string(deviceId) +
        ") is running closed-loop control on the SystemCore (LQR active). "
        "Gains are not consistent with Phoenix 6 hardware PID and control runs at a lower "
        "frequency.";
    m_rioControllerAlert.emplace(alertText, frc::Alert::AlertType::kWarning);
    m_rioControllerAlert->Set(true);

    std::fprintf(stderr, "====== TalonFX(%d) Using RIO Closed Loop Controller ======\n", deviceId);

    if (m_closedLoopControllerThread) {
      StopClosedLoopController();
      m_closedLoopControllerThread.reset();
    }
    m_closedLoopControllerThread =
        std::make_unique<frc::Notifier>([this] { IterateClosedLoopController(); });
    if (auto name = config.GetTelemetryName(); name) {
      m_closedLoopControllerThread->SetName(*name);
    }
    if (config.GetMotorControllerMode() == ControlMode::CLOSED_LOOP) {
      StartClosedLoopController();
    }
  } else if (m_closedLoopControllerThread) {
    StopClosedLoopController();
    m_closedLoopControllerThread.reset();
    if (m_rioControllerAlert) m_rioControllerAlert->Set(false);
  }

  if (auto startPos = config.GetStartingPosition()) {
    if (m_cancoder) {
      m_cancoder->get().SetPosition(*startPos);
      if (frc::RobotBase::IsSimulation()) {
        auto& cancoderSim = m_cancoder->get().GetSimState();
        cancoderSim.SetRawPosition(*startPos);
        cancoderSim.SetMagnetHealth(ctre::phoenix6::signals::MagnetHealthValue::Magnet_Green);
      }
    } else {
      m_talon->SetPosition(*startPos);
    }
  }
  // Tightly coupled followers — accept TalonFX and TalonFXS (same Phoenix 6 vendor)
  for (auto& [hw, inverted] : config.GetFollowers()) {
    if (auto* fx = std::any_cast<hardware::TalonFX*>(&hw)) {
      (*fx)->SetControl(controls::Follower{m_talon->GetDeviceID(), inverted});
    } else if (auto* fxs = std::any_cast<hardware::TalonFXS*>(&hw)) {
      (*fxs)->SetControl(controls::Follower{m_talon->GetDeviceID(), inverted});
    } else {
      FRC_ReportWarning(
          "TalonFXWrapper: follower is not a TalonFX or TalonFXS and will be ignored.");
    }
  }
  LoadLooselyCoupledFollowers();

  config.ValidateBasicOptions();
  config.ValidateExternalEncoderOptions();
  return status.IsOK();
}

void TalonFXWrapper::ApplyPIDConfig() {
  auto gains =
      m_config->GetSlotGains(static_cast<SmartMotorControllerConfig::ClosedLoopControllerSlot>(0));
  m_talonConfig.Slot0.kP = gains.kP;
  m_talonConfig.Slot0.kI = gains.kI;
  m_talonConfig.Slot0.kD = gains.kD;
  m_talonConfig.Slot0.kS = gains.kS;
  m_talonConfig.Slot0.kV = gains.kV;
  m_talonConfig.Slot0.kA = gains.kA;

  auto gains1 =
      m_config->GetSlotGains(SmartMotorControllerConfig::ClosedLoopControllerSlot::SLOT_1);
  m_talonConfig.Slot1.kP = gains1.kP;
  m_talonConfig.Slot1.kI = gains1.kI;
  m_talonConfig.Slot1.kD = gains1.kD;
  m_talonConfig.Slot1.kS = gains1.kS;
  m_talonConfig.Slot1.kV = gains1.kV;
  m_talonConfig.Slot1.kA = gains1.kA;

  auto gains2 =
      m_config->GetSlotGains(SmartMotorControllerConfig::ClosedLoopControllerSlot::SLOT_2);
  m_talonConfig.Slot2.kP = gains2.kP;
  m_talonConfig.Slot2.kI = gains2.kI;
  m_talonConfig.Slot2.kD = gains2.kD;
  m_talonConfig.Slot2.kS = gains2.kS;
  m_talonConfig.Slot2.kV = gains2.kV;
  m_talonConfig.Slot2.kA = gains2.kA;
}

void TalonFXWrapper::ApplyFeedforwardConfig() {
  auto applySlot = [&](auto& slotCfg, ClosedLoopControllerSlot slotEnum) {
    auto gains = m_config->GetSlotGains(slotEnum);
    slotCfg.kG = gains.kG;
    if (gains.armFF)
      slotCfg.GravityType = signals::GravityTypeValue::Arm_Cosine;
    else if (gains.elevatorFF)
      slotCfg.GravityType = signals::GravityTypeValue::Elevator_Static;
  };
  applySlot(m_talonConfig.Slot0, ClosedLoopControllerSlot::SLOT_0);
  applySlot(m_talonConfig.Slot1, ClosedLoopControllerSlot::SLOT_1);
  applySlot(m_talonConfig.Slot2, ClosedLoopControllerSlot::SLOT_2);
}

void TalonFXWrapper::ApplyLimitsConfig() {
  if (auto upper = m_config->GetMechanismUpperLimit(); upper) {
    m_talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    m_talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = *upper;
  }
  if (auto lower = m_config->GetMechanismLowerLimit(); lower) {
    m_talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    m_talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = *lower;
  }
  m_talonConfig.ClosedLoopGeneral.ContinuousWrap = m_config->GetContinuousWrapping().has_value();
}

void TalonFXWrapper::ApplyMotionMagicConfig() {
  bool hasTrap = m_config->HasTrapezoidProfile();
  bool hasExpo = m_config->HasExponentialProfile() || m_config->HasLinearExponentialProfile();
  bool velTrap = m_config->GetVelocityTrapezoidalProfileInUse();
  int slotIdx = static_cast<int>(m_slot);

  if (hasTrap && !velTrap) {
    // Angular profile supplies velocity directly in TPS.  Linear profiles store
    // velocity in m/s; convert to TPS using the mechanism circumference.
    auto cruiseVel = m_config->GetTrapMaxVelocityTurns();
    auto cruiseAcc = m_config->GetTrapMaxAccelTurns();
    if (!cruiseVel) {
      if (auto linVel = m_config->GetTrapMaxVelocityLinear(); linVel)
        if (auto circ = m_config->GetMechanismCircumference(); circ && circ->value() != 0.0)
          cruiseVel = units::turns_per_second_t{linVel->value() / circ->value()};
    }
    if (!cruiseAcc) {
      if (auto linAcc = m_config->GetTrapMaxAccelLinear(); linAcc)
        if (auto circ = m_config->GetMechanismCircumference(); circ && circ->value() != 0.0)
          cruiseAcc = units::turns_per_second_squared_t{linAcc->value() / circ->value()};
    }
    if (cruiseVel) m_talonConfig.MotionMagic.MotionMagicCruiseVelocity = *cruiseVel;
    if (cruiseAcc) m_talonConfig.MotionMagic.MotionMagicAcceleration = *cruiseAcc;
    m_positionReq = controls::MotionMagicVoltage{0_tr}.WithSlot(slotIdx);
    m_velocityReq = controls::VelocityVoltage{0_tps}.WithSlot(slotIdx);
  } else if (hasTrap && velTrap) {
    // Velocity trapezoidal: the profile's maxVelocity param becomes MotionMagicAcceleration,
    // and maxAcceleration becomes MotionMagicJerk (one derivative up in velocity-space).
    auto accel = m_config->GetTrapMaxVelocityTurns();
    auto jerk = m_config->GetTrapMaxAccelTurns();
    if (!accel) {
      if (auto linVel = m_config->GetTrapMaxVelocityLinear(); linVel)
        if (auto circ = m_config->GetMechanismCircumference(); circ && circ->value() != 0.0)
          accel = units::turns_per_second_t{linVel->value() / circ->value()};
    }
    if (!jerk) {
      if (auto linAcc = m_config->GetTrapMaxAccelLinear(); linAcc)
        if (auto circ = m_config->GetMechanismCircumference(); circ && circ->value() != 0.0)
          jerk = units::turns_per_second_squared_t{linAcc->value() / circ->value()};
    }
    if (accel)
      m_talonConfig.MotionMagic.MotionMagicAcceleration =
          units::turns_per_second_squared_t{accel->value()};
    if (jerk)
      m_talonConfig.MotionMagic.MotionMagicJerk =
          units::angular_jerk::turns_per_second_cubed_t{jerk->value()};
    m_positionReq = controls::PositionVoltage{0_tr}.WithSlot(slotIdx);
    m_velocityReq = controls::MotionMagicVelocityVoltage{0_tps}.WithSlot(slotIdx);
  } else if (hasExpo) {
    if (auto kV = m_config->GetExponentialProfileKV())
      m_talonConfig.MotionMagic.MotionMagicExpo_kV = ctre::unit::volts_per_turn_per_second_t{*kV};
    if (auto kA = m_config->GetExponentialProfileKA())
      m_talonConfig.MotionMagic.MotionMagicExpo_kA =
          ctre::unit::volts_per_turn_per_second_squared_t{*kA};
    m_positionReq = controls::MotionMagicExpoVoltage{0_tr}.WithSlot(slotIdx);
    m_velocityReq = controls::VelocityVoltage{0_tps}.WithSlot(slotIdx);
  } else {
    using signals::StaticFeedforwardSignValue;
    m_talonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue::UseClosedLoopSign;
    m_talonConfig.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue::UseClosedLoopSign;
    m_talonConfig.Slot2.StaticFeedforwardSign = StaticFeedforwardSignValue::UseClosedLoopSign;
    m_positionReq = controls::PositionVoltage{0_tr}.WithSlot(slotIdx);
    m_velocityReq = controls::VelocityVoltage{0_tps}.WithSlot(slotIdx);
  }
}

// ---- Simulation -------------------------------------------------------------

void TalonFXWrapper::SetupSimulation() {
  if (!frc::RobotBase::IsSimulation()) return;

  auto simMotor = m_config->GetSimMotor();
  auto& gearing = m_config->GetMotorGearing();
  if (!simMotor || !gearing) return;

  auto plant = frc::LinearSystemId::DCMotorSystem(*simMotor, m_config->GetMOI(),
                                                  gearing->GetMechanismToRotorRatio());
  m_motorSim.emplace(plant, *simMotor);
  //  m_motorSim.SetMotorType(ctre::phoenix6::sim::TalonFXSimState::MotorType::KrakenX40);

  auto period = m_config->GetClosedLoopControlPeriod().value_or(20_ms);
  SetSimSupplier(std::make_shared<simulation::DCMotorSimSupplier>(
      *m_motorSim, [this]() { return GetDutyCycle(); }, *gearing, period));
  if (auto startPos = m_config->GetStartingPosition()) {
    m_simSupplier->SetMechanismPosition(*startPos);
    m_talon->GetSimState().SetRawRotorPosition(
        units::turn_t{startPos->value() * gearing->GetMechanismToRotorRatio()});
  }
}

void TalonFXWrapper::SimIterate() {
  if (!frc::RobotBase::IsSimulation() || !m_simSupplier) return;

  auto& sim = m_talon->GetSimState();
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
  m_talon->SetControl(m_dutyCycleReq.WithOutput(dutyCycle));
}

double TalonFXWrapper::GetDutyCycle() { return m_talon->GetDutyCycle().GetValue(); }

void TalonFXWrapper::SetVoltage(units::volt_t voltage) {
  m_talon->SetControl(m_voltageReq.WithOutput(voltage));
}

units::volt_t TalonFXWrapper::GetVoltage() { return m_talon->GetMotorVoltage().GetValue(); }

// ---- Closed-loop setpoints --------------------------------------------------

void TalonFXWrapper::SetPosition(units::turn_t angle) {
  m_setpointPosition = angle;
  if (m_closedLoopControllerRunning) return;
  std::visit([&](auto& req) { m_talon->SetControl(req.WithPosition(angle)); }, m_positionReq);
  ForwardPositionToFollowers(angle);
}

void TalonFXWrapper::SetPosition(units::meter_t distance) {
  if (auto circ = m_config->GetMechanismCircumference(); circ) {
    SetPosition(units::turn_t{distance.value() / circ->value()});
  } else {
    ForwardPositionToFollowers(distance);
  }
}

void TalonFXWrapper::SetVelocity(units::turns_per_second_t velocity) {
  m_setpointVelocity = velocity;
  if (m_closedLoopControllerRunning) return;
  std::visit([&](auto& req) { m_talon->SetControl(req.WithVelocity(velocity)); }, m_velocityReq);
  ForwardVelocityToFollowers(velocity);
}

void TalonFXWrapper::SetVelocity(units::meters_per_second_t velocity) {
  if (auto circ = m_config->GetMechanismCircumference(); circ) {
    SetVelocity(units::turns_per_second_t{velocity.value() / circ->value()});
  } else {
    ForwardVelocityToFollowers(velocity);
  }
}

// ---- Encoder writes ---------------------------------------------------------

void TalonFXWrapper::SetEncoderPosition(units::turn_t angle) { m_talon->SetPosition(angle); }

void TalonFXWrapper::SetEncoderPosition(units::meter_t distance) {
  if (auto circ = m_config->GetMechanismCircumference(); circ) {
    m_talon->SetPosition(units::turn_t{distance.value() / circ->value()});
  }
}

void TalonFXWrapper::SetEncoderVelocity(units::turns_per_second_t) {}

void TalonFXWrapper::SetEncoderVelocity(units::meters_per_second_t velocity) {}

// ---- Encoder reads ----------------------------------------------------------

units::turn_t TalonFXWrapper::GetMechanismPosition() { return m_talon->GetPosition().GetValue(); }

units::turns_per_second_t TalonFXWrapper::GetMechanismVelocity() {
  return m_talon->GetVelocity().GetValue();
}

units::turns_per_second_squared_t TalonFXWrapper::GetMechanismAcceleration() {
  return m_talon->GetAcceleration().GetValue();
}

units::turn_t TalonFXWrapper::GetRotorPosition() { return m_talon->GetRotorPosition().GetValue(); }

units::turns_per_second_t TalonFXWrapper::GetRotorVelocity() {
  return m_talon->GetRotorVelocity().GetValue();
}

units::meter_t TalonFXWrapper::GetMeasurementPosition() {
  auto circ = m_config->GetMechanismCircumference().value_or(1.0_m);
  return units::meter_t{GetMechanismPosition().value() * circ.value()};
}

units::meters_per_second_t TalonFXWrapper::GetMeasurementVelocity() {
  auto circ = m_config->GetMechanismCircumference().value_or(1.0_m);
  return units::meters_per_second_t{GetMechanismVelocity().value() * circ.value()};
}

units::meters_per_second_squared_t TalonFXWrapper::GetMeasurementAcceleration() {
  auto circ = m_config->GetMechanismCircumference().value_or(1.0_m);
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
  return m_talon->GetSupplyCurrent().GetValue();
}

units::ampere_t TalonFXWrapper::GetStatorCurrent() {
  return m_talon->GetStatorCurrent().GetValue();
}

units::celsius_t TalonFXWrapper::GetTemperature() {
  return units::celsius_t{m_talon->GetDeviceTemp().GetValue().value()};
}

frc::DCMotor TalonFXWrapper::GetDCMotor() { return m_dcMotor; }

// ---- Configuration setters --------------------------------------------------

void TalonFXWrapper::SetIdleMode(MotorMode mode) {
  m_talonConfig.MotorOutput.NeutralMode = mode == MotorMode::BRAKE
                                              ? signals::NeutralModeValue::Brake
                                              : signals::NeutralModeValue::Coast;
  m_talon->GetConfigurator().Apply(m_talonConfig.MotorOutput);
}

void TalonFXWrapper::SetMotorInverted(bool inv) {
  m_talonConfig.MotorOutput.Inverted = inv ? signals::InvertedValue::Clockwise_Positive
                                           : signals::InvertedValue::CounterClockwise_Positive;
  m_talon->GetConfigurator().Apply(m_talonConfig.MotorOutput);
}

void TalonFXWrapper::SetEncoderInverted(bool) {}

void TalonFXWrapper::SetKp(double kP) {
  auto apply = [&](auto& slot) {
    slot.kP = kP;
    m_talon->GetConfigurator().Apply(slot);
  };
  switch (m_slot) {
    case ClosedLoopControllerSlot::SLOT_0:
      apply(m_talonConfig.Slot0);
      break;
    case ClosedLoopControllerSlot::SLOT_1:
      apply(m_talonConfig.Slot1);
      break;
    case ClosedLoopControllerSlot::SLOT_2:
      apply(m_talonConfig.Slot2);
      break;
    default:
      break;
  }
}
void TalonFXWrapper::SetKi(double kI) {
  auto apply = [&](auto& slot) {
    slot.kI = kI;
    m_talon->GetConfigurator().Apply(slot);
  };
  switch (m_slot) {
    case ClosedLoopControllerSlot::SLOT_0:
      apply(m_talonConfig.Slot0);
      break;
    case ClosedLoopControllerSlot::SLOT_1:
      apply(m_talonConfig.Slot1);
      break;
    case ClosedLoopControllerSlot::SLOT_2:
      apply(m_talonConfig.Slot2);
      break;
    default:
      break;
  }
}
void TalonFXWrapper::SetKd(double kD) {
  auto apply = [&](auto& slot) {
    slot.kD = kD;
    m_talon->GetConfigurator().Apply(slot);
  };
  switch (m_slot) {
    case ClosedLoopControllerSlot::SLOT_0:
      apply(m_talonConfig.Slot0);
      break;
    case ClosedLoopControllerSlot::SLOT_1:
      apply(m_talonConfig.Slot1);
      break;
    case ClosedLoopControllerSlot::SLOT_2:
      apply(m_talonConfig.Slot2);
      break;
    default:
      break;
  }
}
void TalonFXWrapper::SetFeedback(double kP, double kI, double kD) {
  SetKp(kP);
  SetKi(kI);
  SetKd(kD);
}
void TalonFXWrapper::SetKs(double kS) {
  auto apply = [&](auto& slot) {
    slot.kS = kS;
    m_talon->GetConfigurator().Apply(slot);
  };
  switch (m_slot) {
    case ClosedLoopControllerSlot::SLOT_0:
      apply(m_talonConfig.Slot0);
      break;
    case ClosedLoopControllerSlot::SLOT_1:
      apply(m_talonConfig.Slot1);
      break;
    case ClosedLoopControllerSlot::SLOT_2:
      apply(m_talonConfig.Slot2);
      break;
    default:
      break;
  }
}
void TalonFXWrapper::SetKv(double kV) {
  auto apply = [&](auto& slot) {
    slot.kV = kV;
    m_talon->GetConfigurator().Apply(slot);
  };
  switch (m_slot) {
    case ClosedLoopControllerSlot::SLOT_0:
      apply(m_talonConfig.Slot0);
      break;
    case ClosedLoopControllerSlot::SLOT_1:
      apply(m_talonConfig.Slot1);
      break;
    case ClosedLoopControllerSlot::SLOT_2:
      apply(m_talonConfig.Slot2);
      break;
    default:
      break;
  }
}
void TalonFXWrapper::SetKa(double kA) {
  auto apply = [&](auto& slot) {
    slot.kA = kA;
    m_talon->GetConfigurator().Apply(slot);
  };
  switch (m_slot) {
    case ClosedLoopControllerSlot::SLOT_0:
      apply(m_talonConfig.Slot0);
      break;
    case ClosedLoopControllerSlot::SLOT_1:
      apply(m_talonConfig.Slot1);
      break;
    case ClosedLoopControllerSlot::SLOT_2:
      apply(m_talonConfig.Slot2);
      break;
    default:
      break;
  }
}
void TalonFXWrapper::SetKg(double kG) {
  auto apply = [&](auto& slot) {
    slot.kG = kG;
    auto gains = m_config->GetSlotGains(m_slot);
    if (gains.armFF)
      slot.GravityType = signals::GravityTypeValue::Arm_Cosine;
    else if (gains.elevatorFF)
      slot.GravityType = signals::GravityTypeValue::Elevator_Static;
    m_talon->GetConfigurator().Apply(slot);
  };
  switch (m_slot) {
    case ClosedLoopControllerSlot::SLOT_0:
      apply(m_talonConfig.Slot0);
      break;
    case ClosedLoopControllerSlot::SLOT_1:
      apply(m_talonConfig.Slot1);
      break;
    case ClosedLoopControllerSlot::SLOT_2:
      apply(m_talonConfig.Slot2);
      break;
    default:
      break;
  }
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
  m_talon->GetConfigurator().Apply(m_talonConfig.CurrentLimits);
}

void TalonFXWrapper::SetSupplyCurrentLimit(units::ampere_t limit) {
  m_talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  m_talonConfig.CurrentLimits.SupplyCurrentLimit = limit;
  m_talon->GetConfigurator().Apply(m_talonConfig.CurrentLimits);
}

void TalonFXWrapper::SetClosedLoopRampRate(units::second_t r) {
  m_talonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = r;
  m_talon->GetConfigurator().Apply(m_talonConfig.ClosedLoopRamps);
}

void TalonFXWrapper::SetOpenLoopRampRate(units::second_t r) {
  m_talonConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = r;
  m_talon->GetConfigurator().Apply(m_talonConfig.OpenLoopRamps);
}

void TalonFXWrapper::SetMechanismUpperLimit(units::turn_t upper) {
  m_talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
  m_talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = upper;
  m_talon->GetConfigurator().Apply(m_talonConfig.SoftwareLimitSwitch);
}

void TalonFXWrapper::SetMechanismLowerLimit(units::turn_t lower) {
  m_talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
  m_talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = lower;
  m_talon->GetConfigurator().Apply(m_talonConfig.SoftwareLimitSwitch);
}

void TalonFXWrapper::SetMechanismLimits(units::turn_t lower, units::turn_t upper) {
  SetMechanismLowerLimit(lower);
  SetMechanismUpperLimit(upper);
}

void TalonFXWrapper::SetMechanismLimitsEnabled(bool en) {
  m_talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = en;
  m_talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = en;
  m_talon->GetConfigurator().Apply(m_talonConfig.SoftwareLimitSwitch);
}

void TalonFXWrapper::SetMeasurementUpperLimit(units::meter_t upper) {
  if (auto circ = m_config->GetMechanismCircumference(); circ)
    SetMechanismUpperLimit(units::turn_t{upper.value() / circ->value()});
}

void TalonFXWrapper::SetMeasurementLowerLimit(units::meter_t lower) {
  if (auto circ = m_config->GetMechanismCircumference(); circ)
    SetMechanismLowerLimit(units::turn_t{lower.value() / circ->value()});
}

void TalonFXWrapper::SetMotionProfileMaxVelocity(units::turns_per_second_t vel) {
  m_talonConfig.MotionMagic.MotionMagicCruiseVelocity = vel;
  m_talon->GetConfigurator().Apply(m_talonConfig.MotionMagic);
}

void TalonFXWrapper::SetMotionProfileMaxVelocity(units::meters_per_second_t vel) {
  if (auto circ = m_config->GetMechanismCircumference(); circ)
    SetMotionProfileMaxVelocity(units::turns_per_second_t{vel.value() / circ->value()});
}

void TalonFXWrapper::SetMotionProfileMaxAcceleration(units::turns_per_second_squared_t acc) {
  m_talonConfig.MotionMagic.MotionMagicAcceleration = acc;
  m_talon->GetConfigurator().Apply(m_talonConfig.MotionMagic);
}

void TalonFXWrapper::SetMotionProfileMaxAcceleration(units::meters_per_second_squared_t acc) {
  if (auto circ = m_config->GetMechanismCircumference(); circ)
    SetMotionProfileMaxAcceleration(units::turns_per_second_squared_t{acc.value() / circ->value()});
}

void TalonFXWrapper::SetMotionProfileMaxJerk(units::angular_jerk::turns_per_second_cubed_t jerk) {
  m_talonConfig.MotionMagic.MotionMagicJerk = jerk;
  m_talon->GetConfigurator().Apply(m_talonConfig.MotionMagic);
}

void TalonFXWrapper::SetExponentialProfile(std::optional<double> kV, std::optional<double> kA,
                                           std::optional<units::volt_t> maxInput) {
  if (kV || kA) {
    if (kV)
      m_talonConfig.MotionMagic.MotionMagicExpo_kV = ctre::unit::volts_per_turn_per_second_t{*kV};
    if (kA)
      m_talonConfig.MotionMagic.MotionMagicExpo_kA =
          ctre::unit::volts_per_turn_per_second_squared_t{*kA};
    m_talon->GetConfigurator().Apply(m_talonConfig.MotionMagic);
  }
}

void TalonFXWrapper::SetClosedLoopSlot(ClosedLoopControllerSlot slot) {
  m_slot = slot;
  int idx = static_cast<int>(slot);
  std::visit([idx](auto& req) { req.WithSlot(idx); }, m_positionReq);
  std::visit([idx](auto& req) { req.WithSlot(idx); }, m_velocityReq);
}

SmartMotorControllerConfig& TalonFXWrapper::GetConfig() { return *m_config; }
void* TalonFXWrapper::GetMotorController() { return m_talon; }
void* TalonFXWrapper::GetMotorControllerConfig() { return &m_talonConfig; }

telemetry::UnsupportedTelemetryFields TalonFXWrapper::GetUnsupportedTelemetryFields() {
  return {};  // TalonFX supports all telemetry fields
}

TalonFXWrapper::~TalonFXWrapper() {
  if (m_closedLoopControllerThread) {
    m_closedLoopControllerThread->Stop();
    m_closedLoopControllerThread.reset();
  }
  // delete m_talon;
}

}  // namespace yams::motorcontrollers::remote
