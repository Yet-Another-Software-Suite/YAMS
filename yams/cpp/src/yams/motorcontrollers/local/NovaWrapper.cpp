// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/local/NovaWrapper.hpp"

#include <thrifty/canEncoder/CanEncoderConfig.h>
#include <thrifty/core/MotorControl.h>
#include <thrifty/nova/NovaConfig.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <wpi/framework/RobotBase.hpp>
#include <wpi/math/system/Models.hpp>
#include <wpi/math/util/MathUtil.hpp>

#include "yams/exceptions.hpp"
#include "yams/math/LQRController.hpp"
#include "yams/motorcontrollers/simulation/BatterySim.hpp"
#include "yams/motorcontrollers/simulation/DCMotorSimSupplier.hpp"
#include "yams/motorcontrollers/simulation/NovaSimSupplier.hpp"

namespace yams::motorcontrollers::local {

namespace {

using Slot = SmartMotorControllerConfig::ClosedLoopControllerSlot;
using FeedbackSensorType = thrifty::Motor::FeedbackSensorType;
using TurnKv = wpi::math::SimpleMotorFeedforward<wpi::units::turns>::kv_unit;
using TurnKa = wpi::math::SimpleMotorFeedforward<wpi::units::turns>::ka_unit;

/** Gains of the configured feedforward in its own units (ArmFeedforward kV/kA per radian). */
struct NativeFeedforward {
  double kS{0.0}, kV{0.0}, kA{0.0}, kG{0.0};
};

/**
 * Apply @p update to the slot's configured feedforward in its native units and store it back in
 * the config, where the SystemCore closed loop reads it. With @p nova, also send it to the Nova's
 * motion profile slot, per mechanism rotation: an arm's feedforward is per radian, an elevator's per
 * meter.
 */
template <typename F>
void UpdateConfigFeedforward(SmartMotorControllerConfig& config, Slot slot, F&& update,
                             thrifty::Nova* nova, bool novaSlot1) {
  const auto sendToNova = [&](double kS, double kV, double kA, double kG) {
    if (!nova) return;
    if (novaSlot1) {
      nova->Configure(thrifty::NovaConfig::Mp1::kS(kS), thrifty::NovaConfig::Mp1::kV(kV),
                      thrifty::NovaConfig::Mp1::kA(kA), thrifty::NovaConfig::Mp1::kG(kG));
    } else {
      nova->Configure(thrifty::NovaConfig::Mp0::kS(kS), thrifty::NovaConfig::Mp0::kV(kV),
                      thrifty::NovaConfig::Mp0::kA(kA), thrifty::NovaConfig::Mp0::kG(kG));
    }
  };
  if (auto ff = config.GetArmFeedforward(slot)) {
    NativeFeedforward v{ff->GetKs().value(), ff->GetKv().value(), ff->GetKa().value(),
                        ff->GetKg().value()};
    update(v);
    using Arm = wpi::math::ArmFeedforward;
    ff->SetKs(wpi::units::volt_t{v.kS});
    ff->SetKv(wpi::units::unit_t<Arm::kv_unit>{v.kV});
    ff->SetKa(wpi::units::unit_t<Arm::ka_unit>{v.kA});
    ff->SetKg(wpi::units::volt_t{v.kG});
    config.WithFeedforward(*ff, slot);
    sendToNova(v.kS, wpi::units::unit_t<TurnKv>{ff->GetKv()}.value(),
               wpi::units::unit_t<TurnKa>{ff->GetKa()}.value(), v.kG);
  } else if (auto ff = config.GetElevatorFeedforward(slot)) {
    NativeFeedforward v{ff->GetKs().value(), ff->GetKv().value(), ff->GetKa().value(),
                        ff->GetKg().value()};
    update(v);
    using Elevator = wpi::math::ElevatorFeedforward;
    ff->SetKs(wpi::units::volt_t{v.kS});
    ff->SetKv(wpi::units::unit_t<Elevator::kv_unit>{v.kV});
    ff->SetKa(wpi::units::unit_t<Elevator::ka_unit>{v.kA});
    ff->SetKg(wpi::units::volt_t{v.kG});
    config.WithFeedforward(*ff, slot);
    if (nova) {
      const double rotationsPerMeter = config.ConvertToMechanism(wpi::units::meter_t{1.0}).value();
      sendToNova(v.kS, v.kV / rotationsPerMeter, v.kA / rotationsPerMeter, v.kG);
    }
  } else if (auto ff = config.GetSimpleFeedforward(slot)) {
    NativeFeedforward v{ff->GetKs().value(), ff->GetKv().value(), ff->GetKa().value(), 0.0};
    update(v);
    ff->SetKs(wpi::units::volt_t{v.kS});
    ff->SetKv(wpi::units::unit_t<TurnKv>{v.kV});
    ff->SetKa(wpi::units::unit_t<TurnKa>{v.kA});
    config.WithFeedforward(*ff, slot);
    sendToNova(v.kS, v.kV, v.kA, 0.0);
  }
}

}  // namespace

// ---- Construction -----------------------------------------------------------

NovaWrapper::NovaWrapper(thrifty::Nova* nova, wpi::math::DCMotor motor,
                         SmartMotorControllerConfig* cfg)
    : SmartMotorController(), m_nova(nova), m_motor(motor) {
  if (auto vc = cfg->GetVendorConfig(); vc.has_value()) {
    if (auto* p = std::any_cast<thrifty::NovaConfigBatch>(&vc.value())) {
      m_vendorConfig = *p;
    } else {
      throw exceptions::SmartMotorControllerConfigurationException(
          "NovaConfigBatch is the only acceptable vendor config for Nova controllers.",
          "NovaConfigBatch not found.", "WithVendorConfig(thrifty::NovaConfigBatch{...})");
    }
  }
  m_config = cfg;
  m_config->WithSimMotor(motor);
  m_systemCoreClosedLoopAlert.emplace(
      "YAMS", AlertId("ClosedLoop"),
      GetName() + " closed loop controller is running on the SystemCore.",
      wpi::util::Alert::Level::MEDIUM);

  SetupSimulation();
  try {
    ApplyConfig(*m_config);
    CheckConfigSafety();
  } catch (...) {
    // Release what applying the config started (e.g. the closed-loop Notifier) for a motor
    // controller that was never made.
    Close();
    throw;
  }
}

NovaWrapper::~NovaWrapper() {
  Close();
  if (m_systemCoreClosedLoopAlert) m_systemCoreClosedLoopAlert->Set(false);
}

// ---- Helpers ----------------------------------------------------------------

std::string NovaWrapper::AlertId(const std::string& alertType) const {
  const std::string prefix = "Nova(" + std::to_string(m_nova->Status().GetCanId()) + ")";
  if (auto name = m_config->GetTelemetryName()) return prefix + "_" + alertType + "_" + *name;
  return prefix + "_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)) + "_" + alertType;
}

double NovaWrapper::MechanismToRotorRatio() const {
  return m_config->GetMotorGearing()
      .value_or(gearing::MechanismGearing::kOne)
      .GetMechanismToRotorRatio();
}

double NovaWrapper::MechanismToExternalEncoderRatio() const {
  // A direct conversion factor (encoder rotations -> mechanism rotations) overrides the gearing.
  if (auto cf = m_config->GetExternalEncoderConversionFactor(); cf && *cf != 0.0) return 1.0 / *cf;
  return m_config->GetExternalEncoderGearing()
      .value_or(gearing::MechanismGearing::kOne)
      .GetMechanismToRotorRatio();
}

void NovaWrapper::LoadClosedLoopController(const SmartMotorControllerConfig& config) {
  const auto gains = config.GetSlotGains(m_slot);
  if (gains.lqr) {
    m_lqr = math::LQRController{*gains.lqr};
  } else {
    m_lqr.reset();
  }
  if (gains.kP != 0.0 || gains.kI != 0.0 || gains.kD != 0.0) {
    m_pid = wpi::math::PIDController{gains.kP, gains.kI, gains.kD};
  } else {
    m_pid.reset();
  }
}

void NovaWrapper::CreateMotorSim() {
  auto plant = wpi::math::Models::SingleJointedArmFromPhysicalConstants(
      m_motor, m_config->GetMOI(), MechanismToRotorRatio());
  m_motorSim.emplace(plant, m_motor);
  SetSimSupplier(std::make_shared<simulation::DCMotorSimSupplier>(*m_motorSim, *this));
}

// ---- Configuration ----------------------------------------------------------

bool NovaWrapper::ApplyConfig(const SmartMotorControllerConfig& config) {
  // Like the Java wrapper, the applied config becomes this controller's config.
  m_config = const_cast<SmartMotorControllerConfig*>(&config);
  config.ResetValidationCheck();
  if (m_systemCoreClosedLoopAlert) m_systemCoreClosedLoopAlert->Set(false);

  if (config.GetVendorControlRequest().has_value())
    throw exceptions::SmartMotorControllerConfigurationException(
        "Nova(" + std::to_string(m_nova->Status().GetCanId()) +
            ") does not support the custom control requests!",
        "Cannot use given control request", "WithVendorControlRequest()");

  LoadClosedLoopController(config);
  if (m_lqr && config.GetClosedLoopTolerance())
    throw std::invalid_argument("[Error] Closed loop tolerance is not supported in LQR mode.");
  ConfigureSoftwarePID(config);
  const bool closedLoop = config.GetMotorControllerMode() == ControlMode::CLOSED_LOOP;
  const bool trapezoidProfile = config.HasTrapezoidProfile();
  m_closedLoopOnNova = closedLoop && NovaCanRunClosedLoop(config);
  m_novaSlot = m_slot == Slot::SLOT_1 ? thrifty::MotorControl::Slot::SLOT1
                                      : thrifty::MotorControl::Slot::SLOT0;
  config.GetMotorGearing();

  // Options only the SystemCore's closed loop applies.
  const bool temperatureCutoff = config.GetTemperatureCutoff().has_value();
  const bool maximumVoltage = config.GetClosedLoopControllerMaximumVoltage().has_value();
  const bool synchronizationThreshold = config.GetFeedbackSynchronizationThreshold().has_value();
  const auto closedLoopControlPeriod = config.GetClosedLoopControlPeriod();
  if (m_closedLoopOnNova && temperatureCutoff)
    throw exceptions::SmartMotorControllerConfigurationException(
        "Temperature cutoff is only available for exponentially profiled or LQR closed loop "
        "controllers",
        "Temperature cutoff could not be applied", "WithTemperatureCutoff");
  if (m_closedLoopOnNova && maximumVoltage)
    throw exceptions::SmartMotorControllerConfigurationException(
        "Closed loop controller maximum voltage is only available for exponentially profiled or "
        "LQR closed loop controllers",
        "Closed loop controller maximum voltage could not be applied",
        "WithClosedLoopControllerMaximumVoltage");
  if (m_closedLoopOnNova && synchronizationThreshold)
    throw exceptions::SmartMotorControllerConfigurationException(
        "Feedback synchronization threshold is only available for exponentially profiled or LQR "
        "closed loop controllers",
        "Feedback synchronization threshold could not be applied",
        "WithFeedbackSynchronizationThreshold");
  if (m_closedLoopOnNova && closedLoopControlPeriod)
    throw exceptions::SmartMotorControllerConfigurationException(
        "Closed loop control period is only available for exponentially profiled or LQR closed "
        "loop controllers",
        "Closed loop control period does not take effect", "WithClosedLoopControlPeriod");
  if (!closedLoop && closedLoopControlPeriod)
    throw std::invalid_argument(
        "[Error] Closed loop control period is only supported in closed loop mode.");

  // The SystemCore's closed loop controller, unless the Nova runs the closed loop. Without it,
  // StartClosedLoopController() does nothing, so the two never drive the Nova together.
  if (m_closedLoopControllerThread) {
    StopClosedLoopController();
    m_closedLoopControllerThread.reset();
  }
  if (!m_closedLoopOnNova) {
    m_closedLoopControllerThread =
        std::make_unique<wpi::Notifier>([this] { IterateClosedLoopController(); });
    if (auto name = config.GetTelemetryName(); name) m_closedLoopControllerThread->SetName(*name);
  }

  // Settings are sent to the Nova together, in order: reset the Nova if configured, then the
  // vendor config, then the YAMS config, which overrides the vendor config.
  thrifty::NovaConfigBatch resetConfig;
  if (config.GetResetPreviousConfig()) resetConfig.Add(thrifty::NovaConfig::FactoryReset());
  thrifty::NovaConfigBatch novaConfig;

  // Ramp rates. The Nova has one ramp rate, applied to the SystemCore's closed loop output too.
  if (config.GetOpenLoopRampRate() && config.GetClosedLoopRampRate() &&
      *config.GetOpenLoopRampRate() != *config.GetClosedLoopRampRate())
    throw exceptions::SmartMotorControllerConfigurationException(
        "Thrifty Nova has one ramp rate for open and closed loop control",
        "Different open and closed loop ramp rates could not be applied",
        "WithOpenLoopRampRate or WithClosedLoopRampRate, not both");
  if (auto rate = config.GetOpenLoopRampRate() ? config.GetOpenLoopRampRate()
                                               : config.GetClosedLoopRampRate()) {
    novaConfig.Add(thrifty::NovaConfig::RampForward(rate->value()),
                   thrifty::NovaConfig::RampReverse(rate->value()));
  }

  if (auto inverted = config.GetMotorInverted())
    novaConfig.Add(thrifty::NovaConfig::Direction(*inverted ? thrifty::Motor::Direction::COUNTER_CLOCKWISE : thrifty::Motor::Direction::CLOCKWISE));
  if (config.GetEncoderInverted())
    throw std::invalid_argument("[ERROR] Thrifty Nova internal encoder cannot be inverted!");

  if (auto supply = config.GetSupplyCurrentLimit())
    novaConfig.Add(thrifty::NovaConfig::SupplyCurrent(supply->value()));
  if (auto stator = config.GetStatorCurrentLimit())
    novaConfig.Add(thrifty::NovaConfig::StatorCurrent(stator->value()));
  if (auto vc = config.GetVoltageCompensation())
    novaConfig.Add(thrifty::NovaConfig::VoltageComp(vc->value()));
  if (auto zp = config.GetZeroPower())
    novaConfig.Add(thrifty::NovaConfig::BrakeMode(*zp == MotorMode::BRAKE));

  ApplyExternalEncoder(config, novaConfig);
  novaConfig.Add(thrifty::NovaConfig::SensorToMechanismRatio(m_mechanismToFeedbackSensorRatio));

  // The mechanism limits are the Nova's soft limits, on its feedback sensor in mechanism rotations,
  // in closed loop. The Nova has one enable for both, so they are set together.
  if (config.GetMechanismUpperLimit().has_value() != config.GetMechanismLowerLimit().has_value())
    throw exceptions::SmartMotorControllerConfigurationException(
        "Thrifty Nova soft limits need both an upper and a lower mechanism limit",
        "Mechanism limit could not be applied", "WithMechanismLimits(lower, upper)");
  if (config.GetMechanismUpperLimit()) {
    novaConfig.Add(thrifty::NovaConfig::SoftLimitSource(m_feedbackSensor),
                   thrifty::NovaConfig::SoftLimits(config.GetMechanismUpperLimit()->value(),
                                                   config.GetMechanismLowerLimit()->value()),
                   thrifty::NovaConfig::SoftLimitEnable(closedLoop));
  }
  // Continuous wrapping, on the Nova, of the absolute encoder closing its loop over [0, 1)
  // rotations; NovaCanRunClosedLoop runs any other wrapping on the SystemCore.
  const bool absoluteFeedback = m_feedbackSensor == FeedbackSensorType::ABS ||
                                m_feedbackSensor == FeedbackSensorType::CAN_ENCODER;
  // A Nova wraps an absolute encoder closing its loop by default, so it is told either way.
  if (absoluteFeedback)
    novaConfig.Add(
        thrifty::NovaConfig::AbsWrapping(config.IsWrappingContinuous() && m_closedLoopOnNova));

  // Each closed loop slot's PID gains, tolerance, motion profile and feedforward, on the Nova's two
  // slots when it runs the closed loop. Its sensor-to-mechanism ratio puts them in mechanism
  // rotations, its voltage closed loops and motion profile output volts, like YAMS gains. The feedforward is the motion
  // profile slot's, so the PID slots' F is 0.
  if (m_closedLoopOnNova) {
    // Gains of a linear mechanism are per meter; the Nova's are per mechanism rotation.
    m_rotationsPerGainUnit = config.GetLinearClosedLoopControllerUse()
                                 ? config.ConvertToMechanism(wpi::units::meter_t{1.0}).value()
                                 : 1.0;
    double maxVelocity = 0.0;
    double maxAcceleration = 0.0;
    if (auto linearVelocity = config.GetTrapMaxVelocityLinear();
        linearVelocity && !config.GetTrapMaxVelocityTurns()) {
      maxVelocity = config.ConvertToMechanism(*linearVelocity).value();
      maxAcceleration = config.ConvertToMechanism(*config.GetTrapMaxAccelLinear()).value();
    } else if (auto velocity = config.GetTrapMaxVelocityTurns()) {
      maxVelocity = velocity->value();
      maxAcceleration = config.GetTrapMaxAccelTurns()->value();
    }
    for (auto slot : {Slot::SLOT_0, Slot::SLOT_1, Slot::SLOT_2, Slot::SLOT_3}) {
      const auto gains = config.GetSlotGains(slot);
      const bool feedforward = gains.armFF || gains.elevatorFF || gains.simpleFF;
      if (slot == Slot::SLOT_2 || slot == Slot::SLOT_3) {
        if (gains.kP != 0.0 || gains.kI != 0.0 || gains.kD != 0.0 || feedforward)
          throw exceptions::SmartMotorControllerConfigurationException(
              "Thrifty Nova has two closed loop slots, SLOT_0 and SLOT_1",
              "Closed loop slot could not be configured", "SLOT_0 or SLOT_1");
        continue;
      }
      const bool slot0 = slot == Slot::SLOT_0;
      if (gains.kP != 0.0 || gains.kI != 0.0 || gains.kD != 0.0) {
        const double kP = gains.kP / m_rotationsPerGainUnit;
        const double kI = gains.kI / m_rotationsPerGainUnit;
        const double kD = gains.kD / m_rotationsPerGainUnit;
        if (slot0) {
          novaConfig.Add(thrifty::NovaConfig::Pid0::P(kP), thrifty::NovaConfig::Pid0::I(kI),
                         thrifty::NovaConfig::Pid0::D(kD));
        } else {
          novaConfig.Add(thrifty::NovaConfig::Pid1::P(kP), thrifty::NovaConfig::Pid1::I(kI),
                         thrifty::NovaConfig::Pid1::D(kD));
        }
        if (auto tolerance = config.GetClosedLoopTolerance())
          novaConfig.Add(slot0 ? thrifty::NovaConfig::Pid0::AllowableError(tolerance->value())
                               : thrifty::NovaConfig::Pid1::AllowableError(tolerance->value()));
      }

      // The motion profile, and its feedforward, which needs one.
      if (!trapezoidProfile) continue;
      double kS = 0.0, kV = 0.0, kA = 0.0, kG = 0.0;
      auto gravityType = thrifty::Motor::MPGravityType::CONSTANT;
      if (auto ff = config.GetArmFeedforward(slot)) {
        // ArmFeedforward gains are per radian; the Nova's are per rotation.
        kS = ff->GetKs().value();
        kV = wpi::units::unit_t<TurnKv>{ff->GetKv()}.value();
        kA = wpi::units::unit_t<TurnKa>{ff->GetKa()}.value();
        kG = ff->GetKg().value();
        gravityType = thrifty::Motor::MPGravityType::COSINE;
      } else if (auto ff = config.GetElevatorFeedforward(slot)) {
        // An elevator's feedforward is per meter.
        const double rotationsPerMeter = config.ConvertToMechanism(wpi::units::meter_t{1.0}).value();
        kS = ff->GetKs().value();
        kV = ff->GetKv().value() / rotationsPerMeter;
        kA = ff->GetKa().value() / rotationsPerMeter;
        kG = ff->GetKg().value();
      } else if (auto ff = config.GetSimpleFeedforward(slot)) {
        kS = ff->GetKs().value();
        kV = ff->GetKv().value();
        kA = ff->GetKa().value();
      }
      novaConfig.Add(
          slot0 ? thrifty::NovaConfig::mp0(maxVelocity, maxAcceleration, kV, kA, kS, kG, gravityType)
                : thrifty::NovaConfig::mp1(maxVelocity, maxAcceleration, kV, kA, kS, kG, gravityType));
    }
  }

  if (m_vendorConfig) {
    m_nova->Configure(resetConfig, *m_vendorConfig, novaConfig);
  } else {
    m_nova->Configure(resetConfig, novaConfig);
  }

  // A quadrature encoder counts from where it powers on, like the motor's encoder.
  const auto startingPosition = config.GetStartingPosition();
  if (m_dataPortEncoder == FeedbackSensorType::QUAD) {
    m_nova->SetQuadPosition(startingPosition.value_or(wpi::units::turn_t{0.0}).value() *
                            MechanismToExternalEncoderRatio());
  }
  // Start from the starting position, or from the absolute encoder without one.
  if (startingPosition) {
    m_nova->SetIntPosition(startingPosition->value() * MechanismToRotorRatio());
    if (m_simSupplier) m_simSupplier->SetMechanismPosition(*startingPosition);
  } else if (m_canEncoder || m_dataPortEncoder == FeedbackSensorType::ABS) {
    SeedRelativeEncoder();
  }

  // Tightly coupled followers accept Novas only.
  if (!config.GetFollowers().empty()) {
    for (auto& [hw, inverted] : config.GetFollowers()) {
      auto* follower = std::any_cast<thrifty::Nova*>(&hw);
      if (!follower || !*follower)
        throw std::invalid_argument(
            "[ERROR] Unknown follower type: NovaWrapper followers must be thrifty::Nova*");
      thrifty::NovaConfigBatch followerConfig{
          thrifty::NovaConfig::Follow(m_nova->Status().GetCanId())};
      if (auto zp = config.GetZeroPower())
        followerConfig.Add(thrifty::NovaConfig::BrakeMode(*zp == MotorMode::BRAKE));
      (*follower)->Configure(followerConfig);
      (*follower)->SetInverted(inverted);
    }
    // The followers keep following; applying the config again must not reconfigure them.
    m_config->ClearFollowers();
  }
  LoadLooselyCoupledFollowers();

  config.ValidateBasicOptions();
  config.ValidateExternalEncoderOptions();

  if (closedLoop && !m_closedLoopOnNova) {
    m_systemCoreClosedLoopAlert->Set(true);
    StartClosedLoopController();
  }
  return true;
}

void NovaWrapper::SetSimSupplier(std::shared_ptr<SimSupplier> supplier) {
  // The mechanism is driven by a simulated Nova, which runs the Nova's closed loop.
  // The external encoder's shaft is driven by a MotorSim of the mechanism.
  if (!supplier) {
    m_novaSim.reset();
    m_externalEncoderSim.reset();
    SmartMotorController::SetSimSupplier(nullptr);
    return;
  }
  m_externalEncoderSim.emplace(
      [this, supplier] {
        return supplier->GetMechanismPosition().value() * MechanismToExternalEncoderRatio();
      },
      [this, supplier] {
        return supplier->GetMechanismVelocity().value() * MechanismToExternalEncoderRatio();
      });
  if (m_canEncoder)
    m_canEncoder->Configure(thrifty::CanEncoderConfig::SimMotor(*m_externalEncoderSim));
  auto novaSim = std::make_shared<simulation::NovaSimSupplier>(std::move(supplier), *m_nova,
                                                               m_config->GetSimulationPeriod());
  if (m_novaSim && m_novaSim->GetControlRequest())
    novaSim->SetControlRequest(*m_novaSim->GetControlRequest());
  m_novaSim = novaSim;
  SmartMotorController::SetSimSupplier(std::move(novaSim));
}

bool NovaWrapper::NovaCanRunClosedLoop(const SmartMotorControllerConfig& config) const {
  const auto wrappingMax = config.GetWrappingMax();
  const bool continuousWrapping = config.IsWrappingContinuous();
  const bool trapezoidProfile = config.HasTrapezoidProfile();
  if (m_lqr || config.HasExponentialProfile() || config.HasLinearExponentialProfile() ||
      (trapezoidProfile && config.GetVelocityTrapezoidalProfileInUse()))
    return false;
  // A feedforward with every gain 0 does nothing, so it needs no profile.
  const auto nonZero = [](const auto& ff, double kG) {
    return ff.GetKs().value() != 0.0 || ff.GetKv().value() != 0.0 || ff.GetKa().value() != 0.0 ||
           kG != 0.0;
  };
  for (auto slot : {Slot::SLOT_0, Slot::SLOT_1, Slot::SLOT_2, Slot::SLOT_3}) {
    const auto gains = config.GetSlotGains(slot);
    const bool feedforward = (gains.armFF && nonZero(*gains.armFF, gains.armFF->GetKg().value())) ||
                             (gains.elevatorFF &&
                              nonZero(*gains.elevatorFF, gains.elevatorFF->GetKg().value())) ||
                             (gains.simpleFF && nonZero(*gains.simpleFF, 0.0));
    if (!trapezoidProfile && feedforward) return false;
  }
  if (!wrappingMax) return true;
  const bool fromZero = std::abs(config.GetWrappingMin()->value()) <= 1e-9;
  bool absoluteFeedback = false;
  if (auto enc = config.GetExternalEncoder(); enc && config.GetUseExternalFeedback()) {
    auto* sensor = std::any_cast<FeedbackSensorType>(&*enc);
    auto* canEncoder = std::any_cast<thrifty::CanEncoder*>(&*enc);
    absoluteFeedback = (sensor && *sensor == FeedbackSensorType::ABS) || (canEncoder && *canEncoder);
  }
  // Without continuous wrapping, the Nova's [0, 1) reading must be the range, or the mechanism would
  // cross the Nova's wrapping point inside it.
  if (!continuousWrapping) return !absoluteFeedback || fromZero;
  // Continuous wrapping takes the short way whatever the range. The Nova wraps an absolute encoder
  // on the mechanism itself, sent each target in [0, 1). It can't wrap its motor's or a quadrature
  // encoder, so each target is sent as the equivalent nearest the mechanism instead. An absolute
  // encoder geared to the mechanism wraps once per encoder rotation, inside the mechanism's range,
  // so only the SystemCore can wrap it.
  return !absoluteFeedback || MechanismToExternalEncoderRatio() == 1.0;
}

double NovaWrapper::NovaPositionTarget(wpi::units::turn_t angle) {
  if (!m_config->IsWrappingContinuous()) return angle.value();
  if (m_feedbackSensor == FeedbackSensorType::ABS ||
      m_feedbackSensor == FeedbackSensorType::CAN_ENCODER)
    return wpi::math::InputModulus(angle.value(), 0.0, 1.0);
  double position = GetRelativeMechanismPosition().value();
  if (m_feedbackSensor == FeedbackSensorType::QUAD)
    if (auto external = GetExternalEncoderMechanismPosition()) position = external->value();
  return position + wpi::math::InputModulus(angle.value() - position, -0.5, 0.5);
}

void NovaWrapper::ApplyExternalEncoder(const SmartMotorControllerConfig& config,
                                       thrifty::NovaConfigBatch& novaConfig) {
  // The external encoder, and the sensor the Nova closes its loop on: the motor's encoder or the
  // external encoder, with its rotations per mechanism rotation, which put the Nova's targets and
  // gains in mechanism rotations.
  m_dataPortEncoder.reset();
  m_canEncoder = nullptr;
  m_feedbackSensor = FeedbackSensorType::INTERNAL;
  m_mechanismToFeedbackSensorRatio = MechanismToRotorRatio();
  const bool useExternalFeedback = config.GetUseExternalFeedback();

  auto enc = config.GetExternalEncoder();
  if (!enc) {
    if (config.GetExternalEncoderZeroOffset().has_value())
      throw exceptions::SmartMotorControllerConfigurationException(
          "Zero offset is only available for external encoders",
          "Zero offset could not be applied", "WithExternalEncoder(encoder)");
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
    return;
  }

  config.GetExternalEncoderGearing();
  const double mechToEncoder = MechanismToExternalEncoderRatio();
  const auto zeroOffset = config.GetExternalEncoderZeroOffset();
  const auto inverted = config.GetExternalEncoderInverted();
  // Zero offsets are in [0, 1) rotations of the encoder.
  const double zeroOffsetRotations =
      zeroOffset ? wpi::math::InputModulus(zeroOffset->value() * mechToEncoder, 0.0, 1.0) : 0.0;

  if (auto* sensor = std::any_cast<FeedbackSensorType>(&*enc);
      sensor && *sensor == FeedbackSensorType::ABS) {
    m_dataPortEncoder = FeedbackSensorType::ABS;
    if (inverted) novaConfig.Add(thrifty::NovaConfig::AbsoluteDirection(*inverted ? thrifty::Motor::Direction::COUNTER_CLOCKWISE : thrifty::Motor::Direction::CLOCKWISE));
    if (zeroOffset) novaConfig.Add(thrifty::NovaConfig::AbsOffset(zeroOffsetRotations));
  } else if (sensor && *sensor == FeedbackSensorType::QUAD) {
    m_dataPortEncoder = FeedbackSensorType::QUAD;
    if (zeroOffset)
      throw exceptions::SmartMotorControllerConfigurationException(
          "Zero offset is only available for absolute encoders",
          "Zero offset could not be applied to the quadrature encoder",
          "WithExternalEncoderZeroOffset");
    if (inverted)
      novaConfig.Add(thrifty::NovaConfig::QuadratureDirection(*inverted ? thrifty::Motor::Direction::COUNTER_CLOCKWISE : thrifty::Motor::Direction::CLOCKWISE));
  } else if (auto* canEncoder = std::any_cast<thrifty::CanEncoder*>(&*enc);
             canEncoder && *canEncoder) {
    m_canEncoder = *canEncoder;
    thrifty::CanEncoderConfigBatch canEncoderConfig;
    if (inverted) canEncoderConfig.Add(thrifty::CanEncoderConfig::Direction(*inverted ? thrifty::Motor::Direction::COUNTER_CLOCKWISE : thrifty::Motor::Direction::CLOCKWISE));
    if (zeroOffset)
      canEncoderConfig.Add(
          thrifty::CanEncoderConfig::ZeroOffset(static_cast<float>(zeroOffsetRotations)));
    m_canEncoder->Configure(canEncoderConfig);
    if (m_externalEncoderSim)
      m_canEncoder->Configure(thrifty::CanEncoderConfig::SimMotor(*m_externalEncoderSim));
  } else {
    throw std::invalid_argument(
        "[ERROR] Unsupported external encoder: NovaWrapper accepts thrifty::Motor::"
        "FeedbackSensorType::ABS, thrifty::Motor::FeedbackSensorType::QUAD or thrifty::"
        "CanEncoder*");
  }
  if (useExternalFeedback) {
    m_feedbackSensor = m_canEncoder ? FeedbackSensorType::CAN_ENCODER : *m_dataPortEncoder;
    m_mechanismToFeedbackSensorRatio = mechToEncoder;
    novaConfig.Add(thrifty::NovaConfig::FeedbackSensor(m_feedbackSensor));
    if (m_canEncoder) {
      novaConfig.Add(thrifty::NovaConfig::CanEncoderId(m_canEncoder->Status().GetCanId()),
                     thrifty::NovaConfig::CanEncoderMode(thrifty::Motor::CanEncoderMode::ABSOLUTE),
                     thrifty::NovaConfig::CanEncoderSource(thrifty::Motor::CanEncoderSource::DIRECT));
    }
  }
}

// ---- Simulation -------------------------------------------------------------

void NovaWrapper::SetupSimulation() {
  if (!wpi::RobotBase::IsSimulation()) return;
  if (!m_motorSim) CreateMotorSim();
  if (auto startPos = m_config->GetStartingPosition()) m_simSupplier->SetMechanismPosition(*startPos);
}

void NovaWrapper::SimIterate() {
  if (!wpi::RobotBase::IsSimulation() || !m_simSupplier) return;
  // Step the physics only if the mechanism has not already stepped them this loop.
  if (!m_simSupplier->GetUpdatedSim()) {
    m_simSupplier->UpdateSim();
    m_simSupplier->StarveUpdateSim();
    simulation::BatterySim::CalculateVoltage(BatterySimKey(), m_simSupplier->GetSupplyCurrent());
  }
  m_nova->SimulationPeriodic();
  if (m_canEncoder) m_canEncoder->SimulationPeriodic();
}

// ---- Encoder sync -----------------------------------------------------------

std::optional<wpi::units::turn_t> NovaWrapper::GetExternalEncoderSensorPosition() {
  if (!m_dataPortEncoder && !m_canEncoder) return std::nullopt;
  const bool absolute = m_canEncoder || m_dataPortEncoder == FeedbackSensorType::ABS;
  double rotations = 0.0;
  if (m_canEncoder) {
    rotations = m_canEncoder->Status().GetPositionAbs();
  } else if (m_externalEncoderSim) {
    // ThriftyLib's Nova simulation does not move its data port encoders with the mechanism.
    rotations = m_externalEncoderSim->GetPosition();
  } else if (absolute) {
    rotations = m_nova->Status().GetAbsPosition();
  } else {
    rotations = m_nova->Status().GetQuadPosition();
  }
  // An absolute encoder reports angles in the continuous wrapping range, or [0, 1) rotations.
  if (absolute)
    rotations = wpi::math::InputModulus(rotations,
                                        m_config->GetWrappingMin().value_or(0_tr).value(),
                                        m_config->GetWrappingMax().value_or(1_tr).value());
  return wpi::units::turn_t{rotations};
}

std::optional<wpi::units::turns_per_second_t> NovaWrapper::GetExternalEncoderSensorVelocity() {
  if (!m_dataPortEncoder && !m_canEncoder) return std::nullopt;
  if (m_canEncoder) return wpi::units::turns_per_second_t{m_canEncoder->Status().GetVelocity()};
  // ThriftyLib's Nova simulation does not move its data port encoders with the mechanism.
  if (m_externalEncoderSim)
    return wpi::units::turns_per_second_t{m_externalEncoderSim->GetVelocity()};
  // The absolute encoder's velocity reads 0 until NovaConfig::AbsoluteFramePeriod enables its
  // frame.
  return wpi::units::turns_per_second_t{m_dataPortEncoder == FeedbackSensorType::ABS
                                            ? m_nova->Status().GetAbsVelocity()
                                            : m_nova->Status().GetQuadVelocity()};
}

void NovaWrapper::SeedRelativeEncoder() {
  if (auto mechanismAngle = GetExternalEncoderMechanismPosition())
    m_nova->SetIntPosition(mechanismAngle->value() * MechanismToRotorRatio());
}

void NovaWrapper::SynchronizeRelativeEncoder() {
  auto threshold = m_config->GetFeedbackSynchronizationThreshold();
  if (!threshold) return;
  auto externalEncoderAngle = GetExternalEncoderMechanismPosition();
  if (!externalEncoderAngle) return;
  if (std::abs((GetRelativeMechanismPosition() - *externalEncoderAngle).value()) > threshold->value())
    SeedRelativeEncoder();
}

// ---- Open-loop outputs ------------------------------------------------------

void NovaWrapper::SetDutyCycle(double dc) {
  m_nova->SetThrottle(dc);
  if (m_simSupplier) m_simSupplier->SetMechanismStatorDutyCycle(dc);
  if (dc == 0.0) {
    for (auto* f : m_looseFollowers) f->SetDutyCycle(dc);
  }
}

double NovaWrapper::GetDutyCycle() {
  return m_novaSim ? m_novaSim->GetDutyCycle() : m_nova->Status().GetAppliedPower();
}

void NovaWrapper::SetVoltage(wpi::units::volt_t voltage) {
  m_nova->SetVoltage(voltage);
  if (m_simSupplier) m_simSupplier->SetMechanismStatorVoltage(voltage);
}

wpi::units::volt_t NovaWrapper::GetVoltage() {
  if (m_novaSim) return m_novaSim->GetMechanismSupplyVoltage() * m_novaSim->GetDutyCycle();
  return wpi::units::volt_t{m_nova->Status().GetAppliedVoltage()};
}

// ---- Closed-loop setpoints --------------------------------------------------

void NovaWrapper::SetPosition(wpi::units::turn_t angle) {
  m_setpointVelocity.reset();
  m_setpointFeedforwardForce.reset();
  m_setpointPosition = angle;
  if (m_closedLoopOnNova) {
    // In mechanism rotations of the Nova's feedback sensor. The Nova keeps the mechanism within its
    // soft limits itself.
    const double target = NovaPositionTarget(angle);
    const auto request = (m_config->HasTrapezoidProfile() ? thrifty::MotorControl::MotionProfile(target)
                                                          : thrifty::MotorControl::VoltagePosition(target))
                             .WithSlot(m_novaSlot);
    m_nova->Control(request);
    if (m_novaSim) m_novaSim->SetControlRequest(request);
  }
  ForwardPositionToFollowers(angle);
}

void NovaWrapper::SetPosition(wpi::units::meter_t distance) {
  SetPosition(m_config->ConvertToMechanism(distance));
}

void NovaWrapper::SetVelocity(wpi::units::turns_per_second_t velocity) {
  m_setpointPosition.reset();
  m_setpointVelocity = velocity;
  m_setpointFeedforwardForce.reset();
  if (m_closedLoopOnNova) {
    const auto request =
        thrifty::MotorControl::VoltageVelocity(velocity.value()).WithSlot(m_novaSlot);
    m_nova->Control(request);
    if (m_novaSim) m_novaSim->SetControlRequest(request);
  }
  ForwardVelocityToFollowers(velocity);
}

void NovaWrapper::SetVelocity(wpi::units::turns_per_second_t velocity,
                              wpi::units::newton_t feedforwardForce) {
  m_setpointPosition.reset();
  m_setpointVelocity = velocity;
  m_setpointFeedforwardForce = feedforwardForce;
  if (m_closedLoopOnNova) {
    const auto request = thrifty::MotorControl::VoltageVelocity(velocity.value())
                             .WithFeedForward(m_config->ConvertToVoltage(m_motor, feedforwardForce).value())
                             .WithSlot(m_novaSlot);
    m_nova->Control(request);
    if (m_novaSim) m_novaSim->SetControlRequest(request);
  }
  for (auto* f : m_looseFollowers) f->SetVelocity(velocity, feedforwardForce);
}

void NovaWrapper::SetVelocity(wpi::units::meters_per_second_t velocity) {
  SetVelocity(m_config->ConvertToMechanism(velocity));
}

// ---- Encoder writes ---------------------------------------------------------

void NovaWrapper::SetEncoderPosition(wpi::units::turn_t angle) {
  const double externalEncoderRotations = angle.value() * MechanismToExternalEncoderRatio();
  if (m_dataPortEncoder == FeedbackSensorType::ABS) m_nova->SetAbsPosition(externalEncoderRotations);
  if (m_dataPortEncoder == FeedbackSensorType::QUAD)
    m_nova->SetQuadPosition(externalEncoderRotations);
  if (m_canEncoder && !wpi::RobotBase::IsSimulation()) {
    // Move the zero offset so the encoder reads the angle where it is now.
    auto status = m_canEncoder->Status();
    const double zeroOffset = wpi::math::InputModulus(
        status.GetPositionAbs() + status.GetZeroOffset() - externalEncoderRotations, 0.0, 1.0);
    m_canEncoder->Configure(thrifty::CanEncoderConfig::ZeroOffset(static_cast<float>(zeroOffset)));
  }
  m_nova->SetIntPosition(angle.value() * MechanismToRotorRatio());
  if (m_simSupplier) m_simSupplier->SetMechanismPosition(angle);
}

void NovaWrapper::SetEncoderPosition(wpi::units::meter_t distance) {
  SetEncoderPosition(m_config->ConvertToMechanism(distance));
}

void NovaWrapper::SetEncoderVelocity(wpi::units::turns_per_second_t velocity) {
  if (!wpi::RobotBase::IsSimulation())
    throw std::runtime_error("Thrifty Nova does not support setting encoder velocity.");
  if (m_simSupplier) m_simSupplier->SetMechanismVelocity(velocity);
}

void NovaWrapper::SetEncoderVelocity(wpi::units::meters_per_second_t velocity) {
  SetEncoderVelocity(m_config->ConvertToMechanism(velocity));
}

// ---- Encoder reads ----------------------------------------------------------

wpi::units::turn_t NovaWrapper::GetMechanismPosition() {
  if (m_config->GetUseExternalFeedback()) {
    if (auto external = GetExternalEncoderMechanismPosition()) return *external;
  }
  return GetRelativeMechanismPosition();
}

wpi::units::turns_per_second_t NovaWrapper::GetMechanismVelocity() {
  if (m_config->GetUseExternalFeedback()) {
    if (auto external = GetExternalEncoderMechanismVelocity()) return *external;
  }
  return GetRelativeMechanismVelocity();
}

wpi::units::turn_t NovaWrapper::GetRelativeMechanismPosition() {
  return GetRotorPosition() / MechanismToRotorRatio();
}

wpi::units::turns_per_second_t NovaWrapper::GetRelativeMechanismVelocity() {
  return GetRotorVelocity() / MechanismToRotorRatio();
}

wpi::units::turns_per_second_squared_t NovaWrapper::GetMechanismAcceleration() {
  return wpi::units::turns_per_second_squared_t{
      m_accelFilter.Derivative(GetMechanismVelocity().value())};
}

wpi::units::turn_t NovaWrapper::GetRotorPosition() {
  if (wpi::RobotBase::IsSimulation() && m_simSupplier) return m_simSupplier->GetRotorPosition();
  return wpi::units::turn_t{m_nova->Status().GetIntPosition()};
}

wpi::units::turns_per_second_t NovaWrapper::GetRotorVelocity() {
  if (wpi::RobotBase::IsSimulation() && m_simSupplier) return m_simSupplier->GetRotorVelocity();
  return wpi::units::turns_per_second_t{m_nova->Status().GetIntVelocity()};
}

wpi::units::meter_t NovaWrapper::GetMeasurementPosition() {
  return m_config->ConvertFromMechanism(GetMechanismPosition());
}
wpi::units::meters_per_second_t NovaWrapper::GetMeasurementVelocity() {
  return m_config->ConvertFromMechanism(GetMechanismVelocity());
}
wpi::units::meters_per_second_squared_t NovaWrapper::GetMeasurementAcceleration() {
  return m_config->ConvertFromMechanism(GetMechanismAcceleration());
}

std::optional<wpi::units::turn_t> NovaWrapper::GetExternalEncoderMechanismPosition() {
  auto sensor = GetExternalEncoderSensorPosition();
  if (!sensor) return std::nullopt;
  return *sensor / MechanismToExternalEncoderRatio();
}

std::optional<wpi::units::turns_per_second_t> NovaWrapper::GetExternalEncoderMechanismVelocity() {
  auto sensor = GetExternalEncoderSensorVelocity();
  if (!sensor) return std::nullopt;
  return *sensor / MechanismToExternalEncoderRatio();
}

// ---- Motor status -----------------------------------------------------------

std::optional<wpi::units::ampere_t> NovaWrapper::GetSupplyCurrent() {
  if (m_simSupplier) return m_simSupplier->GetSupplyCurrent();
  return wpi::units::ampere_t{m_nova->Status().GetCurrentSupply()};
}

wpi::units::ampere_t NovaWrapper::GetStatorCurrent() {
  if (m_simSupplier) return m_simSupplier->GetStatorCurrent();
  return wpi::units::ampere_t{m_nova->Status().GetCurrentStator()};
}

wpi::units::celsius_t NovaWrapper::GetTemperature() {
  return wpi::units::celsius_t{m_nova->Status().GetTemperature()};
}

wpi::math::DCMotor NovaWrapper::GetDCMotor() { return m_motor; }

// ---- Live-tuning setters ----------------------------------------------------

void NovaWrapper::SetZeroPower(MotorMode mode) {
  m_nova->Configure(thrifty::NovaConfig::BrakeMode(mode == MotorMode::BRAKE));
}

void NovaWrapper::SetMotorInverted(bool inverted) {
  m_config->WithMotorInverted(inverted);
  m_nova->SetInverted(inverted);
}

void NovaWrapper::SetEncoderInverted(bool) {
  throw std::runtime_error("Thrifty Nova internal encoder cannot be inverted.");
}

void NovaWrapper::ApplyFeedback(double kP, double kI, double kD) {
  m_config->WithFeedback(kP, kI, kD, m_slot);
  if (m_pid) {
    m_pid->SetP(kP);
    m_pid->SetI(kI);
    m_pid->SetD(kD);
  }
  if (m_closedLoopOnNova) {
    const double novaKp = kP / m_rotationsPerGainUnit;
    const double novaKi = kI / m_rotationsPerGainUnit;
    const double novaKd = kD / m_rotationsPerGainUnit;
    if (m_novaSlot == thrifty::MotorControl::Slot::SLOT1) {
      m_nova->Configure(thrifty::NovaConfig::Pid1::P(novaKp), thrifty::NovaConfig::Pid1::I(novaKi),
                        thrifty::NovaConfig::Pid1::D(novaKd));
    } else {
      m_nova->Configure(thrifty::NovaConfig::Pid0::P(novaKp), thrifty::NovaConfig::Pid0::I(novaKi),
                        thrifty::NovaConfig::Pid0::D(novaKd));
    }
  }
}

void NovaWrapper::SetKp(double kP) {
  const auto gains = m_config->GetSlotGains(m_slot);
  ApplyFeedback(kP, gains.kI, gains.kD);
  for (auto* f : m_looseFollowers) f->SetKp(kP);
}

void NovaWrapper::SetKi(double kI) {
  const auto gains = m_config->GetSlotGains(m_slot);
  ApplyFeedback(gains.kP, kI, gains.kD);
  for (auto* f : m_looseFollowers) f->SetKi(kI);
}

void NovaWrapper::SetKd(double kD) {
  const auto gains = m_config->GetSlotGains(m_slot);
  ApplyFeedback(gains.kP, gains.kI, kD);
  for (auto* f : m_looseFollowers) f->SetKd(kD);
}

void NovaWrapper::SetFeedback(double kP, double kI, double kD) {
  ApplyFeedback(kP, kI, kD);
  for (auto* f : m_looseFollowers) f->SetFeedback(kP, kI, kD);
}

void NovaWrapper::SetKs(double kS) {
  UpdateConfigFeedforward(
      *m_config, m_slot, [&](NativeFeedforward& v) { v.kS = kS;; },
      m_closedLoopOnNova ? m_nova : nullptr, m_novaSlot == thrifty::MotorControl::Slot::SLOT1);
  for (auto* f : m_looseFollowers) f->SetKs(kS);
}

void NovaWrapper::SetKv(double kV) {
  UpdateConfigFeedforward(
      *m_config, m_slot, [&](NativeFeedforward& v) { v.kV = kV;; },
      m_closedLoopOnNova ? m_nova : nullptr, m_novaSlot == thrifty::MotorControl::Slot::SLOT1);
  for (auto* f : m_looseFollowers) f->SetKv(kV);
}

void NovaWrapper::SetKa(double kA) {
  UpdateConfigFeedforward(
      *m_config, m_slot, [&](NativeFeedforward& v) { v.kA = kA;; },
      m_closedLoopOnNova ? m_nova : nullptr, m_novaSlot == thrifty::MotorControl::Slot::SLOT1);
  for (auto* f : m_looseFollowers) f->SetKa(kA);
}

void NovaWrapper::SetKg(double kG) {
  UpdateConfigFeedforward(
      *m_config, m_slot, [&](NativeFeedforward& v) { v.kG = kG;; },
      m_closedLoopOnNova ? m_nova : nullptr, m_novaSlot == thrifty::MotorControl::Slot::SLOT1);
  for (auto* f : m_looseFollowers) f->SetKg(kG);
}

void NovaWrapper::SetFeedforward(double kS, double kV, double kA, double kG) {
  UpdateConfigFeedforward(*m_config, m_slot, [&](NativeFeedforward& v) {
    v.kS = kS;
    v.kV = kV;
    v.kA = kA;
    v.kG = kG;
  }, m_closedLoopOnNova ? m_nova : nullptr, m_novaSlot == thrifty::MotorControl::Slot::SLOT1);
  for (auto* f : m_looseFollowers) f->SetFeedforward(kS, kV, kA, kG);
}

void NovaWrapper::SetStatorCurrentLimit(wpi::units::ampere_t limit) {
  m_config->WithStatorCurrentLimit(limit);
  m_nova->Configure(thrifty::NovaConfig::StatorCurrent(limit.value()));
  for (auto* f : m_looseFollowers) f->SetStatorCurrentLimit(limit);
}

void NovaWrapper::SetSupplyCurrentLimit(wpi::units::ampere_t limit) {
  m_config->WithSupplyCurrentLimit(limit);
  m_nova->Configure(thrifty::NovaConfig::SupplyCurrent(limit.value()));
  for (auto* f : m_looseFollowers) f->SetSupplyCurrentLimit(limit);
}

void NovaWrapper::SetClosedLoopRampRate(wpi::units::second_t r) {
  m_config->WithClosedLoopRampRate(r);
  m_nova->Configure(thrifty::NovaConfig::RampForward(r.value()),
                    thrifty::NovaConfig::RampReverse(r.value()));
  for (auto* f : m_looseFollowers) f->SetClosedLoopRampRate(r);
}

void NovaWrapper::SetOpenLoopRampRate(wpi::units::second_t r) {
  m_config->WithOpenLoopRampRate(r);
  m_nova->Configure(thrifty::NovaConfig::RampForward(r.value()),
                    thrifty::NovaConfig::RampReverse(r.value()));
  for (auto* f : m_looseFollowers) f->SetOpenLoopRampRate(r);
}

void NovaWrapper::SetMechanismUpperLimit(wpi::units::turn_t upper) {
  if (auto lower = m_config->GetMechanismLowerLimit()) m_config->WithMechanismLimits(*lower, upper);
  if (auto upperLimit = m_config->GetMechanismUpperLimit(),
      lowerLimit = m_config->GetMechanismLowerLimit();
      upperLimit && lowerLimit)
    m_nova->Configure(thrifty::NovaConfig::SoftLimits(upperLimit->value(), lowerLimit->value()));
  for (auto* f : m_looseFollowers) f->SetMechanismUpperLimit(upper);
}

void NovaWrapper::SetMechanismLowerLimit(wpi::units::turn_t lower) {
  if (auto upper = m_config->GetMechanismUpperLimit()) m_config->WithMechanismLimits(lower, *upper);
  if (auto upperLimit = m_config->GetMechanismUpperLimit(),
      lowerLimit = m_config->GetMechanismLowerLimit();
      upperLimit && lowerLimit)
    m_nova->Configure(thrifty::NovaConfig::SoftLimits(upperLimit->value(), lowerLimit->value()));
  for (auto* f : m_looseFollowers) f->SetMechanismLowerLimit(lower);
}

void NovaWrapper::SetMechanismLimits(wpi::units::turn_t lower, wpi::units::turn_t upper) {
  m_config->WithMechanismLimits(lower, upper);
  m_nova->Configure(thrifty::NovaConfig::SoftLimits(upper.value(), lower.value()));
  for (auto* f : m_looseFollowers) f->SetMechanismLimits(lower, upper);
}

void NovaWrapper::SetMechanismLimitsEnabled(bool enabled) {
  // The mechanism limits are the Nova's soft limits.
  m_nova->Configure(thrifty::NovaConfig::SoftLimitEnable(enabled));
  for (auto* f : m_looseFollowers) f->SetMechanismLimitsEnabled(enabled);
}

void NovaWrapper::SetMeasurementUpperLimit(wpi::units::meter_t upper) {
  auto lowerAngle = m_config->GetMechanismLowerLimit();
  if (!m_config->GetMechanismCircumference() || !lowerAngle) return;
  m_config->WithMeasurementLimits(m_config->ConvertFromMechanism(*lowerAngle), upper);
  if (auto upperLimit = m_config->GetMechanismUpperLimit(),
      lowerLimit = m_config->GetMechanismLowerLimit();
      upperLimit && lowerLimit)
    m_nova->Configure(thrifty::NovaConfig::SoftLimits(upperLimit->value(), lowerLimit->value()));
  for (auto* f : m_looseFollowers) f->SetMeasurementUpperLimit(upper);
}

void NovaWrapper::SetMeasurementLowerLimit(wpi::units::meter_t lower) {
  auto upperAngle = m_config->GetMechanismUpperLimit();
  if (!m_config->GetMechanismCircumference() || !upperAngle) return;
  m_config->WithMeasurementLimits(lower, m_config->ConvertFromMechanism(*upperAngle));
  if (auto upperLimit = m_config->GetMechanismUpperLimit(),
      lowerLimit = m_config->GetMechanismLowerLimit();
      upperLimit && lowerLimit)
    m_nova->Configure(thrifty::NovaConfig::SoftLimits(upperLimit->value(), lowerLimit->value()));
  for (auto* f : m_looseFollowers) f->SetMeasurementLowerLimit(lower);
}

void NovaWrapper::SetMotionProfileMaxVelocity(wpi::units::turns_per_second_t vel) {
  if (!m_config->GetVelocityTrapezoidalProfileInUse()) {
    // Keep the tuned constraints in the config, so tuning the next one keeps this one.
    if (auto accLin = m_config->GetTrapMaxAccelLinear(); accLin && !m_config->GetTrapMaxVelocityTurns()) {
      m_config->WithLinearTrapezoidProfile(m_config->ConvertFromMechanism(vel), *accLin);
    } else if (auto acc = m_config->GetTrapMaxAccelTurns()) {
      m_config->WithTrapezoidProfile(vel, *acc);
    }
  }
  if (m_closedLoopOnNova) {
    // The constraints are in the closed loop's units, per meter for a linear one.
    const double maxVelocity =
        (m_config->GetTrapMaxVelocityTurns() ? m_config->GetTrapMaxVelocityTurns()->value()
         : m_config->GetTrapMaxVelocityLinear() ? m_config->GetTrapMaxVelocityLinear()->value()
                                                : 0.0) *
        (m_config->GetTrapMaxVelocityTurns() ? 1.0 : m_rotationsPerGainUnit);
    const double maxAcceleration =
        (m_config->GetTrapMaxAccelTurns() ? m_config->GetTrapMaxAccelTurns()->value()
         : m_config->GetTrapMaxAccelLinear() ? m_config->GetTrapMaxAccelLinear()->value()
                                             : 0.0) *
        (m_config->GetTrapMaxAccelTurns() ? 1.0 : m_rotationsPerGainUnit);
    m_nova->Configure(thrifty::NovaConfig::Mp0::MaxVel(maxVelocity),
                      thrifty::NovaConfig::Mp0::MaxAccel(maxAcceleration),
                      thrifty::NovaConfig::Mp1::MaxVel(maxVelocity),
                      thrifty::NovaConfig::Mp1::MaxAccel(maxAcceleration));
  }
  for (auto* f : m_looseFollowers) f->SetMotionProfileMaxVelocity(vel);
}

void NovaWrapper::SetMotionProfileMaxVelocity(wpi::units::meters_per_second_t vel) {
  // Convert first so a missing circumference throws before anything changes.
  const auto mechanismVelocity = m_config->ConvertToMechanism(vel);
  if (!m_config->GetVelocityTrapezoidalProfileInUse()) {
    if (auto accLin = m_config->GetTrapMaxAccelLinear(); accLin && !m_config->GetTrapMaxVelocityTurns()) {
      m_config->WithLinearTrapezoidProfile(vel, *accLin);
    } else if (auto acc = m_config->GetTrapMaxAccelTurns()) {
      m_config->WithTrapezoidProfile(mechanismVelocity, *acc);
    }
  }
  if (m_closedLoopOnNova) {
    // The constraints are in the closed loop's units, per meter for a linear one.
    const double maxVelocity =
        (m_config->GetTrapMaxVelocityTurns() ? m_config->GetTrapMaxVelocityTurns()->value()
         : m_config->GetTrapMaxVelocityLinear() ? m_config->GetTrapMaxVelocityLinear()->value()
                                                : 0.0) *
        (m_config->GetTrapMaxVelocityTurns() ? 1.0 : m_rotationsPerGainUnit);
    const double maxAcceleration =
        (m_config->GetTrapMaxAccelTurns() ? m_config->GetTrapMaxAccelTurns()->value()
         : m_config->GetTrapMaxAccelLinear() ? m_config->GetTrapMaxAccelLinear()->value()
                                             : 0.0) *
        (m_config->GetTrapMaxAccelTurns() ? 1.0 : m_rotationsPerGainUnit);
    m_nova->Configure(thrifty::NovaConfig::Mp0::MaxVel(maxVelocity),
                      thrifty::NovaConfig::Mp0::MaxAccel(maxAcceleration),
                      thrifty::NovaConfig::Mp1::MaxVel(maxVelocity),
                      thrifty::NovaConfig::Mp1::MaxAccel(maxAcceleration));
  }
  for (auto* f : m_looseFollowers) f->SetMotionProfileMaxVelocity(vel);
}

void NovaWrapper::SetMotionProfileMaxAcceleration(wpi::units::turns_per_second_squared_t acc) {
  const bool linear = m_config->GetTrapMaxVelocityLinear() && !m_config->GetTrapMaxVelocityTurns();
  if (m_config->GetVelocityTrapezoidalProfileInUse()) {
    // A velocity profile's acceleration is its first constraint; its jerk the second.
    if (linear) {
      m_config->WithVelocityTrapezoidProfile(
          m_config->ConvertFromMechanism(acc),
          meters_per_second_cubed_t{m_config->GetTrapMaxAccelLinear()->value()});
    } else if (auto jerk = m_config->GetTrapMaxAccelTurns()) {
      m_config->WithVelocityTrapezoidProfile(
          acc, wpi::units::angular_jerk::turns_per_second_cubed_t{jerk->value()});
    }
  } else if (linear) {
    m_config->WithLinearTrapezoidProfile(*m_config->GetTrapMaxVelocityLinear(),
                                         m_config->ConvertFromMechanism(acc));
  } else if (auto vel = m_config->GetTrapMaxVelocityTurns()) {
    m_config->WithTrapezoidProfile(*vel, acc);
  }
  if (m_closedLoopOnNova) {
    // The constraints are in the closed loop's units, per meter for a linear one.
    const double maxVelocity =
        (m_config->GetTrapMaxVelocityTurns() ? m_config->GetTrapMaxVelocityTurns()->value()
         : m_config->GetTrapMaxVelocityLinear() ? m_config->GetTrapMaxVelocityLinear()->value()
                                                : 0.0) *
        (m_config->GetTrapMaxVelocityTurns() ? 1.0 : m_rotationsPerGainUnit);
    const double maxAcceleration =
        (m_config->GetTrapMaxAccelTurns() ? m_config->GetTrapMaxAccelTurns()->value()
         : m_config->GetTrapMaxAccelLinear() ? m_config->GetTrapMaxAccelLinear()->value()
                                             : 0.0) *
        (m_config->GetTrapMaxAccelTurns() ? 1.0 : m_rotationsPerGainUnit);
    m_nova->Configure(thrifty::NovaConfig::Mp0::MaxVel(maxVelocity),
                      thrifty::NovaConfig::Mp0::MaxAccel(maxAcceleration),
                      thrifty::NovaConfig::Mp1::MaxVel(maxVelocity),
                      thrifty::NovaConfig::Mp1::MaxAccel(maxAcceleration));
  }
  for (auto* f : m_looseFollowers) f->SetMotionProfileMaxAcceleration(acc);
}

void NovaWrapper::SetMotionProfileMaxAcceleration(wpi::units::meters_per_second_squared_t acc) {
  const auto mechanismAcceleration = m_config->ConvertToMechanism(acc);
  const bool linear = m_config->GetTrapMaxVelocityLinear() && !m_config->GetTrapMaxVelocityTurns();
  if (m_config->GetVelocityTrapezoidalProfileInUse()) {
    if (linear) {
      m_config->WithVelocityTrapezoidProfile(
          acc, meters_per_second_cubed_t{m_config->GetTrapMaxAccelLinear()->value()});
    } else if (auto jerk = m_config->GetTrapMaxAccelTurns()) {
      m_config->WithVelocityTrapezoidProfile(
          mechanismAcceleration,
          wpi::units::angular_jerk::turns_per_second_cubed_t{jerk->value()});
    }
  } else if (linear) {
    m_config->WithLinearTrapezoidProfile(*m_config->GetTrapMaxVelocityLinear(), acc);
  } else if (auto vel = m_config->GetTrapMaxVelocityTurns()) {
    m_config->WithTrapezoidProfile(*vel, mechanismAcceleration);
  }
  if (m_closedLoopOnNova) {
    // The constraints are in the closed loop's units, per meter for a linear one.
    const double maxVelocity =
        (m_config->GetTrapMaxVelocityTurns() ? m_config->GetTrapMaxVelocityTurns()->value()
         : m_config->GetTrapMaxVelocityLinear() ? m_config->GetTrapMaxVelocityLinear()->value()
                                                : 0.0) *
        (m_config->GetTrapMaxVelocityTurns() ? 1.0 : m_rotationsPerGainUnit);
    const double maxAcceleration =
        (m_config->GetTrapMaxAccelTurns() ? m_config->GetTrapMaxAccelTurns()->value()
         : m_config->GetTrapMaxAccelLinear() ? m_config->GetTrapMaxAccelLinear()->value()
                                             : 0.0) *
        (m_config->GetTrapMaxAccelTurns() ? 1.0 : m_rotationsPerGainUnit);
    m_nova->Configure(thrifty::NovaConfig::Mp0::MaxVel(maxVelocity),
                      thrifty::NovaConfig::Mp0::MaxAccel(maxAcceleration),
                      thrifty::NovaConfig::Mp1::MaxVel(maxVelocity),
                      thrifty::NovaConfig::Mp1::MaxAccel(maxAcceleration));
  }
  for (auto* f : m_looseFollowers) f->SetMotionProfileMaxAcceleration(acc);
}

void NovaWrapper::SetMotionProfileMaxJerk(
    wpi::units::angular_jerk::turns_per_second_cubed_t maxJerk) {
  if (m_config->GetVelocityTrapezoidalProfileInUse()) {
    if (auto accLin = m_config->GetTrapMaxVelocityLinear();
        accLin && !m_config->GetTrapMaxVelocityTurns()) {
      m_config->WithVelocityTrapezoidProfile(
          wpi::units::meters_per_second_squared_t{accLin->value()},
          m_config->ConvertFromMechanism(maxJerk));
    } else if (auto acc = m_config->GetTrapMaxVelocityTurns()) {
      m_config->WithVelocityTrapezoidProfile(
          wpi::units::turns_per_second_squared_t{acc->value()}, maxJerk);
    }
  }
  for (auto* f : m_looseFollowers) f->SetMotionProfileMaxJerk(maxJerk);
}

void NovaWrapper::SetExponentialProfile(std::optional<double> kV, std::optional<double> kA,
                                        std::optional<wpi::units::volt_t> maxInput) {
  if (!m_config->GetExponentialProfile()) return;
  const double newKV = kV.value_or(m_config->GetExponentialProfileKV().value_or(0.0));
  const double newKA = kA.value_or(m_config->GetExponentialProfileKA().value_or(0.0));
  const wpi::units::volt_t newMaxInput =
      maxInput.value_or(m_config->GetExponentialProfileMaxInput().value_or(12_V));
  // Keep the tuned constraints in the config, so tuning the next one keeps this one.
  m_config->WithExponentialProfile(newKV, newKA, newMaxInput);
  for (auto* f : m_looseFollowers) f->SetExponentialProfile(kV, kA, maxInput);
}

void NovaWrapper::SetClosedLoopSlot(ClosedLoopControllerSlot slot) {
  m_slot = slot;
  // SLOT_2 and SLOT_3 have no gains on the Nova; ApplyConfig rejects any.
  m_novaSlot = slot == Slot::SLOT_1 ? thrifty::MotorControl::Slot::SLOT1
                                    : thrifty::MotorControl::Slot::SLOT0;
  LoadClosedLoopController(*m_config);
  ConfigureSoftwarePID(*m_config);
  for (auto* f : m_looseFollowers) f->SetClosedLoopSlot(slot);
}

void NovaWrapper::SetMechanismGearing(const gearing::MechanismGearing& gearing) {
  SmartMotorController::SetMechanismGearing(gearing);
  if (m_feedbackSensor == FeedbackSensorType::INTERNAL) {
    m_mechanismToFeedbackSensorRatio = MechanismToRotorRatio();
    m_nova->Configure(thrifty::NovaConfig::SensorToMechanismRatio(m_mechanismToFeedbackSensorRatio));
  }
  if (wpi::RobotBase::IsSimulation()) {
    // The simulated motor's reduction comes from the gearing.
    const auto position = m_simSupplier ? m_simSupplier->GetMechanismPosition() : wpi::units::turn_t{0.0};
    CreateMotorSim();
    m_simSupplier->SetMechanismPosition(position);
  }
}

void NovaWrapper::SetMechanismCircumference(wpi::units::meter_t circumference) {
  SmartMotorController::SetMechanismCircumference(circumference);
  for (auto* f : m_looseFollowers) f->SetMechanismCircumference(circumference);
}

SmartMotorControllerConfig& NovaWrapper::GetConfig() { return *m_config; }
void* NovaWrapper::GetMotorController() { return m_nova; }
void* NovaWrapper::GetMotorControllerConfig() {
  return m_vendorConfig ? &m_vendorConfig.value() : nullptr;
}

}  // namespace yams::motorcontrollers::local
