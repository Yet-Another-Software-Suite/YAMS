// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/remote/TalonFXWrapper.hpp"

#include <ctre/unit/pid_ff.h>

#include <chrono>
#include <cstdint>
#include <ctre/phoenix6/TalonFXS.hpp>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <wpi/framework/RobotBase.hpp>
#include <wpi/math/system/Models.hpp>
#include <wpi/units/angular_jerk.hpp>
#include <wpi/units/dimensionless.hpp>
#include <wpi/units/moment_of_inertia.hpp>

#include "yams/exceptions.hpp"
#include "yams/math/LQRController.hpp"
#include "yams/motorcontrollers/simulation/BatterySim.hpp"
#include "yams/motorcontrollers/simulation/DCMotorSimSupplier.hpp"

using namespace ctre::phoenix6;

namespace yams::motorcontrollers::remote {

namespace {

using Slot = SmartMotorControllerConfig::ClosedLoopControllerSlot;
using ConfigException = exceptions::SmartMotorControllerConfigurationException;
using TurnKv = wpi::math::SimpleMotorFeedforward<wpi::units::turns>::kv_unit;
using TurnKa = wpi::math::SimpleMotorFeedforward<wpi::units::turns>::ka_unit;

constexpr Slot kAllSlots[] = {Slot::SLOT_0, Slot::SLOT_1, Slot::SLOT_2, Slot::SLOT_3};

ConfigException Slot3Unsupported() {
  return ConfigException("Slot 3 is not available on TalonFX", "Cannot use Slot 3 on TalonFX", "");
}

/** Call @p f with the TalonFX slot configs of @p slot; the TalonFX has no SLOT_3. */
template <typename F>
void WithSlotConfigs(configs::TalonFXConfiguration& cfg, Slot slot, F&& f) {
  switch (slot) {
    case Slot::SLOT_0:
      f(cfg.Slot0);
      return;
    case Slot::SLOT_1:
      f(cfg.Slot1);
      return;
    case Slot::SLOT_2:
      f(cfg.Slot2);
      return;
    case Slot::SLOT_3:
      break;
  }
  throw Slot3Unsupported();
}

/** Feedforward gains of a slot (kV/kA per mechanism rotation, or per meter if linear). */
struct SlotFeedforward {
  double kS{0.0}, kV{0.0}, kA{0.0}, kG{0.0};
  bool arm{false};
  bool elevator{false};
};

std::optional<SlotFeedforward> GetSlotFeedforward(const SmartMotorControllerConfig& config,
                                                  Slot slot) {
  if (auto ff = config.GetArmFeedforward(slot)) {
    // ArmFeedforward gains are per radian; the TalonFX's are per mechanism rotation.
    return SlotFeedforward{ff->GetKs().value(),
                           wpi::units::unit_t<TurnKv>{ff->GetKv()}.value(),
                           wpi::units::unit_t<TurnKa>{ff->GetKa()}.value(),
                           ff->GetKg().value(),
                           true,
                           false};
  }
  if (auto ff = config.GetElevatorFeedforward(slot)) {
    return SlotFeedforward{ff->GetKs().value(),
                           ff->GetKv().value(),
                           ff->GetKa().value(),
                           ff->GetKg().value(),
                           false,
                           true};
  }
  if (auto ff = config.GetSimpleFeedforward(slot)) {
    return SlotFeedforward{
        ff->GetKs().value(), ff->GetKv().value(), ff->GetKa().value(), 0.0, false, false};
  }
  return std::nullopt;
}

/** Feedforward gains in the configured feedforward's own units (ArmFeedforward per radian). */
struct NativeFeedforward {
  double kS{0.0}, kV{0.0}, kA{0.0}, kG{0.0};
};

/** Apply @p update to the slot's configured feedforward and store it back in the config. */
template <typename F>
bool UpdateConfigFeedforward(SmartMotorControllerConfig& config, Slot slot, F&& update) {
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
    return true;
  }
  if (auto ff = config.GetElevatorFeedforward(slot)) {
    NativeFeedforward v{ff->GetKs().value(), ff->GetKv().value(), ff->GetKa().value(),
                        ff->GetKg().value()};
    update(v);
    using Elevator = wpi::math::ElevatorFeedforward;
    ff->SetKs(wpi::units::volt_t{v.kS});
    ff->SetKv(wpi::units::unit_t<Elevator::kv_unit>{v.kV});
    ff->SetKa(wpi::units::unit_t<Elevator::ka_unit>{v.kA});
    ff->SetKg(wpi::units::volt_t{v.kG});
    config.WithFeedforward(*ff, slot);
    return true;
  }
  if (auto ff = config.GetSimpleFeedforward(slot)) {
    NativeFeedforward v{ff->GetKs().value(), ff->GetKv().value(), ff->GetKa().value(), 0.0};
    update(v);
    ff->SetKs(wpi::units::volt_t{v.kS});
    ff->SetKv(wpi::units::unit_t<TurnKv>{v.kV});
    ff->SetKa(wpi::units::unit_t<TurnKa>{v.kA});
    config.WithFeedforward(*ff, slot);
    return true;
  }
  return false;
}

/** Convert an ArmFeedforward kV (per radian) to per rotation; other feedforwards unchanged. */
double KvPerRotation(const SmartMotorControllerConfig& config, Slot slot, double kV) {
  if (config.GetArmFeedforward(slot)) {
    return wpi::units::unit_t<TurnKv>{wpi::units::unit_t<wpi::math::ArmFeedforward::kv_unit>{kV}}
        .value();
  }
  return kV;
}

/** Convert an ArmFeedforward kA (per radian) to per rotation; other feedforwards unchanged. */
double KaPerRotation(const SmartMotorControllerConfig& config, Slot slot, double kA) {
  if (config.GetArmFeedforward(slot)) {
    return wpi::units::unit_t<TurnKa>{wpi::units::unit_t<wpi::math::ArmFeedforward::ka_unit>{kA}}
        .value();
  }
  return kA;
}

void Sleep10ms() { std::this_thread::sleep_for(std::chrono::milliseconds(10)); }

}  // namespace

// ---- Construction -----------------------------------------------------------

TalonFXWrapper::TalonFXWrapper(hardware::TalonFX* talon, wpi::math::DCMotor dcMotor,
                               SmartMotorControllerConfig* config)
    : SmartMotorController(),
      m_talon(talon),
      m_dcMotor(dcMotor),
      m_reportStatusSignalErrors(!wpi::RobotBase::IsSimulation()) {
  m_config = config;
  // Keep a simulation motor the user chose.
  if (!m_config->GetSimMotor()) m_config->WithSimMotor(dcMotor);

  m_rioControllerAlert.emplace("YAMS", AlertId("ClosedLoop"),
                               GetName() + " closed loop controller is running on the SystemCore.",
                               wpi::util::Alert::Level::MEDIUM);
  m_startingPositionExternalEncoderAlert.emplace(
      "YAMS", AlertId("StartingPosition"),
      GetName() + " starting position is not applied because an external encoder is used!",
      wpi::util::Alert::Level::HIGH);
  m_zeroOffsetNoExternalEncoderAlert.emplace(
      "YAMS", AlertId("ZeroOffset"),
      GetName() + " zero offset is not supported without an external encoder.",
      wpi::util::Alert::Level::HIGH);
  m_discontinuityPointNoExternalEncoderAlert.emplace(
      "YAMS", AlertId("DiscontinuityPoint"),
      GetName() + " discontinuity point is not supported without an external encoder.",
      wpi::util::Alert::Level::HIGH);

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

TalonFXWrapper::~TalonFXWrapper() {
  Close();
  if (m_rioControllerAlert) m_rioControllerAlert->Set(false);
  if (m_startingPositionExternalEncoderAlert) m_startingPositionExternalEncoderAlert->Set(false);
  if (m_zeroOffsetNoExternalEncoderAlert) m_zeroOffsetNoExternalEncoderAlert->Set(false);
  if (m_discontinuityPointNoExternalEncoderAlert)
    m_discontinuityPointNoExternalEncoderAlert->Set(false);
}

// ---- Helpers ----------------------------------------------------------------

std::string TalonFXWrapper::AlertId(const std::string& alertType) const {
  const std::string prefix = "TalonFX(" + std::to_string(m_talon->GetDeviceID()) + ")";
  if (auto name = m_config->GetTelemetryName()) return prefix + "_" + alertType + "_" + *name;
  return prefix + "_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)) + "_" + alertType;
}

void TalonFXWrapper::EnsureRequest(const std::function<ctre::phoenix::StatusCode()>& request) {
  for (int i = 0; i < 8; i++) {
    if (request().IsOK()) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

template <typename Group>
ctre::phoenix::StatusCode TalonFXWrapper::ApplyGroup(const Group& group) {
  auto status = m_talon->GetConfigurator().Apply(group);
  for (int i = 0; i < 10 && !status.IsOK(); i++) {
    Sleep10ms();
    status = m_talon->GetConfigurator().Apply(group);
  }
  return status;
}

ctre::phoenix::StatusCode TalonFXWrapper::ForceConfigApply() { return ApplyGroup(m_talonConfig); }

void TalonFXWrapper::SetFOC(bool foc) {
  auto unsupported = [this](std::string_view name) {
    return ConfigException("TalonFX(" + std::to_string(m_talon->GetDeviceID()) +
                               ") does not support toggling FOC on the '" + std::string{name} +
                               "' control request!",
                           "Cannot use given control request", "WithVendorControlRequest()");
  };
  auto setFOC = [&](auto& req) {
    if constexpr (requires { req.WithEnableFOC(foc); }) {
      req.WithEnableFOC(foc);
    } else {
      throw unsupported(req.GetName());
    }
  };
  std::visit(setFOC, m_positionReq);
  std::visit(setFOC, m_velocityReq);
}

TalonFXWrapper& TalonFXWrapper::EnableFOC() {
  SetFOC(true);
  return *this;
}

TalonFXWrapper& TalonFXWrapper::DisableFOC() {
  SetFOC(false);
  return *this;
}

bool TalonFXWrapper::UseCANdiPWM1() const {
  // Read the configuration being applied: YAMS applies the Fused source; Sync and Remote are the
  // ones users select with a vendor config.
  const auto source = m_talonConfig.Feedback.FeedbackSensorSource;
  const bool configured = source == signals::FeedbackSensorSourceValue::FusedCANdiPWM1 ||
                          source == signals::FeedbackSensorSourceValue::SyncCANdiPWM1 ||
                          source == signals::FeedbackSensorSourceValue::RemoteCANdiPWM1;
  if (configured && !m_candi)
    throw std::invalid_argument(
        "[ERROR] CANdi PWM1 has been configured but is not present in SmartMotorControllerConfig!");
  return configured;
}

bool TalonFXWrapper::UseCANdiPWM2() const {
  const auto source = m_talonConfig.Feedback.FeedbackSensorSource;
  const bool configured = source == signals::FeedbackSensorSourceValue::FusedCANdiPWM2 ||
                          source == signals::FeedbackSensorSourceValue::SyncCANdiPWM2 ||
                          source == signals::FeedbackSensorSourceValue::RemoteCANdiPWM2;
  if (configured && !m_candi)
    throw std::invalid_argument(
        "[ERROR] CANdi PWM2 has been configured but is not present in SmartMotorControllerConfig!");
  return configured;
}

void TalonFXWrapper::SetUpdateFrequency(wpi::units::hertz_t frequency) {
  m_talon->GetPosition(false).SetUpdateFrequency(frequency);
  m_talon->GetVelocity(false).SetUpdateFrequency(frequency);
  m_talon->GetAcceleration(false).SetUpdateFrequency(frequency);
  m_talon->GetDutyCycle(false).SetUpdateFrequency(frequency);
  m_talon->GetStatorCurrent(false).SetUpdateFrequency(frequency);
  m_talon->GetSupplyCurrent(false).SetUpdateFrequency(frequency);
  m_talon->GetMotorVoltage(false).SetUpdateFrequency(frequency);
  m_talon->GetRotorPosition(false).SetUpdateFrequency(frequency);
  m_talon->GetRotorVelocity(false).SetUpdateFrequency(frequency);
  m_talon->GetDeviceTemp(false).SetUpdateFrequency(frequency);
}

double TalonFXWrapper::PositionGainUnitsPerRotation() const {
  // Gains of a linear mechanism are per meter; the TalonFX's are per mechanism rotation.
  return m_config->GetLinearClosedLoopControllerUse()
             ? m_config->ConvertToMechanism(wpi::units::meter_t{1.0}).value()
             : 1.0;
}

double TalonFXWrapper::VelocityGainUnitsPerRotation() const {
  return m_config->GetLinearClosedLoopControllerUse()
             ? m_config->ConvertToMechanism(wpi::units::meters_per_second_t{1.0}).value()
             : 1.0;
}

double TalonFXWrapper::AccelerationGainUnitsPerRotation() const {
  return m_config->GetLinearClosedLoopControllerUse()
             ? m_config->ConvertToMechanism(wpi::units::meters_per_second_squared_t{1.0}).value()
             : 1.0;
}

void TalonFXWrapper::WriteSlotPID(ClosedLoopControllerSlot slot, double kP, double kI, double kD) {
  const double scale = PositionGainUnitsPerRotation();
  WithSlotConfigs(m_talonConfig, slot, [&](auto& s) {
    s.kP = kP / scale;
    s.kI = kI / scale;
    s.kD = kD / scale;
  });
}

void TalonFXWrapper::WriteSlotGains(const SmartMotorControllerConfig& config) {
  // Only slots with gains configured are written, so the rest keep the device's values.
  for (auto slot : kAllSlots) {
    const auto gains = config.GetSlotGains(slot);
    if (slot != Slot::SLOT_3 && (gains.kP != 0.0 || gains.kI != 0.0 || gains.kD != 0.0)) {
      WriteSlotPID(slot, gains.kP, gains.kI, gains.kD);
    }
    if (auto ff = GetSlotFeedforward(config, slot)) {
      const double velocityScale = VelocityGainUnitsPerRotation();
      const double accelerationScale = AccelerationGainUnitsPerRotation();
      WithSlotConfigs(m_talonConfig, slot, [&](auto& s) {
        s.kS = ff->kS;
        s.kV = ff->kV / velocityScale;
        s.kA = ff->kA / accelerationScale;
        s.kG = ff->kG;
        if (ff->arm)
          s.GravityType = signals::GravityTypeValue::Arm_Cosine;
        else if (ff->elevator)
          s.GravityType = signals::GravityTypeValue::Elevator_Static;
      });
    }
  }
}

void TalonFXWrapper::WriteMotionMagic(const SmartMotorControllerConfig& config) {
  if (config.HasExponentialProfile() || config.HasLinearExponentialProfile()) {
    // Stored per mechanism rotation, also for a linear (elevator) profile.
    if (auto kV = config.GetExponentialProfileKV())
      m_talonConfig.MotionMagic.MotionMagicExpo_kV = ctre::unit::volts_per_turn_per_second_t{*kV};
    if (auto kA = config.GetExponentialProfileKA())
      m_talonConfig.MotionMagic.MotionMagicExpo_kA =
          ctre::unit::volts_per_turn_per_second_squared_t{*kA};
  }
  if (!config.HasTrapezoidProfile()) return;
  // First and second profile constraints in mechanism units: velocity and acceleration for a
  // position profile; acceleration and jerk for a velocity profile.
  double first = 0.0;
  double second = 0.0;
  if (auto linVel = config.GetTrapMaxVelocityLinear();
      linVel && !config.GetTrapMaxVelocityTurns()) {
    first = config.ConvertToMechanism(*linVel).value();
    second = config.ConvertToMechanism(*config.GetTrapMaxAccelLinear()).value();
  } else if (auto vel = config.GetTrapMaxVelocityTurns()) {
    first = vel->value();
    second = config.GetTrapMaxAccelTurns()->value();
  } else {
    return;
  }
  auto& motionMagic = m_talonConfig.MotionMagic;
  if (config.GetVelocityTrapezoidalProfileInUse()) {
    motionMagic.MotionMagicAcceleration = wpi::units::turns_per_second_squared_t{first};
    motionMagic.MotionMagicJerk = wpi::units::angular_jerk::turns_per_second_cubed_t{second};
  } else {
    motionMagic.MotionMagicCruiseVelocity = wpi::units::turns_per_second_t{first};
    motionMagic.MotionMagicAcceleration = wpi::units::turns_per_second_squared_t{second};
  }
}

void TalonFXWrapper::WriteSensorRatios(const SmartMotorControllerConfig& config) {
  const double mechanismToRotor =
      config.GetMotorGearing().value_or(gearing::MechanismGearing::kOne).GetMechanismToRotorRatio();
  if (m_cancoder || m_candi) {
    const auto externalGearing =
        config.GetExternalEncoderGearing().value_or(gearing::MechanismGearing::kOne);
    m_talonConfig.Feedback.RotorToSensorRatio =
        mechanismToRotor * externalGearing.GetRotorToMechanismRatio();
    m_talonConfig.Feedback.SensorToMechanismRatio = externalGearing.GetMechanismToRotorRatio();
  } else {
    m_talonConfig.Feedback.RotorToSensorRatio = 1.0;
    m_talonConfig.Feedback.SensorToMechanismRatio = mechanismToRotor;
  }
}

// ---- Configuration ----------------------------------------------------------

bool TalonFXWrapper::ApplyConfig(const SmartMotorControllerConfig& config) {
  // Like the Java wrapper, the applied config becomes this controller's config.
  m_config = const_cast<SmartMotorControllerConfig*>(&config);
  config.ResetValidationCheck();

  // Start from the vendor config, or else from the device's configuration so settings YAMS does
  // not manage are kept (unless the previous configuration is to be reset).
  const bool resetPreviousConfig = config.GetResetPreviousConfig();
  if (auto vc = config.GetVendorConfig(); vc.has_value()) {
    auto* vendorConfig = std::any_cast<configs::TalonFXConfiguration>(&vc.value());
    if (!vendorConfig)
      throw ConfigException(
          "TalonFXConfiguration is the only acceptable vendor config type for TalonFXWrapper",
          "Vendor config is unable to be applied", "WithVendorConfig(TalonFXConfiguration{})");
    m_talonConfig = *vendorConfig;
  } else if (!resetPreviousConfig) {
    m_talon->GetConfigurator().Refresh(m_talonConfig);
  }

  m_rioControllerAlert->Set(false);
  m_startingPositionExternalEncoderAlert->Set(false);
  m_zeroOffsetNoExternalEncoderAlert->Set(false);
  m_discontinuityPointNoExternalEncoderAlert->Set(false);
  LoadLooselyCoupledFollowers();

  // LQR and software PID from the active gain slot for IterateClosedLoopController.
  auto gains = config.GetSlotGains(m_slot);
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
  ConfigureSoftwarePID(config);

  // Status signals follow a non-default simulation period.
  if (wpi::RobotBase::IsSimulation() && config.GetSimulationPeriod() != 20_ms)
    SetUpdateFrequency(1.0 / config.GetSimulationPeriod());

  // Closed loop gains and motion profiles, in mechanism rotations.
  WriteSlotGains(config);
  WriteMotionMagic(config);
  const int slotIndex = static_cast<int>(m_slot);
  m_positionReq = controls::PositionVoltage{0_tr}.WithSlot(slotIndex);
  m_velocityReq = controls::VelocityVoltage{0_tps}.WithSlot(slotIndex);
  if (config.HasExponentialProfile() || config.HasLinearExponentialProfile()) {
    m_positionReq = controls::MotionMagicExpoVoltage{0_tr}.WithSlot(slotIndex);
  }
  if (config.HasTrapezoidProfile()) {
    m_positionReq = controls::MotionMagicVoltage{0_tr}.WithSlot(slotIndex);
    m_velocityReq = controls::MotionMagicVelocityVoltage{0_tps}.WithSlot(slotIndex);
  } else if (!config.HasExponentialProfile() && !config.HasLinearExponentialProfile()) {
    // Without a profile, kS follows the closed loop error sign.
    using signals::StaticFeedforwardSignValue;
    m_talonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue::UseClosedLoopSign;
    m_talonConfig.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue::UseClosedLoopSign;
    m_talonConfig.Slot2.StaticFeedforwardSign = StaticFeedforwardSignValue::UseClosedLoopSign;
  }

  // Phoenix 6 has no LQR: hand control to the RoboRIO when one is configured.
  if (m_lqr) {
    if (config.GetClosedLoopTolerance())
      throw std::invalid_argument("[ERROR] Cannot set closed-loop controller error tolerance on " +
                                  GetName());
    m_rioControllerAlert->Set(true);
    if (m_closedLoopControllerThread) {
      StopClosedLoopController();
      m_closedLoopControllerThread.reset();
    }
    m_closedLoopControllerThread =
        std::make_unique<wpi::Notifier>([this] { IterateClosedLoopController(); });
    if (auto name = config.GetTelemetryName(); name) m_closedLoopControllerThread->SetName(*name);
    if (config.GetMotorControllerMode() == ControlMode::CLOSED_LOOP) {
      StartClosedLoopController();
    } else if (config.GetClosedLoopControlPeriod()) {
      throw std::invalid_argument(
          "[Error] Closed loop control period is only supported in closed loop mode.");
    }
  } else if (m_closedLoopControllerThread) {
    StopClosedLoopController();
    m_closedLoopControllerThread.reset();
  }

  if (config.GetClosedLoopTolerance())
    throw ConfigException("Closed loop tolerance is not available on TalonFX",
                          "Cannot set closed loop tolerance on TalonFX", "WithClosedLoopTolerance");

  const bool closedLoop = config.GetMotorControllerMode() == ControlMode::CLOSED_LOOP;

  // Motor output
  if (auto inv = config.GetMotorInverted(); inv)
    m_talonConfig.MotorOutput.Inverted = *inv ? signals::InvertedValue::Clockwise_Positive
                                              : signals::InvertedValue::CounterClockwise_Positive;
  if (auto zp = config.GetZeroPower(); zp)
    m_talonConfig.MotorOutput.NeutralMode = *zp == MotorMode::BRAKE
                                                ? signals::NeutralModeValue::Brake
                                                : signals::NeutralModeValue::Coast;
  if (auto maxV = config.GetClosedLoopControllerMaximumVoltage(); maxV) {
    m_talonConfig.Voltage.PeakForwardVoltage = *maxV;
    m_talonConfig.Voltage.PeakReverseVoltage = -*maxV;
  }

  // Ramp rates, for every control output type.
  if (auto r = config.GetClosedLoopRampRate(); r) {
    m_talonConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = *r;
    m_talonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = *r;
    m_talonConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = *r;
  }
  if (auto r = config.GetOpenLoopRampRate(); r) {
    m_talonConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = *r;
    m_talonConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = *r;
    m_talonConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = *r;
  }

  // Current limits
  if (auto stator = config.GetStatorCurrentLimit(); stator) {
    m_talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    m_talonConfig.CurrentLimits.StatorCurrentLimit = *stator;
  }
  if (auto supply = config.GetSupplyCurrentLimit(); supply) {
    m_talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_talonConfig.CurrentLimits.SupplyCurrentLimit = *supply;
  }

  // Soft limits, enforced in closed loop mode only.
  if (auto upper = config.GetMechanismUpperLimit(); upper) {
    m_talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = closedLoop;
    m_talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = *upper;
  }
  if (auto lower = config.GetMechanismLowerLimit(); lower) {
    m_talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = closedLoop;
    m_talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = *lower;
  }

  ApplyExternalEncoder(config);

  // Continuous wrapping. Set either way: the configuration starts from the device's previous one.
  m_talonConfig.ClosedLoopGeneral.ContinuousWrap = config.GetContinuousWrapping().has_value();

  if (config.GetEncoderInverted())
    throw ConfigException("Integrated encoder phase cannot be set", "Cannot configure TalonFX!",
                          "WithEncoderInverted(false)");

  ApplyFollowers(config);
  ApplyVendorControlRequest(config);

  // Unsupported options.
  if (config.GetClosedLoopControlPeriod())
    throw std::invalid_argument("[ERROR] ClosedLoopControlPeriod is not supported");
  if (config.GetTemperatureCutoff())
    throw std::invalid_argument("[ERROR] TemperatureCutoff is not supported");
  if (config.GetFeedbackSynchronizationThreshold())
    throw std::invalid_argument("[ERROR] FeedbackSynchronizationThreshold is not supported");
  if (config.GetVoltageCompensation())
    throw std::invalid_argument("[ERROR] VoltageCompensation is not supported");

  config.ValidateBasicOptions();
  config.ValidateExternalEncoderOptions();
  return ForceConfigApply().IsOK();
}

void TalonFXWrapper::ApplyExternalEncoder(const SmartMotorControllerConfig& config) {
  m_cancoder = nullptr;
  m_candi = nullptr;
  const bool useExternalEncoder = config.GetUseExternalFeedback();
  const auto encoder = config.GetExternalEncoder();
  const auto startingPosition = config.GetStartingPosition();

  if (encoder && useExternalEncoder) {
    // The external encoder sets the position; the starting position is not applied.
    if (startingPosition) m_startingPositionExternalEncoderAlert->Set(true);

    if (auto* pp = std::any_cast<hardware::CANcoder*>(&*encoder); pp && *pp) {
      m_cancoder = *pp;
      WriteSensorRatios(config);
      auto& configurator = m_cancoder->GetConfigurator();
      configs::CANcoderConfiguration cancoderConfig;
      configurator.Refresh(cancoderConfig);
      m_talonConfig.Feedback.FeedbackRemoteSensorID = m_cancoder->GetDeviceID();
      if (auto inv = config.GetExternalEncoderInverted(); inv)
        cancoderConfig.MagnetSensor.SensorDirection =
            *inv ? signals::SensorDirectionValue::Clockwise_Positive
                 : signals::SensorDirectionValue::CounterClockwise_Positive;
      m_talonConfig.Feedback.FeedbackSensorSource =
          signals::FeedbackSensorSourceValue::FusedCANcoder;
      if (auto offset = config.GetExternalEncoderZeroOffset(); offset) {
        cancoderConfig.MagnetSensor.MagnetOffset = *offset;
        m_talonConfig.Feedback.FeedbackRotorOffset = 0_tr;
      }
      if (auto dp = config.GetExternalEncoderDiscontinuityPoint(); dp)
        cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = *dp;
      configurator.Apply(cancoderConfig);
    } else if (auto* pp = std::any_cast<hardware::CANdi*>(&*encoder); pp && *pp) {
      m_candi = *pp;
      WriteSensorRatios(config);
      auto& configurator = m_candi->GetConfigurator();
      configs::CANdiConfiguration candiConfig;
      configurator.Refresh(candiConfig);
      m_talonConfig.Feedback.FeedbackRemoteSensorID = m_candi->GetDeviceID();
      // Use the fused source of the selected PWM input.
      if (UseCANdiPWM2())
        m_talonConfig.Feedback.FeedbackSensorSource =
            signals::FeedbackSensorSourceValue::FusedCANdiPWM2;
      if (UseCANdiPWM1())
        m_talonConfig.Feedback.FeedbackSensorSource =
            signals::FeedbackSensorSourceValue::FusedCANdiPWM1;
      auto configurePWM = [&](auto& pwm) {
        if (auto inv = config.GetExternalEncoderInverted(); inv) pwm.SensorDirection = *inv;
        if (auto offset = config.GetExternalEncoderZeroOffset(); offset) {
          pwm.AbsoluteSensorOffset = *offset;
          m_talonConfig.Feedback.FeedbackRotorOffset = 0_tr;
        }
        if (auto dp = config.GetExternalEncoderDiscontinuityPoint(); dp)
          pwm.AbsoluteSensorDiscontinuityPoint = *dp;
      };
      if (UseCANdiPWM1()) {
        configurePWM(candiConfig.PWM1);
      } else if (UseCANdiPWM2()) {
        configurePWM(candiConfig.PWM2);
      } else {
        throw ConfigException(
            "CANdi is the external feedback encoder but no PWM input is selected",
            "The CANdi cannot be used as the feedback sensor!",
            "WithVendorConfig() with the feedback sensor source set to SyncCANdiPWM1 or "
            "SyncCANdiPWM2");
      }
      configurator.Apply(candiConfig);
    } else {
      throw ConfigException("TalonFX external encoders must be a CANcoder* or CANdi*",
                            "Unsupported external encoder",
                            "WithExternalEncoder(ctre::phoenix6::hardware::CANcoder*)");
    }
    return;
  }

  if (config.GetExternalEncoderInverted())
    throw ConfigException("External Encoder cannot be inverted if not present!",
                          "External encoder is not inverted!",
                          "WithExternalEncoderInverted(false)");
  if (config.GetExternalEncoderGearing())
    throw ConfigException("External Encoder cannot be set if not present!",
                          "External encoder gearing is not 1.0!",
                          "WithExternalEncoderGearing(1.0)");
  if (config.GetExternalEncoderZeroOffset()) m_zeroOffsetNoExternalEncoderAlert->Set(true);

  m_talonConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RotorSensor;
  WriteSensorRatios(config);

  if (startingPosition) {
    // The position is in mechanism rotations, so the sensor ratios must be applied first.
    ForceConfigApply();
    if (wpi::RobotBase::IsSimulation()) {
      const double mechanismToRotor = config.GetMotorGearing()
                                          .value_or(gearing::MechanismGearing::kOne)
                                          .GetMechanismToRotorRatio();
      m_talon->GetSimState().SetRawRotorPosition(*startingPosition * mechanismToRotor);
    }
    for (int i = 0; i <= 100; i++) {
      const auto applied = m_talon->SetPosition(*startingPosition);
      Sleep10ms();
      if (applied.IsOK()) break;
    }
  }
  if (config.GetExternalEncoderDiscontinuityPoint())
    m_discontinuityPointNoExternalEncoderAlert->Set(true);
}

void TalonFXWrapper::ApplyFollowers(const SmartMotorControllerConfig& config) {
  const auto& followers = config.GetFollowers();
  if (followers.empty()) return;
  const auto zeroPower = config.GetZeroPower();
  for (const auto& [hw, opposed] : followers) {
    const auto alignment =
        opposed ? signals::MotorAlignmentValue::Opposed : signals::MotorAlignmentValue::Aligned;
    auto follow = [&](auto* follower) {
      if (zeroPower) {
        auto status = follower->ConfigNeutralMode(*zeroPower == MotorMode::BRAKE
                                                      ? signals::NeutralModeValue::Brake
                                                      : signals::NeutralModeValue::Coast);
        if (!status.IsOK()) return status;
      }
      return follower->SetControl(controls::Follower{m_talon->GetDeviceID(), alignment});
    };
    std::function<ctre::phoenix::StatusCode()> configureFollower;
    if (auto* fx = std::any_cast<hardware::TalonFX*>(&hw)) {
      configureFollower = [&, fx] { return follow(*fx); };
    } else if (auto* fxs = std::any_cast<hardware::TalonFXS*>(&hw)) {
      configureFollower = [&, fxs] { return follow(*fxs); };
    } else {
      throw std::invalid_argument(
          "[ERROR] Unknown follower type: TalonFXWrapper followers must be "
          "ctre::phoenix6::hardware::TalonFX* or ctre::phoenix6::hardware::TalonFXS*");
    }
    for (int i = 0; i < 100 && !configureFollower().IsOK(); i++) Sleep10ms();
  }
  // The followers keep following; applying the config again must not reconfigure them.
  m_config->ClearFollowers();
}

void TalonFXWrapper::ApplyVendorControlRequest(const SmartMotorControllerConfig& config) {
  auto req = config.GetVendorControlRequest();
  if (!req) return;
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
  else
    throw ConfigException("TalonFX(" + std::to_string(m_talon->GetDeviceID()) +
                              ") does not support the given control request!",
                          "Cannot use given control request", "WithVendorControlRequest()");
}

// ---- Simulation -------------------------------------------------------------

void TalonFXWrapper::SetupSimulation() {
  if (!wpi::RobotBase::IsSimulation()) return;
  const double mechanismToRotor = m_config->GetMotorGearing()
                                      .value_or(gearing::MechanismGearing::kOne)
                                      .GetMechanismToRotorRatio();
  if (!m_motorSim) {
    const auto simMotor = m_config->GetSimMotor().value_or(m_dcMotor);
    auto plant = wpi::math::Models::SingleJointedArmFromPhysicalConstants(
        simMotor, m_config->GetMOI(), mechanismToRotor);
    m_motorSim.emplace(plant, simMotor);
    SetSimSupplier(std::make_shared<simulation::DCMotorSimSupplier>(*m_motorSim, *this));
  }
  if (auto startPos = m_config->GetStartingPosition()) {
    m_simSupplier->SetMechanismPosition(*startPos);
    m_talon->GetSimState().SetRawRotorPosition(*startPos * mechanismToRotor);
  }
}

void TalonFXWrapper::SimIterate() {
  if (!wpi::RobotBase::IsSimulation() || !m_simSupplier) return;

  auto& sim = m_talon->GetSimState();
  sim.SetSupplyVoltage(m_simSupplier->GetMechanismSupplyVoltage());

  m_simSupplier->SetMechanismStatorVoltage(sim.GetMotorVoltage());
  // Steps the physics only if the mechanism has not already stepped them this loop.
  m_simSupplier->UpdateSim();
  m_simSupplier->StarveUpdateSim();
  simulation::BatterySim::CalculateVoltage(BatterySimKey(), m_simSupplier->GetSupplyCurrent());

  sim.SetRawRotorPosition(wpi::units::turn_t{m_simSupplier->GetRotorPosition()});
  sim.SetRotorVelocity(wpi::units::turns_per_second_t{m_simSupplier->GetRotorVelocity()});
  sim.SetRotorAcceleration(
      wpi::units::turns_per_second_squared_t{m_simSupplier->GetRotorAcceleration()});

  if (!m_cancoder && !m_candi) return;
  // The external encoder turns with the mechanism through its gearing; its reading includes the
  // zero offset.
  const double mechanismToSensor = m_config->GetExternalEncoderGearing()
                                       .value_or(gearing::MechanismGearing::kOne)
                                       .GetMechanismToRotorRatio();
  const wpi::units::turn_t zeroOffset = m_config->GetExternalEncoderZeroOffset().value_or(0_tr);
  const wpi::units::turn_t sensorPosition =
      wpi::units::turn_t{m_simSupplier->GetMechanismPosition()} * mechanismToSensor - zeroOffset;
  const wpi::units::turns_per_second_t sensorVelocity =
      wpi::units::turns_per_second_t{m_simSupplier->GetMechanismVelocity()} * mechanismToSensor;
  if (m_cancoder) {
    auto& cancoderSim = m_cancoder->GetSimState();
    cancoderSim.SetSupplyVoltage(m_simSupplier->GetMechanismSupplyVoltage());
    cancoderSim.SetVelocity(sensorVelocity);
    cancoderSim.SetRawPosition(sensorPosition);
    cancoderSim.SetMagnetHealth(signals::MagnetHealthValue::Magnet_Green);
  }
  if (m_candi) {
    auto& candiSim = m_candi->GetSimState();
    candiSim.SetSupplyVoltage(m_simSupplier->GetMechanismSupplyVoltage());
    if (UseCANdiPWM1()) {
      candiSim.SetPwm1Connected(true);
      candiSim.SetPwm1Position(sensorPosition);
      candiSim.SetPwm1Velocity(sensorVelocity);
    } else if (UseCANdiPWM2()) {
      candiSim.SetPwm2Connected(true);
      candiSim.SetPwm2Position(sensorPosition);
      candiSim.SetPwm2Velocity(sensorVelocity);
    }
  }
}

// ---- Encoder sync -----------------------------------------------------------

void TalonFXWrapper::SeedRelativeEncoder() {
  // The TalonFX uses an absolute sensor internally; no seeding needed.
}

void TalonFXWrapper::SynchronizeRelativeEncoder() {
  // The TalonFX fuses its feedback sources on the device.
}

// ---- Open-loop outputs ------------------------------------------------------

void TalonFXWrapper::SetDutyCycle(double dutyCycle) {
  m_talon->SetControl(m_dutyCycleReq.WithOutput(dutyCycle));
  if (dutyCycle == 0.0) {
    for (auto* f : m_looseFollowers) f->SetDutyCycle(dutyCycle);
  }
}

double TalonFXWrapper::GetDutyCycle() {
  return m_talon->GetDutyCycle(false).Refresh(m_reportStatusSignalErrors).GetValue();
}

void TalonFXWrapper::SetVoltage(wpi::units::volt_t voltage) {
  m_talon->SetControl(m_voltageReq.WithOutput(voltage));
}

wpi::units::volt_t TalonFXWrapper::GetVoltage() {
  return m_talon->GetMotorVoltage(false).Refresh(m_reportStatusSignalErrors).GetValue();
}

// ---- Closed-loop setpoints --------------------------------------------------

void TalonFXWrapper::SetPosition(wpi::units::turn_t angle) {
  m_setpointVelocity.reset();
  m_setpointFeedforwardForce.reset();
  m_setpointPosition = angle;
  // With an LQR the RoboRIO closed loop drives the motor.
  if (m_lqr) return;
  EnsureRequest([&] {
    return std::visit([&](auto& req) { return m_talon->SetControl(req.WithPosition(angle)); },
                      m_positionReq);
  });
  ForwardPositionToFollowers(angle);
}

void TalonFXWrapper::SetPosition(wpi::units::meter_t distance) {
  SetPosition(m_config->ConvertToMechanism(distance));
}

void TalonFXWrapper::SetVelocity(wpi::units::turns_per_second_t velocity) {
  m_setpointPosition.reset();
  m_setpointVelocity = velocity;
  m_setpointFeedforwardForce.reset();
  if (m_lqr) return;
  // The request objects are reused, so clear any feedforward left by SetVelocity(velocity, force).
  EnsureRequest([&] {
    return std::visit(
        [&](auto& req) {
          using FeedForward = std::remove_cvref_t<decltype(req.FeedForward)>;
          return m_talon->SetControl(req.WithVelocity(velocity).WithFeedForward(FeedForward{0}));
        },
        m_velocityReq);
  });
  ForwardVelocityToFollowers(velocity);
}

void TalonFXWrapper::SetVelocity(wpi::units::turns_per_second_t velocity,
                                 wpi::units::newton_t feedforwardForce) {
  if (m_lqr) {
    // The RoboRIO loop adds the force's voltage.
    SetVelocity(velocity);
    m_setpointFeedforwardForce = feedforwardForce;
    return;
  }
  const wpi::units::volt_t feedforwardVoltage =
      m_config->ConvertToVoltage(m_dcMotor, feedforwardForce);
  const wpi::units::ampere_t feedforwardCurrent =
      m_config->ConvertToCurrent(m_dcMotor, feedforwardForce);
  m_setpointPosition.reset();
  m_setpointVelocity = velocity;
  m_setpointFeedforwardForce = feedforwardForce;
  EnsureRequest([&] {
    return std::visit(
        [&](auto& req) {
          using FeedForward = std::remove_cvref_t<decltype(req.FeedForward)>;
          FeedForward feedforward{0};
          if constexpr (std::is_same_v<FeedForward, wpi::units::volt_t>) {
            feedforward = feedforwardVoltage;
          } else if constexpr (std::is_same_v<FeedForward, wpi::units::ampere_t>) {
            feedforward = feedforwardCurrent;
          } else {
            feedforward = FeedForward{(feedforwardVoltage / m_dcMotor.nominalVoltage).value()};
          }
          return m_talon->SetControl(req.WithVelocity(velocity).WithFeedForward(feedforward));
        },
        m_velocityReq);
  });
  for (auto* f : m_looseFollowers) f->SetVelocity(velocity, feedforwardForce);
}

void TalonFXWrapper::SetVelocity(wpi::units::meters_per_second_t velocity) {
  SetVelocity(m_config->ConvertToMechanism(velocity));
}

// ---- Encoder writes ---------------------------------------------------------

void TalonFXWrapper::SetEncoderPosition(wpi::units::turn_t angle) {
  if (m_simSupplier) {
    // Move the simulated mechanism first, so the TalonFX keeps the new position.
    const double mechanismToRotor = m_config->GetMotorGearing()
                                        .value_or(gearing::MechanismGearing::kOne)
                                        .GetMechanismToRotorRatio();
    m_talon->GetSimState().SetRawRotorPosition(angle * mechanismToRotor);
    m_simSupplier->SetMechanismPosition(angle);
  }
  m_talon->SetPosition(angle);
  if (m_cancoder) {
    // The CANcoder turns with the mechanism through the external encoder gearing.
    m_cancoder->SetPosition(angle * m_config->GetExternalEncoderGearing()
                                        .value_or(gearing::MechanismGearing::kOne)
                                        .GetMechanismToRotorRatio());
  }
}

void TalonFXWrapper::SetEncoderPosition(wpi::units::meter_t distance) {
  SetEncoderPosition(m_config->ConvertToMechanism(distance));
}

void TalonFXWrapper::SetEncoderVelocity(wpi::units::turns_per_second_t) {}

void TalonFXWrapper::SetEncoderVelocity(wpi::units::meters_per_second_t velocity) {
  SetEncoderVelocity(m_config->ConvertToMechanism(velocity));
}

// ---- Encoder reads ----------------------------------------------------------

wpi::units::turn_t TalonFXWrapper::GetMechanismPosition() {
  if (m_config->GetUseExternalFeedback()) {
    if (auto external = GetExternalEncoderMechanismPosition()) return *external;
  }
  return GetRelativeMechanismPosition();
}

wpi::units::turns_per_second_t TalonFXWrapper::GetMechanismVelocity() {
  if (m_config->GetUseExternalFeedback()) {
    if (auto external = GetExternalEncoderMechanismVelocity()) return *external;
  }
  return GetRelativeMechanismVelocity();
}

wpi::units::turns_per_second_squared_t TalonFXWrapper::GetMechanismAcceleration() {
  return m_talon->GetAcceleration(false).Refresh(m_reportStatusSignalErrors).GetValue();
}

wpi::units::turn_t TalonFXWrapper::GetRotorPosition() {
  return m_talon->GetRotorPosition(false).Refresh(m_reportStatusSignalErrors).GetValue();
}

wpi::units::turns_per_second_t TalonFXWrapper::GetRotorVelocity() {
  return m_talon->GetRotorVelocity(false).Refresh(m_reportStatusSignalErrors).GetValue();
}

wpi::units::turn_t TalonFXWrapper::GetRelativeMechanismPosition() {
  return GetRotorPosition() * m_config->GetMotorGearing()
                                  .value_or(gearing::MechanismGearing::kOne)
                                  .GetRotorToMechanismRatio();
}

wpi::units::turns_per_second_t TalonFXWrapper::GetRelativeMechanismVelocity() {
  return GetRotorVelocity() * m_config->GetMotorGearing()
                                  .value_or(gearing::MechanismGearing::kOne)
                                  .GetRotorToMechanismRatio();
}

wpi::units::meter_t TalonFXWrapper::GetMeasurementPosition() {
  return m_config->ConvertFromMechanism(GetMechanismPosition());
}

wpi::units::meters_per_second_t TalonFXWrapper::GetMeasurementVelocity() {
  return m_config->ConvertFromMechanism(GetMechanismVelocity());
}

wpi::units::meters_per_second_squared_t TalonFXWrapper::GetMeasurementAcceleration() {
  return m_config->ConvertFromMechanism(GetMechanismAcceleration());
}

std::optional<wpi::units::turn_t> TalonFXWrapper::GetExternalEncoderSensorPosition() {
  if (m_cancoder)
    return m_cancoder->GetPosition(false).Refresh(m_reportStatusSignalErrors).GetValue();
  if (m_candi) {
    if (UseCANdiPWM1())
      return m_candi->GetPWM1Position(false).Refresh(m_reportStatusSignalErrors).GetValue();
    if (UseCANdiPWM2())
      return m_candi->GetPWM2Position(false).Refresh(m_reportStatusSignalErrors).GetValue();
  }
  return std::nullopt;
}

std::optional<wpi::units::turns_per_second_t> TalonFXWrapper::GetExternalEncoderSensorVelocity() {
  if (m_cancoder)
    return m_cancoder->GetVelocity(false).Refresh(m_reportStatusSignalErrors).GetValue();
  if (m_candi) {
    if (UseCANdiPWM1())
      return m_candi->GetPWM1Velocity(false).Refresh(m_reportStatusSignalErrors).GetValue();
    if (UseCANdiPWM2())
      return m_candi->GetPWM2Velocity(false).Refresh(m_reportStatusSignalErrors).GetValue();
  }
  return std::nullopt;
}

std::optional<wpi::units::turn_t> TalonFXWrapper::GetExternalEncoderMechanismPosition() {
  auto external = GetExternalEncoderSensorPosition();
  if (!external) return std::nullopt;
  return *external * m_config->GetExternalEncoderGearing()
                         .value_or(gearing::MechanismGearing::kOne)
                         .GetRotorToMechanismRatio();
}

std::optional<wpi::units::turns_per_second_t> TalonFXWrapper::GetExternalEncoderMechanismVelocity() {
  auto external = GetExternalEncoderSensorVelocity();
  if (!external) return std::nullopt;
  return *external * m_config->GetExternalEncoderGearing()
                         .value_or(gearing::MechanismGearing::kOne)
                         .GetRotorToMechanismRatio();
}

// ---- Motor status -----------------------------------------------------------

std::optional<wpi::units::ampere_t> TalonFXWrapper::GetSupplyCurrent() {
  return m_talon->GetSupplyCurrent(false).Refresh(m_reportStatusSignalErrors).GetValue();
}

wpi::units::ampere_t TalonFXWrapper::GetStatorCurrent() {
  return m_talon->GetStatorCurrent(false).Refresh(m_reportStatusSignalErrors).GetValue();
}

wpi::units::celsius_t TalonFXWrapper::GetTemperature() {
  return wpi::units::celsius_t{
      m_talon->GetDeviceTemp(false).Refresh(m_reportStatusSignalErrors).GetValue().value()};
}

wpi::math::DCMotor TalonFXWrapper::GetDCMotor() { return m_dcMotor; }

// ---- Live-tuning setters ----------------------------------------------------

void TalonFXWrapper::SetZeroPower(MotorMode mode) {
  m_talonConfig.MotorOutput.NeutralMode = mode == MotorMode::BRAKE
                                              ? signals::NeutralModeValue::Brake
                                              : signals::NeutralModeValue::Coast;
  ApplyGroup(m_talonConfig.MotorOutput);
}

void TalonFXWrapper::SetMotorInverted(bool inv) {
  m_config->WithMotorInverted(inv);
  m_talonConfig.MotorOutput.Inverted = inv ? signals::InvertedValue::Clockwise_Positive
                                           : signals::InvertedValue::CounterClockwise_Positive;
  ApplyGroup(m_talonConfig.MotorOutput);
}

void TalonFXWrapper::SetEncoderInverted(bool) {}

void TalonFXWrapper::SetKp(double kP) {
  const auto gains = m_config->GetSlotGains(m_slot);
  SetFeedback(kP, gains.kI, gains.kD);
}

void TalonFXWrapper::SetKi(double kI) {
  const auto gains = m_config->GetSlotGains(m_slot);
  SetFeedback(gains.kP, kI, gains.kD);
}

void TalonFXWrapper::SetKd(double kD) {
  const auto gains = m_config->GetSlotGains(m_slot);
  SetFeedback(gains.kP, gains.kI, kD);
}

void TalonFXWrapper::SetFeedback(double kP, double kI, double kD) {
  m_config->WithFeedback(kP, kI, kD, m_slot);
  if (m_pid) {
    m_pid->SetP(kP);
    m_pid->SetI(kI);
    m_pid->SetD(kD);
  }
  WriteSlotPID(m_slot, kP, kI, kD);
  WithSlotConfigs(m_talonConfig, m_slot, [this](auto& s) { ApplyGroup(s); });
  for (auto* f : m_looseFollowers) f->SetFeedback(kP, kI, kD);
}

void TalonFXWrapper::SetKs(double kS) {
  UpdateConfigFeedforward(*m_config, m_slot, [&](NativeFeedforward& v) { v.kS = kS; });
  WithSlotConfigs(m_talonConfig, m_slot, [&](auto& s) {
    s.kS = kS;
    ApplyGroup(s);
  });
  for (auto* f : m_looseFollowers) f->SetKs(kS);
}

void TalonFXWrapper::SetKv(double kV) {
  UpdateConfigFeedforward(*m_config, m_slot, [&](NativeFeedforward& v) { v.kV = kV; });
  const double talonKv = KvPerRotation(*m_config, m_slot, kV) / VelocityGainUnitsPerRotation();
  WithSlotConfigs(m_talonConfig, m_slot, [&](auto& s) {
    s.kV = talonKv;
    ApplyGroup(s);
  });
  for (auto* f : m_looseFollowers) f->SetKv(kV);
}

void TalonFXWrapper::SetKa(double kA) {
  UpdateConfigFeedforward(*m_config, m_slot, [&](NativeFeedforward& v) { v.kA = kA; });
  const double talonKa = KaPerRotation(*m_config, m_slot, kA) / AccelerationGainUnitsPerRotation();
  WithSlotConfigs(m_talonConfig, m_slot, [&](auto& s) {
    s.kA = talonKa;
    ApplyGroup(s);
  });
  for (auto* f : m_looseFollowers) f->SetKa(kA);
}

void TalonFXWrapper::SetKg(double kG) {
  UpdateConfigFeedforward(*m_config, m_slot, [&](NativeFeedforward& v) { v.kG = kG; });
  const bool arm = m_config->GetArmFeedforward(m_slot).has_value();
  const bool elevator = m_config->GetElevatorFeedforward(m_slot).has_value();
  WithSlotConfigs(m_talonConfig, m_slot, [&](auto& s) {
    s.kG = kG;
    if (arm)
      s.GravityType = signals::GravityTypeValue::Arm_Cosine;
    else if (elevator)
      s.GravityType = signals::GravityTypeValue::Elevator_Static;
    ApplyGroup(s);
  });
  for (auto* f : m_looseFollowers) f->SetKg(kG);
}

void TalonFXWrapper::SetFeedforward(double kS, double kV, double kA, double kG) {
  UpdateConfigFeedforward(*m_config, m_slot, [&](NativeFeedforward& v) {
    v.kS = kS;
    v.kV = kV;
    v.kA = kA;
    v.kG = kG;
  });
  const bool arm = m_config->GetArmFeedforward(m_slot).has_value();
  const bool elevator = m_config->GetElevatorFeedforward(m_slot).has_value();
  const double talonKv = KvPerRotation(*m_config, m_slot, kV) / VelocityGainUnitsPerRotation();
  const double talonKa = KaPerRotation(*m_config, m_slot, kA) / AccelerationGainUnitsPerRotation();
  WithSlotConfigs(m_talonConfig, m_slot, [&](auto& s) {
    s.kS = kS;
    s.kV = talonKv;
    s.kA = talonKa;
    s.kG = kG;
    if (arm)
      s.GravityType = signals::GravityTypeValue::Arm_Cosine;
    else if (elevator)
      s.GravityType = signals::GravityTypeValue::Elevator_Static;
    ApplyGroup(s);
  });
  for (auto* f : m_looseFollowers) f->SetFeedforward(kS, kV, kA, kG);
}

void TalonFXWrapper::SetStatorCurrentLimit(wpi::units::ampere_t limit) {
  m_config->WithStatorCurrentLimit(limit);
  m_talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  m_talonConfig.CurrentLimits.StatorCurrentLimit = limit;
  ApplyGroup(m_talonConfig.CurrentLimits);
  for (auto* f : m_looseFollowers) f->SetStatorCurrentLimit(limit);
}

void TalonFXWrapper::SetSupplyCurrentLimit(wpi::units::ampere_t limit) {
  m_config->WithSupplyCurrentLimit(limit);
  m_talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  m_talonConfig.CurrentLimits.SupplyCurrentLimit = limit;
  ApplyGroup(m_talonConfig.CurrentLimits);
  for (auto* f : m_looseFollowers) f->SetSupplyCurrentLimit(limit);
}

void TalonFXWrapper::SetClosedLoopRampRate(wpi::units::second_t r) {
  m_config->WithClosedLoopRampRate(r);
  m_talonConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = r;
  m_talonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = r;
  m_talonConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = r;
  ApplyGroup(m_talonConfig.ClosedLoopRamps);
  for (auto* f : m_looseFollowers) f->SetClosedLoopRampRate(r);
}

void TalonFXWrapper::SetOpenLoopRampRate(wpi::units::second_t r) {
  m_config->WithOpenLoopRampRate(r);
  m_talonConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = r;
  m_talonConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = r;
  m_talonConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = r;
  ApplyGroup(m_talonConfig.OpenLoopRamps);
  for (auto* f : m_looseFollowers) f->SetOpenLoopRampRate(r);
}

void TalonFXWrapper::SetMechanismUpperLimit(wpi::units::turn_t upper) {
  if (auto lower = m_config->GetMechanismLowerLimit()) m_config->WithMechanismLimits(*lower, upper);
  m_talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
  m_talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = upper;
  ApplyGroup(m_talonConfig.SoftwareLimitSwitch);
  for (auto* f : m_looseFollowers) f->SetMechanismUpperLimit(upper);
}

void TalonFXWrapper::SetMechanismLowerLimit(wpi::units::turn_t lower) {
  if (auto upper = m_config->GetMechanismUpperLimit()) m_config->WithMechanismLimits(lower, *upper);
  m_talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
  m_talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = lower;
  ApplyGroup(m_talonConfig.SoftwareLimitSwitch);
  for (auto* f : m_looseFollowers) f->SetMechanismLowerLimit(lower);
}

void TalonFXWrapper::SetMechanismLimits(wpi::units::turn_t lower, wpi::units::turn_t upper) {
  m_config->WithMechanismLimits(lower, upper);
  // Only the thresholds change; whether the limits are enforced is unchanged.
  m_talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = upper;
  m_talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = lower;
  ApplyGroup(m_talonConfig.SoftwareLimitSwitch);
  for (auto* f : m_looseFollowers) f->SetMechanismLimits(lower, upper);
}

void TalonFXWrapper::SetMechanismLimitsEnabled(bool en) {
  m_talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = en;
  m_talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = en;
  ApplyGroup(m_talonConfig.SoftwareLimitSwitch);
  for (auto* f : m_looseFollowers) f->SetMechanismLimitsEnabled(en);
}

void TalonFXWrapper::SetMeasurementUpperLimit(wpi::units::meter_t upper) {
  auto lowerAngle = m_config->GetMechanismLowerLimit();
  if (!m_config->GetMechanismCircumference() || !lowerAngle) return;
  m_config->WithMeasurementLimits(m_config->ConvertFromMechanism(*lowerAngle), upper);
  m_talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
  m_talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = m_config->ConvertToMechanism(upper);
  ApplyGroup(m_talonConfig.SoftwareLimitSwitch);
  for (auto* f : m_looseFollowers) f->SetMeasurementUpperLimit(upper);
}

void TalonFXWrapper::SetMeasurementLowerLimit(wpi::units::meter_t lower) {
  auto upperAngle = m_config->GetMechanismUpperLimit();
  if (!m_config->GetMechanismCircumference() || !upperAngle) return;
  m_config->WithMeasurementLimits(lower, m_config->ConvertFromMechanism(*upperAngle));
  m_talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
  m_talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = m_config->ConvertToMechanism(lower);
  ApplyGroup(m_talonConfig.SoftwareLimitSwitch);
  for (auto* f : m_looseFollowers) f->SetMeasurementLowerLimit(lower);
}

void TalonFXWrapper::SetMotionProfileMaxVelocity(wpi::units::turns_per_second_t vel) {
  if (!m_config->GetVelocityTrapezoidalProfileInUse()) {
    // Keep the tuned constraints in the config, so tuning the next one keeps this one.
    if (auto accLin = m_config->GetTrapMaxAccelLinear();
        accLin && !m_config->GetTrapMaxVelocityTurns()) {
      m_config->WithLinearTrapezoidProfile(m_config->ConvertFromMechanism(vel), *accLin);
    } else if (auto acc = m_config->GetTrapMaxAccelTurns()) {
      m_config->WithTrapezoidProfile(vel, *acc);
    }
  }
  m_talonConfig.MotionMagic.MotionMagicCruiseVelocity = vel;
  ApplyGroup(m_talonConfig.MotionMagic);
  for (auto* f : m_looseFollowers) f->SetMotionProfileMaxVelocity(vel);
}

void TalonFXWrapper::SetMotionProfileMaxVelocity(wpi::units::meters_per_second_t vel) {
  // Convert first so a missing circumference throws before anything changes.
  const auto mechanismVelocity = m_config->ConvertToMechanism(vel);
  if (!m_config->GetVelocityTrapezoidalProfileInUse()) {
    if (auto accLin = m_config->GetTrapMaxAccelLinear();
        accLin && !m_config->GetTrapMaxVelocityTurns()) {
      m_config->WithLinearTrapezoidProfile(vel, *accLin);
    } else if (auto acc = m_config->GetTrapMaxAccelTurns()) {
      m_config->WithTrapezoidProfile(mechanismVelocity, *acc);
    }
  }
  m_talonConfig.MotionMagic.MotionMagicCruiseVelocity = mechanismVelocity;
  ApplyGroup(m_talonConfig.MotionMagic);
  for (auto* f : m_looseFollowers) f->SetMotionProfileMaxVelocity(vel);
}

void TalonFXWrapper::SetMotionProfileMaxAcceleration(wpi::units::turns_per_second_squared_t acc) {
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
  // Motion Magic's acceleration limits both position and velocity profiles.
  m_talonConfig.MotionMagic.MotionMagicAcceleration = acc;
  ApplyGroup(m_talonConfig.MotionMagic);
  for (auto* f : m_looseFollowers) f->SetMotionProfileMaxAcceleration(acc);
}

void TalonFXWrapper::SetMotionProfileMaxAcceleration(wpi::units::meters_per_second_squared_t acc) {
  const auto mechanismAcceleration = m_config->ConvertToMechanism(acc);
  const bool linear = m_config->GetTrapMaxVelocityLinear() && !m_config->GetTrapMaxVelocityTurns();
  if (m_config->GetVelocityTrapezoidalProfileInUse()) {
    if (linear) {
      m_config->WithVelocityTrapezoidProfile(
          acc, meters_per_second_cubed_t{m_config->GetTrapMaxAccelLinear()->value()});
    } else if (auto jerk = m_config->GetTrapMaxAccelTurns()) {
      m_config->WithVelocityTrapezoidProfile(
          mechanismAcceleration, wpi::units::angular_jerk::turns_per_second_cubed_t{jerk->value()});
    }
  } else if (linear) {
    m_config->WithLinearTrapezoidProfile(*m_config->GetTrapMaxVelocityLinear(), acc);
  } else if (auto vel = m_config->GetTrapMaxVelocityTurns()) {
    m_config->WithTrapezoidProfile(*vel, mechanismAcceleration);
  }
  m_talonConfig.MotionMagic.MotionMagicAcceleration = mechanismAcceleration;
  ApplyGroup(m_talonConfig.MotionMagic);
  for (auto* f : m_looseFollowers) f->SetMotionProfileMaxAcceleration(acc);
}

void TalonFXWrapper::SetMotionProfileMaxJerk(
    wpi::units::angular_jerk::turns_per_second_cubed_t jerk) {
  if (m_config->GetVelocityTrapezoidalProfileInUse()) {
    if (auto accLin = m_config->GetTrapMaxVelocityLinear();
        accLin && !m_config->GetTrapMaxVelocityTurns()) {
      m_config->WithVelocityTrapezoidProfile(
          wpi::units::meters_per_second_squared_t{accLin->value()},
          m_config->ConvertFromMechanism(jerk));
    } else if (auto acc = m_config->GetTrapMaxVelocityTurns()) {
      m_config->WithVelocityTrapezoidProfile(wpi::units::turns_per_second_squared_t{acc->value()},
                                             jerk);
    }
  }
  m_talonConfig.MotionMagic.MotionMagicJerk = jerk;
  ApplyGroup(m_talonConfig.MotionMagic);
  for (auto* f : m_looseFollowers) f->SetMotionProfileMaxJerk(jerk);
}

void TalonFXWrapper::SetExponentialProfile(std::optional<double> kV, std::optional<double> kA,
                                           std::optional<wpi::units::volt_t> maxInput) {
  const bool linear = m_config->HasLinearExponentialProfile();
  if (!m_config->HasExponentialProfile() && !linear) return;
  // kV and kA are per mechanism rotation, like GetExponentialProfileKV/KA; missing values keep
  // the configured ones.
  const double newKV = kV.value_or(m_config->GetExponentialProfileKV().value_or(0.0));
  const double newKA = kA.value_or(m_config->GetExponentialProfileKA().value_or(0.0));
  const wpi::units::volt_t newMaxInput =
      maxInput.value_or(m_config->GetExponentialProfileMaxInput().value_or(12_V));

  // Keep the tuned constraints in the config, so tuning the next one keeps this one.
  if (linear) {
    const double rotationsPerMeter = m_config->ConvertToMechanism(wpi::units::meter_t{1.0}).value();
    m_config->WithLinearExponentialProfile(newKV * rotationsPerMeter, newKA * rotationsPerMeter,
                                           newMaxInput);
  } else {
    m_config->WithExponentialProfile(newKV, newKA, newMaxInput);
  }

  m_talonConfig.MotionMagic.MotionMagicExpo_kV = ctre::unit::volts_per_turn_per_second_t{newKV};
  m_talonConfig.MotionMagic.MotionMagicExpo_kA =
      ctre::unit::volts_per_turn_per_second_squared_t{newKA};
  ApplyGroup(m_talonConfig.MotionMagic);
  for (auto* f : m_looseFollowers) f->SetExponentialProfile(kV, kA, maxInput);
}

void TalonFXWrapper::SetClosedLoopSlot(ClosedLoopControllerSlot slot) {
  if (slot == ClosedLoopControllerSlot::SLOT_3)
    throw std::invalid_argument("Invalid slot: TalonFX only supports SLOT_0 through SLOT_2");
  m_slot = slot;
  const int idx = static_cast<int>(slot);
  std::visit([idx](auto& req) { req.WithSlot(idx); }, m_positionReq);
  std::visit([idx](auto& req) { req.WithSlot(idx); }, m_velocityReq);
  for (auto* f : m_looseFollowers) f->SetClosedLoopSlot(slot);
}

void TalonFXWrapper::SetMechanismGearing(const gearing::MechanismGearing& gearing) {
  SmartMotorController::SetMechanismGearing(gearing);
  WriteSensorRatios(*m_config);
  ApplyGroup(m_talonConfig.Feedback);
}

void TalonFXWrapper::SetMechanismCircumference(wpi::units::meter_t circumference) {
  SmartMotorController::SetMechanismCircumference(circumference);
  // Linear gains and profile constraints are converted with the circumference.
  WriteSlotGains(*m_config);
  WriteMotionMagic(*m_config);
  ForceConfigApply();
  for (auto* f : m_looseFollowers) f->SetMechanismCircumference(circumference);
}

SmartMotorControllerConfig& TalonFXWrapper::GetConfig() { return *m_config; }
void* TalonFXWrapper::GetMotorController() { return m_talon; }
void* TalonFXWrapper::GetMotorControllerConfig() { return &m_talonConfig; }

telemetry::UnsupportedTelemetryFields TalonFXWrapper::GetUnsupportedTelemetryFields() {
  return {};  // TalonFX supports all telemetry fields
}

}  // namespace yams::motorcontrollers::remote
