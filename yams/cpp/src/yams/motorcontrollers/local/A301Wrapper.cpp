// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/local/A301Wrapper.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <wpi/framework/RobotBase.hpp>
#include <wpi/math/system/Models.hpp>
#include <wpi/math/util/MathUtil.hpp>

#include "yams/exceptions.hpp"
#include "yams/math/LQRController.hpp"
#include "yams/motorcontrollers/simulation/BatterySim.hpp"
#include "yams/motorcontrollers/simulation/DCMotorSimSupplier.hpp"

namespace yams::motorcontrollers::local {

namespace {

using Slot = SmartMotorControllerConfig::ClosedLoopControllerSlot;
using TurnKv = wpi::math::SimpleMotorFeedforward<wpi::units::turns>::kv_unit;
using TurnKa = wpi::math::SimpleMotorFeedforward<wpi::units::turns>::ka_unit;

/** Gains of the configured feedforward in its own units (ArmFeedforward kV/kA per radian). */
struct NativeFeedforward {
  double kS{0.0}, kV{0.0}, kA{0.0}, kG{0.0};
};

/**
 * Apply @p update to the slot's configured feedforward in its native units and store it back in
 * the config, where the SystemCore closed loop reads it.
 */
template <typename F>
void UpdateConfigFeedforward(SmartMotorControllerConfig& config, Slot slot, F&& update) {
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
  } else if (auto ff = config.GetSimpleFeedforward(slot)) {
    NativeFeedforward v{ff->GetKs().value(), ff->GetKv().value(), ff->GetKa().value(), 0.0};
    update(v);
    ff->SetKs(wpi::units::volt_t{v.kS});
    ff->SetKv(wpi::units::unit_t<TurnKv>{v.kV});
    ff->SetKa(wpi::units::unit_t<TurnKa>{v.kA});
    config.WithFeedforward(*ff, slot);
  }
}

/**
 * Check how many A301s drive the mechanism.
 *
 * @param motorCount How many A301s drive the mechanism.
 * @return The motor count.
 * @throws std::invalid_argument if it is less than 1.
 */
int CheckMotorCount(int motorCount) {
  if (motorCount < 1) {
    throw std::invalid_argument("A301Wrapper motor count must be at least 1, but was " +
                                std::to_string(motorCount));
  }
  return motorCount;
}

}  // namespace

// ---- Construction -----------------------------------------------------------

A301Wrapper::A301Wrapper(first::a301::A301* a301, SmartMotorControllerConfig* cfg)
    : A301Wrapper(a301, 1, cfg) {}

A301Wrapper::A301Wrapper(first::a301::A301* a301, int motorCount, SmartMotorControllerConfig* cfg)
    : SmartMotorController(), m_a301(a301), m_motor(YMotors::A301(CheckMotorCount(motorCount))) {
  m_config = cfg;
  m_config->WithSimMotor(m_motor);
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

A301Wrapper::~A301Wrapper() {
  Close();
  if (m_systemCoreClosedLoopAlert) m_systemCoreClosedLoopAlert->Set(false);
}

// ---- Helpers ----------------------------------------------------------------

std::string A301Wrapper::AlertId(const std::string& alertType) const {
  const std::string prefix = "A301(" + std::to_string(m_a301->GetDeviceId()) + ")";
  if (auto name = m_config->GetTelemetryName()) return prefix + "_" + alertType + "_" + *name;
  return prefix + "_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)) + "_" + alertType;
}

double A301Wrapper::MechanismToRotorRatio() const {
  return m_config->GetMotorGearing()
      .value_or(gearing::MechanismGearing::kOne)
      .GetMechanismToRotorRatio();
}

double A301Wrapper::MechanismToExternalEncoderRatio() const {
  // A direct conversion factor (encoder rotations -> mechanism rotations) overrides the gearing.
  if (auto cf = m_config->GetExternalEncoderConversionFactor(); cf && *cf != 0.0) return 1.0 / *cf;
  return m_config->GetExternalEncoderGearing()
      .value_or(gearing::MechanismGearing::kOne)
      .GetMechanismToRotorRatio();
}

double A301Wrapper::ZeroOffsetRotations() const {
  if (auto offset = m_config->GetExternalEncoderZeroOffset())
    return offset->value() * MechanismToExternalEncoderRatio();
  return 0.0;
}

void A301Wrapper::LoadClosedLoopController(const SmartMotorControllerConfig& config) {
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

void A301Wrapper::CreateMotorSim() {
  auto plant = wpi::math::Models::SingleJointedArmFromPhysicalConstants(m_motor, m_config->GetMOI(),
                                                                        MechanismToRotorRatio());
  m_motorSim.emplace(plant, m_motor);
  SetSimSupplier(std::make_shared<simulation::DCMotorSimSupplier>(*m_motorSim, *this));
}

// ---- Configuration ----------------------------------------------------------

bool A301Wrapper::ApplyConfig(const SmartMotorControllerConfig& config) {
  // Like the Java wrapper, the applied config becomes this controller's config.
  m_config = const_cast<SmartMotorControllerConfig*>(&config);
  config.ResetValidationCheck();
  if (m_systemCoreClosedLoopAlert) m_systemCoreClosedLoopAlert->Set(false);

  if (config.GetVendorConfig().has_value())
    throw exceptions::SmartMotorControllerConfigurationException(
        "A301 has no vendor config", "Vendor config could not be applied",
        "WithVendorConfig should be removed");
  if (config.GetVendorControlRequest().has_value())
    throw exceptions::SmartMotorControllerConfigurationException(
        "A301(" + std::to_string(m_a301->GetDeviceId()) +
            ") does not support the custom control requests!",
        "Cannot use given control request", "WithVendorControlRequest()");
  config.GetResetPreviousConfig();

  // The closed loop, motion profiles and feedforwards run on the SystemCore.
  LoadClosedLoopController(config);
  if (m_lqr && config.GetClosedLoopTolerance())
    throw std::invalid_argument("[Error] Closed loop tolerance is not supported in LQR mode.");
  ConfigureSoftwarePID(config);
  config.HasTrapezoidProfile();
  config.HasExponentialProfile();
  config.GetMechanismLowerLimit();
  config.GetMechanismUpperLimit();
  config.GetTemperatureCutoff();
  config.GetClosedLoopControllerMaximumVoltage();
  config.GetFeedbackSynchronizationThreshold();
  config.GetMotorGearing();

  if (m_closedLoopControllerThread) {
    StopClosedLoopController();
    m_closedLoopControllerThread.reset();
  }
  m_closedLoopControllerThread =
      std::make_unique<wpi::Notifier>([this] { IterateClosedLoopController(); });
  if (auto name = config.GetTelemetryName(); name) m_closedLoopControllerThread->SetName(*name);
  const bool closedLoop = config.GetMotorControllerMode() == ControlMode::CLOSED_LOOP;
  // Read the period even in closed loop mode, where the base class applies it, so it is validated.
  const auto closedLoopControlPeriod = config.GetClosedLoopControlPeriod();
  if (!closedLoop && closedLoopControlPeriod)
    throw std::invalid_argument(
        "[Error] Closed loop control period is only supported in closed loop mode.");

  // The A301 has no ramp rates, current limits or voltage compensation.
  if (config.GetOpenLoopRampRate() || config.GetClosedLoopRampRate())
    throw exceptions::SmartMotorControllerConfigurationException(
        "A301 does not support ramp rates", "Ramp rate could not be applied",
        "WithOpenLoopRampRate and WithClosedLoopRampRate should be removed");
  if (config.GetStatorCurrentLimit() || config.GetSupplyCurrentLimit())
    throw exceptions::SmartMotorControllerConfigurationException(
        "A301 does not support current limits", "Current limit could not be applied",
        "WithStatorCurrentLimit and WithSupplyCurrentLimit should be removed");
  if (config.GetVoltageCompensation())
    throw exceptions::SmartMotorControllerConfigurationException(
        "A301 does not support voltage compensation", "Voltage compensation could not be applied",
        "WithVoltageCompensation should be removed");

  if (auto inverted = config.GetMotorInverted()) m_a301->SetInverted(*inverted);
  if (config.GetEncoderInverted())
    throw std::invalid_argument("[ERROR] A301 internal encoder cannot be inverted!");
  if (auto zp = config.GetZeroPower()) SetZeroPower(*zp);

  // Wrapping sets the absolute encoder's range; continuous wrapping is handled by the closed loop
  // controller. Read the maximum first, since it validates the range.
  config.GetWrappingMax();
  config.GetWrappingMin();
  config.IsWrappingContinuous();

  // External encoder: the A301's absolute encoder.
  m_useAbsoluteEncoder = false;
  config.GetUseExternalFeedback();
  if (auto enc = config.GetExternalEncoder()) {
    auto* a301 = std::any_cast<first::a301::A301*>(&*enc);
    if (!a301 || *a301 != m_a301)
      throw std::invalid_argument(
          "[ERROR] Unsupported external encoder: A301Wrapper accepts its own first::a301::A301*, "
          "for its absolute encoder");
    m_useAbsoluteEncoder = true;
    config.GetExternalEncoderGearing();
    config.GetExternalEncoderInverted();
    config.GetExternalEncoderZeroOffset();
  } else {
    if (config.GetExternalEncoderZeroOffset().has_value())
      throw exceptions::SmartMotorControllerConfigurationException(
          "Zero offset is only available for external encoders", "Zero offset could not be applied",
          "WithExternalEncoder(encoder)");
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
  }

  // Start from the starting position, or from the absolute encoder without one.
  if (auto startingPosition = config.GetStartingPosition()) {
    m_a301->SetRelativeEncoderPosition(startingPosition->value() * MechanismToRotorRatio());
    if (m_simSupplier) m_simSupplier->SetMechanismPosition(*startingPosition);
  } else if (m_useAbsoluteEncoder) {
    SeedRelativeEncoder();
  }

  if (!config.GetFollowers().empty())
    throw exceptions::SmartMotorControllerConfigurationException(
        "A301 cannot be followed by another motor controller", "Followers could not be applied",
        "WithLooselyCoupledFollowers instead of WithFollowers");
  LoadLooselyCoupledFollowers();

  config.ValidateBasicOptions();
  config.ValidateExternalEncoderOptions();

  if (closedLoop) {
    m_systemCoreClosedLoopAlert->Set(true);
    StartClosedLoopController();
  }
  return true;
}

// ---- Simulation -------------------------------------------------------------

void A301Wrapper::SetupSimulation() {
  if (!wpi::RobotBase::IsSimulation()) return;
  if (!m_motorSim) CreateMotorSim();
  if (auto startPos = m_config->GetStartingPosition())
    m_simSupplier->SetMechanismPosition(*startPos);
}

void A301Wrapper::SimIterate() {
  if (!wpi::RobotBase::IsSimulation() || !m_simSupplier) return;
  // Step the physics only if the mechanism has not already stepped them this loop.
  if (!m_simSupplier->GetUpdatedSim()) {
    m_simSupplier->UpdateSim();
    m_simSupplier->StarveUpdateSim();
    simulation::BatterySim::CalculateVoltage(BatterySimKey(), m_simSupplier->GetSupplyCurrent());
  }
}

// ---- Encoder sync -----------------------------------------------------------

std::optional<wpi::units::turn_t> A301Wrapper::GetExternalEncoderSensorPosition() {
  if (!m_useAbsoluteEncoder) return std::nullopt;
  double rotations = 0.0;
  if (wpi::RobotBase::IsSimulation() && m_simSupplier) {
    // REVLib does not simulate the A301, so the encoder follows the simulated mechanism.
    rotations = m_simSupplier->GetMechanismPosition().value() * MechanismToExternalEncoderRatio();
  } else {
    const double raw = m_a301->GetAbsoluteEncoderPosition().Get(0.0);
    rotations = (m_config->GetExternalEncoderInverted().value_or(false) ? -raw : raw) -
                ZeroOffsetRotations();
  }
  // The absolute encoder reports angles in the wrapping range, or [0, 1) rotations.
  return wpi::units::turn_t{wpi::math::InputModulus(
      rotations, m_config->GetWrappingMin().value_or(wpi::units::turn_t{0.0}).value(),
      m_config->GetWrappingMax().value_or(wpi::units::turn_t{1.0}).value())};
}

void A301Wrapper::SeedRelativeEncoder() {
  if (auto mechanismAngle = GetExternalEncoderMechanismPosition())
    m_a301->SetRelativeEncoderPosition(mechanismAngle->value() * MechanismToRotorRatio());
}

void A301Wrapper::SynchronizeRelativeEncoder() {
  auto threshold = m_config->GetFeedbackSynchronizationThreshold();
  if (!threshold) return;
  auto externalEncoderAngle = GetExternalEncoderMechanismPosition();
  if (!externalEncoderAngle) return;
  if (std::abs((GetRelativeMechanismPosition() - *externalEncoderAngle).value()) >
      threshold->value())
    SeedRelativeEncoder();
}

// ---- Open-loop outputs ------------------------------------------------------

void A301Wrapper::SetDutyCycle(double dc) {
  m_a301->SetThrottle(dc);
  if (m_simSupplier) {
    m_simDutyCycle = std::clamp(dc, -1.0, 1.0);
    m_simSupplier->SetMechanismStatorDutyCycle(m_simDutyCycle);
  }
  if (dc == 0.0) {
    for (auto* f : m_looseFollowers) f->SetDutyCycle(dc);
  }
}

double A301Wrapper::GetDutyCycle() {
  return m_simSupplier ? m_simDutyCycle : m_a301->GetAppliedOutput().Get(0.0);
}

void A301Wrapper::SetVoltage(wpi::units::volt_t voltage) {
  m_a301->SetVoltage(voltage);
  if (m_simSupplier) {
    // The A301 cannot apply more than its supply voltage.
    const auto supplyVoltage = m_simSupplier->GetMechanismSupplyVoltage();
    m_simDutyCycle = std::clamp(voltage.value() / supplyVoltage.value(), -1.0, 1.0);
    m_simSupplier->SetMechanismStatorVoltage(supplyVoltage * m_simDutyCycle);
  }
}

wpi::units::volt_t A301Wrapper::GetVoltage() {
  if (m_simSupplier) return m_simSupplier->GetMechanismSupplyVoltage() * m_simDutyCycle;
  return wpi::units::volt_t{m_a301->GetAppliedOutput().Get(0.0) * m_a301->GetBusVoltage().Get(0.0)};
}

// ---- Closed-loop setpoints --------------------------------------------------

void A301Wrapper::SetPosition(wpi::units::turn_t angle) {
  m_setpointVelocity.reset();
  m_setpointFeedforwardForce.reset();
  m_setpointPosition = angle;
  ForwardPositionToFollowers(angle);
}

void A301Wrapper::SetPosition(wpi::units::meter_t distance) {
  SetPosition(m_config->ConvertToMechanism(distance));
}

void A301Wrapper::SetVelocity(wpi::units::turns_per_second_t velocity) {
  m_setpointPosition.reset();
  m_setpointVelocity = velocity;
  m_setpointFeedforwardForce.reset();
  ForwardVelocityToFollowers(velocity);
}

void A301Wrapper::SetVelocity(wpi::units::turns_per_second_t velocity,
                              wpi::units::newton_t feedforwardForce) {
  m_setpointPosition.reset();
  m_setpointVelocity = velocity;
  m_setpointFeedforwardForce = feedforwardForce;
  for (auto* f : m_looseFollowers) f->SetVelocity(velocity, feedforwardForce);
}

void A301Wrapper::SetVelocity(wpi::units::meters_per_second_t velocity) {
  SetVelocity(m_config->ConvertToMechanism(velocity));
}

// ---- Encoder writes ---------------------------------------------------------

void A301Wrapper::SetEncoderPosition(wpi::units::turn_t angle) {
  if (m_useAbsoluteEncoder && !wpi::RobotBase::IsSimulation()) {
    // Write the raw reading that the zero offset and inversion turn into this angle.
    const double encoderRotations =
        angle.value() * MechanismToExternalEncoderRatio() + ZeroOffsetRotations();
    m_a301->SetAbsoluteEncoderPosition(m_config->GetExternalEncoderInverted().value_or(false)
                                           ? -encoderRotations
                                           : encoderRotations);
  }
  m_a301->SetRelativeEncoderPosition(angle.value() * MechanismToRotorRatio());
  if (m_simSupplier) m_simSupplier->SetMechanismPosition(angle);
}

void A301Wrapper::SetEncoderPosition(wpi::units::meter_t distance) {
  SetEncoderPosition(m_config->ConvertToMechanism(distance));
}

void A301Wrapper::SetEncoderVelocity(wpi::units::turns_per_second_t velocity) {
  if (!wpi::RobotBase::IsSimulation())
    throw std::runtime_error("A301 does not support setting encoder velocity.");
  if (m_simSupplier) m_simSupplier->SetMechanismVelocity(velocity);
}

void A301Wrapper::SetEncoderVelocity(wpi::units::meters_per_second_t velocity) {
  SetEncoderVelocity(m_config->ConvertToMechanism(velocity));
}

// ---- Encoder reads ----------------------------------------------------------

wpi::units::turn_t A301Wrapper::GetMechanismPosition() {
  if (m_config->GetUseExternalFeedback()) {
    if (auto external = GetExternalEncoderMechanismPosition()) return *external;
  }
  return GetRelativeMechanismPosition();
}

wpi::units::turns_per_second_t A301Wrapper::GetMechanismVelocity() {
  return GetRelativeMechanismVelocity();
}

wpi::units::turn_t A301Wrapper::GetRelativeMechanismPosition() {
  return GetRotorPosition() / MechanismToRotorRatio();
}

wpi::units::turns_per_second_t A301Wrapper::GetRelativeMechanismVelocity() {
  return GetRotorVelocity() / MechanismToRotorRatio();
}

wpi::units::turns_per_second_squared_t A301Wrapper::GetMechanismAcceleration() {
  return wpi::units::turns_per_second_squared_t{
      m_accelFilter.Derivative(GetMechanismVelocity().value())};
}

wpi::units::turn_t A301Wrapper::GetRotorPosition() {
  if (wpi::RobotBase::IsSimulation() && m_simSupplier) return m_simSupplier->GetRotorPosition();
  return wpi::units::turn_t{m_a301->GetRelativeEncoderPosition().Get(0.0)};
}

wpi::units::turns_per_second_t A301Wrapper::GetRotorVelocity() {
  if (wpi::RobotBase::IsSimulation() && m_simSupplier) return m_simSupplier->GetRotorVelocity();
  // REVLib reports the velocity in RPM.
  return wpi::units::turns_per_second_t{m_a301->GetEncoderVelocity().Get(0.0) / 60.0};
}

wpi::units::meter_t A301Wrapper::GetMeasurementPosition() {
  return m_config->ConvertFromMechanism(GetMechanismPosition());
}
wpi::units::meters_per_second_t A301Wrapper::GetMeasurementVelocity() {
  return m_config->ConvertFromMechanism(GetMechanismVelocity());
}
wpi::units::meters_per_second_squared_t A301Wrapper::GetMeasurementAcceleration() {
  return m_config->ConvertFromMechanism(GetMechanismAcceleration());
}

std::optional<wpi::units::turn_t> A301Wrapper::GetExternalEncoderMechanismPosition() {
  auto sensor = GetExternalEncoderSensorPosition();
  if (!sensor) return std::nullopt;
  return *sensor / MechanismToExternalEncoderRatio();
}

std::optional<wpi::units::turns_per_second_t> A301Wrapper::GetExternalEncoderMechanismVelocity() {
  if (!m_useAbsoluteEncoder) return std::nullopt;
  return GetRelativeMechanismVelocity();
}

// ---- Motor status -----------------------------------------------------------

std::optional<wpi::units::ampere_t> A301Wrapper::GetSupplyCurrent() {
  if (m_simSupplier) return m_simSupplier->GetSupplyCurrent();
  return std::nullopt;
}

wpi::units::ampere_t A301Wrapper::GetStatorCurrent() {
  if (m_simSupplier) return m_simSupplier->GetStatorCurrent();
  return wpi::units::ampere_t{m_a301->GetMotorCurrent().Get(0.0)};
}

wpi::units::celsius_t A301Wrapper::GetTemperature() {
  return wpi::units::celsius_t{m_a301->GetMotorTemperature().Get(0.0)};
}

wpi::math::DCMotor A301Wrapper::GetDCMotor() { return m_motor; }

// ---- Live-tuning setters ----------------------------------------------------

void A301Wrapper::SetZeroPower(MotorMode mode) {
  m_a301->SetIdleMode(mode == MotorMode::BRAKE ? first::a301::A301::IdleMode::kBrake
                                               : first::a301::A301::IdleMode::kCoast);
}

void A301Wrapper::SetMotorInverted(bool inverted) {
  m_config->WithMotorInverted(inverted);
  m_a301->SetInverted(inverted);
}

void A301Wrapper::SetEncoderInverted(bool) {
  throw std::runtime_error("A301 internal encoder cannot be inverted.");
}

void A301Wrapper::ApplyFeedback(double kP, double kI, double kD) {
  m_config->WithFeedback(kP, kI, kD, m_slot);
  if (m_pid) {
    m_pid->SetP(kP);
    m_pid->SetI(kI);
    m_pid->SetD(kD);
  }
}

void A301Wrapper::SetKp(double kP) {
  const auto gains = m_config->GetSlotGains(m_slot);
  ApplyFeedback(kP, gains.kI, gains.kD);
  for (auto* f : m_looseFollowers) f->SetKp(kP);
}

void A301Wrapper::SetKi(double kI) {
  const auto gains = m_config->GetSlotGains(m_slot);
  ApplyFeedback(gains.kP, kI, gains.kD);
  for (auto* f : m_looseFollowers) f->SetKi(kI);
}

void A301Wrapper::SetKd(double kD) {
  const auto gains = m_config->GetSlotGains(m_slot);
  ApplyFeedback(gains.kP, gains.kI, kD);
  for (auto* f : m_looseFollowers) f->SetKd(kD);
}

void A301Wrapper::SetFeedback(double kP, double kI, double kD) {
  ApplyFeedback(kP, kI, kD);
  for (auto* f : m_looseFollowers) f->SetFeedback(kP, kI, kD);
}

void A301Wrapper::SetKs(double kS) {
  UpdateConfigFeedforward(*m_config, m_slot, [&](NativeFeedforward& v) { v.kS = kS; });
  for (auto* f : m_looseFollowers) f->SetKs(kS);
}

void A301Wrapper::SetKv(double kV) {
  UpdateConfigFeedforward(*m_config, m_slot, [&](NativeFeedforward& v) { v.kV = kV; });
  for (auto* f : m_looseFollowers) f->SetKv(kV);
}

void A301Wrapper::SetKa(double kA) {
  UpdateConfigFeedforward(*m_config, m_slot, [&](NativeFeedforward& v) { v.kA = kA; });
  for (auto* f : m_looseFollowers) f->SetKa(kA);
}

void A301Wrapper::SetKg(double kG) {
  UpdateConfigFeedforward(*m_config, m_slot, [&](NativeFeedforward& v) { v.kG = kG; });
  for (auto* f : m_looseFollowers) f->SetKg(kG);
}

void A301Wrapper::SetFeedforward(double kS, double kV, double kA, double kG) {
  UpdateConfigFeedforward(*m_config, m_slot, [&](NativeFeedforward& v) {
    v.kS = kS;
    v.kV = kV;
    v.kA = kA;
    v.kG = kG;
  });
  for (auto* f : m_looseFollowers) f->SetFeedforward(kS, kV, kA, kG);
}

void A301Wrapper::SetStatorCurrentLimit(wpi::units::ampere_t) {
  throw std::runtime_error("A301 does not support current limits.");
}

void A301Wrapper::SetSupplyCurrentLimit(wpi::units::ampere_t) {
  throw std::runtime_error("A301 does not support current limits.");
}

void A301Wrapper::SetClosedLoopRampRate(wpi::units::second_t) {
  throw std::runtime_error("A301 does not support ramp rates.");
}

void A301Wrapper::SetOpenLoopRampRate(wpi::units::second_t) {
  throw std::runtime_error("A301 does not support ramp rates.");
}

void A301Wrapper::SetMechanismUpperLimit(wpi::units::turn_t upper) {
  if (auto lower = m_config->GetMechanismLowerLimit()) m_config->WithMechanismLimits(*lower, upper);
  for (auto* f : m_looseFollowers) f->SetMechanismUpperLimit(upper);
}

void A301Wrapper::SetMechanismLowerLimit(wpi::units::turn_t lower) {
  if (auto upper = m_config->GetMechanismUpperLimit()) m_config->WithMechanismLimits(lower, *upper);
  for (auto* f : m_looseFollowers) f->SetMechanismLowerLimit(lower);
}

void A301Wrapper::SetMechanismLimits(wpi::units::turn_t lower, wpi::units::turn_t upper) {
  m_config->WithMechanismLimits(lower, upper);
  for (auto* f : m_looseFollowers) f->SetMechanismLimits(lower, upper);
}

void A301Wrapper::SetMechanismLimitsEnabled(bool enabled) {
  // The closed loop controller on the SystemCore enforces the mechanism limits.
  for (auto* f : m_looseFollowers) f->SetMechanismLimitsEnabled(enabled);
}

void A301Wrapper::SetMeasurementUpperLimit(wpi::units::meter_t upper) {
  auto lowerAngle = m_config->GetMechanismLowerLimit();
  if (!m_config->GetMechanismCircumference() || !lowerAngle) return;
  m_config->WithMeasurementLimits(m_config->ConvertFromMechanism(*lowerAngle), upper);
  for (auto* f : m_looseFollowers) f->SetMeasurementUpperLimit(upper);
}

void A301Wrapper::SetMeasurementLowerLimit(wpi::units::meter_t lower) {
  auto upperAngle = m_config->GetMechanismUpperLimit();
  if (!m_config->GetMechanismCircumference() || !upperAngle) return;
  m_config->WithMeasurementLimits(lower, m_config->ConvertFromMechanism(*upperAngle));
  for (auto* f : m_looseFollowers) f->SetMeasurementLowerLimit(lower);
}

void A301Wrapper::SetMotionProfileMaxVelocity(wpi::units::turns_per_second_t vel) {
  if (!m_config->GetVelocityTrapezoidalProfileInUse()) {
    // Keep the tuned constraints in the config, so tuning the next one keeps this one.
    if (auto accLin = m_config->GetTrapMaxAccelLinear();
        accLin && !m_config->GetTrapMaxVelocityTurns()) {
      m_config->WithLinearTrapezoidProfile(m_config->ConvertFromMechanism(vel), *accLin);
    } else if (auto acc = m_config->GetTrapMaxAccelTurns()) {
      m_config->WithTrapezoidProfile(vel, *acc);
    }
  }
  for (auto* f : m_looseFollowers) f->SetMotionProfileMaxVelocity(vel);
}

void A301Wrapper::SetMotionProfileMaxVelocity(wpi::units::meters_per_second_t vel) {
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
  for (auto* f : m_looseFollowers) f->SetMotionProfileMaxVelocity(vel);
}

void A301Wrapper::SetMotionProfileMaxAcceleration(wpi::units::turns_per_second_squared_t acc) {
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
  for (auto* f : m_looseFollowers) f->SetMotionProfileMaxAcceleration(acc);
}

void A301Wrapper::SetMotionProfileMaxAcceleration(wpi::units::meters_per_second_squared_t acc) {
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
  for (auto* f : m_looseFollowers) f->SetMotionProfileMaxAcceleration(acc);
}

void A301Wrapper::SetMotionProfileMaxJerk(
    wpi::units::angular_jerk::turns_per_second_cubed_t maxJerk) {
  if (m_config->GetVelocityTrapezoidalProfileInUse()) {
    if (auto accLin = m_config->GetTrapMaxVelocityLinear();
        accLin && !m_config->GetTrapMaxVelocityTurns()) {
      m_config->WithVelocityTrapezoidProfile(
          wpi::units::meters_per_second_squared_t{accLin->value()},
          m_config->ConvertFromMechanism(maxJerk));
    } else if (auto acc = m_config->GetTrapMaxVelocityTurns()) {
      m_config->WithVelocityTrapezoidProfile(wpi::units::turns_per_second_squared_t{acc->value()},
                                             maxJerk);
    }
  }
  for (auto* f : m_looseFollowers) f->SetMotionProfileMaxJerk(maxJerk);
}

void A301Wrapper::SetExponentialProfile(std::optional<double> kV, std::optional<double> kA,
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

void A301Wrapper::SetClosedLoopSlot(ClosedLoopControllerSlot slot) {
  m_slot = slot;
  LoadClosedLoopController(*m_config);
  ConfigureSoftwarePID(*m_config);
  for (auto* f : m_looseFollowers) f->SetClosedLoopSlot(slot);
}

void A301Wrapper::SetMechanismGearing(const gearing::MechanismGearing& gearing) {
  SmartMotorController::SetMechanismGearing(gearing);
  if (wpi::RobotBase::IsSimulation()) {
    // The simulated motor's reduction comes from the gearing.
    const auto position =
        m_simSupplier ? m_simSupplier->GetMechanismPosition() : wpi::units::turn_t{0.0};
    CreateMotorSim();
    m_simSupplier->SetMechanismPosition(position);
  }
}

void A301Wrapper::SetMechanismCircumference(wpi::units::meter_t circumference) {
  SmartMotorController::SetMechanismCircumference(circumference);
  for (auto* f : m_looseFollowers) f->SetMechanismCircumference(circumference);
}

SmartMotorControllerConfig& A301Wrapper::GetConfig() { return *m_config; }
void* A301Wrapper::GetMotorController() { return m_a301; }
void* A301Wrapper::GetMotorControllerConfig() { return nullptr; }

telemetry::UnsupportedTelemetryFields A301Wrapper::GetUnsupportedTelemetryFields() {
  return {std::nullopt, std::vector<telemetry::DoubleTelemetryField>{
                            telemetry::DoubleTelemetryField::SupplyCurrent,
                            telemetry::DoubleTelemetryField::SupplyCurrentLimit,
                            telemetry::DoubleTelemetryField::StatorCurrentLimit,
                            telemetry::DoubleTelemetryField::ClosedloopRampRate,
                            telemetry::DoubleTelemetryField::OpenloopRampRate}};
}

}  // namespace yams::motorcontrollers::local
