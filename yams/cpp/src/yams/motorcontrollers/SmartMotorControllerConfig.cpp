// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"

#include <wpi/system/Errors.hpp>
#include <wpi/framework/RobotBase.hpp>
#include <wpi/simulation/SingleJointedArmSim.hpp>
#include <wpi/math/system/Models.hpp>

#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "yams/exceptions.hpp"

namespace yams::motorcontrollers {

int SmartMotorControllerConfig::SlotIndex(ClosedLoopControllerSlot slot) const {
  return static_cast<int>(slot);
}

// ---- Validation ----------------------------------------------------------

std::string_view SmartMotorControllerConfig::ToString(BasicOptions opt) {
  switch (opt) {
    case BasicOptions::VendorControlRequest:
      return "VendorControlRequest";
    case BasicOptions::ControlMode:
      return "ControlMode";
    case BasicOptions::ClosedLoopMaxVoltage:
      return "ClosedLoopMaxVoltage";
    case BasicOptions::StartingPosition:
      return "StartingPosition";
    case BasicOptions::EncoderInverted:
      return "EncoderInverted";
    case BasicOptions::MotorInverted:
      return "MotorInverted";
    case BasicOptions::TemperatureCutoff:
      return "TemperatureCutoff";
    case BasicOptions::UpperLimit:
      return "UpperLimit";
    case BasicOptions::LowerLimit:
      return "LowerLimit";
    case BasicOptions::IdleMode:
      return "IdleMode";
    case BasicOptions::StatorCurrentLimit:
      return "StatorCurrentLimit";
    case BasicOptions::SupplyCurrentLimit:
      return "SupplyCurrentLimit";
    case BasicOptions::ClosedLoopRampRate:
      return "ClosedLoopRampRate";
    case BasicOptions::OpenLoopRampRate:
      return "OpenLoopRampRate";
    case BasicOptions::ExternalEncoder:
      return "ExternalEncoder";
    case BasicOptions::Gearing:
      return "Gearing";
    case BasicOptions::SlotGains:
      return "SlotGains";
    case BasicOptions::TrapezoidProfile:
      return "TrapezoidProfile";
    case BasicOptions::ExponentialProfile:
      return "ExponentialProfile";
    case BasicOptions::ContinuousWrapping:
      return "ContinuousWrapping";
    default:
      return "Unknown";
  }
}

std::string_view SmartMotorControllerConfig::ToString(ExternalEncoderOptions opt) {
  switch (opt) {
    case ExternalEncoderOptions::ZeroOffset:
      return "ZeroOffset";
    case ExternalEncoderOptions::DiscontinuityPoint:
      return "DiscontinuityPoint";
    case ExternalEncoderOptions::UseExternalFeedback:
      return "UseExternalFeedback";
    case ExternalEncoderOptions::ExternalGearing:
      return "ExternalGearing";
    case ExternalEncoderOptions::ExternalEncoderInverted:
      return "ExternalEncoderInverted";
    default:
      return "Unknown";
  }
}

void SmartMotorControllerConfig::ResetValidationCheck() const {
  m_basicOptions = {
      BasicOptions::VendorControlRequest,
      BasicOptions::ControlMode,
      BasicOptions::ClosedLoopMaxVoltage,
      BasicOptions::StartingPosition,
      BasicOptions::EncoderInverted,
      BasicOptions::MotorInverted,
      BasicOptions::TemperatureCutoff,
      BasicOptions::UpperLimit,
      BasicOptions::LowerLimit,
      BasicOptions::IdleMode,
      BasicOptions::StatorCurrentLimit,
      BasicOptions::SupplyCurrentLimit,
      BasicOptions::ClosedLoopRampRate,
      BasicOptions::OpenLoopRampRate,
      BasicOptions::ExternalEncoder,
      BasicOptions::Gearing,
      BasicOptions::SlotGains,
      BasicOptions::TrapezoidProfile,
      BasicOptions::ExponentialProfile,
      BasicOptions::ContinuousWrapping,
  };
  m_externalEncoderOptions = {
      ExternalEncoderOptions::ZeroOffset,
      ExternalEncoderOptions::DiscontinuityPoint,
      ExternalEncoderOptions::UseExternalFeedback,
      ExternalEncoderOptions::ExternalGearing,
      ExternalEncoderOptions::ExternalEncoderInverted,
  };
}

void SmartMotorControllerConfig::ValidateBasicOptions() const {
  if (m_basicOptions.empty()) return;
  std::cerr << "========= Basic Option Validation FAILED ==========\n";
  for (auto opt : m_basicOptions)
    std::cerr << "  Missing required option: " << ToString(opt) << "\n";
  throw exceptions::SmartMotorControllerConfigurationException(
      "Basic options are not fully applied", "ApplyConfig did not access all tracked options",
      "Call the corresponding getter for each missing option in ApplyConfig");
}

void SmartMotorControllerConfig::ValidateExternalEncoderOptions() const {
  if (m_externalEncoderOptions.empty()) return;
  std::cerr << "========= External Encoder Option Validation FAILED ==========\n";
  for (auto opt : m_externalEncoderOptions)
    std::cerr << "  Missing required option: " << ToString(opt) << "\n";
  throw exceptions::SmartMotorControllerConfigurationException(
      "External encoder options are not fully applied",
      "ApplyConfig did not access all tracked external encoder options",
      "Call the corresponding getter for each missing option in ApplyConfig");
}

// ---- Feedback -----------------------------------------------------------

SmartMotorControllerConfig& SmartMotorControllerConfig::WithFeedback(
    double kP, double kI, double kD, ClosedLoopControllerSlot slot) {
  auto& s = m_slots[SlotIndex(slot)];
  s.kP = kP;
  s.kI = kI;
  s.kD = kD;
  return *this;
}

// ---- Feedforward ---------------------------------------------------------

SmartMotorControllerConfig& SmartMotorControllerConfig::WithFeedforward(
    const wpi::math::ArmFeedforward& ff, ClosedLoopControllerSlot slot) {
  // ArmFeedforward's kV/kA are natively per-radian; the hardware closed loop runs in
  // mechanism turns, so convert via the units library before storing the raw gains.
  using kv_unit = wpi::math::SimpleMotorFeedforward<wpi::units::turns>::kv_unit;
  using ka_unit = wpi::math::SimpleMotorFeedforward<wpi::units::turns>::ka_unit;
  auto& s = m_slots[SlotIndex(slot)];
  s.kS = ff.GetKs().value();
  s.kV = wpi::units::unit_t<kv_unit>{ff.GetKv()}.value();
  s.kA = wpi::units::unit_t<ka_unit>{ff.GetKa()}.value();
  s.kG = ff.GetKg().value();
  s.armFF = ff;
  s.elevatorFF.reset();
  s.simpleFF.reset();
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithFeedforward(
    const wpi::math::ElevatorFeedforward& ff, ClosedLoopControllerSlot slot) {
  auto& s = m_slots[SlotIndex(slot)];
  s.kS = ff.GetKs().value();
  s.kV = ff.GetKv().value();
  s.kA = ff.GetKa().value();
  s.kG = ff.GetKg().value();
  s.elevatorFF = ff;
  s.armFF.reset();
  s.simpleFF.reset();
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithFeedforward(
    const wpi::math::SimpleMotorFeedforward<wpi::units::turns>& ff, ClosedLoopControllerSlot slot) {
  using kv_unit = wpi::math::SimpleMotorFeedforward<wpi::units::turns>::kv_unit;
  using ka_unit = wpi::math::SimpleMotorFeedforward<wpi::units::turns>::ka_unit;
  auto& s = m_slots[SlotIndex(slot)];
  s.kS = ff.GetKs().value();
  s.kV = wpi::units::unit_t<kv_unit>{ff.GetKv()}.value();
  s.kA = wpi::units::unit_t<ka_unit>{ff.GetKa()}.value();
  s.simpleFF = ff;
  s.armFF.reset();
  s.elevatorFF.reset();
  return *this;
}

// ---- Motion Profiles -----------------------------------------------------

SmartMotorControllerConfig& SmartMotorControllerConfig::WithTrapezoidProfile(
    wpi::units::turns_per_second_t maxVelocity, wpi::units::turns_per_second_squared_t maxAcceleration) {
  m_trapProfile = wpi::math::TrapezoidProfile<wpi::units::turns>{{maxVelocity, maxAcceleration}};
  m_trapMaxVelTurns = maxVelocity;
  m_trapMaxAccTurns = maxAcceleration;
  m_velocityTrapProfile = false;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithLinearTrapezoidProfile(
    wpi::units::meters_per_second_t maxVelocity, wpi::units::meters_per_second_squared_t maxAcceleration) {
  m_linearTrapProfile = wpi::math::TrapezoidProfile<wpi::units::meters>{{maxVelocity, maxAcceleration}};
  m_trapMaxVelLinear = maxVelocity;
  m_trapMaxAccLinear = maxAcceleration;
  m_velocityTrapProfile = false;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithVelocityTrapezoidProfile(
    wpi::units::turns_per_second_t maxVelocity, wpi::units::turns_per_second_squared_t maxAcceleration) {
  m_trapProfile = wpi::math::TrapezoidProfile<wpi::units::turns>{{maxVelocity, maxAcceleration}};
  m_trapMaxVelTurns = maxVelocity;
  m_trapMaxAccTurns = maxAcceleration;
  m_velocityTrapProfile = true;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithExponentialProfile(
    double kV, double kA, wpi::units::volt_t maxInput) {
  using Profile = wpi::math::ExponentialProfile<wpi::units::turns, wpi::units::volts>;
  m_expoProfile = Profile{Profile::Constraints{maxInput, Profile::kV_t{kV}, Profile::kA_t{kA}}};
  m_linearExpoProfile = std::nullopt;
  m_trapProfile = std::nullopt;
  m_expoMotionMagicKV = kV;
  m_expoMotionMagicKA = kA;
  m_expoMaxInput = maxInput;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithExponentialProfile(
    wpi::units::volt_t maxVolts, wpi::math::DCMotor motor, wpi::units::kilogram_square_meter_t moi) {
  using Profile = wpi::math::ExponentialProfile<wpi::units::turns, wpi::units::volts>;
  m_moi = moi;
  double gearing = m_motorGearing ? m_motorGearing->GetMechanismToRotorRatio() : 1.0;
  auto sys = wpi::math::Models::FlywheelFromPhysicalConstants(motor, moi, gearing);
  // A is [1/s], B is [(rad/s²)/V] from the flywheel velocity model.
  // Convert to turns: kV [V/(turn/s)] = (-A/B) * 2π, kA [V/(turn/s²)] = (2π/B).
  double A = sys.A()(0, 0);
  double B = sys.B()(0, 0);
  double kV = (-A / B) * (2.0 * std::numbers::pi);
  double kA = (2.0 * std::numbers::pi) / B;
  m_expoProfile = Profile{Profile::Constraints{maxVolts, Profile::kV_t{kV}, Profile::kA_t{kA}}};
  m_linearExpoProfile = std::nullopt;
  m_trapProfile = std::nullopt;
  m_expoMotionMagicKV = kV;
  m_expoMotionMagicKA = kA;
  m_expoMaxInput = maxVolts;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithExponentialProfile(
    wpi::units::volt_t maxVolts, wpi::math::DCMotor motor, wpi::units::kilogram_t mass, wpi::units::meter_t drumRadius) {
  using LinearProfile = wpi::math::ExponentialProfile<wpi::units::meters, wpi::units::volts>;
  double gearing = m_motorGearing ? m_motorGearing->GetMechanismToRotorRatio() : 1.0;
  auto sys = wpi::math::Models::ElevatorFromPhysicalConstants(motor, mass, drumRadius, gearing);
  // Extract velocity-row coefficients from the 2-state [position, velocity] system.
  // A[1][1] is [1/s], B[1][0] is [(m/s²)/V] — already in meters, no conversion needed.
  double A = sys.A()(1, 1);
  double B = sys.B()(1, 0);
  double kV = -A / B;   // [V·s/m = V/(m/s)]
  double kA = 1.0 / B;  // [V·s²/m = V/(m/s²)]
  m_linearExpoProfile = LinearProfile{
      LinearProfile::Constraints{maxVolts, LinearProfile::kV_t{kV}, LinearProfile::kA_t{kA}}};
  m_mechanismCircumference = 2.0 * std::numbers::pi * drumRadius;
  m_expoProfile = std::nullopt;
  m_trapProfile = std::nullopt;
  m_linearTrapProfile = std::nullopt;
  // Convert linear kV/kA to turns-based.
  // kV_turns = kV_linear * circumference (V*s/m * m/turn = V*s/turn)
  double circumferenceValue = m_mechanismCircumference->value();
  m_expoMotionMagicKV = kV * circumferenceValue;
  m_expoMotionMagicKA = kA * circumferenceValue;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithExponentialProfile(
    wpi::units::volt_t maxVolts, wpi::units::turns_per_second_t maxVelocity,
    wpi::units::turns_per_second_squared_t maxAcceleration) {
  using Profile = wpi::math::ExponentialProfile<wpi::units::turns, wpi::units::volts>;
  m_expoProfile = Profile{Profile::Constraints{maxVolts, Profile::kV_t{maxVolts / maxVelocity},
                                               Profile::kA_t{maxVolts / maxAcceleration}}};
  m_linearExpoProfile = std::nullopt;
  m_trapProfile = std::nullopt;
  m_expoMotionMagicKV = (maxVolts / maxVelocity).value();
  m_expoMotionMagicKA = (maxVolts / maxAcceleration).value();
  m_expoMaxInput = maxVolts;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithExponentialProfile(
    wpi::math::ExponentialProfile<wpi::units::turns, wpi::units::volts>::Constraints constraints) {
  using Profile = wpi::math::ExponentialProfile<wpi::units::turns, wpi::units::volts>;
  m_expoProfile = Profile{constraints};
  m_linearExpoProfile = std::nullopt;
  m_trapProfile = std::nullopt;
  // A is [1/s], B is [(turn/s²)/V]; derive kV [V*s/turn] and kA [V*s²/turn]
  m_expoMotionMagicKV = -constraints.A.value() / constraints.B.value();
  m_expoMotionMagicKA = 1.0 / constraints.B.value();
  m_expoMaxInput = constraints.maxInput;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithLQR(const math::LQRConfig& lqrConfig,
                                                                ClosedLoopControllerSlot slot) {
  m_slots[SlotIndex(slot)].lqr = lqrConfig;
  return *this;
}

// ---- Gearing / linear ----------------------------------------------------

SmartMotorControllerConfig& SmartMotorControllerConfig::WithMotorGearing(
    const gearing::MechanismGearing& gearing) {
  m_motorGearing = gearing;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithMechanismCircumference(
    wpi::units::meter_t circumference) {
  m_mechanismCircumference = circumference;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithMechanismCircumference(
    wpi::units::meter_t gearPitch, int teeth) {
  return WithMechanismCircumference(gearPitch * teeth);
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithMechanismDiameter(
    wpi::units::meter_t diameter) {
  return WithMechanismCircumference(diameter * std::numbers::pi);
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithMechanismRadius(wpi::units::meter_t radius) {
  return WithMechanismCircumference(radius * 2.0 * std::numbers::pi);
}

// ---- Limits --------------------------------------------------------------

SmartMotorControllerConfig& SmartMotorControllerConfig::WithMechanismLimits(wpi::units::turn_t lower,
                                                                            wpi::units::turn_t upper) {
  if (m_continuousWrappingMax)
    throw exceptions::SmartMotorControllerConfigurationException(
        "Soft limits set while configuring continuous wrapping", "Cannot set soft limits",
        "WithContinuousWrapping() should be removed");
  m_mechLowerLimit = lower;
  m_mechUpperLimit = upper;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithMeasurementLimits(
    wpi::units::meter_t lower, wpi::units::meter_t upper) {
  m_measLowerLimit = lower;
  m_measUpperLimit = upper;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithStatorCurrentLimit(
    wpi::units::ampere_t limit) {
  m_statorCurrentLimit = limit;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithSupplyCurrentLimit(
    wpi::units::ampere_t limit) {
  m_supplyCurrentLimit = limit;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithTemperatureCutoff(
    wpi::units::celsius_t temp) {
  m_temperatureCutoff = temp;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithClosedLoopMaxVoltage(
    wpi::units::volt_t maxV) {
  m_closedLoopMaxVoltage = maxV;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithContinuousWrapping(wpi::units::turn_t min,
                                                                               wpi::units::turn_t max) {
  if (m_mechLowerLimit || m_mechUpperLimit)
    throw exceptions::SmartMotorControllerConfigurationException(
        "Soft limits set while configuring continuous wrapping", "Cannot set continuous wrapping",
        "WithMechanismLimits() should be removed");
  if (m_measLowerLimit || m_measUpperLimit)
    throw exceptions::SmartMotorControllerConfigurationException(
        "Measurement soft limits set while configuring continuous wrapping",
        "Cannot set continuous wrapping", "WithMeasurementLimits() should be removed");
  if (m_mechanismCircumference)
    throw exceptions::SmartMotorControllerConfigurationException(
        "Distance based mechanism used with continuous wrapping", "Cannot set continuous wrapping",
        "WithMechanismCircumference() should be removed");
  m_continuousWrappingMin = min;
  m_continuousWrappingMax = max;
  return *this;
}

// ---- Control behaviour ---------------------------------------------------

SmartMotorControllerConfig& SmartMotorControllerConfig::WithIdleMode(MotorMode mode) {
  m_idleMode = mode;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithClosedLoopMode() {
  m_controlMode = ControlMode::CLOSED_LOOP;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithOpenLoopMode() {
  m_controlMode = ControlMode::OPEN_LOOP;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithClosedLoopControlPeriod(
    wpi::units::second_t p) {
  m_closedLoopPeriod = p;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithOpenLoopRampRate(wpi::units::second_t r) {
  m_openLoopRampRate = r;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithClosedLoopRampRate(wpi::units::second_t r) {
  m_closedLoopRampRate = r;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithMotorInverted(bool inv) {
  m_motorInverted = inv;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithEncoderInverted(bool inv) {
  m_encoderInverted = inv;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithExternalEncoder(std::any encoder) {
  m_externalEncoder = std::move(encoder);
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithUseExternalFeedbackEncoder(bool use) {
  m_useExternalFeedback = use;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithExternalEncoderInverted(bool inverted) {
  m_externalEncoderInverted = inverted;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithExternalEncoderConversionFactor(
    double f) {
  m_externalEncoderConversionFactor = f;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithExternalEncoderZeroOffset(
    wpi::units::turn_t o) {
  m_externalEncoderZeroOffset = o;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithExternalEncoderGearing(
    const gearing::MechanismGearing& gearing) {
  if (gearing.GetRotorToMechanismRatio() > 1.0) {
    WPILIB_ReportWarning(
        "[IMPORTANT] Your gearing is set in a way that the external encoder will exceed the "
        "maximum reading, this WILL result in multiple angles being read as the same angle. "
        "Ignore this warning IF your mechanism will never travel outside of the slice you are "
        "reading. You have been warned!");
  }
  m_externalEncoderGearing = gearing;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithExternalEncoderGearing(
    double reductionRatio) {
  return WithExternalEncoderGearing(gearing::MechanismGearing{reductionRatio});
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithExternalEncoderDiscontinuityPoint(
    wpi::units::turn_t discontinuityPoint) {
  if (discontinuityPoint != wpi::units::turn_t{0.5} && discontinuityPoint != wpi::units::turn_t{1.0}) {
    throw exceptions::SmartMotorControllerConfigurationException(
        "Cannot set external encoder discontinuity point",
        "Discontinuity point must be 0.5 or 1 rotations",
        "WithExternalEncoderDiscontinuityPoint(wpi::units::turn_t{0.5})");
  }
  m_externalEncoderDiscontinuityPoint = discontinuityPoint;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithTelemetry(
    const std::string& name, TelemetryVerbosity verbosity) {
  m_telemetryName = name;
  m_verbosity = verbosity;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithSubsystem(wpi::cmd::SubsystemBase* sys) {
  m_subsystem = sys;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithSimMotor(wpi::math::DCMotor motor) {
  m_simMotor = motor;
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithMOI(
    wpi::units::kilogram_square_meter_t moi) {
  m_moi = moi;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithMOI(wpi::units::meter_t length,
                                                                wpi::units::kilogram_t mass) {
  m_moi = wpi::sim::SingleJointedArmSim::EstimateMOI(length, mass);
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithStartingPosition(
    wpi::units::degree_t startingAngle) {
  if (m_startingPositionDistance.has_value())
    throw std::invalid_argument(
        "Cannot set starting position as both an angle and a distance. "
        "Call only one of WithStartingPosition(degree_t) or WithStartingPosition(meter_t).");
  m_startingPosition = wpi::units::turn_t{startingAngle};
  return *this;
}
SmartMotorControllerConfig& SmartMotorControllerConfig::WithStartingPosition(
    wpi::units::meter_t startingDistance) {
  if (m_startingPosition.has_value())
    throw std::invalid_argument(
        "Cannot set starting position as both an angle and a distance. "
        "Call only one of WithStartingPosition(degree_t) or WithStartingPosition(meter_t).");
  if (!m_mechanismCircumference.has_value())
    throw std::invalid_argument(
        "WithStartingPosition(meter_t) requires WithMechanismCircumference to be called first.");
  m_startingPositionDistance = startingDistance;
  m_startingPosition = wpi::units::turn_t{startingDistance.value() / m_mechanismCircumference->value()};
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithVendorConfig(std::any cfg) {
  m_vendorConfig = std::move(cfg);
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithVendorControlRequest(std::any req) {
  m_vendorControlRequest = std::move(req);
  return *this;
}

// ---- Simulation overrides ------------------------------------------------

SmartMotorControllerConfig& SmartMotorControllerConfig::WithSimStartingPosition(
    wpi::units::degree_t startingAngle) {
  m_simStartingPosition = wpi::units::turn_t{startingAngle};
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithSimStartingPosition(
    wpi::units::meter_t startingDistance) {
  if (!m_mechanismCircumference.has_value())
    throw std::invalid_argument(
        "WithSimStartingPosition(meter_t) requires WithMechanismCircumference to be called first.");
  m_simStartingPosition =
      wpi::units::turn_t{startingDistance.value() / m_mechanismCircumference->value()};
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithSimFeedforward(
    const wpi::math::ArmFeedforward& ff, ClosedLoopControllerSlot slot) {
  auto& sim = m_simGains[SlotIndex(slot)];
  sim.armFF = ff;
  sim.elevatorFF.reset();
  sim.simpleFF.reset();
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithSimFeedforward(
    const wpi::math::ElevatorFeedforward& ff, ClosedLoopControllerSlot slot) {
  auto& sim = m_simGains[SlotIndex(slot)];
  sim.elevatorFF = ff;
  sim.armFF.reset();
  sim.simpleFF.reset();
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithSimFeedforward(
    const wpi::math::SimpleMotorFeedforward<wpi::units::turns>& ff, ClosedLoopControllerSlot slot) {
  auto& sim = m_simGains[SlotIndex(slot)];
  sim.simpleFF = ff;
  sim.armFF.reset();
  sim.elevatorFF.reset();
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithSimClosedLoopController(
    double kP, double kI, double kD, ClosedLoopControllerSlot slot) {
  auto& sim = m_simGains[SlotIndex(slot)];
  sim.kP = kP;
  sim.kI = kI;
  sim.kD = kD;
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithSimTrapezoidProfile(
    wpi::units::turns_per_second_t maxVelocity, wpi::units::turns_per_second_squared_t maxAcceleration) {
  m_simTrapProfile = wpi::math::TrapezoidProfile<wpi::units::turns>{{maxVelocity, maxAcceleration}};
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithSimTrapezoidProfile(
    wpi::units::meters_per_second_t maxVelocity, wpi::units::meters_per_second_squared_t maxAcceleration) {
  m_simLinearTrapProfile = wpi::math::TrapezoidProfile<wpi::units::meters>{{maxVelocity, maxAcceleration}};
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithSimExponentialProfile(
    wpi::math::ExponentialProfile<wpi::units::turns, wpi::units::volts>::Constraints constraints) {
  using Profile = wpi::math::ExponentialProfile<wpi::units::turns, wpi::units::volts>;
  m_simExpoProfile = Profile{constraints};
  return *this;
}

// ---- Getters -------------------------------------------------------------

SmartMotorControllerConfig::PIDGains SmartMotorControllerConfig::GetSlotGains(
    ClosedLoopControllerSlot slot) const {
  m_basicOptions.erase(BasicOptions::SlotGains);
  PIDGains result = m_slots[SlotIndex(slot)];
  if (wpi::RobotBase::IsSimulation()) {
    const auto& sim = m_simGains[SlotIndex(slot)];
    if (sim.kP) result.kP = *sim.kP;
    if (sim.kI) result.kI = *sim.kI;
    if (sim.kD) result.kD = *sim.kD;
    if (sim.armFF) {
      result.armFF = sim.armFF;
      result.elevatorFF.reset();
      result.simpleFF.reset();
    } else if (sim.elevatorFF) {
      result.elevatorFF = sim.elevatorFF;
      result.armFF.reset();
      result.simpleFF.reset();
    } else if (sim.simpleFF) {
      result.simpleFF = sim.simpleFF;
      result.armFF.reset();
      result.elevatorFF.reset();
    }
  }
  return result;
}

std::optional<wpi::math::ArmFeedforward> SmartMotorControllerConfig::GetArmFeedforward(
    ClosedLoopControllerSlot slot) const {
  m_basicOptions.erase(BasicOptions::SlotGains);
  if (wpi::RobotBase::IsSimulation() && m_simGains[SlotIndex(slot)].armFF)
    return m_simGains[SlotIndex(slot)].armFF;
  return m_slots[SlotIndex(slot)].armFF;
}
std::optional<wpi::math::ElevatorFeedforward> SmartMotorControllerConfig::GetElevatorFeedforward(
    ClosedLoopControllerSlot slot) const {
  m_basicOptions.erase(BasicOptions::SlotGains);
  if (wpi::RobotBase::IsSimulation() && m_simGains[SlotIndex(slot)].elevatorFF)
    return m_simGains[SlotIndex(slot)].elevatorFF;
  return m_slots[SlotIndex(slot)].elevatorFF;
}
std::optional<wpi::math::SimpleMotorFeedforward<wpi::units::turns>>
SmartMotorControllerConfig::GetSimpleFeedforward(ClosedLoopControllerSlot slot) const {
  m_basicOptions.erase(BasicOptions::SlotGains);
  if (wpi::RobotBase::IsSimulation() && m_simGains[SlotIndex(slot)].simpleFF)
    return m_simGains[SlotIndex(slot)].simpleFF;
  return m_slots[SlotIndex(slot)].simpleFF;
}
std::optional<math::LQRConfig> SmartMotorControllerConfig::GetLQR(
    ClosedLoopControllerSlot slot) const {
  m_basicOptions.erase(BasicOptions::SlotGains);
  return m_slots[SlotIndex(slot)].lqr;
}

double SmartMotorControllerConfig::GetKp(ClosedLoopControllerSlot slot) const {
  return GetSlotGains(slot).kP;
}
double SmartMotorControllerConfig::GetKi(ClosedLoopControllerSlot slot) const {
  return GetSlotGains(slot).kI;
}
double SmartMotorControllerConfig::GetKd(ClosedLoopControllerSlot slot) const {
  return GetSlotGains(slot).kD;
}

bool SmartMotorControllerConfig::GetLinearClosedLoopControllerUse() const {
  return m_mechanismCircumference.has_value();
}

std::optional<wpi::units::turn_t> SmartMotorControllerConfig::GetMechanismLowerLimit() const {
  m_basicOptions.erase(BasicOptions::LowerLimit);
  return m_mechLowerLimit;
}
std::optional<wpi::units::turn_t> SmartMotorControllerConfig::GetMechanismUpperLimit() const {
  m_basicOptions.erase(BasicOptions::UpperLimit);
  return m_mechUpperLimit;
}
std::optional<wpi::units::meter_t> SmartMotorControllerConfig::GetMeasurementLowerLimit() const {
  m_basicOptions.erase(BasicOptions::LowerLimit);
  return m_measLowerLimit;
}
std::optional<wpi::units::meter_t> SmartMotorControllerConfig::GetMeasurementUpperLimit() const {
  m_basicOptions.erase(BasicOptions::UpperLimit);
  return m_measUpperLimit;
}

std::optional<wpi::units::turn_t> SmartMotorControllerConfig::GetContinuousWrapping() const {
  if (m_continuousWrappingMax && m_continuousWrappingMin &&
      std::abs((m_continuousWrappingMax->value() - 1.0) - m_continuousWrappingMin->value()) > 1e-9)
    throw exceptions::SmartMotorControllerConfigurationException(
        "Bounds are not correct!", "Cannot get the continuous wrapping point.",
        "WithContinuousWrapping(min, max) where max - min == 1 rotation");
  m_basicOptions.erase(BasicOptions::ContinuousWrapping);
  return m_continuousWrappingMax;
}

std::optional<wpi::units::turn_t> SmartMotorControllerConfig::GetContinuousWrappingMin() const {
  if (m_continuousWrappingMax && m_continuousWrappingMin &&
      std::abs((m_continuousWrappingMax->value() - 1.0) - m_continuousWrappingMin->value()) > 1e-9)
    throw exceptions::SmartMotorControllerConfigurationException(
        "Bounds are not correct!", "Cannot get the continuous wrapping point.",
        "WithContinuousWrapping(min, max) where max - min == 1 rotation");
  return m_continuousWrappingMin;
}

std::optional<wpi::units::ampere_t> SmartMotorControllerConfig::GetStatorCurrentLimit() const {
  m_basicOptions.erase(BasicOptions::StatorCurrentLimit);
  return m_statorCurrentLimit;
}
std::optional<int> SmartMotorControllerConfig::GetStatorStallCurrentLimit() const {
  m_basicOptions.erase(BasicOptions::StatorCurrentLimit);
  if (!m_statorCurrentLimit) return std::nullopt;
  return static_cast<int>(m_statorCurrentLimit->value());
}
std::optional<int> SmartMotorControllerConfig::GetSupplyStallCurrentLimit() const {
  m_basicOptions.erase(BasicOptions::SupplyCurrentLimit);
  if (!m_supplyCurrentLimit) return std::nullopt;
  return static_cast<int>(m_supplyCurrentLimit->value());
}
std::optional<wpi::units::ampere_t> SmartMotorControllerConfig::GetSupplyCurrentLimit() const {
  m_basicOptions.erase(BasicOptions::SupplyCurrentLimit);
  return m_supplyCurrentLimit;
}
std::optional<wpi::units::celsius_t> SmartMotorControllerConfig::GetTemperatureCutoff() const {
  m_basicOptions.erase(BasicOptions::TemperatureCutoff);
  return m_temperatureCutoff;
}
std::optional<wpi::units::volt_t> SmartMotorControllerConfig::GetClosedLoopControllerMaximumVoltage()
    const {
  m_basicOptions.erase(BasicOptions::ClosedLoopMaxVoltage);
  return m_closedLoopMaxVoltage;
}

SmartMotorControllerConfig::ControlMode SmartMotorControllerConfig::GetMotorControllerMode() const {
  m_basicOptions.erase(BasicOptions::ControlMode);
  return m_controlMode;
}
SmartMotorControllerConfig::MotorMode SmartMotorControllerConfig::GetIdleMode() const {
  m_basicOptions.erase(BasicOptions::IdleMode);
  return m_idleMode;
}
std::optional<wpi::units::second_t> SmartMotorControllerConfig::GetClosedLoopControlPeriod() const {
  return m_closedLoopPeriod;
}
std::optional<wpi::units::second_t> SmartMotorControllerConfig::GetOpenLoopRampRate() const {
  m_basicOptions.erase(BasicOptions::OpenLoopRampRate);
  return m_openLoopRampRate;
}
std::optional<wpi::units::second_t> SmartMotorControllerConfig::GetClosedLoopRampRate() const {
  m_basicOptions.erase(BasicOptions::ClosedLoopRampRate);
  return m_closedLoopRampRate;
}

std::optional<bool> SmartMotorControllerConfig::GetMotorInverted() const {
  m_basicOptions.erase(BasicOptions::MotorInverted);
  return m_motorInverted;
}
std::optional<bool> SmartMotorControllerConfig::GetEncoderInverted() const {
  m_basicOptions.erase(BasicOptions::EncoderInverted);
  return m_encoderInverted;
}
bool SmartMotorControllerConfig::GetVelocityTrapezoidalProfileInUse() const {
  m_basicOptions.erase(BasicOptions::TrapezoidProfile);
  return m_velocityTrapProfile;
}

std::optional<std::string> SmartMotorControllerConfig::GetTelemetryName() const {
  return m_telemetryName;
}
std::optional<SmartMotorControllerConfig::TelemetryVerbosity>
SmartMotorControllerConfig::GetVerbosity() const {
  return m_verbosity;
}
wpi::cmd::SubsystemBase* SmartMotorControllerConfig::GetSubsystem() const { return m_subsystem; }
std::optional<wpi::math::DCMotor> SmartMotorControllerConfig::GetSimMotor() const { return m_simMotor; }
wpi::units::kilogram_square_meter_t SmartMotorControllerConfig::GetMOI() const { return m_moi; }
std::optional<wpi::units::turn_t> SmartMotorControllerConfig::GetStartingPosition() const {
  m_basicOptions.erase(BasicOptions::StartingPosition);
  if (wpi::RobotBase::IsSimulation() && m_simStartingPosition.has_value())
    return m_simStartingPosition;
  return m_startingPosition;
}
std::optional<std::any> SmartMotorControllerConfig::GetVendorConfig() const {
  return m_vendorConfig;
}

std::optional<std::any> SmartMotorControllerConfig::GetVendorControlRequest() const {
  m_basicOptions.erase(BasicOptions::VendorControlRequest);
  return m_vendorControlRequest;
}

const std::optional<gearing::MechanismGearing>& SmartMotorControllerConfig::GetMotorGearing()
    const {
  m_basicOptions.erase(BasicOptions::Gearing);
  return m_motorGearing;
}
std::optional<wpi::units::meter_t> SmartMotorControllerConfig::GetMechanismCircumference() const {
  return m_mechanismCircumference;
}

std::optional<std::any> SmartMotorControllerConfig::GetExternalEncoder() const {
  m_basicOptions.erase(BasicOptions::ExternalEncoder);
  return m_externalEncoder;
}
bool SmartMotorControllerConfig::GetUseExternalFeedback() const {
  m_externalEncoderOptions.erase(ExternalEncoderOptions::UseExternalFeedback);
  return m_useExternalFeedback;
}
std::optional<bool> SmartMotorControllerConfig::GetExternalEncoderInverted() const {
  m_externalEncoderOptions.erase(ExternalEncoderOptions::ExternalEncoderInverted);
  return m_externalEncoderInverted;
}
std::optional<double> SmartMotorControllerConfig::GetExternalEncoderConversionFactor() const {
  return m_externalEncoderConversionFactor;
}
std::optional<wpi::units::turn_t> SmartMotorControllerConfig::GetExternalEncoderZeroOffset() const {
  m_externalEncoderOptions.erase(ExternalEncoderOptions::ZeroOffset);
  return m_externalEncoderZeroOffset;
}
const std::optional<gearing::MechanismGearing>&
SmartMotorControllerConfig::GetExternalEncoderGearing() const {
  m_externalEncoderOptions.erase(ExternalEncoderOptions::ExternalGearing);
  return m_externalEncoderGearing;
}
std::optional<wpi::units::turn_t> SmartMotorControllerConfig::GetExternalEncoderDiscontinuityPoint()
    const {
  m_externalEncoderOptions.erase(ExternalEncoderOptions::DiscontinuityPoint);
  return m_externalEncoderDiscontinuityPoint;
}

bool SmartMotorControllerConfig::HasTrapezoidProfile() const {
  m_basicOptions.erase(BasicOptions::TrapezoidProfile);
  return m_trapProfile.has_value() || m_linearTrapProfile.has_value();
}
bool SmartMotorControllerConfig::HasExponentialProfile() const {
  m_basicOptions.erase(BasicOptions::ExponentialProfile);
  return m_expoProfile.has_value();
}
bool SmartMotorControllerConfig::HasLinearExponentialProfile() const {
  m_basicOptions.erase(BasicOptions::ExponentialProfile);
  return m_linearExpoProfile.has_value();
}

std::optional<wpi::math::TrapezoidProfile<wpi::units::turns>> SmartMotorControllerConfig::GetTrapezoidProfile()
    const {
  m_basicOptions.erase(BasicOptions::TrapezoidProfile);
  if (wpi::RobotBase::IsSimulation() && m_simTrapProfile.has_value()) return m_simTrapProfile;
  return m_trapProfile;
}
std::optional<wpi::math::TrapezoidProfile<wpi::units::meters>>
SmartMotorControllerConfig::GetLinearTrapezoidProfile() const {
  m_basicOptions.erase(BasicOptions::TrapezoidProfile);
  if (wpi::RobotBase::IsSimulation() && m_simLinearTrapProfile.has_value())
    return m_simLinearTrapProfile;
  return m_linearTrapProfile;
}
std::optional<wpi::math::ExponentialProfile<wpi::units::turns, wpi::units::volts>>
SmartMotorControllerConfig::GetExponentialProfile() const {
  m_basicOptions.erase(BasicOptions::ExponentialProfile);
  if (wpi::RobotBase::IsSimulation() && m_simExpoProfile.has_value()) return m_simExpoProfile;
  return m_expoProfile;
}
std::optional<wpi::math::ExponentialProfile<wpi::units::meters, wpi::units::volts>>
SmartMotorControllerConfig::GetLinearExponentialProfile() const {
  m_basicOptions.erase(BasicOptions::ExponentialProfile);
  return m_linearExpoProfile;
}

std::optional<wpi::units::turns_per_second_t> SmartMotorControllerConfig::GetTrapMaxVelocityTurns()
    const {
  m_basicOptions.erase(BasicOptions::TrapezoidProfile);
  return m_trapMaxVelTurns;
}
std::optional<wpi::units::turns_per_second_squared_t> SmartMotorControllerConfig::GetTrapMaxAccelTurns()
    const {
  m_basicOptions.erase(BasicOptions::TrapezoidProfile);
  return m_trapMaxAccTurns;
}
std::optional<wpi::units::meters_per_second_t> SmartMotorControllerConfig::GetTrapMaxVelocityLinear()
    const {
  m_basicOptions.erase(BasicOptions::TrapezoidProfile);
  return m_trapMaxVelLinear;
}
std::optional<wpi::units::meters_per_second_squared_t>
SmartMotorControllerConfig::GetTrapMaxAccelLinear() const {
  m_basicOptions.erase(BasicOptions::TrapezoidProfile);
  return m_trapMaxAccLinear;
}

std::optional<double> SmartMotorControllerConfig::GetExponentialProfileKV() const {
  return m_expoMotionMagicKV;
}

std::optional<double> SmartMotorControllerConfig::GetExponentialProfileKA() const {
  return m_expoMotionMagicKA;
}

std::optional<wpi::units::volt_t> SmartMotorControllerConfig::GetExponentialProfileMaxInput() const {
  return m_expoMaxInput;
}

wpi::units::meter_t SmartMotorControllerConfig::ConvertFromMechanism(
    wpi::units::turn_t mechanismPosition) const {
  double circ = m_mechanismCircumference.value_or(wpi::units::meter_t{1.0}).value();
  return wpi::units::meter_t{mechanismPosition.value() * circ};
}

wpi::units::meters_per_second_t SmartMotorControllerConfig::ConvertFromMechanism(
    wpi::units::turns_per_second_t mechanismVelocity) const {
  double circ = m_mechanismCircumference.value_or(wpi::units::meter_t{1.0}).value();
  return wpi::units::meters_per_second_t{mechanismVelocity.value() * circ};
}

// ---- Followers ----------------------------------------------------------------

SmartMotorControllerConfig& SmartMotorControllerConfig::WithFollowers(
    std::vector<std::pair<std::any, bool>> followers) {
  m_followers = std::move(followers);
  return *this;
}

SmartMotorControllerConfig& SmartMotorControllerConfig::WithLooselyCoupledFollowers(
    std::vector<SmartMotorController*> followers) {
  m_looseFollowers = std::move(followers);
  return *this;
}

const std::vector<std::pair<std::any, bool>>& SmartMotorControllerConfig::GetFollowers() const {
  return m_followers;
}

const std::vector<SmartMotorController*>& SmartMotorControllerConfig::GetLooselyCoupledFollowers()
    const {
  return m_looseFollowers;
}

// ---- Clone --------------------------------------------------------------------

SmartMotorControllerConfig SmartMotorControllerConfig::Clone() const { return *this; }

}  // namespace yams::motorcontrollers
