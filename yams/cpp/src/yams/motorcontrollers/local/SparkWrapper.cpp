// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/local/SparkWrapper.hpp"

#include <rev/ClosedLoopTypes.h>
#include <rev/ConfigureTypes.h>
#include <rev/SplineEncoder.h>
#include <rev/config/DetachedEncoderConfig.h>
#include <rev/config/MAXMotionConfig.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <wpi/driverstation/RobotState.hpp>
#include <wpi/framework/RobotBase.hpp>
#include <wpi/math/system/Models.hpp>
#include <wpi/math/util/MathUtil.hpp>
#include <wpi/units/moment_of_inertia.hpp>
#include <wpi/util/Alert.hpp>

#include "yams/exceptions.hpp"
#include "yams/math/LQRController.hpp"
#include "yams/motorcontrollers/simulation/BatterySim.hpp"
#include "yams/motorcontrollers/simulation/DCMotorSimSupplier.hpp"

using namespace rev::spark;

namespace yams::motorcontrollers::local {

namespace {

using Slot = SmartMotorControllerConfig::ClosedLoopControllerSlot;
using TurnKv = wpi::math::SimpleMotorFeedforward<wpi::units::turns>::kv_unit;
using TurnKa = wpi::math::SimpleMotorFeedforward<wpi::units::turns>::ka_unit;

constexpr Slot kAllSlots[] = {Slot::SLOT_0, Slot::SLOT_1, Slot::SLOT_2, Slot::SLOT_3};
constexpr ClosedLoopSlot kAllSparkSlots[] = {ClosedLoopSlot::kSlot0, ClosedLoopSlot::kSlot1,
                                              ClosedLoopSlot::kSlot2, ClosedLoopSlot::kSlot3};

/** The SPARK closes its loop every millisecond; its integral and derivative gains are per loop. */
constexpr double kSparkLoopPeriodSeconds = 0.001;

ClosedLoopSlot ToSparkSlot(Slot slot) {
  switch (slot) {
    case Slot::SLOT_0:
      return ClosedLoopSlot::kSlot0;
    case Slot::SLOT_1:
      return ClosedLoopSlot::kSlot1;
    case Slot::SLOT_2:
      return ClosedLoopSlot::kSlot2;
    case Slot::SLOT_3:
      return ClosedLoopSlot::kSlot3;
  }
  throw std::invalid_argument("SparkWrapper: invalid closed loop slot");
}

/** Feedforward gains of a slot, in YAMS units (per mechanism rotation, or per meter if linear). */
struct SlotFeedforward {
  double kS{0.0}, kV{0.0}, kA{0.0}, kG{0.0};
  bool arm{false};
  bool elevator{false};
};

std::optional<SlotFeedforward> GetSlotFeedforward(const SmartMotorControllerConfig& config,
                                                  Slot slot) {
  if (auto ff = config.GetArmFeedforward(slot)) {
    // ArmFeedforward gains are per radian; the SPARK's are per rotation of its sensor.
    return SlotFeedforward{ff->GetKs().value(), wpi::units::unit_t<TurnKv>{ff->GetKv()}.value(),
                           wpi::units::unit_t<TurnKa>{ff->GetKa()}.value(), ff->GetKg().value(),
                           true, false};
  }
  if (auto ff = config.GetElevatorFeedforward(slot)) {
    return SlotFeedforward{ff->GetKs().value(), ff->GetKv().value(), ff->GetKa().value(),
                           ff->GetKg().value(), false, true};
  }
  if (auto ff = config.GetSimpleFeedforward(slot)) {
    return SlotFeedforward{ff->GetKs().value(), ff->GetKv().value(), ff->GetKa().value(), 0.0,
                           false, false};
  }
  return std::nullopt;
}

/**
 * Gains of the configured feedforward in its own units (ArmFeedforward kV/kA per radian), as used
 * by the live-tuning setters.
 */
struct NativeFeedforward {
  double kS{0.0}, kV{0.0}, kA{0.0}, kG{0.0};
};

/**
 * Apply @p update to the slot's configured feedforward(s) in their native units and store them
 * back in the config.
 *
 * @return true if a feedforward is configured for the slot.
 */
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
    return wpi::units::unit_t<TurnKv>{
        wpi::units::unit_t<wpi::math::ArmFeedforward::kv_unit>{kV}}
        .value();
  }
  return kV;
}

/** Convert an ArmFeedforward kA (per radian) to per rotation; other feedforwards unchanged. */
double KaPerRotation(const SmartMotorControllerConfig& config, Slot slot, double kA) {
  if (config.GetArmFeedforward(slot)) {
    return wpi::units::unit_t<TurnKa>{
        wpi::units::unit_t<wpi::math::ArmFeedforward::ka_unit>{kA}}
        .value();
  }
  return kA;
}

}  // namespace

// ---- Construction -----------------------------------------------------------

SparkWrapper::SparkWrapper(SparkMax* spark, wpi::math::DCMotor motor,
                           SmartMotorControllerConfig* cfg)
    : SmartMotorController(), m_motor(motor) {
  m_maxConfig.emplace();
  if (auto vc = cfg->GetVendorConfig(); vc.has_value()) {
    if (auto* p = std::any_cast<rev::spark::SparkMaxConfig>(&vc.value())) {
      m_maxConfig->Apply(*p);
    } else {
      throw exceptions::SmartMotorControllerConfigurationException(
          "SparkMaxConfig is the only acceptable vendor config for SparkMax controllers.",
          "SparkMaxConfig not found.", "WithVendorConfig(rev::spark::SparkMaxConfig{})");
    }
  }
  Init(spark, motor, cfg);
}

SparkWrapper::SparkWrapper(SparkFlex* spark, wpi::math::DCMotor motor,
                           SmartMotorControllerConfig* cfg)
    : SmartMotorController(), m_motor(motor) {
  m_flexConfig.emplace();
  if (auto vc = cfg->GetVendorConfig(); vc.has_value()) {
    if (auto* p = std::any_cast<rev::spark::SparkFlexConfig>(&vc.value())) {
      m_flexConfig->Apply(*p);
    } else {
      throw exceptions::SmartMotorControllerConfigurationException(
          "SparkFlexConfig is the only acceptable vendor config for SparkFlex controllers.",
          "SparkFlexConfig not found.", "WithVendorConfig(rev::spark::SparkFlexConfig{})");
    }
  }
  Init(spark, motor, cfg);
}

void SparkWrapper::Init(SparkBase* spark, wpi::math::DCMotor motor,
                        SmartMotorControllerConfig* cfg) {
  m_spark = spark;
  m_sparkPid = &spark->GetClosedLoopController();
  m_relEncoder = &spark->GetEncoder();
  m_config = cfg;
  m_config->WithSimMotor(motor);

  m_externalEncoderGearingDiscontinuityAlert.emplace(
      "YAMS", AlertId("ExternalEncoderGearingDiscontinuity"),
      GetName() +
          " external encoder gearing set while ExternalEncoderDiscontinuityPoint is also set; "
          "the discontinuity point will NOT be moved by the gearing, wrapping will occur "
          "non-uniformly",
      wpi::util::Alert::Level::HIGH);

  UpdateFeedbackSensorRatio(*m_config);
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

SparkWrapper::~SparkWrapper() {
  Close();
  if (m_rioControllerAlert) m_rioControllerAlert->Set(false);
  if (m_externalEncoderGearingDiscontinuityAlert)
    m_externalEncoderGearingDiscontinuityAlert->Set(false);
}

// ---- SPARK configuration helpers ---------------------------------------------

SparkBaseConfig& SparkWrapper::SparkConfig() {
  if (m_maxConfig) return *m_maxConfig;
  return *m_flexConfig;
}

bool SparkWrapper::ConfigureSpark(const std::function<rev::REVLibError()>& call) {
  for (int i = 0; i < 8; i++) {
    if (call() == rev::REVLibError::kOk) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

rev::PersistMode SparkWrapper::PersistModeForState() const {
  return wpi::RobotState::IsEnabled() ? rev::PersistMode::kNoPersistParameters
                                         : rev::PersistMode::kPersistParameters;
}

void SparkWrapper::ConfigureAsync() {
  m_spark->ConfigureAsync(SparkConfig(), rev::ResetMode::kNoResetSafeParameters,
                          PersistModeForState());
}

std::string SparkWrapper::AlertId(const std::string& alertType) const {
  const std::string prefix = "Spark(" + std::to_string(m_spark->GetDeviceId()) + ")";
  if (auto name = m_config->GetTelemetryName()) return prefix + "_" + alertType + "_" + *name;
  return prefix + "_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)) + "_" + alertType;
}

double SparkWrapper::MechanismToExternalEncoderRatio() const {
  // A direct conversion factor (encoder rotations -> mechanism rotations) overrides the gearing.
  if (auto cf = m_config->GetExternalEncoderConversionFactor(); cf && *cf != 0.0) return 1.0 / *cf;
  return m_config->GetExternalEncoderGearing()
      .value_or(gearing::MechanismGearing::kOne)
      .GetMechanismToRotorRatio();
}

void SparkWrapper::UpdateFeedbackSensorRatio(const SmartMotorControllerConfig& config) {
  m_mechanismToFeedbackSensorRatio =
      config.GetUseExternalFeedback() && config.GetExternalEncoder().has_value()
          ? MechanismToExternalEncoderRatio()
          : config.GetMotorGearing()
                .value_or(gearing::MechanismGearing::kOne)
                .GetMechanismToRotorRatio();
}

void SparkWrapper::WriteSparkPID(double kP, double kI, double kD, ClosedLoopSlot slot) {
  // YAMS gains are in volts per mechanism rotation (per rotation/s for velocity control); the
  // SPARK's are in duty cycle per rotation (per RPM) of its feedback sensor.
  const double feedbackUnitsPerMechanismUnit = m_velocityGainsApplied
                                                   ? m_mechanismToFeedbackSensorRatio * 60.0
                                                   : m_mechanismToFeedbackSensorRatio;
  // Gains of a linear mechanism are per meter; the SPARK's are per rotation.
  double mechanismRotationsPerGainUnit = 1.0;
  if (m_config->GetLinearClosedLoopControllerUse()) {
    mechanismRotationsPerGainUnit =
        m_velocityGainsApplied
            ? m_config->ConvertToMechanism(wpi::units::meters_per_second_t{1.0}).value()
            : m_config->ConvertToMechanism(wpi::units::meter_t{1.0}).value();
  }
  const double scale = m_config->GetVoltageCompensation().value_or(12_V).value() *
                       feedbackUnitsPerMechanismUnit * mechanismRotationsPerGainUnit;
  // A real SPARK's integral and derivative gains are per 1 ms loop. REVLib's simulated SPARK
  // applies them per second of error instead, so in simulation they are not scaled.
  const double loopPeriodSeconds =
      wpi::RobotBase::IsSimulation() ? 1.0 : kSparkLoopPeriodSeconds;
  SparkConfig().closedLoop.Pid(kP / scale, kI * loopPeriodSeconds / scale,
                               kD / loopPeriodSeconds / scale, slot);
}

void SparkWrapper::WriteSparkFeedforward(double kS, double kV, double kA, ClosedLoopSlot slot) {
  // YAMS gains are volts per mechanism rotation/s (/s); the SPARK's are volts per RPM (/s) of its
  // feedback sensor.
  const double feedbackRPMPerMechanismRPS = m_mechanismToFeedbackSensorRatio * 60.0;
  const bool linear = m_config->GetLinearClosedLoopControllerUse();
  const double velocityPerGainUnit =
      linear ? m_config->ConvertToMechanism(wpi::units::meters_per_second_t{1.0}).value() : 1.0;
  const double accelerationPerGainUnit =
      linear ? m_config->ConvertToMechanism(wpi::units::meters_per_second_squared_t{1.0}).value()
             : 1.0;
  SparkConfig()
      .closedLoop.feedForward.kS(kS, slot)
      .kV(kV / feedbackRPMPerMechanismRPS / velocityPerGainUnit, slot)
      .kA(kA / feedbackRPMPerMechanismRPS / accelerationPerGainUnit, slot);
}

void SparkWrapper::WriteSlotGains(const SmartMotorControllerConfig& config, Slot slot) {
  const ClosedLoopSlot sparkSlot = ToSparkSlot(slot);
  const auto gains = config.GetSlotGains(slot);
  if (gains.kP != 0.0 || gains.kI != 0.0 || gains.kD != 0.0) {
    WriteSparkPID(gains.kP, gains.kI, gains.kD, sparkSlot);
  }
  if (auto ff = GetSlotFeedforward(config, slot)) {
    WriteSparkFeedforward(ff->kS, ff->kV, ff->kA, sparkSlot);
    if (ff->arm) {
      // kCos takes the angle of the mechanism, in rotations, from the feedback sensor position.
      SparkConfig()
          .closedLoop.feedForward.kCos(ff->kG, sparkSlot)
          .kCosRatio(1.0 / m_mechanismToFeedbackSensorRatio, sparkSlot);
    } else if (ff->elevator) {
      SparkConfig().closedLoop.feedForward.kG(ff->kG, sparkSlot);
    }
  }
}

void SparkWrapper::WriteMotionProfile(const SmartMotorControllerConfig& config) {
  if (!config.HasTrapezoidProfile()) return;
  const bool velocityProfile = config.GetVelocityTrapezoidalProfileInUse();
  // First and second profile constraints in mechanism units: velocity and acceleration for a
  // position profile; acceleration and jerk for a velocity profile.
  double first = 0.0;
  double second = 0.0;
  if (auto linVel = config.GetTrapMaxVelocityLinear(); linVel && !config.GetTrapMaxVelocityTurns()) {
    first = config.ConvertToMechanism(*linVel).value();
    second = config.ConvertToMechanism(*config.GetTrapMaxAccelLinear()).value();
  } else if (auto vel = config.GetTrapMaxVelocityTurns()) {
    first = vel->value();
    second = config.GetTrapMaxAccelTurns()->value();
  } else {
    return;
  }
  auto& maxMotion = SparkConfig().closedLoop.maxMotion;
  for (auto sparkSlot : kAllSparkSlots) {
    if (velocityProfile) {
      // MAXMotion velocity control limits the acceleration; it has no jerk limit.
      maxMotion.MaxAcceleration(first * m_mechanismToFeedbackSensorRatio * 60.0, sparkSlot);
    } else {
      maxMotion.PositionMode(MAXMotionConfig::kMAXMotionTrapezoidal, sparkSlot)
          .CruiseVelocity(first * m_mechanismToFeedbackSensorRatio * 60.0, sparkSlot)
          .MaxAcceleration(second * m_mechanismToFeedbackSensorRatio * 60.0, sparkSlot);
    }
  }
}

void SparkWrapper::WriteClosedLoopTolerance(const SmartMotorControllerConfig& config) {
  auto tolerance = config.GetClosedLoopTolerance();
  if (!tolerance) return;
  const double feedbackRotations = tolerance->value() * m_mechanismToFeedbackSensorRatio;
  for (auto sparkSlot : kAllSparkSlots) {
    SparkConfig().closedLoop.AllowedClosedLoopError(feedbackRotations, sparkSlot);
    SparkConfig().closedLoop.maxMotion.AllowedProfileError(feedbackRotations, sparkSlot);
  }
}

void SparkWrapper::WriteSoftLimits(const SmartMotorControllerConfig& config) {
  const bool isClosedLoop = config.GetMotorControllerMode() == ControlMode::CLOSED_LOOP;
  if (auto lower = config.GetMechanismLowerLimit()) {
    SparkConfig()
        .softLimit.ReverseSoftLimit(lower->value() * m_mechanismToFeedbackSensorRatio)
        .ReverseSoftLimitEnabled(isClosedLoop);
  }
  if (auto upper = config.GetMechanismUpperLimit()) {
    SparkConfig()
        .softLimit.ForwardSoftLimit(upper->value() * m_mechanismToFeedbackSensorRatio)
        .ForwardSoftLimitEnabled(isClosedLoop);
  }
}

void SparkWrapper::RewriteScaledParameters() {
  UpdateFeedbackSensorRatio(*m_config);
  for (auto slot : kAllSlots) WriteSlotGains(*m_config, slot);
  WriteMotionProfile(*m_config);
  WriteClosedLoopTolerance(*m_config);
  if (auto lower = m_config->GetMechanismLowerLimit())
    SparkConfig().softLimit.ReverseSoftLimit(lower->value() * m_mechanismToFeedbackSensorRatio);
  if (auto upper = m_config->GetMechanismUpperLimit())
    SparkConfig().softLimit.ForwardSoftLimit(upper->value() * m_mechanismToFeedbackSensorRatio);
  ConfigureAsync();
}

void SparkWrapper::UseVelocityGains(bool velocity) {
  if (m_velocityGainsApplied == velocity) return;
  m_velocityGainsApplied = velocity;
  for (auto slot : kAllSlots) {
    const auto gains = m_config->GetSlotGains(slot);
    if (gains.kP != 0.0 || gains.kI != 0.0 || gains.kD != 0.0)
      WriteSparkPID(gains.kP, gains.kI, gains.kD, ToSparkSlot(slot));
  }
  ConfigureSpark([this] {
    return m_spark->Configure(SparkConfig(), rev::ResetMode::kNoResetSafeParameters,
                              rev::PersistMode::kNoPersistParameters);
  });
}

bool SparkWrapper::HasExponentialProfile() const {
  return m_config->HasExponentialProfile() || m_config->HasLinearExponentialProfile();
}

bool SparkWrapper::SimulatingMaxMotion() const {
  return wpi::RobotBase::IsSimulation() && m_config->HasTrapezoidProfile() &&
         !m_config->GetVelocityTrapezoidalProfileInUse();
}

// ---- Configuration ----------------------------------------------------------

bool SparkWrapper::ApplyConfig(const SmartMotorControllerConfig& config) {
  // Like the Java wrapper, the applied config becomes this controller's config.
  m_config = const_cast<SmartMotorControllerConfig*>(&config);
  config.ResetValidationCheck();
  if (m_rioControllerAlert) m_rioControllerAlert->Set(false);
  m_externalEncoderGearingDiscontinuityAlert->Set(false);

  if (config.GetVendorControlRequest().has_value())
    throw exceptions::SmartMotorControllerConfigurationException(
        "Spark(" + std::to_string(m_spark->GetDeviceId()) +
            ") does not support the custom control requests!",
        "Cannot use given control request", "WithVendorControlRequest()");

  const bool useExternalEncoder = config.GetUseExternalFeedback();
  const double mechToRotorRatio =
      config.GetMotorGearing().value_or(gearing::MechanismGearing::kOne).GetMechanismToRotorRatio();
  UpdateFeedbackSensorRatio(config);
  auto& sparkCfg = SparkConfig();

  for (int i = 1; i <= 4; i++) {
    if (IsMotor(m_motor, wpi::math::DCMotor::Minion(i))) sparkCfg.AdvanceCommutation(120);
  }
  if (m_spark->IsFollower().Get()) {
    m_spark->PauseFollowerMode();
    sparkCfg.DisableFollowerMode();
  }

  // Load LQR and software PID from the active gain slot for IterateClosedLoopController.
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
  if (m_lqr && config.GetClosedLoopTolerance())
    throw std::invalid_argument("[Error] Closed loop tolerance is not supported in LQR mode.");
  ConfigureSoftwarePID(config);

  // Motion profile: MAXMotion on the SPARK, in feedback sensor RPM.
  const bool hasExpo = config.HasExponentialProfile() || config.HasLinearExponentialProfile();
  const bool hasTrap = config.HasTrapezoidProfile();
  m_positionControlType = SparkLowLevel::ControlType::kPosition;
  m_velocityControlType = SparkLowLevel::ControlType::kVelocity;
  m_simMaxMotionState.reset();
  if (hasTrap) {
    WriteMotionProfile(config);
    m_positionControlType = SparkLowLevel::ControlType::kMAXMotionPositionControl;
    m_velocityControlType = SparkLowLevel::ControlType::kMAXMotionVelocityControl;
    if (wpi::RobotBase::IsSimulation() && !config.GetVelocityTrapezoidalProfileInUse() &&
        !hasExpo && !m_lqr) {
      // REVLib's MAXMotion simulation does not follow the profile (REVLib 2027.0.0-alpha-7). In
      // simulation, SimIterate() runs the profile and the SPARK holds each point of it with
      // position control instead.
      m_positionControlType = SparkLowLevel::ControlType::kPosition;
    }
  }

  // SPARKs have no exponential profile or LQR: the RoboRIO runs the closed loop and sends
  // voltages to the SPARK.
  if (hasExpo || m_lqr) {
    if (!m_rioControllerAlert) {
      m_rioControllerAlert.emplace("YAMS", AlertId("ClosedLoop"),
                                   GetName() + " closed loop controller is running on the SystemCore.",
                                   wpi::util::Alert::Level::MEDIUM);
    }
    m_rioControllerAlert->Set(true);

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
    } else if (config.GetClosedLoopControlPeriod()) {
      throw std::invalid_argument(
          "[Error] Closed loop control period is only supported in closed loop mode.");
    }
  } else if (m_closedLoopControllerThread) {
    // Config no longer requires a software controller; tear it down.
    StopClosedLoopController();
    m_closedLoopControllerThread.reset();
  }

  // Base options.
  if (auto r = config.GetOpenLoopRampRate(); r) sparkCfg.OpenLoopRampRate(r->value());
  if (auto r = config.GetClosedLoopRampRate(); r) sparkCfg.ClosedLoopRampRate(r->value());
  if (auto inv = config.GetMotorInverted(); inv) sparkCfg.Inverted(*inv);

  // Gains, in the SPARK's units.
  sparkCfg.closedLoop.SetFeedbackSensor(FeedbackSensor::kPrimaryEncoder);
  for (auto slot : kAllSlots) WriteSlotGains(config, slot);

  WriteClosedLoopTolerance(config);
  WriteSoftLimits(config);

  if (config.GetSupplyCurrentLimit())
    throw exceptions::SmartMotorControllerConfigurationException(
        "Supply current limits are not supported on Sparks", "Supply current limit not set",
        "WithStatorCurrentLimit");
  if (auto stator = config.GetStatorStallCurrentLimit(); stator) sparkCfg.SmartCurrentLimit(*stator);
  if (auto vc = config.GetVoltageCompensation(); vc) sparkCfg.VoltageCompensation(vc->value());
  if (auto zp = config.GetZeroPower(); zp)
    sparkCfg.SetIdleMode(*zp == MotorMode::BRAKE ? SparkBaseConfig::IdleMode::kBrake
                                                 : SparkBaseConfig::IdleMode::kCoast);
  if (auto start = config.GetStartingPosition(); start)
    m_relEncoder->SetPosition(start->value() * mechToRotorRatio);

  // Position wrapping. REVLib's wrapping has no input range: the SPARK wraps every rotation of its
  // feedback sensor, which is one mechanism rotation only for an absolute encoder on the
  // mechanism. Otherwise SetPosition() sends the equivalent setpoint nearest the current position.
  bool absoluteFeedback = false;
  if (useExternalEncoder) {
    if (auto enc = config.GetExternalEncoder(); enc) {
      absoluteFeedback = std::any_cast<SparkAbsoluteEncoder*>(&*enc) ||
                         std::any_cast<rev::detached::DetachedEncoder*>(&*enc) ||
                         std::any_cast<rev::detached::SplineEncoder*>(&*enc);
    }
  }
  sparkCfg.closedLoop.PositionWrappingEnabled(config.GetContinuousWrapping().has_value() &&
                                              absoluteFeedback);

  ApplyExternalEncoder(config, useExternalEncoder);

  // Tightly coupled followers accept SparkMax and SparkFlex only.
  if (!config.GetFollowers().empty()) {
    for (auto& [hw, inverted] : config.GetFollowers()) {
      auto configureFollower = [&](SparkBase* follower, SparkBaseConfig& followCfg) {
        followCfg.Follow(*m_spark, inverted);
        if (auto zp = config.GetZeroPower(); zp)
          followCfg.SetIdleMode(*zp == MotorMode::BRAKE ? SparkBaseConfig::IdleMode::kBrake
                                                        : SparkBaseConfig::IdleMode::kCoast);
        follower->Configure(followCfg, rev::ResetMode::kNoResetSafeParameters,
                            PersistModeForState());
      };
      if (auto* mx = std::any_cast<rev::spark::SparkMax*>(&hw)) {
        SparkMaxConfig followCfg;
        configureFollower(*mx, followCfg);
      } else if (auto* fx = std::any_cast<rev::spark::SparkFlex*>(&hw)) {
        SparkFlexConfig followCfg;
        configureFollower(*fx, followCfg);
      } else {
        throw std::invalid_argument(
            "[ERROR] Unknown follower type: SparkWrapper followers must be rev::spark::SparkMax* "
            "or rev::spark::SparkFlex*");
      }
    }
    // The followers keep following; applying the config again must not reconfigure them.
    m_config->ClearFollowers();
  }
  LoadLooselyCoupledFollowers();

  if (config.GetClosedLoopControlPeriod() && !hasExpo && !m_lqr)
    throw exceptions::SmartMotorControllerConfigurationException(
        "Closed loop control period is unsupported without Exponential Profiles",
        "Closed loop control period does not take affect", "WithClosedLoopControlPeriod");
  if (config.GetClosedLoopControllerMaximumVoltage() && !hasExpo && !m_lqr)
    throw exceptions::SmartMotorControllerConfigurationException(
        "Closed loop controller maximum voltage is only available for Exponential Profiled "
        "closed loop controllers",
        "Closed loop controller maximum voltage could not be applied",
        "WithClosedLoopMaxVoltage");
  if (config.GetTemperatureCutoff() && !hasExpo && !hasTrap)
    throw exceptions::SmartMotorControllerConfigurationException(
        "Temperature cutoff is only available for exponentially profiled closed loop "
        "controllers",
        "Temperature cutoff could not be applied", "WithTemperatureCutoff");
  if (config.GetFeedbackSynchronizationThreshold() && !hasExpo && !m_lqr)
    throw exceptions::SmartMotorControllerConfigurationException(
        "Feedback synchronization threshold is only available for exponentially profiled closed "
        "loop controllers",
        "Feedback synchronization threshold could not be applied",
        "WithFeedbackSynchronizationThreshold");
  if (config.GetEncoderInverted())
    throw std::invalid_argument("[ERROR] Spark relative encoder cannot be inverted!");

  const auto resetMode = config.GetResetPreviousConfig() ? rev::ResetMode::kResetSafeParameters
                                                         : rev::ResetMode::kNoResetSafeParameters;
  config.ValidateBasicOptions();
  config.ValidateExternalEncoderOptions();
  return ConfigureSpark(
      [&] { return m_spark->Configure(sparkCfg, resetMode, PersistModeForState()); });
}

void SparkWrapper::ApplyExternalEncoder(const SmartMotorControllerConfig& config,
                                        bool useExternalEncoder) {
  m_absEncoder = nullptr;
  m_quadratureEncoder = nullptr;
  m_detachedEncoder = nullptr;
  m_absEncoderSim.reset();
  m_altEncoderSim.reset();
  m_extEncoderSim.reset();
  auto& sparkCfg = SparkConfig();
  const bool sim = wpi::RobotBase::IsSimulation();

  auto enc = config.GetExternalEncoder();
  if (!enc) {
    if (config.GetExternalEncoderDiscontinuityPoint().has_value())
      throw exceptions::SmartMotorControllerConfigurationException(
          "External encoder discontinuity point is only available for external encoders",
          "Discontinuity point could not be applied", "WithExternalEncoder(encoder)");
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

  const double mechToEncoder = MechanismToExternalEncoderRatio();
  const auto startingPosition = config.GetStartingPosition();
  const auto zeroOffset = config.GetExternalEncoderZeroOffset();
  const auto discontinuityPoint = config.GetExternalEncoderDiscontinuityPoint();
  const auto inverted = config.GetExternalEncoderInverted();
  const bool hasExternalGearing = config.GetExternalEncoderGearing().has_value();
  m_absoluteEncoderDiscontinuityPoint = discontinuityPoint.value_or(wpi::units::turn_t{1.0});
  // Zero offsets are in [0, 1) rotations of the encoder.
  const double zeroOffsetRotations =
      zeroOffset ? wpi::math::InputModulus(zeroOffset->value() * mechToEncoder, 0.0, 1.0) : 0.0;

  rev::detached::DetachedEncoder* detached = nullptr;
  if (auto* p = std::any_cast<rev::detached::DetachedEncoder*>(&*enc)) detached = *p;
  if (auto* p = std::any_cast<rev::detached::SplineEncoder*>(&*enc)) detached = *p;

  if (auto* absPtr = std::any_cast<SparkAbsoluteEncoder*>(&*enc); absPtr && *absPtr) {
    m_absEncoder = *absPtr;
    // A SPARK MAX's data port is shared with the alternate encoder; select the absolute encoder.
    if (m_maxConfig) m_maxConfig->absoluteEncoder.SetSparkMaxDataPortConfig();
    if (inverted) sparkCfg.absoluteEncoder.Inverted(*inverted);
    if (useExternalEncoder) sparkCfg.closedLoop.SetFeedbackSensor(FeedbackSensor::kAbsoluteEncoder);
    m_absEncoderZeroOffset = zeroOffsetRotations;
    if (zeroOffset) sparkCfg.absoluteEncoder.ZeroOffset(zeroOffsetRotations);
    if (discontinuityPoint) {
      if (hasExternalGearing) m_externalEncoderGearingDiscontinuityAlert->Set(true);
      // REVLib's range offset is the middle of the range the encoder reports: 0 for
      // [-0.5, 0.5) rotations, 0.5 for [0, 1).
      sparkCfg.absoluteEncoder.RangeOffset(discontinuityPoint->value() - 0.5);
    }
    if (sim) {
      if (m_maxConfig)
        m_absEncoderSim.emplace(static_cast<SparkMax*>(m_spark));
      else
        m_absEncoderSim.emplace(static_cast<SparkFlex*>(m_spark));
      if (startingPosition) m_absEncoderSim->SetPosition(startingPosition->value() * mechToEncoder);
      if (zeroOffset) m_absEncoderSim->SetZeroOffset(zeroOffsetRotations);
    }
  } else if (std::any_cast<SparkMaxAlternateEncoder*>(&*enc) ||
             std::any_cast<SparkFlexExternalEncoder*>(&*enc)) {
    // A quadrature encoder on the SPARK; its counts per revolution come from the vendor config.
    if (auto* p = std::any_cast<SparkMaxAlternateEncoder*>(&*enc)) m_quadratureEncoder = *p;
    if (auto* p = std::any_cast<SparkFlexExternalEncoder*>(&*enc)) m_quadratureEncoder = *p;
    if (!m_quadratureEncoder)
      throw std::invalid_argument("SparkWrapper: external quadrature encoder pointer is null");
    if (zeroOffset)
      throw exceptions::SmartMotorControllerConfigurationException(
          "Zero offset is only available for absolute encoders",
          "Zero offset could not be applied to the quadrature encoder",
          "WithExternalEncoderZeroOffset");
    if (discontinuityPoint)
      throw exceptions::SmartMotorControllerConfigurationException(
          "Discontinuity point is only available for absolute encoders",
          "Discontinuity point could not be applied to the quadrature encoder",
          "WithExternalEncoderDiscontinuityPoint");
    // A SPARK MAX's data port is shared with the absolute encoder; select the alternate encoder.
    if (m_maxConfig) m_maxConfig->alternateEncoder.SetSparkMaxDataPortConfig();
    if (inverted) {
      if (m_maxConfig)
        m_maxConfig->alternateEncoder.Inverted(*inverted);
      else
        m_flexConfig->externalEncoder.Inverted(*inverted);
    }
    if (useExternalEncoder)
      sparkCfg.closedLoop.SetFeedbackSensor(FeedbackSensor::kAlternateOrExternalEncoder);
    if (sim) {
      if (m_maxConfig)
        m_altEncoderSim.emplace(static_cast<SparkMax*>(m_spark));
      else
        m_extEncoderSim.emplace(static_cast<SparkFlex*>(m_spark));
    }
    // A quadrature encoder counts from where it powers on, like the motor's encoder.
    const double startingRotations =
        startingPosition.value_or(wpi::units::turn_t{0.0}).value() * mechToEncoder;
    m_quadratureEncoder->SetPosition(startingRotations);
    if (m_altEncoderSim) m_altEncoderSim->SetPosition(startingRotations);
    if (m_extEncoderSim) m_extEncoderSim->SetPosition(startingRotations);
  } else if (detached) {
    // An absolute encoder the SPARK reads over CAN.
    m_detachedEncoder = detached;
    rev::detached::DetachedEncoderConfig detachedConfig;
    if (inverted) detachedConfig.Inverted(*inverted);
    m_detachedEncoderZeroOffset = zeroOffsetRotations;
    detachedConfig.ZeroOffset(m_detachedEncoderZeroOffset);
    if (discontinuityPoint && hasExternalGearing)
      m_externalEncoderGearingDiscontinuityAlert->Set(true);
    // The discontinuity point is 0.5 or 1 rotation; a zero-centered angle wraps at half a turn.
    detachedConfig.ZeroCentered(
        std::abs(m_absoluteEncoderDiscontinuityPoint.value() - 0.5) < 1e-9);
    m_detachedEncoder->Configure(detachedConfig, rev::ResetMode::kNoResetSafeParameters);
    if (useExternalEncoder)
      sparkCfg.closedLoop.SetFeedbackSensor(FeedbackSensor::kDetachedAbsoluteEncoder,
                                            *m_detachedEncoder);
    if (sim) {
      m_detachedEncoderSimAngle =
          wpi::units::turn_t{startingPosition.value_or(wpi::units::turn_t{0.0}).value() *
                             mechToEncoder};
    }
  } else {
    throw std::invalid_argument(
        "[ERROR] Unsupported external encoder: SparkWrapper accepts rev::spark::"
        "SparkAbsoluteEncoder*, SparkMaxAlternateEncoder*, SparkFlexExternalEncoder*, "
        "rev::detached::DetachedEncoder* or rev::detached::SplineEncoder*");
  }

  // Start the motor's encoder from an absolute encoder when no starting position is given.
  if (!startingPosition && (m_absEncoder || m_detachedEncoder)) SeedRelativeEncoder();
}

// ---- Simulation -------------------------------------------------------------

void SparkWrapper::SetupSimulation() {
  if (!wpi::RobotBase::IsSimulation()) return;

  const double mechToRotor = m_config->GetMotorGearing()
                                 .value_or(gearing::MechanismGearing::kOne)
                                 .GetMechanismToRotorRatio();
  if (!m_sparkSim.has_value()) {
    auto plant = wpi::math::Models::SingleJointedArmFromPhysicalConstants(
        m_motor, m_config->GetMOI(), mechToRotor);
    m_motorSim.emplace(plant, m_motor);
    SetSimSupplier(std::make_shared<simulation::DCMotorSimSupplier>(*m_motorSim, *this));
    m_sparkSim.emplace(m_spark, &m_motor);
    if (m_maxConfig)
      m_relEncoderSim.emplace(static_cast<SparkMax*>(m_spark));
    else
      m_relEncoderSim.emplace(static_cast<SparkFlex*>(m_spark));
  }

  if (auto startPos = m_config->GetStartingPosition()) {
    const double mechToEncoder = MechanismToExternalEncoderRatio();
    m_sparkSim->SetPosition(startPos->value() * m_mechanismToFeedbackSensorRatio);
    m_relEncoderSim->SetPosition(startPos->value() * mechToRotor);
    m_simSupplier->SetMechanismPosition(*startPos);
    if (m_absEncoderSim) m_absEncoderSim->SetPosition(startPos->value() * mechToEncoder);
    if (m_altEncoderSim) m_altEncoderSim->SetPosition(startPos->value() * mechToEncoder);
    if (m_extEncoderSim) m_extEncoderSim->SetPosition(startPos->value() * mechToEncoder);
    if (m_detachedEncoder)
      m_detachedEncoderSimAngle = wpi::units::turn_t{startPos->value() * mechToEncoder};
  }
}

void SparkWrapper::SimIterate() {
  if (!wpi::RobotBase::IsSimulation() || !m_simSupplier || !m_sparkSim) return;
  IterateSimulatedMaxMotionPosition();
  // Step the physics only if the mechanism has not already stepped them this loop.
  if (!m_simSupplier->GetUpdatedSim()) {
    m_simSupplier->UpdateSim();
    m_simSupplier->StarveUpdateSim();
    simulation::BatterySim::CalculateVoltage(BatterySimKey(), m_simSupplier->GetSupplyCurrent());
  }
  const double dt = m_config->GetSimulationPeriod().value();
  const double mechanismRPS = m_simSupplier->GetMechanismVelocity().value();
  // iterate() takes the RPM of the feedback sensor: SparkSim closes its loop on that sensor.
  // A SPARK closes its loop every millisecond, so step SparkSim's loop at that rate through the
  // simulation period instead of once per period.
  const double feedbackRPM = mechanismRPS * m_mechanismToFeedbackSensorRatio * 60.0;
  const double busVolts = m_simSupplier->GetMechanismSupplyVoltage().value();
  const int loopSteps = std::max(1, static_cast<int>(std::lround(dt / kSparkLoopPeriodSeconds)));
  const double stepSeconds = dt / loopSteps;
  for (int step = 0; step < loopSteps; step++) {
    m_sparkSim->iterate(feedbackRPM, busVolts, stepSeconds);
  }
  // SparkSim moves the feedback sensor; move the other encoder here.
  const double mechToEncoder = MechanismToExternalEncoderRatio();
  if (m_config->GetUseExternalFeedback() && m_config->GetExternalEncoder().has_value()) {
    const double mechToRotor = m_config->GetMotorGearing()
                                   .value_or(gearing::MechanismGearing::kOne)
                                   .GetMechanismToRotorRatio();
    if (m_relEncoderSim) m_relEncoderSim->iterate(mechanismRPS * mechToRotor * 60.0, dt);
  } else {
    const double externalEncoderRPM = mechanismRPS * mechToEncoder * 60.0;
    if (m_absEncoderSim) m_absEncoderSim->iterate(externalEncoderRPM, dt);
    if (m_altEncoderSim) m_altEncoderSim->iterate(externalEncoderRPM, dt);
    if (m_extEncoderSim) m_extEncoderSim->iterate(externalEncoderRPM, dt);
  }
  // REVLib does not simulate detached encoders, which are not wired to the SPARK.
  if (m_detachedEncoder) {
    m_detachedEncoderSimAngle =
        wpi::units::turn_t{m_simSupplier->GetMechanismPosition().value() * mechToEncoder};
  }
}

void SparkWrapper::IterateSimulatedMaxMotionPosition() {
  if (!SimulatingMaxMotion() || HasExponentialProfile() || m_lqr || !m_setpointPosition) {
    m_simMaxMotionState.reset();
    return;
  }
  using Profile = wpi::math::TrapezoidProfile<wpi::units::turns>;
  const Profile::State state = m_simMaxMotionState.value_or(
      Profile::State{GetMechanismPosition(), GetMechanismVelocity()});
  // Wrap the goal against the profile, which is continuous even where the sensor wraps.
  const wpi::units::turn_t goal =
      m_config->GetContinuousWrapping()
          ? m_config->GetContinuousWrappingSetpoint(*m_setpointPosition, state.position)
          : *m_setpointPosition;
  // The profile runs in mechanism rotations, like MAXMotion.
  std::optional<Profile> profile;
  if (auto linVel = m_config->GetTrapMaxVelocityLinear();
      linVel && !m_config->GetTrapMaxVelocityTurns()) {
    profile.emplace(Profile::Constraints{
        m_config->ConvertToMechanism(*linVel),
        m_config->ConvertToMechanism(*m_config->GetTrapMaxAccelLinear())});
  } else {
    profile = m_config->GetTrapezoidProfile();
  }
  if (!profile) return;
  const Profile::State next =
      profile->Calculate(m_config->GetSimulationPeriod(), state, Profile::State{goal, {}});
  m_simMaxMotionState = next;

  double kS = 0.0;
  double kV = 0.0;
  if (auto ff = GetSlotFeedforward(*m_config, m_slot)) {
    kS = ff->kS;
    kV = ff->kV;
  }
  // kV of a linear mechanism is per meter per second.
  double velocity = next.velocity.value();
  if (m_config->GetLinearClosedLoopControllerUse())
    velocity /= m_config->ConvertToMechanism(wpi::units::meters_per_second_t{1.0}).value();
  const double sign = next.velocity.value() > 0 ? 1.0 : (next.velocity.value() < 0 ? -1.0 : 0.0);
  const double feedforwardVolts = kS * sign + kV * velocity;
  UseVelocityGains(false);
  const double setpoint = next.position.value() * m_mechanismToFeedbackSensorRatio;
  ConfigureSpark([&] {
    return m_sparkPid->SetSetpoint(setpoint, SparkLowLevel::ControlType::kPosition,
                                   m_closedLoopSlot, feedforwardVolts,
                                   SparkClosedLoopController::ArbFFUnits::kVoltage);
  });
}

// ---- Encoder sync -----------------------------------------------------------

std::optional<wpi::units::turn_t> SparkWrapper::GetExternalEncoderSensorPosition() {
  if (m_quadratureEncoder) return wpi::units::turn_t{m_quadratureEncoder->GetPosition().Get()};
  std::optional<double> absoluteRotations;
  if (m_absEncoder) {
    absoluteRotations = m_absEncoder->GetPosition().Get();
  } else if (m_detachedEncoder) {
    absoluteRotations = wpi::RobotBase::IsSimulation() ? m_detachedEncoderSimAngle.value()
                                                       : m_detachedEncoder->GetAngle().Get();
  }
  if (!absoluteRotations) return std::nullopt;
  // REVLib does not wrap a simulated absolute encoder and does not simulate detached encoders:
  // report simulated angles in the range the encoder reports, below its discontinuity point.
  if (wpi::RobotBase::IsSimulation()) {
    const double dp = m_absoluteEncoderDiscontinuityPoint.value();
    absoluteRotations = wpi::math::InputModulus(*absoluteRotations, dp - 1.0, dp);
  }
  return wpi::units::turn_t{*absoluteRotations};
}

std::optional<wpi::units::turns_per_second_t> SparkWrapper::GetExternalEncoderSensorVelocity() {
  if (m_absEncoder) return wpi::units::turns_per_second_t{m_absEncoder->GetVelocity().Get() / 60.0};
  if (m_quadratureEncoder)
    return wpi::units::turns_per_second_t{m_quadratureEncoder->GetVelocity().Get() / 60.0};
  if (m_detachedEncoder) {
    if (wpi::RobotBase::IsSimulation()) {
      return m_simSupplier ? m_simSupplier->GetMechanismVelocity() * MechanismToExternalEncoderRatio()
                           : wpi::units::turns_per_second_t{0.0};
    }
    return wpi::units::turns_per_second_t{m_detachedEncoder->GetVelocity().Get() / 60.0};
  }
  return std::nullopt;
}

void SparkWrapper::SeedRelativeEncoder() {
  auto externalEncoderAngle = GetExternalEncoderSensorPosition();
  if (!externalEncoderAngle) return;
  const double mechToRotor = m_config->GetMotorGearing()
                                 .value_or(gearing::MechanismGearing::kOne)
                                 .GetMechanismToRotorRatio();
  const double rotorRotations =
      externalEncoderAngle->value() / MechanismToExternalEncoderRatio() * mechToRotor;
  m_relEncoder->SetPosition(rotorRotations);
  if (m_relEncoderSim) m_relEncoderSim->SetPosition(rotorRotations);
}

void SparkWrapper::SynchronizeRelativeEncoder() {
  auto threshold = m_config->GetFeedbackSynchronizationThreshold();
  if (!threshold) return;
  auto externalEncoderAngle = GetExternalEncoderSensorPosition();
  if (!externalEncoderAngle) return;
  const double rotorToMech = m_config->GetMotorGearing()
                                 .value_or(gearing::MechanismGearing::kOne)
                                 .GetRotorToMechanismRatio();
  const double relativeMechanism = m_relEncoder->GetPosition().Get() * rotorToMech;
  const double externalMechanism =
      externalEncoderAngle->value() / MechanismToExternalEncoderRatio();
  if (std::abs(relativeMechanism - externalMechanism) > threshold->value()) SeedRelativeEncoder();
}

// ---- Open-loop outputs ------------------------------------------------------

void SparkWrapper::SetDutyCycle(double dc) {
  m_spark->SetThrottle(dc);
  if (dc == 0.0) {
    for (auto* f : m_looseFollowers) f->SetDutyCycle(dc);
  }
}

double SparkWrapper::GetDutyCycle() {
  return m_sparkSim ? m_sparkSim.value().GetAppliedOutput() : m_spark->GetAppliedOutput().Get();
}

void SparkWrapper::SetVoltage(wpi::units::volt_t voltage) {
  m_spark->SetVoltage(voltage);
  if (m_simSupplier) m_simSupplier->SetMechanismStatorVoltage(voltage);
}

wpi::units::volt_t SparkWrapper::GetVoltage() {
  if (m_simSupplier) return m_simSupplier->GetMechanismStatorVoltage();
  return wpi::units::volt_t{m_spark->GetAppliedOutput().Get() * m_spark->GetBusVoltage().Get()};
}

// ---- Closed-loop setpoints --------------------------------------------------

void SparkWrapper::SetPosition(wpi::units::turn_t angle) {
  m_setpointVelocity.reset();
  m_setpointFeedforwardForce.reset();
  m_setpointPosition = angle;
  // While simulating MAXMotion, SimIterate() follows the profile to this setpoint instead.
  if (!HasExponentialProfile() && !m_lqr && !SimulatingMaxMotion()) {
    const wpi::units::turn_t setpoint =
        m_config->GetContinuousWrapping()
            ? m_config->GetContinuousWrappingSetpoint(angle, GetMechanismPosition())
            : angle;
    UseVelocityGains(false);
    ConfigureSpark([&] {
      return m_sparkPid->SetSetpoint(setpoint.value() * m_mechanismToFeedbackSensorRatio,
                                     m_positionControlType, m_closedLoopSlot);
    });
  }
  ForwardPositionToFollowers(angle);
}

void SparkWrapper::SetPosition(wpi::units::meter_t distance) {
  SetPosition(m_config->ConvertToMechanism(distance));
}

void SparkWrapper::SetVelocity(wpi::units::turns_per_second_t velocity) {
  m_setpointPosition.reset();
  m_setpointVelocity = velocity;
  m_setpointFeedforwardForce.reset();
  if (!m_lqr) {
    UseVelocityGains(true);
    ConfigureSpark([&] {
      return m_sparkPid->SetSetpoint(velocity.value() * m_mechanismToFeedbackSensorRatio * 60.0,
                                     m_velocityControlType, m_closedLoopSlot);
    });
  }
  ForwardVelocityToFollowers(velocity);
}

void SparkWrapper::SetVelocity(wpi::units::turns_per_second_t velocity,
                               wpi::units::newton_t feedforwardForce) {
  if (m_lqr) {
    // The RoboRIO loop adds the force's voltage.
    SetVelocity(velocity);
    m_setpointFeedforwardForce = feedforwardForce;
    return;
  }
  m_setpointPosition.reset();
  m_setpointVelocity = velocity;
  m_setpointFeedforwardForce = feedforwardForce;
  const double feedforwardVolts = m_config->ConvertToVoltage(GetDCMotor(), feedforwardForce).value();
  UseVelocityGains(true);
  ConfigureSpark([&] {
    return m_sparkPid->SetSetpoint(velocity.value() * m_mechanismToFeedbackSensorRatio * 60.0,
                                   m_velocityControlType, m_closedLoopSlot, feedforwardVolts,
                                   SparkClosedLoopController::ArbFFUnits::kVoltage);
  });
  for (auto* f : m_looseFollowers) f->SetVelocity(velocity, feedforwardForce);
}

void SparkWrapper::SetVelocity(wpi::units::meters_per_second_t velocity) {
  SetVelocity(m_config->ConvertToMechanism(velocity));
}

// ---- Encoder writes ---------------------------------------------------------

void SparkWrapper::SetEncoderPosition(wpi::units::turn_t angle) {
  const double externalEncoderRotations = angle.value() * MechanismToExternalEncoderRatio();
  const bool sim = wpi::RobotBase::IsSimulation();
  if (m_absEncoder) {
    // Move the zero offset so the encoder reads the angle where it is now.
    const double current = GetExternalEncoderSensorPosition().value_or(wpi::units::turn_t{0}).value();
    m_absEncoderZeroOffset =
        wpi::math::InputModulus(current + m_absEncoderZeroOffset - externalEncoderRotations, 0.0, 1.0);
    SparkConfig().absoluteEncoder.ZeroOffset(m_absEncoderZeroOffset);
    if (m_absEncoderSim) m_absEncoderSim->SetPosition(externalEncoderRotations);
    // Send the new zero offset to the SPARK; without this it only lives in the local config.
    m_spark->ConfigureAsync(SparkConfig(), rev::ResetMode::kNoResetSafeParameters,
                            rev::PersistMode::kNoPersistParameters);
  }
  if (m_quadratureEncoder) m_quadratureEncoder->SetPosition(externalEncoderRotations);
  if (m_altEncoderSim) m_altEncoderSim->SetPosition(externalEncoderRotations);
  if (m_extEncoderSim) m_extEncoderSim->SetPosition(externalEncoderRotations);
  if (m_detachedEncoder) {
    if (sim) {
      m_detachedEncoderSimAngle = wpi::units::turn_t{externalEncoderRotations};
    } else {
      m_detachedEncoderZeroOffset = wpi::math::InputModulus(
          m_detachedEncoder->GetAngle().Get() + m_detachedEncoderZeroOffset -
              externalEncoderRotations,
          0.0, 1.0);
      rev::detached::DetachedEncoderConfig detachedConfig;
      detachedConfig.ZeroOffset(m_detachedEncoderZeroOffset);
      m_detachedEncoder->Configure(detachedConfig, rev::ResetMode::kNoResetSafeParameters);
    }
  }
  const double rotor = angle.value() * m_config->GetMotorGearing()
                                           .value_or(gearing::MechanismGearing::kOne)
                                           .GetMechanismToRotorRatio();
  m_relEncoder->SetPosition(rotor);
  if (m_relEncoderSim) m_relEncoderSim->SetPosition(rotor);
  if (m_sparkSim) m_sparkSim->SetPosition(angle.value() * m_mechanismToFeedbackSensorRatio);
  if (m_simSupplier) m_simSupplier->SetMechanismPosition(angle);
}

void SparkWrapper::SetEncoderPosition(wpi::units::meter_t distance) {
  SetEncoderPosition(m_config->ConvertToMechanism(distance));
}

void SparkWrapper::SetEncoderVelocity(wpi::units::turns_per_second_t velocity) {
  if (!wpi::RobotBase::IsSimulation())
    throw std::runtime_error("REV Spark does not support setting encoder velocity.");
  const double mechToRotor = m_config->GetMotorGearing()
                                 .value_or(gearing::MechanismGearing::kOne)
                                 .GetMechanismToRotorRatio();
  const double externalEncoderRPM = velocity.value() * MechanismToExternalEncoderRatio() * 60.0;
  if (m_relEncoderSim) m_relEncoderSim->SetVelocity(velocity.value() * mechToRotor * 60.0);
  if (m_sparkSim) m_sparkSim->SetVelocity(velocity.value() * m_mechanismToFeedbackSensorRatio * 60.0);
  if (m_absEncoderSim) m_absEncoderSim->SetVelocity(externalEncoderRPM);
  if (m_altEncoderSim) m_altEncoderSim->SetVelocity(externalEncoderRPM);
  if (m_extEncoderSim) m_extEncoderSim->SetVelocity(externalEncoderRPM);
}

void SparkWrapper::SetEncoderVelocity(wpi::units::meters_per_second_t velocity) {
  SetEncoderVelocity(m_config->ConvertToMechanism(velocity));
}

// ---- Encoder reads ----------------------------------------------------------

wpi::units::turn_t SparkWrapper::GetMechanismPosition() {
  if (m_config->GetUseExternalFeedback()) {
    if (auto external = GetExternalEncoderMechanismPosition()) return *external;
  }
  return GetRelativeMechanismPosition();
}

wpi::units::turns_per_second_t SparkWrapper::GetMechanismVelocity() {
  if (m_config->GetUseExternalFeedback()) {
    if (auto external = GetExternalEncoderMechanismVelocity()) return *external;
  }
  return GetRelativeMechanismVelocity();
}

wpi::units::turn_t SparkWrapper::GetRelativeMechanismPosition() {
  return wpi::units::turn_t{m_relEncoder->GetPosition().Get() *
                            m_config->GetMotorGearing()
                                .value_or(gearing::MechanismGearing::kOne)
                                .GetRotorToMechanismRatio()};
}

wpi::units::turns_per_second_t SparkWrapper::GetRelativeMechanismVelocity() {
  const double rotorRPM = m_sparkSim ? m_sparkSim->GetVelocity() : m_relEncoder->GetVelocity().Get();
  return wpi::units::turns_per_second_t{rotorRPM / 60.0 *
                                        m_config->GetMotorGearing()
                                            .value_or(gearing::MechanismGearing::kOne)
                                            .GetRotorToMechanismRatio()};
}

wpi::units::turns_per_second_squared_t SparkWrapper::GetMechanismAcceleration() {
  return wpi::units::turns_per_second_squared_t{
      m_accelFilter.Derivative(GetMechanismVelocity().value())};
}

wpi::units::turn_t SparkWrapper::GetRotorPosition() {
  return GetMechanismPosition() * m_config->GetMotorGearing()
                                      .value_or(gearing::MechanismGearing::kOne)
                                      .GetMechanismToRotorRatio();
}

wpi::units::turns_per_second_t SparkWrapper::GetRotorVelocity() {
  return GetMechanismVelocity() * m_config->GetMotorGearing()
                                      .value_or(gearing::MechanismGearing::kOne)
                                      .GetMechanismToRotorRatio();
}

wpi::units::meter_t SparkWrapper::GetMeasurementPosition() {
  return m_config->ConvertFromMechanism(GetMechanismPosition());
}
wpi::units::meters_per_second_t SparkWrapper::GetMeasurementVelocity() {
  return m_config->ConvertFromMechanism(GetMechanismVelocity());
}
wpi::units::meters_per_second_squared_t SparkWrapper::GetMeasurementAcceleration() {
  return m_config->ConvertFromMechanism(GetMechanismAcceleration());
}

std::optional<wpi::units::turn_t> SparkWrapper::GetExternalEncoderMechanismPosition() {
  auto sensor = GetExternalEncoderSensorPosition();
  if (!sensor) return std::nullopt;
  return wpi::units::turn_t{sensor->value() / MechanismToExternalEncoderRatio()};
}

std::optional<wpi::units::turns_per_second_t> SparkWrapper::GetExternalEncoderMechanismVelocity() {
  auto sensor = GetExternalEncoderSensorVelocity();
  if (!sensor) return std::nullopt;
  return wpi::units::turns_per_second_t{sensor->value() / MechanismToExternalEncoderRatio()};
}

// ---- Motor status -----------------------------------------------------------

std::optional<wpi::units::ampere_t> SparkWrapper::GetSupplyCurrent() { return std::nullopt; }

wpi::units::ampere_t SparkWrapper::GetStatorCurrent() {
  if (m_simSupplier) return m_simSupplier->GetStatorCurrent();
  return wpi::units::ampere_t{m_spark->GetOutputCurrent().Get()};
}

wpi::units::celsius_t SparkWrapper::GetTemperature() {
  return wpi::units::celsius_t{m_spark->GetMotorTemperature().Get()};
}

wpi::math::DCMotor SparkWrapper::GetDCMotor() { return m_motor; }

// ---- Live-tuning setters ----------------------------------------------------

void SparkWrapper::SetZeroPower(MotorMode mode) {
  SparkConfig().SetIdleMode(mode == MotorMode::BRAKE ? SparkBaseConfig::IdleMode::kBrake
                                                     : SparkBaseConfig::IdleMode::kCoast);
  ConfigureSpark([this] {
    return m_spark->Configure(SparkConfig(), rev::ResetMode::kNoResetSafeParameters,
                              PersistModeForState());
  });
}

void SparkWrapper::SetMotorInverted(bool inverted) {
  m_config->WithMotorInverted(inverted);
  SparkConfig().Inverted(inverted);
  ConfigureAsync();
}

void SparkWrapper::SetEncoderInverted(bool inverted) {
  m_config->WithEncoderInverted(inverted);
  SparkConfig().encoder.Inverted(inverted);
  ConfigureAsync();
}

void SparkWrapper::ApplyFeedback(double kP, double kI, double kD) {
  m_config->WithFeedback(kP, kI, kD, m_slot);
  if (m_pid) {
    m_pid->SetP(kP);
    m_pid->SetI(kI);
    m_pid->SetD(kD);
  }
  // The SPARK's gains are in its own units; write the whole slot.
  WriteSparkPID(kP, kI, kD, m_closedLoopSlot);
  ConfigureAsync();
}

void SparkWrapper::SetKp(double kP) {
  const auto gains = m_config->GetSlotGains(m_slot);
  ApplyFeedback(kP, gains.kI, gains.kD);
  for (auto* f : m_looseFollowers) f->SetKp(kP);
}

void SparkWrapper::SetKi(double kI) {
  const auto gains = m_config->GetSlotGains(m_slot);
  ApplyFeedback(gains.kP, kI, gains.kD);
  for (auto* f : m_looseFollowers) f->SetKi(kI);
}

void SparkWrapper::SetKd(double kD) {
  const auto gains = m_config->GetSlotGains(m_slot);
  ApplyFeedback(gains.kP, gains.kI, kD);
  for (auto* f : m_looseFollowers) f->SetKd(kD);
}

void SparkWrapper::SetFeedback(double kP, double kI, double kD) {
  ApplyFeedback(kP, kI, kD);
  for (auto* f : m_looseFollowers) f->SetFeedback(kP, kI, kD);
}

void SparkWrapper::SetKs(double kS) {
  UpdateConfigFeedforward(*m_config, m_slot, [&](NativeFeedforward& v) { v.kS = kS; });
  SparkConfig().closedLoop.feedForward.kS(kS, m_closedLoopSlot);
  ConfigureAsync();
  for (auto* f : m_looseFollowers) f->SetKs(kS);
}

void SparkWrapper::SetKv(double kV) {
  UpdateConfigFeedforward(*m_config, m_slot, [&](NativeFeedforward& v) { v.kV = kV; });
  const double velocityPerGainUnit =
      m_config->GetLinearClosedLoopControllerUse()
          ? m_config->ConvertToMechanism(wpi::units::meters_per_second_t{1.0}).value()
          : 1.0;
  SparkConfig().closedLoop.feedForward.kV(KvPerRotation(*m_config, m_slot, kV) /
                                              (m_mechanismToFeedbackSensorRatio * 60.0) /
                                              velocityPerGainUnit,
                                          m_closedLoopSlot);
  ConfigureAsync();
  for (auto* f : m_looseFollowers) f->SetKv(kV);
}

void SparkWrapper::SetKa(double kA) {
  UpdateConfigFeedforward(*m_config, m_slot, [&](NativeFeedforward& v) { v.kA = kA; });
  const double accelerationPerGainUnit =
      m_config->GetLinearClosedLoopControllerUse()
          ? m_config->ConvertToMechanism(wpi::units::meters_per_second_squared_t{1.0}).value()
          : 1.0;
  SparkConfig().closedLoop.feedForward.kA(KaPerRotation(*m_config, m_slot, kA) /
                                              (m_mechanismToFeedbackSensorRatio * 60.0) /
                                              accelerationPerGainUnit,
                                          m_closedLoopSlot);
  ConfigureAsync();
  for (auto* f : m_looseFollowers) f->SetKa(kA);
}

void SparkWrapper::SetKg(double kG) {
  UpdateConfigFeedforward(*m_config, m_slot, [&](NativeFeedforward& v) { v.kG = kG; });
  if (m_config->GetArmFeedforward(m_slot)) {
    SparkConfig()
        .closedLoop.feedForward.kCos(kG, m_closedLoopSlot)
        .kCosRatio(1.0 / m_mechanismToFeedbackSensorRatio, m_closedLoopSlot);
  } else {
    SparkConfig().closedLoop.feedForward.kG(kG, m_closedLoopSlot);
  }
  ConfigureAsync();
  for (auto* f : m_looseFollowers) f->SetKg(kG);
}

void SparkWrapper::SetFeedforward(double kS, double kV, double kA, double kG) {
  UpdateConfigFeedforward(*m_config, m_slot, [&](NativeFeedforward& v) {
    v.kS = kS;
    v.kV = kV;
    v.kA = kA;
    v.kG = kG;
  });
  if (m_config->GetArmFeedforward(m_slot)) {
    SparkConfig()
        .closedLoop.feedForward.kCos(kG, m_closedLoopSlot)
        .kCosRatio(1.0 / m_mechanismToFeedbackSensorRatio, m_closedLoopSlot);
  } else if (m_config->GetElevatorFeedforward(m_slot)) {
    SparkConfig().closedLoop.feedForward.kG(kG, m_closedLoopSlot);
  }
  WriteSparkFeedforward(kS, KvPerRotation(*m_config, m_slot, kV),
                        KaPerRotation(*m_config, m_slot, kA), m_closedLoopSlot);
  ConfigureAsync();
  for (auto* f : m_looseFollowers) f->SetFeedforward(kS, kV, kA, kG);
}

void SparkWrapper::SetStatorCurrentLimit(wpi::units::ampere_t limit) {
  m_config->WithStatorCurrentLimit(limit);
  SparkConfig().SmartCurrentLimit(static_cast<int>(limit.value()));
  ConfigureAsync();
  for (auto* f : m_looseFollowers) f->SetStatorCurrentLimit(limit);
}

void SparkWrapper::SetSupplyCurrentLimit(wpi::units::ampere_t) {}

void SparkWrapper::SetClosedLoopRampRate(wpi::units::second_t r) {
  m_config->WithClosedLoopRampRate(r);
  SparkConfig().ClosedLoopRampRate(r.value());
  ConfigureAsync();
  for (auto* f : m_looseFollowers) f->SetClosedLoopRampRate(r);
}

void SparkWrapper::SetOpenLoopRampRate(wpi::units::second_t r) {
  m_config->WithOpenLoopRampRate(r);
  SparkConfig().OpenLoopRampRate(r.value());
  ConfigureAsync();
  for (auto* f : m_looseFollowers) f->SetOpenLoopRampRate(r);
}

void SparkWrapper::SetMechanismUpperLimit(wpi::units::turn_t upper) {
  if (auto lower = m_config->GetMechanismLowerLimit()) m_config->WithMechanismLimits(*lower, upper);
  SparkConfig().softLimit.ForwardSoftLimit(upper.value() * m_mechanismToFeedbackSensorRatio);
  ConfigureAsync();
  for (auto* f : m_looseFollowers) f->SetMechanismUpperLimit(upper);
}

void SparkWrapper::SetMechanismLowerLimit(wpi::units::turn_t lower) {
  if (auto upper = m_config->GetMechanismUpperLimit()) m_config->WithMechanismLimits(lower, *upper);
  SparkConfig().softLimit.ReverseSoftLimit(lower.value() * m_mechanismToFeedbackSensorRatio);
  ConfigureAsync();
  for (auto* f : m_looseFollowers) f->SetMechanismLowerLimit(lower);
}

void SparkWrapper::SetMechanismLimits(wpi::units::turn_t lower, wpi::units::turn_t upper) {
  m_config->WithMechanismLimits(lower, upper);
  SparkConfig()
      .softLimit.ReverseSoftLimit(lower.value() * m_mechanismToFeedbackSensorRatio)
      .ForwardSoftLimit(upper.value() * m_mechanismToFeedbackSensorRatio);
  ConfigureAsync();
  for (auto* f : m_looseFollowers) f->SetMechanismLimits(lower, upper);
}

void SparkWrapper::SetMechanismLimitsEnabled(bool enabled) {
  SparkConfig().softLimit.ForwardSoftLimitEnabled(enabled).ReverseSoftLimitEnabled(enabled);
  ConfigureAsync();
  for (auto* f : m_looseFollowers) f->SetMechanismLimitsEnabled(enabled);
}

void SparkWrapper::SetMeasurementUpperLimit(wpi::units::meter_t upper) {
  auto lowerAngle = m_config->GetMechanismLowerLimit();
  if (!m_config->GetMechanismCircumference() || !lowerAngle) return;
  m_config->WithMeasurementLimits(m_config->ConvertFromMechanism(*lowerAngle), upper);
  SparkConfig().softLimit.ForwardSoftLimit(m_config->ConvertToMechanism(upper).value() *
                                           m_mechanismToFeedbackSensorRatio);
  ConfigureAsync();
  for (auto* f : m_looseFollowers) f->SetMeasurementUpperLimit(upper);
}

void SparkWrapper::SetMeasurementLowerLimit(wpi::units::meter_t lower) {
  auto upperAngle = m_config->GetMechanismUpperLimit();
  if (!m_config->GetMechanismCircumference() || !upperAngle) return;
  m_config->WithMeasurementLimits(lower, m_config->ConvertFromMechanism(*upperAngle));
  SparkConfig().softLimit.ReverseSoftLimit(m_config->ConvertToMechanism(lower).value() *
                                           m_mechanismToFeedbackSensorRatio);
  ConfigureAsync();
  for (auto* f : m_looseFollowers) f->SetMeasurementLowerLimit(lower);
}

void SparkWrapper::SetMotionProfileMaxVelocity(wpi::units::turns_per_second_t vel) {
  if (!m_config->GetVelocityTrapezoidalProfileInUse()) {
    // Keep the tuned constraints in the config, so tuning the next one keeps this one.
    if (auto accLin = m_config->GetTrapMaxAccelLinear(); accLin && !m_config->GetTrapMaxVelocityTurns()) {
      m_config->WithLinearTrapezoidProfile(m_config->ConvertFromMechanism(vel), *accLin);
    } else if (auto acc = m_config->GetTrapMaxAccelTurns()) {
      m_config->WithTrapezoidProfile(vel, *acc);
    }
  }
  for (auto sparkSlot : kAllSparkSlots)
    SparkConfig().closedLoop.maxMotion.CruiseVelocity(
        vel.value() * m_mechanismToFeedbackSensorRatio * 60.0, sparkSlot);
  ConfigureAsync();
  for (auto* f : m_looseFollowers) f->SetMotionProfileMaxVelocity(vel);
}

void SparkWrapper::SetMotionProfileMaxVelocity(wpi::units::meters_per_second_t vel) {
  // Convert first so a missing circumference throws before anything changes.
  const auto mechanismVelocity = m_config->ConvertToMechanism(vel);
  if (!m_config->GetVelocityTrapezoidalProfileInUse()) {
    if (auto accLin = m_config->GetTrapMaxAccelLinear(); accLin && !m_config->GetTrapMaxVelocityTurns()) {
      m_config->WithLinearTrapezoidProfile(vel, *accLin);
    } else if (auto acc = m_config->GetTrapMaxAccelTurns()) {
      m_config->WithTrapezoidProfile(mechanismVelocity, *acc);
    }
  }
  for (auto sparkSlot : kAllSparkSlots)
    SparkConfig().closedLoop.maxMotion.CruiseVelocity(
        mechanismVelocity.value() * m_mechanismToFeedbackSensorRatio * 60.0, sparkSlot);
  ConfigureAsync();
  for (auto* f : m_looseFollowers) f->SetMotionProfileMaxVelocity(vel);
}

void SparkWrapper::SetMotionProfileMaxAcceleration(wpi::units::turns_per_second_squared_t acc) {
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
  for (auto sparkSlot : kAllSparkSlots)
    SparkConfig().closedLoop.maxMotion.MaxAcceleration(
        acc.value() * m_mechanismToFeedbackSensorRatio * 60.0, sparkSlot);
  ConfigureAsync();
  for (auto* f : m_looseFollowers) f->SetMotionProfileMaxAcceleration(acc);
}

void SparkWrapper::SetMotionProfileMaxAcceleration(wpi::units::meters_per_second_squared_t acc) {
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
  for (auto sparkSlot : kAllSparkSlots)
    SparkConfig().closedLoop.maxMotion.MaxAcceleration(
        mechanismAcceleration.value() * m_mechanismToFeedbackSensorRatio * 60.0, sparkSlot);
  ConfigureAsync();
  for (auto* f : m_looseFollowers) f->SetMotionProfileMaxAcceleration(acc);
}

void SparkWrapper::SetMotionProfileMaxJerk(
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

void SparkWrapper::SetExponentialProfile(std::optional<double> kV, std::optional<double> kA,
                                         std::optional<wpi::units::volt_t> maxInput) {
  if (!m_config->GetExponentialProfile()) return;

  double newKV = kV.value_or(m_config->GetExponentialProfileKV().value_or(0.0));
  double newKA = kA.value_or(m_config->GetExponentialProfileKA().value_or(0.0));
  wpi::units::volt_t newMaxInput =
      maxInput.value_or(m_config->GetExponentialProfileMaxInput().value_or(12_V));

  // Keep the tuned constraints in the config, so tuning the next one keeps this one.
  m_config->WithExponentialProfile(newKV, newKA, newMaxInput);

  for (auto* f : m_looseFollowers) f->SetExponentialProfile(kV, kA, maxInput);
}

void SparkWrapper::SetClosedLoopSlot(ClosedLoopControllerSlot slot) {
  m_slot = slot;
  m_closedLoopSlot = ToSparkSlot(slot);
  for (auto* f : m_looseFollowers) f->SetClosedLoopSlot(slot);
}

void SparkWrapper::SetMechanismGearing(const gearing::MechanismGearing& gearing) {
  SmartMotorController::SetMechanismGearing(gearing);
  RewriteScaledParameters();
}

void SparkWrapper::SetMechanismCircumference(wpi::units::meter_t circumference) {
  SmartMotorController::SetMechanismCircumference(circumference);
  RewriteScaledParameters();
  for (auto* f : m_looseFollowers) f->SetMechanismCircumference(circumference);
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

}  // namespace yams::motorcontrollers::local
