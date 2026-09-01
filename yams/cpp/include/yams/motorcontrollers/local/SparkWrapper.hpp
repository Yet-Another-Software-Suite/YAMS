// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <wpi/util/Alert.hpp>
#include <wpi/simulation/DCMotorSim.hpp>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkSim.h>
#include <rev/config/SparkFlexConfig.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/sim/SparkAbsoluteEncoderSim.h>
#include <rev/sim/SparkRelativeEncoderSim.h>

#include <memory>
#include <optional>
#include <variant>

#include "yams/math/DerivativeTimeFilter.hpp"
#include "yams/motorcontrollers/SmartMotorController.hpp"

namespace yams::motorcontrollers::local {

/**
 * SmartMotorController implementation for REV SPARK Max and SPARK Flex motor controllers.
 *
 * Supports both SPARK Max and SPARK Flex hardware via a common internal interface.
 * Wraps REV SparkBase, SparkClosedLoopController, and encoder objects to satisfy the
 * SmartMotorController contract.
 *
 * ### Example usage — SPARK Max (inside a subsystem constructor)
 * @code{.cpp}
 * using namespace yams::motorcontrollers;
 * using namespace yams::motorcontrollers::local;
 * using namespace yams::gearing;
 * using Cfg = SmartMotorControllerConfig;
 *
 * // Declare as subsystem members:
 * //   rev::spark::SparkMax          m_sparkMax{3,
 * rev::spark::SparkLowLevel::MotorType::kBrushless};
 * //   std::optional<SparkWrapper>   m_smc;
 *
 * SmartMotorControllerConfig cfg;
 * cfg.WithSubsystem(this)
 *    .WithFeedback(1.0, 0.0, 0.0)
 *    .WithMechanismCircumference(0.25_in, 22)
 *    .WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({3.0, 4.0})})
 *    .WithIdleMode(Cfg::MotorMode::BRAKE)
 *    .WithSupplyCurrentLimit(40.0_A)
 *    .WithMotorInverted(false)
 *    .WithFeedforward(wpi::math::ElevatorFeedforward{
 *        0.0_V, 0.0_V, wpi::units::unit_t<wpi::math::ElevatorFeedforward::kv_unit>{0.0},
 *        wpi::units::unit_t<wpi::math::ElevatorFeedforward::ka_unit>{0.0}})
 *    .WithClosedLoopMode()
 *    .WithTelemetry("ElevatorMotor", Cfg::TelemetryVerbosity::HIGH);
 *
 * m_smc.emplace(&m_sparkMax, wpi::math::DCMotor::NEO(1), &cfg);
 * @endcode
 *
 * ### Example usage — SPARK Flex
 * @code{.cpp}
 * // Declare as subsystem members:
 * //   rev::spark::SparkFlex        m_sparkFlex{4,
 * rev::spark::SparkLowLevel::MotorType::kBrushless};
 * //   std::optional<SparkWrapper>  m_smc;
 *
 * m_smc.emplace(&m_sparkFlex, wpi::math::DCMotor::NeoVortex(1), &cfg);
 * @endcode
 */
class SparkWrapper : public SmartMotorController {
 public:
  /**
   * Construct a SparkWrapper around a SPARK Max.
   *
   * @param spark  Pointer to the SPARK Max hardware object (must outlive this wrapper).
   * @param motor  DC motor model used for simulation.
   * @param config Pointer to the SmartMotorControllerConfig (must outlive this wrapper).
   */
  SparkWrapper(rev::spark::SparkMax* spark, wpi::math::DCMotor motor, SmartMotorControllerConfig* config);

  /**
   * Construct a SparkWrapper around a SPARK Flex.
   *
   * @param spark  Pointer to the SPARK Flex hardware object (must outlive this wrapper).
   * @param motor  DC motor model used for simulation.
   * @param config Pointer to the SmartMotorControllerConfig (must outlive this wrapper).
   */
  SparkWrapper(rev::spark::SparkFlex* spark, wpi::math::DCMotor motor,
               SmartMotorControllerConfig* config);
  ~SparkWrapper();

  // ---- Telemetry ----------------------------------------------------------
  /** @copydoc SmartMotorController::GetUnsupportedTelemetryFields */
  telemetry::UnsupportedTelemetryFields GetUnsupportedTelemetryFields() override;

  // ---- Configuration ------------------------------------------------------
  /** @copydoc SmartMotorController::ApplyConfig */
  bool ApplyConfig(const SmartMotorControllerConfig& config) override;

  // ---- Simulation ---------------------------------------------------------
  /** @copydoc SmartMotorController::SetupSimulation */
  void SetupSimulation() override;
  /** @copydoc SmartMotorController::SimIterate */
  void SimIterate() override;

  // ---- Encoder sync -------------------------------------------------------
  /** @copydoc SmartMotorController::SeedRelativeEncoder */
  void SeedRelativeEncoder() override;
  /** @copydoc SmartMotorController::SynchronizeRelativeEncoder */
  void SynchronizeRelativeEncoder() override;

  // ---- Open-loop outputs --------------------------------------------------
  /** @copydoc SmartMotorController::SetDutyCycle */
  void SetDutyCycle(double dutyCycle) override;
  /** @copydoc SmartMotorController::GetDutyCycle */
  double GetDutyCycle() override;
  /** @copydoc SmartMotorController::SetVoltage */
  void SetVoltage(wpi::units::volt_t voltage) override;
  /** @copydoc SmartMotorController::GetVoltage */
  wpi::units::volt_t GetVoltage() override;

  // ---- Closed-loop setpoints ----------------------------------------------
  /** @copydoc SmartMotorController::SetPosition(wpi::units::turn_t) */
  void SetPosition(wpi::units::turn_t angle) override;
  /**
   * Command a linear measurement position setpoint (closed-loop).
   * Converts distance to turns using the configured mechanism circumference.
   *
   * @param distance Target linear distance.
   */
  void SetPosition(wpi::units::meter_t distance) override;
  /** @copydoc SmartMotorController::SetVelocity(wpi::units::turns_per_second_t) */
  void SetVelocity(wpi::units::turns_per_second_t velocity) override;
  /**
   * Command a linear measurement velocity setpoint (closed-loop).
   * Converts linear velocity to turns per second using the configured mechanism circumference.
   *
   * @param velocity Target linear velocity.
   */
  void SetVelocity(wpi::units::meters_per_second_t velocity) override;

  // ---- Encoder writes -----------------------------------------------------
  /** @copydoc SmartMotorController::SetEncoderPosition(wpi::units::turn_t) */
  void SetEncoderPosition(wpi::units::turn_t angle) override;
  /**
   * Write a linear distance into the encoder (seeds the position).
   * Converts distance to turns using the configured mechanism circumference.
   *
   * @param distance Linear distance to write.
   */
  void SetEncoderPosition(wpi::units::meter_t distance) override;
  /** Not supported by SPARK hardware; has no effect. */
  void SetEncoderVelocity(wpi::units::turns_per_second_t velocity) override;
  /** Not supported by SPARK hardware; has no effect. */
  void SetEncoderVelocity(wpi::units::meters_per_second_t velocity) override;

  // ---- Encoder reads ------------------------------------------------------
  /** @copydoc SmartMotorController::GetMechanismPosition */
  wpi::units::turn_t GetMechanismPosition() override;
  /** @copydoc SmartMotorController::GetMechanismVelocity */
  wpi::units::turns_per_second_t GetMechanismVelocity() override;
  /** @copydoc SmartMotorController::GetMechanismAcceleration */
  wpi::units::turns_per_second_squared_t GetMechanismAcceleration() override;
  /** @copydoc SmartMotorController::GetRotorPosition */
  wpi::units::turn_t GetRotorPosition() override;
  /** @copydoc SmartMotorController::GetRotorVelocity */
  wpi::units::turns_per_second_t GetRotorVelocity() override;
  /** @copydoc SmartMotorController::GetMeasurementPosition */
  wpi::units::meter_t GetMeasurementPosition() override;
  /** @copydoc SmartMotorController::GetMeasurementVelocity */
  wpi::units::meters_per_second_t GetMeasurementVelocity() override;
  /** @copydoc SmartMotorController::GetMeasurementAcceleration */
  wpi::units::meters_per_second_squared_t GetMeasurementAcceleration() override;
  /** @copydoc SmartMotorController::GetExternalEncoderPosition */
  std::optional<wpi::units::degree_t> GetExternalEncoderPosition() override;
  /** @copydoc SmartMotorController::GetExternalEncoderVelocity */
  std::optional<wpi::units::degrees_per_second_t> GetExternalEncoderVelocity() override;

  // ---- Motor status -------------------------------------------------------
  /** @copydoc SmartMotorController::GetSupplyCurrent */
  std::optional<wpi::units::ampere_t> GetSupplyCurrent() override;
  /** @copydoc SmartMotorController::GetStatorCurrent */
  wpi::units::ampere_t GetStatorCurrent() override;
  /** @copydoc SmartMotorController::GetTemperature */
  wpi::units::celsius_t GetTemperature() override;
  /** @copydoc SmartMotorController::GetDCMotor */
  wpi::math::DCMotor GetDCMotor() override;

  // ---- Configuration setters (live tuning) --------------------------------
  /** @copydoc SmartMotorController::SetIdleMode */
  void SetIdleMode(MotorMode mode) override;
  /** @copydoc SmartMotorController::SetMotorInverted */
  void SetMotorInverted(bool inverted) override;
  /** @copydoc SmartMotorController::SetEncoderInverted */
  void SetEncoderInverted(bool inverted) override;
  /** @copydoc SmartMotorController::SetKp */
  void SetKp(double kP) override;
  /** @copydoc SmartMotorController::SetKi */
  void SetKi(double kI) override;
  /** @copydoc SmartMotorController::SetKd */
  void SetKd(double kD) override;
  /** @copydoc SmartMotorController::SetFeedback */
  void SetFeedback(double kP, double kI, double kD) override;
  /** @copydoc SmartMotorController::SetKs */
  void SetKs(double kS) override;
  /** @copydoc SmartMotorController::SetKv */
  void SetKv(double kV) override;
  /** @copydoc SmartMotorController::SetKa */
  void SetKa(double kA) override;
  /**
   * Set the gravity feedforward gain (live update).
   *
   * Uses kCos if an arm feedforward is configured (cosine gravity compensation),
   * or kG otherwise (elevator/simple constant gravity compensation).
   *
   * @param kG Gravity compensation coefficient.
   */
  void SetKg(double kG) override;
  /** @copydoc SmartMotorController::SetFeedforward */
  void SetFeedforward(double kS, double kV, double kA, double kG) override;
  /** @copydoc SmartMotorController::SetStatorCurrentLimit */
  void SetStatorCurrentLimit(wpi::units::ampere_t currentLimit) override;
  /** @copydoc SmartMotorController::SetSupplyCurrentLimit */
  void SetSupplyCurrentLimit(wpi::units::ampere_t currentLimit) override;
  /** @copydoc SmartMotorController::SetClosedLoopRampRate */
  void SetClosedLoopRampRate(wpi::units::second_t rampRate) override;
  /** @copydoc SmartMotorController::SetOpenLoopRampRate */
  void SetOpenLoopRampRate(wpi::units::second_t rampRate) override;
  /** @copydoc SmartMotorController::SetMechanismUpperLimit */
  void SetMechanismUpperLimit(wpi::units::turn_t upperLimit) override;
  /** @copydoc SmartMotorController::SetMechanismLowerLimit */
  void SetMechanismLowerLimit(wpi::units::turn_t lowerLimit) override;
  /** @copydoc SmartMotorController::SetMechanismLimits */
  void SetMechanismLimits(wpi::units::turn_t lower, wpi::units::turn_t upper) override;
  /** @copydoc SmartMotorController::SetMechanismLimitsEnabled */
  void SetMechanismLimitsEnabled(bool enabled) override;
  /**
   * Set the upper soft limit via a linear measurement.
   *
   * Requires both a mechanism circumference and a lower limit to already be configured.
   * Converts the distance to turns using the circumference and applies it as the
   * forward soft limit on the SPARK hardware.
   *
   * @param upperLimit Maximum linear distance the mechanism may travel forward.
   */
  void SetMeasurementUpperLimit(wpi::units::meter_t upperLimit) override;
  /**
   * Set the lower soft limit via a linear measurement.
   *
   * Requires both a mechanism circumference and an upper limit to already be configured.
   * Converts the distance to turns using the circumference and applies it as the
   * reverse soft limit on the SPARK hardware.
   *
   * @param lowerLimit Minimum linear distance the mechanism may travel in reverse.
   */
  void SetMeasurementLowerLimit(wpi::units::meter_t lowerLimit) override;
  /** @copydoc SmartMotorController::SetMotionProfileMaxVelocity(wpi::units::turns_per_second_t) */
  void SetMotionProfileMaxVelocity(wpi::units::turns_per_second_t maxVelocity) override;
  /**
   * Set the maximum linear velocity for the motion profile.
   * Converts linear velocity to turns per second using the configured mechanism circumference.
   *
   * @param maxVelocity Maximum linear velocity.
   */
  void SetMotionProfileMaxVelocity(wpi::units::meters_per_second_t maxVelocity) override;
  /** @copydoc
   * SmartMotorController::SetMotionProfileMaxAcceleration(wpi::units::turns_per_second_squared_t) */
  void SetMotionProfileMaxAcceleration(wpi::units::turns_per_second_squared_t maxAcc) override;
  /**
   * Set the maximum linear acceleration for the motion profile.
   * Converts linear acceleration to turns per second squared using the configured mechanism
   * circumference.
   *
   * @param maxAcc Maximum linear acceleration.
   */
  void SetMotionProfileMaxAcceleration(wpi::units::meters_per_second_squared_t maxAcc) override;
  /** SPARK MAXMotion does not expose jerk limiting; has no effect. */
  void SetMotionProfileMaxJerk(wpi::units::angular_jerk::turns_per_second_cubed_t maxJerk) override;
  /** Exponential profiles are not supported by SPARK hardware; has no effect. */
  void SetExponentialProfile(std::optional<double> kV, std::optional<double> kA,
                             std::optional<wpi::units::volt_t> maxInput) override;
  /** @copydoc SmartMotorController::SetClosedLoopSlot */
  void SetClosedLoopSlot(ClosedLoopControllerSlot slot) override;

  /** @copydoc SmartMotorController::GetConfig */
  SmartMotorControllerConfig& GetConfig() override;
  /**
   * Get a raw pointer to the underlying SPARK hardware object.
   *
   * @return Pointer to the rev::spark::SparkBase instance (SparkMax or SparkFlex).
   */
  void* GetMotorController() override;
  /**
   * Get a raw pointer to the internally managed SPARK configuration object.
   *
   * @return Pointer to the active SparkMaxConfig or SparkFlexConfig instance.
   */
  void* GetMotorControllerConfig() override;

 private:
  SmartMotorControllerConfig* m_config{nullptr};
  rev::spark::SparkBase* m_spark{nullptr};
  rev::spark::SparkClosedLoopController* m_sparkPid{nullptr};
  rev::spark::SparkRelativeEncoder* m_relEncoder{nullptr};
  rev::spark::SparkAbsoluteEncoder* m_absEncoder{nullptr};
  wpi::math::DCMotor m_motor;

  // SparkMaxConfig and SparkFlexConfig are non-copyable/non-movable;
  // exactly one will be constructed via emplace().
  std::optional<rev::spark::SparkMaxConfig> m_maxConfig;
  std::optional<rev::spark::SparkFlexConfig> m_flexConfig;

  std::optional<wpi::sim::DCMotorSim> m_motorSim;
  std::optional<rev::spark::SparkSim> m_sparkSim;
  std::optional<rev::spark::SparkRelativeEncoderSim> m_relEncoderSim;
  std::optional<rev::spark::SparkAbsoluteEncoderSim> m_absEncoderSim;
  math::DerivativeTimeFilter m_accelFilter{20_ms};

  rev::spark::SparkLowLevel::ControlType m_positionControlType{
      rev::spark::SparkLowLevel::ControlType::kPosition};
  rev::spark::SparkLowLevel::ControlType m_velocityControlType{
      rev::spark::SparkLowLevel::ControlType::kVelocity};

  int m_revSlot{0};

  std::optional<wpi::util::Alert> m_rioControllerAlert;

  void Init(rev::spark::SparkBase* spark, wpi::math::DCMotor motor, SmartMotorControllerConfig* config);
  void ApplyBaseConfig();
  void CommitConfig();
};

}  // namespace yams::motorcontrollers::local
