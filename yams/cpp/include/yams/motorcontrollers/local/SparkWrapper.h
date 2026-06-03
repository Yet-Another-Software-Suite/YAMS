// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/simulation/DCMotorSim.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/config/SparkFlexConfig.h>
#include <rev/config/SparkMaxConfig.h>

#include <memory>
#include <optional>
#include <variant>

#include "yams/math/DerivativeTimeFilter.h"
#include "yams/motorcontrollers/SmartMotorController.h"

namespace yams::motorcontrollers::local {

/**
 * SmartMotorController implementation for REV SPARK Max and SPARK Flex motor controllers.
 *
 * Supports both SPARK Max and SPARK Flex hardware via a common internal interface.
 * Wraps REV SparkBase, SparkClosedLoopController, and encoder objects to satisfy the
 * SmartMotorController contract.
 */
class SparkWrapper : public SmartMotorController {
 public:
  /**
   * Construct a SparkWrapper around a SPARK Max.
   *
   * @param spark  SPARK Max hardware object (must outlive this wrapper).
   * @param motor  DC motor model used for simulation.
   * @param config Initial SmartMotorControllerConfig to apply.
   */
  SparkWrapper(rev::spark::SparkMax& spark, frc::DCMotor motor,
               const SmartMotorControllerConfig& config);

  /**
   * Construct a SparkWrapper around a SPARK Flex.
   *
   * @param spark  SPARK Flex hardware object (must outlive this wrapper).
   * @param motor  DC motor model used for simulation.
   * @param config Initial SmartMotorControllerConfig to apply.
   */
  SparkWrapper(rev::spark::SparkFlex& spark, frc::DCMotor motor,
               const SmartMotorControllerConfig& config);

  // ---- Configuration ------------------------------------------------------
  bool ApplyConfig(const SmartMotorControllerConfig& config) override;

  // ---- Simulation ---------------------------------------------------------
  void SetupSimulation() override;
  void SimIterate() override;

  // ---- Encoder sync -------------------------------------------------------
  void SeedRelativeEncoder() override;
  void SynchronizeRelativeEncoder() override;

  // ---- Open-loop outputs --------------------------------------------------
  void SetDutyCycle(double dutyCycle) override;
  double GetDutyCycle() override;
  void SetVoltage(units::volt_t voltage) override;
  units::volt_t GetVoltage() override;

  // ---- Closed-loop setpoints ----------------------------------------------
  void SetPosition(units::degree_t angle) override;
  void SetPosition(units::meter_t distance) override;
  void SetVelocity(units::degrees_per_second_t velocity) override;
  void SetVelocity(units::meters_per_second_t velocity) override;

  // ---- Encoder writes -----------------------------------------------------
  void SetEncoderPosition(units::degree_t angle) override;
  void SetEncoderPosition(units::meter_t distance) override;
  void SetEncoderVelocity(units::degrees_per_second_t velocity) override;
  void SetEncoderVelocity(units::meters_per_second_t velocity) override;

  // ---- Encoder reads ------------------------------------------------------
  units::degree_t GetMechanismPosition() override;
  units::degrees_per_second_t GetMechanismVelocity() override;
  units::degrees_per_second_squared_t GetMechanismAcceleration() override;
  units::degree_t GetRotorPosition() override;
  units::degrees_per_second_t GetRotorVelocity() override;
  units::meter_t GetMeasurementPosition() override;
  units::meters_per_second_t GetMeasurementVelocity() override;
  units::meters_per_second_squared_t GetMeasurementAcceleration() override;

  std::optional<units::degree_t> GetExternalEncoderPosition() override;
  std::optional<units::degrees_per_second_t> GetExternalEncoderVelocity() override;

  // ---- Motor status -------------------------------------------------------
  std::optional<units::ampere_t> GetSupplyCurrent() override;
  units::ampere_t GetStatorCurrent() override;
  units::celsius_t GetTemperature() override;
  frc::DCMotor GetDCMotor() override;

  // ---- Configuration setters ----------------------------------------------
  void SetIdleMode(MotorMode mode) override;
  void SetMotorInverted(bool inverted) override;
  void SetEncoderInverted(bool inverted) override;
  void SetKp(double kP) override;
  void SetKi(double kI) override;
  void SetKd(double kD) override;
  void SetFeedback(double kP, double kI, double kD) override;
  void SetKs(double kS) override;
  void SetKv(double kV) override;
  void SetKa(double kA) override;
  void SetKg(double kG) override;
  void SetFeedforward(double kS, double kV, double kA, double kG) override;
  void SetStatorCurrentLimit(units::ampere_t currentLimit) override;
  void SetSupplyCurrentLimit(units::ampere_t currentLimit) override;
  void SetClosedLoopRampRate(units::second_t rampRate) override;
  void SetOpenLoopRampRate(units::second_t rampRate) override;
  void SetMechanismUpperLimit(units::degree_t upperLimit) override;
  void SetMechanismLowerLimit(units::degree_t lowerLimit) override;
  void SetMechanismLimits(units::degree_t lower, units::degree_t upper) override;
  void SetMechanismLimitsEnabled(bool enabled) override;
  void SetMeasurementUpperLimit(units::meter_t upperLimit) override;
  void SetMeasurementLowerLimit(units::meter_t lowerLimit) override;
  void SetMotionProfileMaxVelocity(units::degrees_per_second_t maxVelocity) override;
  void SetMotionProfileMaxVelocity(units::meters_per_second_t maxVelocity) override;
  void SetMotionProfileMaxAcceleration(units::degrees_per_second_squared_t maxAcc) override;
  void SetMotionProfileMaxAcceleration(units::meters_per_second_squared_t maxAcc) override;
  void SetMotionProfileMaxJerk(
      units::unit_t<units::compound_unit<units::angular_acceleration::degrees_per_second_squared,
                                         units::inverse<units::seconds>>>
          maxJerk) override;
  void SetExponentialProfile(std::optional<double> kV, std::optional<double> kA,
                             std::optional<units::volt_t> maxInput) override;
  void SetClosedLoopSlot(ClosedLoopControllerSlot slot) override;

  SmartMotorControllerConfig& GetConfig() override;
  void* GetMotorController() override;
  void* GetMotorControllerConfig() override;

 private:
  rev::spark::SparkBase* m_spark{nullptr};
  rev::spark::SparkClosedLoopController* m_sparkPid{nullptr};
  rev::spark::SparkRelativeEncoder* m_relEncoder{nullptr};
  rev::spark::SparkAbsoluteEncoder* m_absEncoder{nullptr};
  frc::DCMotor m_motor;

  // SparkMaxConfig and SparkFlexConfig are non-copyable/non-movable;
  // exactly one will be constructed via emplace().
  std::optional<rev::spark::SparkMaxConfig> m_maxConfig;
  std::optional<rev::spark::SparkFlexConfig> m_flexConfig;

  std::optional<frc::sim::DCMotorSim> m_motorSim;
  math::DerivativeTimeFilter m_accelFilter{20_ms};

  rev::spark::SparkLowLevel::ControlType m_positionControlType{
      rev::spark::SparkLowLevel::ControlType::kPosition};
  rev::spark::SparkLowLevel::ControlType m_velocityControlType{
      rev::spark::SparkLowLevel::ControlType::kVelocity};

  int m_revSlot{0};

  void Init(rev::spark::SparkBase* spark, frc::DCMotor motor,
            const SmartMotorControllerConfig& config);
  void ApplyBaseConfig();
  void CommitConfig();

  double GetRotorRotations() const;
  double GetRotorRPS() const;
  double GetMechRotations() const;
  double GetMechRPS() const;
};

}  // namespace yams::motorcontrollers::local
