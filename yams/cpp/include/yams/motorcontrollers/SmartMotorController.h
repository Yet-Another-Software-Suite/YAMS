// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/Notifier.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/trajectory/ExponentialProfile.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <networktables/NetworkTable.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/temperature.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "SmartMotorControllerConfig.h"
#include "yams/math/LQRController.h"

namespace yams::motorcontrollers {

class SmartMotorController {
 public:
  using ClosedLoopControllerSlot = SmartMotorControllerConfig::ClosedLoopControllerSlot;
  using MotorMode = SmartMotorControllerConfig::MotorMode;
  using ControlMode = SmartMotorControllerConfig::ControlMode;

  virtual ~SmartMotorController() = default;

  // ---- Configuration ------------------------------------------------------
  virtual bool ApplyConfig(const SmartMotorControllerConfig& config) = 0;

  // ---- Simulation ---------------------------------------------------------
  virtual void SetupSimulation() = 0;
  virtual void SimIterate() = 0;

  // ---- Encoder sync -------------------------------------------------------
  virtual void SeedRelativeEncoder() = 0;
  virtual void SynchronizeRelativeEncoder() = 0;

  // ---- Open-loop outputs --------------------------------------------------
  virtual void SetDutyCycle(double dutyCycle) = 0;
  virtual double GetDutyCycle() = 0;
  virtual void SetVoltage(units::volt_t voltage) = 0;
  virtual units::volt_t GetVoltage() = 0;

  // ---- Closed-loop setpoints ----------------------------------------------
  virtual void SetPosition(units::degree_t angle) = 0;
  virtual void SetPosition(units::meter_t distance) = 0;
  virtual void SetVelocity(units::degrees_per_second_t velocity) = 0;
  virtual void SetVelocity(units::meters_per_second_t velocity) = 0;

  // ---- Encoder writes -----------------------------------------------------
  virtual void SetEncoderPosition(units::degree_t angle) = 0;
  virtual void SetEncoderPosition(units::meter_t distance) = 0;
  virtual void SetEncoderVelocity(units::degrees_per_second_t velocity) = 0;
  virtual void SetEncoderVelocity(units::meters_per_second_t velocity) = 0;

  // ---- Encoder reads ------------------------------------------------------
  virtual units::degree_t GetMechanismPosition() = 0;
  virtual units::degrees_per_second_t GetMechanismVelocity() = 0;
  virtual units::degrees_per_second_squared_t GetMechanismAcceleration() = 0;
  virtual units::degree_t GetRotorPosition() = 0;
  virtual units::degrees_per_second_t GetRotorVelocity() = 0;
  virtual units::meter_t GetMeasurementPosition() = 0;
  virtual units::meters_per_second_t GetMeasurementVelocity() = 0;
  virtual units::meters_per_second_squared_t GetMeasurementAcceleration() = 0;

  virtual std::optional<units::degree_t> GetExternalEncoderPosition() = 0;
  virtual std::optional<units::degrees_per_second_t> GetExternalEncoderVelocity() = 0;

  // ---- Motor status -------------------------------------------------------
  virtual std::optional<units::ampere_t> GetSupplyCurrent() = 0;
  virtual units::ampere_t GetStatorCurrent() = 0;
  virtual units::celsius_t GetTemperature() = 0;
  virtual frc::DCMotor GetDCMotor() = 0;

  // ---- Configuration setters (live tuning) --------------------------------
  virtual void SetIdleMode(MotorMode mode) = 0;
  virtual void SetMotorInverted(bool inverted) = 0;
  virtual void SetEncoderInverted(bool inverted) = 0;
  virtual void SetKp(double kP) = 0;
  virtual void SetKi(double kI) = 0;
  virtual void SetKd(double kD) = 0;
  virtual void SetFeedback(double kP, double kI, double kD) = 0;
  virtual void SetKs(double kS) = 0;
  virtual void SetKv(double kV) = 0;
  virtual void SetKa(double kA) = 0;
  virtual void SetKg(double kG) = 0;
  virtual void SetFeedforward(double kS, double kV, double kA, double kG) = 0;
  virtual void SetStatorCurrentLimit(units::ampere_t currentLimit) = 0;
  virtual void SetSupplyCurrentLimit(units::ampere_t currentLimit) = 0;
  virtual void SetClosedLoopRampRate(units::second_t rampRate) = 0;
  virtual void SetOpenLoopRampRate(units::second_t rampRate) = 0;
  virtual void SetMechanismUpperLimit(units::degree_t upperLimit) = 0;
  virtual void SetMechanismLowerLimit(units::degree_t lowerLimit) = 0;
  virtual void SetMechanismLimits(units::degree_t lower, units::degree_t upper) = 0;
  virtual void SetMechanismLimitsEnabled(bool enabled) = 0;
  virtual void SetMeasurementUpperLimit(units::meter_t upperLimit) = 0;
  virtual void SetMeasurementLowerLimit(units::meter_t lowerLimit) = 0;
  virtual void SetMotionProfileMaxVelocity(units::degrees_per_second_t maxVelocity) = 0;
  virtual void SetMotionProfileMaxVelocity(units::meters_per_second_t maxVelocity) = 0;
  virtual void SetMotionProfileMaxAcceleration(units::degrees_per_second_squared_t maxAcc) = 0;
  virtual void SetMotionProfileMaxAcceleration(units::meters_per_second_squared_t maxAcc) = 0;
  virtual void SetMotionProfileMaxJerk(
      units::unit_t<units::compound_unit<units::angular_acceleration::degrees_per_second_squared,
                                         units::inverse<units::seconds>>>
          maxJerk) = 0;
  virtual void SetExponentialProfile(std::optional<double> kV, std::optional<double> kA,
                                     std::optional<units::volt_t> maxInput) = 0;
  virtual void SetClosedLoopSlot(ClosedLoopControllerSlot slot) = 0;

  // ---- Closed-loop controller thread --------------------------------------
  void StartClosedLoopController();
  void StopClosedLoopController();
  void IterateClosedLoopController();

  // ---- Telemetry ----------------------------------------------------------
  void SetupTelemetry(std::shared_ptr<nt::NetworkTable> dataTable,
                      std::shared_ptr<nt::NetworkTable> tuningTable);
  void SetupTelemetry();
  void UpdateTelemetry();

  // ---- SysId --------------------------------------------------------------
  frc2::sysid::SysIdRoutine SysId(units::volt_t maxVoltage, frc2::sysid::ramp_rate_t stepVoltage,
                                  units::second_t testDuration);

  // ---- Misc ---------------------------------------------------------------
  std::optional<units::degree_t> GetMechanismPositionSetpoint() const;
  std::optional<units::degrees_per_second_t> GetMechanismSetpointVelocity() const;

  virtual SmartMotorControllerConfig& GetConfig() = 0;
  virtual void* GetMotorController() = 0;
  virtual void* GetMotorControllerConfig() = 0;

  std::string GetName() const;
  bool IsMotor(const frc::DCMotor& a, const frc::DCMotor& b) const;
  void CheckConfigSafety();

  void Close();

 protected:
  SmartMotorControllerConfig m_config;
  ClosedLoopControllerSlot m_slot{ClosedLoopControllerSlot::SLOT_0};

  std::optional<frc::PIDController> m_pid;
  std::optional<math::LQRController> m_lqr;

  // Angular motion profile state
  std::optional<frc::TrapezoidProfile<units::turns>::State> m_trapState;
  std::optional<frc::ExponentialProfile<units::turns, units::volts>::State> m_expoState;

  // Linear motion profile state
  std::optional<frc::TrapezoidProfile<units::meters>::State> m_linearTrapState;

  std::optional<units::degree_t> m_setpointPosition;
  std::optional<units::degrees_per_second_t> m_setpointVelocity;

  std::unique_ptr<frc::Notifier> m_closedLoopControllerThread;
  bool m_closedLoopControllerRunning{false};

  std::shared_ptr<nt::NetworkTable> m_parentTable;
  std::shared_ptr<nt::NetworkTable> m_telemetryTable;
  std::shared_ptr<nt::NetworkTable> m_tuningTable;

 private:
  std::optional<frc::TrapezoidProfile<units::turns>::State> GetTrapezoidalProfileState();
  std::optional<frc::ExponentialProfile<units::turns, units::volts>::State>
  GetExponentialProfileState();
};

}  // namespace yams::motorcontrollers
