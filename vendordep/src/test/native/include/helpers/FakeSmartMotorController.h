// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

// A hardware-free SmartMotorController for exercising the base class logic (RoboRIO closed-loop
// controller, telemetry, close hooks) deterministically. Sensor state is set directly by the
// test; outputs are recorded.

#include <memory>
#include <optional>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/system/Notifier.hpp>

#include "yams/motorcontrollers/SmartMotorController.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"

namespace yams::test {

using motorcontrollers::SmartMotorController;
using motorcontrollers::SmartMotorControllerConfig;

namespace detail {
// The base class holds the config the wrappers use as a pointer.
inline void AssignBaseConfig(SmartMotorControllerConfig*& member, SmartMotorControllerConfig* cfg) {
  member = cfg;
}
// Older base classes held a config value.
inline void AssignBaseConfig(SmartMotorControllerConfig& member, SmartMotorControllerConfig* cfg) {
  member = *cfg;
}
}  // namespace detail

class FakeSmartMotorController : public SmartMotorController {
 public:
  using SmartMotorController::SetVelocity;

  explicit FakeSmartMotorController(SmartMotorControllerConfig* cfg) : m_cfg(cfg) {
    detail::AssignBaseConfig(m_config, cfg);
  }

  // ---- Test controls ------------------------------------------------------

  /** Create the software PID from the slot 0 gains, as the wrappers do. */
  void CreatePID() {
    auto gains = m_cfg->GetSlotGains(ClosedLoopControllerSlot::SLOT_0);
    m_pid = wpi::math::PIDController{gains.kP, gains.kI, gains.kD};
  }

  /** Mark the RoboRIO closed-loop controller running without starting its thread. */
  void SetRunning(bool running) { m_closedLoopControllerRunning = running; }

  /** Give the controller a no-op closed-loop thread so StartClosedLoopController() works. */
  void CreateIdleClosedLoopThread() {
    m_closedLoopControllerThread = std::make_unique<wpi::Notifier>([] {});
  }

  std::optional<wpi::math::TrapezoidProfile<wpi::units::meters>::State> LinearTrapState() const {
    return m_linearTrapState;
  }

  wpi::units::turn_t pos{0};
  wpi::units::turns_per_second_t vel{0};
  wpi::units::volt_t lastVoltage{0};
  double dutyCycle{0.0};
  int setKpCalls{0};
  int synchronizeCalls{0};

  // ---- SmartMotorController -----------------------------------------------

  bool ApplyConfig(const SmartMotorControllerConfig&) override { return true; }
  void SetupSimulation() override {}
  void SimIterate() override {}
  void SeedRelativeEncoder() override {}
  void SynchronizeRelativeEncoder() override { ++synchronizeCalls; }
  void SetDutyCycle(double dc) override { dutyCycle = dc; }
  double GetDutyCycle() override { return dutyCycle; }
  void SetVoltage(wpi::units::volt_t v) override { lastVoltage = v; }
  wpi::units::volt_t GetVoltage() override { return lastVoltage; }
  void SetPosition(wpi::units::turn_t angle) override {
    m_setpointPosition = angle;
    m_setpointVelocity.reset();
  }
  void SetPosition(wpi::units::meter_t distance) override {
    SetPosition(wpi::units::turn_t{distance.value() / Circumference()});
  }
  void SetVelocity(wpi::units::turns_per_second_t velocity) override {
    m_setpointVelocity = velocity;
    m_setpointPosition.reset();
  }
  void SetVelocity(wpi::units::meters_per_second_t velocity) override {
    SetVelocity(wpi::units::turns_per_second_t{velocity.value() / Circumference()});
  }
  void SetEncoderPosition(wpi::units::turn_t angle) override { pos = angle; }
  void SetEncoderPosition(wpi::units::meter_t distance) override {
    pos = wpi::units::turn_t{distance.value() / Circumference()};
  }
  void SetEncoderVelocity(wpi::units::turns_per_second_t velocity) override { vel = velocity; }
  void SetEncoderVelocity(wpi::units::meters_per_second_t velocity) override {
    vel = wpi::units::turns_per_second_t{velocity.value() / Circumference()};
  }
  wpi::units::turn_t GetMechanismPosition() override { return pos; }
  wpi::units::turns_per_second_t GetMechanismVelocity() override { return vel; }
  wpi::units::turn_t GetRelativeMechanismPosition() override { return pos; }
  wpi::units::turns_per_second_t GetRelativeMechanismVelocity() override { return vel; }
  wpi::units::turns_per_second_squared_t GetMechanismAcceleration() override {
    return wpi::units::turns_per_second_squared_t{0};
  }
  wpi::units::turn_t GetRotorPosition() override { return pos; }
  wpi::units::turns_per_second_t GetRotorVelocity() override { return vel; }
  wpi::units::meter_t GetMeasurementPosition() override {
    return wpi::units::meter_t{pos.value() * Circumference()};
  }
  wpi::units::meters_per_second_t GetMeasurementVelocity() override {
    return wpi::units::meters_per_second_t{vel.value() * Circumference()};
  }
  wpi::units::meters_per_second_squared_t GetMeasurementAcceleration() override {
    return wpi::units::meters_per_second_squared_t{0};
  }
  std::optional<wpi::units::turn_t> GetExternalEncoderMechanismPosition() override {
    return std::nullopt;
  }
  std::optional<wpi::units::turns_per_second_t> GetExternalEncoderMechanismVelocity() override {
    return std::nullopt;
  }
  std::optional<wpi::units::ampere_t> GetSupplyCurrent() override {
    return wpi::units::ampere_t{0};
  }
  wpi::units::ampere_t GetStatorCurrent() override { return wpi::units::ampere_t{0}; }
  wpi::units::celsius_t GetTemperature() override { return wpi::units::celsius_t{25}; }
  wpi::math::DCMotor GetDCMotor() override { return wpi::math::DCMotor::KrakenX60(1); }
  void SetZeroPower(MotorMode) override {}
  void SetMotorInverted(bool) override {}
  void SetEncoderInverted(bool) override {}
  void SetKp(double) override { ++setKpCalls; }
  void SetKi(double) override {}
  void SetKd(double) override {}
  void SetFeedback(double, double, double) override {}
  void SetKs(double) override {}
  void SetKv(double) override {}
  void SetKa(double) override {}
  void SetKg(double) override {}
  void SetFeedforward(double, double, double, double) override {}
  void SetStatorCurrentLimit(wpi::units::ampere_t) override {}
  void SetSupplyCurrentLimit(wpi::units::ampere_t) override {}
  void SetClosedLoopRampRate(wpi::units::second_t) override {}
  void SetOpenLoopRampRate(wpi::units::second_t) override {}
  void SetMechanismUpperLimit(wpi::units::turn_t) override {}
  void SetMechanismLowerLimit(wpi::units::turn_t) override {}
  void SetMechanismLimits(wpi::units::turn_t, wpi::units::turn_t) override {}
  void SetMechanismLimitsEnabled(bool) override {}
  void SetMeasurementUpperLimit(wpi::units::meter_t) override {}
  void SetMeasurementLowerLimit(wpi::units::meter_t) override {}
  void SetMotionProfileMaxVelocity(wpi::units::turns_per_second_t) override {}
  void SetMotionProfileMaxVelocity(wpi::units::meters_per_second_t) override {}
  void SetMotionProfileMaxAcceleration(wpi::units::turns_per_second_squared_t) override {}
  void SetMotionProfileMaxAcceleration(wpi::units::meters_per_second_squared_t) override {}
  void SetMotionProfileMaxJerk(wpi::units::angular_jerk::turns_per_second_cubed_t) override {}
  void SetExponentialProfile(std::optional<double>, std::optional<double>,
                             std::optional<wpi::units::volt_t>) override {}
  void SetClosedLoopSlot(ClosedLoopControllerSlot slot) override { m_slot = slot; }
  SmartMotorControllerConfig& GetConfig() override { return *m_cfg; }
  void* GetMotorController() override { return nullptr; }
  void* GetMotorControllerConfig() override { return nullptr; }

 private:
  double Circumference() const {
    auto circ = m_cfg->GetMechanismCircumference();
    return circ ? circ->value() : 1.0;
  }

  SmartMotorControllerConfig* m_cfg;
};

}  // namespace yams::test
