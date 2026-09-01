// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

// Mirrors Java SmartMotorControllerTestSubsystem — a SubsystemBase that owns
// a SmartMotorController reference and wires up simulation/telemetry
// periodics for use in integration tests.

#include <wpi/commands2/Commands.hpp>
#include <wpi/commands2/SubsystemBase.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/length.hpp>
#include <functional>

#include "yams/motorcontrollers/SmartMotorController.hpp"

namespace yams::test {

class TestSubsystem : public wpi::cmd::SubsystemBase {
 public:
  TestSubsystem() = default;

  void SetSMC(motorcontrollers::SmartMotorController* smc) { m_smc = smc; }

  wpi::cmd::CommandPtr SetDutyCycle(double dutyCycle) {
    return StartRun([this] { m_smc->StopClosedLoopController(); },
                    [this, dutyCycle] { m_smc->SetDutyCycle(dutyCycle); })
        .FinallyDo([this](bool) { m_smc->StartClosedLoopController(); });
  }

  wpi::cmd::CommandPtr SetPositionSetpoint(wpi::units::degree_t position) {
    return Run([this, position] { m_smc->SetPosition(position); });
  }

  wpi::cmd::CommandPtr SetPositionSetpoint(wpi::units::meter_t position) {
    return Run([this, position] { m_smc->SetPosition(position); });
  }

  void Close() { m_smc->Close(); }

  void Periodic() override {
    if (m_testRunning) {
      if (m_mechUpdateTelemetry)
        m_mechUpdateTelemetry();
      else
        m_smc->UpdateTelemetry();
    }
  }

  void SimulationPeriodic() override {
    if (m_testRunning) {
      if (m_mechSimPeriodic)
        m_mechSimPeriodic();
      else
        m_smc->SimIterate();
    }
  }

  motorcontrollers::SmartMotorController* m_smc{nullptr};
  std::function<void()> m_mechSimPeriodic;
  std::function<void()> m_mechUpdateTelemetry;
  bool m_testRunning{false};
};

}  // namespace yams::test
