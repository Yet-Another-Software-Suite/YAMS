// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

// Phoenix 6 devices shared by the TalonFX/TalonFXS wrapper tests. Phoenix's simulation crashes if
// Talons are destroyed and recreated while it runs, so the devices are created once for the whole
// test binary (and never destroyed) on CAN IDs no other test uses (Phoenix's simulation tells
// devices apart by type and ID only, not by bus; see kTalonTestCanIdStart). Each test makes its
// own wrappers around them and resets the devices to their default configuration first.

#include <chrono>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/CANdi.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/TalonFXS.hpp>
#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include <functional>
#include <string>
#include <thread>
#include <wpi/simulation/AlertSim.hpp>

#include "MotorControllerFactory.h"

namespace yams::test {

struct TalonTestHardware {
  ctre::phoenix6::CANBus bus{};
  ctre::phoenix6::hardware::TalonFX fx{kTalonTestCanIdStart, bus};
  ctre::phoenix6::hardware::TalonFX fxFollower{kTalonTestCanIdStart + 1, bus};
  ctre::phoenix6::hardware::TalonFXS fxs{kTalonTestCanIdStart + 2, bus};
  ctre::phoenix6::hardware::CANcoder cancoder{kTalonTestCanIdStart, bus};
  ctre::phoenix6::hardware::CANdi candi{kTalonTestCanIdStart, bus};

  /** Restore every device's default configuration and neutral output. */
  void Reset() {
    // A previous test's simulation may have left a device without supply voltage, which stops it
    // answering on the bus.
    fx.GetSimState().SetSupplyVoltage(12_V);
    fxFollower.GetSimState().SetSupplyVoltage(12_V);
    fxs.GetSimState().SetSupplyVoltage(12_V);
    cancoder.GetSimState().SetSupplyVoltage(12_V);
    candi.GetSimState().SetSupplyVoltage(12_V);
    ApplyDefaults(fx, ctre::phoenix6::configs::TalonFXConfiguration{});
    ApplyDefaults(fxFollower, ctre::phoenix6::configs::TalonFXConfiguration{});
    ApplyDefaults(fxs, ctre::phoenix6::configs::TalonFXSConfiguration{});
    ApplyDefaults(cancoder, ctre::phoenix6::configs::CANcoderConfiguration{});
    ApplyDefaults(candi, ctre::phoenix6::configs::CANdiConfiguration{});
    fx.SetControl(ctre::phoenix6::controls::NeutralOut{});
    fxFollower.SetControl(ctre::phoenix6::controls::NeutralOut{});
    fxs.SetControl(ctre::phoenix6::controls::NeutralOut{});
  }

 private:
  // The simulated devices occasionally miss a config frame while the whole suite runs; retry.
  template <typename Device, typename Config>
  static void ApplyDefaults(Device& device, const Config& config) {
    for (int i = 0; i < 10 && !device.GetConfigurator().Apply(config, 1_s).IsOK(); i++) {
    }
  }
};

/** The shared devices, created on first use and intentionally never destroyed. */
inline TalonTestHardware& TalonHardware() {
  static auto* hardware = new TalonTestHardware();
  return *hardware;
}

inline ctre::phoenix6::configs::TalonFXConfiguration ReadConfig(
    ctre::phoenix6::hardware::TalonFX& talon) {
  ctre::phoenix6::configs::TalonFXConfiguration cfg;
  for (int i = 0; i < 10 && !talon.GetConfigurator().Refresh(cfg, 1_s).IsOK(); i++) {
  }
  return cfg;
}

inline ctre::phoenix6::configs::TalonFXSConfiguration ReadConfig(
    ctre::phoenix6::hardware::TalonFXS& talon) {
  ctre::phoenix6::configs::TalonFXSConfiguration cfg;
  for (int i = 0; i < 10 && !talon.GetConfigurator().Refresh(cfg, 1_s).IsOK(); i++) {
  }
  return cfg;
}

/** Name of the control request last sent to @p device. */
template <typename Device>
std::string AppliedControlName(Device& device) {
  auto control = device.GetAppliedControl();
  return control ? std::string{control->GetName()} : std::string{};
}

/** Value of a field of the control request last sent to @p device (empty if absent). */
template <typename Device>
std::string AppliedControlField(Device& device, std::string_view field) {
  auto control = device.GetAppliedControl();
  if (!control) return {};
  auto info = control->GetControlInfo();
  auto it = info.find(field);
  return it == info.end() ? std::string{} : it->second;
}

/** Whether an active alert's text contains @p text. */
inline bool AlertActive(const std::string& text) {
  for (auto& alert : wpi::sim::AlertSim::GetActive()) {
    if (alert.text.find(text) != std::string::npos) return true;
  }
  return false;
}

/** Poll @p condition for up to a second (status signals update asynchronously). */
inline bool WaitFor(const std::function<bool()>& condition) {
  for (int i = 0; i < 100; i++) {
    if (condition()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return condition();
}

}  // namespace yams::test
