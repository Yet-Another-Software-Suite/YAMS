// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

// Factory helpers used by all mechanism tests to create SmartMotorController
// instances for each (HardwareType × ProfileType) combination.

#include <frc/system/plant/DCMotor.h>
#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>

#include <atomic>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/TalonFXS.hpp>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include "TestSubsystem.h"
#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/motorcontrollers/SmartMotorControllerCommandRegistry.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/local/SparkWrapper.hpp"
#include "yams/motorcontrollers/remote/TalonFXSWrapper.hpp"
#include "yams/motorcontrollers/remote/TalonFXWrapper.hpp"

namespace yams::test {

using namespace motorcontrollers;

// Unique CAN IDs across all test instances, incremented atomically.
// Wraps within 1-62 (the valid Phoenix 6 / REV CAN device ID range).
inline std::atomic<int> gCanIdCounter{0};

inline int NextCanId() { return (gCanIdCounter.fetch_add(1) % 62) + 1; }

enum class HardwareType { SparkMax, SparkFlex, TalonFXS, TalonFX };
enum class ProfileType { None, Trapezoid, Exponential };

struct MotorTestParam {
  HardwareType hardware;
  ProfileType profile;
  std::string name;
};

// Lets gtest print human-readable param info in failure messages instead of a
// raw hex dump.
inline void PrintTo(const MotorTestParam& p, std::ostream* os) { *os << p.name; }

// Concrete hardware objects that must outlive the wrapper.
struct HardwareBundle {
  // Only one of these is non-null per bundle.
  std::unique_ptr<rev::spark::SparkMax> sparkMax;
  std::unique_ptr<rev::spark::SparkFlex> sparkFlex;
  std::unique_ptr<ctre::phoenix6::hardware::TalonFXS> talonFXS;
  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> talonFX;

  // Config must outlive the wrapper; stored here so its address stays stable.
  SmartMotorControllerConfig cfg;
  std::unique_ptr<TestSubsystem> subsystem;
  SmartMotorController* smc{nullptr};
};

// Build a base SMC config – callers supply the mechanism-specific parts
// (gearing, circumference, feedforward).
inline SmartMotorControllerConfig MakeBaseConfig(ProfileType profile, double kP,
                                                 const gearing::MechanismGearing& gearing,
                                                 TestSubsystem* subsys,
                                                 const std::string& telemetryName) {
  SmartMotorControllerConfig cfg;
  cfg.WithFeedback(kP, 0.0, 0.0)
      .WithMotorGearing(gearing)
      .WithIdleMode(SmartMotorControllerConfig::MotorMode::BRAKE)
      .WithMotorInverted(false)
      .WithClosedLoopMode()
      .WithSubsystem(subsys)
      .WithTelemetry(telemetryName);
  return cfg;
}

// Return the canonical DCMotor model for a given hardware type.
inline frc::DCMotor MotorForHardware(HardwareType hw) {
  switch (hw) {
    case HardwareType::SparkMax:
      return frc::DCMotor::NEO(1);
    case HardwareType::SparkFlex:
      return frc::DCMotor::NeoVortex(1);
    case HardwareType::TalonFXS:
      return frc::DCMotor::NEO(2);
    case HardwareType::TalonFX:
      return frc::DCMotor::KrakenX60(1);
  }
  return frc::DCMotor::NEO(1);
}

// Create a bundle (hardware + subsystem + wrapper) for a given parameter set.
inline HardwareBundle MakeBundle(const MotorTestParam& param, SmartMotorControllerConfig cfg) {
  HardwareBundle bundle;
  bundle.subsystem = std::make_unique<TestSubsystem>();
  cfg.WithSubsystem(bundle.subsystem.get());
  cfg.WithSimMotor(MotorForHardware(param.hardware));

  // Move the fully-configured cfg into the bundle so its address is stable for
  // the lifetime of the bundle (the wrapper stores a SmartMotorControllerConfig*).
  bundle.cfg = std::move(cfg);

  int canId = NextCanId();

  switch (param.hardware) {
    case HardwareType::SparkMax: {
      bundle.sparkMax = std::make_unique<rev::spark::SparkMax>(
          canId, rev::spark::SparkLowLevel::MotorType::kBrushless);
      bundle.smc = new local::SparkWrapper(bundle.sparkMax.get(), MotorForHardware(param.hardware),
                                           &bundle.cfg);
      break;
    }
    case HardwareType::SparkFlex: {
      bundle.sparkFlex = std::make_unique<rev::spark::SparkFlex>(
          canId, rev::spark::SparkLowLevel::MotorType::kBrushless);
      bundle.smc = new local::SparkWrapper(bundle.sparkFlex.get(), MotorForHardware(param.hardware),
                                           &bundle.cfg);
      break;
    }
    case HardwareType::TalonFXS: {
      bundle.talonFXS = std::make_unique<ctre::phoenix6::hardware::TalonFXS>(canId);
      bundle.smc =
          new remote::TalonFXSWrapper(bundle.talonFXS.get(), MotorForHardware(param.hardware),
                                      remote::TalonFXSWrapper::MotorArrangement::NEO, &bundle.cfg);
      break;
    }
    case HardwareType::TalonFX: {
      bundle.talonFX = std::make_unique<ctre::phoenix6::hardware::TalonFX>(canId);
      bundle.smc = new remote::TalonFXWrapper(bundle.talonFX.get(),
                                              MotorForHardware(param.hardware), &bundle.cfg);
      break;
    }
  }

  bundle.subsystem->SetSMC(bundle.smc);
  return bundle;
}

// True if the bundle wraps a CTRE TalonFX or TalonFXS.
inline bool IsCTRE(const HardwareBundle& b) {
  return b.talonFX != nullptr || b.talonFXS != nullptr;
}

// Standard teardown: unregister subsystem, close SMC, delete wrapper.
inline void CloseBundle(HardwareBundle& b) {
  motorcontrollers::SmartMotorControllerCommandRegistry::RemoveCommands(b.subsystem.get());
  frc2::CommandScheduler::GetInstance().UnregisterSubsystem(b.subsystem.get());
  b.subsystem->Close();
  delete b.smc;
  b.smc = nullptr;
}

// All (hardware × profile) combinations used by each mechanism test suite.
inline std::vector<MotorTestParam> AllMotorParams() {
  std::vector<MotorTestParam> params;
  for (auto hw : {HardwareType::SparkMax, HardwareType::SparkFlex, HardwareType::TalonFXS,
                  HardwareType::TalonFX}) {
    for (auto prof : {ProfileType::None, ProfileType::Trapezoid, ProfileType::Exponential}) {
      std::string hwName;
      switch (hw) {
        case HardwareType::SparkMax:
          hwName = "SparkMax";
          break;
        case HardwareType::SparkFlex:
          hwName = "SparkFlex";
          break;
        case HardwareType::TalonFXS:
          hwName = "TalonFXS";
          break;
        case HardwareType::TalonFX:
          hwName = "TalonFX";
          break;
      }
      std::string profName;
      switch (prof) {
        case ProfileType::None:
          profName = "NoPro";
          break;
        case ProfileType::Trapezoid:
          profName = "Trap";
          break;
        case ProfileType::Exponential:
          profName = "Expo";
          break;
      }
      params.push_back({hw, prof, hwName + "_" + profName});
    }
  }
  return params;
}

}  // namespace yams::test
