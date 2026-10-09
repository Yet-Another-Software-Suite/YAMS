// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Tests for TalonFXWrapper / TalonFXSWrapper API ported from the Java wrappers (FOC, CANdi PWM
// selection, status signal update frequency, forced config apply, motor arrangement from the
// DCMotor) and the config's linear exponential profile setter.

#include <any>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/units/frequency.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/voltage.hpp>

#include "helpers/MockHardware.h"
#include "helpers/TalonTestHardware.h"
#include "yams/exceptions.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/motorcontrollers/remote/TalonFXSWrapper.hpp"
#include "yams/motorcontrollers/remote/TalonFXWrapper.hpp"

namespace yams::test {

namespace {

using motorcontrollers::remote::TalonFXSWrapper;
using motorcontrollers::remote::TalonFXWrapper;
using ConfigException = exceptions::SmartMotorControllerConfigurationException;
using motorcontrollers::SmartMotorControllerConfig;
using namespace ctre::phoenix6;

SmartMotorControllerConfig ApiConfig() {
  SmartMotorControllerConfig cfg;
  cfg.WithMotorGearing(gearing::MechanismGearing{10.0})
      .WithStatorCurrentLimit(40_A)
      .WithFeedback(1.0, 0.0, 0.0)
      .WithClosedLoopMode();
  return cfg;
}

struct TalonApiFixture {
  TalonApiFixture() {
    InitializeHardware();
    TalonHardware().Reset();
  }
  ~TalonApiFixture() { TeardownHardware(); }

  TalonTestHardware& hw = TalonHardware();
};

}  // namespace

TEST_CASE_METHOD(TalonApiFixture, "TalonApi.FOC", "[TalonApi]") {
  {
    auto cfg = ApiConfig();
    TalonFXWrapper smc{&hw.fx, wpi::math::DCMotor::KrakenX60(1), &cfg};
    smc.DisableFOC();
    smc.SetPosition(0.5_tr);
    CHECK(AppliedControlField(hw.fx, "EnableFOC") == "0");
    smc.EnableFOC();
    smc.SetVelocity(1_tps);
    CHECK(AppliedControlField(hw.fx, "EnableFOC") == "1");
  }
  {
    // TorqueCurrentFOC requests always use FOC.
    auto cfg = ApiConfig();
    cfg.WithVendorControlRequest(controls::PositionTorqueCurrentFOC{0_tr});
    TalonFXSWrapper smc{&hw.fxs, wpi::math::DCMotor::NEO(1), &cfg};
    CHECK_THROWS_AS(smc.EnableFOC(), ConfigException);
  }
}

TEST_CASE_METHOD(TalonApiFixture, "TalonApi.CANdiPWMSelection", "[TalonApi]") {
  configs::TalonFXConfiguration vendor;
  vendor.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::SyncCANdiPWM1;
  auto cfg = ApiConfig();
  cfg.WithVendorConfig(vendor).WithExternalEncoder(std::any{&hw.candi});
  TalonFXWrapper smc{&hw.fx, wpi::math::DCMotor::KrakenX60(1), &cfg};
  CHECK(smc.UseCANdiPWM1());
  CHECK_FALSE(smc.UseCANdiPWM2());
  CHECK(ReadConfig(hw.fx).Feedback.FeedbackSensorSource ==
        signals::FeedbackSensorSourceValue::FusedCANdiPWM1);
  // The simulated CANdi reports the mechanism position on PWM1.
  smc.GetSimSupplier()->SetMechanismPosition(0.25_tr);
  smc.SimIterate();
  CHECK(WaitFor([&] {
    auto position = smc.GetExternalEncoderPosition();
    return position && std::abs(wpi::units::turn_t{*position}.value() - 0.25) < 0.02;
  }));
}

TEST_CASE_METHOD(TalonApiFixture, "TalonApi.UpdateFrequencyAndForceConfigApply", "[TalonApi]") {
  auto cfg = ApiConfig();
  TalonFXWrapper smc{&hw.fx, wpi::math::DCMotor::KrakenX60(1), &cfg};
  smc.SetUpdateFrequency(250_Hz);
  CHECK(hw.fx.GetVelocity(false).GetAppliedUpdateFrequency().value() == Catch::Approx(250.0));
  CHECK(smc.ForceConfigApply().IsOK());
}

TEST_CASE_METHOD(TalonApiFixture, "TalonApi.TalonFXSArrangementFromMotor", "[TalonApi]") {
  {
    auto cfg = ApiConfig();
    TalonFXSWrapper smc{&hw.fxs, wpi::math::DCMotor::Minion(1), &cfg};
    auto device = ReadConfig(hw.fxs);
    CHECK(device.Commutation.MotorArrangement == signals::MotorArrangementValue::Minion_JST);
    CHECK(device.Commutation.AdvancedHallSupport == signals::AdvancedHallSupportValue::Enabled);
  }
  {
    auto cfg = ApiConfig();
    TalonFXSWrapper smc{&hw.fxs, wpi::math::DCMotor::NeoVortex(1), &cfg};
    CHECK(ReadConfig(hw.fxs).Commutation.MotorArrangement ==
          signals::MotorArrangementValue::VORTEX_JST);
  }
  {
    auto cfg = ApiConfig();
    CHECK_THROWS_AS((TalonFXSWrapper{&hw.fxs, wpi::math::DCMotor::KrakenX60(1), &cfg}),
                    std::invalid_argument);
  }
}

TEST_CASE("TalonApi.LinearExponentialProfileSetter", "[TalonApi]") {
  SmartMotorControllerConfig cfg;
  CHECK_THROWS_AS(cfg.WithLinearExponentialProfile(1.0, 0.1, 12_V), ConfigException);
  cfg.WithMechanismCircumference(0.2_m).WithLinearExponentialProfile(1.0, 0.1, 12_V);
  CHECK(cfg.HasLinearExponentialProfile());
  CHECK(cfg.GetLinearClosedLoopControllerUse());
  // Per mechanism rotation: 0.2 m per rotation.
  CHECK(cfg.GetExponentialProfileKV().value() == Catch::Approx(0.2));
  CHECK(cfg.GetExponentialProfileKA().value() == Catch::Approx(0.02));
}

}  // namespace yams::test
