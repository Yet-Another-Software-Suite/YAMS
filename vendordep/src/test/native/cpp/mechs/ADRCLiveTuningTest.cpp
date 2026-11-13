// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Live tuning an ADRC on every motor controller: the tuning table shows what the ADRC runs with, the entered values
// are kept in the motor config's ADRC config and run by the ADRC, applying the motor config again keeps them, and a
// value the ADRC can't use leaves it as it was. Mirrors the Java ADRCLiveTuningTest. The ADRC was written by FRC
// Team 8331.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <memory>
#include <string>
#include <vector>

#include <wpi/nt/NetworkTable.hpp>
#include <wpi/nt/NetworkTableInstance.hpp>
#include <wpi/units/moment_of_inertia.hpp>

#include "helpers/MockHardware.h"
#include "helpers/MotorControllerFactory.h"
#include "helpers/SchedulerHelper.h"
#include "yams/math/ADRCConfig.hpp"
#include "yams/math/ADRCController.hpp"

namespace yams::test {

using namespace motorcontrollers;

namespace {
/** Every motor controller once, without a motion profile. */
std::vector<MotorTestParam> Controllers() {
  std::vector<MotorTestParam> params;
  for (auto& param : AllMotorParams()) {
    if (param.profile == ProfileType::None || (param.hardware == HardwareType::Nova &&
                                               param.profile == ProfileType::Trapezoid)) {
      params.push_back({param.hardware, ProfileType::None, param.name});
    }
  }
  return params;
}

struct ADRCLiveTuningFixture {
  ADRCLiveTuningFixture() {
    InitializeHardware();
    SchedulerHelper::Enable();
    SchedulerHelper::CancelAll();
  }
  ~ADRCLiveTuningFixture() {
    SchedulerHelper::CancelAll();
    TeardownHardware();
  }
};

/** The ADRC's tuning, as the controller runs it. */
void CheckRunning(const math::ADRCController& adrc, double b0, double settlingTime, double bandwidthScale) {
  CHECK(adrc.GetB0() == Catch::Approx(b0));
  CHECK(adrc.GetSettlingTime().value() == Catch::Approx(settlingTime));
  CHECK(adrc.GetBandwidthScale() == Catch::Approx(bandwidthScale));
}
}  // namespace

TEST_CASE_METHOD(ADRCLiveTuningFixture, "ADRCLiveTuning.TuningIsKeptInTheConfigAndFollowedByTheADRC",
                 "[ADRCLiveTuning]") {
  for (const auto& param : Controllers()) {
    DYNAMIC_SECTION(param.name) {
      const std::string name = "ADRCLiveTuning" + param.name;
      SmartMotorControllerConfig cfg;
      cfg.WithMotorGearing(gearing::MechanismGearing{gearing::GearBox::FromReductionStages({3.0, 4.0})})
          .WithZeroPower(SmartMotorControllerConfig::MotorMode::BRAKE)
          .WithClosedLoopMode()
          .WithMOI(wpi::units::kilogram_square_meter_t{0.1})
          .WithStartingPosition(0.0_deg)
          .WithTelemetry(name, SmartMotorControllerConfig::TelemetryVerbosity::HIGH);
      ForHardware(cfg, param.hardware, 40.0_A);
      math::ADRCConfig adrcConfig;
      adrcConfig.WithSystemOrder(math::ADRCConfig::SystemOrder::SECOND).WithSettlingTime(0.6_s);
      cfg.WithADRCController(adrcConfig);
      auto bundle = MakeBundle(param, cfg);

      auto root = wpi::nt::NetworkTableInstance::GetDefault();
      auto tuningRoot = root.GetTable("ADRCLiveTuningTestTuning");
      bundle.smc->SetupTelemetry(root.GetTable("ADRCLiveTuningTest"), tuningRoot);
      auto tuning = tuningRoot->GetSubTable(name);
      auto entry = [&tuning](const std::string& key) { return tuning->GetEntry("closedloop/adrc/" + key); };
      REQUIRE(bundle.smc->GetADRCController() != nullptr);
      const double derivedB0 = bundle.smc->GetADRCController()->GetB0();

      // The tuning table shows what the ADRC runs with.
      CHECK(entry("b0").GetDouble(-1) == Catch::Approx(derivedB0));
      CHECK(entry("settlingTime").GetDouble(-1) == Catch::Approx(0.6));
      CHECK(entry("bandwidthScale").GetDouble(-1) == Catch::Approx(1.0));
      // The output limit and ramp are the motor config's, tuned with every other closed loop's.
      CHECK_FALSE(tuning->ContainsKey("closedloop/adrc/maxVoltage"));
      CHECK_FALSE(tuning->ContainsKey("closedloop/adrc/slewRateLimit"));

      // Enter new values and apply them.
      entry("b0").SetDouble(0.8 * derivedB0);
      entry("settlingTime").SetDouble(0.4);
      entry("bandwidthScale").SetDouble(1.5);
      bundle.smc->ApplyTuningValues();

      // The motor config's ADRC config holds exactly the entered values.
      auto tuned = bundle.smc->GetConfig().GetADRCController();
      REQUIRE(tuned != nullptr);
      CHECK(*tuned->GetB0() == Catch::Approx(0.8 * derivedB0));
      CHECK(tuned->GetSettlingTime()->value() == Catch::Approx(0.4));
      CHECK(*tuned->GetBandwidthScale() == Catch::Approx(1.5));
      // The one passed in is a copy, and is left alone.
      CHECK_FALSE(adrcConfig.GetB0());

      // The ADRC runs them.
      CheckRunning(*bundle.smc->GetADRCController(), 0.8 * derivedB0, 0.4, 1.5);

      // Applying the motor config again builds the ADRC from the tuned config.
      bundle.smc->ApplyConfig(bundle.smc->GetConfig());
      REQUIRE(bundle.smc->GetADRCController() != nullptr);
      CheckRunning(*bundle.smc->GetADRCController(), 0.8 * derivedB0, 0.4, 1.5);

      // A value the ADRC can't use leaves every value as it was, the valid ones entered with it too.
      entry("settlingTime").SetDouble(-1);
      entry("bandwidthScale").SetDouble(2.0);
      bundle.smc->ApplyTuningValues();
      CheckRunning(*bundle.smc->GetADRCController(), 0.8 * derivedB0, 0.4, 1.5);

      SchedulerHelper::CancelAll();
      CloseBundle(bundle);
    }
  }
}

}  // namespace yams::test
