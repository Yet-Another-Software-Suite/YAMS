// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <numbers>
#include <random>
#include <stdexcept>
#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/geometry/Rotation3d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/math/kinematics/ChassisVelocities.hpp>
#include <wpi/math/kinematics/SwerveDriveKinematics.hpp>
#include <wpi/math/kinematics/SwerveModulePosition.hpp>
#include <wpi/math/kinematics/SwerveModuleVelocity.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/velocity.hpp>

#include "yams/mechanisms/swerve/SkidDetector.hpp"
#include "yams/mechanisms/swerve/SwerveDriveConfig.hpp"
#include "yams/mechanisms/swerve/TiltOdometryScaler.hpp"

namespace yams::test {

using namespace yams::mechanisms::swerve;
using namespace wpi::units::literals;

namespace {
constexpr double kHalfTrackMeters = 0.30;

std::array<wpi::math::Translation2d, 4> SquareLocations() {
  return {
      wpi::math::Translation2d{wpi::units::meter_t{kHalfTrackMeters},
                               wpi::units::meter_t{kHalfTrackMeters}},
      wpi::math::Translation2d{wpi::units::meter_t{kHalfTrackMeters},
                               wpi::units::meter_t{-kHalfTrackMeters}},
      wpi::math::Translation2d{wpi::units::meter_t{-kHalfTrackMeters},
                               wpi::units::meter_t{kHalfTrackMeters}},
      wpi::math::Translation2d{wpi::units::meter_t{-kHalfTrackMeters},
                               wpi::units::meter_t{-kHalfTrackMeters}},
  };
}

std::array<wpi::math::SwerveModulePosition, 4> MakePositions(double d0, double d1, double d2,
                                                             double d3) {
  return {
      wpi::math::SwerveModulePosition{wpi::units::meter_t{d0}, wpi::math::Rotation2d{}},
      wpi::math::SwerveModulePosition{wpi::units::meter_t{d1}, wpi::math::Rotation2d{}},
      wpi::math::SwerveModulePosition{wpi::units::meter_t{d2}, wpi::math::Rotation2d{}},
      wpi::math::SwerveModulePosition{wpi::units::meter_t{d3}, wpi::math::Rotation2d{}},
  };
}
}  // namespace

TEST_CASE("SkidDetector.NominalDuringStopTranslateSpinAndCombined", "[SkidDetector]") {
  SkidDetector detector{1.35};
  auto locs = SquareLocations();
  wpi::math::SwerveDriveKinematics<4> kinematics{locs[0], locs[1], locs[2], locs[3]};

  auto stopped = kinematics.ToSwerveModuleVelocities(wpi::math::ChassisVelocities{0_mps, 0_mps, 0_rad_per_s});
  auto stopState = detector.Calculate(stopped, locs, 0.0);
  CHECK(stopState.skidRatio == Catch::Approx(1.0));
  CHECK_FALSE(stopState.skidding);
  CHECK(stopState.slippingModuleMask == SkidDetector::kNoSlippingModules);

  auto combo = kinematics.ToSwerveModuleVelocities(wpi::math::ChassisVelocities{2.0_mps, -1.5_mps, 2.2_rad_per_s});
  auto comboState = detector.Calculate(combo, locs, 2.2);
  CHECK(comboState.skidRatio == Catch::Approx(1.0).margin(1e-4));
  CHECK_FALSE(comboState.skidding);
  CHECK(comboState.slippingModuleMask == SkidDetector::kNoSlippingModules);
}

TEST_CASE("SkidDetector.SuppressesLowSpeedNoiseSpreadBelowDeadband", "[SkidDetector]") {
  SkidDetector detector{1.35, 0.10_mps};
  auto locs = SquareLocations();
  std::array<wpi::math::SwerveModuleVelocity, 4> noisy = {
      wpi::math::SwerveModuleVelocity{0.12_mps, wpi::math::Rotation2d{}},
      wpi::math::SwerveModuleVelocity{0.08_mps, wpi::math::Rotation2d{}},
      wpi::math::SwerveModuleVelocity{0.10_mps, wpi::math::Rotation2d{}},
      wpi::math::SwerveModuleVelocity{0.09_mps, wpi::math::Rotation2d{}},
  };
  auto state = detector.Calculate(noisy, locs, 0.0);
  CHECK_FALSE(state.skidding);
  CHECK(state.skidRatio == Catch::Approx(1.0));
  CHECK(state.slippingModuleMask == SkidDetector::kNoSlippingModules);
}

TEST_CASE("SkidDetector.DetectsDualLeftWheelSlipWithGyroOmega", "[SkidDetector]") {
  SkidDetector detector{1.35};
  auto locs = SquareLocations();
  std::array<wpi::math::SwerveModuleVelocity, 4> leftSlip = {
      wpi::math::SwerveModuleVelocity{1.6_mps, wpi::math::Rotation2d{}},
      wpi::math::SwerveModuleVelocity{1.0_mps, wpi::math::Rotation2d{}},
      wpi::math::SwerveModuleVelocity{1.6_mps, wpi::math::Rotation2d{}},
      wpi::math::SwerveModuleVelocity{1.0_mps, wpi::math::Rotation2d{}},
  };
  auto state = detector.Calculate(leftSlip, locs, 0.0);
  CHECK(state.skidding);
  CHECK(state.skidRatio == Catch::Approx(1.6).margin(1e-4));
  CHECK(state.slippingModuleMask == 0b1111U);
}

TEST_CASE("SkidDetector.IsolatesSingleSlippingOrDraggingModuleInBitmask", "[SkidDetector]") {
  SkidDetector detector{1.35};
  auto locs = SquareLocations();

  std::array<wpi::math::SwerveModuleVelocity, 4> singleSpin = {
      wpi::math::SwerveModuleVelocity{1.0_mps, wpi::math::Rotation2d{}},
      wpi::math::SwerveModuleVelocity{1.0_mps, wpi::math::Rotation2d{}},
      wpi::math::SwerveModuleVelocity{1.0_mps, wpi::math::Rotation2d{}},
      wpi::math::SwerveModuleVelocity{1.8_mps, wpi::math::Rotation2d{}},
  };
  auto spinState = detector.Calculate(singleSpin, locs, 0.0);
  CHECK(spinState.skidding);
  CHECK(spinState.slippingModuleMask == (1U << 3));
  CHECK(spinState.IsModuleSlipping(3));
  CHECK_FALSE(spinState.IsModuleSlipping(0));

  std::array<wpi::math::SwerveModuleVelocity, 4> singleDrag = {
      wpi::math::SwerveModuleVelocity{1.0_mps, wpi::math::Rotation2d{}},
      wpi::math::SwerveModuleVelocity{0.2_mps, wpi::math::Rotation2d{}},
      wpi::math::SwerveModuleVelocity{1.0_mps, wpi::math::Rotation2d{}},
      wpi::math::SwerveModuleVelocity{1.0_mps, wpi::math::Rotation2d{}},
  };
  auto dragState = detector.Calculate(singleDrag, locs, 0.0);
  CHECK(dragState.skidding);
  CHECK(dragState.slippingModuleMask == (1U << 1));
  CHECK(dragState.IsModuleSlipping(1));
  CHECK_FALSE(dragState.IsModuleSlipping(2));
}

TEST_CASE("SkidDetector.ValidatesAndFuzzesCleanly", "[SkidDetector]") {
  CHECK_THROWS_AS(SkidDetector(1.0), std::invalid_argument);
  CHECK_THROWS_AS(SkidDetector(1.35, -0.1_mps), std::invalid_argument);

  SkidDetector detector{1.35};
  auto locs = SquareLocations();
  wpi::math::SwerveDriveKinematics<4> kinematics{locs[0], locs[1], locs[2], locs[3]};
  std::mt19937_64 rng{20270278ULL};
  std::uniform_real_distribution<double> velDist{-4.5, 4.5};
  std::uniform_real_distribution<double> omegaDist{-8.0, 8.0};
  std::uniform_real_distribution<double> slipDist{0.0, 3.0};

  for (int trial = 0; trial < 200; ++trial) {
    const double omega = omegaDist(rng);
    auto states = kinematics.ToSwerveModuleVelocities(wpi::math::ChassisVelocities{
        wpi::units::meters_per_second_t{velDist(rng)},
        wpi::units::meters_per_second_t{velDist(rng)},
        wpi::units::radians_per_second_t{omega}});
    if (trial % 3 == 0) {
      const size_t idx = static_cast<size_t>(trial % 4);
      states[idx].velocity = wpi::units::meters_per_second_t{states[idx].velocity.value() * slipDist(rng)};
    }
    auto st = detector.Calculate(states, locs, omega);
    REQUIRE(std::isfinite(st.skidRatio));
    REQUIRE(st.skidRatio >= 1.0 - 1e-6);
    if (st.skidding) {
      REQUIRE(st.slippingModuleMask != SkidDetector::kNoSlippingModules);
    } else {
      REQUIRE(st.slippingModuleMask == SkidDetector::kNoSlippingModules);
    }
  }
}

TEST_CASE("TiltOdometryScaler.LinearRampAndCosineProjectionModes", "[TiltOdometryScaler]") {
  TiltOdometryScaler linear{2_deg, 25_deg, TiltOdometryScaler::ScalingMode::LINEAR_RAMP};
  TiltOdometryScaler cosine{2_deg, 25_deg, TiltOdometryScaler::ScalingMode::COSINE_PROJECTION};

  wpi::math::Rotation3d flatYawed{0_deg, 0_deg, 137_deg};
  CHECK(linear.ComputeScale(flatYawed) == Catch::Approx(1.0));
  CHECK(cosine.ComputeScale(flatYawed) == Catch::Approx(1.0));

  wpi::math::Rotation3d midTilt{0_deg, 13.5_deg, 45_deg};
  CHECK(linear.ComputeScale(midTilt) == Catch::Approx(0.5).margin(1e-4));
  CHECK(cosine.ComputeScale(midTilt) ==
        Catch::Approx(std::cos(13.5 * std::numbers::pi / 180.0)).margin(1e-4));

  wpi::math::Rotation3d steep{30_deg, 0_deg, 0_deg};
  CHECK(linear.ComputeScale(steep) == Catch::Approx(0.0));
  CHECK(cosine.ComputeScale(steep) == Catch::Approx(0.0));
}

TEST_CASE("TiltOdometryScaler.DropsOnlySlippingModuleDelta", "[TiltOdometryScaler]") {
  auto scaler = TiltOdometryScaler::Passthrough();
  scaler.Reset(MakePositions(0.0, 0.0, 0.0, 0.0));

  auto scaled1 = scaler.ScalePositions(wpi::math::Rotation3d{}, MakePositions(1.0, 1.0, 1.0, 1.0),
                                       uint32_t{1U << 3});
  CHECK(scaled1[0].distance.value() == Catch::Approx(1.0));
  CHECK(scaled1[1].distance.value() == Catch::Approx(1.0));
  CHECK(scaled1[2].distance.value() == Catch::Approx(1.0));
  CHECK(scaled1[3].distance.value() == Catch::Approx(0.0));
  CHECK(scaler.GetLastScale() == Catch::Approx(0.75));

  auto scaled2 = scaler.ScalePositions(wpi::math::Rotation3d{}, MakePositions(1.5, 1.5, 1.5, 1.5),
                                       SkidDetector::kNoSlippingModules);
  CHECK(scaled2[0].distance.value() == Catch::Approx(1.5));
  CHECK(scaled2[3].distance.value() == Catch::Approx(0.5));
  CHECK(scaler.GetLastScale() == Catch::Approx(1.0));
}

TEST_CASE("TiltOdometryScaler.ConfigAndSwerveDriveConfigParity", "[TiltOdometryScaler]") {
  CHECK_THROWS_AS(TiltOdometryScaler::Config(10_deg, 10_deg), std::invalid_argument);
  CHECK_THROWS_AS(TiltOdometryScaler::Config(-1_deg, 20_deg), std::invalid_argument);
  CHECK_THROWS_AS(TiltOdometryScaler::Config(2_deg, 95_deg), std::invalid_argument);

  SwerveDriveConfig cfg;
  cfg.WithSkidDetection(1.4, 0.15_mps)
      .WithSkidOdometryRejection(true)
      .WithTiltScaledOdometry(3_deg, 20_deg, TiltOdometryScaler::ScalingMode::COSINE_PROJECTION)
      .WithVisionMaxTilt(6_deg);

  REQUIRE(cfg.GetSkidDetector().has_value());
  CHECK(cfg.GetSkidDetector()->GetSkidRatioThreshold() == Catch::Approx(1.4));
  CHECK(cfg.IsSkidOdometryRejectionEnabled());
  REQUIRE(cfg.GetTiltOdometryScalerConfig().has_value());
  CHECK(cfg.GetTiltOdometryScalerConfig()->scalingMode ==
        TiltOdometryScaler::ScalingMode::COSINE_PROJECTION);
  REQUIRE(cfg.GetVisionMaxTilt().has_value());
  CHECK(wpi::units::degree_t{*cfg.GetVisionMaxTilt()}.value() == Catch::Approx(6.0));
}

}  // namespace yams::test
