// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Unit tests for EasyCRT pure math, no hardware required.
// Reference geometry: 100T shared gear with 19T and 21T encoder pinions.
// Unique range = lcm(19, 21) / 100 = 399 / 100 = 3.99 rotations.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <wpi/units/angle.hpp>

#include "yams/units/EasyCRT.hpp"

namespace yams::test {

using ::yams::units::EasyCRT;

namespace {

constexpr int kSharedTeeth = 100;
constexpr int kATeeth = 19;
constexpr int kBTeeth = 21;

// The reading an ideal absolute encoder gives, wrapped into one rotation.
wpi::units::turn_t Reading(double gearRotations, int pinionTeeth) {
  const double encoderRotations = gearRotations * kSharedTeeth / pinionTeeth;
  return wpi::units::turn_t{encoderRotations - std::floor(encoderRotations)};
}

}  // namespace

TEST_CASE("EasyCRT unique range is lcm over the shared gear", "[EasyCRT]") {
  CHECK(EasyCRT{100, 19, 21}.GetUniqueRange().value() == Catch::Approx(3.99));
  // 20 and 30 share a factor of 10, so they only cover lcm(20, 30) = 60 teeth.
  CHECK(EasyCRT{100, 20, 30}.GetUniqueRange().value() == Catch::Approx(0.6));
}

TEST_CASE("EasyCRT solves across the centered range", "[EasyCRT]") {
  EasyCRT crt{kSharedTeeth, kATeeth, kBTeeth};
  const double half = crt.GetUniqueRange().value() / 2.0;
  for (double gear = -half + 0.01; gear < half - 0.01; gear += 0.0137) {
    auto solved = crt.Solve(Reading(gear, kATeeth), Reading(gear, kBTeeth));
    REQUIRE(solved.has_value());
    CHECK(solved->value() == Catch::Approx(gear).margin(1e-6));
  }
}

TEST_CASE("EasyCRT solves across an off center range", "[EasyCRT]") {
  EasyCRT crt{kSharedTeeth, kATeeth, kBTeeth};
  for (double gear = 0.0; gear <= 3.5; gear += 0.0137) {
    auto solved = crt.Solve(Reading(gear, kATeeth), Reading(gear, kBTeeth), wpi::units::turn_t{0.0},
                            wpi::units::turn_t{3.5});
    REQUIRE(solved.has_value());
    CHECK(solved->value() == Catch::Approx(gear).margin(1e-6));
  }
}

TEST_CASE("EasyCRT accepts unwrapped and negative readings", "[EasyCRT]") {
  EasyCRT crt{kSharedTeeth, kATeeth, kBTeeth};
  const double gear = -1.3;
  auto solved = crt.Solve(wpi::units::turn_t{gear * kSharedTeeth / kATeeth},
                          wpi::units::turn_t{gear * kSharedTeeth / kBTeeth});
  REQUIRE(solved.has_value());
  CHECK(solved->value() == Catch::Approx(gear).margin(1e-6));
}

TEST_CASE("EasyCRT tolerates backlash on either encoder", "[EasyCRT]") {
  EasyCRT crt{kSharedTeeth, kATeeth, kBTeeth};
  const wpi::units::turn_t slop{2.0 / 360.0};
  for (double gear = -1.9; gear < 1.9; gear += 0.0137) {
    auto solved = crt.Solve(Reading(gear, kATeeth) + slop, Reading(gear, kBTeeth) - slop);
    REQUIRE(solved.has_value());
    // The result follows encoder A, so its error is the slop seen through the gearing of A.
    CHECK(solved->value() ==
          Catch::Approx(gear).margin(slop.value() * kATeeth / kSharedTeeth + 1e-6));
  }
}

TEST_CASE("EasyCRT is empty when the encoders disagree", "[EasyCRT]") {
  EasyCRT crt{kSharedTeeth, kATeeth, kBTeeth};
  // Halfway between two candidates: 1 / (2 * 21) of a rotation off on encoder B.
  const auto wrong = Reading(0.5, kBTeeth) + wpi::units::turn_t{0.5 / kBTeeth};
  CHECK_FALSE(crt.Solve(Reading(0.5, kATeeth), wrong).has_value());
}

TEST_CASE("EasyCRT is empty for readings that are not numbers", "[EasyCRT]") {
  EasyCRT crt{kSharedTeeth, kATeeth, kBTeeth};
  const wpi::units::turn_t nan{std::numeric_limits<double>::quiet_NaN()};
  CHECK_FALSE(crt.Solve(nan, wpi::units::turn_t{0.1}).has_value());
}

TEST_CASE("EasyCRT rejects travel longer than the unique range", "[EasyCRT]") {
  EasyCRT crt{kSharedTeeth, kATeeth, kBTeeth};
  const wpi::units::turn_t zero{0.0};
  CHECK_THROWS_AS(crt.Solve(zero, zero, zero, wpi::units::turn_t{4.0}), std::invalid_argument);
  CHECK_THROWS_AS(crt.Solve(zero, zero, wpi::units::turn_t{1.0}, zero), std::invalid_argument);
}

TEST_CASE("EasyCRT rejects tooth counts that are not positive", "[EasyCRT]") {
  CHECK_THROWS_AS(EasyCRT(0, 19, 21), std::invalid_argument);
  CHECK_THROWS_AS(EasyCRT(100, -19, 21), std::invalid_argument);
  CHECK_THROWS_AS(EasyCRT(100, 19, 0), std::invalid_argument);
}

}  // namespace yams::test
