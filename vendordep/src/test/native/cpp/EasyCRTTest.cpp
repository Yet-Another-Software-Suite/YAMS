// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Unit tests for EasyCRT pure math, no hardware required.
// Reference geometry: 100T shared gear with 19T and 21T encoder pinions.
// Unique range = lcm(19, 21) / 100 = 399 / 100 = 3.99 rotations.

#include <gtest/gtest.h>
#include <units/angle.h>

#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

#include "yams/units/EasyCRT.hpp"

namespace yams::test {

using ::yams::units::EasyCRT;

namespace {

constexpr int kSharedTeeth = 100;
constexpr int kATeeth = 19;
constexpr int kBTeeth = 21;

// The reading an ideal absolute encoder gives, wrapped into one rotation.
::units::turn_t Reading(double gearRotations, int pinionTeeth) {
  const double encoderRotations = gearRotations * kSharedTeeth / pinionTeeth;
  return ::units::turn_t{encoderRotations - std::floor(encoderRotations)};
}

}  // namespace

TEST(EasyCRTTest, UniqueRangeIsLcmOverTheSharedGear) {
  EXPECT_NEAR((EasyCRT{100, 19, 21}.GetUniqueRange().value()), 3.99, 1e-9);
  // 20 and 30 share a factor of 10, so they only cover lcm(20, 30) = 60 teeth.
  EXPECT_NEAR((EasyCRT{100, 20, 30}.GetUniqueRange().value()), 0.6, 1e-9);
}

TEST(EasyCRTTest, SolvesAcrossTheCenteredRange) {
  EasyCRT crt{kSharedTeeth, kATeeth, kBTeeth};
  const double half = crt.GetUniqueRange().value() / 2.0;
  for (double gear = -half + 0.01; gear < half - 0.01; gear += 0.0137) {
    auto solved = crt.Solve(Reading(gear, kATeeth), Reading(gear, kBTeeth));
    ASSERT_TRUE(solved.has_value());
    EXPECT_NEAR(solved->value(), gear, 1e-6);
  }
}

TEST(EasyCRTTest, SolvesAcrossAnOffCenterRange) {
  EasyCRT crt{kSharedTeeth, kATeeth, kBTeeth};
  for (double gear = 0.0; gear <= 3.5; gear += 0.0137) {
    auto solved = crt.Solve(Reading(gear, kATeeth), Reading(gear, kBTeeth), ::units::turn_t{0.0},
                            ::units::turn_t{3.5});
    ASSERT_TRUE(solved.has_value());
    EXPECT_NEAR(solved->value(), gear, 1e-6);
  }
}

TEST(EasyCRTTest, AcceptsUnwrappedAndNegativeReadings) {
  EasyCRT crt{kSharedTeeth, kATeeth, kBTeeth};
  const double gear = -1.3;
  auto solved = crt.Solve(::units::turn_t{gear * kSharedTeeth / kATeeth},
                          ::units::turn_t{gear * kSharedTeeth / kBTeeth});
  ASSERT_TRUE(solved.has_value());
  EXPECT_NEAR(solved->value(), gear, 1e-6);
}

TEST(EasyCRTTest, ToleratesBacklashOnEitherEncoder) {
  EasyCRT crt{kSharedTeeth, kATeeth, kBTeeth};
  const ::units::turn_t slop{2.0 / 360.0};
  for (double gear = -1.9; gear < 1.9; gear += 0.0137) {
    auto solved = crt.Solve(Reading(gear, kATeeth) + slop, Reading(gear, kBTeeth) - slop);
    ASSERT_TRUE(solved.has_value());
    // The result follows encoder A, so its error is the slop seen through the gearing of A.
    EXPECT_NEAR(solved->value(), gear, slop.value() * kATeeth / kSharedTeeth + 1e-6);
  }
}

TEST(EasyCRTTest, IsEmptyWhenTheEncodersDisagree) {
  EasyCRT crt{kSharedTeeth, kATeeth, kBTeeth};
  // Halfway between two candidates: 1 / (2 * 21) of a rotation off on encoder B.
  const auto wrong = Reading(0.5, kBTeeth) + ::units::turn_t{0.5 / kBTeeth};
  EXPECT_FALSE(crt.Solve(Reading(0.5, kATeeth), wrong).has_value());
}

TEST(EasyCRTTest, IsEmptyForReadingsThatAreNotNumbers) {
  EasyCRT crt{kSharedTeeth, kATeeth, kBTeeth};
  const ::units::turn_t nan{std::numeric_limits<double>::quiet_NaN()};
  EXPECT_FALSE(crt.Solve(nan, ::units::turn_t{0.1}).has_value());
}

TEST(EasyCRTTest, RejectsTravelLongerThanTheUniqueRange) {
  EasyCRT crt{kSharedTeeth, kATeeth, kBTeeth};
  const ::units::turn_t zero{0.0};
  EXPECT_THROW(crt.Solve(zero, zero, zero, ::units::turn_t{4.0}), std::invalid_argument);
  EXPECT_THROW(crt.Solve(zero, zero, ::units::turn_t{1.0}, zero), std::invalid_argument);
}

TEST(EasyCRTTest, RejectsToothCountsThatAreNotPositive) {
  EXPECT_THROW(EasyCRT(0, 19, 21), std::invalid_argument);
  EXPECT_THROW(EasyCRT(100, -19, 21), std::invalid_argument);
  EXPECT_THROW(EasyCRT(100, 19, 0), std::invalid_argument);
}

}  // namespace yams::test
