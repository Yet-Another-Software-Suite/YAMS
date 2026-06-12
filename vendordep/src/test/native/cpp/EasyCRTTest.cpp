// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Unit tests for EasyCRT — pure math, no hardware required.
// Reference geometry: 200T mechanism with 19T and 21T encoder gears (coprime).
// CRT period = lcm(19, 21) / 200 = 399 / 200 = 1.995 rotations.

#include <gtest/gtest.h>
#include <units/angle.h>

#include <cmath>
#include <optional>

#include "yams/units/EasyCRT.hpp"

namespace yams::test {

using ::yams::units::CrtCommonK;
using ::yams::units::CrtGcd;
using ::yams::units::CrtIsCoprime;
using ::yams::units::CrtLcm;
using ::yams::units::CrtRatioFromChain;
using ::yams::units::CrtRatioFromStages;
using ::yams::units::EasyCRT;
using ::yams::units::EasyCRTConfig;

// ── Helpers ───────────────────────────────────────────────────────────────────

// Returns ideal (noiseless) encoder readings for a given mechanism position.
static EasyCRTConfig MakePerfectConfig(double mechRot, int t1 = 19, int t2 = 21, double k = 200.0) {
  const double e1 = mechRot * k / t1;
  const double e2 = mechRot * k / t2;
  EasyCRTConfig cfg;
  cfg.enc1 = [e1] { return ::units::turn_t{e1}; };
  cfg.enc2 = [e2] { return ::units::turn_t{e2}; };
  const double period = CrtLcm(t1, t2) / k;
  return cfg.WithTeeth(t1, t2, k).WithRange(0.0, period - 1e-6);
}

// ── CrtGcd ────────────────────────────────────────────────────────────────────

TEST(CrtGcdTest, CommonFactor) { EXPECT_EQ(CrtGcd(4, 6), 2); }

TEST(CrtGcdTest, Coprime) { EXPECT_EQ(CrtGcd(19, 21), 1); }

TEST(CrtGcdTest, IdentityWithZero) { EXPECT_EQ(CrtGcd(7, 0), 7); }

// ── CrtLcm ────────────────────────────────────────────────────────────────────

TEST(CrtLcmTest, BasicLcm) { EXPECT_EQ(CrtLcm(4, 6), 12); }

TEST(CrtLcmTest, CoprimeLcm) { EXPECT_EQ(CrtLcm(19, 21), 399); }

// ── CrtIsCoprime ──────────────────────────────────────────────────────────────

TEST(CrtIsCoprimeTest, CoprimePair) { EXPECT_TRUE(CrtIsCoprime(19, 21)); }

TEST(CrtIsCoprimeTest, NonCoprimePair) { EXPECT_FALSE(CrtIsCoprime(4, 6)); }

// ── CrtRatioFromChain ─────────────────────────────────────────────────────────

TEST(CrtRatioFromChainTest, TwoGears) {
  EXPECT_NEAR(CrtRatioFromChain({50, 30}), 50.0 / 30.0, 1e-12);
}

TEST(CrtRatioFromChainTest, ThreeGearsIntermediateCancels) {
  // 50T→20T→40T: result is 50/40, not (50/20)*(20/40) computed separately.
  EXPECT_NEAR(CrtRatioFromChain({50, 20, 40}), 50.0 / 40.0, 1e-12);
}

TEST(CrtRatioFromChainTest, SingleElementReturnsOne) {
  EXPECT_NEAR(CrtRatioFromChain({50}), 1.0, 1e-12);
}

// ── CrtRatioFromStages ────────────────────────────────────────────────────────

TEST(CrtRatioFromStagesTest, SingleStage) {
  EXPECT_NEAR(CrtRatioFromStages({12, 36}), 12.0 / 36.0, 1e-12);
}

TEST(CrtRatioFromStagesTest, TwoStages) {
  EXPECT_NEAR(CrtRatioFromStages({12, 36, 18, 60}), (12.0 / 36.0) * (18.0 / 60.0), 1e-12);
}

// ── CrtCommonK ────────────────────────────────────────────────────────────────

TEST(CrtCommonKTest, Basic) { EXPECT_NEAR(CrtCommonK(11.0, 50), 550.0, 1e-12); }

TEST(CrtCommonKTest, DirectTurret) { EXPECT_NEAR(CrtCommonK(1.0, 200), 200.0, 1e-12); }

// ── EasyCRT: initial state ────────────────────────────────────────────────────

TEST(EasyCRTTest, InitialStatusIsNotAttempted) {
  EasyCRT solver{MakePerfectConfig(0.5)};
  EXPECT_EQ(solver.GetStatus(), EasyCRT::Status::NotAttempted);
}

TEST(EasyCRTTest, InitialErrorIsNaN) {
  EasyCRT solver{MakePerfectConfig(0.5)};
  EXPECT_TRUE(std::isnan(solver.GetLastError()));
}

// ── EasyCRT: successful solves ────────────────────────────────────────────────

TEST(EasyCRTTest, SolveNearZero) {
  EasyCRT solver{MakePerfectConfig(0.05)};
  auto result = solver.GetAngle();
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->value(), 0.05, 1e-9);
}

TEST(EasyCRTTest, SolveAtMidRange) {
  EasyCRT solver{MakePerfectConfig(1.0)};
  auto result = solver.GetAngle();
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->value(), 1.0, 1e-9);
}

TEST(EasyCRTTest, SolveNearEndOfPeriod) {
  // Just under one full CRT period.
  const double pos = 1.99;
  EasyCRT solver{MakePerfectConfig(pos)};
  auto result = solver.GetAngle();
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->value(), pos, 1e-9);
}

TEST(EasyCRTTest, StatusIsOkAfterSuccessfulSolve) {
  EasyCRT solver{MakePerfectConfig(0.75)};
  solver.GetAngle();
  EXPECT_EQ(solver.GetStatus(), EasyCRT::Status::Ok);
}

TEST(EasyCRTTest, LastIterationsIsFourAfterSolve) {
  EasyCRT solver{MakePerfectConfig(0.75)};
  solver.GetAngle();
  EXPECT_EQ(solver.GetLastIterations(), 4);
}

TEST(EasyCRTTest, LastErrorIsSmallAfterSuccessfulSolve) {
  EasyCRT solver{MakePerfectConfig(0.75)};
  solver.GetAngle();
  EXPECT_LT(solver.GetLastError(), 1e-6);
}

TEST(EasyCRTTest, SweepPositions) {
  // Verify the solver recovers every position across the full CRT period.
  for (int i = 1; i <= 199; ++i) {
    const double pos = i * 0.01;  // 0.01 .. 1.99 rotations
    EasyCRT solver{MakePerfectConfig(pos)};
    auto result = solver.GetAngle();
    ASSERT_TRUE(result.has_value()) << "Failed at mechRot=" << pos;
    EXPECT_NEAR(result->value(), pos, 1e-9) << "Wrong result at mechRot=" << pos;
  }
}

// ── EasyCRT: offsets ──────────────────────────────────────────────────────────

TEST(EasyCRTTest, SolveWithEncoderOffsets) {
  const double mechRot = 0.8;
  const double off1 = 0.12, off2 = -0.07;
  const int t1 = 19, t2 = 21;
  const double k = 200.0;

  // Shift the raw encoder readings by the offsets so they read "wrong",
  // then compensate with WithOffsets so the solver still recovers mechRot.
  const double raw1 = mechRot * k / t1 - off1;
  const double raw2 = mechRot * k / t2 - off2;

  EasyCRTConfig cfg;
  cfg.enc1 = [raw1] { return ::units::turn_t{raw1}; };
  cfg.enc2 = [raw2] { return ::units::turn_t{raw2}; };
  const double period = CrtLcm(t1, t2) / k;
  cfg.WithTeeth(t1, t2, k).WithRange(0.0, period - 1e-6).WithOffsets(off1, off2);

  EasyCRT solver{cfg};
  auto result = solver.GetAngle();
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->value(), mechRot, 1e-9);
}

// ── EasyCRT: inversion ────────────────────────────────────────────────────────

TEST(EasyCRTTest, SolveWithEnc1Inverted) {
  const double mechRot = 0.6;
  const int t1 = 19, t2 = 21;
  const double k = 200.0;

  // When inv1 is set the solver flips the enc1 reading (1 - abs),
  // so the supplier must return the un-flipped (inverted hardware) value.
  const double e1_inverted = 1.0 - std::fmod(mechRot * k / t1, 1.0);
  const double e2 = mechRot * k / t2;

  EasyCRTConfig cfg;
  cfg.enc1 = [e1_inverted] { return ::units::turn_t{e1_inverted}; };
  cfg.enc2 = [e2] { return ::units::turn_t{e2}; };
  const double period = CrtLcm(t1, t2) / k;
  cfg.WithTeeth(t1, t2, k).WithRange(0.0, period - 1e-6).WithInversions(true, false);

  EasyCRT solver{cfg};
  auto result = solver.GetAngle();
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->value(), mechRot, 1e-9);
}

// ── EasyCRT: invalid config ───────────────────────────────────────────────────

TEST(EasyCRTTest, InvalidConfig_NaNEncoder) {
  EasyCRTConfig cfg;
  cfg.enc1 = [] { return ::units::turn_t{std::numeric_limits<double>::quiet_NaN()}; };
  cfg.enc2 = [] { return ::units::turn_t{0.5}; };
  cfg.WithTeeth(19, 21, 200.0).WithRange(0.0, 1.99);

  EasyCRT solver{cfg};
  auto result = solver.GetAngle();
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(solver.GetStatus(), EasyCRT::Status::InvalidConfig);
}

TEST(EasyCRTTest, InvalidConfig_ZeroCommonK) {
  EasyCRTConfig cfg;
  cfg.enc1 = [] { return ::units::turn_t{0.1}; };
  cfg.enc2 = [] { return ::units::turn_t{0.2}; };
  cfg.WithTeeth(19, 21, 0.0).WithRange(0.0, 1.99);

  EasyCRT solver{cfg};
  auto result = solver.GetAngle();
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(solver.GetStatus(), EasyCRT::Status::InvalidConfig);
}

TEST(EasyCRTTest, InvalidConfig_InvertedRange) {
  EasyCRTConfig cfg;
  cfg.enc1 = [] { return ::units::turn_t{0.1}; };
  cfg.enc2 = [] { return ::units::turn_t{0.2}; };
  cfg.WithTeeth(19, 21, 200.0).WithRange(2.0, 0.0);  // min > max

  EasyCRT solver{cfg};
  auto result = solver.GetAngle();
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(solver.GetStatus(), EasyCRT::Status::InvalidConfig);
}

// ── EasyCRT: no solution ──────────────────────────────────────────────────────

TEST(EasyCRTTest, NoSolution_PositionOutsideRange) {
  // Mechanism is at 1.5 rot but the range only covers [0, 1.0].
  const double mechRot = 1.5;
  const int t1 = 19, t2 = 21;
  const double k = 200.0;

  const double e1 = mechRot * k / t1;
  const double e2 = mechRot * k / t2;
  EasyCRTConfig cfg;
  cfg.enc1 = [e1] { return ::units::turn_t{e1}; };
  cfg.enc2 = [e2] { return ::units::turn_t{e2}; };
  cfg.WithTeeth(t1, t2, k).WithRange(0.0, 1.0);

  EasyCRT solver{cfg};
  auto result = solver.GetAngle();
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(solver.GetStatus(), EasyCRT::Status::NoSolution);
}

}  // namespace yams::test
