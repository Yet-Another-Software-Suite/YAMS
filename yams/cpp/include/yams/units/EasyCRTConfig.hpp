// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/RobotBase.h>
#include <units/angle.h>

#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"

namespace yams::units {

/**
 * Configuration for the EasyCRT absolute angle solver.  Made by team 6911.
 *
 * Stores encoder suppliers, gearing ratios, offsets, mechanism range, match tolerance,
 * and optional gear-recommendation search parameters.  Uses a fluent builder pattern;
 * all With* methods return *this for chaining.
 */
class EasyCRTConfig {
 public:
  /** Candidate gear pair returned by the CRT gear recommendation search. */
  struct CrtGearPair {
    int gearA;                  ///< Tooth count of encoder 1 gear.
    int gearB;                  ///< Tooth count of encoder 2 gear.
    int lcm;                    ///< Least-common-multiple of gearA and gearB.
    ::units::turn_t coverage;   ///< Unique angular coverage provided by this pair.
    int gcd;                    ///< Greatest-common-divisor of gearA and gearB.
    int theoreticalIterations;  ///< Maximum iterations the solver will need.
  };

  /**
   * Construct an EasyCRTConfig with the two encoder angle suppliers.
   *
   * @param encoder1Supplier Callable returning the current angle of encoder 1 in turns.
   * @param encoder2Supplier Callable returning the current angle of encoder 2 in turns.
   */
  EasyCRTConfig(std::function<::units::turn_t()> encoder1Supplier,
                std::function<::units::turn_t()> encoder2Supplier);

  /**
   * Set encoder rotations-per-mechanism-rotation ratios directly.
   *
   * @param enc1RotPerMechRot Encoder 1 turns per mechanism turn.
   * @param enc2RotPerMechRot Encoder 2 turns per mechanism turn.
   * @return *this for chaining.
   */
  EasyCRTConfig& WithEncoderRatios(double enc1RotPerMechRot, double enc2RotPerMechRot);

  /**
   * Derive encoder ratios from a common drive gear and two encoder pinion gears.
   *
   * @param commonRatio      Gear ratio of the stage shared by both encoders.
   * @param driveGearTeeth   Tooth count of the drive gear.
   * @param enc1PinionTeeth  Tooth count of the encoder 1 pinion.
   * @param enc2PinionTeeth  Tooth count of the encoder 2 pinion.
   * @return *this for chaining.
   */
  EasyCRTConfig& WithCommonDriveGear(double commonRatio, int driveGearTeeth, int enc1PinionTeeth,
                                     int enc2PinionTeeth);

  /**
   * Set zero offsets added to the raw encoder readings before wrapping.
   *
   * @param enc1Offset Offset for encoder 1 in turns.
   * @param enc2Offset Offset for encoder 2 in turns.
   * @return *this for chaining.
   */
  EasyCRTConfig& WithAbsoluteEncoderOffsets(::units::turn_t enc1Offset, ::units::turn_t enc2Offset);

  /**
   * Set the allowed mechanism rotation range for the solver.
   *
   * @param minAngle Minimum mechanism angle in turns.
   * @param maxAngle Maximum mechanism angle in turns.
   * @return *this for chaining.
   */
  EasyCRTConfig& WithMechanismRange(::units::turn_t minAngle, ::units::turn_t maxAngle);

  /**
   * Set the maximum modular error for accepting a solution.
   *
   * @param tolerance Match tolerance in turns (default 0.005 turns).
   * @return *this for chaining.
   */
  EasyCRTConfig& WithMatchTolerance(::units::turn_t tolerance);

  /**
   * Invert encoder 1 reading direction.
   *
   * @param inverted true to negate encoder 1.
   * @return *this for chaining.
   */
  EasyCRTConfig& WithAbsoluteEncoder1Inverted(bool inverted);

  /**
   * Invert encoder 2 reading direction.
   *
   * @param inverted true to negate encoder 2.
   * @return *this for chaining.
   */
  EasyCRTConfig& WithAbsoluteEncoder2Inverted(bool inverted);

  /**
   * Set inversion for both encoders at once.
   *
   * @param enc1Inverted true to negate encoder 1.
   * @param enc2Inverted true to negate encoder 2.
   * @return *this for chaining.
   */
  EasyCRTConfig& WithAbsoluteEncoderInversions(bool enc1Inverted, bool enc2Inverted);

  /**
   * Set the stage 1 gear teeth and stage 2 ratio used by the CRT gear recommendation search.
   *
   * @param stage1GearTeeth Tooth count of the first drive gear.
   * @param stage2Ratio     Gear ratio of the second reduction stage.
   * @return *this for chaining.
   */
  EasyCRTConfig& WithCrtGearRecommendationInputs(int stage1GearTeeth, double stage2Ratio);

  /**
   * Set search constraints for the CRT gear recommendation algorithm.
   *
   * @param coverageMargin   Required coverage margin above the mechanism range (e.g. 1.1).
   * @param minTeeth         Minimum tooth count to consider.
   * @param maxTeeth         Maximum tooth count to consider.
   * @param maxIterationsLimit Maximum solver iterations allowed.
   * @return *this for chaining.
   */
  EasyCRTConfig& WithCrtGearRecommendationConstraints(double coverageMargin, int minTeeth,
                                                      int maxTeeth, int maxIterationsLimit);

  /**
   * Set the gearing for encoder 1 from a meshed tooth chain.
   *
   * @param teethChain Tooth counts in mesh order (alternating driver/driven).
   * @return *this for chaining.
   */
  EasyCRTConfig& WithAbsoluteEncoder1Gearing(std::vector<int> teethChain);

  /**
   * Set the gearing for encoder 2 from a meshed tooth chain.
   *
   * @param teethChain Tooth counts in mesh order (alternating driver/driven).
   * @return *this for chaining.
   */
  EasyCRTConfig& WithAbsoluteEncoder2Gearing(std::vector<int> teethChain);

  /**
   * Set the gearing for encoder 1 from explicit driver/driven pairs.
   *
   * @param driverDrivenPairs Alternating driver and driven tooth counts.
   * @return *this for chaining.
   */
  EasyCRTConfig& WithAbsoluteEncoder1GearingStages(std::vector<int> driverDrivenPairs);

  /**
   * Set the gearing for encoder 2 from explicit driver/driven pairs.
   *
   * @param driverDrivenPairs Alternating driver and driven tooth counts.
   * @return *this for chaining.
   */
  EasyCRTConfig& WithAbsoluteEncoder2GearingStages(std::vector<int> driverDrivenPairs);

  /**
   * Read the current angle from encoder 1 via the configured supplier.
   *
   * @return Encoder 1 angle in turns.
   */
  ::units::turn_t GetAbsoluteEncoder1Angle() const;

  /**
   * Read the current angle from encoder 2 via the configured supplier.
   *
   * @return Encoder 2 angle in turns.
   */
  ::units::turn_t GetAbsoluteEncoder2Angle() const;

  /** @return Zero offset for encoder 1 in turns. */
  ::units::turn_t GetAbsoluteEncoder1Offset() const;
  /** @return Zero offset for encoder 2 in turns. */
  ::units::turn_t GetAbsoluteEncoder2Offset() const;
  /** @return Minimum mechanism angle in turns. */
  ::units::turn_t GetMinMechanismAngle() const;
  /** @return Maximum mechanism angle in turns. */
  ::units::turn_t GetMaxMechanismAngle() const;
  /** @return Mechanism range (max − min) in turns. */
  ::units::turn_t GetMechanismRange() const;
  /** @return Match tolerance in turns. */
  ::units::turn_t GetMatchTolerance() const;

  /** @return Encoder 1 turns per mechanism turn. */
  double GetEncoder1RotationsPerMechanismRotation() const;
  /** @return Encoder 2 turns per mechanism turn. */
  double GetEncoder2RotationsPerMechanismRotation() const;

  /** @return MechanismGearing for encoder 1. */
  gearing::MechanismGearing GetAbsoluteEncoder1Gearing() const;
  /** @return MechanismGearing for encoder 2. */
  gearing::MechanismGearing GetAbsoluteEncoder2Gearing() const;

  /**
   * Compute the unique angular coverage provided by the configured gear pair.
   *
   * @return Coverage in turns if both ratios are configured, otherwise empty.
   */
  std::optional<::units::turn_t> GetUniqueCoverage() const;

  /**
   * Return true if the configured gear pair coverage exceeds the mechanism range.
   *
   * @return true if coverage ≥ mechanism range.
   */
  bool CoverageSatisfiesRange() const;

  /**
   * Run the gear recommendation search and return the best CrtGearPair found.
   *
   * @return Best gear pair if search parameters have been set, otherwise empty.
   */
  std::optional<CrtGearPair> GetRecommendedCrtGearPair() const;

  /**
   * Compute the encoder ratio from a meshed tooth chain.
   *
   * @param teethChain Tooth counts in mesh order.
   * @return Ratio (encoder rotations per mechanism rotation).
   */
  static double RatioFromChain(const std::vector<int>& teethChain);

  /**
   * Compute the encoder ratio from explicit driver/driven tooth pairs.
   *
   * @param pairs Alternating driver and driven tooth counts.
   * @return Ratio (encoder rotations per mechanism rotation).
   */
  static double RatioFromDriverDrivenPairs(const std::vector<int>& pairs);

  /**
   * Compute the encoder ratio from a common drive gear and an encoder pinion.
   *
   * @param commonRatio    Common drive-stage gear ratio.
   * @param driveGearTeeth Tooth count of the drive gear.
   * @param encoderTeeth   Tooth count of the encoder pinion.
   * @return Ratio (encoder rotations per mechanism rotation).
   */
  static double RatioFromCommonDrive(double commonRatio, int driveGearTeeth, int encoderTeeth);

  /**
   * Return true if a and b share no common factors other than 1.
   *
   * @param a First integer.
   * @param b Second integer.
   * @return true if gcd(a, b) == 1.
   */
  static bool IsCoprime(int a, int b);

  /**
   * Find the smallest coprime gear pair that satisfies coverage and iteration constraints.
   *
   * @param stage1GearTeeth    Tooth count of the first drive gear.
   * @param stage2Ratio        Ratio of the second stage.
   * @param maxMechanismAngle  Maximum mechanism angle.
   * @param coverageMargin     Required coverage multiplier (e.g. 1.1 for 10 % margin).
   * @param minTeeth           Minimum gear tooth count to search.
   * @param maxTeeth           Maximum gear tooth count to search.
   * @param maxIterationsLimit Maximum solver iterations.
   * @return Best CrtGearPair found.
   */
  static CrtGearPair FindSmallestCrtGearPair(int stage1GearTeeth, double stage2Ratio,
                                             ::units::turn_t maxMechanismAngle,
                                             double coverageMargin, int minTeeth, int maxTeeth,
                                             int maxIterationsLimit);

 private:
  std::function<::units::turn_t()> m_enc1Supplier;
  std::function<::units::turn_t()> m_enc2Supplier;

  std::optional<double> m_enc1RotPerMechRot;
  std::optional<double> m_enc2RotPerMechRot;

  ::units::turn_t m_enc1Offset{0.0};
  ::units::turn_t m_enc2Offset{0.0};
  ::units::turn_t m_minMechAngle{0.0};
  ::units::turn_t m_maxMechAngle{1.0};
  ::units::turn_t m_matchTolerance{0.005};

  bool m_enc1Inverted{false};
  bool m_enc2Inverted{false};

  std::optional<int> m_enc1PrimeTeeth;
  std::optional<int> m_enc2PrimeTeeth;
  std::optional<double> m_commonScaleK;

  std::optional<int> m_gearSearchStage1Teeth;
  std::optional<double> m_gearSearchStage2Ratio;
  std::optional<double> m_gearSearchCoverageMargin;
  std::optional<int> m_gearSearchMinTeeth;
  std::optional<int> m_gearSearchMaxTeeth;
  std::optional<int> m_gearSearchMaxIterations;

  std::optional<std::vector<int>> m_enc1TeethChain;
  std::optional<std::vector<int>> m_enc2TeethChain;
  std::optional<std::vector<int>> m_enc1TeethStages;
  std::optional<std::vector<int>> m_enc2TeethStages;

  double GetOrComputeRatio(int encoderIndex) const;
  gearing::MechanismGearing BuildGearingForEncoder(int encoderIndex) const;

  static int Gcd(int a, int b);
  static int Lcm(int a, int b);
  static std::vector<std::string> BuildStagesFromChain(const std::vector<int>& chain);
  static std::vector<std::string> BuildStagesFromDriverDrivenPairs(const std::vector<int>& pairs);
  static int TheoreticalIterationsForGear(int gearTeeth, int stage1GearTeeth, double stage2Ratio,
                                          ::units::turn_t maxMechAngle);
};

}  // namespace yams::units
