// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/RobotBase.h>
#include <units/angle.h>

#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "yams/gearing/GearBox.h"
#include "yams/gearing/MechanismGearing.h"

namespace yams::units {

class EasyCRTConfig {
 public:
  struct CrtGearPair {
    int gearA;
    int gearB;
    int lcm;
    ::units::turn_t coverage;
    int gcd;
    int theoreticalIterations;
  };

  EasyCRTConfig(std::function<::units::turn_t()> encoder1Supplier,
                std::function<::units::turn_t()> encoder2Supplier);

  EasyCRTConfig& WithEncoderRatios(double enc1RotPerMechRot, double enc2RotPerMechRot);
  EasyCRTConfig& WithCommonDriveGear(double commonRatio, int driveGearTeeth, int enc1PinionTeeth,
                                     int enc2PinionTeeth);
  EasyCRTConfig& WithAbsoluteEncoderOffsets(::units::turn_t enc1Offset, ::units::turn_t enc2Offset);
  EasyCRTConfig& WithMechanismRange(::units::turn_t minAngle, ::units::turn_t maxAngle);
  EasyCRTConfig& WithMatchTolerance(::units::turn_t tolerance);
  EasyCRTConfig& WithAbsoluteEncoder1Inverted(bool inverted);
  EasyCRTConfig& WithAbsoluteEncoder2Inverted(bool inverted);
  EasyCRTConfig& WithAbsoluteEncoderInversions(bool enc1Inverted, bool enc2Inverted);
  EasyCRTConfig& WithCrtGearRecommendationInputs(int stage1GearTeeth, double stage2Ratio);
  EasyCRTConfig& WithCrtGearRecommendationConstraints(double coverageMargin, int minTeeth,
                                                      int maxTeeth, int maxIterationsLimit);
  EasyCRTConfig& WithAbsoluteEncoder1Gearing(std::vector<int> teethChain);
  EasyCRTConfig& WithAbsoluteEncoder2Gearing(std::vector<int> teethChain);
  EasyCRTConfig& WithAbsoluteEncoder1GearingStages(std::vector<int> driverDrivenPairs);
  EasyCRTConfig& WithAbsoluteEncoder2GearingStages(std::vector<int> driverDrivenPairs);

  ::units::turn_t GetAbsoluteEncoder1Angle() const;
  ::units::turn_t GetAbsoluteEncoder2Angle() const;
  ::units::turn_t GetAbsoluteEncoder1Offset() const;
  ::units::turn_t GetAbsoluteEncoder2Offset() const;
  ::units::turn_t GetMinMechanismAngle() const;
  ::units::turn_t GetMaxMechanismAngle() const;
  ::units::turn_t GetMechanismRange() const;
  ::units::turn_t GetMatchTolerance() const;

  double GetEncoder1RotationsPerMechanismRotation() const;
  double GetEncoder2RotationsPerMechanismRotation() const;

  gearing::MechanismGearing GetAbsoluteEncoder1Gearing() const;
  gearing::MechanismGearing GetAbsoluteEncoder2Gearing() const;

  std::optional<::units::turn_t> GetUniqueCoverage() const;
  bool CoverageSatisfiesRange() const;

  std::optional<CrtGearPair> GetRecommendedCrtGearPair() const;

  static double RatioFromChain(const std::vector<int>& teethChain);
  static double RatioFromDriverDrivenPairs(const std::vector<int>& pairs);
  static double RatioFromCommonDrive(double commonRatio, int driveGearTeeth, int encoderTeeth);
  static bool IsCoprime(int a, int b);
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
