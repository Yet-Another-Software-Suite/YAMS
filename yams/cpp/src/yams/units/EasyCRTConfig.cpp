// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/units/EasyCRTConfig.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace yams::units {

EasyCRTConfig::EasyCRTConfig(std::function<::units::angle::turn_t()> enc1Supplier,
                             std::function<::units::angle::turn_t()> enc2Supplier)
    : m_enc1Supplier(std::move(enc1Supplier)), m_enc2Supplier(std::move(enc2Supplier)) {}

EasyCRTConfig& EasyCRTConfig::WithEncoderRatios(double enc1RotPerMechRot,
                                                double enc2RotPerMechRot) {
  m_enc1RotPerMechRot = enc1RotPerMechRot;
  m_enc2RotPerMechRot = enc2RotPerMechRot;
  return *this;
}

EasyCRTConfig& EasyCRTConfig::WithCommonDriveGear(double commonRatio, int driveGearTeeth,
                                                  int enc1PinionTeeth, int enc2PinionTeeth) {
  if (driveGearTeeth <= 0) throw std::invalid_argument("driveGearTeeth must be > 0");
  if (enc1PinionTeeth <= 0) throw std::invalid_argument("Encoder1GearTeeth must be > 0");
  if (enc2PinionTeeth <= 0) throw std::invalid_argument("Encoder2GearTeeth must be > 0");

  double ratio1 = RatioFromCommonDrive(commonRatio, driveGearTeeth, enc1PinionTeeth);
  double ratio2 = RatioFromCommonDrive(commonRatio, driveGearTeeth, enc2PinionTeeth);
  WithEncoderRatios(ratio1, ratio2);

  m_enc1PrimeTeeth = enc1PinionTeeth;
  m_enc2PrimeTeeth = enc2PinionTeeth;
  m_commonScaleK = commonRatio * driveGearTeeth;
  m_gearSearchStage1Teeth = driveGearTeeth;
  m_gearSearchStage2Ratio = commonRatio;
  return *this;
}

EasyCRTConfig& EasyCRTConfig::WithAbsoluteEncoderOffsets(::units::angle::turn_t enc1Offset,
                                                         ::units::angle::turn_t enc2Offset) {
  m_enc1Offset = enc1Offset;
  m_enc2Offset = enc2Offset;
  return *this;
}

EasyCRTConfig& EasyCRTConfig::WithMechanismRange(::units::angle::turn_t minAngle,
                                                 ::units::angle::turn_t maxAngle) {
  m_minMechAngle = minAngle;
  m_maxMechAngle = maxAngle;
  return *this;
}

EasyCRTConfig& EasyCRTConfig::WithMatchTolerance(::units::angle::turn_t tolerance) {
  m_matchTolerance = tolerance;
  return *this;
}

EasyCRTConfig& EasyCRTConfig::WithAbsoluteEncoder1Inverted(bool inverted) {
  m_enc1Inverted = inverted;
  return *this;
}

EasyCRTConfig& EasyCRTConfig::WithAbsoluteEncoder2Inverted(bool inverted) {
  m_enc2Inverted = inverted;
  return *this;
}

EasyCRTConfig& EasyCRTConfig::WithAbsoluteEncoderInversions(bool enc1Inverted, bool enc2Inverted) {
  m_enc1Inverted = enc1Inverted;
  m_enc2Inverted = enc2Inverted;
  return *this;
}

EasyCRTConfig& EasyCRTConfig::WithCrtGearRecommendationInputs(int stage1GearTeeth,
                                                              double stage2Ratio) {
  if (stage1GearTeeth <= 0) throw std::invalid_argument("stage1GearTeeth must be > 0");
  m_gearSearchStage1Teeth = stage1GearTeeth;
  m_gearSearchStage2Ratio = stage2Ratio;
  return *this;
}

EasyCRTConfig& EasyCRTConfig::WithCrtGearRecommendationConstraints(double coverageMargin,
                                                                   int minTeeth, int maxTeeth,
                                                                   int maxIterationsLimit) {
  if (!frc::RobotBase::IsSimulation()) return *this;
  if (coverageMargin <= 0.0) throw std::invalid_argument("coverageMargin must be > 0");
  if (minTeeth <= 0) throw std::invalid_argument("minTeeth must be > 0");
  if (maxTeeth < minTeeth) throw std::invalid_argument("maxTeeth must be >= minTeeth");
  if (maxIterationsLimit < 1) throw std::invalid_argument("maxIterationsLimit must be >= 1");
  m_gearSearchCoverageMargin = coverageMargin;
  m_gearSearchMinTeeth = minTeeth;
  m_gearSearchMaxTeeth = maxTeeth;
  m_gearSearchMaxIterations = maxIterationsLimit;
  return *this;
}

EasyCRTConfig& EasyCRTConfig::WithAbsoluteEncoder1Gearing(std::vector<int> teethChain) {
  m_enc1TeethChain = std::move(teethChain);
  return *this;
}

EasyCRTConfig& EasyCRTConfig::WithAbsoluteEncoder2Gearing(std::vector<int> teethChain) {
  m_enc2TeethChain = std::move(teethChain);
  return *this;
}

EasyCRTConfig& EasyCRTConfig::WithAbsoluteEncoder1GearingStages(std::vector<int> pairs) {
  m_enc1TeethStages = std::move(pairs);
  return *this;
}

EasyCRTConfig& EasyCRTConfig::WithAbsoluteEncoder2GearingStages(std::vector<int> pairs) {
  m_enc2TeethStages = std::move(pairs);
  return *this;
}

::units::angle::turn_t EasyCRTConfig::GetAbsoluteEncoder1Angle() const {
  auto v = m_enc1Supplier();
  return std::isnan(v.value()) ? ::units::angle::turn_t{std::numeric_limits<double>::quiet_NaN()}
                               : v;
}

::units::angle::turn_t EasyCRTConfig::GetAbsoluteEncoder2Angle() const {
  auto v = m_enc2Supplier();
  return std::isnan(v.value()) ? ::units::angle::turn_t{std::numeric_limits<double>::quiet_NaN()}
                               : v;
}

::units::angle::turn_t EasyCRTConfig::GetAbsoluteEncoder1Offset() const { return m_enc1Offset; }
::units::angle::turn_t EasyCRTConfig::GetAbsoluteEncoder2Offset() const { return m_enc2Offset; }
::units::angle::turn_t EasyCRTConfig::GetMinMechanismAngle() const { return m_minMechAngle; }
::units::angle::turn_t EasyCRTConfig::GetMaxMechanismAngle() const { return m_maxMechAngle; }
::units::angle::turn_t EasyCRTConfig::GetMechanismRange() const {
  return m_maxMechAngle - m_minMechAngle;
}
::units::angle::turn_t EasyCRTConfig::GetMatchTolerance() const { return m_matchTolerance; }

double EasyCRTConfig::GetEncoder1RotationsPerMechanismRotation() const {
  double ratio = GetOrComputeRatio(1);
  return m_enc1Inverted ? -ratio : ratio;
}

double EasyCRTConfig::GetEncoder2RotationsPerMechanismRotation() const {
  double ratio = GetOrComputeRatio(2);
  return m_enc2Inverted ? -ratio : ratio;
}

double EasyCRTConfig::GetOrComputeRatio(int encoderIndex) const {
  if (encoderIndex == 1 && m_enc1RotPerMechRot.has_value()) return *m_enc1RotPerMechRot;
  if (encoderIndex == 2 && m_enc2RotPerMechRot.has_value()) return *m_enc2RotPerMechRot;

  const auto& pairs = (encoderIndex == 1) ? m_enc1TeethStages : m_enc2TeethStages;
  const auto& chain = (encoderIndex == 1) ? m_enc1TeethChain : m_enc2TeethChain;

  if (pairs.has_value()) return RatioFromDriverDrivenPairs(*pairs);
  if (chain.has_value()) return RatioFromChain(*chain);

  throw std::runtime_error("Encoder ratios not configured.");
}

gearing::MechanismGearing EasyCRTConfig::GetAbsoluteEncoder1Gearing() const {
  return BuildGearingForEncoder(1);
}

gearing::MechanismGearing EasyCRTConfig::GetAbsoluteEncoder2Gearing() const {
  return BuildGearingForEncoder(2);
}

gearing::MechanismGearing EasyCRTConfig::BuildGearingForEncoder(int idx) const {
  const auto& pairs = (idx == 1) ? m_enc1TeethStages : m_enc2TeethStages;
  const auto& chain = (idx == 1) ? m_enc1TeethChain : m_enc2TeethChain;

  if (pairs.has_value()) {
    auto stages = BuildStagesFromDriverDrivenPairs(*pairs);
    return gearing::MechanismGearing{gearing::GearBox(stages)};
  }
  if (chain.has_value()) {
    auto stages = BuildStagesFromChain(*chain);
    return gearing::MechanismGearing{gearing::GearBox(stages)};
  }
  throw std::runtime_error("Encoder gearing not set for encoder " + std::to_string(idx));
}

std::optional<::units::angle::turn_t> EasyCRTConfig::GetUniqueCoverage() const {
  if (!m_enc1PrimeTeeth || !m_enc2PrimeTeeth || !m_commonScaleK) return std::nullopt;
  double k = *m_commonScaleK;
  if (!std::isfinite(k) || std::abs(k) < 1e-12) return std::nullopt;
  int l = Lcm(*m_enc1PrimeTeeth, *m_enc2PrimeTeeth);
  return ::units::angle::turn_t{static_cast<double>(l) / k};
}

bool EasyCRTConfig::CoverageSatisfiesRange() const {
  auto cov = GetUniqueCoverage();
  if (!cov) return false;
  double covRot = cov->value();
  double maxRot = m_maxMechAngle.value();
  if (!std::isfinite(covRot) || !std::isfinite(maxRot) || maxRot <= 0.0) return false;
  return covRot >= maxRot;
}

std::optional<EasyCRTConfig::CrtGearPair> EasyCRTConfig::GetRecommendedCrtGearPair() const {
  if (!m_gearSearchStage1Teeth || !m_gearSearchStage2Ratio || !m_gearSearchCoverageMargin ||
      !m_gearSearchMinTeeth || !m_gearSearchMaxTeeth || !m_gearSearchMaxIterations) {
    return std::nullopt;
  }
  auto pair =
      FindSmallestCrtGearPair(*m_gearSearchStage1Teeth, *m_gearSearchStage2Ratio, m_maxMechAngle,
                              *m_gearSearchCoverageMargin, *m_gearSearchMinTeeth,
                              *m_gearSearchMaxTeeth, *m_gearSearchMaxIterations);
  if (pair.gearA == 0) return std::nullopt;
  return pair;
}

// static helpers
double EasyCRTConfig::RatioFromChain(const std::vector<int>& teethChain) {
  auto stages = BuildStagesFromChain(teethChain);
  return gearing::MechanismGearing{gearing::GearBox(stages)}.GetMechanismToRotorRatio();
}

double EasyCRTConfig::RatioFromDriverDrivenPairs(const std::vector<int>& pairs) {
  auto stages = BuildStagesFromDriverDrivenPairs(pairs);
  return gearing::MechanismGearing{gearing::GearBox(stages)}.GetMechanismToRotorRatio();
}

double EasyCRTConfig::RatioFromCommonDrive(double commonRatio, int driveGearTeeth,
                                           int encoderTeeth) {
  if (driveGearTeeth <= 0) throw std::invalid_argument("driveGearTeeth must be > 0");
  if (encoderTeeth <= 0) throw std::invalid_argument("encoderTeeth must be > 0");
  return commonRatio * (static_cast<double>(driveGearTeeth) / encoderTeeth);
}

bool EasyCRTConfig::IsCoprime(int a, int b) { return Gcd(std::abs(a), std::abs(b)) == 1; }

EasyCRTConfig::CrtGearPair EasyCRTConfig::FindSmallestCrtGearPair(
    int stage1GearTeeth, double stage2Ratio, ::units::angle::turn_t maxMechanismAngle,
    double coverageMargin, int minTeeth, int maxTeeth, int maxIterationsLimit) {
  double maxMechRot = maxMechanismAngle.value();
  if (stage1GearTeeth <= 0 || stage2Ratio <= 0.0 || minTeeth < 1 || maxTeeth < minTeeth) return {};

  double requiredCoverage = maxMechRot * coverageMargin;
  int requiredLcm =
      static_cast<int>(std::ceil(requiredCoverage * stage2Ratio * stage1GearTeeth - 1e-9));

  CrtGearPair best{};
  int bestMaxTeeth = std::numeric_limits<int>::max();
  int bestSumTeeth = std::numeric_limits<int>::max();
  int bestLcm = std::numeric_limits<int>::max();

  for (int a = minTeeth; a <= maxTeeth; ++a) {
    for (int b = a + 1; b <= maxTeeth; ++b) {
      int lcm = Lcm(a, b);
      if (lcm < requiredLcm) continue;
      int iterA = TheoreticalIterationsForGear(a, stage1GearTeeth, stage2Ratio, maxMechanismAngle);
      int iterB = TheoreticalIterationsForGear(b, stage1GearTeeth, stage2Ratio, maxMechanismAngle);
      bool aOk = (iterA <= maxIterationsLimit);
      bool bOk = (iterB <= maxIterationsLimit);
      if (!aOk && !bOk) continue;

      int assignedA = a, assignedB = b, assignedIter = iterA;
      if (bOk && (!aOk || iterB < iterA)) {
        assignedA = b;
        assignedB = a;
        assignedIter = iterB;
      }
      int candMax = b, sumT = a + b;
      if (candMax < bestMaxTeeth || (candMax == bestMaxTeeth && sumT < bestSumTeeth) ||
          (candMax == bestMaxTeeth && sumT == bestSumTeeth && lcm < bestLcm)) {
        bestMaxTeeth = candMax;
        bestSumTeeth = sumT;
        bestLcm = lcm;
        double covRot = static_cast<double>(lcm) / (stage2Ratio * stage1GearTeeth);
        best = {assignedA, assignedB, lcm, ::units::angle::turn_t{covRot}, Gcd(a, b), assignedIter};
      }
    }
  }
  return best;
}

int EasyCRTConfig::TheoreticalIterationsForGear(int gearTeeth, int stage1GearTeeth,
                                                double stage2Ratio,
                                                ::units::angle::turn_t maxMechAngle) {
  double ratio = stage2Ratio * (static_cast<double>(stage1GearTeeth) / gearTeeth);
  return static_cast<int>(std::ceil(ratio * maxMechAngle.value())) + 3;
}

std::vector<std::string> EasyCRTConfig::BuildStagesFromChain(const std::vector<int>& chain) {
  if (chain.size() < 2) throw std::invalid_argument("Gear chain must have >= 2 tooth counts");
  std::vector<std::string> stages;
  stages.reserve(chain.size() - 1);
  for (size_t i = 1; i < chain.size(); ++i) {
    stages.push_back(std::to_string(chain[i - 1]) + ":" + std::to_string(chain[i]));
  }
  return stages;
}

std::vector<std::string> EasyCRTConfig::BuildStagesFromDriverDrivenPairs(
    const std::vector<int>& pairs) {
  if (pairs.size() < 2 || (pairs.size() % 2) != 0)
    throw std::invalid_argument("Stages must be (driver,driven) pairs");
  std::vector<std::string> stages;
  stages.reserve(pairs.size() / 2);
  for (size_t i = 0; i < pairs.size(); i += 2) {
    stages.push_back(std::to_string(pairs[i]) + ":" + std::to_string(pairs[i + 1]));
  }
  return stages;
}

int EasyCRTConfig::Gcd(int a, int b) {
  a = std::abs(a);
  b = std::abs(b);
  while (b) {
    int t = b;
    b = a % b;
    a = t;
  }
  return a;
}

int EasyCRTConfig::Lcm(int a, int b) { return (a / Gcd(a, b)) * b; }

}  // namespace yams::units
