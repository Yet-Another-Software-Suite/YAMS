// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

// Mirrors Java SmartMotorFactoryTest — verifies that SmartMotorFactory::Create
// returns a non-null wrapper of the correct concrete type for each supported
// motor controller.
//
// The Java version uses Mockito to stub out the construction; the C++ version
// creates real hardware objects under simulation (HAL must be initialised).

#include <frc/system/plant/DCMotor.h>
#include <gtest/gtest.h>
#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/TalonFXS.hpp>
#include <memory>

#include "helpers/MockHardware.h"
#include "helpers/MotorControllerFactory.h"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/SmartMotorFactory.hpp"
#include "yams/motorcontrollers/local/SparkWrapper.hpp"
#include "yams/motorcontrollers/remote/TalonFXSWrapper.hpp"
#include "yams/motorcontrollers/remote/TalonFXWrapper.hpp"

namespace yams::test {

using namespace motorcontrollers;

class SmartMotorFactoryTest : public ::testing::Test {
 protected:
  void SetUp() override { InitializeHardware(); }
  void TearDown() override { TeardownHardware(); }

  static SmartMotorControllerConfig BasicConfig() { return SmartMotorControllerConfig{}; }
};

TEST_F(SmartMotorFactoryTest, CreateTalonFXWrapperIsNonNull) {
  ctre::phoenix6::hardware::TalonFX talon{NextCanId()};
  auto smc = SmartMotorFactory::Create(talon, frc::DCMotor::KrakenX60(1), BasicConfig());
  EXPECT_NE(smc, nullptr) << "Factory should create a TalonFXWrapper";
}

TEST_F(SmartMotorFactoryTest, CreateTalonFXWrapperIsCorrectType) {
  ctre::phoenix6::hardware::TalonFX talon{NextCanId()};
  auto smc = SmartMotorFactory::Create(talon, frc::DCMotor::KrakenX60(1), BasicConfig());
  ASSERT_NE(smc, nullptr);
  EXPECT_NE(dynamic_cast<remote::TalonFXWrapper*>(smc.get()), nullptr)
      << "Factory should produce a TalonFXWrapper";
}

TEST_F(SmartMotorFactoryTest, CreateTalonFXSWrapperIsNonNull) {
  ctre::phoenix6::hardware::TalonFXS talon{NextCanId()};
  auto smc = SmartMotorFactory::Create(
      talon, frc::DCMotor::NEO(1), remote::TalonFXSWrapper::MotorArrangement::NEO, BasicConfig());
  EXPECT_NE(smc, nullptr) << "Factory should create a TalonFXSWrapper";
}

TEST_F(SmartMotorFactoryTest, CreateTalonFXSWrapperIsCorrectType) {
  ctre::phoenix6::hardware::TalonFXS talon{NextCanId()};
  auto smc = SmartMotorFactory::Create(
      talon, frc::DCMotor::NEO(1), remote::TalonFXSWrapper::MotorArrangement::NEO, BasicConfig());
  ASSERT_NE(smc, nullptr);
  EXPECT_NE(dynamic_cast<remote::TalonFXSWrapper*>(smc.get()), nullptr)
      << "Factory should produce a TalonFXSWrapper";
}

TEST_F(SmartMotorFactoryTest, CreateSparkMaxWrapperIsNonNull) {
  rev::spark::SparkMax spark{NextCanId(), rev::spark::SparkLowLevel::MotorType::kBrushless};
  auto smc = SmartMotorFactory::Create(spark, frc::DCMotor::NEO(1), BasicConfig());
  EXPECT_NE(smc, nullptr) << "Factory should create a SparkWrapper for SparkMax";
}

TEST_F(SmartMotorFactoryTest, CreateSparkMaxWrapperIsCorrectType) {
  rev::spark::SparkMax spark{NextCanId(), rev::spark::SparkLowLevel::MotorType::kBrushless};
  auto smc = SmartMotorFactory::Create(spark, frc::DCMotor::NEO(1), BasicConfig());
  ASSERT_NE(smc, nullptr);
  EXPECT_NE(dynamic_cast<local::SparkWrapper*>(smc.get()), nullptr)
      << "Factory should produce a SparkWrapper";
}

TEST_F(SmartMotorFactoryTest, CreateSparkFlexWrapperIsNonNull) {
  rev::spark::SparkFlex spark{NextCanId(), rev::spark::SparkLowLevel::MotorType::kBrushless};
  auto smc = SmartMotorFactory::Create(spark, frc::DCMotor::NeoVortex(1), BasicConfig());
  EXPECT_NE(smc, nullptr) << "Factory should create a SparkWrapper for SparkFlex";
}

TEST_F(SmartMotorFactoryTest, CreateSparkFlexWrapperIsCorrectType) {
  rev::spark::SparkFlex spark{NextCanId(), rev::spark::SparkLowLevel::MotorType::kBrushless};
  auto smc = SmartMotorFactory::Create(spark, frc::DCMotor::NeoVortex(1), BasicConfig());
  ASSERT_NE(smc, nullptr);
  EXPECT_NE(dynamic_cast<local::SparkWrapper*>(smc.get()), nullptr)
      << "Factory should produce a SparkWrapper";
}

}  // namespace yams::test
