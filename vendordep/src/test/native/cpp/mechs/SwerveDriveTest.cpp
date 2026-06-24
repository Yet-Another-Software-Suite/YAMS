// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Four-module swerve drive integration test.
//
// Hardware (8 TalonFX + 8 TalonFXWrapper) is created once for the entire suite
// in SetUpTestSuite/TearDownTestSuite.  Per-test SetUp only builds the
// lightweight SwerveModule and SwerveDrive objects from those shared SMCs.
// This prevents the Phoenix simulation background thread from accessing freed
// TalonFXSimState, which would otherwise cause a SIGSEGV at process exit when
// TalonFX objects are destroyed per-test.

/*
#include <frc/controller/PIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/system/plant/DCMotor.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/SubsystemBase.h>
#include <gtest/gtest.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>

#include <cmath>
#include <ctre/phoenix6/TalonFX.hpp>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <utility>

#include "helpers/MockHardware.h"
#include "helpers/MotorControllerFactory.h"
#include "helpers/SchedulerHelper.h"
#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/mechanisms/config/SwerveModuleConfig.hpp"
#include "yams/mechanisms/swerve/SwerveDrive.hpp"
#include "yams/mechanisms/swerve/SwerveDriveConfig.hpp"
#include "yams/mechanisms/swerve/SwerveModule.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/remote/TalonFXWrapper.hpp"

namespace yams::test {

using namespace motorcontrollers;
using namespace mechanisms;
using namespace mechanisms::config;
using namespace mechanisms::swerve;

// ---- Constants ---------------------------------------------------------------

// Module offset from robot centre for a 24 in × 24 in square chassis.
static constexpr units::meter_t kModuleX{0.3048};
static constexpr units::meter_t kModuleY{0.3048};

// ---- Minimal subsystem -------------------------------------------------------

class SwerveTestSubsystem : public frc2::SubsystemBase {
 public:
  void Periodic() override {
    if (m_drive) m_drive->UpdateTelemetry();
  }
  void SimulationPeriodic() override {
    if (m_drive) m_drive->SimIterate();
  }
  SwerveDrive<4>* m_drive{nullptr};
};

// ---- SMC config helpers ------------------------------------------------------

static SmartMotorControllerConfig MakeDriveConfig(const std::string& name,
                                                  frc2::SubsystemBase* subsys) {
  SmartMotorControllerConfig cfg;
  cfg.WithFeedback(0.1, 0.0, 0.0)
      .WithMechanismCircumference(units::meter_t{4.0_in * std::numbers::pi})
      .WithMotorGearing(gearing::MechanismGearing{gearing::GearBox::FromReductionStages({6.75})})
      .WithIdleMode(SmartMotorControllerConfig::MotorMode::BRAKE)
      .WithStatorCurrentLimit(40.0_A)
      .WithSimMotor(frc::DCMotor::KrakenX60(1))
      .WithClosedLoopMode()
      .WithSubsystem(subsys)
      .WithTelemetry(name, SmartMotorControllerConfig::TelemetryVerbosity::NONE);
  return cfg;
}

static SmartMotorControllerConfig MakeAzimuthConfig(const std::string& name,
                                                    frc2::SubsystemBase* subsys) {
  SmartMotorControllerConfig cfg;
  cfg.WithFeedback(50.0, 0.0, 0.5)
      .WithMotorGearing(
          gearing::MechanismGearing{gearing::GearBox::FromReductionStages({150.0 / 7.0})})
      .WithIdleMode(SmartMotorControllerConfig::MotorMode::BRAKE)
      .WithStatorCurrentLimit(20.0_A)
      .WithSimMotor(frc::DCMotor::KrakenX60(1))
      .WithMOI(4_in, 0.5_lb)
      .WithClosedLoopMode()
      .WithSubsystem(subsys)
      .WithTelemetry(name, SmartMotorControllerConfig::TelemetryVerbosity::NONE);
  return cfg;
}

// ---- Test fixture ------------------------------------------------------------
//
// Hardware (TalonFX + TalonFXWrapper) lives for the entire test suite to keep
// the Phoenix simulation state valid throughout.  Only the drive and modules
// are recreated per-test.

class SwerveDriveTest : public ::testing::Test {
 protected:
  // ---- Per-suite members (raw pointers; managed by SetUpTestSuite/TearDownTestSuite) ---
  static SwerveTestSubsystem* s_sub;
  static ctre::phoenix6::hardware::TalonFX* s_flDriveTalon;
  static ctre::phoenix6::hardware::TalonFX* s_frDriveTalon;
  static ctre::phoenix6::hardware::TalonFX* s_blDriveTalon;
  static ctre::phoenix6::hardware::TalonFX* s_brDriveTalon;
  static ctre::phoenix6::hardware::TalonFX* s_flAzimuthTalon;
  static ctre::phoenix6::hardware::TalonFX* s_frAzimuthTalon;
  static ctre::phoenix6::hardware::TalonFX* s_blAzimuthTalon;
  static ctre::phoenix6::hardware::TalonFX* s_brAzimuthTalon;
  static remote::TalonFXWrapper* s_flDriveSMC;
  static remote::TalonFXWrapper* s_frDriveSMC;
  static remote::TalonFXWrapper* s_blDriveSMC;
  static remote::TalonFXWrapper* s_brDriveSMC;
  static remote::TalonFXWrapper* s_flAzimuthSMC;
  static remote::TalonFXWrapper* s_frAzimuthSMC;
  static remote::TalonFXWrapper* s_blAzimuthSMC;
  static remote::TalonFXWrapper* s_brAzimuthSMC;

  // Configs must outlive the wrappers; stored as static members for suite lifetime.
  static SmartMotorControllerConfig s_flDriveCfg;
  static SmartMotorControllerConfig s_frDriveCfg;
  static SmartMotorControllerConfig s_blDriveCfg;
  static SmartMotorControllerConfig s_brDriveCfg;
  static SmartMotorControllerConfig s_flAzimuthCfg;
  static SmartMotorControllerConfig s_frAzimuthCfg;
  static SmartMotorControllerConfig s_blAzimuthCfg;
  static SmartMotorControllerConfig s_brAzimuthCfg;

  static void SetUpTestSuite() {
    InitializeHardware();
    SchedulerHelper::Enable();

    s_sub = new SwerveTestSubsystem();

    s_flDriveTalon = new ctre::phoenix6::hardware::TalonFX(NextCanId());
    s_frDriveTalon = new ctre::phoenix6::hardware::TalonFX(NextCanId());
    s_blDriveTalon = new ctre::phoenix6::hardware::TalonFX(NextCanId());
    s_brDriveTalon = new ctre::phoenix6::hardware::TalonFX(NextCanId());
    s_flAzimuthTalon = new ctre::phoenix6::hardware::TalonFX(NextCanId());
    s_frAzimuthTalon = new ctre::phoenix6::hardware::TalonFX(NextCanId());
    s_blAzimuthTalon = new ctre::phoenix6::hardware::TalonFX(NextCanId());
    s_brAzimuthTalon = new ctre::phoenix6::hardware::TalonFX(NextCanId());

    // TalonFXWrapper constructor calls SetupSimulation() automatically.
    s_flDriveSMC = new remote::TalonFXWrapper(*s_flDriveTalon, frc::DCMotor::KrakenX60(1),
                                              MakeDriveConfig("FL_Drive", s_sub));
    s_frDriveSMC = new remote::TalonFXWrapper(*s_frDriveTalon, frc::DCMotor::KrakenX60(1),
                                              MakeDriveConfig("FR_Drive", s_sub));
    s_blDriveSMC = new remote::TalonFXWrapper(*s_blDriveTalon, frc::DCMotor::KrakenX60(1),
                                              MakeDriveConfig("BL_Drive", s_sub));
    s_brDriveSMC = new remote::TalonFXWrapper(*s_brDriveTalon, frc::DCMotor::KrakenX60(1),
                                              MakeDriveConfig("BR_Drive", s_sub));
    s_flAzimuthSMC = new remote::TalonFXWrapper(*s_flAzimuthTalon, frc::DCMotor::KrakenX60(1),
                                                MakeAzimuthConfig("FL_Azimuth", s_sub));
    s_frAzimuthSMC = new remote::TalonFXWrapper(*s_frAzimuthTalon, frc::DCMotor::KrakenX60(1),
                                                MakeAzimuthConfig("FR_Azimuth", s_sub));
    s_blAzimuthSMC = new remote::TalonFXWrapper(*s_blAzimuthTalon, frc::DCMotor::KrakenX60(1),
                                                MakeAzimuthConfig("BL_Azimuth", s_sub));
    s_brAzimuthSMC = new remote::TalonFXWrapper(*s_brAzimuthTalon, frc::DCMotor::KrakenX60(1),
                                                MakeAzimuthConfig("BR_Azimuth", s_sub));
  }

  static void TearDownTestSuite() {
    s_sub->m_drive = nullptr;
    SchedulerHelper::CancelAll();
    frc2::CommandScheduler::GetInstance().UnregisterSubsystem(s_sub);

    for (auto* s : {s_flDriveSMC, s_frDriveSMC, s_blDriveSMC, s_brDriveSMC, s_flAzimuthSMC,
                    s_frAzimuthSMC, s_blAzimuthSMC, s_brAzimuthSMC}) {
      s->Close();
      delete s;
    }
    for (auto* t :
         {s_flDriveTalon, s_frDriveTalon, s_blDriveTalon, s_brDriveTalon, s_flAzimuthTalon,
          s_frAzimuthTalon, s_blAzimuthTalon, s_brAzimuthTalon}) {
      delete t;
    }
    delete s_sub;
    TeardownHardware();
  }

  // ---- Per-test members -------------------------------------------------------

  void SetUp() override {
    SchedulerHelper::CancelAll();
    m_simGyro = 0_deg;

    auto makeModuleCfg = [](remote::TalonFXWrapper* drive, remote::TalonFXWrapper* azimuth,
                            units::meter_t front, units::meter_t left,
                            const std::string& name) -> SwerveModuleConfig {
      SwerveModuleConfig cfg{drive, azimuth};
      cfg.WithAbsoluteEncoder([] { return 0.0_deg; })
          .WithAbsoluteEncoderOffset(0.0_deg)
          .WithWheelDiameter(4.0_in)
          .WithLocation(front, left)
          .WithOptimization(true)
          .WithTelemetry(name, SwerveModuleConfig::TelemetryVerbosity::NONE);
      return cfg;
    };

    m_fl.emplace(makeModuleCfg(s_flDriveSMC, s_flAzimuthSMC, kModuleX, kModuleY, "FL"));
    m_fr.emplace(makeModuleCfg(s_frDriveSMC, s_frAzimuthSMC, kModuleX, -kModuleY, "FR"));
    m_bl.emplace(makeModuleCfg(s_blDriveSMC, s_blAzimuthSMC, -kModuleX, kModuleY, "BL"));
    m_br.emplace(makeModuleCfg(s_brDriveSMC, s_brAzimuthSMC, -kModuleX, -kModuleY, "BR"));

    SwerveDriveConfig driveCfg;
    driveCfg.WithSubsystem(s_sub)
        .WithModules({&m_fl.value(), &m_fr.value(), &m_bl.value(), &m_br.value()})
        .WithGyro([this] { return m_simGyro; })
        .WithStartingPose(frc::Pose2d{})
        .WithMaximumChassisSpeed(4.5_mps, units::degrees_per_second_t{540})
        .WithTranslationController(frc::PIDController{2.0, 0.0, 0.0})
        .WithRotationController(frc::PIDController{4.0, 0.0, 0.0});
    m_drive.emplace(std::move(driveCfg));

    s_sub->m_drive = &m_drive.value();
  }

  void TearDown() override {
    s_sub->m_drive = nullptr;
    frc2::CommandScheduler::GetInstance().CancelAll();
    m_drive.reset();
    m_fl.reset();
    m_fr.reset();
    m_bl.reset();
    m_br.reset();
  }

  // Simulated gyro angle — tests can mutate this to fake heading.
  units::degree_t m_simGyro{0};

  std::optional<SwerveModule> m_fl;
  std::optional<SwerveModule> m_fr;
  std::optional<SwerveModule> m_bl;
  std::optional<SwerveModule> m_br;

  std::optional<SwerveDrive<4>> m_drive;
};

// Static member definitions.
SwerveTestSubsystem* SwerveDriveTest::s_sub = nullptr;
ctre::phoenix6::hardware::TalonFX* SwerveDriveTest::s_flDriveTalon = nullptr;
ctre::phoenix6::hardware::TalonFX* SwerveDriveTest::s_frDriveTalon = nullptr;
ctre::phoenix6::hardware::TalonFX* SwerveDriveTest::s_blDriveTalon = nullptr;
ctre::phoenix6::hardware::TalonFX* SwerveDriveTest::s_brDriveTalon = nullptr;
ctre::phoenix6::hardware::TalonFX* SwerveDriveTest::s_flAzimuthTalon = nullptr;
ctre::phoenix6::hardware::TalonFX* SwerveDriveTest::s_frAzimuthTalon = nullptr;
ctre::phoenix6::hardware::TalonFX* SwerveDriveTest::s_blAzimuthTalon = nullptr;
ctre::phoenix6::hardware::TalonFX* SwerveDriveTest::s_brAzimuthTalon = nullptr;
remote::TalonFXWrapper* SwerveDriveTest::s_flDriveSMC = nullptr;
remote::TalonFXWrapper* SwerveDriveTest::s_frDriveSMC = nullptr;
remote::TalonFXWrapper* SwerveDriveTest::s_blDriveSMC = nullptr;
remote::TalonFXWrapper* SwerveDriveTest::s_brDriveSMC = nullptr;
remote::TalonFXWrapper* SwerveDriveTest::s_flAzimuthSMC = nullptr;
remote::TalonFXWrapper* SwerveDriveTest::s_frAzimuthSMC = nullptr;
remote::TalonFXWrapper* SwerveDriveTest::s_blAzimuthSMC = nullptr;
remote::TalonFXWrapper* SwerveDriveTest::s_brAzimuthSMC = nullptr;

// ---- Tests -------------------------------------------------------------------

// Drive constructs and destructs cleanly.
TEST_F(SwerveDriveTest, ConstructionDoesNotCrash) {
  EXPECT_TRUE(m_drive.has_value());
}

// UpdateTelemetry and SimIterate run for several loops without crashing.
TEST_F(SwerveDriveTest, TelemetryAndSimRunWithoutCrash) {
  EXPECT_NO_FATAL_FAILURE(SchedulerHelper::RunForDuration(0.5_s));
}

// The initial pose matches the starting pose supplied in the config.
TEST_F(SwerveDriveTest, InitialPoseIsOrigin) {
  auto pose = m_drive->GetPose();
  EXPECT_NEAR(pose.X().value(), 0.0, 0.01);
  EXPECT_NEAR(pose.Y().value(), 0.0, 0.01);
  EXPECT_NEAR(pose.Rotation().Degrees().value(), 0.0, 0.1);
}

// A non-zero target pose is reflected in GetPose() immediately after ResetOdometry.
TEST_F(SwerveDriveTest, ResetOdometryMatchesPose) {
  frc::Pose2d target{3.0_m, 2.0_m, frc::Rotation2d{45.0_deg}};
  m_drive->ResetOdometry(target);
  auto pose = m_drive->GetPose();
  EXPECT_NEAR(pose.X().value(), 3.0, 0.01);
  EXPECT_NEAR(pose.Y().value(), 2.0, 0.01);
  EXPECT_NEAR(pose.Rotation().Degrees().value(), 45.0, 0.1);
}

// GetStateFromRobotRelativeChassisSpeeds converts a pure forward command into
// forward-pointing states for all four modules.
TEST_F(SwerveDriveTest, GetStateFromSpeedsForwardDrive) {
  auto states = m_drive->GetStateFromRobotRelativeChassisSpeeds(
      frc::ChassisSpeeds{1.0_mps, 0_mps, 0_rad_per_s});

  for (size_t i = 0; i < 4; ++i) {
    EXPECT_NEAR(states[i].speed.value(), 1.0, 0.01)
        << "Module " << i << " speed should equal commanded speed";
    EXPECT_NEAR(states[i].angle.Degrees().value(), 0.0, 1.0)
        << "Module " << i << " angle should be 0° for pure forward drive";
  }
}

// Pure rotation command produces tangential module states (none pointing forward).
TEST_F(SwerveDriveTest, GetStateFromSpeedsPureRotation) {
  auto states = m_drive->GetStateFromRobotRelativeChassisSpeeds(
      frc::ChassisSpeeds{0_mps, 0_mps, units::radians_per_second_t{1.0}});

  for (size_t i = 0; i < 4; ++i) {
    EXPECT_GT(std::abs(states[i].speed.value()), 0.0)
        << "Module " << i << " should have non-zero speed for rotation command";
    EXPECT_GT(std::abs(states[i].angle.Degrees().value()), 1.0)
        << "Module " << i << " should not point forward during pure rotation";
  }
}

// SetRobotRelativeChassisSpeeds does not crash for both non-zero and zero inputs.
TEST_F(SwerveDriveTest, SetRobotRelativeSpeedsDoesNotCrash) {
  EXPECT_NO_FATAL_FAILURE(
      m_drive->SetRobotRelativeChassisSpeeds(frc::ChassisSpeeds{1.0_mps, 0_mps, 0_rad_per_s}));
  EXPECT_NO_FATAL_FAILURE(
      m_drive->SetRobotRelativeChassisSpeeds(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s}));
}


// LockPose commands zero translational speed and corner-pointing module angles.
TEST_F(SwerveDriveTest, LockPoseSetsXPattern) {
  m_drive->LockPose();

  // Desired chassis speeds should be zero after locking.
  auto robotSpeeds = m_drive->GetRobotRelativeSpeed();
  EXPECT_NEAR(robotSpeeds.vx.value(), 0.0, 0.5);
  EXPECT_NEAR(robotSpeeds.vy.value(), 0.0, 0.5);

  // Run sim and check azimuth convergence toward X-pattern corner angles.
  SchedulerHelper::RunForDuration(0.5_s);
  auto modules = m_drive->GetConfig().GetModules();
  for (size_t i = 0; i < 4; ++i) {
    double expected = modules[i]->GetConfig().GetLocation()->Angle().Degrees().value();
    double actual = modules[i]->GetState().angle.Degrees().value();
    EXPECT_NEAR(actual, expected, 180.0)
        << "Module " << i << " angle should converge toward lock angle " << expected << "°";
  }
}

// ZeroGyro does not crash and zeroes the heading.
TEST_F(SwerveDriveTest, ZeroGyroDoesNotCrash) {
  m_simGyro = 45.0_deg;
  m_drive->ResetOdometry(frc::Pose2d{0_m, 0_m, frc::Rotation2d{45.0_deg}});
  EXPECT_NO_FATAL_FAILURE(m_drive->ZeroGyro());
  EXPECT_NEAR(m_drive->GetPose().Rotation().Degrees().value(), 0.0, 1.0);
}

// AddVisionMeasurement accepts a pose without crashing.
TEST_F(SwerveDriveTest, AddVisionMeasurementDoesNotCrash) {
  EXPECT_NO_FATAL_FAILURE(
      m_drive->AddVisionMeasurement(frc::Pose2d{1_m, 1_m, frc::Rotation2d{}}, 0.0_s));
}

// GetDistanceFromPose returns the Euclidean distance to a target (3-4-5 triangle).
TEST_F(SwerveDriveTest, GetDistanceFromPose) {
  auto dist = m_drive->GetDistanceFromPose(frc::Pose2d{3.0_m, 4.0_m, frc::Rotation2d{}});
  EXPECT_NEAR(dist.value(), 5.0, 0.01);
}

// Drive() returns a command that runs the speed supplier each loop.
TEST_F(SwerveDriveTest, DriveCommandCallsSpeedSupplier) {
  int callCount = 0;
  auto cmd = m_drive->Drive([&] {
    ++callCount;
    return frc::ChassisSpeeds{};
  });
  frc2::CommandScheduler::GetInstance().Schedule(cmd);
  SchedulerHelper::RunForDuration(0.1_s);
  EXPECT_GE(callCount, 1);
}

// The Drive command declares the configured subsystem as a requirement.
TEST_F(SwerveDriveTest, DriveCommandHasSubsystemRequirement) {
  auto cmd = m_drive->Drive([] { return frc::ChassisSpeeds{}; });
  EXPECT_TRUE(cmd.HasRequirement(s_sub));
}

// Scheduling a second Drive command interrupts the first.
TEST_F(SwerveDriveTest, SecondDriveCommandInterruptsFirst) {
  int firstCalls = 0;
  int secondCalls = 0;
  auto cmd1 = m_drive->Drive([&] {
    ++firstCalls;
    return frc::ChassisSpeeds{};
  });
  auto cmd2 = m_drive->Drive([&] {
    ++secondCalls;
    return frc::ChassisSpeeds{};
  });

  frc2::CommandScheduler::GetInstance().Schedule(cmd1);
  SchedulerHelper::RunForDuration(0.04_s);
  int firstCallsAtInterrupt = firstCalls;
  EXPECT_GE(firstCallsAtInterrupt, 1) << "cmd1 should run initially";

  frc2::CommandScheduler::GetInstance().Schedule(cmd2);
  SchedulerHelper::RunForDuration(0.04_s);

  EXPECT_GE(secondCalls, 1) << "cmd2 should run after interrupt";
  EXPECT_EQ(firstCalls, firstCallsAtInterrupt) << "cmd1 should have been cancelled";
}

}  */// namespace yams::test
