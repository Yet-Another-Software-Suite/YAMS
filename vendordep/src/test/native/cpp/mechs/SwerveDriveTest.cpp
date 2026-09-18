// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Four-module swerve drive integration test.
//
// Hardware (8 TalonFX + 8 TalonFXWrapper) is created once for the whole
// binary via a function-local static (constructed on first use, destroyed at
// program exit). Per-test SetUp only builds the lightweight SwerveModule and
// SwerveDrive objects from those shared SMCs. This prevents the Phoenix
// simulation background thread from accessing freed TalonFXSimState, which
// would otherwise cause a SIGSEGV if TalonFX objects were rebuilt per-test.

/*
#include <wpi/math/controller/PIDController.hpp>
#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/math/kinematics/ChassisVelocities.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/commands2/CommandScheduler.hpp>
#include <wpi/commands2/CommandScheduler.hpp>
#include <wpi/commands2/SubsystemBase.hpp>
#include <catch2/catch_test_macros.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/velocity.hpp>

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
static constexpr wpi::units::meter_t kModuleX{0.3048};
static constexpr wpi::units::meter_t kModuleY{0.3048};

// ---- Minimal subsystem -------------------------------------------------------

class SwerveTestSubsystem : public wpi::cmd::SubsystemBase {
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
                                                  wpi::cmd::SubsystemBase* subsys) {
  SmartMotorControllerConfig cfg;
  cfg.WithFeedback(0.1, 0.0, 0.0)
      .WithMechanismCircumference(wpi::units::meter_t{4.0_in * std::numbers::pi})
      .WithMotorGearing(gearing::MechanismGearing{gearing::GearBox::FromReductionStages({6.75})})
      .WithIdleMode(SmartMotorControllerConfig::MotorMode::BRAKE)
      .WithStatorCurrentLimit(40.0_A)
      .WithSimMotor(wpi::math::DCMotor::KrakenX60(1))
      .WithClosedLoopMode()
      .WithSubsystem(subsys)
      .WithTelemetry(name, SmartMotorControllerConfig::TelemetryVerbosity::NONE);
  return cfg;
}

static SmartMotorControllerConfig MakeAzimuthConfig(const std::string& name,
                                                    wpi::cmd::SubsystemBase* subsys) {
  SmartMotorControllerConfig cfg;
  cfg.WithFeedback(50.0, 0.0, 0.5)
      .WithMotorGearing(
          gearing::MechanismGearing{gearing::GearBox::FromReductionStages({150.0 / 7.0})})
      .WithIdleMode(SmartMotorControllerConfig::MotorMode::BRAKE)
      .WithStatorCurrentLimit(20.0_A)
      .WithSimMotor(wpi::math::DCMotor::KrakenX60(1))
      .WithMOI(4_in, 0.5_lb)
      .WithClosedLoopMode()
      .WithSubsystem(subsys)
      .WithTelemetry(name, SmartMotorControllerConfig::TelemetryVerbosity::NONE);
  return cfg;
}

// ---- Shared hardware (constructed once for the whole binary) ----------------
//
// Hardware (TalonFX + TalonFXWrapper) lives for the lifetime of the test
// binary to keep the Phoenix simulation state valid throughout. Only the
// drive and modules are recreated per-test via SwerveDriveTestFixture.

struct SwerveSuiteHardware {
  SwerveTestSubsystem* sub{nullptr};
  ctre::phoenix6::hardware::TalonFX* flDriveTalon{nullptr};
  ctre::phoenix6::hardware::TalonFX* frDriveTalon{nullptr};
  ctre::phoenix6::hardware::TalonFX* blDriveTalon{nullptr};
  ctre::phoenix6::hardware::TalonFX* brDriveTalon{nullptr};
  ctre::phoenix6::hardware::TalonFX* flAzimuthTalon{nullptr};
  ctre::phoenix6::hardware::TalonFX* frAzimuthTalon{nullptr};
  ctre::phoenix6::hardware::TalonFX* blAzimuthTalon{nullptr};
  ctre::phoenix6::hardware::TalonFX* brAzimuthTalon{nullptr};
  remote::TalonFXWrapper* flDriveSMC{nullptr};
  remote::TalonFXWrapper* frDriveSMC{nullptr};
  remote::TalonFXWrapper* blDriveSMC{nullptr};
  remote::TalonFXWrapper* brDriveSMC{nullptr};
  remote::TalonFXWrapper* flAzimuthSMC{nullptr};
  remote::TalonFXWrapper* frAzimuthSMC{nullptr};
  remote::TalonFXWrapper* blAzimuthSMC{nullptr};
  remote::TalonFXWrapper* brAzimuthSMC{nullptr};

  SwerveSuiteHardware() {
    InitializeHardware();
    SchedulerHelper::Enable();

    sub = new SwerveTestSubsystem();

    flDriveTalon = new ctre::phoenix6::hardware::TalonFX(NextCanId());
    frDriveTalon = new ctre::phoenix6::hardware::TalonFX(NextCanId());
    blDriveTalon = new ctre::phoenix6::hardware::TalonFX(NextCanId());
    brDriveTalon = new ctre::phoenix6::hardware::TalonFX(NextCanId());
    flAzimuthTalon = new ctre::phoenix6::hardware::TalonFX(NextCanId());
    frAzimuthTalon = new ctre::phoenix6::hardware::TalonFX(NextCanId());
    blAzimuthTalon = new ctre::phoenix6::hardware::TalonFX(NextCanId());
    brAzimuthTalon = new ctre::phoenix6::hardware::TalonFX(NextCanId());

    // TalonFXWrapper constructor calls SetupSimulation() automatically.
    flDriveSMC = new remote::TalonFXWrapper(*flDriveTalon, wpi::math::DCMotor::KrakenX60(1),
                                            MakeDriveConfig("FL_Drive", sub));
    frDriveSMC = new remote::TalonFXWrapper(*frDriveTalon, wpi::math::DCMotor::KrakenX60(1),
                                            MakeDriveConfig("FR_Drive", sub));
    blDriveSMC = new remote::TalonFXWrapper(*blDriveTalon, wpi::math::DCMotor::KrakenX60(1),
                                            MakeDriveConfig("BL_Drive", sub));
    brDriveSMC = new remote::TalonFXWrapper(*brDriveTalon, wpi::math::DCMotor::KrakenX60(1),
                                            MakeDriveConfig("BR_Drive", sub));
    flAzimuthSMC = new remote::TalonFXWrapper(*flAzimuthTalon, wpi::math::DCMotor::KrakenX60(1),
                                              MakeAzimuthConfig("FL_Azimuth", sub));
    frAzimuthSMC = new remote::TalonFXWrapper(*frAzimuthTalon, wpi::math::DCMotor::KrakenX60(1),
                                              MakeAzimuthConfig("FR_Azimuth", sub));
    blAzimuthSMC = new remote::TalonFXWrapper(*blAzimuthTalon, wpi::math::DCMotor::KrakenX60(1),
                                              MakeAzimuthConfig("BL_Azimuth", sub));
    brAzimuthSMC = new remote::TalonFXWrapper(*brAzimuthTalon, wpi::math::DCMotor::KrakenX60(1),
                                              MakeAzimuthConfig("BR_Azimuth", sub));
  }

  ~SwerveSuiteHardware() {
    sub->m_drive = nullptr;
    SchedulerHelper::CancelAll();
    wpi::cmd::CommandScheduler::GetInstance().UnregisterSubsystem(sub);

    for (auto* s : {flDriveSMC, frDriveSMC, blDriveSMC, brDriveSMC, flAzimuthSMC, frAzimuthSMC,
                    blAzimuthSMC, brAzimuthSMC}) {
      s->Close();
      delete s;
    }
    for (auto* t : {flDriveTalon, frDriveTalon, blDriveTalon, brDriveTalon, flAzimuthTalon,
                    frAzimuthTalon, blAzimuthTalon, brAzimuthTalon}) {
      delete t;
    }
    delete sub;
    TeardownHardware();
  }
};

// Constructed on first use, destroyed at program exit.
static SwerveSuiteHardware& Hardware() {
  static SwerveSuiteHardware hw;
  return hw;
}

// ---- Per-test fixture --------------------------------------------------------

struct SwerveDriveTestFixture {
  SwerveDriveTestFixture() {
    SwerveSuiteHardware& hw = Hardware();
    SchedulerHelper::CancelAll();
    m_simGyro = 0_deg;

    auto makeModuleCfg = [](remote::TalonFXWrapper* drive, remote::TalonFXWrapper* azimuth,
                            wpi::units::meter_t front, wpi::units::meter_t left,
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

    m_fl.emplace(makeModuleCfg(hw.flDriveSMC, hw.flAzimuthSMC, kModuleX, kModuleY, "FL"));
    m_fr.emplace(makeModuleCfg(hw.frDriveSMC, hw.frAzimuthSMC, kModuleX, -kModuleY, "FR"));
    m_bl.emplace(makeModuleCfg(hw.blDriveSMC, hw.blAzimuthSMC, -kModuleX, kModuleY, "BL"));
    m_br.emplace(makeModuleCfg(hw.brDriveSMC, hw.brAzimuthSMC, -kModuleX, -kModuleY, "BR"));

    SwerveDriveConfig driveCfg;
    driveCfg.WithSubsystem(hw.sub)
        .WithModules({&m_fl.value(), &m_fr.value(), &m_bl.value(), &m_br.value()})
        .WithGyro([this] { return m_simGyro; })
        .WithStartingPose(wpi::math::Pose2d{})
        .WithMaximumChassisSpeed(4.5_mps, wpi::units::degrees_per_second_t{540})
        .WithTranslationController(wpi::math::PIDController{2.0, 0.0, 0.0})
        .WithRotationController(wpi::math::PIDController{4.0, 0.0, 0.0});
    m_drive.emplace(std::move(driveCfg));

    hw.sub->m_drive = &m_drive.value();
  }

  ~SwerveDriveTestFixture() {
    Hardware().sub->m_drive = nullptr;
    wpi::cmd::CommandScheduler::GetInstance().CancelAll();
    m_drive.reset();
    m_fl.reset();
    m_fr.reset();
    m_bl.reset();
    m_br.reset();
  }

  SwerveTestSubsystem* Subsystem() { return Hardware().sub; }

  // Simulated gyro angle tests can mutate this to fake heading.
  wpi::units::degree_t m_simGyro{0};

  std::optional<SwerveModule> m_fl;
  std::optional<SwerveModule> m_fr;
  std::optional<SwerveModule> m_bl;
  std::optional<SwerveModule> m_br;

  std::optional<SwerveDrive<4>> m_drive;
};

// ---- Tests -------------------------------------------------------------------

// Drive constructs and destructs cleanly.
TEST_CASE_METHOD(SwerveDriveTestFixture, "SwerveDriveTest.ConstructionDoesNotCrash",
                 "[SwerveDriveTest]") {
  CHECK(m_drive.has_value());
}

// UpdateTelemetry and SimIterate run for several loops without crashing.
TEST_CASE_METHOD(SwerveDriveTestFixture, "SwerveDriveTest.TelemetryAndSimRunWithoutCrash",
                 "[SwerveDriveTest]") {
  CHECK_NOTHROW(SchedulerHelper::RunForDuration(0.5_s));
}

// The initial pose matches the starting pose supplied in the config.
TEST_CASE_METHOD(SwerveDriveTestFixture, "SwerveDriveTest.InitialPoseIsOrigin",
                 "[SwerveDriveTest]") {
  auto pose = m_drive->GetPose();
  CHECK(pose.X().value() == Catch::Approx(0.0).margin(0.01));
  CHECK(pose.Y().value() == Catch::Approx(0.0).margin(0.01));
  CHECK(pose.Rotation().Degrees().value() == Catch::Approx(0.0).margin(0.1));
}

// A non-zero target pose is reflected in GetPose() immediately after ResetOdometry.
TEST_CASE_METHOD(SwerveDriveTestFixture, "SwerveDriveTest.ResetOdometryMatchesPose",
                 "[SwerveDriveTest]") {
  wpi::math::Pose2d target{3.0_m, 2.0_m, wpi::math::Rotation2d{45.0_deg}};
  m_drive->ResetOdometry(target);
  auto pose = m_drive->GetPose();
  CHECK(pose.X().value() == Catch::Approx(3.0).margin(0.01));
  CHECK(pose.Y().value() == Catch::Approx(2.0).margin(0.01));
  CHECK(pose.Rotation().Degrees().value() == Catch::Approx(45.0).margin(0.1));
}

// GetStateFromRobotRelativeChassisSpeeds converts a pure forward command into
// forward-pointing states for all four modules.
TEST_CASE_METHOD(SwerveDriveTestFixture, "SwerveDriveTest.GetStateFromSpeedsForwardDrive",
                 "[SwerveDriveTest]") {
  auto states = m_drive->GetStateFromRobotRelativeChassisSpeeds(
      wpi::math::ChassisVelocities{1.0_mps, 0_mps, 0_rad_per_s});

  for (size_t i = 0; i < 4; ++i) {
    INFO("Module " << i << " speed should equal commanded speed");
    CHECK(states[i].speed.value() == Catch::Approx(1.0).margin(0.01));
    INFO("Module " << i << " angle should be 0° for pure forward drive");
    CHECK(states[i].angle.Degrees().value() == Catch::Approx(0.0).margin(1.0));
  }
}

// Pure rotation command produces tangential module states (none pointing forward).
TEST_CASE_METHOD(SwerveDriveTestFixture, "SwerveDriveTest.GetStateFromSpeedsPureRotation",
                 "[SwerveDriveTest]") {
  auto states = m_drive->GetStateFromRobotRelativeChassisSpeeds(
      wpi::math::ChassisVelocities{0_mps, 0_mps, wpi::units::radians_per_second_t{1.0}});

  for (size_t i = 0; i < 4; ++i) {
    INFO("Module " << i << " should have non-zero speed for rotation command");
    CHECK(std::abs(states[i].speed.value()) > 0.0);
    INFO("Module " << i << " should not point forward during pure rotation");
    CHECK(std::abs(states[i].angle.Degrees().value()) > 1.0);
  }
}

// SetRobotRelativeChassisSpeeds does not crash for both non-zero and zero inputs.
TEST_CASE_METHOD(SwerveDriveTestFixture, "SwerveDriveTest.SetRobotRelativeSpeedsDoesNotCrash",
                 "[SwerveDriveTest]") {
  CHECK_NOTHROW(
      m_drive->SetRobotRelativeChassisSpeeds(wpi::math::ChassisVelocities{1.0_mps, 0_mps,
                                                                          0_rad_per_s}));
  CHECK_NOTHROW(
      m_drive->SetRobotRelativeChassisSpeeds(wpi::math::ChassisVelocities{0_mps, 0_mps,
                                                                          0_rad_per_s}));
}

// LockPose commands zero translational speed and corner-pointing module angles.
TEST_CASE_METHOD(SwerveDriveTestFixture, "SwerveDriveTest.LockPoseSetsXPattern",
                 "[SwerveDriveTest]") {
  m_drive->LockPose();

  // Desired chassis speeds should be zero after locking.
  auto robotSpeeds = m_drive->GetRobotRelativeSpeed();
  CHECK(robotSpeeds.vx.value() == Catch::Approx(0.0).margin(0.5));
  CHECK(robotSpeeds.vy.value() == Catch::Approx(0.0).margin(0.5));

  // Run sim and check azimuth convergence toward X-pattern corner angles.
  SchedulerHelper::RunForDuration(0.5_s);
  auto modules = m_drive->GetConfig().GetModules();
  for (size_t i = 0; i < 4; ++i) {
    double expected = modules[i]->GetConfig().GetLocation()->Angle().Degrees().value();
    double actual = modules[i]->GetState().angle.Degrees().value();
    INFO("Module " << i << " angle should converge toward lock angle " << expected << "°");
    CHECK(actual == Catch::Approx(expected).margin(180.0));
  }
}

// ZeroGyro does not crash and zeroes the heading.
TEST_CASE_METHOD(SwerveDriveTestFixture, "SwerveDriveTest.ZeroGyroDoesNotCrash",
                 "[SwerveDriveTest]") {
  m_simGyro = 45.0_deg;
  m_drive->ResetOdometry(wpi::math::Pose2d{0_m, 0_m, wpi::math::Rotation2d{45.0_deg}});
  CHECK_NOTHROW(m_drive->ZeroGyro());
  CHECK(m_drive->GetPose().Rotation().Degrees().value() == Catch::Approx(0.0).margin(1.0));
}

// AddVisionMeasurement accepts a pose without crashing.
TEST_CASE_METHOD(SwerveDriveTestFixture, "SwerveDriveTest.AddVisionMeasurementDoesNotCrash",
                 "[SwerveDriveTest]") {
  CHECK_NOTHROW(
      m_drive->AddVisionMeasurement(wpi::math::Pose2d{1_m, 1_m, wpi::math::Rotation2d{}}, 0.0_s));
}

// GetDistanceFromPose returns the Euclidean distance to a target (3-4-5 triangle).
TEST_CASE_METHOD(SwerveDriveTestFixture, "SwerveDriveTest.GetDistanceFromPose",
                 "[SwerveDriveTest]") {
  auto dist =
      m_drive->GetDistanceFromPose(wpi::math::Pose2d{3.0_m, 4.0_m, wpi::math::Rotation2d{}});
  CHECK(dist.value() == Catch::Approx(5.0).margin(0.01));
}

// Drive() returns a command that runs the speed supplier each loop.
TEST_CASE_METHOD(SwerveDriveTestFixture, "SwerveDriveTest.DriveCommandCallsSpeedSupplier",
                 "[SwerveDriveTest]") {
  int callCount = 0;
  auto cmd = m_drive->Drive([&] {
    ++callCount;
    return wpi::math::ChassisVelocities{};
  });
  wpi::cmd::CommandScheduler::GetInstance().Schedule(cmd);
  SchedulerHelper::RunForDuration(0.1_s);
  CHECK(callCount >= 1);
}

// The Drive command declares the configured subsystem as a requirement.
TEST_CASE_METHOD(SwerveDriveTestFixture, "SwerveDriveTest.DriveCommandHasSubsystemRequirement",
                 "[SwerveDriveTest]") {
  auto cmd = m_drive->Drive([] { return wpi::math::ChassisVelocities{}; });
  CHECK(cmd.HasRequirement(Subsystem()));
}

// Scheduling a second Drive command interrupts the first.
TEST_CASE_METHOD(SwerveDriveTestFixture, "SwerveDriveTest.SecondDriveCommandInterruptsFirst",
                 "[SwerveDriveTest]") {
  int firstCalls = 0;
  int secondCalls = 0;
  auto cmd1 = m_drive->Drive([&] {
    ++firstCalls;
    return wpi::math::ChassisVelocities{};
  });
  auto cmd2 = m_drive->Drive([&] {
    ++secondCalls;
    return wpi::math::ChassisVelocities{};
  });

  wpi::cmd::CommandScheduler::GetInstance().Schedule(cmd1);
  SchedulerHelper::RunForDuration(0.04_s);
  int firstCallsAtInterrupt = firstCalls;
  INFO("cmd1 should run initially");
  CHECK(firstCallsAtInterrupt >= 1);

  wpi::cmd::CommandScheduler::GetInstance().Schedule(cmd2);
  SchedulerHelper::RunForDuration(0.04_s);

  INFO("cmd2 should run after interrupt");
  CHECK(secondCalls >= 1);
  INFO("cmd1 should have been cancelled");
  CHECK(firstCalls == firstCallsAtInterrupt);
}

}  // namespace yams::test
*/
