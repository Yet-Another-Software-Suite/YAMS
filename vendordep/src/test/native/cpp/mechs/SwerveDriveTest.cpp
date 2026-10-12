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

#include <frc/controller/PIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/system/plant/DCMotor.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/SubsystemBase.h>
#include <gtest/gtest.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include <cmath>
#include <ctre/phoenix6/TalonFX.hpp>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "helpers/MockHardware.h"
#include "helpers/MotorControllerFactory.h"
#include "helpers/SchedulerHelper.h"
#include "yams/exceptions.hpp"
#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/mechanisms/config/SwerveModuleConfig.hpp"
#include "yams/mechanisms/swerve/SwerveDrive.hpp"
#include "yams/mechanisms/swerve/SwerveDriveConfig.hpp"
#include "yams/mechanisms/swerve/SwerveModule.hpp"
#include "yams/mechanisms/swerve/utility/SwerveInputStream.hpp"
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

  // Configs must outlive their wrappers; stored here so their addresses stay stable.
  SmartMotorControllerConfig flDriveCfg;
  SmartMotorControllerConfig frDriveCfg;
  SmartMotorControllerConfig blDriveCfg;
  SmartMotorControllerConfig brDriveCfg;
  SmartMotorControllerConfig flAzimuthCfg;
  SmartMotorControllerConfig frAzimuthCfg;
  SmartMotorControllerConfig blAzimuthCfg;
  SmartMotorControllerConfig brAzimuthCfg;

  SwerveSuiteHardware() {
    InitializeHardware();
    SchedulerHelper::Enable();

    sub = new SwerveTestSubsystem();

    flDriveTalon =
        new ctre::phoenix6::hardware::TalonFX(NextReservedCanId());
    frDriveTalon =
        new ctre::phoenix6::hardware::TalonFX(NextReservedCanId());
    blDriveTalon =
        new ctre::phoenix6::hardware::TalonFX(NextReservedCanId());
    brDriveTalon =
        new ctre::phoenix6::hardware::TalonFX(NextReservedCanId());
    flAzimuthTalon =
        new ctre::phoenix6::hardware::TalonFX(NextReservedCanId());
    frAzimuthTalon =
        new ctre::phoenix6::hardware::TalonFX(NextReservedCanId());
    blAzimuthTalon =
        new ctre::phoenix6::hardware::TalonFX(NextReservedCanId());
    brAzimuthTalon =
        new ctre::phoenix6::hardware::TalonFX(NextReservedCanId());

    flDriveCfg = MakeDriveConfig("FL_Drive", sub);
    frDriveCfg = MakeDriveConfig("FR_Drive", sub);
    blDriveCfg = MakeDriveConfig("BL_Drive", sub);
    brDriveCfg = MakeDriveConfig("BR_Drive", sub);
    flAzimuthCfg = MakeAzimuthConfig("FL_Azimuth", sub);
    frAzimuthCfg = MakeAzimuthConfig("FR_Azimuth", sub);
    blAzimuthCfg = MakeAzimuthConfig("BL_Azimuth", sub);
    brAzimuthCfg = MakeAzimuthConfig("BR_Azimuth", sub);

    // TalonFXWrapper constructor calls SetupSimulation() automatically.
    flDriveSMC = new remote::TalonFXWrapper(flDriveTalon, frc::DCMotor::KrakenX60(1),
                                            &flDriveCfg);
    frDriveSMC = new remote::TalonFXWrapper(frDriveTalon, frc::DCMotor::KrakenX60(1),
                                            &frDriveCfg);
    blDriveSMC = new remote::TalonFXWrapper(blDriveTalon, frc::DCMotor::KrakenX60(1),
                                            &blDriveCfg);
    brDriveSMC = new remote::TalonFXWrapper(brDriveTalon, frc::DCMotor::KrakenX60(1),
                                            &brDriveCfg);
    flAzimuthSMC = new remote::TalonFXWrapper(flAzimuthTalon, frc::DCMotor::KrakenX60(1),
                                              &flAzimuthCfg);
    frAzimuthSMC = new remote::TalonFXWrapper(frAzimuthTalon, frc::DCMotor::KrakenX60(1),
                                              &frAzimuthCfg);
    blAzimuthSMC = new remote::TalonFXWrapper(blAzimuthTalon, frc::DCMotor::KrakenX60(1),
                                              &blAzimuthCfg);
    brAzimuthSMC = new remote::TalonFXWrapper(brAzimuthTalon, frc::DCMotor::KrakenX60(1),
                                              &brAzimuthCfg);
  }

  ~SwerveSuiteHardware() {
    sub->m_drive = nullptr;
    SchedulerHelper::CancelAll();
    frc2::CommandScheduler::GetInstance().UnregisterSubsystem(sub);

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
    // Other suites' teardown resets DriverStation sim data (disabling the robot), and the shared
    // hardware above only initializes once, so re-enable before each test so commands run.
    InitializeHardware();
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

    m_flCfg = makeModuleCfg(hw.flDriveSMC, hw.flAzimuthSMC, kModuleX, kModuleY, "FL");
    m_frCfg = makeModuleCfg(hw.frDriveSMC, hw.frAzimuthSMC, kModuleX, -kModuleY, "FR");
    m_blCfg = makeModuleCfg(hw.blDriveSMC, hw.blAzimuthSMC, -kModuleX, kModuleY, "BL");
    m_brCfg = makeModuleCfg(hw.brDriveSMC, hw.brAzimuthSMC, -kModuleX, -kModuleY, "BR");
    m_fl.emplace(&m_flCfg);
    m_fr.emplace(&m_frCfg);
    m_bl.emplace(&m_blCfg);
    m_br.emplace(&m_brCfg);

    m_driveCfg.WithSubsystem(hw.sub)
        .WithModules({&m_fl.value(), &m_fr.value(), &m_bl.value(), &m_br.value()})
        .WithGyro([this] { return m_simGyro; })
        .WithStartingPose(frc::Pose2d{})
        .WithMaximumChassisSpeed(4.5_mps, units::degrees_per_second_t{540})
        .WithTranslationController(frc::PIDController{2.0, 0.0, 0.0})
        .WithRotationController(frc::PIDController{4.0, 0.0, 0.0});
    m_drive.emplace(&m_driveCfg);

    hw.sub->m_drive = &m_drive.value();
  }

  ~SwerveDriveTestFixture() {
    Hardware().sub->m_drive = nullptr;
    frc2::CommandScheduler::GetInstance().CancelAll();
    m_drive.reset();
    m_fl.reset();
    m_fr.reset();
    m_bl.reset();
    m_br.reset();
  }

  SwerveTestSubsystem* Subsystem() { return Hardware().sub; }

  // Simulated gyro angle tests can mutate this to fake heading.
  units::degree_t m_simGyro{0};

  // Configs must outlive the module/drive objects that hold pointers to them.
  SwerveModuleConfig m_flCfg;
  SwerveModuleConfig m_frCfg;
  SwerveModuleConfig m_blCfg;
  SwerveModuleConfig m_brCfg;
  SwerveDriveConfig m_driveCfg;

  std::optional<SwerveModule> m_fl;
  std::optional<SwerveModule> m_fr;
  std::optional<SwerveModule> m_bl;
  std::optional<SwerveModule> m_br;

  std::optional<SwerveDrive<4>> m_drive;
};

class SwerveDriveTest : public ::testing::Test, public SwerveDriveTestFixture {};
class SwerveInputStreamTelemetryTest : public ::testing::Test, public SwerveDriveTestFixture {};

// ---- Tests -------------------------------------------------------------------

// Drive constructs and destructs cleanly.
TEST_F(SwerveDriveTest, ConstructionDoesNotCrash) {
  EXPECT_TRUE(m_drive.has_value());
}

// Configured translation and rotation controllers are returned by the config.
TEST_F(SwerveDriveTest, ConfiguredPIDControllersArePresent) {
  auto translationPID = m_driveCfg.GetTranslationPID();
  auto rotationPID = m_driveCfg.GetRotationPID();
  ASSERT_TRUE(translationPID.has_value());
  ASSERT_TRUE(rotationPID.has_value());
  EXPECT_NEAR(translationPID->get().GetP(), 2.0, 1e-9);
  EXPECT_NEAR(rotationPID->get().GetP(), 4.0, 1e-9);
}

// Translation and rotation controllers are optional: a drive without them constructs, runs its
// telemetry, and resets cleanly, and only drive to pose reports that they are missing.
TEST_F(SwerveDriveTest, PIDControllersAreOptional) {
  // Release the fixture's drive so its modules can be reused by a drive without controllers.
  Hardware().sub->m_drive = nullptr;
  m_drive.reset();

  // Declared before the drive so the config outlives it.
  SwerveDriveConfig cfg;
  cfg.WithSubsystem(Hardware().sub)
      .WithModules({&m_fl.value(), &m_fr.value(), &m_bl.value(), &m_br.value()})
      .WithGyro([this] { return m_simGyro; })
      .WithStartingPose(frc::Pose2d{})
      .WithMaximumChassisSpeed(4.5_mps, units::degrees_per_second_t{540});
  EXPECT_FALSE(cfg.GetTranslationPID().has_value());
  EXPECT_FALSE(cfg.GetRotationPID().has_value());

  std::optional<SwerveDrive<4>> drive;
  ASSERT_NO_THROW(drive.emplace(&cfg));
  EXPECT_NO_THROW(drive->UpdateTelemetry());
  EXPECT_NO_THROW(drive->SimIterate());
  EXPECT_NO_THROW(drive->ResetTranslationPID());
  EXPECT_NO_THROW(drive->ResetRotationPID());
  EXPECT_THROW(drive->DriveToPoseSetpoint(frc::Pose2d{}),
                  yams::exceptions::SwerveDriveConfigurationException);
  drive.reset();
}

// UpdateTelemetry and SimIterate run for several loops without crashing.
TEST_F(SwerveDriveTest, TelemetryAndSimRunWithoutCrash) {
  EXPECT_NO_THROW(SchedulerHelper::RunForDuration(0.5_s));
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

// ResetOdometry sets the gyro to the pose's heading so field relative driving and heading control,
// which use the gyro, agree with the reset pose. ZeroGyro resets both to 0 degrees.
TEST_F(SwerveDriveTest, ResetOdometryAlignsGyro) {
  m_drive->ResetOdometry(frc::Pose2d{1.0_m, 2.0_m, frc::Rotation2d{90.0_deg}});
  EXPECT_NEAR(m_drive->GetGyroAngle().value(), 90.0, 0.1);
  EXPECT_NEAR(m_drive->GetPose().Rotation().Degrees().value(), 90.0, 0.1);

  m_drive->ZeroGyro();
  EXPECT_NEAR(m_drive->GetGyroAngle().value(), 0.0, 0.1);
  EXPECT_NEAR(m_drive->GetPose().Rotation().Degrees().value(), 0.0, 0.1);
  EXPECT_NEAR(m_drive->GetPose().X().value(), 1.0, 0.01);
}

// GetStateFromRobotRelativeChassisSpeeds converts a pure forward command into
// forward-pointing states for all four modules.
TEST_F(SwerveDriveTest, GetStateFromSpeedsForwardDrive) {
  auto states = m_drive->GetStateFromRobotRelativeChassisSpeeds(
      frc::ChassisSpeeds{1.0_mps, 0_mps, 0_rad_per_s});

  for (size_t i = 0; i < 4; ++i) {
    SCOPED_TRACE(::testing::Message() << "Module " << i << " speed should equal commanded speed");
    EXPECT_NEAR(states[i].speed.value(), 1.0, 0.01);
    SCOPED_TRACE(::testing::Message() << "Module " << i << " angle should be 0° for pure forward drive");
    EXPECT_NEAR(states[i].angle.Degrees().value(), 0.0, 1.0);
  }
}

// Pure rotation command produces tangential module states (none pointing forward).
TEST_F(SwerveDriveTest, GetStateFromSpeedsPureRotation) {
  auto states = m_drive->GetStateFromRobotRelativeChassisSpeeds(
      frc::ChassisSpeeds{0_mps, 0_mps, units::radians_per_second_t{1.0}});

  for (size_t i = 0; i < 4; ++i) {
    SCOPED_TRACE(::testing::Message() << "Module " << i << " should have non-zero speed for rotation command");
    EXPECT_TRUE(std::abs(states[i].speed.value()) > 0.0);
    SCOPED_TRACE(::testing::Message() << "Module " << i << " should not point forward during pure rotation");
    EXPECT_TRUE(std::abs(states[i].angle.Degrees().value()) > 1.0);
  }
}

// SetRobotRelativeChassisSpeeds does not crash for both non-zero and zero inputs.
TEST_F(SwerveDriveTest, SetRobotRelativeSpeedsDoesNotCrash) {
  EXPECT_NO_THROW(
      m_drive->SetRobotRelativeChassisSpeeds(frc::ChassisSpeeds{1.0_mps, 0_mps,
                                                                          0_rad_per_s}));
  EXPECT_NO_THROW(
      m_drive->SetRobotRelativeChassisSpeeds(frc::ChassisSpeeds{0_mps, 0_mps,
                                                                          0_rad_per_s}));
}

// LockPose commands zero translational speed and corner-pointing module angles.
TEST_F(SwerveDriveTest, LockPoseSetsXPattern) {
  m_drive->LockPose();

  // Desired chassis speeds should be zero after locking.
  auto desiredSpeeds = m_drive->GetDesiredChassisSpeeds();
  EXPECT_NEAR(desiredSpeeds.vx.value(), 0.0, 1e-9);
  EXPECT_NEAR(desiredSpeeds.vy.value(), 0.0, 1e-9);
  EXPECT_NEAR(desiredSpeeds.omega.value(), 0.0, 1e-9);

  // Run sim and check azimuth convergence toward X-pattern corner angles.
  SchedulerHelper::RunForDuration(0.5_s);
  auto modules = m_drive->GetConfig().GetModules();
  for (size_t i = 0; i < 4; ++i) {
    double expected = modules[i]->GetConfig().GetLocation()->Angle().Degrees().value();
    double actual = modules[i]->GetState().angle.Degrees().value();
    // Module optimization may reverse the wheel, so angles 180° apart are equivalent.
    double error = std::remainder(actual - expected, 180.0);
    SCOPED_TRACE(::testing::Message() << "Module " << i << " angle " << actual << "° should converge toward lock angle "
                   << expected << "° (mod 180°)");
    EXPECT_NEAR(error, 0.0, 10.0);
  }
}

// ZeroGyro does not crash and zeroes the heading.
TEST_F(SwerveDriveTest, ZeroGyroDoesNotCrash) {
  m_simGyro = 45.0_deg;
  m_drive->ResetOdometry(frc::Pose2d{0_m, 0_m, frc::Rotation2d{45.0_deg}});
  EXPECT_NO_THROW(m_drive->ZeroGyro());
  EXPECT_NEAR(m_drive->GetPose().Rotation().Degrees().value(), 0.0, 1.0);
}

// AddVisionMeasurement accepts a pose without crashing.
TEST_F(SwerveDriveTest, AddVisionMeasurementDoesNotCrash) {
  EXPECT_NO_THROW(
      m_drive->AddVisionMeasurement(frc::Pose2d{1_m, 1_m, frc::Rotation2d{}}, 0.0_s));
}

// GetDistanceFromPose returns the Euclidean distance to a target (3-4-5 triangle).
TEST_F(SwerveDriveTest, GetDistanceFromPose) {
  auto dist =
      m_drive->GetDistanceFromPose(frc::Pose2d{3.0_m, 4.0_m, frc::Rotation2d{}});
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
  EXPECT_TRUE(callCount >= 1);
}

// The Drive command declares the configured subsystem as a requirement.
TEST_F(SwerveDriveTest, DriveCommandHasSubsystemRequirement) {
  auto cmd = m_drive->Drive([] { return frc::ChassisSpeeds{}; });
  EXPECT_TRUE(cmd.HasRequirement(Subsystem()));
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
  SCOPED_TRACE(::testing::Message() << "cmd1 should run initially");
  EXPECT_TRUE(firstCallsAtInterrupt >= 1);

  frc2::CommandScheduler::GetInstance().Schedule(cmd2);
  SchedulerHelper::RunForDuration(0.04_s);

  SCOPED_TRACE(::testing::Message() << "cmd2 should run after interrupt");
  EXPECT_TRUE(secondCalls >= 1);
  SCOPED_TRACE(::testing::Message() << "cmd1 should have been cancelled");
  EXPECT_TRUE(firstCalls == firstCallsAtInterrupt);
}

// The drive to pose setpoint is the controller output along the unit vector toward the target.
TEST_F(SwerveDriveTest, DriveToPoseSetpointIsProportionalToTheDistance) {
  m_drive->ResetTranslationPID();
  m_drive->ResetRotationPID();
  // kP of 2 and 5 m to go is 10 m/s toward the target, along (0.6, 0.8).
  auto speeds = m_drive->DriveToPoseSetpoint(frc::Pose2d{3.0_m, 4.0_m, frc::Rotation2d{}});
  EXPECT_NEAR(speeds.vx.value(), 6.0, 0.01);
  EXPECT_NEAR(speeds.vy.value(), 8.0, 0.01);
  EXPECT_NEAR(speeds.omega.value(), 0.0, 0.01);
  // At the pose there is nowhere to go.
  auto atPose = m_drive->DriveToPoseSetpoint(frc::Pose2d{});
  EXPECT_NEAR(atPose.vx.value(), 0.0, 1e-6);
  EXPECT_NEAR(atPose.vy.value(), 0.0, 1e-6);
}

// The setpoint is robot relative: with the robot facing +Y, a target ahead in +Y is straight ahead.
TEST_F(SwerveDriveTest, DriveToPoseSetpointIsRobotRelative) {
  m_drive->ResetOdometry(frc::Pose2d{0_m, 0_m, frc::Rotation2d{90.0_deg}});
  auto speeds = m_drive->DriveToPoseSetpoint(frc::Pose2d{0_m, 1.0_m, frc::Rotation2d{90.0_deg}});
  EXPECT_NEAR(speeds.vx.value(), 2.0, 0.01);
  EXPECT_NEAR(speeds.vy.value(), 0.0, 0.01);
}

// DriveToPose keeps driving until it is interrupted, then stops the drive.
TEST_F(SwerveDriveTest, DriveToPoseKeepsDrivingUntilInterruptedAndThenStops) {
  auto cmd = m_drive->DriveToPose(frc::Pose2d{30.0_m, 0_m, frc::Rotation2d{}});
  auto& scheduler = frc2::CommandScheduler::GetInstance();
  scheduler.Schedule(cmd);
  SchedulerHelper::RunForDuration(0.2_s);
  EXPECT_TRUE(scheduler.IsScheduled(cmd)) << "should still be driving to the pose";
  EXPECT_GT(m_drive->GetDesiredChassisSpeeds().vx.value(), 0.0);

  scheduler.Cancel(cmd);
  EXPECT_NEAR(m_drive->GetDesiredChassisSpeeds().vx.value(), 0.0, 1e-9)
      << "the drive should be stopped when the command ends";
}

// DriveToPose with tolerances ends once the robot is within them, and stops the drive.
TEST_F(SwerveDriveTest, DriveToPoseWithTolerancesEndsAtThePose) {
  frc::Pose2d pose{0.02_m, 0_m, frc::Rotation2d{1.0_deg}};
  auto cmd = m_drive->DriveToPose(pose, 5.0_cm, 3.0_deg);
  auto& scheduler = frc2::CommandScheduler::GetInstance();
  scheduler.Schedule(cmd);
  SchedulerHelper::RunForDuration(0.1_s);
  EXPECT_FALSE(scheduler.IsScheduled(cmd)) << "should end once within the tolerances";
  EXPECT_NEAR(m_drive->GetDesiredChassisSpeeds().vx.value(), 0.0, 1e-9);
}

// IsNear checks both the distance and the heading.
TEST_F(SwerveDriveTest, IsNearChecksDistanceAndHeading) {
  m_drive->ResetOdometry(frc::Pose2d{1.0_m, 2.0_m, frc::Rotation2d{90.0_deg}});
  EXPECT_TRUE(m_drive->IsNear(frc::Pose2d{1.03_m, 2.0_m, frc::Rotation2d{92.0_deg}}, 5.0_cm, 3.0_deg));
  EXPECT_FALSE(m_drive->IsNear(frc::Pose2d{1.2_m, 2.0_m, frc::Rotation2d{90.0_deg}}, 5.0_cm, 3.0_deg));
  EXPECT_FALSE(m_drive->IsNear(frc::Pose2d{1.0_m, 2.0_m, frc::Rotation2d{100.0_deg}}, 5.0_cm, 3.0_deg));
}

// LockPose turns the wheels even with a velocity deadband, which holds zero speed states in place.
TEST_F(SwerveDriveTest, LockPoseTurnsTheWheelsEvenWithAVelocityDeadband) {
  for (auto* cfg : {&m_flCfg, &m_frCfg, &m_blCfg, &m_brCfg}) {
    cfg->WithVelocityDeadband(0.1_mps);
  }
  // Point every wheel forward first, so the lock has somewhere to turn them to.
  m_drive->SetRobotRelativeChassisSpeeds(frc::ChassisSpeeds{1.0_mps, 0_mps, 0_rad_per_s});
  SchedulerHelper::RunForDuration(0.5_s);
  m_drive->LockPose();
  SchedulerHelper::RunForDuration(0.5_s);

  auto modules = m_drive->GetConfig().GetModules();
  for (size_t i = 0; i < 4; ++i) {
    double expected = modules[i]->GetConfig().GetLocation()->Angle().Degrees().value();
    double actual = modules[i]->GetState().angle.Degrees().value();
    // Module optimization may reverse the wheel, so angles 180° apart are equivalent.
    double error = std::remainder(actual - expected, 180.0);
    EXPECT_NEAR(error, 0.0, 10.0) << "module " << i << " is at " << actual << " degrees";
  }
}

// Optimization reverses the wheel when that is the shorter turn, and then keeps the orientation the
// wheel was sent to instead of reversing it back for a target that moved a little.
TEST_F(SwerveDriveTest, OptimizationKeepsTheOrientationTheWheelWasSentTo) {
  const auto& cfg = m_fl->GetConfig();
  auto current = m_fl->GetState().angle;
  auto first = cfg.GetOptimizedState(
      frc::SwerveModuleState{1.0_mps, current + frc::Rotation2d{135.0_deg}});
  EXPECT_LT(first.speed.value(), 0.0) << "135 degrees away is shorter reversed";
  EXPECT_NEAR((first.angle - current).Degrees().value(), -45.0, 1.0);

  // 80 degrees away would not be reversed on its own, but the wheel was just sent the other way.
  auto second = cfg.GetOptimizedState(
      frc::SwerveModuleState{1.0_mps, current + frc::Rotation2d{80.0_deg}});
  EXPECT_LT(second.speed.value(), 0.0);
  EXPECT_NEAR((second.angle - current).Degrees().value(), -100.0, 1.0);
}

// Robot relative control ignores alliance relative control instead of throwing.
TEST_F(SwerveDriveTest, RobotRelativeStreamWithAllianceRelativeDoesNotThrow) {
  utility::SwerveInputStream<4> stream{*m_drive, [] { return 1.0; }, [] { return 0.0; },
                                       [] { return 0.0; }};
  stream.WithAllianceRelativeControl().WithRobotRelative();
  EXPECT_NO_THROW(stream.Get());
}

// WithHeading faces a heading from any source while heading control is enabled.
TEST_F(SwerveDriveTest, StreamWithHeadingTurnsTowardTheHeading) {
  auto stream = utility::SwerveInputStream<4>::Of(*m_drive, [] { return 0.0; }, [] { return 0.0; });
  stream.WithHeading([] { return units::radian_t{90.0_deg}; }).WithHeadingControl([] {
    return true;
  });
  EXPECT_GT(stream.Get().omega.value(), 0.0) << "should turn counter-clockwise toward 90 degrees";
}

// ---- SwerveInputStream telemetry and live tuning ----------------------------------
//
// SwerveInputStream::WithTelemetry on the fixture's drive (4.5 m/s, 540 deg/s, rotation
// controller). Each test names its stream uniquely; writing to an entry from the test stands in for
// the dashboard.

using utility::SwerveInputStream;
using utility::SwerveInputStreamTelemetry;
using Verbosity = SwerveInputStream<4>::TelemetryVerbosity;

namespace {

constexpr double kConfigMaxLinear = 4.5;
constexpr double kConfigMaxAngular = 540.0 * std::numbers::pi / 180.0;

// Tables of a stream's telemetry, with helpers to read entries and edit them as the dashboard.
struct StreamTables {
  StreamTables() : StreamTables{NextName()} {}

  explicit StreamTables(std::string streamName)
      : name{std::move(streamName)},
        data{nt::NetworkTableInstance::GetDefault().GetTable("SwerveInputStream")->GetSubTable(
            name)},
        tuning{nt::NetworkTableInstance::GetDefault()
                   .GetTable("Tuning")
                   ->GetSubTable("SwerveInputStream")
                   ->GetSubTable(name)} {}

  static std::string NextName() {
    static int count = 0;
    return "SISTelemetryTest" + std::to_string(count++);
  }

  double Data(std::string_view key) {
    return data->GetEntry(key).GetDouble(std::numeric_limits<double>::quiet_NaN());
  }
  std::string Mode() { return data->GetEntry("mode").GetString(""); }
  double Published(std::string_view key) {
    return tuning->GetEntry(key).GetDouble(std::numeric_limits<double>::quiet_NaN());
  }
  bool PublishedBoolean(std::string_view key) { return tuning->GetEntry(key).GetBoolean(false); }
  void Dashboard(std::string_view key, double value) { tuning->GetEntry(key).SetDouble(value); }
  void Dashboard(std::string_view key, bool value) { tuning->GetEntry(key).SetBoolean(value); }
  bool LiveTuningPublished() {
    frc::SmartDashboard::UpdateValues();
    return !nt::NetworkTableInstance::GetDefault()
                .GetTable("SmartDashboard")
                ->GetSubTable("Mechanisms")
                ->GetSubTable("Commands")
                ->GetSubTable("SwerveInputStream")
                ->GetSubTable(name)
                ->GetSubTable("Live Tuning")
                ->GetKeys()
                .empty();
  }

  std::string name;
  std::shared_ptr<nt::NetworkTable> data;
  std::shared_ptr<nt::NetworkTable> tuning;
};

// Apply dashboard edits, as one loop of the Live Tuning command does.
void Tune(SwerveInputStream<4>& stream) {
  stream.GetTelemetry()->get().ApplyTuningValues();
}

}  // namespace

TEST_F(SwerveInputStreamTelemetryTest, LowPublishesOnlyTheMode) {
  StreamTables nt;
  SwerveInputStream<4> stream{*m_drive, [] { return 0.0; }, [] { return 0.0; },
                              [] { return 0.0; }};
  stream.WithDeadband(0.1).WithTelemetry(nt.name, Verbosity::LOW);
  stream.Get();

  EXPECT_TRUE(nt.Mode() == "ANGULAR_VELOCITY");
  EXPECT_FALSE(nt.data->GetTopic("deadband").Exists());
  EXPECT_FALSE(nt.tuning->GetTopic("deadband").Exists());
  EXPECT_FALSE(stream.GetTelemetry()->get().GetLiveTuningCommand().has_value());
}

TEST_F(SwerveInputStreamTelemetryTest, MediumPublishesTheConfigurationReadOnly) {
  StreamTables nt;
  SwerveInputStream<4> stream{*m_drive, [] { return 0.0; }, [] { return 0.0; },
                              [] { return 0.0; }};
  stream.WithDeadband(0.1).WithScaleTranslation(0.8).WithTelemetry(nt.name, Verbosity::MEDIUM);
  stream.Get();

  EXPECT_NEAR(nt.Data("deadband"), 0.1, 1e-9);
  EXPECT_NEAR(nt.Data("translationScale"), 0.8, 1e-9);
  EXPECT_NEAR(nt.Data("maxLinearVelocity"), kConfigMaxLinear, 1e-9);
  EXPECT_FALSE(nt.tuning->GetTopic("deadband").Exists());
  EXPECT_FALSE(stream.GetTelemetry()->get().GetLiveTuningCommand().has_value());

  // Changes made in code are published when the stream is read.
  stream.WithScaleTranslation(0.5);
  stream.Get();
  EXPECT_NEAR(nt.Data("translationScale"), 0.5, 1e-9);
}

TEST_F(SwerveInputStreamTelemetryTest, HighPublishesTuningValuesAndTheLiveTuningCommand) {
  StreamTables nt;
  SwerveInputStream<4> stream{*m_drive, [] { return 0.0; }, [] { return 0.0; },
                              [] { return 0.0; }};
  stream.WithDeadband(0.1)
      .WithScaleTranslation(0.8)
      .WithScaleRotation(0.5)
      .WithCubeTranslationControllerAxis()
      .WithAllianceRelativeControl()
      .WithTelemetry(nt.name, Verbosity::HIGH);
  stream.Get();

  EXPECT_NEAR(nt.Data("deadband"), 0.1, 1e-9);
  EXPECT_NEAR(nt.Published("deadband"), 0.1, 1e-9);
  EXPECT_NEAR(nt.Published("translationScale"), 0.8, 1e-9);
  EXPECT_NEAR(nt.Published("rotationScale"), 0.5, 1e-9);
  EXPECT_NEAR(nt.Published("maxLinearVelocity"), kConfigMaxLinear, 1e-9);
  EXPECT_NEAR(nt.Published("maxAngularVelocity"), kConfigMaxAngular, 1e-9);
  EXPECT_TRUE(nt.PublishedBoolean("translationCube"));
  EXPECT_FALSE(nt.PublishedBoolean("rotationCube"));
  EXPECT_TRUE(nt.PublishedBoolean("allianceRelative"));
  EXPECT_FALSE(nt.PublishedBoolean("robotRelative"));
  EXPECT_TRUE(stream.GetTelemetry()->get().GetLiveTuningCommand().has_value());
  EXPECT_TRUE(nt.LiveTuningPublished());
}

TEST_F(SwerveInputStreamTelemetryTest, ReadingTheStreamPublishesTheMode) {
  StreamTables nt;
  bool headingControl = false;
  auto stream = SwerveInputStream<4>::Of(*m_drive, [] { return 0.0; }, [] { return 0.0; });
  stream.WithControllerHeadingAxis([] { return 0.0; }, [] { return 1.0; })
      .WithHeadingControl([&] { return headingControl; })
      .WithTelemetry(nt.name, Verbosity::LOW);

  // No rotation axis, so the stream holds its heading until heading control is on.
  stream.Get();
  EXPECT_TRUE(nt.Mode() == "TRANSLATION_ONLY");
  headingControl = true;
  stream.Get();
  EXPECT_TRUE(nt.Mode() == "HEADING");
}

TEST_F(SwerveInputStreamTelemetryTest, DashboardEditsApplyOnlyWhileLiveTuningRuns) {
  StreamTables nt;
  SwerveInputStream<4> stream{*m_drive, [] { return 0.0; }, [] { return 0.0; },
                              [] { return 0.0; }};
  stream.WithTelemetry(nt.name, Verbosity::HIGH);
  stream.Get();
  frc2::Command& liveTuning = *stream.GetTelemetry()->get().GetLiveTuningCommand();
  auto& scheduler = frc2::CommandScheduler::GetInstance();

  nt.Dashboard("deadband", 0.2);
  stream.Get();
  SCOPED_TRACE(::testing::Message() << "not applied before Live Tuning runs");
  EXPECT_NEAR(stream.GetAxisDeadband(), 0.0, 1e-9);

  scheduler.Schedule(&liveTuning);
  scheduler.Run();
  SCOPED_TRACE(::testing::Message() << "applied while Live Tuning runs");
  EXPECT_NEAR(stream.GetAxisDeadband(), 0.2, 1e-9);

  scheduler.Cancel(&liveTuning);
  scheduler.Run();
  nt.Dashboard("deadband", 0.3);
  stream.Get();
  SCOPED_TRACE(::testing::Message() << "not applied after Live Tuning stops");
  EXPECT_NEAR(stream.GetAxisDeadband(), 0.2, 1e-9);
}

TEST_F(SwerveInputStreamTelemetryTest, DashboardEditsAreAppliedToTheStream) {
  StreamTables nt;
  SwerveInputStream<4> stream{*m_drive, [] { return 0.0; }, [] { return 0.0; },
                              [] { return 0.0; }};
  stream.WithTelemetry(nt.name, Verbosity::HIGH);
  stream.Get();

  nt.Dashboard("deadband", 0.2);
  nt.Dashboard("translationScale", 0.6);
  nt.Dashboard("rotationScale", 0.4);
  nt.Dashboard("maxLinearVelocity", 3.0);
  nt.Dashboard("maxAngularVelocity", 5.0);
  nt.Dashboard("translationCube", true);
  nt.Dashboard("rotationCube", true);
  nt.Dashboard("allianceRelative", true);
  nt.Dashboard("robotRelative", true);
  Tune(stream);

  EXPECT_NEAR(stream.GetAxisDeadband(), 0.2, 1e-9);
  EXPECT_NEAR(stream.GetTranslationAxisScale(), 0.6, 1e-9);
  EXPECT_NEAR(stream.GetOmegaAxisScale(), 0.4, 1e-9);
  EXPECT_NEAR(stream.GetMaximumChassisLinearVelocity().value(), 3.0, 1e-9);
  EXPECT_NEAR(stream.GetMaximumChassisAngularVelocity().value(), 5.0, 1e-9);
  EXPECT_TRUE(stream.IsTranslationCubeEnabled());
  EXPECT_TRUE(stream.IsOmegaCubeEnabled());
  EXPECT_TRUE(stream.IsAllianceRelativeEnabled());
  EXPECT_TRUE(stream.IsRobotRelativeEnabled());

  // The edits stay applied on later loops.
  Tune(stream);
  EXPECT_NEAR(stream.GetAxisDeadband(), 0.2, 1e-9);
  EXPECT_NEAR(nt.Published("deadband"), 0.2, 1e-9);
  EXPECT_TRUE(stream.IsRobotRelativeEnabled());

  // Turning a feature back off on the dashboard turns it off in the stream.
  nt.Dashboard("translationCube", false);
  Tune(stream);
  EXPECT_FALSE(stream.IsTranslationCubeEnabled());
}

TEST_F(SwerveInputStreamTelemetryTest, TunedMaximumVelocitiesOverrideTheDriveConfig) {
  StreamTables nt;
  SwerveInputStream<4> stream{*m_drive, [] { return 1.0; }, [] { return 0.0; },
                              [] { return 1.0; }};
  stream.WithTelemetry(nt.name, Verbosity::HIGH);

  auto speeds = stream.Get();
  EXPECT_NEAR(speeds.vx.value(), kConfigMaxLinear, 1e-9);
  EXPECT_NEAR(speeds.omega.value(), kConfigMaxAngular, 1e-9);

  nt.Dashboard("maxLinearVelocity", 2.0);
  nt.Dashboard("maxAngularVelocity", 3.0);
  Tune(stream);
  speeds = stream.Get();
  EXPECT_NEAR(speeds.vx.value(), 2.0, 1e-9);
  EXPECT_NEAR(speeds.omega.value(), 3.0, 1e-9);
}

TEST_F(SwerveInputStreamTelemetryTest, TunedScalesChangeTheOutput) {
  StreamTables nt;
  SwerveInputStream<4> stream{*m_drive, [] { return 1.0; }, [] { return 0.0; },
                              [] { return 1.0; }};
  stream.WithTelemetry(nt.name, Verbosity::HIGH);
  stream.Get();

  nt.Dashboard("translationScale", 0.5);
  nt.Dashboard("rotationScale", 0.25);
  Tune(stream);
  auto speeds = stream.Get();
  EXPECT_NEAR(speeds.vx.value(), 0.5 * kConfigMaxLinear, 1e-9);
  EXPECT_NEAR(speeds.omega.value(), 0.25 * kConfigMaxAngular, 1e-9);
}

TEST_F(SwerveInputStreamTelemetryTest, TunedDeadbandChangesTheOutput) {
  StreamTables nt;
  SwerveInputStream<4> stream{*m_drive, [] { return 0.3; }, [] { return 0.0; },
                              [] { return 0.0; }};
  stream.WithTelemetry(nt.name, Verbosity::HIGH);
  EXPECT_GT(stream.Get().vx.value(), 0.0);

  nt.Dashboard("deadband", 0.5);
  Tune(stream);
  EXPECT_NEAR(stream.Get().vx.value(), 0.0, 1e-9);
}

TEST_F(SwerveInputStreamTelemetryTest, TunedCubingChangesTheOutput) {
  StreamTables nt;
  SwerveInputStream<4> stream{*m_drive, [] { return 0.0; }, [] { return 0.0; },
                              [] { return 0.5; }};
  stream.WithTelemetry(nt.name, Verbosity::HIGH);
  stream.Get();

  nt.Dashboard("rotationCube", true);
  Tune(stream);
  EXPECT_NEAR(stream.Get().omega.value(), 0.125 * kConfigMaxAngular, 1e-9);
}

TEST_F(SwerveInputStreamTelemetryTest, InvalidDashboardValuesAreReplacedWithTheStreamValue) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  const std::vector<std::pair<std::string, double>> cases{
      {"deadband", -0.1},         {"deadband", 1.0},           {"deadband", nan},
      {"translationScale", 0.0},  {"translationScale", 1.5},   {"rotationScale", -0.5},
      {"rotationScale", nan},     {"maxLinearVelocity", 0.0},  {"maxLinearVelocity", -1.0},
      {"maxLinearVelocity", infinity}, {"maxAngularVelocity", 0.0}, {"maxAngularVelocity", nan}};
  for (const auto& [key, value] : cases) {
    SCOPED_TRACE(::testing::Message() << key << " = " << value);
    StreamTables nt;
    SwerveInputStream<4> stream{*m_drive, [] { return 0.0; }, [] { return 0.0; },
                                [] { return 0.0; }};
    stream.WithDeadband(0.1).WithScaleTranslation(0.8).WithScaleRotation(0.5).WithTelemetry(
        nt.name, Verbosity::HIGH);
    stream.Get();
    double before = nt.Published(key);

    nt.Dashboard(key, value);
    Tune(stream);
    EXPECT_NEAR(nt.Published(key), before, 1e-9);
    Tune(stream);
    EXPECT_NEAR(nt.Published(key), before, 1e-9);
  }
}

TEST_F(SwerveInputStreamTelemetryTest, CodeChangesArePublishedAndNotOverridden) {
  StreamTables nt;
  SwerveInputStream<4> stream{*m_drive, [] { return 0.0; }, [] { return 0.0; },
                              [] { return 0.0; }};
  stream.WithScaleTranslation(0.8).WithTelemetry(nt.name, Verbosity::HIGH);
  stream.Get();

  // E.g. a slow mode binding changing the scale while driving.
  stream.WithScaleTranslation(0.4);
  Tune(stream);
  EXPECT_NEAR(nt.Published("translationScale"), 0.4, 1e-9);
  EXPECT_NEAR(stream.GetTranslationAxisScale(), 0.4, 1e-9);

  stream.WithScaleTranslation(0.8);
  Tune(stream);
  EXPECT_NEAR(nt.Published("translationScale"), 0.8, 1e-9);
  EXPECT_NEAR(stream.GetTranslationAxisScale(), 0.8, 1e-9);
}

TEST_F(SwerveInputStreamTelemetryTest, SupplierControlledFeaturesAreNotOverridden) {
  StreamTables nt;
  bool allianceRelative = false;
  SwerveInputStream<4> stream{*m_drive, [] { return 0.0; }, [] { return 0.0; },
                              [] { return 0.0; }};
  stream.WithAllianceRelativeControl([&] { return allianceRelative; })
      .WithTelemetry(nt.name, Verbosity::HIGH);
  stream.Get();

  allianceRelative = true;
  Tune(stream);
  EXPECT_TRUE(nt.PublishedBoolean("allianceRelative"));

  // The stream still follows the supplier: the telemetry did not replace it with a fixed value.
  allianceRelative = false;
  EXPECT_FALSE(stream.IsAllianceRelativeEnabled());
  Tune(stream);
  EXPECT_FALSE(nt.PublishedBoolean("allianceRelative"));
}

TEST_F(SwerveInputStreamTelemetryTest, WithTelemetryReplacesTheTelemetry) {
  StreamTables first;
  StreamTables second;
  SwerveInputStream<4> stream{*m_drive, [] { return 0.0; }, [] { return 0.0; },
                              [] { return 0.0; }};
  stream.WithTelemetry(first.name, Verbosity::HIGH);
  stream.Get();
  EXPECT_TRUE(first.data->GetTopic("mode").Exists());

  stream.WithTelemetry(second.name, Verbosity::HIGH);
  stream.Get();
  SCOPED_TRACE(::testing::Message() << "the first telemetry is closed");
  EXPECT_FALSE(first.data->GetTopic("mode").Exists());
  EXPECT_FALSE(first.tuning->GetTopic("deadband").Exists());
  EXPECT_TRUE(second.Mode() == "ANGULAR_VELOCITY");

  stream.WithTelemetry(second.name, Verbosity::NONE);
  SCOPED_TRACE(::testing::Message() << "NONE turns telemetry off");
  EXPECT_FALSE(stream.GetTelemetry().has_value());
  EXPECT_FALSE(second.data->GetTopic("mode").Exists());
}

TEST_F(SwerveInputStreamTelemetryTest, CopiesClonesAndMoves) {
  StreamTables nt;
  // Building a stream copies it from the temporary; the copy keeps the telemetry settings.
  SwerveInputStream<4> stream =
      SwerveInputStream<4>::Of(*m_drive, [] { return 0.0; }, [] { return 0.0; })
          .WithControllerRotationAxis([] { return 0.0; })
          .WithTelemetry(nt.name, Verbosity::HIGH);
  EXPECT_TRUE(stream.GetTelemetry().has_value());
  stream.Get();
  EXPECT_TRUE(nt.Mode() == "ANGULAR_VELOCITY");

  SCOPED_TRACE(::testing::Message() << "a clone has no telemetry");
  auto clone = stream.Clone();
  EXPECT_FALSE(clone.GetTelemetry().has_value());

  SCOPED_TRACE(::testing::Message() << "a moved-to stream publishes, the moved-from one stops");
  SwerveInputStream<4> moved = std::move(stream);
  EXPECT_FALSE(stream.GetTelemetry().has_value());
  EXPECT_FALSE(nt.data->GetTopic("mode").Exists());
  moved.Get();
  EXPECT_TRUE(nt.Mode() == "ANGULAR_VELOCITY");
}

TEST_F(SwerveInputStreamTelemetryTest, DestroyingTheStreamStopsPublishing) {
  StreamTables nt;
  auto& scheduler = frc2::CommandScheduler::GetInstance();
  frc2::Command* liveTuning = nullptr;
  {
    SwerveInputStream<4> stream{*m_drive, [] { return 0.0; }, [] { return 0.0; },
                                [] { return 0.0; }};
    stream.WithTelemetry(nt.name, Verbosity::HIGH);
    stream.Get();
    liveTuning = &stream.GetTelemetry()->get().GetLiveTuningCommand()->get();
    scheduler.Schedule(liveTuning);
    scheduler.Run();
    EXPECT_TRUE(scheduler.IsScheduled(liveTuning));
    EXPECT_TRUE(nt.data->GetTopic("deadband").Exists());
  }
  EXPECT_FALSE(nt.data->GetTopic("mode").Exists());
  EXPECT_FALSE(nt.data->GetTopic("deadband").Exists());
  EXPECT_FALSE(nt.tuning->GetTopic("deadband").Exists());
  EXPECT_FALSE(nt.tuning->GetTopic("robotRelative").Exists());
  EXPECT_FALSE(scheduler.IsScheduled(liveTuning)) << "Live Tuning is canceled";
}

TEST_F(SwerveInputStreamTelemetryTest, WithMaximumVelocityOverridesTheDriveConfig) {
  SwerveInputStream<4> stream{*m_drive, [] { return 1.0; }, [] { return 0.0; },
                              [] { return 1.0; }};
  EXPECT_NEAR(stream.Get().vx.value(), kConfigMaxLinear, 1e-9);

  stream.WithMaximumLinearVelocity(2.0_mps)
      .WithMaximumAngularVelocity(units::radians_per_second_t{1.0});
  auto speeds = stream.Get();
  EXPECT_NEAR(speeds.vx.value(), 2.0, 1e-9);
  EXPECT_NEAR(speeds.omega.value(), 1.0, 1e-9);
}

}  // namespace yams::test
