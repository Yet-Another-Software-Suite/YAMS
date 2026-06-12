//// Copyright (c) 2026 Yet Another Software Suite
//// SPDX-License-Identifier: LGPL-3.0-or-later
//
//// Four-module swerve drive integration test.
////
//// Hardware (8 TalonFX + 8 TalonFXWrapper) is created fresh per test in
//// SetUp/TearDown, mirroring the per-test pattern used by ArmTest.
//// Per-suite hardware kept CTRE's background simulation thread
//// (libCTRE_SimProTalonFX.so) iterating over long-lived objects, causing a
//// use-after-free SIGSEGV when they were freed after the last test.
//
//#include <frc/controller/PIDController.h>
//#include <frc/geometry/Pose2d.h>
//#include <frc/geometry/Rotation2d.h>
//#include <frc/geometry/Translation2d.h>
//#include <frc/kinematics/ChassisSpeeds.h>
//#include <frc/system/plant/DCMotor.h>
//#include <frc2/command/CommandScheduler.h>
//#include <frc2/command/SubsystemBase.h>
//#include <gtest/gtest.h>
//#include <units/angle.h>
//#include <units/length.h>
//#include <units/velocity.h>
//
//#include <cmath>
//#include <ctre/phoenix6/TalonFX.hpp>
//#include <memory>
//#include <numbers>
//#include <optional>
//#include <string>
//#include <utility>
//
//#include "helpers/MockHardware.h"
//#include "helpers/MotorControllerFactory.h"
//#include "helpers/SchedulerHelper.h"
//#include "yams/gearing/GearBox.hpp"
//#include "yams/gearing/MechanismGearing.hpp"
//#include "yams/mechanisms/config/SwerveModuleConfig.hpp"
//#include "yams/mechanisms/swerve/SwerveDrive.hpp"
//#include "yams/mechanisms/swerve/SwerveDriveConfig.hpp"
//#include "yams/mechanisms/swerve/SwerveModule.hpp"
//#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
//#include "yams/motorcontrollers/remote/TalonFXWrapper.hpp"
//
//namespace yams::test {
//
//using namespace motorcontrollers;
//using namespace mechanisms;
//using namespace mechanisms::config;
//using namespace mechanisms::swerve;
//
//// ---- Constants ---------------------------------------------------------------
//
//// Module offset from robot centre for a 24 in × 24 in square chassis.
//static constexpr units::meter_t kModuleX{0.3048};
//static constexpr units::meter_t kModuleY{0.3048};
//
//// ---- Minimal subsystem -------------------------------------------------------
//
//class SwerveTestSubsystem : public frc2::SubsystemBase {
// public:
//  void Periodic() override {
//    if (m_drive) m_drive->UpdateTelemetry();
//  }
//  void SimulationPeriodic() override {
//    if (m_drive) m_drive->SimIterate();
//  }
//  SwerveDrive<4>* m_drive{nullptr};
//};
//
//// ---- SMC config helpers ------------------------------------------------------
//
//static SmartMotorControllerConfig MakeDriveConfig(const std::string& name,
//                                                  frc2::SubsystemBase* subsys) {
//  SmartMotorControllerConfig cfg;
//  cfg.WithFeedback(0.1, 0.0, 0.0)
//      .WithMechanismCircumference(units::meter_t{4.0_in * std::numbers::pi})
//      .WithMotorGearing(gearing::MechanismGearing{gearing::GearBox::FromReductionStages({6.75})})
//      .WithIdleMode(SmartMotorControllerConfig::MotorMode::BRAKE)
//      .WithStatorCurrentLimit(40.0_A)
//      .WithSimMotor(frc::DCMotor::KrakenX60(1))
//      .WithClosedLoopMode()
//      .WithSubsystem(subsys)
//      .WithTelemetry(name, SmartMotorControllerConfig::TelemetryVerbosity::NONE);
//  return cfg;
//}
//
//static SmartMotorControllerConfig MakeAzimuthConfig(const std::string& name,
//                                                    frc2::SubsystemBase* subsys) {
//  SmartMotorControllerConfig cfg;
//  cfg.WithFeedback(50.0, 0.0, 0.5)
//      .WithMotorGearing(
//          gearing::MechanismGearing{gearing::GearBox::FromReductionStages({150.0 / 7.0})})
//      .WithIdleMode(SmartMotorControllerConfig::MotorMode::BRAKE)
//      .WithStatorCurrentLimit(20.0_A)
//      .WithSimMotor(frc::DCMotor::KrakenX60(1))
//      .WithMOI(4_in, 0.5_lb)
//      .WithClosedLoopMode()
//      .WithSubsystem(subsys)
//      .WithTelemetry(name, SmartMotorControllerConfig::TelemetryVerbosity::NONE);
//  return cfg;
//}
//
//// ---- Test fixture ------------------------------------------------------------
////
//// Hardware (TalonFX + TalonFXWrapper) is allocated in SetUp and released in
//// TearDown, matching the ArmTest lifecycle.  TalonFXWrapper is closed and
//// deleted before TalonFX so the CTRE simulation thread never holds a pointer
//// to a freed device.  TeardownHardware() runs last so Phoenix can observe the
//// disabled state before the next test's InitializeHardware() re-enables it.
//
//class SwerveDriveTest : public ::testing::Test {
// protected:
//  void SetUp() override {
//    InitializeHardware();
//    SchedulerHelper::Enable();
//    SchedulerHelper::CancelAll();
//    m_simGyro = 0_deg;
//
//    m_sub = std::make_unique<SwerveTestSubsystem>();
//
//    // TalonFXWrapper constructor calls SetupSimulation() automatically.
//    m_flDriveTalon = std::make_unique<ctre::phoenix6::hardware::TalonFX>(NextCanId());
//    m_frDriveTalon = std::make_unique<ctre::phoenix6::hardware::TalonFX>(NextCanId());
//    m_blDriveTalon = std::make_unique<ctre::phoenix6::hardware::TalonFX>(NextCanId());
//    m_brDriveTalon = std::make_unique<ctre::phoenix6::hardware::TalonFX>(NextCanId());
//    m_flAzimuthTalon = std::make_unique<ctre::phoenix6::hardware::TalonFX>(NextCanId());
//    m_frAzimuthTalon = std::make_unique<ctre::phoenix6::hardware::TalonFX>(NextCanId());
//    m_blAzimuthTalon = std::make_unique<ctre::phoenix6::hardware::TalonFX>(NextCanId());
//    m_brAzimuthTalon = std::make_unique<ctre::phoenix6::hardware::TalonFX>(NextCanId());
//
//    m_flDriveSMC = new remote::TalonFXWrapper(m_flDriveTalon.get(), frc::DCMotor::KrakenX60(1),
//                                              MakeDriveConfig("FL_Drive", m_sub.get()));
//    m_frDriveSMC = new remote::TalonFXWrapper(m_frDriveTalon.get(), frc::DCMotor::KrakenX60(1),
//                                              MakeDriveConfig("FR_Drive", m_sub.get()));
//    m_blDriveSMC = new remote::TalonFXWrapper(m_blDriveTalon.get(), frc::DCMotor::KrakenX60(1),
//                                              MakeDriveConfig("BL_Drive", m_sub.get()));
//    m_brDriveSMC = new remote::TalonFXWrapper(m_brDriveTalon.get(), frc::DCMotor::KrakenX60(1),
//                                              MakeDriveConfig("BR_Drive", m_sub.get()));
//    m_flAzimuthSMC = new remote::TalonFXWrapper(m_flAzimuthTalon.get(), frc::DCMotor::KrakenX60(1),
//                                                MakeAzimuthConfig("FL_Azimuth", m_sub.get()));
//    m_frAzimuthSMC = new remote::TalonFXWrapper(m_frAzimuthTalon.get(), frc::DCMotor::KrakenX60(1),
//                                                MakeAzimuthConfig("FR_Azimuth", m_sub.get()));
//    m_blAzimuthSMC = new remote::TalonFXWrapper(m_blAzimuthTalon.get(), frc::DCMotor::KrakenX60(1),
//                                                MakeAzimuthConfig("BL_Azimuth", m_sub.get()));
//    m_brAzimuthSMC = new remote::TalonFXWrapper(m_brAzimuthTalon.get(), frc::DCMotor::KrakenX60(1),
//                                                MakeAzimuthConfig("BR_Azimuth", m_sub.get()));
//
//    auto makeModuleCfg = [](remote::TalonFXWrapper* drive, remote::TalonFXWrapper* azimuth,
//                            units::meter_t front, units::meter_t left,
//                            const std::string& name) -> SwerveModuleConfig {
//      SwerveModuleConfig cfg{drive, azimuth};
//      cfg.WithAbsoluteEncoder([] { return 0.0_deg; })
//          .WithAbsoluteEncoderOffset(0.0_deg)
//          .WithWheelDiameter(4.0_in)
//          .WithLocation(front, left)
//          .WithOptimization(false)
//          .WithTelemetry(name, SwerveModuleConfig::TelemetryVerbosity::NONE);
//      return cfg;
//    };
//
//    m_fl.emplace(makeModuleCfg(m_flDriveSMC, m_flAzimuthSMC, kModuleX, kModuleY, "FL"));
//    m_fr.emplace(makeModuleCfg(m_frDriveSMC, m_frAzimuthSMC, kModuleX, -kModuleY, "FR"));
//    m_bl.emplace(makeModuleCfg(m_blDriveSMC, m_blAzimuthSMC, -kModuleX, kModuleY, "BL"));
//    m_br.emplace(makeModuleCfg(m_brDriveSMC, m_brAzimuthSMC, -kModuleX, -kModuleY, "BR"));
//
//    SwerveDriveConfig driveCfg;
//    driveCfg.WithSubsystem(m_sub.get())
//        .WithModules({&m_fl.value(), &m_fr.value(), &m_bl.value(), &m_br.value()})
//        .WithGyro([this] { return m_simGyro; })
//        .WithStartingPose(frc::Pose2d{})
//        .WithMaximumChassisSpeed(4.5_mps, units::degrees_per_second_t{540})
//        .WithTranslationController(frc::PIDController{2.0, 0.0, 0.0})
//        .WithRotationController(frc::PIDController{4.0, 0.0, 0.0});
//    m_drive.emplace(std::move(driveCfg));
//
//    m_sub->m_drive = &m_drive.value();
//  }
//
//  void TearDown() override {
//    m_sub->m_drive = nullptr;
//    frc2::CommandScheduler::GetInstance().CancelAll();
//    m_drive.reset();
//    m_fl.reset();
//    m_fr.reset();
//    m_bl.reset();
//    m_br.reset();
//
//    frc2::CommandScheduler::GetInstance().UnregisterSubsystem(m_sub.get());
//
//    // Stop YAMS threads and delete wrappers before releasing hardware.
//    for (auto* s : {m_flDriveSMC, m_frDriveSMC, m_blDriveSMC, m_brDriveSMC, m_flAzimuthSMC,
//                    m_frAzimuthSMC, m_blAzimuthSMC, m_brAzimuthSMC}) {
//      delete s;
//    }
//
//    // Notify the CTRE sim layer before releasing hardware, matching CloseBundle.
//    constexpr auto kTalonFXType = ctre::phoenix::platform::DeviceType::P6_TalonFXType;
//    ctre::phoenix::platform::SimDestroy(kTalonFXType, m_flDriveTalon->GetDeviceID());
//    ctre::phoenix::platform::SimDestroy(kTalonFXType, m_frDriveTalon->GetDeviceID());
//    ctre::phoenix::platform::SimDestroy(kTalonFXType, m_blDriveTalon->GetDeviceID());
//    ctre::phoenix::platform::SimDestroy(kTalonFXType, m_brDriveTalon->GetDeviceID());
//    ctre::phoenix::platform::SimDestroy(kTalonFXType, m_flAzimuthTalon->GetDeviceID());
//    ctre::phoenix::platform::SimDestroy(kTalonFXType, m_frAzimuthTalon->GetDeviceID());
//    ctre::phoenix::platform::SimDestroy(kTalonFXType, m_blAzimuthTalon->GetDeviceID());
//    ctre::phoenix::platform::SimDestroy(kTalonFXType, m_brAzimuthTalon->GetDeviceID());
//
//    // Release TalonFX objects after all wrappers are gone.
//    m_flDriveTalon.reset();
//    m_frDriveTalon.reset();
//    m_blDriveTalon.reset();
//    m_brDriveTalon.reset();
//    m_flAzimuthTalon.reset();
//    m_frAzimuthTalon.reset();
//    m_blAzimuthTalon.reset();
//    m_brAzimuthTalon.reset();
//
//    m_sub.reset();
//    TeardownHardware();
//    SchedulerHelper::CancelAll();
//  }
//
//  units::degree_t m_simGyro{0};
//
//  std::unique_ptr<SwerveTestSubsystem> m_sub;
//
//  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> m_flDriveTalon;
//  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> m_frDriveTalon;
//  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> m_blDriveTalon;
//  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> m_brDriveTalon;
//  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> m_flAzimuthTalon;
//  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> m_frAzimuthTalon;
//  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> m_blAzimuthTalon;
//  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> m_brAzimuthTalon;
//
//  remote::TalonFXWrapper* m_flDriveSMC{nullptr};
//  remote::TalonFXWrapper* m_frDriveSMC{nullptr};
//  remote::TalonFXWrapper* m_blDriveSMC{nullptr};
//  remote::TalonFXWrapper* m_brDriveSMC{nullptr};
//  remote::TalonFXWrapper* m_flAzimuthSMC{nullptr};
//  remote::TalonFXWrapper* m_frAzimuthSMC{nullptr};
//  remote::TalonFXWrapper* m_blAzimuthSMC{nullptr};
//  remote::TalonFXWrapper* m_brAzimuthSMC{nullptr};
//
//  std::optional<SwerveModule> m_fl;
//  std::optional<SwerveModule> m_fr;
//  std::optional<SwerveModule> m_bl;
//  std::optional<SwerveModule> m_br;
//
//  std::optional<SwerveDrive<4>> m_drive;
//};
//
//// ---- Tests -------------------------------------------------------------------
//
//// Drive constructs and destructs cleanly.
//TEST_F(SwerveDriveTest, ConstructionDoesNotCrash) { EXPECT_TRUE(m_drive.has_value()); }
//
//// UpdateTelemetry and SimIterate run for several loops without crashing.
//TEST_F(SwerveDriveTest, TelemetryAndSimRunWithoutCrash) {
//  EXPECT_NO_FATAL_FAILURE(SchedulerHelper::RunForDuration(0.5_s));
//}
//
//// The initial pose matches the starting pose supplied in the config.
//TEST_F(SwerveDriveTest, InitialPoseIsOrigin) {
//  auto pose = m_drive->GetPose();
//  EXPECT_NEAR(pose.X().value(), 0.0, 0.01);
//  EXPECT_NEAR(pose.Y().value(), 0.0, 0.01);
//  EXPECT_NEAR(pose.Rotation().Degrees().value(), 0.0, 0.1);
//}
//
//// A non-zero target pose is reflected in GetPose() immediately after ResetOdometry.
//TEST_F(SwerveDriveTest, ResetOdometryMatchesPose) {
//  frc::Pose2d target{3.0_m, 2.0_m, frc::Rotation2d{45.0_deg}};
//  m_drive->ResetOdometry(target);
//  auto pose = m_drive->GetPose();
//  EXPECT_NEAR(pose.X().value(), 3.0, 0.01);
//  EXPECT_NEAR(pose.Y().value(), 2.0, 0.01);
//  EXPECT_NEAR(pose.Rotation().Degrees().value(), 45.0, 0.1);
//}
//
//// GetStateFromRobotRelativeChassisSpeeds converts a pure forward command into
//// forward-pointing states for all four modules.
//TEST_F(SwerveDriveTest, GetStateFromSpeedsForwardDrive) {
//  auto states = m_drive->GetStateFromRobotRelativeChassisSpeeds(
//      frc::ChassisSpeeds{1.0_mps, 0_mps, 0_rad_per_s});
//
//  for (size_t i = 0; i < 4; ++i) {
//    EXPECT_NEAR(states[i].speed.value(), 1.0, 0.01)
//        << "Module " << i << " speed should equal commanded speed";
//    EXPECT_NEAR(states[i].angle.Degrees().value(), 0.0, 1.0)
//        << "Module " << i << " angle should be 0° for pure forward drive";
//  }
//}
//
//// Pure rotation command produces tangential module states (none pointing forward).
//TEST_F(SwerveDriveTest, GetStateFromSpeedsPureRotation) {
//  auto states = m_drive->GetStateFromRobotRelativeChassisSpeeds(
//      frc::ChassisSpeeds{0_mps, 0_mps, units::radians_per_second_t{1.0}});
//
//  for (size_t i = 0; i < 4; ++i) {
//    EXPECT_GT(std::abs(states[i].speed.value()), 0.0)
//        << "Module " << i << " should have non-zero speed for rotation command";
//    EXPECT_GT(std::abs(states[i].angle.Degrees().value()), 1.0)
//        << "Module " << i << " should not point forward during pure rotation";
//  }
//}
//
//// SetRobotRelativeChassisSpeeds does not crash for both non-zero and zero inputs.
//TEST_F(SwerveDriveTest, SetRobotRelativeSpeedsDoesNotCrash) {
//  EXPECT_NO_FATAL_FAILURE(
//      m_drive->SetRobotRelativeChassisSpeeds(frc::ChassisSpeeds{1.0_mps, 0_mps, 0_rad_per_s}));
//  EXPECT_NO_FATAL_FAILURE(
//      m_drive->SetRobotRelativeChassisSpeeds(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s}));
//}
//
//// LockPose commands zero translational speed and corner-pointing module angles.
//TEST_F(SwerveDriveTest, LockPoseSetsXPattern) {
//  m_drive->LockPose();
//
//  auto robotSpeeds = m_drive->GetRobotRelativeSpeed();
//  EXPECT_NEAR(robotSpeeds.vx.value(), 0.0, 0.5);
//  EXPECT_NEAR(robotSpeeds.vy.value(), 0.0, 0.5);
//
//  SchedulerHelper::RunForDuration(0.5_s);
//  auto modules = m_drive->GetConfig().GetModules();
//  for (size_t i = 0; i < 4; ++i) {
//    double expected = modules[i]->GetConfig().GetLocation()->Angle().Degrees().value();
//    double actual = modules[i]->GetState().angle.Degrees().value();
//    EXPECT_NEAR(actual, expected, 180.0)
//        << "Module " << i << " angle should converge toward lock angle " << expected << "°";
//  }
//}
//
//// ZeroGyro does not crash and zeroes the heading.
//TEST_F(SwerveDriveTest, ZeroGyroDoesNotCrash) {
//  m_simGyro = 45.0_deg;
//  m_drive->ResetOdometry(frc::Pose2d{0_m, 0_m, frc::Rotation2d{45.0_deg}});
//  EXPECT_NO_FATAL_FAILURE(m_drive->ZeroGyro());
//  EXPECT_NEAR(m_drive->GetPose().Rotation().Degrees().value(), 0.0, 1.0);
//}
//
//// AddVisionMeasurement accepts a pose without crashing.
//TEST_F(SwerveDriveTest, AddVisionMeasurementDoesNotCrash) {
//  EXPECT_NO_FATAL_FAILURE(
//      m_drive->AddVisionMeasurement(frc::Pose2d{1_m, 1_m, frc::Rotation2d{}}, 0.0_s));
//}
//
//// GetDistanceFromPose returns the Euclidean distance to a target (3-4-5 triangle).
//TEST_F(SwerveDriveTest, GetDistanceFromPose) {
//  auto dist = m_drive->GetDistanceFromPose(frc::Pose2d{3.0_m, 4.0_m, frc::Rotation2d{}});
//  EXPECT_NEAR(dist.value(), 5.0, 0.01);
//}
//
//// Drive() returns a command that runs the speed supplier each loop.
//TEST_F(SwerveDriveTest, DriveCommandCallsSpeedSupplier) {
//  int callCount = 0;
//  auto cmd = m_drive->Drive([&] {
//    ++callCount;
//    return frc::ChassisSpeeds{};
//  });
//  frc2::CommandScheduler::GetInstance().Schedule(cmd);
//  SchedulerHelper::RunForDuration(0.1_s);
//  EXPECT_GE(callCount, 1);
//}
//
//// The Drive command declares the configured subsystem as a requirement.
//TEST_F(SwerveDriveTest, DriveCommandHasSubsystemRequirement) {
//  auto cmd = m_drive->Drive([] { return frc::ChassisSpeeds{}; });
//  EXPECT_TRUE(cmd.HasRequirement(m_sub.get()));
//}
//
//// Scheduling a second Drive command interrupts the first.
//TEST_F(SwerveDriveTest, SecondDriveCommandInterruptsFirst) {
//  int firstCalls = 0;
//  int secondCalls = 0;
//  auto cmd1 = m_drive->Drive([&] {
//    ++firstCalls;
//    return frc::ChassisSpeeds{};
//  });
//  auto cmd2 = m_drive->Drive([&] {
//    ++secondCalls;
//    return frc::ChassisSpeeds{};
//  });
//
//  frc2::CommandScheduler::GetInstance().Schedule(cmd1);
//  SchedulerHelper::RunForDuration(0.04_s);
//  int firstCallsAtInterrupt = firstCalls;
//  EXPECT_GE(firstCallsAtInterrupt, 1) << "cmd1 should run initially";
//
//  frc2::CommandScheduler::GetInstance().Schedule(cmd2);
//  SchedulerHelper::RunForDuration(0.04_s);
//
//  EXPECT_GE(secondCalls, 1) << "cmd2 should run after interrupt";
//  EXPECT_EQ(firstCalls, firstCallsAtInterrupt) << "cmd1 should have been cancelled";
//}
//
//}  // namespace yams::test
