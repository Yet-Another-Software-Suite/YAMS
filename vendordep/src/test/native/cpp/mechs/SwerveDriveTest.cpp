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

#include <wpi/math/controller/PIDController.hpp>
#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/math/kinematics/ChassisVelocities.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/commands2/CommandScheduler.hpp>
#include <wpi/commands2/CommandScheduler.hpp>
#include <wpi/commands2/SubsystemBase.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/velocity.hpp>

#include <cmath>
#include <ctre/phoenix6/TalonFX.hpp>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#include <wpi/nt/NetworkTableInstance.hpp>

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
      .WithZeroPower(SmartMotorControllerConfig::MotorMode::BRAKE)
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
      .WithZeroPower(SmartMotorControllerConfig::MotorMode::BRAKE)
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
        new ctre::phoenix6::hardware::TalonFX(NextReservedCanId(), ctre::phoenix6::CANBus{});
    frDriveTalon =
        new ctre::phoenix6::hardware::TalonFX(NextReservedCanId(), ctre::phoenix6::CANBus{});
    blDriveTalon =
        new ctre::phoenix6::hardware::TalonFX(NextReservedCanId(), ctre::phoenix6::CANBus{});
    brDriveTalon =
        new ctre::phoenix6::hardware::TalonFX(NextReservedCanId(), ctre::phoenix6::CANBus{});
    flAzimuthTalon =
        new ctre::phoenix6::hardware::TalonFX(NextReservedCanId(), ctre::phoenix6::CANBus{});
    frAzimuthTalon =
        new ctre::phoenix6::hardware::TalonFX(NextReservedCanId(), ctre::phoenix6::CANBus{});
    blAzimuthTalon =
        new ctre::phoenix6::hardware::TalonFX(NextReservedCanId(), ctre::phoenix6::CANBus{});
    brAzimuthTalon =
        new ctre::phoenix6::hardware::TalonFX(NextReservedCanId(), ctre::phoenix6::CANBus{});

    flDriveCfg = MakeDriveConfig("FL_Drive", sub);
    frDriveCfg = MakeDriveConfig("FR_Drive", sub);
    blDriveCfg = MakeDriveConfig("BL_Drive", sub);
    brDriveCfg = MakeDriveConfig("BR_Drive", sub);
    flAzimuthCfg = MakeAzimuthConfig("FL_Azimuth", sub);
    frAzimuthCfg = MakeAzimuthConfig("FR_Azimuth", sub);
    blAzimuthCfg = MakeAzimuthConfig("BL_Azimuth", sub);
    brAzimuthCfg = MakeAzimuthConfig("BR_Azimuth", sub);

    // TalonFXWrapper constructor calls SetupSimulation() automatically.
    flDriveSMC = new remote::TalonFXWrapper(flDriveTalon, wpi::math::DCMotor::KrakenX60(1),
                                            &flDriveCfg);
    frDriveSMC = new remote::TalonFXWrapper(frDriveTalon, wpi::math::DCMotor::KrakenX60(1),
                                            &frDriveCfg);
    blDriveSMC = new remote::TalonFXWrapper(blDriveTalon, wpi::math::DCMotor::KrakenX60(1),
                                            &blDriveCfg);
    brDriveSMC = new remote::TalonFXWrapper(brDriveTalon, wpi::math::DCMotor::KrakenX60(1),
                                            &brDriveCfg);
    flAzimuthSMC = new remote::TalonFXWrapper(flAzimuthTalon, wpi::math::DCMotor::KrakenX60(1),
                                              &flAzimuthCfg);
    frAzimuthSMC = new remote::TalonFXWrapper(frAzimuthTalon, wpi::math::DCMotor::KrakenX60(1),
                                              &frAzimuthCfg);
    blAzimuthSMC = new remote::TalonFXWrapper(blAzimuthTalon, wpi::math::DCMotor::KrakenX60(1),
                                              &blAzimuthCfg);
    brAzimuthSMC = new remote::TalonFXWrapper(brAzimuthTalon, wpi::math::DCMotor::KrakenX60(1),
                                              &brAzimuthCfg);
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
    // Other suites' teardown resets DriverStation sim data (disabling the robot), and the shared
    // hardware above only initializes once, so re-enable before each test so commands run.
    InitializeHardware();
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
        .WithStartingPose(wpi::math::Pose2d{})
        .WithMaximumChassisSpeed(4.5_mps, wpi::units::degrees_per_second_t{540})
        .WithTranslationController(wpi::math::PIDController{2.0, 0.0, 0.0})
        .WithRotationController(wpi::math::PIDController{4.0, 0.0, 0.0});
    m_drive.emplace(&m_driveCfg);

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

// ---- Tests -------------------------------------------------------------------

// Drive constructs and destructs cleanly.
TEST_CASE_METHOD(SwerveDriveTestFixture, "SwerveDriveTest.ConstructionDoesNotCrash",
                 "[SwerveDriveTest]") {
  CHECK(m_drive.has_value());
}

// Configured translation and rotation controllers are returned by the config.
TEST_CASE_METHOD(SwerveDriveTestFixture, "SwerveDriveTest.ConfiguredPIDControllersArePresent",
                 "[SwerveDriveTest]") {
  auto translationPID = m_driveCfg.GetTranslationPID();
  auto rotationPID = m_driveCfg.GetRotationPID();
  REQUIRE(translationPID.has_value());
  REQUIRE(rotationPID.has_value());
  CHECK(translationPID->get().GetP() == Catch::Approx(2.0));
  CHECK(rotationPID->get().GetP() == Catch::Approx(4.0));
}

// Translation and rotation controllers are optional: a drive without them constructs, runs its
// telemetry, and resets cleanly, and only drive to pose reports that they are missing.
TEST_CASE_METHOD(SwerveDriveTestFixture, "SwerveDriveTest.PIDControllersAreOptional",
                 "[SwerveDriveTest]") {
  // Release the fixture's drive so its modules can be reused by a drive without controllers.
  Hardware().sub->m_drive = nullptr;
  m_drive.reset();

  // Declared before the drive so the config outlives it.
  SwerveDriveConfig cfg;
  cfg.WithSubsystem(Hardware().sub)
      .WithModules({&m_fl.value(), &m_fr.value(), &m_bl.value(), &m_br.value()})
      .WithGyro([this] { return m_simGyro; })
      .WithStartingPose(wpi::math::Pose2d{})
      .WithMaximumChassisSpeed(4.5_mps, wpi::units::degrees_per_second_t{540});
  CHECK_FALSE(cfg.GetTranslationPID().has_value());
  CHECK_FALSE(cfg.GetRotationPID().has_value());

  std::optional<SwerveDrive<4>> drive;
  REQUIRE_NOTHROW(drive.emplace(&cfg));
  CHECK_NOTHROW(drive->UpdateTelemetry());
  CHECK_NOTHROW(drive->SimIterate());
  CHECK_NOTHROW(drive->ResetTranslationPID());
  CHECK_NOTHROW(drive->ResetRotationPID());
  CHECK_THROWS_AS(drive->DriveToPoseSetpoint(wpi::math::Pose2d{}),
                  yams::exceptions::SwerveDriveConfigurationException);
  drive.reset();
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

// ResetOdometry sets the gyro to the pose's heading so field relative driving and heading control,
// which use the gyro, agree with the reset pose. ZeroGyro resets both to 0 degrees.
TEST_CASE_METHOD(SwerveDriveTestFixture, "SwerveDriveTest.ResetOdometryAlignsGyro",
                 "[SwerveDriveTest]") {
  m_drive->ResetOdometry(wpi::math::Pose2d{1.0_m, 2.0_m, wpi::math::Rotation2d{90.0_deg}});
  CHECK(m_drive->GetGyroAngle().value() == Catch::Approx(90.0).margin(0.1));
  CHECK(m_drive->GetPose().Rotation().Degrees().value() == Catch::Approx(90.0).margin(0.1));

  m_drive->ZeroGyro();
  CHECK(m_drive->GetGyroAngle().value() == Catch::Approx(0.0).margin(0.1));
  CHECK(m_drive->GetPose().Rotation().Degrees().value() == Catch::Approx(0.0).margin(0.1));
  CHECK(m_drive->GetPose().X().value() == Catch::Approx(1.0).margin(0.01));
}

// GetStateFromRobotRelativeChassisSpeeds converts a pure forward command into
// forward-pointing states for all four modules.
TEST_CASE_METHOD(SwerveDriveTestFixture, "SwerveDriveTest.GetStateFromSpeedsForwardDrive",
                 "[SwerveDriveTest]") {
  auto states = m_drive->GetStateFromRobotRelativeChassisSpeeds(
      wpi::math::ChassisVelocities{1.0_mps, 0_mps, 0_rad_per_s});

  for (size_t i = 0; i < 4; ++i) {
    INFO("Module " << i << " speed should equal commanded speed");
    CHECK(states[i].velocity.value() == Catch::Approx(1.0).margin(0.01));
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
    CHECK(std::abs(states[i].velocity.value()) > 0.0);
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
  auto desiredSpeeds = m_drive->GetDesiredChassisSpeeds();
  CHECK(desiredSpeeds.vx.value() == Catch::Approx(0.0).margin(1e-9));
  CHECK(desiredSpeeds.vy.value() == Catch::Approx(0.0).margin(1e-9));
  CHECK(desiredSpeeds.omega.value() == Catch::Approx(0.0).margin(1e-9));

  // Run sim and check azimuth convergence toward X-pattern corner angles.
  SchedulerHelper::RunForDuration(0.5_s);
  auto modules = m_drive->GetConfig().GetModules();
  for (size_t i = 0; i < 4; ++i) {
    double expected = modules[i]->GetConfig().GetLocation()->Angle()->Degrees().value();
    double actual = modules[i]->GetState().angle.Degrees().value();
    // Module optimization may reverse the wheel, so angles 180° apart are equivalent.
    double error = std::remainder(actual - expected, 180.0);
    INFO("Module " << i << " angle " << actual << "° should converge toward lock angle "
                   << expected << "° (mod 180°)");
    CHECK(error == Catch::Approx(0.0).margin(10.0));
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
        data{wpi::nt::NetworkTableInstance::GetDefault().GetTable("SwerveInputStream")->GetSubTable(
            name)},
        tuning{wpi::nt::NetworkTableInstance::GetDefault()
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
  bool LiveTuningPublished() { return !tuning->GetSubTable("Live Tuning")->GetKeys().empty(); }

  std::string name;
  std::shared_ptr<wpi::nt::NetworkTable> data;
  std::shared_ptr<wpi::nt::NetworkTable> tuning;
};

// Apply dashboard edits, as one loop of the Live Tuning command does.
void Tune(SwerveInputStream<4>& stream) {
  stream.GetTelemetry()->get().ApplyTuningValues();
}

}  // namespace

TEST_CASE_METHOD(SwerveDriveTestFixture, "SwerveInputStreamTelemetryTest.LowPublishesOnlyTheMode",
                 "[SwerveInputStreamTelemetryTest]") {
  StreamTables nt;
  SwerveInputStream<4> stream{*m_drive, [] { return 0.0; }, [] { return 0.0; },
                              [] { return 0.0; }};
  stream.WithDeadband(0.1).WithTelemetry(nt.name, Verbosity::LOW);
  stream.Get();

  CHECK(nt.Mode() == "ANGULAR_VELOCITY");
  CHECK_FALSE(nt.data->GetTopic("deadband").Exists());
  CHECK_FALSE(nt.tuning->GetTopic("deadband").Exists());
  CHECK_FALSE(stream.GetTelemetry()->get().GetLiveTuningCommand().has_value());
}

TEST_CASE_METHOD(SwerveDriveTestFixture,
                 "SwerveInputStreamTelemetryTest.MediumPublishesTheConfigurationReadOnly",
                 "[SwerveInputStreamTelemetryTest]") {
  StreamTables nt;
  SwerveInputStream<4> stream{*m_drive, [] { return 0.0; }, [] { return 0.0; },
                              [] { return 0.0; }};
  stream.WithDeadband(0.1).WithScaleTranslation(0.8).WithTelemetry(nt.name, Verbosity::MEDIUM);
  stream.Get();

  CHECK(nt.Data("deadband") == Catch::Approx(0.1));
  CHECK(nt.Data("translationScale") == Catch::Approx(0.8));
  CHECK(nt.Data("maxLinearVelocity") == Catch::Approx(kConfigMaxLinear));
  CHECK_FALSE(nt.tuning->GetTopic("deadband").Exists());
  CHECK_FALSE(stream.GetTelemetry()->get().GetLiveTuningCommand().has_value());

  // Changes made in code are published when the stream is read.
  stream.WithScaleTranslation(0.5);
  stream.Get();
  CHECK(nt.Data("translationScale") == Catch::Approx(0.5));
}

TEST_CASE_METHOD(SwerveDriveTestFixture,
                 "SwerveInputStreamTelemetryTest.HighPublishesTuningValuesAndTheLiveTuningCommand",
                 "[SwerveInputStreamTelemetryTest]") {
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

  CHECK(nt.Data("deadband") == Catch::Approx(0.1));
  CHECK(nt.Published("deadband") == Catch::Approx(0.1));
  CHECK(nt.Published("translationScale") == Catch::Approx(0.8));
  CHECK(nt.Published("rotationScale") == Catch::Approx(0.5));
  CHECK(nt.Published("maxLinearVelocity") == Catch::Approx(kConfigMaxLinear));
  CHECK(nt.Published("maxAngularVelocity") == Catch::Approx(kConfigMaxAngular));
  CHECK(nt.PublishedBoolean("translationCube"));
  CHECK_FALSE(nt.PublishedBoolean("rotationCube"));
  CHECK(nt.PublishedBoolean("allianceRelative"));
  CHECK_FALSE(nt.PublishedBoolean("robotRelative"));
  CHECK(stream.GetTelemetry()->get().GetLiveTuningCommand().has_value());
  CHECK(nt.LiveTuningPublished());
}

TEST_CASE_METHOD(SwerveDriveTestFixture,
                 "SwerveInputStreamTelemetryTest.ReadingTheStreamPublishesTheMode",
                 "[SwerveInputStreamTelemetryTest]") {
  StreamTables nt;
  bool headingControl = false;
  auto stream = SwerveInputStream<4>::Of(*m_drive, [] { return 0.0; }, [] { return 0.0; });
  stream.WithControllerHeadingAxis([] { return 0.0; }, [] { return 1.0; })
      .WithHeadingControl([&] { return headingControl; })
      .WithTelemetry(nt.name, Verbosity::LOW);

  // No rotation axis, so the stream holds its heading until heading control is on.
  stream.Get();
  CHECK(nt.Mode() == "TRANSLATION_ONLY");
  headingControl = true;
  stream.Get();
  CHECK(nt.Mode() == "HEADING");
}

TEST_CASE_METHOD(SwerveDriveTestFixture,
                 "SwerveInputStreamTelemetryTest.DashboardEditsApplyOnlyWhileLiveTuningRuns",
                 "[SwerveInputStreamTelemetryTest]") {
  StreamTables nt;
  SwerveInputStream<4> stream{*m_drive, [] { return 0.0; }, [] { return 0.0; },
                              [] { return 0.0; }};
  stream.WithTelemetry(nt.name, Verbosity::HIGH);
  stream.Get();
  wpi::cmd::Command& liveTuning = *stream.GetTelemetry()->get().GetLiveTuningCommand();
  auto& scheduler = wpi::cmd::CommandScheduler::GetInstance();

  nt.Dashboard("deadband", 0.2);
  stream.Get();
  INFO("not applied before Live Tuning runs");
  CHECK(stream.GetAxisDeadband() == Catch::Approx(0.0));

  scheduler.Schedule(&liveTuning);
  scheduler.Run();
  INFO("applied while Live Tuning runs");
  CHECK(stream.GetAxisDeadband() == Catch::Approx(0.2));

  scheduler.Cancel(&liveTuning);
  scheduler.Run();
  nt.Dashboard("deadband", 0.3);
  stream.Get();
  INFO("not applied after Live Tuning stops");
  CHECK(stream.GetAxisDeadband() == Catch::Approx(0.2));
}

TEST_CASE_METHOD(SwerveDriveTestFixture,
                 "SwerveInputStreamTelemetryTest.DashboardEditsAreAppliedToTheStream",
                 "[SwerveInputStreamTelemetryTest]") {
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

  CHECK(stream.GetAxisDeadband() == Catch::Approx(0.2));
  CHECK(stream.GetTranslationAxisScale() == Catch::Approx(0.6));
  CHECK(stream.GetOmegaAxisScale() == Catch::Approx(0.4));
  CHECK(stream.GetMaximumChassisLinearVelocity().value() == Catch::Approx(3.0));
  CHECK(stream.GetMaximumChassisAngularVelocity().value() == Catch::Approx(5.0));
  CHECK(stream.IsTranslationCubeEnabled());
  CHECK(stream.IsOmegaCubeEnabled());
  CHECK(stream.IsAllianceRelativeEnabled());
  CHECK(stream.IsRobotRelativeEnabled());

  // The edits stay applied on later loops.
  Tune(stream);
  CHECK(stream.GetAxisDeadband() == Catch::Approx(0.2));
  CHECK(nt.Published("deadband") == Catch::Approx(0.2));
  CHECK(stream.IsRobotRelativeEnabled());

  // Turning a feature back off on the dashboard turns it off in the stream.
  nt.Dashboard("translationCube", false);
  Tune(stream);
  CHECK_FALSE(stream.IsTranslationCubeEnabled());
}

TEST_CASE_METHOD(SwerveDriveTestFixture,
                 "SwerveInputStreamTelemetryTest.TunedValuesChangeTheOutput",
                 "[SwerveInputStreamTelemetryTest]") {
  StreamTables nt;
  double forward = 1.0;
  double rotation = 1.0;
  SwerveInputStream<4> stream{*m_drive, [&] { return forward; }, [] { return 0.0; },
                              [&] { return rotation; }};
  stream.WithTelemetry(nt.name, Verbosity::HIGH);

  auto speeds = stream.Get();
  CHECK(speeds.vx.value() == Catch::Approx(kConfigMaxLinear));
  CHECK(speeds.omega.value() == Catch::Approx(kConfigMaxAngular));

  SECTION("maximum velocities override the drive config") {
    nt.Dashboard("maxLinearVelocity", 2.0);
    nt.Dashboard("maxAngularVelocity", 3.0);
    Tune(stream);
    speeds = stream.Get();
    CHECK(speeds.vx.value() == Catch::Approx(2.0));
    CHECK(speeds.omega.value() == Catch::Approx(3.0));
  }

  SECTION("scales") {
    nt.Dashboard("translationScale", 0.5);
    nt.Dashboard("rotationScale", 0.25);
    Tune(stream);
    speeds = stream.Get();
    CHECK(speeds.vx.value() == Catch::Approx(0.5 * kConfigMaxLinear));
    CHECK(speeds.omega.value() == Catch::Approx(0.25 * kConfigMaxAngular));
  }

  SECTION("deadband") {
    forward = 0.3;
    nt.Dashboard("deadband", 0.5);
    Tune(stream);
    CHECK(stream.Get().vx.value() == Catch::Approx(0.0));
  }

  SECTION("rotation cubing") {
    rotation = 0.5;
    nt.Dashboard("rotationCube", true);
    Tune(stream);
    CHECK(stream.Get().omega.value() == Catch::Approx(0.125 * kConfigMaxAngular));
  }
}

TEST_CASE_METHOD(SwerveDriveTestFixture,
                 "SwerveInputStreamTelemetryTest.InvalidDashboardValuesAreReplacedWithTheStreamValue",
                 "[SwerveInputStreamTelemetryTest]") {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  const std::vector<std::pair<std::string, double>> cases{
      {"deadband", -0.1},         {"deadband", 1.0},           {"deadband", nan},
      {"translationScale", 0.0},  {"translationScale", 1.5},   {"rotationScale", -0.5},
      {"rotationScale", nan},     {"maxLinearVelocity", 0.0},  {"maxLinearVelocity", -1.0},
      {"maxLinearVelocity", infinity}, {"maxAngularVelocity", 0.0}, {"maxAngularVelocity", nan}};
  for (const auto& [key, value] : cases) {
    INFO(key << " = " << value);
    StreamTables nt;
    SwerveInputStream<4> stream{*m_drive, [] { return 0.0; }, [] { return 0.0; },
                                [] { return 0.0; }};
    stream.WithDeadband(0.1).WithScaleTranslation(0.8).WithScaleRotation(0.5).WithTelemetry(
        nt.name, Verbosity::HIGH);
    stream.Get();
    double before = nt.Published(key);

    nt.Dashboard(key, value);
    Tune(stream);
    CHECK(nt.Published(key) == Catch::Approx(before));
    Tune(stream);
    CHECK(nt.Published(key) == Catch::Approx(before));
  }
}

TEST_CASE_METHOD(SwerveDriveTestFixture,
                 "SwerveInputStreamTelemetryTest.CodeChangesArePublishedAndNotOverridden",
                 "[SwerveInputStreamTelemetryTest]") {
  StreamTables nt;
  SwerveInputStream<4> stream{*m_drive, [] { return 0.0; }, [] { return 0.0; },
                              [] { return 0.0; }};
  stream.WithScaleTranslation(0.8).WithTelemetry(nt.name, Verbosity::HIGH);
  stream.Get();

  // E.g. a slow mode binding changing the scale while driving.
  stream.WithScaleTranslation(0.4);
  Tune(stream);
  CHECK(nt.Published("translationScale") == Catch::Approx(0.4));
  CHECK(stream.GetTranslationAxisScale() == Catch::Approx(0.4));

  stream.WithScaleTranslation(0.8);
  Tune(stream);
  CHECK(nt.Published("translationScale") == Catch::Approx(0.8));
  CHECK(stream.GetTranslationAxisScale() == Catch::Approx(0.8));
}

TEST_CASE_METHOD(SwerveDriveTestFixture,
                 "SwerveInputStreamTelemetryTest.SupplierControlledFeaturesAreNotOverridden",
                 "[SwerveInputStreamTelemetryTest]") {
  StreamTables nt;
  bool allianceRelative = false;
  SwerveInputStream<4> stream{*m_drive, [] { return 0.0; }, [] { return 0.0; },
                              [] { return 0.0; }};
  stream.WithAllianceRelativeControl([&] { return allianceRelative; })
      .WithTelemetry(nt.name, Verbosity::HIGH);
  stream.Get();

  allianceRelative = true;
  Tune(stream);
  CHECK(nt.PublishedBoolean("allianceRelative"));

  // The stream still follows the supplier: the telemetry did not replace it with a fixed value.
  allianceRelative = false;
  CHECK_FALSE(stream.IsAllianceRelativeEnabled());
  Tune(stream);
  CHECK_FALSE(nt.PublishedBoolean("allianceRelative"));
}

TEST_CASE_METHOD(SwerveDriveTestFixture,
                 "SwerveInputStreamTelemetryTest.WithTelemetryReplacesTheTelemetry",
                 "[SwerveInputStreamTelemetryTest]") {
  StreamTables first;
  StreamTables second;
  SwerveInputStream<4> stream{*m_drive, [] { return 0.0; }, [] { return 0.0; },
                              [] { return 0.0; }};
  stream.WithTelemetry(first.name, Verbosity::HIGH);
  stream.Get();
  CHECK(first.data->GetTopic("mode").Exists());

  stream.WithTelemetry(second.name, Verbosity::HIGH);
  stream.Get();
  INFO("the first telemetry is closed");
  CHECK_FALSE(first.data->GetTopic("mode").Exists());
  CHECK_FALSE(first.tuning->GetTopic("deadband").Exists());
  CHECK(second.Mode() == "ANGULAR_VELOCITY");

  stream.WithTelemetry(second.name, Verbosity::NONE);
  INFO("NONE turns telemetry off");
  CHECK_FALSE(stream.GetTelemetry().has_value());
  CHECK_FALSE(second.data->GetTopic("mode").Exists());
}

TEST_CASE_METHOD(SwerveDriveTestFixture,
                 "SwerveInputStreamTelemetryTest.CopiesClonesAndMoves",
                 "[SwerveInputStreamTelemetryTest]") {
  StreamTables nt;
  // Building a stream copies it from the temporary; the copy keeps the telemetry settings.
  SwerveInputStream<4> stream =
      SwerveInputStream<4>::Of(*m_drive, [] { return 0.0; }, [] { return 0.0; })
          .WithControllerRotationAxis([] { return 0.0; })
          .WithTelemetry(nt.name, Verbosity::HIGH);
  CHECK(stream.GetTelemetry().has_value());
  stream.Get();
  CHECK(nt.Mode() == "ANGULAR_VELOCITY");

  INFO("a clone has no telemetry");
  auto clone = stream.Clone();
  CHECK_FALSE(clone.GetTelemetry().has_value());

  INFO("a moved-to stream publishes, the moved-from one stops");
  SwerveInputStream<4> moved = std::move(stream);
  CHECK_FALSE(stream.GetTelemetry().has_value());
  CHECK_FALSE(nt.data->GetTopic("mode").Exists());
  moved.Get();
  CHECK(nt.Mode() == "ANGULAR_VELOCITY");
}

TEST_CASE_METHOD(SwerveDriveTestFixture,
                 "SwerveInputStreamTelemetryTest.DestroyingTheStreamStopsPublishing",
                 "[SwerveInputStreamTelemetryTest]") {
  StreamTables nt;
  auto& scheduler = wpi::cmd::CommandScheduler::GetInstance();
  wpi::cmd::Command* liveTuning = nullptr;
  {
    SwerveInputStream<4> stream{*m_drive, [] { return 0.0; }, [] { return 0.0; },
                                [] { return 0.0; }};
    stream.WithTelemetry(nt.name, Verbosity::HIGH);
    stream.Get();
    liveTuning = &stream.GetTelemetry()->get().GetLiveTuningCommand()->get();
    scheduler.Schedule(liveTuning);
    scheduler.Run();
    CHECK(scheduler.IsScheduled(liveTuning));
    CHECK(nt.data->GetTopic("deadband").Exists());
  }
  CHECK_FALSE(nt.data->GetTopic("mode").Exists());
  CHECK_FALSE(nt.data->GetTopic("deadband").Exists());
  CHECK_FALSE(nt.tuning->GetTopic("deadband").Exists());
  CHECK_FALSE(nt.tuning->GetTopic("robotRelative").Exists());
  INFO("Live Tuning is removed");
  CHECK_FALSE(nt.LiveTuningPublished());
}

TEST_CASE_METHOD(SwerveDriveTestFixture,
                 "SwerveInputStreamTelemetryTest.WithMaximumVelocityOverridesTheDriveConfig",
                 "[SwerveInputStreamTelemetryTest]") {
  SwerveInputStream<4> stream{*m_drive, [] { return 1.0; }, [] { return 0.0; },
                              [] { return 1.0; }};
  CHECK(stream.Get().vx.value() == Catch::Approx(kConfigMaxLinear));

  stream.WithMaximumLinearVelocity(2.0_mps)
      .WithMaximumAngularVelocity(wpi::units::radians_per_second_t{1.0});
  auto speeds = stream.Get();
  CHECK(speeds.vx.value() == Catch::Approx(2.0));
  CHECK(speeds.omega.value() == Catch::Approx(1.0));
}

}  // namespace yams::test
