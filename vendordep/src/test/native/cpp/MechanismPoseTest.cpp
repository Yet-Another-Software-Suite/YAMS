// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Mirrors Java MechanismPoseTest: each mechanism's GetPose3d() against poses worked out by hand: where
// it is mounted, turned or moved by its motor's position, in each movement plane, on a turned mount,
// and on another mechanism.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>
#include <wpi/math/geometry/Pose3d.hpp>
#include <wpi/math/geometry/Rotation3d.hpp>
#include <wpi/math/geometry/Transform3d.hpp>
#include <wpi/math/geometry/Translation3d.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/mass.hpp>

#include "helpers/MockHardware.h"
#include "helpers/MotorControllerFactory.h"
#include "helpers/SchedulerHelper.h"
#include "yams/mechanisms/config/ArmConfig.hpp"
#include "yams/mechanisms/config/ElevatorConfig.hpp"
#include "yams/mechanisms/config/FlyWheelConfig.hpp"
#include "yams/mechanisms/config/MechanismPositionConfig.hpp"
#include "yams/mechanisms/config/PivotConfig.hpp"
#include "yams/mechanisms/positional/Arm.hpp"
#include "yams/mechanisms/positional/Elevator.hpp"
#include "yams/mechanisms/positional/Pivot.hpp"
#include "yams/mechanisms/velocity/FlyWheel.hpp"

namespace yams::test {

using namespace motorcontrollers;
using namespace mechanisms;
using namespace mechanisms::config;
using Plane = MechanismPositionConfig::Plane;
using wpi::math::Pose3d;
using wpi::math::Rotation3d;
using wpi::math::Transform3d;
using wpi::math::Translation3d;

namespace {

constexpr double kTolerance = 1e-6;
const Translation3d kAlongX{1_m, 0_m, 0_m};
const Translation3d kAlongY{0_m, 1_m, 0_m};

// Where a point along the part's axis is, for a part at the pose.
Translation3d PointAlong(const Pose3d& pose, const Translation3d& local) {
  return pose.TransformBy(Transform3d{local, Rotation3d{}}).Translation();
}

void CheckNear(const Translation3d& expected, const Translation3d& actual, const std::string& what) {
  INFO(what << ": expected (" << expected.X().value() << ", " << expected.Y().value() << ", "
            << expected.Z().value() << "), got (" << actual.X().value() << ", "
            << actual.Y().value() << ", " << actual.Z().value() << ")");
  CHECK(actual.X().value() == Catch::Approx(expected.X().value()).margin(kTolerance));
  CHECK(actual.Y().value() == Catch::Approx(expected.Y().value()).margin(kTolerance));
  CHECK(actual.Z().value() == Catch::Approx(expected.Z().value()).margin(kTolerance));
}

Translation3d Meters(double x, double y, double z) {
  return Translation3d{wpi::units::meter_t{x}, wpi::units::meter_t{y}, wpi::units::meter_t{z}};
}

struct MechanismPoseFixture {
  MechanismPoseFixture() {
    InitializeHardware();
    SchedulerHelper::Enable();
    SchedulerHelper::CancelAll();
  }
  ~MechanismPoseFixture() {
    TeardownHardware();
    SchedulerHelper::CancelAll();
  }
};

HardwareBundle Spark(const std::string& name, SmartMotorControllerConfig cfg) {
  return MakeBundle({HardwareType::SparkMax, ProfileType::None, name}, cfg);
}

SmartMotorControllerConfig BaseConfig(const std::string& name) {
  auto cfg = MakeBaseConfig(ProfileType::None, 0.0,
                            gearing::MechanismGearing{gearing::GearBox::FromReductionStages({10.0})},
                            nullptr, name);
  cfg.WithMOI(0.5_m, 2_kg);
  return cfg;
}

}  // namespace

// ---- The pose math in MechanismPositionConfig ------------------------------------------------

TEST_CASE("MechanismPose.ArmInTheXzPlaneTurnsUpTowardZ", "[MechanismPose]") {
  MechanismPositionConfig mount;
  mount.WithRelativePosition(Meters(0.2, 0, 0.5));
  CheckNear(Meters(0.2, 0, 0.5), mount.GetRotatedPose(0_deg).Translation(), "pivot");
  CheckNear(Meters(1.2, 0, 0.5), PointAlong(mount.GetRotatedPose(0_deg), kAlongX), "level arm's end");
  CheckNear(Meters(0.2, 0, 1.5), PointAlong(mount.GetRotatedPose(90_deg), kAlongX), "upright arm's end");
  CheckNear(Meters(0.2 + std::cos(std::numbers::pi / 6), 0, 0), PointAlong(mount.GetRotatedPose(-30_deg), kAlongX),
            "arm's end at -30 degrees");
}

TEST_CASE("MechanismPose.ArmInTheYzPlaneTurnsFromYUpTowardZ", "[MechanismPose]") {
  MechanismPositionConfig mount;
  mount.WithMovementPlane(Plane::YZ);
  CheckNear(Meters(0, 1, 0), PointAlong(mount.GetRotatedPose(0_deg), kAlongY), "level arm's end");
  CheckNear(Meters(0, 0, 1), PointAlong(mount.GetRotatedPose(90_deg), kAlongY), "upright arm's end");
}

TEST_CASE("MechanismPose.TurretInTheXyPlaneTurnsFromXTowardY", "[MechanismPose]") {
  MechanismPositionConfig mount;
  mount.WithMovementPlane(Plane::XY).WithRelativePosition(Meters(0, 0, 0.4));
  CheckNear(Meters(0, 1, 0.4), PointAlong(mount.GetRotatedPose(90_deg), kAlongX), "turret turned 90 degrees");
  CheckNear(Meters(-1, 0, 0.4), PointAlong(mount.GetRotatedPose(180_deg), kAlongX), "turret turned around");
}

TEST_CASE("MechanismPose.ElevatorSlidesAlongItsAngle", "[MechanismPose]") {
  MechanismPositionConfig mount;
  mount.WithRelativePosition(Meters(0, 0, 0.1));
  Pose3d upright = mount.GetTranslatedPose(0.7_m, 90_deg);
  CheckNear(Meters(0, 0, 0.8), upright.Translation(), "upright carriage");
  CHECK(upright.Rotation() == Rotation3d{});
  CheckNear(Meters(0.35, 0, 0.1 + 0.7 * std::sin(std::numbers::pi / 3)),
            mount.GetTranslatedPose(0.7_m, 60_deg).Translation(),
            "carriage on an elevator leaning 30 degrees forward");
}

TEST_CASE("MechanismPose.TurnedMountTurnsTheMechanism", "[MechanismPose]") {
  // An arm on the robot's left side, turned to face left.
  MechanismPositionConfig mount;
  mount.WithRelativePosition(Transform3d{Meters(0, 0.3, 0.5), Rotation3d{0_rad, 0_rad, 90_deg}});
  CheckNear(Meters(0, 1.3, 0.5), PointAlong(mount.GetRotatedPose(0_deg), kAlongX), "level arm's end, facing left");
  CheckNear(Meters(0, 0.3, 1.5), PointAlong(mount.GetRotatedPose(90_deg), kAlongX), "upright arm's end");
}

TEST_CASE("MechanismPose.MechanismOnAnotherMovesWithIt", "[MechanismPose]") {
  MechanismPositionConfig elevator;
  double height = 0.5;
  MechanismPositionConfig arm;
  arm.WithParent([&] { return elevator.GetTranslatedPose(wpi::units::meter_t{height}, 90_deg); })
      .WithRelativePosition(Meters(0.1, 0, 0.2));
  CheckNear(Meters(0.1, 0, 0.7), arm.GetMountPose().Translation(), "arm's pivot on the carriage");
  height = 1.0;
  CheckNear(Meters(0.1, 0, 1.2), arm.GetMountPose().Translation(), "arm's pivot with the carriage raised");
  CheckNear(Meters(0.1, 0, 2.2), PointAlong(arm.GetRotatedPose(90_deg), kAlongX),
            "upright arm's end on the raised carriage");
}

// ---- Each mechanism reads its motor -------------------------------------------------------------

TEST_CASE_METHOD(MechanismPoseFixture, "MechanismPose.ArmFollowsItsAngle", "[MechanismPose]") {
  auto cfg = BaseConfig("PoseArm");
  cfg.WithStartingPosition(90_deg);
  auto bundle = Spark("PoseArm", cfg);
  bundle.smc->SetupSimulation();
  MechanismPositionConfig position;
  position.WithRelativePosition(Meters(0.2, 0, 0.5));
  auto* armCfg = new ArmConfig;
  armCfg->WithArmLength(0.5_m).WithMinAngle(-100_deg).WithMaxAngle(100_deg).WithMechanismPositionConfig(position);
  auto* arm = new positional::Arm(armCfg, bundle.smc);
  CheckNear(Meters(0.2, 0, 1.5), PointAlong(arm->GetPose3d(), kAlongX), "upright arm's end");
  CloseBundle(bundle);
  delete arm;
}

TEST_CASE_METHOD(MechanismPoseFixture, "MechanismPose.PivotFollowsItsAngle", "[MechanismPose]") {
  auto cfg = BaseConfig("PosePivot");
  cfg.WithStartingPosition(90_deg);
  auto bundle = Spark("PosePivot", cfg);
  bundle.smc->SetupSimulation();
  MechanismPositionConfig position;
  position.WithMovementPlane(Plane::XY);
  auto* pivotCfg = new PivotConfig;
  pivotCfg->WithMinAngle(-180_deg).WithMaxAngle(180_deg).WithMechanismPositionConfig(position);
  auto* pivot = new positional::Pivot(pivotCfg, bundle.smc);
  CheckNear(Meters(0, 1, 0), PointAlong(pivot->GetPose3d(), kAlongX), "turret turned 90 degrees");
  CloseBundle(bundle);
  delete pivot;
}

TEST_CASE_METHOD(MechanismPoseFixture, "MechanismPose.PivotTurnsInTheXyPlaneByDefault", "[MechanismPose]") {
  // As in Java, a pivot with no MechanismPositionConfig of its own turns like a turret.
  auto cfg = BaseConfig("PoseDefaultPivot");
  cfg.WithStartingPosition(90_deg);
  auto bundle = Spark("PoseDefaultPivot", cfg);
  bundle.smc->SetupSimulation();
  auto* pivotCfg = new PivotConfig;
  pivotCfg->WithMinAngle(-180_deg).WithMaxAngle(180_deg);
  CHECK(pivotCfg->GetMechanismPositionConfig().GetMovementPlane() == Plane::XY);
  auto* pivot = new positional::Pivot(pivotCfg, bundle.smc);
  CheckNear(Meters(0, 1, 0), PointAlong(pivot->GetPose3d(), kAlongX), "turret turned 90 degrees");
  CloseBundle(bundle);
  delete pivot;
}

TEST_CASE_METHOD(MechanismPoseFixture, "MechanismPose.ElevatorFollowsItsHeight", "[MechanismPose]") {
  auto cfg = BaseConfig("PoseElevator");
  cfg.WithMechanismCircumference(0.1_m).WithStartingPosition(0.7_m);
  auto bundle = Spark("PoseElevator", cfg);
  bundle.smc->SetupSimulation();
  MechanismPositionConfig position;
  position.WithRelativePosition(Meters(0, 0, 0.1));
  auto* elevatorCfg = new ElevatorConfig;
  elevatorCfg->WithMinimumHeight(0_m).WithMaximumHeight(2_m).WithCarriageMass(5_kg).WithMechanismPositionConfig(position);
  auto* elevator = new positional::Elevator(elevatorCfg, bundle.smc);
  CheckNear(Meters(0, 0, 0.8), elevator->GetPose3d().Translation(), "carriage");
  CloseBundle(bundle);
  delete elevator;
}

TEST_CASE_METHOD(MechanismPoseFixture, "MechanismPose.ElevatorStages", "[MechanismPose]") {
  auto* elevatorCfg = new ElevatorConfig;
  std::string name = "PoseStages";

  SECTION("cascade stages each move their share of its height") {
    elevatorCfg->WithStages(3, {0.4_m, 0.4_m, 0.4_m});
    name = "PoseCascade";
  }
  SECTION("uneven cascade stages move in proportion to their travel") {
    elevatorCfg->WithStages(3, {0.6_m, 0.3_m, 0.3_m});
    name = "PoseUnevenCascade";
  }
  SECTION("a continuous carriage moves first") {
    elevatorCfg->WithStages(3, {0.5_m, 0.5_m, 0.5_m});
    name = "PoseContinuous";
  }
  SECTION("an elevator with only a carriage has no stage poses") {
    elevatorCfg->WithStages(1, {2_m});
    name = "PoseSingleStage";
  }
  SECTION("no stages") {
    name = "PoseNoStages";
  }

  // An elevator mounted 0.1 m up, with its carriage at 0.9 m.
  auto cfg = BaseConfig(name);
  cfg.WithMechanismCircumference(0.1_m).WithStartingPosition(0.9_m);
  auto bundle = Spark(name, cfg);
  bundle.smc->SetupSimulation();
  MechanismPositionConfig position;
  position.WithRelativePosition(Meters(0, 0, 0.1));
  // The maximum height is the stage heights' total, or 2 m without stages.
  elevatorCfg->WithMinimumHeight(0_m)
      .WithMaximumHeight(elevatorCfg->GetStageHeightTotal().value_or(2_m))
      .WithCarriageMass(5_kg)
      .WithMechanismPositionConfig(position);
  auto* elevator = new positional::Elevator(elevatorCfg, bundle.smc);

  if (name == "PoseCascade") {
    // Equal travel: a third, two thirds and all of the carriage's 0.9 m.
    auto stages = elevator->GetCascadePose3d();
    REQUIRE(stages.size() == 3);
    CheckNear(Meters(0, 0, 0.4), stages[0].Translation(), "first stage");
    CheckNear(Meters(0, 0, 0.7), stages[1].Translation(), "second stage");
    CheckNear(Meters(0, 0, 1.0), stages[2].Translation(), "carriage");
    CHECK(stages[2] == elevator->GetPose3d());
  } else if (name == "PoseUnevenCascade") {
    // The first stage travels half of the 1.2 m total, so it moves half of the carriage's height.
    auto stages = elevator->GetCascadePose3d();
    CheckNear(Meters(0, 0, 0.55), stages[0].Translation(), "first stage");
    CheckNear(Meters(0, 0, 0.775), stages[1].Translation(), "second stage");
    CheckNear(Meters(0, 0, 1.0), stages[2].Translation(), "carriage");
  } else if (name == "PoseContinuous") {
    // The carriage takes the first 0.5 m of its 0.9 m, the second stage the other 0.4 m.
    auto stages = elevator->GetContinuousPose3d();
    REQUIRE(stages.size() == 3);
    CheckNear(Meters(0, 0, 0.1), stages[0].Translation(), "first stage");
    CheckNear(Meters(0, 0, 0.5), stages[1].Translation(), "second stage");
    CheckNear(Meters(0, 0, 1.0), stages[2].Translation(), "carriage");
    CHECK(stages[2] == elevator->GetPose3d());
  } else if (name == "PoseSingleStage") {
    // 1 moving stage, the carriage, is a valid elevator, but has no stages to cascade or move in turn.
    CHECK_THROWS(elevator->GetCascadePose3d());
    CHECK_THROWS(elevator->GetContinuousPose3d());
    CheckNear(Meters(0, 0, 1.0), elevator->GetPose3d().Translation(), "carriage");
  } else {
    CHECK_THROWS(elevator->GetCascadePose3d());
    CHECK_THROWS(elevator->GetContinuousPose3d());
  }

  CloseBundle(bundle);
  delete elevator;
}

TEST_CASE_METHOD(MechanismPoseFixture, "MechanismPose.ElevatorStageHeightsAddUpToItsMaximumHeight",
                 "[MechanismPose]") {
  ElevatorConfig stages;
  stages.WithStages(3, {0.6_m, 0.6_m, 0.5_m});
  CHECK(stages.GetStageHeightTotal()->value() == Catch::Approx(1.7).margin(1e-9));
  CHECK_FALSE(ElevatorConfig{}.GetStageHeightTotal().has_value());

  // Stages that add up to 1 m on a 2 m elevator cannot carry the carriage to the top.
  auto cfg = BaseConfig("PoseShortStages");
  cfg.WithMechanismCircumference(0.1_m).WithStartingPosition(0.9_m);
  auto bundle = Spark("PoseShortStages", cfg);
  bundle.smc->SetupSimulation();
  auto* elevatorCfg = new ElevatorConfig;
  elevatorCfg->WithStages(2, {0.5_m, 0.5_m}).WithMinimumHeight(0_m).WithMaximumHeight(2_m).WithCarriageMass(5_kg);
  CHECK_THROWS_WITH(positional::Elevator(elevatorCfg, bundle.smc),
                    Catch::Matchers::ContainsSubstring("stage heights add up to 1.000000 m, not the maximum height of 2.000000 m"));
  CloseBundle(bundle);
  delete elevatorCfg;
}

TEST_CASE_METHOD(MechanismPoseFixture, "MechanismPose.ElevatorHeightIsItsStagesWithoutLimits",
                 "[MechanismPose]") {
  // No minimum or maximum height: the stages give the elevator's height, from 0 to their 1.5 m total.
  auto* elevatorCfg = new ElevatorConfig;
  elevatorCfg->WithStages(3, {0.5_m, 0.5_m, 0.5_m}).WithCarriageMass(5_kg);
  CHECK(elevatorCfg->GetMaxHeight()->value() == Catch::Approx(1.5).margin(1e-9));
  CHECK(elevatorCfg->GetMinHeight()->value() == Catch::Approx(0.0).margin(1e-9));
  auto cfg = BaseConfig("PoseStagesOnly");
  cfg.WithMechanismCircumference(0.1_m).WithStartingPosition(0.9_m);
  auto bundle = Spark("PoseStagesOnly", cfg);
  bundle.smc->SetupSimulation();
  auto* elevator = new positional::Elevator(elevatorCfg, bundle.smc);
  auto stages = elevator->GetContinuousPose3d();
  REQUIRE(stages.size() == 3);
  CheckNear(Meters(0, 0, 0.9), stages[2].Translation(), "carriage");
  CloseBundle(bundle);
  delete elevator;
}

TEST_CASE("MechanismPose.ElevatorStagesNeedAPositiveHeightForEachStage", "[MechanismPose]") {
  ElevatorConfig config;
  CHECK_THROWS_AS(config.WithStages(0, {}), std::invalid_argument);
  CHECK_THROWS_AS(config.WithStages(3, {0.5_m, 0.5_m}), std::invalid_argument);
  CHECK_THROWS_AS(config.WithStages(2, {0.5_m, 0_m}), std::invalid_argument);
}

TEST_CASE_METHOD(MechanismPoseFixture, "MechanismPose.FlyWheelFollowsItsPosition", "[MechanismPose]") {
  auto cfg = BaseConfig("PoseFlyWheel");
  cfg.WithStartingPosition(90_deg);
  auto bundle = Spark("PoseFlyWheel", cfg);
  bundle.smc->SetupSimulation();
  auto* flyWheelCfg = new FlyWheelConfig;
  flyWheelCfg->WithRollerDiameter(0.1_m);
  auto* flyWheel = new velocity::FlyWheel(flyWheelCfg, bundle.smc);
  CheckNear(Meters(0, 0, 1), PointAlong(flyWheel->GetPose3d(), kAlongX), "a point on the wheel turned 90 degrees");
  CloseBundle(bundle);
  delete flyWheel;
}

}  // namespace yams::test
