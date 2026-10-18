// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

import com.revrobotics.spark.SparkMax;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.preferences.Preferences;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.Arm;
import yams.commands2.mechanisms.DoubleJointedArm;
import yams.commands2.mechanisms.Elevator;
import yams.commands2.mechanisms.FlyWheel;
import yams.commands2.mechanisms.Pivot;
import yams.commands2.telemetry.SmartMotorControllerCommandRegistry;
import yams.core.exceptions.ElevatorConfigurationException;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.mechanisms.config.ElevatorConfig;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.mechanisms.config.MechanismPositionConfig;
import yams.core.mechanisms.config.MechanismPositionConfig.Plane;
import yams.core.mechanisms.config.PivotConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;

/**
 * Verifies each mechanism's {@code getPose3d()} against poses worked out by hand: where it is mounted,
 * turned or moved by its motor's position, in each movement plane, on a turned mount, and on another
 * mechanism.
 */
public class MechanismPoseTest {
  private static final double TOLERANCE = 1e-6;
  private static int offset = 0;
  private final List<SmartMotorController> motors = new ArrayList<>();

  /** Where a point a meter along the part's +X (or +Y) axis is, for a part at the pose. */
  private static Translation3d pointAlong(Pose3d pose, Translation3d local) {
    return pose.transformBy(new Transform3d(local, Rotation3d.ZERO)).getTranslation();
  }

  private static void assertNear(Translation3d expected, Translation3d actual, String what) {
    assertEquals(expected.getX(), actual.getX(), TOLERANCE, what + " x");
    assertEquals(expected.getY(), actual.getY(), TOLERANCE, what + " y");
    assertEquals(expected.getZ(), actual.getZ(), TOLERANCE, what + " z");
  }

  private static final Translation3d ALONG_X = new Translation3d(1, 0, 0);
  private static final Translation3d ALONG_Y = new Translation3d(0, 1, 0);

  // ---- The pose math in MechanismPositionConfig ----------------------------------------------

  @Test
  void anArmInTheXzPlaneTurnsUpTowardZ() {
    MechanismPositionConfig mount = new MechanismPositionConfig().withRelativePosition(new Translation3d(0.2, 0, 0.5));
    // Level, the arm points forward from its pivot.
    assertNear(new Translation3d(0.2, 0, 0.5), mount.getRotatedPose(Degrees.of(0)).getTranslation(), "pivot");
    assertNear(new Translation3d(1.2, 0, 0.5), pointAlong(mount.getRotatedPose(Degrees.of(0)), ALONG_X), "level arm's end");
    // At 90 degrees it points straight up, and at -30 degrees forward and down.
    assertNear(new Translation3d(0.2, 0, 1.5), pointAlong(mount.getRotatedPose(Degrees.of(90)), ALONG_X), "upright arm's end");
    assertNear(new Translation3d(0.2 + Math.cos(Math.toRadians(30)), 0, 0.5 - 0.5), pointAlong(mount.getRotatedPose(Degrees.of(-30)), ALONG_X),
               "arm's end at -30 degrees");
  }

  @Test
  void anArmInTheYzPlaneTurnsFromYUpTowardZ() {
    MechanismPositionConfig mount = new MechanismPositionConfig().withMovementPlane(Plane.YZ);
    assertNear(new Translation3d(0, 1, 0), pointAlong(mount.getRotatedPose(Degrees.of(0)), ALONG_Y), "level arm's end");
    assertNear(new Translation3d(0, 0, 1), pointAlong(mount.getRotatedPose(Degrees.of(90)), ALONG_Y), "upright arm's end");
  }

  @Test
  void aTurretInTheXyPlaneTurnsFromXTowardY() {
    MechanismPositionConfig mount = new MechanismPositionConfig().withMovementPlane(Plane.XY).withRelativePosition(new Translation3d(0, 0, 0.4));
    assertNear(new Translation3d(0, 1, 0.4), pointAlong(mount.getRotatedPose(Degrees.of(90)), ALONG_X), "turret turned 90 degrees");
    assertNear(new Translation3d(-1, 0, 0.4), pointAlong(mount.getRotatedPose(Degrees.of(180)), ALONG_X), "turret turned around");
  }

  @Test
  void anElevatorSlidesAlongItsAngle() {
    MechanismPositionConfig mount = new MechanismPositionConfig().withRelativePosition(new Translation3d(0, 0, 0.1));
    Pose3d upright = mount.getTranslatedPose(Meters.of(0.7), Degrees.of(90));
    assertNear(new Translation3d(0, 0, 0.8), upright.getTranslation(), "upright carriage");
    assertEquals(Rotation3d.ZERO, upright.getRotation(), "a carriage does not turn");
    assertNear(new Translation3d(0.7 * 0.5, 0, 0.1 + 0.7 * Math.sin(Math.toRadians(60))),
               mount.getTranslatedPose(Meters.of(0.7), Degrees.of(60)).getTranslation(), "carriage on an elevator leaning 30 degrees forward");
  }

  @Test
  void aTurnedMountTurnsTheMechanism() {
    // An arm on the robot's left side, turned to face left.
    MechanismPositionConfig mount = new MechanismPositionConfig().withRelativePosition(new Transform3d(new Translation3d(0, 0.3, 0.5), new Rotation3d(0, 0, Math.PI / 2)));
    assertNear(new Translation3d(0, 1.3, 0.5), pointAlong(mount.getRotatedPose(Degrees.of(0)), ALONG_X), "level arm's end, facing left");
    assertNear(new Translation3d(0, 0.3, 1.5), pointAlong(mount.getRotatedPose(Degrees.of(90)), ALONG_X), "upright arm's end");
  }

  @Test
  void aMechanismOnAnotherMovesWithIt() {
    MechanismPositionConfig elevator = new MechanismPositionConfig();
    double[] height = {0.5};
    MechanismPositionConfig arm = new MechanismPositionConfig()
        .withParent(() -> elevator.getTranslatedPose(Meters.of(height[0]), Degrees.of(90)))
        .withRelativePosition(new Translation3d(0.1, 0, 0.2));
    assertNear(new Translation3d(0.1, 0, 0.7), arm.getMountPose().getTranslation(), "arm's pivot on the carriage");
    height[0] = 1.0;
    assertNear(new Translation3d(0.1, 0, 1.2), arm.getMountPose().getTranslation(), "arm's pivot with the carriage raised");
    assertNear(new Translation3d(0.1, 0, 2.2), pointAlong(arm.getRotatedPose(Degrees.of(90)), ALONG_X), "upright arm's end on the raised carriage");
  }

  // ---- Each mechanism reads its motor ----------------------------------------------------------

  private SmartMotorControllerConfig config(String name) {
    SmartMotorControllerTestSubsystem subsys = new SmartMotorControllerTestSubsystem();
    subsys.setName(name + (offset + 1));
    return config(name, subsys);
  }

  private SmartMotorControllerConfig config(String name, SmartMotorControllerTestSubsystem subsys) {
    offset++;
    return (SmartMotorControllerConfig) new SmartMotorControllerConfig()
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(10)))
        .withZeroPower(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withControlMode(ControlMode.OPEN_LOOP)
        .withMomentOfInertia(Meters.of(0.5), Kilograms.of(2))
        .withSubsystem(subsys)
        .withTelemetry(name + offset, TelemetryVerbosity.LOW);
  }

  private SmartMotorController spark(SmartMotorControllerConfig cfg) {
    SmartMotorController motor = new SparkWrapper(DeviceCreator.createSparkMax(), DCMotor.getNEO(1), cfg);
    motors.add(motor);
    return motor;
  }

  @Test
  void anArmsPoseFollowsItsAngle() {
    SmartMotorController motor = spark((SmartMotorControllerConfig) config("PoseArm").withStartingPosition(Degrees.of(90)));
    Arm arm = new Arm(new ArmConfig().withLength(Meters.of(0.5)).withHardLimits(Degrees.of(-100), Degrees.of(100))
                                     .withMechanismPositionConfig(new MechanismPositionConfig().withRelativePosition(new Translation3d(0.2, 0, 0.5))), motor);
    assertNear(new Translation3d(0.2, 0, 1.5), pointAlong(arm.getPose3d(), ALONG_X), "upright arm's end");
  }

  @Test
  void aPivotsPoseFollowsItsAngle() {
    SmartMotorController motor = spark((SmartMotorControllerConfig) config("PosePivot").withStartingPosition(Degrees.of(90)));
    Pivot pivot = new Pivot(new PivotConfig().withHardLimits(Degrees.of(-180), Degrees.of(180))
                                             .withMechanismPositionConfig(new MechanismPositionConfig().withMovementPlane(Plane.XY)), motor);
    assertNear(new Translation3d(0, 1, 0), pointAlong(pivot.getPose3d(), ALONG_X), "turret turned 90 degrees");
  }

  @Test
  void aPivotTurnsInTheXyPlaneByDefault() {
    SmartMotorController motor = spark((SmartMotorControllerConfig) config("PoseDefaultPivot").withStartingPosition(Degrees.of(90)));
    Pivot pivot = new Pivot(new PivotConfig().withHardLimits(Degrees.of(-180), Degrees.of(180)), motor);
    assertNear(new Translation3d(0, 1, 0), pointAlong(pivot.getPose3d(), ALONG_X), "turret turned 90 degrees");
  }

  @Test
  void anElevatorsPoseFollowsItsHeight() {
    SmartMotorController motor = spark((SmartMotorControllerConfig) config("PoseElevator").withMechanismCircumference(Meters.of(0.1)).withStartingPosition(Meters.of(0.7)));
    Elevator elevator = new Elevator(new ElevatorConfig().withHardLimits(Meters.of(0), Meters.of(2)).withCarriageWeight(Kilograms.of(5))
                                                         .withMechanismPositionConfig(new MechanismPositionConfig().withRelativePosition(new Translation3d(0, 0, 0.1))), motor);
    assertNear(new Translation3d(0, 0, 0.8), elevator.getPose3d().getTranslation(), "carriage");
  }

  /** An elevator mounted 0.1 m up, with its carriage at 0.9 m, and the stages set. */
  private Elevator elevatorAtNinetyCentimeters(String name, ElevatorConfig config) {
    SmartMotorController motor = spark((SmartMotorControllerConfig) config(name).withMechanismCircumference(Meters.of(0.1)).withStartingPosition(Meters.of(0.9)));
    return new Elevator(config.withHardLimits(Meters.of(0), Meters.of(2)).withCarriageWeight(Kilograms.of(5))
                              .withMechanismPositionConfig(new MechanismPositionConfig().withRelativePosition(new Translation3d(0, 0, 0.1))), motor);
  }

  @Test
  void aCascadeElevatorsStagesEachMoveTheirShareOfItsHeight() {
    Elevator elevator = elevatorAtNinetyCentimeters("PoseCascade", new ElevatorConfig().withStages(3, List.of(Meters.of(0.4), Meters.of(0.4), Meters.of(0.4))));
    // Equal travel: a third, two thirds and all of the carriage's 0.9 m.
    List<Pose3d> stages = elevator.getCascadePose3d();
    assertEquals(3, stages.size());
    assertNear(new Translation3d(0, 0, 0.4), stages.get(0).getTranslation(), "first stage");
    assertNear(new Translation3d(0, 0, 0.7), stages.get(1).getTranslation(), "second stage");
    assertNear(new Translation3d(0, 0, 1.0), stages.get(2).getTranslation(), "carriage");
    assertEquals(elevator.getPose3d(), stages.get(2), "the last stage is the carriage");
  }

  @Test
  void aCascadeElevatorsStagesMoveInProportionToTheirTravel() {
    // The first stage travels half of the 1.2 m total, so it moves half of the carriage's height,
    // 0.45 m, and the others a quarter each.
    Elevator elevator = elevatorAtNinetyCentimeters("PoseUnevenCascade", new ElevatorConfig().withStages(3, List.of(Meters.of(0.6), Meters.of(0.3), Meters.of(0.3))));
    List<Pose3d> stages = elevator.getCascadePose3d();
    assertNear(new Translation3d(0, 0, 0.55), stages.get(0).getTranslation(), "first stage");
    assertNear(new Translation3d(0, 0, 0.775), stages.get(1).getTranslation(), "second stage");
    assertNear(new Translation3d(0, 0, 1.0), stages.get(2).getTranslation(), "carriage");
  }

  @Test
  void aContinuousElevatorsCarriageMovesFirst() {
    // Each stage travels 0.5 m. The carriage takes the first 0.5 m of its 0.9 m, the second stage the
    // other 0.4 m, and the first stage has not moved.
    Elevator elevator = elevatorAtNinetyCentimeters("PoseContinuous", new ElevatorConfig().withStages(3, List.of(Meters.of(0.5), Meters.of(0.5), Meters.of(0.5))));
    List<Pose3d> stages = elevator.getContinuousPose3d();
    assertEquals(3, stages.size());
    assertNear(new Translation3d(0, 0, 0.1), stages.get(0).getTranslation(), "first stage");
    assertNear(new Translation3d(0, 0, 0.5), stages.get(1).getTranslation(), "second stage");
    assertNear(new Translation3d(0, 0, 1.0), stages.get(2).getTranslation(), "carriage");
    assertEquals(elevator.getPose3d(), stages.get(2), "the last stage is the carriage");
  }

  @Test
  void anElevatorWithOnlyACarriageHasNoStagePoses() {
    // 1 moving stage, the carriage, is a valid elevator, but has no stages to cascade or move in turn.
    Elevator elevator = elevatorAtNinetyCentimeters("PoseSingleStage", new ElevatorConfig().withStages(1, List.of(Meters.of(2))));
    assertThrows(ElevatorConfigurationException.class, elevator::getCascadePose3d);
    assertThrows(ElevatorConfigurationException.class, elevator::getContinuousPose3d);
    assertNear(new Translation3d(0, 0, 1.0), elevator.getPose3d().getTranslation(), "carriage");
  }

  @Test
  void elevatorStagesNeedAPositiveHeightForEachStage() {
    assertThrows(IllegalArgumentException.class, () -> new ElevatorConfig().withStages(0, List.of()));
    assertThrows(IllegalArgumentException.class, () -> new ElevatorConfig().withStages(3, List.of(Meters.of(0.5), Meters.of(0.5))));
    assertThrows(IllegalArgumentException.class, () -> new ElevatorConfig().withStages(2, List.of(Meters.of(0.5), Meters.of(0))));
    Elevator elevator = elevatorAtNinetyCentimeters("PoseNoStages", new ElevatorConfig());
    assertThrows(ElevatorConfigurationException.class, elevator::getCascadePose3d);
    assertThrows(ElevatorConfigurationException.class, elevator::getContinuousPose3d);
  }

  @Test
  void aFlywheelsPoseFollowsItsPosition() {
    SmartMotorController motor = spark((SmartMotorControllerConfig) config("PoseFlyWheel").withStartingPosition(Degrees.of(90)));
    FlyWheel flywheel = new FlyWheel(new FlyWheelConfig().withDiameter(Meters.of(0.1)), motor);
    assertNear(new Translation3d(0, 0, 1), pointAlong(flywheel.getPose3d(), ALONG_X), "a point on the wheel turned 90 degrees");
  }

  @Test
  void aDoubleJointedArmsUpperArmIsAtTheEndOfItsLowerArm() {
    // A double jointed arm's motors share a subsystem.
    SmartMotorControllerTestSubsystem subsys = new SmartMotorControllerTestSubsystem();
    subsys.setName("PoseDoubleJointedArm" + offset);
    SmartMotorController lower = spark((SmartMotorControllerConfig) config("PoseLower", subsys).withStartingPosition(Degrees.of(90)));
    SmartMotorController upper = spark((SmartMotorControllerConfig) config("PoseUpper", subsys).withStartingPosition(Degrees.of(0)));
    DoubleJointedArm arm = new DoubleJointedArm(
        new ArmConfig().withLength(Meters.of(0.6)).withHardLimits(Degrees.of(-180), Degrees.of(180))
                       .withMechanismPositionConfig(new MechanismPositionConfig().withRelativePosition(new Translation3d(0, 0, 0.3))), lower,
        new ArmConfig().withLength(Meters.of(0.4)).withHardLimits(Degrees.of(-180), Degrees.of(180)), upper);
    // The lower arm points up from its pivot, and the upper arm forward from the elbow.
    assertNear(new Translation3d(0, 0, 0.9), pointAlong(arm.getPose3d(), new Translation3d(0.6, 0, 0)), "elbow");
    assertNear(new Translation3d(0, 0, 0.9), arm.getUpperPose3d().getTranslation(), "upper arm's pivot");
    assertNear(new Translation3d(0.4, 0, 0.9), pointAlong(arm.getUpperPose3d(), new Translation3d(0.4, 0, 0)), "upper arm's end");
  }

  @BeforeEach
  void beforeTest() {
    MockHardwareExtension.beforeAll();
  }

  @AfterEach
  void afterTest() {
    for (SmartMotorController motor : motors) {
      var subsys = ((SmartMotorControllerConfig) motor.getConfig()).getSubsystem();
      SmartMotorControllerCommandRegistry.removeCommands(subsys);
      CommandScheduler.getInstance().unregisterSubsystem(subsys);
      motor.close();
      DeviceCreator.silence(motor);
      ((SparkMax) motor.getMotorController()).close();
    }
    motors.clear();
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }
}
