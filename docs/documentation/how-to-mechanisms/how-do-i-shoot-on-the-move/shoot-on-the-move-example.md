---
description: >-
  A complete robot that aims, sets its shooter speed and feeds only when the
  shot is ready, while it drives.
icon: code
---

# Shoot on the move: worked example

**Goal:** turn the robots from the Swerve Drive and Shooter tutorials into one that aims, sets its shooter speed and feeds only when the shot is ready, while it drives.

Read [How do I shoot on the move?](./) first for the sources this example is built from, and [Shooting on the move](../../understanding/shooters.md#shooting-on-the-move) for the idea. This page is only the code, step by step.

**Before you start:** have the complete files from the [Swerve Drive tutorial's Driving step](../../tutorials/swerve-drive/driving.md#the-complete-file) and the [Shooter tutorial's Commands and buttons step](../../tutorials/shooter-flywheels/commands.md#the-complete-file) in one project, Commands v2 or Commands v3, and a feeder built as in [How do I set up an intake roller?](../how-do-i-set-up-an-intake-roller.md), with its telemetry named `Feeder` and `FeederMotor` and its `intake()` command renamed `feed()`. The steps below show only what to add to them.

{% hint style="info" %}
The code is shown for Java with Commands v2 and Commands v3. The C++ port has the same pieces, `SwerveInputStream::WithAim`, `WithAimLookahead` and `IsAimLocked`, and `FlyWheel::Run` with a function, so the same approach works there: start from the C++ tabs of the [Swerve Drive](../../tutorials/swerve-drive/) and [Shooter](../../tutorials/shooter-flywheels/) tutorials. Every file compiles against YAMS, but none of it has been run on a robot: the numbers in the tables are placeholders for your own measurements.
{% endhint %}

{% stepper %}
{% step %}
### Add to the swerve drive

Every calculation starts from where the robot is and how fast it is moving on the field. Add two methods to the tutorial's drive, each calling the `SwerveDrive` method of the same name:

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="subsystems/SwerveSubsystem.java (fragment)" %}
```java
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/** Where the robot is on the field. */
public Pose2d getPose() {
  return drive.getPose();
}

/** How fast the robot is moving on the field. */
public ChassisVelocities getFieldRelativeSpeed() {
  return drive.getFieldRelativeSpeed();
}
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
The same two methods go in `mechanisms/SwerveMechanism.java`.
{% endtab %}
{% endtabs %}

The tutorial's drive motors keep their `kV` of 0.85, and its rotation controller, `withRotationController(new PIDController(1, 0, 0))`, is what turns the robot to aim: tune it once the robot drives well.

**Check it:** the code builds.
{% endstep %}

{% step %}
### Add to the shooter

The tutorial's shooter, with its 1.5:1 gearing, `kV` of 0.19 and `kP` of 0.2, tops out near 3800 RPM, above every speed in the tables below. Add two methods for a shot whose speed changes every loop:

* **`track(Supplier)`** sets the flywheel's speed from a supplier, which `FlyWheel.run(Supplier)` reads every loop. This is how the shot's speed reaches the flywheel.
* **`atSpeed(Supplier)`** is true while the flywheel is within 100 RPM of that same changing speed. `FlyWheel.near(...)` takes a fixed speed, so it cannot follow a speed that changes. The 100 RPM and the 0.1 second debounce on the `ready` trigger in [Aim, set the shooter speed, and shoot safely](shoot-on-the-move-example.md#aim-set-the-shooter-speed-and-shoot-safely) are the same as in [How do I shoot with a shooter?](../how-to-shoot-with-a-shooter.md)

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="subsystems/ShooterSubsystem.java (fragment)" %}
```java
import static org.wpilib.units.Units.RPM;

import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.button.Trigger;
import org.wpilib.units.measure.AngularVelocity;

/** Follow a speed that changes every loop. This command does not end on its own. */
public Command track(Supplier<AngularVelocity> speed) {
  return shooter.run(speed);
}

/** True while the flywheel is within 100 RPM of a speed that changes every loop. */
public Trigger atSpeed(Supplier<AngularVelocity> speed) {
  return new Trigger(() -> shooter.isNear(speed.get(), RPM.of(100)));
}
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="mechanisms/ShooterMechanism.java (fragment)" %}
```java
import static org.wpilib.units.Units.RPM;

import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import org.wpilib.units.measure.AngularVelocity;

/** Follow a speed that changes every loop. This command does not end on its own. */
public Command track(Supplier<AngularVelocity> speed) {
  return shooter.run(speed);
}

/** True while the flywheel is within 100 RPM of a speed that changes every loop. */
public Trigger atSpeed(Supplier<AngularVelocity> speed) {
  return new Trigger(() -> shooter.isNear(speed.get(), RPM.of(100)));
}
```
{% endcode %}
{% endtab %}
{% endtabs %}

**Check it:** the code builds.
{% endstep %}

{% step %}
### Measure the lookup tables

**Read first:** [Static Shooting via Look-Up Table](https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/static-shooting.html), and Eeshwar's [part 2](https://blog.eeshwark.com/robotblog/shooting-on-the-fly-pt2), which describes how to measure the table with a tape measure and a phone.

The tables hold what you measured standing still: the flywheel speed that scores from each distance, and how long that shot is in the air. WPILib's [`InterpolatingDoubleTreeMap`](https://github.wpilib.org/allwpilib/docs/development/java/org/wpilib/math/interpolation/InterpolatingDoubleTreeMap.html) fills in the distances in between. `inRange` says whether the robot is at a distance you measured, which the `ready` trigger uses. Create `ShotCalculator.java` with the start of the class, as in [The complete ShotCalculator file](shoot-on-the-move-example.md#the-complete-shotcalculator-file) below without its three strategy methods.

**For your robot:** set `TARGET`, fill both tables with your own measurements, change the two distances in `inRange` to match, and flip the target for the red alliance. Keep every speed well below the shooter's top speed.

**Check it:** the code builds.
{% endstep %}

{% step %}
### Pick a firing strategy

Each tab is one method to add to `ShotCalculator`; the complete file below has all three. They are the three strategies of the WPILib fire control pages, written against the tables above. The two for a moving robot are **time of flight** methods: they get the lead from the time of flight you measured.

{% tabs %}
{% tab title="Standing still" %}
**Read first:** [Static Shooting via Look-Up Table](https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/static-shooting.html).

Aim at the real target, and read the flywheel speed for the distance. Only right while the robot is not moving.

{% code title="ShotCalculator.java (fragment)" %}
```java
/** Aim at the real target. Only right while the robot is standing still. */
public static Shot standing(Pose2d robotPose) {
  return shotAt(robotPose, TARGET);
}
```
{% endcode %}
{% endtab %}

{% tab title="Time of flight recursion" %}
**Read first:** [Dynamic Shooting via Time-of-Flight Recursion](https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/dynamic-shooting.html).

A time of flight method. It moves the target against the robot's velocity by the time of flight, which changes the distance and so the time of flight, and repeats until the time stops changing. This is the virtual target that most teams use.

{% code title="ShotCalculator.java (fragment)" %}
```java
/** Lead the target by the time of flight, and repeat until the time settles. */
public static Shot recursive(Pose2d robotPose, ChassisVelocities fieldVelocity) {
  Translation2d virtualTarget = TARGET;
  // Each pass uses the distance from the pass before, so the answer settles after a few.
  for (int i = 0; i < 3; i++) {
    double distance = robotPose.getTranslation().getDistance(virtualTarget);
    double timeOfFlight = TIME_OF_FLIGHT.get(distance);
    // The shot drifts by the robot's velocity times the time of flight, so aim that far the other way.
    virtualTarget = TARGET.minus(
        new Translation2d(fieldVelocity.vx * timeOfFlight, fieldVelocity.vy * timeOfFlight));
  }
  return shotAt(robotPose, virtualTarget);
}
```
{% endcode %}
{% endtab %}

{% tab title="Newton's method" %}
**Read first:** [Newton's Method for Dynamic Shooting](https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/newton-shooting.html).

A time of flight method. It finds the same time of flight as the recursion in fewer passes, which helps when driving fast toward or away from the target.

{% code title="ShotCalculator.java (fragment)" %}
```java
/** Find the same time of flight with Newton's method, which needs fewer passes. */
public static Shot newton(Pose2d robotPose, ChassisVelocities fieldVelocity) {
  Translation2d toTarget = TARGET.minus(robotPose.getTranslation());
  double distance = toTarget.getNorm();
  // How fast the game piece covers ground, from the standing shot at this distance.
  double projectileSpeed = distance / TIME_OF_FLIGHT.get(distance);
  // How fast the robot is closing on the target. Negative while it drives away.
  double closingSpeed = (toTarget.getX() * fieldVelocity.vx + toTarget.getY() * fieldVelocity.vy) / distance;

  // A first guess that already allows for the robot's motion toward or away from the target.
  double timeOfFlight = distance / (projectileSpeed + closingSpeed);
  for (int i = 0; i < 3; i++) {
    // The virtual target for this guess.
    double dx = toTarget.getX() - fieldVelocity.vx * timeOfFlight;
    double dy = toTarget.getY() - fieldVelocity.vy * timeOfFlight;
    double virtualDistance = Math.hypot(dx, dy);
    // How far the guess is from what the table says for that distance.
    double error = timeOfFlight - TIME_OF_FLIGHT.get(virtualDistance);
    // How fast that error changes as the guess changes.
    double slope = 1 + (dx * fieldVelocity.vx + dy * fieldVelocity.vy) / (projectileSpeed * virtualDistance);
    timeOfFlight -= error / slope;
  }
  Translation2d virtualTarget = TARGET.minus(
      new Translation2d(fieldVelocity.vx * timeOfFlight, fieldVelocity.vy * timeOfFlight));
  return shotAt(robotPose, virtualTarget);
}
```
{% endcode %}
{% endtab %}
{% endtabs %}

The strategy is chosen in one line, `shot()` in `RobotContainer` (Commands v2) or `Robot` (Commands v3) in the next step. It calls `ShotCalculator.recursive(drive.getPose(), drive.getFieldRelativeSpeed())`. For Newton's method, change it to `ShotCalculator.newton(...)` with the same arguments; for standing still, to `ShotCalculator.standing(drive.getPose())`.

**Check it:** the code builds with all three methods in `ShotCalculator`.
{% endstep %}

{% step %}
### Aim, set the shooter speed, and shoot safely

This is where it comes together. Replace the template's file with this one. It uses the commands from the tutorials and the additions above, and the feeder's `feed()` and default command from [How do I set up an intake roller?](../how-do-i-set-up-an-intake-roller.md)

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="RobotContainer.java" lineNumbers="true" %}
```java
package first.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Seconds;

import first.robot.Constants.OperatorConstants;
import first.robot.subsystems.FeederSubsystem;
import first.robot.subsystems.ShooterSubsystem;
import first.robot.subsystems.SwerveSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.Trigger;
import yams.commands2.swerve.SwerveInputStream;

public class RobotContainer {
  private final SwerveSubsystem drive = new SwerveSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final FeederSubsystem feeder = new FeederSubsystem();

  private final CommandGamepad driverController =
      new CommandGamepad(OperatorConstants.DRIVER_CONTROLLER_PORT);

  /** The shot for where the robot is, and how it is moving, right now. */
  private ShotCalculator.Shot shot() {
    return ShotCalculator.recursive(drive.getPose(), drive.getFieldRelativeSpeed());
  }

  public RobotContainer() {
    // The left stick translates and the right stick's X axis rotates.
    // Pushing a stick forward or left reads negative, so negate each axis to match
    // WPILib's convention: +X forward, +Y left, counterclockwise positive.
    SwerveInputStream input = drive.getChassisSpeedsSupplier(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX())
        // While the right bumper is held, face the shot's target instead of
        // turning with the right stick. The left stick keeps driving.
        .withAim(() -> shot().target(), driverController.rightBumper())
        // Aim from where the robot will be when the game piece leaves it.
        .withAimLookahead(Seconds.of(0.1));
    drive.setDefaultCommand(drive.driveFieldRelative(input));

    // With no button held, the flywheel coasts and the feeder is stopped.
    shooter.setDefaultCommand(shooter.set(0));
    feeder.setDefaultCommand(feeder.stop());

    // While the right bumper is held, keep the flywheel at the speed the shot needs.
    driverController.rightBumper().whileTrue(shooter.track(() -> shot().flywheelSpeed()));

    // The shot is ready when the robot faces the aim heading, the flywheel is at the shot's
    // speed, and the robot is at a distance the tables cover. All three must hold for a tenth
    // of a second, so that one lucky loop does not fire a shot.
    Trigger ready = input.aimLock(Degrees.of(2))
        .and(shooter.atSpeed(() -> shot().flywheelSpeed()))
        .and(() -> ShotCalculator.inRange(drive.getPose()))
        .debounce(0.1);

    // Feed only while the shot is ready. The moment it is not, the feeder stops.
    ready.whileTrue(feeder.feed());
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="Robot.java" lineNumbers="true" %}
```java
package first.robot;

import first.robot.constants.DriverConstants;
import first.robot.mechanisms.ExampleMechanism;
import first.robot.mechanisms.FeederMechanism;
import first.robot.mechanisms.ShooterMechanism;
import first.robot.mechanisms.SwerveMechanism;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.system.Notifier;
import org.wpilib.telemetry.Telemetry;

public class Robot extends OpModeRobot {
  // The template's ExampleAuto still uses this one.
  final ExampleMechanism exampleMechanism = new ExampleMechanism();
  final SwerveMechanism drive = new SwerveMechanism();
  final ShooterMechanism shooter = new ShooterMechanism();
  final FeederMechanism feeder = new FeederMechanism();
  final CommandGamepad exampleController =
      new CommandGamepad(DriverConstants.DRIVER_CONTROLLER_PORT);

  // The scheduler that runs every command.
  private final Scheduler scheduler = Scheduler.getDefault();
  // Each mechanism's simulation Notifier, kept so that it is not garbage collected.
  private final List<Notifier> simulationNotifiers = new ArrayList<>();

  /** The shot for where the robot is, and how it is moving, right now. */
  ShotCalculator.Shot shot() {
    return ShotCalculator.recursive(drive.getPose(), drive.getFieldRelativeSpeed());
  }

  public Robot() {
    // With no button held, the flywheel coasts and the feeder is stopped.
    shooter.setDefaultCommand(shooter.idle());
    feeder.setDefaultCommand(feeder.idle());
  }

  @Override
  public void robotPeriodic() {
    // Runs every loop, on the robot and in simulation.
    // Update every mechanism before the scheduler runs its commands.
    drive.periodic();
    shooter.periodic();
    feeder.periodic();
    scheduler.run();
    // Publish the scheduler, so that a dashboard shows which commands are running.
    Telemetry.log("Scheduler", scheduler, Scheduler.proto);
  }

  @Override
  public void simulationInit() {
    // Runs once, when the simulation starts.
    // Run each mechanism's simulation in its own Notifier, every robot loop.
    startSimulation("Drive", drive::simulationPeriodic);
    startSimulation("Shooter", shooter::simulationPeriodic);
    startSimulation("Feeder", feeder::simulationPeriodic);
  }

  /** Runs a mechanism's simulation in its own Notifier, every robot loop period. */
  private void startSimulation(String name, Runnable simulationPeriodic) {
    Notifier notifier = new Notifier(simulationPeriodic);
    notifier.setName(name + " Simulation");
    notifier.startPeriodic(getPeriod());
    simulationNotifiers.add(notifier);
  }
}
```
{% endcode %}

{% code title="ExampleTeleop.java" lineNumbers="true" %}
```java
package first.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.command3.Trigger;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;
import yams.commands3.swerve.SwerveInputStream;

@Teleop
public class ExampleTeleop implements OpMode {
  public ExampleTeleop(Robot robot) {
    // The left stick translates and the right stick's X axis rotates.
    // Pushing a stick forward or left reads negative, so negate each axis to match
    // WPILib's convention: +X forward, +Y left, counterclockwise positive.
    SwerveInputStream input = robot.drive.getChassisSpeedsSupplier(
            () -> -robot.exampleController.getLeftY(),
            () -> -robot.exampleController.getLeftX(),
            () -> -robot.exampleController.getRightX())
        // While the right bumper is held, face the shot's target instead of
        // turning with the right stick. The left stick keeps driving.
        .withAim(() -> robot.shot().target(), robot.exampleController.rightBumper())
        // Aim from where the robot will be when the game piece leaves it.
        .withAimLookahead(Seconds.of(0.1));
    robot.drive.setDefaultCommand(robot.drive.driveFieldRelative(input));

    // While the right bumper is held, keep the flywheel at the speed the shot needs.
    robot.exampleController.rightBumper()
        .whileTrue(robot.shooter.track(() -> robot.shot().flywheelSpeed()));

    // The shot is ready when the robot faces the aim heading, the flywheel is at the shot's
    // speed, and the robot is at a distance the tables cover. All three must hold for a tenth
    // of a second, so that one lucky loop does not fire a shot.
    Trigger ready = input.aimLock(Degrees.of(2))
        .and(robot.shooter.atSpeed(() -> robot.shot().flywheelSpeed()))
        .and(() -> ShotCalculator.inRange(robot.drive.getPose()))
        .debounce(Seconds.of(0.1));

    // Feed only while the shot is ready. The moment it is not, the feeder stops.
    ready.whileTrue(robot.feeder.feed());
  }
}
```
{% endcode %}
{% endtab %}
{% endtabs %}

Holding the right bumper does three things at once:

1. **Aims.** [`withAim`](../../reference/swerve-input-stream/swerve-input-stream-aim.md#aim-at-a-target) turns the robot to face the shot's target while the left stick keeps driving, and [`withAimLookahead`](../../reference/swerve-input-stream/swerve-input-stream-aim.md#aim-lookahead) covers the short delay before the game piece leaves.
2. **Sets the shooter speed.** `shooter.track(...)` keeps the flywheel at the speed the shot needs, which changes as the robot moves.
3. **Shoots, but only when it is safe to.** The feeder runs only while `ready` is true.

`ready` is what makes the shot safe. It is true only when all of these hold, and have held for a tenth of a second:

| Check                                                                                                                                             | Why                                                                                                                                      |
| ------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| [`aimLock`](../../reference/swerve-input-stream/swerve-input-stream-aim.md#aim-lock): the robot is aiming and within 2 degrees of the aim heading | A shot fired while the robot is still turning goes wide. It is also false whenever the bumper is not held, so nothing feeds by accident. |
| `shooter.atSpeed(...)`: the flywheel is within 100 RPM of the shot's speed                                                                        | A flywheel that is still spinning up, or still recovering from the last shot, throws short.                                              |
| `ShotCalculator.inRange(...)`: the robot is at a distance you measured                                                                            | Outside the tables, the lookup just repeats its last entry and the shot misses.                                                          |
| `debounce(0.1)`                                                                                                                                   | One loop where everything happens to line up is not a ready shot.                                                                        |

Releasing the bumper ends the aim and the flywheel command. `ready` goes false, the feeder stops, and the default commands let the flywheel coast.

**Check it:** the code builds, and in simulation the robot turns to face the target while you hold the right bumper.

**For your robot:** tune the lookahead, the 2 degree and 100 RPM tolerances, and the debounce. Tighter is more accurate and slower to fire. If your shooter does not point out of the front of the robot, add [`withAimHeadingOffset`](../../reference/swerve-input-stream/swerve-input-stream-aim.md#aim-heading-offset).
{% endstep %}

{% step %}
### Try it in simulation

Run **WPILib: Simulate Robot Code**, connect a controller, and enable teleop. The robot starts at (0, 0), about 6.1 m from the target at (4.6, 4.0), which is outside the 1.5 to 4.5 m that `inRange` allows, so nothing feeds there. Drive toward the target until it is between 1.5 and 4.5 m away, such as to (2, 2), about 3.3 m away; `NT:/Mechanisms/swerve/pose` shows where the robot is.

Hold the right bumper and drive. The robot keeps facing the shot's target, and the flywheel's setpoint follows the shot. Graph in AdvantageScope:

* `NT:/Mechanisms/Shooter/ShooterMotor/closedloop/setpoint/velocity` and `mechanism/velocity` beside it: the shot's speed, and the flywheel's.
* `NT:/Mechanisms/Feeder/FeederMotor/motor/outputVoltage`: the feeder, which only runs while `ready` is true.

The simulation does not model the game piece, so it shows that the logic works, and not whether shots land.

**Check it:** in range, with the bumper held and the robot facing the target, the feeder's voltage goes up once the flywheel's speed settles on the setpoint. Out of range, it stays at 0.
{% endstep %}
{% endstepper %}

## The complete ShotCalculator file

{% code title="ShotCalculator.java" lineNumbers="true" %}
```java
package first.robot;

import static org.wpilib.units.Units.RPM;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.measure.AngularVelocity;

/** Works out where to aim and how fast to spin the flywheel. */
public final class ShotCalculator {
  /** Where to aim, and the flywheel speed that reaches it. */
  public record Shot(Pose2d target, AngularVelocity flywheelSpeed) {}

  // Where the real target is on the field, in meters.
  private static final Translation2d TARGET = new Translation2d(4.6, 4.0);

  // Distance to the target in meters, to flywheel speed in RPM.
  private static final InterpolatingDoubleTreeMap FLYWHEEL_RPM = new InterpolatingDoubleTreeMap();
  // Distance to the target in meters, to how long the shot is in the air in seconds.
  private static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT = new InterpolatingDoubleTreeMap();

  static {
    // Measure these on your own robot, standing still. Placeholders, all below the
    // tutorial shooter's top speed of about 3800 RPM.
    FLYWHEEL_RPM.put(1.5, 2000.0);
    FLYWHEEL_RPM.put(3.0, 2500.0);
    FLYWHEEL_RPM.put(4.5, 3000.0);

    TIME_OF_FLIGHT.put(1.5, 0.45);
    TIME_OF_FLIGHT.put(3.0, 0.70);
    TIME_OF_FLIGHT.put(4.5, 0.95);
  }

  private ShotCalculator() {}

  /** Whether the robot is at a distance the tables cover. Outside it they only repeat their last entry. */
  public static boolean inRange(Pose2d robotPose) {
    double distance = robotPose.getTranslation().getDistance(TARGET);
    return distance >= 1.5 && distance <= 4.5;
  }

  /** Aim at the real target. Only right while the robot is standing still. */
  public static Shot standing(Pose2d robotPose) {
    return shotAt(robotPose, TARGET);
  }

  /** Lead the target by the time of flight, and repeat until the time settles. */
  public static Shot recursive(Pose2d robotPose, ChassisVelocities fieldVelocity) {
    Translation2d virtualTarget = TARGET;
    // Each pass uses the distance from the pass before, so the answer settles after a few.
    for (int i = 0; i < 3; i++) {
      double distance = robotPose.getTranslation().getDistance(virtualTarget);
      double timeOfFlight = TIME_OF_FLIGHT.get(distance);
      // The shot drifts by the robot's velocity times the time of flight, so aim that far the other way.
      virtualTarget = TARGET.minus(
          new Translation2d(fieldVelocity.vx * timeOfFlight, fieldVelocity.vy * timeOfFlight));
    }
    return shotAt(robotPose, virtualTarget);
  }

  /** Find the same time of flight with Newton's method, which needs fewer passes. */
  public static Shot newton(Pose2d robotPose, ChassisVelocities fieldVelocity) {
    Translation2d toTarget = TARGET.minus(robotPose.getTranslation());
    double distance = toTarget.getNorm();
    // How fast the game piece covers ground, from the standing shot at this distance.
    double projectileSpeed = distance / TIME_OF_FLIGHT.get(distance);
    // How fast the robot is closing on the target. Negative while it drives away.
    double closingSpeed = (toTarget.getX() * fieldVelocity.vx + toTarget.getY() * fieldVelocity.vy) / distance;

    // A first guess that already allows for the robot's motion toward or away from the target.
    double timeOfFlight = distance / (projectileSpeed + closingSpeed);
    for (int i = 0; i < 3; i++) {
      // The virtual target for this guess.
      double dx = toTarget.getX() - fieldVelocity.vx * timeOfFlight;
      double dy = toTarget.getY() - fieldVelocity.vy * timeOfFlight;
      double virtualDistance = Math.hypot(dx, dy);
      // How far the guess is from what the table says for that distance.
      double error = timeOfFlight - TIME_OF_FLIGHT.get(virtualDistance);
      // How fast that error changes as the guess changes.
      double slope = 1 + (dx * fieldVelocity.vx + dy * fieldVelocity.vy) / (projectileSpeed * virtualDistance);
      timeOfFlight -= error / slope;
    }
    Translation2d virtualTarget = TARGET.minus(
        new Translation2d(fieldVelocity.vx * timeOfFlight, fieldVelocity.vy * timeOfFlight));
    return shotAt(robotPose, virtualTarget);
  }

  /** The shot that reaches a point on the field from where the robot is. */
  private static Shot shotAt(Pose2d robotPose, Translation2d aimPoint) {
    double distance = robotPose.getTranslation().getDistance(aimPoint);
    return new Shot(new Pose2d(aimPoint, new Rotation2d()), RPM.of(FLYWHEEL_RPM.get(distance)));
  }
}
```
{% endcode %}

## If it did not work

* **The robot does not turn to face the target.** The rotation controller in `SwerveDriveConfig` has no gain, or the aim command is not running. See the [Swerve Input Stream](../../reference/swerve-input-stream/swerve-input-stream-aim.md#aim-at-a-target) page.
* **The feeder never runs.** The robot is out of range (see [Try it in simulation](shoot-on-the-move-example.md#try-it-in-simulation)), or the flywheel never gets within its tolerance; tune it with [How do I shoot with a shooter?](../how-to-shoot-with-a-shooter.md)
* **The flywheel's speed stays below its setpoint.** A table asks for more than the shooter can reach: keep the speeds well below its top speed, or change its gearing.
* **Something else.** See [Troubleshooting](../../tutorials/troubleshooting.md).

## Read more

Tune the YAMS side of the shot: [Tuning the YAMS side](./#tuning-the-yams-side).
