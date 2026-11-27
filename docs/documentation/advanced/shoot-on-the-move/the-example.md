---
description: >-
  The shoot_on_the_move example robot in the YAMS repository: which file does
  each part of the idea, how to run it, and how to make it yours.
icon: code
---

# The shoot on the move example

The [`shoot_on_the_move` example](https://github.com/Yet-Another-Software-Suite/YAMS/tree/master/examples/commands2/shoot_on_the_move) in the YAMS repository is a swerve robot that shoots while it drives. Hold the right bumper, and it turns to face the shot's aim point, keeps its flywheel at the speed the shot needs, and runs its feeder only once the shot is ready. The left stick keeps driving the whole time. It comes in Java with Commands v2 and [Commands v3](https://github.com/Yet-Another-Software-Suite/YAMS/tree/master/examples/commands3/shoot_on_the_move).

[Shoot on the move](./) is the reference for the ideas, and [Shooting on the move](../../understanding/shooters.md#shooting-on-the-move) explains the lead with pictures. This page connects them to the code: what each file does, and where each idea lives in it.

**It builds on:** the [Swerve Drive](../../tutorials/swerve-drive/) and [Shooter](../../tutorials/shooter-flywheels/) tutorials, whose drive and flywheel it reuses, and a feeder built as in [How do I set up an intake roller?](../../how-to-mechanisms/how-do-i-set-up-an-intake-roller.md)

{% hint style="info" %}
The numbers in the lookup tables are placeholders, and the robot has only run in simulation. The C++ port of YAMS has the same pieces (`SwerveInputStream::WithAim`, `WithAimLookahead` and `IsAimLocked`, and `FlyWheel::Run` with a function), so the same approach works there too.
{% endhint %}

## Run it first

1. In the YAMS repository, generate the project from `examples`: `make shoot_on_the_move` for Commands v2, or `make commands3-shoot_on_the_move` for Commands v3. The [examples README](https://github.com/Yet-Another-Software-Suite/YAMS/tree/master/examples#readme) explains what that does.
2. Open the generated folder, `examples/generated/shoot_on_the_move` (or `commands3-shoot_on_the_move`), in VS Code, and run **WPILib: Simulate Robot Code**.
3. Connect a controller and enable teleop (for Commands v3, pick **Shoot On The Move Teleop**). The robot starts at (2, 2), about 3.3 m from the target at (4.6, 4.0).
4. Hold the right bumper. The robot turns to face the target, the flywheel spins up, and the feeder runs.
5. Keep holding it and drive with the left stick.

Open AdvantageScope and add `NT:/Mechanisms/swerve/field` to a 2D field. It shows the robot, the real `Target`, and the `Aim point`. Standing still, the aim point sits on the target. Drive, and it slides away from the target, against the direction you are driving: that is the lead. Graph these too:

* `NT:/Mechanisms/Shooter/ShooterMotor/closedloop/setpoint/velocity` and `mechanism/velocity` beside it: the speed the shot needs, which changes as you drive, and the flywheel following it.
* `NT:/Mechanisms/Feeder/FeederMotor/motor/outputVoltage`: the feeder, which only runs while the shot is ready.

The simulation does not model the game piece, so it shows that the logic works, and not whether shots land.

## Where each idea is in the code

The file names are the Commands v2 ones. In Commands v3, the subsystems are mechanisms (`SwerveMechanism`, `ShooterMechanism`, `FeederMechanism`), and what `RobotContainer` does is split between `Robot` and `opmodes/teleop/ShootOnTheMoveTeleop`.

| Idea | Read about it in | In the example |
| --- | --- | --- |
| Where the robot is, and how fast it moves on the field | [Shoot on the move](./#how-the-sources-ideas-apply-to-yams) | `getPose()` and `getFieldRelativeSpeed()` in `SwerveSubsystem` |
| A lookup table of shots that score, measured standing still | [Static Shooting via Look-Up Table](https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/static-shooting.html) | `FLYWHEEL_RPM` and `TIME_OF_FLIGHT` in `ShotCalculator`, with `timeOfFlight(Distance)` to read the second |
| The lead, as a virtual target | [A primer: vector subtraction](../../understanding/shooters.md#a-primer-vector-subtraction) | `ShotCalculator.recursive(...)` and `ShotCalculator.newton(...)` |
| Picking a firing strategy | [The firing strategies](#the-firing-strategies), below | `shot()` in `RobotContainer`, one line |
| Turning to face the aim point while the driver keeps driving | [Aim at a target](../../reference/swerve-input-stream/swerve-input-stream-aim.md#aim-at-a-target) | `withAim(() -> shot().target(), ...)` on the drive's `SwerveInputStream` |
| Aiming from where the robot will be when the game piece leaves | [Aim lookahead](../../reference/swerve-input-stream/swerve-input-stream-aim.md#aim-lookahead) | `withAimLookahead(AIM_LOOKAHEAD)`, set in `Constants` |
| A flywheel speed that changes every loop | [Shooting on the move](../../understanding/shooters.md#shooting-on-the-move) | `track(Supplier)` in `ShooterSubsystem`, which uses `FlyWheel.run(Supplier)` |
| Only shooting when the shot is trustworthy | [When it shoots](#when-it-shoots-the-ready-trigger), below | The `ready` trigger in `RobotContainer` |
| A roller that pushes the game piece in | [How do I set up an intake roller?](../../how-to-mechanisms/how-do-i-set-up-an-intake-roller.md) | `FeederSubsystem`, a `FlyWheel` run by voltage |
| Seeing the lead | This page, [Run it first](#run-it-first) | `showShotOnField()` in `RobotContainer` (`Robot.robotPeriodic()` in Commands v3), on the drive's `getField2d()` |

## The firing strategies

`ShotCalculator` holds the three strategies from WPILib's fire control pages, each one method. All three start from the same two tables:

* **`standing`** aims at the real target and reads the flywheel speed for the distance. Only right while the robot is not moving. Read [Static Shooting via Look-Up Table](https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/static-shooting.html).
* **`recursive`** moves the target against the robot's velocity by the time of flight. That changes the distance, and so the time of flight, so it repeats three times. This is the virtual target most teams use. Read [Dynamic Shooting via Time-of-Flight Recursion](https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/dynamic-shooting.html).
* **`newton`** finds the same time of flight with Newton's method, which settles in fewer passes. That matters when driving fast toward or away from the target, where three passes of the recursion are still a few centimeters off. Read [Newton's Method for Dynamic Shooting](https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/newton-shooting.html).

The heart of the recursion is one line, repeated:

{% code title="ShotCalculator.java (from recursive)" %}
```java
// The shot drifts by the robot's velocity times the time of flight, so aim that far the other way.
virtualTarget = TARGET.minus(new Translation2d(fieldVelocity.vx * timeOfFlight, fieldVelocity.vy * timeOfFlight));
```
{% endcode %}

To try another strategy, change the one line in `shot()`: `ShotCalculator.newton(...)` with the same arguments, or `ShotCalculator.standing(drive.getPose())` to see what happens without a lead. Drive sideways past the target with `standing` and watch the aim point stay on the target: a real shot would land beside it.

`ShotCalculatorTest` checks the strategies against each other: standing still they all aim at the real target, and moving, the aim point's distance from the target is the robot's velocity times the time of flight to the aim point.

## When it shoots: the ready trigger

Holding the right bumper aims the robot and spins up the flywheel. The feeder runs only while `ready` is true:

{% code title="RobotContainer.java (fragment)" %}
```java
Trigger ready = input.aimLock(AIM_TOLERANCE)
    .and(shooter.atSpeed(() -> shot().flywheelSpeed(), SPEED_TOLERANCE))
    .and(() -> ShotCalculator.inRange(drive.getPose()))
    .debounce(READY_DEBOUNCE.in(Seconds));

ready.whileTrue(feeder.feed());
```
{% endcode %}

| Check | Why |
| --- | --- |
| [`aimLock`](../../reference/swerve-input-stream/swerve-input-stream-aim.md#aim-lock): the robot is aiming and within 2 degrees of the aim heading | A shot fired while the robot is still turning goes wide. It is also false whenever the bumper is not held, so nothing feeds by accident. |
| `atSpeed(...)`: the flywheel is within 100 RPM of the shot's speed | A flywheel that is still spinning up, or still recovering from the last shot, throws short. `FlyWheel.near(...)` takes a fixed speed, so `atSpeed` compares against the shot's speed as it changes. |
| `ShotCalculator.inRange(...)`: the robot is at a distance the tables cover | Outside the tables, a lookup only repeats its last entry, and the shot misses. |
| `debounce`: all three hold for a tenth of a second | One loop where everything happens to line up is not a ready shot. |

The tolerances, the debounce and the lookahead are all in `Constants`. Tighter is more accurate, and slower to fire. `ShootOnTheMoveTest` runs the whole robot in simulation and checks this: the feeder waits until the robot faces the target and the flywheel is at speed, then runs, and stops when the bumper is let go.

## Why the heading controller's kP is 5

The Swerve Drive tutorial's rotation controller has a kP of 1. This example uses 5, because with 1 the robot never fires.

A swerve module holds still when its wheel would turn slower than its velocity deadband, 0.1 m/s by default (`SwerveModuleConfig.withVelocityDeadband`). That stops the wheels from twitching when the sticks are barely off center, but it also means the robot cannot turn very slowly. On the example's drive, with modules about 0.86 m from the center, 0.1 m/s at the wheels is about 0.12 radians per second of turning.

A proportional heading controller asks for less and less turning as the robot gets close to the aim heading. With a kP of 1, it asks for less than 0.12 radians per second once the robot is within about 0.12 radians, or 6.6 degrees, of the aim heading, so the robot stops there. That is outside the 2 degree `aimLock`, so `ready` never turns true. With a kP of 5, the robot gets within about 1.3 degrees before it stops.

On your robot, raise the kP until the robot settles inside your aim tolerance without overshooting back and forth. [`withAimFeedforward`](../../reference/swerve-input-stream/swerve-input-stream-aim.md#aim-feedforward) helps the heading keep up while driving past the target.

## Make it yours

Every number in the example stands in for a measurement from your robot:

1. **Measure the tables standing still:** the flywheel speed that scores from each distance, and how long that shot is in the air. Eeshwar's [part 2](https://blog.eeshwark.com/robotblog/shooting-on-the-fly-pt2) describes how, with a tape measure and a phone.
2. **Set the field:** `TARGET` to your target's position, flipped for the red alliance, and `MIN_DISTANCE` and `MAX_DISTANCE` to the distances you measured.
3. **Swap in your mechanisms:** your drive, shooter and feeder, keeping `getPose()`, `getFieldRelativeSpeed()`, `track(...)`, `atSpeed(...)` and `feed()`. Keep every table speed well below your shooter's top speed.
4. **Tune, in this order:** the standing shot, then the rotation controller, then the lookahead (by driving straight across the front of the target), then the tolerances. [Tuning the YAMS side](./#tuning-the-yams-side) lists the settings that go with each piece of advice in the sources.

If your shooter does not point out of the front of the robot, add [`withAimHeadingOffset`](../../reference/swerve-input-stream/swerve-input-stream-aim.md#aim-heading-offset).

## If it does not work

* **The robot does not turn to face the target.** The aim is not running: check that the drive's default command uses the stream with `withAim`, and that the rotation controller in `SwerveDriveConfig` has a kP. See [Aim at a target](../../reference/swerve-input-stream/swerve-input-stream-aim.md#aim-at-a-target).
* **The robot turns, but stops a few degrees short, and the feeder never runs.** The rotation controller's kP is too low for the velocity deadband: see [Why the heading controller's kP is 5](#why-the-heading-controllers-kp-is-5).
* **The feeder never runs, and the robot faces the target.** The robot is out of range, or the flywheel never gets within its tolerance: tune it with [How do I shoot with a shooter?](../../how-to-mechanisms/how-to-shoot-with-a-shooter.md)
* **The flywheel's speed stays below its setpoint.** A table asks for more than the shooter can reach: keep the speeds well below its top speed, or change its gearing.
* **Something else.** See [Troubleshooting](../../tutorials/troubleshooting.md).
