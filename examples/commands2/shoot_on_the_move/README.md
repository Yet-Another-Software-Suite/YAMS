# Shoot on the move

A swerve robot that shoots while it drives: hold the right bumper, and it turns to face the shot's aim point, keeps its
flywheel at the speed the shot needs, and runs the feeder only once the shot is ready. The left stick keeps driving the
whole time.

The ideas behind it, and where to read more about each, are on the **Advanced > Shoot on the move** pages of the YAMS
docs. This example is the code those pages point at.

## The files

| File | What it does |
| --- | --- |
| `ShotCalculator.java` | Works out the shot: where to aim and how fast to spin the flywheel, from two lookup tables measured standing still. Holds all three firing strategies: `standing`, `recursive` and `newton`. |
| `RobotContainer.java` | Puts it together: `withAim` and `withAimLookahead` on the drive's `SwerveInputStream`, `track` on the shooter, and the `ready` trigger that feeds. `shot()` picks the strategy. |
| `subsystems/SwerveSubsystem.java` | The Swerve Drive tutorial's drive, with `getPose()` and `getFieldRelativeSpeed()` for the shot, and a stronger rotation controller (see below). |
| `subsystems/ShooterSubsystem.java` | The Shooter tutorial's flywheel, with `track(Supplier)` for a speed that changes every loop and `atSpeed(Supplier, tolerance)` to compare against it. |
| `subsystems/FeederSubsystem.java` | A roller, as a YAMS `FlyWheel`, that pushes a game piece into the flywheel. |
| `Constants.java` | The aim and speed tolerances, the debounce and the aim lookahead. |

## Try it in simulation

1. Generate and run it: `make shoot_on_the_move` in `examples`, then **WPILib: Simulate Robot Code** in
   `examples/generated/shoot_on_the_move`.
2. Connect a controller and enable teleop. The robot starts at (2, 2), about 3.3 m from the target at (4.6, 4.0), inside
   the 1.5 to 4.5 m the tables cover.
3. Hold the right bumper. The robot turns to face the target, the flywheel spins up, and the feeder runs.
4. Keep holding it and drive. In AdvantageScope, `NT:/Mechanisms/swerve/field` shows the real `Target` and the
   `Aim point`, which moves against the robot's velocity. Graph `NT:/Mechanisms/Feeder/FeederMotor/motor/outputVoltage`:
   it only goes up while the shot is ready.

The simulation does not model the game piece, so it shows the logic works, not whether shots land.

## Why the rotation controller has a kP of 5

A swerve module holds still when its wheel would turn slower than its velocity deadband, 0.1 m/s by default. So a
heading controller stops turning the robot once its output gets that small. With the tutorial's kP of 1, that leaves the
robot about 6.6 degrees short of the aim heading, outside the 2 degree aim lock, and the shot never becomes ready. A kP
of 5 stops it within about 1.3 degrees.

## For your robot

Everything in the tables is a placeholder. Measure the flywheel speed and time of flight from each distance on your own
robot, standing still, set `TARGET` to your field's target (and flip it for the red alliance), change `MIN_DISTANCE` and
`MAX_DISTANCE` to the distances you measured, then tune the lookahead, the tolerances and the rotation controller.

## Tests

- `ShotCalculatorTest` checks the strategies: standing still they all aim at the real target, moving they lead against
  the robot's velocity, and the lead is the velocity times the time of flight to the aim point.
- `ShootOnTheMoveTest` runs the whole robot in simulation: with the right bumper held, the feeder waits until the robot
  faces the target and the flywheel is at speed, then runs, and stops when the bumper is let go.
