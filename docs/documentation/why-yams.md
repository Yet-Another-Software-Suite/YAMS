---
icon: rocket
---

# Why YAMS?

## The problem with creating a subsystem from scratch

Teams routinely spend days getting a single mechanism working in simulation, only to discover the real robot behaves differently because the sim wasn't accurate enough to tune against. YAMS eliminates that cycle.

Writing a motor subsystem from scratch is repetitive and error-prone. A production-quality subsystem requires:

* Motor controller configuration (PID, feedforward, gearing, current limits, zero power mode, ramp rates…)
* A physics simulation that actually matches your hardware
* Periodic telemetry publishing so you can see what's happening
* A `Mechanism2d` drawing of the mechanism, to watch it move in AdvantageScope
* Command factories that respect WPILib's requirement system
* Soft limits and hard limits
* Separate sim-only tuning paths
* Simulation of the motor controllers and sensors, so the same code runs in simulation and on the robot

## What YAMS replaces

A typical Arm subsystem created from scratch is 150-300 lines of code: config, sim state, periodic updates, command factories, limit enforcement, and telemetry. The YAMS equivalent is about 20 lines: one `SmartMotorControllerConfig`, one `SmartMotorController`, one `ArmConfig`, and one `Arm`, whose commands you use directly.

{% code title="Inside an arm subsystem's constructor (Java, Commands v2)" %}
```java
// `this` is the arm subsystem.
SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(4, 0, 0) // Example gain, tune it for your arm.
    .withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
    .withFeedforward(new ArmFeedforward(0, 0, 0, 0)) // Placeholder gains, until you tune them.
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
    .withZeroPower(MotorMode.BRAKE)
    .withStatorCurrentLimit(Amps.of(40))
    .withStartingPosition(Degrees.of(0))
    .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH);

SmartMotorController smc = new SparkWrapper(new SparkMax(CANPort.CAN_S0, 1, MotorType.kBrushless), DCMotor.getNEO(1), motorConfig);

ArmConfig armConfig = new ArmConfig()
    .withLength(Feet.of(2))
    .withHardLimits(Degrees.of(-10), Degrees.of(120));

Arm arm = new Arm(armConfig, smc);
Command raise = arm.setAngle(Degrees.of(90));
```
{% endcode %}

Add two calls to the subsystem, `arm.updateTelemetry()` in `periodic()` and `arm.simIterate()` in `simulationPeriodic()`, and that's it. Simulation, telemetry, hard limits, and a physics-backed `Mechanism2d`, all in one library. The [Arm tutorial](tutorials/arm/) builds this subsystem step by step, with the complete file.

## Built-in physics simulation

Every YAMS mechanism type ships with a physics model:

* **Arm:** WPILib's `SingleJointedArmSim`, with gravity included.
* **Pivot:** a `DCMotorSim` built from the pivot's moment of inertia.
* **Elevator:** `ElevatorSim`, with the carriage weight (`withCarriageWeight`) and the elevator's angle (`withAngle`).
* **FlyWheel:** a `DCMotorSim` built from the flywheel's moment of inertia.

YAMS creates the model for you when the code runs in simulation. Your code only calls `simIterate()` every loop in simulation. In Commands v2 that is the subsystem's `simulationPeriodic()`. In Commands v3, `Robot.java` starts it, as [Update it every loop](tutorials/arm/mechanism.md#update-it-every-loop) shows.

Because the simulation uses accurate physics, the PID and feedforward gains you tune in sim are a reliable starting point for real hardware. You may still need minor adjustments, but you won't be starting from zero on the field.

## TunerX in simulation (CTRE hardware)

For CTRE TalonFX and TalonFXS motors, YAMS feeds its physics results back into Phoenix's simulation, so Phoenix Tuner X can connect to your simulated robot. Every TalonFX signal (position, velocity, voltage and current) reads exactly as it would from real hardware.

You can watch closed-loop error, plot signal graphs, and verify your gains in TunerX's familiar interface without touching a physical robot. The same workflow you use to diagnose real hardware works identically in simulation. [TunerX in simulation](how-to-tuning/live-tuning/tunerx-in-simulation.md) shows how to connect.

## Live tuning without redeploying

`TelemetryVerbosity.HIGH` publishes all PID gains, feedforward constants, motion profile constraints, soft limits, and current limits as values you can change from a dashboard. At `HIGH` verbosity YAMS also registers a `Live Tuning` command for each motor (call `setupLiveTuning()` on the config to register it in other cases). While that command runs, from Elastic or any other NetworkTables dashboard, YAMS applies the values you change to the running motor controller.

Changing `kP` from 4 to 6 and seeing the mechanism respond takes seconds, not the minutes a redeploy would cost. [Live tuning](how-to-tuning/live-tuning/) shows how to tune a mechanism this way, and [Live tuning keys and units](reference/keys-and-units.md) lists every value you can change.

## Loop overruns

Heavy subsystems whose telemetry, simulation, and visualization code is created from scratch can push the 20 ms robot loop toward or past its budget, especially when multiple mechanisms run simultaneously. YAMS's telemetry and simulation paths are tightly optimized and only run the work that the verbosity level requires; `LOW` publishes only the fields you actually need for competition, keeping loop time predictable.

The SystemCore, which YAMS 2027 runs on, has plenty of processing headroom, so running `TelemetryVerbosity.HIGH` with multiple mechanisms active is not a practical loop time concern.

## Safety by default

YAMS enforces mechanism safety automatically:

* **Hard limits** are enforced in simulation as physical stops: the mechanism cannot travel past them.
* **Soft limits** are pushed to the motor controller, so the closed-loop controller respects them independent of your command logic.
* **Misconfigured limits** throw an exception when the code starts (for example, a lower limit greater than the upper limit), so the mistake shows up in testing rather than during a match.

You can also set a **temperature cutoff** with `withTemperatureCutoff`, which stops the motor if it overheats.

## AdvantageKit integration

YAMS mechanisms fit behind AdvantageKit's IO layer, so you keep YAMS's simulation and live tuning and get AdvantageKit's log replay. The [AdvantageKit](tutorials/advantagekit-integration/) tutorial walks you through it.
