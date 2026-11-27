---
icon: people-arrows
---

# Followers

Two ways to have more than one motor move one mechanism: **tightly coupled** followers and **loosely coupled** followers.

<figure><img src="../../../.gitbook/assets/smc-followers.png" alt="Two panels. Tightly coupled, withFollowers: a leader and a follower are on one shaft or gearbox and the follower copies the leader&#x27;s output. The motors are mechanically locked, there is one closed loop and one config, the followers are the same vendor as the leader, and voltage and duty cycle reach every motor. Loosely coupled, withLooselyCoupledFollowers: two separate motors joined only by the mechanism get the same setpoint and each runs its own PID. They are not mechanically locked, each has its own closed loop and config, vendors can be mixed, and voltage and duty cycle are not forwarded, so the leader moves alone."><figcaption><p>Tightly coupled followers share a shaft. Loosely coupled followers are separate motors.</p></figcaption></figure>

|                        | Tightly coupled: `withFollowers`                            | Loosely coupled: `withLooselyCoupledFollowers`                                   |
| ---------------------- | ----------------------------------------------------------- | -------------------------------------------------------------------------------- |
| The motors are         | Mechanically locked on one shaft or gearbox                 | Separate, joined only by the mechanism                                           |
| Closed loop            | One, on the leader. Followers copy its output               | One on each motor, on its own encoder                                            |
| Config                 | Only the leader has one                                     | Every motor has its own                                                          |
| Vendors                | Same vendor as the leader                                   | Can be mixed                                                                     |
| Voltage and duty cycle | Reach every motor                                           | **Not forwarded.** The leader moves alone, and the followers stop                |
| Use it for             | Two motors on one gearbox, such as an elevator or a shooter | Two actuators that must arrive together, such as a cross bar lifted at both ends |
| Mechanism classes      | Work with them                                              | Work with them: give the mechanism the leader                                    |

## `withFollowers`

Tightly coupled followers, for motors on the same shaft or gearbox. Each follower copies the leader's output, so only the leader needs a config.

```java
.withFollowers(Pair.of(new SparkMax(CANPort.CAN_S0, 4, MotorType.kBrushless), false),
               Pair.of(new SparkMax(CANPort.CAN_S0, 5, MotorType.kBrushless), true))
```

Pass the vendor's motor object, not a `SmartMotorController`. The boolean is whether that follower is inverted relative to the leader: motors facing each other across a gearbox usually are.

**Not supported everywhere.** Followers must be the same vendor as the leader. A TalonFX and a TalonFXS are both CTRE, so either can follow the other. To mix brands, use [`withLooselyCoupledFollowers`](followers.md#withlooselycoupledfollowers). An A301 has no follower mode, so its followers are other `A301Wrapper`s, which it sends its voltage in software: `.withFollowers(Pair.of(rightA301Motor, true))`. Anything else throws a `SmartMotorControllerConfigurationException`. See [Several A301s on one mechanism](https://yams.yassrobotics.com/api-reference/java-reference/motor-controllers/a301-wrapper#several-a301s-on-one-mechanism).

| TalonFX                          | TalonFXS                         | SPARK MAX / Flex       | Thrifty Nova          | A301                           |
| -------------------------------- | -------------------------------- | ---------------------- | --------------------- | ------------------------------ |
| ✅ TalonFX and TalonFXS followers | ✅ TalonFXS and TalonFX followers | ✅ SPARK followers only | ✅ Nova followers only | ✅ `A301Wrapper` followers only |

## `withLooselyCoupledFollowers`

Loosely coupled followers are for motors that are not on the same shaft or gearbox but must arrive at the same place: an intake arm driven from both sides, two linear actuators joined by a cross bar, or a shooter with a wheel on each end. Each follower is a full `SmartMotorController` with its own config, its own encoder and its own closed loop. The leader sends it every position and velocity setpoint it is given.

**You can mix brands.** Because each motor closes its own loop, a loosely coupled follower can be any supported motor controller, whatever the leader is, such as a SPARK leading a TalonFX.

**Supported on:** every supported motor controller.

**In simulation.** A mechanism class simulates the leader and its loosely coupled followers together, since they move one mechanism:

* Its simulation has every motor (the leader's `getMechanismDCMotor()`). Give each motor its own `DCMotor`, such as `DCMotor.getNEO(1)`, and the mechanism adds them up.
* Each motor applies its own output, as its motor controller's simulation (`SparkSim`, Phoenix, the Nova or the A301) gives it, with its own current limit. So a follower that isn't driven, such as during manual control, holds the mechanism back, as it does on the robot.
* The mechanism iterates each follower's simulation right after its own, and each follower's encoder reads it. The battery simulation counts their current once.

Without a mechanism class, such as two flywheels on their own shafts, each motor simulates its own load, and you call `simIterate()` on each follower.

### Use them with a mechanism class

A mechanism class such as `Arm`, `Elevator` or `FlyWheel` works with loosely coupled followers. Give the mechanism the leader, and name the followers in the leader's config. The mechanism's commands, such as `arm.setAngle(...)`, then move every motor.

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="subsystems/IntakeArmSubsystem.java" %}
```java
public class IntakeArmSubsystem extends SubsystemBase {

  // The right side of the arm. Declare a follower before the leader, which names it.
  private final SparkMax rightSpark = new SparkMax(CANPort.CAN_S0, 5, MotorType.kBrushless);
  private final SmartMotorController rightMotor = new SparkWrapper(rightSpark, DCMotor.getNEO(1),
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(50, 0, 0)
          .withSimClosedLoopController(50, 0, 0)
          .withTrapezoidalProfile(DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withFeedforward(new ArmFeedforward(0, 0, 0))
          .withSimFeedforward(new ArmFeedforward(0, 0, 0))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          // The right motor faces the other way, so it is inverted here, in its own config.
          .withMotorInverted(true)
          .withZeroPower(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40))
          .withStartingPosition(Degrees.of(-5))
          .withSoftLimits(Degrees.of(-20), Degrees.of(30))
          .withTelemetry("IntakeArmRightMotor", TelemetryVerbosity.HIGH));

  // The left side of the arm: the leader, which the Arm drives.
  private final SparkMax leftSpark = new SparkMax(CANPort.CAN_S0, 4, MotorType.kBrushless);
  private final SmartMotorController leftMotor = new SparkWrapper(leftSpark, DCMotor.getNEO(1),
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(50, 0, 0)
          .withSimClosedLoopController(50, 0, 0)
          .withTrapezoidalProfile(DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withFeedforward(new ArmFeedforward(0, 0, 0))
          .withSimFeedforward(new ArmFeedforward(0, 0, 0))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          .withMotorInverted(false)
          .withZeroPower(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40))
          .withStartingPosition(Degrees.of(-5))
          .withSoftLimits(Degrees.of(-20), Degrees.of(30))
          .withTelemetry("IntakeArmLeftMotor", TelemetryVerbosity.HIGH)
          // The right motor is sent every position and velocity setpoint the left motor is given.
          .withLooselyCoupledFollowers(rightMotor));

  private final Arm arm = new Arm(new ArmConfig()
      .withHardLimits(Degrees.of(-25), Degrees.of(35))
      .withLength(Meters.of(0.5))
      .withTelemetry("IntakeArm", TelemetryVerbosity.HIGH),
      leftMotor);

  public Command setAngle(Angle angle) {
    // Moves both motors.
    return arm.setAngle(angle);
  }

  @Override
  public void periodic() {
    arm.updateTelemetry();
    // The Arm only updates the leader. Update each follower yourself.
    rightMotor.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // Simulates both motors: the Arm iterates its followers.
    arm.simIterate();
  }
}
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
The same arm as a Commands v3 `Mechanism`, with `SmartMotorControllerConfig` and `Arm` from `yams.commands3`. A `Mechanism` has no periodic hook of its own, so the class gets a `periodic()` and a `simulationPeriodic()` that update the arm and the follower, and `Robot.java` calls them.

{% code title="mechanisms/IntakeArmMechanism.java" %}
```java
public class IntakeArmMechanism implements Mechanism {

  // The right side of the arm. Declare a follower before the leader, which names it.
  private final SparkMax rightSpark = new SparkMax(CANPort.CAN_S0, 5, MotorType.kBrushless);
  private final SmartMotorController rightMotor = new SparkWrapper(rightSpark, DCMotor.getNEO(1),
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(50, 0, 0)
          .withSimClosedLoopController(50, 0, 0)
          .withTrapezoidalProfile(DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withFeedforward(new ArmFeedforward(0, 0, 0))
          .withSimFeedforward(new ArmFeedforward(0, 0, 0))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          // The right motor faces the other way, so it is inverted here, in its own config.
          .withMotorInverted(true)
          .withZeroPower(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40))
          .withStartingPosition(Degrees.of(-5))
          .withSoftLimits(Degrees.of(-20), Degrees.of(30))
          .withTelemetry("IntakeArmRightMotor", TelemetryVerbosity.HIGH));

  // The left side of the arm: the leader, which the Arm drives.
  private final SparkMax leftSpark = new SparkMax(CANPort.CAN_S0, 4, MotorType.kBrushless);
  private final SmartMotorController leftMotor = new SparkWrapper(leftSpark, DCMotor.getNEO(1),
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(50, 0, 0)
          .withSimClosedLoopController(50, 0, 0)
          .withTrapezoidalProfile(DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withFeedforward(new ArmFeedforward(0, 0, 0))
          .withSimFeedforward(new ArmFeedforward(0, 0, 0))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          .withMotorInverted(false)
          .withZeroPower(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40))
          .withStartingPosition(Degrees.of(-5))
          .withSoftLimits(Degrees.of(-20), Degrees.of(30))
          .withTelemetry("IntakeArmLeftMotor", TelemetryVerbosity.HIGH)
          // The right motor is sent every position and velocity setpoint the left motor is given.
          .withLooselyCoupledFollowers(rightMotor));

  private final Arm arm = new Arm(new ArmConfig()
      .withHardLimits(Degrees.of(-25), Degrees.of(35))
      .withLength(Meters.of(0.5))
      .withTelemetry("IntakeArm", TelemetryVerbosity.HIGH),
      leftMotor);

  public Command setAngle(Angle angle) {
    // Moves both motors.
    return arm.setAngle(angle);
  }

  /** Call this from Robot.robotPeriodic(), before the scheduler runs. */
  public void periodic() {
    arm.updateTelemetry();
    // The Arm only updates the leader. Update each follower yourself.
    rightMotor.updateTelemetry();
  }

  /** Robot.simulationInit() runs this in its own Notifier, every robot loop. */
  public void simulationPeriodic() {
    // Simulates both motors: the Arm iterates its followers.
    arm.simIterate();
  }
}
```
{% endcode %}

In `Robot.java`, call `intakeArm.periodic()` from `robotPeriodic()` and start `intakeArm::simulationPeriodic` in a `Notifier` from `simulationInit()`, exactly as for any mechanism in [Where the tutorials put things](../../../tutorials/project-setup/#where-the-tutorials-put-things). The imports are the ones in [Step 3 of the Arm tutorial](../../../tutorials/arm/commands.md), plus `Meters`, and the `simple_robot-2027` example has a complete [Commands v3 arm mechanism](https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands3/simple_robot-2027/src/main/java/first/robot/mechanisms/ArmMechanism.java).
{% endtab %}

{% tab title="C++" %}
Create each follower before the leader, and name it in the leader's config:

```cpp
m_rightMotor.emplace(&m_rightSpark, wpi::math::DCMotor::NEO(1), &m_rightConfig);

m_leftConfig
    // ... config options
    .WithLooselyCoupledFollowers({&m_rightMotor.value()});  // the right motor is sent the left motor's setpoints

m_leftMotor.emplace(&m_leftSpark, wpi::math::DCMotor::NEO(1), &m_leftConfig);
```

Call `UpdateTelemetry()` on each follower, as on the arm. The arm's `SimIterate()` simulates every motor.
{% endtab %}
{% endtabs %}

### What reaches the followers

| The leader is given                                                                                                                 | The followers                                                                              |
| ----------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------ |
| A position or velocity setpoint, such as from `arm.setAngle(...)` or `flywheel.run(...)`                                            | Are sent the same setpoint, and each reaches it with its own closed loop                   |
| A change from live tuning: gains, feedforward, motion profile, current limits, ramp rates, soft limits, gearing or closed loop slot | Are given the same change                                                                  |
| A duty cycle of 0, which stops it                                                                                                   | Stop too\*                                                                                 |
| Any other duty cycle or voltage, such as from `arm.set(...)`, `arm.setVoltage(...)` or SysId                                        | **Are not sent it.** The leader moves alone, and the followers are stopped\*               |
| `setEncoderPosition(...)`, such as when zeroing or homing                                                                           | **Are not sent it.** Set each follower's encoder yourself                                  |
| `updateTelemetry()`                                                                                                                 | **Are not updated.** Call it on each follower yourself                                     |
| `simIterate()`, in a mechanism class                                                                                                | Are iterated by the mechanism, right after it steps. Their own `simIterate()` does nothing |
| `simIterate()`, without a mechanism class                                                                                           | **Are not iterated.** Call it on each follower yourself                                    |
| Its starting config                                                                                                                 | **Are not given it.** Each follower only has its own config                                |

\* A follower whose closed loop runs on the SystemCore only stops in YAMS releases after v2026.10.15. See the first pitfall below.

### Common pitfalls

*   **The follower doesn't stop (YAMS v2026.10.15 and earlier).** A follower's closed loop runs on the SystemCore when it is a Thrifty Nova, a SPARK with an exponential profile, or any motor controller with an LQR. In YAMS v2026.10.15 and earlier, stopping the leader doesn't stop that loop, so the follower keeps driving to the last setpoint: on through `arm.set(0)`, and against the leader through `arm.set(...)`, `arm.setVoltage(...)` and SysId. Later releases stop it with the leader. Until you update, stop the follower's closed loop yourself while the leader is driven at a duty cycle or voltage. In Commands v2:

    ```java
    arm.set(dutyCycle)
        .beforeStarting(() -> {
          rightMotor.stopClosedLoopController();
          rightMotor.setDutyCycle(0);
        })
        .finallyDo(rightMotor::startClosedLoopController);
    ```
* **The follower logs nothing.** Call `updateTelemetry()` on each follower, every loop. Without a mechanism class, also call `simIterate()` on it, or it does nothing in simulation.
* **The follower doesn't follow the profile.** The leader sends the follower the final setpoint, so a follower needs the same motion profile as the leader, and the same gearing, gains and feedforward, or it gets there at a different speed.
* **The follower drives the wrong way.** Each motor is inverted in its own config, with `withMotorInverted`. Motors on opposite sides of a mechanism usually face each other, so one of them is inverted.
* **The two sides fight.** Nothing keeps the motors together except the mechanism: each one only follows its own encoder. If they start at different positions, or one side slips, they pull against each other through the mechanism. Give every motor the same starting position, and when you zero or home the mechanism, set every motor's encoder.
* **Manual control moves only one side.** A voltage or duty cycle, such as manual control with a joystick or a SysId routine, only drives the leader, and the followers are stopped. Command each motor yourself, or characterize one side with the other side unpowered. A mechanism class simulates this too: the stopped followers hold the mechanism back.

### Motors that need different setpoints

Loosely coupled followers are sent the leader's setpoints exactly. When the motors need different ones, such as two flywheels spinning at different speeds, command each `SmartMotorController` yourself: see [How do I control a mechanism without a mechanism class?](../../../how-to-mechanisms/how-do-i-control-a-mechanism-without-a-mechanism-class.md) and the [double flywheel tutorial](../../../tutorials/double-flywheel/spin-and-followers.md).

## Code reference

* [`SmartMotorControllerConfig`](https://javadocs.yassrobotics.com/yams/yams/core/motorcontrollers/SmartMotorControllerConfig.html)
