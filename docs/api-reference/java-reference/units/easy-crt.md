# EasyCRT

Package: `yams.core.units`\
Javadoc: [`EasyCRT`](https://javadocs.yassrobotics.com/yams/yams/core/units/EasyCRT.html)

`EasyCRT` finds the absolute angle of a gear that turns more than once, from two absolute encoders on pinions with different tooth counts that both mesh with it. Each encoder only knows its angle within one of its own rotations, but the two readings together are unique over `getUniqueRange()`, which is `lcm(encoderATeeth, encoderBTeeth) / sharedGearTeeth` rotations of the shared gear.

There is no config class. The whole API is one constructor and three methods.

***

## When to use

Use `EasyCRT` when:

* Your mechanism travels more than one full revolution (usually a turret)
* You need absolute position at startup without homing
* You have two absolute encoders on pinions with different tooth counts, both meshing with the same gear

***

## Constructor

```java
EasyCRT(int sharedGearTeeth, int encoderATeeth, int encoderBTeeth)
```

| Parameter         | Description                                                                                   |
| ----------------- | --------------------------------------------------------------------------------------------- |
| `sharedGearTeeth` | Teeth on the gear both encoder pinions mesh with. The solved angle is the angle of this gear. |
| `encoderATeeth`   | Teeth on encoder A's pinion.                                                                  |
| `encoderBTeeth`   | Teeth on encoder B's pinion.                                                                  |

Throws `IllegalArgumentException` if any tooth count is zero or negative.

***

## Methods

| Method                                                                  | Returns           | Description                                                                                                                                                                                                                                                                                                                                       |
| ----------------------------------------------------------------------- | ----------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `solve(Angle encoderA, Angle encoderB)`                                 | `Optional<Angle>` | Solves for the angle of the shared gear, assuming its travel is centered on zero. The result is within half of `getUniqueRange()` either side of zero. Empty if the two readings do not agree on one angle.                                                                                                                                       |
| `solve(Angle encoderA, Angle encoderB, Angle minAngle, Angle maxAngle)` | `Optional<Angle>` | Solves for the angle of the shared gear when its travel is not centered on zero. `minAngle` and `maxAngle` are the lowest and highest angle the shared gear can be at. Empty if the two readings do not agree on one angle. Throws `IllegalArgumentException` if the travel is longer than `getUniqueRange()`, or `maxAngle` is below `minAngle`. |
| `getUniqueRange()`                                                      | `Angle`           | How far the shared gear can turn before the pair of encoder readings repeats.                                                                                                                                                                                                                                                                     |

***

## Behavior

* **Readings are used as they are.** `EasyCRT` has no offset or inversion settings. Both encoders must be zeroed with the mechanism at zero, and both must read increasing angles when the mechanism moves in its positive direction. Set the offset and inversion on the encoders themselves, or adjust the readings before passing them in.
* **Readings are wrapped for you.** Each reading is wrapped into one rotation before solving, so signed or multi-turn readings are fine.
* **The result is the angle of the shared gear.** If the mechanism is geared to that gear, convert the result yourself.
* **Not a textbook Chinese Remainder Theorem solve.** Every angle consistent with encoder A is tried, and the one whose predicted encoder B reading is closest to the real one wins, which stays stable with backlash and sensor noise.
* **The tolerance comes from the gears.** Neighboring candidates predict encoder B readings `gcd(encoderATeeth, encoderBTeeth) / encoderBTeeth` of a rotation apart. A match is accepted when encoder B is within a quarter of that gap of its prediction.
* **Empty instead of a guess.** `solve` returns an empty `Optional` when no candidate is within the tolerance, when two candidates are (which happens at the very ends of the unique range), or when a reading is not finite.
* **No state.** `EasyCRT` holds no suppliers and keeps nothing between calls. There are no status, error or iteration getters.

***

## Example

```java
// 100 tooth turret gear, 19 tooth pinion on encoder A, 21 tooth pinion on encoder B.
EasyCRT crt = new EasyCRT(100, 19, 21);

Angle range = crt.getUniqueRange(); // lcm(19, 21) / 100 = 3.99 rotations

// Read once at startup, while the mechanism is not moving.
Optional<Angle> turretAngle = crt.solve(encoderA.get(), encoderB.get());

// Travel that is not centered on zero.
Optional<Angle> offCenter = crt.solve(encoderA.get(), encoderB.get(), Degrees.of(0), Degrees.of(540));

// Seed a SmartMotorController with the result. It takes the mechanism angle.
turretAngle.ifPresent(motor::setEncoderPosition);
```

{% hint style="info" %}
The pinions can mesh with the turret gear, the motor drive gear, their own common gear, or a second gear on the drive shaft. See [Gear layouts](https://yams.yassrobotics.com/documentation/understanding/easycrt#gear-layouts) in the EasyCRT guide for a drawing and an example of each.
{% endhint %}

### Seeding a `SmartMotorController` with `MechanismGearing`

The motor's relative encoder counts rotor rotations, which is the turret angle multiplied by the reduction between the motor and the turret. Put that reduction in a `MechanismGearing`, give it to the `SmartMotorController` config, and `setEncoderPosition(...)` does the multiplication for you. This is the usual layout when three gears mesh with the turret gear: the two encoder pinions and the motor's pinion.

```java
// Three gears mesh with the 200 tooth turret gear: the two encoder pinions and the motor's
// 40 tooth drive gear, which is driven through a 10:1 gearbox.
MechanismGearing turretGearing = new MechanismGearing(GearBox.fromReductionStages(10, 200.0 / 40));
EasyCRT crt = new EasyCRT(200, 19, 21);

SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
    .withGearing(turretGearing)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(20, 0, 0);
SmartMotorController motor = new SparkWrapper(sparkMax, DCMotor.getNEO(1), motorConfig);

crt.solve(encoderA.get(), encoderB.get()).ifPresent(turretAngle -> {
  // The turret gear is the shared gear, so the solved angle is already the turret angle.
  // setEncoderPosition takes the mechanism angle and sets the motor's relative encoder to
  // turretAngle.times(turretGearing.getMechanismToRotorRatio()) for you.
  motor.setEncoderPosition(turretAngle);

  // The same conversion, if you want the rotor angle yourself (for logging or a sanity check):
  Angle rotorAngle = turretAngle.times(turretGearing.getMechanismToRotorRatio());
});
```

Pass `setEncoderPosition(...)` the mechanism angle, not the rotor angle: it applies the `MechanismGearing` from the config, so multiplying first would apply the gearing twice. `turretAngle.times(turretGearing.getMechanismToRotorRatio())` is the rotor angle it ends up setting. If the pinions mesh with a gear that is not the mechanism, convert the solved angle to the mechanism angle first (divide by the ratio between them).

{% hint style="warning" %}
Solve once at startup while the mechanism is still, and handle an empty result. The two readings are taken one after the other, so any motion between them looks like disagreement. Do not use `EasyCRT` as closed loop feedback.
{% endhint %}

{% hint style="info" %}
Pick pinion tooth counts with no common factor (19 and 21, 17 and 19, 30 and 31), so the unique range is as long as possible. Sharing a factor throws range away: 20 and 30 teeth only cover 60 teeth of the shared gear, not 600.
{% endhint %}

***

See also: [C++ EasyCRT](../../c++-reference/units/easy-crt.md)
