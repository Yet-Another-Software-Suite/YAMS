# EasyCRT Guide

`EasyCRT` finds the absolute angle of a mechanism that turns more than once, usually a turret, from
two absolute encoders. Each encoder sits on a pinion with a different tooth count, and both pinions
mesh with the same gear. Neither encoder knows the angle on its own, but the pair of readings is
unique over a much longer travel than either one.

```java
// 100 tooth turret gear, 19 tooth pinion on encoder A, 21 tooth pinion on encoder B.
EasyCRT crt = new EasyCRT(100, 19, 21);

// Read once at startup, while the turret is not moving.
Optional<Angle> turretAngle = crt.solve(encoderA.get(), encoderB.get());
```

```cpp
yams::units::EasyCRT crt{100, 19, 21};
std::optional<wpi::units::turn_t> turretAngle = crt.Solve(encoderA, encoderB);
```

That is the whole API:

| Java | C++ | What it does |
| ---- | --- | ------------ |
| `new EasyCRT(shared, a, b)` | `EasyCRT{shared, a, b}` | Tooth counts of the shared gear and the two encoder pinions. |
| `solve(a, b)` | `Solve(a, b)` | Angle of the shared gear, for travel centered on zero. |
| `solve(a, b, min, max)` | `Solve(a, b, min, max)` | The same, for travel that is not centered on zero. |
| `getUniqueRange()` | `GetUniqueRange()` | How far the shared gear can turn before the readings repeat. |

`solve` returns an empty `Optional` when the two readings do not agree on one angle.

Everything else you might want (choosing gears, zeroing, inversion, gear trains, checking your
margin) is something you do yourself, and the rest of this guide shows how.

## How the solver works

1. Encoder A's reading fits several shared gear angles, one for each number of whole turns the
   pinion could have made. `EasyCRT` lists every one inside the travel.
2. For each candidate it predicts what encoder B should read.
3. The candidate whose prediction is closest to encoder B's real reading wins.

This is not a textbook Chinese Remainder Theorem solve. It is the same idea done as a search, which
keeps working with backlash and sensor noise. Created by team 6911.

## Before you call solve

`EasyCRT` takes the two readings as they are, so the encoders have to be set up first:

- **Zero both encoders with the mechanism at zero.** Use the vendor tool to set each encoder's
  offset. If an encoder cannot store an offset, subtract it yourself:
  `crt.solve(encoderA.get().minus(offsetA), encoderB.get().minus(offsetB))`.
- **Both encoders must count up when the mechanism moves in its positive direction.** A pinion
  turns the opposite way to the gear it meshes with, so this usually means inverting both encoders.
  Set the direction on the encoder, or negate the reading before passing it in.
- **Read while the mechanism is still.** The two readings are taken one after the other, so any
  motion between them looks like disagreement. Solve once at startup and seed the motor with the
  result. Do not use `EasyCRT` as closed loop feedback.

## Choosing tooth counts

The readings repeat together every `lcm(a, b)` teeth of the shared gear, so:

```
unique range (rotations of the shared gear) = lcm(a, b) / shared
```

Pick pinions with no common factor, so the lcm is their product. Sharing a factor throws range away:
20 and 30 teeth only cover 60 teeth of the shared gear, not 600.

| Pinion A | Pinion B | Teeth covered | Range on a 100 tooth gear | Range on a 200 tooth gear |
| -------- | -------- | ------------- | ------------------------- | ------------------------- |
| 13 | 15 | 195 | 1.95 turns | 0.975 turns |
| 17 | 19 | 323 | 3.23 turns | 1.615 turns |
| 19 | 21 | 399 | 3.99 turns | 1.995 turns |
| 20 | 21 | 420 | 4.20 turns | 2.10 turns |
| 25 | 27 | 675 | 6.75 turns | 3.375 turns |
| 30 | 31 | 930 | 9.30 turns | 4.65 turns |

The range has to be at least your full travel, including how far the mechanism can coast past its
soft limits. `getUniqueRange()` returns it, and `solve(a, b, min, max)` throws if the travel you pass
is longer.

To check a pair before you build it:

```java
Angle range = new EasyCRT(200, 19, 21).getUniqueRange(); // 1.995 rotations
```

## Travel that is not centered on zero

`solve(a, b)` assumes the mechanism's zero is in the middle of its travel, and returns an angle
within half the unique range either side of zero. For a turret that goes from 0 to 540 degrees, give
the travel:

```java
Optional<Angle> angle = crt.solve(encoderA.get(), encoderB.get(), Degrees.of(0), Degrees.of(540));
```

Give the real travel, not the whole unique range. A tighter range leaves fewer candidates, and it
keeps a reading taken slightly past an end stop from wrapping to the other end.

## Gear trains

The solved angle is the angle of the gear both pinions mesh with. If that gear is the mechanism,
you are done. If the mechanism is geared to it, convert the result yourself.

Example: the pinions mesh with a 50 tooth drive gear that turns 12 times per turret rotation.

```java
EasyCRT crt = new EasyCRT(50, 19, 23);
Optional<Angle> turretAngle = crt.solve(encoderA.get(), encoderB.get()).map(driveGear -> driveGear.div(12));
```

The unique range shrinks by the same ratio: `lcm(19, 23) / 50 = 8.74` turns of the drive gear is
`8.74 / 12 = 0.73` turns of the turret.

If the encoders are driven through their own extra stages, work out how many teeth of the shared
gear pass per encoder rotation and use that as the pinion tooth count.

## Margin: how much backlash is too much

`EasyCRT` has no tolerance setting. It works the tolerance out from the gears:

- Neighboring candidates predict encoder B readings `gcd(a, b) / b` of a rotation apart. For 19 and
  21 teeth that is 1/21 of a rotation, about 17 degrees of encoder B.
- A match is accepted when encoder B is within a quarter of that gap of its prediction, about 4.3
  degrees here. Anything further off returns empty instead of a guess.

So the backlash and noise between the two encoders, measured at encoder B, has to stay under a
quarter of the gap. To measure what you actually have, log the error yourself while moving the
mechanism slowly by hand through its travel:

```java
// Encoder B rotations per rotation of the shared gear is shared / b.
double predictedB = solvedAngle.in(Rotations) * 100.0 / 21.0;
double error = MathUtil.inputModulus(predictedB - encoderB.get().in(Rotations), -0.5, 0.5);
```

If the error gets near the limit:

- Preload the mechanism to one side of its backlash before reading.
- Use a larger pinion B, or pinions with a bigger tooth difference, which widens the gap.
- Check that both encoder magnets and pinions are secure.

The accuracy of the result comes from encoder A alone: an error on encoder A shows up divided by
`shared / a`.

## Seeding a motor controller

Use the solved angle to tell the motor where the mechanism is, once, at startup:

```java
// motor is a YAMS SmartMotorController. Give it the mechanism angle; YAMS applies the gearing.
crt.solve(encoderA.get(), encoderB.get()).ifPresent(motor::setEncoderPosition);
```

To set the motor's own relative encoder yourself, convert the turret angle to the angle of the motor's rotor with the `MechanismGearing` between the motor and the turret. `getMechanismToRotorRatio()` is the reduction, so multiply the solved angle by it with `.times(...)`. This is the usual layout when three gears mesh with the turret gear: the two encoder pinions and the motor's pinion.

```java
// Three gears mesh with the 200 tooth turret gear: the two encoder pinions and the motor's
// 20 tooth pinion, which is driven through a 10:1 gearbox.
MechanismGearing turretGearing = new MechanismGearing(GearBox.fromReductionStages(10, 200.0 / 20));
EasyCRT crt = new EasyCRT(200, 19, 21);

crt.solve(encoderA.get(), encoderB.get()).ifPresent(turretAngle -> {
  // The turret gear is the shared gear, so the solved angle is already the turret angle.
  // Turret angle times the reduction is the angle of the motor's rotor.
  Angle rotorAngle = turretAngle.times(turretGearing.getMechanismToRotorRatio());

  sparkMax.getEncoder().setPosition(rotorAngle.in(Rotations)); // REV
  // talonFX.setPosition(rotorAngle);                           // CTRE
});
```

```cpp
// Three gears mesh with the 200 tooth turret gear: the two encoder pinions and the motor's
// 20 tooth pinion, which is driven through a 10:1 gearbox.
yams::gearing::MechanismGearing turretGearing{
    yams::gearing::GearBox::FromReductionStages({10.0, 200.0 / 20.0})};
yams::units::EasyCRT crt{200, 19, 21};

if (auto turretAngle = crt.Solve(encoderA, encoderB)) {
  // The turret gear is the shared gear, so the solved angle is already the turret angle.
  // Turret angle times the reduction is the angle of the motor's rotor.
  wpi::units::turn_t rotorAngle = *turretAngle * turretGearing.GetMechanismToRotorRatio();

  talonFX.SetPosition(rotorAngle);
}
```

Use the same `MechanismGearing` you pass to `SmartMotorControllerConfig.withGearing(...)`, so the seeded position and the closed loop agree. If the pinions mesh with a gear that is not the mechanism, convert the solved angle to the mechanism angle first (see Gear trains), then multiply.

Decide what happens when the result is empty. Raising an alert and refusing to move the mechanism is
safer than assuming it is at zero. The `BroncBotz_2026` examples under `examples/commands2` and
`examples/commands3` have a complete `Turret` class.

## Troubleshooting

| Symptom | Likely cause |
| ------- | ------------ |
| Always empty | An encoder is not zeroed at the mechanism's zero, or one encoder counts the wrong way. |
| Empty in some positions only | Backlash or a loose pinion is using up the margin. See the margin section. |
| Empty at the ends of travel | The mechanism is past the travel you gave, or the travel is the whole unique range. |
| Angle is off by a fixed amount | Encoder A's zero is off. The result follows encoder A. |
| Angle is off by a whole number of pinion turns | The readings were taken while moving, or encoder B's zero is off. |
| Angle is mirrored | Both encoders count the wrong way. |
| `solve(a, b, min, max)` throws | The travel is longer than the unique range. Choose different pinions. |

## Checklist

- [ ] Pinion tooth counts share no factor, and the unique range covers the full travel with room to spare.
- [ ] Both encoders read zero with the mechanism at zero.
- [ ] Both encoders count up when the mechanism moves in its positive direction.
- [ ] The solved angle matches the real angle at several positions across the travel.
- [ ] The error on encoder B stays well under a quarter of the gap between candidates.
- [ ] The robot gives the right angle after a power cycle in several positions.
- [ ] The code handles an empty result.
