# EasyCRT

**Namespace:** `yams::units`\
**Header:** `#include <yams/units/EasyCRT.hpp>`\
**Java equivalent:** [Java EasyCRT](../../java-reference/units/easy-crt.md)\
**Doxygen:** [`EasyCRT`](https://doxygen.yassrobotics.com/yams/classyams_1_1units_1_1EasyCRT.html)

`EasyCRT` finds the absolute angle of a gear that turns more than once, from two absolute encoders on pinions with different tooth counts that both mesh with it. Each encoder only knows its angle within one of its own rotations, but the two readings together are unique over `GetUniqueRange()`, which is `lcm(encoderATeeth, encoderBTeeth) / sharedGearTeeth` rotations of the shared gear.

There is no config struct. The whole API is one constructor and three methods, the same as Java. `EasyCRT.hpp` is not pulled in by `yams/YAMS.hpp`; include it directly.

***

## Constructor

```cpp
EasyCRT(int sharedGearTeeth, int encoderATeeth, int encoderBTeeth)
```

| Parameter         | Description                                                                                   |
| ----------------- | --------------------------------------------------------------------------------------------- |
| `sharedGearTeeth` | Teeth on the gear both encoder pinions mesh with. The solved angle is the angle of this gear. |
| `encoderATeeth`   | Teeth on the pinion of encoder A.                                                             |
| `encoderBTeeth`   | Teeth on the pinion of encoder B.                                                             |

Throws `std::invalid_argument` if any tooth count is zero or negative.

***

## Methods

```cpp
std::optional<wpi::units::turn_t> Solve(wpi::units::turn_t encoderA,
                                        wpi::units::turn_t encoderB) const;

std::optional<wpi::units::turn_t> Solve(wpi::units::turn_t encoderA, wpi::units::turn_t encoderB,
                                        wpi::units::turn_t minAngle,
                                        wpi::units::turn_t maxAngle) const;

wpi::units::turn_t GetUniqueRange() const;
```

| Method                                                | Returns                             | Description                                                                                                                                                                                                                                                                                                                                             |
| ----------------------------------------------------- | ----------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `Solve(encoderA, encoderB) const`                     | `std::optional<wpi::units::turn_t>` | Solves for the angle of the shared gear, assuming its travel is centered on zero. The result is within half of `GetUniqueRange()` either side of zero. `std::nullopt` if the two readings do not agree on one angle.                                                                                                                                    |
| `Solve(encoderA, encoderB, minAngle, maxAngle) const` | `std::optional<wpi::units::turn_t>` | Solves for the angle of the shared gear when its travel is not centered on zero. `minAngle` and `maxAngle` are the lowest and highest angle the shared gear can be at. `std::nullopt` if the two readings do not agree on one angle. Throws `std::invalid_argument` if the travel is longer than `GetUniqueRange()`, or `maxAngle` is below `minAngle`. |
| `GetUniqueRange() const`                              | `wpi::units::turn_t`                | How far the shared gear can turn before the pair of encoder readings repeats.                                                                                                                                                                                                                                                                           |

***

## Behavior

* **Readings are used as they are.** `EasyCRT` has no offset or inversion settings. Both encoders must be zeroed with the mechanism at zero, and both must read increasing angles when the mechanism moves in its positive direction. Set the offset and inversion on the encoders themselves, or adjust the readings before passing them in.
* **The result is the angle of the shared gear.** If the mechanism is geared to that gear, convert the result yourself.
* **Not a textbook Chinese Remainder Theorem solve.** Every angle consistent with encoder A is tried, and the one whose predicted encoder B reading is closest to the real one wins, which stays stable with backlash and sensor noise.
* **The tolerance comes from the gears.** Neighboring candidates predict encoder B readings `gcd(encoderATeeth, encoderBTeeth) / encoderBTeeth` of a rotation apart. A match is accepted when encoder B is within a quarter of that gap of its prediction.
* **`std::nullopt` instead of a guess.** `Solve` returns `std::nullopt` when the two readings do not agree on one angle.
* **No state.** `EasyCRT` holds no suppliers and keeps nothing between calls. There are no status, error or iteration getters, and the `Crt...` helper functions are gone.

***

## Example

```cpp
#include <optional>

#include <yams/units/EasyCRT.hpp>

// 100 tooth turret gear, 19 tooth pinion on encoder A, 21 tooth pinion on encoder B.
yams::units::EasyCRT crt{100, 19, 21};

wpi::units::turn_t range = crt.GetUniqueRange();  // lcm(19, 21) / 100 = 3.99 turns

// Read once at startup, while the mechanism is not moving.
std::optional<wpi::units::turn_t> turretAngle =
    crt.Solve(encoderA.GetAbsolutePosition().GetValue(),
              encoderB.GetAbsolutePosition().GetValue());

if (turretAngle.has_value()) {
  double turns = turretAngle.value().value();
}

// Travel that is not centered on zero: 0 to 1.5 turns.
auto offCenter = crt.Solve(encoderA.GetAbsolutePosition().GetValue(),
                           encoderB.GetAbsolutePosition().GetValue(),
                           wpi::units::turn_t{0.0}, wpi::units::turn_t{1.5});
```

{% hint style="info" %}
The pinions can mesh with the turret gear, the motor drive gear, their own common gear, or a second gear on the drive shaft. See [Gear layouts](https://yams.yassrobotics.com/documentation/understanding/easycrt#gear-layouts) in the EasyCRT guide for a drawing and an example of each.
{% endhint %}

### Seeding a `SmartMotorController` with `MechanismGearing`

The motor's relative encoder counts rotor rotations, which is the turret angle multiplied by the reduction between the motor and the turret. Put that reduction in a `MechanismGearing`, give it to the `SmartMotorController` config, and `SetEncoderPosition(...)` does the multiplication for you. This is the usual layout when three gears mesh with the turret gear: the two encoder pinions and the motor's pinion.

```cpp
#include <yams/gearing/GearBox.hpp>
#include <yams/gearing/MechanismGearing.hpp>
#include <yams/units/EasyCRT.hpp>

// Three gears mesh with the 200 tooth turret gear: the two encoder pinions and the motor's
// 40 tooth drive gear, which is driven through a 10:1 gearbox.
yams::gearing::MechanismGearing turretGearing{
    yams::gearing::GearBox::FromReductionStages({10.0, 200.0 / 40.0})};
yams::units::EasyCRT crt{200, 19, 21};

// motor is a yams::motorcontrollers::SmartMotorController whose config has
// WithMotorGearing(turretGearing).
if (auto turretAngle = crt.Solve(encoderA, encoderB)) {
  // The turret gear is the shared gear, so the solved angle is already the turret angle.
  // SetEncoderPosition takes the mechanism angle and sets the motor's relative encoder to
  // *turretAngle * turretGearing.GetMechanismToRotorRatio() for you.
  motor.SetEncoderPosition(*turretAngle);

  // The same conversion, if you want the rotor angle yourself (for logging or a sanity check):
  wpi::units::turn_t rotorAngle = *turretAngle * turretGearing.GetMechanismToRotorRatio();
}
```

Pass `SetEncoderPosition(...)` the mechanism angle, not the rotor angle: it applies the `MechanismGearing` from the config, so multiplying first would apply the gearing twice. `*turretAngle * turretGearing.GetMechanismToRotorRatio()` is the rotor angle it ends up setting. If the pinions mesh with a gear that is not the mechanism, convert the solved angle to the mechanism angle first (divide by the ratio between them).

{% hint style="warning" %}
Solve once at startup while the mechanism is still, and handle `std::nullopt`. The two readings are taken one after the other, so any motion between them looks like disagreement. Do not use `EasyCRT` as closed loop feedback.
{% endhint %}

{% hint style="info" %}
Pick pinion tooth counts with no common factor (19 and 21, 17 and 19, 30 and 31), so the unique range is as long as possible. Sharing a factor throws range away: 20 and 30 teeth only cover 60 teeth of the shared gear, not 600.
{% endhint %}
