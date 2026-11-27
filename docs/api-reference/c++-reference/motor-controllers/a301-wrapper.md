# A301Wrapper

**Namespace:** `yams::motorcontrollers::local`\
**Extends:** `SmartMotorController`\
**Header:** `#include <yams/motorcontrollers/local/A301Wrapper.hpp>`\
**Doxygen:** [`A301Wrapper`](https://doxygen.yassrobotics.com/yams/classyams_1_1motorcontrollers_1_1local_1_1A301Wrapper.html)

`SmartMotorController` implementation for the FIRST A301 smart motor, using REVLib 2027.

**Java equivalent:** [Java A301Wrapper](../../java-reference/motor-controllers/a301-wrapper.md)

***

## Constructor

```cpp
A301Wrapper(first::a301::A301* a301, SmartMotorControllerConfig* config)
```

| Parameter | Type                          | Description                                        |
| --------- | ----------------------------- | -------------------------------------------------- |
| `a301`    | `first::a301::A301*`          | A301 hardware object. Must outlive this wrapper.   |
| `config`  | `SmartMotorControllerConfig*` | Configuration to apply. Must outlive this wrapper. |

`first::a301::A301` comes from `<first/A301.h>` (REVLib 2027.0.0-alpha-9) and is created with a `wpi::CANPort` and a CAN ID, for example `first::a301::A301 a301{wpi::CANPort::CAN_S0, 3};`.

There is no motor parameter: every A301 has the same motor, so the wrapper uses `YMotors::A301(n)`, where `n` is this A301 and the A301s following it. See [Followers](a301-wrapper.md#followers).

***

## Motor model

WPILib has no `DCMotor` of the A301, so YAMS has one in `#include <yams/motorcontrollers/YMotors.hpp>`:

```cpp
static constexpr wpi::math::DCMotor yams::motorcontrollers::YMotors::A301(int numMotors = 1);
```

It is the motor inside the A301, before its gearbox, at 12 V: a 13,500 RPM free speed and a 0.078 Nm stall torque (twice the 5.27 Nm at peak power of FIRST's 135:1 gearbox, divided by 135), from [FIRST's announcement](https://www.firstinspires.org/community-blog/introducing-the-first-a301). FIRST does not publish its currents, so it uses the NEO 550's for now: 100 A stalled and 1.4 A free. `YMotors` will be removed once WPILib has a model of the A301.

The A301's peak power is about 28 W, much less than a NEO, so an arm needs a large reduction, such as the 64:1 gearbox and a 3:1 chain (192:1) for an 18 in, 2 lb arm.

***

## Key methods

`A301Wrapper` implements the full [`SmartMotorController`](smart-motor-controller.md) interface (open loop outputs, closed loop setpoints, encoder reads and writes, live configuration setters, telemetry and live tuning). See that page for the complete method list. The notes below cover A301-specific behavior.

### Closed loop control

The A301 has no closed loop gains, so in closed loop mode YAMS always runs the PID, motion profile, feedforward or [LQR](../math/lqr-controller.md) on the SystemCore, in a notifier, and commands the A301 with voltage. Gains are per mechanism rotation, as for any loop on the SystemCore. YAMS raises an alert whenever the A301 is in closed loop mode (see [Alerts](a301-wrapper.md#alerts)).

`WithClosedLoopControlPeriod(...)` (default 20 ms) sets the SystemCore's loop period; it is only valid in closed loop mode.

### Gearing

The gearing given to `WithMotorGearing` is from the A301's rotor to the mechanism, so it includes the gearbox attached to the A301 and any gearing after it. FIRST's A301 gearboxes are 135:1, 64:1, 36:1, 27:1, 13.5:1 and 3:1. YAMS does not detect the gearbox, so enter it yourself.

### Encoder reads

| Method                                                                            | A301 behavior                                                                                                                                                        |
| --------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `GetMechanismPosition()`                                                          | The absolute encoder when `WithUseExternalFeedbackEncoder(true)` is set, otherwise the relative encoder.                                                             |
| `GetMechanismVelocity()`                                                          | The relative encoder's velocity at the mechanism. The A301 does not report its absolute encoder's velocity.                                                          |
| `GetRelativeMechanismPosition()` / `GetRelativeMechanismVelocity()`               | Always the A301's relative encoder, with the motor gearing applied.                                                                                                  |
| `GetExternalEncoderMechanismPosition()` / `GetExternalEncoderMechanismVelocity()` | The absolute encoder's position with the external encoder gearing applied, and the relative encoder's velocity at the mechanism. Empty without the absolute encoder. |
| `GetSupplyCurrent()`                                                              | The sim supplier's value in simulation, empty on the robot. The A301 does not report it.                                                                             |

### Restrictions

The A301 has none of the following, so `ApplyConfig` rejects them:

| Option                                                         | Behavior                                                                                         |
| -------------------------------------------------------------- | ------------------------------------------------------------------------------------------------ |
| `WithStatorCurrentLimit`, `WithSupplyCurrentLimit`             | Throw `SmartMotorControllerConfigurationException`. The A301 has no current limits.              |
| `WithOpenLoopRampRate`, `WithClosedLoopRampRate`               | Throw `SmartMotorControllerConfigurationException`. The A301 has no ramp rates.                  |
| `WithVoltageCompensation`                                      | Throws `SmartMotorControllerConfigurationException`.                                             |
| `WithVendorConfig`                                             | Throws `SmartMotorControllerConfigurationException`. The A301 has no vendor config.              |
| `WithVendorControlRequest`                                     | Throws `SmartMotorControllerConfigurationException`.                                             |
| `WithFollowers` with anything but an `A301Wrapper*`            | Throws `SmartMotorControllerConfigurationException`. See [Followers](a301-wrapper.md#followers). |
| `WithEncoderInverted(true)`                                    | Throws `std::invalid_argument`. The A301's internal encoder cannot be inverted.                  |
| Closed loop tolerance with an [LQR](../math/lqr-controller.md) | Throws `std::invalid_argument`.                                                                  |

The live setters `SetStatorCurrentLimit`, `SetSupplyCurrentLimit`, `SetOpenLoopRampRate`, `SetClosedLoopRampRate` and `SetEncoderInverted` throw `std::runtime_error`. `SetEncoderVelocity` is only supported in simulation, and throws `std::runtime_error` on the robot.

***

## External encoders

The A301 has an absolute encoder on its output shaft, after the gearbox. To use it, pass the A301 itself to `WithExternalEncoder(&a301)`. Any other value throws `std::invalid_argument`.

`WithExternalEncoderGearing` is from the A301's output shaft to the mechanism, so it leaves out the gearbox. The zero offset, inversion and wrapping range are applied by the wrapper. Without `WithWrapping(min, max)` the encoder reads `[0, 1)`, like the SPARK and Nova (see [SmartMotorControllerConfig](smart-motor-controller-config.md)).

`WithExternalEncoderZeroOffset`, `WithExternalEncoderInverted` and `WithExternalEncoderGearing` all require `WithExternalEncoder(...)`; setting them without one throws `SmartMotorControllerConfigurationException`.

When no starting position is configured and the absolute encoder is attached, the relative encoder is seeded from it when the config is applied.

***

## Followers

The A301 has no follower mode, so the wrapper follows in software. Wrap each other A301 on the mechanism in its own `A301Wrapper`, and give them to the main A301's `WithFollowers` as `(A301Wrapper*, inverted)` pairs. Only an `A301Wrapper` can follow an A301; anything else throws `SmartMotorControllerConfigurationException`.

```cpp
// The right side of the arm: create it before the main A301, and destroy it after.
m_rightConfig.WithMotorGearing(yams::gearing::MechanismGearing{64.0 * 3.0});
m_rightMotor.emplace(&m_rightA301, &m_rightConfig);

// The main A301. true: the right A301 faces it across the arm.
m_leftConfig.WithMotorGearing(yams::gearing::MechanismGearing{64.0 * 3.0})
    .WithFeedback(30.0, 0.0, 0.0)
    .WithMotorInverted(false)
    .WithFollowers({{std::any{&m_rightMotor.value()}, true}});
m_leftMotor.emplace(&m_leftA301, &m_leftConfig);
```

* The main A301 sends each follower its voltage and duty cycle, inverted where it is, and the follower's own closed loop is stopped. Command and tune the main A301 only.
* The main A301's `GetDCMotor()` is `YMotors::A301(n)` with every A301, so its simulation and a mechanism's, such as an `Arm`'s, have the torque of all of them.
* In simulation a follower reads the main A301's simulation, with its share of the current, and does not draw current from [`BatterySim`](../sensor/battery-sim.md) again. The main A301's `SimIterate()` iterates its followers, so do not call theirs.
* `WithLooselyCoupledFollowers` still works as on any motor controller, for an A301 with its own closed loop.

***

## Simulation

REVLib does not simulate the A301, so the wrapper creates a DC motor simulation of the A301's motor from `YMotors::A301(n)`, this A301 and its followers, the configured MOI and the motor gearing, and reports its current draw to [`BatterySim`](../sensor/battery-sim.md) every loop. A mechanism's own sim, such as an `Arm`'s, replaces it as usual. In simulation the absolute encoder follows the simulated mechanism. Set a realistic moment of inertia with `SmartMotorControllerConfig::WithMOI(...)` for realistic current draw.

***

## Telemetry

The A301 does not report these fields, so `GetUnsupportedTelemetryFields()` lists them and they are not published: `SupplyCurrent`, `SupplyCurrentLimit`, `StatorCurrentLimit`, `ClosedloopRampRate` and `OpenloopRampRate`.

***

## Alerts

Alerts are published in the `YAMS` alert group and cleared when the wrapper is destroyed.

| Alert                                                         | When                                                                            |
| ------------------------------------------------------------- | ------------------------------------------------------------------------------- |
| `<name> closed loop controller is running on the SystemCore.` | The A301 is in closed loop mode. Its closed loop always runs on the SystemCore. |

***

## Example

```cpp
#include <first/A301.h>
#include <wpi/commands2/SubsystemBase.hpp>
#include <yams/motorcontrollers/SmartMotorControllerConfig.hpp>
#include <yams/motorcontrollers/local/A301Wrapper.hpp>

using namespace yams::motorcontrollers;
using namespace yams::motorcontrollers::local;

class ArmSubsystem : public wpi::cmd::SubsystemBase {
 public:
  ArmSubsystem() {
    // From the A301's rotor to the arm: its 64:1 gearbox.
    m_motorConfig.WithSubsystem(this)
        .WithMotorGearing(yams::gearing::MechanismGearing{64.0})
        .WithFeedback(1.0, 0.0, 0.0)
        .WithMotorInverted(false)
        .WithExternalEncoder(&m_a301)  // the A301's absolute encoder, on its output shaft
        .WithUseExternalFeedbackEncoder(true)
        .WithClosedLoopMode()
        .WithTelemetry("ArmMotor", SmartMotorControllerConfig::TelemetryVerbosity::HIGH);

    m_motor.emplace(&m_a301, &m_motorConfig);
  }

 private:
  first::a301::A301 m_a301{wpi::CANPort::CAN_S0, 3};
  SmartMotorControllerConfig m_motorConfig;  // must outlive the wrapper
  std::optional<A301Wrapper> m_motor;
};
```

{% hint style="info" %}
`TelemetryVerbosity::HIGH` together with `WithSubsystem(this)` automatically registers the `Live Tuning` command for the subsystem. See [SmartMotorController](smart-motor-controller.md#telemetry-and-live-tuning).
{% endhint %}

***

## See also

* [SmartMotorController](smart-motor-controller.md)
* [SmartMotorControllerConfig](smart-motor-controller-config.md)
* [SparkWrapper](spark-wrapper.md)
* [NovaWrapper](nova-wrapper.md)
* [TalonFXWrapper / TalonFXSWrapper](talonfx-wrapper.md)
* [BatterySim](../sensor/battery-sim.md)
* [Java A301Wrapper](../../java-reference/motor-controllers/a301-wrapper.md)
