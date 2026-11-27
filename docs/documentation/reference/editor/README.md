---
icon: gear
layout:
  width: default
  title:
    visible: true
  description:
    visible: true
  tableOfContents:
    visible: true
  outline:
    visible: true
  pagination:
    visible: true
  metadata:
    visible: true
  tags:
    visible: true
  actions:
    visible: true
  anchors:
    visible: true
---

# Smart motor controllers

A `SmartMotorController` is how YAMS drives one motor. You describe the motor once in a `SmartMotorControllerConfig` (its gearing, gains, limits and encoders), and a wrapper such as `TalonFXWrapper`, `SparkWrapper`, `NovaWrapper` or `A301Wrapper` applies it to the vendor's motor controller. The same config works on every supported motor controller.

New to YAMS? Read [How YAMS drives a motor](../../tutorials/how-yams-drives-a-motor.md) first. It explains the three layers (motor controller, mechanism, subsystem), the words rotor, mechanism and measurement, open and closed loop, and the units your gains are in.

## Where to go next

| You want to                                             | Read                                                                                                                            |
| ------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------- |
| Look up what an option does                             | [SmartMotorControllerConfig](config/), one page per group of options                                                            |
| Check whether your motor controller supports an option  | [Motor controller feature matrix](feature-matrix.md)                                                                            |
| Set up followers, external encoders or a vendor config  | [Followers](config/followers.md), [Encoders](config/encoders.md), [`withVendorConfig`](config/motor-basics.md#withvendorconfig) |
| Keep settings already saved on the motor controller     | [`withResetPreviousConfig`](config/motor-basics.md#withresetpreviousconfig)                                                     |
| Use different gains in simulation                       | [Simulation-only PID + feedforward](../../understanding/simulation-only-pid-+-feedforward.md)                                   |
| Stop brownouts                                          | [Limiting power consumption](../../how-to-motor-controllers-and-sensors/limiting-power-consumption.md)                          |
| Choose between a trapezoidal and an exponential profile | [Exponential profiles](../../understanding/exponential-profiles.md#when-to-use-each-profile-type)                               |
| Replace PID with a model based controller               | [LQR controllers](../../understanding/lqr-controllers.md)                                                                       |
| Find your gains                                         | [Live tuning](../../how-to-tuning/live-tuning/)                                                                                 |

{% hint style="warning" %}
Not every option works on every motor controller. See [which options are not supported everywhere](config/#not-every-option-works-on-every-motor-controller).
{% endhint %}

## Reach the vendor objects yourself

YAMS does not have an option for every vendor setting. For a setting it does not have, start from your own vendor config with [`withVendorConfig`](config/motor-basics.md#withvendorconfig). To call the vendor's API directly, fetch the vendor objects from the `SmartMotorController`. Both methods return `Object`, so cast to the class your wrapper holds:

```java
// A SparkWrapper around a SPARK Flex
SparkFlex sparkFlex = (SparkFlex) smartMotorController.getMotorController();
SparkFlexConfig sparkFlexConfig = (SparkFlexConfig) smartMotorController.getMotorControllerConfig();
```

## Motor options stay on the motor config

A mechanism config such as `ArmConfig` never changes the `SmartMotorControllerConfig`. Soft limits, the starting position, gearing and gains are always set on the motor config. [Mechanism configs](../mechanism-configs/) lists what each mechanism config sets.

## Simulation without a mechanism

A mechanism class simulates its motor for you. A `SmartMotorController` used on its own needs `simIterate()` and `updateTelemetry()` called every loop: see [Running the simulation](config/simulation.md#running-the-simulation).

## Learn more about motors

{% embed url="https://youtu.be/0BfR_viZSmQ" %}

{% embed url="https://youtu.be/pIKvQX5p1PE" %}

The presentation from the video series continues onto PID, feedforward and motion profiles.

{% file src="../../.gitbook/assets/Motors.pdf" %}

The WPILib docs explain PID and feedforward with interactive simulations: see [tuning a turret position controller](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-turret.html).
