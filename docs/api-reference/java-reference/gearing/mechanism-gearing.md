# MechanismGearing

**Package:** `yams.core.gearing`\
**Javadoc:** [`MechanismGearing`](https://javadocs.yassrobotics.com/yams/yams/core/gearing/MechanismGearing.html)

`MechanismGearing` is the type accepted by `SmartMotorControllerConfig.withGearing()`. It combines a `GearBox` and an optional `Sprocket` into a single ratio describing the motor-to-mechanism reduction.

The constant `MechanismGearing.kOne` represents a 1:1 (direct drive) gearing.

***

## Constructors

```java
MechanismGearing(double reductionRatio)           // Single-value reduction
MechanismGearing(double... reductionRatios)        // Multi-stage, multiplied together
MechanismGearing(GearBox gearBox)
MechanismGearing(GearBox gearBox, Sprocket sprockets)
```

***

## Constants

| Constant                | Description                 |
| ----------------------- | --------------------------- |
| `MechanismGearing.kOne` | 1:1 gearing (direct drive). |

***

## Methods

| Method                       | Returns            | Description                                                                                   |
| ---------------------------- | ------------------ | --------------------------------------------------------------------------------------------- |
| `getRotorToMechanismRatio()` | `double`           | Motor rotations per mechanism rotation (the reduction ratio). For a 10:1 box, returns `10.0`. |
| `getMechanismToRotorRatio()` | `double`           | Mechanism rotations per motor rotation. For a 10:1 box, returns `0.1`.                        |
| `div(double i)`              | `MechanismGearing` | Divides the gearbox portion by `i`.                                                           |

***

## Usage

```java
// A gearbox with a chain stage
MechanismGearing gearing = new MechanismGearing(
    GearBox.fromTeeth(14, 72),      // 5.14:1 gear reduction
    Sprocket.fromStages("18:24")    // 1.33:1 chain reduction
);
// Total: 5.14 × 1.33 ≈ 6.85:1

SmartMotorControllerConfig config = new SmartMotorControllerConfig()
    .withGearing(gearing);
```

{% hint style="info" %}
`getRotorToMechanismRatio()` is what YAMS passes to WPILib's encoder conversion factor. A 10:1 reduction means 10 motor rotations per mechanism rotation, so this method returns `10.0`.
{% endhint %}

***

## See also

* [GearBox](gearbox.md)
* [Sprocket](sprocket.md)
* [SmartMotorControllerConfig](../motor-controllers/smart-motor-controller-config.md)
* [C++ MechanismGearing](../../c++-reference/gearing/mechanism-gearing.md)
