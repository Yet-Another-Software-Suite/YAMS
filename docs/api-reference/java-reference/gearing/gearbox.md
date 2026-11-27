# GearBox

**Package:** `yams.core.gearing`\
**Javadoc:** [`GearBox`](https://javadocs.yassrobotics.com/yams/yams/core/gearing/GearBox.html)

`GearBox` models a multi-stage gear reduction. Each stage reduces or increases speed by a fixed ratio. Multiple stages are combined by multiplication.

A "reduction" in YAMS means motor rotations per one mechanism rotation. A 5:1 reduction means the motor spins 5 times for each mechanism rotation.

***

## Constructors

```java
GearBox(double[] reductionStage)
GearBox(String[] reductionStage)   // "IN:OUT" format per stage
```

***

## Factory methods

| Method                                                          | Description                                                                                                               |
| --------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------- |
| `static GearBox fromReductionStages(double... reductionStages)` | Creates a `GearBox` from one or more numeric ratios. Each ratio is motor-rotations per mechanism-rotation for that stage. |
| `static GearBox fromStages(String... stages)`                   | Creates from stage strings in `"IN:OUT"` format, where `IN` is the driving gear tooth count and `OUT` is the driven.      |
| `static GearBox fromTeeth(int... teeth)`                        | Creates from alternating tooth counts: `driver1, driven1, driver2, driven2, ...`. Each pair is one stage.                 |

***

## Methods

| Method                               | Returns   | Description                                                                             |
| ------------------------------------ | --------- | --------------------------------------------------------------------------------------- |
| `getInputToOutputConversionFactor()` | `double`  | Returns `OUT/IN`: mechanism rotations per motor rotation. For a 5:1 box, returns `0.2`. |
| `getOutputToInputConversionFactor()` | `double`  | Returns `IN/OUT`: motor rotations per mechanism rotation. For a 5:1 box, returns `5.0`. |
| `times(double x)`                    | `GearBox` | Multiplies the reduction ratio by `x`. Returns the same `GearBox` for chaining.         |
| `div(double x)`                      | `GearBox` | Divides the reduction ratio by `x`. Returns the same `GearBox` for chaining.            |

***

## Expressing the same reduction three ways

All three of the following produce an equivalent 5.14:1 reduction:

```java
// From explicit reduction ratio
GearBox g1 = GearBox.fromReductionStages(5.142857);

// From "IN:OUT" stage strings
GearBox g2 = GearBox.fromStages("12:62");

// From tooth counts (driver then driven for each stage)
GearBox g3 = GearBox.fromTeeth(12, 62);
```

For a two-stage gearbox (12T → 48T, then 16T → 64T):

```java
GearBox twoStage = GearBox.fromTeeth(12, 48, 16, 64);
// Equivalent to: (48/12) × (64/16) = 4.0 × 4.0 = 16:1
```

***

## See also

* [MechanismGearing](mechanism-gearing.md)
* [Sprocket](sprocket.md)
* [C++ GearBox](../../c++-reference/gearing/gearbox.md)
