# Sprocket

**Package:** `yams.core.gearing`\
**Javadoc:** [`Sprocket`](https://javadocs.yassrobotics.com/yams/yams/core/gearing/Sprocket.html)

`Sprocket` models a chain or belt drive stage. The API is identical to `GearBox`. The distinction is semantic, reflecting the physical mechanism (toothed sprockets and chain vs. meshed gears).

***

## Constructors

```java
Sprocket(double... sprocketReductionStage)
Sprocket(String[] reductionStage)   // "IN:OUT" format
```

***

## Factory methods

| Method                                         | Description                                           |
| ---------------------------------------------- | ----------------------------------------------------- |
| `static Sprocket fromStages(String... stages)` | Creates from `"IN:OUT"` sprocket tooth count strings. |

***

## Methods

| Method                               | Returns    | Description                                                            |
| ------------------------------------ | ---------- | ---------------------------------------------------------------------- |
| `getInputToOutputConversionFactor()` | `double`   | Mechanism rotations per motor rotation (`OUT/IN`).                     |
| `getOutputToInputConversionFactor()` | `double`   | Motor rotations per mechanism rotation (`IN/OUT`).                     |
| `times(double x)`                    | `Sprocket` | Multiplies reduction by `x`. Returns the same `Sprocket` for chaining. |
| `div(double x)`                      | `Sprocket` | Divides reduction by `x`. Returns the same `Sprocket` for chaining.    |

***

## Example

```java
// 18T driving sprocket, 36T driven sprocket: 2:1 reduction
Sprocket s = Sprocket.fromStages("18:36");
```

***

## See also

* [GearBox](gearbox.md)
* [MechanismGearing](mechanism-gearing.md)
