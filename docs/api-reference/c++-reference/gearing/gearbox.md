# GearBox

**Namespace:** `yams::gearing`\
**Header:** `#include <yams/gearing/GearBox.hpp>`\
**Java equivalent:** [Java GearBox](../../java-reference/gearing/gearbox.md)\
**Doxygen:** [`GearBox`](https://doxygen.yassrobotics.com/yams/classyams_1_1gearing_1_1GearBox.html)

`GearBox` models a multi-stage gear reduction. Semantically represents meshed gears. Use `Sprocket` for chain or belt drives.

***

## Constructors

```cpp
explicit GearBox(double reductionStage)
explicit GearBox(std::initializer_list<double> reductionStages)
explicit GearBox(const std::vector<double>& reductionStages)
explicit GearBox(const std::vector<std::string>& stages)  // "IN:OUT" format
```

Each element of `reductionStages` is the reduction for one stage (for example `3.0` for a 3:1 stage); the stages are multiplied together. Strings must be in `"IN:OUT"` format and are parsed as `IN / OUT`, so `"12:1"` is a 12:1 reduction.

***

## Factory methods

```cpp
static GearBox FromReductionStages(std::initializer_list<double> stages)
static GearBox FromStages(std::initializer_list<std::string_view> stages)  // "IN:OUT"
static GearBox FromTeeth(std::initializer_list<int> teeth)  // alternating driver, driven
```

`FromTeeth` takes the tooth counts of a gear train in order, starting with the driver. Each consecutive pair contributes `next / previous`, so `{12, 62}` is a 62:12 (about 5.17:1) reduction. Because consecutive pairs are chained, intermediate gears cancel out (`{12, 48, 16, 64}` evaluates to 64:12, not 16:1). For compound gearboxes with gears sharing a shaft, use `FromReductionStages` or `FromStages` with one entry per stage.

***

## Methods

| Method                                     | Returns    | Description                                                                                                           |
| ------------------------------------------ | ---------- | --------------------------------------------------------------------------------------------------------------------- |
| `GetInputToOutputConversionFactor() const` | `double`   | Mechanism rotations per motor rotation (less than 1 for a reduction)                                                  |
| `GetOutputToInputConversionFactor() const` | `double`   | Motor rotations per mechanism rotation (the reduction ratio itself)                                                   |
| `Times(double x)`                          | `GearBox&` | Multiplies the input-to-output conversion factor by `x` (divides the reduction by `x`). Returns `*this` for chaining. |
| `Div(double x)`                            | `GearBox&` | Divides the input-to-output conversion factor by `x` (multiplies the reduction by `x`). Returns `*this` for chaining. |

***

## Exceptions

| Exception                                | Thrown When                                                                   |
| ---------------------------------------- | ----------------------------------------------------------------------------- |
| `exceptions::NoStagesGivenException`     | Stage list is empty                                                           |
| `exceptions::InvalidStageGivenException` | A string stage is not in `"IN:OUT"` format                                    |
| `std::invalid_argument`                  | `FromTeeth` gets fewer than two tooth counts, or a count that is not positive |

***

## Example

```cpp
using namespace yams::gearing;

// All three equivalent: 12T driver, 62T driven, about 5.17:1
auto g1 = GearBox::FromTeeth({12, 62});
auto g2 = GearBox::FromStages({"62:12"});
auto g3 = GearBox::FromReductionStages({62.0 / 12.0});

// Two-stage compound gearbox: 3:1 then 4:1 = 12:1
auto g4 = GearBox::FromReductionStages({3.0, 4.0});
auto g5 = GearBox::FromStages({"3:1", "4:1"});
```

{% hint style="info" %}
`GetInputToOutputConversionFactor()` returns mechanism rotations per motor rotation (less than 1 for reductions). `GetOutputToInputConversionFactor()` returns the reduction ratio (greater than 1 for reductions). Pass `GearBox` to `MechanismGearing` for use with `SmartMotorControllerConfig`.
{% endhint %}
