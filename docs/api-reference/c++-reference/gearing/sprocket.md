# Sprocket

**Namespace:** `yams::gearing`\
**Header:** `#include <yams/gearing/Sprocket.hpp>`\
**Java equivalent:** [Java Sprocket](../../java-reference/gearing/sprocket.md)\
**Doxygen:** [`Sprocket`](https://doxygen.yassrobotics.com/yams/classyams_1_1gearing_1_1Sprocket.html)

`Sprocket` models a chain or belt drive stage. The API is identical to `GearBox`; the distinction is semantic. Use `GearBox` for meshed gears.

***

## Constructors

```cpp
explicit Sprocket(double reductionStage)
explicit Sprocket(std::initializer_list<double> reductionStages)
explicit Sprocket(const std::vector<double>& reductionStages)
explicit Sprocket(const std::vector<std::string>& stages)  // "IN:OUT" format
```

***

## Factory methods

```cpp
static Sprocket FromStages(std::initializer_list<std::string_view> stages)  // "IN:OUT"
```

{% hint style="info" %}
`Sprocket` does not provide `FromTeeth`. Use `FromStages` with `"IN:OUT"` strings, parsed as `IN / OUT` reductions (e.g., `"36:18"` for an 18T driving sprocket and a 36T driven sprocket, a 2:1 reduction).
{% endhint %}

***

## Methods

| Method                                     | Returns     | Description                                                                                                           |
| ------------------------------------------ | ----------- | --------------------------------------------------------------------------------------------------------------------- |
| `GetInputToOutputConversionFactor() const` | `double`    | Mechanism rotations per motor rotation (less than 1 for a reduction)                                                  |
| `GetOutputToInputConversionFactor() const` | `double`    | Motor rotations per mechanism rotation (the reduction ratio itself)                                                   |
| `Times(double x)`                          | `Sprocket&` | Multiplies the input-to-output conversion factor by `x` (divides the reduction by `x`). Returns `*this` for chaining. |
| `Div(double x)`                            | `Sprocket&` | Divides the input-to-output conversion factor by `x` (multiplies the reduction by `x`). Returns `*this` for chaining. |

***

## Exceptions

| Exception                                | Thrown When                                |
| ---------------------------------------- | ------------------------------------------ |
| `exceptions::NoStagesGivenException`     | Stage list is empty                        |
| `exceptions::InvalidStageGivenException` | A string stage is not in `"IN:OUT"` format |

***

## Example

```cpp
// 18T driving sprocket, 36T driven sprocket = 2:1
auto s = yams::gearing::Sprocket::FromStages({"36:18"});

// Multi-stage belt drive: 2:1 then 2:1 = 4:1
auto s2 = yams::gearing::Sprocket::FromStages({"36:18", "24:12"});
```
