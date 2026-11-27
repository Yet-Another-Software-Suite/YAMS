# MechanismGearing

**Namespace:** `yams::gearing`\
**Header:** `#include <yams/gearing/MechanismGearing.hpp>`\
**Java equivalent:** [Java MechanismGearing](../../java-reference/gearing/mechanism-gearing.md)\
**Doxygen:** [`MechanismGearing`](https://doxygen.yassrobotics.com/yams/classyams_1_1gearing_1_1MechanismGearing.html)

`MechanismGearing` is the composite gearing type accepted by `SmartMotorControllerConfig::WithMotorGearing()`. It may combine a `GearBox` with an optional `Sprocket`.

***

## Constants

```cpp
static const MechanismGearing kOne;  // 1:1 direct drive
```

***

## Constructors

```cpp
explicit MechanismGearing(double reductionRatio)  // same as GearBox::FromReductionStages({reductionRatio})
explicit MechanismGearing(const GearBox& gearBox)
MechanismGearing(const GearBox& gearBox, const Sprocket& sprockets)
```

***

## Methods

| Method                             | Returns             | Description                                                                                                   |
| ---------------------------------- | ------------------- | ------------------------------------------------------------------------------------------------------------- |
| `GetRotorToMechanismRatio() const` | `double`            | Mechanism rotations per motor rotation (less than 1 for a reduction)                                          |
| `GetMechanismToRotorRatio() const` | `double`            | Motor rotations per mechanism rotation (the overall reduction, greater than 1 for a reduction)                |
| `Div(double i)`                    | `MechanismGearing&` | Calls `GearBox::Div(i)` on the gearbox stage (multiplies the reduction by `i`). Returns `*this` for chaining. |

***

## Example

```cpp
using namespace yams::gearing;

auto gearing = MechanismGearing{
    GearBox::FromTeeth({14, 72}),    // 5.14:1
    Sprocket::FromStages({"24:18"})  // 1.33:1
};
// Total: ~6.85:1

config.WithMotorGearing(gearing);
```

{% hint style="info" %}
`GetMechanismToRotorRatio()` is the overall reduction (motor rotations per mechanism rotation, greater than 1 for reductions). YAMS passes it as the gearing to the WPILib physics simulations. `GetRotorToMechanismRatio()` is its reciprocal.
{% endhint %}
