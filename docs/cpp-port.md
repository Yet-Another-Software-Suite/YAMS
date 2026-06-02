# C++ Port

YAMS now includes a native C++ port in the repository.

## Where it lives

The C++ port source is located in `yams/cpp`:

- `yams/cpp/include` — exported headers for the native port
- `yams/cpp/src` — implementation source files

The vendordep build already includes the native port sources by linking:

- `vendordep/build.gradle` → `../yams/cpp/src`
- `vendordep/build.gradle` → `../yams/cpp/include`

## What it supports

The native C++ port exposes the same core YAMS abstractions as the Java library, including:

- `yams::mechanisms::positional::Arm`
- `yams::mechanisms::positional::Elevator`
- `yams::mechanisms::positional::Pivot`
- `yams::motorcontrollers::SmartMotorController`
- `yams::motorcontrollers::local::SparkWrapper`
- `yams::motorcontrollers::remote::TalonFXSWrapper`

## Getting started

1. Add `yams/cpp/include` to your C++ project's include paths.
2. Add `yams/cpp/src` to your native build or use the vendordep package from this repository.
3. Include the port header:

```cpp
#include <yams/YAMS.h>
```

4. Configure a `SmartMotorController`, wrap your motor, and build the mechanism config.

## Notes

- The C++ port targets WPILib 2026 and supports native C++ WPILib robot code.
- Example code is not yet included in this repo, but the C++ port is already wired into the vendordep build.
