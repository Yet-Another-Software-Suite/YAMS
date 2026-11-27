# YUnits

Package: `yams.core.units`\
Javadoc: [`YUnits`](https://javadocs.yassrobotics.com/yams/yams/core/units/YUnits.html)

`YUnits` is a static utility class containing extended unit constants for use with WPILib's `org.wpilib.units` system. All values are defined as `Unit` instances compatible with WPILib's `Measure` types.

`YUnits` cannot be instantiated.

***

## Distance units

| Constant           | Symbol | Metric Equivalent    |
| ------------------ | ------ | -------------------- |
| `Hands`            | N/A    | 0.1016 m (4 inches)  |
| `Yards`            | yd     | 0.9144 m             |
| `Cubits`           | N/A    | 0.4572 m (18 inches) |
| `Fathoms`          | N/A    | 1.8288 m             |
| `Chains`           | N/A    | 20.1168 m            |
| `Furlongs`         | N/A    | 201.168 m            |
| `Miles`            | mi     | 1609.344 m           |
| `Leagues`          | N/A    | 4828.032 m           |
| `FootlongSandwich` | N/A    | 0.3048 m             |

## Time units

| Constant    | Equivalent            |
| ----------- | --------------------- |
| `Hours`     | 3600 s                |
| `Days`      | 86400 s               |
| `Weeks`     | 604800 s              |
| `Fortnight` | 1209600 s             |
| `Years`     | 31536000 s (365 days) |

## Velocity units

| Constant                     | Description                     |
| ---------------------------- | ------------------------------- |
| `MilesPerHour` (MPH)         | Imperial speed                  |
| `FurlongsPerFortnight` (FPF) | A classic unit of slow speed    |
| `SandwichPerSecond`          | One FootlongSandwich per second |

## Angular rate units

| Constant                 | Description                                             |
| ------------------------ | ------------------------------------------------------- |
| `RotationsPerYear` (RPY) | For very slow mechanisms                                |
| `RPMPerSecond`           | Angular acceleration in rotations-per-minute per second |

## Momentum units

| Constant                      | Description                     |
| ----------------------------- | ------------------------------- |
| `PoundFeetPerSecond`          | Linear momentum, imperial       |
| `PoundInchesPerSecond`        | Linear momentum, smaller scale  |
| `PoundFeetSquaredPerSecond`   | Angular momentum, imperial      |
| `PoundInchesSquaredPerSecond` | Angular momentum, smaller scale |

## Moment of inertia units

| Constant                     | Description                   |
| ---------------------------- | ----------------------------- |
| `PoundSquareFeet` (lb·ft²)   | MOI in imperial units         |
| `PoundSquareInches` (lb·in²) | MOI in smaller imperial units |

***

## Usage

```java
import static yams.core.units.YUnits.*;

Distance armLength = Cubits.of(1.5);   // 1.5 cubits ≈ 0.686 m
LinearVelocity speed = MilesPerHour.of(15);
```

{% hint style="info" %}
`FurlongsPerFortnight` and `FootlongSandwich` are included for completeness (and fun). For actual robot programming, `Meters`, `Inches`, `Feet`, and `Kilograms` from WPILib's standard units are more appropriate.
{% endhint %}
