# SmartMath

**Package:** `yams.core.math`\
**Javadoc:** [`SmartMath`](https://javadocs.yassrobotics.com/yams/yams/core/math/SmartMath.html)

`SmartMath` contains static utility methods for gear ratio calculations. It cannot be instantiated.

## Methods

| Method                   | Signature                                                | Description                                                                                                  |
| ------------------------ | -------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------ |
| `sensorToMechanismRatio` | `static double sensorToMechanismRatio(double... stages)` | Multiplies all stage values together. Returns the combined ratio of sensor rotations per mechanism rotation. |
| `gearBox`                | `static double gearBox(double... stages)`                | Computes the mechanism-to-rotor ratio (MECHANISM\_ROTATIONS / ROTOR\_ROTATIONS) by multiplying all stages.   |

## Example

```java
// A three-stage gearbox: 10:1, then 3:1, then 2:1 = 60:1 total
double ratio = SmartMath.sensorToMechanismRatio(10.0, 3.0, 2.0); // returns 60.0
```
