# ExponentialProfileConfigTest

**Group:** [motor-controllers](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/ExponentialProfileConfigTest.java`

Checks the exponential profile constraints that `withExponentialProfile` builds. The kV and kA inside the constraints must be finite and match the mechanism's velocity dynamics, using the same formula as the C++ port.

## Setup

* Each test uses a `SmartMotorControllerConfig` with a 12:1 gearing (3:1 then 4:1). No motor controller or simulation is created.

## Tests

| Test                                                        | What it checks                                                                                                                                                                                  |
| ----------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `armProfileFromPhysicalConstantsHasFiniteGains`             | An arm profile from 12 V, one NEO and a 0.1 kg m² moment of inertia has the kV and kA of the arm's dynamics (converted from radians to rotations), and a maximum input of 12 V.                 |
| `elevatorProfileFromPhysicalConstantsHasFiniteGains`        | An elevator profile from 10 V, two Kraken X60s, an 8 kg carriage and a 0.025 m drum radius has the kV and kA of the elevator's dynamics in meters, and a maximum input of 10 V.                 |
| `profileFromMaximumVelocityAndAccelerationHasMatchingGains` | A profile from 12 V, 10 rotations per second and 40 rotations per second squared has kV = 12 / 10 and kA = 12 / 40, and a maximum input of 12 V.                                                |
| `createConstraintsMatchesTheConfig`                         | `ExponentialProfilePIDController.createConstraints` with the same values gives the same kV, kA, `A` and `B` as the config. `A` is negative and the maximum velocity is 10 rotations per second. |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.ExponentialProfileConfigTest"
```
