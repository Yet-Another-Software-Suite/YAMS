# MechanismLimitTest

**Group:** [mechanism-limits](./)\
**Source:** `vendordep/src/test/java/yams/mechs/MechanismLimitTest.java`

Checks the limits of a `Pivot`, an `Arm` and an `Elevator`: that the `min()` and `max()` triggers fire only past the soft limits, that a configured starting position is where the simulation starts, and that a starting position outside the hard limits throws. Robot code that uses these triggers to stop a mechanism relies on them firing in the right direction.

## Setup

* Each trigger test runs on 12 cases: SPARK MAX (NEO), SPARK Flex (Vortex), TalonFXS (NEO), TalonFX (Kraken X60), Nova (NEO) and A301, each at a starting position inside the soft limits (expects `false`) and one just past the soft limit (expects `true`).
* The starting position is set in the motor controller config, so the mechanism's simulation is seeded the normal way. Each trigger test runs one 20 ms scheduler cycle before reading the trigger.
* Soft limits are -100° to 100° for the pivot and arm, and 0 m to 5 m for the elevator. The hard limits are a little wider at the bottom so the "past the lower limit" start can be built.
* The Nova gets a trapezoidal profile, as it applies a feedforward only with a motion profile. The A301 runs on its 64:1 gearbox without current limits.

## Tests

| Test                                                 | What it checks                                                                                                                                                       |
| ---------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `testPivotMinTrigger`                                | `pivot.min()` is false at 0° and true at -101°.                                                                                                                      |
| `testPivotMaxTrigger`                                | `pivot.max()` is false at 0° and true at 101°.                                                                                                                       |
| `testArmMinTrigger`                                  | `arm.min()` is false at 0° and true at -105°, far enough past the limit that gravity cannot pull it back in one loop.                                                |
| `testArmMaxTrigger`                                  | `arm.max()` is false at 0° and true at 101°.                                                                                                                         |
| `testElevatorMinTrigger`                             | `elevator.min()` is false at 1.5 m and true at -0.01 m.                                                                                                              |
| `testElevatorMaxTrigger`                             | `elevator.max()` is false at 1.5 m and true at 5.5 m.                                                                                                                |
| `testStartingPositionAppliedAfterSetup`              | With a pivot config starting at 45°, the motor controller reads within 2° of 45° after `setupSimulation()` and one cycle. Runs on each of the six motor controllers. |
| `testPivotThrowsWhenStartingPositionBelowLowerLimit` | Building a `Pivot` with hard limits of -100° to 150° and a starting position of -200° throws `PivotConfigurationException`.                                          |
| `testPivotThrowsWhenStartingPositionAboveUpperLimit` | The same, with a starting position of 200°.                                                                                                                          |
| `testArmThrowsWhenStartingPositionBelowLowerLimit`   | Building an `Arm` with hard limits of -100° to 200° and a starting position of -200° throws `ArmConfigurationException`.                                             |
| `testArmThrowsWhenStartingPositionAboveUpperLimit`   | The same, with a starting position of 300°.                                                                                                                          |
| `testElevatorThrowsWhenStartingHeightBelowMinimum`   | Building an `Elevator` with hard limits of 0 m to 6 m and a starting height of -1 m throws `SmartMotorControllerConfigurationException`.                             |
| `testElevatorThrowsWhenStartingHeightAboveMaximum`   | The same, with a starting height of 7 m.                                                                                                                             |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.mechs.MechanismLimitTest"
```
