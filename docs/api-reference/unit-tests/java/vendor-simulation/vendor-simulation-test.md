# VendorSimulationTest

**Group:** [vendor-simulation](./)\
**Source:** `vendordep/src/test/java/yams/mechs/VendorSimulationTest.java`

Checks that the motor controllers with a vendor simulation (REV's `SparkSim` for a SPARK, Phoenix's for a Talon) are simulated through it on a pivot. The encoder, the applied output, the current limit and the closed loop must all come from the vendor simulation and agree with the pivot.

## Setup

* Every test runs on SPARK MAX (NEO), SPARK Flex (Vortex), TalonFX (Kraken X60) and TalonFXS (NEO).
* The pivot is on a 60:1 gearbox with 1 lb at 4 inches, brake mode, starting at 0°, with hard limits of -180° to 180°.
* Phoenix runs a Talon's simulation in real time, so each loop is paced at 20 ms of real time for a Talon.
* Each test starts with a reset battery and a 12 V input voltage.

## Tests

| Test                               | What it checks                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| ---------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `vendorEncoderFollowsTheMechanism` | Driven at 2 V for 0.5 s, the pivot moves more than 10°. The encoder position is within 1° of the pivot, and the encoder velocity within 5% (plus 2°/s) of the pivot's.                                                                                                                                                                                                                                                                                                                                   |
| `mechanismIsGivenTheVendorOutput`  | Driven at 3 V for 0.5 s, the vendor simulation applies more than a 0.1 duty cycle, and after the first five loops the pivot's voltage stays within 0.3 V of the duty cycle times the battery voltage.                                                                                                                                                                                                                                                                                                    |
| `vendorSimulationLimitsTheCurrent` | On a heavy pivot (30 lb at 36 inches) driven at 12 V for 0.2 s, a 10 A stator limit applies less than 80% of the duty cycle of a 120 A limit, and moves the pivot less than 80% as far.                                                                                                                                                                                                                                                                                                                  |
| `vendorControlReachesTheSetpoint`  | Sent to 45° for 3 s, the pivot and the encoder end within 3° (10° for feedforward). The duty cycle, voltage, stator current and velocity all rise above small minimums while it runs, for the loosely coupled follower too, whose encoder must also reach 45°. Runs over 36 cases: each motor controller with a PID, a feedforward with a trapezoidal profile, or a PID with an exponential profile, and with no follower, a tightly coupled follower of the same vendor, or a loosely coupled follower. |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.mechs.VendorSimulationTest"
```
