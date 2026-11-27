# StartingPositionTest

**Group:** [absolute-encoders](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/StartingPositionTest.java`

Checks a mechanism that starts at 30° instead of 0°, on every motor controller and under every closed loop controller. It makes sure the mechanism reads where it really is at power on, and moves only the 70° it needs to reach 100°, not an extra rotation.

## Setup

* Without an absolute encoder, the encoder only counts from where it powers on, so the config gives a starting position of 30° (`withStartingPosition`).
* With an absolute encoder, no starting position is given. The simulated mechanism rests at 30° (`withSimStartingPosition`) and the encoder reports it.
* Cases come from `AbsoluteEncoderCases`. Without an absolute encoder: the motor's encoder on SPARK MAX, SPARK Flex, TalonFX, TalonFXS, Nova and A301, a quadrature Through Bore Encoder on each SPARK, and a quadrature encoder on a Nova. With one: each SPARK's absolute encoder, an absolute encoder each SPARK reads over CAN, a CANcoder and a CANdi on each Talon, a Nova's absolute encoder, a Thrifty CAN Encoder on a Nova, and an A301's own encoder.
* Each case runs under a PID, a trapezoidal profile, an exponential profile and an LQR, behind 12:1 gearing: 80 cases in all.

## Tests

| Test                 | What it checks                                                                                                                                                                                                                                                                                                                                                                               |
| -------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `startsAwayFromZero` | After 0.5 seconds without a setpoint, the simulated mechanism, the mechanism reading, and the external encoder reading (if any) are all within 5° of 30°. After 2.5 seconds driving to 100°, the simulated mechanism is within 5° of 100° without wrapping (so it moved 70°, not a full extra rotation), and the mechanism and encoder readings are within 5° of 100°. Runs on all 80 cases. |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.StartingPositionTest"
```
