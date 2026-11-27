# TalonFXSMotorArrangementTest

**Group:** [motor-controllers](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/TalonFXSMotorArrangementTest.java`

Checks that `TalonFXSWrapper` sets the right motor arrangement and advanced hall support for each motor a TalonFXS can drive. A wrong arrangement means the TalonFXS commutates the motor incorrectly.

## Setup

* Each case builds a simulated TalonFXS behind 12:1 gearing with a 40 A stator current limit (the most a NEO 550 allows), then reads back its commutation config.

## Tests

| Test                               | What it checks                                                                                                                                                                                                                                                                                                            |
| ---------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `commutatesEachMotor`              | The TalonFXS's `MotorArrangement` and `AdvancedHallSupport` match the motor: Minion gets `Minion_JST` with advanced hall support enabled; NEO, one Thrifty Pulsar and two Thrifty Pulsars get `NEO_JST`; NEO 550 gets `NEO550_JST`; NEO Vortex gets `VORTEX_JST`. All but the Minion have advanced hall support disabled. |
| `recognizesThePulsarYamgGenerates` | For 1 to 4 motors, a Thrifty Pulsar `DCMotor` built the way YAMG builds it has exactly the same constants as `YMotors.getThriftyPulsar`, so the wrapper recognizes it.                                                                                                                                                    |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.TalonFXSMotorArrangementTest"
```
