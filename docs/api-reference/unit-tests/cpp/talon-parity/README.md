# talon-parity

The Talon parity tests. They wait on simulated Talons and CANcoders, which Phoenix runs in real time, so they run on their own.

## Test files

| Test file                                              | Tests | What it checks                                                                                                                                                                             |
| ------------------------------------------------------ | ----- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| [TalonWrapperParityTest](talon-wrapper-parity-test.md) | 21    | Checks that the C++ TalonFX and TalonFXS wrappers configure gains, slots, commutation, Motion Magic, soft limits, ramps, followers, external encoders and simulation the same way as Java. |

## Run the group

From the `vendordep` folder:

```bash
./gradlew catchTests "-PcatchFilter=[TalonParity]"
```
