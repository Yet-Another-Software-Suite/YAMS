# motor-controllers

Motor controller tests that are not about an external encoder: each vendor's device configuration, feedforwards, exponential profiles, the simulation period, the A301, the TalonFXS motor arrangement, and a vendor config over the device's saved settings.

## Test classes

| Test class                                                         | Tests | What it checks                                                                                                                                                           |
| ------------------------------------------------------------------ | ----- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| [TalonFXSMotorArrangementTest](talonfxs-motor-arrangement-test.md) | 2     | Checks that the TalonFXS wrapper sets the right motor arrangement and advanced hall support for each motor, and that it recognizes the Thrifty Pulsar model YAMG builds. |
| [VendorConfigTest](vendor-config-test.md)                          | 2     | Checks that with resetPreviousConfig(false) a Talon keeps a setting saved on the device, unless a vendor config overrides it.                                            |
| [ArmFeedforwardTest](arm-feedforward-test.md)                      | 1     | With an LQR and no profile, holding a horizontal arm commands about the ArmFeedforward's kG on SPARK MAX, TalonFX, TalonFXS, Nova and A301.                              |
| [ExponentialProfileConfigTest](exponential-profile-config-test.md) | 4     | Exponential profile constraints built from physical constants or max velocity and acceleration have finite kV and kA that match the expected formulas.                   |
| [ForceFeedforwardTest](force-feedforward-test.md)                  | 4     | Force to voltage and current conversions match DCMotor, the recorded feedforward force is set and cleared correctly, and a force alone produces motor output.            |
| [SimulationPeriodTest](simulation-period-test.md)                  | 1     | Position control still reaches 80 degrees when simIterate runs every 10 ms and the robot loop every 20 ms, on six motor controllers.                                     |
| [SparkDeviceConfigTest](spark-device-config-test.md)               | 4     | Soft limits and the absolute encoder zero offset reach the SPARK in its own units, and a temperature cutoff needs the SystemCore closed loop.                            |
| [TalonDeviceConfigTest](talon-device-config-test.md)               | 4     | Talon live setters write every ramp and the Motion Magic jerk, a temperature cutoff needs an LQR, and the velocity measurement delay depends on where the loop runs.     |
| [ConfigCopyAndHelpersTest](config-copy-and-helpers-test.md)        | 3     | Config clones do not share controllers, the wheel inertia helper uses a solid disk, and elevator exponential constraints are finite.                                     |
| [A301WrapperTest](a301-wrapper-test.md)                            | 4     | The A301 wrapper rejects unsupported options, closes its loop on the SystemCore, reads its absolute encoder in the wrapping range, and uses the A301 motor model.        |

## Run the group

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.ArmFeedforwardTest" \
  --tests "yams.core.motorcontrollers.ExponentialProfileConfigTest" \
  --tests "yams.core.motorcontrollers.ForceFeedforwardTest" \
  --tests "yams.core.motorcontrollers.SimulationPeriodTest" \
  --tests "yams.core.motorcontrollers.SparkDeviceConfigTest" \
  --tests "yams.core.motorcontrollers.TalonDeviceConfigTest" \
  --tests "yams.core.motorcontrollers.ConfigCopyAndHelpersTest" \
  --tests "yams.core.motorcontrollers.A301WrapperTest" \
  --tests "yams.core.motorcontrollers.TalonFXSMotorArrangementTest" \
  --tests "yams.core.motorcontrollers.VendorConfigTest"
```
