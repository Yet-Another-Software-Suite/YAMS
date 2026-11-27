# VendorConfigTest

**Group:** [motor-controllers](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/VendorConfigTest.java`

Checks which settings a TalonFX or TalonFXS keeps when the config uses `withResetPreviousConfig(false)`. It uses a setting YAMS does not manage, the duty cycle neutral deadband. Without a vendor config, the value saved on the device stays. With a vendor config, the vendor config's value replaces it.

## Setup

* Before each wrapper is built, the device is given a saved deadband of 0.05, as if set earlier in Phoenix Tuner X. The vendor config, when used, sets 0.1.
* The device stores the deadband in steps of about 0.0004, so values are compared within 0.002.

## Tests

| Test                                        | What it checks                                                                                                                                     |
| ------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| `keepsTheSavedSettingsWithoutAVendorConfig` | Without a vendor config, the device's deadband is still 0.05 after construction and after applying the config again. Runs on TalonFX and TalonFXS. |
| `vendorConfigWinsOverTheSavedSettings`      | With a vendor config, the device's deadband is 0.1 after construction and after applying the config again. Runs on TalonFX and TalonFXS.           |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.VendorConfigTest"
```
