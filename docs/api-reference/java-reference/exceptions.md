# Exceptions

Package: `yams.core.exceptions`\
Javadoc: [`yams.core.exceptions`](https://javadocs.yassrobotics.com/yams/yams/core/exceptions/package-summary.html)

YAMS throws typed `RuntimeException` subclasses for configuration and runtime errors. All exceptions extend `RuntimeException` so they are unchecked; you do not need to declare or catch them unless you want to handle configuration problems gracefully.

Most exceptions are thrown during motor controller or mechanism construction if required configuration fields are missing.

***

## Exception classes

| Class                                         | Thrown When                                                                            |
| --------------------------------------------- | -------------------------------------------------------------------------------------- |
| `ArmConfigurationException`                   | Required `ArmConfig` fields are missing or invalid                                     |
| `DifferentialMechanismConfigurationException` | Required `DifferentialMechanismConfig` fields are missing                              |
| `DoubleJointedArmConfigurationException`      | A joint `ArmConfig` is missing required fields                                         |
| `ElevatorConfigurationException`              | Required `ElevatorConfig` fields are missing                                           |
| `FlyWheelConfigurationException`              | Required `FlyWheelConfig` fields are missing                                           |
| `PivotConfigurationException`                 | Required `PivotConfig` fields are missing                                              |
| `SmartMotorControllerConfigurationException`  | The `SmartMotorControllerConfig` is invalid or missing required options                |
| `SwerveDriveConfigurationException`           | Required `SwerveDriveConfig` fields are missing                                        |
| `MotorNotPresentException`                    | A mechanism has no motor controller (defined, but not currently thrown by YAMS itself) |
| `InvalidStageGivenException`                  | A gear stage string is not in `"IN:OUT"` format                                        |
| `NoStagesGivenException`                      | A `GearBox` or `Sprocket` was constructed with an empty stage list                     |

***

## Configuration exception constructors

Most configuration exceptions follow this constructor pattern:

```java
SomeConfigurationException(String message, String result, String remedyFunction)
```

| Parameter        | Description                                             |
| ---------------- | ------------------------------------------------------- |
| `message`        | Description of what is missing or invalid               |
| `result`         | What behavior will be broken without this configuration |
| `remedyFunction` | The builder method to call to fix the problem           |

The remaining exceptions take simpler arguments:

```java
MotorNotPresentException(String mechanismType)
InvalidStageGivenException(String stage)
NoStagesGivenException()
```

***

## Example error handling

```java
try {
    SmartMotorController motor = new SparkWrapper(
        new SparkMax(CANPort.CAN_S0, 1, MotorType.kBrushless),
        DCMotor.getNEO(1),
        config
    );
} catch (SmartMotorControllerConfigurationException e) {
    DriverStation.reportError("Motor config error: " + e.getMessage(), false);
}
```

{% hint style="info" %}
Leave these exceptions uncaught: the stack trace names the config method you need to call. Catching one doesn't keep the robot working. It means the motor controller or mechanism was never built, so the code fails the first time it uses it instead. Fix the config before an event, and catch the exception only to log its message.
{% endhint %}
