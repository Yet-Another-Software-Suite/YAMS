---
icon: file-code
---

# Step 1: Define the IO interface

The IO interface lists what the arm reads (its inputs) and what it can be told to do. Following AdvantageKit conventions, the inputs are grouped in an `Inputs` class. This needs AdvantageKit installed, from [Step 0: Install AdvantageKit and set up Robot](setup.md).

The inputs are plain numbers with the unit in the name, as AdvantageKit's own examples do. The methods that command the mechanism take WPILib units (`Angle`), so a caller cannot mix up degrees and rotations.

The tutorial builds an arm. The [elevator and shooter](elevator-and-shooter.md) follow the same steps, and have their own page at the end.

## Arm IO interface

Create `ArmIO.java` in a new `akit` folder: `subsystems/akit` for Commands v2, or `mechanisms/akit` for Commands v3. The two tabs differ only in the `package` line.

{% tabs %}
{% tab title="Java, Commands v2" %}
```java
package first.robot.subsystems.akit;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.Angle;

public interface ArmIO {

  /**
   * Inputs that will be logged and replayed.
   * The @AutoLog annotation generates the ArmIOInputsAutoLogged class.
   */
  @AutoLog
  public static class ArmIOInputs {
    public double positionRotations = 0.0;
    public double velocityRotationsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public double targetPositionRotations = 0.0;
  }

  /** Update the inputs from hardware. Called every loop cycle. */
  default void updateInputs(ArmIOInputs inputs) {}

  /** Set the target angle for the arm. */
  default void setTargetAngle(Angle angle) {}

  /** Stop the arm motor. */
  default void stop() {}

  /** Run the arm's simulation for one loop. */
  default void simIterate() {}

  /** Publish the arm's YAMS telemetry. */
  default void updateTelemetry() {}
}
```
{% endtab %}

{% tab title="Java, Commands v3" %}
```java
package first.robot.mechanisms.akit;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.Angle;

public interface ArmIO {

  /**
   * Inputs that will be logged and replayed.
   * The @AutoLog annotation generates the ArmIOInputsAutoLogged class.
   */
  @AutoLog
  public static class ArmIOInputs {
    public double positionRotations = 0.0;
    public double velocityRotationsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public double targetPositionRotations = 0.0;
  }

  /** Update the inputs from hardware. Called every loop cycle. */
  default void updateInputs(ArmIOInputs inputs) {}

  /** Set the target angle for the arm. */
  default void setTargetAngle(Angle angle) {}

  /** Stop the arm motor. */
  default void stop() {}

  /** Run the arm's simulation for one loop. */
  default void simIterate() {}

  /** Publish the arm's YAMS telemetry. */
  default void updateTelemetry() {}
}
```
{% endtab %}
{% endtabs %}

Every method has an empty default, so the replay IO class in [Log replay](log-replay.md) can leave them all out.

{% hint style="info" %}
The `@AutoLog` annotation from AdvantageKit generates a class (`ArmIOInputsAutoLogged`) that writes the inputs to the log, and reads them back in replay. See the [AdvantageKit AutoLog documentation](https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces#autolog) for details.

AdvantageKit logs each field with its first letter capitalized, so `positionRotations` shows up as `Arm/PositionRotations`.
{% endhint %}

**Check:** build the project. It succeeds, and `ArmIOInputsAutoLogged` can be imported (it is generated under `build/generated`).

**If it did not work:** if `ArmIOInputsAutoLogged` is not generated, check the line from [Add the AutoLog code generator](setup.md#add-the-autolog-code-generator), then clean the project with `gradlew clean` (Windows) or `./gradlew clean` (Linux/macOS) and build again. If it still does not generate, try commenting out the `annotationProcessor wpi.java.deps.wpilibAnnotations()` line and clean again.
