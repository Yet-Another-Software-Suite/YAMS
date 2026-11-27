---
icon: download
---

# Step 0: Install AdvantageKit and set up Robot

Before the IO classes can log anything, the project needs the AdvantageKit library, its code generator in `build.gradle`, and a `Robot` class that starts the AdvantageKit logger.

**Before you start:** you need a Java project that already builds with YAMS, from a **Command v2 Robot** or **Command v3 Robot** template. If you do not have one yet, follow [Create a project and install YAMS](../project-setup/) first.

{% hint style="info" %}
AdvantageKit is Java only. In a C++ project, use YAMS's own telemetry (`withTelemetry(...)`) and WPILib's data logs instead.
{% endhint %}

{% hint style="info" %}
AdvantageKit is a separate library from Team 6328, and its 2027 releases are still alphas. If anything here does not match what you see, the [AdvantageKit installation docs for existing projects](https://docs.advantagekit.org/getting-started/installation/existing-projects) are the source of truth.
{% endhint %}

{% stepper %}
{% step %}
### Install the vendor library

Click the **WPILib logo** in the left bar to open the vendor library list, the same way you installed YAMS in [Install YAMS](../project-setup/#install-yams). Scroll down to **AdvantageKit** and click **Install**.

The list only shows the libraries for your WPILib version, so it installs the 2027 AdvantageKit.

**Check:** `vendordeps/AdvantageKit.json` exists in your project, and its `"version"` starts with `27.0.0`.
{% endstep %}

{% step %}
### Add the AutoLog code generator

The `@AutoLog` annotation used in [Step 1](io-interface.md) generates a class for each inputs class when the code builds. That needs an annotation processor line in the `dependencies` block of `build.gradle`. Add the three lines under the comment, and leave the rest of the block as it is:

```groovy
dependencies {
    annotationProcessor wpi.java.deps.wpilibAnnotations()
    // AdvantageKit's @AutoLog code generator, at the same version as the vendor library.
    def akitJson = new groovy.json.JsonSlurper().parseText(
        new File(projectDir.getAbsolutePath() + "/vendordeps/AdvantageKit.json").text)
    annotationProcessor "org.littletonrobotics.akit:akit-autolog:$akitJson.version"

    // ... the rest of your dependencies stay as they are
}
```

This is the setup used by the YAMS AdvantageKit examples. AdvantageKit's own docs add more to `build.gradle` (such as recording git metadata); see the [AdvantageKit installation docs](https://docs.advantagekit.org/getting-started/installation/existing-projects) if you want those.

**Check:** the project still builds.
{% endstep %}

{% step %}
### Make `Robot` a logged robot

`Robot` extends AdvantageKit's version of its base class, and starts the logger in its constructor. Anything created before `Logger.start()` is not logged from the first loop, so subsystems and mechanisms are created after it. `Robot` also defines `currentMode`, which [Log replay](log-replay.md) uses to choose the IO class.

{% tabs %}
{% tab title="Java, Commands v2" %}
`Robot` extends AdvantageKit's `LoggedRobot` instead of WPILib's `TimedRobot`. This is the whole file. It keeps the template's autonomous methods, so `RobotContainer.getAutonomousCommand()` is still used.

```java
package first.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.framework.RobotBase;

public class Robot extends LoggedRobot {
  public static enum Mode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }

  /** Change to Mode.REPLAY to replay a log in simulation. */
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  private Command autonomousCommand;

  private final RobotContainer robotContainer;

  public Robot() {
    switch (currentMode) {
      case REAL:
        // On the SystemCore: write a log file and publish to NetworkTables.
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // In simulation: publish to NetworkTables.
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log: read it, and write the replayed values to a new log next to it.
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start the AdvantageKit logger before creating the subsystems.
    Logger.start();

    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousExit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void utilityInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
```
{% endtab %}

{% tab title="Java, Commands v3" %}
`Robot` extends AdvantageKit's `LoggedOpModeRobot` instead of WPILib's `OpModeRobot`. It finds your `@Teleop` and `@Autonomous` op modes the same way, so `ExampleTeleop` and `ExampleAuto` keep working.

The template creates `exampleMechanism` where the field is declared, which runs before the constructor. Here it is created in the constructor instead, after `Logger.start()`. This is the whole file.

```java
package first.robot;

import first.robot.constants.DriverConstants;
import first.robot.mechanisms.ExampleMechanism;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedOpModeRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.framework.RobotBase;

public class Robot extends LoggedOpModeRobot {
  public static enum Mode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }

  /** Change to Mode.REPLAY to replay a log in simulation. */
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // Created in the constructor, after the logger starts.
  final ExampleMechanism exampleMechanism;
  final CommandGamepad exampleController =
      new CommandGamepad(DriverConstants.DRIVER_CONTROLLER_PORT);

  public Robot() {
    switch (currentMode) {
      case REAL:
        // On the SystemCore: write a log file and publish to NetworkTables.
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // In simulation: publish to NetworkTables.
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log: read it, and write the replayed values to a new log next to it.
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start the AdvantageKit logger before creating the mechanisms.
    Logger.start();

    exampleMechanism = new ExampleMechanism();
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getDefault().run();
  }
}
```
{% endtab %}
{% endtabs %}

`first/Main.java` does not change: it still starts `first.robot.Robot`.

**Check:** run the code in simulation, open AdvantageScope and connect it to the simulator (**File > Connect to Simulator**). An `AdvantageKit` table appears next to the YAMS `Mechanisms` table.
{% endstep %}
{% endstepper %}

## If it did not work

* **`org.littletonrobotics.junction` cannot be found:** the vendor library is not installed. Repeat [Install the vendor library](setup.md#install-the-vendor-library), then build again.
* **The build fails inside AdvantageKit, or the robot code crashes as it starts:** `vendordeps/AdvantageKit.json` is not a `27.0.0` version, often because it was installed from a link in AdvantageKit's own docs, which gives the 2026 release. Delete it, and install AdvantageKit from the vendor library list as in [Install the vendor library](setup.md#install-the-vendor-library).
* **`ArmIOInputsAutoLogged` cannot be found later:** the annotation processor line is missing or wrong. Check [Add the AutoLog code generator](setup.md#add-the-autolog-code-generator), clean the project with `gradlew clean` (Windows) or `./gradlew clean` (Linux/macOS) and build again.
* **`getAutonomousCommand()` or `exampleMechanism` cannot be found:** your `Robot` is missing something the rest of the template uses. Use the whole file from the tab for your template.
