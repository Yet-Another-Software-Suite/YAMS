# Getting started

{% hint style="info" %}
YAMS requires WPILib 2027 (`2027_alpha7`) and Java 25. Robot code runs on the SystemCore. Ensure your project is up to date before installing.
{% endhint %}

## Installation

YAMS ships as two vendordeps, one per WPILib command framework. **Install exactly one of them.** The two conflict with each other and cannot be installed side by side.

| Command framework | Vendordep URL                                      | Java packages                      | C++ |
| ----------------- | -------------------------------------------------- | ---------------------------------- | --- |
| Commands v2       | `https://cdn.yassrobotics.com/yams_commands2.json` | `yams.commands2.*` + `yams.core.*` | Yes |
| Commands v3       | `https://cdn.yassrobotics.com/yams_commands3.json` | `yams.commands3.*` + `yams.core.*` | No  |

The Commands v3 vendordep is Java only, because WPILib does not publish a C++ Commands v3 API. All Maven artifacts are served from `https://cdn.yassrobotics.com/`.

### Online

1. In VS Code, open the Command Palette (`Ctrl+Shift+P` / `Cmd+Shift+P`).
2. Run **WPILib: Manage Vendor Libraries**, then **Install new libraries (online)**.
3. Paste the URL for the command framework your project uses (see the table above) and press Enter.

YAMS wraps vendor hardware, so also install the vendordeps for the motor controllers you use: REVLib 2027 (SPARK MAX, SPARK FLEX), Phoenix 6 (TalonFX, TalonFXS), or ThriftyLib 2027 (Nova).

### Offline

Each [GitHub release](https://github.com/Yet-Another-Software-Suite/YAMS/releases) includes `<version>-YAMS-Offline.zip`. It contains a `maven` folder and a `vendordeps` folder that merge into your WPILib `2027_alpha7` home folder. That folder is created by the WPILib installer, so install WPILib first.

{% tabs %}
{% tab title="Windows" %}
```powershell
Expand-Archive -Path "$HOME\Downloads\<version>-YAMS-Offline.zip" -DestinationPath "C:\Users\Public\wpilib\2027_alpha7" -Force
```
{% endtab %}

{% tab title="macOS / Linux" %}
```bash
unzip -o ~/Downloads/<version>-YAMS-Offline.zip -d ~/wpilib/2027_alpha7
```
{% endtab %}
{% endtabs %}

Afterwards, run **WPILib: Manage Vendor Libraries**, then **Install new libraries (offline)**, and pick **Yet Another Mechanism System (Commands v2)** or **(Commands v3)**.

## Java packages

The Java API is split into two layers:

* `yams.core.*`: mechanism and motor controller state, physics simulation, telemetry, configs, wrappers, gearing, math, and units. It has no dependency on a command framework.
* `yams.commands2.*` or `yams.commands3.*`: the classes that create `Command` and `Trigger` objects. Import the mechanisms (`Arm`, `Elevator`, `Pivot`, `FlyWheel`, ...), `SmartMotorControllerConfig`, `SwerveDriveConfig`, `SwerveDrive`, and `SwerveInputStream` from here.

Everything else (`ArmConfig`, `SparkWrapper`, `MechanismGearing`, `TelemetryVerbosity`, ...) comes from `yams.core`. WPILib itself now lives under `org.wpilib.*`.

## Example code

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/subsystems/ArmSubsystem.java#L80-L85" %}

## Minimal example: Java

The snippet below creates a single-motor arm using YAMS.

{% tabs %}
{% tab title="Commands v2" %}
```java
package first.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.util.CANPorts;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.Arm;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class ArmSubsystem extends SubsystemBase {

    private final Arm arm;

    public ArmSubsystem() {
        SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withZeroPower(MotorMode.BRAKE)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            .withClosedLoopController(0.5, 0.0, 0.01)
            .withFeedforward(new ArmFeedforward(0.1, 0.5, 0.01))
            .withSoftLimits(Degrees.of(-90), Degrees.of(90))
            .withMomentOfInertia(Meters.of(0.6), Kilograms.of(2.0))
            .withStartingPosition(Degrees.of(0))
            .withStatorCurrentLimit(Amps.of(40))
            .withClosedLoopTolerance(Degrees.of(2))
            .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH);
        SmartMotorController motor = new SparkWrapper(
            new SparkMax(CANPorts.fromBusId(1), 1, MotorType.kBrushless),
            DCMotor.getNEO(1),
            motorConfig
        );
        ArmConfig config = new ArmConfig()
            .withLength(Meters.of(0.6))
            .withHardLimits(Degrees.of(-100), Degrees.of(100))
            .withTelemetry("Arm", TelemetryVerbosity.HIGH);
        arm = new Arm(config, motor);
    }

    public Command setAngle(Angle angle) {
        return arm.setAngle(angle);
    }

    @Override
    public void periodic() {
        arm.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        arm.simIterate();
    }
}
```
{% endtab %}

{% tab title="Commands v3" %}
```java
package first.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.util.CANPorts;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.Arm;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class ArmMechanism implements Mechanism {

    private final Arm arm;

    public ArmMechanism() {
        SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withZeroPower(MotorMode.BRAKE)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            .withClosedLoopController(0.5, 0.0, 0.01)
            .withFeedforward(new ArmFeedforward(0.1, 0.5, 0.01))
            .withSoftLimits(Degrees.of(-90), Degrees.of(90))
            .withMomentOfInertia(Meters.of(0.6), Kilograms.of(2.0))
            .withStartingPosition(Degrees.of(0))
            .withStatorCurrentLimit(Amps.of(40))
            .withClosedLoopTolerance(Degrees.of(2))
            .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH);
        SmartMotorController motor = new SparkWrapper(
            new SparkMax(CANPorts.fromBusId(1), 1, MotorType.kBrushless),
            DCMotor.getNEO(1),
            motorConfig
        );
        ArmConfig config = new ArmConfig()
            .withLength(Meters.of(0.6))
            .withHardLimits(Degrees.of(-100), Degrees.of(100))
            .withTelemetry("Arm", TelemetryVerbosity.HIGH);
        arm = new Arm(config, motor);
    }

    public Command setAngle(Angle angle) {
        return arm.setAngle(angle);
    }

    /** Call from Robot.robotPeriodic(), before the scheduler runs. */
    public void periodic() {
        arm.updateTelemetry();
    }

    /** Call from Robot.robotPeriodic() when RobotBase.isSimulation() is true. */
    public void simulationPeriodic() {
        arm.simIterate();
    }
}
```

In the Commands v3 examples, each mechanism's `periodic()` and `simulationPeriodic()` are called from `Robot.robotPeriodic()` before `Scheduler.getDefault().run()`. The [Commands v3 example robot](../../examples/commands3/simple_robot-2027) shows the full pattern.
{% endtab %}
{% endtabs %}

## Minimal example: C++

C++ supports WPILib Commands v2. WPILib C++ types live in the `wpi::` namespaces (`wpi::units`, `wpi::math`, `wpi::cmd`).

```cpp
#include <optional>

#include <rev/SparkMax.h>
#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/commands2/SubsystemBase.hpp>
#include <wpi/hardware/bus/CANPort.hpp>
#include <wpi/math/controller/ArmFeedforward.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/units/angle.hpp>

#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/mechanisms/config/ArmConfig.hpp"
#include "yams/mechanisms/positional/Arm.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/local/SparkWrapper.hpp"

class ArmSubsystem : public wpi::cmd::SubsystemBase {
public:
    ArmSubsystem() {
        using Cfg = yams::motorcontrollers::SmartMotorControllerConfig;
        motorConfig
            .WithSubsystem(this)
            .WithZeroPower(Cfg::MotorMode::BRAKE)
            .WithMotorGearing(yams::gearing::MechanismGearing{
                yams::gearing::GearBox::FromReductionStages({3.0, 4.0})})
            .WithFeedback(0.5, 0.0, 0.01)
            .WithFeedforward(wpi::math::ArmFeedforward{
                0.1_V, 0.0_V,
                wpi::units::unit_t<wpi::math::ArmFeedforward::kv_unit>{0.5},
                wpi::units::unit_t<wpi::math::ArmFeedforward::ka_unit>{0.01}})
            .WithMechanismLimits(-90_deg, 90_deg)
            .WithMOI(0.6_m, 2.0_kg)
            .WithStartingPosition(0_deg)
            .WithStatorCurrentLimit(40_A)
            .WithTelemetry("ArmMotor", Cfg::TelemetryVerbosity::HIGH);
        motor.emplace(&m_sparkMax, wpi::math::DCMotor::NEO(1), &motorConfig);
        armConfig.WithArmLength(0.6_m)
                 .WithMinAngle(-100_deg)
                 .WithMaxAngle(100_deg)
                 .WithTelemetryName("Arm");
        arm.emplace(&armConfig, &motor.value());
    }

    wpi::cmd::CommandPtr SetAngle(wpi::units::degree_t angle) { return arm->Run(angle); }

    void Periodic() override { arm->UpdateTelemetry(); }
    void SimulationPeriodic() override { arm->SimIterate(); }

private:
    rev::spark::SparkMax m_sparkMax{wpi::CANPort::CAN_S0, 1,
                                    rev::spark::SparkMax::MotorType::kBrushless};
    yams::motorcontrollers::SmartMotorControllerConfig motorConfig;
    std::optional<yams::motorcontrollers::local::SparkWrapper> motor;
    yams::mechanisms::config::ArmConfig armConfig;
    std::optional<yams::mechanisms::positional::Arm> arm;
};
```

## Next steps

* [Configuring a Motor Controller](guides/configuring-a-motor.md): CAN IDs, inversion, current limits, and encoder configuration
* [Building an Arm Subsystem](guides/building-an-arm.md): full arm setup with motion profiling and simulation
* [Building an Elevator Subsystem](guides/building-an-elevator.md)
* [Setting Up Swerve Drive](guides/setting-up-swerve.md)
