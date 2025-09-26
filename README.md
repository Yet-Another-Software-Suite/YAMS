# YAMS - Yet Another Mechanism System
[![Documentation](https://github.com/Yet-Another-Software-Suite/YAMS/actions/workflows/build-pdf.yml/badge.svg)](https://github.com/Yet-Another-Software-Suite/YAMS/actions/workflows/build-pdf.yml) [![CI](https://github.com/Yet-Another-Software-Suite/YAMS/actions/workflows/ci.yml/badge.svg)](https://github.com/Yet-Another-Software-Suite/YAMS/actions/workflows/ci.yml) [![Release](https://github.com/Yet-Another-Software-Suite/YAMS/actions/workflows/release.yml/badge.svg)](https://github.com/Yet-Another-Software-Suite/YAMS/actions/workflows/release.yml)

> ✨ A flexible, extensible FRC mechanism library built for elevators, arms, turrets, and more — with simulation and telemetry included.

**YAMS** is a WPILib-compatible library that provides a **unified and extensible interface** for common FRC mechanisms like **elevators**, **arms**, and **pivots** (e.g., turrets). It emphasizes clean separation of control, simulation, and configuration, while offering first-class support for **telemetry**, **feedforward**, and **tuning**.

---

## 🔧 Key Features

- 🧠 Unified interfaces for `Arm`, `Elevator`, and `Pivot` mechanisms  
- ⚙️ CTRE-style configuration: familiar and readable  
- 🛠️ SmartMotorController abstraction: consistent API for different motor vendors (REV, CTRE, etc.)  
- 🧪 Physics-based simulation support (`simIterate()`)  
- 📊 Built-in telemetry (works with AdvantageKit, NT, and custom logging)  
- 🔁 Composable and declarative configuration style  

---

## 📦 Installation (WPILib Vendordep)

1. In **VS Code** with WPILib extension:
   - Open Command Palette (`Ctrl+Shift+P` / `Cmd+Shift+P`)
   - Select: `WPILib: Manage Vendor Libraries`
   - Choose `Install new library (online or offline)`
   - Select **Online**
   - Paste the URL to the YAMS vendordep JSON file, e.g.:  
     `https://yet-another-software-suite.github.io/YAMS/yams.json`
   - Press Enter to install

---


## 📂 Examples

The repository contains several example projects under the `/examples` folder demonstrating how to use YAMS for arms, elevators, pivots, and combined subsystems.

These example projects do **not** include YAMS as a dependency via Maven or vendordep directly. Instead, they use a modified `build.gradle` that links the YAMS source code located in the `/yams` folder relative to the example.

This is done by adding the following snippet to the `sourceSets` block in each example’s `build.gradle`:

```groovy
sourceSets {
    main {
        java {
            srcDirs 'src/main/java'
            srcDirs '../../yams/'
        }
    }
}
```

---
## 🚀 Quick Example

Here’s a simplified `ArmSubsystem` using a wrapped `TalonFX`:

```java
public class ArmSubsystem extends SubsystemBase {
  private final TalonFXS armMotor = new TalonFXS(1);
  private final SmartMotorController motor = new TalonFXSWrapper(armMotor, DCMotor.getNEO(1),
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
          .withSoftLimit(Degrees.of(-30), Degrees.of(100))
          .withGearing(gearing(gearbox(3, 4)))
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
          .withControlMode(ControlMode.CLOSED_LOOP)
  );

  private final Arm arm = new Arm(new ArmConfig(motor)
      .withLength(Meters.of(0.135))
      .withHardLimit(Degrees.of(-100), Degrees.of(200))
      .withStartingPosition(Degrees.of(0))
      .withTelemetry("ArmExample", TelemetryVerbosity.HIGH)
  );

  @Override
  public void periodic() {
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    arm.simIterate();
  }

  public Command setAngle(Angle angle) {
    return arm.setAngle(angle);
  }
}
````

More detailed examples for **Elevator** and **Pivot** mechanisms can be found in the `/examples` folder (or your team’s repo if it includes all three like above).

---

## 🧪 Simulation

Just call `simIterate()` inside `simulationPeriodic()` to simulate the mechanism’s physical behavior — based on voltage inputs, mass, length, and more.

---

## 📊 Telemetry

YAMS supports telemetry via its internal telemetry interfaces and `SmartMotorControllerTelemetryConfig`.

It integrates with:

* [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit)
* Custom logging tools
* NetworkTables `/Mechanisms` and `/Tuning`

---

## 📜 License

**This project is licensed under the Lesser GNU General Public License v3.0**.
You are free to use, modify, and redistribute the software, provided that any derivative work is also licensed under LGPLv3.

See [`LICENSE`](./LICENSE.txt) for full details.

---

## 🤝 Contributing

We welcome feedback and contributions!
Open an issue for bug reports or feature requests, or fork and open a pull request to contribute.

Inspired by [Manip-Lib](https://github.com/frc5517/Manip-Lib). 

---

