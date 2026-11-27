# Overview

YAMS (Yet Another Mechanism Suite) is a WPILib vendordep library for FRC robots. It provides a unified motor controller abstraction over REV, CTRE, Thrifty, and FIRST hardware, pre-built mechanism classes for common robot subsystems, and advanced control algorithms, all with simulation support and automatic NetworkTables telemetry. Both Java and C++ are supported with parallel APIs.

This documentation tracks YAMS for WPILib 2027 (`2027_alpha7`), where robot code runs on the SystemCore. Java command bindings ship for both WPILib Commands v2 (`yams.commands2`) and Commands v3 (`yams.commands3`); C++ supports Commands v2. See [Getting Started](getting-started.md) for installation.

## Features

* **Motor controller abstraction:** a single `SmartMotorController` interface over SPARK MAX, SPARK FLEX, TalonFX, TalonFXS, Thrifty Nova, and FIRST A301
* **Mechanism classes:** `Arm`, `Elevator`, `Pivot`, `FlyWheel`, `DoubleJointedArm`, `DifferentialMechanism`, and `SwerveDrive` (`DoubleJointedArm` and `DifferentialMechanism` are Java only)
* **Advanced control:** LQR, exponential motion profiles, trapezoidal profiles, and a derivative-time filter
* **Extended units:** `YUnits` constants and `EasyCRT` multi-encoder absolute position solver
* **Simulation:** built-in sim support for all mechanism classes via WPILib's simulation framework, with one shared `BatterySim` across every motor controller
* **Commands v2 and v3:** the same mechanisms with `Command` and `Trigger` factories for either WPILib command framework (Java)

## Quick navigation

* [Java API](java-reference/java.md)
* [C++ API](c++-reference/cpp.md)
* [Guides](guides/guides.md)

## Supported hardware

| Device     | Vendor          | Interface |
| ---------- | --------------- | --------- |
| SPARK MAX  | REV Robotics    | CAN       |
| SPARK FLEX | REV Robotics    | CAN       |
| TalonFX    | CTR Electronics | CAN       |
| TalonFXS   | CTR Electronics | CAN       |
| Nova       | Thrifty Bot     | CAN       |
| A301       | FIRST           | CAN       |
