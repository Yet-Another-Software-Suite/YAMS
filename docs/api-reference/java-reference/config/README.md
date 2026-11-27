# Configuration

Builder-pattern configuration classes for each mechanism type.

**Package:** `yams.core.mechanisms.config`

Mechanism configs do not need a Subsystem, so they only exist under `yams.core` and are used unchanged with both the Commands v2 (`yams.commands2`) and Commands v3 (`yams.commands3`) mechanisms. The moment of inertia and starting position are set on `SmartMotorControllerConfig`, not on these configs.
