# SensorConfig

The C++ API has no separate `SensorConfig` class. The C++ equivalent of the Java [`SensorConfig`](../../java-reference/config/sensor-config.md) is [`SimSensorConfig`](sim-sensor-config.md) (`yams::mechanisms::config::SimSensorConfig`, header `<yams/mechanisms/config/SimSensorConfig.hpp>`). It registers hardware-backed fields and simulated value overrides, and resolves to a [`Sensor`](../sensor/) via `GetSensor()`.

Absolute and external encoders are configured on the motor controller instead, through [`SmartMotorControllerConfig`](../motor-controllers/smart-motor-controller-config.md) (`WithExternalEncoder(...)`, `WithExternalEncoderInverted(...)`, `WithExternalEncoderZeroOffset(...)`).
