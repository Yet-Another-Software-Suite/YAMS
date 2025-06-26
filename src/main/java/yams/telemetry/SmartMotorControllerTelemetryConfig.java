package yams.telemetry;

import yams.motorcontrollers.SmartMotorController;

public class SmartMotorControllerTelemetryConfig {
    /**
     * Mechanism lower limit reached.
     */
    public boolean mechanismLowerLimitEnabled = false;
    /**
     * Mechanism upper limit reached.
     */
    public boolean mechanismUpperLimitEnabled = false;
    /**
     * Motor temperature cutoff reached.
     */
    public boolean temperatureLimitEnabled = false;
    /**
     * Velocity PID controller used.
     */
    public boolean velocityControlEnabled = false;
    /**
     * Elevator feedforward used.
     */
    public boolean elevatorFeedforwardEnabled = false;
    /**
     * Arm feedforward used.
     */
    public boolean armFeedforwardEnabled = false;
    /**
     * Simple feedforward used.
     */
    public boolean simpleFeedforwardEnabled = false;
    /**
     * Motion profiling used.
     */
    public boolean motionProfileEnabled = false;
    /**
     * Setpoint position given.
     */
    public boolean setpointPositionEnabled = false;
    /**
     * Setpoint velocity given.
     */
    public boolean setpointVelocityEnabled = false;
    /**
     * Feedforward voltage supplied to the {@link SmartMotorController}
     */
    public boolean feedforwardVoltageEnabled = false;
    /**
     * PID Output voltage supplied to the {@link SmartMotorController}
     */
    public boolean pidOutputVoltageEnabled = false;
    /**
     * Output voltage to the {@link SmartMotorController}
     */
    public boolean outputVoltageEnabled = false;
    /**
     * Stator current (motor controller output current) to the Motor.
     */
    public boolean statorCurrentEnabled = false;
    /**
     * Motor temperature.
     */
    public boolean temperatureEnabled = false;
    /**
     * Mechanism distance.
     */
    public boolean distanceEnabled = false;
    /**
     * Mechanism linear velocity.
     */
    public boolean linearVelocityEnabled = false;
    /**
     * Mechanism position.
     */
    public boolean mechanismPositionEnabled = false;
    /**
     * Mechanism velocity.
     */
    public boolean mechanismVelocityEnabled = false;
    /**
     * Rotor position.
     */
    public boolean rotorPositionEnabled = false;
    /**
     * Rotor velocity.
     */
    public boolean rotorVelocityEnabled = false;

    /**
     * Enables the mechanism lower limit logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withMechanismLowerLimit() {
        mechanismLowerLimitEnabled = true;
        return this;
    }

    /**
     * Enables the mechanism upper limit logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withMechanismUpperLimit() {
        mechanismUpperLimitEnabled = true;
        return this;
    }

    /**
     * Enables the temperature limit logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withTemperatureLimit() {
        temperatureLimitEnabled = true;
        return this;
    }

    /**
     * Enables the velocity control mode logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withVelocityControl() {
        velocityControlEnabled = true;
        return this;
    }

    /**
     * Enables the elevator feedforward logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withElevatorFeedforward() {
        elevatorFeedforwardEnabled = true;
        return this;
    }

    /**
     * Enables the arm feedforward logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withArmFeedforward() {
        armFeedforwardEnabled = true;
        return this;
    }

    /**
     * Enables the simple feedforward logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withSimpleFeedforward() {
        simpleFeedforwardEnabled = true;
        return this;
    }

    /**
     * Enables the motion profile logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withMotionProfile() {
        motionProfileEnabled = true;
        return this;
    }

    /**
     * Enables the setpoint position logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withSetpointPosition() {
        setpointPositionEnabled = true;
        return this;
    }

    /**
     * Enables the setpoint velocity logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withSetpointVelocity() {
        setpointVelocityEnabled = true;
        return this;
    }

    /**
     * Enables the feedforward voltage logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withFeedbackVoltage() {
        feedforwardVoltageEnabled = true;
        return this;
    }

    /**
     * Enables the pid output voltage logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withPidOutputVoltage() {
        pidOutputVoltageEnabled = true;
        return this;
    }

    /**
     * Enables the output voltage logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withOutputVoltage() {
        outputVoltageEnabled = true;
        return this;
    }

    /**
     * Enables the stator current logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withStatorCurrent() {
        statorCurrentEnabled = true;
        return this;
    }

    /**
     * Enables the temperature logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withTemperature() {
        temperatureEnabled = true;
        return this;
    }

    /**
     * Enables the distance logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withDistance() {
        distanceEnabled = true;
        return this;
    }

    /**
     * Enables the linear velocity logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withLinearVelocity() {
        linearVelocityEnabled = true;
        return this;
    }

    /**
     * Enables the mechanism position logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withMechanismPosition() {
        mechanismPositionEnabled = true;
        return this;
    }

    /**
     * Enables the mechanism velocity logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withMechanismVelocity() {
        mechanismVelocityEnabled = true;
        return this;
    }

    /**
     * Enables the rotor position logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withRotorPosition() {
        rotorPositionEnabled = true;
        return this;
    }

    /**
     * Enables the rotor velocity logging if available.
     *
     * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
     */
    public SmartMotorControllerTelemetryConfig withRotorVelocity() {
        rotorVelocityEnabled = true;
        return this;
    }
}
