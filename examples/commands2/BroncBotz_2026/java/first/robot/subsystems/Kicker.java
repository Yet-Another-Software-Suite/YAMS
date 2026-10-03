// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Ports;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularVelocity;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.FlyWheel;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/** Kicker wheels that push fuel from the indexer into the shooter. */
public class Kicker extends SubsystemBase {
    private final SparkMax motor = new SparkMax(Ports.kCANPort, Ports.kKicker, MotorType.kBrushless);

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(0, 0, 0)
        .withGearing(new MechanismGearing(3))
        .withZeroPower(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(true)
        .withFeedforward(new SimpleMotorFeedforward(0.18, 0.62, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0.5, 0))
        // Simulation only: rough estimate of the rollers' inertia.
        .withMomentOfInertia(Inches.of(4), Pounds.of(1))
        .withTelemetry("KickerMotor", TelemetryVerbosity.HIGH);

    private final SmartMotorController motorController = new SparkWrapper(motor, DCMotor.getNEO(1), motorConfig);

    private final FlyWheel kicker = new FlyWheel(new FlyWheelConfig()
        .withTelemetry("Kicker", TelemetryVerbosity.HIGH),
        motorController);

    public Kicker() {
    }

    public AngularVelocity getSpeed() {
        return kicker.getSpeed();
    }

    public void setVelocitySetpoint(AngularVelocity velocity) {
        kicker.setMechanismVelocitySetpoint(velocity);
    }

    public void setDutyCycleSetpoint(double dutyCycle) {
        kicker.setDutyCycleSetpoint(dutyCycle);
    }

    /** Run at a duty cycle until interrupted. */
    public Command runAt(double dutyCycle) {
        return kicker.set(dutyCycle);
    }

    /** Stop. The default command. */
    public Command stop() {
        return kicker.set(0);
    }

    @Override
    public void periodic() {
        kicker.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        kicker.simIterate();
    }
}
