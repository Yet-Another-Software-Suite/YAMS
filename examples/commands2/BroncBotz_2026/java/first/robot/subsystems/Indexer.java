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

/** Indexer that feeds fuel to the kicker, or holds it back while the shooter spins up. */
public class Indexer extends SubsystemBase {
    private final SparkMax motor = new SparkMax(Ports.kCANPort, Ports.kIndexer, MotorType.kBrushless);

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(0, 0, 0)
        .withGearing(new MechanismGearing(3))
        .withZeroPower(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(false)
        .withFeedforward(new SimpleMotorFeedforward(0.18, 0.62, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0.5, 0))
        // Simulation only: rough estimate of the rollers' inertia.
        .withMomentOfInertia(Inches.of(4), Pounds.of(1))
        .withTelemetry("IndexerMotor", TelemetryVerbosity.HIGH);

    private final SmartMotorController motorController = new SparkWrapper(motor, DCMotor.getNEO(1), motorConfig);

    private final FlyWheel indexer = new FlyWheel(new FlyWheelConfig()
        .withTelemetry("Indexer", TelemetryVerbosity.HIGH),
        motorController);

    public Indexer() {
    }

    public AngularVelocity getSpeed() {
        return indexer.getSpeed();
    }

    public void setVelocitySetpoint(AngularVelocity velocity) {
        indexer.setMechanismVelocitySetpoint(velocity);
    }

    public void setDutyCycleSetpoint(double dutyCycle) {
        indexer.setDutyCycleSetpoint(dutyCycle);
    }

    /** Run at a duty cycle until interrupted. */
    public Command runAt(double dutyCycle) {
        return indexer.set(dutyCycle);
    }

    /** Stop. The default command. */
    public Command stop() {
        return indexer.set(0);
    }

    @Override
    public void periodic() {
        indexer.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        indexer.simIterate();
    }
}
