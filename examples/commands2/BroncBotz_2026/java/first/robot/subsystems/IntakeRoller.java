// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import first.robot.Ports;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.FlyWheel;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/** Over-the-bumper intake rollers. */
public class IntakeRoller extends SubsystemBase {
    private final SparkFlex motor = new SparkFlex(Ports.kCANPort, Ports.kIntakeRoller, MotorType.kBrushless);

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(0, 0, 0)
        .withGearing(new MechanismGearing(1))
        .withZeroPower(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(false)
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0.5, 0))
        // Simulation only: rough estimate of the rollers' inertia.
        .withMomentOfInertia(Inches.of(4), Pounds.of(1))
        .withTelemetry("IntakeRollerMotor", TelemetryVerbosity.HIGH);

    private final SmartMotorController motorController = new SparkWrapper(motor, DCMotor.getNeoVortex(1), motorConfig);

    private final FlyWheel intakeRoller = new FlyWheel(new FlyWheelConfig()
        .withTelemetry("IntakeRoller", TelemetryVerbosity.HIGH),
        motorController);

    public IntakeRoller() {
    }

    public void setDutyCycleSetpoint(double dutyCycle) {
        intakeRoller.setDutyCycleSetpoint(dutyCycle);
    }

    /** Run at a duty cycle until interrupted. */
    public Command runAt(double dutyCycle) {
        return intakeRoller.set(dutyCycle);
    }

    /** Stop. The default command. */
    public Command stop() {
        return intakeRoller.set(0);
    }

    @Override
    public void periodic() {
        intakeRoller.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        intakeRoller.simIterate();
    }
}
