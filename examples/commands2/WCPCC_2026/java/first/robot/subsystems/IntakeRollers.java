// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import first.robot.Ports;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Voltage;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.FlyWheel;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.core.motorcontrollers.remote.TalonFXWrapper;

/**
 * Intake rollers, modeled as a YAMS {@link FlyWheel} driven open loop. The pivot is a separate
 * subsystem ({@link IntakePivot}) so each mechanism can be tuned live on its own.
 */
public class IntakeRollers extends SubsystemBase {
    public enum Speed {
        STOP(0),
        INTAKE(0.8);

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }

    private final TalonFX rollerMotor = new TalonFX(Ports.kIntakeRollers, Ports.kRoboRioCANBus);

    private final SmartMotorControllerConfig rollerConfig = (SmartMotorControllerConfig) new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.OPEN_LOOP)
        .withGearing(new MechanismGearing(1.0))
        // Clockwise_Positive
        .withMotorInverted(true)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(120))
        .withSupplyCurrentLimit(Amps.of(70))
        // Simulation only: rough estimate of the intake rollers' inertia.
        .withMomentOfInertia(Inches.of(1), Pounds.of(1))
        .withTelemetry("IntakeRollerMotor", TelemetryVerbosity.HIGH);

    private final SmartMotorController rollerMotorController = new TalonFXWrapper(rollerMotor, DCMotor.getKrakenX60(1), rollerConfig);

    // Roller diameter is an estimate; it only affects telemetry and the simulation display.
    private final FlyWheel rollers = new FlyWheel(new FlyWheelConfig()
        .withDiameter(Inches.of(2))
        .withTelemetry("IntakeRollers", TelemetryVerbosity.HIGH),
        rollerMotorController);

    public IntakeRollers() {
    }

    public void set(Speed speed) {
        rollers.setVoltageSetpoint(speed.voltage());
    }

    @Override
    public void periodic() {
        rollers.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        rollers.simIterate();
    }
}
