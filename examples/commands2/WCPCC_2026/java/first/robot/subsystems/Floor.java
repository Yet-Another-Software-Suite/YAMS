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
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Voltage;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.FlyWheel;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Floor rollers that move fuel from the hopper toward the feeder. Open loop voltage, as a YAMS
 * {@link FlyWheel}.
 */
public class Floor extends SubsystemBase {
    public enum Speed {
        STOP(0),
        FEED(0.83);

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }

    private final TalonFX motor = new TalonFX(Ports.kFloor, Ports.kRoboRioCANBus);

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.OPEN_LOOP)
        .withGearing(new MechanismGearing(1.0))
        // Clockwise_Positive
        .withMotorInverted(true)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(120))
        .withSupplyCurrentLimit(Amps.of(30))
        // Simulation only: rough estimate of the floor rollers' inertia.
        .withMomentOfInertia(Inches.of(1), Pounds.of(1))
        .withTelemetry("FloorMotor", TelemetryVerbosity.HIGH);

    private final SmartMotorController motorController = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), motorConfig);

    // Roller diameter is an estimate; it only affects telemetry and the simulation display.
    private final FlyWheel floor = new FlyWheel(new FlyWheelConfig()
        .withDiameter(Inches.of(2))
        .withTelemetry("Floor", TelemetryVerbosity.HIGH),
        motorController);

    public Floor() {
    }

    public void set(Speed speed) {
        floor.setVoltageSetpoint(speed.voltage());
    }

    public Command feedCommand() {
        return startEnd(() -> set(Speed.FEED), () -> set(Speed.STOP));
    }

    @Override
    public void periodic() {
        floor.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        floor.simIterate();
    }
}
