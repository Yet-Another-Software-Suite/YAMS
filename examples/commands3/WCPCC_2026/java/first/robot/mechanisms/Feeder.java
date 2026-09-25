// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import first.robot.Constants.KrakenX60;
import first.robot.Ports;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularVelocity;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.FlyWheel;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.core.motorcontrollers.remote.TalonFXWrapper;

/**
 * Feeder roller that pushes fuel into the shooter. Runs closed loop velocity on the Talon, as a
 * YAMS {@link FlyWheel}.
 */
public class Feeder implements Mechanism {
    public enum Speed {
        FEED(5000);

        private final double rpm;

        private Speed(double rpm) {
            this.rpm = rpm;
        }

        public AngularVelocity angularVelocity() {
            return RPM.of(rpm);
        }
    }

    private final TalonFX motor = new TalonFX(Ports.kFeeder, Ports.kRoboRioCANBus);

    private final SmartMotorControllerConfig motorConfig = (SmartMotorControllerConfig) new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withGearing(new MechanismGearing(1.0))
        // CounterClockwise_Positive
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(120))
        .withSupplyCurrentLimit(Amps.of(50))
        .withClosedLoopController(1, 0, 0)
        // 12 volts when requesting max RPS
        .withFeedforward(new SimpleMotorFeedforward(0, 12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond)))
        // Simulation only: rough estimate of the feeder rollers' inertia.
        .withMomentOfInertia(Inches.of(1), Pounds.of(0.5))
        .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH);

    private final SmartMotorController motorController = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), motorConfig);

    // Roller diameter is an estimate; it only affects telemetry and the simulation display.
    private final FlyWheel feeder = new FlyWheel(new FlyWheelConfig()
        .withDiameter(Inches.of(2))
        .withTelemetry("Feeder", TelemetryVerbosity.HIGH),
        motorController);

    public Feeder() {
    }

    public void set(Speed speed) {
        feeder.setMechanismVelocitySetpoint(speed.angularVelocity());
    }

    public void setPercentOutput(double percentOutput) {
        feeder.setVoltageSetpoint(Volts.of(percentOutput * 12.0));
    }

    /** Run the feeder at feed speed until canceled, then stop it. */
    public Command feedCommand() {
        return run(coroutine -> {
            set(Speed.FEED);
            coroutine.park();
        })
        .whenCanceled(() -> setPercentOutput(0))
        .named("Feeder Feed");
    }

    /** Called from {@code Robot.robotPeriodic()}, replacing the v2 subsystem periodic. */
    public void periodic() {
        feeder.updateTelemetry();
    }

    /** Called from {@code Robot.simulationPeriodic()}. */
    public void simulationPeriodic() {
        feeder.simIterate();
    }
}
