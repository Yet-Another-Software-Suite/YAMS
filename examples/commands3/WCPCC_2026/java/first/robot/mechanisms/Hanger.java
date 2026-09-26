// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import first.robot.Constants;
import first.robot.Constants.KrakenX60;
import first.robot.Ports;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.controller.ElevatorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.Elevator;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ElevatorConfig;
import yams.core.mechanisms.config.SensorConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.motorcontrollers.simulation.Sensor;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Hanger that extends to reach the tower and retracts to lift the robot, modeled as a YAMS
 * {@link Elevator}.
 */
public class Hanger implements Mechanism {
    public enum Position {
        HOMED(0),
        EXTEND_HOPPER(2),
        HANGING(6),
        HUNG(0.2);

        private final double inches;

        private Position(double inches) {
            this.inches = inches;
        }

        public Distance extension() {
            return Inches.of(inches);
        }
    }

    // The hanger extends 6 inches per 142 motor rotations. YAMS models that as a 142:1 reduction
    // onto a mechanism whose single rotation is 6 inches of travel.
    private static final double kMotorRotationsPerMechanismRotation = 142;
    private static final Distance kExtensionPerMechanismRotation = Inches.of(6);
    private static final AngularVelocity kMaxMechanismSpeed = KrakenX60.kFreeSpeed.div(kMotorRotationsPerMechanismRotation);
    private static final Distance kExtensionTolerance = Inches.of(1);

    private final TalonFX motor = new TalonFX(Ports.kHanger, Ports.kRoboRioCANBus);

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withGearing(new MechanismGearing(kMotorRotationsPerMechanismRotation))
        .withMechanismCircumference(kExtensionPerMechanismRotation)
        // Clockwise_Positive
        .withMotorInverted(true)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(20))
        .withSupplyCurrentLimit(Amps.of(70))
        // WCP's gains were per motor rotation (kP 10, kV 12 V at free speed); YAMS closes the loop
        // per mechanism rotation, so both are scaled by the 142:1 reduction.
        .withClosedLoopController(10 * kMotorRotationsPerMechanismRotation, 0, 0)
        .withFeedforward(new ElevatorFeedforward(0, 0, 12.0 / kMaxMechanismSpeed.in(RotationsPerSecond)))
        // Motion Magic cruise at motor free speed, accelerating to it in one second.
        .withTrapezoidalProfile(kMaxMechanismSpeed, kMaxMechanismSpeed.per(Second))
        // Start the simulated hanger off its bottom stop so homing has to drive into it.
        .withSimStartingPosition(Position.EXTEND_HOPPER.extension())
        .withTelemetry("HangerMotor", TelemetryVerbosity.HIGH);

    private final SmartMotorController motorController = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), motorConfig);

    private final Elevator hanger = new Elevator(new ElevatorConfig()
        .withHardLimits(Inches.of(0), Inches.of(7))
        // Simulation only: rough estimate of the hanger's moving mass.
        .withCarriageWeight(Pounds.of(2))
        .withTelemetry("Hanger", TelemetryVerbosity.HIGH),
        motorController);

    // Homing detects the bottom stop from the hanger's supply current. The simulator does not model
    // the current rise against a hard stop, so the sensor reports a spike once the hanger reaches it.
    private final Sensor currentSensor = new SensorConfig("HangerCurrent")
        .withField("SupplyAmps", () -> motorController.getSupplyCurrent().map(current -> current.in(Amps)).orElse(0.0), 0.0)
        .withSimulatedValue("SupplyAmps", () -> hanger.isNear(Position.HOMED.extension(), Inches.of(0.1)), 1.0)
        .getSensor();

    private boolean isHomed = false;

    public Hanger() {
    }

    public void set(Position position) {
        hanger.setMeasurementPositionSetpoint(position.extension());
    }

    public void setPercentOutput(double percentOutput) {
        hanger.setVoltageSetpoint(Volts.of(percentOutput * 12.0));
    }

    /**
     * Move the hanger to a position, finishing once it has been within tolerance for 0.1 s, using the
     * YAMS height command. The closed loop keeps holding it after the command ends.
     */
    public Command moveTo(Position position) {
        return hanger.runTo(position.extension(), kExtensionTolerance);
    }

    /**
     * Retract gently until the current rises against the bottom stop, then zero the encoder there.
     * Does nothing once homed. It runs above the default priority, so other hanger commands cannot
     * interrupt it, like v2's {@code CANCEL_INCOMING}.
     */
    public Command home() {
        return run(coroutine -> {
            if (isHomed) {
                return;
            }
            setPercentOutput(-0.05);
            coroutine.waitUntil(() -> currentSensor.getAsDouble("SupplyAmps") > 0.4);
            motorController.setEncoderPosition(Position.HOMED.extension());
            isHomed = true;
            set(Position.EXTEND_HOPPER);
        })
        .withPriority(Constants.kHomingPriority)
        .named("Hanger Homing");
    }

    public boolean isHomed() {
        return isHomed;
    }

    /** Called from {@code Robot.robotPeriodic()}, replacing the v2 subsystem periodic. */
    public void periodic() {
        hanger.updateTelemetry();
    }

    /** Called from {@code Robot.simulationPeriodic()}. */
    public void simulationPeriodic() {
        hanger.simIterate();
    }
}
