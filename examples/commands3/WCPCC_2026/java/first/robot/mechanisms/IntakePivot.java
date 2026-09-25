// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
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
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.Arm;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.mechanisms.config.SensorConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.motorcontrollers.simulation.Sensor;

/**
 * Intake pivot that swings the over-the-bumper intake out, modeled as a YAMS {@link Arm}. The
 * rollers are a separate mechanism ({@link IntakeRollers}) so each mechanism can be tuned live on
 * its own.
 */
public class IntakePivot implements Mechanism {
    public enum Position {
        HOMED(110),
        STOWED(100),
        INTAKE(-4),
        AGITATE(20);

        private final double degrees;

        private Position(double degrees) {
            this.degrees = degrees;
        }

        public Angle angle() {
            return Degrees.of(degrees);
        }
    }

    private static final double kPivotReduction = 50.0;
    private static final AngularVelocity kMaxPivotSpeed = KrakenX60.kFreeSpeed.div(kPivotReduction);
    private static final Angle kPositionTolerance = Degrees.of(5);

    private final TalonFX pivotMotor = new TalonFX(Ports.kIntakePivot, Ports.kCANivoreCANBus);

    private final SmartMotorControllerConfig pivotConfig = (SmartMotorControllerConfig) new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withGearing(new MechanismGearing(kPivotReduction))
        // CounterClockwise_Positive
        .withMotorInverted(false)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(120))
        .withSupplyCurrentLimit(Amps.of(70))
        .withClosedLoopController(300, 0, 0)
        // 12 volts when requesting max RPS; no gravity term, the 50:1 reduction and kP hold the pivot.
        .withFeedforward(new ArmFeedforward(0, 0, 12.0 / kMaxPivotSpeed.in(RotationsPerSecond)))
        // Motion Magic cruise velocity and acceleration, as in the original.
        .withTrapezoidalProfile(kMaxPivotSpeed, kMaxPivotSpeed.per(Second))
        // Start the simulated pivot off its hard stop so homing has to drive into it.
        .withSimStartingPosition(Position.STOWED.angle())
        // Simulation only: rough estimate of the intake's inertia about the pivot.
        .withMomentOfInertia(Inches.of(12), Pounds.of(6))
        .withTelemetry("IntakePivotMotor", TelemetryVerbosity.HIGH);

    private final SmartMotorController pivotMotorController = new TalonFXWrapper(pivotMotor, DCMotor.getKrakenX60(1), pivotConfig);

    private final Arm pivot = new Arm(new ArmConfig()
        // The upper hard stop is the homing position.
        .withHardLimits(Degrees.of(-10), Position.HOMED.angle())
        // Length is an estimate; it only affects the simulation display.
        .withLength(Inches.of(12))
        .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH),
        pivotMotorController);

    // Homing detects the hard stop from the pivot's supply current. The simulator does not model the
    // current rise against a hard stop, so the sensor reports a spike once the pivot reaches it.
    private final Sensor pivotCurrentSensor = new SensorConfig("IntakePivotCurrent")
        .withField("SupplyAmps", () -> pivotMotorController.getSupplyCurrent().map(current -> current.in(Amps)).orElse(0.0), 0.0)
        .withSimulatedValue("SupplyAmps", () -> pivot.isNear(Position.HOMED.angle(), Degrees.of(1)), 10.0)
        .getSensor();

    private boolean isHomed = false;

    public IntakePivot() {
    }

    public boolean isPositionWithinTolerance() {
        final Angle currentPosition = pivot.getAngle();
        final Angle targetPosition = pivotMotorController.getMechanismPositionSetpoint().orElse(currentPosition);
        return currentPosition.isNear(targetPosition, kPositionTolerance);
    }

    private void setPercentOutput(double percentOutput) {
        pivot.setVoltageSetpoint(Volts.of(percentOutput * 12.0));
    }

    public void set(Position position) {
        pivot.setMechanismPositionSetpoint(position.angle());
    }

    /** Move the pivot to a position. The closed loop keeps holding it after the command ends. */
    public Command positionCommand(Position position) {
        return run(coroutine -> set(position)).named("IntakePivot to " + position);
    }

    /**
     * Drive the pivot into its upper hard stop until the current spikes, then zero the encoder there.
     * Does nothing once homed. It runs above the default priority, so other pivot commands (and any
     * command that requires the pivot) cannot interrupt it, like v2's {@code CANCEL_INCOMING}.
     */
    public Command homingCommand() {
        return run(coroutine -> {
            if (isHomed) {
                return;
            }
            setPercentOutput(0.1);
            coroutine.waitUntil(() -> pivotCurrentSensor.getAsDouble("SupplyAmps") > 6);
            pivotMotorController.setEncoderPosition(Position.HOMED.angle());
            isHomed = true;
            set(Position.STOWED);
        })
        .withPriority(Constants.kHomingPriority)
        .named("IntakePivot Homing");
    }

    /** Called from {@code Robot.robotPeriodic()}, replacing the v2 subsystem periodic. */
    public void periodic() {
        pivot.updateTelemetry();
    }

    /** Called from {@code Robot.simulationPeriodic()}. */
    public void simulationPeriodic() {
        pivot.simIterate();
    }
}
