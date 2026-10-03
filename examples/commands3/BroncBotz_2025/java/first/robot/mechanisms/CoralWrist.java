// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.mechanisms;

import static first.robot.Constants.CoralIntakeConstants.*;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Rotations;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Ports;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.Pivot;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.PivotConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Wrist of the coral intake at the end of the coral arm, as a YAMS {@link Pivot} closing its loop on
 * a through bore encoder. It rests out of the algae arm's way, and swings out to score.
 */
public class CoralWrist implements Mechanism {
    private final SparkMax motor = new SparkMax(Ports.kCANBus, Ports.kCoralWrist, MotorType.kBrushless);

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withGearing(new MechanismGearing(kWristGearRatio))
        .withClosedLoopController(kWristKP, 0, 0)
        // The SPARK closes the loop on the through bore encoder, wrapping over its [0, 1) range.
        .withExternalEncoder(motor.getAbsoluteEncoder())
        .withUseExternalFeedbackEncoder(true)
        .withExternalEncoderInverted(true)
        .withExternalEncoderDiscontinuityPoint(Rotations.of(1))
        .withContinuousWrapping(Rotations.of(0), Rotations.of(1))
        .withZeroPower(MotorMode.COAST)
        .withStatorCurrentLimit(kWristCurrentLimit)
        .withClosedLoopRampRate(kWristRampRate)
        .withMotorInverted(false)
        .withSimStartingPosition(kRest)
        .withMomentOfInertia(KilogramSquareMeters.of(kWristMomentOfInertia))
        .withTelemetry("CoralWristMotor", TelemetryVerbosity.HIGH);

    private final SmartMotorController motorController = new SparkWrapper(motor, DCMotor.getNEO(1), motorConfig);

    private final Pivot wrist = new Pivot(new PivotConfig()
        .withHardLimits(Rotations.of(0), Rotations.of(1))
        .withTelemetry("CoralWrist", TelemetryVerbosity.HIGH),
        motorController);

    public CoralWrist() {
    }

    /** The wrist is at rest, out of the algae arm's way. */
    public boolean isAtRest() {
        return motorController.getMechanismPosition().isNear(kRest, kScoringTolerance);
    }

    /** The wrist is at its scoring angle. */
    public boolean isAtScoringAngle() {
        return motorController.getMechanismPosition().isNear(kActive, kScoringTolerance);
    }

    /** Hold an angle until canceled. */
    public Command holdAt(Angle angle) {
        return wrist.setAngle(angle);
    }

    /** Rest out of the algae arm's way, until canceled. The default command. */
    public Command rest() {
        return holdAt(kRest);
    }

    /** Swing out to the scoring angle, until canceled. */
    public Command swingOut() {
        return holdAt(kActive);
    }

    /** Called from {@code Robot.robotPeriodic()}, replacing the v2 subsystem periodic. */
    public void periodic() {
        wrist.updateTelemetry();
    }

    /** Called from {@code Robot.simulationPeriodic()}. */
    public void simulationPeriodic() {
        wrist.simIterate();
    }
}
