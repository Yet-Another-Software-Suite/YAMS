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
import java.util.function.BooleanSupplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.FlyWheel;
import yams.commands3.mechanisms.Pivot;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.mechanisms.config.PivotConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Coral intake at the end of the coral arm: a wrist, as a YAMS {@link Pivot} closing its loop on a
 * through bore encoder, and a roller, as an open loop YAMS {@link FlyWheel}. They are one subsystem
 * because every command sets both, as in the original.
 */
public class CoralIntake implements Mechanism {
    private final SparkMax wristMotor = new SparkMax(Ports.kCANBus, Ports.kCoralWrist, MotorType.kBrushless);
    private final SparkMax rollerMotor = new SparkMax(Ports.kCANBus, Ports.kCoralRoller, MotorType.kBrushless);

    private final SmartMotorControllerConfig wristConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withGearing(new MechanismGearing(kWristGearRatio))
        .withClosedLoopController(kWristKP, 0, 0)
        // The SPARK closes the loop on the through bore encoder, wrapping over its [0, 1) range.
        .withExternalEncoder(wristMotor.getAbsoluteEncoder())
        .withUseExternalFeedbackEncoder(true)
        .withExternalEncoderInverted(true)
        .withExternalEncoderDiscontinuityPoint(Rotations.of(1))
        .withContinuousWrapping(Rotations.of(0), Rotations.of(1))
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(kWristCurrentLimit)
        .withClosedLoopRampRate(kWristRampRate)
        .withMotorInverted(false)
        .withSimStartingPosition(kRest)
        .withMomentOfInertia(KilogramSquareMeters.of(kWristMomentOfInertia))
        .withTelemetry("CoralWristMotor", TelemetryVerbosity.HIGH);

    private final SmartMotorController wristController = new SparkWrapper(wristMotor, DCMotor.getNEO(1), wristConfig);

    private final Pivot wrist = new Pivot(new PivotConfig()
        .withHardLimits(Rotations.of(0), Rotations.of(1))
        .withTelemetry("CoralWrist", TelemetryVerbosity.HIGH),
        wristController);

    private final SmartMotorControllerConfig rollerConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.OPEN_LOOP)
        .withGearing(new MechanismGearing(1))
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(kRollerCurrentLimit)
        .withMotorInverted(true)
        .withMomentOfInertia(KilogramSquareMeters.of(kWristMomentOfInertia))
        .withTelemetry("CoralRollerMotor", TelemetryVerbosity.HIGH);

    private final SmartMotorController rollerController = new SparkWrapper(rollerMotor, DCMotor.getNEO(1), rollerConfig);

    private final FlyWheel roller = new FlyWheel(new FlyWheelConfig()
        .withTelemetry("CoralRoller", TelemetryVerbosity.HIGH),
        rollerController);


    public CoralIntake() {
    }

    private void set(Angle wristAngle, double rollerDutyCycle) {
        wrist.setMechanismPositionSetpoint(wristAngle);
        roller.setDutyCycleSetpoint(rollerDutyCycle);
    }

    /** The wrist is at rest, out of the algae arm's way. */
    public boolean isAtRest() {
        return wrist.getMotorController().getMechanismPosition().isNear(kRest, kScoringTolerance);
    }

    /** The wrist is at its scoring angle. */
    public boolean isAtScoringAngle() {
        return wrist.getMotorController().getMechanismPosition().isNear(kActive, kScoringTolerance);
    }

    /** Hold the wrist at an angle and run the roller, until canceled. */
    private Command run(Angle wristAngle, double rollerDutyCycle, String name) {
        return run(coroutine -> {
            while (true) {
                set(wristAngle, rollerDutyCycle);
                coroutine.yield();
            }
        }).named(name);
    }

    /** Wrist at rest, intaking from the human player station. */
    public Command intake() {
        return run(kRest, kIntake, "CoralIntake Intake");
    }

    /** Wrist at its scoring angle, holding the coral in. */
    public Command score() {
        return run(kActive, kScore, "CoralIntake Score");
    }

    /** Wrist at its scoring angle, spitting the coral out. */
    public Command outtake() {
        return run(kActive, kOuttake, "CoralIntake Outtake");
    }

    /** Wrist at rest, spitting the coral out. */
    public Command spit() {
        return run(kRest, kSpit, "CoralIntake Spit");
    }

    /** Wrist at rest, roller stopped. */
    public Command rest() {
        return run(kRest, 0, "CoralIntake Rest");
    }

    /** Hold the wrist at an angle without the roller. */
    public Command holdWrist(Angle wristAngle) {
        return run(coroutine -> {
            while (true) {
                wrist.setMechanismPositionSetpoint(wristAngle);
                coroutine.yield();
            }
        }).named("CoralWrist to " + wristAngle);
    }

    /** Run the roller only, at full speed. */
    public Command rollerFull() {
        return run(coroutine -> {
            while (true) {
                roller.setDutyCycleSetpoint(kFull);
                coroutine.yield();
            }
        }).named("CoralRoller Full");
    }

    /** Wrist at rest, gently holding a coral while one is loaded. The default command. */
    public Command hold(BooleanSupplier coralLoaded) {
        return run(coroutine -> {
            while (true) {
                set(kRest, coralLoaded.getAsBoolean() ? kHold : 0);
                coroutine.yield();
            }
        }).withPriority(Command.LOWEST_PRIORITY).named("CoralIntake Hold");
    }

    public double getRollerDutyCycle() {
        return rollerController.getDutyCycle();
    }

    /** Called from {@code Robot.robotPeriodic()}, replacing the v2 subsystem periodic. */
    public void periodic() {
        wrist.updateTelemetry();
        roller.updateTelemetry();
    }

    /** Called from {@code Robot.simulationPeriodic()}. */
    public void simulationPeriodic() {
        wrist.simIterate();
        roller.simIterate();
    }
}
