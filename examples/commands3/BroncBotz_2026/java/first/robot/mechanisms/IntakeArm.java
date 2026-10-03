// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.mechanisms;

import static first.robot.Constants.IntakeConstants.*;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Ports;
import java.util.function.DoubleSupplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Angle;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.Arm;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Intake arm that swings the intake out over the bumper, modeled as a YAMS {@link Arm}. It has a
 * NEO on each side; the right one is a loosely coupled follower, so each SPARK runs its own position
 * loop, and the operator can drive the two sides separately.
 */
public class IntakeArm implements Mechanism {
    private static final MechanismGearing kGearing = new MechanismGearing(kArmGearRatio);

    private final SparkMax followerMotor = new SparkMax(Ports.kCANPort, Ports.kIntakeArmFollower, MotorType.kBrushless);
    private final SmartMotorController followerController = new SparkWrapper(followerMotor, DCMotor.getNEO(2),
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(2, 0, 2)
            .withFeedforward(new ArmFeedforward(0.01, 0.02, 0, 0))
            .withSimClosedLoopController(10, 0, 0)
            .withSimFeedforward(new ArmFeedforward(0.25, 0, 0.25))
            .withGearing(kGearing)
            .withZeroPower(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(35))
            .withMotorInverted(false)
            .withStartingPosition(kArmStart)
            .withResetPreviousConfig(true)
            .withTelemetry("IntakeArmFollowerMotor", TelemetryVerbosity.HIGH));

    private final SparkMax leaderMotor = new SparkMax(Ports.kCANPort, Ports.kIntakeArmLeader, MotorType.kBrushless);
    private final SmartMotorController leaderController = new SparkWrapper(leaderMotor, DCMotor.getNEO(2),
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(2, 0, 2)
            .withFeedforward(new ArmFeedforward(0.01, 0.02, 0, 0))
            .withSimClosedLoopController(10, 0, 0)
            .withSimFeedforward(new ArmFeedforward(0.25, 0, 0.25))
            .withGearing(kGearing)
            .withZeroPower(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40))
            .withMotorInverted(true)
            .withStartingPosition(kArmStart)
            .withResetPreviousConfig(true)
            // Simulation only: the arm's length and mass.
            .withMomentOfInertia(kArmLength, kArmMass)
            .withLooselyCoupledFollowers(followerController)
            .withTelemetry("IntakeArmMotor", TelemetryVerbosity.LOW));

    private final Arm arm = new Arm(new ArmConfig()
        .withHardLimits(kArmMin, kArmMax)
        .withLength(kArmLength)
        .withTelemetry("IntakeArm", TelemetryVerbosity.HIGH),
        leaderController);

    public IntakeArm() {
    }

    public Angle getAngle() {
        return arm.getAngle();
    }

    /** Set the angle both sides hold. The follower is sent the same setpoint. */
    private void set(Angle angle) {
        arm.setMechanismPositionSetpoint(angle);
    }

    /** Move to an angle and hold it, finishing after {@code seconds}. */
    public Command moveTo(Angle angle, double seconds, String name) {
        return run(coroutine -> {
            set(angle);
            coroutine.wait(Seconds.of(seconds));
        }).named(name);
    }

    /** Swing the intake out to its intake angle. Operator left bumper. */
    public Command toIntake() {
        return moveTo(kArmIntake, 1.3, "IntakeArm to Intake");
    }

    /** Swing the intake up. Operator D-pad up. */
    public Command toUp() {
        return moveTo(kArmUp, 1.3, "IntakeArm Up");
    }

    /** Lower the intake, slowing 20° above the bottom. Operator D-pad down. */
    public Command toDown() {
        return run(coroutine -> {
            set(kArmDown.plus(Degrees.of(20)));
            coroutine.wait(Seconds.of(0.5));
            set(kArmDown);
            coroutine.wait(Seconds.of(1.3));
        }).named("IntakeArm Down");
    }

    /** Autonomous "ArmUp": raise the intake to shake fuel toward the indexer while shooting. */
    public Command wiggleUp() {
        return moveTo(kArmWiggleUp, 0.5, "IntakeArm Wiggle Up");
    }

    /** Autonomous "ArmDown": drop the intake back down after {@link #wiggleUp()}. */
    public Command wiggleDown() {
        return moveTo(kArmWiggleDown, 1.2, "IntakeArm Wiggle Down");
    }

    /**
     * Rock the intake to push fuel toward the indexer, until canceled. Operator right bumper. Each
     * rock drops to just above the bottom, swings up after 0.3 s, and pauses 0.3 s once it is up,
     * within 1.2 s in total.
     */
    public Command agitate() {
        return run(coroutine -> {
            final Timer rockTimer = new Timer();
            while (true) {
                rockTimer.restart();
                set(kArmDown.plus(Degrees.of(10)));
                coroutine.wait(Seconds.of(0.3));
                set(kArmUp);
                coroutine.waitUntil(() -> arm.isNear(kArmUp, kAgitateTolerance), Seconds.of(0.7));
                final double remaining = 1.2 - rockTimer.get();
                if (remaining > 0) {
                    coroutine.wait(Seconds.of(Math.min(0.3, remaining)));
                }
            }
        }).named("IntakeArm Agitate");
    }

    /**
     * Drive each side from an operator stick. The default command, at the lowest priority.
     *
     * @param left  Left side input, in [-1, 1].
     * @param right Right side input, in [-1, 1].
     */
    public Command manual(DoubleSupplier left, DoubleSupplier right) {
        return run(coroutine -> {
            while (true) {
                leaderController.setDutyCycle(left.getAsDouble() * kManualArmScale);
                followerController.setDutyCycle(right.getAsDouble() * kManualArmScale);
                coroutine.yield();
            }
        }).withPriority(Command.LOWEST_PRIORITY).named("IntakeArm Manual");
    }

    /** Make the current position read 0° on both sides. */
    public Command resetEncoder() {
        return run(coroutine -> {
            leaderController.setEncoderPosition(Degrees.of(0));
            followerController.setEncoderPosition(Degrees.of(0));
        }).named("IntakeArm Reset Encoder");
    }

    /** Called from {@code Robot.robotPeriodic()}, replacing the v2 subsystem periodic. */
    public void periodic() {
        arm.updateTelemetry();
        followerController.updateTelemetry();
    }

    /** Called from {@code Robot.simulationPeriodic()}. */
    public void simulationPeriodic() {
        arm.simIterate();
        followerController.simIterate();
    }
}
