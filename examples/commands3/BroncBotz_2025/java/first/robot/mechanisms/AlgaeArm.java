// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.mechanisms;

import static first.robot.Constants.AlgaeArmConstants.*;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Millimeters;
import static org.wpilib.units.Units.Rotations;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Ports;
import first.robot.util.DistanceSensor;
import first.robot.util.ReefTargeting;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.system.DCMotor;
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
 * Arm that carries the algae intake, as a YAMS {@link Arm}. A through bore encoder on the arm shaft,
 * plugged into the SPARK MAX, gives the absolute angle. The original ran a ProfiledPIDController and
 * ArmFeedforward on the roboRIO.
 */
public class AlgaeArm implements Mechanism {
    private final SparkMax motor = new SparkMax(Ports.kCANBus, Ports.kAlgaeArm, MotorType.kBrushless);

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withGearing(new MechanismGearing(kReduction))
        .withClosedLoopController(kP, 0, kD)
        // Simulation only: REVLib's simulation runs the SPARK's loop every 10 ms rather than every
        // millisecond, and this derivative gain makes it oscillate at that rate.
        .withSimClosedLoopController(kP, 0, 0)
        .withFeedforward(new ArmFeedforward(kS, kG, kV))
        .withTrapezoidalProfile(kMaxVelocity, kMaxAcceleration)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(kCurrentLimit)
        .withOpenLoopRampRate(kRampRate)
        .withMotorInverted(false)
        // The absolute encoder seeds the motor encoder (synchronizeAbsoluteEncoder() in the original);
        // the loop closes on the motor encoder, which the motor inversion also inverts.
        .withExternalEncoder(motor.getAbsoluteEncoder())
        .withUseExternalFeedbackEncoder(false)
        .withExternalEncoderInverted(true)
        .withExternalEncoderZeroOffset(kAbsoluteEncoderOffset)
        // The SPARK only wraps at 0.5 or 1 rotation, so the encoder reads [-180, 180) degrees. The
        // original read [-115.2, 244.8); the arm only uses -78 to 90 degrees, so this is equivalent.
        .withExternalEncoderDiscontinuityPoint(Rotations.of(0.5))
        .withSimStartingPosition(kStartingAngle)
        .withMomentOfInertia(kLength, kMass)
        .withTelemetry("AlgaeArmMotor", TelemetryVerbosity.HIGH);

    private final SmartMotorController motorController = new SparkWrapper(motor, DCMotor.getNEO(1), motorConfig);

    private final Arm arm = new Arm(new ArmConfig()
        .withHardLimits(kMinAngle, kMaxAngle)
        .withLength(kLength)
        .withTelemetry("AlgaeArm", TelemetryVerbosity.HIGH),
        motorController);

    // In simulation, intaking loads an algae and spitting it out clears it (see setSimAlgaeLoaded).
    private boolean simAlgaeLoaded = false;
    private final DistanceSensor algaeSensor = new DistanceSensor("AlgaeLaserCan",
        () -> simAlgaeLoaded, Millimeters.of(100), Millimeters.of(400));

    /** Whether the coral wrist is at rest; see {@link #guarded}. */
    private final BooleanSupplier wristAtRest;
    /** Where the arm waits for the coral wrist to rest before swinging down to rest, or null. */
    private Angle waitingForWristAt = null;

    /**
     * @param wristAtRest Whether the coral wrist is at rest. The arm only swings down to rest once it
     *                    is, or it would hit the wrist and break.
     */
    public AlgaeArm(BooleanSupplier wristAtRest) {
        this.wristAtRest = wristAtRest;
    }

    public Angle getAngle() {
        return arm.getAngle();
    }

    public boolean isNear(Angle angle) {
        return arm.isNear(angle, kTolerance);
    }

    public boolean isAlgaeLoaded() {
        return algaeSensor.getDistance()
            .map(distance -> distance.gt(kLoadedMin) && distance.lt(kLoadedMax))
            .orElse(false);
    }

    /**
     * Whether the algae has left the intake. Without a reading this is false, so outtaking runs until
     * its timeout; the original returned true, which ended outtaking immediately.
     */
    public boolean isAlgaeScored() {
        return algaeSensor.getDistance().map(distance -> distance.gt(kScoredDistance)).orElse(false);
    }

    /** Simulation only: whether the robot is holding an algae. */
    public void setSimAlgaeLoaded(boolean loaded) {
        if (RobotBase.isSimulation()) {
            simAlgaeLoaded = loaded;
        }
    }

    /** Re-read the absolute encoder into the motor encoder. */
    public void synchronizeAbsoluteEncoder() {
        motorController.synchronizeRelativeEncoder();
    }

    /**
     * The angle to send the arm to for a target. The coral wrist must be at rest before the arm swings
     * down to rest, at or below its stowed angle, or the arm would hit it and break: until it is, the
     * arm waits where it is instead. Every setpoint goes through here.
     */
    private Angle guarded(Angle target) {
        if (target.lte(kStowed) && target.lt(getAngle()) && !wristAtRest.getAsBoolean()) {
            if (waitingForWristAt == null) {
                waitingForWristAt = getAngle();
            }
            return waitingForWristAt;
        }
        waitingForWristAt = null;
        return target;
    }

    /** Move to an angle, finishing once there. */
    public Command moveTo(Angle angle) {
        return run(coroutine -> {
            while (!isNear(angle)) {
                arm.setMechanismPositionSetpoint(guarded(angle));
                coroutine.yield();
            }
        }).named("AlgaeArm to " + angle);
    }

    /** Hold an angle until canceled. */
    public Command holdAt(Angle angle) {
        return arm.setAngle(() -> guarded(angle));
    }

    /** Hold the angle the arm is at when this starts, inside the limits, until canceled. */
    public Command holdCurrent() {
        return Command.noRequirements(coroutine -> coroutine.await(holdAt(Degrees.of(Math.clamp(getAngle().in(Degrees),
            kMinAngle.in(Degrees), kMaxAngle.in(Degrees)))))).named("AlgaeArm Hold");
    }

    /**
     * Keep the arm stowed out of the way, except while it holds an algae: stowed, the arm is too low
     * for the intake to hold one, so it holds the angle it had the algae at instead. The default
     * command.
     */
    public Command stowUnlessHoldingAlgae() {
        return run(coroutine -> {
            Angle heldAt = null;
            while (true) {
                if (isAlgaeLoaded()) {
                    if (heldAt == null) {
                        heldAt = getAngle();
                    }
                    arm.setMechanismPositionSetpoint(guarded(heldAt));
                } else {
                    heldAt = null;
                    arm.setMechanismPositionSetpoint(guarded(kStowed));
                }
                coroutine.yield();
            }
        }).withPriority(Command.LOWEST_PRIORITY).named("AlgaeArm Stow Unless Holding Algae");
    }

    /** Lift the arm to pull the algae off the reef, holding there until canceled. */
    public Command lift() {
        return Command.noRequirements(coroutine -> coroutine.await(holdAt(getAngle().plus(kLoadLift)))).named("AlgaeArm Lift");
    }

    /** The angle for pulling algae off the reef: L2 and L3 targets use the low and high algae. */
    public static Angle algaeAngle(ReefTargeting.Level level) {
        return level == ReefTargeting.Level.L3 ? L34 : L23;
    }

    /** Hold the algae angle for a level until canceled. */
    public Command holdAlgaeLevel(Supplier<ReefTargeting.Level> level) {
        return arm.setAngle(() -> guarded(algaeAngle(level.get())));
    }

    /** Called from {@code Robot.robotPeriodic()}, replacing the v2 subsystem periodic. */
    public void periodic() {
        arm.updateTelemetry();
    }

    /** Called from {@code Robot.simulationPeriodic()}. */
    public void simulationPeriodic() {
        arm.simIterate();
    }
}
