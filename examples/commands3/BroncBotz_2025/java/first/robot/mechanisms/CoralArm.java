// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.mechanisms;

import static first.robot.Constants.CoralArmConstants.*;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Millimeters;
import static org.wpilib.units.Units.Rotations;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Ports;
import first.robot.util.DistanceSensor;
import first.robot.util.ReefTargeting;
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
 * Arm that carries the coral intake, as a YAMS {@link Arm}. A through bore encoder on the arm shaft,
 * plugged into the SPARK MAX, gives the absolute angle. The original ran a ProfiledPIDController and
 * ArmFeedforward on the roboRIO.
 */
public class CoralArm implements Mechanism {
    private final SparkMax motor = new SparkMax(Ports.kCANBus, Ports.kCoralArm, MotorType.kBrushless);

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
        .withMotorInverted(kInverted)
        // The absolute encoder seeds the motor encoder (synchronizeAbsoluteEncoder() in the original);
        // the loop closes on the motor encoder, which the motor inversion also inverts.
        .withExternalEncoder(motor.getAbsoluteEncoder())
        .withUseExternalFeedbackEncoder(false)
        .withExternalEncoderZeroOffset(kAbsoluteEncoderOffset)
        .withExternalEncoderDiscontinuityPoint(Rotations.of(0.5))
        .withSimStartingPosition(kStartingAngle)
        .withMomentOfInertia(kLength, kMass)
        .withTelemetry("CoralArmMotor", TelemetryVerbosity.HIGH);

    private final SmartMotorController motorController = new SparkWrapper(motor, DCMotor.getNEO(1), motorConfig);

    private final Arm arm = new Arm(new ArmConfig()
        .withHardLimits(kMinAngle, kMaxAngle)
        .withLength(kLength)
        .withTelemetry("CoralArm", TelemetryVerbosity.HIGH),
        motorController);

    // In simulation the robot starts with a coral loaded, and Robot's simulated game pieces load and
    // unload it (see setSimCoralLoaded); the sensor reads them like a real one.
    private boolean simCoralLoaded = true;
    private final DistanceSensor coralSensor = new DistanceSensor("CoralLaserCan",
        () -> simCoralLoaded, Millimeters.of(70), Millimeters.of(400));

    public CoralArm() {
    }

    public Angle getAngle() {
        return arm.getAngle();
    }

    public boolean isNear(Angle angle) {
        return arm.isNear(angle, kTolerance);
    }

    public boolean isCoralLoaded() {
        return coralSensor.getDistance()
            .map(distance -> distance.gt(kLoadedMin) && distance.lt(kLoadedMax))
            .orElse(false);
    }

    public boolean isCoralScored() {
        return coralSensor.getDistance().map(distance -> distance.gt(kScoredDistance)).orElse(false);
    }

    /** Simulation only: whether the robot is holding a coral. */
    public void setSimCoralLoaded(boolean loaded) {
        if (RobotBase.isSimulation()) {
            simCoralLoaded = loaded;
        }
    }

    /** Re-read the absolute encoder into the motor encoder. */
    public void synchronizeAbsoluteEncoder() {
        motorController.synchronizeRelativeEncoder();
    }

    /**
     * Move to an angle, finishing once there or at a limit. The closed loop keeps holding it
     * afterwards.
     */
    public Command moveTo(Angle angle) {
        return run(coroutine -> {
            arm.setMechanismPositionSetpoint(angle);
            coroutine.waitUntil(() -> isNear(angle) || arm.isAtMax() || arm.isAtMin());
        }).named("CoralArm to " + angle);
    }

    /** Move to an angle, read when the command starts, finishing once there or at a limit. */
    public Command moveTo(Supplier<Angle> angle) {
        return run(coroutine -> {
            final Angle target = angle.get();
            arm.setMechanismPositionSetpoint(target);
            coroutine.waitUntil(() -> isNear(target) || arm.isAtMax() || arm.isAtMin());
        }).named("CoralArm to Target");
    }

    /** Hold an angle until canceled, using the YAMS angle command. */
    public Command holdAt(Angle angle) {
        return arm.setAngle(angle);
    }

    /** Hold an angle, read when the command starts, until canceled. */
    private Command holdAt(Supplier<Angle> angle, String name) {
        return run(coroutine -> {
            final Angle target = angle.get();
            while (true) {
                arm.setMechanismPositionSetpoint(target);
                coroutine.yield();
            }
        }).named(name);
    }

    /** Hold the angle the arm is at when this starts, inside the limits, until canceled. */
    public Command holdCurrent() {
        return holdAt(() -> Degrees.of(Math.clamp(getAngle().in(Degrees),
            kMinAngle.plus(kTolerance).in(Degrees), kMaxAngle.minus(kTolerance).in(Degrees))), "CoralArm Hold");
    }

    /** Swing down onto the branch to place the coral, finishing once it has left the intake. */
    public Command score() {
        return run(coroutine -> {
            final Angle target = getAngle().minus(kScoreDrop);
            while (!isCoralScored()) {
                arm.setMechanismPositionSetpoint(target);
                coroutine.yield();
            }
        }).named("CoralArm Score");
    }

    public static Angle coralAngle(ReefTargeting.Level level) {
        return switch (level) {
            case L1 -> L1;
            case L2 -> L2;
            case L3 -> L3;
            case L4 -> L4;
        };
    }

    /** Run at a duty cycle until canceled. */
    public Command setDutyCycle(double dutyCycle) {
        return run(coroutine -> {
            while (true) {
                arm.setDutyCycleSetpoint(dutyCycle);
                coroutine.yield();
            }
        }).named("CoralArm Duty Cycle " + dutyCycle);
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
