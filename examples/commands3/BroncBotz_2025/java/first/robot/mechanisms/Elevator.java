// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.mechanisms;

import static first.robot.Constants.ElevatorConstants.*;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Ports;
import first.robot.util.ReefTargeting;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.controller.ElevatorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Distance;
import org.wpilib.util.Pair;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ElevatorConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Two NEO elevator carrying the coral and algae arms, as a YAMS {@link yams.commands3.mechanisms.Elevator}.
 * The original ran a ProfiledPIDController and ElevatorFeedforward on the roboRIO with the same gains.
 */
public class Elevator implements Mechanism {
    private final SparkMax leftMotor = new SparkMax(Ports.kCANBus, Ports.kElevatorLeft, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(Ports.kCANBus, Ports.kElevatorRight, MotorType.kBrushless);

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withGearing(new MechanismGearing(kGearing))
        .withDrumRadius(kChainPitch, kSprocketTeeth)
        // Gains are per meter of travel, like the original's.
        .withLinearClosedLoopController(true)
        .withClosedLoopController(kP, 0, kD)
        .withFeedforward(new ElevatorFeedforward(kS, kG, kV))
        .withTrapezoidalProfile(kMaxVelocity, kMaxAcceleration)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(kCurrentLimit)
        .withClosedLoopRampRate(kRampRate)
        // The right motor mirrors the left.
        .withFollowers(Pair.of(rightMotor, true))
        // The original seeded the encoder from a LaserCAN, which has no 2027 library; the elevator must
        // start at the bottom.
        .withStartingPosition(kMinHeight)
        .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH);

    private final SmartMotorController motorController = new SparkWrapper(leftMotor, DCMotor.getNEO(2), motorConfig);

    private final yams.commands3.mechanisms.Elevator elevator = new yams.commands3.mechanisms.Elevator(
        new ElevatorConfig()
            .withHardLimits(kMinHeight, kMaxHeight)
            .withCarriageWeight(kCarriageMass)
            .withTelemetry("Elevator", TelemetryVerbosity.HIGH),
        motorController);

    public Elevator() {
    }

    public boolean isNear(Distance height) {
        return elevator.isNear(height, kTolerance);
    }

    /** Within an inch of the bottom, like the original's atMin. */
    public boolean isAtMin() {
        return elevator.isNear(kMinHeight, Inches.of(1));
    }

    /** Within an inch of the top, like the original's atMax. */
    public boolean isAtMax() {
        return elevator.isNear(kMaxHeight, Inches.of(1));
    }

    /** Move to a height, finishing once there. The closed loop keeps holding it afterwards. */
    public Command moveTo(Distance height) {
        return run(coroutine -> {
            elevator.setMeasurementPositionSetpoint(height);
            coroutine.waitUntil(() -> isNear(height));
        }).named("Elevator to " + height);
    }

    /** Hold a height until canceled, using the YAMS height command. */
    public Command holdAt(Distance height) {
        return elevator.setHeight(height);
    }

    /** Hold a height, read when the command starts, until canceled. */
    private Command holdAt(Supplier<Distance> height, String name) {
        return run(coroutine -> {
            final Distance target = height.get();
            while (true) {
                elevator.setMeasurementPositionSetpoint(target);
                coroutine.yield();
            }
        }).named(name);
    }

    /** Hold the height the elevator is at when this starts, until canceled. */
    public Command holdCurrent() {
        return holdAt(() -> Meters.of(elevator.getHeight().in(Meters)), "Elevator Hold");
    }

    /** Lower slowly onto the bottom stop, finishing once there. */
    public Command lowerToBottom() {
        return run(coroutine -> {
            elevator.setDutyCycleSetpoint(kLowerDutyCycle);
            coroutine.waitUntil(this::isAtMin);
        }).named("Elevator Lower");
    }

    /** Press down on the bottom stop until canceled; used for L1 and the human player station. */
    private Command holdAtBottom() {
        return setDutyCycle(kLowerDutyCycle);
    }

    /** Hold the coral height for a level, read when the command starts, until canceled. */
    public Command holdCoralLevel(Supplier<ReefTargeting.Level> level) {
        final Command bottom = holdAtBottom();
        return run(coroutine -> {
            final ReefTargeting.Level target = level.get();
            if (target == ReefTargeting.Level.L1) {
                coroutine.await(bottom);
                return;
            }
            while (true) {
                elevator.setMeasurementPositionSetpoint(coralHeight(target));
                coroutine.yield();
            }
        }).named("Elevator Hold Coral Level");
    }

    /** Move to the coral height for a level, finishing once there. */
    public Command moveToCoralLevel(ReefTargeting.Level level) {
        return level == ReefTargeting.Level.L1 ? lowerToBottom() : moveTo(coralHeight(level));
    }

    private static Distance coralHeight(ReefTargeting.Level level) {
        return switch (level) {
            case L1 -> kMinHeight;
            case L2 -> Coral.L2;
            case L3 -> Coral.L3;
            case L4 -> Coral.L4;
        };
    }

    public boolean isAtCoralLevel(ReefTargeting.Level level) {
        return level == ReefTargeting.Level.L1 ? isAtMin() : isNear(coralHeight(level));
    }

    /** The height for pulling algae off the reef: L2 and L3 targets use the low and high algae. */
    public static Distance algaeHeight(ReefTargeting.Level level) {
        return level == ReefTargeting.Level.L3 ? Algae.L34 : Algae.L23;
    }

    /** Hold the algae height for a level, read when the command starts, until canceled. */
    public Command holdAlgaeLevel(Supplier<ReefTargeting.Level> level) {
        return holdAt(() -> algaeHeight(level.get()), "Elevator Hold Algae Level");
    }

    /** Run at a duty cycle until canceled. */
    public Command setDutyCycle(double dutyCycle) {
        return run(coroutine -> {
            while (true) {
                elevator.setDutyCycleSetpoint(dutyCycle);
                coroutine.yield();
            }
        }).named("Elevator Duty Cycle " + dutyCycle);
    }

    /** Called from {@code Robot.robotPeriodic()}, replacing the v2 subsystem periodic. */
    public void periodic() {
        elevator.updateTelemetry();
    }

    /** Called from {@code Robot.simulationPeriodic()}. */
    public void simulationPeriodic() {
        elevator.simIterate();
    }
}
