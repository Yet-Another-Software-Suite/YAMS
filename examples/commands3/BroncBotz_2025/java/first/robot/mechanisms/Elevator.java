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
        // Simulation only: REVLib's simulation runs the SPARK's loop every 10 ms rather than every
        // millisecond, and this derivative gain makes it oscillate at that rate.
        .withSimClosedLoopController(kP, 0, 0)
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

    /** Move to a height, finishing once there. */
    public Command moveTo(Distance height) {
        return elevator.runTo(height, kTolerance);
    }

    /** Hold a height until canceled. */
    public Command holdAt(Distance height) {
        return elevator.setHeight(height);
    }

    /** Hold the height the elevator is at when this starts, until canceled. */
    public Command holdCurrent() {
        return Command.noRequirements(coroutine -> coroutine.await(holdAt(Meters.of(elevator.getHeight().in(Meters))))).named("Elevator Hold");
    }

    /**
     * Hold the coral height for a level, read when the command starts, until canceled. L1 presses
     * down on the bottom stop instead, as for the human player station.
     */
    public Command holdCoralLevel(Supplier<ReefTargeting.Level> level) {
        return Command.noRequirements(coroutine -> {
            final ReefTargeting.Level target = level.get();
            coroutine.await(target == ReefTargeting.Level.L1 ? setDutyCycle(kLowerDutyCycle) : holdAt(coralHeight(target)));
        }).named("Elevator Hold Coral Level");
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

    /** Hold the algae height for a level until canceled. */
    public Command holdAlgaeLevel(Supplier<ReefTargeting.Level> level) {
        return elevator.setHeight(() -> algaeHeight(level.get()));
    }

    /** Run at a duty cycle until canceled. */
    public Command setDutyCycle(double dutyCycle) {
        return elevator.set(dutyCycle);
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
