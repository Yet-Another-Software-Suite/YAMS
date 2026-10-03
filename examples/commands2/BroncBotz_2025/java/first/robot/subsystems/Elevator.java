// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.subsystems;

import static first.robot.Constants.ElevatorConstants.*;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Constants.ElevatorConstants;
import first.robot.Ports;
import first.robot.util.ReefTargeting;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;
import org.wpilib.math.controller.ElevatorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Distance;
import org.wpilib.util.Pair;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ElevatorConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Two NEO elevator carrying the coral and algae arms, as a YAMS {@link yams.commands2.mechanisms.Elevator}.
 * The original ran a ProfiledPIDController and ElevatorFeedforward on the roboRIO with the same gains.
 */
public class Elevator extends SubsystemBase {
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
        .withZeroPower(MotorMode.COAST)
        .withStatorCurrentLimit(kCurrentLimit)
        .withClosedLoopRampRate(kRampRate)
        // The right motor mirrors the left.
        .withFollowers(Pair.of(rightMotor, true))
        // The original seeded the encoder from a LaserCAN, which has no 2027 library; the elevator must
        // start at the bottom.
        .withStartingPosition(kMinHeight)
        .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH);

    private final SmartMotorController motorController = new SparkWrapper(leftMotor, DCMotor.getNEO(2), motorConfig);

    private final yams.commands2.mechanisms.Elevator elevator = new yams.commands2.mechanisms.Elevator(
        new ElevatorConfig()
            .withHardLimits(kMinHeight, kMaxHeight)
            .withCarriageWeight(kCarriageMass)
            .withTelemetry("Elevator", TelemetryVerbosity.HIGH),
        motorController);

    /** Within an inch of the bottom or top, like the original's atMin and atMax. */
    public final Trigger atMin = elevator.near(kMinHeight, Inches.of(1));
    public final Trigger atMax = elevator.near(kMaxHeight, Inches.of(1));

    public Elevator() {
    }

    public boolean isNear(Distance height) {
        return elevator.isNear(height, ElevatorConstants.kTolerance);
    }

    /** Move to a height, ending once there. */
    public Command moveTo(Distance height) {
        return elevator.setHeight(height).until(() -> isNear(height)).withName("Elevator to " + height);
    }

    /** Hold a height until interrupted. */
    public Command holdAt(Distance height) {
        return elevator.setHeight(height);
    }

    /** Hold the height the elevator is at when this starts, until interrupted. */
    public Command holdCurrent() {
        return defer(() -> holdAt(Meters.of(elevator.getHeight().in(Meters))));
    }

    /** Lower slowly onto the bottom stop, ending once there. */
    public Command lowerToBottom() {
        return setDutyCycle(kLowerDutyCycle).until(atMin);
    }

    /** Press down on the bottom stop until interrupted; used for L1 and the human player station. */
    private Command holdAtBottom() {
        return setDutyCycle(kLowerDutyCycle);
    }

    /** Hold the coral height for a level until interrupted. */
    public Command holdCoralLevel(ReefTargeting.Level level) {
        return switch (level) {
            case L1 -> holdAtBottom();
            case L2 -> holdAt(Coral.L2);
            case L3 -> holdAt(Coral.L3);
            case L4 -> holdAt(Coral.L4);
        };
    }

    /** Hold the coral height for the targeted level until interrupted. */
    public Command holdCoralLevel(Supplier<ReefTargeting.Level> level) {
        return defer(() -> holdCoralLevel(level.get()));
    }

    /** Move to the coral height for a level, ending once there. */
    public Command moveToCoralLevel(ReefTargeting.Level level) {
        return switch (level) {
            case L1 -> lowerToBottom();
            case L2 -> moveTo(Coral.L2);
            case L3 -> moveTo(Coral.L3);
            case L4 -> moveTo(Coral.L4);
        };
    }

    public boolean isAtCoralLevel(ReefTargeting.Level level) {
        return switch (level) {
            case L1 -> atMin.getAsBoolean();
            case L2 -> isNear(Coral.L2);
            case L3 -> isNear(Coral.L3);
            case L4 -> isNear(Coral.L4);
        };
    }

    /** The height for pulling algae off the reef: L2 and L3 targets use the low and high algae. */
    public static Distance algaeHeight(ReefTargeting.Level level) {
        return level == ReefTargeting.Level.L3 ? Algae.L34 : Algae.L23;
    }

    /** Hold the algae height for the targeted level until interrupted. */
    public Command holdAlgaeLevel(Supplier<ReefTargeting.Level> level) {
        return defer(() -> holdAt(algaeHeight(level.get())));
    }

    public Command setDutyCycle(double dutyCycle) {
        return run(() -> elevator.setDutyCycleSetpoint(dutyCycle)).withName("Elevator Duty Cycle " + dutyCycle);
    }

    @Override
    public void periodic() {
        elevator.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        elevator.simIterate();
    }
}
