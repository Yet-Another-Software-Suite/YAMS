// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.subsystems;

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
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.Arm;
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
public class CoralArm extends SubsystemBase {
    private final SparkMax motor = new SparkMax(Ports.kCANBus, Ports.kCoralArm, MotorType.kBrushless);

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withGearing(new MechanismGearing(kReduction))
        .withClosedLoopController(kP, 0, kD)
        .withFeedforward(new ArmFeedforward(kS, kG, kV))
        .withTrapezoidalProfile(kMaxVelocity, kMaxAcceleration)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(kCurrentLimit)
        .withOpenLoopRampRate(kRampRate)
        .withMotorInverted(kInverted)
        // The absolute encoder seeds the motor encoder (synchronizeAbsoluteEncoder() in the original).
        .withExternalEncoder(motor.getAbsoluteEncoder())
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

    // In simulation the robot starts with a coral loaded; scoring or spitting it out clears that, and
    // intaking at the human player station loads another (see setSimCoralLoaded).
    private boolean simCoralLoaded = true;
    private final DistanceSensor coralSensor = new DistanceSensor("CoralLaserCan",
        () -> simCoralLoaded, Millimeters.of(70), Millimeters.of(400));

    public final Trigger coralLoaded = new Trigger(this::isCoralLoaded);

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

    /** Move to an angle, ending once there or at a limit. */
    public Command moveTo(Angle angle) {
        return arm.setAngle(angle).until(() -> isNear(angle) || arm.isAtMax() || arm.isAtMin())
            .withName("CoralArm to " + angle);
    }

    /** Hold an angle until interrupted. */
    public Command holdAt(Angle angle) {
        return arm.setAngle(angle);
    }

    /** Hold the angle the arm is at when this starts, inside the limits, until interrupted. */
    public Command holdCurrent() {
        return defer(() -> holdAt(Degrees.of(Math.max(kMinAngle.plus(kTolerance).in(Degrees),
            Math.min(kMaxAngle.minus(kTolerance).in(Degrees), getAngle().in(Degrees))))));
    }

    /** Swing down onto the branch to place the coral, ending once it has left the intake. */
    public Command score() {
        return defer(() -> holdAt(getAngle().minus(kScoreDrop))).until(this::isCoralScored)
            .finallyDo(() -> setSimCoralLoaded(false))
            .withName("CoralArm Score");
    }

    public static Angle coralAngle(ReefTargeting.Level level) {
        return switch (level) {
            case L1 -> L1;
            case L2 -> L2;
            case L3 -> L3;
            case L4 -> L4;
        };
    }

    /** Move to the angle for the targeted level, ending once there. */
    public Command moveToCoralLevel(Supplier<ReefTargeting.Level> level) {
        return defer(() -> moveTo(coralAngle(level.get())));
    }

    public Command setDutyCycle(double dutyCycle) {
        return run(() -> arm.setDutyCycleSetpoint(dutyCycle)).withName("CoralArm Duty Cycle " + dutyCycle);
    }

    @Override
    public void periodic() {
        arm.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        arm.simIterate();
    }
}
