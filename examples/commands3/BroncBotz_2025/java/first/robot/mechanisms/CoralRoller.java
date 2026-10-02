// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.mechanisms;

import static first.robot.Constants.CoralIntakeConstants.*;
import static org.wpilib.units.Units.KilogramSquareMeters;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Ports;
import java.util.function.BooleanSupplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.system.DCMotor;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.FlyWheel;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/** Roller of the coral intake at the end of the coral arm, as an open loop YAMS {@link FlyWheel}. */
public class CoralRoller implements Mechanism {
    private final SparkMax motor = new SparkMax(Ports.kCANBus, Ports.kCoralRoller, MotorType.kBrushless);

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.OPEN_LOOP)
        .withGearing(new MechanismGearing(1))
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(kRollerCurrentLimit)
        .withMotorInverted(true)
        .withMomentOfInertia(KilogramSquareMeters.of(kWristMomentOfInertia))
        .withTelemetry("CoralRollerMotor", TelemetryVerbosity.HIGH);

    private final SmartMotorController motorController = new SparkWrapper(motor, DCMotor.getNEO(1), motorConfig);

    private final FlyWheel roller = new FlyWheel(new FlyWheelConfig()
        .withTelemetry("CoralRoller", TelemetryVerbosity.HIGH),
        motorController);

    public CoralRoller() {
    }

    /** Intake from the human player station, until canceled. */
    public Command intake() {
        return roller.set(kIntake);
    }

    /** Hold the coral in while scoring, until canceled. */
    public Command score() {
        return roller.set(kScore);
    }

    /** Push the coral out, until canceled. */
    public Command outtake() {
        return roller.set(kOuttake);
    }

    /** Spit the coral out, until canceled. */
    public Command spit() {
        return roller.set(kSpit);
    }

    /** Run at full speed, until canceled. */
    public Command full() {
        return roller.set(kFull);
    }

    /** Gently hold a coral while one is loaded. The default command. */
    public Command hold(BooleanSupplier coralLoaded) {
        return roller.set(() -> coralLoaded.getAsBoolean() ? kHold : 0.0);
    }

    public double getDutyCycle() {
        return motorController.getDutyCycle();
    }

    /** Called from {@code Robot.robotPeriodic()}, replacing the v2 subsystem periodic. */
    public void periodic() {
        roller.updateTelemetry();
    }

    /** Called from {@code Robot.simulationPeriodic()}. */
    public void simulationPeriodic() {
        roller.simIterate();
    }
}
