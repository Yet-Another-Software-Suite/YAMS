// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.mechanisms;

import static first.robot.Constants.AlgaeIntakeConstants.*;
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

/** Algae intake roller at the end of the algae arm, as an open loop YAMS {@link FlyWheel}. */
public class AlgaeIntake implements Mechanism {
    private final SparkMax motor = new SparkMax(Ports.kCANBus, Ports.kAlgaeRoller, MotorType.kBrushless);

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.OPEN_LOOP)
        .withGearing(new MechanismGearing(1))
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(kCurrentLimit)
        .withMotorInverted(true)
        // Simulation only: the original's roller inertia.
        .withMomentOfInertia(KilogramSquareMeters.of(0.00032))
        .withTelemetry("AlgaeRollerMotor", TelemetryVerbosity.HIGH);

    private final SmartMotorController motorController = new SparkWrapper(motor, DCMotor.getNEO(1), motorConfig);

    private final FlyWheel roller = new FlyWheel(new FlyWheelConfig()
        .withTelemetry("AlgaeRoller", TelemetryVerbosity.HIGH),
        motorController);

    public AlgaeIntake() {
    }

    public Command setDutyCycle(double dutyCycle) {
        return run(coroutine -> {
            while (true) {
                roller.setDutyCycleSetpoint(dutyCycle);
                coroutine.yield();
            }
        }).named("AlgaeIntake " + dutyCycle);
    }

    public Command intake() {
        return setDutyCycle(kIntake);
    }

    public Command outtake() {
        return setDutyCycle(kOuttake);
    }

    /** Spit the algae out, finishing once {@code done} is true. */
    public Command outtakeUntil(BooleanSupplier done) {
        return run(coroutine -> {
            while (!done.getAsBoolean()) {
                roller.setDutyCycleSetpoint(kOuttake);
                coroutine.yield();
            }
        }).named("AlgaeIntake Outtake Until Done");
    }

    public Command stop() {
        return setDutyCycle(0);
    }

    /** Gently hold an algae while {@code holding} is true. The default command. */
    public Command hold(BooleanSupplier holding) {
        return run(coroutine -> {
            while (true) {
                roller.setDutyCycleSetpoint(holding.getAsBoolean() ? kHold : 0);
                coroutine.yield();
            }
        }).withPriority(Command.LOWEST_PRIORITY).named("AlgaeIntake Hold");
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
