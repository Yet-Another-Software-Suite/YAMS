// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.subsystems;

import static first.robot.Constants.AlgaeIntakeConstants.*;
import static org.wpilib.units.Units.KilogramSquareMeters;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Ports;
import java.util.function.BooleanSupplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.system.DCMotor;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.FlyWheel;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/** Algae intake roller at the end of the algae arm, as an open loop YAMS {@link FlyWheel}. */
public class AlgaeIntake extends SubsystemBase {
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
        return run(() -> roller.setDutyCycleSetpoint(dutyCycle)).withName("AlgaeIntake " + dutyCycle);
    }

    public Command intake() {
        return setDutyCycle(kIntake);
    }

    public Command outtake() {
        return setDutyCycle(kOuttake);
    }

    public Command stop() {
        return setDutyCycle(0);
    }

    /** Gently hold an algae while {@code holding} is true. The default command. */
    public Command hold(BooleanSupplier holding) {
        return run(() -> roller.setDutyCycleSetpoint(holding.getAsBoolean() ? kHold : 0)).withName("AlgaeIntake Hold");
    }

    public double getDutyCycle() {
        return motorController.getDutyCycle();
    }

    @Override
    public void periodic() {
        roller.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        roller.simIterate();
    }
}
