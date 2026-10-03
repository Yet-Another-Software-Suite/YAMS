// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Constants.HoodConstants;
import first.robot.Ports;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.Pivot;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.PivotConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Shooter hood, modeled as a YAMS {@link Pivot} since gravity barely affects it. It is driven by a
 * lead screw on a 1:1 NEO, so its angles are motor degrees.
 */
public class Hood implements Mechanism {
    private final SparkMax motor = new SparkMax(Ports.kCANPort, Ports.kHood, MotorType.kBrushless);

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(2, 0, 0)
        .withGearing(new MechanismGearing(1))
        .withZeroPower(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40))
        .withVoltageCompensation(Volts.of(12))
        .withMotorInverted(false)
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withSoftLimits(HoodConstants.kMin, HoodConstants.kMax)
        .withStartingPosition(HoodConstants.kDown)
        // 306 lb·in², the original's estimate of the hood's inertia.
        .withMomentOfInertia(KilogramSquareMeters.of(Pounds.of(306.068).in(Kilograms) * Inches.of(1).in(Meters) * Inches.of(1).in(Meters)))
        .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH);

    private final SmartMotorController motorController = new SparkWrapper(motor, DCMotor.getNEO(1), motorConfig);

    private final Pivot hood = new Pivot(new PivotConfig()
        .withHardLimits(HoodConstants.kMin, HoodConstants.kMax)
        .withTelemetry("Hood", TelemetryVerbosity.HIGH),
        motorController);

    public Hood() {
    }

    public Angle getAngle() {
        return hood.getAngle();
    }

    public void setAngleSetpoint(Angle angle) {
        hood.setMechanismPositionSetpoint(angle);
    }

    /** Hold the hood down. The default command, at the lowest priority. */
    public Command holdDown() {
        return run(coroutine -> {
            setAngleSetpoint(HoodConstants.kDown);
            coroutine.park();
        }).withPriority(Command.LOWEST_PRIORITY).named("Hood Down");
    }

    @Override
    public Command idle() {
        return holdDown();
    }

    /** Called from {@code Robot.robotPeriodic()}, replacing the v2 subsystem periodic. */
    public void periodic() {
        hood.updateTelemetry();
    }

    /** Called from {@code Robot.simulationPeriodic()}. */
    public void simulationPeriodic() {
        hood.simIterate();
    }
}
