// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;

import com.ctre.phoenix6.hardware.TalonFX;
import first.robot.Constants.ShooterConstants;
import first.robot.Ports;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.util.Pair;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularVelocity;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.FlyWheel;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/** The shooter flywheel: two Kraken X60s, the second following the first inverted. */
public class Shooter implements Mechanism {
    private final TalonFX leader = new TalonFX(Ports.kShooterLeader, Ports.kCTRECANBus);
    private final TalonFX follower = new TalonFX(Ports.kShooterFollower, Ports.kCTRECANBus);

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(0.3447, 0, 0.0025)
        // Kept at 1:1 as at competition; the actual gearing is 22:18.
        .withGearing(new MechanismGearing(1))
        .withZeroPower(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(60))
        .withMotorInverted(true)
        .withFeedforward(new SimpleMotorFeedforward(0.17, 0.117, 0.01))
        .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
        .withFollowers(Pair.of(follower, true))
        // Simulation only: rough estimate of the rollers' inertia.
        .withMomentOfInertia(Inches.of(4), Pounds.of(1))
        .withTelemetry("ShooterMotor", TelemetryVerbosity.LOW);

    private final SmartMotorController motor = new TalonFXWrapper(leader, DCMotor.getKrakenX60(2), motorConfig);

    private final FlyWheel shooter = new FlyWheel(new FlyWheelConfig()
        .withTelemetry("Shooter", TelemetryVerbosity.HIGH),
        motor);

    public Shooter() {
    }

    public AngularVelocity getSpeed() {
        return shooter.getSpeed();
    }

    public boolean isNear(AngularVelocity target) {
        return shooter.isNear(target, ShooterConstants.kReadyTolerance);
    }

    public void setVelocitySetpoint(AngularVelocity velocity) {
        shooter.setMechanismVelocitySetpoint(velocity);
    }

    public void setDutyCycleSetpoint(double dutyCycle) {
        shooter.setDutyCycleSetpoint(dutyCycle);
    }

    /** Coast down and stay stopped. The default command, at the lowest priority. */
    public Command stop() {
        return run(coroutine -> {
            shooter.setDutyCycleSetpoint(0);
            coroutine.park();
        }).withPriority(Command.LOWEST_PRIORITY).named(getName() + " Stop");
    }

    @Override
    public Command idle() {
        return stop();
    }

    /** Called from {@code Robot.robotPeriodic()}, replacing the v2 subsystem periodic. */
    public void periodic() {
        shooter.updateTelemetry();
    }

    /** Called from {@code Robot.simulationPeriodic()}. */
    public void simulationPeriodic() {
        shooter.simIterate();
    }
}
