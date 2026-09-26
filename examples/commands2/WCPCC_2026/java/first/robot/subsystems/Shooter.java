// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import first.robot.Constants.KrakenX60;
import first.robot.Ports;
import java.util.List;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.FlyWheel;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Three-Kraken shooter flywheel. WCP ran a velocity loop on each Talon. With YAMS each motor still
 * has its own {@link SmartMotorController} and closed loop: the left motor leads a {@link FlyWheel},
 * and the middle and right motors are loosely coupled followers that receive the same velocity
 * setpoints.
 */
public class Shooter extends SubsystemBase {
    private static final AngularVelocity kVelocityTolerance = RPM.of(100);

    // YAMS has no setting for the peak reverse voltage, so it is passed through as the base Talon
    // config. Keeping it at 0 V stops the flywheel from ever being driven backwards.
    private static TalonFXConfiguration vendorConfig() {
        final TalonFXConfiguration config = new TalonFXConfiguration();
        config.Voltage.withPeakReverseVoltage(Volts.of(0));
        return config;
    }

    // Same configuration for all three motors, as in the original; only the inversion differs.
    private SmartMotorControllerConfig motorConfig(String telemetryName, boolean inverted) {
        return new SmartMotorControllerConfig(this)
            .withVendorConfig(vendorConfig())
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withGearing(new MechanismGearing(1.0))
            .withMotorInverted(inverted)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(120))
            .withSupplyCurrentLimit(Amps.of(70))
            .withClosedLoopController(0.5, 2, 0)
            // 12 volts when requesting max RPS
            .withFeedforward(new SimpleMotorFeedforward(0, 12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond)))
            // Simulation only: rough estimate of each motor's share of the flywheel's inertia.
            .withMomentOfInertia(Inches.of(2), Pounds.of(2.0 / 3))
            .withTelemetry(telemetryName, TelemetryVerbosity.HIGH);
    }

    // Middle and right are Clockwise_Positive.
    private final SmartMotorController middleMotor = new TalonFXWrapper(
        new TalonFX(Ports.kShooterMiddle, Ports.kRoboRioCANBus), DCMotor.getKrakenX60(1), motorConfig("ShooterMiddleMotor", true));
    private final SmartMotorController rightMotor = new TalonFXWrapper(
        new TalonFX(Ports.kShooterRight, Ports.kRoboRioCANBus), DCMotor.getKrakenX60(1), motorConfig("ShooterRightMotor", true));

    // Left is CounterClockwise_Positive and leads; its velocity setpoints go to the followers too.
    private final SmartMotorController leftMotor = new TalonFXWrapper(
        new TalonFX(Ports.kShooterLeft, Ports.kRoboRioCANBus), DCMotor.getKrakenX60(1),
        motorConfig("ShooterLeftMotor", false)
            .withLooselyCoupledFollowers(middleMotor, rightMotor));

    private final List<SmartMotorController> motors = List.of(leftMotor, middleMotor, rightMotor);
    private final List<SmartMotorController> followers = List.of(middleMotor, rightMotor);

    // Wheel diameter is an estimate; it only affects telemetry and the simulation display.
    private final FlyWheel shooter = new FlyWheel(new FlyWheelConfig()
        .withDiameter(Inches.of(4))
        .withTelemetry("Shooter", TelemetryVerbosity.HIGH),
        leftMotor);

    private final TunableDouble dashboardTargetRPM = Tunables.addDouble("Shooter/Dashboard RPM", 0.0);

    // True while the flywheel is being held at a velocity rather than driven open loop.
    private boolean isInVelocityMode = false;
    private AngularVelocity targetVelocity = RPM.of(0);

    public Shooter() {
    }

    public void setRPM(double rpm) {
        targetVelocity = RPM.of(rpm);
        isInVelocityMode = true;
        // setPercentOutput() stops the followers' closed loops, and a loosely coupled follower only
        // receives the leader's setpoint, so restart them before handing them a velocity again. The
        // leader's closed loop is restarted by setMechanismVelocitySetpoint().
        followers.forEach(SmartMotorController::startClosedLoopController);
        shooter.setMechanismVelocitySetpoint(targetVelocity);
    }

    public void setPercentOutput(double percentOutput) {
        isInVelocityMode = false;
        final Voltage voltage = Volts.of(percentOutput * 12.0);
        shooter.setVoltageSetpoint(voltage);
        // Loosely coupled followers only receive position and velocity setpoints, so open loop
        // output is sent to them directly.
        for (final SmartMotorController follower : followers) {
            follower.stopClosedLoopController();
            follower.setVoltage(voltage);
        }
    }

    public void stop() {
        setPercentOutput(0.0);
    }

    public Command spinUpCommand(double rpm) {
        return runOnce(() -> setRPM(rpm))
            .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
    }

    public Command dashboardSpinUpCommand() {
        return defer(() -> spinUpCommand(dashboardTargetRPM.get()));
    }

    public boolean isVelocityWithinTolerance() {
        return isInVelocityMode && motors.stream()
            .allMatch(motor -> motor.getMechanismVelocity().isNear(targetVelocity, kVelocityTolerance));
    }

    @Override
    public void periodic() {
        shooter.updateTelemetry();
        followers.forEach(SmartMotorController::updateTelemetry);
    }

    @Override
    public void simulationPeriodic() {
        shooter.simIterate();
        followers.forEach(SmartMotorController::simIterate);
    }
}
