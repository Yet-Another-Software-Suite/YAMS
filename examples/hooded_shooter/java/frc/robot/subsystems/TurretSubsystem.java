// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFXS;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Seconds;

import java.util.function.Supplier;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import yams.motorcontrollers.SmartMotorControllerConfig;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXSWrapper;

public class TurretSubsystem extends SubsystemBase {
        private final TalonFXS turretMotor = new TalonFXS(1, CANBus.systemcore(1));
        private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        .withClosedLoopController(4, 0, 0)
                        .withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
                        // Configure Motor and Mechanism properties
                        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
                        .withIdleMode(MotorMode.BRAKE)
                        .withMotorInverted(false)
                        // Setup Telemetry
                        .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
                        // Power Optimization
                        .withStatorCurrentLimit(Amps.of(40))
                        .withClosedLoopRampRate(Seconds.of(0.25))
                        .withOpenLoopRampRate(Seconds.of(0.25))
                        .withStartingPosition(Degrees.of(0)) // Starting position of the Pivot
                        .withContinuousWrapping(Degrees.of(0), Degrees.of(360)) // Wrapping enabled bc the pivot can spin infinitely
                        .withMomentOfInertia(Meters.of(0.25), Pounds.of(4)); // MOI Calculation
        private final SmartMotorController turretSMC = new TalonFXSWrapper(turretMotor,
                        DCMotor.getNEO(1),
                        motorConfig);

        private final PivotConfig turretConfig = new PivotConfig()
                        .withHardLimits(Degrees.of(0), Degrees.of(720)) // Hard limit bc wiring prevents infinite
                                                                       // spinning
                        .withTelemetry("TurretMech", TelemetryVerbosity.HIGH); // Telemetry

        private final Pivot turret = new Pivot(turretConfig, turretSMC);

        public TurretSubsystem() {
        }

        public Command setAngle(Angle angle) {
                return turret.setAngle(angle);
        }

        public void setAngleDirect(Angle angle) {
              turretSMC.setPosition(angle);
        }

        public Command setAngle(Supplier<Angle> angleSupplier) {
                return turret.setAngle(angleSupplier);
        }

        public Angle getAngle() {
                return turret.getAngle();
        }

        public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
                return turret.set(dutyCycleSupplier);
        }

        public Command setDutyCycle(double dutyCycle) {
                return turret.set(dutyCycle);
        }

        @Override
        public void periodic() {
                turret.updateTelemetry();
        }

        @Override
        public void simulationPeriodic() {
                turret.simIterate();
        }
}
