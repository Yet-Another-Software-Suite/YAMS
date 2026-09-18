// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;
import static org.wpilib.units.Units.Seconds;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class HoodSubsystem extends SubsystemBase {
    private final SparkMax hoodMotor = new SparkMax(1, 2, MotorType.kBrushless);

    private final SmartMotorControllerConfig hoodMotorConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(0.00016541, 0, 0)
            .withTrapezoidalProfile(RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            .withIdleMode(MotorMode.COAST)
            .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(40))
            .withMotorInverted(false)
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
            .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withSoftLimits(Degrees.of(5), Degrees.of(100))
            .withStartingPosition(Degrees.of(5));

    private final SmartMotorController hoodSMC = new SparkWrapper(hoodMotor, DCMotor.getNeo550(1), hoodMotorConfig);

    private final ArmConfig hoodConfig = new ArmConfig()
            .withTelemetry("HoodMech", TelemetryVerbosity.HIGH)
            .withLength(Meters.of(0.3)) // Hood arm length for simulation
            .withHardLimits(Degrees.of(0), Degrees.of(120)); // The Hood can be modeled as an arm since it has a
                                                            // gravitational force acted upon based on the angle its in

    private final Arm hood = new Arm(hoodConfig, hoodSMC);

    public HoodSubsystem() {
    }

    public Command setAngle(Angle angle) {
        return hood.setAngle(angle);
    }

  public void setAngleDirect(Angle angle)
  {
    hoodSMC.setPosition(angle);
  }

    public Command setAngle(Supplier<Angle> angleSupplier) {
        return hood.setAngle(angleSupplier);
    }

    public Angle getAngle() {
        return hood.getAngle();
    }

    public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
        return hood.set(dutyCycleSupplier);
    }

    public Command setDutyCycle(double dutyCycle) {
        return hood.set(dutyCycle);
    }

    @Override
    public void periodic() {
        hood.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        hood.simIterate();
    }
}
