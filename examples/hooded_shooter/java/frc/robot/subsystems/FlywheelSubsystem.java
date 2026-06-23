// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class FlywheelSubsystem extends SubsystemBase
{
  private final Distance flywheelDiameter = Inches.of(4);
  private final SparkMax flywheelMotor    = new SparkMax(1, MotorType.kBrushless);

  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(0.00016541, 0, 0)
      .withTrapezoidalProfile(RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
      .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
      .withMomentOfInertia(Inches.of(4), Pounds.of(1))
      .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController motor = new SparkWrapper(flywheelMotor, DCMotor.getNEO(1), motorConfig);

  private final FlyWheelConfig flywheelConfig = new FlyWheelConfig()
      .withDiameter(Inches.of(4))
      .withTelemetry("FlywheelMech", TelemetryVerbosity.HIGH)
      .withSpeedometerSimulation(RPM.of(7500));

  private final FlyWheel flywheel = new FlyWheel(flywheelConfig, motor);

  public FlywheelSubsystem()
  {
  }

  public AngularVelocity getVelocity()
  {
    return flywheel.getSpeed();
  }

  public Command setVelocity(AngularVelocity speed)
  {
    return flywheel.run(speed);
  }

  public Command setDutyCycle(double dutyCycle)
  {
    return flywheel.set(dutyCycle);
  }

  public Command setVelocity(Supplier<AngularVelocity> speed)
  {
    return flywheel.run(speed);
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle)
  {
    return flywheel.set(dutyCycle);
  }

  @Override
  public void periodic()
  {
    flywheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic()
  {
    flywheel.simIterate();
  }

  public Command setRPM(LinearVelocity speed)
  {
    return flywheel.run(RotationsPerSecond.of(speed.in(MetersPerSecond) / flywheelDiameter.times(Math.PI).in(Meters)));
  }

  public void setRPMDirect(LinearVelocity speed)
  {
    motor.setVelocity(RotationsPerSecond.of(speed.in(MetersPerSecond) / flywheelDiameter.times(Math.PI).in(Meters)));
  }
}
