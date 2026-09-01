// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.system.plant.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.DifferentialMechanismConfig;
import yams.mechanisms.positional.DifferentialMechanism;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

import static org.wpilib.units.Units.*;

public class DiffyMechSubsystem extends SubsystemBase
{
  private final SparkMax                   leftMotor  = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);
  private final SmartMotorControllerConfig leftConfig = new SmartMotorControllerConfig(this)
          .withClosedLoopController(16, 0, 0)
    .withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
          //.withSoftLimits(Degrees.of(-30), Degrees.of(100))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4, 5)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
          .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
          .withTelemetry("LeftMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
          .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP);
  private final SmartMotorController       leftSMC    = new SparkWrapper(leftMotor,
          DCMotor.getNEO(1),
          leftConfig);
  private final SparkMax                   rightMotor  = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);
  private final SmartMotorControllerConfig rightConfig = new SmartMotorControllerConfig(this)
          .withClosedLoopController(16, 0, 0)
    .withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
          //.withSoftLimits(Degrees.of(-30), Degrees.of(100))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4, 5)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
          .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
          .withTelemetry("RightMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
          .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP);
  private final SmartMotorController       rightSMC    = new SparkWrapper(rightMotor,
                                                                          DCMotor.getNEO(1),
                                                                          rightConfig);
  private final DifferentialMechanismConfig config = new DifferentialMechanismConfig(leftSMC, rightSMC)
          .withStartingPosition(Degrees.of(90), Degrees.of(0))
          .withMOI(Meters.of(0.3), Pounds.of(4))
          .withTelemetry("DiffyMech", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
  private final DifferentialMechanism diffy     = new DifferentialMechanism(config);

  public DiffyMechSubsystem()
  {
  }

  public Command setAngle(Angle tilt, Angle twist) {
    return diffy.setPosition(tilt, twist);
  }

  public Command set(double tilt, double twist) {
    return diffy.set(tilt, twist);
  }

  public void periodic()
  {
    diffy.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    diffy.simIterate();
  }
}
