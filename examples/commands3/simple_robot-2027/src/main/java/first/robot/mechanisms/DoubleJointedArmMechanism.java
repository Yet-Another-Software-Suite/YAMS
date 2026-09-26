// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms;

import com.revrobotics.util.CANPorts;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.DoubleJointedArm;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.local.SparkWrapper;

import static org.wpilib.units.Units.*;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class DoubleJointedArmMechanism implements Mechanism
{
  private final SparkMax                   lowerMotor  = new SparkMax(CANPorts.fromBusId(1), 1, SparkLowLevel.MotorType.kBrushless);
  private final SmartMotorControllerConfig lowerConfig = new SmartMotorControllerConfig(this)
          .withClosedLoopController(16, 0, 0)
    .withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
          //.withSoftLimits(Degrees.of(-30), Degrees.of(100))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4, 5)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry("LowerMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withStartingPosition(Degrees.of(45))
          .withMomentOfInertia(Feet.of(2), Pounds.of(5));
  private final SmartMotorController       lowerSMC    = new SparkWrapper(lowerMotor,
          DCMotor.getNEO(1),
          lowerConfig);
  private final ArmConfig        lowerArmConfig = new ArmConfig()
          .withLength(Feet.of(2))
          .withHardLimits(Degrees.of(-720), Degrees.of(720))
          .withTelemetry("LowerArm", TelemetryVerbosity.HIGH);
  private final SparkMax                   upperMotor  = new SparkMax(CANPorts.fromBusId(1), 2, SparkLowLevel.MotorType.kBrushless);
  private final SmartMotorControllerConfig upperConfig = new SmartMotorControllerConfig(this)
          .withClosedLoopController(16, 0, 0)
    .withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
          //.withSoftLimits(Degrees.of(-30), Degrees.of(100))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4, 5)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry("UpperMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withStartingPosition(Degrees.of(45)).withMomentOfInertia(Feet.of(2.5), Pounds.of(2));
  private final SmartMotorController       upperSMC    = new SparkWrapper(upperMotor,
                                                                          DCMotor.getNEO(1),
                                                                          upperConfig);
  private final ArmConfig        upperArmConfig = new ArmConfig()
      .withLength(Feet.of(2.5))
      .withHardLimits(Degrees.of(-720), Degrees.of(720))
      .withTelemetry("UpperArm", TelemetryVerbosity.HIGH)
      .withSimColor(new Color8Bit(Color.DARK_RED));
  private final DoubleJointedArm jointedArm     = new DoubleJointedArm(lowerArmConfig, lowerSMC, upperArmConfig, upperSMC);

  public DoubleJointedArmMechanism()
  {
  }

  public Command setPosition(Distance x, Distance y, boolean elbowRequest)
  {
    return jointedArm.setPosition(new Translation2d(x.in(Meters), y.in(Meters)), elbowRequest);
  }


  public Command setAngle(Angle lowerAngle, Angle upperAngle) {
    return jointedArm.setAngle(lowerAngle, upperAngle);
  }

  public Command set(Double lowerDutycycle, Double upperDutycycle) {
    return jointedArm.set(lowerDutycycle, upperDutycycle);
  }

  public void periodic()
  {
    jointedArm.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    jointedArm.simIterate();
  }

}
