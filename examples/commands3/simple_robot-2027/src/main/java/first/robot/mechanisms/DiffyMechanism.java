// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms;

import static org.wpilib.units.Units.*;

import com.revrobotics.util.CANPorts;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.DifferentialMechanism;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.DifferentialMechanismConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class DiffyMechanism implements Mechanism
{
  private final SparkMax                   leftMotor  = new SparkMax(CANPorts.fromBusId(1), 1, SparkLowLevel.MotorType.kBrushless);
  private final SmartMotorControllerConfig leftConfig = new SmartMotorControllerConfig(this)
          .withClosedLoopController(16, 0, 0)
          .withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
          // .withSoftLimits(Degrees.of(-30), Degrees.of(100))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4, 5)))
          //      .withExternalEncoder(armMotor.getAbsoluteEncoder())
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry("LeftMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
          .withControlMode(ControlMode.CLOSED_LOOP);
  private final SmartMotorController       leftSMC    = new SparkWrapper(leftMotor,
          DCMotor.getNEO(1),
          leftConfig);
  private final SparkMax                   rightMotor  = new SparkMax(CANPorts.fromBusId(1), 2, SparkLowLevel.MotorType.kBrushless);
  private final SmartMotorControllerConfig rightConfig = new SmartMotorControllerConfig(this)
          .withClosedLoopController(16, 0, 0)
          .withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
          // .withSoftLimits(Degrees.of(-30), Degrees.of(100))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4, 5)))
          //      .withExternalEncoder(armMotor.getAbsoluteEncoder())
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry("RightMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
          .withControlMode(ControlMode.CLOSED_LOOP);
  private final SmartMotorController rightSMC =
      new SparkWrapper(rightMotor, DCMotor.getNEO(1), rightConfig);
  private final DifferentialMechanismConfig config =
      new DifferentialMechanismConfig(leftSMC, rightSMC)
          .withStartingPosition(Degrees.of(90), Degrees.of(0))
          .withMOI(Meters.of(0.3), Pounds.of(4))
          .withTelemetry("DiffyMech", TelemetryVerbosity.HIGH);
  private final DifferentialMechanism diffy = new DifferentialMechanism(config);

  public DiffyMechanism() {}

  public Command setAngle(Angle tilt, Angle twist) {
    return diffy.setPosition(tilt, twist);
  }

  public Command set(double tilt, double twist) {
    return diffy.set(tilt, twist);
  }

  public void periodic() {
    diffy.updateTelemetry();
  }

  public void simulationPeriodic() {
    diffy.simIterate();
  }
}
