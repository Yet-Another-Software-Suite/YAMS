// Copyright (c) 2025-2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms;

import static org.wpilib.units.Units.Inches;

import com.revrobotics.util.CANPorts;
import com.revrobotics.spark.SparkMax;
import org.wpilib.util.Pair;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Distance;
import org.wpilib.drive.DifferentialDrive;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import java.util.function.DoubleSupplier;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.SmartMotorController;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.core.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.core.motorcontrollers.local.SparkWrapper;

public class DiffDriveMechanism implements Mechanism {
  private MechanismGearing gearing = new MechanismGearing(3, 4);
  private Distance wheelDiameter = Inches.of(4);

  private SparkMax leftMotor  = new SparkMax(CANPorts.fromBusId(1), 21, SparkMax.MotorType.kBrushless);
  private SparkMax rightMotor = new SparkMax(CANPorts.fromBusId(1), 24, SparkMax.MotorType.kBrushless);

  private SparkMax leftFollowerMotor  = new SparkMax(CANPorts.fromBusId(1), 22, SparkMax.MotorType.kBrushless);
  private SparkMax rightFollowerMotor = new SparkMax(CANPorts.fromBusId(1), 23, SparkMax.MotorType.kBrushless);

  private SmartMotorControllerConfig leftMotorConfig =
      (SmartMotorControllerConfig) new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.OPEN_LOOP)
          .withGearing(gearing)
          .withIdleMode(MotorMode.COAST)
          .withMotorInverted(true)
          .withWheelDiameter(wheelDiameter)
          .withTelemetry("LeftMotorMain", TelemetryVerbosity.LOW)
          .withFollowers(Pair.of(leftFollowerMotor, false));

  private SmartMotorControllerConfig rightMotorConfig =
      (SmartMotorControllerConfig) new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.OPEN_LOOP)
          .withGearing(gearing)
          .withIdleMode(MotorMode.COAST)
          .withMotorInverted(false)
          .withWheelDiameter(wheelDiameter)
          .withTelemetry("RightMotorMain", TelemetryVerbosity.LOW)
          .withFollowers(Pair.of(rightFollowerMotor, false));

  private SmartMotorController leftMotorController =
      new SparkWrapper(leftMotor, DCMotor.getNEO(2), leftMotorConfig);
  private SmartMotorController rightMotorController =
      new SparkWrapper(rightMotor, DCMotor.getNEO(2), rightMotorConfig);

  private DifferentialDrive drive =
      new DifferentialDrive(leftMotorController::setDutyCycle, rightMotorController::setDutyCycle);

  public DiffDriveMechanism() {
    setDefaultCommand(stop());
  }

  public Command stop() {
    return runRepeatedly(drive::stopMotor).named("DiffDrive Stop");
  }

  public Command tankDrive(DoubleSupplier left, DoubleSupplier right) {
    return runRepeatedly(() -> drive.tankDrive(left.getAsDouble(), right.getAsDouble()))
        .named("DiffDrive Tank Drive");
  }

  public Command arcadeDrive(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return runRepeatedly(() -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()))
        .named("DiffDrive Arcade Drive");
  }

  public void periodic() {
    leftMotorController.updateTelemetry();
    rightMotorController.updateTelemetry();
  }

  public void simulationPeriodic() {
    leftMotorController.simIterate();
    rightMotorController.simIterate();
  }
}
