// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Unqualified Quokkas quokkas2025 (MIT, see LICENSE-UQ).

package first.robot.subsystems;

import static first.robot.Constants.DriveConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.util.CANPorts;
import java.util.function.DoubleSupplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.system.DCMotor;
import org.wpilib.util.Pair;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/** Tank drive with two NEOs per side, driven open loop. */
public class DriveTrain extends SubsystemBase {
  private final SparkMax leftFront = new SparkMax(CANPorts.fromBusId(1), kLeftFrontId, MotorType.kBrushless);
  private final SparkMax leftRear = new SparkMax(CANPorts.fromBusId(1), kLeftRearId, MotorType.kBrushless);
  private final SparkMax rightFront = new SparkMax(CANPorts.fromBusId(1), kRightFrontId, MotorType.kBrushless);
  private final SparkMax rightRear = new SparkMax(CANPorts.fromBusId(1), kRightRearId, MotorType.kBrushless);

  private final SmartMotorControllerConfig leftConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withGearing(new MechanismGearing(kGearRatio))
      .withWheelDiameter(kWheelDiameter)
      .withIdleMode(MotorMode.BRAKE)
      .withMotorInverted(true)
      .withTelemetry("LeftDrive", TelemetryVerbosity.HIGH)
      .withFollowers(Pair.of(leftRear, false));

  private final SmartMotorControllerConfig rightConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withGearing(new MechanismGearing(kGearRatio))
      .withWheelDiameter(kWheelDiameter)
      .withIdleMode(MotorMode.BRAKE)
      .withMotorInverted(false)
      .withTelemetry("RightDrive", TelemetryVerbosity.HIGH)
      .withFollowers(Pair.of(rightRear, false));

  private final SmartMotorController left = new SparkWrapper(leftFront, DCMotor.getNEO(2), leftConfig);
  private final SmartMotorController right = new SparkWrapper(rightFront, DCMotor.getNEO(2), rightConfig);

  /** Creates a new DriveTrain. */
  public DriveTrain() {
  }

  public Command driveTank(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
    return run(
        () -> {
          left.setDutyCycle(leftSpeed.getAsDouble());
          right.setDutyCycle(rightSpeed.getAsDouble());
        });
  }

  public Command moveStraight(double velocity) {
    return run(
        () -> {
          left.setDutyCycle(velocity);
          right.setDutyCycle(velocity);
        });
  }

  public Command turn(double velocity) {
    return run(
        () -> {
          left.setDutyCycle(velocity);
          right.setDutyCycle(-velocity);
        });
  }

  @Override
  public void periodic() {
    left.updateTelemetry();
    right.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    left.simIterate();
    right.simIterate();
  }
}
