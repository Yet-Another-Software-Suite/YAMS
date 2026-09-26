// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.subsystems;

import static first.robot.Constants.DriveConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.util.CANPorts;
import java.util.function.DoubleSupplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.drive.DifferentialDrive;
import org.wpilib.math.system.DCMotor;
import org.wpilib.util.Pair;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Tank drive for the 2026 FIRST KitBot: four brushed CIMs on SPARK MAXes, two per side in a
 * leader/follower layout. Driven open loop from the joysticks through {@link DifferentialDrive}.
 *
 * <p>
 * YAMS replaces the hand-built SparkMaxConfig: each leader is wrapped in a
 * {@link SmartMotorController} and its follower is handed to the leader's config.
 */
public class CANDriveSubsystem extends SubsystemBase {
  // create brushed motors for drive
  private final SparkMax leftLeader = new SparkMax(CANPorts.fromBusId(1), LEFT_LEADER_ID, MotorType.kBrushed);
  private final SparkMax leftFollower = new SparkMax(CANPorts.fromBusId(1), LEFT_FOLLOWER_ID, MotorType.kBrushed);
  private final SparkMax rightLeader = new SparkMax(CANPorts.fromBusId(1), RIGHT_LEADER_ID, MotorType.kBrushed);
  private final SparkMax rightFollower = new SparkMax(CANPorts.fromBusId(1), RIGHT_FOLLOWER_ID, MotorType.kBrushed);

  // Voltage compensation helps the robot perform more similarly on different battery voltages (at
  // the cost of a little bit of top speed on a fully charged battery). The current limit helps
  // prevent tripping breakers.
  private final SmartMotorControllerConfig leftConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withGearing(new MechanismGearing(DRIVE_GEAR_RATIO))
      .withWheelDiameter(WHEEL_DIAMETER)
      .withVoltageCompensation(NOMINAL_VOLTAGE)
      .withStatorCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT)
      // Set Left side inverted so that positive values drive both sides forward
      .withMotorInverted(true)
      .withTelemetry("LeftDrive", TelemetryVerbosity.HIGH)
      // Follower spins the same direction as its leader
      .withFollowers(Pair.of(leftFollower, false));

  private final SmartMotorControllerConfig rightConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withGearing(new MechanismGearing(DRIVE_GEAR_RATIO))
      .withWheelDiameter(WHEEL_DIAMETER)
      .withVoltageCompensation(NOMINAL_VOLTAGE)
      .withStatorCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT)
      .withMotorInverted(false)
      .withTelemetry("RightDrive", TelemetryVerbosity.HIGH)
      .withFollowers(Pair.of(rightFollower, false));

  // DCMotor.getCIM(2) tells the simulator two CIMs drive each side.
  private final SmartMotorController left = new SparkWrapper(leftLeader, DCMotor.getCIM(2), leftConfig);
  private final SmartMotorController right = new SparkWrapper(rightLeader, DCMotor.getCIM(2), rightConfig);

  // set up differential drive class
  private final DifferentialDrive drive = new DifferentialDrive(left::setDutyCycle, right::setDutyCycle);

  public CANDriveSubsystem() {
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

  // Command factory to create command to drive the robot with joystick inputs.
  public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return this.run(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()));
  }

  // Command factory to create a command that stops the drive motors once and then ends.
  public Command stopCommand() {
    return this.runOnce(() -> drive.arcadeDrive(0, 0));
  }
}
