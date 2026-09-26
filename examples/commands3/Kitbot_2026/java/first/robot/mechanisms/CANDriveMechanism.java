// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms;

import static first.robot.Constants.DriveConstants.*;
import static first.robot.Constants.OperatorConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.util.CANPorts;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.drive.DifferentialDrive;
import org.wpilib.math.system.DCMotor;
import org.wpilib.util.Pair;
import yams.commands3.config.SmartMotorControllerConfig;
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
public class CANDriveMechanism implements Mechanism {
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

  public CANDriveMechanism() {
  }

  /** Publishes YAMS telemetry. Called from {@link first.robot.Robot#robotPeriodic()}. */
  public void periodic() {
    left.updateTelemetry();
    right.updateTelemetry();
  }

  /** Steps the YAMS simulation. Called from {@link first.robot.Robot#simulationPeriodic()}. */
  public void simulationPeriodic() {
    left.simIterate();
    right.simIterate();
  }

  // Drives the robot open loop. DifferentialDrive motor safety expects this to be called every loop
  // while driving.
  public void arcadeDrive(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  // Command factory to create command to drive the robot with the driver controller's joysticks.
  // Runs until interrupted by another command.
  public Command driveArcade(CommandNiDsXboxController controller) {
    return run(coroutine -> {
      while (true) {
        // The Y axis of the controller is inverted so that pushing the stick away from you (a
        // negative value) drives the robot forwards (a positive value). The X-axis is also inverted
        // so a positive value (stick to the right) results in clockwise rotation (front of the
        // robot turning right). Both axes are also scaled down so the rotation is more easily
        // controllable.
        arcadeDrive(-controller.getLeftY() * DRIVE_SCALING, -controller.getRightX() * ROTATION_SCALING);
        coroutine.yield();
      }
    }).named("Drive.Arcade");
  }

  // Command factory to drive at a fixed speed and rotation, such as during autonomous. Runs until
  // interrupted, sending the output every loop to keep motor safety fed.
  public Command driveArcade(double xSpeed, double zRotation) {
    return run(coroutine -> {
      while (true) {
        arcadeDrive(xSpeed, zRotation);
        coroutine.yield();
      }
    }).named("Drive.ArcadeFixed");
  }

  // Keeps the motors stopped (and the DifferentialDrive motor safety fed) whenever no other command
  // is driving. Used as the default command outside of teleop.
  @Override
  public Command idle() {
    return run(coroutine -> {
      while (true) {
        arcadeDrive(0, 0);
        coroutine.yield();
      }
    }).withPriority(Command.LOWEST_PRIORITY).named("Drive.Idle");
  }
}
