// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms;

import static first.robot.Constants.DriveConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.util.CANPorts;
import java.util.function.DoubleSupplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.drive.DifferentialDrive;
import org.wpilib.math.system.DCMotor;
import org.wpilib.util.Pair;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.core.motorcontrollers.local.SparkWrapper;

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
  private final SmartMotorControllerConfig leftConfig = (SmartMotorControllerConfig) new SmartMotorControllerConfig(this)
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

  private final SmartMotorControllerConfig rightConfig = (SmartMotorControllerConfig) new SmartMotorControllerConfig(this)
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

  // Command factory to create command to drive the robot with joystick inputs. Runs until
  // interrupted by another command.
  public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return runRepeatedly(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()))
        .named("Drive.Arcade");
  }

  // Command factory to stop driving. Commands the motors to stop once and then ends immediately.
  public Command stop() {
    return run(coroutine -> drive.arcadeDrive(0, 0)).named("Drive.Stop");
  }

  // Keeps the motors stopped (and the DifferentialDrive motor safety fed) whenever no other command
  // is driving. Used as the default command outside of teleop.
  @Override
  public Command idle() {
    return runRepeatedly(() -> drive.arcadeDrive(0, 0))
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Drive.Idle");
  }
}
