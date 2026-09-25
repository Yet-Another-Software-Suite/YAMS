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
import yams.core.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.core.motorcontrollers.local.SparkWrapper;

/**
 * Tank drive for the 2026 Everybot: four brushed CIMs on SPARK MAXes, two per side in a
 * leader/follower layout. Driven open loop through {@link DifferentialDrive}.
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
      .withIdleMode(MotorMode.BRAKE)
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
      .withIdleMode(MotorMode.BRAKE)
      .withMotorInverted(false)
      .withTelemetry("RightDrive", TelemetryVerbosity.HIGH)
      .withFollowers(Pair.of(rightFollower, false));

  // DCMotor.getCIM(2) tells the simulator two CIMs drive each side.
  private final SmartMotorController left = new SparkWrapper(leftLeader, DCMotor.getCIM(2), leftConfig);
  private final SmartMotorController right = new SparkWrapper(rightLeader, DCMotor.getCIM(2), rightConfig);

  // set up differential drive class
  private final DifferentialDrive drive = new DifferentialDrive(left::setDutyCycle, right::setDutyCycle);

  /** Creates a new CANDriveMechanism. */
  public CANDriveMechanism() {
  }

  /**
   * Arcade drive from suppliers. Runs until interrupted and stops the drive when canceled. The
   * values are sent every loop, which also feeds the {@link DifferentialDrive} watchdog.
   *
   * @param xSpeed    forward speed supplier, [-1, 1]
   * @param zRotation rotation supplier, [-1, 1]
   * @return a command that drives the robot with arcade controls
   */
  public Command arcadeDrive(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return runRepeatedly(() -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()))
        .whenCanceled(() -> drive.arcadeDrive(0, 0))
        .named("CANDrive.ArcadeDrive");
  }

  /**
   * Arcade drive at fixed values. This command never ends on its own, so use a timeout to control
   * how long it runs.
   *
   * @param xSpeed    forward speed, [-1, 1]
   * @param zRotation rotation, [-1, 1]
   * @return a command that drives the robot at the given values until interrupted
   */
  public Command autoDrive(double xSpeed, double zRotation) {
    return runRepeatedly(() -> drive.arcadeDrive(xSpeed, zRotation))
        .whenCanceled(() -> drive.arcadeDrive(0, 0))
        .named("CANDrive.AutoDrive[" + xSpeed + ", " + zRotation + "]");
  }

  /**
   * Holds the drive stopped. Used as the default command.
   *
   * @return a command that stops the drive until interrupted
   */
  public Command stop() {
    return runRepeatedly(() -> drive.arcadeDrive(0, 0)).named("CANDrive.Stop");
  }

  @Override
  public Command idle() {
    return stop();
  }

  /** Publishes YAMS telemetry. Called from {@code Robot.robotPeriodic()}. */
  public void updateTelemetry() {
    left.updateTelemetry();
    right.updateTelemetry();
  }

  /** Steps the YAMS simulation. Called from {@code Robot.simulationPeriodic()}. */
  public void simIterate() {
    left.simIterate();
    right.simIterate();
  }
}
