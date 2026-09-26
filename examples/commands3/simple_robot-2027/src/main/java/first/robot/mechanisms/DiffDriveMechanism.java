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
import org.wpilib.command3.button.CommandNiDsXboxController;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class DiffDriveMechanism implements Mechanism {
  private MechanismGearing gearing = new MechanismGearing(3, 4);
  private Distance wheelDiameter = Inches.of(4);

  private SparkMax leftMotor  = new SparkMax(CANPorts.fromBusId(1), 21, SparkMax.MotorType.kBrushless);
  private SparkMax rightMotor = new SparkMax(CANPorts.fromBusId(1), 24, SparkMax.MotorType.kBrushless);

  private SparkMax leftFollowerMotor  = new SparkMax(CANPorts.fromBusId(1), 22, SparkMax.MotorType.kBrushless);
  private SparkMax rightFollowerMotor = new SparkMax(CANPorts.fromBusId(1), 23, SparkMax.MotorType.kBrushless);

  private SmartMotorControllerConfig leftMotorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.OPEN_LOOP)
          .withGearing(gearing)
          .withIdleMode(MotorMode.COAST)
          .withMotorInverted(true)
          .withWheelDiameter(wheelDiameter)
          .withTelemetry("LeftMotorMain", TelemetryVerbosity.LOW)
          .withFollowers(Pair.of(leftFollowerMotor, false));

  private SmartMotorControllerConfig rightMotorConfig =
      new SmartMotorControllerConfig(this)
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

  /**
   * Stop the drivetrain, feeding the motor safety watchdog every loop. Lowest priority, so it only runs as the
   * default command while no other drive command is active.
   */
  public Command stop() {
    return run(coroutine -> {
      while (true) {
        drive.stopMotor();
        coroutine.yield();
      }
    }).withPriority(Command.LOWEST_PRIORITY).named("DiffDrive Stop");
  }

  /** Tank drive from the controller: the left stick drives the left side, the right stick the right. */
  public Command tankDrive(CommandNiDsXboxController controller) {
    return run(coroutine -> {
      while (true) {
        drive.tankDrive(-controller.getLeftY(), -controller.getRightY());
        coroutine.yield();
      }
    }).named("DiffDrive Tank Drive");
  }

  /** Arcade drive from the controller: the left stick drives forward, the right stick turns. */
  public Command arcadeDrive(CommandNiDsXboxController controller) {
    return run(coroutine -> {
      while (true) {
        drive.arcadeDrive(-controller.getLeftY(), -controller.getRightX());
        coroutine.yield();
      }
    }).named("DiffDrive Arcade Drive");
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
