// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms;

import static first.robot.Constants.ClimbConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.util.CANPorts;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.system.DCMotor;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.core.motorcontrollers.local.SparkWrapper;

/**
 * Climber for the 2026 Everybot: one brushed motor driven open loop up or down while a button is
 * held.
 */
public class ClimberMechanism implements Mechanism {
  // create a brushed motor for the climber
  private final SparkMax climberMotor = new SparkMax(CANPorts.fromBusId(1), CLIMBER_MOTOR_ID, MotorType.kBrushed);

  // create the configuration for the climb motor and set a current limit. BRAKE holds the robot
  // on the tower when the button is released.
  private final SmartMotorControllerConfig climbConfig = (SmartMotorControllerConfig) new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      // Gearing is only used for telemetry and simulation on this open loop climber.
      .withGearing(new MechanismGearing(1.0))
      .withStatorCurrentLimit(CLIMBER_MOTOR_CURRENT_LIMIT)
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("Climber", TelemetryVerbosity.HIGH);

  private final SmartMotorController climber = new SparkWrapper(climberMotor, DCMotor.getCIM(1), climbConfig);

  /** Creates a new ClimberMechanism. */
  public ClimberMechanism() {
  }

  /**
   * Runs the climber at the climbing percentage until interrupted.
   *
   * @return a command that climbs the tower
   */
  public Command climbUp() {
    return run(coroutine -> {
      while (true) {
        climber.setDutyCycle(CLIMBER_MOTOR_UP_PERCENT);
        coroutine.yield();
      }
    }).named("Climber.ClimbUp");
  }

  /**
   * Runs the climber at the unclimbing percentage until interrupted.
   *
   * @return a command that lowers the robot off the tower
   */
  public Command climbDown() {
    return run(coroutine -> {
      while (true) {
        climber.setDutyCycle(CLIMBER_MOTOR_DOWN_PERCENT);
        coroutine.yield();
      }
    }).named("Climber.ClimbDown");
  }

  /**
   * Holds the climber stopped. Used as the default command.
   *
   * @return a command that stops the climber until interrupted
   */
  @Override
  public Command idle() {
    return run(coroutine -> {
      while (true) {
        climber.setDutyCycle(0);
        coroutine.yield();
      }
    }).named("Climber.Stop");
  }

  /** Publishes YAMS telemetry. Called from {@code Robot.robotPeriodic()}. */
  public void updateTelemetry() {
    climber.updateTelemetry();
  }

  /** Steps the YAMS simulation. Called from {@code Robot.simulationPeriodic()}. */
  public void simIterate() {
    climber.simIterate();
  }
}
