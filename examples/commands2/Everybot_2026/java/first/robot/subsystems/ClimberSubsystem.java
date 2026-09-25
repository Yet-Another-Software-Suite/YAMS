// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.subsystems;

import static first.robot.Constants.ClimbConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.util.CANPorts;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.system.DCMotor;
import yams.commands2.config.SmartMotorControllerConfig;
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
public class ClimberSubsystem extends SubsystemBase {
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

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
  }

  // A method to set the percentage of the climber
  public void setClimber(double power) {
    climber.setDutyCycle(power);
  }

  // A method to stop the climber
  public void stop() {
    climber.setDutyCycle(0);
  }

  @Override
  public void periodic() {
    climber.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    climber.simIterate();
  }
}
