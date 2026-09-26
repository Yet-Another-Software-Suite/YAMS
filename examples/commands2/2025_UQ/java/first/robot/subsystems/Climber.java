// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Unqualified Quokkas quokkas2025 (MIT, see LICENSE-UQ).

package first.robot.subsystems;

import static first.robot.Constants.ClimberConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.util.CANPorts;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.system.DCMotor;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/** Climber hooks driven by one brushed motor, open loop. */
public class Climber extends SubsystemBase {
  private final SparkMax climberMotor = new SparkMax(CANPorts.fromBusId(1), kMotorId, MotorType.kBrushed);

  private final SmartMotorControllerConfig climberConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      // Gearing is only used for telemetry and simulation on this open loop climber.
      .withGearing(new MechanismGearing(1.0))
      .withIdleMode(MotorMode.BRAKE)
      .withMotorInverted(true)
      .withTelemetry("Climber", TelemetryVerbosity.HIGH);

  // DCMotor.getCIM(1) is an estimate for the brushed motor; it only affects simulation.
  private final SmartMotorController climber = new SparkWrapper(climberMotor, DCMotor.getCIM(1), climberConfig);

  /** Creates a new Climber. */
  public Climber() {
  }

  public Command moveClimber(double velocity) {
    return run(() -> climber.setDutyCycle(velocity));
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
