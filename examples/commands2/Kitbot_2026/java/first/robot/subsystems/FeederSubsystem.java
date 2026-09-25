// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.subsystems;

import static first.robot.Constants.FuelConstants.*;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.util.CANPorts;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Voltage;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.FlyWheel;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.core.motorcontrollers.local.SparkWrapper;

/**
 * Feeder roller of the 2026 FIRST KitBot fuel mechanism: a brushed motor that moves fuel into the
 * intake/launcher roller, as a YAMS {@link FlyWheel}. Each fuel roller is its own subsystem so it
 * can be tuned live on its own; {@link first.robot.commands.FuelCommands} runs them together.
 */
public class FeederSubsystem extends SubsystemBase {
  private final SparkMax feederMotor = new SparkMax(CANPorts.fromBusId(1), FEEDER_MOTOR_ID, MotorType.kBrushed);

  // create the configuration for the feeder roller and set a current limit
  private final SmartMotorControllerConfig feederConfig = (SmartMotorControllerConfig) new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      // Gearing is only used for telemetry and simulation on this open loop roller.
      .withGearing(new MechanismGearing(1.0))
      .withStatorCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT)
      // Simulation only: rough estimate of the feeder roller's inertia.
      .withMomentOfInertia(Inches.of(1), Pounds.of(0.25))
      .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH);

  private final SmartMotorController feederMotorController = new SparkWrapper(feederMotor, DCMotor.getCIM(1), feederConfig);

  // Roller diameter is an estimate; it only affects telemetry and the simulation display.
  private final FlyWheel feederRoller = new FlyWheel(new FlyWheelConfig()
      .withDiameter(Inches.of(2))
      .withTelemetry("FeederRoller", TelemetryVerbosity.HIGH),
      feederMotorController);

  public void setVoltage(Voltage voltage) {
    feederRoller.setVoltageSetpoint(voltage);
  }

  public void stop() {
    feederRoller.setDutyCycleSetpoint(0);
  }

  @Override
  public void periodic() {
    feederRoller.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    feederRoller.simIterate();
  }
}
