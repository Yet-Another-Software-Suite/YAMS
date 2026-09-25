// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms;

import static first.robot.Constants.FuelConstants.*;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.util.CANPorts;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Voltage;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.FlyWheel;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.core.motorcontrollers.local.SparkWrapper;

/**
 * Intake/launcher roller of the 2026 FIRST KitBot fuel mechanism: a brushed motor that both intakes
 * and launches fuel, as a YAMS {@link FlyWheel}. Each fuel roller is its own mechanism so it can be
 * tuned live on its own; {@link first.robot.commands.FuelCommands} runs them together.
 */
public class IntakeLauncherMechanism implements Mechanism {
  private final SparkMax intakeLauncherMotor = new SparkMax(CANPorts.fromBusId(1), INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushed);

  // create the configuration for the launcher roller, set a current limit, and set the motor to
  // inverted so that positive values are used for both intaking and launching
  private final SmartMotorControllerConfig launcherConfig = (SmartMotorControllerConfig) new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withGearing(new MechanismGearing(1.0))
      .withMotorInverted(true)
      .withStatorCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT)
      // Simulation only: rough estimate of the launcher wheel's inertia.
      .withMomentOfInertia(Inches.of(2), Pounds.of(0.5))
      .withTelemetry("IntakeLauncherMotor", TelemetryVerbosity.HIGH);

  private final SmartMotorController intakeLauncherMotorController = new SparkWrapper(intakeLauncherMotor, DCMotor.getCIM(1), launcherConfig);

  // Wheel diameter is an estimate; it only affects telemetry and the simulation display.
  private final FlyWheel intakeLauncherRoller = new FlyWheel(new FlyWheelConfig()
      .withDiameter(Inches.of(4))
      .withTelemetry("IntakeLauncherRoller", TelemetryVerbosity.HIGH),
      intakeLauncherMotorController);

  public void setVoltage(Voltage voltage) {
    intakeLauncherRoller.setVoltageSetpoint(voltage);
  }

  public void stop() {
    intakeLauncherRoller.setDutyCycleSetpoint(0);
  }

  // Runs the roller at the given voltage until interrupted. Does not stop the roller when it ends;
  // the fuel commands decide when to stop.
  public Command runAt(Voltage voltage) {
    return runRepeatedly(() -> setVoltage(voltage)).named("IntakeLauncher.RunAt[" + voltage + "]");
  }

  /** Publishes YAMS telemetry. Called from {@link first.robot.Robot#robotPeriodic()}. */
  public void periodic() {
    intakeLauncherRoller.updateTelemetry();
  }

  /** Steps the YAMS simulation. Called from {@link first.robot.Robot#simulationPeriodic()}. */
  public void simulationPeriodic() {
    intakeLauncherRoller.simIterate();
  }
}
