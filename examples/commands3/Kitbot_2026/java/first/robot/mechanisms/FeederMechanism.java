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
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Feeder roller of the 2026 FIRST KitBot fuel mechanism: a brushed motor that moves fuel into the
 * intake/launcher roller, as a YAMS {@link FlyWheel}. Each fuel roller is its own mechanism so it
 * can be tuned live on its own. It exposes one command per roller action, and
 * {@link first.robot.commands.FuelCommands} combines them with the intake/launcher's commands.
 */
public class FeederMechanism implements Mechanism {
  private final SparkMax feederMotor = new SparkMax(CANPorts.fromBusId(1), FEEDER_MOTOR_ID, MotorType.kBrushed);

  // create the configuration for the feeder roller and set a current limit
  private final SmartMotorControllerConfig feederConfig = new SmartMotorControllerConfig(this)
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

  // Runs the feeder at the given voltage and holds it there until the command is interrupted. The
  // idle command stops the roller again once nothing else is using it.
  private Command runAt(Voltage voltage, String name) {
    return run(coroutine -> {
      feederRoller.setVoltageSetpoint(voltage);
      coroutine.park();
    }).named(name);
  }

  // Pulls fuel in from the intake.
  public Command intake() {
    return runAt(INTAKING_FEEDER_VOLTAGE, "Feeder.Intake");
  }

  // Pushes fuel back out the intake. Same voltage as intaking, opposite direction.
  public Command eject() {
    return runAt(INTAKING_FEEDER_VOLTAGE.unaryMinus(), "Feeder.Eject");
  }

  // Holds fuel away from the launcher while it spins up.
  public Command spinUp() {
    return runAt(SPIN_UP_FEEDER_VOLTAGE, "Feeder.SpinUp");
  }

  // Feeds fuel into the launcher.
  public Command launch() {
    return runAt(LAUNCHING_FEEDER_VOLTAGE, "Feeder.Launch");
  }

  // Stops the roller and keeps it stopped until another command claims the feeder. Used as the
  // default command, so it runs whenever no fuel command is using the feeder.
  @Override
  public Command idle() {
    return run(coroutine -> {
      feederRoller.setDutyCycleSetpoint(0);
      coroutine.park();
    }).withPriority(Command.LOWEST_PRIORITY).named("Feeder.Idle");
  }

  /** Publishes YAMS telemetry. Called from {@link first.robot.Robot#robotPeriodic()}. */
  public void periodic() {
    feederRoller.updateTelemetry();
  }

  /** Steps the YAMS simulation. Called from {@link first.robot.Robot#simulationPeriodic()}. */
  public void simulationPeriodic() {
    feederRoller.simIterate();
  }
}
