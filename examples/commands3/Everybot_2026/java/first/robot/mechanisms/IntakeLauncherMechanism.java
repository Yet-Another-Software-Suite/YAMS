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
import org.wpilib.util.Pair;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.FlyWheel;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.core.motorcontrollers.local.SparkWrapper;

/**
 * Intake/launcher rollers of the 2026 Everybot: two NEOs spin the rollers from either side, open
 * loop, as a YAMS {@link FlyWheel}. The indexer is its own mechanism ({@link IndexerMechanism}) so
 * each one can be tuned live on its own; the fuel commands in
 * {@link first.robot.commands.FuelCommands} require both.
 */
public class IntakeLauncherMechanism implements Mechanism {
  // The right launcher motor is wrapped by YAMS; the left one faces the opposite way, so it is
  // configured as an inverted follower instead of being commanded separately.
  private final SparkMax rightIntakeLauncherMotor = new SparkMax(CANPorts.fromBusId(1), RIGHT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax leftIntakeLauncherMotor = new SparkMax(CANPorts.fromBusId(1), LEFT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);

  // create the configuration for the launcher rollers, set a current limit, and coast so the
  // rollers spin down freely
  private final SmartMotorControllerConfig launcherConfig = (SmartMotorControllerConfig) new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withGearing(new MechanismGearing(1.0))
      .withStatorCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT)
      .withVoltageCompensation(NOMINAL_VOLTAGE)
      .withIdleMode(MotorMode.COAST)
      .withMotorInverted(false)
      // Simulation only: rough estimate of the launcher wheels' inertia.
      .withMomentOfInertia(Inches.of(2), Pounds.of(1))
      .withTelemetry("IntakeLauncherMotor", TelemetryVerbosity.HIGH)
      // Left side is inverted so that positive values are used for both intaking and launching
      .withFollowers(Pair.of(leftIntakeLauncherMotor, true));

  private final SmartMotorController intakeLauncherMotorController = new SparkWrapper(rightIntakeLauncherMotor, DCMotor.getNEO(2), launcherConfig);

  // Diameter is an estimate; it only affects telemetry and the simulation display.
  private final FlyWheel intakeLauncher = new FlyWheel(new FlyWheelConfig()
      .withDiameter(Inches.of(4))
      .withTelemetry("IntakeLauncher", TelemetryVerbosity.HIGH),
      intakeLauncherMotorController);

  /** Creates a new IntakeLauncherMechanism. */
  public IntakeLauncherMechanism() {
  }

  /**
   * Runs the intake/launcher rollers at a fixed power until interrupted. Positive is for shooting.
   *
   * @param power duty cycle, [-1, 1]
   * @param name  name of the command
   * @return a command that holds the rollers at the given power
   */
  private Command runRoller(double power, String name) {
    return runRepeatedly(() -> intakeLauncher.setDutyCycleSetpoint(power)).named("IntakeLauncher." + name);
  }

  /**
   * Pulls fuel in off the ground.
   *
   * @return a command that runs the rollers at the intaking percentage
   */
  public Command intake() {
    return runRoller(INTAKE_INTAKING_PERCENT, "Intake");
  }

  /**
   * Spins the rollers for launching. Used both while spinning up and while launching.
   *
   * @return a command that runs the rollers at the launching percentage
   */
  public Command launch() {
    return runRoller(LAUNCHING_LAUNCHER_PERCENT, "Launch");
  }

  /**
   * Pushes fuel back out through the intake.
   *
   * @return a command that runs the rollers at the eject percentage
   */
  public Command eject() {
    return runRoller(INTAKE_EJECT_PERCENT, "Eject");
  }

  /**
   * Holds the rollers stopped. Used as the default command.
   *
   * @return a command that stops the rollers until interrupted
   */
  public Command stop() {
    return runRoller(0, "Stop");
  }

  @Override
  public Command idle() {
    return stop();
  }

  /** Publishes YAMS telemetry. Called from {@code Robot.robotPeriodic()}. */
  public void updateTelemetry() {
    intakeLauncher.updateTelemetry();
  }

  /** Steps the YAMS simulation. Called from {@code Robot.simulationPeriodic()}. */
  public void simIterate() {
    intakeLauncher.simIterate();
  }
}
