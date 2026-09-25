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
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

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
  private final SmartMotorControllerConfig launcherConfig = new SmartMotorControllerConfig(this)
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
   * Sets the intake/launcher roller power. Positive is for shooting. The power holds until it is
   * set again.
   *
   * @param power duty cycle, [-1, 1]
   */
  public void setPower(double power) {
    intakeLauncher.setDutyCycleSetpoint(power);
  }

  /** Stops the intake/launcher rollers. */
  public void stop() {
    setPower(0);
  }

  /**
   * Holds the rollers stopped. Used as the default command.
   *
   * @return a command that stops the rollers until interrupted
   */
  @Override
  public Command idle() {
    return run(coroutine -> {
      while (true) {
        stop();
        coroutine.yield();
      }
    }).named("IntakeLauncher.Stop");
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
