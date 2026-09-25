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
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.FlyWheel;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.core.motorcontrollers.local.SparkWrapper;

/**
 * Indexer of the 2026 Everybot: a brushed motor that moves fuel between the intake and the
 * launcher, open loop, as a YAMS {@link FlyWheel}. The intake/launcher is its own mechanism
 * ({@link IntakeLauncherMechanism}) so each one can be tuned live on its own; the fuel commands in
 * {@link first.robot.commands.FuelCommands} require both.
 */
public class IndexerMechanism implements Mechanism {
  private final SparkMax indexerMotor = new SparkMax(CANPorts.fromBusId(1), INDEXER_MOTOR_ID, MotorType.kBrushed);

  // create the configuration for the feeder roller and set a current limit
  private final SmartMotorControllerConfig indexerConfig = (SmartMotorControllerConfig) new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      // Gearing is only used for telemetry and simulation on this open loop roller.
      .withGearing(new MechanismGearing(1.0))
      .withStatorCurrentLimit(INDEXER_MOTOR_CURRENT_LIMIT)
      // Simulation only: rough estimate of the indexer roller's inertia.
      .withMomentOfInertia(Inches.of(1), Pounds.of(0.25))
      .withTelemetry("IndexerMotor", TelemetryVerbosity.HIGH);

  private final SmartMotorController indexerMotorController = new SparkWrapper(indexerMotor, DCMotor.getCIM(1), indexerConfig);

  // Diameter is an estimate; it only affects telemetry and the simulation display.
  private final FlyWheel indexer = new FlyWheel(new FlyWheelConfig()
      .withDiameter(Inches.of(2))
      .withTelemetry("Indexer", TelemetryVerbosity.HIGH),
      indexerMotorController);

  /** Creates a new IndexerMechanism. */
  public IndexerMechanism() {
  }

  /**
   * Sets the indexer roller power. Positive is toward the launcher. The power holds until it is
   * set again.
   *
   * @param power duty cycle, [-1, 1]
   */
  public void setPower(double power) {
    indexer.setDutyCycleSetpoint(power);
  }

  /** Stops the indexer roller. */
  public void stop() {
    setPower(0);
  }

  /**
   * Holds the indexer stopped. Used as the default command.
   *
   * @return a command that stops the indexer until interrupted
   */
  @Override
  public Command idle() {
    return run(coroutine -> {
      while (true) {
        stop();
        coroutine.yield();
      }
    }).named("Indexer.Stop");
  }

  /** Publishes YAMS telemetry. Called from {@code Robot.robotPeriodic()}. */
  public void updateTelemetry() {
    indexer.updateTelemetry();
  }

  /** Steps the YAMS simulation. Called from {@code Robot.simulationPeriodic()}. */
  public void simIterate() {
    indexer.simIterate();
  }
}
