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
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.FlyWheel;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.core.motorcontrollers.local.SparkWrapper;

/**
 * Indexer of the 2026 Everybot: a brushed motor that moves fuel between the intake and the
 * launcher, open loop, as a YAMS {@link FlyWheel}. The intake/launcher is its own subsystem
 * ({@link IntakeLauncherSubsystem}) so each mechanism can be tuned live on its own; the fuel
 * commands require both.
 */
public class IndexerSubsystem extends SubsystemBase {
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

  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {
  }

  // A method to set the power of the indexer roller
  public void setFeederRoller(double power) {
    indexer.setDutyCycleSetpoint(power); // positive for shooting
  }

  // A method to stop the indexer
  public void stop() {
    indexer.setDutyCycleSetpoint(0);
  }

  @Override
  public void periodic() {
    indexer.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    indexer.simIterate();
  }
}
