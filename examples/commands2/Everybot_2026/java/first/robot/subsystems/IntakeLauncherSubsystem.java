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
import org.wpilib.util.Pair;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.FlyWheel;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.core.motorcontrollers.local.SparkWrapper;

/**
 * Intake/launcher rollers of the 2026 Everybot: two NEOs spin the rollers from either side, open
 * loop, as a YAMS {@link FlyWheel}. The indexer is its own subsystem ({@link IndexerSubsystem}) so
 * each mechanism can be tuned live on its own; the fuel commands require both.
 */
public class IntakeLauncherSubsystem extends SubsystemBase {
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

  /** Creates a new IntakeLauncherSubsystem. */
  public IntakeLauncherSubsystem() {
  }

  // A method to set the power of the intake/launcher rollers
  public void setIntakeLauncherRoller(double power) {
    intakeLauncher.setDutyCycleSetpoint(power); // positive for shooting
  }

  // A method to stop the rollers
  public void stop() {
    intakeLauncher.setDutyCycleSetpoint(0);
  }

  @Override
  public void periodic() {
    intakeLauncher.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    intakeLauncher.simIterate();
  }
}
