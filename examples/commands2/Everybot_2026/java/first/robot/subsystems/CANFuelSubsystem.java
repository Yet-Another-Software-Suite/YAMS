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
import org.wpilib.tunable.Tunables;
import org.wpilib.tunable.TunableDouble;
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
 * Fuel mechanism for the 2026 Everybot: two NEOs spin the intake/launcher rollers from either side
 * and a brushed indexer moves fuel between the intake and the launcher. Everything is open loop.
 *
 * <p>
 * Both are YAMS {@link FlyWheel}s: they are velocity mechanisms even though they are driven open
 * loop, and the mechanism adds telemetry and a physics simulation.
 */
public class CANFuelSubsystem extends SubsystemBase {
  // The right launcher motor is wrapped by YAMS; the left one faces the opposite way, so it is
  // configured as an inverted follower instead of being commanded separately.
  private final SparkMax rightIntakeLauncherMotor = new SparkMax(CANPorts.fromBusId(1), RIGHT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax leftIntakeLauncherMotor = new SparkMax(CANPorts.fromBusId(1), LEFT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax indexerMotor = new SparkMax(CANPorts.fromBusId(1), INDEXER_MOTOR_ID, MotorType.kBrushed);

  // create the configuration for the feeder roller and set a current limit
  private final SmartMotorControllerConfig indexerConfig = (SmartMotorControllerConfig) new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      // Gearing is only used for telemetry and simulation on these open loop rollers.
      .withGearing(new MechanismGearing(1.0))
      .withStatorCurrentLimit(INDEXER_MOTOR_CURRENT_LIMIT)
      // Simulation only: rough estimate of the indexer roller's inertia.
      .withMomentOfInertia(Inches.of(1), Pounds.of(0.25))
      .withTelemetry("IndexerMotor", TelemetryVerbosity.HIGH);

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

  private final SmartMotorController indexerMotorController = new SparkWrapper(indexerMotor, DCMotor.getCIM(1), indexerConfig);
  private final SmartMotorController intakeLauncherMotorController = new SparkWrapper(rightIntakeLauncherMotor, DCMotor.getNEO(2), launcherConfig);

  // Diameters are estimates; they only affect telemetry and the simulation display.
  private final FlyWheel indexer = new FlyWheel(new FlyWheelConfig()
      .withDiameter(Inches.of(2))
      .withTelemetry("Indexer", TelemetryVerbosity.HIGH),
      indexerMotorController);
  private final FlyWheel intakeLauncher = new FlyWheel(new FlyWheelConfig()
      .withDiameter(Inches.of(4))
      .withTelemetry("IntakeLauncher", TelemetryVerbosity.HIGH),
      intakeLauncherMotorController);

  // put default values for various fuel operations onto the dashboard
  // all commands using this subsystem pull values from the dashboard to allow
  // you to tune the values easily, and then replace the values in Constants.java
  // with your new values. For more information, see the Software Guide.
  public final TunableDouble intakingFeederValue = Tunables.addDouble("Intaking feeder roller value", INDEXER_INTAKING_PERCENT);
  public final TunableDouble intakingIntakeValue = Tunables.addDouble("Intaking intake roller value", INTAKE_INTAKING_PERCENT);
  public final TunableDouble ejectingIntakeValue = Tunables.addDouble("Ejecting intake roller value", INTAKE_EJECT_PERCENT);
  public final TunableDouble launchingFeederValue = Tunables.addDouble("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT);
  public final TunableDouble launchingLauncherValue = Tunables.addDouble("Launching launcher roller value", LAUNCHING_LAUNCHER_PERCENT);
  public final TunableDouble spinUpFeederValue = Tunables.addDouble("Launching spin-up feeder value", INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT);

  /** Creates a new CANFuelSubsystem. */
  public CANFuelSubsystem() {
  }

  // A method to set the power of the intake/launcher rollers
  public void setIntakeLauncherRoller(double power) {
    intakeLauncher.setDutyCycleSetpoint(power); // positive for shooting
  }

  // A method to set the power of the indexer roller
  public void setFeederRoller(double power) {
    indexer.setDutyCycleSetpoint(power); // positive for shooting
  }

  // A method to stop the rollers
  public void stop() {
    indexer.setDutyCycleSetpoint(0);
    intakeLauncher.setDutyCycleSetpoint(0);
  }

  @Override
  public void periodic() {
    indexer.updateTelemetry();
    intakeLauncher.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    indexer.simIterate();
    intakeLauncher.simIterate();
  }
}
