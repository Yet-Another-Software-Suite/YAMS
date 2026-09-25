// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.subsystems;

import static first.robot.Constants.FuelConstants.*;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.util.CANPorts;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.system.DCMotor;
import org.wpilib.tunable.Tunables;
import org.wpilib.tunable.TunableDouble;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.FlyWheel;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.core.motorcontrollers.local.SparkWrapper;

/**
 * Fuel mechanism for the 2026 FIRST KitBot: one brushed roller that both intakes and launches,
 * and a brushed feeder roller that moves fuel into it. Both are commanded in volts.
 *
 * <p>
 * Both rollers are YAMS {@link FlyWheel}s: they are velocity mechanisms even though they are
 * driven open loop, and the mechanism adds telemetry and a physics simulation.
 */
public class CANFuelSubsystem extends SubsystemBase {
  // create brushed motors for each of the motors on the launcher mechanism
  private final SparkMax intakeLauncherMotor = new SparkMax(CANPorts.fromBusId(1), INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushed);
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

  private final SmartMotorController feederMotorController = new SparkWrapper(feederMotor, DCMotor.getCIM(1), feederConfig);
  private final SmartMotorController intakeLauncherMotorController = new SparkWrapper(intakeLauncherMotor, DCMotor.getCIM(1), launcherConfig);

  // Wheel diameter is an estimate; it only affects telemetry and the simulation display.
  private final FlyWheel intakeLauncherRoller = new FlyWheel(new FlyWheelConfig()
      .withDiameter(Inches.of(4))
      .withTelemetry("IntakeLauncherRoller", TelemetryVerbosity.HIGH),
      intakeLauncherMotorController);

  // Roller diameter is an estimate; it only affects telemetry and the simulation display.
  private final FlyWheel feederRoller = new FlyWheel(new FlyWheelConfig()
      .withDiameter(Inches.of(2))
      .withTelemetry("FeederRoller", TelemetryVerbosity.HIGH),
      feederMotorController);

  // put default values for various fuel operations onto the dashboard
  // all methods in this subsystem pull their values from the dashboard to allow
  // you to tune the values easily, and then replace the values in Constants.java
  // with your new values. For more information, see the Software Guide.
  private final TunableDouble intakingFeederVoltage = Tunables.addDouble("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
  private final TunableDouble intakingIntakeVoltage = Tunables.addDouble("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
  private final TunableDouble launchingFeederVoltage = Tunables.addDouble("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
  private final TunableDouble launchingLauncherVoltage = Tunables.addDouble("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
  private final TunableDouble spinUpFeederVoltage = Tunables.addDouble("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);

  /** Creates a new CANFuelSubsystem. */
  public CANFuelSubsystem() {
  }

  // A method to set the rollers to values for intaking
  public void intake() {
    feederRoller.setVoltageSetpoint(Volts.of(intakingFeederVoltage.get()));
    intakeLauncherRoller.setVoltageSetpoint(Volts.of(intakingIntakeVoltage.get()));
  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void eject() {
    feederRoller.setVoltageSetpoint(Volts.of(-1 * intakingFeederVoltage.get()));
    intakeLauncherRoller.setVoltageSetpoint(Volts.of(-1 * intakingIntakeVoltage.get()));
  }

  // A method to set the rollers to values for launching.
  public void launch() {
    feederRoller.setVoltageSetpoint(Volts.of(launchingFeederVoltage.get()));
    intakeLauncherRoller.setVoltageSetpoint(Volts.of(launchingLauncherVoltage.get()));
  }

  // A method to stop the rollers
  public void stop() {
    feederRoller.setDutyCycleSetpoint(0);
    intakeLauncherRoller.setDutyCycleSetpoint(0);
  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher
  public void spinUp() {
    feederRoller.setVoltageSetpoint(Volts.of(spinUpFeederVoltage.get()));
    intakeLauncherRoller.setVoltageSetpoint(Volts.of(launchingLauncherVoltage.get()));
  }

  // A command factory to turn the spinUp method into a command that requires this
  // subsystem
  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }

  // A command factory to turn the launch method into a command that requires this
  // subsystem
  public Command launchCommand() {
    return this.run(() -> launch());
  }

  @Override
  public void periodic() {
    feederRoller.updateTelemetry();
    intakeLauncherRoller.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    feederRoller.simIterate();
    intakeLauncherRoller.simIterate();
  }
}
