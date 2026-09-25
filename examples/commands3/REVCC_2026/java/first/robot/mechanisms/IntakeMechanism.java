// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Seconds;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.util.CANPorts;
import first.robot.Constants.IntakeSubsystemConstants;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.system.DCMotor;
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
 * Fuel intake rollers for the 2026 REV ION Starter Bot: one Vortex, open loop. The rollers are a
 * velocity mechanism, so they are a YAMS {@link FlyWheel}, which adds telemetry and a physics
 * simulation.
 *
 * <p>The REV code drove the intake and conveyor from one subsystem. They are separate mechanisms
 * here so each one can be tuned live on its own; {@link first.robot.commands.FuelCommands} runs
 * them together.
 */
public class IntakeMechanism implements Mechanism
{
  // Initialize intake SPARK. We will use open loop control for this.
  private final SparkFlex intakeMotor = new SparkFlex(CANPorts.fromBusId(1),
                                                      IntakeSubsystemConstants.kIntakeMotorCanId,
                                                      MotorType.kBrushless);

  private final SmartMotorControllerConfig intakeConfig = (SmartMotorControllerConfig) new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      // Direct drive; the ratio only matters for telemetry and simulation here.
      .withGearing(new MechanismGearing(1.0))
      .withMotorInverted(false)
      // COAST lets the rollers spin down freely instead of jerking fuel when released.
      .withIdleMode(MotorMode.COAST)
      // 0.5 s ramp softens the current spike when the rollers start.
      .withOpenLoopRampRate(Seconds.of(0.5))
      .withStatorCurrentLimit(Amps.of(40))
      // Simulation only: rough estimate of the intake rollers' inertia.
      .withMomentOfInertia(Inches.of(1), Pounds.of(0.5))
      .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH);

  private final SmartMotorController intakeMotorController = new SparkWrapper(intakeMotor, DCMotor.getNeoVortex(1), intakeConfig);

  // Roller diameter is an estimate; it only affects telemetry and the simulation display.
  private final FlyWheel intake = new FlyWheel(new FlyWheelConfig()
                                                   .withDiameter(Inches.of(2))
                                                   .withTelemetry("Intake", TelemetryVerbosity.HIGH),
                                               intakeMotorController);

  /** Creates a new IntakeMechanism. */
  public IntakeMechanism()
  {
    System.out.println("---> IntakeMechanism initialized");
  }

  /**
   * Runs the intake motor at a power in the range of [-1, 1].
   *
   * @param power Duty cycle to run at.
   */
  public void setPower(double power)
  {
    intake.setDutyCycleSetpoint(power);
  }

  /** Stops the intake motor. */
  public void stop()
  {
    intake.setDutyCycleSetpoint(0.0);
  }

  /** Replaces the SmartDashboard applied-output entries from the REV code. */
  public void updateTelemetry()
  {
    intake.updateTelemetry();
  }

  /** Steps the intake physics simulation. */
  public void simIterate()
  {
    intake.simIterate();
  }
}
