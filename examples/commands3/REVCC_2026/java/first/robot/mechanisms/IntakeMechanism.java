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
import first.robot.Constants.IntakeSubsystemConstants.IntakeSetpoints;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.system.DCMotor;
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
 * Fuel intake rollers for the 2026 REV ION Starter Bot: one Vortex, open loop. The rollers are a
 * velocity mechanism, so they are a YAMS {@link FlyWheel}, which adds telemetry and a physics
 * simulation.
 *
 * <p>The REV code drove the intake and conveyor from one subsystem. They are separate mechanisms
 * here so each one can be tuned live on its own; {@link first.robot.commands.FuelCommands} runs
 * their commands together.
 *
 * <p>The mechanism's own commands ({@link #intake()}, {@link #extake()}) run until canceled. The
 * default command, {@link #idle()}, stops the rollers whenever no other command owns the intake.
 */
public class IntakeMechanism implements Mechanism
{
  // Initialize intake SPARK. We will use open loop control for this.
  private final SparkFlex intakeMotor = new SparkFlex(CANPorts.fromBusId(1),
                                                      IntakeSubsystemConstants.kIntakeMotorCanId,
                                                      MotorType.kBrushless);

  private final SmartMotorControllerConfig intakeConfig = new SmartMotorControllerConfig(this)
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
    setDefaultCommand(idle());
    System.out.println("---> IntakeMechanism initialized");
  }

  /** Command to run the intake rollers inward until canceled. */
  public Command intake()
  {
    return intake.set(IntakeSetpoints.kIntake);
  }

  /** Command to run the intake rollers in reverse until canceled. */
  public Command extake()
  {
    return intake.set(IntakeSetpoints.kExtake);
  }

  /**
   * Stops the intake motor and keeps it stopped. This is the default command, so the rollers stop
   * as soon as the command using them ends. It has the lowest priority so any other command can
   * take the intake.
   */
  @Override
  public Command idle()
  {
    return run(coroutine -> {
      intake.setDutyCycleSetpoint(0.0);
      coroutine.park();
    }).withPriority(Command.LOWEST_PRIORITY)
      .named("Intake.Stop");
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
