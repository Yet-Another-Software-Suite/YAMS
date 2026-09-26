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
import first.robot.Constants.ShooterSubsystemConstants;
import first.robot.Constants.ShooterSubsystemConstants.FeederSetpoints;
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
 * Feeder that pushes fuel into the shooter flywheel: one Vortex, open loop, as a YAMS
 * {@link FlyWheel}. Split from the shooter so each mechanism can be tuned live on its own.
 */
public class FeederMechanism implements Mechanism
{
  // Initialize feeder SPARK. We will use open loop control for this.
  private final SparkFlex feederMotor = new SparkFlex(CANPorts.fromBusId(1),
                                                      ShooterSubsystemConstants.kFeederMotorCanId,
                                                      MotorType.kBrushless);

  private final SmartMotorControllerConfig feederConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withGearing(new MechanismGearing(1.0))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withOpenLoopRampRate(Seconds.of(1.0))
      .withStatorCurrentLimit(Amps.of(60))
      // Simulation only: rough estimate of the feeder rollers' inertia.
      .withMomentOfInertia(Inches.of(1), Pounds.of(0.5))
      .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH);

  private final SmartMotorController feederMotorController = new SparkWrapper(feederMotor, DCMotor.getNeoVortex(1), feederConfig);

  // Roller diameter is an estimate; it only affects telemetry and the simulation display.
  private final FlyWheel feeder = new FlyWheel(new FlyWheelConfig()
                                                   .withDiameter(Inches.of(2))
                                                   .withTelemetry("Feeder", TelemetryVerbosity.HIGH),
                                               feederMotorController);

  /** Creates a new FeederMechanism. Its default command stops the motor. */
  public FeederMechanism()
  {
    setDefaultCommand(idle());
  }

  /** Command to push fuel into the shooter flywheel until canceled. */
  public Command feed()
  {
    return feeder.set(FeederSetpoints.kFeed);
  }

  /**
   * Stops the feeder motor and keeps it stopped. This is the default command, so the motor stops
   * as soon as the command using it ends. It has the lowest priority so any other command can take
   * the feeder.
   */
  @Override
  public Command idle()
  {
    return run(coroutine -> {
      feeder.setDutyCycleSetpoint(0.0);
      coroutine.park();
    }).withPriority(Command.LOWEST_PRIORITY)
      .named("Feeder.Stop");
  }

  /** Publishes the feeder telemetry. */
  public void updateTelemetry()
  {
    feeder.updateTelemetry();
  }

  /** Steps the feeder physics simulation. */
  public void simIterate()
  {
    feeder.simIterate();
  }
}
