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
 * Feeder that pushes fuel into the shooter flywheel: one Vortex, open loop, as a YAMS
 * {@link FlyWheel}. Split from the shooter so each mechanism can be tuned live on its own.
 */
public class FeederMechanism implements Mechanism
{
  // Initialize feeder SPARK. We will use open loop control for this.
  private final SparkFlex feederMotor = new SparkFlex(CANPorts.fromBusId(1),
                                                      ShooterSubsystemConstants.kFeederMotorCanId,
                                                      MotorType.kBrushless);

  private final SmartMotorControllerConfig feederConfig = (SmartMotorControllerConfig) new SmartMotorControllerConfig(this)
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

  /**
   * Runs the feeder motor at a power in the range of [-1, 1].
   *
   * @param power Duty cycle to run at.
   */
  public void setPower(double power)
  {
    feeder.setDutyCycleSetpoint(power);
  }

  /** Stops the feeder motor. */
  public void stop()
  {
    feeder.setDutyCycleSetpoint(0.0);
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
