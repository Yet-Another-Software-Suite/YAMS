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
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Conveyor that carries fuel from the intake up to the shooter: one Vortex, open loop, as a YAMS
 * {@link FlyWheel}. Split from the intake so each mechanism can be tuned live on its own.
 */
public class ConveyorMechanism implements Mechanism
{
  // Initialize conveyor SPARK. We will use open loop control for this.
  private final SparkFlex conveyorMotor = new SparkFlex(CANPorts.fromBusId(1),
                                                        IntakeSubsystemConstants.kConveyorMotorCanId,
                                                        MotorType.kBrushless);

  private final SmartMotorControllerConfig conveyorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withGearing(new MechanismGearing(1.0))
      // Mounted opposite the intake motor, so it is inverted to make positive mean "toward the shooter".
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withOpenLoopRampRate(Seconds.of(0.5))
      .withStatorCurrentLimit(Amps.of(40))
      // Simulation only: rough estimate of the conveyor rollers' inertia.
      .withMomentOfInertia(Inches.of(1), Pounds.of(0.5))
      .withTelemetry("ConveyorMotor", TelemetryVerbosity.HIGH);

  private final SmartMotorController conveyorMotorController = new SparkWrapper(conveyorMotor, DCMotor.getNeoVortex(1), conveyorConfig);

  // Roller diameter is an estimate; it only affects telemetry and the simulation display.
  private final FlyWheel conveyor = new FlyWheel(new FlyWheelConfig()
                                                     .withDiameter(Inches.of(2))
                                                     .withTelemetry("Conveyor", TelemetryVerbosity.HIGH),
                                                 conveyorMotorController);

  /**
   * Runs the conveyor motor at a power in the range of [-1, 1].
   *
   * @param power Duty cycle to run at.
   */
  public void setPower(double power)
  {
    conveyor.setDutyCycleSetpoint(power);
  }

  /** Stops the conveyor motor. */
  public void stop()
  {
    conveyor.setDutyCycleSetpoint(0.0);
  }

  /** Publishes the conveyor telemetry. */
  public void updateTelemetry()
  {
    conveyor.updateTelemetry();
  }

  /** Steps the conveyor physics simulation. */
  public void simIterate()
  {
    conveyor.simIterate();
  }
}
