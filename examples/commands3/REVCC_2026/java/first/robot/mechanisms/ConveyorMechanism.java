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
import org.wpilib.command3.Command;
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
 * Conveyor that carries fuel from the intake up to the shooter: one Vortex, open loop, as a YAMS
 * {@link FlyWheel}. Split from the intake so each mechanism can be tuned live on its own.
 */
public class ConveyorMechanism implements Mechanism
{
  // Initialize conveyor SPARK. We will use open loop control for this.
  private final SparkFlex conveyorMotor = new SparkFlex(CANPorts.fromBusId(1),
                                                        IntakeSubsystemConstants.kConveyorMotorCanId,
                                                        MotorType.kBrushless);

  private final SmartMotorControllerConfig conveyorConfig = (SmartMotorControllerConfig) new SmartMotorControllerConfig(this)
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
   * Runs the conveyor motor at a power in the range of [-1, 1] until canceled, then stops the
   * motor.
   *
   * @param power Duty cycle to run at.
   * @return Command that runs the conveyor.
   */
  public Command runAtPower(double power)
  {
    return run(coroutine -> {
      conveyor.setDutyCycleSetpoint(power);
      coroutine.park();
    }).whenCanceled(() -> conveyor.setDutyCycleSetpoint(0.0))
      .named("Conveyor.RunAtPower[" + power + "]");
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
