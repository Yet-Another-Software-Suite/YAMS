// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Seconds;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.util.CANPorts;
import first.robot.Constants.IntakeSubsystemConstants;
import first.robot.Constants.IntakeSubsystemConstants.ConveyorSetpoints;
import first.robot.Constants.IntakeSubsystemConstants.IntakeSetpoints;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.system.DCMotor;
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
 * Fuel intake for the 2026 REV ION Starter Bot. One Vortex spins the intake rollers and a second
 * Vortex runs the conveyor that carries fuel up to the shooter. Both are open loop: the rollers only
 * ever run at a fixed duty cycle, so there is no setpoint to close a loop on.
 *
 * <p>Both rollers are still velocity mechanisms, so each is a YAMS {@link FlyWheel}, which adds
 * telemetry and a physics simulation. The SparkFlexConfig objects from the REV Configs class are
 * replaced by the same settings expressed through {@link SmartMotorControllerConfig}.
 */
public class IntakeSubsystem extends SubsystemBase
{
  // Initialize intake SPARK. We will use open loop control for this.
  private final SparkFlex intakeMotor   = new SparkFlex(CANPorts.fromBusId(1),
                                                        IntakeSubsystemConstants.kIntakeMotorCanId,
                                                        MotorType.kBrushless);
  // Initialize conveyor SPARK. We will use open loop control for this.
  private final SparkFlex conveyorMotor = new SparkFlex(CANPorts.fromBusId(1),
                                                        IntakeSubsystemConstants.kConveyorMotorCanId,
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

  private final SmartMotorController intakeMotorController   = new SparkWrapper(intakeMotor, DCMotor.getNeoVortex(1), intakeConfig);
  private final SmartMotorController conveyorMotorController = new SparkWrapper(conveyorMotor, DCMotor.getNeoVortex(1), conveyorConfig);

  // Roller diameters are estimates; they only affect telemetry and the simulation display.
  private final FlyWheel intake   = new FlyWheel(new FlyWheelConfig()
                                                     .withDiameter(Inches.of(2))
                                                     .withTelemetry("Intake", TelemetryVerbosity.HIGH),
                                                 intakeMotorController);
  private final FlyWheel conveyor = new FlyWheel(new FlyWheelConfig()
                                                     .withDiameter(Inches.of(2))
                                                     .withTelemetry("Conveyor", TelemetryVerbosity.HIGH),
                                                 conveyorMotorController);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem()
  {
    System.out.println("---> IntakeSubsystem initialized");
  }

  /** Set the intake motor power in the range of [-1, 1]. */
  private void setIntakePower(double power)
  {
    intake.setDutyCycleSetpoint(power);
  }

  /** Set the conveyor motor power in the range of [-1, 1]. */
  private void setConveyorPower(double power)
  {
    conveyor.setDutyCycleSetpoint(power);
  }

  /**
   * Command to run the intake and conveyor motors. When the command is interrupted, e.g. the button
   * is released, the motors will stop.
   */
  public Command runIntakeCommand()
  {
    return this.startEnd(
        () -> {
          this.setIntakePower(IntakeSetpoints.kIntake);
          this.setConveyorPower(ConveyorSetpoints.kIntake);
        }, () -> {
          this.setIntakePower(0.0);
          this.setConveyorPower(0.0);
        }).withName("Intaking");
  }

  /**
   * Command to reverse the intake motor and conveyor motors. When the command is interrupted, e.g.
   * the button is released, the motors will stop.
   */
  public Command runExtakeCommand()
  {
    return this.startEnd(
        () -> {
          this.setIntakePower(IntakeSetpoints.kExtake);
          this.setConveyorPower(ConveyorSetpoints.kExtake);
        }, () -> {
          this.setIntakePower(0.0);
          this.setConveyorPower(0.0);
        }).withName("Extaking");
  }

  @Override
  public void periodic()
  {
    // Replaces the SmartDashboard applied-output entries from the REV code.
    intake.updateTelemetry();
    conveyor.updateTelemetry();
  }

  @Override
  public void simulationPeriodic()
  {
    intake.simIterate();
    conveyor.simIterate();
  }
}
