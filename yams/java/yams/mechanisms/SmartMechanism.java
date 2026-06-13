// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechanisms;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Optional;
import java.util.function.Supplier;
import yams.motorcontrollers.SmartMotorController;
import yams.telemetry.MechanismTelemetry;

/**
 * Generic implementation of a mechanism with advanced telemetry.
 *
 * <p>
 * {@code SmartMechanism} is the abstract root of all YAMS mechanism implementations. It combines a
 * {@link yams.motorcontrollers.SmartMotorController} with integrated telemetry, simulation support,
 * and WPILib {@link edu.wpi.first.wpilibj2.command.Command} / {@link edu.wpi.first.wpilibj2.command.button.Trigger}
 * integration so that every concrete mechanism (Arm, Elevator, Flywheel, etc.) shares a consistent
 * API for control, feedback, and visualization.
 * </p>
 *
 * <h2>Mechanism Lifecycle</h2>
 * <ol>
 *   <li>Configure a motor controller: {@link yams.motorcontrollers.SmartMotorControllerConfig}</li>
 *   <li>Instantiate the appropriate wrapper: {@link yams.motorcontrollers.local.SparkWrapper} (REV), {@link yams.motorcontrollers.remote.TalonFXWrapper} or {@link yams.motorcontrollers.remote.TalonFXSWrapper} (CTRE)</li>
 *   <li>Build a mechanism config (e.g., {@link yams.mechanisms.config.ArmConfig})</li>
 *   <li>Construct the concrete mechanism (e.g., {@link yams.mechanisms.positional.Arm})</li>
 *   <li>Schedule setpoint commands and bind triggers</li>
 * </ol>
 *
 * <p>
 * <b>Periodic calls required:</b> {@link #simIterate()}, {@link #updateTelemetry()}, and
 * {@link #visualizationUpdate()} must be called periodically — typically from
 * {@code robotPeriodic()} — so that simulation state, telemetry, and the {@link edu.wpi.first.wpilibj.smartdashboard.Mechanism2d}
 * visualization remain up to date.
 * </p>
 *
 * <h2>Example</h2>
 * <pre>{@code
 * // 1. Motor config
 * SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig()
 *     .withClosedLoopController(0.2,0,0).withFeedforward(new ArmFeedforawrd(0.05,0,0)).withStatorCurrentLimit(Amps.of(40));
 * // 2. Motor
 * SmartMotorController motor = new SparkWrapper(
 *     new SparkMax(1, MotorType.kBrushless), DCMotor.getNEO(1), motorConfig);
 * // 3. Mechanism config + 4. Mechanism
 * Arm arm = new Arm(new ArmConfig(motor).withLength(Meters.of(0.5)));
 * // 5. Schedule a command (in a subsystem or robot container)
 * arm.setAngle(Degrees.of(45));
 * // 5. Bind a trigger
 * arm.isNear(Degrees.of(45), Degrees.of(2)).onTrue(Commands.print("At target!"));
 * // Call in robotPeriodic():
 * arm.simIterate();
 * arm.updateTelemetry();
 * }</pre>
 */
public abstract class SmartMechanism
{
  /**
   * Subsystem for the Mechanism.
   */
  protected Subsystem            m_subsystem;
  /**
   * Motor for the subsystem.
   */
  protected SmartMotorController m_smc;
  /**
   * Mechanism telemetry.
   */
  protected MechanismTelemetry   m_telemetry = new MechanismTelemetry();
  /**
   * Mechanism Window.
   */
  protected Mechanism2d m_mechanismWindow;

  /**
   * Set the DutyCycle of the {@link SmartMotorController}.
   *
   * @param dutycycle [-1,1] to set.
   * @return {@link Command}
   */
  public Command set(double dutycycle)
  {
    return Commands.startRun(m_smc::stopClosedLoopController, () -> m_smc.setDutyCycle(dutycycle), m_subsystem)
                   .finallyDo(m_smc::startClosedLoopController)
                   .withName(m_subsystem.getName() + " SetDutyCycle");
  }

  /**
   * Set the DutyCycle of the {@link SmartMotorController}.
   *
   * @param dutycycle [-1,1] to set via an {@link Supplier}.
   * @return {@link Command}
   */
  public Command set(Supplier<Double> dutycycle)
  {
    return Commands.startRun(m_smc::stopClosedLoopController,
                             () -> m_smc.setDutyCycle(dutycycle.get()), m_subsystem)
                   .finallyDo(m_smc::startClosedLoopController)
                   .withName(m_subsystem.getName() + " SetDutyCycle Supplier");
  }

  /**
   * Set the voltage of the {@link SmartMotorController}.
   *
   * @param volts {@link Voltage} of the {@link SmartMotorController} to set.
   * @return {@link Command}
   */
  public Command setVoltage(Voltage volts)
  {
    return Commands.startRun(m_smc::stopClosedLoopController, () -> m_smc.setVoltage(volts), m_subsystem)
                   .finallyDo(m_smc::startClosedLoopController)
                   .withName(m_subsystem.getName() + " SetVoltage");
  }

  /**
   * Set the voltage of the {@link SmartMotorController}.
   *
   * @param volts {@link Voltage} of the {@link SmartMotorController} to set, via a
   *              {@link Supplier}.
   * @return {@link Command}
   */
  public Command setVoltage(Supplier<Voltage> volts)
  {
    return Commands.startRun(m_smc::stopClosedLoopController, () -> m_smc.setVoltage(volts.get()), m_subsystem)
                   .finallyDo(m_smc::startClosedLoopController)
                   .withName(m_subsystem.getName() + " SetVoltage Supplier");
  }

  /**
   * Set the {@link SmartMotorController} to the given speed.
   *
   * @param velocity {@link LinearVelocity} to go to.
   */
  public void setMeasurementVelocitySetpoint(LinearVelocity velocity)
  {
    m_smc.startClosedLoopController();
    m_smc.setVelocity(velocity);
  }

  /**
   * Set the {@link SmartMotorController} to the given speed.
   *
   * @param velocity {@link AngularVelocity} to go to.
   */
  public void setMechanismVelocitySetpoint(AngularVelocity velocity)
  {
    m_smc.startClosedLoopController();
    m_smc.setVelocity(velocity);
  }

  /**
   * Sets the {@link SmartMotorController} to the given distance.
   *
   * @param distance {@link Distance} to go to.
   */
  public void setMeasurementPositionSetpoint(Distance distance)
  {
    m_smc.startClosedLoopController();
    m_smc.setPosition(distance);
  }

  /**
   * Sets the {@link SmartMotorController} to the given angle.
   *
   * @param angle {@link Angle} to go to.
   */
  public void setMechanismPositionSetpoint(Angle angle)
  {
    m_smc.startClosedLoopController();
    m_smc.setPosition(angle);
  }

  /**
   * Set the voltage of the {@link SmartMotorController}.
   *
   * @param voltage {@link Voltage} to go to.
   */
  public void setVoltageSetpoint(Voltage voltage)
  {
    m_smc.stopClosedLoopController();
    m_smc.setVoltage(voltage);
  }

  /**
   * Set the dutycycle of the {@link SmartMotorController}.
   *
   * @param dutycycle [-1,1] to set.
   */
  public void setDutyCycleSetpoint(double dutycycle)
  {
    m_smc.stopClosedLoopController();
    m_smc.setDutyCycle(dutycycle);
  }

  /**
   * Get the {@link SmartMotorController}
   *
   * @return {@link SmartMotorController} for the mechanism.
   */
  public SmartMotorController getMotorController()
  {
    return m_smc;
  }

  /**
   * Get the {@link SmartMechanism}'s setpoint as an {@link Angle} if it exists.
   *
   * @return {@link Optional} setpoint {@link Angle} of the mechanism..
   */
  public Optional<Angle> getMechanismSetpoint()
  {
    return m_smc.getMechanismPositionSetpoint();
  }

  /**
   * Iterate sim
   */
  public abstract void simIterate();

  /**
   * Update the mechanism's telemetry.
   */
  public abstract void updateTelemetry();

  /**
   * Get the {@link Mechanism2d} for the mechanism.
   *
   * @return {@link Mechanism2d} for the mechanism.
   */
  public Mechanism2d getMechanismWindow()
  {
    return m_mechanismWindow;
  }

  /**
   * Update the mechanism's visualization state.
   */
  public abstract void visualizationUpdate();

  /**
   * Get the {@link Translation3d} of the mechanism using {@link Mechanism2d} coordinates.
   *
   * @return {@link Translation3d} of the mechanism.
   */
  public abstract Translation3d getRelativeMechanismPosition();

  /**
   * Get the name of the mechanism.
   *
   * @return {@link String} name.
   */
  public abstract String getName();
}
