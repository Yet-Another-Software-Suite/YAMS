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
