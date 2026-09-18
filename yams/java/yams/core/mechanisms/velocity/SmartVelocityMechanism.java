// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.velocity;

import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.smartdashboard.MechanismRoot2d;
import yams.core.mechanisms.SmartMechanism;
import yams.core.motorcontrollers.SmartMotorController;

/**
 * Abstract intermediate base class for all closed-loop velocity mechanisms in YAMS.
 *
 * <p>Sits between {@link yams.core.mechanisms.SmartMechanism} and concrete velocity mechanisms
 * such as {@link yams.core.mechanisms.velocity.FlyWheel}. Provides velocity setpoint management
 * ({@link yams.core.mechanisms.SmartMechanism#setMechanismVelocitySetpoint} and
 * {@link yams.core.mechanisms.SmartMechanism#setMeasurementVelocitySetpoint}) and a 2D
 * visualization model that subclasses populate and update.
 *
 * <p>Command and Trigger factories live on the corresponding {@code yams.commands2.mechanisms}
 * subclass, not here. Velocity mechanisms have no {@code max()}/{@code min()} concept.
 *
 * <h2>For Subclass Authors</h2>
 * <p>Every concrete subclass must implement these abstract methods inherited from
 * {@link yams.core.mechanisms.SmartMechanism}:
 * <ul>
 * <li>{@link yams.core.mechanisms.SmartMechanism#getRelativeMechanismPosition()}</li>
 * <li>{@link yams.core.mechanisms.SmartMechanism#visualizationUpdate()}</li>
 * <li>{@link yams.core.mechanisms.SmartMechanism#getName()}</li>
 * <li>{@link yams.core.mechanisms.SmartMechanism#simIterate()}</li>
 * <li>{@link yams.core.mechanisms.SmartMechanism#updateTelemetry()}</li>
 * </ul>
 */
public abstract class SmartVelocityMechanism extends SmartMechanism {
  /**
   * The root point of the Mechanism.
   */
  protected MechanismRoot2d     mechanismRoot;
  /**
   * The ligament that is being moved.
   */
  protected MechanismLigament2d mechanismLigament;

  /**
   * Get the ligament of the 2D mechanism model. Used to change the position of the mechanism model
   * in the SmartDashboard.
   *
   * @return Ligament of the 2D mechanism model.
   */
  public MechanismLigament2d getMechanismLigament() {
    return mechanismLigament;
  }

  /**
   * The root of the 2D mechanism model.
   *
   * @return Root of the 2D mechanism model.
   */
  public MechanismRoot2d getMechanismRoot() {
    return mechanismRoot;
  }

  /**
   * Get the motor controller which is moving the mechanism.
   *
   * @return Motor controller which is moving the mechanism.
   */
  public SmartMotorController getMotor() {
    return m_smc;
  }
}
