// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechanisms.velocity;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.smartdashboard.MechanismRoot2d;
import org.wpilib.command2.button.Trigger;
import yams.mechanisms.SmartMechanism;
import yams.motorcontrollers.SmartMotorController;

/**
 * Abstract intermediate base class for all closed-loop velocity mechanisms in YAMS.
 *
 * <p>
 * {@code SmartVelocityMechanism} sits between {@link yams.mechanisms.SmartMechanism} and concrete
 * velocity-controlled mechanism implementations such as
 * {@link yams.mechanisms.velocity.FlyWheel}. It provides the shared infrastructure required for
 * closed-loop velocity control:
 * </p>
 * <ul>
 *   <li>Velocity setpoint management via
 *       {@link yams.mechanisms.SmartMechanism#setMechanismVelocitySetpoint} and
 *       {@link yams.mechanisms.SmartMechanism#setMeasurementVelocitySetpoint}</li>
 *   <li>Velocity-based {@link org.wpilib.command2.button.Trigger} factories
 *       ({@code isNear()}, {@code gte()}, {@code lte()}, {@code between()}) — defined by each
 *       concrete subclass</li>
 *   <li>Command factories such as {@code setSpeed()} defined by each concrete subclass, which
 *       internally call the base-class velocity setpoint methods</li>
 *   <li>A 2D visualization model via {@link org.wpilib.smartdashboard.MechanismRoot2d}
 *       and {@link org.wpilib.smartdashboard.MechanismLigament2d} fields that subclasses
 *       populate and update</li>
 * </ul>
 *
 * <p>
 * <b>Important — {@code max()} and {@code min()} are not applicable to velocity mechanisms.</b>
 * Velocity-controlled mechanisms have no meaningful upper or lower soft limit in the same sense as
 * positional mechanisms. Consequently, calling {@link #max()} or {@link #min()} on any concrete
 * subclass of {@code SmartVelocityMechanism} will throw
 * {@link java.lang.UnsupportedOperationException}. Use the following trigger factories instead:
 * </p>
 * <ul>
 *   <li>{@code isNear(speed, tolerance)} — true when actual velocity is within tolerance of target</li>
 *   <li>{@code gte(speed)} — true when actual velocity is greater than or equal to the given speed</li>
 *   <li>{@code lte(speed)} — true when actual velocity is less than or equal to the given speed</li>
 *   <li>{@code between(start, end)} — true when actual velocity falls within the given range</li>
 * </ul>
 *
 * <p>
 * <b>Conceptual subclass pattern:</b> A concrete velocity mechanism extends
 * {@code SmartVelocityMechanism}, provides a typed command factory, and calls into the base-class
 * setpoint methods:
 * </p>
 * <pre>{@code
 * // A concrete mechanism delegates setpoint management to this base class:
 * public Command setSpeed(AngularVelocity target) {
 *     return Commands.startRun(
 *         () -> setMechanismVelocitySetpoint(target),
 *         () -> {},
 *         m_subsystem
 *     ).withName(getName() + " SetSpeed");
 * }
 * }</pre>
 *
 * <h2>For Subclass Authors</h2>
 * <p>
 * Every concrete subclass of {@code SmartVelocityMechanism} must implement the following abstract
 * methods inherited from {@link yams.mechanisms.SmartMechanism}:
 * </p>
 * <ul>
 *   <li>{@link yams.mechanisms.SmartMechanism#getRelativeMechanismPosition()} — returns the
 *       current 3-D position of the mechanism end-point in
 *       {@link org.wpilib.smartdashboard.Mechanism2d} coordinates</li>
 *   <li>{@link yams.mechanisms.SmartMechanism#visualizationUpdate()} — updates
 *       {@code mechanismLigament} to reflect the current rotational state</li>
 *   <li>{@link yams.mechanisms.SmartMechanism#getName()} — returns a human-readable mechanism
 *       name used for telemetry and command names</li>
 *   <li>{@link yams.mechanisms.SmartMechanism#simIterate()} — advances the physics simulation
 *       model and writes back simulated encoder values each robot loop</li>
 *   <li>{@link yams.mechanisms.SmartMechanism#updateTelemetry()} — publishes mechanism state
 *       to {@link org.wpilib.smartdashboard.SmartDashboard} or an equivalent
 *       telemetry sink</li>
 *   <li>{@link #max()} — must throw {@link java.lang.UnsupportedOperationException}; velocity
 *       mechanisms do not support positional limits</li>
 *   <li>{@link #min()} — must throw {@link java.lang.UnsupportedOperationException}; velocity
 *       mechanisms do not support positional limits</li>
 * </ul>
 */
public abstract class SmartVelocityMechanism extends SmartMechanism
{
  /**
   * The root point of the Mechanism.
   */
  protected MechanismRoot2d     mechanismRoot;
  /**
   * The ligament that is being moved.
   */
  protected MechanismLigament2d mechanismLigament;

  /**
   * {@link SmartVelocityMechanism} is at max, defined by the soft limit or hard limit on the
   * {@link SmartVelocityMechanism}.
   *
   * @return Maximum angle for the {@link SmartVelocityMechanism}.
   */
  public abstract Trigger max();

  /**
   * Minimum angle of the {@link SmartVelocityMechanism} given by the soft limit or hard limit of the
   * {@link SmartVelocityMechanism}.
   *
   * @return {@link Trigger} on minimum of the {@link SmartVelocityMechanism}.
   */
  public abstract Trigger min();

  /**
   * Get the ligament of the 2D mechanism model. Used to change the position of the mechanism model in the
   * SmartDashboard.
   *
   * @return Ligament of the 2D mechanism model.
   */
  public MechanismLigament2d getMechanismLigament()
  {
    return mechanismLigament;
  }

  /**
   * The root of the 2D mechanism model.
   *
   * @return Root of the 2D mechanism model.
   */
  public MechanismRoot2d getMechanismRoot()
  {
    return mechanismRoot;
  }

  /**
   * Get the motor controller which is moving the mechanism.
   *
   * @return Motor controller which is moving the mechanism.
   */
  public SmartMotorController getMotor()
  {
    return m_smc;
  }
}
