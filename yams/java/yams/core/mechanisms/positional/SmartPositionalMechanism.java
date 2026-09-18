// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.positional;

import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.smartdashboard.MechanismRoot2d;
import yams.core.mechanisms.SmartMechanism;
import yams.core.motorcontrollers.SmartMotorController;

/**
 * Abstract intermediate base class for all closed-loop positional mechanisms in YAMS.
 *
 * <p>
 * {@code SmartPositionalMechanism} sits between {@link yams.core.mechanisms.SmartMechanism} and the
 * concrete positional mechanism implementations such as {@link
 * yams.core.mechanisms.positional.Arm},
 * {@link yams.core.mechanisms.positional.Elevator}, {@link yams.core.mechanisms.positional.Pivot},
 * and
 * {@link yams.core.mechanisms.positional.DifferentialMechanism}. It provides the shared
 * infrastructure required for closed-loop position control:
 * </p>
 * <ul>
 * <li>Setpoint management via {@link
 * yams.core.mechanisms.SmartMechanism#setMechanismPositionSetpoint} and {@link
 * yams.core.mechanisms.SmartMechanism#setMeasurementPositionSetpoint}</li> <li>Position-based
 * {@link org.wpilib.command2.button.Trigger} factories
 * ({@code near()}, {@code gte()}, {@code lte()}, {@code between()}, {@code max()},
 * {@code min()}) — defined by each concrete subclass</li>
 * <li>Command factories such as {@code setAngle()} and {@code setHeight()} defined by each
 * concrete subclass, which internally call
 * {@link yams.core.mechanisms.SmartMechanism#setMechanismPositionSetpoint} or
 * {@link yams.core.mechanisms.SmartMechanism#setMeasurementPositionSetpoint}</li>
 * <li>A 2D visualization model via {@link org.wpilib.smartdashboard.MechanismRoot2d}
 * and {@link org.wpilib.smartdashboard.MechanismLigament2d} fields that subclasses
 * populate and update</li>
 * </ul>
 *
 * <p>
 * Concrete mechanisms extend this class and supply the physics-specific details: how to read the
 * current position, how to drive the simulation model, and how to update visualization and
 * telemetry. The setpoint infrastructure and WPILib subsystem integration live here so that all
 * positional mechanisms share a consistent, command-based API without code duplication.
 * </p>
 *
 * <p>
 * <b>Conceptual subclass pattern:</b> A concrete positional mechanism extends
 * {@code SmartPositionalMechanism}, provides a typed command factory, and calls into the
 * base-class setpoint methods:
 * </p>
 * <pre>{@code
 * // A concrete mechanism delegates setpoint management to this base class:
 * public Command setAngle(Angle target) {
 *     return Commands.startRun(
 *         () -> setMechanismPositionSetpoint(target),
 *         () -> {},
 *         m_subsystem
 *     ).withName(getName() + " SetAngle");
 * }
 * }</pre>
 *
 * <h2>For Subclass Authors</h2>
 * <p>
 * Every concrete subclass of {@code SmartPositionalMechanism} must implement the following
 * abstract methods inherited from {@link yams.core.mechanisms.SmartMechanism}:
 * </p>
 * <ul>
 * <li>{@link yams.core.mechanisms.SmartMechanism#getRelativeMechanismPosition()} — returns the
 * current 3-D position of the mechanism end-point in {@link org.wpilib.smartdashboard.Mechanism2d}
 * coordinates</li>
 * <li>{@link yams.core.mechanisms.SmartMechanism#visualizationUpdate()} — updates
 * {@code m_mechanismLigament} (and any setpoint ligament) to reflect the current state</li>
 * <li>{@link yams.core.mechanisms.SmartMechanism#getName()} — returns a human-readable mechanism
 * name used for telemetry and command names</li>
 * <li>{@link yams.core.mechanisms.SmartMechanism#simIterate()} — advances the physics simulation
 * model and writes back simulated encoder values each robot loop</li>
 * <li>{@link yams.core.mechanisms.SmartMechanism#updateTelemetry()} — publishes mechanism state
 * to NetworkTables or an equivalent telemetry sink</li>
 * <li>{@link #isAtMax()} — true when the mechanism reaches its configured maximum limit</li>
 * <li>{@link #isAtMin()} — true when the mechanism reaches its configured minimum limit</li>
 * </ul>
 */
public abstract class SmartPositionalMechanism extends SmartMechanism {
  /**
   * The root point of the Mechanism.
   */
  protected MechanismRoot2d m_mechanismRoot;
  /**
   * The ligament that is being moved.
   */
  protected MechanismLigament2d m_mechanismLigament;

  /**
   * {@link SmartPositionalMechanism} is at max, defined by the soft limit or hard limit on the
   * {@link SmartPositionalMechanism}.
   *
   * @return True if the mechanism is at its configured maximum.
   */
  public abstract boolean isAtMax();

  /**
   * Minimum angle of the {@link SmartPositionalMechanism} given by the soft limit or hard limit of
   * the
   * {@link SmartPositionalMechanism}.
   *
   * @return True if the mechanism is at its configured minimum.
   */
  public abstract boolean isAtMin();

  /**
   * Get the ligament of the 2D mechanism model. Used to change the position of the mechanism model
   * in the SmartDashboard.
   *
   * @return Ligament of the 2D mechanism model.
   */
  public MechanismLigament2d getMechanismLigament() {
    return m_mechanismLigament;
  }

  /**
   * The root of the 2D mechanism model.
   *
   * @return Root of the 2D mechanism model.
   */
  public MechanismRoot2d getMechanismRoot() {
    return m_mechanismRoot;
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
