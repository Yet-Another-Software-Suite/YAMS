// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.motorcontrollers;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * Provides sim functions for a YAMS simulated mechanism.
 *
 * <p>
 * {@code SimSupplier} is the abstract bridge between WPILib's physics simulation models (such as
 * {@link edu.wpi.first.wpilibj.simulation.SingleJointedArmSim} and
 * {@link edu.wpi.first.wpilibj.simulation.DCMotorSim}) and YAMS
 * {@link yams.motorcontrollers.SmartMotorController} wrappers. Concrete implementations
 * translate the simulation state — position, velocity, current draw, voltage — into the
 * typed unit-safe values that YAMS motor controller wrappers consume each control loop.
 * </p>
 *
 * <p>
 * On a real robot the motor controller hardware provides these values; in simulation a
 * {@code SimSupplier} steps the physics model forward and exposes the same interface so
 * that mechanism and control code requires no changes between real and simulated runs.
 * </p>
 *
 * <h2>How to use</h2>
 * <p>
 * Create a concrete {@code SimSupplier} (e.g. {@link yams.motorcontrollers.simulation.ArmSimSupplier}
 * or {@link yams.motorcontrollers.simulation.DCMotorSimSupplier}) and pass it to
 * {@code SmartMotorControllerConfig} via {@code withSimSupplier()}:
 * </p>
 * <pre>{@code
 * // Create the WPILib physics model
 * SingleJointedArmSim armPhysics = new SingleJointedArmSim(
 *     DCMotor.getNEO(1),
 *     SingleJointedArmSim.estimateMOI(0.5, 2.0),
 *     5.0,
 *     0.5,
 *     Units.degreesToRadians(-10),
 *     Units.degreesToRadians(90),
 *     true,
 *     0);
 *
 * // Wrap it in a YAMS supplier
 * SmartMotorControllerConfig config = new SmartMotorControllerConfig()
 *     // ... other config ...
 *     .withSimSupplier(new ArmSimSupplier(armPhysics, motorController));
 * }</pre>
 *
 * <p>
 * Implementations must advance the physics model on each control loop tick via
 * {@link #updateSimState()}, and signal readiness through the watchdog helpers
 * ({@link #feedUpdateSim()}/{@link #starveUpdateSim()}).
 * </p>
 */
public interface SimSupplier
{
  /**
   * Update the sim state.
   */
  void updateSimState();

  /**
   * Get the updated sim watchdog.
   *
   * @return Updated sim.
   */
  boolean getUpdatedSim();

  /**
   * Feed the update sim watch
   */
  void feedUpdateSim();

  /**
   * Starve the update sim watch.
   */
  void starveUpdateSim();

  /**
   * Check if the input was fed.
   *
   * @return Input fed.
   */
  boolean isInputFed();

  /**
   * Feed input
   *
   */
  void feedInput();

  /**
   * Starve the input.
   */
  void starveInput();

  /**
   * Set the dutycyle of the mechanism stator.
   *
   * @param dutyCycle Dutycycle value.
   */
  void setMechanismStatorDutyCycle(double dutyCycle);

  /**
   * Gets the supply voltage for the motor controller.
   *
   * @return Supply voltage to the motor controller
   */
  Voltage getMechanismSupplyVoltage();

  /**
   * Get the mechanism stator voltage.
   *
   * @return Stator voltage of the mechanism.
   */
  Voltage getMechanismStatorVoltage();

  /**
   * Set mechanism voltage, mostly used for SysId testing.
   *
   * @param volts Voltage to set.
   */
  void setMechanismStatorVoltage(Voltage volts);

  /**
   * Get the mechanism position.
   *
   * @return mechanism angle.
   */
  Angle getMechanismPosition();

  /**
   * Set the Mechanism position
   *
   * @param position Position of the mechanism.
   */
  void setMechanismPosition(Angle position);

  /**
   * Get the rotor position.
   *
   * @return rotor position.
   */
  Angle getRotorPosition();

  /**
   * Get the mechanism velocity.
   *
   * @return Mechanism velocity.
   */
  AngularVelocity getMechanismVelocity();

  /**
   * Set the Mechanism velocity.
   *
   * @param velocity Mechanism velocity.
   */
  void setMechanismVelocity(AngularVelocity velocity);

  /**
   * Get the rotor velocity.
   *
   * @return rotor velocity.
   */
  AngularVelocity getRotorVelocity();

  /**
   * Get the current draw of from the sim.
   *
   * @return Current draw.
   */
  Current getCurrentDraw();

  /**
   * Get the rotor acceleration.
   *
   * @return Rotor acceleration.
   */
  AngularAcceleration getRotorAcceleration();
}
