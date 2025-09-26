package yams.motorcontrollers;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * Provides sim functions for
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

}
