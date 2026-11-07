// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers.simulation;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;
import yams.core.motorcontrollers.SimSupplier;
import yams.core.motorcontrollers.SmartMotorController;

/**
 * Simulation of a motor whose leader's simulation has its motor too, such as an A301 following
 * another, or a loosely coupled follower of a mechanism's motor controller. It reads the leader's
 * simulation, with its rotor values from its own gearing and its share of the current, and does
 * not step a simulation of its own. Its current is in the leader's, so {@link BatterySim} does not
 * count it again.
 */
public class FollowerSimSupplier implements SimSupplier {
  /** Motor controller whose simulation has this one's motor. */
  private final SmartMotorController m_leader;

  /** Motor controller reading the leader's simulation. */
  private final SmartMotorController m_follower;

  /** -1 when the follower is inverted from its leader, otherwise 1. */
  private final double m_sign;

  /**
   * Read the simulation of a leader whose motor model has this follower's motor.
   *
   * @param leader Motor controller whose simulation has the follower's motor.
   * @param follower Motor controller reading it.
   * @param inverted Whether the follower is inverted from its leader.
   */
  public FollowerSimSupplier(
      SmartMotorController leader, SmartMotorController follower, boolean inverted) {
    m_leader = leader;
    m_follower = follower;
    m_sign = inverted ? -1 : 1;
  }

  /**
   * The leader's simulation, which a mechanism can replace with its own.
   *
   * @return The leader's {@link SimSupplier}.
   */
  private SimSupplier leader() {
    return m_leader.getSimSupplier().orElseThrow();
  }

  /**
   * Gearing from the follower's rotor to the mechanism.
   *
   * @return Rotor rotations per mechanism rotation.
   */
  private double rotorRatio() {
    return m_follower.getConfig().getGearing().getMechanismToRotorRatio();
  }

  /**
   * The follower's share of the leader's current, by its share of the stall current.
   *
   * @param current Current of every motor in the leader's simulation.
   * @return Current of the follower.
   */
  private Current share(Current current) {
    return current.times(
        m_follower.getDCMotor().stallCurrent / m_leader.getMechanismDCMotor().stallCurrent);
  }

  @Override
  public void updateSimState() {}

  @Override
  public boolean getUpdatedSim() {
    return true;
  }

  @Override
  public void feedUpdateSim() {}

  @Override
  public void starveUpdateSim() {}

  @Override
  public boolean isInputFed() {
    return true;
  }

  @Override
  public void feedInput() {}

  @Override
  public void starveInput() {}

  @Override
  public void setMechanismStatorDutyCycle(double dutyCycle) {}

  @Override
  public Voltage getMechanismSupplyVoltage() {
    return leader().getMechanismSupplyVoltage();
  }

  @Override
  public Voltage getMechanismStatorVoltage() {
    return leader().getMechanismStatorVoltage().times(m_sign);
  }

  @Override
  public void setMechanismStatorVoltage(Voltage volts) {}

  @Override
  public Angle getMechanismPosition() {
    return leader().getMechanismPosition();
  }

  @Override
  public void setMechanismPosition(Angle position) {}

  @Override
  public Angle getRotorPosition() {
    return getMechanismPosition().times(rotorRatio());
  }

  @Override
  public AngularVelocity getMechanismVelocity() {
    return leader().getMechanismVelocity();
  }

  @Override
  public void setMechanismVelocity(AngularVelocity velocity) {}

  @Override
  public AngularVelocity getRotorVelocity() {
    return getMechanismVelocity().times(rotorRatio());
  }

  @Override
  public Current getStatorCurrent() {
    return share(leader().getStatorCurrent());
  }

  @Override
  public Current getSupplyCurrent() {
    return share(leader().getSupplyCurrent());
  }

  @Override
  public AngularAcceleration getRotorAcceleration() {
    return leader()
        .getRotorAcceleration()
        .times(rotorRatio() / m_leader.getConfig().getGearing().getMechanismToRotorRatio());
  }
}
