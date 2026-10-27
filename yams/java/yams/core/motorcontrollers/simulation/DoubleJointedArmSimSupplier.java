// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers.simulation;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Microsecond;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import java.util.UUID;
import java.util.function.Supplier;
import org.wpilib.math.filter.LinearFilter;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;
import yams.core.gearing.MechanismGearing;
import yams.core.math.DerivativeTimeFilter;
import yams.core.motorcontrollers.SimSupplier;
import yams.core.motorcontrollers.SmartMotorController;

/**
 * Sim supplier for one joint of a {@link DoubleJointedArmSim}.
 *
 * <p>Both joints share one simulation, which must step once a loop with both motors' voltages, so this supplier doesn't
 * step it: {@link #updateSimState()} only gives the simulation this joint's voltage, and
 * {@link yams.core.mechanisms.positional.DoubleJointedArm#simIterate()} steps the simulation once both joints have.
 */
public class DoubleJointedArmSimSupplier implements SimSupplier {
  /** A joint of the arm. */
  public enum Joint {
    /** The lower arm, at the shoulder. */
    LOWER,
    /** The upper arm, at the elbow. */
    UPPER
  }

  private boolean                    inputFed   = false;
  private boolean                    simUpdated = false;
  private final Supplier<Double>     motorDutyCycleSupplier;
  private final DerivativeTimeFilter accel;
  private final LinearFilter         supplyCurrentFilter;
  private final DoubleJointedArmSim  sim;
  private final Joint                joint;
  private final MechanismGearing     mechGearing;
  private final UUID                 uuid;

  /**
   * Construct the supplier for one joint.
   *
   * @param simulation           Simulation of the whole arm.
   * @param joint                Joint this supplier is for.
   * @param smartMotorController {@link SmartMotorController} driving the joint.
   */
  public DoubleJointedArmSimSupplier(DoubleJointedArmSim simulation, Joint joint, SmartMotorController smartMotorController) {
    var config = smartMotorController.getConfig();
    sim = simulation;
    this.joint = joint;
    motorDutyCycleSupplier = smartMotorController::getDutyCycle;
    mechGearing = config.getGearing();
    accel = new DerivativeTimeFilter(config.getSimulationPeriod());
    // Based off comment from https://github.com/wpilibsuite/allwpilib/issues/8691
    supplyCurrentFilter = LinearFilter.singlePoleIIR(Milliseconds.of(100).in(Seconds), config.getSimulationPeriod().in(Seconds));
    uuid = smartMotorController.m_batterySimUUID;
  }

  /** Give the simulation this joint's voltage, from the motor's duty cycle unless a voltage was set this loop. */
  @Override
  public void updateSimState() {
    if (!isInputFed()) {
      setInputVoltage(motorDutyCycleSupplier.get() * RoboRioSim.getVInVoltage());
      RoboRioSim.setVInVoltage(BatterySim.calculateVoltage(uuid, getSupplyCurrent()));
    }
    starveInput();
  }

  @Override
  public boolean getUpdatedSim() {
    return simUpdated;
  }

  @Override
  public void feedUpdateSim() {
    simUpdated = true;
  }

  @Override
  public void starveUpdateSim() {
    simUpdated = false;
  }

  @Override
  public boolean isInputFed() {
    return inputFed;
  }

  @Override
  public void feedInput() {
    inputFed = true;
  }

  @Override
  public void starveInput() {
    inputFed = false;
  }

  @Override
  public void setMechanismStatorDutyCycle(double dutyCycle) {
    feedInput();
    setInputVoltage(dutyCycle * getMechanismSupplyVoltage().in(Volts));
  }

  @Override
  public Voltage getMechanismSupplyVoltage() {
    return Volts.of(RoboRioSim.getVInVoltage());
  }

  @Override
  public Voltage getMechanismStatorVoltage() {
    return Volts.of(joint == Joint.LOWER ? sim.getLowerInputVoltage() : sim.getUpperInputVoltage());
  }

  @Override
  public void setMechanismStatorVoltage(Voltage volts) {
    feedInput();
    setInputVoltage(volts.in(Volts));
  }

  private void setInputVoltage(double volts) {
    if (joint == Joint.LOWER) {
      sim.setLowerInputVoltage(volts);
    } else {
      sim.setUpperInputVoltage(volts);
    }
  }

  @Override
  public Angle getMechanismPosition() {
    return Radians.of(joint == Joint.LOWER ? sim.getLowerAngle() : sim.getUpperAngle());
  }

  @Override
  public void setMechanismPosition(Angle position) {
    if (joint == Joint.LOWER) {
      sim.setState(position.in(Radians), sim.getLowerVelocity(), sim.getUpperAngle(), sim.getUpperVelocity());
    } else {
      sim.setState(sim.getLowerAngle(), sim.getLowerVelocity(), position.in(Radians), sim.getUpperVelocity());
    }
  }

  @Override
  public Angle getRotorPosition() {
    return getMechanismPosition().times(mechGearing.getMechanismToRotorRatio());
  }

  @Override
  public AngularVelocity getMechanismVelocity() {
    return RadiansPerSecond.of(joint == Joint.LOWER ? sim.getLowerVelocity() : sim.getUpperVelocity());
  }

  @Override
  public void setMechanismVelocity(AngularVelocity velocity) {
    if (joint == Joint.LOWER) {
      sim.setState(sim.getLowerAngle(), velocity.in(RadiansPerSecond), sim.getUpperAngle(), sim.getUpperVelocity());
    } else {
      sim.setState(sim.getLowerAngle(), sim.getLowerVelocity(), sim.getUpperAngle(), velocity.in(RadiansPerSecond));
    }
  }

  @Override
  public AngularVelocity getRotorVelocity() {
    return getMechanismVelocity().times(mechGearing.getMechanismToRotorRatio());
  }

  @Override
  public Current getStatorCurrent() {
    return Amps.of(joint == Joint.LOWER ? sim.getLowerCurrentDraw() : sim.getUpperCurrentDraw());
  }

  @Override
  public Current getSupplyCurrent() {
    // Power is conserved across the motor controller: supplyCurrent = dutyCycle * statorCurrent.
    return Amps.of(supplyCurrentFilter.calculate(motorDutyCycleSupplier.get() * getStatorCurrent().in(Amps)));
  }

  @Override
  public AngularAcceleration getRotorAcceleration() {
    return RotationsPerSecond.per(Microsecond).of(accel.derivative(getRotorVelocity().in(RotationsPerSecond)));
  }
}
