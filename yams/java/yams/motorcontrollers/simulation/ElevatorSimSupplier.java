// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.motorcontrollers.simulation;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Microsecond;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.simulation.RoboRioSim;
import java.util.UUID;
import java.util.function.Supplier;
import yams.gearing.MechanismGearing;
import yams.math.DerivativeTimeFilter;
import yams.motorcontrollers.SimSupplier;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;

/**
 * ElevatorSim Supplier — simulates an elevator mechanism using WPILib's
 * {@link org.wpilib.simulation.ElevatorSim}.
 *
 * <p>
 * This supplier steps WPILib's {@code ElevatorSim} physics model each control loop and exposes
 * the resulting height, linear velocity, current draw, and voltage through the
 * {@link yams.motorcontrollers.SimSupplier} interface. Because {@code ElevatorSim} operates in
 * linear units (meters), positions and velocities are converted to and from mechanism (angular)
 * units using the associated {@link yams.motorcontrollers.SmartMotorController}'s config.
 * </p>
 *
 * <h2>Example</h2>
 * <pre>{@code
 * // 1. Build the WPILib elevator physics model
 * ElevatorSim elevatorPhysics = new ElevatorSim(
 *     DCMotor.getNEO(1),
 *     10.0,                 // gear ratio (rotor/mechanism)
 *     5.0,                  // carriage mass (kg)
 *     Units.inchesToMeters(1.0), // drum radius (meters)
 *     0.0,                  // minimum height (meters)
 *     1.5,                  // maximum height (meters)
 *     true,                 // simulate gravity
 *     0.0);                 // starting height (meters)
 *
 * // 2. Configure and build the YAMS smart motor controller
 * SmartMotorController motor = new SparkMaxController(
 *     new SmartMotorControllerConfig()
 *         .withGearing(new MechanismGearing(10.0))
 *         .withClosedLoopControlPeriod(Milliseconds.of(20)));
 *
 * // 3. Wrap physics model in the supplier and register it
 * ElevatorSimSupplier sim = new ElevatorSimSupplier(elevatorPhysics, motor);
 * motor.getConfig().withSimSupplier(sim);
 * }</pre>
 */
public class ElevatorSimSupplier implements SimSupplier {
  private final ElevatorSim sim;
  private final SmartMotorControllerConfig config;
  private final MechanismGearing mechGearing;
  private final DCMotor motor;
  private final UUID uuid;
  private final Supplier<Double> motorDutyCycleSupplier;
  private final Supplier<Double> pos;
  private final Supplier<Double> mps;
  private final DerivativeTimeFilter mpsps;
  private boolean inputFed = false;
  private boolean simUpdated = false;

  /**
   * Construct the ElevatorSim supplier
   *
   * @param simulation           Simulation instance
   * @param smartMotorController SMC for the ElevatorSim.
   */
  public ElevatorSimSupplier(ElevatorSim simulation, SmartMotorController smartMotorController) {
    sim = simulation;
    config = smartMotorController.getConfig();
    mechGearing = config.getGearing();
    motor = smartMotorController.getDCMotor();
    uuid = smartMotorController.m_batterySimUUID;
    motorDutyCycleSupplier = smartMotorController::getDutyCycle;
    pos = sim::getPosition;
    mps = sim::getVelocity;
    mpsps = new DerivativeTimeFilter(
        pos.get(), config.getClosedLoopControlPeriod().orElse(Milliseconds.of(20)));
  }

  @Override
  public void updateSimState() {
    if (!isInputFed()) {
      sim.setInputVoltage(motorDutyCycleSupplier.get() * RoboRioSim.getVInVoltage());
      RoboRioSim.setVInVoltage(BatterySim.calculateVoltage(uuid, sim.getCurrentDraw()));
    }
    if (!simUpdated) {
      starveInput();
      sim.update(config.getClosedLoopControlPeriod().orElse(Milliseconds.of(20)).in(Seconds));
      feedUpdateSim();
    }
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
    sim.setInputVoltage(dutyCycle * getMechanismSupplyVoltage().in(Volts));
  }

  @Override
  public Voltage getMechanismSupplyVoltage() {
    return Volts.of(RoboRioSim.getVInVoltage());
  }

  @Override
  public Voltage getMechanismStatorVoltage() {
    return Volts.of(motor.getVoltage(
        motor.getTorque(sim.getCurrentDraw()), getMechanismVelocity().in(RadiansPerSecond)));
  }

  @Override
  public void setMechanismStatorVoltage(Voltage volts) {
    feedInput();
    sim.setInputVoltage(volts.in(Volts));
  }

  @Override
  public Angle getMechanismPosition() {
    return config.convertToMechanism(Meters.of(pos.get()));
  }

  @Override
  public void setMechanismPosition(Angle position) {
    sim.setState(config.convertFromMechanism(position).in(Meters), mps.get());
  }

  @Override
  public Angle getRotorPosition() {
    return getMechanismPosition().times(mechGearing.getMechanismToRotorRatio());
  }

  @Override
  public AngularVelocity getMechanismVelocity() {
    return config.convertToMechanism(MetersPerSecond.of(mps.get()));
  }

  @Override
  public void setMechanismVelocity(AngularVelocity velocity) {
    sim.setState(pos.get(), config.convertFromMechanism(velocity).in(MetersPerSecond));
  }

  @Override
  public AngularVelocity getRotorVelocity() {
    return getMechanismVelocity().times(mechGearing.getMechanismToRotorRatio());
  }

  @Override
  public Current getCurrentDraw() {
    return Amps.of(sim.getCurrentDraw());
  }

  @Override
  public AngularAcceleration getRotorAcceleration() {
    return RotationsPerSecond.per(Microsecond)
        .of(mpsps.derivative(getRotorVelocity().in(RotationsPerSecond)));
  }
}
