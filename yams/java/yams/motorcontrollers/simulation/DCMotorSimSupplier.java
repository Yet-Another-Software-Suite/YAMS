// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.motorcontrollers.simulation;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.function.Supplier;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SimSupplier;
import yams.motorcontrollers.SmartMotorController;

/**
 * DCMotorSim Supplier — simulates a generic DC motor load (flywheel, roller, or elevator)
 * using WPILib's {@link edu.wpi.first.wpilibj.simulation.DCMotorSim}.
 *
 * <p>
 * This supplier steps WPILib's {@code DCMotorSim} physics model each control loop and exposes the
 * resulting angular position, angular velocity, current draw, and acceleration through the
 * {@link yams.motorcontrollers.SimSupplier} interface. Unlike
 * {@link yams.motorcontrollers.simulation.ArmSimSupplier}, this model does not simulate gravity or
 * joint limits — it is suited for continuous-rotation mechanisms such as flywheels or rollers, as
 * well as linear mechanisms (elevators) when paired with appropriate gearing.
 * </p>
 *
 * <p>
 * The gear ratio and control period are read from the associated
 * {@link yams.motorcontrollers.SmartMotorController}'s config, so they do not need to be repeated
 * here.
 * </p>
 *
 * <h2>Example</h2>
 * <pre>{@code
 * // 1. Build the WPILib DC motor physics model (e.g. a flywheel with MOI 0.001 kg·m²)
 * DCMotorSim flywheelPhysics = new DCMotorSim(
 *     LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.001, 1.0),
 *     DCMotor.getNEO(1));
 *
 * // 2. Configure and build the YAMS smart motor controller
 * SmartMotorController motor = new SparkMaxController(
 *     new SmartMotorControllerConfig()
 *         .withGearing(new MechanismGearing(1.0))
 *         .withClosedLoopControlPeriod(Milliseconds.of(20)));
 *
 * // 3. Wrap physics model in the supplier and register it
 * DCMotorSimSupplier sim = new DCMotorSimSupplier(flywheelPhysics, motor);
 * motor.getConfig().withSimSupplier(sim);
 * }</pre>
 */
public class DCMotorSimSupplier implements SimSupplier
{
  private boolean          inputFed   = false;
  private       boolean          simUpdated = false;
  private final Supplier<Double> motorDutyCycleSupplier;
  private final DCMotorSim       sim;
  private final MechanismGearing mechGearing;
  private final Time             period;
  private final DCMotor          motor;


  /**
   * Construct the DCMotorSim supplier
   *
   * @param simulation           Simulatoin instance
   * @param smartMotorController SMC for the DCMotorSim..
   */
  public DCMotorSimSupplier(DCMotorSim simulation, SmartMotorController smartMotorController)
  {
    var config = smartMotorController.getConfig();
    sim = simulation;
    motorDutyCycleSupplier = smartMotorController::getDutyCycle;
    mechGearing = config.getGearing();
    period = config.getClosedLoopControlPeriod().orElse(Milliseconds.of(20));
    motor = smartMotorController.getDCMotor();
  }

  @Override
  public void updateSimState()
  {
    if (!isInputFed())
    {
      sim.setInputVoltage(motorDutyCycleSupplier.get() * RoboRioSim.getVInVoltage());
    }
    if (!simUpdated)
    {
      starveInput();
      sim.update(period.in(Seconds));
      try
      {
        //Thread.sleep(1);
      } catch (Exception e)
      {
      }
      feedUpdateSim();
    }

  }

  @Override
  public boolean getUpdatedSim()
  {
    return simUpdated;
  }

  @Override
  public void feedUpdateSim()
  {
    simUpdated = true;
  }

  @Override
  public void starveUpdateSim()
  {
    simUpdated = false;
  }

  @Override
  public boolean isInputFed()
  {
    return inputFed;
  }

  @Override
  public void feedInput()
  {
    inputFed = true;
  }

  @Override
  public void starveInput()
  {
    inputFed = false;
  }

  @Override
  public void setMechanismStatorDutyCycle(double dutyCycle)
  {
    feedInput();
    sim.setInputVoltage(dutyCycle * getMechanismSupplyVoltage().in(Volts));
  }

  @Override
  public Voltage getMechanismSupplyVoltage()
  {
    return Volts.of(RoboRioSim.getVInVoltage());
  }

  @Override
  public Voltage getMechanismStatorVoltage()
  {
    return Volts.of(motor.getVoltage(sim.getTorqueNewtonMeters(),
                                     sim.getAngularVelocityRadPerSec()));
  }

  @Override
  public void setMechanismStatorVoltage(Voltage volts)
  {
    feedInput();
    sim.setInputVoltage(volts.in(Volts));
  }

  @Override
  public Angle getMechanismPosition()
  {
    return sim.getAngularPosition();
  }

  @Override
  public void setMechanismPosition(Angle position)
  {
    sim
        .setAngle(position.in(Radians));//.times(config.getGearing().getMechanismToRotorRatio()).in(Radians));
  }

  @Override
  public Angle getRotorPosition()
  {
    return getMechanismPosition().times(mechGearing.getMechanismToRotorRatio());
  }

  @Override
  public AngularVelocity getMechanismVelocity()
  {
    return sim.getAngularVelocity();
  }

  @Override
  public void setMechanismVelocity(AngularVelocity velocity)
  {
    sim.setAngularVelocity(velocity.in(RadiansPerSecond));
  }

  @Override
  public AngularVelocity getRotorVelocity()
  {
    return getMechanismVelocity().times(mechGearing.getMechanismToRotorRatio());
  }

  @Override
  public Current getCurrentDraw()
  {
    return Amps.of(sim.getCurrentDrawAmps());
  }

  @Override
  public AngularAcceleration getRotorAcceleration()
  {
    return sim.getAngularAcceleration();
  }
}
