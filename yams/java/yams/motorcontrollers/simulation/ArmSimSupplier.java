package yams.motorcontrollers.simulation;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import java.util.function.Supplier;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SimSupplier;
import yams.motorcontrollers.SmartMotorController;

/**
 * ArmSim Supplier
 */
public class ArmSimSupplier implements SimSupplier
{

  private       boolean             inputFed   = false;
  private       boolean             simUpdated = false;
  private final Supplier<Double>    motorDutyCycleSupplier;
  private final SingleJointedArmSim sim;
  private final MechanismGearing    mechGearing;
  private final Time                period;
  private final DCMotor             motor;


  /**
   * Construct the ArmSim supplier
   *
   * @param simulation           Simulatoin instance
   * @param smartMotorController SMC for the ArmSim..
   */
  public ArmSimSupplier(SingleJointedArmSim simulation, SmartMotorController smartMotorController)
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
    return Volts.of(motor.getVoltage(motor.getTorque(sim.getCurrentDrawAmps()),
                                     sim.getVelocityRadPerSec()));
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
    return Radians.of(sim.getAngleRads());
  }

  @Override
  public void setMechanismPosition(Angle position)
  {
    sim.setState(position.in(Radians),
                 sim.getVelocityRadPerSec());//.times(config.getGearing().getMechanismToRotorRatio()).in(Radians));
  }

  @Override
  public Angle getRotorPosition()
  {
    return getMechanismPosition().times(mechGearing.getMechanismToRotorRatio());
  }

  @Override
  public AngularVelocity getMechanismVelocity()
  {
    return RadiansPerSecond.of(sim.getVelocityRadPerSec());
  }

  @Override
  public void setMechanismVelocity(AngularVelocity velocity)
  {
    sim.setState(sim.getAngleRads(), velocity.in(RadiansPerSecond));
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
}
