package yams.mechanisms.positional;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import yams.mechanisms.SmartMechanism;
import yams.motorcontrollers.SmartMotorController;

/**
 * Generic for positional mechanisms.
 */
public abstract class SmartPositionalMechanism extends SmartMechanism
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
   * {@link SmartPositionalMechanism} is at max, defined by the soft limit or hard limit on the
   * {@link SmartPositionalMechanism}.
   *
   * @return Maximum angle for the {@link SmartPositionalMechanism}.
   */
  public abstract Trigger max();

  /**
   * Minimum angle of the {@link SmartPositionalMechanism} given by the soft limit or hard limit of the
   * {@link SmartPositionalMechanism}.
   *
   * @return {@link Trigger} on minimum of the {@link SmartPositionalMechanism}.
   */
  public abstract Trigger min();

  /**
   * Create the SysId routine and commands to run the SysId tests. The SysId test will run the mechanism up then down at
   * a constant speed, then run the mechanism up at an increasing speed and down at an increasing speed. Requires the
   * maximum and minimum limit to be set. Runs the mechanism within 1 degree of the maximum and minimum.
   *
   * @param maximumVoltage Maximum {@link Voltage} to give to the {@link SmartPositionalMechanism}, is the voltage given
   *                       to run the {@link SmartPositionalMechanism} up at a static speed.
   * @param step           Step {@link Voltage} to give to the {@link SmartPositionalMechanism}.
   * @param duration       SysId test duration.
   * @return {@link edu.wpi.first.wpilibj2.command.SequentialCommandGroup} running the SysId commands.
   */
  public abstract Command sysId(Voltage maximumVoltage, Velocity<VoltageUnit> step, Time duration);

  /**
   * Get the ligament of the 2D mechanism model. Used to change the position of the mechanism model
   * in the SmartDashboard.
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
    return m_motor;
  }
}
