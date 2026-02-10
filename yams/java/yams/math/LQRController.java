package yams.math;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import yams.math.LQRConfig.LQRType;

/**
 * Linear Quadratic Regulator (Regulator is a type of controller)
 */
public class LQRController
{

  private final LQRType m_type;
  private final LinearSystemLoop<?, ?, ?> m_loop;
  private final Time                      m_period;

  /**
   * Create a LQR Controller.
   *
   * @param loop {@link LinearSystemLoop} which can be derived from {@link LQRConfig}
   */
  public LQRController(LQRType type, LinearSystemLoop<?, ?, ?> loop, Time period)
  {
    m_type = type;
    m_loop = loop;
    m_period = period;
  }

  /**
   * Create a LQR Controller.
   *
   * @param config {@link LQRConfig} to create the controller from.
   */
  public LQRController(LQRConfig config)
  {
    m_type = config.getType();
    m_loop = config.getLoop();
    m_period = config.getPeriod();
  }

  /**
   * Calculate the voltage for an Arm LQR
   *
   * @param measured Measured angle from the arm.
   * @param position {@link Angle} setpoint of the arm from the motion profile.
   * @param velocity {@link AngularVelocity} setpoint of the arm from the motion profile.
   * @return {@link Voltage} to apply to the arm.
   */
  public Voltage calculate(Angle measured, Angle position, AngularVelocity velocity)
  {
    LinearSystemLoop<N2, N1, N1> loop = (LinearSystemLoop<N2, N1, N1>) m_loop;
    loop.setNextR(position.in(Radians), velocity.in(RadiansPerSecond));
    loop.correct((VecBuilder.fill(measured.in(Radians))));
    loop.predict(m_period.in(Seconds));
    return Volts.of(loop.getU(0));
  }

  /**
   * Calculate the voltage for a Elevator LQR
   *
   * @param measured Measured distance of the elevator.
   * @param position Setpoint distance of the elevator from the motion profile.
   * @param velocity Setpoint velocity of the elevator from the motion profile.
   * @return {@link Voltage} to apply to the elevator.
   */
  public Voltage calculate(Distance measured, Distance position, LinearVelocity velocity)
  {
    LinearSystemLoop<N2, N1, N1> loop = (LinearSystemLoop<N2, N1, N1>) m_loop;
    loop.setNextR(position.in(Meters), velocity.in(MetersPerSecond));
    loop.correct((VecBuilder.fill(measured.in(Meters))));
    loop.predict(m_period.in(Seconds));
    return Volts.of(loop.getU(0));
  }

  /**
   * Calculate the voltage for a Flywheel LQR
   *
   * @param measured Measured velocity of the flywheel.
   * @param velocity Setpoint velocity of the flywheel.
   * @return {@link Voltage} to apply to the flywheel.
   */
  public Voltage calculate(AngularVelocity measured, AngularVelocity velocity)
  {
    LinearSystemLoop<N1, N1, N1> loop = (LinearSystemLoop<N1, N1, N1>) m_loop;
    loop.setNextR(velocity.in(RadiansPerSecond));
    loop.correct((VecBuilder.fill(measured.in(RadiansPerSecond))));
    loop.predict(m_period.in(Seconds));
    return Volts.of(loop.getU(0));
  }

  /**
   * Get the type of LQR.
   *
   * @return {@link LQRType}
   */
  public LQRType getType()
  {
    return m_type;
  }


}
