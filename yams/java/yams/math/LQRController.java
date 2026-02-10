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
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import java.util.Objects;
import java.util.Optional;
import yams.math.LQRConfig.LQRType;

/**
 * Linear Quadratic Regulator (Regulator is a type of controller)
 */
public class LQRController
{

  private Optional<LQRConfig>          m_config           = Optional.empty();
  private LQRType                      m_type;
  private LinearSystemLoop<?, ?, ?>    m_loop;
  private Time                         m_period;
  private Optional<ExponentialProfile> m_expoProfile      = Optional.empty();
  private Optional<TrapezoidProfile>   m_trapProfile      = Optional.empty();
  private ExponentialProfile.State     m_currentExpoState = new ExponentialProfile.State();
  private TrapezoidProfile.State       m_currentTrapState = new TrapezoidProfile.State();

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
    m_config = Optional.of(config);
    m_type = config.getType();
    m_loop = config.getLoop();
    m_period = config.getPeriod();
    m_trapProfile = config.getTrapezoidalProfile();
    m_expoProfile = config.getExponentialProfile();
  }

  /**
   * Update LQR based off config.
   *
   * @param config {@link LQRConfig}
   */
  public void updateConfig(LQRConfig config)
  {
    m_config = Optional.of(config);
    m_type = config.getType();
    m_loop = config.getLoop();
    m_period = config.getPeriod();
    m_trapProfile = config.getTrapezoidalProfile();
    m_expoProfile = config.getExponentialProfile();
    m_currentExpoState = new ExponentialProfile.State();
    m_currentTrapState = new TrapezoidProfile.State();
  }

  /**
   * Set the motion profile.
   *
   * @param exponentialProfile Exponential profile to use, must be in meters/s or radians/s.
   */
  public void setProfile(ExponentialProfile exponentialProfile)
  {
    m_expoProfile = Optional.of(exponentialProfile);
    m_trapProfile = Optional.empty();
    m_currentExpoState = new ExponentialProfile.State();
    m_currentTrapState = new TrapezoidProfile.State();
  }

  /**
   * Set the motion profile.
   *
   * @param trapezoidProfile Trapezoid profile to use, must be in meters/s or radians/s.
   */
  public void setProfile(TrapezoidProfile trapezoidProfile)
  {
    m_expoProfile = Optional.empty();
    m_trapProfile = Optional.of(trapezoidProfile);
    m_currentExpoState = new ExponentialProfile.State();
    m_currentTrapState = new TrapezoidProfile.State();
  }

  /**
   * Reset the Arm or Flywheel LQR.
   *
   * @param angle    Current angle.
   * @param velocity Current velocity.
   */
  public void reset(Angle angle, AngularVelocity velocity)
  {
    switch (m_type)
    {
      case FLYWHEEL ->
      {
        ((LinearSystemLoop<N1, N1, N1>) m_loop).reset(VecBuilder.fill(velocity.in(RadiansPerSecond)));
      }
      case ARM ->
      {
        ((LinearSystemLoop<N2, N1, N1>) m_loop).reset(VecBuilder.fill(angle.in(Radians),
                                                                      velocity.in(RadiansPerSecond)));
      }
    }
    m_currentExpoState = new ExponentialProfile.State(angle.in(Radians), velocity.in(RadiansPerSecond));
    m_currentTrapState = new TrapezoidProfile.State(angle.in(Radians), velocity.in(RadiansPerSecond));
  }

  /**
   * Reset the Elevator LQR.
   *
   * @param distance Current distance.
   * @param velocity Current velocity.
   */
  public void reset(Distance distance, LinearVelocity velocity)
  {
    if (Objects.requireNonNull(m_type) == LQRType.ELEVATOR)
    {
      ((LinearSystemLoop<N2, N1, N1>) m_loop).reset(VecBuilder.fill(distance.in(Meters),
                                                                    velocity.in(MetersPerSecond)));
    }
    m_currentExpoState = new ExponentialProfile.State(distance.in(Meters), velocity.in(MetersPerSecond));
    m_currentTrapState = new TrapezoidProfile.State(distance.in(Meters), velocity.in(MetersPerSecond));
  }

  /**
   * Calculate the voltage for an Arm LQR
   *
   * @param measured Measured angle from the arm.
   * @param position {@link Angle} setpoint to go to.
   * @param velocity {@link AngularVelocity} setpoint to go to.
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
   * Calculate the voltage for a Elevator LQR
   *
   * @param measured Measured distance of the elevator.
   * @param position Setpoint distance of the elevator from the motion profile.
   * @return {@link Voltage} to apply to the elevator.
   */
  public Voltage calculate(Distance measured, Distance position)
  {
    if (m_expoProfile.isPresent())
    {
      var goal    = new ExponentialProfile.State(position.in(Meters), 0.0);
      var profile = m_expoProfile.get();
      m_currentExpoState = profile.calculate(m_period.in(Seconds), m_currentExpoState, goal);
      return calculate(measured,
                       Meters.of(m_currentExpoState.position),
                       MetersPerSecond.of(m_currentExpoState.velocity));
    }
    if (m_trapProfile.isPresent())
    {
      var goal    = new TrapezoidProfile.State(position.in(Meters), 0.0);
      var profile = m_trapProfile.get();
      m_currentTrapState = profile.calculate(m_period.in(Seconds), m_currentTrapState, goal);
      return calculate(measured,
                       Meters.of(m_currentTrapState.position),
                       MetersPerSecond.of(m_currentTrapState.velocity));
    }
    throw new IllegalStateException("No profile set for elevator");
  }

  /**
   * Calculate the voltage for an Arm LQR
   *
   * @param measured Measured angle from the arm.
   * @param position {@link Angle} setpoint to go to.
   * @return {@link Voltage} to apply to the arm.
   */
  public Voltage calculate(Angle measured, Angle position)
  {
    if (m_expoProfile.isPresent())
    {
      var goal    = new ExponentialProfile.State(position.in(Radians), 0.0);
      var profile = m_expoProfile.get();
      m_currentExpoState = profile.calculate(m_period.in(Seconds), m_currentExpoState, goal);
      return calculate(measured,
                       Radians.of(m_currentExpoState.position),
                       RadiansPerSecond.of(m_currentExpoState.velocity));
    }
    if (m_trapProfile.isPresent())
    {
      var goal    = new TrapezoidProfile.State(position.in(Radians), 0.0);
      var profile = m_trapProfile.get();
      m_currentTrapState = profile.calculate(m_period.in(Seconds), m_currentTrapState, goal);
      return calculate(measured,
                       Radians.of(m_currentTrapState.position),
                       RadiansPerSecond.of(m_currentTrapState.velocity));
    }
    throw new IllegalStateException("No profile set for arm");
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
    // Motion profiles are ignored for flywheels.
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

  /**
   * Get the {@link LQRConfig} for the LQR.
   *
   * @return {@link LQRConfig}
   */
  public Optional<LQRConfig> getConfig()
  {
    return m_config;
  }

}
