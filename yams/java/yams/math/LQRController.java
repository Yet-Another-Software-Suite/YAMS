// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

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
import java.util.Objects;
import java.util.Optional;
import yams.math.LQRConfig.LQRType;

/**
 * Linear Quadratic Regulator (Regulator is a type of controller)
 *
 * <p>An LQR computes optimal control inputs (voltages) that minimize a quadratic cost function
 * combining state error and control effort. It requires a linearized plant model, which is built
 * automatically from the physical parameters supplied via {@link LQRConfig}. Three mechanism types
 * are supported: {@code ARM}, {@code ELEVATOR}, and {@code FLYWHEEL}.
 *
 * <p>On each robot loop iteration, call the appropriate {@code calculate()} overload with the
 * current sensor measurement and the desired setpoint. The controller internally runs a
 * {@link edu.wpi.first.math.system.LinearSystemLoop} that fuses a
 * {@link edu.wpi.first.math.controller.LinearQuadraticRegulator} with a
 * {@link edu.wpi.first.math.estimator.KalmanFilter} observer.
 *
 * <h2>Example — construct from LQRConfig and calculate arm voltage</h2>
 * <pre>{@code
 * import static edu.wpi.first.units.Units.*;
 *
 * // Build configuration (see LQRConfig for full parameter details)
 * LQRConfig config = new LQRConfig(
 *         DCMotor.getNEO(1),
 *         MechanismGearing.ofReduction(60.0),
 *         KilogramSquareMeters.of(0.25))
 *     .withArm(
 *         Radians.of(0.01),
 *         RadiansPerSecond.of(0.5),
 *         Radians.of(0.05),
 *         RadiansPerSecond.of(0.5),
 *         Radians.of(0.01));
 *
 * LQRController controller = new LQRController(config);
 *
 * // In robotInit / subsystem initialization, reset to the current sensor state:
 * controller.reset(encoder.getAngle(), encoder.getAngularVelocity());
 *
 * // In periodic, calculate and apply voltage:
 * Voltage output = controller.calculate(
 *         encoder.getAngle(),           // measured position
 *         Radians.of(Math.PI / 4),      // position setpoint
 *         RadiansPerSecond.of(0));       // velocity setpoint
 * motor.setVoltage(output.in(Volts));
 * }</pre>
 */
public class LQRController
{
  private Optional<LQRConfig>          m_config           = Optional.empty();
  private LQRType                      m_type;
  private LinearSystemLoop<?, ?, ?>    m_loop;
  private Time                         m_period;

  /**
   * Create a LQR Controller.
   *
   * @param type   LQR Type.
   * @param loop   {@link LinearSystemLoop} which can be derived from {@link LQRConfig}
   * @param period Loop time.
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
   * Calculate the voltage for a Flywheel LQR
   *
   * @param measured Measured velocity of the flywheel.
   * @param velocity Setpoint velocity of the flywheel.
   * @return {@link Voltage} to apply to the flywheel.
   */
  public Voltage calculate(LinearVelocity measured, LinearVelocity velocity)
  {
    // Motion profiles are ignored for flywheels.
    LinearSystemLoop<N1, N1, N1> loop = (LinearSystemLoop<N1, N1, N1>) m_loop;
    loop.setNextR(velocity.in(MetersPerSecond));
    loop.correct((VecBuilder.fill(measured.in(MetersPerSecond))));
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
