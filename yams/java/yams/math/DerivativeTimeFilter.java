// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.math;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

/**
 * Find the derivative of a value over time in microseconds.
 *
 * <p>This filter computes a rate-of-change (derivative) estimate from successive measurements
 * timestamped with the FPGA clock. A built-in debounce timer prevents the derivative from being
 * recalculated faster than the configured period, which avoids amplifying high-frequency sensor
 * noise that would otherwise cause derivative kick.
 *
 * <p>Two {@code derivative()} overloads are available:
 * <ul>
 *   <li>{@link #derivative(double)} — uses the FPGA clock to measure elapsed time automatically.</li>
 *   <li>{@link #derivative(double, edu.wpi.first.units.measure.Time)} — uses a caller-supplied
 *       delta-time (useful when the loop period is already known).</li>
 * </ul>
 *
 * <h2>Example</h2>
 * <pre>{@code
 * import static edu.wpi.first.units.Units.Milliseconds;
 *
 * // Debounce period matches the 20 ms robot loop
 * DerivativeTimeFilter filter = new DerivativeTimeFilter(0.0, Milliseconds.of(20));
 *
 * // In periodic:
 * double velocity = filter.derivative(encoder.getPosition());
 * }</pre>
 */
public class DerivativeTimeFilter
{
  /**
   * Last value to derive from.
   */
  private double last;
  /**
   * Last FPGA time in microseconds
   */
  private long   lastFpgaTime_us;
  /**
   * Current derivation value within the loop period.
   */
  private double value          = 0;
  /**
   * Prevent the filter from being called too often.
   */
  private Timer  debouncer      = new Timer();
  /**
   * Prevent the filter from being called too often.
   */
  private Time   debouncePeriod = Seconds.of(0.02);

  /**
   * Create a derivative filter with an initial value
   *
   * @param initial         Initial value
   * @param debouncerPeriod Period to debounce the filter.
   * @implNote This value is timestamped at the time of construction.
   */
  public DerivativeTimeFilter(double initial, Time debouncerPeriod)
  {
    last = initial;
    lastFpgaTime_us = RobotController.getFPGATime();
    debouncer = new Timer();
    this.debouncePeriod = debouncerPeriod;
    debouncer.start();
  }

  /**
   * Create a derivative filter with no initial value
   *
   * @param debouncerPeriod Period to debounce the filter.
   */
  public DerivativeTimeFilter(Time debouncerPeriod)
  {
    last = 0;
    lastFpgaTime_us = 0;
    debouncer = new Timer();
    this.debouncePeriod = debouncerPeriod;
    debouncer.start();
  }

  /**
   * Get the derivative of the current value over the specified delta.
   *
   * @param current Current value
   * @param dt      Delta time
   * @return Derivative of the current value from the previous value over the delta time in microseconds.
   * @implNote If this function is not called periodically at the dt specified, the derivative will be incorrect
   */
  public double derivative(double current, Time dt)
  {
    if (debouncer.advanceIfElapsed(debouncePeriod.in(Seconds)))
    {
      double derivative = (current - last) / dt.in(Microseconds);
      last = current;
      value = derivative;
      return derivative;
    }
    return value;
  }

  /**
   * Get the derivative of the current value over the time since the last call to this function.
   *
   * @param current Current value
   * @return Derivative of the current value from the previous value over the time since the last call to this in
   * microseconds.
   */
  public double derivative(double current)
  {
    if (debouncer.hasElapsed(debouncePeriod))
    {
      long   currentFpgaTime_us = RobotController.getFPGATime();
      double derivative         = derivative(current, Microseconds.of(currentFpgaTime_us - lastFpgaTime_us));
      lastFpgaTime_us = currentFpgaTime_us;
      return derivative;
    }

    return value;
  }


}
