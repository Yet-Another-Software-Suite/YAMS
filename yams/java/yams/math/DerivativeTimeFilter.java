package yams.math;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

/**
 * Find the derivative of a value over time in microseconds.
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
  private long   lastFpgaTime_ms;
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
    lastFpgaTime_ms = RobotController.getFPGATime();
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
    lastFpgaTime_ms = 0;
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
      long   currentFpgaTime_ms = RobotController.getFPGATime();
      double derivative         = derivative(current, Microseconds.of(current - lastFpgaTime_ms));
      lastFpgaTime_ms = currentFpgaTime_ms;
      return derivative;
    }

    return value;
  }


}
