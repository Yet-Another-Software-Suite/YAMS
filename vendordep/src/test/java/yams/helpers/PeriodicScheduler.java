// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.helpers;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import java.util.PriorityQueue;

/**
 * Minimal, deterministic stand-in for how {@link edu.wpi.first.wpilibj.TimedRobot} runs multiple
 * callbacks at independent periods from a single loop: a {@link PriorityQueue} of callbacks
 * ordered by when they next come due, each rescheduled by its own period after it runs.
 *
 * <p>
 * Unlike a real {@code Notifier} per callback, this advances a purely virtual timeline instead of
 * depending on real wall-clock time or spawning any threads — {@link #runFor(Time)} runs entirely
 * on the calling thread. It does keep WPILib's simulated FPGA clock ({@code SimHooks}/
 * {@code RobotController}'s time source) in lockstep with that virtual timeline, since some vendor
 * SDKs (e.g. CTRE's Phoenix6 SimState) treat status signals as stale until the FPGA clock actually
 * advances. That makes this well suited to unit tests that need two (or more) callbacks to run at
 * different rates against the same object — e.g. stepping simulation physics on one period while
 * publishing telemetry on another — without the flakiness of real background-thread timing under a
 * simulated clock.
 * </p>
 */
public class PeriodicScheduler
{
  private final PriorityQueue<Callback> callbacks = new PriorityQueue<>();
  private long nowMicros = 0;

  public PeriodicScheduler()
  {
    RobotController.setTimeSource(() -> nowMicros);
  }

  /**
   * Register a callback to run at the given period, first coming due one period from now.
   *
   * @param callback Callback to run each time it comes due.
   * @param period   Period to run the callback at.
   */
  public void addPeriodic(Runnable callback, Time period)
  {
    long periodMicros = Math.round(period.in(Microseconds));
    callbacks.add(new Callback(callback, periodMicros, nowMicros + periodMicros));
  }

  /**
   * Advance the virtual timeline by the given duration, running every registered callback exactly
   * as many times as its period divides into the elapsed duration, in temporal order — exactly
   * like {@link edu.wpi.first.wpilibj.TimedRobot}'s own callback scheduler.
   *
   * @param duration Duration to advance the virtual timeline by.
   */
  public void runFor(Time duration)
  {
    long endMicros = nowMicros + Math.round(duration.in(Microseconds));
    Callback next;
    while ((next = callbacks.peek()) != null && next.expirationMicros <= endMicros)
    {
      callbacks.poll();
      advanceTo(next.expirationMicros);
      next.run.run();
      next.expirationMicros += next.periodMicros;
      callbacks.add(next);
    }
    advanceTo(endMicros);
  }

  /**
   * Advance the virtual timeline (and WPILib's simulated FPGA clock) to the given point in time.
   *
   * @param targetMicros Point in time, in microseconds, to advance to.
   */
  private void advanceTo(long targetMicros)
  {
    long dtMicros = targetMicros - nowMicros;
    nowMicros = targetMicros;
    if (dtMicros > 0)
    {
      SimHooks.stepTiming(Microseconds.of(dtMicros).in(Seconds));
    }
  }

  /**
   * A registered periodic callback, ordered by when it next comes due.
   */
  private static final class Callback implements Comparable<Callback>
  {
    private final Runnable run;
    private final long periodMicros;
    private long expirationMicros;

    private Callback(Runnable run, long periodMicros, long expirationMicros)
    {
      this.run = run;
      this.periodMicros = periodMicros;
      this.expirationMicros = expirationMicros;
    }

    @Override
    public int compareTo(Callback other)
    {
      return Long.compare(expirationMicros, other.expirationMicros);
    }
  }
}
