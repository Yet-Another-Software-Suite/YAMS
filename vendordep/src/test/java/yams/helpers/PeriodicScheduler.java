// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.helpers;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
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
 * on the calling thread. Time is controlled via WPILib's own documented mechanism for this,
 * {@link SimHooks#pauseTiming()}/{@link SimHooks#stepTiming(double)}, rather than a raw custom
 * {@code RobotController} time source — the latter was found to leave some vendor simulation SDKs
 * (whose own retry/config logic depends on the FPGA clock behaving exactly as WPILib expects) in
 * an inconsistent state, silently dropping simulated hardware calls in a way that reproduced in CI
 * but not always locally.
 * </p>
 */
public class PeriodicScheduler implements AutoCloseable
{
  private final PriorityQueue<Callback> callbacks = new PriorityQueue<>();
  private long nowMicros = 0;

  public PeriodicScheduler()
  {
    // Other tests sharing this JVM (e.g. via SchedulerPumpHelper) install their own custom
    // RobotController time source and never restore it, which can leave the clock in a stale,
    // frozen state. Resuming (real-time) timing first forces a clean, consistent baseline before
    // this scheduler takes manual control of it, regardless of what an earlier test left behind.
    SimHooks.resumeTiming();
    SimHooks.pauseTiming();
  }

  /**
   * Resume normal (real-time) simulated timing. Call this once done, so this scheduler doesn't
   * leave WPILib's simulated clock paused for whatever runs next.
   */
  @Override
  public void close()
  {
    SimHooks.resumeTiming();
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
      // Some vendor simulation SDKs (e.g. CTRE's Phoenix6 SimState) refresh their status signals
      // from a real background thread rather than reacting to SimHooks synchronously. Since this
      // scheduler otherwise runs a whole virtual timeline in a tight loop with no real elapsed
      // time, that background thread may never actually get scheduled to run. Yielding a sliver of
      // real wall-clock time here gives it a chance to catch up.
      try
      {
        Thread.sleep(1);
      } catch (InterruptedException e)
      {
        Thread.currentThread().interrupt();
      }
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
