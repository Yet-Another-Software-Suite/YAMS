// Copyright (c) 2025-2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.helpers;

import edu.wpi.first.hal.simulation.NotifierDataJNI;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.concurrent.atomic.AtomicLong;

/**
 * Integration test helper class that will run the command scheduler. Use of
 * this class requires that your test class be extended from
 * {@link TestWithScheduler}.
 */
public final class SchedulerPumpHelper {
	private static int defaultHeartbeatInMs = 20;
	// Upper bound on how many 1ms polls to spend waiting for HAL notifiers (e.g. CTRE/REV CAN
	// simulation refresh threads) to catch up with a simulated time step. stepTimingAsync()
	// wakes those threads without waiting for them, so without this settle loop the very next
	// cycleRunnable can observe stale duty cycle/velocity data.
	private static final int MAX_NOTIFIER_SETTLE_ATTEMPTS = 50;

	/**
	 * Static class. Do not initialize.
	 */
	private SchedulerPumpHelper() {
	}

	/**
	 * Change the default heartbeat for the scheduler pump.
	 *
	 * @param defaultHeartbeatInMs Heartbeat in milliseconds
	 */
	public static void setDefaultHeartbeat(int defaultHeartbeatInMs) {
		SchedulerPumpHelper.defaultHeartbeatInMs = defaultHeartbeatInMs;
	}

	/**
	 * Helper to figure out what heartbeat to use.
	 *
	 * @param optionalHeartbeatInMs Optional heartbeat in array form to simulate
	 *                              optional parameters
	 * @return The heartbeat to use
	 */
	private static int getHeartbeatToUse(int[] optionalHeartbeatInMs) {
		if (optionalHeartbeatInMs.length > 1) {
			throw new IllegalArgumentException("There can be only one optional heartbeat parameter.");
		}
		return optionalHeartbeatInMs.length > 0 ? optionalHeartbeatInMs[0] : defaultHeartbeatInMs;
	}

	/**
	 * Wait for any HAL notifiers (e.g. CTRE/REV CAN simulation refresh threads) whose alarm has
	 * already elapsed as of {@code nowMicros} to run and rearm for their next period.
	 * {@link SimHooks#stepTimingAsync(double)} advances the simulated clock and wakes those
	 * notifier threads without waiting for them to finish, so polling
	 * {@link NotifierDataJNI#getNextTimeout()} until it moves past {@code nowMicros} (or there
	 * are no notifiers left to wait on) confirms they've actually drained before we let the
	 * caller observe the new simulated state.
	 *
	 * @param nowMicros Simulated time, in microseconds, that the notifiers must have caught up to.
	 * @throws InterruptedException Thrown if sleeping is interrupted.
	 */
	private static void awaitNotifierSettle(long nowMicros) throws InterruptedException {
    Thread.sleep(1);
		for (int attempt = 0; attempt < MAX_NOTIFIER_SETTLE_ATTEMPTS; attempt++) {
			if (NotifierDataJNI.getNumNotifiers() == 0 || NotifierDataJNI.getNextTimeout() > nowMicros) {
				return;
			}
			Thread.sleep(1);
		}
	}

	/**
	 * Run the command scheduler every heartbeatInMs for a durationInMs amount of
	 * time. Calls will be serialized as the Scheduler is not threadsafe, so beware
	 * of deadlocks. As of this writing, parallel testing is NOT the default mode
	 * for JUnit. So if you have not decorated your tests to run in parallel, you
	 * are fine.
	 * @param cycleRunnable 		Runnable that runs at the end of each cycle.
	 * @param durationInMs          Duration to run in milliseconds
	 * @param optionalHeartbeatInMs Optional pump time in milliseconds. If omitted,
	 *                              20ms default unless changed.
	 * @throws InterruptedException Thrown if sleeping interrupted
	 */
	public static synchronized void runForDuration(Runnable cycleRunnable, Time durationInMs, int... optionalHeartbeatInMs)
			throws InterruptedException {
		int heartbeatToUseInMs = getHeartbeatToUse(optionalHeartbeatInMs);
		long start = System.currentTimeMillis();
		AtomicLong time = new AtomicLong();
		RobotController.setTimeSource(time::get);

		for (int i = 0; i < durationInMs.in(Units.Milliseconds)/heartbeatToUseInMs; i++) {
			long nowMicros = (long) i * 20 * 1_000; // 20,000 microseconds = 20ms time step
			time.set(nowMicros);
			CommandScheduler.getInstance().run();
			SimHooks.stepTimingAsync(heartbeatToUseInMs);
			awaitNotifierSettle(nowMicros + heartbeatToUseInMs * 1_000L);
			if(cycleRunnable != null)
				cycleRunnable.run();
		}
//		while (System.currentTimeMillis() < (start + durationInMs.in(Units.Milliseconds))) {
//			CommandScheduler.getInstance().run();
//			Thread.sleep(heartbeatToUseInMs);
////			SimHooks.stepTiming(heartbeatToUseInMs);
//		}
	}
}
