// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.util;

import static org.wpilib.units.Units.Seconds;

import org.wpilib.units.measure.Time;
import org.wpilib.system.Timer;

public class Stopwatch {
    private double startTimeInSeconds = Double.POSITIVE_INFINITY;

    public void start() {
        startTimeInSeconds = Timer.getTimestamp();
    }

    public void startIfNotRunning() {
        if (Double.isInfinite(startTimeInSeconds)) {
            start();
        }
    }

    public void reset() {
        startTimeInSeconds = Double.POSITIVE_INFINITY;
    }

    public double elapsedSeconds() {
        if (Double.isInfinite(startTimeInSeconds)) {
            return 0.0;
        }
        return Timer.getTimestamp() - startTimeInSeconds;
    }

    public Time elapsedTime() {
        return Seconds.of(elapsedSeconds());
    }
}
