// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.mechanisms;

import static org.wpilib.units.Units.Millimeters;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Value;

import first.robot.Ports;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.discrete.PWM;
import org.wpilib.math.util.MathUtil;
import org.wpilib.system.Timer;
import org.wpilib.tunable.Tunables;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Time;

/**
 * Shooter hood driven by two linear servos. Servos have no YAMS mechanism, so this mechanism drives
 * them directly. WPILib 2027 removed the Servo class, so the pulse width is set on the PWM channels
 * using the same 1000 to 2000 microsecond range the original servo bounds used.
 */
public class Hood implements Mechanism {
    private static final Distance kServoLength = Millimeters.of(100);
    private static final LinearVelocity kMaxServoSpeed = Millimeters.of(20).per(Second);
    private static final double kMinPosition = 0.01;
    private static final double kMaxPosition = 0.77;
    private static final double kPositionTolerance = 0.01;

    // Servo pulse range: position 0.0 maps to the minimum pulse and 1.0 to the maximum pulse.
    private static final int kMinPulseMicroseconds = 1000;
    private static final int kMaxPulseMicroseconds = 2000;

    private final PWM leftServo;
    private final PWM rightServo;

    private double currentPosition = 0.5;
    private double targetPosition = 0.5;
    private Time lastUpdateTime = Seconds.of(0);

    public Hood() {
        leftServo = new PWM(Ports.kHoodLeftServo);
        rightServo = new PWM(Ports.kHoodRightServo);
        setPosition(currentPosition);

        // The servos report no feedback, so the current position is estimated from their speed.
        Tunables.publishDouble("Hood/Current Position", () -> currentPosition, value -> {});
        Tunables.publishDouble("Hood/Target Position", () -> targetPosition, this::setPosition);
    }

    /** Expects a position between 0.0 and 1.0 */
    public void setPosition(double position) {
        final double clampedPosition = Math.clamp(position, kMinPosition, kMaxPosition);
        final int pulseMicroseconds = (int) Math.round(
            kMinPulseMicroseconds + clampedPosition * (kMaxPulseMicroseconds - kMinPulseMicroseconds));
        leftServo.setPulseTimeMicroseconds(pulseMicroseconds);
        rightServo.setPulseTimeMicroseconds(pulseMicroseconds);
        targetPosition = clampedPosition;
    }

    /** Move to a position between 0.0 and 1.0, finishing once the estimated position reaches it. */
    public Command moveTo(double position) {
        return run(coroutine -> {
            setPosition(position);
            coroutine.waitUntil(this::isPositionWithinTolerance);
        }).named("Hood to " + position);
    }

    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(targetPosition, currentPosition, kPositionTolerance);
    }

    private void updateCurrentPosition() {
        final Time currentTime = Seconds.of(Timer.getTimestamp());
        final Time elapsedTime = currentTime.minus(lastUpdateTime);
        lastUpdateTime = currentTime;

        if (isPositionWithinTolerance()) {
            currentPosition = targetPosition;
            return;
        }

        final Distance maxDistanceTraveled = kMaxServoSpeed.times(elapsedTime);
        final double maxPercentageTraveled = maxDistanceTraveled.div(kServoLength).in(Value);
        currentPosition = targetPosition > currentPosition
            ? Math.min(targetPosition, currentPosition + maxPercentageTraveled)
            : Math.max(targetPosition, currentPosition - maxPercentageTraveled);
    }

    /** Called from {@code Robot.robotPeriodic()}, replacing the v2 subsystem periodic. */
    public void periodic() {
        updateCurrentPosition();
    }
}
