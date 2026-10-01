// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot;

import java.util.Optional;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.NiDsXboxController;
import org.wpilib.driverstation.RobotState;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringPublisher;
import org.wpilib.util.Color;

/**
 * Tracks whether this alliance's hub is active during the 2026 match shifts, publishes it with the
 * time left in the current shift, and rumbles the driver controller as a shift ends.
 *
 * <p>During teleop the hubs take turns: the transition shift and endgame are always active, and
 * shifts 1-4 alternate, starting with the alliance that did not win autonomous. The game data's
 * first character names the alliance whose hub goes inactive first.
 */
public class HubTracker {
    // Rumble once less than this fraction of the shift is left.
    private static final double kRumbleFraction = 0.15;
    // The hub color blinks for the last seconds of a shift.
    private static final double kBlinkSeconds = 5;

    private final NiDsXboxController driver;
    private final BooleanPublisher hubActivePublisher;
    private final DoublePublisher timeLeftPublisher;
    private final StringPublisher hubColorPublisher;
    private boolean blink = false;

    /** The current shift: whether the hub is active, its end in match time, and its length. */
    private record Shift(boolean active, double endMatchTime, double length) {
    }

    public HubTracker(NiDsXboxController driver) {
        this.driver = driver;
        final NetworkTable table = NetworkTableInstance.getDefault().getTable("HubTracker");
        hubActivePublisher = table.getBooleanTopic("HubActive").publish();
        timeLeftPublisher = table.getDoubleTopic("TimeLeft").publish();
        hubColorPublisher = table.getStringTopic("HubColor").publish();
    }

    /** Whether this alliance's hub is active right now. */
    public static boolean isHubActive() {
        return currentShift().map(Shift::active).orElse(false);
    }

    /** Update the dashboard and the rumble. Call once per loop. */
    public void update() {
        blink = !blink;
        final Optional<Shift> shift = currentShift();
        hubActivePublisher.set(shift.map(Shift::active).orElse(false));
        if (shift.isEmpty()) {
            timeLeftPublisher.set(-1);
            hubColorPublisher.set(Color.RED.toHexString());
            setRumble(0);
            return;
        }

        // Match time counts down, so the time left is how far it still is above the shift's end.
        final double timeLeft = Math.max(MatchState.getMatchTime() - shift.get().endMatchTime(), 0);
        final double fractionLeft = timeLeft / shift.get().length();
        timeLeftPublisher.set(Math.floor(timeLeft));

        final boolean showActive = timeLeft <= kBlinkSeconds ? blink : shift.get().active();
        hubColorPublisher.set((showActive ? Color.GREEN : Color.RED).toHexString());

        if (fractionLeft <= kRumbleFraction && RobotState.isFMSAttached()) {
            setRumble(kRumbleFraction * Math.pow(1.0 - fractionLeft, 2));
        } else {
            setRumble(0);
        }
    }

    private void setRumble(double strength) {
        driver.setRumble(RumbleType.LEFT_RUMBLE, strength);
        driver.setRumble(RumbleType.RIGHT_RUMBLE, strength);
    }

    private static Optional<Shift> currentShift() {
        final Optional<Alliance> alliance = MatchState.getAlliance();
        if (alliance.isEmpty()) {
            return Optional.empty();
        }
        final double matchTime = MatchState.getMatchTime();
        if (RobotState.isAutonomousEnabled()) {
            // Always active in autonomous, which is 20 seconds long.
            return Optional.of(new Shift(true, 0, 20));
        }
        if (!RobotState.isTeleopEnabled()) {
            return Optional.empty();
        }
        if (matchTime <= 30) {
            // Endgame, always active.
            return Optional.of(new Shift(true, 0, 30));
        }
        if (matchTime > 130) {
            // Transition shift, always active.
            return Optional.of(new Shift(true, 130, 10));
        }

        final String gameData = MatchState.getGameData().orElse("");
        // Without game data, assume the hub is active; it is likely early in teleop.
        final boolean redInactiveFirst;
        if (gameData.startsWith("R")) {
            redInactiveFirst = true;
        } else if (gameData.startsWith("B")) {
            redInactiveFirst = false;
        } else {
            return Optional.of(new Shift(true, shiftEnd(matchTime), 25));
        }
        final boolean shiftOneActive = alliance.get() == Alliance.RED ? !redInactiveFirst : redInactiveFirst;
        // Shifts 1 and 3 match shift one; shifts 2 and 4 are the opposite.
        final boolean oddShift = matchTime > 105 || (matchTime > 55 && matchTime <= 80);
        return Optional.of(new Shift(oddShift == shiftOneActive, shiftEnd(matchTime), 25));
    }

    /** Match time at which the 25 second shift containing the given match time ends. */
    private static double shiftEnd(double matchTime) {
        if (matchTime > 105) {
            return 105;
        } else if (matchTime > 80) {
            return 80;
        } else if (matchTime > 55) {
            return 55;
        }
        return 30;
    }
}
