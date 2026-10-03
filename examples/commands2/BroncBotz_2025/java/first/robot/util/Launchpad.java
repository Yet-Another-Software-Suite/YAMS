// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.util;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGenericHID;
import org.wpilib.command2.button.Trigger;
import org.wpilib.networktables.IntegerArrayPublisher;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;

/**
 * Novation Launchpad Mini Mk3 operator board. {@code deploy/launchpad/launch.py} runs on the driver
 * station: it presents the 9x9 grid as three vJoy controllers and lights each pad with the colour
 * published to {@code launchpad/colors}.
 *
 * <p>Pads are addressed by (x, y), +x to the right and +y down, from the top left. While a pad is
 * held it shows the pressed colour, then returns to its own colour.
 */
public class Launchpad {
    public static final Color kCoralLevel = Color.PURPLE;
    public static final Color kAlgae = Color.GREEN;
    public static final Color kScoreCoral = Color.CYAN;
    public static final Color kHumanPlayer = Color.RED;
    public static final Color kPressed = Color.BLUE;
    public static final Color kNotSelected = Color.WHITE;
    public static final Color kSelected = Color.BLUE;
    public static final Color kCoralLoaded = Color.MEDIUM_PURPLE;
    public static final Color kAlgaeLoaded = Color.LIME_GREEN;
    public static final Color kUnloaded = Color.WHITE;

    private static final int kSize = 9;
    private static final int kButtonsPerController = 32;

    private final IntegerArrayPublisher colorPublisher =
        NetworkTableInstance.getDefault().getTable("launchpad").getIntegerArrayTopic("colors").publish();
    // One 0x00RRGGBB colour per pad (6 bits per channel), indexed like the vJoy buttons.
    private final long[] colors = new long[kSize * kSize + 1];
    private final long[] savedColors = new long[kSize * kSize + 1];
    private final Trigger[][] buttons = new Trigger[kSize][kSize];

    /**
     * @param vjoy1        Controller port of the vJoy device with the first 32 pads.
     * @param vjoy2        Controller port of the vJoy device with the next 32 pads.
     * @param vjoy3        Controller port of the vJoy device with the remaining pads.
     * @param pressedColor Colour shown while a pad is held.
     */
    public Launchpad(int vjoy1, int vjoy2, int vjoy3, Color pressedColor) {
        final CommandGenericHID[] vjoys = {
            new CommandGenericHID(vjoy1), new CommandGenericHID(vjoy2), new CommandGenericHID(vjoy3)
        };
        for (int i = 0; i < kSize * kSize; i++) {
            final int row = i / kSize;
            final int col = i % kSize;
            final int button = getButtonNum(col, row) - kButtonsPerController * (i / kButtonsPerController);
            // The top left pad has no button.
            if (button == 0) {
                continue;
            }
            setColor(col, row, Color.ORANGE_RED);
            buttons[row][col] = vjoys[i / kButtonsPerController].button(button);
            buttons[row][col].whileTrue(Commands.run(() -> showPressed(col, row, pressedColor)));
            buttons[row][col].onFalse(Commands.runOnce(() -> restoreColor(col, row)));
        }
    }

    /** The trigger for a pad. */
    public Trigger getButton(int x, int y) {
        checkCoordinates(x, y);
        return buttons[y][x];
    }

    /** Set a pad's own colour. */
    public void setColor(int x, int y, Color color) {
        checkCoordinates(x, y);
        final int button = getButtonNum(x, y);
        colors[button] = toPadColor(color);
        savedColors[button] = colors[button];
        publish();
    }

    /** Set a pad's own colour and bind a command to it. */
    public void bind(int x, int y, Color color, Command command) {
        setColor(x, y, color);
        getButton(x, y).whileTrue(command);
    }

    private void showPressed(int x, int y, Color color) {
        colors[getButtonNum(x, y)] = toPadColor(color);
        publish();
    }

    private void restoreColor(int x, int y) {
        final int button = getButtonNum(x, y);
        colors[button] = savedColors[button];
        publish();
    }

    private static int getButtonNum(int x, int y) {
        // The top row has 8 pads plus the logo, the others 9; vJoy buttons start at 1.
        return y * kSize + x + (y > 0 ? 1 : 0);
    }

    private static void checkCoordinates(int x, int y) {
        if (x < 0 || y < 0 || x >= kSize || y >= kSize) {
            throw new IllegalArgumentException("Launchpad coordinates must be within 0-8, got (" + x + ", " + y + ")");
        }
    }

    private static long toPadColor(Color color) {
        final Color8Bit color8Bit = new Color8Bit(color);
        // The Launchpad takes 0-63 per channel.
        return ((long) (color8Bit.red / 4) << 16) | ((long) (color8Bit.green / 4) << 8) | (color8Bit.blue / 4);
    }

    private void publish() {
        // launch.py reads two pad colours per long, low 32 bits first.
        final long[] packed = new long[(colors.length + 1) / 2];
        for (int i = 0; i < packed.length; i++) {
            long value = colors[i * 2] & 0xFFFFFFFFL;
            if (i * 2 + 1 < colors.length) {
                value |= (colors[i * 2 + 1] & 0xFFFFFFFFL) << 32;
            }
            packed[i] = value;
        }
        colorPublisher.set(packed);
    }
}
