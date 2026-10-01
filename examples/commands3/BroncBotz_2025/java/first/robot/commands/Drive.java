// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.commands;

import first.robot.Constants.OperatorConstants;
import first.robot.mechanisms.Swerve;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.driverstation.NiDsXboxController;

/** Drive commands for the swerve drivetrain. */
public final class Drive {
    private Drive() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Teleop drive command. The sticks drive relative to the robot, at a slower translation scale
     * while the left bumper is held. X turns alliance relative control on (flipping the sticks for
     * the red alliance) and Y turns it off; it starts off, as in the original.
     *
     * @param swerve     The drivetrain.
     * @param controller The driver's controller.
     * @return A command that drives from the controller until canceled.
     */
    public static Command teleop(Swerve swerve, CommandNiDsXboxController controller) {
        final NiDsXboxController hid = controller.getNiDsXboxController();
        return swerve.run(coroutine -> {
            boolean allianceRelative = false;
            // Drop button presses from before this command started.
            hid.getXButtonPressed();
            hid.getYButtonPressed();
            while (true) {
                if (hid.getXButtonPressed()) {
                    allianceRelative = true;
                }
                if (hid.getYButtonPressed()) {
                    allianceRelative = false;
                }
                final double translationScale = hid.getLeftBumperButton()
                    ? OperatorConstants.kSlowTranslationScale
                    : OperatorConstants.kTranslationScale;
                swerve.setDriveInput(-hid.getLeftY(), -hid.getLeftX(), -hid.getRightX(), translationScale, allianceRelative);
                swerve.driveFromInput();
                coroutine.yield();
            }
        }).named("Teleop Drive");
    }
}
