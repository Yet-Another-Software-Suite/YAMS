// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot;

import static org.wpilib.units.Units.Inches;

import java.util.Optional;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.Alliance;

public class Landmarks {
    public static Translation2d hubPosition() {
        final Optional<Alliance> alliance = MatchState.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.BLUE) {
            return new Translation2d(Inches.of(182.105), Inches.of(158.845));
        }
        return new Translation2d(Inches.of(469.115), Inches.of(158.845));
    }
}
