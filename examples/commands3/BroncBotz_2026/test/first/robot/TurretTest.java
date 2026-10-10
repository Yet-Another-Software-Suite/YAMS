// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;

import first.robot.Constants.TurretConstants;
import first.robot.mechanisms.Turret;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.units.measure.Angle;

class TurretTest {
    // One turret for every test: a second one would claim the same CAN IDs.
    private static Turret turret;

    @BeforeAll
    static void createTurret() {
        assertTrue(HAL.initialize());
        turret = new Turret();
    }

    /** The reading an ideal absolute encoder on a pinion gives with the turret at an angle. */
    private static Angle reading(Angle turretAngle, int pinionTeeth) {
        final double encoderRotations = turretAngle.in(Rotations) * TurretConstants.kRingTeeth / pinionTeeth;
        return Rotations.of(encoderRotations - Math.floor(encoderRotations));
    }

    private static boolean seedAt(Angle turretAngle) {
        return turret.seed(reading(turretAngle, TurretConstants.kEncoderATeeth), reading(turretAngle, TurretConstants.kEncoderBTeeth));
    }

    @Test
    void seedsTheMotorAcrossTheTravel() {
        for (double degrees = TurretConstants.kMin.in(Degrees); degrees <= TurretConstants.kMax.in(Degrees); degrees += 25) {
            assertTrue(seedAt(Degrees.of(degrees)), "Encoders disagreed at " + degrees + " degrees");
            assertEquals(degrees, turret.getAngle().in(Degrees), 0.5, "Wrong angle at " + degrees + " degrees");
        }
    }

    @Test
    void tellsFullTurnsApart() {
        // 170 and -190 degrees point the same way a full turn apart, which one encoder cannot tell apart.
        assertTrue(seedAt(Degrees.of(170)));
        assertEquals(170, turret.getAngle().in(Degrees), 0.5);
        assertTrue(seedAt(Degrees.of(-190)));
        assertEquals(-190, turret.getAngle().in(Degrees), 0.5);
    }

    @Test
    void leavesTheMotorAloneWhenTheEncodersDisagree() {
        assertTrue(seedAt(Degrees.of(45)));
        // Encoder B is halfway between two candidates, so no turret angle fits both readings.
        final Angle wrongB = reading(Degrees.of(90), TurretConstants.kEncoderBTeeth).plus(Rotations.of(0.5 / TurretConstants.kEncoderBTeeth));
        assertFalse(turret.seed(reading(Degrees.of(90), TurretConstants.kEncoderATeeth), wrongB));
        assertEquals(45, turret.getAngle().in(Degrees), 0.5);
    }

    @Test
    void unambiguousTravelCoversTheHardLimits() {
        final double uniqueDegrees = 360.0 * TurretConstants.kEncoderATeeth * TurretConstants.kEncoderBTeeth / TurretConstants.kRingTeeth;
        assertTrue(TurretConstants.kHardMax.in(Degrees) < uniqueDegrees / 2);
        assertTrue(TurretConstants.kHardMin.in(Degrees) > -uniqueDegrees / 2);
    }
}
