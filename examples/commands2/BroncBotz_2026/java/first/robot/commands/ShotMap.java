// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.commands;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;

import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.units.measure.AngularVelocity;

/** Shooter speed by distance to the hub, interpolated between recorded shots. */
public final class ShotMap {
    // Distance to the hub center in inches, shooter RPM. Tune here.
    private static final double[][] kShots = {
        {93, 2200},
        {97, 2200},
        {105, 2400},
        {115, 2400},
        {116, 2400},
        {132, 2500},
        {133, 2500},
        {140, 2600},
        {153, 2600},
        {162, 2700},
        {165, 2800},
        {174, 2800},
        {191, 3050},
        {196, 3000},
        {202, 3150},
        {207, 3100},
        {208, 3200},
    };

    private static final InterpolatingDoubleTreeMap kRPMByMeters = new InterpolatingDoubleTreeMap();

    static {
        for (double[] shot : kShots) {
            kRPMByMeters.put(Inches.of(shot[0]).in(Meters), shot[1]);
        }
    }

    private ShotMap() {
    }

    /** Shooter speed for a distance to the hub in meters. */
    public static AngularVelocity speedFor(double distanceMeters) {
        return RPM.of(kRPMByMeters.get(distanceMeters));
    }
}
