// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.util;

import static org.wpilib.units.Units.Millimeters;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.wpilib.units.measure.Distance;
import yams.core.mechanisms.config.SensorConfig;
import yams.core.motorcontrollers.simulation.Sensor;

/**
 * Stand-in for the LaserCAN time of flight sensors the original used to detect coral and algae.
 *
 * <p>Grapple has not published a LaserCAN library for WPILib 2027, so on a real robot this reports
 * no measurement, which the mechanisms treat as nothing loaded. Once a 2027 library exists, read the
 * LaserCAN in {@link #readHardwareMillimeters()}. In simulation it is a YAMS {@link Sensor}: the
 * reading follows the {@code loaded} supplier, and can also be overridden from the sim GUI.
 */
public class DistanceSensor {
    private static final String kField = "DistanceMm";
    // Reported while there is no valid measurement.
    private static final double kNoMeasurement = -1;

    private final Sensor sensor;

    /**
     * @param name           Sensor name, used for the simulated device.
     * @param loadedInSim    While true in simulation, read {@code loadedDistance}; otherwise read
     *                       {@code emptyDistance}.
     * @param loadedDistance Simulated reading with a game piece loaded.
     * @param emptyDistance  Simulated reading without a game piece.
     */
    public DistanceSensor(String name, BooleanSupplier loadedInSim, Distance loadedDistance,
                          Distance emptyDistance) {
        sensor = new SensorConfig(name)
            .withField(kField, this::readHardwareMillimeters, kNoMeasurement)
            .withSimulatedValue(kField, loadedInSim, loadedDistance.in(Millimeters))
            .withSimulatedValue(kField, () -> !loadedInSim.getAsBoolean(), emptyDistance.in(Millimeters))
            .getSensor();
    }

    private double readHardwareMillimeters() {
        // No 2027 LaserCAN library yet.
        return kNoMeasurement;
    }

    /** The latest valid distance, or empty without one. */
    public Optional<Distance> getDistance() {
        final double millimeters = sensor.getAsDouble(kField);
        return millimeters < 0 ? Optional.empty() : Optional.of(Millimeters.of(millimeters));
    }
}
