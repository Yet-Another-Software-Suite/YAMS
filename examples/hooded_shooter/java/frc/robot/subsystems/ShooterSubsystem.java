// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;

import static org.wpilib.units.Units.DegreesPerSecond;

import java.util.function.Supplier;

import org.wpilib.math.controller.PIDController;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.command2.Command;

public class ShooterSubsystem {
    // Holds and manages turret, hood and flywheel

    private final TurretSubsystem turret = new TurretSubsystem();
    private final HoodSubsystem hood = new HoodSubsystem();
    private final FlywheelSubsystem flywheel = new FlywheelSubsystem();

    private Supplier<AngularVelocity> flywheelVelocitySupplier = () -> DegreesPerSecond.of(0);

    private PIDController hoodPIDController = new PIDController(10, 0, 0);
    private PIDController turretPIDController = new PIDController(12, 0, 0);

    public ShooterSubsystem(VisionSubsystem vision) {
        // Will make both hood and turret repeatedly try to correct themself to point at
        // the closest tag, can be reused and filtered with tag ids to get it to point
        // on the hub

        hood.setDefaultCommand(hood.setDutyCycle(() -> {
            var results = vision.getClosestTag();
            if (results.isPresent()) {
                return hoodPIDController.calculate(results.get().skew, 0);
            }
            return 0.0;
        }));

        turret.setDefaultCommand(turret.setDutyCycle(() -> {
            var results = vision.getClosestTag();
            if (results.isPresent()) {
                return turretPIDController.calculate(results.get().yaw, 0);
            }
            return 0.0;
        }));
    }

    public Command aimAt(Angle hoodAngle, Angle turretAngle) {
        return hood.setAngle(hoodAngle).alongWith(turret.setAngle(turretAngle));
    }

    public Command runShooter() {
        if (flywheelVelocitySupplier == null) {
            DriverStationErrors.reportWarning("Shooter velocity set to null, not running shooter", true);
            return flywheel.idle(); // Do nothing until a valid request is set
        }

        return flywheel.setVelocity(flywheelVelocitySupplier);
    }

    public Command stopShooter() {
        return flywheel.setVelocity(DegreesPerSecond.of(0));
    }

    public Command runShooter(AngularVelocity velocity) {
        if (velocity == null) {
            DriverStationErrors.reportWarning("Shooter velocity set to null, defaulting to 0", true);
            velocity = DegreesPerSecond.of(0);
        }

        return flywheel.setVelocity(velocity);
    }

    public void setVelocitySupplier(Supplier<AngularVelocity> velocitySupplier) {
        // You would have some mathematical model for the speed of the shooter based on
        // other arguments on the field (likely distance from the hub) and use this
        // method in order to set the right shooter speed relative to that
        flywheelVelocitySupplier = velocitySupplier;
    }

}
