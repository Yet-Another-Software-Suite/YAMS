package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final TurretSubsystem turret = new TurretSubsystem();
    private final HoodSubsystem hood = new HoodSubsystem();
    private final FlywheelSubsystem flywheel = new FlywheelSubsystem();

    private Optional<Supplier<AngularVelocity>> flywheelVelocitySupplier = Optional.empty();

    public ShooterSubsystem() {
    }

    public Command aimAt(Angle hoodAngle, Angle turretAngle) {
        return hood.setAngle(hoodAngle).alongWith(turret.setAngle(turretAngle));
    }

    public Command aimAt(Pose3d poseFromShooterOrigin) {
        return hood.setAngle(poseFromShooterOrigin.getRotation().getMeasureY())
                .alongWith(turret.setAngle(poseFromShooterOrigin.getRotation().getMeasureZ()));
    }

    public Command runShooter() {
        if (flywheelVelocitySupplier.isPresent()) {
            return flywheel.setVelocity(flywheelVelocitySupplier.get());
        } else {
            return flywheel.setVelocity(DegreesPerSecond.of(0));
        }
    }
}
