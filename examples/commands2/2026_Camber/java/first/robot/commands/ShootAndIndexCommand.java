// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.commands;

import static org.wpilib.units.Units.Feet;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Second;

import first.robot.subsystems.IndexerSubsystem;
import first.robot.subsystems.ShooterSubsystem;
import first.robot.subsystems.SwerveSubsystem;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;


public class ShootAndIndexCommand extends Command {

    private final IndexerSubsystem indexerSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    private final Supplier<AngularVelocity> goal;
    private final List<Data> shots = List.of(
            // TUNE HERE
            new Data(Inches.of(115), RPM.of(3250), Second.of(0.64)),
            new Data(Inches.of(135), RPM.of(3500), Second.of(0.64)),
            new Data(Inches.of(166), RPM.of(4000), Second.of(0.64)),
            new Data(Inches.of(92), RPM.of(3200), Second.of(0.64)),
            new Data(Meters.of(8.41326147155525), RPM.of(3450), Second.of(0.64)),
            new Data(Inches.of(118.25), RPM.of(3400), Second.of(.64)),
            new Data(Inches.of(74), RPM.of(2900), Second.of(.64))
    );
    @SuppressWarnings("unchecked")
    private final InterpolatingDoubleTreeMap shotMap = InterpolatingDoubleTreeMap.ofEntries(shots.stream()
            .map(Data::toEntry)
            .toArray(Map.Entry[]::new));

    public ShootAndIndexCommand(IndexerSubsystem indexerSubsystem,
                                ShooterSubsystem shooterSubsystem, Supplier<AngularVelocity> goalRPM) {
        this.indexerSubsystem = indexerSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.goal = goalRPM;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.indexerSubsystem, this.shooterSubsystem);
    }

    public ShootAndIndexCommand(IndexerSubsystem indexerSubsystem,
                                ShooterSubsystem shooterSubsystem, AngularVelocity goalRPM) {
        this(indexerSubsystem, shooterSubsystem, () -> goalRPM);
    }

    public ShootAndIndexCommand(IndexerSubsystem indexerSubsystem,
                                ShooterSubsystem shooterSubsystem, SwerveSubsystem swerveSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.goal = () -> {
          return RPM.of(shotMap.get((swerveSubsystem.distanceFromHubMeters().minus(Feet.of(1)).in(Meters))));
        };
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.indexerSubsystem, this.shooterSubsystem);
    }

    @Override
    public void initialize() {
        indexerSubsystem.setDutycycle(0);
    }

    @Override
    public void execute() {
        shooterSubsystem.setVelocity(goal.get());
        Telemetry.log("Shooter/Goal", goal.get().in(RPM));
        Telemetry.log("Shooter/RPM", shooterSubsystem.getVelocity().in(RPM));
        if ((shooterSubsystem.getVelocity().isNear(goal.get(), RPM.of(50)))) {
            indexerSubsystem.setDutycycle(-.60);


        } else {
            indexerSubsystem.setDutycycle(0.3);
        }
    }

    record Data(Distance distance, AngularVelocity velocity, Time timeOfFlight) {

        public Map.Entry<Double, Double> toEntry() {
            return Map.entry(distance.in(Meters), velocity.in(RPM));
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.setDutycycle(0);
        shooterSubsystem.setDutycycle(0);
    }
}
