// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.commands;

import static first.robot.Constants.FeedConstants.*;
import static first.robot.Constants.IntakeConstants.kRollerIntake;
import static first.robot.Constants.IntakeConstants.kRollerOuttake;

import first.robot.Constants.HoodConstants;
import first.robot.Constants.ShooterConstants;
import first.robot.subsystems.Agitator;
import first.robot.subsystems.Hood;
import first.robot.subsystems.Indexer;
import first.robot.subsystems.IntakeRoller;
import first.robot.subsystems.Kicker;
import first.robot.subsystems.Shooter;
import first.robot.subsystems.Swerve;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.units.measure.AngularVelocity;

/**
 * Commands that use several of the fuel handling subsystems. Each runs until interrupted, and the
 * subsystems' default commands stop them again afterwards.
 */
public class SuperstructureCommands {
    // How long the shooter may dip out of tolerance before the indexer stops feeding.
    private static final double kShotDebounceSeconds = 0.1;
    private static final double kPassDebounceSeconds = 0.3;

    private final Swerve swerve;
    private final Shooter shooter;
    private final Kicker kicker;
    private final Indexer indexer;
    private final Agitator agitator;
    private final Hood hood;
    private final IntakeRoller intakeRoller;

    public SuperstructureCommands(Swerve swerve, Shooter shooter, Kicker kicker, Indexer indexer, Agitator agitator,
                                  Hood hood, IntakeRoller intakeRoller) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.kicker = kicker;
        this.indexer = indexer;
        this.agitator = agitator;
        this.hood = hood;
        this.intakeRoller = intakeRoller;
    }

    /** Shoot at the hub at a fixed speed. */
    public Command shootAt(AngularVelocity speed) {
        return new ShootCommand(shooter, kicker, indexer, agitator, hood, () -> speed, HoodConstants.kShoot, kShotDebounceSeconds)
            .withName("Shoot at " + speed);
    }

    /** Shoot at the hub at the speed for the robot's distance from it, from {@link ShotMap}. */
    public Command shootFromDistance() {
        return new ShootCommand(shooter, kicker, indexer, agitator, hood, () -> ShotMap.speedFor(swerve.distanceToHub()),
            HoodConstants.kShoot, kShotDebounceSeconds)
            .withName("Shoot from Distance");
    }

    /** Pass fuel across the field with the hood raised for a lob. */
    public Command pass() {
        return new ShootCommand(shooter, kicker, indexer, agitator, hood, () -> ShooterConstants.kPass, HoodConstants.kPass, kPassDebounceSeconds)
            .withName("Pass");
    }

    /** Run the intake rollers in, with the agitator stirring and the indexer holding fuel back. */
    public Command intake() {
        return Commands.parallel(
            intakeRoller.runAt(kRollerIntake),
            agitator.runAt(kAgitatorFeed),
            indexer.runAt(kIndexerHold))
            .withName("Intake");
    }

    /** Spit fuel back out of the intake, with the agitator reversed. */
    public Command outtake() {
        return Commands.parallel(
            intakeRoller.runAt(kRollerOuttake),
            agitator.runAt(kAgitatorReverse))
            .withName("Outtake");
    }

    /** Run the feed path backwards to clear a jam. */
    public Command unjam() {
        return Commands.parallel(
            kicker.runAt(kUnjamKicker),
            indexer.runAt(kUnjamIndexer),
            agitator.runAt(kUnjamAgitator))
            .withName("Unjam");
    }
}
