// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.commands;

import static first.robot.Constants.FeedConstants.*;
import static first.robot.Constants.IntakeConstants.kRollerIntake;
import static first.robot.Constants.IntakeConstants.kRollerOuttake;

import first.robot.Constants.HoodConstants;
import first.robot.Constants.ShooterConstants;
import first.robot.mechanisms.Agitator;
import first.robot.mechanisms.Hood;
import first.robot.mechanisms.Indexer;
import first.robot.mechanisms.IntakeRoller;
import first.robot.mechanisms.Kicker;
import first.robot.mechanisms.Shooter;
import first.robot.mechanisms.Swerve;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

/**
 * Commands that use several of the fuel handling mechanisms. Each requires every mechanism it runs
 * and sets them all every loop until canceled; the mechanisms' default commands stop them again
 * afterwards.
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

    /**
     * Spin the shooter up and feed fuel into it until canceled. The kicker and agitator run the whole
     * time; the indexer holds fuel back until the shooter is within tolerance of its target, then
     * feeds. Replaces the original's {@code ShootKickIndexCommand} and {@code PassCommand}.
     *
     * @param targetSpeed          Shooter speed, read every loop.
     * @param hoodAngle            Hood angle while shooting.
     * @param readyDebounceSeconds How long the shooter may leave its tolerance before the indexer
     *                             stops feeding.
     * @param name                 Command name.
     */
    private Command shoot(Supplier<AngularVelocity> targetSpeed, Angle hoodAngle, double readyDebounceSeconds, String name) {
        return Command.requiring(shooter, kicker, indexer, agitator, hood).executing(coroutine -> {
            final Debouncer readyDebouncer = new Debouncer(readyDebounceSeconds, DebounceType.FALLING);
            while (true) {
                final AngularVelocity target = targetSpeed.get();
                shooter.setVelocitySetpoint(target);
                hood.setAngleSetpoint(hoodAngle);
                kicker.setVelocitySetpoint(kKickerSpeed);
                agitator.setDutyCycleSetpoint(kAgitatorFeed);
                if (readyDebouncer.calculate(shooter.isNear(target))) {
                    indexer.setVelocitySetpoint(kIndexerSpeed);
                } else {
                    indexer.setDutyCycleSetpoint(kIndexerHold);
                }
                coroutine.yield();
            }
        }).named(name);
    }

    /** Shoot at the hub at a fixed speed. */
    public Command shootAt(AngularVelocity speed) {
        return shoot(() -> speed, HoodConstants.kShoot, kShotDebounceSeconds, "Shoot at " + speed);
    }

    /** Shoot at the hub at the speed for the robot's distance from it, from {@link ShotMap}. */
    public Command shootFromDistance() {
        return shoot(() -> ShotMap.speedFor(swerve.distanceToHub()), HoodConstants.kShoot, kShotDebounceSeconds,
            "Shoot from Distance");
    }

    /** Pass fuel across the field with the hood raised for a lob. */
    public Command pass() {
        return shoot(() -> ShooterConstants.kPass, HoodConstants.kPass, kPassDebounceSeconds, "Pass");
    }

    /** Run the intake rollers in, with the agitator stirring and the indexer holding fuel back. */
    public Command intake() {
        return Command.requiring(intakeRoller, agitator, indexer).executing(coroutine -> {
            while (true) {
                intakeRoller.setDutyCycleSetpoint(kRollerIntake);
                agitator.setDutyCycleSetpoint(kAgitatorFeed);
                indexer.setDutyCycleSetpoint(kIndexerHold);
                coroutine.yield();
            }
        }).named("Intake");
    }

    /**
     * Autonomous "IntakeStart" event: run the intake rollers in with the agitator stirring, until
     * "IntakeStop" or a shot takes over the agitator.
     */
    public Command startIntake() {
        return Command.requiring(intakeRoller, agitator).executing(coroutine -> {
            while (true) {
                intakeRoller.setDutyCycleSetpoint(kRollerIntake);
                agitator.setDutyCycleSetpoint(kAgitatorFeed);
                coroutine.yield();
            }
        }).named("Intake Start");
    }

    /**
     * Autonomous "IntakeStop" event: keep the agitator stirring. Taking over the agitator cancels
     * {@link #startIntake()}, so the intake rollers go back to their default command and stop.
     */
    public Command stopIntake() {
        return agitator.runAt(kAgitatorFeed);
    }

    /** Spit fuel back out of the intake, with the agitator reversed. */
    public Command outtake() {
        return Command.requiring(intakeRoller, agitator).executing(coroutine -> {
            while (true) {
                intakeRoller.setDutyCycleSetpoint(kRollerOuttake);
                agitator.setDutyCycleSetpoint(kAgitatorReverse);
                coroutine.yield();
            }
        }).named("Outtake");
    }

    /** Run the feed path backwards to clear a jam. */
    public Command unjam() {
        return Command.requiring(kicker, indexer, agitator).executing(coroutine -> {
            while (true) {
                kicker.setDutyCycleSetpoint(kUnjamKicker);
                indexer.setDutyCycleSetpoint(kUnjamIndexer);
                agitator.setDutyCycleSetpoint(kUnjamAgitator);
                coroutine.yield();
            }
        }).named("Unjam");
    }
}
