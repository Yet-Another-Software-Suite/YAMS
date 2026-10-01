// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.commands;

import first.robot.Constants.FeedConstants;
import first.robot.subsystems.Agitator;
import first.robot.subsystems.Hood;
import first.robot.subsystems.Indexer;
import first.robot.subsystems.Kicker;
import first.robot.subsystems.Shooter;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

/**
 * Spin the shooter up and feed fuel into it until interrupted. The kicker and agitator run the whole
 * time; the indexer holds fuel back until the shooter is within tolerance of its target, then feeds.
 * The same command shoots at the hub and passes, with a different speed and hood angle.
 *
 * <p>Replaces the original's {@code ShootKickIndexCommand} and {@code PassCommand}.
 */
public class ShootCommand extends Command {
    private final Shooter shooter;
    private final Kicker kicker;
    private final Indexer indexer;
    private final Agitator agitator;
    private final Hood hood;
    private final Supplier<AngularVelocity> targetSpeed;
    private final Angle hoodAngle;
    private final double readyDebounceSeconds;
    private Debouncer readyDebouncer;

    /**
     * @param targetSpeed          Shooter speed, read every loop.
     * @param hoodAngle            Hood angle while shooting.
     * @param readyDebounceSeconds How long the shooter may leave its tolerance before the indexer
     *                             stops feeding.
     */
    public ShootCommand(Shooter shooter, Kicker kicker, Indexer indexer, Agitator agitator, Hood hood,
                        Supplier<AngularVelocity> targetSpeed, Angle hoodAngle, double readyDebounceSeconds) {
        this.shooter = shooter;
        this.kicker = kicker;
        this.indexer = indexer;
        this.agitator = agitator;
        this.hood = hood;
        this.targetSpeed = targetSpeed;
        this.hoodAngle = hoodAngle;
        this.readyDebounceSeconds = readyDebounceSeconds;
        addRequirements(shooter, kicker, indexer, agitator, hood);
    }

    @Override
    public void initialize() {
        readyDebouncer = new Debouncer(readyDebounceSeconds, DebounceType.FALLING);
    }

    @Override
    public void execute() {
        final AngularVelocity target = targetSpeed.get();
        shooter.setVelocitySetpoint(target);
        hood.setAngleSetpoint(hoodAngle);
        kicker.setVelocitySetpoint(FeedConstants.kKickerSpeed);
        agitator.setDutyCycleSetpoint(FeedConstants.kAgitatorFeed);
        if (readyDebouncer.calculate(shooter.isNear(target))) {
            indexer.setVelocitySetpoint(FeedConstants.kIndexerSpeed);
        } else {
            indexer.setDutyCycleSetpoint(FeedConstants.kIndexerHold);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // The default commands take over afterwards: the rollers stop and the hood goes down.
        shooter.setDutyCycleSetpoint(0);
        kicker.setDutyCycleSetpoint(0);
        indexer.setDutyCycleSetpoint(0);
        agitator.setDutyCycleSetpoint(0);
    }
}
