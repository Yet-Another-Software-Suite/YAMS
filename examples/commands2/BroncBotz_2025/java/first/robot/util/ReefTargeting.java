// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.util;

import first.robot.Constants.AutoScoring;
import first.robot.util.FieldConstants.Reef;
import first.robot.util.FieldConstants.ReefHeight;
import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Pose2d;

/**
 * Which reef branch, level and side the operator has picked, and where the robot has to stand to
 * score there. This was the original's {@code TargetingSystem}; the commands that change the target
 * now live with the bindings.
 */
public class ReefTargeting {
    /** Branches in the order of {@link Reef#branchPositions}. */
    public enum Branch {
        A, B, K, L, I, J, G, H, E, F, C, D
    }

    public enum Level {
        L1, L2, L3, L4
    }

    /** Which branch of the targeted reef face to score on. */
    public enum Side {
        CLOSEST, RIGHT, LEFT
    }

    private Optional<Branch> branch = Optional.empty();
    // The original started without a level, which made the scoring command wait out its timeout.
    private Level level = Level.L4;
    private Side side = Side.CLOSEST;

    public void setTarget(Branch branch, Level level) {
        this.branch = Optional.of(branch);
        this.level = level;
    }

    public void setLevel(Level level) {
        this.level = level;
    }

    public void setSide(Side side) {
        this.side = side;
    }

    public Level getLevel() {
        return level;
    }

    public Optional<Branch> getBranch() {
        return branch;
    }

    /** Target the branch closest to the robot. */
    public void targetClosestBranch(Pose2d robotPose) {
        // Recomputed every time so the alliance is always current; the original cached it the first
        // time it was used.
        final List<Pose2d> branchPoses = Reef.branchPositions.stream()
            .map(position -> AllianceFlipUtil.apply(position.get(ReefHeight.L4).toPose2d()))
            .toList();
        branch = Optional.of(Branch.values()[branchPoses.indexOf(robotPose.nearest(branchPoses))]);
    }

    /** Where to stand to score coral on the targeted branch, if there is one. */
    public Optional<Pose2d> getCoralScoringPose() {
        return branch.map(target -> branchPose(sideOrdinal(target)).plus(AutoScoring.kCoralOffset));
    }

    /** Where to stand to pull algae off the targeted reef face, if there is one. */
    public Optional<Pose2d> getAlgaeScoringPose() {
        return branch.map(target -> branchPose(rightBranchOrdinal(target)).plus(AutoScoring.kAlgaeOffset));
    }

    private static Pose2d branchPose(int ordinal) {
        return AllianceFlipUtil.apply(Reef.branchPositions.get(ordinal).get(ReefHeight.L2).toPose2d());
    }

    private int sideOrdinal(Branch target) {
        return switch (side) {
            case CLOSEST -> target.ordinal();
            case RIGHT -> rightBranchOrdinal(target);
            case LEFT -> leftBranchOrdinal(target);
        };
    }

    private static boolean isRightBranch(Branch target) {
        return (target.ordinal() + 1) % 2 == 0;
    }

    private static int rightBranchOrdinal(Branch target) {
        return Math.clamp(target.ordinal() + (isRightBranch(target) ? 0 : 1), 0, 11);
    }

    private static int leftBranchOrdinal(Branch target) {
        return Math.clamp(target.ordinal() - (isRightBranch(target) ? 1 : 0), 0, 11);
    }
}
