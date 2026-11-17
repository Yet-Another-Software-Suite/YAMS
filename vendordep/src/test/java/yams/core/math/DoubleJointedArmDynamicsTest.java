// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.math;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Meters;

import org.junit.jupiter.api.Test;
import yams.core.mechanisms.config.DoubleJointedArmConfig.UpperAngleReference;

class DoubleJointedArmDynamicsTest {
  private static final double kLowerLength = 0.6;
  private static final double kUpperLength = 0.75;
  private static final double kLowerMass = 3;
  private static final double kUpperMass = 2;

  private static DoubleJointedArmDynamics dynamics(UpperAngleReference reference) {
    return new DoubleJointedArmDynamics(
        Meters.of(kLowerLength),
        kLowerMass * kLowerLength * kLowerLength / 3,
        Meters.of(kUpperLength),
        kUpperMass * kUpperLength * kUpperLength / 3,
        reference);
  }

  @Test
  void anElbowDrivenFromTheBaseNeverChangesKg() {
    DoubleJointedArmDynamics dynamics = dynamics(UpperAngleReference.HORIZONTAL);
    for (double lower = -80; lower <= 80; lower += 20) {
      for (double upper = -170; upper <= 170; upper += 20) {
        double scale0 = dynamics.getKgScale(0, Math.toRadians(lower), Math.toRadians(upper));
        double scale1 = dynamics.getKgScale(1, Math.toRadians(lower), Math.toRadians(upper));
        assertEquals(1, scale0, 1e-9, "shoulder at " + lower + ", " + upper);
        assertTrue(
            Double.isNaN(scale1) || Math.abs(scale1 - 1) < 1e-9,
            "elbow at " + lower + ", " + upper);
      }
    }
  }

  @Test
  void anElbowMotorOnTheLowerArmChangesTheShouldersKgWithTheUpperArm() {
    DoubleJointedArmDynamics dynamics = dynamics(UpperAngleReference.LOWER_ARM);
    double lowerOnly = kLowerMass * kLowerLength / 2 + kUpperMass * kLowerLength;
    double upperArm = kUpperMass * kUpperLength / 2;

    // Straight out is what kG is tuned at.
    assertEquals(1, dynamics.getKgScale(0, 0, 0), 1e-9);
    assertEquals(1, dynamics.getKgScale(1, 0, 0), 1e-9);
    // The lower arm horizontal and the upper arm straight up: the shoulder no longer holds the
    // upper arm's turning weight.
    assertEquals(lowerOnly / (lowerOnly + upperArm), dynamics.getKgScale(0, 0, Math.PI / 2), 1e-9);
    // The lower arm at 60 degrees and the upper arm horizontal: kG cos(60°) must hold the whole
    // upper arm's weight.
    double lower = Math.toRadians(60);
    double expected =
        (lowerOnly * Math.cos(lower) + upperArm) / ((lowerOnly + upperArm) * Math.cos(lower));
    assertEquals(expected, dynamics.getKgScale(0, lower, -lower), 1e-9);
    // The elbow's ArmFeedforward uses its joint angle, -60 degrees, but the upper arm is
    // horizontal: 1 / cos(-60°).
    assertEquals(2, dynamics.getKgScale(1, lower, -lower), 1e-9);
  }

  @Test
  void kGIsLeftAloneNearStraightUpOrDown() {
    DoubleJointedArmDynamics dynamics = dynamics(UpperAngleReference.LOWER_ARM);

    assertTrue(Double.isNaN(dynamics.getKgScale(0, Math.PI / 2, 0)));
    assertTrue(Double.isNaN(dynamics.getKgScale(1, 0, -Math.PI / 2)));
  }

  @Test
  void theJointAnglesConvertToAndFromHorizontal() {
    DoubleJointedArmDynamics fromLowerArm = dynamics(UpperAngleReference.LOWER_ARM);
    DoubleJointedArmDynamics fromHorizontal = dynamics(UpperAngleReference.HORIZONTAL);

    assertEquals(1.0, fromLowerArm.getUpperAngleFromHorizontal(0.4, 0.6), 1e-12);
    assertEquals(0.6, fromLowerArm.getUpperJointAngle(0.4, 1.0), 1e-12);
    assertEquals(0.6, fromHorizontal.getUpperAngleFromHorizontal(0.4, 0.6), 1e-12);
    assertEquals(1.0, fromHorizontal.getUpperJointAngle(0.4, 1.0), 1e-12);
  }
}
