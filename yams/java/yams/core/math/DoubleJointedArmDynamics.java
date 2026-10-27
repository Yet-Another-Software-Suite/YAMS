// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.math;

import static org.wpilib.units.Units.Meters;

import org.wpilib.units.measure.Distance;
import yams.core.mechanisms.config.DoubleJointedArmConfig.UpperAngleReference;

/**
 * Equations of motion of a double jointed arm, in its motor controllers' joint angles.
 *
 * <p>The joint angles are {@code q1}, the lower arm's angle from horizontal, and {@code q2}, the upper arm's angle from
 * horizontal or from the lower arm (see {@link UpperAngleReference}). The upper arm's angle from horizontal is
 * {@code θ2 = q2 + k q1}, with {@code k} 0 from horizontal and 1 from the lower arm. In angles from horizontal
 * {@code θ = (q1, θ2)} a two-link arm is
 *
 * <pre>
 *   M(θ) θ'' + C(θ, θ') = G(θ) + Q
 *
 *   M = [ J1 + m2 l1²              m2 l1 r2 cos(θ1 − θ2) ]
 *       [ m2 l1 r2 cos(θ1 − θ2)    J2                    ]
 *   C = [  m2 l1 r2 sin(θ1 − θ2) θ2'² ]
 *       [ −m2 l1 r2 sin(θ1 − θ2) θ1'² ]
 *   G = −[ (m1 r1 + m2 l1) g cos θ1 ]
 *        [ m2 r2 g cos θ2           ]
 * </pre>
 *
 * <p>With {@code θ = A q}, {@code A = [1 0; k 1]}, each motor's torque acts on its own joint angle, so in joint angles
 * the arm is {@code Aᵀ M A q'' = τ + Aᵀ (G − C)}. An elbow motor on the lower arm pushes back on the lower arm, which is
 * the {@code k} in the shoulder's row.
 *
 * <p>Not thread safe: it keeps working arrays, so it allocates nothing.
 *
 * <p>Each arm is a uniform rod, as {@link org.wpilib.simulation.SingleJointedArmSim} assumes: its moment of inertia
 * {@code J} about its own pivot gives its mass, {@code m = 3 J / l²}, with its center of mass halfway along it,
 * {@code r = l / 2}.
 */
public class DoubleJointedArmDynamics {
  /** Acceleration of gravity, in meters per second squared, as WPILib's arm simulation uses. */
  public static final double kGravity = 9.8;
  /** Below this |cos| of a joint's angle its ArmFeedforward gives almost nothing, so its kG can't be compensated. */
  private static final double kMinimumCosine = 0.1;

  /** Lower arm length, in meters. */
  private final double lowerLength;
  /** Upper arm length, in meters. */
  private final double upperLength;
  /** Lower arm moment of inertia about the shoulder, in kilogram square meters. */
  private final double lowerMOI;
  /** Upper arm moment of inertia about the elbow, in kilogram square meters. */
  private final double upperMOI;
  /** Lower arm mass times the distance from the shoulder to its center of mass, in kilogram meters. */
  private final double lowerMassMoment;
  /** Upper arm mass, in kilograms. */
  private final double upperMass;
  /** Distance from the elbow to the upper arm's center of mass, in meters. */
  private final double upperCenterOfMass;
  /** 0 when the upper arm's angle is from horizontal, 1 when it is from the lower arm. */
  private final double k;
  /** Gravity torques with the arm straight out horizontally, which kG is tuned at. */
  private final double[] straightGravity = new double[2];
  /** Gravity torques being worked out, kept so a kG scale allocates nothing. */
  private final double[] gravity         = new double[2];

  /**
   * Create the dynamics of a double jointed arm.
   *
   * @param lowerLength Lower arm length, from the shoulder to the elbow.
   * @param lowerMOI    Lower arm moment of inertia about the shoulder, in kilogram square meters.
   * @param upperLength Upper arm length.
   * @param upperMOI    Upper arm moment of inertia about the elbow, in kilogram square meters.
   * @param reference   What the upper arm's angle is measured from.
   * @throws IllegalArgumentException if an arm's length or moment of inertia is not above 0.
   */
  public DoubleJointedArmDynamics(Distance lowerLength, double lowerMOI, Distance upperLength, double upperMOI, UpperAngleReference reference) {
    this.lowerLength = lowerLength.in(Meters);
    this.upperLength = upperLength.in(Meters);
    if (!(this.lowerLength > 0 && this.upperLength > 0 && lowerMOI > 0 && upperMOI > 0)) {
      throw new IllegalArgumentException("Arm lengths and moments of inertia must be above 0");
    }
    this.lowerMOI = lowerMOI;
    this.upperMOI = upperMOI;
    // Uniform rods: m = 3 J / l², with the center of mass at l / 2.
    this.lowerMassMoment = 3 * lowerMOI / (this.lowerLength * this.lowerLength) * (this.lowerLength / 2);
    this.upperMass = 3 * upperMOI / (this.upperLength * this.upperLength);
    this.upperCenterOfMass = this.upperLength / 2;
    this.k = reference == UpperAngleReference.LOWER_ARM ? 1 : 0;
    getGravityTorques(0, 0, straightGravity);
  }

  /**
   * Get the upper arm's angle from horizontal.
   *
   * @param lowerAngle Lower arm joint angle, in radians.
   * @param upperAngle Upper arm joint angle, in radians.
   * @return Upper arm angle from horizontal, in radians.
   */
  public double getUpperAngleFromHorizontal(double lowerAngle, double upperAngle) {
    return upperAngle + k * lowerAngle;
  }

  /**
   * Get the upper arm's joint angle for an angle from horizontal.
   *
   * @param lowerAngle               Lower arm joint angle, in radians.
   * @param upperAngleFromHorizontal Upper arm angle from horizontal, in radians.
   * @return Upper arm joint angle, in radians.
   */
  public double getUpperJointAngle(double lowerAngle, double upperAngleFromHorizontal) {
    return upperAngleFromHorizontal - k * lowerAngle;
  }

  /**
   * Whether the upper arm's angle is from the lower arm.
   *
   * @return True from the lower arm, false from horizontal.
   */
  public boolean isUpperAngleFromLowerArm() {
    return k == 1;
  }

  /**
   * Torque gravity puts on each joint, in newton meters, positive raising the joint: {@code Aᵀ G}.
   *
   * @param lowerAngle Lower arm joint angle, in radians.
   * @param upperAngle Upper arm joint angle, in radians.
   * @param torques    Filled with the lower then upper joint's torque.
   */
  public void getGravityTorques(double lowerAngle, double upperAngle, double[] torques) {
    final double lowerGravity = -(lowerMassMoment + upperMass * lowerLength) * kGravity * Math.cos(lowerAngle);
    final double upperGravity = -upperMass * upperCenterOfMass * kGravity * Math.cos(getUpperAngleFromHorizontal(lowerAngle, upperAngle));
    torques[0] = lowerGravity + k * upperGravity;
    torques[1] = upperGravity;
  }

  /**
   * How much a joint's kG must be scaled from the kG for the arm held straight out horizontally, for its
   * {@link org.wpilib.math.controller.ArmFeedforward}, {@code kG cos q}, to hold the arm against gravity here.
   *
   * @param joint      0 for the lower joint, 1 for the upper joint.
   * @param lowerAngle Lower arm joint angle, in radians.
   * @param upperAngle Upper arm joint angle, in radians.
   * @return The scale, or NaN when the joint points too near straight up or down for its ArmFeedforward to matter.
   */
  public double getKgScale(int joint, double lowerAngle, double upperAngle) {
    final double cosine = Math.cos(joint == 0 ? lowerAngle : upperAngle);
    if (Math.abs(cosine) < kMinimumCosine) {
      return Double.NaN;
    }
    getGravityTorques(lowerAngle, upperAngle, gravity);
    return gravity[joint] / (straightGravity[joint] * cosine);
  }

  /**
   * Joint space mass matrix {@code Aᵀ M A}, which is symmetric.
   *
   * @param lowerAngle Lower arm joint angle, in radians.
   * @param upperAngle Upper arm joint angle, in radians.
   * @param matrix     Filled with the lower diagonal, the off diagonal, then the upper diagonal entry.
   */
  public void getMassMatrix(double lowerAngle, double upperAngle, double[] matrix) {
    final double a = lowerMOI + upperMass * lowerLength * lowerLength;
    final double b = upperMass * lowerLength * upperCenterOfMass * Math.cos(lowerAngle - getUpperAngleFromHorizontal(lowerAngle, upperAngle));
    final double c = upperMOI;
    matrix[0] = a + 2 * b * k + c * k * k;
    matrix[1] = b + c * k;
    matrix[2] = c;
  }

  /**
   * Torques from gravity and the arms swinging on each other, in joint space: {@code Aᵀ (G − C)}.
   *
   * @param lowerAngle    Lower arm joint angle, in radians.
   * @param upperAngle    Upper arm joint angle, in radians.
   * @param lowerVelocity Lower arm joint velocity, in radians per second.
   * @param upperVelocity Upper arm joint velocity, in radians per second.
   * @param torques       Filled with the lower then upper joint's torque.
   */
  public void getPassiveTorques(double lowerAngle, double upperAngle, double lowerVelocity, double upperVelocity, double[] torques) {
    final double upperFromHorizontal = getUpperAngleFromHorizontal(lowerAngle, upperAngle);
    final double upperSpeed          = upperVelocity + k * lowerVelocity;
    final double swing               = upperMass * lowerLength * upperCenterOfMass * Math.sin(lowerAngle - upperFromHorizontal);
    final double lowerGravity        = -(lowerMassMoment + upperMass * lowerLength) * kGravity * Math.cos(lowerAngle);
    final double upperGravity        = -upperMass * upperCenterOfMass * kGravity * Math.cos(upperFromHorizontal);
    final double lower               = lowerGravity - swing * upperSpeed * upperSpeed;
    final double upper               = upperGravity + swing * lowerVelocity * lowerVelocity;
    torques[0] = lower + k * upper;
    torques[1] = upper;
  }

  /**
   * The least inertia each joint can have, when the other joint is free and the arms line up, in kilogram square
   * meters: how fast a torque can speed it up.
   *
   * @param joint 0 for the lower joint, 1 for the upper joint.
   * @return Least effective inertia of the joint.
   */
  public double getLeastEffectiveInertia(int joint) {
    final double a           = lowerMOI + upperMass * lowerLength * lowerLength;
    final double coupling    = upperMass * lowerLength * upperCenterOfMass;
    final double c           = upperMOI;
    // det(Aᵀ M A) = det(M), as det(A) = 1, and is least with the arms lined up.
    final double determinant = a * c - coupling * coupling;
    return joint == 0 ? determinant / c : determinant / (a + 2 * coupling * k + c * k * k);
  }
}
