// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers.simulation;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

/**
 * Physics simulation of a double jointed arm, with both joints simulated together so each moves the other.
 *
 * <p>Two {@link org.wpilib.simulation.SingleJointedArmSim}s can't simulate a double jointed arm: the shoulder carries
 * the upper arm's weight, which depends on where the upper arm points, and the upper arm's inertia, which depends on the
 * angle between the arms, and swinging either arm pushes on the other. This simulates the two-link equations of
 * motion,
 *
 * <pre>
 *   M(θ) θ'' + C(θ, θ') + G(θ) = τ
 *
 *   M = [ J1 + m2 l1²              m2 l1 r2 cos(θ1 − θ2) ]
 *       [ m2 l1 r2 cos(θ1 − θ2)    J2                    ]
 *   C = [  m2 l1 r2 sin(θ1 − θ2) θ2'² ]
 *       [ −m2 l1 r2 sin(θ1 − θ2) θ1'² ]
 *   G = [ (m1 r1 + m2 l1) g cos θ1 ]
 *       [ m2 r2 g cos θ2           ]
 * </pre>
 *
 * <p>Both angles are from horizontal, as {@link yams.core.mechanisms.positional.DoubleJointedArm} measures them. That
 * is an arm whose elbow is driven from the base, e.g. by a chain through the shoulder, so the elbow motor's torque and
 * speed are those of the upper arm itself.
 *
 * <p>Each arm is a uniform rod, as {@link org.wpilib.simulation.SingleJointedArmSim} assumes: its moment of inertia
 * {@code J} about its own pivot gives its mass, {@code m = 3 J / l²}, with its center of mass halfway along it,
 * {@code r = l / 2}. A moment of inertia from
 * {@link yams.core.motorcontrollers.SmartMotorControllerConfig#withMomentOfInertia(Distance, org.wpilib.units.measure.Mass)}
 * gives back the mass it was made from.
 */
public class DoubleJointedArmSim {
  /** Acceleration of gravity, in meters per second squared, as WPILib's arm simulation uses. */
  private static final double kGravity      = 9.8;
  /** Longest integration step, in seconds. */
  private static final double kMaxStep      = 0.001;
  /** Lower arm motors. */
  private final DCMotor       lowerMotor;
  /** Upper arm motors. */
  private final DCMotor       upperMotor;
  /** Lower arm gearing, rotor rotations per arm rotation. */
  private final double        lowerGearing;
  /** Upper arm gearing, rotor rotations per arm rotation. */
  private final double        upperGearing;
  /** Lower arm length, in meters. */
  private final double        lowerLength;
  /** Lower arm moment of inertia about the shoulder, in kilogram square meters. */
  private final double        lowerMOI;
  /** Upper arm moment of inertia about the elbow, in kilogram square meters. */
  private final double        upperMOI;
  /** Lower arm mass times the distance from the shoulder to its center of mass, in kilogram meters. */
  private final double        lowerMassMoment;
  /** Upper arm mass, in kilograms. */
  private final double        upperMass;
  /** Distance from the elbow to the upper arm's center of mass, in meters. */
  private final double        upperCenterOfMass;
  /** Lower arm angle limits, in radians. */
  private final double        lowerMin;
  /** Lower arm angle limits, in radians. */
  private final double        lowerMax;
  /** Upper arm angle limits, in radians. */
  private final double        upperMin;
  /** Upper arm angle limits, in radians. */
  private final double        upperMax;
  /** Simulate gravity? */
  private final boolean       simulateGravity;
  /**
   * Fastest rate a joint's motor can slow it, in 1/s: its back-EMF damping over the least inertia it can have. Integration
   * steps are kept short enough for it, so a light arm on a big reduction doesn't blow up.
   */
  private final double        stiffness;
  /** State: lower angle, upper angle, lower velocity, upper velocity, in radians and radians per second. */
  private final double[]      state         = new double[4];
  /** RK4 slopes and the state they're taken at, kept so stepping allocates nothing. */
  private final double[]      k1            = new double[4];
  private final double[]      k2            = new double[4];
  private final double[]      k3            = new double[4];
  private final double[]      k4            = new double[4];
  private final double[]      stage         = new double[4];
  /** Gravity torques, kept so stepping allocates nothing. */
  private final double[]      gravity       = new double[2];
  /** Lower arm motor voltage. */
  private double              lowerVoltage  = 0;
  /** Upper arm motor voltage. */
  private double              upperVoltage  = 0;

  /**
   * Create a double jointed arm simulation.
   *
   * @param lowerMotor        Lower arm motors.
   * @param lowerGearing      Lower arm gearing, rotor rotations per arm rotation.
   * @param lowerMOI          Lower arm moment of inertia about the shoulder, in kilogram square meters.
   * @param lowerLength       Lower arm length, from the shoulder to the elbow.
   * @param lowerMin          Lowest lower arm angle, from horizontal.
   * @param lowerMax          Highest lower arm angle, from horizontal.
   * @param lowerStart        Starting lower arm angle, from horizontal.
   * @param upperMotor        Upper arm motors.
   * @param upperGearing      Upper arm gearing, rotor rotations per arm rotation.
   * @param upperMOI          Upper arm moment of inertia about the elbow, in kilogram square meters.
   * @param upperLength       Upper arm length.
   * @param upperMin          Lowest upper arm angle, from horizontal.
   * @param upperMax          Highest upper arm angle, from horizontal.
   * @param upperStart        Starting upper arm angle, from horizontal.
   * @param simulateGravity   Whether gravity acts on the arm.
   * @throws IllegalArgumentException if an arm's length, moment of inertia or gearing is not above 0.
   */
  public DoubleJointedArmSim(DCMotor lowerMotor, double lowerGearing, double lowerMOI, Distance lowerLength, Angle lowerMin, Angle lowerMax, Angle lowerStart,
                             DCMotor upperMotor, double upperGearing, double upperMOI, Distance upperLength, Angle upperMin, Angle upperMax, Angle upperStart,
                             boolean simulateGravity) {
    this.lowerMotor = lowerMotor;
    this.upperMotor = upperMotor;
    this.lowerGearing = lowerGearing;
    this.upperGearing = upperGearing;
    this.lowerLength = lowerLength.in(Meters);
    this.lowerMOI = lowerMOI;
    this.upperMOI = upperMOI;
    final double upperLengthMeters = upperLength.in(Meters);
    if (!(this.lowerLength > 0 && upperLengthMeters > 0 && lowerMOI > 0 && upperMOI > 0 && lowerGearing > 0 && upperGearing > 0)) {
      throw new IllegalArgumentException("Arm lengths, moments of inertia and gearings must be above 0");
    }
    // Uniform rods: m = 3 J / l², with the center of mass at l / 2.
    this.lowerMassMoment = 3 * lowerMOI / (this.lowerLength * this.lowerLength) * (this.lowerLength / 2);
    this.upperMass = 3 * upperMOI / (upperLengthMeters * upperLengthMeters);
    this.upperCenterOfMass = upperLengthMeters / 2;
    this.lowerMin = lowerMin.in(Radians);
    this.lowerMax = lowerMax.in(Radians);
    this.upperMin = upperMin.in(Radians);
    this.upperMax = upperMax.in(Radians);
    this.simulateGravity = simulateGravity;
    // A joint's effective inertia is least when the other joint is free and lined up with it: M11 − M12² / M22 and
    // M22 − M12² / M11 at cos(θ1 − θ2) = ±1.
    final double coupling  = upperMass * this.lowerLength * upperCenterOfMass;
    final double m11       = lowerMOI + upperMass * this.lowerLength * this.lowerLength;
    final double lowerDamping = lowerGearing * lowerGearing * lowerMotor.Kt / (lowerMotor.R * lowerMotor.Kv);
    final double upperDamping = upperGearing * upperGearing * upperMotor.Kt / (upperMotor.R * upperMotor.Kv);
    this.stiffness = Math.max(lowerDamping / (m11 - coupling * coupling / upperMOI), upperDamping / (upperMOI - coupling * coupling / m11));
    state[0] = lowerStart.in(Radians);
    state[1] = upperStart.in(Radians);
  }

  /**
   * Set the lower arm motor voltage.
   *
   * @param volts Voltage.
   */
  public void setLowerInputVoltage(double volts) {
    lowerVoltage = volts;
  }

  /**
   * Set the upper arm motor voltage.
   *
   * @param volts Voltage.
   */
  public void setUpperInputVoltage(double volts) {
    upperVoltage = volts;
  }

  /**
   * Get the lower arm motor voltage.
   *
   * @return Voltage.
   */
  public double getLowerInputVoltage() {
    return lowerVoltage;
  }

  /**
   * Get the upper arm motor voltage.
   *
   * @return Voltage.
   */
  public double getUpperInputVoltage() {
    return upperVoltage;
  }

  /**
   * Set the arm's state.
   *
   * @param lowerAngle    Lower arm angle, in radians.
   * @param lowerVelocity Lower arm velocity, in radians per second.
   * @param upperAngle    Upper arm angle, in radians.
   * @param upperVelocity Upper arm velocity, in radians per second.
   */
  public void setState(double lowerAngle, double lowerVelocity, double upperAngle, double upperVelocity) {
    state[0] = lowerAngle;
    state[1] = upperAngle;
    state[2] = lowerVelocity;
    state[3] = upperVelocity;
  }

  /**
   * Step the simulation, integrating the equations of motion with RK4 in steps of at most a millisecond, and shorter
   * when the motors could slow the arm faster than that.
   *
   * @param dtSeconds Time to step, in seconds.
   */
  public void update(double dtSeconds) {
    final int    steps = Math.max(1, (int) Math.ceil(Math.max(dtSeconds / kMaxStep, dtSeconds * stiffness)));
    final double dt    = dtSeconds / steps;
    for (int step = 0; step < steps; step++) {
      derivative(state, k1);
      derivative(add(state, k1, dt / 2, stage), k2);
      derivative(add(state, k2, dt / 2, stage), k3);
      derivative(add(state, k3, dt, stage), k4);
      for (int i = 0; i < 4; i++) {
        state[i] += dt / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
      }
      // Hard stops: the arm stops where it hits them.
      if (state[0] < lowerMin || state[0] > lowerMax) {
        state[0] = Math.max(lowerMin, Math.min(lowerMax, state[0]));
        state[2] = 0;
      }
      if (state[1] < upperMin || state[1] > upperMax) {
        state[1] = Math.max(upperMin, Math.min(upperMax, state[1]));
        state[3] = 0;
      }
    }
  }

  /**
   * Torque on each arm from gravity, in newton meters, positive raising the arm.
   *
   * @param lowerAngle Lower arm angle, in radians.
   * @param upperAngle Upper arm angle, in radians.
   * @return Lower then upper arm gravity torque.
   */
  public double[] getGravityTorques(double lowerAngle, double upperAngle) {
    final double[] torques = new double[2];
    gravityTorques(lowerAngle, upperAngle, torques);
    return torques;
  }

  /** Put the gravity torques into {@code torques}, lower then upper. */
  private void gravityTorques(double lowerAngle, double upperAngle, double[] torques) {
    torques[0] = simulateGravity ? -(lowerMassMoment + upperMass * lowerLength) * kGravity * Math.cos(lowerAngle) : 0;
    torques[1] = simulateGravity ? -upperMass * upperCenterOfMass * kGravity * Math.cos(upperAngle) : 0;
  }

  /**
   * State derivative into {@code dx}: velocities, then the accelerations solved from M(θ) θ'' = τ − C(θ, θ') − G(θ).
   */
  private void derivative(double[] x, double[] dx) {
    final double delta   = x[0] - x[1];
    final double coupling = upperMass * lowerLength * upperCenterOfMass;
    final double m11     = lowerMOI + upperMass * lowerLength * lowerLength;
    final double m12     = coupling * Math.cos(delta);
    final double m22     = upperMOI;
    gravityTorques(x[0], x[1], gravity);
    final double lowerTorque = motorTorque(lowerMotor, lowerGearing, x[2], lowerVoltage) - coupling * Math.sin(delta) * x[3] * x[3] + gravity[0];
    final double upperTorque = motorTorque(upperMotor, upperGearing, x[3], upperVoltage) + coupling * Math.sin(delta) * x[2] * x[2] + gravity[1];
    final double determinant = m11 * m22 - m12 * m12;
    double lowerAcceleration = (m22 * lowerTorque - m12 * upperTorque) / determinant;
    double upperAcceleration = (m11 * upperTorque - m12 * lowerTorque) / determinant;
    // A joint pushed into its hard stop is held there, so it can't accelerate, and the other joint moves as if it was
    // fixed: M θ'' = τ with that joint's θ'' = 0. Holding one can push the other into its stop, so check twice.
    boolean lowerStopped = false;
    boolean upperStopped = false;
    for (int check = 0; check < 2; check++) {
      lowerStopped |= isPushedIntoStop(x[0], x[2], lowerAcceleration, lowerMin, lowerMax);
      upperStopped |= isPushedIntoStop(x[1], x[3], upperAcceleration, upperMin, upperMax);
      lowerAcceleration = lowerStopped ? 0 : upperStopped ? lowerTorque / m11 : lowerAcceleration;
      upperAcceleration = upperStopped ? 0 : lowerStopped ? upperTorque / m22 : upperAcceleration;
    }
    dx[0] = x[2];
    dx[1] = x[3];
    dx[2] = lowerAcceleration;
    dx[3] = upperAcceleration;
  }

  /** Whether a joint is at a hard stop and moving or accelerating into it. */
  private static boolean isPushedIntoStop(double angle, double velocity, double acceleration, double min, double max) {
    final double epsilon = 1e-9;
    return (angle <= min + epsilon && velocity <= 0 && acceleration < 0) || (angle >= max - epsilon && velocity >= 0 && acceleration > 0);
  }

  /** Torque at the arm from its motors, in newton meters. */
  private static double motorTorque(DCMotor motor, double gearing, double armVelocity, double volts) {
    return gearing * motor.getTorque(motor.getCurrent(armVelocity * gearing, volts));
  }

  /** {@code x + k * scale}, into {@code out}. */
  private static double[] add(double[] x, double[] k, double scale, double[] out) {
    for (int i = 0; i < 4; i++) {
      out[i] = x[i] + k[i] * scale;
    }
    return out;
  }

  /**
   * Get the lower arm angle.
   *
   * @return Angle from horizontal, in radians.
   */
  public double getLowerAngle() {
    return state[0];
  }

  /**
   * Get the upper arm angle.
   *
   * @return Angle from horizontal, in radians.
   */
  public double getUpperAngle() {
    return state[1];
  }

  /**
   * Get the lower arm velocity.
   *
   * @return Radians per second.
   */
  public double getLowerVelocity() {
    return state[2];
  }

  /**
   * Get the upper arm velocity.
   *
   * @return Radians per second.
   */
  public double getUpperVelocity() {
    return state[3];
  }

  /**
   * Get the lower arm motors' current draw, in the direction of the voltage as WPILib's arm simulation reports it.
   *
   * @return Amps.
   */
  public double getLowerCurrentDraw() {
    return lowerMotor.getCurrent(state[2] * lowerGearing, lowerVoltage) * Math.signum(lowerVoltage);
  }

  /**
   * Get the upper arm motors' current draw, in the direction of the voltage as WPILib's arm simulation reports it.
   *
   * @return Amps.
   */
  public double getUpperCurrentDraw() {
    return upperMotor.getCurrent(state[3] * upperGearing, upperVoltage) * Math.signum(upperVoltage);
  }
}
