// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers.simulation;

import static org.wpilib.units.Units.Radians;

import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import yams.core.math.DoubleJointedArmDynamics;

/**
 * Physics simulation of a double jointed arm, with both joints simulated together so each moves the other.
 *
 * <p>Two {@link org.wpilib.simulation.SingleJointedArmSim}s can't simulate a double jointed arm: the shoulder carries
 * the upper arm's weight and its inertia, which depends on the angle between the arms, and swinging either arm pushes
 * on the other. This integrates the equations of motion of {@link DoubleJointedArmDynamics}, in the motor controllers'
 * joint angles, so each motor's torque and back-EMF are on its own joint.
 *
 * <p>Not thread safe: it keeps working arrays, so stepping allocates nothing.
 */
public class DoubleJointedArmSim {
  /** Longest integration step, in seconds. */
  private static final double            kMaxStep     = 0.001;
  /** Equations of motion. */
  private final DoubleJointedArmDynamics dynamics;
  /** Lower arm motors. */
  private final DCMotor                  lowerMotor;
  /** Upper arm motors. */
  private final DCMotor                  upperMotor;
  /** Lower arm gearing, rotor rotations per arm rotation. */
  private final double                   lowerGearing;
  /** Upper arm gearing, rotor rotations per arm rotation. */
  private final double                   upperGearing;
  /** Lowest lower joint angle, in radians. */
  private final double                   lowerMin;
  /** Highest lower joint angle, in radians. */
  private final double                   lowerMax;
  /** Lowest upper joint angle, in radians. */
  private final double                   upperMin;
  /** Highest upper joint angle, in radians. */
  private final double                   upperMax;
  /** Simulate gravity? */
  private final boolean                  simulateGravity;
  /**
   * Fastest rate a joint's motor can slow it, in 1/s: its back-EMF damping over the least inertia it can have.
   * Integration steps are kept short enough for it, so a light arm on a big reduction doesn't blow up.
   */
  private final double                   stiffness;
  /** State: lower angle, upper angle, lower velocity, upper velocity, in radians and radians per second. */
  private final double[]                 state        = new double[4];
  /** RK4 slopes and the state they're taken at, kept so stepping allocates nothing. */
  private final double[]                 k1           = new double[4];
  private final double[]                 k2           = new double[4];
  private final double[]                 k3           = new double[4];
  private final double[]                 k4           = new double[4];
  private final double[]                 stage        = new double[4];
  /** Mass matrix, passive torques and gravity torques, kept so stepping allocates nothing. */
  private final double[]                 massMatrix   = new double[3];
  private final double[]                 passive      = new double[2];
  private final double[]                 gravity      = new double[2];
  /** Lower arm motor voltage. */
  private double                         lowerVoltage = 0;
  /** Upper arm motor voltage. */
  private double                         upperVoltage = 0;

  /**
   * Create a double jointed arm simulation. Angles are the motor controllers' joint angles.
   *
   * @param dynamics        Equations of motion of the arm.
   * @param lowerMotor      Lower arm motors.
   * @param lowerGearing    Lower arm gearing, rotor rotations per arm rotation.
   * @param lowerMin        Lowest lower joint angle.
   * @param lowerMax        Highest lower joint angle.
   * @param lowerStart      Starting lower joint angle.
   * @param upperMotor      Upper arm motors.
   * @param upperGearing    Upper arm gearing, rotor rotations per arm rotation.
   * @param upperMin        Lowest upper joint angle.
   * @param upperMax        Highest upper joint angle.
   * @param upperStart      Starting upper joint angle.
   * @param simulateGravity Whether gravity acts on the arm.
   * @throws IllegalArgumentException if a gearing is not above 0.
   */
  public DoubleJointedArmSim(DoubleJointedArmDynamics dynamics, DCMotor lowerMotor, double lowerGearing, Angle lowerMin, Angle lowerMax, Angle lowerStart,
                             DCMotor upperMotor, double upperGearing, Angle upperMin, Angle upperMax, Angle upperStart, boolean simulateGravity) {
    if (!(lowerGearing > 0 && upperGearing > 0)) {
      throw new IllegalArgumentException("Gearings must be above 0");
    }
    this.dynamics = dynamics;
    this.lowerMotor = lowerMotor;
    this.upperMotor = upperMotor;
    this.lowerGearing = lowerGearing;
    this.upperGearing = upperGearing;
    this.lowerMin = lowerMin.in(Radians);
    this.lowerMax = lowerMax.in(Radians);
    this.upperMin = upperMin.in(Radians);
    this.upperMax = upperMax.in(Radians);
    this.simulateGravity = simulateGravity;
    final double lowerDamping = lowerGearing * lowerGearing * lowerMotor.Kt / (lowerMotor.R * lowerMotor.Kv);
    final double upperDamping = upperGearing * upperGearing * upperMotor.Kt / (upperMotor.R * upperMotor.Kv);
    this.stiffness = Math.max(lowerDamping / dynamics.getLeastEffectiveInertia(0), upperDamping / dynamics.getLeastEffectiveInertia(1));
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
   * Set the arm's state, in joint angles.
   *
   * @param lowerAngle    Lower joint angle, in radians.
   * @param lowerVelocity Lower joint velocity, in radians per second.
   * @param upperAngle    Upper joint angle, in radians.
   * @param upperVelocity Upper joint velocity, in radians per second.
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
   * Torque gravity puts on each joint, in newton meters, positive raising the joint.
   *
   * @param lowerAngle Lower joint angle, in radians.
   * @param upperAngle Upper joint angle, in radians.
   * @return Lower then upper joint gravity torque.
   */
  public double[] getGravityTorques(double lowerAngle, double upperAngle) {
    final double[] torques = new double[2];
    if (simulateGravity) {
      dynamics.getGravityTorques(lowerAngle, upperAngle, torques);
    }
    return torques;
  }

  /**
   * State derivative into {@code dx}: velocities, then the accelerations solved from Aᵀ M A q'' = τ + Aᵀ (G − C).
   */
  private void derivative(double[] x, double[] dx) {
    dynamics.getMassMatrix(x[0], x[1], massMatrix);
    dynamics.getPassiveTorques(x[0], x[1], x[2], x[3], passive);
    if (!simulateGravity) {
      dynamics.getGravityTorques(x[0], x[1], gravity);
      passive[0] -= gravity[0];
      passive[1] -= gravity[1];
    }
    final double lowerTorque = motorTorque(lowerMotor, lowerGearing, x[2], lowerVoltage) + passive[0];
    final double upperTorque = motorTorque(upperMotor, upperGearing, x[3], upperVoltage) + passive[1];
    final double m11         = massMatrix[0];
    final double m12         = massMatrix[1];
    final double m22         = massMatrix[2];
    final double determinant = m11 * m22 - m12 * m12;
    double lowerAcceleration = (m22 * lowerTorque - m12 * upperTorque) / determinant;
    double upperAcceleration = (m11 * upperTorque - m12 * lowerTorque) / determinant;
    // A joint pushed into its hard stop is held there, so it can't accelerate, and the other joint moves as if it was
    // fixed. Holding one can push the other into its stop, so check twice.
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

  /** Torque at the joint from its motors, in newton meters. */
  private static double motorTorque(DCMotor motor, double gearing, double jointVelocity, double volts) {
    return gearing * motor.getTorque(motor.getCurrent(jointVelocity * gearing, volts));
  }

  /** {@code x + k * scale}, into {@code out}. */
  private static double[] add(double[] x, double[] k, double scale, double[] out) {
    for (int i = 0; i < 4; i++) {
      out[i] = x[i] + k[i] * scale;
    }
    return out;
  }

  /**
   * Get the lower joint angle.
   *
   * @return Angle, in radians.
   */
  public double getLowerAngle() {
    return state[0];
  }

  /**
   * Get the upper joint angle.
   *
   * @return Angle, in radians.
   */
  public double getUpperAngle() {
    return state[1];
  }

  /**
   * Get the lower joint velocity.
   *
   * @return Radians per second.
   */
  public double getLowerVelocity() {
    return state[2];
  }

  /**
   * Get the upper joint velocity.
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
