// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers.simulation;

import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

import java.util.ArrayDeque;
import java.util.Deque;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Mass;

/**
 * The game pieces a simulated mechanism carries, and the voltage that makes WPILib's simulation of the
 * mechanism without them move as the mechanism moves with them.
 *
 * <p>WPILib's simulations fix the mechanism's mass, or moment of inertia, when they are created. A game
 * piece makes the motor's voltage move the mechanism less, and on an arm, pulls it down further. Both
 * can be given to the unchanged simulation as a different voltage: the voltage that gives the simulation
 * the acceleration the mechanism has with the game pieces. That voltage is worked out at the start of
 * each step, and the simulation holds it through the step, so it is exact for the step's first moment
 * and close to it for the rest.
 */
final class GamePieces {
  /** The acceleration of gravity, in meters per second squared, as WPILib's simulations use it. */
  private static final double GRAVITY = 9.8;

  /** Each game piece: its mass in kilograms, and its distance from the axis in meters. */
  private final Deque<double[]> m_pieces = new ArrayDeque<>();
  /** The game pieces' total mass, in kilograms. */
  private volatile double m_mass = 0;
  /** The sum of each game piece's mass times its distance from the axis, in kilogram meters. */
  private volatile double m_massMoment = 0;
  /** The game pieces' moment of inertia about the axis, in kilogram square meters. */
  private volatile double m_moi = 0;

  /**
   * Add a game piece.
   *
   * @param mass     The game piece's mass.
   * @param distance Its distance from the axis the mechanism turns about. Zero for an elevator.
   */
  synchronized void add(Mass mass, Distance distance) {
    double kg = mass.in(Kilograms);
    double meters = distance.in(Meters);
    if (kg < 0) {
      throw new IllegalArgumentException("A game piece's mass cannot be negative: " + mass);
    }
    m_pieces.push(new double[] {kg, meters});
    total();
  }

  /** Remove the game piece added last. Does nothing when there is none. */
  synchronized void removeLast() {
    m_pieces.poll();
    total();
  }

  private void total() {
    double mass = 0;
    double massMoment = 0;
    double moi = 0;
    for (double[] piece : m_pieces) {
      mass += piece[0];
      massMoment += piece[0] * piece[1];
      moi += piece[0] * piece[1] * piece[1];
    }
    m_mass = mass;
    m_massMoment = massMoment;
    m_moi = moi;
  }

  /** @return Whether there are no game pieces, so the simulation needs no different voltage. */
  boolean isEmpty() {
    return m_mass == 0;
  }

  /** @return The game pieces' total mass, in kilograms. */
  double mass() {
    return m_mass;
  }

  /** @return The game pieces' moment of inertia about the axis, in kilogram square meters. */
  double moi() {
    return m_moi;
  }

  /**
   * The voltage that makes a simulation without the game pieces accelerate as the mechanism with them
   * does, for a mechanism whose only change is its inertia: an elevator, a pivot or a flywheel. The
   * motor's own share of the voltage, the voltage above its back EMF, is scaled by the inertia without
   * the game pieces over the inertia with them. Gravity does not change it: it accelerates an elevator
   * the same with or without them.
   *
   * @param volts             The voltage the motor is given.
   * @param backEmfVolts      The motor's back EMF.
   * @param inertiaWithout    The mass or moment of inertia without the game pieces.
   * @param inertiaWithPieces The mass or moment of inertia with them.
   * @return The voltage to give the simulation.
   */
  static double inertiaVoltage(double volts, double backEmfVolts, double inertiaWithout, double inertiaWithPieces) {
    double ratio = inertiaWithout / inertiaWithPieces;
    return backEmfVolts + ratio * (volts - backEmfVolts);
  }

  /**
   * The voltage that makes WPILib's arm simulation without the game pieces accelerate as the arm with
   * them does. The game pieces add to the arm's moment of inertia, and their weight pulls it down.
   *
   * @param volts           The voltage the motor is given.
   * @param backEmfVolts    The motor's back EMF.
   * @param angleRadians    The arm's angle, 0 being level.
   * @param armMoi          The arm's moment of inertia without the game pieces, in kilogram square
   *                        meters.
   * @param armLength       The arm's length, in meters, from which WPILib works out its gravity.
   * @param simulateGravity Whether the simulation simulates gravity.
   * @param motor           The arm's motors.
   * @param gearing         The reduction from the motors to the arm.
   * @return The voltage to give the simulation.
   */
  double armVoltage(double volts, double backEmfVolts, double angleRadians, double armMoi, double armLength,
                    boolean simulateGravity, DCMotor motor, double gearing) {
    double moiWithPieces = armMoi + m_moi;
    // The angular acceleration of one volt above the back EMF, without the game pieces.
    double accelerationPerVolt = gearing * motor.Kt / (motor.R * armMoi);
    double cos = Math.cos(angleRadians);
    // WPILib's arm is a uniform rod: gravity's angular acceleration is 3/2 g cos(angle) / length.
    double armGravity = simulateGravity ? -1.5 * GRAVITY * cos / armLength : 0;
    double pieceGravityTorque = simulateGravity ? -GRAVITY * cos * m_massMoment : 0;
    double without = accelerationPerVolt * (volts - backEmfVolts) + armGravity;
    double with = (armMoi * (accelerationPerVolt * (volts - backEmfVolts) + armGravity) + pieceGravityTorque) / moiWithPieces;
    return volts + (with - without) / accelerationPerVolt;
  }
}
