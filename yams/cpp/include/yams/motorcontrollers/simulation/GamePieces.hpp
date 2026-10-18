// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <cmath>
#include <mutex>
#include <stdexcept>
#include <vector>

#include <wpi/math/system/DCMotor.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/mass.hpp>

namespace yams::motorcontrollers::simulation {

/**
 * The game pieces a simulated mechanism carries, and the voltage that makes WPILib's simulation of the
 * mechanism without them move as the mechanism moves with them.
 *
 * WPILib's simulations fix the mechanism's mass, or moment of inertia, when they are created. A game
 * piece makes the motor's voltage move the mechanism less, and on an arm, pulls it down further. Both
 * can be given to the unchanged simulation as a different voltage: the voltage that gives the
 * simulation the acceleration the mechanism has with the game pieces. That voltage is worked out at the
 * start of each step, and the simulation holds it through the step, so it is exact for the step's first
 * moment and close to it for the rest.
 */
class GamePieces {
 public:
  /**
   * Add a game piece.
   *
   * @param mass The game piece's mass.
   * @param distance Its distance from the axis the mechanism turns about. Zero for an elevator.
   */
  void Add(wpi::units::kilogram_t mass, wpi::units::meter_t distance) {
    if (mass.value() < 0) {
      throw std::invalid_argument("A game piece's mass cannot be negative.");
    }
    std::scoped_lock lock{m_mutex};
    m_pieces.push_back({mass.value(), distance.value()});
    Total();
  }

  /** Remove the game piece added last. Does nothing when there is none. */
  void RemoveLast() {
    std::scoped_lock lock{m_mutex};
    if (!m_pieces.empty()) m_pieces.pop_back();
    Total();
  }

  /** Whether there are no game pieces, so the simulation needs no different voltage. */
  bool IsEmpty() const {
    std::scoped_lock lock{m_mutex};
    return m_mass == 0;
  }

  /** The game pieces' total mass, in kilograms. */
  double Mass() const {
    std::scoped_lock lock{m_mutex};
    return m_mass;
  }

  /** The game pieces' moment of inertia about the axis, in kilogram square meters. */
  double Moi() const {
    std::scoped_lock lock{m_mutex};
    return m_moi;
  }

  /**
   * The voltage that makes a simulation without the game pieces accelerate as the mechanism with them
   * does, for a mechanism whose only change is its inertia: an elevator, a pivot or a flywheel. The
   * motor's own share of the voltage, the voltage above its back EMF, is scaled by the inertia without
   * the game pieces over the inertia with them.
   *
   * @param volts The voltage the motor is given.
   * @param backEmfVolts The motor's back EMF.
   * @param inertiaWithout The mass or moment of inertia without the game pieces.
   * @param inertiaWithPieces The mass or moment of inertia with them.
   * @return The voltage to give the simulation.
   */
  static double InertiaVoltage(double volts, double backEmfVolts, double inertiaWithout,
                               double inertiaWithPieces) {
    return backEmfVolts + inertiaWithout / inertiaWithPieces * (volts - backEmfVolts);
  }

  /**
   * The voltage that makes WPILib's arm simulation without the game pieces accelerate as the arm with
   * them does. The game pieces add to the arm's moment of inertia, and their weight pulls it down.
   *
   * @param volts The voltage the motor is given.
   * @param backEmfVolts The motor's back EMF.
   * @param angleRadians The arm's angle, 0 being level.
   * @param armMoi The arm's moment of inertia without the game pieces, in kilogram square meters.
   * @param armLength The arm's length, in meters, from which WPILib works out its gravity.
   * @param simulateGravity Whether the simulation simulates gravity.
   * @param motor The arm's motors.
   * @param gearing The reduction from the motors to the arm.
   * @return The voltage to give the simulation.
   */
  double ArmVoltage(double volts, double backEmfVolts, double angleRadians, double armMoi,
                    double armLength, bool simulateGravity, const wpi::math::DCMotor& motor,
                    double gearing) const {
    double moi;
    double massMoment;
    {
      std::scoped_lock lock{m_mutex};
      moi = m_moi;
      massMoment = m_massMoment;
    }
    const double moiWithPieces = armMoi + moi;
    // The angular acceleration of one volt above the back EMF, without the game pieces.
    const double accelerationPerVolt = gearing * motor.Kt.value() / (motor.R.value() * armMoi);
    const double cos = std::cos(angleRadians);
    // WPILib's arm is a uniform rod: gravity's angular acceleration is 3/2 g cos(angle) / length.
    const double armGravity = simulateGravity ? -1.5 * kGravity * cos / armLength : 0;
    const double pieceGravityTorque = simulateGravity ? -kGravity * cos * massMoment : 0;
    const double without = accelerationPerVolt * (volts - backEmfVolts) + armGravity;
    const double with =
        (armMoi * (accelerationPerVolt * (volts - backEmfVolts) + armGravity) + pieceGravityTorque) /
        moiWithPieces;
    return volts + (with - without) / accelerationPerVolt;
  }

 private:
  /** The acceleration of gravity, in meters per second squared, as WPILib's simulations use it. */
  static constexpr double kGravity = 9.8;

  struct Piece {
    double mass;
    double distance;
  };

  void Total() {
    double mass = 0;
    double massMoment = 0;
    double moi = 0;
    for (const auto& piece : m_pieces) {
      mass += piece.mass;
      massMoment += piece.mass * piece.distance;
      moi += piece.mass * piece.distance * piece.distance;
    }
    m_mass = mass;
    m_massMoment = massMoment;
    m_moi = moi;
  }

  mutable std::mutex m_mutex;
  std::vector<Piece> m_pieces;
  double m_mass{0};
  double m_massMoment{0};
  double m_moi{0};
};

}  // namespace yams::motorcontrollers::simulation
