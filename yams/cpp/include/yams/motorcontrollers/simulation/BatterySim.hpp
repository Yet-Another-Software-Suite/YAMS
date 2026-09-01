// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <wpi/units/current.hpp>
#include <wpi/units/impedance.hpp>
#include <wpi/units/voltage.hpp>

#include <map>
#include <unordered_map>

namespace yams::motorcontrollers::simulation {

/**
 * Battery Simulation that takes the entirety of the robot into account by storing all currently
 * used currents on the robot.
 *
 * Each simulated mechanism should call CalculateVoltage() every simulation iteration with a
 * stable, unique identity (e.g. the address of its SimSupplier instance) and its own current
 * draw. The result accounts for the combined current draw of every registered mechanism, and
 * should be fed into wpi::sim::RoboRioSim::SetVInVoltage().
 */
class BatterySim {
 public:
  /** Battery open circuit voltage, used when discharge simulation is disabled. */
  static wpi::units::volt_t BatteryVoltage;
  /** Battery internal resistance, used when discharge simulation is disabled. */
  static wpi::units::ohm_t BatteryResistance;

  /**
   * Calculate the voltage based on the currents used by the robot.
   *
   * @param id      Stable, unique identity of the simulation to calculate the voltage for.
   * @param current Current used by the robot.
   * @return Loaded voltage of the robot.
   */
  static wpi::units::volt_t CalculateVoltage(const void* id, wpi::units::ampere_t current);

  /**
   * Replace the default state-of-charge -> open circuit voltage interpolation table used when
   * discharge simulation is enabled with EnableDischarge().
   *
   * Not every battery discharges like YAMS's built-in sealed lead-acid curve. Reach for this
   * method when you want to model something different, for example:
   *   - A well-used competition battery that sags earlier and harder than a fresh one.
   *   - Matching a curve you measured from an actual battery on a load tester, for the most
   *     accurate brownout predictions possible.
   *
   * @code
   * // Model a well-used competition battery that sags earlier and more severely than a new one.
   * std::map<double, double> wornBatteryCurve{
   *     {0.00, 8.0},  {0.05, 9.5},  {0.10, 10.5}, {0.20, 11.2}, {0.40, 11.6},
   *     {0.60, 11.9}, {0.80, 12.2}, {0.90, 12.4}, {1.00, 12.6},
   * };
   *
   * BatterySim::ReplaceSOCInterpolation(wornBatteryCurve);
   * // Pair with a reduced usable capacity and higher resistance to match a worn battery.
   * BatterySim::EnableDischarge(15.0, wpi::units::volt_t{12.6}, wpi::units::ohm_t{0.028});
   * @endcode
   *
   * @param socToVoltage Interpolation table mapping state of charge [0, 1] to open circuit
   *                     voltage. Call this before EnableDischarge() so discharge simulation uses
   *                     the new curve from the start.
   */
  static void ReplaceSOCInterpolation(const std::map<double, double>& socToVoltage);

  /**
   * Enable realistic battery discharge simulation. As current is drawn from the battery over
   * time its state of charge will drop, reducing the open circuit voltage and increasing the
   * internal resistance to more realistically model a depleted battery.
   *
   * @param batteryCapacityAmpHours Capacity of the battery in amp-hours (Ah). A typical FRC
   *                                battery is around 18 Ah.
   * @param nominalVoltage          Nominal (fully charged) open circuit voltage of the battery.
   * @param nominalResistance       Nominal internal resistance of the battery.
   */
  static void EnableDischarge(double batteryCapacityAmpHours, wpi::units::volt_t nominalVoltage,
                              wpi::units::ohm_t nominalResistance);

  /**
   * Disable battery discharge simulation, reverting to a constant BatteryVoltage and
   * BatteryResistance.
   */
  static void DisableDischarge();

  /** Reset the simulated battery back to a full charge. */
  static void ResetDischarge();

  /**
   * Get the simulated state of charge of the battery, from 0 (empty) to 1 (full).
   *
   * @return State of charge of the battery.
   */
  static double GetStateOfCharge();

 private:
  static void UpdateDischarge(double totalCurrentAmps);
  static double InterpolateOpenCircuitVoltage(double stateOfCharge);

  static std::unordered_map<const void*, double> m_currents;
  static bool m_dischargeEnabled;
  static double m_batteryCapacityAmpHours;
  static double m_ampHoursUsed;
  static double m_lastTimestampSeconds;
  static std::map<double, double> m_socToVoltage;
};

}  // namespace yams::motorcontrollers::simulation
