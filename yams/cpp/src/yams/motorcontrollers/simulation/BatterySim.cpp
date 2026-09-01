// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/simulation/BatterySim.hpp"

#include <wpi/system/Timer.hpp>
#include <wpi/simulation/BatterySim.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace yams::motorcontrollers::simulation {

wpi::units::volt_t BatterySim::BatteryVoltage = wpi::units::volt_t{12.0};
wpi::units::ohm_t BatterySim::BatteryResistance = wpi::units::ohm_t{0.020};

std::unordered_map<const void*, double> BatterySim::m_currents{};
bool BatterySim::m_dischargeEnabled = false;
double BatterySim::m_batteryCapacityAmpHours = 18.0;
double BatterySim::m_ampHoursUsed = 0.0;
double BatterySim::m_lastTimestampSeconds = std::numeric_limits<double>::quiet_NaN();

// Open circuit voltage of the battery as a function of state of charge (0 to 1), based on a
// typical FRC sealed lead-acid battery discharge curve. Voltage stays relatively flat for most
// of the discharge before sagging quickly near depletion. Replaceable via ReplaceSOCInterpolation.
std::map<double, double> BatterySim::m_socToVoltage{
    {0.00, 9.0},
    {0.05, 10.5},
    {0.10, 11.5},
    {0.20, 12.0},
    {0.40, 12.3},
    {0.60, 12.5},
    {0.80, 12.7},
    {0.90, 12.8},
    {1.00, 12.9},
};

void BatterySim::ReplaceSOCInterpolation(const std::map<double, double>& socToVoltage) {
  m_socToVoltage = socToVoltage;
}

double BatterySim::InterpolateOpenCircuitVoltage(double stateOfCharge) {
  stateOfCharge = std::clamp(stateOfCharge, 0.0, 1.0);
  if (m_socToVoltage.empty()) {
    return BatteryVoltage.value();
  }

  auto hi = m_socToVoltage.lower_bound(stateOfCharge);
  if (hi == m_socToVoltage.begin()) {
    return hi->second;
  }
  if (hi == m_socToVoltage.end()) {
    return std::prev(hi)->second;
  }

  auto lo = std::prev(hi);
  double t = (hi->first > lo->first) ? (stateOfCharge - lo->first) / (hi->first - lo->first) : 0.0;
  return lo->second + t * (hi->second - lo->second);
}

void BatterySim::EnableDischarge(double batteryCapacityAmpHours, wpi::units::volt_t nominalVoltage,
                                 wpi::units::ohm_t nominalResistance) {
  m_dischargeEnabled = true;
  m_batteryCapacityAmpHours = batteryCapacityAmpHours;
  BatteryVoltage = nominalVoltage;
  BatteryResistance = nominalResistance;
}

void BatterySim::DisableDischarge() { m_dischargeEnabled = false; }

void BatterySim::ResetDischarge() {
  m_ampHoursUsed = 0.0;
  m_lastTimestampSeconds = std::numeric_limits<double>::quiet_NaN();
}

double BatterySim::GetStateOfCharge() {
  return std::clamp(1.0 - (m_ampHoursUsed / m_batteryCapacityAmpHours), 0.0, 1.0);
}

void BatterySim::UpdateDischarge(double totalCurrentAmps) {
  double now = wpi::Timer::GetTimestamp().value();
  if (!std::isnan(m_lastTimestampSeconds)) {
    double dtHours = (now - m_lastTimestampSeconds) / 3600.0;
    if (dtHours > 0) {
      m_ampHoursUsed += totalCurrentAmps * dtHours;
      m_ampHoursUsed = std::clamp(m_ampHoursUsed, 0.0, m_batteryCapacityAmpHours);
    }
  }
  m_lastTimestampSeconds = now;
}

wpi::units::volt_t BatterySim::CalculateVoltage(const void* id, wpi::units::ampere_t current) {
  m_currents[id] = current.value();

  std::vector<wpi::units::ampere_t> currentDraws;
  currentDraws.reserve(m_currents.size());
  double totalCurrentAmps = 0;
  for (const auto& [key, amps] : m_currents) {
    currentDraws.push_back(wpi::units::ampere_t{amps});
    totalCurrentAmps += amps;
  }

  double openCircuitVoltage = BatteryVoltage.value();
  double internalResistance = BatteryResistance.value();
  if (m_dischargeEnabled) {
    UpdateDischarge(totalCurrentAmps);

    double stateOfCharge = GetStateOfCharge();
    openCircuitVoltage = InterpolateOpenCircuitVoltage(stateOfCharge);
    // Internal resistance rises as the battery depletes, exaggerating voltage sag under load.
    internalResistance *= 1.0 + (2.0 * (1.0 - stateOfCharge));
  }

  return wpi::sim::BatterySim::Calculate(wpi::units::volt_t{openCircuitVoltage},
                                         wpi::units::ohm_t{internalResistance}, currentDraws);
}

}  // namespace yams::motorcontrollers::simulation
