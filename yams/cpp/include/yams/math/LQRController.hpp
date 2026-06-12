// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/system/LinearSystem.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <memory>
#include <optional>
#include <utility>
#include <variant>

#include "LQRConfig.hpp"

namespace yams::math {

/**
 * Linear Quadratic Regulator controller wrapper.
 *
 * Wraps a WPILib LinearSystemLoop to provide a common Calculate() interface for
 * flywheel (velocity), arm, and elevator (position + velocity) plant types.
 *
 * Internally the LQR gain matrix and Kalman filter are heap-allocated alongside
 * the LinearSystemLoop so that the loop's internal raw pointers remain valid for
 * the lifetime of this object. Never copy or std::move an LQRController while
 * it is in use.
 */
class LQRController {
 public:
  /**
   * Construct an LQRController by building the loop from an LQRConfig.
   *
   * @param config Fully-configured LQRConfig.
   */
  explicit LQRController(LQRConfig config);

  /**
   * Replace the current LQR configuration and rebuild the internal loop.
   *
   * @param config New LQRConfig to apply.
   */
  void UpdateConfig(LQRConfig config);

  /**
   * Reset the angular position+velocity state of the controller (arm/elevator).
   *
   * @param angle    Current mechanism angle.
   * @param velocity Current mechanism velocity.
   */
  void Reset(units::radian_t angle, units::radians_per_second_t velocity);

  /**
   * Reset the linear position+velocity state of the controller (elevator).
   *
   * @param distance Current mechanism distance.
   * @param velocity Current mechanism linear velocity.
   */
  void Reset(units::meter_t distance, units::meters_per_second_t velocity);

  /**
   * Calculate the next voltage output for an angular positional mechanism (arm).
   *
   * @param measured  Current measured angle.
   * @param position  Target angle setpoint.
   * @param velocity  Target angular velocity setpoint.
   * @return Voltage to apply to the motor.
   */
  units::volt_t Calculate(units::radian_t measured, units::radian_t position,
                          units::radians_per_second_t velocity);

  /**
   * Calculate the next voltage output for a linear positional mechanism (elevator).
   *
   * @param measured  Current measured distance.
   * @param position  Target distance setpoint.
   * @param velocity  Target linear velocity setpoint.
   * @return Voltage to apply to the motor.
   */
  units::volt_t Calculate(units::meter_t measured, units::meter_t position,
                          units::meters_per_second_t velocity);

  /**
   * Calculate the next voltage output for an angular velocity mechanism (flywheel).
   *
   * @param measured  Current measured angular velocity.
   * @param velocity  Target angular velocity setpoint.
   * @return Voltage to apply to the motor.
   */
  units::volt_t Calculate(units::radians_per_second_t measured,
                          units::radians_per_second_t velocity);

  /**
   * Calculate the next voltage output for a linear velocity mechanism.
   *
   * @param measured  Current measured linear velocity.
   * @param velocity  Target linear velocity setpoint.
   * @return Voltage to apply to the motor.
   */
  units::volt_t Calculate(units::meters_per_second_t measured, units::meters_per_second_t velocity);

  /**
   * Get the configured LQR plant type.
   *
   * @return LQRType.
   */
  LQRConfig::LQRType GetType() const;

  /**
   * Get the optional LQRConfig used to construct this controller.
   *
   * @return Reference to the optional config.
   */
  const std::optional<LQRConfig>& GetConfig() const;

 private:
  // frc::LinearSystemLoop stores raw pointers to the LQR and Kalman filter passed to its
  // constructor. These bundles heap-pin all three together so the pointers are never dangled.
  struct FlywheelBundle {
    frc::LinearQuadraticRegulator<1, 1> controller;
    frc::KalmanFilter<1, 1, 1> observer;
    LQRConfig::Loop1 loop;

    FlywheelBundle(frc::LinearSystem<1, 1, 1> plant, frc::LinearQuadraticRegulator<1, 1> ctrl,
                   frc::KalmanFilter<1, 1, 1> obs, units::volt_t maxV, units::second_t dt)
        : controller{std::move(ctrl)},
          observer{std::move(obs)},
          loop{plant, controller, observer, maxV, dt} {}

    FlywheelBundle(const FlywheelBundle&) = delete;
    FlywheelBundle& operator=(const FlywheelBundle&) = delete;
    FlywheelBundle(FlywheelBundle&&) = delete;
    FlywheelBundle& operator=(FlywheelBundle&&) = delete;
  };

  struct ArmElevatorBundle {
    frc::LinearQuadraticRegulator<2, 1> controller;
    frc::KalmanFilter<2, 1, 1> observer;
    LQRConfig::Loop2 loop;

    ArmElevatorBundle(frc::LinearSystem<2, 1, 1> plant, frc::LinearQuadraticRegulator<2, 1> ctrl,
                      frc::KalmanFilter<2, 1, 1> obs, units::volt_t maxV, units::second_t dt)
        : controller{std::move(ctrl)},
          observer{std::move(obs)},
          loop{plant, controller, observer, maxV, dt} {}

    ArmElevatorBundle(const ArmElevatorBundle&) = delete;
    ArmElevatorBundle& operator=(const ArmElevatorBundle&) = delete;
    ArmElevatorBundle(ArmElevatorBundle&&) = delete;
    ArmElevatorBundle& operator=(ArmElevatorBundle&&) = delete;
  };

  static std::unique_ptr<FlywheelBundle> BuildFlywheel(const LQRConfig& cfg);
  static std::unique_ptr<ArmElevatorBundle> BuildArmElevator(const LQRConfig& cfg);

  std::optional<LQRConfig> m_config;
  LQRConfig::LQRType m_type;
  std::variant<std::unique_ptr<FlywheelBundle>, std::unique_ptr<ArmElevatorBundle>> m_bundle;
  units::second_t m_period;
};

}  // namespace yams::math
