// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <wpi/math/controller/LinearQuadraticRegulator.hpp>
#include <wpi/math/estimator/KalmanFilter.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/math/system/LinearSystem.hpp>
#include <wpi/math/system/LinearSystemLoop.hpp>
#include <wpi/math/system/Models.hpp>
#include <wpi/units/time.hpp>
#include <wpi/units/voltage.hpp>

#include <optional>
#include <variant>
#include <vector>

namespace yams::math {

/**
 * Configuration for an LQR (Linear Quadratic Regulator) closed-loop controller.
 *
 * Uses a fluent builder pattern; all With* methods return *this for chaining.
 * After configuring the plant type, gains, and noise parameters call GetLoop() to obtain
 * the ready-to-use LinearSystemLoop.
 */
class LQRConfig {
 public:
  /** Identifies which plant model the LQR is built for. */
  enum class LQRType {
    FLYWHEEL,  ///< Flywheel (1-state velocity model).
    ARM,       ///< Single-jointed arm (2-state position+velocity model).
    ELEVATOR,  ///< Elevator (2-state position+velocity model).
  };

  /** LinearSystemLoop type alias for 1-state plants (flywheel). */
  using Loop1 = wpi::math::LinearSystemLoop<1, 1, 1>;
  /** LinearSystemLoop type alias for 2-state plants (arm, elevator). */
  using Loop2 = wpi::math::LinearSystemLoop<2, 1, 1>;

  /**
   * Set the LQR plant type explicitly.
   *
   * @param type LQRType to use.
   * @return *this for chaining.
   */
  LQRConfig& WithType(LQRType type);

  /**
   * Configure a flywheel plant model.
   *
   * @param motor          DC motor model.
   * @param momentOfInertia Moment of inertia of the flywheel (kg·m²).
   * @param gearing        Gear reduction from motor to flywheel.
   * @return *this for chaining.
   */
  LQRConfig& WithFlywheelSystem(const wpi::math::DCMotor& motor, double momentOfInertia, double gearing);

  /**
   * Configure a single-jointed arm plant model.
   *
   * @param motor          DC motor model.
   * @param momentOfInertia Moment of inertia of the arm (kg·m²).
   * @param gearing        Gear reduction from motor to arm joint.
   * @return *this for chaining.
   */
  LQRConfig& WithArmSystem(const wpi::math::DCMotor& motor, double momentOfInertia, double gearing);

  /**
   * Configure an elevator plant model.
   *
   * @param motor      DC motor model.
   * @param mass       Carriage mass (kg).
   * @param drumRadius Radius of the drum (m).
   * @param gearing    Gear reduction from motor to drum.
   * @return *this for chaining.
   */
  LQRConfig& WithElevatorSystem(const wpi::math::DCMotor& motor, double mass, double drumRadius,
                                double gearing);

  /**
   * Set the Q matrix diagonal elements (state error tolerances).
   *
   * Decrease values to more aggressively penalize state excursion.
   *
   * @param qElems Q-matrix diagonal entries (one per state).
   * @return *this for chaining.
   */
  LQRConfig& WithQElems(std::initializer_list<double> qElems);

  /**
   * Set the R matrix diagonal elements (control effort tolerances).
   *
   * Decrease values to more aggressively penalize control effort.
   * A good starting value is 12 (approximately the maximum battery voltage).
   *
   * @param rElems R-matrix diagonal entries (one per input).
   * @return *this for chaining.
   */
  LQRConfig& WithRElems(std::initializer_list<double> rElems);

  /**
   * Set the Kalman filter model standard deviations.
   *
   * @param stateStdDevs Standard deviations of the model states.
   * @return *this for chaining.
   */
  LQRConfig& WithStateStdDevs(std::initializer_list<double> stateStdDevs);

  /**
   * Set the Kalman filter measurement standard deviations.
   *
   * @param measStdDevs Standard deviations of the encoder measurements.
   * @return *this for chaining.
   */
  LQRConfig& WithMeasurementStdDevs(std::initializer_list<double> measStdDevs);

  /**
   * Set the maximum voltage output of the LinearSystemLoop.
   *
   * @param maxVoltage Maximum voltage (default 12 V).
   * @return *this for chaining.
   */
  LQRConfig& WithMaxVoltage(wpi::units::volt_t maxVoltage);

  /**
   * Set the loop period for the LQR.
   *
   * @param period Loop period (default 20 ms).
   * @return *this for chaining.
   */
  LQRConfig& WithPeriod(wpi::units::second_t period);

  /**
   * Get the configured LQR plant type.
   *
   * @return LQRType.
   */
  LQRType GetType() const;

  /**
   * Get the configured loop period.
   *
   * @return Loop period.
   */
  wpi::units::second_t GetPeriod() const;

  /**
   * Get the configured maximum voltage.
   *
   * @return Maximum voltage.
   */
  wpi::units::volt_t GetMaxVoltage() const;

  /**
   * Build and return the linear plant model.
   *
   * @return Variant holding a 1-state system (flywheel) or 2-state system (arm/elevator).
   */
  std::variant<wpi::math::LinearSystem<1, 1, 1>, wpi::math::LinearSystem<2, 1, 1>> GetSystem() const;

  /**
   * Build and return the complete LinearSystemLoop ready for use.
   *
   * @return Variant holding Loop1 (flywheel) or Loop2 (arm/elevator).
   */
  std::variant<Loop1, Loop2> GetLoop() const;

 private:
  friend class LQRController;

  std::optional<LQRType> m_type;
  std::optional<wpi::math::LinearSystem<1, 1, 1>> m_flywheelPlant;
  std::optional<wpi::math::LinearSystem<2, 1, 1>> m_armElevatorPlant;

  std::vector<double> m_qElems;
  std::vector<double> m_rElems;
  std::vector<double> m_stateStdDevs;
  std::vector<double> m_measStdDevs;

  wpi::units::volt_t m_maxVoltage{wpi::units::volt_t{12.0}};
  wpi::units::second_t m_period{wpi::units::second_t{0.020}};
};

}  // namespace yams::math
