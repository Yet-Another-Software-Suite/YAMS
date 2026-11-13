// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <memory>
#include <optional>
#include <vector>

#include <wpi/math/linalg/EigenCore.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/time.hpp>
#include <wpi/units/voltage.hpp>

#include "yams/math/ADRCConfig.hpp"
#include "yams/math/TrackingDifferentiator.hpp"

namespace yams::math {

/**
 * Active disturbance rejection controller.
 *
 * <b>Written by FRC Team 8331</b>, who contributed active disturbance rejection control to YAMS.
 *
 * An active disturbance rejection controller treats every unmodeled effect (friction, gravity, back-EMF, coupling,
 * dropped battery voltage, external loads) as a single "total disturbance" and estimates it online with an extended
 * state observer. The controller then cancels the estimate while driving the modeled part of the plant with a
 * bandwidth-parameterized control law, so it needs very little model: just the system order and an approximate input
 * gain b0.
 *
 * Most teams do not construct this class directly; they describe the controller with ADRCConfig and the motor
 * controller builds and runs it.
 *
 * Plant orders 1 through 4 are supported. The extended state observer is one order higher than the plant, with its
 * final state holding the disturbance estimate. Gains come from a single closed-loop bandwidth wc; the observer
 * bandwidth is 4 * wc.
 *
 * The controller keeps only its running state: the observer's estimate, its last outputs and the tracking
 * differentiator. Everything else it reads from its
 * ADRCConfig every step, including the observer and control gains,
 * which the config works out again whenever its settling time, bandwidth scale or b0 changes. The system order, input
 * delay and tracking differentiator are fixed when the controller is initialized.
 *
 * The controller does not limit or ramp its output. A motor controller running it limits and ramps the output itself,
 * with WithClosedLoopMaxVoltage and WithClosedLoopRampRate, and tells the observer the voltage it applied. Run on its
 * own, clamp the output to what the actuator can apply, ramp it with WPILib's SlewRateLimiter if it needs one, and tell
 * the observer what was applied with Calculate(measurement, setpoint, applied), so a clamped output is not mistaken for
 * a disturbance.
 *
 * @code
 * auto config = std::make_shared<yams::math::ADRCConfig>();
 * config->WithSystemOrder(yams::math::ADRCConfig::SystemOrder::FIRST).WithSettlingTime(200_ms);
 * yams::math::ADRCController adrc{config};
 * // b0, which the config leaves unset, and the control period.
 * adrc.Initialize(b0, 20_ms);
 *
 * // Every loop: clamp the output, and tell the observer what was applied last loop.
 * wpi::units::volt_t volts = std::clamp(adrc.Calculate(measurement, setpoint, appliedVoltage), -12_V, 12_V);
 * @endcode
 */
class ADRCController {
 public:
  /**
   * Create a controller that reads its settings from a config, with the config's system order.
   *
   * @param config Config this controller runs.
   * @throws std::invalid_argument if the config is null or has no system order.
   */
  explicit ADRCController(std::shared_ptr<ADRCConfig> config);

  /**
   * Initialize the controller: give its config the values it cannot hold itself, which also works out its gains, and
   * start the observer and the output at zero.
   *
   * @param b0     Input gain used while the config leaves b0 unset, such as the one ADRCConfig::ResolveB0 derives. Must
   *               be finite and non-zero when used.
   * @param period Control period. Must be finite and positive.
   * @throws std::invalid_argument if a value is invalid, or the tracking differentiator cannot run at this period.
   */
  void Initialize(double b0, wpi::units::second_t period);

  /**
   * Run one control step.
   *
   * @param reference      Reference.
   * @param output         Measured output.
   * @param refDerivatives Reference derivatives; only the first systemOrder entries are used.
   * @return Controller output.
   * @throws std::logic_error if the controller has not been initialized.
   */
  wpi::units::volt_t Step(double reference, double output, const std::vector<double>& refDerivatives = {});

  /**
   * Run one control step, as Step(setpoint, measurement).
   *
   * @param measurement Measured output.
   * @param setpoint    Reference.
   * @return Controller output.
   * @throws std::logic_error if the controller has not been initialized.
   */
  wpi::units::volt_t Calculate(double measurement, double setpoint);

  /**
   * Run one control step, telling the observer what the actuator applied since the last step. A current limit, a low
   * battery or saturation can apply less than asked; told the applied output, the observer does not mistake the
   * shortfall for a disturbance.
   *
   * @param measurement   Measured output.
   * @param setpoint      Reference.
   * @param appliedOutput Output the actuator applied, such as the motor controller's voltage. A non-finite value is
   *                      ignored.
   * @return Controller output.
   * @throws std::logic_error if the controller has not been initialized.
   */
  wpi::units::volt_t Calculate(double measurement, double setpoint, wpi::units::volt_t appliedOutput);

  /**
   * Start the observer at a measurement, with the other states, the disturbance and the output at zero.
   *
   * @param measurement Measured output, in the units the controller is stepped with.
   * @throws std::logic_error      if the controller has not been initialized.
   * @throws std::invalid_argument if the measurement is not finite.
   */
  void Reset(double measurement);

  /**
   * Start the observers at a full state and the output at a value.
   *
   * @param xhatInit Observer state, systemOrder + 1 long; empty starts at zero.
   * @param uInit    Output.
   * @throws std::logic_error      if the controller has not been initialized.
   * @throws std::invalid_argument if the state has the wrong length or the output is not finite.
   */
  void Reset(const std::vector<double>& xhatInit, wpi::units::volt_t uInit);

  /** @return The input gain the gains were worked out with. Throws std::logic_error until initialized. */
  double GetB0() const;
  /** @return The closed-loop bandwidth the config's settling time and bandwidth scale give. */
  wpi::units::radians_per_second_t GetBandwidth() const;
  /** @return The observer bandwidth, four times the closed-loop bandwidth. */
  wpi::units::radians_per_second_t GetObserverBandwidth() const;
  /** @return The config's settling time, or its default. */
  wpi::units::second_t GetSettlingTime() const;
  /** @return The config's bandwidth scale, or its default. */
  double GetBandwidthScale() const;
  /** @return The tracking differentiator, or null when it is off. */
  const TrackingDifferentiator* GetTrackingDifferentiator() const;
  /** @return The observer's state: the measurement, its derivatives, then the disturbance. Throws until initialized. */
  std::vector<double> GetEstimatedStates() const;
  /**
   * @return The estimated total disturbance, in the plant's highest derivative per second. Divide by b0 to see it as an
   *         input. Throws until initialized.
   */
  double GetEstimatedDisturbance() const;
  /** @return The last output. */
  wpi::units::volt_t GetLastOutput() const;
  /** @return The plant order. */
  int GetSystemOrder() const { return m_systemOrder; }
  /** @return The observer order, systemOrder + 1. */
  int GetControllerOrder() const { return m_systemOrder + 1; }
  /** @return The config this controller reads its settings and gains from. */
  const std::shared_ptr<ADRCConfig>& GetConfig() const { return m_config; }
  /** @return Whether Initialize has completed. */
  bool IsInitialized() const { return m_initialized; }

 private:
  friend class ADRCConfig;

  /**
   * The closed-loop bandwidth a settling time and bandwidth scale give.
   *
   * @param systemOrder    Plant order.
   * @param bandwidthScale Bandwidth scale.
   * @param settlingTime   Settling time.
   * @return Bandwidth, radians per second.
   */
  static double Bandwidth(int systemOrder, double bandwidthScale, wpi::units::second_t settlingTime);

  /**
   * Work out the observer and controller gains.
   *
   * @param systemOrder Plant order, 1 through 4.
   * @param wc          Closed-loop bandwidth, radians per second.
   * @param b0          Input gain.
   * @param dt          Control period, seconds.
   * @return The gains.
   */
  static ADRCGains ComputeGains(int systemOrder, double wc, double b0, double dt);

  /** Throw std::logic_error unless Initialize has completed. */
  void RequireInitialized() const;

  std::shared_ptr<ADRCConfig> m_config;
  int m_systemOrder;
  Eigen::VectorXd m_xhat;
  Eigen::VectorXd m_secondaryXhat;
  double m_uPrev{0.0};
  std::vector<double> m_uHistory;
  std::optional<TrackingDifferentiator> m_td;
  bool m_initialized{false};
};

}  // namespace yams::math
