// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <wpi/math/linalg/EigenCore.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/units/time.hpp>
#include <wpi/units/voltage.hpp>
#include <wpi/util/Alert.hpp>

#include "yams/math/TrackingDifferentiator.hpp"

namespace yams::gearing {
class MechanismGearing;
}  // namespace yams::gearing

namespace yams::motorcontrollers {
class SmartMotorControllerConfig;
}  // namespace yams::motorcontrollers

namespace yams::math {

class ADRCController;

/** A rate of change of voltage, for a standalone ADRCController's output slew-rate limit. */
using volts_per_second_t = wpi::units::unit_t<
    wpi::units::compound_unit<wpi::units::volts, wpi::units::inverse<wpi::units::seconds>>>;

/**
 * Observer and control gains worked out from an ADRCConfig. Never changed once made: the config replaces the whole
 * object, so a step never uses a mix of old and new gains.
 */
struct ADRCGains {
  Eigen::VectorXd ld;  ///< ESO observer gains.
  Eigen::MatrixXd ad;  ///< Discrete ESO state matrix.
  Eigen::VectorXd bd;  ///< Discrete ESO input matrix.
  Eigen::VectorXd k;   ///< Controller gains.
  double b0{1.0};      ///< Input gain the matrices were worked out with, which the control law divides by.
};

/**
 * Configuration for an active disturbance rejection controller.
 *
 * <b>Written by FRC Team 8331</b>, who contributed active disturbance rejection control to YAMS.
 *
 * An active disturbance rejection controller lumps everything the mechanism model does not capture (friction,
 * back-EMF, game-piece loads, dropped battery voltage, unmodeled dynamics) into a single "total disturbance" and
 * estimates it live with an extended state observer. The estimate is cancelled out of the control law, so the closed
 * loop behaves like a clean chain of integrators that settles in the time you ask for. There is no proportional,
 * integral or derivative gain to tune: you describe the plant (order and input gain) and the response you want
 * (settling time), and the controller derives its own gains.
 *
 * This is the drop-in equivalent of a PID: pass it to SmartMotorControllerConfig::WithADRCController and the motor
 * controller runs the controller on its closed loop thread.
 *
 * @code
 * config.WithADRCController(yams::math::ADRCConfig{}
 *     .WithSystemOrder(yams::math::ADRCConfig::SystemOrder::FIRST)
 *     .WithSettlingTime(200_ms));
 * @endcode
 *
 * The output limit and ramp come from the motor config, as for every closed loop controller:
 * WithClosedLoopMaxVoltage limits the output, and WithClosedLoopRampRate limits how quickly it changes.
 *
 * Every value starts empty. The getters return that empty optional, and the controller fills it with the default
 * documented on the With method. The system order has no default: BuildController throws until it is set.
 *
 * The With methods only accept values the controller can use: anything else throws std::invalid_argument and leaves
 * the config as it was. The controller reads its settings from this config every step instead of keeping copies, so
 * the config never holds a value it cannot run.
 *
 * Once a controller is built from it, the config also holds what the controller needs that the settings do not give:
 * the control period, the b0 used while it is unset, and the gains worked out from them. The
 * config works the gains out again whenever its settling time, bandwidth scale or b0 changes, so every live-tunable
 * value takes effect on the controller's next step. The system order, input delay and tracking differentiator only
 * take effect when a controller is built, and a built config refuses a different system order.
 *
 * Copies are deep: a copy has its own settings, built values and gains, and raises its own alerts. Every method is safe
 * to call while the controller's thread runs.
 */
class ADRCConfig {
 public:
  /**
   * Plant order: how many integrators sit between the motor voltage and the measured output. Velocity loops on a DC
   * motor are FIRST, position loops SECOND. An elevator's position is FIRST too, with its speed per volt as b0.
   */
  enum class SystemOrder {
    FIRST = 1,     ///< First order: the loop commands and measures velocity, such as a flywheel.
    FLYWHEEL = 1,  ///< A flywheel's velocity loop; the same as FIRST.
    SECOND = 2,    ///< Second order: the loop commands and measures position, such as an arm.
    ARM = 2,       ///< An arm's position loop; the same as SECOND.
    ELEVATOR = 2,  ///< An elevator's position loop; the same as SECOND.
    THIRD = 3,     ///< Third order: the measurement sits two integrations behind the voltage. Rare.
    FOURTH = 4,    ///< Fourth order: three integrations between the voltage and the measurement. Rare.
  };

  /**
   * Get a system order as an integer.
   *
   * @param order System order.
   * @return Plant order, 1 through 4.
   */
  static constexpr int GetOrder(SystemOrder order) { return static_cast<int>(order); }

  /** Closed-loop settling time used when none is set. */
  static constexpr wpi::units::second_t kDefaultSettlingTime{0.2};
  /** Bandwidth scale used when none is set. */
  static constexpr double kDefaultBandwidthScale{1.0};
  /** Input delay used when none is set; zero turns it off. */
  static constexpr wpi::units::second_t kDefaultInputDelay{0.0};

  /** Create a configuration. It belongs to the motor controller it is given to, whose telemetry name its errors use. */
  ADRCConfig();
  /**
   * Copy a configuration deeply: its settings, the values given when a controller was built from it, and its own copy
   * of the gains. The copy raises its own alerts.
   *
   * @param other Configuration to copy.
   */
  ADRCConfig(const ADRCConfig& other);
  /**
   * Copy a configuration deeply, as the copy constructor does.
   *
   * @param other Configuration to copy.
   * @return This config.
   */
  ADRCConfig& operator=(const ADRCConfig& other);
  ~ADRCConfig();

  /**
   * Set the plant order. There is no default; the controller cannot be built without it.
   *
   * @param order Plant order.
   * @return This config for chaining.
   * @throws std::logic_error if a controller was built from this config with a different order.
   */
  ADRCConfig& WithSystemOrder(SystemOrder order);
  /**
   * Set the desired closed-loop settling time. Defaults to kDefaultSettlingTime.
   *
   * @param settlingTime Settling time; must be finite and positive.
   * @return This config for chaining.
   * @throws std::invalid_argument if the settling time is not finite and positive.
   */
  ADRCConfig& WithSettlingTime(wpi::units::second_t settlingTime);
  /**
   * Set the closed-loop bandwidth scale. Defaults to kDefaultBandwidthScale.
   *
   * @param bandwidthScale Bandwidth scale; must be finite and positive. Larger is more aggressive.
   * @return This config for chaining.
   * @throws std::invalid_argument if the bandwidth scale is not finite and positive.
   */
  ADRCConfig& WithBandwidthScale(double bandwidthScale);
  /**
   * Set the input gain instead of deriving it.
   *
   * @param b0 Input gain in mechanism units per second squared per volt; must be finite and non-zero.
   * @return This config for chaining.
   * @throws std::invalid_argument if b0 is not finite or is zero.
   */
  ADRCConfig& WithB0(double b0);
  /**
   * Set the input gain to the mechanism's speed per volt: the motor's free speed per volt, through the gearing. This is
   * the b0 of a FIRST order loop on position for a mechanism whose motor's back-EMF, more than its inertia, sets how fast
   * it moves, such as an elevator. It is in mechanism rotations per second per volt, so it fits a closed loop in
   * rotations, not a linear one.
   *
   * @param motor   Motor driving the mechanism, with every motor in its gearbox.
   * @param gearing Gearing from the motor to the mechanism.
   * @return This config for chaining.
   * @throws std::invalid_argument if the speed per volt is not finite or is zero.
   */
  ADRCConfig& WithB0(const wpi::math::DCMotor& motor, const gearing::MechanismGearing& gearing);
  /**
   * Set the input delay, rounded to whole control periods. Defaults to kDefaultInputDelay, which turns it off.
   *
   * @param inputDelay Input delay; must be finite and not negative.
   * @return This config for chaining.
   * @throws std::invalid_argument if the delay is not finite or is negative.
   */
  ADRCConfig& WithInputDelay(wpi::units::second_t inputDelay);
  /**
   * Select the cascaded extended state observer: a second observer, fed the first one's estimate of the measurement,
   * supplies the disturbance the control law cancels. Off by default.
   *
   * @param cascaded Whether to run the cascaded observer.
   * @return This config for chaining.
   */
  ADRCConfig& WithCascaded(bool cascaded);
  /**
   * Smooth the reference with a tracking differentiator. Off by default. Its parameters are checked when the
   * controller is built, as some depend on the period.
   *
   * @param method Differentiation method.
   * @param params Method parameters in order; empty for the method defaults.
   * @return This config for chaining.
   */
  ADRCConfig& WithTrackingDifferentiator(TrackingDifferentiator::Method method, std::vector<double> params = {});
  /**
   * Turn the tracking differentiator off.
   *
   * @return This config for chaining.
   */
  ADRCConfig& WithoutTrackingDifferentiator();

  /** @return The plant order, or empty when it has not been set. */
  std::optional<SystemOrder> GetSystemOrder() const;
  /** @return The settling time, or empty for kDefaultSettlingTime. */
  std::optional<wpi::units::second_t> GetSettlingTime() const;
  /** @return The bandwidth scale, or empty for kDefaultBandwidthScale. */
  std::optional<double> GetBandwidthScale() const;
  /** @return The input gain, or empty when it should be derived. */
  std::optional<double> GetB0() const;
  /** @return The input delay, or empty for kDefaultInputDelay. */
  std::optional<wpi::units::second_t> GetInputDelay() const;
  /** @return Whether the cascaded observer is on. */
  bool IsCascaded() const;
  /** @return The tracking differentiator method, or empty when it is off. */
  std::optional<TrackingDifferentiator::Method> GetTdMethod() const;
  /** @return The tracking differentiator parameters; empty for the method defaults. */
  std::vector<double> GetTdParams() const;

  /**
   * Resolve the input gain the controller will use: the one set, or one derived from the motor, the gearing and the
   * mechanism's moment of inertia, only when the mechanism set that inertia.
   *
   * @param motor  Motor model.
   * @param config Motor config with the gearing, inertia and circumference.
   * @return Input gain in mechanism units per second squared per volt.
   * @throws exceptions::SmartMotorControllerConfigurationException if b0 is not set and cannot be derived.
   */
  double ResolveB0(const wpi::math::DCMotor& motor, const motorcontrollers::SmartMotorControllerConfig& config) const;

  /**
   * Build an initialized controller that reads this config, giving the config the period and b0 it needs. The
   * controller saturates at the motor config's closed loop maximum voltage, which the motor controller limits its
   * output to as well, and has no slew-rate limit of its own: the motor controller ramps its output with
   * WithClosedLoopRampRate. The motor controller calls this; you only need it to use the controller yourself.
   *
   * @param self   This config, shared with the controller, which reads it every step.
   * @param motor  Motor model, for deriving b0 and the nominal voltage fallback.
   * @param config Motor config with the gearing, inertia, circumference and closed loop voltage limit.
   * @param period Closed loop control period.
   * @return The initialized controller.
   * @throws exceptions::SmartMotorControllerConfigurationException if the order is missing or b0 cannot be derived.
   * @throws std::invalid_argument if the controller rejects the derived b0, the motor config's limit or the tracking
   *         differentiator.
   */
  static ADRCController BuildController(const std::shared_ptr<ADRCConfig>& self, const wpi::math::DCMotor& motor,
                                        const motorcontrollers::SmartMotorControllerConfig& config,
                                        wpi::units::second_t period);

  /**
   * Report a value this config refused, as live tuning does when a tuned value throws. The alert lives on the config,
   * so it outlasts a controller rebuilt from it.
   *
   * @param reason Why the value was refused, or empty once a tuned value is accepted.
   */
  void ReportRejected(std::optional<std::string> reason);

 private:
  friend class ADRCController;

  /** Everything a step reads, read together so a step sees one consistent config. */
  struct StepValues {
    std::shared_ptr<const ADRCGains> gains;
    double period;
    bool cascaded;
  };

  /** Give the config the values a built controller needs, and work out the gains; throws std::invalid_argument. */
  void SetBuiltValues(double b0, wpi::units::second_t period);
  /** Work the gains out again, once built. The caller holds the lock. */
  void RebuildGains();
  /** @return What a step reads. */
  StepValues GetStepValues() const;
  /** @return The gains, or null until a controller is built. */
  std::shared_ptr<const ADRCGains> GetGains() const;
  /** Resolve the maximum output from the motor config, raising an alert for the nominal voltage fallback. */
  wpi::units::volt_t ResolveMaxVoltage(const wpi::math::DCMotor& motor,
                                       const motorcontrollers::SmartMotorControllerConfig& config);
  /** @return An alert id of this config's own. */
  std::string AlertId(const std::string& alert) const;

  mutable std::mutex m_mutex;
  std::string m_owner{"ADRC"};
  std::optional<SystemOrder> m_systemOrder;
  std::optional<wpi::units::second_t> m_settlingTime;
  std::optional<double> m_bandwidthScale;
  std::optional<double> m_b0;
  std::optional<wpi::units::second_t> m_inputDelay;
  bool m_cascaded{false};
  std::optional<TrackingDifferentiator::Method> m_tdMethod;
  std::vector<double> m_tdParams;

  std::optional<wpi::units::second_t> m_builtPeriod;
  std::optional<double> m_builtB0;
  std::shared_ptr<const ADRCGains> m_gains;

  std::unique_ptr<wpi::util::Alert> m_nominalVoltageAlert;
  std::unique_ptr<wpi::util::Alert> m_rejectedAlert;
  int m_instance;
};

}  // namespace yams::math
