// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <wpi/math/geometry/Translation3d.hpp>
#include <wpi/simulation/DCMotorSim.hpp>
#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/commands2/button/Trigger.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/velocity.hpp>

#include <functional>
#include <optional>
#include <string>

#include "SmartVelocityMechanism.hpp"
#include "yams/mechanisms/config/FlyWheelConfig.hpp"

namespace yams::mechanisms::velocity {

/**
 * Smart mechanism implementation for a flywheel (velocity-controlled roller).
 *
 * Drives a spinning element using a SmartMotorController configured for
 * mechanism-space (degrees/second) closed-loop velocity control.  If a roller
 * diameter is provided, surface-speed commands and reads are also supported.
 *
 * ### Example usage (inside a subsystem constructor)
 * @code{.cpp}
 * using namespace yams::motorcontrollers;
 * using namespace yams::motorcontrollers::remote;
 * using namespace yams::gearing;
 * using namespace yams::mechanisms::velocity;
 * using Cfg = SmartMotorControllerConfig;
 *
 * // Declare as subsystem members:
 * //   ctre::phoenix6::hardware::TalonFX m_talon{4};
 * //   std::optional<TalonFXWrapper>     m_motor;
 * //   FlyWheelConfig                     m_flyWheelConfig;
 * //   std::optional<FlyWheel>            m_flyWheel;
 *
 * SmartMotorControllerConfig motorCfg;
 * motorCfg.WithSubsystem(this)
 *         .WithFeedback(100.0, 0.0, 0.0)
 *         .WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({3.0, 4.0})})
 *         .WithIdleMode(Cfg::MotorMode::COAST)
 *         .WithStatorCurrentLimit(60.0_A)
 *         .WithMotorInverted(false)
 *         .WithFeedforward(wpi::math::SimpleMotorFeedforward<wpi::units::turns>{
 *             0.0_V, wpi::units::unit_t<wpi::math::SimpleMotorFeedforward<wpi::units::turns>::kv_unit>{1.0},
 *             wpi::units::unit_t<wpi::math::SimpleMotorFeedforward<wpi::units::turns>::ka_unit>{0.0}})
 *         .WithClosedLoopMode()
 *         .WithTelemetry("ShooterMotor", Cfg::TelemetryVerbosity::HIGH);
 *
 * m_motor.emplace(m_talon, wpi::math::DCMotor::Falcon500(1), motorCfg);
 *
 * m_flyWheelConfig.WithRollerDiameter(4.0 * 0.0254_m)  // 4-inch roller
 *                 .WithTelemetryName("Shooter");
 *
 * m_flyWheel.emplace(&m_flyWheelConfig, &m_motor.value());
 *
 * // Run at a fixed angular velocity (~2000 RPM):
 * m_flyWheel->Run(wpi::units::degrees_per_second_t{12000.0});
 *
 * // Or run at a surface speed (requires roller diameter):
 * m_flyWheel->Run(10.0_mps);
 * @endcode
 */
class FlyWheel : public SmartVelocityMechanism {
 public:
  /**
   * Construct a FlyWheel from a FlyWheelConfig and a SmartMotorController.
   *
   * @param config Pointer to the flywheel configuration (must outlive this object).
   * @param smc    Pointer to the motor controller (must outlive this object).
   */
  FlyWheel(config::FlyWheelConfig* config, motorcontrollers::SmartMotorController* smc);

  // ---- SmartMechanism overrides ---------------------------------------------

  /** Advance the flywheel's simulation by one loop iteration. */
  void SimIterate() override;

  /** Publish flywheel telemetry to NetworkTables / SmartDashboard. */
  void UpdateTelemetry() override;

  /** Update the Mechanism2d ligament to reflect the current velocity. */
  void VisualizationUpdate() override;

  /** Get the human-readable name of this flywheel. */
  std::string GetName() const override;

  // ---- SmartVelocityMechanism overrides -------------------------------------

  /**
   * Trigger that becomes true when the flywheel is at or above its maximum
   * configured velocity.
   *
   * @return Trigger for the upper velocity limit.
   */
  wpi::cmd::Trigger Max() override;

  /**
   * Trigger that becomes true when the flywheel is at or below its minimum
   * configured velocity (e.g. stalled or reversed).
   *
   * @return Trigger for the lower velocity limit.
   */
  wpi::cmd::Trigger Min() override;

  // ---- FlyWheel-specific interface ------------------------------------------

  /**
   * Get the current angular velocity of the flywheel.
   *
   * @return Current mechanism velocity in degrees per second.
   */
  wpi::units::degrees_per_second_t GetVelocity() const;

  // ---- Run / RunTo interface ------------------------------------------------

  /**
   * Set the flywheel to the given angular velocity.
   *
   * @param velocity FlyWheel angular velocity to go to.
   * @return CommandPtr that sets the flywheel to the desired speed.
   */
  wpi::cmd::CommandPtr Run(wpi::units::degrees_per_second_t velocity);

  /**
   * Set the flywheel to the given angular velocity via a supplier.
   *
   * @param velocity Supplier returning the desired angular velocity each loop.
   * @return CommandPtr that sets the flywheel to the desired speed.
   */
  wpi::cmd::CommandPtr Run(std::function<wpi::units::degrees_per_second_t()> velocity);

  /**
   * Set the flywheel to the given surface speed.
   *
   * Requires that a roller diameter was provided in the config.
   *
   * @param surfaceSpeed Desired surface speed in metres per second.
   * @return CommandPtr that sets the flywheel to the desired surface speed.
   */
  wpi::cmd::CommandPtr Run(wpi::units::meters_per_second_t surfaceSpeed);

  /**
   * Set the flywheel to the given surface speed via a supplier.
   *
   * Requires that a roller diameter was provided in the config.
   *
   * @param surfaceSpeed Supplier returning the desired surface speed each loop.
   * @return CommandPtr that sets the flywheel to the desired surface speed.
   */
  wpi::cmd::CommandPtr Run(std::function<wpi::units::meters_per_second_t()> surfaceSpeed);

  /**
   * Run the flywheel to an angular velocity within a tolerance, then end the command.
   *
   * @param velocity  Target angular velocity.
   * @param tolerance Allowable error.
   * @return CommandPtr that ends once the flywheel is near the target velocity.
   * @note Do not use with a default command on the subsystem, as it will override the setting after
   * this ends.
   */
  wpi::cmd::CommandPtr RunTo(wpi::units::degrees_per_second_t velocity,
                         wpi::units::degrees_per_second_t tolerance = wpi::units::degrees_per_second_t{5.0});

  /**
   * Run the flywheel to a supplier-provided angular velocity within a tolerance, then end the
   * command.
   *
   * The supplier is evaluated once when the command is created.
   *
   * @param velocity  Supplier returning the target angular velocity.
   * @param tolerance Allowable error.
   * @return CommandPtr that ends once the flywheel is near the target velocity.
   * @note Do not use with a default command on the subsystem, as it will override the setting after
   * this ends.
   */
  wpi::cmd::CommandPtr RunTo(std::function<wpi::units::degrees_per_second_t()> velocity,
                         wpi::units::degrees_per_second_t tolerance = wpi::units::degrees_per_second_t{5.0});

  /**
   * Run the flywheel to a surface speed within a tolerance, then end the command.
   *
   * Requires that a roller diameter was provided in the config.
   *
   * @param velocity  Target surface speed.
   * @param tolerance Allowable error.
   * @return CommandPtr that ends once the flywheel is near the target surface speed.
   * @note Do not use with a default command on the subsystem, as it will override the setting after
   * this ends.
   */
  wpi::cmd::CommandPtr RunTo(wpi::units::meters_per_second_t velocity, wpi::units::meters_per_second_t tolerance);

  /**
   * Run the flywheel to a supplier-provided surface speed within a tolerance, then end the command.
   *
   * Requires that a roller diameter was provided in the config.
   *
   * @param velocity  Supplier returning the target surface speed.
   * @param tolerance Allowable error.
   * @return CommandPtr that ends once the flywheel is near the target surface speed.
   * @note Do not use with a default command on the subsystem, as it will override the setting after
   * this ends.
   */
  wpi::cmd::CommandPtr RunTo(std::function<wpi::units::meters_per_second_t()> velocity,
                         wpi::units::meters_per_second_t tolerance);

  // ---- Comparison triggers ---------------------------------------------------

  /**
   * Trigger that fires while the flywheel angular velocity is >= the given velocity.
   *
   * @param velocity Reference angular velocity.
   * @return Trigger for the >= condition.
   */
  wpi::cmd::Trigger Gte(wpi::units::degrees_per_second_t velocity);

  /**
   * Trigger that fires while the flywheel angular velocity is <= the given velocity.
   *
   * @param velocity Reference angular velocity.
   * @return Trigger for the <= condition.
   */
  wpi::cmd::Trigger Lte(wpi::units::degrees_per_second_t velocity);

  /**
   * Trigger that fires while the flywheel angular velocity is between start and end (inclusive).
   *
   * @param start Lower bound.
   * @param end   Upper bound.
   * @return Trigger for the range condition.
   */
  wpi::cmd::Trigger Between(wpi::units::degrees_per_second_t start, wpi::units::degrees_per_second_t end);

  /**
   * Trigger that fires while the flywheel is within tolerance of a velocity.
   *
   * @param velocity Reference angular velocity.
   * @param within   Tolerance.
   * @return Trigger for the near condition.
   */
  wpi::cmd::Trigger IsNear(wpi::units::degrees_per_second_t velocity,
                       wpi::units::degrees_per_second_t within = wpi::units::degrees_per_second_t{5.0}) const;

  // ---- Direct setpoint setters -----------------------------------------------

  /**
   * Directly command the flywheel to an angular velocity setpoint (non-command).
   *
   * @param velocity Desired angular velocity.
   */
  void SetVelocity(wpi::units::degrees_per_second_t velocity);

  /**
   * Directly command the flywheel to a surface speed setpoint (non-command).
   *
   * Requires roller diameter to be configured; no-op otherwise.
   *
   * @param speed Desired surface speed.
   */
  void SetSurfaceSpeed(wpi::units::meters_per_second_t speed);

  /**
   * Set flywheel velocity from a linear surface speed (base-class override).
   *
   * Converts the surface speed to angular velocity using the configured roller diameter.
   * Requires roller diameter to be configured; no-op otherwise.
   *
   * @param velocity Desired surface speed.
   */
  void SetMeasurementVelocitySetpoint(wpi::units::meters_per_second_t velocity);

  // ---- Misc ------------------------------------------------------------------

  /**
   * Get the 3-D position of the flywheel relative to the robot origin.
   *
   * @return Translation3d representing the mechanism endpoint.
   */
  wpi::math::Translation3d GetRelativeMechanismPosition() const;

  /**
   * Get the configuration used to construct this flywheel.
   *
   * @return Const reference to the FlyWheelConfig.
   */
  const config::FlyWheelConfig& GetConfig() const;

 private:
  config::FlyWheelConfig* m_flyWheelConfig{nullptr};
  std::string m_name{"FlyWheel"};
  std::optional<wpi::sim::DCMotorSim> m_dcMotorSim;
};

}  // namespace yams::mechanisms::velocity
