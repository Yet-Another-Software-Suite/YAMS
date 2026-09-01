// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <wpi/math/geometry/Translation3d.hpp>
#include <wpi/simulation/SingleJointedArmSim.hpp>
#include <wpi/smartdashboard/MechanismLigament2d.hpp>
#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/commands2/button/Trigger.hpp>
#include <wpi/units/angle.hpp>

#include <functional>
#include <optional>
#include <string>

#include "SmartPositionalMechanism.hpp"
#include "yams/mechanisms/config/ArmConfig.hpp"

namespace yams::mechanisms::positional {

/**
 * Smart mechanism implementation for a single-jointed arm.
 *
 * Drives an arm joint using a SmartMotorController configured for
 * mechanism-space (degree) closed-loop angular control.
 *
 * ### Example usage (inside a subsystem constructor)
 * @code{.cpp}
 * using namespace yams::motorcontrollers;
 * using namespace yams::motorcontrollers::remote;
 * using namespace yams::gearing;
 * using namespace yams::mechanisms::positional;
 * using Cfg = SmartMotorControllerConfig;
 *
 * // Declare as subsystem members:
 * //   ctre::phoenix6::hardware::TalonFX m_talon{1};
 * //   std::optional<TalonFXWrapper>     m_smc;
 * //   ArmConfig                          m_armConfig;
 * //   std::optional<Arm>                 m_arm;
 *
 * SmartMotorControllerConfig motorCfg;
 * motorCfg.WithSubsystem(this)
 *         .WithFeedback(4.0, 0.0, 0.0)
 *         .WithTrapezoidProfile(wpi::units::turns_per_second_t{0.5},
 *                               wpi::units::turns_per_second_squared_t{0.25})
 *         .WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({3.0, 4.0})})
 *         .WithIdleMode(Cfg::MotorMode::BRAKE)
 *         .WithStatorCurrentLimit(40.0_A)
 *         .WithMotorInverted(false)
 *         .WithFeedforward(wpi::math::ArmFeedforward{
 *             0.0_V, 0.0_V, wpi::units::unit_t<wpi::math::ArmFeedforward::kv_unit>{0.0},
 *             wpi::units::unit_t<wpi::math::ArmFeedforward::ka_unit>{0.0}})
 *         .WithClosedLoopMode()
 *         .WithTelemetry("ArmMotor", Cfg::TelemetryVerbosity::HIGH);
 *
 * m_smc.emplace(m_talon, wpi::math::DCMotor::KrakenX60(1), motorCfg);
 *
 * m_armConfig.WithMinAngle(-100.0_deg)
 *            .WithMaxAngle(200.0_deg)
 *            .WithArmLength(0.135_m)
 *            .WithTelemetryName("ArmExample");
 *
 * m_arm.emplace(&m_armConfig, &m_smc.value());
 *
 * // In commands or bindings:
 * m_arm->RunTo(45.0_deg);
 * @endcode
 */
class Arm : public SmartPositionalMechanism {
 public:
  /**
   * Construct an Arm from an ArmConfig pointer and a SmartMotorController pointer.
   *
   * @param config Pointer to the arm configuration (must outlive this Arm).
   * @param smc    Pointer to the motor controller (must outlive this Arm).
   */
  Arm(config::ArmConfig* config, motorcontrollers::SmartMotorController* smc);

  // ---- SmartMechanism overrides ---------------------------------------------

  /** Advance the arm's simulation by one loop iteration. */
  void SimIterate() override;

  /** Publish arm telemetry to NetworkTables / SmartDashboard. */
  void UpdateTelemetry() override;

  /** Update the Mechanism2d ligament to reflect the current angle. */
  void VisualizationUpdate() override;

  /** Get the human-readable name of this arm. */
  std::string GetName() const override;

  // ---- SmartPositionalMechanism overrides -----------------------------------

  /**
   * Trigger that becomes true when the arm is at or past its maximum
   * configured angle.
   *
   * @return Trigger for the upper angular hard limit.
   */
  wpi::cmd::Trigger Max() override;

  /**
   * Trigger that becomes true when the arm is at or past its minimum
   * configured angle.
   *
   * @return Trigger for the lower angular hard limit.
   */
  wpi::cmd::Trigger Min() override;

  // ---- Arm-specific interface -----------------------------------------------

  /**
   * Set the arm to the given angle.
   *
   * @param angle Arm angle to go to.
   * @return CommandPtr that requires the configured subsystem.
   */
  wpi::cmd::CommandPtr Run(wpi::units::degree_t angle);

  /**
   * Set the arm to the given angle via a supplier.
   *
   * @param angle Supplier returning the desired arm angle each loop.
   * @return CommandPtr that requires the configured subsystem.
   */
  wpi::cmd::CommandPtr Run(std::function<wpi::units::degree_t()> angle);

  /**
   * Command the arm to a fixed angle, then end when within tolerance.
   *
   * @param angle     Target joint angle.
   * @param tolerance Acceptable error.
   * @return CommandPtr that ends once the arm is near the target.
   */
  wpi::cmd::CommandPtr RunTo(wpi::units::degree_t angle, wpi::units::degree_t tolerance = wpi::units::degree_t{1.0});

  /**
   * Command the arm to an angle from a supplier, then end when within tolerance.
   *
   * The supplier is evaluated once when the command is created.
   *
   * @param angle     Supplier for the target angle.
   * @param tolerance Acceptable error.
   * @return CommandPtr that ends once the arm is near the target.
   */
  wpi::cmd::CommandPtr RunTo(std::function<wpi::units::degree_t()> angle,
                         wpi::units::degree_t tolerance = wpi::units::degree_t{1.0});

  /**
   * Get the current joint angle from the motor encoder.
   *
   * @return Current mechanism angle in degrees.
   */
  wpi::units::degree_t GetAngle() const;

  /**
   * Trigger that fires while the arm angle is >= the given angle.
   *
   * @param angle Reference angle.
   * @return Trigger for the >= condition.
   */
  wpi::cmd::Trigger Gte(wpi::units::degree_t angle);

  /**
   * Trigger that fires while the arm angle is <= the given angle.
   *
   * @param angle Reference angle.
   * @return Trigger for the <= condition.
   */
  wpi::cmd::Trigger Lte(wpi::units::degree_t angle);

  /**
   * Trigger that fires while the arm angle is between start and end (inclusive).
   *
   * @param start Lower bound.
   * @param end   Upper bound.
   * @return Trigger for the range condition.
   */
  wpi::cmd::Trigger Between(wpi::units::degree_t start, wpi::units::degree_t end);

  /**
   * Trigger that fires while the arm is within tolerance of an angle.
   *
   * @param angle  Reference angle.
   * @param within Tolerance.
   * @return Trigger for the near condition.
   */
  wpi::cmd::Trigger IsNear(wpi::units::degree_t angle, wpi::units::degree_t within = wpi::units::degree_t{1.0});

  /**
   * Get the configuration used to construct this arm.
   *
   * @return Const reference to the ArmConfig.
   */
  const config::ArmConfig& GetConfig() const;

  /**
   * Get the 3-D position of the arm tip relative to the robot origin.
   *
   * @return Translation3d representing the mechanism endpoint.
   */
  wpi::math::Translation3d GetRelativeMechanismPosition() const;

  /**
   * Directly command the arm to an angle setpoint (non-command, for use in periodic).
   *
   * @param angle Desired joint angle.
   */
  void SetAngle(wpi::units::degree_t angle);

 private:
  config::ArmConfig* m_armConfig{nullptr};
  std::string m_name{"Arm"};
  std::optional<wpi::sim::SingleJointedArmSim> m_armSim;
  wpi::MechanismLigament2d* m_setpointLigament{nullptr};
};

}  // namespace yams::mechanisms::positional
