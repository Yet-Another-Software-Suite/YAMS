// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/geometry/Translation3d.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>
#include <units/angle.h>

#include <functional>
#include <optional>
#include <string>

#include "SmartPositionalMechanism.hpp"
#include "yams/mechanisms/config/PivotConfig.hpp"

namespace yams::mechanisms::positional {

/**
 * Smart mechanism implementation for a pivot joint.
 *
 * Similar to an Arm but intended for mechanisms that rotate a fixed-length
 * assembly (e.g. a shooter hood or wrist) rather than a cantilevered arm with
 * a meaningful length.  Uses mechanism-space (degree) closed-loop control.
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
 * //   ctre::phoenix6::hardware::TalonFXS m_talonFXS{3};
 * //   std::optional<TalonFXSWrapper>      m_smc;
 * //   PivotConfig                          m_pivotConfig;
 * //   std::optional<Pivot>                 m_pivot;
 *
 * SmartMotorControllerConfig motorCfg;
 * motorCfg.WithSubsystem(this)
 *         .WithFeedback(4.0, 0.0, 0.0)
 *         .WithTrapezoidProfile(units::turns_per_second_t{0.5},
 *                               units::turns_per_second_squared_t{0.25})
 *         .WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({3.0, 4.0})})
 *         .WithIdleMode(Cfg::MotorMode::BRAKE)
 *         .WithStatorCurrentLimit(40.0_A)
 *         .WithMotorInverted(false)
 *         .WithArmFeedforward(0.0, 0.0, 0.0, 0.0)
 *         .WithClosedLoopMode()
 *         .WithTelemetry("HoodMotor", Cfg::TelemetryVerbosity::HIGH);
 *
 * m_smc.emplace(m_talonFXS, frc::DCMotor::NEO(1),
 *               TalonFXSWrapper::MotorArrangement::NEO, motorCfg);
 *
 * m_pivotConfig.WithMinAngle(-100.0_deg)
 *              .WithMaxAngle(200.0_deg)
 *              .WithTelemetryName("HoodExample");
 *
 * m_pivot.emplace(&m_pivotConfig, &m_smc.value());
 *
 * // In commands or bindings:
 * m_pivot->RunTo(30.0_deg);
 * @endcode
 */
class Pivot : public SmartPositionalMechanism {
 public:
  /**
   * Construct a Pivot from a PivotConfig and a SmartMotorController.
   *
   * @param config Pointer to the pivot configuration (must outlive this Pivot).
   * @param smc    Pointer to the motor controller (must outlive this Pivot).
   */
  Pivot(config::PivotConfig* config, motorcontrollers::SmartMotorController* smc);

  // ---- SmartMechanism overrides ---------------------------------------------

  /** Advance the pivot's simulation by one loop iteration. */
  void SimIterate() override;

  /** Publish pivot telemetry to NetworkTables / SmartDashboard. */
  void UpdateTelemetry() override;

  /** Update the Mechanism2d ligament to reflect the current angle. */
  void VisualizationUpdate() override;

  /** Get the human-readable name of this pivot. */
  std::string GetName() const override;

  // ---- SmartPositionalMechanism overrides -----------------------------------

  /**
   * Trigger that becomes true when the pivot is at or past its maximum
   * configured angle.
   *
   * @return Trigger for the upper angular hard limit.
   */
  frc2::Trigger Max() override;

  /**
   * Trigger that becomes true when the pivot is at or past its minimum
   * configured angle.
   *
   * @return Trigger for the lower angular hard limit.
   */
  frc2::Trigger Min() override;

  // ---- Pivot-specific interface ---------------------------------------------

  /**
   * Set the pivot to the given angle.
   *
   * @param angle Pivot angle to go to.
   * @return CommandPtr that requires the configured subsystem.
   */
  frc2::CommandPtr Run(units::degree_t angle);

  /**
   * Set the pivot to the given angle via a supplier.
   *
   * @param angle Supplier returning the desired pivot angle each loop.
   * @return CommandPtr that requires the configured subsystem.
   */
  frc2::CommandPtr Run(std::function<units::degree_t()> angle);

  /**
   * Command the pivot to a fixed angle, then end when within tolerance.
   *
   * @param angle     Target pivot angle.
   * @param tolerance Acceptable error.
   * @return CommandPtr that ends once the pivot is near the target.
   */
  frc2::CommandPtr RunTo(units::degree_t angle, units::degree_t tolerance = units::degree_t{1.0});

  /**
   * Command the pivot to an angle from a supplier, then end when within tolerance.
   *
   * The supplier is evaluated once when the command is created.
   *
   * @param angle     Supplier for the target angle.
   * @param tolerance Acceptable error.
   * @return CommandPtr that ends once the pivot is near the target.
   */
  frc2::CommandPtr RunTo(std::function<units::degree_t()> angle,
                         units::degree_t tolerance = units::degree_t{1.0});

  /**
   * Get the current pivot angle from the motor encoder.
   *
   * @return Current mechanism angle in degrees.
   */
  units::degree_t GetAngle() const;

  /**
   * Trigger that fires while the pivot angle is >= the given angle.
   *
   * @param angle Reference angle.
   * @return Trigger for the >= condition.
   */
  frc2::Trigger Gte(units::degree_t angle);

  /**
   * Trigger that fires while the pivot angle is <= the given angle.
   *
   * @param angle Reference angle.
   * @return Trigger for the <= condition.
   */
  frc2::Trigger Lte(units::degree_t angle);

  /**
   * Trigger that fires while the pivot angle is between start and end (inclusive).
   *
   * @param start Lower bound.
   * @param end   Upper bound.
   * @return Trigger for the range condition.
   */
  frc2::Trigger Between(units::degree_t start, units::degree_t end);

  /**
   * Trigger that fires while the pivot is within tolerance of an angle.
   *
   * @param angle  Reference angle.
   * @param within Tolerance.
   * @return Trigger for the near condition.
   */
  frc2::Trigger IsNear(units::degree_t angle, units::degree_t within = units::degree_t{1.0});

  /**
   * Get the configuration used to construct this pivot.
   *
   * @return Const reference to the PivotConfig.
   */
  const config::PivotConfig& GetConfig() const;

  /**
   * Get the 3-D position of the pivot tip relative to the robot origin.
   *
   * @return Translation3d representing the mechanism endpoint.
   */
  frc::Translation3d GetRelativeMechanismPosition() const;

  /**
   * Directly command the pivot to an angle setpoint (non-command, for use in periodic).
   *
   * @param angle Desired pivot angle.
   */
  void SetAngle(units::degree_t angle);

 private:
  config::PivotConfig* m_pivotConfig{nullptr};
  std::string m_name{"Pivot"};
  std::optional<frc::sim::DCMotorSim> m_dcMotorSim;
  frc::MechanismLigament2d* m_setpointLigament{nullptr};
};

}  // namespace yams::mechanisms::positional
