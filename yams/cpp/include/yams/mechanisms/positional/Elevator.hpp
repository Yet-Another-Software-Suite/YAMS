// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/geometry/Translation3d.h>
#include <frc/simulation/ElevatorSim.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>
#include <units/length.h>

#include <functional>
#include <optional>
#include <string>

#include "SmartPositionalMechanism.hpp"
#include "yams/mechanisms/config/ElevatorConfig.hpp"
#include "yams/motorcontrollers/SmartMotorController.hpp"

namespace yams::mechanisms::positional {

/**
 * Smart mechanism implementation for a linear elevator.
 *
 * Drives a carriage along a linear axis using a SmartMotorController
 * configured for measurement-space (meter) closed-loop control.
 *
 * ### Example usage (inside a subsystem constructor)
 * @code{.cpp}
 * using namespace yams::motorcontrollers;
 * using namespace yams::motorcontrollers::local;
 * using namespace yams::gearing;
 * using namespace yams::mechanisms::positional;
 * using Cfg = SmartMotorControllerConfig;
 *
 * // Declare as subsystem members:
 * //   rev::spark::SparkMax          m_sparkMax{2,
 * rev::spark::SparkLowLevel::MotorType::kBrushless};
 * //   std::optional<SparkWrapper>   m_smc;
 * //   ElevatorConfig                 m_elevatorConfig;
 * //   std::optional<Elevator>        m_elevator;
 *
 * SmartMotorControllerConfig motorCfg;
 * motorCfg.WithSubsystem(this)
 *         .WithFeedback(1.0, 0.0, 0.0)
 *         .WithMechanismCircumference(0.25_in, 22)
 *         .WithStartingPosition(0.5_m)
 *         .WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({3.0, 4.0})})
 *         .WithMeasurementLimits(0.0_m, 2.0_m)
 *         .WithIdleMode(Cfg::MotorMode::BRAKE)
 *         .WithSupplyCurrentLimit(40.0_A)
 *         .WithMotorInverted(false)
 *         .WithElevatorFeedforward(0.0, 0.0, 0.0)
 *         .WithClosedLoopMode()
 *         .WithTelemetry("ElevatorMotor", Cfg::TelemetryVerbosity::HIGH);
 *
 * m_smc.emplace(&m_sparkMax, frc::DCMotor::NEO(1), &motorCfg);
 *
 * m_elevatorConfig.WithMinimumHeight(0.0_m)
 *                 .WithMaximumHeight(3.0_m)
 *                 .WithCarriageMass(2.0_lb)
 *                 .WithTelemetryName("Elevator");
 *
 * m_elevator.emplace(&m_elevatorConfig, &m_smc.value());
 *
 * // In commands or bindings:
 * m_elevator->RunTo(1.2_m);
 * @endcode
 */
class Elevator : public SmartPositionalMechanism {
 public:
  /**
   * Construct an Elevator from an ElevatorConfig and a SmartMotorController.
   *
   * @param config Pointer to the elevator configuration (must outlive this Elevator).
   * @param smc    Pointer to the motor controller (must outlive this Elevator).
   */
  Elevator(config::ElevatorConfig* config, motorcontrollers::SmartMotorController* smc);

  // ---- SmartMechanism overrides ---------------------------------------------

  /** Advance the elevator's simulation by one loop iteration. */
  void SimIterate() override;

  /** Publish elevator telemetry to NetworkTables / SmartDashboard. */
  void UpdateTelemetry() override;

  /** Update the Mechanism2d ligament to reflect the current height. */
  void VisualizationUpdate() override;

  /** Get the human-readable name of this elevator. */
  std::string GetName() const override;

  // ---- SmartPositionalMechanism overrides -----------------------------------

  /**
   * Trigger that becomes true when the elevator is at or above its maximum
   * configured height.
   *
   * @return Trigger for the upper hard limit.
   */
  frc2::Trigger Max() override;

  /**
   * Trigger that becomes true when the elevator is at or below its minimum
   * configured height.
   *
   * @return Trigger for the lower hard limit.
   */
  frc2::Trigger Min() override;

  // ---- Elevator-specific interface ------------------------------------------

  /**
   * Set the height of the elevator.
   *
   * @param height Height of the elevator to reach.
   * @return CommandPtr that sets the elevator height, runs continuously.
   */
  frc2::CommandPtr Run(units::meter_t height);

  /**
   * Set the height of the elevator via a supplier.
   *
   * @param height Supplier returning the desired height each loop.
   * @return CommandPtr that sets the elevator height, runs continuously.
   */
  frc2::CommandPtr Run(std::function<units::meter_t()> height);

  /**
   * Command the elevator to a fixed height, then end when within tolerance.
   *
   * @param height    Target carriage height.
   * @param tolerance Acceptable error.
   * @return CommandPtr that ends once the elevator is near the target.
   */
  frc2::CommandPtr RunTo(units::meter_t height, units::meter_t tolerance = units::meter_t{0.01});

  /**
   * Command the elevator to a height from a supplier, then end when within tolerance.
   *
   * The supplier is evaluated once when the command is created.
   *
   * @param height    Supplier for the target height.
   * @param tolerance Acceptable error.
   * @return CommandPtr that ends once the elevator is near the target.
   */
  frc2::CommandPtr RunTo(std::function<units::meter_t()> height,
                         units::meter_t tolerance = units::meter_t{0.01});

  /**
   * Get the current carriage height from the motor encoder.
   *
   * @return Current height in meters.
   */
  units::meter_t GetHeight() const;

  /**
   * Trigger that fires while the elevator height is >= the given height.
   *
   * @param height Reference height.
   * @return Trigger for the >= condition.
   */
  frc2::Trigger Gte(units::meter_t height);

  /**
   * Trigger that fires while the elevator height is <= the given height.
   *
   * @param height Reference height.
   * @return Trigger for the <= condition.
   */
  frc2::Trigger Lte(units::meter_t height);

  /**
   * Trigger that fires while the elevator height is between start and end (inclusive).
   *
   * @param start Lower bound.
   * @param end   Upper bound.
   * @return Trigger for the range condition.
   */
  frc2::Trigger Between(units::meter_t start, units::meter_t end);

  /**
   * Trigger that fires while the elevator is within tolerance of a height.
   *
   * @param height Reference height.
   * @param within Tolerance.
   * @return Trigger for the near condition.
   */
  frc2::Trigger IsNear(units::meter_t height, units::meter_t within = units::meter_t{0.01});

  /**
   * Get the configuration used to construct this elevator.
   *
   * @return Const reference to the ElevatorConfig.
   */
  const config::ElevatorConfig& GetConfig() const;

  /**
   * Get the 3-D position of the elevator carriage relative to the robot origin.
   *
   * @return Translation3d representing the current carriage position.
   */
  frc::Translation3d GetRelativeMechanismPosition() const;

  /**
   * Directly command the elevator to a height setpoint (non-command, for use in periodic).
   *
   * @param height Desired carriage height.
   */
  void SetHeight(units::meter_t height);

 private:
  config::ElevatorConfig* m_elevatorConfig{nullptr};
  std::string m_name{"Elevator"};
  std::optional<frc::sim::ElevatorSim> m_elevatorSim;
  frc::MechanismLigament2d* m_setpointLigament{nullptr};
};

}  // namespace yams::mechanisms::positional
