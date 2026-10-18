// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <functional>
#include <optional>
#include <string>
#include <vector>
#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/commands2/button/Trigger.hpp>
#include <wpi/math/geometry/Pose3d.hpp>
#include <wpi/simulation/ElevatorSim.hpp>
#include <wpi/smartdashboard/MechanismLigament2d.hpp>
#include <wpi/units/length.hpp>

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
 *         .WithZeroPower(Cfg::MotorMode::BRAKE)
 *         .WithSupplyCurrentLimit(40.0_A)
 *         .WithMotorInverted(false)
 *         .WithFeedforward(wpi::math::ElevatorFeedforward{
 *             0.0_V, 0.0_V, wpi::units::unit_t<wpi::math::ElevatorFeedforward::kv_unit>{0.0},
 *             wpi::units::unit_t<wpi::math::ElevatorFeedforward::ka_unit>{0.0}})
 *         .WithClosedLoopMode()
 *         .WithTelemetry("ElevatorMotor", Cfg::TelemetryVerbosity::HIGH);
 *
 * m_smc.emplace(&m_sparkMax, wpi::math::DCMotor::NEO(1), &motorCfg);
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
  wpi::cmd::Trigger Max() override;

  /**
   * Trigger that becomes true when the elevator is at or below its minimum
   * configured height.
   *
   * @return Trigger for the lower hard limit.
   */
  wpi::cmd::Trigger Min() override;

  // ---- Elevator-specific interface ------------------------------------------

  /**
   * Set the height of the elevator.
   *
   * @param height Height of the elevator to reach.
   * @return CommandPtr that sets the elevator height, runs continuously.
   */
  wpi::cmd::CommandPtr Run(wpi::units::meter_t height);

  /**
   * Set the height of the elevator via a supplier.
   *
   * @param height Supplier returning the desired height each loop.
   * @return CommandPtr that sets the elevator height, runs continuously.
   */
  wpi::cmd::CommandPtr Run(std::function<wpi::units::meter_t()> height);

  /**
   * Command the elevator to a fixed height, then end when within tolerance.
   *
   * @param height    Target carriage height.
   * @param tolerance Acceptable error.
   * @return CommandPtr that ends once the elevator is near the target.
   */
  wpi::cmd::CommandPtr RunTo(wpi::units::meter_t height,
                             wpi::units::meter_t tolerance = wpi::units::meter_t{0.01});

  /**
   * Command the elevator to a height from a supplier, then end when within tolerance.
   *
   * The supplier is evaluated once when the command is created.
   *
   * @param height    Supplier for the target height.
   * @param tolerance Acceptable error.
   * @return CommandPtr that ends once the elevator is near the target.
   */
  wpi::cmd::CommandPtr RunTo(std::function<wpi::units::meter_t()> height,
                             wpi::units::meter_t tolerance = wpi::units::meter_t{0.01});

  /**
   * Get the current carriage height from the motor encoder.
   *
   * @return Current height in meters.
   */
  wpi::units::meter_t GetHeight() const;

  /**
   * Trigger that fires while the elevator height is >= the given height.
   *
   * @param height Reference height.
   * @return Trigger for the >= condition.
   */
  wpi::cmd::Trigger Gte(wpi::units::meter_t height);

  /**
   * Trigger that fires while the elevator height is <= the given height.
   *
   * @param height Reference height.
   * @return Trigger for the <= condition.
   */
  wpi::cmd::Trigger Lte(wpi::units::meter_t height);

  /**
   * Trigger that fires while the elevator height is between start and end (inclusive).
   *
   * @param start Lower bound.
   * @param end   Upper bound.
   * @return Trigger for the range condition.
   */
  wpi::cmd::Trigger Between(wpi::units::meter_t start, wpi::units::meter_t end);

  /**
   * Trigger that fires while the elevator is within tolerance of a height.
   *
   * @param height Reference height.
   * @param within Tolerance.
   * @return Trigger for the near condition.
   */
  wpi::cmd::Trigger IsNear(wpi::units::meter_t height,
                           wpi::units::meter_t within = wpi::units::meter_t{0.01});

  /**
   * Get the configuration used to construct this elevator.
   *
   * @return Const reference to the ElevatorConfig.
   */
  const config::ElevatorConfig& GetConfig() const;

  /**
   * Get the carriage's pose, for publishing to AdvantageScope as a component of a robot model:
   * where the elevator is mounted, as set in its MechanismPositionConfig, moved up the elevator by
   * its height, at the elevator's angle in its movement plane.
   *
   * @return The pose of the carriage, relative to the robot.
   */
  wpi::math::Pose3d GetPose3d() const override;

  /**
   * Get the pose of each moving stage of a cascade elevator, for publishing to AdvantageScope as
   * components of a robot model. A cascade elevator's stages all move at once, each by its share of
   * the carriage's height: a stage that travels a third of the elevator's total travel moves a third
   * of the carriage's height relative to the stage below it.
   *
   * @param stages The number of stages that move, counting the carriage as the last.
   * @param stageHeights How far each moving stage travels relative to the stage below it, from the
   *        stage nearest the fixed stage to the carriage.
   * @return Each moving stage's pose, relative to the robot, in the same order. The last is the
   *         carriage, GetPose3d().
   * @throws std::invalid_argument if stages is less than 1, there is not one height for each stage,
   *         or a height is not positive.
   */
  std::vector<wpi::math::Pose3d> GetCascadePose3d(
      int stages, const std::vector<wpi::units::meter_t>& stageHeights) const;

  /**
   * Get the pose of each moving stage of a continuous elevator, for publishing to AdvantageScope as
   * components of a robot model. A continuous elevator's stages move one at a time: the carriage
   * rises the whole of its travel first, then the stage below it, and so on out to the stage nearest
   * the fixed stage.
   *
   * @param stages The number of stages that move, counting the carriage as the last.
   * @param heights How far each moving stage travels relative to the stage below it, from the stage
   *        nearest the fixed stage to the carriage.
   * @return Each moving stage's pose, relative to the robot, in the same order. The last is the
   *         carriage, GetPose3d().
   * @throws std::invalid_argument if stages is less than 1, there is not one height for each stage,
   *         or a height is not positive.
   */
  std::vector<wpi::math::Pose3d> GetContinuousPose3d(
      int stages, const std::vector<wpi::units::meter_t>& heights) const;

  /**
   * Directly command the elevator to a height setpoint (non-command, for use in periodic).
   *
   * @param height Desired carriage height.
   */
  void SetHeight(wpi::units::meter_t height);


 private:
  /** The pose of each stage, from how far each is extended relative to the stage below it. */
  std::vector<wpi::math::Pose3d> StagePoses(const std::vector<double>& extension) const;
  config::ElevatorConfig* m_elevatorConfig{nullptr};
  std::string m_name{"Elevator"};
  std::optional<wpi::sim::ElevatorSim> m_elevatorSim;
  wpi::MechanismLigament2d* m_setpointLigament{nullptr};
};

}  // namespace yams::mechanisms::positional
