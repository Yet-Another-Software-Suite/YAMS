// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Single-stage chain-driven elevator. Kraken X44 (CAN 2) -> 3:1 -> 4:1 gearbox -> 22-tooth
// #25 sprocket. Heights are in meters; all position limits live in m_motorConfig and
// m_elevatorConfig. See ElevatorSubsystem.h for the hardware layout overview.

#include "subsystems/ElevatorSubsystem.h"

#include <frc/system/plant/DCMotor.h>
#include <units/current.h>
#include <units/length.h>

#include <cmath>
#include <numbers>

using namespace yams::motorcontrollers;
using namespace yams::gearing;
using namespace yams::mechanisms;
using Cfg = SmartMotorControllerConfig;

ElevatorSubsystem::ElevatorSubsystem() {
  // Chain pitch 0.25 in, 22 teeth → circumference = 0.25 * 22 = 5.5 in
  constexpr double kChainPitchIn = 0.25;
  constexpr int kToothCount = 22;
  constexpr double kCircumferenceIn = kChainPitchIn * kToothCount;
  const units::meter_t circumference{kCircumferenceIn * 0.0254};

  m_motorConfig
      .WithSubsystem(this)
      // #25 chain, 22-tooth output sprocket: circumference = 0.25 in * 22 = 5.5 in (0.1397 m).
      // This is what converts one sprocket rotation into linear carriage travel.
      .WithMechanismCircumference(0.25_in, 22)
      // Carriage sits at 0.5 m on robot enable; seeds the TalonFX encoder so position
      // reads correctly before the first closed-loop move.
      .WithStartingPosition(units::meter_t{0.5})
      // kP=1: light-load elevator at 12:1 reduction responds cleanly. Increase if the
      // carriage sags under load or undershoots at full height. kI/kD stay at 0 until
      // steady-state error at the top of travel warrants integral.
      .WithFeedback(1, 0, 0)
      // .WithExponentialProfile(0.0, 0.0, units::volt_t{12})
      // Two serial reduction stages: 3:1 then 4:1 = 12:1 total at the sprocket.
      // If the ratio is wrong here, encoder-to-meters conversion will be off by the
      // same factor, causing the setpoint to over- or under-shoot proportionally.
      .WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({3.0, 4.0})})
      // Motor-controller-level soft limits: [0, 2] m. The Elevator mechanism config adds
      // a separate ceiling at 3 m, but these limits trip first and protect the hardware
      // even if a command bypasses the mechanism wrapper.
      .WithMeasurementLimits(units::meter_t{0}, units::meter_t{2})
      // BRAKE keeps the carriage from drifting down when a command ends. COAST here would
      // let gravity pull it down against the feedforward, wasting current.
      .WithIdleMode(Cfg::MotorMode::BRAKE)
      .WithTelemetry("ElevatorMotor", Cfg::TelemetryVerbosity::HIGH)
      // Supply limit (input side of the controller): 40 A caps battery draw without
      // significantly reducing peak force on this ~1 kg carriage. Stator is left
      // uncapped because the Kraken X44 rated stall is already modest.
      .WithSupplyCurrentLimit(units::ampere_t{40})
      .WithMotorInverted(false)
      // kS/kV/kA all at 0: placeholder for sysid tuning. Once characterized, kS
      // removes dead-band stiction and kV will improve velocity tracking during moves.
      .WithFeedforward(frc::ElevatorFeedforward{units::volt_t{0}, units::volt_t{0},
                                                units::unit_t<frc::ElevatorFeedforward::kv_unit>{0},
                                                units::unit_t<frc::ElevatorFeedforward::ka_unit>{0}})
      .WithClosedLoopMode();

  // emplace constructs TalonFXWrapper in-place inside the optional; pass a pointer to the
  // hardware object (not a reference) because the wrapper stores a raw pointer internally.
  // KrakenX44(1) is the sim motor model -- 1 motor in the gearbox, not a dual-motor config.
  // m_motorConfig is passed by pointer so the wrapper references the member directly.
  m_motor.emplace(&m_elevatorMotor, frc::DCMotor::KrakenX44(1), &m_motorConfig);

  // ElevatorConfig carries mechanism-level bounds and the carriage mass used by the sim.
  // WithMinimumHeight / WithMaximumHeight set the ElevatorSim travel limits (not the
  // motor-controller soft limits -- those are set in m_motorConfig above).
  m_elevatorConfig
      .WithMinimumHeight(units::meter_t{0})
      // 3 m is the sim ceiling; it is intentionally larger than the 2 m motor-controller
      // soft limit so the sim does not clip before the soft limit fires in real code.
      .WithMaximumHeight(units::meter_t{3})
      // Carriage mass for ElevatorSim gravity model. Under-estimating will make the sim
      // feel springier than the real bot; over-estimating will make it feel sluggish.
      .WithCarriageMass(2.0_lb)
      .WithTelemetryName("Elevator");

  // Elevator constructor takes (ElevatorConfig*, SmartMotorController*); the SMC is wired
  // in here rather than through ElevatorConfig::WithMotorController.
  m_elevator.emplace(&m_elevatorConfig, &m_motor.value());
}

void ElevatorSubsystem::Periodic() { m_elevator->UpdateTelemetry(); }

void ElevatorSubsystem::SimulationPeriodic() { m_elevator->SimIterate(); }

frc2::CommandPtr ElevatorSubsystem::ElevCmd(double dutycycle) { return m_elevator->Set(dutycycle); }

frc2::CommandPtr ElevatorSubsystem::SetHeight(units::meter_t height) {
  // Runs continuously -- the command never finishes on its own. Wrap with
  // Until() or use a finishing variant if you need a "move then stop" command.
  return m_elevator->Run(height);
}
