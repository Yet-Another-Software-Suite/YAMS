// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "subsystems/DiffDriveSubsystem.h"

#include <frc/system/plant/DCMotor.h>
#include <units/length.h>

using namespace yams::motorcontrollers;
using namespace yams::gearing;
using Cfg = SmartMotorControllerConfig;

DiffDriveSubsystem::DiffDriveSubsystem() {
  MechanismGearing gearing{GearBox::FromReductionStages({3.0, 4.0})};
  const units::meter_t wheelDiameter{4.0 * 0.0254};

  m_leftConfig.WithSubsystem(this)
      .WithOpenLoopMode()
      .WithMotorGearing(gearing)
      .WithIdleMode(Cfg::MotorMode::COAST)
      .WithMotorInverted(true)
      .WithMechanismCircumference(wheelDiameter * 3.14159265)
      .WithTelemetry("LeftMotorMain", Cfg::TelemetryVerbosity::LOW);

  m_rightConfig.WithSubsystem(this)
      .WithOpenLoopMode()
      .WithMotorGearing(gearing)
      .WithIdleMode(Cfg::MotorMode::COAST)
      .WithMotorInverted(false)
      .WithMechanismCircumference(wheelDiameter * 3.14159265)
      .WithTelemetry("RightMotorMain", Cfg::TelemetryVerbosity::LOW);

  m_leftSMC.emplace(m_leftMotor, frc::DCMotor::NEO(2), m_leftConfig);
  m_rightSMC.emplace(m_rightMotor, frc::DCMotor::NEO(2), m_rightConfig);

  m_drive.emplace([this](double dc) { m_leftSMC->SetDutyCycle(dc); },
                  [this](double dc) { m_rightSMC->SetDutyCycle(dc); });

  SetDefaultCommand(Stop());
}

frc2::CommandPtr DiffDriveSubsystem::Stop() {
  return Run([this] { m_drive->StopMotor(); });
}

frc2::CommandPtr DiffDriveSubsystem::TankDrive(std::function<double()> left,
                                               std::function<double()> right) {
  return Run([this, left, right] { m_drive->TankDrive(left(), right()); });
}

frc2::CommandPtr DiffDriveSubsystem::ArcadeDrive(std::function<double()> xSpeed,
                                                 std::function<double()> zRotation) {
  return Run([this, xSpeed, zRotation] { m_drive->ArcadeDrive(xSpeed(), zRotation()); });
}

void DiffDriveSubsystem::Periodic() {}

void DiffDriveSubsystem::SimulationPeriodic() {
  m_leftSMC->SimIterate();
  m_rightSMC->SimIterate();
}
