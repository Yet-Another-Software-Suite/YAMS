// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "subsystems/DiffDriveSubsystem.h"

#include <frc/system/plant/DCMotor.h>
#include <units/length.h>

using namespace yams::motorcontrollers;
using namespace yams::gearing;
using Cfg = SmartMotorControllerConfig;

DiffDriveSubsystem::DiffDriveSubsystem() {
  // 3:1 then 4:1 in series = 12:1 total rotor-to-wheel reduction.
  // 4-inch wheel diameter (4.0 * 0.0254 m) gives circumference used for distance tracking.
  MechanismGearing gearing{GearBox::FromReductionStages({3.0, 4.0})};
  const units::meter_t wheelDiameter{4.0 * 0.0254};

  // Left side inverted because the left motor is mounted mirrored to the right.
  // Open-loop mode: DifferentialDrive sends duty-cycle directly, no closed-loop PID needed
  // for basic tank/arcade drive.  COAST idle mode: wheels roll freely when the driver
  // releases the stick, which feels natural for driver practice but may overshoot in auto --
  // switch to BRAKE if you want the robot to stop sharply.
  //
  // DCMotor::NEO(2) tells the sim there are 2 NEOs per side (two motors per gearbox).
  // This affects the simulated stall torque and free speed.
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

  // SparkWrapper takes a pointer (CAN IDs 21 and 24 from the header).
  // Followers (22 and 23) are not wired through YAMS here -- configure them via
  // SmartMotorControllerConfig::WithFollowers if you need YAMS to manage them.
  m_leftSMC.emplace(&m_leftMotor, frc::DCMotor::NEO(2), &m_leftConfig);
  m_rightSMC.emplace(&m_rightMotor, frc::DCMotor::NEO(2), &m_rightConfig);

  // DifferentialDrive's output callbacks: each lambda captures the SparkWrapper by pointer
  // and forwards the duty-cycle from WPILib's tank/arcade math to the actual motor.
  // The callbacks are invoked by DifferentialDrive::TankDrive / ArcadeDrive every loop --
  // they must stay valid for the lifetime of m_drive, which is why they capture `this`.
  m_drive.emplace([this](double dc) { m_leftSMC->SetDutyCycle(dc); },
                  [this](double dc) { m_rightSMC->SetDutyCycle(dc); });

  // SetDefaultCommand(Stop()) ensures the robot applies zero output whenever no other
  // command is running on this subsystem.  Without a default command the motors would
  // hold their last duty-cycle across command boundaries, which can cause unintended
  // movement or brownouts during auto-to-teleop transitions.
  SetDefaultCommand(Stop());
}

// Actively commands 0% output each loop.  Using Run (not RunOnce) means the subsystem
// is continuously required, so any command that needs the drivetrain will properly
// interrupt Stop rather than fighting it.
frc2::CommandPtr DiffDriveSubsystem::Stop() {
  return Run([this] { m_drive->StopMotor(); });
}

// Supplier-based overloads let callers pass joystick lambdas; the values are read
// fresh each loop inside the Run closure so axis changes are picked up immediately.
frc2::CommandPtr DiffDriveSubsystem::TankDrive(std::function<double()> left,
                                               std::function<double()> right) {
  return Run([this, left, right] { m_drive->TankDrive(left(), right()); });
}

// zRotation is the turning rate (positive = turn left per WPILib convention).
frc2::CommandPtr DiffDriveSubsystem::ArcadeDrive(std::function<double()> xSpeed,
                                                 std::function<double()> zRotation) {
  return Run([this, xSpeed, zRotation] { m_drive->ArcadeDrive(xSpeed(), zRotation()); });
}

void DiffDriveSubsystem::Periodic() {}

// Steps each side's DCMotorSim independently.  Sim does not model follower motors;
// each SparkWrapper simulates only the single motor passed to its constructor.
void DiffDriveSubsystem::SimulationPeriodic() {
  m_leftSMC->SimIterate();
  m_rightSMC->SimIterate();
}
