// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Turret pivot (yaw-only) driven by a Kraken X60 (CAN 12) through a three-stage compound
// gearbox (~51.84:1). Mounted 1.5 ft behind and 0.5 ft above the robot center.
// GetPose / GetVelocity translate that offset into field-relative coordinates for targeting.

#include "subsystems/TurretSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/system/plant/DCMotor.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>

#include <cmath>
#include <iostream>
#include <numbers>

using namespace yams::motorcontrollers;
using namespace yams::gearing;
using namespace yams::mechanisms;
using Cfg = SmartMotorControllerConfig;

TurretSubsystem::TurretSubsystem()
    // Pivot is 1.5 ft behind (-X) and 0.5 ft above (+Z) the robot origin; no yaw offset
    // at construction -- the turret's angular setpoint is relative to the robot heading.
    : m_roboToTurret{frc::Translation3d{units::foot_t{-1.5}, units::foot_t{0}, units::foot_t{0.5}},
                     frc::Rotation3d{}} {
  m_motorConfig.WithSubsystem(this)
      .WithClosedLoopMode()
      // kP=0 placeholder -- the turret is not yet PID-tuned. Set kP first; add kD for
      // damping if the turret oscillates when tracking a moving target.
      .WithFeedback(0.0, 0.0, 0)
      // Three-stage gearbox expressed as individual reduction factors, multiplied together:
      //   stage 1: 144/15 = 9.6:1, stage 2: 5:1, stage 3: 1.08:1 -> 51.84:1 total.
      // The initializer-list GearBox constructor multiplies all stages, same as
      // FromReductionStages.
      .WithMotorGearing(MechanismGearing{GearBox({144.0 / 15.0, 5.0, 1.08})})
      // BRAKE prevents the turret from drifting off-target when no command is running.
      // On a turret with a heavy mechanism attached, COAST can cause dangerous snap rotation.
      .WithIdleMode(Cfg::MotorMode::BRAKE)
      .WithMotorInverted(false)
      // WithArmFeedforward on a turret: kS=0.5 V overcomes static friction at the worm/gear
      // interface; kV=5.0 V/(turn/s) is the velocity gain for tracking; kA=0 (no accel FF yet);
      // kG=0 because the turret rotates in the horizontal plane -- gravity does no work.
      .WithArmFeedforward(0.5, 5.0, 0, 0.0)
      // Turret faces forward (0 deg = robot heading) at power-on. If the turret has a
      // hard-stop home position, replace 0 with the measured home angle.
      .WithStartingPosition(units::degree_t{0})
      .WithTelemetry("TurretMotor", Cfg::TelemetryVerbosity::HIGH)
      // Stator limit (output side): 60 A caps peak torque on the output shaft.
      // Using stator (not supply) here because we care about peak force, not battery draw.
      .WithStatorCurrentLimit(units::ampere_t{60});

  // KrakenX60(1): single-motor sim model for the turret. The (1) is the motor count
  // used to average free-speed and stall torque -- leave it at 1 for a single-motor drive.
  m_motor.emplace(&m_talonFX, frc::DCMotor::KrakenX60(1), &m_motorConfig);

  // Soft limits span a full revolution in each direction. This allows 360-degree tracking
  // without a hard stop but prevents winding the cable harness past two full turns.
  m_pivotConfig.WithMinAngle(units::degree_t{-360})
      .WithMaxAngle(units::degree_t{360})
      .WithTelemetryName("Turret");

  m_turret.emplace(&m_pivotConfig, &m_motor.value());
}

frc::Pose2d TurretSubsystem::GetPose(frc::Pose2d robotPose) const {
  // Projects the robot-to-turret 3D transform down to 2D for use with field-relative
  // pose estimators. The Z component (height) is discarded; only XY offset and yaw matter.
  return robotPose.TransformBy(frc::Transform2d{m_roboToTurret.Translation().ToTranslation2d(),
                                                m_roboToTurret.Rotation().ToRotation2d()});
}

frc::ChassisSpeeds TurretSubsystem::GetVelocity(frc::ChassisSpeeds robotVelocity,
                                                units::degree_t robotAngle) {
  // Compute the linear velocity at the turret pivot due to the robot's own rotation.
  // v = omega x r: in 2D, vx = -omega * ry, vy = omega * rx (right-hand cross product).
  // rWorld is the pivot offset rotated to the field frame so the cross product is correct.
  auto rRobot = m_roboToTurret.Translation().ToTranslation2d();
  auto rWorld = rRobot.RotateBy(frc::Rotation2d{robotAngle});

  double omega = robotVelocity.omega.value();  // rad/s

  double vRotX = -omega * rWorld.Y().value();
  double vRotY = omega * rWorld.X().value();

  units::meters_per_second_t turretVx{robotVelocity.vx.value() + vRotX};
  units::meters_per_second_t turretVy{robotVelocity.vy.value() + vRotY};

  // Add the turret's own rotational velocity (deg/s -> rad/s) to the robot yaw rate
  // so turret-relative omega is correct for a shooter that follows the turret barrel.
  double mecVelDegPerS = m_motor->GetMechanismVelocity().value();
  double mecVelRadPerS = mecVelDegPerS * std::numbers::pi / 180.0;
  units::radians_per_second_t turretOmega{omega + mecVelRadPerS};

  return frc::ChassisSpeeds{turretVx, turretVy, turretOmega};
}

void TurretSubsystem::SetAngleSetpoint(units::degree_t angle) {
  // Direct setpoint write -- useful for auto-aim from Periodic without wrapping in a command.
  // If a SetAngle() command is also running, its next iteration will overwrite this.
  m_turret->SetMechanismPositionSetpoint(angle);
}

void TurretSubsystem::Periodic() { m_turret->UpdateTelemetry(); }

void TurretSubsystem::SimulationPeriodic() { m_turret->SimIterate(); }

frc2::CommandPtr TurretSubsystem::TurretCmd(double dutycycle) {
  // Bypasses the Pivot mechanism and writes duty-cycle directly to the motor wrapper.
  // The "HIIII" cout is debug scaffolding -- remove before competition.
  return Run([this, dutycycle] {
    m_motor->SetDutyCycle(dutycycle);
    std::cout << "HIIII";
  });
}

frc2::CommandPtr TurretSubsystem::SetAngle(units::degree_t angle) { return m_turret->Run(angle); }
