// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

/**
 * @file YAMS.h
 * @brief Convenience header that includes the entire YAMS C++ library.
 *
 * YAMS — Yet Another Mechanism System (C++ port).
 * Requires WPILib 2026, REVLib, and CTRE Phoenix 6.
 *
 * Include this single header to access all gearing, math, unit, motor-controller,
 * mechanism, and exception types provided by YAMS.
 */

// YAMS — Yet Another Mechanism System (C++ port)
// WPILib 2026 | REVLib | Phoenix 6

// ---- Gearing ---------------------------------------------------------------
#include "yams/gearing/GearBox.h"
#include "yams/gearing/MechanismGearing.h"
#include "yams/gearing/Sprocket.h"

// ---- Math ------------------------------------------------------------------
#include "yams/math/DerivativeTimeFilter.h"
#include "yams/math/LQRConfig.h"
#include "yams/math/LQRController.h"
#include "yams/math/SmartMath.h"

// ---- Units -----------------------------------------------------------------
#include "yams/units/EasyCRT.h"
#include "yams/units/EasyCRTConfig.h"
#include "yams/units/YUnits.h"

// ---- Motor Controllers -----------------------------------------------------
#include "yams/motorcontrollers/SimSupplier.h"
#include "yams/motorcontrollers/SmartMotorController.h"
#include "yams/motorcontrollers/SmartMotorControllerConfig.h"
#include "yams/motorcontrollers/SmartMotorFactory.h"

// ---- Simulation Suppliers --------------------------------------------------
#include "yams/motorcontrollers/simulation/ArmSimSupplier.h"
#include "yams/motorcontrollers/simulation/DCMotorSimSupplier.h"
#include "yams/motorcontrollers/simulation/ElevatorSimSupplier.h"

// ---- Motor Controller Wrappers ---------------------------------------------
#include "yams/motorcontrollers/local/SparkWrapper.h"
#include "yams/motorcontrollers/remote/TalonFXSWrapper.h"
#include "yams/motorcontrollers/remote/TalonFXWrapper.h"

// ---- Mechanism Configs -----------------------------------------------------
#include "yams/mechanisms/config/ArmConfig.h"
#include "yams/mechanisms/config/ElevatorConfig.h"
#include "yams/mechanisms/config/FlyWheelConfig.h"
#include "yams/mechanisms/config/MechanismPositionConfig.h"
#include "yams/mechanisms/config/PivotConfig.h"
#include "yams/mechanisms/config/SensorConfig.h"

// ---- Mechanisms ------------------------------------------------------------
#include "yams/mechanisms/SmartMechanism.h"
#include "yams/mechanisms/positional/Arm.h"
#include "yams/mechanisms/positional/Elevator.h"
#include "yams/mechanisms/positional/Pivot.h"
#include "yams/mechanisms/positional/SmartPositionalMechanism.h"
#include "yams/mechanisms/velocity/FlyWheel.h"
#include "yams/mechanisms/velocity/SmartVelocityMechanism.h"

// ---- Swerve Mechanism Configs ----------------------------------------------
#include "yams/mechanisms/config/SwerveModuleConfig.h"

// ---- Swerve Mechanisms -----------------------------------------------------
#include "yams/mechanisms/swerve/SwerveDrive.h"
#include "yams/mechanisms/swerve/SwerveDriveConfig.h"
#include "yams/mechanisms/swerve/SwerveModule.h"

// ---- Telemetry -------------------------------------------------------------
#include "yams/telemetry/MechanismTelemetry.h"
#include "yams/telemetry/SmartMotorControllerTelemetry.h"
#include "yams/telemetry/SmartMotorControllerTelemetryConfig.h"

// ---- Exceptions ------------------------------------------------------------
#include "yams/exceptions/InvalidStageGivenException.h"
#include "yams/exceptions/NoStagesGivenException.h"
#include "yams/exceptions/SmartMotorControllerConfigurationException.h"
#include "yams/exceptions/SwerveDriveConfigurationException.h"
#include "yams/exceptions/YamsException.h"
