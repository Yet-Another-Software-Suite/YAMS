// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

/**
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
#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/gearing/Sprocket.hpp"

// ---- Math ------------------------------------------------------------------
#include "yams/math/DerivativeTimeFilter.hpp"
#include "yams/math/LQRConfig.hpp"
#include "yams/math/LQRController.hpp"
#include "yams/math/SmartMath.hpp"

// ---- Units -----------------------------------------------------------------
#include "yams/units/YUnits.hpp"

// ---- Motor Controllers -----------------------------------------------------
#include "yams/motorcontrollers/SimSupplier.hpp"
#include "yams/motorcontrollers/SmartMotorController.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"

// ---- Simulation Suppliers --------------------------------------------------
#include "yams/motorcontrollers/simulation/ArmSimSupplier.hpp"
#include "yams/motorcontrollers/simulation/DCMotorSimSupplier.hpp"
#include "yams/motorcontrollers/simulation/ElevatorSimSupplier.hpp"

// ---- Mechanism Configs -----------------------------------------------------
#include "yams/mechanisms/config/ArmConfig.hpp"
#include "yams/mechanisms/config/ElevatorConfig.hpp"
#include "yams/mechanisms/config/FlyWheelConfig.hpp"
#include "yams/mechanisms/config/MechanismPositionConfig.hpp"
#include "yams/mechanisms/config/PivotConfig.hpp"

// ---- Mechanisms ------------------------------------------------------------
#include "yams/mechanisms/SmartMechanism.hpp"
#include "yams/mechanisms/positional/Arm.hpp"
#include "yams/mechanisms/positional/Elevator.hpp"
#include "yams/mechanisms/positional/Pivot.hpp"
#include "yams/mechanisms/positional/SmartPositionalMechanism.hpp"
#include "yams/mechanisms/velocity/FlyWheel.hpp"
#include "yams/mechanisms/velocity/SmartVelocityMechanism.hpp"

// ---- Swerve Mechanism Configs ----------------------------------------------
#include "yams/mechanisms/config/SwerveModuleConfig.hpp"

// ---- Swerve Mechanisms -----------------------------------------------------
#include "yams/mechanisms/swerve/SwerveDrive.hpp"
#include "yams/mechanisms/swerve/SwerveDriveConfig.hpp"
#include "yams/mechanisms/swerve/SwerveModule.hpp"

// ---- Swerve Utilities ------------------------------------------------------
#include "yams/mechanisms/swerve/utility/SwerveInputStream.hpp"

// ---- Telemetry -------------------------------------------------------------
#include "yams/telemetry/MechanismTelemetry.hpp"
#include "yams/telemetry/SmartMotorControllerTelemetry.hpp"
#include "yams/telemetry/SmartMotorControllerTelemetryConfig.hpp"

// ---- Exceptions ------------------------------------------------------------
#include "yams/exceptions.hpp"
