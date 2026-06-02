// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

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

// ---- Exceptions ------------------------------------------------------------
#include "yams/exceptions/InvalidStageGivenException.h"
#include "yams/exceptions/NoStagesGivenException.h"
#include "yams/exceptions/SmartMotorControllerConfigurationException.h"
#include "yams/exceptions/YamsException.h"
