// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/trajectory/ExponentialProfile.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/SubsystemBase.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>
#include <units/temperature.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <any>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "yams/gearing/MechanismGearing.hpp"
#include "yams/math/LQRConfig.hpp"

namespace yams::motorcontrollers {

// Forward declaration to avoid circular dependency (SmartMotorController.hpp
// includes this header, so we cannot include it here).
class SmartMotorController;

/**
 * Unified configuration for a SmartMotorController.
 *
 * Stores PID/feedforward gains (up to 4 slots), motion-profile parameters, soft limits,
 * current limits, gearing, telemetry settings, and simulation motor model.  Uses a fluent
 * builder pattern; all With* methods return *this for chaining.
 */
class SmartMotorControllerConfig {
 public:
  /** Closed-loop vs open-loop output mode. */
  enum class ControlMode { CLOSED_LOOP, OPEN_LOOP };
  /** Motor idle (neutral) behaviour. */
  enum class MotorMode { COAST, BRAKE };
  /** Amount of data published to NetworkTables. */
  enum class TelemetryVerbosity { NONE, LOW, MEDIUM, HIGH };
  /** Which PID/feedforward gain slot to use (hardware-level slot selection). */
  enum class ClosedLoopControllerSlot { SLOT_0, SLOT_1, SLOT_2, SLOT_3 };

  // ---- Feedback -------------------------------------------------------

  /**
   * Set PID feedback gains for the specified slot.
   *
   * @param kP   Proportional gain.
   * @param kI   Integral gain.
   * @param kD   Derivative gain.
   * @param slot Gain slot to write (default SLOT_0).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithFeedback(
      double kP, double kI, double kD,
      ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);

  /**
   * Set the proportional gain for the specified slot.
   *
   * @param kP   Proportional gain.
   * @param slot Gain slot to write (default SLOT_0).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithKp(
      double kP, ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);

  /**
   * Set the integral gain for the specified slot.
   *
   * @param kI   Integral gain.
   * @param slot Gain slot to write (default SLOT_0).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithKi(
      double kI, ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);

  /**
   * Set the derivative gain for the specified slot.
   *
   * @param kD   Derivative gain.
   * @param slot Gain slot to write (default SLOT_0).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithKd(
      double kD, ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);

  // ---- Feedforward -------------------------------------------------------

  /**
   * Configure an arm feedforward model (kS, kV, kA, kG) for the specified slot.
   *
   * @param kS   Static friction voltage.
   * @param kV   Velocity feedforward coefficient.
   * @param kA   Acceleration feedforward coefficient.
   * @param kG   Gravity feedforward coefficient.
   * @param slot Gain slot to write (default SLOT_0).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithArmFeedforward(
      double kS, double kV, double kA, double kG,
      ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);

  /**
   * Configure an elevator feedforward model (kS, kV, kA) for the specified slot.
   *
   * @param kS   Static friction voltage.
   * @param kV   Velocity feedforward coefficient.
   * @param kA   Acceleration feedforward coefficient.
   * @param slot Gain slot to write (default SLOT_0).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithElevatorFeedforward(
      double kS, double kV, double kA,
      ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);

  /**
   * Configure a simple motor feedforward model (kS, kV, optional kA) for the specified slot.
   *
   * @param kS   Static friction voltage.
   * @param kV   Velocity feedforward coefficient.
   * @param kA   Acceleration feedforward coefficient (default 0).
   * @param slot Gain slot to write (default SLOT_0).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithSimpleFeedforward(
      double kS, double kV, double kA = 0.0,
      ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);

  /**
   * Set the static friction feedforward (kS) for the specified slot.
   *
   * @param kS   Static friction voltage.
   * @param slot Gain slot to write (default SLOT_0).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithKs(
      double kS, ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);

  /**
   * Set the velocity feedforward coefficient (kV) for the specified slot.
   *
   * @param kV   Velocity feedforward coefficient.
   * @param slot Gain slot to write (default SLOT_0).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithKv(
      double kV, ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);

  /**
   * Set the acceleration feedforward coefficient (kA) for the specified slot.
   *
   * @param kA   Acceleration feedforward coefficient.
   * @param slot Gain slot to write (default SLOT_0).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithKa(
      double kA, ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);

  /**
   * Set the gravity feedforward coefficient (kG) for the specified slot.
   *
   * @param kG   Gravity feedforward coefficient.
   * @param slot Gain slot to write (default SLOT_0).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithKg(
      double kG, ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);

  // ---- Motion profiles ---------------------------------------------------

  /**
   * Enable a trapezoidal motion profile for angular position control.
   *
   * @param maxVelocity     Maximum angular velocity constraint.
   * @param maxAcceleration Maximum angular acceleration constraint.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithTrapezoidProfile(
      units::turns_per_second_t maxVelocity, units::turns_per_second_squared_t maxAcceleration);

  /**
   * Enable a trapezoidal motion profile for linear position control.
   *
   * @param maxVelocity     Maximum linear velocity constraint.
   * @param maxAcceleration Maximum linear acceleration constraint.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithLinearTrapezoidProfile(
      units::meters_per_second_t maxVelocity, units::meters_per_second_squared_t maxAcceleration);

  /**
   * Enable a trapezoidal motion profile for velocity control.
   *
   * @param maxVelocity     Maximum velocity constraint.
   * @param maxAcceleration Maximum acceleration constraint.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithVelocityTrapezoidProfile(
      units::turns_per_second_t maxVelocity, units::turns_per_second_squared_t maxAcceleration);

  /**
   * Enable an exponential motion profile for position control.
   *
   * @param kV       Velocity constant.
   * @param kA       Acceleration constant.
   * @param maxInput Maximum voltage input.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithExponentialProfile(double kV, double kA, units::volt_t maxInput);

  /**
   * Derive an exponential motion profile from arm/flywheel system characteristics.
   *
   * Computes kV and kA from the motor model and moment of inertia via the
   * flywheel velocity state-space model.  Uses the gearing configured via
   * WithMotorGearing (defaults to 1:1 if not set).
   *
   * @param maxVolts Maximum input voltage.
   * @param motor    DC motor model.
   * @param moi      Moment of inertia of the mechanism.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithExponentialProfile(units::volt_t maxVolts, frc::DCMotor motor,
                                                     units::kilogram_square_meter_t moi);

  /**
   * Derive a linear exponential motion profile from elevator system characteristics.
   *
   * Computes kV and kA from the motor model, carriage mass, and drum radius via the
   * elevator velocity state-space model.  Also sets the mechanism circumference so
   * that linear closed-loop mode is activated.
   *
   * @param maxVolts   Maximum input voltage.
   * @param motor      DC motor model.
   * @param mass       Mass of the elevator carriage.
   * @param drumRadius Radius of the elevator drum.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithExponentialProfile(units::volt_t maxVolts, frc::DCMotor motor,
                                                     units::kilogram_t mass,
                                                     units::meter_t drumRadius);

  /**
   * Build an exponential motion profile from max velocity and acceleration constraints.
   *
   * @param maxVolts        Maximum input voltage.
   * @param maxVelocity     Maximum angular velocity.
   * @param maxAcceleration Maximum angular acceleration.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithExponentialProfile(
      units::volt_t maxVolts, units::turns_per_second_t maxVelocity,
      units::turns_per_second_squared_t maxAcceleration);

  /**
   * Set the exponential motion profile directly from a Constraints object.
   *
   * @param constraints Pre-built ExponentialProfile Constraints.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithExponentialProfile(
      frc::ExponentialProfile<units::turns, units::volts>::Constraints constraints);

  // ---- LQR ---------------------------------------------------------------

  /**
   * Attach an LQR configuration to the specified gain slot.
   *
   * @param lqrConfig LQRConfig to use.
   * @param slot      Gain slot to write (default SLOT_0).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithLQR(
      const math::LQRConfig& lqrConfig,
      ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);

  // ---- Gearing / linear --------------------------------------------------

  /**
   * Set the mechanism gearing (rotor-to-mechanism ratio).
   *
   * @param gearing MechanismGearing describing the drive train.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithMotorGearing(const gearing::MechanismGearing& gearing);

  /**
   * Set the mechanism circumference for linear distance conversion.
   *
   * @param circumference Wheel or drum circumference in meters.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithMechanismCircumference(units::meter_t circumference);

  /**
   * Set the mechanism circumference from a sprocket or gear pitch and tooth count.
   *
   * Circumference is computed as @p gearPitch × @p teeth.
   *
   * @param gearPitch Distance between teeth (e.g. 0.25_in for #25 chain).
   * @param teeth     Number of teeth on the sprocket or gear.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithMechanismCircumference(units::meter_t gearPitch, int teeth);

  /**
   * Set the mechanism circumference from a wheel or drum diameter.
   *
   * Circumference is computed as π × @p diameter.
   *
   * @param diameter Wheel or drum diameter.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithMechanismDiameter(units::meter_t diameter);

  /**
   * Set the mechanism circumference from a wheel or drum radius.
   *
   * Circumference is computed as 2π × @p radius.
   *
   * @param radius Wheel or drum radius.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithMechanismRadius(units::meter_t radius);

  // ---- Limits ------------------------------------------------------------

  /**
   * Set angular soft limits for the mechanism.
   *
   * @param lower Lower angle limit (in turns; accepts any angular unit via implicit conversion).
   * @param upper Upper angle limit (in turns; accepts any angular unit via implicit conversion).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithMechanismLimits(units::turn_t lower, units::turn_t upper);

  /**
   * Set linear soft limits for the mechanism.
   *
   * @param lower Lower distance limit.
   * @param upper Upper distance limit.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithMeasurementLimits(units::meter_t lower, units::meter_t upper);

  /**
   * Set the stator (output) current limit.
   *
   * @param limit Maximum stator current.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithStatorCurrentLimit(units::ampere_t limit);

  /**
   * Set the supply (input) current limit.
   *
   * @param limit Maximum supply current.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithSupplyCurrentLimit(units::ampere_t limit);

  /**
   * Set the temperature above which the motor controller disables output.
   *
   * @param temperature Cutoff temperature.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithTemperatureCutoff(units::celsius_t temperature);

  /**
   * Set the maximum voltage the closed-loop controller may output.
   *
   * @param maxVoltage Maximum closed-loop output voltage.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithClosedLoopMaxVoltage(units::volt_t maxVoltage);

  // ---- Control behaviour -------------------------------------------------

  /**
   * Set the idle (neutral) mode of the motor.
   *
   * @param mode COAST or BRAKE.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithIdleMode(MotorMode mode);

  /**
   * Switch to closed-loop control mode.
   *
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithClosedLoopMode();

  /**
   * Switch to open-loop control mode.
   *
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithOpenLoopMode();

  /**
   * Override the closed-loop control thread period.
   *
   * @param period Desired loop period.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithClosedLoopControlPeriod(units::second_t period);

  // ---- Ramp rates --------------------------------------------------------

  /**
   * Set the open-loop output ramp rate.
   *
   * @param rampRate Time to ramp from 0 to full output.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithOpenLoopRampRate(units::second_t rampRate);

  /**
   * Set the closed-loop output ramp rate.
   *
   * @param rampRate Time to ramp from 0 to full output.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithClosedLoopRampRate(units::second_t rampRate);

  // ---- Inversion ---------------------------------------------------------

  /**
   * Set the motor output direction.
   *
   * @param inverted true to invert the motor direction.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithMotorInverted(bool inverted);

  /**
   * Set the encoder count direction.
   *
   * @param inverted true to invert the encoder.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithEncoderInverted(bool inverted);

  // ---- External encoder --------------------------------------------------

  /**
   * Attach an external (absolute) encoder hardware object to this configuration.
   *
   * The encoder object is stored as std::any and unpacked inside the motor-controller
   * wrapper (e.g. SparkWrapper expects a `rev::spark::SparkAbsoluteEncoder*`;
   * TalonFXWrapper / TalonFXSWrapper accept a `ctre::phoenix6::hardware::CANcoder*`
   * or `ctre::phoenix6::hardware::CANdi*`).
   *
   * @note When passing a CANdi, the TalonFX(S) configuration must also have
   *       `Feedback.FeedbackSensorSource` (TalonFX) or
   *       `ExternalFeedback.ExternalFeedbackSensorSource` (TalonFXS) pre-set to one
   *       of `SyncCANdiPWM1`, `RemoteCANdiPWM1`, `SyncCANdiPWM2`, or
   *       `RemoteCANdiPWM2` via a vendor config so that ApplyConfig() knows which
   *       PWM channel to configure. Without this, offset, inversion, and
   *       discontinuity-point settings will not be applied to either channel.
   *
   * @param encoder External encoder hardware object pointer.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithExternalEncoder(std::any encoder);

  /**
   * Set whether the external encoder's position is used as the primary feedback
   * source for the closed-loop controller (default: true when an encoder is attached).
   *
   * @param use true to route external encoder to the PID feedback input.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithUseExternalFeedbackEncoder(bool use);

  /**
   * Invert the external encoder independently of the motor output direction.
   *
   * @param inverted true to invert the external encoder reading.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithExternalEncoderInverted(bool inverted);

  /**
   * Override the external encoder position/velocity conversion factor directly.
   *
   * When set this takes priority over the gearing-derived conversion factor from
   * WithExternalEncoderGearing. Prefer WithExternalEncoderGearing for type safety.
   *
   * @param factor Conversion factor (raw encoder units → mechanism turns).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithExternalEncoderConversionFactor(double factor);

  /**
   * Set the external encoder zero-point offset stored in the encoder hardware.
   *
   * @param zeroOffset Hardware zero offset (turns).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithExternalEncoderZeroOffset(units::turn_t zeroOffset);

  /**
   * Set the external encoder gearing (encoder to mechanism).
   *
   * Emits a driver station warning if the rotor-to-mechanism ratio exceeds 1, since
   * the encoder may alias and report duplicate angles.
   *
   * @param gearing MechanismGearing describing the external encoder drive train.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithExternalEncoderGearing(const gearing::MechanismGearing& gearing);

  /**
   * Set the external encoder gearing from a scalar reduction ratio.
   *
   * @param reductionRatio Reduction ratio (e.g. 3.0 for a 3:1 reduction).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithExternalEncoderGearing(double reductionRatio);

  /**
   * Set the discontinuity point for absolute encoder wrapping.
   *
   * Must be exactly 0.5_tr (encoder reads [−0.5, 0.5]) or 1_tr (reads [0, 1]).
   * Throws SmartMotorControllerConfigurationException otherwise.
   *
   * @param discontinuityPoint Wrap-around point (0.5 or 1 turns).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithExternalEncoderDiscontinuityPoint(
      units::turn_t discontinuityPoint);

  /**
   * Enable continuous position wrapping for the closed-loop controller.
   *
   * Throws SmartMotorControllerConfigurationException if soft limits or a linear
   * mechanism circumference are already configured.
   *
   * @param min Bottom of the wrapping range (turns).
   * @param max Top of the wrapping range (turns).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithContinuousWrapping(units::turn_t min, units::turn_t max);

  // ---- Telemetry ----------------------------------------------------------

  /**
   * Enable NetworkTables telemetry for this motor controller.
   *
   * @param name      Table key for this motor's telemetry.
   * @param verbosity Amount of data to publish (default HIGH).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithTelemetry(
      const std::string& name, TelemetryVerbosity verbosity = TelemetryVerbosity::HIGH);

  // ---- Subsystem ---------------------------------------------------------

  /**
   * Associate a command subsystem with this motor controller.
   *
   * @param subsystem Pointer to the owning subsystem (must outlive this config).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithSubsystem(frc2::SubsystemBase* subsystem);

  // ---- Simulation --------------------------------------------------------

  /**
   * Set the DC motor model used for simulation.
   *
   * @param motor frc::DCMotor model describing the motor physics.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithSimMotor(frc::DCMotor motor);

  // ---- Simulation overrides -----------------------------------------------

  /**
   * Override the starting mechanism position used in simulation.
   *
   * When running in simulation, GetStartingPosition() returns this value instead of the
   * value set by WithStartingPosition().
   *
   * @param startingAngle Starting angle override for simulation.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithSimStartingPosition(units::degree_t startingAngle);

  /**
   * Override the starting mechanism position (linear) used in simulation.
   *
   * WithMechanismCircumference must be set before calling this overload.
   *
   * @param startingDistance Starting linear distance override for simulation.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithSimStartingPosition(units::meter_t startingDistance);

  /**
   * Override the arm feedforward for the specified slot in simulation.
   *
   * @param ff   ArmFeedforward to use in simulation.
   * @param slot Gain slot to override (default SLOT_0).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithSimFeedforward(
      const frc::ArmFeedforward& ff,
      ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);

  /**
   * Override the elevator feedforward for the specified slot in simulation.
   *
   * @param ff   ElevatorFeedforward to use in simulation.
   * @param slot Gain slot to override (default SLOT_0).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithSimFeedforward(
      const frc::ElevatorFeedforward& ff,
      ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);

  /**
   * Override the simple motor feedforward for the specified slot in simulation.
   *
   * @param ff   SimpleMotorFeedforward to use in simulation.
   * @param slot Gain slot to override (default SLOT_0).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithSimFeedforward(
      const frc::SimpleMotorFeedforward<units::turns>& ff,
      ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);

  /**
   * Override the PID gains for the specified slot in simulation.
   *
   * @param kP   Proportional gain override.
   * @param kI   Integral gain override.
   * @param kD   Derivative gain override.
   * @param slot Gain slot to override (default SLOT_0).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithSimClosedLoopController(
      double kP, double kI, double kD,
      ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);

  /**
   * Override the angular trapezoidal motion profile used in simulation.
   *
   * @param maxVelocity     Maximum angular velocity for simulation.
   * @param maxAcceleration Maximum angular acceleration for simulation.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithSimTrapezoidProfile(
      units::turns_per_second_t maxVelocity, units::turns_per_second_squared_t maxAcceleration);

  /**
   * Override the linear trapezoidal motion profile used in simulation.
   *
   * @param maxVelocity     Maximum linear velocity for simulation.
   * @param maxAcceleration Maximum linear acceleration for simulation.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithSimTrapezoidProfile(
      units::meters_per_second_t maxVelocity, units::meters_per_second_squared_t maxAcceleration);

  /**
   * Override the angular exponential motion profile used in simulation.
   *
   * @param constraints ExponentialProfile constraints for simulation.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithSimExponentialProfile(
      frc::ExponentialProfile<units::turns, units::volts>::Constraints constraints);

  /**
   * Set the moment of inertia of the mechanism for simulation.
   *
   * @param moi Moment of inertia in kg·m².
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithMOI(units::kilogram_square_meter_t moi);

  /**
   * Estimate and set the moment of inertia for simulation from arm length and mass.
   *
   * Uses SingleJointedArmSim::EstimateMOI (1/3 * mass * length²).
   *
   * @param length Arm length.
   * @param mass   Arm mass.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithMOI(units::meter_t length, units::kilogram_t mass);

  /**
   * Set the starting mechanism position (seeds the encoder and sim objects on init).
   *
   * Throws std::invalid_argument if WithStartingPosition(meter_t) was already called.
   *
   * @param startingAngle Starting mechanism angle in degrees.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithStartingPosition(units::degree_t startingAngle);

  /**
   * Set the starting mechanism position from a linear distance.
   *
   * WithMechanismCircumference must be called before this overload. The distance is
   * converted to an angle internally. Throws std::invalid_argument if
   * WithMechanismCircumference has not been set, or if WithStartingPosition(degree_t)
   * was already called.
   *
   * @param startingDistance Starting linear distance (e.g. starting height for an elevator).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithStartingPosition(units::meter_t startingDistance);

  /**
   * Set a vendor-specific hardware configuration to use as the base for this motor controller.
   *
   * The provided object is used as the starting point before SmartMotorControllerConfig options
   * are applied on top. SmartMotorControllerConfig options always take precedence.
   *
   * Accepted types per wrapper:
   *  - TalonFXWrapper:  ctre::phoenix6::configs::TalonFXConfiguration
   *  - TalonFXSWrapper: ctre::phoenix6::configs::TalonFXSConfiguration
   *  - SparkWrapper (Max):  rev::spark::SparkMaxConfig
   *  - SparkWrapper (Flex): rev::spark::SparkFlexConfig
   *
   * Passing the wrong type for the wrapper will throw std::invalid_argument at construction time.
   *
   * @param cfg Vendor configuration object (stored via std::any).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithVendorConfig(std::any cfg);

  // ---- Followers -------------------------------------------------------------

  /**
   * Configure tightly coupled hardware followers for this motor controller.
   *
   * Followers must be the same vendor as the master (CTRE or REV).
   * TalonFX and TalonFXS are cross-compatible as Phoenix 6 followers.
   * Type checking is performed inside the wrapper's ApplyConfig; passing an
   * incompatible type emits a driver-station warning and is ignored.
   *
   * @param followers Vector of (hardware object pointer, opposeMasterDirection) pairs.
   *                  Hardware pointers are stored via std::any.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithFollowers(std::vector<std::pair<std::any, bool>> followers);

  /**
   * Configure loosely coupled SmartMotorController followers.
   *
   * Only position and velocity setpoint requests are forwarded to these followers;
   * configurations are not transferred.  Typically used when follower motors have
   * independent gains (e.g. inverted orientation) or a different controller brand.
   *
   * @param followers SmartMotorController instances to mirror setpoints.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithLooselyCoupledFollowers(
      std::vector<SmartMotorController*> followers);

  /** Get the list of tightly coupled hardware followers. */
  const std::vector<std::pair<std::any, bool>>& GetFollowers() const;

  /** Get the list of loosely coupled SmartMotorController followers. */
  const std::vector<SmartMotorController*>& GetLooselyCoupledFollowers() const;

  // ---- Clone -----------------------------------------------------------------

  /**
   * Return a deep copy of this configuration.
   *
   * Useful when multiple motor controllers share the same base config but
   * need independent modifications (e.g. different inversion settings).
   *
   * @return Copy of this SmartMotorControllerConfig.
   */
  SmartMotorControllerConfig Clone() const;

  // ---- Validation ------------------------------------------------------------

  /**
   * Reset the validation tracking sets so that a fresh ApplyConfig pass can be verified.
   *
   * Call this at the very start of ApplyConfig in every SmartMotorController wrapper.
   */
  void ResetValidationCheck() const;

  /**
   * Assert that every tracked basic config option was accessed during ApplyConfig.
   *
   * Throws SmartMotorControllerConfigurationException if any option was never fetched,
   * indicating that the wrapper's ApplyConfig implementation is incomplete.
   */
  void ValidateBasicOptions() const;

  /**
   * Assert that every tracked external-encoder config option was accessed during ApplyConfig.
   *
   * Throws SmartMotorControllerConfigurationException if any option was never fetched.
   */
  void ValidateExternalEncoderOptions() const;

  /**
   * Set a vendor-specific control request to use for position or velocity control.
   *
   * For TalonFX/TalonFXS: accepts any Phoenix 6 position or velocity ControlRequest
   * (e.g. PositionDutyCycle, MotionMagicTorqueCurrentFOC, VelocityDutyCycle).
   * The slot baked into the request object is honoured, overriding SetClosedLoopSlot.
   *
   * @param req Control request object (copied into std::any storage).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithVendorControlRequest(std::any req);

  // === Getters ============================================================

  /** Aggregated PID and feedforward gains for one closed-loop slot. */
  struct PIDGains {
    double kP{0.0}, kI{0.0}, kD{0.0};
    double kS{0.0}, kV{0.0}, kA{0.0}, kG{0.0};
    std::optional<frc::ArmFeedforward> armFF;
    std::optional<frc::ElevatorFeedforward> elevatorFF;
    std::optional<frc::SimpleMotorFeedforward<units::turns>> simpleFF;
    std::optional<math::LQRConfig> lqr;
  };

  /**
   * Get all gains for the specified closed-loop slot.
   *
   * In simulation, any overrides set via WithSimClosedLoopController or WithSimFeedforward
   * are merged into the returned copy.
   *
   * @param slot Gain slot to query.
   * @return PIDGains for that slot (simulation overrides applied when in sim).
   */
  PIDGains GetSlotGains(ClosedLoopControllerSlot slot) const;

  /**
   * Get the optional arm feedforward for the specified slot.
   *
   * @param slot Gain slot to query.
   * @return ArmFeedforward if configured, otherwise empty.
   */
  std::optional<frc::ArmFeedforward> GetArmFeedforward(ClosedLoopControllerSlot slot) const;

  /**
   * Get the optional elevator feedforward for the specified slot.
   *
   * @param slot Gain slot to query.
   * @return ElevatorFeedforward if configured, otherwise empty.
   */
  std::optional<frc::ElevatorFeedforward> GetElevatorFeedforward(
      ClosedLoopControllerSlot slot) const;

  /**
   * Get the optional simple motor feedforward for the specified slot.
   *
   * @param slot Gain slot to query.
   * @return SimpleMotorFeedforward if configured, otherwise empty.
   */
  std::optional<frc::SimpleMotorFeedforward<units::turns>> GetSimpleFeedforward(
      ClosedLoopControllerSlot slot) const;

  /**
   * Get the optional LQR configuration for the specified slot.
   *
   * @param slot Gain slot to query.
   * @return LQRConfig if configured, otherwise empty.
   */
  std::optional<math::LQRConfig> GetLQR(ClosedLoopControllerSlot slot) const;

  /**
   * Get the proportional gain for the specified slot.
   *
   * @param slot Gain slot to query (default SLOT_0).
   * @return kP value.
   */
  double GetKp(ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0) const;

  /**
   * Get the integral gain for the specified slot.
   *
   * @param slot Gain slot to query (default SLOT_0).
   * @return kI value.
   */
  double GetKi(ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0) const;

  /**
   * Get the derivative gain for the specified slot.
   *
   * @param slot Gain slot to query (default SLOT_0).
   * @return kD value.
   */
  double GetKd(ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0) const;

  /**
   * Return true if the config targets a linear (distance-based) closed-loop controller.
   *
   * @return true when a mechanism circumference has been set.
   */
  bool GetLinearClosedLoopControllerUse() const;

  /** @return Optional lower angular soft limit (in turns). */
  std::optional<units::turn_t> GetMechanismLowerLimit() const;
  /** @return Optional upper angular soft limit (in turns). */
  std::optional<units::turn_t> GetMechanismUpperLimit() const;
  /** @return Optional lower linear soft limit. */
  std::optional<units::meter_t> GetMeasurementLowerLimit() const;
  /** @return Optional upper linear soft limit. */
  std::optional<units::meter_t> GetMeasurementUpperLimit() const;

  /** @return Optional upper continuous wrapping bound (turns). Erases ContinuousWrapping tracking.
   */
  std::optional<units::turn_t> GetContinuousWrapping() const;
  /** @return Optional lower continuous wrapping bound (turns). */
  std::optional<units::turn_t> GetContinuousWrappingMin() const;

  /** @return Optional stator current limit. */
  std::optional<units::ampere_t> GetStatorCurrentLimit() const;
  /** @return Optional stator stall current limit (integer amps). */
  std::optional<int> GetStatorStallCurrentLimit() const;
  /** @return Optional supply stall current limit (integer amps). */
  std::optional<int> GetSupplyStallCurrentLimit() const;
  /** @return Optional supply current limit. */
  std::optional<units::ampere_t> GetSupplyCurrentLimit() const;
  /** @return Optional temperature cutoff. */
  std::optional<units::celsius_t> GetTemperatureCutoff() const;
  /** @return Optional maximum closed-loop output voltage. */
  std::optional<units::volt_t> GetClosedLoopControllerMaximumVoltage() const;

  /** @return Current control mode (CLOSED_LOOP or OPEN_LOOP). */
  ControlMode GetMotorControllerMode() const;
  /** @return Current idle mode (COAST or BRAKE). */
  MotorMode GetIdleMode() const;
  /** @return Optional closed-loop control thread period. */
  std::optional<units::second_t> GetClosedLoopControlPeriod() const;
  /** @return Optional open-loop ramp rate. */
  std::optional<units::second_t> GetOpenLoopRampRate() const;
  /** @return Optional closed-loop ramp rate. */
  std::optional<units::second_t> GetClosedLoopRampRate() const;

  /** @return Inverted state if explicitly configured, otherwise empty. */
  std::optional<bool> GetMotorInverted() const;
  /** @return Encoder inverted state if explicitly configured, otherwise empty. */
  std::optional<bool> GetEncoderInverted() const;
  /** @return true if a velocity trapezoidal profile is configured. */
  bool GetVelocityTrapezoidalProfileInUse() const;

  /** @return Optional NetworkTables telemetry name. */
  std::optional<std::string> GetTelemetryName() const;
  /** @return Optional telemetry verbosity level. */
  std::optional<TelemetryVerbosity> GetVerbosity() const;
  /** @return Owning subsystem pointer (may be nullptr). */
  frc2::SubsystemBase* GetSubsystem() const;
  /** @return Optional DC motor model for simulation. */
  std::optional<frc::DCMotor> GetSimMotor() const;
  /** @return Moment of inertia for simulation (kg·m²). */
  units::kilogram_square_meter_t GetMOI() const;
  /** @return Optional starting mechanism position (turns). */
  std::optional<units::turn_t> GetStartingPosition() const;
  /** @return Optional vendor-specific hardware config (set via WithVendorConfig). */
  std::optional<std::any> GetVendorConfig() const;
  /** @return Optional vendor-specific control request (set via WithVendorControlRequest). */
  std::optional<std::any> GetVendorControlRequest() const;

  /** @return Optional mechanism gearing. */
  const std::optional<gearing::MechanismGearing>& GetMotorGearing() const;
  /** @return Optional mechanism circumference for linear conversion. */
  std::optional<units::meter_t> GetMechanismCircumference() const;

  /** @return Optional external encoder hardware object (type-erased). */
  std::optional<std::any> GetExternalEncoder() const;
  /** @return true if the external encoder should be the PID feedback source. */
  bool GetUseExternalFeedback() const;
  /** @return Optional external encoder inversion override. */
  std::optional<bool> GetExternalEncoderInverted() const;
  /** @return Optional external encoder direct conversion factor override. */
  std::optional<double> GetExternalEncoderConversionFactor() const;
  /** @return Optional external encoder zero offset (turns). */
  std::optional<units::turn_t> GetExternalEncoderZeroOffset() const;
  /** @return Optional external encoder gearing. */
  const std::optional<gearing::MechanismGearing>& GetExternalEncoderGearing() const;
  /** @return Optional external encoder discontinuity point (turns). */
  std::optional<units::turn_t> GetExternalEncoderDiscontinuityPoint() const;

  /** @return true if a trapezoidal motion profile is configured. */
  bool HasTrapezoidProfile() const;
  /** @return true if an angular exponential motion profile is configured. */
  bool HasExponentialProfile() const;
  /** @return true if a linear (meters-based) exponential motion profile is configured. */
  bool HasLinearExponentialProfile() const;

  /** @return Optional angular trapezoidal profile. */
  std::optional<frc::TrapezoidProfile<units::turns>> GetTrapezoidProfile() const;
  /** @return Optional linear trapezoidal profile. */
  std::optional<frc::TrapezoidProfile<units::meters>> GetLinearTrapezoidProfile() const;
  /** @return Optional angular exponential profile. */
  std::optional<frc::ExponentialProfile<units::turns, units::volts>> GetExponentialProfile() const;
  /** @return Optional linear (meters-based) exponential profile. */
  std::optional<frc::ExponentialProfile<units::meters, units::volts>> GetLinearExponentialProfile()
      const;

  /** @return Optional max angular velocity constraint for hardware configuration. */
  std::optional<units::turns_per_second_t> GetTrapMaxVelocityTurns() const;
  /** @return Optional max angular acceleration constraint for hardware configuration. */
  std::optional<units::turns_per_second_squared_t> GetTrapMaxAccelTurns() const;
  /** @return Optional max linear velocity constraint for hardware configuration. */
  std::optional<units::meters_per_second_t> GetTrapMaxVelocityLinear() const;
  /** @return Optional max linear acceleration constraint for hardware configuration. */
  std::optional<units::meters_per_second_squared_t> GetTrapMaxAccelLinear() const;

  /** @return Optional kV for CTRE MotionMagicExpo (V*s/turn = V/(turn/s)). */
  std::optional<double> GetExponentialProfileKV() const;
  /** @return Optional kA for CTRE MotionMagicExpo (V*s²/turn = V/(turn/s²)). */
  std::optional<double> GetExponentialProfileKA() const;
  /** @return Optional maximum input voltage for the exponential profile. */
  std::optional<units::volt_t> GetExponentialProfileMaxInput() const;

  /**
   * Convert a mechanism position (turns) to a linear distance using the configured circumference.
   *
   * @param mechanismPosition Mechanism position in turns to convert.
   * @return Equivalent linear distance.
   */
  units::meter_t ConvertFromMechanism(units::turn_t mechanismPosition) const;

  /**
   * Convert a mechanism velocity (turns/s) to a linear velocity using the configured circumference.
   *
   * @param mechanismVelocity Mechanism velocity in turns/s to convert.
   * @return Equivalent linear velocity.
   */
  units::meters_per_second_t ConvertFromMechanism(
      units::turns_per_second_t mechanismVelocity) const;

 private:
  static constexpr int kNumSlots = 4;

  PIDGains m_slots[kNumSlots];
  int SlotIndex(ClosedLoopControllerSlot slot) const;

  // Validation tracking — options that every ApplyConfig implementation must access
  enum class BasicOptions {
    VendorControlRequest,
    ControlMode,
    ClosedLoopMaxVoltage,
    StartingPosition,
    EncoderInverted,
    MotorInverted,
    TemperatureCutoff,
    UpperLimit,
    LowerLimit,
    IdleMode,
    StatorCurrentLimit,
    SupplyCurrentLimit,
    ClosedLoopRampRate,
    OpenLoopRampRate,
    ExternalEncoder,
    Gearing,
    SlotGains,
    TrapezoidProfile,
    ExponentialProfile,
    ContinuousWrapping,
  };
  enum class ExternalEncoderOptions {
    ZeroOffset,
    DiscontinuityPoint,
    UseExternalFeedback,
    ExternalGearing,
    ExternalEncoderInverted,
  };
  static std::string_view ToString(BasicOptions opt);
  static std::string_view ToString(ExternalEncoderOptions opt);
  mutable std::set<BasicOptions> m_basicOptions;
  mutable std::set<ExternalEncoderOptions> m_externalEncoderOptions;

  // Per-slot simulation gain overrides
  struct SimGainsOverride {
    std::optional<double> kP, kI, kD;
    std::optional<frc::ArmFeedforward> armFF;
    std::optional<frc::ElevatorFeedforward> elevatorFF;
    std::optional<frc::SimpleMotorFeedforward<units::turns>> simpleFF;
  };
  SimGainsOverride m_simGains[kNumSlots];

  // Profiles
  std::optional<frc::TrapezoidProfile<units::turns>> m_trapProfile;
  std::optional<frc::TrapezoidProfile<units::meters>> m_linearTrapProfile;
  std::optional<frc::ExponentialProfile<units::turns, units::volts>> m_expoProfile;
  std::optional<frc::ExponentialProfile<units::meters, units::volts>> m_linearExpoProfile;
  bool m_velocityTrapProfile{false};
  // Stored constraint values for hardware motor controller configuration
  std::optional<units::turns_per_second_t> m_trapMaxVelTurns;
  std::optional<units::turns_per_second_squared_t> m_trapMaxAccTurns;
  std::optional<units::meters_per_second_t> m_trapMaxVelLinear;
  std::optional<units::meters_per_second_squared_t> m_trapMaxAccLinear;
  // kV/kA in V*s/turn and V*s²/turn for direct CTRE MotionMagicExpo assignment
  std::optional<double> m_expoMotionMagicKV;
  std::optional<double> m_expoMotionMagicKA;
  std::optional<units::volt_t> m_expoMaxInput;

  // Limits
  std::optional<units::turn_t> m_mechLowerLimit;
  std::optional<units::turn_t> m_mechUpperLimit;
  std::optional<units::meter_t> m_measLowerLimit;
  std::optional<units::meter_t> m_measUpperLimit;
  std::optional<units::ampere_t> m_statorCurrentLimit;
  std::optional<units::ampere_t> m_supplyCurrentLimit;
  std::optional<units::celsius_t> m_temperatureCutoff;
  std::optional<units::volt_t> m_closedLoopMaxVoltage;
  std::optional<units::turn_t> m_continuousWrappingMin;
  std::optional<units::turn_t> m_continuousWrappingMax;

  // Control behaviour
  ControlMode m_controlMode{ControlMode::CLOSED_LOOP};
  MotorMode m_idleMode{MotorMode::COAST};
  std::optional<units::second_t> m_closedLoopPeriod;
  std::optional<units::second_t> m_openLoopRampRate;
  std::optional<units::second_t> m_closedLoopRampRate;

  // Inversion — empty means the user never called WithMotorInverted / WithEncoderInverted.
  std::optional<bool> m_motorInverted;
  std::optional<bool> m_encoderInverted;

  // Gearing / linear
  std::optional<gearing::MechanismGearing> m_motorGearing;
  std::optional<units::meter_t> m_mechanismCircumference;

  // External encoder
  std::optional<std::any> m_externalEncoder;
  bool m_useExternalFeedback{true};
  std::optional<bool> m_externalEncoderInverted;
  std::optional<double> m_externalEncoderConversionFactor;
  std::optional<units::turn_t> m_externalEncoderZeroOffset;
  std::optional<gearing::MechanismGearing> m_externalEncoderGearing;
  std::optional<units::turn_t> m_externalEncoderDiscontinuityPoint;

  // Telemetry
  std::optional<std::string> m_telemetryName;
  std::optional<TelemetryVerbosity> m_verbosity;

  frc2::SubsystemBase* m_subsystem{nullptr};
  std::optional<frc::DCMotor> m_simMotor;
  units::kilogram_square_meter_t m_moi{0.0001_kg_sq_m};
  std::optional<units::turn_t> m_startingPosition;
  std::optional<units::meter_t> m_startingPositionDistance;

  // Simulation overrides
  std::optional<units::turn_t> m_simStartingPosition;
  std::optional<frc::TrapezoidProfile<units::turns>> m_simTrapProfile;
  std::optional<frc::TrapezoidProfile<units::meters>> m_simLinearTrapProfile;
  std::optional<frc::ExponentialProfile<units::turns, units::volts>> m_simExpoProfile;

  std::optional<std::any> m_vendorConfig;
  std::optional<std::any> m_vendorControlRequest;

  // Followers
  std::vector<std::pair<std::any, bool>> m_followers;
  std::vector<SmartMotorController*> m_looseFollowers;
};

}  // namespace yams::motorcontrollers
