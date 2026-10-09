// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <any>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#include <wpi/commands2/SubsystemBase.hpp>
#include <wpi/math/controller/ArmFeedforward.hpp>
#include <wpi/math/controller/ElevatorFeedforward.hpp>
#include <wpi/math/controller/SimpleMotorFeedforward.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/math/trajectory/ExponentialProfile.hpp>
#include <wpi/math/trajectory/TrapezoidProfile.hpp>
#include <wpi/units/acceleration.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_acceleration.hpp>
#include <wpi/units/angular_jerk.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/current.hpp>
#include <wpi/units/force.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/mass.hpp>
#include <wpi/units/moment_of_inertia.hpp>
#include <wpi/units/temperature.hpp>
#include <wpi/units/time.hpp>
#include <wpi/units/velocity.hpp>
#include <wpi/units/voltage.hpp>

#include "yams/gearing/MechanismGearing.hpp"
#include "yams/math/LQRConfig.hpp"

namespace yams::motorcontrollers {

// Forward declaration to avoid circular dependency (SmartMotorController.hpp
// includes this header, so we cannot include it here).
class SmartMotorController;

}  // namespace yams::motorcontrollers

namespace yams::telemetry {
// Forward declaration; SmartMotorControllerTelemetryConfig.hpp includes this header.
class SmartMotorControllerTelemetryConfig;
}  // namespace yams::telemetry

namespace yams::motorcontrollers {

/** Linear jerk in meters per second cubed (no predefined unit in wpi::units). */
using meters_per_second_cubed_t = wpi::units::unit_t<wpi::units::compound_unit<
    wpi::units::meters, wpi::units::inverse<wpi::units::cubed<wpi::units::seconds>>>>;

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

  // ---- Feedforward -------------------------------------------------------

  /**
   * Configure an arm feedforward model for the specified slot.
   *
   * @param ff   ArmFeedforward (kS, kG, kV, kA) to use.
   * @param slot Gain slot to write (default SLOT_0).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithFeedforward(
      const wpi::math::ArmFeedforward& ff,
      ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);

  /**
   * Configure an elevator feedforward model for the specified slot.
   *
   * @param ff   ElevatorFeedforward (kS, kG, kV, kA) to use.
   * @param slot Gain slot to write (default SLOT_0).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithFeedforward(
      const wpi::math::ElevatorFeedforward& ff,
      ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);

  /**
   * Configure a simple motor feedforward model for the specified slot.
   *
   * @param ff   SimpleMotorFeedforward (kS, kV, kA) to use.
   * @param slot Gain slot to write (default SLOT_0).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithFeedforward(
      const wpi::math::SimpleMotorFeedforward<wpi::units::turns>& ff,
      ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);

  // ---- Motion profiles ---------------------------------------------------

  /**
   * Enable a trapezoidal motion profile for angular position control.
   *
   * @param maxVelocity     Maximum angular velocity constraint.
   * @param maxAcceleration Maximum angular acceleration constraint.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithTrapezoidProfile(
      wpi::units::turns_per_second_t maxVelocity,
      wpi::units::turns_per_second_squared_t maxAcceleration);

  /**
   * Enable a trapezoidal motion profile for linear position control.
   *
   * @param maxVelocity     Maximum linear velocity constraint.
   * @param maxAcceleration Maximum linear acceleration constraint.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithLinearTrapezoidProfile(
      wpi::units::meters_per_second_t maxVelocity,
      wpi::units::meters_per_second_squared_t maxAcceleration);

  /**
   * Enable a trapezoidal motion profile for angular velocity control.
   *
   * The velocity setpoint is profiled: its rate of change is limited by @p maxAcceleration and
   * the rate of change of that by @p maxJerk.
   *
   * @param maxAcceleration Maximum mechanism angular acceleration.
   * @param maxJerk         Maximum mechanism angular jerk.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithVelocityTrapezoidProfile(
      wpi::units::turns_per_second_squared_t maxAcceleration,
      wpi::units::angular_jerk::turns_per_second_cubed_t maxJerk);

  /**
   * Enable a trapezoidal motion profile for linear velocity control.
   *
   * Also switches the closed-loop controller to linear (distance based) units.
   *
   * @param maxAcceleration Maximum linear acceleration.
   * @param maxJerk         Maximum linear jerk.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithVelocityTrapezoidProfile(
      wpi::units::meters_per_second_squared_t maxAcceleration, meters_per_second_cubed_t maxJerk);

  /**
   * Enable an exponential motion profile for position control.
   *
   * @param kV       Velocity constant.
   * @param kA       Acceleration constant.
   * @param maxInput Maximum voltage input.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithExponentialProfile(double kV, double kA,
                                                     wpi::units::volt_t maxInput);

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
  SmartMotorControllerConfig& WithExponentialProfile(wpi::units::volt_t maxVolts,
                                                     wpi::math::DCMotor motor,
                                                     wpi::units::kilogram_square_meter_t moi);

  /**
   * Derive a linear exponential motion profile from elevator system characteristics.
   *
   * Computes kV and kA from the motor model, carriage mass, and drum radius via the
   * elevator velocity state-space model.  Also sets the mechanism circumference and
   * activates linear closed-loop mode.
   *
   * @param maxVolts   Maximum input voltage.
   * @param motor      DC motor model.
   * @param mass       Mass of the elevator carriage.
   * @param drumRadius Radius of the elevator drum.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithExponentialProfile(wpi::units::volt_t maxVolts,
                                                     wpi::math::DCMotor motor,
                                                     wpi::units::kilogram_t mass,
                                                     wpi::units::meter_t drumRadius);

  /**
   * Build an exponential motion profile from max velocity and acceleration constraints.
   *
   * @param maxVolts        Maximum input voltage.
   * @param maxVelocity     Maximum angular velocity.
   * @param maxAcceleration Maximum angular acceleration.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithExponentialProfile(
      wpi::units::volt_t maxVolts, wpi::units::turns_per_second_t maxVelocity,
      wpi::units::turns_per_second_squared_t maxAcceleration);

  /**
   * Set the exponential motion profile directly from a Constraints object.
   *
   * @param constraints Pre-built ExponentialProfile Constraints.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithExponentialProfile(
      wpi::math::ExponentialProfile<wpi::units::turns, wpi::units::volts>::Constraints constraints);

  /**
   * Set a linear (meters based) exponential motion profile directly from its gains.
   *
   * Requires a mechanism circumference (throws SmartMotorControllerConfigurationException
   * otherwise) and switches the closed-loop controller to linear units.
   *
   * @param kV       Velocity constant in volts per meter per second.
   * @param kA       Acceleration constant in volts per meter per second squared.
   * @param maxInput Maximum voltage input.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithLinearExponentialProfile(double kV, double kA,
                                                           wpi::units::volt_t maxInput);

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

  /**
   * Set whether the closed-loop controller runs in linear (distance based) units.
   *
   * Linear mode only takes effect when a mechanism circumference is also configured.  It is
   * enabled automatically by elevator feedforwards, linear trapezoidal profiles, and the
   * elevator exponential profile.
   *
   * @param linear true for meters-based closed-loop control, false for rotations.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithLinearClosedLoopController(bool linear);

  /**
   * Set the closed-loop tolerance of the software PID controller in mechanism rotations.
   *
   * Throws SmartMotorControllerConfigurationException if no PID gains are configured.
   *
   * @param tolerance Position tolerance.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithClosedLoopTolerance(wpi::units::turn_t tolerance);

  /**
   * Set the closed-loop tolerance of the software PID controller as a distance.
   *
   * Throws SmartMotorControllerConfigurationException if the linear closed-loop controller is not
   * in use, the mechanism circumference is not configured, or no PID gains are configured.
   *
   * @param tolerance Distance tolerance.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithClosedLoopTolerance(wpi::units::meter_t tolerance);

  // ---- Gearing / linear --------------------------------------------------

  /**
   * Set the mechanism gearing (rotor-to-mechanism ratio).
   *
   * @param gearing MechanismGearing describing the drive train.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithMotorGearing(const gearing::MechanismGearing& gearing);

  /**
   * Set the mechanism gearing from a scalar reduction ratio.
   *
   * @param reductionRatio Reduction ratio (e.g. 3.0 for a 3:1 reduction, 0.5 for 1:2).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithMotorGearing(double reductionRatio);

  /**
   * Divide the configured gearing by the number of cascading elevator stages.
   *
   * Throws SmartMotorControllerConfigurationException if no gearing is configured.
   *
   * @param stages Number of cascading stages.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithCascadingElevatorStages(int stages);

  /**
   * Set the mechanism circumference for linear distance conversion.
   *
   * @param circumference Wheel or drum circumference in meters.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithMechanismCircumference(wpi::units::meter_t circumference);

  /**
   * Set the mechanism circumference from a sprocket or gear pitch and tooth count.
   *
   * Circumference is computed as @p gearPitch × @p teeth.
   *
   * @param gearPitch Distance between teeth (e.g. 0.25_in for #25 chain).
   * @param teeth     Number of teeth on the sprocket or gear.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithMechanismCircumference(wpi::units::meter_t gearPitch, int teeth);

  /**
   * Set the mechanism circumference from a wheel or drum diameter.
   *
   * Circumference is computed as π × @p diameter.
   *
   * @param diameter Wheel or drum diameter.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithMechanismDiameter(wpi::units::meter_t diameter);

  /**
   * Set the mechanism circumference from a wheel or drum radius.
   *
   * Circumference is computed as 2π × @p radius.
   *
   * @param radius Wheel or drum radius.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithMechanismRadius(wpi::units::meter_t radius);

  // ---- Limits ------------------------------------------------------------

  /**
   * Set angular soft limits for the mechanism.
   *
   * Throws SmartMotorControllerConfigurationException if @p lower is not below @p upper.
   *
   * @param lower Lower angle limit (in turns; accepts any angular unit via implicit conversion).
   * @param upper Upper angle limit (in turns; accepts any angular unit via implicit conversion).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithMechanismLimits(wpi::units::turn_t lower,
                                                  wpi::units::turn_t upper);

  /**
   * Set linear soft limits for the mechanism.
   *
   * The limits are converted into mechanism angle limits with the mechanism circumference, so
   * WithMechanismCircumference must be called first.  Throws
   * SmartMotorControllerConfigurationException if the circumference is not configured or if
   * @p lower is not below @p upper.
   *
   * @param lower Lower distance limit.
   * @param upper Upper distance limit.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithMeasurementLimits(wpi::units::meter_t lower,
                                                    wpi::units::meter_t upper);

  /**
   * Set the stator (output) current limit.
   *
   * @param limit Maximum stator current.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithStatorCurrentLimit(wpi::units::ampere_t limit);

  /**
   * Set the supply (input) current limit.
   *
   * @param limit Maximum supply current.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithSupplyCurrentLimit(wpi::units::ampere_t limit);

  /**
   * Set the temperature above which the motor controller disables output.
   *
   * @param temperature Cutoff temperature.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithTemperatureCutoff(wpi::units::celsius_t temperature);

  /**
   * Set the maximum voltage the closed-loop controller may output.
   *
   * @param maxVoltage Maximum closed-loop output voltage.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithClosedLoopMaxVoltage(wpi::units::volt_t maxVoltage);

  /**
   * Set the nominal voltage the motor controller compensates its output for.
   *
   * @param voltage Ideal (compensated) voltage.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithVoltageCompensation(wpi::units::volt_t voltage);

  /**
   * Set the angle error beyond which the relative encoder is resynchronized to the absolute
   * encoder.
   *
   * Throws SmartMotorControllerConfigurationException if a mechanism circumference is configured,
   * since auto-synchronization is unavailable for distance based mechanisms.
   *
   * @param threshold Synchronization threshold.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithFeedbackSynchronizationThreshold(wpi::units::turn_t threshold);

  // ---- Control behaviour -------------------------------------------------

  /**
   * Set the idle (neutral) mode of the motor.
   *
   * @param mode COAST or BRAKE.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithZeroPower(MotorMode mode);

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
  SmartMotorControllerConfig& WithClosedLoopControlPeriod(wpi::units::second_t period);

  /**
   * Set whether the motor controller resets its previous configuration and only applies what is
   * given to this config (default true).
   *
   * @param reset Reset the previous configuration.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithResetPreviousConfig(bool reset);

  // ---- Ramp rates --------------------------------------------------------

  /**
   * Set the open-loop output ramp rate.
   *
   * @param rampRate Time to ramp from 0 to full output.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithOpenLoopRampRate(wpi::units::second_t rampRate);

  /**
   * Set the closed-loop output ramp rate.
   *
   * @param rampRate Time to ramp from 0 to full output.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithClosedLoopRampRate(wpi::units::second_t rampRate);

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
  SmartMotorControllerConfig& WithExternalEncoderZeroOffset(wpi::units::turn_t zeroOffset);

  /**
   * Set the external encoder zero-point offset as a distance.
   *
   * Throws SmartMotorControllerConfigurationException if the mechanism circumference is not
   * configured.
   *
   * @param zeroOffset Hardware zero offset as a linear distance.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithExternalEncoderZeroOffset(wpi::units::meter_t zeroOffset);

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
      wpi::units::turn_t discontinuityPoint);

  /**
   * Enable continuous position wrapping for the closed-loop controller.
   *
   * Throws SmartMotorControllerConfigurationException if soft limits or a linear closed-loop
   * controller are already configured, or if no PID or LQR controller is configured.
   *
   * @param min Bottom of the wrapping range (turns).
   * @param max Top of the wrapping range (turns).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithContinuousWrapping(wpi::units::turn_t min,
                                                     wpi::units::turn_t max);

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

  /**
   * Enable NetworkTables telemetry for this motor controller under the name "motor".
   *
   * @param verbosity Amount of data to publish.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithTelemetry(TelemetryVerbosity verbosity);

  /**
   * Enable NetworkTables telemetry with a field-level telemetry configuration.
   *
   * The verbosity is set to HIGH so that live tuning is available.  The telemetry configuration
   * is used by the first motor controller that sets up telemetry with this config (copies made
   * with Clone() share it), so give each motor controller its own.
   *
   * @param name            Table key for this motor's telemetry.
   * @param telemetryConfig Telemetry configuration specifying the published fields.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithTelemetry(
      const std::string& name, telemetry::SmartMotorControllerTelemetryConfig telemetryConfig);

  // ---- Subsystem ---------------------------------------------------------

  /**
   * Associate a command subsystem with this motor controller.
   *
   * Throws SmartMotorControllerConfigurationException if a subsystem has already been set.
   * Passing nullptr leaves the subsystem unset.
   *
   * @param subsystem Pointer to the owning subsystem (must outlive this config).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithSubsystem(wpi::cmd::SubsystemBase* subsystem);

  // ---- Simulation --------------------------------------------------------

  /**
   * Set the DC motor model used for simulation.
   *
   * @param motor wpi::math::DCMotor model describing the motor physics.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithSimMotor(wpi::math::DCMotor motor);

  /**
   * Set the simulation period, the rate at which SimIterate() steps the simulated physics.
   *
   * Independent of the closed-loop control period.  Defaults to 20 ms.
   *
   * @param period Simulation loop period.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithSimulationPeriod(wpi::units::second_t period);

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
  SmartMotorControllerConfig& WithSimStartingPosition(wpi::units::degree_t startingAngle);

  /**
   * Override the starting mechanism position (linear) used in simulation.
   *
   * WithMechanismCircumference must be set before calling this overload.
   *
   * @param startingDistance Starting linear distance override for simulation.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithSimStartingPosition(wpi::units::meter_t startingDistance);

  /**
   * Override the arm feedforward for the specified slot in simulation.
   *
   * @param ff   ArmFeedforward to use in simulation.
   * @param slot Gain slot to override (default SLOT_0).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithSimFeedforward(
      const wpi::math::ArmFeedforward& ff,
      ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);

  /**
   * Override the elevator feedforward for the specified slot in simulation.
   *
   * @param ff   ElevatorFeedforward to use in simulation.
   * @param slot Gain slot to override (default SLOT_0).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithSimFeedforward(
      const wpi::math::ElevatorFeedforward& ff,
      ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);

  /**
   * Override the simple motor feedforward for the specified slot in simulation.
   *
   * @param ff   SimpleMotorFeedforward to use in simulation.
   * @param slot Gain slot to override (default SLOT_0).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithSimFeedforward(
      const wpi::math::SimpleMotorFeedforward<wpi::units::turns>& ff,
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
   * Override the closed-loop controller for the specified slot with an LQR in simulation.
   *
   * Clears any simulation PID override for that slot.
   *
   * @param lqrConfig LQRConfig to use in simulation.
   * @param slot      Gain slot to override (default SLOT_0).
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithSimClosedLoopController(
      const math::LQRConfig& lqrConfig,
      ClosedLoopControllerSlot slot = ClosedLoopControllerSlot::SLOT_0);

  /**
   * Override the angular trapezoidal motion profile used in simulation.
   *
   * @param maxVelocity     Maximum angular velocity for simulation.
   * @param maxAcceleration Maximum angular acceleration for simulation.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithSimTrapezoidProfile(
      wpi::units::turns_per_second_t maxVelocity,
      wpi::units::turns_per_second_squared_t maxAcceleration);

  /**
   * Override the linear trapezoidal motion profile used in simulation.
   *
   * @param maxVelocity     Maximum linear velocity for simulation.
   * @param maxAcceleration Maximum linear acceleration for simulation.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithSimTrapezoidProfile(
      wpi::units::meters_per_second_t maxVelocity,
      wpi::units::meters_per_second_squared_t maxAcceleration);

  /**
   * Override the angular exponential motion profile used in simulation.
   *
   * @param constraints ExponentialProfile constraints for simulation.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithSimExponentialProfile(
      wpi::math::ExponentialProfile<wpi::units::turns, wpi::units::volts>::Constraints constraints);

  /**
   * Set the moment of inertia of the mechanism for simulation.
   *
   * @param moi Moment of inertia in kg·m².
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithMOI(wpi::units::kilogram_square_meter_t moi);

  /**
   * Estimate and set the moment of inertia for simulation from arm length and mass.
   *
   * Uses SingleJointedArmSim::EstimateMOI (1/3 * mass * length²).
   *
   * @param length Arm length.
   * @param mass   Arm mass.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithMOI(wpi::units::meter_t length, wpi::units::kilogram_t mass);

  /**
   * Set the starting mechanism position (seeds the encoder and sim objects on init).
   *
   * Throws std::invalid_argument if WithStartingPosition(meter_t) was already called.
   *
   * @param startingAngle Starting mechanism angle in degrees.
   * @return *this for chaining.
   */
  SmartMotorControllerConfig& WithStartingPosition(wpi::units::degree_t startingAngle);

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
  SmartMotorControllerConfig& WithStartingPosition(wpi::units::meter_t startingDistance);

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

  /** Clear the tightly coupled hardware followers so they are not reapplied. */
  void ClearFollowers();

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
    std::optional<wpi::math::ArmFeedforward> armFF;
    std::optional<wpi::math::ElevatorFeedforward> elevatorFF;
    std::optional<wpi::math::SimpleMotorFeedforward<wpi::units::turns>> simpleFF;
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
  std::optional<wpi::math::ArmFeedforward> GetArmFeedforward(ClosedLoopControllerSlot slot) const;

  /**
   * Get the optional elevator feedforward for the specified slot.
   *
   * @param slot Gain slot to query.
   * @return ElevatorFeedforward if configured, otherwise empty.
   */
  std::optional<wpi::math::ElevatorFeedforward> GetElevatorFeedforward(
      ClosedLoopControllerSlot slot) const;

  /**
   * Get the optional simple motor feedforward for the specified slot.
   *
   * @param slot Gain slot to query.
   * @return SimpleMotorFeedforward if configured, otherwise empty.
   */
  std::optional<wpi::math::SimpleMotorFeedforward<wpi::units::turns>> GetSimpleFeedforward(
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
   * @return true when linear closed-loop control is enabled and a mechanism circumference is set.
   */
  bool GetLinearClosedLoopControllerUse() const;

  /** @return Optional lower angular soft limit (in turns). */
  std::optional<wpi::units::turn_t> GetMechanismLowerLimit() const;
  /** @return Optional upper angular soft limit (in turns). */
  std::optional<wpi::units::turn_t> GetMechanismUpperLimit() const;
  /** @return Optional lower linear soft limit. */
  std::optional<wpi::units::meter_t> GetMeasurementLowerLimit() const;
  /** @return Optional upper linear soft limit. */
  std::optional<wpi::units::meter_t> GetMeasurementUpperLimit() const;

  /** @return Optional upper continuous wrapping bound (turns). Erases ContinuousWrapping tracking.
   */
  std::optional<wpi::units::turn_t> GetContinuousWrapping() const;
  /** @return Optional lower continuous wrapping bound (turns). */
  std::optional<wpi::units::turn_t> GetContinuousWrappingMin() const;

  /**
   * Get the position setpoint to command with continuous wrapping: the angle equivalent to
   * @p setpoint, a whole number of wrapping ranges away, that is nearest @p current.
   *
   * @param setpoint Mechanism angle to go to.
   * @param current  Current mechanism angle.
   * @return The equivalent setpoint nearest @p current, or @p setpoint unchanged when continuous
   *         wrapping is not configured.
   */
  wpi::units::turn_t GetContinuousWrappingSetpoint(wpi::units::turn_t setpoint,
                                                   wpi::units::turn_t current) const;

  /** @return Optional stator current limit. */
  std::optional<wpi::units::ampere_t> GetStatorCurrentLimit() const;
  /** @return Optional stator stall current limit (integer amps). */
  std::optional<int> GetStatorStallCurrentLimit() const;
  /** @return Optional supply stall current limit (integer amps). */
  std::optional<int> GetSupplyStallCurrentLimit() const;
  /** @return Optional supply current limit. */
  std::optional<wpi::units::ampere_t> GetSupplyCurrentLimit() const;
  /** @return Optional temperature cutoff. */
  std::optional<wpi::units::celsius_t> GetTemperatureCutoff() const;
  /** @return Optional maximum closed-loop output voltage. */
  std::optional<wpi::units::volt_t> GetClosedLoopControllerMaximumVoltage() const;
  /** @return Optional voltage compensation (nominal) voltage. */
  std::optional<wpi::units::volt_t> GetVoltageCompensation() const;
  /** @return Optional relative-to-absolute encoder synchronization threshold. */
  std::optional<wpi::units::turn_t> GetFeedbackSynchronizationThreshold() const;
  /** @return Optional closed-loop tolerance in mechanism rotations. */
  std::optional<wpi::units::turn_t> GetClosedLoopTolerance() const;
  /** @return true if the previous motor controller configuration should be reset. */
  bool GetResetPreviousConfig() const;

  /** @return Current control mode (CLOSED_LOOP or OPEN_LOOP). */
  ControlMode GetMotorControllerMode() const;
  /** @return Zero power mode (COAST or BRAKE) if configured, otherwise empty. */
  std::optional<MotorMode> GetZeroPower() const;
  /** @return Optional closed-loop control thread period. */
  std::optional<wpi::units::second_t> GetClosedLoopControlPeriod() const;
  /** @return Optional open-loop ramp rate. */
  std::optional<wpi::units::second_t> GetOpenLoopRampRate() const;
  /** @return Optional closed-loop ramp rate. */
  std::optional<wpi::units::second_t> GetClosedLoopRampRate() const;

  /**
   * @return Inverted state if explicitly configured, otherwise empty.  Always false in
   *         simulation when configured, since simulated physics are not inverted.
   */
  std::optional<bool> GetMotorInverted() const;
  /** @return Encoder inverted state if explicitly configured, otherwise empty. */
  std::optional<bool> GetEncoderInverted() const;
  /** @return true if a velocity trapezoidal profile is configured. */
  bool GetVelocityTrapezoidalProfileInUse() const;

  /** @return Optional NetworkTables telemetry name. */
  std::optional<std::string> GetTelemetryName() const;
  /** @return Optional telemetry verbosity level. */
  std::optional<TelemetryVerbosity> GetVerbosity() const;
  /**
   * @return Owning subsystem pointer.  Throws SmartMotorControllerConfigurationException if no
   *         subsystem has been set.
   */
  wpi::cmd::SubsystemBase* GetSubsystem() const;
  /** @return true if a subsystem has been set via WithSubsystem. */
  bool HasSubsystem() const;
  /**
   * @return Telemetry configuration set via WithTelemetry(name, telemetryConfig), or nullptr if
   *         none was given.
   */
  std::shared_ptr<telemetry::SmartMotorControllerTelemetryConfig>
  GetSmartControllerTelemetryConfig() const;
  /** @return Optional DC motor model for simulation. */
  std::optional<wpi::math::DCMotor> GetSimMotor() const;
  /** @return Simulation loop period (default 20 ms). */
  wpi::units::second_t GetSimulationPeriod() const;
  /** @return Moment of inertia for simulation (kg·m²). */
  wpi::units::kilogram_square_meter_t GetMOI() const;
  /** @return Optional starting mechanism position (turns). */
  std::optional<wpi::units::turn_t> GetStartingPosition() const;
  /** @return Optional vendor-specific hardware config (set via WithVendorConfig). */
  std::optional<std::any> GetVendorConfig() const;
  /** @return Optional vendor-specific control request (set via WithVendorControlRequest). */
  std::optional<std::any> GetVendorControlRequest() const;

  /** @return Optional mechanism gearing. */
  const std::optional<gearing::MechanismGearing>& GetMotorGearing() const;
  /** @return Optional mechanism circumference for linear conversion. */
  std::optional<wpi::units::meter_t> GetMechanismCircumference() const;

  /** @return Optional external encoder hardware object (type-erased). */
  std::optional<std::any> GetExternalEncoder() const;
  /** @return true if the external encoder should be the PID feedback source. */
  bool GetUseExternalFeedback() const;
  /**
   * @return Optional external encoder inversion override.  Always false in simulation when
   *         configured, since simulated sensors are not inverted.
   */
  std::optional<bool> GetExternalEncoderInverted() const;
  /** @return Optional external encoder direct conversion factor override. */
  std::optional<double> GetExternalEncoderConversionFactor() const;
  /** @return Optional external encoder zero offset (turns). */
  std::optional<wpi::units::turn_t> GetExternalEncoderZeroOffset() const;
  /** @return Optional external encoder gearing. */
  const std::optional<gearing::MechanismGearing>& GetExternalEncoderGearing() const;
  /** @return Optional external encoder discontinuity point (turns). */
  std::optional<wpi::units::turn_t> GetExternalEncoderDiscontinuityPoint() const;

  /** @return true if a trapezoidal motion profile is configured. */
  bool HasTrapezoidProfile() const;
  /** @return true if an angular exponential motion profile is configured. */
  bool HasExponentialProfile() const;
  /** @return true if a linear (meters-based) exponential motion profile is configured. */
  bool HasLinearExponentialProfile() const;

  /** @return Optional angular trapezoidal profile. */
  std::optional<wpi::math::TrapezoidProfile<wpi::units::turns>> GetTrapezoidProfile() const;
  /** @return Optional linear trapezoidal profile. */
  std::optional<wpi::math::TrapezoidProfile<wpi::units::meters>> GetLinearTrapezoidProfile() const;
  /** @return Optional angular exponential profile. */
  std::optional<wpi::math::ExponentialProfile<wpi::units::turns, wpi::units::volts>>
  GetExponentialProfile() const;
  /** @return Optional linear (meters-based) exponential profile. */
  std::optional<wpi::math::ExponentialProfile<wpi::units::meters, wpi::units::volts>>
  GetLinearExponentialProfile() const;

  /** @return Optional max angular velocity constraint for hardware configuration. */
  std::optional<wpi::units::turns_per_second_t> GetTrapMaxVelocityTurns() const;
  /** @return Optional max angular acceleration constraint for hardware configuration. */
  std::optional<wpi::units::turns_per_second_squared_t> GetTrapMaxAccelTurns() const;
  /** @return Optional max linear velocity constraint for hardware configuration. */
  std::optional<wpi::units::meters_per_second_t> GetTrapMaxVelocityLinear() const;
  /** @return Optional max linear acceleration constraint for hardware configuration. */
  std::optional<wpi::units::meters_per_second_squared_t> GetTrapMaxAccelLinear() const;

  /** @return Optional kV for CTRE MotionMagicExpo (V*s/turn = V/(turn/s)). */
  std::optional<double> GetExponentialProfileKV() const;
  /** @return Optional kA for CTRE MotionMagicExpo (V*s²/turn = V/(turn/s²)). */
  std::optional<double> GetExponentialProfileKA() const;
  /** @return Optional maximum input voltage for the exponential profile. */
  std::optional<wpi::units::volt_t> GetExponentialProfileMaxInput() const;

  /**
   * Convert a mechanism position (turns) to a linear distance using the configured circumference.
   *
   * All ConvertFromMechanism/ConvertToMechanism/ConvertToVoltage/ConvertToCurrent overloads throw
   * SmartMotorControllerConfigurationException if the mechanism circumference is not configured.
   *
   * @param mechanismPosition Mechanism position in turns to convert.
   * @return Equivalent linear distance.
   */
  wpi::units::meter_t ConvertFromMechanism(wpi::units::turn_t mechanismPosition) const;

  /**
   * Convert a mechanism velocity (turns/s) to a linear velocity using the configured circumference.
   *
   * @param mechanismVelocity Mechanism velocity in turns/s to convert.
   * @return Equivalent linear velocity.
   */
  wpi::units::meters_per_second_t ConvertFromMechanism(
      wpi::units::turns_per_second_t mechanismVelocity) const;

  /**
   * Convert a mechanism acceleration to a linear acceleration using the configured circumference.
   *
   * @param mechanismAcceleration Mechanism acceleration to convert.
   * @return Equivalent linear acceleration.
   */
  wpi::units::meters_per_second_squared_t ConvertFromMechanism(
      wpi::units::turns_per_second_squared_t mechanismAcceleration) const;

  /**
   * Convert a mechanism jerk to a linear jerk using the configured circumference.
   *
   * @param mechanismJerk Mechanism jerk to convert.
   * @return Equivalent linear jerk.
   */
  meters_per_second_cubed_t ConvertFromMechanism(
      wpi::units::angular_jerk::turns_per_second_cubed_t mechanismJerk) const;

  /**
   * Convert a linear distance to a mechanism position using the configured circumference.
   *
   * @param distance Linear distance to convert.
   * @return Equivalent mechanism position.
   */
  wpi::units::turn_t ConvertToMechanism(wpi::units::meter_t distance) const;

  /**
   * Convert a linear velocity to a mechanism velocity using the configured circumference.
   *
   * @param velocity Linear velocity to convert.
   * @return Equivalent mechanism velocity.
   */
  wpi::units::turns_per_second_t ConvertToMechanism(wpi::units::meters_per_second_t velocity) const;

  /**
   * Convert a linear acceleration to a mechanism acceleration using the configured circumference.
   *
   * @param acceleration Linear acceleration to convert.
   * @return Equivalent mechanism acceleration.
   */
  wpi::units::turns_per_second_squared_t ConvertToMechanism(
      wpi::units::meters_per_second_squared_t acceleration) const;

  /**
   * Convert a linear jerk to a mechanism jerk using the configured circumference.
   *
   * @param jerk Linear jerk to convert.
   * @return Equivalent mechanism jerk.
   */
  wpi::units::angular_jerk::turns_per_second_cubed_t ConvertToMechanism(
      meters_per_second_cubed_t jerk) const;

  /**
   * Convert a feedforward force applied at the mechanism into the equivalent motor voltage, using
   * the gearing and mechanism circumference.  Only the voltage producing the force (the resistive
   * drop of the matching current) is returned, not the back-EMF of the commanded speed.
   *
   * @param motor            DC motor model of the mechanism.
   * @param feedforwardForce Feedforward force applied to the mechanism.
   * @return Equivalent feedforward voltage at the motor.
   */
  wpi::units::volt_t ConvertToVoltage(const wpi::math::DCMotor& motor,
                                      wpi::units::newton_t feedforwardForce) const;

  /**
   * Convert a feedforward force applied at the mechanism into the equivalent motor current, using
   * the gearing and mechanism circumference (e.g. for torque-current closed-loop control).
   *
   * @param motor            DC motor model of the mechanism.
   * @param feedforwardForce Feedforward force applied to the mechanism.
   * @return Equivalent feedforward current at the motor.
   */
  wpi::units::ampere_t ConvertToCurrent(const wpi::math::DCMotor& motor,
                                        wpi::units::newton_t feedforwardForce) const;

 private:
  static constexpr int kNumSlots = 4;

  PIDGains m_slots[kNumSlots];
  // Slots that received PID gains via WithFeedback (zero gains still count as configured).
  bool m_slotHasFeedback[kNumSlots]{false, false, false, false};
  int SlotIndex(ClosedLoopControllerSlot slot) const;

  // Validation tracking options that every ApplyConfig implementation must access
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
    ZeroPower,
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
    Followers,
    LooselyCoupledFollowers,
    VoltageCompensation,
    FeedbackSynchronizationThreshold,
    ClosedLoopTolerance,
    ClosedLoopControlPeriod,
    ResetPreviousConfig,
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

  /** Rotor torque (N·m) equivalent to a force at the mechanism; throws without circumference. */
  double ForceToRotorTorque(wpi::units::newton_t feedforwardForce) const;
  /** Throw SmartMotorControllerConfigurationException if no circumference is configured. */
  void RequireCircumference(const std::string& action) const;
  /** @return true if any slot has PID gains or an LQR configured. */
  bool HasClosedLoopController() const;

  // Per-slot simulation gain overrides
  struct SimGainsOverride {
    std::optional<double> kP, kI, kD;
    std::optional<math::LQRConfig> lqr;
    std::optional<wpi::math::ArmFeedforward> armFF;
    std::optional<wpi::math::ElevatorFeedforward> elevatorFF;
    std::optional<wpi::math::SimpleMotorFeedforward<wpi::units::turns>> simpleFF;
  };
  SimGainsOverride m_simGains[kNumSlots];

  // Profiles
  std::optional<wpi::math::TrapezoidProfile<wpi::units::turns>> m_trapProfile;
  std::optional<wpi::math::TrapezoidProfile<wpi::units::meters>> m_linearTrapProfile;
  std::optional<wpi::math::ExponentialProfile<wpi::units::turns, wpi::units::volts>> m_expoProfile;
  std::optional<wpi::math::ExponentialProfile<wpi::units::meters, wpi::units::volts>>
      m_linearExpoProfile;
  bool m_velocityTrapProfile{false};
  // Stored constraint values for hardware motor controller configuration
  std::optional<wpi::units::turns_per_second_t> m_trapMaxVelTurns;
  std::optional<wpi::units::turns_per_second_squared_t> m_trapMaxAccTurns;
  std::optional<wpi::units::meters_per_second_t> m_trapMaxVelLinear;
  std::optional<wpi::units::meters_per_second_squared_t> m_trapMaxAccLinear;
  // kV/kA in V*s/turn and V*s²/turn for direct CTRE MotionMagicExpo assignment
  std::optional<double> m_expoMotionMagicKV;
  std::optional<double> m_expoMotionMagicKA;
  std::optional<wpi::units::volt_t> m_expoMaxInput;

  // Limits
  std::optional<wpi::units::turn_t> m_mechLowerLimit;
  std::optional<wpi::units::turn_t> m_mechUpperLimit;
  std::optional<wpi::units::ampere_t> m_statorCurrentLimit;
  std::optional<wpi::units::ampere_t> m_supplyCurrentLimit;
  std::optional<wpi::units::celsius_t> m_temperatureCutoff;
  std::optional<wpi::units::volt_t> m_closedLoopMaxVoltage;
  std::optional<wpi::units::volt_t> m_voltageCompensation;
  std::optional<wpi::units::turn_t> m_feedbackSynchronizationThreshold;
  std::optional<wpi::units::turn_t> m_closedLoopTolerance;
  bool m_resetPreviousConfig{true};
  std::optional<wpi::units::turn_t> m_continuousWrappingMin;
  std::optional<wpi::units::turn_t> m_continuousWrappingMax;

  // Control behaviour
  ControlMode m_controlMode{ControlMode::CLOSED_LOOP};
  std::optional<MotorMode> m_zeroPower;
  std::optional<wpi::units::second_t> m_closedLoopPeriod;
  std::optional<wpi::units::second_t> m_openLoopRampRate;
  std::optional<wpi::units::second_t> m_closedLoopRampRate;

  // Inversion empty means the user never called WithMotorInverted / WithEncoderInverted.
  std::optional<bool> m_motorInverted;
  std::optional<bool> m_encoderInverted;

  // Gearing / linear
  std::optional<gearing::MechanismGearing> m_motorGearing;
  std::optional<wpi::units::meter_t> m_mechanismCircumference;
  bool m_linearClosedLoopController{false};

  // External encoder
  std::optional<std::any> m_externalEncoder;
  bool m_useExternalFeedback{true};
  std::optional<bool> m_externalEncoderInverted;
  std::optional<double> m_externalEncoderConversionFactor;
  std::optional<wpi::units::turn_t> m_externalEncoderZeroOffset;
  std::optional<gearing::MechanismGearing> m_externalEncoderGearing;
  std::optional<wpi::units::turn_t> m_externalEncoderDiscontinuityPoint;

  // Telemetry
  std::optional<std::string> m_telemetryName;
  std::optional<TelemetryVerbosity> m_verbosity;
  // shared_ptr so the forward-declared, non-copyable type can be held.
  std::shared_ptr<telemetry::SmartMotorControllerTelemetryConfig> m_telemetryConfig;

  wpi::cmd::SubsystemBase* m_subsystem{nullptr};
  std::optional<wpi::math::DCMotor> m_simMotor;
  wpi::units::kilogram_square_meter_t m_moi{0.02_kg_sq_m};
  std::optional<wpi::units::second_t> m_simulationPeriod;
  std::optional<wpi::units::turn_t> m_startingPosition;
  std::optional<wpi::units::meter_t> m_startingPositionDistance;

  // Simulation overrides
  std::optional<wpi::units::turn_t> m_simStartingPosition;
  std::optional<wpi::math::TrapezoidProfile<wpi::units::turns>> m_simTrapProfile;
  std::optional<wpi::math::TrapezoidProfile<wpi::units::meters>> m_simLinearTrapProfile;
  std::optional<wpi::math::ExponentialProfile<wpi::units::turns, wpi::units::volts>>
      m_simExpoProfile;

  std::optional<std::any> m_vendorConfig;
  std::optional<std::any> m_vendorControlRequest;

  // Followers
  std::vector<std::pair<std::any, bool>> m_followers;
  std::vector<SmartMotorController*> m_looseFollowers;
};

}  // namespace yams::motorcontrollers
