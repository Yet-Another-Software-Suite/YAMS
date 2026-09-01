// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <wpi/system/Notifier.hpp>
#include <wpi/math/controller/ArmFeedforward.hpp>
#include <wpi/math/controller/ElevatorFeedforward.hpp>
#include <wpi/math/controller/PIDController.hpp>
#include <wpi/math/controller/SimpleMotorFeedforward.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/math/trajectory/ExponentialProfile.hpp>
#include <wpi/math/trajectory/TrapezoidProfile.hpp>
#include <wpi/nt/NetworkTable.hpp>
#include <wpi/units/acceleration.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_acceleration.hpp>
#include <wpi/units/angular_jerk.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/current.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/temperature.hpp>
#include <wpi/units/time.hpp>
#include <wpi/units/velocity.hpp>
#include <wpi/units/voltage.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "SimSupplier.hpp"
#include "SmartMotorControllerConfig.hpp"
#include "yams/math/LQRController.hpp"
#include "yams/telemetry/SmartMotorControllerTelemetry.hpp"
#include "yams/telemetry/SmartMotorControllerTelemetryConfig.hpp"

namespace yams::motorcontrollers {

/**
 * Abstract base class for all YAMS motor controller wrappers.
 *
 * Provides a hardware-agnostic interface for open-loop control, closed-loop setpoints,
 * encoder reads/writes, simulation, telemetry, and SysId characterization.  Concrete
 * implementations wrap CTRE TalonFX/TalonFXS and REV SPARK Max/Flex.
 */
class SmartMotorController {
 public:
  using ClosedLoopControllerSlot = SmartMotorControllerConfig::ClosedLoopControllerSlot;
  using MotorMode = SmartMotorControllerConfig::MotorMode;
  using ControlMode = SmartMotorControllerConfig::ControlMode;

  virtual ~SmartMotorController() = default;

  // ---- Configuration ------------------------------------------------------

  /**
   * Apply a SmartMotorControllerConfig to the underlying hardware.
   *
   * @param config Configuration to apply.
   * @return true if the configuration was applied successfully.
   */
  virtual bool ApplyConfig(const SmartMotorControllerConfig& config) = 0;

  // ---- Simulation ---------------------------------------------------------

  /** Initialize the DCMotorSim used for simulation. */
  virtual void SetupSimulation() = 0;

  /** Advance the motor simulation by one loop iteration. */
  virtual void SimIterate() = 0;

  /**
   * Attach a SimSupplier that provides physics-model position and velocity for
   * simulation.  When set, mechanism SimIterate() calls will use this supplier
   * instead of the internal DCMotorSim.
   *
   * @param supplier Shared SimSupplier instance, or nullptr to detach.
   */
  void SetSimSupplier(std::shared_ptr<SimSupplier> supplier);

  /**
   * Get the attached SimSupplier, or nullptr if none is set.
   *
   * @return Raw pointer to the SimSupplier (valid as long as the shared_ptr is held).
   */
  SimSupplier* GetSimSupplier() const;

  // ---- Encoder sync -------------------------------------------------------

  /** Seed the relative encoder from the absolute sensor position (one-shot on startup). */
  virtual void SeedRelativeEncoder() = 0;

  /** Periodically re-synchronize the relative encoder to the absolute sensor. */
  virtual void SynchronizeRelativeEncoder() = 0;

  // ---- Open-loop outputs --------------------------------------------------

  /**
   * Command a duty-cycle output to the motor.
   *
   * @param dutyCycle Normalized duty cycle in [-1, 1].
   */
  virtual void SetDutyCycle(double dutyCycle) = 0;

  /**
   * Get the current motor duty cycle.
   *
   * @return Duty cycle in [-1, 1].
   */
  virtual double GetDutyCycle() = 0;

  /**
   * Command a voltage output to the motor.
   *
   * @param voltage Voltage to apply.
   */
  virtual void SetVoltage(wpi::units::volt_t voltage) = 0;

  /**
   * Get the voltage currently applied to the motor.
   *
   * @return Applied voltage.
   */
  virtual wpi::units::volt_t GetVoltage() = 0;

  // ---- Closed-loop setpoints ----------------------------------------------

  /**
   * Command a mechanism angle position setpoint (closed-loop).
   *
   * @param angle Target mechanism angle.
   */
  virtual void SetPosition(wpi::units::turn_t angle) = 0;

  /**
   * Command a linear measurement position setpoint (closed-loop).
   *
   * @param distance Target linear distance.
   */
  virtual void SetPosition(wpi::units::meter_t distance) = 0;

  /**
   * Command a mechanism angular velocity setpoint (closed-loop).
   *
   * @param velocity Target mechanism angular velocity.
   */
  virtual void SetVelocity(wpi::units::turns_per_second_t velocity) = 0;

  /**
   * Command a linear measurement velocity setpoint (closed-loop).
   *
   * @param velocity Target linear velocity.
   */
  virtual void SetVelocity(wpi::units::meters_per_second_t velocity) = 0;

  // ---- Encoder writes -----------------------------------------------------

  /**
   * Write a mechanism angle into the encoder (seeds the position).
   *
   * @param angle Mechanism angle to write.
   */
  virtual void SetEncoderPosition(wpi::units::turn_t angle) = 0;

  /**
   * Write a linear distance into the encoder (seeds the position).
   *
   * @param distance Linear distance to write.
   */
  virtual void SetEncoderPosition(wpi::units::meter_t distance) = 0;

  /**
   * Write a mechanism angular velocity into the encoder.
   *
   * @param velocity Angular velocity to write.
   */
  virtual void SetEncoderVelocity(wpi::units::turns_per_second_t velocity) = 0;

  /**
   * Write a linear velocity into the encoder.
   *
   * @param velocity Linear velocity to write.
   */
  virtual void SetEncoderVelocity(wpi::units::meters_per_second_t velocity) = 0;

  // ---- Encoder reads ------------------------------------------------------

  /**
   * Get the mechanism position from the motor encoder (applies gearing).
   *
   * @return Mechanism position in turns.
   */
  virtual wpi::units::turn_t GetMechanismPosition() = 0;

  /**
   * Get the mechanism velocity from the motor encoder (applies gearing).
   *
   * @return Mechanism velocity in turns per second.
   */
  virtual wpi::units::turns_per_second_t GetMechanismVelocity() = 0;

  /**
   * Get the mechanism acceleration (derived from velocity).
   *
   * @return Mechanism acceleration in turns per second squared.
   */
  virtual wpi::units::turns_per_second_squared_t GetMechanismAcceleration() = 0;

  /**
   * Get the raw rotor position (no gearing applied).
   *
   * @return Rotor position in turns.
   */
  virtual wpi::units::turn_t GetRotorPosition() = 0;

  /**
   * Get the raw rotor velocity (no gearing applied).
   *
   * @return Rotor velocity in turns per second.
   */
  virtual wpi::units::turns_per_second_t GetRotorVelocity() = 0;

  /**
   * Get the linear measurement position (applies circumference conversion).
   *
   * @return Linear position in meters.
   */
  virtual wpi::units::meter_t GetMeasurementPosition() = 0;

  /**
   * Get the linear measurement velocity (applies circumference conversion).
   *
   * @return Linear velocity in meters per second.
   */
  virtual wpi::units::meters_per_second_t GetMeasurementVelocity() = 0;

  /**
   * Get the linear measurement acceleration.
   *
   * @return Linear acceleration in meters per second squared.
   */
  virtual wpi::units::meters_per_second_squared_t GetMeasurementAcceleration() = 0;

  /**
   * Get the position of the attached external (absolute) encoder if available.
   *
   * @return External encoder position, or empty if not present or not configured.
   */
  virtual std::optional<wpi::units::degree_t> GetExternalEncoderPosition() = 0;

  /**
   * Get the velocity of the attached external (absolute) encoder if available.
   *
   * @return External encoder velocity, or empty if not present or not configured.
   */
  virtual std::optional<wpi::units::degrees_per_second_t> GetExternalEncoderVelocity() = 0;

  // ---- Motor status -------------------------------------------------------

  /**
   * Get the supply (input) current draw.
   *
   * @return Supply current, or empty if not supported by the hardware.
   */
  virtual std::optional<wpi::units::ampere_t> GetSupplyCurrent() = 0;

  /**
   * Get the stator (output) current draw.
   *
   * @return Stator current.
   */
  virtual wpi::units::ampere_t GetStatorCurrent() = 0;

  /**
   * Get the motor controller temperature.
   *
   * @return Temperature in degrees Celsius.
   */
  virtual wpi::units::celsius_t GetTemperature() = 0;

  /**
   * Get the DC motor model associated with this controller.
   *
   * @return wpi::math::DCMotor model.
   */
  virtual wpi::math::DCMotor GetDCMotor() = 0;

  // ---- Configuration setters (live tuning) --------------------------------

  /**
   * Set the idle (neutral) mode of the motor.
   *
   * @param mode COAST or BRAKE.
   */
  virtual void SetIdleMode(MotorMode mode) = 0;

  /**
   * Set the motor output direction.
   *
   * @param inverted true to invert the motor direction.
   */
  virtual void SetMotorInverted(bool inverted) = 0;

  /**
   * Set the encoder count direction.
   *
   * @param inverted true to invert the encoder.
   */
  virtual void SetEncoderInverted(bool inverted) = 0;

  /**
   * Set the proportional gain (live update).
   *
   * @param kP Proportional gain.
   */
  virtual void SetKp(double kP) = 0;

  /**
   * Set the integral gain (live update).
   *
   * @param kI Integral gain.
   */
  virtual void SetKi(double kI) = 0;

  /**
   * Set the derivative gain (live update).
   *
   * @param kD Derivative gain.
   */
  virtual void SetKd(double kD) = 0;

  /**
   * Set all three PID feedback gains at once (live update).
   *
   * @param kP Proportional gain.
   * @param kI Integral gain.
   * @param kD Derivative gain.
   */
  virtual void SetFeedback(double kP, double kI, double kD) = 0;

  /**
   * Set the static friction feedforward (live update).
   *
   * @param kS Static friction voltage.
   */
  virtual void SetKs(double kS) = 0;

  /**
   * Set the velocity feedforward coefficient (live update).
   *
   * @param kV Velocity feedforward coefficient.
   */
  virtual void SetKv(double kV) = 0;

  /**
   * Set the acceleration feedforward coefficient (live update).
   *
   * @param kA Acceleration feedforward coefficient.
   */
  virtual void SetKa(double kA) = 0;

  /**
   * Set the gravity feedforward coefficient (live update).
   *
   * @param kG Gravity feedforward coefficient.
   */
  virtual void SetKg(double kG) = 0;

  /**
   * Set all four feedforward gains at once (live update).
   *
   * @param kS Static friction voltage.
   * @param kV Velocity feedforward coefficient.
   * @param kA Acceleration feedforward coefficient.
   * @param kG Gravity feedforward coefficient.
   */
  virtual void SetFeedforward(double kS, double kV, double kA, double kG) = 0;

  /**
   * Set the stator (output) current limit.
   *
   * @param currentLimit Maximum stator current.
   */
  virtual void SetStatorCurrentLimit(wpi::units::ampere_t currentLimit) = 0;

  /**
   * Set the supply (input) current limit.
   *
   * @param currentLimit Maximum supply current.
   */
  virtual void SetSupplyCurrentLimit(wpi::units::ampere_t currentLimit) = 0;

  /**
   * Set the closed-loop output ramp rate.
   *
   * @param rampRate Time to ramp from 0 to full output.
   */
  virtual void SetClosedLoopRampRate(wpi::units::second_t rampRate) = 0;

  /**
   * Set the open-loop output ramp rate.
   *
   * @param rampRate Time to ramp from 0 to full output.
   */
  virtual void SetOpenLoopRampRate(wpi::units::second_t rampRate) = 0;

  /**
   * Set the upper angular soft limit for the mechanism.
   *
   * @param upperLimit Upper angle limit.
   */
  virtual void SetMechanismUpperLimit(wpi::units::turn_t upperLimit) = 0;

  /**
   * Set the lower angular soft limit for the mechanism.
   *
   * @param lowerLimit Lower angle limit.
   */
  virtual void SetMechanismLowerLimit(wpi::units::turn_t lowerLimit) = 0;

  /**
   * Set both angular soft limits for the mechanism.
   *
   * @param lower Lower angle limit.
   * @param upper Upper angle limit.
   */
  virtual void SetMechanismLimits(wpi::units::turn_t lower, wpi::units::turn_t upper) = 0;

  /**
   * Enable or disable the mechanism angular soft limits.
   *
   * @param enabled true to enforce the limits.
   */
  virtual void SetMechanismLimitsEnabled(bool enabled) = 0;

  /**
   * Set the upper linear soft limit for the measurement.
   *
   * @param upperLimit Upper distance limit.
   */
  virtual void SetMeasurementUpperLimit(wpi::units::meter_t upperLimit) = 0;

  /**
   * Set the lower linear soft limit for the measurement.
   *
   * @param lowerLimit Lower distance limit.
   */
  virtual void SetMeasurementLowerLimit(wpi::units::meter_t lowerLimit) = 0;

  /**
   * Set the maximum angular velocity for the motion profile.
   *
   * @param maxVelocity Maximum mechanism angular velocity.
   */
  virtual void SetMotionProfileMaxVelocity(wpi::units::turns_per_second_t maxVelocity) = 0;

  /**
   * Set the maximum linear velocity for the motion profile.
   *
   * @param maxVelocity Maximum linear velocity.
   */
  virtual void SetMotionProfileMaxVelocity(wpi::units::meters_per_second_t maxVelocity) = 0;

  /**
   * Set the maximum angular acceleration for the motion profile.
   *
   * @param maxAcc Maximum mechanism angular acceleration.
   */
  virtual void SetMotionProfileMaxAcceleration(wpi::units::turns_per_second_squared_t maxAcc) = 0;

  /**
   * Set the maximum linear acceleration for the motion profile.
   *
   * @param maxAcc Maximum linear acceleration.
   */
  virtual void SetMotionProfileMaxAcceleration(wpi::units::meters_per_second_squared_t maxAcc) = 0;

  /**
   * Set the maximum angular jerk for the motion profile.
   *
   * @param maxJerk Maximum angular jerk (degrees/s²/s).
   */
  virtual void SetMotionProfileMaxJerk(wpi::units::angular_jerk::turns_per_second_cubed_t maxJerk) = 0;

  /**
   * Configure or update the exponential motion profile parameters.
   *
   * @param kV       Optional velocity constant (empty keeps the current value).
   * @param kA       Optional acceleration constant (empty keeps the current value).
   * @param maxInput Optional maximum voltage input (empty keeps the current value).
   */
  virtual void SetExponentialProfile(std::optional<double> kV, std::optional<double> kA,
                                     std::optional<wpi::units::volt_t> maxInput) = 0;

  /**
   * Select the active closed-loop gain slot.
   *
   * @param slot Gain slot to activate.
   */
  virtual void SetClosedLoopSlot(ClosedLoopControllerSlot slot) = 0;

  // ---- Closed-loop controller thread --------------------------------------

  /** Start the background closed-loop controller thread. */
  void StartClosedLoopController();

  /** Stop the background closed-loop controller thread. */
  void StopClosedLoopController();

  /** Run one iteration of the closed-loop controller on the calling thread. */
  void IterateClosedLoopController();

  // ---- Telemetry ----------------------------------------------------------

  /**
   * Set up NetworkTables telemetry using explicit table references.
   *
   * @param dataTable   Table for sensor and state data.
   * @param tuningTable Table for live gain tuning entries.
   */
  void SetupTelemetry(std::shared_ptr<wpi::nt::NetworkTable> dataTable,
                      std::shared_ptr<wpi::nt::NetworkTable> tuningTable);

  /** Set up NetworkTables telemetry using the name configured in the SmartMotorControllerConfig. */
  void SetupTelemetry();

  /** Publish the current sensor readings and setpoints to NetworkTables. */
  void UpdateTelemetry();

  /**
   * Override the telemetry configuration used when publishing data to NetworkTables.
   *
   * Call before SetupTelemetry() or UpdateTelemetry() to apply fine-grained control over
   * which fields are published.
   *
   * @param config SmartMotorControllerTelemetryConfig to apply.
   * @return *this for chaining.
   */
  SmartMotorController& WithTelemetry(telemetry::SmartMotorControllerTelemetryConfig config);

  /**
   * Return telemetry fields not supported by this motor controller implementation.
   *
   * Subclasses override this to disable fields for hardware that lacks certain signals
   * (e.g. SPARK controllers do not expose supply current).
   *
   * @return Struct with optional lists of unsupported boolean and double fields.
   */
  virtual telemetry::UnsupportedTelemetryFields GetUnsupportedTelemetryFields();

  /**
   * Get the active closed-loop gain slot.
   *
   * @return Current ClosedLoopControllerSlot.
   */
  ClosedLoopControllerSlot GetClosedLoopControllerSlot() const;

  // ---- Misc ---------------------------------------------------------------

  /**
   * Get the last commanded mechanism position setpoint.
   *
   * @return Optional mechanism angle setpoint.
   */
  std::optional<wpi::units::turn_t> GetMechanismPositionSetpoint() const;

  /**
   * Get the last commanded mechanism velocity setpoint.
   *
   * @return Optional mechanism angular velocity setpoint.
   */
  std::optional<wpi::units::turns_per_second_t> GetMechanismSetpointVelocity() const;

  /**
   * Get the last commanded linear measurement position setpoint.
   * Converts the internal mechanism-angle setpoint using the configured circumference.
   *
   * @return Optional linear position setpoint, or empty if no setpoint has been commanded or
   *         no circumference is configured.
   */
  std::optional<wpi::units::meter_t> GetMeasurementPositionSetpoint() const;

  /**
   * Get the last commanded linear measurement velocity setpoint.
   * Converts the internal mechanism-velocity setpoint using the configured circumference.
   *
   * @return Optional linear velocity setpoint, or empty if no setpoint has been commanded or
   *         no circumference is configured.
   */
  std::optional<wpi::units::meters_per_second_t> GetMeasurementSetpointVelocity() const;

  /**
   * Get a mutable reference to the current configuration.
   *
   * @return Reference to the SmartMotorControllerConfig.
   */
  virtual SmartMotorControllerConfig& GetConfig() = 0;

  /**
   * Get a type-erased pointer to the underlying hardware motor controller object.
   *
   * @return void* that can be cast to the concrete motor controller type.
   */
  virtual void* GetMotorController() = 0;

  /**
   * Get a type-erased pointer to the underlying hardware configuration object.
   *
   * @return void* that can be cast to the concrete motor controller config type.
   */
  virtual void* GetMotorControllerConfig() = 0;

  /**
   * Get the telemetry name of this motor controller.
   *
   * @return Name string from the configuration.
   */
  std::string GetName() const;

  /**
   * Check whether two DCMotor models describe the same motor type.
   *
   * @param a First motor model.
   * @param b Second motor model.
   * @return true if the models have matching electrical constants.
   */
  bool IsMotor(const wpi::math::DCMotor& a, const wpi::math::DCMotor& b) const;

  /** Validate the current configuration and throw if safety constraints are violated. */
  void CheckConfigSafety();

  /** Release any hardware resources held by this controller. */
  void Close();

 protected:
  SmartMotorControllerConfig m_config;
  ClosedLoopControllerSlot m_slot{ClosedLoopControllerSlot::SLOT_0};
  std::shared_ptr<SimSupplier> m_simSupplier;

  /** Loosely coupled followers populated by LoadLooselyCoupledFollowers(). */
  std::vector<SmartMotorController*> m_looseFollowers;

  /**
   * Populate m_looseFollowers from the current m_config.
   * Call this at the end of each concrete ApplyConfig() implementation.
   */
  void LoadLooselyCoupledFollowers();

  /**
   * Forward a position setpoint to all loosely coupled followers.
   * Call at the end of SetPosition(turn_t) in each concrete wrapper.
   */
  void ForwardPositionToFollowers(wpi::units::turn_t pos);

  /**
   * Forward a linear position setpoint to all loosely coupled followers.
   * Call at the end of SetPosition(meter_t) in each concrete wrapper.
   */
  void ForwardPositionToFollowers(wpi::units::meter_t dist);

  /**
   * Forward a velocity setpoint to all loosely coupled followers.
   * Call at the end of SetVelocity(turns_per_second_t) in each concrete wrapper.
   */
  void ForwardVelocityToFollowers(wpi::units::turns_per_second_t vel);

  /**
   * Forward a linear velocity setpoint to all loosely coupled followers.
   * Call at the end of SetVelocity(meters_per_second_t) in each concrete wrapper.
   */
  void ForwardVelocityToFollowers(wpi::units::meters_per_second_t vel);

  std::optional<wpi::math::PIDController> m_pid;
  std::optional<math::LQRController> m_lqr;

  // Angular motion profile state
  std::optional<wpi::math::TrapezoidProfile<wpi::units::turns>::State> m_trapState;
  std::optional<wpi::math::ExponentialProfile<wpi::units::turns, wpi::units::volts>::State> m_expoState;

  // Linear motion profile state
  std::optional<wpi::math::TrapezoidProfile<wpi::units::meters>::State> m_linearTrapState;

  std::optional<wpi::units::turn_t> m_setpointPosition;
  std::optional<wpi::units::turns_per_second_t> m_setpointVelocity;

  std::unique_ptr<wpi::Notifier> m_closedLoopControllerThread;
  bool m_closedLoopControllerRunning{false};

  std::shared_ptr<wpi::nt::NetworkTable> m_parentTable;
  std::shared_ptr<wpi::nt::NetworkTable> m_telemetryTable;
  std::shared_ptr<wpi::nt::NetworkTable> m_tuningTable;

  telemetry::SmartMotorControllerTelemetry m_telemetry;
  telemetry::SmartMotorControllerTelemetryConfig m_telemetryConfig;
  bool m_telemetryConfigExplicit{false};

 private:
  std::optional<wpi::math::TrapezoidProfile<wpi::units::turns>::State> GetTrapezoidalProfileState();
  std::optional<wpi::math::ExponentialProfile<wpi::units::turns, wpi::units::volts>::State>
  GetExponentialProfileState();
};

}  // namespace yams::motorcontrollers
