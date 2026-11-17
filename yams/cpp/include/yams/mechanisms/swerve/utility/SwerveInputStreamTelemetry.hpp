// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/commands2/Commands.hpp>
#include <wpi/nt/NetworkTable.hpp>
#include <wpi/nt/NetworkTableInstance.hpp>
#include <wpi/tunables/Tunables.hpp>

#include "yams/mechanisms/swerve/SwerveDriveConfig.hpp"
#include "yams/mechanisms/swerve/utility/SwerveInputStream.hpp"
#include "yams/telemetry/NetworkTablesBackends.hpp"
#include "yams/telemetry/SmartMotorControllerTelemetry.hpp"
#include "yams/telemetry/StringTelemetry.hpp"

namespace yams::mechanisms::swerve::utility {

/**
 * Telemetry and live tuning for a SwerveInputStream, created by
 * SwerveInputStream::WithTelemetry(name, verbosity) and published every time the stream is read.
 *
 * - LOW: the current drive mode, under SwerveInputStream/<name>.
 * - MEDIUM: also the stream's configuration (deadband, scales, maximum velocities, cubing, alliance
 *   and robot relative), read-only, and the rotation controller's measurement and setpoint under
 *   rotationController, in radians in the gyro's frame. Those are what heading, aim and translation
 *   only control give the controller, and are NaN while the stream isn't using it.
 * - HIGH: also editable copies of the configuration under Tuning/SwerveInputStream/<name>, and a
 *   "Live Tuning" command there that applies them to the stream every loop while it runs.
 *
 * Each value is a DoubleTelemetryField, BooleanTelemetryField or StringTelemetryField, which gives
 * its key and whether it is tunable.
 *
 * While live tuning, values stay in sync both ways: a dashboard edit is applied to the stream, and
 * a change made in code, e.g. a binding that changes the translation scale, is published to the
 * dashboard. Invalid dashboard values are replaced with the stream's current value.
 *
 * The stream owns its telemetry and creates it when first read, so the telemetry never outlives the
 * stream or follows it to a new address.
 */
template <size_t NumModules = 4>
class SwerveInputStreamTelemetry {
 public:
  using TelemetryVerbosity = SwerveDriveConfig::TelemetryVerbosity;

  /** Double telemetry field for a SwerveInputStream. */
  enum class DoubleTelemetryField {
    /** Joystick axis deadband. */
    Deadband,
    /** Translation axis scale. */
    TranslationScale,
    /** Rotation axis scale. */
    RotationScale,
    /** Maximum chassis linear velocity, in meters per second. */
    MaxLinearVelocity,
    /** Maximum chassis angular velocity, in radians per second. */
    MaxAngularVelocity,
    /**
     * Heading given to the rotation controller as its measurement, in radians in the gyro's frame.
     * NaN while the stream isn't using the controller.
     */
    RotationControllerMeasurement,
    /**
     * Heading given to the rotation controller as its setpoint, in radians in the gyro's frame. NaN
     * while the stream isn't using the controller.
     */
    RotationControllerSetpoint,
    /** Tunable joystick axis deadband, from 0 up to but not including 1. */
    TunableDeadband,
    /** Tunable translation axis scale, above 0 and up to 1. */
    TunableTranslationScale,
    /** Tunable rotation axis scale, above 0 and up to 1. */
    TunableRotationScale,
    /** Tunable maximum chassis linear velocity, in meters per second. */
    TunableMaxLinearVelocity,
    /** Tunable maximum chassis angular velocity, in radians per second. */
    TunableMaxAngularVelocity,
  };

  /** Boolean telemetry field for a SwerveInputStream. */
  enum class BooleanTelemetryField {
    /** Translation axes are cubed. */
    TranslationCube,
    /** Rotation axis is cubed. */
    RotationCube,
    /** Driving is relative to the alliance. */
    AllianceRelative,
    /** Driving is relative to the robot. */
    RobotRelative,
    /** Tunable translation axis cubing. */
    TunableTranslationCube,
    /** Tunable rotation axis cubing. */
    TunableRotationCube,
    /** Tunable alliance relative driving. */
    TunableAllianceRelative,
    /** Tunable robot relative driving. */
    TunableRobotRelative,
  };

  /** String telemetry field for a SwerveInputStream. */
  enum class StringTelemetryField {
    /** Name of the stream's current drive mode. */
    Mode,
  };

  /**
   * Publish telemetry for a SwerveInputStream. Use SwerveInputStream::WithTelemetry rather than
   * creating this directly.
   *
   * @param stream    The SwerveInputStream to monitor and tune. Must outlive this object.
   * @param name      Name of the stream in NetworkTables (e.g., "drive").
   * @param verbosity TelemetryVerbosity to publish at.
   */
  SwerveInputStreamTelemetry(SwerveInputStream<NumModules>& stream, std::string_view name,
                             TelemetryVerbosity verbosity)
      : m_stream{&stream},
        m_liveTuningPath{"Tuning/SwerveInputStream/" + std::string{name} + "/Live Tuning"} {
    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    AddField(DoubleTelemetryField::Deadband, "deadband", 0.0, false, "none");
    AddField(DoubleTelemetryField::TranslationScale, "translationScale", 1.0, false, "none");
    AddField(DoubleTelemetryField::RotationScale, "rotationScale", 1.0, false, "none");
    AddField(DoubleTelemetryField::MaxLinearVelocity, "maxLinearVelocity", 0.0, false,
             "meters_per_second");
    AddField(DoubleTelemetryField::MaxAngularVelocity, "maxAngularVelocity", 0.0, false,
             "radians_per_second");
    AddField(DoubleTelemetryField::RotationControllerMeasurement, "rotationController/measurement",
             kNaN, false, "radians");
    AddField(DoubleTelemetryField::RotationControllerSetpoint, "rotationController/setpoint", kNaN,
             false, "radians");
    AddField(DoubleTelemetryField::TunableDeadband, "deadband", 0.0, true, "none");
    AddField(DoubleTelemetryField::TunableTranslationScale, "translationScale", 1.0, true, "none");
    AddField(DoubleTelemetryField::TunableRotationScale, "rotationScale", 1.0, true, "none");
    AddField(DoubleTelemetryField::TunableMaxLinearVelocity, "maxLinearVelocity", 0.0, true,
             "meters_per_second");
    AddField(DoubleTelemetryField::TunableMaxAngularVelocity, "maxAngularVelocity", 0.0, true,
             "radians_per_second");
    AddField(BooleanTelemetryField::TranslationCube, "translationCube", false);
    AddField(BooleanTelemetryField::RotationCube, "rotationCube", false);
    AddField(BooleanTelemetryField::AllianceRelative, "allianceRelative", false);
    AddField(BooleanTelemetryField::RobotRelative, "robotRelative", false);
    AddField(BooleanTelemetryField::TunableTranslationCube, "translationCube", true);
    AddField(BooleanTelemetryField::TunableRotationCube, "rotationCube", true);
    AddField(BooleanTelemetryField::TunableAllianceRelative, "allianceRelative", true);
    AddField(BooleanTelemetryField::TunableRobotRelative, "robotRelative", true);
    m_stringFields.emplace(StringTelemetryField::Mode,
                           telemetry::StringTelemetry<StringTelemetryField>{
                               "mode", "", StringTelemetryField::Mode, false});

    switch (verbosity) {
      case TelemetryVerbosity::HIGH:
        m_doubleFields.at(DoubleTelemetryField::TunableDeadband).Enable();
        m_doubleFields.at(DoubleTelemetryField::TunableTranslationScale).Enable();
        m_doubleFields.at(DoubleTelemetryField::TunableRotationScale).Enable();
        m_doubleFields.at(DoubleTelemetryField::TunableMaxLinearVelocity).Enable();
        m_doubleFields.at(DoubleTelemetryField::TunableMaxAngularVelocity).Enable();
        m_boolFields.at(BooleanTelemetryField::TunableTranslationCube).Enable();
        m_boolFields.at(BooleanTelemetryField::TunableRotationCube).Enable();
        m_boolFields.at(BooleanTelemetryField::TunableAllianceRelative).Enable();
        m_boolFields.at(BooleanTelemetryField::TunableRobotRelative).Enable();
        [[fallthrough]];
      case TelemetryVerbosity::MEDIUM:
        m_doubleFields.at(DoubleTelemetryField::Deadband).Enable();
        m_doubleFields.at(DoubleTelemetryField::TranslationScale).Enable();
        m_doubleFields.at(DoubleTelemetryField::RotationScale).Enable();
        m_doubleFields.at(DoubleTelemetryField::MaxLinearVelocity).Enable();
        m_doubleFields.at(DoubleTelemetryField::MaxAngularVelocity).Enable();
        m_doubleFields.at(DoubleTelemetryField::RotationControllerMeasurement).Enable();
        m_doubleFields.at(DoubleTelemetryField::RotationControllerSetpoint).Enable();
        m_boolFields.at(BooleanTelemetryField::TranslationCube).Enable();
        m_boolFields.at(BooleanTelemetryField::RotationCube).Enable();
        m_boolFields.at(BooleanTelemetryField::AllianceRelative).Enable();
        m_boolFields.at(BooleanTelemetryField::RobotRelative).Enable();
        [[fallthrough]];
      case TelemetryVerbosity::LOW:
        m_stringFields.at(StringTelemetryField::Mode).Enable();
        break;
      case TelemetryVerbosity::NONE:
        break;
    }

    auto instance = wpi::nt::NetworkTableInstance::GetDefault();
    auto dataTable = instance.GetTable("SwerveInputStream")->GetSubTable(name);
    auto tuningTable = instance.GetTable("Tuning")->GetSubTable("SwerveInputStream")->GetSubTable(name);
    for (auto& [field, dt] : m_doubleFields) {
      if (dt.IsEnabled()) {
        dt.SetupNetworkTables(dataTable, tuningTable);
        // Skip a value an earlier telemetry left on the dashboard, so the stream's own value is
        // published instead.
        dt.IsTunable();
      }
    }
    for (auto& [field, bt] : m_boolFields) {
      if (bt.IsEnabled()) {
        bt.SetupNetworkTables(dataTable, tuningTable);
        bt.IsTunable();
      }
    }
    for (auto& [field, st] : m_stringFields) {
      if (st.IsEnabled()) {
        st.SetupNetworkTables(dataTable, tuningTable);
      }
    }

    if (verbosity == TelemetryVerbosity::HIGH) {
      // Publish the stream's values to the Tuning table.
      ApplyTuningValues();
      // No requirements, so tuning does not interrupt the command driving with the stream.
      m_liveTuningCommand.emplace(
          wpi::cmd::Run([this] { ApplyTuningValues(); }).WithName("Live Tuning"));
      telemetry::EnsureTuningTunableBackend();
      wpi::tunables::Publish(m_liveTuningPath, *m_liveTuningCommand->get());
    }
    UpdateTelemetry();
  }

  SwerveInputStreamTelemetry(const SwerveInputStreamTelemetry&) = delete;
  SwerveInputStreamTelemetry& operator=(const SwerveInputStreamTelemetry&) = delete;

  /** Stop publishing, canceling and removing the "Live Tuning" command. */
  ~SwerveInputStreamTelemetry() {
    if (m_liveTuningCommand) {
      m_liveTuningCommand->Cancel();
      wpi::tunables::Remove(m_liveTuningPath);
    }
    for (auto& [field, dt] : m_doubleFields) {
      dt.Close();
    }
    for (auto& [field, bt] : m_boolFields) {
      bt.Close();
    }
    for (auto& [field, st] : m_stringFields) {
      st.Close();
    }
  }

  /**
   * Publish the stream's current mode and, at MEDIUM and above, its configuration. Called by
   * SwerveInputStream::Get().
   */
  void UpdateTelemetry() {
    for (auto& [field, st] : m_stringFields) {
      if (!st.IsEnabled()) continue;
      switch (field) {
        case StringTelemetryField::Mode:
          st.Set(m_stream->GetCurrentModeName());
          break;
      }
    }
    for (auto& [field, dt] : m_doubleFields) {
      if (!dt.IsEnabled()) continue;
      switch (field) {
        case DoubleTelemetryField::Deadband:
          dt.Set(m_stream->GetAxisDeadband());
          break;
        case DoubleTelemetryField::TranslationScale:
          dt.Set(m_stream->GetTranslationAxisScale());
          break;
        case DoubleTelemetryField::RotationScale:
          dt.Set(m_stream->GetOmegaAxisScale());
          break;
        case DoubleTelemetryField::MaxLinearVelocity:
          dt.Set(m_stream->GetMaximumChassisLinearVelocity().value());
          break;
        case DoubleTelemetryField::MaxAngularVelocity:
          dt.Set(m_stream->GetMaximumChassisAngularVelocity().value());
          break;
        case DoubleTelemetryField::RotationControllerMeasurement: {
          auto measurement = m_stream->GetRotationControllerMeasurement();
          dt.Set(measurement ? measurement->value() : std::numeric_limits<double>::quiet_NaN());
          break;
        }
        case DoubleTelemetryField::RotationControllerSetpoint: {
          auto setpoint = m_stream->GetRotationControllerSetpoint();
          dt.Set(setpoint ? setpoint->value() : std::numeric_limits<double>::quiet_NaN());
          break;
        }
        default:
          // Tunable fields are published by ApplyTuningValues.
          break;
      }
    }
    for (auto& [field, bt] : m_boolFields) {
      if (!bt.IsEnabled()) continue;
      switch (field) {
        case BooleanTelemetryField::TranslationCube:
          bt.Set(m_stream->IsTranslationCubeEnabled());
          break;
        case BooleanTelemetryField::RotationCube:
          bt.Set(m_stream->IsOmegaCubeEnabled());
          break;
        case BooleanTelemetryField::AllianceRelative:
          bt.Set(m_stream->IsAllianceRelativeEnabled());
          break;
        case BooleanTelemetryField::RobotRelative:
          bt.Set(m_stream->IsRobotRelativeEnabled());
          break;
        default:
          // Tunable fields are published by ApplyTuningValues.
          break;
      }
    }
  }

  /**
   * Apply values edited on the dashboard to the stream, and publish changes made to the stream in
   * code. The "Live Tuning" command calls this every loop. Does nothing below HIGH.
   */
  void ApplyTuningValues() {
    for (auto& [field, dt] : m_doubleFields) {
      if (!dt.IsEnabled()) continue;
      switch (field) {
        case DoubleTelemetryField::TunableDeadband:
          if (dt.IsTunable() && dt.Get() >= 0.0 && dt.Get() < 1.0) {
            m_stream->SetAxisDeadband(dt.Get());
          }
          PublishStreamValue(dt, m_stream->GetAxisDeadband());
          break;
        case DoubleTelemetryField::TunableTranslationScale:
          if (dt.IsTunable() && dt.Get() > 0.0 && dt.Get() <= 1.0) {
            m_stream->SetTranslationAxisScale(dt.Get());
          }
          PublishStreamValue(dt, m_stream->GetTranslationAxisScale());
          break;
        case DoubleTelemetryField::TunableRotationScale:
          if (dt.IsTunable() && dt.Get() > 0.0 && dt.Get() <= 1.0) {
            m_stream->SetOmegaAxisScale(dt.Get());
          }
          PublishStreamValue(dt, m_stream->GetOmegaAxisScale());
          break;
        case DoubleTelemetryField::TunableMaxLinearVelocity:
          if (dt.IsTunable() && dt.Get() > 0.0 && std::isfinite(dt.Get())) {
            m_stream->SetMaximumChassisLinearVelocity(wpi::units::meters_per_second_t{dt.Get()});
          }
          PublishStreamValue(dt, m_stream->GetMaximumChassisLinearVelocity().value());
          break;
        case DoubleTelemetryField::TunableMaxAngularVelocity:
          if (dt.IsTunable() && dt.Get() > 0.0 && std::isfinite(dt.Get())) {
            m_stream->SetMaximumChassisAngularVelocity(wpi::units::radians_per_second_t{dt.Get()});
          }
          PublishStreamValue(dt, m_stream->GetMaximumChassisAngularVelocity().value());
          break;
        default:
          // Read-only fields are published by UpdateTelemetry.
          break;
      }
    }
    for (auto& [field, bt] : m_boolFields) {
      if (!bt.IsEnabled()) continue;
      switch (field) {
        case BooleanTelemetryField::TunableTranslationCube:
          if (bt.IsTunable()) {
            m_stream->SetTranslationCubeEnabled(bt.Get());
          }
          PublishStreamValue(bt, m_stream->IsTranslationCubeEnabled());
          break;
        case BooleanTelemetryField::TunableRotationCube:
          if (bt.IsTunable()) {
            m_stream->SetOmegaCubeEnabled(bt.Get());
          }
          PublishStreamValue(bt, m_stream->IsOmegaCubeEnabled());
          break;
        case BooleanTelemetryField::TunableAllianceRelative:
          if (bt.IsTunable()) {
            m_stream->SetAllianceRelativeEnabled(bt.Get());
          }
          PublishStreamValue(bt, m_stream->IsAllianceRelativeEnabled());
          break;
        case BooleanTelemetryField::TunableRobotRelative:
          if (bt.IsTunable()) {
            m_stream->SetRobotRelativeEnabled(bt.Get());
          }
          PublishStreamValue(bt, m_stream->IsRobotRelativeEnabled());
          break;
        default:
          // Read-only fields are published by UpdateTelemetry.
          break;
      }
    }
  }

  /**
   * The "Live Tuning" command published to the dashboard, which applies dashboard edits while it
   * runs.
   *
   * @return The command at HIGH, otherwise empty.
   */
  std::optional<std::reference_wrapper<wpi::cmd::Command>> GetLiveTuningCommand() {
    if (!m_liveTuningCommand) {
      return std::nullopt;
    }
    return std::ref(*m_liveTuningCommand->get());
  }

 private:
  void AddField(DoubleTelemetryField field, std::string key, double defaultValue, bool tunable,
                std::string unit) {
    m_doubleFields.emplace(field, telemetry::DoubleTelemetry<DoubleTelemetryField>{
                                      std::move(key), defaultValue, field, tunable, std::move(unit)});
  }

  void AddField(BooleanTelemetryField field, std::string key, bool tunable) {
    m_boolFields.emplace(field, telemetry::BooleanTelemetry<BooleanTelemetryField>{
                                    std::move(key), false, field, tunable});
  }

  /**
   * Publish the stream's value of a tunable field if the dashboard shows another, as after a change
   * made in code or an invalid edit. The change is marked as seen, so it isn't applied back to the
   * stream as a dashboard edit.
   */
  template <typename Telemetry, typename T>
  static void PublishStreamValue(Telemetry& telemetry, T streamValue) {
    if (telemetry.Get() != streamValue) {
      telemetry.ForceSet(streamValue);
      telemetry.IsTunable();
    }
  }

  SwerveInputStream<NumModules>* m_stream;
  std::string m_liveTuningPath;
  std::unordered_map<DoubleTelemetryField, telemetry::DoubleTelemetry<DoubleTelemetryField>>
      m_doubleFields;
  std::unordered_map<BooleanTelemetryField, telemetry::BooleanTelemetry<BooleanTelemetryField>>
      m_boolFields;
  std::unordered_map<StringTelemetryField, telemetry::StringTelemetry<StringTelemetryField>>
      m_stringFields;
  /** Command that applies the editable configuration while it runs, at HIGH. */
  std::optional<wpi::cmd::CommandPtr> m_liveTuningCommand;
};

}  // namespace yams::mechanisms::swerve::utility
