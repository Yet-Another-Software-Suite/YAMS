// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include "yams/mechanisms/swerve/SwerveDriveConfig.hpp"
#include "yams/mechanisms/swerve/utility/SwerveInputStream.hpp"

namespace yams::mechanisms::swerve::utility {

/**
 * Telemetry and live tuning for a SwerveInputStream, created by
 * SwerveInputStream::WithTelemetry(name, verbosity) and published every time the stream is read.
 *
 * - LOW: the current drive mode, under SwerveInputStream/<name>.
 * - MEDIUM: also the stream's configuration (deadband, scales, maximum velocities, cubing, alliance
 *   and robot relative), read-only.
 * - HIGH: also editable copies of the configuration under Tuning/SwerveInputStream/<name>, and a
 *   "Live Tuning" command under SmartDashboard/Mechanisms/Commands/SwerveInputStream/<name> that
 *   applies them to the stream every loop while it runs.
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
        m_liveTuningPath{"Mechanisms/Commands/SwerveInputStream/" + std::string{name} +
                         "/Live Tuning"} {
    auto instance = nt::NetworkTableInstance::GetDefault();
    auto dataTable = instance.GetTable("SwerveInputStream")->GetSubTable(name);
    m_modePublisher = dataTable->GetStringTopic("mode").Publish();

    if (verbosity == TelemetryVerbosity::MEDIUM || verbosity == TelemetryVerbosity::HIGH) {
      PublishConfig(*dataTable);
    }
    if (verbosity == TelemetryVerbosity::HIGH) {
      AddTunableValues(
          *instance.GetTable("Tuning")->GetSubTable("SwerveInputStream")->GetSubTable(name));
      // No requirements, so tuning does not interrupt the command driving with the stream.
      m_liveTuningCommand.emplace(
          frc2::cmd::Run([this] { ApplyTuningValues(); }).WithName("Live Tuning"));
      frc::SmartDashboard::PutData(m_liveTuningPath, m_liveTuningCommand->get());
    }
    UpdateTelemetry();
  }

  SwerveInputStreamTelemetry(const SwerveInputStreamTelemetry&) = delete;
  SwerveInputStreamTelemetry& operator=(const SwerveInputStreamTelemetry&) = delete;

  /** Stop publishing and cancel the "Live Tuning" command. */
  ~SwerveInputStreamTelemetry() {
    if (m_liveTuningCommand) {
      m_liveTuningCommand->Cancel();
    }
  }

  /**
   * Publish the stream's current mode and, at MEDIUM and above, its configuration. Called by
   * SwerveInputStream::Get().
   */
  void UpdateTelemetry() {
    m_modePublisher.Set(m_stream->GetCurrentModeName());
    for (auto& publish : m_configPublishers) {
      publish();
    }
  }

  /**
   * Apply values edited on the dashboard to the stream, and publish changes made to the stream in
   * code. The "Live Tuning" command calls this every loop. Does nothing below HIGH.
   */
  void ApplyTuningValues() {
    for (auto& value : m_doubles) {
      value.Update();
    }
    for (auto& value : m_booleans) {
      value.Update();
    }
  }

  /**
   * The "Live Tuning" command published to the dashboard, which applies dashboard edits while it
   * runs.
   *
   * @return The command at HIGH, otherwise empty.
   */
  std::optional<std::reference_wrapper<frc2::Command>> GetLiveTuningCommand() {
    if (!m_liveTuningCommand) {
      return std::nullopt;
    }
    return std::ref(*m_liveTuningCommand->get());
  }

 private:
  /** A stream value of type T kept in sync with a NetworkTables entry of type Entry. */
  template <typename T, typename Entry>
  class TunableValue {
   public:
    /**
     * Publish a tunable stream value.
     *
     * @param table  Table to publish to.
     * @param key    Entry key.
     * @param getter Reads the value from the stream.
     * @param setter Applies a value to the stream.
     * @param valid  Whether a dashboard value can be applied.
     */
    TunableValue(nt::NetworkTable& table, std::string_view key, std::function<T()> getter,
                 std::function<void(T)> setter,
                 std::function<bool(T)> valid = [](T) { return true; })
        : m_getter{std::move(getter)},
          m_setter{std::move(setter)},
          m_valid{std::move(valid)},
          m_lastValue{m_getter()},
          m_entry{MakeEntry(table, key, m_lastValue)} {
      m_entry.Set(m_lastValue);
    }

    /** Apply a dashboard edit to the stream, or publish a change made in code. */
    void Update() {
      T published = m_entry.Get();
      if (published != m_lastValue && m_valid(published)) {
        m_setter(published);
      }
      T current = m_getter();
      if (current != published) {
        m_entry.Set(current);
      }
      m_lastValue = current;
    }

   private:
    static Entry MakeEntry(nt::NetworkTable& table, std::string_view key, T defaultValue) {
      if constexpr (std::is_same_v<T, bool>) {
        return table.GetBooleanTopic(key).GetEntry(defaultValue);
      } else {
        return table.GetDoubleTopic(key).GetEntry(defaultValue);
      }
    }

    std::function<T()> m_getter;
    std::function<void(T)> m_setter;
    std::function<bool(T)> m_valid;
    /** Value last published or applied, to tell dashboard edits apart from changes made in code. */
    T m_lastValue;
    Entry m_entry;
  };

  void PublishConfig(nt::NetworkTable& table) {
    PublishDouble(table, "deadband", [this] { return m_stream->GetAxisDeadband(); });
    PublishDouble(table, "translationScale", [this] { return m_stream->GetTranslationAxisScale(); });
    PublishDouble(table, "rotationScale", [this] { return m_stream->GetOmegaAxisScale(); });
    PublishDouble(table, "maxLinearVelocity",
                  [this] { return m_stream->GetMaximumChassisLinearVelocity().value(); });
    PublishDouble(table, "maxAngularVelocity",
                  [this] { return m_stream->GetMaximumChassisAngularVelocity().value(); });
    PublishBoolean(table, "translationCube", [this] { return m_stream->IsTranslationCubeEnabled(); });
    PublishBoolean(table, "rotationCube", [this] { return m_stream->IsOmegaCubeEnabled(); });
    PublishBoolean(table, "allianceRelative",
                   [this] { return m_stream->IsAllianceRelativeEnabled(); });
    PublishBoolean(table, "robotRelative", [this] { return m_stream->IsRobotRelativeEnabled(); });
  }

  void PublishDouble(nt::NetworkTable& table, std::string_view key,
                     std::function<double()> value) {
    auto publisher =
        std::make_shared<nt::DoublePublisher>(table.GetDoubleTopic(key).Publish());
    m_configPublishers.emplace_back(
        [publisher, value = std::move(value)] { publisher->Set(value()); });
  }

  void PublishBoolean(nt::NetworkTable& table, std::string_view key,
                      std::function<bool()> value) {
    auto publisher =
        std::make_shared<nt::BooleanPublisher>(table.GetBooleanTopic(key).Publish());
    m_configPublishers.emplace_back(
        [publisher, value = std::move(value)] { publisher->Set(value()); });
  }

  void AddTunableValues(nt::NetworkTable& table) {
    m_doubles.emplace_back(
        table, "deadband", [this] { return m_stream->GetAxisDeadband(); },
        [this](double value) { m_stream->SetAxisDeadband(value); },
        [](double value) { return value >= 0.0 && value < 1.0; });
    m_doubles.emplace_back(
        table, "translationScale", [this] { return m_stream->GetTranslationAxisScale(); },
        [this](double value) { m_stream->SetTranslationAxisScale(value); },
        [](double value) { return value > 0.0 && value <= 1.0; });
    m_doubles.emplace_back(
        table, "rotationScale", [this] { return m_stream->GetOmegaAxisScale(); },
        [this](double value) { m_stream->SetOmegaAxisScale(value); },
        [](double value) { return value > 0.0 && value <= 1.0; });
    m_doubles.emplace_back(
        table, "maxLinearVelocity",
        [this] { return m_stream->GetMaximumChassisLinearVelocity().value(); },
        [this](double value) {
          m_stream->SetMaximumChassisLinearVelocity(units::meters_per_second_t{value});
        },
        [](double value) { return value > 0.0 && std::isfinite(value); });
    m_doubles.emplace_back(
        table, "maxAngularVelocity",
        [this] { return m_stream->GetMaximumChassisAngularVelocity().value(); },
        [this](double value) {
          m_stream->SetMaximumChassisAngularVelocity(units::radians_per_second_t{value});
        },
        [](double value) { return value > 0.0 && std::isfinite(value); });
    m_booleans.emplace_back(
        table, "translationCube", [this] { return m_stream->IsTranslationCubeEnabled(); },
        [this](bool value) { m_stream->SetTranslationCubeEnabled(value); });
    m_booleans.emplace_back(
        table, "rotationCube", [this] { return m_stream->IsOmegaCubeEnabled(); },
        [this](bool value) { m_stream->SetOmegaCubeEnabled(value); });
    m_booleans.emplace_back(
        table, "allianceRelative", [this] { return m_stream->IsAllianceRelativeEnabled(); },
        [this](bool value) { m_stream->SetAllianceRelativeEnabled(value); });
    m_booleans.emplace_back(
        table, "robotRelative", [this] { return m_stream->IsRobotRelativeEnabled(); },
        [this](bool value) { m_stream->SetRobotRelativeEnabled(value); });
  }

  SwerveInputStream<NumModules>* m_stream;
  std::string m_liveTuningPath;
  nt::StringPublisher m_modePublisher;
  /** Read-only configuration, published at MEDIUM and above. */
  std::vector<std::function<void()>> m_configPublishers;
  /** Editable configuration, at HIGH. */
  std::vector<TunableValue<double, nt::DoubleEntry>> m_doubles;
  std::vector<TunableValue<bool, nt::BooleanEntry>> m_booleans;
  /** Command that applies the editable configuration while it runs, at HIGH. */
  std::optional<frc2::CommandPtr> m_liveTuningCommand;
};

}  // namespace yams::mechanisms::swerve::utility
