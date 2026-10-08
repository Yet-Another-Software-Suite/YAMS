// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#include <wpi/nt/BooleanTopic.hpp>
#include <wpi/nt/DoubleTopic.hpp>
#include <wpi/nt/NetworkTable.hpp>
#include <wpi/nt/NetworkTableInstance.hpp>
#include <wpi/nt/StringTopic.hpp>

#include "yams/mechanisms/swerve/utility/SwerveInputStream.hpp"

namespace yams::mechanisms::swerve::utility {

/**
 * Telemetry and live tuning support for SwerveInputStream.
 *
 * Publishes the current mode and configuration of a SwerveInputStream to NetworkTables under
 * SwerveInputStream/<name>, and applies values edited on the dashboard to the stream. Tunable
 * values stay in sync both ways: a dashboard edit is applied to the stream on the next Update(),
 * and a change made in code, e.g. a binding that changes the translation scale, is published to the
 * dashboard. Invalid dashboard values are replaced with the stream's current value.
 *
 * The stream must outlive this object.
 */
template <size_t NumModules = 4>
class SwerveInputStreamTelemetry {
 public:
  /**
   * Create telemetry for a SwerveInputStream under SwerveInputStream/<name>.
   *
   * @param stream The SwerveInputStream to monitor and tune.
   * @param name   The NetworkTables subtable name (e.g., "drive").
   */
  SwerveInputStreamTelemetry(SwerveInputStream<NumModules>& stream, std::string_view name)
      : SwerveInputStreamTelemetry{stream, wpi::nt::NetworkTableInstance::GetDefault()
                                               .GetTable("SwerveInputStream")
                                               ->GetSubTable(name)} {}

  /**
   * Create telemetry for a SwerveInputStream in the given table.
   *
   * @param stream The SwerveInputStream to monitor and tune.
   * @param table  The NetworkTable to publish to.
   */
  SwerveInputStreamTelemetry(SwerveInputStream<NumModules>& stream,
                             std::shared_ptr<wpi::nt::NetworkTable> table)
      : m_stream{&stream}, m_modePublisher{table->GetStringTopic("mode").Publish()} {
    auto* s = m_stream;
    m_modePublisher.Set(s->GetCurrentModeName());
    m_doubles.emplace_back(
        *table, "deadband", [s] { return s->GetAxisDeadband(); },
        [s](double value) { s->SetAxisDeadband(value); },
        [](double value) { return value >= 0.0 && value < 1.0; });
    m_doubles.emplace_back(
        *table, "translationScale", [s] { return s->GetTranslationAxisScale(); },
        [s](double value) { s->SetTranslationAxisScale(value); },
        [](double value) { return value > 0.0 && value <= 1.0; });
    m_doubles.emplace_back(
        *table, "rotationScale", [s] { return s->GetOmegaAxisScale(); },
        [s](double value) { s->SetOmegaAxisScale(value); },
        [](double value) { return value > 0.0 && value <= 1.0; });
    m_doubles.emplace_back(
        *table, "maxLinearVelocity", [s] { return s->GetMaximumChassisLinearVelocity().value(); },
        [s](double value) {
          s->SetMaximumChassisLinearVelocity(wpi::units::meters_per_second_t{value});
        },
        [](double value) { return value > 0.0 && std::isfinite(value); });
    m_doubles.emplace_back(
        *table, "maxAngularVelocity",
        [s] { return s->GetMaximumChassisAngularVelocity().value(); },
        [s](double value) {
          s->SetMaximumChassisAngularVelocity(wpi::units::radians_per_second_t{value});
        },
        [](double value) { return value > 0.0 && std::isfinite(value); });
    m_booleans.emplace_back(
        *table, "translationCube", [s] { return s->IsTranslationCubeEnabled(); },
        [s](bool value) { s->SetTranslationCubeEnabled(value); });
    m_booleans.emplace_back(
        *table, "rotationCube", [s] { return s->IsOmegaCubeEnabled(); },
        [s](bool value) { s->SetOmegaCubeEnabled(value); });
    m_booleans.emplace_back(
        *table, "allianceRelative", [s] { return s->IsAllianceRelativeEnabled(); },
        [s](bool value) { s->SetAllianceRelativeEnabled(value); });
    m_booleans.emplace_back(
        *table, "robotRelative", [s] { return s->IsRobotRelativeEnabled(); },
        [s](bool value) { s->SetRobotRelativeEnabled(value); });
  }

  /**
   * Publish the current state and apply any live-tuned values. Call once per robot loop, before
   * reading the stream.
   */
  void Update() {
    m_modePublisher.Set(m_stream->GetCurrentModeName());
    for (auto& value : m_doubles) {
      value.Update();
    }
    for (auto& value : m_booleans) {
      value.Update();
    }
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
    TunableValue(wpi::nt::NetworkTable& table, std::string_view key, std::function<T()> getter,
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
    static Entry MakeEntry(wpi::nt::NetworkTable& table, std::string_view key, T defaultValue) {
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

  SwerveInputStream<NumModules>* m_stream;
  wpi::nt::StringPublisher m_modePublisher;
  std::vector<TunableValue<double, wpi::nt::DoubleEntry>> m_doubles;
  std::vector<TunableValue<bool, wpi::nt::BooleanEntry>> m_booleans;
};

}  // namespace yams::mechanisms::swerve::utility
