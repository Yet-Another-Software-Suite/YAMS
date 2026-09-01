// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <wpi/datalog/DataLog.hpp>
#include <wpi/nt/NetworkTable.hpp>
#include <wpi/nt/StructTopic.hpp>
#include <wpi/system/DataLogManager.hpp>
#include <wpi/util/struct/Struct.hpp>

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace yams::telemetry {

/**
 * NT4 pub/sub wrapper for a single struct-encoded telemetry field.
 *
 * Mirrors DoubleTelemetry/BooleanTelemetry, but is generic over both the published value type
 * T (must satisfy wpi::StructSerializable) and the enum type F identifying which field this
 * entry represents, so the same wrapper can be reused across unrelated telemetry coordinators
 * (e.g. SwerveDriveTelemetry, SwerveModuleTelemetry) each with their own field enum.
 *
 * @tparam T Struct-serializable value type being published.
 * @tparam F Enum type identifying which field this telemetry entry represents.
 */
template <typename T, typename F>
class StructTelemetry {
 public:
  /**
   * Construct a StructTelemetry entry.
   *
   * @param key        NT4 table key for this field.
   * @param defaultVal Default value published on startup.
   * @param field      Field identifier.
   * @param tunable    true if the field should be placed in the tuning table with a subscriber.
   */
  StructTelemetry(std::string key, T defaultVal, F field, bool tunable)
      : m_field{field},
        m_key{std::move(key)},
        m_tunable{tunable},
        m_defaultValue{std::move(defaultVal)},
        m_cachedValue{m_defaultValue} {}

  /**
   * Override the default published value.
   *
   * @param value New default value.
   */
  void SetDefaultValue(T value) { m_cachedValue = m_defaultValue = std::move(value); }

  /**
   * Create the NT4 publisher (and subscriber if tunable) under the given tables.
   *
   * @param dataTable   NT4 table for read-only sensor data.
   * @param tuningTable NT4 table for live-tunable fields.
   */
  void SetupNetworkTables(std::shared_ptr<wpi::nt::NetworkTable> dataTable,
                          std::shared_ptr<wpi::nt::NetworkTable> tuningTable) {
    m_dataTable = dataTable;
    m_tuningTable = tuningTable;
    if (!m_enabled) return;

    if (tuningTable && m_tunable) {
      auto topic = tuningTable->template GetStructTopic<T>(m_key);
      m_subPublisher = topic.Publish();
      m_subPublisher->SetDefault(m_defaultValue);
      m_subscriber = topic.Subscribe(m_defaultValue);
    } else if (dataTable) {
      auto topic = dataTable->template GetStructTopic<T>(m_key);
      m_publisher = topic.Publish();
      m_publisher->SetDefault(m_defaultValue);
    }
  }

  /**
   * Create a read-only NT4 publisher under the data table (no tuning table).
   *
   * @param dataTable NT4 table for sensor data.
   */
  void SetupNetworkTable(std::shared_ptr<wpi::nt::NetworkTable> dataTable) {
    SetupNetworkTables(dataTable, nullptr);
  }

  /**
   * Create a DataLog entry under the given prefix path.
   *
   * @param prefix DataLog path prefix (a trailing "/" is added if missing).
   */
  void SetupDataLog(const std::string& prefix) {
    if (m_tunable) return;
    std::string path = prefix;
    if (!path.empty() && path.back() != '/') path += '/';
    path += m_key;
    m_dataLogEntry = wpi::log::StructLogEntry<T>{wpi::DataLogManager::GetLog(), path};
  }

  /**
   * Publish a value; returns false if the field is disabled or a tunable subscriber disagrees.
   *
   * @param value Value to publish.
   * @return true if the value was accepted and published.
   */
  bool Set(const T& value) {
    if (!m_enabled) return false;
    if (m_dataLogEntry) {
      m_dataLogEntry->Append(value);
    }
    if (m_subscriber) {
      T tuningValue = m_subscriber->Get(m_defaultValue);
      if (!(tuningValue == value)) return false;
    }
    if (m_publisher) {
      m_publisher->Set(value);
    }
    return true;
  }

  /**
   * Read the current value from the subscriber (tunable) or return the default.
   *
   * @return Current field value.
   */
  T Get() const {
    if (!m_enabled) return m_defaultValue;
    if (m_subscriber) {
      return m_subscriber->Get(m_defaultValue);
    }
    throw std::runtime_error("Tuning table not configured for " + m_key + "!");
  }

  /**
   * Return true if a tunable subscriber is active and the value has changed since the last call.
   *
   * @return true when the NT4 subscriber holds a value different from the cached value.
   */
  bool IsTunable() {
    if (m_subscriber && m_tunable && m_enabled) {
      T current = m_subscriber->Get(m_defaultValue);
      if (!(current == m_cachedValue)) {
        m_cachedValue = current;
        return true;
      }
    }
    return false;
  }

  /** Enable this field so it is published / read each loop. */
  void Enable() { m_enabled = true; }
  /** Disable this field so publishing and reading are skipped. */
  void Disable() { m_enabled = false; }
  /**
   * Enable or disable this field.
   *
   * @param state true to enable.
   */
  void Display(bool state) { m_enabled = state; }
  /** @return The field identifier for this entry. */
  F GetField() const { return m_field; }
  /** @return true if this field is currently enabled. */
  bool IsEnabled() const { return m_enabled; }

  /** Release all NT4 publishers/subscribers and DataLog entries. */
  void Close() {
    m_subscriber.reset();
    m_subPublisher.reset();
    m_publisher.reset();
    if (m_dataTable) {
      m_dataTable->GetEntry(m_key).Unpublish();
    }
    if (m_tuningTable) {
      m_tuningTable->GetEntry(m_key).Unpublish();
    }
  }

 private:
  F m_field;
  std::string m_key;
  bool m_tunable;
  bool m_enabled{false};
  T m_defaultValue;
  T m_cachedValue;

  std::optional<wpi::nt::StructPublisher<T>> m_publisher;
  std::optional<wpi::nt::StructSubscriber<T>> m_subscriber;
  std::optional<wpi::nt::StructPublisher<T>> m_subPublisher;  // tunable: publish + subscribe
  std::optional<wpi::log::StructLogEntry<T>> m_dataLogEntry;

  std::shared_ptr<wpi::nt::NetworkTable> m_tuningTable;
  std::shared_ptr<wpi::nt::NetworkTable> m_dataTable;
};

}  // namespace yams::telemetry
