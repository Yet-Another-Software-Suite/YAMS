// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <wpi/datalog/DataLog.hpp>
#include <wpi/nt/NetworkTable.hpp>
#include <wpi/nt/StringTopic.hpp>
#include <wpi/system/DataLogManager.hpp>

namespace yams::telemetry {

/**
 * NT4 pub/sub wrapper for a single string-valued telemetry field, such as the name of the mode a
 * mechanism is in.
 *
 * @tparam F Enum type identifying which field this telemetry entry represents.
 */
template <typename F>
class StringTelemetry {
 public:
  /**
   * Construct a StringTelemetry entry.
   *
   * @param key        NT4 table key for this field.
   * @param defaultVal Default value published on startup.
   * @param field      Field identifier.
   * @param tunable    true if the field should be placed in the tuning table with a subscriber.
   */
  StringTelemetry(std::string key, std::string defaultVal, F field, bool tunable)
      : m_field{field},
        m_key{std::move(key)},
        m_tunable{tunable},
        m_defaultValue{defaultVal},
        m_cachedValue{std::move(defaultVal)} {}

  /**
   * Override the default published value.
   *
   * @param value New default value.
   */
  void SetDefaultValue(std::string value) {
    m_defaultValue = value;
    m_cachedValue = std::move(value);
  }

  /**
   * Create the NT4 publisher (and subscriber if tunable) under the given tables.
   *
   * @param dataTable   NT4 table for read-only status data.
   * @param tuningTable NT4 table for live-tunable fields.
   */
  void SetupNetworkTables(std::shared_ptr<wpi::nt::NetworkTable> dataTable,
                          std::shared_ptr<wpi::nt::NetworkTable> tuningTable) {
    m_dataTable = dataTable;
    m_tuningTable = tuningTable;
    if (!m_enabled) return;

    if (tuningTable && m_tunable) {
      auto topic = tuningTable->GetStringTopic(m_key);
      m_pubSub = topic.Publish();
      m_pubSub->SetDefault(m_defaultValue);
      m_subscriber = topic.Subscribe(m_defaultValue);
    } else if (dataTable) {
      auto topic = dataTable->GetStringTopic(m_key);
      m_publisher = topic.Publish();
      m_publisher->SetDefault(m_defaultValue);
    }
  }

  /**
   * Create a read-only NT4 publisher under the data table (no tuning table).
   *
   * @param dataTable NT4 table for status data.
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
    m_dataLogEntry = wpi::log::StringLogEntry{wpi::DataLogManager::GetLog(), path};
  }

  /**
   * Publish a value; returns false if the field is disabled or a tunable subscriber disagrees.
   *
   * @param value Value to publish.
   * @return true if the value was accepted and published.
   */
  bool Set(std::string_view value) {
    if (!m_enabled) return false;
    if (m_dataLogEntry) {
      m_dataLogEntry->Append(value);
    }
    if (m_subscriber) {
      if (m_subscriber->Get(m_defaultValue) != value) return false;
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
  std::string Get() const {
    if (!m_enabled) return m_defaultValue;
    if (m_subscriber) {
      return m_subscriber->Get(m_defaultValue);
    }
    throw std::runtime_error("Tuning table not configured for " + m_key + "!");
  }

  /**
   * Forcibly overwrite the tunable value in NetworkTables, bypassing the subscriber-must-match
   * guard in Set(). The next IsTunable() reports the change, as if it was made on the dashboard.
   *
   * @param value Value to force.
   */
  void ForceSet(std::string_view value) {
    if (m_pubSub) {
      m_pubSub->Set(value);
    } else if (m_publisher) {
      m_publisher->Set(value);
    }
  }

  /**
   * Return true if a tunable subscriber is active and the value has changed since the last call.
   *
   * @return true when the NT4 subscriber holds a value different from the cached value.
   */
  bool IsTunable() {
    if (m_subscriber && m_tunable && m_enabled) {
      std::string current = m_subscriber->Get(m_defaultValue);
      if (current != m_cachedValue) {
        m_cachedValue = std::move(current);
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
    m_pubSub.reset();
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
  std::string m_defaultValue;
  std::string m_cachedValue;

  std::optional<wpi::nt::StringPublisher> m_publisher;
  std::optional<wpi::nt::StringSubscriber> m_subscriber;
  std::optional<wpi::nt::StringPublisher> m_pubSub;  // tunable: publish + subscribe
  std::optional<wpi::log::StringLogEntry> m_dataLogEntry;

  std::shared_ptr<wpi::nt::NetworkTable> m_tuningTable;
  std::shared_ptr<wpi::nt::NetworkTable> m_dataTable;
};

}  // namespace yams::telemetry
