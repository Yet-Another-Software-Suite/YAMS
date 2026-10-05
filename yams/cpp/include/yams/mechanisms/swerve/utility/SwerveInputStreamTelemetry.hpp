// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <memory>
#include <string>
#include <wpi/networktables/BooleanTopic.h>
#include <wpi/networktables/DoubleTopic.h>
#include <wpi/networktables/NetworkTable.h>
#include <wpi/networktables/NetworkTableInstance.h>
#include <wpi/networktables/StringTopic.h>

#include "yams/mechanisms/swerve/utility/SwerveInputStream.hpp"

namespace yams::mechanisms::swerve::utility {

/**
 * Telemetry and live tuning support for SwerveInputStream.
 *
 * Publishes the current state and configuration of a SwerveInputStream to NetworkTables for
 * monitoring on the dashboard. Also enables real-time tuning of deadband, scale, and maximum
 * velocity parameters via NetworkTables entries.
 *
 * Usage:
 * ```cpp
 * auto driveStream = SwerveInputStream<4>::Of(drive,
 *     [this]{ return -m_driverController.GetLeftY(); },
 *     [this]{ return -m_driverController.GetLeftX(); })
 *   .WithControllerRotationAxis([this]{ return -m_driverController.GetRightX(); })
 *   .WithDeadband(0.05);
 *
 * auto telemetry = std::make_unique<SwerveInputStreamTelemetry<4>>(driveStream, "drive");
 * // In periodic:
 * telemetry->Update();
 * ```
 *
 * @tparam NumModules Number of swerve modules (must match the SwerveInputStream).
 */
template <size_t NumModules = 4>
class SwerveInputStreamTelemetry {
 public:
  /**
   * Create telemetry for a SwerveInputStream.
   *
   * @param stream The SwerveInputStream to monitor and tune.
   * @param name   The NetworkTables subtable name (e.g., "drive").
   */
  SwerveInputStreamTelemetry(SwerveInputStream<NumModules>& stream, std::string_view name)
      : m_stream{&stream},
        m_table{wpi::NetworkTableInstance::GetDefault()
                    .GetTable("SwerveInputStream")
                    ->GetSubTable(name)} {
    // State publishers
    m_stateModePublisher = m_table->GetStringTopic("mode").Publish();
    m_stateVxPublisher = m_table->GetDoubleTopic("vx").Publish();
    m_stateVyPublisher = m_table->GetDoubleTopic("vy").Publish();
    m_stateOmegaPublisher = m_table->GetDoubleTopic("omega").Publish();

    // Live tuning publishers
    m_deadbandPublisher = m_table->GetDoubleTopic("deadband").Publish();
    m_translationScalePublisher = m_table->GetDoubleTopic("translationScale").Publish();
    m_rotationScalePublisher = m_table->GetDoubleTopic("rotationScale").Publish();
    m_maxLinearVelocityPublisher = m_table->GetDoubleTopic("maxLinearVelocity").Publish();
    m_maxAngularVelocityPublisher = m_table->GetDoubleTopic("maxAngularVelocity").Publish();
    m_translationCubePublisher = m_table->GetBooleanTopic("translationCube").Publish();
    m_rotationCubePublisher = m_table->GetBooleanTopic("rotationCube").Publish();
    m_allianceRelativePublisher = m_table->GetBooleanTopic("allianceRelative").Publish();
    m_robotRelativePublisher = m_table->GetBooleanTopic("robotRelative").Publish();
  }

  /**
   * Update telemetry and apply any live-tuned values.
   *
   * Call this method once per robot loop (e.g., in periodic() or in your command's execute())
   * to publish the current state and check for tuning changes.
   */
  void Update() {
    // Get current output
    auto currentSpeeds = m_stream->Get();
    m_stateModePublisher.Set(ModeToString());
    m_stateVxPublisher.Set(currentSpeeds.vx.value());
    m_stateVyPublisher.Set(currentSpeeds.vy.value());
    m_stateOmegaPublisher.Set(currentSpeeds.omega.value());

    // Publish and poll live-tuned values
    UpdateDeadband();
    UpdateTranslationScale();
    UpdateRotationScale();
    UpdateMaxLinearVelocity();
    UpdateMaxAngularVelocity();
    UpdateTranslationCube();
    UpdateRotationCube();
    UpdateAllianceRelative();
    UpdateRobotRelative();
  }

 private:
  SwerveInputStream<NumModules>* m_stream;
  std::shared_ptr<wpi::NetworkTable> m_table;

  // State publishers
  wpi::StringPublisher m_stateModePublisher;
  wpi::DoublePublisher m_stateVxPublisher;
  wpi::DoublePublisher m_stateVyPublisher;
  wpi::DoublePublisher m_stateOmegaPublisher;

  // Live tuning publishers
  wpi::DoublePublisher m_deadbandPublisher;
  wpi::DoublePublisher m_translationScalePublisher;
  wpi::DoublePublisher m_rotationScalePublisher;
  wpi::DoublePublisher m_maxLinearVelocityPublisher;
  wpi::DoublePublisher m_maxAngularVelocityPublisher;
  wpi::BooleanPublisher m_translationCubePublisher;
  wpi::BooleanPublisher m_rotationCubePublisher;
  wpi::BooleanPublisher m_allianceRelativePublisher;
  wpi::BooleanPublisher m_robotRelativePublisher;

  // Cached values to detect changes
  double m_lastDeadband = 0.0;
  double m_lastTranslationScale = 1.0;
  double m_lastRotationScale = 1.0;
  double m_lastMaxLinearVelocity = 4.0;
  double m_lastMaxAngularVelocity = 2.0 * std::numbers::pi;

  std::string ModeToString() const {
    // Access via member introspection would require public getters.
    // For now, return a placeholder.
    return "UNKNOWN";
  }

  void UpdateDeadband() {
    double current = m_stream->m_axisDeadband.has_value() ? m_stream->m_axisDeadband.value() : 0.0;
    if (current != m_lastDeadband) {
      m_deadbandPublisher.Set(current);
      m_lastDeadband = current;
    }
    double newValue = m_deadbandPublisher.Get(current);
    if (newValue != current) {
      m_stream->m_axisDeadband =
          (newValue == 0.0) ? std::nullopt : std::optional<double>{newValue};
      m_lastDeadband = newValue;
    }
  }

  void UpdateTranslationScale() {
    double current =
        m_stream->m_translationAxisScale.has_value() ? m_stream->m_translationAxisScale.value() : 1.0;
    if (current != m_lastTranslationScale) {
      m_translationScalePublisher.Set(current);
      m_lastTranslationScale = current;
    }
    double newValue = m_translationScalePublisher.Get(current);
    if (newValue != current && newValue > 0 && newValue <= 1.0) {
      m_stream->m_translationAxisScale = newValue;
      m_lastTranslationScale = newValue;
    }
  }

  void UpdateRotationScale() {
    double current =
        m_stream->m_omegaAxisScale.has_value() ? m_stream->m_omegaAxisScale.value() : 1.0;
    if (current != m_lastRotationScale) {
      m_rotationScalePublisher.Set(current);
      m_lastRotationScale = current;
    }
    double newValue = m_rotationScalePublisher.Get(current);
    if (newValue != current && newValue > 0 && newValue <= 1.0) {
      m_stream->m_omegaAxisScale = newValue;
      m_lastRotationScale = newValue;
    }
  }

  void UpdateMaxLinearVelocity() {
    double current = m_stream->m_maximumChassisLinearVelocity.value();
    if (current != m_lastMaxLinearVelocity) {
      m_maxLinearVelocityPublisher.Set(current);
      m_lastMaxLinearVelocity = current;
    }
    double newValue = m_maxLinearVelocityPublisher.Get(current);
    if (newValue != current && newValue > 0) {
      m_stream->m_maximumChassisLinearVelocity =
          wpi::units::meters_per_second_t{newValue};
      m_lastMaxLinearVelocity = newValue;
    }
  }

  void UpdateMaxAngularVelocity() {
    double current = m_stream->m_maximumChassisAngularVelocity.value();
    if (current != m_lastMaxAngularVelocity) {
      m_maxAngularVelocityPublisher.Set(current);
      m_lastMaxAngularVelocity = current;
    }
    double newValue = m_maxAngularVelocityPublisher.Get(current);
    if (newValue != current && newValue > 0) {
      m_stream->m_maximumChassisAngularVelocity =
          wpi::units::radians_per_second_t{newValue};
      m_lastMaxAngularVelocity = newValue;
    }
  }

  void UpdateTranslationCube() {
    bool current =
        m_stream->m_translationCube.has_value() && m_stream->m_translationCube.value()();
    bool newValue = m_translationCubePublisher.Get(current);
    if (newValue != current) {
      m_stream->m_translationCube =
          newValue ? std::optional<std::function<bool()>>{[] { return true; }} : std::nullopt;
    }
  }

  void UpdateRotationCube() {
    bool current =
        m_stream->m_omegaCube.has_value() && m_stream->m_omegaCube.value()();
    bool newValue = m_rotationCubePublisher.Get(current);
    if (newValue != current) {
      m_stream->m_omegaCube =
          newValue ? std::optional<std::function<bool()>>{[] { return true; }} : std::nullopt;
    }
  }

  void UpdateAllianceRelative() {
    bool current =
        m_stream->m_allianceRelative.has_value() && m_stream->m_allianceRelative.value()();
    bool newValue = m_allianceRelativePublisher.Get(current);
    if (newValue != current) {
      m_stream->m_allianceRelative =
          newValue ? std::optional<std::function<bool()>>{[] { return true; }} : std::nullopt;
    }
  }

  void UpdateRobotRelative() {
    bool current =
        m_stream->m_robotRelative.has_value() && m_stream->m_robotRelative.value()();
    bool newValue = m_robotRelativePublisher.Get(current);
    if (newValue != current) {
      m_stream->m_robotRelative =
          newValue ? std::optional<std::function<bool()>>{[] { return true; }} : std::nullopt;
    }
  }
};

}  // namespace yams::mechanisms::swerve::utility
