// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <wpi/SmallString.h>
#include <wpi/networktables/BooleanTopic.h>
#include <wpi/networktables/DoubleTopic.h>
#include <wpi/networktables/NetworkTable.h>
#include <wpi/networktables/NetworkTableInstance.h>
#include <wpi/networktables/StringTopic.h>

#include "yams/mechanisms/swerve/utility/SwerveInputStream.hpp"

namespace yams::mechanisms::swerve::utility {

template <size_t NumModules = 4>
class SwerveInputStreamTelemetry {
 public:
  SwerveInputStreamTelemetry(SwerveInputStream<NumModules>& stream, std::string_view name)
      : m_stream{&stream},
        m_table{wpi::NetworkTableInstance::GetDefault().GetTable("SwerveInputStream")
                    ->GetSubTable(std::string{name})} {
    m_stateModePublisher = m_table->GetStringTopic("mode").Publish();
    m_stateVxPublisher = m_table->GetDoubleTopic("vx").Publish();
    m_stateVyPublisher = m_table->GetDoubleTopic("vy").Publish();
    m_stateOmegaPublisher = m_table->GetDoubleTopic("omega").Publish();
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

  void Update() {
    auto speeds = m_stream->Get();
    m_stateModePublisher.Set(m_stream->GetCurrentModeName());
    m_stateVxPublisher.Set(speeds.vx.value());
    m_stateVyPublisher.Set(speeds.vy.value());
    m_stateOmegaPublisher.Set(speeds.omega.value());

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

  wpi::StringPublisher m_stateModePublisher;
  wpi::DoublePublisher m_stateVxPublisher;
  wpi::DoublePublisher m_stateVyPublisher;
  wpi::DoublePublisher m_stateOmegaPublisher;

  wpi::DoublePublisher m_deadbandPublisher;
  wpi::DoublePublisher m_translationScalePublisher;
  wpi::DoublePublisher m_rotationScalePublisher;
  wpi::DoublePublisher m_maxLinearVelocityPublisher;
  wpi::DoublePublisher m_maxAngularVelocityPublisher;
  wpi::BooleanPublisher m_translationCubePublisher;
  wpi::BooleanPublisher m_rotationCubePublisher;
  wpi::BooleanPublisher m_allianceRelativePublisher;
  wpi::BooleanPublisher m_robotRelativePublisher;

  void UpdateDeadband() {
    auto entry = m_table->GetDoubleTopic("deadband").GetEntry(m_stream->GetAxisDeadband());
    entry.Set(m_stream->GetAxisDeadband());
    double tuned = entry.Get();
    if (tuned != m_stream->GetAxisDeadband()) {
      m_stream->SetAxisDeadband(tuned);
    }
  }

  void UpdateTranslationScale() {
    auto entry = m_table->GetDoubleTopic("translationScale").GetEntry(m_stream->GetTranslationAxisScale());
    entry.Set(m_stream->GetTranslationAxisScale());
    double tuned = entry.Get();
    if (tuned != m_stream->GetTranslationAxisScale() && tuned > 0.0 && tuned <= 1.0) {
      m_stream->SetTranslationAxisScale(tuned);
    }
  }

  void UpdateRotationScale() {
    auto entry = m_table->GetDoubleTopic("rotationScale").GetEntry(m_stream->GetOmegaAxisScale());
    entry.Set(m_stream->GetOmegaAxisScale());
    double tuned = entry.Get();
    if (tuned != m_stream->GetOmegaAxisScale() && tuned > 0.0 && tuned <= 1.0) {
      m_stream->SetOmegaAxisScale(tuned);
    }
  }

  void UpdateMaxLinearVelocity() {
    auto entry = m_table->GetDoubleTopic("maxLinearVelocity").GetEntry(m_stream->GetMaximumChassisLinearVelocity().value());
    entry.Set(m_stream->GetMaximumChassisLinearVelocity().value());
    double tuned = entry.Get();
    if (tuned != m_stream->GetMaximumChassisLinearVelocity().value() && tuned > 0.0) {
      m_stream->SetMaximumChassisLinearVelocity(wpi::units::meters_per_second_t{tuned});
    }
  }

  void UpdateMaxAngularVelocity() {
    auto entry = m_table->GetDoubleTopic("maxAngularVelocity").GetEntry(m_stream->GetMaximumChassisAngularVelocity().value());
    entry.Set(m_stream->GetMaximumChassisAngularVelocity().value());
    double tuned = entry.Get();
    if (tuned != m_stream->GetMaximumChassisAngularVelocity().value() && tuned > 0.0) {
      m_stream->SetMaximumChassisAngularVelocity(wpi::units::radians_per_second_t{tuned});
    }
  }

  void UpdateTranslationCube() {
    auto entry = m_table->GetBooleanTopic("translationCube").GetEntry(m_stream->IsTranslationCubeEnabled());
    entry.Set(m_stream->IsTranslationCubeEnabled());
    bool tuned = entry.Get();
    if (tuned != m_stream->IsTranslationCubeEnabled()) {
      m_stream->SetTranslationCubeEnabled(tuned);
    }
  }

  void UpdateRotationCube() {
    auto entry = m_table->GetBooleanTopic("rotationCube").GetEntry(m_stream->IsOmegaCubeEnabled());
    entry.Set(m_stream->IsOmegaCubeEnabled());
    bool tuned = entry.Get();
    if (tuned != m_stream->IsOmegaCubeEnabled()) {
      m_stream->SetOmegaCubeEnabled(tuned);
    }
  }

  void UpdateAllianceRelative() {
    auto entry = m_table->GetBooleanTopic("allianceRelative").GetEntry(m_stream->IsAllianceRelativeEnabled());
    entry.Set(m_stream->IsAllianceRelativeEnabled());
    bool tuned = entry.Get();
    if (tuned != m_stream->IsAllianceRelativeEnabled()) {
      m_stream->SetAllianceRelativeEnabled(tuned);
    }
  }

  void UpdateRobotRelative() {
    auto entry = m_table->GetBooleanTopic("robotRelative").GetEntry(m_stream->IsRobotRelativeEnabled());
    entry.Set(m_stream->IsRobotRelativeEnabled());
    bool tuned = entry.Get();
    if (tuned != m_stream->IsRobotRelativeEnabled()) {
      m_stream->SetRobotRelativeEnabled(tuned);
    }
  }
};

}  // namespace yams::mechanisms::swerve::utility
