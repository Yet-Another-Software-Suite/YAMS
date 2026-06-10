// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/SmartMotorControllerCommandRegistry.hpp"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace yams::motorcontrollers {

std::unordered_map<std::string, frc2::CommandPtr> SmartMotorControllerCommandRegistry::s_commands;
std::unordered_map<std::string, std::vector<std::function<void()>>>
    SmartMotorControllerCommandRegistry::s_callbacks;

std::string SmartMotorControllerCommandRegistry::MakeKey(const std::string& cmdName,
                                                         frc2::SubsystemBase* subsystem) {
  return subsystem->GetName() + "/" + cmdName;
}

void SmartMotorControllerCommandRegistry::PublishToNT(const std::string& cmdName,
                                                      frc2::SubsystemBase* subsystem) {
  auto key = MakeKey(cmdName, subsystem);
  auto& callbacks = s_callbacks[key];
  s_commands.insert_or_assign(key, frc2::cmd::Run(
                                       [&callbacks] {
                                         for (auto& cb : callbacks) cb();
                                       },
                                       {subsystem})
                                       .WithName(cmdName));
  frc::SmartDashboard::PutData("Mechanisms/Commands/" + key, s_commands.at(key).get());
}

void SmartMotorControllerCommandRegistry::AddCommand(const std::string& cmdName,
                                                     frc2::SubsystemBase* subsystem,
                                                     std::function<void()> callback) {
  auto key = MakeKey(cmdName, subsystem);
  s_callbacks[key].push_back(std::move(callback));
  if (!CommandExists(cmdName, subsystem)) {
    PublishToNT(cmdName, subsystem);
  }
}

bool SmartMotorControllerCommandRegistry::CommandExists(const std::string& cmdName,
                                                        frc2::SubsystemBase* subsystem) {
  return s_commands.count(MakeKey(cmdName, subsystem)) > 0;
}

}  // namespace yams::motorcontrollers
