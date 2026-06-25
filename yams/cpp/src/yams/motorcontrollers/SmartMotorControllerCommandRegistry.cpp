// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/SmartMotorControllerCommandRegistry.hpp"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace yams::motorcontrollers {

std::unordered_map<std::string, frc2::CommandPtr> SmartMotorControllerCommandRegistry::s_commands;
std::unordered_map<std::string, std::vector<std::function<void()>>>
    SmartMotorControllerCommandRegistry::s_callbacks;
std::unordered_map<std::string, frc2::SubsystemBase*> SmartMotorControllerCommandRegistry::s_owners;

std::string SmartMotorControllerCommandRegistry::MakeKey(const std::string& cmdName,
                                                         frc2::SubsystemBase* subsystem) {
  return subsystem->GetName() + "/" + cmdName;
}

void SmartMotorControllerCommandRegistry::PublishToNT(const std::string& cmdName,
                                                      frc2::SubsystemBase* subsystem) {
  auto key = MakeKey(cmdName, subsystem);
  s_commands.insert_or_assign(key, frc2::cmd::Run(
                                       [capturedKey = key] {
                                         for (auto& cb : s_callbacks[capturedKey]) cb();
                                       },
                                       {subsystem})
                                       .WithName(cmdName));
  frc::SmartDashboard::PutData("Mechanisms/Commands/" + key, s_commands.at(key).get());
}

void SmartMotorControllerCommandRegistry::AddCommand(const std::string& cmdName,
                                                     frc2::SubsystemBase* subsystem,
                                                     std::function<void()> callback) {
  auto key = MakeKey(cmdName, subsystem);
  auto ownerIt = s_owners.find(key);
  if (ownerIt != s_owners.end() && ownerIt->second != subsystem) {
    throw std::runtime_error("SmartMotorControllerCommandRegistry: subsystem name conflict — \"" +
                             subsystem->GetName() +
                             "\" is already registered by a different subsystem instance. "
                             "Use unique subsystem names for each subsystem instance (e.g. "
                             "\"LeftTurret\", \"RightTurret\").");
  }
  s_owners[key] = subsystem;
  s_callbacks[key].push_back(std::move(callback));
  if (!CommandExists(cmdName, subsystem)) {
    PublishToNT(cmdName, subsystem);
  }
}

bool SmartMotorControllerCommandRegistry::CommandExists(const std::string& cmdName,
                                                        frc2::SubsystemBase* subsystem) {
  return s_commands.count(MakeKey(cmdName, subsystem)) > 0;
}

void SmartMotorControllerCommandRegistry::RemoveCommands(frc2::SubsystemBase* subsystem) {
  for (auto it = s_owners.begin(); it != s_owners.end();) {
    if (it->second == subsystem) {
      s_commands.erase(it->first);
      s_callbacks.erase(it->first);
      it = s_owners.erase(it);
    } else {
      ++it;
    }
  }
}

void SmartMotorControllerCommandRegistry::Clear() {
  s_commands.clear();
  s_callbacks.clear();
  s_owners.clear();
}

}  // namespace yams::motorcontrollers
