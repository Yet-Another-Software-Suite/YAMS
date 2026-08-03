// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/SmartMotorControllerCommandRegistry.hpp"

#include <wpi/smartdashboard/SmartDashboard.hpp>
#include <wpi/commands2/Commands.hpp>

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace yams::motorcontrollers {

std::unordered_map<std::string, wpi::cmd::CommandPtr> SmartMotorControllerCommandRegistry::s_commands;
std::unordered_map<std::string, std::vector<std::function<void()>>>
    SmartMotorControllerCommandRegistry::s_callbacks;
std::unordered_map<std::string, wpi::cmd::SubsystemBase*> SmartMotorControllerCommandRegistry::s_owners;

std::string SmartMotorControllerCommandRegistry::MakeKey(const std::string& cmdName,
                                                         wpi::cmd::SubsystemBase* subsystem) {
  return subsystem->GetName() + "/" + cmdName;
}

void SmartMotorControllerCommandRegistry::PublishToNT(const std::string& cmdName,
                                                      wpi::cmd::SubsystemBase* subsystem) {
  auto key = MakeKey(cmdName, subsystem);
  s_commands.insert_or_assign(key, wpi::cmd::Run(
                                       [capturedKey = key] {
                                         for (auto& cb : s_callbacks[capturedKey]) cb();
                                       },
                                       {subsystem})
                                       .WithName(cmdName));
  wpi::SmartDashboard::PutData("Mechanisms/Commands/" + key, s_commands.at(key).get());
}

void SmartMotorControllerCommandRegistry::AddCommand(const std::string& cmdName,
                                                     wpi::cmd::SubsystemBase* subsystem,
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
                                                        wpi::cmd::SubsystemBase* subsystem) {
  return s_commands.count(MakeKey(cmdName, subsystem)) > 0;
}

void SmartMotorControllerCommandRegistry::RemoveCommands(wpi::cmd::SubsystemBase* subsystem) {
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
