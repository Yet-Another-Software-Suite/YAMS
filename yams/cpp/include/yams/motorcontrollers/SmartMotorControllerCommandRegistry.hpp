// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace yams::motorcontrollers {

/**
 * Registry that aggregates per-subsystem commands from multiple SmartMotorControllers.
 *
 * When multiple SmartMotorControllers belong to the same subsystem (e.g. a dual-motor arm),
 * each one registers its live-tuning callback here.  The registry creates a single combined
 * command per subsystem that runs all registered callbacks on every periodic loop.
 *
 * The published SmartDashboard path is: Mechanisms/Commands/%SubsystemName%/%CommandName%
 */
class SmartMotorControllerCommandRegistry {
 public:
  /**
   * Register a callback for a command on a subsystem.
   *
   * If no command with this name exists for the subsystem yet, a new one is created and
   * published to SmartDashboard.  If one already exists, the callback is appended to it.
   *
   * @param cmdName   Display name for the command (e.g. "Live Tuning").
   * @param subsystem Subsystem that owns the command (used for requirement and naming).
   * @param callback  Runnable invoked each loop while the command is running.
   */
  static void AddCommand(const std::string& cmdName, frc2::SubsystemBase* subsystem,
                         std::function<void()> callback);

  /**
   * Check whether a command with the given name already exists for a subsystem.
   *
   * @param cmdName   Command name.
   * @param subsystem Subsystem.
   * @return true if the command is already registered.
   */
  static bool CommandExists(const std::string& cmdName, frc2::SubsystemBase* subsystem);

  /**
   * Remove all commands registered for a specific subsystem instance.
   *
   * Call this when a subsystem is being torn down (e.g. between tests) so
   * that a new instance with the same name can register without triggering
   * the conflict check.
   *
   * @param subsystem Subsystem whose commands should be removed.
   */
  static void RemoveCommands(frc2::SubsystemBase* subsystem);

  /**
   * Destroy all registered commands and callbacks.
   *
   * Must be called before program exit while WPILib's SendableRegistry is
   * still alive, otherwise the CommandPtr destructors will attempt to lock an
   * already-destroyed mutex and crash (SIGSEGV/exit 139).
   */
  static void Clear();

 private:
  static std::unordered_map<std::string, frc2::CommandPtr> s_commands;
  static std::unordered_map<std::string, std::vector<std::function<void()>>> s_callbacks;
  static std::unordered_map<std::string, frc2::SubsystemBase*> s_owners;

  static std::string MakeKey(const std::string& cmdName, frc2::SubsystemBase* subsystem);
  static void PublishToNT(const std::string& cmdName, frc2::SubsystemBase* subsystem);
};

}  // namespace yams::motorcontrollers
