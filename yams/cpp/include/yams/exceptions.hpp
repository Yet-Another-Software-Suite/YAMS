// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <stdexcept>
#include <string>

namespace yams::exceptions {
/**
 * Base exception class for all YAMS runtime errors.
 *
 * All YAMS exceptions derive from this class, which itself extends std::runtime_error.
 */
class YamsException : public std::runtime_error {
 public:
  /**
   * Construct a YamsException with the given message.
   *
   * @param message Human-readable description of the error.
   */
  explicit YamsException(const std::string& message) : std::runtime_error(message) {}
};

/**
 * Thrown when an ArmConfig is incomplete or missing required simulation fields.
 */
class ArmConfigurationException : public YamsException {
 public:
  /**
   * Construct an ArmConfigurationException.
   *
   * @param issue  Short description of what is misconfigured.
   * @param result Description of the undesirable consequence.
   * @param fix    Suggested remediation for the caller.
   */
  ArmConfigurationException(const std::string& issue, const std::string& result,
                            const std::string& fix)
      : YamsException(
            "[Arm Configuration Error]\n"
            "Issue: " +
            issue + "\nResult: " + result + "\nFix: " + fix) {}
};

/**
 * Thrown when a SwerveDriveConfig is invalid or incomplete.
 */
class SwerveDriveConfigurationException : public YamsException {
 public:
  /**
   * Construct a SwerveDriveConfigurationException.
   *
   * @param message Description of the configuration error.
   */
  explicit SwerveDriveConfigurationException(const std::string& message)
      : YamsException("[SwerveDrive Configuration Error]\n" + message) {}
};

/**
 * Thrown when a SmartMotorControllerConfig is incomplete or contains conflicting settings.
 */
class SmartMotorControllerConfigurationException : public YamsException {
 public:
  /**
   * Construct a SmartMotorControllerConfigurationException.
   *
   * @param issue  Short description of what is misconfigured.
   * @param result Description of the undesirable consequence.
   * @param fix    Suggested remediation for the caller.
   */
  SmartMotorControllerConfigurationException(const std::string& issue, const std::string& result,
                                             const std::string& fix)
      : YamsException(
            "[SmartMotorController Configuration Error]\n"
            "Issue: " +
            issue + "\nResult: " + result + "\nFix: " + fix) {}
};

/**
 * Thrown when a PivotConfig is incomplete or missing required simulation fields.
 */
class PivotConfigurationException : public YamsException {
 public:
  /**
   * Construct a PivotConfigurationException.
   *
   * @param issue  Short description of what is misconfigured.
   * @param result Description of the undesirable consequence.
   * @param fix    Suggested remediation for the caller.
   */
  PivotConfigurationException(const std::string& issue, const std::string& result,
                              const std::string& fix)
      : YamsException(
            "[Pivot Configuration Error]\n"
            "Issue: " +
            issue + "\nResult: " + result + "\nFix: " + fix) {}
};

/**
 * Thrown when a GearBox or Sprocket is constructed with an empty list of reduction stages.
 */
class NoStagesGivenException : public YamsException {
 public:
  /** Construct a NoStagesGivenException. */
  NoStagesGivenException()
      : YamsException("No reduction stages were given to the GearBox/Sprocket!") {}
};

/**
 * Thrown when a gearbox or sprocket stage string is not in the required "IN:OUT" format.
 */
class InvalidStageGivenException : public YamsException {
 public:
  /**
   * Construct an InvalidStageGivenException for the offending stage string.
   *
   * @param stage The invalid stage string that was provided.
   */
  explicit InvalidStageGivenException(const std::string& stage)
      : YamsException("Invalid stage given: \"" + stage +
                      "\". Stage must be in the format of \"IN:OUT\"") {}
};

/**
 * Thrown when an ElevatorConfig is incomplete or missing required simulation fields.
 */
class ElevatorConfigurationException : public YamsException {
 public:
  /**
   * Construct an ElevatorConfigurationException.
   *
   * @param issue  Short description of what is misconfigured.
   * @param result Description of the undesirable consequence.
   * @param fix    Suggested remediation for the caller.
   */
  ElevatorConfigurationException(const std::string& issue, const std::string& result,
                                 const std::string& fix)
      : YamsException(
            "[Elevator Configuration Error]\n"
            "Issue: " +
            issue + "\nResult: " + result + "\nFix: " + fix) {}
};

}  // namespace yams::exceptions
