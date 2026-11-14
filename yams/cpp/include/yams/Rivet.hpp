// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <string_view>

namespace yams {

/**
 * Rivet, the YASS mascot, says hi on the SystemCore display when YAMS starts up.
 *
 * The art is written once, the first time a YAMS motor controller or swerve drive is built. It
 * uses wpi::DriverStationDisplay line mode, so robot code that writes its own lines afterwards
 * simply replaces it.
 */
class Rivet final {
 public:
  Rivet() = delete;

  /**
   * Show Rivet on the display, with the YAMS version and yassrobotics.com to the right of Rivet and
   * a random phrase underneath. Only the first call does anything.
   */
  static void Show();

  /**
   * Get the YAMS version this library was built as.
   *
   * @return The release version, such as 2026.11.14, or "dev" when YAMS was built from source.
   */
  static std::string_view GetVersion();
};

}  // namespace yams
