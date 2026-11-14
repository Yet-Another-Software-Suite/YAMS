// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <string_view>

namespace yams {

/**
 * Rivet, the YASS mascot, says hi on the Driver Station display when YAMS starts up.
 *
 * The splash is written once, the first time a YAMS motor controller or swerve drive is built, and
 * fits the display's default 25 columns by 10 rows: Rivet on the left, the YAMS version and a
 * random phrase from Rivet beside, and the YASS links along the bottom. It uses
 * wpi::DriverStationDisplay line mode, so robot code that writes its own lines afterwards simply
 * replaces it.
 */
class Rivet final {
 public:
  Rivet() = delete;

  /**
   * Show Rivet on the Driver Station display, with the YAMS version and a random phrase beside
   * Rivet and the YASS links underneath. Only the first call does anything.
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
