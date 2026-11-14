// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <string_view>

namespace yams {

/**
 * Rivet, the YASS mascot, says hi on the Driver Station display when YAMS starts up.
 *
 * The splash is written once, the first time a YAMS motor controller or swerve drive is built. The
 * Display panel docked in the Driver Station shows about 26 columns by 9 rows, so it gets a short
 * welcome with the YAMS version, yassrobotics.com and the live tuner, plus an arrow to the panel's
 * pop-out button. Popping the display out (or scrolling right) shows Rivet at full resolution with
 * a random phrase from Rivet under it. It uses wpi::DriverStationDisplay line mode, so robot code
 * that writes its own lines afterwards simply replaces it.
 */
class Rivet final {
 public:
  Rivet() = delete;

  /** Show the welcome and Rivet on the Driver Station display. Only the first call does anything. */
  static void Show();

  /**
   * Get the YAMS version this library was built as.
   *
   * @return The release version, such as 2026.11.14, or "dev" when YAMS was built from source.
   */
  static std::string_view GetVersion();
};

}  // namespace yams
