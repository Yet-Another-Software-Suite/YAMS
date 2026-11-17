// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.helpers;

import org.junit.platform.launcher.LauncherSession;
import org.junit.platform.launcher.LauncherSessionListener;
import org.wpilib.hardware.hal.HAL;

/**
 * Initializes the HAL once, before JUnit discovers or loads any test class. A test class can make a
 * YAMS config in a static field, and the first config shows Rivet on the Driver Station display,
 * which crashes in native code if the HAL isn't initialized yet. A robot program initializes the HAL
 * before it makes the robot, so only the tests need this. Registered in
 * META-INF/services/org.junit.platform.launcher.LauncherSessionListener.
 */
public class HalInitializationListener implements LauncherSessionListener {
  @Override
  public void launcherSessionOpened(LauncherSession session) {
    HAL.initialize();
  }
}
