// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers.enums;

/** Control mode for a motor controller. */
public enum ControlMode {
  /** Open loop control mode. Does not use the PID controller. */
  OPEN_LOOP,
  /** Use the PID controller. */
  CLOSED_LOOP,
}
