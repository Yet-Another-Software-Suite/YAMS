// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import org.wpilib.math.system.DCMotor;

/**
 * {@link DCMotor} models of motors WPILib does not have a model of.
 *
 * <p>The motor controller wrappers recognize a motor by comparing its {@link DCMotor} with these
 * (see {@link SmartMotorController#isMotor(DCMotor, DCMotor)}), so use them rather than building
 * your own from the same specs.
 *
 * <p>This class will be deleted once WPILib has a {@link DCMotor} of the Thrifty Pulsar. Use
 * WPILib's then.
 */
public final class YMotors {
  private YMotors() {}

  /**
   * The Thrifty Pulsar 775, from Thrifty's specs: 12 V, 3.1 N-m and 189 A stalled, and 2.7 A at 7500
   * RPM free. A Talon FXS drives it as a NEO.
   *
   * <p>YAMG generates the same model as {@code new DCMotor(12.0, 3.1, 189.0, 2.7, 7500 * 2 * Math.PI
   * / 60, numMotors)}. Keep the two the same, since the comparison is exact.
   *
   * @param numMotors How many Pulsars are in the gearbox.
   * @return The Thrifty Pulsar's {@link DCMotor}.
   */
  public static DCMotor getThriftyPulsar(int numMotors) {
    return new DCMotor(12.0, 3.1, 189.0, 2.7, 7500 * 2 * Math.PI / 60, numMotors);
  }
}
