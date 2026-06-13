// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.math;

import yams.exceptions.NoStagesGivenException;

/**
 * SmartMath class to handle math operations.
 *
 * <p>Provides static utility methods for common FRC drivetrain and gearing calculations:
 * <ul>
 *   <li><b>sensorToMechanismRatio</b> — multiplies a chain of gear-stage ratios to produce the
 *       overall sensor-to-mechanism ratio used by motor controller configuration.</li>
 *   <li><b>gearBox</b> — multiplies a chain of gear-stage ratios to produce the rotor-to-mechanism
 *       gear ratio (MECHANISM_ROTATIONS / ROTOR_ROTATIONS).</li>
 * </ul>
 *
 * <h2>Example</h2>
 * <pre>{@code
 * // Single-stage 10:1 reduction — sensor on motor shaft
 * double sensorRatio = SmartMath.sensorToMechanismRatio(10.0);
 *
 * // Two-stage gearbox: 5:1 first stage, 4:1 second stage → overall 20:1
 * double gearRatio = SmartMath.gearBox(5.0, 4.0);
 *
 * // Three-stage gearbox: 3:1, 4:1, 5:1 → overall 60:1
 * double complexGearRatio = SmartMath.gearBox(3.0, 4.0, 5.0);
 * }</pre>
 */
public class SmartMath
{
  /**
   * Create the sensor to mechanism ratio.
   *
   * @param stages stages between the motor and output shaft.
   * @return sensor to mechanism ratio.
   */
  public static double sensorToMechanismRatio(double... stages)
  {
    if (stages.length == 0)
    {
      throw new NoStagesGivenException();
    }
    double sensorToMechanismRatio = stages[0];
    for (int i = 1; i < stages.length; i++)
    {
      sensorToMechanismRatio *= stages[i];
    }
    return sensorToMechanismRatio;
  }

  /**
   * Create the gear ratio based off of the stages in the gear box.
   *
   * @param stages stages between the motor and output shaft.
   * @return rotor rotations to mechanism ratio in the form of MECHANISM_ROTATIONS/ROTOR_ROTATIONS or
   * ROTOR_ROTATIONS:MECHANISM_ROTATIONS
   */
  public static double gearBox(double... stages)
  {
    if (stages.length == 0)
    {
      throw new NoStagesGivenException();
    }
    double gearBox = stages[0];
    for (int i = 1; i < stages.length; i++)
    {
      gearBox *= stages[i];
    }
    return gearBox;
  }
}
