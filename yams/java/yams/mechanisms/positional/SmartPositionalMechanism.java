// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechanisms.positional;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import yams.mechanisms.SmartMechanism;
import yams.motorcontrollers.SmartMotorController;

/**
 * Generic for positional mechanisms.
 */
public abstract class SmartPositionalMechanism extends SmartMechanism
{
  /**
   * The root point of the Mechanism.
   */
  protected MechanismRoot2d     m_mechanismRoot;
  /**
   * The ligament that is being moved.
   */
  protected MechanismLigament2d m_mechanismLigament;

  /**
   * {@link SmartPositionalMechanism} is at max, defined by the soft limit or hard limit on the
   * {@link SmartPositionalMechanism}.
   *
   * @return Maximum angle for the {@link SmartPositionalMechanism}.
   */
  public abstract Trigger max();

  /**
   * Minimum angle of the {@link SmartPositionalMechanism} given by the soft limit or hard limit of the
   * {@link SmartPositionalMechanism}.
   *
   * @return {@link Trigger} on minimum of the {@link SmartPositionalMechanism}.
   */
  public abstract Trigger min();

  /**
   * Get the ligament of the 2D mechanism model. Used to change the position of the mechanism model in the
   * SmartDashboard.
   *
   * @return Ligament of the 2D mechanism model.
   */
  public MechanismLigament2d getMechanismLigament()
  {
    return m_mechanismLigament;
  }

  /**
   * The root of the 2D mechanism model.
   *
   * @return Root of the 2D mechanism model.
   */
  public MechanismRoot2d getMechanismRoot()
  {
    return m_mechanismRoot;
  }

  /**
   * Get the motor controller which is moving the mechanism.
   *
   * @return Motor controller which is moving the mechanism.
   */
  public SmartMotorController getMotor()
  {
    return m_smc;
  }
}
