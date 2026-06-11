// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechanisms.velocity;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import yams.mechanisms.SmartMechanism;
import yams.motorcontrollers.SmartMotorController;

/**
 * Generic for positional mechanisms.
 */
public abstract class SmartVelocityMechanism extends SmartMechanism
{
  /**
   * The root point of the Mechanism.
   */
  protected MechanismRoot2d     mechanismRoot;
  /**
   * The ligament that is being moved.
   */
  protected MechanismLigament2d mechanismLigament;

  /**
   * {@link SmartVelocityMechanism} is at max, defined by the soft limit or hard limit on the
   * {@link SmartVelocityMechanism}.
   *
   * @return Maximum angle for the {@link SmartVelocityMechanism}.
   */
  public abstract Trigger max();

  /**
   * Minimum angle of the {@link SmartVelocityMechanism} given by the soft limit or hard limit of the
   * {@link SmartVelocityMechanism}.
   *
   * @return {@link Trigger} on minimum of the {@link SmartVelocityMechanism}.
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
    return mechanismLigament;
  }

  /**
   * The root of the 2D mechanism model.
   *
   * @return Root of the 2D mechanism model.
   */
  public MechanismRoot2d getMechanismRoot()
  {
    return mechanismRoot;
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
