// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.gearing;

import java.util.Optional;

/**
 * Mechanism gearing for conversions from the motor output to the mechanism output.
 *
 * <p>{@link MechanismGearing} combines a {@link GearBox} (and an optional {@link Sprocket})
 * into a single object that describes the complete power-transmission path between the motor
 * and the mechanism. It is the primary object passed to motor-controller configuration helpers
 * (such as {@code SmartMotorControllerConfig}) so that encoder readings can be automatically
 * scaled to real-world mechanism positions or velocities.</p>
 *
 * <p>Constructors available:</p>
 * <ul>
 *   <li><b>{@code new MechanismGearing(double)}</b> — single overall reduction ratio</li>
 *   <li><b>{@code new MechanismGearing(double...)}</b> — one ratio per gearbox stage</li>
 *   <li><b>{@code new MechanismGearing(GearBox)}</b> — pre-built {@link GearBox}</li>
 *   <li><b>{@code new MechanismGearing(GearBox, Sprocket)}</b> — gearbox followed by a chain/belt stage</li>
 * </ul>
 *
 * <h2>Example</h2>
 * <pre>{@code
 * // Simple 10:1 gearbox attached directly to the mechanism
 * MechanismGearing simple = new MechanismGearing(10.0);
 *
 * // Two-stage gearbox (5:1 then 3:1 = 15:1 overall)
 * MechanismGearing twoStage = new MechanismGearing(5.0, 3.0);
 *
 * // Gearbox followed by a 2:1 chain-drive sprocket stage
 * GearBox gearBox  = GearBox.fromTeeth(12, 60);          // 5:1
 * Sprocket sprocket = Sprocket.fromStages("18:36");       // 2:1
 * MechanismGearing combined = new MechanismGearing(gearBox, sprocket); // 10:1 overall
 *
 * // Query the ratios
 * double rotorToMech = combined.getRotorToMechanismRatio(); // motor rotations per mechanism rotation
 * double mechToRotor = combined.getMechanismToRotorRatio(); // mechanism rotations per motor rotation
 * }</pre>
 */
public class MechanismGearing
{
  /**
   * 1:1 Mechanism Gearing.
   */
  public static final MechanismGearing kOne = new MechanismGearing(1.0);
  /**
   * Mechanism gearbox attached to the motor.
   */
  private final GearBox            gearBox;
  /**
   * Mechanism sprockets attached to the gearbox.
   */
  private       Optional<Sprocket> sprockets = Optional.empty();

  /**
   * Construct a {@link MechanismGearing} with a reduction ratio.
   *
   * @param reductionRatio Reduction ratio. For example, a reduction of "3:1" is 3.0; a reduction of "1:2" is 0.5.
   */
  public MechanismGearing(double reductionRatio)
  {
    gearBox = GearBox.fromReductionStages(reductionRatio);
  }

  /**
   * Construct a {@link MechanismGearing} with a reduction ratios.
   *
   * @param reductionRatios Reduction ratio. For example, a reduction of "3:1" is 3.0; a reduction of "1:2" is 0.5.
   */
  public MechanismGearing(double... reductionRatios)
  {
    gearBox = GearBox.fromReductionStages(reductionRatios);
  }

  /**
   * Initialize the {@link MechanismGearing} with only a {@link GearBox} attached to the mechanism motor.
   *
   * @param gearBox {@link GearBox} of the Mechanism.
   */
  public MechanismGearing(GearBox gearBox)
  {
    this.gearBox = gearBox;
  }

  /**
   * Initialize the {@link MechanismGearing} with a {@link GearBox} and {@link Sprocket}
   *
   * @param gearBox   {@link GearBox} attached to the motor.
   * @param sprockets {@link Sprocket} attached to the gearbox.
   */
  public MechanismGearing(GearBox gearBox, Sprocket sprockets)
  {
    this.gearBox = gearBox;
    this.sprockets = Optional.of(sprockets);
  }

  /**
   * Get the sensor to the mechanism ratio for the motor to the mechanism.
   *
   * @return OUT:IN or OUT/IN ratio to use for sensor to mechanism calculations.
   */
  public double getRotorToMechanismRatio()
  {
    double ratio = gearBox.getInputToOutputConversionFactor();
    if (sprockets.isPresent())
    {
      ratio *= sprockets.get().getInputToOutputConversionFactor();
    }
    return ratio;
  }

  /**
   * Get the mechanism rotation to sensor rotation ratio for the mechanism. AKA THE REDUCTION!
   *
   * @return IN:OUT or IN/OUT to use for mechanism to sensor calculations.
   */
  public double getMechanismToRotorRatio()
  {
    double ratio = gearBox.getOutputToInputConversionFactor();
    if (sprockets.isPresent())
    {
      ratio *= sprockets.get().getOutputToInputConversionFactor();
    }
    return ratio;
  }

  /**
   * Divide the gearbox reduction ratio by i.
   *
   * @param i Numerator.
   * @return {@link MechanismGearing}
   */
  public MechanismGearing div(double i)
  {
    gearBox.div(i);
    return this;
  }
}
