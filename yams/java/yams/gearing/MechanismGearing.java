package yams.gearing;

import java.util.Optional;

/**
 * Mechanism gearing for conversions from the motor output to the mechanism output.
 */
public class MechanismGearing
{

  /**
   * Mechanism gearbox attached to the motor.
   */
  private final GearBox            gearBox;
  /**
   * Mechanism sprockets attached to the gearbox.
   */
  private       Optional<Sprocket> sprockets = Optional.empty();

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
