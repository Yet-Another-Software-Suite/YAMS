package yams.gearing;

import yams.exceptions.InvalidStageGivenException;
import yams.exceptions.NoStagesGivenException;

/**
 * GearBox class to calculate input and output conversion factors and check if the current configuration is supported.
 */
public class GearBox
{

  /**
   * Stages in the gear box
   */
  private double[] reductionStages;
  /**
   * Conversion factor of the gearbox from input to output.
   */
  private double   gearReductionRatio;

  /**
   * Construct the {@link GearBox} with the reduction stages given.
   *
   * @param reductionStage Reduction stages where the number is > 0 to indicate a reduction.
   */
  public GearBox(double[] reductionStage)
  {
    setupGearBox(reductionStage);
  }

  /**
   * Construct the {@link GearBox} with the reduction stages given.
   *
   * @param reductionStage List of stages in the format of "IN:OUT".
   */
  public GearBox(String[] reductionStage)
  {
    double[] stages = new double[reductionStage.length];
    for (int i = 0; i < reductionStage.length; i++)
    {
      String stage = reductionStage[i];
      if (!stage.contains(":"))
      {
        throw new InvalidStageGivenException(stage);
      }
      String[] parts = stage.split(":");
      double   in    = Double.parseDouble(parts[0]);
      double   out   = Double.parseDouble(parts[1]);
      stages[i] = in / out;
    }
    setupGearBox(stages);
  }

  /**
   * Create the gearbox given the reduction stages of the gearbox.
   *
   * @param reductionStages Reduction stages where the number is > 0 to indicate a reduction.
   * @return {@link GearBox}.
   */
  public static GearBox fromReductionStages(double... reductionStages)
  {
    return new GearBox(reductionStages);
  }

  /**
   * Create the gearbox given the reduction stages of the gearbox.
   *
   * @param stages Stages in the format of "IN:OUT". For example, "3:1"
   * @return {@link GearBox}
   */
  public static GearBox fromStages(String... stages)
  {
    return new GearBox(stages);
  }

  /**
   * Sets the stages and calculates the reduction for the {@link GearBox}
   *
   * @param reductionStage Reduction stages where the number is > 0 to indicate a reduction.
   */
  private void setupGearBox(double[] reductionStage)
  {
    this.reductionStages = reductionStage;
    if (reductionStages.length == 0)
    {
      throw new NoStagesGivenException();
    }
    double gearBox = 1.0 / reductionStages[0];
    for (int i = 1; i < reductionStages.length; i++)
    {
      gearBox *= (1.0 / reductionStages[i]);
    }
    gearReductionRatio = gearBox;
  }

  /**
   * Multiply the gear reduction ratio by X.
   *
   * @param x X to multiply by.
   * @return {@link GearBox} for chaining.
   */
  public GearBox times(double x)
  {
    gearReductionRatio *= x;
    return this;
  }

  /**
   * Divide the gear reduction ratio by X.
   *
   * @param x X to divide by.
   * @return {@link GearBox} for chaining.
   */
  public GearBox div(double x)
  {
    gearReductionRatio /= x;
    return this;
  }

  /**
   * Get the conversion factor to transform the gearbox input into the gear box output rotations.
   *
   * @return OUT/IN or OUT:IN
   */
  public double getInputToOutputConversionFactor()
  {
    return gearReductionRatio;
  }

  /**
   * Get the conversion factor to transform the gearbox output value into the gear box input value.
   *
   * @return IN:OUT or IN/OUT
   */
  public double getOutputToInputConversionFactor()
  {
    return 1.0 / gearReductionRatio;
  }

}
