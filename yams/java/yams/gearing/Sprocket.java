package yams.gearing;

import yams.exceptions.InvalidStageGivenException;
import yams.exceptions.NoStagesGivenException;

/**
 * Sprocket class to handle calculating the conversion factor of a sprocket in your mechanism.
 */
public class Sprocket
{

  /**
   * Stages in the Sprocket chain.
   */
  private double[] reductionStages;
  /**
   * The input to output conversion factor.
   */
  private double   sprocketReductionRatio;

  /**
   * Create the sprocket given the teeth of each sprocket in the chain.
   *
   * @param sprocketReductionStage Sprocket teeth, in the form of "IN:OUT" => IN/OUT
   */
  public Sprocket(double[] sprocketReductionStage)
  {
    setupStages(sprocketReductionStage);
  }

  /**
   * Construct the {@link Sprocket} with the reduction stages given.
   *
   * @param reductionStage List of stages in the format of "IN:OUT".
   */
  public Sprocket(String[] reductionStage)
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
    setupStages(stages);
  }

  /**
   * Construct the {@link Sprocket} with the reduction stages given.
   *
   * @param stages List of stages in the format of "IN:OUT".
   */
  public static Sprocket fromStages(String... stages)
  {
    return new Sprocket(stages);
  }

  /**
   * Set up the reduction stages for the {@link Sprocket}
   *
   * @param sprocketReductionStage Reductions in the form of "IN:OUT" => IN/OUT
   */
  private void setupStages(double[] sprocketReductionStage)
  {
    reductionStages = sprocketReductionStage;
    if (reductionStages.length == 0)
    {
      throw new NoStagesGivenException();
    }
    double sprocketRatio = (1 / reductionStages[0]);
    for (int i = 1; i < reductionStages.length; i++)
    {
      sprocketRatio *= (1 / reductionStages[i]);
    }
    sprocketReductionRatio = sprocketRatio;
  }

  /**
   * Multiply the sprocket reduction ratio by X.
   *
   * @param x X to multiply by.
   * @return {@link Sprocket} for chaining.
   */
  public Sprocket times(double x)
  {
    sprocketReductionRatio *= x;
    return this;
  }

  /**
   * Divide the sprocket reduction ratio by X.
   *
   * @param x X to divide by.
   * @return {@link Sprocket}
   */
  public Sprocket div(double x)
  {
    sprocketReductionRatio /= x;
    return this;
  }

  /**
   * Get the conversion factor to transform the sprocket input into the sprocket output rotations.
   *
   * @return OUT/IN or OUT:IN
   */
  public double getInputToOutputConversionFactor()
  {
    return 1.0 / sprocketReductionRatio;
  }

  /**
   * Get the conversion factor to transform the sprocket output value into the sprocket input value.
   *
   * @return IN:OUT or IN/OUT
   */
  public double getOutputToInputConversionFactor()
  {
    return sprocketReductionRatio;
  }


}
