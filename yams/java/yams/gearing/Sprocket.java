// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.gearing;

import yams.exceptions.InvalidStageGivenException;
import yams.exceptions.NoStagesGivenException;

/**
 * Sprocket class to handle calculating the conversion factor of a sprocket in your mechanism.
 *
 * <p>A {@link Sprocket} models a chain or belt drive made up of two or more sprockets/pulleys.
 * The ratio for each stage is expressed as {@code driver / driven} (IN/OUT). A ratio less than
 * 1.0 indicates a reduction (the driven sprocket has more teeth than the driver).</p>
 *
 * <p>You can construct a {@link Sprocket} in several ways:</p>
 * <ul>
 *   <li><b>{@code new Sprocket(double...)}</b> — provide per-stage IN/OUT ratios directly</li>
 *   <li><b>{@code fromStages(String...)}</b> — provide stages as {@code "IN:OUT"} strings</li>
 * </ul>
 *
 * <h2>Example</h2>
 * <pre>{@code
 * // 2:1 reduction — 18-tooth driver sprocket driving a 36-tooth driven sprocket
 * // The ratio passed is driver/driven = 18/36 = 0.5
 * Sprocket twoToOne = new Sprocket(18.0 / 36.0);
 *
 * // Equivalent using an "IN:OUT" stage string
 * Sprocket fromString = Sprocket.fromStages("18:36");
 *
 * // Get the input-to-output conversion factor (< 1.0 means the output spins slower)
 * double factor = twoToOne.getInputToOutputConversionFactor(); // 0.5
 * // Get the output-to-input conversion factor (the reduction ratio)
 * double reduction = twoToOne.getOutputToInputConversionFactor(); // 2.0
 * }</pre>
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
  public Sprocket(double... sprocketReductionStage)
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
   * @return Sprocket representation
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
