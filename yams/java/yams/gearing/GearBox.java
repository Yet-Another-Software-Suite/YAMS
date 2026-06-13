// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.gearing;

import yams.exceptions.InvalidStageGivenException;
import yams.exceptions.NoStagesGivenException;

/**
 * GearBox class to calculate input and output conversion factors and check if the current configuration is supported.
 *
 * <p>A {@link GearBox} models a multi-stage gear reduction (or increase). Each stage is defined
 * by a ratio (driver teeth / driven teeth) or a plain ratio double. The overall ratio is the
 * product of all individual stage ratios.</p>
 *
 * <p>You can construct a {@link GearBox} in several ways:</p>
 * <ul>
 *   <li><b>{@code fromTeeth(int...)}</b> — provide alternating driver/driven tooth counts</li>
 *   <li><b>{@code fromStages(String...)}</b> — provide stages as {@code "IN:OUT"} strings</li>
 *   <li><b>{@code fromReductionStages(double...)}</b> — provide per-stage ratios directly</li>
 * </ul>
 *
 * <h2>Example</h2>
 * <pre>{@code
 * // 5:1 single-stage gearbox (12-tooth driver meshing with a 60-tooth driven gear)
 * GearBox fiveToOne = GearBox.fromTeeth(12, 60);
 *
 * // 25:1 two-stage gearbox (each stage is 5:1)
 * GearBox twoStage = GearBox.fromTeeth(12, 60, 12, 60);
 *
 * // Equivalent using "IN:OUT" stage strings
 * GearBox fromStrings = GearBox.fromStages("12:60", "12:60");
 *
 * // Equivalent using raw reduction ratios (driver/driven per stage)
 * GearBox fromRatios = GearBox.fromReductionStages(12.0 / 60.0, 12.0 / 60.0);
 *
 * // Get the overall input-to-output conversion factor (< 1.0 means reduction)
 * double factor = fiveToOne.getInputToOutputConversionFactor(); // 0.2  (1/5)
 * // Get the overall output-to-input conversion factor (the reduction ratio itself)
 * double reduction = fiveToOne.getOutputToInputConversionFactor(); // 5.0
 * }</pre>
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
   * Create the gearbox given the teeth of each gear.
   *
   * @param teeth Gear teeth from driven gear to drive gear.
   * @return {@link GearBox}
   */
  public static GearBox fromTeeth(int... teeth)
  {
    if (teeth == null || teeth.length < 2)
    {
      throw new IllegalArgumentException(
          "At least two gears (drive and driven) are required"
      );
    }

    double reductionRatio = 1.0;

    for (int i = 0; i < teeth.length - 1; i++)
    {
      if (teeth[i] <= 0 || teeth[i + 1] <= 0)
      {
        throw new IllegalArgumentException(
            "Gear teeth counts must be positive integers"
        );
      }

      reductionRatio *= (double) teeth[i + 1] / teeth[i];
    }

    return new GearBox(new double[]{reductionRatio});
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
