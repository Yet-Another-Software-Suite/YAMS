package yams.math;

import yams.exceptions.NoStagesGivenException;

/**
 * SmartMath class to handle math operations.
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
