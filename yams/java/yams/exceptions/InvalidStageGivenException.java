package yams.exceptions;

/**
 * Exception for math errors when trying to find the sensor to mechanism ratio.
 */
public class InvalidStageGivenException extends RuntimeException
{

  /**
   * Constructs exception for failure to provide stages.
   *
   * @param stage Stage given.
   */
  public InvalidStageGivenException(String stage)
  {
    super("Invalid stage given! '" + stage + "'; should be in the format of 'IN:OUT'!");
  }
}
