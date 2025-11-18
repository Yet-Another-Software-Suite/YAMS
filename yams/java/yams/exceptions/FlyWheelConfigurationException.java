package yams.exceptions;

/**
 * Exception for when the FlyWheel is configured incorrectly.
 */
public class FlyWheelConfigurationException extends RuntimeException
{

  /**
   * FlyWheel configuration exception.
   *
   * @param message        Message to display.
   * @param result         Result of the configuration.
   * @param remedyFunction Remedy function to use.
   */
  public FlyWheelConfigurationException(String message, String result, String remedyFunction)
  {
    super(message + "!\n" + result + "\nPlease use FlyWheelConfig." + remedyFunction + " to fix this error.");
  }
}
