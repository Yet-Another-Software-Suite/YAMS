package yams.exceptions;

/**
 * Exception for when the Shooter is configured incorrectly.
 */
public class ShooterConfigurationException extends RuntimeException
{

  /**
   * Shooter configuration exception.
   *
   * @param message        Message to display.
   * @param result         Result of the configuration.
   * @param remedyFunction Remedy function to use.
   */
  public ShooterConfigurationException(String message, String result, String remedyFunction)
  {
    super(message + "!\n" + result + "\nPlease use ShooterConfig." + remedyFunction + " to fix this error.");
  }
}
