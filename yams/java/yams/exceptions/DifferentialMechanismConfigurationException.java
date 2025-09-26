package yams.exceptions;

/**
 * Exception for when the Differential Mechanism is configured incorrectly.
 */
public class DifferentialMechanismConfigurationException extends RuntimeException
{

  /**
   * Differential Mechanism configuration exception.
   *
   * @param message        Message to display.
   * @param result         Result of the configuration.
   * @param remedyFunction Remedy function to use.
   */
  public DifferentialMechanismConfigurationException(String message, String result, String remedyFunction)
  {
    super(message + "!\n" + result + "\nPlease use DifferentialMechanism." + remedyFunction + " to fix this error.");
  }
}
