package yams.exceptions;

/**
 * Exception for when the Elevator is configured incorrectly.
 */
public class ElevatorConfigurationException extends RuntimeException
{

  /**
   * Elevator configuration exception.
   *
   * @param message        Message to display.
   * @param result         Result of the configuration.
   * @param remedyFunction Remedy function to use.
   */
  public ElevatorConfigurationException(String message, String result, String remedyFunction)
  {
    super(message + "!\n" + result + "\nPlease use ElevatorConfig." + remedyFunction + " to fix this error.");
  }
}
