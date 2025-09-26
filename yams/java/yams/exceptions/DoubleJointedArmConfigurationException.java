package yams.exceptions;

/**
 * Exception for when the Arm is configured incorrectly.
 */
public class DoubleJointedArmConfigurationException extends RuntimeException
{

  /**
   * Arm configuration exception.
   *
   * @param message        Message to display.
   * @param result         Result of the configuration.
   * @param remedyFunction Remedy function to use.
   */
  public DoubleJointedArmConfigurationException(String message, String result, String remedyFunction)
  {
    super(message + "!\n" + result + "\nPlease use ArmConfig." + remedyFunction + " to fix this error.");
  }
}
