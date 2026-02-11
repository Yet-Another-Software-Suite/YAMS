package yams.exceptions;

/**
 * SwerveDrive Config Exception
 */
public class SwerveDriveConfigurationException extends RuntimeException {
    /**
     * SwerveDrive configuration exception.
     *
     * @param message        Message to display.
     * @param result         Result of the configuration.
     * @param remedyFunction Remedy function to use.
     */
    public SwerveDriveConfigurationException(String message, String result, String remedyFunction)
    {
        super(message + "!\n" + result + "\nPlease use SwerveDriveConfig." + remedyFunction + " to fix this error.");
    }
}
