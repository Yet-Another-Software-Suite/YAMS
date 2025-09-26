package yams.exceptions;

/**
 * Custom exception for when there is no motor in the mechanism.
 */
public class MotorNotPresentException extends RuntimeException
{

  /**
   * Create {@link RuntimeException} for {@link yams.mechanisms.SmartMechanism}
   *
   * @param mechanismType Name of the mechanism
   */
  public MotorNotPresentException(String mechanismType)
  {
    super(mechanismType +
          " primary motor not present! Please set one using `setMotor(SmartMotorController.create(MOTOR_CONTROLLER, DCMotor.getNEO(1))`");
  }
}
