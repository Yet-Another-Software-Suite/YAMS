package yams.telemetry;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.measure.*;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

import static edu.wpi.first.units.Units.*;

public class SmartMotorControllerTelemetry {

  /**
   * Mechanism lower limit reached.
   */
  public boolean         mechanismLowerLimit = false;
  /**
   * Mechanism upper limit reached.
   */
  public boolean         mechanismUpperLimit = false;
  /**
   * Motor temperature cutoff reached.
   */
  public boolean         temperatureLimit    = false;
  /**
   * Velocity PID controller used.
   */
  public boolean         velocityControl     = false;
  /**
   * Elevator feedforward used.
   */
  public boolean         elevatorFeedforward = false;
  /**
   * Arm feedforward used.
   */
  public boolean         armFeedforward      = false;
  /**
   * Simple feedforward used.
   */
  public boolean         simpleFeedforward   = false;
  /**
   * Motion profiling used.
   */
  public boolean         motionProfile       = false;
  /**
   * Setpoint position given.
   */
  public double          setpointPosition    = 0;
  /**
   * Setpoint velocity given.
   */
  public double          setpointVelocity    = 0;
  /**
   * Feedforward voltage supplied to the {@link SmartMotorController}
   */
  public double          feedforwardVoltage  = 0.0;
  /**
   * PID Output voltage supplied to the {@link SmartMotorController}
   */
  public double          pidOutputVoltage    = 0.0;
  /**
   * Output voltage to the {@link SmartMotorController}
   */
  public double          outputVoltage       = 0.0;
  /**
   * Stator current (motor controller output current) to the Motor.
   */
  public double          statorCurrent       = 0.0;
  /**
   * Motor temperature.
   */
  public Temperature     temperature         = Fahrenheit.of(72);
  /**
   * Mechanism distance.
   */
  public Distance        distance            = Meters.of(0);
  /**
   * Mechanism linear velocity.
   */
  public LinearVelocity  linearVelocity      = MetersPerSecond.of(0);
  /**
   * Mechanism position.
   */
  public Angle           mechanismPosition;
  /**
   * Mechanism velocity.
   */
  public AngularVelocity mechanismVelocity;
  /**
   * Rotor position.
   */
  public Angle           rotorPosition;
  /**
   * Rotor velocity.
   */
  public AngularVelocity rotorVelocity;
  /**
   * Network table to publish to.
   */
  private NetworkTable     table;
  /**
   * Mechanism lower limit boolean publisher
   */
  private BooleanPublisher mechanismLowerLimitPublisher;
  /**
   * Mechanism upper limit boolean publisher
   */
  private BooleanPublisher mechanismUpperLimitPublisher;
  /**
   * Motor temperature limit hit.
   */
  private BooleanPublisher temperatureLimitPublisher;
  /**
   * Velocity control used.
   */
  private BooleanPublisher velocityControlPublisher;
  /**
   * Elevator feedforward used.
   */
  private BooleanPublisher elevatorFeedforwardPublisher;
  /**
   * Arm feedforward used.
   */
  private BooleanPublisher armFeedforwardPublisher;
  /**
   * Simple motor feedforward used.
   */
  private BooleanPublisher simpleFeedforwardPublisher;
  /**
   * Motion profile used.
   */
  private BooleanPublisher motionProfilePublisher;
  /**
   * Setpoint targetted.
   */
  private DoublePublisher  setpointPositionPublisher;
  /**
   * Setpoint velocity targetted.
   */
  private DoublePublisher  setpointVelocityPublisher;
  /**
   * Feedforward voltage output.
   */
  private DoublePublisher  feedforwardVoltagePublisher;
  /**
   * PID Output voltage.
   */
  private DoublePublisher  pidOutputVoltagePublisher;
  /**
   * Motor controller output voltage.
   */
  private DoublePublisher  outputVoltagePublisher;
  /**
   * Stator current output.
   */
  private DoublePublisher  statorCurrentPublisher;
  /**
   * Motor temperature
   */
  private DoublePublisher  temperaturePublisher;
  /**
   * Distance/Mechanism measurement
   */
  private DoublePublisher  measurementPositionPublisher;
  /**
   * Linear Velocity/Mechanism measurement velocity.
   */
  private DoublePublisher  measurementVelocityPublisher;
  /**
   * Mechanism position
   */
  private DoublePublisher  mechanismPositionPublisher;
  /**
   * Mechanism velocity
   */
  private DoublePublisher  mechanismVelocityPublisher;
  /**
   * Rotor position
   */
  private DoublePublisher  rotorPositionPublisher;
  /**
   * Rotor velocity.
   */
  private DoublePublisher  rotorVelocityPublisher;

  /**
   * Publish {@link SmartMotorController} telemetry to {@link NetworkTable}
   *
   * @param publishTable {@link NetworkTable} to publish to.
   * @param verbosity    {@link TelemetryVerbosity} to publish.
   */
  public void publish(NetworkTable publishTable, TelemetryVerbosity verbosity)
  {
    if (!publishTable.equals(this.table))
    {
      table = publishTable;
      mechanismLowerLimitPublisher = table.getBooleanTopic("Mechanism Lower Limit").publish();
      mechanismUpperLimitPublisher = table.getBooleanTopic("Mechanism Upper Limit").publish();
      temperatureLimitPublisher = table.getBooleanTopic("Temperature Limit").publish();
      velocityControlPublisher = table.getBooleanTopic("Velocity Control").publish();
      elevatorFeedforwardPublisher = table.getBooleanTopic("Elevator Feedforward").publish();
      armFeedforwardPublisher = table.getBooleanTopic("Arm Feedforward").publish();
      simpleFeedforwardPublisher = table.getBooleanTopic("Simple Feedforward").publish();
      motionProfilePublisher = table.getBooleanTopic("Motion Profile").publish();
      setpointPositionPublisher = table.getDoubleTopic("Setpoint Position (Rotations)").publish();
      setpointVelocityPublisher = table.getDoubleTopic("Setpoint Velocity (Rotations per Second)").publish();
      feedforwardVoltagePublisher = table.getDoubleTopic("Feedforward Voltage").publish();
      pidOutputVoltagePublisher = table.getDoubleTopic("PID Output (Voltage)").publish();
      outputVoltagePublisher = table.getDoubleTopic("Motor Output Voltage").publish();
      statorCurrentPublisher = table.getDoubleTopic("Stator Current (Amps)").publish();
      temperaturePublisher = table.getDoubleTopic("Temperature (Celsius)").publish();
      measurementPositionPublisher = table.getDoubleTopic("Measurement Position (Meters)").publish();
      measurementVelocityPublisher = table.getDoubleTopic("Measurement Velocity (Meters per Second)").publish();
      mechanismPositionPublisher = table.getDoubleTopic("Mechanism Position (Rotations)").publish();
      mechanismVelocityPublisher = table.getDoubleTopic("Mechanism Velocity (Rotations per Second)").publish();
      rotorPositionPublisher = table.getDoubleTopic("Rotor Position (Rotations)").publish();
      rotorVelocityPublisher = table.getDoubleTopic("Rotor Velocity (Rotations per Second)").publish();
    }
    if (table != null)
    {
      mechanismLowerLimitPublisher.set(mechanismLowerLimit);
      mechanismUpperLimitPublisher.set(mechanismUpperLimit);
      temperatureLimitPublisher.set(temperatureLimit);
      velocityControlPublisher.set(velocityControl);
      elevatorFeedforwardPublisher.set(elevatorFeedforward);
      armFeedforwardPublisher.set(armFeedforward);
      simpleFeedforwardPublisher.set(simpleFeedforward);
      motionProfilePublisher.set(motionProfile);
      setpointPositionPublisher.set(setpointPosition);
      setpointVelocityPublisher.set(setpointVelocity);
      feedforwardVoltagePublisher.set(feedforwardVoltage);
      pidOutputVoltagePublisher.set(pidOutputVoltage);
      outputVoltagePublisher.set(outputVoltage);
      statorCurrentPublisher.set(statorCurrent);
      temperaturePublisher.set(temperature.in(Celsius));
      measurementPositionPublisher.set(distance.in(Meters));
      measurementVelocityPublisher.set(linearVelocity.in(MetersPerSecond));
      mechanismPositionPublisher.set(mechanismPosition.in(Rotations));
      mechanismVelocityPublisher.set(mechanismVelocity.in(RotationsPerSecond));
      rotorPositionPublisher.set(rotorPosition.in(Rotations));
      rotorVelocityPublisher.set(rotorVelocity.in(RotationsPerSecond));
    }
  }
}
