package yams.telemetry;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.Measure;
import java.util.List;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import yams.motorcontrollers.SmartMotorController;

public class MechanismTelemetry
{

  /**
   * Telemetry NetworkTable.
   */
  private NetworkTable         networkTable;
  /**
   * Tuning NetworkTable.
   */
  private NetworkTable         tuningNetworkTable;
  /**
   * Network Tables publisher for the units displayed by the positional mechanism.
   */
  private StringPublisher      unitsPublisher;
  /**
   * Units for the mechanism setpoint and position.
   */
  private String               unitsString;
  /**
   * Setpoint for tracking changes.
   */
  private double           setpoint;
  /**
   * Network Tables Publisher for the setpoint.
   */
  private DoublePublisher      tunableSetpointPublisher;
  /**
   * Network Tables Subscriber for the setpoint.
   */
  private DoubleSubscriber tunableSetpointSubscriber;
  /**
   * Network Tables publisher for the position of the mechanism.
   */
  private DoublePublisher      positionPublisher;
  /**
   * Motor controller for the mechanism.
   */
  private SmartMotorController motorController;

  /**
   * Setup telemetry for the Mechanism and motor controller.
   *
   * @param mechanismTelemetryName Mechanism Telemetry Name.
   * @param motorController        {@link SmartMotorController} to setup telemetry for.
   * @param units                  Units of the Mechanism.
   * @param position               Position of the mechanism.
   * @param setpoint               Setpoint of the mechanism.
   */
  public void setupTelemetry(String mechanismTelemetryName, SmartMotorController motorController, String units,
                             Measure<?> setpoint, Measure<?> position)
  {
    tuningNetworkTable = NetworkTableInstance.getDefault().getTable("Tuning")
                                             .getSubTable(mechanismTelemetryName);
    networkTable = NetworkTableInstance.getDefault().getTable("Mechanisms")
                                       .getSubTable(mechanismTelemetryName);
    this.motorController = motorController;
    unitsString = units;

    var tunableSetpointTopic = tuningNetworkTable.getDoubleTopic("Setpoint");
    tunableSetpointPublisher = tunableSetpointTopic.publish();
    tunableSetpointSubscriber = tunableSetpointTopic.subscribe(convertToNativeUnit(position),
                                                               PubSubOption.keepDuplicates(true),
                                                               PubSubOption.pollStorage(10));
    unitsPublisher = networkTable.getStringTopic("Units").publish();
    positionPublisher = networkTable.getDoubleTopic("Position").publish();

    this.setpoint = convertToNativeUnit(setpoint);

    this.unitsPublisher.set(units);
    this.tunableSetpointPublisher.set(this.setpoint);
    this.positionPublisher.set(convertToNativeUnit(position));
    var motorName = motorController.getConfig().getTelemetryName().orElse("motor");
    motorController.setupTelemetry(networkTable.getSubTable(motorName),
            tuningNetworkTable.getSubTable(motorName));
  }

  /**
   * Convert the given unit to the telemetry type.
   *
   * @param unit Measurable unit like {@link Units#Meters} or {@link Units#Radians}
   * @return double representation of the measurable unit for telemetry.
   */
  private double convertToNativeUnit(Measure<?> unit)
  {
      return switch (unitsString) {
          case "Degrees" -> ((Angle)unit).in(Degrees);
          case "Radians" -> ((Angle)unit).in(Units.Radians);
          case "Feet" -> ((Distance)unit).in(Feet);
          case "Meters" -> ((Distance)unit).in(Units.Meters);
          default -> throw new IllegalArgumentException(
                  "Cannot convert " + unit.toLongString() + " to double! Invalid unit given to mechanism telemetry!");
      };
  }

  /**
   * Convert native units to units type expected.
   *
   * @param unit Native unit from telemetry.
   * @return Unit representation.
   */
  private Measure<?> convertFromNativeUnit(double unit)
  {
      return switch (unitsString) {
          case "Degrees" -> Degrees.of(unit);
          case "Radians" -> Units.Radians.of(unit);
          case "Feet" -> Feet.of(unit);
          case "Meters" -> Units.Meters.of(unit);
          default -> throw new IllegalArgumentException(
                  "Cannot convert " + unit + " to " + unitsString + "! Invalid unit given to mechanism telemetry!");
      };
  }

  /**
   * Checks if the setpoint in NetworkTables is different from the setpoint in the class, if it is then the setpoint in
   * NT has been updated and should be used instead of the requested setpoint.
   *
   * @return
   */
  public boolean setpointChanged()
  {
    return tunableSetpointSubscriber.get(setpoint) != setpoint;
  }

  /**
   * Get the setpoint from the tunable setpoint.
   *
   * @return Tunable setpoint.
   */
  public Measure<?> getSetpoint()
  {
    return convertFromNativeUnit(tunableSetpointSubscriber.get(setpoint));
  }

  /**
   * Update the position of the mechanism.
   *
   * @param position Position of the mechanism..
   */
  public void updatePosition(Measure<?> position)
  {
    positionPublisher.set(convertToNativeUnit(position));
  }


  /**
   * Update the setpoint telemetry for the mechanism.
   *
   * @param setpoint Setpoint of the Mechanism.
   */
  public void updateSetpoint(Measure<?> setpoint)
  {
    this.setpoint = convertToNativeUnit(setpoint);
    tunableSetpointPublisher.set(this.setpoint);
  }

  /**
   * Update the units of the telemetry to display.
   *
   * @param units Unit to display, valid options are "Meters", "Feet", "Degrees", "Radians".
   */
  public void updateUnits(String units)
  {
    if (!List.of("Meters", "Feet", "Degrees", "Radians").contains(units))
    {
      throw new IllegalArgumentException("Invalid units given to mechanism telemetry!");
    }
    this.unitsString = units;
    unitsPublisher.set(units);
  }
}
