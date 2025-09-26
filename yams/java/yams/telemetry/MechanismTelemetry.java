package yams.telemetry;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import yams.motorcontrollers.SmartMotorController;

/**
 * Mechanism telemetry.
 */
public class MechanismTelemetry
{

  /**
   * Telemetry NetworkTable.
   */
  private NetworkTable networkTable;
  /**
   * Tuning NetworkTable.
   */
  private NetworkTable tuningNetworkTable;

  /**
   * Setup telemetry for the Mechanism and motor controller.
   *
   * @param mechanismTelemetryName Mechanism Telemetry Name.
   * @param motorController        {@link SmartMotorController} to setup telemetry for.
   */
  public void setupTelemetry(String mechanismTelemetryName, SmartMotorController motorController)
  {
    tuningNetworkTable = NetworkTableInstance.getDefault().getTable("Tuning")
                                             .getSubTable(mechanismTelemetryName);
    networkTable = NetworkTableInstance.getDefault().getTable("Mechanisms")
                                       .getSubTable(mechanismTelemetryName);
    motorController.setupTelemetry(networkTable, tuningNetworkTable);
  }

  /**
   * Setup telemetry for the Mechanism and motor controller.
   *
   * @param mechanismTelemetryName Mechanism Telemetry Name.
   */
  public void setupTelemetry(String mechanismTelemetryName)
  {
    tuningNetworkTable = NetworkTableInstance.getDefault().getTable("Tuning")
                                             .getSubTable(mechanismTelemetryName);
    networkTable = NetworkTableInstance.getDefault().getTable("Mechanisms")
                                       .getSubTable(mechanismTelemetryName);
  }

  /**
   * Get the telemetry NetworkTable.
   *
   * @return Telemetry NetworkTable.
   */
  public NetworkTable getDataTable()
  {
    return networkTable;
  }

  /**
   * Get the tuning NetworkTable.
   *
   * @return Tuning NetworkTable.
   */
  public NetworkTable getTuningTable()
  {
    return tuningNetworkTable;
  }
}
