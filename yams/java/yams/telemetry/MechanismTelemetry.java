package yams.telemetry;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
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
   * Loop time publisher.
   */
  private Optional<DoublePublisher> loopTimePublisher = Optional.empty();
  /**
   * Loop time timer.
   */
  private double prevTimestamp = 0;

  /**
   * Setup loop time publisher.
   */
  public void setupLoopTime()
  {
    var loopTimePublisherTopic = networkTable.getDoubleTopic("loopTime");
    loopTimePublisherTopic.setProperties("{\"unit\":\"second\"}");
    loopTimePublisher = Optional.of(loopTimePublisherTopic.publish());
  }

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
    setupLoopTime();
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
    setupLoopTime();
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

  /**
   * Update the loop time.
   */
  public void updateLoopTime()
  {
    loopTimePublisher.ifPresent(publisher -> {
      if (prevTimestamp != 0)
      {
        publisher.set(Timer.getFPGATimestamp() - prevTimestamp);
      }
      prevTimestamp = Timer.getFPGATimestamp();
    });
  }
}
