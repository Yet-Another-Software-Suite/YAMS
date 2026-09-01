// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.telemetry;

import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.datalog.DoubleLogEntry;
import org.wpilib.datalog.StructArrayLogEntry;
import org.wpilib.datalog.StructLogEntry;
import org.wpilib.util.struct.Struct;
import org.wpilib.system.DataLogManager;
import org.wpilib.system.Timer;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import yams.motorcontrollers.SmartMotorController;

/**
 * Mechanism telemetry.
 *
 * <p>Publishes mechanism state — setpoint position/velocity, current mechanism position, loop
 * time, and motor controller data — to NetworkTables under the {@code Mechanisms/<name>} and
 * {@code Tuning/<name>} tables. Data is visible in SmartDashboard, Elastic, and Advantage Scope.
 *
 * <p>This class is managed internally by mechanism classes such as {@link yams.mechanisms.positional.Arm},
 * {@link yams.mechanisms.positional.Elevator}, and {@link yams.mechanisms.velocity.FlyWheel}.
 * You do <b>not</b> usually construct it directly; instead enable telemetry through the
 * mechanism's configuration object before constructing the mechanism:
 *
 * <h2>Enabling via ArmConfig</h2>
 * <pre>{@code
 * ArmConfig armConfig = new ArmConfig()
 *     .withMotor(new TalonFXConfig(1))
 *     .withLength(Meters.of(0.5))
 *     .withMass(Kilograms.of(2.0))
 *     .withHardLimits(Degrees.of(-10), Degrees.of(90))
 *     .withTelemetry("Arm", TelemetryVerbosity.HIGH);  // enables MechanismTelemetry
 *
 * Arm arm = new Arm(armConfig);
 * }</pre>
 *
 * <h2>Enabling via ElevatorConfig</h2>
 * <pre>{@code
 * ElevatorConfig elevatorConfig = new ElevatorConfig()
 *     .withMotor(new TalonFXConfig(2))
 *     .withDrumRadius(Inches.of(1.0))
 *     .withMass(Kilograms.of(4.0))
 *     .withTelemetry("Elevator", TelemetryVerbosity.HIGH);  // enables MechanismTelemetry
 *
 * Elevator elevator = new Elevator(elevatorConfig);
 * }</pre>
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
   * DataLog entry name prefix for this mechanism's fields, if configured via {@link #setupTelemetry(String, String)}.
   * Must be set before {@link #publishDouble(String, String)}, {@link #publishStruct(String, Struct)}, or
   * {@link #publishStructArray(String, Struct)} are called, since each of those decides once, at the time it's
   * called, whether to also create a DataLog entry.
   */
  private Optional<String> dataLogName = Optional.empty();

  /**
   * Setup loop time publisher.
   */
  public void setupLoopTime()
  {
    var loopTimePublisherTopic = networkTable.getDoubleTopic("loopTime");
    loopTimePublisherTopic.setProperties("{\"units\": \"second\"}");
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
   * Setup telemetry for the Mechanism, additionally logging every field published through
   * {@link #publishDouble(String, String)}, {@link #publishStruct(String, Struct)}, and
   * {@link #publishStructArray(String, Struct)} to a WPILib DataLog under the given name.
   *
   * @param mechanismTelemetryName Mechanism Telemetry Name.
   * @param dataLogName            DataLog entry name prefix for this mechanism's fields.
   */
  public void setupTelemetry(String mechanismTelemetryName, String dataLogName)
  {
    setupTelemetry(mechanismTelemetryName);
    this.dataLogName = Optional.ofNullable(dataLogName);
  }

  /**
   * Wire an additional {@link SmartMotorController}'s telemetry into a named child table of this mechanism, without
   * disturbing the mechanism's own data/tuning table references or loop-time publisher.
   *
   * <p>Use this for mechanisms driven by more than one {@link SmartMotorController} (e.g. a swerve module's drive and
   * azimuth motors, or a differential mechanism's left and right motors) so that every motor shares the single
   * {@link #setupTelemetry(String)} call and loop timer for the mechanism, instead of each motor re-running
   * {@link #setupTelemetry(String, SmartMotorController)} and clobbering the previous motor's table/loop-time state.
   *
   * @param subTableName    Name of the child table to nest this motor's telemetry under, relative to the mechanism's
   *                        data and tuning tables.
   * @param motorController {@link SmartMotorController} to set up telemetry for.
   */
  public void addMotorController(String subTableName, SmartMotorController motorController)
  {
    motorController.setupTelemetry(networkTable.getSubTable(subTableName), tuningNetworkTable.getSubTable(subTableName));
  }

  /**
   * Publish a mechanism-level {@code double} field under this mechanism's data table, and — if this mechanism was
   * set up with a DataLog name via {@link #setupTelemetry(String, String)} — to the DataLog as well. For fields tied
   * to a {@link SmartMotorController} use {@link #setupTelemetry(String, SmartMotorController)} or
   * {@link #addMotorController(String, SmartMotorController)} instead.
   *
   * @param key  NetworkTables key, relative to this mechanism's data table.
   * @param unit Unit metadata for the field (consumed by Advantage Scope/Elastic), or {@code null} for none.
   * @return {@link DoubleConsumer} to push values to.
   */
  public DoubleConsumer publishDouble(String key, String unit)
  {
    var topic = networkTable.getDoubleTopic(key);
    if (unit != null)
    {
      topic.setProperties("{\"units\": \"" + unit + "\"}");
    }
    DoublePublisher publisher = topic.publish();
    Optional<DoubleLogEntry> logEntry = dataLogName.map(
        prefix -> new DoubleLogEntry(DataLogManager.getLog(), prefix + "/" + key, (long) Timer.getTimestamp()));
    return value -> {
      publisher.accept(value);
      logEntry.ifPresent(entry -> entry.append(value, (long) Timer.getTimestamp()));
    };
  }

  /**
   * Publish a mechanism-level struct field under this mechanism's data table, and — if this mechanism was set up
   * with a DataLog name via {@link #setupTelemetry(String, String)} — to the DataLog as well.
   *
   * @param key    NetworkTables key, relative to this mechanism's data table.
   * @param struct {@link Struct} schema for the published type.
   * @param <T>    Type of the published value.
   * @return {@link Consumer} to push values to.
   */
  public <T> Consumer<T> publishStruct(String key, Struct<T> struct)
  {
    StructPublisher<T> publisher = networkTable.getStructTopic(key, struct).publish();
    Optional<StructLogEntry<T>> logEntry = dataLogName.map(
        prefix -> StructLogEntry.create(DataLogManager.getLog(), prefix + "/" + key, struct));
    return value -> {
      publisher.accept(value);
      logEntry.ifPresent(entry -> entry.append(value));
    };
  }

  /**
   * Publish a mechanism-level struct array field under this mechanism's data table, and — if this mechanism was set
   * up with a DataLog name via {@link #setupTelemetry(String, String)} — to the DataLog as well.
   *
   * @param key    NetworkTables key, relative to this mechanism's data table.
   * @param struct {@link Struct} schema for the published element type.
   * @param <T>    Type of the published array elements.
   * @return {@link Consumer} to push values to.
   */
  public <T> Consumer<T[]> publishStructArray(String key, Struct<T> struct)
  {
    StructArrayPublisher<T> publisher = networkTable.getStructArrayTopic(key, struct).publish();
    Optional<StructArrayLogEntry<T>> logEntry = dataLogName.map(
        prefix -> StructArrayLogEntry.create(DataLogManager.getLog(), prefix + "/" + key, struct));
    return value -> {
      publisher.accept(value);
      logEntry.ifPresent(entry -> entry.append(value));
    };
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
        publisher.set(Timer.getTimestamp() - prevTimestamp);
      }
      prevTimestamp = Timer.getTimestamp();
    });
  }
}
