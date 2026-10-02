// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.helpers;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.revrobotics.encoder.SplineEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.util.CANPorts;
import java.util.concurrent.atomic.AtomicInteger;
import org.wpilib.hardware.bus.CANPort;
import yams.core.motorcontrollers.SmartMotorController;

/**
 * Centralised factory for test motor controller devices.
 *
 * <p>REV devices share one JVM-wide counter (not reset per class), and {@value #MAX_REV_ID} is
 * set well above the total number of devices the whole suite creates in one run. CTRE devices share
 * another, and once every ID on a simulated CAN bus is used they move on to the next bus. A later
 * device that reused an earlier one's bus and ID would start from the simulated state that device
 * left behind, corrupting its test in a way that only reproduces when the full suite runs (not in
 * isolation). A CANcoder or CANdi made for a Talon shares its ID and bus.
 */
public class DeviceCreator {
  private static final int MAX_REV_ID = 10000;
  private static final int MAX_CTRE_ID = 61;

  private static final AtomicInteger revId = new AtomicInteger(1);

  private static final int MAX_DETACHED_ID = 62;

  private static final AtomicInteger detachedId = new AtomicInteger(0);

  private static final AtomicInteger ctreId = new AtomicInteger(0);

  /** Simulated CAN buses CTRE devices are spread over. */
  private static final CANPort[] kCtreBuses = {CANPort.CAN_S0, CANPort.CAN_S1, CANPort.CAN_S2, CANPort.CAN_S3, CANPort.CAN_S4};

  public static SparkMax createSparkMax() {
    int id = revId.getAndIncrement();
    if (id > MAX_REV_ID) {
      revId.setRelease(1);
      System.err.println("Warning: used maximum device IDs, resetting to 0");
    }
    return new SparkMax(CANPorts.fromBusId(1), id, MotorType.kBrushless);
  }

  public static SparkFlex createSparkFlex() {
    int id = revId.getAndIncrement();
    if (id > MAX_REV_ID) {
      revId.setRelease(1);
      System.err.println("Warning: used maximum device IDs, resetting to 0");
    }
    return new SparkFlex(CANPorts.fromBusId(1), id, MotorType.kBrushless);
  }

  // Creates an encoder a SPARK reads over CAN. REVLib's only detached encoder model is the MAXSpline
  // Encoder.
  // Detached encoders take CAN IDs 1 to 62 and are closed by the test that creates them, so their IDs
  // are reused.
  public static SplineEncoder createDetachedEncoder() {
    return new SplineEncoder(CANPorts.fromBusId(1), detachedId.getAndIncrement() % MAX_DETACHED_ID + 1);
  }

  public static TalonFX createTalonFX() {
    final int device = ctreId.getAndIncrement();
    return fresh(new TalonFX(ctreDeviceId(device), ctreBus(device)));
  }

  public static TalonFXS createTalonFXS() {
    final int device = ctreId.getAndIncrement();
    return fresh(new TalonFXS(ctreDeviceId(device), ctreBus(device)));
  }

  /**
   * Start a CTRE device from its default status signal frequencies, in case an earlier device with
   * its bus and ID was {@link #silence(Object) silenced}.
   */
  private static <T extends ParentDevice> T fresh(T device) {
    device.resetSignalFrequencies();
    return device;
  }

  /** CAN ID of the {@code device}th CTRE device. */
  private static int ctreDeviceId(int device) {
    return device % MAX_CTRE_ID + 1;
  }

  /**
   * CAN bus of the {@code device}th CTRE device: every bus's IDs are used before the next bus, so
   * no two devices in a run share a bus and ID until all of them are used. A simulated device keeps
   * its state for the rest of the run, and a later device with its bus and ID would start from it.
   */
  private static CANBus ctreBus(int device) {
    final int bus = device / MAX_CTRE_ID % kCtreBuses.length;
    if (device > 0 && device % (MAX_CTRE_ID * kCtreBuses.length) == 0) {
      System.err.println("Warning: used every CTRE device ID on every bus, reusing them");
    }
    return new CANBus(kCtreBuses[bus]);
  }

  // Creates a CANcoder sharing the same CAN ID as the given TalonFX.
  public static CANcoder createCANcoderFor(TalonFX talon) {
    return fresh(new CANcoder(talon.getDeviceID(), talon.getNetwork()));
  }

  // Creates a CANcoder sharing the same CAN ID as the given TalonFXS.
  public static CANcoder createCANcoderFor(TalonFXS talon) {
    return fresh(new CANcoder(talon.getDeviceID(), talon.getNetwork()));
  }

  // Creates a CANdi sharing the same CAN ID as the given TalonFX.
  public static CANdi createCANdiFor(TalonFX talon) {
    return fresh(new CANdi(talon.getDeviceID(), talon.getNetwork()));
  }

  // Creates a CANdi sharing the same CAN ID as the given TalonFXS.
  public static CANdi createCANdiFor(TalonFXS talon) {
    return fresh(new CANdi(talon.getDeviceID(), talon.getNetwork()));
  }

  /**
   * Stop a closed motor controller's CTRE devices, the Talon and its CANcoder or CANdi, from sending
   * status signals. A simulated device keeps running after it is closed, and the signals of every
   * device a run has closed fill the simulated CAN bus until a Talon's remote sensor data arrives too
   * late and is reported invalid.
   *
   * @param smc Closed motor controller.
   */
  public static void silence(SmartMotorController smc) {
    silence(smc.getMotorController());
    smc.getConfig().getExternalEncoder().ifPresent(DeviceCreator::silence);
  }

  /**
   * Stop a closed CTRE device from sending status signals; see {@link #silence(SmartMotorController)}.
   *
   * @param device Closed device. Devices of other vendors are left as they are.
   */
  public static void silence(Object device) {
    if (device instanceof ParentDevice ctreDevice) {
      // Bus utilization optimization keeps the signals given an update frequency, as YAMS gives the
      // signals it reads in simulation, so forget their frequencies first.
      ctreDevice.resetSignalFrequencies();
      ctreDevice.optimizeBusUtilization(0);
    }
  }
}
