// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.helpers;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.util.CANPorts;
import java.util.concurrent.atomic.AtomicInteger;
import org.wpilib.hardware.bus.CANPort;

/**
 * Centralised factory for test motor controller devices.
 *
 * <p>Each device type (SparkMax, SparkFlex, TalonFX, TalonFXS) maintains its own monotonic
 * counter so that every call produces a device with an ID unique within that type. IDs are
 * shared across device types and vendors a SparkMax(3) and a TalonFX(3) are allowed to
 * coexist. The counter is a single JVM-wide counter shared across every test class in the
 * suite (not reset per class), so {@value #MAX_REV_ID} is set well above the total number of
 * devices the whole suite creates in one run if it wraps mid-suite, a later test can reuse
 * the CAN ID of an earlier test's still-registered simulated device, corrupting that test's
 * results in a way that only reproduces when the full suite runs (not in isolation).
 */
public class DeviceCreator {
  private static final int MAX_REV_ID = 10000;
  private static final int MAX_CTRE_ID = 61;

  private static final AtomicInteger revId = new AtomicInteger(1);

  private static final AtomicInteger ctreId = new AtomicInteger(1);

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

  public static TalonFX createTalonFX() {
    int id = ctreId.getAndIncrement();
    if (id > MAX_CTRE_ID) {
      ctreId.setRelease(1);
      System.err.println("Warning: used maximum device IDs, resetting to 0");
    }
    return new TalonFX(id, new CANBus(CANPort.CAN_S0));
  }

  public static TalonFXS createTalonFXS() {
    int id = ctreId.getAndIncrement();
    if (id > MAX_CTRE_ID) {
      ctreId.setRelease(1);
      System.err.println("Warning: used maximum device IDs, resetting to 0");
    }
    return new TalonFXS(id, new CANBus(CANPort.CAN_S0));
  }

  // Creates a CANcoder sharing the same CAN ID as the given TalonFX.
  public static CANcoder createCANcoderFor(TalonFX talon) {
    return new CANcoder(talon.getDeviceID(), new CANBus(CANPort.CAN_S0));
  }
}
