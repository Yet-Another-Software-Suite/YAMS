// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.helpers;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Centralised factory for test motor controller devices.
 *
 * <p>Each device type (SparkMax, SparkFlex, TalonFX, TalonFXS) maintains its own monotonic
 * counter so that every call produces a device with an ID unique within that type. IDs are
 * shared across device types and vendors — a SparkMax(3) and a TalonFX(3) are allowed to
 * coexist. IDs are capped at {@value #MAX_ID} to stay within the CAN bus limit.
 */
public class DeviceCreator
{
  private static final int MAX_REV_ID = 85;
  private static final int MAX_CTRE_ID = 61;

  private static final AtomicInteger revId = new AtomicInteger(1);
  private static final AtomicInteger ctreId   = new AtomicInteger(1);

  public static SparkMax createSparkMax()
  {
    int id = revId.getAndIncrement();
    if (id > MAX_REV_ID)
    {
        revId.setRelease(1);
        System.err.println("Warning: used maximum device IDs, resetting to 0");
    }
    return new SparkMax(id, MotorType.kBrushless);
  }

  public static SparkFlex createSparkFlex()
  {
    int id = revId.getAndIncrement();
    if (id > MAX_REV_ID)
    {
        revId.setRelease(1);
        System.err.println("Warning: used maximum device IDs, resetting to 0");
    }
    return new SparkFlex(id, MotorType.kBrushless);
  }

  public static TalonFX createTalonFX()
  {
    int id = ctreId.getAndIncrement();
    if (id > MAX_CTRE_ID)
    {
        ctreId.setRelease(1);
        System.err.println("Warning: used maximum device IDs, resetting to 0");
    }
    return new TalonFX(id);
  }

  public static TalonFXS createTalonFXS()
  {
    int id = ctreId.getAndIncrement();
    if (id > MAX_CTRE_ID)
    {
        ctreId.setRelease(1);
        System.err.println("Warning: used maximum device IDs, resetting to 0");
    }
    return new TalonFXS(id);
  }

  /** Creates a CANcoder sharing the same CAN ID as the given TalonFX. */
  public static CANcoder createCANcoderFor(TalonFX talon)
  {
    return new CANcoder(talon.getDeviceID());
  }
}
