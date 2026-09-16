// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.helpers;

/* CTRE has not published a Phoenix6 build compatible with wpilib 2027-alpha-7; re-enable once
   com.ctre.phoenix6 is available again.
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
*/
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.util.CANPorts;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Centralised factory for test motor controller devices.
 *
 * <p>Each device type (SparkMax, SparkFlex, TalonFX, TalonFXS) maintains its own monotonic
 * counter so that every call produces a device with an ID unique within that type. IDs are
 * shared across device types and vendors — a SparkMax(3) and a TalonFX(3) are allowed to
 * coexist. IDs are capped at {@value #MAX_REV_ID} to stay within the CAN bus limit.
 *
 * <p>CTRE device factories (TalonFX, TalonFXS, CANcoder) are commented out until CTRE publishes
 * a Phoenix6 build compatible with wpilib 2027-alpha-7.
 */
public class DeviceCreator
{
  private static final int MAX_REV_ID = 85;
  // private static final int MAX_CTRE_ID = 61;

  private static final AtomicInteger revId = new AtomicInteger(1);
  // private static final AtomicInteger ctreId = new AtomicInteger(1);

  public static SparkMax createSparkMax()
  {
    int id = revId.getAndIncrement();
    if (id > MAX_REV_ID)
    {
        revId.setRelease(1);
        System.err.println("Warning: used maximum device IDs, resetting to 0");
    }
    return new SparkMax(CANPorts.fromBusId(1), id, MotorType.kBrushless);
  }

  public static SparkFlex createSparkFlex()
  {
    int id = revId.getAndIncrement();
    if (id > MAX_REV_ID)
    {
        revId.setRelease(1);
        System.err.println("Warning: used maximum device IDs, resetting to 0");
    }
    return new SparkFlex(CANPorts.fromBusId(1), id, MotorType.kBrushless);
  }

  /* CTRE has not published a Phoenix6 build compatible with wpilib 2027-alpha-7.
  public static TalonFX createTalonFX()
  {
    int id = ctreId.getAndIncrement();
    if (id > MAX_CTRE_ID)
    {
        ctreId.setRelease(1);
        System.err.println("Warning: used maximum device IDs, resetting to 0");
    }
    return new TalonFX(id, CANBus.systemcore(1));
  }

  public static TalonFXS createTalonFXS()
  {
    int id = ctreId.getAndIncrement();
    if (id > MAX_CTRE_ID)
    {
        ctreId.setRelease(1);
        System.err.println("Warning: used maximum device IDs, resetting to 0");
    }
    return new TalonFXS(id, CANBus.systemcore(1));
  }

  // Creates a CANcoder sharing the same CAN ID as the given TalonFX.
  public static CANcoder createCANcoderFor(TalonFX talon)
  {
    return new CANcoder(talon.getDeviceID(), CANBus.systemcore(1));
  }
  */
}
