// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot;

import com.ctre.phoenix6.CANBus;
import org.wpilib.hardware.bus.CANPort;

/** CAN IDs and ports, from the original's {@code HWMap} and YAGSL module files. */
public final class Ports {
    // Systemcore has no "rio" bus; its first onboard CAN port takes that role.
    public static final CANPort kCANBus = CANPort.CAN_S0;
    // The same bus, for CTRE devices.
    public static final CANBus kCTRECANBus = new CANBus(kCANBus);

    // Swerve: SPARK MAX drive and angle motors, Thrifty absolute encoders on analog inputs.
    public static final int kFrontLeftDrive = 1;
    public static final int kFrontLeftAngle = 2;
    public static final int kFrontLeftEncoder = 3;

    public static final int kFrontRightDrive = 4;
    public static final int kFrontRightAngle = 5;
    public static final int kFrontRightEncoder = 0;

    public static final int kBackLeftDrive = 7;
    public static final int kBackLeftAngle = 8;
    public static final int kBackLeftEncoder = 2;

    public static final int kBackRightDrive = 10;
    public static final int kBackRightAngle = 11;
    public static final int kBackRightEncoder = 1;

    // Pigeon 2, in place of the original's Redux Canandgyro.
    public static final int kPigeon = 25;

    public static final int kElevatorLeft = 13;
    public static final int kElevatorRight = 14;
    public static final int kCoralArm = 15;
    public static final int kAlgaeArm = 16;
    public static final int kCoralWrist = 17;
    public static final int kCoralRoller = 18;
    public static final int kAlgaeRoller = 19;

    // LaserCAN IDs, for when a 2027 LaserCAN library exists (see util.DistanceSensor).
    public static final int kCoralLaserCan = 20;
    public static final int kAlgaeLaserCan = 21;
}
